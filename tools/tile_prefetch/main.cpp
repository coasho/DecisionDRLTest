// tile_prefetch: fill the shared tile cache for a region ahead of time
// (design 7.3 "offline pyramids"), so training machines without network
// access still get terrain physics (WorldOptions::terrain) and the viewer
// still draws imagery and relief there. Downloads go to the same
// <cache>/<host>/<path> layout io::TerrainTiles and the viewer read from
// (%LOCALAPPDATA%\flightsim\tilecache by default), skipping files already
// present, so it is safe to re-run and to copy the directory to another
// machine.
//
//   tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 [--min-level 0] [--max-level 15]
//                 [--elevation-only] [--imagery-only] [--cache <dir>] [--threads 8]
//                 [--elevation-url <tmpl>] [--imagery-url <tmpl>]
//
// The whole pyramid from --min-level to --max-level is fetched within the
// radius (levels 0-7 are a handful of tiles; level 15 at 30 km is ~1500
// tiles per layer, ~150 MB). Physics uses level 12 (WorldOptions::terrainZoom)
// and the viewer's camera collision level 14.

#include "core/Log.h"
#include "io/TerrainTiles.h"
#include "platform/Http.h"
#include "platform/Paths.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr const char* kImageryUrl = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}.jpg";

std::string expand(std::string tmpl, unsigned z, unsigned x, unsigned y) {
    auto replace = [&](const char* key, unsigned v) {
        for (auto p = tmpl.find(key); p != std::string::npos; p = tmpl.find(key)) tmpl.replace(p, 3, std::to_string(v));
    };
    replace("{z}", z);
    replace("{x}", x);
    replace("{y}", y);
    return tmpl;
}

std::filesystem::path cachePathOf(const std::filesystem::path& cache, const std::string& url) {
    std::string u = url;
    const auto scheme = u.find("://");
    if (scheme != std::string::npos) u = u.substr(scheme + 3);
    return cache / std::filesystem::path(u);
}

struct Job {
    std::string url;
    std::filesystem::path path;
};

} // namespace

int main(int argc, char** argv) {
    double lat = 37.62, lon = -122.4, radiusKm = 20.0;
    unsigned minLevel = 0, maxLevel = 15, threads = 8;
    bool elevation = true, imagery = true;
    std::filesystem::path cache;
    std::string elevationUrl = io::TerrainTiles::Options{}.urlTemplate, imageryUrl = kImageryUrl;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--lat") lat = std::atof(next());
        else if (k == "--lon") lon = std::atof(next());
        else if (k == "--radius-km") radiusKm = std::atof(next());
        else if (k == "--min-level") minLevel = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--max-level") maxLevel = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--threads") threads = static_cast<unsigned>(std::max(1, std::atoi(next())));
        else if (k == "--elevation-only") imagery = false;
        else if (k == "--imagery-only") elevation = false;
        else if (k == "--cache") cache = next();
        else if (k == "--elevation-url") elevationUrl = next();
        else if (k == "--imagery-url") imageryUrl = next();
        else {
            std::fprintf(stderr, "usage: tile_prefetch --lat L --lon L --radius-km R [--min-level 0] [--max-level 15] [--elevation-only] "
                                 "[--imagery-only] [--cache dir] [--threads 8] [--elevation-url tmpl] [--imagery-url tmpl]\n");
            return 2;
        }
    }
    if (cache.empty()) cache = platform::configDir() / "tilecache";
    maxLevel = std::min(maxLevel, 19u);

    // Every tile whose centre is within the radius, per level (plus the tile under the centre).
    std::vector<Job> jobs;
    std::size_t present = 0;
    for (unsigned z = minLevel; z <= maxLevel; ++z) {
        double tx, ty;
        io::mercatorTile(lat * kDeg, lon * kDeg, z, tx, ty);
        const int n = static_cast<int>(1u << z);
        const double metresPerTile = 2.0 * 3.14159265358979323846 * 6378137.0 * std::cos(lat * kDeg) / n;
        const int r = static_cast<int>(std::ceil(radiusKm * 1000.0 / metresPerTile));
        const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                const int y = cy + dy;
                if (y < 0 || y >= n) continue;
                if (std::hypot(dx, dy) > r + 0.5) continue;
                const unsigned x = static_cast<unsigned>(((cx + dx) % n + n) % n);
                for (const auto* tmpl : {elevation ? elevationUrl.c_str() : nullptr, imagery ? imageryUrl.c_str() : nullptr}) {
                    if (!tmpl) continue;
                    Job j{expand(tmpl, z, x, static_cast<unsigned>(y)), {}};
                    j.path = cachePathOf(cache, j.url);
                    if (std::filesystem::exists(j.path)) ++present;
                    else jobs.push_back(std::move(j));
                }
            }
    }
    std::printf("tile_prefetch: %.4f, %.4f  radius %.1f km  levels %u-%u  cache %s\n", lat, lon, radiusKm, minLevel, maxLevel, cache.string().c_str());
    std::printf("  %zu tile(s) already cached, %zu to download\n", present, jobs.size());
    if (jobs.empty()) return 0;

    std::atomic<std::size_t> next{0}, done{0}, failed{0}, bytes{0};
    const auto t0 = std::chrono::steady_clock::now();
    auto worker = [&] {
        std::vector<std::uint8_t> body;
        for (std::size_t i = next.fetch_add(1); i < jobs.size(); i = next.fetch_add(1)) {
            const Job& j = jobs[i];
            std::string error;
            bool ok = false;
            for (int attempt = 0; attempt < 3 && !ok; ++attempt) ok = platform::httpGet(j.url, body, &error);
            if (!ok) {
                LOG_WARN("tool") << j.url << ": " << error;
                failed.fetch_add(1);
                continue;
            }
            std::error_code ec;
            std::filesystem::create_directories(j.path.parent_path(), ec);
            const auto tmp = j.path.string() + ".part";
            {
                std::ofstream out(tmp, std::ios::binary);
                out.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
            }
            std::filesystem::rename(tmp, j.path, ec); // atomic: a reader never sees a half-written tile
            if (ec) failed.fetch_add(1);
            else {
                bytes.fetch_add(body.size());
                done.fetch_add(1);
            }
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::min<std::size_t>(threads, jobs.size()); ++t) pool.emplace_back(worker);
    std::size_t lastReport = 0;
    while (done.load() + failed.load() < jobs.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const std::size_t d = done.load() + failed.load();
        if (d - lastReport >= 50 || d == jobs.size()) {
            lastReport = d;
            std::printf("  %zu / %zu  (%.1f MB)\n", d, jobs.size(), static_cast<double>(bytes.load()) / 1e6);
            std::fflush(stdout);
        }
    }
    for (auto& t : pool) t.join();
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("done: %zu downloaded (%.1f MB), %zu failed, %.1f s\n", done.load(), static_cast<double>(bytes.load()) / 1e6, failed.load(), s);
    return failed.load() ? 1 : 0;
}
