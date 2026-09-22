#include "core/HeightGrid.h"
#include "io/TerrainTiles.h"

#include "core/Log.h"
#include "platform/Http.h"
#include "platform/Paths.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include <stb_image.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace fsim::io {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

void mercatorTile(double latRad, double lonRad, unsigned zoom, double& tx, double& ty) noexcept {
    const double n = static_cast<double>(1u << zoom);
    const double lat = std::clamp(latRad, -1.4844222297453322, 1.4844222297453322); // +-85.0511 deg
    tx = (lonRad / kPi + 1.0) * 0.5 * n;
    ty = (1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / kPi) * 0.5 * n;
    if (tx < 0.0) tx += n;
    if (tx >= n) tx -= n;
}

ElevationTile decodeTerrariumPng(const std::vector<std::uint8_t>& png) {
    ElevationTile tile;
    int w = 0, h = 0, channels = 0;
    unsigned char* pixels = stbi_load_from_memory(png.data(), static_cast<int>(png.size()), &w, &h, &channels, 3);
    if (!pixels || w < 2 || w != h) {
        if (pixels) stbi_image_free(pixels);
        return tile;
    }
    tile.size = static_cast<std::uint32_t>(w);
    tile.heights.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    for (std::size_t i = 0; i < tile.heights.size(); ++i) {
        const unsigned char* p = pixels + i * 3;
        // Sea floor clamped to 0: no water surface is drawn, and the viewer does the same.
        tile.heights[i] = std::max(0.0f, static_cast<float>(p[0]) * 256.0f + static_cast<float>(p[1]) + static_cast<float>(p[2]) / 256.0f - 32768.0f);
    }
    stbi_image_free(pixels);
    return tile;
}

TerrainTiles::TerrainTiles(Options options, Fetch fetch) : options_(std::move(options)), fetch_(std::move(fetch)) {
    options_.zoom = std::min(options_.zoom, 22u);
    options_.cacheTiles = std::max<std::size_t>(options_.cacheTiles, 8);
    if (options_.cacheDir.empty()) options_.cacheDir = platform::configDir() / "tilecache";
}

TerrainTiles::~TerrainTiles() {
    stopLoaders_.store(true);
    queueCv_.notify_all();
    for (auto& t : loaders_) t.join();
}

std::string TerrainTiles::url(unsigned x, unsigned y) const { return urlAt(options_.zoom, x, y); }

std::string TerrainTiles::urlAt(unsigned z, unsigned x, unsigned y) const {
    std::string s = options_.urlTemplate;
    auto sub = [&](const char* key, unsigned v) {
        for (auto pos = s.find(key); pos != std::string::npos; pos = s.find(key)) s.replace(pos, 3, std::to_string(v));
    };
    sub("{z}", z);
    sub("{x}", x);
    sub("{y}", y);
    return s;
}

std::filesystem::path TerrainTiles::cachePath(unsigned x, unsigned y) const { return cachePathAt(options_.zoom, x, y); }

std::filesystem::path TerrainTiles::cachePathAt(unsigned z, unsigned x, unsigned y) const {
    // Same layout as vsgXchange's file cache (<cache>/<host>/<path>) so the
    // viewer and a training application share downloads.
    std::string u = urlAt(z, x, y);
    const auto scheme = u.find("://");
    if (scheme != std::string::npos) u = u.substr(scheme + 3);
    return options_.cacheDir / std::filesystem::path(u);
}

TerrainTiles::Tile TerrainTiles::fromAncestor(unsigned x, unsigned y) const {
    for (unsigned d = 1; d <= options_.zoom; ++d) {
        const unsigned az = options_.zoom - d, ax = x >> d, ay = y >> d;
        std::ifstream in(cachePathAt(az, ax, ay), std::ios::binary);
        if (!in) continue;
        const std::vector<std::uint8_t> png{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
        const ElevationTile raw = decodeTerrariumPng(png);
        if (!raw.valid()) return nullptr;
        const unsigned mask = (1u << d) - 1u, qx = x & mask, qy = y & mask;
        const double share = 1.0 / static_cast<double>(1u << d);
        auto tile = std::make_shared<ElevationTile>();

        if (options_.meshDimension > 0 && raw.size > options_.meshDimension) {
            // The camera: the ancestor's *drawn* surface, restricted to this
            // tile. Where there is nothing deeper, the renderer stops refining
            // at the ancestor's level and draws exactly its vertex grid, so
            // interpolating between those vertices is what is on screen - not
            // the raster resampled afresh at this tile's size, which filters
            // over a smaller window and puts valley floors metres lower than
            // they are drawn.
            const std::uint32_t g = options_.meshDimension;
            const auto grid = core::resampleHeightGrid(raw.heights.data(), raw.size, g);
            tile->size = g;
            tile->heights.resize(static_cast<std::size_t>(g) * g);
            const double last = static_cast<double>(g - 1);
            for (std::uint32_t j = 0; j < g; ++j) {
                const double fy = std::clamp((static_cast<double>(qy) + static_cast<double>(j) / last) * share * last, 0.0, last);
                const auto y0 = static_cast<std::uint32_t>(std::min(std::floor(fy), last - 1.0));
                const double ty = fy - static_cast<double>(y0);
                for (std::uint32_t i = 0; i < g; ++i) {
                    const double fx = std::clamp((static_cast<double>(qx) + static_cast<double>(i) / last) * share * last, 0.0, last);
                    const auto x0 = static_cast<std::uint32_t>(std::min(std::floor(fx), last - 1.0));
                    const double tx = fx - static_cast<double>(x0);
                    auto at = [&](std::uint32_t a, std::uint32_t b) {
                        return static_cast<double>(grid[static_cast<std::size_t>(b) * g + a]);
                    };
                    const double h = (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) +
                                     (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
                    tile->heights[static_cast<std::size_t>(j) * g + i] = static_cast<float>(h);
                }
            }
        } else {
            // Physics: the ground where it is, at the ancestor's resolution -
            // this tile's share of its raster, texel centres as a real tile has.
            const std::uint32_t n = raw.size;
            tile->size = n;
            tile->heights.resize(static_cast<std::size_t>(n) * n);
            const double hi = static_cast<double>(n - 1);
            auto axis = [&](double s, std::uint32_t& a, double& t) {
                a = static_cast<std::uint32_t>(std::min(std::floor(std::clamp(s, 0.0, hi)), hi - 1.0));
                t = std::clamp(s - static_cast<double>(a), 0.0, 1.0);
            };
            for (std::uint32_t j = 0; j < n; ++j) {
                std::uint32_t y0;
                double ty;
                axis((static_cast<double>(qy) + (static_cast<double>(j) + 0.5) / n) * share * n - 0.5, y0, ty);
                for (std::uint32_t i = 0; i < n; ++i) {
                    std::uint32_t x0;
                    double tx;
                    axis((static_cast<double>(qx) + (static_cast<double>(i) + 0.5) / n) * share * n - 0.5, x0, tx);
                    auto at = [&](std::uint32_t a, std::uint32_t b) {
                        return static_cast<double>(raw.heights[static_cast<std::size_t>(b) * n + a]);
                    };
                    const double h = (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) +
                                     (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
                    tile->heights[static_cast<std::size_t>(j) * n + i] = static_cast<float>(h);
                }
            }
        }
        return tile;
    }
    return nullptr;
}

TerrainTiles::Tile TerrainTiles::load(unsigned x, unsigned y) const {
    std::vector<std::uint8_t> png;
    const auto path = cachePath(x, y);
    bool fromDisk = false;
    if (!fetch_) {
        std::ifstream in(path, std::ios::binary);
        if (in) {
            png.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
            fromDisk = !png.empty();
        }
    }
    if (!fromDisk && options_.offline) {
        // An offline set is partial: this level may be missing where a coarser
        // one is not. Answering "no ground" there left the camera with no
        // clearance and the focus at 0 m inside the Alps, whose elevation stops
        // at level 11 while the camera asks for 12.
        if (auto ancestor = fromAncestor(x, y)) return ancestor;
        failures_.fetch_add(1);
        return nullptr;
    }
    if (!fromDisk) {
        bool ok = false;
        if (fetch_) {
            ok = fetch_(options_.zoom, x, y, png);
        } else {
            std::string error;
            for (int attempt = 0; attempt < 3 && !ok; ++attempt) ok = platform::httpGet(url(x, y), png, &error);
            if (!ok) LOG_WARN("io") << "elevation tile " << url(x, y) << ": " << error;
        }
        if (!ok) {
            failures_.fetch_add(1);
            return nullptr;
        }
    }
    auto decoded = std::make_shared<ElevationTile>(decodeTerrariumPng(png));
    if (!decoded->valid()) {
        LOG_WARN("io") << "elevation tile " << url(x, y) << ": not a decodable PNG";
        failures_.fetch_add(1);
        return nullptr;
    }
    // Serve the surface the renderer draws, when asked. The mesh is built from
    // a box-filtered, resampled copy of this raster, and a box filter lowers
    // peaks and raises valley floors: over 40 tiles of real relief the drawn
    // mesh sat up to 11.3 m above the raw raster, more than the clearance the
    // camera keeps at close range, so the eye finished up inside the hillside.
    if (options_.meshDimension > 0 && decoded->size > options_.meshDimension) {
        auto resampled = core::resampleHeightGrid(decoded->heights.data(), decoded->size, options_.meshDimension);
        if (!resampled.empty()) {
            decoded->heights = std::move(resampled);
            decoded->size = options_.meshDimension;
        }
    }
    if (!fromDisk && !fetch_) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        std::ofstream out(path, std::ios::binary);
        if (out) out.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    }
    return decoded;
}

TerrainTiles::Tile TerrainTiles::tile(unsigned x, unsigned y) const {
    const Key key{x, y};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(key); it != cache_.end()) {
            lru_.splice(lru_.begin(), lru_, it->second.second);
            return it->second.first;
        }
    }
    Tile t = load(x, y); // outside the lock: other workers keep sampling cached tiles
    std::lock_guard<std::mutex> lock(mutex_);
    misses_.fetch_add(1);
    if (auto it = cache_.find(key); it != cache_.end()) return it->second.first; // another worker won the race
    lru_.push_front(key);
    cache_.emplace(key, std::make_pair(t, lru_.begin()));
    while (cache_.size() > options_.cacheTiles) {
        cache_.erase(lru_.back());
        lru_.pop_back();
    }
    return t;
}

double TerrainTiles::sample(const ElevationTile& t, double tx, double ty) noexcept {
    const double fx = (tx - std::floor(tx)) * static_cast<double>(t.size - 1);
    const double fy = (ty - std::floor(ty)) * static_cast<double>(t.size - 1);
    const std::uint32_t x0 = static_cast<std::uint32_t>(fx), y0 = static_cast<std::uint32_t>(fy);
    const std::uint32_t x1 = std::min(x0 + 1, t.size - 1), y1 = std::min(y0 + 1, t.size - 1);
    const double ax = fx - x0, ay = fy - y0;
    auto at = [&](std::uint32_t x, std::uint32_t y) { return static_cast<double>(t.heights[static_cast<std::size_t>(y) * t.size + x]); };
    return (at(x0, y0) * (1 - ax) + at(x1, y0) * ax) * (1 - ay) + (at(x0, y1) * (1 - ax) + at(x1, y1) * ax) * ay;
}

double TerrainTiles::heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double n = static_cast<double>(1u << options_.zoom);
    const Tile t = tile(static_cast<unsigned>(std::floor(tx)), static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0)));
    if (!t || !t->valid()) return 0.0;
    return sample(*t, tx, ty);
}

std::optional<double> TerrainTiles::cachedHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double n = static_cast<double>(1u << options_.zoom);
    const Key key{static_cast<unsigned>(std::floor(tx)), static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0))};
    Tile t;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(key); it != cache_.end()) {
            lru_.splice(lru_.begin(), lru_, it->second.second);
            t = it->second.first;
        }
    }
    if (!t) {
        requestAround(latitudeRad, longitudeRad);
        return std::nullopt;
    }
    if (!t->valid()) return 0.0;
    return sample(*t, tx, ty);
}

void TerrainTiles::prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const int n = static_cast<int>(1u << options_.zoom);
    const double metresPerTile = 2.0 * kPi * 6378137.0 * std::cos(latitudeRad) / n;
    const int r = static_cast<int>(std::ceil(radiusM / metresPerTile));
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    std::vector<Key> keys;
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const int y = cy + dy;
            if (y < 0 || y >= n) continue;
            keys.push_back(Key{static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)});
        }
    std::atomic<std::size_t> next{0};
    auto worker = [&] {
        for (std::size_t i = next.fetch_add(1); i < keys.size(); i = next.fetch_add(1)) tile(keys[i].x, keys[i].y);
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::max(1u, std::min<unsigned>(threads, static_cast<unsigned>(keys.size()))); ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();
    LOG_DEBUG("io") << "prefetched " << keys.size() << " elevation tiles at zoom " << options_.zoom << " (" << failures_.load() << " failures)";
}

void TerrainTiles::requestAround(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const int n = static_cast<int>(1u << options_.zoom);
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    bool any = false;
    {
        std::lock_guard<std::mutex> cacheLock(mutex_);
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                const int y = cy + dy;
                if (y < 0 || y >= n) continue;
                const Key k{static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)};
                const std::uint64_t id = (static_cast<std::uint64_t>(k.x) << 32) | k.y;
                if (cache_.count(k) || queued_.count(id)) continue;
                queued_.insert(id);
                queue_.push_back(k);
                any = true;
            }
    }
    if (any) {
        ensureLoaders();
        queueCv_.notify_all();
    }
}

void TerrainTiles::ensureLoaders() {
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (!loaders_.empty()) return;
    for (unsigned i = 0; i < std::max(1u, options_.loaderThreads); ++i) loaders_.emplace_back([this] { loaderLoop(); });
}

void TerrainTiles::loaderLoop() {
    for (;;) {
        Key k;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [&] { return stopLoaders_.load() || !queue_.empty(); });
            if (stopLoaders_.load()) return;
            k = queue_.front();
            queue_.pop_front();
        }
        tile(k.x, k.y);
        std::lock_guard<std::mutex> lock(queueMutex_);
        queued_.erase((static_cast<std::uint64_t>(k.x) << 32) | k.y);
    }
}

std::size_t TerrainTiles::cachedTiles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.size();
}

} // namespace fsim::io
