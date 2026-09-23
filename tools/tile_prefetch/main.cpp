// tile_prefetch: fill the tile cache ahead of time (design 7.3 "offline
// pyramids"), so machines without network access still get terrain physics
// (WorldOptions::terrain) and the viewer still draws imagery and relief.
// Downloads go to the same <cache>/<host>/<path> layout io::TerrainTiles and
// the viewer read from (%LOCALAPPDATA%\flightsim\tilecache by default),
// skipping files already present, so it is safe to re-run and to copy the
// directory to another machine. Elevation is stored smaller than it is served
// and still as Terrarium (storeElevationPng), so every reader is unaffected.
//
// A region is a circle, a lat/lon box, a corridor along a route, the whole
// globe, or all of its land, and each layer gets its own range of levels -
// which is the whole point, because a city wants imagery and a mountain wants
// elevation, and every level costs four times the one below it. A tapered
// region's levels are the equator's, stopping short toward the poles where
// Mercator tiles are already that sharp (taperedOut). --dry-run prices a plan
// without downloading a byte.
//
//   tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --imagery-levels 8-14
//   tile_prefetch --bbox 45.8,6.0,47.2,10.5 --elevation-levels 8-11
//   tile_prefetch --route 37.62,-122.4;34.05,-118.24 --width-km 40 --levels 7-10
//   tile_prefetch --global --levels 0-6 --dry-run
//   tile_prefetch --land --taper --levels 7-9
//   tile_prefetch --plan offline.json --dry-run
//
// The plan file is a list of regions, each with its own levels:
//
//   { "regions": [
//       { "name": "globe",  "type": "global", "imagery": [0,7], "elevation": [0,6] },
//       { "name": "Alps",   "type": "circle", "lat": 46.5, "lon": 8.0,
//         "radiusKm": 150, "elevation": [8,11] },
//       { "name": "SFO-LAX","type": "route",  "widthKm": 40, "imagery": [8,11],
//         "points": [[37.62,-122.4],[34.05,-118.24]] } ] }
//
// A layer left out of a region is not fetched for that region.

#include "core/Json.h"
#include "core/Log.h"
#include "io/TerrainTiles.h"
#include "platform/Http.h"
#include "platform/Paths.h"

#include <zlib.h>

#include <cstdlib>

// stb's own deflate writes fixed Huffman codes only; zlib at its best makes
// the same elevation tile a quarter smaller.
static unsigned char* fsimZlibCompress(unsigned char* data, int length, int* outLength, int /*quality*/) {
    uLongf size = compressBound(static_cast<uLong>(length));
    auto* out = static_cast<unsigned char*>(std::malloc(size));
    if (!out) return nullptr;
    if (compress2(out, &size, data, static_cast<uLong>(length), Z_BEST_COMPRESSION) != Z_OK) {
        std::free(out);
        return nullptr;
    }
    *outLength = static_cast<int>(size);
    return out;
}

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#define STBIW_ZLIB_COMPRESS fsimZlibCompress
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference" // inside stb, not ours
#endif
#include <stb_image_write.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace fsim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthR = 6378137.0;
constexpr const char* kImageryUrl =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}.jpg";

// Of Web Mercator area, land skewing north; used only where there is no land
// mask to ask.
constexpr double kLandFraction = 0.35;

enum class Layer { Elevation, Imagery };

struct Levels {
    int lo = -1, hi = -1;
    bool wanted() const noexcept { return lo >= 0 && hi >= lo; }
};

struct Region {
    enum class Kind { Circle, Box, Route, Global, Land };
    std::string name;
    Kind kind = Kind::Circle;
    double lat = 0.0, lon = 0.0, radiusKm = 0.0;
    double minLat = 0.0, minLon = 0.0, maxLat = 0.0, maxLon = 0.0;
    std::vector<std::pair<double, double>> points;
    double widthKm = 0.0;
    // How much wider than its nominal extent the region is fetched. `margin`
    // widens every level by a fraction of its size; `feather` adds a skirt
    // measured in *tiles at that level*, so detail tapers off instead of
    // ending at a wall.
    double margin = 0.15;
    double feather = 2.0;
    // Levels are the equator's; toward the poles each one stops where the
    // level above it already has the equator's ground resolution. See taperedOut().
    bool taper = false;
    Levels imagery, elevation;
};

/// z, x, y in one integer so a whole plan's tiles can be de-duplicated: a
/// route that passes through a city must not fetch the overlap twice.
std::uint64_t key(unsigned z, unsigned x, unsigned y) noexcept {
    return (static_cast<std::uint64_t>(z) << 58) | (static_cast<std::uint64_t>(x) << 29) | y;
}

std::string expand(std::string tmpl, unsigned z, unsigned x, unsigned y) {
    auto replace = [&](const char* k, unsigned v) {
        for (auto p = tmpl.find(k); p != std::string::npos; p = tmpl.find(k)) tmpl.replace(p, 3, std::to_string(v));
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
    bool elevation = false;
};

/// Which parts of the Earth are land, from the level-6 elevation tiles.
///
/// "Lower resolution for the oceans" needs a map of where the oceans are, and
/// the global base already contains one: every level-6 Terrarium tile, which
/// is above zero wherever there is land. A tile at any level is land if any
/// level-6 texel under it is. Measured this way, 44.7 % of level-7 tiles have
/// land in them and 38.7 % of level-8 - so a layer that skips the rest costs
/// about 40 % of the same layer laid over the whole planet.
struct LandMask {
    struct Tile {
        std::uint32_t n = 0;             ///< texels per edge; 0 = not on disk
        std::vector<std::uint8_t> land;  ///< n * n, 1 where above sea level
    };
    std::vector<Tile> tiles = std::vector<Tile>(64 * 64);
    std::size_t missing = 0;

    bool land(unsigned z, unsigned x, unsigned y) const {
        if (z < 6) {
            const unsigned d = 6 - z;
            for (unsigned yy = y << d; yy < ((y + 1) << d); ++yy)
                for (unsigned xx = x << d; xx < ((x + 1) << d); ++xx)
                    if (land(6, xx, yy)) return true;
            return false;
        }
        const unsigned d = z - 6, k = 1u << d;
        const Tile& t = tiles[(y >> d) * 64u + (x >> d)];
        if (t.n == 0) return true; // unknown: keep it rather than drop real land
        const std::uint32_t span = std::max<std::uint32_t>(1, t.n / k);
        const std::uint32_t x0 = (x & (k - 1)) * t.n / k, y0 = (y & (k - 1)) * t.n / k;
        for (std::uint32_t j = y0; j < std::min(t.n, y0 + span); ++j)
            for (std::uint32_t i = x0; i < std::min(t.n, x0 + span); ++i)
                if (t.land[static_cast<std::size_t>(j) * t.n + i]) return true;
        return false;
    }
};

const LandMask* g_land = nullptr;
std::uint32_t g_elevationSize = 256; ///< texels per edge elevation tiles are stored at

void appendBytes(void* context, void* data, int size) {
    auto* out = static_cast<std::vector<std::uint8_t>*>(context);
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    out->insert(out->end(), bytes, bytes + size);
}

/// An elevation tile in the form the package stores it, still Terrarium so
/// every reader is none the wiser, but three ways cheaper than it is served:
///
/// - No more than `size` texels per edge. The mesh never uses more than
///   64 x 64 of a tile, so 256 x 256 carries sixteen times what is drawn;
///   128 x 128 keeps twice what is drawn in each direction - enough that a
///   deeper tile made from it still gains something. Box-filtered, which is
///   also what the mesh does with it.
/// - Whole metres. Terrarium spends a third byte on 1/256 m, which after a box
///   filter is noise to a compressor; the DEMs underneath are good to metres,
///   and rounding costs half a metre at most. Half the bytes of a level-11
///   tile, a third of a level-7 one.
/// - zlib at its best rather than stb's fixed-code deflate: a quarter less.
///
/// The sea is stored at 0, as every reader already clamps it, which is also
/// what makes a mostly ocean tile nearly free.
bool storeElevationPng(std::vector<std::uint8_t>& png, std::uint32_t size) {
    const io::ElevationTile t = io::decodeTerrariumPng(png);
    if (!t.valid() || size == 0) return false;
    const std::uint32_t n = t.size > size && t.size % size == 0 ? size : t.size;
    const std::uint32_t f = t.size / n;
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(n) * n * 3);
    for (std::uint32_t j = 0; j < n; ++j)
        for (std::uint32_t i = 0; i < n; ++i) {
            double sum = 0.0;
            for (std::uint32_t b = 0; b < f; ++b)
                for (std::uint32_t a = 0; a < f; ++a)
                    sum += static_cast<double>(t.heights[static_cast<std::size_t>(j * f + b) * t.size + (i * f + a)]);
            const auto v = static_cast<std::uint32_t>(std::round(std::max(0.0, sum / static_cast<double>(f * f))) + 32768.0);
            std::uint8_t* px = &rgb[(static_cast<std::size_t>(j) * n + i) * 3];
            px[0] = static_cast<std::uint8_t>(std::min(255u, v / 256u));
            px[1] = static_cast<std::uint8_t>(v % 256u);
            px[2] = 0;
        }
    std::vector<std::uint8_t> out;
    if (!stbi_write_png_to_func(appendBytes, &out, static_cast<int>(n), static_cast<int>(n), 3, rgb.data(),
                                static_cast<int>(n * 3)))
        return false;
    png = std::move(out);
    return true;
}

/// Whether an elevation tile on disk is already in the stored form, from its
/// first hundred bytes: no wider than `size`, and deflated by zlib at its
/// best, which stamps the stream 78 DA - stb's deflate stamps 78 5E, and the
/// tiles as served are 256 wide.
bool inStoredForm(const std::filesystem::path& file, std::uint32_t size) {
    std::ifstream in(file, std::ios::binary);
    unsigned char h[128] = {};
    in.read(reinterpret_cast<char*>(h), sizeof(h));
    const auto got = static_cast<std::size_t>(in.gcount());
    if (got < 33 || h[1] != 'P' || h[2] != 'N' || h[3] != 'G') return false;
    const std::uint32_t width = (std::uint32_t(h[16]) << 24) | (std::uint32_t(h[17]) << 16) | (std::uint32_t(h[18]) << 8) | h[19];
    if (width > size) return false;
    for (std::size_t at = 8; at + 10 <= got;) { // walk the chunks to the first IDAT
        const std::uint32_t len = (std::uint32_t(h[at]) << 24) | (std::uint32_t(h[at + 1]) << 16) |
                                  (std::uint32_t(h[at + 2]) << 8) | h[at + 3];
        if (std::memcmp(h + at + 4, "IDAT", 4) == 0) return h[at + 8] == 0x78 && h[at + 9] == 0xDA;
        at += 12 + static_cast<std::size_t>(len);
    }
    return false;
}

bool writeAtomically(const std::filesystem::path& path, const std::vector<std::uint8_t>& body) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    const auto tmp = path.string() + ".part";
    {
        std::ofstream out(tmp, std::ios::binary);
        out.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
        if (!out) return false;
    }
    std::filesystem::rename(tmp, path, ec); // a reader never sees a half-written tile
    return !ec;
}

struct Fetched {
    std::size_t done = 0, failed = 0, bytes = 0;
};

/// Download jobs on a pool of threads, shrinking elevation as it arrives.
Fetched download(const std::vector<Job>& jobs, unsigned threads, const char* what) {
    Fetched result;
    if (jobs.empty()) return result;
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
            if (j.elevation) storeElevationPng(body, g_elevationSize);
            if (!writeAtomically(j.path, body)) {
                failed.fetch_add(1);
                continue;
            }
            bytes.fetch_add(body.size());
            done.fetch_add(1);
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::min<std::size_t>(threads, jobs.size()); ++t) pool.emplace_back(worker);
    std::size_t lastReport = 0;
    while (done.load() + failed.load() < jobs.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const std::size_t d = done.load() + failed.load();
        if (d - lastReport >= 500 || d == jobs.size()) {
            lastReport = d;
            const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("  %s: %zu / %zu  (%.1f MB, %.0f tiles/s)\n", what, d, jobs.size(),
                        static_cast<double>(bytes.load()) / 1e6, sec > 0 ? static_cast<double>(d) / sec : 0.0);
            std::fflush(stdout);
        }
    }
    for (auto& t : pool) t.join();
    result.done = done.load();
    result.failed = failed.load();
    result.bytes = bytes.load();
    return result;
}

/// Every level-6 elevation tile, so the land mask can be built; fetched first
/// if it is not already there (it is part of any global base anyway).
LandMask buildLandMask(const std::filesystem::path& cache, const std::string& elevationUrl, bool fetchMissing,
                       unsigned threads) {
    std::vector<Job> missing;
    for (unsigned y = 0; y < 64; ++y)
        for (unsigned x = 0; x < 64; ++x) {
            Job j{expand(elevationUrl, 6, x, y), {}, true};
            j.path = cachePathOf(cache, j.url);
            if (!std::filesystem::exists(j.path)) missing.push_back(std::move(j));
        }
    if (fetchMissing && !missing.empty()) download(missing, threads, "land mask");

    LandMask mask;
    for (unsigned y = 0; y < 64; ++y)
        for (unsigned x = 0; x < 64; ++x) {
            std::ifstream in(cachePathOf(cache, expand(elevationUrl, 6, x, y)), std::ios::binary);
            if (!in) {
                ++mask.missing;
                continue;
            }
            const std::vector<std::uint8_t> png{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
            const io::ElevationTile t = io::decodeTerrariumPng(png);
            if (!t.valid()) {
                ++mask.missing;
                continue;
            }
            auto& m = mask.tiles[y * 64u + x];
            m.n = t.size;
            m.land.resize(t.heights.size());
            // Rounded as the stored form rounds, so the mask - and with it the
            // plan - is the same before and after these tiles are re-stored.
            for (std::size_t i = 0; i < t.heights.size(); ++i) m.land[i] = std::round(t.heights[i]) > 0.0f ? 1 : 0;
        }
    return mask;
}

void addLand(std::unordered_set<std::uint64_t>& out, unsigned z) {
    if (!g_land) return;
    if (z < 6) {
        const unsigned n = 1u << z;
        for (unsigned y = 0; y < n; ++y)
            for (unsigned x = 0; x < n; ++x)
                if (g_land->land(z, x, y)) out.insert(key(z, x, y));
        return;
    }
    const unsigned k = 1u << (z - 6);
    for (unsigned ty = 0; ty < 64; ++ty)
        for (unsigned tx = 0; tx < 64; ++tx) {
            const auto& t = g_land->tiles[ty * 64u + tx];
            if (t.n != 0 && std::find(t.land.begin(), t.land.end(), std::uint8_t{1}) == t.land.end()) continue;
            for (unsigned sy = 0; sy < k; ++sy)
                for (unsigned sx = 0; sx < k; ++sx) {
                    const unsigned x = tx * k + sx, y = ty * k + sy;
                    if (g_land->land(z, x, y)) out.insert(key(z, x, y));
                }
        }
}

double haversineKm(double lat1, double lon1, double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * kDeg, dLon = (lon2 - lon1) * kDeg;
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                     std::cos(lat1 * kDeg) * std::cos(lat2 * kDeg) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2.0 * kEarthR * std::asin(std::min(1.0, std::sqrt(a))) / 1000.0;
}

/// Every tile at `z` whose centre is within `radiusKm` of a point.
void addDisc(std::unordered_set<std::uint64_t>& out, double lat, double lon, double radiusKm, unsigned z) {
    double tx, ty;
    io::mercatorTile(lat * kDeg, lon * kDeg, z, tx, ty);
    const int n = static_cast<int>(1u << z);
    const double metresPerTile = 2.0 * kPi * kEarthR * std::cos(lat * kDeg) / n;
    if (metresPerTile <= 0.0) return;
    const int r = static_cast<int>(std::ceil(radiusKm * 1000.0 / metresPerTile));
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const int y = cy + dy;
            if (y < 0 || y >= n) continue;
            if (std::hypot(dx, dy) > r + 0.5) continue;
            out.insert(key(z, static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)));
        }
}

void addBox(std::unordered_set<std::uint64_t>& out, double minLat, double minLon, double maxLat, double maxLon, unsigned z) {
    double x0, y0, x1, y1;
    io::mercatorTile(maxLat * kDeg, minLon * kDeg, z, x0, y0); // north-west
    io::mercatorTile(minLat * kDeg, maxLon * kDeg, z, x1, y1); // south-east
    const int n = static_cast<int>(1u << z);
    const int xa = static_cast<int>(std::floor(x0)), xb = static_cast<int>(std::floor(x1));
    const int ya = std::max(0, static_cast<int>(std::floor(y0)));
    const int yb = std::min(n - 1, static_cast<int>(std::floor(y1)));
    for (int y = ya; y <= yb; ++y)
        for (int x = xa; x <= xb; ++x) out.insert(key(z, static_cast<unsigned>((x % n + n) % n), static_cast<unsigned>(y)));
}

void addGlobal(std::unordered_set<std::uint64_t>& out, unsigned z) {
    const unsigned n = 1u << z;
    for (unsigned y = 0; y < n; ++y)
        for (unsigned x = 0; x < n; ++x) out.insert(key(z, x, y));
}

/// A corridor: discs of half the width, stepped along the route closely
/// enough that they overlap. The set takes care of the overlap.
void addRoute(std::unordered_set<std::uint64_t>& out, const std::vector<std::pair<double, double>>& pts,
              double widthKm, unsigned z) {
    if (pts.size() < 2 || widthKm <= 0.0) return;
    const double half = widthKm / 2.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto [la, lo] = pts[i];
        const auto [lb, lb2] = pts[i + 1];
        const double lenKm = haversineKm(la, lo, lb, lb2);
        const int steps = std::max(1, static_cast<int>(std::ceil(lenKm / std::max(1.0, half))));
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            addDisc(out, la + (lb - la) * t, lo + (lb2 - lo) * t, half, z);
        }
    }
}

/// Whether a tapered region leaves out tile row `y` of level `z`, given the
/// region's deepest level `top`.
///
/// Web Mercator tiles shrink with cos(latitude): a level-8 tile at 60 degrees
/// is as sharp as a level-9 tile at the equator, and at 75 degrees as sharp as
/// a level-10 one. One level everywhere therefore spends most of its tiles
/// near the poles - 62 % of the land's level-8 tiles lie beyond 60 degrees -
/// on detail the rest of the world does not get. Tapered, `top` is what the
/// equator gets, and every level stops where its parent already matches that:
/// nowhere coarser than the equator, and nothing spent on being finer. Tested
/// at the tile's edge nearest the equator, so a tile is kept if any of it needs
/// to be, and a kept tile's parent always is.
bool taperedOut(unsigned z, unsigned y, int top) {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    auto latitude = [n](double row) { return std::atan(std::sinh(kPi * (1.0 - 2.0 * row / n))); };
    const double a = latitude(static_cast<double>(y)), b = latitude(static_cast<double>(y) + 1.0);
    const double nearest = (a > 0.0) == (b > 0.0) ? std::min(std::abs(a), std::abs(b)) : 0.0;
    return std::cos(nearest) <= std::ldexp(1.0, static_cast<int>(z) - top - 1);
}

void enumerate(const Region& r, Layer layer, std::unordered_set<std::uint64_t>& out) {
    const Levels& lv = layer == Layer::Imagery ? r.imagery : r.elevation;
    if (!lv.wanted()) return;
    for (int z = lv.lo; z <= lv.hi; ++z) {
        const auto uz = static_cast<unsigned>(z);
        // The deepest level covers the region plus a margin; every level above
        // it covers more ground still. Fetching the same extent at every level
        // puts the whole pyramid inside one outline, and the viewer draws that
        // outline: a rectangle of detail sitting in coarse terrain, with a
        // visible step along its edge. A skirt spreads the step over several
        // levels, where each one is four times cheaper than the last, so most
        // of the fix costs almost nothing.
        //
        // The margin is what the frustum needs. A camera looking at the middle
        // of a disc still has its far edge over ground outside it: eight
        // places tested pulled 410 tiles from beyond their regions.
        //
        // The skirt is measured in tiles, not as a fraction of the region. A
        // fraction multiplies area, and at four levels up that is seven times
        // the ground - the plan went from 1.95 GB to 2.64 GB for a cosmetic
        // fix. A skirt two tiles wide costs a ring around the perimeter
        // instead, and because a tile at a coarse level covers far more
        // ground, the same two tiles make a far wider skirt exactly where it
        // is cheap to do so.
        // Capped at the region's own size, or a long thin one runs away with
        // it: two tiles at level 8 is 218 km, which would turn a 40 km flight
        // corridor into a 482 km one for twenty thousand kilometres.
        const double tileKm = 2.0 * kPi * kEarthR * std::cos(r.lat * kDeg) / (1 << z) / 1000.0;
        switch (r.kind) {
        case Region::Kind::Circle:
            addDisc(out, r.lat, r.lon,
                    r.radiusKm * (1.0 + r.margin) + std::min(r.feather * tileKm, r.radiusKm), uz);
            break;
        case Region::Kind::Box: {
            const double midLat = (r.minLat + r.maxLat) * 0.5;
            const double kmPerLat = 111.32, kmPerLon = 111.32 * std::max(0.05, std::cos(midLat * kDeg));
            const double boxTileKm = 2.0 * kPi * kEarthR * std::cos(midLat * kDeg) / (1 << z) / 1000.0;
            const double boxSkirt = std::min(r.feather * boxTileKm, (r.maxLat - r.minLat) * 111.32);
            const double dLat = (r.maxLat - r.minLat) * r.margin * 0.5 + boxSkirt / kmPerLat;
            const double dLon = (r.maxLon - r.minLon) * r.margin * 0.5 + boxSkirt / kmPerLon;
            addBox(out, r.minLat - dLat, r.minLon - dLon, r.maxLat + dLat, r.maxLon + dLon, uz);
            break;
        }
        case Region::Kind::Route:
            addRoute(out, r.points, r.widthKm * (1.0 + r.margin) + 2.0 * std::min(r.feather * tileKm, r.widthKm), uz);
            break;
        case Region::Kind::Global: addGlobal(out, uz); break;
        case Region::Kind::Land: addLand(out, uz); break;
        }
    }
    if (r.taper)
        for (auto it = out.begin(); it != out.end();) {
            const auto z = static_cast<unsigned>(*it >> 58), y = static_cast<unsigned>(*it & 0x1FFFFFFFu);
            it = taperedOut(z, y, lv.hi) ? out.erase(it) : std::next(it);
        }
}

/// Latitude of the middle of tile row `y` at level `z`, in degrees.
double rowLatitude(unsigned z, unsigned y) {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    return std::atan(std::sinh(kPi * (1.0 - 2.0 * (static_cast<double>(y) + 0.5) / n))) / kDeg;
}

/// What one tile will weigh on disk: the mean, by latitude band, of the land
/// tier's own tiles (levels 7 to 9) in the shipped plan. Good to about 5 %
/// over a plan that size; a city costs some 15 % more than the land around it.
///
/// Imagery is JPEG as served, and ice and snow compress to a third of what
/// the rest of the land does. Elevation is as it is stored here - whole
/// metres, zlib - and depends utterly on whether there is land in the tile: a
/// tile of nothing but sea is 253 bytes.
double estimateTile(Layer layer, const Region& r, std::uint64_t k) {
    const auto z = static_cast<unsigned>(k >> 58), x = static_cast<unsigned>((k >> 29) & 0x1FFFFFFFu),
               y = static_cast<unsigned>(k & 0x1FFFFFFFu);
    const double a = std::abs(rowLatitude(z, y));
    if (layer == Layer::Imagery)
        return 1024.0 * (a < 30 ? 10.6 : a < 45 ? 14.5 : a < 60 ? 13.4 : a < 70 ? 11.8 : a < 80 ? 6.0 : 4.2);
    // Per 128 x 128 tile; 256 x 256 measured 3.36 times that, not 4.
    const double scale = std::pow(static_cast<double>(g_elevationSize) / 128.0, 1.75);
    const double land = scale * 1024.0 * (a < 30 ? 12.0 : a < 45 ? 15.5 : a < 60 ? 13.5 : a < 70 ? 12.5 : a < 80 ? 6.5 : 6.0);
    const double sea = scale * 253.0;
    if (g_land) return g_land->land(z, x, y) ? land : sea;
    // Without a mask only a global region can be assumed to be mostly sea.
    return r.kind == Region::Kind::Global ? kLandFraction * land + (1.0 - kLandFraction) * sea : land;
}

bool parseLevels(const std::string& s, Levels& out) {
    const auto dash = s.find('-');
    if (dash == std::string::npos) {
        out.lo = 0;
        out.hi = std::atoi(s.c_str());
    } else {
        out.lo = std::atoi(s.substr(0, dash).c_str());
        out.hi = std::atoi(s.substr(dash + 1).c_str());
    }
    return out.wanted();
}

std::vector<std::pair<double, double>> parsePoints(const std::string& s) {
    std::vector<std::pair<double, double>> pts;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ';')) {
        const auto comma = item.find(',');
        if (comma == std::string::npos) continue;
        pts.emplace_back(std::atof(item.substr(0, comma).c_str()), std::atof(item.substr(comma + 1).c_str()));
    }
    return pts;
}

Levels levelsFrom(const core::Json& node) {
    Levels lv;
    if (!node.isArray()) return lv;
    const auto& a = node.asArray();
    if (a.size() == 2) {
        lv.lo = static_cast<int>(a[0].asNumber());
        lv.hi = static_cast<int>(a[1].asNumber());
    }
    return lv;
}

bool loadPlan(const std::filesystem::path& file, std::vector<Region>& regions) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open plan %s\n", file.string().c_str());
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    core::Json doc;
    try {
        doc = core::Json::parse(text, file.string());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "plan %s: %s\n", file.string().c_str(), e.what());
        return false;
    }
    if (doc.has("elevationSize")) g_elevationSize = static_cast<std::uint32_t>(doc.number("elevationSize", 256));
    const core::Json& list = doc.child("regions");
    if (!list.isArray()) {
        std::fprintf(stderr, "plan %s: no \"regions\" array\n", file.string().c_str());
        return false;
    }
    for (const auto& node : list.asArray()) {
        Region r;
        r.name = node.string("name", "region");
        const std::string kind = node.string("type", "circle");
        if (kind == "global") r.kind = Region::Kind::Global;
        else if (kind == "land") r.kind = Region::Kind::Land;
        else if (kind == "box") r.kind = Region::Kind::Box;
        else if (kind == "route") r.kind = Region::Kind::Route;
        else r.kind = Region::Kind::Circle;
        r.lat = node.number("lat", 0.0);
        r.lon = node.number("lon", 0.0);
        r.radiusKm = node.number("radiusKm", 0.0);
        r.widthKm = node.number("widthKm", 0.0);
        r.margin = node.number("margin", r.margin);
        r.feather = node.number("feather", r.feather);
        r.taper = node.boolean("taper", false);
        if (const core::Json& bbox = node.child("bbox"); bbox.isArray() && bbox.asArray().size() == 4) {
            r.minLat = bbox.asArray()[0].asNumber();
            r.minLon = bbox.asArray()[1].asNumber();
            r.maxLat = bbox.asArray()[2].asNumber();
            r.maxLon = bbox.asArray()[3].asNumber();
        }
        if (const core::Json& pts = node.child("points"); pts.isArray())
            for (const auto& p : pts.asArray())
                if (p.isArray() && p.asArray().size() == 2)
                    r.points.emplace_back(p.asArray()[0].asNumber(), p.asArray()[1].asNumber());
        r.imagery = levelsFrom(node.child("imagery"));
        r.elevation = levelsFrom(node.child("elevation"));
        regions.push_back(std::move(r));
    }
    return !regions.empty();
}

void usage() {
    std::printf(
        "tile_prefetch: fill the tile cache for chosen regions and levels\n"
        "\n"
        "  region (one of):\n"
        "    --lat <deg> --lon <deg> --radius-km <km>   a disc\n"
        "    --bbox <minLat,minLon,maxLat,maxLon>       a lat/lon box\n"
        "    --route <lat,lon;lat,lon;...> --width-km <km>   a corridor along a route\n"
        "    --global                                   every tile on Earth\n"
        "    --land                                     every tile with land in it (the oceans skipped)\n"
        "  levels:\n"
        "    --levels <lo-hi>             both layers\n"
        "    --imagery-levels <lo-hi>     imagery only (omit to skip imagery)\n"
        "    --elevation-levels <lo-hi>   elevation only (omit to skip elevation)\n"
        "    --min-level/--max-level      the older spelling of --levels\n"
        "  plan:\n"
        "    --plan <file.json>           many regions at once; see the header comment\n"
        "  other:\n"
        "    --dry-run                    count and price the tiles, download nothing\n"
        "    --elevation-size <n>         store elevation at n x n texels (256); 128 is a quarter\n"
        "                                 of the bytes and still twice what the mesh draws\n"
        "    --list <file>                write every tile the plan asks for, one per line\n"
        "                                 (\"imagery z x y\"), for pricing a plan another way\n"
        "    --prune                      delete cached tiles the plan does not ask for,\n"
        "                                 so a package carries only what it needs\n"
        "    --margin <f>                 widen every level by this fraction (0.15)\n"
        "    --feather <tiles>            skirt of this many tiles at every level (2), so\n"
        "                                 detail tapers instead of ending at a wall\n"
        "    --taper                      the levels are the equator's; nearer the poles, where\n"
        "                                 Mercator tiles are smaller, stop each level where the\n"
        "                                 one above already has the equator's ground resolution\n"
        "    --cache <dir> --threads <n> --imagery-url <tmpl> --elevation-url <tmpl>\n");
}

} // namespace

int main(int argc, char** argv) {
    std::vector<Region> regions;
    Region cli;
    cli.name = "region";
    bool haveCli = false, dryRun = false, prune = false;
    Levels bothLevels;
    unsigned threads = 8;
    std::filesystem::path cache, planFile, listFile;
    std::string elevationUrl = io::TerrainTiles::Options{}.urlTemplate, imageryUrl = kImageryUrl;

    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "-h" || k == "--help") {
            usage();
            return 0;
        } else if (k == "--lat") { cli.lat = std::atof(next()); haveCli = true; }
        else if (k == "--lon") { cli.lon = std::atof(next()); haveCli = true; }
        else if (k == "--radius-km") { cli.radiusKm = std::atof(next()); cli.kind = Region::Kind::Circle; haveCli = true; }
        else if (k == "--bbox") {
            const auto v = parsePoints(std::string(next()) + ";");
            const std::string raw = argv[i];
            double a[4] = {0, 0, 0, 0};
            std::stringstream ss(raw);
            std::string tok;
            for (int n = 0; n < 4 && std::getline(ss, tok, ','); ++n) a[n] = std::atof(tok.c_str());
            cli.minLat = a[0]; cli.minLon = a[1]; cli.maxLat = a[2]; cli.maxLon = a[3];
            cli.kind = Region::Kind::Box;
            haveCli = true;
        } else if (k == "--route") { cli.points = parsePoints(next()); cli.kind = Region::Kind::Route; haveCli = true; }
        else if (k == "--width-km") { cli.widthKm = std::atof(next()); haveCli = true; }
        else if (k == "--global") { cli.kind = Region::Kind::Global; haveCli = true; }
        else if (k == "--levels") parseLevels(next(), bothLevels);
        else if (k == "--imagery-levels") parseLevels(next(), cli.imagery);
        else if (k == "--elevation-levels") parseLevels(next(), cli.elevation);
        else if (k == "--min-level") { if (bothLevels.lo < 0) bothLevels.lo = 0; bothLevels.lo = std::atoi(next()); }
        else if (k == "--max-level") { if (bothLevels.lo < 0) bothLevels.lo = 0; bothLevels.hi = std::atoi(next()); }
        else if (k == "--elevation-only") { cli.imagery = Levels{}; }
        else if (k == "--imagery-only") { cli.elevation = Levels{}; }
        else if (k == "--margin") cli.margin = std::atof(next());
        else if (k == "--feather") cli.feather = std::atof(next());
        else if (k == "--threads") threads = static_cast<unsigned>(std::max(1, std::atoi(next())));
        else if (k == "--cache") cache = next();
        else if (k == "--plan") planFile = next();
        else if (k == "--list") listFile = next();
        else if (k == "--dry-run") dryRun = true;
        else if (k == "--prune") prune = true;
        else if (k == "--land") { cli.kind = Region::Kind::Land; haveCli = true; }
        else if (k == "--taper") cli.taper = true;
        else if (k == "--elevation-size") g_elevationSize = static_cast<std::uint32_t>(std::max(2, std::atoi(next())));
        else if (k == "--elevation-url") elevationUrl = next();
        else if (k == "--imagery-url") imageryUrl = next();
        else {
            std::fprintf(stderr, "unknown option %s\n", k.c_str());
            usage();
            return 2;
        }
    }
    if (cache.empty()) cache = platform::configDir() / "tilecache";

    if (!planFile.empty()) {
        if (!loadPlan(planFile, regions)) return 2;
    } else if (haveCli) {
        // --levels (or the older --min/--max-level) fills in whichever layers
        // were not given a range of their own.
        if (bothLevels.wanted()) {
            if (!cli.imagery.wanted()) cli.imagery = bothLevels;
            if (!cli.elevation.wanted()) cli.elevation = bothLevels;
        }
        if (!cli.imagery.wanted() && !cli.elevation.wanted()) {
            std::fprintf(stderr, "nothing to fetch: give --levels, --imagery-levels or --elevation-levels\n");
            return 2;
        }
        regions.push_back(cli);
    } else {
        usage();
        return 2;
    }

    // Land regions need to know where the land is first.
    LandMask landMask;
    if (std::any_of(regions.begin(), regions.end(), [](const Region& r) { return r.kind == Region::Kind::Land; })) {
        landMask = buildLandMask(cache, elevationUrl, !dryRun, threads);
        g_land = &landMask;
        if (landMask.missing)
            std::printf("land mask: %zu of 4096 level-6 tiles not on disk; counted as land (run without --dry-run to fetch them)\n",
                        landMask.missing);
    }

    // Enumerate every region, keeping the two layers apart and de-duplicating
    // within each: overlapping regions cost what their union costs.
    std::unordered_set<std::uint64_t> elevSet, imgSet;
    // Per-tile estimate, kept per tile rather than per region: regions overlap,
    // and a total that adds the regions up charges for the overlap twice.
    std::unordered_map<std::uint64_t, double> elevRate, imgRate;
    std::printf("%-22s %-8s %10s %10s %10s\n", "region", "layer", "levels", "tiles", "est MB");
    for (const auto& r : regions) {
        for (const Layer layer : {Layer::Elevation, Layer::Imagery}) {
            const Levels& lv = layer == Layer::Imagery ? r.imagery : r.elevation;
            if (!lv.wanted()) continue;
            std::unordered_set<std::uint64_t> mine;
            enumerate(r, layer, mine);
            auto& rates = layer == Layer::Imagery ? imgRate : elevRate;
            double mb = 0.0;
            for (const std::uint64_t k : mine) {
                const double b = estimateTile(layer, r, k);
                mb += b / 1e6;
                const auto [it, fresh] = rates.emplace(k, b);
                if (!fresh) it->second = std::max(it->second, b);
            }
            char range[32];
            std::snprintf(range, sizeof(range), "%d-%d", lv.lo, lv.hi);
            std::printf("%-22s %-8s %10s %10zu %10.0f\n", r.name.c_str(),
                        layer == Layer::Imagery ? "imagery" : "elev", range, mine.size(), mb);
            auto& into = layer == Layer::Imagery ? imgSet : elevSet;
            into.insert(mine.begin(), mine.end());
        }
    }
    double elevMB = 0.0, imgMB = 0.0;
    for (const auto& kv : elevRate) elevMB += kv.second / 1e6;
    for (const auto& kv : imgRate) imgMB += kv.second / 1e6;
    std::printf("%-22s %-8s %10s %10zu %10.0f\n", "unique", "elev", "", elevSet.size(), elevMB);
    std::printf("%-22s %-8s %10s %10zu %10.0f\n", "unique", "imagery", "", imgSet.size(), imgMB);
    std::printf("%-22s %-8s %10s %10zu %10.0f   (%.2f GB)\n", "TOTAL", "", "",
                elevSet.size() + imgSet.size(), elevMB + imgMB, (elevMB + imgMB) / 1000.0);

    if (!listFile.empty()) {
        std::ofstream list(listFile);
        for (const auto* set : {&imgSet, &elevSet})
            for (const std::uint64_t k : *set)
                list << (set == &imgSet ? "imagery " : "elevation ") << (k >> 58) << ' ' << ((k >> 29) & 0x1FFFFFFFu)
                     << ' ' << (k & 0x1FFFFFFFu) << '\n';
    }

    // Turn the sets into work, skipping what is already on disk.
    std::vector<Job> jobs;
    std::unordered_set<std::string> wanted;
    std::size_t present = 0;
    for (const auto* pair : {&elevSet, &imgSet}) {
        const bool isImagery = pair == &imgSet;
        for (const std::uint64_t k : *pair) {
            const unsigned z = static_cast<unsigned>(k >> 58);
            const unsigned x = static_cast<unsigned>((k >> 29) & 0x1FFFFFFFu);
            const unsigned y = static_cast<unsigned>(k & 0x1FFFFFFFu);
            Job j{expand(isImagery ? imageryUrl : elevationUrl, z, x, y), {}, !isImagery};
            j.path = cachePathOf(cache, j.url);
            wanted.insert(j.path.lexically_normal().string());
            if (std::filesystem::exists(j.path)) ++present;
            else jobs.push_back(std::move(j));
        }
    }

    // Tiles from an earlier, wider plan are still on disk and still cost their
    // bytes. A package built to a budget should carry what the plan asks for
    // and nothing else, so --prune sweeps the rest - only inside the cache
    // directory it was given, and only files, never directories.
    if (prune) {
        std::size_t removed = 0, freed = 0;
        std::error_code ec;
        if (std::filesystem::is_directory(cache, ec)) {
            std::vector<std::filesystem::path> doomed;
            for (std::filesystem::recursive_directory_iterator it(cache, ec), end; it != end; it.increment(ec)) {
                if (ec || !it->is_regular_file(ec)) continue;
                if (wanted.count(it->path().lexically_normal().string())) continue;
                doomed.push_back(it->path());
            }
            for (const auto& f : doomed) {
                const auto size = std::filesystem::file_size(f, ec);
                if (std::filesystem::remove(f, ec)) {
                    ++removed;
                    freed += ec ? 0 : size;
                }
            }
        }
        std::printf("  pruned %zu tile(s) the plan does not want (%.1f MB)\n", removed,
                    static_cast<double>(freed) / 1e6);
    }
    // A viewer that asks for detail the package does not have spends its time
    // on requests that cannot be answered, and offline it just waits for them
    // to fail. Tell the operator what ceilings match this plan.
    int deepestImg = -1, deepestElev = -1;
    for (const std::uint64_t k : imgSet) deepestImg = std::max(deepestImg, static_cast<int>(k >> 58));
    for (const std::uint64_t k : elevSet) deepestElev = std::max(deepestElev, static_cast<int>(k >> 58));
    if (deepestImg >= 0 || deepestElev >= 0)
        std::printf("for an offline package set viewer.json  \"maxLevel\": %d,  \"elevationMaxLevel\": %d\n",
                    deepestImg, deepestElev);
    // Elevation already on disk from an earlier run - served as-is, or stored
    // in an older form - is re-stored in place rather than fetched again.
    std::vector<std::filesystem::path> restore;
    for (const std::uint64_t k : elevSet) {
        const auto path = cachePathOf(cache, expand(elevationUrl, static_cast<unsigned>(k >> 58),
                                                    static_cast<unsigned>((k >> 29) & 0x1FFFFFFFu),
                                                    static_cast<unsigned>(k & 0x1FFFFFFFu)));
        if (std::filesystem::exists(path) && !inStoredForm(path, g_elevationSize)) restore.push_back(path);
    }
    std::printf("cache %s\n  %zu already there, %zu to download", cache.string().c_str(), present, jobs.size());
    if (!restore.empty()) std::printf(", %zu elevation tile(s) to re-store", restore.size());
    std::printf("\n");
    if (dryRun) {
        std::printf("dry run: nothing downloaded\n");
        return 0;
    }

    const Fetched got = download(jobs, threads, "tiles");
    std::printf("done: %zu downloaded (%.1f MB), %zu failed\n", got.done, static_cast<double>(got.bytes) / 1e6, got.failed);

    std::atomic<std::size_t> at{0}, restored{0}, before{0}, after{0};
    auto worker = [&] {
        for (std::size_t i = at.fetch_add(1); i < restore.size(); i = at.fetch_add(1)) {
            std::ifstream in(restore[i], std::ios::binary);
            std::vector<std::uint8_t> png{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
            in.close();
            const std::size_t was = png.size();
            if (storeElevationPng(png, g_elevationSize) && writeAtomically(restore[i], png)) {
                restored.fetch_add(1);
                before.fetch_add(was);
                after.fetch_add(png.size());
            }
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::min<std::size_t>(threads, restore.size()); ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();
    if (!restore.empty())
        std::printf("re-stored %zu elevation tile(s) at up to %u x %u, whole metres: %.1f MB -> %.1f MB\n",
                    restored.load(), g_elevationSize, g_elevationSize, static_cast<double>(before.load()) / 1e6,
                    static_cast<double>(after.load()) / 1e6);
    return got.failed ? 1 : 0;
}
