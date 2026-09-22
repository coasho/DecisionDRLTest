// tile_prefetch: fill the tile cache ahead of time (design 7.3 "offline
// pyramids"), so machines without network access still get terrain physics
// (WorldOptions::terrain) and the viewer still draws imagery and relief.
// Downloads go to the same <cache>/<host>/<path> layout io::TerrainTiles and
// the viewer read from (%LOCALAPPDATA%\flightsim\tilecache by default),
// skipping files already present, so it is safe to re-run and to copy the
// directory to another machine.
//
// A region is a circle, a lat/lon box, a corridor along a route, or the whole
// globe, and each layer gets its own range of levels - which is the whole
// point, because a city wants imagery and a mountain wants elevation, and
// every level costs four times the one below it. --dry-run prices a plan
// without downloading a byte.
//
//   tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --imagery-levels 8-14
//   tile_prefetch --bbox 45.8,6.0,47.2,10.5 --elevation-levels 8-11
//   tile_prefetch --route 37.62,-122.4;34.05,-118.24 --width-km 40 --levels 7-10
//   tile_prefetch --global --levels 0-6 --dry-run
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// Mean bytes per tile, measured over a real cache of 65,000 tiles. Elevation
// depends utterly on whether there is land in the tile: bathymetry is smooth
// and compresses to almost nothing.
constexpr double kImageryBytes = 14.5 * 1024.0;
constexpr double kElevLandBytes = 90.1 * 1024.0;
constexpr double kElevSeaBytes = 16.3 * 1024.0;
constexpr double kLandFraction = 0.35; // of Web Mercator area, land skewing north

enum class Layer { Elevation, Imagery };

struct Levels {
    int lo = -1, hi = -1;
    bool wanted() const noexcept { return lo >= 0 && hi >= lo; }
};

struct Region {
    enum class Kind { Circle, Box, Route, Global };
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

void enumerate(const Region& r, Layer layer, std::unordered_set<std::uint64_t>& out) {
    const Levels& lv = layer == Layer::Imagery ? r.imagery : r.elevation;
    if (!lv.wanted()) return;
    for (int z = lv.lo; z <= lv.hi; ++z) {
        const auto uz = static_cast<unsigned>(z);
        // The deepest level covers the region plus a margin; every level above
        // it covers more ground still. Fetching the same extent at every level
        // puts the whole pyramid inside one outline, and the viewer draws that
        // outline: a rectangle of detail sitting in coarse terrain, with a
        // visible step along its edge. Tapering spreads the step over a skirt
        // several levels wide, where each one is four times cheaper than the
        // last, so most of the fix costs almost nothing.
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
        }
    }
}

double estimateBytes(Layer layer, const Region& r, std::size_t tiles) {
    if (layer == Layer::Imagery) return static_cast<double>(tiles) * kImageryBytes;
    // Only a global region can be assumed to be mostly sea.
    const double per = r.kind == Region::Kind::Global
                           ? kLandFraction * kElevLandBytes + (1.0 - kLandFraction) * kElevSeaBytes
                           : kElevLandBytes;
    return static_cast<double>(tiles) * per;
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
        else if (kind == "box") r.kind = Region::Kind::Box;
        else if (kind == "route") r.kind = Region::Kind::Route;
        else r.kind = Region::Kind::Circle;
        r.lat = node.number("lat", 0.0);
        r.lon = node.number("lon", 0.0);
        r.radiusKm = node.number("radiusKm", 0.0);
        r.widthKm = node.number("widthKm", 0.0);
        r.margin = node.number("margin", r.margin);
        r.feather = node.number("feather", r.feather);
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

struct Job {
    std::string url;
    std::filesystem::path path;
};

void usage() {
    std::printf(
        "tile_prefetch: fill the tile cache for chosen regions and levels\n"
        "\n"
        "  region (one of):\n"
        "    --lat <deg> --lon <deg> --radius-km <km>   a disc\n"
        "    --bbox <minLat,minLon,maxLat,maxLon>       a lat/lon box\n"
        "    --route <lat,lon;lat,lon;...> --width-km <km>   a corridor along a route\n"
        "    --global                                   every tile on Earth\n"
        "  levels:\n"
        "    --levels <lo-hi>             both layers\n"
        "    --imagery-levels <lo-hi>     imagery only (omit to skip imagery)\n"
        "    --elevation-levels <lo-hi>   elevation only (omit to skip elevation)\n"
        "    --min-level/--max-level      the older spelling of --levels\n"
        "  plan:\n"
        "    --plan <file.json>           many regions at once; see the header comment\n"
        "  other:\n"
        "    --dry-run                    count and price the tiles, download nothing\n"
        "    --prune                      delete cached tiles the plan does not ask for,\n"
        "                                 so a package carries only what it needs\n"
        "    --margin <f>                 widen every level by this fraction (0.15)\n"
        "    --feather <tiles>            skirt of this many tiles at every level (2), so\n"
        "                                 detail tapers instead of ending at a wall\n"
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
    std::filesystem::path cache, planFile;
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
        else if (k == "--dry-run") dryRun = true;
        else if (k == "--prune") prune = true;
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

    // Enumerate every region, keeping the two layers apart and de-duplicating
    // within each: overlapping regions cost what their union costs.
    std::unordered_set<std::uint64_t> elevSet, imgSet;
    // Per-tile estimate, kept per tile rather than per region: regions overlap,
    // and a total that adds the regions up charges for the overlap twice.
    std::unordered_map<std::uint64_t, double> elevRate;
    std::printf("%-22s %-8s %10s %10s %10s\n", "region", "layer", "levels", "tiles", "est MB");
    for (const auto& r : regions) {
        for (const Layer layer : {Layer::Elevation, Layer::Imagery}) {
            const Levels& lv = layer == Layer::Imagery ? r.imagery : r.elevation;
            if (!lv.wanted()) continue;
            std::unordered_set<std::uint64_t> mine;
            enumerate(r, layer, mine);
            const double mb = estimateBytes(layer, r, mine.size()) / 1e6;
            char range[32];
            std::snprintf(range, sizeof(range), "%d-%d", lv.lo, lv.hi);
            std::printf("%-22s %-8s %10s %10zu %10.0f\n", r.name.c_str(),
                        layer == Layer::Imagery ? "imagery" : "elev", range, mine.size(), mb);
            auto& into = layer == Layer::Imagery ? imgSet : elevSet;
            into.insert(mine.begin(), mine.end());
            if (layer == Layer::Elevation) {
                const double rate = estimateBytes(layer, r, 1);
                for (const std::uint64_t k : mine) {
                    auto it = elevRate.find(k);
                    if (it == elevRate.end()) elevRate.emplace(k, rate);
                    else it->second = std::max(it->second, rate);
                }
            }
        }
    }
    double elevMB = 0.0;
    for (const auto& [k, rate] : elevRate) {
        (void)k;
        elevMB += rate / 1e6;
    }
    const double imgMB = static_cast<double>(imgSet.size()) * kImageryBytes / 1e6;
    std::printf("%-22s %-8s %10s %10zu %10.0f\n", "unique", "elev", "", elevSet.size(), elevMB);
    std::printf("%-22s %-8s %10s %10zu %10.0f\n", "unique", "imagery", "", imgSet.size(), imgMB);
    std::printf("%-22s %-8s %10s %10zu %10.0f   (%.2f GB)\n", "TOTAL", "", "",
                elevSet.size() + imgSet.size(), elevMB + imgMB, (elevMB + imgMB) / 1000.0);

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
            Job j{expand(isImagery ? imageryUrl : elevationUrl, z, x, y), {}};
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
    std::printf("cache %s\n  %zu already there, %zu to download\n", cache.string().c_str(), present, jobs.size());
    if (dryRun) {
        std::printf("dry run: nothing downloaded\n");
        return 0;
    }
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
        if (d - lastReport >= 200 || d == jobs.size()) {
            lastReport = d;
            const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("  %zu / %zu  (%.1f MB, %.0f tiles/s)\n", d, jobs.size(),
                        static_cast<double>(bytes.load()) / 1e6, s > 0 ? static_cast<double>(d) / s : 0.0);
            std::fflush(stdout);
        }
    }
    for (auto& t : pool) t.join();
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("done: %zu downloaded (%.1f MB), %zu failed, %.0f s\n", done.load(),
                static_cast<double>(bytes.load()) / 1e6, failed.load(), s);
    return failed.load() ? 1 : 0;
}
