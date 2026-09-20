#include "world/Terrain.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::world {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Web-Mercator (EPSG:3857) XYZ tile coordinates for a geodetic position at `zoom`,
/// as continuous values: integer part = tile index, fraction = position in tile.
void mercatorTile(double latRad, double lonRad, unsigned zoom, double& tx, double& ty) {
    const double n = static_cast<double>(1u << zoom);
    const double lat = std::clamp(latRad, -1.4844222297453322, 1.4844222297453322); // +-85.0511 deg
    tx = (lonRad / kPi + 1.0) * 0.5 * n;
    ty = (1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / kPi) * 0.5 * n;
    if (tx < 0.0) tx += n;
    if (tx >= n) tx -= n;
}

// Terrarium tiles include bathymetry. Without a water surface the sea floor
// would render as terrain, and adjacent tiles of different LOD disagree on
// the depth, so coasts and bays showed stepped "roof tiles". Sea level is the
// floor: the ocean is flat at 0 in every LOD. (Land below sea level - Death
// Valley, the Dead Sea - is flattened to 0 as well; a water mask would fix that.)
float terrarium(const vsg::ubvec4& p) noexcept {
    return std::max(0.0f, static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}
float terrarium(const vsg::ubvec3& p) noexcept {
    return std::max(0.0f, static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}

template <typename Array, typename Fn>
vsg::ref_ptr<vsg::floatArray2D> convert(const Array& src, std::uint32_t maxDim, Fn fn) {
    const std::uint32_t w = src.width(), h = src.height();
    const std::uint32_t step = (maxDim > 0 && w > maxDim) ? (w + maxDim - 1) / maxDim : 1;
    const std::uint32_t ow = (w + step - 1) / step, oh = (h + step - 1) / step;
    auto out = vsg::floatArray2D::create(ow, oh, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = src.properties.origin;
    for (std::uint32_t y = 0; y < oh; ++y)
        for (std::uint32_t x = 0; x < ow; ++x)
            out->set(x, y, fn(src.at(std::min(x * step, w - 1), std::min(y * step, h - 1))));
    return out;
}

} // namespace

vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension) {
    if (!image) return {};
    switch (encoding) {
    case ElevationEncoding::Float:
        if (auto f = image.cast<vsg::floatArray2D>()) {
            if (maxDimension == 0 || f->width() <= maxDimension) return f;
            return convert(*f, maxDimension, [](float v) { return v; });
        }
        if (auto s = image.cast<vsg::ushortArray2D>())
            return convert(*s, maxDimension, [](std::uint16_t v) { return static_cast<float>(v); });
        break;
    case ElevationEncoding::Terrarium:
        if (auto rgba = image.cast<vsg::ubvec4Array2D>()) return convert(*rgba, maxDimension, [](const vsg::ubvec4& p) { return terrarium(p); });
        if (auto rgb = image.cast<vsg::ubvec3Array2D>()) return convert(*rgb, maxDimension, [](const vsg::ubvec3& p) { return terrarium(p); });
        break;
    }
    LOG_WARN("world") << "elevation tile has an unexpected pixel format (" << image->className() << ")";
    return {};
}

// ---------------------------------------------------------------------------

TileGroundProvider::TileGroundProvider(std::string urlTemplate, ElevationEncoding encoding, unsigned zoom,
                                       vsg::ref_ptr<const vsg::Options> options, std::size_t cacheTiles)
    : urlTemplate_(std::move(urlTemplate)), encoding_(encoding), zoom_(std::min(zoom, 22u)), options_(options),
      capacity_(std::max<std::size_t>(cacheTiles, 4)) {}

std::string TileGroundProvider::tilePath(unsigned x, unsigned y) const {
    std::string s = urlTemplate_;
    auto sub = [&](const char* key, unsigned v) {
        for (auto pos = s.find(key); pos != std::string::npos; pos = s.find(key)) s.replace(pos, 3, std::to_string(v));
    };
    sub("{z}", zoom_);
    sub("{x}", x);
    sub("{y}", y);
    return s;
}

TileGroundProvider::Tile TileGroundProvider::load(unsigned x, unsigned y) const {
    auto data = vsg::read_cast<vsg::Data>(tilePath(x, y), options_);
    if (!data) {
        LOG_WARN("world") << "elevation tile missing: " << tilePath(x, y);
        return {};
    }
    return decodeElevation(data, encoding_, 0);
}

TileGroundProvider::Tile TileGroundProvider::tile(unsigned x, unsigned y) const {
    const Key key{x, y};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(key); it != cache_.end()) {
            lru_.splice(lru_.begin(), lru_, it->second.second);
            return it->second.first;
        }
    }
    // Fetch outside the lock so other workers keep sampling cached tiles.
    Tile t = load(x, y);
    std::lock_guard<std::mutex> lock(mutex_);
    ++misses_;
    if (auto it = cache_.find(key); it != cache_.end()) return it->second.first; // another worker won the race
    lru_.push_front(key);
    cache_.emplace(key, std::make_pair(t, lru_.begin()));
    while (cache_.size() > capacity_) {
        cache_.erase(lru_.back());
        lru_.pop_back();
    }
    return t;
}

double TileGroundProvider::heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, zoom_, tx, ty);
    const double n = static_cast<double>(1u << zoom_);
    const unsigned x = static_cast<unsigned>(std::floor(tx)), y = static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0));
    return sample(tile(x, y), tx, ty);
}

std::optional<double> TileGroundProvider::cachedHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, zoom_, tx, ty);
    const double n = static_cast<double>(1u << zoom_);
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
        requestAround(latitudeRad, longitudeRad); // background load; known next time
        return std::nullopt;
    }
    return sample(t, tx, ty);
}

double TileGroundProvider::sample(const Tile& t, double tx, double ty) {
    if (!t || t->width() < 2 || t->height() < 2) return 0.0;

    // Bilinear sample; tile pixel (0,0) is the north-west corner (top-left origin).
    const double fx = (tx - std::floor(tx)) * static_cast<double>(t->width() - 1);
    const double fy = (ty - std::floor(ty)) * static_cast<double>(t->height() - 1);
    const std::uint32_t x0 = static_cast<std::uint32_t>(fx), y0 = static_cast<std::uint32_t>(fy);
    const std::uint32_t x1 = std::min(x0 + 1, t->width() - 1), y1 = std::min(y0 + 1, t->height() - 1);
    const double ax = fx - x0, ay = fy - y0;
    const double h00 = t->at(x0, y0), h10 = t->at(x1, y0), h01 = t->at(x0, y1), h11 = t->at(x1, y1);
    const double h = (h00 * (1 - ax) + h10 * ax) * (1 - ay) + (h01 * (1 - ax) + h11 * ax) * ay;
    // Terrarium heights are above mean sea level; the geoid/ellipsoid offset
    // (up to ~100 m) is ignored for now - rendering makes the same assumption,
    // so physics and visuals agree (design 8.2 "Physics vs. visual ground").
    return h;
}

void TileGroundProvider::prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, zoom_, tx, ty);
    const int n = static_cast<int>(1u << zoom_);
    const double metresPerTile = 40075016.686 * std::cos(latitudeRad) / static_cast<double>(n);
    const int r = static_cast<int>(std::ceil(radiusM / std::max(metresPerTile, 1.0)));
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);

    std::vector<Key> keys;
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const int x = cx + dx, y = cy + dy;
            if (y < 0 || y >= n) continue;
            keys.push_back(Key{static_cast<unsigned>((x % n + n) % n), static_cast<unsigned>(y)});
        }

    // Tiles come from S3 at ~1 s each from far away; fetch them in parallel.
    std::atomic<std::size_t> next{0}, loaded{0};
    auto worker = [&] {
        for (std::size_t i = next.fetch_add(1); i < keys.size(); i = next.fetch_add(1))
            if (tile(keys[i].x, keys[i].y)) ++loaded;
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::max(1u, std::min<unsigned>(threads, static_cast<unsigned>(keys.size()))); ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();
    LOG_INFO("world") << "terrain prefetch: " << loaded.load() << "/" << keys.size() << " tile(s) at zoom " << zoom_
                      << " within " << radiusM << " m";
}

void TileGroundProvider::requestAround(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, zoom_, tx, ty);
    const int n = static_cast<int>(1u << zoom_);
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    std::vector<Key> wanted;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
            const int y = cy + dy;
            if (y < 0 || y >= n) continue;
            wanted.push_back(Key{static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)});
        }
    bool any = false;
    {
        std::lock_guard<std::mutex> cacheLock(mutex_);
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        for (const Key& k : wanted) {
            const std::uint64_t id = (static_cast<std::uint64_t>(k.x) << 32) | k.y;
            if (cache_.count(k) || queued_.count(id)) continue;
            queued_.insert(id);
            queue_.push_back(k);
            any = true;
        }
    }
    if (any) {
        ensureLoader();
        queueCv_.notify_all();
    }
}

void TileGroundProvider::ensureLoader() {
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (!loaders_.empty()) return;
    for (int i = 0; i < 2; ++i) loaders_.emplace_back([this] { loaderLoop(); });
}

void TileGroundProvider::loaderLoop() {
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

TileGroundProvider::~TileGroundProvider() {
    stopLoaders_.store(true);
    queueCv_.notify_all();
    for (auto& t : loaders_) t.join();
}

std::size_t TileGroundProvider::cachedTiles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.size();
}

} // namespace fsim::world
