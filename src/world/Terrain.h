#pragma once

#include "sim/GroundProvider.h"

#include <vsg/all.h>

#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <list>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <string>
#include <unordered_map>

namespace fsim::world {

/// Elevation tile encodings the platform understands (design 8.2 "Tile
/// pyramid format").
enum class ElevationEncoding {
    Float,     ///< single-channel float metres (R32_SFLOAT / R16_SFLOAT), VSG native
    Terrarium, ///< RGB8: h = R*256 + G + B/256 - 32768 (AWS terrain tiles, Mapzen)
};

/// Public, key-free global elevation: AWS Terrain Tiles, Terrarium encoding,
/// zoom 0-15 (~5 m/px at z15, 30 m SRTM/derived data).
inline constexpr const char* kAwsTerrariumUrl = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";

/// Decode an elevation tile image into metres. Returns a floatArray2D
/// (R32_SFLOAT) or nullptr if the input is not understood. `maxDimension`
/// downsamples large tiles (VSG builds one mesh vertex per elevation texel,
/// so 256x256 tiles would cost 130k triangles each); 0 keeps full resolution.
vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension);

/// Physics ground height from the same elevation tiles the renderer shows
/// (design 7.3, ADR-7). Tiles are fetched through VSG's readers (http via
/// vsgXchange curl with the file cache, or local files), decoded at full
/// resolution and kept in a small LRU cache; samples are bilinear.
///
/// Fetches are synchronous on the calling (simulation worker) thread: the
/// first visit to a tile costs a network round trip (or a disk read once
/// cached); `prefetch()` warms the cache around a spawn point.
class TileGroundProvider final : public sim::GroundProvider {
public:
    TileGroundProvider(std::string urlTemplate, ElevationEncoding encoding, unsigned zoom,
                       vsg::ref_ptr<const vsg::Options> options, std::size_t cacheTiles = 256);

    double heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const override;

    /// Load every tile within `radiusM` of a position (blocking, parallel).
    void prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads = 8);

    /// Queue the tile under a position and its 8 neighbours for background
    /// loading (non-blocking). Call periodically per vehicle so the sim thread
    /// rarely misses (design 8.4 "prefetch"). Safe from any thread.
    void requestAround(double latitudeRad, double longitudeRad);

    ~TileGroundProvider() override;

    unsigned zoom() const noexcept { return zoom_; }
    std::size_t cachedTiles() const;
    std::size_t misses() const noexcept { return misses_; }

private:
    struct Key {
        unsigned x, y;
        bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept { return (static_cast<std::size_t>(k.x) << 32) ^ k.y; }
    };
    using Tile = vsg::ref_ptr<vsg::floatArray2D>;

    Tile tile(unsigned x, unsigned y) const;
    Tile load(unsigned x, unsigned y) const;
    std::string tilePath(unsigned x, unsigned y) const;

    std::string urlTemplate_;
    ElevationEncoding encoding_;
    unsigned zoom_;
    vsg::ref_ptr<const vsg::Options> options_;
    std::size_t capacity_;

    mutable std::mutex mutex_;
    mutable std::unordered_map<Key, std::pair<Tile, std::list<Key>::iterator>, KeyHash> cache_;
    mutable std::list<Key> lru_; ///< front = most recently used
    mutable std::size_t misses_ = 0;

    // Background loader for requestAround().
    void loaderLoop();
    void ensureLoader();
    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<Key> queue_;
    std::unordered_set<std::uint64_t> queued_;
    std::vector<std::thread> loaders_;
    std::atomic<bool> stopLoaders_{false};
};

} // namespace fsim::world
