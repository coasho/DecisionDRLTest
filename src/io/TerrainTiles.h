#pragma once

#include "fsim/GroundProvider.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fsim::io {

/// A decoded elevation tile: `size` x `size` metres above the ellipsoid, row 0 = north.
struct ElevationTile {
    std::uint32_t size = 0;
    std::vector<float> heights;
    bool valid() const noexcept { return size >= 2 && heights.size() == static_cast<std::size_t>(size) * size; }
};

/// Decode a Terrarium PNG (AWS terrain tiles: h = R*256 + G + B/256 - 32768)
/// into metres. Bathymetry is clamped to sea level, matching the viewer.
/// Returns an invalid tile if the bytes are not a decodable image.
ElevationTile decodeTerrariumPng(const std::vector<std::uint8_t>& png);

/// Web-Mercator tile coordinates of a geodetic position at `zoom` (continuous:
/// integer part = tile index, fraction = position within the tile).
void mercatorTile(double latitudeRad, double longitudeRad, unsigned zoom, double& tx, double& ty) noexcept;

/// Physics ground from public elevation tiles, headless (design 7.3, ADR-7):
/// the same AWS Terrarium tiles the viewer drapes its imagery over, fetched
/// with the platform HTTP client, kept in a disk cache the viewer shares
/// (`<config>/tilecache/<host>/<path>`), decoded into an LRU. Sampling is
/// bilinear. A first visit to a tile costs a download (~0.2-1.5 s); use
/// `prefetch()` around spawn points and `requestAround()` as vehicles move.
class TerrainTiles final : public sim::GroundProvider {
public:
    struct Options {
        std::string urlTemplate = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
        unsigned zoom = 12;                     ///< ~38 m/px at mid latitudes (30 m source data)
        std::filesystem::path cacheDir;         ///< empty = <config>/tilecache
        std::size_t cacheTiles = 512;           ///< decoded tiles kept in memory (~260 KB each)
        /// Resample each tile to this many samples per edge, as the renderer
        /// resamples its mesh (world::EarthSettings::elevationMeshDimension).
        /// 0 keeps the raster at full resolution, which is what the physics
        /// wants: it asks where the ground *is*, not where it is drawn. The
        /// camera wants the drawn surface, because flying into scenery it can
        /// see is the thing anyone notices.
        std::uint32_t meshDimension = 0;
        /// Read only what is already in cacheDir; never fetch. What a
        /// distributed package does: everything it can draw was put there
        /// before it shipped, and a missing tile is a missing tile.
        bool offline = false;
        unsigned loaderThreads = 2;
    };
    /// Test hook: replaces the HTTP fetch (bytes of the tile at z/x/y).
    using Fetch = std::function<bool(unsigned z, unsigned x, unsigned y, std::vector<std::uint8_t>& png)>;

    explicit TerrainTiles(Options options, Fetch fetch = {});
    ~TerrainTiles() override;

    double heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const override;

    /// Height if the tile is already decoded (never blocks); else nullopt and
    /// the tile is queued for background loading.
    std::optional<double> cachedHeightAboveEllipsoidM(double latitudeRad, double longitudeRad);

    /// Load every tile within `radiusM` of a position (blocking, parallel).
    void prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads = 8);
    /// Queue the tile under a position and its 8 neighbours for background loading.
    void requestAround(double latitudeRad, double longitudeRad);

    unsigned zoom() const noexcept { return options_.zoom; }
    std::size_t cachedTiles() const;
    std::size_t misses() const noexcept { return misses_.load(); }
    std::size_t failures() const noexcept { return failures_.load(); }

private:
    struct Key {
        unsigned x, y;
        bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept { return (static_cast<std::size_t>(k.x) << 32) ^ k.y; }
    };
    using Tile = std::shared_ptr<const ElevationTile>;

    Tile tile(unsigned x, unsigned y) const;
    Tile load(unsigned x, unsigned y) const;
    /// Offline, with this level missing: the nearest ancestor on disk,
    /// restricted to this tile (see load()).
    Tile fromAncestor(unsigned x, unsigned y) const;
    std::string url(unsigned x, unsigned y) const;
    std::string urlAt(unsigned z, unsigned x, unsigned y) const;
    std::filesystem::path cachePath(unsigned x, unsigned y) const;
    std::filesystem::path cachePathAt(unsigned z, unsigned x, unsigned y) const;
    static double sample(const ElevationTile& t, double tx, double ty) noexcept;
    void ensureLoaders();
    void loaderLoop();

    Options options_;
    Fetch fetch_;
    mutable std::mutex mutex_;
    mutable std::unordered_map<Key, std::pair<Tile, std::list<Key>::iterator>, KeyHash> cache_;
    mutable std::list<Key> lru_;
    mutable std::atomic<std::size_t> misses_{0}, failures_{0};

    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<Key> queue_;
    std::unordered_set<std::uint64_t> queued_;
    std::vector<std::thread> loaders_;
    std::atomic<bool> stopLoaders_{false};
};

} // namespace fsim::io
