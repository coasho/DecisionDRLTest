#pragma once

#include "world/Terrain.h"

#include <vsg/all.h>

#include <filesystem>
#include <string>

namespace fsim::world {

/// Where the Earth's imagery and elevation tiles come from (design 8.2).
struct EarthSettings {
    enum class Source { None, OpenStreetMap, EsriWorldImagery, Bing, Custom };
    Source source = Source::EsriWorldImagery;

    std::string bingKey;                 ///< Source::Bing
    std::string bingImagerySet = "Aerial";

    // Source::Custom: XYZ template with {z}/{x}/{y}, file:// or http(s)://
    std::string imageryUrl;
    unsigned maxLevel = 19;

    // Elevation (any imagery source): XYZ template + encoding. Empty = smooth ellipsoid.
    std::string elevationUrl;

    /// Read tiles from this directory instead of the network. The layer URLs
    /// are rewritten to <offlineRoot>/<host>/<path>, which is the layout the
    /// tile cache and tile_prefetch already write, so a directory fetched
    /// ahead of time is drawn straight off the disk and no socket is opened.
    std::filesystem::path offlineRoot;
    ElevationEncoding elevationEncoding = ElevationEncoding::Terrarium;
    unsigned elevationMeshDimension = 64; ///< mesh vertices per tile edge (elevation texels are downsampled to this)
    unsigned elevationMaxLevel = 15;      ///< deepest level the elevation pyramid has (AWS Terrarium: 15)
    bool upsampleElevation = true;        ///< below it, synthesise relief from the deepest level so imagery keeps refining (else cap the pyramid there)
    bool originTopLeft = true;           ///< XYZ (true) vs TMS (false) row order
    std::string projection = "EPSG:3857";
    double lodTransitionScreenHeightRatio = 0.25;
    double skirtRatio = 0.02;             ///< skirt hanging from every tile edge, as a fraction of the tile size (hides cracks between real elevation levels)
};

/// Builds the vsg::TileDatabase Earth (imagery + optional elevation) streamed
/// by VSG's DatabasePager. Returns nullptr for Source::None or on failure.
vsg::ref_ptr<vsg::Node> createEarth(const EarthSettings& settings, vsg::ref_ptr<vsg::Options> options,
                                    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

/// Web-Mercator tile pyramids stop at +-85.05 deg latitude, which leaves both
/// poles as holes in the globe. Two flat-coloured caps (ice) close them.
vsg::ref_ptr<vsg::Node> createPolarCaps(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid, vsg::ref_ptr<const vsg::Options> options,
                                        double fromLatitudeDeg = 84.9);

} // namespace fsim::world
