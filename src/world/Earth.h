#pragma once

#include "world/Terrain.h"

#include <vsg/all.h>

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
    unsigned maxLevel = 17;

    // Elevation (any imagery source): XYZ template + encoding. Empty = smooth ellipsoid.
    std::string elevationUrl;
    ElevationEncoding elevationEncoding = ElevationEncoding::Terrarium;
    unsigned elevationMeshDimension = 64; ///< mesh vertices per tile edge (elevation texels are downsampled to this)
    unsigned elevationMaxLevel = 15;      ///< deepest level the elevation pyramid has (AWS Terrarium: 15); imagery is capped to it
    bool originTopLeft = true;           ///< XYZ (true) vs TMS (false) row order
    std::string projection = "EPSG:3857";
    double lodTransitionScreenHeightRatio = 0.25;
};

/// Builds the vsg::TileDatabase Earth (imagery + optional elevation) streamed
/// by VSG's DatabasePager. Returns nullptr for Source::None or on failure.
vsg::ref_ptr<vsg::Node> createEarth(const EarthSettings& settings, vsg::ref_ptr<vsg::Options> options,
                                    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

} // namespace fsim::world
