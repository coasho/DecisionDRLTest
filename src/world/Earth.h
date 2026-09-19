#pragma once

#include <vsg/all.h>

#include <string>

namespace fsim::world {

/// Where the Earth's imagery and elevation tiles come from (design 8.2).
struct EarthSettings {
    enum class Source { None, OpenStreetMap, Bing, Custom };
    Source source = Source::OpenStreetMap;

    std::string bingKey;                 ///< Source::Bing
    std::string bingImagerySet = "Aerial";

    // Source::Custom: XYZ templates with {z}/{x}/{y}, file:// or http(s)://
    std::string imageryUrl;
    std::string elevationUrl;            ///< optional 16-bit PNG heightmaps
    unsigned maxLevel = 17;
    bool originTopLeft = true;           ///< XYZ (true) vs TMS (false) row order
    std::string projection = "EPSG:3857";
    double lodTransitionScreenHeightRatio = 0.25;
};

/// Builds the vsg::TileDatabase Earth (imagery + optional elevation) streamed
/// by VSG's DatabasePager. Returns nullptr for Source::None or on failure.
vsg::ref_ptr<vsg::Node> createEarth(const EarthSettings& settings, vsg::ref_ptr<vsg::Options> options,
                                    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

} // namespace fsim::world
