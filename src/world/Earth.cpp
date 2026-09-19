#include "world/Earth.h"

#include "core/Log.h"

#include <algorithm>

namespace fsim::world {

vsg::ref_ptr<vsg::Node> createEarth(const EarthSettings& settings, vsg::ref_ptr<vsg::Options> options,
                                    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid) {
    vsg::ref_ptr<vsg::TileDatabaseSettings> tiles;

    switch (settings.source) {
    case EarthSettings::Source::None: return {};
    case EarthSettings::Source::OpenStreetMap:
        tiles = vsg::createOpenStreetMapSettings(options);
        break;
    case EarthSettings::Source::EsriWorldImagery:
        // Public satellite/aerial imagery (Esri, Maxar et al.); attribution required.
        tiles = vsg::createOpenStreetMapSettings(options); // same XYZ / EPSG:3857 layout
        tiles->imageLayer = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}.jpg"; // .jpg suffix is accepted and lets VSG pick the reader
        tiles->maxLevel = 19;
        break;
    case EarthSettings::Source::Bing:
        if (settings.bingKey.empty()) {
            LOG_ERROR("world") << "Bing imagery needs --bing-key";
            return {};
        }
        tiles = vsg::createBingMapsSettings(settings.bingImagerySet, "en-GB", settings.bingKey, options);
        break;
    case EarthSettings::Source::Custom:
        if (settings.imageryUrl.empty()) {
            LOG_ERROR("world") << "custom imagery needs --imagery-url";
            return {};
        }
        tiles = vsg::TileDatabaseSettings::create();
        tiles->extents = {{-180.0, -90.0, 0.0}, {180.0, 90.0, 1.0}};
        tiles->noX = 1;
        tiles->noY = 1;
        tiles->maxLevel = settings.maxLevel;
        tiles->originTopLeft = settings.originTopLeft;
        tiles->lighting = false;
        tiles->projection = settings.projection;
        tiles->imageLayer = settings.imageryUrl;
        break;
    }
    if (!tiles) return {};

    tiles->ellipsoidModel = ellipsoid;
    tiles->lodTransitionScreenHeightRatio = settings.lodTransitionScreenHeightRatio;

    if (!settings.elevationUrl.empty()) {
        // Relief: VSG displaces each tile's mesh with the elevation texture (one
        // vertex per texel), so the decoder also downsamples to a sane mesh size.
        const auto encoding = settings.elevationEncoding;
        const auto meshDim = settings.elevationMeshDimension;
        tiles->elevationLayer = settings.elevationUrl;
        tiles->elevationLayerCallback = [encoding, meshDim](vsg::ref_ptr<vsg::Data> data) -> vsg::ref_ptr<vsg::Data> {
            return decodeElevation(data, encoding, meshDim);
        };
        tiles->lighting = true; // relief needs shading to be visible
        // Tiles below the elevation pyramid's deepest level would come back flat
        // and pop; cap the whole pyramid there.
        tiles->maxLevel = std::min(tiles->maxLevel, settings.elevationMaxLevel);
    }

    auto earth = vsg::TileDatabase::create();
    earth->settings = tiles;
    if (!earth->readDatabase(options)) {
        LOG_ERROR("world") << "TileDatabase::readDatabase failed for " << tiles->imageLayer.string()
                           << " (network down or reader missing?)";
        return {};
    }
    LOG_INFO("world") << "Earth: " << tiles->imageLayer.string()
                      << (tiles->elevationLayer.empty() ? "" : " + elevation " + tiles->elevationLayer.string())
                      << ", max level " << tiles->maxLevel;
    return earth;
}

} // namespace fsim::world
