#include "world/Earth.h"

#include "core/Log.h"
#include "world/ElevatedTile.h"
#include "world/FlatGeometry.h"

#include <cmath>

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
    // Our reader: identical tiles, but culling bounds that include the relief.
    if (!readElevatedDatabase(*earth, options)) {
        LOG_ERROR("world") << "TileDatabase::readDatabase failed for " << tiles->imageLayer.string()
                           << " (network down or reader missing?)";
        return {};
    }
    LOG_INFO("world") << "Earth: " << tiles->imageLayer.string()
                      << (tiles->elevationLayer.empty() ? "" : " + elevation " + tiles->elevationLayer.string())
                      << ", max level " << tiles->maxLevel;
    return earth;
}

vsg::ref_ptr<vsg::Node> createPolarCaps(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid, vsg::ref_ptr<const vsg::Options> options,
                                        double fromLatitudeDeg) {
    FlatGeometrySettings settings;
    settings.cullBackFaces = false;
    auto state = createFlatStateGroup(settings, options);
    if (!state) return {};

    // Concentric rings from the cap edge to the pole, a few metres above the
    // ellipsoid so the flat tiles at 85 deg do not z-fight with the ring.
    const int rings = 6, segments = 96;
    const vsg::vec4 ice(0.93f, 0.95f, 0.97f, 1.0f);
    for (double sign : {1.0, -1.0}) {
        const std::uint32_t count = static_cast<std::uint32_t>(rings * segments * 6);
        auto vertices = vsg::vec3Array::create(count);
        auto colors = vsg::vec4Array::create(count, ice);
        auto at = [&](double latDeg, double lonDeg) {
            const vsg::dvec3 p = ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(latDeg, lonDeg, 5.0));
            return vsg::vec3(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
        };
        std::uint32_t i = 0;
        for (int r = 0; r < rings; ++r) {
            const double lat0 = fromLatitudeDeg + (90.0 - fromLatitudeDeg) * r / rings;
            const double lat1 = fromLatitudeDeg + (90.0 - fromLatitudeDeg) * (r + 1) / rings;
            for (int s = 0; s < segments; ++s) {
                const double lon0 = 360.0 * s / segments, lon1 = 360.0 * (s + 1) / segments;
                // Two triangles per quad (the innermost ring degenerates to a fan at the pole; harmless).
                const std::pair<double, double> corners[6] = {{lat0, lon0}, {lat0, lon1}, {lat1, lon1}, {lat0, lon0}, {lat1, lon1}, {lat1, lon0}};
                for (const auto& [la, lo] : corners) vertices->set(i++, at(sign * la, lo));
            }
        }
        state->addChild(createFlatDraw(vertices, colors, false));
    }
    // Float vertices at ECEF magnitude 6.4e6 keep ~0.5 m precision: fine for a flat ice sheet.
    return state;
}

} // namespace fsim::world
