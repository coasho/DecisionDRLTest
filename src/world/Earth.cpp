#include "world/Earth.h"

#include "core/Log.h"
#include "world/ElevatedTile.h"
#include "world/ElevationUpsampler.h"
#include "world/FlatGeometry.h"
#include "world/Scattering.h"

#include <cmath>

#include <algorithm>
#include <string>

namespace fsim::world {

namespace {

// Esri serves one fixed "Map data not yet available" tile wherever its imagery
// does not reach, which over open water is most of the deep ocean past about
// zoom 13. Among real tiles it is a light grey patch in a dark blue sea: the
// mismatched neighbours you see when flying over water.
//
// That tile is byte-identical everywhere it appears and perfectly neutral -
// R == G == B in every pixel, which a photograph never is, JPEG chroma alone
// sees to that - so it can be recognised exactly rather than by a heuristic
// that might catch snow or cloud, and repainted in the colour Esri's own deep
// water has (measured across its low-zoom ocean tiles).
constexpr float kOceanR = 8.0f, kOceanG = 57.0f, kOceanB = 74.0f;

/// True when every sampled pixel is neutral grey around the placeholder's own
/// brightness; `set` then paints the whole tile.
template <typename Array, typename Set>
bool repaintPlaceholder(Array& image, Set set) {
    const std::size_t count = image.valueCount();
    if (count < 256) return false;
    const std::size_t stride = std::max<std::size_t>(1, count / 512);
    std::size_t sampled = 0;
    double sum = 0.0;
    for (std::size_t i = 0; i < count; i += stride) {
        const auto& p = image.at(i);
        if (p.r != p.g || p.g != p.b) return false; // any colour at all: real imagery
        sum += p.r;
        ++sampled;
    }
    if (sampled == 0) return false;
    const double mean = sum / static_cast<double>(sampled);
    if (mean < 195.0 || mean > 215.0) return false; // grey, but not this grey
    for (std::size_t i = 0; i < count; ++i) set(image.at(i), i);
    return true;
}

vsg::ref_ptr<vsg::Data> replaceMissingImagery(vsg::ref_ptr<vsg::Data> data) {
    if (!data) return data;
    // A touch of deterministic variation, so the repainted water is not a dead
    // flat plane next to real tiles that vary by a few levels.
    const auto shade = [](std::size_t i, float base) {
        const std::size_t h = (i * 2654435761u) >> 13;
        return static_cast<std::uint8_t>(std::clamp(static_cast<int>(base) + static_cast<int>(h % 7u) - 3, 0, 255));
    };
    if (auto rgba = data.cast<vsg::ubvec4Array2D>()) {
        repaintPlaceholder(*rgba, [&](vsg::ubvec4& p, std::size_t i) {
            p.r = shade(i, kOceanR);
            p.g = shade(i, kOceanG);
            p.b = shade(i, kOceanB);
            p.a = 255;
        });
    } else if (auto rgb = data.cast<vsg::ubvec3Array2D>()) {
        repaintPlaceholder(*rgb, [&](vsg::ubvec3& p, std::size_t i) {
            p.r = shade(i, kOceanR);
            p.g = shade(i, kOceanG);
            p.b = shade(i, kOceanB);
        });
    }
    return data;
}

} // namespace

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
        tiles->maxLevel = std::min(19u, settings.maxLevel); // Esri serves 19 levels
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
    tiles->skirtRatio = settings.skirtRatio;

    tiles->imageLayerCallback = replaceMissingImagery;

    // VSG's tile shaders default to a Phong material with a specular lobe, which
    // puts a wet sheen on ground that is already lit in the satellite image.
    // Terrain is matte: keep the diffuse shading that makes relief readable and
    // drop the highlight.
    if (auto shaderSet = vsg::createPhongShaderSet(options)) {
        auto matte = vsg::PhongMaterialValue::create();
        matte->value().specular = vsg::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        matte->value().ambient = vsg::vec4(0.45f, 0.45f, 0.45f, 1.0f); // shadowed slopes stay legible
        shaderSet->getDescriptorBinding("material").data = matte;
        addAerialPerspective(*shaderSet);
        tiles->shaderSet = shaderSet;
    }

    std::vector<vsg::ref_ptr<vsg::ReaderWriter>> extraReaders;
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
        // and pop: either synthesise them from the deepest level or cap the
        // whole pyramid there.
        if (settings.upsampleElevation && tiles->maxLevel > settings.elevationMaxLevel)
            extraReaders.push_back(ElevationUpsampler::create(settings.elevationUrl, settings.elevationMaxLevel, encoding));
        else
            tiles->maxLevel = std::min(tiles->maxLevel, settings.elevationMaxLevel);
    }

    auto earth = vsg::TileDatabase::create();
    earth->settings = tiles;
    // Our reader: identical tiles, but culling bounds that include the relief.
    if (!readElevatedDatabase(*earth, options, extraReaders)) {
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
    settings.lit = true;                // shade with the sun, like the terrain it continues
    settings.aerialPerspective = true;  // ... and haze with it, or the cap stands out as the one clear thing
    settings.ambient = vsg::vec4(0.45f, 0.45f, 0.45f, 1.0f); // the same as the tiles, or it glows against them
    auto state = createFlatStateGroup(settings, options);
    if (!state) return {};

    // Concentric rings from the cap edge to the pole, a few metres above the
    // ellipsoid so the flat tiles at 85 deg do not z-fight with the ring. The
    // rings bunch towards the edge, where the colour has to do its work.
    const int rings = 16, segments = 128;
    // A flat white disc reads as a hole in the globe. Instead each cap starts
    // at the colour the imagery ends on and grades inwards: the Arctic from
    // open water to sea ice, the Antarctic from ice shelf to snow.
    // The edge colours are not invented: they are the mean of Esri's own
    // imagery in the last band it serves, 82.6-85 deg, measured off the tiles
    // - open water in the Arctic, ice shelf in the Antarctic. Matching the
    // colour the imagery ends on is what stops the cap reading as a disc
    // pasted over the globe. Inland it grades to sea ice and to snow.
    const auto capColour = [fromLatitudeDeg](double latDeg, double sign) {
        const double t = std::clamp((std::abs(latDeg) - fromLatitudeDeg) / (90.0 - fromLatitudeDeg), 0.0, 1.0);
        const float s = static_cast<float>(t * t * (3.0 - 2.0 * t)); // smoothstep: no banding at the seam
        // North: deep water, and specifically the same deep water the missing
        // ocean tiles are repainted with, because at the zooms where the cap
        // is on screen that is what borders it. Anything else and the two
        // blues meet in a visible circle. It lifts only slightly towards the
        // pole, for sea ice.
        const vsg::vec3 ocean(kOceanR / 255.0f, kOceanG / 255.0f, kOceanB / 255.0f);
        // Flat, not graded. A gradient across the disc reads as a dome and
        // gives the circle away even when the colours match at the rim; the
        // ocean it continues has no such shading either. The 1.03 is measured:
        // the cap came out about that much darker than its neighbours.
        const vsg::vec3 edge = sign > 0.0 ? ocean * 1.03f
                                          : vsg::vec3(0.957f, 0.969f, 1.000f); // measured off Esri: (244, 247, 255)
        const vsg::vec3 inner = edge;
        const vsg::vec3 c = edge + (inner - edge) * s;
        return vsg::vec4(c.r, c.g, c.b, 1.0f);
    };
    for (double sign : {1.0, -1.0}) {
        const std::uint32_t count = static_cast<std::uint32_t>(rings * segments * 6);
        auto vertices = vsg::vec3Array::create(count);
        auto normals = vsg::vec3Array::create(count);
        auto colors = vsg::vec4Array::create(count);
        // Under the imagery, not above it. The cap overlaps the last Mercator
        // tiles by a fraction of a degree, and a cap that sits above them
        // shows through in patches - a dotted ring around the edge. Sunk
        // below, the tiles win wherever they exist and the cap is seen only
        // past 85 deg, where there is no imagery at all. The southern cap
        // sits at the height of the polar plateau rather than at sea level.
        const double capAltitude = sign > 0.0 ? -30.0 : 2700.0;
        auto at = [&](double latDeg, double lonDeg) {
            const vsg::dvec3 p = ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(latDeg, lonDeg, capAltitude));
            return vsg::vec3(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
        };
        std::uint32_t i = 0;
        for (int r = 0; r < rings; ++r) {
            // Even spacing. Bunching the rings towards the seam was for a
            // gradient the cap no longer has, and uneven triangles showed as
            // faint concentric banding across a flat colour.
            const auto band = [&](int k) {
                return fromLatitudeDeg + (90.0 - fromLatitudeDeg) * static_cast<double>(k) / rings;
            };
            const double lat0 = band(r), lat1 = band(r + 1);
            for (int sg = 0; sg < segments; ++sg) {
                const double lon0 = 360.0 * sg / segments, lon1 = 360.0 * (sg + 1) / segments;
                // Two triangles per quad (the innermost ring degenerates to a fan at the pole; harmless).
                const std::pair<double, double> corners[6] = {{lat0, lon0}, {lat0, lon1}, {lat1, lon1}, {lat0, lon0}, {lat1, lon1}, {lat1, lon0}};
                for (const auto& [la, lo] : corners) {
                    const vsg::vec3 p = at(sign * la, lo);
                    vertices->set(i, p);
                    normals->set(i, vsg::normalize(p)); // the ellipsoid normal is close enough for a cap
                    colors->set(i, capColour(la, sign));
                    ++i;
                }
            }
        }
        state->addChild(createFlatDraw(vertices, colors, false, normals));
    }
    // Float vertices at ECEF magnitude 6.4e6 keep ~0.5 m precision: fine for a flat ice sheet.
    return state;
}

} // namespace fsim::world
