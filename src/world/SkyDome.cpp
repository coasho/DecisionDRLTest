#include "world/SkyDome.h"

#include "world/FlatGeometry.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kRings = 24;   // latitude bands (-90..+90)
constexpr int kSegments = 48; // longitude segments

vsg::vec3 mix(const vsg::vec3& a, const vsg::vec3& b, float t) { return a * (1.0f - t) + b * t; }
} // namespace

SkyDome::SkyDome(vsg::ref_ptr<const vsg::Options> options, const vsg::dvec3& sunDirectionEcef, double radiusM)
    : sun_(vsg::normalize(sunDirectionEcef)), radius_(radiusM) {
    // Triangle list over a lat/long sphere, local frame with +z = up. Colours
    // are per vertex and recomputed when the local up direction changes.
    const int quads = kRings * kSegments;
    vertices_ = vsg::vec3Array::create(static_cast<std::uint32_t>(quads * 6));
    colors_ = vsg::vec4Array::create(static_cast<std::uint32_t>(quads * 6), vsg::vec4(0.5f, 0.7f, 1.0f, 1.0f));

    auto point = [&](int ring, int seg) {
        const double lat = -kPi / 2.0 + kPi * ring / kRings;
        const double lon = 2.0 * kPi * seg / kSegments;
        return vsg::vec3(static_cast<float>(std::cos(lat) * std::cos(lon)), static_cast<float>(std::cos(lat) * std::sin(lon)),
                         static_cast<float>(std::sin(lat)));
    };
    std::uint32_t i = 0;
    for (int r = 0; r < kRings; ++r)
        for (int s = 0; s < kSegments; ++s) {
            const vsg::vec3 a = point(r, s), b = point(r, s + 1), c = point(r + 1, s + 1), d = point(r + 1, s);
            // Wind so the faces point *inward* (we are inside the sphere) with back-face culling off anyway.
            vertices_->set(i++, a); vertices_->set(i++, c); vertices_->set(i++, b);
            vertices_->set(i++, a); vertices_->set(i++, d); vertices_->set(i++, c);
        }

    FlatGeometrySettings settings;
    settings.depthTest = false;
    settings.depthWrite = false;
    settings.cullBackFaces = false;
    auto state = createFlatStateGroup(settings, options);
    transform_ = vsg::MatrixTransform::create();
    if (state) {
        state->addChild(createFlatDraw(vertices_, colors_, /*dynamic=*/true));
        transform_->addChild(state);
    }
}

void SkyDome::colour(const vsg::dvec3& upEcef) {
    // Sun elevation above the local horizon drives day/night; azimuth-relative
    // glow is computed per vertex in the local frame.
    const double sunElevation = vsg::dot(sun_, upEcef); // sin(elevation)
    const float day = static_cast<float>(std::clamp((sunElevation + 0.10) / 0.25, 0.0, 1.0)); // 1 = full day
    const vsg::vec3 zenithDay(0.20f, 0.42f, 0.85f), horizonDay(0.72f, 0.82f, 0.93f);
    const vsg::vec3 zenithNight(0.02f, 0.03f, 0.07f), horizonNight(0.08f, 0.09f, 0.14f);
    const vsg::vec3 below(0.30f, 0.33f, 0.38f); // only visible through gaps in the terrain
    const vsg::vec3 sunGlow(1.0f, 0.85f, 0.55f);

    // Local frame: build east/north from up so the sun can be expressed locally.
    vsg::dvec3 east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), upEcef);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    const vsg::dvec3 north = vsg::cross(upEcef, east);
    const vsg::vec3 sunLocal(static_cast<float>(vsg::dot(sun_, east)), static_cast<float>(vsg::dot(sun_, north)),
                             static_cast<float>(sunElevation));

    for (std::uint32_t i = 0; i < vertices_->size(); ++i) {
        const vsg::vec3& v = vertices_->at(i);
        vsg::vec3 c;
        if (v.z >= 0.0f) {
            const float t = std::pow(1.0f - v.z, 1.6f); // 0 at zenith, 1 at horizon
            c = mix(mix(zenithNight, zenithDay, day), mix(horizonNight, horizonDay, day), t);
            const float toSun = std::max(0.0f, vsg::dot(v, sunLocal));
            const float glow = std::pow(toSun, 32.0f) * 0.9f + std::pow(toSun, 4.0f) * 0.15f;
            c = mix(c, sunGlow, glow * day);
        } else {
            c = mix(below * 0.15f, below, day);
        }
        colors_->set(i, vsg::vec4(c.x, c.y, c.z, 1.0f));
    }
    colors_->dirty();
}

void SkyDome::update(const vsg::dvec3& eyeEcef) {
    const vsg::dvec3 up = vsg::normalize(eyeEcef);
    vsg::dvec3 east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    const vsg::dvec3 north = vsg::cross(up, east);

    // Columns: local x = east, y = north, z = up; scaled by the radius; at the eye.
    vsg::dmat4 m(east.x * radius_, east.y * radius_, east.z * radius_, 0.0,
                 north.x * radius_, north.y * radius_, north.z * radius_, 0.0,
                 up.x * radius_, up.y * radius_, up.z * radius_, 0.0,
                 eyeEcef.x, eyeEcef.y, eyeEcef.z, 1.0);
    transform_->matrix = m;

    if (!coloured_ || vsg::length(up - lastUp_) > 0.01) { // ~60 km of travel
        colour(up);
        lastUp_ = up;
        coloured_ = true;
    }
}

} // namespace fsim::world
