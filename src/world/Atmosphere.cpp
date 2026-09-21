#include "world/Atmosphere.h"

#include "world/FlatGeometry.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kRings = 32;    // latitude bands: the limb is a silhouette, so it wants a round edge
constexpr int kSegments = 64; // longitude segments
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kShellM = 80000.0;      ///< where the air has effectively run out
constexpr double kFadeInStartM = 90000.0; ///< eye altitude: above the shell, so the eye is never inside it
constexpr double kFadeInEndM = 260000.0;

/// Rayleigh-ish: the air scatters blue far more than red.
const vsg::vec3 kSkyBlue(0.34f, 0.56f, 1.00f);
/// Low sun through a long path loses the blue first, as at sunrise.
const vsg::vec3 kSunsetWarm(1.00f, 0.62f, 0.38f);

vsg::vec3 mix(const vsg::vec3& a, const vsg::vec3& b, float t) { return a * (1.0f - t) + b * t; }
} // namespace

Atmosphere::Atmosphere(vsg::ref_ptr<const vsg::Options> options, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
                       const vsg::dvec3& sunDirectionEcef)
    : sun_(vsg::normalize(sunDirectionEcef)) {
    root_ = vsg::Group::create();

    // A sphere in ECEF at the top of the air. The ellipsoid is flattened, so
    // the shell follows it rather than being a plain sphere, or the rim would
    // sit low over the poles.
    const int quads = kRings * kSegments;
    vertices_ = vsg::vec3Array::create(static_cast<std::uint32_t>(quads * 6));
    normals_ = vsg::vec3Array::create(static_cast<std::uint32_t>(quads * 6));
    colors_ = vsg::vec4Array::create(static_cast<std::uint32_t>(quads * 6), vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));

    const auto point = [&](int ring, int seg) {
        const double lat = -90.0 + 180.0 * ring / kRings;
        const double lon = -180.0 + 360.0 * seg / kSegments;
        const vsg::dvec3 p = ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(lat, lon, kShellM));
        return vsg::vec3(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    };
    std::uint32_t i = 0;
    for (int r = 0; r < kRings; ++r)
        for (int s = 0; s < kSegments; ++s) {
            const vsg::vec3 a = point(r, s), b = point(r, s + 1), c = point(r + 1, s + 1), d = point(r + 1, s);
            for (const vsg::vec3& v : {a, c, b, a, d, c}) {
                vertices_->set(i, v);
                normals_->set(i, vsg::normalize(v));
                ++i;
            }
        }

    FlatGeometrySettings settings;
    settings.blending = true;      // the whole point: it is a veil, not a surface
    settings.depthWrite = false;   // never hide what is behind it
    settings.cullBackFaces = false; // at the limb the ray crosses the shell twice, and both sides count
    if (auto state = createFlatStateGroup(settings, options)) {
        state->addChild(createFlatDraw(vertices_, colors_, /*dynamic=*/true, normals_));
        root_->addChild(state);
    }
}

void Atmosphere::colour(const vsg::dvec3& eyeEcef, float strength) {
    for (std::uint32_t i = 0; i < vertices_->size(); ++i) {
        const vsg::vec3& v = vertices_->at(i);
        const vsg::dvec3 p(v.x, v.y, v.z);
        const vsg::dvec3 n = vsg::normalize(p);

        // How much air the eye looks through here. Straight down at the shell
        // the ray crosses it once and briefly; towards the limb it runs along
        // the shell and crosses a great deal. That grazing angle is the whole
        // shape of the effect - it is what makes the rim, and it needs no
        // scattering integral to get right enough to believe.
        vsg::dvec3 toVertex = p - eyeEcef;
        const double range = vsg::length(toVertex);
        if (range < 1.0) continue;
        toVertex /= range;
        const double graze = 1.0 - std::abs(vsg::dot(toVertex, n));
        const float thickness = static_cast<float>(std::pow(std::clamp(graze, 0.0, 1.0), 3.0));

        // Lit by the sun, with a soft terminator: the rim should follow the
        // day side round and fade into the night, not stop at a hard edge.
        const double sunDot = vsg::dot(n, sun_);
        const float lit = static_cast<float>(std::clamp((sunDot + 0.25) / 0.45, 0.0, 1.0));
        // Where the sun is low over this patch its light has come the long way
        // through the air, so what is left of it is warm.
        const float grazingSun = static_cast<float>(std::clamp(1.0 - std::abs(sunDot) / 0.35, 0.0, 1.0));
        const vsg::vec3 tint = mix(kSkyBlue, kSunsetWarm, grazingSun * 0.7f);

        const float alpha = std::clamp(thickness * lit * strength * 0.95f, 0.0f, 1.0f);
        colors_->set(i, vsg::vec4(tint.x, tint.y, tint.z, alpha));
    }
    colors_->dirty();
}

void Atmosphere::update(const vsg::dvec3& eyeEcef) {
    // Out of sight until the eye is clear of the shell; below that the sky
    // dome is painting the same air from the inside.
    const double altitude = vsg::length(eyeEcef) - kEarthRadiusM;
    const float strength =
        static_cast<float>(std::clamp((altitude - kFadeInStartM) / (kFadeInEndM - kFadeInStartM), 0.0, 1.0));
    if (strength <= 0.0f && lastStrength_ <= 0.0f) return; // nothing drawn, nothing to recompute

    // Recolour when the view has moved enough to show: a degree or so around
    // the globe, or a visible step in the fade.
    const double moved = vsg::length(eyeEcef - lastEye_);
    if (coloured_ && moved < 0.01 * vsg::length(eyeEcef) && std::abs(strength - lastStrength_) < 0.02) return;
    colour(eyeEcef, strength);
    lastEye_ = eyeEcef;
    lastStrength_ = strength;
    coloured_ = true;
}

} // namespace fsim::world
