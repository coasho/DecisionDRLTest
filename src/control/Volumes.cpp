// Operational volumes' geometry (docs/flight-autonomy.md, 4.45; A-GRA's OpVolumeType): see Volumes.h.
#include "control/Volumes.h"

#include "control/Zones.h"
#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control::volumes {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
/// As far along an axis without end as the host looks into it (a cone's or a cylinder's).
constexpr double kReachM = 2000.0;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }
bool positive(double v) noexcept { return std::isfinite(v) && v > 0.0; }

/// Its axes (rows: its x, y and z) in north-east-down from its yaw, pitch and roll (aerospace, z-y-x).
void axesOf(double yaw, double pitch, double roll, double (&m)[3][3]) noexcept {
    const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch), cr = std::cos(roll), sr = std::sin(roll);
    m[0][0] = cp * cy, m[0][1] = cp * sy, m[0][2] = -sp;
    m[1][0] = sr * sp * cy - cr * sy, m[1][1] = sr * sp * sy + cr * cy, m[1][2] = sr * cp;
    m[2][0] = cr * sp * cy + sr * sy, m[2][1] = cr * sp * sy - sr * cy, m[2][2] = cr * cp;
}

/// A geocentric volume's longitude within its span, from the least clockwise to the most.
bool withinLongitudes(const MustFlyArea& a, double lon) noexcept {
    return geo::wrapTwoPi(lon - a.longitudeMinRad) <= geo::wrapTwoPi(a.longitudeMaxRad - a.longitudeMinRad);
}

} // namespace

int fault(const OpVolume& v, bool frameKnown) noexcept {
    if (isHold(v.shape) || !code(v.shape, static_cast<double>(VolumeShape::Count))) return 0;
    const auto shape = static_cast<VolumeShape>(static_cast<int>(v.shape));
    const bool geocentric = shape == VolumeShape::Geocentric, framed = !isHold(v.frame);
    const auto given = [](double x) { return !isHold(x); };
    // its frame, and its velocity: both ways or neither (down alone never), finite; a frame moves it instead
    if (framed && (geocentric || !(v.frame == std::floor(v.frame) && v.frame >= 1.0 && frameKnown))) return 5;
    if (!code(v.frameRotation, static_cast<double>(FrameRotation::Count)) || (!framed && given(v.frameRotation))) return 5;
    if (isHold(v.northMs) != isHold(v.eastMs) || (isHold(v.northMs) && given(v.downMs)) || !finiteOr(v.northMs) || !finiteOr(v.eastMs) ||
        !finiteOr(v.downMs) || !finiteOr(v.timeS))
        return 6;
    if ((given(v.northMs) && (framed || geocentric)) || (isHold(v.northMs) && given(v.timeS))) return 6;
    if (!code(v.altitudeReference, static_cast<double>(AltitudeReference::Count))) return geocentric ? 4 : 1;
    // its point: on the Earth (with its altitude), or in the frame (its altitude left out: the frame's); a geocentric one has none
    const bool place = given(v.latitudeRad) || given(v.longitudeRad), plane = given(v.xM) || given(v.yM);
    if (geocentric) {
        if (place || plane || given(v.altitudeM)) return 1;
    } else if (framed) {
        if (place || !std::isfinite(v.xM) || !std::isfinite(v.yM) || !finiteOr(v.altitudeM)) return 1;
        if (given(v.altitudeReference) && isHold(v.altitudeM)) return 1;
    } else if (plane || !std::isfinite(v.latitudeRad) || !std::isfinite(v.longitudeRad) || std::abs(v.latitudeRad) > 0.5 * kPi ||
               !std::isfinite(v.altitudeM)) {
        return 1;
    }
    // its dimensions: its shape's, above 0 (a half angle below a quarter turn); another's, none
    const double dims[] = {v.radiusM, v.semiAxisAM, v.semiAxisBM, v.semiAxisCM, v.lengthM, v.halfAngleRad, v.lengthHalfAngleRad, v.widthHalfAngleRad, v.rangeM};
    auto only = [&](std::initializer_list<int> mine) {
        for (int i = 0; i < 9; ++i)
            if (given(dims[i]) && std::find(mine.begin(), mine.end(), i) == mine.end()) return false;
        return true;
    };
    auto angle = [](double x) { return positive(x) && x < 0.5 * kPi; };
    auto optional = [](double x) { return isHold(x) || positive(x); };
    switch (shape) {
    case VolumeShape::Sphere:
    case VolumeShape::Dome:
        if (!only({0}) || !positive(v.radiusM)) return 2;
        break;
    case VolumeShape::Ellipsoid:
        if (!only({1, 2, 3}) || !positive(v.semiAxisAM) || !positive(v.semiAxisBM) || !positive(v.semiAxisCM)) return 2;
        break;
    case VolumeShape::Cylinder:
        if (!only({0, 4}) || !positive(v.radiusM) || !optional(v.lengthM)) return 2;
        break;
    case VolumeShape::Cone:
        if (!only({5, 8}) || !angle(v.halfAngleRad) || !optional(v.rangeM)) return 2;
        break;
    case VolumeShape::RectangularCone:
        if (!only({6, 7, 8}) || !angle(v.lengthHalfAngleRad) || !angle(v.widthHalfAngleRad) || !optional(v.rangeM)) return 2;
        break;
    default:
        if (!only({})) return 2;
        break;
    }
    // its attitude: an ellipsoid's, a cylinder's or a cone's, finite
    const bool turns = shape == VolumeShape::Ellipsoid || shape == VolumeShape::Cylinder || shape == VolumeShape::Cone || shape == VolumeShape::RectangularCone;
    for (const double a : {v.yawRad, v.pitchRad, v.rollRad})
        if ((given(a) && !turns) || !finiteOr(a)) return 3;
    // a geocentric volume's bounds: its latitudes in order within a quarter turn, its longitudes within half a turn, its band in
    // order - and another's none
    const double bounds[] = {v.latitudeMinRad, v.latitudeMaxRad, v.longitudeMinRad, v.longitudeMaxRad, v.altitudeMinM, v.altitudeMaxM};
    if (!geocentric) {
        for (const double b : bounds)
            if (given(b)) return 4;
        return -1;
    }
    for (const double b : bounds)
        if (!finiteOr(b)) return 4;
    if (isHold(v.latitudeMinRad) || isHold(v.latitudeMaxRad) || isHold(v.longitudeMinRad) || isHold(v.longitudeMaxRad)) return 4;
    if (std::abs(v.latitudeMinRad) > 0.5 * kPi || std::abs(v.latitudeMaxRad) > 0.5 * kPi || v.latitudeMinRad >= v.latitudeMaxRad) return 4;
    if (std::abs(v.longitudeMinRad) > kPi || std::abs(v.longitudeMaxRad) > kPi || v.longitudeMinRad == v.longitudeMaxRad) return 4;
    if (given(v.altitudeMinM) && given(v.altitudeMaxM) && v.altitudeMinM > v.altitudeMaxM) return 4;
    if (given(v.altitudeReference) && isHold(v.altitudeMinM) && isHold(v.altitudeMaxM)) return 4;
    return -1;
}

void layOut(const OpVolume& v, const FrameSpec* spec, double nowS, MustFlyArea& a) noexcept {
    a = MustFlyArea{};
    a.volume = static_cast<VolumeShape>(static_cast<int>(v.shape));
    a.framed = spec != nullptr;
    if (spec)
        a.frame = *spec, a.frameId = static_cast<FrameId>(v.frame), a.frameXM = v.xM, a.frameYM = v.yM,
        a.rotation = isHold(v.frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(v.frameRotation));
    a.northMs = orHold(v.northMs, 0.0), a.eastMs = orHold(v.eastMs, 0.0), a.downMs = orHold(v.downMs, 0.0), a.timeS = orHold(v.timeS, nowS);
    a.altitudeReference = isHold(v.altitudeReference) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(v.altitudeReference));
    a.pointAltitudeM = v.altitudeM; // (kHold in a frame: the frame's)
    a.radiusM = orHold(v.radiusM, 0.0);
    a.semiAxesM[0] = orHold(v.semiAxisAM, 0.0), a.semiAxesM[1] = orHold(v.semiAxisBM, 0.0), a.semiAxesM[2] = orHold(v.semiAxisCM, 0.0);
    a.lengthM = orHold(v.lengthM, kInf), a.rangeM = orHold(v.rangeM, kInf);
    a.halfAngleRad = orHold(v.halfAngleRad, 0.0), a.lengthHalfAngleRad = orHold(v.lengthHalfAngleRad, 0.0), a.widthHalfAngleRad = orHold(v.widthHalfAngleRad, 0.0);
    axesOf(orHold(v.yawRad, 0.0), orHold(v.pitchRad, 0.0), orHold(v.rollRad, 0.0), a.axes);
    if (a.volume == VolumeShape::Geocentric) { // (laid out at its bounds' middle, its altitude that of the sea)
        a.latitudeMinRad = v.latitudeMinRad, a.latitudeMaxRad = v.latitudeMaxRad, a.longitudeMinRad = v.longitudeMinRad, a.longitudeMaxRad = v.longitudeMaxRad;
        a.altitudeMinM = v.altitudeMinM, a.altitudeMaxM = v.altitudeMaxM;
        a.latitudeRad = 0.5 * (v.latitudeMinRad + v.latitudeMaxRad);
        a.longitudeRad = geo::wrapPi(v.longitudeMinRad + 0.5 * geo::wrapTwoPi(v.longitudeMaxRad - v.longitudeMinRad));
        a.pointAltitudeM = 0.0;
        return;
    }
    if (!spec) a.latitudeRad = v.latitudeRad, a.longitudeRad = geo::wrapPi(v.longitudeRad);
}

double pointAltitude(const MustFlyArea& a, const FramePose* pose, double nowS) noexcept {
    if (isHold(a.pointAltitudeM)) return pose ? pose->altitudeMslM : 0.0;
    return a.pointAltitudeM - a.downMs * (nowS - a.timeS);
}

bool contains(const MustFlyArea& a, double x, double y, double downM) noexcept {
    if (a.volume == VolumeShape::Geocentric) {
        double lat = 0.0, lon = 0.0;
        zones::fromPlane(a, nullptr, a.timeS, x, y, lat, lon);
        const double h = -downM;
        return lat >= a.latitudeMinRad && lat <= a.latitudeMaxRad && withinLongitudes(a, lon) && (isHold(a.altitudeMinM) || h >= a.altitudeMinM) &&
               (isHold(a.altitudeMaxM) || h <= a.altitudeMaxM);
    }
    const double r2 = x * x + y * y + downM * downM;
    const double u = x * a.axes[0][0] + y * a.axes[0][1] + downM * a.axes[0][2];
    const double v = x * a.axes[1][0] + y * a.axes[1][1] + downM * a.axes[1][2];
    const double w = x * a.axes[2][0] + y * a.axes[2][1] + downM * a.axes[2][2];
    switch (a.volume) {
    case VolumeShape::Sphere: return r2 <= a.radiusM * a.radiusM;
    case VolumeShape::Dome: return downM <= 0.0 && r2 <= a.radiusM * a.radiusM;
    case VolumeShape::Ellipsoid: {
        const double p = u / a.semiAxesM[0], q = v / a.semiAxesM[1], s = w / a.semiAxesM[2];
        return p * p + q * q + s * s <= 1.0;
    }
    case VolumeShape::Cylinder: return u >= 0.0 && u <= a.lengthM && v * v + w * w <= a.radiusM * a.radiusM;
    case VolumeShape::Cone: return u > 0.0 && std::hypot(v, w) <= u * std::tan(a.halfAngleRad) && r2 <= a.rangeM * a.rangeM;
    case VolumeShape::RectangularCone:
        return u > 0.0 && std::abs(v) <= u * std::tan(a.lengthHalfAngleRad) && std::abs(w) <= u * std::tan(a.widthHalfAngleRad) && r2 <= a.rangeM * a.rangeM;
    default: return false;
    }
}

void inner(const MustFlyArea& a, double& x, double& y, double& downM) noexcept {
    x = y = downM = 0.0;
    double along = 0.0; // (along its x)
    switch (a.volume) {
    case VolumeShape::Dome: downM = -0.5 * a.radiusM; return;
    case VolumeShape::Cylinder: along = std::isfinite(a.lengthM) ? 0.5 * a.lengthM : std::max(a.radiusM, 100.0); break;
    case VolumeShape::Cone:
    case VolumeShape::RectangularCone: along = std::isfinite(a.rangeM) ? 0.5 * a.rangeM : 0.5 * kReachM; break;
    case VolumeShape::Geocentric: { // (its band's middle; one way open, 100 m in from its edge; neither, the sea)
        const double lo = a.altitudeMinM, hi = a.altitudeMaxM;
        const double h = !isHold(lo) && !isHold(hi) ? 0.5 * (lo + hi) : !isHold(lo) ? lo + 100.0 : !isHold(hi) ? hi - 100.0 : 0.0;
        downM = -h;
        return;
    }
    default: return;
    }
    x = along * a.axes[0][0], y = along * a.axes[0][1], downM = along * a.axes[0][2];
}

double extent(const MustFlyArea& a) noexcept {
    switch (a.volume) {
    case VolumeShape::Sphere:
    case VolumeShape::Dome: return a.radiusM;
    case VolumeShape::Ellipsoid: return std::max({a.semiAxesM[0], a.semiAxesM[1], a.semiAxesM[2]});
    case VolumeShape::Cylinder: return std::max(a.radiusM, std::isfinite(a.lengthM) ? 0.5 * a.lengthM : kReachM);
    case VolumeShape::Cone:
    case VolumeShape::RectangularCone: return std::isfinite(a.rangeM) ? a.rangeM : kReachM;
    case VolumeShape::Geocentric: {
        const double north = 0.5 * (a.latitudeMaxRad - a.latitudeMinRad) * geo::kEarthRadiusM;
        const double east = 0.5 * geo::wrapTwoPi(a.longitudeMaxRad - a.longitudeMinRad) * geo::kEarthRadiusM * std::cos(a.latitudeRad);
        return std::hypot(north, east);
    }
    default: return 0.0;
    }
}

} // namespace fsim::control::volumes
