// Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame and its relative points): a point in a frame
// as a place, the frame as it is at a time. Asked between steps, never stepped.
#include "fsim/Frames.h"

#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kR = geo::kEarthRadiusM;
constexpr double kPi = 3.14159265358979323846;

/// `north` and `east` metres from a place in the plane square to the vertical there, and `up` metres above it: the
/// place under that point, on the sphere, and its height.
GeoPoint inPlane(double lat, double lon, double altitudeM, double north, double east, double up) noexcept {
    const double cl = std::cos(lat), sl = std::sin(lat), co = std::cos(lon), so = std::sin(lon);
    const double r = kR + altitudeM + up;
    // the origin's radius out, and north and east along the plane
    const double p[3] = {r * cl * co - north * sl * co - east * so, r * cl * so - north * sl * so + east * co, r * sl + north * cl};
    const double length = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    return {std::asin(std::clamp(p[2] / length, -1.0, 1.0)), std::atan2(p[1], p[0]), length - kR};
}

} // namespace

FramePose carried(const FramePose& pose, double seconds) noexcept {
    FramePose out = pose;
    if (!std::isfinite(seconds) || seconds == 0.0) return out;
    // over the ground along the great circle its track gives, at its speed; up or down at its vertical speed
    const double north = pose.northMs * seconds, east = pose.eastMs * seconds;
    if (const double d = std::hypot(north, east); d > 0.0) geo::destination(pose.latitudeRad, pose.longitudeRad, std::atan2(east, north), d, out.latitudeRad, out.longitudeRad);
    out.altitudeMslM = pose.altitudeMslM - pose.downMs * seconds;
    return out;
}

FramePose vehiclePose(const sim::VehicleState& s) noexcept {
    FramePose pose;
    pose.latitudeRad = s.latitudeRad, pose.longitudeRad = s.longitudeRad, pose.altitudeMslM = s.altitudeMslM;
    pose.rollRad = s.eulerRad[0], pose.pitchRad = s.eulerRad[1], pose.yawRad = s.eulerRad[2];
    pose.northMs = s.velocityNedMs[0], pose.eastMs = s.velocityNedMs[1], pose.downMs = s.velocityNedMs[2];
    return pose;
}

FramePose framePose(const FrameSpec& spec, double timeS) noexcept {
    FramePose pose;
    pose.latitudeRad = spec.latitudeRad, pose.longitudeRad = spec.longitudeRad, pose.altitudeMslM = spec.altitudeMslM;
    pose.yawRad = spec.yawRad, pose.pitchRad = spec.pitchRad, pose.rollRad = spec.rollRad;
    if (spec.origin != FrameOrigin::Moving) return pose;
    pose.northMs = spec.northMs, pose.eastMs = spec.eastMs, pose.downMs = spec.downMs;
    return carried(pose, timeS - spec.timeS);
}

GeoPoint framePoint(const FramePose& pose, const FrameOffset& offset) noexcept {
    // the offsets turned into north, east and down
    double north = offset.x, east = offset.y, down = offset.z;
    auto turn = [&](double angle) {
        const double c = std::cos(angle), s = std::sin(angle);
        north = offset.x * c - offset.y * s, east = offset.x * s + offset.y * c;
    };
    switch (offset.rotation) {
    case FrameRotation::Yaw: turn(pose.yawRad); break;
    case FrameRotation::Heading: turn(std::hypot(pose.northMs, pose.eastMs) > 0.1 ? std::atan2(pose.eastMs, pose.northMs) : pose.yawRad); break;
    case FrameRotation::Attitude: {
        // the body's axes over north, east and down (its roll, pitch and yaw)
        const double cf = std::cos(pose.rollRad), sf = std::sin(pose.rollRad), ct = std::cos(pose.pitchRad), st = std::sin(pose.pitchRad);
        const double cp = std::cos(pose.yawRad), sp = std::sin(pose.yawRad);
        const double x = offset.x, y = offset.y, z = offset.z;
        north = ct * cp * x + (sf * st * cp - cf * sp) * y + (cf * st * cp + sf * sp) * z;
        east = ct * sp * x + (sf * st * sp + cf * cp) * y + (cf * st * sp - sf * cp) * z;
        down = -st * x + sf * ct * y + cf * ct * z;
        break;
    }
    default: break;
    }
    // laid out on the Earth
    GeoPoint out{pose.latitudeRad, pose.longitudeRad, pose.altitudeMslM - down};
    switch (offset.offsets) {
    case FrameOffsets::GreatCircle:
        if (const double d = std::hypot(north, east); d > 0.0) geo::destination(pose.latitudeRad, pose.longitudeRad, std::atan2(east, north), d, out.latitudeRad, out.longitudeRad);
        return out;
    case FrameOffsets::Rhumb: {
        // north along the meridian; east as the rhumb line between the latitudes has it (the parallel's, east-west)
        out.latitudeRad = std::clamp(pose.latitudeRad + north / kR, -0.5 * kPi, 0.5 * kPi);
        const double dPsi = std::log(std::tan(0.25 * kPi + 0.5 * std::clamp(out.latitudeRad, -1.5, 1.5)) /
                                     std::tan(0.25 * kPi + 0.5 * std::clamp(pose.latitudeRad, -1.5, 1.5)));
        const double q = std::abs(dPsi) > 1e-12 ? (out.latitudeRad - pose.latitudeRad) / dPsi : std::cos(pose.latitudeRad);
        out.longitudeRad = geo::wrapPi(pose.longitudeRad + east / (kR * q));
        return out;
    }
    default:
        return inPlane(pose.latitudeRad, pose.longitudeRad, pose.altitudeMslM, north, east, -down);
    }
}

} // namespace fsim::control
