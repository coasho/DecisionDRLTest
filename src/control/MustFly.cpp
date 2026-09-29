// A must fly (docs/flight-autonomy.md, 4.42 to 4.45; A-GRA's MUST_FLY): its location laid out as a route from where the
// aircraft is - the points it approaches through where it has a window of bearings to come from, then the location, flown
// over; a zone's or a volume's point a little inside it; a corridor's vertices - as it is commanded or updated, and flown by
// the route's follower. In a file of its own: the host's checks and the behaviour.
#include "control/CapabilityHost.h"

#include "control/Checks.h"
#include "control/Lines.h"
#include "control/Route.h"
#include "control/Runtime.h"
#include "control/Volumes.h"
#include "control/Zones.h"
#include "core/Geodesy.h"
#include "fsim/ControllerRegistry.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
/// How far above a vehicle a must fly flies over it, given no altitude: 500 ft (4.42).
constexpr double kOverEntityM = 152.4;
/// A turn onto an approach's last leg sharper than this is split by a point abeam (4.42).
constexpr double kMostTurnRad = 120.0 * kDeg;
/// A corridor is entered from within this of its first segment's line, behind its first vertex, given no window (4.44).
constexpr double kEntryRad = 10.0 * kDeg;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }
/// An id as a field carries it: whole, from 1, within the 2^53 a double holds exactly.
bool anId(double v) noexcept { return v == std::floor(v) && v >= 1.0 && v <= 9007199254740992.0; }

/// `r` with what the check said the answer is about, as CapabilityHost.cpp's.
CommandResult about(CommandResult r, const CommandResult& detail) noexcept {
    r.index = detail.index;
    r.constraint = detail.constraint;
    r.from = detail.from;
    r.to = detail.to;
    return r;
}

/// The approach to a location from within its window of bearings (A-GRA's IngressConstraint: the bearings from the
/// location to the aircraft, `minRad` clockwise to `maxRad`), the aircraft `northA`, `eastA` from it: none where it comes
/// from within the window now; else the points it flies through, north and east of the location - the last `lengthM` out
/// along a bearing within the window (its nearer edge, moved in by 5 degrees or half the window), so the leg from there to
/// the location is flown on that bearing's reciprocal, and a turn onto that leg of more than 120 degrees split by a point
/// `2.5 radiusM` to the aircraft's side of it. Their count into `count`.
void approach(double northA, double eastA, double minRad, double maxRad, double lengthM, double radiusM, double out[2][2], int& count) noexcept {
    count = 0;
    const double width = geo::wrapTwoPi(maxRad - minRad); // (0: one bearing)
    const double now = geo::wrapTwoPi(std::atan2(eastA, northA));
    if (geo::wrapTwoPi(now - minRad) <= width) return; // (within it: straight in)
    const double margin = std::min(5.0 * kDeg, 0.5 * width);
    const double bearing = geo::wrapTwoPi(minRad - now) <= geo::wrapTwoPi(now - maxRad) ? minRad + margin : maxRad - margin;
    const double un = std::cos(bearing), ue = std::sin(bearing);
    const double n2 = lengthM * un, e2 = lengthM * ue;
    // the turn onto the last leg there, from the leg the aircraft flies to it
    const double in = std::atan2(e2 - eastA, n2 - northA);
    if (std::abs(geo::wrapPi(bearing + kPi - in)) > kMostTurnRad) {
        const double side = un * eastA - ue * northA >= 0.0 ? 1.0 : -1.0; // (the aircraft right of the bearing out, or left)
        out[count][0] = n2 - side * 2.5 * radiusM * ue, out[count][1] = e2 + side * 2.5 * radiusM * un, ++count;
    }
    out[count][0] = n2, out[count][1] = e2, ++count;
}

/// A must fly's altitude against a band (4.43, 4.44): the altitude given, within it and in its reference (left out, the
/// band's) - else the field at fault, 3 (outside it) or 4 (another reference); given none, the aircraft's (`own`, in the
/// band's reference) held within it - a tenth of it in, 30 m at most and never past its middle - or kHold where it is
/// within it already. -1 where it is fine; no band, the altitude and reference given.
int inBand(double given, double givenReference, double lo, double hi, AltitudeReference bandReference, double own, double& altitude,
           double& reference) noexcept {
    altitude = given, reference = givenReference;
    if (isHold(lo) && isHold(hi)) return -1;
    const double band = static_cast<double>(bandReference);
    if (!isHold(given)) {
        if (!isHold(givenReference) && givenReference != band) return 4;
        if ((!isHold(lo) && given < lo) || (!isHold(hi) && given > hi)) return 3;
        reference = band;
        return -1;
    }
    const double in = !isHold(lo) && !isHold(hi) ? std::min(0.1 * (hi - lo), 30.0) : 30.0;
    if (!isHold(lo) && own < lo) altitude = !isHold(hi) ? std::min(lo + in, 0.5 * (lo + hi)) : lo + in;
    else if (!isHold(hi) && own > hi) altitude = !isHold(lo) ? std::max(hi - in, 0.5 * (lo + hi)) : hi - in;
    if (!isHold(altitude)) reference = band;
    return -1;
}

/// A width left out: none that way.
double orNone(double widthM) noexcept { return isHold(widthM) ? std::numeric_limits<double>::infinity() : widthM; }

} // namespace

// --- The host: laid out and checked ----------------------------------------------------------------

void mergeMustFly(MustFlyCommand& dst, const MustFlyCommand& src) noexcept {
    if (!isHold(src.location) && src.location != dst.location) dst.latitudeRad = dst.longitudeRad = dst.target = kHold;
    const double* from[] = {&src.location, &src.latitudeRad,   &src.longitudeRad,  &src.altitudeM, &src.altitudeReference,
                            &src.target,   &src.ingressMinRad, &src.ingressMaxRad, &src.speed,     &src.speedReference};
    double* to[] = {&dst.location, &dst.latitudeRad,   &dst.longitudeRad,  &dst.altitudeM, &dst.altitudeReference,
                    &dst.target,   &dst.ingressMinRad, &dst.ingressMaxRad, &dst.speed,     &dst.speedReference};
    for (std::size_t i = 0; i < std::size(from); ++i)
        if (!isHold(*from[i])) *to[i] = *from[i];
}

Reason CapabilityHost::layOutZone(const OpZone& zone, MustFlyArea& out, CommandResult& detail) const {
    FrameSpec spec;
    FramePose pose;
    const bool known = isHold(zone.frame) ||
                       (zone.frame == std::floor(zone.frame) && zone.frame >= 1.0 && zone.frame <= 9007199254740992.0 && sessionView_ &&
                        sessionView_->frame(static_cast<FrameId>(zone.frame), spec, pose));
    if (const int f = zones::fault(zone, known); f >= 0) {
        detail.index = static_cast<std::int16_t>(10 + f); // (a zone's fields named after the must fly's ten)
        return Reason::InvalidParameter;
    }
    zones::layOut(zone, isHold(zone.frame) ? nullptr : &spec, sessionView_ ? sessionView_->simTimeS() : 0.0, out);
    return Reason::None;
}

Reason CapabilityHost::layOutLine(const OpLine& line, MustFlyArea& out, CommandResult& detail) const {
    FrameSpec spec;
    FramePose pose;
    const bool known = isHold(line.frame) ||
                       (line.frame == std::floor(line.frame) && line.frame >= 1.0 && line.frame <= 9007199254740992.0 && sessionView_ &&
                        sessionView_->frame(static_cast<FrameId>(line.frame), spec, pose));
    if (const int f = lines::fault(line, known); f >= 0) {
        detail.index = static_cast<std::int16_t>(10 + f); // (a line's fields named after the must fly's ten)
        return Reason::InvalidParameter;
    }
    lines::layOut(line, isHold(line.frame) ? nullptr : &spec, sessionView_ ? sessionView_->simTimeS() : 0.0, out);
    return Reason::None;
}

Reason CapabilityHost::layOutVolume(const OpVolume& volume, MustFlyArea& out, CommandResult& detail) const {
    FrameSpec spec;
    FramePose pose;
    const bool known = isHold(volume.frame) ||
                       (volume.frame == std::floor(volume.frame) && volume.frame >= 1.0 && volume.frame <= 9007199254740992.0 && sessionView_ &&
                        sessionView_->frame(static_cast<FrameId>(volume.frame), spec, pose));
    if (const int f = volumes::fault(volume, known); f >= 0) {
        detail.index = static_cast<std::int16_t>(10 + f); // (a volume's fields named after the must fly's ten)
        return Reason::InvalidParameter;
    }
    volumes::layOut(volume, isHold(volume.frame) ? nullptr : &spec, sessionView_ ? sessionView_->simTimeS() : 0.0, out);
    return Reason::None;
}

Reason CapabilityHost::prepareMustFly(MustFlyCommand& c, const sim::VehicleState& state, CheckLog& log, const MustFlyArea* given) {
    CommandResult& detail = log.result;
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    // its fields whole, finite and in range; a location's own given, and nothing another's is
    if (!code(c.location, static_cast<double>(MustFlyLocation::Count))) return bad(0);
    double* fields[] = {&c.latitudeRad, &c.longitudeRad, &c.altitudeM, &c.altitudeReference, &c.target, &c.ingressMinRad, &c.ingressMaxRad, &c.speed,
                        &c.speedReference};
    for (std::size_t i = 0; i < std::size(fields); ++i)
        if (!finiteOr(*fields[i])) return bad(static_cast<std::int16_t>(i + 1));
    if (!code(c.altitudeReference, static_cast<double>(AltitudeReference::Count))) return bad(4);
    if (!code(c.speedReference, static_cast<double>(SpeedReference::Count))) return bad(9);
    if (!isHold(c.speed) && c.speed < 0.0) return bad(8);
    if (isHold(c.ingressMinRad) != isHold(c.ingressMaxRad)) return bad(isHold(c.ingressMinRad) ? 6 : 7);
    if (!isHold(c.ingressMinRad) && std::abs(c.ingressMinRad) > kPi) return bad(6); // (A-GRA's AngleType)
    if (!isHold(c.ingressMaxRad) && std::abs(c.ingressMaxRad) > kPi) return bad(7);
    c.location = orHold(c.location, 0.0);
    const auto kind = static_cast<MustFlyLocation>(static_cast<int>(c.location));
    const bool named = kind == MustFlyLocation::Entity || kind == MustFlyLocation::OpPoint || kind == MustFlyLocation::OpZone ||
                       kind == MustFlyLocation::OpLine || kind == MustFlyLocation::OpVolume;
    if (kind != MustFlyLocation::Point && (!isHold(c.latitudeRad) || !isHold(c.longitudeRad))) return bad(isHold(c.latitudeRad) ? 2 : 1);
    if (!named && !isHold(c.target)) return bad(5);
    if (named && (isHold(c.target) || !anId(c.target))) return bad(5);

    // the location, a point flown over (EndPointKind::Waypoint), at its altitude and speed - or an area to enter
    Waypoint at;
    at.kind = static_cast<double>(EndPointKind::Waypoint);
    at.altitudeM = c.altitudeM, at.altitudeReference = c.altitudeReference, at.speed = c.speed, at.speedReference = c.speedReference;
    double ingressMin = c.ingressMinRad, ingressMax = c.ingressMaxRad;
    FramePose pose;       // (a location in a frame: the frame now)
    bool framed = false;
    MustFlyArea area;     // (a zone's, a corridor's or a volume's: 4.43 to 4.45)
    switch (kind) {
    case MustFlyLocation::Point:
        if (isHold(c.latitudeRad) || std::abs(c.latitudeRad) > 0.5 * kPi) return bad(1);
        if (isHold(c.longitudeRad)) return bad(2);
        at.latitudeRad = c.latitudeRad, at.longitudeRad = geo::wrapPi(c.longitudeRad);
        break;
    case MustFlyLocation::Entity: {
        // another vehicle, flown over in its own frame (kVehicleFrames): at the altitude given, else as far above it as the
        // aircraft is now - no less than 500 ft - followed as it climbs or descends
        const double id = c.target;
        FrameSpec spec;
        if (id == static_cast<double>(vehicle_) || !sessionView_ || !sessionView_->frame(kVehicleFrames + static_cast<FrameId>(id), spec, pose))
            return bad(5);
        if (!isHold(c.altitudeReference) && isHold(c.altitudeM)) return bad(4); // (a reference to read nothing in)
        at.frame = static_cast<double>(kVehicleFrames + static_cast<FrameId>(id));
        if (isHold(c.altitudeM)) at.frameZM = -std::max(state.altitudeMslM - pose.altitudeMslM, kOverEntityM);
        framed = true;
        break;
    }
    case MustFlyLocation::OpPoint: {
        OpPoint p;
        if (!sessionView_ || !sessionView_->opPoint(static_cast<OpPointId>(c.target), p)) {
            detail.index = 5;
            return Reason::UnknownGeometry;
        }
        at.latitudeRad = p.latitudeRad, at.longitudeRad = isHold(p.longitudeRad) ? kHold : geo::wrapPi(p.longitudeRad);
        at.frame = p.frame, at.frameRotation = p.frameRotation, at.frameOffsets = p.frameOffsets;
        at.frameXM = p.frameXM, at.frameYM = p.frameYM, at.frameZM = p.frameZM;
        if (isHold(c.altitudeM) && isHold(c.altitudeReference)) at.altitudeM = p.altitudeM, at.altitudeReference = p.altitudeReference;
        if (isHold(ingressMin)) ingressMin = p.ingressMinRad, ingressMax = p.ingressMaxRad;
        if (!isHold(p.frame)) {
            FrameSpec spec;
            if (!sessionView_->frame(static_cast<FrameId>(p.frame), spec, pose)) {
                detail.index = 5;
                return Reason::UnknownGeometry; // (its frame gone)
            }
            framed = true;
        }
        break;
    }
    case MustFlyLocation::Zone: // (given with it, as it was laid out then)
        if (!given || given->shape == ZoneShape::Count) return bad(0);
        area = *given;
        break;
    case MustFlyLocation::OpZone: { // (the world's, as it is now)
        const OpZone* z = sessionView_ ? sessionView_->opZone(static_cast<OpZoneId>(c.target)) : nullptr;
        CommandResult ignored;
        if (!z || layOutZone(*z, area, ignored) != Reason::None) {
            detail.index = 5;
            return Reason::UnknownGeometry; // (none by that id, or its frame gone)
        }
        break;
    }
    case MustFlyLocation::Line: // (given with it, as it was laid out then)
        if (!given || given->lineCount == 0) return bad(0);
        area = *given;
        break;
    case MustFlyLocation::OpLine: { // (the world's, as it is now)
        const OpLine* l = sessionView_ ? sessionView_->opLine(static_cast<OpLineId>(c.target)) : nullptr;
        CommandResult ignored;
        if (!l || layOutLine(*l, area, ignored) != Reason::None) {
            detail.index = 5;
            return Reason::UnknownGeometry; // (none by that id, or its frame gone)
        }
        break;
    }
    case MustFlyLocation::Volume: // (given with it, as it was laid out then)
        if (!given || given->volume == VolumeShape::Count) return bad(0);
        area = *given;
        break;
    case MustFlyLocation::OpVolume: { // (the world's, as it is now)
        const OpVolume* v = sessionView_ ? sessionView_->opVolume(static_cast<OpVolumeId>(c.target)) : nullptr;
        CommandResult ignored;
        if (!v || layOutVolume(*v, area, ignored) != Reason::None) {
            detail.index = 5;
            return Reason::UnknownGeometry; // (none by that id, or its frame gone)
        }
        break;
    }
    default: return bad(0);
    }
    if (area.shape != ZoneShape::Count) return enterZone(c, area, ingressMin, ingressMax, state, log);
    if (area.lineCount) return flyLine(c, area, ingressMin, ingressMax, state, log);
    if (area.volume != VolumeShape::Count) return enterVolume(c, area, ingressMin, ingressMax, state, log);

    // its window of bearings to come from: the points it approaches through, laid out from where the location and the
    // aircraft are now - in the location's frame, where it is in one, north and east of the frame's origin
    Waypoint points[3];
    std::uint32_t n = 0;
    if (!isHold(ingressMin)) {
        double latL = at.latitudeRad, lonL = at.longitudeRad, northL = 0.0, eastL = 0.0; // (in a frame: the location from its origin)
        if (framed) {
            const GeoPoint g = framePoint(pose, at.frameOffset());
            latL = g.latitudeRad, lonL = g.longitudeRad;
            geo::localNorthEastM(pose.latitudeRad, pose.longitudeRad, latL, lonL, northL, eastL);
        }
        double northA = 0.0, eastA = 0.0;
        geo::localNorthEastM(latL, lonL, state.latitudeRad, state.longitudeRad, northA, eastA);
        double out[2][2];
        int k = 0;
        const double altitudeMsl = isHold(at.altitudeM) || !isHold(at.frameZM) ? state.altitudeMslM : at.altitudeM;
        const double radius = approachRadius(c, state, altitudeMsl);
        approach(northA, eastA, ingressMin, ingressMax, std::max(3.0 * radius, 10.0 * approachSpeed(c, state, altitudeMsl)), radius, out, k);
        for (int j = 0; j < k; ++j) {
            Waypoint& w = points[n++];
            w.altitudeM = at.altitudeM, w.altitudeReference = at.altitudeReference, w.speed = at.speed, w.speedReference = at.speedReference;
            if (framed) { // (unturned, where the frame's origin was: the frame's moves carry them)
                w.frame = at.frame, w.frameRotation = static_cast<double>(FrameRotation::Unrotated);
                w.frameOffsets = static_cast<double>(FrameOffsets::Cartesian);
                w.frameXM = northL + out[j][0], w.frameYM = eastL + out[j][1], w.frameZM = at.frameZM;
            } else {
                geo::offsetLatLon(latL, lonL, out[j][0], out[j][1], w.latitudeRad, w.longitudeRad);
                w.longitudeRad = geo::wrapPi(w.longitudeRad);
            }
        }
    }
    points[n++] = at;
    RouteCommand laid; // (great circles, once, on along its course at its end)
    return checkRoute(laid, Span<const Waypoint>(points, n), state, log);
}

double CapabilityHost::approachSpeed(const MustFlyCommand& c, const sim::VehicleState& state, double altitudeMslM) const noexcept {
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    const double own = hovers ? orHold(performance_.cruiseTasMs, 5.0) : state.airspeedTrueMs;
    return isHold(c.speed) ? own : route::plannedSpeed(c.speed, orHold(c.speedReference, 0.0), altitudeMslM);
}

double CapabilityHost::approachRadius(const MustFlyCommand& c, const sim::VehicleState& state, double altitudeMslM) const noexcept {
    // the turn onto its last leg at its speed plus the wind (a rotorcraft's cruise, given none)
    const WindEstimate wind = checkWind(state);
    return performance_.turnRadiusM(approachSpeed(c, state, altitudeMslM) + std::hypot(wind.northMs, wind.eastMs));
}

Reason CapabilityHost::enterZone(const MustFlyCommand& c, MustFlyArea& area, double ingressMin, double ingressMax, const sim::VehicleState& state,
                                 CheckLog& log) {
    // the zone where it is now (a frame's where the frame is), and the aircraft in its plane
    FramePose pose;
    if (area.framed) {
        FrameSpec spec;
        if (!sessionView_ || !sessionView_->frame(area.frameId, spec, pose)) {
            log.result.index = 5;
            return Reason::UnknownGeometry; // (its frame gone)
        }
    }
    const double now = sessionView_ ? sessionView_->simTimeS() : 0.0;
    const FramePose* at = area.framed ? &pose : nullptr;
    const double turn = zones::turnNow(area, at); // (its plane's axes from north: bearings turned into it)
    double ax = 0.0, ay = 0.0, cx = 0.0, cy = 0.0;
    zones::toPlane(area, at, now, state.latitudeRad, state.longitudeRad, ax, ay);
    zones::centre(area, cx, cy);
    // its altitude: the one given - where it has a band, within it and in its reference (left out, the band's): else it is
    // never entered - or the aircraft's, held within its band (a tenth of it in, or 30 m where it is open)
    double altitude = kHold, reference = kHold;
    const double own = altitudeNow(area.altitudeReference, state, &config_->altimeter);
    if (const int field = inBand(c.altitudeM, c.altitudeReference, area.altitudeMinM, area.altitudeMaxM, area.altitudeReference, own, altitude, reference);
        field >= 0) {
        log.result.index = static_cast<std::int16_t>(field);
        return Reason::InvalidParameter;
    }
    // where it goes in: its nearest point - or, with a window of bearings, its edge that way from its centre - and a little
    // further in (a fifth of the way across it, 200 m at most), aimed at over the ground
    double ex = ax, ey = ay, inX = 1.0, inY = 0.0;
    const bool inside = zones::contains(area, ax, ay);
    const double speed = approachSpeed(c, state, state.altitudeMslM);
    if (inside) { // (in it already, over the ground: on as it flies for ten seconds, to its altitude)
        const double track = (std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]) > 0.5 ? std::atan2(state.velocityNedMs[1], state.velocityNedMs[0])
                                                                                                : state.eulerRad[2]) -
                             turn;
        ex = ax + 10.0 * std::max(speed, 1.0) * std::cos(track), ey = ay + 10.0 * std::max(speed, 1.0) * std::sin(track);
        inX = inY = 0.0;
    } else if (!isHold(ingressMin)) {
        // (the bearing from its centre to the aircraft, where it is within the window; else the window's nearer edge, moved in)
        const double from = geo::wrapTwoPi(std::atan2(ay - cy, ax - cx)), lo = ingressMin - turn, hi = ingressMax - turn;
        const double width = geo::wrapTwoPi(hi - lo), margin = std::min(5.0 * kDeg, 0.5 * width);
        const double bearing = geo::wrapTwoPi(from - lo) <= width                          ? from
                               : geo::wrapTwoPi(lo - from) <= geo::wrapTwoPi(from - hi) ? lo + margin
                                                                                         : hi - margin;
        zones::ray(area, bearing, ex, ey);
        const double d = std::hypot(cx - ex, cy - ey);
        inX = d > 1e-9 ? (cx - ex) / d : 0.0, inY = d > 1e-9 ? (cy - ey) / d : 0.0;
    } else {
        double nx = 0.0, ny = 0.0;
        zones::nearest(area, ax, ay, nx, ny, inX, inY);
        ex = nx, ey = ny;
    }
    const double deep = std::min(0.2 * zones::extent(area), 200.0);
    const double aimX = ex + deep * inX, aimY = ey + deep * inY;
    // its points: the approach from within its window (as a point's), then where it aims - in its frame where it is in one,
    // else on the Earth (a moving zone's where it will be as the aircraft gets there, as it was reckoned now)
    Waypoint points[3];
    std::uint32_t n = 0;
    double plane[3][2];
    if (!isHold(ingressMin) && !inside) {
        double out[2][2];
        int k = 0;
        const double radius = approachRadius(c, state, state.altitudeMslM);
        approach(ax - aimX, ay - aimY, ingressMin - turn, ingressMax - turn, std::max(3.0 * radius, 10.0 * speed), radius, out, k);
        for (int j = 0; j < k; ++j) plane[n][0] = aimX + out[j][0], plane[n][1] = aimY + out[j][1], ++n;
    }
    plane[n][0] = aimX, plane[n][1] = aimY, ++n;
    const double lead = area.framed ? 0.0 : std::hypot(aimX - ax, aimY - ay) / std::max(speed, 1.0);
    for (std::uint32_t j = 0; j < n; ++j) {
        Waypoint& w = points[j];
        w.altitudeM = altitude, w.altitudeReference = reference, w.speed = c.speed, w.speedReference = c.speedReference;
        if (area.framed) {
            // (along its frame's axes from the origin, turned as the zone's are: flat)
            const double fx = area.frameXM + plane[j][0], fy = area.frameYM + plane[j][1];
            w.frame = static_cast<double>(area.frameId);
            w.frameRotation = static_cast<double>(area.rotation == FrameRotation::Attitude ? FrameRotation::Yaw : area.rotation);
            w.frameOffsets = static_cast<double>(FrameOffsets::Cartesian);
            w.frameXM = fx, w.frameYM = fy;
        } else {
            zones::fromPlane(area, nullptr, now + lead, plane[j][0], plane[j][1], w.latitudeRad, w.longitudeRad);
        }
    }
    points[n - 1].kind = static_cast<double>(EndPointKind::Waypoint); // (flown over, where it has not gone in before it)
    RouteCommand laid; // (ending in a loiter at its aim, until the aircraft is in it: as the behaviour flies it)
    laid.end = static_cast<double>(EndBehavior::Loiter);
    if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, n), state, log); why != Reason::None) return why;
    routePlan_->area = area; // (after the checks, which clear it)
    return Reason::None;
}

Reason CapabilityHost::flyLine(const MustFlyCommand& c, MustFlyArea& line, double ingressMin, double ingressMax, const sim::VehicleState& state,
                               CheckLog& log) {
    // the line where it is now (a frame's where the frame is), and the aircraft in its plane
    FramePose pose;
    if (line.framed) {
        FrameSpec spec;
        if (!sessionView_ || !sessionView_->frame(line.frameId, spec, pose)) {
            log.result.index = 5;
            return Reason::UnknownGeometry; // (its frame gone)
        }
    }
    const double now = sessionView_ ? sessionView_->simTimeS() : 0.0;
    const FramePose* at = line.framed ? &pose : nullptr;
    const double turn = zones::turnNow(line, at);
    double ax = 0.0, ay = 0.0;
    zones::toPlane(line, at, now, state.latitudeRad, state.longitudeRad, ax, ay);
    const std::uint32_t n = line.lineCount;
    // the altitude at each vertex: the one given - within the band there, in its reference - else the vertex's own, else the
    // aircraft's held within the band there (4.43's rule)
    double altitude[MustFlyArea::kVertices], reference[MustFlyArea::kVertices];
    for (std::uint32_t v = 0; v < n; ++v) {
        if (isHold(c.altitudeM) && !isHold(line.lineAltitudeM[v])) {
            altitude[v] = line.lineAltitudeM[v], reference[v] = static_cast<double>(line.lineAltitudeReference[v]);
            continue;
        }
        const double own = altitudeNow(line.lineBandReference[v], state, &config_->altimeter);
        if (const int field = inBand(c.altitudeM, c.altitudeReference, line.lineMinM[v], line.lineMaxM[v], line.lineBandReference[v], own, altitude[v],
                                     reference[v]);
            field >= 0) {
            log.result.index = static_cast<std::int16_t>(field);
            return Reason::InvalidParameter;
        }
    }
    // its entry: onto its first segment behind its first vertex, from within 10 degrees of its line there - or from within the
    // window given (the bearings from its first vertex, turned into its plane) - approached as a point is (4.42)
    const double x0 = line.vertices[0][0], y0 = line.vertices[0][1];
    const double back = std::atan2(y0 - line.vertices[1][1], x0 - line.vertices[1][0]); // (from its first vertex, away from its second)
    const bool windowed = !isHold(ingressMin);
    const double lo = windowed ? ingressMin - turn : back - kEntryRad, hi = windowed ? ingressMax - turn : back + kEntryRad;
    const double speed = approachSpeed(c, state, state.altitudeMslM), radius = approachRadius(c, state, state.altitudeMslM);
    double out[2][2];
    int k = 0;
    approach(ax - x0, ay - y0, lo, hi, std::max(3.0 * radius, 10.0 * speed), radius, out, k);
    // its points: the approach, at its first vertex's altitude, then every vertex - flown by, the last flown over - in its
    // frame where it is in one, else on the Earth (a moving line's each where it will be as the aircraft gets there)
    Waypoint points[2 + MustFlyArea::kVertices];
    double plane[2 + MustFlyArea::kVertices][2];
    std::uint32_t m = 0;
    for (int j = 0; j < k; ++j) plane[m][0] = x0 + out[j][0], plane[m][1] = y0 + out[j][1], ++m;
    const std::uint32_t first = m; // (its first vertex's point)
    for (std::uint32_t v = 0; v < n; ++v) plane[m][0] = line.vertices[v][0], plane[m][1] = line.vertices[v][1], ++m;
    double along = 0.0, px = ax, py = ay;
    for (std::uint32_t j = 0; j < m; ++j) {
        Waypoint& w = points[j];
        const std::uint32_t v = j < first ? 0 : j - first;
        w.altitudeM = altitude[v], w.altitudeReference = reference[v], w.speed = c.speed, w.speedReference = c.speedReference;
        along += std::hypot(plane[j][0] - px, plane[j][1] - py), px = plane[j][0], py = plane[j][1];
        if (line.framed) { // (along its frame's axes from the origin, turned as the line's are: flat)
            w.frame = static_cast<double>(line.frameId);
            w.frameRotation = static_cast<double>(line.rotation == FrameRotation::Attitude ? FrameRotation::Yaw : line.rotation);
            w.frameOffsets = static_cast<double>(FrameOffsets::Cartesian);
            w.frameXM = line.frameXM + plane[j][0], w.frameYM = line.frameYM + plane[j][1];
        } else {
            zones::fromPlane(line, nullptr, now + along / std::max(speed, 1.0), plane[j][0], plane[j][1], w.latitudeRad, w.longitudeRad);
        }
    }
    points[m - 1].kind = static_cast<double>(EndPointKind::Waypoint); // (its last vertex, flown over: done as it is passed)
    RouteCommand laid; // (on its projection, once, on along its course at its end)
    laid.projection = static_cast<double>(line.projection);
    if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, m), state, log); why != Reason::None) return why;
    // its widths: each turn within it kept inside them - one flown by cuts inside by r (1 - cos(a/2)); one flown over (beyond
    // 150 degrees) swings out by r (1 - cos a) - against the lesser width that side of the segments either side of it (at its
    // first vertex, its first segment's): beyond it, a turn the aircraft cannot fly within it (named by its point)
    const route::Plan& p = *routePlan_;
    for (std::uint32_t v = 0; v + 1 < n; ++v) {
        const std::uint32_t j = first + v;
        const route::Turn& t = j == p.start ? p.entryTurn : p.turns[j];
        const double in = j == p.start ? p.entry.courseInRad : p.legs[j].courseInRad, change = geo::wrapPi(p.legs[j + 1].courseOutRad - in);
        if (std::abs(change) < kDeg) continue;
        const bool flownBy = t.radiusM > 0.0;
        const double off = flownBy ? t.radiusM * (1.0 - std::cos(0.5 * std::abs(change))) : radius * (1.0 - std::cos(change));
        const bool right = (change > 0.0) == flownBy; // (the side it goes out to: inside a turn flown by, outside one flown over)
        const double* widths = right ? line.rightWidthM : line.leftWidthM;
        const double room = v == 0 ? orNone(widths[0]) : std::min(orNone(widths[v - 1]), orNone(widths[v]));
        if (off > room) log.find(Reason::PerformanceLimit, static_cast<std::int16_t>(j), Constraint::MaxTurnRate);
    }
    routePlan_->area = line; // (after the checks, which clear it)
    return Reason::None;
}

Reason CapabilityHost::enterVolume(const MustFlyCommand& c, MustFlyArea& volume, double ingressMin, double ingressMax, const sim::VehicleState& state,
                                   CheckLog& log) {
    // the volume where it is now (a frame's where the frame is), and the aircraft in its plane and below its point
    FramePose pose;
    if (volume.framed) {
        FrameSpec spec;
        if (!sessionView_ || !sessionView_->frame(volume.frameId, spec, pose)) {
            log.result.index = 5;
            return Reason::UnknownGeometry; // (its frame gone)
        }
    }
    const double now = sessionView_ ? sessionView_->simTimeS() : 0.0;
    const FramePose* at = volume.framed ? &pose : nullptr;
    const double turn = zones::turnNow(volume, at);
    double ax = 0.0, ay = 0.0, ix = 0.0, iy = 0.0, iz = 0.0;
    zones::toPlane(volume, at, now, state.latitudeRad, state.longitudeRad, ax, ay);
    volumes::inner(volume, ix, iy, iz);
    const double base = volumes::pointAltitude(volume, at, now);
    const double reference = static_cast<double>(volume.altitudeReference);
    const double az = base - altitudeNow(volume.altitudeReference, state, &config_->altimeter); // (below its point)
    // the height to go in at, below its point: the altitude given - inside it over its inner point, in its reference - else the
    // aircraft's, held within it on that vertical as a zone's band holds it (a tenth of the way in from its top and bottom, 30 m
    // at most; its middle, where it is thinner)
    const double speed = approachSpeed(c, state, state.altitudeMslM), reach = volumes::extent(volume);
    const double far = 2.0 * reach + std::abs(az - iz) + 1000.0;
    auto edge = [&](double outside) { // (its edge between the inner point and one that way, halved toward it: every shape is convex)
        double in = iz, out = outside;
        if (volumes::contains(volume, ix, iy, out)) return out; // (it runs on that way)
        for (int k = 0; k < 48; ++k) (volumes::contains(volume, ix, iy, 0.5 * (in + out)) ? in : out) = 0.5 * (in + out);
        return in;
    };
    const double top = edge(iz - far), bottom = edge(iz + far);
    double z = az;
    if (!isHold(c.altitudeM)) {
        const int field = !isHold(c.altitudeReference) && c.altitudeReference != reference ? 4 : volumes::contains(volume, ix, iy, base - c.altitudeM) ? -1 : 3;
        if (field >= 0) {
            log.result.index = static_cast<std::int16_t>(field);
            return Reason::InvalidParameter;
        }
        z = base - c.altitudeM;
    } else {
        const double in = std::min(0.1 * (bottom - top), 30.0);
        z = top + in <= bottom - in ? std::clamp(az, top + in, bottom - in) : 0.5 * (top + bottom);
    }
    // where it goes in, at that height: toward its inner point from the aircraft - or, with a window of bearings, from the
    // bearing within it from the inner point - its edge, and a fifth of the way across it further in (200 m at most, never
    // past the inner point); over it already, on as it flies for ten seconds
    double aimX = ax, aimY = ay;
    const bool inside = volumes::contains(volume, ax, ay, z);
    if (inside) {
        const double track = (std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]) > 0.5 ? std::atan2(state.velocityNedMs[1], state.velocityNedMs[0])
                                                                                                : state.eulerRad[2]) -
                             turn;
        aimX = ax + 10.0 * std::max(speed, 1.0) * std::cos(track), aimY = ay + 10.0 * std::max(speed, 1.0) * std::sin(track);
    } else {
        double ox = ax, oy = ay; // (outside it)
        if (!isHold(ingressMin)) {
            const double from = geo::wrapTwoPi(std::atan2(ay - iy, ax - ix)), lo = ingressMin - turn, hi = ingressMax - turn;
            const double width = geo::wrapTwoPi(hi - lo), margin = std::min(5.0 * kDeg, 0.5 * width);
            const double bearing = geo::wrapTwoPi(from - lo) <= width                          ? from
                                   : geo::wrapTwoPi(lo - from) <= geo::wrapTwoPi(from - hi) ? lo + margin
                                                                                             : hi - margin;
            for (double outM = 2.0 * reach + 1000.0; outM < 64.0 * (reach + 1000.0); outM *= 2.0) { // (out of it that way: a cone may run on)
                ox = ix + outM * std::cos(bearing), oy = iy + outM * std::sin(bearing);
                if (!volumes::contains(volume, ox, oy, z)) break;
            }
            if (volumes::contains(volume, ox, oy, z)) ox = ax, oy = ay; // (it has no edge that way: from the aircraft's side)
        }
        double t0 = 0.0, t1 = 1.0; // (along from outside to the inner point: its edge)
        for (int k = 0; k < 48; ++k) {
            const double t = 0.5 * (t0 + t1);
            (volumes::contains(volume, ox + t * (ix - ox), oy + t * (iy - oy), z) ? t1 : t0) = t;
        }
        const double length = std::hypot(ix - ox, iy - oy);
        const double deep = std::min({0.2 * reach, 200.0, (1.0 - t1) * length});
        const double ux = length > 1e-9 ? (ix - ox) / length : 0.0, uy = length > 1e-9 ? (iy - oy) / length : 0.0;
        aimX = ox + t1 * (ix - ox) + deep * ux, aimY = oy + t1 * (iy - oy) + deep * uy;
    }
    // its points: the approach from within its window (as a point's), then where it aims - in its frame where it is in one,
    // else on the Earth (a moving volume's where it will be as the aircraft gets there) - at the height chosen
    Waypoint points[3];
    std::uint32_t n = 0;
    double plane[3][2];
    if (!isHold(ingressMin) && !inside) {
        double out[2][2];
        int k = 0;
        const double radius = approachRadius(c, state, state.altitudeMslM);
        approach(ax - aimX, ay - aimY, ingressMin - turn, ingressMax - turn, std::max(3.0 * radius, 10.0 * speed), radius, out, k);
        for (int j = 0; j < k; ++j) plane[n][0] = aimX + out[j][0], plane[n][1] = aimY + out[j][1], ++n;
    }
    plane[n][0] = aimX, plane[n][1] = aimY, ++n;
    const double lead = volume.framed ? 0.0 : std::hypot(aimX - ax, aimY - ay) / std::max(speed, 1.0);
    for (std::uint32_t j = 0; j < n; ++j) {
        Waypoint& w = points[j];
        w.altitudeM = volumes::pointAltitude(volume, at, now + lead) - z, w.altitudeReference = reference;
        w.speed = c.speed, w.speedReference = c.speedReference;
        if (volume.framed) {
            w.frame = static_cast<double>(volume.frameId);
            w.frameRotation = static_cast<double>(volume.rotation == FrameRotation::Attitude ? FrameRotation::Yaw : volume.rotation);
            w.frameOffsets = static_cast<double>(FrameOffsets::Cartesian);
            w.frameXM = volume.frameXM + plane[j][0], w.frameYM = volume.frameYM + plane[j][1];
        } else {
            zones::fromPlane(volume, nullptr, now + lead, plane[j][0], plane[j][1], w.latitudeRad, w.longitudeRad);
        }
    }
    points[n - 1].kind = static_cast<double>(EndPointKind::Waypoint); // (flown over, where it has not gone in before it)
    RouteCommand laid; // (ending in a loiter at its aim, as a zone's)
    laid.end = static_cast<double>(EndBehavior::Loiter);
    if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, n), state, log); why != Reason::None) return why;
    routePlan_->area = volume; // (after the checks, which clear it)
    return Reason::None;
}

CommandResult CapabilityHost::submitLaidOut(const MustFlyCommand& mustFly, const MustFlyArea& area, const CommandOptions& options,
                                            const sim::VehicleState& state, double now) {
    RouteExtras extras;
    extras.area = &area;
    return submitWith(Command(mustFly), {}, {}, options, state, now, true, nullptr, nullptr, &extras);
}

CommandResult CapabilityHost::submit(const MustFlyCommand& mustFly, const OpZone* zone, const CommandOptions& options, const sim::VehicleState& state,
                                     double now) {
    if (!zone) return submitWith(Command(mustFly), {}, {}, options, state, now);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutZone(*zone, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why);
        r.index = detail.index;
        return r;
    }
    return submitLaidOut(mustFly, area, options, state, now);
}

CommandResult CapabilityHost::submit(const MustFlyCommand& mustFly, const OpLine* line, const CommandOptions& options, const sim::VehicleState& state,
                                     double now) {
    if (!line) return submitWith(Command(mustFly), {}, {}, options, state, now);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutLine(*line, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why);
        r.index = detail.index;
        return r;
    }
    return submitLaidOut(mustFly, area, options, state, now);
}

CommandResult CapabilityHost::update(ActivityId activity, const MustFlyCommand& mustFly, const OpZone* zone, const sim::VehicleState& state,
                                     Caller caller) {
    if (!zone) return update(activity, Command(mustFly), state, caller);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutZone(*zone, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why, activity);
        r.index = detail.index;
        return r;
    }
    return updateLaidOut(activity, mustFly, area, state, caller);
}

CommandResult CapabilityHost::submit(const MustFlyCommand& mustFly, const OpVolume* volume, const CommandOptions& options, const sim::VehicleState& state,
                                     double now) {
    if (!volume) return submitWith(Command(mustFly), {}, {}, options, state, now);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutVolume(*volume, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why);
        r.index = detail.index;
        return r;
    }
    return submitLaidOut(mustFly, area, options, state, now);
}

CommandResult CapabilityHost::update(ActivityId activity, const MustFlyCommand& mustFly, const OpVolume* volume, const sim::VehicleState& state,
                                     Caller caller) {
    if (!volume) return update(activity, Command(mustFly), state, caller);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutVolume(*volume, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why, activity);
        r.index = detail.index;
        return r;
    }
    return updateLaidOut(activity, mustFly, area, state, caller);
}

CommandResult CapabilityHost::update(ActivityId activity, const MustFlyCommand& mustFly, const OpLine* line, const sim::VehicleState& state,
                                     Caller caller) {
    if (!line) return update(activity, Command(mustFly), state, caller);
    MustFlyArea area;
    CommandResult detail;
    if (const Reason why = layOutLine(*line, area, detail); why != Reason::None) {
        details_.clear();
        CommandResult r = rejected(why, activity);
        r.index = detail.index;
        return r;
    }
    return updateLaidOut(activity, mustFly, area, state, caller);
}

CommandResult CapabilityHost::updateLaidOut(ActivityId activity, const MustFlyCommand& mustFly, const MustFlyArea& area, const sim::VehicleState& state,
                                            Caller caller) {
    details_.clear();
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) {
            RouteExtras extras;
            extras.area = &area;
            return updateWaiting(*w, Command(mustFly), {}, {}, state, caller, nullptr, nullptr, &extras);
        }
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    if (!std::holds_alternative<MustFlyCommand>(config_->slots[s].command)) return rejected(Reason::WrongCommandType, activity);
    CommandResult result; // (accepted, as the other UPDATEs' answer begins)
    result.status = CommandStatus::Accepted, result.activity = activity;
    result.commandId = records_[s].commandId;
    CheckLog log{result, slots_[s].range, &details_};
    return updateMustFly(s, activity, mustFly, state, result, log, &area);
}

CommandResult CapabilityHost::updateMustFly(std::size_t s, ActivityId activity, const MustFlyCommand& next, const sim::VehicleState& state,
                                            CommandResult& result, CheckLog& log, const MustFlyArea* given) noexcept {
    SetpointSlot& slot = config_->slots[s];
    MustFlyCommand merged = std::get<MustFlyCommand>(slot.command);
    mergeMustFly(merged, next);
    // a window given both ways, or neither
    if (isHold(next.ingressMinRad) != isHold(next.ingressMaxRad)) {
        result.index = isHold(next.ingressMinRad) ? 6 : 7;
        return about(rejected(Reason::InvalidParameter, activity), result);
    }
    // a zone, a corridor or a volume given with it before: the one it flies, as it was laid out then, where none is given now
    // (4.43 to 4.45)
    const PathStore* store = config_->path.get();
    const bool keeps = merged.location == static_cast<double>(MustFlyLocation::Zone) || merged.location == static_cast<double>(MustFlyLocation::Line) ||
                       merged.location == static_cast<double>(MustFlyLocation::Volume);
    if (!given && store && store->mustFlyArea.laidOut() && keeps) given = &store->mustFlyArea;
    MustFlyArea kept; // (a copy: the checks lay a route out into the scratch plan, never the store it may be in)
    if (given) kept = *given;
    if (const Reason why = prepareMustFly(merged, state, log, given ? &kept : nullptr); why != Reason::None) return about(rejected(why, activity), result);
    if (slots_[s].range != RangePolicy::None) {
        Command checked = merged;
        if (const Reason why = catalog_->check(records_[s].capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
        merged = std::get<MustFlyCommand>(checked);
    }
    // what the checks found, and its terrain, whatever its range policy - as a NEW's (4.8, 4.19)
    checkTerrain(Command(merged), state, log);
    if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
    if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    writeRoute(), routePlan_->passed.clear(); // (the path store's route afresh: flown from where the aircraft is)
    std::get<MustFlyCommand>(slot.command) = merged;
    ++slot.revision;
    return result;
}

// --- The behaviour ------------------------------------------------------------------------------

/// The world as the route sees it while it flies to its first point over another vehicle: that vehicle where it will be
/// as the aircraft gets there, carried on at its velocity - so the leg to it is a collision course, which closes on the
/// vehicle itself (4.42). Every other answer the world's.
struct MustFlyBehavior::Lead final : WorldView {
    const WorldView* world = nullptr;
    std::uint32_t entity = 0;
    bool leading = false;
    sim::VehicleState ahead{};
    const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override {
        if (leading && id == entity) return &ahead;
        return world->vehicleState(id);
    }
    double simTime() const noexcept override { return world->simTime(); }
    const sim::EnvironmentState& environment() const noexcept override { return world->environment(); }
    bool navigation(std::uint32_t id, NavigationReport& out) const noexcept override { return world->navigation(id, out); }
};

namespace {

/// The aircraft in a must fly's zone now: over the ground (the zone where it is, a frame's as the step began) and within its
/// band, in its reference (4.43); or in its volume, below the volume's point as it is now (4.45).
bool inZone(const ControlContext& ctx, const MustFlyArea& a) noexcept {
    const sim::VehicleState& s = ctx.sensed;
    const double now = ctx.world ? ctx.world->simTime() : s.simTime;
    FramePose pose;
    if (a.framed) {
        if (a.frame.origin == FrameOrigin::Vehicle) {
            const sim::VehicleState* v = ctx.world ? ctx.world->vehicleState(a.frame.vehicle) : nullptr;
            if (!v) return false;
            pose = vehiclePose(*v);
        } else {
            pose = framePose(a.frame, now);
        }
    }
    double x = 0.0, y = 0.0;
    zones::toPlane(a, a.framed ? &pose : nullptr, now, s.latitudeRad, s.longitudeRad, x, y);
    if (a.volume != VolumeShape::Count)
        return volumes::contains(a, x, y, volumes::pointAltitude(a, a.framed ? &pose : nullptr, now) - altitudeNow(a.altitudeReference, s, ctx.altimeter));
    if (!zones::contains(a, x, y)) return false;
    const double h = altitudeNow(a.altitudeReference, s, ctx.altimeter);
    return (isHold(a.altitudeMinM) || h >= a.altitudeMinM) && (isHold(a.altitudeMaxM) || h <= a.altitudeMaxM);
}

/// How a must fly's route ends (4.43 to 4.45): a zone's or a volume's in a loiter at its aim - a rotorcraft stops over it, a wing
/// orbits it - until the aircraft is in it; a point's and a corridor's on along its course, done as it is passed.
double endOf(const ControlContext& ctx) noexcept {
    const bool enters = ctx.path && (ctx.path->mustFlyArea.shape != ZoneShape::Count || ctx.path->mustFlyArea.volume != VolumeShape::Count);
    return static_cast<double>(enters ? EndBehavior::Loiter : EndBehavior::Continue);
}

} // namespace

MustFlyBehavior::MustFlyBehavior() : route_(std::make_unique<RouteBehavior>()), lead_(std::make_unique<Lead>()), options_(RouteCommand{}) {}
MustFlyBehavior::~MustFlyBehavior() = default;

void MustFlyBehavior::begin(const ControlContext& ctx, const Command&) {
    zoned_ = inside_ = false, areaRevision_ = 0;
    std::get<RouteCommand>(options_).end = endOf(ctx);
    route_->begin(ctx, options_);
}

Command MustFlyBehavior::update(const ControlContext& ctx, const Command& in) {
    // a zone's: entered once the aircraft is in it (a new route in the store - an UPDATE - looks afresh)
    zoned_ = ctx.path && (ctx.path->mustFlyArea.shape != ZoneShape::Count || ctx.path->mustFlyArea.volume != VolumeShape::Count);
    std::get<RouteCommand>(options_).end = endOf(ctx); // (an UPDATE to another kind of location changes it)
    if (zoned_) {
        if (ctx.path->revision != areaRevision_) inside_ = false, areaRevision_ = ctx.path->revision;
        inside_ = inside_ || inZone(ctx, ctx.path->mustFlyArea);
    }
    return fly(ctx, in);
}

Command MustFlyBehavior::fly(const ControlContext& ctx, const Command& in) {
    const auto* m = std::get_if<MustFlyCommand>(&in);
    ActivityProgress progress;
    const bool first = !route_->progress(progress) || progress.segment == 0;
    const sim::VehicleState* e = m && m->location == static_cast<double>(MustFlyLocation::Entity) && ctx.world && ctx.path && ctx.path->count
                                     ? ctx.world->vehicleState(static_cast<std::uint32_t>(m->target))
                                     : nullptr;
    if (!e || !first) return route_->update(ctx, options_);
    // the first point, where that vehicle's frame puts it (north and east of it, unturned: 4.42), met at the aircraft's
    // speed over the ground: when, found again from where the point will be by then (three times: it converges)
    const sim::VehicleState& s = ctx.sensed;
    const Waypoint& p = ctx.path->waypoints[0];
    const double pn = orHold(p.frameXM, 0.0), pe = orHold(p.frameYM, 0.0);
    double n0 = 0.0, e0 = 0.0;
    geo::localNorthEastM(s.latitudeRad, s.longitudeRad, e->latitudeRad, e->longitudeRad, n0, e0);
    // (a rotorcraft from a hover: at the speed it will fly - its cruise, given none or in another reference - not the speed it
    // has yet)
    const bool metres = !isHold(m->speed) && (isHold(m->speedReference) || m->speedReference == static_cast<double>(SpeedReference::TrueAirspeed) ||
                                              m->speedReference == static_cast<double>(SpeedReference::GroundSpeed));
    const double least =
        (ctx.features & kFeatureHover) && ctx.performance ? (metres ? m->speed : orHold(ctx.performance->cruiseTasMs, 1.0)) : 1.0;
    const double speed = std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), std::max(least, 1.0));
    double t = 0.0;
    for (int k = 0; k < 3; ++k)
        t = std::min(std::hypot(n0 + pn + e->velocityNedMs[0] * t, e0 + pe + e->velocityNedMs[1] * t) / speed, 120.0);
    Lead& lead = *lead_;
    lead.world = ctx.world, lead.entity = static_cast<std::uint32_t>(m->target), lead.leading = true;
    lead.ahead = *e;
    geo::offsetLatLon(e->latitudeRad, e->longitudeRad, e->velocityNedMs[0] * t, e->velocityNedMs[1] * t, lead.ahead.latitudeRad, lead.ahead.longitudeRad);
    lead.ahead.altitudeMslM = e->altitudeMslM - e->velocityNedMs[2] * t;
    ControlContext led = ctx;
    led.world = &lead;
    return route_->update(led, options_);
}

void MustFlyBehavior::reset() { route_->reset(); }

bool MustFlyBehavior::finished() const noexcept { return zoned_ ? inside_ : route_->finished(); }

Reason MustFlyBehavior::failure() const noexcept { return route_->failure(); }

std::uint16_t MustFlyBehavior::constraints() const noexcept { return route_->constraints(); }

bool MustFlyBehavior::progress(ActivityProgress& out) const noexcept { return route_->progress(out); }

std::uint32_t MustFlyBehavior::ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept { return route_->ahead(points, max, ends); }

bool MustFlyBehavior::arrival(ArrivalEstimate& out) const noexcept { return static_cast<const Behavior&>(*route_).arrival(out); }

void registerMustFly(ControllerRegistry& r) {
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi, Constraint below = Constraint::None,
                Constraint above = Constraint::None) { return ParameterInfo{name, unit, lo, hi, def, true, below, above}; };
    // its setpoint's fields, in order (the C ABI's too); left out: as the host lays it out (4.42)
    BehaviorTraits mustFly;
    mustFly.persistence = Persistence::Terminating;
    mustFly.parameters = {p("location", "", now, 0.0, static_cast<double>(MustFlyLocation::Count) - 1.0),
                          p("latitude_rad", "rad", now, -0.5 * kPi, 0.5 * kPi),
                          p("longitude_rad", "rad", now, -inf, inf),
                          p("altitude_m", "m", now, -inf, inf),
                          p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                          p("target", "", now, 1.0, 9007199254740992.0),
                          p("ingress_min_rad", "rad", now, -kPi, kPi),
                          p("ingress_max_rad", "rad", now, -kPi, kPi),
                          p("speed", "m/s or Mach", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                          p("speed_reference", "", now, 0.0, static_cast<double>(SpeedReference::Count) - 1.0)};
    mustFly.uses = {"fsim.flight.velocity", "fsim.flight.position"};
    mustFly.mode = FlightMode::MustFly;
    mustFly.setpoint = SetpointKind::MustFly;
    r.addBehavior("must_fly", [] { return std::make_unique<MustFlyBehavior>(); }, std::move(mustFly));
}

} // namespace fsim::control
