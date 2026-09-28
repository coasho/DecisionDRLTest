// A route's civil path terminators (docs/flight-autonomy.md, 4.38; ADR-29 FA-6f; A-GRA's CivilPathTerminator): the ARINC
// 424 leg type of the leg into each point - checked and kept by the host, laid out by the plan, flown by the route's
// behaviour. Its own translation unit, last in the library: what is added before the runtime's code moves it (the A/B's
// lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;

void unitOf(double lat, double lon, double v[3]) noexcept {
    v[0] = std::cos(lat) * std::cos(lon), v[1] = std::cos(lat) * std::sin(lon), v[2] = std::sin(lat);
}

void crossOf(const double a[3], const double b[3], double c[3]) noexcept {
    c[0] = a[1] * b[2] - a[2] * b[1], c[1] = a[2] * b[0] - a[0] * b[2], c[2] = a[0] * b[1] - a[1] * b[0];
}

double dotOf(const double a[3], const double b[3]) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

/// The legs FA-6f2 builds: to an altitude, an intercept, a distance or a manual termination, and a hold's.
bool later(PathTerminator t) noexcept {
    switch (t) {
    case PathTerminator::CourseToAltitude:
    case PathTerminator::CourseToIntercept:
    case PathTerminator::TrackToAltitude:
    case PathTerminator::TrackFromFixToDistanceAlongTrack:
    case PathTerminator::FixToManualTermination:
    case PathTerminator::HoldingWithAltitudeTermination:
    case PathTerminator::HoldingWithFixTermination:
    case PathTerminator::HoldingWithManualTermination:
    case PathTerminator::HeadingToAltitude:
    case PathTerminator::HeadingToIntercept:
    case PathTerminator::HeadingToManual: return true;
    default: return false;
    }
}

/// The legs A-GRA 6.0a's segment does not define: a navaid's DME distance, radial or arc (none is given, nor has the world
/// one), and a procedure turn's outbound course, side and limit.
bool undefined(PathTerminator t) noexcept {
    switch (t) {
    case PathTerminator::ArcToFix:
    case PathTerminator::CourseToDmeDistance:
    case PathTerminator::CourseToRadial:
    case PathTerminator::TrackFromFixToDmeDistance:
    case PathTerminator::ProcedureTurnToIntercept:
    case PathTerminator::HeadingToDmeDistanceTermination:
    case PathTerminator::HeadingToRadialTermination: return true;
    default: return false;
    }
}

} // namespace

namespace route {

Leg makeRadiusArc(double latA, double lonA, double latB, double lonB, double latC, double lonC, bool right) noexcept {
    Leg leg = makeLeg(latA, lonA, latB, lonB, false);
    double an, ae, bn, be; // (a and b from the centre, in the plane there: a circle round it on the Earth)
    geo::localNorthEastM(latC, lonC, latA, lonA, an, ae);
    geo::localNorthEastM(latC, lonC, latB, lonB, bn, be);
    const double r = std::hypot(an, ae);
    if (!(r > 1e-6)) return leg; // (its centre at a: no circle)
    const double side = right ? 1.0 : -1.0;
    const double from = std::atan2(ae, an), to = std::atan2(be, bn); // (a's bearing from the centre, and b's)
    double sweep = std::fmod(side * (to - from), 2.0 * kPi); // (round its way, from a to b)
    if (sweep <= 0.0) sweep += 2.0 * kPi;
    leg.arcRadiusM = r;
    leg.arcAngleRad = side * sweep;
    leg.arcCentreNorthM = leg.arcCentreEastM = 0.0;
    leg.arcEntryBearingRad = from;
    leg.arcLatRad = latC, leg.arcLonRad = lonC;
    leg.lengthM = r * sweep;
    leg.courseOutRad = geo::wrapPi(from + side * 0.5 * kPi);
    leg.courseInRad = geo::wrapPi(from + side * (sweep + 0.5 * kPi));
    return leg;
}

bool makeCourseLeg(double latB, double lonB, double courseRad, double lat, double lon, Leg& out) noexcept {
    // its great circle: through b, along the course there - its normal b x t, the left of the way it runs
    double b[3], q[3], n[3], c[3];
    unitOf(latB, lonB, b);
    const double north[3] = {-std::sin(latB) * std::cos(lonB), -std::sin(latB) * std::sin(lonB), std::cos(latB)};
    const double east[3] = {-std::sin(lonB), std::cos(lonB), 0.0};
    const double t[3] = {std::cos(courseRad) * north[0] + std::sin(courseRad) * east[0], std::cos(courseRad) * north[1] + std::sin(courseRad) * east[1],
                         std::cos(courseRad) * north[2] + std::sin(courseRad) * east[2]};
    crossOf(b, t, n);
    // abeam (lat, lon): the foot of the perpendicular from there onto it
    unitOf(lat, lon, q);
    const double side = dotOf(q, n);
    for (int k = 0; k < 3; ++k) q[k] -= side * n[k];
    const double m = std::sqrt(dotOf(q, q));
    if (!(m > 1e-9)) return false; // (at its pole: abeam everywhere)
    for (double& v : q) v /= m;
    crossOf(q, b, c);
    if (!(std::atan2(dotOf(c, n), dotOf(q, b)) * geo::kEarthRadiusM > 1.0)) return false; // (not behind the point along its course)
    out = makeLeg(std::asin(std::clamp(q[2], -1.0, 1.0)), std::atan2(q[1], q[0]), latB, lonB, false);
    return true;
}

Leg legFrom(const Plan& p, std::uint32_t i, double lat, double lon, const Waypoint* before) noexcept {
    const Waypoint& b = p.points[i];
    const PathTerminator type = terminatorOf(b);
    if (type == PathTerminator::Count) return makeLeg(lat, lon, b.latitudeRad, b.longitudeRad, p.rhumb); // (none: the route's own line)
    const RouteTerminator* t = p.terminatorAt(i);
    if (type == PathTerminator::RadiusToFix && t && before && !isHold(t->centerLatitudeRad) && !isHold(t->centerLongitudeRad))
        return makeRadiusArc(before->latitudeRad, before->longitudeRad, b.latitudeRad, b.longitudeRad, t->centerLatitudeRad, t->centerLongitudeRad,
                             t->clockwise == 1.0);
    Leg leg;
    if (type == PathTerminator::CourseToFix && t && !isHold(t->courseRad) && makeCourseLeg(b.latitudeRad, b.longitudeRad, t->courseRad, lat, lon, leg))
        return leg;
    return makeLeg(lat, lon, b.latitudeRad, b.longitudeRad, false); // (a track to fix's, a direct one's, an initial fix's: the great circle)
}

void planTerminators(Plan& p) noexcept {
    const std::uint32_t n = p.count;
    for (std::uint32_t i = 1; i < n; ++i) {
        const Waypoint& a = p.points[i - 1];
        if (isHold(p.points[i].terminator)) continue;
        p.legs[i] = legFrom(p, i, a.latitudeRad, a.longitudeRad, &a);
        p.arcs = p.arcs || p.legs[i].arcRadiusM > 0.0;
    }
    if (p.repeat && n > 1 && !isHold(p.points[p.loop].terminator)) { // (the leg back to where it repeats from: 4.36)
        const Waypoint& last = p.points[n - 1];
        Leg& closing = p.loop > 0 ? p.loopLeg : p.legs[0];
        closing = legFrom(p, p.loop, last.latitudeRad, last.longitudeRad, &last);
        p.arcs = p.arcs || closing.arcRadiusM > 0.0;
    }
}

} // namespace route

// --- the host's ------------------------------------------------------------------------------

Reason CapabilityHost::checkTerminators(route::Plan& p, Span<const Waypoint> waypoints, Span<const RouteTerminator> terminators,
                                        CommandResult& detail) const noexcept {
    p.terminatorCount = 0, p.terminated = false;
    const auto count = static_cast<std::uint32_t>(std::min<std::size_t>(waypoints.size(), route::Plan::kMax));
    bool any = false;
    for (std::uint32_t k = 0; k < count && !any; ++k) any = !isHold(waypoints[k].terminator);
    if (!any && terminators.empty()) return Reason::None;
    // named where it is in the plan (a linked route's flight order), so the answer names it as given (4.36)
    auto at = [&p, &detail](std::uint32_t k, Reason why) {
        const std::uint32_t i = p.linked && k < p.given ? p.position[k] : k;
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return why;
    };
    if (terminators.size() > PathStore::kRouteTerminators) return at(terminators[PathStore::kRouteTerminators].point, Reason::InvalidWaypoint);
    // each one's data: at a point it has, one a point; finite, whole where a code; a course to fix's course alone, a radius
    // to fix's arc alone - its places whole, its lengths above 0 - for a point whose terminator is that
    for (std::size_t j = 0; j < terminators.size(); ++j) {
        const RouteTerminator& t = terminators[j];
        const std::uint32_t k = t.point;
        if (k >= count) return at(k, Reason::InvalidWaypoint);
        for (std::size_t m = 0; m < j; ++m)
            if (terminators[m].point == k) return at(k, Reason::InvalidWaypoint);
        RouteTerminator c = t;
        double* f[RouteTerminator::kFields];
        c.fields(f);
        bool arc = false;
        for (std::size_t m = 0; m < RouteTerminator::kFields; ++m) {
            if (!isHold(*f[m]) && !std::isfinite(*f[m])) return at(k, Reason::InvalidWaypoint);
            arc = arc || (m > 0 && !isHold(*f[m]));
        }
        const auto pair = [](double a, double b) { return isHold(a) == isHold(b); };
        if (!pair(t.centerLatitudeRad, t.centerLongitudeRad) || !pair(t.initialLatitudeRad, t.initialLongitudeRad) ||
            !pair(t.endLatitudeRad, t.endLongitudeRad) || std::abs(t.centerLatitudeRad) > 0.5 * kPi || std::abs(t.initialLatitudeRad) > 0.5 * kPi ||
            std::abs(t.endLatitudeRad) > 0.5 * kPi || t.radiusM <= 0.0 || t.arcM <= 0.0 || t.chordM <= 0.0 ||
            (!isHold(t.clockwise) && t.clockwise != 0.0 && t.clockwise != 1.0))
            return at(k, Reason::InvalidWaypoint);
        const PathTerminator type = route::terminatorOf(waypoints[k]);
        if (type == PathTerminator::CourseToFix ? arc : type == PathTerminator::RadiusToFix ? !isHold(t.courseRad) : true)
            return at(k, Reason::InvalidWaypoint); // (data its leg has none of)
    }
    // each point's: a course to fix with its course, a radius to fix with its centre and its way round; a leg its segment
    // does not define - a navaid's distance, radial or arc, a procedure turn's - none
    for (std::uint32_t k = 0; k < count; ++k) {
        const PathTerminator type = route::terminatorOf(waypoints[k]);
        if (type == PathTerminator::Count) continue;
        const RouteTerminator* t = nullptr;
        for (const RouteTerminator& d : terminators)
            if (d.point == k) t = &d;
        if (type == PathTerminator::CourseToFix && (!t || isHold(t->courseRad))) return at(k, Reason::InvalidWaypoint);
        if (type == PathTerminator::RadiusToFix && (!t || isHold(t->centerLatitudeRad) || isHold(t->clockwise))) return at(k, Reason::InvalidWaypoint);
        if (undefined(type)) return at(k, Reason::InvalidWaypoint);
    }
    // not built yet (its row, partial): a leg to an altitude, an intercept, a distance or a manual termination, and a
    // hold's - FA-6f2's
    for (std::uint32_t k = 0; k < count; ++k)
        if (later(route::terminatorOf(waypoints[k]))) return at(k, Reason::NotImplemented);
    p.terminatorCount = static_cast<std::uint32_t>(terminators.size());
    std::copy_n(terminators.data(), p.terminatorCount, p.terminators);
    p.terminated = any;
    return Reason::None;
}

Reason CapabilityHost::checkLegs(const route::Plan& p, CheckLog& log) const noexcept {
    CommandResult& detail = log.result;
    auto at = [&detail](std::uint32_t i) {
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return Reason::InvalidWaypoint;
    };
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    const Performance& f = performance_;
    auto moves = [&p](double frame) {
        if (isHold(frame)) return false;
        for (std::uint32_t k = 0; k < p.frameCount; ++k)
            if (p.frameIds[k] == static_cast<FrameId>(frame)) return p.frames[k].origin != FrameOrigin::Fixed;
        return false;
    };
    for (std::uint32_t i = 0; i < p.count; ++i) {
        const Waypoint& w = p.points[i];
        const PathTerminator type = route::terminatorOf(w);
        if (type == PathTerminator::Count || !(p.repeat || i >= p.start)) continue; // (a route that does not repeat flies nothing before its start)
        const bool rf = type == PathTerminator::RadiusToFix;
        // an arc runs from the point before it, as the route flies: none where the route's entry comes to it, from where the
        // aircraft is, on its only lap; two where a later lap comes back to it from the last; none for a branch taken to it;
        // its ends in no moving frame (its centre does not move)
        if (rf && ((i == p.start && !p.repeat) || (p.repeat && p.loop > 0 && i == p.loop))) return at(i);
        if (rf && (moves(w.frame) || moves(p.points[p.prev(i)].frame))) return at(i);
        for (std::uint32_t k = 0; k < p.branchCount && rf; ++k)
            if (p.branches[k].next == static_cast<double>(p.named(i))) {
                const std::uint32_t from = p.branches[k].point;
                return at(p.linked && from < p.given ? p.position[from] : from);
            }
        if (i == p.start && !p.repeat) continue; // (the entry: from where the aircraft is - no fault of the route's)
        if (i == 0 && !(p.repeat && p.loop == 0)) continue;
        const std::uint32_t h = p.prev(i);
        const Waypoint& b = p.points[h];
        const auto turn = static_cast<TurnType>(static_cast<int>(b.turn));
        // after a start turn point its arc: a radius to fix's only; after a capture its course: a track to fix's or a course
        // to fix's on it (4.30)
        if (turn == TurnType::StartTurn && !rf) return at(i);
        if (turn == TurnType::CaptureOutboundCourse && type != PathTerminator::TrackToFix && type != PathTerminator::CourseToFix) return at(i);
        const RouteTerminator* t = p.terminatorAt(i);
        if (type == PathTerminator::CourseToFix && t) {
            route::Leg leg;
            if (!route::makeCourseLeg(w.latitudeRad, w.longitudeRad, t->courseRad, b.latitudeRad, b.longitudeRad, leg)) return at(i); // (the point before past it)
            if (turn == TurnType::CaptureOutboundCourse && std::abs(geo::wrapPi(b.courseRad - t->courseRad)) > kDegree) return at(i);
        }
        if (!rf || !t) continue;
        // its arc from the point before: a circle through both (within a metre, or half a percent of its radius), sweeping
        // 170 degrees at most, and what else it gives its own - a radius, courses (within a degree), places, lengths
        if (route::loiterPoint(b)) { // (left at its point: a hold's, a racetrack's, a figure-eight's, a hover's - 4.31)
            const RouteLoiter* l = p.loiterAt(h);
            const auto kind = l ? static_cast<PatternKind>(static_cast<int>(orHold(l->pattern.pattern, 0.0))) : PatternKind::Orbit;
            if (!l || kind == PatternKind::Orbit || l->shape.twoCircles() || !isHold(l->shape.exitLatitudeRad)) return at(i);
        }
        const route::Leg& arc = p.legs[i];
        const double r = arc.arcRadiusM, near = std::max(1.0, 0.005 * r);
        if (!(r > 0.0) || std::abs(arc.arcAngleRad) > 170.0 * kDegree) return at(i);
        double n, e; // (its point from its centre, in its plane)
        geo::localNorthEastM(arc.arcLatRad, arc.arcLonRad, w.latitudeRad, w.longitudeRad, n, e);
        if (std::abs(std::hypot(n - arc.arcCentreNorthM, e - arc.arcCentreEastM) - r) > near) return at(i);
        const double chord = geo::distanceM(b.latitudeRad, b.longitudeRad, w.latitudeRad, w.longitudeRad);
        if ((!isHold(t->radiusM) && std::abs(t->radiusM - r) > near) ||
            (!isHold(t->courseInRad) && std::abs(geo::wrapPi(t->courseInRad - arc.courseOutRad)) > kDegree) ||
            (!isHold(t->courseOutRad) && std::abs(geo::wrapPi(t->courseOutRad - arc.courseInRad)) > kDegree) ||
            (!isHold(t->initialLatitudeRad) && geo::distanceM(t->initialLatitudeRad, t->initialLongitudeRad, b.latitudeRad, b.longitudeRad) > near) ||
            (!isHold(t->endLatitudeRad) && geo::distanceM(t->endLatitudeRad, t->endLongitudeRad, w.latitudeRad, w.longitudeRad) > near) ||
            (!isHold(t->arcM) && std::abs(t->arcM - arc.lengthM) > std::max(1.0, 0.005 * arc.lengthM)) ||
            (!isHold(t->chordM) && std::abs(t->chordM - chord) > std::max(1.0, 0.005 * chord)))
            return at(i);
        if (turn == TurnType::StartTurn) { // (its tangent there the start's course: given, or the leg into it's)
            const double course = !isHold(b.courseRad) ? b.courseRad : h == p.start && !p.repeat ? p.entry.courseInRad : p.legs[h].courseInRad;
            if (std::abs(geo::wrapPi(course - arc.courseOutRad)) > kDegree) return at(i);
        }
        // one the aircraft turns at its point's speed and its full bank (a rotorcraft's, its tilt), whatever the policy (4.30:
        // after a start turn point, its check's)
        const double v = route::plannedSpeed(w.speed, w.speedReference, w.altitudeM), bank = hovers ? f.maxTiltRad : f.maxBankRad;
        if (turn != TurnType::StartTurn && std::isfinite(bank) && bank > 0.0 && r < v * v / (9.80665 * std::tan(bank)))
            log.find(Reason::InvalidWaypoint, static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF)), Constraint::MaxTurnRate);
    }
    return Reason::None;
}

// --- flown ---------------------------------------------------------------------------------

void RouteBehavior::takeTerminators(const ControlContext& ctx) noexcept {
    route::Plan& p = *plan_;
    p.terminatorCount = ctx.path ? std::min<std::uint32_t>(ctx.path->routeTerminatorCount, static_cast<std::uint32_t>(PathStore::kRouteTerminators)) : 0;
    if (p.terminatorCount) std::copy_n(ctx.path->routeTerminators, p.terminatorCount, p.terminators);
    p.terminated = false;
    for (std::uint32_t i = 0; i < p.count && !p.terminated; ++i) p.terminated = !isHold(p.points[i].terminator);
}

void RouteBehavior::direct(const sim::VehicleState& s, const Performance& perf) {
    if (onArc_) { // (in the turn at its point: the leg flown)
        pursuing_ = false;
        return;
    }
    route::Plan& p = *plan_;
    const std::uint32_t k = target_;
    route::Leg& in = firstLap_ && k == p.start ? p.entry : p.looped(k, firstLap_) ? p.loopLeg : p.legs[k];
    // what it flew toward its point since the leg was made, then the leg from here - the turn there planned again
    finishedM_ += std::max(route::onLeg(in, s.latitudeRad, s.longitudeRad).alongM, 0.0);
    const Waypoint& w = p.points[k];
    in = route::makeLeg(s.latitudeRad, s.longitudeRad, w.latitudeRad, w.longitudeRad, false);
    route::replan(p, k, firstLap_, s.altitudeMslM, std::hypot(wind_.northMs, wind_.eastMs), perf, hovers_);
    if (std::abs(geo::wrapPi(route::trackOf(s) - in.courseOutRad)) < kDegree) pursuing_ = false; // (on course to it: that leg flown)
}

} // namespace fsim::control
