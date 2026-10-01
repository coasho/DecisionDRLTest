// A route that starts on the ground (docs/flight-autonomy.md, 4.52; ADR-29 FA-9d): its taxi to the runway and its takeoff -
// from its runway points, or from the airfield and runway of the takeoff's path of FA's own plan - flown before its first point
// in the air by a taxi's and a launch's behaviours; a rejected takeoff taxies off the runway on the path its takeoff branches
// to, or fails the route.
#include "control/CapabilityHost.h"
#include "core/Geodesy.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kTaxiSpeedMs = 8.0;     // a route's taxi points' speed, where theirs is none (or no taxi's)
constexpr double kClimbedOutAglM = 150.0; // its takeoff hands over to its points in the air, climbed out and cleaned up
constexpr double kHoverAglM = 10.0;       // a rotorcraft's, lifted to its hover
constexpr double kDepartureMaxCasMs = 250.0 * 1852.0 / 3600.0; // 250 kt

WaypointType typeOf(const Waypoint& w) noexcept {
    return isHold(w.waypointType) ? WaypointType::NavOnly : static_cast<WaypointType>(static_cast<int>(w.waypointType));
}

bool runwayType(WaypointType t) noexcept { return t == WaypointType::RunwayStart || t == WaypointType::RunwayThreshold || t == WaypointType::RunwayLimit; }

/// The path point i is in (its index), or -1 for none.
int pathOf(Span<const RoutePath> paths, std::uint32_t i) noexcept {
    for (std::size_t k = 0; k < paths.size(); ++k)
        if (i >= paths[k].first && i < paths[k].first + paths[k].count) return static_cast<int>(k);
    return -1;
}

PathType pathTypeOf(Span<const RoutePath> paths, std::uint32_t i) noexcept {
    const int k = pathOf(paths, i);
    return k < 0 || isHold(paths[static_cast<std::size_t>(k)].type) ? PathType::Primary : static_cast<PathType>(static_cast<int>(paths[static_cast<std::size_t>(k)].type));
}

/// The point flown after point i (A-GRA's NextPathSegment, else the next in its path, else the next given), or -1: the end.
std::int64_t nextOf(Span<const Waypoint> points, Span<const RoutePath> paths, std::uint32_t i) noexcept {
    const Waypoint& w = points[i];
    if (!isHold(w.next)) return w.next < 0.0 || w.next >= static_cast<double>(points.size()) ? -1 : static_cast<std::int64_t>(w.next);
    if (!paths.empty()) {
        const int k = pathOf(paths, i);
        if (k < 0) return -1;
        const RoutePath& p = paths[static_cast<std::size_t>(k)];
        return i + 1 < p.first + p.count ? static_cast<std::int64_t>(i) + 1 : -1;
    }
    return i + 1 < points.size() ? static_cast<std::int64_t>(i) + 1 : -1;
}

bool taxiAt(Span<const Waypoint> points, Span<const RoutePath> paths, std::uint32_t i) noexcept {
    return typeOf(points[i]) == WaypointType::Taxi || pathTypeOf(paths, i) == PathType::Taxi;
}

std::uint32_t startOf(const RouteCommand& route, std::size_t count) noexcept {
    const double s = isHold(route.start) ? 0.0 : route.start;
    return s >= 0.0 && s < static_cast<double>(count) ? static_cast<std::uint32_t>(s) : 0;
}

} // namespace

bool CapabilityHost::startsOnGround(const RouteCommand& route, Span<const Waypoint> waypoints, Span<const RoutePath> paths) const noexcept {
    if (waypoints.empty()) return false;
    const std::uint32_t s = startOf(route, waypoints.size());
    const WaypointType t = typeOf(waypoints[s]);
    const PathType p = pathTypeOf(paths, s);
    return t == WaypointType::Taxi || runwayType(t) || p == PathType::Taxi || p == PathType::Takeoff;
}

bool CapabilityHost::groundStart(const Command& command, Span<const Waypoint> waypoints, Span<const RoutePath> paths) const noexcept {
    const auto* route = std::get_if<RouteCommand>(&command);
    return route && startsOnGround(*route, waypoints, paths);
}

Reason CapabilityHost::prepareGroundRoute(RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log,
                                          const RouteExtras* extras) {
    CommandResult& detail = log.result;
    auto at = [&detail](std::int64_t point, Reason why, Constraint c = Constraint::None) {
        detail.index = static_cast<std::int16_t>(std::clamp<std::int64_t>(point, 0, 0x7FFF)), detail.constraint = c;
        return why;
    };
    const Span<const RoutePath> paths = extras ? extras->paths : Span<const RoutePath>{};
    const std::uint32_t s0 = startOf(route, waypoints.size());
    if (!state.onGround) return at(s0, Reason::Airborne); // (a start on the ground, the aircraft flying)
    if (!ground_) ground_ = std::make_unique<RouteGround>();
    RouteGround& g = *ground_;
    g = RouteGround{};
    // its taxi points from its start, in the order flown (A-GRA's TAXI: a point's type, or its path's)
    std::int64_t i = s0;
    std::vector<std::uint32_t> taxi; // (its indices, for the point a refusal names)
    while (i >= 0 && taxiAt(waypoints, paths, static_cast<std::uint32_t>(i))) {
        if (taxi.size() >= RouteGround::kPoints - 1) return at(i, Reason::InvalidWaypoint); // (and its runway's start: kPoints)
        if (std::find(taxi.begin(), taxi.end(), static_cast<std::uint32_t>(i)) != taxi.end()) return at(i, Reason::InvalidWaypoint); // (a loop)
        taxi.push_back(static_cast<std::uint32_t>(i));
        i = nextOf(waypoints, paths, static_cast<std::uint32_t>(i));
    }
    if (performance_.hovers && !taxi.empty()) return at(taxi.front(), Reason::NotImplemented); // (a rotorcraft's taxi: 4.51)
    // then its takeoff: from its runway points - its start, then its threshold or limit, the line between them - or from the
    // airfield and runway the takeoff's path it comes to names (FA's own plan: 4.40)
    std::int64_t air = -1; // (its first point in the air)
    bool fromPoints = false;
    if (i >= 0 && typeOf(waypoints[static_cast<std::size_t>(i)]) == WaypointType::RunwayStart) {
        if (performance_.hovers) return at(i, Reason::NotImplemented); // (a running takeoff's: 4.49)
        const Waypoint& start = waypoints[static_cast<std::size_t>(i)];
        std::int64_t j = nextOf(waypoints, paths, static_cast<std::uint32_t>(i)), last = -1;
        while (j >= 0 && runwayType(typeOf(waypoints[static_cast<std::size_t>(j)])) && typeOf(waypoints[static_cast<std::size_t>(j)]) != WaypointType::RunwayStart)
            last = j, j = nextOf(waypoints, paths, static_cast<std::uint32_t>(j));
        if (last < 0) return at(i, Reason::InvalidWaypoint); // (a runway's start, and no end to take off along)
        const Waypoint& end = waypoints[static_cast<std::size_t>(last)];
        g.startLatitudeRad = start.latitudeRad, g.startLongitudeRad = start.longitudeRad;
        g.courseRad = geo::wrapTwoPi(geo::bearingRad(start.latitudeRad, start.longitudeRad, end.latitudeRad, end.longitudeRad));
        g.lengthM = geo::distanceM(start.latitudeRad, start.longitudeRad, end.latitudeRad, end.longitudeRad);
        if (!(g.lengthM > 100.0)) return at(last, Reason::InvalidWaypoint);
        air = j, fromPoints = true;
    } else if (i >= 0 && pathTypeOf(paths, static_cast<std::uint32_t>(i)) == PathType::Takeoff) {
        const int k = pathOf(paths, static_cast<std::uint32_t>(i));
        const PathMetadata* meta = nullptr;
        if (extras)
            for (const PathMetadata& m : extras->pathMetadata)
                if (static_cast<int>(m.path) == k) meta = &m;
        CommandResult line;
        if (!meta || takeoffLine(static_cast<double>(meta->airfield), static_cast<double>(meta->runway), g, line) != Reason::None) return at(i, Reason::UnknownAirfield); // (4.40)
        air = i;
    } else {
        // a taxi alone (A-GRA's TAXI path, or after a landing - FA-10's), or a taxi to a point in the air: not to a takeoff
        return at(i >= 0 ? i : (taxi.empty() ? s0 : taxi.back()), taxi.empty() ? Reason::InvalidWaypoint : Reason::NotImplemented);
    }
    if (air < 0) return at(fromPoints ? i : s0, Reason::InvalidWaypoint); // (a takeoff to nowhere: its first point in the air)
    // nothing on the ground after it: a taxi's or a runway's point is flown no more once it flies (a landing's: FA-10)
    {
        std::int64_t k = air;
        for (std::size_t steps = 0; k >= 0 && steps < waypoints.size(); ++steps, k = nextOf(waypoints, paths, static_cast<std::uint32_t>(k))) {
            const WaypointType t = typeOf(waypoints[static_cast<std::size_t>(k)]);
            if (t == WaypointType::Taxi || runwayType(t)) return at(k, Reason::InvalidWaypoint);
        }
    }
    if (!performance_.hovers && !takeoffSpeeds(g)) return at(fromPoints ? i : air, Reason::NotImplemented); // (no speed to rotate at)
    // its taxi to the runway's start, checked as a taxi's (4.51): its corners' arcs fit its legs; with none, on the runway
    if (!taxi.empty()) {
        const Waypoint& first = waypoints[taxi.front()];
        const double speed = !isHold(first.speed) && first.speed >= 1.0 && first.speed <= 15.0 ? first.speed : kTaxiSpeedMs;
        taxiHandling(speed, g);
        for (std::uint32_t t : taxi) g.taxiLatitudeRad[g.taxiCount] = waypoints[t].latitudeRad, g.taxiLongitudeRad[g.taxiCount++] = waypoints[t].longitudeRad;
        g.taxiLatitudeRad[g.taxiCount] = g.startLatitudeRad, g.taxiLongitudeRad[g.taxiCount++] = g.startLongitudeRad; // (onto the runway)
        const int bad = taxiFault(state.latitudeRad, state.longitudeRad, g.taxiLatitudeRad, g.taxiLongitudeRad, g.taxiCount, g.taxiRadiusM);
        if (bad >= 0) return at(bad < static_cast<int>(taxi.size()) ? taxi[static_cast<std::size_t>(bad)] : (fromPoints ? i : air), Reason::InvalidWaypoint, Constraint::MaxTurnRate);
    } else if (!performance_.hovers && !onRunway(g, state)) {
        return at(fromPoints ? i : s0, Reason::InvalidWaypoint); // (no taxi, and not on the runway to take off from)
    }
    // the taxi off the runway its takeoff branches to, should it be rejected (VI 1.2.6: A-GRA's aborted takeoff route, a TAXI
    // path its takeoff's chains to): a branch at a runway point or at its takeoff path's first, to a taxi point
    if (extras)
        for (const RouteBranch& b : extras->branches) {
            const bool fromTakeoff = (fromPoints && runwayType(typeOf(waypoints[std::min<std::size_t>(b.point, waypoints.size() - 1)]))) ||
                                     (!fromPoints && static_cast<std::int64_t>(b.point) == air);
            if (!fromTakeoff || isHold(b.next) || b.next < 0.0 || b.next >= static_cast<double>(waypoints.size())) continue;
            std::int64_t k = static_cast<std::int64_t>(b.next);
            while (k >= 0 && taxiAt(waypoints, paths, static_cast<std::uint32_t>(k)) && g.abortCount < RouteGround::kPoints)
                g.abortLatitudeRad[g.abortCount] = waypoints[static_cast<std::size_t>(k)].latitudeRad,
                g.abortLongitudeRad[g.abortCount++] = waypoints[static_cast<std::size_t>(k)].longitudeRad, k = nextOf(waypoints, paths, static_cast<std::uint32_t>(k));
            if (g.abortCount) break;
        }
    if (g.abortCount && g.taxiRadiusM <= 0.0) taxiHandling(kTaxiSpeedMs, g);
    g.active = grounded_ = true;
    // the route checked, and flown once it flies, from its first point in the air: what its points leave out completed from
    // where it will have climbed out (not parked: its first point's speed its own now)
    route.start = static_cast<double>(air);
    sim::VehicleState from = state;
    departed(from);
    return checkRoute(route, waypoints, from, log, extras);
}

void CapabilityHost::departed(sim::VehicleState& s) const noexcept {
    const RouteGround& g = *ground_;
    const bool wing = !performance_.hovers;
    if (wing) s.latitudeRad = g.startLatitudeRad, s.longitudeRad = g.startLongitudeRad, s.eulerRad[2] = g.courseRad;
    // its points' speeds left out: its reference airspeed, as a mode given no speed flies, but no faster than 250 kt (the
    // limit below 10,000 ft: 14 CFR 91.117) - its reference is a cruise's true airspeed up high, the KC-46A's 197 m/s, which
    // overshot the first turn by 4.5 km - and no slower than its climb speed. (At its climb speed, 1.3 times its stall, the
    // EA-18G and the RQ-4B, clean and banked, lost 500 m in the first turn, into the ground.)
    const double cruise = std::isfinite(performance_.cruiseTasMs) ? std::min(performance_.cruiseTasMs, kDepartureMaxCasMs) : 0.0;
    const double up = wing ? kClimbedOutAglM : kHoverAglM, speed = wing ? std::max(g.climbCasMs, cruise) : 0.0;
    s.altitudeMslM += up, s.altitudeAglM += up;
    s.airspeedTrueMs = s.airspeedCalibratedMs = speed;
    s.velocityNedMs[0] = speed * std::cos(s.eulerRad[2]), s.velocityNedMs[1] = speed * std::sin(s.eulerRad[2]), s.velocityNedMs[2] = 0.0;
    s.eulerRad[0] = s.eulerRad[1] = 0.0;
    s.onGround = false;
}

// --- the route's start on the ground, flown -----------------------------------------------------------------------------

bool RouteBehavior::taxiPoint(const PathStore& store, std::uint32_t i) noexcept {
    if (i >= store.count) return false;
    return typeOf(store.waypoints[i]) == WaypointType::Taxi ||
           pathTypeOf(Span<const RoutePath>(store.routePaths, store.routePathCount), i) == PathType::Taxi;
}

void RouteBehavior::beginGround(const ControlContext& ctx) {
    const RouteGround& g = ctx.path->routeGround;
    if (g.taxiCount == 0) return beginTakeoff(ctx);
    taxi_->startResolved(ctx, g, g.taxiLatitudeRad, g.taxiLongitudeRad, g.taxiCount);
    ground_ = 1;
}

void RouteBehavior::beginTakeoff(const ControlContext& ctx) {
    launch_->startResolved(ctx, ctx.path->routeGround, kClimbedOutAglM, kHoverAglM);
    ground_ = 2;
}

Command RouteBehavior::ground(const ControlContext& ctx, const Command& in) {
    const RouteGround& g = ctx.path->routeGround;
    switch (ground_) {
    case 1: { // to the runway
        Command out = taxi_->update(ctx, in);
        if (taxi_->finished()) beginTakeoff(ctx);
        return out;
    }
    case 2: { // its takeoff: rejected, off the runway on its abort path, else the route failed; climbed out, its points in the air
        Command out = launch_->update(ctx, in);
        if (launch_->failure() == Reason::TakeoffRejected) {
            if (g.abortCount == 0) {
                failure_ = Reason::TakeoffRejected, ground_ = 4;
                return out;
            }
            taxi_->startResolved(ctx, g, g.abortLatitudeRad, g.abortLongitudeRad, g.abortCount);
            ground_ = 3;
            return out;
        }
        if (!launch_->finished()) return out;
        ground_ = 0;
        restart(ctx, flown_); // (planned afresh from where it is, climbed out: its first point in the air)
        return update(ctx, in);
    }
    case 3: { // off the runway, its takeoff rejected: then the route failed so
        Command out = taxi_->update(ctx, in);
        if (taxi_->finished()) failure_ = Reason::TakeoffRejected, ground_ = 5;
        return out;
    }
    case 4: return launch_->update(ctx, in); // (stopped on the runway, on its brakes)
    default: return taxi_->update(ctx, in);  // (stopped off it)
    }
}

bool RouteBehavior::handOver(BehaviorCommand& out) const {
    return ground_ == 2 && launch_->handOver(out); // (its takeoff on the runway: FA's own rest of it - 4.50)
}

bool RouteBehavior::groundProgress(ActivityProgress& out) const noexcept {
    if (ground_ == 0) return false;
    return ground_ == 2 || ground_ == 4 ? launch_->progress(out) : taxi_->progress(out);
}

} // namespace fsim::control
