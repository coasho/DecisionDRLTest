// A route that ends in a landing (docs/flight-autonomy.md, 4.59; ADR-29 FA-10d): its points in the air, then its landing - on
// its runway points, or on the runway FA's own plan's LANDING path names - flown by a recovery laid from its last point in the
// air (4.53), then its taxi off the runway on the taxi points after it.
#include "control/CapabilityHost.h"
#include "control/Route.h"
#include "control/RouteOrder.h"
#include "core/Geodesy.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

namespace fsim::control {

using namespace order;

namespace {

constexpr double kTaxiSpeedMs = 8.0;    // its taxi points' speed, where theirs is none
constexpr double kRunwayLeastM = 100.0; // a runway's points no closer than this: a line to land along

} // namespace

std::int64_t CapabilityHost::landingOf(const RouteCommand& route, Span<const Waypoint> waypoints, Span<const RoutePath> paths) const noexcept {
    if (waypoints.empty()) return -1;
    std::int64_t i = startOf(route, waypoints.size());
    for (std::size_t steps = 0; i >= 0 && steps < waypoints.size(); ++steps) {
        if (landingAt(waypoints, paths, static_cast<std::uint32_t>(i))) return i;
        i = nextOf(waypoints, paths, static_cast<std::uint32_t>(i));
    }
    return -1;
}

Reason CapabilityHost::prepareLanding(RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log,
                                      const RouteExtras* extras, std::uint32_t at) {
    CommandResult& detail = log.result;
    auto refuse = [&detail](std::int64_t point, Reason why, Constraint c = Constraint::None) {
        detail.index = static_cast<std::int16_t>(std::clamp<std::int64_t>(point, 0, 0x7FFF)), detail.constraint = c;
        return why;
    };
    const Span<const RoutePath> paths = extras ? extras->paths : Span<const RoutePath>{};
    // its points in the air: from its start to its landing, in the order flown
    std::vector<std::uint32_t> air;
    for (std::int64_t i = startOf(route, waypoints.size()); i >= 0 && i != at && air.size() < waypoints.size();
         i = nextOf(waypoints, paths, static_cast<std::uint32_t>(i)))
        air.push_back(static_cast<std::uint32_t>(i));
    if (air.empty()) return refuse(at, Reason::InvalidWaypoint); // (a landing from where it is: a recovery's)
    for (std::uint32_t a : air) {
        const WaypointType t = typeOf(waypoints[a]);
        if (t == WaypointType::Taxi || runwayType(t) || t == WaypointType::Touchdown) return refuse(a, Reason::InvalidWaypoint);
    }
    // its runway: its threshold (or touchdown) and its limit, the line between them; or the runway its LANDING path names, on
    // FA's own plan (4.40) - that path's points the recovery's, which lays its own approach to it
    RouteGround line;
    std::vector<std::uint32_t> ground; // (its landing's points, then its taxi's: the route's last)
    std::int64_t after = -1;
    if (pathTypeOf(paths, at) == PathType::Landing) {
        const int k = pathOf(paths, at);
        const PathMetadata* meta = nullptr;
        if (extras)
            for (const PathMetadata& m : extras->pathMetadata)
                if (static_cast<int>(m.path) == k) meta = &m;
        CommandResult resolved;
        if (!meta || landingLine(static_cast<double>(meta->airfield), static_cast<double>(meta->runway), line, resolved) != Reason::None)
            return refuse(at, Reason::UnknownAirfield);
        after = at;
        while (after >= 0 && pathOf(paths, static_cast<std::uint32_t>(after)) == k && ground.size() < waypoints.size())
            ground.push_back(static_cast<std::uint32_t>(after)), after = nextOf(waypoints, paths, static_cast<std::uint32_t>(after));
    } else {
        std::int64_t j = at, end = -1;
        while (j >= 0 && ground.size() < waypoints.size()) {
            const WaypointType t = typeOf(waypoints[static_cast<std::size_t>(j)]);
            if (t != WaypointType::RunwayThreshold && t != WaypointType::Touchdown && t != WaypointType::RunwayLimit) break;
            ground.push_back(static_cast<std::uint32_t>(j));
            const bool limit = t == WaypointType::RunwayLimit;
            if (limit) end = j;
            j = nextOf(waypoints, paths, static_cast<std::uint32_t>(j));
            if (limit) break;
        }
        if (end < 0) return refuse(at, Reason::InvalidWaypoint); // (a threshold, and no limit to land along)
        const Waypoint& t = waypoints[at];
        const Waypoint& e = waypoints[static_cast<std::size_t>(end)];
        line.startLatitudeRad = t.latitudeRad, line.startLongitudeRad = t.longitudeRad;
        line.courseRad = geo::wrapTwoPi(geo::bearingRad(t.latitudeRad, t.longitudeRad, e.latitudeRad, e.longitudeRad));
        line.lengthM = geo::distanceM(t.latitudeRad, t.longitudeRad, e.latitudeRad, e.longitudeRad);
        if (!(line.lengthM > kRunwayLeastM)) return refuse(end, Reason::InvalidWaypoint);
        // its elevation: its threshold's above mean sea level, else the ground's there (and above it)
        const bool above = !isHold(t.altitudeReference) && t.altitudeReference == static_cast<double>(AltitudeReference::AboveGround);
        double elevation = above ? kHold : t.altitudeM;
        if (isHold(elevation)) {
            const double groundM = sessionView_ ? sessionView_->groundM(t.latitudeRad, t.longitudeRad) : kHold;
            if (!std::isfinite(groundM)) return refuse(at, Reason::InvalidWaypoint); // (no elevation, and no ground known there)
            elevation = groundM + (above ? orHold(t.altitudeM, 0.0) : 0.0);
        }
        line.elevationM = elevation;
        after = j;
    }
    // its taxi after it: its taxi points from there, and nothing after them (a touch and go is not one)
    std::vector<std::uint32_t> taxi;
    std::int64_t i = after;
    while (i >= 0 && taxiAt(waypoints, paths, static_cast<std::uint32_t>(i))) {
        if (taxi.size() >= RouteGround::kPoints) return refuse(i, Reason::InvalidWaypoint);
        if (std::find(taxi.begin(), taxi.end(), static_cast<std::uint32_t>(i)) != taxi.end()) return refuse(i, Reason::InvalidWaypoint); // (a loop)
        taxi.push_back(static_cast<std::uint32_t>(i));
        i = nextOf(waypoints, paths, static_cast<std::uint32_t>(i));
    }
    if (i >= 0) return refuse(i, Reason::InvalidWaypoint);
    if (performance_.hovers && !taxi.empty()) return refuse(taxi.front(), Reason::NotImplemented); // (a rotorcraft's taxi: 4.51)
    // its landing's and its taxi's points the route's last, given and flown: before them, the route flown in the air
    ground.insert(ground.end(), taxi.begin(), taxi.end());
    const std::uint32_t cut = *std::min_element(ground.begin(), ground.end());
    if (*std::max_element(air.begin(), air.end()) >= cut || waypoints.size() - cut != ground.size()) return refuse(at, Reason::InvalidWaypoint);
    // what goes with its points in the air alone: no path, branch, terminator, state or loiter reaches past them
    std::vector<RoutePath> keptPaths;
    std::vector<RouteBranch> keptBranches;
    std::vector<RouteTerminator> keptTerminators;
    std::vector<RouteState> keptStates;
    std::vector<RouteLoiter> keptLoiters;
    std::vector<PathMetadata> keptMetadata;
    if (extras) {
        for (const RoutePath& p : extras->paths) {
            if (p.first + p.count <= cut) keptPaths.push_back(p);
            else if (p.first < cut) return refuse(cut, Reason::InvalidWaypoint);
        }
        for (const RouteBranch& b : extras->branches) {
            if (b.point >= cut) continue; // (its landing's own: the recovery flies its missed approach - 4.54)
            if (!isHold(b.next) && b.next >= static_cast<double>(cut)) return refuse(b.point, Reason::InvalidWaypoint);
            keptBranches.push_back(b);
        }
        for (const RouteTerminator& t : extras->terminators)
            if (t.point < cut) keptTerminators.push_back(t);
            else return refuse(t.point, Reason::InvalidWaypoint);
        for (const RouteState& s : extras->states)
            if (s.point < cut) keptStates.push_back(s);
            else return refuse(s.point, Reason::InvalidWaypoint);
        for (const RouteLoiter& l : extras->loiters)
            if (l.point < cut) keptLoiters.push_back(l);
            else return refuse(l.point, Reason::InvalidWaypoint);
        for (const PathMetadata& m : extras->pathMetadata)
            if (m.path < keptPaths.size()) keptMetadata.push_back(m);
    }
    // where it will be at its last point in the air: there, at its altitude and speed, on the course it comes to it on
    sim::VehicleState from = state;
    {
        const Waypoint& last = waypoints[air.back()];
        const double groundM = sessionView_ ? sessionView_->groundM(last.latitudeRad, last.longitudeRad) : kHold;
        const double here = state.altitudeMslM - state.altitudeAglM;
        const double below = std::isfinite(groundM) ? groundM : here;
        const bool above = !isHold(last.altitudeReference) && last.altitudeReference == static_cast<double>(AltitudeReference::AboveGround);
        from.altitudeMslM = isHold(last.altitudeM) ? state.altitudeMslM : above ? below + last.altitudeM : last.altitudeM;
        from.altitudeAglM = from.altitudeMslM - below;
        double lat0 = state.latitudeRad, lon0 = state.longitudeRad;
        if (air.size() > 1) lat0 = waypoints[air[air.size() - 2]].latitudeRad, lon0 = waypoints[air[air.size() - 2]].longitudeRad;
        const double course = geo::distanceM(lat0, lon0, last.latitudeRad, last.longitudeRad) > 1.0
                                  ? geo::bearingRad(lat0, lon0, last.latitudeRad, last.longitudeRad)
                                  : state.eulerRad[2];
        const bool given = !isHold(last.speed) && (isHold(last.speedReference) || last.speedReference == static_cast<double>(SpeedReference::TrueAirspeed) ||
                                                  last.speedReference == static_cast<double>(SpeedReference::CalibratedAirspeed));
        const double speed = given ? last.speed : state.airspeedTrueMs;
        from.latitudeRad = last.latitudeRad, from.longitudeRad = last.longitudeRad;
        from.eulerRad[0] = from.eulerRad[1] = 0.0, from.eulerRad[2] = course;
        from.airspeedTrueMs = from.airspeedCalibratedMs = speed;
        from.velocityNedMs[0] = speed * std::cos(course), from.velocityNedMs[1] = speed * std::sin(course), from.velocityNedMs[2] = 0.0;
        from.onGround = false;
    }
    // the recovery it lands by, laid from there as a recovery's NEW lays it (4.53): its approach, its go-arounds, its missed
    // approach on FA's own landing path for that runway
    BehaviorCommand recovery;
    recovery.id = "recovery";
    recovery.params["airfield"] = line.airfield, recovery.params["runway"] = line.runway;
    {
        CommandResult laid;
        CheckLog landingLog{laid, log.range, nullptr};
        routeEnd(kInLanding, true);
        const Reason why = layRecovery(recovery, line, from, landingLog);
        routeEnd(kInLanding, false);
        if (why != Reason::None || landingLog.refused != Reason::None) return refuse(at, why != Reason::None ? why : landingLog.refused);
    }
    if (!ends_) ends_ = std::make_unique<RouteEnds>();
    if (!ends_->approach) ends_->approach = std::make_unique<route::Plan>();
    std::swap(routePlan_, ends_->approach); // (its approach kept: the route's own plan is checked next)
    RouteLanding& landing = ends_->landing;
    landing.active = true, landing.recovery = recovery, landing.path = nullptr, landing.taxi = RouteGround{};
    // its taxi after it, checked as a taxi's (4.51): from the runway's end, its corners' arcs fit its legs at its first point's speed
    if (!taxi.empty()) {
        const Waypoint& first = waypoints[taxi.front()];
        const double speed = !isHold(first.speed) && first.speed >= 1.0 && first.speed <= 15.0 ? first.speed : kTaxiSpeedMs;
        RouteGround& g = landing.taxi;
        taxiHandling(speed, g);
        for (std::uint32_t t : taxi) g.taxiLatitudeRad[g.taxiCount] = waypoints[t].latitudeRad, g.taxiLongitudeRad[g.taxiCount++] = waypoints[t].longitudeRad;
        double endLat, endLon;
        geo::offsetLatLon(line.startLatitudeRad, line.startLongitudeRad, line.lengthM * std::cos(line.courseRad), line.lengthM * std::sin(line.courseRad),
                          endLat, endLon);
        const int bad = taxiFault(endLat, endLon, g.taxiLatitudeRad, g.taxiLongitudeRad, g.taxiCount, g.taxiRadiusM);
        if (bad >= 0) {
            std::swap(routePlan_, ends_->approach);
            return refuse(taxi[static_cast<std::size_t>(bad)], Reason::InvalidWaypoint, Constraint::MaxTurnRate);
        }
    }
    // its points in the air, checked as a route's (its own checks look for no landing) - one that leads on into its landing
    // ending the route there
    std::vector<Waypoint> flown(waypoints.begin(), waypoints.begin() + cut);
    for (Waypoint& w : flown)
        if (!isHold(w.next) && w.next >= static_cast<double>(cut)) w.next = kHold;
    RouteExtras own = extras ? *extras : RouteExtras{};
    own.paths = keptPaths, own.branches = keptBranches, own.terminators = keptTerminators, own.states = keptStates, own.loiters = keptLoiters;
    own.pathMetadata = keptMetadata;
    routeEnd(kInLanding, true);
    const Reason why = checkRoute(route, Span<const Waypoint>(flown), state, log, extras ? &own : nullptr);
    routeEnd(kInLanding, false);
    if (why != Reason::None) return why;
    routeEnd(kLands, true);
    return Reason::None;
}

void CapabilityHost::carryLanding(Behavior& behavior) {
    // a route that ends in a landing, installed (a NEW's, or one that waited): its recovery allocated now, between steps
    if (std::string_view(behavior.id()) == "route") static_cast<RouteBehavior&>(behavior).carryLanding();
}

// --- its landing, flown -------------------------------------------------------------------------------------------------

void RouteBehavior::carryLanding() {
    if (!landing_) landing_ = std::make_unique<RecoveryBehavior>();
}

Command RouteBehavior::update(const ControlContext& ctx, const Command& in) {
    if (lands_ >= 2) return landed(ctx, in);
    Command out = flyRoute(ctx, in);
    // its last point in the air reached: its landing (4.59)
    if (lands_ == 1 && finished_ && failure_ == Reason::None && ctx.path->landing) return beginLanding(ctx, in);
    return out;
}

Command RouteBehavior::beginLanding(const ControlContext& ctx, const Command& in) {
    ControlContext there = ctx;
    there.path = ctx.path->landing->path;
    landing_->begin(there, ctx.path->landing->recovery);
    finished_ = false, lands_ = 2;
    return landed(ctx, in);
}

Command RouteBehavior::landed(const ControlContext& ctx, const Command& in) {
    const RouteLanding& l = *ctx.path->landing;
    if (lands_ == 2) { // landing: then its taxi off the runway, or done
        ControlContext there = ctx;
        there.path = l.path;
        Command out = landing_->update(there, l.recovery);
        if (const Reason why = landing_->failure(); why != Reason::None) failure_ = why;
        if (failure_ != Reason::None || !landing_->finished()) return out;
        if (l.taxi.taxiCount == 0) {
            lands_ = 4, finished_ = true;
            return out;
        }
        taxi_->startResolved(ctx, l.taxi, l.taxi.taxiLatitudeRad, l.taxi.taxiLongitudeRad, l.taxi.taxiCount);
        lands_ = 3;
    }
    Command out = taxi_->update(ctx, in);
    if (lands_ == 3 && taxi_->finished()) lands_ = 4, finished_ = true;
    return out;
}

void RouteBehavior::configure(ActuatorCommand& out) const noexcept {
    if (lands_ == 2) landing_->configure(out); // (its gear, flaps and brakes as its recovery sets them: 4.53)
}

double RouteBehavior::speedbrake() const noexcept {
    return lands_ == 2 ? landing_->speedbrake() : lands_ >= 3 ? 0.0 : kHold; // (closed for its taxi)
}

bool RouteBehavior::landedProgress(ActivityProgress& out) const noexcept {
    return lands_ == 2 ? landing_->progress(out) : taxi_->progress(out);
}

} // namespace fsim::control
