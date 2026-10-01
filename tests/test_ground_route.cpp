// A route that starts on the ground (docs/flight-autonomy.md, 4.52; ADR-29 FA-9d: WPT-26, LCH-01, LCH-02, CAP-02): its taxi
// points flown as a taxi to the runway, its runway points - or the airfield and runway its takeoff's path names, on FA's own
// plan - as a takeoff, then its points in the air; a rejected takeoff taxies off the runway on the path its takeoff branches
// to, and the route fails. The NEW's refusals name their points.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kHalfWidthM = 22.5; // half a 45 m runway: FA-9's acceptance

/// The layout, north and east metres from where the aircraft is parked facing north: a taxi north 60 m and east 150 m onto
/// the runway's start, the runway north from it; then two points in the air 600 m above the field, a right turn between.
const std::vector<std::pair<double, double>> kTaxi = {{60.0, 0.0}, {60.0, 150.0}};
constexpr double kStartN = 160.0, kStartE = 150.0;
const std::vector<std::pair<double, double>> kAir = {{5000.0, 150.0}, {5000.0, 9000.0}}; // (a heavy's 90 deg turn at 250 kt: 3.7 km wide)
constexpr double kAirAglM = 600.0;

/// An aircraft parked on the ground at the default place, facing north, settled on its wheels.
std::uint32_t parked(session::World& w, const std::string& type) {
    session::VehicleSpec s;
    s.name = type;
    s.type = "jsbsim:" + type;
    s.initial.onGround = true;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    w.step(stepsFor(w, 1.0));
    return id;
}

struct Field {
    double lat0 = 0.0, lon0 = 0.0, elevationM = 0.0;
    Waypoint at(double north, double east, WaypointType type = WaypointType::NavOnly) const {
        Waypoint p;
        p.latitudeRad = lat0 + north / kEarthM, p.longitudeRad = lon0 + east / (kEarthM * std::cos(lat0));
        if (type != WaypointType::NavOnly) p.waypointType = static_cast<double>(type);
        return p;
    }
    Waypoint air(double north, double east) const {
        Waypoint p = at(north, east);
        p.altitudeM = elevationM + kAirAglM, p.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        return p;
    }
};

/// Where `id` is parked; airfield 7's runway 3 from the runway's start, `lengthM` north.
Field field(session::World& w, std::uint32_t id, double lengthM = 3000.0) {
    const sim::VehicleState& s = *w.vehicleState(id);
    Field f{s.latitudeRad, s.longitudeRad, s.altitudeMslM - s.altitudeAglM};
    const Waypoint start = f.at(kStartN, kStartE), limit = f.at(kStartN + lengthM, kStartE);
    Runway r;
    r.id = 3;
    r.takeoff.start = RunwayPoint{start.latitudeRad, start.longitudeRad, f.elevationM};
    r.takeoff.threshold = r.takeoff.start;
    r.takeoff.limit = RunwayPoint{limit.latitudeRad, limit.longitudeRad, f.elevationM};
    Airfield a;
    a.id = 7;
    a.runways = {r};
    REQUIRE(w.loadAirfield(id, a) == Reason::None);
    return f;
}

/// The route as points: its taxi points, the runway's start and limit (`lengthM` along), its points in the air.
std::vector<Waypoint> pointsRoute(const Field& f, double lengthM = 3000.0) {
    std::vector<Waypoint> out;
    for (const auto& [n, e] : kTaxi) out.push_back(f.at(n, e, WaypointType::Taxi));
    out.push_back(f.at(kStartN, kStartE, WaypointType::RunwayStart));
    out.push_back(f.at(kStartN + lengthM, kStartE, WaypointType::RunwayLimit));
    for (const auto& [n, e] : kAir) out.push_back(f.air(n, e));
    return out;
}

double cornerRadius(session::World& w, std::uint32_t id) {
    const double tightest = w.profile(id)->envelope.groundTurnRadiusM;
    return std::max(1.25 * (std::isfinite(tightest) ? tightest : 15.0), 8.0);
}

struct Flown {
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;
    double worstTaxiM = 0.0, worstLineM = 0.0, liftOffCasMs = kHold;
    double north = 0.0, east = 0.0, aglM = 0.0, nearestM = 1e9; ///< where it ended; the nearest it came to its last point
    bool onGround = true;
};

/// A route's flight from its NEW (`activity`) on `id` to within 300 m of its last point (900 s at most), measured: its taxi
/// against the path drawn through `taxi` from where it stood, its roll against the runway's centre line, its lift-off speed.
Flown fly(session::World& w, std::uint32_t id, ActivityId activity, const Field& f) {
    const std::vector<std::pair<double, double>> taxi = {{0.0, 0.0}, kTaxi[0], kTaxi[1], {kStartN, kStartE}}; // (from where it stood)
    const double radius = cornerRadius(w, id);
    Flown out;
    for (double t = 0.0; t < 900.0 && w.activity(activity)->live(); t += 0.1) {
        w.step(stepsFor(w, 0.1));
        const sim::VehicleState& s = *w.vehicleState(id);
        double n, e;
        offset(s, f.lat0, f.lon0, n, e);
        if (s.onGround && n < kStartN - 2.0 && (n > 20.0 || e > 1.0)) out.worstTaxiM = std::max(out.worstTaxiM, taxiPathDistance(taxi, radius, n, e));
        if (s.onGround && n > kStartN + 20.0) out.worstLineM = std::max(out.worstLineM, std::abs(e - kStartE));
        if (std::isnan(out.liftOffCasMs) && !s.onGround && s.altitudeAglM > 3.0) out.liftOffCasMs = s.airspeedCalibratedMs;
        out.nearestM = std::min(out.nearestM, std::hypot(n - kAir.back().first, e - kAir.back().second));
        if (out.nearestM < 300.0) break;
    }
    const sim::VehicleState& s = *w.vehicleState(id);
    offset(s, f.lat0, f.lon0, out.north, out.east);
    out.aglM = s.altitudeMslM - f.elevationM, out.onGround = s.onGround;
    out.state = w.activity(activity)->state, out.reason = w.activity(activity)->reason;
    return out;
}

} // namespace

TEST_CASE("ground route: a wing taxies its taxi points, takes off along its runway points and flies its points in the air, "
          "calm and in a 10 m/s crosswind (FA-9d)",
          "[modes][ground_route]") {
    // a light single, a fighter, a heavy, and the two that flew into the ground at their climb speeds in the first turn
    for (const char* type : {"c172", "f16c", "kc135r", "ea18g", "rq4b"}) {
        for (double wind : {0.0, 10.0}) {
            if (wind > 0.0 && (std::string(type) == "ea18g" || std::string(type) == "rq4b")) continue;
            INFO(type << " in a " << wind << " m/s crosswind");
            session::World w(options(("ground-route-" + std::string(type)).c_str()));
            if (wind > 0.0) setWind(w, 270.0, wind);
            const auto id = parked(w, type);
            const Field f = field(w, id);
            const std::vector<Waypoint> route = pointsRoute(f);
            const CommandResult r = w.submit(id, RouteCommand{}, route);
            INFO(reasonName(r.reason) << " at " << r.index);
            REQUIRE(r.accepted());
            const Flown flown = fly(w, id, r.activity, f);
            CHECK(flown.worstTaxiM < 2.0);           // (the KC-135R's 0.63 m)
            CHECK(flown.worstLineM < kHalfWidthM);   // (within 3.4 m across: the C172)
            CHECK(flown.liftOffCasMs > 0.0);
            CHECK((flown.state == ActivityState::Active || flown.state == ActivityState::Completed));
            CHECK(!flown.onGround);
            CHECK(flown.nearestM < 500.0); // (the KC-135R, its 90 deg turn 3.7 km wide at 250 kt, 330 m off as it passed)
            CHECK(std::abs(flown.aglM - kAirAglM) < 50.0); // at its points' height
        }
    }
}

TEST_CASE("ground route: FA's own plan flies its taxi path, then its takeoff path's runway, then its points; a rotorcraft "
          "lifts from where it stands (FA-9d, 4.40)",
          "[modes][ground_route]") {
    {
        session::World w(options("ground-route-plan"));
        const auto id = parked(w, "f16c");
        const Field f = field(w, id);
        RoutePlan plan;
        plan.id = 41;
        for (const auto& [n, e] : kTaxi) plan.waypoints.push_back(f.at(n, e));
        for (const auto& [n, e] : kAir) plan.waypoints.push_back(f.air(n, e));
        plan.waypoints[1].next = 2, plan.waypoints[2].next = 3; // (its paths chained: 4.36)
        plan.paths = {RoutePath{1, static_cast<double>(PathType::Taxi), 0, 2}, RoutePath{2, static_cast<double>(PathType::Takeoff), 2, 1},
                      RoutePath{3, static_cast<double>(PathType::Primary), 3, 1}};
        PathMetadata takeoff;
        takeoff.path = 1, takeoff.airfield = 7, takeoff.runway = 3;
        plan.pathMetadata = {takeoff};
        REQUIRE(w.loadPlan(id, plan) == Reason::None);
        const PlanCommandResult ready = w.planCommand(id, 41, PlanCommand::PrepareForActivation);
        INFO(reasonName(ready.reason) << " at " << ready.check.index);
        REQUIRE(ready.completed);
        const PlanCommandResult a = w.planCommand(id, 41, PlanCommand::Activate);
        REQUIRE(a.completed);
        const Flown flown = fly(w, id, a.check.activity, f);
        CHECK(flown.worstTaxiM < 2.0);
        CHECK(flown.worstLineM < 1.0);
        CHECK(!flown.onGround);
        CHECK(flown.nearestM < 300.0);
        CHECK(std::abs(flown.aglM - kAirAglM) < 50.0);
    }
    {
        // the UH-1H: its takeoff's path first, its lift 10 m over where it stands, then its point
        session::World w(options("ground-route-rotor"));
        const auto id = parked(w, "uh1h");
        const Field f = field(w, id);
        RoutePlan plan;
        plan.id = 42;
        plan.waypoints = {f.air(1500.0, 0.0), f.air(1500.0, 1000.0)};
        plan.waypoints[0].next = 1;
        plan.paths = {RoutePath{1, static_cast<double>(PathType::Takeoff), 0, 1}, RoutePath{2, static_cast<double>(PathType::Primary), 1, 1}};
        PathMetadata takeoff;
        takeoff.path = 0, takeoff.airfield = 7, takeoff.runway = 3;
        plan.pathMetadata = {takeoff};
        REQUIRE(w.loadPlan(id, plan) == Reason::None);
        REQUIRE(w.planCommand(id, 42, PlanCommand::PrepareForActivation).completed);
        const PlanCommandResult a = w.planCommand(id, 42, PlanCommand::Activate);
        REQUIRE(a.completed);
        double hoverAgl = 0.0, drift = 0.0;
        for (double t = 0.0; t < 30.0; t += 0.5) { // (lifting: straight up, over where it stood)
            w.step(stepsFor(w, 0.5));
            double n, e;
            offset(*w.vehicleState(id), f.lat0, f.lon0, n, e);
            drift = std::max(drift, std::hypot(n, e));
            hoverAgl = std::max(hoverAgl, w.vehicleState(id)->altitudeAglM);
        }
        CHECK(drift < 10.0);
        CHECK(hoverAgl > 8.0);
        for (double t = 0.0; t < 400.0 && w.activity(a.check.activity)->live(); t += 1.0) w.step(stepsFor(w, 1.0));
        double n, e;
        offset(*w.vehicleState(id), f.lat0, f.lon0, n, e);
        CHECK(std::hypot(n - 1500.0, e - 1000.0) < 100.0); // (at its last point: hovering there, the route complete)
        CHECK(w.activity(a.check.activity)->state == ActivityState::Completed);
    }
}

TEST_CASE("ground route: a rejected takeoff taxies off the runway on the path its takeoff branches to, then the route fails; "
          "with none it stops on the runway (FA-9d, 4.50)",
          "[modes][ground_route]") {
    for (bool abortPath : {true, false}) {
        INFO((abortPath ? "with" : "without") << " an abort path");
        session::World w(options("ground-route-reject"));
        const auto id = parked(w, "c172");
        const Field f = field(w, id, 250.0); // (a runway too short: rejected early in the roll)
        std::vector<Waypoint> route = pointsRoute(f, 250.0);
        std::vector<RouteBranch> branches;
        if (abortPath) { // east 100 m off the runway, 200 m along it: a taxi path the runway's start branches to
            route.back().next = -1;
            route.push_back(f.at(kStartN + 200.0, kStartE + 100.0, WaypointType::Taxi));
            route.push_back(f.at(kStartN + 300.0, kStartE + 100.0, WaypointType::Taxi));
            RouteBranch b;
            b.point = 2, b.next = static_cast<double>(route.size() - 2);
            branches.push_back(b);
        }
        const CommandResult r = w.submit(id, RouteCommand{}, route, {}, {}, {}, {}, branches);
        REQUIRE(r.accepted());
        const Flown flown = fly(w, id, r.activity, f);
        CHECK(flown.state == ActivityState::Failed);
        CHECK(flown.reason == Reason::TakeoffRejected);
        CHECK(flown.onGround);
        if (abortPath) CHECK(std::hypot(flown.north - (kStartN + 300.0), flown.east - (kStartE + 100.0)) < 2.0);
        else CHECK((std::abs(flown.east - kStartE) < 2.0 && flown.north < kStartN + 250.0));
        CHECK(groundSpeed(*w.vehicleState(id)) < 0.5);
    }
    {
        // a policy's CANCEL in the roll: FA's own rejection, as a launch's (4.50)
        session::World w(options("ground-route-cancel"));
        const auto id = parked(w, "c172");
        const Field f = field(w, id);
        const CommandResult r = w.submit(id, RouteCommand{}, pointsRoute(f));
        REQUIRE(r.accepted());
        for (double t = 0.0; t < 200.0 && w.vehicleState(id)->airspeedCalibratedMs < 12.0; t += 0.1) w.step(stepsFor(w, 0.1));
        double n, e;
        offset(*w.vehicleState(id), f.lat0, f.lon0, n, e);
        REQUIRE(n > kStartN); // (rolling)
        const CommandResult c = w.cancel(r.activity);
        CHECK(c.status == CommandStatus::Canceled);
        REQUIRE(c.other != 0);
        for (double t = 0.0; t < 120.0 && w.activity(c.other)->live(); t += 0.5) w.step(stepsFor(w, 0.5));
        CHECK(w.activity(c.other)->state == ActivityState::Completed);
        CHECK(w.activity(c.other)->source == Source::Autopilot);
        CHECK(w.vehicleState(id)->onGround);
        CHECK(groundSpeed(*w.vehicleState(id)) < 0.5);
    }
}

TEST_CASE("ground route: its NEW checks its taxi and its takeoff; only on the ground; its axes and support rows (FA-9d)",
          "[modes][ground_route]") {
    session::World w(options("ground-route-refusals"));
    const auto id = parked(w, "c172");
    const Field f = field(w, id);
    const auto refused = [&](const std::vector<Waypoint>& route, Reason why, std::int16_t index, Span<const RoutePath> paths = {}) {
        const CommandResult r = w.submit(id, RouteCommand{}, route, {}, {}, {}, paths);
        INFO("refused " << reasonName(r.reason) << " at " << r.index);
        CHECK(!r.accepted());
        CHECK(r.reason == why);
        CHECK(r.index == index);
        return r;
    };
    const std::vector<Waypoint> full = pointsRoute(f);
    // a taxi alone, or to a point in the air with no runway between: not to a takeoff (a taxi after a landing: FA-10's)
    refused({full[0], full[1]}, Reason::NotImplemented, 1);
    refused({full[0], full[1], full[4]}, Reason::NotImplemented, 2);
    // a runway's start and no end; a runway with no point in the air after it
    refused({full[0], full[1], full[2], full[4]}, Reason::InvalidWaypoint, 2);
    refused({full[0], full[1], full[2], full[3]}, Reason::InvalidWaypoint, 2);
    // no taxi, and not on the runway: parked 150 m from it
    refused({full[2], full[3], full[4]}, Reason::InvalidWaypoint, 0);
    // a taxi point after it flies
    refused({full[0], full[1], full[2], full[3], full[4], f.at(300.0, 300.0, WaypointType::Taxi)}, Reason::InvalidWaypoint, 5);
    // a corner the C172's 12 m arcs cannot turn: back on itself
    const CommandResult back = refused({f.at(60.0, 0.0, WaypointType::Taxi), f.at(20.0, 0.0, WaypointType::Taxi), full[2], full[3], full[4]},
                                       Reason::InvalidWaypoint, 0);
    CHECK(back.constraint == Constraint::MaxTurnRate);
    // a takeoff's path with no airfield and runway: a route's paths carry no metadata (a plan's do)
    {
        const RoutePath paths[] = {RoutePath{1, static_cast<double>(PathType::Takeoff), 0, 1}, RoutePath{2, static_cast<double>(PathType::Primary), 1, 1}};
        std::vector<Waypoint> route = {f.air(5000.0, 150.0), f.air(5000.0, 3000.0)};
        route[0].next = 1;
        refused(route, Reason::UnknownAirfield, 0, Span<const RoutePath>(paths, 2));
    }
    // its lifecycle: NEW on the primary axes, the gear, flaps and brakes; active after a step; canceled at the taxi, nothing handed on
    {
        const CommandResult r = w.submit(id, RouteCommand{}, full);
        REQUIRE(r.accepted());
        const AxisMask ground = axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes);
        CHECK((w.activity(r.activity)->axes & (kPrimaryAxes | ground)) == (kPrimaryAxes | ground));
        w.step();
        CHECK(w.activity(r.activity)->state == ActivityState::Active);
        const CommandResult c = w.cancel(r.activity);
        CHECK(c.status == CommandStatus::Canceled);
        CHECK(c.other == 0);
    }
    // a route that starts in the air, on the ground: still waits for it to fly (4.5)
    CHECK(w.submit(id, RouteCommand{}, std::vector<Waypoint>{full[4], full[5]}).reason == Reason::OnGround);
    // in the air: a route that starts on the ground cannot
    const auto flying = wing(w, "c172", 1000.0, 50.0, 1);
    CHECK(w.submit(flying, RouteCommand{}, full).reason == Reason::Airborne);
    CHECK(w.support(id, "fsim.guidance.route/waypoint_type/taxi")->support == Support::Partial);
    CHECK(w.support(id, "fsim.guidance.route/waypoint_type/runway")->support == Support::Supported);
    CHECK(w.support(id, "fsim.guidance.route/waypoint_type/takeoff")->support == Support::Supported);
    // a rotorcraft: no taxi, no running takeoff
    session::World r(options("ground-route-rotor-refusals"));
    const auto heli = parked(r, "uh60");
    const Field g = field(r, heli);
    CHECK(r.submit(heli, RouteCommand{}, pointsRoute(g)).reason == Reason::NotImplemented);
    CHECK(r.support(heli, "fsim.guidance.route/waypoint_type/taxi")->support == Support::NotImplemented);
    CHECK(r.support(heli, "fsim.guidance.route/waypoint_type/runway")->support == Support::NotImplemented);
    CHECK(r.support(heli, "fsim.guidance.route/waypoint_type/takeoff")->support == Support::Supported);
}
