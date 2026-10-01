// A route that ends in a landing (docs/flight-autonomy.md, 4.59; ADR-29 FA-10d): its points in the air, then its landing - on its
// runway's threshold and limit points, or on the runway FA's own plan's LANDING path names - flown by a recovery laid from its
// last point in the air (4.53), then its taxi off the runway on the taxi points after it. With a start on the ground (4.52) a
// whole flight: taxi, takeoff, points, landing, taxi. The NEW's refusals name their points.
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

constexpr double kSinkLimitMs = 3.2; // the recovery's acceptance (4.53)
constexpr double kThresholdN = 20000.0, kLengthM = 3000.0;
const std::vector<std::pair<double, double>> kTaxiAfter = {{kThresholdN + 2900.0, 120.0}, {kThresholdN + 2700.0, 250.0}};

struct Field {
    double lat0 = 0.0, lon0 = 0.0, elevationM = 0.0;
    Waypoint at(double north, double east, WaypointType type = WaypointType::NavOnly) const {
        Waypoint p;
        p.latitudeRad = lat0 + north / kEarthM, p.longitudeRad = lon0 + east / (kEarthM * std::cos(lat0));
        if (type != WaypointType::NavOnly) p.waypointType = static_cast<double>(type);
        return p;
    }
    Waypoint air(double north, double east, double aglM) const {
        Waypoint p = at(north, east);
        p.altitudeM = elevationM + aglM, p.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        return p;
    }
    Waypoint runway(double north, WaypointType type) const {
        Waypoint p = at(north, 0.0, type);
        p.altitudeM = elevationM, p.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        return p;
    }
};

/// Where `id` is, at its field's elevation.
Field fieldOf(session::World& w, std::uint32_t id) {
    const sim::VehicleState& s = *w.vehicleState(id);
    return Field{s.latitudeRad, s.longitudeRad, s.altitudeMslM - s.altitudeAglM};
}

/// Its points in the air - 5 km north and 3 km east 900 m up, 10 km north and 6 km east 800 m up - its runway's threshold 20 km
/// north and its limit 3 km on, then its taxi off it to the east.
std::vector<Waypoint> landingRoute(const Field& f) {
    std::vector<Waypoint> out = {f.air(5000.0, 3000.0, 900.0), f.air(10000.0, 6000.0, 800.0), f.runway(kThresholdN, WaypointType::RunwayThreshold),
                                 f.runway(kThresholdN + kLengthM, WaypointType::RunwayLimit)};
    for (const auto& [n, e] : kTaxiAfter) out.push_back(f.at(n, e, WaypointType::Taxi));
    return out;
}

struct Landed {
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;
    bool touched = false;
    double along = 0.0, sink = 0.0, worstAcross = 0.0, north = 0.0, east = 0.0, endSpeed = 0.0;
};

/// `id`'s route flown to its end (2,400 s at most), measured from its threshold: where it touched down and how fast it sank, the
/// worst it was off the centre line rolling out, where it ended.
Landed fly(session::World& w, std::uint32_t id, ActivityId activity, const Field& f) {
    Landed out;
    double sink = 0.0;
    bool flown = false, stopped = false; // (in the air once; stopped on the runway, its rollout over)
    const double thrLat = f.lat0 + kThresholdN / kEarthM;
    for (double t = 0.0; t < 2400.0 && w.activity(activity)->live(); t += 0.1) {
        w.step(stepsFor(w, 0.1));
        const sim::VehicleState& s = *w.vehicleState(id);
        double n, e;
        offset(s, thrLat, f.lon0, n, e);
        flown = flown || s.altitudeAglM > 50.0;
        if (flown && !out.touched && !s.onGround) sink = s.velocityNedMs[2];
        if (flown && !out.touched && s.onGround) out.touched = true, out.along = n, out.sink = sink;
        if (out.touched && !stopped) out.worstAcross = std::max(out.worstAcross, std::abs(e)); // (its rollout)
        stopped = stopped || (out.touched && groundSpeed(s) < 0.5);
    }
    const sim::VehicleState& s = *w.vehicleState(id);
    offset(s, f.lat0, f.lon0, out.north, out.east);
    out.state = w.activity(activity)->state, out.reason = w.activity(activity)->reason, out.endSpeed = groundSpeed(s);
    return out;
}

/// A wing in the air at the origin, 1,000 m up, at 1.4 times its least speed (a probe's: its loops' speeds).
std::uint32_t flying(session::World& w, const std::string& type) {
    const auto probe = wing(w, type, 1000.0, 100.0, 0);
    const double tas = std::max(1.4 * w.performance(probe)->minCasMs, std::min(60.0, w.performance(probe)->cruiseTasMs));
    REQUIRE(w.removeVehicle(probe));
    const auto id = wing(w, type, 1000.0, tas, 1);
    w.step(stepsFor(w, 0.5));
    return id;
}

} // namespace

TEST_CASE("landing route: a wing flies its points, lands on its runway points in the touchdown zone and taxies off on its taxi "
          "points, calm and in a 10 m/s crosswind (4.59)",
          "[modes][landing_route]") {
    // a light single, a fighter, a heavy, the C-130J (its flight idle: 4.56)
    for (const char* type : {"c172", "f16c", "kc135r", "c130j"}) {
        for (double wind : {0.0, 10.0}) {
            if (wind > 0.0 && std::string(type) == "c172") continue; // (beyond its 15 kt: 4.54)
            INFO(type << " in a " << wind << " m/s crosswind");
            session::World w(options(("landing-route-" + std::string(type)).c_str()));
            if (wind > 0.0) setWind(w, 270.0, wind);
            const auto id = flying(w, type);
            const Field f = fieldOf(w, id);
            const std::vector<Waypoint> points = landingRoute(f);
            const CommandResult r = w.submit(id, RouteCommand{}, Span<const Waypoint>(points.data(), points.size()));
            INFO(reasonName(r.reason) << " at " << r.index);
            REQUIRE(r.accepted());
            // its gear, flaps, brakes and speedbrake its recovery's
            const AxisMask config = axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes) | axisBit(Axis::Speedbrake);
            CHECK((w.activity(r.activity)->axes & config) == config);
            const Landed l = fly(w, id, r.activity, f);
            INFO("touched down " << l.along << " m along, sinking " << l.sink << " m/s; ended " << l.north << " n " << l.east << " e");
            CHECK(l.state == ActivityState::Completed);
            CHECK(l.touched);
            CHECK((l.along > 0.0 && l.along < 900.0));
            CHECK(l.sink < kSinkLimitMs);
            CHECK(l.worstAcross < 22.5);
            CHECK(std::hypot(l.north - kTaxiAfter.back().first, l.east - kTaxiAfter.back().second) < 1.0); // (its taxi's end)
            CHECK(l.endSpeed < 0.5);
        }
    }
}

TEST_CASE("landing route: FA's own plan lands on the runway its LANDING path names and taxies off on its TAXI path (4.59, 4.40)",
          "[modes][landing_route]") {
    session::World w(options("landing-route-plan"));
    const auto id = flying(w, "f16c");
    const Field f = fieldOf(w, id);
    Runway r;
    r.id = 3;
    const Waypoint thr = f.at(kThresholdN, 0.0), lim = f.at(kThresholdN + kLengthM, 0.0);
    r.landing.start = r.landing.threshold = RunwayPoint{thr.latitudeRad, thr.longitudeRad, f.elevationM};
    r.landing.limit = RunwayPoint{lim.latitudeRad, lim.longitudeRad, f.elevationM};
    Airfield a;
    a.id = 7;
    a.runways = {r};
    REQUIRE(w.loadAirfield(id, a) == Reason::None);
    RoutePlan plan;
    plan.id = 51;
    plan.waypoints = {f.air(5000.0, 3000.0, 900.0), f.air(10000.0, 6000.0, 800.0), f.air(15000.0, 0.0, 300.0)};
    for (const auto& [n, e] : kTaxiAfter) plan.waypoints.push_back(f.at(n, e));
    plan.waypoints[1].next = 2, plan.waypoints[2].next = 3; // (its paths chained: 4.36)
    plan.paths = {RoutePath{1, static_cast<double>(PathType::Primary), 0, 2}, RoutePath{2, static_cast<double>(PathType::Landing), 2, 1},
                  RoutePath{3, static_cast<double>(PathType::Taxi), 3, 2}};
    PathMetadata landing;
    landing.path = 1, landing.airfield = 7, landing.runway = 3;
    plan.pathMetadata = {landing};
    REQUIRE(w.loadPlan(id, plan) == Reason::None);
    const PlanCommandResult ready = w.planCommand(id, 51, PlanCommand::PrepareForActivation);
    INFO(reasonName(ready.reason) << " at " << ready.check.index);
    REQUIRE(ready.completed);
    const PlanCommandResult go = w.planCommand(id, 51, PlanCommand::Activate);
    REQUIRE(go.completed);
    const Landed l = fly(w, id, go.check.activity, f);
    INFO("touched down " << l.along << " m along, sinking " << l.sink << " m/s");
    CHECK(l.state == ActivityState::Completed);
    CHECK((l.touched && l.along > 0.0 && l.along < 900.0 && l.sink < kSinkLimitMs));
    CHECK(std::hypot(l.north - kTaxiAfter.back().first, l.east - kTaxiAfter.back().second) < 1.0);
    // the route's landing waypoint types supported on a wing
    CHECK(w.support(id, "fsim.guidance.route/waypoint_type/landing")->support == Support::Supported);
}

TEST_CASE("landing route: a whole flight - taxi, takeoff, points, landing, taxi - as one route (4.52, 4.59)", "[modes][landing_route]") {
    session::World w(options("landing-route-whole"));
    session::VehicleSpec s;
    s.name = "viper";
    s.type = "jsbsim:f16c";
    s.initial.onGround = true;
    s.initial.airspeedTrueMs = 0.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    w.step(stepsFor(w, 1.0));
    const Field f = fieldOf(w, id);
    std::vector<Waypoint> points = {f.at(60.0, 0.0, WaypointType::Taxi), f.at(60.0, 150.0, WaypointType::Taxi), f.at(160.0, 150.0, WaypointType::RunwayStart),
                                    f.at(3160.0, 150.0, WaypointType::RunwayLimit), f.air(9000.0, 3000.0, 800.0), f.air(12000.0, 3000.0, 700.0)};
    const std::vector<Waypoint> landing = landingRoute(f);
    points.insert(points.end(), landing.begin() + 2, landing.end()); // (the threshold 20 km north, its limit, its taxi off)
    const CommandResult r = w.submit(id, RouteCommand{}, Span<const Waypoint>(points.data(), points.size()));
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    const Landed l = fly(w, id, r.activity, f);
    INFO("touched down " << l.along << " m along, sinking " << l.sink << " m/s; ended " << l.north << " n " << l.east << " e");
    CHECK(l.state == ActivityState::Completed);
    CHECK((l.touched && l.along > 0.0 && l.along < 900.0 && l.sink < kSinkLimitMs));
    CHECK(std::hypot(l.north - kTaxiAfter.back().first, l.east - kTaxiAfter.back().second) < 1.0);
}

TEST_CASE("landing route: its NEW names the point it cannot land or taxi at (4.59)", "[modes][landing_route]") {
    session::World w(options("landing-route-refusals"));
    const auto id = flying(w, "c172");
    const Field f = fieldOf(w, id);
    const auto refused = [&](const std::vector<Waypoint>& points, Reason why, std::int16_t point) {
        const CommandResult r = w.submit(id, RouteCommand{}, Span<const Waypoint>(points.data(), points.size()));
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(!r.accepted());
        CHECK(r.reason == why);
        CHECK(r.index == point);
    };
    std::vector<Waypoint> p = landingRoute(f);
    // a threshold and no limit to land along
    refused({p[0], p[1], p[2]}, Reason::InvalidWaypoint, 2);
    // a runway too short (60 m)
    std::vector<Waypoint> shortRunway = {p[0], p[1], p[2], f.runway(kThresholdN + 60.0, WaypointType::RunwayLimit)};
    refused(shortRunway, Reason::InvalidWaypoint, 3);
    // a point in the air after its landing: a touch and go is not one
    refused({p[0], p[1], p[2], p[3], f.air(30000.0, 0.0, 500.0)}, Reason::InvalidWaypoint, 4);
    // a route that begins on a runway point begins on the ground (a takeoff's: 4.52) - not flown in the air
    refused({p[2], p[3]}, Reason::Airborne, 0);
    // its landing not the route's last points: given first, its points in the air started at and led to it
    {
        std::vector<Waypoint> linked = {p[2], p[3], p[0], p[1]};
        linked[3].next = 0, linked[1].next = -1; // (its limit the end)
        RouteCommand from;
        from.start = 2;
        const CommandResult r = w.submit(id, from, Span<const Waypoint>(linked.data(), linked.size()));
        CHECK(r.reason == Reason::InvalidWaypoint);
        CHECK(r.index == 0);
    }
    // a landing at its end is its NEW's: an UPDATE does not add one to a route begun without
    {
        const std::vector<Waypoint> plain = {p[0], p[1]};
        const CommandResult r = w.submit(id, RouteCommand{}, Span<const Waypoint>(plain.data(), plain.size()));
        REQUIRE(r.accepted());
        CHECK(w.update(r.activity, RouteCommand{}, Span<const Waypoint>(p.data(), p.size())).reason == Reason::NotUpdatable);
        CHECK(w.cancel(r.activity).status == CommandStatus::Canceled);
    }
    // a rotorcraft lands as its recovery does, on its spot (4.53), and does not taxi
    const auto heli = rotor(w, "uh1h");
    const Field h = fieldOf(w, heli);
    const std::vector<Waypoint> rotorRoute = {h.air(500.0, 0.0, 100.0), h.runway(1000.0, WaypointType::RunwayThreshold),
                                              h.runway(1000.0 + kLengthM, WaypointType::RunwayLimit), h.at(1000.0 + kLengthM, 100.0, WaypointType::Taxi)};
    const CommandResult r = w.submit(heli, RouteCommand{}, Span<const Waypoint>(rotorRoute.data(), rotorRoute.size()));
    CHECK(r.reason == Reason::NotImplemented);
    CHECK(r.index == 3);
    CHECK(w.support(heli, "fsim.guidance.route/waypoint_type/landing")->support == Support::Supported);
}

TEST_CASE("landing route: a rotorcraft flies its points and lands on its runway's spot, as its recovery does (4.59, 4.53)",
          "[modes][landing_route]") {
    session::World w(options("landing-route-rotor"));
    const auto id = rotor(w, "uh1h");
    const Field f = fieldOf(w, id);
    const std::vector<Waypoint> points = {f.air(800.0, 300.0, 120.0), f.runway(2000.0, WaypointType::RunwayThreshold),
                                          f.runway(2000.0 + kLengthM, WaypointType::RunwayLimit)};
    const CommandResult r = w.submit(id, RouteCommand{}, Span<const Waypoint>(points.data(), points.size()));
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    for (double t = 0.0; t < 900.0 && w.activity(r.activity)->live(); t += 0.5) w.step(stepsFor(w, 0.5));
    const sim::VehicleState& s = *w.vehicleState(id);
    double n, e;
    offset(s, f.lat0 + 2000.0 / kEarthM, f.lon0, n, e);
    INFO(n << " m along, " << e << " m across");
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    CHECK(s.onGround);
    CHECK(std::hypot(n - 60.0, e) < 3.0); // (its spot 60 m beyond the threshold: the recovery's)
}
