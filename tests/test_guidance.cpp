// Guidance flown (docs/vehicle-interface.md): the behaviours an autonomy
// commands a vehicle through, flown on a wing and on rotorcraft, in wind -
// the checks the Vehicle Interface's gap analysis found missing (step VI-1).
#include "control/Adapter.h"
#include "fsim/Control.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthM = 6371000.0;

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 11;
    return o;
}

/// A c172x flying east at 1500 m and 55 m/s.
std::uint32_t wing(session::World& w, const char* name, double northDeg = 0.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:c172x";
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = 1500.0;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = 55.0;
    return w.createVehicle(s);
}

/// A rotorcraft hovering at 100 m, spawned at its hover attitude and settled
/// for `settleS` holding still over the ground.
std::uint32_t rotor(session::World& w, const std::string& type, double settleS = 15.0, double northDeg = 0.0) {
    double pitchDeg = 0.0, rollDeg = 0.0;
    {
        session::VehicleSpec probe;
        probe.name = type + "-probe";
        probe.type = "jsbsim:" + type;
        probe.initial.latitudeDeg += 1.0;
        probe.initial.altitudeMslM = 100.0;
        const auto id = w.createVehicle(probe);
        REQUIRE(id != 0);
        const auto& hover = w.profile(id)->hover;
        if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
        if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
        REQUIRE(w.removeVehicle(id));
    }
    session::VehicleSpec s;
    s.name = type;
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = 100.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    s.initial.pitchDeg = pitchDeg;
    s.initial.rollDeg = rollDeg;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    VelocityCommand still;
    still.verticalSpeedMs = 0.0;
    still.northMs = still.eastMs = 0.0;
    REQUIRE(w.submit(id, still).accepted());
    w.step(static_cast<unsigned>(std::lround(settleS / (w.dt() * w.frameSkip()))));
    return id;
}

unsigned stepsFor(const session::World& w, double seconds) { return static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip()))); }

/// North and east of (lat0, lon0), m.
void offset(const sim::VehicleState& s, double lat0, double lon0, double& north, double& east) {
    north = (s.latitudeRad - lat0) * kEarthM;
    east = (s.longitudeRad - lon0) * kEarthM * std::cos(lat0);
}

PositionCommand pointAt(double lat0, double lon0, double north, double east, double altitudeM, double captureM) {
    PositionCommand p;
    p.latitudeRad = lat0 + north / kEarthM;
    p.longitudeRad = lon0 + east / (kEarthM * std::cos(lat0));
    p.altitudeMslM = altitudeM;
    p.captureRadiusM = captureM;
    return p;
}

void setWind(session::World& w, double fromDeg, double speedMs) {
    sim::EnvironmentState e = w.environment();
    e.windDirectionDeg = fromDeg;
    e.windSpeedMs = speedMs;
    w.setEnvironment(e);
}

/// Flies a route of legs (north, east) from where the vehicle is, until it
/// completes or `seconds` pass; the closest the vehicle came to each point.
std::vector<double> flyRoute(session::World& w, std::uint32_t v, const std::vector<std::pair<double, double>>& legs, double altitudeStepM,
                             double captureM, double seconds, bool& completed) {
    const sim::VehicleState s0 = *w.vehicleState(v);
    BehaviorCommand route;
    route.id = "waypoints";
    for (std::size_t i = 0; i < legs.size(); ++i)
        route.points.push_back(pointAt(s0.latitudeRad, s0.longitudeRad, legs[i].first, legs[i].second, s0.altitudeMslM + altitudeStepM * static_cast<double>(i + 1), captureM));
    const auto r = w.submit(v, route);
    REQUIRE(r.accepted());
    std::vector<double> closest(legs.size(), 1e30);
    completed = false;
    const unsigned n = stepsFor(w, seconds);
    for (unsigned k = 0; k < n && !completed; ++k) {
        w.step();
        double north, east;
        offset(*w.vehicleState(v), s0.latitudeRad, s0.longitudeRad, north, east);
        for (std::size_t i = 0; i < legs.size(); ++i) closest[i] = std::min(closest[i], std::hypot(north - legs[i].first, east - legs[i].second));
        completed = w.activity(r.activity)->state == ActivityState::Completed;
    }
    return closest;
}

} // namespace

TEST_CASE("guidance: a wing's route passes every point, climbing to each, and completes", "[guidance]") {
    session::World w(options("guidance-wing-route"));
    const auto v = wing(w, "c172x");
    bool completed = false;
    const auto closest = flyRoute(w, v, {{0.0, 4000.0}, {4000.0, 4000.0}, {4000.0, 0.0}}, 100.0, 300.0, 400.0, completed);
    CHECK(completed);
    for (const double d : closest) CHECK(d < 300.0);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - 1800.0) < 30.0);
}

TEST_CASE("guidance: rotorcraft fly a route given no airspeed at their position loops' own speed", "[guidance]") {
    struct Case {
        const char* type;
        double legM, captureM, seconds;
    };
    for (const Case c : {Case{"cf2", 3.0, 0.3, 60.0}, Case{"iris", 30.0, 2.0, 60.0}, Case{"uh1h", 400.0, 20.0, 150.0}}) {
        INFO(c.type);
        session::World w(options("guidance-rotor-route"));
        const auto v = rotor(w, c.type);
        bool completed = false;
        const auto closest = flyRoute(w, v, {{c.legM, 0.0}, {c.legM, c.legM}, {0.0, c.legM}}, 0.0, c.captureM, c.seconds, completed);
        CHECK(completed); // before VI-1 it never moved: its speed was the hover's airspeed, none
        for (const double d : closest) CHECK(d < c.captureM);
    }
}

TEST_CASE("guidance: hold turns a wing to its heading, altitude and airspeed, and keeps them in a crosswind", "[guidance]") {
    session::World w(options("guidance-wing-hold"));
    const auto v = wing(w, "c172x");
    setWind(w, 90.0, 12.0); // from the east: across the heading it is given
    BehaviorCommand hold;
    hold.id = "hold";
    hold.params = {{"heading_deg", 180.0}, {"altitude_m", 1700.0}, {"airspeed_ms", 50.0}};
    const auto r = w.submit(v, hold);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 150.0));
    const auto& s = *w.vehicleState(v);
    CHECK(std::abs(std::remainder(s.eulerRad[2] - kPi, 2.0 * kPi)) < 3.0 * kDeg);
    CHECK(std::abs(s.altitudeMslM - 1700.0) < 15.0);
    CHECK(std::abs(s.airspeedTrueMs - 50.0) < 1.5);
    CHECK(w.activity(r.activity)->state == ActivityState::Active); // a hold is persistent
}

TEST_CASE("guidance: a rotorcraft's hold keeps its hover in wind", "[guidance]") {
    struct Case {
        const char* type;
        double windMs, driftM;
    };
    // the hold keeps the velocity over the ground it had: none. Before VI-1 it
    // kept the airspeed it had, the wind's, along the nose, and flew off
    for (const Case c : {Case{"cf2", 1.0, 4.0}, Case{"iris", 5.0, 0.5}, Case{"uh1h", 5.0, 25.0}, Case{"uh60", 5.0, 5.0}}) {
        INFO(c.type);
        session::World w(options("guidance-rotor-hold"));
        setWind(w, 270.0, c.windMs);
        const auto v = rotor(w, c.type, 20.0);
        const sim::VehicleState s0 = *w.vehicleState(v);
        BehaviorCommand hold;
        hold.id = "hold";
        REQUIRE(w.submit(v, hold).accepted());
        double worst = 0.0;
        const unsigned n = stepsFor(w, 30.0);
        for (unsigned k = 0; k < n; ++k) {
            w.step();
            double north, east;
            offset(*w.vehicleState(v), s0.latitudeRad, s0.longitudeRad, north, east);
            worst = std::max(worst, std::hypot(north, east));
        }
        CHECK(worst < c.driftM);
        CHECK(std::abs(w.vehicleState(v)->altitudeMslM - s0.altitudeMslM) < 1.0);
    }
}

TEST_CASE("guidance: loiter circles at its radius, a rotorcraft's as small as it asks", "[guidance]") {
    struct Case {
        const char* type;
        double radiusM, settleS, measureS, tolerance;
    };
    for (const Case c : {Case{"c172x", 800.0, 120.0, 180.0, 0.10}, Case{"iris", 20.0, 30.0, 60.0, 0.10}, Case{"cf2", 2.0, 20.0, 40.0, 0.15}}) {
        INFO(c.type);
        session::World w(options("guidance-loiter"));
        const bool isWing = std::string(c.type) == "c172x";
        const auto v = isWing ? wing(w, "c172x") : rotor(w, c.type);
        const sim::VehicleState s0 = *w.vehicleState(v);
        BehaviorCommand loiter;
        loiter.id = "loiter";
        loiter.params = {{"radius_m", c.radiusM}};
        REQUIRE(w.submit(v, loiter).accepted());
        w.step(stepsFor(w, c.settleS));
        double sum = 0.0, lo = 1e30, hi = 0.0;
        const unsigned n = stepsFor(w, c.measureS);
        for (unsigned k = 0; k < n; ++k) {
            w.step();
            double north, east;
            offset(*w.vehicleState(v), s0.latitudeRad, s0.longitudeRad, north, east);
            const double r = std::hypot(north, east);
            sum += r, lo = std::min(lo, r), hi = std::max(hi, r);
        }
        const double mean = sum / n;
        CHECK(std::abs(mean - c.radiusM) < c.tolerance * c.radiusM);
        CHECK(hi - lo < 2.0 * c.tolerance * c.radiusM);
    }
    // a wing's catalog keeps the loiter no tighter than 100 m; a rotorcraft's lets it go to a metre
    session::World w(options("guidance-loiter-range"));
    auto radius = [&](std::uint32_t v) {
        for (const auto& d : w.capabilities(v))
            if (d.id == "fsim.guidance.loiter")
                for (const auto& p : d.parameters)
                    if (p.name == "radius_m") return p.min;
        return -1.0;
    };
    CHECK(radius(wing(w, "wing")) == 100.0);
    CHECK(radius(rotor(w, "iris", 0.0, 0.2)) == 1.0);
}

TEST_CASE("guidance: a rotorcraft flies an airspeed along its nose", "[guidance]") {
    session::World w(options("guidance-rotor-airspeed"));
    const auto v = rotor(w, "iris");
    VelocityCommand along;
    along.airspeedMs = 3.0;
    along.verticalSpeedMs = 0.0;
    along.headingRad = 0.5;
    REQUIRE(w.submit(v, along).accepted());
    w.step(stepsFor(w, 30.0));
    const auto& s = *w.vehicleState(v);
    CHECK(std::abs(s.airspeedTrueMs - 3.0) < 0.3);
    CHECK(std::abs(std::remainder(s.eulerRad[2] - 0.5, 2.0 * kPi)) < 2.0 * kDeg);
    CHECK(std::abs(std::remainder(std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) - 0.5, 2.0 * kPi)) < 3.0 * kDeg); // no wind: its track is its nose
    CHECK(std::abs(s.betaRad) < 0.05);
}

TEST_CASE("guidance: a route's points are checked before it is accepted", "[guidance]") {
    session::World w(options("guidance-route-check"));
    const auto v = wing(w, "c172x");
    const auto& s = *w.vehicleState(v);
    auto route = [&](auto&& spoil) {
        BehaviorCommand b;
        b.id = "waypoints";
        b.points = {pointAt(s.latitudeRad, s.longitudeRad, 3000.0, 0.0, 1500.0, 200.0), pointAt(s.latitudeRad, s.longitudeRad, 6000.0, 0.0, 1500.0, 200.0)};
        spoil(b.points[1]);
        return b;
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(w.submit(v, route([](PositionCommand&) {})).accepted());
    CHECK(w.submit(v, route([&](PositionCommand& p) { p.latitudeRad = nan; })).reason == Reason::InvalidParameter);
    CHECK(w.submit(v, route([](PositionCommand& p) { p.latitudeRad = 2.0; })).reason == Reason::InvalidParameter);
    CHECK(w.submit(v, route([](PositionCommand& p) { p.longitudeRad = std::numeric_limits<double>::infinity(); })).reason == Reason::InvalidParameter);
    CHECK(w.submit(v, route([](PositionCommand& p) { p.captureRadiusM = 0.0; })).reason == Reason::InvalidParameter);
    CHECK(w.submit(v, route([](PositionCommand& p) { p.airspeedMs = -5.0; })).reason == Reason::InvalidParameter);
    CHECK(w.submit(v, route([](PositionCommand& p) { p.airspeedMs = kHold; })).accepted()); // the airspeed may be left out
    // the existing entry points send what they always sent (no checks: ADR-26 10.7)
    CHECK(w.command(v, Command(route([](PositionCommand& p) { p.captureRadiusM = 0.0; }))));
}
