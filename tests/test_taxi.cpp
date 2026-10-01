// A-GRA's taxi route (docs/flight-autonomy.md, 4.51; ADR-29 FA-9c: WPT-26): a wing taxies a route on the ground within 2 m of
// its path - its legs, and each corner drawn as an arc wider than its tightest turn on its wheels - and stops at its end; the
// route is checked at the NEW against that turn and its taxi speed; it stops short of another vehicle in its way.
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

/// An aircraft parked on the ground at the default place (`northM` north of it), facing north, held on its brakes if `braked`.
std::uint32_t parked(session::World& w, const std::string& type, double northM = 0.0, bool braked = false) {
    session::VehicleSpec s;
    s.name = type + std::to_string(static_cast<int>(northM));
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += northM / kEarthM / kDeg;
    s.initial.onGround = true;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    if (braked) {
        ActuatorCommand hold;
        hold.gearDown = 1.0, hold.brakeLeft = hold.brakeRight = 1.0;
        REQUIRE(w.submit(id, hold).accepted());
    }
    return id;
}

/// A taxi through points (north, east metres from where `id` stands).
BehaviorCommand taxi(session::World& w, std::uint32_t id, const std::vector<std::pair<double, double>>& points, double speedMs = 8.0) {
    const sim::VehicleState& s = *w.vehicleState(id);
    BehaviorCommand b;
    b.id = "taxi";
    b.params["speed_ms"] = speedMs;
    for (const auto& [n, e] : points) {
        PositionCommand p;
        p.latitudeRad = s.latitudeRad + n / kEarthM, p.longitudeRad = s.longitudeRad + e / (kEarthM * std::cos(s.latitudeRad));
        b.points.push_back(p);
    }
    return b;
}

/// The radius the host draws its corners with: a quarter wider than its tightest turn on its wheels, 8 m at least.
double cornerRadius(session::World& w, std::uint32_t id) {
    const double tightest = w.profile(id)->envelope.groundTurnRadiusM;
    return std::max(1.25 * (std::isfinite(tightest) ? tightest : 15.0), 8.0);
}

} // namespace

TEST_CASE("taxi: a wing taxies its route within 2 m of its path and stops at its end, calm and in a 10 m/s crosswind (WPT-26)",
          "[modes][taxi]") {
    // north 150 m, east 200, north 200, west 150: three corners each way; a light single, a fighter, the heavies whose nose
    // tyres turn them slowly, a bicycle gear, the B-52H's 52 m arcs
    const std::vector<std::pair<double, double>> route = {{150.0, 0.0}, {150.0, 200.0}, {350.0, 200.0}, {350.0, 50.0}};
    const std::vector<std::pair<double, double>> path = {{0.0, 0.0}, {150.0, 0.0}, {150.0, 200.0}, {350.0, 200.0}, {350.0, 50.0}}; // (from where it starts)
    for (const char* type : {"c172", "f16c", "kc135r", "c130j", "b52h", "u2s"}) {
        for (double wind : {0.0, 10.0}) {
            if (wind > 0.0 && std::string(type) == "u2s") continue; // (2.55 m across the wind on its wingtip skid: 4.51)
            INFO(type << " in a " << wind << " m/s crosswind");
            session::World w(options(("taxi-" + std::string(type)).c_str()));
            if (wind > 0.0) setWind(w, 270.0, wind);
            const auto id = parked(w, type);
            w.step(stepsFor(w, 1.0));
            const sim::VehicleState s0 = *w.vehicleState(id);
            const CommandResult r = w.submit(id, taxi(w, id, route));
            REQUIRE(r.accepted());
            const double radius = cornerRadius(w, id);
            double worst = 0.0;
            for (double t = 0.0; t < 400.0 && w.activity(r.activity)->live(); t += 0.1) {
                w.step(stepsFor(w, 0.1));
                double north, east;
                offset(*w.vehicleState(id), s0.latitudeRad, s0.longitudeRad, north, east);
                if (north > 75.0 || east > 1.0) worst = std::max(worst, taxiPathDistance(path, radius, north, east)); // (from its line)
                CHECK(w.vehicleState(id)->onGround);
            }
            CHECK(w.activity(r.activity)->state == ActivityState::Completed);
            CHECK(worst < 2.0); // (the worst: the U-2S, 1.64 m calm; the rest within 1.3 m)
            double north, east;
            offset(*w.vehicleState(id), s0.latitudeRad, s0.longitudeRad, north, east);
            CHECK(std::hypot(north - 350.0, east - 50.0) < 1.0);
            CHECK(groundSpeed(*w.vehicleState(id)) < 0.5);
        }
    }
}

TEST_CASE("taxi: it stops short of another vehicle in its way and goes on when it has gone", "[modes][taxi]") {
    session::World w(options("taxi-obstructed"));
    const auto id = parked(w, "f16c");
    const auto other = parked(w, "c172", 200.0, true); // (on its brakes, 200 m up its path)
    w.step(stepsFor(w, 1.0));
    const sim::VehicleState s0 = *w.vehicleState(id);
    const CommandResult r = w.submit(id, taxi(w, id, {{300.0, 0.0}}));
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 60.0));
    double north, east;
    offset(*w.vehicleState(id), s0.latitudeRad, s0.longitudeRad, north, east);
    CHECK(w.activity(r.activity)->state == ActivityState::Active);
    CHECK(groundSpeed(*w.vehicleState(id)) < 0.1);
    CHECK((north > 120.0 && north < 175.0)); // stopped well short of it: 25 m its clearance, 30 m more (146 m along)
    CHECK((w.activity(r.activity)->constraints & kActivityClamped) != 0);
    REQUIRE(w.removeVehicle(other));
    for (double t = 0.0; t < 120.0 && w.activity(r.activity)->live(); t += 1.0) w.step(stepsFor(w, 1.0));
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    offset(*w.vehicleState(id), s0.latitudeRad, s0.longitudeRad, north, east);
    CHECK(std::abs(north - 300.0) < 1.0);
}

TEST_CASE("taxi: its NEW checks the route against the aircraft's turn on its wheels and its taxi speed; only on the ground",
          "[modes][taxi]") {
    session::World w(options("taxi-refusals"));
    const auto id = parked(w, "b52h"); // (its corners 52 m arcs)
    w.step(stepsFor(w, 1.0));
    const auto refused = [&](const BehaviorCommand& b, Reason why, std::int16_t index, CommandOptions options = {}) {
        const CommandResult r = w.submit(id, b, options);
        CHECK(!r.accepted());
        CHECK(r.reason == why);
        CHECK(r.index == index);
        return r;
    };
    refused(taxi(w, id, {}), Reason::InvalidWaypoint, 0);                                     // nowhere to taxi
    const CommandResult back = refused(taxi(w, id, {{200.0, 0.0}, {100.0, 0.0}}), Reason::InvalidWaypoint, 0); // back on itself at point 0
    CHECK(back.constraint == Constraint::MaxTurnRate);
    // a 90 deg corner's 52 m arc on a 40 m leg: the leg into point 1 cannot hold it
    const CommandResult tight = refused(taxi(w, id, {{200.0, 0.0}, {200.0, 40.0}, {300.0, 40.0}}), Reason::InvalidWaypoint, 1);
    CHECK(tight.constraint == Constraint::MaxTurnRate);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    refused(taxi(w, id, {{200.0, 0.0}}, 20.0), Reason::OutOfRange, 0, reject);                // faster than 15 m/s
    CHECK(w.submit(id, taxi(w, id, {{200.0, 0.0}, {200.0, 200.0}})).accepted());               // the same corner, room for it

    // its lifecycle: NEW on the primary axes, the gear and the brakes; active after a step; canceled - nothing handed on
    {
        const CommandResult r = w.submit(id, taxi(w, id, {{300.0, 0.0}}));
        REQUIRE(r.accepted());
        CHECK((w.activity(r.activity)->axes & (kPrimaryAxes | axisBit(Axis::Gear) | axisBit(Axis::Brakes))) ==
              (kPrimaryAxes | axisBit(Axis::Gear) | axisBit(Axis::Brakes)));
        w.step();
        CHECK(w.activity(r.activity)->state == ActivityState::Active);
        const CommandResult c = w.cancel(r.activity);
        CHECK(c.status == CommandStatus::Canceled);
        CHECK(c.other == 0);
    }
    // in the air: unavailable
    const auto flying = wing(w, "c172", 1000.0, 50.0, 1);
    CHECK(w.submit(flying, taxi(w, flying, {{500.0, 0.0}})).reason == Reason::Airborne);
    // a rotorcraft on wheels: not built; on skids or none, not supported (R2: wheels)
    session::World r(options("taxi-rotor"));
    CHECK(r.support(parked(r, "uh60"), "fsim.guidance.taxi")->support == Support::NotImplemented);
    CHECK(r.support(parked(r, "uh1h", 100.0), "fsim.guidance.taxi")->support == Support::NotSupported);
    CHECK(w.support(id, "fsim.guidance.taxi")->support == Support::Supported);
}
