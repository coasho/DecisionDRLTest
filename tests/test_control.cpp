// Control stack: levels, cascade, registry, built-in loop signs and behaviours
// on synthetic states (no flight model needed).
#include "fsim/BuiltinControllers.h"
#include "fsim/ControlStack.h"
#include "fsim/ControllerRegistry.h"
#include "core/Geodesy.h"
#include "core/Units.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace fsim;
using namespace fsim::control;

namespace {

sim::VehicleState levelFlight(double tas = 60.0, double headingRad = 0.0) {
    sim::VehicleState s;
    s.latitudeRad = units::degreesToRadians(37.6);
    s.longitudeRad = units::degreesToRadians(-122.4);
    s.altitudeMslM = s.altitudeAglM = 1500.0;
    s.airspeedTrueMs = tas;
    s.velocityBodyMs[0] = tas;
    s.velocityNedMs[0] = tas * std::cos(headingRad);
    s.velocityNedMs[1] = tas * std::sin(headingRad);
    s.eulerRad[2] = headingRad;
    s.loadFactor = 1.0;
    return s;
}

struct Harness {
    sim::VehicleState state = levelFlight();
    Rng rng{1};
    ControlContext ctx{7, state, state, 1.0 / 120.0, nullptr, &rng};
    ControlStack stack;
    sim::ControlInputs out;
    void run(int steps = 1) {
        for (int i = 0; i < steps; ++i) stack.update(ctx, out);
    }
};

} // namespace

TEST_CASE("levels are strictly ordered and named", "[control]") {
    REQUIRE(Level::Actuator < Level::Attitude);
    REQUIRE(Level::Attitude < Level::Acceleration);
    REQUIRE(Level::Acceleration < Level::Velocity);
    REQUIRE(Level::Velocity < Level::Position);
    REQUIRE(Level::Position < Level::Behavior);
    REQUIRE(std::string(levelName(Level::Velocity)) == "velocity");
    REQUIRE(levelOf(Command(VelocityCommand{})) == Level::Velocity);
    REQUIRE(isHold(kHold));
    REQUIRE(orHold(kHold, 3.0) == 3.0);
    REQUIRE(orHold(2.0, 3.0) == 2.0);
}

TEST_CASE("registry has the built-ins and accepts user controllers", "[control]") {
    auto& r = ControllerRegistry::instance();
    for (const char* id : {"actuator", "pid_attitude", "pid_acceleration", "pid_velocity", "pid_position"}) REQUIRE(r.create(id) != nullptr);
    for (const char* id : {"hold", "waypoints", "loiter", "pursuit", "evade", "formation", "aerobatics"}) {
        auto b = r.create(id);
        REQUIRE(b != nullptr);
        REQUIRE(b->level() == Level::Behavior);
        REQUIRE(dynamic_cast<Behavior*>(b.get()) != nullptr);
    }
    REQUIRE(r.create("no_such_controller") == nullptr);

    struct FullDeflection final : Controller {
        const char* id() const noexcept override { return "test_full"; }
        Level level() const noexcept override { return Level::Attitude; }
        Command update(const ControlContext&, const Command&) override {
            ActuatorCommand a;
            a.aileron = 1.0;
            return a;
        }
    };
    r.add("test_full", Level::Attitude, [] { return std::make_unique<FullDeflection>(); });
    REQUIRE(r.levelOf("test_full") == Level::Attitude);

    Harness h;
    REQUIRE(h.stack.use(Level::Attitude, "test_full"));
    REQUIRE_FALSE(h.stack.use(Level::Velocity, "test_full")); // wrong level
    h.stack.command(AttitudeCommand{});
    h.run();
    REQUIRE(h.out.aileron == 1.0);
}

TEST_CASE("actuator commands pass through clamped and hold channels", "[control]") {
    Harness h;
    sim::ControlInputs initial;
    initial.setThrottleAll(0.4);
    initial.gearDown = 1.0;
    h.stack.setInitialInputs(initial);
    ActuatorCommand a;
    a.aileron = 2.0;
    a.elevator = -3.0;
    a.throttle = kHold;
    a.gearDown = kHold;
    h.stack.command(a);
    h.run();
    REQUIRE(h.out.aileron == 1.0);
    REQUIRE(h.out.elevator == -1.0);
    REQUIRE(h.out.throttle[0] == 0.4); // held
    REQUIRE(h.out.gearDown == 1.0);    // held
    REQUIRE(h.stack.activeLevel() == Level::Actuator);
    REQUIRE(h.stack.derived(Level::Actuator) != nullptr);
    REQUIRE(h.stack.derived(Level::Attitude) == nullptr);
}

TEST_CASE("attitude loop steers towards the commanded roll, pitch, heading and airspeed", "[control]") {
    Harness h;
    h.stack.command(AttitudeCommand{units::degreesToRadians(30.0), units::degreesToRadians(5.0), kHold, 0.785, kHold, 80.0});
    h.run(60); // let the roll reference slew
    REQUIRE(h.out.aileron > 0.2);   // right roll wanted -> right aileron
    REQUIRE(h.out.elevator < -0.05); // nose up wanted -> negative (nose-up) elevator
    REQUIRE(h.out.throttle[0] > 0.9); // 80 m/s wanted at 60 m/s -> more power
    REQUIRE(h.stack.derived(Level::Attitude) != nullptr);
    REQUIRE(h.stack.derived(Level::Actuator) != nullptr);

    // Heading mode: a target 90 degrees to the right rolls right, bounded by the bank limit.
    AttitudeCommand hdg;
    hdg.headingRad = units::degreesToRadians(90.0);
    hdg.maxBankRad = units::degreesToRadians(20.0);
    h.stack.command(hdg);
    h.run(120);
    REQUIRE(h.out.aileron > 0.1);
    const auto* derived = std::get_if<ActuatorCommand>(h.stack.derived(Level::Actuator));
    REQUIRE(derived != nullptr);
}

TEST_CASE("velocity and position loops cascade down to actuators", "[control]") {
    Harness h;
    VelocityCommand v;
    v.airspeedMs = 60.0;
    v.verticalSpeedMs = 5.0;   // climb wanted -> nose up
    v.headingRad = units::degreesToRadians(0.0);
    h.stack.command(v);
    h.run(10);
    REQUIRE(h.stack.activeLevel() == Level::Velocity);
    REQUIRE(h.stack.derived(Level::Attitude) != nullptr);
    const auto* att = std::get_if<AttitudeCommand>(h.stack.derived(Level::Attitude));
    REQUIRE(att != nullptr);
    REQUIRE(att->pitchRad > 0.0);
    REQUIRE(h.out.elevator < 0.0);

    PositionCommand p;
    geo::offsetLatLon(h.state.latitudeRad, h.state.longitudeRad, 0.0, 5000.0, p.latitudeRad, p.longitudeRad); // 5 km east
    p.altitudeMslM = 1500.0;
    p.airspeedMs = 60.0;
    h.stack.command(p);
    h.run(60);
    const auto* vel = std::get_if<VelocityCommand>(h.stack.derived(Level::Velocity));
    REQUIRE(vel != nullptr);
    REQUIRE_THAT(vel->headingRad, Catch::Matchers::WithinAbs(units::degreesToRadians(90.0), 0.02));
    REQUIRE(h.out.aileron > 0.0); // turn right towards east
}

TEST_CASE("behaviours start, cascade and report completion", "[control]") {
    Harness h;
    BehaviorCommand wp;
    wp.id = "waypoints";
    PositionCommand p;
    p.latitudeRad = h.state.latitudeRad;
    p.longitudeRad = h.state.longitudeRad;
    p.altitudeMslM = 1500.0;
    p.captureRadiusM = 200.0; // already inside: captured on the first update
    wp.points.push_back(p);
    h.stack.command(wp);
    REQUIRE(h.stack.activeLevel() == Level::Behavior);
    REQUIRE(h.stack.behavior() != nullptr);
    h.run(2);
    REQUIRE(h.stack.behaviorFinished());
    REQUIRE(h.stack.derived(Level::Position) != nullptr); // the behaviour emitted a position command
    REQUIRE(h.stack.derived(Level::Actuator) != nullptr);

    BehaviorCommand unknown;
    unknown.id = "no_such_behaviour";
    h.stack.command(unknown);
    REQUIRE(h.stack.activeLevel() == Level::Behavior); // previous command kept
    REQUIRE(h.stack.behaviorFinished());
}

TEST_CASE("controller parameters are tunable by name", "[control]") {
    Harness h;
    auto* att = h.stack.controller(Level::Attitude);
    REQUIRE(att != nullptr);
    REQUIRE(att->parameter("roll.kp").has_value());
    REQUIRE(att->setParameter("roll.kp", 4.0));
    REQUIRE(*att->parameter("roll.kp") == 4.0);
    REQUIRE_FALSE(att->setParameter("nonsense", 1.0));
}
