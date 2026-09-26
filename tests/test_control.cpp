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

TEST_CASE("a scaled PID scales its gains, not what its integrator holds", "[control]") {
    Pid a{2.0, 0.5, 0.3, 1.0, -10.0, 10.0}, b = a;
    // scale 1 is the unscaled update
    REQUIRE_THAT(a.update(0.2, 0.1, 0.01), Catch::Matchers::WithinAbs(b.update(0.2, 0.1, 0.01, 1.0), 1e-12));
    const double held = b.integral;
    const double out = b.update(0.0, 0.0, 0.01, 3.0);
    REQUIRE(b.integral == held);   // no error: the integrator keeps what it held
    REQUIRE_THAT(out, Catch::Matchers::WithinAbs(held, 1e-12));
    Pid c{2.0, 0.0, 0.5, 1.0, -10.0, 10.0};
    REQUIRE_THAT(c.update(0.2, 0.1, 0.01, 2.0), Catch::Matchers::WithinAbs(2.0 * (2.0 * 0.2 - 0.5 * 0.1), 1e-12));
}

TEST_CASE("the airspeed schedule follows the aircraft's response, and its trim law the lift", "[control]") {
    AirspeedSchedule off;
    sim::VehicleState s = levelFlight(200.0);
    s.airspeedCalibratedMs = 160.0;
    REQUIRE(off.factor(s, 2.0, -1.0) == 1.0);
    REQUIRE(off.speedRatio(s) == 1.0);
    REQUIRE(off.trimElevator(s, 1.0, 0.0, 0.0) == 0.0);

    AirspeedSchedule on{100.0, 80.0};
    // a surface's load factor grows as eas^2: twice the eas, a quarter of the gain
    REQUIRE_THAT(on.factor(s, 2.0, 0.0), Catch::Matchers::WithinAbs(0.25, 1e-12));
    // a law's pitch rate per g falls as 1 / tas: twice the tas, twice the gain
    REQUIRE_THAT(on.factor(s, 0.0, -1.0), Catch::Matchers::WithinAbs(2.0, 1e-12));
    REQUIRE(on.factor(s, 8.0, 0.0) == 0.2);   // held within 0.2 .. 5
    REQUIRE_THAT(on.speedRatio(s), Catch::Matchers::WithinAbs(2.0, 1e-12));
    // the trim law: the trim at the reference eas, the lift's part growing as n (eas0 / eas)^2
    s.airspeedCalibratedMs = 80.0;
    REQUIRE_THAT(on.trimElevator(s, 1.0, 0.3, 0.2), Catch::Matchers::WithinAbs(0.3, 1e-12));
    REQUIRE_THAT(on.trimElevator(s, 2.0, 0.3, 0.2), Catch::Matchers::WithinAbs(0.5, 1e-12));
    s.airspeedCalibratedMs = 80.0 / std::sqrt(2.0);
    REQUIRE_THAT(on.trimElevator(s, 1.0, 0.3, 0.2), Catch::Matchers::WithinAbs(0.5, 1e-12));
}

TEST_CASE("an aircraft's controller settings reach the loops the stack creates by id", "[control]") {
    Harness h;
    const auto unused = h.stack.setControllerSettings({{"pid_attitude", "pitch.kp", 1.25},
                                                       {"pid_velocity", "schedule.tas_ms", 150.0},
                                                       {"pid_attitude", "no.such", 1.0},
                                                       {"someone_elses", "gain", 2.0}});
    REQUIRE(unused.size() == 2);
    REQUIRE(*h.stack.controller(Level::Attitude)->parameter("pitch.kp") == 1.25);
    REQUIRE(*h.stack.controller(Level::Velocity)->parameter("schedule.tas_ms") == 150.0);
    // a new one by id gets them again; an instance handed over is left as it is
    REQUIRE(h.stack.use(Level::Attitude, "pid_attitude"));
    REQUIRE(*h.stack.controller(Level::Attitude)->parameter("pitch.kp") == 1.25);
    REQUIRE(h.stack.use(Level::Attitude, std::make_unique<AttitudeLoop>()));
    REQUIRE(*h.stack.controller(Level::Attitude)->parameter("pitch.kp") == 2.5);
    REQUIRE(h.stack.controllerSettings().size() == 4);
}

TEST_CASE("the velocity loop's feedforwards lag the command and hold the 1 g angle of attack", "[control]") {
    VelocityLoop plain, fed;
    fed.flightPathFeedforward = 1.0;
    fed.commandLagS = 2.0;
    fed.alphaZeroLift = -0.05;
    sim::VehicleState s = levelFlight(100.0);
    s.alphaRad = 0.05;
    Rng rng{1};
    ControlContext ctx{1, s, s, 0.1, nullptr, &rng};
    VelocityCommand c;
    c.verticalSpeedMs = 10.0;
    // the lagged command starts from the vertical speed flown: no jump in pitch
    const auto a = std::get<AttitudeCommand>(fed.update(ctx, c));
    REQUIRE_THAT(a.pitchRad, Catch::Matchers::WithinAbs(0.05, 1e-3));   // just the 1 g alpha
    // at 2 g the wing's alpha doubles about its zero-lift angle: the 1 g one is fed forward
    s.alphaRad = -0.05 + 2.0 * 0.1;
    s.loadFactor = 2.0;
    VelocityCommand level;
    level.verticalSpeedMs = 0.0;
    const auto b = std::get<AttitudeCommand>(fed.update(ctx, level));
    REQUIRE_THAT(b.pitchRad, Catch::Matchers::WithinAbs(0.05, 1e-3));
    // without them the loop is what it was
    const auto p = std::get<AttitudeCommand>(plain.update(ctx, c));
    REQUIRE(p.pitchRad > 0.0);
}
