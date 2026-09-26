// Per-axis authority (docs/control-architecture.md, 9.1-9.6): commands that
// own the axes apart, merged by the runtime into one pass down the cascade;
// what a preempted activity keeps flying; the engines' throttles beside the
// cascade; the vehicle default; controllers that do not know about axes.
#include "control/Runtime.h"
#include "fsim/ControllerRegistry.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr AxisMask kPitch = axisBit(Axis::Pitch);
constexpr AxisMask kThrust = axisBit(Axis::Thrust);

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 3;
    return o;
}

session::VehicleSpec spec(const char* name, const char* type = "jsbsim:c172x", double altitudeM = 1500.0, double tasMs = 55.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    return s;
}

CommandOptions owning(AxisMask axes, Source source = Source::Policy) {
    CommandOptions o;
    o.axes = axes;
    o.source = source;
    return o;
}

std::size_t index(Axis a) { return static_cast<std::size_t>(a); }

/// How closely the stock c172x's attitude loop holds a bank: its roll channel
/// has no integrator worth the name, so about 0.04 rad short to the left.
/// That is the loop's own accuracy, whoever owns the other axes (the merge
/// itself is exact: see "merging" below).
constexpr double kBankTolerance = 0.06;

/// The heading change since the last call, unwrapped.
double turned(double& last, double now) {
    double d = now - last;
    while (d > kPi) d -= 2 * kPi;
    while (d < -kPi) d += 2 * kPi;
    last = now;
    return d;
}

/// A trainer's own attitude loop that knows nothing about axes.
class Unaware final : public Controller {
public:
    const char* id() const noexcept override { return "test_unaware_attitude"; }
    Level level() const noexcept override { return Level::Attitude; }
    Command update(const ControlContext&, const Command&) override { return ActuatorCommand{0.0, -0.05, 0.0, 0.6}; }
};

} // namespace

TEST_CASE("axes owned apart: a policy banks while an autopilot holds the height and the speed", "[axes]") {
    session::World w(options("axes-mixed"));
    const auto v = w.createVehicle(spec("a"));
    const CommandResult hold = w.submit(v, VelocityCommand{55.0, 0.0, kHold, kHold}, owning(kPitch | kThrust, Source::Autopilot));
    REQUIRE(hold.accepted());
    const CommandResult bank = w.submit(v, AttitudeCommand{0.35, kHold, kHold, 0.785, kHold, kHold}, owning(axisBit(Axis::Roll)));
    REQUIRE(bank.accepted());
    CHECK(w.activity(bank.activity)->axes == kLateral); // the loop that banks also coordinates
    CHECK(w.activity(hold.activity)->live());         // nothing in common: nobody was preempted
    CHECK(w.controls(v)->activeLevel() == Level::Velocity);
    // the autopilot's axes are its own
    const CommandResult refused = w.submit(v, AttitudeCommand{0.0, 0.1, kHold, 0.785, kHold, kHold}, owning(kPitch));
    CHECK(refused.reason == Reason::AuthorityHeld);
    CHECK(refused.other == hold.activity);

    double heading = w.vehicleState(v)->eulerRad[2], turn = 0.0;
    double lowest = 1e9, highest = -1e9, slowest = 1e9, fastest = -1e9;
    for (int k = 0; k < 1800; ++k) { // a minute
        w.step();
        const auto& s = *w.vehicleState(v);
        turn += turned(heading, s.eulerRad[2]);
        if (k < 600) continue; // settled into the turn
        lowest = std::min(lowest, s.altitudeMslM), highest = std::max(highest, s.altitudeMslM);
        slowest = std::min(slowest, s.airspeedTrueMs), fastest = std::max(fastest, s.airspeedTrueMs);
        CHECK(std::abs(s.eulerRad[0] - 0.35) < kBankTolerance);
    }
    CHECK(turn > kPi); // a 20 degree bank at 55 m/s turns about 3.7 degrees a second
    CHECK(lowest > 1480.0);
    CHECK(highest < 1520.0);
    CHECK(slowest > 51.0);
    CHECK(fastest < 59.0);
    CHECK(w.activity(hold.activity)->state == ActivityState::Active);
    CHECK(w.activity(bank.activity)->state == ActivityState::Active);
}

TEST_CASE("axes owned apart fly within the manoeuvre suite's tolerances for every adapter family", "[axes]") {
    // hangar's suite: a velocity hold keeps its height within 50 m over 40 s,
    // a bank step overshoots by at most 30 % (tools/hangar/hangar/autopilot.py)
    struct Aircraft {
        const char* type;
        double altitudeM, tasMs;
    };
    for (const Aircraft& a : {Aircraft{"jsbsim:c172x", 1500.0, 55.0}, Aircraft{"jsbsim:f16c", 3000.0, 160.0},
                              Aircraft{"jsbsim:b52h", 3000.0, 180.0}}) {
        INFO(a.type);
        session::World w(options("axes-families"));
        const auto v = w.createVehicle(spec("a", a.type, a.altitudeM, a.tasMs));
        REQUIRE(v != 0);
        REQUIRE(w.submit(v, VelocityCommand{a.tasMs, 0.0, kHold, kHold}, owning(kPitch | kThrust, Source::Autopilot)).accepted());
        REQUIRE(w.submit(v, AttitudeCommand{0.35, kHold, kHold, 0.785, kHold, kHold}, owning(kLateral)).accepted());
        w.step(600); // 20 s into the turn
        const double altitude = w.vehicleState(v)->altitudeMslM;
        double worstHeight = 0.0, worstBank = 0.0, worstSpeed = 0.0;
        for (int k = 0; k < 1200; ++k) { // 40 s
            w.step();
            const auto& s = *w.vehicleState(v);
            worstHeight = std::max(worstHeight, std::abs(s.altitudeMslM - altitude));
            worstBank = std::max(worstBank, std::abs(s.eulerRad[0] - 0.35));
            worstSpeed = std::max(worstSpeed, std::abs(s.airspeedTrueMs - a.tasMs));
        }
        CHECK(worstHeight < 50.0);
        CHECK(worstBank < 0.3 * 0.35);
        CHECK(worstSpeed < 0.05 * a.tasMs);
    }
}

TEST_CASE("merging: two commands owning the axes apart fly exactly as one owning them all", "[axes]") {
    auto fly = [](bool apart) {
        session::World w(options(apart ? "axes-apart" : "axes-whole"));
        const auto v = w.createVehicle(spec("a"));
        if (apart) {
            REQUIRE(w.submit(v, VelocityCommand{kHold, kHold, 1.8, kHold}, owning(kLateral)).accepted());
            REQUIRE(w.submit(v, VelocityCommand{58.0, 1.5, kHold, kHold}, owning(kPitch | kThrust)).accepted());
        } else {
            REQUIRE(w.submit(v, VelocityCommand{58.0, 1.5, 1.8, kHold}).accepted());
        }
        std::vector<double> trace;
        for (int k = 0; k < 600; ++k) {
            w.step();
            const auto& s = *w.vehicleState(v);
            const auto& in = *w.inputs(v);
            trace.insert(trace.end(), {in.aileron, in.elevator, in.rudder, in.throttle[0], s.altitudeMslM, s.eulerRad[2], s.airspeedTrueMs});
        }
        return trace;
    };
    const auto apart = fly(true), whole = fly(false);
    REQUIRE(apart.size() == whole.size());
    CHECK(apart == whole); // bit for bit
}

TEST_CASE("merging through the loop over pseudo-controls: the attitude level above the acceleration level it flies with", "[axes]") {
    // a hangar design flies its attitude over pseudo-controls, so a velocity
    // hold's demand passes the attitude level down to the acceleration level;
    // merged there, it flies exactly as one command owning every axis
    auto fly = [](bool apart) {
        session::World w(options(apart ? "pseudo-apart" : "pseudo-whole"));
        const auto v = w.createVehicle(spec("a", "jsbsim:f16c", 3000.0, 160.0));
        REQUIRE(std::string(w.controls(v)->controller(Level::Attitude)->id()) == "pseudo_attitude");
        if (apart) {
            REQUIRE(w.submit(v, VelocityCommand{kHold, kHold, 1.8, kHold}, owning(kLateral)).accepted());
            REQUIRE(w.submit(v, VelocityCommand{165.0, 3.0, kHold, kHold}, owning(kPitch | kThrust)).accepted());
        } else {
            REQUIRE(w.submit(v, VelocityCommand{165.0, 3.0, 1.8, kHold}).accepted());
        }
        std::vector<double> trace;
        for (int k = 0; k < 600; ++k) {
            w.step();
            const auto& s = *w.vehicleState(v);
            const auto& in = *w.inputs(v);
            trace.insert(trace.end(), {in.aileron, in.elevator, in.rudder, in.throttle[0], s.altitudeMslM, s.eulerRad[2], s.airspeedTrueMs});
        }
        return trace;
    };
    const auto apart = fly(true), whole = fly(false);
    REQUIRE(apart.size() == whole.size());
    CHECK(apart == whole); // bit for bit

    // a policy's roll rate, at the acceleration level, beside an autopilot's
    // height and speed flown through the attitude level above it
    session::World w(options("pseudo-mixed"));
    const auto v = w.createVehicle(spec("a", "jsbsim:f16c", 3000.0, 160.0));
    REQUIRE(w.submit(v, VelocityCommand{160.0, 0.0, kHold, kHold}, owning(kPitch | kThrust, Source::Autopilot)).accepted());
    REQUIRE(w.submit(v, AccelerationCommand{kHold, 0.2, kHold, kHold}, owning(kLateral)).accepted());
    w.step(30); // a second
    const auto* merged = std::get_if<AccelerationCommand>(w.controls(v)->derived(Level::Acceleration));
    REQUIRE(merged != nullptr);
    CHECK(merged->rollRateRadS == 0.2);       // the policy's
    CHECK_FALSE(isHold(merged->loadFactorG)); // the autopilot's, from the attitude level
    CHECK_FALSE(isHold(merged->longitudinalMs2));
    CHECK(w.vehicleState(v)->eulerRad[0] > 0.1); // rolling right
    CHECK(w.controls(v)->report().errors == 0);
}

TEST_CASE("preempted on some axes, an activity's others fly on as a residual hold; CANCEL gives axes back to the default", "[axes]") {
    session::World w(options("axes-residual"));
    const auto v = w.createVehicle(spec("a"));
    const auto level = w.submit(v, VelocityCommand{55.0, 0.0, 0.5 * kPi, kHold}).activity;
    REQUIRE(level != 0);
    w.step(30);
    const auto bank = w.submit(v, AttitudeCommand{0.3, kHold, kHold, 0.785, kHold, kHold}, owning(kLateral)).activity;
    REQUIRE(bank != 0);
    const ActivityRecord* old = w.activity(level);
    CHECK(old->state == ActivityState::Canceled);
    CHECK(old->reason == Reason::Preempted);
    CHECK(old->by == bank);
    // what the bank did not take still flies: the old command's height and speed
    const RuntimeConfig& c = w.controls(v)->config();
    const std::uint8_t residual = c.owner[index(Axis::Pitch)];
    REQUIRE(residual < kSlotCount);
    CHECK(c.owner[index(Axis::Thrust)] == residual);
    CHECK(c.slots[residual].axes == (kPitch | kThrust));
    CHECK(std::holds_alternative<VelocityCommand>(c.slots[residual].command));
    const double altitude = w.vehicleState(v)->altitudeMslM;
    w.step(600);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - altitude) < 20.0);
    CHECK(std::abs(w.vehicleState(v)->eulerRad[0] - 0.3) < kBankTolerance);

    // claiming a residual's axis preempts nobody; the residual keeps the rest
    const CommandResult throttle = w.submit(v, ActuatorCommand{kHold, kHold, kHold, 0.8}, owning(kThrust));
    REQUIRE(throttle.accepted());
    CHECK(w.activity(bank)->live());
    CHECK(c.slots[residual].axes == kPitch);
    w.step();
    CHECK(w.inputs(v)->throttle[0] == 0.8);

    // CANCEL: the bank's axes return to the default, the neutral actuator command
    REQUIRE(w.cancel(bank).status == CommandStatus::Canceled);
    CHECK(c.owner[index(Axis::Roll)] == RuntimeConfig::kNone);
    CHECK(c.owner[index(Axis::Yaw)] == RuntimeConfig::kNone);
    w.step();
    CHECK(w.inputs(v)->aileron == 0.0);
    CHECK(w.inputs(v)->rudder == 0.0);
    CHECK(w.inputs(v)->throttle[0] == 0.8);

    // a residual whose last axis is claimed is gone
    REQUIRE(w.submit(v, AttitudeCommand{kHold, 0.05, kHold, kHold, kHold, kHold}, owning(kPitch)).accepted());
    for (const auto& slot : c.slots) CHECK_FALSE((slot.axes && std::holds_alternative<VelocityCommand>(slot.command)));
}

TEST_CASE("a behaviour preempted on some axes flies on as a residual on the others", "[axes]") {
    session::World w(options("axes-behaviour-residual"));
    const auto v = w.createVehicle(spec("a"));
    BehaviorCommand hold;
    hold.id = "hold";
    const auto held = w.submit(v, hold).activity;
    REQUIRE(held != 0);
    w.step(30);
    const double altitude = w.vehicleState(v)->altitudeMslM;
    const auto bank = w.submit(v, AttitudeCommand{-0.3, kHold, kHold, 0.785, kHold, kHold}, owning(kLateral)).activity;
    REQUIRE(bank != 0);
    CHECK(w.activity(held)->reason == Reason::Preempted);
    CHECK(w.controls(v)->activeLevel() == Level::Behavior); // the hold's output still enters above the levels
    double lowest = 1e9, highest = -1e9;
    for (int k = 0; k < 900; ++k) {
        w.step();
        lowest = std::min(lowest, w.vehicleState(v)->altitudeMslM);
        highest = std::max(highest, w.vehicleState(v)->altitudeMslM);
    }
    CHECK(std::abs(w.vehicleState(v)->eulerRad[0] + 0.3) < kBankTolerance); // the policy's bank
    CHECK(lowest > altitude - 20.0);                                // the hold's height
    CHECK(highest < altitude + 20.0);
    // guidance owns every primary axis or none
    CHECK(w.submit(v, hold, owning(kPitch)).reason == Reason::InvalidAxes);
}

TEST_CASE("axis groups: roll and yaw go together above the actuators, each axis alone at them", "[axes]") {
    session::World w(options("axes-groups"));
    const auto v = w.createVehicle(spec("a"));
    CHECK(w.submit(v, AttitudeCommand{}, owning(axisBit(Axis::Yaw))).accepted());
    CHECK(w.activities(v).front().axes == kLateral);
    const CommandResult rudder = w.submit(v, ActuatorCommand{kHold, kHold, 0.2, kHold}, owning(axisBit(Axis::Yaw)));
    REQUIRE(rudder.accepted());
    CHECK(w.activity(rudder.activity)->axes == axisBit(Axis::Yaw));
    // the attitude command loses yaw: preempted, its roll a residual
    const RuntimeConfig& c = w.controls(v)->config();
    CHECK(c.owner[index(Axis::Roll)] < kSlotCount);
    CHECK(c.owner[index(Axis::Roll)] != c.owner[index(Axis::Yaw)]);
    w.step();
    CHECK(w.inputs(v)->rudder == 0.2);
    // no primary axis, or one that does not exist
    CHECK(w.submit(v, AttitudeCommand{}, owning(axisBit(Axis::Flaps))).reason == Reason::InvalidAxes);
    CHECK(w.submit(v, AttitudeCommand{}, owning(static_cast<AxisMask>(1u << 12))).reason == Reason::InvalidAxes);
}

TEST_CASE("an axis-unaware controller keeps flying whole-vehicle commands and refuses axes owned apart", "[axes]") {
    session::World w(options("axes-unaware"));
    const auto v = w.createVehicle(spec("a"));
    REQUIRE(w.controls(v)->use(Level::Attitude, std::make_unique<Unaware>()));
    const CommandResult whole = w.submit(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0});
    REQUIRE(whole.accepted());
    w.step();
    CHECK(w.inputs(v)->throttle[0] == 0.6); // its output, as always
    CHECK(w.submit(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}, owning(kLateral)).reason == Reason::ControllerNotAxisAware);
    // a velocity command's demand passes through the attitude loop too
    CHECK(w.submit(v, VelocityCommand{55.0, 0.0, kHold, kHold}, owning(kPitch | kThrust)).reason == Reason::ControllerNotAxisAware);
    // at the actuators no controller runs: the attitude command's residual would, apart
    CHECK(w.submit(v, ActuatorCommand{0.1}, owning(axisBit(Axis::Roll))).reason == Reason::ControllerNotAxisAware);
    CHECK(w.activity(whole.activity)->live());
    // with the built-in loop back, the same command is taken
    REQUIRE(w.controls(v)->use(Level::Attitude, "pid_attitude"));
    CHECK(w.submit(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}, owning(kLateral)).accepted());
    // the built-in loops say they are axis-aware
    for (const Level l : {Level::Attitude, Level::Acceleration, Level::Velocity, Level::Position}) {
        const auto c = ControllerRegistry::instance().create(ControllerRegistry::instance().defaultId(l));
        REQUIRE(c != nullptr);
        CHECK(c->axisAware());
    }
}

TEST_CASE("the engines' throttles: one per engine, beside a cascade that flies the rest", "[axes][engines]") {
    session::World w(options("axes-engines"));
    const auto twin = w.createVehicle(spec("twin", "jsbsim:a10c", 3000.0, 140.0));
    REQUIRE(twin != 0);
    const auto& all = w.capabilities(twin);
    const auto engines = std::find_if(all.begin(), all.end(), [](const CapabilityDescriptor& d) { return d.id == "fsim.flight.engines"; });
    REQUIRE(engines != all.end());
    CHECK(engines->kind == CapabilityKind::Flight);
    CHECK(engines->axes == kThrust);
    CHECK(engines->parameters.size() == 2);

    // the cascade keeps roll and pitch; the throttles are set per engine
    const auto attitude = w.submit(twin, AttitudeCommand{0.0, 0.03, kHold, 0.785, kHold, kHold}, owning(kLateral | kPitch)).activity;
    REQUIRE(attitude != 0);
    const CommandResult set = w.submit(twin, EnginesCommand{{0.9, 0.4, kHold, kHold}});
    REQUIRE(set.accepted());
    CHECK(w.activity(attitude)->live());
    w.step();
    CHECK(w.inputs(twin)->throttle[0] == 0.9);
    CHECK(w.inputs(twin)->throttle[1] == 0.4);
    REQUIRE(w.update(set.activity, EnginesCommand{{0.7, kHold, kHold, kHold}}).accepted());
    w.step();
    CHECK(w.inputs(twin)->throttle[0] == 0.7);
    CHECK(w.inputs(twin)->throttle[1] == 0.4); // held
    const CommandResult full = w.submit(twin, EnginesCommand{{1.5, 0.4, kHold, kHold}});
    REQUIRE(full.accepted());
    CHECK((full.flags & kClamped) != 0);
    CHECK(w.activity(set.activity)->by == full.activity); // the same capability again: the newer command wins
    w.step(30);
    CHECK(w.inputs(twin)->throttle[0] == 1.0);
    const auto& s = *w.vehicleState(twin);
    CHECK(std::abs(s.eulerRad[0]) < 0.05); // the attitude activity flew on

    // a command that owns thrust takes it back from the engines
    REQUIRE(w.submit(twin, AttitudeCommand{0.0, 0.03, kHold, 0.785, 0.5, kHold}, owning(kThrust)).accepted());
    CHECK(w.activity(full.activity)->reason == Reason::Preempted);
    CHECK(w.activity(attitude)->live());
    w.step();
    CHECK(w.inputs(twin)->throttle[0] == 0.5);
    CHECK(w.inputs(twin)->throttle[1] == 0.5);

    // a single engine has no such capability
    const auto single = w.createVehicle(spec("single"));
    CHECK(w.submit(single, EnginesCommand{{0.5, kHold, kHold, kHold}}).reason == Reason::UnknownCapability);
}

TEST_CASE("the vehicle default: neutral idles, a hold keeps the heading, speed and height the axes were let go at", "[axes][default]") {
    session::World w(options("axes-default"));
    const auto held = w.createVehicle(spec("held"));
    const auto idle = w.createVehicle(spec("idle", "jsbsim:c172x", 1500.0, 55.0));
    CHECK(w.vehicleDefault(held) == VehicleDefault::Neutral);
    REQUIRE(w.setVehicleDefault(held, VehicleDefault::Hold) == Reason::None);
    CHECK(w.vehicleDefault(held) == VehicleDefault::Hold);
    CHECK(w.setVehicleDefault(999, VehicleDefault::Hold) == Reason::UnknownVehicle);
    CHECK(w.controls(held)->activeLevel() == Level::Velocity);
    const double heading = w.vehicleState(held)->eulerRad[2];
    double lowest = 1e9, highest = -1e9;
    for (int k = 0; k < 1800; ++k) {
        w.step();
        if (k < 300) continue;
        lowest = std::min(lowest, w.vehicleState(held)->altitudeMslM);
        highest = std::max(highest, w.vehicleState(held)->altitudeMslM);
    }
    const auto& s = *w.vehicleState(held);
    double h = heading;
    CHECK(std::abs(turned(h, s.eulerRad[2])) < 0.05);
    CHECK(std::abs(s.airspeedTrueMs - 55.0) < 3.0);
    CHECK(lowest > 1485.0);
    CHECK(highest < 1515.0);
    CHECK(w.inputs(idle)->throttle[0] == 0.0); // the neutral default: idle, as every vehicle always has

    // a turn cancelled: the heading it had then is held
    const auto turn = w.submit(held, AttitudeCommand{0.35, 0.03, kHold, 0.785, kHold, 55.0}).activity;
    w.step(300);
    REQUIRE(w.cancel(turn).status == CommandStatus::Canceled);
    const double letGo = w.vehicleState(held)->eulerRad[2];
    w.step(900);
    double g = letGo;
    CHECK(std::abs(turned(g, w.vehicleState(held)->eulerRad[2])) < 0.1);

    // a policy on the lateral axes alone: the hold keeps the height and the speed
    const auto roll = w.submit(held, AttitudeCommand{-0.3, kHold, kHold, 0.785, kHold, kHold}, owning(kLateral)).activity;
    REQUIRE(roll != 0);
    const double altitude = w.vehicleState(held)->altitudeMslM;
    w.step(900);
    CHECK(std::abs(w.vehicleState(held)->altitudeMslM - altitude) < 25.0);
    CHECK(std::abs(w.vehicleState(held)->eulerRad[0] + 0.3) < kBankTolerance);

    // back to neutral: what nobody owns idles
    REQUIRE(w.setVehicleDefault(held, VehicleDefault::Neutral) == Reason::None);
    w.step();
    CHECK(w.inputs(held)->throttle[0] == 0.0);
    CHECK(w.inputs(held)->elevator == 0.0);
}

TEST_CASE("the vehicle default's hold is refused where an axis-unaware controller would fly it apart", "[axes][default]") {
    session::World w(options("axes-default-unaware"));
    const auto v = w.createVehicle(spec("a"));
    REQUIRE(w.controls(v)->use(Level::Attitude, std::make_unique<Unaware>()));
    // nobody owns anything: the hold would fly the whole vehicle
    REQUIRE(w.setVehicleDefault(v, VehicleDefault::Hold) == Reason::None);
    REQUIRE(w.setVehicleDefault(v, VehicleDefault::Neutral) == Reason::None);
    REQUIRE(w.submit(v, ActuatorCommand{0.0, 0.0, 0.0, 0.5}, owning(kThrust)).accepted());
    CHECK(w.setVehicleDefault(v, VehicleDefault::Hold) == Reason::ControllerNotAxisAware);
    CHECK(w.vehicleDefault(v) == VehicleDefault::Neutral);
}
