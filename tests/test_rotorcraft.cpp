// The rotorcraft (docs/rotorcraft.md): the four aircraft through their
// families' adapters - what they declare, what they refuse - and a manoeuvre
// suite flown by the platform's own rotorcraft loops, designed from each
// aircraft's hover section: hover, climb, heading and velocity steps, body
// rates, a point to stop at, the hover behaviour, axes owned apart, the
// vehicle default's hold and envelope protection. Judged as the fixed-wing
// suite is (test_manoeuvres.cpp): a step fails if control is lost, it never
// reaches 90 % of its target, or it overshoots by more than half.
#include "control/Adapter.h"
#include "fsim/Control.h"
#include "fsim/GuidanceModes.h"
#include "session/World.h"
#include "sim/FlightModel.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEarthM = 6371000.0;

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 7;
    return o;
}

session::VehicleSpec spec(const std::string& name, const std::string& type, double northDeg) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = 100.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    return s;
}

/// Each aircraft's steps, sized for it.
struct Aircraft {
    const char* type;
    ControlFamily family;
    double speedMs;     ///< the velocity step
    double climbMs;     ///< the climb step
    double rateRadS;    ///< the roll-rate step
    double distanceM;   ///< the point to fly to (north and east of the start)
    double driftM;      ///< what a hover may drift
    double arriveS;     ///< how long it has to get there
    double rateS;       ///< how long the roll rate is held
};
const Aircraft kAircraft[] = {
    {"cf2", ControlFamily::Multirotor, 1.0, 0.5, 0.5, 5.0, 0.3, 40.0, 1.0},
    {"iris", ControlFamily::Multirotor, 5.0, 1.0, 0.5, 30.0, 0.5, 40.0, 1.0},
    {"uh1h", ControlFamily::Helicopter, 10.0, 2.0, 0.1, 200.0, 15.0, 90.0, 2.0},
    {"uh60", ControlFamily::Helicopter, 10.0, 2.0, 0.1, 200.0, 10.0, 90.0, 2.0},
};

struct Step {
    bool lost = false;
    double riseS = kInf;    ///< to 90 % of the target
    double overshoot = 0.0; ///< beyond it, a fraction of the step
    bool passes() const noexcept { return !lost && std::isfinite(riseS) && overshoot <= 0.5; }
};

Step stepOf(const std::vector<double>& y, double target, double dt, bool lost) {
    Step s;
    s.lost = lost || y.size() < 10;
    if (s.lost) return s;
    double most = -kInf;
    for (std::size_t i = 0; i < y.size(); ++i) {
        const double progress = y[i] / target;
        if (!std::isfinite(s.riseS) && progress >= 0.9) s.riseS = static_cast<double>(i + 1) * dt;
        most = std::max(most, progress);
    }
    s.overshoot = std::max(0.0, most - 1.0);
    return s;
}

/// North and east of a point, m.
void offset(const sim::VehicleState& s, double lat0, double lon0, double& north, double& east) {
    north = (s.latitudeRad - lat0) * kEarthM;
    east = (s.longitudeRad - lon0) * kEarthM * std::cos(lat0);
}

const ParameterInfo* parameterOf(const std::vector<CapabilityDescriptor>& caps, const char* capability, const char* name) {
    for (const auto& d : caps)
        if (d.id == capability)
            for (const auto& p : d.parameters)
                if (p.name == name) return &p;
    return nullptr;
}

const CapabilityDescriptor* capabilityOf(const std::vector<CapabilityDescriptor>& caps, const char* id) {
    for (const auto& d : caps)
        if (d.id == id) return &d;
    return nullptr;
}

VelocityCommand hoverHere() {
    VelocityCommand v;
    v.airspeedMs = kHold;
    v.verticalSpeedMs = 0.0;
    v.northMs = v.eastMs = 0.0;
    return v;
}

} // namespace

TEST_CASE("rotorcraft: each family's adapter names its controls, owns its axes in its own groups and refuses what it has nothing for", "[rotorcraft]") {
    session::World w(options("rotorcraft-contract"));
    for (const auto& a : kAircraft) {
        INFO(a.type);
        const auto id = w.createVehicle(spec(a.type, a.type, 0.0));
        REQUIRE(id != 0);
        const VehicleProfile& p = *w.profile(id);
        CHECK(p.identity.family == a.family);
        CHECK(p.hover.header.present());
        CHECK(p.control.header.provenance == Provenance::Derived); // the rotorcraft loops, designed from the hover
        const bool heli = a.family == ControlFamily::Helicopter;
        CHECK(p.effectors.thrust == (heli ? ThrustControl::Collective : ThrustControl::RotorThrust));
        CHECK(p.effectors.pitch == (heli ? PitchControl::Cyclic : PitchControl::Mixer));
        CHECK(p.propulsion.type == (heli ? EngineType::Turboshaft : EngineType::Electric));
        CHECK(std::string(adapterFor(p.identity.family).family()) == (heli ? "jsbsim.helicopter" : "jsbsim.multirotor"));
        const auto& caps = w.capabilities(id);
        // its own controls' names, and nothing it has nothing for
        CHECK(parameterOf(caps, "fsim.flight.actuator", heli ? "collective" : "thrust") != nullptr);
        CHECK(parameterOf(caps, "fsim.flight.actuator", heli ? "lateral_cyclic" : "roll") != nullptr);
        CHECK(parameterOf(caps, "fsim.flight.actuator", "throttle") == nullptr);
        CHECK_FALSE(parameterOf(caps, "fsim.flight.actuator", "flaps")->supported);
        CHECK_FALSE(parameterOf(caps, "fsim.flight.acceleration", "longitudinal_ms2")->supported);
        CHECK_FALSE(parameterOf(caps, "fsim.flight.attitude", "airspeed_ms")->supported);
        CHECK(parameterOf(caps, "fsim.flight.acceleration", "pitch_rate_rad_s")->supported);
        CHECK(parameterOf(caps, "fsim.flight.velocity", "north_ms")->supported);
        CHECK(parameterOf(caps, "fsim.flight.position", "heading_rad")->supported);
        // the cyclic, the pedals, the collective
        CHECK(capabilityOf(caps, "fsim.flight.velocity")->axisGroups == (kGroupCyclic | kGroupYaw | kGroupThrust));
        CHECK(capabilityOf(caps, "fsim.flight.actuator")->axisGroups == (kGroupEachAxis | kGroupCyclic | kGroupYaw | kGroupThrust));
        // the hover is offered; the wing's aerobatics are not
        CHECK(capabilityOf(caps, "fsim.guidance.hover") != nullptr);
        CHECK(capabilityOf(caps, "fsim.guidance.aerobatics") == nullptr);
        // a multirotor's rotors fly every axis; a helicopter's engines are governed
        const auto* engines = capabilityOf(caps, "fsim.flight.engines");
        if (heli) {
            CHECK(engines == nullptr);
        } else {
            REQUIRE(engines != nullptr);
            CHECK(engines->axes == kPrimaryAxes);
            CHECK(engines->parameters.size() == 4);
            CHECK(engines->parameters[0].name == "rotor_1");
        }
        // refused: what it has nothing for; accepted: what it flies with
        const auto& s = *w.vehicleState(id);
        AccelerationCommand along{1.0, 0.0, 0.5, kHold, kHold, kHold};
        const auto refused = w.submit(id, along);
        CHECK(refused.reason == Reason::NotSupported); // with the field it has nothing for (docs/flight-autonomy.md, 4.3)
        CHECK(refused.index == 2);
        AccelerationCommand rates{1.0, 0.1, kHold, kHold, 0.05, -0.05};
        CHECK(w.submit(id, rates).accepted());
        // a cyclic command owns roll and pitch together
        const auto bank = w.submit(id, AttitudeCommand{0.05, kHold, kHold, 0.785, kHold, kHold}, CommandOptions{Source::Policy, axisBit(Axis::Roll)});
        REQUIRE(bank.accepted());
        CHECK(w.activity(bank.activity)->axes == kCyclic);
        (void)s;
    }
    // and a wing keeps its own: rotorcraft fields refused, groups as they were, no hover
    const auto wing = w.createVehicle(spec("c172x", "c172x", 0.2));
    REQUIRE(wing != 0);
    const auto& caps = w.capabilities(wing);
    CHECK_FALSE(parameterOf(caps, "fsim.flight.acceleration", "pitch_rate_rad_s")->supported);
    CHECK_FALSE(parameterOf(caps, "fsim.flight.velocity", "east_ms")->supported);
    CHECK(capabilityOf(caps, "fsim.flight.velocity")->axisGroups == (kGroupLateral | kGroupPitch | kGroupThrust));
    CHECK(capabilityOf(caps, "fsim.guidance.hover") == nullptr);
    CHECK(capabilityOf(caps, "fsim.guidance.aerobatics") != nullptr);
    AccelerationCommand pitchRate{1.0, 0.0, kHold, kHold, 0.1, kHold};
    CHECK(w.submit(wing, pitchRate).reason == Reason::NotSupported);
    VelocityCommand overGround{kHold, 0.0, kHold, kHold, 5.0, 0.0};
    CHECK(w.submit(wing, overGround).reason == Reason::NotSupported);
}

TEST_CASE("rotorcraft: the manoeuvre suite, flown by the rotorcraft loops designed from each aircraft's hover", "[conformance]") {
    for (const auto& a : kAircraft) {
        INFO(a.type);
        session::World w(options("rotorcraft-suite"));
        const double dt = w.dt() * w.frameSkip();
        enum { Hover, Climb, Heading, Velocity, Rate, Point, HoverBehaviour, Apart, Default, Protected, kCount };
        std::uint32_t v[kCount];
        ActivityId act[kCount];
        double lat0[kCount], lon0[kCount], h0[kCount];
        // each enters its hover from the hover's attitude (its profile's): spawned level, a helicopter
        // that hovers nose up lurches forward until its loops have tilted it back - the spawn, not the
        // loops, which is not what the suite measures
        double pitchDeg = 0.0, rollDeg = 0.0;
        {
            const std::uint32_t probe = w.createVehicle(spec(std::string(a.type) + "-probe", a.type, 0.5));
            REQUIRE(probe != 0);
            const auto& hover = w.profile(probe)->hover;
            if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
            if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
            REQUIRE(w.removeVehicle(probe));
        }
        for (int m = 0; m < kCount; ++m) {
            session::VehicleSpec s = spec(std::string(a.type) + std::to_string(m), a.type, 0.02 * (m + 1));
            s.initial.pitchDeg = pitchDeg;
            s.initial.rollDeg = rollDeg;
            v[m] = w.createVehicle(s);
            REQUIRE(v[m] != 0);
            const auto r = w.submit(v[m], hoverHere());
            REQUIRE(r.accepted());
            act[m] = r.activity;
        }
        auto lost = [&](int m) {
            const auto& s = *w.vehicleState(v[m]);
            return s.diverged || !(std::abs(s.altitudeMslM - 100.0) < 60.0 + a.distanceM);
        };
        // settle: 20 s of hover from the spawn
        w.step(static_cast<unsigned>(std::lround(20.0 / dt)));
        for (int m = 0; m < kCount; ++m) {
            REQUIRE_FALSE(lost(m));
            const auto& s = *w.vehicleState(v[m]);
            lat0[m] = s.latitudeRad, lon0[m] = s.longitudeRad, h0[m] = s.altitudeMslM;
        }
        const auto s0 = [&](int m) -> const sim::VehicleState& { return *w.vehicleState(v[m]); };
        const double psi0 = s0(Heading).eulerRad[2];

        // the manoeuvres
        VelocityCommand climb = hoverHere();
        climb.verticalSpeedMs = a.climbMs;
        REQUIRE(w.update(act[Climb], climb).accepted());
        VelocityCommand turn = hoverHere();
        turn.headingRad = psi0 + 90.0 * kDeg;
        REQUIRE(w.update(act[Heading], turn).accepted());
        VelocityCommand go = hoverHere();
        go.northMs = a.speedMs;
        REQUIRE(w.update(act[Velocity], go).accepted());
        const auto rate = w.submit(v[Rate], AccelerationCommand{1.0, a.rateRadS, kHold, kHold, 0.0, 0.0});
        REQUIRE(rate.accepted());
        PositionCommand point;
        point.latitudeRad = lat0[Point] + a.distanceM / kEarthM;
        point.longitudeRad = lon0[Point] + a.distanceM / (kEarthM * std::cos(lat0[Point]));
        point.altitudeMslM = h0[Point] + 0.1 * a.distanceM;
        point.headingRad = 90.0 * kDeg;
        point.captureRadiusM = 0.05 * a.distanceM;
        REQUIRE(w.submit(v[Point], point).accepted());
        BehaviorCommand hover;
        hover.id = "hover";
        REQUIRE(w.submit(v[HoverBehaviour], hover).accepted());
        // axes apart: an autopilot holds the cyclic and the heading, a policy flies the thrust (a climb on the collective)
        {
            VelocityCommand hold = hoverHere();
            CommandOptions autopilot{Source::Autopilot, static_cast<AxisMask>(kCyclic | axisBit(Axis::Yaw))};
            const auto held = w.submit(v[Apart], hold, autopilot);
            REQUIRE(held.accepted());
            CHECK(w.activity(held.activity)->axes == (kCyclic | axisBit(Axis::Yaw)));
            const double trim = w.profile(v[Apart])->hover.throttleTrim;
            ActuatorCommand up{kHold, kHold, kHold, std::min(1.0, trim * 1.08), kHold, kHold, kHold, kHold};
            REQUIRE(w.submit(v[Apart], up, CommandOptions{Source::Policy, axisBit(Axis::Thrust)}).accepted());
        }
        REQUIRE(w.setVehicleDefault(v[Default], VehicleDefault::Hold) == Reason::None);
        REQUIRE(w.cancel(act[Default]).status == CommandStatus::Canceled);
        // protection: a bank beyond the envelope's is limited to it
        const double bankMax = w.profile(v[Protected])->envelope.clean.bankMaxRad;
        const auto steep = w.submit(v[Protected], AttitudeCommand{bankMax + 20.0 * kDeg, 0.0, kHold, 0.785, kHold, kHold});
        REQUIRE(steep.accepted());
        CHECK((steep.flags & kClamped) != 0);

        std::vector<double> vz, psi, vn, p;
        double worstBank = 0.0, apartClimbM = 0.0;
        bool anyLost = false;
        // a manoeuvre that cannot go on (a roll rate, a bank, a climb) ends in the hover, taken back by the autopilot
        auto backToHover = [&](int m) { REQUIRE(w.submit(v[m], hoverHere(), CommandOptions{Source::Autopilot}).accepted()); };
        const int n = static_cast<int>(std::lround(a.arriveS / dt));
        for (int k = 0; k < n; ++k) {
            w.step();
            const double t = (k + 1) * dt;
            for (int m = 0; m < kCount; ++m) anyLost = anyLost || lost(m);
            if (t <= 15.0) vz.push_back(-s0(Climb).velocityNedMs[2]), psi.push_back(std::remainder(s0(Heading).eulerRad[2] - psi0, 2.0 * kPi));
            if (t <= 30.0) vn.push_back(s0(Velocity).velocityNedMs[0]);
            if (t <= a.rateS) p.push_back(s0(Rate).angularRateBodyRadS[0]);
            if (t <= 4.0) worstBank = std::max(worstBank, std::abs(s0(Protected).eulerRad[0]));
            if (k + 1 == static_cast<int>(std::lround(a.rateS / dt))) backToHover(Rate);
            if (k + 1 == static_cast<int>(std::lround(4.0 / dt))) backToHover(Protected);
            if (k + 1 == static_cast<int>(std::lround(10.0 / dt))) {
                apartClimbM = s0(Apart).altitudeMslM - h0[Apart];
                backToHover(Apart);
            }
        }
        CHECK_FALSE(anyLost);
        // the hover held
        double north, east;
        offset(s0(Hover), lat0[Hover], lon0[Hover], north, east);
        CHECK(std::hypot(north, east) < a.driftM);
        CHECK(std::abs(s0(Hover).altitudeMslM - h0[Hover]) < 0.1 * a.driftM + 0.2);
        // the steps
        const Step climbStep = stepOf(vz, a.climbMs, dt, anyLost);
        CHECK(climbStep.passes());
        CHECK(climbStep.riseS < 5.0);
        const Step headingStep = stepOf(psi, 90.0 * kDeg, dt, anyLost);
        CHECK(headingStep.passes());
        CHECK(std::abs(psi.back() - 90.0 * kDeg) < 3.0 * kDeg);
        const Step speedStep = stepOf(vn, a.speedMs, dt, anyLost);
        CHECK(speedStep.passes());
        const Step rateStep = stepOf(p, a.rateRadS, dt, anyLost);
        CHECK(rateStep.passes());
        // the point: arrived, facing east, at the height
        offset(s0(Point), lat0[Point], lon0[Point], north, east);
        CHECK(std::hypot(north - a.distanceM, east - a.distanceM) < 0.05 * a.distanceM);
        CHECK(std::abs(s0(Point).altitudeMslM - (h0[Point] + 0.1 * a.distanceM)) < 0.02 * a.distanceM + 0.2);
        CHECK(std::abs(std::remainder(s0(Point).eulerRad[2] - 90.0 * kDeg, 2.0 * kPi)) < 5.0 * kDeg);
        // the hover behaviour held where it started
        offset(s0(HoverBehaviour), lat0[HoverBehaviour], lon0[HoverBehaviour], north, east);
        CHECK(std::hypot(north, east) < a.driftM);
        // the thrust flown apart climbed while the autopilot held the rest
        offset(s0(Apart), lat0[Apart], lon0[Apart], north, east);
        CHECK(apartClimbM > 1.0);
        CHECK(std::hypot(north, east) < 2.0 * a.driftM);
        // the vehicle default's hold kept it
        offset(s0(Default), lat0[Default], lon0[Default], north, east);
        CHECK(std::hypot(north, east) < 2.0 * a.driftM);
        CHECK(std::abs(s0(Default).altitudeMslM - h0[Default]) < 0.1 * a.driftM + 0.5);
        // protection kept the bank near the envelope's
        CHECK(worstBank < bankMax + 3.0 * kDeg);
        UNSCOPED_INFO(a.type << ": climbed apart " << apartClimbM << " m, climb rise " << climbStep.riseS << " s overshoot " << climbStep.overshoot << ", heading rise " << headingStep.riseS
                             << " s, speed rise " << speedStep.riseS << " s overshoot " << speedStep.overshoot << ", rate rise " << rateStep.riseS
                             << " s overshoot " << rateStep.overshoot << ", worst bank " << worstBank / kDeg << " deg of " << bankMax / kDeg);
    }
}

TEST_CASE("rotorcraft: two worlds fly the same commands to the same bits", "[rotorcraft]") {
    std::vector<double> first;
    for (int run = 0; run < 2; ++run) {
        session::World w(options("rotorcraft-determinism"));
        std::vector<std::uint32_t> ids;
        for (const auto& a : kAircraft) {
            ids.push_back(w.createVehicle(spec(a.type, a.type, 0.01 * static_cast<double>(ids.size() + 1))));
            REQUIRE(ids.back() != 0);
            VelocityCommand go = hoverHere();
            go.northMs = a.speedMs;
            go.headingRad = 0.5;
            REQUIRE(w.submit(ids.back(), go).accepted());
        }
        w.step(600);
        std::vector<double> bits;
        for (const auto id : ids) {
            const auto& s = *w.vehicleState(id);
            bits.insert(bits.end(), {s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.eulerRad[0], s.eulerRad[1], s.eulerRad[2]});
        }
        if (run == 0) first = bits;
        else CHECK(bits == first);
    }
}

TEST_CASE("rotorcraft: asked beyond its top speed and flown so, a multirotor's velocity loop holds its fastest at its tilt, "
          "and comes back asked less",
          "[rotorcraft]") {
    // (ADR-29 FA-3e's open finding, the Crazyflie diverging asked for 25 and 30 m/s: not its loops - that repro let it fall
    // for 5 s under the neutral vehicle default, and it struck the ground at 35 m/s, the crash docs/rotorcraft.md section 7
    // names. Settled, unchecked - held to nothing, as a trainer's command is flown - it saturates at its tilt and holds
    // its fastest along its nose, a little past its tables' top: the Crazyflie 18.30 m/s against 18.0, the IRIS+ 13.23 against
    // 13.09, their heights within 0.1 mm)
    session::World w(options("rotorcraft-saturated"));
    const double dt = w.dt() * w.frameSkip();
    auto nose = [](const sim::VehicleState& s) { return s.airspeedTrueMs * std::cos(s.alphaRad) * std::cos(s.betaRad); };
    struct Case {
        const char* type;
        double askedMs;
    };
    for (const Case c : {Case{"cf2", 30.0}, Case{"iris", 40.0}}) {
        INFO(c.type);
        const auto id = w.createVehicle(spec(std::string(c.type) + "-saturated", c.type, 0.0));
        REQUIRE(id != 0);
        REQUIRE(w.submit(id, hoverHere()).accepted());
        w.step(static_cast<unsigned>(std::lround(10.0 / dt)));
        const sim::VehicleState start = *w.vehicleState(id);
        const double top = topTasMs(&w.profile(id)->tables, start.altitudeMslM, start.fuelKg), tilt = w.performance(id)->maxTiltRad;
        REQUIRE(std::isfinite(top));
        HsaCommand fast;
        fast.headingRad = 0.0, fast.speed = c.askedMs, fast.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
        fast.altitudeM = start.altitudeMslM;
        CommandOptions unchecked;
        unchecked.range = RangePolicy::None;
        const CommandResult r = w.submit(id, fast, unchecked);
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) == 0);
        double fastest = 0.0, worstHeightM = 0.0;
        const auto n = static_cast<unsigned>(std::lround(90.0 / dt));
        for (unsigned k = 0; k < n; ++k) {
            w.step();
            const sim::VehicleState& s = *w.vehicleState(id);
            REQUIRE_FALSE(s.diverged);
            if (static_cast<double>(k) * dt < 60.0) continue; // (on its way up to it)
            fastest = std::max(fastest, nose(s));
            worstHeightM = std::max(worstHeightM, std::abs(s.altitudeMslM - start.altitudeMslM));
        }
        const sim::VehicleState& held = *w.vehicleState(id);
        INFO("its fastest " << fastest << " m/s along the nose, its tables' top " << top << ", pitched " << held.eulerRad[1] / kDeg << " deg");
        CHECK(w.activity(r.activity)->state == ActivityState::Active);
        CHECK(std::abs(fastest - top) < 0.03 * top);
        CHECK(worstHeightM < 0.5);
        CHECK(std::abs(held.eulerRad[1] + tilt) < 1.0 * kDeg); // nose down at its tilt
        // asked half its top, it comes back to it
        HsaCommand slower;
        slower.speed = 0.5 * top;
        REQUIRE(w.update(r.activity, slower).accepted());
        w.step(static_cast<unsigned>(std::lround(30.0 / dt)));
        CHECK(std::abs(nose(*w.vehicleState(id)) - 0.5 * top) < 0.02 * top);
        REQUIRE(w.removeVehicle(id));
    }
}

TEST_CASE("rotorcraft: a quadrotor that strikes the ground comes to rest, whatever its attitude", "[rotorcraft]") {
    // Dropped with its motors off (the vehicle default's neutral) from 62 m, it meets the ground at 35 m/s: the
    // Crazyflie moves sixteen times its legs' length in a step. The ground's forces, applied for a step, sent its
    // contacts back out faster than they came in, and the Crazyflie diverged (docs/rotorcraft.md, 7; JSBSim's
    // FGAccelerations, patched). Now no contact throws it back faster than it struck, and it comes to rest.
    struct Drop {
        const char* name;
        double pitchDeg, rollDeg;
    };
    const Drop drops[] = {{"level", 0.0, 0.0}, {"tilted", 10.0, 25.0}, {"nose first", -60.0, 0.0}, {"on its edge", 0.0, 80.0},
                          {"inverted", 0.0, 180.0}};
    for (const char* type : {"cf2", "iris"}) {
        session::WorldOptions o = options("rotorcraft-impact");
        o.frameSkip = 1; // every step of the flight model seen
        session::World w(o);
        struct Flight {
            const Drop* drop;
            std::uint32_t id = 0;
            double nearMs = -1.0;   ///< its speed 1 m above the ground
            double nearS = -1.0;    ///< and when
            double fastestMs = 0.0; ///< its fastest from there on
        };
        std::vector<Flight> flights;
        for (const auto& d : drops) {
            session::VehicleSpec s = spec(std::string(type) + " " + d.name, type, 0.01 * static_cast<double>(flights.size() + 1));
            s.initial.altitudeMslM = 62.0;
            s.initial.pitchDeg = d.pitchDeg;
            s.initial.rollDeg = d.rollDeg;
            flights.push_back({&d, w.createVehicle(s)});
            REQUIRE(flights.back().id != 0);
        }
        for (int k = 0; k < 8 * 120; ++k) {
            w.step();
            for (auto& f : flights) {
                const auto& s = *w.vehicleState(f.id);
                INFO(type << " " << f.drop->name << " at " << s.simTime << " s");
                REQUIRE_FALSE(s.diverged);
                const double speed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]);
                if (f.nearS < 0.0 && s.altitudeAglM < 1.0) {
                    f.nearMs = speed;
                    f.nearS = s.simTime;
                }
                if (f.nearS >= 0.0) f.fastestMs = std::max(f.fastestMs, speed);
            }
        }
        for (const auto& f : flights) {
            const auto& s = *w.vehicleState(f.id);
            INFO(type << " " << f.drop->name << ": " << f.nearMs << " m/s 1 m up, the fastest after " << f.fastestMs
                      << " m/s; 8 s on " << std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2])
                      << " m/s, " << s.altitudeAglM << " m up");
            REQUIRE(f.nearS > 0.0);
            CHECK(f.nearMs > 30.0);
            // (the last metre's fall adds 1 %; before, the Crazyflie left the ground 1.6 to 6.4 times as fast, and
            // three of the five diverged; the IRIS+ up to 4.3 times)
            CHECK(f.fastestMs < 1.05 * f.nearMs);
            // at rest on the ground at least 4 s after it struck (the fastest 0.004 m/s, the IRIS+ on its back; on
            // their contacts before they were sized for the step, the Crazyflie still rocked at 0.05 m/s and the
            // IRIS+ lay on its side, its centre of gravity 0.21 m up)
            CHECK(s.simTime - f.nearS > 4.0);
            CHECK(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]) < 0.5);
            CHECK(s.altitudeAglM < 0.3);
        }
    }
}

TEST_CASE("rotorcraft: a quadrotor parked stays still, and dropped a little comes to rest without bouncing up again", "[rotorcraft]") {
    // Its contacts are sized for the step (hangar's rotorcraft/multi.py, contact_set). The Crazyflie's own, 60 N/m
    // and 1.2 N s/m each on 27 g, made it hop parked, its feet off the ground a quarter of the time at up to
    // 0.37 m/s, and dropped on its back from 5 cm it rose 12 cm; the IRIS+ rocked on its feet and never came to
    // rest (docs/rotorcraft.md, 7).
    struct Type {
        const char* name;
        double legM, topM; ///< its feet below the c.g., and its top above (the lowest point upright and upside down)
    };
    struct Case {
        const char* name;
        double heightM; ///< its lowest point above the ground as it is let go; 0: parked
        double rollDeg;
    };
    const Case cases[] = {{"parked", 0.0, 0.0},
                          {"level from 5 cm", 0.05, 0.0},
                          {"level from 30 cm", 0.30, 0.0},
                          {"on its back from 5 cm", 0.05, 180.0},
                          {"on its back from 30 cm", 0.30, 180.0}};
    for (const Type& t : {Type{"cf2", 0.018, 0.001}, Type{"iris", 0.054, 0.047}}) {
        session::WorldOptions o = options("rotorcraft-rest");
        o.frameSkip = 1; // every step of the flight model seen
        session::World w(o);
        struct Flight {
            const Case* c;
            std::uint32_t id = 0;
            double releaseM = 0.0;  ///< its c.g.'s height as it is let go
            bool rose = false;      ///< it has moved up since
            double highestM = 0.0;  ///< the c.g.'s highest after that
            double fastestUp = 0.0; ///< its fastest up
            int offGround = 0;      ///< steps with no foot touching (parked)
        };
        std::vector<Flight> flights;
        for (const auto& c : cases) {
            session::VehicleSpec s = spec(std::string(t.name) + " " + c.name, t.name, 0.01 * static_cast<double>(flights.size() + 1));
            if (c.heightM > 0.0) {
                s.initial.altitudeMslM = c.heightM + (c.rollDeg > 90.0 ? t.topM : t.legM); // the ground at 0 m
                s.initial.rollDeg = c.rollDeg;
            } else {
                s.initial.onGround = true;
            }
            flights.push_back({&c, w.createVehicle(s)});
            REQUIRE(flights.back().id != 0);
            flights.back().releaseM = w.vehicleState(flights.back().id)->altitudeAglM;
        }
        for (int k = 0; k < 6 * 120; ++k) {
            w.step();
            for (auto& f : flights) {
                const auto& s = *w.vehicleState(f.id);
                const double up = -s.velocityNedMs[2];
                f.rose = f.rose || up > 0.0;
                if (f.rose) f.highestM = std::max(f.highestM, s.altitudeAglM);
                f.fastestUp = std::max(f.fastestUp, up);
                if (!s.onGround) ++f.offGround;
            }
        }
        for (const auto& f : flights) {
            const auto& s = *w.vehicleState(f.id);
            const double speed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]);
            const double rate = std::max({std::abs(s.angularRateBodyRadS[0]), std::abs(s.angularRateBodyRadS[1]),
                                          std::abs(s.angularRateBodyRadS[2])});
            const double impactMs = std::sqrt(2.0 * 9.80665 * f.c->heightM);
            INFO(t.name << " " << f.c->name << ": let go at " << f.releaseM << " m, the highest after it moved up " << f.highestM
                        << " m, the fastest up " << f.fastestUp << " m/s (struck at " << impactMs << "), off the ground "
                        << f.offGround << " steps; 6 s on " << speed << " m/s, " << rate << " rad/s");
            if (f.c->heightM > 0.0) {
                // it never rises half as high as it fell from, nor moves up as fast as it struck (the highest: the
                // Crazyflie's centre of gravity 26 mm up after 30 cm level, 11 mm over where it rests; the fastest
                // up: 0.82 of its strike, the same flight)
                CHECK(f.highestM < f.releaseM - 0.5 * f.c->heightM);
                CHECK(f.fastestUp < impactMs);
            } else {
                CHECK(f.offGround == 0);
                // its drawn feet on the ground: its contacts sunk by the legs' deflection under its weight, the c.g.
                // the legs' height up (it stood 2.9 and 4.1 mm lower, its feet drawn into the ground)
                CHECK(std::abs(s.altitudeAglM - t.legM) < 1e-4);
            }
            // at rest (the last to settle: the IRIS+ on its back, 4.2 s after it was let go)
            CHECK(speed < 1e-3);
            CHECK(rate < 1e-2);
        }
    }
}

TEST_CASE("rotorcraft: a Crazyflie commanded after it has fallen 124 m strikes the ground and flies on or lies there", "[rotorcraft]") {
    // The case the finding was traced in: 5 s with no command (its motors off) from 150 m, then an HSA. It meets the
    // ground at 35 m/s as its motors spin up; it was thrown back up at 84 m/s, tumbling, and diverged a second later.
    session::WorldOptions o = options("rotorcraft-impact-hsa");
    o.seed = 0;
    session::World w(o);
    session::VehicleSpec s = spec("cf2", "cf2", 0.0);
    s.initial.latitudeDeg = 30.0; // where it was found
    s.initial.longitudeDeg = 0.0;
    s.initial.altitudeMslM = 150.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    const double stepS = w.dt() * w.frameSkip();
    w.step(static_cast<unsigned>(std::lround(5.0 / stepS)));
    HsaCommand hsa;
    hsa.headingRad = 0.0;
    hsa.speed = 10.0;
    hsa.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
    REQUIRE(w.submit(id, hsa).accepted());
    double nearMs = -1.0, fastest = 0.0; // its speed 1 m above the ground, and its fastest from there on
    for (long k = 0; k < std::lround(10.0 / stepS); ++k) {
        w.step();
        const auto& st = *w.vehicleState(id);
        INFO("at " << st.simTime << " s");
        REQUIRE_FALSE(st.diverged);
        const double speed = std::hypot(st.velocityNedMs[0], st.velocityNedMs[1], st.velocityNedMs[2]);
        if (nearMs < 0.0 && st.altitudeAglM < 1.0) nearMs = speed;
        if (nearMs >= 0.0) fastest = std::max(fastest, speed);
    }
    // (it strikes at 35 m/s, its motors spinning up; it was thrown back up at 84)
    INFO(nearMs << " m/s 1 m up, the fastest after " << fastest << " m/s");
    CHECK(nearMs > 30.0);
    CHECK(fastest < 1.05 * nearMs);
}

TEST_CASE("rotorcraft: a helicopter let go from its hover, or dropped on its back or its side, stays whole on the ground", "[rotorcraft]") {
    // docs/rotorcraft.md, 7 ("The helicopters on the ground"). Let go from its hover at 150 m (the fleet test's case),
    // the UH-1H struck its skids at 16 m/s, pitched up over their rear ends, rolled over and diverged. Dropped on
    // their backs or sides, their only contacts - skids or wheels - pointing up, the airframes sank into the ground:
    // the UH-1H's main rotor, the air turning it faster, spun the airframe until it diverged; the UH-60A fell 440 m
    // through. Now the airframe meets the ground where no gear is (hangar's structure contacts: the tail skid, the
    // stabilizer, the nose, the hubs), and it feels the rotors' drive - the engine's torque - not an air-driven
    // rotor's.
    struct Case {
        const char* name;
        double altitudeM, rollDeg;
        bool fromHover; ///< held in its hover 10 s at its hover attitude, then let go (the vehicle default's neutral)
    };
    const Case cases[] = {{"let go from its hover", 150.0, 0.0, true}, {"dropped on its back", 10.0, 180.0, false},
                          {"dropped on its side", 5.0, 90.0, false}};
    for (const char* type : {"uh1h", "uh60"}) {
        session::WorldOptions o = options("rotorcraft-helicopter-ground");
        o.frameSkip = 1; // every step of the flight model seen
        session::World w(o);
        const double stepS = w.dt() * w.frameSkip();
        double pitchDeg = 0.0, rollDeg = 0.0; // its hover's attitude
        {
            const std::uint32_t probe = w.createVehicle(spec(std::string(type) + "-probe", type, 0.5));
            REQUIRE(probe != 0);
            const auto& hover = w.profile(probe)->hover;
            if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
            if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
            REQUIRE(w.removeVehicle(probe));
        }
        struct Flight {
            const Case* c;
            std::uint32_t id = 0;
            ActivityId held{};
            double fallMs = 0.0;      ///< its fastest before the ground first pushes on it (skids, wheels or airframe)
            double strikeS = -1.0;    ///< when it does
            double fastestMs = 0.0;   ///< its fastest from then on
            double lowestM = kInf;    ///< its c.g.'s least height from then on
        };
        std::vector<Flight> flights;
        for (const auto& c : cases) {
            session::VehicleSpec s = spec(std::string(type) + " " + c.name, type, 0.01 * static_cast<double>(flights.size() + 1));
            s.initial.altitudeMslM = c.altitudeM;
            s.initial.pitchDeg = c.fromHover ? pitchDeg : 0.0;
            s.initial.rollDeg = c.fromHover ? rollDeg : c.rollDeg;
            Flight f{&c, w.createVehicle(s)};
            REQUIRE(f.id != 0);
            if (c.fromHover) {
                const auto r = w.submit(f.id, hoverHere());
                REQUIRE(r.accepted());
                f.held = r.activity;
            }
            flights.push_back(f);
        }
        const long holdSteps = std::lround(10.0 / stepS);
        for (long k = 0; k < std::lround(45.0 / stepS); ++k) {
            if (k == holdSteps)
                for (const auto& f : flights)
                    if (f.c->fromHover) REQUIRE(w.cancel(f.held).status == CommandStatus::Canceled);
            w.step();
            for (auto& f : flights) {
                const auto& s = *w.vehicleState(f.id);
                INFO(type << " " << f.c->name << " at " << s.simTime << " s");
                REQUIRE_FALSE(s.diverged);
                const double speed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]);
                auto* m = w.model(f.id);
                if (f.strikeS < 0.0 && std::hypot(m->property("forces/fbx-gear-lbs").get(), m->property("forces/fby-gear-lbs").get(),
                                                  m->property("forces/fbz-gear-lbs").get()) > 0.0)
                    f.strikeS = s.simTime;
                if (f.strikeS < 0.0) f.fallMs = std::max(f.fallMs, speed);
                else {
                    f.fastestMs = std::max(f.fastestMs, speed);
                    f.lowestM = std::min(f.lowestM, s.altitudeAglM);
                }
            }
        }
        for (const auto& f : flights) {
            const auto& s = *w.vehicleState(f.id);
            const double speed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]);
            INFO(type << " " << f.c->name << ": " << f.fallMs << " m/s as it struck, the fastest after " << f.fastestMs
                      << " m/s, its c.g. at least " << f.lowestM << " m up; at the end " << speed << " m/s, roll "
                      << s.eulerRad[0] / kDeg << ", pitch " << s.eulerRad[1] / kDeg << " deg, " << s.altitudeAglM << " m up");
            REQUIRE(f.strikeS > 0.0);
            // (let go, the UH-1H strikes at 16 m/s, the UH-60A at 23; dropped, at 8 to 12)
            CHECK(f.fastestMs < 1.05 * f.fallMs);
            // its airframe on the ground, not in it (its c.g. at least 0.8 m up; parked, 1.4 and 1.6 m)
            CHECK(f.lowestM > 0.5);
            // still or nearly: its rotors turn on at their governed speed, and the UH-1H on its side rocks under its
            // tail rotor's thrust (0.9 m/s at most); let go, the UH-60A turns on its wheels at 1.3 m/s, as it did
            CHECK(speed < 2.0);
            if (f.c->fromHover) { // on its gear: tipped onto the UH-1H's tail skid, never over
                CHECK(std::abs(s.eulerRad[0]) < 20.0 * kDeg);
                CHECK(std::abs(s.eulerRad[1]) < 20.0 * kDeg);
            }
        }
    }
}

TEST_CASE("rotorcraft: a helicopter's rotor speed stops where its rotors' does, driven past it", "[rotorcraft]") {
    // Collective down and the cyclic forward from 2,000 m, the engine left to its governor: the air drives the rotor
    // up to FGRotor's 130 % stop. The flight control system's rotor speed, which drives the engine and the rotor-drive
    // moment, had no stop of its own and wound on past it (the UH-1H to 895 %); now it stops there too
    // (docs/rotorcraft.md, 7).
    for (const char* type : {"uh1h", "uh60"}) {
        session::World w(options("rotorcraft-rotor-stop"));
        session::VehicleSpec s = spec(type, type, 0.0);
        s.initial.altitudeMslM = 2000.0;
        s.initial.airspeedTrueMs = 40.0;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        const double stepS = w.dt() * w.frameSkip();
        w.step(static_cast<unsigned>(std::lround(2.0 / stepS)));
        const std::string fcs = std::string("fcs/") + type + "/rotor-rpm";
        const double nominal = w.model(id)->property(fcs).get();
        REQUIRE(w.submit(id, ActuatorCommand{0.0, 0.6, 0.0, 0.0, 0.0, kHold, 0.0, 0.0}).accepted());
        double most = 0.0, rotorMost = 0.0;
        for (long k = 0; k < std::lround(40.0 / stepS); ++k) {
            w.step();
            REQUIRE_FALSE(w.vehicleState(id)->diverged);
            most = std::max(most, w.model(id)->property(fcs).get());
            rotorMost = std::max(rotorMost, w.model(id)->property("propulsion/engine[0]/rotor-rpm").get());
        }
        INFO(type << ": the flight control system's rotor speed at most " << most << " rpm, the rotor's " << rotorMost << ", nominal " << nominal);
        CHECK(rotorMost > 1.29 * nominal); // (driven to the stop)
        CHECK(most <= 1.3 * nominal + 1e-6);
    }
}

TEST_CASE("rotorcraft: the stock AH-1S's engine starts in a reused slot as on a new model", "[rotorcraft]") {
    // Its rotor is driven through a transmission whose free-wheel unit lags its coupling. The rotor's reset left that
    // lag as the last run did, and the engine started 2 % faster than on a new model (JSBSim's FGTransmission and
    // FGRotor, patched: cmake/JsbsimPatches.cmake).
    auto engineAtStart = [](bool reused) {
        session::World w(options("rotorcraft-ah1s-reuse"));
        session::VehicleSpec s = spec("ah1s", "ah1s", 0.0);
        s.initial.altitudeMslM = 150.0;
        if (reused) {
            const auto first = w.createVehicle(s);
            REQUIRE(first != 0);
            REQUIRE(w.removeVehicle(first));
        }
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        return w.model(id)->property("propulsion/engine/engine-rpm").get();
    };
    const double fresh = engineAtStart(false), reused = engineAtStart(true);
    INFO("engine rpm at the start: " << fresh << " on a new model, " << reused << " in a reused slot");
    CHECK(std::memcmp(&fresh, &reused, sizeof(double)) == 0);
}
