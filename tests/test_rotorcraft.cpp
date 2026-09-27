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
