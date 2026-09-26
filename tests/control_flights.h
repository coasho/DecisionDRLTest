#pragma once

// Scripted flights through the legacy command path (World::command), the
// same calls every existing entry point makes. The control architecture's
// compatibility checks fly them (docs/control-architecture.md, sections 12
// and 13): fsim_control_bench's digests compare a build with the one before
// it bit for bit, and the stock-aircraft checkpoints test compares the c172x
// flights with values recorded before the architecture changed.

#include "fsim/Control.h"
#include "session/World.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace fsim::flights {

inline constexpr double kDeg = 3.14159265358979323846 / 180.0;
inline constexpr double kLatDeg = 47.25, kLonDeg = 8.5; // over the Swiss plateau, well above the ground

inline session::WorldOptions worldOptions(const char* name, const char* jsbsimRoot, unsigned workers = 1) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = jsbsimRoot;
    o.workers = workers;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 11;
    return o;
}

inline session::VehicleSpec spec(const std::string& name, const std::string& type, int index, double altitudeM, double tasMs) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.latitudeDeg = kLatDeg + 0.02 * (index % 8);
    s.initial.longitudeDeg = kLonDeg + 0.03 * (index / 8);
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 20.0 * index;
    s.initial.airspeedTrueMs = tasMs;
    return s;
}

/// One flight: a vehicle and what it is told before each world step.
struct Flight {
    std::string name;
    std::uint32_t id = 0;
    int kind = 0;
    /// Flown by the control loops (not open loop): stable, so a toolchain's
    /// last-bit differences stay that small - the checkpoints keep only these.
    bool closedLoop = false;
};

/// The stock c172x flights the checkpoints test records: every command level,
/// a behaviour with and without parameters, and a vehicle nobody commands.
enum Kind : int {
    Uncommanded,
    Actuator,
    Attitude,
    Acceleration,
    Velocity,
    Position,
    Hold,
    Loiter,
    // the digests' extra flights
    Waypoints,
    Aerobatics,
    Pursuit,
    Formation,
    Evade,
    LevelSwitch,
    Reset,
    PerStep,
    FbwVelocity,
    FbwAcceleration,
    DirectVelocity,
    DirectAttitude,
};

inline control::BehaviorCommand behavior(const char* id) {
    control::BehaviorCommand b;
    b.id = id;
    return b;
}

class Flights {
public:
    /// The c172x flights (checkpoints); with `all`, the digests' flights too
    /// (behaviours that follow other vehicles, level switches, a reset, a
    /// command every step, and two hangar designs when they are found).
    Flights(session::World& world, bool all) : world_(world) {
        const char* c172 = "jsbsim:c172x";
        add("uncommanded", c172, Uncommanded, 1500.0, 55.0);
        add("actuator", c172, Actuator, 1500.0, 55.0);
        add("attitude", c172, Attitude, 1500.0, 55.0);
        add("acceleration", c172, Acceleration, 1500.0, 55.0);
        add("velocity", c172, Velocity, 1500.0, 55.0);
        add("position", c172, Position, 1500.0, 55.0);
        add("hold", c172, Hold, 1500.0, 55.0);
        add("loiter", c172, Loiter, 1500.0, 55.0);
        if (!all) return;
        add("waypoints", c172, Waypoints, 1500.0, 55.0);
        add("aerobatics", c172, Aerobatics, 2000.0, 60.0);
        add("pursuit", c172, Pursuit, 1500.0, 55.0);
        add("formation", c172, Formation, 1500.0, 55.0);
        add("evade", c172, Evade, 1800.0, 55.0);
        add("level_switch", c172, LevelSwitch, 1500.0, 55.0);
        add("reset", c172, Reset, 1500.0, 55.0);
        add("per_step", c172, PerStep, 1500.0, 55.0);
        add("f16c_velocity", "jsbsim:f16c", FbwVelocity, 3000.0, 160.0);
        add("f16c_acceleration", "jsbsim:f16c", FbwAcceleration, 3000.0, 160.0);
        add("b52h_velocity", "jsbsim:b52h", DirectVelocity, 3000.0, 180.0);
        add("b52h_attitude", "jsbsim:b52h", DirectAttitude, 3000.0, 180.0);
    }

    const std::vector<Flight>& flights() const noexcept { return flights_; }

    /// Called before world step `step` (0-based).
    void drive(int step) {
        for (const auto& f : flights_) command(f, step);
    }

private:
    void add(const std::string& name, const char* type, int kind, double altitudeM, double tasMs) {
        const auto id = world_.createVehicle(spec(name, type, static_cast<int>(flights_.size()), altitudeM, tasMs));
        if (id == 0) return; // a hangar design not found: that flight is skipped
        flights_.push_back({name, id, kind, kind >= Attitude && kind <= Loiter});
    }

    const Flight* find(int kind) const noexcept {
        for (const auto& f : flights_)
            if (f.kind == kind) return &f;
        return nullptr;
    }

    void command(const Flight& f, int step) {
        using namespace control;
        auto& w = world_;
        const auto* s = w.vehicleState(f.id);
        switch (f.kind) {
        case Uncommanded: break;
        case Actuator:
            if (step == 0) w.command(f.id, ActuatorCommand{0.02, -0.03, 0.0, 0.7});
            break;
        case Attitude:
            if (step == 0) w.command(f.id, AttitudeCommand{0.2, 0.05, kHold, 0.785, kHold, 55.0});
            break;
        case Acceleration:
            if (step == 0) w.command(f.id, AccelerationCommand{1.0, 0.0, kHold, 0.65});
            break;
        case Velocity:
            if (step == 0) w.command(f.id, VelocityCommand{55.0, 1.0, 1.0, kHold});
            break;
        case Position:
            if (step == 0) w.command(f.id, PositionCommand{s->latitudeRad + 0.002, s->longitudeRad + 0.002, 1700.0, 55.0, 200.0});
            break;
        case Hold:
            if (step == 0) w.command(f.id, behavior("hold"));
            break;
        case Loiter:
            if (step == 0) {
                auto b = behavior("loiter");
                b.params = {{"radius_m", 800.0}, {"altitude_m", 1600.0}, {"clockwise", 0.0},
                            {"lat_deg", s->latitudeRad / kDeg + 0.01}, {"lon_deg", s->longitudeRad / kDeg}};
                w.command(f.id, b);
            }
            break;
        case Waypoints:
            if (step == 0) {
                auto b = behavior("waypoints");
                for (int i = 1; i <= 3; ++i)
                    b.points.push_back(PositionCommand{s->latitudeRad + 0.0006 * i, s->longitudeRad + 0.0004 * (i % 2), 1500.0 + 50.0 * i, 55.0, 150.0});
                w.command(f.id, b);
            }
            break;
        case Aerobatics:
            if (step == 0) {
                auto b = behavior("aerobatics");
                b.params = {{"manoeuvre", 1.0}, {"load_factor_g", 3.0}};
                w.command(f.id, b);
            }
            break;
        case Pursuit:
            if (step == 0)
                if (const auto* t = find(Hold)) {
                    auto b = behavior("pursuit");
                    b.target = t->id;
                    b.params = {{"range_m", 400.0}};
                    w.command(f.id, b);
                }
            break;
        case Formation:
            if (step == 0)
                if (const auto* t = find(Velocity)) {
                    auto b = behavior("formation");
                    b.target = t->id;
                    w.command(f.id, b);
                }
            break;
        case Evade:
            if (step == 0)
                if (const auto* t = find(Pursuit)) {
                    auto b = behavior("evade");
                    b.target = t->id;
                    w.command(f.id, b);
                }
            break;
        case LevelSwitch:
            switch (step % 400) {
            case 0: w.command(f.id, AttitudeCommand{-0.15, 0.04, kHold, 0.785, kHold, 55.0}); break;
            case 100: w.command(f.id, VelocityCommand{58.0, -1.0, kHold, 0.05}); break;
            case 200: w.command(f.id, behavior("hold")); break;
            case 300: w.command(f.id, AccelerationCommand{1.05, -0.05, kHold, 0.6}); break;
            default: break;
            }
            break;
        case Reset:
            if (step == 0 || step == 301) w.command(f.id, VelocityCommand{57.0, 1.5, kHold, 0.03});
            if (step == 300) w.resetVehicle(f.id);
            break;
        case PerStep: {
            // as a policy acts: a new attitude setpoint every step
            const double t = step / 30.0;
            w.command(f.id, AttitudeCommand{0.3 * std::sin(0.4 * t), 0.04 + 0.03 * std::cos(0.3 * t), kHold, 0.785, 0.6 + 0.1 * std::sin(0.2 * t), kHold});
            break;
        }
        case FbwVelocity:
            if (step == 0) w.command(f.id, VelocityCommand{165.0, 5.0, kHold, 0.04});
            break;
        case FbwAcceleration:
            if (step == 0) w.command(f.id, AccelerationCommand{2.0, 0.3, kHold, 0.8});
            if (step == 150) w.command(f.id, AccelerationCommand{1.0, 0.0, kHold, 0.8});
            break;
        case DirectVelocity:
            if (step == 0) w.command(f.id, VelocityCommand{185.0, 3.0, kHold, 0.02});
            break;
        case DirectAttitude:
            if (step == 0) w.command(f.id, AttitudeCommand{0.25, 0.03, kHold, 0.785, kHold, 180.0});
            break;
        default: break;
        }
    }

    session::World& world_;
    std::vector<Flight> flights_;
};

/// FNV-1a over the values of a state and the inputs flown, field by field
/// (never the padding bytes).
class Digest {
public:
    void add(const sim::VehicleState& s, const sim::ControlInputs& in) noexcept {
        add(s.simTime);
        addN(s.positionEcef, 3);
        addN(s.attitudeEcefToBody, 4);
        addN(s.eulerRad, 3);
        addN(s.velocityBodyMs, 3);
        addN(s.velocityNedMs, 3);
        addN(s.angularRateBodyRadS, 3);
        addN(s.accelerationBodyMs2, 3);
        add(s.airspeedTrueMs);
        add(s.airspeedCalibratedMs);
        add(s.alphaRad);
        add(s.betaRad);
        add(s.loadFactor);
        add(s.aileronRad);
        add(s.elevatorRad);
        add(s.rudderRad);
        add(s.flapsRad);
        add(s.gearPosition);
        addN(s.throttlePosition, sim::VehicleState::kMaxEngines);
        addN(s.thrustN, sim::VehicleState::kMaxEngines);
        add(s.fuelKg);
        add(in.aileron);
        add(in.elevator);
        add(in.rudder);
        addN(in.throttle, sim::ControlInputs::kMaxEngines);
        add(in.flaps);
        add(in.gearDown);
        add(in.brakeLeft);
        add(in.brakeRight);
    }
    std::uint64_t value() const noexcept { return h_; }

private:
    void add(double v) noexcept {
        unsigned char b[sizeof(double)];
        std::memcpy(b, &v, sizeof v);
        for (unsigned char c : b) h_ = (h_ ^ c) * 1099511628211ull;
    }
    void addN(const double* v, int n) noexcept {
        for (int i = 0; i < n; ++i) add(v[i]);
    }
    std::uint64_t h_ = 1469598103934665603ull;
};

/// The values the checkpoints test compares, in a fixed order.
inline std::vector<double> checkpoint(const sim::VehicleState& s, const sim::ControlInputs& in) {
    return {s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.eulerRad[0], s.eulerRad[1], s.eulerRad[2],
            s.airspeedTrueMs, in.aileron, in.elevator, in.rudder, in.throttle[0]};
}

inline constexpr int kCheckpointEvery = 150; // world steps (5 s at 1/120 s x 4)
inline constexpr int kCheckpointSteps = 1800; // 60 s

} // namespace fsim::flights
