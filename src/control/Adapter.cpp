#include "control/Adapter.h"

#include "control/Catalog.h"
#include "fsim/ControlStack.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

/// A parameter of the loop `runtime` flies `level` with; NaN if none.
double loopParameter(const ControlStack& runtime, Level level, std::string_view name) noexcept {
    const Controller* c = runtime.controller(level);
    if (!c) return kUnknown;
    const auto v = c->parameter(name);
    return v ? *v : kUnknown;
}

double known(double v, double fallback) noexcept { return std::isnan(v) ? fallback : v; }

/// A stock JSBSim aircraft: its own flight control system, whose inputs'
/// meaning the platform does not know; an actuator command is what it was.
class JsbsimStock final : public VehicleAdapter {
public:
    const char* family() const noexcept override { return "jsbsim.stock"; }
};

/// A surface-controlled aircraft (hangar's direct designs): the inputs move surfaces.
class JsbsimDirect final : public VehicleAdapter {
public:
    const char* family() const noexcept override { return "jsbsim.direct"; }
};

/// A fly-by-wire aircraft (hangar's fbw designs): the stick commands load
/// factor and roll rate, and centred it holds the flight path.
class JsbsimFbw final : public VehicleAdapter {
public:
    const char* family() const noexcept override { return "jsbsim.fbw"; }
    void complete(VehicleProfile& p) const override {
        if (p.effectors.header.present()) return;
        p.effectors.pitch = PitchControl::LoadFactor;
        p.effectors.roll = RollControl::RollRate;
        p.effectors.neutral = NeutralStick::PathHold;
        p.effectors.header = {EffectorsSection::kVersion, Provenance::Derived};
    }
};

/// What helicopters and multirotors share (docs/rotorcraft.md, 3.2 and 3.3):
/// they hover; the cyclic's roll and pitch are owned together (they tilt the
/// thrust that moves the aircraft), the yaw and the thrust each alone; and
/// their loops fly the heading with the yaw, the height with the thrust.
class JsbsimRotorcraft : public VehicleAdapter {
public:
    std::uint32_t features() const noexcept override { return kFeatureHover; }
    std::uint8_t axisGroups() const noexcept override { return kGroupCyclic | kGroupYaw | kGroupThrust; }
    HoldAxes holdAxes() const noexcept override { return {Axis::Yaw, Axis::Thrust, Axis::Count, Axis::Roll}; }
    void copyAxisFields(Command& dst, const Command& src, Axis axis) const noexcept override;
    /// A rotorcraft's: its tilt, its velocity loop's bandwidth, its position
    /// loop's speed, deceleration and vertical speed.
    Performance performance(const VehicleProfile& p, const ControlStack& runtime) const override;

protected:
    /// The flight capabilities as a rotorcraft flies them: the actuator's
    /// parameters by its own controls' names (aileron, elevator, rudder,
    /// throttle in that order), what it has nothing for, its axis groups.
    void declareFlight(const VehicleProfile& p, CapabilityCatalog& catalog, const char* const names[4]) const;
};

/// A helicopter (hangar's rotorcraft designs): the aileron and elevator
/// inputs are the cyclic, the rudder the pedals, the throttle the collective;
/// its engines are governed, so it has no throttles to offer.
class JsbsimHelicopter final : public JsbsimRotorcraft {
public:
    const char* family() const noexcept override { return "jsbsim.helicopter"; }
    void complete(VehicleProfile& p) const override {
        if (p.effectors.header.present()) return;
        p.effectors.pitch = PitchControl::Cyclic;
        p.effectors.roll = RollControl::Cyclic;
        p.effectors.yaw = YawControl::TailRotor;
        p.effectors.thrust = ThrustControl::Collective;
        p.effectors.flaps = p.effectors.retractableGear = p.effectors.wheelBrakes = false;
        p.effectors.header = {EffectorsSection::kVersion, Provenance::Derived};
    }
    void declare(const VehicleProfile& p, CapabilityCatalog& catalog) const override {
        static const char* const names[] = {"lateral_cyclic", "longitudinal_cyclic", "pedals", "collective"};
        declareSupport(p, catalog, false);
        declareFlight(p, catalog, names);
        declareEnvelope(p, catalog);
    }
};

/// A multirotor (hangar's rotorcraft designs): a mixer shares the roll,
/// pitch and yaw inputs among the rotors, the throttle is every rotor's
/// thrust. Its rotors' own thrusts (fsim.flight.engines) fly every axis.
class JsbsimMultirotor final : public JsbsimRotorcraft {
public:
    const char* family() const noexcept override { return "jsbsim.multirotor"; }
    void complete(VehicleProfile& p) const override {
        if (p.effectors.header.present()) return;
        p.effectors.pitch = PitchControl::Mixer;
        p.effectors.roll = RollControl::Mixer;
        p.effectors.yaw = YawControl::Mixer;
        p.effectors.thrust = ThrustControl::RotorThrust;
        p.effectors.flaps = p.effectors.retractableGear = p.effectors.wheelBrakes = false;
        p.effectors.header = {EffectorsSection::kVersion, Provenance::Derived};
    }
    void declare(const VehicleProfile& p, CapabilityCatalog& catalog) const override {
        static const char* const names[] = {"roll", "pitch", "yaw", "thrust"};
        static const char* const throttles[] = {"throttle_1", "throttle_2", "throttle_3", "throttle_4"};
        static const char* const rotors[] = {"rotor_1", "rotor_2", "rotor_3", "rotor_4"};
        declareSupport(p, catalog, false);
        if (p.propulsion.engines > 1) {
            catalog.addSupport(5, p.propulsion.engines, kPrimaryAxes); // each rotor's thrust: every axis, all or nothing
            for (int i = 0; i < std::min(p.propulsion.engines, 4); ++i) catalog.rename("fsim.flight.engines", throttles[i], rotors[i]);
        }
        declareFlight(p, catalog, names);
        declareEnvelope(p, catalog);
    }
};

void JsbsimRotorcraft::declareFlight(const VehicleProfile& p, CapabilityCatalog& c, const char* const names[4]) const {
    static const char* const inputs[] = {"aileron", "elevator", "rudder", "throttle"};
    for (int i = 0; i < 4; ++i) c.rename("fsim.flight.actuator", inputs[i], names[i]);
    const EffectorsSection& fx = p.effectors;
    if (!fx.flaps) c.unsupport("fsim.flight.actuator", "flaps");
    if (!fx.retractableGear) c.unsupport("fsim.flight.actuator", "gear_down");
    if (!fx.wheelBrakes) c.unsupport("fsim.flight.actuator", "brake_left"), c.unsupport("fsim.flight.actuator", "brake_right");
    c.unsupport("fsim.flight.attitude", "max_bank_rad");         // it turns with its yaw, not a bank
    c.unsupport("fsim.flight.attitude", "airspeed_ms");          // its speed comes from its attitude, not its thrust
    c.unsupport("fsim.flight.acceleration", "longitudinal_ms2"); // likewise
    c.setAxisGroups(axisGroups());
}

Performance JsbsimRotorcraft::performance(const VehicleProfile& p, const ControlStack& runtime) const {
    Performance f;
    f.hovers = true;
    const EnvelopeLimits& e = p.envelope.clean;
    f.maxCasMs = e.casMaxMs;
    f.maxBankRad = e.bankMaxRad, f.minPitchRad = e.pitchMinRad, f.maxPitchRad = e.pitchMaxRad, f.maxRollRateRadS = e.rollRateMaxRadS;
    f.minLoadFactor = e.loadFactorMin, f.maxLoadFactor = e.loadFactorMax;
    f.maxTiltRad = loopParameter(runtime, Level::Velocity, "max_tilt");
    if (!std::isnan(f.maxTiltRad)) f.maxAccelerationMs2 = kG * std::tan(f.maxTiltRad);
    f.velocityBandwidthRadS = loopParameter(runtime, Level::Velocity, "horizontal.kp");
    f.maxGroundSpeedMs = loopParameter(runtime, Level::Position, "max_speed");
    f.maxDecelerationMs2 = loopParameter(runtime, Level::Position, "deceleration");
    f.maxClimbMs = f.maxDescentMs = loopParameter(runtime, Level::Position, "max_vertical_speed");
    f.altitudeGainPerS = loopParameter(runtime, Level::Position, "altitude.gain");
    if (!std::isnan(f.maxGroundSpeedMs)) f.cruiseTasMs = 0.5 * f.maxGroundSpeedMs;
    return f;
}

void JsbsimRotorcraft::copyAxisFields(Command& dst, const Command& src, Axis axis) const noexcept {
    // the cyclic's roll and pitch share the fields that move the aircraft over the ground
    const bool roll = axis == Axis::Roll, pitch = axis == Axis::Pitch, cyclic = roll || pitch;
    const bool yaw = axis == Axis::Yaw, thrust = axis == Axis::Thrust;
    if (auto* d = std::get_if<AttitudeCommand>(&dst)) {
        const auto* s = std::get_if<AttitudeCommand>(&src);
        if (!s) return;
        if (roll) d->rollRad = s->rollRad, d->maxBankRad = s->maxBankRad;
        if (pitch) d->pitchRad = s->pitchRad;
        if (yaw) d->headingRad = s->headingRad;
        if (thrust) d->throttle = s->throttle, d->airspeedMs = s->airspeedMs;
    } else if (auto* d2 = std::get_if<AccelerationCommand>(&dst)) {
        const auto* s = std::get_if<AccelerationCommand>(&src);
        if (!s) return;
        if (roll) d2->rollRateRadS = s->rollRateRadS;
        if (pitch) d2->pitchRateRadS = s->pitchRateRadS;
        if (yaw) d2->yawRateRadS = s->yawRateRadS;
        if (thrust) d2->loadFactorG = s->loadFactorG, d2->throttle = s->throttle, d2->longitudinalMs2 = s->longitudinalMs2;
    } else if (auto* d3 = std::get_if<VelocityCommand>(&dst)) {
        const auto* s = std::get_if<VelocityCommand>(&src);
        if (!s) return;
        if (cyclic) d3->airspeedMs = s->airspeedMs, d3->northMs = s->northMs, d3->eastMs = s->eastMs;
        if (yaw) d3->headingRad = s->headingRad, d3->turnRateRadS = s->turnRateRadS;
        if (thrust) d3->verticalSpeedMs = s->verticalSpeedMs;
    } else if (auto* d4 = std::get_if<PositionCommand>(&dst)) {
        const auto* s = std::get_if<PositionCommand>(&src);
        if (!s) return;
        if (cyclic)
            d4->latitudeRad = s->latitudeRad, d4->longitudeRad = s->longitudeRad, d4->captureRadiusM = s->captureRadiusM,
            d4->airspeedMs = s->airspeedMs;
        if (yaw) d4->headingRad = s->headingRad;
        if (thrust) d4->altitudeMslM = s->altitudeMslM;
    }
}

} // namespace

void VehicleAdapter::complete(VehicleProfile&) const {}

void VehicleAdapter::declareSupport(const VehicleProfile& p, CapabilityCatalog& catalog, bool engines) {
    // the support effectors it has (without an effectors section: those an actuator command has always set)
    const EffectorsSection& fx = p.effectors;
    if (fx.retractableGear) catalog.addSupport(0);
    if (fx.flaps) catalog.addSupport(1);
    if (fx.wheelBrakes) catalog.addSupport(2);
    if (fx.speedbrake) catalog.addSupport(3);
    if (fx.pitchTrim) catalog.addSupport(4);
    if (engines && p.propulsion.engines > 1) catalog.addSupport(5, p.propulsion.engines); // a throttle per engine (fsim.flight.engines)
}

void VehicleAdapter::declareEnvelope(const VehicleProfile& p, CapabilityCatalog& catalog) {
    if (!p.envelope.header.present()) return;
    catalog.addProtection();
    // A command may ask for what the clean configuration allows; with flaps
    // out, protection narrows further as the aircraft flies.
    const EnvelopeLimits& e = p.envelope.clean;
    catalog.narrow("fsim.flight.attitude", "roll_rad", -e.bankMaxRad, e.bankMaxRad);
    catalog.narrow("fsim.flight.attitude", "max_bank_rad", 0.0, e.bankMaxRad);
    catalog.narrow("fsim.flight.attitude", "pitch_rad", e.pitchMinRad, e.pitchMaxRad);
    catalog.narrow("fsim.flight.acceleration", "load_factor_g", e.loadFactorMin, e.loadFactorMax);
    catalog.narrow("fsim.flight.acceleration", "roll_rate_rad_s", -e.rollRateMaxRadS, e.rollRateMaxRadS);
}

void VehicleAdapter::declare(const VehicleProfile& p, CapabilityCatalog& catalog) const {
    declareSupport(p, catalog, true);
    // what a wing has nothing of its own for: its pitch and yaw follow the
    // load factor and the bank, it flies through the air, not over the ground
    catalog.unsupport("fsim.flight.acceleration", "pitch_rate_rad_s");
    catalog.unsupport("fsim.flight.acceleration", "yaw_rate_rad_s");
    catalog.unsupport("fsim.flight.velocity", "north_ms");
    catalog.unsupport("fsim.flight.velocity", "east_ms");
    catalog.unsupport("fsim.flight.position", "heading_rad");
    catalog.narrow("fsim.guidance.loiter", "radius_m", 100.0, kUnknown); // a wing circles no tighter (the behaviour's own floor)
    declareEnvelope(p, catalog);
}

Placard VehicleAdapter::placard(std::size_t alternative, const sim::VehicleState& s, const VehicleProfile& p) const noexcept {
    Placard out;
    const double cas = s.airspeedCalibratedMs;
    if (alternative == 0) { // the gear
        if (!std::isnan(p.envelope.gearCasMaxMs) && cas > p.envelope.gearCasMaxMs) {
            out.reason = Reason::Unavailable;
            out.description = "above the gear's operating speed";
        } else if (s.onGround) {
            out.min = 0.5; // down (a value of 0.5 or more)
            out.description = "on the ground the gear stays down";
        }
    } else if (alternative == 1) { // the flaps
        if (!std::isnan(p.envelope.flaps.casMaxMs) && cas > p.envelope.flaps.casMaxMs) {
            out.max = p.envelope.flapsThreshold;
            out.description = "above the flap speed the flaps stay in";
        }
    }
    return out;
}

Reason VehicleAdapter::admit(const SupportCommand& c, const sim::VehicleState& s, const VehicleProfile& p) const noexcept {
    const Placard placard = this->placard(c.index(), s, p);
    if (placard.reason != Reason::None) return placard.reason;
    double value = kHold, value2 = kHold;
    supportValues(c, value, value2);
    if (!isHold(value) && (value < placard.min || value > placard.max)) return Reason::Unavailable;
    return Reason::None;
}

Performance VehicleAdapter::performance(const VehicleProfile& p, const ControlStack& runtime) const {
    // a wing's: its envelope, its performance section, its reference speed and its loops' limits and gains
    Performance f;
    const EnvelopeLimits& e = p.envelope.clean;
    const PerformanceSection& perf = p.performance;
    f.minCasMs = known(e.casMinMs, 1.2 * perf.stallCasMs); // NaN where neither is given
    f.maxCasMs = e.casMaxMs, f.maxMach = e.machMax;
    f.maxTasMs = perf.maxTasMs;
    f.cruiseTasMs = p.plant.tasMs;
    f.ceilingM = perf.ceilingM;
    f.maxBankRad = known(e.bankMaxRad, loopParameter(runtime, Level::Velocity, "max_bank"));
    f.minPitchRad = e.pitchMinRad, f.maxPitchRad = e.pitchMaxRad, f.maxRollRateRadS = e.rollRateMaxRadS;
    f.minLoadFactor = e.loadFactorMin, f.maxLoadFactor = e.loadFactorMax;
    f.maxClimbMs = f.maxDescentMs = loopParameter(runtime, Level::Position, "max_vertical_speed");
    if (!std::isnan(perf.climbMs) && !(f.maxClimbMs < perf.climbMs)) f.maxClimbMs = perf.climbMs; // no faster than it climbs
    f.altitudeGainPerS = loopParameter(runtime, Level::Position, "altitude.gain");
    f.headingGain = loopParameter(runtime, Level::Attitude, "heading.gain");
    f.bankRateRadS = loopParameter(runtime, Level::Attitude, "roll.max_rate");
    const double reference = loopParameter(runtime, Level::Attitude, "schedule.tas_ms");
    f.headingReferenceTasMs = reference > 0.0 ? reference : kUnknown; // a schedule holds the heading loop's speed; without one it slows with speed
    return f;
}

void VehicleAdapter::apply(const ActuatorCommand& a, const sim::ControlInputs& last, sim::ControlInputs& out) const noexcept {
    auto clamp01 = [](double v) { return std::clamp(v, 0.0, 1.0); };
    auto clamp11 = [](double v) { return std::clamp(v, -1.0, 1.0); };
    out.aileron = clamp11(orHold(a.aileron, 0.0));
    out.elevator = clamp11(orHold(a.elevator, 0.0));
    out.rudder = clamp11(orHold(a.rudder, 0.0));
    out.setThrottleAll(clamp01(orHold(a.throttle, last.throttle[0])));
    out.flaps = clamp01(orHold(a.flaps, 0.0));
    out.gearDown = isHold(a.gearDown) ? last.gearDown : (a.gearDown >= 0.5 ? 1.0 : 0.0);
    out.brakeLeft = clamp01(orHold(a.brakeLeft, 0.0));
    out.brakeRight = clamp01(orHold(a.brakeRight, 0.0));
}

void VehicleAdapter::copyAxisFields(Command& dst, const Command& src, Axis axis) const noexcept {
    // Above the actuators a wing's yaw goes with its roll (the loop that
    // banks also coordinates), so yaw's fields are roll's.
    const bool lateral = axis == Axis::Roll, pitch = axis == Axis::Pitch, thrust = axis == Axis::Thrust;
    if (auto* d = std::get_if<AttitudeCommand>(&dst)) {
        const auto* s = std::get_if<AttitudeCommand>(&src);
        if (!s) return;
        if (lateral) d->rollRad = s->rollRad, d->headingRad = s->headingRad, d->maxBankRad = s->maxBankRad;
        if (pitch) d->pitchRad = s->pitchRad;
        if (thrust) d->throttle = s->throttle, d->airspeedMs = s->airspeedMs;
    } else if (auto* d2 = std::get_if<AccelerationCommand>(&dst)) {
        const auto* s = std::get_if<AccelerationCommand>(&src);
        if (!s) return;
        if (lateral) d2->rollRateRadS = s->rollRateRadS;
        if (pitch) d2->loadFactorG = s->loadFactorG;
        if (thrust) d2->longitudinalMs2 = s->longitudinalMs2, d2->throttle = s->throttle;
    } else if (auto* d3 = std::get_if<VelocityCommand>(&dst)) {
        const auto* s = std::get_if<VelocityCommand>(&src);
        if (!s) return;
        if (lateral) d3->headingRad = s->headingRad, d3->turnRateRadS = s->turnRateRadS;
        if (pitch) d3->verticalSpeedMs = s->verticalSpeedMs;
        if (thrust) d3->airspeedMs = s->airspeedMs;
    } else if (auto* d4 = std::get_if<PositionCommand>(&dst)) {
        const auto* s = std::get_if<PositionCommand>(&src);
        if (!s) return;
        if (lateral) d4->latitudeRad = s->latitudeRad, d4->longitudeRad = s->longitudeRad, d4->captureRadiusM = s->captureRadiusM;
        if (pitch) d4->altitudeMslM = s->altitudeMslM;
        if (thrust) d4->airspeedMs = s->airspeedMs;
    }
}

const VehicleAdapter& adapterFor(ControlFamily family) noexcept {
    static const JsbsimStock stock;
    static const JsbsimDirect direct;
    static const JsbsimFbw fbw;
    static const JsbsimHelicopter helicopter;
    static const JsbsimMultirotor multirotor;
    switch (family) {
    case ControlFamily::Direct: return direct;
    case ControlFamily::FlyByWire: return fbw;
    case ControlFamily::Helicopter: return helicopter;
    case ControlFamily::Multirotor: return multirotor;
    default: return stock;
    }
}

} // namespace fsim::control
