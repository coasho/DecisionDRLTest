#include "control/Adapter.h"

#include "control/Catalog.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

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

} // namespace

void VehicleAdapter::complete(VehicleProfile&) const {}

void VehicleAdapter::declare(const VehicleProfile& p, CapabilityCatalog& catalog) const {
    // the support effectors it has (without an effectors section: those an actuator command has always set)
    const EffectorsSection& fx = p.effectors;
    if (fx.retractableGear) catalog.addSupport(0);
    if (fx.flaps) catalog.addSupport(1);
    if (fx.wheelBrakes) catalog.addSupport(2);
    if (fx.speedbrake) catalog.addSupport(3);
    if (fx.pitchTrim) catalog.addSupport(4);
    if (!p.envelope.header.present()) return;
    // A command may ask for what the clean configuration allows; with flaps
    // out, protection narrows further as the aircraft flies (step 4).
    const EnvelopeLimits& e = p.envelope.clean;
    catalog.narrow("fsim.flight.attitude", "roll_rad", -e.bankMaxRad, e.bankMaxRad);
    catalog.narrow("fsim.flight.attitude", "max_bank_rad", 0.0, e.bankMaxRad);
    catalog.narrow("fsim.flight.attitude", "pitch_rad", e.pitchMinRad, e.pitchMaxRad);
    catalog.narrow("fsim.flight.acceleration", "load_factor_g", e.loadFactorMin, e.loadFactorMax);
    catalog.narrow("fsim.flight.acceleration", "roll_rate_rad_s", -e.rollRateMaxRadS, e.rollRateMaxRadS);
}

Reason VehicleAdapter::admit(const SupportCommand& c, const sim::VehicleState& s, const VehicleProfile& p) const noexcept {
    const double cas = s.airspeedCalibratedMs;
    if (const auto* g = std::get_if<GearCommand>(&c)) {
        if (g->down < 0.5 && s.onGround) return Reason::Unavailable;
        if (!std::isnan(p.envelope.gearCasMaxMs) && cas > p.envelope.gearCasMaxMs) return Reason::Unavailable;
    } else if (const auto* f = std::get_if<FlapsCommand>(&c)) {
        if (f->position > p.envelope.flapsThreshold && !std::isnan(p.envelope.flaps.casMaxMs) && cas > p.envelope.flaps.casMaxMs)
            return Reason::Unavailable;
    }
    return Reason::None;
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

const VehicleAdapter& adapterFor(ControlFamily family) noexcept {
    static const JsbsimStock stock;
    static const JsbsimDirect direct;
    static const JsbsimFbw fbw;
    switch (family) {
    case ControlFamily::Direct: return direct;
    case ControlFamily::FlyByWire: return fbw;
    default: return stock;
    }
}

} // namespace fsim::control
