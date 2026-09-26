#pragma once

// What is specific to an aircraft family (docs/control-architecture.md,
// sections 2 and 4): a descriptive face for the contract layer (what the
// profile implies, which capabilities and ranges the aircraft offers) and a
// real-time face for the runtime (demand -> the flight model's inputs).
// Adapters hold no state; one instance serves every vehicle of a family.

#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

namespace fsim::control {

class CapabilityCatalog;

class VehicleAdapter {
public:
    virtual ~VehicleAdapter() = default;

    /// "jsbsim.stock", "jsbsim.direct", "jsbsim.fbw".
    virtual const char* family() const noexcept = 0;

    // --- Descriptive face: the contract layer, between steps -----------------------
    /// Fill in what the family implies and the aircraft did not say (a
    /// fly-by-wire stick commands load factor and roll rate), as Derived.
    virtual void complete(VehicleProfile& profile) const;
    /// Offer the support effectors the profile has, and narrow the catalog's
    /// flight parameters to its envelope.
    virtual void declare(const VehicleProfile& profile, CapabilityCatalog& catalog) const;
    /// Whether a support command may be flown now: the placards (no gear up on
    /// the ground, no gear or flaps out above their speeds). None if it may.
    virtual Reason admit(const SupportCommand& command, const sim::VehicleState& state, const VehicleProfile& profile) const noexcept;

    // --- Real-time face: the runtime, every control update ---------------------------
    /// The final actuator demand into the flight model's inputs. `last` is
    /// what the previous update wrote (held channels keep it).
    virtual void apply(const ActuatorCommand& demand, const sim::ControlInputs& last, sim::ControlInputs& out) const noexcept;
};

/// The adapter for a family (stateless, lives for the program).
const VehicleAdapter& adapterFor(ControlFamily family) noexcept;

} // namespace fsim::control
