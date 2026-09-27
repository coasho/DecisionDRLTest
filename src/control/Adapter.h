#pragma once

// What is specific to an aircraft family (docs/control-architecture.md,
// sections 2 and 4; docs/rotorcraft.md, 3.2): a descriptive face for the
// contract layer (what the profile implies, which capabilities and ranges the
// aircraft offers, what it can do) and a real-time face for the runtime
// (demand -> the flight model's inputs, and which command fields fly which
// axis). Adapters hold no state; one instance serves every vehicle of a family.

#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <limits>

namespace fsim::control {

class CapabilityCatalog;

/// What a support effector allows now (docs/flight-autonomy.md, 4.6): the
/// reason it may not be commanded at all, else the range its value may take.
struct Placard {
    Reason reason = Reason::None;
    const char* description = ""; ///< the placard in words
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();
};

/// Which primary axis, let go, has the vehicle default's hold
/// (VehicleDefault::Hold) keep each target: a wing keeps its heading with
/// the roll, its height with the pitch, its airspeed with the thrust; a
/// rotorcraft its heading with the yaw, its height with the thrust, its
/// velocity over the ground with the cyclic. Axis::Count: never kept.
struct HoldAxes {
    Axis heading = Axis::Roll;
    Axis altitude = Axis::Pitch;
    Axis airspeed = Axis::Thrust;
    Axis groundVelocity = Axis::Count;
};

class VehicleAdapter {
public:
    virtual ~VehicleAdapter() = default;

    /// "jsbsim.stock", "jsbsim.direct", "jsbsim.fbw", "jsbsim.helicopter", "jsbsim.multirotor".
    virtual const char* family() const noexcept = 0;

    // --- What the family's aircraft are ------------------------------------------
    /// What they can do (Feature bits): the behaviours offered them, what protection limits.
    virtual std::uint32_t features() const noexcept { return kFeatureWingborne; }
    /// The groups their primary axes are owned apart in above the actuators
    /// (AxisGroup bits): a wing's roll and yaw together, pitch, thrust.
    virtual std::uint8_t axisGroups() const noexcept { return kGroupLateral | kGroupPitch | kGroupThrust; }
    /// What the vehicle default's hold keeps as each axis is let go.
    virtual HoldAxes holdAxes() const noexcept { return {}; }

    // --- Descriptive face: the contract layer, between steps -----------------------
    /// Fill in what the family implies and the aircraft did not say (a
    /// fly-by-wire stick commands load factor and roll rate), as Derived.
    virtual void complete(VehicleProfile& profile) const;
    /// Offer the support effectors the profile has, and narrow the catalog's
    /// flight parameters to its envelope.
    virtual void declare(const VehicleProfile& profile, CapabilityCatalog& catalog) const;
    /// A support effector's placard now, by SupportCommand's alternative: no
    /// gear operated above its speed, the gear down on the ground, no flaps out
    /// past their threshold above the flap speed.
    virtual Placard placard(std::size_t alternative, const sim::VehicleState& state, const VehicleProfile& profile) const noexcept;
    /// Whether a support command may be flown now, by its placard: the
    /// placard's reason, Unavailable for a value outside its range, else None.
    /// A capability's status reports the same placard (CapabilityHost::status).
    Reason admit(const SupportCommand& command, const sim::VehicleState& state, const VehicleProfile& profile) const noexcept;
    /// What the aircraft can do, as its guidance plans with it
    /// (docs/vehicle-interface.md, 7.1): from the profile's sections and the
    /// loops `runtime` flies with (their limits and gains).
    virtual Performance performance(const VehicleProfile& profile, const ControlStack& runtime) const;

    // --- Real-time face: the runtime, every control update ---------------------------
    /// The final actuator demand into the flight model's inputs. `last` is
    /// what the previous update wrote (held channels keep it).
    virtual void apply(const ActuatorCommand& demand, const sim::ControlInputs& last, sim::ControlInputs& out) const noexcept;
    /// Which command fields fly which axis: `axis`'s fields of `src` into
    /// `dst`, two commands of the same level above the actuators - the
    /// runtime's merge of the demands of axes owned apart (9.5).
    virtual void copyAxisFields(Command& dst, const Command& src, Axis axis) const noexcept;

protected:
    /// The support effectors the profile has; with `engines`, a throttle per
    /// engine where there are several (fsim.flight.engines).
    static void declareSupport(const VehicleProfile& profile, CapabilityCatalog& catalog, bool engines);
    /// Envelope protection and the flight parameters narrowed to the envelope, where the profile has one.
    static void declareEnvelope(const VehicleProfile& profile, CapabilityCatalog& catalog);
};

/// The adapter for a family (stateless, lives for the program).
const VehicleAdapter& adapterFor(ControlFamily family) noexcept;

} // namespace fsim::control
