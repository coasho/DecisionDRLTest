#pragma once

// Envelope protection (docs/control-architecture.md, section 11): the demand
// the control system passes on, limited to the aircraft's envelope, and the
// state checked against it. It limits what is demanded; it never promises
// where the aircraft goes. A gust, inertia, a saturated surface or a law the
// profile describes imperfectly can still take it past a limit: that is
// reported, not prevented.

#include "control/Runtime.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <array>
#include <cstdint>

namespace fsim::control {

using LimitMask = std::uint16_t;
constexpr LimitMask limitBit(Limit l) noexcept { return static_cast<LimitMask>(1u << static_cast<unsigned>(l)); }

/// The axis whose control flies against a limit (6.3): load factor, alpha
/// and pitch the pitch; bank and roll rate the roll; airspeed and Mach the thrust.
inline Axis limitAxis(Limit l) noexcept {
    constexpr Axis kAxes[kLimitCount] = {Axis::Pitch, Axis::Pitch, Axis::Pitch, Axis::Roll, Axis::Pitch,
                                         Axis::Pitch, Axis::Roll,  Axis::Thrust, Axis::Thrust, Axis::Thrust};
    return kAxes[static_cast<std::size_t>(l)];
}

/// What the limits need from the state, worked out once per update.
struct LimitState {
    const sim::VehicleState* s = nullptr;
    double tasPerCas = 1.0;  ///< a calibrated airspeed limit, as true airspeed here
    double tasPerMach = 0.0; ///< the speed of sound here (0: not known, below Mach 0.05)
    double bankCos = 1.0;    ///< cos(bank), floored at 0.1
};
/// Only what `e` asks for is worked out (a division is a division).
LimitState limitState(const sim::VehicleState& s, const EnvelopeLimits& e, const Protection& p) noexcept;

/// The protection a profile gives: its envelope's limits (the flaps
/// configuration completed from the clean one), what the aircraft's own law
/// enforces, the plant's numbers the elevator limiter scales by. Limit if the
/// profile has an envelope section, else Off (D3).
Protection protectionFor(const VehicleProfile& profile) noexcept;

/// The limits in force: the flaps configuration's with the flaps commanded
/// beyond the threshold, the gear's speed with the gear down (then written into `scratch`).
const EnvelopeLimits& activeLimits(const Protection& p, double flapsCommand, double gearPosition, EnvelopeLimits& scratch) noexcept;

/// A setpoint at its level (above the actuators) limited as the state is
/// now (11.3): each field to its limit, the pitch attitude and the load
/// factor also to what puts the wing at alpha_max here (unless the
/// aircraft's law enforces alpha). kHold fields are left alone. Returns the
/// limits that changed it.
LimitMask limitSetpoint(Command& c, const EnvelopeLimits& e, const Protection& p, const LimitState& here) noexcept;
inline LimitMask limitSetpoint(Command& c, const EnvelopeLimits& e, const Protection& p, const sim::VehicleState& s) noexcept {
    return limitSetpoint(c, e, p, limitState(s, e, p));
}

/// The elevator of a surface-controlled aircraft, moved nose-down as the
/// load factor or alpha nears its maximum and nose-up as the load factor
/// nears its minimum - never towards a limit. Returns the limits that moved it.
LimitMask limitElevator(double& elevator, const EnvelopeLimits& e, const Protection& p, const sim::VehicleState& s) noexcept;

/// The state against the limits, in flight: each limit it is beyond, and by
/// how much in `excess` (in the limit's unit; the others' entries are left alone).
LimitMask exceeded(const EnvelopeLimits& e, const sim::VehicleState& s, std::array<double, kLimitCount>& excess) noexcept;

} // namespace fsim::control
