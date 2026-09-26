#pragma once

// The built-in loops' settings from an aircraft's identified plant
// (docs/control-architecture.md, section 14, step 5): the closed-form design
// hangar's autopilot stage used to run offline, now the platform's. An
// aircraft carries what was measured on it - its responses to each control
// at a reference condition - rather than fifty numbers derived from them.

#include "fsim/ControlStack.h"
#include "fsim/VehicleProfile.h"

#include <vector>

namespace fsim::control {

/// Whether the profile's plant section gives what the design needs: the
/// reference condition and the roll, pitch and speed responses (the yaw
/// response is optional: without it the rudder does not chase sideslip).
bool canDesignLaws(const VehicleProfile& profile) noexcept;

/// The settings of pid_attitude, pid_acceleration, pid_velocity and
/// pid_position for this plant: each loop's poles placed from the response
/// it flies on, the outer loops a fraction of the inner ones' speed, the
/// gains scheduled on the airspeed from the reference condition. Empty if
/// the plant does not give enough.
std::vector<ControllerSetting> designLaws(const VehicleProfile& profile);

/// A profile without gains of its own (no control section) gets the laws
/// designed from its plant, as a control section with provenance Derived.
/// Returns whether it did. Gains an aircraft or a trainer gives win.
bool completeControl(VehicleProfile& profile);

} // namespace fsim::control
