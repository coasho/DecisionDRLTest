#pragma once

// The Vehicle Interface's guidance modes (docs/vehicle-interface.md, ADR-28):
// behaviours that fly a mode's fixed-size setpoint - an HSA/CSA - the way the
// vehicle flies: a wing banks to turn and holds an airspeed, a rotorcraft flies
// its velocity over the ground. Plain classes, as the built-in loops are, so a
// trainer can subclass them or register its own under their ids.

#include "fsim/Control.h"
#include "fsim/Export.h"

namespace fsim::control {

class ControllerRegistry;

/// The wind as the aircraft's own air data see it: the ground velocity less
/// the air velocity (true airspeed along its angles of attack and sideslip,
/// turned to north-east-down), low-passed. Guidance never reads the
/// environment's wind, so sensor effects reach it as they would a real one.
struct FSIM_API WindEstimate {
    double northMs = 0.0, eastMs = 0.0; ///< where it blows to
    bool valid = false;
    double timeConstantS = 3.0;
    void update(const sim::VehicleState& s, double dt) noexcept;
    void reset() noexcept { northMs = eastMs = 0.0, valid = false; }
};

/// A speed in `reference` as a true airspeed here: a calibrated one by the
/// ratio of the two airspeeds the state has (the standard atmosphere's when
/// it is too slow to have one), a Mach number by the speed of sound here. NaN
/// for a ground speed, which no airspeed makes alone.
FSIM_API double trueAirspeedOf(double speed, SpeedReference reference, const sim::VehicleState& s) noexcept;
/// The aircraft's own speed now in `reference`.
FSIM_API double speedNow(SpeedReference reference, const sim::VehicleState& s) noexcept;
/// An altitude in `reference` as the height above sea level to fly now: above
/// the ground, over the terrain under the aircraft.
FSIM_API double altitudeMslOf(double altitudeM, AltitudeReference reference, const sim::VehicleState& s) noexcept;
/// The aircraft's own altitude now in `reference`.
FSIM_API double altitudeNow(AltitudeReference reference, const sim::VehicleState& s) noexcept;

/// "hsa": fsim.guidance.hsa, A-GRA's HSA/CSA (docs/vehicle-interface.md, 4.4).
/// Flies a complete HsaCommand (the host resolves what a command leaves out):
/// - a wing: the heading, or the heading that holds the course against the
///   wind estimate plus a slow trim on the course error; the airspeed its
///   reference asks, or that makes the ground speed along its track; the
///   altitude by a vertical speed at its position loop's gain and limits;
/// - a rotorcraft: a ground speed as its velocity over the ground along the
///   heading or course (the nose along the track), an airspeed along the nose
///   or, for a course, the air velocity whose track is the course; the same
///   altitude law.
/// Its output is a velocity command, flown by the vehicle's own loops.
class FSIM_API HsaBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "hsa"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    /// What it commands: the heading or course, the altitude (as the height
    /// above sea level to fly now), the speed and its reference.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    WindEstimate wind_;
    double courseTrim_ = 0.0;  ///< a wing's course error integral, rad of heading
    double speedTrim_ = 0.0;   ///< a wing's ground speed error integral, m/s of airspeed
    double lastTime_ = -1.0;   ///< the vehicle's time at the last update: a loop that missed a period starts again
    HsaCommand flown_{};       ///< the setpoint as last flown
    double altitudeMsl_ = kHold, headingFlown_ = kHold;
};

/// Registers the modes' behaviours ("hsa"); registerBuiltinControllers calls it.
void registerGuidanceModes(ControllerRegistry& registry);

} // namespace fsim::control
