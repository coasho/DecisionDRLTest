#pragma once

// The rotorcraft's built-in loops (docs/rotorcraft.md, 3.6): what the
// cascade's levels mean for an aircraft that hovers - body rates and its
// rotors' thrust, attitude with the heading on the yaw, a velocity over the
// ground or through the air, a point to stop at. Designed from the profile's
// hover section (control/Laws.h); plain classes, as the fixed-wing loops
// are, so a trainer can subclass, retune or replace them by id.

#include "fsim/BuiltinControllers.h"
#include "fsim/Control.h"
#include "fsim/Export.h"

namespace fsim::control {

class ControllerRegistry;

/// The load factor a rotorcraft's thrust gives: the specific force along the
/// body's -z axis, in g (1 in a steady hover). VehicleState::loadFactor is
/// the wing's lift over the weight, which a rotor's thrust is no part of.
FSIM_API double thrustLoadFactor(const sim::VehicleState& s) noexcept;

/// One axis of the rate loops: d(rate)/dt = -damping rate + power u. The
/// command is the trim plus what the demanded rate needs against the damping,
/// a proportional term to the bandwidth and an integral, over the power.
struct FSIM_API RotorRateAxis {
    double power = 1.0;     ///< "<axis>.power": body acceleration per unit command (signed: the platform's senses)
    double damping = 0.0;   ///< "<axis>.damping", 1/s
    double bandwidth = 4.0; ///< "<axis>.bandwidth", rad/s: the closed loop's, the damping included
    double integral = 1.0;  ///< "<axis>.ki", 1/s2
    double trim = 0.0;      ///< "<axis>.trim": the hover's command
    double sum = 0.0;       ///< the integral so far (an acceleration)

    /// The command for a rate `demand`, the rate now `rate`; kept within +-1 (the integral stops there).
    double update(double demand, double rate, double dt) noexcept;
    void reset() noexcept { sum = 0.0; }
};

/// Level::Acceleration -> Actuator: the rates to the cyclic (or the mixer's
/// roll and pitch) and the pedals (its yaw), each on its identified
/// response; the load factor to the collective (every rotor's thrust), fed
/// forward from the heave's response and trimmed on the load factor flown.
/// A throttle given instead of the load factor goes straight through.
class FSIM_API RotorAllocation final : public Controller {
public:
    RotorAllocation();
    const char* id() const noexcept override { return "rotor_allocation"; }
    Level level() const noexcept override { return Level::Acceleration; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    RotorRateAxis roll{1.0, 0.0, 4.0, 2.0, 0.0}, pitch{-1.0, 0.0, 4.0, 2.0, 0.0}, yaw{-1.0, 0.0, 3.0, 1.0, 0.0};
    double throttleTrim = 0.5;    ///< "throttle.trim": the hover's
    double heavePower = 10.0;     ///< "heave.power": vertical acceleration per unit throttle, m/s2
    double loadFactorGain = 2.0;  ///< "load_factor.ki": 1/s, the integral that trims the load factor flown

private:
    Parameters params_;
    double loadSum_ = 0.0;   ///< throttle the load factor's integral holds
    double lastTime_ = -1.0;
};

/// Level::Attitude -> Acceleration: the attitude's errors to the rates of the
/// Euler angles ("<axis>.gain" rad/s per rad, within "<axis>.max_rate"), and
/// those to body rates. The heading is flown with the yaw, whatever the
/// roll; without one, the heading it had when it took over. The pitch, held,
/// is kept where it was; the roll levels. The throttle goes through.
class FSIM_API RotorAttitude final : public Controller {
public:
    RotorAttitude();
    const char* id() const noexcept override { return "rotor_attitude"; }
    Level level() const noexcept override { return Level::Attitude; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    double rollGain = 2.0, pitchGain = 2.0, yawGain = 1.0;          ///< rad/s per rad
    double maxRollRateRadS = 1.0, maxPitchRateRadS = 1.0, maxYawRateRadS = 0.5;

private:
    Parameters params_;
    double headingRef_ = 0.0, pitchRef_ = 0.0;
    bool haveRef_ = false;
    double lastTime_ = -1.0;
};

/// Level::Velocity -> Attitude: a velocity over the ground (north, east) or
/// an airspeed along the heading - without either, the velocity over the
/// ground it had when it took over - to the tilt that accelerates it there
/// ("horizontal.kp", "horizontal.ki", within "max_tilt"); the vertical speed
/// to the throttle on the heave's response, with what the tilt takes from
/// the thrust fed forward; the heading, or a turn rate integrated into one.
class FSIM_API RotorVelocity final : public Controller {
public:
    RotorVelocity();
    const char* id() const noexcept override { return "rotor_velocity"; }
    Level level() const noexcept override { return Level::Velocity; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    double horizontalKp = 0.5;    ///< 1/s: m/s2 of acceleration per m/s of velocity error
    double horizontalKi = 0.06;   ///< 1/s2
    double maxTiltRad = 0.44;     ///< the most it tilts to accelerate
    double verticalKp = 1.5;      ///< 1/s: m/s2 per m/s of vertical speed error
    double verticalKi = 0.5;      ///< 1/s2
    double throttleTrim = 0.5;    ///< the hover's
    double heavePower = 10.0;     ///< m/s2 per unit throttle
    double heaveDamping = 0.3;    ///< 1/s
    /// "roll.trim", "pitch.trim": the attitude it hovers at, rad (a
    /// helicopter's rotor is tilted on its fuselage: it hovers nose up and
    /// leaning), which the tilt it accelerates with is measured from
    double rollTrimRad = 0.0, pitchTrimRad = 0.0;

private:
    Parameters params_;
    double northHold_ = 0.0, eastHold_ = 0.0; ///< the velocity over the ground held without a command
    double sumX_ = 0.0, sumY_ = 0.0, sumZ_ = 0.0;
    double headingRef_ = 0.0;
    bool started_ = false;
    double lastTime_ = -1.0;
};

/// Level::Position -> Velocity: the point to a velocity over the ground that
/// brings the aircraft to a stop there (the least of "horizontal.gain" times
/// the distance, what it can stop from - "velocity.lag_s" of the velocity
/// loop's lag, then "deceleration" - and the airspeed given or "max_speed");
/// the altitude to a vertical speed; the heading given, or the point while it
/// is further than the capture radius (and 20 m).
class FSIM_API RotorPosition final : public Controller {
public:
    RotorPosition();
    const char* id() const noexcept override { return "rotor_position"; }
    Level level() const noexcept override { return Level::Position; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    double horizontalGain = 0.3;     ///< 1/s: m/s per m to go
    double maxSpeedMs = 10.0;
    double decelerationMs2 = 2.0;
    double velocityLagS = 1.0;       ///< "velocity.lag_s": how far behind its command the velocity loop is
    double altitudeGain = 0.5;       ///< 1/s
    double maxVerticalSpeedMs = 3.0;

private:
    Parameters params_;
};

/// "hover": hold the point, height and heading it had when it started (or
/// params lat_deg, lon_deg, altitude_m, heading_deg). For an aircraft that
/// can hover (kFeatureHover).
class FSIM_API HoverBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "hover"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    /// Its point, height and heading, and how far it is from the point.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    PositionCommand target_;
    double lat_ = 0.0, lon_ = 0.0; ///< as of the last update
};

/// Registers the loops above and "hover" (registerBuiltinControllers calls it).
void registerRotorControllers(ControllerRegistry& registry);

} // namespace fsim::control
