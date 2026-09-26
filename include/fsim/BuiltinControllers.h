#pragma once

// Built-in controllers and behaviours (design 9.3). Plain classes so trainer
// code can subclass, retune (`setParameter`) or replace them by id.

#include "fsim/Control.h"
#include "fsim/Export.h"
#include "fsim/Pid.h"

#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::control {

/// name -> member pointer table shared by the built-ins.
class FSIM_API Parameters {
public:
    void add(const char* name, double* value) { table_.emplace_back(name, value); }
    bool set(std::string_view name, double value) noexcept;
    std::optional<double> get(std::string_view name) const noexcept;

private:
    std::vector<std::pair<const char*, double*>> table_;
};

/// A gain schedule on airspeed (design 9.3). A loop's gains are as set at the
/// reference true and equivalent airspeeds `tasMs` and `easMs`; elsewhere a
/// channel's are multiplied by (easMs / eas)^a * (tasMs / tas)^b, where the
/// aircraft's response to that control grows as eas^a * tas^b - so the loop
/// stays as fast and as damped over the envelope. A surface whose moment
/// grows with dynamic pressure has a = 2 for the load factor it pulls; a
/// fly-by-wire law that commands the load factor has a = b = 0. Off while
/// `tasMs` is 0 (the default): the gains are then fixed.
struct AirspeedSchedule {
    double tasMs = 0.0; ///< reference true airspeed, m/s; 0 = no schedule
    double easMs = 0.0; ///< reference equivalent airspeed, m/s; 0 = the same as tasMs

    bool active() const noexcept { return tasMs > 0.0; }
    /// The factor for a channel whose response grows as eas^a * tas^b (the
    /// calibrated airspeed stands in for the equivalent), within 0.2 .. 5.
    double factor(const sim::VehicleState& s, double a, double b) const noexcept {
        if (!active() || (a == 0.0 && b == 0.0)) return 1.0;
        const double eas0 = easMs > 0.0 ? easMs : tasMs;
        const double tas = std::max(s.airspeedTrueMs, 0.3 * tasMs);
        const double eas = std::max(s.airspeedCalibratedMs, 0.3 * eas0);
        return std::clamp(power(eas0 / eas, a) * power(tasMs / tas, b), 0.2, 5.0);
    }
    /// x^e, by multiplication for the small whole exponents a schedule has.
    static double power(double x, double e) noexcept {
        if (e == 0.0) return 1.0;
        if (e == std::floor(e) && std::abs(e) <= 4.0) {
            double r = 1.0;
            for (int i = 0; i < static_cast<int>(std::abs(e)); ++i) r *= x;
            return e > 0.0 ? r : 1.0 / r;
        }
        return std::pow(x, e);
    }
    /// tas / tasMs (at least 0.3), or 1 without a schedule: for gains that
    /// turn an angle into a rate through the flight path, which grow with speed.
    double speedRatio(const sim::VehicleState& s) const noexcept {
        return active() ? std::max(s.airspeedTrueMs, 0.3 * tasMs) / tasMs : 1.0;
    }
    /// The elevator a surface-controlled aircraft holds at load factor `n`
    /// (+ nose down): `trim` at 1 g at the reference speed, of which
    /// `trimLift` goes with the lift - it grows as n (easMs / eas)^2.
    double trimElevator(const sim::VehicleState& s, double n, double trim, double trimLift) const noexcept {
        if (trim == 0.0 && trimLift == 0.0) return 0.0;
        const double eas0 = easMs > 0.0 ? easMs : tasMs;
        const double ratio = eas0 > 0.0 ? eas0 / std::max(s.airspeedCalibratedMs, 0.5 * eas0) : 1.0;
        return std::clamp(trim - trimLift + trimLift * n * ratio * ratio, -1.0, 1.0);
    }
};

/// Level::Actuator: identity (the stack clamps).
class FSIM_API ActuatorPassthrough final : public Controller {
public:
    const char* id() const noexcept override { return "actuator"; }
    Level level() const noexcept override { return Level::Actuator; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext&, const Command& in) override { return in; }
};

/// Level::Attitude -> Actuator: roll/pitch PID with pitch trim integrator, beta
/// feedback on the rudder, heading-to-roll outer loop, airspeed-to-throttle PI.
/// With a schedule, the roll and pitch gains follow the airspeed and the
/// heading gain grows with it (a bank turns the heading at g tan(bank) / tas).
class FSIM_API AttitudeLoop final : public Controller {
public:
    AttitudeLoop();
    const char* id() const noexcept override { return "pid_attitude"; }
    Level level() const noexcept override { return Level::Attitude; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    Pid roll{2.0, 0.0, 0.25, 0.3};
    Pid pitch{2.5, 0.4, 0.5, 0.5};
    Pid airspeed{0.06, 0.02, 0.0, 0.5, 0.0, 1.0};
    double headingGain = 1.5;      ///< rad roll per rad heading error
    double rudderBetaGain = 1.0;   ///< rudder per rad sideslip
    double throttleFeedforward = 0.55;
    double maxRollRateRadS = 1.5;  ///< roll setpoint slew
    AirspeedSchedule schedule;     ///< "schedule.tas_ms", "schedule.eas_ms"
    double rollEasExponent = 0.0, rollTasExponent = 0.0;   ///< the roll gains' schedule (AirspeedSchedule::factor)
    double pitchEasExponent = 0.0, pitchTasExponent = 0.0; ///< the pitch gains' schedule
    /// "pitch.trim", "pitch.trim_lift": the elevator that holds level flight at
    /// the reference speed and the part of it that goes with lift - fed forward
    /// (AirspeedSchedule::trimElevator, at the load factor of a level turn), so
    /// the integrator only trims what is left. 0 for a law that trims itself.
    double pitchTrim = 0.0, pitchTrimLift = 0.0;

private:
    Parameters params_;
    double rollRef_ = 0.0;
    bool haveRef_ = false;
    double lastTime_ = -1.0; ///< the vehicle's sim time at the last update: a loop that missed a period starts again
};

/// Level::Acceleration -> Actuator: load-factor error to elevator, roll rate
/// to aileron, sideslip to rudder, longitudinal acceleration to throttle.
/// Works through any attitude (no Euler angles), so it flies loops and rolls.
/// Optional feedforwards give the stick the command needs before any error
/// builds: the load factor beyond what gravity asks at this attitude
/// (cos pitch cos roll), and the roll rate; with a schedule they and the
/// gains follow the airspeed.
class FSIM_API AccelerationLoop final : public Controller {
public:
    AccelerationLoop();
    const char* id() const noexcept override { return "pid_acceleration"; }
    Level level() const noexcept override { return Level::Acceleration; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    Pid loadFactor{0.35, 0.3, 0.15, 0.6};
    Pid rollRate{0.8, 0.2, 0.0, 0.5};
    Pid longitudinal{0.15, 0.05, 0.0, 0.5, 0.0, 1.0};
    double rudderBetaGain = 1.0;
    double throttleFeedforward = 0.6;
    double loadFactorFeedforward = 0.0; ///< nose-up stick per g beyond what neutral stick gives; 0 = none
    /// What neutral stick gives: 1 = cos(pitch) cos(roll) - it holds the flight
    /// path, as fly-by-wire laws do; 0 = 1 g, as a trimmed surface does
    double loadFactorPathHold = 1.0;
    double rollRateFeedforward = 0.0;   ///< aileron per rad/s of roll rate; 0 = none
    AirspeedSchedule schedule;          ///< "schedule.tas_ms", "schedule.eas_ms"
    double loadFactorEasExponent = 0.0, loadFactorTasExponent = 0.0; ///< the load-factor gains' schedule
    double rollRateEasExponent = 0.0, rollRateTasExponent = 0.0;     ///< the roll-rate gains' schedule
    double pitchTrim = 0.0, pitchTrimLift = 0.0; ///< as AttitudeLoop's, fed forward at the load factor commanded

private:
    Parameters params_;
};

/// Level::Velocity -> Attitude: vertical speed to pitch (with trim), turn
/// rate to bank, heading and airspeed passed to the attitude loop. With a
/// reference speed its gains hold there and scale as 1 / tas elsewhere (a
/// pitch change moves the vertical speed by tas times as much); with the
/// feedforward the pitch is the flight-path angle the vertical speed needs,
/// asin(vz / tas), plus the angle of attack the wing would fly at 1 g -
/// alpha0 + (alpha - alpha0) / n, about its zero-lift angle alpha0 - so the
/// integrator only trims what is left.
class FSIM_API VelocityLoop final : public Controller {
public:
    VelocityLoop();
    const char* id() const noexcept override { return "pid_velocity"; }
    Level level() const noexcept override { return Level::Velocity; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    Pid verticalSpeed{0.045, 0.012, 0.0, 0.25, -0.4, 0.35};
    double maxBankRad = 0.785;
    double referenceSpeedMs = 0.0;      ///< "schedule.tas_ms": where the gains are as set; 0 = fixed gains
    double flightPathFeedforward = 0.0; ///< "vertical_speed.feedforward": 1 adds asin(vz / tas) + the 1 g alpha; 0 = none
    double alphaZeroLift = 0.0;         ///< "vertical_speed.alpha_zero_lift": the wing's zero-lift angle of attack, rad
    /// "vertical_speed.command_lag": the commanded vertical speed is followed
    /// through a first-order lag of this time constant (s), so a step asks for
    /// a climb the aircraft can enter without overshoot; 0 = none
    double commandLagS = 0.0;

private:
    /// The roll and the speed of the attitude command (whichever are engaged).
    Command lateralAndSpeed(const ControlContext& ctx, const VelocityCommand& c, AttitudeCommand& out) const;

    Parameters params_;
    double vzRef_ = 0.0;  ///< the lagged vertical speed command
    double alpha_ = 0.0;  ///< the 1 g angle of attack, smoothed
    bool started_ = false;
    double lastTime_ = -1.0;
};

/// Level::Position -> Velocity: bearing to the point becomes the heading,
/// altitude error becomes a bounded vertical speed.
class FSIM_API PositionLoop final : public Controller {
public:
    PositionLoop();
    const char* id() const noexcept override { return "pid_position"; }
    Level level() const noexcept override { return Level::Position; }
    bool axisAware() const noexcept override { return true; }
    Command update(const ControlContext& ctx, const Command& in) override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    double altitudeGain = 0.25;     ///< m/s vertical speed per m altitude error
    double maxVerticalSpeedMs = 6.0;

private:
    Parameters params_;
};

// --- Behaviours -------------------------------------------------------------

/// "hold": keep the altitude, heading and airspeed at the moment it started.
class FSIM_API HoldBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "hold"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;

private:
    VelocityCommand target_;
    double altitude_ = 0.0;
};

/// "waypoints": fly `points` in order; params: loop (0/1). Finished after the last capture.
class FSIM_API WaypointsBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "waypoints"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    bool finished() const noexcept override { return finished_; }
    std::size_t index() const noexcept { return index_; }

private:
    std::size_t index_ = 0;
    bool loop_ = false, finished_ = false;
    double airspeed_ = kHold;
};

/// "loiter": circle a point. params: lat_deg, lon_deg (or `target` vehicle),
/// radius_m (1500), altitude_m (current), clockwise (1), airspeed_ms (hold).
class FSIM_API LoiterBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "loiter"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;

    /// TargetLost while the vehicle it follows is gone (it flies on as it can).
    Reason failure() const noexcept override { return lost_ ? Reason::TargetLost : Reason::None; }

private:
    bool lost_ = false;
    double centreLat_ = 0.0, centreLon_ = 0.0, radius_ = 1500.0, altitude_ = 0.0, airspeed_ = kHold;
    bool clockwise_ = true;
    std::uint32_t target_ = 0;
};

/// "pursuit": chase `target` with lead pursuit. params: range_m (300),
/// lead_s (2), max_airspeed_ms, min_airspeed_ms.
class FSIM_API PursuitBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "pursuit"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;

    /// TargetLost while the vehicle it follows is gone (it flies on as it can).
    Reason failure() const noexcept override { return lost_ ? Reason::TargetLost : Reason::None; }

private:
    bool lost_ = false;
    std::uint32_t target_ = 0;
    double rangeM_ = 300.0, leadS_ = 2.0, minSpeed_ = 30.0, maxSpeed_ = 400.0;
};

/// "evade": fly away from `target`, descending or climbing. params:
/// altitude_delta_m (-300), airspeed_ms (hold).
class FSIM_API EvadeBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "evade"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;

    /// TargetLost while the vehicle it follows is gone (it flies on as it can).
    Reason failure() const noexcept override { return lost_ ? Reason::TargetLost : Reason::None; }

private:
    bool lost_ = false;
    std::uint32_t target_ = 0;
    double altitude_ = 0.0, airspeed_ = kHold;
};

/// "formation": hold a slot relative to `target` (leader). params: ahead_m,
/// right_m, below_m (in the leader's heading frame).
class FSIM_API FormationBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "formation"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;

    /// TargetLost while the vehicle it follows is gone (it flies on as it can).
    Reason failure() const noexcept override { return lost_ ? Reason::TargetLost : Reason::None; }

private:
    bool lost_ = false;
    std::uint32_t target_ = 0;
    double ahead_ = -100.0, right_ = 60.0, below_ = 0.0;
    double closureGain_ = 0.1;
};

/// "aerobatics": a manoeuvre flown open-loop through the acceleration level.
/// params: manoeuvre (0 aileron roll, 1 loop, 2 immelmann, 3 split-s),
/// load_factor_g (3.5), roll_rate_rad_s (1.5). Finished when the manoeuvre
/// completes; then holds level flight.
class FSIM_API AerobaticBehavior final : public Behavior {
public:
    enum Manoeuvre { AileronRoll = 0, Loop = 1, Immelmann = 2, SplitS = 3 };
    const char* id() const noexcept override { return "aerobatics"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    bool finished() const noexcept override { return phase_ == Done; }
    void reset() override;

private:
    enum Phase { Entry, Pull, Roll, Done } phase_ = Entry;
    Manoeuvre manoeuvre_ = Loop;
    double loadFactor_ = 3.5, rollRate_ = 1.5;
    double pitchTravel_ = 0.0, rollTravel_ = 0.0, timer_ = 0.0;
    double entryAltitude_ = 0.0, entryHeading_ = 0.0, entrySpeed_ = 0.0;
};

} // namespace fsim::control
