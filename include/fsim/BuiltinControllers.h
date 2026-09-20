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

/// Level::Actuator: identity (the stack clamps).
class FSIM_API ActuatorPassthrough final : public Controller {
public:
    const char* id() const noexcept override { return "actuator"; }
    Level level() const noexcept override { return Level::Actuator; }
    Command update(const ControlContext&, const Command& in) override { return in; }
};

/// Level::Attitude -> Actuator: roll/pitch PID with pitch trim integrator, beta
/// feedback on the rudder, heading-to-roll outer loop, airspeed-to-throttle PI.
class FSIM_API AttitudeLoop final : public Controller {
public:
    AttitudeLoop();
    const char* id() const noexcept override { return "pid_attitude"; }
    Level level() const noexcept override { return Level::Attitude; }
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

private:
    Parameters params_;
    double rollRef_ = 0.0;
    bool haveRef_ = false;
};

/// Level::Acceleration -> Actuator: load-factor error to elevator, roll rate
/// to aileron, sideslip to rudder, longitudinal acceleration to throttle.
/// Works through any attitude (no Euler angles), so it flies loops and rolls.
class FSIM_API AccelerationLoop final : public Controller {
public:
    AccelerationLoop();
    const char* id() const noexcept override { return "pid_acceleration"; }
    Level level() const noexcept override { return Level::Acceleration; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    Pid loadFactor{0.35, 0.3, 0.15, 0.6};
    Pid rollRate{0.8, 0.2, 0.0, 0.5};
    Pid longitudinal{0.15, 0.05, 0.0, 0.5, 0.0, 1.0};
    double rudderBetaGain = 1.0;
    double throttleFeedforward = 0.6;

private:
    Parameters params_;
};

/// Level::Velocity -> Attitude: vertical speed to pitch (with trim), turn
/// rate to bank, heading and airspeed passed to the attitude loop.
class FSIM_API VelocityLoop final : public Controller {
public:
    VelocityLoop();
    const char* id() const noexcept override { return "pid_velocity"; }
    Level level() const noexcept override { return Level::Velocity; }
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool setParameter(std::string_view name, double value) override { return params_.set(name, value); }
    std::optional<double> parameter(std::string_view name) const override { return params_.get(name); }

    Pid verticalSpeed{0.045, 0.012, 0.0, 0.25, -0.4, 0.35};
    double maxBankRad = 0.785;

private:
    Parameters params_;
};

/// Level::Position -> Velocity: bearing to the point becomes the heading,
/// altitude error becomes a bounded vertical speed.
class FSIM_API PositionLoop final : public Controller {
public:
    PositionLoop();
    const char* id() const noexcept override { return "pid_position"; }
    Level level() const noexcept override { return Level::Position; }
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

private:
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

private:
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

private:
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

private:
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
