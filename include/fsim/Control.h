#pragma once

// Multi-level control (design 9.3, ADR-20): commands, controller and
// behaviour interfaces. One plain struct per
// level; `kHold` (NaN) in a field means "keep the current value / let the
// controller decide", so partial commands are natural.

#include "fsim/Capability.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Rng.h"
#include "fsim/VehicleState.h"

#include "fsim/Export.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <memory>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace fsim::control {

/// The control levels. Their values are the C ABI's and Python's; their order
/// in the cascade is levelRank's.
enum class Level : std::uint8_t { Actuator = 0, Attitude, Acceleration, Velocity, Position, Behavior, Count };

FSIM_API const char* levelName(Level level) noexcept;

/// Where a level sits in the cascade, from the bottom: actuator 0,
/// acceleration 1 (the roll rate and load factor an attitude is flown with),
/// attitude 2, velocity 3, position 4, behaviour 5. A controller returns a
/// command at a level of lower rank than its own.
constexpr int levelRank(Level l) noexcept {
    switch (l) {
    case Level::Actuator: return 0;
    case Level::Acceleration: return 1;
    case Level::Attitude: return 2;
    case Level::Velocity: return 3;
    case Level::Position: return 4;
    case Level::Behavior: return 5;
    default: return -1;
    }
}
/// The levels with a controller, from the top of the cascade down.
inline constexpr Level kCascadeOrder[] = {Level::Position, Level::Velocity, Level::Attitude, Level::Acceleration};

inline constexpr double kHold = std::numeric_limits<double>::quiet_NaN();
inline bool isHold(double v) noexcept { return std::isnan(v); }
/// `v` unless it is kHold, then `fallback`.
inline double orHold(double v, double fallback) noexcept { return std::isnan(v) ? fallback : v; }

/// Normalised actuator positions, JSBSim conventions: aileron +right,
/// elevator +nose-down, rudder +nose-left (trailing edge left), throttle
/// 0..1, flaps 0..1.
struct ActuatorCommand {
    double aileron = 0.0, elevator = 0.0, rudder = 0.0;
    double throttle = 0.0;
    double flaps = 0.0;
    double gearDown = kHold;
    double brakeLeft = 0.0, brakeRight = 0.0;
};

/// Attitude hold. With `headingRad` set the roll is derived from the heading
/// error (bounded by `maxBankRad`) and `rollRad` is ignored. Throttle either
/// directly or through an airspeed hold. A rotorcraft flies the heading with
/// its yaw, keeping the roll asked for (and without one holds the heading it
/// had); its thrust is the throttle alone (docs/rotorcraft.md, 3.4).
struct AttitudeCommand {
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double headingRad = kHold;
    double maxBankRad = 0.785;
    double throttle = kHold;
    double airspeedMs = kHold;
};

/// Manoeuvre-style command: normal load factor and roll rate (enough for
/// loops, rolls and hard turns), optional longitudinal acceleration hold.
/// A rotorcraft flies body rates and its thrust: the roll, pitch and yaw
/// rates, and the load factor its rotors' thrust gives (1 in the hover);
/// it has no longitudinal acceleration of its own. A wing has no pitch or
/// yaw rate of its own to fly (its pitch follows the load factor).
struct AccelerationCommand {
    double loadFactorG = 1.0;
    double rollRateRadS = 0.0;
    double longitudinalMs2 = kHold;
    double throttle = kHold;
    double pitchRateRadS = kHold; ///< body, + nose up (a rotorcraft)
    double yawRateRadS = kHold;   ///< body, + nose right (a rotorcraft)
};

/// Flight-path command: airspeed, vertical speed and either a heading to hold
/// or a turn rate to fly. A rotorcraft may be given a velocity over the
/// ground instead of the airspeed (north and east, both), which it flies
/// whatever its heading: hovering, sideways, holding a point against the wind.
struct VelocityCommand {
    double airspeedMs = kHold;
    double verticalSpeedMs = 0.0;
    double headingRad = kHold;
    double turnRateRadS = kHold;
    double northMs = kHold; ///< over the ground (a rotorcraft)
    double eastMs = kHold;
};

/// Fly to a geodetic point at an altitude; "captured" within the radius. A
/// rotorcraft stops there (its airspeed the most it flies at on the way),
/// facing `headingRad` - or, without one, the point while it is far, then
/// the way it came.
struct PositionCommand {
    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double altitudeMslM = 0.0;
    double airspeedMs = kHold;
    double captureRadiusM = 200.0;
    double headingRad = kHold; ///< (a rotorcraft)
};

// --- Guidance modes (docs/vehicle-interface.md, ADR-28): fixed-size setpoints,
// flown by a behaviour per activity, that take UPDATE ----------------------------------

/// What a mode's speed is measured against (A-GRA's SpeedReferenceEnum and MachType).
enum class SpeedReference : std::uint8_t { TrueAirspeed = 0, CalibratedAirspeed = 1, GroundSpeed = 2, Mach = 3, Count };
/// What a mode's altitude is measured from (A-GRA's AltitudeReferenceEnum). The
/// simulation's sea level is the WGS-84 ellipsoid (JSBSim's), so Msl and
/// Ellipsoid are one; AboveGround follows the terrain under the aircraft.
enum class AltitudeReference : std::uint8_t { Msl = 0, AboveGround = 1, Ellipsoid = 2, Count };

/// fsim.guidance.hsa (A-GRA's HSA/CSA): hold a heading or a course, a speed
/// and an altitude, until told otherwise. Each field may be left out (kHold):
/// a NEW keeps what a live hsa activity it replaces commanded, else what the
/// aircraft flies now; an UPDATE keeps what was commanded. A reference given
/// without its value takes the aircraft's own now (a Mach reference alone:
/// hold the Mach it flies). A heading replaces a course and a course a heading.
/// References are enum values carried as doubles, so kHold can mean "as before".
struct HsaCommand {
    double headingRad = kHold;        ///< the nose's direction, true north
    double courseRad = kHold;         ///< or the track over the ground's
    double speed = kHold;             ///< m/s, or a Mach number
    double speedReference = kHold;    ///< SpeedReference
    double altitudeM = kHold;
    double altitudeReference = kHold; ///< AltitudeReference
};

/// A registered behaviour with its parameters (design 9.3 "Behavior").
struct BehaviorCommand {
    std::string id;                        ///< registry id: "hold", "waypoints", "loiter", "pursuit", ...
    std::uint32_t target = 0;              ///< another vehicle's id when the behaviour needs one
    std::map<std::string, double> params;  ///< behaviour-specific numbers (documented per behaviour)
    std::vector<PositionCommand> points;   ///< route for "waypoints"

    double param(const std::string& key, double fallback) const noexcept {
        const auto it = params.find(key);
        return it == params.end() ? fallback : it->second;
    }
};

/// A command: a level's, a behaviour's, or a guidance mode's setpoint. The
/// modes come after BehaviorCommand and enter at Level::Behavior (levelOf):
/// the variant's index is a level's only up to it.
using Command = std::variant<ActuatorCommand, AttitudeCommand, AccelerationCommand, VelocityCommand, PositionCommand, BehaviorCommand, HsaCommand>;

// Support effectors (docs/control-architecture.md, 8.2): set directly, not
// flown through the cascade; each its own capability (fsim.support.*) where
// the aircraft has the effector.

/// fsim.support.gear: completes when the gear is where it was told.
struct GearCommand {
    double down = 1.0; ///< 1 down, 0 up
};
/// fsim.support.flaps: completes when the flaps are where they were told.
struct FlapsCommand {
    double position = 0.0; ///< 0 up .. 1 full
};
/// fsim.support.wheel_brakes
struct WheelBrakesCommand {
    double left = 0.0, right = 0.0; ///< 0 .. 1
};
/// fsim.support.speedbrake
struct SpeedbrakeCommand {
    double position = 0.0; ///< 0 in .. 1 out
};
/// fsim.support.pitch_trim
struct PitchTrimCommand {
    double position = 0.0; ///< -1 .. 1, + nose down (JSBSim's sign)
};
/// fsim.flight.engines: a throttle per engine. A flight capability that owns
/// thrust, set directly like the support effectors (so beside the cascade,
/// which then flies roll and pitch without it).
struct EnginesCommand {
    double throttle[4] = {kHold, kHold, kHold, kHold}; ///< 0 .. 1 per engine; kHold keeps it
};

/// The commands set directly beside the cascade: the support effectors and per-engine throttles.
using SupportCommand = std::variant<GearCommand, FlapsCommand, WheelBrakesCommand, SpeedbrakeCommand, PitchTrimCommand, EnginesCommand>;

inline Level levelOf(const Command& c) noexcept {
    return c.index() < static_cast<std::size_t>(Level::Behavior) ? static_cast<Level>(c.index()) : Level::Behavior;
}
/// The registered behaviour that flies a mode's setpoint ("hsa"); null for a level's or a behaviour's command.
inline const char* modeBehavior(const Command& c) noexcept { return std::holds_alternative<HsaCommand>(c) ? "hsa" : nullptr; }



/// Read-only view of the world for behaviours that look at other vehicles.
/// Implemented by the session; states are the previous step's snapshots.
class WorldView {
public:
    virtual ~WorldView() = default;
    virtual const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept = 0;
    virtual double simTime() const noexcept = 0;
    virtual const sim::EnvironmentState& environment() const noexcept = 0;
};

/// What a controller sees each update (design 9.3 "ControlContext").
struct ControlContext {
    std::uint32_t vehicleId = 0;
    const sim::VehicleState& state;     ///< truth, before this FDM step
    const sim::VehicleState& sensed;    ///< through the vehicle's sensor models (equals `state` without effects)
    double dt = 0.0;                    ///< controller period, seconds
    const WorldView* world = nullptr;   ///< null when the stack runs stand-alone
    Rng* rng = nullptr;                 ///< this vehicle's stream (deterministic)
    /// The primary axes this level's command drives this update
    /// (docs/control-architecture.md, 9.5): all of them unless the vehicle's
    /// axes are owned apart. A controller that says axisAware() leaves the
    /// others alone - no integrating, no output (kHold).
    AxisMask engaged = kPrimaryAxes;
    /// What the vehicle can do (Feature bits), for a behaviour whose guidance
    /// differs by it: a rotorcraft hovers (docs/vehicle-interface.md, 4.9).
    std::uint32_t features = kFeatureWingborne;
    /// What it can do in numbers, for a behaviour: its guidance plans with it
    /// (docs/vehicle-interface.md, 7.1). Null outside a vehicle's runtime.
    const Performance* performance = nullptr;
};

/// One level of the cascade: accepts a command at `level()` and returns a
/// command at any level below it, by levelRank (design 9.3 "Controller").
class Controller {
public:
    virtual ~Controller() = default;

    virtual const char* id() const noexcept = 0;
    virtual Level level() const noexcept = 0;

    /// Translate `in` (a command at `level()`) into a lower-level command.
    virtual Command update(const ControlContext& ctx, const Command& in) = 0;

    /// Forget integrators and internal state (vehicle reset, controller swap).
    virtual void reset() {}

    /// Gains and other tunables by name; unknown names return false / nullopt.
    virtual bool setParameter(std::string_view name, double value) { (void)name; (void)value; return false; }
    virtual std::optional<double> parameter(std::string_view name) const { (void)name; return std::nullopt; }

    /// True if it honours ControlContext::engaged: then a command whose axes
    /// are owned apart may pass through it (docs/control-architecture.md, 9.6).
    virtual bool axisAware() const noexcept { return false; }
};

/// A top-level controller with a lifecycle (design 9.3 "Behavior"). A finished
/// behaviour keeps producing its last output until replaced.
class Behavior : public Controller {
public:
    Level level() const noexcept final { return Level::Behavior; }

    /// Called once with the command that selected this behaviour, before the first update().
    virtual void start(const ControlContext& ctx, const BehaviorCommand& command) { (void)ctx; (void)command; }
    /// What the runtime calls first: with a BehaviorCommand, start() above; a
    /// guidance mode (docs/vehicle-interface.md) takes its setpoint here.
    virtual void begin(const ControlContext& ctx, const Command& command) {
        if (const auto* b = std::get_if<BehaviorCommand>(&command)) start(ctx, *b);
    }
    /// The goal is reached: its activity completes (docs/control-architecture.md, 10.3).
    virtual bool finished() const noexcept { return false; }
    /// Why it can no longer do what it was asked, e.g. Reason::TargetLost once
    /// the vehicle it follows is gone; None while it can. Its activity fails,
    /// and it keeps flying whatever it falls back to.
    virtual Reason failure() const noexcept { return Reason::None; }
    /// How far it has got and what it commands, as of its last update
    /// (docs/vehicle-interface.md, 5.3): the fields it knows into `out`, and
    /// true; false if it reports nothing. Asked between world steps (the
    /// activity's record carries it), never during one.
    virtual bool progress(ActivityProgress& out) const noexcept {
        (void)out;
        return false;
    }
};

/// What the cascade asked for in its last control update, level by level
/// (docs/vehicle-interface.md, 5.3; A-GRA's VehicleCommandState): each field
/// from the level that sets it where that level ran, NaN where none did.
struct CommandedState {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    Level top = Level::Actuator;                                       ///< the highest level that ran
    double latitudeRad = kNone, longitudeRad = kNone, altitudeMslM = kNone; ///< the position level's point
    double headingRad = kNone;      ///< the velocity level's, else the attitude level's
    double turnRateRadS = kNone;    ///< the velocity level's
    double airspeedMs = kNone;      ///< true: the velocity level's, else the attitude level's
    double verticalSpeedMs = kNone; ///< the velocity level's
    double northMs = kNone, eastMs = kNone; ///< the velocity level's over the ground (a rotorcraft's)
    double rollRad = kNone, pitchRad = kNone; ///< the attitude level's
    double loadFactorG = kNone, rollRateRadS = kNone, pitchRateRadS = kNone, yawRateRadS = kNone; ///< the acceleration level's
    double throttle = kNone;        ///< what the actuators were given (the first engine's)
};

} // namespace fsim::control
