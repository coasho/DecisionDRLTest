#pragma once

// Multi-level control (design 9.3, ADR-20): commands, controller and
// behaviour interfaces. One plain struct per
// level; `kHold` (NaN) in a field means "keep the current value / let the
// controller decide", so partial commands are natural.

#include "fsim/Capability.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Rng.h"
#include "fsim/Span.h"
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
/// The speed a mode varies by itself (A-GRA's SpeedOptimizationEnum;
/// docs/flight-autonomy.md, 4.17): the performance tables' best-range speed
/// (the most distance for the fuel) or best-endurance speed (the most time),
/// at the altitude and the weight now, flown as a true airspeed.
enum class SpeedOptimization : std::uint8_t { LongRangeCruise = 0, MaxEndurance = 1, Count };

/// fsim.guidance.hsa (A-GRA's HSA/CSA): hold a heading or a course, a speed
/// and an altitude, until told otherwise. Each field may be left out (kHold):
/// a NEW keeps what a live hsa activity it replaces commanded, else what the
/// aircraft flies now; an UPDATE keeps what was commanded. A reference given
/// without its value takes the aircraft's own now (a Mach reference alone:
/// hold the Mach it flies). A heading replaces a course and a course a heading;
/// a speed replaces a speed optimisation and an optimisation a speed.
/// References are enum values carried as doubles, so kHold can mean "as before".
struct HsaCommand {
    double headingRad = kHold;        ///< the nose's direction, true north
    double courseRad = kHold;         ///< or the track over the ground's
    double speed = kHold;             ///< m/s, or a Mach number
    double speedReference = kHold;    ///< SpeedReference
    double altitudeM = kHold;
    double altitudeReference = kHold; ///< AltitudeReference
    /// SpeedOptimization: the speed it varies by itself. Resolved, `speed` is
    /// the optimum's true airspeed at the altitude flown to, as the command
    /// was given; the mode flies it afresh at the altitude and weight now.
    double speedOptimization = kHold;
};

/// How a route passes a waypoint (A-GRA's TurnType).
enum class TurnType : std::uint8_t {
    FlyBy = 0,   ///< the turn begins before the point, on a circle tangent to both legs
    FlyOver = 1, ///< over the point, then the next leg is intercepted
    Count
};
/// What a route's legs are on the Earth.
enum class Projection : std::uint8_t { GreatCircle = 0, Rhumb = 1, Count };
/// What a route does after its last point.
enum class EndBehavior : std::uint8_t {
    Continue = 0, ///< the last leg's course, altitude and speed, on along the leg
    Loiter = 1,   ///< a wing orbits the point, at the radius its speed and 80 % of its bank give; a rotorcraft stops and hovers there
    Count
};

/// One waypoint of a route (A-GRA's), and the segment that ends at it: flown
/// to from the previous point, the first from where the aircraft is when the
/// route starts. A field left out (kHold) continues the previous point's; the
/// first point's is the aircraft's own now - a reference given alone, its
/// value in that reference now - and a rotorcraft given no speed flies its
/// cruise speed over the ground.
struct Waypoint {
    double latitudeRad = 0.0, longitudeRad = 0.0;
    double altitudeM = kHold;          ///< reached at the point, along a straight profile from the previous one
    double altitudeReference = kHold;  ///< AltitudeReference
    double speed = kHold;              ///< flown on the segment to the point: m/s, or a Mach number
    double speedReference = kHold;     ///< SpeedReference
    double turn = 0.0;                 ///< TurnType
    double maxBankRad = kHold;         ///< the bank its fly-by turn is planned with; kHold: 80 % of the aircraft's
    double climbRateMs = kHold;        ///< climb or descend at this rate, then level; kHold: along the segment's gradient
    std::uint64_t id = 0;              ///< the caller's, reported back in the progress
};

/// fsim.guidance.route (A-GRA's waypoint following): its waypoints go beside
/// it (World::submit and update take a Span) into the vehicle's path store.
/// Completes after the last point, unless it repeats.
struct RouteCommand {
    double projection = 0.0; ///< Projection: 0 great circles, 1 rhumb lines
    double repeat = 0.0;     ///< 1: after the last point, fly the route again from its first
    double end = 0.0;        ///< EndBehavior after the last point
    double start = 0.0;      ///< the point to fly to first
};

/// One piece of a curve (A-GRA's): a quintic Bezier by its six control
/// points (weights 1, the clamped knots [0,0,0,0,0,0,1,1,1,1,1,1]), metres
/// north, east and down from the curve's reference point.
struct BezierSegment {
    double north[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double east[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double down[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

/// fsim.guidance.curve (A-GRA's curve following): its segments - 1 to 10 a
/// command, each starting where the one before ends - go beside it (World::submit
/// and update take a Span) into the path store. Flown at a ground speed within
/// a range, or so as to take a duration; completes at its end. A field left
/// out (kHold) takes its default in a NEW and keeps its value in an UPDATE.
struct CurveCommand {
    double latitudeRad = kHold, longitudeRad = kHold; ///< the reference its segments are from; kHold: where the aircraft is
    double altitudeM = kHold;          ///< the reference's height above sea level; kHold: the aircraft's
    double speedMinMs = kHold;         ///< the ground speed to fly it within; both kHold: as now (a rotorcraft's cruise)
    double speedMaxMs = kHold;
    double durationS = kHold;          ///< or the time to fly all of it, from the NEW
    double end = kHold;                ///< EndBehavior after its end; kHold: continue
    double append = kHold;             ///< in an UPDATE, 1: its segments after the curve's end, from the same reference
};

/// Where a vehicle's route or curve lives while it is flown
/// (docs/vehicle-interface.md, 4.2): allocated at its first and kept, written
/// by the host between steps, read by the mode's behaviour during them
/// (ControlContext::path).
struct PathStore {
    static constexpr std::size_t kWaypoints = 256;
    static constexpr std::size_t kSegments = 32;
    std::uint32_t revision = 0; ///< bumped on every write
    std::uint32_t count = 0;    ///< waypoints
    Waypoint waypoints[kWaypoints];
    std::uint32_t curve = 0;    ///< bumped when a curve is replaced (not appended to): it is flown afresh
    std::uint32_t segmentCount = 0;
    BezierSegment segments[kSegments];
};

/// A loiter pattern (A-GRA's LOITER).
enum class PatternKind : std::uint8_t {
    Orbit = 0,       ///< a circle round the centre
    Racetrack = 1,   ///< two half circles joined by straight legs, the inbound one ending at the fix
    FigureEight = 2, ///< two circles meeting at the centre, one flown each way round
    Hold = 3,        ///< ATC's holding pattern: a racetrack on the fix with a minute's legs, entered direct to the fix
    Count
};

/// fsim.guidance.pattern (A-GRA's loiter): an orbit, a racetrack, a
/// figure-eight or a hold, until canceled or for a duration. A field left
/// out (kHold) takes its default in a NEW and keeps what was commanded in an
/// UPDATE. The defaults: an orbit, where the aircraft is, as it flies now (a
/// rotorcraft at its cruise speed over the ground), right turns, the course it
/// tracks now (a hold's: to its fix), the radius its speed, the wind and 80 %
/// of its bank give (a hold's: rate one, at most 25 degrees of bank), legs of
/// twice the radius (a hold's: a minute's flight, 90 s above 14,000 ft).
struct PatternCommand {
    double pattern = kHold;            ///< PatternKind
    double latitudeRad = kHold, longitudeRad = kHold; ///< the centre, or a racetrack's or a hold's fix
    double altitudeM = kHold;
    double altitudeReference = kHold;  ///< AltitudeReference
    double radiusM = kHold;            ///< at least the turn radius at the aircraft's speed and full bank; a rotorcraft's a metre
    double clockwise = kHold;          ///< 1 right turns, 0 left
    double courseRad = kHold;          ///< a racetrack's or a hold's inbound course, a figure-eight's axis
    double legM = kHold;               ///< a racetrack's or a hold's straight legs
    double speed = kHold;              ///< m/s, or a Mach number
    double speedReference = kHold;     ///< SpeedReference
    double durationS = kHold;          ///< then it completes (and flies on); kHold: until canceled
    double speedOptimization = kHold;  ///< SpeedOptimization, as an hsa's: a speed replaces it, it a speed
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
using Command = std::variant<ActuatorCommand, AttitudeCommand, AccelerationCommand, VelocityCommand, PositionCommand, BehaviorCommand, HsaCommand,
                             RouteCommand, PatternCommand, CurveCommand>;

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

/// One command of a batch NEW (World::submitBatch; A-GRA's several command
/// instances in one message, each answered on its own; docs/flight-autonomy.md, 4.8):
/// a flight or guidance command - a route's waypoints or a curve's segments
/// beside it - or a support effector's, with its options.
struct BatchCommand {
    std::variant<Command, SupportCommand> command;
    Span<const Waypoint> waypoints;     ///< a RouteCommand's
    Span<const BezierSegment> segments; ///< a CurveCommand's
    CommandOptions options;
};

/// What a live activity flies now, or waits to fly (A-GRA's last flight
/// command, in the activity report; docs/flight-autonomy.md, 4.12): its
/// setpoint as updated - a mode's merged, a level's, a support effector's -
/// and a route's waypoints or a curve's segments, appended ones too (a
/// curve's: A-GRA's FlyoutCurve, from the CurveCommand's reference).
struct Setpoint {
    std::variant<Command, SupportCommand> command;
    std::vector<Waypoint> waypoints;
    std::vector<BezierSegment> segments;
};

/// Where an activity flies to (A-GRA's ActualEndPoint, MA_EndPointType;
/// docs/flight-autonomy.md, 4.12): a point, a turn flown by or over it
/// (TurnType), or a loiter.
enum class EndPointKind : std::uint8_t { Waypoint, TurnPoint, LoiterPoint, Count };
struct EndPoint {
    EndPointKind kind = EndPointKind::Waypoint;
    double latitudeRad = kHold, longitudeRad = kHold;
    double altitudeM = kHold;          ///< in `altitudeReference` (a waiting route's as given: kHold continues the point before)
    double altitudeReference = kHold;  ///< AltitudeReference
    double turn = kHold;               ///< a turn point's TurnType
    std::uint64_t id = 0;              ///< a route waypoint's id; 0 none
    std::int32_t index = -1;           ///< its waypoint or curve segment; 0 a pattern's or the position level's point
};

inline Level levelOf(const Command& c) noexcept {
    return c.index() < static_cast<std::size_t>(Level::Behavior) ? static_cast<Level>(c.index()) : Level::Behavior;
}
/// The registered behaviour that flies a mode's setpoint ("hsa", "route", "pattern", "curve"); null for a level's or a behaviour's command.
inline const char* modeBehavior(const Command& c) noexcept {
    if (std::holds_alternative<HsaCommand>(c)) return "hsa";
    if (std::holds_alternative<RouteCommand>(c)) return "route";
    if (std::holds_alternative<PatternCommand>(c)) return "pattern";
    if (std::holds_alternative<CurveCommand>(c)) return "curve";
    return nullptr;
}



/// Read-only view of the world for behaviours that look at other vehicles.
/// Implemented by the session; states are the previous step's snapshots.
struct TablesSection; // fsim/VehicleProfile.h

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
    /// Its clean configuration's limits (the profile's envelope; NaN where it
    /// has none), for a behaviour that flies to them: a manoeuvre that gives
    /// up when it departs. Null outside a vehicle's runtime.
    const EnvelopeLimits* envelope = nullptr;
    /// The vehicle's route, for the behaviour that flies it; null until one was given.
    const PathStore* path = nullptr;
    /// Its performance tables (the profile's; docs/flight-autonomy.md, 4.13),
    /// for a mode that flies their best speeds: null without them.
    const TablesSection* tables = nullptr;
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
    /// ActivityFlag bits for its last update, beside what the runtime sees of
    /// the loops: a setpoint it held back to keep the aircraft safe
    /// (kActivityClamped: an evade's descent stopped at its floor).
    virtual std::uint16_t constraints() const noexcept { return 0; }
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

/// What a vehicle is commanded (A-GRA's VehicleCommandState;
/// docs/flight-autonomy.md, 4.12): the cascade's levels, the acceleration
/// they ask of the aircraft in north, east and down, and the altitude as its
/// mode commanded it, in the reference it was commanded in.
struct VehicleCommandState : CommandedState {
    /// A wing's: the acceleration level's longitudinal acceleration along its
    /// flight path and its load factor's lift normal to the path (JSBSim's
    /// lift load factor), with gravity's pull, at the attitude, angle of
    /// attack and sideslip it flies - its acceleration over the Earth. NaN
    /// where no longitudinal acceleration is commanded (a throttle given
    /// instead), and for a rotorcraft (whose thrust, its drag unmodelled, is
    /// not its acceleration).
    double northAccelerationMs2 = kNone, eastAccelerationMs2 = kNone, downAccelerationMs2 = kNone;
    /// A live hsa's or pattern's altitude, a route's point flown to's, in
    /// their reference; a curve's, and the position level's, above sea level.
    /// NaN where none is commanded.
    double altitudeM = kNone;
    double altitudeReference = kNone; ///< AltitudeReference
};

// --- The navigation report (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport) ---

/// What a vehicle flies on.
enum class Energy : std::uint8_t {
    Unknown = 0, ///< its flight model does not say (neither fuel nor a battery it knows)
    Fuel = 1,
    Battery = 2,
};

/// A-GRA's SystemContingencyLevelEnum: the vehicle's contingency. The
/// platform reports a low fuel state (FlightCritical); it models no
/// subsystem failures (MissionCritical) and no communications (LostComms).
enum class Contingency : std::uint8_t {
    Normal = 0,
    MissionCritical = 1,
    FlightCritical = 2, ///< at or below its reserve, or an engine starved
    LostComms = 3,
};

/// Where a vehicle recovers to, and what it keeps for the end: its playtime
/// counts the return and the reserve.
struct NavigationSettings {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    bool recovery = false;               ///< a recovery point set
    double latitudeDeg = kNone, longitudeDeg = kNone, altitudeMslM = kNone;
    double reserveFraction = 0.1;        ///< of its capacity, fuel or charge, kept for the end
};

/// A-GRA's MA_NavigationReport (its Endurance, Playtime and ContingencyLevel):
/// what the vehicle flies on, how much it has and for how long. Fuel in kg
/// (a battery's charge in J), its consumption now (kg/s, W): the engines'
/// fuel flow or the power the battery gives.
struct NavigationReport {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    Energy energy = Energy::Unknown;
    double fuelKg = kNone;       ///< A-GRA's Fuel: in its tanks (0 for a battery)
    double remaining = kNone;    ///< fuel (kg) or charge (J)
    double capacity = kNone;     ///< its tanks' (kg) or battery's (J), full
    double percent = kNone;      ///< A-GRA's Percent: remaining over capacity, %
    double consumption = kNone;  ///< now: fuel flow (kg/s) or power (W)
    /// A-GRA's Duration: remaining over consumption now, s (infinite while it
    /// consumes nothing: a helicopter's engines off, a multirotor at rest; 0
    /// with nothing left).
    double enduranceS = kNone;
    double reserve = kNone;      ///< kept for the end (NavigationSettings), kg or J
    /// A-GRA's Playtime, with a recovery point: what it can spend before it
    /// turns back - remaining less the reserve and the return, over its
    /// consumption now, s (0 once past it). NaN without a recovery point.
    double playtimeS = kNone;
    double returnDistanceM = kNone; ///< to the recovery point, over the ground
    double returnTasMs = kNone;     ///< it would fly back at: its best-range speed (the tables), else its cruise
    double returnConsumption = kNone; ///< it would burn back (kg/s or W)
    Contingency contingency = Contingency::Normal;
    bool starved = false;           ///< its engines have nothing left: a fuel burner's tanks empty, or the battery spent
};

FSIM_API const char* energyName(Energy e) noexcept;           ///< "unknown", "fuel", "battery"
FSIM_API const char* contingencyName(Contingency c) noexcept; ///< A-GRA's: "NORMAL", "MISSION_CRITICAL", "FLIGHT_CRITICAL", "LOST_COMMS"

} // namespace fsim::control
