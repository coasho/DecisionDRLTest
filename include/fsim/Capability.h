#pragma once

// Capability contracts (design 9.3, ADR-26, docs/control-architecture.md):
// what a vehicle offers (capabilities), how a command is answered (NEW,
// UPDATE, CANCEL with a synchronous result) and what it becomes (an activity
// that is pending, active, then completed, failed or canceled).

#include "fsim/Export.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace fsim::control {

enum class Level : std::uint8_t; // fsim/Control.h

/// What an activity can own (docs/control-architecture.md, 6.1). The primary
/// axes are flown through the cascade; the support axes are set directly.
enum class Axis : std::uint8_t {
    Roll, Pitch, Yaw, Thrust,                    ///< primary
    Flaps, Gear, Brakes, Speedbrake, PitchTrim,  ///< support
    Count
};

using AxisMask = std::uint16_t;

inline constexpr std::size_t kAxisCount = static_cast<std::size_t>(Axis::Count);
inline constexpr std::size_t kPrimaryAxisCount = static_cast<std::size_t>(Axis::Flaps);
inline constexpr std::size_t kSupportAxisCount = kAxisCount - static_cast<std::size_t>(Axis::Flaps);

constexpr AxisMask axisBit(Axis a) noexcept { return static_cast<AxisMask>(1u << static_cast<unsigned>(a)); }

inline constexpr AxisMask kPrimaryAxes = axisBit(Axis::Roll) | axisBit(Axis::Pitch) | axisBit(Axis::Yaw) | axisBit(Axis::Thrust);
inline constexpr AxisMask kAllAxes = static_cast<AxisMask>((1u << kAxisCount) - 1u);
/// What a command through the existing entry points owns: everything its
/// command structs can set (docs/control-architecture.md, 10.7).
inline constexpr AxisMask kLegacyAxes = kPrimaryAxes | axisBit(Axis::Flaps) | axisBit(Axis::Gear) | axisBit(Axis::Brakes);

/// At most this many activities fly through the cascade at once: each owns
/// at least one primary axis.
inline constexpr std::size_t kSlotCount = 4;

/// The groups a flight command may own on its own (docs/control-architecture.md,
/// 6.1), which the aircraft's family sets (docs/rotorcraft.md, 3.3). A wing's:
/// above the actuators, roll and yaw together (the loop that banks also
/// coordinates), pitch, thrust. A rotorcraft's: the cyclic's roll and pitch
/// together (they tilt the thrust that moves it), the yaw (the pedals), the
/// thrust (the collective). At the actuators each primary axis alone.
inline constexpr AxisMask kLateral = axisBit(Axis::Roll) | axisBit(Axis::Yaw);
inline constexpr AxisMask kCyclic = axisBit(Axis::Roll) | axisBit(Axis::Pitch);
enum AxisGroup : std::uint8_t {
    kGroupLateral = 1u << 0,  ///< roll and yaw
    kGroupPitch = 1u << 1,    ///< pitch
    kGroupThrust = 1u << 2,   ///< thrust
    kGroupEachAxis = 1u << 3, ///< any primary axis alone
    kGroupCyclic = 1u << 4,   ///< roll and pitch (a rotorcraft)
    kGroupYaw = 1u << 5,      ///< yaw (a rotorcraft)
};
/// The axes a group (one AxisGroup bit) owns; 0 for kGroupEachAxis.
constexpr AxisMask groupAxes(AxisGroup group) noexcept {
    switch (group) {
    case kGroupLateral: return kLateral;
    case kGroupPitch: return axisBit(Axis::Pitch);
    case kGroupThrust: return axisBit(Axis::Thrust);
    case kGroupCyclic: return kCyclic;
    case kGroupYaw: return axisBit(Axis::Yaw);
    default: return 0;
    }
}
/// `axes` widened to every group of `groups` it touches: what a command above
/// the actuators owns when it asks for part of a group.
constexpr AxisMask widenToGroups(AxisMask axes, std::uint8_t groups) noexcept {
    AxisMask out = axes;
    for (unsigned bit = 0; bit < 8; ++bit) {
        const auto g = static_cast<AxisGroup>(1u << bit);
        if ((groups & g) && (groupAxes(g) & axes)) out = static_cast<AxisMask>(out | groupAxes(g));
    }
    return out;
}

/// What an aircraft can do that a behaviour may need (BehaviorTraits::features),
/// as its family's adapter declares it.
enum Feature : std::uint32_t {
    kFeatureWingborne = 1u << 0, ///< flies on a wing: manoeuvres by load factor and bank
    kFeatureHover = 1u << 1,     ///< holds a point in the air
};

/// What flies a primary axis nobody owns.
enum class VehicleDefault : std::uint8_t {
    Neutral, ///< the neutral actuator command every vehicle starts with: surfaces centred, throttle 0
    Hold,    ///< the airspeed, heading and height it had when the axes were let go
};

/// Why a command was rejected or an activity ended (docs/control-architecture.md, 10.2).
enum class Reason : std::uint8_t {
    None = 0,
    // results
    UnknownVehicle,
    UnknownCapability,
    UnknownActivity,
    // NEW rejected
    Unavailable,
    VersionUnsupported,
    InvalidParameter,
    OutOfRange,
    InvalidAxes,
    AuthorityHeld,
    ControllerNotAxisAware,
    // UPDATE or CANCEL rejected
    ActivityEnded,
    NotUpdatable,
    WrongCommandType,
    // an activity's end
    GoalReached,     ///< Completed
    Requested,       ///< Canceled by CANCEL
    Preempted,       ///< Canceled: another activity took its axes
    TargetLost,      ///< Failed: the vehicle it follows is gone
    BehaviorFailed,  ///< Failed: the behaviour gave up
    CapabilityLost,  ///< Failed: the capability became unavailable
    Diverged,        ///< Failed: the flight model diverged
    // the Vehicle Interface (docs/vehicle-interface.md, 5.1 and 6)
    InvalidWaypoint,    ///< NEW or UPDATE rejected: a route point it cannot fly (CommandResult::index)
    InvalidCurve,       ///< NEW or UPDATE rejected: a curve segment it cannot fly (index, from, to)
    PerformanceLimit,   ///< NEW or UPDATE rejected: beyond what the aircraft can do (CommandResult::constraint)
    NotGranted,         ///< NEW rejected: the vehicle requires a grant the policy does not hold
    NotAllowed,         ///< a request for control of a capability the policy may not have
    Revoked,            ///< Canceled: the platform revoked the policy's grant
    Released,           ///< Canceled: the policy released its grant
    CollisionAvoidance, ///< unavailable while the platform avoids a collision
    Restricted,         ///< unavailable: the platform restricts it
    // what the vehicle can do at all, and in which flight phase (docs/flight-autonomy.md, 4.3)
    NotSupported,   ///< a physical exception on this aircraft: its support table names the rule and the evidence
    NotImplemented, ///< applicable to this aircraft, not built yet: its support table names the stage that builds it
    OnGround,       ///< unavailable to a policy on the ground (the airborne guidance)
    Airborne,       ///< unavailable to a policy in the air (the ground modes)
    Count
};

/// "authority_held", "goal_reached", ...
FSIM_API const char* reasonName(Reason reason) noexcept;
/// The reason in words: "the aircraft is on the ground", ...; "" for None.
FSIM_API const char* reasonDescription(Reason reason) noexcept;
/// A set of reasons (CapabilityStatus::reasons): a bit per Reason.
constexpr std::uint64_t reasonBit(Reason reason) noexcept { return std::uint64_t{1} << static_cast<unsigned>(reason); }
static_assert(static_cast<unsigned>(Reason::Count) <= 64, "a reason set is 64 bits");

/// The performance limit a command's value breaks (CommandResult::constraint;
/// A-GRA's MA_PerformanceConstraintEnum, docs/vehicle-interface.md 5.1).
enum class Constraint : std::uint8_t {
    None,
    MinAirspeed,
    MaxAirspeed,
    MinAltitude,
    MaxAltitude,
    MinAcceleration,
    MaxAcceleration,
    MaxOrientation,     ///< a bank or a pitch
    MaxOrientationRate, ///< a roll, pitch or yaw rate
    MaxTurnRate,
    MaxClimbRate,
    MaxDescentRate,
    Count
};
/// "max_airspeed", "max_orientation", ...; "none".
FSIM_API const char* constraintName(Constraint constraint) noexcept;

// --- Commands ---------------------------------------------------------------------

/// An activity: the vehicle's id in the high 32 bits, a serial per vehicle
/// (from 1) in the low ones. 0 = none.
using ActivityId = std::uint64_t;
constexpr ActivityId activityId(std::uint32_t vehicle, std::uint32_t serial) noexcept { return (static_cast<ActivityId>(vehicle) << 32) | serial; }
constexpr std::uint32_t activityVehicle(ActivityId id) noexcept { return static_cast<std::uint32_t>(id >> 32); }

/// Who commands, in rising priority: a newer command preempts activities of
/// its own or a lower priority and is rejected by a higher one.
enum class Source : std::uint8_t {
    Policy = 0,    ///< a trainer's or RL policy's commands; every existing entry point
    Autopilot = 1, ///< an engaged autopilot mode a policy must not silently override
    Override = 2,  ///< an operator or a scenario script taking control
};

/// What happens to a value outside the capability's advertised range.
enum class RangePolicy : std::uint8_t {
    Clamp,  ///< clamp it; the result says so (kClamped)
    Reject, ///< reject the command (OutOfRange)
    None,   ///< no parameter or availability checks: the existing entry points' behaviour
};

struct CommandOptions {
    Source source = Source::Policy;
    AxisMask axes = 0;                      ///< 0 = the capability's default axes
    RangePolicy range = RangePolicy::Clamp;
    std::uint16_t minVersion = 0;           ///< reject a capability older than this
};

enum class CommandStatus : std::uint8_t { Accepted, Rejected, Canceled };
enum CommandFlag : std::uint16_t { kClamped = 1u << 0 };

/// The synchronous answer to NEW, UPDATE and CANCEL.
struct CommandResult {
    CommandStatus status = CommandStatus::Rejected;
    Reason reason = Reason::None;
    ActivityId activity = 0; ///< the new (NEW) or addressed (UPDATE, CANCEL) activity
    ActivityId other = 0;    ///< the activity that holds the authority (AuthorityHeld)
    std::uint16_t flags = 0; ///< CommandFlag bits
    // What a rejection or a clamp was about (docs/vehicle-interface.md, 5.1):
    /// the field (in the command struct's order, or a behaviour's named
    /// parameter's place in its descriptor), route point or curve segment; -1: none
    std::int16_t index = -1;
    Constraint constraint = Constraint::None; ///< the performance limit its value broke
    float from = std::numeric_limits<float>::quiet_NaN(); ///< a curve segment's section that breaks it: its parameter, 0..1
    float to = std::numeric_limits<float>::quiet_NaN();
    bool accepted() const noexcept { return status == CommandStatus::Accepted; }
};

// --- Activities ---------------------------------------------------------------------

enum class ActivityState : std::uint8_t { Pending, Active, Completed, Failed, Canceled };
/// "pending", "active", ...
FSIM_API const char* activityStateName(ActivityState state) noexcept;

enum ActivityFlag : std::uint16_t {
    kActivitySaturated = 1u << 0,     ///< an effector it drives sat at its travel limit
    kActivityDemandLimited = 1u << 1, ///< protection reduced its demand
    kActivityExceeded = 1u << 2,      ///< the state was beyond a limit on its axes
    kActivityClamped = 1u << 3,       ///< a setpoint was clamped to the advertised range
    kActivityAxesReduced = 1u << 4,   ///< another activity took one of its support axes
};

/// How far an activity has got and what it commands (docs/vehicle-interface.md,
/// 5.3): a route's, a pattern's or a curve's, from its behaviour
/// (Behavior::progress). NaN, or 0 counts, where it says nothing.
struct ActivityProgress {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    std::uint32_t segment = 0;   ///< the waypoint, curve segment or pattern leg flown now (from 0)
    std::uint32_t segments = 0;  ///< of how many; 0: nothing segmented
    std::uint64_t segmentId = 0; ///< the waypoint's id, where the route gave one
    std::uint32_t laps = 0;      ///< a pattern's or a repeating route's, completed
    double percent = kNone;      ///< of the whole activity, 0..100
    double segmentPercent = kNone;
    double distanceToGoM = kNone; ///< to the end: a route's last point, a curve's end
    double timeToGoS = kNone;     ///< at the ground speed now
    double crossTrackM = kNone;   ///< + right of the path
    // what it commands: A-GRA's VehicleCommandState
    double courseRad = kNone, headingRad = kNone; ///< true
    double altitudeMslM = kNone;
    double speedMs = kNone;        ///< in speedReference's terms
    double speedReference = kNone; ///< SpeedReference (fsim/Control.h)
};

struct ActivityRecord {
    ActivityId id = 0;
    std::uint32_t vehicle = 0;
    std::uint16_t capability = 0;      ///< index into the vehicle's capabilities
    Source source = Source::Policy;
    AxisMask axes = 0;                 ///< the axes it owns (or owned, once ended)
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;      ///< why it ended, else None
    ActivityId by = 0;                 ///< the preempting activity, with Preempted
    std::uint16_t constraints = 0;     ///< ActivityFlag bits of the last world step
    std::uint16_t constraintsSeen = 0; ///< every ActivityFlag bit since it started
    double startTime = 0.0;            ///< simulation time
    double endTime = std::numeric_limits<double>::quiet_NaN(); ///< NaN while live
    ActivityProgress progress{};       ///< as its behaviour reported it after the last world step (a mode's)
    bool live() const noexcept { return state == ActivityState::Pending || state == ActivityState::Active; }
};

// --- Envelope protection ------------------------------------------------------------

/// What envelope protection does (docs/control-architecture.md, section 11).
/// It limits what the control system demands; it does not keep the aircraft
/// inside its envelope - a gust, inertia or a saturated surface can still take
/// it past a limit, and that is reported, not prevented.
enum class ProtectionMode : std::uint8_t {
    Off,    ///< no limiting, no reports: the default for an aircraft without an envelope
    Report, ///< the state's exceedances reported; the demand untouched
    Limit,  ///< the demand limited to the envelope, and exceedances reported: the default with one
};

/// The limits of an envelope, by what they bound.
enum class Limit : std::uint8_t { LoadFactorMax, LoadFactorMin, AlphaMax, Bank, PitchMax, PitchMin, RollRate, CasMin, CasMax, Mach, Count };
inline constexpr std::size_t kLimitCount = static_cast<std::size_t>(Limit::Count);
/// "load_factor_max", "load_factor_min", "alpha_max", "bank", "pitch_max",
/// "pitch_min", "roll_rate", "cas_min", "cas_max", "mach".
FSIM_API const char* limitName(Limit limit) noexcept;

/// One configuration's limits (the profile's envelope section); NaN = no limit.
struct EnvelopeLimits {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double loadFactorMin = kNone, loadFactorMax = kNone; ///< g
    double alphaMaxRad = kNone;
    double bankMaxRad = kNone, pitchMinRad = kNone, pitchMaxRad = kNone;
    double rollRateMaxRadS = kNone;
    double casMinMs = kNone, casMaxMs = kNone; ///< calibrated airspeed
    double machMax = kNone;
};

/// One limit's record since the status was last read.
struct LimitStatus {
    std::uint32_t limitedUpdates = 0;  ///< control updates in which protection reduced the demand for it
    std::uint32_t exceededUpdates = 0; ///< control updates in which the state was beyond it
    double exceededS = 0.0;            ///< how long the state was beyond it, s
    double worstExcess = 0.0;          ///< the most it was beyond: g, rad, rad/s, m/s (calibrated) or Mach
};

/// What envelope protection saw: the mode, and each limit's record since the
/// status was last read. What to do about an exceedance - a reward term, the
/// end of an episode - is the caller's; the platform only reports it.
struct EnvelopeStatus {
    ProtectionMode mode = ProtectionMode::Off;
    std::array<LimitStatus, kLimitCount> limits{};
    const LimitStatus& operator[](Limit l) const noexcept { return limits[static_cast<std::size_t>(l)]; }
};

// --- Capabilities -------------------------------------------------------------------

/// Whether a capability can be commanded now (A-GRA's CapabilityAvailabilityEnum;
/// docs/flight-autonomy.md, 4.1). Whether the aircraft can do it at all is
/// its support (Support), which never shows here as Disabled.
enum class Availability : std::uint8_t {
    Available,
    TemporarilyUnavailable, ///< a condition that will pass (weight on wheels, a placard speed, a diverged vehicle)
    Faulted,                ///< an effector has failed
    Disabled,               ///< switched off for this vehicle (the platform's setAvailability)
    Unavailable,            ///< not available, and when it returns is not known (an aircraft that cannot do it)
    Expended,               ///< used up
};
/// "available", "temporarily_unavailable", "faulted", "disabled", "unavailable", "expended".
FSIM_API const char* availabilityName(Availability availability) noexcept;

/// A command field whose range is narrower now than the capability
/// advertises (a placard: docs/flight-autonomy.md, 4.6). A NEW outside it is
/// refused with the reason the status gives for it.
struct CurrentRange {
    std::int16_t parameter = -1; ///< its place in the descriptor's parameters
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();
};

/// A capability's availability now (A-GRA's AvailabilityInfo), as a policy
/// is answered: what admission would say to its NEW.
struct CapabilityStatus {
    static constexpr std::size_t kMaxRanges = 4;
    Availability availability = Availability::Available;
    Reason reason = Reason::None;  ///< the first of `reasons`
    std::uint64_t reasons = 0;     ///< every reason that holds now (reasonBit)
    const char* description = "";  ///< the reason in words (reasonDescription, or the placard's own)
    std::uint64_t associated = 0;  ///< an id the reason is about (the vehicle avoided, an activity); 0: none
    double nextAvailableS = std::numeric_limits<double>::quiet_NaN(); ///< simulation time it is expected back; NaN: not known
    std::uint8_t rangeCount = 0;   ///< parameters narrower now
    std::array<CurrentRange, kMaxRanges> ranges{};
};

/// Who may command a vehicle's capabilities as Source::Policy
/// (docs/vehicle-interface.md, 6.1). The platform's own sources (Autopilot,
/// Override) never need a grant.
enum class ControlMode : std::uint8_t {
    Open,    ///< as ADR-26: any capability, arbitrated by source and axes (the default)
    Granted, ///< only the capabilities a grant covers: a policy's NEW without one is refused NotGranted
};

/// A capability's control status for a vehicle's policy (A-GRA's ControlStatus;
/// the primary controller is always the platform).
struct ControlStatus {
    bool allowed = true;  ///< the policy may request it (all may, by default)
    bool granted = false; ///< the policy holds a grant for it
};

enum class CapabilityKind : std::uint8_t { Flight, Guidance, Support, Status };
enum class Persistence : std::uint8_t {
    Persistent,  ///< runs until canceled, preempted or failed (a hold)
    Terminating, ///< completes when it reaches its goal (a route, a manoeuvre)
};
enum Interaction : std::uint8_t { kCommand = 1u << 0, kUpdate = 1u << 1, kCancel = 1u << 2, kSettings = 1u << 3, kStatus = 1u << 4 };

/// How a capability is controlled (A-GRA's AcceptedInterface,
/// CapabilityControlInterfacesEnum; CapabilityDescriptor::accepted).
enum AcceptedInterface : std::uint8_t {
    kAcceptsCapabilityCommand = 1u << 0, ///< a command starts an activity (NEW)
    kAcceptsActivityCommand = 1u << 1,   ///< its activities change or end on command (UPDATE, CANCEL)
    kAcceptsTaskCommand = 1u << 2,       ///< a flight task starts it (docs/flight-autonomy.md, FA-2)
    kAcceptsAutoMdf = 1u << 3,           ///< it acts on its own, as it is built to (envelope protection)
};

/// Which of A-GRA's flight capability types (MA_FlightCapabilityEnum) a
/// capability is (CapabilityDescriptor::mode; docs/vehicle-interface.md 4.1).
enum class FlightMode : std::uint8_t {
    None,
    HsaCsa,
    WaypointFollowing,
    CurveFollowing,
    Loiter,
    Formation,
    MustFly,
    AltitudeStackedMarshall,
    Launch,
    Recovery,
    RouteIntercept,
    Count
};
/// "hsa_csa", "waypoint_following", ...; "none".
FSIM_API const char* flightModeName(FlightMode mode) noexcept;

/// What a capability's command is (CapabilityDescriptor::setpoint;
/// docs/vehicle-interface.md 4.2): a level's struct, a behaviour's parameters
/// (heap data: no UPDATE), or a guidance mode's fixed-size setpoint (UPDATE
/// writes it in place).
enum class SetpointKind : std::uint8_t {
    Level,    ///< a level's struct (ActuatorCommand ... PositionCommand), or a support effector's
    Behavior, ///< BehaviorCommand
    Hsa,      ///< HsaCommand (fsim.guidance.hsa)
    Route,    ///< RouteCommand and its waypoints (fsim.guidance.route)
    Pattern,  ///< PatternCommand (fsim.guidance.pattern)
    Curve,    ///< CurveCommand and its segments (fsim.guidance.curve)
    Count
};

/// What an aircraft can do, as its guidance plans with it and a consumer
/// reads it (A-GRA's flight capability performance profile;
/// docs/vehicle-interface.md 7.1). Its family's adapter works it out from
/// the profile and the loops the vehicle flies with; NaN where it is not known.
struct Performance {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    std::uint32_t revision = 0;      ///< counts recomputations
    bool hovers = false;             ///< a rotorcraft: holds a point, flies any direction over the ground
    // speeds
    double minCasMs = kNone;         ///< calibrated: the envelope's minimum, else 1.2 times the stall speed
    double maxCasMs = kNone;         ///< calibrated: the envelope's maximum
    double maxMach = kNone;
    double maxTasMs = kNone;         ///< the fastest it flies (its performance section's, level at full power)
    double cruiseTasMs = kNone;      ///< what a mode flies given no speed: a wing's reference airspeed, a rotorcraft's half its fastest
    double maxGroundSpeedMs = kNone; ///< a rotorcraft's fastest over the ground (its position loop's)
    // altitude
    double ceilingM = kNone;
    // attitude, rates, accelerations
    double maxBankRad = kNone, minPitchRad = kNone, maxPitchRad = kNone, maxRollRateRadS = kNone;
    double minLoadFactor = kNone, maxLoadFactor = kNone;
    double maxTiltRad = kNone;           ///< a rotorcraft's most tilt to accelerate
    double maxAccelerationMs2 = kNone;   ///< a rotorcraft's over the ground: g tan(maxTilt)
    double maxDecelerationMs2 = kNone;   ///< a rotorcraft's, as its position loop stops
    double maxClimbMs = kNone, maxDescentMs = kNone; ///< the vertical speeds guidance asks for
    // how fast it answers guidance
    double altitudeGainPerS = kNone;     ///< vertical speed per metre of altitude error (its position loop's)
    double headingGain = kNone;          ///< a wing's heading loop: rad of bank per rad of heading error...
    double headingReferenceTasMs = kNone; ///< ...at this true airspeed (it grows with speed); NaN: the same at every speed
    double bankRateRadS = kNone;         ///< a wing's attitude loop: the fastest it changes the bank (a turn is anticipated by it)
    double velocityBandwidthRadS = kNone; ///< a rotorcraft's velocity loop: how fast its ground velocity follows a demand

    /// How fast the track follows a course demand at true airspeed `tasMs`,
    /// rad/s: a wing's heading loop (g times its gain over the speed it holds
    /// at), a rotorcraft's velocity loop. A guess of 0.2 where neither is known.
    double courseBandwidthRadS(double tasMs) const noexcept;
    /// The radius of a turn at ground speed `speedMs` with the bank or tilt
    /// guidance may use (80 % of the most), m.
    double turnRadiusM(double speedMs) const noexcept;
};

/// One parameter: a field of the capability's command struct, in order, or a
/// behaviour's named parameter.
struct ParameterInfo {
    std::string name;  ///< "roll_rad", "radius_m"
    std::string unit;  ///< "rad", "m/s", "" for a ratio
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();
    double defaultValue = std::numeric_limits<double>::quiet_NaN();
    bool optional = true; ///< accepts kHold (NaN)
    /// The performance limits a value below `min` or above `max` breaks
    /// (CommandResult::constraint): a bank's MaxOrientation, an airspeed's
    /// MinAirspeed and MaxAirspeed. None for a range that is not the aircraft's.
    Constraint below = Constraint::None, above = Constraint::None;
    /// false: this aircraft has nothing the field could move (a rotorcraft's
    /// longitudinal acceleration, a wing's pitch rate). A command that sets
    /// it - to anything but kHold or its default - is refused (InvalidParameter).
    bool supported = true;
};

struct CapabilityDescriptor {
    std::string id;                        ///< "<namespace>.<domain>.<name>", e.g. "fsim.flight.attitude"
    std::uint16_t version = 1;
    CapabilityKind kind = CapabilityKind::Flight;
    std::uint8_t interactions = 0;         ///< Interaction bits
    Level level{};                         ///< where its commands enter the cascade
    AxisMask axes = 0;                     ///< the axes a command owns by default
    std::uint8_t axisGroups = 0;           ///< AxisGroup bits it may own apart (CommandOptions::axes); 0 = all or nothing
    Persistence persistence = Persistence::Persistent;
    std::vector<ParameterInfo> parameters;
    std::vector<std::string> uses;         ///< the capabilities it flies through
    std::string behavior;                  ///< guidance: the behaviour's registry id
    bool needsTarget = false;              ///< guidance: follows BehaviorCommand::target
    FlightMode mode = FlightMode::None;    ///< the A-GRA flight capability type it is
    SetpointKind setpoint = SetpointKind::Level; ///< what its command is
    std::uint8_t accepted = 0;             ///< AcceptedInterface bits
    /// A platform behaviour an A-GRA capability does better, that one's id
    /// (hold: "fsim.guidance.hsa"); it stays, and works. "" for the others.
    std::string superseded;
};

/// What a behaviour declares when it is registered (ControllerRegistry::addBehavior).
struct BehaviorTraits {
    Persistence persistence = Persistence::Persistent;
    std::vector<ParameterInfo> parameters; ///< its BehaviorCommand::params, for discovery
    std::vector<std::string> uses;         ///< what it flies through, e.g. "fsim.flight.velocity"
    bool needsTarget = false;
    /// The Feature bits an aircraft needs for it to be offered (a hover needs
    /// kFeatureHover); 0: every aircraft.
    std::uint32_t features = 0;
    FlightMode mode = FlightMode::None; ///< the A-GRA flight capability type it is
    /// Behavior: parameters in a BehaviorCommand. A guidance mode's fixed-size
    /// setpoint (SetpointKind::Hsa, ...) makes it a mode: it takes UPDATE.
    SetpointKind setpoint = SetpointKind::Behavior;
};

// --- Support (docs/flight-autonomy.md, 4.1, 4.2 and 5) --------------------------------

/// Whether an aircraft, in this build, can do something at all. It does not
/// change while the vehicle flies; whether it can be commanded now is its
/// availability (CapabilityStatus).
enum class Support : std::uint8_t {
    Supported,      ///< built, and offered on this aircraft
    Partial,        ///< built and offered; SupportInfo::missing names what is not, `stage` the stage that completes it
    NotImplemented, ///< applicable to this aircraft, not built yet: `stage` builds it
    NotSupported,   ///< a physical exception: `rules` exclude it, `evidence` is the aircraft's
};
/// "supported", "partial", "not_implemented", "not_supported".
FSIM_API const char* supportName(Support support) noexcept;

/// The applicability rules (docs/flight-autonomy.md, 5.1): what a physical
/// exception rests on. SupportInfo::rules holds them as bits (ruleBit).
enum class Rule : std::uint8_t {
    None = 0,
    Hover,            ///< R1: holding a point in the air, on rotors
    GroundTaxi,       ///< R2: taxiing, on wheels
    CatapultArrested, ///< R3: a catapult launch and an arrested landing, carrier-capable wings
    DeckOperations,   ///< R4: a moving deck's vertical take-off and landing
    ArresterHook,     ///< R5: the arrester hook, as R3
    RetractableGear,  ///< R6
    Flaps,            ///< R7: a flap function
    DragDevices,      ///< R8: spoilers, airbrakes, or surfaces deployed as a speedbrake
    ReleasableStores, ///< R9: stores it carries and releases
    Aerobatic,        ///< R10: a wing cleared for aerobatic manoeuvres
    WheelBrakes,      ///< R11: wheel brakes, on wheels
    PitchTrim,        ///< R12: a pitch trim the controls move (not a fly-by-wire law's, not a rotorcraft's)
    EngineThrottles,  ///< R13: a throttle per engine, with more than one the pilot moves apart
    Count
};
constexpr std::uint16_t ruleBit(Rule rule) noexcept { return static_cast<std::uint16_t>(1u << static_cast<unsigned>(rule)); }
/// "R1" ... "R13"; "" for None.
FSIM_API const char* ruleName(Rule rule) noexcept;
/// What it says: "holding a point in the air applies to aircraft that fly on rotors", ...
FSIM_API const char* ruleDescription(Rule rule) noexcept;

/// One public feature's support on a vehicle. Its strings live as long as the vehicle.
struct SupportInfo {
    const char* feature = "";    ///< its public identifier: "fsim.guidance.hover", "fsim.guidance.hsa/direction/magnetic_north"
    Support support = Support::NotImplemented;
    std::uint16_t rules = 0;     ///< NotSupported: the rules that exclude it; else those that govern it (ruleBit)
    std::uint8_t stage = 0;      ///< Partial, NotImplemented: the stage that builds it (FA-n: n); 0 none planned
    const char* capability = ""; ///< the capability that carries it ("" for the command interface's own)
    const char* missing = "";    ///< Partial: what is not built yet
    const char* evidence = "";   ///< NotSupported: the aircraft's declarations the rules rest on, each with its source
};

/// Every public feature identifier, in the order a vehicle's support table
/// lists them. An identifier, once published, never changes meaning.
FSIM_API std::size_t supportFeatureCount() noexcept;
FSIM_API const char* supportFeature(std::size_t index) noexcept;

} // namespace fsim::control
