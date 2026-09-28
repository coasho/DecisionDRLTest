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

namespace fsim::sim {
struct VehicleState; // fsim/VehicleState.h
}

namespace fsim::control {

enum class Level : std::uint8_t; // fsim/Control.h
struct BehaviorCommand;          // fsim/Control.h

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
    // ranks, queues and time windows (docs/flight-autonomy.md, 4.9)
    TimeConstraint, ///< NEW rejected, or Failed: a time window it must meet cannot be met, or was missed
    QueueFull,      ///< NEW rejected: it would wait, and as many activities as can wait already do
    // activity commands (docs/flight-autonomy.md, 4.10)
    NotInteractive, ///< an activity command refused: its command said it takes none (CommandOptions::interactive)
    // flight tasks (docs/flight-autonomy.md, 4.11)
    UnknownTask,    ///< a task command refused: the vehicle keeps no task by that id
    TaskActive,     ///< a task command or store refused: the task's activity is live
    // endurance (docs/flight-autonomy.md, 4.18)
    InsufficientEndurance, ///< NEW refused - a soft rejection, which CommandOptions::overrideRejection overrides: its flight needs more fuel or charge than the vehicle has above its reserve
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

/// Which of a vehicle's policies commands (A-GRA's SystemServiceType: the
/// mission autonomy service that holds control; docs/flight-autonomy.md,
/// 4.12): each a name the caller gives, 0 the default policy. Under
/// ControlMode::Granted a grant is one controller's.
using ControllerId = std::uint32_t;

/// Who addresses an activity (UPDATE, CANCEL, an activity command): a source
/// and, a policy's, its controller. Made from a Source alone: the default policy's.
struct Caller {
    Source source = Source::Policy;
    ControllerId controller = 0;
    constexpr Caller(Source s = Source::Policy, ControllerId c = 0) noexcept : source(s), controller(c) {}
};

/// What happens to a value outside the capability's advertised range.
enum class RangePolicy : std::uint8_t {
    Clamp,  ///< clamp it; the result says so (kClamped)
    Reject, ///< reject the command (OutOfRange)
    None,   ///< no parameter or availability checks: the existing entry points' behaviour
};

/// What a requirement a command traces to is (A-GRA's RequirementInstanceID_Choice).
enum class RequirementKind : std::uint8_t { None, Effect, Action, Task, Command, Count };
/// "effect", "action", "task", "command"; "none".
FSIM_API const char* requirementKindName(RequirementKind kind) noexcept;

/// A requirement a command comes from (A-GRA's Traceability.Requirement;
/// docs/flight-autonomy.md, 4.8): the caller's ids, kept with the activity.
struct Requirement {
    RequirementKind kind = RequirementKind::None;
    std::uint64_t id = 0;
};
/// At most this many requirements a command traces to.
inline constexpr std::size_t kMaxRequirements = 4;

/// A command's rank among the others (A-GRA's ComparableRankingType;
/// docs/flight-autonomy.md, 4.9): lower first - its priority, then its
/// precedence within that priority. {0, 0}, every command's without one, ranks first.
struct Rank {
    std::uint16_t priority = 0;
    std::uint16_t precedence = 0;
    friend constexpr bool operator==(Rank a, Rank b) noexcept { return a.priority == b.priority && a.precedence == b.precedence; }
};
/// `a` ranks strictly ahead of `b`.
constexpr bool ranksAhead(Rank a, Rank b) noexcept { return a.priority != b.priority ? a.priority < b.priority : a.precedence < b.precedence; }

/// A command's capability precedence left as its capability's (CommandOptions::precedenceOverride).
inline constexpr std::uint32_t kNoPrecedenceOverride = 0xFFFFFFFFu;

/// Which of a command's time windows must be met for it to be of use (A-GRA's SchedulingCriticalityEnum).
enum class TimeCriticality : std::uint8_t { None, Start, End, StartAndEnd, Count };
/// "none", "start", "end", "start_and_end".
FSIM_API const char* timeCriticalityName(TimeCriticality criticality) noexcept;

/// When a command may start and should end (A-GRA's TemporalConstraints;
/// docs/flight-autonomy.md, 4.9), in simulation seconds; NaN: no bound. It
/// starts no earlier than startNotBefore; the rest, and what missing them
/// does, `criticality` says - checked after each world step.
struct TimeWindow {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double startNotBefore = kNone, startNotAfter = kNone;
    double endNotBefore = kNone, endNotAfter = kNone;
    TimeCriticality criticality = TimeCriticality::None;
    bool startCritical() const noexcept { return criticality == TimeCriticality::Start || criticality == TimeCriticality::StartAndEnd; }
    bool endCritical() const noexcept { return criticality == TimeCriticality::End || criticality == TimeCriticality::StartAndEnd; }
    /// Any bound given.
    bool any() const noexcept { return startNotBefore == startNotBefore || startNotAfter == startNotAfter || endNotBefore == endNotBefore || endNotAfter == endNotAfter; }
};

struct CommandOptions {
    Source source = Source::Policy;
    AxisMask axes = 0;                      ///< 0 = the capability's default axes
    RangePolicy range = RangePolicy::Clamp;
    std::uint16_t minVersion = 0;           ///< reject a capability older than this
    // The command envelope (docs/flight-autonomy.md, 4.8):
    /// The caller's id for this command (A-GRA's CommandID): echoed in its
    /// answers and kept with its activity. 0: none.
    std::uint64_t commandId = 0;
    /// The requirements it comes from (A-GRA's Traceability): kept with its activity.
    std::array<Requirement, kMaxRequirements> trace{};
    /// Whether its activity takes activity commands (A-GRA's Interactive).
    bool interactive = true;
    /// Check it as a NEW is checked and answer as a NEW would, flying nothing:
    /// Valid, or Rejected with every finding (A-GRA's validation; FLIGHT_COMMAND_VALID).
    bool validateOnly = false;
    // How it is arbitrated and scheduled (docs/flight-autonomy.md, 4.9):
    /// Whether it interrupts what flies on its axes (A-GRA's
    /// InterruptOtherActivities). A policy's that does takes them from what it
    /// ranks at or ahead of and waits for the rest; one that does not waits
    /// until they are free. The platform's own sources interrupt any rank, or,
    /// if not, defer to it. (A-GRA's omission is false: fsim.agra maps it.)
    bool interrupt = true;
    /// Fly it even where a soft rejection would refuse it (A-GRA's
    /// OverrideRejection); never over a safety limit. None exists yet: the
    /// endurance (FA-3) and air traffic (FA-15) checks bring them.
    bool overrideRejection = false;
    /// Its rank among the commands its axes are contested by (A-GRA's Ranking.Rank).
    Rank rank{};
    /// Its capability's precedence for this command alone (A-GRA's
    /// CapabilityPrecedenceOverride; lower first); kNoPrecedenceOverride: the
    /// capability's own. The platform's own sources only: a policy's is refused NotAllowed.
    std::uint32_t precedenceOverride = kNoPrecedenceOverride;
    /// A policy's controller (docs/flight-autonomy.md, 4.12): under
    /// ControlMode::Granted its capability's grant must be this controller's.
    ControllerId controller = 0;
    /// When it may start and should end (A-GRA's TemporalConstraints).
    TimeWindow window{};
};

enum class CommandStatus : std::uint8_t {
    Accepted,
    Rejected,
    Canceled,
    Valid, ///< CommandOptions::validateOnly: it would be accepted; nothing flies
};
enum CommandFlag : std::uint16_t {
    kClamped = 1u << 0,  ///< a value was clamped to what the aircraft can do
    kDeferred = 1u << 1, ///< accepted to wait: it starts when its start window opens and its axes are free (ActivityRecord::waiting)
    kOverridden = 1u << 2, ///< accepted over a soft rejection (CommandOptions::overrideRejection): CommandDetails::endurance says by how much
};

/// The synchronous answer to NEW, UPDATE and CANCEL. The reason in words
/// is reasonDescription(reason) (A-GRA's CannotComply description).
struct CommandResult {
    CommandStatus status = CommandStatus::Rejected;
    Reason reason = Reason::None;
    ActivityId activity = 0; ///< the new (NEW) or addressed (UPDATE, CANCEL) activity
    /// An id the reason is about (A-GRA's AssociatedID): the activity that
    /// holds the authority (AuthorityHeld), or that a deferred NEW waits for; 0 none.
    ActivityId other = 0;
    std::uint16_t flags = 0; ///< CommandFlag bits
    // What a rejection or a clamp was about (docs/vehicle-interface.md, 5.1):
    /// the field (in the command struct's order, or a behaviour's named
    /// parameter's place in its descriptor), route point or curve segment; -1: none
    std::int16_t index = -1;
    Constraint constraint = Constraint::None; ///< the performance limit its value broke
    /// A NEW that made an activity (A-GRA's NewActivity; docs/flight-autonomy.md,
    /// 4.8); false for an UPDATE, a CANCEL, a command the live activity takes
    /// (the existing entry points), a validation.
    bool newActivity = false;
    float from = std::numeric_limits<float>::quiet_NaN(); ///< a curve segment's section that breaks it: its parameter, 0..1
    float to = std::numeric_limits<float>::quiet_NaN();
    std::uint64_t commandId = 0; ///< the command's id (docs/flight-autonomy.md, 4.8): a NEW's, or the addressed activity's
    bool accepted() const noexcept { return status == CommandStatus::Accepted; }
};

/// One reason a command cannot be flown as asked (A-GRA's ValidationResult and
/// its reason's detail; docs/flight-autonomy.md, 4.8), as CommandResult's first.
struct Finding {
    Reason reason = Reason::None;
    std::int16_t index = -1;                  ///< the field, route point or curve segment
    Constraint constraint = Constraint::None; ///< the performance limit its value breaks
    float from = std::numeric_limits<float>::quiet_NaN(); ///< a curve segment's section: its parameter, 0..1
    float to = std::numeric_limits<float>::quiet_NaN();
    std::uint64_t associated = 0;             ///< an id it is about
    const char* description = "";            ///< in words
};

/// A value a command was flown with other than it asked (A-GRA's "accepted
/// with less than optimum results"): clamped to what the aircraft can do.
struct Adjustment {
    std::int16_t index = -1;                  ///< the command's field, a route point, a curve segment
    std::int16_t field = -1;                  ///< a route point's field (fsim/Control.h Waypoint's order); -1 none
    Constraint constraint = Constraint::None; ///< the limit it was held to
    double requested = std::numeric_limits<double>::quiet_NaN(); ///< NaN where it is not one number (a turn flown smaller)
    double adjusted = std::numeric_limits<double>::quiet_NaN();
};

/// Everything the checks of a command's answer found (docs/flight-autonomy.md,
/// 4.8): every reason it cannot be flown as asked, the first of them the
/// answer's reason, and every value it is flown with other than it asked.
struct CommandDetails {
    static constexpr std::size_t kMax = 16;
    std::uint8_t findingCount = 0, adjustmentCount = 0; ///< (beyond kMax: counted, not kept)
    /// A refused command's suggestion (docs/flight-autonomy.md, 4.11): the
    /// task the platform keeps with what it can fly in its place - every value
    /// held to the aircraft's limits (A-GRA's best-effort MA_TaskMT); 0 none.
    std::uint64_t suggestion = 0;
    std::array<Finding, kMax> findings{};
    std::array<Adjustment, kMax> adjustments{};
    /// What a flight with an end needs against what the vehicle has
    /// (docs/flight-autonomy.md, 4.18; A-GRA's MA_InsufficientEnduranceType):
    /// set when it needs more - refused InsufficientEndurance, or accepted
    /// over it (kOverridden); `energy` 0 otherwise.
    struct Endurance {
        std::uint8_t energy = 0;  ///< what it is counted in (Energy, fsim/Control.h): 1 fuel (kg), 2 a battery's charge (J)
        double remaining = std::numeric_limits<double>::quiet_NaN();  ///< what the vehicle has above its reserve
        double required = std::numeric_limits<double>::quiet_NaN();   ///< what the flight needs
        double remainingS = std::numeric_limits<double>::quiet_NaN(); ///< how long `remaining` lasts at what it consumes now
        double requiredS = std::numeric_limits<double>::quiet_NaN();  ///< how long the flight takes
    } endurance{};
    void clear() noexcept { findingCount = adjustmentCount = 0, suggestion = 0, endurance.energy = 0; } // (the rest means nothing without it)
};

// --- Flight tasks (docs/flight-autonomy.md, 4.11) --------------------------------------

/// A flight task's id (A-GRA's TaskID): the caller's own, or - kSuggestedTask
/// set - a suggestion the platform made.
using TaskId = std::uint64_t;
inline constexpr TaskId kSuggestedTask = TaskId{1} << 63;

/// How often a task flies (A-GRA's finite repetition): `attempts` runs in
/// all, each after the one before completes, `intervalS` later (NaN: at
/// once). Its one activity stays active between runs, flying what its
/// behaviour flies when done. A terminating capability's only.
struct TaskRepetition {
    std::uint32_t attempts = 1;
    double intervalS = std::numeric_limits<double>::quiet_NaN();
};

/// A task's execution state (A-GRA's RequirementExecutionStateEnum).
enum class TaskState : std::uint8_t {
    AwaitingExecution, ///< kept, not commanded (or commanded and refused)
    ExecutionPending,  ///< commanded: its activity waits to start, has not flown yet, or is disabled
    Executing,         ///< its activity flies
    Completed,         ///< its activity completed, every run
    Dropped,           ///< its activity lost its axes or its authority (preempted, revoked, released, ...)
    Failed,            ///< its activity failed
    Canceled,          ///< canceled, or its activity canceled or deleted on request
    Count
};
/// "awaiting_execution", "execution_pending", "executing", "completed", "dropped", "failed", "canceled".
FSIM_API const char* taskStateName(TaskState state) noexcept;

enum class EndPointKind : std::uint8_t; // fsim/Control.h
/// "waypoint", "turn_point", "loiter_point".
FSIM_API const char* endPointKindName(EndPointKind kind) noexcept;

/// A task's status (A-GRA's TaskStatus).
struct TaskStatus {
    TaskId id = 0;
    TaskState state = TaskState::AwaitingExecution;
    Reason reason = Reason::None;          ///< why it failed, was dropped or canceled: its activity's end
    bool suggested = false;                ///< the platform's: what it can fly in place of a command it refused
    ActivityId activity = 0;               ///< its activity (every run's); 0 before it is commanded
    std::uint32_t run = 0, runs = 0;       ///< the run flying or flown last, of how many
    double percent = std::numeric_limits<double>::quiet_NaN(); ///< of the whole task, 0..100 (A-GRA's PercentCompleted)
    double startTime = std::numeric_limits<double>::quiet_NaN(); ///< when it was commanded
    double endTime = std::numeric_limits<double>::quiet_NaN();   ///< when its activity ended
    std::uint64_t commandId = 0;           ///< its task command's id
};

// --- Activities ---------------------------------------------------------------------

/// An activity's state. Disabled (docs/flight-autonomy.md, 4.10) is live - it
/// can be enabled - and flies nothing; Deleted is a sticky disable, ended.
enum class ActivityState : std::uint8_t { Pending, Active, Completed, Failed, Canceled, Disabled, Deleted };
/// "pending", "active", ..., "disabled", "deleted"
FSIM_API const char* activityStateName(ActivityState state) noexcept;

/// What an activity command asks of a live activity (A-GRA's
/// ActivityCommandBaseType; docs/flight-autonomy.md, 4.10).
enum class ActivityCommand : std::uint8_t {
    Disable,    ///< it stops flying and is kept (Disabled); its axes are free
    Enable,     ///< a disabled one waits to start again (from where it was: a route, the point it flew to)
    Reset,      ///< it starts over from its beginning
    Delete,     ///< a sticky disable: it ends Deleted, and cannot be enabled
    ChangeRank, ///< its rank changes: what it contests is arbitrated afresh
    Unassign,   ///< it gives up its axes and waits for them again, behind what waits
    Count
};
/// "disable", "enable", "reset", "delete", "change_rank", "unassign".
FSIM_API const char* activityCommandName(ActivityCommand command) noexcept;

/// Why a pending activity has not started (docs/flight-autonomy.md, 4.9).
enum class ActivityWait : std::uint8_t {
    None,      ///< it starts at the next world step, or has started
    Scheduled, ///< its start window has not opened
    Queued,    ///< its axes are held by what it may not interrupt (ActivityRecord::waitingFor)
    Count
};
/// "none", "scheduled", "queued".
FSIM_API const char* activityWaitName(ActivityWait wait) noexcept;

/// What an activity's record rests on (A-GRA's ActivityBasisEnum): flown, or
/// planned - one waiting to start. The platform senses and predicts no other's.
enum class ActivityBasis : std::uint8_t { Actual, Sensed, Predicted, Planned, Count };
/// "actual", "sensed", "predicted", "planned".
FSIM_API const char* activityBasisName(ActivityBasis basis) noexcept;

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
    // the command it came from (docs/flight-autonomy.md, 4.8)
    std::uint64_t commandId = 0;       ///< the caller's id for it
    std::array<Requirement, kMaxRequirements> trace{}; ///< the requirements it comes from
    bool interactive = true;           ///< it takes activity commands
    // how it is arbitrated and scheduled (docs/flight-autonomy.md, 4.9)
    bool interrupt = true;             ///< its command's CommandOptions::interrupt
    ActivityWait waiting = ActivityWait::None; ///< why it has not started, while pending
    std::uint16_t run = 0, runs = 0;   ///< a task's repetition (4.11): the run flying, of how many; 0, 0 none
    Rank rank{};                       ///< A-GRA's ActivityRank
    std::uint32_t precedence = 0;      ///< its capability's precedence it is arbitrated by (its command's override, else the capability's)
    ActivityId waitingFor = 0;         ///< Queued: an activity on its axes it may not interrupt
    TimeWindow window{};
    AxisMask axes = 0;                 ///< the axes it owns (or owned, once ended; will own, while it waits)
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;      ///< why it ended, else None
    ControllerId controller = 0;       ///< its command's (a policy's controller; docs/flight-autonomy.md, 4.12)
    ActivityId by = 0;                 ///< the preempting activity, with Preempted
    std::uint64_t suggestion = 0;      ///< Failed as it would start: the task the platform suggests in its place (4.11), else 0
    std::uint16_t constraints = 0;     ///< ActivityFlag bits of the last world step
    std::uint16_t constraintsSeen = 0; ///< every ActivityFlag bit since it started
    double startTime = 0.0;            ///< simulation time
    double endTime = std::numeric_limits<double>::quiet_NaN(); ///< NaN while live
    ActivityProgress progress{};       ///< as its behaviour reported it after the last world step (a mode's)
    bool live() const noexcept { return state == ActivityState::Pending || state == ActivityState::Active || state == ActivityState::Disabled; }
    ActivityBasis basis() const noexcept { return waiting == ActivityWait::None ? ActivityBasis::Actual : ActivityBasis::Planned; }
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
    ControllerId holder = 0; ///< the controller whose grant it is, while granted (docs/flight-autonomy.md, 4.12)
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
    /// guidance may use (80 % of the most), m; a rotorcraft's no tighter than
    /// its velocity loop follows (a turn rate a third of its bandwidth).
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
    /// What a checked NEW (RangePolicy Clamp or Reject) must also meet, from
    /// where the aircraft is: None, or the refusal with its detail (the
    /// point, the limit a value breaks) - an aerobatic manoeuvre's entry
    /// speed, a route point the aircraft cannot capture
    /// (docs/flight-autonomy.md, section 6). Null: nothing beyond the ranges.
    using Admission = Reason (*)(const BehaviorCommand& command, const sim::VehicleState& state, const Performance& performance,
                                 CommandResult& detail);
    Admission admit = nullptr;
    /// Completes only if flown within the envelope: a limit exceeded on its
    /// axes while it flew (kActivityExceeded, envelope protection's report)
    /// fails it (BehaviorFailed) when it finishes - an aerobatic manoeuvre.
    bool withinEnvelope = false;
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
