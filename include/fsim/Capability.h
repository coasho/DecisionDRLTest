#pragma once

// Capability contracts (design 9.3, ADR-26, docs/control-architecture.md):
// what a vehicle offers (capabilities), how a command is answered (NEW,
// UPDATE, CANCEL with a synchronous result) and what it becomes (an activity
// that is pending, active, then completed, failed or canceled).

#include "fsim/Export.h"

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
    Count
};

/// "authority_held", "goal_reached", ...
FSIM_API const char* reasonName(Reason reason) noexcept;

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
    bool live() const noexcept { return state == ActivityState::Pending || state == ActivityState::Active; }
};

// --- Capabilities -------------------------------------------------------------------

enum class Availability : std::uint8_t {
    Available,
    TemporarilyUnavailable, ///< a condition that will pass (weight on wheels, a placard speed, a diverged vehicle)
    Faulted,                ///< an effector has failed
    Disabled,               ///< switched off for this vehicle
};

struct CapabilityStatus {
    Availability availability = Availability::Available;
    Reason reason = Reason::None;
};

enum class CapabilityKind : std::uint8_t { Flight, Guidance, Support, Status };
enum class Persistence : std::uint8_t {
    Persistent,  ///< runs until canceled, preempted or failed (a hold)
    Terminating, ///< completes when it reaches its goal (a route, a manoeuvre)
};
enum Interaction : std::uint8_t { kCommand = 1u << 0, kUpdate = 1u << 1, kCancel = 1u << 2, kSettings = 1u << 3, kStatus = 1u << 4 };

/// One parameter: a field of the capability's command struct, in order, or a
/// behaviour's named parameter.
struct ParameterInfo {
    std::string name;  ///< "roll_rad", "radius_m"
    std::string unit;  ///< "rad", "m/s", "" for a ratio
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();
    double defaultValue = std::numeric_limits<double>::quiet_NaN();
    bool optional = true; ///< accepts kHold (NaN)
};

struct CapabilityDescriptor {
    std::string id;                        ///< "<namespace>.<domain>.<name>", e.g. "fsim.flight.attitude"
    std::uint16_t version = 1;
    CapabilityKind kind = CapabilityKind::Flight;
    std::uint8_t interactions = 0;         ///< Interaction bits
    Level level{};                         ///< where its commands enter the cascade
    AxisMask axes = 0;                     ///< the axes a command owns by default
    Persistence persistence = Persistence::Persistent;
    std::vector<ParameterInfo> parameters;
    std::vector<std::string> uses;         ///< the capabilities it flies through
    std::string behavior;                  ///< guidance: the behaviour's registry id
    bool needsTarget = false;              ///< guidance: follows BehaviorCommand::target
};

/// What a behaviour declares when it is registered (ControllerRegistry::addBehavior).
struct BehaviorTraits {
    Persistence persistence = Persistence::Persistent;
    std::vector<ParameterInfo> parameters; ///< its BehaviorCommand::params, for discovery
    std::vector<std::string> uses;         ///< what it flies through, e.g. "fsim.flight.velocity"
    bool needsTarget = false;
};

} // namespace fsim::control
