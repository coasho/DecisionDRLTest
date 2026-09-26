#pragma once

// Capability contracts (design 9.3, ADR-26, docs/control-architecture.md):
// the axes a command owns and the reasons a command or an activity reports.

#include "fsim/Export.h"

#include <cstddef>
#include <cstdint>

namespace fsim::control {

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

} // namespace fsim::control
