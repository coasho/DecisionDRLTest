#pragma once

// The boundary between the contract layer and the control runtime
// (docs/control-architecture.md, section 6): the only two structs they
// share. The host writes RuntimeConfig between steps and the runtime reads
// it during them; the runtime writes RuntimeReport during steps and the host
// reads and clears it after each world step. Both are per vehicle, fixed in
// size and held by the vehicle's ControlStack.

#include "fsim/Capability.h"
#include "fsim/Control.h"

#include <array>
#include <cstdint>
#include <limits>

namespace fsim::control {

/// One engaged activity's input to the cascade.
struct SetpointSlot {
    std::uint32_t generation = 0;  ///< bumped when the slot is given to a new activity: per-slot runtime state restarts
    std::uint32_t revision = 0;    ///< bumped on every write
    Level level = Level::Actuator; ///< where the setpoint enters the cascade
    AxisMask axes = 0;             ///< the axes it drives; 0 = the slot is free
    Command command;               ///< the setpoint, a command at `level` (a behaviour's parameters at Level::Behavior)
};

/// A support axis owned by a support activity (gear, flaps, ...).
struct SupportDemand {
    std::uint32_t revision = 0;
    double value = kHold;          ///< normalised position; kHold = keep the last input
};

inline constexpr double kNoLimit = std::numeric_limits<double>::quiet_NaN();

/// One configuration's limits (section 11); NaN = no limit.
struct EnvelopeLimits {
    double loadFactorMin = kNoLimit, loadFactorMax = kNoLimit; ///< g
    double alphaMaxRad = kNoLimit;
    double bankMaxRad = kNoLimit, pitchMinRad = kNoLimit, pitchMaxRad = kNoLimit;
    double rollRateMaxRadS = kNoLimit;
    double casMinMs = kNoLimit, casMaxMs = kNoLimit;           ///< calibrated airspeed
    double machMax = kNoLimit;
};

enum class ProtectionMode : std::uint8_t { Off, Report, Limit };

struct Protection {
    ProtectionMode mode = ProtectionMode::Off;
    std::uint16_t lawEnforces = 0; ///< limits (bit per Limit) the aircraft's own law already enforces
    EnvelopeLimits clean{}, flaps{};
    double flapsThreshold = 0.05;  ///< flaps beyond this select `flaps`
    double gearCasMaxMs = kNoLimit;
};

struct RuntimeConfig {
    static constexpr std::size_t kSlots = kSlotCount;
    static constexpr std::uint8_t kNone = 0xFF;    ///< no owner: the vehicle default flies the axis
    static constexpr std::uint8_t kSupport = 0xFE; ///< a support activity: support[axis - Axis::Flaps]

    RuntimeConfig() noexcept { owner.fill(kNone); }

    std::array<SetpointSlot, kSlots> slots{};
    std::array<std::uint8_t, kAxisCount> owner{};  ///< per axis: a slot index, kSupport or kNone
    std::array<SupportDemand, kSupportAxisCount> support{};
    std::uint32_t revision = 0;                    ///< bumped whenever owner[] or a slot's axes change
    Protection protection{};
};

enum class Limit : std::uint8_t { LoadFactorMax, LoadFactorMin, AlphaMax, Bank, PitchMax, PitchMin, RollRate, CasMin, CasMax, Mach, Count };
inline constexpr std::size_t kLimitCount = static_cast<std::size_t>(Limit::Count);

enum AxisFlag : std::uint16_t {
    kSaturated = 1u << 0,     ///< an actuator output sat at its travel limit
    kDemandLimited = 1u << 1, ///< the protection stage reduced the demand on this axis
    kExceeded = 1u << 2,      ///< the state was beyond a limit this axis flies against
};

enum SlotEvent : std::uint8_t { kFinished = 1u << 0, kFailed = 1u << 1 };

struct SlotReport {
    std::uint32_t generation = 0;  ///< the slot generation the runtime last flew
    std::uint32_t revision = 0;    ///< the setpoint revision it last flew
    std::uint8_t events = 0;       ///< SlotEvent bits, latched until the host clears them
    Reason failure = Reason::None; ///< with kFailed
};

struct LimitReport {
    std::uint32_t limitedUpdates = 0;  ///< updates in which this limit reduced the demand
    std::uint32_t exceededUpdates = 0; ///< updates in which the state was beyond it
    float worstExcess = 0.0f;          ///< the largest excess, in the limit's unit
};

struct RuntimeReport {
    std::array<SlotReport, RuntimeConfig::kSlots> slots{};
    std::array<std::uint16_t, kAxisCount> axisFlags{}; ///< OR of AxisFlag over the updates
    std::array<LimitReport, kLimitCount> limits{};
    std::uint32_t updates = 0;                         ///< control updates run
    std::uint32_t errors = 0;                          ///< cascade errors: a controller's invalid output, a missing controller

    /// What the host does after reading: everything but unprocessed events.
    void clearAccumulators() noexcept {
        axisFlags.fill(0);
        limits.fill(LimitReport{});
        updates = 0;
        errors = 0;
    }
};

} // namespace fsim::control
