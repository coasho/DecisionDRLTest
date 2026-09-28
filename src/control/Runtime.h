#pragma once

// The boundary between the contract layer and the control runtime
// (docs/control-architecture.md, section 6): the only two structs they
// share. The host writes RuntimeConfig between steps and the runtime reads
// it during them; the runtime writes RuntimeReport during steps and the host
// reads and clears it after each world step. Both are per vehicle, fixed in
// size and held by the vehicle's ControlStack.

#include "fsim/Altimeter.h"
#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/VehicleProfile.h"

#include <array>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>

namespace fsim::control {

/// `dst = src` for two commands of the same level below Behavior, or of the
/// same guidance mode, without std::variant's general assignment: the per-step
/// setpoint write.
inline void assignSetpoint(Command& dst, const Command& src) noexcept {
    auto as = [&](auto tag) {
        using T = decltype(tag);
        auto* d = std::get_if<T>(&dst);
        const auto* s = std::get_if<T>(&src);
        if (d && s) *d = *s;
    };
    switch (src.index()) {
    case 0: as(ActuatorCommand{}); break;
    case 1: as(AttitudeCommand{}); break;
    case 2: as(AccelerationCommand{}); break;
    case 3: as(VelocityCommand{}); break;
    case 4: as(PositionCommand{}); break;
    case 6: as(HsaCommand{}); break;
    case 7: as(RouteCommand{}); break;
    case 8: as(PatternCommand{}); break;
    case 9: as(CurveCommand{}); break;
    default: break;
    }
}

/// A speed given replaces a speed optimisation, and an optimisation a speed
/// (given both, the optimisation's: it is the speed a mode varies by itself).
inline void mergeSpeed(double& speed, double& optimization, double givenSpeed, double givenOptimization) noexcept {
    if (!isHold(givenSpeed)) speed = givenSpeed, optimization = kHold;
    if (!isHold(givenOptimization)) optimization = givenOptimization, speed = kHold;
}

/// A partial HSA (docs/vehicle-interface.md, 4.4): the fields given replace
/// the commanded ones, the rest stay; a heading replaces a course, a course a
/// heading; a speed a speed optimisation, an optimisation a speed.
inline void mergeHsa(HsaCommand& dst, const HsaCommand& src) noexcept {
    if (!isHold(src.headingRad)) dst.headingRad = src.headingRad, dst.courseRad = kHold;
    if (!isHold(src.courseRad)) dst.courseRad = src.courseRad, dst.headingRad = kHold;
    mergeSpeed(dst.speed, dst.speedOptimization, src.speed, src.speedOptimization);
    if (!isHold(src.speedReference)) dst.speedReference = src.speedReference;
    if (!isHold(src.altitudeM)) dst.altitudeM = src.altitudeM;
    if (!isHold(src.altitudeReference)) dst.altitudeReference = src.altitudeReference;
}

/// A partial pattern (docs/vehicle-interface.md, 4.6): each field given
/// replaces the commanded one; a speed a speed optimisation, an optimisation a speed.
inline void mergePattern(PatternCommand& dst, const PatternCommand& src) noexcept {
    double* d[] = {&dst.pattern, &dst.latitudeRad, &dst.longitudeRad, &dst.altitudeM, &dst.altitudeReference, &dst.radiusM,
                   &dst.clockwise, &dst.courseRad, &dst.legM, &dst.speedReference, &dst.durationS};
    const double s[] = {src.pattern, src.latitudeRad, src.longitudeRad, src.altitudeM, src.altitudeReference, src.radiusM,
                        src.clockwise, src.courseRad, src.legM, src.speedReference, src.durationS};
    for (std::size_t i = 0; i < std::size(s); ++i)
        if (!isHold(s[i])) *d[i] = s[i];
    mergeSpeed(dst.speed, dst.speedOptimization, src.speed, src.speedOptimization);
}

/// What UPDATE (and the existing entry points' per-step path) writes into a
/// slot: a level's setpoint replaced, a mode's merged (its kHold fields keep
/// what was commanded).
inline void applySetpoint(Command& dst, const Command& src) noexcept {
    if (auto* d = std::get_if<HsaCommand>(&dst))
        if (const auto* s = std::get_if<HsaCommand>(&src)) return mergeHsa(*d, *s);
    if (auto* d = std::get_if<PatternCommand>(&dst))
        if (const auto* s = std::get_if<PatternCommand>(&src)) return mergePattern(*d, *s);
    assignSetpoint(dst, src);
}

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
    double value = kHold;  ///< normalised position (the left brake); kHold = keep the last input
    double value2 = kHold; ///< the right brake
};

inline constexpr double kNoLimit = kUnknown;

/// The protection stage's configuration (section 11), from the profile: the
/// host writes it when the vehicle is created and when its mode changes.
struct Protection {
    ProtectionMode mode = ProtectionMode::Off;
    std::uint16_t lawEnforces = 0;   ///< limits (bit per Limit) the aircraft's own law enforces: no feedback limiter of ours on them
    EnvelopeLimits clean{}, flaps{}; ///< flaps: each limit the profile gives only for clean is clean's
    double flapsThreshold = 0.05;    ///< flaps commanded beyond this select `flaps`
    double gearCasMaxMs = kNoLimit;
    // for the feedback limiters
    double alphaZeroLiftRad = 0.0;   ///< the angle of attack of zero lift (the plant's, else 0)
    bool pitchSurface = false;       ///< the elevator input moves a surface (not a law's demand)
    double elevatorGainG = kNoLimit; ///< load factor per unit of elevator at elevatorGainCasMs (the plant's); NaN: no elevator limiter
    double elevatorGainCasMs = kNoLimit;
    /// It flies on a wing (kFeatureWingborne): its flight path follows its
    /// pitch attitude and it turns by banking, so a vertical speed is bounded
    /// by the pitch limits and a turn rate by the bank limit. A rotorcraft
    /// climbs level and turns with its yaw: neither is.
    bool wingborne = true;
};

struct RuntimeConfig {
    static constexpr std::size_t kSlots = kSlotCount;
    static constexpr std::uint8_t kNone = 0xFF;    ///< no owner: the vehicle default flies the axis
    static constexpr std::uint8_t kSupport = 0xFE; ///< a support activity: support[axis - Axis::Flaps]
    static constexpr std::uint8_t kEngines = 0xFD; ///< thrust set per engine: engines[]

    RuntimeConfig() noexcept { owner.fill(kNone); }

    std::array<SetpointSlot, kSlots> slots{};
    std::array<std::uint8_t, kAxisCount> owner{};  ///< per axis: a slot index, kSupport or kNone
    std::array<SupportDemand, kSupportAxisCount> support{};
    std::array<double, 4> engines{kHold, kHold, kHold, kHold}; ///< per-engine throttle when thrust's owner is kEngines
    VehicleDefault vehicleDefault = VehicleDefault::Neutral;  ///< what flies a primary axis whose owner is kNone
    /// What the vehicle can do, for its guidance modes (ControlContext::performance):
    /// the host writes it when it binds and when the vehicle's loops change.
    Performance performance{};
    /// The route a guidance mode flies (ControlContext::path): allocated at the
    /// vehicle's first route and kept; the host writes it between steps.
    std::unique_ptr<PathStore> path;
    /// Per primary axis: bumped each time it returns to the vehicle default
    /// (or the default becomes a hold), so the hold captures it afresh.
    std::array<std::uint32_t, kPrimaryAxisCount> letGo{};
    std::uint32_t revision = 0;                    ///< bumped whenever owner[] or a slot's axes change
    Protection protection{};
    /// Its performance tables (ControlContext::tables): the profile's, set
    /// when the host binds; null without them.
    const TablesSection* tables = nullptr;
    /// Its barometric altimeter (ControlContext::altimeter): the world's air
    /// and the QNH it is set to, written by the session between steps.
    Altimeter altimeter{};
};

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
    std::uint16_t flags = 0;       ///< ActivityFlag bits its behaviour reported (Behavior::constraints), latched likewise
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
