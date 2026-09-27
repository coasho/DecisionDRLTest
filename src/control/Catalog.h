#pragma once

// The capabilities a vehicle offers (docs/control-architecture.md, section
// 8): one descriptor per flight level and per registered behaviour. Built
// between steps and only ever extended, so an index into it never changes.

#include "control/Checks.h"
#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/VehicleProfile.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::control {

/// The most fields a command struct or a guidance mode's setpoint has (a pattern's 12).
inline constexpr std::size_t kMaxCommandFields = 16;
/// The fields of a command struct below Level::Behavior, or of a guidance
/// mode's setpoint, in declaration order (the C ABI's order too): pointers
/// into `c`. Returns how many (0 for a BehaviorCommand).
std::size_t commandFields(Command& c, double* fields[kMaxCommandFields]) noexcept;
/// How many of a level's fields it had before the rotorcraft's were appended
/// (docs/rotorcraft.md, 3.4): what the C ABI's fixed-size calls still take.
std::size_t legacyFieldCount(Level level) noexcept;

// Support effectors (docs/control-architecture.md, 8.2), by SupportCommand's alternative.
inline constexpr std::size_t kSupportKinds = std::variant_size_v<SupportCommand>;
/// "fsim.support.gear", "fsim.support.flaps", "fsim.support.wheel_brakes", "fsim.support.speedbrake",
/// "fsim.support.pitch_trim", and "fsim.flight.engines" (a flight capability set beside the cascade like them).
const char* supportCapability(std::size_t alternative) noexcept;
Axis supportAxis(const SupportCommand& c) noexcept;
/// Its demand: the one value, or the left and right brake.
void supportValues(const SupportCommand& c, double& value, double& value2) noexcept;
/// Where a terminating one is going: gear 1 down or 0 up, a flap position; NaN for the others.
double supportGoal(const SupportCommand& c) noexcept;
/// Its fields, as commandFields does; at most 4.
std::size_t supportFields(SupportCommand& c, double* fields[4]) noexcept;

class VehicleAdapter;

class CapabilityCatalog {
public:
    /// The five flight capabilities with the loops' own ranges, and every behaviour registered so far.
    CapabilityCatalog();
    /// For an aircraft: what its adapter declares for its profile, and the
    /// behaviours its features allow.
    CapabilityCatalog(const VehicleProfile& profile, const VehicleAdapter& adapter);

    /// Intersect a parameter's range with [lo, hi] (a NaN bound leaves that side).
    void narrow(std::string_view capability, std::string_view parameter, double lo, double hi);
    /// Give a parameter the name the aircraft's own controls have (a
    /// helicopter's aileron is its lateral cyclic); its place does not change.
    void rename(std::string_view capability, std::string_view parameter, std::string name);
    /// Mark a parameter as one this aircraft has nothing for (ParameterInfo::supported).
    void unsupport(std::string_view capability, std::string_view parameter);
    /// What the flight capabilities may own apart (AxisGroup bits): the
    /// family's groups above the actuators, and each axis alone at them.
    void setAxisGroups(std::uint8_t groups);
    /// Offer a support effector (an adapter's declare()), by SupportCommand's
    /// alternative; for the engines' throttles, a parameter per engine (at
    /// most 4), owning `axes` (thrust, or every primary axis where the
    /// engines fly the aircraft).
    void addSupport(std::size_t alternative, int engines = 0, AxisMask axes = 0);
    /// Offer fsim.envelope.protection: the aircraft has an envelope.
    void addProtection();
    /// The Feature bits the behaviours offered were chosen by.
    std::uint32_t features() const noexcept { return features_; }

    /// Add the behaviours registered since; true if there were any. Between steps only.
    bool refresh();

    std::size_t size() const noexcept { return descriptors_.size(); }
    const CapabilityDescriptor& descriptor(std::size_t index) const noexcept { return descriptors_[index]; }
    const std::vector<CapabilityDescriptor>& descriptors() const noexcept { return descriptors_; }

    /// The capability a command selects (its level, its behaviour's id, or its mode); -1 if none.
    int indexOf(const Command& command) const noexcept;
    /// A support effector's capability; -1 if the aircraft has none.
    int indexOf(const SupportCommand& command) const noexcept { return bySupport_[command.index()]; }
    /// The SupportCommand alternative a capability sets; -1 if it is not a support effector's.
    int supportKindOf(std::size_t index) const noexcept {
        for (std::size_t k = 0; k < kSupportKinds; ++k)
            if (bySupport_[k] == static_cast<int>(index)) return static_cast<int>(k);
        return -1;
    }
    /// By capability id ("fsim.guidance.hold") or a behaviour's registry id ("hold"); -1 if none.
    int find(std::string_view id) const noexcept;
    /// Whether a behaviour's activity completes only if flown within the envelope (BehaviorTraits::withinEnvelope).
    bool withinEnvelope(std::size_t index) const noexcept {
        return std::find(withinEnvelope_.begin(), withinEnvelope_.end(), index) != withinEnvelope_.end();
    }
    /// A behaviour's admission (BehaviorTraits::admit) by its capability's index; null if none.
    BehaviorTraits::Admission admission(std::size_t index) const noexcept {
        for (const auto& [i, admit] : admissions_)
            if (i == index) return admit;
        return nullptr;
    }

    /// The axes a command owns when its options name none.
    AxisMask defaultAxes(std::size_t index, const Command& command) const noexcept;

    /// Check a command's parameters against the capability's (NEW and UPDATE):
    /// a required field held (kHold) or not finite is InvalidParameter, and a
    /// field the aircraft has nothing for NotSupported - returned at once, with
    /// the field in the log's result. One outside its range is held to it and
    /// logged (docs/flight-autonomy.md, 4.8): clamped (kClamped) or, with
    /// Reject, an OutOfRange finding, with the field and the performance limit
    /// its value broke (docs/vehicle-interface.md, 5.1).
    Reason check(std::size_t index, Command& command, CheckLog& log) const noexcept;
    Reason check(std::size_t index, SupportCommand& command, CheckLog& log) const noexcept;

    /// The applicability rules that exclude this aircraft (ruleBit): what it is never offered.
    std::uint16_t excludedRules() const noexcept { return excludedRules_; }

private:
    CapabilityCatalog(std::uint32_t features, std::uint16_t excludedRules);
    void addBehaviors();
    ParameterInfo* parameterOf(std::string_view capability, std::string_view parameter) noexcept;

    std::uint32_t features_ = ~0u; ///< the aircraft's; without one, every behaviour is offered
    std::uint16_t excludedRules_ = 0; ///< its physical exceptions (docs/flight-autonomy.md, 5)
    std::vector<std::pair<std::size_t, BehaviorTraits::Admission>> admissions_; ///< the behaviours' that have one, by index
    std::vector<std::size_t> withinEnvelope_; ///< the behaviours whose activities complete only within the envelope
    std::vector<CapabilityDescriptor> descriptors_;
    std::array<int, static_cast<std::size_t>(Level::Behavior)> byLevel_{};
    std::array<int, kSupportKinds> bySupport_{-1, -1, -1, -1, -1, -1};
    std::array<int, static_cast<std::size_t>(SetpointKind::Count)> byMode_{-1, -1, -1, -1, -1, -1}; ///< a guidance mode's capability, by its setpoint
    std::uint64_t registryRevision_ = 0;
};

} // namespace fsim::control
