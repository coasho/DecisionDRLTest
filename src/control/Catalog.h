#pragma once

// The capabilities a vehicle offers (docs/control-architecture.md, section
// 8): one descriptor per flight level and per registered behaviour. Built
// between steps and only ever extended, so an index into it never changes.

#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/VehicleProfile.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::control {

/// The fields of a command struct below Level::Behavior, in declaration
/// order (the C ABI's order too): pointers into `c`, at most 8. Returns how many.
std::size_t commandFields(Command& c, double* fields[8]) noexcept;

class VehicleAdapter;

class CapabilityCatalog {
public:
    /// The five flight capabilities with the loops' own ranges, and every behaviour registered so far.
    CapabilityCatalog();
    /// For an aircraft: what its adapter declares for its profile.
    CapabilityCatalog(const VehicleProfile& profile, const VehicleAdapter& adapter);

    /// Intersect a parameter's range with [lo, hi] (a NaN bound leaves that side).
    void narrow(std::string_view capability, std::string_view parameter, double lo, double hi);

    /// Add the behaviours registered since; true if there were any. Between steps only.
    bool refresh();

    std::size_t size() const noexcept { return descriptors_.size(); }
    const CapabilityDescriptor& descriptor(std::size_t index) const noexcept { return descriptors_[index]; }
    const std::vector<CapabilityDescriptor>& descriptors() const noexcept { return descriptors_; }

    /// The capability a command selects (its level, or its behaviour's id); -1 if none.
    int indexOf(const Command& command) const noexcept;
    /// By capability id ("fsim.guidance.hold") or a behaviour's registry id ("hold"); -1 if none.
    int find(std::string_view id) const noexcept;

    /// The axes a command owns when its options name none.
    AxisMask defaultAxes(std::size_t index, const Command& command) const noexcept;

    /// Check a command's parameters against the capability's (NEW and UPDATE):
    /// a required field held (kHold) or not finite is InvalidParameter; one
    /// outside its range is clamped (kClamped in `flags`) or, with Reject,
    /// OutOfRange. None if it may fly.
    Reason check(std::size_t index, Command& command, RangePolicy range, std::uint16_t& flags) const noexcept;

private:
    void addBehaviors();

    std::vector<CapabilityDescriptor> descriptors_;
    std::array<int, static_cast<std::size_t>(Level::Behavior)> byLevel_{};
    std::uint64_t registryRevision_ = 0;
};

} // namespace fsim::control
