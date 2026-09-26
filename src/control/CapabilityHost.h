#pragma once

// The contract layer's per-vehicle half (docs/control-architecture.md,
// sections 9 and 10): takes commands (NEW, UPDATE, CANCEL) and answers them
// at once, arbitrates authority, keeps the activities and their records, and
// writes the runtime's configuration - between steps, on the caller's thread.
// After each world step it reads the runtime's report into its activities.

#include "control/Catalog.h"
#include "control/Runtime.h"
#include "fsim/Capability.h"
#include "fsim/ControlStack.h"
#include "fsim/VehicleState.h"

#include <array>
#include <cstdint>
#include <vector>

namespace fsim::control {

class CapabilityHost {
public:
    /// Ended activities a vehicle remembers for queries.
    static constexpr std::size_t kRecent = 16;

    /// The vehicle this host serves, its runtime and its catalog (which outlive it).
    void bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog) noexcept;

    /// NEW. `state` is the vehicle's, for availability; `now` the simulation time.
    CommandResult submit(const Command& command, const CommandOptions& options, const sim::VehicleState& state, double now);
    /// UPDATE: a new setpoint for a live activity - the fast path; allocates nothing.
    CommandResult update(ActivityId activity, const Command& setpoint) noexcept;
    /// CANCEL: the activity ends and its axes return to the vehicle default.
    CommandResult cancel(ActivityId activity, double now) noexcept;
    /// The existing entry points (docs/control-architecture.md, 10.7): an
    /// UPDATE of their live activity at the same capability, else a NEW with
    /// the legacy options.
    CommandResult command(const Command& command, const sim::VehicleState& state, double now);
    /// command()'s per-step path, inline: a new setpoint for the live legacy
    /// activity at the same level; false if command() has more to do.
    bool updateLegacy(const Command& command) noexcept {
        if (legacySlot_ < 0 || std::holds_alternative<BehaviorCommand>(command)) return false;
        SetpointSlot& slot = config_->slots[static_cast<std::size_t>(legacySlot_)];
        if (slot.command.index() != command.index()) return false;
        assignSetpoint(slot.command, command);
        ++slot.revision;
        return true;
    }

    /// Live or recently ended; null if unknown.
    const ActivityRecord* activity(ActivityId activity) const noexcept;
    /// The live activities, then the ended ones it remembers, newest first.
    std::vector<ActivityRecord> activities() const;
    CapabilityStatus status(std::size_t capability, const sim::VehicleState& state) const noexcept;

    /// After each world step: the runtime's report into the activities, then cleared.
    void afterStep(const sim::VehicleState& state, double now) noexcept;
    /// Vehicle reset: live activities start again.
    void onReset() noexcept;

private:
    struct Slot {
        ActivityId activity = 0; ///< what this slot flies: a live activity, or an ended one's residual hold; 0 = free
        bool live = false;
        RangePolicy range = RangePolicy::Clamp;
        std::uint16_t flags = 0; ///< host flags since the last world step (kActivityClamped)
    };

    int liveSlot(ActivityId activity) const noexcept;
    void end(std::size_t slot, ActivityState state, Reason reason, ActivityId by, double now) noexcept;
    void release(std::size_t slot) noexcept;
    CommandResult rejected(Reason reason, ActivityId activity = 0, ActivityId other = 0) const noexcept;

    std::uint32_t vehicle_ = 0;
    ControlStack* runtime_ = nullptr;
    RuntimeConfig* config_ = nullptr; ///< the runtime's, held for the fast path
    const CapabilityCatalog* catalog_ = nullptr;
    std::uint32_t serial_ = 0;
    std::array<Slot, kSlotCount> slots_{};
    std::array<ActivityRecord, kSlotCount> records_{}; ///< per slot: its activity's record
    std::array<ActivityRecord, kRecent> recent_{};     ///< ended records, a ring
    std::size_t recentNext_ = 0, recentCount_ = 0;
    ActivityId legacy_ = 0;                            ///< the activity the existing entry points command
    int legacySlot_ = -1;                              ///< its slot while it is live
};

} // namespace fsim::control
