#include "control/CapabilityHost.h"

#include "control/Registry.h"
#include "core/Log.h"

#include <algorithm>

namespace fsim::control {

namespace {

constexpr AxisMask kLateral = axisBit(Axis::Roll) | axisBit(Axis::Yaw);

/// What an activity takes from the runtime's flags on the axes it owns.
std::uint16_t flagsOn(const RuntimeReport& report, AxisMask axes) noexcept {
    std::uint16_t f = 0;
    for (std::size_t a = 0; a < kAxisCount; ++a) {
        if (!(axes & (1u << a))) continue;
        const std::uint16_t x = report.axisFlags[a];
        if (x & kSaturated) f |= kActivitySaturated;
        if (x & kDemandLimited) f |= kActivityDemandLimited;
        if (x & kExceeded) f |= kActivityExceeded;
    }
    return f;
}

CommandResult accepted(ActivityId activity, std::uint16_t flags = 0) noexcept {
    CommandResult r;
    r.status = CommandStatus::Accepted;
    r.activity = activity;
    r.flags = flags;
    return r;
}

} // namespace

void CapabilityHost::bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog) noexcept {
    vehicle_ = vehicle;
    runtime_ = &runtime;
    config_ = &runtime.config();
    catalog_ = &catalog;
}

CommandResult CapabilityHost::rejected(Reason reason, ActivityId activity, ActivityId other) const noexcept {
    CommandResult r;
    r.status = CommandStatus::Rejected;
    r.reason = reason;
    r.activity = activity;
    r.other = other;
    return r;
}

int CapabilityHost::liveSlot(ActivityId activity) const noexcept {
    if (!activity) return -1;
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (slots_[s].live && slots_[s].activity == activity) return static_cast<int>(s);
    return -1;
}

CapabilityStatus CapabilityHost::status(std::size_t capability, const sim::VehicleState& state) const noexcept {
    if (capability >= catalog_->size()) return {Availability::Disabled, Reason::UnknownCapability};
    const CapabilityDescriptor& d = catalog_->descriptor(capability);
    if ((d.kind == CapabilityKind::Flight || d.kind == CapabilityKind::Guidance) && state.diverged)
        return {Availability::TemporarilyUnavailable, Reason::Diverged};
    return {};
}

CommandResult CapabilityHost::submit(const Command& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(Reason::UnknownCapability);
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    if (!(d.interactions & kCommand)) return rejected(Reason::UnknownCapability);
    const bool checked = options.range != RangePolicy::None;
    if (checked) {
        if (status(index, state).availability != Availability::Available) return rejected(Reason::Unavailable);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
    }

    AxisMask axes = options.axes ? options.axes : catalog_->defaultAxes(index, command);
    if (d.level != Level::Actuator && (axes & kLateral)) axes |= kLateral; // the loop that banks also coordinates
    // Until axes can be owned apart (step 3), a command flies every primary axis.
    if ((axes & kPrimaryAxes) != kPrimaryAxes || (axes & ~kAllAxes)) return rejected(Reason::InvalidAxes);

    Command setpoint = command;
    std::uint16_t flags = 0;
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, options.range, flags); why != Reason::None) return rejected(why);

    // Authority: a live activity of a higher priority keeps its axes.
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (slots_[s].live && (records_[s].axes & axes) && records_[s].source > options.source)
            return rejected(Reason::AuthorityHeld, 0, records_[s].id);

    std::unique_ptr<Behavior> behavior;
    if (d.kind == CapabilityKind::Guidance) {
        auto created = ControllerRegistry::instance().create(d.behavior);
        auto* b = dynamic_cast<Behavior*>(created.get());
        if (!b) return rejected(Reason::UnknownCapability);
        created.release();
        behavior.reset(b);
    }

    // Accepted: what it takes over ends, preempted; a residual hold on its axes is simply replaced.
    const ActivityId id = activityId(vehicle_, ++serial_);
    for (std::size_t s = 0; s < kSlotCount; ++s) {
        if (!slots_[s].activity || !(records_[s].axes & axes)) continue;
        if (slots_[s].live) end(s, ActivityState::Canceled, Reason::Preempted, id, now);
        release(s); // until step 3 merges slots, what loses its primary axes flies nothing
    }
    std::size_t s = 0;
    while (s + 1 < kSlotCount && slots_[s].activity) ++s;

    RuntimeConfig& config = *config_;
    SetpointSlot& slot = config.slots[s];
    slot.command = std::move(setpoint);
    slot.level = d.level;
    slot.axes = axes;
    ++slot.generation;
    ++slot.revision;
    for (std::size_t a = 0; a < kAxisCount; ++a)
        if (axes & (1u << a)) config.owner[a] = static_cast<std::uint8_t>(s);
    ++config.revision;
    runtime_->install(s, std::move(behavior));

    slots_[s] = Slot{id, true, options.range, static_cast<std::uint16_t>(flags & kClamped ? kActivityClamped : 0)};
    ActivityRecord& record = records_[s];
    record = ActivityRecord{};
    record.id = id;
    record.vehicle = vehicle_;
    record.capability = static_cast<std::uint16_t>(index);
    record.source = options.source;
    record.axes = axes;
    record.state = ActivityState::Pending;
    record.startTime = now;
    return accepted(id, flags);
}

CommandResult CapabilityHost::update(ActivityId activity, const Command& setpoint) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    const ActivityRecord& record = records_[s];
    if (!(catalog_->descriptor(record.capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    SetpointSlot& slot = config_->slots[s];
    if (setpoint.index() != slot.command.index()) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    if (slots_[s].range == RangePolicy::None) {
        assignSetpoint(slot.command, setpoint);
    } else {
        Command checked = setpoint; // no heap data: guidance takes no UPDATE
        if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags); why != Reason::None)
            return rejected(why, activity);
        assignSetpoint(slot.command, checked);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    ++slot.revision;
    return result;
}

CommandResult CapabilityHost::cancel(ActivityId activity, double now) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    end(s, ActivityState::Canceled, Reason::Requested, 0, now);
    release(s); // its axes: the vehicle default
    CommandResult r;
    r.status = CommandStatus::Canceled;
    r.activity = activity;
    return r;
}

CommandResult CapabilityHost::command(const Command& command, const sim::VehicleState& state, double now) {
    // The fast path: the same capability as the live legacy activity (a
    // behaviour command re-creates its behaviour, as it always has).
    if (updateLegacy(command)) return accepted(legacy_);
    CommandOptions legacy;
    legacy.source = Source::Policy;
    legacy.axes = kLegacyAxes;
    legacy.range = RangePolicy::None;
    const CommandResult r = submit(command, legacy, state, now);
    if (r.accepted()) {
        legacy_ = r.activity;
        legacySlot_ = liveSlot(legacy_);
    } else if (const auto* b = std::get_if<BehaviorCommand>(&command); b && r.reason == Reason::UnknownCapability) {
        LOG_ERROR("control") << "unknown behaviour '" << b->id << "'; holding the current command";
    }
    return r;
}

const ActivityRecord* CapabilityHost::activity(ActivityId activity) const noexcept {
    if (!activity) return nullptr;
    if (const int s = liveSlot(activity); s >= 0) return &records_[static_cast<std::size_t>(s)];
    for (std::size_t i = 0; i < recentCount_; ++i) {
        const ActivityRecord& r = recent_[(recentNext_ + kRecent - 1 - i) % kRecent];
        if (r.id == activity) return &r;
    }
    return nullptr;
}

std::vector<ActivityRecord> CapabilityHost::activities() const {
    std::vector<ActivityRecord> out;
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (slots_[s].live) out.push_back(records_[s]);
    for (std::size_t i = 0; i < recentCount_; ++i) out.push_back(recent_[(recentNext_ + kRecent - 1 - i) % kRecent]);
    return out;
}

void CapabilityHost::end(std::size_t s, ActivityState state, Reason reason, ActivityId by, double now) noexcept {
    ActivityRecord& record = records_[s];
    record.state = state;
    record.reason = reason;
    record.by = by;
    record.endTime = now;
    slots_[s].live = false; // the runtime flies its residual hold until another activity takes the axes
    if (slots_[s].activity == legacy_) legacySlot_ = -1;
    recent_[recentNext_] = record;
    recentNext_ = (recentNext_ + 1) % kRecent;
    recentCount_ = std::min(recentCount_ + 1, kRecent);
}

void CapabilityHost::release(std::size_t s) noexcept {
    RuntimeConfig& config = *config_;
    for (auto& owner : config.owner)
        if (owner == s) owner = RuntimeConfig::kNone;
    config.slots[s].axes = 0;
    ++config.revision;
    runtime_->install(s, nullptr);
    slots_[s] = Slot{};
}

void CapabilityHost::afterStep(const sim::VehicleState& state, double now) noexcept {
    RuntimeReport& report = runtime_->report();
    const RuntimeConfig& config = *config_;
    for (std::size_t s = 0; s < kSlotCount; ++s) {
        SlotReport& flown = report.slots[s];
        Slot& slot = slots_[s];
        if (slot.live) {
            ActivityRecord& record = records_[s];
            if (record.state == ActivityState::Pending && report.updates > 0 && flown.generation == config.slots[s].generation)
                record.state = ActivityState::Active;
            const std::uint16_t flags = static_cast<std::uint16_t>(slot.flags | flagsOn(report, record.axes));
            record.constraints = flags;
            record.constraintsSeen = static_cast<std::uint16_t>(record.constraintsSeen | flags);
            slot.flags = 0;
            if (state.diverged) end(s, ActivityState::Failed, Reason::Diverged, 0, now);
            else if (record.state == ActivityState::Active && (flown.events & kFailed)) end(s, ActivityState::Failed, flown.failure, 0, now);
            else if (record.state == ActivityState::Active && (flown.events & kFinished)) end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
        }
        flown.events = 0;
        flown.failure = Reason::None;
    }
    report.clearAccumulators();
}

void CapabilityHost::onReset() noexcept {
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (slots_[s].live) {
            records_[s].state = ActivityState::Pending; // the runtime starts it again
            slots_[s].flags = 0;
        }
}

} // namespace fsim::control
