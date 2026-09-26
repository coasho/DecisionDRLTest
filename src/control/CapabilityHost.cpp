#include "control/CapabilityHost.h"

#include "control/Registry.h"
#include "core/Log.h"

#include <algorithm>
#include <cmath>

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

void CapabilityHost::bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog, const VehicleAdapter& adapter,
                          const VehicleProfile& profile) noexcept {
    vehicle_ = vehicle;
    runtime_ = &runtime;
    config_ = &runtime.config();
    catalog_ = &catalog;
    adapter_ = &adapter;
    profile_ = &profile;
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
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && slots_[s].activity == activity) return static_cast<int>(s);
    return -1;
}

CapabilityStatus CapabilityHost::status(std::size_t capability, const sim::VehicleState& state) const noexcept {
    if (capability >= catalog_->size()) return {Availability::Disabled, Reason::UnknownCapability};
    if (state.diverged) return {Availability::TemporarilyUnavailable, Reason::Diverged};
    return {};
}

ActivityId CapabilityHost::holder(AxisMask axes, Source source) const noexcept {
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && (records_[s].axes & axes) && records_[s].source > source) return records_[s].id;
    return 0;
}

void CapabilityHost::takeOver(AxisMask axes, ActivityId id, double now) noexcept {
    for (std::size_t s = 0; s < kActivities; ++s) {
        if (!slots_[s].activity || !(records_[s].axes & axes)) continue;
        const AxisMask lost = records_[s].axes & axes;
        const auto kept = static_cast<AxisMask>(records_[s].axes & ~axes);
        if ((lost & kPrimaryAxes) || !kept) {
            // It can no longer do its job: it ends (preempted, if it was still
            // at it) and - until step 3 lets slots share the cascade - flies nothing.
            if (slots_[s].live) end(s, ActivityState::Canceled, Reason::Preempted, id, now);
            release(s);
        } else {
            // Only support axes it can do without: it carries on without them.
            records_[s].axes = kept;
            if (slots_[s].live) slots_[s].flags |= kActivityAxesReduced;
            if (!isSupport(s)) config_->slots[s].axes = kept;
        }
    }
}

ActivityRecord& CapabilityHost::start(std::size_t s, ActivityId id, std::size_t capability, const CommandOptions& options, AxisMask axes,
                                      std::uint16_t flags, double now) noexcept {
    slots_[s] = Slot{id, true, options.range, static_cast<std::uint16_t>(flags & kClamped ? kActivityClamped : 0), kUnknown};
    ActivityRecord& record = records_[s];
    record = ActivityRecord{};
    record.id = id;
    record.vehicle = vehicle_;
    record.capability = static_cast<std::uint16_t>(capability);
    record.source = options.source;
    record.axes = axes;
    record.state = ActivityState::Pending;
    record.startTime = now;
    return record;
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
    if (const ActivityId other = holder(axes, options.source)) return rejected(Reason::AuthorityHeld, 0, other);

    std::unique_ptr<Behavior> behavior;
    if (d.kind == CapabilityKind::Guidance) {
        auto created = ControllerRegistry::instance().create(d.behavior);
        auto* b = dynamic_cast<Behavior*>(created.get());
        if (!b) return rejected(Reason::UnknownCapability);
        created.release();
        behavior.reset(b);
    }

    const ActivityId id = activityId(vehicle_, ++serial_);
    takeOver(axes, id, now);
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
    start(s, id, index, options, axes, flags, now);
    return accepted(id, flags);
}

CommandResult CapabilityHost::submit(const SupportCommand& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(Reason::UnknownCapability); // the aircraft has no such effector
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    const bool checked = options.range != RangePolicy::None;
    if (checked) {
        if (status(index, state).availability != Availability::Available) return rejected(Reason::Unavailable);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
    }
    SupportCommand setpoint = command;
    std::uint16_t flags = 0;
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, options.range, flags); why != Reason::None) return rejected(why);
    // the placards: no gear up on the ground, no gear or flaps out above their speeds
    if (const Reason why = adapter_->admit(setpoint, state, *profile_); why != Reason::None) return rejected(why);
    const AxisMask axes = d.axes;
    if (const ActivityId other = holder(axes, options.source)) return rejected(Reason::AuthorityHeld, 0, other);

    const ActivityId id = activityId(vehicle_, ++serial_);
    takeOver(axes, id, now);
    const Axis axis = supportAxis(setpoint);
    const std::size_t s = supportSlot(axis);
    SupportDemand& demand = config_->support[s - kSlotCount];
    supportValues(setpoint, demand.value, demand.value2);
    ++demand.revision;
    config_->owner[static_cast<std::size_t>(axis)] = RuntimeConfig::kSupport;
    ++config_->revision;
    start(s, id, index, options, axes, flags, now);
    if (d.persistence == Persistence::Terminating) slots_[s].target = supportGoal(setpoint);
    return accepted(id, flags);
}

CommandResult CapabilityHost::update(ActivityId activity, const Command& setpoint) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    if (isSupport(s)) return rejected(Reason::WrongCommandType, activity);
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

CommandResult CapabilityHost::update(ActivityId activity, const SupportCommand& setpoint) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    const ActivityRecord& record = records_[s];
    if (!isSupport(s) || catalog_->indexOf(setpoint) != static_cast<int>(record.capability)) return rejected(Reason::WrongCommandType, activity);
    SupportCommand checked = setpoint;
    CommandResult result = accepted(activity);
    if (slots_[s].range != RangePolicy::None) {
        if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags); why != Reason::None)
            return rejected(why, activity);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    SupportDemand& demand = config_->support[s - kSlotCount];
    supportValues(checked, demand.value, demand.value2);
    ++demand.revision;
    if (!std::isnan(slots_[s].target)) slots_[s].target = supportGoal(checked);
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
    for (std::size_t s = 0; s < kActivities; ++s)
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
    if (isSupport(s)) {
        const auto axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
        if (config.owner[axis] == RuntimeConfig::kSupport) config.owner[axis] = RuntimeConfig::kNone;
        SupportDemand& demand = config.support[s - kSlotCount];
        demand.value = demand.value2 = kHold;
        ++demand.revision;
    } else {
        for (auto& owner : config.owner)
            if (owner == s) owner = RuntimeConfig::kNone;
        config.slots[s].axes = 0;
        runtime_->install(s, nullptr);
        runtime_->report().slots[s] = SlotReport{}; // nothing it reported carries over to the slot's next activity
    }
    ++config.revision;
    if (static_cast<int>(s) == legacySlot_) legacySlot_ = -1;
    slots_[s] = Slot{};
}

void CapabilityHost::afterStep(const sim::VehicleState& state, const EffectorPositions& positions, double now) noexcept {
    RuntimeReport& report = runtime_->report();
    const RuntimeConfig& config = *config_;
    for (std::size_t s = 0; s < kActivities; ++s) {
        Slot& slot = slots_[s];
        if (!slot.activity) continue; // nothing flies here (and nothing reported)
        if (slot.live) {
            ActivityRecord& record = records_[s];
            const bool flown = isSupport(s) ? report.updates > 0
                                            : report.updates > 0 && report.slots[s].generation == config.slots[s].generation;
            if (record.state == ActivityState::Pending && flown) record.state = ActivityState::Active;
            const std::uint16_t flags = static_cast<std::uint16_t>(slot.flags | flagsOn(report, record.axes));
            record.constraints = flags;
            record.constraintsSeen = static_cast<std::uint16_t>(record.constraintsSeen | flags);
            slot.flags = 0;
            if (state.diverged) {
                end(s, ActivityState::Failed, Reason::Diverged, 0, now);
            } else if (record.state == ActivityState::Active && !isSupport(s)) {
                const SlotReport& events = report.slots[s];
                if (events.events & kFailed) end(s, ActivityState::Failed, events.failure, 0, now);
                else if (events.events & kFinished) end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            } else if (record.state == ActivityState::Active && !std::isnan(slot.target)) {
                // gear or flaps: done when they are there (and held there, as a residual)
                const std::size_t axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
                const double position = axis == static_cast<std::size_t>(Axis::Gear) ? positions.gear : positions.flaps;
                if (!std::isnan(position) && std::abs(position - slot.target) < 0.01)
                    end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            }
        }
        if (!isSupport(s)) {
            report.slots[s].events = 0;
            report.slots[s].failure = Reason::None;
        }
    }
    report.clearAccumulators();
}

void CapabilityHost::onReset() noexcept {
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live) {
            records_[s].state = ActivityState::Pending; // the runtime starts it again
            slots_[s].flags = 0;
        }
}

} // namespace fsim::control
