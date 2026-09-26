#include "control/CapabilityHost.h"

#include "control/Protection.h"
#include "control/Registry.h"
#include "core/Log.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

/// A level no higher than Position: a behaviour's output enters below it.
int cascadeTop(Level level) noexcept { return std::min(levelRank(level), levelRank(Level::Position)); }

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
                          const VehicleProfile& profile, double controlPeriodS) noexcept {
    vehicle_ = vehicle;
    runtime_ = &runtime;
    config_ = &runtime.config();
    catalog_ = &catalog;
    adapter_ = &adapter;
    profile_ = &profile;
    controlPeriodS_ = controlPeriodS;
    config_->protection = protectionFor(profile); // Limit with an envelope section, else Off
    ++config_->revision;
}

void CapabilityHost::setProtection(ProtectionMode mode) noexcept {
    config_->protection.mode = mode;
    ++config_->revision;
}

ProtectionMode CapabilityHost::protection() const noexcept { return config_->protection.mode; }

EnvelopeStatus CapabilityHost::envelope() noexcept {
    EnvelopeStatus status = envelope_;
    status.mode = config_->protection.mode;
    envelope_ = EnvelopeStatus{};
    return status;
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

std::size_t CapabilityHost::directSlot(const SupportCommand& command) noexcept {
    if (std::holds_alternative<EnginesCommand>(command)) return kEnginesSlot;
    return kSlotCount + static_cast<std::size_t>(supportAxis(command)) - static_cast<std::size_t>(Axis::Flaps);
}

void CapabilityHost::writeDirect(std::size_t s, const SupportCommand& command) noexcept {
    RuntimeConfig& config = *config_;
    if (const auto* e = std::get_if<EnginesCommand>(&command)) {
        for (std::size_t i = 0; i < config.engines.size(); ++i) config.engines[i] = e->throttle[i];
        return;
    }
    SupportDemand& demand = config.support[s - kSlotCount];
    supportValues(command, demand.value, demand.value2);
    ++demand.revision;
}

Reason CapabilityHost::checkAxes(const CapabilityDescriptor& d, AxisMask axes) noexcept {
    if (axes & ~kAllAxes) return Reason::InvalidAxes;
    const auto primary = static_cast<AxisMask>(axes & kPrimaryAxes);
    if (!primary) return Reason::InvalidAxes; // a slot flies through the cascade on at least one primary axis
    if (primary == kPrimaryAxes || (d.axisGroups & kGroupEachAxis)) return Reason::None;
    // owned apart: a union of the groups the capability allows (guidance allows none)
    if ((primary & kLateral) && !(d.axisGroups & kGroupLateral)) return Reason::InvalidAxes;
    if ((primary & axisBit(Axis::Pitch)) && !(d.axisGroups & kGroupPitch)) return Reason::InvalidAxes;
    if ((primary & axisBit(Axis::Thrust)) && !(d.axisGroups & kGroupThrust)) return Reason::InvalidAxes;
    return Reason::None;
}

Reason CapabilityHost::awareUpTo(int top) const noexcept {
    for (const Level l : kCascadeOrder) // every level with a controller, from the acceleration level up to rank `top`
        if (levelRank(l) <= top)
            if (const Controller* c = runtime_->controller(l); c && !c->axisAware()) return Reason::ControllerNotAxisAware;
    return Reason::None;
}

Reason CapabilityHost::checkAwareness(AxisMask taken, Level level, bool cascade) const noexcept {
    const RuntimeConfig& c = *config_;
    const auto primary = static_cast<AxisMask>(taken & kPrimaryAxes);
    if (!primary || (cascade && primary == kPrimaryAxes)) return Reason::None; // no change, or one owner of every axis
    // The highest level a controller would see axes owned apart at, afterwards:
    // the new owner's, what the others keep, and the default's hold. (Where a
    // demand really goes depends on the controllers; every level up to it is checked.)
    int top = cascade ? cascadeTop(level) : -1;
    AxisMask unowned = 0;
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (c.slots[s].axes & kPrimaryAxes & ~primary) top = std::max(top, cascadeTop(c.slots[s].level));
    for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
        if (!(primary & (1u << a)) && c.owner[a] == RuntimeConfig::kNone) unowned = static_cast<AxisMask>(unowned | (1u << a));
    if (unowned && c.vehicleDefault == VehicleDefault::Hold) top = std::max(top, levelRank(Level::Velocity));
    return awareUpTo(top);
}

Reason CapabilityHost::setVehicleDefault(VehicleDefault mode) noexcept {
    RuntimeConfig& c = *config_;
    if (mode == c.vehicleDefault) return Reason::None;
    if (mode == VehicleDefault::Hold) {
        AxisMask unowned = 0;
        int top = levelRank(Level::Velocity);
        for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
            if (c.owner[a] == RuntimeConfig::kNone) unowned = static_cast<AxisMask>(unowned | (1u << a));
        for (std::size_t s = 0; s < kSlotCount; ++s)
            if (c.slots[s].axes & kPrimaryAxes) top = std::max(top, cascadeTop(c.slots[s].level));
        // the hold would fly beside what others own
        if (unowned && unowned != kPrimaryAxes)
            if (const Reason why = awareUpTo(top); why != Reason::None) return why;
        for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
            if (unowned & (1u << a)) ++c.letGo[a]; // held from where they are now
    }
    c.vehicleDefault = mode;
    ++c.revision;
    return Reason::None;
}

void CapabilityHost::takeOver(AxisMask axes, ActivityId id, double now) noexcept {
    for (std::size_t s = 0; s < kActivities; ++s) {
        if (!slots_[s].activity || !(records_[s].axes & axes)) continue;
        const auto lost = static_cast<AxisMask>(records_[s].axes & axes);
        const auto kept = static_cast<AxisMask>(records_[s].axes & ~axes);
        if (slots_[s].live) {
            if ((lost & kPrimaryAxes) || !kept) end(s, ActivityState::Canceled, Reason::Preempted, id, now);
            else slots_[s].flags |= kActivityAxesReduced; // only support axes it can do without
        }
        if (!isCascade(s) || !(kept & kPrimaryAxes)) {
            release(s); // nothing left to fly through the cascade; support axes it kept return to the default
            continue;
        }
        // What it keeps flies on: the live activity, or the ended one's residual hold (9.4).
        records_[s].axes = kept;
        config_->slots[s].axes = kept;
        ++config_->revision;
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

    Command setpoint = command;
    std::uint16_t flags = 0;
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, options.range, flags); why != Reason::None) return rejected(why);
    AxisMask axes = options.axes ? options.axes : catalog_->defaultAxes(index, command);
    if (d.level != Level::Actuator && (axes & kLateral)) axes |= kLateral; // the loop that banks also coordinates
    if (const Reason why = checkAxes(d, axes); why != Reason::None) return rejected(why);
    if (const Reason why = checkAwareness(axes, d.level, true); why != Reason::None) return rejected(why);
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
    // A free slot: every slot in use flies a primary axis the new activity did not take.
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
    if (options.axes && options.axes != axes) return rejected(Reason::InvalidAxes);
    // the engines take thrust from the cascade: what flies the rest flies it apart
    if (const Reason why = checkAwareness(axes, Level::Actuator, false); why != Reason::None) return rejected(why);
    if (const ActivityId other = holder(axes, options.source)) return rejected(Reason::AuthorityHeld, 0, other);

    const ActivityId id = activityId(vehicle_, ++serial_);
    takeOver(axes, id, now);
    const std::size_t s = directSlot(setpoint);
    writeDirect(s, setpoint);
    const auto axis = static_cast<std::size_t>(supportAxis(setpoint));
    config_->owner[axis] = s == kEnginesSlot ? RuntimeConfig::kEngines : RuntimeConfig::kSupport;
    ++config_->revision;
    start(s, id, index, options, axes, flags, now);
    if (d.persistence == Persistence::Terminating) slots_[s].target = supportGoal(setpoint);
    return accepted(id, flags);
}

CommandResult CapabilityHost::update(ActivityId activity, const Command& setpoint) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
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
    if (isCascade(s) || catalog_->indexOf(setpoint) != static_cast<int>(record.capability)) return rejected(Reason::WrongCommandType, activity);
    SupportCommand checked = setpoint;
    CommandResult result = accepted(activity);
    if (slots_[s].range != RangePolicy::None) {
        if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags); why != Reason::None)
            return rejected(why, activity);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    writeDirect(s, checked);
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
    if (s == kEnginesSlot) {
        const auto thrust = static_cast<std::size_t>(Axis::Thrust);
        if (config.owner[thrust] == RuntimeConfig::kEngines) {
            config.owner[thrust] = RuntimeConfig::kNone;
            ++config.letGo[thrust];
        }
        config.engines.fill(kHold);
    } else if (isSupport(s)) {
        const auto axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
        if (config.owner[axis] == RuntimeConfig::kSupport) config.owner[axis] = RuntimeConfig::kNone;
        SupportDemand& demand = config.support[s - kSlotCount];
        demand.value = demand.value2 = kHold;
        ++demand.revision;
    } else {
        for (std::size_t a = 0; a < kAxisCount; ++a)
            if (config.owner[a] == s) {
                config.owner[a] = RuntimeConfig::kNone;
                if (a < kPrimaryAxisCount) ++config.letGo[a]; // the default's hold captures it afresh
            }
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
            const bool flown = isCascade(s) ? report.updates > 0 && report.slots[s].generation == config.slots[s].generation
                                            : report.updates > 0;
            if (record.state == ActivityState::Pending && flown) record.state = ActivityState::Active;
            const std::uint16_t flags = static_cast<std::uint16_t>(slot.flags | flagsOn(report, record.axes));
            record.constraints = flags;
            record.constraintsSeen = static_cast<std::uint16_t>(record.constraintsSeen | flags);
            slot.flags = 0;
            if (state.diverged) {
                end(s, ActivityState::Failed, Reason::Diverged, 0, now);
            } else if (record.state == ActivityState::Active && isCascade(s)) {
                const SlotReport& events = report.slots[s];
                if (events.events & kFailed) end(s, ActivityState::Failed, events.failure, 0, now);
                else if (events.events & kFinished) end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            } else if (record.state == ActivityState::Active && isSupport(s) && !std::isnan(slot.target)) {
                // gear or flaps: done when they are there (and held there, as a residual)
                const std::size_t axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
                const double position = axis == static_cast<std::size_t>(Axis::Gear) ? positions.gear : positions.flaps;
                if (!std::isnan(position) && std::abs(position - slot.target) < 0.01)
                    end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            }
        }
        if (isCascade(s)) {
            report.slots[s].events = 0;
            report.slots[s].failure = Reason::None;
        }
    }
    // the envelope: what was limited, how long and how far the state was beyond each limit
    for (std::size_t l = 0; l < kLimitCount; ++l) {
        const LimitReport& r = report.limits[l];
        if (!r.limitedUpdates && !r.exceededUpdates) continue;
        LimitStatus& status = envelope_.limits[l];
        status.limitedUpdates += r.limitedUpdates;
        status.exceededUpdates += r.exceededUpdates;
        status.exceededS += r.exceededUpdates * controlPeriodS_;
        status.worstExcess = std::max(status.worstExcess, static_cast<double>(r.worstExcess));
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
