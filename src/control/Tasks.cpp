// Flight tasks and suggestions (docs/flight-autonomy.md, 4.11): the vehicle's
// task store, a task's runs, the platform's suggestions - CapabilityHost's.
// Apart from the host's other code, so that growing either moves neither.
#include "control/CapabilityHost.h"

#include "control/Features.h"
#include "control/Route.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

CapabilityHost::Task* CapabilityHost::findTask(TaskId id) noexcept {
    for (Task& t : tasks_)
        if (t.id == id) return &t;
    return nullptr;
}

ActivityRecord* CapabilityHost::liveRecord(ActivityId activity) noexcept {
    if (const int s = liveSlot(activity); s >= 0) return &records_[static_cast<std::size_t>(s)];
    if (Waiting* w = waitingEntry(activity)) return &w->record;
    return nullptr;
}

double CapabilityHost::repeatInterval(ActivityId activity) const noexcept {
    for (const Task& t : tasks_)
        if (t.activity == activity) return t.repetition.intervalS;
    return kUnknown;
}

void CapabilityHost::restartRun(std::size_t s) noexcept {
    ActivityRecord& record = records_[s];
    ++record.run;
    record.progress = ActivityProgress{};
    slots_[s].restartAt = kUnknown, slots_[s].outside = false;
    if (auto* route = std::get_if<RouteCommand>(&config_->slots[s].command); route && !isHold(slots_[s].firstStart)) route->start = slots_[s].firstStart;
    ++config_->slots[s].generation, ++config_->slots[s].revision; // its behaviour, afresh
}

namespace {

/// A task's state once its activity ended.
TaskState endOf(const ActivityRecord& r) noexcept {
    switch (r.state) {
    case ActivityState::Completed: return TaskState::Completed;
    case ActivityState::Failed: return TaskState::Failed;
    case ActivityState::Deleted: return TaskState::Canceled;
    default: return r.reason == Reason::Requested ? TaskState::Canceled : TaskState::Dropped; // (preempted, revoked, released, ...)
    }
}

/// The whole task's percent: the runs done, and the one flying's progress.
double percentOf(const ActivityRecord& r) noexcept {
    const double p = std::isnan(r.progress.percent) ? 0.0 : r.progress.percent;
    return r.runs > 1 ? ((r.run - 1) * 100.0 + p) / r.runs : p;
}

} // namespace

void CapabilityHost::noteEnd(const ActivityRecord& r) noexcept {
    for (Task& t : tasks_)
        if (t.activity == r.id) {
            t.ended = endOf(r), t.reason = r.reason, t.endTime = r.endTime;
            t.run = r.run, t.runs = r.runs;
            t.percent = r.state == ActivityState::Completed ? 100.0 : percentOf(r);
            return;
        }
}

TaskStatus CapabilityHost::statusOf(const Task& t) const noexcept {
    TaskStatus s;
    s.id = t.id, s.suggested = t.suggested, s.activity = t.activity, s.commandId = t.commandId, s.startTime = t.startTime;
    const ActivityRecord* a = t.activity ? activity(t.activity) : nullptr;
    if (a && a->live()) {
        s.state = a->state == ActivityState::Active ? TaskState::Executing : TaskState::ExecutionPending;
        s.run = a->run, s.runs = a->runs;
        s.percent = percentOf(*a);
        return s;
    }
    s.state = t.ended, s.reason = t.reason, s.endTime = t.endTime;
    s.run = t.run, s.runs = t.runs, s.percent = t.percent;
    if (!t.activity) s.runs = t.repetition.attempts;
    return s;
}

Reason CapabilityHost::storeTask(TaskId id, const Command& command, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments,
                                 TaskRepetition repetition, const PatternShape* shape, const CurveShape* curveShape, Span<const RouteLoiter> loiters) {
    if (pendingSuggestions_) materialize();
    if (id == 0 || (id & kSuggestedTask)) return Reason::InvalidParameter; // (the platform's own ids)
    if (repetition.attempts == 0 || repetition.attempts > 0xFFFF) return Reason::InvalidParameter;
    if (!std::isnan(repetition.intervalS) && !(std::isfinite(repetition.intervalS) && repetition.intervalS >= 0.0)) return Reason::InvalidParameter;
    const int found = catalog_->indexOf(command);
    if (found < 0) return missing(featureOf(command)); // what the vehicle cannot fly: why
    const CapabilityDescriptor& d = catalog_->descriptor(static_cast<std::size_t>(found));
    if (!(d.interactions & kCommand)) return Reason::UnknownCapability;
    if (repetition.attempts > 1 && d.persistence != Persistence::Terminating) return Reason::InvalidParameter; // (runs of what never completes)
    Task* t = findTask(id);
    if (t && t->activity) {
        const ActivityRecord* a = activity(t->activity);
        if (a && a->live()) return Reason::TaskActive;
    }
    if (!t) t = &tasks_.emplace_back();
    *t = Task{};
    t->id = id;
    t->command = command;
    t->waypoints.assign(waypoints.begin(), waypoints.end());
    t->segments.assign(segments.begin(), segments.end());
    t->loiters.assign(loiters.begin(), loiters.end());
    if (shape) t->shape = *shape;
    if (curveShape) t->curveShape = *curveShape;
    t->repetition = repetition;
    return Reason::None;
}

CommandResult CapabilityHost::commandTask(TaskId id, CommandOptions options, const sim::VehicleState& state, double now) {
    if (pendingSuggestions_) materialize();
    Task* t = findTask(id);
    if (!t) {
        details_.clear();
        return rejected(Reason::UnknownTask);
    }
    if (const ActivityRecord* a = t->activity ? activity(t->activity) : nullptr; a && a->live()) {
        details_.clear();
        return rejected(Reason::TaskActive, 0, t->activity);
    }
    // the task it comes from, among the requirements it traces to (the first place free, else the last)
    std::size_t k = 0;
    while (k + 1 < kMaxRequirements && options.trace[k].kind != RequirementKind::None) ++k;
    options.trace[k] = {RequirementKind::Task, id};
    // its NEW (from copies: a suggestion made as it is refused may move the tasks kept)
    const Command command = t->command;
    const std::vector<Waypoint> waypoints = t->waypoints;
    const std::vector<NurbsSegment> segments = t->segments;
    const PatternShape shape = t->shape;
    const CurveShape curveShape = t->curveShape;
    const std::vector<RouteLoiter> loiters = t->loiters;
    const std::uint32_t attempts = t->repetition.attempts;
    CommandResult r = submitWith(command, waypoints, segments, options, state, now, true, &shape, &curveShape, loiters);
    if (!r.accepted()) return r;
    if (Task* again = findTask(id)) { // (found again: the tasks kept may have moved)
        again->activity = r.activity, again->commandId = options.commandId;
        again->ended = TaskState::AwaitingExecution, again->reason = Reason::None, again->startTime = now, again->endTime = kUnknown;
        again->run = 1, again->runs = attempts;
    }
    if (ActivityRecord* a = liveRecord(r.activity)) a->run = 1, a->runs = static_cast<std::uint16_t>(attempts); // its runs
    else if (const ActivityRecord* ended = activity(r.activity)) noteEnd(*ended); // (what waited took it at once: the task learns its end)
    return r;
}

CommandResult CapabilityHost::cancelTask(TaskId id, const sim::VehicleState& state, double now, Caller caller) {
    if (pendingSuggestions_) materialize();
    Task* t = findTask(id);
    if (!t) return rejected(Reason::UnknownTask);
    if (const ActivityRecord* a = t->activity ? activity(t->activity) : nullptr; a && a->live()) return cancel(t->activity, state, now, caller);
    CommandResult r; // not flying: it will not be
    r.status = CommandStatus::Canceled;
    r.commandId = t->commandId;
    if (!t->activity) t->ended = TaskState::Canceled, t->reason = Reason::Requested, t->endTime = now;
    return r;
}

Reason CapabilityHost::removeTask(TaskId id) {
    if (pendingSuggestions_) materialize();
    Task* t = findTask(id);
    if (!t) return Reason::UnknownTask;
    if (const ActivityRecord* a = t->activity ? activity(t->activity) : nullptr; a && a->live()) return Reason::TaskActive;
    tasks_.erase(tasks_.begin() + (t - tasks_.data()));
    return Reason::None;
}

bool CapabilityHost::taskStatus(TaskId id, TaskStatus& out) {
    if (pendingSuggestions_) materialize();
    const Task* t = findTask(id);
    if (!t) return false;
    out = statusOf(*t);
    return true;
}

std::vector<TaskStatus> CapabilityHost::tasks() {
    if (pendingSuggestions_) materialize();
    std::vector<TaskStatus> out;
    out.reserve(tasks_.size());
    for (const Task& t : tasks_) out.push_back(statusOf(t));
    return out;
}

CapabilityHost::Task& CapabilityHost::newSuggestion(TaskId id) {
    std::size_t kept = 0;
    for (const Task& t : tasks_) kept += t.suggested;
    for (auto it = tasks_.begin(); kept >= kSuggestions && it != tasks_.end();) { // the oldest not flying makes room
        const ActivityRecord* a = it->suggested && it->activity ? activity(it->activity) : nullptr;
        if (it->suggested && !(a && a->live())) it = tasks_.erase(it), --kept;
        else ++it;
    }
    Task& t = tasks_.emplace_back();
    t.id = id;
    t.suggested = true;
    return t;
}

TaskId CapabilityHost::suggest(const Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments, const PatternShape* shape,
                              const CurveShape* curveShape, Span<const RouteLoiter> loiters) {
    Task& t = newSuggestion(kSuggestedTask | ++suggestionSerial_);
    t.command = setpoint;
    if (shape && std::holds_alternative<PatternCommand>(setpoint)) t.shape = *shape;
    if (curveShape && std::holds_alternative<CurveCommand>(setpoint)) t.curveShape = *curveShape;
    if (std::holds_alternative<RouteCommand>(setpoint) && routePlan_) { // (as held: its loiters complete, their place their points' - 4.31)
        t.waypoints.assign(routePlan_->points, routePlan_->points + routePlan_->count);
        for (std::uint32_t k = 0; k < routePlan_->loiterCount; ++k) t.loiters.push_back(route::unplaced(routePlan_->loiters[k]));
    } else {
        t.waypoints.assign(waypoints.begin(), waypoints.end());
        t.loiters.assign(loiters.begin(), loiters.end());
    }
    t.segments.assign(segments.begin(), segments.end());
    return t.id;
}

void CapabilityHost::materialize() {
    if (!pendingSuggestions_ || !waiting_) return;
    for (Waiting& w : *waiting_)
        if (w.used && w.suggested) {
            Task& t = newSuggestion(w.suggestion);
            t.command = std::move(w.command);
            t.waypoints = w.waypoints;
            t.segments = w.segments;
            t.loiters = w.loiters;
            t.shape = w.shape, t.curveShape = w.curveShape;
            w.used = false, w.suggested = false, w.behavior.reset();
            --waitingCount_, --pendingSuggestions_;
        }
}

} // namespace fsim::control
