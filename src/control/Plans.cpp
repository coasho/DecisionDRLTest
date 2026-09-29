// Route plans (docs/flight-autonomy.md, 4.39): the vehicle's store of A-GRA's
// route plans, taken through the VI's plan activation states (1.2.5), and
// their execution; FA's own plans and the airfields (4.40) - CapabilityHost's.
// Apart from the host's other code, so that growing either moves neither.
#include "control/CapabilityHost.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fsim::control {

struct CapabilityHost::PlanEntry {
    PlanId id = 0;
    RoutePlan kept;               ///< as uploaded last (revision above 0)
    RoutePlan received;           ///< published since it was prepared for upload
    bool hasReceived = false;
    std::uint32_t revision = 0;   ///< uploads kept
    PlanState state = PlanState::ReadyForUpload;
    Reason reason = Reason::None; ///< why its last command failed, or why its execution ended
    ActivityId activity = 0;      ///< its activity, since it was activated last
    std::uint64_t commandId = 0;  ///< its activation's
    PlanExecution ended = PlanExecution::None; ///< once its activity ended (or FA aborted it unflown)
    double percent = kUnknown, startTime = kUnknown, endTime = kUnknown;
    bool faOwned = false;         ///< FA's own: loaded by the platform, read only to MA (4.40)
};

struct CapabilityHost::PlanStore {
    std::vector<PlanEntry> plans;    ///< in the order they were first prepared for upload, or loaded
    std::vector<Airfield> airfields; ///< FA's (4.40), in the order they were first loaded
};

void CapabilityHost::PlanStoreFree::operator()(PlanStore* store) const noexcept { delete store; }

namespace {

constexpr double kPi = 3.14159265358979323846;

/// A text A-GRA's VisibleString takes: printable ASCII, `most` characters at most.
bool visible(const std::string& s, std::size_t most) noexcept {
    if (s.size() > most) return false;
    return std::all_of(s.begin(), s.end(), [](char c) { return c >= ' ' && c <= '~'; });
}

/// Left out (NaN) or finite, and not below `least`.
bool given(double v, double least = -std::numeric_limits<double>::infinity()) noexcept { return std::isnan(v) || (std::isfinite(v) && v >= least); }

/// A plan's metadata as A-GRA's schema bounds it: at a point or a path the route has, one each; its texts printable and
/// no longer than A-GRA's; its numbers left out or finite, an endurance, a fuel and a weight not below 0.
bool wellFormed(const RoutePlan& p) noexcept {
    if (!visible(p.remarksName, 32) || !visible(p.remarks, 1024)) return false;
    for (std::size_t i = 0; i < p.pointMetadata.size(); ++i) {
        const PointMetadata& m = p.pointMetadata[i];
        if (m.point >= p.waypoints.size() || m.source >= PointSource::Count) return false;
        if (!visible(m.remarksName, 32) || !visible(m.remarks, 1024) || !visible(m.fixKey, 256) || !visible(m.fixSystem, 256)) return false;
        for (std::size_t j = 0; j < i; ++j)
            if (p.pointMetadata[j].point == m.point) return false;
    }
    const std::size_t paths = std::max<std::size_t>(p.paths.size(), 1); // (without paths, the route is one)
    for (std::size_t i = 0; i < p.pathMetadata.size(); ++i) {
        const PathMetadata& m = p.pathMetadata[i];
        if (m.path >= paths) return false;
        RouteState initial = m.initial;
        double* f[RouteState::kFields];
        initial.fields(f);
        for (double* v : f)
            if (!given(*v)) return false;
        if (!given(m.enduranceS, 0.0) || !given(m.fuelKg, 0.0) || !given(m.grossWeightKg, 0.0)) return false;
        for (std::size_t j = 0; j < i; ++j)
            if (p.pathMetadata[j].path == m.path) return false;
    }
    return true;
}

/// A path of a takeoff's, a departure's, an approach's or a landing's type: FA's own alone (VI 1.2.5.2; 4.40).
bool safetyCritical(double type) noexcept {
    if (!(type >= 0.0 && type < static_cast<double>(PathType::Count))) return false;
    switch (static_cast<PathType>(static_cast<int>(type))) {
    case PathType::Takeoff:
    case PathType::Landing:
    case PathType::EmergencyLanding:
    case PathType::Airborne:          // (a departure's: CV Admin's)
    case PathType::Arcing:
    case PathType::Breaking:
    case PathType::OnDepartureRadial:
    case PathType::InitialApproach:   // (an approach's: CV Admin's recovery)
    case PathType::IntermediateApproach:
    case PathType::FinalApproach:
    case PathType::BolterWaveoff: return true;
    default: return false;
    }
}

/// A takeoff's or a landing's path, which names its airfield and runway.
bool atRunway(double type) noexcept {
    return type == static_cast<double>(PathType::Takeoff) || type == static_cast<double>(PathType::Landing) ||
           type == static_cast<double>(PathType::EmergencyLanding);
}

/// A runway's point: none (its latitude left out, and all of it), or whole - on the Earth, its altitude finite, its
/// reference left out or an AltitudeReference.
bool runwayPoint(const RunwayPoint& q, bool& given) noexcept {
    given = !std::isnan(q.latitudeRad);
    if (!given) return std::isnan(q.longitudeRad) && std::isnan(q.altitudeM) && std::isnan(q.altitudeReference);
    const double r = q.altitudeReference;
    return std::abs(q.latitudeRad) <= 0.5 * kPi && std::abs(q.longitudeRad) <= kPi && std::isfinite(q.altitudeM) &&
           (std::isnan(r) || (r >= 0.0 && r < static_cast<double>(AltitudeReference::Count) && r == std::floor(r)));
}

/// A runway's takeoff or landing coordinates: none of them, or its start with its threshold and limit if given.
bool runwayCoordinates(const RunwayCoordinates& c, bool& given) noexcept {
    bool start = false, threshold = false, limit = false;
    if (!runwayPoint(c.start, start) || !runwayPoint(c.threshold, threshold) || !runwayPoint(c.limit, limit)) return false;
    given = start;
    return start || !(threshold || limit);
}

/// An airfield as A-GRA's schema bounds it (4.40).
bool wellFormed(const Airfield& a) noexcept {
    if (a.id == 0 || a.runways.size() > Airfield::kRunways) return false;
    if (!a.icao.empty() && !(a.icao.size() == 4 && std::all_of(a.icao.begin(), a.icao.end(), [](char c) { return c >= 'A' && c <= 'Z'; })))
        return false;
    if (!std::isnan(a.qnhPa) && !(a.qnhPa >= 85000.0 && a.qnhPa <= 110000.0)) return false; // (the altimeter's range: 4.20)
    for (std::size_t i = 0; i < a.runways.size(); ++i) {
        const Runway& r = a.runways[i];
        if (r.id == 0) return false;
        for (std::size_t j = 0; j < i; ++j)
            if (a.runways[j].id == r.id) return false;
        if (!std::isnan(r.directionRad) && !(r.directionRad >= 0.0 && r.directionRad < 2.0 * kPi)) return false;
        if (!std::isnan(r.availableLengthM) && !(std::isfinite(r.availableLengthM) && r.availableLengthM > 0.0)) return false;
        bool takeoff = false, landing = false;
        if (!runwayCoordinates(r.takeoff, takeoff) || !runwayCoordinates(r.landing, landing) || !(takeoff || landing)) return false;
    }
    return true;
}

/// Its takeoff's and landing's paths each name an airfield kept, and one of its runways.
bool runwaysKept(const RoutePlan& p, const std::vector<Airfield>& airfields) noexcept {
    for (std::size_t i = 0; i < p.paths.size(); ++i) {
        if (!atRunway(p.paths[i].type)) continue;
        const PathMetadata* m = nullptr;
        for (const PathMetadata& x : p.pathMetadata)
            if (x.path == i) m = &x;
        const auto a = std::find_if(airfields.begin(), airfields.end(), [&](const Airfield& f) { return m && m->airfield && f.id == m->airfield; });
        if (a == airfields.end() || std::none_of(a->runways.begin(), a->runways.end(), [&](const Runway& r) { return r.id == m->runway; })) return false;
    }
    return true;
}

/// A plan's execution once its activity ended.
PlanExecution executionOf(const ActivityRecord& r) noexcept {
    switch (r.state) {
    case ActivityState::Completed: return PlanExecution::Complete;
    case ActivityState::Failed: return PlanExecution::Failed;
    case ActivityState::Deleted: return PlanExecution::Canceled;
    default: return r.reason == Reason::Preempted ? PlanExecution::Superseded : PlanExecution::Canceled;
    }
}

/// The platform's own end of an activity: FA aborted its plan (VI 1.2.5.7).
bool platformEnded(Reason why) noexcept { return why == Reason::Revoked || why == Reason::Restricted || why == Reason::CollisionAvoidance; }

/// Its activity flies or waits.
bool live(const CapabilityHost& host, ActivityId activity) noexcept {
    const ActivityRecord* a = activity ? host.activity(activity) : nullptr;
    return a && a->live();
}

} // namespace

void CapabilityHost::notePlanEnd(const ActivityRecord& r) noexcept {
    if (!plans_) return; // (a host moved from)
    for (PlanEntry& e : plans_->plans)
        if (e.activity == r.id) {
            e.ended = executionOf(r), e.reason = r.reason, e.endTime = r.endTime;
            e.percent = r.state == ActivityState::Completed ? 100.0 : r.progress.percent;
            if (e.state == PlanState::Activated && platformEnded(r.reason)) e.state = PlanState::Deactivated; // (FA's abort)
            return;
        }
}

CapabilityHost::PlanEntry* CapabilityHost::findPlan(PlanId id) const noexcept {
    if (!plans_ || id == 0) return nullptr;
    for (PlanEntry& e : plans_->plans)
        if (e.id == id) return &e;
    return nullptr;
}

PlanStatus CapabilityHost::planStatusOf(const PlanEntry& e) const noexcept {
    PlanStatus s;
    s.id = e.id, s.revision = e.revision, s.state = e.state, s.reason = e.reason;
    if (e.revision) s.version = e.kept.version, s.forPlanningUseOnly = e.kept.forPlanningUseOnly;
    s.activity = e.activity, s.commandId = e.commandId, s.startTime = e.startTime;
    s.execution = e.ended, s.percent = e.percent, s.endTime = e.endTime, s.faOwned = e.faOwned;
    if (const ActivityRecord* a = e.activity ? activity(e.activity) : nullptr; a && a->live()) {
        s.execution = a->state == ActivityState::Active ? PlanExecution::Executing : PlanExecution::Pending;
        s.percent = a->progress.percent;
    }
    return s;
}

void CapabilityHost::endPlanActivity(ActivityId activity, Reason reason, const sim::VehicleState& state, double now) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) endWaiting(*w, ActivityState::Canceled, reason, now); // (it held nothing)
        return;
    }
    const auto s = static_cast<std::size_t>(found);
    end(s, ActivityState::Canceled, reason, 0, now);
    release(s);           // its axes: the vehicle default...
    schedule(state, now); // ...or what waited for them
}

Reason CapabilityHost::publishPlan(const RoutePlan& plan) {
    if (plan.id == 0 || !wellFormed(plan)) return Reason::InvalidParameter;
    PlanEntry* e = findPlan(plan.id);
    if (!e || e->state != PlanState::ReadyForUpload) return Reason::WrongPlanState; // (FA listens for those prepared for upload alone)
    if (std::any_of(plan.paths.begin(), plan.paths.end(), [](const RoutePath& path) { return safetyCritical(path.type); }))
        return Reason::SafetyCriticalPlan; // (FA's own alone: VI 1.2.5.2)
    e->received = plan, e->hasReceived = true; // (published again: the later one)
    return Reason::None;
}

PlanCommandResult CapabilityHost::planCommand(PlanId id, PlanCommand command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    if (pendingSuggestions_) materialize();
    PlanCommandResult r;
    r.plan = id, r.command = command;
    r.check.commandId = options.commandId;
    PlanEntry* e = findPlan(id);
    const auto refused = [&](Reason why) { // the plan unchanged
        r.state = e ? e->state : PlanState::Inactive, r.reason = why;
        return r;
    };
    const auto failed = [&](PlanState to, Reason why) { // its own work failed
        e->state = to, e->reason = why;
        r.state = to, r.reason = why;
        return r;
    };
    const auto done = [&](PlanState to) {
        e->state = to;
        r.completed = true, r.state = to;
        return r;
    };
    const auto submit = [&](const CommandOptions& o) { // its route's NEW, or its validation
        const RoutePlan& k = e->kept;
        const RouteExtras extras{k.loiters, k.states, k.paths, k.branches, k.terminators};
        return submitWith(Command(k.route), k.waypoints, {}, o, state, now, true, nullptr, nullptr, &extras);
    };
    if (id == 0 || command >= PlanCommand::Count) return refused(Reason::InvalidParameter);
    if (!e && command != PlanCommand::PrepareForUpload) return refused(Reason::UnknownPlan);
    // kept, and its activity neither flying nor waiting
    const bool idle = e && !(e->state == PlanState::Activated && live(*this, e->activity));
    switch (command) {
    case PlanCommand::PrepareForUpload:
        if (!e) {
            if (!plans_) plans_.reset(new PlanStore), planned_ = true;
            if (plans_->plans.size() >= kPlans) {
                r.state = PlanState::PreparationForUploadFailed, r.reason = Reason::PlanStoreFull; // (a new id's: nothing kept)
                return r;
            }
            e = &plans_->plans.emplace_back();
            e->id = id;
        } else if (e->faOwned) {
            return refused(Reason::ReadOnlyPlan);
        } else if (!idle) {
            return refused(Reason::WrongPlanState);
        }
        e->received = RoutePlan{}, e->hasReceived = false, e->reason = Reason::None;
        return done(PlanState::ReadyForUpload);
    case PlanCommand::Upload:
        if (e->state != PlanState::ReadyForUpload) return refused(Reason::WrongPlanState);
        if (!e->hasReceived) return failed(PlanState::UploadFailed, Reason::PlanNotReceived);
        e->kept = std::move(e->received), e->received = RoutePlan{}, e->hasReceived = false;
        ++e->revision;
        e->reason = Reason::None, e->activity = 0, e->commandId = 0, e->ended = PlanExecution::None; // (a new version, never activated)
        e->percent = e->startTime = e->endTime = kUnknown;
        return done(PlanState::Uploaded);
    case PlanCommand::PrepareForActivation: {
        const bool kept = e->state == PlanState::Uploaded || e->state == PlanState::ReadyForActivation ||
                          e->state == PlanState::PreparationForActivationFailed || e->state == PlanState::ActivationFailed ||
                          e->state == PlanState::Deactivated || e->state == PlanState::Activated;
        if (!kept || !idle) return refused(Reason::WrongPlanState);
        if (e->kept.forPlanningUseOnly) return failed(PlanState::PreparationForActivationFailed, Reason::PlanningOnly);
        CommandOptions validation = options;
        validation.validateOnly = true;
        r.check = submit(validation);
        if (r.check.status != CommandStatus::Valid) return failed(PlanState::PreparationForActivationFailed, r.check.reason);
        e->reason = Reason::None;
        return done(PlanState::ReadyForActivation);
    }
    case PlanCommand::Activate: {
        if (e->state != PlanState::ReadyForActivation) return refused(Reason::WrongPlanState);
        if (e->kept.forPlanningUseOnly) return failed(PlanState::ActivationFailed, Reason::PlanningOnly);
        r.check = submit(options);
        if (!r.check.accepted()) return failed(PlanState::ActivationFailed, r.check.reason);
        e->activity = r.check.activity, e->commandId = options.commandId, e->startTime = now;
        e->reason = Reason::None, e->ended = PlanExecution::None, e->percent = e->endTime = kUnknown;
        done(PlanState::Activated);
        if (const ActivityRecord* a = activity(r.check.activity); a && !a->live()) notePlanEnd(*a); // (it ended at once: the plan learns its end)
        r.state = e->state;
        return r;
    }
    case PlanCommand::Deactivate:
        if (e->state == PlanState::ReadyForActivation) return done(PlanState::Deactivated);
        if (e->state != PlanState::Activated) return refused(Reason::WrongPlanState);
        if (!idle) {
            if (const ActivityRecord* a = activity(e->activity); a && a->state == ActivityState::Active) return refused(Reason::PlanExecuting); // (it flies on)
            r.check = cancel(e->activity, state, now, Caller{options.source, options.controller}); // (not flown yet: its end noted, Canceled)
            if (r.check.status != CommandStatus::Canceled) return refused(r.check.reason);
        }
        return done(PlanState::Deactivated);
    default: return refused(Reason::InvalidParameter);
    }
}

PlanCommandResult CapabilityHost::abortPlan(PlanId id, Reason reason, const sim::VehicleState& state, double now) {
    PlanCommandResult r;
    r.plan = id, r.command = PlanCommand::Deactivate;
    PlanEntry* e = findPlan(id);
    if (!e) {
        r.reason = Reason::UnknownPlan;
        return r;
    }
    if (e->state != PlanState::ReadyForActivation && e->state != PlanState::Activated) {
        r.state = e->state, r.reason = Reason::WrongPlanState;
        return r;
    }
    if (reason == Reason::None) reason = Reason::Restricted;
    if (e->state == PlanState::Activated && live(*this, e->activity)) {
        endPlanActivity(e->activity, reason, state, now); // (its end noted: Canceled)
    } else if (e->state == PlanState::ReadyForActivation) {
        e->ended = PlanExecution::Canceled, e->reason = reason, e->endTime = now; // (DEACTIVATED, its execution CANCELED: VI 1.2.5.7)
    }
    e->state = PlanState::Deactivated;
    r.completed = true, r.state = e->state;
    return r;
}

Reason CapabilityHost::removePlan(PlanId id) {
    PlanEntry* e = findPlan(id);
    if (!e) return Reason::UnknownPlan;
    if (e->faOwned) return Reason::ReadOnlyPlan;
    if (e->state == PlanState::Activated && live(*this, e->activity)) return Reason::WrongPlanState;
    plans_->plans.erase(plans_->plans.begin() + (e - plans_->plans.data()));
    return Reason::None;
}

bool CapabilityHost::planStatus(PlanId id, PlanStatus& out) const {
    const PlanEntry* e = findPlan(id);
    if (!e) return false;
    out = planStatusOf(*e);
    return true;
}

std::vector<PlanStatus> CapabilityHost::plans() const {
    std::vector<PlanStatus> out;
    if (!plans_) return out;
    out.reserve(plans_->plans.size());
    for (const PlanEntry& e : plans_->plans) out.push_back(planStatusOf(e));
    return out;
}

bool CapabilityHost::plan(PlanId id, RoutePlan& out) const {
    const PlanEntry* e = findPlan(id);
    if (!e || !e->revision) return false;
    out = e->kept;
    return true;
}

Reason CapabilityHost::loadPlan(const RoutePlan& plan) {
    if (plan.id == 0 || !wellFormed(plan)) return Reason::InvalidParameter;
    static const std::vector<Airfield> none;
    if (!runwaysKept(plan, plans_ ? plans_->airfields : none)) return Reason::UnknownAirfield;
    PlanEntry* e = findPlan(plan.id);
    if (e && e->state == PlanState::Activated && live(*this, e->activity)) return Reason::WrongPlanState;
    if (!e) {
        if (!plans_) plans_.reset(new PlanStore), planned_ = true;
        if (plans_->plans.size() >= kPlans) return Reason::PlanStoreFull;
        e = &plans_->plans.emplace_back();
        e->id = plan.id;
    }
    e->kept = plan, e->received = RoutePlan{}, e->hasReceived = false;
    ++e->revision;
    e->faOwned = true, e->state = PlanState::Uploaded, e->reason = Reason::None; // (kept, as an upload keeps one)
    e->activity = 0, e->commandId = 0, e->ended = PlanExecution::None;
    e->percent = e->startTime = e->endTime = kUnknown;
    return Reason::None;
}

Reason CapabilityHost::loadAirfield(const Airfield& airfield) {
    if (!wellFormed(airfield)) return Reason::InvalidParameter;
    if (!plans_) plans_.reset(new PlanStore), planned_ = true;
    for (Airfield& a : plans_->airfields)
        if (a.id == airfield.id) { // (in its place, its revision one more)
            const std::uint32_t revision = a.revision + 1;
            a = airfield, a.revision = revision;
            return Reason::None;
        }
    if (plans_->airfields.size() >= kAirfields) return Reason::PlanStoreFull;
    plans_->airfields.push_back(airfield);
    plans_->airfields.back().revision = 1;
    return Reason::None;
}

std::vector<Airfield> CapabilityHost::airfields() const { return plans_ ? plans_->airfields : std::vector<Airfield>{}; }

bool CapabilityHost::airfield(AirfieldId id, Airfield& out) const {
    if (!plans_ || id == 0) return false;
    for (const Airfield& a : plans_->airfields)
        if (a.id == id) {
            out = a;
            return true;
        }
    return false;
}

} // namespace fsim::control
