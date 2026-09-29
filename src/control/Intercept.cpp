// A-GRA's route intercept (docs/flight-autonomy.md, 4.47; ROUTE_INTERCEPT, its MA_RoutePlanInterceptType): a route plan the
// vehicle keeps, joined where its method chooses from where the aircraft is, and flown from there as the plan's activation flies
// it - the plan activated by it. Its activity flies the plan's route, a RouteCommand, and keeps the intercept beside it (not in
// the command variant: 4.46). Its checks are the route's; a join on a leg short of its end point is laid into the route as one
// point more. Apart from the host's other code, so that growing either moves neither.
#include "control/PlanStore.h"

#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/ControlStack.h"
#include "fsim/ControllerRegistry.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
/// The largest id a mode's field names exactly: a double's whole numbers.
constexpr double kMaxId = 9007199254740992.0;
/// A join this near its leg's end point is at the point.
constexpr double kAtPointM = 1.0;

bool whole(double v, double least, double below) noexcept { return std::isfinite(v) && v == std::floor(v) && v >= least && v < below; }

double stretched(double lat) noexcept { return std::log(std::tan(0.25 * kPi + 0.5 * lat)); }

/// The point `m` along a straight leg - a great circle's or a rhumb line's - from its start.
void pointAlong(const route::Leg& leg, double m, double& lat, double& lon) noexcept {
    if (!leg.rhumb) {
        geo::destination(leg.latA, leg.lonA, leg.courseOutRad, m, lat, lon);
        return;
    }
    const double f = leg.lengthM > 0.0 ? m / leg.lengthM : 0.0;
    if (std::abs(leg.dPsi) > 1e-12) { // (its latitude by its share of the length; its longitude on the line in the projection)
        lat = leg.latA + f * (leg.latB - leg.latA);
        lon = geo::wrapPi(leg.lonA + leg.dLon * (stretched(lat) - leg.psiA) / leg.dPsi);
    } else { // (east or west: along the parallel)
        lat = leg.latA, lon = geo::wrapPi(leg.lonA + f * leg.dLon);
    }
}

/// The aircraft as a join sees it: where it is, its track, its true airspeed, its turn's radius (0: it turns at once), the wind.
struct Mover {
    double lat = 0.0, lon = 0.0, trackRad = 0.0, tasMs = 0.0, radiusM = 0.0, windNorthMs = 0.0, windEastMs = 0.0;
};

/// Its speed over the ground on `courseRad`, crabbed into the wind; 0 where the wind across is its airspeed or more.
double groundSpeed(const Mover& a, double courseRad) noexcept {
    const double cn = std::cos(courseRad), ce = std::sin(courseRad);
    const double along = a.windNorthMs * cn + a.windEastMs * ce, across = -a.windNorthMs * ce + a.windEastMs * cn;
    if (std::abs(across) >= a.tasMs) return 0.0;
    return std::max(std::sqrt(a.tasMs * a.tasMs - across * across) + along, 0.0);
}

/// How long it takes to reach (lat, lon): turning from its track toward it on its radius - either way round, the sooner - then
/// straight to it at its ground speed there; the turn at its airspeed in still air. Infinite where it cannot.
double timeTo(const Mover& a, double lat, double lon) noexcept {
    double n = 0.0, e = 0.0;
    geo::localNorthEastM(a.lat, a.lon, lat, lon, n, e);
    if (!(a.radiusM > 0.0)) { // (a rotorcraft: straight there)
        const double g = groundSpeed(a, std::atan2(e, n));
        return g > 0.1 ? std::hypot(n, e) / g : kInf;
    }
    const double sx = std::sin(a.trackRad), cx = std::cos(a.trackRad);
    double best = kInf;
    for (const double side : {1.0, -1.0}) { // (a right turn, then a left)
        // the point from the turn's centre, off the track that side
        const double dn = n + side * a.radiusM * sx, de = e - side * a.radiusM * cx;
        const double d = std::hypot(dn, de);
        if (d < a.radiusM) continue; // (inside that turn's circle: not reached that way)
        const double beta = std::atan2(de, dn), gamma = std::acos(a.radiusM / d);
        const double out = side > 0.0 ? beta - gamma + 0.5 * kPi : beta + gamma - 0.5 * kPi; // (the course it leaves the turn on)
        double sweep = std::remainder(side > 0.0 ? out - a.trackRad : a.trackRad - out, 2.0 * kPi);
        if (sweep < 0.0) sweep = sweep > -1e-9 ? 0.0 : sweep + 2.0 * kPi;
        const double g = groundSpeed(a, out);
        if (!(g > 0.1)) continue;
        best = std::min(best, a.radiusM * sweep / std::max(a.tasMs, 0.1) + std::sqrt(d * d - a.radiusM * a.radiusM) / g);
    }
    return best;
}

/// A segment it may join: the leg into point `point` from `from` (-1: none), straight between two points or not.
struct Candidate {
    std::uint32_t point = 0;
    std::int32_t from = -1;
    bool straight = false;
    route::Leg leg;
};

/// The route's points it may join, in its flight order (4.36): on `path`, from its first point along the links until the path is
/// left; without one, from the route's start, once round. Their count.
std::uint32_t orderOf(const RoutePlan& plan, const RoutePath* path, std::uint32_t* order) noexcept {
    const auto n = static_cast<std::uint32_t>(std::min<std::size_t>(plan.waypoints.size(), PathStore::kWaypoints));
    const auto pathCount = static_cast<std::uint32_t>(std::min<std::size_t>(plan.paths.size(), PathStore::kRoutePaths));
    if (n == 0) return 0;
    const std::uint32_t start = path ? path->first : whole(plan.route.start, 0.0, static_cast<double>(n)) ? static_cast<std::uint32_t>(plan.route.start) : 0u;
    std::int32_t loop = -1;
    std::int16_t bad = -1;
    const int got = route::flightOrder(plan.waypoints.data(), n, plan.paths.data(), pathCount, start, !path && plan.route.repeat == 1.0, order, loop, bad);
    if (got <= 0) { // (links that make no order: its start alone - the route's checks name what is wrong)
        order[0] = start < n ? start : 0u;
        return 1;
    }
    auto count = static_cast<std::uint32_t>(got);
    if (path) // (on the path alone)
        for (std::uint32_t k = 0; k < count; ++k)
            if (order[k] < path->first || order[k] - path->first >= path->count) count = k;
    return count;
}

/// A straight leg between two points, joined short of its end point: no arc, no leg a civil path terminator lays, not out of a
/// loiter point, neither end in a frame (4.47).
bool straightLeg(const Waypoint& a, const Waypoint& b) noexcept {
    return isHold(b.terminator) && a.turn != static_cast<double>(TurnType::StartTurn) && b.turn != static_cast<double>(TurnType::EndTurn) &&
           a.kind != static_cast<double>(EndPointKind::LoiterPoint) && isHold(a.frame) && isHold(b.frame);
}

/// Its execution once its activity ended, as a plan's (4.39).
PlanExecution executionOf(const ActivityRecord& r) noexcept {
    switch (r.state) {
    case ActivityState::Completed: return PlanExecution::Complete;
    case ActivityState::Failed: return PlanExecution::Failed;
    case ActivityState::Deleted: return PlanExecution::Canceled;
    default: return r.reason == Reason::Preempted ? PlanExecution::Superseded : PlanExecution::Canceled;
    }
}

/// The path holding point `k` (its id; 0 without paths).
std::uint64_t pathOf(const PathStore& store, std::int32_t k) noexcept {
    for (std::uint32_t i = 0; i < std::min<std::uint32_t>(store.routePathCount, static_cast<std::uint32_t>(PathStore::kRoutePaths)); ++i) {
        const RoutePath& p = store.routePaths[i];
        if (k >= 0 && static_cast<std::uint32_t>(k) >= p.first && static_cast<std::uint32_t>(k) - p.first < p.count) return p.id;
    }
    return 0;
}

} // namespace

// --- Its plan, and where it joins it ----------------------------------------------------------------

const RoutePlan* CapabilityHost::interceptPlan(const InterceptCommand& c, Reason& why, std::int16_t& field) const noexcept {
    why = Reason::InvalidParameter, field = 0;
    if (!whole(c.plan, 1.0, kMaxId + 1.0)) return nullptr;
    const PlanEntry* e = findPlan(static_cast<PlanId>(c.plan));
    if (!e) {
        why = Reason::UnknownPlan;
        return nullptr;
    }
    // kept, in a state its activation leaves from - its activity live or not: the intercept replaces it, as any NEW does - and
    // not for planning use only
    const bool from = e->state == PlanState::Uploaded || e->state == PlanState::ReadyForActivation ||
                      e->state == PlanState::PreparationForActivationFailed || e->state == PlanState::ActivationFailed ||
                      e->state == PlanState::Deactivated || e->state == PlanState::Activated;
    if (!e->revision || !from) {
        why = Reason::WrongPlanState;
        return nullptr;
    }
    if (e->kept.forPlanningUseOnly) {
        why = Reason::PlanningOnly;
        return nullptr;
    }
    const RoutePlan& plan = e->kept;
    // its path, one the plan has; its method, one there is; its bounds, points on its way - the earliest not after the latest
    const RoutePath* path = nullptr;
    if (!isHold(c.path)) {
        for (const RoutePath& p : plan.paths)
            if (static_cast<double>(p.id) == c.path) path = &p;
        if (!path) return field = 1, nullptr;
    }
    if (!isHold(c.method) && !whole(c.method, 0.0, static_cast<double>(InterceptMethod::Count))) return field = 2, nullptr;
    std::uint32_t order[PathStore::kWaypoints];
    const std::uint32_t count = orderOf(plan, path, order);
    if (count == 0) return field = path ? 1 : 0, nullptr;
    auto position = [&](double point) {
        for (std::uint32_t k = 0; k < count; ++k)
            if (static_cast<double>(order[k]) == point) return static_cast<int>(k);
        return -1;
    };
    const int earliest = isHold(c.earliest) ? 0 : position(c.earliest), latest = isHold(c.latest) ? static_cast<int>(count) - 1 : position(c.latest);
    if (earliest < 0) return field = 3, nullptr;
    if (latest < 0 || latest < earliest) return field = 4, nullptr;
    why = Reason::None, field = -1;
    return &plan;
}

void CapabilityHost::layIntercept(const RoutePlan& plan, const InterceptCommand& c, const sim::VehicleState& s, std::vector<Waypoint>& waypoints,
                                  std::vector<RoutePath>& paths, RouteCommand& route, InterceptJoin& join) const noexcept {
    const auto n = static_cast<std::uint32_t>(std::min<std::size_t>(plan.waypoints.size(), PathStore::kWaypoints));
    waypoints.assign(plan.waypoints.begin(), plan.waypoints.begin() + n);
    paths.assign(plan.paths.begin(), plan.paths.end());
    route = plan.route, join = InterceptJoin{};
    if (n == 0) return;
    const RoutePath* path = nullptr;
    for (const RoutePath& p : plan.paths)
        if (!isHold(c.path) && static_cast<double>(p.id) == c.path) path = &p;
    std::uint32_t order[PathStore::kWaypoints];
    const std::uint32_t count = orderOf(plan, path, order);
    std::uint32_t first = 0, last = count - 1;
    for (std::uint32_t k = 0; k < count; ++k) {
        if (!isHold(c.earliest) && static_cast<double>(order[k]) == c.earliest) first = k;
        if (!isHold(c.latest) && static_cast<double>(order[k]) == c.latest) last = k;
    }
    // the aircraft as the join sees it
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    const WindEstimate wind = checkWind(s);
    Mover a;
    a.lat = s.latitudeRad, a.lon = s.longitudeRad;
    const double ground = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    a.trackRad = ground > 1.0 ? std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) : s.eulerRad[2];
    const double cruise = std::isfinite(performance_.cruiseTasMs) ? performance_.cruiseTasMs : 10.0;
    a.tasMs = std::max({s.airspeedTrueMs, hovers ? cruise : 10.0, 1.0});
    a.radiusM = hovers ? 0.0 : performance_.turnRadiusM(a.tasMs);
    if (!std::isfinite(a.radiusM)) a.radiusM = 0.0;
    a.windNorthMs = wind.northMs, a.windEastMs = wind.eastMs;
    // each segment it may join, and the best: the first (left out), else the nearest point, or the nearest place on a leg, or the soonest
    const bool rhumb = plan.route.projection == 1.0;
    const double method = c.method;
    std::uint32_t best = first;
    double bestScore = kInf, bestAlong = -1.0; // (-1: at the point)
    route::Leg bestLeg;
    for (std::uint32_t k = first; k <= last && !isHold(method); ++k) {
        Candidate cand;
        cand.point = order[k];
        const Waypoint& b = waypoints[cand.point];
        if (k > 0) {
            cand.from = static_cast<std::int32_t>(order[k - 1]);
            const Waypoint& from = waypoints[static_cast<std::size_t>(cand.from)];
            cand.straight = straightLeg(from, b);
            if (cand.straight) cand.leg = route::makeLeg(from.latitudeRad, from.longitudeRad, b.latitudeRad, b.longitudeRad, rhumb);
        }
        double score = kInf, along = -1.0;
        if (method == static_cast<double>(InterceptMethod::Discrete) || !cand.straight) {
            score = method == static_cast<double>(InterceptMethod::Soonest) ? timeTo(a, b.latitudeRad, b.longitudeRad)
                                                                           : geo::distanceM(a.lat, a.lon, b.latitudeRad, b.longitudeRad);
        } else if (method == static_cast<double>(InterceptMethod::ShortestDistance)) { // (the foot of the perpendicular, on the leg)
            const double length = cand.leg.lengthM;
            along = std::clamp(route::onLeg(cand.leg, a.lat, a.lon).alongM, 0.0, length);
            double lat = 0.0, lon = 0.0;
            pointAlong(cand.leg, along, lat, lon);
            score = geo::distanceM(a.lat, a.lon, lat, lon);
        } else { // Soonest: sampled along the leg, then narrowed round the best
            const double length = cand.leg.lengthM;
            auto at = [&](double m) {
                double lat = 0.0, lon = 0.0;
                pointAlong(cand.leg, m, lat, lon);
                return timeTo(a, lat, lon);
            };
            constexpr int kSamples = 32;
            for (int i = 0; i <= kSamples; ++i) {
                const double m = length * static_cast<double>(i) / kSamples, t = at(m);
                if (t < score) score = t, along = m;
            }
            double lo = std::max(along - length / kSamples, 0.0), hi = std::min(along + length / kSamples, length);
            constexpr double kGolden = 0.6180339887498949;
            for (int i = 0; i < 40 && hi - lo > 0.01; ++i) {
                const double m1 = hi - kGolden * (hi - lo), m2 = lo + kGolden * (hi - lo);
                if (at(m1) <= at(m2)) hi = m2;
                else lo = m1;
            }
            if (const double m = 0.5 * (lo + hi), t = at(m); t < score) score = t, along = m;
        }
        const bool better = std::isfinite(bestScore) ? score < bestScore - 1e-9 * std::max(bestScore, 1.0) : score < bestScore; // (a tie: the earlier)
        if (better) best = k, bestScore = score, bestAlong = along, bestLeg = cand.leg;
    }
    if (bestAlong >= 0.0 && bestAlong < kAtPointM && best > first) best -= 1, bestAlong = -1.0; // (at the leg's start: the point before, itself a candidate)
    const std::uint32_t point = order[best];
    join.point = static_cast<std::int32_t>(point);
    route.start = static_cast<double>(point);
    // short of its end point: laid in, room left for it (256 points, 16 paths at most) - else at the point
    const bool shortOf = bestAlong >= 0.0 && bestAlong < bestLeg.lengthM - kAtPointM;
    if (!shortOf || n >= PathStore::kWaypoints || (!paths.empty() && paths.size() >= PathStore::kRoutePaths) || best == 0) return;
    const Waypoint& from = waypoints[order[best - 1]];
    const Waypoint& to = waypoints[point];
    Waypoint j;
    pointAlong(bestLeg, bestAlong, j.latitudeRad, j.longitudeRad);
    const double refFrom = isHold(from.altitudeReference) ? 0.0 : from.altitudeReference, refTo = isHold(to.altitudeReference) ? 0.0 : to.altitudeReference;
    if (!isHold(from.altitudeM) && !isHold(to.altitudeM) && refFrom == refTo && bestLeg.lengthM > 0.0) // (its altitude along the profile there)
        j.altitudeM = from.altitudeM + bestAlong / bestLeg.lengthM * (to.altitudeM - from.altitudeM);
    else
        j.altitudeM = to.altitudeM;
    j.altitudeReference = to.altitudeReference;
    j.speed = to.speed, j.speedReference = to.speedReference, j.speedOptimization = to.speedOptimization; // (the leg's)
    j.turn = static_cast<double>(TurnType::FlyBy);
    j.next = static_cast<double>(point);
    waypoints.push_back(j); // (its room reserved)
    if (paths.empty()) {
        if (isHold(waypoints[n - 1].next)) waypoints[n - 1].next = -1.0; // (the route's end there, as it was: not on to the join)
    } else {
        std::uint64_t id = 1; // (a path of its own, its id one the plan has not)
        while (std::any_of(paths.begin(), paths.end(), [id](const RoutePath& p) { return p.id == id; })) ++id;
        RoutePath own;
        own.id = id, own.first = n, own.count = 1;
        paths.push_back(own);
    }
    route.start = static_cast<double>(n);
    join.laid = static_cast<std::int32_t>(n), join.latitudeRad = j.latitudeRad, join.longitudeRad = j.longitudeRad;
}

Reason CapabilityHost::prepareLate(Command& setpoint, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log, const PatternShape* shape,
                                   const RouteExtras& extras) {
    if (extras.late == SetpointKind::Marshall) return extras.marshall ? prepareMarshall(setpoint, *extras.marshall, state, log, shape) : Reason::InvalidParameter;
    auto* route = std::get_if<RouteCommand>(&setpoint);
    const int index = catalog_->indexOf(SetpointKind::Route);
    if (extras.late != SetpointKind::Intercept || !route || index < 0) return Reason::WrongCommandType;
    // checked as the plan's route is, from where it joins it: the route's checks
    RouteExtras plain = extras;
    plain.late = SetpointKind::Count, plain.intercept = nullptr;
    if (!extras.intercept) return prepare(static_cast<std::size_t>(index), setpoint, waypoints, {}, state, log, nullptr, nullptr, &plain); // (resumed)
    Reason why = Reason::None;
    std::int16_t field = -1;
    const RoutePlan* plan = interceptPlan(*extras.intercept, why, field);
    if (!plan) {
        log.result.index = field;
        return why;
    }
    PlanStore& store = *plans_; // (a plan kept: the store made)
    store.laid.reserve(PathStore::kWaypoints), store.laidPaths.reserve(PathStore::kRoutePaths); // (room made at its first NEW)
    layIntercept(*plan, *extras.intercept, state, store.laid, store.laidPaths, *route, store.laidJoin);
    plain.loiters = Span<const RouteLoiter>(plan->loiters.data(), plan->loiters.size());
    plain.states = Span<const RouteState>(plan->states.data(), plan->states.size());
    plain.paths = Span<const RoutePath>(store.laidPaths.data(), store.laidPaths.size());
    plain.branches = Span<const RouteBranch>(plan->branches.data(), plan->branches.size());
    plain.terminators = Span<const RouteTerminator>(plan->terminators.data(), plan->terminators.size());
    return prepare(static_cast<std::size_t>(index), setpoint, Span<const Waypoint>(store.laid.data(), store.laid.size()), {}, state, log, nullptr, nullptr,
                   &plain);
}

// --- The host ---------------------------------------------------------------------------------------

CommandResult CapabilityHost::submit(const InterceptCommand& intercept, const CommandOptions& options, const sim::VehicleState& state, double now) {
    if (catalog_->indexOf(SetpointKind::Intercept) < 0) { // (what the vehicle cannot fly: why)
        details_.clear();
        return rejected(missing("fsim.guidance.intercept"));
    }
    RouteExtras extras;
    extras.late = SetpointKind::Intercept, extras.intercept = &intercept;
    const CommandResult r = submitWith(Command(RouteCommand{}), {}, {}, options, state, now, true, nullptr, nullptr, &extras);
    if (!r.accepted() || !plans_) return r;
    const PlanStore& store = *plans_;
    interceptedBy(static_cast<PlanId>(intercept.plan), r.activity, options.commandId, now);
    if (!(r.flags & kDeferred)) {
        if (config_->path) config_->path->intercept = intercept, config_->path->interceptJoin = store.laidJoin; // (beside its route)
    } else if (Waiting* w = waitingEntry(r.activity)) { // (kept as it was laid now, for its readback: laid afresh as it starts)
        const PlanEntry* e = findPlan(static_cast<PlanId>(intercept.plan));
        const RoutePlan& plan = e->kept;
        w->waypoints.assign(store.laid.begin(), store.laid.end()), w->paths.assign(store.laidPaths.begin(), store.laidPaths.end());
        w->loiters.assign(plan.loiters.begin(), plan.loiters.end()), w->states.assign(plan.states.begin(), plan.states.end());
        w->branches.assign(plan.branches.begin(), plan.branches.end()), w->terminators.assign(plan.terminators.begin(), plan.terminators.end());
        RouteCommand laid = plan.route;
        laid.start = static_cast<double>(store.laidJoin.laid >= 0 ? store.laidJoin.laid : store.laidJoin.point);
        w->command = Command(laid), w->firstStart = kHold, w->join = store.laidJoin;
    }
    return r;
}

void CapabilityHost::launchedIntercept(const Waiting& w) noexcept {
    config_->path->intercept = w.intercept;
    config_->path->interceptJoin = w.resumed || !plans_ ? w.join : plans_->laidJoin; // (a resumed one's, as it was laid)
}

CommandResult CapabilityHost::update(ActivityId activity, const InterceptCommand& intercept, const sim::VehicleState& state, Caller caller) noexcept {
    (void)intercept, (void)state;
    details_.clear();
    const int live = liveSlot(activity);
    const ActivityRecord* record = live >= 0 ? &records_[static_cast<std::size_t>(live)] : nullptr;
    if (!record)
        if (const Waiting* w = waitingEntry(activity)) record = &w->record;
    if (!record) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    if (const Reason why = addresses(*record, caller); why != Reason::None) return rejected(why, activity, activity);
    const CapabilityDescriptor& d = catalog_->descriptor(record->capability);
    if (!(d.interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity); // (an intercept's: a new one replaces it)
    return rejected(Reason::WrongCommandType, activity);
}

bool CapabilityHost::intercept(ActivityId activity, InterceptCommand& out, InterceptJoin& join) const noexcept {
    if (const int live = liveSlot(activity); live >= 0) {
        const auto s = static_cast<std::size_t>(live);
        if (!isCascade(s) || !config_->path || catalog_->descriptor(records_[s].capability).setpoint != SetpointKind::Intercept) return false;
        out = config_->path->intercept, join = config_->path->interceptJoin;
        return true;
    }
    const Waiting* w = waitingEntry(activity);
    if (!w || w->support || catalog_->descriptor(w->record.capability).setpoint != SetpointKind::Intercept) return false;
    out = w->intercept, join = w->join;
    return true;
}

bool CapabilityHost::interceptStatus(ActivityId activity, const sim::VehicleState& state, InterceptStatus& out) const noexcept {
    out = InterceptStatus{};
    const ActivityRecord* a = this->activity(activity);
    if (!a || a->capability >= catalog_->size() || catalog_->descriptor(a->capability).setpoint != SetpointKind::Intercept) return false;
    out.plan = planOf(activity);
    out.execution = a->live() ? (a->state == ActivityState::Active ? PlanExecution::Executing : PlanExecution::Pending) : executionOf(*a);
    InterceptCommand given;
    InterceptJoin join;
    if (!intercept(activity, given, join)) return true; // (ended: its plan and execution alone)
    out.joined = join.point, out.laid = join.laid, out.joinLatitudeRad = join.latitudeRad, out.joinLongitudeRad = join.longitudeRad;
    // its segments, as its route's behaviour has them: the join's point named as the leg it joins
    const int live = liveSlot(activity);
    SegmentEstimate e;
    if (live < 0 || !isCascade(static_cast<std::size_t>(live)) || !config_->path || !runtime_->segments(static_cast<std::size_t>(live), e)) return true;
    const PathStore& store = *config_->path;
    const double worldNow = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    const double speed = std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]);
    auto place = [&](std::int32_t k, double& lat, double& lon) {
        if (k < 0 || static_cast<std::uint32_t>(k) >= store.count) return false;
        lat = store.waypoints[k].latitudeRad, lon = store.waypoints[k].longitudeRad;
        return true;
    };
    auto fill = [&](SegmentStatus& seg, std::int32_t point, std::int32_t from, double toGoM) {
        seg.path = pathOf(store, point), seg.point = point;
        seg.pointId = point >= 0 && static_cast<std::uint32_t>(point) < store.count ? store.waypoints[point].id : 0;
        seg.captureDistanceM = toGoM;
        seg.captureTimeS = toGoM == 0.0 ? kHold : speed > 0.1 ? worldNow + toGoM / speed : kHold;
        double la = 0.0, lo = 0.0, lb = 0.0, lob = 0.0;
        if (place(from, la, lo) && place(point, lb, lob) && (la != lb || lo != lob)) { // (its heading, and the aircraft's velocity against it)
            seg.headingRad = geo::bearingRad(la, lo, lb, lob);
            const double cn = std::cos(seg.headingRad), ce = std::sin(seg.headingRad);
            seg.alongMs = state.velocityNedMs[0] * cn + state.velocityNedMs[1] * ce;
            seg.acrossMs = -state.velocityNedMs[0] * ce + state.velocityNedMs[1] * cn;
        }
    };
    const std::int32_t laid = join.laid;
    std::int32_t current = e.current, previous = e.previous, next = e.next;
    double currentM = e.currentM, nextM = e.nextM;
    std::int32_t from = previous;
    if (laid >= 0 && current == laid) { // (flying to the point laid in: the leg it joins is the segment)
        const std::int32_t to = next;
        currentM = std::isfinite(nextM) ? currentM + nextM : kHold;
        from = laid, current = to, previous = -1;
        next = -1, nextM = kHold;
        if (to >= 0) { // (the point after the one it joins at: the route's order in the store, or the next as they are)
            const auto t = static_cast<std::uint32_t>(to);
            if (store.routeLinked) {
                for (std::uint32_t k = 0; k + 1 < store.routeFlown; ++k)
                    if (store.routeOrder[k] == t) next = store.routeOrder[k + 1];
            } else if (t + 1 < store.count) {
                next = static_cast<std::int32_t>(t + 1);
            }
            double la = 0.0, lo = 0.0, lb = 0.0, lob = 0.0;
            if (next >= 0 && place(to, la, lo) && place(next, lb, lob)) nextM = geo::distanceM(la, lo, lb, lob);
        }
    }
    if (previous == laid) previous = -1, from = laid; // (from the point laid in: no segment of the plan's flown before)
    if (from < 0 && current >= 0) // (the leg it lies on: from the point before it in the route's order)
        for (std::uint32_t k = 1; k < store.routeFlown && store.routeLinked; ++k)
            if (store.routeOrder[k] == static_cast<std::uint32_t>(current)) from = store.routeOrder[k - 1];
    if (previous >= 0) {
        out.hasPrevious = true;
        std::int32_t before = -1;
        for (std::uint32_t k = 1; k < store.routeFlown && store.routeLinked; ++k)
            if (store.routeOrder[k] == static_cast<std::uint32_t>(previous)) before = store.routeOrder[k - 1];
        if (!store.routeLinked && previous > 0) before = previous - 1;
        fill(out.previous, previous, before, 0.0);
        out.previous.captureTimeS = std::isfinite(e.previousCaptureS) ? worldNow - (state.simTime - e.previousCaptureS) : kHold; // (its clock the world's)
    }
    if (current >= 0) {
        out.hasCurrent = true;
        fill(out.current, current, from, currentM);
        out.current.loiter = e.loiter, out.current.orbits = e.orbits, out.current.loiterEndS = e.loiterEndS;
    }
    if (next >= 0) {
        out.hasNext = true;
        fill(out.next, next, current, std::isfinite(currentM) && std::isfinite(nextM) ? currentM + nextM : kHold);
    }
    return true;
}

// --- The plan it activates ------------------------------------------------------------------------

void CapabilityHost::interceptedBy(PlanId id, ActivityId activity, std::uint64_t commandId, double now) noexcept {
    PlanEntry* e = findPlan(id);
    if (!e) return;
    e->state = PlanState::Activated, e->activity = activity, e->commandId = commandId, e->startTime = now;
    e->reason = Reason::None, e->ended = PlanExecution::None, e->percent = e->endTime = kUnknown;
    if (const ActivityRecord* a = this->activity(activity); a && !a->live()) notePlanEnd(*a); // (it ended at once: the plan learns its end)
}

PlanId CapabilityHost::planOf(ActivityId activity) const noexcept {
    if (!plans_ || !activity) return 0;
    for (const PlanEntry& e : plans_->plans)
        if (e.activity == activity) return e.id;
    return 0;
}

// --- Flown: the route's behaviour --------------------------------------------------------------------

bool ControlStack::segments(std::size_t slot, SegmentEstimate& out) const noexcept {
    if (slot >= kSlotCount || !behaviors_[slot] || started_[slot] != config_->slots[slot].generation) return false;
    return behaviors_[slot]->segments(out);
}

bool RouteBehavior::segments(SegmentEstimate& out) const noexcept {
    const route::Plan& p = *plan_;
    if (!planned_ || p.count == 0) return false;
    out = SegmentEstimate{};
    out.current = static_cast<std::int32_t>(p.named(segment_));
    // the point flown from: none on the route's entry, or at a first point it does not come back to
    if (!(firstLap_ && segment_ == p.start) && (segment_ > 0 || (p.repeat && p.loop == 0) || !firstLap_)) {
        out.previous = static_cast<std::int32_t>(p.named(p.before(segment_, firstLap_)));
        out.previousCaptureS = segmentStartS_;
    }
    const double routeM = finishedM_ + inPieceM_;
    out.currentM = ended_ ? 0.0 : std::max(segmentM_ - (routeM - segmentStartM_), 0.0);
    if (!ended_ && p.leaves(segment_)) {
        out.next = static_cast<std::int32_t>(p.named(p.next(segment_)));
        out.nextM = p.legOut(segment_).lengthM;
    }
    if (loitering_ && loiter_) { // (its loiter flies: its orbits, and when it will be left)
        ActivityProgress q;
        loiter_->progress(q);
        out.loiter = true, out.orbits = q.laps, out.loiterEndS = loiterEndsS_;
    }
    return true;
}

// --- Registered ---------------------------------------------------------------------------------

void registerIntercept(ControllerRegistry& r) {
    constexpr double now = kHold;
    auto p = [](const char* name, double lo, double hi) { return ParameterInfo{name, "", lo, hi, now, true, Constraint::None, Constraint::None}; };
    // its setpoint's fields, in order (the C ABI's too)
    BehaviorTraits intercept;
    intercept.persistence = Persistence::Terminating; // (as its plan's route: at its end it completes)
    intercept.parameters = {p("plan", 1.0, kMaxId), p("path", 0.0, kMaxId), p("method", 0.0, static_cast<double>(InterceptMethod::Count) - 1.0),
                            p("earliest", 0.0, static_cast<double>(PathStore::kWaypoints) - 1.0),
                            p("latest", 0.0, static_cast<double>(PathStore::kWaypoints) - 1.0)};
    intercept.uses = {"fsim.flight.velocity", "fsim.flight.position"};
    intercept.mode = FlightMode::RouteIntercept;
    intercept.setpoint = SetpointKind::Intercept;
    // flown as its plan's route: the slot's command is a RouteCommand, the plan's points in the path store (the intercept beside them)
    r.addBehavior("intercept", [] { return std::make_unique<RouteBehavior>(); }, std::move(intercept));
}

} // namespace fsim::control
