// A route's conditional branches (docs/flight-autonomy.md, 4.37; ADR-29 FA-6e2; A-GRA's ConditionalPathSegment): checked
// and kept by the host, commanded by the operator where they take its input, and taken by the route's behaviour as the
// aircraft comes to their point - the route planned again from there. Its own translation unit, last in the library: what
// is added before the runtime's code moves it (the A/B's lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Route.h"
#include "fsim/ControlStack.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

/// `value` compared so with `with` (A-GRA's EqualityExpressionEnum: the value under test on the left).
bool compared(double value, double how, double with) noexcept {
    switch (static_cast<Comparison>(static_cast<int>(how))) {
    case Comparison::Greater: return value > with;
    case Comparison::GreaterEqual: return value >= with;
    case Comparison::Less: return value < with;
    case Comparison::LessEqual: return value <= with;
    case Comparison::Equal: return value == with;
    case Comparison::NotEqual: return value != with;
    default: return false;
    }
}

} // namespace

namespace route {

void takeBranches(PathStore& store, Span<const RouteBranch> branches) noexcept {
    store.routeBranchCount = static_cast<std::uint32_t>(std::min(branches.size(), PathStore::kRouteBranches));
    std::copy_n(branches.data(), store.routeBranchCount, store.routeBranches);
    store.routeCommanded = 0;
}

} // namespace route

// --- the host's ------------------------------------------------------------------------------

Reason CapabilityHost::checkBranches(route::Plan& p, Span<const Waypoint> waypoints, Span<const RouteBranch> branches, bool repeat,
                                     CommandResult& detail) const noexcept {
    p.branchCount = 0;
    if (branches.empty()) return Reason::None;
    // named where its point is in the plan (a linked route's flight order), so the answer names it as given (4.36)
    auto at = [&p, &detail](std::uint32_t point, Reason why) {
        const std::uint32_t i = p.linked && point < p.given ? p.position[point] : point;
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return why;
    };
    if (branches.size() > PathStore::kRouteBranches) return at(branches[PathStore::kRouteBranches].point, Reason::InvalidWaypoint);
    const std::uint32_t given = std::min<std::uint32_t>(p.given, static_cast<std::uint32_t>(waypoints.size()));
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    for (const RouteBranch& b : branches) {
        const std::uint32_t k = b.point;
        // at a point it has, on to another one (or the route's end there)
        if (k >= given || isHold(b.next) || !(b.next == std::floor(b.next) && b.next >= -1.0 && b.next < static_cast<double>(given)) ||
            b.next == static_cast<double>(k))
            return at(k, Reason::InvalidWaypoint);
        // its conditions: each field finite where given, its codes whole; a range and a window the right way up, a count
        // with its comparison (and an endurance with one), a percent within 100
        RouteBranch c = b;
        double* f[RouteBranch::kFields];
        c.fields(f);
        for (const double* v : f)
            if (!isHold(*v) && !std::isfinite(*v)) return at(k, Reason::InvalidWaypoint);
        const bool ranged = !isHold(b.altitudeMinM) || !isHold(b.altitudeMaxM);
        const bool endurance = !isHold(b.fuelKg) || !isHold(b.enduranceS) || !isHold(b.enduranceEndS) || !isHold(b.percent);
        if (!code(b.altitudeReference, static_cast<double>(AltitudeReference::Count)) || (!isHold(b.altitudeReference) && !ranged) ||
            (!isHold(b.altitudeMinM) && !isHold(b.altitudeMaxM) && b.altitudeMinM > b.altitudeMaxM) ||
            (!isHold(b.timeBeginS) && !isHold(b.timeEndS) && b.timeEndS < b.timeBeginS) ||
            isHold(b.captures) != isHold(b.capturesComparison) || !code(b.capturesComparison, static_cast<double>(Comparison::Count)) ||
            (!isHold(b.captures) && !(b.captures >= 0.0 && b.captures == std::floor(b.captures))) ||
            (!isHold(b.operatorInput) && b.operatorInput != 0.0 && b.operatorInput != 1.0) || endurance != !isHold(b.enduranceComparison) ||
            !code(b.enduranceComparison, static_cast<double>(Comparison::Count)) || b.fuelKg < 0.0 || b.enduranceS < 0.0 || b.percent < 0.0 ||
            b.percent > 100.0 || !code(b.contingency, 4.0))
            return at(k, Reason::InvalidWaypoint);
        // the flight it goes on with, from its point along the links as the route's own is: one it can fly
        std::uint32_t order[route::Plan::kMax];
        std::int32_t loop = -1;
        std::int16_t bad = -1;
        const int n = route::flightOrder(waypoints.data(), given, p.paths, p.pathCount, k, repeat, order, loop, bad, b.next);
        if (n < 0 || (loop >= 0 && loop + 1 == n)) return at(k, Reason::InvalidWaypoint); // (a next that is none, round one point)
    }
    // not built yet (its row, partial): a contingency the platform does not reach yet - mission critical (a subsystem's
    // failure) and lost comms (the policy's link) are FA-16's
    for (const RouteBranch& b : branches)
        if (b.contingency == static_cast<double>(Contingency::MissionCritical) || b.contingency == static_cast<double>(Contingency::LostComms))
            return at(b.point, Reason::NotImplemented);
    p.branchCount = static_cast<std::uint32_t>(branches.size());
    std::copy_n(branches.data(), p.branchCount, p.branches);
    return Reason::None;
}

CommandResult CapabilityHost::commandBranch(ActivityId activity, std::uint32_t branch, bool commanded, Caller caller) noexcept {
    details_.clear();
    const int found = liveSlot(activity);
    Waiting* w = found < 0 ? waitingEntry(activity) : nullptr;
    if (found < 0 && !w) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(std::max(found, 0));
    const ActivityRecord& record = w ? w->record : records_[s];
    if (const Reason why = addresses(record, caller); why != Reason::None) return rejected(why, activity, activity);
    // a route's, flying (its branches in the path store) or waiting
    const Command* command = w ? (w->support ? nullptr : &w->command) : isCascade(s) ? &config_->slots[s].command : nullptr;
    if (!command || !std::holds_alternative<RouteCommand>(*command) || (!w && !config_->path)) return rejected(Reason::WrongCommandType, activity);
    const RouteBranch* branches = w ? w->branches.data() : config_->path->routeBranches;
    const std::uint32_t count = w ? static_cast<std::uint32_t>(w->branches.size()) : config_->path->routeBranchCount;
    if (branch >= count || branches[branch].operatorInput != 1.0) { // (one it has not, or one that takes no operator input)
        CommandResult r = rejected(Reason::InvalidParameter, activity);
        r.index = static_cast<std::int16_t>(std::min<std::uint32_t>(branch, 0x7FFF));
        return r;
    }
    std::uint32_t& bits = w ? w->commanded : config_->path->routeCommanded;
    bits = commanded ? bits | (1u << branch) : bits & ~(1u << branch);
    CommandResult r;
    r.status = CommandStatus::Accepted;
    r.activity = activity;
    r.commandId = record.commandId;
    return r;
}

void CapabilityHost::holdExtras(Waiting& w, const RouteExtras* extras) const {
    w.loiters.reserve(PathStore::kRouteLoiters), w.loiters.clear();
    w.states.reserve(PathStore::kRouteStates), w.states.clear(), w.passed.clear();
    w.paths.reserve(PathStore::kRoutePaths), w.paths.clear();
    w.branches.reserve(PathStore::kRouteBranches), w.branches.clear(), w.commanded = 0;
    w.terminators.reserve(PathStore::kRouteTerminators), w.terminators.clear();
    if (!extras) return;
    w.loiters.assign(extras->loiters.begin(), extras->loiters.end());
    w.states.assign(extras->states.begin(), extras->states.end());
    w.paths.assign(extras->paths.begin(), extras->paths.end());
    w.branches.assign(extras->branches.begin(), extras->branches.end());
    w.terminators.assign(extras->terminators.begin(), extras->terminators.end());
}

std::uint32_t ControlStack::ahead(std::size_t slot, std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept {
    ends = false;
    if (slot >= kSlotCount || !behaviors_[slot] || started_[slot] != config_->slots[slot].generation) return 0;
    return behaviors_[slot]->ahead(points, max, ends);
}

// --- flown ---------------------------------------------------------------------------------

bool RouteBehavior::orderFrom(const ControlContext& ctx, std::uint32_t start, double startNext, bool repeat) noexcept {
    route::Plan& p = *plan_;
    const PathStore& store = *ctx.path;
    const std::uint32_t count = std::min<std::uint32_t>(store.count, route::Plan::kMax);
    const std::uint32_t pathCount = std::min<std::uint32_t>(store.routePathCount, static_cast<std::uint32_t>(PathStore::kRoutePaths));
    std::uint32_t order[route::Plan::kMax];
    std::int32_t loop = -1;
    std::int16_t bad = -1;
    const int n = route::flightOrder(store.waypoints, count, store.routePaths, pathCount, start, repeat, order, loop, bad, startNext);
    if (n <= 0 || (loop >= 0 && loop + 1 == n)) return false;
    const auto flown = static_cast<std::uint32_t>(n);
    route::orderRest(order, flown, count);
    for (std::uint32_t j = 0; j < count; ++j) p.order[j] = order[j], p.position[order[j]] = j, p.points[j] = store.waypoints[order[j]];
    p.linked = true, p.given = count, p.pathCount = pathCount;
    std::copy_n(store.routePaths, pathCount, p.paths);
    p.count = flown, p.repeat = loop >= 0, p.loop = loop > 0 ? static_cast<std::uint32_t>(loop) : 0;
    return true;
}

bool RouteBehavior::holds(const ControlContext& ctx, const RouteBranch& b, std::uint32_t captures) const noexcept {
    const sim::VehicleState& s = ctx.sensed;
    if (!isHold(b.altitudeMinM) || !isHold(b.altitudeMaxM)) { // (its altitude in the range's reference: above mean sea level, left out)
        const auto reference = isHold(b.altitudeReference) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(b.altitudeReference));
        const double h = altitudeNow(reference, s, ctx.altimeter);
        if (!(isHold(b.altitudeMinM) || h >= b.altitudeMinM) || !(isHold(b.altitudeMaxM) || h <= b.altitudeMaxM)) return false;
    }
    const double now = ctx.world ? ctx.world->simTime() : s.simTime; // (its window's clock: the world's)
    if (!(isHold(b.timeBeginS) || now >= b.timeBeginS) || !(isHold(b.timeEndS) || now <= b.timeEndS)) return false;
    if (!isHold(b.captures) && !compared(static_cast<double>(captures), b.capturesComparison, b.captures)) return false;
    // what it has left and its contingency, as its navigation report says (4.14): without one (a stack on its own), neither
    // holds; an endurance where its flight model tells of no energy, none
    const bool endurance = !isHold(b.fuelKg) || !isHold(b.enduranceS) || !isHold(b.enduranceEndS) || !isHold(b.percent);
    if (!endurance && isHold(b.contingency)) return true;
    NavigationReport r;
    if (!ctx.world || !ctx.world->navigation(ctx.vehicleId, r)) return false;
    if (endurance) {
        const double how = b.enduranceComparison;
        if (r.energy == Energy::Unknown || (!isHold(b.fuelKg) && !compared(r.fuelKg, how, b.fuelKg)) ||
            (!isHold(b.enduranceS) && !compared(r.enduranceS, how, b.enduranceS)) ||
            (!isHold(b.enduranceEndS) && !compared(now + r.enduranceS, how, b.enduranceEndS)) || (!isHold(b.percent) && !compared(r.percent, how, b.percent)))
            return false;
    }
    return isHold(b.contingency) || static_cast<double>(r.contingency) == b.contingency;
}

bool RouteBehavior::branchAt(const ControlContext& ctx, const Performance& perf, bool fromPoint, double flownM) {
    route::Plan& p = *plan_;
    if (decided_ || p.branchCount == 0 || !ctx.path) return false;
    decided_ = true;
    const std::uint32_t at = p.named(target_);
    bool any = false;
    for (std::uint32_t k = 0; k < p.branchCount; ++k)
        if (p.branches[k].point == at) ++p.branchCaptures[k], any = true;
    if (!any) return false;
    (void)perf;
    for (std::uint32_t k = 0; k < p.branchCount; ++k) {
        const RouteBranch& b = p.branches[k];
        const bool commanded = (ctx.path->routeCommanded >> k & 1u) != 0;
        if (b.point != at || (b.operatorInput == 1.0 && !commanded) || !holds(ctx, b, p.branchCaptures[k])) continue;
        double next = b.next;
        if (next < 0.0 && !fromPoint && route::loiterPoint(p.points[target_])) {
            // the route's end there, as its loiter ends: it loiters on - or, a route that repeats, back to its first point
            if (!(!isHold(flown_.repeat) && std::floor(flown_.repeat) >= 1.0 && ctx.path->count > 1)) {
                ended_ = finished_ = true;
                return true;
            }
            next = 0.0;
        }
        finishedM_ += flownM;
        restart(ctx, flown_, next, fromPoint || next < 0.0);
        return true;
    }
    return false;
}

std::uint32_t RouteBehavior::ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept {
    const route::Plan& p = *plan_;
    ends = false;
    if (!planned_ || p.count == 0) return 0;
    std::uint32_t n = 0;
    for (std::uint32_t i = target_; n < max; i = p.next(i)) {
        points[n++] = p.named(i);
        if (!p.leaves(i)) {
            ends = true;
            break;
        }
    }
    return n;
}

} // namespace fsim::control
