// A route's paths and links (docs/flight-autonomy.md, 4.36; ADR-29 FA-6e1; A-GRA's MA_RoutePathType, NextPathSegment): its
// flight order from its start along each point's next, the host's plan and the behaviour's in that order, each point
// named as given. Its own translation unit, last in the library: what is added before the runtime's code moves it (the A/B's
// lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Route.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace route {

int pathFault(const RoutePath* paths, std::uint32_t pathCount, std::uint32_t count) noexcept {
    if (pathCount > PathStore::kRoutePaths) return 0;
    auto at = [](std::uint32_t i) { return static_cast<int>(std::min<std::uint32_t>(i, 0x7FFF)); };
    std::uint32_t next = 0; // (they tile the waypoints in order)
    for (std::uint32_t k = 0; k < pathCount; ++k) {
        const RoutePath& r = paths[k];
        if (r.count == 0 || r.first != next || r.count > count - std::min(r.first, count)) return at(std::min(r.first, next));
        if (!isHold(r.type) && !(r.type == std::floor(r.type) && r.type >= 0.0 && r.type < static_cast<double>(PathType::Count))) return at(r.first);
        for (std::uint32_t j = 0; j < k; ++j)
            if (paths[j].id == r.id) return at(r.first);
        next = r.first + r.count;
    }
    return pathCount && next != count ? at(next) : -1; // (a point no path holds)
}

bool linked(const Waypoint* points, std::uint32_t count, std::uint32_t pathCount) noexcept {
    if (pathCount) return true;
    for (std::uint32_t i = 0; i < count; ++i)
        if (!isHold(points[i].next)) return true;
    return false;
}

int flightOrder(const Waypoint* points, std::uint32_t count, const RoutePath* paths, std::uint32_t pathCount, std::uint32_t start, bool repeat,
                std::uint32_t* order, std::int32_t& loop, std::int16_t& bad, double startNext) noexcept {
    loop = -1;
    auto at = [&bad](std::uint32_t i) {
        bad = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return -1;
    };
    // every point's next a point's index, or -1
    auto none = [count](double next) { return !isHold(next) && !(next == std::floor(next) && next >= -1.0 && next < static_cast<double>(count)); };
    for (std::uint32_t i = 0; i < count; ++i)
        if (none(points[i].next)) return at(i);
    if (start >= count || none(startNext)) return at(start);
    std::int32_t seen[PathStore::kWaypoints];
    std::fill_n(seen, count, -1);
    std::uint32_t n = 0;
    for (std::uint32_t i = start;;) {
        if (seen[i] >= 0) { // flown before: a lap on from there
            loop = seen[i];
            break;
        }
        seen[i] = static_cast<std::int32_t>(n);
        order[n++] = i;
        const double next = n == 1 && !isHold(startNext) ? startNext : points[i].next;
        if (!isHold(next) && next >= 0.0) {
            i = static_cast<std::uint32_t>(next);
            continue;
        }
        std::uint32_t end = count; // (the next in its path; its path's last, the route's end)
        for (std::uint32_t k = 0; k < pathCount; ++k)
            if (i >= paths[k].first && i - paths[k].first < paths[k].count) end = paths[k].first + paths[k].count;
        if (isHold(next) && i + 1 < end) {
            ++i;
            continue;
        }
        if (!repeat) break; // (the route's end, there)
        i = 0;              // (a route that repeats: back to its first point, as an unlinked one)
    }
    return static_cast<int>(n);
}

void orderRest(std::uint32_t* order, std::uint32_t flown, std::uint32_t count) noexcept {
    bool held[Plan::kMax] = {};
    for (std::uint32_t j = 0; j < flown; ++j) held[order[j]] = true;
    std::uint32_t m = flown;
    for (std::uint32_t i = 0; i < count; ++i)
        if (!held[i]) order[m++] = i;
}

bool linkStore(PathStore& store, const RouteCommand& c) noexcept {
    const std::uint32_t count = std::min<std::uint32_t>(store.count, Plan::kMax);
    const std::uint32_t pathCount = std::min<std::uint32_t>(store.routePathCount, static_cast<std::uint32_t>(PathStore::kRoutePaths));
    if (!linked(store.waypoints, count, pathCount)) {
        store.routeLinked = false;
        return true;
    }
    const double start = isHold(c.start) ? 0.0 : c.start;
    if (pathFault(store.routePaths, pathCount, count) >= 0 || !(start >= 0.0 && start < static_cast<double>(count) && start == std::floor(start))) return false;
    std::uint32_t order[Plan::kMax];
    std::int32_t loop = -1;
    std::int16_t bad = -1;
    const int n = flightOrder(store.waypoints, count, store.routePaths, pathCount, static_cast<std::uint32_t>(start), c.repeat == 1.0, order, loop, bad);
    if (n <= 0 || (loop >= 0 && loop + 1 == n)) return false;
    orderRest(order, static_cast<std::uint32_t>(n), count);
    for (std::uint32_t j = 0; j < count; ++j) store.routeOrder[j] = static_cast<std::uint16_t>(order[j]);
    store.routeLinked = true, store.routeFlown = static_cast<std::uint32_t>(n), store.routeRepeats = loop >= 0;
    store.routeLoop = loop > 0 ? static_cast<std::uint32_t>(loop) : 0;
    return true;
}

bool endsPath(const Plan& p, std::uint32_t i) noexcept {
    const std::uint32_t g = p.named(i);
    if (p.points[i].next == -1.0) return true;      // (the route's end there)
    if (p.pathCount == 0) return g + 1 == p.given; // (none given: one path, its points as given)
    for (std::uint32_t k = 0; k < p.pathCount; ++k)
        if (g + 1 == p.paths[k].first + p.paths[k].count) return true;
    return false;
}

} // namespace route

Reason CapabilityHost::linkRoute(route::Plan& p, const RouteCommand& c, Span<const RoutePath> paths, std::uint32_t count,
                                 CommandResult& detail) const noexcept {
    p.linked = false, p.given = count, p.pathCount = 0, p.loop = 0;
    const auto pathCount = static_cast<std::uint32_t>(std::min<std::size_t>(paths.size(), 0xFFFF));
    if (count > route::Plan::kMax || !route::linked(p.points, count, pathCount)) return Reason::None; // (too many: complete() refuses them)
    auto at = [&detail](int i) {
        detail.index = static_cast<std::int16_t>(i);
        return Reason::InvalidWaypoint;
    };
    if (const int fault = route::pathFault(paths.data(), pathCount, count); fault >= 0) return at(fault);
    p.pathCount = pathCount;
    std::copy_n(paths.data(), pathCount, p.paths);
    std::uint32_t order[route::Plan::kMax];
    std::int32_t loop = -1;
    std::int16_t bad = -1;
    const int n = route::flightOrder(p.points, count, p.paths, pathCount, static_cast<std::uint32_t>(c.start), c.repeat == 1.0, order, loop, bad);
    if (n < 0) return at(bad);
    const auto flown = static_cast<std::uint32_t>(n);
    if (loop >= 0 && static_cast<std::uint32_t>(loop) + 1 == flown) return at(static_cast<int>(order[flown - 1])); // (round one point: a pattern)
    // its points in flight order, those it does not fly after them in their order as given
    route::orderRest(order, flown, count);
    bool done[route::Plan::kMax] = {};
    for (std::uint32_t j = 0; j < count; ++j) { // (in place: each cycle of the permutation, one point aside)
        if (done[j]) continue;
        const Waypoint aside = p.points[j];
        std::uint32_t k = j;
        while (order[k] != j) {
            p.points[k] = p.points[order[k]];
            done[k] = true;
            k = order[k];
        }
        p.points[k] = aside;
        done[k] = true;
    }
    for (std::uint32_t j = 0; j < count; ++j) p.order[j] = order[j], p.position[order[j]] = j;
    p.linked = true, p.count = flown, p.start = 0, p.repeat = loop >= 0, p.loop = loop > 0 ? static_cast<std::uint32_t>(loop) : 0;
    return Reason::None;
}

void CapabilityHost::resetRoute(const RouteCommand& c) noexcept {
    if (!config_->path || !routePlan_) return;
    PathStore& store = *config_->path;
    std::vector<RouteState>& passed = routePlan_->passed;
    store.routeCommanded = 0; // (flown afresh: what the operator commanded goes - 4.37)
    bool changed = false;
    if (!passed.empty()) { // (the states it flew past, ahead of those it kept: 4.34)
        const auto n = static_cast<std::uint32_t>(std::min<std::size_t>(passed.size(), PathStore::kRouteStates - store.routeStateCount));
        std::copy_backward(store.routeStates, store.routeStates + store.routeStateCount, store.routeStates + store.routeStateCount + n);
        std::copy_n(passed.data(), n, store.routeStates);
        store.routeStateCount += n;
        passed.clear();
        changed = true;
    }
    if (store.routeLinked && route::linkStore(store, c)) changed = true; // (its flight order from its first start: a resumed one's began elsewhere)
    if (changed) ++store.revision;
}

void CapabilityHost::nameAsGiven(const route::Plan& p, CheckLog& log, int findingsFrom, int adjustmentsFrom, bool indexed) const noexcept {
    if (!p.linked) return;
    auto name = [&p](std::int16_t& index) {
        if (index >= 0 && static_cast<std::uint32_t>(index) < p.given) index = static_cast<std::int16_t>(std::min<std::uint32_t>(p.order[index], 0x7FFF));
    };
    if (!indexed) name(log.result.index);
    if (!log.details) return;
    const int findings = std::min<int>(log.details->findingCount, static_cast<int>(CommandDetails::kMax));
    const int adjustments = std::min<int>(log.details->adjustmentCount, static_cast<int>(CommandDetails::kMax));
    for (int f = findingsFrom; f < findings; ++f) name(log.details->findings[static_cast<std::size_t>(f)].index);
    for (int a = adjustmentsFrom; a < adjustments; ++a) name(log.details->adjustments[static_cast<std::size_t>(a)].index);
}

void CapabilityHost::givenRoute(std::vector<Waypoint>& points, std::vector<RouteLoiter>& loiters, std::vector<RouteState>& states,
                                std::vector<RoutePath>& paths, std::vector<RouteBranch>& branches) const {
    const route::Plan& p = *routePlan_;
    const std::uint32_t count = p.linked ? p.given : p.count;
    points.clear(), loiters.clear(), states.clear(), paths.clear();
    for (std::uint32_t i = 0; i < count; ++i) points.push_back(p.points[p.linked ? p.position[i] : i]);
    for (std::uint32_t k = 0; k < p.loiterCount; ++k) {
        RouteLoiter l = route::unplaced(p.loiters[k]); // (as held: its loiters complete, their place their points' - 4.31)
        l.point = p.named(l.point);
        loiters.push_back(l);
    }
    for (std::uint32_t j = 0; j < p.stateCount; ++j) {
        RouteState s = p.states[j]; // (as placed: 4.34)
        s.point = p.named(s.point);
        states.push_back(s);
    }
    paths.assign(p.paths, p.paths + p.pathCount);
    branches.assign(p.branches, p.branches + p.branchCount);
}

bool RouteBehavior::takeOrder(const ControlContext& ctx) noexcept {
    route::Plan& p = *plan_;
    const PathStore& store = *ctx.path;
    const std::uint32_t count = std::min<std::uint32_t>(store.count, route::Plan::kMax);
    p.linked = store.routeLinked, p.given = count, p.loop = 0, p.pathCount = 0;
    if (!p.linked) return false;
    for (std::uint32_t j = 0; j < count; ++j) {
        const std::uint32_t i = std::min<std::uint32_t>(store.routeOrder[j], count - 1);
        p.order[j] = i, p.position[i] = j;
        p.points[j] = store.waypoints[i];
    }
    p.pathCount = std::min<std::uint32_t>(store.routePathCount, static_cast<std::uint32_t>(PathStore::kRoutePaths));
    std::copy_n(store.routePaths, p.pathCount, p.paths);
    p.count = std::min(store.routeFlown, count), p.repeat = store.routeRepeats, p.loop = store.routeLoop < p.count ? store.routeLoop : 0;
    return true;
}

} // namespace fsim::control
