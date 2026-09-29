// What an activity flies, and where to (docs/flight-autonomy.md, 4.12):
// CapabilityHost's reports - an activity's setpoint read back, its end
// points, what the vehicle is commanded. Apart from the host's other code, so
// that growing either moves neither.
#include "control/CapabilityHost.h"

#include "control/Route.h"
#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

template <typename E>
bool is(double code, E value) noexcept {
    return code == static_cast<double>(value);
}

/// A curve's segments as it flies them (docs/flight-autonomy.md, 4.26): every one as A-GRA's schema gives it, and
/// as Bezier segments too where each is one's form.
void curveOf(Setpoint& out, const NurbsSegment* segments, std::size_t count) {
    out.nurbs.assign(segments, segments + count);
    if (std::all_of(segments, segments + count, [](const NurbsSegment& s) { return s.bezier(); }))
        for (std::size_t i = 0; i < count; ++i) out.segments.push_back(segments[i].asBezier());
}

} // namespace

bool CapabilityHost::setpoint(ActivityId activity, Setpoint& out) const {
    out.waypoints.clear(), out.segments.clear(), out.nurbs.clear(), out.loiters.clear(), out.states.clear(), out.paths.clear();
    out.branches.clear(), out.terminators.clear();
    out.shape = PatternShape{};
    out.curveShape = CurveShape{};
    if (const int found = liveSlot(activity); found >= 0) {
        const auto s = static_cast<std::size_t>(found);
        if (!isCascade(s)) {
            out.command = supportCommandOf(s);
            return true;
        }
        const Command& flown = config_->slots[s].command;
        out.command = flown;
        if (const PathStore* store = config_->path.get()) { // (one route or curve flies at a time: the store's)
            if (std::holds_alternative<RouteCommand>(flown) || std::holds_alternative<MustFlyCommand>(flown)) { // (a must fly's laid out: 4.42)
                out.waypoints.assign(store->waypoints, store->waypoints + store->count);
                out.loiters.assign(store->routeLoiters, store->routeLoiters + store->routeLoiterCount); // (4.31)
                out.states.assign(store->routeStates, store->routeStates + store->routeStateCount);     // (4.34)
                out.paths.assign(store->routePaths, store->routePaths + store->routePathCount);          // (4.36)
                out.branches.assign(store->routeBranches, store->routeBranches + store->routeBranchCount); // (4.37)
                out.terminators.assign(store->routeTerminators, store->routeTerminators + store->routeTerminatorCount); // (4.38)
            }
            if (std::holds_alternative<CurveCommand>(flown)) curveOf(out, store->segments, store->segmentCount), out.curveShape = store->curveShape;
            if (std::holds_alternative<PatternCommand>(flown)) out.shape = store->pattern;
        }
        return true;
    }
    const Waiting* w = waitingEntry(activity); // (as given: a waiting command is prepared afresh as it starts)
    if (!w) return false;
    if (w->support) out.command = w->supportCommand;
    else out.command = w->command;
    out.waypoints = w->waypoints, out.shape = w->shape, out.curveShape = w->curveShape, out.loiters = w->loiters, out.states = w->states;
    out.paths = w->paths, out.branches = w->branches, out.terminators = w->terminators;
    curveOf(out, w->segments.data(), w->segments.size());
    return true;
}

std::vector<EndPoint> CapabilityHost::endPoints(ActivityId activity, std::size_t max) const {
    std::vector<EndPoint> out;
    Setpoint s;
    const ActivityRecord* record = this->activity(activity);
    if (!max || !record || !record->live() || !setpoint(activity, s)) return out;
    const auto* c = std::get_if<Command>(&s.command);
    if (!c) return out;
    // flying, where its progress says; waiting (or disabled), from its start - a route kept out of its slot resumes there
    const ActivityProgress& progress = record->progress;
    const bool reported = record->waiting == ActivityWait::None && record->state != ActivityState::Disabled && progress.segments;
    const bool past = reported && progress.distanceToGoM == 0.0; // after a route's or a curve's end, flying its end behaviour
    static const RouteCommand kLaid{}; // (a must fly's route: great circles, once, on at its end - 4.42)
    const RouteCommand* route = std::holds_alternative<MustFlyCommand>(*c) ? &kLaid : std::get_if<RouteCommand>(c);
    if (route && !s.waypoints.empty()) {
        const auto n = static_cast<std::uint32_t>(std::min<std::size_t>(s.waypoints.size(), route::Plan::kMax));
        // its flight order (a linked route's, along its points' next: 4.36), from the point flown to
        const std::uint32_t first = static_cast<std::uint32_t>(std::max(orHold(route->start, 0.0), 0.0));
        std::uint32_t order[route::Plan::kMax];
        std::int32_t loop = is(route->repeat, 1) ? 0 : -1;
        std::uint32_t flown = n;
        const auto pathCount = static_cast<std::uint32_t>(s.paths.size());
        if (route::linked(s.waypoints.data(), n, pathCount)) {
            std::int16_t bad = -1;
            const int m = route::flightOrder(s.waypoints.data(), n, s.paths.data(), pathCount, std::min(first, n - 1), is(route->repeat, 1), order, loop, bad);
            if (m <= 0) return out;
            flown = static_cast<std::uint32_t>(m);
        } else {
            for (std::uint32_t i = 0; i < n; ++i) order[i] = i;
        }
        const bool repeats = loop >= 0;
        std::uint32_t j = 0; // (where in its order: the point flown to, or its start)
        const std::uint32_t flying = reported ? progress.segment : first;
        while (j < flown && order[j] != flying) ++j;
        if (j == flown) j = 0;
        // its points from here: in that order, round its laps - or, a route with branches flying, as its behaviour flies it
        // (one taken, on from there: 4.37)
        std::uint32_t ahead[route::Plan::kMax];
        std::uint32_t m = 0;
        bool ends = false;
        const int slot = reported && !s.branches.empty() ? liveSlot(activity) : -1;
        if (slot >= 0 && isCascade(static_cast<std::size_t>(slot)))
            m = runtime_->ahead(static_cast<std::size_t>(slot), ahead, static_cast<std::uint32_t>(std::min<std::size_t>(max, route::Plan::kMax)), ends);
        const bool asFlown = m > 0;
        for (; !asFlown && !ends && m < max && m < route::Plan::kMax; ++j) {
            if (j >= flown) {
                if (!repeats) break;
                j = static_cast<std::uint32_t>(loop);
            }
            ahead[m++] = order[j];
            ends = !repeats && j + 1 == flown;
        }
        if (m == 0) return out;
        // (a route whose last point is a loiter point ends in its loiter, whatever its end says: 4.31)
        const bool loiters = is(route->end, EndBehavior::Loiter) || route::loiterPoint(s.waypoints[ends ? ahead[m - 1] : order[flown - 1]]);
        if (past && ends && !loiters) return out; // it flies on along its last leg
        for (std::uint32_t k = 0; k < m && out.size() < max; ++k) {
            const std::uint32_t i = ahead[k];
            const Waypoint& w = s.waypoints[i];
            const bool last = ends && k + 1 == m;
            EndPoint e;
            e.kind = route::loiterPoint(w) ? EndPointKind::LoiterPoint
                     : !last             ? (route::noTurn(w) ? EndPointKind::Waypoint : EndPointKind::TurnPoint)
                     : loiters           ? EndPointKind::LoiterPoint
                                         : EndPointKind::Waypoint;
            e.latitudeRad = w.latitudeRad, e.longitudeRad = w.longitudeRad, e.altitudeM = w.altitudeM, e.altitudeReference = w.altitudeReference;
            FrameSpec spec; // (a point in a frame: where the frame puts it now - 4.29)
            FramePose now;
            if (!isHold(w.frame) && sessionView_ && sessionView_->frame(static_cast<FrameId>(w.frame), spec, now)) {
                const GeoPoint at = framePoint(now, w.frameOffset());
                e.latitudeRad = at.latitudeRad, e.longitudeRad = geo::wrapPi(at.longitudeRad);
                if (!isHold(w.frameZM)) e.altitudeM = at.altitudeMslM, e.altitudeReference = static_cast<double>(AltitudeReference::Msl);
            }
            if (!last && !route::noTurn(w)) e.turn = w.turn;
            e.id = w.id, e.index = static_cast<std::int32_t>(i);
            out.push_back(e);
        }
        return out;
    }
    if (const auto* curve = std::get_if<CurveCommand>(c); curve && !s.nurbs.empty()) {
        // each segment's end, from the one flown now: metres from its reference (one left out is where it will start: not yet known)
        const bool loiters = is(curve->end, EndBehavior::Loiter);
        if (isHold(curve->latitudeRad) || (past && !loiters)) return out;
        // its plane now (4.27): its reference where its frame is, its axes turned with it; its points as it reads them
        double lat0 = curve->latitudeRad, lon0 = curve->longitudeRad, alt0 = curve->altitudeM;
        if (!isHold(s.curveShape.frame)) {
            FrameSpec spec;
            FramePose now;
            if (sessionView_ && sessionView_->frame(static_cast<FrameId>(s.curveShape.frame), spec, now)) {
                const GeoPoint at = framePoint(now, s.curveShape.frameOffset());
                lat0 = at.latitudeRad, lon0 = at.longitudeRad;
                if (!isHold(s.curveShape.frameZM)) alt0 = at.altitudeMslM;
            }
        }
        const auto offsets = isHold(curve->pointOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(curve->pointOffsets));
        const auto z = isHold(curve->pointZ) ? CurveZ::Down : static_cast<CurveZ>(static_cast<int>(curve->pointZ));
        const double psi = curveTurn(*curve, s.curveShape);
        FramePose pose;
        const bool turned = curveAttitude(*curve, s.curveShape, pose); // (in three dimensions, by its frame's attitude now)
        const route::Attitude attitude(pose);
        for (std::size_t i = reported ? progress.segment : 0; i < s.nurbs.size() && out.size() < max; ++i) {
            const NurbsSegment& b = s.nurbs[i];
            const std::uint32_t last = b.points - 1; // (clamped: it ends at its last point)
            EndPoint e;
            e.kind = loiters && i + 1 == s.nurbs.size() ? EndPointKind::LoiterPoint : EndPointKind::Waypoint;
            double north = b.north[last], east = b.east[last];
            e.altitudeM = z == CurveZ::Down ? alt0 - b.down[last] : z == CurveZ::AltitudeOffset ? alt0 + b.down[last] : b.down[last];
            if (turned) {
                double down;
                attitude.turn(north, east, z == CurveZ::Down ? b.down[last] : z == CurveZ::AltitudeOffset ? -b.down[last] : 0.0, north, east, down);
                if (z != CurveZ::AbsoluteAltitude) e.altitudeM = alt0 - down; // (an absolute altitude's kept)
            }
            if (offsets == FrameOffsets::Cartesian && psi == 0.0) geo::offsetLatLon(lat0, lon0, north, east, e.latitudeRad, e.longitudeRad);
            else route::planeToEarth(lat0, lon0, offsets, psi, north, east, e.latitudeRad, e.longitudeRad);
            e.altitudeReference = isHold(curve->altitudeReference) ? static_cast<double>(AltitudeReference::Msl) : curve->altitudeReference;
            e.index = static_cast<std::int32_t>(i);
            out.push_back(e);
        }
        return out;
    }
    EndPoint e;
    e.index = 0;
    if (const auto* pattern = std::get_if<PatternCommand>(c); pattern && !isHold(pattern->latitudeRad)) { // its centre or fix
        e.kind = EndPointKind::LoiterPoint;
        e.latitudeRad = pattern->latitudeRad, e.longitudeRad = pattern->longitudeRad;
        e.altitudeM = pattern->altitudeM, e.altitudeReference = pattern->altitudeReference;
        out.push_back(e);
    } else if (const auto* point = std::get_if<PositionCommand>(c)) {
        e.latitudeRad = point->latitudeRad, e.longitudeRad = point->longitudeRad;
        e.altitudeM = point->altitudeMslM, e.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        out.push_back(e);
    }
    return out;
}

VehicleCommandState CapabilityHost::commandState(const sim::VehicleState& state) const noexcept {
    VehicleCommandState c;
    static_cast<CommandedState&>(c) = runtime_->commanded();
    // A wing's acceleration, in its body's axes: the longitudinal along its flight path; across the path, its load
    // factor's lift (the lift's, JSBSim's Nlf: normal to the path, in the plane of symmetry) and gravity's pull.
    const Command* level = runtime_->derived(Level::Acceleration);
    const auto* a = level ? std::get_if<AccelerationCommand>(level) : nullptr;
    if (a && !(adapter_->features() & kFeatureHover) && !isHold(a->longitudinalMs2) && !isHold(a->loadFactorG)) {
        const double sr = std::sin(state.eulerRad[0]), cr = std::cos(state.eulerRad[0]);
        const double sp = std::sin(state.eulerRad[1]), cp = std::cos(state.eulerRad[1]);
        const double sy = std::sin(state.eulerRad[2]), cy = std::cos(state.eulerRad[2]);
        const double sa = std::sin(state.alphaRad), ca = std::cos(state.alphaRad), sb = std::sin(state.betaRad), cb = std::cos(state.betaRad);
        const double path[3] = {ca * cb, sb, sa * cb}, lift[3] = {sa, 0.0, -ca}; // (the path's, and the lift's, directions)
        const double gravity[3] = {-kG * sp, kG * sr * cp, kG * cr * cp};
        const double along = gravity[0] * path[0] + gravity[1] * path[1] + gravity[2] * path[2];
        double body[3];
        for (int i = 0; i < 3; ++i) body[i] = a->longitudinalMs2 * path[i] + gravity[i] - along * path[i] + a->loadFactorG * kG * lift[i];
        const double x = body[0], y = body[1], z = body[2]; // to north, east and down
        c.northAccelerationMs2 = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z;
        c.eastAccelerationMs2 = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z;
        c.downAccelerationMs2 = -sp * x + sr * cp * y + cr * cp * z;
    }
    // the altitude as a live mode commanded it, in its reference
    for (std::size_t s = 0; s < kSlotCount; ++s) {
        if (!slots_[s].live) continue;
        const Command& flown = config_->slots[s].command;
        if (const auto* h = std::get_if<HsaCommand>(&flown)) {
            c.altitudeM = h->altitudeM, c.altitudeReference = h->altitudeReference;
            return c;
        }
        if (const auto* p = std::get_if<PatternCommand>(&flown)) {
            c.altitudeM = p->altitudeM, c.altitudeReference = p->altitudeReference;
            return c;
        }
        const PathStore* store = config_->path.get();
        if ((std::holds_alternative<RouteCommand>(flown) || std::holds_alternative<MustFlyCommand>(flown)) && store && store->count) {
            const Waypoint& w = store->waypoints[std::min(records_[s].progress.segment, store->count - 1)];
            c.altitudeM = w.altitudeM, c.altitudeReference = w.altitudeReference;
            return c;
        }
        if (std::holds_alternative<CurveCommand>(flown) && !std::isnan(records_[s].progress.altitudeMslM)) {
            c.altitudeM = records_[s].progress.altitudeMslM, c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
            return c;
        }
    }
    if (!std::isnan(c.altitudeMslM)) c.altitudeM = c.altitudeMslM, c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    return c;
}

} // namespace fsim::control
