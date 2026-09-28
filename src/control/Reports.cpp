// What an activity flies, and where to (docs/flight-autonomy.md, 4.12):
// CapabilityHost's reports - an activity's setpoint read back, its end
// points, what the vehicle is commanded. Apart from the host's other code, so
// that growing either moves neither.
#include "control/CapabilityHost.h"

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

} // namespace

bool CapabilityHost::setpoint(ActivityId activity, Setpoint& out) const {
    out.waypoints.clear(), out.segments.clear();
    out.shape = PatternShape{};
    if (const int found = liveSlot(activity); found >= 0) {
        const auto s = static_cast<std::size_t>(found);
        if (!isCascade(s)) {
            out.command = supportCommandOf(s);
            return true;
        }
        const Command& flown = config_->slots[s].command;
        out.command = flown;
        if (const PathStore* store = config_->path.get()) { // (one route or curve flies at a time: the store's)
            if (std::holds_alternative<RouteCommand>(flown)) out.waypoints.assign(store->waypoints, store->waypoints + store->count);
            if (std::holds_alternative<CurveCommand>(flown)) out.segments.assign(store->segments, store->segments + store->segmentCount);
            if (std::holds_alternative<PatternCommand>(flown)) out.shape = store->pattern;
        }
        return true;
    }
    const Waiting* w = waitingEntry(activity); // (as given: a waiting command is prepared afresh as it starts)
    if (!w) return false;
    if (w->support) out.command = w->supportCommand;
    else out.command = w->command;
    out.waypoints = w->waypoints, out.segments = w->segments, out.shape = w->shape;
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
    if (const auto* route = std::get_if<RouteCommand>(c); route && !s.waypoints.empty()) {
        const auto n = static_cast<std::uint32_t>(s.waypoints.size());
        const bool repeats = is(route->repeat, 1), loiters = is(route->end, EndBehavior::Loiter);
        std::uint32_t i = reported ? progress.segment : static_cast<std::uint32_t>(std::max(orHold(route->start, 0.0), 0.0));
        if (past && !repeats && !loiters) return out; // it flies on along its last leg
        for (; out.size() < max; ++i) {
            if (i >= n) {
                if (!repeats) break;
                i = 0;
            }
            const Waypoint& w = s.waypoints[i];
            const bool last = !repeats && i + 1 == n;
            EndPoint e;
            e.kind = !last ? EndPointKind::TurnPoint : loiters ? EndPointKind::LoiterPoint : EndPointKind::Waypoint;
            e.latitudeRad = w.latitudeRad, e.longitudeRad = w.longitudeRad, e.altitudeM = w.altitudeM, e.altitudeReference = w.altitudeReference;
            if (!last) e.turn = w.turn;
            e.id = w.id, e.index = static_cast<std::int32_t>(i);
            out.push_back(e);
        }
        return out;
    }
    if (const auto* curve = std::get_if<CurveCommand>(c); curve && !s.segments.empty()) {
        // each segment's end, from the one flown now: metres from its reference (one left out is where it will start: not yet known)
        const bool loiters = is(curve->end, EndBehavior::Loiter);
        if (isHold(curve->latitudeRad) || (past && !loiters)) return out;
        for (std::size_t i = reported ? progress.segment : 0; i < s.segments.size() && out.size() < max; ++i) {
            const BezierSegment& b = s.segments[i];
            EndPoint e;
            e.kind = loiters && i + 1 == s.segments.size() ? EndPointKind::LoiterPoint : EndPointKind::Waypoint;
            geo::offsetLatLon(curve->latitudeRad, curve->longitudeRad, b.north[5], b.east[5], e.latitudeRad, e.longitudeRad);
            e.altitudeM = curve->altitudeM - b.down[5];
            e.altitudeReference = static_cast<double>(AltitudeReference::Msl);
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
        if (std::holds_alternative<RouteCommand>(flown) && store && store->count) {
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
