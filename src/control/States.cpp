// A route's planned inertial states (docs/flight-autonomy.md, 4.34; ADR-29 FA-6d2; A-GRA's InertialState): checked and
// placed by the host, where they are on their legs, and flown by the route's behaviour - its segments' altitudes through
// theirs, and at each its time. Its own translation unit, last in the library: what is added before the runtime's code
// moves it (the A/B's lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Features.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

bool aboveGround(double reference) noexcept { return reference == static_cast<double>(AltitudeReference::AboveGround); }

} // namespace

// --- where they are ----------------------------------------------------------------------

namespace route {

void placeStates(Plan& p) noexcept {
    for (std::uint32_t j = 0; j < p.stateCount; ++j) {
        const RouteState& s = p.states[j];
        const std::uint32_t k = s.point;
        p.stateAlongM[j] = p.stateLapM[j] = -1.0;
        if (k < p.start || k >= p.count) continue; // (a segment its first lap does not fly)
        if (afterLoiter(p, k)) { // (the leg after a loiter point: from where the loiter is left - Schedule.cpp)
            placeAfterLoiter(p, j);
            continue;
        }
        const Fix f = onLeg(p.leg(k, true), s.latitudeRad, s.longitudeRad);
        const Turn* before = p.turnBefore(k, true);
        const Turn& turn = p.turn(k, true);
        const double piece = p.pieceM(k, true);
        p.stateAlongM[j] = f.alongM;
        p.stateLapM[j] = p.arrivalM(k, true) - 0.5 * turn.radiusM * std::abs(turn.angleRad) - piece +
                         std::clamp(f.alongM - (before ? before->leadM : 0.0), 0.0, piece);
    }
}

void limitClimbs(Plan& p, const sim::VehicleState& state, const TablesSection* tables, const Performance& f, bool hovers,
                 const Altimeter* altimeter) noexcept {
    p.climbCount = 0;
    bool timed = false; // (a time to arrive at: a point's window, a state's time)
    for (std::uint32_t i = p.start; i < p.count && !timed; ++i) timed = !isHold(p.points[i].arrivalBeginS) || !isHold(p.points[i].arrivalEndS);
    for (std::uint32_t j = 0; j < p.stateCount && !timed; ++j) timed = !isHold(p.states[j].timeS);
    if (!timed) return;
    auto msl = [altimeter](double h, double reference) { // (on its isobar: above sea level)
        return reference == static_cast<double>(AltitudeReference::Barometric) && altimeter ? barometricMslM(*altimeter, h) : h;
    };
    double segmentFromM = 0.0;
    for (std::uint32_t k = p.start; k < p.count; ++k) {
        const Waypoint& w = p.points[k];
        // (after a loiter point, from where its loiter is left: its span its own - 4.33)
        const double fromM = afterLoiter(p, k) ? p.exitM(k - 1) + (k - 1 < p.loitersLeftTo ? 0.0 : loiterShortM(p, k - 1)) : segmentFromM;
        const double toM = p.arrivalM(k, true);
        segmentFromM = toM;
        if (aboveGround(w.altitudeReference) || !isHold(w.climbRateMs) || !isHold(w.climbOptimization) || isHold(w.altitudeM)) continue;
        // from where it climbs from - the aircraft's altitude now for its first, else the point before's in its reference - through
        // its states' altitudes to its point's
        const AltitudeReference reference = static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference));
        const Waypoint& b = p.points[k > 0 ? k - 1 : 0];
        double xa = fromM, ha = k == p.start ? altitudeNow(reference, state, altimeter) : b.altitudeReference == w.altitudeReference ? b.altitudeM : kHold;
        auto piece = [&](double xb, double hb) {
            const double d = xb - xa;
            if (!isHold(ha) && d > 1.0 && hb > ha && p.climbCount < Plan::kClimbs) {
                const double lo = msl(ha, w.altitudeReference), hi = msl(hb, w.altitudeReference), gradient = (hi - lo) / d, mid = 0.5 * (lo + hi);
                double least = kHold, most = kHold;
                levelSpeedsMs(tables, f, hovers, mid, state.fuelKg, least, most);
                auto climbs = [&](double v) { return climbRateMs(tables, f, hovers, true, mid, v, state.fuelKg) >= gradient * v; };
                if (std::isfinite(least) && std::isfinite(most) && most > least && !climbs(most)) {
                    // down from its fastest level speed: the first it climbs it at, then between that and the one above
                    const double step = (most - least) / 32.0;
                    double ok = least;
                    for (int n = 1; n <= 32; ++n)
                        if (climbs(most - n * step)) {
                            ok = most - n * step;
                            break;
                        }
                    if (climbs(ok)) {
                        double no = std::min(ok + step, most);
                        for (int n = 0; n < 16; ++n) {
                            const double middle = 0.5 * (ok + no);
                            (climbs(middle) ? ok : no) = middle;
                        }
                    }
                    const std::uint32_t c = p.climbCount++;
                    p.climbFromM[c] = xa, p.climbToM[c] = xb, p.climbMostMs[c] = ok;
                }
            }
            xa = xb, ha = hb;
        };
        for (std::uint32_t j = 0; j < p.stateCount; ++j) {
            const RouteState& st = p.states[j];
            if (st.point == k && !isHold(st.altitudeM) && st.altitudeReference == w.altitudeReference && p.stateLapM[j] >= 0.0)
                piece(p.stateLapM[j], st.altitudeM);
        }
        piece(toM, w.altitudeM);
    }
}

double Plan::climbTimeS(double fromM, double toM, double speedMs, double alongMs) const noexcept {
    double seconds = 0.0, climbed = 0.0;
    for (std::uint32_t c = 0; c < climbCount; ++c) {
        const double a = std::max(fromM, climbFromM[c]), b = std::min(toM, climbToM[c]);
        if (!(b > a)) continue;
        seconds += (b - a) / std::max(std::min(speedMs, climbMostMs[c] + alongMs), 0.1);
        climbed += b - a;
    }
    return seconds + std::max(toM - fromM - climbed, 0.0) / std::max(speedMs, 0.1);
}

} // namespace route

// --- checked -------------------------------------------------------------------------------

Reason CapabilityHost::checkStates(route::Plan& p, Span<const RouteState> states, const sim::VehicleState& state, CommandResult& detail) const {
    p.stateCount = 0;
    if (states.empty()) return Reason::None;
    auto at = [&detail](std::uint32_t point, Reason why) {
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(point, 0x7FFF));
        return why;
    };
    if (states.size() > PathStore::kRouteStates) return at(states[PathStore::kRouteStates].point, Reason::InvalidWaypoint);
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    const double now = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    // what is not built yet (its row, partial): a state on a route with a point in a moving frame, whose legs move; a time
    // where the tables its speeds come from are missing, as an arrival window's is (4.33)
    bool moving = false;
    for (std::uint32_t k = 0; k < p.frameCount; ++k) moving = moving || p.frames[k].origin != FrameOrigin::Fixed;
    const SupportInfo* timed = support_ ? support_->find("fsim.guidance.route/required_time_of_arrival") : nullptr;
    double before = -std::numeric_limits<double>::infinity();
    std::uint32_t last = 0;
    for (std::uint32_t j = 0; j < states.size(); ++j) {
        RouteState s = states[j];
        const std::uint32_t k = p.linked && s.point < p.given ? p.position[s.point] : s.point; // (in a linked route's flight order: 4.36)
        s.point = k;
        // on a segment its first lap flies, in order along the route; its fields finite, its codes whole
        if (k < p.start || k >= p.count || (j > 0 && k < last)) return at(k, Reason::InvalidWaypoint);
        last = k;
        double* f[RouteState::kFields];
        s.fields(f);
        for (const double* v : f)
            if (!isHold(*v) && !std::isfinite(*v)) return at(k, Reason::InvalidWaypoint);
        if (!code(s.altitudeReference, static_cast<double>(AltitudeReference::Count)) || !code(s.frameRotation, static_cast<double>(FrameRotation::Count)) ||
            !code(s.frameOffsets, static_cast<double>(FrameOffsets::Count)) || (!isHold(s.uncertaintyM) && s.uncertaintyM < 0.0))
            return at(k, Reason::InvalidWaypoint);
        // its place: in a frame, where the frame is at its time (a vehicle's, where its velocity now carries it; no time, now)
        if (!isHold(s.frame)) {
            FrameSpec spec;
            FramePose pose;
            if (!(s.frame == std::floor(s.frame) && s.frame >= 1.0 && s.frame <= 9007199254740992.0) || !sessionView_ ||
                !sessionView_->frame(static_cast<FrameId>(s.frame), spec, pose))
                return at(k, Reason::InvalidWaypoint);
            if (spec.origin == FrameOrigin::Vehicle) {
                spec.origin = FrameOrigin::Moving;
                spec.latitudeRad = pose.latitudeRad, spec.longitudeRad = pose.longitudeRad, spec.altitudeMslM = pose.altitudeMslM;
                spec.yawRad = pose.yawRad, spec.pitchRad = pose.pitchRad, spec.rollRad = pose.rollRad;
                spec.northMs = pose.northMs, spec.eastMs = pose.eastMs, spec.downMs = pose.downMs;
                spec.timeS = now;
            }
            const GeoPoint there = framePoint(framePose(spec, isHold(s.timeS) ? now : s.timeS), s.frameOffset());
            s.latitudeRad = there.latitudeRad, s.longitudeRad = geo::wrapPi(there.longitudeRad);
            if (!isHold(s.frameZM)) s.altitudeM = there.altitudeMslM, s.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        }
        if (isHold(s.latitudeRad) || isHold(s.longitudeRad) || std::abs(s.latitudeRad) > 0.5 * 3.14159265358979323846) return at(k, Reason::InvalidWaypoint);
        // its altitude in its point's reference (the segment's profile); its time not past, after the one before
        const Waypoint& w = p.points[k];
        if (isHold(s.altitudeReference)) s.altitudeReference = w.altitudeReference;
        else if (s.altitudeReference != w.altitudeReference) return at(k, Reason::InvalidWaypoint);
        if (!isHold(s.timeS)) {
            if (s.timeS < now || !(s.timeS > before)) return at(k, Reason::InvalidWaypoint);
            before = s.timeS;
        }
        // one profile: an altitude beside a climb rate or optimisation is two
        if (!isHold(s.altitudeM) && (!isHold(w.climbRateMs) || !isHold(w.climbOptimization))) return at(k, Reason::InvalidWaypoint);
        if (moving) return at(k, Reason::NotImplemented);
        if (!isHold(s.timeS) && (!timed || timed->support == Support::NotImplemented)) return at(k, Reason::NotImplemented);
        p.states[j] = s;
    }
    p.stateCount = static_cast<std::uint32_t>(states.size());
    return Reason::None;
}

Reason CapabilityHost::limitStates(route::Plan& p, const sim::VehicleState& state, CheckLog& log) const noexcept {
    if (!p.stateCount) return Reason::None;
    if (p.loiterCount) { // (the legs on from its loiters, laid from where they are left: FA-6g3a)
        bool magnetic = false;
        for (std::uint32_t k = 0; k < p.loiterCount; ++k)
            magnetic = magnetic || p.loiters[k].shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth);
        route::measureLoiters(p, state, performance_, (adapter_->features() & kFeatureHover) != 0, &config_->altimeter, magnetic ? yearNow() : 2025.0);
    }
    route::placeStates(p);
    CommandResult& detail = log.result;
    // each on its leg as planned - after a loiter point, as laid from where the loiter is left - within its uncertainty (50 m
    // at least, or 1 % of the leg), behind none before it; on a loiter point's own leg, before where its loiter begins
    for (std::uint32_t j = 0; j < p.stateCount; ++j) {
        const RouteState& s = p.states[j];
        const route::Leg leg = route::stateLeg(p, s.point);
        const route::Fix fix = route::onLeg(leg, s.latitudeRad, s.longitudeRad);
        const double within = std::max({50.0, isHold(s.uncertaintyM) ? 0.0 : s.uncertaintyM, 0.01 * leg.lengthM});
        const bool behind = j > 0 && p.states[j - 1].point == s.point && fix.alongM < p.stateAlongM[j - 1];
        const double reach = route::loiterPoint(p.points[s.point]) ? route::loiterReachM(p, s.point, performance_, (adapter_->features() & kFeatureHover) != 0) : 0.0;
        if (std::abs(fix.crossTrackM) > within || fix.alongM < -within || fix.alongM > leg.lengthM - reach + within || behind) {
            detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(s.point, 0x7FFF));
            return Reason::InvalidWaypoint;
        }
    }
    // its profile: each piece between two altitudes - where it climbs from, its states', its point's - no steeper than the
    // aircraft climbs or descends, at its point's speed; between two times, in the time between them
    const Performance& f = performance_;
    auto msl = [&](double h, double reference) {
        return reference == static_cast<double>(AltitudeReference::Barometric) ? barometricMslM(config_->altimeter, h) : h; // (on its isobar)
    };
    for (std::uint32_t k = p.start; k < p.count; ++k) {
        const Waypoint& w = p.points[k];
        if (!p.stateAltitudes(k) || aboveGround(w.altitudeReference)) continue;
        const double v = route::plannedSpeed(w.speed, w.speedReference, w.altitudeM);
        double x0 = 0.0, t0 = kHold;
        double h0 = k == p.start ? altitudeNow(static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state, &config_->altimeter)
                    : p.points[k - 1].altitudeReference == w.altitudeReference ? p.points[k - 1].altitudeM
                                                                                 : kHold;
        auto piece = [&](double x1, double h1, double t1) {
            const double d = std::max(x1 - x0, 1.0);
            const double seconds = !isHold(t0) && !isHold(t1) ? t1 - t0 : v > 0.0 ? d / v : kHold;
            bool steep = false;
            if (!isHold(h0) && !isHold(seconds) && seconds > 0.0) {
                const double rate = std::abs(msl(h1, w.altitudeReference) - msl(h0, w.altitudeReference)) / seconds;
                const bool descends = h1 < h0;
                const double most = descends ? f.maxDescentMs : f.maxClimbMs;
                steep = rate > most;
                if (steep) log.find(Reason::PerformanceLimit, static_cast<std::int16_t>(std::min<std::uint32_t>(k, 0x7FFF)),
                                    descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate);
            }
            x0 = x1, h0 = h1, t0 = t1;
            return steep;
        };
        bool steep = false;
        for (std::uint32_t j = 0; j < p.stateCount && !steep; ++j) {
            const RouteState& s = p.states[j];
            if (s.point == k && !isHold(s.altitudeM)) steep = piece(p.stateAlongM[j], s.altitudeM, s.timeS);
        }
        if (!steep) piece(route::stateLeg(p, k).lengthM, w.altitudeM, kHold);
    }
    return Reason::None;
}

void CapabilityHost::keepStates(Waiting& w, const PathStore& store, double start) const {
    w.states.reserve(PathStore::kRouteStates), w.states.clear(), w.passed.clear();
    auto along = [&store](double point) { // (where a point comes in its flight: a linked route's order - 4.36 - else as given)
        for (std::uint32_t j = 0; j < store.count && store.routeLinked && point >= 0.0; ++j)
            if (static_cast<double>(store.routeOrder[j]) == point) return static_cast<double>(j);
        return point;
    };
    const double from = along(start);
    for (std::uint32_t j = 0; j < store.routeStateCount; ++j)
        (along(static_cast<double>(store.routeStates[j].point)) > from ? w.states : w.passed).push_back(store.routeStates[j]);
}

// --- flown ---------------------------------------------------------------------------------

bool RouteBehavior::takeStates(const ControlContext& ctx) noexcept {
    route::Plan& p = *plan_;
    p.stateCount = std::min<std::uint32_t>(ctx.path->routeStateCount, static_cast<std::uint32_t>(PathStore::kRouteStates));
    std::copy_n(ctx.path->routeStates, p.stateCount, p.states);
    for (std::uint32_t j = 0; j < p.stateCount; ++j) {
        RouteState& s = p.states[j];
        if (p.linked && s.point < p.given) s.point = p.position[s.point]; // (in a linked route's flight order: 4.36)
        if (s.point >= p.count || !std::isfinite(s.latitudeRad) || !std::isfinite(s.longitudeRad)) return false;
        if (isHold(s.altitudeReference)) s.altitudeReference = p.points[s.point].altitudeReference; // (a stack's own: its point's)
    }
    return true;
}

void RouteBehavior::nextArrival(double lapM) noexcept {
    const route::Plan& p = *plan_;
    const std::uint32_t k = segment_;
    const std::int32_t point = arrivalPoint_, state = arrivalState_;
    arrivalPoint_ = arrivalState_ = -1;
    stateAltitudes_ = false;
    if (segmentFirstLap_) {
        for (std::uint32_t i = k; i < p.count; ++i)
            if (!isHold(p.points[i].arrivalBeginS) || !isHold(p.points[i].arrivalEndS)) {
                arrivalPoint_ = static_cast<std::int32_t>(i);
                break;
            }
        // a timed state before it (on its segment or one before), not passed: the nearest
        double nearest = std::numeric_limits<double>::infinity();
        for (std::uint32_t j = 0; j < p.stateCount; ++j) {
            const RouteState& s = p.states[j];
            if (s.point == k && !isHold(s.altitudeM)) stateAltitudes_ = true;
            if (isHold(s.timeS) || s.point < k || (arrivalPoint_ >= 0 && s.point > static_cast<std::uint32_t>(arrivalPoint_))) continue;
            if (p.stateLapM[j] > lapM && p.stateLapM[j] < nearest) nearest = p.stateLapM[j], arrivalState_ = static_cast<std::int32_t>(j);
        }
        if (arrivalState_ >= 0) arrivalPoint_ = -1;
    }
    if (arrivalPoint_ != point || arrivalState_ != state) arrivalAimS_ = arrivalSpeedMs_ = arrivalS_ = arrivalDeltaS_ = kHold;
}

double RouteBehavior::paceThroughClimbs(double fromM, double toM, double seconds, double lo, double hi, double alongMs) const noexcept {
    const route::Plan& p = *plan_;
    if (!std::isfinite(hi)) return std::max(toM - fromM, 0.0) / seconds;
    const double slowest = std::isfinite(lo) ? std::max(lo, 0.5) : 0.5;
    if (p.climbTimeS(fromM, toM, hi, alongMs) > seconds) return hi;           // (it cannot: as fast as it flies)
    if (p.climbTimeS(fromM, toM, slowest, alongMs) <= seconds) return slowest; // (nor as slow)
    double fast = hi, slow = slowest;
    for (int n = 0; n < 40; ++n) {
        const double middle = 0.5 * (fast + slow);
        (p.climbTimeS(fromM, toM, middle, alongMs) > seconds ? slow : fast) = middle;
    }
    return fast;
}

double RouteBehavior::climbMostHere(double lapM, double alongMs) const noexcept {
    const route::Plan& p = *plan_;
    for (std::uint32_t c = 0; c < p.climbCount; ++c)
        if (lapM >= p.climbFromM[c] && lapM < p.climbToM[c]) return p.climbMostMs[c] + alongMs;
    return std::numeric_limits<double>::infinity();
}

double RouteBehavior::stateAltitude(double routeM, double& feedforward) const noexcept {
    const route::Plan& p = *plan_;
    const Waypoint& to = p.points[segment_];
    double x0 = segmentStartM_, h0 = segmentFrom_, x1 = segmentStartM_ + segmentM_, h1 = to.altitudeM;
    for (std::uint32_t j = 0; j < p.stateCount; ++j) { // (the nearest behind it, and ahead: in its point's reference)
        const RouteState& s = p.states[j];
        if (s.point != segment_ || isHold(s.altitudeM) || s.altitudeReference != to.altitudeReference) continue;
        const double x = lapStartM_ + p.stateLapM[j] - arrivalShiftM_; // (past a loiter, in the route's own measure: 4.33)
        if (x <= routeM && x >= x0) x0 = x, h0 = s.altitudeM;
        else if (x > routeM && x < x1) x1 = x, h1 = s.altitudeM;
    }
    if (!(x1 - x0 > 1.0)) return routeM >= x1 ? h1 : h0;
    const double fraction = std::clamp((routeM - x0) / (x1 - x0), 0.0, 1.0);
    if (fraction < 1.0) feedforward = (h1 - h0) / (x1 - x0) * groundSpeed_;
    return h0 + (h1 - h0) * fraction;
}

} // namespace fsim::control
