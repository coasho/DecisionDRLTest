// A route's times of arrival through its loiters (docs/flight-autonomy.md, 4.33; ADR-29 FA-6g2): each loiter's own time -
// the first of its ends, then on round to where it is left - counted, the legs' speed scheduled round it; a window at a
// loiter point met where its loiter begins. Its own translation unit, last in the library: what is added before the
// runtime's code moves it (the A/B's lesson).
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace route {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

} // namespace

double Plan::joinM(std::uint32_t k, double reachM) const noexcept {
    double m = 0.0;
    for (std::uint32_t i = start; i < k; ++i) {
        const Turn& t = turn(i, true);
        m += pieceM(i, true) + t.radiusM * std::abs(t.angleRad);
    }
    const Turn* before = turnBefore(k, true);
    return m + std::max(0.0, leg(k, true).lengthM - (before ? before->leadM : 0.0) - reachM);
}

double Plan::exitM(std::uint32_t k) const noexcept {
    double m = 0.0;
    for (std::uint32_t i = start; i <= k; ++i) {
        const Turn& t = turn(i, true);
        m += pieceM(i, true) + t.radiusM * std::abs(t.angleRad);
    }
    return m - turn(k, true).leadM;
}

bool Plan::timed() const noexcept {
    for (std::uint32_t i = 0; i < count; ++i)
        if (!isHold(points[i].arrivalBeginS) || !isHold(points[i].arrivalEndS)) return true;
    return false;
}

double loiterReachM(const Plan& p, std::uint32_t k, const Performance& performance, bool hovers) noexcept {
    const RouteLoiter* l = p.loiterAt(k);
    if (!l) return 0.0;
    const Waypoint& w = p.points[k];
    if (hovers && l->pattern.pattern == static_cast<double>(PatternKind::Hover))
        return std::max(stoppingDistanceM(performance, plannedSpeed(w.speed, w.speedReference, 0.0)), 1.0);
    return loiterJoinM(l->pattern, l->shape);
}

double loiterTimeS(const LoiterMeasure& m, bool hover, double orbits, double durationS, double untilS, bool begun) noexcept {
    if (hover) { // over its point from its arrival: its duration; its end time, where sooner
        double t = isHold(durationS) ? kNaN : m.approachS + durationS;
        if (!isHold(untilS)) t = std::fmin(t, std::max(untilS, 0.0));
        return t;
    }
    if (!(m.speedMs > 0.5)) return kNaN;
    // the first of its ends due, as a distance from where it begins: its laps flown, its duration, its end time
    double due = kNaN;
    bool timed = false;
    if (!isHold(orbits)) due = m.entryM + orbits * m.lapM;
    if (!isHold(durationS)) due = std::fmin(due, m.speedMs * durationS);
    if (!isHold(untilS)) {
        const double until = m.speedMs * std::max(untilS, 0.0);
        if (!(until >= due)) due = until, timed = true;
    }
    if (std::isnan(due)) return kNaN;
    if (m.exitM < 0.0 || !(m.lapM > 0.0)) return due / m.speedMs; // (no exit point: left where it is due)
    if (timed && untilS > 0.0 && !begun) return untilS; // (where it will be round its lap at its end time: known once it begins)
    // then on round to its exit point: first passed after its way in (a lap on, where the lap begins there), then each lap
    const double first = m.entryM + (m.exitM > 0.0 ? m.exitM : m.lapM);
    const double laps = std::max(0.0, std::ceil((due - first) / m.lapM - 1e-9));
    return (first + laps * m.lapM) / m.speedMs;
}

void measureLoiters(Plan& p, const sim::VehicleState& state, const Performance& performance, bool hovers, const Altimeter* altimeter,
                    double magneticYear) noexcept {
    for (std::uint32_t n = 0; n < p.loiterCount; ++n) {
        const RouteLoiter& l = p.loiters[n];
        LoiterMeasure& m = p.loiterMeasures[n];
        m = LoiterMeasure{};
        if (l.point >= p.count) continue; // (on a point it does not fly: 4.36)
        const PatternCommand& c = l.pattern;
        const Waypoint& w = p.points[l.point];
        const double h = w.altitudeReference == static_cast<double>(AltitudeReference::AboveGround)
                             ? state.altitudeMslM
                             : altitudeMslOf(w.altitudeM, static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state, altimeter);
        if (c.pattern == static_cast<double>(PatternKind::Hover)) { // (its way to its point, from where it begins to stop)
            const double v = plannedSpeed(w.speed, w.speedReference, h);
            m.approachS = v > 0.1 ? 2.0 * loiterReachM(p, l.point, performance, hovers) / v : 0.0;
            continue;
        }
        // planned where it is joined, on the course the leg into it arrives on: its way in, its lap, where it is left
        m.speedMs = plannedSpeed(c.speed, c.speedReference, h);
        const Leg& in = p.leg(l.point, true);
        double lat = w.latitudeRad, lon = w.longitudeRad, course = in.courseInRad;
        if (const double join = loiterJoinM(c, l.shape); join > 0.0 && in.lengthM > join)
            geo::destination(w.latitudeRad, w.longitudeRad, geo::wrapPi(in.courseInRad + 3.14159265358979323846), join, lat, lon);
        else if (join > 0.0) // (a leg shorter than that: joined where it begins - 4.31)
            lat = in.latA, lon = in.lonA, course = in.courseOutRad;
        PatternShape shape = l.shape;
        loiterEntry(c, lat, lon, shape);
        if (p.leaves(l.point)) loiterExit(c, p.points[p.next(l.point)].latitudeRad, p.points[p.next(l.point)].longitudeRad, shape);
        Pattern lap;
        planPattern(lap, c, lat, lon, shape, magneticYear, course);
        m.entryM = lap.entryM(), m.lapM = lap.lapM(), m.exitM = lap.exit >= 0 ? lap.toExitM() : -1.0;
        if (p.leaves(l.point) && !isHold(shape.exitLatitudeRad)) { // (the leg on, laid from where it is left)
            const Waypoint& next = p.points[p.next(l.point)];
            m.shortM = makeLeg(w.latitudeRad, w.longitudeRad, next.latitudeRad, next.longitudeRad, p.rhumb).lengthM -
                       makeLeg(shape.exitLatitudeRad, shape.exitLongitudeRad, next.latitudeRad, next.longitudeRad, p.rhumb).lengthM;
        }
    }
}

double loiterS(const Plan& p, std::uint32_t k, double beginS) noexcept {
    for (std::uint32_t n = 0; n < p.loiterCount; ++n) {
        const RouteLoiter& l = p.loiters[n];
        if (l.point != k) continue;
        return loiterTimeS(p.loiterMeasures[n], l.pattern.pattern == static_cast<double>(PatternKind::Hover), l.shape.orbits, l.pattern.durationS,
                           isHold(l.endTimeS) ? kNaN : l.endTimeS - beginS, false);
    }
    return 0.0;
}

double loiterShortM(const Plan& p, std::uint32_t k) noexcept {
    for (std::uint32_t n = 0; n < p.loiterCount; ++n)
        if (p.loiters[n].point == k) return p.loiterMeasures[n].shortM;
    return 0.0;
}

} // namespace route

namespace {

/// The loiters on the way from point `from` to point i's window (4.33): where each begins and where the leg on from it
/// begins, in the first lap's measure, and which it is; where it arrives - at a loiter point, where its loiter begins.
struct Ahead {
    double joinM[PathStore::kRouteLoiters] = {}, exitM[PathStore::kRouteLoiters] = {};
    std::uint32_t point[PathStore::kRouteLoiters] = {};
    std::uint32_t count = 0;
    double toM = 0.0;

    Ahead(const route::Plan& p, std::uint32_t from, std::uint32_t i, const Performance& performance, bool hovers) noexcept {
        toM = p.arrivalM(i, true);
        for (std::uint32_t k = from; k <= i; ++k) {
            if (!route::loiterPoint(p.points[k]) || !p.loiterAt(k)) continue;
            const double join = p.joinM(k, route::loiterReachM(p, k, performance, hovers));
            if (k == i) {
                toM = join;
                break;
            }
            if (count < PathStore::kRouteLoiters) joinM[count] = join, exitM[count] = p.exitM(k) + route::loiterShortM(p, k), point[count++] = k;
        }
    }

    /// From `fromM` at ground speed `v`: its legs - to each loiter's join, then from where the leg on from it begins (laid
    /// from where the loiter is left: its tangent's, an orbit's, the shorter) - its
    /// climbs no faster than it climbs them, and each loiter's own time from when it begins (`now` on); NaN where one's
    /// end is not known ahead.
    double takesS(const route::Plan& p, double fromM, double v, double alongMs, double now) const noexcept {
        auto legs = [&](double a, double b) { return p.climbCount ? p.climbTimeS(a, b, v, alongMs) : std::max(b - a, 0.0) / std::max(v, 0.1); };
        double t = 0.0, at = fromM;
        for (std::uint32_t m = 0; m < count; ++m) {
            t += legs(at, joinM[m]);
            t += route::loiterS(p, point[m], now + t);
            at = exitM[m];
        }
        return t + legs(at, toM);
    }
};

} // namespace

bool RouteBehavior::loitersAhead() const noexcept {
    const route::Plan& p = *plan_;
    if (arrivalPoint_ < 0 || arrivalState_ >= 0) return false;
    for (std::uint32_t k = segment_; k <= static_cast<std::uint32_t>(arrivalPoint_); ++k)
        if (route::loiterPoint(p.points[k])) return true;
    return false;
}

void RouteBehavior::scheduleThroughLoiters(const ControlContext& ctx, const Performance& perf, double routeM, route::Steer& steer) {
    const route::Plan& p = *plan_;
    const auto& s = ctx.sensed;
    const auto i = static_cast<std::uint32_t>(arrivalPoint_);
    const double now = ctx.world ? ctx.world->simTime() : s.simTime; // (its window's clock: the world's)
    const Ahead ahead(p, segment_, i, perf, hovers_);
    const double fromM = routeM - lapStartM_ + arrivalShiftM_;
    // as planned: its speed over the ground, in the wind now along its course; each loiter on its way its own time
    const double along = isHold(course_) ? 0.0 : wind_.northMs * std::cos(course_) + wind_.eastMs * std::sin(course_);
    const double climbAlong = hovers_ ? 0.0 : along;
    double speed = steer.reference == SpeedReference::GroundSpeed ? steer.speed : trueAirspeedOf(steer.speed, steer.reference, s) + along;
    const double eta = now + ahead.takesS(p, fromM, speed, climbAlong, now);
    if (std::isnan(eta)) { // (a loiter on its way whose end is not known ahead: flown as planned, estimated once it has ended)
        speedFlown_ = steer.speed, referenceFlown_ = static_cast<double>(steer.reference);
        arrivalS_ = arrivalDeltaS_ = kHold;
        return;
    }
    // beyond its window: to arrive a quarter of its width inside it (5 s at most, and past a side alone; a point window, at
    // it), within the speeds it flies level - the legs' one speed, the loiters flying their own
    const double begin = p.points[i].arrivalBeginS, end = p.points[i].arrivalEndS;
    const double inside = !isHold(begin) && !isHold(end) ? std::min(0.25 * (end - begin), 5.0) : 5.0;
    if (isHold(arrivalAimS_)) arrivalAimS_ = eta < begin ? begin + inside : eta > end ? end - inside : kHold; // (a side left out: never beyond it)
    const double aim = arrivalAimS_;
    if (!isHold(aim)) {
        double least = kHold, most = kHold;
        route::levelSpeedsMs(ctx.tables, perf, hovers_, s.altitudeMslM, s.fuelKg, least, most);
        const double lo = hovers_ ? least : least + along, hi = hovers_ ? most : most + along;
        if (aim - now > 1.0) { // (the least that arrives by its aim: its time to go falls as it flies faster)
            const double slow = std::isfinite(lo) ? std::max(lo, 0.5) : 0.5, fast = std::isfinite(hi) ? std::max(hi, slow) : 1000.0;
            const double seconds = aim - now;
            if (ahead.takesS(p, fromM, fast, climbAlong, now) > seconds) {
                speed = fast;
            } else if (ahead.takesS(p, fromM, slow, climbAlong, now) <= seconds) {
                speed = slow;
            } else {
                double a = slow, b = fast;
                for (int n = 0; n < 40; ++n) {
                    const double middle = 0.5 * (a + b);
                    (ahead.takesS(p, fromM, middle, climbAlong, now) > seconds ? a : b) = middle;
                }
                speed = b;
            }
        } else { // (its last second: the speed it asked, held; its aim past, as fast as it flies)
            speed = aim > now && !isHold(arrivalSpeedMs_) ? arrivalSpeedMs_ : hi;
        }
        if (std::isfinite(hi)) speed = std::min(speed, hi);
        if (std::isfinite(lo)) speed = std::max(speed, lo);
        speed = std::max(speed, 0.5);
        steer.speed = arrivalSpeedMs_ = speed, steer.reference = SpeedReference::GroundSpeed;
        if (p.climbCount) steer.speed = std::min(speed, climbMostHere(fromM, climbAlong)); // (in a climb, no faster than it climbs it)
    }
    speedFlown_ = steer.speed, referenceFlown_ = static_cast<double>(steer.reference);
    arrivalS_ = now + ahead.takesS(p, fromM, speed, climbAlong, now);
    arrivalDeltaS_ = arrivalS_ < begin - 1e-6 ? arrivalS_ - begin : arrivalS_ > end + 1e-6 ? arrivalS_ - end : 0.0; // (within it, to a microsecond)
}

void RouteBehavior::loiterBegun(const ControlContext& ctx, const PatternCommand& c, const PatternShape& shape) {
    const route::Plan& p = *plan_;
    const auto& s = ctx.sensed;
    // a window at its point met where its loiter begins: on to the next point's this lap
    if (arrivalPoint_ == static_cast<std::int32_t>(target_)) {
        arrivalPoint_ = -1;
        for (std::uint32_t i = target_ + 1; i < p.count && segmentFirstLap_; ++i)
            if (!isHold(p.points[i].arrivalBeginS) || !isHold(p.points[i].arrivalEndS)) {
                arrivalPoint_ = static_cast<std::int32_t>(i);
                break;
            }
        arrivalAimS_ = arrivalSpeedMs_ = arrivalS_ = arrivalDeltaS_ = kHold;
    }
    loiterEndsS_ = kHold;
    if (arrivalPoint_ < 0) return;
    // when it will be left, as it begins here: its pattern's way in, lap and exit point, its phase round it known
    const RouteLoiter& l = *loiterAhead_;
    const double now = ctx.world ? ctx.world->simTime() : s.simTime;
    const bool hover = c.pattern == static_cast<double>(PatternKind::Hover);
    route::LoiterMeasure m;
    if (hover) {
        const Waypoint& w = p.points[target_];
        const double away = geo::distanceM(s.latitudeRad, s.longitudeRad, w.latitudeRad, w.longitudeRad);
        m.approachS = 2.0 * away / std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), 0.5);
    } else {
        const route::Pattern& lap = *loiter_->pattern_;
        m.entryM = lap.entryM(), m.lapM = lap.lapM(), m.exitM = lap.exit >= 0 ? lap.toExitM() : -1.0;
        m.speedMs = route::plannedSpeed(c.speed, c.speedReference, s.altitudeMslM);
    }
    const double t = route::loiterTimeS(m, hover, shape.orbits, c.durationS, isHold(l.endTimeS) ? kHold : l.endTimeS - now, true);
    if (!std::isnan(t)) loiterEndsS_ = now + t;
    loiterShortM_ = 0.0; // (the leg on, laid from where it is left)
    if (p.leaves(target_) && !isHold(shape.exitLatitudeRad)) {
        const Waypoint &w = p.points[target_], &next = p.points[p.next(target_)];
        loiterShortM_ = route::makeLeg(w.latitudeRad, w.longitudeRad, next.latitudeRad, next.longitudeRad, p.rhumb).lengthM -
                        route::makeLeg(shape.exitLatitudeRad, shape.exitLongitudeRad, next.latitudeRad, next.longitudeRad, p.rhumb).lengthM;
    }
}

void RouteBehavior::loiterArrival(const ControlContext& ctx, const Performance& perf) {
    const route::Plan& p = *plan_;
    const auto i = static_cast<std::uint32_t>(arrivalPoint_);
    const double now = ctx.world ? ctx.world->simTime() : ctx.sensed.simTime;
    // (a hover's duration counts from its arrival over its point, known then; until then, at least its duration on - 4.25)
    double ends = loiterEndsS_;
    const PatternBehavior& hover = *loiter_;
    if (hover.pattern_->kind == PatternKind::Hover && !isHold(hover.resolved_.durationS)) {
        const double duration = hover.resolved_.durationS;
        ends = hover.startS_ >= 0.0 ? now + std::max(duration - (ctx.sensed.simTime - hover.startS_), 0.0) : std::fmax(ends, now + duration);
        if (!isHold(loiterAhead_->endTimeS)) ends = std::fmin(ends, std::max(loiterAhead_->endTimeS, now));
    }
    if (isHold(ends)) { // (its end not known ahead: estimated once it has ended)
        arrivalS_ = arrivalDeltaS_ = kHold;
        return;
    }
    // what the loiter has left, then the legs on from where it is left - at the speed its schedule last asked, else the next
    // point's in calm air - and the loiters after it (their own times)
    const double left = std::max(ends - now, 0.0);
    const Waypoint& next = p.points[p.next(target_)];
    const double v = !isHold(arrivalSpeedMs_) ? arrivalSpeedMs_ : route::plannedSpeed(next.speed, next.speedReference, ctx.sensed.altitudeMslM);
    const Ahead ahead(p, target_ + 1, i, perf, hovers_);
    const double t = left + ahead.takesS(p, p.exitM(target_) + loiterShortM_, v, 0.0, now + left);
    if (std::isnan(t)) {
        arrivalS_ = arrivalDeltaS_ = kHold;
        return;
    }
    const double begin = p.points[i].arrivalBeginS, end = p.points[i].arrivalEndS;
    arrivalS_ = now + t;
    arrivalDeltaS_ = arrivalS_ < begin - 1e-6 ? arrivalS_ - begin : arrivalS_ > end + 1e-6 ? arrivalS_ - end : 0.0; // (within it, to a microsecond)
}

} // namespace fsim::control
