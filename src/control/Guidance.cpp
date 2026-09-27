#include "fsim/GuidanceModes.h"

#include "control/Atmosphere.h"
#include "control/Registry.h"
#include "control/Route.h"
#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

double known(double v, double fallback) noexcept { return std::isnan(v) ? fallback : v; }

/// As the built-in loops' (Builtin.cpp): true when the behaviour missed a
/// period - another level flew meanwhile - so what it holds no longer describes the flight.
bool resumed(const ControlContext& ctx, double& lastTime) noexcept {
    const double t = ctx.state.simTime;
    const bool stale = lastTime >= 0.0 && (t - lastTime > 1.5 * ctx.dt || t < lastTime);
    lastTime = t;
    return stale;
}

SpeedReference speedReferenceOf(double code, SpeedReference fallback) noexcept {
    if (isHold(code) || code < 0.0 || code >= static_cast<double>(SpeedReference::Count)) return fallback;
    return static_cast<SpeedReference>(static_cast<int>(code));
}

AltitudeReference altitudeReferenceOf(double code) noexcept {
    if (isHold(code) || code < 0.0 || code >= static_cast<double>(AltitudeReference::Count)) return AltitudeReference::Msl;
    return static_cast<AltitudeReference>(static_cast<int>(code));
}

} // namespace

// --- Performance --------------------------------------------------------------------

double Performance::courseBandwidthRadS(double tasMs) const noexcept {
    if (hovers) return std::clamp(known(velocityBandwidthRadS, 0.3), 0.05, 5.0);
    if (std::isnan(headingGain)) return 0.2;
    // bank = gain * error turns the heading at g tan(bank) / v: a bandwidth of g gain / v, at the
    // speed a schedule holds it at, or at the speed it flies
    const double v = std::isnan(headingReferenceTasMs) ? std::max(tasMs, 10.0) : headingReferenceTasMs;
    return std::clamp(kG * headingGain / v, 0.02, 2.0);
}

double Performance::turnRadiusM(double speedMs) const noexcept {
    const double v2 = speedMs * speedMs;
    if (hovers) return v2 / std::max(0.8 * known(maxAccelerationMs2, kG * std::tan(0.35)), 0.1);
    const double bank = 0.8 * known(maxBankRad, 0.52);
    return v2 / (kG * std::tan(std::max(bank, 0.05)));
}

// --- References ---------------------------------------------------------------------

void WindEstimate::update(const sim::VehicleState& s, double dt) noexcept {
    const double v = s.airspeedTrueMs;
    const double ca = std::cos(s.alphaRad), sa = std::sin(s.alphaRad), cb = std::cos(s.betaRad), sb = std::sin(s.betaRad);
    const double u = v * ca * cb, w = v * sb, z = v * sa * cb; // the air velocity along the body's axes
    const double cf = std::cos(s.eulerRad[0]), sf = std::sin(s.eulerRad[0]);
    const double ct = std::cos(s.eulerRad[1]), st = std::sin(s.eulerRad[1]);
    const double cp = std::cos(s.eulerRad[2]), sp = std::sin(s.eulerRad[2]);
    const double airNorth = ct * cp * u + (sf * st * cp - cf * sp) * w + (cf * st * cp + sf * sp) * z;
    const double airEast = ct * sp * u + (sf * st * sp + cf * cp) * w + (cf * st * sp - sf * cp) * z;
    const double n = s.velocityNedMs[0] - airNorth, e = s.velocityNedMs[1] - airEast;
    if (!std::isfinite(n) || !std::isfinite(e)) return;
    if (!valid) {
        northMs = n, eastMs = e, valid = true;
        return;
    }
    const double k = std::min(1.0, dt / std::max(timeConstantS, 1e-3));
    northMs += (n - northMs) * k;
    eastMs += (e - eastMs) * k;
}

double trueAirspeedOf(double speed, SpeedReference reference, const sim::VehicleState& s) noexcept {
    switch (reference) {
    case SpeedReference::TrueAirspeed: return speed;
    case SpeedReference::CalibratedAirspeed:
        return s.airspeedCalibratedMs > 10.0 ? speed * s.airspeedTrueMs / s.airspeedCalibratedMs : isa::trueFromCalibrated(speed, s.altitudeMslM);
    case SpeedReference::Mach: return speed * (s.mach > 0.05 ? s.airspeedTrueMs / s.mach : isa::speedOfSound(s.altitudeMslM));
    default: return std::numeric_limits<double>::quiet_NaN();
    }
}

double speedNow(SpeedReference reference, const sim::VehicleState& s) noexcept {
    switch (reference) {
    case SpeedReference::TrueAirspeed: return s.airspeedTrueMs;
    case SpeedReference::CalibratedAirspeed: return s.airspeedCalibratedMs;
    case SpeedReference::GroundSpeed: return std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    case SpeedReference::Mach: return s.mach;
    default: return std::numeric_limits<double>::quiet_NaN();
    }
}

double altitudeMslOf(double altitudeM, AltitudeReference reference, const sim::VehicleState& s) noexcept {
    return reference == AltitudeReference::AboveGround ? altitudeM + (s.altitudeMslM - s.altitudeAglM) : altitudeM;
}

double altitudeNow(AltitudeReference reference, const sim::VehicleState& s) noexcept {
    return reference == AltitudeReference::AboveGround ? s.altitudeAglM : s.altitudeMslM;
}

// --- HsaBehavior --------------------------------------------------------------------

void HsaBehavior::begin(const ControlContext&, const Command& command) {
    reset();
    if (const auto* h = std::get_if<HsaCommand>(&command)) flown_ = *h;
}

void HsaBehavior::reset() {
    wind_.reset();
    courseTrim_ = speedTrim_ = 0.0;
    lastTime_ = -1.0;
}

Command HsaBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    const bool hovers = (ctx.features & kFeatureHover) != 0;
    if (resumed(ctx, lastTime_)) courseTrim_ = speedTrim_ = 0.0, wind_.reset();
    wind_.update(s, ctx.dt);
    VelocityCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    const auto* h = std::get_if<HsaCommand>(&in);
    if (!h) { // (the runtime gives it its own setpoint) hold what it flies
        out.verticalSpeedMs = 0.0, out.headingRad = s.eulerRad[2];
        if (hovers) out.northMs = out.eastMs = 0.0;
        else out.airspeedMs = s.airspeedTrueMs;
        return out;
    }
    flown_ = *h;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;

    // the altitude: a vertical speed at the position loop's gain, within its limits
    const AltitudeReference altitudeReference = altitudeReferenceOf(h->altitudeReference);
    altitudeMsl_ = isHold(h->altitudeM) ? s.altitudeMslM : altitudeMslOf(h->altitudeM, altitudeReference, s);
    const double gain = known(perf.altitudeGainPerS, hovers ? 0.5 : 0.25);
    const double climb = known(perf.maxClimbMs, hovers ? 3.0 : 6.0), descent = known(perf.maxDescentMs, climb);
    out.verticalSpeedMs = std::clamp(gain * (altitudeMsl_ - s.altitudeMslM), -descent, climb);

    // the direction, and the wind along it and to its right
    const bool course = !isHold(h->courseRad);
    const double direction = course ? h->courseRad : orHold(h->headingRad, s.eulerRad[2]);
    const double tn = std::cos(direction), te = std::sin(direction);
    const double windAlong = wind_.northMs * tn + wind_.eastMs * te, windAcross = -wind_.northMs * te + wind_.eastMs * tn;
    const SpeedReference reference = speedReferenceOf(h->speedReference, hovers ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
    const double speed = isHold(h->speed) ? speedNow(reference, s) : h->speed;

    if (hovers) {
        if (reference == SpeedReference::GroundSpeed) {
            // over the ground along the heading or the course, the nose along it
            out.northMs = speed * tn, out.eastMs = speed * te;
            out.headingRad = direction;
        } else {
            const double tas = trueAirspeedOf(speed, reference, s);
            if (course) {
                // the air velocity whose track is the course: turned into the wind across it, the nose along it
                const double crab = std::asin(std::clamp(-windAcross / std::max(tas, 0.5), -0.9, 0.9));
                const double along = std::max(tas * std::cos(crab) + windAlong, 0.0);
                out.northMs = along * tn, out.eastMs = along * te;
                out.headingRad = geo::wrapPi(direction + crab);
            } else {
                out.airspeedMs = tas; // along the nose, through the air
                out.headingRad = direction;
            }
        }
        headingFlown_ = out.headingRad;
        return out;
    }

    // a wing: the heading its track needs, and the airspeed
    const double tasNow = std::max(s.airspeedTrueMs, 10.0);
    const double crab = std::asin(std::clamp(-windAcross / tasNow, -0.8, 0.8)); // off the track, into the wind
    const double north = s.velocityNedMs[0], east = s.velocityNedMs[1], ground = std::hypot(north, east);
    const double bandwidth = perf.courseBandwidthRadS(tasNow);
    double tas;
    if (reference == SpeedReference::GroundSpeed) {
        // the airspeed that makes this speed over the ground along its track, and a slow trim on what the wind estimate misses
        if (course) {
            tas = (speed - windAlong) / std::max(std::cos(crab), 0.3);
        } else { // along the heading: |tas h + wind| = speed
            const double w2 = wind_.northMs * wind_.northMs + wind_.eastMs * wind_.eastMs;
            tas = -windAlong + std::sqrt(std::max(windAlong * windAlong - w2 + speed * speed, 0.0));
        }
        const double error = speed - ground;
        if (std::abs(error) < 5.0) speedTrim_ = std::clamp(speedTrim_ + 0.05 * error * ctx.dt, -5.0, 5.0);
        tas += speedTrim_;
    } else {
        speedTrim_ = 0.0;
        tas = trueAirspeedOf(speed, reference, s);
    }
    out.airspeedMs = std::max(tas, 0.0);
    if (course) {
        // the heading that holds the course against the wind, trimmed slowly on the course error once near it
        if (ground > 5.0) {
            const double error = geo::wrapPi(direction - std::atan2(east, north));
            if (std::abs(error) < 0.2) courseTrim_ = std::clamp(courseTrim_ + 0.25 * bandwidth * bandwidth * error * ctx.dt, -0.3, 0.3);
        }
        out.headingRad = geo::wrapPi(direction + crab + courseTrim_);
    } else {
        courseTrim_ = 0.0;
        out.headingRad = direction;
    }
    headingFlown_ = out.headingRad;
    return out;
}

bool HsaBehavior::progress(ActivityProgress& out) const noexcept {
    out.courseRad = flown_.courseRad;
    out.headingRad = isHold(flown_.courseRad) ? flown_.headingRad : headingFlown_;
    out.altitudeMslM = altitudeMsl_;
    out.speedMs = flown_.speed;
    out.speedReference = flown_.speedReference;
    return true;
}

// --- RouteBehavior ------------------------------------------------------------------

namespace {

bool same(const RouteCommand& a, const RouteCommand& b) noexcept {
    auto eq = [](double x, double y) { return x == y || (isHold(x) && isHold(y)); };
    return eq(a.projection, b.projection) && eq(a.repeat, b.repeat) && eq(a.end, b.end) && eq(a.start, b.start);
}

/// An option's value: kHold its default, else the whole number within [0, count).
double option(double v, double count) noexcept { return isHold(v) ? 0.0 : std::clamp(std::floor(v), 0.0, count - 1.0); }

} // namespace

RouteBehavior::RouteBehavior() : plan_(std::make_unique<route::Plan>()) {}
RouteBehavior::~RouteBehavior() = default;

void RouteBehavior::begin(const ControlContext& ctx, const Command& command) {
    reset();
    wind_.update(ctx.sensed, ctx.dt); // what the first turns are planned with
    if (const auto* r = std::get_if<RouteCommand>(&command)) restart(ctx, *r);
}

void RouteBehavior::reset() {
    wind_.reset();
    courseTrim_ = speedTrim_ = 0.0;
    lastTime_ = -1.0;
    planned_ = false; // planned afresh from where the aircraft is
}

bool RouteBehavior::stops() const noexcept {
    const route::Plan& p = *plan_;
    return hovers_ && p.end == EndBehavior::Loiter && !p.repeat && target_ == p.last();
}

void RouteBehavior::restart(const ControlContext& ctx, const RouteCommand& command) {
    route::Plan& p = *plan_;
    const auto& s = ctx.sensed;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    hovers_ = (ctx.features & kFeatureHover) != 0;
    flown_ = command;
    planned_ = true;
    revision_ = ctx.path ? ctx.path->revision : 0;
    target_ = 0, laps_ = 0;
    firstLap_ = true, onArc_ = midway_ = ended_ = finished_ = false;
    failure_ = Reason::None;
    leadOut_ = finishedM_ = inPieceM_ = lapStartM_ = 0.0;
    const std::uint32_t count = ctx.path ? ctx.path->count : 0;
    p.repeat = option(command.repeat, 2.0) == 1.0 && count > 1;
    p.rhumb = option(command.projection, 2.0) == 1.0;
    p.end = static_cast<EndBehavior>(static_cast<int>(option(command.end, 2.0)));
    // what the waypoints leave out, as the host fills it in (it has, for a World's vehicle)
    std::int16_t bad = -1;
    if (count == 0 || route::complete(p.points, ctx.path->waypoints, count, p.repeat, s, perf, hovers_, bad) != Reason::None) {
        p.count = 0;
        failure_ = Reason::BehaviorFailed; // (only a stack on its own is given a route nobody checked)
        return;
    }
    p.count = count;
    p.start = static_cast<std::uint32_t>(option(command.start, static_cast<double>(count)));
    route::plan(p, s.latitudeRad, s.longitudeRad, s.altitudeMslM, std::hypot(wind_.northMs, wind_.eastMs), perf, hovers_);
    target_ = p.start;
    lapM_ = p.lapM(true);
    beginSegment(p.start, s, 0.0, 0.0, 0.0, true, false);
}

void RouteBehavior::beginSegment(std::uint32_t k, const sim::VehicleState& s, double atM, double halfArcM, double leadM, bool firstLap, bool fromPoint) {
    const route::Plan& p = *plan_;
    const std::uint32_t from = segment_;
    segment_ = k;
    segmentStartM_ = atM;
    segmentStartS_ = s.simTime;
    const route::Turn& turn = p.turn(k, firstLap);
    segmentM_ = halfArcM + std::max(0.0, p.leg(k, firstLap).lengthM - leadM - turn.leadM) + 0.5 * turn.radiusM * std::abs(turn.angleRad);
    // it climbs from the previous point's altitude, in the same reference; else from the aircraft's own now
    const Waypoint& to = p.points[k];
    if (fromPoint && p.points[from].altitudeReference == to.altitudeReference) segmentFrom_ = p.points[from].altitudeM;
    else segmentFrom_ = altitudeNow(altitudeReferenceOf(to.altitudeReference), s);
}

void RouteBehavior::advance(const sim::VehicleState& s, const Performance& perf) {
    route::Plan& p = *plan_;
    onArc_ = midway_ = false;
    if (target_ == p.last() && !p.repeat) {
        ended_ = finished_ = true; // then on along the last leg, or loitering at its point
        if (p.end == EndBehavior::Loiter && !hovers_) {
            const Waypoint& w = p.points[target_];
            const double v = route::plannedSpeed(w.speed, w.speedReference, s.altitudeMslM) + std::hypot(wind_.northMs, wind_.eastMs);
            p.orbit = route::Turn{};
            p.orbit.radiusM = isHold(w.maxBankRad) ? perf.turnRadiusM(v) : v * v / (kG * std::tan(w.maxBankRad));
            p.orbit.angleRad = 1.0; // right turns, round the point itself
        }
        return;
    }
    if (target_ == p.last()) { // a lap flown: the route again from its first point
        firstLap_ = false;
        ++laps_;
        lapStartM_ = finishedM_;
        lapM_ = p.lapM(false);
    }
    target_ = p.next(target_);
}

route::Fix RouteBehavior::locate(const sim::VehicleState& s, const Performance& perf) {
    const route::Plan& p = *plan_;
    const double lat = s.latitudeRad, lon = s.longitudeRad;
    for (int passed = 0;; ++passed) {
        const Waypoint& point = p.points[target_];
        if (ended_) {
            inPieceM_ = 0.0;
            if (p.end == EndBehavior::Loiter && !hovers_) return route::onArc(p.orbit, point.latitudeRad, point.longitudeRad, lat, lon);
            return route::onLeg(p.leg(target_, firstLap_), lat, lon); // the last leg, on beyond its point
        }
        const route::Turn& turn = p.turn(target_, firstLap_);
        const bool more = passed < 8; // (at most so many pieces a control period: the rest at the next)
        if (onArc_) {
            const route::Fix f = route::onArc(turn, point.latitudeRad, point.longitudeRad, lat, lon);
            inPieceM_ = f.alongM;
            const double angle = std::abs(turn.angleRad), swept = f.alongM / turn.radiusM;
            if (!midway_ && swept >= 0.5 * angle && p.leaves(target_)) {
                // the next segment begins at the turn's middle, nearest the point
                midway_ = true;
                const bool lap = firstLap_ && target_ != p.last();
                beginSegment(p.next(target_), s, finishedM_ + f.alongM, 0.5 * turn.radiusM * angle, turn.leadM, lap, true);
            }
            if (swept < angle || !more) return f;
            finishedM_ += turn.radiusM * angle;
            leadOut_ = turn.leadM;
            advance(s, perf);
            continue;
        }
        const route::Leg& leg = p.leg(target_, firstLap_);
        const route::Fix f = route::onLeg(leg, lat, lon);
        inPieceM_ = f.alongM - leadOut_;
        if (!more) return f;
        if (turn.radiusM > 0.0) {
            if (f.alongM < leg.lengthM - turn.leadM) return f;
            finishedM_ += std::max(0.0, leg.lengthM - leadOut_ - turn.leadM);
            onArc_ = true, midway_ = false;
            continue;
        }
        // no arc: the point is passed abeam (a rotorcraft that stops there, within a metre of it)
        if (f.alongM < leg.lengthM - (stops() ? 1.0 : 0.0)) return f;
        finishedM_ += std::max(0.0, leg.lengthM - leadOut_);
        if (p.leaves(target_)) beginSegment(p.next(target_), s, finishedM_, 0.0, 0.0, firstLap_ && target_ != p.last(), true);
        leadOut_ = 0.0;
        advance(s, perf);
    }
}

Command RouteBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    if (resumed(ctx, lastTime_)) courseTrim_ = speedTrim_ = 0.0, wind_.reset();
    wind_.update(s, ctx.dt);
    const auto* command = std::get_if<RouteCommand>(&in);
    if (command && (!planned_ || !ctx.path || ctx.path->revision != revision_ || !same(*command, flown_))) restart(ctx, *command);
    const route::Plan& p = *plan_;
    groundSpeed_ = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    if (!command || p.count == 0) { // nothing to fly: on as it flies (a rotorcraft still)
        VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
        if (hovers_) hold.northMs = hold.eastMs = 0.0;
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    const route::Fix fix = locate(s, perf);
    const Waypoint& segment = p.points[segment_];

    // the altitude: straight from the segment's start to its point, or at its climb rate; then held
    const double routeM = finishedM_ + inPieceM_;
    const double span = segment.altitudeM - segmentFrom_;
    double altitude = segment.altitudeM, feedforward = 0.0;
    if (!ended_ && !isHold(segment.climbRateMs)) {
        const double climbed = segment.climbRateMs * (s.simTime - segmentStartS_);
        if (climbed < std::abs(span)) altitude = segmentFrom_ + std::copysign(climbed, span), feedforward = std::copysign(segment.climbRateMs, span);
    } else if (!ended_ && segmentM_ > 1.0) {
        const double fraction = std::clamp((routeM - segmentStartM_) / segmentM_, 0.0, 1.0);
        altitude = segmentFrom_ + span * fraction;
        if (fraction < 1.0) feedforward = span / segmentM_ * groundSpeed_;
    }
    altitudeMsl_ = altitudeMslOf(altitude, altitudeReferenceOf(segment.altitudeReference), s);
    const double gain = known(perf.altitudeGainPerS, hovers_ ? 0.5 : 0.25);
    const double climb = known(perf.maxClimbMs, hovers_ ? 3.0 : 6.0), descent = known(perf.maxDescentMs, climb);
    const double verticalSpeed = std::clamp(feedforward + gain * (altitudeMsl_ - s.altitudeMslM), -descent, climb);

    // the path's course and the wind along it and across it
    const double tn = std::cos(fix.courseRad), te = std::sin(fix.courseRad);
    const double windAlong = wind_.northMs * tn + wind_.eastMs * te, windAcross = -wind_.northMs * te + wind_.eastMs * tn;
    const SpeedReference reference = speedReferenceOf(segment.speedReference, hovers_ ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
    const double e = fix.crossTrackM;
    crossTrack_ = e;
    // The path's curvature `preview` metres ahead: what a loop that lags should
    // fly now - a turn begun before its arc and ended before the arc does.
    auto curvatureAhead = [&](double preview) {
        const route::Turn& turn = p.turn(target_, firstLap_);
        if (ended_ || turn.radiusM <= 0.0) return fix.curvature;
        if (onArc_) return turn.radiusM * std::abs(turn.angleRad) - inPieceM_ < preview ? 0.0 : fix.curvature;
        const double toArc = p.leg(target_, firstLap_).lengthM - turn.leadM - (inPieceM_ + leadOut_);
        return toArc < preview ? (turn.angleRad >= 0.0 ? 1.0 : -1.0) / turn.radiusM : fix.curvature;
    };

    if (hovers_) {
        const Waypoint& last = p.points[p.last()];
        if (ended_ && p.end == EndBehavior::Loiter) { // stopped: hover over the last point
            course_ = heading_ = kHold;
            return PositionCommand{last.latitudeRad, last.longitudeRad, altitudeMsl_, kHold, 1.0, kHold};
        }
        // the path speed over the ground: the segment's, or what its airspeed makes along the path in the wind
        double v = segment.speed;
        if (reference != SpeedReference::GroundSpeed) {
            const double tas = trueAirspeedOf(segment.speed, reference, s);
            const double crab = std::asin(std::clamp(-windAcross / std::max(tas, 0.5), -0.9, 0.9));
            v = std::max(tas * std::cos(crab) + windAlong, 0.0);
        }
        // no faster than a turn's radius allows (and than stops it in time for one), or than stops it at the end
        const double lateral = 0.8 * known(perf.maxAccelerationMs2, kG * std::tan(0.35));
        const double braking = 0.8 * known(perf.maxDecelerationMs2, known(perf.maxAccelerationMs2, kG * std::tan(0.35)));
        if (!ended_) {
            const route::Turn& turn = p.turn(target_, firstLap_);
            if (onArc_) {
                v = std::min(v, std::sqrt(lateral * turn.radiusM));
            } else {
                const double toGo = std::max(p.leg(target_, firstLap_).lengthM - std::max(inPieceM_ + leadOut_, 0.0) - turn.leadM, 0.0);
                if (turn.radiusM > 0.0) v = std::min(v, std::sqrt(lateral * turn.radiusM + 2.0 * braking * toGo));
                else if (stops()) v = std::min(v, std::max(std::sqrt(2.0 * braking * toGo), std::min(0.5, v)));
            }
        }
        // line of sight to the path a few velocity-loop time constants ahead,
        // turned ahead of its curve by the loop's lag (and so the curve there)
        const double bandwidth = perf.courseBandwidthRadS(0.0);
        const double lookahead = std::max(3.0 * v / bandwidth, 3.0);
        const double course = geo::wrapPi(fix.courseRad - std::atan(e / lookahead) + curvatureAhead(v / bandwidth) * v / bandwidth);
        course_ = heading_ = course;
        return VelocityCommand{kHold, verticalSpeed, course, kHold, v * std::cos(course), v * std::sin(course)};
    }

    // a wing: the airspeed, and the turn rate that holds the path
    const double tasNow = std::max(s.airspeedTrueMs, 10.0);
    double tas;
    if (reference == SpeedReference::GroundSpeed) {
        // the airspeed that makes the ground speed along the path, and a slow trim on what the wind estimate misses
        const double crab = std::asin(std::clamp(-windAcross / tasNow, -0.8, 0.8));
        tas = (segment.speed - windAlong) / std::max(std::cos(crab), 0.3);
        const double error = segment.speed - (s.velocityNedMs[0] * tn + s.velocityNedMs[1] * te);
        if (std::abs(error) < 5.0) speedTrim_ = std::clamp(speedTrim_ + 0.05 * error * ctx.dt, -5.0, 5.0);
        tas += speedTrim_;
    } else {
        speedTrim_ = 0.0;
        tas = trueAirspeedOf(segment.speed, reference, s);
    }
    const double vg = std::max(groundSpeed_, 5.0);
    const double bandwidth = perf.courseBandwidthRadS(tasNow);
    const double lookahead = std::max(3.0 * vg / bandwidth, 50.0);
    // the turn anticipated: its curvature fed forward as long before it as the
    // roll into it lags - half the bank at the loop's rate, and the roll's own
    // time constant (a fifth of the heading loop's, or faster)
    double curvature = fix.curvature;
    if (const route::Turn& turn = p.turn(target_, firstLap_); !ended_ && turn.radiusM > 0.0) {
        const double bank = std::atan(vg * vg / (kG * turn.radiusM));
        curvature = curvatureAhead(vg * (0.5 * bank / std::max(known(perf.bankRateRadS, 0.35), 0.05) + std::clamp(0.2 / bandwidth, 0.3, 1.5)));
    }
    // the course to the path, and the turn rate that flies it: proportional, and an
    // integral (its zero at a quarter of the bandwidth) on what the turn-rate loop
    // leaves - an aircraft that needs a little bank to fly straight - once near it
    const double course = geo::wrapPi(fix.courseRad - std::atan(e / lookahead));
    const double error = geo::wrapPi(course - std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]));
    if (std::abs(error) < 0.15) courseTrim_ = std::clamp(courseTrim_ + 0.25 * bandwidth * bandwidth * error * ctx.dt, -0.05, 0.05);
    course_ = course, heading_ = kHold;
    return VelocityCommand{std::max(tas, 0.0), verticalSpeed, kHold, curvature * vg + bandwidth * error + courseTrim_, kHold, kHold};
}

bool RouteBehavior::progress(ActivityProgress& out) const noexcept {
    const route::Plan& p = *plan_;
    if (!planned_ || p.count == 0) return false;
    const Waypoint& segment = p.points[segment_];
    out.segment = segment_;
    out.segments = p.count;
    out.segmentId = segment.id;
    out.laps = laps_;
    const double routeM = finishedM_ + inPieceM_;
    if (ended_) {
        out.percent = out.segmentPercent = 100.0;
    } else {
        out.segmentPercent = segmentM_ > 1e-6 ? std::clamp(100.0 * (routeM - segmentStartM_) / segmentM_, 0.0, 100.0) : 100.0;
        out.percent = lapM_ > 1e-6 ? std::clamp(100.0 * (routeM - lapStartM_) / lapM_, 0.0, 100.0) : 100.0; // of the lap, when it repeats
    }
    if (!p.repeat) { // a route that repeats has no end to go to
        out.distanceToGoM = ended_ ? 0.0 : std::max(lapM_ - routeM, 0.0);
        if (ended_) out.timeToGoS = 0.0;
        else if (groundSpeed_ > 0.1) out.timeToGoS = out.distanceToGoM / groundSpeed_;
    }
    out.crossTrackM = crossTrack_;
    out.courseRad = course_;
    out.headingRad = heading_;
    out.altitudeMslM = altitudeMsl_;
    out.speedMs = segment.speed;
    out.speedReference = segment.speedReference;
    return true;
}

// --- Registration -------------------------------------------------------------------

void registerGuidanceModes(ControllerRegistry& r) {
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi, Constraint below = Constraint::None,
                Constraint above = Constraint::None) { return ParameterInfo{name, unit, lo, hi, def, true, below, above}; };
    // a mode's parameters are its setpoint's fields, in order (the C ABI's too); left out: as the host resolves
    BehaviorTraits hsa;
    hsa.persistence = Persistence::Persistent;
    hsa.parameters = {p("heading_rad", "rad", now, -inf, inf),
                      p("course_rad", "rad", now, -inf, inf),
                      p("speed", "m/s or Mach", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                      p("speed_reference", "", now, 0.0, static_cast<double>(SpeedReference::Count) - 1.0),
                      p("altitude_m", "m", now, -inf, inf, Constraint::MinAltitude, Constraint::MaxAltitude),
                      p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0)};
    hsa.uses = {"fsim.flight.velocity"};
    hsa.mode = FlightMode::HsaCsa;
    hsa.setpoint = SetpointKind::Hsa;
    r.addBehavior("hsa", [] { return std::make_unique<HsaBehavior>(); }, std::move(hsa));
    // the route's options; its waypoints go beside them into the path store
    BehaviorTraits route;
    route.persistence = Persistence::Terminating;
    route.parameters = {p("projection", "", 0.0, 0.0, static_cast<double>(Projection::Count) - 1.0),
                        p("repeat", "", 0.0, 0.0, 1.0),
                        p("end", "", 0.0, 0.0, static_cast<double>(EndBehavior::Count) - 1.0),
                        p("start", "", 0.0, 0.0, static_cast<double>(PathStore::kWaypoints) - 1.0)};
    route.uses = {"fsim.flight.velocity", "fsim.flight.position"};
    route.mode = FlightMode::WaypointFollowing;
    route.setpoint = SetpointKind::Route;
    r.addBehavior("route", [] { return std::make_unique<RouteBehavior>(); }, std::move(route));
}

} // namespace fsim::control
