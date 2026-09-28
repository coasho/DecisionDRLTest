#include "fsim/GuidanceModes.h"

#include "control/Atmosphere.h"
#include "control/Registry.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
#include "fsim/Magnetic.h"
#include "fsim/VehicleProfile.h"

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
    // a rotorcraft's: at 80 % of its acceleration, and no tighter than its velocity loop follows - a turn
    // rate a third of its bandwidth, the look-ahead a path is flown with (route::follow)
    if (hovers) return std::max(v2 / std::max(0.8 * known(maxAccelerationMs2, kG * std::tan(0.35)), 0.1), 3.0 * std::abs(speedMs) / courseBandwidthRadS(speedMs));
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

double altitudeMslOf(double altitudeM, AltitudeReference reference, const sim::VehicleState& s, const Altimeter* altimeter) noexcept {
    if (reference == AltitudeReference::AboveGround) return altitudeM + (s.altitudeMslM - s.altitudeAglM);
    if (reference == AltitudeReference::Barometric) return barometricMslM(altimeter ? *altimeter : Altimeter{}, altitudeM);
    return altitudeM;
}

double altitudeNow(AltitudeReference reference, const sim::VehicleState& s, const Altimeter* altimeter) noexcept {
    if (reference == AltitudeReference::AboveGround) return s.altitudeAglM;
    if (reference == AltitudeReference::Barometric) return indicatedAltitudeM(altimeter ? *altimeter : Altimeter{}, s.altitudeMslM);
    return s.altitudeMslM;
}

double optimalTasMs(const TablesSection* tables, double optimization, double altitudeMslM, double fuelKg) noexcept {
    constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    if (!tables || tables->empty() || isHold(optimization)) return kNone;
    const TablesSection& t = *tables;
    // the weight: the fuel on board on the tables' weight with the tanks empty (their heaviest is flown full)
    double weight = t.weightKg.back();
    if (t.weightKg.size() > 1 && std::isfinite(t.fuelCapacityKg) && std::isfinite(fuelKg)) weight += fuelKg - t.fuelCapacityKg;
    const TablesAt at = tablesAt(t, altitudeMslM, weight);
    if (optimization == static_cast<double>(SpeedOptimization::LongRangeCruise)) return at.bestRangeTasMs;
    if (optimization == static_cast<double>(SpeedOptimization::MaxEndurance)) return at.bestEnduranceTasMs;
    return kNone;
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
    declinationAt_ = kHold;
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
    altitudeMsl_ = isHold(h->altitudeM) ? s.altitudeMslM : altitudeMslOf(h->altitudeM, altitudeReference, s, ctx.altimeter);
    const double gain = known(perf.altitudeGainPerS, hovers ? 0.5 : 0.25);
    const double climb = known(perf.maxClimbMs, hovers ? 3.0 : 6.0), descent = known(perf.maxDescentMs, climb);
    out.verticalSpeedMs = std::clamp(gain * (altitudeMsl_ - s.altitudeMslM), -descent, climb);

    // the direction, and the wind along it and to its right
    const bool course = !isHold(h->courseRad);
    double direction = course ? h->courseRad : orHold(h->headingRad, s.eulerRad[2]);
    if (h->directionReference == static_cast<double>(DirectionReference::MagneticNorth) && (course || !isHold(h->headingRad))) {
        // a magnetic one, turned by the declination where the aircraft is, at the world's date, every 10 s (4.22)
        if (!(s.simTime - declinationAt_ < 10.0)) {
            const double utc = ctx.world ? ctx.world->environment().epochUtcSeconds + s.simTime : 0.0;
            declination_ = declinationRad(s.latitudeRad, s.longitudeRad, s.altitudeMslM, magneticYear(utc));
            declinationAt_ = s.simTime;
        }
        direction = geo::wrapPi(direction + declination_);
    }
    const double tn = std::cos(direction), te = std::sin(direction);
    const double windAlong = wind_.northMs * tn + wind_.eastMs * te, windAcross = -wind_.northMs * te + wind_.eastMs * tn;
    SpeedReference reference = speedReferenceOf(h->speedReference, hovers ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
    double speed = isHold(h->speed) ? speedNow(reference, s) : h->speed;
    if (!isHold(h->speedOptimization)) { // the tables' best at the altitude and weight now (where they give none: as resolved)
        const double best = optimalTasMs(ctx.tables, h->speedOptimization, s.altitudeMslM, s.fuelKg);
        reference = SpeedReference::TrueAirspeed;
        speed = std::isfinite(best) ? best : isHold(h->speed) ? s.airspeedTrueMs : h->speed;
    }
    speedFlown_ = speed;

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
    // (the heading a course is flown on, from the north it was commanded from)
    const bool magnetic = flown_.directionReference == static_cast<double>(DirectionReference::MagneticNorth);
    out.headingRad = isHold(flown_.courseRad) ? flown_.headingRad : isHold(headingFlown_) || !magnetic ? headingFlown_ : geo::wrapPi(headingFlown_ - declination_);
    out.altitudeMslM = altitudeMsl_;
    const bool optimised = !isHold(flown_.speedOptimization); // (the optimum it flies now, a true airspeed)
    out.speedMs = optimised && !isHold(speedFlown_) ? speedFlown_ : flown_.speed;
    out.speedReference = optimised ? static_cast<double>(SpeedReference::TrueAirspeed) : flown_.speedReference;
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

namespace {

/// A route point's frame as the host placed it (its store's table): null for none (docs/flight-autonomy.md, 4.29).
const FrameSpec* routeFrame(const PathStore* store, double frame) noexcept {
    if (!store || isHold(frame)) return nullptr;
    const auto id = static_cast<FrameId>(frame);
    for (std::uint32_t k = 0; k < store->routeFrameCount; ++k)
        if (store->routeFrameIds[k] == id) return &store->routeFrames[k];
    return nullptr;
}

bool moves(const FrameSpec* f) noexcept { return f && f->origin != FrameOrigin::Fixed; }

} // namespace

bool RouteBehavior::place(const ControlContext& ctx, std::uint32_t i, FramePose& pose) {
    Waypoint& w = plan_->points[i];
    const FrameSpec* f = routeFrame(ctx.path, w.frame);
    if (!moves(f)) return true; // (where the host placed it)
    if (f->origin == FrameOrigin::Vehicle) {
        const sim::VehicleState* v = ctx.world ? ctx.world->vehicleState(f->vehicle) : nullptr;
        if (!v) return false;
        pose = vehiclePose(*v);
    } else {
        pose = framePose(*f, ctx.world ? ctx.world->simTime() : ctx.sensed.simTime);
    }
    const GeoPoint at = framePoint(pose, w.frameOffset());
    w.latitudeRad = at.latitudeRad, w.longitudeRad = geo::wrapPi(at.longitudeRad);
    if (!isHold(w.frameZM)) w.altitudeM = at.altitudeMslM;
    return true;
}

void RouteBehavior::begin(const ControlContext& ctx, const Command& command) {
    reset();
    wind_.update(ctx.sensed, ctx.dt); // what the first turns are planned with
    if (const auto* r = std::get_if<RouteCommand>(&command)) restart(ctx, *r);
}

void RouteBehavior::reset() {
    wind_.reset();
    plan_->trims = route::Trims{};
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
    if (count == 0 || route::complete(p.points, ctx.path->waypoints, count, p.repeat, s, perf, hovers_, bad, ctx.altimeter) != Reason::None) {
        p.count = 0;
        failure_ = Reason::BehaviorFailed; // (only a stack on its own is given a route nobody checked)
        return;
    }
    p.count = count;
    // its points in moving frames where the frames are now (4.29): placed and planned again as it flies them
    moving_ = overFrame_ = false;
    for (std::uint32_t i = 0; i < count; ++i) {
        if (!moves(routeFrame(ctx.path, p.points[i].frame))) continue;
        moving_ = true;
        FramePose pose;
        if (!place(ctx, i, pose)) { // (its frame's vehicle gone: nothing to fly to)
            p.count = 0;
            failure_ = Reason::TargetLost;
            return;
        }
    }
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
        lastCross_ = f.crossTrackM;
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
    if (resumed(ctx, lastTime_)) plan_->trims = route::Trims{}, wind_.reset();
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
    if (moving_) { // its points in moving frames where they are, as the step began, and the piece it flies planned again (4.29)
        route::Plan& q = *plan_;
        const std::uint32_t at = ended_ ? q.last() : target_;
        const bool entry = firstLap_ && at == q.start, before = !ended_ && !entry && (at > 0 || q.repeat), after = !ended_ && q.leaves(at);
        FramePose pose, other;
        if (!place(ctx, at, pose) || (before && !place(ctx, q.prev(at), other)) || (after && !place(ctx, q.next(at), other))) {
            failure_ = Reason::TargetLost; // (its frame's vehicle gone: on as it flies, a rotorcraft still)
            VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
            if (hovers_) hold.northMs = hold.eastMs = 0.0;
            else hold.airspeedMs = s.airspeedTrueMs;
            return hold;
        }
        if (!ended_) route::replan(q, at, firstLap_, s.altitudeMslM, std::hypot(wind_.northMs, wind_.eastMs), perf, hovers_);
        // flown over the frame the piece is in: its point's and the one before's, one moving frame (past the end, the last point's)
        const Waypoint& w = q.points[at];
        overFrame_ = moves(routeFrame(ctx.path, w.frame)) && (ended_ || (before && q.points[q.prev(at)].frame == w.frame));
        frameNorthMs_ = overFrame_ ? pose.northMs : 0.0, frameEastMs_ = overFrame_ ? pose.eastMs : 0.0;
    }
    const bool wasEnded = ended_;
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
    altitudeMsl_ = altitudeMslOf(altitude, altitudeReferenceOf(segment.altitudeReference), s, ctx.altimeter);
    // off the route; past its end, as it was off the last leg there (the progress of what it completed, not
    // of the orbit round its point: the host may read it a few control updates later, when the world step ends)
    if (!ended_) crossTrack_ = fix.crossTrackM;
    else if (!wasEnded) crossTrack_ = lastCross_;
    if (hovers_ && ended_ && p.end == EndBehavior::Loiter) { // stopped: hover over the last point
        const Waypoint& last = p.points[p.last()];
        course_ = heading_ = kHold;
        if (overFrame_) // (a point a moving frame carries: over it, closing on it as the hover pattern does - 4.25)
            return route::hoverOver(s, perf, last.latitudeRad, last.longitudeRad, route::plannedSpeed(last.speed, last.speedReference, s.altitudeMslM),
                                    frameNorthMs_, frameEastMs_, route::verticalSpeedTo(altitudeMsl_, 0.0, s, perf, true));
        return PositionCommand{last.latitudeRad, last.longitudeRad, altitudeMsl_, kHold, 1.0, kHold};
    }
    route::Steer steer;
    steer.speed = segment.speed;
    steer.reference = speedReferenceOf(segment.speedReference, hovers_ ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
    steer.verticalSpeedMs = route::verticalSpeedTo(altitudeMsl_, feedforward, s, perf, hovers_);
    // what comes next: the turn at the point flown to - and, for a rotorcraft,
    // no faster than the turn's radius allows (and than stops it in time for
    // one), or than stops it at the end
    route::Ahead ahead;
    if (const route::Turn& turn = p.turn(target_, firstLap_); !ended_) {
        const double toGo = std::max(p.leg(target_, firstLap_).lengthM - std::max(inPieceM_ + leadOut_, 0.0) - turn.leadM, 0.0);
        if (turn.radiusM > 0.0) {
            ahead.turnRadiusM = turn.radiusM;
            if (onArc_) {
                ahead.toChangeM = turn.radiusM * std::abs(turn.angleRad) - inPieceM_; // then its leg out, straight
                steer.speedLimitMs = route::lateralLimit(perf, turn.radiusM);
            } else {
                ahead.toChangeM = p.leg(target_, firstLap_).lengthM - turn.leadM - (inPieceM_ + leadOut_);
                ahead.curvature = (turn.angleRad >= 0.0 ? 1.0 : -1.0) / turn.radiusM;
                steer.speedLimitMs = route::brakingLimit(perf, route::lateralLimit(perf, turn.radiusM), toGo);
            }
        } else if (stops()) {
            steer.speedLimitMs = std::max(route::brakingLimit(perf, 0.0, toGo), 0.5);
        }
    }
    if (overFrame_) { // over its frame, as a pattern's (4.25): its velocity and the wind's over it, a rotorcraft's given back
        sim::VehicleState over = s;
        over.velocityNedMs[0] -= frameNorthMs_, over.velocityNedMs[1] -= frameEastMs_;
        WindEstimate wind = wind_;
        wind.northMs -= frameNorthMs_, wind.eastMs -= frameEastMs_;
        VelocityCommand out = route::follow(ctx, over, perf, wind, hovers_, fix, ahead, steer, plan_->trims, course_, heading_);
        if (hovers_) out.northMs += frameNorthMs_, out.eastMs += frameEastMs_;
        return out;
    }
    return route::follow(ctx, perf, wind_, hovers_, fix, ahead, steer, plan_->trims, course_, heading_);
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

// --- PatternBehavior ----------------------------------------------------------------

namespace {

bool same(const PatternCommand& a, const PatternCommand& b) noexcept {
    const double x[] = {a.pattern, a.latitudeRad, a.longitudeRad, a.altitudeM, a.altitudeReference, a.radiusM,
                        a.clockwise, a.courseRad, a.legM, a.speed, a.speedReference, a.durationS, a.speedOptimization};
    const double y[] = {b.pattern, b.latitudeRad, b.longitudeRad, b.altitudeM, b.altitudeReference, b.radiusM,
                        b.clockwise, b.courseRad, b.legM, b.speed, b.speedReference, b.durationS, b.speedOptimization};
    for (std::size_t i = 0; i < std::size(x); ++i)
        if (!(x[i] == y[i] || (isHold(x[i]) && isHold(y[i])))) return false;
    return true;
}

/// The world's date as a decimal year, for the magnetic model (a stack on its own: the model's epoch).
double worldYear(const ControlContext& ctx, const sim::VehicleState& s) noexcept {
    return magneticYear(ctx.world ? ctx.world->environment().epochUtcSeconds + s.simTime : 0.0);
}

} // namespace

PatternBehavior::PatternBehavior() : pattern_(std::make_unique<route::Pattern>()) {}
PatternBehavior::~PatternBehavior() = default;

void PatternBehavior::begin(const ControlContext& ctx, const Command& command) {
    reset();
    startS_ = -1.0;
    finished_ = false;
    failure_ = Reason::None;
    wind_.update(ctx.sensed, ctx.dt);
    if (const auto* c = std::get_if<PatternCommand>(&command)) plan(ctx, *c);
}

void PatternBehavior::reset() {
    wind_.reset();
    pattern_->trims = route::Trims{};
    lastTime_ = -1.0;
    planned_ = false;
}

void PatternBehavior::plan(const ControlContext& ctx, const PatternCommand& c) {
    const auto& s = ctx.sensed;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    hovers_ = (ctx.features & kFeatureHover) != 0;
    flown_ = resolved_ = c;
    if (!isHold(c.speedOptimization)) { // planned at the optimum where it flies it, as the host resolves it
        const double h = isHold(c.altitudeM) ? s.altitudeMslM : altitudeMslOf(c.altitudeM, altitudeReferenceOf(c.altitudeReference), s, ctx.altimeter);
        const double best = optimalTasMs(ctx.tables, c.speedOptimization, h, s.fuelKg);
        if (std::isfinite(best)) resolved_.speed = best, resolved_.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
    }
    // its shape (docs/flight-autonomy.md, 4.23), the path store's; what it leaves out, as the host fills it in (it has, for a
    // World's vehicle)
    shape_ = ctx.path ? ctx.path->pattern : PatternShape{};
    pathRevision_ = ctx.path ? ctx.path->revision : 0;
    if (!isHold(shape_.frame) && ctx.path) frame_ = ctx.path->patternFrame;
    frameMoves_ = !isHold(shape_.frame) && frame_.origin != FrameOrigin::Fixed;
    frameAltitude_ = kHold;
    frameNorthMs_ = frameEastMs_ = frameDownMs_ = 0.0;
    const double year = shape_.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? worldYear(ctx, s) : 2025.0;
    route::completePattern(resolved_, shape_, s, perf, hovers_, wind_.northMs, wind_.eastMs, ctx.altimeter, year);
    route::planPattern(*pattern_, resolved_, s.latitudeRad, s.longitudeRad, shape_, year, route::trackOf(s));
    planned_ = true;
    laps_ = 0;
    lapDoneM_ = inPieceM_ = 0.0;
    leaving_ = false;
    entryPiece_ = 0;
    entering_ = pattern_->entryCount > 0;
    if (!entering_) {
        startPiece(pattern_->first, s);
    } else {
        piece_ = 0, swept_ = 0.0;
        if (pattern_->entry[0].arc) startArc(pattern_->entry[0].turn, s);
    }
}

void PatternBehavior::startPiece(std::uint32_t i, const sim::VehicleState& s) {
    const route::Pattern& p = *pattern_;
    piece_ = i;
    swept_ = 0.0;
    if (!p.pieces[i].arc) return;
    startArc(p.pieces[i].turn, s);
}

void PatternBehavior::startArc(const route::Turn& t, const sim::VehicleState& s) {
    const route::Pattern& p = *pattern_;
    double north, east;
    geo::localNorthEastM(p.lat0, p.lon0, s.latitudeRad, s.longitudeRad, north, east);
    lastBearing_ = std::atan2(east - t.centreEastM, north - t.centreNorthM);
    const double way = t.angleRad >= 0.0 ? 1.0 : -1.0;
    swept_ = way * geo::wrapPi(lastBearing_ - t.entryBearingRad); // (a little before its start: negative)
}

bool PatternBehavior::placeInFrame(const ControlContext& ctx, const sim::VehicleState& s) {
    // as the step began, as a vehicle followed is read (the world's time: a vehicle's own clock starts at its creation)
    FramePose pose;
    if (frame_.origin == FrameOrigin::Vehicle) {
        const sim::VehicleState* v = ctx.world ? ctx.world->vehicleState(frame_.vehicle) : nullptr;
        if (!v) return false;
        pose = vehiclePose(*v);
    } else {
        pose = framePose(frame_, ctx.world ? ctx.world->simTime() : s.simTime);
    }
    const GeoPoint at = framePoint(pose, shape_.frameOffset());
    pattern_->lat0 = at.latitudeRad, pattern_->lon0 = at.longitudeRad; // (the pattern's pieces are laid out from it: all move with it)
    if (!isHold(shape_.frameZM)) frameAltitude_ = at.altitudeMslM;
    frameNorthMs_ = pose.northMs, frameEastMs_ = pose.eastMs, frameDownMs_ = pose.downMs;
    return true;
}

VelocityCommand PatternBehavior::followInFrame(const ControlContext& ctx, const Performance& perf, const route::Fix& fix, const route::Ahead& ahead,
                                               const route::Steer& steer) {
    // A frame moving steadily is ground like any other: the aircraft flies over it at its velocity less the frame's,
    // through air moving over it at the wind less the frame's. (A frame turned with its origin swings the point as
    // the origin turns: that the path's feedback follows.)
    sim::VehicleState over = ctx.sensed;
    over.velocityNedMs[0] -= frameNorthMs_, over.velocityNedMs[1] -= frameEastMs_;
    WindEstimate wind = wind_;
    wind.northMs -= frameNorthMs_, wind.eastMs -= frameEastMs_;
    VelocityCommand out = route::follow(ctx, over, perf, wind, hovers_, fix, ahead, steer, pattern_->trims, course_, heading_);
    if (hovers_) out.northMs += frameNorthMs_, out.eastMs += frameEastMs_; // (a rotorcraft's velocity is over the ground)
    return out;
}

VelocityCommand PatternBehavior::hoverInFrame(const ControlContext& ctx, const Performance& perf, double transit) const noexcept {
    const auto& s = ctx.sensed;
    const double climb = isHold(frameAltitude_) ? 0.0 : -frameDownMs_; // (its frame's height, where it has a z)
    return route::hoverOver(s, perf, pattern_->lat0, pattern_->lon0, transit, frameNorthMs_, frameEastMs_,
                            route::verticalSpeedTo(altitudeMsl_, climb, s, perf, true));
}

bool PatternBehavior::due(double now) const noexcept {
    return (!isHold(resolved_.durationS) && startS_ >= 0.0 && now - startS_ >= resolved_.durationS) ||
           (!isHold(shape_.orbits) && !entering_ && laps_ >= shape_.orbits);
}

route::Fix PatternBehavior::locate(const sim::VehicleState& s) {
    const route::Pattern& p = *pattern_;
    const double lat = s.latitudeRad, lon = s.longitudeRad;
    if (leaving_) return route::onLine(p.away, p.lat0, p.lon0, lat, lon);
    for (int passed = 0;; ++passed) {
        const bool more = passed < 4; // (at most so many pieces a control period: the rest at the next)
        const route::Pattern::Piece& piece = entering_ ? p.entry[entryPiece_] : p.pieces[piece_];
        route::Fix f;
        if (piece.arc) {
            f = route::onArc(piece.turn, p.lat0, p.lon0, lat, lon);
            double north, east;
            geo::localNorthEastM(p.lat0, p.lon0, lat, lon, north, east);
            const double bearing = std::atan2(east - piece.turn.centreEastM, north - piece.turn.centreNorthM);
            swept_ += (piece.turn.angleRad >= 0.0 ? 1.0 : -1.0) * geo::wrapPi(bearing - lastBearing_);
            lastBearing_ = bearing;
            f.alongM = inPieceM_ = swept_ * piece.turn.radiusM;
            if (swept_ < piece.sweepRad || !more) return f;
        } else {
            f = route::onLine(piece.line, p.lat0, p.lon0, lat, lon);
            if (!entering_) inPieceM_ = f.alongM;
            if (f.alongM < piece.line.lengthM || !more) return f;
        }
        if (entering_) { // on to the next piece of the way in, or into the lap
            if (entryPiece_ + 1 < p.entryCount) {
                ++entryPiece_;
                swept_ = 0.0;
                if (p.entry[entryPiece_].arc) startArc(p.entry[entryPiece_].turn, s);
                continue;
            }
            entering_ = false;
            startPiece(p.first, s);
            continue;
        }
        // on to the next piece; a lap flown when the loop begins again
        const std::uint32_t next = p.next(piece_);
        lapDoneM_ += p.pieceM(piece_);
        if (next == p.first) ++laps_, lapDoneM_ = 0.0;
        if (p.exit >= 0 && next == static_cast<std::uint32_t>(p.exit) && due(s.simTime)) { // its exit point: out along its course
            leaving_ = finished_ = true;
            return route::onLine(p.away, p.lat0, p.lon0, lat, lon);
        }
        startPiece(next, s);
    }
}

Command PatternBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    if (resumed(ctx, lastTime_)) pattern_->trims = route::Trims{}, wind_.reset();
    wind_.update(s, ctx.dt);
    groundSpeed_ = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    simTime_ = s.simTime;
    const auto* c = std::get_if<PatternCommand>(&in);
    if (!c) { // nothing to fly: on as it flies (a rotorcraft still)
        VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
        if (hovers_) hold.northMs = hold.eastMs = 0.0;
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    if (!planned_ || !same(*c, flown_) || (ctx.path && ctx.path->revision != pathRevision_)) { // (an UPDATE: the pattern it makes, afresh)
        if (planned_ && c->pattern == static_cast<double>(PatternKind::Hover) && flown_.pattern != c->pattern) startS_ = -1.0; // (a hover's from its arrival)
        plan(ctx, *c);
    }
    if (!isHold(shape_.frame) && !placeInFrame(ctx, s)) { // (its frame's vehicle gone: nothing to fly round)
        failure_ = Reason::TargetLost;
        VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
        if (hovers_) hold.northMs = hold.eastMs = 0.0;
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    const route::Pattern& p = *pattern_;
    if (p.kind == PatternKind::Hover) { // over its point, by the position loop; its duration from its arrival there (4.25)
        altitudeMsl_ = isHold(frameAltitude_) ? altitudeMslOf(resolved_.altitudeM, altitudeReferenceOf(resolved_.altitudeReference), s, ctx.altimeter)
                                              : frameAltitude_;
        const double off = geo::distanceM(s.latitudeRad, s.longitudeRad, p.lat0, p.lon0);
        if (startS_ < 0.0 && off < 1.0 && std::abs(s.altitudeMslM - altitudeMsl_) < 2.0) startS_ = s.simTime;
        if (due(s.simTime)) finished_ = true; // (and hovers on)
        // (its speed there: the position loop's most, a ground speed - an airspeed's true airspeed, calm)
        const SpeedReference reference = speedReferenceOf(resolved_.speedReference, SpeedReference::GroundSpeed);
        const double transit = reference == SpeedReference::GroundSpeed ? resolved_.speed : trueAirspeedOf(resolved_.speed, reference, s);
        crossTrack_ = off, course_ = heading_ = kHold, speedFlown_ = transit;
        if (frameMoves_) return hoverInFrame(ctx, perf, transit);
        return PositionCommand{p.lat0, p.lon0, altitudeMsl_, transit, 1.0, kHold};
    }
    if (startS_ < 0.0) startS_ = s.simTime;
    const route::Fix fix = locate(s);
    if (p.exit < 0 && due(s.simTime)) finished_ = true; // (and flies on)
    crossTrack_ = fix.crossTrackM;
    altitudeMsl_ = isHold(frameAltitude_) ? altitudeMslOf(resolved_.altitudeM, altitudeReferenceOf(resolved_.altitudeReference), s, ctx.altimeter)
                                          : frameAltitude_; // (its frame's, given a z)
    route::Steer steer;
    steer.speed = resolved_.speed;
    steer.reference = speedReferenceOf(resolved_.speedReference, hovers_ ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
    if (!isHold(resolved_.speedOptimization)) { // the optimum at the altitude and weight now
        const double best = optimalTasMs(ctx.tables, resolved_.speedOptimization, s.altitudeMslM, s.fuelKg);
        if (std::isfinite(best)) steer.speed = best, steer.reference = SpeedReference::TrueAirspeed;
    }
    speedFlown_ = steer.speed;
    steer.verticalSpeedMs = route::verticalSpeedTo(altitudeMsl_, frameMoves_ && !isHold(frameAltitude_) ? -frameDownMs_ : 0.0, s, perf, hovers_);
    // what comes next: the pattern's next piece, a turn of its radius - and, for a
    // rotorcraft, no faster than the radius allows (and than slows it in time for one)
    route::Ahead ahead;
    ahead.turnRadiusM = p.radiusM;
    auto curvatureOf = [&](const route::Pattern::Piece& piece) {
        return piece.arc ? (piece.turn.angleRad >= 0.0 ? 1.0 : -1.0) / piece.turn.radiusM : 0.0;
    };
    const double turning = route::lateralLimit(perf, p.radiusM);
    if (leaving_) { // straight on
    } else if (entering_) {
        const route::Pattern::Piece& piece = p.entry[entryPiece_];
        const route::Pattern::Piece& next = entryPiece_ + 1 < p.entryCount ? p.entry[entryPiece_ + 1] : p.pieces[p.first];
        ahead.toChangeM = piece.arc ? piece.lengthM() - inPieceM_ : piece.line.lengthM - fix.alongM;
        ahead.curvature = curvatureOf(next);
        steer.speedLimitMs = piece.arc ? turning : route::brakingLimit(perf, turning, ahead.toChangeM);
    } else {
        const route::Pattern::Piece& piece = p.pieces[piece_];
        const std::uint32_t n = p.next(piece_);
        const bool leaves = p.exit >= 0 && n == static_cast<std::uint32_t>(p.exit) && due(s.simTime); // (then the line out)
        const double nextCurvature = leaves ? 0.0 : curvatureOf(p.pieces[n]);
        if (piece.arc) {
            if (n != piece_ || leaves) ahead.toChangeM = piece.turn.radiusM * piece.sweepRad - inPieceM_, ahead.curvature = nextCurvature;
            steer.speedLimitMs = turning;
        } else {
            ahead.toChangeM = piece.line.lengthM - inPieceM_;
            ahead.curvature = nextCurvature;
            steer.speedLimitMs = route::brakingLimit(perf, turning, ahead.toChangeM);
        }
    }
    if (frameMoves_) return followInFrame(ctx, perf, fix, ahead, steer);
    return route::follow(ctx, perf, wind_, hovers_, fix, ahead, steer, pattern_->trims, course_, heading_);
}

bool PatternBehavior::progress(ActivityProgress& out) const noexcept {
    if (!planned_) return false;
    const route::Pattern& p = *pattern_;
    out.segment = leaving_ ? p.count : entering_ ? 0 : piece_;
    out.segments = p.count;
    out.laps = laps_;
    const bool timed = !isHold(resolved_.durationS), counted = !isHold(shape_.orbits);
    if (p.kind == PatternKind::Hover) { // on its way there; then over it, its duration from its arrival
        out.segment = 0;
        out.segmentPercent = out.percent = startS_ >= 0.0 ? 100.0 : 0.0;
        if (startS_ < 0.0 && timed) out.timeToGoS = crossTrack_ / std::fmax(speedFlown_, 1.0) + resolved_.durationS;
    } else if (!entering_) {
        const double piece = p.pieceM(piece_), lap = p.lapM();
        out.segmentPercent = piece > 1e-6 ? std::clamp(100.0 * inPieceM_ / piece, 0.0, 100.0) : 100.0;
        if (!timed && !counted) out.percent = lap > 1e-6 ? std::clamp(100.0 * (lapDoneM_ + inPieceM_) / lap, 0.0, 100.0) : 100.0;
        if (!timed && counted) { // through its laps
            const double left = (shape_.orbits - laps_) * lap - (lapDoneM_ + inPieceM_);
            out.percent = shape_.orbits > 0.0 && lap > 1e-6 ? std::clamp(100.0 * (1.0 - left / (shape_.orbits * lap)), 0.0, 100.0) : 100.0;
            out.timeToGoS = std::max(left, 0.0) / std::max(groundSpeed_, 1.0);
        }
    } else if (!timed) {
        out.percent = out.segmentPercent = 0.0;
    }
    if (timed && startS_ >= 0.0) { // timed: through its duration
        const double elapsed = simTime_ - startS_;
        out.percent = std::clamp(100.0 * elapsed / resolved_.durationS, 0.0, 100.0);
        out.timeToGoS = std::max(resolved_.durationS - elapsed, 0.0);
    }
    if (leaving_) out.percent = out.segmentPercent = 100.0, out.timeToGoS = 0.0;
    out.crossTrackM = crossTrack_;
    out.courseRad = course_;
    out.headingRad = heading_;
    out.altitudeMslM = altitudeMsl_;
    const bool optimised = !isHold(resolved_.speedOptimization); // (the optimum it flies now, a true airspeed)
    out.speedMs = optimised && !isHold(speedFlown_) ? speedFlown_ : resolved_.speed;
    out.speedReference = optimised ? static_cast<double>(SpeedReference::TrueAirspeed) : resolved_.speedReference;
    return true;
}

// --- CurveBehavior ------------------------------------------------------------------

CurveBehavior::CurveBehavior() : curve_(std::make_unique<route::Curve>()) {}
CurveBehavior::~CurveBehavior() = default;

void CurveBehavior::begin(const ControlContext& ctx, const Command& command) {
    reset();
    startS_ = -1.0;
    wind_.update(ctx.sensed, ctx.dt);
    if (const auto* c = std::get_if<CurveCommand>(&command)) restart(ctx, *c);
}

void CurveBehavior::reset() {
    wind_.reset();
    curve_->trims = route::Trims{};
    lastTime_ = -1.0;
    planned_ = false;
}

void CurveBehavior::restart(const ControlContext& ctx, const CurveCommand& c) {
    route::Curve& k = *curve_;
    const auto& s = ctx.sensed;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    hovers_ = (ctx.features & kFeatureHover) != 0;
    flown_ = c;
    planned_ = true;
    ended_ = finished_ = false;
    failure_ = Reason::None;
    segment_ = 0, t_ = 0.0;
    const PathStore* store = ctx.path;
    if (!store || store->segmentCount == 0) {
        k.count = 0;
        failure_ = Reason::BehaviorFailed; // (only a stack on its own is given a curve nobody checked)
        return;
    }
    generation_ = store->curve, stored_ = store->segmentCount;
    k.lat0 = orHold(c.latitudeRad, s.latitudeRad), k.lon0 = orHold(c.longitudeRad, s.longitudeRad), k.alt0 = orHold(c.altitudeM, s.altitudeMslM);
    // its plane and its points' reading, its reference in a frame (4.27)
    k.offsets = isHold(c.pointOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(c.pointOffsets));
    k.psi = 0.0;
    rotation_ = isHold(c.pointRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(c.pointRotation));
    altitudeReference_ = isHold(c.altitudeReference) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference));
    z_ = isHold(c.pointZ) ? CurveZ::Down : static_cast<CurveZ>(static_cast<int>(c.pointZ));
    absoluteBase_ = k.alt0;
    shape_ = store->curveShape;
    if (!isHold(shape_.frame)) frame_ = store->curveFrame;
    frameMoves_ = !isHold(shape_.frame) && frame_.origin != FrameOrigin::Fixed;
    frameNorthMs_ = frameEastMs_ = frameDownMs_ = 0.0;
    turnedRoll_ = turnedPitch_ = turnedYaw_ = kHold; // (in three dimensions: turned where its frame is placed)
    k.count = std::min<std::uint32_t>(stored_, route::Curve::kMax);
    std::copy_n(store->segments, k.count, k.segments);
    k.normalize(0, z_);
    k.measure(0);
    const double ground = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    if (hovers_) ownSpeed_ = ground < 1.0 ? (std::isfinite(perf.cruiseTasMs) ? perf.cruiseTasMs : 5.0) : ground;
    else ownSpeed_ = std::max(s.airspeedTrueMs, 1.0);
}

void CurveBehavior::pace(const Performance& perf, const route::Fix& fix) noexcept {
    const route::Curve& k = *curve_;
    auto within = [this](double v) {
        if (!isHold(flown_.speedMinMs)) v = std::max(v, flown_.speedMinMs);
        if (!isHold(flown_.speedMaxMs)) v = std::min(v, flown_.speedMaxMs);
        return v;
    };
    const SpeedReference was = reference_;
    reference_ = SpeedReference::GroundSpeed;
    if (!isHold(flown_.durationS) && startS_ >= 0.0) {
        // the rest of it in the rest of the time; its last second (or once late) at the pace it had,
        // not slowing with what is left to nothing short of its end
        const double left = flown_.durationS - (simTime_ - startS_);
        if (left > 1.0 || isHold(speed_) || was != SpeedReference::GroundSpeed) speed_ = within(std::max(k.lengthM() - k.fromM, 0.0) / std::max(left, 1.0));
    } else if (hovers_) {
        speed_ = within(ownSpeed_);
    } else {
        // a wing holds its airspeed; the ground speed that makes along the curve in the wind, kept within the range
        const double tn = std::cos(fix.courseRad), te = std::sin(fix.courseRad);
        const double along = wind_.northMs * tn + wind_.eastMs * te, across = -wind_.northMs * te + wind_.eastMs * tn;
        auto air = [along, across](double ground) { return std::hypot(ground - along, across); };
        speed_ = ownSpeed_;
        if (!isHold(flown_.speedMaxMs)) speed_ = std::min(speed_, air(flown_.speedMaxMs));
        if (!isHold(flown_.speedMinMs)) speed_ = std::max(speed_, air(flown_.speedMinMs));
        reference_ = SpeedReference::TrueAirspeed;
    }
    if (hovers_ && std::isfinite(perf.maxGroundSpeedMs)) speed_ = std::min(speed_, perf.maxGroundSpeedMs);
    speed_ = std::max(speed_, 0.1);
}

bool CurveBehavior::placeInFrame(const ControlContext& ctx, const sim::VehicleState& s) {
    FramePose pose;
    if (frame_.origin == FrameOrigin::Vehicle) {
        const sim::VehicleState* v = ctx.world ? ctx.world->vehicleState(frame_.vehicle) : nullptr;
        if (!v) return false;
        pose = vehiclePose(*v);
    } else {
        pose = framePose(frame_, ctx.world ? ctx.world->simTime() : s.simTime);
    }
    const GeoPoint at = framePoint(pose, shape_.frameOffset());
    route::Curve& k = *curve_;
    k.lat0 = at.latitudeRad, k.lon0 = at.longitudeRad; // (its plane is laid out from it: all of it moves with it)
    if (!isHold(shape_.frameZM)) k.alt0 = at.altitudeMslM;
    k.psi = route::frameTurn(pose, rotation_);
    frameNorthMs_ = pose.northMs, frameEastMs_ = pose.eastMs, frameDownMs_ = pose.downMs;
    if (rotation_ == FrameRotation::Attitude && (pose.rollRad != turnedRoll_ || pose.pitchRad != turnedPitch_ || pose.yawRad != turnedYaw_))
        orient(ctx, pose); // (a vehicle's as it turns: every step)
    return true;
}

void CurveBehavior::orient(const ControlContext& ctx, const FramePose& pose) {
    route::Curve& k = *curve_;
    if (!ctx.path) return;
    // its points as given, made down from its reference as its first were, turned as the frame is now
    k.count = std::min<std::uint32_t>(stored_, route::Curve::kMax);
    std::copy_n(ctx.path->segments, k.count, k.segments);
    const double alt0 = k.alt0;
    k.alt0 = absoluteBase_;
    k.normalize(0, z_);
    k.alt0 = alt0;
    k.orient(0, route::Attitude(pose), z_);
    k.measure(0);
    turnedRoll_ = pose.rollRad, turnedPitch_ = pose.pitchRad, turnedYaw_ = pose.yawRad;
}

double CurveBehavior::altitudeOf(const ControlContext& ctx, const sim::VehicleState& s, double down) const noexcept {
    const double at = (z_ == CurveZ::AbsoluteAltitude ? absoluteBase_ : curve_->alt0) - down; // (an absolute altitude stays one)
    return altitudeReference_ == AltitudeReference::Msl ? at : altitudeMslOf(at, altitudeReference_, s, ctx.altimeter);
}

void CurveBehavior::end(const Performance& perf) {
    route::Curve& k = *curve_;
    ended_ = finished_ = true;
    const route::CurvePoint p = k.point(k.count - 1, 1.0);
    endNorth_ = p.p[0], endEast_ = p.p[1];
    if (!k.plain() || !isHold(shape_.frame)) { // (laid out from its end, on the Earth: 4.27)
        k.exit = route::Line{0.0, 0.0, geo::wrapPi(p.courseRad() + k.psi), 0.0};
        k.orbit = route::Turn{};
    } else {
        k.exit = route::Line{p.p[0], p.p[1], p.courseRad(), 0.0};
        k.orbit = route::Turn{};
        k.orbit.centreNorthM = p.p[0], k.orbit.centreEastM = p.p[1];
    }
    // (a wing's airspeed with the wind behind it; a rotorcraft's ground speed, the radius its velocity loop follows)
    k.orbit.radiusM = perf.turnRadiusM(hovers_ ? speed_ : speed_ + std::hypot(wind_.northMs, wind_.eastMs));
    k.orbit.angleRad = 1.0; // right turns, round its end (A-GRA's CIRCULAR_LOITER: 4.28)
}

Command CurveBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    if (resumed(ctx, lastTime_)) curve_->trims = route::Trims{}, wind_.reset();
    wind_.update(s, ctx.dt);
    groundSpeed_ = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    simTime_ = s.simTime;
    route::Curve& k = *curve_;
    const auto* c = std::get_if<CurveCommand>(&in);
    if (c && (!planned_ || !ctx.path || ctx.path->curve != generation_)) {
        restart(ctx, *c); // a new curve: flown afresh
    } else if (c && ctx.path->segmentCount > stored_ && k.count < route::Curve::kMax) {
        // segments appended: on along them
        const auto from = k.count;
        stored_ = ctx.path->segmentCount;
        k.count = std::min<std::uint32_t>(stored_, route::Curve::kMax);
        std::copy_n(ctx.path->segments + from, k.count - from, k.segments + from);
        const double alt0 = k.alt0;
        k.alt0 = absoluteBase_; // (its absolute altitudes made down from where the first were)
        k.normalize(from, z_);
        k.alt0 = alt0;
        k.measure(from);
        if (rotation_ == FrameRotation::Attitude) turnedYaw_ = kHold; // (in three dimensions: turned afresh with them, where placed)
    }
    if (c) flown_.speedMinMs = c->speedMinMs, flown_.speedMaxMs = c->speedMaxMs, flown_.durationS = c->durationS, flown_.end = c->end;
    if (!c || k.count == 0) { // nothing to fly: on as it flies (a rotorcraft still)
        VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
        if (hovers_) hold.northMs = hold.eastMs = 0.0;
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    if (!isHold(shape_.frame) && !placeInFrame(ctx, s)) { // (its frame's vehicle gone: nothing to fly along)
        failure_ = Reason::TargetLost;
        VelocityCommand hold{kHold, 0.0, s.eulerRad[2], kHold, kHold, kHold};
        if (hovers_) hold.northMs = hold.eastMs = 0.0;
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    if (startS_ < 0.0) startS_ = s.simTime;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    const bool loiter = orHold(flown_.end, 0.0) == static_cast<double>(EndBehavior::Loiter);

    route::Fix fix;
    double feedforward = 0.0;
    if (!ended_) {
        fix = route::onCurve(k, segment_, t_, s.latitudeRad, s.longitudeRad);
        k.fromM = fix.alongM;
        crossTrack_ = fix.crossTrackM; // (past its end, as it was there: what it completed, not the orbit's)
        pace(perf, fix);               // (past its end, on at the speed it had)
        // its end passed abeam (loitering, each circles it: 4.28)
        if (segment_ + 1 == k.count && t_ >= 1.0 - 1e-9) end(perf);
    }
    const route::CurvePoint here = k.point(ended_ ? k.count - 1 : segment_, ended_ ? 1.0 : t_);
    altitudeMsl_ = altitudeReference_ == AltitudeReference::Msl && z_ != CurveZ::AbsoluteAltitude ? k.alt0 - here.p[2] : altitudeOf(ctx, s, here.p[2]);
    if (!ended_) feedforward = here.gradient() * groundSpeed_;
    if (frameMoves_ && !isHold(shape_.frameZM) && z_ != CurveZ::AbsoluteAltitude) feedforward -= frameDownMs_; // (its frame's climb)
    if (ended_) {
        if (k.plain() && isHold(shape_.frame)) {
            fix = loiter ? route::onArc(k.orbit, k.lat0, k.lon0, s.latitudeRad, s.longitudeRad)
                         : route::onLine(k.exit, k.lat0, k.lon0, s.latitudeRad, s.longitudeRad);
        } else { // (from its end on the Earth, as its plane lays it out now: 4.27)
            double lat, lon;
            k.fromPlane(endNorth_, endEast_, lat, lon);
            fix = loiter ? route::onArc(k.orbit, lat, lon, s.latitudeRad, s.longitudeRad) : route::onLine(k.exit, lat, lon, s.latitudeRad, s.longitudeRad);
        }
    }
    route::Steer steer;
    steer.speed = speed_;
    steer.reference = reference_;
    steer.verticalSpeedMs = route::verticalSpeedTo(altitudeMsl_, feedforward, s, perf, hovers_);
    route::Ahead ahead;
    if (!ended_) {
        // the curve ahead: its curvature where a lagging loop should fly it now, and the
        // tightest a few seconds on for a wing's roll into it; a rotorcraft no faster than
        // the sections ahead allow, slowing in time
        ahead.curvatureAt = route::curvatureAhead;
        ahead.path = &k;
        double tightest = std::abs(fix.curvature);
        for (const double seconds : {1.0, 2.0, 4.0}) tightest = std::max(tightest, std::abs(route::curvatureAhead(&k, seconds * std::max(groundSpeed_, 1.0))));
        if (tightest > 1e-6) ahead.turnRadiusM = 1.0 / tightest;
        if (hovers_) steer.speedLimitMs = route::speedLimitAhead(perf, k, std::max(speed_, groundSpeed_), false);
    }
    if (frameMoves_) { // flown over its frame, as a pattern is (4.25): its velocity and the wind's over it, a rotorcraft's given back
        sim::VehicleState over = s;
        over.velocityNedMs[0] -= frameNorthMs_, over.velocityNedMs[1] -= frameEastMs_;
        WindEstimate wind = wind_;
        wind.northMs -= frameNorthMs_, wind.eastMs -= frameEastMs_;
        VelocityCommand out = route::follow(ctx, over, perf, wind, hovers_, fix, ahead, steer, k.trims, course_, heading_);
        if (hovers_) out.northMs += frameNorthMs_, out.eastMs += frameEastMs_;
        return out;
    }
    return route::follow(ctx, perf, wind_, hovers_, fix, ahead, steer, k.trims, course_, heading_);
}

bool CurveBehavior::progress(ActivityProgress& out) const noexcept {
    const route::Curve& k = *curve_;
    if (!planned_ || k.count == 0) return false;
    out.segment = std::min(segment_, k.count - 1);
    out.segments = k.count;
    const double length = k.lengthM(), along = ended_ ? length : k.fromM;
    out.percent = length > 1e-6 ? std::clamp(100.0 * along / length, 0.0, 100.0) : 100.0;
    out.segmentPercent = ended_ ? 100.0 : std::clamp(100.0 * t_, 0.0, 100.0);
    out.distanceToGoM = std::max(length - along, 0.0);
    if (ended_) out.timeToGoS = 0.0;
    else if (!isHold(flown_.durationS) && startS_ >= 0.0) out.timeToGoS = std::max(flown_.durationS - (simTime_ - startS_), 0.0);
    else if (groundSpeed_ > 0.1) out.timeToGoS = out.distanceToGoM / groundSpeed_;
    out.crossTrackM = crossTrack_;
    out.courseRad = course_;
    out.headingRad = heading_;
    out.altitudeMslM = altitudeMsl_;
    out.speedMs = speed_;
    out.speedReference = static_cast<double>(reference_);
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
                      p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                      p("speed_optimization", "", now, 0.0, static_cast<double>(SpeedOptimization::Count) - 1.0),
                      p("direction_reference", "", now, 0.0, static_cast<double>(DirectionReference::Count) - 1.0)};
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
    BehaviorTraits pattern;
    pattern.persistence = Persistence::Persistent; // (with a duration it completes, and flies on)
    pattern.parameters = {p("pattern", "", now, 0.0, static_cast<double>(PatternKind::Count) - 1.0),
                          p("latitude_rad", "rad", now, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846),
                          p("longitude_rad", "rad", now, -inf, inf),
                          p("altitude_m", "m", now, -inf, inf, Constraint::MinAltitude, Constraint::MaxAltitude),
                          p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                          p("radius_m", "m", now, 0.0, inf, Constraint::MaxOrientation, Constraint::None),
                          p("clockwise", "", now, 0.0, 1.0),
                          p("course_rad", "rad", now, -inf, inf),
                          p("leg_m", "m", now, 0.0, inf),
                          p("speed", "m/s or Mach", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                          p("speed_reference", "", now, 0.0, static_cast<double>(SpeedReference::Count) - 1.0),
                          p("duration_s", "s", now, 0.0, inf),
                          p("speed_optimization", "", now, 0.0, static_cast<double>(SpeedOptimization::Count) - 1.0),
                          p("direction_reference", "", now, 0.0, static_cast<double>(DirectionReference::Count) - 1.0),
                          p("heading_rad", "rad", now, -inf, inf),
                          p("leg_s", "s", now, 0.0, inf),
                          p("bank_rad", "rad", now, 0.0, 0.5 * 3.14159265358979323846, Constraint::None, Constraint::MaxOrientation),
                          p("orbits", "", now, 0.0, inf),
                          p("latitude2_rad", "rad", now, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846),
                          p("longitude2_rad", "rad", now, -inf, inf),
                          p("radius2_m", "m", now, 0.0, inf, Constraint::MaxOrientation, Constraint::None),
                          p("entry_latitude_rad", "rad", now, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846),
                          p("entry_longitude_rad", "rad", now, -inf, inf),
                          p("exit_latitude_rad", "rad", now, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846),
                          p("exit_longitude_rad", "rad", now, -inf, inf),
                          p("turn_rate_rad_s", "rad/s", now, 0.0, inf, Constraint::None, Constraint::MaxOrientation),
                          p("turn_type", "", now, 0.0, static_cast<double>(HoldTurn::Count) - 1.0),
                          p("hold_entry", "", now, 0.0, static_cast<double>(HoldEntry::Count) - 1.0),
                          p("hold_context", "", now, 0.0, static_cast<double>(HoldContext::Count) - 1.0),
                          p("frame", "", now, 1.0, 9007199254740992.0),
                          p("frame_rotation", "", now, 0.0, static_cast<double>(FrameRotation::Count) - 1.0),
                          p("frame_offsets", "", now, 0.0, static_cast<double>(FrameOffsets::Count) - 1.0),
                          p("frame_x_m", "m", now, -inf, inf),
                          p("frame_y_m", "m", now, -inf, inf),
                          p("frame_z_m", "m", now, -inf, inf)};
    pattern.uses = {"fsim.flight.velocity"};
    pattern.mode = FlightMode::Loiter;
    pattern.setpoint = SetpointKind::Pattern;
    r.addBehavior("pattern", [] { return std::make_unique<PatternBehavior>(); }, std::move(pattern));
    // the curve's options; its segments go beside them into the path store
    BehaviorTraits curve;
    curve.persistence = Persistence::Terminating;
    curve.parameters = {p("latitude_rad", "rad", now, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846),
                        p("longitude_rad", "rad", now, -inf, inf),
                        p("altitude_m", "m", now, -inf, inf),
                        p("speed_min_ms", "m/s", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                        p("speed_max_ms", "m/s", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                        p("duration_s", "s", now, 0.0, inf),
                        p("end", "", now, 0.0, static_cast<double>(EndBehavior::Count) - 1.0),
                        p("append", "", now, 0.0, 1.0),
                        p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                        p("altitude_min_m", "m", now, -inf, inf),
                        p("altitude_max_m", "m", now, -inf, inf),
                        p("point_rotation", "", now, 0.0, static_cast<double>(FrameRotation::Count) - 1.0),
                        p("point_offsets", "", now, 0.0, static_cast<double>(FrameOffsets::Count) - 1.0),
                        p("point_z", "", now, 0.0, static_cast<double>(CurveZ::Count) - 1.0),
                        p("frame", "", now, 1.0, 9007199254740992.0),
                        p("frame_rotation", "", now, 0.0, static_cast<double>(FrameRotation::Count) - 1.0),
                        p("frame_offsets", "", now, 0.0, static_cast<double>(FrameOffsets::Count) - 1.0),
                        p("frame_x_m", "m", now, -inf, inf),
                        p("frame_y_m", "m", now, -inf, inf),
                        p("frame_z_m", "m", now, -inf, inf)};
    curve.uses = {"fsim.flight.velocity", "fsim.flight.position"};
    curve.mode = FlightMode::CurveFollowing;
    curve.setpoint = SetpointKind::Curve;
    r.addBehavior("curve", [] { return std::make_unique<CurveBehavior>(); }, std::move(curve));
}

} // namespace fsim::control
