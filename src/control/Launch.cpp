// A-GRA's LAUNCH from an airfield's runway (docs/flight-autonomy.md, 4.49; ADR-29 FA-9a: LCH-01, LCH-04): the host resolves
// the runway at the NEW, and the "launch" behaviour flies the departure - a wing's line-up, takeoff roll, rotation, climb-out
// and clean-up; a rotorcraft's lift to a hover.
#include "control/CapabilityHost.h"
#include "core/Geodesy.h"
#include "fsim/ControlStack.h"
#include "fsim/BuiltinControllers.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kHalfWidthM = 22.5;     // half a 45 m runway (ICAO code letter E: A-GRA's runway gives no width)
constexpr double kBehindStartM = 60.0;   // on the runway short of its takeoff start: lined up from there
constexpr double kHeadingOffRad = 30.0 * kDeg;
constexpr double kTakeoffFlaps = 0.3;    // the flaps set for the roll and the climb-out, up at kFlapsUpAglM
constexpr double kRotationShare = 0.6;  // of its angle of attack's limit: lifting off near 1.15 times its least speed
constexpr double kRotationMinRad = 7.0 * kDeg, kRotationMaxRad = 12.0 * kDeg, kRotationRad = 10.0 * kDeg;
constexpr double kTailMarginRad = 2.0 * kDeg; // short of the attitude its tail touches at
constexpr double kRotationRateRadS = 3.0 * kDeg;
constexpr double kSlipGain = -2.0;    // rudder per rad of sideslip
constexpr double kGearUpAglM = 10.0, kFlapsUpAglM = 120.0;
// a rejected takeoff (4.50): stopped with a second's reaction and its brakes' 2.5 m/s^2 (a wet runway's; dry runways give
// twice that), 30 m short of the runway's end; rejected when it cannot reach its rotation speed and rotate (5 s) in the runway
// left at the acceleration it makes, or when it is a quarter of the runway's width off its line, for a second on end - judged
// once it makes 30 % of its rotation speed, its engines spooled up (5 s in, a heavy's projection read kilometres at 2 m/s),
// or after 40 s of roll short of it
constexpr double kReactionS = 1.0, kBrakeMs2 = 2.5, kEndMarginM = 30.0, kRotateS = 5.0, kLineLostM = 11.25;
constexpr double kJudgeFromShare = 0.3, kJudgeAfterS = 40.0, kNoGoS = 1.0;
constexpr double kStoppedMs = 0.3; // stopped: slower than this for a second

double clamp1(double x) noexcept { return std::clamp(x, -1.0, 1.0); }

/// Along and across (+ right) the line from (lat0, lon0) on `course`.
void onLine(double lat0, double lon0, double course, double lat, double lon, double& along, double& cross) noexcept {
    double north, east;
    geo::localNorthEastM(lat0, lon0, lat, lon, north, east);
    along = north * std::cos(course) + east * std::sin(course);
    cross = -north * std::sin(course) + east * std::cos(course);
}

} // namespace

Reason CapabilityHost::prepareLaunch(BehaviorCommand& b, const sim::VehicleState& state, CommandResult& detail) {
    auto at = [&detail](std::int16_t field, Reason why) {
        detail.index = field;
        return why;
    };
    // a policy's parameters are its own: what the host writes ("_...": its resolutions, FA's hand-over, 4.50) it cannot
    // give ('_' to '`', the characters' order)
    if (policyNew_) b.params.erase(b.params.lower_bound("_"), b.params.lower_bound("`"));
    // its airfield and runway, the vehicle's (4.40); the runway's takeoff line: its start, on its direction (else to its
    // limit), as long as it is (else to its limit)
    const double af = b.param("airfield", 0.0), rw = b.param("runway", 0.0);
    Airfield a;
    if (!(af >= 1.0) || af != std::floor(af) || af > 4294967295.0 || !airfield(static_cast<AirfieldId>(af), a)) return at(0, Reason::UnknownAirfield);
    const Runway* r = nullptr;
    for (const Runway& x : a.runways)
        if (static_cast<double>(x.id) == rw) r = &x;
    if (!r) return at(1, Reason::UnknownAirfield);
    const RunwayCoordinates& t = r->takeoff;
    if (isHold(t.start.latitudeRad)) return at(1, Reason::InvalidParameter); // (a runway for landing only)
    double course = r->directionRad, length = r->availableLengthM;
    if (!isHold(t.limit.latitudeRad)) {
        if (isHold(course)) course = geo::bearingRad(t.start.latitudeRad, t.start.longitudeRad, t.limit.latitudeRad, t.limit.longitudeRad);
        if (isHold(length)) length = geo::distanceM(t.start.latitudeRad, t.start.longitudeRad, t.limit.latitudeRad, t.limit.longitudeRad);
    }
    if (isHold(course) || isHold(length) || !(length > 0.0)) return at(1, Reason::InvalidParameter);
    // a wing on the runway: within half its width of the centre line, facing along it, short of its middle - where it
    // starts; FA's own rejection or continuation of a takeoff a policy canceled (4.50) goes on from wherever it is
    double along, cross;
    onLine(t.start.latitudeRad, t.start.longitudeRad, course, state.latitudeRad, state.longitudeRad, along, cross);
    if (!performance_.hovers && b.params.count("_mode") == 0 &&
        (std::abs(cross) > kHalfWidthM || along < -kBehindStartM || along > 0.5 * length ||
         std::abs(geo::wrapPi(state.eulerRad[2] - course)) > kHeadingOffRad))
        return at(1, Reason::InvalidParameter);
    // a wing's rotation and climb speeds: 1.1 and 1.3 times its stall speed with its flaps out (else clean; else the
    // envelope's least - a fly-by-wire fighter's, whose limiter keeps it from stalling)
    double stall = kHold;
    if (profile_) stall = std::isfinite(profile_->performance.stallFlapsCasMs) ? profile_->performance.stallFlapsCasMs : profile_->performance.stallCasMs;
    if (!std::isfinite(stall)) stall = performance_.minCasMs;
    if (!performance_.hovers && !std::isfinite(stall)) return at(1, Reason::NotImplemented); // (no speed to rotate at: a stock model's)
    b.params["_start_lat"] = t.start.latitudeRad, b.params["_start_lon"] = t.start.longitudeRad;
    b.params["_course"] = geo::wrapTwoPi(course), b.params["_length"] = length;
    if (!performance_.hovers) b.params["_vr"] = 1.1 * stall, b.params["_climb"] = 1.3 * stall;
    // its rotation attitude: a share of its angle of attack's limit, kept short of the attitude its tail touches at
    const double alphaMax = profile_ ? profile_->envelope.clean.alphaMaxRad : kHold;
    const double tail = profile_ ? profile_->envelope.groundPitchMaxRad : kHold;
    double rotate = std::isfinite(alphaMax) ? std::clamp(kRotationShare * alphaMax, kRotationMinRad, kRotationMaxRad) : kRotationRad;
    if (std::isfinite(tail)) rotate = std::min(rotate, tail - kTailMarginRad);
    if (!performance_.hovers) b.params["_rotate"] = rotate;
    return Reason::None;
}

bool ControlStack::handOver(std::size_t slot, BehaviorCommand& out) const {
    if (slot >= kSlotCount || !behaviors_[slot] || started_[slot] != config_->slots[slot].generation) return false;
    return behaviors_[slot]->handOver(out);
}

ActivityId CapabilityHost::handOver(const BehaviorCommand& next, const sim::VehicleState& state, double now) {
    // FA's own (4.50): the rest of a takeoff a policy canceled, flown at once on the axes the cancel freed
    CommandOptions own;
    own.source = Source::Autopilot;
    own.range = RangePolicy::Clamp;
    try {
        const CommandResult r = submitWith(Command(next), {}, {}, own, state, now, false);
        return r.accepted() ? r.activity : 0;
    } catch (...) {
        return 0; // (no memory for its parameters: the axes go, as any canceled activity's)
    }
}

void LaunchBehavior::reset() {
    phase_ = Phase::LineUp;
    failure_ = Reason::None;
    rotateStartS_ = airborneS_ = lastS_ = rollStartS_ = -1.0;
    crossIntegral_ = settledS_ = pitchIntegral_ = pitchRefRad_ = rotateFromRad_ = accelMs2_ = lastGroundSpeed_ = stoppedS_ = casMs_ = noGoS_ = 0.0;
    gearUp_ = flapsUp_ = own_ = false;
    onGround_ = true;
}

void LaunchBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    reset();
    const auto& s = ctx.sensed;
    hovers_ = (ctx.features & kFeatureHover) != 0;
    startLat_ = command.param("_start_lat", s.latitudeRad), startLon_ = command.param("_start_lon", s.longitudeRad);
    courseRad_ = command.param("_course", s.eulerRad[2]), lengthM_ = command.param("_length", 3000.0);
    vrCasMs_ = command.param("_vr", 0.0), climbCasMs_ = command.param("_climb", 0.0);
    completeAglM_ = command.param("complete_agl_m", 450.0), hoverAglM_ = command.param("hover_agl_m", 10.0);
    groundAglM_ = s.altitudeAglM;
    rotationRad_ = command.param("_rotate", kRotationRad); // (the host's, from its envelope)
    hoverMslM_ = s.altitudeMslM - s.altitudeAglM + hoverAglM_; // (the c.g. that high over the ground it stands on)
    airfield_ = command.param("airfield", 0.0), runway_ = command.param("runway", 0.0);
    parkedRad_ = command.param("_parked", s.eulerRad[1]);
    lastGroundSpeed_ = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    if (hovers_) startLat_ = s.latitudeRad, startLon_ = s.longitudeRad, phase_ = Phase::Lift; // (straight up, from where it is)
    // FA's own, handed on from a takeoff a policy canceled on the runway (4.50): its rejection, or its continuation - rolling
    // on, or rotating from where the nose is
    const double mode = command.param("_mode", 0.0);
    if (!hovers_ && mode != 0.0) {
        own_ = true;
        pitchRefRad_ = parkedRad_, rollStartS_ = s.simTime;
        if (mode == 1.0) phase_ = Phase::Reject;
        else if (s.airspeedCalibratedMs < vrCasMs_) phase_ = Phase::Roll;
        else phase_ = Phase::Rotate, rotateStartS_ = s.simTime, rotateFromRad_ = s.eulerRad[1];
    }
}

bool LaunchBehavior::canStop(double speedMs) const noexcept {
    return alongM_ + speedMs * kReactionS + speedMs * speedMs / (2.0 * kBrakeMs2) <= lengthM_ - kEndMarginM;
}

bool LaunchBehavior::handOver(BehaviorCommand& out) const {
    // a wing on the runway, not yet flying: stopped where it could still stop (below its decision speed), flown off where
    // it could not; once it flies, or stopped, its axes may go
    if (hovers_ || !onGround_ || phase_ == Phase::Stopped || phase_ == Phase::Climb || phase_ == Phase::Done) return false;
    const bool stop = phase_ == Phase::LineUp || phase_ == Phase::Reject || (phase_ == Phase::Roll && casMs_ < vrCasMs_ && canStop(lastGroundSpeed_));
    out = BehaviorCommand{};
    out.id = "launch";
    out.params = {{"airfield", airfield_}, {"runway", runway_}, {"complete_agl_m", completeAglM_}, {"hover_agl_m", hoverAglM_},
                  {"_mode", stop ? 1.0 : 2.0}, {"_parked", parkedRad_}};
    return true;
}

Command LaunchBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    onLine(startLat_, startLon_, courseRad_, s.latitudeRad, s.longitudeRad, alongM_, crossM_);
    Command out;
    switch (phase_) {
    case Phase::Lift: out = lift(ctx); break;
    case Phase::LineUp:
    case Phase::Roll:
    case Phase::Rotate: out = roll(ctx); break;
    case Phase::Climb:
    case Phase::Done: out = climb(ctx); break;
    case Phase::Reject:
    case Phase::Stopped: out = reject(ctx); break;
    }
    lastS_ = s.simTime;
    onGround_ = s.onGround, casMs_ = s.airspeedCalibratedMs;
    return out;
}

double LaunchBehavior::steer(const sim::VehicleState& s, double dt) {
    // on the wheels: the nose on a point ahead on the centre line - the offset's integral against a crosswind - the yaw
    // rate damped (the rudder steers the nose or tail wheel: + yaws left)
    if (!s.onGround) return clamp1(0.5 * s.angularRateBodyRadS[2]);
    if (phase_ != Phase::LineUp) crossIntegral_ = std::clamp(crossIntegral_ + crossM_ * dt, -200.0, 200.0);
    const double ahead = phase_ == Phase::LineUp ? 20.0 : 60.0;
    const double err = geo::wrapPi(courseRad_ - std::atan2(crossM_ + 0.1 * crossIntegral_, ahead) - s.eulerRad[2]);
    return clamp1(-2.0 * err + 0.5 * s.angularRateBodyRadS[2]);
}

Command LaunchBehavior::reject(const ControlContext& ctx) {
    // the rejected takeoff (4.50): idle, full brakes, still steered on the centre line, its wings level and its nose held
    // down; stopped - slower than kStoppedMs for a second - it holds its brakes, and its activity ends
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    const double groundSpeed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    ActuatorCommand a;
    a.throttle = 0.0, a.brakeLeft = a.brakeRight = 1.0;
    a.gearDown = 1.0, a.flaps = kTakeoffFlaps;
    a.rudder = steer(s, dt);
    a.aileron = clamp1(-2.0 * s.eulerRad[0] - 0.5 * s.angularRateBodyRadS[0]);
    // its nose held at its parked attitude: down only - a tricycle's nose wheel kept loaded to steer, its strut's give
    // allowed - unless it pitches onto its nose, more than 2 deg under (braking the U-2S's single main wheel lifted its tail
    // wheel, and it turned 37 deg off its line across the wind)
    pitchRefRad_ = parkedRad_;
    const double hold = pitchHold(s.eulerRad[1], s.angularRateBodyRadS[1], dt, true);
    if (s.eulerRad[1] < parkedRad_ - 2.0 * kDeg) {
        a.elevator = hold;
    } else {
        a.elevator = std::max(hold, 0.0);
        pitchIntegral_ = std::min(pitchIntegral_, 0.0);
    }
    stoppedS_ = groundSpeed < kStoppedMs ? stoppedS_ + dt : 0.0;
    if (phase_ == Phase::Reject && stoppedS_ >= 1.0) phase_ = Phase::Stopped;
    lastGroundSpeed_ = groundSpeed;
    return a;
}

Command LaunchBehavior::roll(const ControlContext& ctx) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    const double p = s.angularRateBodyRadS[0], q = s.angularRateBodyRadS[1];
    const double groundSpeed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    ActuatorCommand a;
    a.gearDown = 1.0, a.flaps = kTakeoffFlaps;
    a.rudder = steer(s, dt); // (on its line; the wings held level)
    a.aileron = clamp1(-2.0 * phi - 0.5 * p);
    // its acceleration along the ground, smoothed over a second: what its go or no-go judges by
    if (dt > 0.0) accelMs2_ += std::min(dt, 1.0) * ((groundSpeed - lastGroundSpeed_) / dt - accelMs2_);
    lastGroundSpeed_ = groundSpeed;
    if (phase_ == Phase::LineUp) { // onto the centre line at a walking pace, then the roll
        a.throttle = std::clamp(0.1 + 0.2 * (5.0 - groundSpeed), 0.0, 0.6);
        a.brakeLeft = a.brakeRight = groundSpeed > 7.0 ? 0.3 : 0.0;
        a.elevator = 0.0;
        if (std::abs(crossM_) < 2.0 && std::abs(geo::wrapPi(courseRad_ - psi)) < 3.0 * kDeg)
            phase_ = Phase::Roll, pitchRefRad_ = parkedRad_ = theta, rollStartS_ = s.simTime;
        return a;
    }
    a.throttle = 1.0, a.brakeLeft = a.brakeRight = 0.0;
    // go or no-go (4.50): below its rotation speed, where it can still stop, it rejects its takeoff when it can no longer
    // reach that speed and rotate in the runway left at the acceleration it makes (an engine lost; a runway too short), or
    // when it has lost its line
    if (phase_ == Phase::Roll && s.onGround && canStop(groundSpeed) &&
        (s.airspeedCalibratedMs >= kJudgeFromShare * vrCasMs_ || s.simTime - rollStartS_ >= kJudgeAfterS)) {
        const double vr = std::max(vrCasMs_, s.airspeedCalibratedMs), gain = groundSpeed / std::max(s.airspeedCalibratedMs, 1.0);
        const double toVr = accelMs2_ > 0.05 ? (std::pow(vr * gain, 2) - groundSpeed * groundSpeed) / (2.0 * accelMs2_) : 1e9;
        const bool noGo = alongM_ + toVr + kRotateS * vr * gain > lengthM_ - kEndMarginM || std::abs(crossM_) > kLineLostM;
        noGoS_ = noGo ? noGoS_ + dt : 0.0;
        if (noGoS_ >= kNoGoS) {
            phase_ = Phase::Reject;
            return reject(ctx);
        }
    }
    if (phase_ == Phase::Roll) {
        // the nose held down at its parked attitude until the rotation speed (a pitch-up of its own - engines under the
        // wing, the flaps: the E-7A's at 45 m/s - sat it on its tail): nose-down elevator only, its integral too
        a.elevator = std::max(pitchHold(theta, q, dt, true), 0.0);
        pitchIntegral_ = std::min(pitchIntegral_, 0.0);
        // and, too slow for the elevator, by easing the thrust as the nose rises past a degree over it (a thrust line
        // under the CG: the Su-25's rocked it back onto its tail at a walking pace)
        a.throttle = std::clamp(1.0 - 0.35 * (theta - pitchRefRad_ - kDeg) / kDeg, 0.3, 1.0);
        if (s.airspeedCalibratedMs >= vrCasMs_) phase_ = Phase::Rotate, rotateStartS_ = s.simTime, rotateFromRad_ = theta;
        return a;
    }
    // the rotation: the nose up at kRotationRateRadS from where it stands (a nose-high one: from its parked attitude) to its
    // rotation attitude, held until it flies (+ elevator: nose down)
    pitchRefRad_ = std::min(std::max(rotationRad_, rotateFromRad_), rotateFromRad_ + kRotationRateRadS * (s.simTime - rotateStartS_));
    a.elevator = pitchHold(theta, q, dt, s.onGround);
    if (!s.onGround && airborneS_ < 0.0) airborneS_ = s.simTime;
    if (s.onGround) airborneS_ = -1.0;
    if (airborneS_ >= 0.0 && s.simTime - airborneS_ > 1.0 && s.altitudeAglM > groundAglM_ + 3.0) phase_ = Phase::Climb;
    return a;
}

double LaunchBehavior::pitchHold(double theta, double q, double dt, bool onWheels) {
    // the pitch attitude held (+ elevator: nose down), its integral trimming the nose wheel's load in the rotation and the
    // climb's out-of-trim moment; stiffer on the wheels, where the gear holds the nose against the tail
    const double err = pitchRefRad_ - theta;
    pitchIntegral_ = std::clamp(pitchIntegral_ + (onWheels ? 2.0 : 1.0) * err * dt, -0.6, 0.6);
    return clamp1(-(onWheels ? 6.0 : 4.0) * err - pitchIntegral_ + q);
}

Command LaunchBehavior::climb(const ControlContext& ctx) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    if (phase_ == Phase::Done) { // climbed out, cleaned up: that height held on the runway's course, as the loops fly
        VelocityCommand v;
        v.airspeedMs = std::max(s.airspeedTrueMs, 1.0);
        v.verticalSpeedMs = std::clamp(0.2 * (hoverMslM_ - s.altitudeMslM), -5.0, 5.0);
        v.headingRad = courseRad_;
        return v;
    }
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    const double p = s.angularRateBodyRadS[0], q = s.angularRateBodyRadS[1], r = s.angularRateBodyRadS[2];
    ActuatorCommand a;
    a.throttle = 1.0, a.brakeLeft = a.brakeRight = 0.0;
    // its speed held with its pitch at full power: nose up as it is faster than its climb speed, between 2 and 15 deg
    pitchRefRad_ = std::clamp(pitchRefRad_ + dt * 0.5 * kDeg * (s.airspeedCalibratedMs - climbCasMs_), 2.0 * kDeg, 15.0 * kDeg);
    a.elevator = pitchHold(theta, q, dt, false);
    // the runway's course over the ground (crabbed into a crosswind), by a bank of at most 15 deg; the sideslip flown out
    // with the rudder (beta < 0: the nose right of the air's path; + rudder: nose left) - a long wing's adverse yaw from
    // the aileron (the RQ-4B's) swung its nose 50 deg off its path - and the yaw rate damped
    const double track = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) > 5.0 ? std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) : psi;
    const double bank = std::clamp(1.5 * geo::wrapPi(courseRad_ - track), -15.0 * kDeg, 15.0 * kDeg);
    a.aileron = clamp1(2.0 * (bank - phi) - 0.5 * p);
    a.rudder = clamp1(kSlipGain * s.betaRad + 0.5 * r);
    // cleaned up: its gear once it climbs, its flaps at kFlapsUpAglM
    const double agl = s.altitudeAglM - groundAglM_;
    if (!gearUp_ && agl > kGearUpAglM && -s.velocityNedMs[2] > 1.0) gearUp_ = true;
    if (!flapsUp_ && agl > kFlapsUpAglM) flapsUp_ = true;
    a.gearDown = gearUp_ ? 0.0 : 1.0;
    a.flaps = flapsUp_ ? 0.0 : kTakeoffFlaps;
    if (agl >= completeAglM_ && gearUp_ && flapsUp_) {
        phase_ = Phase::Done;
        hoverMslM_ = s.altitudeMslM; // (the height it levels at)
    }
    return a;
}

Command LaunchBehavior::lift(const ControlContext& ctx) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    // straight up to its hover over where it stood, facing the runway's course; there within a metre, still, for 2 s
    double north, east;
    geo::localNorthEastM(startLat_, startLon_, s.latitudeRad, s.longitudeRad, north, east);
    const double off = std::hypot(std::hypot(north, east), s.altitudeMslM - hoverMslM_);
    const double speed = std::hypot(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), s.velocityNedMs[2]);
    settledS_ = off < 1.0 && speed < 0.3 ? settledS_ + dt : 0.0;
    if (settledS_ >= 2.0) phase_ = Phase::Done;
    return PositionCommand{startLat_, startLon_, hoverMslM_, 2.0, 1.0, courseRad_};
}

bool LaunchBehavior::progress(ActivityProgress& out) const noexcept {
    out.courseRad = out.headingRad = courseRad_;
    out.crossTrackM = hovers_ ? kHold : crossM_;
    switch (phase_) {
    case Phase::LineUp: out.percent = 0.0; break;
    case Phase::Roll: out.percent = 10.0; break;
    case Phase::Rotate: out.percent = 25.0; break;
    case Phase::Climb: out.percent = 50.0; break;
    case Phase::Lift: out.percent = 50.0; break;
    case Phase::Done: out.percent = 100.0; break;
    case Phase::Reject: out.percent = 10.0; break;   // (as far as it went: it stops)
    case Phase::Stopped: out.percent = own_ ? 100.0 : 10.0; break;
    }
    return true;
}

} // namespace fsim::control
