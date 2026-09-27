#include "control/Builtin.h"

#include "control/Registry.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"
#include "fsim/RotorControllers.h"
#include "core/Units.h"

#include <algorithm>
#include <limits>
#include <cmath>
#include <cstring>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

double clamp11(double v) noexcept { return std::clamp(v, -1.0, 1.0); }

/// True when a loop missed a period - the stack flew another level meanwhile
/// - so what it holds (a setpoint on its way, a smoothed value) no longer
/// describes the flight and it starts again from the state. Measured on the
/// vehicle's own clock, which a running loop sees advance by its period.
bool resumed(const ControlContext& ctx, double& lastTime) noexcept {
    const double t = ctx.state.simTime;
    const bool stale = lastTime >= 0.0 && (t - lastTime > 1.5 * ctx.dt || t < lastTime);
    lastTime = t;
    return stale;
}

} // namespace

// --- Parameters ----------------------------------------------------------------

bool Parameters::set(std::string_view name, double value) noexcept {
    for (auto& [n, p] : table_)
        if (name == n) {
            *p = value;
            return true;
        }
    return false;
}

std::optional<double> Parameters::get(std::string_view name) const noexcept {
    for (const auto& [n, p] : table_)
        if (name == n) return *p;
    return std::nullopt;
}

// --- AttitudeLoop ---------------------------------------------------------------

AttitudeLoop::AttitudeLoop() {
    params_.add("roll.kp", &roll.kp); params_.add("roll.ki", &roll.ki); params_.add("roll.kd", &roll.kd);
    params_.add("pitch.kp", &pitch.kp); params_.add("pitch.ki", &pitch.ki); params_.add("pitch.kd", &pitch.kd);
    params_.add("airspeed.kp", &airspeed.kp); params_.add("airspeed.ki", &airspeed.ki);
    params_.add("heading.gain", &headingGain);
    params_.add("rudder.beta_gain", &rudderBetaGain);
    params_.add("throttle.feedforward", &throttleFeedforward);
    params_.add("roll.max_rate", &maxRollRateRadS);
    params_.add("roll.integral_limit", &roll.integralLimit);
    params_.add("pitch.integral_limit", &pitch.integralLimit);
    params_.add("airspeed.integral_limit", &airspeed.integralLimit);
    params_.add("schedule.tas_ms", &schedule.tasMs);
    params_.add("schedule.eas_ms", &schedule.easMs);
    params_.add("roll.eas_exponent", &rollEasExponent); params_.add("roll.tas_exponent", &rollTasExponent);
    params_.add("pitch.eas_exponent", &pitchEasExponent); params_.add("pitch.tas_exponent", &pitchTasExponent);
    params_.add("pitch.trim", &pitchTrim); params_.add("pitch.trim_lift", &pitchTrimLift);
    airspeed.outMin = -1.0;
    airspeed.outMax = 1.0;
}

Command AttitudeLoop::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AttitudeCommand>(in);
    const auto& s = ctx.sensed;
    const double rollNow = s.eulerRad[0], pitchNow = s.eulerRad[1], yawNow = s.eulerRad[2];
    ActuatorCommand out;

    // lateral: bank (or heading through bank) on the ailerons, sideslip on the rudder
    if (ctx.engaged & axisBit(Axis::Roll)) {
        double rollTarget = orHold(c.rollRad, 0.0);
        if (!isHold(c.headingRad)) {
            const double maxBank = orHold(c.maxBankRad, 0.785);
            rollTarget = std::clamp(headingGain * schedule.speedRatio(s) * geo::wrapPi(c.headingRad - yawNow), -maxBank, maxBank);
        }
        if (!haveRef_ || resumed(ctx, lastTime_)) {
            rollRef_ = rollNow;
            haveRef_ = true;
        }
        rollRef_ = rateLimit(rollRef_, rollTarget, maxRollRateRadS, ctx.dt);
        out.aileron = roll.update(geo::wrapPi(rollRef_ - rollNow), s.angularRateBodyRadS[0], ctx.dt,
                                  schedule.factor(s, rollEasExponent, rollTasExponent));
        out.rudder = clamp11(rudderBetaGain * s.betaRad);
    } else {
        roll.reset(); // someone else flies it: start afresh when it comes back
        haveRef_ = false;
        out.aileron = out.rudder = kHold;
    }
    // longitudinal: pitch attitude on the elevator. Positive elevator is
    // nose-down in JSBSim: negate the nose-up demand. The trim is the
    // elevator a level turn at this bank holds.
    if (ctx.engaged & axisBit(Axis::Pitch)) {
        const double turn = 1.0 / std::max(std::cos(rollNow), 0.3);
        out.elevator = schedule.trimElevator(s, turn, pitchTrim, pitchTrimLift) -
                       pitch.update(orHold(c.pitchRad, pitchNow) - pitchNow, s.angularRateBodyRadS[1], ctx.dt,
                                    schedule.factor(s, pitchEasExponent, pitchTasExponent));
    } else {
        pitch.reset();
        out.elevator = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Thrust)) {
        if (!isHold(c.airspeedMs))
            out.throttle = std::clamp(throttleFeedforward + airspeed.update(c.airspeedMs - s.airspeedTrueMs, 0.0, ctx.dt), 0.0, 1.0);
        else
            out.throttle = c.throttle; // may be kHold: the stack keeps the last value
    } else {
        airspeed.reset();
        out.throttle = kHold;
    }
    return out;
}

void AttitudeLoop::reset() {
    roll.reset();
    pitch.reset();
    airspeed.reset();
    haveRef_ = false;
    lastTime_ = -1.0;
}

// --- PseudoAttitudeLoop ---------------------------------------------------------

PseudoAttitudeLoop::PseudoAttitudeLoop() {
    params_.add("roll.gain", &rollGain);
    params_.add("roll.kd", &rollDamping);
    params_.add("roll.max_rate", &maxRollRateRadS);
    params_.add("heading.gain", &headingGain);
    params_.add("pitch.kp", &pitch.kp); params_.add("pitch.ki", &pitch.ki); params_.add("pitch.kd", &pitch.kd);
    params_.add("pitch.integral_limit", &pitch.integralLimit);
    params_.add("pitch.max_rate", &pitch.outMax);
    params_.add("airspeed.kp", &airspeed.kp); params_.add("airspeed.ki", &airspeed.ki);
    params_.add("airspeed.integral_limit", &airspeed.integralLimit);
    params_.add("schedule.tas_ms", &schedule.tasMs);
}

Command PseudoAttitudeLoop::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AttitudeCommand>(in);
    const auto& s = ctx.sensed;
    const double rollNow = s.eulerRad[0], pitchNow = s.eulerRad[1];
    const double rollCos = std::cos(rollNow);
    AccelerationCommand out{kHold, kHold, kHold, kHold};

    // lateral: the bank error to a roll rate (the setpoint slews at
    // roll.max_rate). A heading is the path's through the air - the nose's
    // plus the sideslip - so the dutch roll, which swings the nose about a
    // path that stays put, does not reach the bank.
    if (ctx.engaged & axisBit(Axis::Roll)) {
        double rollTarget = orHold(c.rollRad, 0.0);
        if (!isHold(c.headingRad)) {
            const double maxBank = orHold(c.maxBankRad, 0.785);
            const double path = s.eulerRad[2] + s.betaRad * rollCos;
            rollTarget = std::clamp(headingGain * schedule.speedRatio(s) * geo::wrapPi(c.headingRad - path), -maxBank, maxBank);
        }
        if (!haveRef_ || resumed(ctx, lastTime_)) {
            rollRef_ = rollNow;
            haveRef_ = true;
        }
        rollRef_ = rateLimit(rollRef_, rollTarget, maxRollRateRadS, ctx.dt);
        out.rollRateRadS = std::clamp(rollGain * geo::wrapPi(rollRef_ - rollNow) - rollDamping * s.angularRateBodyRadS[0], -2.0 * maxRollRateRadS,
                                      2.0 * maxRollRateRadS);
    } else {
        haveRef_ = false; // someone else flies it: start afresh when it comes back
    }
    // longitudinal: the pitch error to a pitch rate (damped on the rate the
    // pitch changes at, q cos(roll) - r sin(roll)), and that to the load
    // factor that turns the path so at this bank and speed: the steady
    // cos(path) / cos(bank), plus tas q / g over the bank
    if (ctx.engaged & axisBit(Axis::Pitch)) {
        pitch.outMin = -pitch.outMax;
        const double tas = std::max(s.airspeedTrueMs, 10.0);
        const double climb = std::clamp(-s.velocityNedMs[2] / tas, -1.0, 1.0);
        const double pitchRate = s.angularRateBodyRadS[1] * rollCos - s.angularRateBodyRadS[2] * std::sin(rollNow);
        const double q = pitch.update(orHold(c.pitchRad, pitchNow) - pitchNow, pitchRate, ctx.dt);
        out.loadFactorG = (std::sqrt(1.0 - climb * climb) + tas * q / kG) / std::max(rollCos, 0.3);
    } else {
        pitch.reset();
    }
    // thrust: the airspeed error to an acceleration along the path, or the throttle as given
    if (ctx.engaged & axisBit(Axis::Thrust)) {
        if (!isHold(c.airspeedMs)) out.longitudinalMs2 = airspeed.update(c.airspeedMs - s.airspeedTrueMs, 0.0, ctx.dt);
        else out.throttle = c.throttle; // may be kHold: the stack keeps the last value
    } else {
        airspeed.reset();
    }
    return out;
}

void PseudoAttitudeLoop::reset() {
    pitch.reset();
    airspeed.reset();
    haveRef_ = false;
    lastTime_ = -1.0;
}

// --- AccelerationLoop -----------------------------------------------------------

AccelerationLoop::AccelerationLoop() {
    params_.add("load_factor.kp", &loadFactor.kp); params_.add("load_factor.ki", &loadFactor.ki); params_.add("load_factor.kd", &loadFactor.kd);
    params_.add("roll_rate.kp", &rollRate.kp); params_.add("roll_rate.ki", &rollRate.ki);
    params_.add("longitudinal.kp", &longitudinal.kp); params_.add("longitudinal.ki", &longitudinal.ki);
    params_.add("rudder.beta_gain", &rudderBetaGain);
    params_.add("throttle.feedforward", &throttleFeedforward);
    params_.add("load_factor.integral_limit", &loadFactor.integralLimit);
    params_.add("roll_rate.integral_limit", &rollRate.integralLimit);
    params_.add("load_factor.feedforward", &loadFactorFeedforward);
    params_.add("load_factor.path_hold", &loadFactorPathHold);
    params_.add("roll_rate.feedforward", &rollRateFeedforward);
    params_.add("longitudinal.feedforward", &longitudinalFeedforward);
    params_.add("schedule.tas_ms", &schedule.tasMs);
    params_.add("schedule.eas_ms", &schedule.easMs);
    params_.add("load_factor.eas_exponent", &loadFactorEasExponent); params_.add("load_factor.tas_exponent", &loadFactorTasExponent);
    params_.add("roll_rate.eas_exponent", &rollRateEasExponent); params_.add("roll_rate.tas_exponent", &rollRateTasExponent);
    params_.add("pitch.trim", &pitchTrim); params_.add("pitch.trim_lift", &pitchTrimLift);
    longitudinal.outMin = -1.0;
    longitudinal.outMax = 1.0;
}

Command AccelerationLoop::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AccelerationCommand>(in);
    const auto& s = ctx.sensed;
    const double n = orHold(c.loadFactorG, 1.0), p = orHold(c.rollRateRadS, 0.0);
    const double sn = schedule.factor(s, loadFactorEasExponent, loadFactorTasExponent);
    const double sp = schedule.factor(s, rollRateEasExponent, rollRateTasExponent);
    // what neutral stick gives: the load factor that balances gravity at this
    // attitude (a law that holds the flight path), or 1 g (a trimmed surface)
    const double level = 1.0 + loadFactorPathHold * (std::cos(s.eulerRad[1]) * std::cos(s.eulerRad[0]) - 1.0);
    ActuatorCommand out;
    if (ctx.engaged & axisBit(Axis::Pitch)) {
        out.elevator = schedule.trimElevator(s, n, pitchTrim, pitchTrimLift) -
                       (sn * loadFactorFeedforward * (n - level) + loadFactor.update(n - s.loadFactor, s.angularRateBodyRadS[1], ctx.dt, sn));
    } else {
        loadFactor.reset();
        out.elevator = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Roll)) {
        out.aileron = sp * rollRateFeedforward * p + rollRate.update(p - s.angularRateBodyRadS[0], 0.0, ctx.dt, sp);
        out.rudder = clamp11(rudderBetaGain * s.betaRad);
    } else {
        rollRate.reset();
        out.aileron = out.rudder = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Thrust)) {
        if (!isHold(c.longitudinalMs2))
            out.throttle = std::clamp(throttleFeedforward + longitudinalFeedforward * c.longitudinalMs2 +
                                          longitudinal.update(c.longitudinalMs2 - s.accelerationBodyMs2[0], 0.0, ctx.dt),
                                      0.0, 1.0);
        else
            out.throttle = c.throttle;
    } else {
        longitudinal.reset();
        out.throttle = kHold;
    }
    return out;
}

void AccelerationLoop::reset() {
    loadFactor.reset();
    rollRate.reset();
    longitudinal.reset();
}

// --- VelocityLoop ---------------------------------------------------------------

VelocityLoop::VelocityLoop() {
    params_.add("vertical_speed.kp", &verticalSpeed.kp); params_.add("vertical_speed.ki", &verticalSpeed.ki);
    params_.add("vertical_speed.integral_limit", &verticalSpeed.integralLimit);
    params_.add("vertical_speed.feedforward", &flightPathFeedforward);
    params_.add("vertical_speed.command_lag", &commandLagS);
    params_.add("vertical_speed.alpha_zero_lift", &alphaZeroLift);
    params_.add("pitch.min", &verticalSpeed.outMin);
    params_.add("pitch.max", &verticalSpeed.outMax);
    params_.add("max_bank", &maxBankRad);
    params_.add("schedule.tas_ms", &referenceSpeedMs);
}

Command VelocityLoop::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<VelocityCommand>(in);
    const auto& s = ctx.sensed;
    AttitudeCommand out;
    if (!(ctx.engaged & axisBit(Axis::Pitch))) {
        // someone else flies the pitch: start afresh when it comes back
        verticalSpeed.reset();
        started_ = false;
        lastTime_ = -1.0;
        out.pitchRad = kHold;
        return lateralAndSpeed(ctx, c, out);
    }
    const double vz = -s.velocityNedMs[2];
    double vzTarget = orHold(c.verticalSpeedMs, 0.0);
    const double tas = std::max(s.airspeedTrueMs, 10.0);
    // a pitch change moves the vertical speed by tas times as much: the gains hold at the reference speed
    const double scale = referenceSpeedMs > 0.0 ? referenceSpeedMs / std::max(tas, 0.3 * referenceSpeedMs) : 1.0;
    const auto lag = [&](double value, double target, double seconds) {
        return seconds > 0.0 ? value + (target - value) * std::min(1.0, ctx.dt / seconds) : target;
    };
    if (resumed(ctx, lastTime_)) started_ = false;
    if (commandLagS > 0.0) {
        // the command followed through its lag, from the vertical speed flown when the loop took over
        vzRef_ = started_ ? lag(vzRef_, vzTarget, commandLagS) : vz;
        vzTarget = vzRef_;
    }
    double path = 0.0;
    if (flightPathFeedforward != 0.0) {
        // pitch = the flight path the vertical speed needs + the angle of attack
        // the wing would fly at 1 g: alpha scaled back by the load factor about
        // the zero-lift angle, so a pull does not feed itself (and a steady
        // turn gives the level value). Smoothed over a second; the integrator
        // trims what is left.
        const double alpha1g = alphaZeroLift + (s.alphaRad - alphaZeroLift) / std::clamp(s.loadFactor, 0.5, 3.0);
        alpha_ = started_ ? lag(alpha_, alpha1g, 1.0) : alpha1g;
        path = flightPathFeedforward * (std::asin(std::clamp(vzTarget / tas, -0.5, 0.5)) + alpha_);
    }
    started_ = true;
    out.pitchRad = std::clamp(path + verticalSpeed.update(vzTarget - vz, 0.0, ctx.dt, scale), verticalSpeed.outMin, verticalSpeed.outMax);
    return lateralAndSpeed(ctx, c, out);
}

Command VelocityLoop::lateralAndSpeed(const ControlContext& ctx, const VelocityCommand& c, AttitudeCommand& out) const {
    if (ctx.engaged & axisBit(Axis::Roll)) {
        out.maxBankRad = maxBankRad;
        if (!isHold(c.turnRateRadS)) {
            const double v = std::max(ctx.sensed.airspeedTrueMs, 10.0);
            out.rollRad = std::clamp(std::atan(c.turnRateRadS * v / kG), -maxBankRad, maxBankRad);
        } else if (!isHold(c.headingRad)) {
            out.headingRad = c.headingRad;
        } else {
            out.rollRad = 0.0;
        }
    } else {
        out.rollRad = out.maxBankRad = kHold;
    }
    out.airspeedMs = ctx.engaged & axisBit(Axis::Thrust) ? c.airspeedMs : kHold;
    return out;
}

void VelocityLoop::reset() {
    verticalSpeed.reset();
    started_ = false;
    lastTime_ = -1.0;
}

// --- PositionLoop ---------------------------------------------------------------

PositionLoop::PositionLoop() {
    params_.add("altitude.gain", &altitudeGain);
    params_.add("max_vertical_speed", &maxVerticalSpeedMs);
}

Command PositionLoop::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<PositionCommand>(in);
    const auto& s = ctx.sensed;
    VelocityCommand out;
    if (ctx.engaged & axisBit(Axis::Roll)) {
        const double distance = geo::distanceM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
        out.headingRad = distance < c.captureRadiusM ? s.eulerRad[2] : geo::bearingRad(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
    }
    out.verticalSpeedMs = ctx.engaged & axisBit(Axis::Pitch)
                              ? std::clamp(altitudeGain * (c.altitudeMslM - s.altitudeMslM), -maxVerticalSpeedMs, maxVerticalSpeedMs)
                              : kHold;
    out.airspeedMs = ctx.engaged & axisBit(Axis::Thrust) ? c.airspeedMs : kHold;
    return out;
}

// --- Behaviours -----------------------------------------------------------------

void HoldBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    const double airspeed = command.param("airspeed_ms", kHold);
    if ((ctx.features & kFeatureHover) && isHold(airspeed)) {
        // a rotorcraft keeps the velocity over the ground it had: a hover stays
        // put in wind, where holding the airspeed it had would fly it off
        target_.airspeedMs = kHold;
        target_.northMs = s.velocityNedMs[0];
        target_.eastMs = s.velocityNedMs[1];
    } else {
        target_.airspeedMs = orHold(airspeed, s.airspeedTrueMs);
        target_.northMs = target_.eastMs = kHold;
    }
    target_.verticalSpeedMs = 0.0;
    target_.headingRad = command.param("heading_deg", units::radiansToDegrees(s.eulerRad[2])) * units::kDegreesToRadians;
    altitude_ = command.param("altitude_m", s.altitudeMslM);
    hovers_ = (ctx.features & kFeatureHover) != 0;
}

Command HoldBehavior::update(const ControlContext& ctx, const Command&) {
    VelocityCommand out = target_;
    // to its altitude at its own position loop's gain and vertical speeds, as a mode flies it (a gain of
    // 0.25/s is six times what a Mirage 2000's loops follow: it circled its new altitude 25 m either way)
    out.verticalSpeedMs = ctx.performance ? route::verticalSpeedTo(altitude_, 0.0, ctx.sensed, *ctx.performance, hovers_)
                                          : std::clamp(0.25 * (altitude_ - ctx.sensed.altitudeMslM), -6.0, 6.0);
    return out;
}

bool HoldBehavior::progress(ActivityProgress& out) const noexcept {
    out.headingRad = target_.headingRad;
    out.altitudeMslM = altitude_;
    if (!isHold(target_.northMs)) { // a rotorcraft's velocity over the ground
        out.speedMs = std::hypot(target_.northMs, target_.eastMs);
        out.speedReference = 2.0; // SpeedReference::GroundSpeed
        if (out.speedMs > 0.1) out.courseRad = std::atan2(target_.eastMs, target_.northMs);
    } else {
        out.speedMs = target_.airspeedMs;
        out.speedReference = 0.0; // SpeedReference::TrueAirspeed
    }
    return true;
}

void WaypointsBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    index_ = 0;
    loop_ = command.param("loop", 0.0) != 0.0;
    finished_ = command.points.empty();
    failed_ = false;
    closest_ = lastHeading_ = kHold;
    turned_ = 0.0;
    hovers_ = (ctx.features & kFeatureHover) != 0;
    // without one, a wing flies the airspeed it had; an aircraft that hovers
    // its position loop's own speed (hovering, the airspeed it had is none)
    airspeed_ = command.param("airspeed_ms", hovers_ ? kHold : ctx.sensed.airspeedTrueMs);
    // the route's length from here, for progress()
    route_ = &command;
    laps_ = 0;
    startLat_ = lat_ = ctx.sensed.latitudeRad, startLon_ = lon_ = ctx.sensed.longitudeRad;
    totalM_ = 0.0;
    double lat = startLat_, lon = startLon_;
    for (const auto& p : command.points) {
        totalM_ += geo::distanceM(lat, lon, p.latitudeRad, p.longitudeRad);
        lat = p.latitudeRad, lon = p.longitudeRad;
    }
}

Command WaypointsBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<BehaviorCommand>(in);
    const auto& s = ctx.sensed;
    route_ = &c;
    lat_ = s.latitudeRad, lon_ = s.longitudeRad, north_ = s.velocityNedMs[0], east_ = s.velocityNedMs[1]; // for progress()
    if (c.points.empty()) {
        VelocityCommand hold;
        hold.headingRad = s.eulerRad[2];
        if (hovers_) hold.northMs = hold.eastMs = 0.0; // stay where it is
        else hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    if (index_ >= c.points.size()) index_ = c.points.size() - 1;
    if (failed_) return fallback_; // straight and level: it could not close on its point
    const PositionCommand& p = c.points[index_];
    const double distance = geo::distanceM(s.latitudeRad, s.longitudeRad, p.latitudeRad, p.longitudeRad);
    if (!finished_ && distance < p.captureRadiusM) {
        if (index_ + 1 < c.points.size()) ++index_;
        else if (loop_) index_ = 0, ++laps_;
        else finished_ = true;
        closest_ = lastHeading_ = kHold; // a new point to close on
        turned_ = 0.0;
    } else if (!finished_) {
        // a point it circles without closing: a full turn (and a little) with no new least distance
        if (isHold(closest_) || distance < closest_ - 1.0) closest_ = distance, turned_ = 0.0;
        else if (!isHold(lastHeading_)) turned_ += std::abs(geo::wrapPi(s.eulerRad[2] - lastHeading_));
        lastHeading_ = s.eulerRad[2];
        if (turned_ > 2.0 * units::kPi + 0.5) {
            failed_ = true;
            fallback_ = VelocityCommand{};
            fallback_.headingRad = s.eulerRad[2];
            fallback_.verticalSpeedMs = 0.0;
            if (hovers_) fallback_.northMs = fallback_.eastMs = 0.0; // stay where it is
            else fallback_.airspeedMs = s.airspeedTrueMs;
            return fallback_;
        }
    }
    PositionCommand out = c.points[index_];
    if (isHold(out.airspeedMs)) out.airspeedMs = airspeed_;
    return out;
}

bool WaypointsBehavior::progress(ActivityProgress& out) const noexcept {
    if (!route_ || route_->points.empty()) return false;
    const auto& points = route_->points;
    const std::size_t i = std::min(index_, points.size() - 1);
    const PositionCommand& p = points[i];
    out.segment = static_cast<std::uint32_t>(i);
    out.segments = static_cast<std::uint32_t>(points.size());
    out.laps = laps_;
    out.altitudeMslM = p.altitudeMslM;
    const double speed = isHold(p.airspeedMs) ? airspeed_ : p.airspeedMs;
    out.speedMs = speed;
    out.speedReference = isHold(speed) ? kHold : 0.0; // SpeedReference::TrueAirspeed; none: the position loop's own
    if (finished_) {
        out.percent = out.segmentPercent = 100.0;
        out.distanceToGoM = out.timeToGoS = 0.0;
        return true;
    }
    const double toPoint = geo::distanceM(lat_, lon_, p.latitudeRad, p.longitudeRad);
    const double leg = i == 0 ? geo::distanceM(startLat_, startLon_, p.latitudeRad, p.longitudeRad)
                              : geo::distanceM(points[i - 1].latitudeRad, points[i - 1].longitudeRad, p.latitudeRad, p.longitudeRad);
    out.segmentPercent = leg > 1e-6 ? std::clamp(100.0 * (1.0 - toPoint / leg), 0.0, 100.0) : 100.0;
    out.courseRad = geo::wrapPi(geo::bearingRad(lat_, lon_, p.latitudeRad, p.longitudeRad));
    if (loop_) return true; // a route that repeats has no end to go to
    double rest = 0.0;
    for (std::size_t k = i + 1; k < points.size(); ++k)
        rest += geo::distanceM(points[k - 1].latitudeRad, points[k - 1].longitudeRad, points[k].latitudeRad, points[k].longitudeRad);
    out.distanceToGoM = toPoint + rest;
    out.percent = totalM_ > 1e-6 ? std::clamp(100.0 * (1.0 - out.distanceToGoM / totalM_), 0.0, 100.0) : 100.0;
    if (const double overGround = std::hypot(north_, east_); overGround > 0.1) out.timeToGoS = out.distanceToGoM / overGround;
    return true;
}

void LoiterBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    target_ = command.target;
    centreLat_ = command.param("lat_deg", units::radiansToDegrees(s.latitudeRad)) * units::kDegreesToRadians;
    centreLon_ = command.param("lon_deg", units::radiansToDegrees(s.longitudeRad)) * units::kDegreesToRadians;
    const bool hovers = (ctx.features & kFeatureHover) != 0;
    airspeed_ = command.param("airspeed_ms", hovers ? kHold : s.airspeedTrueMs); // kHold: the position loop's speed
    // left out: 1500 m, or wider where the aircraft's turn at the speed it flies needs it (a heavy's is kilometres)
    double radius = command.param("radius_m", kHold);
    if (isHold(radius)) {
        radius = 1500.0;
        if (ctx.performance && !hovers) radius = std::max(radius, 1.25 * orbitRadiusM(*ctx.performance, orHold(airspeed_, s.airspeedTrueMs)));
    }
    radius_ = std::max(hovers ? 1.0 : 100.0, radius);
    // a rotorcraft given no speed: its cruise, no faster than it can follow the circle (route::lateralLimit;
    // at the position loop's fastest, 30 m/s, a helicopter's 150 m circle needs 6 m/s2 of the 3.6 it has)
    if (hovers && isHold(airspeed_) && ctx.performance) {
        const Performance& perf = *ctx.performance;
        const double cruise = std::isfinite(perf.cruiseTasMs) ? perf.cruiseTasMs : perf.maxGroundSpeedMs;
        airspeed_ = std::isfinite(cruise) ? std::min(cruise, route::lateralLimit(perf, radius_)) : route::lateralLimit(perf, radius_);
    }
    altitude_ = command.param("altitude_m", s.altitudeMslM);
    clockwise_ = command.param("clockwise", 1.0) != 0.0;
    lastTheta_ = distance_ = kHold;
    swept_ = trim_ = 0.0;
}

Command LoiterBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    double cLat = centreLat_, cLon = centreLon_;
    const sim::VehicleState* t = target_ && ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    lost_ = target_ && !t; // circling the last centre
    if (t) {
        cLat = t->latitudeRad;
        cLon = t->longitudeRad;
    }
    double north, east;
    geo::localNorthEastM(cLat, cLon, s.latitudeRad, s.longitudeRad, north, east);
    const double distance = std::hypot(north, east);
    const double theta = std::atan2(east, north);
    if (!isHold(lastTheta_)) { // the laps, for progress(): the angle swept (atan2's jump at +-pi taken out)
        double d = theta - lastTheta_;
        d += d > units::kPi ? -2.0 * units::kPi : d < -units::kPi ? 2.0 * units::kPi : 0.0;
        swept_ += d;
    }
    lastTheta_ = theta, distance_ = distance;
    // Carrot on the circle, ahead of the vehicle's current angular position;
    // far away the carrot leads less so the approach is nearly direct. Near
    // the circle the carrot's own circle is trimmed until the one flown is the
    // one asked: a vehicle that follows the carrot at once flies the chord to
    // it, inside (R cos 0.6 = 0.83 R), and one whose turn lags flies wider.
    const double lead = distance > 2.0 * radius_ ? 0.15 : 0.6;
    if (std::abs(distance - radius_) < 0.3 * radius_ && ctx.dt > 0.0) {
        const double lapS = 2.0 * units::kPi * radius_ / std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), 1.0);
        trim_ = std::clamp(trim_ + (radius_ - distance) * ctx.dt / std::max(20.0, 0.25 * lapS), -0.3 * radius_, 0.4 * radius_);
    }
    const double aim = theta + (clockwise_ ? lead : -lead), carrot = radius_ + trim_;
    PositionCommand out;
    geo::offsetLatLon(cLat, cLon, carrot * std::cos(aim), carrot * std::sin(aim), out.latitudeRad, out.longitudeRad);
    out.altitudeMslM = altitude_;
    out.airspeedMs = airspeed_;
    out.captureRadiusM = 30.0;
    return out;
}

bool LoiterBehavior::progress(ActivityProgress& out) const noexcept {
    out.laps = static_cast<std::uint32_t>(std::abs(swept_) / (2.0 * units::kPi));
    if (!isHold(distance_)) out.crossTrackM = clockwise_ ? radius_ - distance_ : distance_ - radius_; // + right of its path: inside a right turn
    out.altitudeMslM = altitude_;
    out.speedMs = airspeed_;
    out.speedReference = isHold(airspeed_) ? kHold : 0.0; // SpeedReference::TrueAirspeed
    return true;
}

void PursuitBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    target_ = command.target;
    rangeM_ = command.param("range_m", 300.0);
    leadS_ = command.param("lead_s", 2.0);
    minSpeed_ = command.param("min_airspeed_ms", 30.0);
    maxSpeed_ = command.param("max_airspeed_ms", 400.0);
    decelMs2_ = 0.25, lagS_ = 10.0;
    if (const Performance* perf = ctx.performance) {
        if (std::isfinite(perf->maxDecelerationMs2) && perf->maxDecelerationMs2 > 0.0) decelMs2_ = 0.5 * perf->maxDecelerationMs2;
        if (std::isfinite(perf->velocityBandwidthRadS) && perf->velocityBandwidthRadS > 0.0) lagS_ = 1.0 / perf->velocityBandwidthRadS;
    }
}

Command PursuitBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    lost_ = !t;
    if (!t) {
        VelocityCommand hold;
        hold.headingRad = s.eulerRad[2];
        hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    // Aim at the stand-off point: range_m behind where the target will be,
    // along its track (a target that hardly moves: range_m from it on this
    // side). Inside the range the point is behind the pursuer, which turns
    // back out: the range is the least it keeps.
    double north, east; // the pursuer from the target
    geo::localNorthEastM(t->latitudeRad, t->longitudeRad, s.latitudeRad, s.longitudeRad, north, east);
    const double distance = std::hypot(north, east);
    const double targetSpeed = std::hypot(t->velocityNedMs[0], t->velocityNedMs[1]);
    double un = 0.0, ue = 0.0; // the unit vector from the stand-off point to the target
    if (targetSpeed > 1.0) un = t->velocityNedMs[0] / targetSpeed, ue = t->velocityNedMs[1] / targetSpeed;
    else if (distance > 1.0) un = -north / distance, ue = -east / distance;
    PositionCommand out;
    geo::offsetLatLon(t->latitudeRad, t->longitudeRad, t->velocityNedMs[0] * leadS_ - rangeM_ * un, t->velocityNedMs[1] * leadS_ - rangeM_ * ue,
                      out.latitudeRad, out.longitudeRad);
    out.altitudeMslM = t->altitudeMslM - t->velocityNedMs[2] * leadS_;
    // the target's speed, and closing no faster than it can stop closing before the range: after its
    // speed loop's lag, at the deceleration it is designed for (lag v + v^2 / 2a = gap)
    const double gap = distance - rangeM_;
    const double stoppable = decelMs2_ * (std::sqrt(lagS_ * lagS_ + 2.0 * gap / decelMs2_) - lagS_);
    const double closing = gap > 0.0 ? std::min(0.05 * gap, stoppable) : 0.05 * gap; // (a gain its speed loop follows)
    out.airspeedMs = std::clamp(targetSpeed + closing, minSpeed_, maxSpeed_);
    out.captureRadiusM = 20.0;
    return out;
}

void EvadeBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    target_ = command.target;
    altitude_ = ctx.sensed.altitudeMslM + command.param("altitude_delta_m", -300.0);
    // left out: the airspeed it has; a rotorcraft's at least its cruise (hovering, it has none to flee at)
    double airspeed = ctx.sensed.airspeedTrueMs;
    if ((ctx.features & kFeatureHover) != 0 && ctx.performance && std::isfinite(ctx.performance->cruiseTasMs))
        airspeed = std::max(airspeed, ctx.performance->cruiseTasMs);
    airspeed_ = command.param("airspeed_ms", airspeed);
    // the floor: never lower over the terrain than floor_agl_m, nor than it started when that was lower
    floorAglM_ = std::min(command.param("floor_agl_m", 150.0), std::max(ctx.sensed.altitudeAglM, 0.0));
    floored_ = false;
}

Command EvadeBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    VelocityCommand out;
    out.airspeedMs = airspeed_;
    const double floor = s.altitudeMslM - s.altitudeAglM + floorAglM_; // over the terrain under it now
    floored_ = altitude_ < floor;
    out.verticalSpeedMs = std::clamp(0.25 * (std::max(altitude_, floor) - s.altitudeMslM), -8.0, 8.0);
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    lost_ = !t;
    out.headingRad = t ? geo::bearingRad(t->latitudeRad, t->longitudeRad, s.latitudeRad, s.longitudeRad) : s.eulerRad[2];
    return out;
}

void FormationBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    target_ = command.target;
    ahead_ = command.param("ahead_m", -100.0);
    right_ = command.param("right_m", 60.0);
    below_ = command.param("below_m", 0.0);
    hovers_ = (ctx.features & kFeatureHover) != 0;
    // what its closing is designed for, as pursuit's: a deceleration it can keep up and its speed loop's lag
    decelMs2_ = 0.25, lagS_ = 10.0;
    if (const Performance* perf = ctx.performance) {
        if (std::isfinite(perf->maxDecelerationMs2) && perf->maxDecelerationMs2 > 0.0) decelMs2_ = 0.5 * perf->maxDecelerationMs2;
        if (std::isfinite(perf->velocityBandwidthRadS) && perf->velocityBandwidthRadS > 0.0) lagS_ = 1.0 / perf->velocityBandwidthRadS;
    }
    // left out: half its speed loop's bandwidth, a closing its speed follows without overshoot (a wing's 0.05/s)
    closureGain_ = command.param("closure_gain", 0.5 / lagS_);
}

Command FormationBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    lost_ = !t;
    VelocityCommand out;
    if (!t) {
        out.headingRad = s.eulerRad[2];
        if (hovers_) out.northMs = out.eastMs = 0.0;
        else out.airspeedMs = s.airspeedTrueMs;
        return out;
    }
    const double psi = t->eulerRad[2];
    const double cs = std::cos(psi), sn = std::sin(psi);
    // Slot position in the leader's heading frame -> local NE.
    const double slotNorth = ahead_ * cs - right_ * sn;
    const double slotEast = ahead_ * sn + right_ * cs;
    double meNorth, meEast;
    geo::localNorthEastM(t->latitudeRad, t->longitudeRad, s.latitudeRad, s.longitudeRad, meNorth, meEast);
    const double errNorth = slotNorth - meNorth, errEast = slotEast - meEast;
    // closing on the slot no faster than it could stop closing there: after its speed loop's lag, at the
    // deceleration it is designed for (lag v + v^2 / 2a = distance), and at its gain nearer
    auto closing = [&](double distance) {
        return std::min(closureGain_ * distance, decelMs2_ * (std::sqrt(lagS_ * lagS_ + 2.0 * distance / decelMs2_) - lagS_));
    };
    const double slotAltitude = t->altitudeMslM - below_;
    out.verticalSpeedMs = std::clamp(0.3 * (slotAltitude - s.altitudeMslM) - t->velocityNedMs[2], -8.0, 8.0);
    if (hovers_) { // over the ground: the leader's velocity and a closing one straight to the slot, facing as the leader does
        const double distance = std::hypot(errNorth, errEast);
        const double speed = distance > 1e-6 ? closing(distance) / distance : 0.0;
        out.northMs = t->velocityNedMs[0] + speed * errNorth;
        out.eastMs = t->velocityNedMs[1] + speed * errEast;
        out.headingRad = psi;
        return out;
    }
    const double along = errNorth * cs + errEast * sn;  // + = slot is ahead of me
    const double cross = -errNorth * sn + errEast * cs; // + = slot is to my right
    // along: the leader's airspeed (the same air), and the closing; across: onto the slot's line along a
    // look-ahead its heading loop follows, as a route's legs are flown
    const double leaderTas = t->airspeedTrueMs > 1.0 ? t->airspeedTrueMs : std::hypot(t->velocityNedMs[0], t->velocityNedMs[1]);
    out.airspeedMs = std::clamp(leaderTas + std::copysign(closing(std::abs(along)), along), 0.5 * leaderTas, 1.5 * leaderTas + 5.0);
    const double bandwidth = ctx.performance ? ctx.performance->courseBandwidthRadS(s.airspeedTrueMs) : 0.2;
    const double lookahead = std::max(3.0 * std::max(s.airspeedTrueMs, 1.0) / bandwidth, 50.0);
    out.headingRad = psi + std::clamp(std::atan(cross / lookahead), -0.6, 0.6);
    return out;
}

void AerobaticBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    manoeuvre_ = static_cast<Manoeuvre>(static_cast<int>(command.param("manoeuvre", 1.0)));
    loadFactor_ = command.param("load_factor_g", 3.5);
    rollRate_ = command.param("roll_rate_rad_s", 1.5);
    minCasMs_ = ctx.performance ? ctx.performance->minCasMs : kHold;
    alphaMaxRad_ = ctx.envelope ? ctx.envelope->alphaMaxRad : kHold;
    reset();
    const auto& s = ctx.sensed;
    entryAltitude_ = s.altitudeMslM;
    entryHeading_ = s.eulerRad[2];
    entrySpeed_ = s.airspeedTrueMs;
    phase_ = (manoeuvre_ == AileronRoll || manoeuvre_ == SplitS) ? Roll : Pull;
}

void AerobaticBehavior::reset() {
    phase_ = Entry;
    pitchTravel_ = rollTravel_ = timer_ = 0.0;
    failed_ = false;
}

Command AerobaticBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    timer_ += ctx.dt;
    // flown within the envelope, or not at all: too slow, departed (its angle of attack past the
    // envelope's by more than a limiter overshoots, either way), or too near the ground, it gives up and levels out
    const bool departed = std::isfinite(alphaMaxRad_) && std::abs(s.alphaRad) > alphaMaxRad_ + 0.0524;
    if ((phase_ == Pull || phase_ == Roll) &&
        ((std::isfinite(minCasMs_) && s.airspeedCalibratedMs < minCasMs_) || departed || s.altitudeAglM < 150.0)) {
        failed_ = true;
        phase_ = Done;
    }
    AccelerationCommand out;
    out.throttle = 1.0;
    switch (phase_) {
    case Pull: {
        out.loadFactorG = loadFactor_;
        out.rollRateRadS = 0.0;
        pitchTravel_ += std::abs(s.angularRateBodyRadS[1]) * ctx.dt;
        const double needed = manoeuvre_ == Loop ? 2.0 * units::kPi : units::kPi;
        if (pitchTravel_ >= needed - 0.05) {
            pitchTravel_ = 0.0;
            phase_ = manoeuvre_ == Immelmann ? Roll : Done;
        }
        return out;
    }
    case Roll: {
        out.loadFactorG = manoeuvre_ == SplitS ? 0.5 : 0.9;
        out.rollRateRadS = rollRate_;
        rollTravel_ += std::abs(s.angularRateBodyRadS[0]) * ctx.dt;
        const double needed = manoeuvre_ == AileronRoll ? 2.0 * units::kPi : units::kPi;
        if (rollTravel_ >= needed - 0.05) {
            rollTravel_ = 0.0;
            phase_ = manoeuvre_ == SplitS ? Pull : Done;
        }
        return out;
    }
    case Entry:
    case Done:
    default: {
        VelocityCommand level;
        level.airspeedMs = entrySpeed_;
        level.headingRad = manoeuvre_ == Immelmann || manoeuvre_ == SplitS ? geo::wrapTwoPi(entryHeading_ + units::kPi) : entryHeading_;
        level.verticalSpeedMs = std::clamp(0.2 * (entryAltitude_ - s.altitudeMslM), -8.0, 8.0);
        return level;
    }
    }
}

double LoiterBehavior::orbitRadiusM(const Performance& perf, double tasMs) noexcept {
    // the bank its circle needs, at most what guidance may use (80 % of the most) and what its heading
    // loop gives the carrot's 0.3 rad off its heading on the circle (0.6 rad ahead of it)
    double bank = 0.8 * (std::isfinite(perf.maxBankRad) ? perf.maxBankRad : 0.52);
    if (std::isfinite(perf.headingGain) && perf.headingGain > 0.0) {
        const double gain = std::isfinite(perf.headingReferenceTasMs) ? perf.headingGain * tasMs / perf.headingReferenceTasMs : perf.headingGain;
        bank = std::min(bank, 0.3 * gain);
    }
    return tasMs * tasMs / (kG * std::tan(std::max(bank, 0.05)));
}

Reason admitWaypoints(const BehaviorCommand& c, const sim::VehicleState& s, const Performance& perf, CommandResult& detail) {
    if (c.points.empty() || perf.hovers) return Reason::None; // a rotorcraft turns on the spot
    const double bank = std::isfinite(perf.maxBankRad) && perf.maxBankRad > 0.05 ? perf.maxBankRad : 0.52;
    const double airspeed = c.param("airspeed_ms", s.airspeedTrueMs);
    const bool loop = c.param("loop", 0.0) != 0.0;
    const std::size_t n = c.points.size();
    // where it arrives from, and the track it arrives along: the aircraft now, then each point from the one before
    double lat = s.latitudeRad, lon = s.longitudeRad, track = s.eulerRad[2];
    for (std::size_t k = 0; k < n + (loop && n > 1 ? 1 : 0); ++k) {
        const std::size_t i = k % n;
        const PositionCommand& p = c.points[i];
        const double speed = isHold(p.airspeedMs) ? airspeed : p.airspeedMs;
        const double radius = speed * speed / (kG * std::tan(bank)); // at its full bank
        if (std::isfinite(radius) && p.captureRadiusM < radius) {
            double north, east; // the point from where it arrives
            geo::localNorthEastM(lat, lon, p.latitudeRad, p.longitudeRad, north, east);
            const double cn = -radius * std::sin(track), ce = radius * std::cos(track); // the right turn's centre (the left's: minus)
            const double inside = std::min(std::hypot(north - cn, east - ce), std::hypot(north + cn, east + ce));
            if (inside < radius - p.captureRadiusM) {
                detail.index = static_cast<std::int16_t>(std::min<std::size_t>(i, 0x7FFF));
                detail.constraint = Constraint::MaxTurnRate;
                return Reason::InvalidWaypoint;
            }
        }
        if (geo::distanceM(lat, lon, p.latitudeRad, p.longitudeRad) > 1.0) track = geo::bearingRad(lat, lon, p.latitudeRad, p.longitudeRad);
        lat = p.latitudeRad, lon = p.longitudeRad;
    }
    return Reason::None;
}

Reason admitAerobatics(const BehaviorCommand& c, const sim::VehicleState& s, const Performance& perf, CommandResult& detail) {
    if (!std::isfinite(perf.minCasMs)) return Reason::None; // nothing known to judge by
    const int manoeuvre = static_cast<int>(c.param("manoeuvre", 1.0));
    const double n = std::max(c.param("load_factor_g", 3.5), 1.0);
    const bool overTheTop = manoeuvre == AerobaticBehavior::Loop || manoeuvre == AerobaticBehavior::Immelmann;
    const double needed = perf.minCasMs * (overTheTop ? 2.0 : std::sqrt(n));
    if (s.airspeedCalibratedMs < needed) {
        detail.constraint = Constraint::MinAirspeed;
        return Reason::PerformanceLimit;
    }
    if (manoeuvre == AerobaticBehavior::SplitS) { // half a loop down: 2 v^2 / (g (n - 1)), and room to spare
        const double v = s.airspeedTrueMs;
        const double drop = n > 1.05 ? 2.0 * v * v / (kG * (n - 1.0)) : std::numeric_limits<double>::infinity();
        if (s.altitudeAglM < drop + 300.0) {
            detail.constraint = Constraint::MinAltitude;
            return Reason::PerformanceLimit;
        }
    }
    return Reason::None;
}

// --- Registration ----------------------------------------------------------------

void registerBuiltinControllers(ControllerRegistry& r) {
    r.add("actuator", Level::Actuator, [] { return std::make_unique<ActuatorPassthrough>(); });
    r.add("pid_attitude", Level::Attitude, [] { return std::make_unique<AttitudeLoop>(); });
    r.add("pseudo_attitude", Level::Attitude, [] { return std::make_unique<PseudoAttitudeLoop>(); });
    r.add("pid_acceleration", Level::Acceleration, [] { return std::make_unique<AccelerationLoop>(); });
    r.add("pid_velocity", Level::Velocity, [] { return std::make_unique<VelocityLoop>(); });
    r.add("pid_position", Level::Position, [] { return std::make_unique<PositionLoop>(); });
    // What each behaviour tells a consumer (docs/control-architecture.md, 8.2):
    // its parameters (BehaviorCommand::params; NaN default = as at the start)
    // and the flight capabilities its output goes through.
    constexpr double now = kHold;
    auto p = [](const char* name, const char* unit, double def, double lo = -std::numeric_limits<double>::infinity(),
                double hi = std::numeric_limits<double>::infinity()) { return ParameterInfo{name, unit, lo, hi, def, true}; };
    auto traits = [](Persistence persistence, std::vector<ParameterInfo> params, std::vector<std::string> uses, bool target = false) {
        return BehaviorTraits{persistence, std::move(params), std::move(uses), target};
    };
    const std::string position = "fsim.flight.position", velocity = "fsim.flight.velocity", acceleration = "fsim.flight.acceleration";
    r.addBehavior("hold", [] { return std::make_unique<HoldBehavior>(); },
                  traits(Persistence::Persistent, {p("airspeed_ms", "m/s", now, 0.0), p("heading_deg", "deg", now), p("altitude_m", "m", now)}, {velocity}));
    auto waypoints = traits(Persistence::Terminating, {p("loop", "", 0.0, 0.0, 1.0), p("airspeed_ms", "m/s", now, 0.0)}, {position, velocity});
    waypoints.admit = admitWaypoints; // no point it cannot capture
    r.addBehavior("waypoints", [] { return std::make_unique<WaypointsBehavior>(); }, std::move(waypoints));
    r.addBehavior("loiter", [] { return std::make_unique<LoiterBehavior>(); },
                  traits(Persistence::Persistent, // a wing's adapter narrows the radius to at least 100 m; left out, from its turn
                         {p("lat_deg", "deg", now, -90.0, 90.0), p("lon_deg", "deg", now, -180.0, 180.0), p("radius_m", "m", now, 1.0),
                          p("altitude_m", "m", now), p("clockwise", "", 1.0, 0.0, 1.0), p("airspeed_ms", "m/s", now, 0.0)},
                         {position}));
    r.addBehavior("pursuit", [] { return std::make_unique<PursuitBehavior>(); },
                  traits(Persistence::Persistent,
                         {p("range_m", "m", 300.0, 0.0), p("lead_s", "s", 2.0, 0.0), p("min_airspeed_ms", "m/s", 30.0, 0.0), p("max_airspeed_ms", "m/s", 400.0, 0.0)},
                         {position, velocity}, true));
    r.addBehavior("evade", [] { return std::make_unique<EvadeBehavior>(); },
                  traits(Persistence::Persistent, {p("altitude_delta_m", "m", -300.0), p("airspeed_ms", "m/s", now, 0.0), p("floor_agl_m", "m", 150.0, 0.0)},
                         {velocity}, true));
    auto formation = traits(Persistence::Persistent,
                            {p("ahead_m", "m", -100.0), p("right_m", "m", 60.0), p("below_m", "m", 0.0), p("closure_gain", "1/s", now, 0.0)},
                            {velocity}, true);
    formation.mode = FlightMode::Formation; // A-GRA's FORMATION: a slot on a leader
    r.addBehavior("formation", [] { return std::make_unique<FormationBehavior>(); }, std::move(formation));
    auto aerobatic = traits(Persistence::Terminating,
                            {p("manoeuvre", "", 1.0, 0.0, 3.0), p("load_factor_g", "g", 3.5, 0.0), p("roll_rate_rad_s", "rad/s", 1.5, 0.0)},
                            {acceleration, velocity});
    aerobatic.features = kFeatureWingborne; // a loop pulled by load factor: a wing's (and an aerobatic type's: rule R10)
    aerobatic.admit = admitAerobatics;      // fast enough for it, and a split-S high enough
    aerobatic.withinEnvelope = true;        // and completed only if flown within the envelope
    r.addBehavior("aerobatics", [] { return std::make_unique<AerobaticBehavior>(); }, std::move(aerobatic));
    registerRotorControllers(r); // the rotorcraft's loops and "hover" (docs/rotorcraft.md, 3.6)
    registerGuidanceModes(r);    // the Vehicle Interface's modes: "hsa" (docs/vehicle-interface.md)
}

} // namespace fsim::control
