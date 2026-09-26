#include "control/Builtin.h"

#include "control/Registry.h"
#include "core/Geodesy.h"
#include "core/Units.h"

#include <algorithm>
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

    ActuatorCommand out;
    out.aileron = roll.update(geo::wrapPi(rollRef_ - rollNow), s.angularRateBodyRadS[0], ctx.dt,
                              schedule.factor(s, rollEasExponent, rollTasExponent));
    // Positive elevator is nose-down in JSBSim: negate the nose-up demand. The
    // trim is the elevator a level turn at this bank holds.
    const double turn = 1.0 / std::max(std::cos(rollNow), 0.3);
    out.elevator = schedule.trimElevator(s, turn, pitchTrim, pitchTrimLift) -
                   pitch.update(orHold(c.pitchRad, pitchNow) - pitchNow, s.angularRateBodyRadS[1], ctx.dt,
                                schedule.factor(s, pitchEasExponent, pitchTasExponent));
    out.rudder = clamp11(rudderBetaGain * s.betaRad);
    if (!isHold(c.airspeedMs))
        out.throttle = std::clamp(throttleFeedforward + airspeed.update(c.airspeedMs - s.airspeedTrueMs, 0.0, ctx.dt), 0.0, 1.0);
    else
        out.throttle = c.throttle; // may be kHold: the stack keeps the last value
    return out;
}

void AttitudeLoop::reset() {
    roll.reset();
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
    out.elevator = schedule.trimElevator(s, n, pitchTrim, pitchTrimLift) -
                   (sn * loadFactorFeedforward * (n - level) + loadFactor.update(n - s.loadFactor, s.angularRateBodyRadS[1], ctx.dt, sn));
    out.aileron = sp * rollRateFeedforward * p + rollRate.update(p - s.angularRateBodyRadS[0], 0.0, ctx.dt, sp);
    out.rudder = clamp11(rudderBetaGain * s.betaRad);
    if (!isHold(c.longitudinalMs2))
        out.throttle = std::clamp(throttleFeedforward + longitudinal.update(c.longitudinalMs2 - s.accelerationBodyMs2[0], 0.0, ctx.dt), 0.0, 1.0);
    else
        out.throttle = c.throttle;
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
    out.maxBankRad = maxBankRad;
    if (!isHold(c.turnRateRadS)) {
        const double v = std::max(s.airspeedTrueMs, 10.0);
        out.rollRad = std::clamp(std::atan(c.turnRateRadS * v / kG), -maxBankRad, maxBankRad);
    } else if (!isHold(c.headingRad)) {
        out.headingRad = c.headingRad;
    } else {
        out.rollRad = 0.0;
    }
    out.airspeedMs = c.airspeedMs;
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
    const double distance = geo::distanceM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
    out.headingRad = distance < c.captureRadiusM ? s.eulerRad[2] : geo::bearingRad(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
    out.verticalSpeedMs = std::clamp(altitudeGain * (c.altitudeMslM - s.altitudeMslM), -maxVerticalSpeedMs, maxVerticalSpeedMs);
    out.airspeedMs = c.airspeedMs;
    return out;
}

// --- Behaviours -----------------------------------------------------------------

void HoldBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    target_.airspeedMs = command.param("airspeed_ms", s.airspeedTrueMs);
    target_.verticalSpeedMs = 0.0;
    target_.headingRad = command.param("heading_deg", units::radiansToDegrees(s.eulerRad[2])) * units::kDegreesToRadians;
    altitude_ = command.param("altitude_m", s.altitudeMslM);
}

Command HoldBehavior::update(const ControlContext& ctx, const Command&) {
    VelocityCommand out = target_;
    out.verticalSpeedMs = std::clamp(0.25 * (altitude_ - ctx.sensed.altitudeMslM), -6.0, 6.0);
    return out;
}

void WaypointsBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    index_ = 0;
    loop_ = command.param("loop", 0.0) != 0.0;
    finished_ = command.points.empty();
    airspeed_ = command.param("airspeed_ms", ctx.sensed.airspeedTrueMs);
}

Command WaypointsBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<BehaviorCommand>(in);
    const auto& s = ctx.sensed;
    if (c.points.empty()) {
        VelocityCommand hold;
        hold.headingRad = s.eulerRad[2];
        hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    if (index_ >= c.points.size()) index_ = c.points.size() - 1;
    const PositionCommand& p = c.points[index_];
    if (!finished_ && geo::distanceM(s.latitudeRad, s.longitudeRad, p.latitudeRad, p.longitudeRad) < p.captureRadiusM) {
        if (index_ + 1 < c.points.size()) ++index_;
        else if (loop_) index_ = 0;
        else finished_ = true;
    }
    PositionCommand out = c.points[index_];
    if (isHold(out.airspeedMs)) out.airspeedMs = airspeed_;
    return out;
}

void LoiterBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    target_ = command.target;
    centreLat_ = command.param("lat_deg", units::radiansToDegrees(s.latitudeRad)) * units::kDegreesToRadians;
    centreLon_ = command.param("lon_deg", units::radiansToDegrees(s.longitudeRad)) * units::kDegreesToRadians;
    radius_ = std::max(100.0, command.param("radius_m", 1500.0));
    altitude_ = command.param("altitude_m", s.altitudeMslM);
    clockwise_ = command.param("clockwise", 1.0) != 0.0;
    airspeed_ = command.param("airspeed_ms", s.airspeedTrueMs);
}

Command LoiterBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    double cLat = centreLat_, cLon = centreLon_;
    if (target_ && ctx.world)
        if (const auto* t = ctx.world->vehicleState(target_)) {
            cLat = t->latitudeRad;
            cLon = t->longitudeRad;
        }
    double north, east;
    geo::localNorthEastM(cLat, cLon, s.latitudeRad, s.longitudeRad, north, east);
    const double distance = std::hypot(north, east);
    const double theta = std::atan2(east, north);
    // Carrot on the circle, ahead of the vehicle's current angular position;
    // far away the carrot leads less so the approach is nearly direct.
    const double lead = distance > 2.0 * radius_ ? 0.15 : 0.6;
    const double aim = theta + (clockwise_ ? lead : -lead);
    PositionCommand out;
    geo::offsetLatLon(cLat, cLon, radius_ * std::cos(aim), radius_ * std::sin(aim), out.latitudeRad, out.longitudeRad);
    out.altitudeMslM = altitude_;
    out.airspeedMs = airspeed_;
    out.captureRadiusM = 30.0;
    return out;
}

void PursuitBehavior::start(const ControlContext&, const BehaviorCommand& command) {
    target_ = command.target;
    rangeM_ = command.param("range_m", 300.0);
    leadS_ = command.param("lead_s", 2.0);
    minSpeed_ = command.param("min_airspeed_ms", 30.0);
    maxSpeed_ = command.param("max_airspeed_ms", 400.0);
}

Command PursuitBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    if (!t) {
        VelocityCommand hold;
        hold.headingRad = s.eulerRad[2];
        hold.airspeedMs = s.airspeedTrueMs;
        return hold;
    }
    PositionCommand out;
    geo::offsetLatLon(t->latitudeRad, t->longitudeRad, t->velocityNedMs[0] * leadS_, t->velocityNedMs[1] * leadS_, out.latitudeRad, out.longitudeRad);
    out.altitudeMslM = t->altitudeMslM - t->velocityNedMs[2] * leadS_;
    const double distance = geo::distanceM(s.latitudeRad, s.longitudeRad, t->latitudeRad, t->longitudeRad);
    const double targetSpeed = std::hypot(t->velocityNedMs[0], t->velocityNedMs[1]);
    out.airspeedMs = std::clamp(targetSpeed + 0.1 * (distance - rangeM_), minSpeed_, maxSpeed_);
    out.captureRadiusM = 20.0;
    return out;
}

void EvadeBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    target_ = command.target;
    altitude_ = ctx.sensed.altitudeMslM + command.param("altitude_delta_m", -300.0);
    airspeed_ = command.param("airspeed_ms", ctx.sensed.airspeedTrueMs);
}

Command EvadeBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    VelocityCommand out;
    out.airspeedMs = airspeed_;
    out.verticalSpeedMs = std::clamp(0.25 * (altitude_ - s.altitudeMslM), -8.0, 8.0);
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    out.headingRad = t ? geo::bearingRad(t->latitudeRad, t->longitudeRad, s.latitudeRad, s.longitudeRad) : s.eulerRad[2];
    return out;
}

void FormationBehavior::start(const ControlContext&, const BehaviorCommand& command) {
    target_ = command.target;
    ahead_ = command.param("ahead_m", -100.0);
    right_ = command.param("right_m", 60.0);
    below_ = command.param("below_m", 0.0);
    closureGain_ = command.param("closure_gain", 0.1);
}

Command FormationBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    const sim::VehicleState* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
    VelocityCommand out;
    if (!t) {
        out.headingRad = s.eulerRad[2];
        out.airspeedMs = s.airspeedTrueMs;
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
    const double along = errNorth * cs + errEast * sn;   // + = slot is ahead of me
    const double cross = -errNorth * sn + errEast * cs;  // + = slot is to my right
    const double leaderSpeed = std::hypot(t->velocityNedMs[0], t->velocityNedMs[1]);
    out.headingRad = psi + std::clamp(0.01 * cross, -0.6, 0.6);
    out.airspeedMs = std::clamp(leaderSpeed + closureGain_ * along, 0.5 * leaderSpeed, 1.5 * leaderSpeed + 5.0);
    const double slotAltitude = t->altitudeMslM - below_;
    out.verticalSpeedMs = std::clamp(0.3 * (slotAltitude - s.altitudeMslM) - t->velocityNedMs[2], -8.0, 8.0);
    return out;
}

void AerobaticBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    manoeuvre_ = static_cast<Manoeuvre>(static_cast<int>(command.param("manoeuvre", 1.0)));
    loadFactor_ = command.param("load_factor_g", 3.5);
    rollRate_ = command.param("roll_rate_rad_s", 1.5);
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
}

Command AerobaticBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    timer_ += ctx.dt;
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

// --- Registration ----------------------------------------------------------------

void registerBuiltinControllers(ControllerRegistry& r) {
    r.add("actuator", Level::Actuator, [] { return std::make_unique<ActuatorPassthrough>(); });
    r.add("pid_attitude", Level::Attitude, [] { return std::make_unique<AttitudeLoop>(); });
    r.add("pid_acceleration", Level::Acceleration, [] { return std::make_unique<AccelerationLoop>(); });
    r.add("pid_velocity", Level::Velocity, [] { return std::make_unique<VelocityLoop>(); });
    r.add("pid_position", Level::Position, [] { return std::make_unique<PositionLoop>(); });
    r.addBehavior("hold", [] { return std::make_unique<HoldBehavior>(); });
    r.addBehavior("waypoints", [] { return std::make_unique<WaypointsBehavior>(); });
    r.addBehavior("loiter", [] { return std::make_unique<LoiterBehavior>(); });
    r.addBehavior("pursuit", [] { return std::make_unique<PursuitBehavior>(); });
    r.addBehavior("evade", [] { return std::make_unique<EvadeBehavior>(); });
    r.addBehavior("formation", [] { return std::make_unique<FormationBehavior>(); });
    r.addBehavior("aerobatics", [] { return std::make_unique<AerobaticBehavior>(); });
}

} // namespace fsim::control
