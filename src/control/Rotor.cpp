#include "fsim/RotorControllers.h"

#include "control/Registry.h"
#include "core/Geodesy.h"
#include "core/Units.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

/// As the fixed-wing loops' (Builtin.cpp): true when the loop missed a period
/// - another level flew meanwhile - so what it holds no longer describes the flight.
bool resumed(const ControlContext& ctx, double& lastTime) noexcept {
    const double t = ctx.state.simTime;
    const bool stale = lastTime >= 0.0 && (t - lastTime > 1.5 * ctx.dt || t < lastTime);
    lastTime = t;
    return stale;
}

} // namespace

double thrustLoadFactor(const sim::VehicleState& s) noexcept {
    // the body's acceleration along z - the change of w seen in the turning
    // body frame, plus (omega x v)_z - less gravity's share of it
    const double p = s.angularRateBodyRadS[0], q = s.angularRateBodyRadS[1];
    const double az = s.accelerationBodyMs2[2] + p * s.velocityBodyMs[1] - q * s.velocityBodyMs[0];
    const double gz = kG * std::cos(s.eulerRad[1]) * std::cos(s.eulerRad[0]);
    return (gz - az) / kG;
}

double RotorRateAxis::update(double demand, double rate, double dt) noexcept {
    if (!(std::abs(power) > 1e-9)) return trim;
    const double error = demand - rate;
    // the plant's own damping gives part of the bandwidth: the rest is proportional
    const double proportional = std::max(bandwidth - damping, 0.25 * bandwidth);
    const double accel = damping * demand + proportional * error;
    const double next = sum + integral * error * dt;
    const double u = trim + (accel + next) / power;
    if (u >= -1.0 && u <= 1.0) {
        sum = next;
        return u;
    }
    return std::clamp(trim + (accel + sum) / power, -1.0, 1.0); // at the stop: the integral waits
}

// --- RotorAllocation -------------------------------------------------------------

RotorAllocation::RotorAllocation() {
#define AXIS(a)                                                                                                      params_.add(#a ".power", &a.power), params_.add(#a ".damping", &a.damping), params_.add(#a ".bandwidth", &a.bandwidth),         params_.add(#a ".ki", &a.integral), params_.add(#a ".trim", &a.trim)
    AXIS(roll);
    AXIS(pitch);
    AXIS(yaw);
#undef AXIS
    params_.add("throttle.trim", &throttleTrim);
    params_.add("heave.power", &heavePower);
    params_.add("load_factor.ki", &loadFactorGain);
}

Command RotorAllocation::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AccelerationCommand>(in);
    const auto& s = ctx.sensed;
    if (resumed(ctx, lastTime_)) reset();
    const double* rate = s.angularRateBodyRadS;
    ActuatorCommand out;
    if (ctx.engaged & axisBit(Axis::Roll)) {
        out.aileron = roll.update(orHold(c.rollRateRadS, 0.0), rate[0], ctx.dt);
    } else {
        roll.reset(); // someone else flies it: start afresh when it comes back
        out.aileron = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Pitch)) {
        out.elevator = pitch.update(orHold(c.pitchRateRadS, 0.0), rate[1], ctx.dt);
    } else {
        pitch.reset();
        out.elevator = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        out.rudder = yaw.update(orHold(c.yawRateRadS, 0.0), rate[2], ctx.dt);
    } else {
        yaw.reset();
        out.rudder = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Thrust)) {
        if (!isHold(c.loadFactorG) && std::abs(heavePower) > 1e-9) {
            // the throttle per g of thrust, from the heave's response; the integral trims what the feedforward misses
            const double perG = kG / heavePower;
            const double feed = throttleTrim + (c.loadFactorG - 1.0) * perG;
            const double next = loadSum_ + loadFactorGain * (c.loadFactorG - thrustLoadFactor(s)) * perG * ctx.dt;
            const double u = feed + next;
            if (u >= 0.0 && u <= 1.0) {
                loadSum_ = next;
                out.throttle = u;
            } else {
                out.throttle = std::clamp(feed + loadSum_, 0.0, 1.0);
            }
        } else {
            out.throttle = c.throttle; // may be kHold: the stack keeps the last value
            loadSum_ = 0.0;
        }
    } else {
        loadSum_ = 0.0;
        out.throttle = kHold;
    }
    return out;
}

void RotorAllocation::reset() {
    roll.reset();
    pitch.reset();
    yaw.reset();
    loadSum_ = 0.0;
}

// --- RotorAttitude ---------------------------------------------------------------

RotorAttitude::RotorAttitude() {
    params_.add("roll.gain", &rollGain);
    params_.add("pitch.gain", &pitchGain);
    params_.add("yaw.gain", &yawGain);
    params_.add("roll.max_rate", &maxRollRateRadS);
    params_.add("pitch.max_rate", &maxPitchRateRadS);
    params_.add("yaw.max_rate", &maxYawRateRadS);
}

Command RotorAttitude::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AttitudeCommand>(in);
    const auto& s = ctx.sensed;
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    if (!haveRef_ || resumed(ctx, lastTime_)) {
        headingRef_ = psi;
        pitchRef_ = theta;
        haveRef_ = true;
    }
    AccelerationCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    const bool cyclic = (ctx.engaged & kCyclic) != 0, yaw = (ctx.engaged & axisBit(Axis::Yaw)) != 0;
    // the rates the Euler angles are to change at
    double phiDot = 0.0, thetaDot = 0.0, psiDot = 0.0;
    if (cyclic) {
        if (!isHold(c.pitchRad)) pitchRef_ = c.pitchRad;
        phiDot = std::clamp(rollGain * geo::wrapPi(orHold(c.rollRad, 0.0) - phi), -maxRollRateRadS, maxRollRateRadS);
        thetaDot = std::clamp(pitchGain * (pitchRef_ - theta), -maxPitchRateRadS, maxPitchRateRadS);
    } else {
        pitchRef_ = theta;
    }
    if (yaw) {
        if (!isHold(c.headingRad)) headingRef_ = c.headingRad;
        psiDot = std::clamp(yawGain * geo::wrapPi(headingRef_ - psi), -maxYawRateRadS, maxYawRateRadS);
    } else {
        headingRef_ = psi;
    }
    // ... as body rates
    const double sphi = std::sin(phi), cphi = std::cos(phi), sth = std::sin(theta), cth = std::cos(theta);
    if (cyclic) {
        out.rollRateRadS = phiDot - psiDot * sth;
        out.pitchRateRadS = thetaDot * cphi + psiDot * sphi * cth;
    }
    if (yaw) out.yawRateRadS = -thetaDot * sphi + psiDot * cphi * cth;
    if (ctx.engaged & axisBit(Axis::Thrust)) out.throttle = c.throttle; // may be kHold: the stack keeps the last value
    return out;
}

void RotorAttitude::reset() {
    haveRef_ = false;
    lastTime_ = -1.0;
}

// --- RotorVelocity ---------------------------------------------------------------

RotorVelocity::RotorVelocity() {
    params_.add("horizontal.kp", &horizontalKp);
    params_.add("horizontal.ki", &horizontalKi);
    params_.add("max_tilt", &maxTiltRad);
    params_.add("vertical.kp", &verticalKp);
    params_.add("vertical.ki", &verticalKi);
    params_.add("throttle.trim", &throttleTrim);
    params_.add("heave.power", &heavePower);
    params_.add("heave.damping", &heaveDamping);
    params_.add("roll.trim", &rollTrimRad);
    params_.add("pitch.trim", &pitchTrimRad);
}

Command RotorVelocity::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<VelocityCommand>(in);
    const auto& s = ctx.sensed;
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    if (!started_ || resumed(ctx, lastTime_)) {
        northHold_ = s.velocityNedMs[0];
        eastHold_ = s.velocityNedMs[1];
        sumX_ = sumY_ = sumZ_ = 0.0;
        headingRef_ = psi;
        started_ = true;
    }
    AttitudeCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    const double dt = ctx.dt;

    // over the ground or through the air: the velocity error along the heading and to its right
    if (ctx.engaged & kCyclic) {
        double ex, ey;
        const bool ground = !isHold(c.northMs) && !isHold(c.eastMs);
        if (!ground && !isHold(c.airspeedMs)) {
            const double v = s.airspeedTrueMs;
            ex = c.airspeedMs - v * std::cos(s.alphaRad) * std::cos(s.betaRad);
            ey = -v * std::sin(s.betaRad);
            northHold_ = s.velocityNedMs[0], eastHold_ = s.velocityNedMs[1]; // a hold after it keeps what this flew
        } else {
            if (ground) northHold_ = c.northMs, eastHold_ = c.eastMs;
            const double en = northHold_ - s.velocityNedMs[0], ee = eastHold_ - s.velocityNedMs[1];
            ex = en * std::cos(psi) + ee * std::sin(psi);
            ey = -en * std::sin(psi) + ee * std::cos(psi);
        }
        // the acceleration that closes it, and the tilt that makes it (nose down to go forward, bank to go right);
        // the integral trims near the target and slows far from it - through a change of speed it would wind up and
        // overshoot: its gain falls off as band^2 / (band^2 + error^2), band the error the proportional term answers
        // with a quarter of the tilt
        const double aMax = kG * std::tan(maxTiltRad);
        const double band = horizontalKp > 0.0 ? 0.25 * aMax / horizontalKp : 1.0, band2 = band * band;
        sumX_ = std::clamp(sumX_ + horizontalKi * band2 / (band2 + ex * ex) * ex * dt, -0.5 * aMax, 0.5 * aMax);
        sumY_ = std::clamp(sumY_ + horizontalKi * band2 / (band2 + ey * ey) * ey * dt, -0.5 * aMax, 0.5 * aMax);
        const double ax = std::clamp(horizontalKp * ex + sumX_, -aMax, aMax);
        const double ay = std::clamp(horizontalKp * ey + sumY_, -aMax, aMax);
        out.pitchRad = pitchTrimRad - std::atan(ax / kG);
        out.rollRad = rollTrimRad + std::atan(ay * std::cos(out.pitchRad) / kG);
    } else {
        sumX_ = sumY_ = 0.0;
        northHold_ = s.velocityNedMs[0], eastHold_ = s.velocityNedMs[1];
    }

    // the vertical speed on the throttle: the hover's, what the tilt takes from the thrust, the damping's share, the loop
    if ((ctx.engaged & axisBit(Axis::Thrust)) && std::abs(heavePower) > 1e-9) {
        const double target = orHold(c.verticalSpeedMs, 0.0);
        const double error = target - (-s.velocityNedMs[2]);
        const double tilt = std::max(std::cos(phi) * std::cos(theta), 0.5);
        const double feed = throttleTrim + ((1.0 / tilt - 1.0) * kG + heaveDamping * target) / heavePower;
        const double next = sumZ_ + verticalKi * error * dt;
        const double u = feed + (verticalKp * error + next) / heavePower;
        if (u >= 0.0 && u <= 1.0) {
            sumZ_ = next;
            out.throttle = u;
        } else {
            out.throttle = std::clamp(feed + (verticalKp * error + sumZ_) / heavePower, 0.0, 1.0);
        }
    } else {
        sumZ_ = 0.0;
    }

    // the heading: given, or a turn rate's (kept within a radian of the nose, so it never laps it)
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        if (!isHold(c.headingRad)) headingRef_ = c.headingRad;
        else if (!isHold(c.turnRateRadS)) headingRef_ = psi + std::clamp(geo::wrapPi(headingRef_ + c.turnRateRadS * dt - psi), -1.0, 1.0);
        out.headingRad = headingRef_;
    } else {
        headingRef_ = psi;
    }
    return out;
}

void RotorVelocity::reset() {
    started_ = false;
    lastTime_ = -1.0;
    sumX_ = sumY_ = sumZ_ = 0.0;
}

// --- RotorPosition ---------------------------------------------------------------

RotorPosition::RotorPosition() {
    params_.add("horizontal.gain", &horizontalGain);
    params_.add("max_speed", &maxSpeedMs);
    params_.add("deceleration", &decelerationMs2);
    params_.add("velocity.lag_s", &velocityLagS);
    params_.add("altitude.gain", &altitudeGain);
    params_.add("max_vertical_speed", &maxVerticalSpeedMs);
}

Command RotorPosition::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<PositionCommand>(in);
    const auto& s = ctx.sensed;
    VelocityCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    double north, east; // the point, from here
    geo::localNorthEastM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad, north, east);
    const double distance = std::hypot(north, east);
    if (ctx.engaged & kCyclic) {
        // as fast as it may go, and no faster than it can stop from - the velocity loop's lag
        // first, then the deceleration (v tau + v^2 / 2a = distance): arriving, it slows to a halt
        const double a = decelerationMs2, tau = velocityLagS;
        const double stopping = a * (std::sqrt(tau * tau + 2.0 * distance / a) - tau);
        const double speed = std::min({orHold(c.airspeedMs, maxSpeedMs), horizontalGain * distance, stopping});
        out.northMs = distance > 1e-6 ? speed * north / distance : 0.0;
        out.eastMs = distance > 1e-6 ? speed * east / distance : 0.0;
    }
    if (ctx.engaged & axisBit(Axis::Thrust))
        out.verticalSpeedMs = std::clamp(altitudeGain * (c.altitudeMslM - s.altitudeMslM), -maxVerticalSpeedMs, maxVerticalSpeedMs);
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        if (!isHold(c.headingRad)) out.headingRad = c.headingRad;
        else if (distance > std::max(20.0, c.captureRadiusM)) out.headingRad = std::atan2(east, north);
        // near it and without a heading: kHold, the heading it has
    }
    return out;
}

// --- HoverBehavior ---------------------------------------------------------------

void HoverBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    target_ = PositionCommand{};
    target_.latitudeRad = command.param("lat_deg", units::radiansToDegrees(s.latitudeRad)) * units::kDegreesToRadians;
    target_.longitudeRad = command.param("lon_deg", units::radiansToDegrees(s.longitudeRad)) * units::kDegreesToRadians;
    target_.altitudeMslM = command.param("altitude_m", s.altitudeMslM);
    target_.headingRad = command.param("heading_deg", units::radiansToDegrees(s.eulerRad[2])) * units::kDegreesToRadians;
    target_.airspeedMs = kHold;
    target_.captureRadiusM = 1.0;
}

Command HoverBehavior::update(const ControlContext&, const Command&) { return target_; }

// --- Registration ----------------------------------------------------------------

void registerRotorControllers(ControllerRegistry& r) {
    r.add("rotor_allocation", Level::Acceleration, [] { return std::make_unique<RotorAllocation>(); });
    r.add("rotor_attitude", Level::Attitude, [] { return std::make_unique<RotorAttitude>(); });
    r.add("rotor_velocity", Level::Velocity, [] { return std::make_unique<RotorVelocity>(); });
    r.add("rotor_position", Level::Position, [] { return std::make_unique<RotorPosition>(); });
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi) { return ParameterInfo{name, unit, lo, hi, def, true}; };
    BehaviorTraits hover{Persistence::Persistent,
                         {p("lat_deg", "deg", now, -90.0, 90.0), p("lon_deg", "deg", now, -180.0, 180.0), p("altitude_m", "m", now, -inf, inf),
                          p("heading_deg", "deg", now, -inf, inf)},
                         {"fsim.flight.position"},
                         false,
                         kFeatureHover};
    r.addBehavior("hover", [] { return std::make_unique<HoverBehavior>(); }, std::move(hover));
}

} // namespace fsim::control
