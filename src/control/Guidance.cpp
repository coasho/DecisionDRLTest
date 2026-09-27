#include "fsim/GuidanceModes.h"

#include "control/Atmosphere.h"
#include "control/Registry.h"
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
}

} // namespace fsim::control
