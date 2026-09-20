#include "effects/Builtin.h"

#include "core/Geodesy.h"
#include "core/Units.h"

#include <algorithm>
#include <cmath>

namespace fsim::effects {

void GaussianSensorNoise::apply(EffectContext& ctx) {
    auto& s = ctx.sensed.state;
    Rng& rng = ctx.rng;
    if (positionSigmaM > 0.0) {
        const double north = rng.normal() * positionSigmaM, east = rng.normal() * positionSigmaM;
        geo::offsetLatLon(s.latitudeRad, s.longitudeRad, north, east, s.latitudeRad, s.longitudeRad);
        ctx.sensed.positionErrorM = std::hypot(north, east);
    }
    if (altitudeSigmaM > 0.0) {
        const double dz = rng.normal() * altitudeSigmaM;
        s.altitudeMslM += dz;
        s.altitudeAglM += dz;
    }
    if (velocitySigmaMs > 0.0)
        for (double& v : s.velocityNedMs) v += rng.normal() * velocitySigmaMs;
    if (attitudeSigmaRad > 0.0)
        for (double& e : s.eulerRad) e += rng.normal() * attitudeSigmaRad;
    if (rateSigmaRadS > 0.0)
        for (double& r : s.angularRateBodyRadS) r += rng.normal() * rateSigmaRadS;
    if (airspeedSigmaMs > 0.0) s.airspeedTrueMs = std::max(0.0, s.airspeedTrueMs + rng.normal() * airspeedSigmaMs);
}

void SensorLatency::apply(EffectContext& ctx) {
    history_.push_back(ctx.state);
    while (history_.size() > delaySteps_ + 1) history_.pop_front();
    ctx.sensed.state = history_.front();
}

void WindGusts::apply(EffectContext& ctx) {
    const double t = ctx.simTime;
    if (nextAt_ < 0.0) nextAt_ = t + ctx.rng.uniform(0.0, meanIntervalS);
    if (!active_ && t >= nextAt_) {
        active_ = true;
        startAt_ = t;
        magnitude_ = ctx.rng.uniform(0.3, 1.0) * peakMs;
        const double az = ctx.rng.uniform(0.0, 2.0 * units::kPi), el = ctx.rng.uniform(-0.3, 0.3);
        direction_[0] = std::cos(el) * std::cos(az);
        direction_[1] = std::cos(el) * std::sin(az);
        direction_[2] = std::sin(el);
        nextAt_ = t + durationS + ctx.rng.uniform(0.5, 1.5) * meanIntervalS;
    }
    if (!active_) return;
    const double phase = (t - startAt_) / durationS;
    if (phase >= 1.0) {
        active_ = false;
        return;
    }
    const double g = magnitude_ * 0.5 * (1.0 - std::cos(2.0 * units::kPi * phase)); // 1 - cosine envelope
    ctx.addWindNed(g * direction_[0], g * direction_[1], g * direction_[2]);
}

void GnssDegradation::apply(EffectContext& ctx) {
    const double t = ctx.simTime;
    if (t < outageUntil_) {
        ctx.sensed.gnssValid = false;
        return;
    }
    if (ctx.rng.uniform() < lossProbabilityPerS * ctx.dt) {
        outageUntil_ = t + outageS;
        ctx.sensed.gnssValid = false;
        return;
    }
    // Bounded random walk of the fix.
    const double step = driftM * 0.05 * std::sqrt(std::max(ctx.dt, 1e-6));
    for (double& b : bias_) b = std::clamp(b + ctx.rng.normal() * step, -driftM, driftM);
    auto& s = ctx.sensed.state;
    geo::offsetLatLon(s.latitudeRad, s.longitudeRad, bias_[0], bias_[1], s.latitudeRad, s.longitudeRad);
    ctx.sensed.positionErrorM = std::hypot(bias_[0], bias_[1]);
}

} // namespace fsim::effects
