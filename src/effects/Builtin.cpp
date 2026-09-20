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

std::unique_ptr<Effect> createBuiltinEffect(std::string_view id, const std::vector<std::pair<std::string, double>>& params) {
    auto get = [&](const char* name, double& target) {
        for (const auto& [k, v] : params)
            if (k == name) target = v;
    };
    if (id == "gaussian_sensor_noise") {
        auto e = std::make_unique<GaussianSensorNoise>();
        get("position_sigma_m", e->positionSigmaM); get("altitude_sigma_m", e->altitudeSigmaM); get("velocity_sigma_ms", e->velocitySigmaMs);
        get("attitude_sigma_rad", e->attitudeSigmaRad); get("rate_sigma_rad_s", e->rateSigmaRadS); get("airspeed_sigma_ms", e->airspeedSigmaMs);
        return e;
    }
    if (id == "sensor_latency") {
        double steps = 6.0;
        get("delay_steps", steps);
        return std::make_unique<SensorLatency>(static_cast<unsigned>(std::max(0.0, steps)));
    }
    if (id == "constant_force") {
        auto e = std::make_unique<ConstantForce>();
        get("force_n_x", e->forceN[0]); get("force_n_y", e->forceN[1]); get("force_n_z", e->forceN[2]);
        get("moment_nm_l", e->momentNm[0]); get("moment_nm_m", e->momentNm[1]); get("moment_nm_n", e->momentNm[2]);
        return e;
    }
    if (id == "wind_gusts") {
        auto e = std::make_unique<WindGusts>();
        get("mean_interval_s", e->meanIntervalS); get("peak_ms", e->peakMs); get("duration_s", e->durationS);
        return e;
    }
    if (id == "gnss_degradation") {
        auto e = std::make_unique<GnssDegradation>();
        get("loss_probability_per_s", e->lossProbabilityPerS); get("outage_s", e->outageS); get("drift_m", e->driftM);
        return e;
    }
    return nullptr;
}

} // namespace fsim::effects
