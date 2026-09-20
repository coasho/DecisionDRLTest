#pragma once

// Built-in effects (design 9.5). Small and explicit; researchers add their
// own by subclassing effects::Effect.

#include "fsim/Effects.h"
#include "fsim/Export.h"

#include <deque>

namespace fsim::effects {

/// Additive white noise on the sensed state (sigmas; 0 = untouched).
class FSIM_API GaussianSensorNoise final : public Effect {
public:
    const char* id() const noexcept override { return "gaussian_sensor_noise"; }
    void apply(EffectContext& ctx) override;

    double positionSigmaM = 3.0;
    double altitudeSigmaM = 2.0;
    double velocitySigmaMs = 0.3;
    double attitudeSigmaRad = 0.005;
    double rateSigmaRadS = 0.01;
    double airspeedSigmaMs = 0.5;
};

/// Sensors report the state as it was `delaySteps` FDM steps ago.
class FSIM_API SensorLatency final : public Effect {
public:
    explicit SensorLatency(unsigned delaySteps = 6) : delaySteps_(delaySteps) {}
    const char* id() const noexcept override { return "sensor_latency"; }
    void apply(EffectContext& ctx) override;
    void onReset() override { history_.clear(); }

private:
    unsigned delaySteps_;
    std::deque<sim::VehicleState> history_;
};

/// A constant body-frame force and moment (e.g. a towed load, damage asymmetry).
class FSIM_API ConstantForce final : public Effect {
public:
    const char* id() const noexcept override { return "constant_force"; }
    void apply(EffectContext& ctx) override { ctx.addForceBody(forceN[0], forceN[1], forceN[2]); ctx.addMomentBody(momentNm[0], momentNm[1], momentNm[2]); }
    double forceN[3] = {0, 0, 0};
    double momentNm[3] = {0, 0, 0};
};

/// Random discrete gusts on top of the global wind: each gust ramps up, holds
/// and fades (1 - cosine), with random direction and strength.
class FSIM_API WindGusts final : public Effect {
public:
    const char* id() const noexcept override { return "wind_gusts"; }
    void apply(EffectContext& ctx) override;
    void onReset() override { active_ = false; nextAt_ = -1.0; }

    double meanIntervalS = 20.0;
    double peakMs = 6.0;      ///< maximum gust magnitude
    double durationS = 4.0;

private:
    bool active_ = false;
    double nextAt_ = -1.0, startAt_ = 0.0, magnitude_ = 0.0, direction_[3] = {1, 0, 0};
};

/// GNSS degradation: with `lossProbabilityPerS` the fix drops for `outageS`;
/// while valid, a slow random-walk position bias up to `driftM`.
class FSIM_API GnssDegradation final : public Effect {
public:
    const char* id() const noexcept override { return "gnss_degradation"; }
    void apply(EffectContext& ctx) override;
    void onReset() override { outageUntil_ = -1.0; bias_[0] = bias_[1] = 0.0; }

    double lossProbabilityPerS = 0.01;
    double outageS = 5.0;
    double driftM = 10.0;

private:
    double outageUntil_ = -1.0;
    double bias_[2] = {0, 0};
};

} // namespace fsim::effects
