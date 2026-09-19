#include "world/Interpolator.h"

#include "sim/Attitude.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {

using sim::attitude::bodyToEcefFromQuaternion;
using sim::attitude::slerp;

double lerp(double a, double b, double t) { return a + (b - a) * t; }

} // namespace

Interpolator::Interpolator(std::size_t vehicles) : vehicles_(vehicles), out_(vehicles) {}

void Interpolator::push(const sim::SnapshotBatch& batch) {
    if (batch.states.size() != vehicles_) return;
    if (!ring_.empty() && batch.simTime <= ring_.back().simTime) {
        // Same or older sim time (paused / duplicate): refresh the newest sample only.
        ring_.back().states = batch.states;
        return;
    }
    if (!ring_.empty()) {
        const double spacing = batch.simTime - ring_.back().simTime;
        if (spacing > 1e-6 && spacing < 1.0) stepSeconds_ = 0.9 * stepSeconds_ + 0.1 * spacing;
    }
    ring_.push_back(Sample{batch.simTime, batch.states});
    while (ring_.size() > kRing) ring_.pop_front();
    if (!clockStarted_) {
        renderTime_ = batch.simTime - delaySteps_ * stepSeconds_;
        clockStarted_ = true;
    }
}

bool Interpolator::update(double frameSeconds, double timeFactor, bool paused) {
    if (ring_.empty()) return false;
    if (ring_.size() == 1) {
        out_ = ring_.back().states;
        return false;
    }

    // Advance the render clock and keep it `delaySteps_` behind the newest snapshot.
    const double newest = ring_.back().simTime;
    if (!paused) renderTime_ += frameSeconds * timeFactor;
    const double target = newest - delaySteps_ * stepSeconds_;
    const double err = target - renderTime_;
    if (std::abs(err) > 4.0 * stepSeconds_) renderTime_ = target; // far off (stall, time-factor jump): snap
    else renderTime_ += err * 0.05;                                 // else pull gently: no visible speed change

    // Find the bracketing pair.
    if (renderTime_ <= ring_.front().simTime) {
        out_ = ring_.front().states;
        return true;
    }
    if (renderTime_ >= newest) {
        const double dt = std::min(renderTime_ - newest, 3.0 * stepSeconds_); // bounded extrapolation
        const auto& b = ring_.back().states;
        for (std::size_t i = 0; i < vehicles_; ++i) extrapolate(b[i], dt, out_[i]);
        return true;
    }
    std::size_t hi = 1;
    while (hi < ring_.size() && ring_[hi].simTime < renderTime_) ++hi;
    const Sample& a = ring_[hi - 1];
    const Sample& b = ring_[hi];
    const double span = b.simTime - a.simTime;
    const double t = span > 0.0 ? std::clamp((renderTime_ - a.simTime) / span, 0.0, 1.0) : 1.0;
    for (std::size_t i = 0; i < vehicles_; ++i) blend(a.states[i], b.states[i], t, out_[i]);
    return true;
}

void Interpolator::blend(const sim::VehicleState& a, const sim::VehicleState& b, double t, sim::VehicleState& out) {
    out = b; // discrete fields (flags, counts, engine data) from the newer sample
    for (int i = 0; i < 3; ++i) {
        out.positionEcef[i] = lerp(a.positionEcef[i], b.positionEcef[i], t);
        out.velocityBodyMs[i] = lerp(a.velocityBodyMs[i], b.velocityBodyMs[i], t);
        out.velocityNedMs[i] = lerp(a.velocityNedMs[i], b.velocityNedMs[i], t);
        out.angularRateBodyRadS[i] = lerp(a.angularRateBodyRadS[i], b.angularRateBodyRadS[i], t);
        out.eulerRad[i] = lerp(a.eulerRad[i], b.eulerRad[i], t); // display only; wraps at +-pi are rare and brief
    }
    out.simTime = lerp(a.simTime, b.simTime, t);
    out.altitudeMslM = lerp(a.altitudeMslM, b.altitudeMslM, t);
    out.altitudeAglM = lerp(a.altitudeAglM, b.altitudeAglM, t);
    out.airspeedTrueMs = lerp(a.airspeedTrueMs, b.airspeedTrueMs, t);
    out.aileronRad = lerp(a.aileronRad, b.aileronRad, t);
    out.elevatorRad = lerp(a.elevatorRad, b.elevatorRad, t);
    out.rudderRad = lerp(a.rudderRad, b.rudderRad, t);
    slerp(a.attitudeEcefToBody, b.attitudeEcefToBody, t, out.attitudeEcefToBody);
    bodyToEcefFromQuaternion(out.attitudeEcefToBody, out.rotationBodyToEcef);
}

void Interpolator::extrapolate(const sim::VehicleState& b, double dt, sim::VehicleState& out) {
    out = b;
    // ECEF velocity = R_b2e * v_body; attitude held (rates are small over a few steps).
    const double* R = b.rotationBodyToEcef;
    const double* v = b.velocityBodyMs;
    for (int r = 0; r < 3; ++r) {
        const double ve = R[r * 3 + 0] * v[0] + R[r * 3 + 1] * v[1] + R[r * 3 + 2] * v[2];
        out.positionEcef[r] = b.positionEcef[r] + ve * dt;
    }
    out.simTime = b.simTime + dt;
}

} // namespace fsim::world
