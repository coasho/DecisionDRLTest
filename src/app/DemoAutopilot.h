#pragma once

#include "sim/ControlInputs.h"
#include "sim/VehicleState.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::app {

/// Tiny attitude-hold controller so viewer demo vehicles fly stable, visibly
/// different patterns instead of stalling. Not part of the platform proper:
/// scenario tasks and trained policies replace it (design 9.1).
class DemoAutopilot {
public:
    DemoAutopilot(std::size_t vehicles, double throttle, bool parked = false) : throttle_(throttle), parked_(parked) {
        targets_.resize(vehicles);
        for (std::size_t i = 0; i < vehicles; ++i) {
            // Alternate gentle left/right turns of varying bank so formations spread out.
            const double bank = 5.0 + 20.0 * static_cast<double>(i % 5) / 4.0;
            targets_[i].rollDeg = (i % 2 == 0) ? bank : -bank;
            targets_[i].altitudeM = 0.0; // set from the first state
        }
    }

    /// Forget the held altitude of a vehicle that was reset.
    void forget(std::size_t i) {
        if (i < targets_.size()) targets_[i].altitudeM = 0.0;
    }

    void compute(const std::vector<sim::VehicleState>& states, std::vector<sim::ControlInputs>& out) {
        constexpr double kDeg = 3.14159265358979323846 / 180.0;
        for (std::size_t i = 0; i < states.size() && i < out.size(); ++i) {
            const auto& s = states[i];
            auto& t = targets_[i];
            if (parked_) { // sit on the terrain: idle, brakes on, gear down
                sim::ControlInputs& c = out[i];
                c = sim::ControlInputs{};
                c.brakeLeft = c.brakeRight = 1.0;
                c.gearDown = 1.0;
                continue;
            }
            if (t.altitudeM <= 0.0) t.altitudeM = s.altitudeMslM;

            // Outer loop: altitude -> pitch target (clamped), inner: pitch/roll -> surfaces.
            const double altErr = t.altitudeM - s.altitudeMslM;
            const double pitchTarget = std::clamp(2.0 + 0.02 * altErr, -8.0, 12.0) * kDeg;
            const double pitchErr = pitchTarget - s.eulerRad[1];
            const double rollErr = t.rollDeg * kDeg - s.eulerRad[0];

            sim::ControlInputs& c = out[i];
            // JSBSim: elevator +1 = nose down, so a positive pitch error needs negative elevator.
            c.elevator = std::clamp(-2.5 * pitchErr + 1.2 * s.angularRateBodyRadS[1], -0.6, 0.6);
            c.aileron = std::clamp(1.2 * rollErr - 0.35 * s.angularRateBodyRadS[0], -0.5, 0.5);
            c.rudder = std::clamp(-0.5 * s.betaRad, -0.3, 0.3);
            c.setThrottleAll(std::clamp(throttle_ + 0.01 * altErr, 0.2, 1.0));
            c.gearDown = 0.0;
        }
    }

private:
    struct Target {
        double rollDeg = 0.0;
        double altitudeM = 0.0;
    };
    std::vector<Target> targets_;
    double throttle_;
    bool parked_;
};

} // namespace fsim::app
