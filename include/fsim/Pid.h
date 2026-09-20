#pragma once

#include <algorithm>
#include <cmath>

namespace fsim::control {

/// Small PID with anti-windup used by the built-in loops. Derivative acts on
/// the measured rate (not the error) so setpoint steps do not kick.
struct Pid {
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double integralLimit = 1.0;
    double outMin = -1.0, outMax = 1.0;
    double integral = 0.0;

    double update(double error, double measuredRate, double dt) noexcept {
        integral = std::clamp(integral + ki * error * dt, -integralLimit, integralLimit);
        return std::clamp(kp * error + integral - kd * measuredRate, outMin, outMax);
    }
    void reset() noexcept { integral = 0.0; }
};

/// First-order rate limiter for setpoints.
inline double rateLimit(double current, double target, double maxRatePerS, double dt) noexcept {
    const double step = maxRatePerS * dt;
    return current + std::clamp(target - current, -step, step);
}

} // namespace fsim::control
