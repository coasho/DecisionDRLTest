#pragma once

namespace fsim::sim {

/// Normalised control commands written to JSBSim's `fcs/*-cmd-norm`
/// properties each step (design 6.3, action mapping "surfaces").
struct ControlInputs {
    static constexpr int kMaxEngines = 4;

    double aileron = 0.0;  ///< -1 (left) .. +1 (right)
    double elevator = 0.0; ///< -1 (nose down) .. +1 (nose up), JSBSim sign convention
    double rudder = 0.0;   ///< -1 .. +1
    double throttle[kMaxEngines] = {0, 0, 0, 0}; ///< 0 .. 1 per engine
    double flaps = 0.0;    ///< 0 .. 1
    double gearDown = 1.0; ///< 0 (up) or 1 (down)
    double brakeLeft = 0.0;  ///< 0 .. 1
    double brakeRight = 0.0; ///< 0 .. 1

    /// Same throttle for every engine.
    void setThrottleAll(double value) noexcept {
        for (double& t : throttle) t = value;
    }
};

} // namespace fsim::sim

namespace fsim {
using sim::ControlInputs;
}
