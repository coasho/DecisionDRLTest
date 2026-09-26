#pragma once

#include <limits>

namespace fsim::sim {

/// Normalised control commands written to JSBSim's `fcs/*-cmd-norm`
/// properties each step (design 6.3, action mapping "surfaces").
struct ControlInputs {
    static constexpr int kMaxEngines = 4;

    double aileron = 0.0;  ///< -1 (left) .. +1 (right)
    double elevator = 0.0; ///< -1 (nose up) .. +1 (nose down): JSBSim's sign, trailing edge down
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

/// Effectors ControlInputs has no room for - its layout is shared with
/// viewers, recordings and the C ABI, so it does not grow
/// (docs/control-architecture.md, 13). Written to the flight model beside it;
/// NaN = leave the effector as it is.
struct EffectorInputs {
    double speedbrake = std::numeric_limits<double>::quiet_NaN(); ///< 0 in .. 1 out
    double pitchTrim = std::numeric_limits<double>::quiet_NaN();  ///< -1 .. 1, + nose down
};

} // namespace fsim::sim

namespace fsim {
using sim::ControlInputs;
using sim::EffectorInputs;
}
