#pragma once

namespace fsim::sim {

/// Where and how to start. Geodetic, SI. Sampled per vehicle by the
/// environment from the scenario's distributions.
struct InitialConditions {
    double latitudeDeg = 37.6188;   ///< default: KSFO area
    double longitudeDeg = -122.375;
    double altitudeMslM = 1500.0;
    double headingDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    double airspeedTrueMs = 60.0;
    bool onGround = false;          ///< if true, altitude is taken from terrain
};

} // namespace fsim::sim

namespace fsim {
using sim::InitialConditions;
}
