#pragma once

#include <cstdint>

namespace fsim::sim {

/// Fixed-size snapshot of one vehicle after a step (design 6.3). Plain data,
/// SI units, ECEF metres; written whole by the worker that stepped the vehicle
/// and read by observation builders and the viewer.
///
/// Compatibility rule (design 10.3): fields may only be appended.
struct VehicleState {
    static constexpr int kMaxEngines = 4;

    double simTime = 0.0; ///< seconds since reset

    // Position / attitude
    double positionEcef[3] = {0, 0, 0};             ///< m, WGS-84 ECEF
    double attitudeEcefToBody[4] = {1, 0, 0, 0};    ///< quaternion w,x,y,z (JSBSim body: x fwd, y right, z down)
    double latitudeRad = 0.0;                       ///< geodetic
    double longitudeRad = 0.0;
    double altitudeMslM = 0.0;                      ///< above the WGS-84 ellipsoid (JSBSim "ASL")
    double altitudeAglM = 0.0;                      ///< above ground provider terrain
    double eulerRad[3] = {0, 0, 0};                 ///< roll, pitch, yaw of body wrt local NED

    // Velocities / rates
    double velocityBodyMs[3] = {0, 0, 0};           ///< u, v, w
    double velocityNedMs[3] = {0, 0, 0};            ///< north, east, down
    double angularRateBodyRadS[3] = {0, 0, 0};      ///< p, q, r
    double accelerationBodyMs2[3] = {0, 0, 0};      ///< du/dt, dv/dt, dw/dt

    // Air data
    double airspeedTrueMs = 0.0;
    double airspeedCalibratedMs = 0.0;
    double mach = 0.0;
    double alphaRad = 0.0;
    double betaRad = 0.0;
    double loadFactor = 1.0;

    // Controls and propulsion (positions, not commands)
    double aileronRad = 0.0;
    double elevatorRad = 0.0;
    double rudderRad = 0.0;
    double flapsRad = 0.0;
    double gearPosition = 1.0;                      ///< 0 up .. 1 down
    int engineCount = 0;
    double throttlePosition[kMaxEngines] = {0, 0, 0, 0};
    double thrustN[kMaxEngines] = {0, 0, 0, 0};
    double fuelKg = 0.0;

    // Status
    std::uint32_t stepCount = 0;                    ///< FDM steps since reset
    bool onGround = false;
    bool diverged = false;                          ///< NaN/inf detected; vehicle must be reset

    // Appended (viewer): body -> ECEF rotation, row-major. Columns are the body
    // axes (x forward, y right, z down) expressed in ECEF; equals the transpose
    // of JSBSim's Tec2b. Redundant with the quaternion but unambiguous.
    double rotationBodyToEcef[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

} // namespace fsim::sim

namespace fsim {
using sim::VehicleState;
}
