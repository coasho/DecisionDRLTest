#pragma once

#include <cstdint>

namespace fsim::sim {

/// Global environmental conditions of a world (design 9.4): set by the
/// training application, applied to every flight model each step, and
/// published to the viewer for illumination and weather. Plain data, SI.
///
/// Compatibility rule (design 10.3): fields may only be appended.
struct EnvironmentState {
    // Time: simulation epoch as Unix UTC seconds; current UTC = epoch + simTime.
    double epochUtcSeconds = 0.0;
    double timeFactor = 1.0;             ///< informational (viewer pacing hint); the trainer sets the pace

    // Atmosphere (JSBSim standard atmosphere adjusted at sea level)
    double temperatureSeaLevelK = 288.15;
    double pressureSeaLevelPa = 101325.0;
    double humidity = 0.0;               ///< 0..1, sensors/rendering only

    // Wind (meteorological: direction the wind blows FROM, degrees true)
    double windDirectionDeg = 0.0;
    double windSpeedMs = 0.0;
    double windGustMs = 0.0;             ///< peak gust above the mean
    double turbulence = 0.0;             ///< 0 (none) .. 1 (severe), JSBSim Milspec intensity

    // Weather (rendering and sensor models; no FDM effect)
    double visibilityM = 50000.0;
    double cloudBaseM = 2000.0;
    double cloudCover = 0.0;             ///< 0..1
    double precipitation = 0.0;          ///< 0..1

    std::uint64_t revision = 0;          ///< bumped on every change
};

} // namespace fsim::sim

namespace fsim {
using sim::EnvironmentState;
}
