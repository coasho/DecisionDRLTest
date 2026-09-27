#pragma once

// The ICAO standard atmosphere, for what a command's speed means at the
// altitude it names (docs/vehicle-interface.md, 4.3): a calibrated airspeed
// or a Mach number turned into a true airspeed there, before the aircraft is
// at it. Below 20 km; the equivalent airspeed stands in for the calibrated.

#include <algorithm>
#include <cmath>

namespace fsim::control::isa {

inline constexpr double kSeaLevelDensity = 1.225;       ///< kg/m3
inline constexpr double kSeaLevelTemperature = 288.15;  ///< K
inline constexpr double kGasConstant = 287.053;         ///< J/(kg K), dry air
inline constexpr double kGamma = 1.4;
inline constexpr double kLapse = 0.0065;                ///< K/m, to the tropopause
inline constexpr double kTropopause = 11000.0;          ///< m
inline constexpr double kG0 = 9.80665;

/// Temperature, K, at a geopotential altitude, m (the stratosphere's 216.65 K above 11 km).
inline double temperature(double altitudeM) noexcept {
    return kSeaLevelTemperature - kLapse * std::clamp(altitudeM, -1000.0, kTropopause);
}

/// Density over sea level's, at an altitude.
inline double densityRatio(double altitudeM) noexcept {
    const double h = std::clamp(altitudeM, -1000.0, 20000.0);
    const double t = temperature(h);
    const double exponent = kG0 / (kLapse * kGasConstant); // 5.2559
    const double troposphere = std::pow(t / kSeaLevelTemperature, exponent - 1.0);
    if (h <= kTropopause) return troposphere;
    return troposphere * std::exp(-kG0 * (h - kTropopause) / (kGasConstant * t));
}

/// The speed of sound, m/s, at an altitude.
inline double speedOfSound(double altitudeM) noexcept { return std::sqrt(kGamma * kGasConstant * temperature(altitudeM)); }

/// A true airspeed at an altitude as a calibrated one (equivalent: tas sqrt(sigma)).
inline double calibratedFromTrue(double tasMs, double altitudeM) noexcept { return tasMs * std::sqrt(densityRatio(altitudeM)); }
/// A calibrated airspeed at an altitude as a true one.
inline double trueFromCalibrated(double casMs, double altitudeM) noexcept { return casMs / std::sqrt(densityRatio(altitudeM)); }

} // namespace fsim::control::isa
