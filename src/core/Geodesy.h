#pragma once

#include "core/Units.h"

#include <cmath>

namespace fsim::geo {

inline constexpr double kEarthRadiusM = 6371008.8; ///< mean radius, for local geometry only

/// Wrap an angle to (-pi, pi].
inline double wrapPi(double a) noexcept {
    a = std::fmod(a + units::kPi, 2.0 * units::kPi);
    if (a < 0.0) a += 2.0 * units::kPi;
    return a - units::kPi;
}

/// Wrap an angle to [0, 2pi).
inline double wrapTwoPi(double a) noexcept {
    a = std::fmod(a, 2.0 * units::kPi);
    return a < 0.0 ? a + 2.0 * units::kPi : a;
}

/// Initial bearing (radians, 0 = north, clockwise) from 1 to 2.
inline double bearingRad(double lat1, double lon1, double lat2, double lon2) noexcept {
    const double dLon = lon2 - lon1;
    const double y = std::sin(dLon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
    return wrapTwoPi(std::atan2(y, x));
}

/// Great-circle surface distance in metres (haversine).
inline double distanceM(double lat1, double lon1, double lat2, double lon2) noexcept {
    const double dLat = lat2 - lat1, dLon = lon2 - lon1;
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(a)));
}

/// Local north/east offset (metres) of 2 relative to 1 (flat-Earth, fine for tens of km).
inline void localNorthEastM(double lat1, double lon1, double lat2, double lon2, double& north, double& east) noexcept {
    north = (lat2 - lat1) * kEarthRadiusM;
    east = (lon2 - lon1) * kEarthRadiusM * std::cos(0.5 * (lat1 + lat2));
}

/// Geodetic point `north`/`east` metres from (lat, lon).
inline void offsetLatLon(double lat, double lon, double north, double east, double& latOut, double& lonOut) noexcept {
    latOut = lat + north / kEarthRadiusM;
    lonOut = lon + east / (kEarthRadiusM * std::cos(lat));
}

} // namespace fsim::geo
