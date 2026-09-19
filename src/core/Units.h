#pragma once

/// Unit conversions between JSBSim's imperial internals and the platform's SI
/// world (design 7.3). All conversions to/from JSBSim happen in the sim module;
/// nothing outside it ever sees feet, slugs or knots.
namespace fsim::units {

inline constexpr double kFeetToMetres = 0.3048;
inline constexpr double kMetresToFeet = 1.0 / kFeetToMetres;

inline constexpr double kKnotsToMetresPerSecond = 1852.0 / 3600.0; // 0.514444...
inline constexpr double kMetresPerSecondToKnots = 1.0 / kKnotsToMetresPerSecond;

inline constexpr double kFeetPerSecondToMetresPerSecond = kFeetToMetres;
inline constexpr double kMetresPerSecondToFeetPerSecond = kMetresToFeet;

inline constexpr double kSlugsToKilograms = 14.593902937;
inline constexpr double kKilogramsToSlugs = 1.0 / kSlugsToKilograms;

inline constexpr double kPoundsForceToNewtons = 4.4482216152605;
inline constexpr double kNewtonsToPoundsForce = 1.0 / kPoundsForceToNewtons;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegreesToRadians = kPi / 180.0;
inline constexpr double kRadiansToDegrees = 180.0 / kPi;

constexpr double feetToMetres(double ft) noexcept { return ft * kFeetToMetres; }
constexpr double metresToFeet(double m) noexcept { return m * kMetresToFeet; }
constexpr double knotsToMetresPerSecond(double kts) noexcept { return kts * kKnotsToMetresPerSecond; }
constexpr double metresPerSecondToKnots(double ms) noexcept { return ms * kMetresPerSecondToKnots; }
constexpr double degreesToRadians(double deg) noexcept { return deg * kDegreesToRadians; }
constexpr double radiansToDegrees(double rad) noexcept { return rad * kRadiansToDegrees; }
constexpr double poundsForceToNewtons(double lbf) noexcept { return lbf * kPoundsForceToNewtons; }

} // namespace fsim::units
