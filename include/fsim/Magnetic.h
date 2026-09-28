#pragma once

// The Earth's magnetic field (docs/flight-autonomy.md, 4.22): the World Magnetic Model 2025 of NOAA's National Centers
// for Environmental Information and the British Geological Survey - its published coefficients and equations - for a
// declination at a place and date, what a magnetic heading or course is measured from. Places are geodetic, heights
// above the WGS-84 ellipsoid (the platform's sea level), dates decimal years.

#include "fsim/Export.h"

namespace fsim::control {

/// The model's epoch, and the end of the five years it is made for. magneticField carries it on outside them at its
/// rates of change, as NOAA's own software does; a world's time is read within them (magneticYear).
inline constexpr double kMagneticEpochYear = 2025.0;
inline constexpr double kMagneticValidUntilYear = 2030.0;

/// The field at a place and date: its components north, east and down (nT), its horizontal and total intensity, its
/// declination (east of true north) and inclination (below the horizontal).
struct MagneticField {
    double northNt = 0.0, eastNt = 0.0, downNt = 0.0;
    double horizontalNt = 0.0, totalNt = 0.0;
    double declinationRad = 0.0, inclinationRad = 0.0;
};

FSIM_API MagneticField magneticField(double latitudeRad, double longitudeRad, double heightAboveEllipsoidM, double decimalYear) noexcept;
/// Its declination alone: what a magnetic direction is turned by to be a true one (true = magnetic + declination).
FSIM_API double declinationRad(double latitudeRad, double longitudeRad, double heightAboveEllipsoidM, double decimalYear) noexcept;
/// A UTC time (Unix seconds) as a decimal year: the year and the part of it gone.
FSIM_API double decimalYear(double unixSeconds) noexcept;
/// The date the model is read at for a world's time (Unix seconds): its decimal year, held within the model's five
/// years - a world whose clock was never set (1970) reads its epoch.
FSIM_API double magneticYear(double unixSeconds) noexcept;

} // namespace fsim::control
