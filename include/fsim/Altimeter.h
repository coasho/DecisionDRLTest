#pragma once

// The barometric altimeter (docs/flight-autonomy.md, 4.20; A-GRA's MA_AirDataType and its QNH setting, VI 1.2.6.5).
// The air a vehicle flies in is the 1976 standard atmosphere's layers with a uniform temperature bias and a
// sea-level pressure - the world's (EnvironmentState), as the flight models have it (JSBSim's standard atmosphere).
// The altimeter is calibrated to the ICAO standard atmosphere and set to a pressure: its QNH, A-GRA's Kollsman.
// Heights above sea level are above the WGS-84 ellipsoid, as everywhere in the platform; an altimeter reads
// geopotential metres, as the standard atmosphere is laid out in them.

#include "fsim/Export.h"

namespace fsim::control {

/// The world's air: its sea-level temperature and pressure.
struct Air {
    double temperatureSeaLevelK = 288.15;
    double pressureSeaLevelPa = 101325.0;
};

/// A vehicle's barometric altimeter: the air it is in, and the pressure it is set to.
struct Altimeter {
    static constexpr double kStandardPa = 101325.0; ///< 1013.25 hPa: set to it, an altimeter reads the pressure altitude
    Air air{};
    double qnhPa = kStandardPa;
};

/// The air's static pressure (Pa) and temperature (K) at a height above sea level.
FSIM_API double staticPressurePa(const Air& air, double altitudeMslM) noexcept;
FSIM_API double staticTemperatureK(const Air& air, double altitudeMslM) noexcept;
/// The height above sea level at which the air has a pressure: its isobar.
FSIM_API double altitudeOfPressureM(const Air& air, double pressurePa) noexcept;

/// What an altimeter set to `qnhPa` reads at a static pressure: the ICAO standard atmosphere's height of that
/// pressure above the pressure it is set to. And the pressure at which it reads an altitude.
FSIM_API double indicatedAltitudeM(double pressurePa, double qnhPa) noexcept;
FSIM_API double pressureIndicatingPa(double indicatedM, double qnhPa) noexcept;

/// An altimeter's reading at a height above sea level; and the height at which it reads an altitude - the isobar
/// a barometric altitude is flown on.
FSIM_API double indicatedAltitudeM(const Altimeter& a, double altitudeMslM) noexcept;
FSIM_API double barometricMslM(const Altimeter& a, double indicatedM) noexcept;
/// How fast its reading changes, climbing at `upMs` through a height above sea level.
FSIM_API double indicatedRateMs(const Altimeter& a, double altitudeMslM, double upMs) noexcept;

} // namespace fsim::control
