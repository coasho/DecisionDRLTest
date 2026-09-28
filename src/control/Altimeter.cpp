// The barometric altimeter (docs/flight-autonomy.md, 4.20): the world's air, the 1976 standard atmosphere's layers
// with a uniform temperature bias and a sea-level pressure, as the flight models have it; an altimeter calibrated to
// the ICAO standard atmosphere. Asked when a command is checked, and by a mode flying a barometric altitude.
#include "fsim/Altimeter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace fsim::control {

namespace {

constexpr double kG0 = 9.80665;        ///< m/s2
constexpr double kRadius = 6356766.0;  ///< m: the standard atmosphere's, for geopotential heights
constexpr double kR = 8.31432;         ///< J/(mol K)
/// Dry air's gas constant: the flight model's (JSBSim takes air's molar mass as 28.9645 g/mol), and the ICAO
/// standard atmosphere's (28.9644 g/mol), which the altimeter is calibrated to.
constexpr double kAirR = kR / 0.0289645;
constexpr double kIcaoR = kR / 0.0289644;

/// The layers: each one's geopotential base (m), its standard temperature there (K) and its lapse (K/m).
struct Layer {
    double baseM, temperatureK, lapse;
};
constexpr std::array<Layer, 8> kLayers{{
    {0.0, 288.15, -0.0065},
    {11000.0, 216.65, 0.0},
    {20000.0, 216.65, 0.001},
    {32000.0, 228.65, 0.0028},
    {47000.0, 270.65, 0.0},
    {51000.0, 270.65, -0.0028},
    {71000.0, 214.65, -0.002},
    {84852.0, 186.946, 0.0},
}};

double geopotential(double h) noexcept { return kRadius * h / (kRadius + h); }
double geometric(double H) noexcept { return kRadius * H / (kRadius - H); }

/// The pressure at the end of `height` into a layer from its base, at `baseT` there (the bias in it) and `baseP`.
double across(const Layer& l, double baseT, double baseP, double height, double r) noexcept {
    if (l.lapse != 0.0) return baseP * std::pow(baseT / (baseT + l.lapse * height), kG0 / (r * l.lapse));
    return baseP * std::exp(-kG0 * height / (r * baseT));
}

/// The layer a geopotential height is in (below sea level: the first), with its base's pressure.
std::size_t layerOf(double H, double bias, double seaLevelPa, double r, double& baseP) noexcept {
    baseP = seaLevelPa;
    std::size_t b = 0;
    while (b + 1 < kLayers.size() && H >= kLayers[b + 1].baseM) {
        baseP = across(kLayers[b], kLayers[b].temperatureK + bias, baseP, kLayers[b + 1].baseM - kLayers[b].baseM, r);
        ++b;
    }
    return b;
}

/// Pressure at a geopotential height, in an atmosphere of a temperature bias, sea-level pressure and gas constant.
double pressureAt(double H, double bias, double seaLevelPa, double r) noexcept {
    double baseP = 0.0;
    const Layer& l = kLayers[layerOf(H, bias, seaLevelPa, r, baseP)];
    return across(l, l.temperatureK + bias, baseP, H - l.baseM, r);
}

/// The geopotential height of a pressure, likewise.
double heightOf(double p, double bias, double seaLevelPa, double r) noexcept {
    double baseP = seaLevelPa;
    std::size_t b = 0;
    for (; b + 1 < kLayers.size(); ++b) { // (the first layer whose top is at a lower pressure; below sea level, the first)
        const double top = across(kLayers[b], kLayers[b].temperatureK + bias, baseP, kLayers[b + 1].baseM - kLayers[b].baseM, r);
        if (p > top) break;
        baseP = top;
    }
    const Layer& l = kLayers[b];
    const double t = l.temperatureK + bias;
    if (l.lapse != 0.0) return l.baseM + (t * std::pow(p / baseP, -r * l.lapse / kG0) - t) / l.lapse;
    return l.baseM - r * t / kG0 * std::log(p / baseP);
}

double biasOf(const Air& air) noexcept { return air.temperatureSeaLevelK - kLayers[0].temperatureK; }

double temperatureAt(double H, double bias) noexcept {
    std::size_t b = 0;
    while (b + 1 < kLayers.size() && H >= kLayers[b + 1].baseM) ++b;
    return kLayers[b].temperatureK + bias + kLayers[b].lapse * (H - kLayers[b].baseM);
}

} // namespace

double staticPressurePa(const Air& air, double altitudeMslM) noexcept {
    return pressureAt(geopotential(altitudeMslM), biasOf(air), air.pressureSeaLevelPa, kAirR);
}

double staticTemperatureK(const Air& air, double altitudeMslM) noexcept { return temperatureAt(geopotential(altitudeMslM), biasOf(air)); }

double altitudeOfPressureM(const Air& air, double pressurePa) noexcept {
    return geometric(heightOf(pressurePa, biasOf(air), air.pressureSeaLevelPa, kAirR));
}

double indicatedAltitudeM(double pressurePa, double qnhPa) noexcept {
    return heightOf(pressurePa, 0.0, Altimeter::kStandardPa, kIcaoR) - heightOf(qnhPa, 0.0, Altimeter::kStandardPa, kIcaoR);
}

double pressureIndicatingPa(double indicatedM, double qnhPa) noexcept {
    return pressureAt(indicatedM + heightOf(qnhPa, 0.0, Altimeter::kStandardPa, kIcaoR), 0.0, Altimeter::kStandardPa, kIcaoR);
}

double indicatedAltitudeM(const Altimeter& a, double altitudeMslM) noexcept { return indicatedAltitudeM(staticPressurePa(a.air, altitudeMslM), a.qnhPa); }

double barometricMslM(const Altimeter& a, double indicatedM) noexcept { return altitudeOfPressureM(a.air, pressureIndicatingPa(indicatedM, a.qnhPa)); }

double indicatedRateMs(const Altimeter& a, double altitudeMslM, double upMs) noexcept {
    // through the air's pressure (hydrostatic, in geopotential metres) and back through the standard atmosphere's:
    // the ratio of the temperatures the two have at that pressure, and of their gas constants
    const double p = staticPressurePa(a.air, altitudeMslM);
    const double pressureAltitude = heightOf(p, 0.0, Altimeter::kStandardPa, kIcaoR);
    const double stretch = kRadius / (kRadius + altitudeMslM);
    return kIcaoR * temperatureAt(pressureAltitude, 0.0) / (kAirR * staticTemperatureK(a.air, altitudeMslM)) * stretch * stretch * upMs;
}

} // namespace fsim::control
