// The Earth's magnetic field (docs/flight-autonomy.md, 4.22): the World Magnetic Model 2025 of NOAA's National Centers
// for Environmental Information and the British Geological Survey (a work of the US Government, in the public domain),
// its coefficients as WMM.COF of 11/13/2024 gives them, evaluated with the equations its technical report publishes.
// Asked when a magnetic direction is resolved, and now and then as one is flown: never every step.
#include "fsim/Magnetic.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

/// WMM.COF's rows: degree n, order m, g and h at the epoch (nT), and their changes (nT a year).
struct Coefficient {
    int n, m;
    double g, h, gDot, hDot;
};
constexpr Coefficient kWmm2025[] = {
    {1, 0, -29351.8, 0.0, 12.0, 0.0},
    {1, 1, -1410.8, 4545.4, 9.7, -21.5},
    {2, 0, -2556.6, 0.0, -11.6, 0.0},
    {2, 1, 2951.1, -3133.6, -5.2, -27.7},
    {2, 2, 1649.3, -815.1, -8.0, -12.1},
    {3, 0, 1361.0, 0.0, -1.3, 0.0},
    {3, 1, -2404.1, -56.6, -4.2, 4.0},
    {3, 2, 1243.8, 237.5, 0.4, -0.3},
    {3, 3, 453.6, -549.5, -15.6, -4.1},
    {4, 0, 895.0, 0.0, -1.6, 0.0},
    {4, 1, 799.5, 278.6, -2.4, -1.1},
    {4, 2, 55.7, -133.9, -6.0, 4.1},
    {4, 3, -281.1, 212.0, 5.6, 1.6},
    {4, 4, 12.1, -375.6, -7.0, -4.4},
    {5, 0, -233.2, 0.0, 0.6, 0.0},
    {5, 1, 368.9, 45.4, 1.4, -0.5},
    {5, 2, 187.2, 220.2, 0.0, 2.2},
    {5, 3, -138.7, -122.9, 0.6, 0.4},
    {5, 4, -142.0, 43.0, 2.2, 1.7},
    {5, 5, 20.9, 106.1, 0.9, 1.9},
    {6, 0, 64.4, 0.0, -0.2, 0.0},
    {6, 1, 63.8, -18.4, -0.4, 0.3},
    {6, 2, 76.9, 16.8, 0.9, -1.6},
    {6, 3, -115.7, 48.8, 1.2, -0.4},
    {6, 4, -40.9, -59.8, -0.9, 0.9},
    {6, 5, 14.9, 10.9, 0.3, 0.7},
    {6, 6, -60.7, 72.7, 0.9, 0.9},
    {7, 0, 79.5, 0.0, -0.0, 0.0},
    {7, 1, -77.0, -48.9, -0.1, 0.6},
    {7, 2, -8.8, -14.4, -0.1, 0.5},
    {7, 3, 59.3, -1.0, 0.5, -0.8},
    {7, 4, 15.8, 23.4, -0.1, 0.0},
    {7, 5, 2.5, -7.4, -0.8, -1.0},
    {7, 6, -11.1, -25.1, -0.8, 0.6},
    {7, 7, 14.2, -2.3, 0.8, -0.2},
    {8, 0, 23.2, 0.0, -0.1, 0.0},
    {8, 1, 10.8, 7.1, 0.2, -0.2},
    {8, 2, -17.5, -12.6, 0.0, 0.5},
    {8, 3, 2.0, 11.4, 0.5, -0.4},
    {8, 4, -21.7, -9.7, -0.1, 0.4},
    {8, 5, 16.9, 12.7, 0.3, -0.5},
    {8, 6, 15.0, 0.7, 0.2, -0.6},
    {8, 7, -16.8, -5.2, -0.0, 0.3},
    {8, 8, 0.9, 3.9, 0.2, 0.2},
    {9, 0, 4.6, 0.0, -0.0, 0.0},
    {9, 1, 7.8, -24.8, -0.1, -0.3},
    {9, 2, 3.0, 12.2, 0.1, 0.3},
    {9, 3, -0.2, 8.3, 0.3, -0.3},
    {9, 4, -2.5, -3.3, -0.3, 0.3},
    {9, 5, -13.1, -5.2, 0.0, 0.2},
    {9, 6, 2.4, 7.2, 0.3, -0.1},
    {9, 7, 8.6, -0.6, -0.1, -0.2},
    {9, 8, -8.7, 0.8, 0.1, 0.4},
    {9, 9, -12.9, 10.0, -0.1, 0.1},
    {10, 0, -1.3, 0.0, 0.1, 0.0},
    {10, 1, -6.4, 3.3, 0.0, 0.0},
    {10, 2, 0.2, 0.0, 0.1, -0.0},
    {10, 3, 2.0, 2.4, 0.1, -0.2},
    {10, 4, -1.0, 5.3, -0.0, 0.1},
    {10, 5, -0.6, -9.1, -0.3, -0.1},
    {10, 6, -0.9, 0.4, 0.0, 0.1},
    {10, 7, 1.5, -4.2, -0.1, 0.0},
    {10, 8, 0.9, -3.8, -0.1, -0.1},
    {10, 9, -2.7, 0.9, -0.0, 0.2},
    {10, 10, -3.9, -9.1, -0.0, -0.0},
    {11, 0, 2.9, 0.0, 0.0, 0.0},
    {11, 1, -1.5, 0.0, -0.0, -0.0},
    {11, 2, -2.5, 2.9, 0.0, 0.1},
    {11, 3, 2.4, -0.6, 0.0, -0.0},
    {11, 4, -0.6, 0.2, 0.0, 0.1},
    {11, 5, -0.1, 0.5, -0.1, -0.0},
    {11, 6, -0.6, -0.3, 0.0, -0.0},
    {11, 7, -0.1, -1.2, -0.0, 0.1},
    {11, 8, 1.1, -1.7, -0.1, -0.0},
    {11, 9, -1.0, -2.9, -0.1, 0.0},
    {11, 10, -0.2, -1.8, -0.1, 0.0},
    {11, 11, 2.6, -2.3, -0.1, 0.0},
    {12, 0, -2.0, 0.0, 0.0, 0.0},
    {12, 1, -0.2, -1.3, 0.0, -0.0},
    {12, 2, 0.3, 0.7, -0.0, 0.0},
    {12, 3, 1.2, 1.0, -0.0, -0.1},
    {12, 4, -1.3, -1.4, -0.0, 0.1},
    {12, 5, 0.6, -0.0, -0.0, -0.0},
    {12, 6, 0.6, 0.6, 0.1, -0.0},
    {12, 7, 0.5, -0.1, -0.0, -0.0},
    {12, 8, -0.1, 0.8, 0.0, 0.0},
    {12, 9, -0.4, 0.1, 0.0, -0.0},
    {12, 10, -0.2, -1.0, -0.1, -0.0},
    {12, 11, -1.3, 0.1, -0.0, 0.0},
    {12, 12, -0.7, 0.2, -0.1, -0.1},
};
constexpr int kDegree = 12;
constexpr double kReferenceRadiusM = 6371200.0; ///< the model's
constexpr double kWgs84A = 6378137.0, kWgs84F = 1.0 / 298.257223563;
constexpr double kPi = 3.14159265358979323846;

} // namespace

MagneticField magneticField(double latitudeRad, double longitudeRad, double heightAboveEllipsoidM, double year) noexcept {
    // the place as a radius and a geocentric latitude, on the WGS-84 ellipsoid (a hair off a pole, where east is none)
    const double e2 = kWgs84F * (2.0 - kWgs84F);
    const double sl = std::sin(latitudeRad), cl = std::cos(latitudeRad);
    const double normal = kWgs84A / std::sqrt(1.0 - e2 * sl * sl);
    const double p = (normal + heightAboveEllipsoidM) * cl, z = (normal * (1.0 - e2) + heightAboveEllipsoidM) * sl;
    const double r = std::hypot(p, z);
    const double geocentric = std::clamp(std::asin(z / r), -0.5 * kPi + 1e-9, 0.5 * kPi - 1e-9);
    const double ct = std::sin(geocentric), st = std::cos(geocentric); // (the colatitude's cosine and sine)

    // the coefficients at the date
    const double t = year - kMagneticEpochYear;
    double g[kDegree + 1][kDegree + 1] = {}, h[kDegree + 1][kDegree + 1] = {};
    for (const Coefficient& c : kWmm2025) g[c.n][c.m] = c.g + t * c.gDot, h[c.n][c.m] = c.h + t * c.hDot;

    // the associated Legendre functions of the colatitude, Gauss-normalized, and their derivatives by it; and the
    // factors that make them Schmidt semi-normalized, as the coefficients are
    double pnm[kDegree + 1][kDegree + 1] = {}, dpnm[kDegree + 1][kDegree + 1] = {}, schmidt[kDegree + 1][kDegree + 1] = {};
    pnm[0][0] = 1.0, schmidt[0][0] = 1.0;
    for (int n = 1; n <= kDegree; ++n) {
        for (int m = 0; m <= n; ++m) {
            if (m == n) {
                pnm[n][n] = st * pnm[n - 1][n - 1];
                dpnm[n][n] = st * dpnm[n - 1][n - 1] + ct * pnm[n - 1][n - 1];
            } else {
                const double k = n > 1 ? static_cast<double>((n - 1) * (n - 1) - m * m) / static_cast<double>((2 * n - 1) * (2 * n - 3)) : 0.0;
                const double before = n > 1 ? pnm[n - 2][m] : 0.0, dBefore = n > 1 ? dpnm[n - 2][m] : 0.0;
                pnm[n][m] = ct * pnm[n - 1][m] - k * before;
                dpnm[n][m] = ct * dpnm[n - 1][m] - st * pnm[n - 1][m] - k * dBefore;
            }
        }
        schmidt[n][0] = schmidt[n - 1][0] * static_cast<double>(2 * n - 1) / static_cast<double>(n);
        for (int m = 1; m <= n; ++m)
            schmidt[n][m] = schmidt[n][m - 1] * std::sqrt(static_cast<double>((n - m + 1) * (m == 1 ? 2 : 1)) / static_cast<double>(n + m));
    }

    // the field in the geocentric frame: radial, colatitudinal and east (the potential's gradient)
    double radial = 0.0, colatitudinal = 0.0, east = 0.0;
    const double ratio = kReferenceRadiusM / r;
    double power = ratio * ratio; // ((a/r)^(n+2), from n = 1)
    for (int n = 1; n <= kDegree; ++n) {
        power *= ratio;
        for (int m = 0; m <= n; ++m) {
            const double cm = std::cos(m * longitudeRad), sm = std::sin(m * longitudeRad);
            const double gs = schmidt[n][m] * g[n][m], hs = schmidt[n][m] * h[n][m];
            const double along = gs * cm + hs * sm;
            radial += (n + 1) * power * along * pnm[n][m];
            colatitudinal -= power * along * dpnm[n][m];
            east -= power * m * (hs * cm - gs * sm) * pnm[n][m];
        }
    }
    east /= st;
    // north, east and down there, turned from the geocentric frame to the geodetic
    const double northC = -colatitudinal, downC = -radial, tilt = geocentric - latitudeRad;
    MagneticField f;
    f.northNt = northC * std::cos(tilt) - downC * std::sin(tilt);
    f.eastNt = east;
    f.downNt = northC * std::sin(tilt) + downC * std::cos(tilt);
    f.horizontalNt = std::hypot(f.northNt, f.eastNt);
    f.totalNt = std::hypot(f.horizontalNt, f.downNt);
    f.declinationRad = std::atan2(f.eastNt, f.northNt);
    f.inclinationRad = std::atan2(f.downNt, f.horizontalNt);
    return f;
}

double declinationRad(double latitudeRad, double longitudeRad, double heightAboveEllipsoidM, double year) noexcept {
    return magneticField(latitudeRad, longitudeRad, heightAboveEllipsoidM, year).declinationRad;
}

double decimalYear(double unixSeconds) noexcept {
    // the year it is in, counted from 1970, and the part of it gone
    auto days = [](long long y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0 ? 366.0 : 365.0; };
    long long year = 1970;
    double seconds = unixSeconds;
    while (seconds >= days(year) * 86400.0) seconds -= days(year) * 86400.0, ++year;
    while (seconds < 0.0) --year, seconds += days(year) * 86400.0;
    return static_cast<double>(year) + seconds / (days(year) * 86400.0);
}

double magneticYear(double unixSeconds) noexcept {
    return std::clamp(decimalYear(unixSeconds), kMagneticEpochYear, kMagneticValidUntilYear);
}

} // namespace fsim::control
