// A route segment's climb optimisation (docs/flight-autonomy.md, 4.32; ADR-29 FA-6c2): the rate the aircraft climbs
// or descends at holding its speed, and the altitude an efficient change holds - both from its performance tables; and
// the speeds it flies level at, a required time of arrival's (4.33; FA-6d1).
#include "control/Route.h"

#include "fsim/VehicleProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control::route {

namespace {

/// The tables' weight with the fuel on board: their weight with the tanks empty, and the fuel (as optimalTasMs's).
double weightOf(const TablesSection& t, double fuelKg) noexcept {
    double weight = t.weightKg.back();
    if (t.weightKg.size() > 1 && std::isfinite(t.fuelCapacityKg) && std::isfinite(fuelKg)) weight += fuelKg - t.fuelCapacityKg;
    return weight;
}

} // namespace

double climbRateMs(const TablesSection* tables, const Performance& f, bool hovers, bool climbing, double altitudeMslM, double tasMs, double fuelKg) noexcept {
    const double most = climbing ? f.maxClimbMs : f.maxDescentMs;
    double rate = std::numeric_limits<double>::quiet_NaN();
    if (tables && !tables->empty() && (climbing || !hovers)) { // (energy held: the height changes at the excess power's rate)
        const TablesSection& t = *tables;
        const double weight = weightOf(t, fuelKg);
        auto read = [&](double v) {
            const TablesAtSpeed at = tablesAt(t, altitudeMslM, weight, v);
            return climbing ? at.psFullMs : -at.psIdleMs;
        };
        rate = read(tasMs);
        // a speed they do not read there (outside the level speeds, or at a row's edge): the nearest one they do, toward the
        // middle of the level speeds
        const TablesAt level = std::isnan(rate) ? tablesAt(t, altitudeMslM, weight) : TablesAt{};
        if (std::isnan(rate) && std::isfinite(level.minTasMs) && std::isfinite(level.maxTasMs)) {
            const double lo = level.minTasMs, hi = 0.97 * level.maxTasMs, v = std::clamp(tasMs, lo, hi), middle = 0.5 * (lo + hi);
            for (int k = 0; k <= 8 && std::isnan(rate); ++k) rate = read(v + (middle - v) * k / 8.0);
        }
    }
    if (std::isnan(rate)) return most;
    rate = std::max(rate, 0.0);
    return std::isfinite(most) ? std::min(rate, most) : rate;
}

double cheapestAltitudeM(const TablesSection* tables, double fromMslM, double toMslM, double tasMs, double fuelKg) noexcept {
    if (!tables || tables->empty()) return toMslM;
    const TablesSection& t = *tables;
    const double weight = weightOf(t, fuelKg);
    // fuel per second at one speed is fuel per metre, scaled alike at every altitude; an electric aircraft's, power
    const bool fuel = std::any_of(t.fuelKgS.begin(), t.fuelKgS.end(), [](double v) { return std::isfinite(v); });
    auto cost = [&](double h) {
        const TablesAtSpeed at = tablesAt(t, h, weight, tasMs);
        return fuel ? at.fuelKgS : at.powerW;
    };
    double best = toMslM, least = cost(toMslM);
    auto consider = [&](double h) {
        const double c = cost(h);
        if (std::isfinite(c) && !(c >= least)) best = h, least = c; // (strictly less: ties stay toward the far end)
    };
    // from the far end back: the rows between, then the near end
    const double lo = std::min(fromMslM, toMslM), hi = std::max(fromMslM, toMslM);
    if (toMslM >= fromMslM) {
        for (auto h = t.altitudeM.rbegin(); h != t.altitudeM.rend(); ++h)
            if (*h > lo && *h < hi) consider(*h);
    } else {
        for (const double h : t.altitudeM)
            if (h > lo && h < hi) consider(h);
    }
    consider(fromMslM);
    return best;
}

void levelSpeedsMs(const TablesSection* tables, const Performance& f, bool hovers, double altitudeMslM, double fuelKg, double& least,
                   double& most) noexcept {
    least = hovers ? 1.0 : std::numeric_limits<double>::quiet_NaN(), most = std::numeric_limits<double>::quiet_NaN();
    if (hovers) { // (its fastest over the ground: a route's own limit)
        most = std::isfinite(f.maxGroundSpeedMs) ? f.maxGroundSpeedMs : f.maxTasMs;
        return;
    }
    // (its slowest: 1.2 times its envelope's least, above where energy management begins to act (1.1 times) - the tables'
    // slowest level speed is narrower than what it flies: a Typhoon at 139 m/s where they read 168 - else theirs)
    if (tables && !tables->empty()) {
        const TablesAt level = tablesAt(*tables, altitudeMslM, weightOf(*tables, fuelKg));
        least = level.minTasMs, most = 0.97 * level.maxTasMs;
    }
    if (std::isfinite(f.minCasMs)) least = plannedSpeed(1.2 * f.minCasMs, static_cast<double>(SpeedReference::CalibratedAirspeed), altitudeMslM);
    if (std::isfinite(f.maxTasMs)) most = std::isfinite(most) ? std::min(most, f.maxTasMs) : f.maxTasMs;
}

} // namespace fsim::control::route
