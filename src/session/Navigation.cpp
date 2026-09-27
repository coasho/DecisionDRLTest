// The navigation report (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport): what a vehicle flies on,
// how much it has and for how long, its playtime to its recovery point and its contingency. Asked for, never
// stepped: in a file of its own, apart from the step.
#include "session/World.h"

#include "fsim/VehicleProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::session {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kEarthRadiusM = 6371008.8; // the mean radius: a return's distance over the ground

/// The distance over the ground between two points (the haversine), m.
double groundDistanceM(double lat1Rad, double lon1Rad, double lat2Rad, double lon2Rad) noexcept {
    const double a = std::sin(0.5 * (lat2Rad - lat1Rad)), b = std::sin(0.5 * (lon2Rad - lon1Rad));
    const double h = a * a + std::cos(lat1Rad) * std::cos(lat2Rad) * b * b;
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

} // namespace

control::Reason World::setNavigation(std::uint32_t id, const control::NavigationSettings& settings) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    const auto& s = settings;
    if (!(s.reserveFraction >= 0.0 && s.reserveFraction < 1.0)) return control::Reason::OutOfRange;
    if (s.recovery && !(std::abs(s.latitudeDeg) <= 90.0 && std::abs(s.longitudeDeg) <= 180.0 && std::isfinite(s.altitudeMslM)))
        return control::Reason::OutOfRange;
    e->navigation = settings;
    return control::Reason::None;
}

control::NavigationSettings World::navigation(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->navigation : control::NavigationSettings{};
}

control::NavigationReport World::navigationReport(std::uint32_t id) const {
    control::NavigationReport r;
    const Entry* e = entry(id);
    if (!e) return r;
    const sim::EnergyOnBoard en = pool_->vehicle(e->slot).energy();
    const sim::VehicleState& s = pool_->states()[e->slot];
    const bool fuel = en.fuelCapacityKg > 0.0; // (NaN: no tanks)
    const bool battery = !fuel && en.chargeCapacityJ > 0.0;
    if (fuel) {
        r.energy = control::Energy::Fuel;
        r.fuelKg = r.remaining = en.fuelKg, r.capacity = en.fuelCapacityKg, r.consumption = en.fuelFlowKgS;
    } else if (battery) {
        r.energy = control::Energy::Battery;
        r.fuelKg = 0.0, r.remaining = en.chargeJ, r.capacity = en.chargeCapacityJ, r.consumption = en.powerW;
    } else {
        return r; // (it flies on nothing its flight model tells of)
    }
    constexpr double kForever = std::numeric_limits<double>::infinity();
    r.percent = 100.0 * r.remaining / r.capacity;
    // (nothing left: none, whatever it consumes - a spent battery's motors draw nothing)
    r.enduranceS = !(r.remaining > 0.0) ? 0.0 : r.consumption > 0.0 ? r.remaining / r.consumption : kForever;
    r.reserve = e->navigation.reserveFraction * r.capacity;
    r.starved = en.starved || (battery && !(r.remaining > 0.0));
    r.contingency = r.starved || r.remaining <= r.reserve ? control::Contingency::FlightCritical : control::Contingency::Normal;
    if (!e->navigation.recovery) return r;
    // the return: at its best-range speed and the fuel it burns there (the tables, at its altitude and weight), else
    // at its cruise speed burning what it burns now
    const auto& nav = e->navigation;
    r.returnDistanceM = groundDistanceM(s.latitudeRad, s.longitudeRad, nav.latitudeDeg * kDeg, nav.longitudeDeg * kDeg);
    if (fuel && e->profile && !e->profile->tables.empty()) {
        const control::TablesAt at = control::tablesAt(e->profile->tables, s.altitudeMslM, en.massKg);
        r.returnTasMs = at.bestRangeTasMs, r.returnConsumption = at.bestRangeFuelKgS;
    }
    if (!(r.returnTasMs > 0.0)) r.returnTasMs = e->host.performance().cruiseTasMs;
    if (!(r.returnConsumption > 0.0)) r.returnConsumption = r.consumption;
    const double toReturn = r.returnDistanceM > 0.0 ? r.returnDistanceM / r.returnTasMs * r.returnConsumption : 0.0;
    const double spare = r.remaining - r.reserve - toReturn;
    if (!std::isfinite(spare)) return r; // (no speed to return at: no playtime)
    r.playtimeS = !(spare > 0.0) ? 0.0 : r.consumption > 0.0 ? spare / r.consumption : kForever;
    return r;
}

} // namespace fsim::session
