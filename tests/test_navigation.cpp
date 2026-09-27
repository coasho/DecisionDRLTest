// The navigation report (docs/flight-autonomy.md, 4.14; ADR-29 FA-3b; A-GRA's
// MA_NavigationReport, STS-07): what a vehicle flies on and how much it has -
// fuel in its tanks or a battery's charge - its consumption now, which is
// what leaves them, its endurance at that, its playtime to a recovery point
// and its contingency.
#include "mode_flights.h"

#include "fsim/VehicleProfile.h"
#include "sim/FlightModel.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <tuple>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

/// Its consumption as the report gives it, step by step for `seconds`: what
/// it says left its tanks or battery.
double consumed(session::World& w, std::uint32_t id, double seconds) {
    const double dt = w.dt() * w.frameSkip();
    double used = 0.0;
    for (unsigned i = 0, n = stepsFor(w, seconds); i < n; ++i) {
        w.step(1);
        used += w.navigationReport(id).consumption * dt;
    }
    return used;
}

/// The same report, field for field: its identities.
void checkIdentities(const NavigationReport& r) {
    CHECK(r.remaining >= 0.0);
    CHECK(r.remaining <= r.capacity * (1.0 + 1e-12));
    CHECK(std::abs(r.percent - 100.0 * r.remaining / r.capacity) < 1e-9);
    if (r.remaining > 0.0 && r.consumption > 0.0) CHECK(std::abs(r.enduranceS - r.remaining / r.consumption) <= 1e-9 * r.enduranceS);
    CHECK(std::abs(r.reserve - 0.1 * r.capacity) <= 1e-9 * r.capacity); // (the reserve by default)
    CHECK(std::isnan(r.playtimeS));                                      // (no recovery point)
    CHECK(std::isnan(r.returnDistanceM));
}

double groundM(double lat1, double lon1, double lat2, double lon2) {
    const double a = std::sin(0.5 * (lat2 - lat1)), b = std::sin(0.5 * (lon2 - lon1));
    return 2.0 * 6371008.8 * std::asin(std::sqrt(a * a + std::cos(lat1) * std::cos(lat2) * b * b));
}

} // namespace

TEST_CASE("navigation: fuel - a designed wing's and a stock aircraft's, the consumption what leaves the tanks", "[navigation]") {
    session::World w(options("navigation-fuel"));
    for (const char* type : {"c172", "c172x", "f16c"}) {
        INFO(type);
        const bool fast = std::string(type) == "f16c";
        const auto id = wing(w, type, 1500.0, fast ? 180.0 : 50.0);
        VelocityCommand level;
        level.airspeedMs = fast ? 180.0 : 50.0;
        level.verticalSpeedMs = 0.0;
        level.headingRad = kPi / 2.0;
        REQUIRE(w.submit(id, level).accepted());
        w.step(stepsFor(w, 20.0));
        const NavigationReport a = w.navigationReport(id);
        CHECK(a.energy == Energy::Fuel);
        CHECK(a.fuelKg == a.remaining);
        CHECK(a.remaining > 0.0);
        CHECK(a.consumption > 0.0);
        CHECK(a.contingency == Contingency::Normal);
        CHECK_FALSE(a.starved);
        checkIdentities(a);
        // what leaves the tanks, step by step, is what the report says it burns
        const double used = consumed(w, id, 30.0);
        const NavigationReport b = w.navigationReport(id);
        CHECK(std::abs((a.remaining - b.remaining) / used - 1.0) < 1e-3);
        // its designed capacity: the performance tables' (hangar's wings)
        if (const auto* p = w.profile(id); p && !p->tables.empty())
            CHECK(std::abs(a.capacity - p->tables.fuelCapacityKg) < 1e-3 * a.capacity);
    }
}

TEST_CASE("navigation: a helicopter's fuel and a quadrotor's battery, each against its flight", "[navigation]") {
    session::World w(options("navigation-rotorcraft"));
    SECTION("the UH-60A's fuel in the hover") {
        const auto id = rotor(w, "uh60");
        const NavigationReport a = w.navigationReport(id);
        CHECK(a.energy == Energy::Fuel);
        CHECK(a.capacity > 1000.0);                                // (360 US gal of JP-4)
        CHECK(a.consumption > 0.1);                                // kg/s
        CHECK(a.consumption < 0.2);
        checkIdentities(a);
        const double used = consumed(w, id, 60.0);
        const NavigationReport b = w.navigationReport(id);
        CHECK(std::abs((a.remaining - b.remaining) / used - 1.0) < 1e-3);
        CHECK(std::abs((a.remaining - b.remaining) / (60.0 * a.consumption) - 1.0) < 0.02); // (its hover: steady)
    }
    SECTION("the Crazyflie's battery: its endurance flown, then spent") {
        const auto id = rotor(w, "cf2", 20.0);
        const NavigationReport a = w.navigationReport(id);
        CHECK(a.energy == Energy::Battery);
        CHECK(a.fuelKg == 0.0);
        CHECK(a.capacity > 3000.0);                                // J: 240 mAh at 3.7 V
        CHECK(a.consumption > 5.0);                                // W
        CHECK(a.consumption < 10.0);
        checkIdentities(a);
        double t = 0.0;
        NavigationReport r = a;
        while (r.remaining > 0.0 && t < 2.0 * a.enduranceS) {
            w.step(stepsFor(w, 1.0));
            t += 1.0;
            r = w.navigationReport(id);
            if (r.remaining <= a.reserve) CHECK(r.contingency == Contingency::FlightCritical);
        }
        CHECK(std::abs(t / a.enduranceS - 1.0) < 0.01);          // (a hover's power: steady)
        CHECK(r.remaining == 0.0);
        CHECK(r.enduranceS == 0.0);                                // (nothing left: none, whatever it draws)
        CHECK(r.starved);
        CHECK(r.contingency == Contingency::FlightCritical);
    }
}

TEST_CASE("navigation: an aircraft that flies on neither", "[navigation]") {
    session::World w(options("navigation-neither"));
    const auto id = wing(w, "SGS", 1500.0, 30.0);             // (JSBSim's glider: no tanks, no engine)
    const NavigationReport r = w.navigationReport(id);
    CHECK(r.energy == Energy::Unknown);
    CHECK(std::isnan(r.remaining));
    CHECK(std::isnan(r.enduranceS));
    CHECK(r.contingency == Contingency::Normal);
    CHECK_FALSE(r.starved);
    CHECK(w.navigationReport(999).energy == Energy::Unknown);  // (no such vehicle)
}

TEST_CASE("navigation: the playtime to a recovery point, the reserve, the contingency", "[navigation]") {
    session::World w(options("navigation-playtime"));
    const auto id = wing(w, "c172", 1500.0, 50.0);
    VelocityCommand level;
    level.airspeedMs = 50.0;
    level.verticalSpeedMs = 0.0;
    level.headingRad = kPi / 2.0;
    REQUIRE(w.submit(id, level).accepted());
    w.step(stepsFor(w, 20.0));

    NavigationSettings home;
    home.recovery = true;
    const sim::VehicleState& s = *w.vehicleState(id);
    home.latitudeDeg = s.latitudeRad / kDeg;
    home.longitudeDeg = s.longitudeRad / kDeg - 0.3;              // (behind it: some 29 km west)
    home.altitudeMslM = 300.0;
    REQUIRE(w.setNavigation(id, home) == Reason::None);
    CHECK(w.navigation(id).recovery);
    CHECK(w.navigation(id).longitudeDeg == home.longitudeDeg);

    const NavigationReport r = w.navigationReport(id);
    const double far = groundM(s.latitudeRad, s.longitudeRad, home.latitudeDeg * kDeg, home.longitudeDeg * kDeg);
    CHECK(std::abs(r.returnDistanceM - far) < 1.0);
    CHECK(r.returnDistanceM > 25000.0);
    // its best-range speed and fuel flow from its tables, at its height and weight
    const TablesAt at = tablesAt(w.profile(id)->tables, s.altitudeMslM, w.model(id)->energy().massKg);
    CHECK(r.returnTasMs == at.bestRangeTasMs);
    CHECK(r.returnConsumption == at.bestRangeFuelKgS);
    const double toReturn = r.returnDistanceM / r.returnTasMs * r.returnConsumption;
    CHECK(std::abs(r.playtimeS - (r.remaining - r.reserve - toReturn) / r.consumption) < 1e-6 * r.playtimeS);
    CHECK(r.playtimeS > 0.0);
    CHECK(r.playtimeS < r.enduranceS);
    CHECK(r.contingency == Contingency::Normal);

    SECTION("a reserve of all it has: flight critical, no playtime; just below it, normal") {
        NavigationSettings most = home;
        most.reserveFraction = r.remaining / r.capacity + 1e-4; // (it is nearly full)
        REQUIRE(w.setNavigation(id, most) == Reason::None);
        const NavigationReport c = w.navigationReport(id);
        CHECK(c.contingency == Contingency::FlightCritical);
        CHECK(c.playtimeS == 0.0);
        CHECK(c.enduranceS > 0.0);                               // (it still flies: the reserve is kept, not gone)
        most.reserveFraction = r.remaining / r.capacity - 1e-3;
        REQUIRE(w.setNavigation(id, most) == Reason::None);
        const NavigationReport d = w.navigationReport(id);
        CHECK(d.contingency == Contingency::Normal);
        CHECK(d.playtimeS == 0.0);                               // (the way back needs more than it has over the reserve)
    }
    SECTION("the recovery point cleared: no playtime, the reserve kept") {
        NavigationSettings none;
        none.reserveFraction = 0.25;
        REQUIRE(w.setNavigation(id, none) == Reason::None);
        const NavigationReport c = w.navigationReport(id);
        CHECK(std::isnan(c.playtimeS));
        CHECK(std::isnan(c.returnDistanceM));
        CHECK(std::abs(c.reserve - 0.25 * c.capacity) < 1e-9 * c.capacity);
    }
    SECTION("its tanks emptied: starved, flight critical, no endurance") {
        auto* m = w.model(id);
        for (int i = 0;; ++i) {
            auto tank = m->property("propulsion/tank[" + std::to_string(i) + "]/contents-lbs");
            if (!tank.valid()) break;
            tank.set(0.0);
        }
        w.step(stepsFor(w, 1.0));
        const NavigationReport c = w.navigationReport(id);
        CHECK(c.remaining == 0.0);
        CHECK(c.percent == 0.0);
        CHECK(c.starved);
        CHECK(c.enduranceS == 0.0);
        CHECK(c.playtimeS == 0.0);
        CHECK(c.contingency == Contingency::FlightCritical);
    }
    SECTION("what it refuses: a reserve outside [0, 1), a point off the Earth, no such vehicle") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (double reserve : {-0.1, 1.0, nan}) {
            NavigationSettings bad = home;
            bad.reserveFraction = reserve;
            CHECK(w.setNavigation(id, bad) == Reason::OutOfRange);
        }
        for (auto [lat, lon, alt] : {std::tuple{91.0, 0.0, 0.0}, {0.0, 181.0, 0.0}, {0.0, 0.0, nan}, {nan, 0.0, 0.0}}) {
            NavigationSettings bad = home;
            bad.latitudeDeg = lat, bad.longitudeDeg = lon, bad.altitudeMslM = alt;
            CHECK(w.setNavigation(id, bad) == Reason::OutOfRange);
        }
        NavigationSettings noPoint;                                 // (a point unset needs none)
        noPoint.latitudeDeg = 91.0;
        CHECK(w.setNavigation(id, noPoint) == Reason::None);
        CHECK(w.setNavigation(999, home) == Reason::UnknownVehicle);
        CHECK(std::string(energyName(Energy::Battery)) == "battery");
        CHECK(std::string(contingencyName(Contingency::FlightCritical)) == "FLIGHT_CRITICAL");
    }
}
