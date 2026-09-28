// A flight mode's performance profile (docs/flight-autonomy.md, 4.15; ADR-29
// FA-3c; A-GRA's MA_FlightControlModesPerformanceProfileType, VI 1.2.6.7):
// the airspeeds against altitude, the altitude limits, the accelerations,
// excess power, descents, decelerations and burn against speed and altitude,
// the attitudes, rates, turn and climb - worked out at the vehicle's
// condition now from its tables, its envelope and its loops.
#include "mode_flights.h"

#include "control/Atmosphere.h"
#include "fsim/PerformanceProfile.h"
#include "fsim/VehicleProfile.h"
#include "sim/FlightModel.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <string>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

bool same(double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(1.0, std::abs(b)); }

} // namespace

TEST_CASE("performance profile: a designed wing's, from its tables at its weight now", "[performance-profile]") {
    session::World w(options("profile-wing"));
    const auto id = wing(w, "f16c", 3000.0, 160.0);
    VelocityCommand level;
    level.airspeedMs = 160.0;
    level.verticalSpeedMs = 0.0;
    level.headingRad = kPi / 2.0;
    REQUIRE(w.submit(id, level).accepted());
    w.step(stepsFor(w, 5.0));
    PerformanceProfile p;
    REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, p) == Reason::None);
    const VehicleProfile& prof = *w.profile(id);
    const TablesSection& t = prof.tables;
    const Performance& perf = *w.performance(id);
    const sim::VehicleState& s = *w.vehicleState(id);
    const double mass = w.model(id)->energy().massKg;
    CHECK(p.mode == FlightMode::HsaCsa);
    CHECK(p.clean);
    CHECK(p.energy == Energy::Fuel);
    CHECK(p.weightKg == mass);
    CHECK(p.altitudeMslM == s.altitudeMslM);

    SECTION("the airspeeds against altitude: what it flies, within what it may") {
        REQUIRE(p.maxAirspeed.size() >= 3);
        CHECK(p.maxAirspeed.size() == p.minAirspeed.size());
        for (const ProfilePoint& q : p.maxAirspeed) {
            INFO(q.altitudeMslM);
            const TablesAt at = tablesAt(t, q.altitudeMslM, mass);
            CHECK(q.weightKg == mass);
            CHECK(std::isnan(q.tasMs)); // (an airspeed's own)
            CHECK(q.value <= at.maxTasMs * (1.0 + 1e-12));
            const double placard = std::isfinite(prof.envelope.clean.casMaxMs) ? isa::trueFromCalibrated(prof.envelope.clean.casMaxMs, q.altitudeMslM)
                                                                                 : at.maxTasMs;
            CHECK(q.value <= placard * (1.0 + 1e-12));
            CHECK((same(q.value, at.maxTasMs) || same(q.value, placard) ||
                   same(q.value, prof.envelope.clean.machMax * isa::speedOfSound(q.altitudeMslM))));
        }
        for (const ProfilePoint& q : p.minAirspeed) CHECK(q.value >= tablesAt(t, q.altitudeMslM, mass).minTasMs * (1.0 - 1e-12));
        REQUIRE_FALSE(p.bestRangeAirspeed.empty());
        const ProfilePoint& r = p.bestRangeAirspeed.front();
        CHECK(same(r.value, tablesAt(t, r.altitudeMslM, mass).bestRangeTasMs));
        CHECK(same(p.bestEnduranceAirspeed.front().value, tablesAt(t, p.bestEnduranceAirspeed.front().altitudeMslM, mass).bestEnduranceTasMs));
    }
    SECTION("the ceiling at the weight now; no floor") {
        CHECK(same(p.maxAltitudeMslM, tablesCeilingM(t, mass)));
        CHECK(p.maxAltitudeMslM > 10000.0);
        CHECK(std::isnan(p.minAltitudeMslM));
    }
    SECTION("against speed and altitude: the tables' excess power, idle descents and decelerations, burn") {
        REQUIRE(p.excessPower.size() >= 3 * t.speedFraction.size());
        for (const ProfileExcessPower& e : p.excessPower) {
            const TablesAtSpeed a = tablesAt(t, e.altitudeMslM, mass, e.tasMs);
            CHECK(same(e.climbMs, a.psFullMs));
            CHECK(same(e.accelerationMs2, a.psFullMs * isa::kG0 / e.tasMs));
        }
        REQUIRE_FALSE(p.burn.empty());
        for (const ProfilePoint& b : p.burn) {
            CHECK(b.value > 0.0);
            CHECK(same(b.value, tablesAt(t, b.altitudeMslM, mass, b.tasMs).fuelKgS));
        }
        REQUIRE_FALSE(p.maxDeceleration.empty());
        for (const ProfileAcceleration& d : p.maxDeceleration) CHECK(d.xMs2 < 0.0);
        for (const ProfilePoint& d : p.maxDescentRate) CHECK(d.value <= perf.maxDescentMs * (1.0 + 1e-12));
        // the load factors' accelerations with them: a pull is -z, body axes
        REQUIRE_FALSE(p.minAcceleration.empty());
        CHECK(same(p.minAcceleration.front().zMs2, -prof.envelope.clean.loadFactorMax * isa::kG0));
        CHECK(p.maxAcceleration.front().mach > 0.0);
    }
    SECTION("at the condition now: its attitudes, rates, turn and climb") {
        REQUIRE(p.maxOrientation.size() == 1);
        CHECK(p.maxOrientation.front().rollRad == (std::isfinite(prof.envelope.clean.bankMaxRad) ? prof.envelope.clean.bankMaxRad : perf.maxBankRad));
        CHECK(std::isnan(p.maxOrientation.front().yawRad));
        REQUIRE(p.maxOrientationRate.size() == 1);
        CHECK(same(p.maxTurnRateRadS, s.airspeedTrueMs / perf.turnRadiusM(s.airspeedTrueMs)));
        CHECK(p.maxClimbRateMs == perf.maxClimbMs);
    }
    SECTION("the three modes A-GRA profiles share it; another has none") {
        PerformanceProfile route, curve;
        REQUIRE(w.performanceProfile(id, FlightMode::WaypointFollowing, route) == Reason::None);
        REQUIRE(w.performanceProfile(id, FlightMode::CurveFollowing, curve) == Reason::None);
        CHECK(route.mode == FlightMode::WaypointFollowing);
        CHECK(route.maxAirspeed.size() == p.maxAirspeed.size());
        CHECK(curve.burn.size() == p.burn.size());
        CHECK(w.performanceProfile(id, FlightMode::Loiter, route) == Reason::InvalidParameter);
        CHECK(w.performanceProfile(999, FlightMode::HsaCsa, route) == Reason::UnknownVehicle);
    }
    SECTION("asked again into the same profile: its vectors reused") {
        const ProfilePoint* airspeeds = p.maxAirspeed.data();
        const ProfileExcessPower* power = p.excessPower.data();
        w.step(stepsFor(w, 1.0));
        REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, p) == Reason::None);
        CHECK(p.maxAirspeed.data() == airspeeds);
        CHECK(p.excessPower.data() == power);
        CHECK(p.weightKg < mass); // (it burns fuel)
    }
}

TEST_CASE("performance profile: its configuration - the flaps and the gear out, their placards, no clean tables", "[performance-profile]") {
    session::World w(options("profile-configuration"));
    const auto probe = wing(w, "f16c", 3000.0, 160.0);
    auto own = std::make_shared<VehicleProfile>(*w.profile(probe));
    own->envelope.flaps.casMaxMs = 200.0;
    own->envelope.gearCasMaxMs = 180.0;
    REQUIRE(w.removeVehicle(probe));
    session::VehicleSpec spec;
    spec.name = "configured";
    spec.type = "jsbsim:f16c";
    spec.initial.altitudeMslM = 3000.0;
    spec.initial.airspeedTrueMs = 150.0;
    spec.profile = own;
    const auto id = w.createVehicle(spec);
    REQUIRE(id != 0);
    w.step(stepsFor(w, 10.0)); // (spawned in the air with its gear down, it retracts it)
    PerformanceProfile clean;
    REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, clean) == Reason::None);
    INFO("flaps out " << clean.flapsOut << ", gear down " << clean.gearDown << " (position " << w.vehicleState(id)->gearPosition
         << ", flaps commanded " << w.inputs(id)->flaps << ")");
    REQUIRE(clean.clean);
    REQUIRE(w.submit(id, FlapsCommand{1.0}).accepted());
    REQUIRE(w.submit(id, GearCommand{1.0}).accepted());
    w.step(stepsFor(w, 8.0));
    PerformanceProfile p;
    REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, p) == Reason::None);
    CHECK(p.flapsOut);
    CHECK(p.gearDown);
    CHECK_FALSE(p.clean);
    // the placards bound its airspeed at the altitude now; what the tables give (flown clean) is left out
    REQUIRE(p.maxAirspeed.size() == 1);
    CHECK(same(p.maxAirspeed.front().value, isa::trueFromCalibrated(180.0, p.altitudeMslM)));
    CHECK(p.excessPower.empty());
    CHECK(p.burn.empty());
    CHECK(p.bestRangeAirspeed.empty());
    CHECK(std::isnan(p.maxAltitudeMslM));
    CHECK(clean.maxAirspeed.size() > 1);
}

TEST_CASE("performance profile: a rotorcraft's from the hover; a stock aircraft's from its Performance", "[performance-profile]") {
    session::World w(options("profile-others"));
    SECTION("the UH-60A: from the hover, its fuel, its climb, its tilt's accelerations") {
        const auto id = rotor(w, "uh60");
        PerformanceProfile p;
        REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, p) == Reason::None);
        const Performance& perf = *w.performance(id);
        REQUIRE_FALSE(p.minAirspeed.empty());
        CHECK(p.minAirspeed.front().value == 0.0); // (it hovers)
        CHECK(p.energy == Energy::Fuel);
        REQUIRE_FALSE(p.excessPower.empty());
        CHECK(p.excessPower.front().climbMs > 5.0);
        CHECK(std::isnan(p.excessPower.front().accelerationMs2)); // (a rotorcraft accelerates by its tilt)
        CHECK(p.maxAcceleration.front().xMs2 == perf.maxAccelerationMs2);
        CHECK(std::isnan(p.maxAltitudeMslM)); // (its ceiling far above the rows flown)
        CHECK(p.bestRangeAirspeed.front().value > 50.0);
    }
    SECTION("the IRIS+: its battery's power, no climb") {
        const auto id = rotor(w, "iris");
        PerformanceProfile p;
        REQUIRE(w.performanceProfile(id, FlightMode::WaypointFollowing, p) == Reason::None);
        CHECK(p.energy == Energy::Battery);
        REQUIRE_FALSE(p.burn.empty());
        CHECK(p.burn.front().value > 100.0); // W
        CHECK(p.excessPower.empty());
    }
    SECTION("the stock C172X: no tables - its Performance's airspeeds at the altitude now, no burn") {
        const auto id = wing(w, "c172x", 1500.0, 50.0);
        w.step(stepsFor(w, 1.0));
        PerformanceProfile p;
        REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, p) == Reason::None);
        CHECK(p.maxAirspeed.size() <= 1);
        CHECK(p.minAirspeed.size() <= 1);
        CHECK(p.excessPower.empty());
        CHECK(p.burn.empty());
        CHECK(p.bestRangeAirspeed.empty());
        REQUIRE(p.maxOrientation.size() == 1);
    }
}
