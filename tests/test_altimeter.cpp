// The barometric altimeter (docs/flight-autonomy.md, 4.20; A-GRA's QNH setting, VI 1.2.6.5, and MA_AirDataType,
// VI 1.2.6.8): the air as the flight model has it, the altimeter against the standard atmosphere's published values
// at a standard and a non-standard QNH, and a barometric altitude flown on its isobar in air that is not standard.
#include "mode_flights.h"

#include "core/Log.h"
#include "fsim/Altimeter.h"
#include "fsim/Capability.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimModel.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <variant>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kPsfToPa = 47.880258980335840; // (lbf/ft2: 4.4482216152605 N over 0.3048^2 m2)
constexpr double kRadius = 6356766.0;           // the standard atmosphere's, for geopotential heights

double code(AltitudeReference r) { return static_cast<double>(r); }
double geopotential(double h) { return kRadius * h / (kRadius + h); }

/// The ICAO standard atmosphere's pressure altitude in the troposphere, from its published constants, apart from
/// the platform's: (T0 / L) (1 - (p / p0) ^ (R L / g0)).
double troposphereAltitude(double p) {
    const double r = 8.31432 / 0.0289644, lapse = 0.0065;
    return 288.15 / lapse * (1.0 - std::pow(p / 101325.0, r * lapse / 9.80665));
}

Setpoint flown(const session::World& w, ActivityId a) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(a, s));
    return s;
}

} // namespace

TEST_CASE("altimeter: the air is the flight model's, standard or not", "[altimeter]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<sim::FlatGround>(0.0);
    for (const Air air : {Air{}, Air{303.15, 102000.0}, Air{263.15, 99000.0}}) {
        for (const double h : {0.0, 1500.0, 9000.0, 15000.0, 25000.0}) {
            INFO("air " << air.temperatureSeaLevelK << " K, " << air.pressureSeaLevelPa << " Pa, at " << h << " m");
            sim::JsbsimModel model(1.0 / 120.0, ground);
            sim::InitialConditions ic;
            ic.altitudeMslM = h + 2.0;
            ic.airspeedTrueMs = 60.0;
            REQUIRE(model.load(sim::AircraftSpec{"c172x", FSIM_TEST_JSBSIM_ROOT}, ic));
            model.setAtmosphere(air.temperatureSeaLevelK, air.pressureSeaLevelPa);
            sim::VehicleState before;
            model.state(before);
            model.step(sim::ControlInputs{}); // (its atmosphere is read where the step begins)
            const double p = model.property("atmosphere/P-psf").get() * kPsfToPa;
            const double t = model.property("atmosphere/T-R").get() / 1.8;
            // (to JSBSim's own rounding - its layers' bases kept in feet to 1e-4, their temperatures in Rankine: the worst
            // 2.1e-9 of the pressure, 7.8e-8 K)
            CHECK(std::abs(staticPressurePa(air, before.altitudeMslM) / p - 1.0) < 1e-8);
            CHECK(std::abs(staticTemperatureK(air, before.altitudeMslM) - t) < 1e-6);
            CHECK(std::abs(altitudeOfPressureM(air, p) - before.altitudeMslM) < 1e-4);
        }
    }
}

TEST_CASE("altimeter: what it reads against the standard atmosphere, at the standard setting and not", "[altimeter]") {
    // the standard's layer bases (the 1976 standard's published pressures): at 1013.25 hPa it reads them
    CHECK(std::abs(indicatedAltitudeM(101325.0, Altimeter::kStandardPa)) < 1e-9);
    CHECK(std::abs(indicatedAltitudeM(22632.06, Altimeter::kStandardPa) - 11000.0) < 0.01);
    CHECK(std::abs(indicatedAltitudeM(5474.889, Altimeter::kStandardPa) - 20000.0) < 0.01);
    CHECK(std::abs(indicatedAltitudeM(868.0187, Altimeter::kStandardPa) - 32000.0) < 0.01);
    // set to another pressure, the height between the two, as the standard atmosphere has it
    for (const double qnh : {99500.0, 101325.0, 103000.0})
        for (const double p : {101000.0, 90000.0, 70000.0, 40000.0, 25000.0}) {
            INFO(qnh << " Pa set, at " << p << " Pa");
            CHECK(std::abs(indicatedAltitudeM(p, qnh) - (troposphereAltitude(p) - troposphereAltitude(qnh))) < 1e-6);
            CHECK(std::abs(pressureIndicatingPa(indicatedAltitudeM(p, qnh), qnh) / p - 1.0) < 1e-12);
        }
    // 1000 hPa is 110.9 m up the standard atmosphere: set to it, an altimeter reads that much low at every height
    CHECK(std::abs(indicatedAltitudeM(100000.0, Altimeter::kStandardPa) - 110.886) < 0.01);
    const Altimeter low{Air{}, 100000.0};
    for (const double h : {0.0, 1000.0, 5000.0, 10000.0}) {
        INFO("at " << h << " m");
        CHECK(std::abs(indicatedAltitudeM(low, h) - (geopotential(h) - 110.886)) < 0.05); // (the flight model's gas constant is JSBSim's, 28.9645 g/mol)
        CHECK(std::abs(barometricMslM(low, indicatedAltitudeM(low, h)) - h) < 1e-6);
    }
    // in warm air, the isobar a reading is flown on is higher than the reading: 15 K above the standard, 5 % at 2 km
    const Altimeter warm{Air{303.15, 101325.0}, Altimeter::kStandardPa};
    CHECK(barometricMslM(warm, 2000.0) > 2090.0);
    CHECK(barometricMslM(warm, 2000.0) < 2115.0);
}

TEST_CASE("altimeter: an hsa and an orbit fly a barometric altitude on its isobar, and follow the setting", "[altimeter]") {
    session::World w(options("altimeter-hsa"));
    sim::EnvironmentState env = w.environment();
    env.temperatureSeaLevelK = 303.15, env.pressureSeaLevelPa = 102000.0; // warm, and high
    w.setEnvironment(env);
    const auto v = wing(w, "c172x", 2000.0, 55.0);
    CHECK(w.setQnh(v, 102000.0) == Reason::None);
    CHECK(w.qnh(v) == 102000.0);
    const Altimeter a{Air{303.15, 102000.0}, 102000.0};
    // a reference given alone: the reading now, held
    HsaCommand now;
    now.altitudeReference = code(AltitudeReference::Barometric);
    const ActivityId held = w.submit(v, now).activity;
    REQUIRE(held != 0);
    CHECK(std::abs(std::get<HsaCommand>(std::get<Command>(flown(w, held).command)).altitudeM - w.stateData(v).indicatedAltitudeM) < 0.5);
    // 2,000 m on the altimeter: flown on its isobar, 100 m above 2,000 m
    HsaCommand hsa;
    hsa.headingRad = 0.5 * kPi, hsa.altitudeM = 2000.0, hsa.altitudeReference = code(AltitudeReference::Barometric);
    const ActivityId flying = w.submit(v, hsa).activity;
    REQUIRE(flying != 0);
    w.step(stepsFor(w, 150.0));
    StateData d = w.stateData(v);
    CHECK(std::abs(d.indicatedAltitudeM - 2000.0) < 10.0);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - barometricMslM(a, 2000.0)) < 10.0);
    CHECK(w.vehicleState(v)->altitudeMslM - 2000.0 > 80.0);
    CHECK(d.kollsmanHpa == 1020.0);
    CHECK(std::abs(d.staticPressurePa - staticPressurePa(a.air, w.vehicleState(v)->altitudeMslM)) < 1e-6);
    // set 20 hPa lower, it reads 170 m less where it is: it climbs to read 2,000 m again
    CHECK(w.setQnh(v, 100000.0) == Reason::None);
    CHECK(std::abs(w.stateData(v).indicatedAltitudeM - (d.indicatedAltitudeM - (indicatedAltitudeM(100000.0, 101325.0) - indicatedAltitudeM(102000.0, 101325.0)))) < 1.0);
    w.step(stepsFor(w, 150.0));
    d = w.stateData(v);
    CHECK(std::abs(d.indicatedAltitudeM - 2000.0) < 10.0);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - barometricMslM(Altimeter{a.air, 100000.0}, 2000.0)) < 10.0);
    // its rate: what the reading does as it climbs
    HsaCommand up;
    up.altitudeM = 2600.0;
    REQUIRE(w.update(flying, Command(up)).accepted());
    w.step(stepsFor(w, 20.0));
    const double before = w.stateData(v).indicatedAltitudeM, rate = w.stateData(v).indicatedAltitudeRateMs;
    w.step(stepsFor(w, 1.0));
    const double after = w.stateData(v).indicatedAltitudeM;
    CHECK(rate > 1.0);
    CHECK(std::abs((after - before) - rate) < 0.1 * rate);
    // an orbit at 2,300 m on the altimeter
    PatternCommand orbit;
    orbit.altitudeM = 2300.0, orbit.altitudeReference = code(AltitudeReference::Barometric);
    REQUIRE(w.submit(v, orbit).accepted());
    w.step(stepsFor(w, 150.0));
    CHECK(std::abs(w.stateData(v).indicatedAltitudeM - 2300.0) < 15.0);
}

TEST_CASE("altimeter: the setting's range, a route's barometric altitude refused as FA-6's", "[altimeter]") {
    session::World w(options("altimeter-setting"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    CHECK(w.qnh(v) == Altimeter::kStandardPa); // (until set: the pressure altitude)
    CHECK(std::abs(w.stateData(v).indicatedAltitudeM - geopotential(w.vehicleState(v)->altitudeMslM)) < 0.05);
    CHECK(w.setQnh(v, 84000.0) == Reason::OutOfRange);
    CHECK(w.setQnh(v, 111000.0) == Reason::OutOfRange);
    CHECK(w.setQnh(v, std::nan("")) == Reason::OutOfRange);
    CHECK(w.qnh(v) == Altimeter::kStandardPa);
    CHECK(w.setQnh(999, 101000.0) == Reason::UnknownVehicle);
    CHECK(std::isnan(w.qnh(999)));
    CHECK(std::isnan(w.stateData(999).indicatedAltitudeM));
    const auto& s = *w.vehicleState(v);
    Waypoint p;
    p.latitudeRad = s.latitudeRad + 0.01, p.longitudeRad = s.longitudeRad, p.altitudeM = 1500.0;
    Waypoint q = p;
    q.latitudeRad += 0.01, q.altitudeReference = code(AltitudeReference::Barometric);
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p, q});
    CHECK(r.reason == Reason::NotImplemented);
    CHECK(r.index == 1);
}
