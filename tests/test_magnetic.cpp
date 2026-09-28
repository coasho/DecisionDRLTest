// The Earth's magnetic field (docs/flight-autonomy.md, 4.22): the World Magnetic Model 2025 against NOAA's published test
// values - the twelve of its technical report, and the hundred of its coefficient package (tests/data/
// wmm2025_test_values.txt, WMM2025_TestValues.txt from WMM2025COF.zip: NOAA NCEI, a work of the US Government in the
// public domain) - within the rounding they are published to.
#include "mode_flights.h"

#include "fsim/Magnetic.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

double code(DirectionReference r) { return static_cast<double>(r); }
double headingOff(double a, double b) { return std::abs(std::remainder(a - b, 2.0 * kPi)); }

} // namespace

TEST_CASE("magnetic: the model against its technical report's test values", "[magnetic]") {
    // the date, height (km), latitude and longitude (deg); X, Y, Z, H and F (nT); inclination and declination (deg)
    struct Published {
        double year, km, lat, lon, x, y, z, h, f, incl, decl;
    };
    const Published values[] = {
        {2025.0, 0.0, 80.0, 0.0, 6521.6, 145.9, 54791.5, 6523.2, 55178.5, 83.21, 1.28},
        {2025.0, 0.0, 0.0, 120.0, 39677.8, -109.6, -10580.2, 39677.9, 41064.3, -14.93, -0.16},
        {2025.0, 0.0, -80.0, 240.0, 6117.5, 15751.9, -52022.5, 16898.1, 54698.2, -72.00, 68.78},
        {2025.0, 100.0, 80.0, 0.0, 6216.0, 92.4, 52598.8, 6216.7, 52964.9, 83.26, 0.85},
        {2025.0, 100.0, 0.0, 120.0, 37688.6, -96.2, -10152.1, 37688.7, 39032.1, -15.08, -0.15},
        {2025.0, 100.0, -80.0, 240.0, 5907.6, 14780.3, -49540.7, 15917.1, 52035.0, -72.19, 68.21},
        {2027.5, 0.0, 80.0, 0.0, 6500.8, 294.5, 54869.4, 6507.5, 55253.9, 83.24, 2.59},
        {2027.5, 0.0, 0.0, 120.0, 39701.6, -167.4, -10381.8, 39702.0, 41036.9, -14.65, -0.24},
        {2027.5, 0.0, -80.0, 240.0, 6200.7, 15730.3, -51783.7, 16908.3, 54474.2, -71.92, 68.49},
        {2027.5, 100.0, 80.0, 0.0, 6196.7, 233.8, 52670.5, 6201.1, 53034.3, 83.29, 2.16},
        {2027.5, 100.0, 0.0, 120.0, 37711.5, -148.7, -9969.8, 37711.8, 39007.4, -14.81, -0.23},
        {2027.5, 100.0, -80.0, 240.0, 5984.0, 14760.1, -49317.7, 15927.0, 51825.7, -72.10, 67.93},
    };
    for (const Published& v : values) {
        INFO(v.year << " at " << v.km << " km, " << v.lat << ", " << v.lon);
        const MagneticField m = magneticField(v.lat * kDeg, v.lon * kDeg, v.km * 1000.0, v.year);
        // (half the last figure they are given to)
        CHECK(std::abs(m.northNt - v.x) <= 0.05 + 1e-6);
        CHECK(std::abs(m.eastNt - v.y) <= 0.05 + 1e-6);
        CHECK(std::abs(m.downNt - v.z) <= 0.05 + 1e-6);
        CHECK(std::abs(m.horizontalNt - v.h) <= 0.05 + 1e-6);
        CHECK(std::abs(m.totalNt - v.f) <= 0.05 + 1e-6);
        CHECK(std::abs(m.inclinationRad / kDeg - v.incl) <= 0.005 + 1e-9);
        CHECK(std::abs(m.declinationRad / kDeg - v.decl) <= 0.005 + 1e-9);
        CHECK(declinationRad(v.lat * kDeg, v.lon * kDeg, v.km * 1000.0, v.year) == m.declinationRad);
    }
}

TEST_CASE("magnetic: the model against its coefficient package's hundred test values", "[magnetic]") {
    std::ifstream in(std::string(FSIM_TEST_DATA_DIR) + "/wmm2025_test_values.txt");
    REQUIRE(in.good());
    int count = 0;
    double worstNt = 0.0, worstDeg = 0.0;
    for (std::string line; std::getline(in, line);) {
        if (line.empty() || line[0] == '#') continue;
        // the date, height (km), latitude and longitude (deg); declination and inclination (deg); H, X, Y, Z and F (nT)
        std::istringstream row(line);
        double year = 0, km = 0, lat = 0, lon = 0, decl = 0, incl = 0, h = 0, x = 0, y = 0, z = 0, f = 0;
        row >> year >> km >> lat >> lon >> decl >> incl >> h >> x >> y >> z >> f;
        REQUIRE(row.good());
        INFO(year << " at " << km << " km, " << lat << ", " << lon);
        const MagneticField m = magneticField(lat * kDeg, lon * kDeg, km * 1000.0, year);
        for (const auto& [ours, theirs] : {std::pair{m.horizontalNt, h}, {m.northNt, x}, {m.eastNt, y}, {m.downNt, z}, {m.totalNt, f}})
            worstNt = std::max(worstNt, std::abs(ours - theirs));
        worstDeg = std::max({worstDeg, std::abs(m.declinationRad / kDeg - decl), std::abs(m.inclinationRad / kDeg - incl)});
        ++count;
    }
    INFO("the worst: " << worstNt << " nT, " << worstDeg << " deg");
    CHECK(count == 100);
    CHECK(worstNt < 1e-3);        // (given to a millionth of a nT)
    CHECK(worstDeg <= 0.005 + 1e-9); // (given to a hundredth of a degree)
}

TEST_CASE("magnetic: a UTC time as a decimal year", "[magnetic]") {
    CHECK(decimalYear(0.0) == 1970.0);
    CHECK(decimalYear(1735689600.0) == 2025.0);                               // 2025-01-01T00:00Z
    CHECK(std::abs(decimalYear(1735689600.0 + 182.5 * 86400.0) - 2025.5) < 1e-12); // (2025 has 365 days)
    CHECK(std::abs(decimalYear(1830297600.0 + 183.0 * 86400.0) - 2028.5) < 1e-12); // (2028 has 366)
    CHECK(std::abs(decimalYear(-86400.0) - (1969.0 + 364.0 / 365.0)) < 1e-12);
}

TEST_CASE("magnetic: an hsa flies a magnetic heading and course, turned by the declination where it is", "[magnetic]") {
    session::World w(options("magnetic-hsa"));
    sim::EnvironmentState env = w.environment();
    env.epochUtcSeconds = 1767225600.0; // 2026-01-01T00:00Z
    w.setEnvironment(env);
    const auto v = wing(w, "c172x", 1500.0, 55.0); // (off San Francisco: some 13 degrees east)
    auto declination = [&] {
        const auto& s = *w.vehicleState(v);
        return declinationRad(s.latitudeRad, s.longitudeRad, s.altitudeMslM, decimalYear(1767225600.0 + w.simTime()));
    };
    const double d0 = declination();
    CHECK(d0 > 12.0 * kDeg);
    CHECK(d0 < 14.0 * kDeg);
    // magnetic north: commanded 13 degrees east of true north (the declination where it is, refreshed every 10 s),
    // and flown there - as closely as its heading hold flies a true heading (the stock C172x's settles 1.6 degrees off
    // this turn, the same told true north); its state data reads its heading from magnetic north
    HsaCommand north;
    north.headingRad = 0.0, north.directionReference = code(DirectionReference::MagneticNorth);
    const ActivityId a = w.submit(v, north).activity;
    REQUIRE(a != 0);
    w.step(stepsFor(w, 90.0));
    const StateData data = w.stateData(v);
    CHECK(headingOff(w.commandState(v).headingRad, declination()) < 0.01 * kDeg);
    CHECK(headingOff(w.vehicleState(v)->eulerRad[2], declination()) < 2.0 * kDeg);
    CHECK(headingOff(data.magneticHeadingRad, w.vehicleState(v)->eulerRad[2] - declination()) < 1e-9);
    CHECK(std::abs(data.declinationRad - declination()) < 1e-12);
    REQUIRE(w.activity(a) != nullptr);
    CHECK(headingOff(w.activity(a)->progress.headingRad, 0.0) < 1e-12); // (in the reference it was commanded in)
    // an UPDATE's heading alone continues its reference; a reference alone has no value to take
    HsaCommand east;
    east.headingRad = 0.5 * kPi;
    REQUIRE(w.update(a, Command(east)).accepted());
    w.step(stepsFor(w, 90.0));
    CHECK(headingOff(w.commandState(v).headingRad, 0.5 * kPi + declination()) < 0.01 * kDeg);
    CHECK(headingOff(w.stateData(v).magneticHeadingRad, 0.5 * kPi) < 2.0 * kDeg);
    HsaCommand alone;
    alone.directionReference = code(DirectionReference::TrueNorth);
    CommandResult r = w.update(a, Command(alone));
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 0);
    // a course from magnetic north: the track over the ground, 13 degrees east of it
    HsaCommand course;
    course.courseRad = 0.0, course.directionReference = code(DirectionReference::MagneticNorth);
    REQUIRE(w.submit(v, course).accepted());
    w.step(stepsFor(w, 90.0));
    const auto& s = *w.vehicleState(v);
    CHECK(headingOff(std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]), declination()) < 1.0 * kDeg); // (a course is trimmed on its track)
    // a reference given alone in a NEW: the heading it flies now, from that north
    HsaCommand now;
    now.directionReference = code(DirectionReference::MagneticNorth);
    const ActivityId held = w.submit(v, now).activity;
    REQUIRE(held != 0);
    Setpoint flown;
    REQUIRE(w.activitySetpoint(held, flown));
    CHECK(headingOff(std::get<HsaCommand>(std::get<Command>(flown.command)).headingRad, s.eulerRad[2] - declination()) < 1e-6);
    // not a reference
    HsaCommand bad;
    bad.headingRad = 0.0, bad.directionReference = 2.0;
    r = w.submit(v, bad);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 7);
}

TEST_CASE("magnetic: a world's date, held within the model's five years", "[magnetic]") {
    CHECK(magneticYear(0.0) == kMagneticEpochYear);           // (a clock never set: 1970)
    CHECK(magneticYear(1767225600.0) == decimalYear(1767225600.0)); // 2026
    CHECK(magneticYear(2000000000.0) == kMagneticValidUntilYear);   // 2033
}
