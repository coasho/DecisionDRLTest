// The state data beyond the vehicle's state (docs/flight-autonomy.md, 4.21; A-GRA's OrientationRate and
// OrientationAcceleration, WanderAngle and wind data, VI 1.2.6.8) and reference frames (A-GRA's ReferenceFrame and its
// relative points): the wind a vehicle measures against the world's, its Euler angles' rates and accelerations
// against their changes step by step, and points in fixed, moving and vehicle frames against the sphere's geometry,
// worked out here apart from the platform's.
#include "mode_flights.h"

#include "fsim/Frames.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

void setWindFrom(session::World& w, double fromDeg, double speedMs) {
    sim::EnvironmentState env = w.environment();
    env.windDirectionDeg = fromDeg, env.windSpeedMs = speedMs;
    w.setEnvironment(env);
}

} // namespace

TEST_CASE("state data: the wind a vehicle measures is the world's, flying and hovering", "[state_data]") {
    session::World w(options("state-wind"));
    setWindFrom(w, 270.0, 10.0); // (blowing east)
    const auto wing1 = wing(w, "c172x", 1500.0, 55.0);
    const auto rotorcraft = rotor(w, "iris", 5.0);
    w.step(stepsFor(w, 10.0));
    for (const auto v : {wing1, rotorcraft}) {
        const StateData d = w.stateData(v);
        INFO("vehicle " << v << ": " << d.windNorthMs << ", " << d.windEastMs << ", " << d.windDownMs);
        CHECK(std::abs(d.windNorthMs) < 1e-6);
        CHECK(std::abs(d.windEastMs - 10.0) < 1e-6);
        CHECK(std::abs(d.windDownMs) < 1e-6);
        CHECK(d.wanderAngleRad == 0.0);
    }
    setWindFrom(w, 0.0, 0.0);
    w.step(stepsFor(w, 2.0));
    CHECK(std::hypot(w.stateData(wing1).windNorthMs, w.stateData(wing1).windEastMs) < 1e-6);
}

TEST_CASE("state data: its Euler angles' rates and accelerations, against their changes step by step", "[state_data]") {
    auto o = options("state-rates");
    o.frameSkip = 1; // (a step of the flight model at a time)
    session::World w(o);
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    AttitudeCommand bank{0.6, 0.05, kHold, 0.785, kHold, 55.0}; // (rolling in, as the bench commands it)
    REQUIRE(w.submit(v, bank).accepted());
    const double dt = w.dt();
    std::vector<sim::VehicleState> states;
    std::vector<StateData> data;
    for (int k = 0; k < 360; ++k) {
        w.step();
        states.push_back(*w.vehicleState(v));
        data.push_back(w.stateData(v));
    }
    auto unwrap = [](double a) { return std::remainder(a, 2.0 * kPi); };
    double peakRate[3] = {0, 0, 0}, peakAcceleration[3] = {0, 0, 0}, worstRate[3] = {0, 0, 0}, worstAcceleration[3] = {0, 0, 0};
    for (std::size_t k = 1; k + 1 < states.size(); ++k) {
        const double rates[3] = {data[k].rollRateRadS, data[k].pitchRateRadS, data[k].yawRateRadS};
        const double accelerations[3] = {data[k].rollAccelerationRadS2, data[k].pitchAccelerationRadS2, data[k].yawAccelerationRadS2};
        const double previousRates[3] = {data[k - 1].rollRateRadS, data[k - 1].pitchRateRadS, data[k - 1].yawRateRadS};
        const double nextRates[3] = {data[k + 1].rollRateRadS, data[k + 1].pitchRateRadS, data[k + 1].yawRateRadS};
        for (int a = 0; a < 3; ++a) {
            const double changed = unwrap(states[k + 1].eulerRad[a] - states[k - 1].eulerRad[a]) / (2.0 * dt);
            const double rateChanged = (nextRates[a] - previousRates[a]) / (2.0 * dt);
            peakRate[a] = std::max(peakRate[a], std::abs(rates[a]));
            peakAcceleration[a] = std::max(peakAcceleration[a], std::abs(accelerations[a]));
            worstRate[a] = std::max(worstRate[a], std::abs(rates[a] - changed));
            worstAcceleration[a] = std::max(worstAcceleration[a], std::abs(accelerations[a] - rateChanged));
        }
    }
    for (int a = 0; a < 3; ++a) {
        INFO("axis " << a << ": rate " << worstRate[a] << " of " << peakRate[a] << ", acceleration " << worstAcceleration[a] << " of "
                     << peakAcceleration[a]);
        CHECK(peakRate[a] > 0.0);
        // (a central difference is itself off by half a step's worth of an acceleration that jumps - 0.0104 rad/s where
        // the roll begins, 2.5 rad/s2 in a step - and by half the jump in the acceleration's)
        CHECK(worstRate[a] < 0.01 * peakRate[a] + 0.5 * peakAcceleration[a] * dt);
        CHECK(worstAcceleration[a] < 0.1 * peakAcceleration[a] + 1e-3);
    }
    CHECK(peakRate[0] > 0.1); // (it rolled)
}

TEST_CASE("frames: points in a fixed frame, laid out and turned as A-GRA's offsets say", "[frames]") {
    session::World w(options("frames-fixed"));
    const double lat0 = 0.6, lon0 = -2.1, alt0 = 500.0;
    FrameSpec spec;
    spec.latitudeRad = lat0, spec.longitudeRad = lon0, spec.altitudeMslM = alt0;
    const FrameId f = w.createFrame(spec);
    REQUIRE(f != 0);
    auto at = [&](FrameId id, FrameRotation rotation, FrameOffsets offsets, double x, double y, double z) {
        const std::optional<GeoPoint> p = w.framePoint(id, FrameOffset{rotation, offsets, x, y, z});
        REQUIRE(p.has_value());
        return *p;
    };
    // the plane square to the vertical at the origin: 1 km north on it is atan(1000 / r) round the sphere, and higher
    GeoPoint p = at(f, FrameRotation::Unrotated, FrameOffsets::Cartesian, 1000.0, 0.0, 0.0);
    CHECK(std::abs(p.latitudeRad - (lat0 + std::atan(1000.0 / (kR + alt0)))) < 1e-12);
    CHECK(std::abs(p.longitudeRad - lon0) < 1e-12);
    CHECK(std::abs(p.altitudeMslM - (std::hypot(kR + alt0, 1000.0) - kR)) < 1e-6);
    // round the sphere: 1 km north is 1 km of latitude, at its height
    p = at(f, FrameRotation::Unrotated, FrameOffsets::GreatCircle, 1000.0, 0.0, -100.0);
    CHECK(std::abs(p.latitudeRad - (lat0 + 1000.0 / kR)) < 1e-12);
    CHECK(std::abs(p.altitudeMslM - (alt0 + 100.0)) < 1e-9);
    // east along the rhumb line: the parallel's
    p = at(f, FrameRotation::Unrotated, FrameOffsets::Rhumb, 0.0, 1000.0, 0.0);
    CHECK(std::abs(p.latitudeRad - lat0) < 1e-15);
    CHECK(std::abs(p.longitudeRad - (lon0 + 1000.0 / (kR * std::cos(lat0)))) < 1e-12);
    // turned by its yaw: forward, a quarter turn right, is east
    spec.yawRad = 0.5 * kPi;
    const FrameId turned = w.createFrame(spec);
    p = at(turned, FrameRotation::Yaw, FrameOffsets::GreatCircle, 1000.0, 0.0, 0.0);
    CHECK(std::abs(p.latitudeRad - lat0) < 1e-6);
    CHECK(p.longitudeRad > lon0 + 0.9 * 1000.0 / (kR * std::cos(lat0)));
    // its body's axes: pitched up 30 degrees, 1 km forward is 866 m north and 500 m up
    spec.yawRad = 0.0, spec.pitchRad = 30.0 * kDeg;
    const FrameId pitched = w.createFrame(spec);
    p = at(pitched, FrameRotation::Attitude, FrameOffsets::GreatCircle, 1000.0, 0.0, 0.0);
    CHECK(std::abs(p.latitudeRad - (lat0 + 1000.0 * std::cos(30.0 * kDeg) / kR)) < 1e-12);
    CHECK(std::abs(p.altitudeMslM - (alt0 + 500.0)) < 1e-9);
    // refused: a latitude off the Earth, a value not finite
    FrameSpec bad = spec;
    bad.latitudeRad = 2.0;
    CHECK(w.createFrame(bad) == 0);
    bad = spec, bad.northMs = std::nan("");
    CHECK(w.createFrame(bad) == 0);
    CHECK(w.frame(f)->latitudeRad == lat0);
    CHECK(w.removeFrame(f));
    CHECK_FALSE(w.removeFrame(f));
    CHECK_FALSE(w.framePoint(f, FrameOffset{}).has_value());
}

TEST_CASE("frames: a moving frame and a vehicle's, now and at another time", "[frames]") {
    session::World w(options("frames-moving"));
    // moving north at 10 m/s from time 0: 100 s on, 1 km north; its heading turns an offset along its track
    FrameSpec moving;
    moving.origin = FrameOrigin::Moving;
    moving.latitudeRad = 0.6, moving.longitudeRad = -2.1, moving.altitudeMslM = 100.0, moving.northMs = 10.0, moving.downMs = -1.0;
    const FrameId m = w.createFrame(moving);
    REQUIRE(m != 0);
    std::optional<GeoPoint> p = w.framePoint(m, FrameOffset{}, 100.0);
    REQUIRE(p.has_value());
    CHECK(std::abs(p->latitudeRad - (0.6 + 1000.0 / kR)) < 1e-12);
    CHECK(std::abs(p->altitudeMslM - 200.0) < 1e-9);
    p = w.framePoint(m, FrameOffset{FrameRotation::Heading, FrameOffsets::GreatCircle, 500.0, 0.0, 0.0}, 0.0); // (its track: north)
    CHECK(std::abs(p->latitudeRad - (0.6 + 500.0 / kR)) < 1e-12);
    // a vehicle's: where it is now, carried on at its velocity to another time; gone with it
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    FrameSpec follows;
    follows.origin = FrameOrigin::Vehicle, follows.vehicle = v;
    const FrameId fv = w.createFrame(follows);
    REQUIRE(fv != 0);
    const sim::VehicleState s = *w.vehicleState(v);
    p = w.framePoint(fv, FrameOffset{});
    REQUIRE(p.has_value());
    CHECK((std::abs(p->latitudeRad - s.latitudeRad) < 1e-12 && std::abs(p->longitudeRad - s.longitudeRad) < 1e-12 &&
           std::abs(p->altitudeMslM - s.altitudeMslM) < 1e-6));
    const double later = w.simTime() + 10.0;
    p = w.framePoint(fv, FrameOffset{}, later);
    const double moved = std::hypot((p->latitudeRad - s.latitudeRad) * kR, (p->longitudeRad - s.longitudeRad) * kR * std::cos(s.latitudeRad));
    CHECK(std::abs(moved - 10.0 * std::hypot(s.velocityNedMs[0], s.velocityNedMs[1])) < 0.01);
    // its body's axes: 100 m ahead of its nose, on its heading
    p = w.framePoint(fv, FrameOffset{FrameRotation::Yaw, FrameOffsets::GreatCircle, 100.0, 0.0, 0.0});
    const double dLon = p->longitudeRad - s.longitudeRad; // (the great circle's initial bearing)
    const double bearing = std::atan2(std::sin(dLon) * std::cos(p->latitudeRad),
                                      std::cos(s.latitudeRad) * std::sin(p->latitudeRad) - std::sin(s.latitudeRad) * std::cos(p->latitudeRad) * std::cos(dLon));
    CHECK(std::abs(std::remainder(bearing - s.eulerRad[2], 2.0 * kPi)) < 1e-9);
    FrameSpec nobody = follows;
    nobody.vehicle = 999;
    CHECK(w.createFrame(nobody) == 0);
    REQUIRE(w.removeVehicle(v));
    CHECK_FALSE(w.framePoint(fv, FrameOffset{}).has_value());
}
