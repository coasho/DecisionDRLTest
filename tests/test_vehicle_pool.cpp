// Integration tests against the real JSBSim data tree (submodule). They are
// the M1 exit criteria in miniature: vehicles load, fly, and produce
// bit-identical trajectories regardless of how many workers step them.

#include "core/Log.h"
#include "core/Rng.h"
#include "sim/Attitude.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimModel.h"
#include "sim/VehiclePool.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <vector>

using namespace fsim;
using namespace fsim::sim;

namespace {

const std::filesystem::path kRoot = FSIM_TEST_JSBSIM_ROOT;
constexpr double kDt = 1.0 / 120.0;

InitialConditions icFor(unsigned i) {
    Rng rng = Rng::forVehicle(99, 0, i);
    InitialConditions ic;
    ic.latitudeDeg += rng.uniform(-0.1, 0.1);
    ic.longitudeDeg += rng.uniform(-0.1, 0.1);
    ic.altitudeMslM = 1200.0 + rng.uniform(-100.0, 100.0);
    ic.headingDeg = rng.uniform(0.0, 360.0);
    ic.airspeedTrueMs = 55.0;
    return ic;
}

std::vector<VehicleState> fly(unsigned vehicles, unsigned workers, unsigned agentSteps, int frameSkip) {
    auto ground = std::make_shared<FlatGround>(0.0);
    VehiclePool pool(workers);
    for (unsigned i = 0; i < vehicles; ++i) {
        auto m = std::make_unique<JsbsimModel>(kDt, ground);
        REQUIRE(m->load(AircraftSpec{"c172x", kRoot}, icFor(i)));
        pool.add(std::move(m));
    }
    std::vector<ControlInputs> inputs(vehicles);
    for (unsigned i = 0; i < vehicles; ++i) {
        inputs[i].setThrottleAll(0.6);
        inputs[i].gearDown = 0.0;
        inputs[i].aileron = (i % 2 ? 0.05 : -0.05);
    }
    for (unsigned s = 0; s < agentSteps; ++s) pool.step(Span<const ControlInputs>(inputs), frameSkip);
    const auto states = pool.states();
    return std::vector<VehicleState>(states.begin(), states.end());
}

bool bitIdentical(const VehicleState& a, const VehicleState& b) {
    return std::memcmp(&a, &b, sizeof(VehicleState)) == 0;
}

} // namespace

TEST_CASE("c172x loads and flies with sane state", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 1000.0;
    ic.airspeedTrueMs = 55.0;
    ic.headingDeg = 90.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    REQUIRE(model.loaded());
    CHECK(model.dt() == kDt);

    VehicleState s;
    model.state(s);
    CHECK(std::abs(s.altitudeMslM - 1000.0) < 1.0);
    CHECK(std::abs(s.airspeedTrueMs - 55.0) < 0.5);
    CHECK(s.engineCount == 1);
    CHECK_FALSE(s.diverged);
    CHECK_FALSE(s.onGround);
    // ECEF radius ~ Earth radius + altitude
    const double r = std::sqrt(s.positionEcef[0] * s.positionEcef[0] + s.positionEcef[1] * s.positionEcef[1] +
                               s.positionEcef[2] * s.positionEcef[2]);
    CHECK(r > 6.35e6);
    CHECK(r < 6.40e6);
    // unit quaternion
    double n = 0;
    for (double q : s.attitudeEcefToBody) n += q * q;
    CHECK(std::abs(n - 1.0) < 1e-9);

    ControlInputs in;
    in.setThrottleAll(0.7);
    in.gearDown = 0.0;
    for (int i = 0; i < 120; ++i) model.step(in); // 1 s
    model.state(s);
    CHECK(std::abs(s.simTime - 1.0) < 1e-9);
    CHECK(s.stepCount == 120);
    CHECK_FALSE(s.diverged);
    CHECK(s.airspeedTrueMs > 40.0);
    CHECK(s.altitudeMslM > 900.0);
}

TEST_CASE("property handles read live JSBSim values", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, InitialConditions{}));

    auto vc = model.property("velocities/vc-kts");
    REQUIRE(vc.valid());
    CHECK(vc.get() > 50.0);

    auto bogus = model.property("no/such/property");
    CHECK_FALSE(bogus.valid());
    CHECK(bogus.get() == 0.0);
}

TEST_CASE("reset returns the vehicle to initial conditions", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    VehicleState first;
    model.state(first);

    ControlInputs in;
    in.setThrottleAll(1.0);
    in.elevator = -0.3;
    for (int i = 0; i < 600; ++i) model.step(in);
    VehicleState flown;
    model.state(flown);
    REQUIRE(std::abs(flown.altitudeMslM - first.altitudeMslM) > 5.0);

    REQUIRE(model.reset(ic));
    VehicleState after;
    model.state(after);
    CHECK(after.stepCount == 0);
    CHECK(std::abs(after.altitudeMslM - first.altitudeMslM) < 0.5);
    CHECK(std::abs(after.airspeedTrueMs - first.airspeedTrueMs) < 0.5);
    CHECK(std::abs(after.simTime) < 1e-9);
}

TEST_CASE("ground provider drives AGL and gear contact", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(500.0); // terrain at 500 m
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 1500.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    VehicleState s;
    model.state(s);
    CHECK(std::abs(s.altitudeAglM - 1000.0) < 2.0);

    // Start on the ground: JSBSim should place the gear on the provider's terrain.
    InitialConditions onGround;
    onGround.onGround = true;
    onGround.airspeedTrueMs = 0.0;
    JsbsimModel parked(kDt, ground);
    REQUIRE(parked.load(AircraftSpec{"c172x", kRoot}, onGround));
    ControlInputs idle;
    idle.brakeLeft = idle.brakeRight = 1.0;
    for (int i = 0; i < 600; ++i) parked.step(idle); // settle 5 s on the gear springs
    bool touchedGround = false;
    for (int i = 0; i < 120; ++i) { // observe 1 s
        parked.step(idle);
        parked.state(s);
        touchedGround = touchedGround || s.onGround;
    }
    CAPTURE(s.altitudeMslM, s.altitudeAglM, s.airspeedTrueMs, s.velocityNedMs[2]);
    CHECK(touchedGround);
    CHECK(std::abs(s.altitudeMslM - 500.0) < 3.0); // CG sits ~1 m above the terrain on its gear
    CHECK(s.altitudeAglM < 3.0);
    CHECK(std::abs(s.velocityNedMs[2]) < 0.5);
}

TEST_CASE("trajectories are bit-identical across worker counts", "[sim][jsbsim][determinism]") {
    log::setLevel(log::Level::Warn);
    constexpr unsigned vehicles = 6, agentSteps = 60;
    constexpr int frameSkip = 4;

    const auto serial = fly(vehicles, 1, agentSteps, frameSkip);
    const auto two = fly(vehicles, 2, agentSteps, frameSkip);
    const auto four = fly(vehicles, 4, agentSteps, frameSkip);

    REQUIRE(serial.size() == vehicles);
    for (unsigned i = 0; i < vehicles; ++i) {
        CHECK_FALSE(serial[i].diverged);
        CHECK(serial[i].stepCount == agentSteps * frameSkip);
        CHECK(bitIdentical(serial[i], two[i]));
        CHECK(bitIdentical(serial[i], four[i]));
    }
    // Vehicles really are different from each other (different ICs / ailerons).
    CHECK_FALSE(bitIdentical(serial[0], serial[1]));
}

TEST_CASE("quaternion helper reproduces JSBSim's body-to-ECEF matrix", "[sim][jsbsim][attitude]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.headingDeg = 123.0;
    ic.pitchDeg = 4.0;
    ic.rollDeg = -17.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    ControlInputs in;
    in.setThrottleAll(0.7);
    in.aileron = 0.2;
    for (int i = 0; i < 90; ++i) model.step(in); // a rolling, turning state
    VehicleState s;
    model.state(s);

    double R[9];
    attitude::bodyToEcefFromQuaternion(s.attitudeEcefToBody, R);
    for (int i = 0; i < 9; ++i) CHECK(std::abs(R[i] - s.rotationBodyToEcef[i]) < 1e-9);

    // slerp end points are the inputs; the midpoint is a unit quaternion.
    double q[4];
    attitude::slerp(s.attitudeEcefToBody, s.attitudeEcefToBody, 0.5, q);
    for (int i = 0; i < 4; ++i) CHECK(std::abs(q[i] - s.attitudeEcefToBody[i]) < 1e-12);
}
