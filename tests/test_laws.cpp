// The built-in loops designed from an aircraft's identified plant
// (docs/control-architecture.md, step 5): the poles where the plant's lags
// ask, the schedules and feedforwards each family needs, and which gains win.
#include "control/Laws.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

VehicleProfile plant(ControlFamily family, AircraftClass cls = AircraftClass::Fighter) {
    VehicleProfile p;
    p.identity.header = {1, Provenance::Hangar};
    p.identity.family = family;
    p.identity.aircraftClass = cls;
    PlantSection& pl = p.plant;
    pl.header = {1, Provenance::Hangar};
    pl.altitudeM = 3000.0, pl.tasMs = 150.0, pl.easMs = 130.0;
    pl.roll = {0.25, 3.0};
    pl.pitch = {0.7, 8.0};
    pl.yaw = {kUnknown, 0.4};
    pl.speed = {0.3, 6.0};
    pl.throttleTrim = 0.5;
    pl.elevatorTrim = family == ControlFamily::FlyByWire ? 0.0 : 0.3;
    pl.elevatorTrimLift = family == ControlFamily::FlyByWire ? 0.0 : 0.2;
    pl.alphaZeroLiftRad = -0.03;
    return p;
}

std::map<std::string, double> byName(const std::vector<ControllerSetting>& settings) {
    std::map<std::string, double> out;
    for (const auto& s : settings) out[s.controller + " " + s.parameter] = s.value;
    return out;
}

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    return o;
}

session::VehicleSpec spec(const char* name, const char* type) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.altitudeMslM = 3000.0;
    s.initial.airspeedTrueMs = 150.0;
    return s;
}

} // namespace

TEST_CASE("each loop's poles go where the plant's lag asks", "[laws]") {
    const auto fbw = byName(designLaws(plant(ControlFamily::FlyByWire)));
    // roll: the bank on gain / (s (tau s + 1)), poles at 0.7 / tau (at most 2.5 rad/s) with damping 0.8
    const double g = 3.0, tau = 0.25, w = std::min(2.5, 0.7 / tau);
    const double kp = fbw.at("pid_attitude roll.kp"), kd = fbw.at("pid_attitude roll.kd");
    // tau s^2 + (1 + g kd) s + g kp
    CHECK(std::abs(std::sqrt(g * kp / tau) - w) < 1e-9);
    CHECK(std::abs((1.0 + g * kd) / tau / (2.0 * w) - 0.8) < 1e-9);
    // pitch: on the pitch rate the load factor gives, g0 G_n / tas
    const double gq = 9.80665 * 8.0 / 150.0, tq = 0.7, wq = std::min(2.5, 0.7 / tq);
    CHECK(std::abs(std::sqrt(gq * fbw.at("pid_attitude pitch.kp") / tq) - wq) < 1e-9);
    // a plant damped enough on its own: no negative rate feedback, the dominant pole at omega
    VehicleProfile quick = plant(ControlFamily::FlyByWire);
    quick.plant.roll = {0.05, 4.0};
    const auto q = byName(designLaws(quick));
    CHECK(q.at("pid_attitude roll.kd") == 0.0);
    CHECK(designLaws(quick).size() == 51);
}

TEST_CASE("a law and surfaces are designed each as it answers", "[laws]") {
    const auto fbw = byName(designLaws(plant(ControlFamily::FlyByWire)));
    const auto direct = byName(designLaws(plant(ControlFamily::Direct, AircraftClass::Transport)));
    // a law that commands the load factor: its pitch rate per stick falls as 1 / tas
    CHECK(fbw.at("pid_attitude pitch.eas_exponent") == 0.0);
    CHECK(fbw.at("pid_attitude pitch.tas_exponent") == -1.0);
    CHECK(fbw.at("pid_attitude roll.tas_exponent") == 0.0);
    CHECK(fbw.at("pid_attitude pitch.trim") == 0.0);
    CHECK(fbw.at("pid_attitude rudder.beta_gain") == 0.0); // a law coordinates its turns
    CHECK(fbw.at("pid_acceleration load_factor.path_hold") == 1.0);
    CHECK(std::abs(fbw.at("pid_acceleration load_factor.feedforward") - 0.85 / 8.0) < 1e-12);
    CHECK(std::abs(fbw.at("pid_velocity max_bank") - 60.0 * kDeg) < 1e-12); // a fighter's bank
    CHECK(std::abs(fbw.at("pid_velocity vertical_speed.alpha_zero_lift") + 0.03) < 1e-12);
    // surfaces: the roll gains fall as (eas / tas)^2, the trim law fed forward,
    // the rudder takes half the sideslip out against its own step's sign
    CHECK(direct.at("pid_attitude roll.eas_exponent") == -2.0);
    CHECK(direct.at("pid_attitude roll.tas_exponent") == 2.0);
    CHECK(direct.at("pid_attitude pitch.trim") == 0.3);
    CHECK(direct.at("pid_attitude pitch.trim_lift") == 0.2);
    CHECK(std::abs(direct.at("pid_attitude rudder.beta_gain") + 0.5 / 0.4) < 1e-12);
    CHECK(direct.at("pid_attitude pitch.kd") >= 0.25 * direct.at("pid_attitude pitch.kp") - 1e-12);
    CHECK(direct.at("pid_acceleration load_factor.path_hold") == 0.0);
    CHECK(std::abs(direct.at("pid_velocity max_bank") - 30.0 * kDeg) < 1e-12);
    CHECK(direct.at("pid_attitude throttle.feedforward") == 0.5);
    // the outer loops slower than the inner ones
    CHECK(direct.at("pid_velocity vertical_speed.kp") * 150.0 < 0.36);
    // what the design cannot do without
    VehicleProfile blind = plant(ControlFamily::Direct);
    blind.plant.pitch = {};
    CHECK_FALSE(canDesignLaws(blind));
    CHECK(designLaws(blind).empty());
}

TEST_CASE("an aircraft flies the loops designed from its plant; gains it or a trainer gives win", "[laws]") {
    VehicleProfile own = plant(ControlFamily::Direct);
    own.control.header = {1, Provenance::User};
    own.control.settings = {{"pid_attitude", "roll.kp", 0.123}};
    CHECK_FALSE(completeControl(own));
    CHECK(own.control.settings.size() == 1);
    VehicleProfile derived = plant(ControlFamily::Direct);
    REQUIRE(completeControl(derived));
    CHECK(derived.control.header.provenance == Provenance::Derived);

    session::World w(options("laws-world"));
    const auto viper = w.createVehicle(spec("viper", "jsbsim:f16c"));
    REQUIRE(viper != 0);
    const VehicleProfile& p = *w.profile(viper);
    CHECK(p.control.header.provenance == Provenance::Derived); // hangar wrote its plant, not gains
    CHECK(p.control.settings.size() == 51);
    const Controller* attitude = w.controls(viper)->controller(Level::Attitude);
    REQUIRE(attitude != nullptr);
    CHECK(attitude->parameter("schedule.tas_ms").value_or(0.0) == p.plant.tasMs);
    CHECK(attitude->parameter("throttle.feedforward").value_or(0.0) == p.plant.throttleTrim);
    // a stock aircraft has neither: the loops' own defaults
    const auto stock = w.createVehicle(spec("stock", "jsbsim:c172x"));
    CHECK_FALSE(w.profile(stock)->control.header.present());

    // a trainer's plant of its own: the loops designed from it
    auto faster = std::make_shared<VehicleProfile>();
    faster->plant = p.plant;
    faster->plant.roll.gain = 2.0 * p.plant.roll.gain;
    auto s = spec("twitchy", "jsbsim:f16c");
    s.profile = faster;
    const auto twitchy = w.createVehicle(s);
    const double kp = attitude->parameter("roll.kp").value_or(0.0);
    const double kpFaster = w.controls(twitchy)->controller(Level::Attitude)->parameter("roll.kp").value_or(0.0);
    CHECK(std::abs(kpFaster - 0.5 * kp) < 1e-9 * kp); // twice the roll rate per aileron: half the gain
    // a trainer's gains of their own win
    auto gains = std::make_shared<VehicleProfile>();
    gains->control.header = {1, Provenance::User};
    gains->control.settings = {{"pid_attitude", "roll.kp", 0.123}};
    s.name = "tuned";
    s.profile = gains;
    const auto tuned = w.createVehicle(s);
    CHECK(w.controls(tuned)->controller(Level::Attitude)->parameter("roll.kp").value_or(0.0) == 0.123);
}
