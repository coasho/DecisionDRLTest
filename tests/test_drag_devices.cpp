// Drag devices (docs/flight-autonomy.md, 4.55; ADR-29 FA-10c: SUB-04, STS-16): hangar models each type's where rule R8
// applies - airbrake plates, spoilers, control surfaces deflected as one - and the speedbrake opens them: supported where they
// are, not supported where the type has none. Their drag slows the aircraft; spoilers dump lift.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

/// The sum of the type's drag devices' functions on `axis` ("CD", "CL"), as coefficients.
double deviceCoefficient(session::World& w, std::uint32_t id, const char* axis) {
    auto* m = w.model(id);
    const double qs = m->property("aero/qbar-psf").get() * m->property("metrics/Sw-sqft").get();
    double sum = 0.0;
    for (const char* name : {"speed_brake", "speed_brakes", "airbrake", "airbrakes", "decelerons", "spoilers", "surfaces"})
        for (const char* suffix : {"", "_rudder", "_flap", "_aileron", "_elevator"}) {
            auto h = m->property(std::string("aero/coefficient/") + axis + "_" + name + suffix);
            if (h.valid()) sum += h.get() / qs;
        }
    return sum;
}

} // namespace

TEST_CASE("drag devices: the speedbrake opens each type's - airbrakes, spoilers, surfaces - where rule R8 applies; none where it "
          "does not (SUB-04, STS-16)",
          "[modes][drag]") {
    struct Type {
        const char* type;
        bool lift; // spoilers: lift lost
    };
    for (const Type t : {Type{"f15c", false}, Type{"f16c", false}, Type{"a10c", false}, Type{"e7a", true}, Type{"b52h", true},
                         Type{"f22a", false}, Type{"rafale", false}}) {
        INFO(t.type);
        session::World w(options(("drag-" + std::string(t.type)).c_str()));
        const auto probe = wing(w, t.type, 2000.0, 100.0, 0);
        const double speed = 1.5 * w.performance(probe)->minCasMs;
        REQUIRE(w.removeVehicle(probe));
        const auto id = wing(w, t.type, 2000.0, speed, 1);
        CHECK(w.support(id, "fsim.support.speedbrake")->support == Support::Supported);
        CHECK(deviceCoefficient(w, id, "CD") == 0.0); // (closed)
        REQUIRE(w.submit(id, SpeedbrakeCommand{1.0}).accepted());
        w.step(stepsFor(w, 4.0));
        CHECK(std::abs(w.model(id)->property("fcs/speedbrake-pos-norm").get() - 1.0) < 1e-9);
        const double cd = deviceCoefficient(w, id, "CD"), cl = deviceCoefficient(w, id, "CL");
        INFO("dCD " << cd << ", dCL " << cl);
        CHECK(cd > 0.02); // (the least: the F-15C's 0.029, a plate of 2.93 m2 on its 56.5 m2 wing)
        CHECK(cd < 0.11);
        CHECK((t.lift ? cl < -0.05 : cl == 0.0));
    }
    for (const char* type : {"c172", "c130j"}) { // (no spoilers or airbrakes: rule R8)
        INFO(type);
        session::World w(options("drag-none"));
        const auto id = wing(w, type, 2000.0, 60.0);
        CHECK(w.support(id, "fsim.support.speedbrake")->support == Support::NotSupported);
        CHECK(w.submit(id, SpeedbrakeCommand{1.0}).reason == Reason::NotSupported);
    }
}

TEST_CASE("drag devices: the F-15C's speed brake slows it at idle by its drag (4.55)", "[modes][drag]") {
    double lost[2] = {0.0, 0.0};
    for (const int out : {0, 1}) {
        session::World w(options("drag-f15c"));
        const auto id = wing(w, "f15c", 3000.0, 200.0);
        const sim::VehicleState s = *w.vehicleState(id);
        AttitudeCommand idle;
        idle.rollRad = 0.0, idle.pitchRad = s.eulerRad[1], idle.throttle = 0.0;
        REQUIRE(w.command(id, idle));
        if (out) REQUIRE(w.submit(id, SpeedbrakeCommand{1.0}).accepted());
        w.step(stepsFor(w, 20.0));
        lost[out] = s.airspeedTrueMs - w.vehicleState(id)->airspeedTrueMs;
    }
    INFO("in: " << lost[0] << " m/s lost in 20 s; out: " << lost[1]);
    // (its drag, q S 0.031 at 200 m/s, about 32 kN on its 20 t, falling as it slows: 1.12 m/s2)
    CHECK(lost[1] - lost[0] > 15.0);
    CHECK(lost[1] - lost[0] < 30.0);
}
