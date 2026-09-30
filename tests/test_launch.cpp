// A-GRA's LAUNCH from an airfield's runway (docs/flight-autonomy.md, 4.49; ADR-29 FA-9a: LCH-01, LCH-04): a wing lines up,
// rolls on the centre line, rotates at its rotation speed short of its tail's touching, climbs out and cleans up; a
// rotorcraft lifts to its hover over where it stands. The NEW's refusals name their fields.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kHalfWidthM = 22.5; // half a 45 m runway: FA-9's acceptance
constexpr double kKnotMs = 0.514444;

/// An aircraft parked on the ground at the default place, facing north.
std::uint32_t parked(session::World& w, const std::string& type) {
    session::VehicleSpec s;
    s.name = type;
    s.type = "jsbsim:" + type;
    s.initial.onGround = true;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    w.step(stepsFor(w, 1.0)); // (settled on its wheels)
    return id;
}

/// Airfield 7 with runway 3: 3,500 m north from where the vehicle stands.
void runwayNorth(session::World& w, std::uint32_t id, double lengthM = 3500.0) {
    const sim::VehicleState& s = *w.vehicleState(id);
    Runway r;
    r.id = 3;
    r.takeoff.start = RunwayPoint{s.latitudeRad, s.longitudeRad, s.altitudeMslM - s.altitudeAglM};
    r.takeoff.threshold = r.takeoff.start;
    r.takeoff.limit = RunwayPoint{s.latitudeRad + lengthM / kEarthM, s.longitudeRad, s.altitudeMslM - s.altitudeAglM};
    Airfield a;
    a.id = 7;
    a.runways = {r};
    REQUIRE(w.loadAirfield(id, a) == Reason::None);
}

BehaviorCommand launch(double airfield = 7.0, double runway = 3.0) {
    BehaviorCommand b;
    b.id = "launch";
    b.params["airfield"] = airfield;
    b.params["runway"] = runway;
    return b;
}

/// The speed the host rotates a wing at: 1.1 times its stall with flaps (else clean; else its least).
double rotationSpeed(session::World& w, std::uint32_t id) {
    const VehicleProfile* p = w.profile(id);
    double stall = std::isfinite(p->performance.stallFlapsCasMs) ? p->performance.stallFlapsCasMs : p->performance.stallCasMs;
    if (!std::isfinite(stall)) stall = w.performance(id)->minCasMs;
    return 1.1 * stall;
}

struct Takeoff {
    ActivityState state = ActivityState::Pending;
    double worstCrossM = 0.0;     ///< on the wheels, off the centre line
    double rotateCasMs = kHold;   ///< as the rotation began
    double struckFt = 0.0;        ///< the most any structure contact was pressed into the ground
};

/// A wing's launch flown to its end (240 s at most) in a crosswind of `windMs` from the west.
Takeoff takeoff(const std::string& type, double windMs) {
    session::World w(options(("launch-" + type).c_str()));
    if (windMs > 0.0) setWind(w, 270.0, windMs);
    const auto id = parked(w, type);
    runwayNorth(w, id);
    const sim::VehicleState s0 = *w.vehicleState(id);
    std::vector<sim::PropertyHandle> structure;
    for (int i = 0; i < 64; ++i)
        if (auto h = w.model(id)->property("contact/unit[" + std::to_string(i) + "]/compression-ft"); h.valid()) structure.push_back(h);
    const CommandResult r = w.submit(id, launch());
    REQUIRE(r.accepted());
    Takeoff t;
    for (double time = 0.0; time < 240.0; time += 0.1) {
        w.step(stepsFor(w, 0.1));
        const sim::VehicleState& s = *w.vehicleState(id);
        const ActivityRecord* a = w.activity(r.activity);
        if (s.onGround) {
            double north, east;
            offset(s, s0.latitudeRad, s0.longitudeRad, north, east);
            t.worstCrossM = std::max(t.worstCrossM, std::abs(east));
            for (auto& h : structure) t.struckFt = std::max(t.struckFt, h.get());
        }
        if (isHold(t.rotateCasMs) && a->progress.percent >= 25.0) t.rotateCasMs = s.airspeedCalibratedMs;
        if (!a->live()) break;
    }
    t.state = w.activity(r.activity)->state;
    return t;
}

} // namespace

TEST_CASE("launch: wings take off along their runway, calm and in a 10 m/s crosswind - rotated at Vr, the tail clear (LCH-01)",
          "[modes][launch]") {
    // a light single, a fly-by-wire fighter, an airliner with little tail clearance and a pitch-up of its own, an attack
    // aircraft parked nose-high, a tail-wheel glider-winged one on its wingtip skid, a long-winged one with adverse yaw
    for (const char* type : {"c172", "f35a", "e7a", "su25", "u2s", "rq4b"}) {
        for (double wind : {0.0, 10.0}) {
            INFO(type << " in a " << wind << " m/s crosswind");
            const Takeoff t = takeoff(type, wind);
            CHECK(t.state == ActivityState::Completed);
            CHECK(t.worstCrossM < kHalfWidthM);
            REQUIRE(!isHold(t.rotateCasMs));
            session::World w(options("vr"));
            const double vr = rotationSpeed(w, parked(w, type));
            CHECK(std::abs(t.rotateCasMs - vr) < 5.0 * kKnotMs);
            // its tail clear of the runway (the U-2S rests on its wingtip skid: that, and no more)
            CHECK(t.struckFt < (std::string(type) == "u2s" ? 0.05 : 1e-6));
        }
    }
}

TEST_CASE("launch: a rotorcraft lifts to its hover over where it stands, within a metre (LCH-04)", "[modes][launch]") {
    for (const char* type : {"cf2", "uh1h", "uh60"}) {
        for (double wind : {0.0, 10.0}) {
            INFO(type << " in a " << wind << " m/s wind");
            session::World w(options(("lift-" + std::string(type)).c_str()));
            if (wind > 0.0) setWind(w, 270.0, wind);
            const auto id = parked(w, type);
            runwayNorth(w, id);
            const sim::VehicleState s0 = *w.vehicleState(id);
            const CommandResult r = w.submit(id, launch());
            REQUIRE(r.accepted());
            for (double time = 0.0; time < 120.0 && w.activity(r.activity)->live(); time += 0.5) w.step(stepsFor(w, 0.5));
            CHECK(w.activity(r.activity)->state == ActivityState::Completed);
            const sim::VehicleState& s = *w.vehicleState(id);
            double north, east;
            offset(s, s0.latitudeRad, s0.longitudeRad, north, east);
            const double up = (s.altitudeMslM - s0.altitudeMslM) - (10.0 - s0.altitudeAglM); // (its c.g. 10 m over the ground)
            CHECK(std::hypot(std::hypot(north, east), up) < 1.0);
        }
    }
}

TEST_CASE("launch: refusals naming their fields; only on the ground; a wing's runway takeoff not built for a rotorcraft", "[modes][launch]") {
    session::World w(options("launch-refusals"));
    const auto id = parked(w, "c172");
    runwayNorth(w, id);
    const auto refused = [&](const BehaviorCommand& b, Reason why, std::int16_t field) {
        const CommandResult r = w.submit(id, b);
        CHECK(!r.accepted());
        CHECK(r.reason == why);
        CHECK(r.index == field);
    };
    refused(launch(8.0, 3.0), Reason::UnknownAirfield, 0); // not its airfield
    refused(launch(0.0, 3.0), Reason::UnknownAirfield, 0);
    refused(launch(7.0, 4.0), Reason::UnknownAirfield, 1); // not that airfield's runway
    // a runway for landing only; one it is not on (1 km east of it)
    {
        const sim::VehicleState& s = *w.vehicleState(id);
        Airfield a = *w.airfield(id, 7);
        Runway landing;
        landing.id = 5;
        landing.landing.start = RunwayPoint{s.latitudeRad, s.longitudeRad, 0.0};
        Runway away = a.runways[0];
        away.id = 6;
        const double east = 1000.0 / (kEarthM * std::cos(s.latitudeRad));
        away.takeoff.start.longitudeRad += east, away.takeoff.threshold.longitudeRad += east, away.takeoff.limit.longitudeRad += east;
        a.runways.push_back(landing);
        a.runways.push_back(away);
        REQUIRE(w.loadAirfield(id, a) == Reason::None);
    }
    refused(launch(7.0, 5.0), Reason::InvalidParameter, 1);
    refused(launch(7.0, 6.0), Reason::InvalidParameter, 1);
    // its lifecycle on the ground: NEW pending on its axes - the gear, flaps and brakes with the primary ones - a step makes it
    // active, CANCEL ends it
    {
        const CommandResult r = w.submit(id, launch());
        REQUIRE(r.accepted());
        const ActivityRecord* a = w.activity(r.activity);
        CHECK(a->state == ActivityState::Pending);
        CHECK((a->axes & (axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes) | kPrimaryAxes)) ==
              (axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes) | kPrimaryAxes));
        w.step();
        CHECK(w.activity(r.activity)->state == ActivityState::Active);
        CHECK(w.cancel(r.activity).status == CommandStatus::Canceled);
        CHECK(w.activity(r.activity)->state == ActivityState::Canceled);
        CHECK(w.activity(r.activity)->reason == Reason::Requested);
    }
    CHECK(w.support(id, "fsim.guidance.launch")->support == Support::Supported);
    CHECK(w.support(id, "fsim.guidance.launch/runway")->support == Support::Supported);
    CHECK(w.support(id, "fsim.guidance.launch/vertical")->support == Support::NotSupported);

    // airborne: unavailable
    const auto flying = wing(w, "c172", 1000.0, 50.0, 1);
    Airfield a = *w.airfield(id, 7);
    REQUIRE(w.loadAirfield(flying, a) == Reason::None);
    const CommandResult air = w.submit(flying, launch());
    CHECK(!air.accepted());
    CHECK(air.reason == Reason::Airborne);

    // a rotorcraft: its lift, not a wing's run along the runway
    session::World r(options("launch-rotor"));
    const auto heli = parked(r, "uh60");
    CHECK(r.support(heli, "fsim.guidance.launch")->support == Support::Supported);
    CHECK(r.support(heli, "fsim.guidance.launch/vertical")->support == Support::Supported);
    CHECK(r.support(heli, "fsim.guidance.launch/runway")->support == Support::NotImplemented);
}
