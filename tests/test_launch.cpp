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

/// The speed the host rotates a wing at: 1.1 times its stall clean (its landing configuration's is its approach's:
/// docs/flight-autonomy.md, 4.63), else its least.
double rotationSpeed(session::World& w, std::uint32_t id) {
    const VehicleProfile* p = w.profile(id);
    double stall = p->performance.stallCasMs;
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

TEST_CASE("launch: wings take off along their runway, calm and in a 10 m/s crosswind (7 m/s for a type whose limit is 15 kt) - "
          "rotated at Vr, the tail clear (LCH-01)",
          "[modes][launch]") {
    // a light single, a fly-by-wire fighter, an airliner with little tail clearance and a pitch-up of its own, an attack
    // aircraft parked nose-high, a tail-wheel glider-winged one on its wingtip skid, a long-winged one with adverse yaw
    for (const char* type : {"c172", "f35a", "e7a", "su25", "u2s", "rq4b"}) {
        const bool light = std::string(type) == "c172" || std::string(type) == "u2s"; // (their limit 15 kt: 4.54)
        for (double wind : {0.0, light ? 7.0 : 10.0}) {
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
    CHECK(w.support(id, "fsim.guidance.launch/rejected_takeoff")->support == Support::Supported);

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
    CHECK(r.support(heli, "fsim.guidance.launch/rejected_takeoff")->support == Support::NotImplemented); // (a wing's)
}

namespace {

/// Where a rejected takeoff ends: its activity's end, how far along the runway and across it the aircraft stopped (the worst
/// across on the way), and FA's own activity where a CANCEL handed it on.
struct Rejection {
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;
    ActivityId own = 0;
    ActivityState ownState = ActivityState::Pending;
    Source ownSource = Source::Policy;
    double alongM = 0.0, worstCrossM = 0.0, groundSpeedMs = 0.0, aglM = 0.0;
};

/// A wing's launch on a runway of `lengthM`, disturbed at `share` of its rotation speed: every fuel tank emptied (a flame-out,
/// its engines starved: `cancel` false) or its launch canceled by its policy; flown until nothing it started is live. A
/// crosswind of `windMs` from the west.
Rejection rejected(const std::string& type, double share, bool cancel, double lengthM = 3500.0, double windMs = 0.0) {
    session::World w(options(("rto-" + type).c_str()));
    if (windMs > 0.0) setWind(w, 270.0, windMs);
    const auto id = parked(w, type);
    runwayNorth(w, id, lengthM);
    const sim::VehicleState s0 = *w.vehicleState(id);
    const double vr = rotationSpeed(w, id);
    std::vector<sim::PropertyHandle> tanks;
    for (int i = 0; i < 16; ++i)
        if (auto h = w.model(id)->property("propulsion/tank[" + std::to_string(i) + "]/contents-lbs"); h.valid()) tanks.push_back(h);
    const CommandResult r = w.submit(id, launch());
    REQUIRE(r.accepted());
    Rejection out;
    bool struck = false;
    for (double time = 0.0; time < 240.0; time += 0.1) {
        w.step(stepsFor(w, 0.1));
        const sim::VehicleState& s = *w.vehicleState(id);
        double north, east;
        offset(s, s0.latitudeRad, s0.longitudeRad, north, east);
        if (s.onGround) out.worstCrossM = std::max(out.worstCrossM, std::abs(east));
        if (!struck && s.airspeedCalibratedMs >= share * vr) {
            struck = true;
            if (cancel) {
                const CommandResult c = w.cancel(r.activity);
                REQUIRE(c.status == CommandStatus::Canceled);
                out.own = c.other;
            }
        }
        if (struck && !cancel)
            for (auto& h : tanks) h.set(0.0);
        const bool live = w.activity(r.activity)->live() || (out.own && w.activity(out.own)->live());
        if (!live) break;
    }
    const sim::VehicleState& s = *w.vehicleState(id);
    double north, east;
    offset(s, s0.latitudeRad, s0.longitudeRad, north, east);
    out.alongM = north, out.groundSpeedMs = groundSpeed(s), out.aglM = s.altitudeAglM - s0.altitudeAglM;
    out.state = w.activity(r.activity)->state, out.reason = w.activity(r.activity)->reason;
    if (out.own) out.ownState = w.activity(out.own)->state, out.ownSource = w.activity(out.own)->source;
    return out;
}

} // namespace

TEST_CASE("launch: a wind across the runway beyond the type's limit refused crosswind_limit, and a taxi in a wind beyond it (4.54)",
          "[modes][launch]") {
    // the C172's limit is 15 kt (7.72 m/s)
    for (const double wind : {5.0, 10.0}) {
        INFO(wind << " m/s across");
        session::World w(options("launch-crosswind"));
        setWind(w, 270.0, wind);
        const auto id = parked(w, "c172");
        runwayNorth(w, id);
        w.step(stepsFor(w, 2.0));
        const CommandResult r = w.submit(id, launch());
        BehaviorCommand taxi;
        taxi.id = "taxi";
        const sim::VehicleState& s = *w.vehicleState(id);
        PositionCommand ahead;
        ahead.latitudeRad = s.latitudeRad + 200.0 / kEarthM, ahead.longitudeRad = s.longitudeRad;
        taxi.points = {ahead};
        if (wind > 7.72) {
            CHECK(r.reason == Reason::CrosswindLimit);
            CHECK(r.index == -1);
            CHECK(w.submit(id, taxi).reason == Reason::CrosswindLimit);
        } else {
            CHECK(r.accepted());
        }
    }
    // a rotorcraft's from any side (4.66): the UH-1H's 30 kt (15.4 m/s)
    for (const double wind : {10.0, 20.0}) {
        INFO("the UH-1H in " << wind << " m/s");
        session::World w(options("launch-wind-rotor"));
        setWind(w, 270.0, wind);
        const auto id = parked(w, "uh1h");
        runwayNorth(w, id);
        w.step(stepsFor(w, 2.0));
        const CommandResult r = w.submit(id, launch());
        if (wind > 15.4) CHECK(r.reason == Reason::CrosswindLimit);
        else CHECK(r.accepted());
    }
}

TEST_CASE("launch: an engine lost below its decision speed - the takeoff rejected, stopped on the runway (LCH-02)", "[modes][launch]") {
    for (const char* type : {"c172", "f16c", "b52h", "e7a", "su25"}) {
        INFO(type);
        const Rejection r = rejected(type, 0.6, false);
        CHECK(r.state == ActivityState::Failed);
        CHECK(r.reason == Reason::TakeoffRejected);
        CHECK(r.groundSpeedMs < 0.5);                    // stopped...
        CHECK((r.alongM > 0.0 && r.alongM < 3500.0));    // ...on the runway (the worst: the B-52H 462 m along)
        CHECK(r.worstCrossM < kHalfWidthM);
    }
    // a runway too short to take off from: rejected as soon as it is judged, at 30 % of its rotation speed, and stopped on it
    // (the B-52H on 700 m: 142 m along)
    const Rejection shortRunway = rejected("b52h", 2.0, false, 700.0);
    CHECK(shortRunway.reason == Reason::TakeoffRejected);
    CHECK(shortRunway.groundSpeedMs < 0.5);
    CHECK(shortRunway.alongM < 700.0);
}

TEST_CASE("launch: a tail-wheel aircraft's tail held down on its wheel, which steers it: on its line across its limit (4.65)",
          "[modes][launch]") {
    // the U-2S across 7 m/s (its limit 15 kt): before 4.65 its tail rose at 27 m/s, it ran on its single main wheel nose down,
    // steered by its rudder alone - its takeoff 12.7 m off its line, its rejection 4.7
    const Takeoff t = takeoff("u2s", 7.0);
    CHECK(t.state == ActivityState::Completed);
    CHECK(t.worstCrossM < 3.0); // (1.1 m)
    const Rejection r = rejected("u2s", 0.6, true, 3500.0, 7.0);
    REQUIRE(r.own != 0);
    CHECK(r.ownState == ActivityState::Completed);
    CHECK(r.groundSpeedMs < 0.5);
    CHECK(r.worstCrossM < 3.0); // (1.7 m)
    // an engine lost: stopped on its runway, its line held calm (1.7 m); across 7 m/s its failure judged 3.5 s after it, the
    // tail light, it runs 15 m off (a finding: docs/flight-autonomy.md, B-8)
    const Rejection lost = rejected("u2s", 0.6, false);
    CHECK(lost.reason == Reason::TakeoffRejected);
    CHECK(lost.groundSpeedMs < 0.5);
    CHECK(lost.worstCrossM < 3.0);
}

TEST_CASE("launch: a policy's CANCEL on the runway - FA stops it below its decision speed, flies it off above (LCH-02)", "[modes][launch]") {
    // below: FA's own rejection, completed stopped on the runway; the policy's launch canceled
    for (const char* type : {"c172", "f16c", "kc46a"}) {
        INFO(type << " canceled at half its rotation speed");
        const Rejection r = rejected(type, 0.5, true);
        CHECK(r.state == ActivityState::Canceled);
        REQUIRE(r.own != 0);
        CHECK(r.ownSource == Source::Autopilot);
        CHECK(r.ownState == ActivityState::Completed);
        CHECK(r.groundSpeedMs < 0.5);
        CHECK((r.alongM > 0.0 && r.alongM < 3500.0));
        CHECK(r.worstCrossM < kHalfWidthM);
    }
    // above: the E-3G reaches 97 % of its rotation speed 2 km along, where it could no longer stop - FA flies it off
    const Rejection go = rejected("e3g", 0.97, true);
    REQUIRE(go.own != 0);
    CHECK(go.ownState == ActivityState::Completed);
    CHECK(go.aglM > 400.0);

    // once it flies, a CANCEL hands nothing on; a policy's own "_" parameters are not its to give
    session::World w(options("launch-flown"));
    const auto id = parked(w, "f16c");
    runwayNorth(w, id);
    const CommandResult r = w.submit(id, launch());
    REQUIRE(r.accepted());
    for (int i = 0; i < 600 && !(w.activity(r.activity)->progress.percent >= 50.0); ++i) w.step(stepsFor(w, 0.1)); // (climbing out)
    REQUIRE(!w.vehicleState(id)->onGround);
    const CommandResult c = w.cancel(r.activity);
    CHECK(c.status == CommandStatus::Canceled);
    CHECK(c.other == 0);
    session::World g(options("launch-forged"));
    const auto parkedId = parked(g, "f16c");
    runwayNorth(g, parkedId);
    BehaviorCommand forged = launch();
    forged.params["_mode"] = 1.0; // (FA's rejection: stripped, it lines up and rolls)
    const CommandResult f = g.submit(parkedId, forged);
    REQUIRE(f.accepted());
    g.step(stepsFor(g, 8.0));
    CHECK(g.activity(f.activity)->state == ActivityState::Active);
    CHECK(groundSpeed(*g.vehicleState(parkedId)) > 10.0);
}
