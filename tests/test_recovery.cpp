// A-GRA's RECOVERY to an airfield's runway (docs/flight-autonomy.md, 4.53; ADR-29 FA-10a: RCV-01, RCV-04): a wing flies the
// approach the host laid out - a base leg, the extended centre line, the glide slope - flares, touches down in the touchdown zone
// below its sink-rate limit and stops on the runway; a rotorcraft lands on its spot. The NEW's refusals name their fields. Its
// go-around and missed approach (4.54; FA-10b: RCV-02, RCV-03): a crosswind beyond the type's limit waves it off, and it flies
// its circuit, or FA's own chained missed approach, to a second approach.
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

constexpr double kLengthM = 3000.0;
constexpr double kSinkLimitMs = 3.2; // under 10.5 ft/s: about FAR 25.473's 10 ft/s at the design landing weight

/// Airfield 7's runway 3: its threshold `aheadM` north of where `id` is, 3,000 m on north, at the ground's elevation there.
void runwayAhead(session::World& w, std::uint32_t id, double aheadM, double& lat0, double& lon0, double& elevation) {
    const sim::VehicleState& s = *w.vehicleState(id);
    lat0 = s.latitudeRad + aheadM / kEarthM, lon0 = s.longitudeRad, elevation = s.altitudeMslM - s.altitudeAglM;
    Runway r;
    r.id = 3;
    r.landing.start = r.landing.threshold = RunwayPoint{lat0, lon0, elevation};
    r.landing.limit = RunwayPoint{lat0 + kLengthM / kEarthM, lon0, elevation};
    Airfield a;
    a.id = 7;
    a.runways = {r};
    REQUIRE(w.loadAirfield(id, a) == Reason::None);
}

BehaviorCommand recovery(double airfield = 7.0, double runway = 3.0) {
    BehaviorCommand b;
    b.id = "recovery";
    b.params["airfield"] = airfield;
    b.params["runway"] = runway;
    return b;
}

struct Landed {
    ActivityState state = ActivityState::Pending;
    bool touched = false;
    double along = 0.0, across = 0.0, sink = 0.0, worstAcross = 0.0, endSpeed = 0.0;
};

/// `id`'s recovery flown to its end (1,500 s at most), measured from the threshold at (lat0, lon0).
Landed fly(session::World& w, std::uint32_t id, ActivityId activity, double lat0, double lon0) {
    Landed out;
    double sink = 0.0;
    for (double t = 0.0; t < 1500.0 && w.activity(activity)->live(); t += 0.1) {
        w.step(stepsFor(w, 0.1));
        const sim::VehicleState& s = *w.vehicleState(id);
        double north, east;
        offset(s, lat0, lon0, north, east);
        if (!out.touched && !s.onGround) sink = s.velocityNedMs[2];
        if (!out.touched && s.onGround) out.touched = true, out.along = north, out.across = east, out.sink = sink;
        if (out.touched) out.worstAcross = std::max(out.worstAcross, std::abs(east));
    }
    out.state = w.activity(activity)->state, out.endSpeed = groundSpeed(*w.vehicleState(id));
    return out;
}

} // namespace

TEST_CASE("recovery: a wing touches down in the touchdown zone below its sink-rate limit and stops on the runway, calm and in a 10 m/s "
          "crosswind (RCV-01)",
          "[modes][recovery]") {
    // a light single, a fighter that approaches nose high (its speed raised for room to flare), a delta, two heavies (the
    // E-7A, stopped across the wind, was thrown off the runway by its angle of attack's rate: 4.54), a glider
    for (const char* type : {"c172", "f16c", "mirage2000", "kc135r", "e7a", "rq4b"}) {
        for (double wind : {0.0, 10.0}) {
            if (wind > 0.0 && (std::string(type) == "rq4b" || std::string(type) == "c172")) continue; // (the C172's limit is 15 kt)
            INFO(type << " in a " << wind << " m/s crosswind");
            session::World w(options(("recovery-" + std::string(type)).c_str()));
            if (wind > 0.0) setWind(w, 270.0, wind);
            const auto probe = wing(w, type, 300.0, 100.0, 0); // (its speeds: then flown from a speed it holds)
            const double tas = std::max(1.4 * w.performance(probe)->minCasMs, std::min(60.0, w.performance(probe)->cruiseTasMs));
            REQUIRE(w.removeVehicle(probe));
            const auto id = wing(w, type, 300.0, tas, 1);
            double lat0, lon0, elevation;
            runwayAhead(w, id, 20000.0, lat0, lon0, elevation);
            const CommandResult r = w.submit(id, recovery());
            INFO(reasonName(r.reason) << " at " << r.index);
            REQUIRE(r.accepted());
            const Landed l = fly(w, id, r.activity, lat0, lon0);
            INFO("touched down " << l.along << " m along, " << l.across << " m across, sinking " << l.sink << " m/s");
            CHECK(l.state == ActivityState::Completed);
            CHECK(l.touched);
            CHECK((l.along > 0.0 && l.along < 900.0));
            CHECK(l.sink < kSinkLimitMs);
            CHECK(l.worstAcross < 22.5);
            CHECK(l.endSpeed < 0.5);
        }
    }
}

namespace {

struct Approached {
    ActivityState state = ActivityState::Pending;
    Reason reason = Reason::None;
    bool touched = false;
    int climbs = 0;             ///< times it passed over the threshold more than 100 m up: its go-arounds
    double nearestM = 1e9;      ///< to `watch`, horizontally
};

/// `id`'s recovery to the runway north from (lat0, lon0) flown to its end (3,000 s at most); `calmAfter` its go-arounds, the
/// wind dropped; `watch` a point it may pass.
Approached approach(session::World& w, std::uint32_t id, ActivityId activity, double lat0, double lon0, double elevation, int calmAfter = 99,
                    double watchLat = 0.0, double watchLon = 0.0) {
    Approached out;
    double lastNorth = -1e9;
    for (double t = 0.0; t < 3000.0 && w.activity(activity)->live(); t += 0.5) {
        w.step(stepsFor(w, 0.5));
        const sim::VehicleState& s = *w.vehicleState(id);
        out.touched = out.touched || s.onGround;
        double n, e;
        offset(s, lat0, lon0, n, e);
        if (lastNorth < 0.0 && n >= 0.0 && s.altitudeMslM - elevation > 100.0 && ++out.climbs == calmAfter) setWind(w, 270.0, 0.0);
        lastNorth = n;
        if (watchLat != 0.0) {
            double north, east;
            offset(s, watchLat, watchLon, north, east);
            out.nearestM = std::min(out.nearestM, std::hypot(north, east));
        }
    }
    out.state = w.activity(activity)->state, out.reason = w.activity(activity)->reason;
    return out;
}

} // namespace

TEST_CASE("recovery: a crosswind beyond the type's limit waves it off - it goes around and approaches again; waved off again it "
          "fails crosswind_limit, never down; the wind dropped, it lands (RCV-02, 4.54)",
          "[modes][recovery]") {
    // the F-16C's limit is 25 kt (12.9 m/s): 15 m/s across
    for (const int calmAfter : {99, 1}) {
        INFO((calmAfter == 1 ? "the wind dropped after its first go-around" : "the wind held"));
        session::World w(options("recovery-waveoff"));
        setWind(w, 270.0, 15.0);
        const auto id = wing(w, "f16c", 300.0, 100.0, 1);
        double lat0, lon0, elevation;
        runwayAhead(w, id, 20000.0, lat0, lon0, elevation);
        const CommandResult r = w.submit(id, recovery());
        REQUIRE(r.accepted());
        const Approached a = approach(w, id, r.activity, lat0, lon0, elevation, calmAfter);
        INFO("climbed away " << a.climbs << " times");
        if (calmAfter == 1) {
            CHECK(a.state == ActivityState::Completed);
            CHECK(a.touched);
            CHECK(a.climbs == 1);
        } else {
            CHECK(a.state == ActivityState::Failed);
            CHECK(a.reason == Reason::CrosswindLimit);
            CHECK(!a.touched);
            CHECK(a.climbs == 2);
        }
    }
}

TEST_CASE("recovery: going around, FA's own landing path's chained missed approach is flown before the next approach (RCV-03, "
          "VI 1.2.6.3)",
          "[modes][recovery]") {
    session::World w(options("recovery-missed"));
    setWind(w, 270.0, 15.0);
    const auto id = wing(w, "f16c", 300.0, 100.0, 1);
    double lat0, lon0, elevation;
    runwayAhead(w, id, 20000.0, lat0, lon0, elevation);
    // FA's own plan: a landing path for runway 3 of airfield 7 - 4 km out, then over its threshold - and a branch from its last
    // point to a missed approach path: 6 km on past the threshold climbing to 600 m, then 4 km east
    const auto point = [&](double north, double east, double up) {
        Waypoint p;
        p.latitudeRad = lat0 + north / kEarthM, p.longitudeRad = lon0 + east / (kEarthM * std::cos(lat0));
        p.altitudeM = elevation + up;
        return p;
    };
    RoutePlan plan;
    plan.id = 81, plan.version = 1;
    plan.waypoints = {point(-4000.0, 0.0, 210.0), point(0.0, 0.0, 15.0), point(6000.0, 0.0, 600.0), point(6000.0, 4000.0, 600.0)};
    plan.paths = {RoutePath{1, static_cast<double>(PathType::Landing), 0, 2}, RoutePath{2, static_cast<double>(PathType::Primary), 2, 2}};
    RouteBranch branch;
    branch.point = 1, branch.next = 2.0;
    plan.branches = {branch};
    PathMetadata meta;
    meta.path = 0, meta.airfield = 7, meta.runway = 3;
    plan.pathMetadata = {meta};
    REQUIRE(w.loadPlan(id, plan) == Reason::None);
    const CommandResult r = w.submit(id, recovery());
    REQUIRE(r.accepted());
    const Waypoint& last = plan.waypoints[3];
    const Approached a = approach(w, id, r.activity, lat0, lon0, elevation, 99, last.latitudeRad, last.longitudeRad);
    double north, east;
    offset(*w.vehicleState(id), lat0, lon0, north, east);
    INFO("climbed away " << a.climbs << " times; came within " << a.nearestM << " m of the missed approach's last point; now "
                         << north << " m north, " << east << " m east, " << w.vehicleState(id)->altitudeMslM - elevation << " m up, flying to point "
                         << w.activity(r.activity)->progress.segment);
    CHECK(a.state == ActivityState::Failed);
    CHECK(a.reason == Reason::CrosswindLimit);
    CHECK(a.climbs == 2);
    CHECK(a.nearestM < 500.0);
}

TEST_CASE("recovery: a rotorcraft flies to a hover over the runway and lands straight down on its spot (RCV-04)", "[modes][recovery]") {
    for (const char* type : {"uh1h", "uh60", "iris", "cf2"}) {
        INFO(type);
        session::World w(options(("recovery-" + std::string(type)).c_str()));
        const auto id = rotor(w, type);
        const double cruise = w.performance(id)->cruiseTasMs;
        double lat0, lon0, elevation;
        runwayAhead(w, id, std::clamp(60.0 * cruise, 30.0, 1000.0), lat0, lon0, elevation);
        const CommandResult r = w.submit(id, recovery());
        REQUIRE(r.accepted());
        const Landed l = fly(w, id, r.activity, lat0, lon0);
        INFO("touched down " << l.along << " m along, " << l.across << " m across, sinking " << l.sink << " m/s");
        CHECK(l.state == ActivityState::Completed);
        CHECK(std::hypot(l.along - 60.0, l.across) < 3.0); // (60 m beyond the threshold: the worst, the UH-1H's 1.4 m)
        CHECK(l.sink < 1.0);
    }
}

TEST_CASE("recovery: its NEW names the airfield or runway it cannot use; only in the air; its axes, its configuration, its rollout "
          "handed on (4.53)",
          "[modes][recovery]") {
    session::World w(options("recovery-refusals"));
    const auto id = wing(w, "c172", 300.0, 45.0);
    double lat0, lon0, elevation;
    runwayAhead(w, id, 6000.0, lat0, lon0, elevation);
    const auto refused = [&](const BehaviorCommand& b, Reason why, std::int16_t field) {
        const CommandResult r = w.submit(id, b);
        CHECK(!r.accepted());
        CHECK(r.reason == why);
        CHECK(r.index == field);
    };
    refused(recovery(8.0, 3.0), Reason::UnknownAirfield, 0); // not its airfield
    refused(recovery(0.0, 3.0), Reason::UnknownAirfield, 0);
    refused(recovery(7.0, 4.0), Reason::UnknownAirfield, 1); // not that airfield's runway
    {
        Airfield a = *w.airfield(id, 7);
        Runway takeoffs;
        takeoffs.id = 5;
        takeoffs.takeoff.start = RunwayPoint{lat0, lon0, elevation};
        a.runways.push_back(takeoffs);
        REQUIRE(w.loadAirfield(id, a) == Reason::None);
    }
    refused(recovery(7.0, 5.0), Reason::InvalidParameter, 1); // a runway for takeoffs only
    CHECK(w.support(id, "fsim.guidance.recovery")->support == Support::Supported);
    CHECK(w.support(id, "fsim.guidance.recovery/runway")->support == Support::Supported);
    CHECK(w.support(id, "fsim.guidance.recovery/vertical")->support == Support::NotSupported); // (R1: it does not hover)
    CHECK(w.support(id, "fsim.guidance.recovery/go_around")->support == Support::Supported); // (4.54)
    CHECK(w.support(id, "fsim.guidance.recovery/missed_approach")->support == Support::Supported);

    // its lifecycle: NEW pending on the primary axes with the gear, flaps and brakes; active after a step; its gear down and flaps
    // out from the intermediate fix on (the configuration a behaviour above the actuators sets: Behavior::configure)
    const CommandResult r = w.submit(id, recovery());
    REQUIRE(r.accepted());
    const AxisMask config = axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes);
    CHECK((w.activity(r.activity)->axes & (kPrimaryAxes | config)) == (kPrimaryAxes | config));
    w.step();
    CHECK(w.activity(r.activity)->state == ActivityState::Active);
    bool configured = false, rolling = false;
    for (double t = 0.0; t < 900.0 && !rolling; t += 0.5) {
        w.step(stepsFor(w, 0.5));
        const sim::VehicleState& s = *w.vehicleState(id);
        configured = configured || (s.flapsRad > 0.0 && !s.onGround);
        rolling = s.onGround && groundSpeed(s) > 15.0;
    }
    CHECK(configured);
    REQUIRE(rolling);
    // a policy's CANCEL in the rollout: FA's own stop, as a rejected takeoff's (4.50)
    const CommandResult c = w.cancel(r.activity);
    CHECK(c.status == CommandStatus::Canceled);
    REQUIRE(c.other != 0);
    CHECK(w.activity(c.other)->source == Source::Autopilot);
    for (double t = 0.0; t < 120.0 && w.activity(c.other)->live(); t += 0.5) w.step(stepsFor(w, 0.5));
    CHECK(w.activity(c.other)->state == ActivityState::Completed);
    CHECK(groundSpeed(*w.vehicleState(id)) < 0.5);

    // on the ground: a policy's waits for it to fly
    session::World g(options("recovery-parked"));
    session::VehicleSpec s;
    s.name = "parked";
    s.type = "jsbsim:c172";
    s.initial.onGround = true;
    s.initial.airspeedTrueMs = 0.0;
    const auto parked = g.createVehicle(s);
    REQUIRE(parked != 0);
    g.step(stepsFor(g, 1.0));
    const CommandResult onGround = g.submit(parked, recovery());
    INFO(reasonName(onGround.reason));
    CHECK(onGround.reason == Reason::OnGround);
}
