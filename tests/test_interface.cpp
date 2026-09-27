// What a mission autonomy is told (docs/vehicle-interface.md, section 5): a
// rejection's detail, an activity's progress, the commanded state, and which
// of A-GRA's flight capability types a capability is (step VI-2).
#include "fsim/Control.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kEarthM = 6371000.0;

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 13;
    return o;
}

std::uint32_t aircraft(session::World& w, const char* type, double altitudeM, double tasMs, double northDeg = 0.0) {
    session::VehicleSpec s;
    s.name = std::string(type) + std::to_string(northDeg);
    s.type = std::string("jsbsim:") + type;
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

PositionCommand pointAt(const sim::VehicleState& s, double north, double east, double altitudeM, double captureM) {
    PositionCommand p;
    p.latitudeRad = s.latitudeRad + north / kEarthM;
    p.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    p.altitudeMslM = altitudeM;
    p.captureRadiusM = captureM;
    return p;
}

const CapabilityDescriptor* capabilityOf(session::World& w, std::uint32_t v, const char* id) {
    for (const auto& d : w.capabilities(v))
        if (d.id == id) return &d;
    return nullptr;
}

} // namespace

TEST_CASE("interface: a rejection says which field it was about and which limit its value broke", "[interface]") {
    session::World w(options("interface-detail"));
    const auto c172 = aircraft(w, "c172x", 1500.0, 55.0);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    AttitudeCommand steep{4.0, 0.0, kHold, 0.785, kHold, kHold}; // beyond +-pi
    CommandResult r = w.submit(c172, steep, reject);
    CHECK(r.reason == Reason::OutOfRange);
    CHECK(r.index == 0);
    CHECK(r.constraint == Constraint::MaxOrientation);
    // clamped instead: accepted, with the first value it clamped
    r = w.submit(c172, steep);
    REQUIRE(r.accepted());
    CHECK((r.flags & kClamped) != 0);
    CHECK(r.index == 0);
    CHECK(r.constraint == Constraint::MaxOrientation);
    // an airspeed below nothing
    r = w.submit(c172, VelocityCommand{-5.0, 0.0, kHold, kHold}, reject);
    CHECK(r.reason == Reason::OutOfRange);
    CHECK(r.index == 0);
    CHECK(r.constraint == Constraint::MinAirspeed);
    // a design's envelope narrows the load factor: 12 g breaks its maximum acceleration
    const auto f16 = aircraft(w, "f16c", 3000.0, 160.0, 0.05);
    r = w.submit(f16, AccelerationCommand{12.0, 0.0, kHold, kHold}, reject);
    CHECK(r.reason == Reason::OutOfRange);
    CHECK(r.index == 0);
    CHECK(r.constraint == Constraint::MaxAcceleration);
    // a route's bad point, by its place in the route
    BehaviorCommand route;
    route.id = "waypoints";
    const auto& s = *w.vehicleState(c172);
    route.points = {pointAt(s, 3000.0, 0.0, 1500.0, 200.0), pointAt(s, 6000.0, 0.0, 1500.0, 200.0), pointAt(s, 9000.0, 0.0, 1500.0, 0.0)};
    r = w.submit(c172, route);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 2);
    // an answer about nothing in particular says so
    CHECK(w.submit(c172, VelocityCommand{55.0, 0.0, kHold, kHold}).index == -1);
    CHECK(std::string(reasonName(Reason::InvalidWaypoint)) == "invalid_waypoint");
    CHECK(std::string(constraintName(Constraint::MaxClimbRate)) == "max_climb_rate");
}

TEST_CASE("interface: a route reports the point it flies to, the distance and time to go, and how much is done", "[interface]") {
    session::World w(options("interface-route"));
    const auto v = aircraft(w, "c172x", 1500.0, 55.0);
    const auto& s0 = *w.vehicleState(v);
    BehaviorCommand route;
    route.id = "waypoints";
    route.points = {pointAt(s0, 0.0, 3000.0, 1500.0, 300.0), pointAt(s0, 3000.0, 3000.0, 1600.0, 300.0)};
    const auto id = w.submit(v, route).activity;
    REQUIRE(id != 0);
    std::uint32_t segment = 0;
    double percent = 0.0;
    bool completed = false;
    for (int k = 0; k < 6000 && !completed; ++k) { // 200 s: 6 km at 55 m/s, and a turn
        w.step();
        const ActivityRecord* a = w.activity(id);
        REQUIRE(a != nullptr);
        const ActivityProgress& p = a->progress;
        CHECK(p.segments == 2);
        CHECK(p.segment >= segment); // it moves on, never back
        CHECK(p.percent >= percent - 0.5); // it only gets further (a turn onto the next leg may lose a little)
        CHECK((p.percent >= 0.0 && p.percent <= 100.0));
        segment = p.segment, percent = std::max(percent, p.percent);
        completed = a->state == ActivityState::Completed;
        if (!completed) {
            CHECK(p.distanceToGoM > 0.0);
            CHECK(p.timeToGoS > 0.0);
            CHECK(p.altitudeMslM == route.points[p.segment].altitudeMslM);
        } else {
            CHECK(p.percent == 100.0);
            CHECK(p.distanceToGoM == 0.0);
        }
    }
    CHECK(completed);
    CHECK(segment == 1);
}

TEST_CASE("interface: a loiter counts its laps, a hold and a hover report their targets", "[interface]") {
    session::World w(options("interface-behaviours"));
    const auto wing = aircraft(w, "c172x", 1500.0, 55.0);
    BehaviorCommand loiter;
    loiter.id = "loiter";
    loiter.params = {{"radius_m", 500.0}, {"altitude_m", 1600.0}};
    const auto circling = w.submit(wing, loiter).activity;
    const auto other = aircraft(w, "c172x", 1500.0, 55.0, 0.05);
    BehaviorCommand hold;
    hold.id = "hold";
    hold.params = {{"heading_deg", 180.0}, {"altitude_m", 1700.0}, {"airspeed_ms", 50.0}};
    const auto holding = w.submit(other, hold).activity;
    w.step(static_cast<unsigned>(std::lround(240.0 / (w.dt() * w.frameSkip()))));
    const ActivityProgress& l = w.activity(circling)->progress;
    CHECK(l.laps >= 2); // 500 m at 55 m/s: a lap a minute
    CHECK(std::abs(l.crossTrackM) < 100.0);
    CHECK(l.altitudeMslM == 1600.0);
    const ActivityProgress& h = w.activity(holding)->progress;
    CHECK(std::abs(h.headingRad - 3.14159265358979) < 1e-9);
    CHECK(h.altitudeMslM == 1700.0);
    CHECK(h.speedMs == 50.0);
    CHECK(h.speedReference == 0.0); // true airspeed
    CHECK(h.segments == 0);        // nothing segmented
    // a level's activity reports no progress
    const auto level = w.submit(other, VelocityCommand{55.0, 0.0, kHold, kHold}).activity;
    w.step();
    CHECK(std::isnan(w.activity(level)->progress.percent));
    CHECK(std::isnan(w.activity(level)->progress.altitudeMslM));
}

TEST_CASE("interface: the commanded state is what the cascade asked for, level by level", "[interface]") {
    session::World w(options("interface-commanded"));
    const auto v = aircraft(w, "c172x", 1500.0, 55.0);
    VelocityCommand climb{58.0, 2.0, 0.5, kHold};
    REQUIRE(w.submit(v, climb).accepted());
    w.step();
    const CommandedState c = w.controls(v)->commanded();
    CHECK(c.top == Level::Velocity);
    CHECK(c.airspeedMs == 58.0);
    CHECK(c.verticalSpeedMs == 2.0);
    CHECK(c.headingRad == 0.5);
    CHECK(std::isnan(c.altitudeMslM)); // no position level ran
    CHECK(!std::isnan(c.pitchRad));    // the attitude level did
    CHECK((c.throttle >= 0.0 && c.throttle <= 1.0));
    const auto& s = *w.vehicleState(v);
    PositionCommand there = pointAt(s, 5000.0, 0.0, 1800.0, 200.0);
    REQUIRE(w.submit(v, there).accepted());
    w.step();
    const CommandedState p = w.controls(v)->commanded();
    CHECK(p.top == Level::Position);
    CHECK(p.altitudeMslM == 1800.0);
    CHECK(p.latitudeRad == there.latitudeRad);
    CHECK(p.verticalSpeedMs > 0.0); // climbing to it
}

TEST_CASE("interface: capabilities say which of A-GRA's flight capability types they are", "[interface]") {
    session::World w(options("interface-modes"));
    const auto wing = aircraft(w, "c172x", 1500.0, 55.0);
    CHECK(capabilityOf(w, wing, "fsim.guidance.formation")->mode == FlightMode::Formation);
    CHECK(capabilityOf(w, wing, "fsim.flight.velocity")->mode == FlightMode::None);
    CHECK(capabilityOf(w, wing, "fsim.guidance.pursuit")->mode == FlightMode::None);
    const auto quad = aircraft(w, "iris", 100.0, 0.0, 0.05);
    CHECK(capabilityOf(w, quad, "fsim.guidance.hover")->mode == FlightMode::Loiter);
    CHECK(std::string(flightModeName(FlightMode::WaypointFollowing)) == "waypoint_following");
}
