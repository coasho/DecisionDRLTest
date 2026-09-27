// What an activity flies, and where to (docs/flight-autonomy.md, 4.12; ADR-29
// FA-2): an activity's setpoint read back - merged, completed, appended, or as
// given while it waits - its end points, and what the vehicle is commanded:
// the acceleration in north, east and down, the altitude in the reference it
// was commanded in.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

Waypoint at(const sim::VehicleState& s, double north, double east, double altitudeM, std::uint64_t id = 0) {
    Waypoint p;
    p.latitudeRad = s.latitudeRad + north / kEarthM;
    p.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    p.altitudeM = altitudeM;
    p.id = id;
    return p;
}

BezierSegment straight(double north0, double north1, double east0 = 0.0, double east1 = 0.0) {
    BezierSegment b;
    for (int i = 0; i < 6; ++i) b.north[i] = north0 + (north1 - north0) * i / 5.0, b.east[i] = east0 + (east1 - east0) * i / 5.0;
    return b;
}

CommandOptions startingAt(double t) {
    CommandOptions o;
    o.window.startNotBefore = t;
    return o;
}

template <typename T>
const T* commandOf(const Setpoint& s) {
    const auto* c = std::get_if<Command>(&s.command);
    return c ? std::get_if<T>(c) : nullptr;
}

} // namespace

TEST_CASE("reports: an activity's setpoint read back - completed, merged, appended - and as given while it waits", "[report]") {
    session::World w(options("reports-setpoint"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    fly(w, 1.0);
    const sim::VehicleState& s = *w.vehicleState(f16);
    Setpoint read;

    SECTION("an hsa: what it left out completed, an UPDATE merged") {
        HsaCommand h;
        h.headingRad = 1.2;
        const CommandResult a = w.submit(f16, Command(h));
        REQUIRE(a.accepted());
        REQUIRE(w.activitySetpoint(a.activity, read));
        const auto* flown = commandOf<HsaCommand>(read);
        REQUIRE(flown);
        CHECK(std::abs(flown->headingRad - 1.2) < 1e-9);
        CHECK(flown->speedReference == static_cast<double>(SpeedReference::TrueAirspeed)); // (what the aircraft flies now)
        CHECK(std::abs(flown->speed - 160.0) < 5.0);
        CHECK(flown->altitudeReference == static_cast<double>(AltitudeReference::Msl));
        CHECK(std::abs(flown->altitudeM - 3000.0) < 30.0);
        HsaCommand up;
        up.altitudeM = 3300.0;
        REQUIRE(w.update(a.activity, Command(up)).accepted());
        REQUIRE(w.activitySetpoint(a.activity, read));
        CHECK(commandOf<HsaCommand>(read)->altitudeM == 3300.0);
        CHECK(std::abs(commandOf<HsaCommand>(read)->headingRad - 1.2) < 1e-9); // (kept)
        REQUIRE(w.cancel(a.activity).status == CommandStatus::Canceled);
        CHECK_FALSE(w.activitySetpoint(a.activity, read));
    }
    SECTION("a route: its waypoints completed") {
        const std::vector<Waypoint> points = {at(s, 3000.0, 3000.0, 3100.0, 11), at(s, 6000.0, 0.0, kHold, 12)};
        const CommandResult r = w.submit(f16, RouteCommand{}, points);
        REQUIRE(r.accepted());
        REQUIRE(w.activitySetpoint(r.activity, read));
        REQUIRE(commandOf<RouteCommand>(read));
        REQUIRE(read.waypoints.size() == 2);
        CHECK(read.waypoints[1].altitudeM == 3100.0); // (the point before's)
        CHECK(read.waypoints[1].altitudeReference == static_cast<double>(AltitudeReference::Msl));
        CHECK(read.waypoints[1].id == 12);
    }
    SECTION("a curve: its reference, and its segments with the appended ones - its flyout curve") {
        const BezierSegment first = straight(0.0, 2000.0), second = straight(2000.0, 4000.0, 0.0, 500.0);
        const CommandResult k = w.submit(f16, CurveCommand{}, Span<const BezierSegment>(&first, 1));
        REQUIRE(k.accepted());
        REQUIRE(w.activitySetpoint(k.activity, read));
        const auto* curve = commandOf<CurveCommand>(read);
        REQUIRE(curve);
        const double reference = curve->latitudeRad;
        CHECK(std::abs(reference - s.latitudeRad) < 1e-6); // (where the aircraft was)
        CHECK(read.segments.size() == 1);
        fly(w, 1.0);
        CurveCommand append;
        append.append = 1.0;
        REQUIRE(w.update(k.activity, append, Span<const BezierSegment>(&second, 1)).accepted());
        REQUIRE(w.activitySetpoint(k.activity, read));
        REQUIRE(read.segments.size() == 2);
        CHECK(read.segments[1].east[5] == 500.0);
        CHECK(commandOf<CurveCommand>(read)->latitudeRad == reference); // (the same reference)
    }
    SECTION("a support command, and a level's") {
        const CommandResult g = w.submit(f16, SupportCommand(GearCommand{0.0}));
        REQUIRE(g.accepted());
        REQUIRE(w.activitySetpoint(g.activity, read));
        const auto* gear = std::get_if<SupportCommand>(&read.command);
        REQUIRE(gear);
        CHECK(std::get<GearCommand>(*gear).down == 0.0);
        const CommandResult v = w.submit(f16, Command(VelocityCommand{170.0, 2.0, 1.4}));
        REQUIRE(v.accepted());
        REQUIRE(w.activitySetpoint(v.activity, read));
        const auto* velocity = commandOf<VelocityCommand>(read);
        REQUIRE(velocity);
        CHECK((velocity->airspeedMs == 170.0 && velocity->verticalSpeedMs == 2.0 && velocity->headingRad == 1.4));
    }
    SECTION("waiting, as given") {
        HsaCommand h;
        h.headingRad = 2.0;
        const CommandResult later = w.submit(f16, Command(h), startingAt(w.simTime() + 30.0));
        REQUIRE(later.accepted());
        REQUIRE((later.flags & kDeferred) != 0);
        REQUIRE(w.activitySetpoint(later.activity, read));
        const auto* given = commandOf<HsaCommand>(read);
        REQUIRE(given);
        CHECK(given->headingRad == 2.0);
        CHECK(isHold(given->speed)); // (completed as it starts, from where the aircraft is then)
    }
}

TEST_CASE("reports: end points - a route's points and turns, round again, a curve's segment ends, a loiter", "[report]") {
    session::World w(options("reports-end-points"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    fly(w, 1.0);
    const sim::VehicleState& s = *w.vehicleState(f16);
    std::vector<Waypoint> points = {at(s, 0.0, 4000.0, 3000.0, 11), at(s, 4000.0, 8000.0, 3100.0, 12), at(s, 8000.0, 4000.0, 3200.0, 13)};
    points[1].turn = static_cast<double>(TurnType::FlyOver);

    SECTION("a route's: turn points, then its last") {
        const CommandResult r = w.submit(f16, RouteCommand{}, points);
        REQUIRE(r.accepted());
        std::vector<EndPoint> e = w.endPoints(r.activity);
        REQUIRE(e.size() == 3);
        CHECK((e[0].kind == EndPointKind::TurnPoint && e[0].turn == static_cast<double>(TurnType::FlyBy) && e[0].id == 11 && e[0].index == 0));
        CHECK((e[1].kind == EndPointKind::TurnPoint && e[1].turn == static_cast<double>(TurnType::FlyOver) && e[1].id == 12));
        CHECK((e[2].kind == EndPointKind::Waypoint && isHold(e[2].turn) && e[2].id == 13 && e[2].altitudeM == 3200.0));
        CHECK(e[2].latitudeRad == points[2].latitudeRad);
        CHECK(w.endPoints(r.activity, 2).size() == 2);
        // flown on, from the point it flies to
        for (int k = 0; k < 200 && w.activity(r.activity)->progress.segment == 0; ++k) fly(w, 0.5);
        REQUIRE(w.activity(r.activity)->progress.segment == 1);
        e = w.endPoints(r.activity);
        REQUIRE(e.size() == 2);
        CHECK((e[0].id == 12 && e[1].id == 13));
    }
    SECTION("a repeating route's, round again; one that loiters, its last a loiter point") {
        RouteCommand again;
        again.repeat = 1.0;
        const CommandResult r = w.submit(f16, again, points);
        REQUIRE(r.accepted());
        const std::vector<EndPoint> e = w.endPoints(r.activity, 5);
        REQUIRE(e.size() == 5);
        const std::int32_t order[] = {0, 1, 2, 0, 1};
        for (std::size_t i = 0; i < 5; ++i) CHECK((e[i].index == order[i] && e[i].kind == EndPointKind::TurnPoint));
        RouteCommand loiter;
        loiter.end = static_cast<double>(EndBehavior::Loiter);
        const CommandResult l = w.submit(f16, loiter, points);
        REQUIRE(l.accepted());
        CHECK(w.endPoints(l.activity).back().kind == EndPointKind::LoiterPoint);
    }
    SECTION("a curve's segment ends, from its reference") {
        const BezierSegment pieces[] = {straight(0.0, 2000.0), straight(2000.0, 4000.0, 0.0, 500.0)};
        const CommandResult k = w.submit(f16, CurveCommand{}, pieces);
        REQUIRE(k.accepted());
        const std::vector<EndPoint> e = w.endPoints(k.activity);
        REQUIRE(e.size() == 2);
        CHECK(std::abs(e[0].latitudeRad - (s.latitudeRad + 2000.0 / kEarthM)) < 1e-6);
        CHECK(std::abs(e[1].latitudeRad - (s.latitudeRad + 4000.0 / kEarthM)) < 1e-6);
        CHECK(std::abs(e[1].longitudeRad - (s.longitudeRad + 500.0 / (kEarthM * std::cos(s.latitudeRad)))) < 1e-6);
        CHECK((e[1].index == 1 && e[1].kind == EndPointKind::Waypoint && e[1].altitudeReference == static_cast<double>(AltitudeReference::Msl)));
        // one waiting with its reference left out: where it will start is not yet known
        const CommandResult later = w.submit(f16, CurveCommand{}, pieces, startingAt(w.simTime() + 30.0));
        REQUIRE(later.accepted());
        CHECK(w.endPoints(later.activity).empty());
    }
    SECTION("a pattern's loiter point, the position level's point, none for an hsa") {
        PatternCommand orbit;
        orbit.latitudeRad = points[1].latitudeRad, orbit.longitudeRad = points[1].longitudeRad;
        orbit.altitudeM = 3500.0, orbit.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        const CommandResult p = w.submit(f16, Command(orbit));
        REQUIRE(p.accepted());
        std::vector<EndPoint> e = w.endPoints(p.activity);
        REQUIRE(e.size() == 1);
        CHECK((e[0].kind == EndPointKind::LoiterPoint && e[0].latitudeRad == orbit.latitudeRad && e[0].altitudeM == 3500.0 && e[0].index == 0));
        PositionCommand there;
        there.latitudeRad = points[2].latitudeRad, there.longitudeRad = points[2].longitudeRad, there.altitudeMslM = 3200.0;
        const CommandResult q = w.submit(f16, Command(there));
        REQUIRE(q.accepted());
        e = w.endPoints(q.activity);
        REQUIRE(e.size() == 1);
        CHECK((e[0].kind == EndPointKind::Waypoint && e[0].altitudeM == 3200.0 && e[0].altitudeReference == static_cast<double>(AltitudeReference::Msl)));
        HsaCommand h;
        h.headingRad = 1.0;
        const CommandResult a = w.submit(f16, Command(h));
        REQUIRE(a.accepted());
        CHECK(w.endPoints(a.activity).empty());
        CHECK(w.endPoints(q.activity).empty()); // (no longer live)
    }
}

TEST_CASE("reports: the NED acceleration commanded is the one flown, in a turn", "[report]") {
    session::World w(options("reports-acceleration"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    fly(w, 1.0);
    HsaCommand h;
    h.headingRad = -1.0; // (a long turn left from east, at the bank the hsa flies)
    REQUIRE(w.submit(f16, Command(h)).accepted());
    fly(w, 5.0);
    const double dt = w.dt() * w.frameSkip();
    for (int k = 0; k < 4; ++k) { // rolled in: what it asks for is what it flies, within the loops' lag
        fly(w, 0.5);
        const VehicleCommandState c = w.commandState(f16);
        const sim::VehicleState before = *w.vehicleState(f16);
        w.step(1);
        const sim::VehicleState& after = *w.vehicleState(f16);
        INFO("t " << w.simTime() << " roll " << before.eulerRad[0] / kDeg);
        CHECK(c.loadFactorG > 2.0);
        CHECK(std::abs(c.northAccelerationMs2 - (after.velocityNedMs[0] - before.velocityNedMs[0]) / dt) < 1.0);
        CHECK(std::abs(c.eastAccelerationMs2 - (after.velocityNedMs[1] - before.velocityNedMs[1]) / dt) < 1.0);
        CHECK(std::abs(c.downAccelerationMs2 - (after.velocityNedMs[2] - before.velocityNedMs[2]) / dt) < 1.0);
    }
}

TEST_CASE("reports: what the vehicle is commanded - the acceleration where it is, the altitude in its reference", "[report]") {
    session::World w(options("reports-commanded"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    fly(w, 1.0);
    const sim::VehicleState& s = *w.vehicleState(f16);

    SECTION("a pull: a g above gravity's, upward") {
        REQUIRE(w.submit(f16, Command(AccelerationCommand{2.0, 0.0, 0.0})).accepted());
        w.step(2);
        const VehicleCommandState c = w.commandState(f16);
        CHECK(c.loadFactorG == 2.0);
        CHECK(std::abs(c.downAccelerationMs2 + 9.80665) < 1.0);
        CHECK(std::hypot(c.northAccelerationMs2, c.eastAccelerationMs2) < 1.5);
    }
    SECTION("a throttle in place of a longitudinal acceleration: none") {
        AccelerationCommand throttle{1.0, 0.0};
        throttle.throttle = 0.8;
        REQUIRE(w.submit(f16, Command(throttle)).accepted());
        w.step(2);
        const VehicleCommandState c = w.commandState(f16);
        CHECK(c.loadFactorG == 1.0);
        CHECK((std::isnan(c.northAccelerationMs2) && std::isnan(c.eastAccelerationMs2) && std::isnan(c.downAccelerationMs2)));
    }
    SECTION("the altitude as an hsa commanded it, above the ground") {
        HsaCommand h;
        h.altitudeM = 2500.0, h.altitudeReference = static_cast<double>(AltitudeReference::AboveGround);
        REQUIRE(w.submit(f16, Command(h)).accepted());
        w.step(2);
        const VehicleCommandState c = w.commandState(f16);
        CHECK((c.altitudeM == 2500.0 && c.altitudeReference == static_cast<double>(AltitudeReference::AboveGround)));
        CHECK(std::isfinite(c.northAccelerationMs2)); // (its loops command a longitudinal acceleration)
    }
    SECTION("a route's point flown to, a curve's and the position level's above sea level") {
        const std::vector<Waypoint> points = {at(s, 0.0, 5000.0, 3150.0), at(s, 5000.0, 5000.0, 3250.0)};
        REQUIRE(w.submit(f16, RouteCommand{}, points).accepted());
        w.step(2);
        VehicleCommandState c = w.commandState(f16);
        CHECK((c.altitudeM == 3150.0 && c.altitudeReference == static_cast<double>(AltitudeReference::Msl)));
        const BezierSegment piece = straight(0.0, 3000.0);
        REQUIRE(w.submit(f16, CurveCommand{}, Span<const BezierSegment>(&piece, 1)).accepted());
        w.step(2);
        c = w.commandState(f16);
        CHECK(std::abs(c.altitudeM - w.activities(f16).front().progress.altitudeMslM) < 1e-9);
        CHECK(c.altitudeReference == static_cast<double>(AltitudeReference::Msl));
        PositionCommand there;
        there.latitudeRad = points[1].latitudeRad, there.longitudeRad = points[1].longitudeRad, there.altitudeMslM = 3300.0;
        REQUIRE(w.submit(f16, Command(there)).accepted());
        w.step(2);
        c = w.commandState(f16);
        CHECK((c.altitudeM == 3300.0 && c.altitudeReference == static_cast<double>(AltitudeReference::Msl)));
    }
    SECTION("a rotorcraft's: no acceleration (its thrust is not its acceleration)") {
        const auto uh60 = rotor(w, "uh60", 5.0);
        REQUIRE(w.submit(uh60, Command(AccelerationCommand{1.1, 0.0})).accepted());
        w.step(2);
        const VehicleCommandState c = w.commandState(uh60);
        CHECK(c.loadFactorG == 1.1);
        CHECK(std::isnan(c.downAccelerationMs2));
    }
}
