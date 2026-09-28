// A route's turn points as A-GRA's schema gives them (docs/flight-autonomy.md, 4.30; ADR-29 FA-6b): an arc from a start
// turn point to the point after it - ARINC 424's radius to fix - flown by a wing and a rotorcraft; a capture's course; a
// fly-by's radius given, and one too tight clamped or refused; and what does not make a turn point refused, naming it.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

double code(TurnType t) { return static_cast<double>(t); }

/// A point `north` and `east` metres from (lat0, lon0).
Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint p;
    p.latitudeRad = lat0 + north / kR, p.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return p;
}

void apart(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    north = (lat - lat0) * kR, east = (lon - lon0) * kR * std::cos(lat0);
}

} // namespace

TEST_CASE("turn points: an arc from a start turn point to an end turn point, flown by a wing and a rotorcraft; its geometry read back",
          "[modes]") {
    session::World w(options("turn-points-arcs"));
    const auto quad = rotor(w, "iris", 15.0, 6);
    const auto v = wing(w, "c172x", 1500.0, 55.0); // (east)
    w.step(stepsFor(w, 1.0));
    // east 3 km, then a quarter circle of 1,200 m round to the right - south - and on south 3 km
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad, r = 1200.0;
    Waypoint p0 = at(lat0, lon0, 0.0, 3000.0), p1 = at(lat0, lon0, -r, 3000.0 + r), p2 = at(lat0, lon0, -r - 3000.0, 3000.0 + r);
    p0.turn = code(TurnType::StartTurn), p0.turnRadiusM = r; // (its course left out: the leg in's, east)
    p1.turn = code(TurnType::EndTurn), p1.courseRad = kPi; // (south: the arc's there)
    const CommandResult c = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(c.activity, sp));
    CHECK(sp.waypoints[0].turn == code(TurnType::StartTurn));
    CHECK(sp.waypoints[0].turnRadiusM == r);
    CHECK(sp.waypoints[1].courseRad == kPi);
    // a rotorcraft's: north 40 m, a half-circle-less quarter of 30 m round to the left, on west
    const auto& q0 = *w.vehicleState(quad);
    const double qlat = q0.latitudeRad, qlon = q0.longitudeRad, qr = 30.0;
    Waypoint a0 = at(qlat, qlon, 40.0, 0.0), a1 = at(qlat, qlon, 40.0 + qr, -qr), a2 = at(qlat, qlon, 40.0 + qr, -qr - 40.0);
    a0.turn = code(TurnType::StartTurn), a0.turnRadiusM = qr;
    a1.turn = code(TurnType::EndTurn);
    const CommandResult qc = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{a0, a1, a2});
    INFO(reasonName(qc.reason) << " at " << qc.index);
    REQUIRE(qc.accepted());
    // off the arc while its segment is flown, in its middle (its centre 1,200 m south of its start)
    double arcOff = 0.0, qOff = 0.0, arcFirst = 1e18, arcLast = -1e18;
    int arcSamples = 0, qSamples = 0;
    for (unsigned k = 0; k < stepsFor(w, 260.0); ++k) {
        w.step();
        const ActivityProgress& g = w.activity(c.activity)->progress;
        if (w.activity(c.activity)->live() && g.segment == 1 && g.segmentPercent > 15.0 && g.segmentPercent < 85.0) {
            double n, e;
            apart(lat0, lon0, w.vehicleState(v)->latitudeRad, w.vehicleState(v)->longitudeRad, n, e);
            arcOff = std::max(arcOff, std::abs(std::hypot(n + r, e - 3000.0) - r));
            arcFirst = std::min(arcFirst, g.segmentPercent), arcLast = std::max(arcLast, g.segmentPercent);
            ++arcSamples;
        }
        const ActivityProgress& h = w.activity(qc.activity)->progress;
        if (w.activity(qc.activity)->live() && h.segment == 1 && h.segmentPercent > 15.0 && h.segmentPercent < 85.0) {
            double n, e;
            apart(qlat, qlon, w.vehicleState(quad)->latitudeRad, w.vehicleState(quad)->longitudeRad, n, e);
            qOff = std::max(qOff, std::abs(std::hypot(n - 40.0, e + qr) - qr));
            ++qSamples;
        }
    }
    std::printf("turn points, arcs: a C172x %.1f m off its 1,200 m arc (%d samples), an IRIS %.2f m off its 30 m one (%d); both %s, %s\n", arcOff,
                arcSamples, qOff, qSamples, activityStateName(w.activity(c.activity)->state), activityStateName(w.activity(qc.activity)->state));
    CHECK(arcSamples > 100);
    CHECK(qSamples > 20);
    CHECK(arcOff < 15.0);
    CHECK(qOff < 0.5);
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
    CHECK(w.activity(qc.activity)->state == ActivityState::Completed);
}

TEST_CASE("turn points: a capture's course flown; a fly-by's radius given, one too tight clamped or refused", "[modes]") {
    session::World w(options("turn-points-courses"));
    const auto capturer = wing(w, "c172x", 1500.0, 55.0), wide = wing(w, "c172x", 1500.0, 55.0, 3);
    w.step(stepsFor(w, 1.0));
    // over a point 3 km east, then its course captured - north-east, the next point 4 km along it
    const auto& c0 = *w.vehicleState(capturer);
    Waypoint p0 = at(c0.latitudeRad, c0.longitudeRad, 0.0, 3000.0);
    p0.turn = code(TurnType::CaptureOutboundCourse), p0.courseRad = 0.25 * kPi;
    const Waypoint p1 = at(c0.latitudeRad, c0.longitudeRad, 4000.0 * std::cos(0.25 * kPi), 3000.0 + 4000.0 * std::sin(0.25 * kPi));
    const CommandResult cap = w.submit(capturer, RouteCommand{}, std::vector<Waypoint>{p0, p1});
    INFO(reasonName(cap.reason) << " at " << cap.index);
    REQUIRE(cap.accepted());
    // a corner 3 km east, turned north on a 2 km circle given: passed 2 km x (sqrt 2 - 1) from it
    const auto& d0 = *w.vehicleState(wide);
    Waypoint q0 = at(d0.latitudeRad, d0.longitudeRad, 0.0, 4000.0);
    q0.turnRadiusM = 2000.0;
    const Waypoint q1 = at(d0.latitudeRad, d0.longitudeRad, 5000.0, 4000.0);
    const CommandResult by = w.submit(wide, RouteCommand{}, std::vector<Waypoint>{q0, q1});
    REQUIRE(by.accepted());
    double overP0 = 1e18, trackOff = 0.0, nearQ0 = 1e18;
    for (unsigned k = 0; k < stepsFor(w, 150.0); ++k) {
        w.step();
        double n, e;
        const auto& s = *w.vehicleState(capturer);
        apart(p0.latitudeRad, p0.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
        overP0 = std::min(overP0, std::hypot(n, e));
        if (w.activity(cap.activity)->live() && w.activity(cap.activity)->progress.segment == 1 && std::hypot(n, e) > 1500.0)
            trackOff = std::max(trackOff, std::abs(degreesApart(track(s), 0.25 * kPi)));
        apart(q0.latitudeRad, q0.longitudeRad, w.vehicleState(wide)->latitudeRad, w.vehicleState(wide)->longitudeRad, n, e);
        nearQ0 = std::min(nearQ0, std::hypot(n, e));
    }
    std::printf("turn points: over the capture's point %.1f m, then %.2f deg off its course; a 2 km fly-by passed %.0f m off its corner (%.0f)\n", overP0,
                trackOff, nearQ0, 2000.0 * (std::sqrt(2.0) - 1.0));
    CHECK(overP0 < 30.0);
    CHECK(trackOff < 2.0);
    CHECK(std::abs(nearQ0 - 2000.0 * (std::sqrt(2.0) - 1.0)) < 40.0);
    // a radius tighter than full bank turns at its speed: clamped (flagged), or refused naming it under Reject
    q0.turnRadiusM = 100.0;
    const CommandResult tight = w.submit(wide, RouteCommand{}, std::vector<Waypoint>{q0, q1});
    REQUIRE(tight.accepted());
    CHECK((tight.flags & kClamped) != 0);
    Setpoint sp;
    REQUIRE(w.activitySetpoint(tight.activity, sp));
    CHECK(sp.waypoints[0].turnRadiusM > 150.0);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    const CommandResult refused = w.submit(wide, RouteCommand{}, std::vector<Waypoint>{q0, q1}, reject);
    CHECK(refused.reason == Reason::PerformanceLimit);
    CHECK(refused.index == 0);
}

TEST_CASE("turn points: what does not make a turn point is refused, naming it", "[modes]") {
    session::World w(options("turn-points-refusals"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const auto& s = *w.vehicleState(v);
    const double lat0 = s.latitudeRad, lon0 = s.longitudeRad, r = 1200.0;
    auto refusedAt = [&](const std::vector<Waypoint>& points, Reason why, int index) {
        const CommandResult c = w.submit(v, RouteCommand{}, points);
        INFO(reasonName(c.reason) << " at " << c.index);
        CHECK(c.reason == why);
        CHECK(c.index == index);
    };
    Waypoint p0 = at(lat0, lon0, 0.0, 3000.0), p1 = at(lat0, lon0, -r, 3000.0 + r), p2 = at(lat0, lon0, -r - 3000.0, 3000.0 + r);
    p0.turn = code(TurnType::StartTurn), p1.turn = code(TurnType::EndTurn);
    Waypoint x = p0;
    x.turnRadiusM = 1500.0; // (the arc through the next point is 1,200 m)
    refusedAt({x, p1, p2}, Reason::InvalidWaypoint, 1);
    Waypoint y = p1;
    y.courseRad = 0.75 * kPi; // (the arc ends heading south)
    refusedAt({p0, y, p2}, Reason::InvalidWaypoint, 1);
    Waypoint last = p1;
    last.turn = code(TurnType::StartTurn); // (an arc to no point)
    refusedAt({p0, last}, Reason::InvalidWaypoint, 1);
    Waypoint lone = p2;
    lone.turn = code(TurnType::EndTurn); // (a turn nothing began)
    refusedAt({p0, p1, lone}, Reason::InvalidWaypoint, 2);
    Waypoint byCourse = p2;
    byCourse.courseRad = kPi; // (a fly-by has no course)
    refusedAt({p0, p1, byCourse}, Reason::InvalidWaypoint, 2);
    Waypoint overRadius = p2;
    overRadius.turn = code(TurnType::FlyOver), overRadius.turnRadiusM = 500.0; // (nor a fly-over a radius)
    refusedAt({p0, p1, overRadius}, Reason::InvalidWaypoint, 2);
    Waypoint noCourse = p0;
    noCourse.turn = code(TurnType::CaptureOutboundCourse); // (a capture without its course)
    refusedAt({noCourse, p2}, Reason::InvalidWaypoint, 0);
    Waypoint away = p0;
    away.turn = code(TurnType::CaptureOutboundCourse), away.courseRad = 0.0; // (north: the next point is not along it)
    refusedAt({away, p2}, Reason::InvalidWaypoint, 0);
    Waypoint turned = p0;
    turned.kind = static_cast<double>(EndPointKind::Waypoint); // (a waypoint turns none)
    refusedAt({turned, p1, p2}, Reason::InvalidWaypoint, 0);
    Waypoint notOne = p0;
    notOne.turn = 5.0;
    refusedAt({notOne, p1, p2}, Reason::InvalidWaypoint, 0);
    // an arc sweeping more than 170 degrees: its end named - a point between them, each piece less
    Waypoint behind = at(lat0, lon0, -2.0 * r, 3000.0 + 0.1 * r);
    behind.turn = code(TurnType::EndTurn);
    refusedAt({p0, behind}, Reason::InvalidWaypoint, 1);
    // an arc tighter than its full bank turns at its speed: its end named, whatever the policy
    const double small = 150.0;
    Waypoint t0 = at(lat0, lon0, 0.0, 3000.0), t1 = at(lat0, lon0, -small, 3000.0 + small);
    t0.turn = code(TurnType::StartTurn), t1.turn = code(TurnType::EndTurn);
    refusedAt({t0, t1, p2}, Reason::InvalidWaypoint, 1);
}
