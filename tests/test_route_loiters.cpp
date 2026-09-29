// A route's loiter points as A-GRA's schema gives them (docs/flight-autonomy.md, 4.31; ADR-29 FA-6b2): a loiter inside a
// route - an orbit of so many laps, a hold until its end time, a rotorcraft's hover for a time - flown where the leg meets
// it, then on along the route; a route that ends in one; a rotorcraft's stop at a route's end, flown as that hover; its
// loiters read back, reported and refused as its points are; an orbit's laps timed for the endurance check from where it
// is joined.
#include "control/Route.h"
#include "fsim/GuidanceModes.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// A point `north` and `east` metres from (lat0, lon0).
Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint p;
    p.latitudeRad = lat0 + north / kR, p.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return p;
}

void apart(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    north = (lat - lat0) * kR, east = (lon - lon0) * kR * std::cos(lat0);
}

Waypoint loiterPoint(Waypoint p) {
    p.kind = static_cast<double>(EndPointKind::LoiterPoint);
    return p;
}

RouteLoiter loiter(std::uint32_t point, PatternKind kind) {
    RouteLoiter l;
    l.point = point;
    l.pattern.pattern = static_cast<double>(kind);
    return l;
}

Setpoint flown(session::World& w, ActivityId a) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(a, s));
    return s;
}

/// Loitering at point i: its segment flown to its end (4.31).
bool loiteringAt(const ActivityRecord& a, std::uint32_t i) { return a.live() && a.progress.segment == i && a.progress.segmentPercent == 100.0; }

} // namespace

TEST_CASE("route loiters: a wing orbits a point inside a route for its laps, then flies on to the route's end", "[modes]") {
    session::World w(options("route-loiters-orbit"));
    const auto v = wing(w, "c172x", 1500.0, 55.0); // (east)
    w.step(stepsFor(w, 1.0));
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    // east 4 km and 10 km - twice round an orbit there - then 4 km north of it
    const Waypoint p0 = at(lat0, lon0, 0.0, 4000.0), p1 = loiterPoint(at(lat0, lon0, 0.0, 10000.0)), p2 = at(lat0, lon0, 4000.0, 10000.0);
    RouteLoiter orbit = loiter(1, PatternKind::Orbit);
    orbit.shape.orbits = 2.0;
    const CommandResult c = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2}, {}, std::vector<RouteLoiter>{orbit});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    // read back complete, its place left out (its point's)
    const Setpoint sp = flown(w, c.activity);
    REQUIRE(sp.loiters.size() == 1);
    const RouteLoiter held = sp.loiters[0];
    CHECK(held.point == 1);
    CHECK((isHold(held.pattern.latitudeRad) && isHold(held.pattern.longitudeRad) && isHold(held.pattern.altitudeM)));
    CHECK(held.pattern.pattern == static_cast<double>(PatternKind::Orbit));
    CHECK(held.pattern.clockwise == 1.0);
    CHECK(held.shape.orbits == 2.0);
    const double r = held.pattern.radiusM;
    CHECK(std::abs(r - w.performance(v)->turnRadiusM(sp.waypoints[1].speed)) < 1.0); // (its speed's and 80 % of its bank's, calm)
    CHECK(held.pattern.speed == sp.waypoints[1].speed);                               // (the point's segment's)
    const std::vector<EndPoint> ends = w.endPoints(c.activity, 4);
    REQUIRE(ends.size() == 3);
    CHECK(ends[1].kind == EndPointKind::LoiterPoint);
    // flown: the leg, along the tangent onto its circle from a radius outside it, twice round from there and on round to
    // where its tangent runs to the next point, then on north direct
    double swept = 0.0, lastBearing = kHold, worst = 0.0, loiterS = 0.0, off = 0.0, metAt = kHold, leftAt = kHold, completedAt = kHold;
    double timeToGo = kHold;
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 560.0) && isHold(completedAt); ++k) {
        w.step();
        const ActivityRecord& a = *w.activity(c.activity);
        double n, e;
        apart(p1.latitudeRad, p1.longitudeRad, w.vehicleState(v)->latitudeRad, w.vehicleState(v)->longitudeRad, n, e);
        if (loiteringAt(a, 1)) {
            if (isHold(metAt)) metAt = std::hypot(n, e);
            loiterS += w.dt() * w.frameSkip();
            const double bearing = std::atan2(e, n);
            if (!isHold(lastBearing)) swept += std::remainder(bearing - lastBearing, 2.0 * kPi);
            lastBearing = bearing;
            if (swept > kPi) worst = std::max(worst, std::abs(std::hypot(n, e) - r)); // (after half a lap: joined)
            if (swept > kPi && isHold(timeToGo)) timeToGo = a.progress.timeToGoS;    // (its laps' and the leg's after it, joined)
        } else if (!isHold(lastBearing) && isHold(leftAt)) {
            leftAt = w.simTime() - t0;
        }
        if (!isHold(leftAt) && a.progress.segment == 2 && a.progress.segmentPercent > 50.0) off = std::max(off, std::abs(a.progress.crossTrackM));
        if (a.state == ActivityState::Completed) completedAt = w.simTime() - t0;
    }
    std::printf("route loiters, an orbit: R %.0f m, met %.0f m from its point, %.2f laps swept in %.0f s (%.0f s to go half a lap in), %.1f m "
                "off its circle once joined; left at %.0f s, then %.1f m off the leg direct on (its second half); completed at %.0f s\n",
                r, metAt, swept / (2.0 * kPi), loiterS, timeToGo, worst, leftAt, off, completedAt);
    CHECK(std::abs(metAt - 2.0 * r) < 60.0);
    CHECK(timeToGo > 60.0);
    CHECK(swept / (2.0 * kPi) > 1.95);
    CHECK(swept / (2.0 * kPi) < 3.1); // (its laps from where it joins its circle, and on round to where it leaves for the next point)
    CHECK(worst < 0.05 * r);
    CHECK(off < 5.0);
    CHECK(!isHold(completedAt));
}

TEST_CASE("route loiters: a rotorcraft stops and hovers at a point inside its route for its time, then flies on; its route's last, a hover "
          "until canceled: completed as it arrives, it hovers on",
          "[modes]") {
    session::World w(options("route-loiters-hover"));
    const auto quad = rotor(w, "iris", 15.0, 0);
    const auto& s0 = *w.vehicleState(quad);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    // north 50 m; east 50 m, a 15 s hover there; south 50 m, a hover there until canceled
    const Waypoint q0 = at(lat0, lon0, 50.0, 0.0), q1 = loiterPoint(at(lat0, lon0, 50.0, 50.0)), q2 = loiterPoint(at(lat0, lon0, 0.0, 50.0));
    RouteLoiter hover = loiter(1, PatternKind::Hover);
    hover.pattern.durationS = 15.0;
    const RouteLoiter stay = loiter(2, PatternKind::Hover);
    const CommandResult c = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{q0, q1, q2}, {}, std::vector<RouteLoiter>{hover, stay});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    double arrivedAt = kHold, leftAt = kHold, over = 0.0, completedAt = kHold, fastest = 0.0, drift = 0.0;
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 120.0); ++k) {
        w.step();
        const ActivityRecord& a = *w.activity(c.activity);
        const auto& s = *w.vehicleState(quad);
        const double t = w.simTime() - t0;
        double n, e;
        apart(q1.latitudeRad, q1.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
        if (isHold(arrivedAt) && std::hypot(n, e) < 1.0) arrivedAt = t;
        if (!isHold(arrivedAt) && isHold(leftAt)) {
            over = std::max(over, std::hypot(n, e));
            if (!loiteringAt(a, 1) && a.progress.segment == 2) leftAt = t;
        }
        if (!isHold(leftAt) && a.progress.segment == 2 && isHold(completedAt)) fastest = std::max(fastest, groundSpeed(s));
        if (isHold(completedAt) && a.state == ActivityState::Completed) completedAt = t;
        if (!isHold(completedAt) && t - completedAt > 5.0) { // (hovering on over its last point)
            apart(q2.latitudeRad, q2.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
            drift = std::max(drift, std::hypot(n, e));
        }
    }
    std::printf("route loiters, a hover: arrived at %.1f s, left at %.1f s (%.1f s there), within %.2f m of it; on at %.1f m/s at most; completed "
                "at %.1f s, then %.2f m off its last point\n",
                arrivedAt, leftAt, leftAt - arrivedAt, over, fastest, completedAt, drift);
    REQUIRE(!isHold(leftAt));
    CHECK(leftAt - arrivedAt > 14.9);
    CHECK(leftAt - arrivedAt < 19.0); // (its time from its arrival, within a metre and two of its altitude)
    CHECK(over < 1.5);
    CHECK(!isHold(completedAt));
    CHECK(drift < 1.0);
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
}

TEST_CASE("route loiters: a route that ends in a hover stops as the hover pattern and a loiter point's hover do - its position loop's "
          "approach from where it would stop - and completes at its point",
          "[modes]") {
    // every rotorcraft three ways to a point eight turn radii east at its cruise over the ground: the hover pattern, its
    // position loop all the way; a route to a loiter point with a hover (4.31); a route that ends in a loiter, its end stop -
    // how far each swings past its point, and where the route that ends in a loiter completes
    session::World w(options("route-loiters-stops"));
    const char* const types[] = {"cf2", "iris", "uh1h", "uh60"};
    const char* const ways[] = {"pattern", "loiter point", "end"};
    struct Stop {
        std::uint32_t v = 0;
        ActivityId a = 0;
        Waypoint point;
        double past = -1e9, speedAbeam = kHold, arrived = kHold, settled = kHold, after = 0.0, completed = kHold, offAtCompletion = kHold;
    };
    Stop stops[4][3];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) stops[i][j].v = rotor(w, types[i], 0.0, 3 * i + j);
    w.step(stepsFor(w, 15.0)); // (settled together: one let go flies its neutral default and falls)
    const double t0 = w.simTime();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) {
            Stop& s = stops[i][j];
            const auto& s0 = *w.vehicleState(s.v);
            const Performance& perf = *w.performance(s.v);
            const double cruise = perf.cruiseTasMs, r = perf.turnRadiusM(cruise);
            s.point = at(s0.latitudeRad, s0.longitudeRad, 0.0, 8.0 * r);
            s.point.altitudeM = s0.altitudeMslM;
            s.point.speed = cruise, s.point.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            CommandResult c;
            if (j == 0) {
                PatternCommand hover;
                hover.pattern = static_cast<double>(PatternKind::Hover);
                hover.latitudeRad = s.point.latitudeRad, hover.longitudeRad = s.point.longitudeRad, hover.altitudeM = s.point.altitudeM;
                hover.speed = s.point.speed, hover.speedReference = s.point.speedReference;
                c = w.submit(s.v, hover);
            } else if (j == 1) {
                c = w.submit(s.v, RouteCommand{}, std::vector<Waypoint>{loiterPoint(s.point)}, {}, std::vector<RouteLoiter>{loiter(0, PatternKind::Hover)});
            } else {
                RouteCommand stop;
                stop.end = static_cast<double>(EndBehavior::Loiter);
                c = w.submit(s.v, stop, std::vector<Waypoint>{s.point});
            }
            INFO(types[i] << " " << ways[j] << ": " << reasonName(c.reason) << " at " << c.index);
            REQUIRE(c.accepted());
            s.a = c.activity;
        }
    for (unsigned k = 0; k < stepsFor(w, 300.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        for (auto& row : stops)
            for (Stop& s : row) {
                const auto& v = *w.vehicleState(s.v);
                double n, e;
                apart(s.point.latitudeRad, s.point.longitudeRad, v.latitudeRad, v.longitudeRad, n, e);
                const double off = std::hypot(n, e);
                s.past = std::max(s.past, e); // (along its way there, east: past it)
                if (isHold(s.speedAbeam) && e > -1.0) s.speedAbeam = groundSpeed(v);
                if (isHold(s.arrived) && off < 1.0) s.arrived = t;
                if (!isHold(s.arrived)) s.after = std::max(s.after, off);
                if (!isHold(s.arrived) && (isHold(s.settled) || off >= 1.0)) s.settled = off < 1.0 ? t : kHold; // (within the metre for good)
                if (isHold(s.completed) && w.activity(s.a)->state == ActivityState::Completed) s.completed = t, s.offAtCompletion = off;
            }
    }
    for (int i = 0; i < 4; ++i) {
        const Stop &pattern = stops[i][0], &loiterHover = stops[i][1], &end = stops[i][2];
        const Performance& perf = *w.performance(end.v);
        std::printf("route stops, %s (R %.1f m, stopping %.1f m from %.2f m/s): past its point the pattern's %.2f m, a loiter point's %.2f m, its "
                    "end's %.2f m (abeam at %.2f m/s); its end within a metre at %.1f s, for good from %.1f s, then within %.2f m; completed "
                    "at %.1f s, %.2f m from it\n",
                    types[i], perf.turnRadiusM(perf.cruiseTasMs), route::stoppingDistanceM(perf, perf.cruiseTasMs), perf.cruiseTasMs, pattern.past,
                    loiterHover.past, end.past, end.speedAbeam, end.arrived, end.settled, end.after, end.completed, end.offAtCompletion);
        INFO(types[i]);
        for (const Stop* s : {&pattern, &loiterHover, &end}) {
            INFO(ways[s - stops[i]]);
            REQUIRE(!isHold(s->arrived));
        }
        // as the hover pattern and a loiter point's hover: no farther past it (the worst: the Crazyflie's 0.78 m, a loiter
        // point's too; handed over within a metre of it, it swung 1.40 m past it, the IRIS 1.20 m, the UH-60A 12.4 m and the
        // UH-1H 42.2 m, then 49.1 m from it)
        CHECK(end.past < std::max(pattern.past, loiterHover.past) + 0.1);
        CHECK(end.past < 1.0);
        CHECK(end.after < 1.5);
        // completed at its point, as it gets there: within a metre of it along its leg
        CHECK(end.offAtCompletion < 1.5);
        CHECK(std::abs(end.completed - end.arrived) < 1.0);
    }
}

TEST_CASE("route loiters: a rotorcraft whose stop reaches back into the turn before its point slows on the turn and stops at the point, "
          "calm and in a tailwind - a route's end and a loiter point's hover alike, its nose on the last leg's course",
          "[modes]") {
    // every rotorcraft from its hover 4R east at its cruise over the ground (R its turn radius there), a fly-by turn right,
    // then its last point R and half its stopping distance south: its stop reaches back into the turn's arc by half that
    // distance (4.31). Calm, and a tailwind on its last leg at 0.4 of its cruise (the world's wind: a world each); a route that
    // ends in a loiter, and one whose last point is a loiter point with a hover - how far past its point along its last leg,
    // how far from it once it passed it, and its nose over it
    const char* const types[] = {"cf2", "iris", "uh1h", "uh60"};
    const char* const ways[] = {"end", "loiter point"};
    for (const char* type : types)
        for (const double tailwind : {0.0, 0.4}) {
            session::World w(options("route-loiters-turn-stop"));
            std::uint32_t v[2];
            for (int j = 0; j < 2; ++j) v[j] = rotor(w, type, 0.0, j);
            const Performance& perf = *w.performance(v[0]);
            const double cruise = perf.cruiseTasMs, r = perf.turnRadiusM(cruise), stop = route::stoppingDistanceM(perf, cruise);
            setWind(w, 0.0, tailwind * cruise); // (from the north: behind it on its last leg, south)
            w.step(stepsFor(w, 15.0));          // (settled together, in the wind: one let go flies its neutral default and falls)
            struct Stop {
                ActivityId a = 0;
                Waypoint point;
                double past = 0.0, passed = kHold, after = 0.0, settled = kHold, heading = kHold, completedOff = kHold;
            } stops[2];
            for (int j = 0; j < 2; ++j) {
                Stop& s = stops[j];
                const auto& s0 = *w.vehicleState(v[j]);
                Waypoint turn = at(s0.latitudeRad, s0.longitudeRad, 0.0, 4.0 * r);
                turn.speed = cruise, turn.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
                s.point = at(s0.latitudeRad, s0.longitudeRad, -(r + 0.5 * stop), 4.0 * r);
                CommandResult c;
                if (j == 0) {
                    RouteCommand end;
                    end.end = static_cast<double>(EndBehavior::Loiter);
                    c = w.submit(v[j], end, std::vector<Waypoint>{turn, s.point});
                } else {
                    c = w.submit(v[j], RouteCommand{}, std::vector<Waypoint>{turn, loiterPoint(s.point)}, {},
                                 std::vector<RouteLoiter>{loiter(1, PatternKind::Hover)});
                }
                INFO(type << " " << ways[j] << ": " << reasonName(c.reason) << " at " << c.index);
                REQUIRE(c.accepted());
                s.a = c.activity;
            }
            const double t0 = w.simTime();
            for (unsigned k = 0; k < stepsFor(w, 9.0 * r / cruise + 20.0 / perf.velocityBandwidthRadS + 60.0); ++k) {
                w.step();
                const double t = w.simTime() - t0;
                for (Stop& s : stops) {
                    const auto& vs = *w.vehicleState(v[&s - stops]);
                    const ActivityRecord& a = *w.activity(s.a);
                    double n, e;
                    apart(s.point.latitudeRad, s.point.longitudeRad, vs.latitudeRad, vs.longitudeRad, n, e);
                    const double off = std::hypot(n, e);
                    const bool last = a.progress.segment == 1 || a.state == ActivityState::Completed; // (on its last leg, or there)
                    if (last) s.past = std::max(s.past, -n); // (its last leg runs south: past it, south of it)
                    if (last && isHold(s.passed) && -n >= -1.0) s.passed = t;
                    if (!isHold(s.passed)) s.after = std::max(s.after, off);
                    s.settled = off >= 1.0 ? kHold : isHold(s.settled) ? t : s.settled; // (within the metre for good)
                    s.heading = vs.eulerRad[2];
                    if (isHold(s.completedOff) && a.state == ActivityState::Completed) s.completedOff = off;
                }
            }
            for (const Stop& s : stops) {
                const char* way = ways[&s - stops];
                std::printf("route turn stops, %s %s (R %.1f m, stopping %.1f m from %.2f m/s; tailwind %.2f m/s): past its point %.2f m, then "
                            "within %.2f m of it; within a metre for good from %.1f s; its nose %.2f deg off the leg's course; completed %.2f m "
                            "from it\n",
                            type, way, r, stop, cruise, tailwind * cruise, s.past, s.after, s.settled, degreesApart(s.heading, kPi), s.completedOff);
                INFO(type << " " << way << ", tailwind " << tailwind * cruise << " m/s");
                CHECK(w.activity(s.a)->state == ActivityState::Completed);
                CHECK(!isHold(s.settled));
                // (the worst: the Crazyflie 1.40 m past its point in the tailwind, the UH-1H 1.73 m from it once there, calm,
                // every nose on the course. Its stop not counted on the turn and its nose turned to the point, the UH-1H
                // swung 14.4 m past it, then to 17.5 m from it, its nose 75 degrees off the course, and the UH-60A to 6.3 m
                // from it. Slowed on the turn alone, the UH-1H 10.4 m past and 18.6 m from it; the UH-60A 1.26 m)
                CHECK(s.past < 3.0);
                CHECK(s.after < 3.5);
                CHECK(std::abs(degreesApart(s.heading, kPi)) < 2.0);
            }
        }
}

TEST_CASE("route loiters: a wing holds at a fix until its end time and leaves at the fix; a loiter on a moving point moves with it; a route "
          "ending in an orbit completes as it joins it and orbits on",
          "[modes]") {
    session::World w(options("route-loiters-hold"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), shipped = wing(w, "c172x", 1500.0, 55.0, 3);
    w.step(stepsFor(w, 1.0));
    const double t0 = w.simTime();
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    // east 4 km; a hold at 10 km east until 300 s from now, left at its fix; an orbit 5 km north of it, the route's end
    const Waypoint p0 = at(lat0, lon0, 0.0, 4000.0), p1 = loiterPoint(at(lat0, lon0, 0.0, 10000.0)), p2 = loiterPoint(at(lat0, lon0, 5000.0, 10000.0));
    RouteLoiter hold = loiter(1, PatternKind::Hold);
    hold.endTimeS = t0 + 300.0;
    hold.shape.exitLatitudeRad = p1.latitudeRad, hold.shape.exitLongitudeRad = p1.longitudeRad;
    const RouteLoiter orbit = loiter(2, PatternKind::Orbit);
    const CommandResult c = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2}, {}, std::vector<RouteLoiter>{hold, orbit});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    const Setpoint sp = flown(w, c.activity);
    REQUIRE(sp.loiters.size() == 2);
    const double inbound = sp.loiters[0].pattern.courseRad, legs = sp.loiters[0].pattern.legM, turn = sp.loiters[0].pattern.radiusM;
    CHECK(std::abs(std::remainder(inbound - 0.5 * kPi, 2.0 * kPi)) < 0.01); // (its inbound course the leg's, arriving: east)
    CHECK(sp.loiters[0].endTimeS == hold.endTimeS);
    const double r = sp.loiters[1].pattern.radiusM;
    // a ship 8 km east of the second, moving north at 6 m/s: once round an orbit 500 m to its left, then 3 km on east
    const auto& b0 = *w.vehicleState(shipped);
    FrameSpec ship;
    ship.origin = FrameOrigin::Moving;
    ship.latitudeRad = b0.latitudeRad, ship.longitudeRad = b0.longitudeRad + 8000.0 / (kR * std::cos(b0.latitudeRad)), ship.northMs = 6.0;
    ship.timeS = w.simTime();
    const FrameId shipId = w.createFrame(ship);
    Waypoint deck = loiterPoint(Waypoint{});
    deck.frame = static_cast<double>(shipId), deck.frameYM = -500.0; // (y: east, x: north, unrotated)
    RouteLoiter round = loiter(1, PatternKind::Orbit);
    round.shape.orbits = 1.0;
    const CommandResult sc = w.submit(shipped, RouteCommand{}, std::vector<Waypoint>{at(b0.latitudeRad, b0.longitudeRad, 0.0, 3000.0), deck,
                                                                                    at(b0.latitudeRad, b0.longitudeRad, 3000.0, 11000.0)},
                                      {}, std::vector<RouteLoiter>{round});
    INFO(reasonName(sc.reason) << " at " << sc.index);
    REQUIRE(sc.accepted());
    const Setpoint ss = flown(w, sc.activity);
    REQUIRE(ss.loiters.size() == 1);
    CHECK(isHold(ss.loiters[0].shape.frame)); // (kept with its place left out: the point's frame, its own again as it flies)
    const double shipR = ss.loiters[0].pattern.radiusM;
    double east = -1e18, south = 0.0, west = 0.0, leftAt = kHold, leftFrom = kHold, completedAt = kHold, near = 1e18, far = 0.0;
    double shipOff = 0.0, shipLeftAt = kHold, shipSwept = 0.0, shipBearing = kHold;
    bool held = false;
    for (unsigned k = 0; k < stepsFor(w, 700.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        const ActivityRecord& a = *w.activity(c.activity);
        const auto& s = *w.vehicleState(v);
        double n, e;
        apart(p1.latitudeRad, p1.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
        if (loiteringAt(a, 1)) held = true, east = std::max(east, e), south = std::min(south, n), west = std::min(west, e);
        else if (held && isHold(leftAt)) leftAt = t, leftFrom = std::hypot(n, e);
        if (isHold(completedAt) && a.state == ActivityState::Completed) completedAt = t;
        if (!isHold(completedAt) && t - completedAt > 120.0) { // (round its last point, a minute after joining)
            apart(p2.latitudeRad, p2.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
            near = std::min(near, std::hypot(n, e)), far = std::max(far, std::hypot(n, e));
        }
        // the ship's: round its point where the ship is now (500 m west of its origin), from half a lap in
        const ActivityRecord& b = *w.activity(sc.activity);
        if (loiteringAt(b, 1)) {
            const FramePose pose = framePose(ship, w.simTime());
            apart(pose.latitudeRad, pose.longitudeRad, w.vehicleState(shipped)->latitudeRad, w.vehicleState(shipped)->longitudeRad, n, e);
            const double bearing = std::atan2(e + 500.0, n);
            if (!isHold(shipBearing)) shipSwept += std::remainder(bearing - shipBearing, 2.0 * kPi);
            shipBearing = bearing;
            if (shipSwept > kPi) shipOff = std::max(shipOff, std::abs(std::hypot(n, e + 500.0) - shipR));
        } else if (b.progress.segment == 2 && isHold(shipLeftAt)) {
            shipLeftAt = t;
        }
    }
    std::printf("route loiters, a hold: legs %.0f m, turns R %.0f m; flown %.0f m east of its fix to %.0f m west, %.0f m south; left at %.0f s "
                "(its end time 300 s) %.1f m from its fix; its route's end: completed at %.0f s, round it %.0f to %.0f m (R %.0f); round a "
                "ship's point %.2f laps, %.1f m off (R %.0f), left at %.0f s\n",
                legs, turn, east, -west, -south, leftAt, leftFrom, completedAt, near, far, r, shipSwept / (2.0 * kPi), shipOff, shipR, shipLeftAt);
    CHECK(held);
    CHECK(east < turn + 100.0);         // (the turn at its fix beyond it)
    CHECK(-west < legs + turn + 300.0); // (its legs back along its inbound course)
    CHECK(-south < 2.0 * turn + 300.0); // (on its right: south)
    CHECK(leftAt >= 300.0);
    CHECK(leftFrom < 10.0); // (at its exit point: the fix)
    CHECK(!isHold(completedAt));
    CHECK(completedAt > leftAt);
    CHECK(near > 0.9 * r);
    CHECK(far < 1.1 * r);
    CHECK(shipSwept / (2.0 * kPi) > 0.95);
    CHECK(shipOff < 0.1 * shipR);
    CHECK(!isHold(shipLeftAt));
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
}

TEST_CASE("route loiters: refused as a point is, naming it - no loiter for a loiter point, a loiter at none, its own place, no end before the "
          "route's, a hover where the aircraft does not hover; limited as a pattern, named by its point and its field after the point's",
          "[modes]") {
    session::World w(options("route-loiters-refusals"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), declared = wing(w, "c172", 1500.0, 50.0, 3); // (hangar's C172: its design's, and tables)
    w.step(stepsFor(w, 1.0));
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    const Waypoint p0 = at(lat0, lon0, 0.0, 4000.0), p1 = loiterPoint(at(lat0, lon0, 0.0, 8000.0)), p2 = at(lat0, lon0, 4000.0, 8000.0);
    RouteLoiter ok = loiter(1, PatternKind::Orbit);
    ok.pattern.durationS = 60.0;
    auto refusedAt = [&](std::vector<Waypoint> points, std::vector<RouteLoiter> loiters, Reason why, int index, const char* what) {
        const CommandResult r = w.submit(v, RouteCommand{}, points, {}, loiters);
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == index);
    };
    // its pairing: a loiter point with none, a loiter at a point that is not one or past the route, two for one point
    refusedAt({p0, p1, p2}, {}, Reason::InvalidWaypoint, 1, "no loiter");
    RouteLoiter at0 = ok;
    at0.point = 0;
    refusedAt({p0, p1, p2}, {ok, at0}, Reason::InvalidWaypoint, 0, "at a waypoint");
    RouteLoiter past = ok;
    past.point = 7;
    refusedAt({p0, p1, p2}, {ok, past}, Reason::InvalidWaypoint, 7, "past the route");
    refusedAt({p0, p1, p2}, {ok, ok}, Reason::InvalidWaypoint, 1, "two");
    // its place its point's: a latitude, an altitude, a frame of its own
    RouteLoiter placed = ok;
    placed.pattern.latitudeRad = lat0, placed.pattern.longitudeRad = lon0;
    refusedAt({p0, p1, p2}, {placed}, Reason::InvalidWaypoint, 1, "its own point");
    RouteLoiter high = ok;
    high.pattern.altitudeM = 2000.0;
    refusedAt({p0, p1, p2}, {high}, Reason::InvalidWaypoint, 1, "its own altitude");
    RouteLoiter framed = ok;
    framed.shape.frameXM = 10.0;
    refusedAt({p0, p1, p2}, {framed}, Reason::InvalidWaypoint, 1, "its own offsets");
    // an end - duration, laps, end time - but at the last point of a route that does not repeat; an end time finite
    RouteLoiter endless = ok;
    endless.pattern.durationS = kHold;
    refusedAt({p0, p1, p2}, {endless}, Reason::InvalidWaypoint, 1, "no end");
    RouteLoiter forever = ok;
    forever.endTimeS = std::numeric_limits<double>::infinity();
    refusedAt({p0, p1, p2}, {forever}, Reason::InvalidWaypoint, 1, "an end time not finite");
    Waypoint last = p2;
    last.kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter atEnd = endless;
    atEnd.point = 2;
    CHECK(w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, last}, {}, std::vector<RouteLoiter>{ok, atEnd}).accepted()); // (the route's end)
    RouteCommand again;
    again.repeat = 1.0;
    const CommandResult round = w.submit(v, again, std::vector<Waypoint>{p0, p1, last}, {}, std::vector<RouteLoiter>{ok, atEnd});
    CHECK((round.reason == Reason::InvalidWaypoint && round.index == 2)); // (a route that repeats never ends)
    // its fields as a pattern NEW's: a kind that is not one, a radius not above 0; a turn or its radius at a loiter point
    RouteLoiter kind = ok;
    kind.pattern.pattern = 7.0;
    refusedAt({p0, p1, p2}, {kind}, Reason::InvalidWaypoint, 1, "a kind");
    RouteLoiter flat = ok;
    flat.pattern.radiusM = -5.0;
    refusedAt({p0, p1, p2}, {flat}, Reason::InvalidWaypoint, 1, "a radius");
    Waypoint turned = p1;
    turned.turn = static_cast<double>(TurnType::StartTurn);
    refusedAt({p0, turned, p2}, {ok}, Reason::InvalidWaypoint, 1, "a turn type");
    Waypoint radius = p1;
    radius.turnRadiusM = 500.0;
    refusedAt({p0, radius, p2}, {ok}, Reason::InvalidWaypoint, 1, "a turn radius");
    // a wing's hover, as the hover pattern's row says: not supported where its design says it does not fly on rotors, not
    // implemented where nothing says (a stock model's); an optimisation with no tables to fly it from - with them, flown
    RouteLoiter hover = loiter(1, PatternKind::Hover);
    hover.pattern.durationS = 30.0;
    refusedAt({p0, p1, p2}, {hover}, Reason::NotImplemented, 1, "a stock model's hover");
    CHECK(w.supportTable(v)->find("fsim.guidance.pattern/hover")->support == Support::NotImplemented);
    const auto& d0 = *w.vehicleState(declared);
    const Waypoint d1 = loiterPoint(at(d0.latitudeRad, d0.longitudeRad, 0.0, 8000.0));
    const CommandResult rotorless = w.submit(declared, RouteCommand{}, std::vector<Waypoint>{d1}, {}, std::vector<RouteLoiter>{loiter(0, PatternKind::Hover)});
    CHECK((rotorless.reason == Reason::NotSupported && rotorless.index == 0));
    CHECK(w.supportTable(declared)->find("fsim.guidance.pattern/hover")->support == Support::NotSupported);
    RouteLoiter best = ok;
    best.pattern.speedOptimization = static_cast<double>(SpeedOptimization::MaxEndurance);
    REQUIRE(w.profile(v)->tables.empty()); // (the stock C172's: no tables)
    refusedAt({p0, p1, p2}, {best}, Reason::NotImplemented, 1, "an optimisation");
    RouteLoiter bestAt0 = best;
    bestAt0.point = 0;
    const CommandResult tabled = w.submit(declared, RouteCommand{}, std::vector<Waypoint>{d1}, {}, std::vector<RouteLoiter>{bestAt0});
    INFO(reasonName(tabled.reason) << " at " << tabled.index);
    REQUIRE(tabled.accepted());
    const RouteLoiter optimised = flown(w, tabled.activity).loiters[0];
    const double fuel = w.vehicleState(declared)->fuelKg;
    CHECK(std::abs(optimised.pattern.speed - optimalTasMs(&w.profile(declared)->tables, best.pattern.speedOptimization,
                                                           flown(w, tabled.activity).waypoints[0].altitudeM, fuel)) < 1e-6);
    // 16 a route at most
    std::vector<Waypoint> many;
    std::vector<RouteLoiter> loiters;
    for (std::uint32_t i = 0; i < 17; ++i) {
        many.push_back(loiterPoint(at(lat0, lon0, 0.0, 3000.0 * (i + 1))));
        RouteLoiter l = ok;
        l.point = i;
        loiters.push_back(l);
    }
    refusedAt(many, loiters, Reason::InvalidWaypoint, 16, "a seventeenth");
    // limited as a pattern: a radius tighter than its full bank flies, held to it - named by its point, its field 100 + 5
    RouteLoiter tight = ok;
    tight.pattern.radiusM = 50.0;
    const CommandResult clamped = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2}, {}, std::vector<RouteLoiter>{tight});
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped));
    const CommandDetails d = w.commandDetails(v) ? *w.commandDetails(v) : CommandDetails{};
    REQUIRE(d.adjustmentCount >= 1);
    CHECK(d.adjustments[0].index == 1);
    CHECK(d.adjustments[0].field == 100 + 5);
    CHECK(d.adjustments[0].constraint == Constraint::MaxOrientation);
    CHECK(flown(w, clamped.activity).loiters[0].pattern.radiusM == d.adjustments[0].adjusted);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    const CommandResult refused = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2}, reject, std::vector<RouteLoiter>{tight});
    CHECK((refused.reason == Reason::PerformanceLimit && refused.index == 1 && refused.constraint == Constraint::MaxOrientation));
}

TEST_CASE("route loiters: kept by an UPDATE of its options, replaced with its waypoints; a waiting route's and a task's flown with their "
          "loiters; validated",
          "[modes]") {
    session::World w(options("route-loiters-update"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    const Waypoint p0 = at(lat0, lon0, 0.0, 4000.0), p1 = loiterPoint(at(lat0, lon0, 0.0, 8000.0)), p2 = at(lat0, lon0, 4000.0, 8000.0);
    RouteLoiter orbit = loiter(1, PatternKind::Orbit);
    orbit.shape.orbits = 3.0;
    const std::vector<Waypoint> points{p0, p1, p2};
    const std::vector<RouteLoiter> loiters{orbit};
    // validated: as a NEW would be answered, nothing flies
    CommandOptions check;
    check.validateOnly = true;
    CHECK(w.submit(v, RouteCommand{}, points, check, loiters).status == CommandStatus::Valid);
    const CommandResult c = w.submit(v, RouteCommand{}, points, {}, loiters);
    REQUIRE(c.accepted());
    // its options alone: its waypoints and loiters kept
    RouteCommand rhumb;
    rhumb.projection = 1.0, rhumb.repeat = rhumb.end = rhumb.start = kHold;
    REQUIRE(w.update(c.activity, rhumb, {}).accepted());
    Setpoint sp = flown(w, c.activity);
    REQUIRE(sp.loiters.size() == 1);
    CHECK(sp.loiters[0].shape.orbits == 3.0);
    // new waypoints come with their loiters: none for a loiter point, refused; given, replaced
    const CommandResult bare = w.update(c.activity, rhumb, points);
    CHECK((bare.reason == Reason::InvalidWaypoint && bare.index == 1));
    RouteLoiter twice = orbit;
    twice.shape.orbits = 2.0;
    REQUIRE(w.update(c.activity, rhumb, points, std::vector<RouteLoiter>{twice}).accepted());
    CHECK(flown(w, c.activity).loiters[0].shape.orbits == 2.0);
    CHECK(w.update(c.activity, rhumb, std::vector<Waypoint>{p0, p2}).accepted()); // (no loiter point: none)
    CHECK(flown(w, c.activity).loiters.empty());
    REQUIRE(w.cancel(c.activity).status == CommandStatus::Canceled);
    // waiting for its start window: read back as given, then flown with them
    CommandOptions later;
    later.window.startNotBefore = w.simTime() + 5.0;
    const CommandResult waiting = w.submit(v, RouteCommand{}, points, later, loiters);
    REQUIRE(waiting.accepted());
    CHECK((waiting.flags & kDeferred));
    sp = flown(w, waiting.activity);
    REQUIRE(sp.loiters.size() == 1);
    CHECK(sp.loiters[0].shape.orbits == 3.0);
    CHECK(isHold(sp.loiters[0].pattern.radiusM)); // (as given: completed as it starts)
    w.step(stepsFor(w, 8.0));
    REQUIRE(w.activity(waiting.activity)->state == ActivityState::Active);
    sp = flown(w, waiting.activity);
    REQUIRE(sp.loiters.size() == 1);
    CHECK(sp.loiters[0].pattern.radiusM > 100.0);
    REQUIRE(w.cancel(waiting.activity).status == CommandStatus::Canceled);
    // a task: kept with its loiters, flown with them
    BatchCommand item;
    item.command = Command(RouteCommand{});
    item.waypoints = points, item.loiters = loiters;
    REQUIRE(w.storeTask(v, 41, item) == Reason::None);
    const CommandResult task = w.commandTask(v, 41);
    INFO(reasonName(task.reason) << " at " << task.index);
    REQUIRE(task.accepted());
    sp = flown(w, task.activity);
    REQUIRE(sp.loiters.size() == 1);
    CHECK(sp.loiters[0].shape.orbits == 3.0);
    // a batch's route with its loiters
    const std::vector<CommandResult> batch = w.submitBatch(v, std::vector<BatchCommand>{item});
    REQUIRE(batch.size() == 1);
    REQUIRE(batch[0].accepted());
    CHECK(flown(w, batch[0].activity).loiters.size() == 1);
}

TEST_CASE("route loiters: the endurance times an orbit's laps from where it is joined - two radii out, or where a leg too short for that "
          "begins",
          "[modes]") {
    session::World w(options("route-loiters-endurance"));
    const auto v = wing(w, "c172x", 1500.0, 55.0); // (east)
    w.step(stepsFor(w, 1.0));
    NavigationSettings all = w.navigation(v);
    all.reserveFraction = 0.9999; // (every flight refused, what it needs told: 4.18)
    REQUIRE(w.setNavigation(v, all) == Reason::None);
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad, r = 1000.0, speed = 55.0;
    CommandOptions validate;
    validate.validateOnly = true;
    // what a route east needs, s: its points `east` metres on, at `loiterAt` (-1: none) a loiter point once round an orbit of 1 km
    auto needs = [&](const std::vector<double>& east, int loiterAt) {
        std::vector<Waypoint> points;
        for (const double m : east) {
            Waypoint p = at(lat0, lon0, 0.0, m);
            p.altitudeM = 1500.0, p.speed = speed, p.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
            points.push_back(p);
        }
        std::vector<RouteLoiter> loiters;
        if (loiterAt >= 0) {
            points[static_cast<std::size_t>(loiterAt)].kind = static_cast<double>(EndPointKind::LoiterPoint);
            RouteLoiter orbit = loiter(static_cast<std::uint32_t>(loiterAt), PatternKind::Orbit);
            orbit.pattern.radiusM = r, orbit.shape.orbits = 1.0;
            loiters.push_back(orbit);
        }
        const CommandResult c = w.submit(v, RouteCommand{}, points, validate, loiters);
        INFO(reasonName(c.reason) << " at " << c.index);
        REQUIRE(c.reason == Reason::InsufficientEndurance);
        return w.commandDetails(v)->endurance.requiredS;
    };
    // its own time (the route's with it, less without): along the tangent from where it is joined, `d` from its point, onto its
    // circle; once round; on round (right turns) to where its tangent runs to the next point, `on` past its point
    auto laps = [&](double d, double on) {
        return (std::sqrt(d * d - r * r) + 2.0 * kPi * r + r * (kPi - std::acos(r / on) - std::acos(r / d))) / speed;
    };
    // two radii out, its leg of 3 km longer than that; a leg of 1.45 radii, joined where it begins: where the aircraft is, the
    // point before (4.31)
    const double joined = needs({3000.0, 6000.0}, 0) - needs({3000.0, 6000.0}, -1);
    const double here = needs({1450.0, 4450.0}, 0) - needs({1450.0, 4450.0}, -1);
    const double before = needs({1000.0, 2450.0, 5450.0}, 1) - needs({1000.0, 2450.0, 5450.0}, -1);
    std::printf("route loiters, the endurance of an orbit once round: %.4f s joined two radii out (%.4f); %.4f and %.4f s where a leg of 1.45 "
                "radii begins (%.4f)\n",
                joined, laps(2.0 * r, 3000.0), here, before, laps(1450.0, 3000.0));
    CHECK(std::abs(joined - laps(2.0 * r, 3000.0)) < 0.01);
    CHECK(std::abs(here - laps(1450.0, 3000.0)) < 0.01); // (planned at its centre, as until then: 32.9 s less)
    CHECK(std::abs(before - laps(1450.0, 3000.0)) < 0.01);
}
