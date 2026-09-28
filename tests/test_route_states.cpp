// A route's planned inertial states as A-GRA's schema gives them (docs/flight-autonomy.md, 4.34; ADR-29 FA-6d2): the
// segment flown through each state's altitude and at its time - a state in a frame placed where the frame is at its time -
// what else the plan gives kept and read back; what cannot be flown refused, naming the point; a route's times taken
// together (4.33).
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

/// A state on segment `point`, `north` and `east` metres from (lat0, lon0), at its altitude and time (either kHold).
RouteState stateAt(std::uint32_t point, double lat0, double lon0, double north, double east, double altitudeM, double timeS) {
    RouteState s;
    const Waypoint p = at(lat0, lon0, north, east);
    s.point = point;
    s.latitudeRad = p.latitudeRad, s.longitudeRad = p.longitudeRad, s.altitudeM = altitudeM, s.timeS = timeS;
    return s;
}

} // namespace

TEST_CASE("route states: the segment flown through its states' altitudes, at each its time; its estimate told; read back as placed", "[modes]") {
    session::World w(options("route-states"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    // east 3 km at 55 m/s, then 12 km on at 1500 m. Between them 1650 m at 7 km, at 140 s (at 55 m/s it would be there at
    // 127 s: slowed), and 1550 m at 11 km, at 210 s (sped up); before them, at 1.5 km, what else A-GRA's plan gives there
    Waypoint a = at(lat0, lon0, 0.0, 3000.0), b = at(lat0, lon0, 0.0, 15000.0);
    a.speed = 55.0, a.altitudeM = b.altitudeM = 1500.0;
    RouteState plan = stateAt(0, lat0, lon0, 0.0, 1500.0, kHold, kHold);
    plan.groundEastMs = 55.0, plan.yawRad = 0.5 * kPi, plan.uncertaintyM = 10.0, plan.rollRateRadS = 0.0;
    const std::vector<RouteState> states = {plan, stateAt(1, lat0, lon0, 0.0, 7000.0, 1650.0, t0 + 140.0),
                                            stateAt(1, lat0, lon0, 0.0, 11000.0, 1550.0, t0 + 210.0)};
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b}, {}, {}, states);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.states.size() == 3);
    CHECK((sp.states[0].groundEastMs == 55.0 && sp.states[0].yawRad == 0.5 * kPi && sp.states[0].uncertaintyM == 10.0 && sp.states[0].rollRateRadS == 0.0));
    CHECK(isHold(sp.states[0].timeS));
    CHECK((sp.states[1].point == 1 && sp.states[1].timeS == t0 + 140.0 && sp.states[1].altitudeM == 1650.0));
    CHECK(sp.states[1].altitudeReference == sp.waypoints[1].altitudeReference); // (completed: its point's)
    CHECK(sp.states[2].latitudeRad == states[2].latitudeRad);
    // flown: when it passed each (east of where it began) and how high; its estimate as it closed on the first
    const double east[2] = {7000.0, 11000.0}, due[2] = {140.0, 210.0}, high[2] = {1650.0, 1550.0};
    double passed[2] = {kHold, kHold}, altitude[2] = {kHold, kHold}, estimate = kHold, done = kHold, highest = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 400.0) && isHold(done); ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        double north = 0.0, e = 0.0;
        offset(s, lat0, lon0, north, e);
        const double t = w.simTime() - t0;
        for (int j = 0; j < 2; ++j)
            if (isHold(passed[j]) && e >= east[j]) passed[j] = t, altitude[j] = s.altitudeMslM;
        highest = std::max(highest, s.altitudeMslM);
        ArrivalEstimate est;
        if (isHold(estimate) && e > 5000.0 && w.activityArrival(r.activity, est)) estimate = est.arrivalS - t0;
        if (!w.activity(r.activity)->live()) done = t;
    }
    std::printf("route states: a C172 passed 1650 m at 7 km due at 140 s at %.1f s, %.1f m high (its estimate %.2f s); 1550 m at 11 km due at 210 s "
                "at %.1f s, %.1f m; at most %.1f m; done at %.1f s\n",
                passed[0], altitude[0], estimate, passed[1], altitude[1], highest, done);
    for (int j = 0; j < 2; ++j) {
        INFO("state " << j + 1);
        REQUIRE(!isHold(passed[j]));
        CHECK(std::abs(passed[j] - due[j]) < 2.0); // (FA-6's acceptance: within 2 s; passed 0.2 s after each)
        CHECK(std::abs(altitude[j] - high[j]) < 25.0); // (0.1 m off)
    }
    CHECK(std::abs(estimate - 140.0) < 0.5); // (its climb's pace, no faster than it climbs it: 140.12 s)
    CHECK(highest < 1675.0);
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
}

TEST_CASE("route states: a rotorcraft through its states; one in a frame placed where the frame is at its time", "[modes]") {
    session::World w(options("route-states-rotor"));
    const auto quad = rotor(w, "iris", 15.0);
    const double t0 = w.simTime();
    const auto& q0 = *w.vehicleState(quad);
    const double lat0 = q0.latitudeRad, lon0 = q0.longitudeRad, h0 = q0.altitudeMslM;
    // a frame moving north at 2 m/s from where the IRIS is now: its origin 100 m north at 50 s
    FrameSpec moving;
    moving.origin = FrameOrigin::Moving;
    moving.latitudeRad = lat0, moving.longitudeRad = lon0, moving.altitudeMslM = h0;
    moving.northMs = 2.0, moving.timeS = t0;
    const FrameId frame = w.createFrame(moving);
    REQUIRE(frame != 0);
    // north 300 m at 5 m/s over the ground: 20 m up at the frame's origin at 50 s, 30 m up 200 m north at 75 s
    Waypoint q = at(lat0, lon0, 300.0, 0.0);
    q.speed = 5.0, q.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
    q.altitudeM = h0, q.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    RouteState inFrame;
    inFrame.point = 0, inFrame.frame = static_cast<double>(frame), inFrame.frameZM = -20.0, inFrame.timeS = t0 + 50.0;
    const std::vector<RouteState> states = {inFrame, stateAt(0, lat0, lon0, 200.0, 0.0, h0 + 30.0, t0 + 75.0)};
    const CommandResult r = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{q}, {}, {}, states);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.states.size() == 2);
    const RouteState& placed = sp.states[0];
    CHECK(std::abs((placed.latitudeRad - lat0) * kR - 100.0) < 0.5); // (where the frame is at its time)
    CHECK(std::abs(placed.longitudeRad - lon0) < 1e-9);
    CHECK(std::abs(placed.altitudeM - (h0 + 20.0)) < 1e-6);
    CHECK(placed.altitudeReference == static_cast<double>(AltitudeReference::Msl));
    CHECK((placed.frame == static_cast<double>(frame) && placed.frameZM == -20.0)); // (its frame kept)
    const double north[2] = {100.0, 200.0}, due[2] = {50.0, 75.0}, high[2] = {20.0, 30.0};
    double passed[2] = {kHold, kHold}, above[2] = {kHold, kHold}, done = kHold;
    for (unsigned k = 0; k < stepsFor(w, 150.0) && isHold(done); ++k) {
        w.step();
        const auto& s = *w.vehicleState(quad);
        double n = 0.0, e = 0.0;
        offset(s, lat0, lon0, n, e);
        const double t = w.simTime() - t0;
        for (int j = 0; j < 2; ++j)
            if (isHold(passed[j]) && n >= north[j]) passed[j] = t, above[j] = s.altitudeMslM - h0;
        if (!w.activity(r.activity)->live()) done = t;
    }
    std::printf("route states: an IRIS passed 100 m north, due at 50 s (a moving frame's origin then), at %.1f s %.1f m up (20); 200 m, due at 75 s, "
                "at %.1f s %.1f m up (30); done at %.1f s\n",
                passed[0], above[0], passed[1], above[1], done);
    for (int j = 0; j < 2; ++j) {
        INFO("state " << j + 1);
        REQUIRE(!isHold(passed[j]));
        CHECK(std::abs(passed[j] - due[j]) < 2.0);
        CHECK(std::abs(above[j] - high[j]) < 3.0);
    }
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
}

TEST_CASE("route states: refused as a point is, naming it - malformed, off its leg, steeper than it climbs, times it cannot make; unchecked, flown; "
          "at or after a loiter point, beside moving points, a time without tables, not implemented",
          "[modes]") {
    session::World w(options("route-states-refused"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    Waypoint p0 = at(lat0, lon0, 0.0, 3000.0), p1 = at(lat0, lon0, 0.0, 15000.0);
    p0.altitudeM = p1.altitudeM = 1500.0;
    const RouteState good = stateAt(1, lat0, lon0, 0.0, 9000.0, 1550.0, t0 + 200.0);
    auto answer = [&](std::vector<RouteState> states, CommandOptions o = {}, std::vector<Waypoint> points = {}) {
        if (points.empty()) points = {p0, p1};
        return w.submit(v, RouteCommand{}, points, o, {}, states);
    };
    auto refused = [&](const std::vector<RouteState>& states, Reason why, std::int16_t index, const char* what, Constraint constraint = Constraint::None,
                       CommandOptions o = {}) {
        const CommandResult r = answer(states, o);
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index << " (" << constraintName(r.constraint) << ")");
        CHECK(r.reason == why);
        CHECK(r.index == index);
        if (constraint != Constraint::None) CHECK(r.constraint == constraint);
    };
    CHECK(answer({good}).accepted());
    // malformed
    RouteState past = good;
    past.point = 2;
    refused({past}, Reason::InvalidWaypoint, 2, "past the route");
    refused({good, stateAt(0, lat0, lon0, 0.0, 1500.0, kHold, kHold)}, Reason::InvalidWaypoint, 0, "out of order");
    RouteState infinite = good;
    infinite.groundNorthMs = std::numeric_limits<double>::infinity();
    refused({infinite}, Reason::InvalidWaypoint, 1, "a field not finite");
    RouteState nowhere = good;
    nowhere.latitudeRad = nowhere.longitudeRad = kHold;
    refused({nowhere}, Reason::InvalidWaypoint, 1, "no place");
    RouteState unknownFrame = nowhere;
    unknownFrame.frame = 99.0;
    refused({unknownFrame}, Reason::InvalidWaypoint, 1, "a frame the session has not");
    RouteState above = good;
    above.altitudeReference = static_cast<double>(AltitudeReference::AboveGround);
    refused({above}, Reason::InvalidWaypoint, 1, "an altitude in another reference than its point's");
    RouteState gone = good;
    gone.timeS = t0 - 1.0;
    refused({gone}, Reason::InvalidWaypoint, 1, "a time past");
    refused({good, stateAt(1, lat0, lon0, 0.0, 11000.0, kHold, t0 + 200.0)}, Reason::InvalidWaypoint, 1, "a time not after the one before");
    Waypoint rated = p1;
    rated.climbRateMs = 2.0;
    const CommandResult climb = answer({good}, {}, {p0, rated});
    CHECK((climb.reason == Reason::InvalidWaypoint && climb.index == 1)); // (an altitude beside a climb rate: two profiles)
    // a segment its first lap does not fly: a route begun at its second point
    {
        RouteCommand second;
        second.start = 1.0;
        const CommandResult r = w.submit(v, second, std::vector<Waypoint>{p0, p1}, {}, {}, std::vector<RouteState>{stateAt(0, lat0, lon0, 0.0, 1500.0, kHold, kHold)});
        CHECK((r.reason == Reason::InvalidWaypoint && r.index == 0));
    }
    // 65: one too many
    std::vector<RouteState> many;
    for (int k = 0; k < 65; ++k) many.push_back(stateAt(1, lat0, lon0, 0.0, 3100.0 + 100.0 * k, kHold, kHold));
    refused(many, Reason::InvalidWaypoint, 1, "65 states");
    // off its leg (1 km abeam, where 1 % of the leg is 120 m); behind the one before
    refused({stateAt(1, lat0, lon0, 1000.0, 9000.0, kHold, kHold)}, Reason::InvalidWaypoint, 1, "off its leg");
    refused({stateAt(1, lat0, lon0, 0.0, 9000.0, kHold, kHold), stateAt(1, lat0, lon0, 0.0, 7000.0, kHold, kHold)}, Reason::InvalidWaypoint, 1,
            "behind the one before");
    RouteState wide = stateAt(1, lat0, lon0, 300.0, 9000.0, kHold, kHold);
    wide.uncertaintyM = 400.0;
    CHECK(answer({wide}).accepted()); // (within its own uncertainty)
    // steeper than it climbs: 500 m in the kilometre after the first point, whatever the range policy
    CommandOptions clamp;
    clamp.range = RangePolicy::Clamp;
    refused({stateAt(1, lat0, lon0, 0.0, 4000.0, 2000.0, kHold)}, Reason::PerformanceLimit, 1, "too steep", Constraint::MaxClimbRate);
    refused({stateAt(1, lat0, lon0, 0.0, 4000.0, 2000.0, kHold)}, Reason::PerformanceLimit, 1, "too steep, clamped", Constraint::MaxClimbRate, clamp);
    // times it cannot make: 4 km in 10 s; one too late for its slowest
    refused({stateAt(1, lat0, lon0, 0.0, 5000.0, kHold, t0 + 100.0), stateAt(1, lat0, lon0, 0.0, 9000.0, kHold, t0 + 110.0)}, Reason::PerformanceLimit, 1,
            "4 km in 10 s", Constraint::MaxAirspeed);
    refused({stateAt(1, lat0, lon0, 0.0, 5000.0, kHold, t0 + 1000.0)}, Reason::PerformanceLimit, 1, "too late for its slowest", Constraint::MinAirspeed);
    // unchecked: off its leg, flown (its place along it)
    CommandOptions unchecked;
    unchecked.range = RangePolicy::None;
    CHECK(answer({stateAt(1, lat0, lon0, 1000.0, 9000.0, kHold, kHold)}, unchecked).accepted());
    // not built yet: at or after a loiter point; on a route with a point in a moving frame; a time without tables
    Waypoint loiterAt = p0;
    loiterAt.kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter l;
    l.point = 0, l.pattern.durationS = 60.0;
    const CommandResult afterLoiter = w.submit(v, RouteCommand{}, std::vector<Waypoint>{loiterAt, p1}, {}, std::vector<RouteLoiter>{l}, std::vector<RouteState>{good});
    CHECK((afterLoiter.reason == Reason::NotImplemented && afterLoiter.index == 1));
    FrameSpec drifting;
    drifting.origin = FrameOrigin::Moving;
    drifting.latitudeRad = p1.latitudeRad, drifting.longitudeRad = p1.longitudeRad, drifting.altitudeMslM = 1500.0, drifting.northMs = 1.0;
    drifting.timeS = t0;
    Waypoint carried = p1;
    carried.frame = static_cast<double>(w.createFrame(drifting));
    const CommandResult moves = answer({good}, {}, {p0, carried});
    CHECK((moves.reason == Reason::NotImplemented && moves.index == 1));
    const SupportInfo* row = w.supportTable(v)->find("fsim.guidance.route/inertial_states");
    REQUIRE(row != nullptr);
    CHECK(row->support == Support::Partial);
    const auto stock = wing(w, "c172x", 1500.0, 55.0, 3);
    const auto& x0 = *w.vehicleState(stock);
    const Waypoint x1 = at(x0.latitudeRad, x0.longitudeRad, 0.0, 3000.0), x2 = at(x0.latitudeRad, x0.longitudeRad, 0.0, 15000.0);
    const CommandResult timed = w.submit(stock, RouteCommand{}, std::vector<Waypoint>{x1, x2}, {}, {},
                                         std::vector<RouteState>{stateAt(1, x0.latitudeRad, x0.longitudeRad, 0.0, 9000.0, kHold, t0 + 200.0)});
    CHECK((timed.reason == Reason::NotImplemented && timed.index == 1));
    const CommandResult untimed = w.submit(stock, RouteCommand{}, std::vector<Waypoint>{x1, x2}, {}, {},
                                           std::vector<RouteState>{stateAt(1, x0.latitudeRad, x0.longitudeRad, 0.0, 9000.0, 1550.0, kHold)});
    CHECK(untimed.accepted()); // (an altitude alone: no tables needed)
    CHECK(w.supportTable(stock)->find("fsim.guidance.route/inertial_states")->support == Support::Partial);
}

TEST_CASE("route states: an UPDATE's new waypoints come with theirs, none keeps its own; a route's times taken together", "[modes]") {
    session::World w(options("route-states-update"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    const Waypoint p0 = at(lat0, lon0, 0.0, 3000.0), p1 = at(lat0, lon0, 0.0, 15000.0);
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1}, {}, {},
                                     std::vector<RouteState>{stateAt(1, lat0, lon0, 0.0, 9000.0, 1550.0, kHold)});
    REQUIRE(r.accepted());
    Setpoint sp;
    RouteCommand ended;
    ended.end = static_cast<double>(EndBehavior::Loiter);
    REQUIRE(w.update(r.activity, ended, std::vector<Waypoint>{}).accepted()); // (options alone: its waypoints and states kept)
    REQUIRE(w.activitySetpoint(r.activity, sp));
    CHECK(sp.states.size() == 1);
    REQUIRE(w.update(r.activity, RouteCommand{}, std::vector<Waypoint>{p0, p1}).accepted()); // (new waypoints, none: none)
    REQUIRE(w.activitySetpoint(r.activity, sp));
    CHECK(sp.states.empty());
    // two windows it could make each alone, not both: after the first at 240 s, the second 5 km on by 260 s (refused
    // before FA-6d2 took them together: only its first)
    Waypoint a = at(lat0, lon0, 0.0, 3000.0), b = at(lat0, lon0, 0.0, 8000.0), c = at(lat0, lon0, 0.0, 13000.0);
    b.arrivalBeginS = t0 + 240.0, b.arrivalEndS = t0 + 245.0;
    c.arrivalBeginS = t0 + 250.0, c.arrivalEndS = t0 + 260.0;
    const CommandResult both = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b, c});
    INFO(reasonName(both.reason) << " at " << both.index);
    CHECK((both.reason == Reason::PerformanceLimit && both.index == 2 && both.constraint == Constraint::MaxAirspeed));
    c.arrivalBeginS = t0 + 330.0, c.arrivalEndS = t0 + 340.0;
    CHECK(w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b, c}).accepted());
}
