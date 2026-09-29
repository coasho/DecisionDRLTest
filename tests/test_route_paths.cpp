// A route's paths and links as A-GRA's schema gives them (docs/flight-autonomy.md, 4.36; ADR-29 FA-6e1): paths of
// waypoints with ids and types, a point's next - into another path, back round a loop, the route's end - flown in that
// order, every point named as given; what does not make one refused, naming the point.
#include "control/Route.h"
#include "fsim/ControlStack.h"
#include "fsim/GuidanceModes.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
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

/// Three paths: A to its second point, linked on into B, whose four points go round and round; C, linked from nowhere.
/// `scale` metres a unit: A at (0, 1) and (0, 2), B at (1, 3), (2, 3), (2, 4), (1, 4), C at (-1, 1) and (-1, 2).
std::vector<Waypoint> threePaths(double lat0, double lon0, double scale, std::vector<RoutePath>& paths) {
    const double ne[8][2] = {{0, 1}, {0, 2}, {1, 3}, {2, 3}, {2, 4}, {1, 4}, {-1, 1}, {-1, 2}};
    std::vector<Waypoint> points;
    for (int i = 0; i < 8; ++i) {
        Waypoint p = at(lat0, lon0, ne[i][0] * scale, ne[i][1] * scale);
        p.id = 100 + static_cast<std::uint64_t>(i);
        points.push_back(p);
    }
    points[1].next = 2.0; // (A's last on into B)
    points[5].next = 2.0; // (B round again)
    paths = {RoutePath{10, static_cast<double>(PathType::Primary), 0, 2}, RoutePath{20, static_cast<double>(PathType::Ingress), 2, 4},
             RoutePath{30, static_cast<double>(PathType::Alternate), 6, 2}};
    return points;
}

/// The points a route flew to, in turn (its progress's segment as it changed), and how near it came to each point given.
struct Flown {
    std::vector<std::uint32_t> order;
    std::vector<double> nearest;
    std::uint32_t laps = 0;
};

void watch(session::World& w, std::uint32_t v, ActivityId a, const std::vector<Waypoint>& points, Flown& f) {
    const ActivityRecord& r = *w.activity(a);
    if (!r.live()) return;
    if (f.order.empty() || f.order.back() != r.progress.segment) f.order.push_back(r.progress.segment);
    f.laps = r.progress.laps;
    const auto& s = *w.vehicleState(v);
    if (f.nearest.empty()) f.nearest.assign(points.size(), 1e30);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double north = (points[i].latitudeRad - s.latitudeRad) * kR, east = (points[i].longitudeRad - s.longitudeRad) * kR * std::cos(s.latitudeRad);
        f.nearest[i] = std::min(f.nearest[i], std::hypot(north, east));
    }
}

std::string list(const std::vector<std::uint32_t>& v) {
    std::string s;
    for (std::size_t i = 0; i < v.size() && i < 20; ++i) s += (i ? " " : "") + std::to_string(v[i]);
    return s;
}

} // namespace

TEST_CASE("route paths: along its links - into another path, round a loop - its points named as given; a path linked from nowhere not flown",
          "[modes]") {
    session::World w(options("route-paths"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    const auto quad = rotor(w, "iris", 15.0, 6);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    const std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, {}, {}, paths);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    const auto& q0 = *w.vehicleState(quad);
    std::vector<RoutePath> qpaths;
    std::vector<Waypoint> qpoints = threePaths(q0.latitudeRad, q0.longitudeRad, 20.0, qpaths);
    for (Waypoint& p : qpoints) p.speed = 3.0, p.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
    const CommandResult rq = w.submit(quad, RouteCommand{}, qpoints, {}, {}, {}, qpaths);
    REQUIRE(rq.accepted());
    // read back as given: its points in their order, their links, its paths
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.waypoints.size() == 8);
    CHECK((sp.waypoints[5].next == 2.0 && sp.waypoints[6].id == 106 && sp.paths.size() == 3 && sp.paths[1].id == 20));
    CHECK(sp.paths[1].type == static_cast<double>(PathType::Ingress));
    // its end points in its flight order, round the loop
    const std::vector<EndPoint> ahead = w.endPoints(r.activity, 8);
    std::vector<std::uint32_t> next;
    for (const EndPoint& e : ahead) next.push_back(static_cast<std::uint32_t>(e.index));
    CHECK(list(next) == "0 1 2 3 4 5 2 3");
    Flown f, fq;
    for (unsigned k = 0; k < stepsFor(w, 900.0); ++k) {
        w.step();
        watch(w, v, r.activity, points, f);
        watch(w, quad, rq.activity, qpoints, fq);
    }
    std::printf("route paths: a C172 flew %s (%u laps round B), nearest A %.0f %.0f, B %.0f %.0f %.0f %.0f, C %.0f %.0f m; an IRIS flew %s (%u laps), "
                "nearest B %.2f %.2f %.2f %.2f, C %.1f %.1f m\n",
                list(f.order).c_str(), f.laps, f.nearest[0], f.nearest[1], f.nearest[2], f.nearest[3], f.nearest[4], f.nearest[5], f.nearest[6], f.nearest[7],
                list(fq.order).c_str(), fq.laps, fq.nearest[2], fq.nearest[3], fq.nearest[4], fq.nearest[5], fq.nearest[6], fq.nearest[7]);
    REQUIRE(f.order.size() >= 11);
    CHECK(list(std::vector<std::uint32_t>(f.order.begin(), f.order.begin() + 11)) == "0 1 2 3 4 5 2 3 4 5 2");
    CHECK(f.laps >= 1);
    for (int i = 0; i < 6; ++i) CHECK(f.nearest[static_cast<std::size_t>(i)] < 400.0); // (a fly-by's arc cuts its corner)
    CHECK(std::min(f.nearest[6], f.nearest[7]) > 2000.0);                                // (C never flown)
    REQUIRE(fq.order.size() >= 11);
    CHECK(list(std::vector<std::uint32_t>(fq.order.begin(), fq.order.begin() + 11)) == "0 1 2 3 4 5 2 3 4 5 2");
    for (int i = 2; i < 6; ++i) CHECK(fq.nearest[static_cast<std::size_t>(i)] < 5.0);
    CHECK(std::min(fq.nearest[6], fq.nearest[7]) > 15.0);
    CHECK(w.activity(r.activity)->live()); // (round and round: no end)
}

TEST_CASE("route paths: refused as a point is, naming it - paths that do not tile the points, a type that is none, one id twice, a next "
          "that is none, round one point; a link to the route's end ends it there",
          "[modes]") {
    session::World w(options("route-paths-refused"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    const std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    auto refused = [&](std::vector<Waypoint> pts, std::vector<RoutePath> ps, std::int16_t index, const char* what) {
        const CommandResult r = w.submit(v, RouteCommand{}, pts, {}, {}, {}, ps);
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == Reason::InvalidWaypoint);
        CHECK(r.index == index);
    };
    auto pathWith = [&](std::size_t k, auto change) {
        std::vector<RoutePath> changed = paths;
        change(changed.at(k));
        return changed;
    };
    auto pointWith = [&](std::size_t i, double next) {
        std::vector<Waypoint> changed = points;
        changed.at(i).next = next;
        return changed;
    };
    refused(points, pathWith(1, [](RoutePath& r) { r.first = 3, r.count = 3; }), 2, "a point in no path");
    refused(points, pathWith(2, [](RoutePath& r) { r.count = 0; }), 6, "a path of none");
    refused(points, pathWith(1, [](RoutePath& r) { r.type = 20.0; }), 2, "a type that is none");
    refused(points, pathWith(2, [](RoutePath& r) { r.id = 10; }), 6, "one id twice");
    refused(pointWith(3, 2.5), paths, 3, "a next that is not a point's");
    refused(pointWith(4, 8.0), paths, 4, "a next past the route");
    refused(pointWith(5, 5.0), paths, 5, "round one point");
    // a link to the end: the route ends there, the points after it in its path not flown
    const CommandResult r = w.submit(v, RouteCommand{}, pointWith(3, -1.0), {}, {}, {}, paths);
    REQUIRE(r.accepted());
    const std::vector<EndPoint> ahead = w.endPoints(r.activity, 8);
    std::vector<std::uint32_t> next;
    for (const EndPoint& e : ahead) next.push_back(static_cast<std::uint32_t>(e.index));
    CHECK(list(next) == "0 1 2 3");
    // links without paths: the points as they are, each on to the next unless it says otherwise
    std::vector<Waypoint> jumps = pointWith(1, 4.0); // (0 1 4 5, then round: 5 on to 2)
    jumps.resize(6);
    const CommandResult rj = w.submit(v, RouteCommand{}, jumps);
    REQUIRE(rj.accepted());
    next.clear();
    for (const EndPoint& e : w.endPoints(rj.activity, 8)) next.push_back(static_cast<std::uint32_t>(e.index));
    CHECK(list(next) == "0 1 4 5 2 3 4 5");
    // a route that repeats, begun past its first point: at its end back to its first, as an unlinked one (4.36, as FA-6e2
    // made it: FA-6e1's went back to where it began)
    std::vector<Waypoint> fromTwo(points.begin(), points.begin() + 6);
    fromTwo.at(5).next = kHold; // (A on into B; B's last the end: back to A)
    RouteCommand again;
    again.start = 2.0, again.repeat = 1.0;
    const std::vector<RoutePath> two = {RoutePath{1, kHold, 0, 2}, RoutePath{2, kHold, 2, 4}};
    const CommandResult rr = w.submit(v, again, fromTwo, {}, {}, {}, two);
    REQUIRE(rr.accepted());
    next.clear();
    for (const EndPoint& e : w.endPoints(rr.activity, 8)) next.push_back(static_cast<std::uint32_t>(e.index));
    CHECK(list(next) == "2 3 4 5 0 1 2 3");
    const SupportInfo* row = w.supportTable(v)->find("fsim.guidance.route/paths");
    REQUIRE(row != nullptr);
    CHECK(row->support == Support::Supported);
    CHECK(w.supportTable(v)->find("fsim.guidance.route/next_segment")->support == Support::Supported); // (a start turn looped back to too: FA-6g1)
}

TEST_CASE("route paths: disabled and enabled, it resumes at the point it flew to along its links - its planned states beyond it in "
          "that order kept",
          "[modes]") {
    session::World w(options("route-paths-resumed"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    points[1].next = 4.0; // (0 1 4 5, then 5 on to 2: 2 3, and round from 4)
    points.resize(6);
    // a state halfway along the leg into 2 (from 5) and one along the leg into 4 (from 1)
    auto halfway = [&](std::uint32_t k, std::uint32_t from) {
        RouteState st;
        st.point = k;
        st.latitudeRad = 0.5 * (points[k].latitudeRad + points[from].latitudeRad);
        st.longitudeRad = 0.5 * (points[k].longitudeRad + points[from].longitudeRad);
        return st;
    };
    const std::vector<RouteState> states = {halfway(4, 1), halfway(2, 5)}; // (in its flight order)
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, {}, states);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    for (unsigned k = 0; k < stepsFor(w, 600.0) && w.activity(r.activity)->progress.segment != 5; ++k) w.step();
    REQUIRE(w.activity(r.activity)->progress.segment == 5);
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Disable).accepted());
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Enable).accepted());
    w.step();
    const ActivityRecord& a = *w.activity(r.activity);
    CHECK(a.state == ActivityState::Active);
    CHECK(a.progress.segment == 5);
    std::vector<std::uint32_t> next;
    for (const EndPoint& e : w.endPoints(r.activity, 8)) next.push_back(static_cast<std::uint32_t>(e.index));
    CHECK(list(next) == "5 2 3 4 5 2 3 4");
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.states.size() == 1); // (the one into 2, ahead; the one into 4 passed - though 4 comes after 2 as given)
    CHECK(sp.states[0].point == 2);
    // reset as it flies: over from its first point, along its links from there, with the state it flew past
    auto overAgain = [&](const char* how) {
        INFO(how);
        for (int k = 0; k < 20 && w.activity(r.activity)->state != ActivityState::Active; ++k) w.step();
        CHECK(w.activity(r.activity)->state == ActivityState::Active);
        CHECK(w.activity(r.activity)->progress.segment == 0);
        next.clear();
        for (const EndPoint& e : w.endPoints(r.activity, 8)) next.push_back(static_cast<std::uint32_t>(e.index));
        CHECK(list(next) == "0 1 4 5 2 3 4 5");
        Setpoint again;
        REQUIRE(w.activitySetpoint(r.activity, again));
        REQUIRE(again.states.size() == 2);
        CHECK((again.states[0].point == 4 && again.states[1].point == 2));
    };
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Reset).accepted());
    overAgain("reset as it flies");
    // and reset as it waits, disabled past the leg into 4: the same
    for (unsigned k = 0; k < stepsFor(w, 400.0) && w.activity(r.activity)->progress.segment != 5; ++k) w.step();
    REQUIRE(w.activity(r.activity)->progress.segment == 5);
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Disable).accepted());
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Reset).accepted());
    REQUIRE(w.activityCommand(Source::Policy, r.activity, ActivityCommand::Enable).accepted());
    overAgain("reset as it waits");
}

TEST_CASE("route paths: a stack on its own flies them along its links too; links it cannot fly leave it nothing, and it fails", "[modes]") {
    sim::VehicleState s;
    s.latitudeRad = 37.6 * 3.14159265358979323846 / 180.0, s.longitudeRad = -122.4 * 3.14159265358979323846 / 180.0;
    s.altitudeMslM = s.altitudeAglM = 1500.0;
    s.airspeedTrueMs = s.velocityBodyMs[0] = s.velocityNedMs[1] = 60.0; // (east)
    s.eulerRad[2] = 0.5 * 3.14159265358979323846;
    s.loadFactor = 1.0;
    Rng rng{1};
    sim::ControlInputs out;
    ControlContext ctx{1, s, s, 1.0 / 120.0, nullptr, &rng};
    std::vector<RoutePath> paths;
    std::vector<Waypoint> points = threePaths(s.latitudeRad, s.longitudeRad, 3000.0, paths);
    points[1].next = -1.0; // (A, and the route ends there: 6 km to go, not the 20 its points make in their order)
    ControlStack stack;
    stack.command(RouteCommand{}, points, {}, {}, paths);
    for (int i = 0; i < 10; ++i) stack.update(ctx, out);
    ActivityProgress p;
    REQUIRE(stack.progress(0, p));
    INFO("to go " << p.distanceToGoM << " m, segment " << p.segment << " of " << p.segments);
    CHECK(p.segments == 8);
    CHECK(p.segment == 0);
    CHECK(p.distanceToGoM < 6500.0);
    REQUIRE(stack.behavior() != nullptr);
    CHECK(stack.behavior()->failure() == Reason::None);
    points[1].next = 1.0; // (round one point)
    stack.command(RouteCommand{}, points, {}, {}, paths);
    for (int i = 0; i < 10; ++i) stack.update(ctx, out);
    REQUIRE(stack.behavior() != nullptr);
    CHECK(stack.behavior()->failure() == Reason::BehaviorFailed);
}

TEST_CASE("route paths: a path it does not fly held as given - its points checked as points, a loiter there kept, not flown; the end of a "
          "path at its last point, where it links on too; a start turn looped back to, its course left out, its later laps' arc checked too",
          "[modes]") {
    session::World w(options("route-paths-held"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    // C's last a loiter point with its hold beside it (kept as given: not completed, never flown); A's last and B's (which
    // links back round) the ends of their paths
    points[7].kind = static_cast<double>(EndPointKind::LoiterPoint);
    points[1].waypointType = points[5].waypointType = static_cast<double>(WaypointType::EndOfPath);
    RouteLoiter hold;
    hold.point = 7;
    hold.pattern.pattern = static_cast<double>(PatternKind::Orbit);
    hold.pattern.durationS = 120.0;
    const std::vector<RouteLoiter> holds = {hold};
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, holds, {}, paths);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.loiters.size() == 1);
    CHECK(sp.loiters[0].point == 7);
    CHECK(sp.loiters[0].pattern.durationS == 120.0);
    CHECK(isHold(sp.loiters[0].pattern.radiusM)); // (not completed)
    for (unsigned k = 0; k < stepsFor(w, 120.0); ++k) w.step();
    CHECK(w.activity(r.activity)->live());
    CHECK(w.activity(r.activity)->progress.segment < 6); // (A, then B)
    auto refused = [&](const std::vector<Waypoint>& pts, const std::vector<RouteLoiter>& ls, const std::vector<RouteState>& ss, Reason why,
                       std::int16_t index, const char* what) {
        const CommandResult x = w.submit(v, RouteCommand{}, pts, {}, ls, ss, paths);
        INFO(what << ": " << reasonName(x.reason) << " at " << x.index);
        CHECK(x.reason == why);
        CHECK(x.index == index);
    };
    // its points checked as points: C's loiter point without its loiter, as one flown would be
    refused(points, {}, {}, Reason::InvalidWaypoint, 7, "a loiter point with none, on C");
    // a planned state on a segment its first lap does not fly (4.34): C's
    RouteState st;
    st.point = 6;
    st.latitudeRad = points[6].latitudeRad, st.longitudeRad = points[6].longitudeRad;
    refused(points, holds, {st}, Reason::InvalidWaypoint, 6, "a state on C");
    // the end of a path where its path goes on: none
    std::vector<Waypoint> early = points;
    early[3].waypointType = static_cast<double>(WaypointType::EndOfPath);
    refused(early, holds, {}, Reason::InvalidWaypoint, 3, "the end of a path in B's middle");
    // a start turn where the links loop back (B's first), its course left out: its first lap's arc (on from A, north-east)
    // sweeps 90 degrees, its later laps' (on from B's last, west) 180 - refused as any start's arc beyond 170 degrees is,
    // naming the point it reaches (FA-6g1; until then, not implemented); its course given, as any start turn
    std::vector<Waypoint> arc = points;
    arc[2].turn = static_cast<double>(TurnType::StartTurn);
    refused(arc, holds, {}, Reason::InvalidWaypoint, 3, "a start turn looped back to, its later laps' arc 180 degrees round");
    arc[2].turn = static_cast<double>(TurnType::StartTurn), arc[2].courseRad = 0.0, arc[2].turnRadiusM = 50.0;
    const CommandResult given = w.submit(v, RouteCommand{}, arc, {}, holds, {}, paths);
    CHECK(given.reason != Reason::NotImplemented);
}

TEST_CASE("route paths: a start turn looped back to, its course left out, begins each lap's arc on the course that lap comes to it on - "
          "the first lap's from the path before, a later lap's from the loop's last point (FA-6g1)",
          "[modes]") {
    session::World w(options("route-paths-looped-arc"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    // A, 3 km east, on into B: its first 6 km east, a start turn whose arc runs to (2, 8) km, then (2, 12), (-3, 12) and
    // (-3, 6), and back to its first from the south. The first lap comes to it heading east: a left arc of 2 km, 90 degrees
    // round, north at its end, then a right turn onto the leg east; a later lap comes heading north: a right arc, east at its
    // end, straight on east.
    const double ne[6][2] = {{0, 3}, {0, 6}, {2, 8}, {2, 12}, {-3, 12}, {-3, 6}};
    std::vector<Waypoint> points;
    for (const auto& q : ne) points.push_back(at(s0.latitudeRad, s0.longitudeRad, q[0] * 1000.0, q[1] * 1000.0));
    points[1].turn = static_cast<double>(TurnType::StartTurn);
    points[0].next = 1.0; // (A's last on into B)
    points[5].next = 1.0; // (B round again)
    const std::vector<RoutePath> paths = {RoutePath{1, kHold, 0, 1}, RoutePath{2, kHold, 1, 5}};
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, {}, {}, paths);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    // each arc's middle: the first lap's round (2, 6) km, a later lap's round (0, 8)
    const double h = 1000.0 * std::sqrt(0.5) * 2.0;
    const Waypoint firstMiddle = at(s0.latitudeRad, s0.longitudeRad, 2000.0 - h, 6000.0 + h), laterMiddle = at(s0.latitudeRad, s0.longitudeRad, h, 8000.0 - h);
    // on the arc (the segment to point 2), each lap round: the most it is off it (on its first 60 %), the course it begins it
    // on, how near it comes to each lap's middle
    double off[3] = {0.0, 0.0, 0.0}, begun[3] = {kHold, kHold, kHold}, nearFirst[3] = {1e30, 1e30, 1e30}, nearLater[3] = {1e30, 1e30, 1e30};
    auto distance = [&](const Waypoint& q, const sim::VehicleState& s) {
        return std::hypot((q.latitudeRad - s.latitudeRad) * kR, (q.longitudeRad - s.longitudeRad) * kR * std::cos(s.latitudeRad));
    };
    for (unsigned k = 0; k < stepsFor(w, 1000.0); ++k) {
        w.step();
        const ActivityProgress& g = w.activity(r.activity)->progress;
        if (g.segment != 2 || g.laps > 2) continue;
        const auto& s = *w.vehicleState(v);
        const std::size_t lap = g.laps;
        if (g.segmentPercent < 60.0) off[lap] = std::max(off[lap], std::abs(g.crossTrackM)); // (its arc: the first lap's turn at its end left out)
        if (isHold(begun[lap])) begun[lap] = std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) * 180.0 / 3.14159265358979323846;
        nearFirst[lap] = std::min(nearFirst[lap], distance(firstMiddle, s)), nearLater[lap] = std::min(nearLater[lap], distance(laterMiddle, s));
    }
    std::printf("route paths: a start turn looped back to - its arc begun heading %.1f, %.1f, %.1f deg on laps 0, 1, 2; the most off it %.1f, %.1f, "
                "%.1f m; the first lap's middle passed at %.0f m, a later lap's at %.0f and %.0f m\n",
                begun[0], begun[1], begun[2], off[0], off[1], off[2], nearFirst[0], nearLater[1], nearLater[2]);
    CHECK(std::abs(begun[0] - 90.0) < 5.0); // (east, from A)
    CHECK(std::abs(begun[1]) < 5.0);        // (north, from B's last)
    CHECK(std::abs(begun[2]) < 5.0);
    for (const double m : off) CHECK(m < 20.0); // (on each lap's arc from its start: none begun across it)
    CHECK(nearFirst[0] < 40.0);
    CHECK(nearLater[1] < 40.0);
    CHECK(nearLater[2] < 40.0);
    CHECK(nearFirst[1] > 1000.0); // (a later lap flies its own arc, not the first's)
}
