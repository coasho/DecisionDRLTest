// A route's conditional branches as A-GRA's schema gives them (docs/flight-autonomy.md, 4.37; ADR-29 FA-6e2): at a point,
// the next taken when its conditions hold - the times the point has been come to, the altitude, the time, the operator's
// input - the route planned again from there; what does not make one refused, naming its point.
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

Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint p;
    p.latitudeRad = lat0 + north / kR, p.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return p;
}

/// A, two points east, on into B; B, four points round a square (its last on to its first); C, two points south of B, the
/// way out. `scale` metres a unit: A (0, 1), (0, 2); B (1, 3), (2, 3), (2, 4), (1, 4); C (-1, 4), (-1, 5).
std::vector<Waypoint> threePaths(double lat0, double lon0, double scale, std::vector<RoutePath>& paths) {
    const double ne[8][2] = {{0, 1}, {0, 2}, {1, 3}, {2, 3}, {2, 4}, {1, 4}, {-1, 4}, {-1, 5}};
    std::vector<Waypoint> points;
    for (int i = 0; i < 8; ++i) points.push_back(at(lat0, lon0, ne[i][0] * scale, ne[i][1] * scale));
    points[1].next = 2.0, points[5].next = 2.0;
    paths = {RoutePath{10, static_cast<double>(PathType::Primary), 0, 2}, RoutePath{20, static_cast<double>(PathType::Primary), 2, 4},
             RoutePath{30, static_cast<double>(PathType::Egress), 6, 2}};
    return points;
}

RouteBranch branchAt(std::uint32_t point, double next) {
    RouteBranch b;
    b.point = point, b.next = next;
    return b;
}

std::string list(const std::vector<std::uint32_t>& v) {
    std::string s;
    for (std::size_t i = 0; i < v.size() && i < 24; ++i) s += (i ? " " : "") + std::to_string(v[i]);
    return s;
}

/// The points it flew to, in turn, as its progress said.
struct Flown {
    std::vector<std::uint32_t> order;
    void watch(session::World& w, ActivityId a) {
        const ActivityRecord& r = *w.activity(a);
        if (r.live() && (order.empty() || order.back() != r.progress.segment)) order.push_back(r.progress.segment);
    }
};

std::string ahead(session::World& w, ActivityId a, std::size_t n) {
    std::vector<std::uint32_t> next;
    for (const EndPoint& e : w.endPoints(a, n)) next.push_back(static_cast<std::uint32_t>(e.index));
    return list(next);
}

} // namespace

TEST_CASE("route branches: taken as their conditions say - round B twice then out by C, the first branch that holds, a branch "
          "to the route's end; its end points as it will fly",
          "[modes]") {
    session::World w(options("route-branches"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    const auto u = wing(w, "c172", 1500.0, 55.0, 1);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    const std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    // at B's last: out by C once it has come there twice; at A's last, an altitude it is not at, then one it is
    RouteBranch out = branchAt(5, 6.0);
    out.captures = 2.0, out.capturesComparison = static_cast<double>(Comparison::GreaterEqual);
    RouteBranch high = branchAt(1, 6.0), low = branchAt(1, 2.0);
    high.altitudeMinM = 2500.0, low.altitudeMinM = 1000.0, low.altitudeMaxM = 2000.0; // (the first that holds: straight on)
    const std::vector<RouteBranch> branches = {high, low, out};
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, {}, {}, paths, branches);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    // a branch to the route's end: the second comes to B's second point and ends there, its end its end behaviour's
    const auto& u0 = *w.vehicleState(u);
    std::vector<RoutePath> upaths;
    const std::vector<Waypoint> upoints = threePaths(u0.latitudeRad, u0.longitudeRad, 3000.0, upaths);
    const std::vector<RouteBranch> end = {branchAt(3, -1.0)};
    const CommandResult ru = w.submit(u, RouteCommand{}, upoints, {}, {}, {}, upaths, end);
    REQUIRE(ru.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.branches.size() == 3);
    CHECK((sp.branches[2].point == 5 && sp.branches[2].next == 6.0 && sp.branches[2].captures == 2.0));
    CHECK(ahead(w, r.activity, 8) == "0 1 2 3 4 5 2 3"); // (as the links say: what a branch will do is not known yet)
    Flown f, fu;
    std::string afterOut;
    for (unsigned k = 0; k < stepsFor(w, 1200.0) && w.activity(r.activity)->live(); ++k) {
        w.step();
        f.watch(w, r.activity);
        fu.watch(w, ru.activity);
        if (afterOut.empty() && f.order.size() >= 11 && f.order.back() == 6) afterOut = ahead(w, r.activity, 8);
    }
    std::printf("route branches: a C172 flew %s, then %s; one branching to the end flew %s, %s\n", list(f.order).c_str(),
                activityStateName(w.activity(r.activity)->state), list(fu.order).c_str(), activityStateName(w.activity(ru.activity)->state));
    CHECK(list(f.order) == "0 1 2 3 4 5 2 3 4 5 6 7");
    CHECK(afterOut == "6 7");
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    CHECK(list(fu.order) == "0 1 2 3");
    CHECK(w.activity(ru.activity)->state == ActivityState::Completed);
}

TEST_CASE("route branches: the operator's input - held until commanded, then taken as the point is next come to; a time window; "
          "a loiter point's as its loiter ends",
          "[modes]") {
    session::World w(options("route-branches-operator"));
    const auto quad = rotor(w, "iris", 15.0, 6);
    const auto late = rotor(w, "iris", 15.0, 7);
    const auto hold = rotor(w, "iris", 15.0, 8);
    const auto kept = rotor(w, "iris", 15.0, 9);
    const auto fresh = rotor(w, "iris", 15.0, 10);
    auto routeFor = [&](std::uint32_t id, std::vector<RoutePath>& paths) {
        const auto& s = *w.vehicleState(id);
        std::vector<Waypoint> points = threePaths(s.latitudeRad, s.longitudeRad, 20.0, paths);
        for (Waypoint& p : points) p.speed = 3.0, p.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
        return points;
    };
    // out by C at B's second point, once the operator commands it
    std::vector<RoutePath> paths, lpaths, hpaths;
    const std::vector<Waypoint> points = routeFor(quad, paths);
    RouteBranch asked = branchAt(3, 6.0);
    asked.operatorInput = 1.0;
    const CommandResult r = w.submit(quad, RouteCommand{}, points, {}, {}, {}, paths, std::vector<RouteBranch>{asked});
    REQUIRE(r.accepted());
    // out by C at B's last, in a time window a minute and a half on
    const std::vector<Waypoint> lpoints = routeFor(late, lpaths);
    RouteBranch timed = branchAt(5, 6.0);
    timed.timeBeginS = w.simTime() + 90.0, timed.timeEndS = w.simTime() + 400.0;
    const CommandResult rl = w.submit(late, RouteCommand{}, lpoints, {}, {}, {}, lpaths, std::vector<RouteBranch>{timed});
    REQUIRE(rl.accepted());
    // A's last a loiter point - an orbit of 20 s - and out by C as it ends
    std::vector<Waypoint> hpoints = routeFor(hold, hpaths);
    hpoints[1].kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter orbit;
    orbit.point = 1;
    orbit.pattern.pattern = static_cast<double>(PatternKind::Hover);
    orbit.pattern.durationS = 20.0;
    const CommandResult rh =
        w.submit(hold, RouteCommand{}, hpoints, {}, std::vector<RouteLoiter>{orbit}, {}, hpaths, std::vector<RouteBranch>{branchAt(1, 6.0)});
    INFO(reasonName(rh.reason) << " at " << rh.index);
    REQUIRE(rh.accepted());
    // commanded as it waits (disabled), then flown: out at the first time it comes to the point; commanded, then updated
    // (flown afresh): the input forgotten, round B
    std::vector<RoutePath> kpaths, fpaths;
    const std::vector<Waypoint> kpoints = routeFor(kept, kpaths), fpoints = routeFor(fresh, fpaths);
    const CommandResult rk = w.submit(kept, RouteCommand{}, kpoints, {}, {}, {}, kpaths, std::vector<RouteBranch>{asked});
    const CommandResult rf = w.submit(fresh, RouteCommand{}, fpoints, {}, {}, {}, fpaths, std::vector<RouteBranch>{asked});
    REQUIRE((rk.accepted() && rf.accepted()));
    REQUIRE(w.activityCommand(Source::Policy, rk.activity, ActivityCommand::Disable).accepted());
    REQUIRE(w.commandBranch(rk.activity, 0).accepted());
    REQUIRE(w.activityCommand(Source::Policy, rk.activity, ActivityCommand::Enable).accepted());
    REQUIRE(w.commandBranch(rf.activity, 0).accepted());
    RouteCommand options; // (its options as they were: flown afresh from its start)
    options.projection = options.repeat = options.end = options.start = kHold;
    REQUIRE(w.update(rf.activity, options, {}).accepted());
    Flown f, fl, fh, fk, ff;
    bool commanded = false;
    double firstLapEnd = -1.0;
    for (unsigned k = 0; k < stepsFor(w, 500.0); ++k) {
        w.step();
        f.watch(w, r.activity), fl.watch(w, rl.activity), fh.watch(w, rh.activity), fk.watch(w, rk.activity), ff.watch(w, rf.activity);
        // commanded once round B, the aircraft past its second point again
        if (!commanded && f.order.size() >= 8 && f.order.back() == 4) {
            REQUIRE(w.commandBranch(r.activity, 0).accepted());
            commanded = true;
            firstLapEnd = w.simTime();
        }
    }
    std::printf("route branches: an IRIS commanded out at %.0f s flew %s; one in a window flew %s; one out as its hover ended flew %s\n",
                firstLapEnd, list(f.order).c_str(), list(fl.order).c_str(), list(fh.order).c_str());
    REQUIRE(commanded);
    CHECK(list(f.order).rfind("0 1 2 3 4 5 2 3 4 5 2 3 6 7", 0) == 0); // (held twice at 3, the operator's input not given; then out)
    // the window: its first comes to B's last before it opens, round again, then out
    REQUIRE(fl.order.size() >= 6);
    CHECK(list(fl.order).find("5 6 7") != std::string::npos);
    CHECK(list(fl.order).rfind("0 1 2 3 4 5 2", 0) == 0);
    CHECK(list(fh.order).rfind("0 1 6 7", 0) == 0);
    std::printf("route branches: one commanded as it waited flew %s; one updated after it was commanded flew %s\n", list(fk.order).c_str(),
                list(ff.order).c_str());
    CHECK(list(fk.order).rfind("0 1 2 3 6 7", 0) == 0);
    CHECK(list(ff.order).rfind("0 1 2 3 4 5 2 3 4 5 2", 0) == 0);
}

TEST_CASE("route branches: refused as a point is, naming its point - one that is none, a condition malformed, 17; not built, "
          "an endurance or a contingency; the operator's input refused where it takes none",
          "[modes]") {
    session::World w(options("route-branches-refused"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const auto& s0 = *w.vehicleState(v);
    std::vector<RoutePath> paths;
    std::vector<Waypoint> points = threePaths(s0.latitudeRad, s0.longitudeRad, 3000.0, paths);
    auto answer = [&](std::vector<RouteBranch> bs, std::vector<Waypoint> pts = {}) {
        return w.submit(v, RouteCommand{}, pts.empty() ? points : pts, {}, {}, {}, paths, bs);
    };
    auto refused = [&](const RouteBranch& b, Reason why, std::int16_t index, const char* what) {
        const CommandResult r = answer({b});
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == index);
    };
    refused(branchAt(8, 6.0), Reason::InvalidWaypoint, 8, "at a point it has not");
    refused(branchAt(5, 8.0), Reason::InvalidWaypoint, 5, "a next past the route");
    refused(branchAt(5, 5.0), Reason::InvalidWaypoint, 5, "on to its own point");
    refused(branchAt(5, 2.5), Reason::InvalidWaypoint, 5, "a next that is not a point's");
    RouteBranch b = branchAt(5, 6.0);
    b.altitudeMinM = 2000.0, b.altitudeMaxM = 1000.0;
    refused(b, Reason::InvalidWaypoint, 5, "a range upside down");
    b = branchAt(5, 6.0), b.altitudeReference = 0.0;
    refused(b, Reason::InvalidWaypoint, 5, "a reference with no range");
    b = branchAt(5, 6.0), b.timeBeginS = 100.0, b.timeEndS = 50.0;
    refused(b, Reason::InvalidWaypoint, 5, "a window upside down");
    b = branchAt(5, 6.0), b.captures = 2.0;
    refused(b, Reason::InvalidWaypoint, 5, "a count with no comparison");
    b = branchAt(5, 6.0), b.captures = 1.5, b.capturesComparison = 0.0;
    refused(b, Reason::InvalidWaypoint, 5, "a count not whole");
    b = branchAt(5, 6.0), b.operatorInput = 2.0;
    refused(b, Reason::InvalidWaypoint, 5, "an operator input that is not one");
    b = branchAt(5, 6.0), b.contingency = 7.0;
    refused(b, Reason::InvalidWaypoint, 5, "a contingency that is none");
    b = branchAt(5, 6.0), b.percent = 20.0;
    refused(b, Reason::InvalidWaypoint, 5, "an endurance with no comparison");
    std::vector<Waypoint> round = points; // (its next point on to itself: round one point once the branch is taken)
    round.at(6).next = 6.0;
    const CommandResult r1 = answer({branchAt(5, 6.0)}, round);
    CHECK((r1.reason == Reason::InvalidWaypoint && r1.index == 5));
    const CommandResult many = answer(std::vector<RouteBranch>(17, branchAt(5, 6.0)));
    CHECK((many.reason == Reason::InvalidWaypoint && many.index == 5));
    // not built yet: its row, partial
    b = branchAt(5, 6.0), b.percent = 20.0, b.enduranceComparison = static_cast<double>(Comparison::LessEqual);
    refused(b, Reason::NotImplemented, 5, "an endurance");
    b = branchAt(5, 6.0), b.contingency = static_cast<double>(Contingency::FlightCritical);
    refused(b, Reason::NotImplemented, 5, "a contingency");
    const SupportInfo* row = w.supportTable(v)->find("fsim.guidance.route/conditional_segment");
    REQUIRE(row != nullptr);
    CHECK(row->support == Support::Partial);
    // the operator's input: to a branch that takes it; one it has not, or one that takes none, invalid_parameter naming it
    RouteBranch asked = branchAt(5, 6.0);
    asked.operatorInput = 1.0;
    const CommandResult r = answer({branchAt(1, 2.0), asked});
    REQUIRE(r.accepted());
    CHECK(w.commandBranch(r.activity, 1).accepted());
    CHECK(w.commandBranch(r.activity, 1, false).accepted());
    const CommandResult none = w.commandBranch(r.activity, 0);
    CHECK((none.reason == Reason::InvalidParameter && none.index == 0));
    const CommandResult missing = w.commandBranch(r.activity, 2);
    CHECK((missing.reason == Reason::InvalidParameter && missing.index == 2));
    const CommandResult hsa = w.submit(v, HsaCommand{});
    REQUIRE(hsa.accepted());
    CHECK(w.commandBranch(hsa.activity, 0).reason == Reason::WrongCommandType);
    CHECK(w.commandBranch(r.activity, 1).reason == Reason::ActivityEnded); // (the hsa took its axes)
}

TEST_CASE("route branches: a stack on its own takes them too", "[modes]") {
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
    const std::vector<Waypoint> points = threePaths(s.latitudeRad, s.longitudeRad, 3000.0, paths);
    ControlStack stack;
    stack.command(RouteCommand{}, points, {}, {}, paths, std::vector<RouteBranch>{branchAt(1, 6.0)});
    for (int i = 0; i < 10; ++i) stack.update(ctx, out);
    std::uint32_t next[8];
    bool ends = false;
    const std::uint32_t n = stack.ahead(0, next, 8, ends);
    CHECK(list(std::vector<std::uint32_t>(next, next + n)) == "0 1 2 3 4 5 2 3");
}
