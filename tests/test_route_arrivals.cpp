// A route's required times of arrival as A-GRA's schema gives them (docs/flight-autonomy.md, 4.33; ADR-29 FA-6d1): a
// point's arrival window, the speed scheduled over the ground to arrive in it - slowed or sped up within the speeds the
// aircraft flies level - its estimate against the window reported; what cannot be made refused, naming the point.
#include "control/Route.h"
#include "fsim/GuidanceModes.h"
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

/// A point `north` and `east` metres from (lat0, lon0).
Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint p;
    p.latitudeRad = lat0 + north / kR, p.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return p;
}

/// A route flown to its end, or `seconds` on: when (world time) each of its points was reached (NaN: not).
struct Flight {
    std::vector<double> reached;
    double worstEstimate = 0.0; ///< its estimate against when it arrived, from a third of the way on
    double worstDelta = 0.0;    ///< its reported delta while scheduled
    bool ground = false;        ///< it reported a ground speed, scheduled
};

} // namespace

TEST_CASE("route arrivals: slowed to arrive in its window, or sped up; within it as planned; its estimate reported", "[modes]") {
    session::World w(options("route-arrivals"));
    const auto early = wing(w, "c172", 1500.0, 55.0), late = wing(w, "c172", 1500.0, 45.0, 3), within = wing(w, "c172", 1500.0, 55.0, 6);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    // east 3 km, then 12 km on to a point with a window: at 55 m/s it would be there some 273 s on. The first's window is a
    // minute later (slowed); the second's, at 45 m/s, about 40 s sooner than it would be (sped up); the third's holds it
    auto route = [&](std::uint32_t v, double speed, double begin, double end) {
        const auto& s0 = *w.vehicleState(v);
        Waypoint a = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0), b = at(s0.latitudeRad, s0.longitudeRad, 0.0, 15000.0);
        a.speed = speed;
        b.arrivalBeginS = t0 + begin, b.arrivalEndS = t0 + end;
        const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b});
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        return r.activity;
    };
    const ActivityId ids[3] = {route(early, 55.0, 330.0, 340.0), route(late, 45.0, 280.0, 290.0), route(within, 55.0, 260.0, 300.0)};
    const double aims[3] = {332.5, 287.5, kHold};
    Setpoint sp;
    REQUIRE(w.activitySetpoint(ids[0], sp));
    CHECK((sp.waypoints[1].arrivalBeginS == t0 + 330.0 && sp.waypoints[1].arrivalEndS == t0 + 340.0));
    double arrived[3] = {kHold, kHold, kHold}, worstEstimate[3] = {0.0, 0.0, 0.0}, estimateAt[3] = {kHold, kHold, kHold};
    bool ground[3] = {false, false, false};
    for (unsigned k = 0; k < stepsFor(w, 420.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        for (int j = 0; j < 3; ++j) {
            const ActivityRecord& r = *w.activity(ids[j]);
            if (!isHold(arrived[j])) continue;
            if (!r.live()) {
                arrived[j] = t;
                continue;
            }
            // (its estimate from a third of the way on, and its delta against its window: 0 once scheduled into it)
            ArrivalEstimate e;
            if (r.progress.percent > 33.0 && w.activityArrival(ids[j], e)) {
                if (isHold(estimateAt[j])) estimateAt[j] = e.arrivalS - t0;
                worstEstimate[j] = std::max(worstEstimate[j], std::abs(e.arrivalS - t0 - estimateAt[j]));
                CHECK(e.deltaS == 0.0);
            }
            ground[j] = ground[j] || r.progress.speedReference == static_cast<double>(SpeedReference::GroundSpeed);
        }
    }
    std::printf("route arrivals: a C172 slowed to its window 330-340 s arrived at %.1f s (aimed 332.5; its estimate from a third on %.1f, within %.2f s), "
                "one sped up to 280-290 s at %.1f s (aimed 287.5), one within 260-300 s as planned at %.1f s\n",
                arrived[0], estimateAt[0], worstEstimate[0], arrived[1], arrived[2]);
    for (int j = 0; j < 2; ++j) {
        INFO("the " << (j == 0 ? "slowed" : "sped-up") << " one");
        REQUIRE(!isHold(arrived[j]));
        CHECK(std::abs(arrived[j] - aims[j]) < 2.0); // (FA-6's acceptance: within 2 s)
        CHECK(std::abs(estimateAt[j] - aims[j]) < 1e-6);
        CHECK(worstEstimate[j] < 0.5);
        CHECK(ground[j]); // (its progress tells the ground speed it flies)
    }
    REQUIRE(!isHold(arrived[2]));
    CHECK((arrived[2] > 260.0 && arrived[2] < 300.0));
    CHECK_FALSE(ground[2]); // (as planned: its own speed)
}

TEST_CASE("route arrivals: a rotorcraft slowed over the ground; a window between points, the points after flown as planned; the next window's "
          "in turn",
          "[modes]") {
    session::World w(options("route-arrivals-between"));
    const auto quad = rotor(w, "iris", 15.0, 6);
    const auto v = wing(w, "c172", 1500.0, 55.0);
    const double t0 = w.simTime();
    // an IRIS north 300 m at 5 m/s over the ground (some 60 s): its window at 90-100 s
    const auto& q0 = *w.vehicleState(quad);
    Waypoint q = at(q0.latitudeRad, q0.longitudeRad, 300.0, 0.0);
    q.speed = 5.0, q.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
    q.arrivalBeginS = t0 + 90.0, q.arrivalEndS = t0 + 100.0;
    const CommandResult rq = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{q});
    REQUIRE(rq.accepted());
    // a C172 east: 3 km; 5 km on to a point at 200-205 s (slowed); 5 km on at its own speed, and 5 km on to 370-400 s (as
    // planned from there, it arrives in it)
    const auto& s0 = *w.vehicleState(v);
    Waypoint a = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0), b = at(s0.latitudeRad, s0.longitudeRad, 0.0, 8000.0),
             c = at(s0.latitudeRad, s0.longitudeRad, 0.0, 13000.0), d = at(s0.latitudeRad, s0.longitudeRad, 0.0, 18000.0);
    a.speed = 55.0;
    b.arrivalBeginS = t0 + 200.0, b.arrivalEndS = t0 + 205.0;
    d.arrivalBeginS = t0 + 370.0, d.arrivalEndS = t0 + 400.0;
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b, c, d});
    REQUIRE(r.accepted());
    double quadArrived = kHold, reached[4] = {kHold, kHold, kHold, kHold}, ownSpeed = kHold;
    std::uint32_t last = 0;
    for (unsigned k = 0; k < stepsFor(w, 480.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        const ActivityRecord& rr = *w.activity(r.activity);
        if (rr.live() && rr.progress.segment != last) reached[last] = t, last = rr.progress.segment;
        if (!rr.live() && isHold(reached[3])) reached[3] = t;
        if (rr.live() && rr.progress.segment == 2 && rr.progress.segmentPercent > 40.0 && isHold(ownSpeed)) ownSpeed = rr.progress.speedMs;
        if (isHold(quadArrived) && !w.activity(rq.activity)->live()) quadArrived = t;
    }
    std::printf("route arrivals: an IRIS's window 90-100 s arrived at %.1f s (aimed 92.5); a C172's 200-205 s at %.1f s (aimed 201.25), then at %.1f m/s "
                "its own, and 370-400 s at %.1f s\n",
                quadArrived, reached[1], ownSpeed, reached[3]);
    CHECK(std::abs(quadArrived - 92.5) < 2.0);
    CHECK(std::abs(reached[1] - 201.25) < 2.0);
    CHECK(std::abs(ownSpeed - 55.0) < 1e-9); // (the point after its window: as planned)
    CHECK((reached[3] > 370.0 && reached[3] < 400.0));
}

TEST_CASE("route arrivals: refused as a point is, naming it - a window upside down or not finite, one past, one it cannot make at the speeds it "
          "flies level; unchecked, flown; at or after a loiter point, not implemented",
          "[modes]") {
    session::World w(options("route-arrivals-refused"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    const auto& s0 = *w.vehicleState(v);
    const Waypoint p0 = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0);
    Waypoint p1 = at(s0.latitudeRad, s0.longitudeRad, 0.0, 15000.0);
    auto refusedAt = [&](const Waypoint& bad, Reason why, const char* what, Constraint constraint = Constraint::None, CommandOptions o = {}) {
        const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, bad}, o);
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == 1);
        if (constraint != Constraint::None) CHECK(r.constraint == constraint);
    };
    Waypoint upside = p1;
    upside.arrivalBeginS = t0 + 300.0, upside.arrivalEndS = t0 + 290.0;
    refusedAt(upside, Reason::InvalidWaypoint, "a begin after its end");
    Waypoint infinite = p1;
    infinite.arrivalEndS = std::numeric_limits<double>::infinity();
    refusedAt(infinite, Reason::InvalidWaypoint, "an end not finite");
    Waypoint past = p1;
    past.arrivalEndS = t0 - 1.0;
    refusedAt(past, Reason::InvalidWaypoint, "an end already past");
    // 15 km: at its fastest level speed some 230 s; at its slowest some 470 s
    double least = kHold, most = kHold;
    route::levelSpeedsMs(&w.profile(v)->tables, *w.performance(v), false, 1500.0, s0.fuelKg, least, most);
    INFO("the C172's level speeds: " << least << " to " << most << " m/s");
    Waypoint soon = p1;
    soon.arrivalEndS = t0 + 0.8 * 15000.0 / most;
    refusedAt(soon, Reason::PerformanceLimit, "too soon for its fastest", Constraint::MaxAirspeed);
    CommandOptions clamp;
    clamp.range = RangePolicy::Clamp;
    refusedAt(soon, Reason::PerformanceLimit, "too soon, clamped: no clamp mends it", Constraint::MaxAirspeed, clamp);
    Waypoint slow = p1;
    slow.arrivalBeginS = t0 + 1.25 * 15000.0 / least;
    refusedAt(slow, Reason::PerformanceLimit, "too late for its slowest", Constraint::MinAirspeed);
    CommandOptions unchecked;
    unchecked.range = RangePolicy::None;
    CHECK(w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, soon}, unchecked).accepted()); // (flown at its fastest)
    // a window at or after a loiter point: not implemented, naming it
    Waypoint loiterAt = p0;
    loiterAt.kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter l;
    l.point = 0, l.pattern.durationS = 60.0;
    Waypoint after = p1;
    after.arrivalEndS = t0 + 600.0;
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{loiterAt, after}, {}, std::vector<RouteLoiter>{l});
    CHECK((r.reason == Reason::NotImplemented && r.index == 1));
    const SupportInfo* row = w.supportTable(v)->find("fsim.guidance.route/required_time_of_arrival");
    REQUIRE(row != nullptr);
    CHECK(row->support == Support::Partial);
    // the stock C172x, without the tables its speeds would come from: not implemented, naming the point
    const auto stock = wing(w, "c172x", 1500.0, 55.0, 3);
    const auto& x0 = *w.vehicleState(stock);
    Waypoint timed = at(x0.latitudeRad, x0.longitudeRad, 0.0, 9000.0);
    timed.arrivalEndS = t0 + 600.0;
    const CommandResult none = w.submit(stock, RouteCommand{}, std::vector<Waypoint>{at(x0.latitudeRad, x0.longitudeRad, 0.0, 3000.0), timed});
    CHECK((none.reason == Reason::NotImplemented && none.index == 1));
    CHECK(w.supportTable(stock)->find("fsim.guidance.route/required_time_of_arrival")->support == Support::NotImplemented);
}
