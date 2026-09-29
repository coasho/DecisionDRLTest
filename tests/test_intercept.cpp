// A-GRA's route intercept (docs/flight-autonomy.md, 4.47; ADR-29 FA-8d): ROUTE_INTERCEPT - a route plan the vehicle keeps,
// joined where its method chooses from where the aircraft is, and flown from there; the plan activated by it, and its status.
#include "mode_flights.h"

#include "core/Geodesy.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

/// A place `u` metres along `headingRad` from (lat0, lon0) and `v` metres to its left.
void placeAt(double lat0, double lon0, double headingRad, double u, double v, double& lat, double& lon) {
    const double north = u * std::cos(headingRad) + v * std::sin(headingRad), east = u * std::sin(headingRad) - v * std::cos(headingRad);
    lat = lat0 + north / kEarthM, lon = lon0 + east / (kEarthM * std::cos(lat0));
}

/// Along and left of `headingRad` from (lat0, lon0), m.
void frameOf(double lat0, double lon0, double headingRad, double lat, double lon, double& u, double& v) {
    const double north = (lat - lat0) * kEarthM, east = (lon - lon0) * kEarthM * std::cos(lat0);
    u = north * std::cos(headingRad) + east * std::sin(headingRad), v = north * std::sin(headingRad) - east * std::cos(headingRad);
}

/// A straight plan of six points along the aircraft's heading, `leftM` to its left, `spacingM` apart - the aircraft abeam 40 %
/// of the leg from point 2 to point 3 - at its altitude and `speed` (in `reference`).
RoutePlan besideRoute(PlanId id, const sim::VehicleState& s, double headingRad, double spacingM, double leftM, double speed, SpeedReference reference) {
    RoutePlan p;
    p.id = id;
    for (int i = 0; i < 6; ++i) {
        Waypoint q;
        placeAt(s.latitudeRad, s.longitudeRad, headingRad, (i - 2.4) * spacingM, leftM, q.latitudeRad, q.longitudeRad);
        q.altitudeM = s.altitudeMslM, q.speed = speed, q.speedReference = static_cast<double>(reference);
        q.id = 100 + static_cast<std::uint64_t>(i);
        p.waypoints.push_back(q);
    }
    return p;
}

void upload(session::World& w, std::uint32_t v, const RoutePlan& plan) {
    REQUIRE(w.planCommand(v, plan.id, PlanCommand::PrepareForUpload).completed);
    REQUIRE(w.publishPlan(v, plan) == Reason::None);
    REQUIRE(w.planCommand(v, plan.id, PlanCommand::Upload).completed);
}

InterceptCommand interceptOf(PlanId plan, double method) {
    InterceptCommand c;
    c.plan = static_cast<double>(plan), c.method = method;
    return c;
}

/// What a flight of an intercept came to: the points captured in order, how near it passed each of the plan's, how long.
struct Flown {
    std::vector<std::int32_t> captured;
    std::vector<double> nearest;
    double seconds = 0.0;
    bool completed = false;
};

Flown flyIntercept(session::World& w, std::uint32_t v, ActivityId a, const RoutePlan& plan, double limitS) {
    Flown f;
    f.nearest.assign(plan.waypoints.size(), 1e12);
    std::int32_t current = -1;
    for (double t = 0.0; t < limitS; t += 1.0) {
        fly(w, 1.0);
        const sim::VehicleState& s = *w.vehicleState(v);
        for (std::size_t i = 0; i < plan.waypoints.size(); ++i)
            f.nearest[i] = std::min(f.nearest[i], geo::distanceM(s.latitudeRad, s.longitudeRad, plan.waypoints[i].latitudeRad, plan.waypoints[i].longitudeRad));
        InterceptStatus st;
        if (w.interceptStatus(a, st) && st.hasPrevious && st.previous.point != current) f.captured.push_back(current = st.previous.point);
        if (w.activity(a)->state == ActivityState::Completed) {
            f.completed = true, f.seconds = t + 1.0;
            if (f.captured.empty() || f.captured.back() != 5) f.captured.push_back(5); // (its last point, as it completed)
            break;
        }
    }
    return f;
}

struct Class {
    const char* type;
    bool rotor;
    double spacingM, leftM, speed;
    double passM; ///< how near it passes each point after the one it joins at, at most (that one: a fly-by's corner cut)
};

} // namespace

TEST_CASE("intercept: each method joins where it should, and flies the plan from there - per class (RIC-01, RIC-02)", "[modes][intercept]") {
    // a plan of six points in a line along the aircraft's heading, to its left, the aircraft abeam 40 % of the leg into point 3
    // (how near each passes the points after the one it joins at is the route follower's: at worst an F-16C 192 m at the point
    // after turning back onto the plan's beginning, a UH-60A 7.4 m, an IRIS 2.5 m)
    const Class classes[] = {{"c172x", false, 3000.0, 2000.0, 55.0, 30.0},
                             {"f16c", false, 8000.0, 5000.0, 160.0, 250.0},
                             {"uh60", true, 600.0, 300.0, 15.0, 10.0},
                             {"iris", true, 150.0, 60.0, 5.0, 3.0}};
    for (const Class& k : classes) {
        for (int m = -1; m < static_cast<int>(InterceptMethod::Count); ++m) {
            INFO(k.type << ", method " << m);
            session::World w(options("intercept-methods"));
            const std::uint32_t v = k.rotor ? rotor(w, k.type) : wing(w, k.type, 1500.0, k.speed, 0);
            if (!k.rotor) fly(w, 5.0);
            const sim::VehicleState s0 = *w.vehicleState(v);
            const double heading = k.rotor ? 0.0 : track(s0);
            const RoutePlan plan = besideRoute(7, s0, heading, k.spacingM, k.leftM, k.speed, k.rotor ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
            upload(w, v, plan);
            const CommandResult r = w.submit(v, interceptOf(7, m < 0 ? kHold : static_cast<double>(m)));
            INFO(reasonName(r.reason) << " at " << r.index);
            REQUIRE(r.accepted());
            InterceptStatus st;
            REQUIRE(w.interceptStatus(r.activity, st));
            CHECK(st.plan == 7);
            CHECK(w.planStatus(v, 7)->state == PlanState::Activated);
            CHECK(w.planStatus(v, 7)->activity == r.activity);
            // where it joins: the beginning; the nearest point (2); the foot of the perpendicular, 40 % along the leg into point 3;
            // the soonest - a wing's ahead of the foot, a rotorcraft's (turning at once, in still air) the foot
            double u = 0.0, lv = 0.0;
            if (st.laid >= 0) frameOf(s0.latitudeRad, s0.longitudeRad, heading, st.joinLatitudeRad, st.joinLongitudeRad, u, lv);
            int first = 0;
            if (m < 0) {
                CHECK(st.joined == 0);
                CHECK(st.laid == -1);
            } else if (m == static_cast<int>(InterceptMethod::Discrete)) {
                CHECK(st.joined == 2);
                CHECK(st.laid == -1);
                first = 2;
            } else if (m == static_cast<int>(InterceptMethod::ShortestDistance)) {
                CHECK(st.joined == 3);
                CHECK(st.laid == 6);
                CHECK(std::abs(u) < 1.0);
                CHECK(std::abs(lv - k.leftM) < 1.0);
                first = 3;
            } else {
                CHECK(st.joined >= 3);
                CHECK(st.laid == 6);
                if (k.rotor) CHECK(std::abs(u) < 1.0);
                else CHECK(u > 0.05 * k.spacingM); // (ahead: it turns toward the route as it goes on)
                CHECK(std::abs(lv - k.leftM) < 1.0);
                first = st.joined;
            }
            // flown: from where it joins to the plan's end, near each point on the way; the plan's execution complete
            const double length = (2.4 + 2.6 + 2.4) * k.spacingM + k.leftM;
            const Flown f = flyIntercept(w, v, r.activity, plan, 3.0 * length / k.speed + 120.0);
            CHECK(f.completed);
            for (int i = first + 1; i < 6; ++i) CHECK(f.nearest[static_cast<std::size_t>(i)] < k.passM);
            REQUIRE(!f.captured.empty());
            CHECK(f.captured.front() == first); // (the point it joined at first, then each after it in turn)
            for (std::size_t i = 1; i < f.captured.size(); ++i) CHECK(f.captured[i] == f.captured[i - 1] + 1);
            CHECK(f.captured.back() == 5);
            CHECK(w.planStatus(v, 7)->execution == PlanExecution::Complete);
            REQUIRE(w.interceptStatus(r.activity, st));
            CHECK(st.execution == PlanExecution::Complete);
        }
    }
}

TEST_CASE("intercept: bounds, a path of its own, and its status - capture estimates, loiter progress, kinematics (RIC-01, RIC-03)",
          "[modes][intercept]") {
    session::World w(options("intercept-status"));
    const std::uint32_t v = wing(w, "c172x", 1500.0, 55.0, 0);
    fly(w, 5.0);
    const sim::VehicleState s0 = *w.vehicleState(v);
    const double heading = track(s0);
    RoutePlan plan = besideRoute(8, s0, heading, 3000.0, 2000.0, 55.0, SpeedReference::TrueAirspeed);
    // an alternate path of three points 4 km to the aircraft's right, apart from the primary (points 0 to 5)
    for (int i = 0; i < 3; ++i) {
        Waypoint q = plan.waypoints[0];
        placeAt(s0.latitudeRad, s0.longitudeRad, heading, (i + 1) * 3000.0, -4000.0, q.latitudeRad, q.longitudeRad);
        q.id = 200 + static_cast<std::uint64_t>(i);
        plan.waypoints.push_back(q);
    }
    RoutePath primary, alternate;
    primary.id = 11, primary.first = 0, primary.count = 6;
    alternate.id = 12, alternate.type = static_cast<double>(PathType::Alternate), alternate.first = 6, alternate.count = 3;
    plan.paths = {primary, alternate};
    upload(w, v, plan);
    auto joined = [&](InterceptCommand c) {
        const CommandResult r = w.submit(v, c);
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        InterceptStatus st;
        REQUIRE(w.interceptStatus(r.activity, st));
        return st;
    };
    // bounds: from point 4 on, the nearest point is 4; to point 1, the nearest place is point 1's; 3 alone, soonest: on its leg
    InterceptCommand c = interceptOf(8, static_cast<double>(InterceptMethod::Discrete));
    c.earliest = 4.0;
    CHECK(joined(c).joined == 4);
    c = interceptOf(8, static_cast<double>(InterceptMethod::ShortestDistance));
    c.latest = 1.0;
    InterceptStatus st = joined(c);
    CHECK(st.joined == 1);
    CHECK(st.laid == -1);
    c = interceptOf(8, static_cast<double>(InterceptMethod::Soonest));
    c.earliest = c.latest = 3.0;
    st = joined(c);
    CHECK(st.joined == 3);
    // a path of its own: the alternate's points alone - the nearest of them (its first, 3 km ahead and 4 km right)
    c = interceptOf(8, static_cast<double>(InterceptMethod::Discrete));
    c.path = 12.0;
    CHECK(joined(c).joined == 6);
    // the primary's nearest place: the foot on the leg into point 3; flown, its segments reported as it goes
    c = interceptOf(8, static_cast<double>(InterceptMethod::ShortestDistance));
    c.path = 11.0;
    const CommandResult r = w.submit(v, c);
    REQUIRE(r.accepted());
    REQUIRE(w.interceptStatus(r.activity, st));
    CHECK(st.joined == 3);
    CHECK(st.laid == 9); // (after the plan's nine)
    fly(w, 1.0);
    REQUIRE(w.interceptStatus(r.activity, st));
    CHECK(st.execution == PlanExecution::Executing);
    // flying to the point laid in: the current segment is the leg it joins - point 3, its path the primary - and none before it
    CHECK_FALSE(st.hasPrevious);
    REQUIRE(st.hasCurrent);
    CHECK(st.current.point == 3);
    CHECK(st.current.pointId == 103);
    CHECK(st.current.path == 11);
    CHECK(std::abs(degreesApart(st.current.headingRad, heading)) < 0.5); // (the leg's course: the aircraft's heading)
    REQUIRE(st.hasNext);
    CHECK(st.next.point == 4);
    CHECK(st.next.captureDistanceM > st.current.captureDistanceM + 2900.0);
    const double firstDistance = st.current.captureDistanceM, firstTime = st.current.captureTimeS;
    CHECK(firstDistance > 0.6 * 3000.0);
    CHECK(firstTime > w.simTime());
    // on the leg: nearer, sooner; along it at its speed over the ground, not across
    double capturedAt = -1.0;
    for (int k = 0; k < 400 && capturedAt < 0.0; ++k) {
        fly(w, 1.0);
        REQUIRE(w.interceptStatus(r.activity, st));
        if (st.hasPrevious && st.previous.point == 3) capturedAt = w.simTime();
    }
    REQUIRE(capturedAt > 0.0);
    CHECK(std::abs(st.previous.captureTimeS - capturedAt) < 1.5); // (when it captured point 3)
    CHECK(st.previous.captureDistanceM == 0.0);
    CHECK(st.current.point == 4);
    CHECK(std::abs(degreesApart(st.current.headingRad, heading)) < 0.5);
    const sim::VehicleState& now = *w.vehicleState(v);
    CHECK(std::abs(st.current.alongMs - groundSpeed(now)) < 2.0);
    CHECK(std::abs(st.current.acrossMs) < 2.0);
    CHECK(std::abs(st.current.captureTimeS - (w.simTime() + st.current.captureDistanceM / groundSpeed(now))) < 1e-6);
    // a loiter at point 4 in a plan of its own: its orbits counted in the status as it flies them
    RoutePlan loitering = besideRoute(9, *w.vehicleState(v), track(*w.vehicleState(v)), 3000.0, 1000.0, 55.0, SpeedReference::TrueAirspeed);
    loitering.waypoints[4].kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter orbit;
    orbit.point = 4;
    orbit.pattern.pattern = static_cast<double>(PatternKind::Orbit);
    orbit.pattern.radiusM = 800.0;
    orbit.shape.orbits = 2.0;
    loitering.loiters = {orbit};
    upload(w, v, loitering);
    InterceptCommand l = interceptOf(9, static_cast<double>(InterceptMethod::Discrete));
    l.earliest = 3.0;
    const CommandResult lr = w.submit(v, l);
    INFO(reasonName(lr.reason) << " at " << lr.index);
    REQUIRE(lr.accepted());
    CHECK(w.planStatus(v, 8)->execution == PlanExecution::Superseded); // (the one it replaced)
    std::uint32_t mostOrbits = 0;
    bool loitered = false;
    for (int k = 0; k < 900 && w.activity(lr.activity)->state != ActivityState::Completed; ++k) {
        fly(w, 1.0);
        if (w.interceptStatus(lr.activity, st) && st.hasCurrent && st.current.loiter) {
            loitered = true;
            CHECK(st.current.point == 4);
            mostOrbits = std::max(mostOrbits, st.current.orbits);
        }
    }
    CHECK(loitered);
    CHECK(mostOrbits >= 1);
    CHECK(w.activity(lr.activity)->state == ActivityState::Completed);
    CHECK(w.planStatus(v, 9)->execution == PlanExecution::Complete);
}

TEST_CASE("intercept: refusals naming their fields, the plan's states, no UPDATE, no task; superseded and joined again; queued, "
          "its join laid as it starts",
          "[modes][intercept]") {
    session::World w(options("intercept-refusals"));
    const std::uint32_t v = wing(w, "c172x", 1500.0, 55.0, 0);
    fly(w, 5.0);
    const sim::VehicleState s0 = *w.vehicleState(v);
    const double heading = track(s0);
    const RoutePlan plan = besideRoute(7, s0, heading, 3000.0, 2000.0, 55.0, SpeedReference::TrueAirspeed);
    auto refusal = [&](InterceptCommand c) {
        const CommandResult r = w.submit(v, c);
        CHECK_FALSE(r.accepted());
        return std::pair<Reason, int>{r.reason, r.index};
    };
    CHECK(refusal(interceptOf(7, kHold)).first == Reason::UnknownPlan);
    REQUIRE(w.planCommand(v, 7, PlanCommand::PrepareForUpload).completed);
    CHECK(refusal(interceptOf(7, kHold)).first == Reason::WrongPlanState); // (prepared for upload: never uploaded)
    REQUIRE(w.publishPlan(v, plan) == Reason::None);
    REQUIRE(w.planCommand(v, 7, PlanCommand::Upload).completed);
    CHECK(refusal(interceptOf(0, kHold)) == std::pair<Reason, int>{Reason::InvalidParameter, 0});
    CHECK(refusal(interceptOf(7, 3.0)) == std::pair<Reason, int>{Reason::InvalidParameter, 2});
    InterceptCommand c = interceptOf(7, kHold);
    c.path = 5.0; // (the plan has no paths)
    CHECK(refusal(c) == std::pair<Reason, int>{Reason::InvalidParameter, 1});
    c = interceptOf(7, kHold);
    c.earliest = 9.0;
    CHECK(refusal(c) == std::pair<Reason, int>{Reason::InvalidParameter, 3});
    c.earliest = 4.0, c.latest = 2.0;
    CHECK(refusal(c) == std::pair<Reason, int>{Reason::InvalidParameter, 4});
    RoutePlan planning = plan;
    planning.id = 8, planning.forPlanningUseOnly = true;
    REQUIRE(w.planCommand(v, 8, PlanCommand::PrepareForUpload).completed);
    REQUIRE(w.publishPlan(v, planning) == Reason::None);
    REQUIRE(w.planCommand(v, 8, PlanCommand::Upload).completed);
    CHECK(refusal(interceptOf(8, kHold)).first == Reason::PlanningOnly);
    // validated alone: nothing flies, the plan as it was
    CommandOptions check;
    check.validateOnly = true;
    CHECK(w.submit(v, interceptOf(7, static_cast<double>(InterceptMethod::Soonest)), check).status == CommandStatus::Valid);
    CHECK(w.planStatus(v, 7)->state == PlanState::Uploaded);
    // accepted: the plan activated by it; executing, a second refused; no UPDATE; never a task
    const CommandResult r = w.submit(v, interceptOf(7, static_cast<double>(InterceptMethod::ShortestDistance)));
    REQUIRE(r.accepted());
    fly(w, 2.0);
    CHECK(w.planStatus(v, 7)->execution == PlanExecution::Executing);
    CHECK(w.update(r.activity, interceptOf(7, kHold)).reason == Reason::NotUpdatable);
    CHECK(w.update(r.activity, RouteCommand{}, {}).reason == Reason::NotUpdatable); // (its route: through it alone)
    TaskRepetition once;
    once.attempts = 1;
    BatchCommand item;
    const InterceptCommand kept = interceptOf(7, kHold);
    item.intercept = &kept;
    CHECK(w.storeTask(v, 3, item, once) == Reason::NotImplemented);
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    REQUIRE(sp.intercept.has_value());
    CHECK(sp.intercept->method == static_cast<double>(InterceptMethod::ShortestDistance));
    CHECK(sp.waypoints.size() == 7); // (the plan's six, and the point laid in for its join)
    CHECK(sp.join.laid == 6);
    // intercepted again while it flies: the new one replaces it, as any NEW does - the plan's activity the new one's
    const CommandResult rejoin = w.submit(v, interceptOf(7, static_cast<double>(InterceptMethod::Discrete)));
    REQUIRE(rejoin.accepted());
    fly(w, 1.0);
    CHECK(w.activity(r.activity)->state != ActivityState::Active);
    CHECK(w.planStatus(v, 7)->activity == rejoin.activity);
    CHECK(w.planStatus(v, 7)->execution == PlanExecution::Executing);
    // superseded - another command diverts it - then joined again: the plan's execution its new activity's
    HsaCommand away;
    away.courseRad = heading + 0.5 * kPi, away.speed = 55.0, away.speedReference = 0.0, away.altitudeM = 1500.0;
    const CommandResult diverted = w.submit(v, Command(away));
    REQUIRE(diverted.accepted());
    fly(w, 60.0);
    CHECK(w.activity(r.activity)->state != ActivityState::Active);
    CHECK(w.planStatus(v, 7)->execution == PlanExecution::Superseded);
    const CommandResult again = w.submit(v, interceptOf(7, static_cast<double>(InterceptMethod::Soonest)));
    INFO(reasonName(again.reason) << " at " << again.index);
    REQUIRE(again.accepted());
    fly(w, 2.0);
    CHECK(w.planStatus(v, 7)->activity == again.activity);
    CHECK(w.planStatus(v, 7)->execution == PlanExecution::Executing);
    InterceptStatus st;
    REQUIRE(w.interceptStatus(r.activity, st)); // (the one it replaced: its plan and its end)
    CHECK(st.plan == 0);                         // (the plan's activity is the new one's now)
    CHECK(st.execution == PlanExecution::Superseded);
    // queued behind a start window: its join laid as it starts, from where the aircraft is then
    const std::uint32_t other = wing(w, "c172x", 1500.0, 55.0, 3);
    fly(w, 1.0);
    const sim::VehicleState o0 = *w.vehicleState(other);
    HsaCommand on; // (on along its track while the intercept waits)
    on.courseRad = track(o0), on.speed = 55.0, on.speedReference = 0.0, on.altitudeM = 1500.0;
    REQUIRE(w.submit(other, Command(on)).accepted());
    upload(w, other, besideRoute(7, o0, track(o0), 3000.0, 2000.0, 55.0, SpeedReference::TrueAirspeed));
    CommandOptions later;
    later.window.startNotBefore = w.simTime() + 45.0;
    const CommandResult q = w.submit(other, interceptOf(7, static_cast<double>(InterceptMethod::ShortestDistance)), later);
    REQUIRE(q.accepted());
    CHECK((q.flags & kDeferred) != 0);
    REQUIRE(w.interceptStatus(q.activity, st));
    CHECK(st.execution == PlanExecution::Pending);
    CHECK(st.joined == 3); // (as it was laid at its NEW)
    fly(w, 50.0);          // (on 2.5 km more: abeam the leg into point 4 now)
    REQUIRE(w.interceptStatus(q.activity, st));
    CHECK(st.execution == PlanExecution::Executing);
    CHECK(st.joined == 4);
}
