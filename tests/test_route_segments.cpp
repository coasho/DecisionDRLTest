// A route's segment performance as A-GRA's schema gives it (docs/flight-autonomy.md, 4.32; ADR-29 FA-6c): a segment flown
// at the performance tables' best speed at the altitude and weight now, its speed replaced; the speed change into a
// segment made at its acceleration, a wing's through the air and a rotorcraft's over the ground; a climb or descent at the
// most the aircraft makes holding its speed, an efficient climb timed by the tables and an efficient descent along its
// gradient; what does not make one refused, naming the point; an acceleration beyond the aircraft's held to it.
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

Setpoint flown(session::World& w, ActivityId a) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(a, s));
    return s;
}

double code(SpeedOptimization o) { return static_cast<double>(o); }

/// The least-squares slope of y against x.
double slope(const std::vector<double>& x, const std::vector<double>& y) {
    const double n = static_cast<double>(x.size());
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) sx += x[i], sy += y[i], sxx += x[i] * x[i], sxy += x[i] * y[i];
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

} // namespace

TEST_CASE("route segments: a segment flown at the tables' best speed now, its speed replaced; continued with the speed left out, replaced by "
          "one given; not implemented without tables",
          "[modes]") {
    session::World w(options("route-segments-optimised"));
    const auto v = wing(w, "c172", 1500.0, 50.0); // (hangar's C172: it has performance tables)
    const auto stock = wing(w, "c172x", 1500.0, 55.0, 3);
    w.step(stepsFor(w, 2.0));
    const VehicleProfile& profile = *w.profile(v);
    REQUIRE_FALSE(profile.tables.empty());
    // no tables: not implemented, naming the point
    const auto& t0 = *w.vehicleState(stock);
    Waypoint q = at(t0.latitudeRad, t0.longitudeRad, 0.0, 5000.0);
    q.speedOptimization = code(SpeedOptimization::MaxEndurance);
    const CommandResult none = w.submit(stock, RouteCommand{}, std::vector<Waypoint>{at(t0.latitudeRad, t0.longitudeRad, 0.0, 2000.0), q});
    CHECK((none.reason == Reason::NotImplemented && none.index == 1));
    CHECK(w.supportTable(stock)->find("fsim.guidance.route/speed/max_endurance")->support == Support::NotImplemented);
    CHECK(w.supportTable(v)->find("fsim.guidance.route/speed/max_endurance")->support == Support::Supported);
    Waypoint up = q;
    up.speedOptimization = kHold, up.climbOptimization = static_cast<double>(ClimbOptimization::ExtendedRange), up.altitudeM = 1800.0;
    const CommandResult noClimb = w.submit(stock, RouteCommand{}, std::vector<Waypoint>{at(t0.latitudeRad, t0.longitudeRad, 0.0, 2000.0), up});
    CHECK((noClimb.reason == Reason::NotImplemented && noClimb.index == 1));
    CHECK(w.supportTable(stock)->find("fsim.guidance.route/climb/extended_range")->support == Support::NotImplemented);
    CHECK(w.supportTable(v)->find("fsim.guidance.route/climb/best_rate")->support == Support::Supported);
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    // east: 3 km at 50 m/s; to 15 km at the best range speed; on to 22 km, its speed left out; to 28 km at 45 m/s
    Waypoint p0 = at(lat0, lon0, 0.0, 3000.0), p1 = at(lat0, lon0, 0.0, 15000.0), p2 = at(lat0, lon0, 0.0, 22000.0), p3 = at(lat0, lon0, 0.0, 28000.0);
    p0.speed = 50.0;
    p1.speedOptimization = code(SpeedOptimization::LongRangeCruise);
    p3.speed = 45.0;
    const CommandResult c = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, p1, p2, p3});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    const Setpoint sp = flown(w, c.activity);
    const double fuel = w.vehicleState(v)->fuelKg, h = w.vehicleState(v)->altitudeMslM; // (its altitude left out: the aircraft's as it was sent)
    INFO("point 1's altitude " << sp.waypoints[1].altitudeM << " (reference " << sp.waypoints[1].altitudeReference << "), the aircraft's " << h);
    const double best = optimalTasMs(&profile.tables, code(SpeedOptimization::LongRangeCruise), h, fuel);
    REQUIRE(std::isfinite(best));
    // read back: its snapshot at its point's altitude, a true airspeed; the next continues it; the last a speed of its own
    CHECK(sp.waypoints[1].speedOptimization == code(SpeedOptimization::LongRangeCruise));
    CHECK(std::abs(sp.waypoints[1].speed - best) < 1e-6);
    CHECK(sp.waypoints[1].speedReference == static_cast<double>(SpeedReference::TrueAirspeed));
    CHECK(sp.waypoints[2].speedOptimization == code(SpeedOptimization::LongRangeCruise));
    CHECK(isHold(sp.waypoints[3].speedOptimization));
    CHECK(sp.waypoints[3].speed == 45.0);
    // flown at the best range speed now, in its segments' middles; then at 45 m/s
    double worst = 0.0, reported = 0.0, lastBest = kHold, settled = kHold;
    int samples = 0;
    for (unsigned k = 0; k < stepsFor(w, 900.0); ++k) {
        w.step();
        const ActivityRecord& a = *w.activity(c.activity);
        if (!a.live()) break;
        const ActivityProgress& g = a.progress;
        const auto& s = *w.vehicleState(v);
        if ((g.segment == 1 || g.segment == 2) && g.segmentPercent > 40.0 && g.segmentPercent < 95.0) {
            lastBest = optimalTasMs(&profile.tables, code(SpeedOptimization::LongRangeCruise), s.altitudeMslM, s.fuelKg);
            worst = std::max(worst, std::abs(s.airspeedTrueMs - lastBest));
            reported = std::max(reported, std::abs(g.speedMs - lastBest));
            ++samples;
        }
        if (g.segment == 3 && g.segmentPercent > 60.0) settled = s.airspeedTrueMs;
    }
    std::printf("route segments, a best range speed: %.3f m/s planned, %.3f now; flown within %.4f m/s of it (%d samples), reported within %.6f; "
                "then %.2f m/s given 45\n",
                best, lastBest, worst, samples, reported, settled);
    CHECK(samples > 0);
    CHECK(worst < 1.5);
    CHECK(reported < 1e-3); // (as the loops read the fuel, a step before)
    CHECK(std::abs(settled - 45.0) < 1.0);
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
}

TEST_CASE("route segments: the speed change into a segment made at its acceleration - a wing's through the air, a rotorcraft's over the ground",
          "[modes]") {
    session::World w(options("route-segments-acceleration"));
    const auto ramped = wing(w, "c172x", 1500.0, 55.0), natural = wing(w, "c172x", 1500.0, 55.0, 3);
    const auto quad = rotor(w, "iris", 15.0, 6);
    const auto& r0 = *w.vehicleState(ramped);
    const auto& n0 = *w.vehicleState(natural);
    // east 3 km at 55 m/s, then 12 km on at 40 m/s: slowing at 0.2 m/s^2 over 75 s, or as its loops do
    Waypoint a0 = at(r0.latitudeRad, r0.longitudeRad, 0.0, 3000.0), a1 = at(r0.latitudeRad, r0.longitudeRad, 0.0, 15000.0);
    a0.speed = 55.0, a1.speed = 40.0, a1.accelerationMs2 = 0.2;
    Waypoint b0 = at(n0.latitudeRad, n0.longitudeRad, 0.0, 3000.0), b1 = at(n0.latitudeRad, n0.longitudeRad, 0.0, 15000.0);
    b0.speed = 55.0, b1.speed = 40.0;
    const CommandResult rc = w.submit(ramped, RouteCommand{}, std::vector<Waypoint>{a0, a1});
    const CommandResult nc = w.submit(natural, RouteCommand{}, std::vector<Waypoint>{b0, b1});
    REQUIRE((rc.accepted() && nc.accepted()));
    CHECK(flown(w, rc.activity).waypoints[1].accelerationMs2 == 0.2);
    // a rotorcraft from its hover: north 150 m at 6 m/s, reached at 0.5 m/s^2 over the ground
    const auto& q0 = *w.vehicleState(quad);
    Waypoint c0 = at(q0.latitudeRad, q0.longitudeRad, 150.0, 0.0);
    c0.speed = 6.0, c0.speedReference = static_cast<double>(SpeedReference::GroundSpeed), c0.accelerationMs2 = 0.5;
    const CommandResult qc = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{c0});
    INFO(reasonName(qc.reason) << " at " << qc.index);
    REQUIRE(qc.accepted());
    std::vector<double> tr, vr, tq, vq;
    double down[2][2] = {{kHold, kHold}, {kHold, kHold}}, reported = 0.0; // (when each first slowed through 53 m/s and 42 m/s)
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 200.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        const ActivityRecord& a = *w.activity(rc.activity);
        const ActivityProgress& g = a.progress;
        const double va = w.vehicleState(ramped)->airspeedTrueMs, vb = w.vehicleState(natural)->airspeedTrueMs;
        if (a.live() && g.segment == 1 && va > 42.0 && va < 53.0) tr.push_back(t), vr.push_back(va); // (its middle)
        const double speeds[2] = {va, vb};
        const bool second[2] = {a.live() && g.segment == 1, w.activity(nc.activity)->live() && w.activity(nc.activity)->progress.segment == 1};
        for (int i = 0; i < 2; ++i) {
            if (second[i] && isHold(down[i][0]) && speeds[i] < 53.0) down[i][0] = t;
            if (second[i] && isHold(down[i][1]) && speeds[i] < 42.0) down[i][1] = t;
        }
        if (a.live() && g.segment == 1 && g.speedMs > 40.5 && g.speedMs < 54.5) reported = 1.0; // (its progress tells the ramp's speed)
        const auto& sq = *w.vehicleState(quad);
        const double gs = std::hypot(sq.velocityNedMs[0], sq.velocityNedMs[1]);
        if (w.activity(qc.activity)->live() && gs > 1.0 && gs < 5.0) tq.push_back(t), vq.push_back(gs);
    }
    const double wingRate = slope(tr, vr), quadRate = slope(tq, vq);
    const double rampAverage = 11.0 / (down[0][1] - down[0][0]), naturalAverage = 11.0 / (down[1][1] - down[1][0]);
    std::printf("route segments, accelerations: a C172x slowing at %.3f m/s^2 asked 0.2 (%zu samples; from 53 to 42 m/s at %.3f); as its loops do, "
                "at %.3f; an IRIS from its hover at %.3f m/s^2 asked 0.5 (%zu samples)\n",
                wingRate, tr.size(), rampAverage, naturalAverage, quadRate, tq.size());
    CHECK(tr.size() > 10);
    CHECK(std::abs(wingRate + 0.2) < 0.03);
    CHECK(std::abs(rampAverage - 0.2) < 0.03);
    CHECK(naturalAverage > 0.3); // (as its loops change it: faster)
    CHECK(reported == 1.0);
    CHECK(tq.size() > 10);
    CHECK(std::abs(quadRate - 0.5) < 0.08);
}

TEST_CASE("route segments: refused as a point is, naming it - an acceleration not above 0, an optimisation that is not one, a climb rate "
          "beside a climb optimisation; an acceleration beyond the aircraft's held to it, named by its point and field 24",
          "[modes]") {
    session::World w(options("route-segments-refusals"));
    const auto v = wing(w, "c172", 1500.0, 50.0);
    const auto quad = rotor(w, "iris", 15.0, 6);
    w.step(stepsFor(w, 1.0));
    const auto& s0 = *w.vehicleState(v);
    const Waypoint p0 = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0);
    Waypoint p1 = at(s0.latitudeRad, s0.longitudeRad, 0.0, 9000.0);
    auto refusedAt = [&](const Waypoint& bad, Reason why, const char* what) {
        const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p0, bad});
        INFO(what << ": " << reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == 1);
    };
    Waypoint zero = p1;
    zero.accelerationMs2 = 0.0;
    refusedAt(zero, Reason::InvalidWaypoint, "an acceleration of 0");
    Waypoint back = p1;
    back.accelerationMs2 = -1.0;
    refusedAt(back, Reason::InvalidWaypoint, "a negative acceleration");
    Waypoint notOne = p1;
    notOne.speedOptimization = 2.0;
    refusedAt(notOne, Reason::InvalidWaypoint, "an optimisation of 2");
    Waypoint climb = p1;
    climb.climbOptimization = static_cast<double>(ClimbOptimization::BestRate);
    climb.altitudeM = 2500.0, climb.climbRateMs = 1.0;
    refusedAt(climb, Reason::InvalidWaypoint, "a climb rate beside a climb optimisation (A-GRA's choice)");
    Waypoint notClimb = p1;
    notClimb.climbOptimization = 2.0;
    refusedAt(notClimb, Reason::InvalidWaypoint, "a climb optimisation of 2");
    // a C172 slowing from 50 to 35 m/s at 5 m/s^2: more than idle slows it at 35, held to that (MinAcceleration)
    Waypoint hard = p1;
    hard.speed = 35.0, hard.accelerationMs2 = 5.0;
    Waypoint first = p0;
    first.speed = 50.0;
    const CommandResult held = w.submit(v, RouteCommand{}, std::vector<Waypoint>{first, hard});
    REQUIRE(held.accepted());
    CHECK((held.flags & kClamped));
    const CommandDetails d = *w.commandDetails(v);
    REQUIRE(d.adjustmentCount >= 1);
    CHECK((d.adjustments[0].index == 1 && d.adjustments[0].field == 24 && d.adjustments[0].constraint == Constraint::MinAcceleration));
    std::printf("route segments, held: a C172 slowing from 50 to 35 m/s at 5 m/s^2, to %.3f m/s^2 (its idle's least between)\n", d.adjustments[0].adjusted);
    CHECK(d.adjustments[0].adjusted < 5.0);
    CHECK(d.adjustments[0].adjusted > 0.1);
    CHECK(flown(w, held.activity).waypoints[1].accelerationMs2 == d.adjustments[0].adjusted);
    // an IRIS's 30 m/s^2 from its hover: its own most (MaxAcceleration); under Reject, refused
    const auto& q0 = *w.vehicleState(quad);
    Waypoint fast = at(q0.latitudeRad, q0.longitudeRad, 100.0, 0.0);
    fast.speed = 6.0, fast.speedReference = static_cast<double>(SpeedReference::GroundSpeed), fast.accelerationMs2 = 30.0;
    const CommandResult quick = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{fast});
    REQUIRE(quick.accepted());
    const CommandDetails dq = *w.commandDetails(quad);
    REQUIRE(dq.adjustmentCount >= 1);
    CHECK((dq.adjustments[0].index == 0 && dq.adjustments[0].field == 24 && dq.adjustments[0].constraint == Constraint::MaxAcceleration));
    CHECK(std::abs(dq.adjustments[0].adjusted - w.performance(quad)->maxAccelerationMs2) < 1e-9);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    const CommandResult refused = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{fast}, reject);
    CHECK((refused.reason == Reason::PerformanceLimit && refused.index == 0 && refused.constraint == Constraint::MaxAcceleration));
}

TEST_CASE("route segments: a climb at the most the aircraft climbs holding its speed, then level; an efficient climb timed by the tables; "
          "an efficient descent along its gradient; a rotorcraft's",
          "[modes]") {
    session::World w(options("route-segments-climb"));
    const auto best = wing(w, "c172", 1500.0, 50.0), efficient = wing(w, "c172", 1500.0, 40.0, 3);
    const auto quad = rotor(w, "iris", 15.0, 6);
    w.step(stepsFor(w, 2.0));
    const VehicleProfile& profile = *w.profile(best);
    const Performance& perf = *w.performance(best);
    // east: 3 km, then 20 km on to 2,000 m, then 20 km on back down to 1,500 m, then 7 km: the first at 50 m/s flies a best
    // rate climb and descent; the second at 40 m/s an efficient climb and descent - where its tables have level flight
    // cheaper low at every altitude they fly, so it climbs late
    auto route = [&](std::uint32_t v, ClimbOptimization o, double speed) {
        const auto& s0 = *w.vehicleState(v);
        Waypoint a = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0), b = at(s0.latitudeRad, s0.longitudeRad, 0.0, 23000.0),
                 c = at(s0.latitudeRad, s0.longitudeRad, 0.0, 43000.0);
        a.speed = speed, a.altitudeM = 1500.0;
        b.altitudeM = 2000.0, c.altitudeM = 1500.0;
        b.climbOptimization = c.climbOptimization = static_cast<double>(o);
        const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b, c, at(s0.latitudeRad, s0.longitudeRad, 0.0, 50000.0)});
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        return r.activity;
    };
    const ActivityId ra = route(best, ClimbOptimization::BestRate, 50.0), rb = route(efficient, ClimbOptimization::ExtendedRange, 40.0);
    CHECK(route::cheapestAltitudeM(&profile.tables, 1500.0, 2000.0, 40.0, w.vehicleState(efficient)->fuelKg) == 1500.0);
    const Setpoint sp = flown(w, rb);
    CHECK(sp.waypoints[1].climbOptimization == static_cast<double>(ClimbOptimization::ExtendedRange));
    CHECK(isHold(sp.waypoints[1].climbRateMs)); // (its rate the aircraft's, as it flies)
    // a rotorcraft from its hover: north 200 m at 5 m/s, 600 m on up to 140 m at its best rate, 600 m on down to 100 m at it
    const auto& q0 = *w.vehicleState(quad);
    Waypoint qa = at(q0.latitudeRad, q0.longitudeRad, 200.0, 0.0), qb = at(q0.latitudeRad, q0.longitudeRad, 800.0, 0.0), qc = at(q0.latitudeRad, q0.longitudeRad, 1400.0, 0.0);
    qa.speed = 5.0, qa.speedReference = static_cast<double>(SpeedReference::GroundSpeed), qa.altitudeM = 100.0;
    qb.altitudeM = 140.0, qc.altitudeM = 100.0;
    qb.climbOptimization = qc.climbOptimization = static_cast<double>(ClimbOptimization::BestRate);
    const CommandResult rq = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{qa, qb, qc});
    INFO(reasonName(rq.reason) << " at " << rq.index);
    REQUIRE(rq.accepted());
    const VehicleProfile& quadProfile = *w.profile(quad);
    const Performance& quadPerf = *w.performance(quad);
    double climbOff = 0.0, descentOff = 0.0, speedOff = 0.0, gradientOff = 0.0, quadClimbOff = 0.0, quadDescentOff = 0.0;
    double halfway = kHold, levelAt = kHold, arrivedA = kHold, arrivedB = kHold, toppedB = kHold, climbVz = 0.0, quadVz = 0.0, quadDown = 0.0;
    int samples[4] = {0, 0, 0, 0};
    std::uint32_t lastA = 0, lastB = 0;
    for (unsigned k = 0; k < stepsFor(w, 1400.0); ++k) {
        w.step();
        const ActivityRecord& a = *w.activity(ra);
        const ActivityRecord& b = *w.activity(rb);
        const auto& sa = *w.vehicleState(best);
        const auto& sb = *w.vehicleState(efficient);
        // the best rate's: in the middle of its climb and its descent, the rate its tables promise now, its speed held
        if (a.live() && sa.altitudeMslM > 1550.0 && sa.altitudeMslM < 1950.0 && (a.progress.segment == 1 || a.progress.segment == 2)) {
            const bool up = a.progress.segment == 1;
            const double promised = route::climbRateMs(&profile.tables, perf, false, up, sa.altitudeMslM, sa.airspeedTrueMs, sa.fuelKg);
            const double vz = -sa.velocityNedMs[2];
            (up ? climbOff : descentOff) = std::max(up ? climbOff : descentOff, std::abs(std::abs(vz) - promised) / promised);
            if (up) climbVz = vz;
            speedOff = std::max(speedOff, std::abs(sa.airspeedTrueMs - 50.0));
            ++samples[up ? 0 : 1];
        }
        if (a.live() && a.progress.segment == 1 && isHold(levelAt) && sa.altitudeMslM > 1999.0) levelAt = a.progress.segmentPercent;
        if (a.live() && a.progress.segment != lastA) {
            if (lastA == 2) arrivedA = sa.altitudeMslM;
            lastA = a.progress.segment;
        }
        // the efficient one's: climbing early where the higher is cheaper at its speed; descending along its gradient
        if (b.live() && b.progress.segment == 1 && isHold(halfway) && b.progress.segmentPercent >= 50.0) halfway = sb.altitudeMslM;
        if (b.live() && b.progress.segment == 2 && b.progress.segmentPercent > 10.0 && b.progress.segmentPercent < 90.0) {
            gradientOff = std::max(gradientOff, std::abs(sb.altitudeMslM - (2000.0 - 5.0 * b.progress.segmentPercent)));
            ++samples[2];
        }
        if (b.live() && b.progress.segment != lastB) {
            if (lastB == 1) toppedB = sb.altitudeMslM;
            if (lastB == 2) arrivedB = sb.altitudeMslM;
            lastB = b.progress.segment;
        }
        // the rotorcraft's
        const ActivityRecord& r = *w.activity(rq.activity);
        const auto& sq = *w.vehicleState(quad);
        if (r.live() && (r.progress.segment == 1 || r.progress.segment == 2) && sq.altitudeMslM > 105.0 && sq.altitudeMslM < 135.0) {
            const bool up = r.progress.segment == 1;
            const double promised = route::climbRateMs(&quadProfile.tables, quadPerf, true, up, sq.altitudeMslM, sq.airspeedTrueMs, sq.fuelKg);
            const double vz = -sq.velocityNedMs[2];
            (up ? quadClimbOff : quadDescentOff) = std::max(up ? quadClimbOff : quadDescentOff, std::abs(std::abs(vz) - promised) / promised);
            (up ? quadVz : quadDown) = vz;
            ++samples[3];
        }
    }
    std::printf("route segments, climbs: a C172's best rate climb %.2f m/s, within %.1f %% of its tables' now, its descent within %.1f %%, its "
                "speed within %.2f m/s (%d, %d samples); level at 2,000 m %.0f %% into the segment, at 1,500 m %.1f m off at the point. An "
                "efficient climb at 1,500 m %+.1f m halfway, at 2,000 m %+.1f m at its point; its descent within %.1f m of its gradient (%d samples), %.1f m off at "
                "the point. An "
                "IRIS's climb %.2f m/s, within %.1f %% of its tables', its descent %.2f m/s, within %.1f %% of its most (%d samples)\n",
                climbVz, 100.0 * climbOff, 100.0 * descentOff, speedOff, samples[0], samples[1], levelAt, arrivedA - 1500.0, halfway - 1500.0, toppedB - 2000.0, gradientOff,
                samples[2], arrivedB - 1500.0, quadVz, 100.0 * quadClimbOff, quadDown, 100.0 * quadDescentOff, samples[3]);
    CHECK((samples[0] > 10 && samples[1] > 10 && samples[2] > 10 && samples[3] > 10));
    CHECK(climbOff < 0.05);
    CHECK(descentOff < 0.05);
    CHECK(speedOff < 2.5);
    CHECK(levelAt < 90.0);
    CHECK(std::abs(arrivedA - 1500.0) < 3.0);
    CHECK(std::abs(halfway - 1500.0) < 3.0); // (held low, where it is cheaper, until the climb's time)
    CHECK(std::abs(toppedB - 2000.0) < 10.0);
    CHECK(gradientOff < 10.0);
    CHECK(std::abs(arrivedB - 1500.0) < 3.0);
    CHECK(quadClimbOff < 0.1);
    CHECK(quadDescentOff < 0.1);
    CHECK(w.activity(ra)->state == ActivityState::Completed);
    CHECK(w.activity(rb)->state == ActivityState::Completed);
}

TEST_CASE("route segments: an efficient climb spends less fuel than a climb along the gradient; an efficient descent as much as one",
          "[modes]") {
    session::World w(options("route-segments-climb-fuel"));
    // two KC-135Rs east at 3,000 m and 170 m/s: 10 km, then 68 km on up to 4,000 m, then 68 km on back down; one efficient,
    // the other along the gradients
    const std::uint32_t ids[2] = {wing(w, "kc135r", 3000.0, 170.0), wing(w, "kc135r", 3000.0, 170.0, 3)};
    w.step(stepsFor(w, 2.0));
    ActivityId acts[2];
    for (int k = 0; k < 2; ++k) {
        const auto& s0 = *w.vehicleState(ids[k]);
        Waypoint a = at(s0.latitudeRad, s0.longitudeRad, 0.0, 10200.0), b = at(s0.latitudeRad, s0.longitudeRad, 0.0, 78200.0),
                 c = at(s0.latitudeRad, s0.longitudeRad, 0.0, 146200.0);
        a.speed = 170.0, a.altitudeM = 3000.0;
        b.altitudeM = 4000.0, c.altitudeM = 3000.0;
        if (k == 0) b.climbOptimization = c.climbOptimization = static_cast<double>(ClimbOptimization::ExtendedRange);
        const CommandResult r = w.submit(ids[k], RouteCommand{}, std::vector<Waypoint>{a, b, c, at(s0.latitudeRad, s0.longitudeRad, 0.0, 151300.0)});
        REQUIRE(r.accepted());
        acts[k] = r.activity;
    }
    double fuelAt[2][4] = {{kHold, kHold, kHold, kHold}, {kHold, kHold, kHold, kHold}};
    for (unsigned k = 0; k < stepsFor(w, 1000.0); ++k) {
        w.step();
        for (int j = 0; j < 2; ++j) {
            const ActivityRecord& r = *w.activity(acts[j]);
            const std::uint32_t g = r.live() ? r.progress.segment : 3;
            if (g >= 1 && g <= 3 && isHold(fuelAt[j][g])) fuelAt[j][g] = w.vehicleState(ids[j])->fuelKg;
        }
        if (!w.activity(acts[0])->live() && !w.activity(acts[1])->live()) break;
    }
    const double climbs[2] = {fuelAt[0][1] - fuelAt[0][2], fuelAt[1][1] - fuelAt[1][2]}, descents[2] = {fuelAt[0][2] - fuelAt[0][3], fuelAt[1][2] - fuelAt[1][3]};
    std::printf("route segments, efficiency: a KC-135R's efficient climb %.1f kg, along the gradient %.1f kg (%.1f %%); its descent %.1f kg, along the "
                "gradient %.1f kg\n",
                climbs[0], climbs[1], 100.0 * (climbs[0] / climbs[1] - 1.0), descents[0], descents[1]);
    CHECK(climbs[0] < 0.98 * climbs[1]);
    CHECK(std::abs(descents[0] / descents[1] - 1.0) < 0.01);
}
