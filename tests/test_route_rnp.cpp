// A route's required navigation performance as A-GRA's schema gives it (docs/flight-autonomy.md, 4.35; ADR-29 FA-6d3):
// a segment's RNP, in metres; the route farther off its path than it, its activity says so each world step
// (kActivityNavigationPerformance in its constraints) and remembers it (constraintsSeen); within it, and left out,
// nothing. A fly-by's arc is on its path. Refused as a point is, naming it.
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

/// A route's segment flown: its farthest off its path (as each step ends), the steps its activity said it was off its RNP,
/// and every step that ended off by more than it without saying so.
struct Watch {
    double farthest = 0.0;
    int flagged = 0, missed = 0, spurious = 0, steps = 0; ///< (spurious: said so, as the step ended well within it)
    bool seen = false, lastFlagged = false;
};

} // namespace

TEST_CASE("route RNP: farther off its path than its segment's RNP, its activity says so; within it, or left out, nothing", "[modes]") {
    session::World w(options("route-rnp"));
    // three C172s east at 1,500 m: each given a route north - a turn onto its first leg, 3 km, then 5 km on - with an RNP of
    // 20 m, 5 km, and none
    const std::uint32_t v[3] = {wing(w, "c172", 1500.0, 55.0), wing(w, "c172", 1500.0, 55.0, 3), wing(w, "c172", 1500.0, 55.0, 6)};
    w.step(stepsFor(w, 2.0));
    const double rnp[3] = {20.0, 5000.0, kHold};
    ActivityId ids[3];
    for (int j = 0; j < 3; ++j) {
        const auto& s = *w.vehicleState(v[j]);
        Waypoint a = at(s.latitudeRad, s.longitudeRad, 3000.0, 0.0), b = at(s.latitudeRad, s.longitudeRad, 8000.0, 0.0);
        a.rnpM = b.rnpM = rnp[j];
        const CommandResult r = w.submit(v[j], RouteCommand{}, std::vector<Waypoint>{a, b});
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        ids[j] = r.activity;
        Setpoint sp;
        REQUIRE(w.activitySetpoint(r.activity, sp));
        CHECK((sp.waypoints[0].rnpM == rnp[j] || (isHold(rnp[j]) && isHold(sp.waypoints[0].rnpM)))); // (read back; left out, none)
    }
    // its first leg flown: 90 s; then the second, straight
    Watch first[3], second[3];
    for (unsigned k = 0; k < stepsFor(w, 150.0); ++k) {
        w.step();
        for (int j = 0; j < 3; ++j) {
            const ActivityRecord& r = *w.activity(ids[j]);
            if (!r.live()) continue;
            Watch& o = r.progress.segment == 0 ? first[j] : second[j];
            const bool flagged = (r.constraints & kActivityNavigationPerformance) != 0;
            const double off = std::abs(r.progress.crossTrackM);
            ++o.steps;
            o.farthest = std::max(o.farthest, off);
            if (flagged) ++o.flagged;
            if (!isHold(rnp[j]) && off > rnp[j] && !flagged) ++o.missed;
            if (!isHold(rnp[j]) && off < 0.9 * rnp[j] && flagged) ++o.spurious;
            o.lastFlagged = flagged;
            o.seen = (r.constraintsSeen & kActivityNavigationPerformance) != 0;
        }
    }
    std::printf("route RNP: a C172 turning onto its leg, %.1f m off at most: at RNP 20 m said so %d of %d steps (missed %d, %d well within), then on "
                "%d steps %.2f m off at most, said so %d (missed %d, %d well within), not at its end; at RNP 5 km %d; with none %d\n",
                first[0].farthest, first[0].flagged, first[0].steps, first[0].missed, first[0].spurious, second[0].steps, second[0].farthest,
                second[0].flagged, second[0].missed, second[0].spurious, first[1].flagged + second[1].flagged, first[2].flagged + second[2].flagged);
    CHECK(first[0].farthest > 20.0);
    CHECK(first[0].flagged > 0);
    for (const Watch* o : {&first[0], &second[0]}) {
        CHECK(o->missed == 0);   // (every step that ended off by more than it, said so...)
        CHECK(o->spurious == 0); // (...and none that ended well within it)
    }
    CHECK_FALSE(second[0].lastFlagged); // (back within it: this step's constraints clear...)
    CHECK(second[0].seen);              // (...and it is remembered)
    CHECK(first[1].flagged + second[1].flagged == 0);
    CHECK(first[2].flagged + second[2].flagged == 0);
    CHECK_FALSE(second[1].seen);
    CHECK_FALSE(second[2].seen);
    // refused, naming the point: an RNP of 0, below it, not finite
    const auto& s = *w.vehicleState(v[0]);
    for (const double bad : {0.0, -5.0, std::numeric_limits<double>::infinity()}) {
        Waypoint a = at(s.latitudeRad, s.longitudeRad, 0.0, 3000.0), b = at(s.latitudeRad, s.longitudeRad, 0.0, 8000.0);
        b.rnpM = bad;
        const CommandResult r = w.submit(v[0], RouteCommand{}, std::vector<Waypoint>{a, b});
        INFO("rnp " << bad);
        CHECK((r.reason == Reason::InvalidWaypoint && r.index == 1));
    }
    const SupportInfo* row = w.supportTable(v[0])->find("fsim.guidance.route/required_navigation_performance");
    REQUIRE(row != nullptr);
    CHECK(row->support == Support::Supported);
}

TEST_CASE("route RNP: a fly-by's arc is on its path; a rotorcraft past a fly-over corner off its next leg says so", "[modes]") {
    session::World w(options("route-rnp-corners"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    const auto quad = rotor(w, "iris", 15.0, 6);
    // a C172 east 3 km, a fly-by turn there north, 4 km on - an RNP of 30 m on both segments
    const auto& s0 = *w.vehicleState(v);
    Waypoint a = at(s0.latitudeRad, s0.longitudeRad, 0.0, 3000.0), b = at(s0.latitudeRad, s0.longitudeRad, 4000.0, 3000.0);
    a.rnpM = b.rnpM = 30.0;
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{a, b});
    REQUIRE(r.accepted());
    // an IRIS north 100 m, flown over, then east 100 m at 5 m/s over the ground: 0.5 m on its second leg
    const auto& q0 = *w.vehicleState(quad);
    Waypoint p = at(q0.latitudeRad, q0.longitudeRad, 100.0, 0.0), e = at(q0.latitudeRad, q0.longitudeRad, 100.0, 100.0);
    p.speed = 5.0, p.speedReference = static_cast<double>(SpeedReference::GroundSpeed), p.turn = static_cast<double>(TurnType::FlyOver);
    e.rnpM = 0.5;
    const CommandResult rq = w.submit(quad, RouteCommand{}, std::vector<Waypoint>{p, e});
    REQUIRE(rq.accepted());
    Watch whole, rotorFirst, rotorSecond;
    for (unsigned k = 0; k < stepsFor(w, 150.0); ++k) {
        w.step();
        const ActivityRecord& rr = *w.activity(r.activity);
        if (rr.live()) {
            const bool flagged = (rr.constraints & kActivityNavigationPerformance) != 0;
            whole.farthest = std::max(whole.farthest, std::abs(rr.progress.crossTrackM));
            whole.flagged += flagged ? 1 : 0, ++whole.steps;
        }
        const ActivityRecord& rq2 = *w.activity(rq.activity);
        if (rq2.live()) {
            Watch& o = rq2.progress.segment == 0 ? rotorFirst : rotorSecond;
            const bool flagged = (rq2.constraints & kActivityNavigationPerformance) != 0;
            const double off = std::abs(rq2.progress.crossTrackM);
            ++o.steps, o.farthest = std::max(o.farthest, off), o.flagged += flagged ? 1 : 0;
            if (&o == &rotorSecond && off > 0.5 && !flagged) ++o.missed;
            if (&o == &rotorSecond && off < 0.45 && flagged) ++o.spurious;
        }
    }
    std::printf("route RNP: a C172's fly-by, %.2f m off its path at most (RNP 30 m), said so %d of %d steps; an IRIS past its fly-over corner %.2f m off "
                "its next leg at most (RNP 0.5 m), said so %d of %d steps (missed %d, %d well within); before it, %d\n",
                whole.farthest, whole.flagged, whole.steps, rotorSecond.farthest, rotorSecond.flagged, rotorSecond.steps, rotorSecond.missed,
                rotorSecond.spurious, rotorFirst.flagged);
    CHECK(whole.farthest < 30.0);
    CHECK(whole.flagged == 0);
    CHECK(rotorSecond.farthest > 0.5);
    CHECK(rotorSecond.flagged > 0);
    CHECK(rotorSecond.missed == 0);
    CHECK(rotorSecond.spurious == 0);
    CHECK(rotorFirst.flagged == 0); // (its first segment has none)
}
