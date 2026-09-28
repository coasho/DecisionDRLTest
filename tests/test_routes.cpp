// The Vehicle Interface's waypoint following, flown (docs/vehicle-interface.md,
// 4.5 and 4.8): routes of great circles and rhumb lines with fly-by and
// fly-over points, on a stock wing, a direct and a fly-by-wire design, a
// helicopter and a multirotor, calm and in a crosswind - measured against
// geometry computed here with the classic spherical formulas, apart from the
// platform's vectors - and what a mission autonomy relies on: the altitude
// and speed profiles, repeats and ends, progress point by point, UPDATE
// mid-route, and the waypoint a rejection is about.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// A waypoint `north` and `east` metres from (lat0, lon0).
Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint w;
    w.latitudeRad = lat0 + north / kR;
    w.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return w;
}

double bearing(double lat1, double lon1, double lat2, double lon2) {
    const double y = std::sin(lon2 - lon1) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lon2 - lon1);
    return std::atan2(y, x);
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    const double a = std::pow(std::sin(0.5 * (lat2 - lat1)), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(0.5 * (lon2 - lon1)), 2);
    return 2.0 * kR * std::asin(std::min(1.0, std::sqrt(a)));
}

double distance(const sim::VehicleState& s, const Waypoint& w) { return distance(s.latitudeRad, s.longitudeRad, w.latitudeRad, w.longitudeRad); }

/// Against the great circle from 1 to 2: the cross-track distance, + right,
/// and the along-track one from 1 (the aviation formulary's).
struct Track {
    double cross, along;
};
Track against(double lat1, double lon1, double lat2, double lon2, double lat, double lon) {
    const double d13 = distance(lat1, lon1, lat, lon) / kR;
    const double t13 = bearing(lat1, lon1, lat, lon), t12 = bearing(lat1, lon1, lat2, lon2);
    const double xt = std::asin(std::sin(d13) * std::sin(t13 - t12));
    const double at = std::acos(std::clamp(std::cos(d13) / std::cos(xt), -1.0, 1.0));
    return {xt * kR, (std::cos(t13 - t12) < 0.0 ? -at : at) * kR};
}

/// What a flight along a route did.
struct Flight {
    ActivityId activity = 0;
    double seconds = -1.0;           ///< to completion (-1: it did not complete)
    std::vector<double> legMiddle;   ///< per segment: the most cross-track in the middle of its leg (0.3 to 0.7 of it), m
    std::vector<double> ownCross;    ///< per segment: the most cross-track the route reported
    std::vector<double> slowest;     ///< per segment: the least ground speed
    std::vector<double> closest;     ///< per waypoint: the closest the aircraft came
    std::vector<double> altitudeAt;  ///< per waypoint: its altitude when it came closest
    std::vector<std::uint32_t> segments; ///< the segments progress went through, in order
};

/// Fly `route` from the vehicle's position until it completes or `limitS` passes.
Flight fly(session::World& w, std::uint32_t v, const std::vector<Waypoint>& route, const RouteCommand& options, double limitS) {
    const sim::VehicleState s0 = *w.vehicleState(v);
    const CommandResult r = w.submit(v, options, route);
    INFO("the route: " << reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Flight f;
    f.activity = r.activity;
    const std::size_t n = route.size();
    f.legMiddle.assign(n, 0.0), f.ownCross.assign(n, 0.0), f.slowest.assign(n, 1e9);
    f.closest.assign(n, 1e12), f.altitudeAt.assign(n, 0.0);
    const double t0 = w.simTime();
    const unsigned steps = stepsFor(w, limitS);
    for (unsigned k = 0; k < steps; ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        const ActivityRecord& a = *w.activity(r.activity);
        for (std::size_t i = 0; i < n; ++i)
            if (const double d = distance(s, route[i]); d < f.closest[i]) f.closest[i] = d, f.altitudeAt[i] = s.altitudeMslM;
        const std::uint32_t seg = std::min<std::uint32_t>(a.progress.segment, static_cast<std::uint32_t>(n - 1));
        if (f.segments.empty() || f.segments.back() != seg) f.segments.push_back(seg);
        const double lat1 = seg == 0 ? s0.latitudeRad : route[seg - 1].latitudeRad, lon1 = seg == 0 ? s0.longitudeRad : route[seg - 1].longitudeRad;
        const Track t = against(lat1, lon1, route[seg].latitudeRad, route[seg].longitudeRad, s.latitudeRad, s.longitudeRad);
        const double fraction = t.along / distance(lat1, lon1, route[seg].latitudeRad, route[seg].longitudeRad);
        if (fraction > 0.3 && fraction < 0.7) f.legMiddle[seg] = std::max(f.legMiddle[seg], std::abs(t.cross));
        f.ownCross[seg] = std::max(f.ownCross[seg], std::abs(a.progress.crossTrackM));
        f.slowest[seg] = std::min(f.slowest[seg], groundSpeed(s));
        if (a.state == ActivityState::Completed) {
            f.seconds = w.simTime() - t0;
            break;
        }
    }
    return f;
}

/// A wing flying on east at its speed and height for `seconds` (an hsa): its guidance has seen the wind.
void settle(session::World& w, std::uint32_t v, double seconds) {
    const auto& s = *w.vehicleState(v);
    HsaCommand east;
    east.headingRad = 0.5 * kPi, east.speed = s.airspeedTrueMs, east.speedReference = 0.0, east.altitudeM = s.altitudeMslM;
    REQUIRE(w.submit(v, east).accepted());
    w.step(stepsFor(w, seconds));
}

/// The route every class flies, `leg` metres a side from where the aircraft
/// is, flying east: a 90 degree fly-by left at 0, one right at 1, 51 degrees
/// right at 2, 78 degrees right flown over at 3, and on to 4.
std::vector<Waypoint> zigzag(const sim::VehicleState& s, double leg, double altitudeM, double speed, SpeedReference reference) {
    const double lat0 = s.latitudeRad, lon0 = s.longitudeRad;
    std::vector<Waypoint> route = {at(lat0, lon0, 0, leg), at(lat0, lon0, leg, leg), at(lat0, lon0, leg, 2 * leg), at(lat0, lon0, 0, 2.8 * leg),
                                   at(lat0, lon0, -leg, 2 * leg)};
    for (auto& p : route) p.altitudeM = altitudeM, p.speed = speed, p.speedReference = static_cast<double>(reference);
    route[3].turn = static_cast<double>(TurnType::FlyOver);
    return route;
}

double code(SpeedReference r) { return static_cast<double>(r); }

/// A wing at `latitudeDeg`, flying east at its altitude and speed; `index` spaces them 3 km apart north.
std::uint32_t wingAt(session::World& w, const std::string& type, double latitudeDeg, double altitudeM, double tasMs, int index) {
    session::VehicleSpec s;
    s.name = type + "-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg = latitudeDeg + 0.03 * index;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

} // namespace

TEST_CASE("route: every class flies its legs, fly-by turns and a fly-over point, calm and in a crosswind", "[modes]") {
    struct Aircraft {
        const char* type;
        bool rotor;
        double altitudeM, speedMs, windMs;
        double onLeg; ///< the cross-track bound once on a leg, m
    };
    // a 12 m/s crosswind; the multirotor's 5 m/s (it cruises at 4)
    for (const Aircraft a : {Aircraft{"c172x", false, 1500.0, 55.0, 12.0, 50.0}, Aircraft{"b52h", false, 3000.0, 180.0, 12.0, 50.0},
                             Aircraft{"f16c", false, 3000.0, 160.0, 12.0, 50.0}, Aircraft{"uh60", true, 100.0, 20.0, 12.0, 15.0},
                             Aircraft{"iris", true, 100.0, 5.0, 5.0, 3.0}}) {
        for (const double wind : {0.0, a.windMs}) {
            INFO(a.type << ", wind " << wind);
            session::World w(options("routes-classes"));
            if (wind > 0.0) setWind(w, 0.0, wind); // from the north: across the first leg
            const auto v = a.rotor ? rotor(w, a.type, 20.0) : wing(w, a.type, a.altitudeM, a.speedMs);
            if (!a.rotor) settle(w, v, 5.0);
            // the turn radius the route plans with: the speed plus the wind, 80 % of the bank
            const double radius = w.performance(v)->turnRadiusM(a.speedMs + (a.rotor ? 0.0 : wind));
            const double leg = 5.0 * radius;
            const auto route = zigzag(*w.vehicleState(v), leg, a.altitudeM, a.speedMs, a.rotor ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
            const Flight f = fly(w, v, route, RouteCommand{}, 12.0 * leg / a.speedMs + 120.0);
            std::printf("route %-6s wind %4.1f: %6.0f s, legs %5.1f %5.1f %5.1f %5.1f m, turns %6.1f m (R %6.0f), closest %6.0f %6.0f %6.0f %6.0f %6.0f, slowest %5.1f\n",
                        a.type, wind, f.seconds, f.legMiddle[0], f.legMiddle[1], f.legMiddle[2], f.legMiddle[3],
                        std::max({f.ownCross[1], f.ownCross[2], f.ownCross[3]}), radius, f.closest[0], f.closest[1], f.closest[2], f.closest[3],
                        f.closest[4], std::min({f.slowest[1], f.slowest[2], f.slowest[3]}));
            CHECK(f.seconds > 0.0); // completed after its last point
            CHECK(f.segments == std::vector<std::uint32_t>{0, 1, 2, 3, 4}); // every point, in order
            // once on a leg (its middle; the one after the fly-over is an intercept)
            for (std::size_t i = 1; i < 4; ++i) CHECK(f.legMiddle[i] < a.onLeg);
            // a fly-by turn passes its point where its arc does, R (1/cos(a/2) - 1), and overshoots it by less than R
            for (const auto& [i, turn] : {std::pair<std::size_t, double>{0, 90.0}, {1, 90.0}, {2, 51.3}})
                CHECK(std::abs(f.closest[i] - radius * (1.0 / std::cos(0.5 * turn * kDeg) - 1.0)) < std::max(a.onLeg, 0.08 * radius));
            CHECK(std::max({f.ownCross[1], f.ownCross[2], f.ownCross[3]}) < radius);
            CHECK(f.closest[3] < 2.0 * a.onLeg); // flown over
            CHECK(f.closest[4] < 2.0 * a.onLeg); // the last point, passed abeam
            // a rotorcraft does not stop at its points: through the turns at its speed, less what the wind takes
            if (a.rotor) CHECK(std::min({f.slowest[1], f.slowest[2], f.slowest[3]}) > 0.8 * a.speedMs);
        }
    }
}

TEST_CASE("route: the altitude runs from point to point or climbs at a rate, the speed is each segment's in its reference", "[modes]") {
    session::World w(options("routes-profiles"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    settle(w, v, 5.0);
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
    std::vector<Waypoint> route = {at(lat0, lon0, 0, 3000), at(lat0, lon0, 0, 6000), at(lat0, lon0, 8000, 6000), at(lat0, lon0, 8000, 9000)};
    route[0].altitudeM = 1600.0, route[0].speed = 55.0, route[0].speedReference = code(SpeedReference::TrueAirspeed);
    route[1].altitudeM = 1700.0, route[1].speed = 50.0, route[1].speedReference = code(SpeedReference::CalibratedAirspeed);
    route[2].altitudeM = 1900.0, route[2].climbRateMs = 2.0;                                  // at 2 m/s, then level; the speed as before
    route[3].speed = 60.0, route[3].speedReference = code(SpeedReference::GroundSpeed);         // the altitude as before
    const CommandResult r = w.submit(v, RouteCommand{}, route);
    REQUIRE(r.accepted());
    std::vector<double> closest(route.size(), 1e12), altitude(route.size(), 0.0);
    double climbSum = 0.0, cas = 0.0, gs = 0.0;
    int climbN = 0, casN = 0, gsN = 0;
    const unsigned steps = stepsFor(w, 600.0);
    double t2 = -1.0; // when the climb-rate segment began
    for (unsigned k = 0; k < steps && w.activity(r.activity)->live(); ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        const ActivityProgress& p = w.activity(r.activity)->progress;
        for (std::size_t i = 0; i < route.size(); ++i)
            if (const double d = distance(s, route[i]); d < closest[i]) closest[i] = d, altitude[i] = s.altitudeMslM;
        if (p.segment == 2 && t2 < 0.0) t2 = w.simTime();
        if (p.segment == 2 && w.simTime() > t2 + 15.0 && w.simTime() < t2 + 60.0) climbSum += -s.velocityNedMs[2], ++climbN; // well inside its 100 s climb
        if (p.segment == 1 && p.segmentPercent > 40.0) cas += s.airspeedCalibratedMs, ++casN;
        if (p.segment == 3 && p.segmentPercent > 40.0) gs += groundSpeed(s), ++gsN;
        if (p.segment == 1) CHECK((p.speedMs == 50.0 && p.speedReference == code(SpeedReference::CalibratedAirspeed)));
        if (p.segment == 3) CHECK((p.speedMs == 60.0 && p.speedReference == code(SpeedReference::GroundSpeed) && p.altitudeMslM == 1900.0));
    }
    REQUIRE(w.activity(r.activity)->state == ActivityState::Completed);
    REQUIRE((climbN > 0 && casN > 0 && gsN > 0));
    std::printf("route profiles: altitudes at the points %.0f %.0f %.0f %.0f, climb %.2f m/s, CAS %.2f, GS %.2f\n", altitude[0], altitude[1],
                altitude[2], altitude[3], climbSum / climbN, cas / casN, gs / gsN);
    CHECK(std::abs(altitude[0] - 1600.0) < 15.0); // straight from where it was to each point
    CHECK(std::abs(altitude[1] - 1700.0) < 15.0);
    CHECK(std::abs(climbSum / climbN - 2.0) < 0.3); // at the rate asked
    CHECK(std::abs(altitude[2] - 1900.0) < 15.0);   // and level at the point
    CHECK(std::abs(cas / casN - 50.0) < 1.5);
    CHECK(std::abs(gs / gsN - 60.0) < 1.5);

    // a gradient steeper than it climbs: refused, or flown at its best rate (clamped)
    const auto& s1 = *w.vehicleState(v);
    std::vector<Waypoint> steep = {at(s1.latitudeRad, s1.longitudeRad, 0, 2000)};
    steep[0].altitudeM = s1.altitudeMslM + 1000.0;
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    CommandResult refused = w.submit(v, RouteCommand{}, steep, reject);
    CHECK(refused.reason == Reason::PerformanceLimit);
    CHECK(refused.index == 0);
    CHECK(refused.constraint == Constraint::MaxClimbRate);
    const CommandResult clamped = w.submit(v, RouteCommand{}, steep);
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped) != 0);
    CHECK(clamped.index == 0);
    CHECK(clamped.constraint == Constraint::MaxClimbRate);
    w.step(stepsFor(w, 20.0));
    CHECK(-w.vehicleState(v)->velocityNedMs[2] > 0.8 * w.performance(v)->maxClimbMs); // climbing at its best
}

TEST_CASE("route: a great circle bows to the pole, a rhumb line holds its course", "[modes]") {
    session::World w(options("routes-projection"));
    const auto great = wingAt(w, "f16c", 60.0, 3000.0, 160.0, 0), rhumb = wingAt(w, "f16c", 60.0, 3000.0, 160.0, 1);
    settle(w, great, 3.0);
    settle(w, rhumb, 3.0);
    const sim::VehicleState g0 = *w.vehicleState(great), r0 = *w.vehicleState(rhumb);
    // 120 km due east along a parallel at 60 degrees north: the great circle
    // between its ends reaches L^2 tan(lat) / 8R north of the parallel
    const double length = 120000.0, bow = length * length * std::tan(g0.latitudeRad) / (8.0 * kR);
    const Waypoint gEnd = at(g0.latitudeRad, g0.longitudeRad, 0.0, length), rEnd = at(r0.latitudeRad, r0.longitudeRad, 0.0, length);
    RouteCommand onRhumb;
    onRhumb.projection = static_cast<double>(Projection::Rhumb);
    const ActivityId a = w.submit(great, RouteCommand{}, std::vector<Waypoint>{gEnd}).activity, b = w.submit(rhumb, onRhumb, std::vector<Waypoint>{rEnd}).activity;
    REQUIRE((a != 0 && b != 0));
    double north = 0.0, offRhumb = 0.0, offGreat = 0.0, firstTrack = kHold, lastTrack = kHold, rhumbCourse = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 800.0) && (w.activity(a)->live() || w.activity(b)->live()); ++k) {
        w.step();
        const auto& g = *w.vehicleState(great);
        const auto& r = *w.vehicleState(rhumb);
        if (w.activity(a)->live()) {
            north = std::max(north, (g.latitudeRad - g0.latitudeRad) * kR);
            const Track t = against(g0.latitudeRad, g0.longitudeRad, gEnd.latitudeRad, gEnd.longitudeRad, g.latitudeRad, g.longitudeRad);
            if (t.along > 10000.0 && t.along < length - 5000.0) {
                offGreat = std::max(offGreat, std::abs(t.cross));
                if (isHold(firstTrack)) firstTrack = track(g);
                lastTrack = track(g);
            }
        }
        if (w.activity(b)->live() && (r.longitudeRad - r0.longitudeRad) * kR * std::cos(r0.latitudeRad) > 10000.0) {
            offRhumb = std::max(offRhumb, std::abs(r.latitudeRad - r0.latitudeRad) * kR);
            rhumbCourse = std::max(rhumbCourse, std::abs(degreesApart(w.activity(b)->progress.courseRad, 0.5 * kPi)));
        }
    }
    CHECK(w.activity(a)->state == ActivityState::Completed);
    CHECK(w.activity(b)->state == ActivityState::Completed);
    std::printf("route projections: great circle %.0f m north (%.0f by the formula), %.1f m off it, track %.2f to %.2f deg; rhumb %.1f m off its parallel\n",
                north, bow, offGreat, firstTrack / kDeg, lastTrack / kDeg, offRhumb);
    CHECK(std::abs(north - bow) < 40.0);
    CHECK(offGreat < 30.0);
    CHECK(firstTrack / kDeg < 89.5); // its course turns through east as the meridians close
    CHECK(lastTrack / kDeg > 90.5);
    CHECK(offRhumb < 30.0);
    CHECK(rhumbCourse < 1.0); // a rhumb line's course is one
}

TEST_CASE("route: a route that repeats flies laps; one that ends continues, orbits or hovers", "[modes]") {
    session::World w(options("routes-ends"));
    // a triangle, round and round
    const auto lapper = wing(w, "c172x", 1500.0, 55.0, 0);
    settle(w, lapper, 3.0);
    const auto& s = *w.vehicleState(lapper);
    RouteCommand repeat;
    repeat.repeat = 1.0;
    const std::vector<Waypoint> triangle = {at(s.latitudeRad, s.longitudeRad, 0, 3000), at(s.latitudeRad, s.longitudeRad, 3000, 4500),
                                            at(s.latitudeRad, s.longitudeRad, 1500, 0)};
    const ActivityId laps = w.submit(lapper, repeat, triangle).activity;
    REQUIRE(laps != 0);
    // a wing that loiters at the end orbits its last point; a rotorcraft stops and hovers over it; one that continues flies on
    const auto orbiter = wing(w, "c172x", 1500.0, 55.0, 3);
    settle(w, orbiter, 3.0);
    const auto& o = *w.vehicleState(orbiter);
    RouteCommand loiter;
    loiter.end = static_cast<double>(EndBehavior::Loiter);
    const std::vector<Waypoint> out = {at(o.latitudeRad, o.longitudeRad, 0, 3000), at(o.latitudeRad, o.longitudeRad, 3000, 3000)};
    const ActivityId orbit = w.submit(orbiter, loiter, out).activity;
    const auto hoverer = rotor(w, "iris", 15.0, 6), goer = rotor(w, "uh60", 15.0, 9);
    const auto& h = *w.vehicleState(hoverer);
    const std::vector<Waypoint> hop = {at(h.latitudeRad, h.longitudeRad, 0, 40), at(h.latitudeRad, h.longitudeRad, 40, 40)};
    const ActivityId hover = w.submit(hoverer, loiter, hop).activity;
    const auto& g = *w.vehicleState(goer);
    const std::vector<Waypoint> dash = {at(g.latitudeRad, g.longitudeRad, 0, 500), at(g.latitudeRad, g.longitudeRad, 500, 500)};
    const ActivityId go = w.submit(goer, RouteCommand{}, dash).activity;
    REQUIRE((orbit != 0 && hover != 0 && go != 0));

    std::uint32_t lastSegment = 0, wraps = 0;
    double orbitMin = 1e9, orbitMax = 0.0, done = -1.0;
    for (unsigned k = 0; k < stepsFor(w, 900.0); ++k) {
        w.step();
        const ActivityRecord& lap = *w.activity(laps);
        REQUIRE(lap.live()); // it never completes
        CHECK(std::isnan(lap.progress.distanceToGoM));
        CHECK((lap.progress.percent >= 0.0 && lap.progress.percent <= 100.0));
        if (lap.progress.segment < lastSegment) ++wraps;
        lastSegment = lap.progress.segment;
        const ActivityRecord& orbited = *w.activity(orbit);
        if (!orbited.live() && done < 0.0) done = w.simTime();
        if (done > 0.0 && w.simTime() > done + 120.0) { // settled in its orbit
            const double d = distance(*w.vehicleState(orbiter), out[1]);
            orbitMin = std::min(orbitMin, d), orbitMax = std::max(orbitMax, d);
        }
    }
    const double radius = w.performance(orbiter)->turnRadiusM(55.0);
    std::printf("route ends: %u laps (%u wraps); orbit %.0f to %.0f m (R %.0f); iris %.2f m from its point at %.2f m/s; uh60 on at %.1f deg, %.1f m/s\n",
                w.activity(laps)->progress.laps, wraps, orbitMin, orbitMax, radius, distance(*w.vehicleState(hoverer), hop[1]),
                groundSpeed(*w.vehicleState(hoverer)), track(*w.vehicleState(goer)) / kDeg, groundSpeed(*w.vehicleState(goer)));
    CHECK(w.activity(laps)->progress.laps >= 2);
    CHECK(w.activity(laps)->progress.laps == wraps);
    CHECK(w.activity(orbit)->state == ActivityState::Completed); // at its last point, then on as its end says
    CHECK(w.activity(orbit)->reason == Reason::GoalReached);
    CHECK(orbitMin > 0.8 * radius);
    CHECK(orbitMax < 1.25 * radius);
    CHECK(std::abs(w.activity(orbit)->progress.crossTrackM) < 50.0); // off its last leg when it ended: not the orbit's radius
    CHECK(w.activity(hover)->state == ActivityState::Completed);
    CHECK(distance(*w.vehicleState(hoverer), hop[1]) < 2.0);
    CHECK(groundSpeed(*w.vehicleState(hoverer)) < 0.3);
    CHECK(w.activity(go)->state == ActivityState::Completed);
    CHECK(std::abs(degreesApart(track(*w.vehicleState(goer)), 0.0)) < 3.0); // the last leg's course, north
    CHECK(std::abs(groundSpeed(*w.vehicleState(goer)) - w.performance(goer)->cruiseTasMs) < 1.0); // at the speed it was given: its cruise
}

TEST_CASE("route: progress point by point, an UPDATE mid-route flies the new route from where it is", "[modes]") {
    session::World w(options("routes-progress"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    settle(w, v, 3.0);
    const auto& s0 = *w.vehicleState(v);
    std::vector<Waypoint> route = {at(s0.latitudeRad, s0.longitudeRad, 0, 3000), at(s0.latitudeRad, s0.longitudeRad, 3000, 3000),
                                   at(s0.latitudeRad, s0.longitudeRad, 3000, 6000)};
    for (std::size_t i = 0; i < route.size(); ++i) route[i].id = 101 + i;
    const CommandResult r = w.submit(v, RouteCommand{}, route);
    REQUIRE(r.accepted());
    double percent = 0.0, toGo = 1e12;
    std::uint32_t segment = 0;
    bool timed = false;
    for (unsigned k = 0; k < stepsFor(w, 400.0) && w.activity(r.activity)->live(); ++k) {
        w.step();
        const ActivityProgress& p = w.activity(r.activity)->progress;
        INFO("t " << w.simTime());
        CHECK(p.segments == 3);
        CHECK(p.segment >= segment); // on, point by point
        segment = p.segment;
        CHECK(p.segmentId == 101 + p.segment);
        CHECK(p.percent >= percent - 0.2); // (within a step's jitter of the path it reads)
        percent = std::max(percent, p.percent);
        CHECK(p.distanceToGoM <= toGo + 10.0);
        toGo = std::min(toGo, p.distanceToGoM);
        if (p.timeToGoS > 0.0) timed = std::abs(p.timeToGoS - p.distanceToGoM / groundSpeed(*w.vehicleState(v))) < 1.0;
    }
    const ActivityRecord& done = *w.activity(r.activity);
    CHECK(done.state == ActivityState::Completed);
    CHECK(done.reason == Reason::GoalReached);
    CHECK(done.progress.segment == 2);
    CHECK(done.progress.percent == 100.0);
    CHECK(done.progress.distanceToGoM == 0.0);
    CHECK(timed);

    // UPDATE mid-route: the new route flown from where the aircraft is, the same activity
    const auto& s1 = *w.vehicleState(v);
    std::vector<Waypoint> first = {at(s1.latitudeRad, s1.longitudeRad, 0, 3000), at(s1.latitudeRad, s1.longitudeRad, 0, 6000),
                                   at(s1.latitudeRad, s1.longitudeRad, 0, 9000), at(s1.latitudeRad, s1.longitudeRad, 0, 12000)};
    const CommandResult a = w.submit(v, RouteCommand{}, first);
    REQUIRE(a.accepted());
    while (w.activity(a.activity)->progress.segment < 1) w.step();
    const auto& s2 = *w.vehicleState(v);
    const std::vector<Waypoint> north = {at(s2.latitudeRad, s2.longitudeRad, 3000, 0), at(s2.latitudeRad, s2.longitudeRad, 6000, 0)};
    const CommandResult updated = w.update(a.activity, RouteCommand{}, north);
    REQUIRE(updated.accepted());
    CHECK(updated.activity == a.activity);
    w.step();
    CHECK(w.activity(a.activity)->progress.segments == 2);
    CHECK(w.activity(a.activity)->progress.segment == 0);
    CHECK(w.activity(a.activity)->progress.percent < 5.0);
    // a waypoint it cannot fly is refused, naming it; the route flies on as it was
    std::vector<Waypoint> bad = north;
    bad[1].latitudeRad = std::numeric_limits<double>::quiet_NaN();
    const CommandResult refused = w.update(a.activity, RouteCommand{}, bad);
    CHECK(refused.reason == Reason::InvalidWaypoint);
    CHECK(refused.index == 1);
    for (unsigned k = 0; k < stepsFor(w, 200.0) && w.activity(a.activity)->live(); ++k) w.step();
    CHECK(w.activity(a.activity)->state == ActivityState::Completed);
    CHECK(distance(*w.vehicleState(v), north[1]) < 100.0); // passed abeam, near

    // an UPDATE of its options alone keeps its waypoints: flown again, from the point it names
    const CommandResult b = w.submit(v, RouteCommand{}, first);
    REQUIRE(b.accepted());
    w.step();
    RouteCommand from2;
    from2.projection = from2.repeat = from2.end = kHold, from2.start = 2.0;
    REQUIRE(w.update(b.activity, from2).accepted()); // (through the Command overload: the same as with no waypoints)
    w.step();
    CHECK(w.activity(b.activity)->progress.segment == 2);
    CHECK(w.activity(b.activity)->progress.segments == 4);
    RouteCommand beyond = from2;
    beyond.start = 4.0;
    const CommandResult out = w.update(b.activity, beyond, {});
    CHECK(out.reason == Reason::InvalidParameter);
    CHECK(out.index == 3);
}

TEST_CASE("route: what cannot be flown is refused, naming the waypoint; what can be made to fit is clamped", "[modes]") {
    session::World w(options("routes-checks"));
    const auto c172 = wing(w, "c172x", 1500.0, 55.0, 0), f16 = wing(w, "f16c", 3000.0, 160.0, 2), iris = rotor(w, "iris", 0.0, 4);
    const auto& s = *w.vehicleState(c172);
    const double lat0 = s.latitudeRad, lon0 = s.longitudeRad;
    const std::vector<Waypoint> good = {at(lat0, lon0, 0, 3000), at(lat0, lon0, 3000, 3000), at(lat0, lon0, 3000, 6000)};
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    auto refuse = [&](std::uint32_t v, std::vector<Waypoint> route, const RouteCommand& c, Reason reason, int index, Constraint constraint = Constraint::None) {
        const CommandResult r = w.submit(v, c, route, reject);
        INFO(reasonName(r.reason) << " at " << r.index << ", " << constraintName(r.constraint));
        CHECK(r.reason == reason);
        CHECK(r.index == index);
        CHECK(r.constraint == constraint);
    };
    auto with = [&](std::size_t i, auto change) {
        std::vector<Waypoint> route = good;
        change(route[i]);
        return route;
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // the waypoints themselves
    refuse(c172, {}, RouteCommand{}, Reason::InvalidWaypoint, 0);
    CHECK(w.submit(c172, Command(RouteCommand{})).reason == Reason::InvalidWaypoint); // a route's command alone has none
    refuse(c172, std::vector<Waypoint>(PathStore::kWaypoints + 1, good[0]), RouteCommand{}, Reason::InvalidWaypoint, static_cast<int>(PathStore::kWaypoints));
    refuse(c172, with(2, [&](Waypoint& p) { p.latitudeRad = nan; }), RouteCommand{}, Reason::InvalidWaypoint, 2);
    refuse(c172, with(1, [](Waypoint& p) { p.latitudeRad = 2.0; }), RouteCommand{}, Reason::InvalidWaypoint, 1);
    refuse(c172, with(1, [](Waypoint& p) { p.turn = 0.5; }), RouteCommand{}, Reason::InvalidWaypoint, 1);
    refuse(c172, with(0, [](Waypoint& p) { p.speedReference = 7.0; }), RouteCommand{}, Reason::InvalidWaypoint, 0);
    refuse(c172, with(2, [](Waypoint& p) { p.speed = -1.0; }), RouteCommand{}, Reason::InvalidWaypoint, 2);
    refuse(c172, with(1, [](Waypoint& p) { p.maxBankRad = 2.0; }), RouteCommand{}, Reason::InvalidWaypoint, 1);
    refuse(c172, with(1, [](Waypoint& p) { p.climbRateMs = 0.0; }), RouteCommand{}, Reason::InvalidWaypoint, 1);
    refuse(c172, with(1, [&](Waypoint& p) { p = good[0]; }), RouteCommand{}, Reason::InvalidWaypoint, 1); // the same place twice
    refuse(c172, with(1, [](Waypoint& p) { p.altitudeReference = static_cast<double>(AltitudeReference::AboveGround); }), RouteCommand{},
           Reason::InvalidWaypoint, 1); // a reference alone, with no altitude of the point before's in it
    // its options
    RouteCommand c;
    c.projection = 2.0;
    refuse(c172, good, c, Reason::InvalidParameter, 0);
    c = RouteCommand{}, c.repeat = 0.5;
    refuse(c172, good, c, Reason::InvalidParameter, 1);
    c = RouteCommand{}, c.end = 3.0;
    refuse(c172, good, c, Reason::InvalidParameter, 2);
    c = RouteCommand{}, c.start = 3.0;
    refuse(c172, good, c, Reason::InvalidParameter, 3);
    c = RouteCommand{}, c.repeat = 1.0;
    refuse(c172, {good[0]}, c, Reason::InvalidParameter, 1); // round one point is a pattern
    // what the aircraft cannot do, the point and the limit named
    refuse(f16, with(1, [](Waypoint& p) { p.speed = 600.0, p.speedReference = 0.0; }), RouteCommand{}, Reason::PerformanceLimit, 1, Constraint::MaxAirspeed);
    refuse(f16, with(2, [](Waypoint& p) { p.altitudeM = 25000.0; }), RouteCommand{}, Reason::PerformanceLimit, 2, Constraint::MaxAltitude);
    refuse(c172, with(0, [](Waypoint& p) { p.maxBankRad = 1.2; }), RouteCommand{}, Reason::PerformanceLimit, 0, Constraint::MaxOrientation);
    refuse(c172, with(1, [](Waypoint& p) { p.altitudeM = 1800.0, p.climbRateMs = 30.0; }), RouteCommand{}, Reason::PerformanceLimit, 1,
           Constraint::MaxClimbRate);
    // (a rotorcraft given no speed: its cruise; 30 m from where it is - the C172's point, 13 km off, is further than its battery lasts)
    const auto& hover = *w.vehicleState(iris);
    refuse(iris, {at(hover.latitudeRad, hover.longitudeRad, 0, 30)}, RouteCommand{}, Reason::None, -1);
    std::vector<Waypoint> fast = {at(hover.latitudeRad, hover.longitudeRad, 0, 30)};
    fast[0].speed = 50.0, fast[0].speedReference = code(SpeedReference::GroundSpeed);
    refuse(iris, fast, RouteCommand{}, Reason::PerformanceLimit, 0, Constraint::MaxAirspeed);
    // a leg too short for the fly-by turns at its ends: refused, or the turns flown smaller
    const std::vector<Waypoint> tight = {at(lat0, lon0, 0, 3000), at(lat0, lon0, 300, 3000), at(lat0, lon0, 300, 6000)};
    refuse(c172, tight, RouteCommand{}, Reason::InvalidWaypoint, 0);
    const CommandResult fitted = w.submit(c172, RouteCommand{}, tight);
    REQUIRE(fitted.accepted());
    CHECK((fitted.flags & kClamped) != 0);
    CHECK(fitted.index == 0);
    w.step();
    CHECK((w.activity(fitted.activity)->constraints & kActivityClamped) != 0);
    // flown over, a short leg is no fault
    std::vector<Waypoint> over = tight;
    over[0].turn = over[1].turn = static_cast<double>(TurnType::FlyOver);
    CHECK(w.submit(c172, RouteCommand{}, over, reject).accepted());
}

TEST_CASE("route: every aircraft offers A-GRA's waypoint following, which takes UPDATE", "[modes]") {
    session::World w(options("routes-discovery"));
    const std::uint32_t aircraft[] = {wing(w, "c172x", 1500.0, 55.0, 0), wing(w, "b52h", 3000.0, 180.0, 2), wing(w, "f16c", 3000.0, 160.0, 4),
                                      rotor(w, "uh60", 0.0, 6), rotor(w, "iris", 0.0, 8)};
    for (const auto v : aircraft) {
        const CapabilityDescriptor* d = nullptr;
        for (const auto& c : w.capabilities(v))
            if (c.id == "fsim.guidance.route") d = &c;
        REQUIRE(d != nullptr);
        CHECK(d->mode == FlightMode::WaypointFollowing);
        CHECK(d->setpoint == SetpointKind::Route);
        CHECK(d->persistence == Persistence::Terminating);
        CHECK((d->interactions & kUpdate) != 0);
        REQUIRE(d->parameters.size() == 4);
        CHECK(d->parameters[3].name == "start");
        CHECK(d->parameters[3].max == static_cast<double>(PathStore::kWaypoints) - 1.0);
    }
}
