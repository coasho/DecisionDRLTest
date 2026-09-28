// The Vehicle Interface's loiter patterns, flown (docs/vehicle-interface.md,
// 4.6): orbits for every class calm and in a crosswind, down to a
// multirotor's 2 m; a racetrack, a figure-eight and ATC's hold, each measured
// against its own geometry computed here; a duration; UPDATE; the defaults;
// and what cannot be flown refused.
#include "fsim/GuidanceModes.h"
#include "fsim/VehicleProfile.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// Where the aircraft is from (lat0, lon0): north and east, m (the plane there).
struct Local {
    double north, east;
};
Local from(const sim::VehicleState& s, double lat0, double lon0) {
    return {(s.latitudeRad - lat0) * kR, (s.longitudeRad - lon0) * kR * std::cos(0.5 * (s.latitudeRad + lat0))};
}

/// A point along `course` and to its right, in the plane.
struct Frame {
    double course;
    double along(Local p) const { return p.north * std::cos(course) + p.east * std::sin(course); }
    double right(Local p) const { return -p.north * std::sin(course) + p.east * std::cos(course); }
};

/// How far the aircraft is off a racetrack of radius r and legs l, right turns,
/// its inbound leg ending at the fix along `course`: the racetrack is the set of
/// points r from the segment joining its turns' centres.
double offRacetrack(Local p, double course, double r, double l) {
    const Frame f{course};
    const double x = f.along(p), y = f.right(p) - r; // from the line through the centres
    const double dx = x > 0.0 ? x : x < -l ? x + l : 0.0;
    return std::abs(std::hypot(dx, y) - r);
}

/// Submit a pattern; REQUIRE it accepted.
ActivityId submit(session::World& w, std::uint32_t v, const PatternCommand& c) {
    const CommandResult r = w.submit(v, c);
    INFO("the pattern: " << reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    return r.activity;
}

double code(PatternKind k) { return static_cast<double>(k); }

/// Wings flying on east at their speed and height for `seconds` (an hsa each): what they fly "now" is settled, and they have seen the wind.
void settle(session::World& w, std::initializer_list<std::uint32_t> wings, double seconds) {
    for (const auto v : wings) {
        const auto& s = *w.vehicleState(v);
        HsaCommand east;
        east.headingRad = 0.5 * kPi, east.speed = s.airspeedTrueMs, east.speedReference = 0.0, east.altitudeM = s.altitudeMslM;
        REQUIRE(w.submit(v, east).accepted());
    }
    w.step(stepsFor(w, seconds));
}

} // namespace

TEST_CASE("pattern: every class orbits its centre at the radius asked, calm and in a crosswind, down to a multirotor's 2 m", "[modes]") {
    struct Orbit {
        const char* type;
        bool rotor;
        double altitudeM, speedMs, radiusM, windMs;
        double calm, windy; ///< off the circle once on it, calm and in the wind, m
    };
    // a 12 m/s wind across the circle: a wing's ground speed swings by twice it each lap
    for (const Orbit a : {Orbit{"c172x", false, 1500.0, 55.0, 800.0, 12.0, 5.0, 40.0}, Orbit{"b52h", false, 3000.0, 180.0, 9000.0, 12.0, 15.0, 30.0},
                          Orbit{"f16c", false, 3000.0, 160.0, 4000.0, 12.0, 10.0, 40.0}, Orbit{"uh60", true, 100.0, 20.0, 150.0, 12.0, 3.0, 8.0},
                          Orbit{"iris", true, 100.0, 4.0, 10.0, 5.0, 0.5, 1.5}, Orbit{"iris", true, 100.0, 1.0, 2.0, 0.0, 0.3, 0.3}}) {
        for (const double wind : {0.0, a.windMs}) {
            for (const double clockwise : {1.0, 0.0}) {
                INFO(a.type << " R " << a.radiusM << ", wind " << wind << (clockwise == 1.0 ? ", right" : ", left"));
                session::World w(options("patterns-orbit"));
                if (wind > 0.0) setWind(w, 0.0, wind);
                const auto v = a.rotor ? rotor(w, a.type, 15.0) : wing(w, a.type, a.altitudeM, a.speedMs);
                if (!a.rotor) settle(w, {v}, 5.0);
                const auto& s0 = *w.vehicleState(v);
                // round a point ahead, so it joins its circle from outside
                const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad + 2.0 * a.radiusM / (kR * std::cos(s0.latitudeRad));
                PatternCommand c;
                c.latitudeRad = lat0, c.longitudeRad = lon0, c.radiusM = a.radiusM, c.clockwise = clockwise;
                c.speed = a.speedMs, c.speedReference = static_cast<double>(a.rotor ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
                const ActivityId id = submit(w, v, c);
                const double lap = 2.0 * kPi * a.radiusM / a.speedMs;
                w.step(stepsFor(w, 1.5 * lap + 30.0)); // on the circle
                double off = 0.0, turned = 0.0, lastBearing = kHold;
                const std::uint32_t laps0 = w.activity(id)->progress.laps;
                for (unsigned k = 0; k < stepsFor(w, lap); ++k) {
                    w.step();
                    const Local p = from(*w.vehicleState(v), lat0, lon0);
                    off = std::max(off, std::abs(std::hypot(p.north, p.east) - a.radiusM));
                    const double bearing = std::atan2(p.east, p.north);
                    if (!isHold(lastBearing)) turned += std::remainder(bearing - lastBearing, 2.0 * kPi);
                    lastBearing = bearing;
                }
                const ActivityRecord& r = *w.activity(id);
                std::printf("pattern orbit %-6s R %6.0f: off the circle %6.2f m, %s, %s\n", a.type, a.radiusM, off, wind > 0.0 ? "windy" : "calm",
                            clockwise == 1.0 ? "right" : "left");
                const double bound = wind > 0.0 ? a.windy : a.calm;
                CHECK(r.state == ActivityState::Active);
                CHECK(off < bound);
                CHECK(std::abs(std::abs(turned) - 2.0 * kPi) < 0.3); // a lap in a lap's time
                CHECK((turned > 0.0) == (clockwise == 1.0));        // the way round it was asked
                CHECK(r.progress.laps >= laps0);
                CHECK(r.progress.laps >= 1);
                CHECK(std::abs(r.progress.crossTrackM) < bound);
            }
        }
    }
}

TEST_CASE("pattern: a racetrack and a hold fly their legs and half circles; a figure-eight its two circles", "[modes]") {
    session::World w(options("patterns-shapes"));
    setWind(w, 0.0, 8.0);
    const auto racer = wing(w, "c172x", 1500.0, 55.0, 0), hold = wing(w, "c172x", 1500.0, 55.0, 3), eight = wing(w, "c172x", 1500.0, 55.0, 6);
    settle(w, {racer, hold, eight}, 5.0);
    session::World calm(options("patterns-small")); // (a multirotor that cruises at 4 m/s, in no wind)
    const auto quad = rotor(calm, "iris", 15.0, 9);
    auto ahead = [&](std::uint32_t v, double east) {
        const auto& s = *w.vehicleState(v);
        return std::pair<double, double>{s.latitudeRad, s.longitudeRad + east / (kR * std::cos(s.latitudeRad))};
    };
    // a racetrack round a fix 4 km ahead, inbound north, 4 km legs, 800 m turns
    const auto [trackLat, trackLon] = ahead(racer, 4000.0);
    PatternCommand rt;
    rt.pattern = code(PatternKind::Racetrack), rt.latitudeRad = trackLat, rt.longitudeRad = trackLon;
    rt.courseRad = 0.0, rt.legM = 4000.0, rt.radiusM = 800.0, rt.speed = 55.0, rt.speedReference = 0.0;
    const ActivityId a = submit(w, racer, rt);
    // a hold on a fix 5 km ahead, all by default: right turns, inbound the way it arrives (east), a minute's legs, rate one
    const auto [holdLat, holdLon] = ahead(hold, 5000.0);
    PatternCommand h;
    h.pattern = code(PatternKind::Hold), h.latitudeRad = holdLat, h.longitudeRad = holdLon;
    const ActivityId b = submit(w, hold, h);
    // a figure-eight round a point 3 km ahead, its axis north, 700 m circles
    const auto [eightLat, eightLon] = ahead(eight, 3000.0);
    PatternCommand f8;
    f8.pattern = code(PatternKind::FigureEight), f8.latitudeRad = eightLat, f8.longitudeRad = eightLon, f8.courseRad = 0.0, f8.radiusM = 700.0;
    const ActivityId c = submit(w, eight, f8);
    // a multirotor's small racetrack, 20 m legs, 5 m turns, where it is
    PatternCommand small;
    small.pattern = code(PatternKind::Racetrack), small.courseRad = 0.5 * kPi, small.legM = 20.0, small.radiusM = 5.0, small.speed = 3.0;
    const auto& q0 = *calm.vehicleState(quad);
    const double quadLat = q0.latitudeRad, quadLon = q0.longitudeRad;
    const ActivityId d = submit(calm, quad, small);

    // what the hold made of its defaults: its speed now, rate one at that plus the wind, a minute's legs
    const double holdTas = w.vehicleState(hold)->airspeedTrueMs;
    w.step();
    const ActivityRecord& held = *w.activity(b);
    CHECK(std::abs(held.progress.speedMs - holdTas) < 0.5);
    const double gusted = held.progress.speedMs + 8.0;
    const double holdRadius = std::max(gusted / (3.0 * kDeg), gusted * gusted / (9.80665 * std::tan(25.0 * kDeg)));
    const double holdLeg = held.progress.speedMs * 60.0;
    w.step(stepsFor(w, 240.0)); // entered, a lap and more
    calm.step(stepsFor(calm, 240.0));
    double offTrack = 0.0, offHold = 0.0, offEight = 0.0, offSmall = 0.0, nearCentre = 1e9;
    int crossings = 0;
    bool wasNear = false;
    double inboundS = 0.0, inboundCourse = 0.0;
    int inboundN = 0;
    for (unsigned k = 0; k < stepsFor(w, 600.0); ++k) {
        w.step();
        calm.step();
        offTrack = std::max(offTrack, offRacetrack(from(*w.vehicleState(racer), trackLat, trackLon), 0.0, 800.0, 4000.0));
        const Local hp = from(*w.vehicleState(hold), holdLat, holdLon);
        offHold = std::max(offHold, offRacetrack(hp, 0.5 * kPi, holdRadius, holdLeg));
        if (w.activity(b)->progress.segment == 0) { // on the inbound leg: its time and its track
            inboundS += w.dt() * w.frameSkip();
            inboundCourse += track(*w.vehicleState(hold)), ++inboundN;
        }
        const Local ep = from(*w.vehicleState(eight), eightLat, eightLon);
        offEight = std::max(offEight, std::min(std::abs(std::hypot(ep.north - 700.0, ep.east) - 700.0), std::abs(std::hypot(ep.north + 700.0, ep.east) - 700.0)));
        const double centre = std::hypot(ep.north, ep.east);
        nearCentre = std::min(nearCentre, centre);
        if (centre < 150.0 && !wasNear) ++crossings;
        wasNear = centre < 150.0;
        const Local sp = from(*calm.vehicleState(quad), quadLat, quadLon);
        offSmall = std::max(offSmall, offRacetrack(sp, 0.5 * kPi, 5.0, 20.0));
    }
    const int holdLaps = static_cast<int>(w.activity(b)->progress.laps);
    std::printf("pattern shapes: racetrack %.1f m off, hold %.1f m off (R %.0f, legs %.0f m; inbound %.0f s a lap, at %.1f deg), figure-eight %.1f m off "
                "(%d crossings, %.0f m from the centre), multirotor racetrack %.2f m off\n",
                offTrack, offHold, holdRadius, holdLeg, inboundS / std::max(holdLaps, 1), inboundCourse / std::max(inboundN, 1) / kDeg, offEight,
                crossings, nearCentre, offSmall);
    CHECK(offTrack < 60.0);
    CHECK(offHold < 60.0);
    CHECK(std::abs(inboundCourse / inboundN - 0.5 * kPi) < 3.0 * kDeg); // inbound the way it arrived
    CHECK(offEight < 60.0);
    CHECK(crossings >= 2);  // through the centre, each circle
    CHECK(nearCentre < 50.0);
    CHECK(offSmall < 1.5);
    for (const ActivityId id : {a, b, c}) {
        CHECK(w.activity(id)->state == ActivityState::Active);
        CHECK(w.activity(id)->progress.laps >= 1);
    }
    CHECK(calm.activity(d)->state == ActivityState::Active);
    CHECK(calm.activity(d)->progress.laps >= 10);
    CHECK(w.activity(a)->progress.segments == 4);
    CHECK(w.activity(c)->progress.segments == 2);
}

TEST_CASE("pattern: a speed optimisation is planned at the optimum where it orbits and flown at the optimum now", "[modes]") {
    session::World w(options("patterns-optimised"));
    const auto v = wing(w, "c172", 1500.0, 50.0); // (hangar's C172: it has performance tables)
    settle(w, {v}, 5.0);
    const VehicleProfile& profile = *w.profile(v);
    REQUIRE_FALSE(profile.tables.empty());
    PatternCommand endure;
    endure.altitudeM = 1700.0;
    endure.speedOptimization = static_cast<double>(SpeedOptimization::MaxEndurance);
    const ActivityId id = submit(w, v, endure);
    // planned at the optimum at 1,700 m: its speed, and the radius that speed and 80 % of its bank give
    Setpoint planned;
    REQUIRE(w.activitySetpoint(id, planned));
    const auto& c = std::get<PatternCommand>(std::get<Command>(planned.command));
    const double fuel = w.vehicleState(v)->fuelKg, there = optimalTasMs(&profile.tables, c.speedOptimization, 1700.0, fuel);
    CHECK(std::abs(c.speed - there) < 1e-9);
    CHECK(c.speedReference == static_cast<double>(SpeedReference::TrueAirspeed));
    CHECK(c.radiusM >= w.performance(v)->turnRadiusM(there));
    // flown at the optimum at the altitude and weight now - asked as it climbs to it, held level there
    w.step(stepsFor(w, 330.0)); // (it climbs at its energy balance, overshoots as it levels off, and settles by 280 s)
    const auto& s = *w.vehicleState(v);
    const double now = optimalTasMs(&profile.tables, c.speedOptimization, s.altitudeMslM, s.fuelKg);
    CHECK(std::abs(w.activity(id)->progress.speedMs - now) < 1e-3 * now);
    CHECK(std::abs(s.altitudeMslM - 1700.0) < 20.0);
    CHECK(std::abs(s.airspeedTrueMs - now) < 0.03 * now);
    // a speed replaces it
    PatternCommand faster;
    faster.speed = 55.0;
    REQUIRE(w.update(id, faster).accepted());
    REQUIRE(w.activitySetpoint(id, planned));
    CHECK((std::get<PatternCommand>(std::get<Command>(planned.command)).speed == 55.0 &&
           std::isnan(std::get<PatternCommand>(std::get<Command>(planned.command)).speedOptimization)));
}

TEST_CASE("pattern: a duration completes it; an UPDATE changes only what it gives; what cannot be flown is refused", "[modes]") {
    session::World w(options("patterns-semantics"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    settle(w, {v}, 5.0);
    const double tasNow = w.vehicleState(v)->airspeedTrueMs;
    // defaults: an orbit here, as it flies, right turns, the radius its speed and 80 % of its bank give
    PatternCommand timed;
    timed.durationS = 90.0;
    const ActivityId id = submit(w, v, timed);
    w.step(stepsFor(w, 30.0));
    const ActivityProgress& p = w.activity(id)->progress;
    CHECK(std::abs(p.speedMs - tasNow) < 0.01);
    CHECK(std::abs(p.percent - 100.0 * 30.0 / 90.0) < 2.0);
    CHECK(std::abs(p.timeToGoS - 60.0) < 1.0);
    CHECK(p.segments == 1);
    // only the radius: its centre, speed and duration as commanded
    PatternCommand wider;
    wider.radiusM = 1500.0;
    REQUIRE(w.update(id, wider).accepted());
    for (unsigned k = 0; k < stepsFor(w, 70.0) && w.activity(id)->live(); ++k) w.step();
    const ActivityRecord& done = *w.activity(id);
    CHECK(done.state == ActivityState::Completed);
    CHECK(done.reason == Reason::GoalReached);
    CHECK(std::abs(done.endTime - done.startTime - 90.0) < 0.5);
    CHECK(done.progress.percent == 100.0);
    CHECK(done.progress.speedMs == p.speedMs); // (kept through the UPDATE)

    // refused, naming the field (or clamped, and said so)
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    auto refused = [&](const PatternCommand& c, Reason reason, int index, const CommandOptions& o = {}) {
        const CommandResult r = w.submit(v, c, o);
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == reason);
        CHECK(r.index == index);
        return r;
    };
    PatternCommand c;
    c.pattern = 4.0;
    refused(c, Reason::InvalidParameter, 0);
    c = PatternCommand{}, c.latitudeRad = 0.5; // a point needs both
    refused(c, Reason::InvalidParameter, 2);
    c = PatternCommand{}, c.clockwise = 0.5;
    refused(c, Reason::InvalidParameter, 6);
    c = PatternCommand{}, c.legM = -1.0;
    refused(c, Reason::InvalidParameter, 8);
    c = PatternCommand{}, c.durationS = 0.0;
    refused(c, Reason::InvalidParameter, 11);
    c = PatternCommand{}, c.radiusM = 50.0; // tighter than the c172x turns at 55 m/s at its full bank
    CHECK(refused(c, Reason::PerformanceLimit, 5, reject).constraint == Constraint::MaxOrientation);
    const CommandResult clamped = w.submit(v, c);
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped) != 0);
    CHECK(clamped.index == 5);
    PatternCommand referenceAlone;
    referenceAlone.speedReference = static_cast<double>(SpeedReference::Mach);
    const CommandResult r = w.update(clamped.activity, referenceAlone);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 9);
    // discovery: A-GRA's LOITER, taking UPDATE, on every aircraft
    const auto quad = rotor(w, "iris", 0.0, 3);
    for (const auto a : {v, quad}) {
        const CapabilityDescriptor* d = nullptr;
        for (const auto& cap : w.capabilities(a))
            if (cap.id == "fsim.guidance.pattern") d = &cap;
        REQUIRE(d != nullptr);
        CHECK(d->mode == FlightMode::Loiter);
        CHECK(d->setpoint == SetpointKind::Pattern);
        CHECK((d->interactions & kUpdate) != 0);
        CHECK(d->parameters.size() == 13); // (the speed optimisation last: ADR-29 FA-3e)
    }
}
