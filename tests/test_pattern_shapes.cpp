// A-GRA's orbit and hold as its schema gives them, flown (docs/flight-autonomy.md,
// 4.23 and 4.24; ADR-29 FA-5): a racetrack and a figure-eight by two circles for
// each class, measured against their geometry computed here; a fix-point orbit
// by its inbound heading, its legs' time and its turns' bank, and by a magnetic
// course; laps that complete it; an entry point flown to and an exit point left
// from; a hold's turns by type and rate, its entry and exit times, and each of
// its entries; and what does not make a pattern refused, naming the field.
#include "fsim/GuidanceModes.h"
#include "fsim/Magnetic.h"
#include "fsim/VehicleProfile.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// A place north and east of (lat0, lon0), m, in the plane there; and back.
struct Local {
    double north, east;
};
Local from(double lat, double lon, double lat0, double lon0) {
    return {(lat - lat0) * kR, (lon - lon0) * kR * std::cos(0.5 * (lat + lat0))};
}
Local from(const sim::VehicleState& s, double lat0, double lon0) { return from(s.latitudeRad, s.longitudeRad, lat0, lon0); }
std::pair<double, double> at(Local p, double lat0, double lon0) {
    const double lat = lat0 + p.north / kR;
    return {lat, lon0 + p.east / (kR * std::cos(0.5 * (lat + lat0)))};
}

double toSegment(Local p, Local a, Local b) {
    const double dn = b.north - a.north, de = b.east - a.east, l2 = dn * dn + de * de;
    const double t = l2 > 0.0 ? std::clamp(((p.north - a.north) * dn + (p.east - a.east) * de) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(p.north - a.north - t * dn, p.east - a.east - t * de);
}
double toLine(Local p, Local a, Local b) { // (the whole line)
    const double dn = b.north - a.north, de = b.east - a.east;
    return std::abs((p.north - a.north) * de - (p.east - a.east) * dn) / std::hypot(dn, de);
}

/// Two circles joined by the lines that touch both - a racetrack's on the outside, a figure-eight's crossing
/// between them: the lines' ends where they touch.
struct Circles {
    Local c1, c2;
    double r1, r2;
    bool eight;
    Local touch[2][2]; ///< each line's ends, on the first circle and on the second

    Circles(Local a, double ra, Local b, double rb, bool figureEight) : c1(a), c2(b), r1(ra), r2(rb), eight(figureEight) {
        const double dn = c2.north - c1.north, de = c2.east - c1.east, d = std::hypot(dn, de);
        const double gamma = std::atan2(de, dn), beta = std::acos((eight ? r1 + r2 : r1 - r2) / d);
        for (int k = 0; k < 2; ++k) { // the normal to each line, turned by beta either side of the axis
            const double phi = gamma + (k ? beta : -beta), nn = std::cos(phi), ne = std::sin(phi), s2 = eight ? -1.0 : 1.0;
            touch[k][0] = {c1.north + r1 * nn, c1.east + r1 * ne};
            touch[k][1] = {c2.north + s2 * r2 * nn, c2.east + s2 * r2 * ne};
        }
    }
    /// How far p is off them: the nearer circle or line.
    double off(Local p) const {
        double best = std::min(std::abs(std::hypot(p.north - c1.north, p.east - c1.east) - r1), std::abs(std::hypot(p.north - c2.north, p.east - c2.east) - r2));
        for (const auto& line : touch) best = std::min(best, toSegment(p, line[0], line[1]));
        return best;
    }
};

/// How far p is off a racetrack of radius r and legs l, turning `side` (+1 right), its inbound leg ending at the fix
/// (the origin) along `course`: the points r from the segment joining its turns' centres.
double offRacetrack(Local p, double course, double r, double l, double side = 1.0) {
    const double x = p.north * std::cos(course) + p.east * std::sin(course), y = side * (-p.north * std::sin(course) + p.east * std::cos(course)) - r;
    const double dx = x > 0.0 ? x : x < -l ? x + l : 0.0;
    return std::abs(std::hypot(dx, y) - r);
}

ActivityId submit(session::World& w, std::uint32_t v, const PatternCommand& c, const PatternShape& shape) {
    const CommandResult r = w.submit(v, c, shape);
    INFO("the pattern: " << reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    return r.activity;
}

/// A pattern and its shape.
struct Shaped {
    PatternCommand c;
    PatternShape s;
};

/// What the host made of a pattern: its setpoint and its shape, complete.
Shaped planned(session::World& w, ActivityId id) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(id, s));
    return {std::get<PatternCommand>(std::get<Command>(s.command)), s.shape};
}

double code(PatternKind k) { return static_cast<double>(k); }

/// Wings flying on east at their speed and height for `seconds`: settled, and they have seen the wind.
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

TEST_CASE("pattern shapes: two circles make a racetrack and a figure-eight for each class, their legs touching the circles", "[modes]") {
    struct Shape {
        const char* type;
        bool rotor;
        double speedMs, windMs;
        double r1, r2, apart; ///< the circles, and their centres apart
        bool eight;
        double bound;         ///< off its geometry once on it, m
    };
    // a wing in an 8 m/s wind (as VI-5's shapes), a helicopter and a multirotor calm; circles of two sizes
    for (const Shape& a : {Shape{"c172x", false, 55.0, 8.0, 800.0, 1100.0, 4000.0, false, 60.0}, Shape{"c172x", false, 55.0, 8.0, 700.0, 900.0, 3000.0, true, 60.0},
                          Shape{"uh60", true, 20.0, 0.0, 150.0, 250.0, 800.0, false, 15.0}, Shape{"uh60", true, 20.0, 0.0, 150.0, 200.0, 600.0, true, 15.0},
                          Shape{"iris", true, 3.0, 0.0, 5.0, 8.0, 30.0, false, 1.5}, Shape{"iris", true, 3.0, 0.0, 5.0, 5.0, 20.0, true, 1.5}}) {
        INFO(a.type << (a.eight ? " figure-eight" : " racetrack") << " R " << a.r1 << " and " << a.r2);
        session::World w(options("pattern-circles"));
        if (a.windMs > 0.0) setWind(w, 270.0, a.windMs);
        const auto v = a.rotor ? rotor(w, a.type, 15.0) : wing(w, a.type, 1500.0, a.speedMs);
        if (!a.rotor) settle(w, {v}, 5.0);
        const auto& s0 = *w.vehicleState(v);
        const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
        // the first circle ahead (a rotorcraft's: north), the second beyond it, on an axis 30 degrees off
        const Local c1 = a.rotor ? Local{2.0 * a.r1, 0.0} : Local{0.0, 3000.0};
        const Local c2{c1.north + a.apart * std::cos(30.0 * kDeg), c1.east + a.apart * std::sin(30.0 * kDeg)};
        const Circles geometry(c1, a.r1, c2, a.r2, a.eight);
        for (const auto& line : geometry.touch) { // (the reference itself: each line touches both circles)
            CHECK(std::abs(toLine(c1, line[0], line[1]) - a.r1) < 1e-6);
            CHECK(std::abs(toLine(c2, line[0], line[1]) - a.r2) < 1e-6);
        }
        PatternCommand c;
        PatternShape shape;
        c.pattern = code(a.eight ? PatternKind::FigureEight : PatternKind::Racetrack);
        std::tie(c.latitudeRad, c.longitudeRad) = at(c1, lat0, lon0);
        std::tie(shape.latitude2Rad, shape.longitude2Rad) = at(c2, lat0, lon0);
        c.radiusM = a.r1, shape.radius2M = a.r2;
        c.speed = a.speedMs, c.speedReference = static_cast<double>(a.rotor ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
        const ActivityId id = submit(w, v, c, shape);
        const PatternCommand p = planned(w, id).c;
        CHECK(std::isnan(p.courseRad)); // (the circles give the legs)
        CHECK(std::isnan(p.legM));
        // a lap: round both circles and along both lines
        const double beta = std::acos((a.eight ? a.r1 + a.r2 : a.r1 - a.r2) / a.apart);
        const double lap = a.eight ? 2.0 * (2.0 * kPi - (kPi - 2.0 * beta)) * 0.5 * (a.r1 + a.r2) + 2.0 * a.apart * std::sin(beta)
                                   : (2.0 * kPi - 2.0 * beta) * a.r1 + 2.0 * beta * a.r2 + 2.0 * a.apart * std::sin(beta);
        const double lapS = lap / a.speedMs;
        w.step(stepsFor(w, 0.6 * lapS + 3.0 * a.r1 / a.speedMs + 20.0)); // joined
        double off = 0.0, turned = 0.0, right = 0.0, left = 0.0, last = kHold, nearest[2] = {1e9, 1e9};
        const std::uint32_t laps0 = w.activity(id)->progress.laps;
        for (unsigned k = 0; k < stepsFor(w, 1.2 * lapS); ++k) {
            w.step();
            const auto& s = *w.vehicleState(v);
            const Local q = from(s, lat0, lon0);
            off = std::max(off, geometry.off(q));
            for (int i = 0; i < 2; ++i) // (each line flown: the aircraft passes its middle)
                nearest[i] = std::min(nearest[i], std::hypot(q.north - 0.5 * (geometry.touch[i][0].north + geometry.touch[i][1].north),
                                                             q.east - 0.5 * (geometry.touch[i][0].east + geometry.touch[i][1].east)));
            if (groundSpeed(s) > 0.5) {
                const double t = track(s);
                if (!isHold(last)) {
                    const double d = std::remainder(t - last, 2.0 * kPi);
                    turned += d;
                    (d > 0.0 ? right : left) += std::abs(d);
                }
                last = t;
            }
        }
        const ActivityRecord& r = *w.activity(id);
        std::printf("pattern two circles %-6s %-12s R %5.0f/%5.0f %5.0f m apart: off %6.2f m, turned %4.0f deg right and %4.0f left, lines passed %.1f "
                    "and %.1f m\n",
                    a.type, a.eight ? "figure-eight" : "racetrack", a.r1, a.r2, a.apart, off, right / kDeg, left / kDeg, nearest[0], nearest[1]);
        CHECK(r.state == ActivityState::Active);
        CHECK(off < a.bound);
        CHECK(nearest[0] < a.bound); // both lines flown
        CHECK(nearest[1] < a.bound);
        CHECK(r.progress.laps > laps0);
        if (a.eight) { // round the first circle right, the second left: more than half of each
            CHECK(right > kPi);
            CHECK(left > kPi);
        } else {
            CHECK(turned > 1.8 * kPi); // right turns: once round, and more
        }
    }
}

TEST_CASE("pattern shapes: a fix-point orbit by its inbound heading, its legs' time and its turns' bank; a magnetic course", "[modes]") {
    session::World w(options("pattern-fix"));
    setWind(w, 270.0, 8.0); // from the west: it makes good a course east of its heading
    const auto v = wing(w, "c172x", 1500.0, 55.0), m = wing(w, "c172x", 1500.0, 55.0, 3);
    settle(w, {v, m}, 5.0);
    const auto& s0 = *w.vehicleState(v);
    const double fixLat = s0.latitudeRad, fixLon = s0.longitudeRad + 4000.0 / (kR * std::cos(s0.latitudeRad));
    PatternCommand c;
    PatternShape fix;
    c.pattern = code(PatternKind::Racetrack), c.latitudeRad = fixLat, c.longitudeRad = fixLon;
    fix.headingRad = 0.0, fix.legS = 60.0, fix.bankRad = 20.0 * kDeg;
    const ActivityId id = submit(w, v, c, fix);
    // what it made of them: the course on that heading, the legs of a minute inbound, the radius of 20 degrees'
    // bank at its speed with the wind
    const Shaped plan = planned(w, id);
    const PatternCommand& p = plan.c;
    const double tas = p.speed, gusted = tas + 8.0;
    const double course = std::atan2(8.0, tas), inbound = std::hypot(tas, 8.0);
    CHECK(std::abs(p.courseRad - course) < 0.3 * kDeg);
    CHECK(std::abs(p.legM - inbound * 60.0) < 0.01 * p.legM);
    CHECK(std::abs(p.radiusM - gusted * gusted / (9.80665 * std::tan(20.0 * kDeg))) < 0.01 * p.radiusM);
    CHECK((plan.s.headingRad == 0.0 && plan.s.legS == 60.0 && plan.s.bankRad == 20.0 * kDeg)); // (kept as given)
    // and a racetrack on its magnetic inbound course north, a mile to the north: true north turned by the declination
    const double magneticLat = w.vehicleState(m)->latitudeRad;
    PatternCommand magnetic;
    PatternShape fromMagnetic;
    magnetic.pattern = code(PatternKind::Racetrack), magnetic.latitudeRad = magneticLat, magnetic.longitudeRad = fixLon;
    magnetic.courseRad = 0.0, fromMagnetic.directionReference = static_cast<double>(DirectionReference::MagneticNorth);
    magnetic.legM = 3000.0, magnetic.radiusM = 900.0;
    const ActivityId mid = submit(w, m, magnetic, fromMagnetic);
    const double declination = declinationRad(magneticLat, fixLon, 0.0, 2025.0); // (a world's clock left at its epoch reads 2025)
    REQUIRE(std::abs(declination) > 10.0 * kDeg); // (off San Francisco)

    w.step(stepsFor(w, 300.0)); // entered, and round
    double off = 0.0, offMagnetic = 0.0, inboundS = 0.0, inboundTrack = 0.0, inboundHeading = 0.0, magneticTrack = 0.0, bank = 0.0;
    double legBegan = kHold, legsS = 0.0;
    int inboundN = 0, magneticN = 0, legs = 0;
    bool wasInbound = w.activity(id)->progress.segment == 0;
    for (unsigned k = 0; k < stepsFor(w, 700.0); ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        off = std::max(off, offRacetrack(from(s, fixLat, fixLon), p.courseRad, p.radiusM, p.legM));
        bank = std::max(bank, std::abs(s.eulerRad[0]));
        const bool onInbound = w.activity(id)->progress.segment == 0;
        if (onInbound) inboundTrack += track(s), inboundHeading += std::remainder(s.eulerRad[2], 2.0 * kPi), ++inboundN; // its track and heading
        if (onInbound && !wasInbound) legBegan = w.simTime();                                                          // its time, whole legs
        if (!onInbound && wasInbound && !isHold(legBegan)) legsS += w.simTime() - legBegan, ++legs;
        wasInbound = onInbound;
        const auto& sm = *w.vehicleState(m);
        offMagnetic = std::max(offMagnetic, offRacetrack(from(sm, magneticLat, fixLon), declination, 900.0, 3000.0));
        if (w.activity(mid)->progress.segment == 0 && w.activity(mid)->progress.segmentPercent > 20.0) magneticTrack += track(sm), ++magneticN;
    }
    REQUIRE(legs >= 2);
    inboundS = legsS / legs;
    REQUIRE(inboundN > 0);
    REQUIRE(magneticN > 0);
    std::printf("pattern fix point: off %.1f m; inbound %.1f s a leg (legs %.0f m), track %.2f deg (course %.2f), heading %.2f deg; bank at most "
                "%.1f deg (R %.0f m); magnetic: off %.1f m, inbound track %.2f deg (declination %.2f)\n",
                off, inboundS, p.legM, inboundTrack / inboundN / kDeg, course / kDeg, inboundHeading / inboundN / kDeg, bank / kDeg, p.radiusM,
                offMagnetic, magneticTrack / magneticN / kDeg, declination / kDeg);
    CHECK(off < 60.0);
    CHECK(std::abs(inboundS - 60.0) < 2.0);                                  // a minute's inbound leg
    CHECK(std::abs(inboundTrack / inboundN - course) < 2.0 * kDeg);         // the course it makes good...
    CHECK(std::abs(inboundHeading / inboundN) < 3.0 * kDeg);                // ...on the heading it was given
    CHECK(bank < 23.0 * kDeg);                                              // its turns at the bank given, downwind at most
    CHECK(bank > 14.0 * kDeg);
    CHECK(offMagnetic < 60.0);
    CHECK(std::abs(magneticTrack / magneticN - declination) < 2.0 * kDeg); // magnetic north, flown true
}

TEST_CASE("pattern shapes: laps complete it; an entry point is flown to and an exit point left from, a wing's and a multirotor's", "[modes]") {
    session::World w(options("pattern-ends"));
    const auto counted = wing(w, "c172x", 1500.0, 55.0), entered = wing(w, "c172x", 1500.0, 55.0, 3), left = wing(w, "c172x", 1500.0, 55.0, 6),
               timed = wing(w, "c172x", 1500.0, 55.0, 9);
    settle(w, {counted, entered, left, timed}, 5.0);
    auto ahead = [&](std::uint32_t v, double north, double east) {
        const auto& s = *w.vehicleState(v);
        return std::pair<double, double>{s.latitudeRad + north / kR, s.longitudeRad + east / (kR * std::cos(s.latitudeRad))};
    };
    // two laps of an orbit 2 km ahead, of 800 m
    PatternCommand twice;
    PatternShape twiceShape;
    std::tie(twice.latitudeRad, twice.longitudeRad) = ahead(counted, 0.0, 2000.0);
    twice.radiusM = 800.0, twiceShape.orbits = 2.0;
    const ActivityId a = submit(w, counted, twice, twiceShape);
    const double lapS = 2.0 * kPi * 800.0 / 55.0;
    // an orbit of 1 km 3 km ahead, entered at its south
    const auto [centreLat, centreLon] = ahead(entered, 0.0, 3000.0);
    PatternCommand in;
    PatternShape inShape;
    in.latitudeRad = centreLat, in.longitudeRad = centreLon, in.radiusM = 1000.0;
    std::tie(inShape.entryLatitudeRad, inShape.entryLongitudeRad) = at(Local{-1000.0, 0.0}, centreLat, centreLon);
    const ActivityId b = submit(w, entered, in, inShape);
    // a racetrack on a fix 3 km ahead, inbound north, once round and out at its outbound leg's end
    const auto [fixLat, fixLon] = ahead(left, 0.0, 3000.0);
    PatternCommand out;
    PatternShape outShape;
    out.pattern = code(PatternKind::Racetrack), out.latitudeRad = fixLat, out.longitudeRad = fixLon;
    out.courseRad = 0.0, out.radiusM = 800.0, out.legM = 2000.0, outShape.orbits = 1.0;
    std::tie(outShape.exitLatitudeRad, outShape.exitLongitudeRad) = at(Local{-2000.0, 1600.0}, fixLat, fixLon);
    const ActivityId c = submit(w, left, out, outShape);
    // a minute round an orbit, then out at its east
    const auto [timedLat, timedLon] = ahead(timed, 0.0, 2000.0);
    PatternCommand minute;
    PatternShape minuteShape;
    minute.latitudeRad = timedLat, minute.longitudeRad = timedLon, minute.radiusM = 800.0, minute.durationS = 60.0;
    std::tie(minuteShape.exitLatitudeRad, minuteShape.exitLongitudeRad) = at(Local{0.0, 800.0}, timedLat, timedLon);
    const ActivityId d = submit(w, timed, minute, minuteShape);

    double entryNearest = 1e9, halfway = kHold;
    double exitAt[2] = {kHold, kHold}, exitTime[2] = {kHold, kHold};
    std::uint32_t lapsAtEntry = 99;
    int entryPass = 0; // 0 not yet near its entry point, 1 passing it the first time, 2 past
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 900.0); ++k) {
        w.step();
        const Local e = from(*w.vehicleState(entered), centreLat, centreLon);
        const double toEntry = std::hypot(e.north + 1000.0, e.east);
        if (entryPass == 0 && toEntry < 300.0) entryPass = 1;
        if (entryPass == 1 && toEntry > 300.0) entryPass = 2;
        if (entryPass == 1 && toEntry < entryNearest) entryNearest = toEntry, lapsAtEntry = w.activity(b)->progress.laps; // (the first pass)
        if (isHold(halfway) && w.activity(a)->progress.laps == 1) halfway = w.activity(a)->progress.percent;
        const ActivityId ends[2] = {c, d};
        const Local exits[2] = {from(*w.vehicleState(left), fixLat, fixLon), from(*w.vehicleState(timed), timedLat, timedLon)};
        const Local targets[2] = {Local{-2000.0, 1600.0}, Local{0.0, 800.0}};
        for (int i = 0; i < 2; ++i)
            if (isHold(exitAt[i]) && !w.activity(ends[i])->live())
                exitAt[i] = std::hypot(exits[i].north - targets[i].north, exits[i].east - targets[i].east), exitTime[i] = w.simTime() - t0;
    }
    // then out along the course there: the racetrack's outbound (south), the orbit's at its east (south, right turns)
    w.step(stepsFor(w, 30.0));
    const double outTrack = track(*w.vehicleState(left)), timedTrack = track(*w.vehicleState(timed));
    const Local gone = from(*w.vehicleState(left), fixLat, fixLon);
    const ActivityRecord &ra = *w.activity(a), &rb = *w.activity(b), &rc = *w.activity(c), &rd = *w.activity(d);
    std::printf("pattern ends: 2 laps completed at %.0f s (a lap %.0f s; %.0f %% after the first); entry passed %.1f m off (laps then %u); "
                "exit %.1f m off at %.0f s, then track %.1f deg, %.1f m off its line; timed exit %.1f m off at %.0f s, then track %.1f deg\n",
                ra.endTime - ra.startTime, lapS, halfway, entryNearest, lapsAtEntry, exitAt[0], exitTime[0], outTrack / kDeg,
                std::abs(gone.east - 1600.0), exitAt[1], exitTime[1], timedTrack / kDeg);
    CHECK(ra.state == ActivityState::Completed);
    CHECK(ra.reason == Reason::GoalReached);
    CHECK(ra.progress.laps == 2);
    CHECK(ra.endTime - ra.startTime > 2.0 * lapS);
    CHECK(ra.endTime - ra.startTime < 2.0 * lapS + 90.0);
    CHECK(std::abs(halfway - 50.0) < 12.0);
    CHECK(entryNearest < 150.0); // flown to its entry point...
    CHECK(lapsAtEntry == 0);     // ...before a lap was counted: they are counted from there
    CHECK(rb.progress.laps >= 2);
    CHECK(rb.state == ActivityState::Active);
    CHECK(rc.state == ActivityState::Completed);
    CHECK(rc.progress.laps == 1);
    CHECK(exitAt[0] < 150.0);                                     // completed at its exit point...
    CHECK(std::abs(degreesApart(outTrack, kPi)) < 5.0);           // ...and out along its course there
    CHECK(std::abs(gone.east - 1600.0) < 60.0);
    CHECK(rd.state == ActivityState::Completed);
    CHECK(exitTime[1] > 60.0);                                    // its minute flown, then round to its exit
    CHECK(exitAt[1] < 150.0);
    CHECK(std::abs(degreesApart(timedTrack, kPi)) < 5.0);

    // a multirotor's: to its entry point east of the centre, once round, out at its west (north: right turns)
    session::World calm(options("pattern-ends-small"));
    const auto quad = rotor(calm, "iris", 15.0);
    const auto& q0 = *calm.vehicleState(quad);
    const double qLat = q0.latitudeRad + 30.0 / kR, qLon = q0.longitudeRad;
    PatternCommand small;
    PatternShape smallShape;
    small.latitudeRad = qLat, small.longitudeRad = qLon, small.radiusM = 10.0, small.speed = 3.0, smallShape.orbits = 1.0;
    std::tie(smallShape.entryLatitudeRad, smallShape.entryLongitudeRad) = at(Local{0.0, 10.0}, qLat, qLon);
    std::tie(smallShape.exitLatitudeRad, smallShape.exitLongitudeRad) = at(Local{0.0, -10.0}, qLat, qLon);
    const ActivityId q = submit(calm, quad, small, smallShape);
    double qEntry = 1e9, qExit = kHold;
    for (unsigned k = 0; k < stepsFor(calm, 120.0) && isHold(qExit); ++k) {
        calm.step();
        const Local p = from(*calm.vehicleState(quad), qLat, qLon);
        qEntry = std::min(qEntry, std::hypot(p.north, p.east - 10.0));
        if (!calm.activity(q)->live()) qExit = std::hypot(p.north, p.east + 10.0);
    }
    calm.step(stepsFor(calm, 5.0));
    const double qTrack = track(*calm.vehicleState(quad));
    std::printf("pattern ends, multirotor: entry passed %.2f m off; exit %.2f m off, then track %.1f deg\n", qEntry, qExit, qTrack / kDeg);
    CHECK(calm.activity(q)->state == ActivityState::Completed);
    CHECK(calm.activity(q)->progress.laps == 1);
    CHECK(qEntry < 1.5);
    CHECK(qExit < 1.5);
    CHECK(std::abs(degreesApart(qTrack, 0.0)) < 10.0);
}

TEST_CASE("pattern shapes: what does not make a pattern is refused naming the field; an UPDATE of either way replaces both", "[modes]") {
    session::World w(options("pattern-shape-semantics"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    settle(w, {v}, 5.0);
    const auto& s = *w.vehicleState(v);
    const double east1 = 1.0 / (kR * std::cos(s.latitudeRad)); // a metre east, in longitude
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    auto refused = [&](const Shaped& p, Reason reason, int index, const CommandOptions& o = {}) {
        const CommandResult r = w.submit(v, p.c, p.s, o);
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == reason);
        CHECK(r.index == index);
        return r;
    };
    Shaped circles; // a racetrack's two circles, 4 km apart
    circles.c.pattern = code(PatternKind::Racetrack), circles.c.latitudeRad = s.latitudeRad, circles.c.longitudeRad = s.longitudeRad + 3000.0 * east1;
    circles.s.latitude2Rad = s.latitudeRad + 4000.0 / kR, circles.s.longitude2Rad = circles.c.longitudeRad, circles.c.radiusM = 900.0;
    Shaped c = circles;
    c.c.pattern = code(PatternKind::Orbit); // an orbit is one circle
    refused(c, Reason::InvalidParameter, 18);
    c = circles, c.c.pattern = code(PatternKind::Hold); // a hold is on its fix
    refused(c, Reason::InvalidParameter, 18);
    c = circles, c.c.courseRad = 0.3; // the circles give the course and the legs
    refused(c, Reason::InvalidParameter, 7);
    c = circles, c.s.legS = 60.0;
    refused(c, Reason::InvalidParameter, 15);
    c = circles, c.s.radius2M = 5000.0; // one inside the other
    refused(c, Reason::InvalidParameter, 18);
    c = circles, c.c.pattern = code(PatternKind::FigureEight), c.s.radius2M = 3500.0; // a figure-eight's overlapping
    refused(c, Reason::InvalidParameter, 18);
    c = circles, c.s.longitude2Rad = kHold; // a point needs both
    refused(c, Reason::InvalidParameter, 19);
    c = Shaped{}, c.s.bankRad = 0.0;
    refused(c, Reason::InvalidParameter, 16);
    c.s.bankRad = 0.5 * kPi;
    refused(c, Reason::InvalidParameter, 16);
    c = Shaped{}, c.s.orbits = 1.5;
    refused(c, Reason::InvalidParameter, 17);
    c.s.orbits = -1.0;
    refused(c, Reason::InvalidParameter, 17);
    c = Shaped{}, c.s.legS = -1.0;
    refused(c, Reason::InvalidParameter, 15);
    c = Shaped{}, c.s.directionReference = 2.0;
    refused(c, Reason::InvalidParameter, 13);
    c = Shaped{}, c.s.entryLatitudeRad = s.latitudeRad;
    refused(c, Reason::InvalidParameter, 22);
    c = Shaped{}, c.s.exitLongitudeRad = s.longitudeRad;
    refused(c, Reason::InvalidParameter, 24);
    // a hold's (4.24): a turn rate above 0, a type, an entry and a context of theirs; an entry only a racetrack's or a
    // hold's on its fix, a context only a hold's
    c = Shaped{}, c.s.turnRateRadS = 0.0;
    refused(c, Reason::InvalidParameter, 25);
    c = Shaped{}, c.s.turnType = 3.0;
    refused(c, Reason::InvalidParameter, 26);
    c = Shaped{}, c.c.pattern = code(PatternKind::Hold), c.s.holdEntry = 6.0;
    refused(c, Reason::InvalidParameter, 27);
    c = Shaped{}, c.c.pattern = code(PatternKind::Hold), c.s.holdContext = 3.0;
    refused(c, Reason::InvalidParameter, 28);
    c = Shaped{}, c.s.holdEntry = static_cast<double>(HoldEntry::Parallel); // an orbit has no fix
    refused(c, Reason::InvalidParameter, 27);
    c = circles, c.s.holdEntry = static_cast<double>(HoldEntry::Inbound); // nor two circles
    refused(c, Reason::InvalidParameter, 27);
    c = Shaped{}, c.c.pattern = code(PatternKind::Hold), c.s.holdEntry = static_cast<double>(HoldEntry::Anchor); // an entry, or an entry point
    c.s.entryLatitudeRad = s.latitudeRad, c.s.entryLongitudeRad = s.longitudeRad;
    refused(c, Reason::InvalidParameter, 27);
    c = Shaped{}, c.c.pattern = code(PatternKind::Racetrack), c.s.holdContext = static_cast<double>(HoldContext::Tactical);
    refused(c, Reason::InvalidParameter, 28);
    c = Shaped{}, c.s.turnRateRadS = 0.5; // faster than it turns at its full bank: held to that, and the radius it gives
    CHECK(refused(c, Reason::PerformanceLimit, 25, reject).constraint == Constraint::MaxOrientation);
    {
        const CommandResult rated = w.submit(v, c.c, c.s);
        REQUIRE(rated.accepted());
        CHECK(rated.index == 25);
        const Shaped flown = planned(w, rated.activity);
        CHECK(std::abs(flown.s.turnRateRadS - 9.80665 * std::tan(w.performance(v)->maxBankRad) / flown.c.speed) < 1e-9);
        CHECK(std::abs(flown.c.radiusM - flown.c.speed / flown.s.turnRateRadS) < 1e-6 * flown.c.radiusM); // (calm)
    }
    // a bank steeper than it flies: refused, or held to its full bank and the radius that gives
    const double most = w.performance(v)->maxBankRad;
    REQUIRE(most < 1.3);
    c = Shaped{}, c.s.bankRad = 1.3;
    CHECK(refused(c, Reason::PerformanceLimit, 16, reject).constraint == Constraint::MaxOrientation);
    const CommandResult clamped = w.submit(v, c.c, c.s);
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped) != 0);
    CHECK(clamped.index == 16);
    const Shaped held = planned(w, clamped.activity);
    CHECK(held.s.bankRad == most);
    const double gusted = held.c.speed + std::hypot(w.environment().windSpeedMs, 0.0);
    CHECK(std::abs(held.c.radiusM - gusted * gusted / (9.80665 * std::tan(most))) < 0.02 * held.c.radiusM);

    // an UPDATE: a heading replaces the course, a bank the radius, a time the legs (each filled in again)
    PatternCommand hold;
    hold.pattern = code(PatternKind::Hold), hold.latitudeRad = s.latitudeRad, hold.longitudeRad = s.longitudeRad + 5000.0 * east1;
    const ActivityId h = submit(w, v, hold, PatternShape{});
    const Shaped before = planned(w, h);
    PatternShape turn;
    turn.headingRad = 0.25 * kPi, turn.bankRad = 15.0 * kDeg, turn.legS = 45.0;
    REQUIRE(w.update(h, PatternCommand{}, turn).accepted());
    const Shaped after = planned(w, h);
    CHECK(after.s.headingRad == 0.25 * kPi);
    CHECK(std::abs(after.c.courseRad - 0.25 * kPi) < 1.0 * kDeg); // (calm: the course is the heading)
    CHECK(after.s.bankRad == 15.0 * kDeg);
    CHECK(std::abs(after.c.radiusM - after.c.speed * after.c.speed / (9.80665 * std::tan(15.0 * kDeg))) < 0.01 * after.c.radiusM);
    CHECK(std::abs(after.c.legM - after.c.speed * 45.0) < 0.01 * after.c.legM);
    CHECK(after.c.radiusM != before.c.radiusM);
    PatternCommand course; // and a course the heading, a radius the bank - given alone, as ever (no shape)
    course.courseRad = 0.5, course.radiusM = 2000.0;
    REQUIRE(w.update(h, course).accepted());
    const Shaped last = planned(w, h);
    CHECK(last.c.courseRad == 0.5);
    CHECK(std::isnan(last.s.headingRad));
    CHECK(last.c.radiusM == 2000.0);
    CHECK(std::isnan(last.s.bankRad));
    CHECK(last.c.legM == after.c.legM); // (the legs as they were)
    CHECK(last.s.legS == 45.0);
    PatternShape alone; // a direction reference without its course
    alone.directionReference = static_cast<double>(DirectionReference::MagneticNorth);
    const CommandResult r = w.update(h, PatternCommand{}, alone);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 13);
    PatternShape relaxed; // a turn type replaces the radius, and is filled in again (a hold's context kept as given)
    relaxed.turnType = static_cast<double>(HoldTurn::Relax), relaxed.holdContext = static_cast<double>(HoldContext::Admin);
    REQUIRE(w.update(h, PatternCommand{}, relaxed).accepted());
    const Shaped easy = planned(w, h);
    CHECK((std::isnan(easy.s.bankRad) && easy.s.turnType == static_cast<double>(HoldTurn::Relax) && easy.s.holdContext == 0.0));
    CHECK(std::abs(easy.c.radiusM - std::max(easy.c.speed / (1.5 * kDeg), easy.c.speed * easy.c.speed / (9.80665 * std::tan(15.0 * kDeg)))) <
          1e-6 * easy.c.radiusM);
    PatternShape second; // a second circle for a hold, in an UPDATE
    second.latitude2Rad = s.latitudeRad, second.longitude2Rad = s.longitudeRad;
    const CommandResult r2 = w.update(h, PatternCommand{}, second);
    CHECK(r2.reason == Reason::InvalidParameter);
    CHECK(r2.index == 18);
    CHECK(w.activity(h)->live()); // (what was refused changed nothing)
}

TEST_CASE("pattern shapes: a hold's turns by type and by rate; its entry and exit times, the command's window", "[modes]") {
    session::World w(options("pattern-hold-turns"));
    const auto standard = wing(w, "c172x", 1500.0, 55.0), mil = wing(w, "c172x", 1500.0, 55.0, 3), relax = wing(w, "c172x", 1500.0, 55.0, 6),
               rated = wing(w, "c172x", 1500.0, 55.0, 9), timed = wing(w, "c172x", 1500.0, 55.0, 12);
    settle(w, {standard, mil, relax, rated, timed}, 5.0);
    auto holdAhead = [&](std::uint32_t v) {
        const auto& s = *w.vehicleState(v);
        PatternCommand h;
        h.pattern = code(PatternKind::Hold), h.latitudeRad = s.latitudeRad, h.longitudeRad = s.longitudeRad + 3000.0 / (kR * std::cos(s.latitudeRad));
        return h;
    };
    // calm: the radius is the speed's alone
    const std::uint32_t by[] = {standard, mil, relax, rated};
    const double types[] = {static_cast<double>(HoldTurn::Standard), static_cast<double>(HoldTurn::MilPower), static_cast<double>(HoldTurn::Relax), kHold};
    ActivityId ids[4];
    for (int i = 0; i < 4; ++i) {
        PatternShape turns;
        turns.turnType = types[i];
        if (i == 3) turns.turnRateRadS = 0.04;
        ids[i] = submit(w, by[i], holdAhead(by[i]), turns);
    }
    const Performance& perf = *w.performance(standard);
    double v[4], r[4];
    for (int i = 0; i < 4; ++i) {
        const Shaped p = planned(w, ids[i]);
        v[i] = p.c.speed, r[i] = p.c.radiusM;
    }
    CHECK(std::abs(r[0] - std::max(v[0] / (3.0 * kDeg), v[0] * v[0] / (9.80665 * std::tan(25.0 * kDeg)))) < 1e-6 * r[0]); // rate one, at most 25 degrees
    CHECK(std::abs(r[1] - perf.turnRadiusM(v[1])) < 1e-6 * r[1]);                                                         // 80 % of its bank
    CHECK(std::abs(r[2] - std::max(v[2] / (1.5 * kDeg), v[2] * v[2] / (9.80665 * std::tan(15.0 * kDeg)))) < 1e-6 * r[2]); // half rate one, at most 15
    CHECK(std::abs(r[3] - v[3] / 0.04) < 1e-6 * r[3]);                                                                     // its rate
    CHECK((r[1] < r[0] && r[0] < r[2]));
    // the entry and exit times: its window - it waits for the first, and completes at the second
    CommandOptions window;
    window.window.startNotBefore = w.simTime() + 30.0, window.window.endNotAfter = w.simTime() + 150.0;
    const CommandResult waits = w.submit(timed, holdAhead(timed), PatternShape{}, window);
    REQUIRE(waits.accepted());
    CHECK((waits.flags & kDeferred) != 0);
    double started = kHold;
    double bank[4] = {0.0, 0.0, 0.0, 0.0};
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 240.0); ++k) {
        w.step();
        const ActivityRecord& h = *w.activity(waits.activity);
        if (isHold(started) && h.state == ActivityState::Active) started = w.simTime() - t0;
        if (w.simTime() - t0 > 100.0) // (entered: round its turns)
            for (int i = 0; i < 4; ++i) bank[i] = std::max(bank[i], std::abs(w.vehicleState(by[i])->eulerRad[0]));
    }
    const ActivityRecord& h = *w.activity(waits.activity);
    std::printf("pattern hold turns: R %.0f (standard), %.0f (mil power), %.0f (relax), %.0f m (0.04 rad/s); banked at most %.1f, %.1f, %.1f, %.1f deg; "
                "window: started at %.1f s, ended at %.1f s\n",
                r[0], r[1], r[2], r[3], bank[0] / kDeg, bank[1] / kDeg, bank[2] / kDeg, bank[3] / kDeg, started, h.endTime - t0);
    CHECK(bank[0] < 26.0 * kDeg);
    CHECK(bank[1] > 0.7 * perf.maxBankRad);
    CHECK(bank[2] < 16.0 * kDeg);
    CHECK(std::abs(started - 30.0) < 1.0);
    CHECK(h.state == ActivityState::Completed);
    CHECK(h.reason == Reason::GoalReached);
    CHECK(std::abs(h.endTime - t0 - 150.0) < 1.0);
}

TEST_CASE("pattern shapes: each hold entry flown as specified - direct, anchor from each sector, inbound, outbound, parallel, teardrop", "[modes]") {
    // wings flying east at 55 m/s, each toward a hold on a fix 5 km ahead; calm
    struct Entry {
        const char* name;
        HoldEntry entry;
        double inboundDeg; ///< the hold's inbound course: which side of it the aircraft comes from
        double clockwise;
    };
    const Entry cases[] = {{"inbound", HoldEntry::Inbound, 0.0, 1.0},
                           {"outbound", HoldEntry::Outbound, 0.0, 1.0},
                           {"parallel", HoldEntry::Parallel, 0.0, 1.0},
                           {"teardrop", HoldEntry::Teardrop, 0.0, 1.0},
                           {"direct (left turns)", HoldEntry::Direct, 0.0, 0.0},
                           {"anchor, direct sector", HoldEntry::Anchor, 90.0, 1.0},
                           {"anchor, teardrop sector", HoldEntry::Anchor, -60.0, 1.0},
                           {"anchor, parallel sector", HoldEntry::Anchor, 210.0, 1.0}};
    constexpr int kCases = static_cast<int>(std::size(cases));
    session::World w(options("pattern-hold-entries"));
    std::uint32_t v[kCases];
    for (int i = 0; i < kCases; ++i) v[i] = wing(w, "c172x", 1500.0, 55.0, 3 * i);
    settle(w, {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]}, 5.0);
    double fixLat[kCases], fixLon[kCases], chi[kCases], side[kCases], r[kCases], L[kCases];
    ActivityId ids[kCases];
    for (int i = 0; i < kCases; ++i) {
        const auto& s = *w.vehicleState(v[i]);
        fixLat[i] = s.latitudeRad, fixLon[i] = s.longitudeRad + 5000.0 / (kR * std::cos(s.latitudeRad));
        PatternCommand h;
        h.pattern = code(PatternKind::Hold), h.latitudeRad = fixLat[i], h.longitudeRad = fixLon[i];
        h.courseRad = cases[i].inboundDeg * kDeg, h.clockwise = cases[i].clockwise;
        PatternShape e;
        e.holdEntry = static_cast<double>(cases[i].entry);
        ids[i] = submit(w, v[i], h, e);
        const Shaped p = planned(w, ids[i]);
        chi[i] = p.c.courseRad, side[i] = p.c.clockwise == 0.0 ? -1.0 : 1.0, r[i] = p.c.radiusM, L[i] = p.c.legM;
    }
    // what marks each entry: a straight it flies, and the way it flies it, that the hold itself does not
    struct Mark {
        Local at;
        double trackRad;
    };
    // a leg flown out from the fix: begun where the turn over the fix, from the way the aircraft came (east), onto its
    // course `c` ends - its middle
    auto legOut = [&](int i, double c) -> Mark {
        const double in = 0.5 * kPi, turn = std::remainder(c - in, 2.0 * kPi), way = turn >= 0.0 ? 1.0 : -1.0;
        const double xn = way * r[i] * (std::sin(c) - std::sin(in)), xe = way * r[i] * (std::cos(in) - std::cos(c));
        return {{xn + 0.5 * L[i] * std::cos(c), xe + 0.5 * L[i] * std::sin(c)}, c};
    };
    auto mark = [&](int i) -> Mark {
        const double un = std::cos(chi[i]), ue = std::sin(chi[i]), hn = -side[i] * ue, he = side[i] * un;
        const Mark outAlongInbound = legOut(i, chi[i] + kPi), teardrop = legOut(i, chi[i] + kPi - side[i] * 30.0 * kDeg);
        switch (cases[i].entry) {
        // (entered along a leg: on its course as it begins - from before it, onto its course round a corner)
        case HoldEntry::Inbound: return {{-L[i] * un, -L[i] * ue}, chi[i]};                         // the inbound leg's start
        case HoldEntry::Outbound: return {{2.0 * r[i] * hn, 2.0 * r[i] * he}, chi[i] + kPi};        // the outbound leg's, abeam the fix
        case HoldEntry::Parallel: return outAlongInbound;                                           // out along the inbound course
        case HoldEntry::Teardrop: return teardrop;                                                  // out 30 degrees into the holding side
        case HoldEntry::Direct: return {{2.0 * r[i] * hn - L[i] * un, 2.0 * r[i] * he - L[i] * ue}, chi[i] + kPi}; // joined at the outbound leg: its end
        default: // anchor: ATC's entry for its sector - direct over the fix along the inbound course, or the teardrop's, or the parallel's
            return cases[i].inboundDeg == -60.0 ? teardrop : cases[i].inboundDeg == 210.0 ? outAlongInbound : Mark{{0.0, 0.0}, chi[i]};
        }
    };
    double nearest[kCases], trackThere[kCases], firstAtFix[kCases], off[kCases];
    for (int i = 0; i < kCases; ++i) nearest[i] = 1e9, trackThere[i] = kHold, firstAtFix[i] = kHold, off[i] = 0.0;
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 720.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        for (int i = 0; i < kCases; ++i) {
            const auto& s = *w.vehicleState(v[i]);
            const Local q = from(s, fixLat[i], fixLon[i]);
            const Mark m = mark(i);
            if (const double d = std::hypot(q.north - m.at.north, q.east - m.at.east); t < 300.0 && d < nearest[i]) nearest[i] = d, trackThere[i] = track(s);
            if (isHold(firstAtFix[i]) && std::hypot(q.north, q.east) < 300.0) firstAtFix[i] = t;
            if (t > 480.0) off[i] = std::max(off[i], offRacetrack(q, chi[i], r[i], L[i], side[i])); // (entered, and half a lap on)
        }
    }
    for (int i = 0; i < kCases; ++i) {
        INFO(cases[i].name);
        const double apart = std::abs(degreesApart(trackThere[i], mark(i).trackRad));
        std::printf("pattern hold entry %-24s R %4.0f m, legs %4.0f m: its mark passed %5.1f m off, track %5.1f deg off; first at the fix %5.0f s; "
                    "then %.1f m off the hold\n",
                    cases[i].name, r[i], L[i], nearest[i], apart, firstAtFix[i], off[i]);
        CHECK(w.activity(ids[i])->state == ActivityState::Active);
        CHECK(nearest[i] < 0.1 * r[i]); // it flew there...
        CHECK(apart < 10.0);            // ...the way the entry goes
        if (cases[i].entry == HoldEntry::Direct) CHECK(firstAtFix[i] > 150.0); // (joined where nearest, not over the fix)
        CHECK(off[i] < 60.0);
    }
}
