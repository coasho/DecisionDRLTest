// A route's civil path terminators as A-GRA's schema gives them (docs/flight-autonomy.md, 4.38; ADR-29 FA-6f): the ARINC
// 424 leg type of the leg into each point - a track to fix's great circle, a direct to fix's leg from where the aircraft is,
// a course to fix's course, a radius to fix's arc round its centre - flown; what does not make one refused, naming its point;
// the legs FA-6f2 builds not implemented.
#include "control/Route.h"
#include "fsim/ControlStack.h"
#include "fsim/GuidanceModes.h"
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

double code(PathTerminator t) { return static_cast<double>(t); }

/// A point `north` and `east` metres from (lat0, lon0).
Waypoint at(double lat0, double lon0, double north, double east) {
    Waypoint p;
    p.latitudeRad = lat0 + north / kR, p.longitudeRad = lon0 + east / (kR * std::cos(lat0));
    return p;
}

void apart(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    north = (lat - lat0) * kR, east = (lon - lon0) * kR * std::cos(lat0);
}

/// A C172 east at 1,500 m and 55 m/s, at a latitude of its own (`latDeg`), `index` 3 km apart north.
std::uint32_t wingAt(session::World& w, double latDeg, int index) {
    session::VehicleSpec s;
    s.name = "c172-" + std::to_string(index);
    s.type = "jsbsim:c172";
    s.initial.latitudeDeg = latDeg + 0.03 * index;
    s.initial.longitudeDeg = 0.5;
    s.initial.altitudeMslM = 1500.0;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = 55.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

/// A radius to fix's arc round the point `north`, `east` from (lat0, lon0), its way round.
RouteTerminator arcRound(std::uint32_t point, double lat0, double lon0, double north, double east, bool right) {
    const Waypoint c = at(lat0, lon0, north, east);
    RouteTerminator t;
    t.point = point, t.centerLatitudeRad = c.latitudeRad, t.centerLongitudeRad = c.longitudeRad, t.clockwise = right ? 1.0 : 0.0;
    return t;
}

} // namespace

TEST_CASE("route terminators: a track to fix flies the great circle on a rhumb route; a direct to fix turns straight to its point "
          "from where it flew over the one before; a course to fix arrives on its course",
          "[modes]") {
    session::World w(options("route-terminators-legs"));
    const auto tf = wingAt(w, 60.0, 0), rl = wingAt(w, 60.0, 1);         // (a track to fix, and the route's own rhumb line)
    const auto df = wingAt(w, 37.6, 2), line = wingAt(w, 37.6, 3);       // (a direct to fix, and the route's own leg)
    const auto cf = wingAt(w, 37.6, 4);                                  // (a course to fix)
    w.step(stepsFor(w, 1.0));
    auto origin = [&w](std::uint32_t v, double& lat, double& lon) { lat = w.vehicleState(v)->latitudeRad, lon = w.vehicleState(v)->longitudeRad; };
    // 2 km east, then 40 km east on a rhumb route: its leg to the second a track to fix (a great circle, 54 m poleward of
    // the rhumb line in its middle at 60 degrees), or the route's own
    RouteCommand rhumb;
    rhumb.projection = static_cast<double>(Projection::Rhumb);
    double tlat, tlon, rlat, rlon;
    origin(tf, tlat, tlon), origin(rl, rlat, rlon);
    std::vector<Waypoint> tpoints = {at(tlat, tlon, 0.0, 2000.0), at(tlat, tlon, 0.0, 42000.0)}, rpoints = {at(rlat, rlon, 0.0, 2000.0), at(rlat, rlon, 0.0, 42000.0)};
    tpoints[1].terminator = code(PathTerminator::TrackToFix);
    const CommandResult ct = w.submit(tf, rhumb, tpoints), cr = w.submit(rl, rhumb, rpoints);
    INFO(reasonName(ct.reason) << " at " << ct.index);
    REQUIRE(ct.accepted());
    REQUIRE(cr.accepted());
    // east 3 km to a waypoint flown over, then 6 km north - direct to fix, or the route's own leg - and 3 km on
    double dlat, dlon, llat, llon;
    origin(df, dlat, dlon), origin(line, llat, llon);
    std::vector<Waypoint> dpoints = {at(dlat, dlon, 0.0, 3000.0), at(dlat, dlon, 6000.0, 3000.0), at(dlat, dlon, 9000.0, 3000.0)};
    std::vector<Waypoint> lpoints = {at(llat, llon, 0.0, 3000.0), at(llat, llon, 6000.0, 3000.0), at(llat, llon, 9000.0, 3000.0)};
    dpoints[0].kind = lpoints[0].kind = static_cast<double>(EndPointKind::Waypoint);
    dpoints[1].terminator = code(PathTerminator::DirectToFix);
    const CommandResult cd = w.submit(df, RouteCommand{}, dpoints), cl = w.submit(line, RouteCommand{}, lpoints);
    INFO(reasonName(cd.reason) << " at " << cd.index);
    REQUIRE(cd.accepted());
    REQUIRE(cl.accepted());
    // east 3 km to a waypoint flown over, then to a point 6 km north and 1 km east of it on a course to fix due north: its
    // course's line, 1 km east of the point before, joined from abeam it
    double clat, clon;
    origin(cf, clat, clon);
    std::vector<Waypoint> cpoints = {at(clat, clon, 0.0, 3000.0), at(clat, clon, 6000.0, 4000.0), at(clat, clon, 9000.0, 4000.0)};
    cpoints[0].kind = static_cast<double>(EndPointKind::Waypoint);
    cpoints[1].terminator = code(PathTerminator::CourseToFix);
    RouteTerminator course;
    course.point = 1, course.courseRad = 0.0;
    const CommandResult cc = w.submit(cf, RouteCommand{}, cpoints, {}, {}, {}, {}, {}, std::vector<RouteTerminator>{course});
    INFO(reasonName(cc.reason) << " at " << cc.index);
    REQUIRE(cc.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(cc.activity, sp));
    REQUIRE(sp.terminators.size() == 1);
    CHECK((sp.terminators[0].point == 1 && sp.terminators[0].courseRad == 0.0));
    CHECK(sp.waypoints[1].terminator == code(PathTerminator::CourseToFix));

    double tfNorth = 0.0, rlNorth = 0.0, tfCross = 0.0, dfEast = 0.0, lineEast = 0.0, dfWest = 0.0, cfOff = 0.0, cfTrack = 0.0;
    bool tfMid = false, rlMid = false, dfMid = false, lineMid = false;
    int cfSamples = 0;
    for (unsigned k = 0; k < stepsFor(w, 900.0); ++k) {
        w.step();
        double n, e;
        const auto& t = *w.vehicleState(tf);
        apart(tlat, tlon, t.latitudeRad, t.longitudeRad, n, e);
        if (!tfMid && e >= 22000.0) tfMid = true, tfNorth = n, tfCross = w.activity(ct.activity)->progress.crossTrackM;
        const auto& r = *w.vehicleState(rl);
        apart(rlat, rlon, r.latitudeRad, r.longitudeRad, n, e);
        if (!rlMid && e >= 22000.0) rlMid = true, rlNorth = n;
        const auto& d = *w.vehicleState(df);
        apart(dlat, dlon, d.latitudeRad, d.longitudeRad, n, e);
        if (w.activity(cd.activity)->live() && w.activity(cd.activity)->progress.segment == 1) dfWest = std::min(dfWest, e - 3000.0);
        if (!dfMid && n >= 3000.0) dfMid = true, dfEast = e - 3000.0;
        const auto& l = *w.vehicleState(line);
        apart(llat, llon, l.latitudeRad, l.longitudeRad, n, e);
        if (!lineMid && n >= 3000.0) lineMid = true, lineEast = e - 3000.0;
        const auto& c = *w.vehicleState(cf);
        apart(clat, clon, c.latitudeRad, c.longitudeRad, n, e);
        if (w.activity(cc.activity)->live() && w.activity(cc.activity)->progress.segment == 1 && n >= 4000.0 && n <= 6000.0) {
            cfOff = std::max(cfOff, std::abs(e - 4000.0));
            cfTrack = std::max(cfTrack, std::abs(route::trackOf(c)) / kDeg);
            ++cfSamples;
        }
    }
    std::printf("route terminators: a track to fix %.1f m north of the rhumb line in its leg's middle (the route's own line %.1f m; "
                "%.1f m off its great circle); a direct to fix %.0f m east of the line from the point it flew over, halfway, where "
                "the route's own leg was %.0f m (never west of it: %.1f m); a course to fix within %.1f m of its course's line and "
                "%.2f degrees of its course in its last 2 km (%d samples)\n",
                tfNorth, rlNorth, tfCross, dfEast, lineEast, dfWest, cfOff, cfTrack, cfSamples);
    CHECK((tfMid && rlMid));
    CHECK(tfNorth - rlNorth > 45.0);
    CHECK(std::abs(tfCross) < 5.0);
    CHECK(std::abs(rlNorth) < 5.0);
    CHECK((dfMid && lineMid));
    CHECK(dfEast > 150.0);
    CHECK(dfEast - lineEast > 100.0);
    CHECK(dfWest > -5.0);
    CHECK(cfSamples > 20);
    CHECK(cfOff < 15.0);
    CHECK(cfTrack < 2.0);
    for (const auto a : {ct.activity, cr.activity, cd.activity, cl.activity, cc.activity}) CHECK(w.activity(a)->state == ActivityState::Completed);
}

TEST_CASE("route terminators: a radius to fix's arc round its centre, flown by a wing and a rotorcraft within 20 m; its data read "
          "back",
          "[modes]") {
    session::World w(options("route-terminators-arcs"));
    const auto quad = rotor(w, "iris", 15.0, 6);
    const auto v = wing(w, "c172", 1500.0, 55.0); // (east)
    w.step(stepsFor(w, 1.0));
    // east 3 km, then a quarter circle of 1,500 m round to the right - its centre 1,500 m south of its start - and on south
    // 3 km: every field of its arc given, as A-GRA's schema has them
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad, r = 1500.0;
    std::vector<Waypoint> points = {at(lat0, lon0, 0.0, 3000.0), at(lat0, lon0, -r, 3000.0 + r), at(lat0, lon0, -r - 3000.0, 3000.0 + r)};
    points[1].terminator = code(PathTerminator::RadiusToFix);
    RouteTerminator arc = arcRound(1, lat0, lon0, -r, 3000.0, true);
    arc.radiusM = r, arc.courseInRad = 0.5 * kPi, arc.courseOutRad = kPi;
    arc.initialLatitudeRad = points[0].latitudeRad, arc.initialLongitudeRad = points[0].longitudeRad;
    arc.endLatitudeRad = points[1].latitudeRad, arc.endLongitudeRad = points[1].longitudeRad;
    arc.arcM = r * 0.5 * kPi, arc.chordM = r * std::sqrt(2.0);
    const CommandResult c = w.submit(v, RouteCommand{}, points, {}, {}, {}, {}, {}, std::vector<RouteTerminator>{arc});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(c.activity, sp));
    REQUIRE(sp.terminators.size() == 1);
    CHECK((sp.terminators[0].point == 1 && sp.terminators[0].radiusM == r && sp.terminators[0].clockwise == 1.0 && sp.terminators[0].arcM == arc.arcM));
    // a rotorcraft's: north 40 m, then a quarter circle of 50 m round to the left - its centre 50 m west of its start - on west
    const auto& q0 = *w.vehicleState(quad);
    const double qlat = q0.latitudeRad, qlon = q0.longitudeRad, qr = 50.0;
    std::vector<Waypoint> qpoints = {at(qlat, qlon, 40.0, 0.0), at(qlat, qlon, 40.0 + qr, -qr), at(qlat, qlon, 40.0 + qr, -qr - 40.0)};
    qpoints[1].terminator = code(PathTerminator::RadiusToFix);
    const CommandResult qc = w.submit(quad, RouteCommand{}, qpoints, {}, {}, {}, {}, {}, std::vector<RouteTerminator>{arcRound(1, qlat, qlon, 40.0, -qr, false)});
    INFO(reasonName(qc.reason) << " at " << qc.index);
    REQUIRE(qc.accepted());
    // off the arc while its segment is flown, all of it
    double arcOff = 0.0, qOff = 0.0;
    int arcSamples = 0, qSamples = 0;
    for (unsigned k = 0; k < stepsFor(w, 260.0); ++k) {
        w.step();
        const ActivityProgress& g = w.activity(c.activity)->progress;
        if (w.activity(c.activity)->live() && g.segment == 1 && g.segmentPercent > 5.0 && g.segmentPercent < 95.0) {
            double n, e;
            apart(lat0, lon0, w.vehicleState(v)->latitudeRad, w.vehicleState(v)->longitudeRad, n, e);
            arcOff = std::max(arcOff, std::abs(std::hypot(n + r, e - 3000.0) - r));
            ++arcSamples;
        }
        const ActivityProgress& h = w.activity(qc.activity)->progress;
        if (w.activity(qc.activity)->live() && h.segment == 1 && h.segmentPercent > 5.0 && h.segmentPercent < 95.0) {
            double n, e;
            apart(qlat, qlon, w.vehicleState(quad)->latitudeRad, w.vehicleState(quad)->longitudeRad, n, e);
            qOff = std::max(qOff, std::abs(std::hypot(n - 40.0, e + qr) - qr));
            ++qSamples;
        }
    }
    std::printf("route terminators, arcs: a C172 %.1f m off its 1,500 m radius to fix (%d samples), an IRIS %.2f m off its 50 m one (%d); "
                "both %s, %s\n",
                arcOff, arcSamples, qOff, qSamples, activityStateName(w.activity(c.activity)->state), activityStateName(w.activity(qc.activity)->state));
    CHECK(arcSamples > 100);
    CHECK(qSamples > 20);
    CHECK(arcOff < 20.0);
    CHECK(qOff < 1.0);
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
    CHECK(w.activity(qc.activity)->state == ActivityState::Completed);
}

TEST_CASE("route terminators: refused as a point is, naming it - data that is none, a leg its segment does not define, an arc or a "
          "course that is not its own; not built, FA-6f2's legs",
          "[modes]") {
    session::World w(options("route-terminators-refused"));
    const auto v = wing(w, "c172", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad, r = 1500.0;
    // east 3 km, a quarter circle round to the right to (-r, 3000 + r), south 3 km
    const std::vector<Waypoint> base = {at(lat0, lon0, 0.0, 3000.0), at(lat0, lon0, -r, 3000.0 + r), at(lat0, lon0, -r - 3000.0, 3000.0 + r)};
    const RouteTerminator arc = arcRound(1, lat0, lon0, -r, 3000.0, true);
    auto refuse = [&](const char* what, std::vector<Waypoint> points, std::vector<RouteTerminator> terminators, Reason why, int index,
                      std::vector<RouteBranch> branches = {}, std::vector<RouteLoiter> loiters = {}) {
        const CommandResult c = w.submit(v, RouteCommand{}, points, {}, loiters, {}, {}, branches, terminators);
        INFO(what << ": " << reasonName(c.reason) << " at " << c.index);
        CHECK(c.reason == why);
        CHECK(c.index == index);
    };
    auto with = [&base](std::uint32_t i, PathTerminator t) {
        std::vector<Waypoint> p = base;
        p.at(i).terminator = code(t);
        return p;
    };
    std::vector<Waypoint> p = base;
    p.at(1).terminator = 23.0;
    refuse("a code that is none", p, {}, Reason::InvalidWaypoint, 1);
    RouteTerminator bad = arc;
    bad.point = 7;
    refuse("data at a point past the route", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 7);
    refuse("two for one point", with(1, PathTerminator::RadiusToFix), {arc, arc}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.radiusM = std::numeric_limits<double>::infinity();
    refuse("a field not finite", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.clockwise = 2.0;
    refuse("a way round that is none", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.courseRad = 0.0;
    refuse("a course on an arc", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    refuse("an arc's data on a course to fix", with(1, PathTerminator::CourseToFix), {arc}, Reason::InvalidWaypoint, 1);
    refuse("data on a track to fix", with(1, PathTerminator::TrackToFix), {arc}, Reason::InvalidWaypoint, 1);
    refuse("a course to fix without its course", with(1, PathTerminator::CourseToFix), {}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.centerLatitudeRad = bad.centerLongitudeRad = kHold;
    refuse("an arc without its centre", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.clockwise = kHold;
    refuse("an arc without its way round", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.centerLongitudeRad = kHold;
    refuse("a centre's latitude alone", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    // a leg its segment does not define: a navaid's, a procedure turn's
    for (const PathTerminator t : {PathTerminator::ArcToFix, PathTerminator::CourseToDmeDistance, PathTerminator::CourseToRadial,
                                   PathTerminator::TrackFromFixToDmeDistance, PathTerminator::ProcedureTurnToIntercept,
                                   PathTerminator::HeadingToDmeDistanceTermination, PathTerminator::HeadingToRadialTermination})
        refuse("a leg not defined", with(2, t), {}, Reason::InvalidWaypoint, 2);
    // not built: FA-6f2's
    for (const PathTerminator t : {PathTerminator::CourseToAltitude, PathTerminator::CourseToIntercept, PathTerminator::TrackToAltitude,
                                   PathTerminator::TrackFromFixToDistanceAlongTrack, PathTerminator::FixToManualTermination,
                                   PathTerminator::HoldingWithAltitudeTermination, PathTerminator::HoldingWithFixTermination,
                                   PathTerminator::HoldingWithManualTermination, PathTerminator::HeadingToAltitude,
                                   PathTerminator::HeadingToIntercept, PathTerminator::HeadingToManual})
        refuse("FA-6f2's", with(2, t), {}, Reason::NotImplemented, 2);
    // an arc not its own: its end off its circle, the long way round, a radius, courses, places or lengths not its
    p = with(1, PathTerminator::RadiusToFix);
    p.at(1) = at(lat0, lon0, -r + 100.0, 3000.0 + r - 100.0);
    p.at(1).terminator = code(PathTerminator::RadiusToFix);
    refuse("its end off its circle", p, {arc}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.clockwise = 0.0;
    refuse("the long way round", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.radiusM = 1450.0;
    refuse("a radius not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.courseInRad = 0.0;
    refuse("a course in not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.courseOutRad = 0.5 * kPi;
    refuse("a course out not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.initialLatitudeRad = base[2].latitudeRad, bad.initialLongitudeRad = base[2].longitudeRad;
    refuse("a start not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.arcM = 2000.0;
    refuse("a length not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    bad = arc, bad.chordM = 2000.0;
    refuse("a chord not its", with(1, PathTerminator::RadiusToFix), {bad}, Reason::InvalidWaypoint, 1);
    // an arc the route never flies from the point before: at its start (from where the aircraft is); a branch to it (from the
    // start, on to the arc's point: named at the branch's point)
    RouteTerminator first = arc;
    first.point = 0;
    refuse("an arc at a route's start", with(0, PathTerminator::RadiusToFix), {first}, Reason::InvalidWaypoint, 0);
    RouteBranch toArc;
    toArc.point = 0, toArc.next = 1.0;
    refuse("a branch to an arc", with(1, PathTerminator::RadiusToFix), {arc}, Reason::InvalidWaypoint, 0, {toArc});
    // after a start turn point its arc - a track to fix there refused; after a capture its course - a direct to fix refused
    p = with(1, PathTerminator::TrackToFix);
    p.at(0).turn = static_cast<double>(TurnType::StartTurn);
    refuse("a track to fix after a start turn", p, {}, Reason::InvalidWaypoint, 1);
    p = with(1, PathTerminator::DirectToFix);
    p.at(0).turn = static_cast<double>(TurnType::CaptureOutboundCourse), p.at(0).courseRad = std::atan2(r, -r);
    refuse("a direct to fix after a capture", p, {}, Reason::InvalidWaypoint, 1);
    // a course to fix whose point before is past it along its course
    RouteTerminator back;
    back.point = 1, back.courseRad = 0.5 * kPi * 3.0; // (west: its point before, west of it, is ahead of it that way)
    refuse("a course to fix its point before is past", with(1, PathTerminator::CourseToFix), {back}, Reason::InvalidWaypoint, 1);
    // an arc after an orbit's loiter point (left where its circle's tangent runs on, not at its point)
    p = with(1, PathTerminator::RadiusToFix);
    p.at(0).kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter orbit;
    orbit.point = 0, orbit.pattern.pattern = static_cast<double>(PatternKind::Orbit), orbit.pattern.durationS = 60.0;
    refuse("an arc after an orbit", p, {arc}, Reason::InvalidWaypoint, 1, {}, {orbit});
    // an arc of 100 m, tighter than its full bank turns at its speed
    std::vector<Waypoint> tight = {base[0], at(lat0, lon0, -100.0, 3100.0), at(lat0, lon0, -3100.0, 3100.0)};
    tight[1].terminator = code(PathTerminator::RadiusToFix);
    refuse("an arc too tight", tight, {arcRound(1, lat0, lon0, -100.0, 3000.0, true)}, Reason::InvalidWaypoint, 1);
    // 65: the 65th's point
    std::vector<Waypoint> many;
    std::vector<RouteTerminator> tfs;
    for (std::uint32_t i = 0; i < 66; ++i) {
        many.push_back(at(lat0, lon0, 0.0, 3000.0 + 1000.0 * i));
        many.back().terminator = code(PathTerminator::CourseToFix);
        RouteTerminator t;
        t.point = i, t.courseRad = 0.5 * kPi;
        if (i < 65) tfs.push_back(t);
    }
    refuse("65", many, tfs, Reason::InvalidWaypoint, 64);
    // named as given where its flight order is not: a point aside (its first's next the arc's point), the arc's data wrong
    // and its radius not its own, each at its point as given
    std::vector<Waypoint> skip = {base[0], at(lat0, lon0, 3000.0, 3000.0), base[1], base[2]};
    skip.at(0).next = 2.0, skip.at(2).terminator = code(PathTerminator::RadiusToFix);
    RouteTerminator aside = arcRound(2, lat0, lon0, -r, 3000.0, true);
    const CommandResult linked = w.submit(v, RouteCommand{}, skip, {}, {}, {}, {}, {}, std::vector<RouteTerminator>{aside});
    INFO(reasonName(linked.reason) << " at " << linked.index);
    CHECK(linked.accepted());
    bad = aside, bad.clockwise = 2.0;
    refuse("data wrong, linked", skip, {bad}, Reason::InvalidWaypoint, 2);
    bad = aside, bad.radiusM = 1450.0;
    refuse("a radius not its, linked", skip, {bad}, Reason::InvalidWaypoint, 2);
    // what is fine is flown: the arc as given
    const CommandResult ok = w.submit(v, RouteCommand{}, with(1, PathTerminator::RadiusToFix), {}, {}, {}, {}, {}, std::vector<RouteTerminator>{arc});
    INFO(reasonName(ok.reason) << " at " << ok.index);
    CHECK(ok.accepted());
}

TEST_CASE("route terminators: a stack on its own takes them too", "[modes]") {
    // the aircraft at its route's first point, east: on to the second round a radius to fix's arc from there
    sim::VehicleState s;
    s.latitudeRad = 37.6 * kDeg, s.longitudeRad = -122.4 * kDeg;
    s.altitudeMslM = s.altitudeAglM = 1500.0;
    s.airspeedTrueMs = s.velocityBodyMs[0] = s.velocityNedMs[1] = 60.0; // (east)
    s.eulerRad[2] = 0.5 * kPi;
    s.loadFactor = 1.0;
    Rng rng{1};
    sim::ControlInputs out;
    ControlContext ctx{1, s, s, 1.0 / 120.0, nullptr, &rng};
    const double r = 1500.0;
    std::vector<Waypoint> points = {at(s.latitudeRad, s.longitudeRad, 0.0, 0.0), at(s.latitudeRad, s.longitudeRad, -r, r),
                                    at(s.latitudeRad, s.longitudeRad, -r - 3000.0, r)};
    points[1].terminator = code(PathTerminator::RadiusToFix);
    ControlStack stack;
    stack.command(RouteCommand{}, points, {}, {}, {}, {}, std::vector<RouteTerminator>{arcRound(1, s.latitudeRad, s.longitudeRad, -r, 0.0, true)});
    for (int i = 0; i < 10; ++i) stack.update(ctx, out);
    ActivityProgress g;
    REQUIRE(stack.progress(0, g));
    CHECK(g.segment == 1);
    CHECK(std::abs(g.crossTrackM) < 1.0);
    CHECK(g.distanceToGoM > r * 0.5 * kPi); // (its arc, and on)
}
