// A curve's reference and control points as A-GRA's schema gives them (docs/flight-autonomy.md, 4.27; ADR-29 FA-5d2):
// its altitude in a reference - above the ground over rising terrain - and within a range; its points' third read
// three ways, flown alike; laid out along great circles and rhumb lines, measured against those lines computed here;
// turned with a moving frame and carried by it; a vehicle's frame gone; and what does not make a reference refused,
// naming the field.
#include "control/Route.h"
#include "fsim/Frames.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// A clamped cubic through its points, knots evenly within.
NurbsSegment cubic(const std::vector<std::array<double, 3>>& points) {
    NurbsSegment s;
    s.points = static_cast<std::uint32_t>(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) s.north[i] = points[i][0], s.east[i] = points[i][1], s.down[i] = points[i][2];
    const int interior = static_cast<int>(points.size()) - 4;
    s.knots = s.points + 4;
    for (int i = 0; i < 4; ++i) s.knot[i] = 0.0, s.knot[s.knots - 1 - static_cast<std::uint32_t>(i)] = 1.0;
    for (int i = 0; i < interior; ++i) s.knot[4 + i] = (i + 1.0) / (interior + 1.0);
    return s;
}

/// A straight cubic `length` along `course` (rad) in its plane, from (n0, e0), its height `down0` to `down1`.
NurbsSegment straight(double n0, double e0, double course, double length, double down0, double down1) {
    std::vector<std::array<double, 3>> p;
    for (int i = 0; i < 4; ++i) {
        const double u = i / 3.0;
        p.push_back({n0 + u * length * std::cos(course), e0 + u * length * std::sin(course), down0 + u * (down1 - down0)});
    }
    return cubic(p);
}

double code(AltitudeReference r) { return static_cast<double>(r); }

/// Great-circle destination (the sphere of kR): from (lat, lon) `d` metres on course `c`.
void greatCircle(double lat, double lon, double c, double d, double& latOut, double& lonOut) {
    const double a = d / kR;
    latOut = std::asin(std::sin(lat) * std::cos(a) + std::cos(lat) * std::sin(a) * std::cos(c));
    lonOut = lon + std::atan2(std::sin(c) * std::sin(a) * std::cos(lat), std::cos(a) - std::sin(lat) * std::sin(latOut));
}

/// Metres between two places over the sphere (haversine).
double apart(double lat1, double lon1, double lat2, double lon2) {
    const double a = std::pow(std::sin(0.5 * (lat2 - lat1)), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(0.5 * (lon2 - lon1)), 2);
    return 2.0 * kR * std::asin(std::sqrt(a));
}

} // namespace

TEST_CASE("curve references: above the ground over rising terrain; within its altitude range; its points' third read three ways, flown alike",
          "[modes]") {
    // above the ground: a level curve 10 km east at 400 m over ground rising 2 % to the east
    {
        const sim::InitialConditions start;
        auto o = options("curve-agl");
        o.ground = std::make_shared<RisingGround>(start.longitudeDeg * kDeg, start.latitudeDeg * kDeg, 0.02);
        session::World w(o);
        const auto v = wing(w, "c172x", 500.0, 55.0);
        CurveCommand c;
        c.altitudeM = 400.0, c.altitudeReference = code(AltitudeReference::AboveGround);
        const NurbsSegment east = straight(0.0, 0.0, 0.5 * kPi, 10000.0, 0.0, 0.0);
        const CommandResult r = w.submit(v, c, Span<const NurbsSegment>(&east, 1));
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        double worst = 0.0;
        for (unsigned k = 0; k < stepsFor(w, 170.0); ++k) {
            w.step();
            if (k > stepsFor(w, 60.0) && w.activity(r.activity)->live()) worst = std::max(worst, std::abs(w.vehicleState(v)->altitudeAglM - 400.0));
        }
        const auto& s = *w.vehicleState(v);
        std::printf("curve references, above the ground: within %.1f m of 400 m over the ground, the ground %.0f m higher\n", worst,
                    s.altitudeMslM - s.altitudeAglM);
        CHECK(worst < 10.0);
        CHECK(s.altitudeMslM - s.altitudeAglM > 150.0); // (it climbed with the ground)
    }
    // its range: left out, the aircraft's altitude held within it; given, within it; a range upside down
    {
        session::World w(options("curve-range"));
        const auto v = wing(w, "c172x", 500.0, 55.0);
        w.step(stepsFor(w, 1.0));
        const NurbsSegment east = straight(0.0, 0.0, 0.5 * kPi, 5000.0, 0.0, 0.0);
        CurveCommand ranged;
        ranged.altitudeMinM = 1000.0, ranged.altitudeMaxM = 1200.0;
        CommandResult r = w.submit(v, ranged, Span<const NurbsSegment>(&east, 1));
        REQUIRE(r.accepted());
        Setpoint flown;
        REQUIRE(w.activitySetpoint(r.activity, flown));
        CHECK(std::get<CurveCommand>(std::get<Command>(flown.command)).altitudeM == 1000.0); // (500 m, held up to its least)
        ranged.altitudeM = 800.0;
        r = w.submit(v, ranged, Span<const NurbsSegment>(&east, 1));
        CHECK(r.reason == Reason::InvalidParameter);
        CHECK(r.index == 2);
        ranged.altitudeM = kHold, ranged.altitudeMinM = 1300.0;
        r = w.submit(v, ranged, Span<const NurbsSegment>(&east, 1));
        CHECK(r.reason == Reason::InvalidParameter);
        CHECK(r.index == 10);
    }
    // its points' third: down, an altitude offset up, an absolute altitude - the same climbing curve, flown alike
    std::array<sim::VehicleState, 3> ends{};
    for (int z = 0; z < 3; ++z) {
        session::World w(options("curve-z"));
        const auto v = wing(w, "c172x", 500.0, 55.0);
        CurveCommand c;
        c.altitudeM = 500.0, c.pointZ = static_cast<double>(z);
        NurbsSegment climb = straight(0.0, 0.0, 0.5 * kPi, 6000.0, 0.0, -150.0); // (down: 150 m up at its end)
        for (std::uint32_t i = 0; i < climb.points; ++i)
            climb.down[i] = z == 0 ? climb.down[i] : z == 1 ? -climb.down[i] : 500.0 - climb.down[i];
        REQUIRE(w.submit(v, c, Span<const NurbsSegment>(&climb, 1)).accepted());
        w.step(stepsFor(w, 90.0));
        ends[static_cast<std::size_t>(z)] = *w.vehicleState(v);
    }
    std::printf("curve references, the third read three ways: %.3f, %.3f and %.3f m\n", ends[0].altitudeMslM, ends[1].altitudeMslM, ends[2].altitudeMslM);
    CHECK(ends[0].altitudeMslM == ends[1].altitudeMslM); // (an offset up is down negated: the same numbers)
    CHECK(ends[0].latitudeRad == ends[1].latitudeRad);
    CHECK(std::abs(ends[0].altitudeMslM - ends[2].altitudeMslM) < 1e-3); // (an absolute altitude less its reference's: within rounding)
    CHECK(ends[0].altitudeMslM > 560.0); // (it climbed)
}

TEST_CASE("curve references: laid out along great circles and rhumb lines; turned with a moving frame, carried by it; its vehicle gone", "[modes]") {
    session::World w(options("curve-frames"));
    const auto leader = rotor(w, "uh1h", 15.0, 9), follower = rotor(w, "iris", 15.0, 9);
    const auto gc = wing(w, "c172x", 1000.0, 55.0), rh = wing(w, "c172x", 1000.0, 55.0, 3), shipped = wing(w, "c172x", 1000.0, 55.0, 6);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    // along a great circle: 30 km east - its end where the great circle east from its reference ends (in the platform's
    // plane, where it was: 30 km along its parallel)
    const sim::VehicleState g0 = *w.vehicleState(gc);
    const NurbsSegment east = straight(0.0, 0.0, 0.5 * kPi, 30000.0, 0.0, 0.0);
    CommandResult a = w.submit(gc, CurveCommand{}, Span<const NurbsSegment>(&east, 1));
    REQUIRE(a.accepted());
    double eLat, eLon;
    greatCircle(g0.latitudeRad, g0.longitudeRad, 0.5 * kPi, 30000.0, eLat, eLon);
    std::vector<EndPoint> ends = w.endPoints(a.activity, 4);
    REQUIRE(!ends.empty());
    const double gcPlainOff = apart(ends[0].latitudeRad, ends[0].longitudeRad, eLat, eLon);
    CurveCommand alongGc;
    alongGc.pointOffsets = static_cast<double>(FrameOffsets::GreatCircle);
    a = w.submit(gc, alongGc, Span<const NurbsSegment>(&east, 1));
    REQUIRE(a.accepted());
    ends = w.endPoints(a.activity, 4);
    REQUIRE(!ends.empty());
    const double gcEndOff = apart(ends[0].latitudeRad, ends[0].longitudeRad, eLat, eLon);
    // along a rhumb line: 25 km north-east - its end where that rhumb line ends (north along the meridian, east its
    // departure); likewise in the plane first
    const sim::VehicleState r0 = *w.vehicleState(rh);
    const NurbsSegment diagonal = straight(0.0, 0.0, 0.25 * kPi, 25000.0, 0.0, 0.0);
    const double dn = 25000.0 * std::cos(0.25 * kPi), de = 25000.0 * std::sin(0.25 * kPi);
    const double rLat = r0.latitudeRad + dn / kR;
    auto mercator = [](double lat) { return std::log(std::tan(0.25 * kPi + 0.5 * lat)); };
    const double rLon = r0.longitudeRad + de / (kR * (rLat - r0.latitudeRad) / (mercator(rLat) - mercator(r0.latitudeRad)));
    CommandResult b = w.submit(rh, CurveCommand{}, Span<const NurbsSegment>(&diagonal, 1));
    REQUIRE(b.accepted());
    ends = w.endPoints(b.activity, 4);
    REQUIRE(!ends.empty());
    const double rhumbPlainOff = apart(ends[0].latitudeRad, ends[0].longitudeRad, rLat, rLon);
    CurveCommand alongRhumb;
    alongRhumb.pointOffsets = static_cast<double>(FrameOffsets::Rhumb);
    b = w.submit(rh, alongRhumb, Span<const NurbsSegment>(&diagonal, 1));
    REQUIRE(b.accepted());
    ends = w.endPoints(b.activity, 4);
    REQUIRE(!ends.empty());
    const double rhumbEndOff = apart(ends[0].latitudeRad, ends[0].longitudeRad, rLat, rLon);
    // turned with a moving frame: a ship heading 60 degrees at 8 m/s; the curve 8 km abeam of it to starboard, along its
    // y axis (150 degrees), the aircraft 2 km short of its start - carried across the curve by the ship, 1.5 km by its end
    const sim::VehicleState s0 = *w.vehicleState(shipped);
    FrameSpec ship;
    ship.origin = FrameOrigin::Moving;
    ship.latitudeRad = s0.latitudeRad + 2000.0 * std::cos(150.0 * kDeg) / kR;
    ship.longitudeRad = s0.longitudeRad + 2000.0 * std::sin(150.0 * kDeg) / (kR * std::cos(s0.latitudeRad));
    ship.yawRad = 60.0 * kDeg, ship.northMs = 8.0 * std::cos(60.0 * kDeg), ship.eastMs = 8.0 * std::sin(60.0 * kDeg), ship.timeS = w.simTime();
    const FrameId shipId = w.createFrame(ship);
    REQUIRE(shipId != 0);
    CurveCommand turned;
    turned.pointRotation = static_cast<double>(FrameRotation::Yaw);
    CurveShape onShip;
    onShip.frame = static_cast<double>(shipId);
    const NurbsSegment abeam = straight(0.0, 0.0, 0.5 * kPi, 8000.0, 0.0, 0.0);
    const CommandResult c = w.submit(shipped, turned, Span<const NurbsSegment>(&abeam, 1), {}, &onShip);
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    // carried by a vehicle: the IRIS along a curve from 20 m east of the UH-1H, 150 m north, as the UH-1H flies north at
    // 2 m/s; the UH-1H gone before its end
    FrameSpec other;
    other.origin = FrameOrigin::Vehicle, other.vehicle = leader;
    const FrameId otherId = w.createFrame(other);
    VelocityCommand northward;
    northward.northMs = 2.0, northward.eastMs = 0.0, northward.verticalSpeedMs = 0.0;
    REQUIRE(w.submit(leader, northward).accepted());
    CurveShape beside;
    beside.frame = static_cast<double>(otherId), beside.frameYM = 20.0;
    CurveCommand carried;
    carried.speedMinMs = carried.speedMaxMs = 3.0;
    const NurbsSegment up = straight(0.0, 0.0, 0.0, 150.0, 0.0, 0.0);
    const CommandResult d = w.submit(follower, carried, Span<const NurbsSegment>(&up, 1), {}, &beside);
    REQUIRE(d.accepted());

    auto crossRhumb = [&](const sim::VehicleState& s) { // (off the rhumb line at 45 degrees: its departure east, across)
        const double lonOn = r0.longitudeRad + (mercator(s.latitudeRad) - mercator(r0.latitudeRad)); // (tan 45 degrees)
        return std::abs((s.longitudeRad - lonOn) * kR * std::cos(s.latitudeRad) * std::cos(0.25 * kPi));
    };
    double gcOff = 0.0, rhumbOff = 0.0, shipOff = 0.0, carriedOff = 0.0, shipTrack = 0.0, stillAfter = 0.0;
    int shipTracks = 0;
    for (unsigned k = 0; k < stepsFor(w, 560.0); ++k) {
        w.step();
        const double t = w.simTime() - t0;
        // the great circle's line: the aircraft's distance from it, past its first 5 km
        const auto& s = *w.vehicleState(gc);
        if (w.activity(a.activity)->live()) {
            const double along = apart(g0.latitudeRad, g0.longitudeRad, s.latitudeRad, s.longitudeRad);
            if (along > 5000.0) {
                double best = 1e18; // (the nearest of a few along it: the line bends little)
                for (double da = -60.0; da <= 60.0; da += 2.0) {
                    double lat, lon;
                    greatCircle(g0.latitudeRad, g0.longitudeRad, 0.5 * kPi, along + da, lat, lon);
                    best = std::min(best, apart(s.latitudeRad, s.longitudeRad, lat, lon));
                }
                gcOff = std::max(gcOff, best);
            }
        }
        if (w.activity(b.activity)->live() && t > 90.0) rhumbOff = std::max(rhumbOff, crossRhumb(*w.vehicleState(rh)));
        // the ship's curve: from the ship's point now, abeam - off it, and over the ship along it
        if (w.activity(c.activity)->live() && t > 80.0) {
            const FramePose pose = framePose(ship, w.simTime());
            const auto& q = *w.vehicleState(shipped);
            const double n = (q.latitudeRad - pose.latitudeRad) * kR, e = (q.longitudeRad - pose.longitudeRad) * kR * std::cos(pose.latitudeRad);
            const double line = pose.yawRad + 0.5 * kPi;
            shipOff = std::max(shipOff, std::abs(-n * std::sin(line) + e * std::cos(line)));
            const double vn = q.velocityNedMs[0] - ship.northMs, ve = q.velocityNedMs[1] - ship.eastMs;
            shipTrack += std::remainder(std::atan2(ve, vn) - line, 2.0 * kPi), ++shipTracks;
        }
        // the IRIS's: 20 m east of the UH-1H
        if (w.activity(d.activity)->live() && t > 20.0 && t < 30.0) {
            const auto& l = *w.vehicleState(leader);
            const auto& f = *w.vehicleState(follower);
            carriedOff = std::max(carriedOff, std::abs((f.longitudeRad - l.longitudeRad) * kR * std::cos(l.latitudeRad) - 20.0));
        }
        if (k == stepsFor(w, 30.0)) { // the curve not yet done: the UH-1H gone
            REQUIRE(w.removeVehicle(leader));
        }
        if (t > 60.0) stillAfter = std::max(stillAfter, groundSpeed(*w.vehicleState(follower)));
    }
    const ActivityRecord& lost = *w.activity(d.activity);
    std::printf("curve references: a great circle's end %.2f m from its own (in the plane %.1f m), flown %.1f m off it; a rhumb line's end %.2f m "
                "from its own (in the plane %.1f m), flown %.1f m off it;\n  a ship's abeam, across it %.1f m, its track over the ship %.2f deg off; "
                "beside a UH-1H within %.2f m, it gone: %s (%s), holding (%.2f m/s)\n",
                gcEndOff, gcPlainOff, gcOff, rhumbEndOff, rhumbPlainOff, rhumbOff, shipOff, shipTracks ? shipTrack / shipTracks / kDeg : 0.0, carriedOff,
                activityStateName(lost.state), reasonName(lost.reason), stillAfter);
    CHECK(gcEndOff < 1.0);
    CHECK(gcPlainOff > 40.0);
    CHECK(gcOff < 15.0);
    CHECK(rhumbEndOff < 1.0);
    CHECK(rhumbPlainOff > 10.0);
    CHECK(rhumbOff < 15.0);
    CHECK(shipTracks > 0);
    CHECK(shipOff < 10.0); // (left where it was put, 1.5 km; its cross-track in turned axes read as if north-up, 579 m)
    CHECK(std::abs(shipTrack / std::max(shipTracks, 1)) < 1.0 * kDeg);
    CHECK(carriedOff < 0.5);
    CHECK(lost.state == ActivityState::Failed);
    CHECK(lost.reason == Reason::TargetLost);
    CHECK(stillAfter < 1.0);
}

TEST_CASE("curve references: its points turned in three dimensions - by a fixed frame's attitude, climbing with its pitch, an absolute "
          "altitude kept; by a vehicle's as it flies, every step",
          "[modes]") {
    session::World w(options("curve-attitude"));
    const auto leader = rotor(w, "uh1h", 15.0, 9), follower = rotor(w, "iris", 15.0, 9);
    const auto v = wing(w, "c172x", 1000.0, 55.0);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    // a frame where the aircraft is, heading east as it does and pitched 2 degrees up: a curve 6 km along its x climbs 209 m
    // by its end, where it reads back - its points' absolute altitudes kept, their x and y turned - and it is flown so
    const sim::VehicleState s0 = *w.vehicleState(v);
    FrameSpec pitched;
    pitched.latitudeRad = s0.latitudeRad, pitched.longitudeRad = s0.longitudeRad, pitched.altitudeMslM = s0.altitudeMslM;
    pitched.yawRad = 0.5 * kPi, pitched.pitchRad = 2.0 * kDeg;
    const FrameId id = w.createFrame(pitched);
    REQUIRE(id != 0);
    CurveCommand turned;
    turned.pointRotation = static_cast<double>(FrameRotation::Attitude);
    CurveShape in;
    in.frame = static_cast<double>(id);
    CurveCommand absolute = turned;
    absolute.pointZ = static_cast<double>(CurveZ::AbsoluteAltitude);
    NurbsSegment level = straight(0.0, 0.0, 0.0, 6000.0, 0.0, 0.0);
    for (std::uint32_t i = 0; i < level.points; ++i) level.down[i] = s0.altitudeMslM + 50.0; // (its third the altitude itself)
    CommandResult r = w.submit(v, absolute, Span<const NurbsSegment>(&level, 1), {}, &in);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    std::vector<EndPoint> ends = w.endPoints(r.activity, 4);
    REQUIRE(!ends.empty());
    const double keptEast = (ends[0].longitudeRad - s0.longitudeRad) * kR * std::cos(s0.latitudeRad), keptUp = ends[0].altitudeM - s0.altitudeMslM;
    const NurbsSegment ahead = straight(0.0, 0.0, 0.0, 6000.0, 0.0, 0.0);
    r = w.submit(v, turned, Span<const NurbsSegment>(&ahead, 1), {}, &in);
    REQUIRE(r.accepted());
    ends = w.endPoints(r.activity, 4);
    REQUIRE(!ends.empty());
    const double endEast = (ends[0].longitudeRad - s0.longitudeRad) * kR * std::cos(s0.latitudeRad), endUp = ends[0].altitudeM - s0.altitudeMslM;
    // in a vehicle's body axes as it flies, turned afresh every step: the IRIS along a line from 20 m to the UH-1H's right, along
    // its nose, as the UH-1H flies north at 2 m/s - in the UH-1H's axes, 20 m to its right and level with it
    FrameSpec body;
    body.origin = FrameOrigin::Vehicle, body.vehicle = leader;
    const FrameId bodyId = w.createFrame(body);
    VelocityCommand northward;
    northward.northMs = 2.0, northward.eastMs = 0.0, northward.verticalSpeedMs = 0.0;
    REQUIRE(w.submit(leader, northward).accepted());
    CurveShape right;
    right.frame = static_cast<double>(bodyId), right.frameRotation = static_cast<double>(FrameRotation::Attitude), right.frameYM = 20.0;
    CurveCommand along = turned;
    along.speedMinMs = along.speedMaxMs = 3.0;
    const NurbsSegment nose = straight(0.0, 0.0, 0.0, 200.0, 0.0, 0.0);
    const CommandResult b = w.submit(follower, along, Span<const NurbsSegment>(&nose, 1), {}, &right);
    REQUIRE(b.accepted());
    double sideOff = 0.0, levelOff = 0.0, pitchMost = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 140.0); ++k) {
        w.step();
        if (w.simTime() - t0 < 20.0 || w.simTime() - t0 > 60.0) continue;
        const auto& l = *w.vehicleState(leader);
        const auto& f = *w.vehicleState(follower);
        const route::Attitude m(vehiclePose(l)); // (north, east and down from its body's axes: turned back)
        const double n = (f.latitudeRad - l.latitudeRad) * kR, e = (f.longitudeRad - l.longitudeRad) * kR * std::cos(l.latitudeRad),
                     d = l.altitudeMslM - f.altitudeMslM;
        const double y = m.m[0][1] * n + m.m[1][1] * e + m.m[2][1] * d, z = m.m[0][2] * n + m.m[1][2] * e + m.m[2][2] * d;
        sideOff = std::max(sideOff, std::abs(y - 20.0)), levelOff = std::max(levelOff, std::abs(z));
        pitchMost = std::max(pitchMost, std::abs(l.eulerRad[1]));
    }
    const double flownUp = w.vehicleState(v)->altitudeMslM - s0.altitudeMslM;
    std::printf("curve references, in three dimensions: its end %.1f m east and %.2f m up (absolute: %.1f m east, %.3f m up), flown %.1f m up; "
                "in the UH-1H's axes %.2f m off its side and %.2f m off its level, the UH-1H pitched up to %.1f deg\n",
                endEast, endUp, keptEast, keptUp, flownUp, sideOff, levelOff, pitchMost / kDeg);
    CHECK(std::abs(endEast - 6000.0 * std::cos(2.0 * kDeg)) < 1.0);
    CHECK(std::abs(endUp - 6000.0 * std::sin(2.0 * kDeg)) < 0.5);
    CHECK(std::abs(keptEast - 6000.0 * std::cos(2.0 * kDeg)) < 1.0);
    CHECK(std::abs(keptUp - 50.0) < 1e-9);
    CHECK(std::abs(flownUp - 6000.0 * std::sin(2.0 * kDeg)) < 15.0);
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    CHECK(sideOff < 1.0);
    CHECK(levelOff < 1.5); // (0.84 m: the line swings up and down with the UH-1H's pitch, a metre a degree 60 m along it)
}

TEST_CASE("curve references: what does not make a reference is refused, naming the field; where a curve is changes only with its segments",
          "[modes]") {
    session::World w(options("curve-reference-refusals"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    const NurbsSegment east = straight(0.0, 0.0, 0.5 * kPi, 5000.0, 0.0, 0.0);
    auto refused = [&](const CurveCommand& c, const CurveShape* shape, Reason reason, int index) {
        const CommandResult r = w.submit(v, c, Span<const NurbsSegment>(&east, 1), {}, shape);
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == reason);
        CHECK(r.index == index);
    };
    CurveCommand c;
    c.altitudeReference = 4.0; // (a reference is 0 to 3)
    refused(c, nullptr, Reason::InvalidParameter, 8);
    c = CurveCommand{}, c.pointOffsets = 3.0;
    refused(c, nullptr, Reason::InvalidParameter, 12);
    c = CurveCommand{}, c.pointZ = 3.0;
    refused(c, nullptr, Reason::InvalidParameter, 13);
    c = CurveCommand{}, c.pointRotation = static_cast<double>(FrameRotation::Yaw); // (turned with no frame to turn with)
    refused(c, nullptr, Reason::InvalidParameter, 11);
    FrameSpec here;
    here.latitudeRad = w.vehicleState(v)->latitudeRad, here.longitudeRad = w.vehicleState(v)->longitudeRad, here.altitudeMslM = 1500.0;
    const FrameId id = w.createFrame(here);
    CurveShape framed;
    framed.frame = static_cast<double>(id);
    c = CurveCommand{};
    CurveShape unknown;
    unknown.frame = static_cast<double>(id + 100);
    refused(c, &unknown, Reason::InvalidParameter, 14);
    CurveShape loose;
    loose.frameYM = 5.0; // (offsets without their frame)
    refused(c, &loose, Reason::InvalidParameter, 18);
    // accepted in the frame, 2 km north of its origin and 200 m up: read back placed there
    CurveShape up;
    up.frame = static_cast<double>(id), up.frameXM = 2000.0, up.frameZM = -200.0;
    const CommandResult r = w.submit(v, c, Span<const NurbsSegment>(&east, 1), {}, &up);
    REQUIRE(r.accepted());
    Setpoint flown;
    REQUIRE(w.activitySetpoint(r.activity, flown));
    const auto& placed = std::get<CurveCommand>(std::get<Command>(flown.command));
    CHECK(std::abs((placed.latitudeRad - here.latitudeRad) * kR - 2000.0) < 1.0);
    CHECK(std::abs(placed.altitudeM - 1700.31) < 0.05); // (the plane at the origin: 0.31 m above the sphere 2 km out)
    CHECK(flown.curveShape.frame == static_cast<double>(id));
    // where it is changes only with its segments: its reference, its points' reading, its frame, given with its options alone
    CurveCommand moved;
    moved.altitudeM = 1600.0;
    CommandResult u = w.update(r.activity, moved, Span<const NurbsSegment>{});
    CHECK(u.reason == Reason::InvalidParameter);
    CHECK(u.index == 2);
    moved = CurveCommand{}, moved.pointOffsets = static_cast<double>(FrameOffsets::GreatCircle);
    u = w.update(r.activity, moved, Span<const NurbsSegment>{});
    CHECK(u.index == 12);
    u = w.update(r.activity, CurveCommand{}, Span<const NurbsSegment>{}, &framed);
    CHECK(u.reason == Reason::InvalidParameter);
    CHECK(u.index == 14);
    // with segments, a point leaves the frame; a reference given alone has no value to read
    moved = CurveCommand{}, moved.latitudeRad = here.latitudeRad, moved.longitudeRad = here.longitudeRad;
    u = w.update(r.activity, moved, Span<const NurbsSegment>(&east, 1));
    REQUIRE(u.accepted());
    REQUIRE(w.activitySetpoint(r.activity, flown));
    CHECK(isHold(flown.curveShape.frame));
    moved = CurveCommand{}, moved.altitudeReference = code(AltitudeReference::AboveGround);
    u = w.update(r.activity, moved, Span<const NurbsSegment>(&east, 1));
    CHECK(u.reason == Reason::InvalidParameter);
    CHECK(u.index == 8);
    moved = CurveCommand{}, moved.speedMaxMs = 70.0; // (its options alone: how it is flown)
    CHECK(w.update(r.activity, moved, Span<const NurbsSegment>{}).accepted());
    // segments appended go on from its reference, their points read as its (A-GRA's append uses the preceding command's
    // CenterReference): a reference or a reading given with them is not used
    const NurbsSegment further = straight(0.0, 5000.0, 0.5 * kPi, 5000.0, 0.0, 0.0);
    CurveCommand append;
    append.append = 1.0, append.pointZ = static_cast<double>(CurveZ::AltitudeOffset), append.latitudeRad = here.latitudeRad + 0.01;
    append.longitudeRad = here.longitudeRad;
    REQUIRE(w.update(r.activity, append, Span<const NurbsSegment>(&further, 1)).accepted());
    REQUIRE(w.activitySetpoint(r.activity, flown));
    CHECK(flown.nurbs.size() == 2);
    const auto& kept = std::get<CurveCommand>(std::get<Command>(flown.command));
    CHECK(kept.latitudeRad == here.latitudeRad);
    CHECK(isHold(kept.pointZ));
}
