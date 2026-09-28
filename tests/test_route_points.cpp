// A route's waypoints as A-GRA's schema gives them (docs/flight-autonomy.md, 4.29; ADR-29 FA-6a): an altitude block
// holding a point's altitude, a barometric altitude flown on its isobar; a waypoint flown over, and what each type
// asks; points in frames - a fixed one's placed, a ship's route flown over the ship, a leg to a moving point pursued, a
// rotorcraft over a moving point, a vehicle's gone - and what does not make a point refused, naming it.
#include "fsim/Altimeter.h"
#include "fsim/Frames.h"
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

double code(AltitudeReference r) { return static_cast<double>(r); }

/// A point `north` and `east` metres from where `s` is.
Waypoint at(const sim::VehicleState& s, double north, double east) {
    Waypoint p;
    p.latitudeRad = s.latitudeRad + north / kR, p.longitudeRad = s.longitudeRad + east / (kR * std::cos(s.latitudeRad));
    return p;
}

/// A point in frame `frame`, `x` and `y` metres along its axes (unturned: north and east).
Waypoint in(FrameId frame, double x, double y) {
    Waypoint p;
    p.frame = static_cast<double>(frame), p.frameXM = x, p.frameYM = y;
    return p;
}

/// North and east metres of (lat, lon) from (lat0, lon0).
void apart(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    north = (lat - lat0) * kR, east = (lon - lon0) * kR * std::cos(lat0);
}

Setpoint flown(session::World& w, ActivityId a) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(a, s));
    return s;
}

} // namespace

TEST_CASE("route points: an altitude block holds a point's altitude within it; a barometric altitude is flown on its isobar", "[modes]") {
    session::World w(options("route-points-blocks"));
    sim::EnvironmentState env = w.environment();
    env.temperatureSeaLevelK = 303.15, env.pressureSeaLevelPa = 102000.0; // warm, and high
    w.setEnvironment(env);
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    REQUIRE(w.setQnh(v, 102000.0) == Reason::None);
    const Altimeter a{Air{303.15, 102000.0}, 102000.0};
    const sim::VehicleState s0 = *w.vehicleState(v);
    // east: 1,500 m, then a block of 1,700 to 1,900 m left to choose (the point before's held up to its least), then 2,000 m
    // on the altimeter
    Waypoint p = at(s0, 0.0, 5000.0), q = at(s0, 0.0, 10000.0), r = at(s0, 0.0, 24000.0);
    p.altitudeM = 1500.0;
    q.altitudeMinM = 1700.0, q.altitudeMaxM = 1900.0;
    r.altitudeM = 2000.0, r.altitudeReference = code(AltitudeReference::Barometric);
    auto refusedAt = [&](const std::vector<Waypoint>& points, Reason why, int index) {
        const CommandResult c = w.submit(v, RouteCommand{}, points);
        INFO(reasonName(c.reason) << " at " << c.index);
        CHECK(c.reason == why);
        CHECK(c.index == index);
    };
    Waypoint outside = q;
    outside.altitudeM = 1600.0;
    refusedAt({p, outside, r}, Reason::InvalidWaypoint, 1);
    Waypoint upsideDown = q;
    upsideDown.altitudeMinM = 2000.0;
    refusedAt({p, upsideDown, r}, Reason::InvalidWaypoint, 1);
    const CommandResult c = w.submit(v, RouteCommand{}, std::vector<Waypoint>{p, q, r});
    INFO(reasonName(c.reason) << " at " << c.index);
    REQUIRE(c.accepted());
    const Setpoint sp = flown(w, c.activity);
    REQUIRE(sp.waypoints.size() == 3);
    CHECK(sp.waypoints[1].altitudeM == 1700.0);
    CHECK(sp.waypoints[1].altitudeMinM == 1700.0);
    CHECK(sp.waypoints[1].altitudeMaxM == 1900.0);
    // flown: over the block's point at 1,700 m, and on along its last leg on the isobar of 2,000 m on the altimeter
    double atBlock = kHold, nearest = 1e18;
    for (unsigned k = 0; k < stepsFor(w, 560.0); ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        double n, e;
        apart(q.latitudeRad, q.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
        if (std::hypot(n, e) < nearest) nearest = std::hypot(n, e), atBlock = s.altitudeMslM;
    }
    const StateData d = w.stateData(v);
    std::printf("route points, a block: over its point at %.1f m (1,700); on the altimeter %.1f m (2,000), %.1f m above sea level (its isobar %.1f)\n",
                atBlock, d.indicatedAltitudeM, w.vehicleState(v)->altitudeMslM, barometricMslM(a, 2000.0));
    CHECK(std::abs(atBlock - 1700.0) < 20.0);
    CHECK(w.activity(c.activity)->state == ActivityState::Completed);
    CHECK(std::abs(d.indicatedAltitudeM - 2000.0) < 15.0);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - barometricMslM(a, 2000.0)) < 15.0);
    CHECK(w.vehicleState(v)->altitudeMslM - 2000.0 > 80.0);
}

TEST_CASE("route points: a waypoint is flown over, as its type asks; the types not built are refused not implemented, naming the point",
          "[modes]") {
    session::World w(options("route-points-types"));
    const auto byer = wing(w, "c172x", 1500.0, 55.0), over = wing(w, "c172x", 1500.0, 55.0, 3);
    const sim::VehicleState s0 = *w.vehicleState(byer), s1 = *w.vehicleState(over);
    // a corner 4 km east: a turn point turned short of it; a waypoint - no turn there - flown over
    const Waypoint a = at(s0, 0.0, 4000.0), b = at(s0, 4000.0, 4000.0);
    Waypoint c = at(s1, 0.0, 4000.0);
    const Waypoint d = at(s1, 4000.0, 4000.0);
    c.kind = static_cast<double>(EndPointKind::Waypoint);
    const CommandResult cut = w.submit(byer, RouteCommand{}, std::vector<Waypoint>{a, b});
    const CommandResult passed = w.submit(over, RouteCommand{}, std::vector<Waypoint>{c, d});
    REQUIRE((cut.accepted() && passed.accepted()));
    double nearA = 1e18, nearC = 1e18;
    for (unsigned k = 0; k < stepsFor(w, 150.0); ++k) {
        w.step();
        double n, e;
        apart(a.latitudeRad, a.longitudeRad, w.vehicleState(byer)->latitudeRad, w.vehicleState(byer)->longitudeRad, n, e);
        nearA = std::min(nearA, std::hypot(n, e));
        apart(c.latitudeRad, c.longitudeRad, w.vehicleState(over)->latitudeRad, w.vehicleState(over)->longitudeRad, n, e);
        nearC = std::min(nearC, std::hypot(n, e));
    }
    std::vector<EndPoint> ends = w.endPoints(passed.activity, 4);
    std::printf("route points, a corner: a turn point passed %.0f m off (turned short), a waypoint %.1f m (flown over)\n", nearA, nearC);
    CHECK(nearA > 100.0);
    CHECK(nearC < 30.0);
    // its type: nav only and passive flown, the end of a path its last point; given alone, a waypoint
    const sim::VehicleState s = *w.vehicleState(byer);
    Waypoint t0 = at(s, 0.0, 3000.0), t1 = at(s, 0.0, 6000.0), t2 = at(s, 0.0, 9000.0);
    t0.waypointType = static_cast<double>(WaypointType::NavOnly), t1.waypointType = static_cast<double>(WaypointType::Passive);
    t2.waypointType = static_cast<double>(WaypointType::EndOfPath);
    const CommandResult typed = w.submit(byer, RouteCommand{}, std::vector<Waypoint>{t0, t1, t2});
    INFO(reasonName(typed.reason) << " at " << typed.index);
    REQUIRE(typed.accepted());
    CHECK(flown(w, typed.activity).waypoints[1].kind == static_cast<double>(EndPointKind::Waypoint));
    ends = w.endPoints(typed.activity, 4);
    REQUIRE(ends.size() == 3);
    CHECK(ends[0].kind == EndPointKind::Waypoint); // (reported as given: no turn there)
    // not built yet: each named at its point, not implemented - a taxi's and a runway's and a takeoff's points (FA-9), an
    // approach's and a touchdown (FA-10), a ditch (FA-16)
    auto refusedAt = [&](const std::vector<Waypoint>& points, Reason why, int index) {
        const CommandResult r = w.submit(byer, RouteCommand{}, points);
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == index);
    };
    const struct {
        WaypointType type;
        const char* row;
    } unbuilt[] = {{WaypointType::Taxi, "taxi"},
                   {WaypointType::RunwayStart, "runway"},
                   {WaypointType::RunwayThreshold, "runway"},
                   {WaypointType::RunwayLimit, "runway"},
                   {WaypointType::Takeoff, "takeoff"},
                   {WaypointType::TakeoffInitialPoint, "takeoff"},
                   {WaypointType::TakeoffFinalPoint, "takeoff"},
                   {WaypointType::Approach, "landing"},
                   {WaypointType::ApproachInitialPoint, "landing"},
                   {WaypointType::ApproachFinalPoint, "landing"},
                   {WaypointType::Touchdown, "landing"},
                   {WaypointType::HardDitch, "hard_ditch"}};
    for (const auto& u : unbuilt) { // (as its row in the support table says: not supported where the aircraft cannot)
        Waypoint x = t1;
        x.waypointType = static_cast<double>(u.type);
        const SupportInfo* row = w.supportTable(byer)->find(std::string("fsim.guidance.route/waypoint_type/") + u.row);
        INFO(static_cast<int>(u.type) << " " << u.row);
        REQUIRE(row);
        CHECK((row->support == Support::NotImplemented || row->support == Support::NotSupported));
        refusedAt({t0, x, t2}, row->support == Support::NotSupported ? Reason::NotSupported : Reason::NotImplemented, 1);
    }
    Waypoint early = t1; // (the end of a path where it goes on is none: 4.36 - not implemented until FA-6e1 built paths)
    early.waypointType = static_cast<double>(WaypointType::EndOfPath);
    refusedAt({t0, early, t2}, Reason::InvalidWaypoint, 1);
    Waypoint loiter = t1; // (a loiter point is flown with its loiter beside it: without one, it is no point - 4.31)
    loiter.waypointType = kHold, loiter.kind = static_cast<double>(EndPointKind::LoiterPoint);
    refusedAt({t0, loiter, t2}, Reason::InvalidWaypoint, 1);
    // what does not make a point: a type with a turn point, a kind or a type that is not one
    Waypoint turned = t1;
    turned.kind = static_cast<double>(EndPointKind::TurnPoint);
    refusedAt({t0, turned, t2}, Reason::InvalidWaypoint, 1);
    Waypoint notKind = t0;
    notKind.waypointType = kHold, notKind.kind = 3.0;
    refusedAt({t0, notKind, t2}, Reason::InvalidWaypoint, 1);
    Waypoint notType = t0;
    notType.waypointType = 15.0;
    refusedAt({t0, notType, t2}, Reason::InvalidWaypoint, 1);
}

TEST_CASE("route points: in frames - a fixed one's placed; a ship's route flown over the ship; a leg to a moving point pursued; a rotorcraft over a "
          "moving point; a vehicle's gone",
          "[modes]") {
    session::World w(options("route-points-frames"));
    const auto leader = rotor(w, "uh1h", 15.0, 9), follower = rotor(w, "iris", 15.0, 9), hoverer = rotor(w, "iris", 15.0, 12);
    const auto placed = wing(w, "c172x", 1000.0, 55.0), boxer = wing(w, "c172x", 1000.0, 55.0, 3), chaser = wing(w, "c172x", 1000.0, 55.0, 6);
    w.step(stepsFor(w, 2.0));
    const double t0 = w.simTime();
    // a fixed frame 5 km east, heading east: its origin and 4 km along its x, turned with its yaw - placed there
    const sim::VehicleState p0 = *w.vehicleState(placed);
    FrameSpec fixed;
    fixed.latitudeRad = p0.latitudeRad, fixed.longitudeRad = p0.longitudeRad + 5000.0 / (kR * std::cos(p0.latitudeRad)), fixed.yawRad = 0.5 * kPi;
    const FrameId fixedId = w.createFrame(fixed);
    Waypoint f0 = in(fixedId, 0.0, 0.0), f1 = in(fixedId, 4000.0, 0.0);
    f1.frameRotation = static_cast<double>(FrameRotation::Yaw);
    const CommandResult fr = w.submit(placed, RouteCommand{}, std::vector<Waypoint>{f0, f1});
    INFO(reasonName(fr.reason) << " at " << fr.index);
    REQUIRE(fr.accepted());
    const Setpoint fs = flown(w, fr.activity);
    double n, e;
    apart(fixed.latitudeRad, fixed.longitudeRad, fs.waypoints[1].latitudeRad, fs.waypoints[1].longitudeRad, n, e);
    CHECK(std::abs(n) < 1.0);
    CHECK(std::abs(e - 4000.0) < 1.0);
    CHECK(fs.waypoints[1].frame == static_cast<double>(fixedId));
    // a ship 4 km north of a C172x, moving north at 8 m/s: round a box about it, 3 km a side, again and again
    const sim::VehicleState b0 = *w.vehicleState(boxer);
    FrameSpec ship;
    ship.origin = FrameOrigin::Moving;
    ship.latitudeRad = b0.latitudeRad + 4000.0 / kR, ship.longitudeRad = b0.longitudeRad, ship.northMs = 8.0, ship.timeS = w.simTime();
    const FrameId shipId = w.createFrame(ship);
    const double box[4][2] = {{1500.0, 1500.0}, {-1500.0, 1500.0}, {-1500.0, -1500.0}, {1500.0, -1500.0}};
    std::vector<Waypoint> round;
    for (const auto& corner : box) round.push_back(in(shipId, corner[0], corner[1]));
    RouteCommand again;
    again.repeat = 1.0;
    const CommandResult br = w.submit(boxer, again, round);
    REQUIRE(br.accepted());
    // a chaser: a point 2 km east of it, then two of a second ship's points, 3 km off, flown over - its legs to them pursuing them
    const sim::VehicleState c0 = *w.vehicleState(chaser);
    FrameSpec ship2 = ship;
    ship2.latitudeRad = c0.latitudeRad + 3000.0 / kR, ship2.longitudeRad = c0.longitudeRad + 3000.0 / (kR * std::cos(c0.latitudeRad));
    const FrameId ship2Id = w.createFrame(ship2);
    std::vector<Waypoint> chase = {at(c0, 0.0, 2000.0), in(ship2Id, 0.0, 0.0), in(ship2Id, 0.0, 2500.0)};
    chase[1].turn = chase[2].turn = static_cast<double>(TurnType::FlyOver);
    const CommandResult cr = w.submit(chaser, RouteCommand{}, chase);
    REQUIRE(cr.accepted());
    // a rotorcraft ending over a point a frame carries east at 2 m/s: hovering over it as it moves
    const sim::VehicleState h0 = *w.vehicleState(hoverer);
    FrameSpec raft;
    raft.origin = FrameOrigin::Moving, raft.latitudeRad = h0.latitudeRad, raft.longitudeRad = h0.longitudeRad, raft.eastMs = 2.0, raft.timeS = w.simTime();
    const FrameId raftId = w.createFrame(raft);
    RouteCommand stop;
    stop.end = static_cast<double>(EndBehavior::Loiter);
    const CommandResult hr = w.submit(hoverer, stop, std::vector<Waypoint>{in(raftId, 30.0, 0.0), in(raftId, 30.0, 30.0)});
    REQUIRE(hr.accepted());
    // an IRIS round two points in a UH-1H's frame, the UH-1H gone
    FrameSpec other;
    other.origin = FrameOrigin::Vehicle, other.vehicle = leader;
    const FrameId otherId = w.createFrame(other);
    const CommandResult lr = w.submit(follower, stop, std::vector<Waypoint>{in(otherId, 0.0, 40.0), in(otherId, 40.0, 40.0)});
    REQUIRE(lr.accepted());

    double boxOff = 0.0, overShip = 0.0, near1 = 1e18, near2 = 1e18, hoverOff = 0.0;
    int boxSamples = 0;
    for (unsigned k = 0; k < stepsFor(w, 420.0); ++k) {
        w.step();
        const double t = w.simTime();
        // the box: off the leg to the corner flown to, where the ship is now, from the middle third of each leg of its second lap on
        const ActivityProgress& g = w.activity(br.activity)->progress;
        if (g.laps >= 1 && g.segmentPercent > 33.0 && g.segmentPercent < 67.0) {
            const FramePose pose = framePose(ship, t);
            const auto& s = *w.vehicleState(boxer);
            apart(pose.latitudeRad, pose.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
            const auto& to = box[g.segment];
            const auto& from = box[(g.segment + 3) % 4];
            const double dn = to[0] - from[0], de = to[1] - from[1], length = std::hypot(dn, de);
            boxOff = std::max(boxOff, std::abs(-(n - from[0]) * de / length + (e - from[1]) * dn / length));
            const double vn = s.velocityNedMs[0] - ship.northMs, ve = s.velocityNedMs[1] - ship.eastMs;
            overShip = std::max(overShip, std::abs(std::remainder(std::atan2(ve, vn) - std::atan2(de, dn), 2.0 * kPi)));
            ++boxSamples;
        }
        // the chaser: over each of the second ship's points, as they are when it passes
        if (w.activity(cr.activity)->live()) {
            const FramePose pose = framePose(ship2, t);
            const auto& s = *w.vehicleState(chaser);
            apart(pose.latitudeRad, pose.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
            near1 = std::min(near1, std::hypot(n, e)), near2 = std::min(near2, std::hypot(n, e - 2500.0));
        }
        // the rotorcraft: over its moving point, from a minute on
        if (t - t0 > 60.0) {
            const FramePose pose = framePose(raft, t);
            const auto& s = *w.vehicleState(hoverer);
            apart(pose.latitudeRad, pose.longitudeRad, s.latitudeRad, s.longitudeRad, n, e);
            hoverOff = std::max(hoverOff, std::hypot(n - 30.0, e - 30.0));
        }
        if (k == stepsFor(w, 20.0)) REQUIRE(w.removeVehicle(leader)); // (before the IRIS's last point)
    }
    // its end points where the ship is now
    const std::vector<EndPoint> ends = w.endPoints(br.activity, 4);
    REQUIRE(!ends.empty());
    const FramePose now = framePose(ship, w.simTime());
    const ActivityProgress& g = w.activity(br.activity)->progress;
    apart(now.latitudeRad, now.longitudeRad, ends[0].latitudeRad, ends[0].longitudeRad, n, e);
    const double endOff = std::hypot(n - box[g.segment][0], e - box[g.segment][1]);
    const ActivityRecord& lost = *w.activity(lr.activity);
    std::printf("route points, frames: a ship's box flown %.1f m off its legs (%d samples, %u laps), %.2f deg off them over the ship; its end point "
                "%.2f m from the ship's corner now; a chaser over the ship's points %.1f and %.1f m (%s); a rotorcraft over a moving point within "
                "%.2f m; the UH-1H gone: %s (%s)\n",
                boxOff, boxSamples, g.laps, overShip / kDeg, endOff, near1, near2, activityStateName(w.activity(cr.activity)->state), hoverOff,
                activityStateName(lost.state), reasonName(lost.reason));
    CHECK(boxSamples > 100);
    CHECK(boxOff < 30.0);
    CHECK(overShip < 5.0 * kDeg);
    CHECK(endOff < 1.0);
    CHECK(near1 < 60.0);
    CHECK(near2 < 60.0);
    CHECK(w.activity(cr.activity)->state == ActivityState::Completed);
    CHECK(hoverOff < 1.0);
    CHECK(lost.state == ActivityState::Failed);
    CHECK(lost.reason == Reason::TargetLost);
}

TEST_CASE("route points: what does not make a point in a frame is refused, naming the point", "[modes]") {
    session::World w(options("route-points-refusals"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState s = *w.vehicleState(v);
    FrameSpec here;
    here.latitudeRad = s.latitudeRad, here.longitudeRad = s.longitudeRad;
    const FrameId id = w.createFrame(here);
    auto refusedAt = [&](const std::vector<Waypoint>& points, Reason why, int index) {
        const CommandResult r = w.submit(v, RouteCommand{}, points);
        INFO(reasonName(r.reason) << " at " << r.index);
        CHECK(r.reason == why);
        CHECK(r.index == index);
    };
    const Waypoint first = at(s, 0.0, 3000.0);
    refusedAt({first, in(id + 100, 0.0, 5000.0)}, Reason::InvalidWaypoint, 1); // (not the world's)
    Waypoint loose = at(s, 0.0, 6000.0);
    loose.frameYM = 10.0; // (offsets without their frame)
    refusedAt({first, loose}, Reason::InvalidWaypoint, 1);
    Waypoint turned = in(id, 0.0, 5000.0);
    turned.frameRotation = 4.0; // (a rotation is 0 to 3)
    refusedAt({first, turned}, Reason::InvalidWaypoint, 1);
    // seventeen frames: one more than a route's table has room for
    std::vector<Waypoint> many;
    for (int k = 0; k < 17; ++k) {
        FrameSpec f = here;
        f.longitudeRad += (k + 1) * 2000.0 / (kR * std::cos(s.latitudeRad));
        many.push_back(in(w.createFrame(f), 0.0, 0.0));
    }
    refusedAt(many, Reason::InvalidWaypoint, 16);
    many.pop_back();
    const CommandResult r = w.submit(v, RouteCommand{}, many);
    INFO(reasonName(r.reason) << " at " << r.index);
    CHECK(r.accepted()); // (sixteen: taken)
}
