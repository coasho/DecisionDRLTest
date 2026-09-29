// A must fly into a zone (docs/flight-autonomy.md, 4.43; ADR-29 FA-8b1): A-GRA's ZoneTarget and OpZone - a polygon with holes,
// an ellipse, a rectangle, a slant range area, within a band of altitudes, on the Earth or in a frame, moving - entered: the
// route aims a little inside its nearest point (or its edge the window's way), and the must fly completes once the aircraft is
// in it.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

void placeFrom(const sim::VehicleState& s, double north, double east, double& lat, double& lon) {
    lat = s.latitudeRad + north / kR;
    lon = s.longitudeRad + east / (kR * std::cos(s.latitudeRad));
}

ZoneVertex vertex(const sim::VehicleState& s, double north, double east) {
    ZoneVertex v;
    placeFrom(s, north, east, v.latitudeRad, v.longitudeRad);
    return v;
}

/// A place north and east of a reference, in metres.
void offset(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    north = (lat - lat0) * kR;
    east = (lon - lon0) * kR * std::cos(0.5 * (lat + lat0));
}

/// Where each must fly ended, and where its aircraft was then.
struct Ended {
    ActivityState state = ActivityState::Active;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeM = 0.0, trackRad = 0.0, timeS = 0.0;
};

std::vector<Ended> flyUntilEnded(session::World& w, const std::vector<std::pair<std::uint32_t, ActivityId>>& all, double seconds) {
    std::vector<Ended> out(all.size());
    std::vector<bool> done(all.size(), false);
    for (int i = 0; i < static_cast<int>(seconds * 10.0); ++i) {
        fly(w, 0.1);
        bool live = false;
        for (std::size_t k = 0; k < all.size(); ++k) {
            if (done[k]) continue;
            const ActivityRecord* r = w.activity(all[k].second);
            if (r->live()) {
                live = true;
                continue;
            }
            const sim::VehicleState& s = *w.vehicleState(all[k].first);
            out[k] = Ended{r->state, s.latitudeRad, s.longitudeRad, s.altitudeMslM, track(s), w.simTime()};
            done[k] = true;
        }
        if (!live) break;
    }
    return out;
}

} // namespace

TEST_CASE("must fly: a zone of each shape entered, and completed once in it - per class (MFY-04)", "[modes][must_fly]") {
    session::World w(options("must-fly-zones"));
    const auto iris = rotor(w, "iris", 15.0, 6), uh60 = rotor(w, "uh60", 15.0, 9);
    const auto c172 = wing(w, "c172x", 1500.0, 55.0), f16 = wing(w, "f16c", 3000.0, 160.0, 3);
    const sim::VehicleState a = *w.vehicleState(c172), b = *w.vehicleState(f16), c = *w.vehicleState(uh60), d = *w.vehicleState(iris);
    // an ellipse 5 km east of the C172, 1.5 by 0.8 km, its major axis north
    OpZone ellipse;
    ellipse.shape = static_cast<double>(ZoneShape::Ellipse);
    placeFrom(a, 0.0, 5000.0, ellipse.latitudeRad, ellipse.longitudeRad);
    ellipse.semiMajorM = 1500.0, ellipse.semiMinorM = 800.0;
    // a rectangle 10 km north-east of the F-16C, 3 km across and 6 km along 30 degrees
    OpZone rectangle;
    rectangle.shape = static_cast<double>(ZoneShape::Rectangle);
    placeFrom(b, 7000.0, 7000.0, rectangle.latitudeRad, rectangle.longitudeRad);
    rectangle.widthM = 3000.0, rectangle.heightM = 6000.0, rectangle.orientationRad = 30.0 * kDeg;
    // a polygon ahead of the UH-60A, a hole in its middle
    OpZone polygon;
    polygon.shape = static_cast<double>(ZoneShape::Polygon);
    polygon.vertices = {vertex(c, 300.0, -300.0), vertex(c, 900.0, -300.0), vertex(c, 900.0, 300.0), vertex(c, 300.0, 300.0)};
    polygon.holes = {{vertex(c, 500.0, -100.0), vertex(c, 700.0, -100.0), vertex(c, 700.0, 100.0), vertex(c, 500.0, 100.0)}};
    // a slant range area 200 m north of the IRIS: 50 to 150 m out, bearings 150 to 210 degrees (its side toward the IRIS)
    OpZone slant;
    slant.shape = static_cast<double>(ZoneShape::SlantRange);
    placeFrom(d, 200.0, 0.0, slant.latitudeRad, slant.longitudeRad);
    slant.rangeMinM = 50.0, slant.rangeMaxM = 150.0, slant.azimuthMinRad = 150.0 * kDeg, slant.azimuthMaxRad = -150.0 * kDeg;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Zone);
    std::vector<std::pair<std::uint32_t, ActivityId>> all;
    const std::pair<std::uint32_t, const OpZone*> cases[] = {{c172, &ellipse}, {f16, &rectangle}, {uh60, &polygon}, {iris, &slant}};
    for (const auto& [v, zone] : cases) {
        const CommandResult r = w.submit(v, into, *zone);
        INFO(v << ": " << reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        all.emplace_back(v, r.activity);
    }
    const std::vector<Ended> ended = flyUntilEnded(w, all, 200.0);
    for (const Ended& e : ended) CHECK(e.state == ActivityState::Completed);
    // each in its zone as it completed: within a metre of it
    double n = 0.0, e = 0.0;
    offset(ellipse.latitudeRad, ellipse.longitudeRad, ended[0].latitudeRad, ended[0].longitudeRad, n, e);
    CHECK((n / 1500.0) * (n / 1500.0) + (e / 800.0) * (e / 800.0) <= 1.0 + 2.0 / 800.0);
    offset(rectangle.latitudeRad, rectangle.longitudeRad, ended[1].latitudeRad, ended[1].longitudeRad, n, e);
    const double along = n * std::cos(30.0 * kDeg) + e * std::sin(30.0 * kDeg), across = -n * std::sin(30.0 * kDeg) + e * std::cos(30.0 * kDeg);
    CHECK((std::abs(along) <= 3001.0 && std::abs(across) <= 1501.0));
    offset(c.latitudeRad, c.longitudeRad, ended[2].latitudeRad, ended[2].longitudeRad, n, e);
    CHECK((n >= 299.0 && n <= 901.0 && std::abs(e) <= 301.0 && !(n > 501.0 && n < 699.0 && std::abs(e) < 99.0)));
    offset(slant.latitudeRad, slant.longitudeRad, ended[3].latitudeRad, ended[3].longitudeRad, n, e);
    const double range = std::hypot(n, e), bearing = std::atan2(e, n);
    CHECK((range >= 49.0 && range <= 151.0 && std::abs(std::remainder(bearing - kPi, 2.0 * kPi)) <= 30.5 * kDeg));
    CHECK(w.supportTable(c172)->find("fsim.guidance.must_fly")->support == Support::Partial);
}

TEST_CASE("must fly: a zone's band climbed into, and its window of bearings kept - on the Earth and in a turned frame", "[modes][must_fly]") {
    session::World w(options("must-fly-zone-band"));
    const auto banded = wing(w, "c172x", 1500.0, 55.0), windowed = wing(w, "c172x", 1500.0, 55.0, 3);
    const auto turnedWindow = wing(w, "c172x", 1500.0, 55.0, 6);
    const sim::VehicleState a = *w.vehicleState(banded), b = *w.vehicleState(windowed), c = *w.vehicleState(turnedWindow);
    OpZone circle; // (a circle 5 km east, 1,700 to 1,900 m up: 200 m above the aircraft)
    circle.shape = static_cast<double>(ZoneShape::Ellipse);
    placeFrom(a, 0.0, 5000.0, circle.latitudeRad, circle.longitudeRad);
    circle.semiMajorM = circle.semiMinorM = 1500.0, circle.altitudeMinM = 1700.0, circle.altitudeMaxM = 1900.0;
    OpZone north = circle; // (a circle 6 km east, entered from the north: bearings from its centre 350 to 10 degrees)
    placeFrom(b, 0.0, 6000.0, north.latitudeRad, north.longitudeRad);
    north.semiMajorM = north.semiMinorM = 1000.0, north.altitudeMinM = north.altitudeMaxM = kHold;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Zone);
    const CommandResult r1 = w.submit(banded, into, circle);
    into.ingressMinRad = -10.0 * kDeg, into.ingressMaxRad = 10.0 * kDeg;
    const CommandResult r2 = w.submit(windowed, into, north);
    // the same, in a frame turned 90 degrees (its x east): the window's bearings are true ones, turned into its plane
    FrameSpec turned;
    turned.latitudeRad = c.latitudeRad, turned.longitudeRad = c.longitudeRad, turned.altitudeMslM = 1500.0, turned.yawRad = 90.0 * kDeg;
    const FrameId frame = w.createFrame(turned);
    REQUIRE(frame != 0);
    OpZone inFrame = north;
    inFrame.latitudeRad = inFrame.longitudeRad = kHold, inFrame.xM = 6000.0, inFrame.yM = 0.0;
    inFrame.frame = static_cast<double>(frame), inFrame.frameRotation = static_cast<double>(FrameRotation::Yaw);
    const CommandResult r3 = w.submit(turnedWindow, into, inFrame);
    REQUIRE((r1.accepted() && r2.accepted() && r3.accepted()));
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r2.activity, sp));
    CHECK(sp.waypoints.size() >= 2); // (round to its north, then in)
    const std::vector<Ended> ended = flyUntilEnded(w, {{banded, r1.activity}, {windowed, r2.activity}, {turnedWindow, r3.activity}}, 300.0);
    CHECK(ended[0].state == ActivityState::Completed);
    CHECK((ended[0].altitudeM >= 1699.0 && ended[0].altitudeM <= 1901.0));
    CHECK(ended[1].state == ActivityState::Completed);
    INFO("came from " << std::remainder(ended[1].trackRad + kPi, 2.0 * kPi) / kDeg);
    CHECK(std::abs(std::remainder(ended[1].trackRad + kPi, 2.0 * kPi)) <= 12.0 * kDeg); // (heading south: from the north)
    CHECK(ended[2].state == ActivityState::Completed);
    INFO("in the frame, came from " << std::remainder(ended[2].trackRad + kPi, 2.0 * kPi) / kDeg);
    CHECK(std::abs(std::remainder(ended[2].trackRad + kPi, 2.0 * kPi)) <= 12.0 * kDeg); // (from the north too, not along its x)
}

TEST_CASE("must fly: an operational zone kept by its id, one in a frame and one moving; refusals naming its fields (MFY-03, ENV-06)",
          "[modes][must_fly]") {
    session::World w(options("must-fly-op-zones"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), framed = wing(w, "c172x", 1500.0, 55.0, 3), moving = wing(w, "c172x", 1500.0, 55.0, 6);
    const sim::VehicleState s = *w.vehicleState(v);
    OpZone square;
    square.id = 12, square.shape = static_cast<double>(ZoneShape::Polygon);
    square.vertices = {vertex(s, -1000.0, 4000.0), vertex(s, 1000.0, 4000.0), vertex(s, 1000.0, 6000.0), vertex(s, -1000.0, 6000.0)};
    square.altitudeMinM = 1000.0, square.altitudeMaxM = 2000.0;
    CHECK(w.setOpZone(square) == Reason::None);
    CHECK(w.opZone(12)->revision == 1);
    CHECK(w.setOpZone(square) == Reason::None);
    CHECK(w.opZone(12)->revision == 2);
    CHECK(w.opZones() == std::vector<OpZoneId>{12});
    CHECK(w.opZone(12)->vertices.size() == 4);
    MustFlyCommand byId;
    byId.location = static_cast<double>(MustFlyLocation::OpZone), byId.target = 12.0;
    const CommandResult r = w.submit(v, byId);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    // a rectangle in a fixed frame turned 90 degrees (its x east), 4 km along x from the second aircraft; a circle 4 km east of
    // the third, moving north at 15 m/s
    const sim::VehicleState f = *w.vehicleState(framed), m = *w.vehicleState(moving);
    FrameSpec turned;
    turned.latitudeRad = f.latitudeRad, turned.longitudeRad = f.longitudeRad, turned.altitudeMslM = 1500.0, turned.yawRad = 90.0 * kDeg;
    const FrameId frame = w.createFrame(turned);
    REQUIRE(frame != 0);
    OpZone boxed;
    boxed.shape = static_cast<double>(ZoneShape::Rectangle);
    boxed.frame = static_cast<double>(frame), boxed.frameRotation = static_cast<double>(FrameRotation::Yaw);
    boxed.xM = 4000.0, boxed.yM = 0.0, boxed.widthM = 1000.0, boxed.heightM = 1000.0;
    OpZone drifting;
    drifting.shape = static_cast<double>(ZoneShape::Ellipse);
    placeFrom(m, -1500.0, 4000.0, drifting.latitudeRad, drifting.longitudeRad);
    drifting.semiMajorM = drifting.semiMinorM = 500.0, drifting.northMs = 15.0, drifting.eastMs = 0.0;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Zone);
    const CommandResult rf = w.submit(framed, into, boxed), rm = w.submit(moving, into, drifting);
    INFO(reasonName(rf.reason) << " " << reasonName(rm.reason));
    REQUIRE((rf.accepted() && rm.accepted()));
    const std::vector<Ended> ended = flyUntilEnded(w, {{v, r.activity}, {framed, rf.activity}, {moving, rm.activity}}, 300.0);
    for (const Ended& e : ended) CHECK(e.state == ActivityState::Completed);
    double n = 0.0, e = 0.0;
    offset(f.latitudeRad, f.longitudeRad, ended[1].latitudeRad, ended[1].longitudeRad, n, e); // (the box: 3.5 to 4.5 km east, 500 m either side)
    CHECK((e >= 3499.0 && e <= 4501.0 && std::abs(n) <= 501.0));
    // the moving circle where it was then: its centre carried on from when it was given
    const double centreNorth = -1500.0 + 15.0 * (ended[2].timeS - w.activity(rm.activity)->startTime);
    offset(m.latitudeRad, m.longitudeRad, ended[2].latitudeRad, ended[2].longitudeRad, n, e);
    CHECK(std::hypot(n - centreNorth, e - 4000.0) <= 501.0);
    // one the world does not keep, or removed; and zones A-GRA's schema would not take, named from its field 10
    byId.target = 13.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    CHECK(w.removeOpZone(12));
    CHECK_FALSE(w.removeOpZone(12));
    byId.target = 12.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    auto refusal = [&](void (*spoil)(OpZone&)) {
        OpZone z = square;
        z.id = 0, z.altitudeMinM = z.altitudeMaxM = kHold;
        spoil(z);
        const CommandResult bad = w.submit(v, into, z);
        return bad.reason == Reason::InvalidParameter ? static_cast<int>(bad.index) : -100;
    };
    CHECK(refusal([](OpZone& z) { z.shape = 7.0; }) == 10);
    CHECK(refusal([](OpZone& z) { z.vertices.resize(2); }) == 11);
    CHECK(refusal([](OpZone& z) { std::swap(z.vertices[1], z.vertices[2]); }) == 11); // (a bow tie: not simple)
    CHECK(refusal([](OpZone& z) {
              std::vector<ZoneVertex> far = {z.vertices[0], z.vertices[1], z.vertices[2]};
              for (ZoneVertex& q : far) q.latitudeRad += 0.01;
              z.holes = {far};
          }) == 12); // (not inside it)
    CHECK(refusal([](OpZone& z) { z.latitudeRad = 0.5; }) == 13);                     // (a polygon has no centre)
    CHECK(refusal([](OpZone& z) {
              z.shape = static_cast<double>(ZoneShape::Ellipse), z.vertices.clear(), z.latitudeRad = 0.6, z.longitudeRad = -2.1;
              z.semiMajorM = 100.0, z.semiMinorM = 200.0;
          }) == 14);                                                                 // (its minor above its major)
    CHECK(refusal([](OpZone& z) {
              z.shape = static_cast<double>(ZoneShape::Rectangle), z.vertices.clear(), z.latitudeRad = 0.6, z.longitudeRad = -2.1;
              z.widthM = z.heightM = 100.0, z.orientationRad = 2.0;
          }) == 15);                                                                 // (beyond a quarter turn)
    CHECK(refusal([](OpZone& z) { z.altitudeMinM = 2000.0, z.altitudeMaxM = 1000.0; }) == 16);
    CHECK(refusal([](OpZone& z) { z.frame = 99.0; }) == 17);
    CHECK(refusal([](OpZone& z) { z.northMs = 5.0; }) == 18);                         // (a velocity one way)
    CHECK(w.submit(v, into).reason == Reason::InvalidParameter);                       // (a zone's, given none)
    // an altitude given outside its band, or in another reference: it would never be entered
    OpZone banded = square;
    banded.id = 0;
    MustFlyCommand at = into;
    at.altitudeM = 2500.0;
    CommandResult outside = w.submit(v, at, banded);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 3));
    at.altitudeM = 1500.0, at.altitudeReference = static_cast<double>(AltitudeReference::AboveGround);
    outside = w.submit(v, at, banded);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 4));
    OpZone kept = square;
    kept.id = 0;
    CHECK(w.setOpZone(kept) == Reason::InvalidParameter);
    CHECK(w.supportTable(v)->find("fsim.geometry")->support == Support::Partial);
}

TEST_CASE("must fly: a zone's UPDATE flies to the new one; queued with its zone, it starts later", "[modes][must_fly]") {
    session::World w(options("must-fly-zone-update"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), later = wing(w, "c172x", 1500.0, 55.0, 3);
    const sim::VehicleState s = *w.vehicleState(v);
    OpZone near;
    near.shape = static_cast<double>(ZoneShape::Ellipse);
    placeFrom(s, 0.0, 8000.0, near.latitudeRad, near.longitudeRad);
    near.semiMajorM = near.semiMinorM = 500.0;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Zone);
    const CommandResult r = w.submit(v, into, near);
    REQUIRE(r.accepted());
    fly(w, 10.0);
    OpZone north = near; // (moved 3 km north)
    placeFrom(s, 3000.0, 8000.0, north.latitudeRad, north.longitudeRad);
    CHECK(w.update(r.activity, MustFlyCommand{}, north).accepted());
    // one queued behind a start window, its zone kept with it
    const sim::VehicleState t = *w.vehicleState(later);
    OpZone ahead = near;
    placeFrom(t, 0.0, 5000.0, ahead.latitudeRad, ahead.longitudeRad);
    CommandOptions wait;
    wait.window.startNotBefore = w.simTime() + 20.0;
    const CommandResult q = w.submit(later, into, ahead, wait);
    REQUIRE(q.accepted());
    CHECK((q.flags & kDeferred) != 0);
    const std::vector<Ended> ended = flyUntilEnded(w, {{v, r.activity}, {later, q.activity}}, 300.0);
    CHECK(ended[0].state == ActivityState::Completed);
    double n = 0.0, e = 0.0;
    offset(north.latitudeRad, north.longitudeRad, ended[0].latitudeRad, ended[0].longitudeRad, n, e);
    CHECK(std::hypot(n, e) <= 501.0);
    INFO("the queued one: " << reasonName(w.activity(q.activity)->reason));
    CHECK(ended[1].state == ActivityState::Completed);
    offset(ahead.latitudeRad, ahead.longitudeRad, ended[1].latitudeRad, ended[1].longitudeRad, n, e);
    CHECK(std::hypot(n, e) <= 501.0);
}
