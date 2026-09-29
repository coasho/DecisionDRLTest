// A must fly into a volume (docs/flight-autonomy.md, 4.45; ADR-29 FA-8b3): A-GRA's VolumeTarget and OpVolume - a sphere, a dome,
// an ellipsoid, a cylinder, a cone, a rectangular cone at a point (on the Earth or in a frame, turned, moving), or a geocentric
// box - entered: at a height inside it, toward a point well inside, and completed once the aircraft is in it.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
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

/// North and east of a place, m.
void offsetFrom(double lat0, double lon0, const sim::VehicleState& s, double& north, double& east) {
    north = (s.latitudeRad - lat0) * kR;
    east = (s.longitudeRad - lon0) * kR * std::cos(lat0);
}

/// Where each must fly ended, and where its aircraft was then.
struct Ended {
    ActivityState state = ActivityState::Active;
    sim::VehicleState s{};
    double timeS = 0.0;
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
            out[k] = Ended{r->state, *w.vehicleState(all[k].first), w.simTime()};
            done[k] = true;
        }
        if (!live) break;
    }
    return out;
}

OpVolume at(const sim::VehicleState& s, VolumeShape shape, double north, double east, double altitudeM) {
    OpVolume v;
    v.shape = static_cast<double>(shape);
    placeFrom(s, north, east, v.latitudeRad, v.longitudeRad);
    v.altitudeM = altitudeM;
    return v;
}

} // namespace

TEST_CASE("must fly: a volume of each shape entered, and completed once in it - per class (MFY-06)", "[modes][must_fly]") {
    session::World w(options("must-fly-volumes"));
    const auto iris = rotor(w, "iris", 15.0, 6), uh60 = rotor(w, "uh60", 15.0, 9);
    const auto c172 = wing(w, "c172x", 1500.0, 55.0), f16 = wing(w, "f16c", 3000.0, 160.0, 3);
    const sim::VehicleState a = *w.vehicleState(c172), b = *w.vehicleState(f16), c = *w.vehicleState(uh60), d = *w.vehicleState(iris);
    // a sphere 5 km east of the C172, round 1,800 m; a cone ahead of the F-16C, its vertex 6 km east at its height, opening east
    // 10 degrees each side; a dome 300 m north of the UH-60A; a column 60 m north of the IRIS, 50 to 150 m up
    OpVolume sphere = at(a, VolumeShape::Sphere, 0.0, 5000.0, 1800.0);
    sphere.radiusM = 800.0;
    OpVolume cone = at(b, VolumeShape::Cone, 0.0, 6000.0, b.altitudeMslM);
    cone.halfAngleRad = 10.0 * kDeg, cone.rangeM = 10000.0, cone.yawRad = 90.0 * kDeg;
    OpVolume dome = at(c, VolumeShape::Dome, 300.0, 0.0, c.altitudeMslM - 100.0);
    dome.radiusM = 200.0;
    OpVolume column = at(d, VolumeShape::Cylinder, 60.0, 0.0, d.altitudeMslM - 50.0);
    column.radiusM = 15.0, column.lengthM = 100.0, column.pitchRad = 90.0 * kDeg; // (its x up)
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Volume);
    std::vector<std::pair<std::uint32_t, ActivityId>> all;
    const std::pair<std::uint32_t, const OpVolume*> cases[] = {{c172, &sphere}, {f16, &cone}, {uh60, &dome}, {iris, &column}};
    for (const auto& [v, volume] : cases) {
        const CommandResult r = w.submit(v, into, *volume);
        INFO(v << ": " << reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        all.emplace_back(v, r.activity);
    }
    const std::vector<Ended> ended = flyUntilEnded(w, all, 300.0);
    for (const Ended& e : ended) CHECK(e.state == ActivityState::Completed);
    // each in its volume as it completed: within a metre of it
    double n = 0.0, e = 0.0;
    offsetFrom(sphere.latitudeRad, sphere.longitudeRad, ended[0].s, n, e);
    CHECK(std::sqrt(n * n + e * e + (ended[0].s.altitudeMslM - 1800.0) * (ended[0].s.altitudeMslM - 1800.0)) <= 801.0);
    offsetFrom(cone.latitudeRad, cone.longitudeRad, ended[1].s, n, e); // (east of its vertex, within 10 degrees of its axis)
    const double off = std::hypot(n, ended[1].s.altitudeMslM - b.altitudeMslM);
    INFO("the F-16C " << e << " m past the cone's vertex, " << off << " m off its axis");
    CHECK((e > 0.0 && off <= e * std::tan(10.0 * kDeg) + 1.0));
    offsetFrom(dome.latitudeRad, dome.longitudeRad, ended[2].s, n, e);
    const double up = ended[2].s.altitudeMslM - dome.altitudeM;
    CHECK((up >= -1.0 && std::sqrt(n * n + e * e + up * up) <= 201.0));
    offsetFrom(column.latitudeRad, column.longitudeRad, ended[3].s, n, e);
    const double high = ended[3].s.altitudeMslM - column.altitudeM;
    CHECK((std::hypot(n, e) <= 16.0 && high >= 49.0 && high <= 101.0));
    CHECK(w.supportTable(c172)->find("fsim.guidance.must_fly")->support == Support::Supported);
}

TEST_CASE("must fly: a volume above climbed into; one from its window, a geocentric box, one moving, one round another vehicle",
          "[modes][must_fly]") {
    session::World w(options("must-fly-volume-more"));
    const auto above = wing(w, "c172x", 1500.0, 55.0), windowed = wing(w, "c172x", 1500.0, 55.0, 3), boxed = wing(w, "c172x", 1500.0, 55.0, 6);
    const auto moving = wing(w, "c172x", 1500.0, 55.0, 9), chaser = wing(w, "c172x", 1500.0, 55.0, 12);
    // the vehicle the last flies into a sphere round: 3 km ahead and 1 km to the side, flying north
    session::VehicleSpec lead;
    lead.name = "lead", lead.type = "jsbsim:c172x";
    lead.initial.latitudeDeg += 0.36 + 1000.0 / kR / kDeg, lead.initial.longitudeDeg += 3000.0 / (kR * std::cos(lead.initial.latitudeDeg * kDeg)) / kDeg;
    lead.initial.altitudeMslM = 1500.0, lead.initial.headingDeg = 0.0, lead.initial.airspeedTrueMs = 45.0;
    const auto led = w.createVehicle(lead);
    REQUIRE(led != 0);
    CHECK(w.submit(led, VelocityCommand{45.0, 0.0, 0.0, kHold}).accepted());
    const sim::VehicleState a = *w.vehicleState(above), b = *w.vehicleState(windowed), c = *w.vehicleState(boxed), m = *w.vehicleState(moving);
    OpVolume high = at(a, VolumeShape::Sphere, 0.0, 5000.0, 2500.0); // (its lowest 1,000 m above the aircraft)
    high.radiusM = 500.0;
    OpVolume egg = at(b, VolumeShape::Ellipsoid, 0.0, 6000.0, 1500.0); // (entered from the north: 350 to 10 degrees)
    egg.semiAxisAM = 1000.0, egg.semiAxisBM = 700.0, egg.semiAxisCM = 300.0;
    OpVolume box; // (4 to 6 km east, 1 km either side, 1,700 to 1,900 m)
    box.shape = static_cast<double>(VolumeShape::Geocentric);
    double lat = 0.0, lon = 0.0;
    placeFrom(c, -1000.0, 4000.0, box.latitudeMinRad, box.longitudeMinRad);
    placeFrom(c, 1000.0, 6000.0, box.latitudeMaxRad, box.longitudeMaxRad);
    box.altitudeMinM = 1700.0, box.altitudeMaxM = 1900.0;
    OpVolume drifting = at(m, VolumeShape::Sphere, -1500.0, 5000.0, 1500.0); // (moving north at 10 m/s)
    drifting.radiusM = 600.0, drifting.northMs = 10.0, drifting.eastMs = 0.0;
    FrameSpec follows;
    follows.origin = FrameOrigin::Vehicle, follows.vehicle = led;
    const FrameId frame = w.createFrame(follows);
    REQUIRE(frame != 0);
    OpVolume round; // (300 m round the vehicle, at its height)
    round.shape = static_cast<double>(VolumeShape::Sphere), round.frame = static_cast<double>(frame), round.xM = round.yM = 0.0, round.radiusM = 300.0;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Volume);
    MustFlyCommand fromNorth = into;
    fromNorth.ingressMinRad = -10.0 * kDeg, fromNorth.ingressMaxRad = 10.0 * kDeg;
    const CommandResult r1 = w.submit(above, into, high), r2 = w.submit(windowed, fromNorth, egg), r3 = w.submit(boxed, into, box);
    const CommandResult r4 = w.submit(moving, into, drifting), r5 = w.submit(chaser, into, round);
    INFO(reasonName(r1.reason) << " " << reasonName(r2.reason) << " " << reasonName(r3.reason) << " " << reasonName(r4.reason) << " "
                               << reasonName(r5.reason) << " at " << r5.index);
    REQUIRE((r1.accepted() && r2.accepted() && r3.accepted() && r4.accepted() && r5.accepted()));
    const double t0 = w.simTime();
    const std::vector<Ended> ended = flyUntilEnded(
        w, {{above, r1.activity}, {windowed, r2.activity}, {boxed, r3.activity}, {moving, r4.activity}, {chaser, r5.activity}}, 400.0);
    for (const Ended& e : ended) CHECK(e.state == ActivityState::Completed);
    CHECK((ended[0].s.altitudeMslM >= 1999.0 && ended[0].s.altitudeMslM <= 3001.0)); // (climbed into it)
    INFO("came from " << std::remainder(track(ended[1].s) + kPi, 2.0 * kPi) / kDeg);
    CHECK(std::abs(std::remainder(track(ended[1].s) + kPi, 2.0 * kPi)) <= 12.0 * kDeg); // (heading south: from the north)
    CHECK((ended[2].s.altitudeMslM >= 1699.0 && ended[2].s.altitudeMslM <= 1901.0));
    offsetFrom(box.latitudeMinRad, box.longitudeMinRad, ended[2].s, lat, lon);
    CHECK((lat >= -1.0 && lat <= 2001.0 && lon >= -1.0 && lon <= 2001.0)); // (in its bounds)
    double n = 0.0, e = 0.0;
    offsetFrom(drifting.latitudeRad, drifting.longitudeRad, ended[3].s, n, e); // (the sphere where it was then)
    CHECK(std::hypot(std::hypot(n - 10.0 * (ended[3].timeS - t0), e), ended[3].s.altitudeMslM - 1500.0) <= 601.0);
    const sim::VehicleState l = *w.vehicleState(led); // (within 300 m of the vehicle as it completed: a step on, the lead a little on)
    offsetFrom(l.latitudeRad, l.longitudeRad, ended[4].s, n, e);
    INFO("the chaser " << std::hypot(n, e) << " m from the lead, " << ended[4].s.altitudeMslM - l.altitudeMslM << " m above it");
    CHECK(std::hypot(std::hypot(n, e), ended[4].s.altitudeMslM - l.altitudeMslM) <= 310.0);
}

TEST_CASE("must fly: an operational volume kept by its id; refusals naming its fields (MFY-03, ENV-06)", "[modes][must_fly]") {
    session::World w(options("must-fly-op-volumes"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState s = *w.vehicleState(v);
    OpVolume ball = at(s, VolumeShape::Sphere, 0.0, 4000.0, 1500.0);
    ball.id = 51, ball.radiusM = 600.0;
    CHECK(w.setOpVolume(ball) == Reason::None);
    CHECK(w.opVolume(51)->revision == 1);
    CHECK(w.setOpVolume(ball) == Reason::None);
    CHECK(w.opVolume(51)->revision == 2);
    CHECK(w.opVolumes() == std::vector<OpVolumeId>{51});
    CHECK(w.opVolume(51)->radiusM == 600.0);
    MustFlyCommand byId;
    byId.location = static_cast<double>(MustFlyLocation::OpVolume), byId.target = 51.0;
    const CommandResult r = w.submit(v, byId);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    const std::vector<Ended> ended = flyUntilEnded(w, {{v, r.activity}}, 200.0);
    CHECK(ended[0].state == ActivityState::Completed);
    // one the world does not keep, or removed; and volumes A-GRA's schema would not take, named from its field 10
    byId.target = 52.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    CHECK(w.removeOpVolume(51));
    CHECK_FALSE(w.removeOpVolume(51));
    byId.target = 51.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Volume);
    auto refusal = [&](void (*spoil)(OpVolume&)) {
        OpVolume x = ball;
        x.id = 0;
        spoil(x);
        const CommandResult bad = w.submit(v, into, x);
        return bad.reason == Reason::InvalidParameter ? static_cast<int>(bad.index) : -100;
    };
    CHECK(refusal([](OpVolume& x) { x.shape = 9.0; }) == 10);
    CHECK(refusal([](OpVolume& x) { x.altitudeM = kHold; }) == 11);                     // (on the Earth without its altitude)
    CHECK(refusal([](OpVolume& x) { x.radiusM = -1.0; }) == 12);
    CHECK(refusal([](OpVolume& x) { x.lengthM = 100.0; }) == 12);                      // (a cylinder's, on a sphere)
    CHECK(refusal([](OpVolume& x) {
              x.shape = static_cast<double>(VolumeShape::Cone), x.radiusM = kHold, x.halfAngleRad = 100.0 * kDeg;
          }) == 12);                                                                    // (beyond a quarter turn)
    CHECK(refusal([](OpVolume& x) { x.yawRad = 1.0; }) == 13);                         // (a sphere has none)
    CHECK(refusal([](OpVolume& x) { x.latitudeMinRad = 0.1; }) == 14);                 // (a geocentric volume's, on a sphere)
    CHECK(refusal([](OpVolume& x) { x.frame = 99.0; }) == 15);
    CHECK(refusal([](OpVolume& x) { x.northMs = 5.0; }) == 16);                        // (a velocity one way)
    CHECK(w.submit(v, into).reason == Reason::InvalidParameter);                        // (a volume's, given none)
    // an altitude given outside it over its inner point, or in another reference
    MustFlyCommand at2500 = into;
    at2500.altitudeM = 2500.0;
    OpVolume none = ball;
    none.id = 0;
    CommandResult outside = w.submit(v, at2500, none);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 3));
    at2500.altitudeM = 1500.0, at2500.altitudeReference = static_cast<double>(AltitudeReference::AboveGround);
    outside = w.submit(v, at2500, none);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 4));
    CHECK(w.setOpVolume(none) == Reason::InvalidParameter);
    CHECK(w.supportTable(v)->find("fsim.geometry")->support == Support::Supported);
}

TEST_CASE("must fly: a volume's UPDATE flies to the new one; queued with its volume, it starts later", "[modes][must_fly]") {
    session::World w(options("must-fly-volume-update"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), later = wing(w, "c172x", 1500.0, 55.0, 3);
    const sim::VehicleState s = *w.vehicleState(v), t = *w.vehicleState(later);
    OpVolume near = at(s, VolumeShape::Sphere, 0.0, 8000.0, 1500.0);
    near.radiusM = 500.0;
    MustFlyCommand into;
    into.location = static_cast<double>(MustFlyLocation::Volume);
    const CommandResult r = w.submit(v, into, near);
    REQUIRE(r.accepted());
    fly(w, 10.0);
    OpVolume north = at(s, VolumeShape::Sphere, 3000.0, 8000.0, 1500.0); // (moved 3 km north)
    north.radiusM = 500.0;
    CHECK(w.update(r.activity, MustFlyCommand{}, north).accepted());
    OpVolume ahead = at(t, VolumeShape::Sphere, 0.0, 5000.0, 1500.0);
    ahead.radiusM = 500.0;
    CommandOptions wait;
    wait.window.startNotBefore = w.simTime() + 20.0;
    const CommandResult q = w.submit(later, into, ahead, wait);
    REQUIRE(q.accepted());
    CHECK((q.flags & kDeferred) != 0);
    const std::vector<Ended> ended = flyUntilEnded(w, {{v, r.activity}, {later, q.activity}}, 300.0);
    CHECK(ended[0].state == ActivityState::Completed);
    double n = 0.0, e = 0.0;
    offsetFrom(north.latitudeRad, north.longitudeRad, ended[0].s, n, e);
    CHECK(std::hypot(std::hypot(n, e), ended[0].s.altitudeMslM - 1500.0) <= 501.0);
    CHECK(ended[1].state == ActivityState::Completed);
    offsetFrom(ahead.latitudeRad, ahead.longitudeRad, ended[1].s, n, e);
    CHECK(std::hypot(std::hypot(n, e), ended[1].s.altitudeMslM - 1500.0) <= 501.0);
}
