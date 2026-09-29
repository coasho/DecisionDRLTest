// A must fly (docs/flight-autonomy.md, 4.42; ADR-29 FA-8a): A-GRA's MUST_FLY - a point flown over at its altitude, another
// vehicle flown over as it moves, an operational point the world keeps by its id - approached from within a window of
// bearings where one is given, laid out as a route from where the aircraft is; completed as the location is passed.
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

/// A place `north` and `east` metres from where the vehicle is.
void placeFrom(const sim::VehicleState& s, double north, double east, double& lat, double& lon) {
    lat = s.latitudeRad + north / kR;
    lon = s.longitudeRad + east / (kR * std::cos(s.latitudeRad));
}

MustFlyCommand pointAt(const sim::VehicleState& s, double north, double east, double altitudeM) {
    MustFlyCommand c;
    c.location = static_cast<double>(MustFlyLocation::Point);
    placeFrom(s, north, east, c.latitudeRad, c.longitudeRad);
    c.altitudeM = altitudeM;
    return c;
}

/// How a must fly passed its location: the least distance over the ground to it (a vehicle's, where it moves), the
/// height above it (a vehicle's) or its altitude there, the track then, and how the activity ended.
struct Pass {
    double closestM = 1e12, altitudeM = 0.0, trackRad = 0.0, aboveM = 0.0;
    ActivityState end = ActivityState::Active;
    Reason reason = Reason::None;
};

/// A must fly followed as all fly at once: how it passed its location (a vehicle's where it flies over one).
struct Tracked {
    std::uint32_t v = 0;
    ActivityId activity = 0;
    double lat = 0.0, lon = 0.0;
    std::uint32_t entity = 0;
    Pass pass;
    bool done = false;
};

void flyAll(session::World& w, std::vector<Tracked>& all, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 10.0); ++i) {
        fly(w, 0.1);
        bool live = false;
        for (Tracked& k : all) {
            if (k.done) continue;
            const sim::VehicleState& s = *w.vehicleState(k.v);
            double tLat = k.lat, tLon = k.lon, tAlt = 0.0;
            if (k.entity) {
                const sim::VehicleState& e = *w.vehicleState(k.entity);
                tLat = e.latitudeRad, tLon = e.longitudeRad, tAlt = e.altitudeMslM;
            }
            const double north = (s.latitudeRad - tLat) * kR, east = (s.longitudeRad - tLon) * kR * std::cos(tLat);
            const double d = std::hypot(north, east);
            if (d < k.pass.closestM) k.pass.closestM = d, k.pass.altitudeM = s.altitudeMslM, k.pass.trackRad = track(s), k.pass.aboveM = s.altitudeMslM - tAlt;
            const ActivityRecord* r = w.activity(k.activity);
            if (!r->live()) k.done = true, k.pass.end = r->state, k.pass.reason = r->reason;
            live = live || !k.done;
        }
        if (!live) break;
    }
}

Pass flyOver(session::World& w, std::uint32_t v, ActivityId a, double lat, double lon, double seconds, std::uint32_t entity = 0) {
    std::vector<Tracked> one(1);
    one[0].v = v, one[0].activity = a, one[0].lat = lat, one[0].lon = lon, one[0].entity = entity;
    flyAll(w, one, seconds);
    return one[0].pass;
}

/// The bearing the aircraft came from (its track's reciprocal) within [min, max] clockwise, give or take `slackDeg`.
bool cameFromWithin(double trackRad, double minRad, double maxRad, double slackDeg) {
    const double from = std::remainder(trackRad + kPi, 2.0 * kPi);
    const double width = std::fmod(std::fmod(maxRad - minRad, 2.0 * kPi) + 2.0 * kPi, 2.0 * kPi);
    const double past = std::fmod(std::fmod(from - minRad, 2.0 * kPi) + 2.0 * kPi, 2.0 * kPi);
    return past <= width + slackDeg * kDeg || past >= 2.0 * kPi - slackDeg * kDeg;
}

} // namespace

TEST_CASE("must fly: a point flown over at its altitude, completed as it is passed - per class (MFY-01)", "[modes][must_fly]") {
    session::World w(options("must-fly-point"));
    const auto iris = rotor(w, "iris", 15.0, 6), uh60 = rotor(w, "uh60", 15.0, 9); // (settled first: the wings are let go at once)
    const auto c172 = wing(w, "c172x", 1500.0, 55.0), f16 = wing(w, "f16c", 3000.0, 160.0, 3);
    struct Case {
        std::uint32_t v;
        double north, east, up, seconds, closestM, heightM;
        const char* name;
    };
    // ahead and to the side - some 20 s of flight at a wing's speed, or a rotorcraft's cruise - 100 m up (a rotorcraft 20 m)
    const Case cases[] = {{c172, 1500.0, 4000.0, 100.0, 200.0, 40.0, 20.0, "c172x"},
                          {f16, 3000.0, 9000.0, 100.0, 200.0, 80.0, 30.0, "f16c"},
                          {iris, 80.0, 60.0, 20.0, 120.0, 5.0, 3.0, "iris"},
                          {uh60, 600.0, 450.0, 20.0, 120.0, 15.0, 5.0, "uh60"}};
    std::vector<ActivityId> ids;
    std::vector<MustFlyCommand> commands;
    for (const Case& k : cases) {
        const sim::VehicleState& s = *w.vehicleState(k.v);
        const MustFlyCommand c = pointAt(s, k.north, k.east, s.altitudeMslM + k.up);
        const CommandResult r = w.submit(k.v, c);
        INFO(k.name << ": " << reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        ids.push_back(r.activity), commands.push_back(c);
    }
    std::vector<Tracked> all(std::size(cases));
    for (std::size_t i = 0; i < all.size(); ++i)
        all[i].v = cases[i].v, all[i].activity = ids[i], all[i].lat = commands[i].latitudeRad, all[i].lon = commands[i].longitudeRad;
    flyAll(w, all, 200.0);
    for (std::size_t i = 0; i < std::size(cases); ++i) {
        const Case& k = cases[i];
        const Pass& p = all[i].pass;
        INFO(k.name << ": passed " << p.closestM << " m off, " << p.altitudeM - commands[i].altitudeM << " m high");
        CHECK(p.end == ActivityState::Completed);
        CHECK(p.closestM < k.closestM);
        CHECK(std::abs(p.altitudeM - commands[i].altitudeM) < k.heightM);
    }
    CHECK(w.supportTable(c172)->find("fsim.guidance.must_fly")->support == Support::Supported);
}

TEST_CASE("must fly: from within its window of bearings - the nearer edge, or behind it through a point abeam (MFY-07)", "[modes][must_fly]") {
    session::World w(options("must-fly-ingress"));
    const auto side = wing(w, "c172x", 1500.0, 55.0), behind = wing(w, "c172x", 1500.0, 55.0, 3), within = wing(w, "c172x", 1500.0, 55.0, 6);
    const auto jet = wing(w, "f16c", 3000.0, 160.0, 9);
    struct Case {
        std::uint32_t v;
        double east, minDeg, maxDeg;
        std::size_t points; // the route laid out: approaches, then the location
        const char* name;
    };
    // the point 4 km east of each (9 km the jet's), flying east: from the south (a turn onto the last leg of about 110
    // degrees); from the east, behind it (a point abeam splitting the turn); from the west, where it comes from (straight in)
    const Case cases[] = {{side, 4000.0, 170.0, 190.0, 2, "from the south"},
                          {behind, 4000.0, 80.0, 100.0, 3, "from the east"},
                          {within, 4000.0, 250.0, 290.0, 1, "from the west"},
                          {jet, 9000.0, 170.0, 190.0, 2, "a jet from the south"}};
    std::vector<ActivityId> ids;
    std::vector<MustFlyCommand> commands;
    for (const Case& k : cases) {
        const sim::VehicleState& s = *w.vehicleState(k.v);
        MustFlyCommand c = pointAt(s, 0.0, k.east, s.altitudeMslM);
        c.ingressMinRad = std::remainder(k.minDeg * kDeg, 2.0 * kPi), c.ingressMaxRad = std::remainder(k.maxDeg * kDeg, 2.0 * kPi);
        const CommandResult r = w.submit(k.v, c);
        INFO(k.name << ": " << reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        Setpoint sp;
        REQUIRE(w.activitySetpoint(r.activity, sp));
        CHECK((k.v == jet ? sp.waypoints.size() >= 2 : sp.waypoints.size() == k.points)); // (a jet's turn onto its last leg: split or not)
        CHECK(std::get<MustFlyCommand>(std::get<Command>(sp.command)).ingressMinRad == c.ingressMinRad);
        ids.push_back(r.activity), commands.push_back(c);
    }
    std::vector<Tracked> all(std::size(cases));
    for (std::size_t i = 0; i < all.size(); ++i)
        all[i].v = cases[i].v, all[i].activity = ids[i], all[i].lat = commands[i].latitudeRad, all[i].lon = commands[i].longitudeRad;
    flyAll(w, all, 400.0);
    for (std::size_t i = 0; i < std::size(cases); ++i) {
        const Case& k = cases[i];
        const Pass& p = all[i].pass;
        INFO(k.name << ": passed " << p.closestM << " m off");
        CHECK(p.end == ActivityState::Completed);
        CHECK(p.closestM < (k.v == jet ? 80.0 : 40.0));
        INFO("came from " << std::remainder(p.trackRad + kPi, 2.0 * kPi) / kDeg << " deg");
        CHECK(cameFromWithin(p.trackRad, commands[i].ingressMinRad, commands[i].ingressMaxRad, 2.0));
    }
}

TEST_CASE("must fly: another vehicle, flown over as it moves, above it; gone, it fails (MFY-02)", "[modes][must_fly]") {
    session::World w(options("must-fly-entity"));
    // a C172 flying east at 50 m/s, a jet 8 km behind it at 150 m/s; a second C172 that goes away
    const auto slow = wing(w, "c172x", 1500.0, 50.0), jet = wing(w, "f16c", 1500.0, 150.0, 3), gone = wing(w, "c172x", 1500.0, 50.0, 6);
    HsaCommand straight;
    straight.courseRad = 90.0 * kDeg, straight.speed = 50.0, straight.altitudeM = 1500.0;
    REQUIRE(w.submit(slow, straight).accepted());
    REQUIRE(w.submit(gone, straight).accepted());
    // the jet, 10 km north of it, flies over it: 500 ft above it, given no altitude
    MustFlyCommand over;
    over.location = static_cast<double>(MustFlyLocation::Entity), over.target = static_cast<double>(slow);
    const CommandResult r = w.submit(jet, over);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    const Pass p = flyOver(w, jet, r.activity, 0.0, 0.0, 300.0, slow);
    CHECK(p.end == ActivityState::Completed);
    CHECK(p.closestM < 150.0);
    CHECK(p.aboveM > 100.0); // (500 ft above it, given no altitude)
    CHECK(p.aboveM < 220.0);
    // one flown over by another whose vehicle goes: it fails target_lost
    MustFlyCommand after;
    after.location = static_cast<double>(MustFlyLocation::Entity), after.target = static_cast<double>(gone);
    const CommandResult lost = w.submit(slow, after);
    REQUIRE(lost.accepted());
    fly(w, 2.0);
    REQUIRE(w.removeVehicle(gone));
    fly(w, 1.0);
    CHECK(w.activity(lost.activity)->state == ActivityState::Failed);
    CHECK(w.activity(lost.activity)->reason == Reason::TargetLost);
    // itself, or a vehicle the world does not have: refused
    over.target = static_cast<double>(jet);
    CHECK(w.submit(jet, over).reason == Reason::InvalidParameter);
    over.target = 999.0;
    CHECK(w.submit(jet, over).reason == Reason::InvalidParameter);
}

TEST_CASE("must fly: an operational point the world keeps by its id, with its own window (MFY-03, ENV-06)", "[modes][must_fly]") {
    session::World w(options("must-fly-op-point"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState& s = *w.vehicleState(v);
    OpPoint p;
    p.id = 7;
    placeFrom(s, 0.0, 4000.0, p.latitudeRad, p.longitudeRad);
    p.altitudeM = 1600.0, p.ingressMinRad = 170.0 * kDeg, p.ingressMaxRad = -170.0 * kDeg;
    CHECK(w.setOpPoint(p) == Reason::None);
    CHECK(w.opPoint(7)->revision == 1);
    CHECK(w.setOpPoint(p) == Reason::None);
    CHECK(w.opPoint(7)->revision == 2);
    CHECK(w.opPoints() == std::vector<OpPointId>{7});
    // malformed: refused as its schema restricts it
    auto refused = [&](void (*spoil)(OpPoint&)) {
        OpPoint q = p;
        q.id = 8;
        spoil(q);
        return w.setOpPoint(q) == Reason::InvalidParameter;
    };
    CHECK(refused([](OpPoint& q) { q.id = 0; }));
    CHECK(refused([](OpPoint& q) { q.longitudeRad = kHold; }));                          // (half a place)
    CHECK(refused([](OpPoint& q) { q.frame = 1.0; }));                                   // (a place and a frame)
    CHECK(refused([](OpPoint& q) { q.latitudeRad = 2.0; }));                             // (off the Earth)
    CHECK(refused([](OpPoint& q) { q.frameXM = 5.0; }));                                 // (offsets without a frame)
    CHECK(refused([](OpPoint& q) { q.ingressMaxRad = kHold; }));                         // (a window one way)
    CHECK(refused([](OpPoint& q) { q.ingressMinRad = 4.0; }));                           // (beyond half a turn)
    CHECK(refused([](OpPoint& q) { q.altitudeM = kHold, q.altitudeReference = 0.0; })); // (a reference alone)
    CHECK(refused([](OpPoint& q) { q.latitudeRad = q.longitudeRad = kHold, q.frame = 99.0; })); // (a frame the world does not have)
    CHECK_FALSE(w.opPoint(8));
    // flown: its altitude, and its window (from the south)
    MustFlyCommand c;
    c.location = static_cast<double>(MustFlyLocation::OpPoint), c.target = 7.0;
    const CommandResult r = w.submit(v, c);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    CHECK(sp.waypoints.size() == 2);
    const Pass pass = flyOver(w, v, r.activity, p.latitudeRad, p.longitudeRad, 400.0);
    CHECK(pass.end == ActivityState::Completed);
    CHECK(pass.closestM < 40.0);
    CHECK(std::abs(pass.altitudeM - 1600.0) < 20.0);
    CHECK(cameFromWithin(pass.trackRad, p.ingressMinRad, p.ingressMaxRad, 2.0));
    // one the world does not keep; one removed
    c.target = 9.0;
    CHECK(w.submit(v, c).reason == Reason::UnknownGeometry);
    CHECK(w.removeOpPoint(7));
    CHECK_FALSE(w.removeOpPoint(7));
    c.target = 7.0;
    CHECK(w.submit(v, c).reason == Reason::UnknownGeometry);
    CHECK(w.supportTable(v)->find("fsim.geometry")->support == Support::Supported);
}

TEST_CASE("must fly: refused as malformed, naming the field; an UPDATE merged and flown afresh", "[modes][must_fly]") {
    session::World w(options("must-fly-refused"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState& s = *w.vehicleState(v);
    auto refusal = [&](void (*spoil)(MustFlyCommand&)) {
        MustFlyCommand c = pointAt(s, 0.0, 4000.0, 1500.0);
        spoil(c);
        const CommandResult r = w.submit(v, c);
        return r.reason == Reason::InvalidParameter ? static_cast<int>(r.index) : -100;
    };
    CHECK(refusal([](MustFlyCommand& c) { c.location = 9.0; }) == 0);
    CHECK(refusal([](MustFlyCommand& c) { c.longitudeRad = kHold; }) == 2);
    CHECK(refusal([](MustFlyCommand& c) { c.latitudeRad = 2.0; }) == 1);
    CHECK(refusal([](MustFlyCommand& c) { c.altitudeReference = 9.0; }) == 4);
    CHECK(refusal([](MustFlyCommand& c) { c.target = 3.0; }) == 5);           // (a point names nothing)
    CHECK(refusal([](MustFlyCommand& c) { c.ingressMinRad = 1.0; }) == 7);    // (a window one way)
    CHECK(refusal([](MustFlyCommand& c) { c.ingressMinRad = 4.0, c.ingressMaxRad = 1.0; }) == 6);
    CHECK(refusal([](MustFlyCommand& c) { c.speed = -1.0; }) == 8);
    CHECK(refusal([](MustFlyCommand& c) { c.location = 1.0; }) == 1);          // (an entity's place is its own)
    // an UPDATE: a point further on, flown afresh; its window given one way refused
    const CommandResult r = w.submit(v, pointAt(s, 0.0, 4000.0, 1500.0));
    REQUIRE(r.accepted());
    fly(w, 5.0);
    MustFlyCommand next;
    placeFrom(*w.vehicleState(v), 1000.0, 5000.0, next.latitudeRad, next.longitudeRad);
    CHECK(w.update(r.activity, next).accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    const MustFlyCommand merged = std::get<MustFlyCommand>(std::get<Command>(sp.command));
    CHECK(merged.latitudeRad == next.latitudeRad);
    CHECK(merged.altitudeM == 1500.0); // (kept)
    MustFlyCommand oneWay;
    oneWay.ingressMaxRad = 1.0;
    CHECK(w.update(r.activity, oneWay).reason == Reason::InvalidParameter);
    const Pass p = flyOver(w, v, r.activity, next.latitudeRad, next.longitudeRad, 300.0);
    CHECK(p.end == ActivityState::Completed);
    CHECK(p.closestM < 40.0);
}

