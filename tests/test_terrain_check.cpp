// The terrain under a commanded path (docs/flight-autonomy.md, 4.19; A-GRA's VIOLATION_TERRAIN with its
// TerrainConstraint, VI 1.2.6.9's elevation request): a route, a pattern, a curve or an hsa that would go below the
// ground is refused terrain_conflict at its NEW or its UPDATE, with the place it would meet it and when; what clears
// it flies; the query answers the ground the physics has. The ground: a ridge 1,500 m high across the way east.
#include "mode_flights.h"

#include "fsim/Capability.h"
#include "fsim/GroundProvider.h"
#include "fsim/InitialConditions.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)
const sim::InitialConditions kStart;
const double kLat0 = kStart.latitudeDeg * kDeg, kLon0 = kStart.longitudeDeg * kDeg;

double code(AltitudeReference r) { return static_cast<double>(r); }
double eastOf(double lon) { return (lon - kLon0) * kR * std::cos(kLat0); }

/// A ridge 1,500 m high from 10 to 12 km east of the start, running north and south; flat at sea level elsewhere, and
/// no data at all more than 20 km west of it.
class Ridge final : public sim::GroundProvider {
public:
    double heightAboveEllipsoidM(double, double lon) const override {
        const double east = eastOf(lon);
        return east >= 10000.0 && east <= 12000.0 ? 1500.0 : 0.0;
    }
    std::optional<double> knownHeightAboveEllipsoidM(double lat, double lon) const override {
        if (eastOf(lon) < -20000.0) return std::nullopt;
        return heightAboveEllipsoidM(lat, lon);
    }
};

session::WorldOptions ridged(const char* name) {
    auto o = options(name);
    o.ground = std::make_shared<Ridge>();
    return o;
}

/// A wing `eastM` east of the start at `altitudeM`, flying `headingDeg` at 55 m/s.
std::uint32_t spawn(session::World& w, const std::string& name, double eastM, double altitudeM, double headingDeg = 90.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:c172x";
    s.initial.longitudeDeg += eastM / (kR * std::cos(kLat0)) / kDeg;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = headingDeg;
    s.initial.airspeedTrueMs = 55.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

/// A waypoint `eastM` east of the start.
Waypoint east(double eastM, double altitudeM, double reference = kHold) {
    Waypoint p;
    p.latitudeRad = kLat0;
    p.longitudeRad = kLon0 + eastM / (kR * std::cos(kLat0));
    p.altitudeM = altitudeM;
    p.altitudeReference = reference;
    return p;
}

/// A straight curve segment from `fromM` to `toM` east of its reference, level at it.
BezierSegment straightEast(double fromM, double toM) {
    BezierSegment b;
    for (int i = 0; i < 6; ++i) b.east[i] = fromM + (toM - fromM) * i / 5.0;
    return b;
}

/// What a live activity flies now: its setpoint as updated.
Setpoint flown(const session::World& w, ActivityId a) {
    Setpoint s;
    REQUIRE(w.activitySetpoint(a, s));
    return s;
}
template <typename C>
C flownAs(const session::World& w, ActivityId a) {
    return std::get<C>(std::get<Command>(flown(w, a).command));
}

/// The last answer's terrain point: refused terrain_conflict, where and when.
const CommandDetails::Terrain& hit(const session::World& w, std::uint32_t v) {
    const CommandDetails* d = w.commandDetails(v);
    REQUIRE(d != nullptr);
    REQUIRE(d->findingCount >= 1);
    CHECK(d->findings[0].reason == Reason::TerrainConflict);
    REQUIRE(d->terrain.hit == 1);
    return d->terrain;
}

} // namespace

TEST_CASE("terrain: a route into the ground is refused terrain_conflict, with the place, the time and the point it flies to",
          "[terrain_check]") {
    session::World w(ridged("terrain-route"));
    const auto v = spawn(w, "c172x", 0.0, 1000.0);
    // level at 1,000 m through the ridge: its second leg meets it at 10 km, 182 s on at 55 m/s
    const std::vector<Waypoint> low = {east(5000.0, 1000.0), east(20000.0, 1000.0)};
    CommandResult r = w.submit(v, RouteCommand{}, low);
    CHECK(r.reason == Reason::TerrainConflict);
    CHECK(r.index == 1);
    {
        const CommandDetails::Terrain& t = hit(w, v);
        CHECK(t.index == 1);
        CHECK(std::abs(t.latitudeRad - kLat0) < 10.0 / kR);
        CHECK((eastOf(t.longitudeRad) >= 10000.0 && eastOf(t.longitudeRad) < 10000.2)); // (its edge, to a tenth of a metre)
        CHECK(std::abs(t.altitudeMslM - 1000.0) < 1e-6);
        CHECK(t.groundM == 1500.0);
        CHECK(std::abs(t.timeS - eastOf(t.longitudeRad) / 55.0) < 0.5);
        CHECK(w.commandDetails(v)->suggestion == 0); // (no clamp mends it)
    }
    // climbing to 1,600 m along the leg: still under the ridge where it meets it
    r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(5000.0, 1000.0), east(20000.0, 1600.0)});
    CHECK(r.reason == Reason::TerrainConflict);
    CHECK(std::abs(hit(w, v).altitudeMslM - (1000.0 + 600.0 * (eastOf(hit(w, v).longitudeRad) - 5000.0) / 15000.0)) < 1.0);
    // validated: the same answer, nothing flies; clamped: still refused; unchecked: flown
    CommandOptions validate;
    validate.validateOnly = true;
    const std::size_t live = w.activities(v).size();
    CHECK(w.submit(v, RouteCommand{}, low, validate).reason == Reason::TerrainConflict);
    CHECK(w.activities(v).size() == live);
    CommandOptions clamp;
    clamp.range = RangePolicy::Clamp;
    CHECK(w.submit(v, RouteCommand{}, low, clamp).reason == Reason::TerrainConflict);
    CommandOptions unchecked;
    unchecked.range = RangePolicy::None;
    CHECK(w.submit(v, RouteCommand{}, low, unchecked).accepted());
    // over it at 1,600 m, or above the ground at 300 m: flown
    CHECK(w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(5000.0, 1600.0), east(20000.0, 1600.0)}).accepted());
    CHECK(w.commandDetails(v)->terrain.hit == 0);
    const double agl = code(AltitudeReference::AboveGround);
    CHECK(w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(5000.0, 300.0, agl), east(20000.0, 300.0, agl)}).accepted());
    // what it flies after its last point, short of the ridge: on along its course for a minute, into it; round its point, clear
    const std::vector<Waypoint> shortOf = {east(4000.0, 1000.0), east(9000.0, 1000.0)};
    r = w.submit(v, RouteCommand{}, shortOf);
    CHECK(r.reason == Reason::TerrainConflict);
    CHECK(hit(w, v).index == 1);
    CHECK(eastOf(hit(w, v).longitudeRad) < 10000.2);
    RouteCommand loiter;
    loiter.end = static_cast<double>(EndBehavior::Loiter);
    CHECK(w.submit(v, loiter, shortOf).accepted());
    // an UPDATE into it: refused, and the route flies on as it was
    const ActivityId flying = w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(5000.0, 1600.0), east(20000.0, 1600.0)}).activity;
    REQUIRE(flying != 0);
    CHECK(w.update(flying, RouteCommand{}, low).reason == Reason::TerrainConflict);
    REQUIRE(w.activity(flying) != nullptr);
    CHECK(w.activity(flying)->live());
    const Setpoint kept = flown(w, flying);
    REQUIRE(kept.waypoints.size() == 2);
    CHECK(kept.waypoints[1].altitudeM == 1600.0);
}

TEST_CASE("terrain: a route's loiter is walked where it flies - an orbit into the ridge refused naming its point, one clear of it flown",
          "[terrain_check]") {
    session::World w(ridged("terrain-loiter"));
    const auto v = spawn(w, "c172x", 0.0, 1000.0);
    // level at 1,000 m, 3 km east, then an orbit at 8 km east (docs/flight-autonomy.md, 4.31): of 3 km its circle reaches
    // 11 km east, into the ridge; of 800 m it keeps clear of it - the legs clear either way
    Waypoint at = east(8000.0, 1000.0);
    at.kind = static_cast<double>(EndPointKind::LoiterPoint);
    RouteLoiter wide;
    wide.point = 1, wide.pattern.radiusM = 3000.0, wide.pattern.durationS = 120.0;
    const std::vector<Waypoint> points = {east(3000.0, 1000.0), at};
    const CommandResult r = w.submit(v, RouteCommand{}, points, {}, std::vector<RouteLoiter>{wide});
    CHECK(r.reason == Reason::TerrainConflict);
    CHECK(r.index == 1);
    const CommandDetails::Terrain& t = hit(w, v);
    CHECK(t.index == 1);
    CHECK((eastOf(t.longitudeRad) >= 10000.0 && eastOf(t.longitudeRad) <= 11000.1)); // (on its circle, where it meets the ridge)
    CHECK(std::abs(t.altitudeMslM - 1000.0) < 1e-6);
    RouteLoiter tight = wide;
    tight.pattern.radiusM = 800.0;
    CHECK(w.submit(v, RouteCommand{}, points, {}, std::vector<RouteLoiter>{tight}).accepted());
}

TEST_CASE("terrain: a climb optimisation is walked as the lowest it could fly - a best rate climb over the ridge flown over it, an efficient "
          "one, which may climb late, refused at its point",
          "[terrain_check]") {
    session::World w(ridged("terrain-climb"));
    session::VehicleSpec spec;
    spec.name = "c172";
    spec.type = "jsbsim:c172"; // (the hangar's: it has performance tables)
    spec.initial.altitudeMslM = 1400.0;
    spec.initial.headingDeg = 90.0;
    spec.initial.airspeedTrueMs = 50.0;
    const auto v = w.createVehicle(spec);
    REQUIRE(v != 0);
    // 3 km east at 1,400 m, then on to 20 km east at 1,600 m, over the ridge 10 km east (docs/flight-autonomy.md, 4.32):
    // climbing at once, even at the least rate its tables give it clears the ridge; an efficient climb is walked from the
    // latest it could start - there, whichever it would choose, still at 1,400 m at the ridge
    Waypoint up = east(20000.0, 1600.0);
    up.climbOptimization = static_cast<double>(ClimbOptimization::ExtendedRange);
    const CommandResult r = w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(3000.0, 1400.0), up});
    CHECK(r.reason == Reason::TerrainConflict);
    CHECK(r.index == 1);
    {
        const CommandDetails::Terrain& t = hit(w, v);
        CHECK((eastOf(t.longitudeRad) >= 10000.0 && eastOf(t.longitudeRad) < 10000.2));
        CHECK(std::abs(t.altitudeMslM - 1400.0) < 1e-6);
    }
    up.climbOptimization = static_cast<double>(ClimbOptimization::BestRate);
    const CommandResult flying = w.submit(v, RouteCommand{}, std::vector<Waypoint>{east(3000.0, 1400.0), up});
    REQUIRE(flying.accepted());
    CHECK(w.commandDetails(v)->terrain.hit == 0);
    double over = kHold; // (its least height over the ridge)
    for (unsigned k = 0; k < stepsFor(w, 300.0) && eastOf(w.vehicleState(v)->longitudeRad) < 12500.0; ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        if (eastOf(s.longitudeRad) >= 10000.0 && eastOf(s.longitudeRad) <= 12000.0) over = isHold(over) ? s.altitudeMslM : std::min(over, s.altitudeMslM);
    }
    INFO("its least over the ridge: " << over << " m");
    REQUIRE(!isHold(over));
    CHECK(over > 1590.0);
}

TEST_CASE("terrain: a pattern, a curve and an hsa into the ground are refused; clear of it, they fly", "[terrain_check]") {
    session::World w(ridged("terrain-modes"));
    const auto v = spawn(w, "c172x", 0.0, 1000.0);
    // an orbit over the ridge, under its top: refused; over it, or above the ground, flown
    PatternCommand orbit;
    orbit.latitudeRad = kLat0, orbit.longitudeRad = kLon0 + 10500.0 / (kR * std::cos(kLat0));
    orbit.radiusM = 1000.0, orbit.altitudeM = 1000.0;
    CHECK(w.submit(v, orbit).reason == Reason::TerrainConflict);
    CHECK(hit(w, v).index == -1);
    CHECK(hit(w, v).groundM == 1500.0);
    orbit.altitudeM = 1600.0;
    const ActivityId circling = w.submit(v, orbit).activity;
    CHECK(circling != 0);
    // its UPDATE down into it: refused, flown on as it was
    PatternCommand lower;
    lower.altitudeM = 1000.0;
    CHECK(w.update(circling, Command(lower)).reason == Reason::TerrainConflict);
    CHECK(flownAs<PatternCommand>(w, circling).altitudeM == 1600.0);
    orbit.altitudeM = 300.0, orbit.altitudeReference = code(AltitudeReference::AboveGround);
    CHECK(w.submit(v, orbit).accepted());

    // a curve east through it at the aircraft's height: the second of its segments meets it
    CHECK(w.submit(v, CurveCommand{}, std::vector<BezierSegment>{straightEast(0.0, 8000.0), straightEast(8000.0, 20000.0)}).reason ==
          Reason::TerrainConflict);
    CHECK(hit(w, v).index == 1);
    CHECK(eastOf(hit(w, v).longitudeRad) >= 10000.0);
    CHECK(std::abs(hit(w, v).timeS - eastOf(hit(w, v).longitudeRad) / 55.0) < 1.0);
    CurveCommand high;
    high.altitudeM = 1600.0;
    CHECK(w.submit(v, high, std::vector<BezierSegment>{straightEast(0.0, 8000.0), straightEast(8000.0, 20000.0)}).accepted());

    // an hsa's line a minute ahead: 2 km short of the ridge, east into it at its height; west, or over it, clear
    const auto near = spawn(w, "near", 8000.0, 1000.0);
    HsaCommand toward;
    toward.headingRad = 0.5 * kPi;
    CHECK(w.submit(near, toward).reason == Reason::TerrainConflict);
    {
        const CommandDetails::Terrain& t = hit(w, near);
        CHECK(t.index == -1);
        CHECK((eastOf(t.longitudeRad) >= 10000.0 && eastOf(t.longitudeRad) < 10000.2));
        CHECK(std::abs(t.timeS - (eastOf(t.longitudeRad) - 8000.0) / 55.0) < 0.5);
    }
    HsaCommand away;
    away.headingRad = 1.5 * kPi;
    CHECK(w.submit(near, away).accepted());
    toward.altitudeM = 1600.0;
    const ActivityId climbing = w.submit(near, toward).activity;
    CHECK(climbing != 0);
    // its UPDATE down into it: refused, flown on as it was
    HsaCommand down;
    down.altitudeM = 1000.0;
    CHECK(w.update(climbing, Command(down)).reason == Reason::TerrainConflict);
    CHECK(flownAs<HsaCommand>(w, climbing).altitudeM == 1600.0);
    toward.altitudeM = 300.0, toward.altitudeReference = code(AltitudeReference::AboveGround);
    CHECK(w.submit(near, toward).accepted());
}

TEST_CASE("terrain: a rotorcraft's hsa below the ground is refused where it hovers, at once", "[terrain_check]") {
    session::World w(options("terrain-hover"));
    const auto v = rotor(w, "iris", 5.0);
    HsaCommand into;
    into.altitudeM = -10.0;
    CHECK(w.submit(v, into).reason == Reason::TerrainConflict);
    const CommandDetails::Terrain& t = hit(w, v);
    const auto& s = *w.vehicleState(v);
    CHECK(std::abs(t.latitudeRad - s.latitudeRad) < 5.0 / kR);
    CHECK(std::abs(t.timeS) < 0.5);
    CHECK(t.altitudeMslM == -10.0);
    CHECK(t.groundM == 0.0);
    into.altitudeM = 20.0;
    CHECK(w.submit(v, into).accepted());
}

TEST_CASE("terrain: the query answers the ground the physics has, and nothing where it has no data", "[terrain_check]") {
    session::World w(ridged("terrain-query"));
    const double onRidge = kLon0 + 11000.0 / (kR * std::cos(kLat0));
    CHECK(w.terrainHeightM(kLat0, onRidge) == std::optional<double>(1500.0));
    CHECK(w.terrainHeightM(kLat0, kLon0) == std::optional<double>(0.0));
    CHECK_FALSE(w.terrainHeightM(kLat0, kLon0 - 30000.0 / (kR * std::cos(kLat0))).has_value());
    // flat by default: sea level everywhere
    session::World flat(options("terrain-query-flat"));
    CHECK(flat.terrainHeightM(0.3, -1.2) == std::optional<double>(0.0));
}
