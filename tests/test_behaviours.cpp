// The platform's own behaviours, fixed (docs/flight-autonomy.md, section 6;
// stage FA-1): each works wherever it is offered, or says why it does not -
// a route point no turn can capture, a circle the aircraft can fly, a
// pursuit that keeps its range, an evasion that keeps off the ground, and a
// manoeuvre flown fast enough and completed only within the envelope.
#include "mode_flights.h"

#include "fsim/BuiltinControllers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

PositionCommand pointAt(double lat0, double lon0, double north, double east, double altitudeM, double captureM) {
    PositionCommand p;
    p.latitudeRad = lat0 + north / kEarthM;
    p.longitudeRad = lon0 + east / (kEarthM * std::cos(lat0));
    p.altitudeMslM = altitudeM;
    p.captureRadiusM = captureM;
    return p;
}

BehaviorCommand behavior(const char* id, std::uint32_t target = 0) {
    BehaviorCommand b;
    b.id = id;
    b.target = target;
    return b;
}

double horizontal(const sim::VehicleState& a, const sim::VehicleState& b) {
    double north, east;
    offset(a, b.latitudeRad, b.longitudeRad, north, east);
    return std::hypot(north, east);
}

/// A wing on its velocity loop: straight and level on its heading at its speed (a target to follow).
void cruise(session::World& w, std::uint32_t v) {
    const auto& s = *w.vehicleState(v);
    REQUIRE(w.submit(v, VelocityCommand{s.airspeedTrueMs, 0.0, s.eulerRad[2], kHold}).accepted());
}

} // namespace

TEST_CASE("waypoints: a point inside the turn at its arrival is refused, with the point and the turn rate", "[behaviours]") {
    session::World w(options("behaviours-waypoints-admit"));
    const auto v = wing(w, "b52h", 6000.0, 200.0); // east; a 30 deg bank turns it on a 7 km radius
    const auto& s = *w.vehicleState(v);
    auto route = [&](double captureM) {
        BehaviorCommand b = behavior("waypoints");
        b.points = {pointAt(s.latitudeRad, s.longitudeRad, 0.0, 20000.0, 6000.0, captureM),     // straight ahead
                    pointAt(s.latitudeRad, s.longitudeRad, 3000.0, 20000.0, 6000.0, captureM)}; // then 3 km to the left: inside its turn
        return b;
    };
    const CommandResult refused = w.submit(v, route(300.0));
    CHECK(refused.reason == Reason::InvalidWaypoint);
    CHECK(refused.index == 1);
    CHECK(refused.constraint == Constraint::MaxTurnRate);
    CHECK(w.submit(v, route(5000.0)).accepted()); // a capture wide enough reaches out of its turn
    BehaviorCommand behind = behavior("waypoints");
    behind.points = {pointAt(s.latitudeRad, s.longitudeRad, 0.0, -3000.0, 6000.0, 300.0)}; // behind it, outside both turns: it turns back
    CHECK(w.submit(v, behind).accepted());
}

TEST_CASE("waypoints: a point it circles without closing fails the activity, and it flies on straight", "[behaviours]") {
    session::World w(options("behaviours-waypoints-fail"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const auto& s = *w.vehicleState(v);
    BehaviorCommand circle = behavior("waypoints");
    circle.points = {pointAt(s.latitudeRad, s.longitudeRad, -150.0, 0.0, 1500.0, 20.0)}; // abeam, well inside its turn
    CHECK(w.submit(v, circle).reason == Reason::InvalidWaypoint);                       // refused, checked
    CommandOptions unchecked;
    unchecked.range = RangePolicy::None; // the existing entry points' path: nothing checked, so it flies it
    const CommandResult r = w.submit(v, circle, unchecked);
    REQUIRE(r.accepted());
    const unsigned n = stepsFor(w, 180.0);
    for (unsigned k = 0; k < n && w.activity(r.activity)->live(); ++k) w.step();
    const ActivityRecord& record = *w.activity(r.activity);
    CHECK(record.state == ActivityState::Failed);
    CHECK(record.reason == Reason::BehaviorFailed);
    CHECK(record.endTime < 120.0); // a turn and a half at 55 m/s
    const double heading = w.vehicleState(v)->eulerRad[2];
    w.step(stepsFor(w, 20.0));
    CHECK(std::abs(std::remainder(w.vehicleState(v)->eulerRad[2] - heading, 2.0 * kPi)) < 5.0 * kDeg); // straight on
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - 1500.0) < 60.0);
}

TEST_CASE("loiter: a wing given no radius circles as wide as its turn needs", "[behaviours]") {
    struct Case {
        const char* type;
        double altitudeM, tasMs, settleS, measureS;
    };
    for (const Case c : {Case{"c172x", 1500.0, 55.0, 150.0, 180.0}, Case{"b52h", 6000.0, 200.0, 200.0, 360.0}}) {
        INFO(c.type);
        session::World w(options("behaviours-loiter"));
        const auto v = wing(w, c.type, c.altitudeM, c.tasMs);
        const sim::VehicleState s0 = *w.vehicleState(v);
        const double expected = std::max(1500.0, 1.25 * LoiterBehavior::orbitRadiusM(*w.performance(v), s0.airspeedTrueMs)); // its bank and heading loop
        REQUIRE(w.submit(v, behavior("loiter")).accepted());
        w.step(stepsFor(w, c.settleS));
        double sum = 0.0;
        const unsigned n = stepsFor(w, c.measureS);
        for (unsigned k = 0; k < n; ++k) {
            w.step();
            double north, east;
            offset(*w.vehicleState(v), s0.latitudeRad, s0.longitudeRad, north, east);
            sum += std::hypot(north, east);
        }
        // the c172x's 1500 m; the B-52H's kilometres more. Its carrot law flies a slow-rolling aircraft's circle
        // up to 15 % inside (the B-52H's 0.3 rad/s bank rate); the pattern mode, its successor, flies it exact
        CHECK(std::abs(sum / n - expected) < 0.2 * expected);
    }
}

TEST_CASE("pursuit: the pursuer holds its range, closing no faster than it can stop, and never runs into its target", "[behaviours]") {
    session::World w(options("behaviours-pursuit"));
    const auto target = wing(w, "f16c", 3000.0, 160.0);
    session::VehicleSpec chaser;
    chaser.name = "chaser";
    chaser.type = "jsbsim:f16c";
    chaser.initial.longitudeDeg -= 3000.0 / (kEarthM * std::cos(chaser.initial.latitudeDeg * kDeg)) / kDeg; // 3 km behind it
    chaser.initial.altitudeMslM = 3000.0;
    chaser.initial.headingDeg = 90.0;
    chaser.initial.airspeedTrueMs = 175.0; // closing
    const auto v = w.createVehicle(chaser);
    REQUIRE(v != 0);
    cruise(w, target);
    BehaviorCommand pursue = behavior("pursuit", target);
    pursue.params = {{"range_m", 400.0}};
    REQUIRE(w.submit(v, pursue).accepted());
    double closest = std::numeric_limits<double>::infinity();
    const unsigned n = stepsFor(w, 150.0);
    for (unsigned k = 0; k < n; ++k) {
        w.step();
        closest = std::min(closest, horizontal(*w.vehicleState(v), *w.vehicleState(target)));
    }
    const double last = horizontal(*w.vehicleState(v), *w.vehicleState(target));
    CHECK(closest > 200.0); // before FA-1 it closed to within metres
    CHECK(std::abs(last - 400.0) < 150.0);
}

TEST_CASE("evade: never below its floor over the terrain; the floor holding its descent is reported", "[behaviours]") {
    struct Case {
        const char* type;
        double altitudeM, tasMs, expectM;
        bool floored;
    };
    // a wing high up descends its 300 m; one 400 m up stops at 150 m; a helicopter hovering at 100 m stays there
    for (const Case c : {Case{"c172x", 1500.0, 55.0, 1200.0, false}, Case{"c172x", 400.0, 55.0, 150.0, true}, Case{"uh1h", 100.0, 0.0, 100.0, true}}) {
        INFO(c.type << " at " << c.altitudeM << " m");
        session::World w(options("behaviours-evade"));
        const bool rotorcraft = std::string(c.type) == "uh1h";
        const auto v = rotorcraft ? rotor(w, c.type) : wing(w, c.type, c.altitudeM, c.tasMs);
        const auto threat = wing(w, "c172x", c.altitudeM + 200.0, 55.0, 1);
        cruise(w, threat);
        const CommandResult r = w.submit(v, behavior("evade", threat));
        REQUIRE(r.accepted());
        double lowest = std::numeric_limits<double>::infinity();
        const unsigned n = stepsFor(w, 120.0);
        for (unsigned k = 0; k < n; ++k) {
            w.step();
            lowest = std::min(lowest, w.vehicleState(v)->altitudeAglM);
        }
        CHECK(lowest > std::min(c.expectM, c.altitudeM) - 15.0);
        CHECK(std::abs(w.vehicleState(v)->altitudeMslM - c.expectM) < 30.0);
        CHECK(((w.activity(r.activity)->constraintsSeen & kActivityClamped) != 0) == c.floored);
    }
}

TEST_CASE("aerobatics: too slow, or a split-S too low, is refused; one flown out of the envelope fails, one flown within it completes",
          "[behaviours]") {
    session::World w(options("behaviours-aerobatics"));
    const auto slow = wing(w, "a10c", 3000.0, 100.0);      // its least airspeed is 56 m/s calibrated: a loop needs twice that
    const auto marginal = wing(w, "a10c", 3000.0, 150.0, 1); // enough to be let in, not to get over the top at 3.5 g
    const auto low = wing(w, "a10c", 900.0, 170.0, 2);
    const auto fighter = wing(w, "f16c", 3000.0, 165.0, 3);
    BehaviorCommand loop = behavior("aerobatics");
    loop.params = {{"manoeuvre", 1.0}};
    const CommandResult tooSlow = w.submit(slow, loop);
    CHECK(tooSlow.reason == Reason::PerformanceLimit);
    CHECK(tooSlow.constraint == Constraint::MinAirspeed);
    BehaviorCommand splitS = behavior("aerobatics");
    splitS.params = {{"manoeuvre", 3.0}};
    const CommandResult tooLow = w.submit(low, splitS);
    CHECK(tooLow.reason == Reason::PerformanceLimit);
    CHECK(tooLow.constraint == Constraint::MinAltitude);
    // flown: the A-10C runs out of airspeed over the top and gives up (before FA-1 it reported goal_reached
    // at 24 m/s); the F-16C's limiter holds it within the envelope, and it completes
    const CommandResult outOf = w.submit(marginal, loop);
    const CommandResult within = w.submit(fighter, loop);
    REQUIRE(outOf.accepted());
    REQUIRE(within.accepted());
    double slowest = std::numeric_limits<double>::infinity();
    const unsigned n = stepsFor(w, 90.0);
    for (unsigned k = 0; k < n && (w.activity(outOf.activity)->live() || w.activity(within.activity)->live()); ++k) {
        w.step();
        if (w.activity(outOf.activity)->live()) slowest = std::min(slowest, w.vehicleState(marginal)->airspeedCalibratedMs);
    }
    const ActivityRecord& failed = *w.activity(outOf.activity);
    CHECK(failed.state == ActivityState::Failed);
    CHECK(failed.reason == Reason::BehaviorFailed);
    CHECK(slowest > 50.0); // given up at its least airspeed, not flown on below it
    CHECK(w.activity(within.activity)->state == ActivityState::Completed);
    for (const auto v : {marginal, fighter}) CHECK(w.vehicleState(v)->altitudeAglM > 1500.0);
}
