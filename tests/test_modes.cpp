// The Vehicle Interface's modes, flown (docs/vehicle-interface.md, section 4):
// HSA/CSA on a stock wing, a direct and a fly-by-wire design, a helicopter
// and a multirotor, in wind - and the semantics a mission autonomy relies on:
// partial commands, references, validation against the aircraft's performance.
#include "control/Atmosphere.h"
#include "fsim/GuidanceModes.h"
#include "fsim/VehicleProfile.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

double code(SpeedReference r) { return static_cast<double>(r); }
double code(AltitudeReference r) { return static_cast<double>(r); }

HsaCommand heading(double headingRad, double speed, SpeedReference reference, double altitudeM = kHold) {
    HsaCommand h;
    h.headingRad = headingRad, h.speed = speed, h.speedReference = code(reference), h.altitudeM = altitudeM;
    return h;
}

HsaCommand course(double courseRad, double speed, SpeedReference reference, double altitudeM = kHold) {
    HsaCommand h;
    h.courseRad = courseRad, h.speed = speed, h.speedReference = code(reference), h.altitudeM = altitudeM;
    return h;
}

/// A speed in its reference, as the state has it.
double speedIn(SpeedReference r, const sim::VehicleState& s) { return speedNow(r, s); }

} // namespace

TEST_CASE("hsa: a wing holds a heading, a course, its altitude and its speed in each reference, across a crosswind", "[modes]") {
    struct Wing {
        const char* type;
        double altitudeM, tasMs;
    };
    // a 12 m/s crosswind from the east: a heading south drifts 13 deg off it at 55 m/s, a course does not
    for (const Wing a : {Wing{"c172x", 1500.0, 55.0}, Wing{"b52h", 3000.0, 180.0}, Wing{"f16c", 3000.0, 160.0}}) {
        INFO(a.type);
        session::World w(options("modes-hsa-wing"));
        setWind(w, 90.0, 12.0);
        struct Case {
            HsaCommand command;
            SpeedReference reference;
            bool isCourse;
        };
        const Case cases[] = {
            {heading(kPi, 0.95 * a.tasMs, SpeedReference::TrueAirspeed, a.altitudeM + 100.0), SpeedReference::TrueAirspeed, false},
            {course(kPi, 0.95 * a.tasMs, SpeedReference::TrueAirspeed, a.altitudeM + 100.0), SpeedReference::TrueAirspeed, true},
            {course(0.0, a.tasMs, SpeedReference::GroundSpeed), SpeedReference::GroundSpeed, true},
            {heading(0.5 * kPi, isa::calibratedFromTrue(a.tasMs, a.altitudeM), SpeedReference::CalibratedAirspeed), SpeedReference::CalibratedAirspeed, false},
        };
        std::vector<std::uint32_t> v;
        std::vector<ActivityId> act;
        for (int i = 0; i < 4; ++i) {
            v.push_back(wing(w, a.type, a.altitudeM, a.tasMs, i));
            const auto r = w.submit(v.back(), cases[i].command);
            REQUIRE(r.accepted());
            act.push_back(r.activity);
        }
        w.step(stepsFor(w, 150.0));
        for (int i = 0; i < 4; ++i) {
            INFO("case " << i);
            const auto& s = *w.vehicleState(v[static_cast<std::size_t>(i)]);
            const HsaCommand& c = cases[i].command;
            if (cases[i].isCourse) CHECK(std::abs(degreesApart(track(s), c.courseRad)) < 2.0);
            else CHECK(std::abs(degreesApart(s.eulerRad[2], c.headingRad)) < 2.0);
            CHECK(std::abs(speedIn(cases[i].reference, s) - c.speed) < 1.0);
            const double altitude = isHold(c.altitudeM) ? a.altitudeM : c.altitudeM;
            CHECK(std::abs(s.altitudeMslM - altitude) < 15.0);
            const ActivityRecord& record = *w.activity(act[static_cast<std::size_t>(i)]);
            CHECK(record.state == ActivityState::Active); // persistent
            CHECK(record.progress.speedMs == c.speed);    // what it commands
            CHECK(std::abs(record.progress.altitudeMslM - altitude) < 1.0); // (left out: the altitude it had at the NEW)
        }
    }
    // a Mach number, where the jet flies at it
    session::World w(options("modes-hsa-mach"));
    const auto f16 = wing(w, "f16c", 6000.0, 200.0);
    HsaCommand mach;
    mach.speed = 0.7, mach.speedReference = code(SpeedReference::Mach), mach.altitudeM = 6500.0;
    REQUIRE(w.submit(f16, mach).accepted());
    w.step(stepsFor(w, 120.0));
    CHECK(std::abs(w.vehicleState(f16)->mach - 0.7) < 0.01);
    CHECK(std::abs(w.vehicleState(f16)->altitudeMslM - 6500.0) < 15.0);
    CHECK(std::abs(degreesApart(w.vehicleState(f16)->eulerRad[2], 0.5 * kPi)) < 2.0); // no direction given: the heading it had
}

TEST_CASE("hsa: a wing settles on its heading, what its loops leave trimmed out - the stock c172x's", "[modes]") {
    // The stock c172x flies the shared loops: its bank loop has no integral and its propeller rolls it (0.07 aileron at
    // cruise power), so its heading law held it 1.6 deg right of every heading - short of one turned onto left, past one
    // turned onto right, off one held from the start - until the mode trimmed out what the loops leave. Four of them:
    // a left turn onto 12.86 deg, a right turn onto 167.14 deg, the heading they had (east) held, and a magnetic
    // heading flown to 12.86 deg true.
    session::World w(options("modes-hsa-heading"));
    const double headings[] = {12.86 * kDeg, 167.14 * kDeg, 0.5 * kPi, 12.86 * kDeg};
    std::vector<std::uint32_t> v;
    std::vector<ActivityId> act;
    std::vector<double> asked;
    for (int i = 0; i < 4; ++i) {
        v.push_back(wing(w, "c172x", 1500.0, 55.0, i));
        HsaCommand h;
        h.headingRad = headings[i];
        if (i == 3) {
            h.headingRad = std::remainder(headings[i] - w.stateData(v.back()).declinationRad, 2.0 * kPi);
            h.directionReference = static_cast<double>(DirectionReference::MagneticNorth);
        }
        const auto r = w.submit(v.back(), h);
        REQUIRE(r.accepted());
        act.push_back(r.activity);
        asked.push_back(h.headingRad);
    }
    // settled once the turn is flown and its trim taken up: from a minute on (the worst 0.20 deg, the magnetic heading's;
    // untrimmed, all four 1.6 deg)
    w.step(stepsFor(w, 60.0));
    double worst[4] = {};
    for (int k = 0; k < 180; ++k) {
        w.step(stepsFor(w, 1.0));
        for (int i = 0; i < 4; ++i) worst[i] = std::max(worst[i], std::abs(degreesApart(w.vehicleState(v[static_cast<std::size_t>(i)])->eulerRad[2], headings[i])));
    }
    for (int i = 0; i < 4; ++i) {
        INFO("case " << i << ": " << headings[i] / kDeg << " deg");
        CHECK(worst[i] < 0.3);
        const auto k = static_cast<std::size_t>(i); // its progress: what it commands, not its trim
        CHECK(std::abs(degreesApart(w.activity(act[k])->progress.headingRad, asked[k])) < 1e-9);
    }
    // a course between two headings: the second counts the turns it makes from its start, its trim kept (the 60 deg turned
    // on the course, counted as one update's, threw its trim to its most: 6.5 deg off within 30 s)
    HsaCommand onCourse;
    onCourse.courseRad = 30.0 * kDeg;
    REQUIRE(w.update(act[2], Command(onCourse)).accepted());
    w.step(stepsFor(w, 60.0));
    HsaCommand back;
    back.headingRad = w.vehicleState(v[2])->eulerRad[2];
    REQUIRE(w.update(act[2], Command(back)).accepted());
    double off = 0.0;
    for (int k = 0; k < 30; ++k) {
        w.step(stepsFor(w, 1.0));
        off = std::max(off, std::abs(degreesApart(w.vehicleState(v[2])->eulerRad[2], back.headingRad)));
    }
    CHECK(off < 0.3);
}

TEST_CASE("hsa: a rotorcraft hovers facing a heading in wind, and holds a course by ground speed or airspeed", "[modes]") {
    struct Rotor {
        const char* type;
        double speedMs, driftM, speedTolerance;
    };
    // a 5 m/s wind from the west
    for (const Rotor a : {Rotor{"iris", 5.0, 10.0, 0.3}, Rotor{"uh60", 20.0, 10.0, 1.0}, Rotor{"uh1h", 20.0, 25.0, 1.0}}) {
        INFO(a.type);
        session::World w(options("modes-hsa-rotor"));
        setWind(w, 270.0, 5.0);
        std::vector<std::uint32_t> v;
        for (int i = 0; i < 4; ++i) v.push_back(rotor(w, a.type, 0.0, i));
        w.step(stepsFor(w, 20.0));
        std::vector<sim::VehicleState> s0;
        for (auto id : v) s0.push_back(*w.vehicleState(id));
        HsaCommand still = heading(0.5 * kPi, 0.0, SpeedReference::GroundSpeed); // hover, facing east
        HsaCommand overGround = course(0.25 * kPi, a.speedMs, SpeedReference::GroundSpeed);
        HsaCommand throughAir = course(0.25 * kPi, a.speedMs, SpeedReference::TrueAirspeed);
        HsaCommand nose = heading(0.0, a.speedMs, SpeedReference::TrueAirspeed, 110.0);
        const HsaCommand commands[] = {still, overGround, throughAir, nose};
        for (int i = 0; i < 4; ++i) REQUIRE(w.submit(v[static_cast<std::size_t>(i)], commands[i]).accepted());
        w.step(stepsFor(w, 60.0));
        auto state = [&](int i) -> const sim::VehicleState& { return *w.vehicleState(v[static_cast<std::size_t>(i)]); };
        double north, east;
        offset(state(0), s0[0].latitudeRad, s0[0].longitudeRad, north, east);
        CHECK(std::hypot(north, east) < a.driftM);                          // a ground speed of none holds it (near) where it was
        CHECK(std::abs(degreesApart(state(0).eulerRad[2], 0.5 * kPi)) < 2.0); // facing east
        CHECK(std::abs(degreesApart(track(state(1)), 0.25 * kPi)) < 2.0);  // over the ground along the course
        CHECK(std::abs(groundSpeed(state(1)) - a.speedMs) < a.speedTolerance);
        CHECK(std::abs(degreesApart(state(1).eulerRad[2], 0.25 * kPi)) < 2.0); // the nose along the track
        CHECK(std::abs(degreesApart(track(state(2)), 0.25 * kPi)) < 2.0);  // an airspeed turned into the wind to hold the course
        CHECK(std::abs(state(2).airspeedTrueMs - a.speedMs) < a.speedTolerance);
        CHECK(std::abs(degreesApart(state(3).eulerRad[2], 0.0)) < 2.0);     // an airspeed along the nose drifts with the wind
        CHECK(std::abs(state(3).airspeedTrueMs - a.speedMs) < a.speedTolerance);
        CHECK(std::abs(state(3).altitudeMslM - 110.0) < 1.0);
    }
}

TEST_CASE("hsa: an UPDATE changes only what it gives, a NEW continues the hsa it replaces", "[modes]") {
    session::World w(options("modes-hsa-partial"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const auto first = w.submit(v, heading(kPi, 50.0, SpeedReference::TrueAirspeed, 1600.0));
    REQUIRE(first.accepted());
    w.step(stepsFor(w, 5.0));
    HsaCommand climb;
    climb.altitudeM = 1700.0; // only the altitude
    REQUIRE(w.update(first.activity, climb).accepted());
    w.step();
    const ActivityProgress& p = w.activity(first.activity)->progress;
    CHECK(std::abs(degreesApart(p.headingRad, kPi)) < 1e-6); // the heading and the speed as they were commanded
    CHECK(p.speedMs == 50.0);
    CHECK(p.altitudeMslM == 1700.0);
    w.step(stepsFor(w, 150.0));
    const auto& s = *w.vehicleState(v);
    CHECK(std::abs(degreesApart(s.eulerRad[2], kPi)) < 2.0);
    CHECK(std::abs(s.altitudeMslM - 1700.0) < 15.0);
    CHECK(std::abs(s.airspeedTrueMs - 50.0) < 1.0);
    // a NEW with only a speed continues the heading and altitude the live hsa commanded, and replaces it
    HsaCommand slower;
    slower.speed = 45.0;
    const auto second = w.submit(v, slower);
    REQUIRE(second.accepted());
    CHECK(w.activity(first.activity)->reason == Reason::Preempted);
    w.step();
    const ActivityProgress& q = w.activity(second.activity)->progress;
    CHECK(std::abs(degreesApart(q.headingRad, kPi)) < 1e-6);
    CHECK(q.altitudeMslM == 1700.0);
    CHECK(q.speedMs == 45.0);
    CHECK(q.speedReference == code(SpeedReference::TrueAirspeed));
    // a course replaces the heading; a reference without its value is refused in an UPDATE (it has no state to take one from)
    HsaCommand toCourse;
    toCourse.courseRad = 0.5 * kPi;
    REQUIRE(w.update(second.activity, toCourse).accepted());
    w.step();
    CHECK(std::isnan(w.activity(second.activity)->progress.headingRad) == false); // the heading it flies for the course
    CHECK(std::abs(w.activity(second.activity)->progress.courseRad - 0.5 * kPi) < 1e-9);
    HsaCommand referenceAlone;
    referenceAlone.speedReference = code(SpeedReference::Mach);
    const auto refused = w.update(second.activity, referenceAlone);
    CHECK(refused.reason == Reason::InvalidParameter);
    CHECK(refused.index == 2);
    // a NEW with a reference alone holds the aircraft's own value in it: the Mach it flies
    const double machNow = w.vehicleState(v)->mach;
    const auto holdMach = w.submit(v, referenceAlone);
    REQUIRE(holdMach.accepted());
    w.step();
    CHECK(std::abs(w.activity(holdMach.activity)->progress.speedMs - machNow) < 0.01);
}

TEST_CASE("hsa: a speed optimisation flies the tables' best speed at the altitude and weight now; a speed replaces it", "[modes]") {
    session::World w(options("modes-hsa-optimised"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const VehicleProfile& profile = *w.profile(f16);
    REQUIRE_FALSE(profile.tables.empty());
    auto setpoint = [&](ActivityId id) {
        Setpoint out;
        REQUIRE(w.activitySetpoint(id, out));
        return std::get<HsaCommand>(std::get<Command>(out.command));
    };
    auto best = [&](SpeedOptimization o, double altitudeM = kHold) { // (at the altitude now, else at `altitudeM`)
        const auto& s = *w.vehicleState(f16);
        return optimalTasMs(&profile.tables, static_cast<double>(o), isHold(altitudeM) ? s.altitudeMslM : altitudeM, s.fuelKg);
    };
    ActivityId flying = 0;
    for (const SpeedOptimization o : {SpeedOptimization::MaxEndurance, SpeedOptimization::LongRangeCruise}) {
        INFO(static_cast<int>(o));
        HsaCommand h;
        h.headingRad = 0.0, h.speedOptimization = static_cast<double>(o);
        const auto r = w.submit(f16, h);
        REQUIRE(r.accepted());
        flying = r.activity;
        // resolved as given: the optimum's true airspeed at the altitude it flies to (checked as a speed is)
        const HsaCommand given = setpoint(r.activity);
        CHECK(given.speedOptimization == static_cast<double>(o));
        CHECK(given.speedReference == code(SpeedReference::TrueAirspeed));
        CHECK(std::abs(given.speed - best(o, given.altitudeM)) < 1e-9);
        // flown: the optimum at the altitude and weight now, which the progress gives
        w.step(stepsFor(w, 90.0));
        const auto& s = *w.vehicleState(f16);
        const double now = best(o);
        CHECK(std::abs(w.activity(r.activity)->progress.speedMs - now) < 1e-4 * now); // (as its last update had it)
        CHECK(w.activity(r.activity)->progress.speedReference == code(SpeedReference::TrueAirspeed));
        CHECK(std::abs(s.airspeedTrueMs - now) < 0.02 * now); // (the velocity loop's: within 2 %)
    }
    // best endurance is the slower: the least fuel flow, below the most distance per kilogram
    CHECK(best(SpeedOptimization::MaxEndurance) < best(SpeedOptimization::LongRangeCruise));
    // a speed replaces it, and an optimisation a speed
    HsaCommand faster;
    faster.speed = 200.0;
    REQUIRE(w.update(flying, faster).accepted());
    CHECK((setpoint(flying).speed == 200.0 && std::isnan(setpoint(flying).speedOptimization)));
    HsaCommand endure;
    endure.speedOptimization = static_cast<double>(SpeedOptimization::MaxEndurance);
    REQUIRE(w.update(flying, endure).accepted());
    const HsaCommand again = setpoint(flying);
    CHECK((again.speedOptimization == endure.speedOptimization && std::abs(again.speed - best(SpeedOptimization::MaxEndurance, again.altitudeM)) < 1e-9));
    // not a code: refused, the field named
    HsaCommand odd;
    odd.speedOptimization = 0.5;
    const auto refused = w.submit(f16, odd);
    CHECK((refused.reason == Reason::InvalidParameter && refused.index == 6));
    // a stock aircraft has no tables to fly one from: not implemented, as its support table says
    const auto stock = wing(w, "c172x", 1500.0, 55.0, 1);
    const auto none = w.submit(stock, endure);
    CHECK((none.reason == Reason::NotImplemented && none.index == 6));
    CHECK(w.support(stock, "fsim.guidance.hsa/speed/max_endurance")->support == Support::NotImplemented);
    CHECK(w.support(f16, "fsim.guidance.hsa/speed/max_endurance")->support == Support::Supported);
}

TEST_CASE("hsa: above the ground over rising terrain", "[modes]") {
    const sim::InitialConditions start;
    auto o = options("modes-hsa-agl");
    o.ground = std::make_shared<RisingGround>(start.longitudeDeg * kDeg, start.latitudeDeg * kDeg, 0.02); // 2 % up to the east
    session::World w(o);
    const auto v = wing(w, "c172x", 500.0, 55.0);
    HsaCommand agl;
    agl.headingRad = 0.5 * kPi, agl.altitudeM = 400.0, agl.altitudeReference = code(AltitudeReference::AboveGround);
    REQUIRE(w.submit(v, agl).accepted());
    double worst = 0.0;
    const unsigned n = stepsFor(w, 180.0);
    for (unsigned k = 0; k < n; ++k) {
        w.step();
        if (k > stepsFor(w, 60.0)) worst = std::max(worst, std::abs(w.vehicleState(v)->altitudeAglM - 400.0));
    }
    const auto& s = *w.vehicleState(v);
    CHECK(worst < 20.0);
    CHECK(s.altitudeMslM - s.altitudeAglM > 150.0); // it did climb with the ground: 2 % over ~10 km
}

TEST_CASE("hsa: what the aircraft cannot do is refused, or clamped and said so", "[modes]") {
    session::World w(options("modes-hsa-checks"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    HsaCommand both;
    both.headingRad = 1.0, both.courseRad = 1.0;
    CommandResult r = w.submit(f16, both);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 1);
    HsaCommand oddReference;
    oddReference.speed = 150.0, oddReference.speedReference = 1.5;
    r = w.submit(f16, oddReference);
    CHECK(r.reason == Reason::InvalidParameter);
    CHECK(r.index == 3);
    // faster than it flies (its performance section's 446 m/s), higher than its ceiling
    HsaCommand fast;
    fast.speed = 600.0, fast.speedReference = code(SpeedReference::TrueAirspeed);
    r = w.submit(f16, fast, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.index == 2);
    CHECK(r.constraint == Constraint::MaxAirspeed);
    r = w.submit(f16, fast); // clamped instead
    REQUIRE(r.accepted());
    CHECK((r.flags & kClamped) != 0);
    CHECK(r.constraint == Constraint::MaxAirspeed);
    w.step();
    CHECK(w.activity(r.activity)->progress.speedMs < 450.0);
    HsaCommand high;
    high.altitudeM = 25000.0;
    r = w.submit(f16, high, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.index == 4);
    CHECK(r.constraint == Constraint::MaxAltitude);
    // slower than it can fly: the B-52H's calibrated 1.2 times its stall speed
    const auto b52 = wing(w, "b52h", 3000.0, 180.0, 5);
    HsaCommand slow;
    slow.speed = 50.0, slow.speedReference = code(SpeedReference::CalibratedAirspeed);
    r = w.submit(b52, slow, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.constraint == Constraint::MinAirspeed);
    // below the ground
    HsaCommand low;
    low.altitudeM = -10.0, low.altitudeReference = code(AltitudeReference::AboveGround);
    r = w.submit(f16, low, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.constraint == Constraint::MinAltitude);
    // an hsa is its setpoint, not a behaviour named "hsa"
    BehaviorCommand named;
    named.id = "hsa";
    CHECK(w.submit(f16, named).reason == Reason::WrongCommandType);
    // discovery: an A-GRA HSA_CSA capability that takes UPDATE, on every aircraft
    const auto quad = rotor(w, "iris", 0.0, 3);
    for (auto id : {f16, quad}) {
        const CapabilityDescriptor* d = nullptr;
        for (const auto& c : w.capabilities(id))
            if (c.id == "fsim.guidance.hsa") d = &c;
        REQUIRE(d != nullptr);
        CHECK(d->mode == FlightMode::HsaCsa);
        CHECK(d->setpoint == SetpointKind::Hsa);
        CHECK((d->interactions & kUpdate) != 0);
        CHECK(d->parameters.size() == 8); // (the speed optimisation: ADR-29 FA-3e; the direction reference last: FA-4d)
    }
}

TEST_CASE("a rotorcraft's airspeed is held to its tables' top level speed at the altitude and weight: clamped, or refused under Reject",
          "[modes]") {
    // (docs/flight-autonomy.md, 4.48: an hsa's, a pattern's, a route point's and the velocity level's, along the nose as
    // it flies them. Before, a multirotor's airspeed was held to nothing and a helicopter's to its never-exceed speed)
    session::World w(options("modes-rotor-top"));
    CommandOptions reject, unchecked;
    reject.range = RangePolicy::Reject;
    unchecked.range = RangePolicy::None;
    int index = 0;
    for (const char* type : {"cf2", "iris", "uh1h", "uh60"}) {
        INFO(type);
        const auto v = rotor(w, type, 5.0, index++);
        const VehicleProfile& profile = *w.profile(v);
        REQUIRE_FALSE(profile.tables.empty());
        const sim::VehicleState& s = *w.vehicleState(v);
        auto top = [&](double altitudeM) { return topTasMs(&profile.tables, altitudeM, s.fuelKg); };
        auto setpoint = [&](ActivityId id) {
            Setpoint out;
            REQUIRE(w.activitySetpoint(id, out));
            return out;
        };
        auto commanded = [&](ActivityId id) { return std::get<Command>(setpoint(id).command); };
        const double high = 1000.0, fast = 1.5 * top(high);
        REQUIRE(std::isfinite(fast));

        // an hsa at 1,000 m: refused, its field and the limit named; clamped, it flies the top there
        const HsaCommand hsa = heading(0.0, fast, SpeedReference::TrueAirspeed, high);
        CommandResult r = w.submit(v, hsa, reject);
        CHECK(r.reason == Reason::PerformanceLimit);
        CHECK((r.index == 2 && r.constraint == Constraint::MaxAirspeed));
        r = w.submit(v, hsa);
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) != 0);
        CHECK(r.constraint == Constraint::MaxAirspeed);
        CHECK(std::get<HsaCommand>(commanded(r.activity)).speed == top(high));
        // an UPDATE beyond it, in each airspeed reference; at the top, as asked
        HsaCommand next;
        next.speed = fast;
        CommandResult u = w.update(r.activity, next);
        CHECK((u.accepted() && (u.flags & kClamped) != 0));
        next.speedReference = code(SpeedReference::CalibratedAirspeed);
        u = w.update(r.activity, next);
        CHECK((u.accepted() && (u.flags & kClamped) != 0));
        CHECK(std::abs(std::get<HsaCommand>(commanded(r.activity)).speed - isa::calibratedFromTrue(top(high), high)) < 1e-9);
        next.speed = 0.5, next.speedReference = code(SpeedReference::Mach);
        u = w.update(r.activity, next);
        CHECK((u.accepted() && (u.flags & kClamped) != 0));
        CHECK(std::abs(std::get<HsaCommand>(commanded(r.activity)).speed - top(high) / isa::speedOfSound(high)) < 1e-12);
        next.speed = top(high), next.speedReference = code(SpeedReference::TrueAirspeed);
        u = w.update(r.activity, next);
        CHECK((u.accepted() && (u.flags & kClamped) == 0));
        w.cancel(r.activity);

        // a pattern at 1,000 m (its speed: field 9)
        PatternCommand pattern;
        pattern.speed = fast, pattern.speedReference = code(SpeedReference::TrueAirspeed), pattern.altitudeM = high;
        r = w.submit(v, pattern, reject);
        CHECK((r.reason == Reason::PerformanceLimit && r.index == 9 && r.constraint == Constraint::MaxAirspeed));
        r = w.submit(v, pattern);
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) != 0);
        CHECK(std::get<PatternCommand>(commanded(r.activity)).speed == top(high));
        w.cancel(r.activity);

        // a route's second point, at its altitude
        Waypoint first, second;
        first.latitudeRad = s.latitudeRad + 300.0 / kEarthM, first.longitudeRad = s.longitudeRad, first.altitudeM = s.altitudeMslM;
        second = first;
        second.latitudeRad = s.latitudeRad + 600.0 / kEarthM, second.speed = fast, second.speedReference = code(SpeedReference::TrueAirspeed);
        const Waypoint points[] = {first, second};
        r = w.submit(v, RouteCommand{}, Span<const Waypoint>(points, 2), reject);
        CHECK((r.reason == Reason::PerformanceLimit && r.index == 1 && r.constraint == Constraint::MaxAirspeed));
        r = w.submit(v, RouteCommand{}, Span<const Waypoint>(points, 2));
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) != 0);
        const Setpoint route = setpoint(r.activity);
        REQUIRE(route.waypoints.size() == 2);
        CHECK(route.waypoints[1].speed == top(second.altitudeM));
        w.cancel(r.activity);

        // the velocity level: its airspeed (field 0) at the altitude now; unchecked, as asked
        VelocityCommand velocity;
        velocity.airspeedMs = 1.5 * top(s.altitudeMslM), velocity.verticalSpeedMs = 0.0, velocity.headingRad = 0.0;
        r = w.submit(v, velocity, reject);
        CHECK((r.reason == Reason::PerformanceLimit && r.index == 0 && r.constraint == Constraint::MaxAirspeed));
        r = w.submit(v, velocity);
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) != 0);
        CHECK(std::get<VelocityCommand>(commanded(r.activity)).airspeedMs == top(s.altitudeMslM));
        r = w.submit(v, velocity, unchecked);
        REQUIRE(r.accepted());
        CHECK((r.flags & kClamped) == 0);
        CHECK(std::get<VelocityCommand>(commanded(r.activity)).airspeedMs == velocity.airspeedMs);
        velocity.airspeedMs = kHold, velocity.northMs = velocity.eastMs = 0.0; // (held still while the next one settles)
        REQUIRE(w.submit(v, velocity).accepted());
    }
}
