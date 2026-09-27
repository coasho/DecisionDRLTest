// The Vehicle Interface's modes, flown (docs/vehicle-interface.md, section 4):
// HSA/CSA on a stock wing, a direct and a fly-by-wire design, a helicopter
// and a multirotor, in wind - and the semantics a mission autonomy relies on:
// partial commands, references, validation against the aircraft's performance.
#include "control/Atmosphere.h"
#include "fsim/GuidanceModes.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

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
        CHECK(d->parameters.size() == 6);
    }
}
