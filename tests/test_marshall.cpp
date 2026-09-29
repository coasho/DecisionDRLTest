// The altitude stacked marshall (docs/flight-autonomy.md, 4.46; ADR-29 FA-8c): A-GRA's ALTITUDE_STACKED_MARSHALL - each aircraft of
// a stack flies its pattern at an altitude of its own, the lowest of its stack clear by its separation of every other aircraft
// marshalling round the same point, which the world chooses at the NEW.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

/// A wing spawned at an altitude of its own: `index` spaces it 3 km north, as wing() does.
std::uint32_t wingAt(session::World& w, const std::string& type, double altitudeM, double tasMs, int index) {
    return wing(w, type, altitudeM, tasMs, index);
}

/// A rotorcraft hovering at 100 m, as rotor() spawns one, but a short hop from the others: `index` 60 m north of the last,
/// from 0.5 degrees north of the default place.
std::uint32_t rotorNear(session::World& w, const std::string& type, int index) {
    double pitchDeg = 0.0, rollDeg = 0.0;
    {
        session::VehicleSpec probe;
        probe.name = type + "-probe-" + std::to_string(index);
        probe.type = "jsbsim:" + type;
        probe.initial.latitudeDeg += 1.0;
        probe.initial.altitudeMslM = 100.0;
        const auto id = w.createVehicle(probe);
        REQUIRE(id != 0);
        const auto& hover = w.profile(id)->hover;
        if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
        if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
        REQUIRE(w.removeVehicle(id));
    }
    session::VehicleSpec s;
    s.name = type + "-near-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += 0.5 + 60.0 * index / 111195.0;
    s.initial.altitudeMslM = 100.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    s.initial.pitchDeg = pitchDeg;
    s.initial.rollDeg = rollDeg;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    VelocityCommand still;
    still.verticalSpeedMs = 0.0;
    still.northMs = still.eastMs = 0.0;
    REQUIRE(w.submit(id, still).accepted());
    return id;
}

/// A marshall round a point, its stack from `leastM`.
MarshallCommand stack(double latitudeRad, double longitudeRad, double leastM, double separationM = kHold, double radiusM = kHold) {
    MarshallCommand m;
    m.latitudeRad = latitudeRad, m.longitudeRad = longitudeRad, m.altitudeMinM = leastM, m.separationM = separationM, m.radiusM = radiusM;
    return m;
}

double slotOf(session::World& w, ActivityId activity) {
    Setpoint sp;
    REQUIRE(w.activitySetpoint(activity, sp));
    REQUIRE(sp.marshall.has_value());
    // flown as its pattern at its slot, its stack beside it (4.46)
    const auto* flown = std::get_if<PatternCommand>(&std::get<Command>(sp.command));
    REQUIRE(flown != nullptr);
    CHECK(flown->altitudeM == sp.marshall->altitudeM);
    return sp.marshall->altitudeM;
}

} // namespace

TEST_CASE("marshall: a stack of four holds its separation - per class (ASM-01)", "[modes][marshall]") {
    session::World w(options("marshall-stacks"));
    // four C172s and four F-16Cs orbiting a point each, spawned near the slots they will take; four UH-60As and four IRIS
    // hovering over a point each, 30 m and 10 m apart
    std::vector<std::uint32_t> c172, f16;
    for (int i = 0; i < 4; ++i) c172.push_back(wingAt(w, "c172x", 1500.0 + 304.8 * i, 55.0, i));
    for (int i = 0; i < 4; ++i) f16.push_back(wingAt(w, "f16c", 3000.0 + 304.8 * i, 160.0, 4 + i));
    std::vector<std::uint32_t> uh60, iris;
    for (int i = 0; i < 4; ++i) uh60.push_back(rotorNear(w, "uh60", i));
    for (int i = 0; i < 4; ++i) iris.push_back(rotorNear(w, "iris", 10 + i));
    fly(w, 15.0); // (settled, as rotor() leaves one)
    const sim::VehicleState a = *w.vehicleState(c172[0]), b = *w.vehicleState(f16[0]), c = *w.vehicleState(uh60[0]), d = *w.vehicleState(iris[0]);
    constexpr double kR = 6371008.8;
    const double latA = a.latitudeRad + 2000.0 / kR, lonA = a.longitudeRad + 3000.0 / (kR * std::cos(a.latitudeRad));
    const double latB = b.latitudeRad, lonB = b.longitudeRad + 12000.0 / (kR * std::cos(b.latitudeRad));
    const double latC = c.latitudeRad + 300.0 / kR, lonC = c.longitudeRad, latD = d.latitudeRad + 40.0 / kR, lonD = d.longitudeRad;
    struct Stack {
        std::vector<std::uint32_t> ids;
        MarshallCommand m;
        double separation;
        std::vector<ActivityId> acts;
    };
    MarshallCommand hover = stack(latC, lonC, c.altitudeMslM + 50.0, 30.0);
    hover.pattern = static_cast<double>(PatternKind::Hover);
    MarshallCommand small = stack(latD, lonD, d.altitudeMslM + 10.0, 10.0);
    small.pattern = static_cast<double>(PatternKind::Hover);
    std::vector<Stack> stacks = {{c172, stack(latA, lonA, 1500.0, kHold, 1500.0), 304.8, {}},
                                 {f16, stack(latB, lonB, 3000.0, kHold, 5000.0), 304.8, {}},
                                 {uh60, hover, 30.0, {}},
                                 {iris, small, 10.0, {}}};
    for (Stack& s : stacks)
        for (std::size_t i = 0; i < s.ids.size(); ++i) {
            const CommandResult r = w.submit(s.ids[i], s.m, PatternShape{});
            INFO(s.ids[i] << ": " << reasonName(r.reason) << " at " << r.index);
            REQUIRE(r.accepted());
            s.acts.push_back(r.activity);
            CHECK(slotOf(w, r.activity) == s.m.altitudeMinM + s.separation * static_cast<double>(i)); // (each the lowest left)
        }
    // flown a while: each at its slot, and every two its separation apart
    fly(w, 150.0);
    double worstOff[4] = {}, leastApart[4] = {1e9, 1e9, 1e9, 1e9};
    for (int k = 0; k < 60; ++k) {
        fly(w, 1.0);
        for (std::size_t j = 0; j < stacks.size(); ++j) {
            const Stack& s = stacks[j];
            std::vector<double> alt;
            for (std::size_t i = 0; i < s.ids.size(); ++i) {
                alt.push_back(w.vehicleState(s.ids[i])->altitudeMslM);
                worstOff[j] = std::max(worstOff[j], std::abs(alt.back() - (s.m.altitudeMinM + s.separation * static_cast<double>(i))));
                CHECK(w.activity(s.acts[i])->state == ActivityState::Active);
            }
            for (std::size_t p = 0; p < alt.size(); ++p)
                for (std::size_t q = p + 1; q < alt.size(); ++q) leastApart[j] = std::min(leastApart[j], std::abs(alt[p] - alt[q]));
        }
    }
    // (at worst the C172s 0.43 m off their slots, the F-16Cs 1.6 m, the hovering UH-60As and IRIS under a millimetre; every two
    // their separation apart, the C172s' least 304.05 m of 304.8)
    const double offAtMost[] = {1.0, 4.0, 0.5, 0.5};
    for (std::size_t j = 0; j < stacks.size(); ++j) {
        INFO("stack " << j << ": " << worstOff[j] << " m off its slots at worst, " << leastApart[j] << " m apart at least");
        CHECK(worstOff[j] <= offAtMost[j]);                  // (at its slot)
        CHECK(leastApart[j] >= 0.99 * stacks[j].separation); // (its separation held)
    }
    CHECK(w.supportTable(c172[0])->find("fsim.guidance.marshall")->support == Support::Supported);
    CHECK(w.supportTable(uh60[0])->find("fsim.guidance.marshall/hover")->support == Support::Supported);
    CHECK(w.supportTable(c172[0])->find("fsim.guidance.marshall/hover")->support ==
          w.supportTable(c172[0])->find("fsim.guidance.pattern/hover")->support); // (as its hover's: the stock c172x declares nothing)
}

TEST_CASE("marshall: slots - the lowest clear, one asked for, a full stack, a slot freed, another point apart, a stack moved",
          "[modes][marshall]") {
    session::World w(options("marshall-slots"));
    std::vector<std::uint32_t> v;
    for (int i = 0; i < 5; ++i) v.push_back(wingAt(w, "c172x", 1500.0, 55.0, i));
    const sim::VehicleState s = *w.vehicleState(v[0]);
    constexpr double kR = 6371008.8;
    const double lat = s.latitudeRad + 3000.0 / kR, lon = s.longitudeRad + 3000.0 / (kR * std::cos(s.latitudeRad));
    MarshallCommand m = stack(lat, lon, 1500.0, 300.0);
    m.altitudeMaxM = 2100.0; // (three slots: 1,500, 1,800 and 2,100 m)
    const CommandResult r0 = w.submit(v[0], m, PatternShape{});
    MarshallCommand asked = m;
    asked.altitudeM = 2100.0; // (the top, asked for)
    const CommandResult r1 = w.submit(v[1], asked, PatternShape{});
    const CommandResult r2 = w.submit(v[2], m, PatternShape{});
    REQUIRE((r0.accepted() && r1.accepted() && r2.accepted()));
    CHECK(slotOf(w, r0.activity) == 1500.0);
    CHECK(slotOf(w, r1.activity) == 2100.0);
    CHECK(slotOf(w, r2.activity) == 1800.0);
    // the stack full: refused, naming its slot; one asked for, taken
    const CommandResult full = w.submit(v[3], m, PatternShape{});
    CHECK(full.reason == Reason::StackFull);
    CHECK(full.index == 3);
    CHECK(w.submit(v[3], asked, PatternShape{}).reason == Reason::StackFull);
    // another point, 5 km away: a stack of its own
    MarshallCommand apart = m;
    apart.latitudeRad += 5000.0 / kR;
    const CommandResult r3 = w.submit(v[3], apart, PatternShape{});
    REQUIRE(r3.accepted());
    CHECK(slotOf(w, r3.activity) == 1500.0);
    // a slot freed: the middle canceled, the next NEW takes it
    REQUIRE(w.cancel(r2.activity).status == CommandStatus::Canceled);
    const CommandResult r4 = w.submit(v[4], m, PatternShape{});
    REQUIRE(r4.accepted());
    CHECK(slotOf(w, r4.activity) == 1800.0);
    // an UPDATE that moves the stack: its slot chosen afresh (the other point's stack, whose lowest the fourth holds); one
    // that leaves it (a speed): its slot kept
    MarshallCommand moved;
    moved.latitudeRad = apart.latitudeRad, moved.longitudeRad = apart.longitudeRad;
    const CommandResult u = w.update(r4.activity, moved);
    INFO(reasonName(u.reason) << " at " << u.index);
    REQUIRE(u.accepted());
    CHECK(slotOf(w, r4.activity) == 1800.0); // (1,500 m taken there)
    MarshallCommand faster;
    faster.speed = 60.0;
    REQUIRE(w.update(r0.activity, faster).accepted());
    CHECK(slotOf(w, r0.activity) == 1500.0);
    // its pattern through its own UPDATE alone: a pattern's, refused
    PatternCommand climb;
    climb.altitudeM = 2500.0;
    CHECK(w.update(r0.activity, Command(climb)).reason == Reason::WrongCommandType);
    CHECK(slotOf(w, r0.activity) == 1500.0);
    fly(w, 20.0);
    CHECK(w.activity(r0.activity)->state == ActivityState::Active);
}

TEST_CASE("marshall: refusals naming their fields; a wing's hover not supported; never a task; queued with its slot held", "[modes][marshall]") {
    session::World w(options("marshall-refusals"));
    const auto v = wingAt(w, "c172x", 1500.0, 55.0, 0), later = wingAt(w, "c172x", 1500.0, 55.0, 3);
    const sim::VehicleState s = *w.vehicleState(v);
    constexpr double kR = 6371008.8;
    const double lat = s.latitudeRad + 3000.0 / kR, lon = s.longitudeRad;
    auto refusal = [&](void (*spoil)(MarshallCommand&)) {
        MarshallCommand m = stack(lat, lon, 1500.0);
        spoil(m);
        const CommandResult r = w.submit(v, m, PatternShape{});
        return r.reason == Reason::InvalidParameter ? static_cast<int>(r.index) : -100 - static_cast<int>(r.reason);
    };
    CHECK(refusal([](MarshallCommand& m) { m.pattern = static_cast<double>(PatternKind::Hold); }) == 0); // (ATC's, not a marshall's)
    CHECK(refusal([](MarshallCommand& m) { m.pattern = static_cast<double>(PatternKind::Racetrack); }) == 18); // (its second circle)
    CHECK(refusal([](MarshallCommand& m) { m.altitudeMinM = kHold; }) == 10);
    CHECK(refusal([](MarshallCommand& m) { m.altitudeMaxM = 1000.0; }) == 11);
    CHECK(refusal([](MarshallCommand& m) { m.separationM = 0.0; }) == 12);
    CHECK(refusal([](MarshallCommand& m) { m.altitudeReference = 9.0; }) == 4);
    MarshallCommand hover = stack(lat, lon, 1500.0);
    hover.pattern = static_cast<double>(PatternKind::Hover);
    const auto declared = wingAt(w, "c172", 1500.0, 50.0, 6); // (a design that declares it hovers not)
    CHECK(w.supportTable(declared)->find("fsim.guidance.marshall/hover")->support == Support::NotSupported);
    CHECK(w.submit(declared, hover, PatternShape{}).reason == Reason::NotSupported); // (a wing: its support table says so)
    // a racetrack by two circles, as A-GRA gives it
    MarshallCommand track = stack(lat, lon, 1500.0);
    track.pattern = static_cast<double>(PatternKind::Racetrack);
    PatternShape circles;
    circles.latitude2Rad = lat + 4000.0 / kR, circles.longitude2Rad = lon;
    const CommandResult r = w.submit(v, track, circles);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    Setpoint sp;
    REQUIRE(w.activitySetpoint(r.activity, sp));
    CHECK(sp.shape.latitude2Rad == circles.latitude2Rad);
    // never a task: its slot is the stack's at its NEW
    TaskRepetition once;
    once.attempts = 1;
    BatchCommand item;
    const MarshallCommand kept = stack(lat, lon, 1500.0);
    item.marshall = &kept;
    CHECK(w.storeTask(v, 9, item, once) == Reason::InvalidParameter);
    // queued behind a start window: its slot held while it waits - the next NEW takes the one above
    const sim::VehicleState t = *w.vehicleState(later);
    MarshallCommand next = stack(t.latitudeRad + 3000.0 / kR, t.longitudeRad, 1500.0);
    CommandOptions wait;
    wait.window.startNotBefore = w.simTime() + 20.0;
    const CommandResult q = w.submit(later, next, PatternShape{}, wait);
    REQUIRE(q.accepted());
    CHECK((q.flags & kDeferred) != 0);
    CHECK(slotOf(w, q.activity) == 1500.0);
    const CommandResult taken = w.submit(v, next, PatternShape{});
    REQUIRE(taken.accepted());
    CHECK(slotOf(w, taken.activity) == 1804.8);
    fly(w, 30.0);
    CHECK(w.activity(q.activity)->state == ActivityState::Active);
}
