// Support capabilities (docs/control-architecture.md, 8.2): gear, flaps,
// wheel brakes, speedbrake and pitch trim, set directly beside the cascade;
// offered where the aircraft has them, refused by their placards, and able to
// take an axis from the command that set it until then.
#include "control/Catalog.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using namespace fsim;
using namespace fsim::control;

namespace {

session::World makeWorld(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    return session::World(o);
}

session::VehicleSpec spec(const char* name, bool onGround = false) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:c172x";
    s.initial.altitudeMslM = onGround ? 0.0 : 1500.0;
    s.initial.airspeedTrueMs = onGround ? 0.0 : 55.0;
    s.initial.onGround = onGround;
    return s;
}

bool offers(session::World& w, std::uint32_t v, const std::string& id) {
    const auto& all = w.capabilities(v);
    return std::any_of(all.begin(), all.end(), [&](const CapabilityDescriptor& d) { return d.id == id; });
}

const AttitudeCommand kLevel{0.0, 0.04, kHold, 0.785, kHold, 55.0};

} // namespace

TEST_CASE("support effectors are offered where the aircraft has them", "[support]") {
    auto w = makeWorld("support-offered");
    const auto plain = w.createVehicle(spec("plain"));
    // without an effectors section: what an actuator command has always set
    CHECK(offers(w, plain, "fsim.support.gear"));
    CHECK(offers(w, plain, "fsim.support.flaps"));
    CHECK(offers(w, plain, "fsim.support.wheel_brakes"));
    CHECK_FALSE(offers(w, plain, "fsim.support.speedbrake"));
    CHECK_FALSE(offers(w, plain, "fsim.support.pitch_trim"));
    CHECK(w.submit(plain, SpeedbrakeCommand{1.0}).reason == Reason::UnknownCapability);

    auto own = std::make_shared<VehicleProfile>();
    own->effectors.header = {1, Provenance::User};
    own->effectors.retractableGear = false; // a fixed-gear Cessna
    own->effectors.speedbrake = true;
    own->effectors.pitchTrim = true;
    auto s = spec("equipped");
    s.profile = own;
    const auto equipped = w.createVehicle(s);
    CHECK_FALSE(offers(w, equipped, "fsim.support.gear"));
    CHECK(offers(w, equipped, "fsim.support.speedbrake"));
    CHECK(offers(w, equipped, "fsim.support.pitch_trim"));
    const auto& caps = w.capabilities(equipped);
    const auto flaps = std::find_if(caps.begin(), caps.end(), [](const CapabilityDescriptor& d) { return d.id == "fsim.support.flaps"; });
    REQUIRE(flaps != caps.end());
    CHECK(flaps->kind == CapabilityKind::Support);
    CHECK(flaps->persistence == Persistence::Terminating);
    CHECK(flaps->axes == axisBit(Axis::Flaps));

    // the speedbrake and trim reach the flight model beside ControlInputs
    const auto sb = w.submit(equipped, SpeedbrakeCommand{1.0});
    const auto trim = w.submit(equipped, PitchTrimCommand{-0.2});
    REQUIRE(sb.accepted());
    REQUIRE(trim.accepted());
    w.step();
    CHECK(w.model(equipped)->property("fcs/speedbrake-cmd-norm").get() == 1.0);
    CHECK(w.model(equipped)->property("fcs/pitch-trim-cmd-norm").get() == -0.2);
    CHECK(w.activity(sb.activity)->state == ActivityState::Active);
}

TEST_CASE("flaps complete in position and stay there; cancelled, they go back up", "[support]") {
    auto w = makeWorld("support-flaps");
    const auto v = w.createVehicle(spec("flapper"));
    w.command(v, kLevel);
    const auto r = w.submit(v, FlapsCommand{0.5});
    REQUIRE(r.accepted());
    CHECK(w.activity(r.activity)->state == ActivityState::Pending);
    w.step();
    CHECK(w.activity(r.activity)->state == ActivityState::Active);
    CHECK(w.inputs(v)->flaps == 0.5); // the legacy attitude activity gave up its flaps
    for (int k = 0; k < 900 && w.activity(r.activity)->state == ActivityState::Active; ++k) w.step();
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    CHECK(w.activity(r.activity)->reason == Reason::GoalReached);
    w.step(10);
    CHECK(w.inputs(v)->flaps == 0.5); // held there, as its residual

    const auto again = w.submit(v, FlapsCommand{0.3});
    REQUIRE(again.accepted());
    CHECK(w.update(again.activity, FlapsCommand{0.2}).accepted());
    CHECK(w.update(again.activity, GearCommand{1.0}).reason == Reason::WrongCommandType);
    CHECK(w.update(again.activity, kLevel).reason == Reason::WrongCommandType);
    w.step();
    CHECK(w.inputs(v)->flaps == 0.2);
    REQUIRE(w.cancel(again.activity).status == CommandStatus::Canceled);
    w.step();
    CHECK(w.inputs(v)->flaps == 0.0); // the neutral default: flaps up
}

TEST_CASE("a support activity takes its axis from the command that set it, which carries on without it", "[support]") {
    auto w = makeWorld("support-shrink");
    const auto v = w.createVehicle(spec("shared"));
    REQUIRE(w.command(v, kLevel));
    const ActivityId legacy = w.activities(v)[0].id;
    REQUIRE(w.activity(legacy)->axes == kLegacyAxes);
    const auto flaps = w.submit(v, FlapsCommand{0.4});
    REQUIRE(flaps.accepted());
    const ActivityRecord* a = w.activity(legacy);
    CHECK(a->live());
    CHECK(a->axes == (kLegacyAxes & ~axisBit(Axis::Flaps)));
    w.step();
    CHECK((w.activity(legacy)->constraints & kActivityAxesReduced) != 0);
    // the per-step path keeps updating the attitude activity; the flaps stay the flaps activity's
    REQUIRE(w.command(v, AttitudeCommand{0.1, 0.04, kHold, 0.785, kHold, 55.0}));
    CHECK(w.activity(legacy)->live());
    w.step();
    CHECK(w.inputs(v)->flaps == 0.4);
    // a command at another level is a new activity with every axis the command can set: it takes the flaps back
    REQUIRE(w.command(v, VelocityCommand{55.0, 0.0, kHold, kHold}));
    CHECK(w.activity(flaps.activity)->state == ActivityState::Canceled);
    CHECK(w.activity(flaps.activity)->reason == Reason::Preempted);
    w.step();
    CHECK(w.inputs(v)->flaps == 0.0);
}

TEST_CASE("the placards: no gear up on the ground, no gear or flaps out above their speeds", "[support]") {
    auto w = makeWorld("support-placards");
    const auto parked = w.createVehicle(spec("parked", true));
    CHECK(w.submit(parked, GearCommand{0.0}).reason == Reason::Unavailable);
    CHECK(w.submit(parked, GearCommand{1.0}).accepted());
    CHECK(w.submit(parked, WheelBrakesCommand{1.0, 1.0}).accepted());
    w.step();
    CHECK(w.inputs(parked)->brakeLeft == 1.0);

    auto own = std::make_shared<VehicleProfile>();
    own->envelope.header = {1, Provenance::User};
    own->envelope.flaps.casMaxMs = 40.0; // VFE below the 55 m/s it flies at
    own->envelope.gearCasMaxMs = 45.0;
    auto s = spec("fast");
    s.profile = own;
    const auto fast = w.createVehicle(s);
    w.step();
    CHECK(w.submit(fast, FlapsCommand{0.5}).reason == Reason::Unavailable);
    CHECK(w.submit(fast, FlapsCommand{0.0}).accepted()); // retracting is always allowed
    CHECK(w.submit(fast, GearCommand{1.0}).reason == Reason::Unavailable);

    // authority works for support axes as for the rest
    CommandOptions override;
    override.source = Source::Override;
    const auto held = w.submit(fast, WheelBrakesCommand{0.0, 0.0}, override);
    REQUIRE(held.accepted());
    const auto refused = w.submit(fast, WheelBrakesCommand{0.5, 0.5});
    CHECK(refused.reason == Reason::AuthorityHeld);
    CHECK(refused.other == held.activity);
}

TEST_CASE("support commands are checked against their ranges", "[support]") {
    auto w = makeWorld("support-ranges");
    const auto v = w.createVehicle(spec("ranged"));
    const auto r = w.submit(v, FlapsCommand{1.7});
    REQUIRE(r.accepted());
    CHECK((r.flags & kClamped) != 0);
    CommandOptions strict;
    strict.range = RangePolicy::Reject;
    CHECK(w.submit(v, FlapsCommand{1.7}, strict).reason == Reason::OutOfRange);
    CHECK(w.submit(v, GearCommand{kHold}).reason == Reason::InvalidParameter);
}
