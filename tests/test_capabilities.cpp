// The contract layer (docs/control-architecture.md, sections 8-10): commands
// answered at once, activities and their states, authority between sources,
// range policies, the existing entry points, discovery.
#include "control/CapabilityHost.h"
#include "control/Catalog.h"
#include "control/Runtime.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/ControllerRegistry.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 3;
    return o;
}

session::VehicleSpec spec(const char* name, double latitudeOffsetDeg = 0.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:c172x";
    s.initial.latitudeDeg += latitudeOffsetDeg;
    s.initial.altitudeMslM = 1500.0;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = 55.0;
    return s;
}

const AttitudeCommand kBank{0.2, 0.04, kHold, 0.785, kHold, 55.0};
const VelocityCommand kClimb{55.0, 2.0, kHold, kHold};

std::string capabilityId(session::World& w, std::uint32_t vehicle, const ActivityRecord& r) {
    return w.capabilities(vehicle)[r.capability].id;
}

} // namespace

TEST_CASE("a command becomes an activity: pending, then active once the runtime has flown it", "[capabilities]") {
    session::World w(options("cap-lifecycle"));
    const auto v = w.createVehicle(spec("a"));
    const CommandResult r = w.submit(v, kBank);
    REQUIRE(r.accepted());
    REQUIRE(r.activity == activityId(v, 1));
    REQUIRE(activityVehicle(r.activity) == v);
    const ActivityRecord* a = w.activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == ActivityState::Pending);
    CHECK(a->vehicle == v);
    CHECK(a->source == Source::Policy);
    CHECK(a->axes == kPrimaryAxes);
    CHECK(capabilityId(w, v, *a) == "fsim.flight.attitude");
    CHECK(w.controls(v)->activeLevel() == Level::Attitude);
    w.step();
    CHECK(w.activity(r.activity)->state == ActivityState::Active);
    CHECK(std::isnan(w.activity(r.activity)->endTime));
}

TEST_CASE("UPDATE changes a live activity's setpoint; another command type or an ended activity is rejected", "[capabilities]") {
    session::World w(options("cap-update"));
    const auto v = w.createVehicle(spec("a"));
    const auto id = w.submit(v, kBank).activity;
    AttitudeCommand steeper = kBank;
    steeper.rollRad = -0.3;
    REQUIRE(w.update(id, steeper).accepted());
    CHECK(std::get<AttitudeCommand>(*w.controls(v)->activeCommand()).rollRad == -0.3);
    CHECK(w.update(id, kClimb).reason == Reason::WrongCommandType);
    CHECK(w.update(activityId(v, 99), steeper).reason == Reason::UnknownActivity);
    CHECK(w.update(activityId(12345, 1), steeper).reason == Reason::UnknownActivity);
    REQUIRE(w.cancel(id).status == CommandStatus::Canceled);
    CHECK(w.update(id, steeper).reason == Reason::ActivityEnded);
    CHECK(w.cancel(id).reason == Reason::ActivityEnded);
    // a behaviour's parameters are heap data: a new target is a NEW
    BehaviorCommand hold;
    hold.id = "hold";
    const auto b = w.submit(v, hold).activity;
    CHECK(w.update(b, hold).reason == Reason::NotUpdatable);
}

TEST_CASE("CANCEL ends an activity and the vehicle flies the neutral default", "[capabilities]") {
    session::World w(options("cap-cancel"));
    const auto v = w.createVehicle(spec("a"));
    const auto id = w.submit(v, kClimb).activity;
    w.step(5);
    const CommandResult r = w.cancel(id);
    REQUIRE(r.status == CommandStatus::Canceled);
    const ActivityRecord* a = w.activity(id);
    REQUIRE(a != nullptr);
    CHECK(a->state == ActivityState::Canceled);
    CHECK(a->reason == Reason::Requested);
    CHECK(a->endTime == w.simTime());
    CHECK(w.controls(v)->activeLevel() == Level::Actuator);
    w.step();
    const ControlInputs& in = *w.inputs(v);
    CHECK(in.aileron == 0.0);
    CHECK(in.elevator == 0.0);
    CHECK(in.throttle[0] == 0.0); // ActuatorCommand{}, as every vehicle starts
}

TEST_CASE("a newer command preempts its own or a lower priority and is rejected by a higher one", "[capabilities]") {
    session::World w(options("cap-authority"));
    const auto v = w.createVehicle(spec("a"));
    const auto policy = w.submit(v, kBank).activity;
    const auto policy2 = w.submit(v, kClimb).activity;
    REQUIRE(policy2 != 0);
    CHECK(w.activity(policy)->state == ActivityState::Canceled);
    CHECK(w.activity(policy)->reason == Reason::Preempted);
    CHECK(w.activity(policy)->by == policy2);

    CommandOptions autopilot;
    autopilot.source = Source::Autopilot;
    const auto hold = w.submit(v, kClimb, autopilot).activity;
    REQUIRE(hold != 0);
    CHECK(w.activity(policy2)->reason == Reason::Preempted);

    // a policy cannot take the axes an engaged autopilot mode holds, through either path
    const CommandResult refused = w.submit(v, kBank);
    CHECK(refused.status == CommandStatus::Rejected);
    CHECK(refused.reason == Reason::AuthorityHeld);
    CHECK(refused.other == hold);
    CHECK_FALSE(w.command(v, kBank));
    CHECK(w.activity(hold)->live());

    CommandOptions override;
    override.source = Source::Override;
    const auto operatorId = w.submit(v, kBank, override).activity;
    REQUIRE(operatorId != 0);
    CHECK(w.activity(hold)->state == ActivityState::Canceled);
    CHECK(w.activity(hold)->by == operatorId);
    CHECK(w.activity(operatorId)->source == Source::Override);
    // once the override lets go, the policy may command again
    w.cancel(operatorId);
    CHECK(w.command(v, kBank));
}

TEST_CASE("range policies: clamp, reject, and no checks for the existing entry points", "[capabilities]") {
    session::World w(options("cap-range"));
    const auto v = w.createVehicle(spec("a"));
    AttitudeCommand wild = kBank;
    wild.rollRad = 4.0; // beyond +-pi
    const CommandResult clamped = w.submit(v, wild);
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped) != 0);
    CHECK(std::get<AttitudeCommand>(*w.controls(v)->activeCommand()).rollRad <= 3.1416);
    w.step();
    CHECK((w.activity(clamped.activity)->constraints & kActivityClamped) != 0);

    CommandOptions strict;
    strict.range = RangePolicy::Reject;
    CHECK(w.submit(v, wild, strict).reason == Reason::OutOfRange);
    PositionCommand nowhere;
    nowhere.latitudeRad = kHold;
    CHECK(w.submit(v, nowhere).reason == Reason::InvalidParameter);

    // the existing entry points send what they always sent
    REQUIRE(w.command(v, wild));
    CHECK(std::get<AttitudeCommand>(*w.controls(v)->activeCommand()).rollRad == 4.0);
    REQUIRE(w.command(v, nowhere));
}

TEST_CASE("a behaviour that reaches its goal completes and keeps flying its hold until another command", "[capabilities]") {
    session::World w(options("cap-complete"));
    const auto v = w.createVehicle(spec("a"));
    BehaviorCommand route;
    route.id = "waypoints";
    const auto* s = w.vehicleState(v);
    route.points.push_back(PositionCommand{s->latitudeRad, s->longitudeRad, 1500.0, kHold, 5000.0}); // captured at once
    const auto id = w.submit(v, route).activity;
    REQUIRE(id != 0);
    w.step();
    const ActivityRecord* a = w.activity(id);
    REQUIRE(a != nullptr);
    CHECK(a->state == ActivityState::Completed);
    CHECK(a->reason == Reason::GoalReached);
    CHECK(w.controls(v)->activeLevel() == Level::Behavior); // its hold flies on
    CHECK(w.controls(v)->behaviorFinished());
    const auto next = w.submit(v, kClimb).activity;
    REQUIRE(next != 0);
    CHECK(w.activity(id)->state == ActivityState::Completed); // taken over, not preempted
    CHECK(w.activity(id)->by == 0);
    CHECK(w.controls(v)->activeLevel() == Level::Velocity);
}

TEST_CASE("a follower fails with TargetLost when its target goes, and flies on", "[capabilities]") {
    session::World w(options("cap-target"));
    const auto leader = w.createVehicle(spec("leader"));
    const auto follower = w.createVehicle(spec("follower", 0.01));
    BehaviorCommand pursue;
    pursue.id = "pursuit";
    pursue.target = leader;
    const auto id = w.submit(follower, pursue).activity;
    REQUIRE(id != 0);
    w.step(3);
    CHECK(w.activity(id)->state == ActivityState::Active);
    REQUIRE(w.removeVehicle(leader));
    w.step();
    CHECK(w.activity(id)->state == ActivityState::Failed);
    CHECK(w.activity(id)->reason == Reason::TargetLost);
    CHECK(w.controls(follower)->activeLevel() == Level::Behavior);
    // pursuit needs a target: without one, a NEW is refused (the existing path takes it, as before)
    pursue.target = 0;
    CHECK(w.submit(follower, pursue).reason == Reason::InvalidParameter);
    CHECK(w.command(follower, pursue));
}

TEST_CASE("a reset starts live activities again under the same id", "[capabilities]") {
    session::World w(options("cap-reset"));
    const auto v = w.createVehicle(spec("a"));
    BehaviorCommand hold;
    hold.id = "hold";
    const auto id = w.submit(v, hold).activity;
    w.step(3);
    REQUIRE(w.activity(id)->state == ActivityState::Active);
    REQUIRE(w.resetVehicle(v));
    CHECK(w.activity(id)->state == ActivityState::Pending);
    w.step();
    CHECK(w.activity(id)->state == ActivityState::Active);
    CHECK(w.controls(v)->activeLevel() == Level::Behavior);
}

TEST_CASE("the existing entry points: the same level updates, another level is a new activity, an unknown behaviour is refused", "[capabilities]") {
    session::World w(options("cap-legacy"));
    const auto v = w.createVehicle(spec("a"));
    REQUIRE(w.command(v, kBank));
    REQUIRE(w.command(v, AttitudeCommand{-0.1, 0.02, kHold, 0.785, kHold, 55.0}));
    auto live = [&] {
        std::vector<ActivityRecord> out;
        for (const auto& a : w.activities(v))
            if (a.live()) out.push_back(a);
        return out;
    };
    REQUIRE(live().size() == 1);
    const ActivityId first = live()[0].id;
    CHECK(live()[0].axes == kLegacyAxes);
    REQUIRE(w.command(v, kClimb));
    REQUIRE(live().size() == 1);
    CHECK(live()[0].id != first);
    CHECK(w.activity(first)->reason == Reason::Preempted);
    BehaviorCommand bogus;
    bogus.id = "no_such_behaviour";
    CHECK_FALSE(w.command(v, bogus)); // D1: it used to be ignored and reported as success
    CHECK(w.controls(v)->activeLevel() == Level::Velocity);
}

TEST_CASE("the existing path flies exactly as an explicit NEW and UPDATEs", "[capabilities]") {
    auto fly = [](bool explicitly) {
        session::World w(options(explicitly ? "cap-eq-new" : "cap-eq-legacy"));
        const auto v = w.createVehicle(spec("a"));
        ActivityId id = 0;
        for (int k = 0; k < 200; ++k) {
            const AttitudeCommand c{0.3 * std::sin(0.05 * k), 0.03, kHold, 0.785, 0.6, kHold};
            if (!explicitly) w.command(v, c);
            else if (!id) id = w.submit(v, c).activity;
            else w.update(id, c);
            w.step();
        }
        const auto& s = *w.vehicleState(v);
        return std::vector<double>{s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.eulerRad[0], s.eulerRad[1], s.airspeedTrueMs};
    };
    CHECK(fly(false) == fly(true));
}

TEST_CASE("activity ids are the vehicle's and count its commands; the same calls give the same ids", "[capabilities]") {
    auto ids = [] {
        session::World w(options("cap-ids"));
        const auto a = w.createVehicle(spec("a"));
        const auto b = w.createVehicle(spec("b", 0.01));
        return std::vector<ActivityId>{w.submit(a, kBank).activity, w.submit(b, kBank).activity, w.submit(a, kClimb).activity,
                                       w.submit(b, kClimb).activity};
    };
    const auto first = ids();
    CHECK(first == ids());
    CHECK(first[0] == activityId(1, 1));
    CHECK(first[1] == activityId(2, 1));
    CHECK(first[2] == activityId(1, 2));
}

TEST_CASE("a vehicle remembers its sixteen latest ended activities", "[capabilities]") {
    session::World w(options("cap-records"));
    const auto v = w.createVehicle(spec("a"));
    std::vector<ActivityId> ids;
    for (int i = 0; i < 20; ++i) ids.push_back(w.submit(v, i % 2 ? Command(kBank) : Command(kClimb)).activity);
    const auto all = w.activities(v);
    REQUIRE(all.size() == 1 + control::CapabilityHost::kRecent);
    CHECK(all[0].id == ids.back());
    CHECK(all[0].live());
    CHECK(all[1].id == ids[18]); // newest first
    CHECK(w.activity(ids[3]) != nullptr);
    CHECK(w.activity(ids[2]) == nullptr);
    CHECK(w.update(ids[2], kBank).reason == Reason::UnknownActivity);
}

TEST_CASE("discovery: the flight levels, the behaviours with their parameters, and behaviours registered later", "[capabilities]") {
    session::World w(options("cap-discover"));
    const auto v = w.createVehicle(spec("a"));
    const auto& caps = w.capabilities(v);
    auto has = [&](const std::string& id) {
        const auto& all = w.capabilities(v);
        return std::any_of(all.begin(), all.end(), [&](const CapabilityDescriptor& c) { return c.id == id; });
    };
    auto get = [&](const std::string& id) -> const CapabilityDescriptor& {
        static const CapabilityDescriptor none;
        for (const auto& c : w.capabilities(v))
            if (c.id == id) return c;
        FAIL_CHECK("no capability " << id);
        return none;
    };
    for (const char* id : {"fsim.flight.actuator", "fsim.flight.attitude", "fsim.flight.acceleration", "fsim.flight.velocity",
                           "fsim.flight.position", "fsim.guidance.hold", "fsim.guidance.waypoints", "fsim.guidance.loiter",
                           "fsim.guidance.pursuit", "fsim.guidance.evade", "fsim.guidance.formation", "fsim.guidance.aerobatics"})
        CHECK(has(id));
    CHECK(caps.size() >= 12);
    const auto& attitude = get("fsim.flight.attitude");
    REQUIRE(attitude.parameters.size() == 6); // the command struct's fields, in order
    CHECK(attitude.parameters[0].name == "roll_rad");
    CHECK((attitude.interactions & kUpdate) != 0);
    CHECK(get("fsim.guidance.waypoints").persistence == Persistence::Terminating);
    CHECK(get("fsim.guidance.pursuit").needsTarget);
    CHECK(get("fsim.guidance.hold").uses == std::vector<std::string>{"fsim.flight.velocity"});
    CHECK(w.capabilityStatus(v, "fsim.flight.attitude").availability == Availability::Available);
    CHECK(w.capabilityStatus(v, "no.such.capability").reason == Reason::UnknownCapability);
    CHECK(w.capabilityStatus(999, "fsim.flight.attitude").reason == Reason::UnknownVehicle);

    // a behaviour registered after the world was made is offered and commandable by either id
    struct Circle final : Behavior {
        const char* id() const noexcept override { return "test_circle"; }
        Command update(const ControlContext&, const Command&) override { return VelocityCommand{55.0, 0.0, kHold, 0.1}; }
    };
    BehaviorTraits traits;
    traits.uses = {"fsim.flight.velocity"};
    ControllerRegistry::instance().addBehavior("test_circle", [] { return std::make_unique<Circle>(); }, traits);
    REQUIRE(has("user.guidance.test_circle"));
    BehaviorCommand circle;
    circle.id = "test_circle";
    CHECK(w.submit(v, circle).accepted());
    circle.id = "user.guidance.test_circle";
    CHECK(w.submit(v, circle).accepted());
}

TEST_CASE("the host fails live activities when the flight model diverges", "[capabilities]") {
    ControlStack runtime;
    CapabilityCatalog catalog;
    VehicleProfile profile;
    CapabilityHost host;
    host.bind(7, runtime, catalog, adapterFor(ControlFamily::Stock), profile);
    sim::VehicleState s;
    const auto id = host.submit(kClimb, {}, s, 0.0).activity;
    REQUIRE(id == activityId(7, 1));
    s.diverged = true;
    CHECK(host.status(static_cast<std::size_t>(catalog.find("fsim.flight.velocity")), s).availability == Availability::TemporarilyUnavailable);
    CHECK(host.submit(kBank, {}, s, 0.0).reason == Reason::Unavailable);
    host.afterStep(s, {}, 1.0);
    CHECK(host.activity(id)->state == ActivityState::Failed);
    CHECK(host.activity(id)->reason == Reason::Diverged);
    CHECK(host.activity(id)->endTime == 1.0);
}
