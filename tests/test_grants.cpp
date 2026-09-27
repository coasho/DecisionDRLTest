// The Vehicle Interface's authority and availability (docs/vehicle-interface.md,
// 6 and 7): grants over the priorities - a vehicle whose policy commands only
// what it holds a grant for, requests, releases, revocations and what is
// allowed, each checked against the rules as conformance checks the
// lifecycle - the platform's restrictions of a capability, the revision a
// consumer polls, and the performance, against the aircraft's profile and
// loops and after a change of its loops.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <optional>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

/// An hsa east at the aircraft's speed and height, from `source`.
CommandResult hsa(session::World& w, std::uint32_t v, Source source = Source::Policy) {
    const auto& s = *w.vehicleState(v);
    HsaCommand h;
    h.headingRad = 0.5 * kPi, h.speed = s.airspeedTrueMs, h.speedReference = 0.0, h.altitudeM = s.altitudeMslM;
    CommandOptions o;
    o.source = source;
    return w.submit(v, h, o);
}

/// How the activity ended.
void endedAs(const session::World& w, ActivityId id, Reason reason) {
    const ActivityRecord* r = w.activity(id);
    REQUIRE(r != nullptr);
    INFO(activityStateName(r->state) << ", " << reasonName(r->reason));
    CHECK(r->state == ActivityState::Canceled);
    CHECK(r->reason == reason);
    CHECK(r->by == 0);
    CHECK(r->endTime == w.simTime());
}

constexpr const char* kHsa = "fsim.guidance.hsa";

} // namespace

TEST_CASE("grants: under Granted a policy commands only what it holds a grant for; the platform's own sources never need one", "[modes]") {
    session::World w(options("grants-gate"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    CHECK(w.controlMode(v) == ControlMode::Open);
    REQUIRE(hsa(w, v).accepted()); // Open: as ever
    REQUIRE(w.setControlMode(v, ControlMode::Granted) == Reason::None);
    CHECK(w.controlMode(v) == ControlMode::Granted);
    // no grant: refused, and nothing recorded
    const std::size_t before = w.activities(v).size();
    CommandResult r = hsa(w, v);
    CHECK(r.reason == Reason::NotGranted);
    CHECK(w.activities(v).size() == before);
    const ControlStatus none = w.controlStatus(v, kHsa);
    CHECK((none.allowed && !none.granted));
    // granted: flown; another capability still refused
    CHECK(w.requestControl(v, kHsa) == Reason::None);
    CHECK(w.controlStatus(v, kHsa).granted);
    r = hsa(w, v);
    REQUIRE(r.accepted());
    const ActivityId flying = r.activity;
    w.step(stepsFor(w, 1.0));
    CHECK(w.activity(flying)->state == ActivityState::Active);
    CHECK(w.submit(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}).reason == Reason::NotGranted);
    // the legacy entry points are gated the same way
    CHECK_FALSE(w.command(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}));
    CHECK(w.commandResult(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}).reason == Reason::NotGranted);
    CHECK(w.activity(flying)->state == ActivityState::Active); // (refused: nothing changed)
    CHECK(w.requestControl(v, "fsim.flight.attitude") == Reason::None);
    CHECK(w.command(v, AttitudeCommand{0.1, 0.05, kHold, 0.785, kHold, 55.0}));
    CHECK(w.activity(flying)->state == ActivityState::Canceled); // (preempted by it, as ever)
    // the platform's own sources: no grant, and they take the axes as before
    CommandOptions autopilot;
    autopilot.source = Source::Autopilot;
    const CommandResult ap = w.submit(v, VelocityCommand{55.0, 0.0, 1.5, kHold}, autopilot);
    CHECK(ap.accepted());
    // a grant opens a gate and nothing more: the autopilot still holds its axes
    CHECK(w.requestControl(v, "fsim.flight.velocity") == Reason::None);
    CHECK(w.submit(v, VelocityCommand{55.0, 0.0, 1.0, kHold}).reason == Reason::AuthorityHeld);
    // unknown capabilities and vehicles
    CHECK(w.requestControl(v, "fsim.guidance.nonsense") == Reason::UnknownCapability);
    CHECK(w.requestControl(999, kHsa) == Reason::UnknownVehicle);
    CHECK(w.setControlMode(999, ControlMode::Granted) == Reason::UnknownVehicle);
}

TEST_CASE("grants: release and revoke end the policy's activities; a capability no longer allowed is revoked; switching to Granted ends what flies without one",
          "[modes]") {
    session::World w(options("grants-lifecycle"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    // Open, then Granted: the policy's hsa has no grant, and ends
    ActivityId a = hsa(w, v).activity;
    REQUIRE(a != 0);
    w.step();
    REQUIRE(w.setControlMode(v, ControlMode::Granted) == Reason::None);
    endedAs(w, a, Reason::NotGranted);
    // released: the policy lets go, its activity ends, and the vehicle default flies
    REQUIRE(w.requestControl(v, kHsa) == Reason::None);
    a = hsa(w, v).activity;
    REQUIRE(a != 0);
    w.step();
    CHECK(w.releaseControl(v, kHsa) == Reason::None);
    endedAs(w, a, Reason::Released);
    CHECK_FALSE(w.controlStatus(v, kHsa).granted);
    CHECK(hsa(w, v).reason == Reason::NotGranted);
    w.step(stepsFor(w, 2.0)); // (the neutral default flies it: no activity)
    // revoked, as the platform says why
    for (const Reason why : {Reason::Revoked, Reason::CollisionAvoidance}) {
        REQUIRE(w.requestControl(v, kHsa) == Reason::None);
        a = hsa(w, v).activity;
        REQUIRE(a != 0);
        w.step();
        CHECK(w.revokeControl(v, kHsa, why) == Reason::None);
        endedAs(w, a, why);
        CHECK_FALSE(w.controlStatus(v, kHsa).granted);
    }
    // no longer allowed: the grant revoked, and requests refused until it is allowed again
    REQUIRE(w.requestControl(v, kHsa) == Reason::None);
    a = hsa(w, v).activity;
    REQUIRE(a != 0);
    w.step();
    CHECK(w.setAllowed(v, kHsa, false) == Reason::None);
    endedAs(w, a, Reason::Revoked);
    CHECK_FALSE(w.controlStatus(v, kHsa).allowed);
    CHECK(w.requestControl(v, kHsa) == Reason::NotAllowed);
    CHECK(w.setAllowed(v, kHsa, true) == Reason::None);
    CHECK(w.requestControl(v, kHsa) == Reason::None);
    // what the platform flies is not the policy's to release, nor is it revoked with it
    const ActivityId o = hsa(w, v, Source::Override).activity;
    REQUIRE(o != 0);
    CHECK(w.releaseControl(v, kHsa) == Reason::None);
    CHECK(w.revokeControl(v, kHsa) == Reason::None);
    w.step();
    CHECK(w.activity(o)->state == ActivityState::Active);
    // Open again: no grant needed; release still lets go of what the policy flies
    REQUIRE(w.setControlMode(v, ControlMode::Open) == Reason::None);
    REQUIRE(w.cancel(o).status == CommandStatus::Canceled);
    a = hsa(w, v).activity;
    REQUIRE(a != 0);
    CHECK(w.releaseControl(v, kHsa) == Reason::None);
    endedAs(w, a, Reason::Released);
}

TEST_CASE("availability: the platform's restriction refuses a policy's NEW and its request with the reason; what flies goes on; the revision counts each change",
          "[modes]") {
    session::World w(options("grants-availability"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 1.0));
    std::uint32_t revision = w.controlRevision(v);
    auto changed = [&](bool expected) {
        const std::uint32_t now = w.controlRevision(v);
        CHECK((now != revision) == expected);
        revision = now;
    };
    // a live hsa goes on when its capability is restricted; a new one is refused
    const ActivityId a = hsa(w, v).activity;
    REQUIRE(a != 0);
    CHECK(w.setAvailability(v, kHsa, Availability::TemporarilyUnavailable, Reason::CollisionAvoidance) == Reason::None);
    changed(true);
    const CapabilityStatus st = w.capabilityStatus(v, kHsa);
    CHECK(st.availability == Availability::TemporarilyUnavailable);
    CHECK(st.reason == Reason::CollisionAvoidance);
    CHECK(hsa(w, v).reason == Reason::CollisionAvoidance);
    CHECK(w.requestControl(v, kHsa) == Reason::CollisionAvoidance);
    w.step(stepsFor(w, 1.0));
    CHECK(w.activity(a)->state == ActivityState::Active);
    HsaCommand climb;
    climb.altitudeM = 1600.0;
    CHECK(w.update(a, climb).accepted()); // (what flies takes its UPDATEs)
    // the platform's own sources fly what it restricts (its collision avoidance)
    const CommandResult over = hsa(w, v, Source::Override);
    CHECK(over.accepted());
    REQUIRE(w.cancel(over.activity).status == CommandStatus::Canceled); // (its axes back to the policy)
    // restricted without a reason: "restricted"; the same again is no change
    CHECK(w.setAvailability(v, "fsim.guidance.route", Availability::Disabled) == Reason::None);
    changed(true);
    CHECK(w.capabilityStatus(v, "fsim.guidance.route").reason == Reason::Restricted);
    CHECK(w.setAvailability(v, "fsim.guidance.route", Availability::Disabled) == Reason::None);
    changed(false);
    // lifted
    CHECK(w.setAvailability(v, kHsa, Availability::Available, Reason::None) == Reason::None);
    changed(true);
    CHECK(w.capabilityStatus(v, kHsa).availability == Availability::Available);
    CHECK(hsa(w, v).accepted());
    // every change to the grants, what is allowed and the mode is counted - and no more
    CHECK(w.setControlMode(v, ControlMode::Granted) == Reason::None);
    changed(true);
    CHECK(w.setControlMode(v, ControlMode::Granted) == Reason::None);
    changed(false);
    CHECK(w.requestControl(v, kHsa) == Reason::None);
    changed(true);
    CHECK(w.requestControl(v, kHsa) == Reason::None); // (held already)
    changed(false);
    CHECK(w.releaseControl(v, kHsa) == Reason::None);
    changed(true);
    CHECK(w.releaseControl(v, kHsa) == Reason::None); // (nothing held: nothing changed)
    changed(false);
    CHECK(w.setAllowed(v, kHsa, false) == Reason::None);
    changed(true);
    CHECK(w.setAllowed(v, kHsa, false) == Reason::None);
    changed(false);
    w.step(stepsFor(w, 1.0));
    changed(false); // (flying changes none of it)
}

TEST_CASE("performance: a vehicle's is its profile's and its loops', and follows a change of its loops", "[modes]") {
    session::World w(options("grants-performance"));
    const auto c172 = wing(w, "c172x", 1500.0, 55.0, 0);
    const auto iris = rotor(w, "iris", 0.0, 3);
    auto loop = [&](std::uint32_t v, Level level, const char* name) {
        const Controller* c = w.controls(v)->controller(level);
        const std::optional<double> p = c ? c->parameter(name) : std::nullopt;
        REQUIRE(p.has_value());
        return *p;
    };
    // a wing: its loops' bank, climb and roll rate, its plant's airspeed
    const Performance wing0 = *w.performance(c172);
    CHECK_FALSE(wing0.hovers);
    CHECK(wing0.maxBankRad == loop(c172, Level::Velocity, "max_bank"));
    CHECK(wing0.bankRateRadS == loop(c172, Level::Attitude, "roll.max_rate"));
    CHECK(wing0.altitudeGainPerS == loop(c172, Level::Position, "altitude.gain"));
    const double plant = w.profile(c172)->plant.tasMs;
    CHECK((wing0.cruiseTasMs == plant || (std::isnan(wing0.cruiseTasMs) && std::isnan(plant)))); // (NaN: the c172x's profile gives none)
    CHECK(wing0.maxClimbMs <= loop(c172, Level::Position, "max_vertical_speed"));
    // a rotorcraft: its velocity loop's tilt and bandwidth, its position loop's speed, deceleration and climb
    const Performance quad = *w.performance(iris);
    CHECK(quad.hovers);
    CHECK(quad.maxTiltRad == loop(iris, Level::Velocity, "max_tilt"));
    CHECK(std::abs(quad.maxAccelerationMs2 - 9.80665 * std::tan(quad.maxTiltRad)) < 1e-9);
    CHECK(quad.velocityBandwidthRadS == loop(iris, Level::Velocity, "horizontal.kp"));
    CHECK(quad.maxGroundSpeedMs == loop(iris, Level::Position, "max_speed"));
    CHECK(quad.cruiseTasMs == 0.5 * quad.maxGroundSpeedMs);
    CHECK(quad.maxDecelerationMs2 == loop(iris, Level::Position, "deceleration"));
    CHECK(quad.maxClimbMs == loop(iris, Level::Position, "max_vertical_speed"));
    // a change of the loops: computed afresh, a new revision, counted for a consumer
    const std::uint32_t control0 = w.controlRevision(c172);
    REQUIRE(w.controls(c172)->setParameter(Level::Velocity, "max_bank", 0.4));
    const Performance wing1 = *w.performance(c172);
    CHECK(wing1.maxBankRad == 0.4);
    CHECK(wing1.revision == wing0.revision + 1);
    CHECK(w.controlRevision(c172) == control0 + 1);
    // the same value again: nothing changed, no new revision
    REQUIRE(w.controls(c172)->setParameter(Level::Velocity, "max_bank", 0.4));
    CHECK(w.performance(c172)->revision == wing1.revision);
    CHECK(w.controlRevision(c172) == control0 + 1);
    // guidance plans with it: an orbit from the defaults at the new bank
    w.step();
    PatternCommand orbit;
    const CommandResult r = w.submit(c172, orbit);
    REQUIRE(r.accepted());
    w.step();
    const double v = w.activity(r.activity)->progress.speedMs;
    INFO("orbit at " << v << " m/s");
    CHECK(w.performance(c172)->turnRadiusM(v) > wing0.turnRadiusM(v)); // (less bank, a wider turn)
}
