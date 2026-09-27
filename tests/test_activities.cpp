// Activity commands (docs/flight-autonomy.md, 4.10; ADR-29 FA-2): a live
// activity disabled and kept, enabled again (a route at the point it flew
// to), reset, deleted for good, re-ranked, unassigned from its axes; one whose
// command said it takes none refuses them all.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

/// A waypoint `north` and `east` metres from where the vehicle is, at `altitudeM`.
Waypoint at(const sim::VehicleState& s, double north, double east, double altitudeM) {
    Waypoint p;
    p.latitudeRad = s.latitudeRad + north / kEarthM;
    p.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    p.altitudeM = altitudeM;
    return p;
}

/// Whether any of the vehicle's primary axes is owned (else the vehicle default flies them all).
bool owned(const session::World& w, std::uint32_t v) {
    const RuntimeConfig& c = w.controls(v)->config();
    for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
        if (c.owner[a] != RuntimeConfig::kNone) return true;
    return false;
}

CommandResult act(session::World& w, ActivityId a, ActivityCommand c, Rank rank = {}) { return w.activityCommand(Source::Policy, a, c, rank); }

} // namespace

TEST_CASE("activity commands: disable keeps an activity and frees its axes; enable flies it again, a route from the point it flew to",
          "[activity]") {
    session::World w(options("activity-disable"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const sim::VehicleState& s = *w.vehicleState(f16);
    const std::vector<Waypoint> route = {at(s, 0, 2500, 3000), at(s, 0, 9000, 3000), at(s, 0, 18000, 3000)};
    const CommandResult r = w.submit(f16, RouteCommand{}, route);
    REQUIRE(r.accepted());
    for (int i = 0; i < 40 && w.activity(r.activity)->progress.segment < 1; ++i) fly(w, 1.0);
    REQUIRE(w.activity(r.activity)->progress.segment == 1);

    const CommandResult off = act(w, r.activity, ActivityCommand::Disable);
    REQUIRE(off.accepted());
    const ActivityRecord* a = w.activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == ActivityState::Disabled);
    CHECK(a->live());
    CHECK(std::isnan(a->endTime));
    CHECK_FALSE(owned(w, f16)); // the vehicle default flies
    fly(w, 2.0);
    CHECK(w.activity(r.activity)->state == ActivityState::Disabled); // it stays, flying nothing
    CHECK(act(w, r.activity, ActivityCommand::Disable).accepted()); // (again: so it is)

    // enabled: it waits to start - nothing holds its axes, so it starts - from the point it flew to
    REQUIRE(act(w, r.activity, ActivityCommand::Enable).accepted());
    a = w.activity(r.activity);
    CHECK(a->state == ActivityState::Pending);
    CHECK(owned(w, f16));
    fly(w, 1.0);
    a = w.activity(r.activity);
    CHECK(a->state == ActivityState::Active);
    CHECK(a->progress.segment == 1);
    // reset: over from its beginning
    REQUIRE(act(w, r.activity, ActivityCommand::Reset).accepted());
    CHECK(w.activity(r.activity)->state == ActivityState::Pending);
    fly(w, 1.0);
    CHECK(w.activity(r.activity)->progress.segment == 0);
}

TEST_CASE("activity commands: delete ends an activity for good; unassign gives its axes to what waits; a new rank is arbitrated at once",
          "[activity]") {
    session::World w(options("activity-delete"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    HsaCommand h;
    h.headingRad = 1.0;
    const CommandResult hsa = w.submit(f16, h);
    REQUIRE(hsa.accepted());
    // a "nice" command of the same rank waits behind it
    CommandOptions nice;
    nice.interrupt = false;
    const CommandResult velocity = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, nice);
    REQUIRE((velocity.flags & kDeferred));

    // unassigned, the hsa waits behind it: the velocity command starts, and the older hsa does not take it back
    REQUIRE(act(w, hsa.activity, ActivityCommand::Unassign).accepted());
    CHECK(w.activity(velocity.activity)->waiting == ActivityWait::None);
    const ActivityRecord* waits = w.activity(hsa.activity);
    CHECK(waits->state == ActivityState::Pending);
    CHECK(waits->waiting == ActivityWait::Queued);
    CHECK(waits->waitingFor == velocity.activity);
    fly(w, 0.5);
    CHECK(w.activity(velocity.activity)->state == ActivityState::Active);

    // re-ranked ahead of what flies, it takes the axes back at once
    REQUIRE(act(w, velocity.activity, ActivityCommand::ChangeRank, Rank{3, 0}).accepted());
    CHECK((w.activity(velocity.activity)->rank == Rank{3, 0}));
    CHECK(w.activity(velocity.activity)->reason == Reason::Preempted);
    CHECK(w.activity(hsa.activity)->waiting == ActivityWait::None);

    // deleted: it ends, and nothing brings it back
    REQUIRE(act(w, hsa.activity, ActivityCommand::Delete).accepted());
    const ActivityRecord* gone = w.activity(hsa.activity);
    CHECK(gone->state == ActivityState::Deleted);
    CHECK(gone->reason == Reason::Requested);
    CHECK_FALSE(gone->live());
    CHECK(act(w, hsa.activity, ActivityCommand::Enable).reason == Reason::ActivityEnded);
    CHECK(act(w, activityId(f16, 999), ActivityCommand::Enable).reason == Reason::UnknownActivity);
}

TEST_CASE("activity commands: one whose command takes none refuses them all, and still takes UPDATE and CANCEL", "[activity]") {
    session::World w(options("activity-interactive"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions o;
    o.interactive = false;
    o.commandId = 12;
    const CommandResult r = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, o);
    REQUIRE(r.accepted());
    for (int c = 0; c < static_cast<int>(ActivityCommand::Count); ++c) {
        const CommandResult x = act(w, r.activity, static_cast<ActivityCommand>(c));
        INFO(activityCommandName(static_cast<ActivityCommand>(c)));
        CHECK(x.reason == Reason::NotInteractive);
        CHECK(x.commandId == 12);
    }
    CHECK(w.activity(r.activity)->state == ActivityState::Pending);
    CHECK(w.update(r.activity, Command(VelocityCommand{160.0, 0.0, 1.0})).accepted());
    CHECK(w.cancel(r.activity).status == CommandStatus::Canceled);
    // under Granted, a policy may not command what the platform flies
    REQUIRE(w.setControlMode(f16, ControlMode::Granted) == Reason::None);
    CommandOptions autopilot;
    autopilot.source = Source::Autopilot;
    const CommandResult ap = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, autopilot);
    REQUIRE(ap.accepted());
    const CommandResult held = act(w, ap.activity, ActivityCommand::Disable);
    CHECK(held.reason == Reason::AuthorityHeld);
    CHECK(held.other == ap.activity);
    CHECK(w.activityCommand(Source::Autopilot, ap.activity, ActivityCommand::Disable).accepted());
    CHECK(std::string_view(activityStateName(ActivityState::Disabled)) == "disabled");
    CHECK(std::string_view(activityCommandName(ActivityCommand::ChangeRank)) == "change_rank");
}
