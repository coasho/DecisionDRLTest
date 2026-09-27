// Ranks, queues and time windows (docs/flight-autonomy.md, 4.9; ADR-29 FA-2):
// a command takes contested axes from what it may interrupt and waits for the
// rest; a capability's precedence decides before a rank; a start window delays
// a start, and a time window it must meet and misses fails it; what waits is
// addressed as what flies, and starts when it may.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string_view>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

CommandOptions ranked(std::uint16_t priority, std::uint16_t precedence = 0, Source source = Source::Policy) {
    CommandOptions o;
    o.source = source;
    o.rank = {priority, precedence};
    return o;
}

HsaCommand heading(double rad) {
    HsaCommand h;
    h.headingRad = rad;
    return h;
}

/// The hsa the vehicle's runtime flies now, if one does.
const HsaCommand* flownHsa(const session::World& w, std::uint32_t v) {
    for (const SetpointSlot& s : w.controls(v)->config().slots)
        if (s.axes)
            if (const auto* h = std::get_if<HsaCommand>(&s.command)) return h;
    return nullptr;
}

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

} // namespace

TEST_CASE("schedule: a policy's command waits for what ranks ahead of it and takes from what it ranks at or ahead of", "[schedule]") {
    session::World w(options("schedule-rank"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const CommandResult a = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(1));
    REQUIRE(a.accepted());
    CHECK_FALSE(a.flags & kDeferred);

    // ranked behind it: accepted to wait, the answer naming what it waits for
    const CommandResult b = w.submit(f16, heading(1.0), ranked(5));
    REQUIRE(b.accepted());
    CHECK(b.newActivity);
    CHECK(b.flags & kDeferred);
    CHECK(b.other == a.activity);
    const ActivityRecord* waiting = w.activity(b.activity);
    REQUIRE(waiting != nullptr);
    CHECK(waiting->state == ActivityState::Pending);
    CHECK(waiting->waiting == ActivityWait::Queued);
    CHECK(waiting->waitingFor == a.activity);
    CHECK(waiting->basis() == ActivityBasis::Planned);
    CHECK((waiting->rank == Rank{5, 0}));
    fly(w, 1.0);
    CHECK(w.activity(b.activity)->state == ActivityState::Pending); // a step does not start it: what it waits for flies on
    CHECK(w.activity(a.activity)->state == ActivityState::Active);
    // listed with what flies, after it
    const auto listed = w.activities(f16);
    REQUIRE(listed.size() >= 2);
    CHECK(listed[0].id == a.activity);
    CHECK(listed[1].id == b.activity);

    // what it waited for ends: it starts at once, and flies from the next step
    REQUIRE(w.cancel(a.activity).status == CommandStatus::Canceled);
    const ActivityRecord* started = w.activity(b.activity);
    REQUIRE(started != nullptr);
    CHECK(started->state == ActivityState::Pending);
    CHECK(started->waiting == ActivityWait::None);
    CHECK(started->basis() == ActivityBasis::Actual);
    REQUIRE(flownHsa(w, f16) != nullptr);
    CHECK(std::abs(flownHsa(w, f16)->headingRad - 1.0) < 1e-9);
    fly(w, 0.5);
    CHECK(w.activity(b.activity)->state == ActivityState::Active);

    // ranked ahead of what flies: it takes the axes (at the same rank, the newest does - as ever)
    const CommandResult c = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(5));
    REQUIRE(c.accepted());
    CHECK_FALSE(c.flags & kDeferred);
    CHECK(w.activity(b.activity)->state == ActivityState::Canceled);
    CHECK(w.activity(b.activity)->by == c.activity);
    const CommandResult d = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(4, 7));
    REQUIRE(d.accepted());
    CHECK(w.activity(c.activity)->reason == Reason::Preempted);
}

TEST_CASE("schedule: a command that does not interrupt waits; the platform's interrupts any rank or, told not to, defers to it", "[schedule]") {
    session::World w(options("schedule-interrupt"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const CommandResult flying = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(9, 9));
    REQUIRE(flying.accepted());

    // a policy's "nice" command lets what flies finish, whatever its rank
    CommandOptions nice = ranked(0);
    nice.interrupt = false;
    const CommandResult waits = w.submit(f16, heading(1.0), nice);
    REQUIRE(waits.accepted());
    CHECK(waits.flags & kDeferred);
    CHECK(w.activity(waits.activity)->waitingFor == flying.activity);
    CHECK_FALSE(w.activity(waits.activity)->interrupt);

    // the platform's own interrupting command takes any rank
    CommandOptions autopilot = ranked(9, 9, Source::Autopilot);
    const CommandResult ap = w.submit(f16, heading(0.5), autopilot);
    REQUIRE(ap.accepted());
    CHECK_FALSE(ap.flags & kDeferred);
    CHECK(w.activity(flying.activity)->reason == Reason::Preempted);
    // (the nice one waits on: now for the autopilot, a higher source)
    CHECK(w.activity(waits.activity)->waitingFor == ap.activity);

    // the platform's deferring command: rank decides - behind it waits, ahead of it takes
    CommandOptions deferring = ranked(9, 10, Source::Autopilot);
    deferring.interrupt = false;
    const CommandResult behind = w.submit(f16, VelocityCommand{160.0, 0.0, 0.5}, deferring);
    REQUIRE(behind.accepted());
    CHECK(behind.flags & kDeferred);
    deferring.rank = {3, 0};
    const CommandResult ahead = w.submit(f16, VelocityCommand{160.0, 0.0, 0.5}, deferring);
    REQUIRE(ahead.accepted());
    CHECK_FALSE(ahead.flags & kDeferred);
    CHECK(w.activity(ap.activity)->by == ahead.activity);

    // a policy's interrupting command against the platform's activity: refused, as ever
    const CommandResult refused = w.submit(f16, heading(2.0), ranked(0));
    CHECK(refused.reason == Reason::AuthorityHeld);
    CHECK(refused.other == ahead.activity);
}

TEST_CASE("schedule: a capability's precedence decides before a rank, and only the platform overrides it", "[schedule]") {
    session::World w(options("schedule-precedence"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CHECK(w.capabilityPrecedence(f16, "fsim.flight.velocity") == 0);
    REQUIRE(w.setCapabilityPrecedence(f16, "fsim.flight.velocity", 2) == Reason::None);
    CHECK(w.capabilityPrecedence(f16, "fsim.flight.velocity") == 2);
    CHECK(w.setCapabilityPrecedence(f16, "fsim.guidance.no_such_thing", 1) == Reason::UnknownCapability);

    const CommandResult velocity = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(0));
    REQUIRE(velocity.accepted());
    CHECK(w.activity(velocity.activity)->precedence == 2);
    // the hsa's capability comes first (0 < 2): it takes the axes though it ranks last
    const CommandResult hsa = w.submit(f16, heading(1.0), ranked(9, 9));
    REQUIRE(hsa.accepted());
    CHECK_FALSE(hsa.flags & kDeferred);
    // and a velocity command, however it ranks, waits behind it...
    const CommandResult queued = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(0));
    REQUIRE(queued.accepted());
    CHECK(queued.flags & kDeferred);
    CHECK(w.activity(queued.activity)->waitingFor == hsa.activity);
    // ...until the platform puts the two capabilities level: then the rank decides, and it starts at once
    REQUIRE(w.setCapabilityPrecedence(f16, "fsim.flight.velocity", 0) == Reason::None);
    CHECK(w.activity(queued.activity)->waiting == ActivityWait::None);
    CHECK(w.activity(hsa.activity)->by == queued.activity);

    // a precedence override is the platform's own: a policy's is refused
    CommandOptions o = ranked(0);
    o.precedenceOverride = 0;
    CHECK(w.submit(f16, heading(1.0), o).reason == Reason::NotAllowed);
    o.source = Source::Autopilot;
    o.interrupt = false; // (deferring: the override decides)
    o.precedenceOverride = 5;
    const CommandResult overridden = w.submit(f16, heading(1.0), o);
    REQUIRE(overridden.accepted());
    CHECK(overridden.flags & kDeferred); // 5 comes after the velocity's 0
    CHECK(w.activity(overridden.activity)->precedence == 5);
}

TEST_CASE("schedule: a start window delays the start; an end window ends a hold; a critical window missed fails the activity", "[schedule]") {
    session::World w(options("schedule-windows"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const double t0 = w.simTime();

    // a start window: it waits, planned, until it opens
    CommandOptions later;
    later.window.startNotBefore = t0 + 1.0;
    const CommandResult scheduled = w.submit(f16, heading(1.0), later);
    REQUIRE(scheduled.accepted());
    CHECK(scheduled.flags & kDeferred);
    CHECK(scheduled.other == 0);
    CHECK(w.activity(scheduled.activity)->waiting == ActivityWait::Scheduled);
    fly(w, 0.5);
    CHECK(w.activity(scheduled.activity)->waiting == ActivityWait::Scheduled);
    fly(w, 0.6);
    CHECK(w.activity(scheduled.activity)->waiting == ActivityWait::None);
    fly(w, 0.1);
    CHECK(w.activity(scheduled.activity)->state == ActivityState::Active);

    // an end window: a persistent activity is done when it closes
    CommandOptions until;
    until.window.endNotAfter = w.simTime() + 0.5;
    const CommandResult held = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, until);
    REQUIRE(held.accepted());
    fly(w, 0.4);
    CHECK(w.activity(held.activity)->live());
    fly(w, 0.2);
    CHECK(w.activity(held.activity)->state == ActivityState::Completed);
    CHECK(w.activity(held.activity)->reason == Reason::GoalReached);

    // a critical start window closes while it waits for what ranks ahead of it: it fails
    const CommandResult ahead = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, ranked(0));
    REQUIRE(ahead.accepted());
    CommandOptions critical = ranked(3);
    critical.window.startNotAfter = w.simTime() + 0.5;
    critical.window.criticality = TimeCriticality::Start;
    const CommandResult missed = w.submit(f16, heading(1.0), critical);
    REQUIRE(missed.accepted());
    CHECK(missed.flags & kDeferred);
    fly(w, 0.6);
    CHECK(w.activity(missed.activity)->state == ActivityState::Failed);
    CHECK(w.activity(missed.activity)->reason == Reason::TimeConstraint);
    // not critical, it waits on and starts late
    critical.window.criticality = TimeCriticality::None;
    critical.window.startNotAfter = w.simTime() + 0.2;
    const CommandResult late = w.submit(f16, heading(1.0), critical);
    REQUIRE(late.accepted());
    fly(w, 0.5);
    CHECK(w.activity(late.activity)->state == ActivityState::Pending);
    REQUIRE(w.cancel(ahead.activity).status == CommandStatus::Canceled);
    CHECK(w.activity(late.activity)->waiting == ActivityWait::None);

    // windows it cannot meet are refused: an end window closed, a critical start window closed; out of order, malformed
    CommandOptions bad;
    bad.window.endNotAfter = w.simTime();
    CHECK(w.submit(f16, heading(1.0), bad).reason == Reason::TimeConstraint);
    bad = CommandOptions{};
    bad.window.startNotAfter = w.simTime() - 1.0;
    bad.window.criticality = TimeCriticality::StartAndEnd;
    CHECK(w.submit(f16, heading(1.0), bad).reason == Reason::TimeConstraint);
    bad.window.criticality = TimeCriticality::End; // (its start is not critical: it starts now, late)
    CHECK(w.submit(f16, heading(1.0), bad).accepted());
    bad = CommandOptions{};
    bad.window.startNotBefore = w.simTime() + 2.0, bad.window.startNotAfter = w.simTime() + 1.0;
    CHECK(w.submit(f16, heading(1.0), bad).reason == Reason::InvalidParameter);
    bad = CommandOptions{};
    bad.window.endNotAfter = std::numeric_limits<double>::infinity();
    CHECK(w.submit(f16, heading(1.0), bad).reason == Reason::InvalidParameter);
}

TEST_CASE("schedule: a terminating activity with a critical end window fails done too early or too late", "[schedule]") {
    session::World w(options("schedule-terminating"));
    const auto c172 = wing(w, "c172x", 1500.0, 50.0);
    // the flaps get there in a few seconds: before an end window that opens in a minute, which is critical
    CommandOptions o;
    o.window.endNotBefore = w.simTime() + 60.0;
    o.window.criticality = TimeCriticality::End;
    const CommandResult early = w.submit(c172, FlapsCommand{0.5}, o);
    REQUIRE(early.accepted());
    fly(w, 20.0);
    CHECK(w.activity(early.activity)->state == ActivityState::Failed);
    CHECK(w.activity(early.activity)->reason == Reason::TimeConstraint);
    // not critical, it completes
    o.window.criticality = TimeCriticality::None;
    const CommandResult done = w.submit(c172, FlapsCommand{0.0}, o);
    REQUIRE(done.accepted());
    fly(w, 20.0);
    CHECK(w.activity(done.activity)->state == ActivityState::Completed);

    // too late: a route that cannot be flown by its critical end
    const sim::VehicleState& s = *w.vehicleState(c172);
    Waypoint far;
    far.latitudeRad = s.latitudeRad;
    far.longitudeRad = s.longitudeRad + 20000.0 / (kEarthM * std::cos(s.latitudeRad));
    far.altitudeM = 1500.0;
    CommandOptions deadline;
    deadline.window.endNotAfter = w.simTime() + 5.0;
    deadline.window.criticality = TimeCriticality::End;
    const CommandResult route = w.submit(c172, RouteCommand{}, std::vector<Waypoint>{far}, deadline);
    REQUIRE(route.accepted());
    fly(w, 6.0);
    CHECK(w.activity(route.activity)->state == ActivityState::Failed);
    CHECK(w.activity(route.activity)->reason == Reason::TimeConstraint);
}

TEST_CASE("schedule: what waits is updated and canceled as what flies; a validation says it would wait; sixteen can", "[schedule]") {
    session::World w(options("schedule-queue"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions later;
    later.window.startNotBefore = w.simTime() + 0.5;
    const CommandResult waiting = w.submit(f16, heading(1.0), later);
    REQUIRE(waiting.accepted());

    // an UPDATE changes what it will fly, checked as its NEW was; a wrong type is refused
    const CommandResult updated = w.update(waiting.activity, Command(heading(2.0)));
    CHECK(updated.accepted());
    CHECK(updated.commandId == 0);
    CHECK(w.update(waiting.activity, Command(VelocityCommand{})).reason == Reason::WrongCommandType);
    CHECK(w.update(waiting.activity, Command(heading(std::numeric_limits<double>::infinity()))).reason == Reason::InvalidParameter);
    fly(w, 0.6);
    REQUIRE(flownHsa(w, f16) != nullptr);
    CHECK(std::abs(flownHsa(w, f16)->headingRad - 2.0) < 1e-9);

    // a CANCEL ends one that waits, and it never flew
    later.window.startNotBefore = w.simTime() + 10.0;
    const CommandResult canceled = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, later);
    REQUIRE(canceled.accepted());
    const CommandResult x = w.cancel(canceled.activity);
    CHECK(x.status == CommandStatus::Canceled);
    CHECK(w.activity(canceled.activity)->state == ActivityState::Canceled);
    CHECK(w.activity(canceled.activity)->reason == Reason::Requested);
    CHECK(w.activity(canceled.activity)->waiting == ActivityWait::Scheduled); // (as it ended)
    CHECK(w.cancel(canceled.activity).reason == Reason::ActivityEnded);

    // a validation of one that would wait: valid, deferred, nothing made
    CommandOptions check = ranked(7);
    check.validateOnly = true;
    const std::size_t before = w.activities(f16).size();
    const CommandResult valid = w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, check);
    CHECK(valid.status == CommandStatus::Valid);
    CHECK(valid.flags & kDeferred);
    CHECK(w.activities(f16).size() == before);

    // sixteen wait; the seventeenth is refused, and so is its validation
    for (int i = 0; i < 16; ++i) REQUIRE(w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, later).accepted());
    CHECK(w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, later).reason == Reason::QueueFull);
    later.validateOnly = true;
    CHECK(w.submit(f16, VelocityCommand{160.0, 0.0, 1.5}, later).reason == Reason::QueueFull);
    later.validateOnly = false;
    // the policy's waiting activities end with its authority: under Granted, those it holds no grant for
    REQUIRE(w.setControlMode(f16, ControlMode::Granted) == Reason::None);
    std::size_t waitingNow = 0;
    for (const auto& a : w.activities(f16)) waitingNow += a.live() && a.waiting != ActivityWait::None;
    CHECK(waitingNow == 0);
    CHECK(w.activities(f16).front().state != ActivityState::Pending);
}
