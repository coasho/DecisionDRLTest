// Flight tasks and suggestions (docs/flight-autonomy.md, 4.11; ADR-29 FA-2):
// a command kept by id, flown on a task command with the task among the
// requirements it traces to, as often as its repetition says, its state
// reported; a command refused that Clamp would fly, and a waiting one that
// fails as it would start, name a task the platform keeps with what it can fly.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

Waypoint at(const sim::VehicleState& s, double north, double east, double altitudeM) {
    Waypoint p;
    p.latitudeRad = s.latitudeRad + north / kEarthM;
    p.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    p.altitudeM = altitudeM;
    return p;
}

BehaviorCommand aileronRoll() {
    BehaviorCommand b;
    b.id = "aerobatics";
    b.params = {{"manoeuvre", 0.0}};
    return b;
}

bool traces(const ActivityRecord& a, TaskId task) {
    for (const Requirement& r : a.trace)
        if (r.kind == RequirementKind::Task && r.id == task) return true;
    return false;
}

} // namespace

TEST_CASE("tasks: kept, flown on a task command with the task in its trace, and reported", "[task]") {
    session::World w(options("tasks-store"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    // what it refuses to keep
    CHECK(w.storeTask(f16, 0, Command(aileronRoll())) == Reason::InvalidParameter);
    CHECK(w.storeTask(f16, kSuggestedTask | 1, Command(aileronRoll())) == Reason::InvalidParameter);
    HsaCommand h;
    h.headingRad = 1.0;
    CHECK(w.storeTask(f16, 2, Command(h), {}, {}, TaskRepetition{3, 1.0}) == Reason::InvalidParameter); // (runs of what never completes)
    BehaviorCommand hover;
    hover.id = "hover";
    CHECK(w.storeTask(f16, 3, Command(hover)) == Reason::NotSupported); // (a wing does not hover)

    REQUIRE(w.storeTask(f16, 7, Command(aileronRoll())) == Reason::None);
    auto s = w.taskStatus(f16, 7);
    REQUIRE(s.has_value());
    CHECK(s->state == TaskState::AwaitingExecution);
    CHECK(s->activity == 0);
    CHECK(s->runs == 1);
    CHECK_FALSE(s->suggested);
    CHECK(w.commandTask(f16, 99).reason == Reason::UnknownTask);

    CommandOptions o;
    o.commandId = 5;
    const CommandResult r = w.commandTask(f16, 7, o);
    REQUIRE(r.accepted());
    CHECK(r.commandId == 5);
    CHECK(traces(*w.activity(r.activity), 7));
    s = w.taskStatus(f16, 7);
    CHECK(s->state == TaskState::ExecutionPending);
    CHECK(s->activity == r.activity);
    CHECK(s->commandId == 5);
    CHECK(w.commandTask(f16, 7).reason == Reason::TaskActive);
    CHECK(w.storeTask(f16, 7, Command(aileronRoll())) == Reason::TaskActive);
    CHECK(w.removeTask(f16, 7) == Reason::TaskActive);
    fly(w, 0.5);
    CHECK(w.taskStatus(f16, 7)->state == TaskState::Executing);
    for (int i = 0; i < 30 && w.activity(r.activity)->live(); ++i) fly(w, 1.0);
    s = w.taskStatus(f16, 7);
    CHECK(s->state == TaskState::Completed);
    CHECK(s->percent == 100.0);
    CHECK(std::isfinite(s->endTime));

    // again, then canceled: its activity ends, and the task says so
    const CommandResult again = w.commandTask(f16, 7);
    REQUIRE(again.accepted());
    CHECK(again.activity != r.activity);
    CHECK(w.cancelTask(f16, 7).status == CommandStatus::Canceled);
    CHECK(w.activity(again.activity)->state == ActivityState::Canceled);
    CHECK(w.taskStatus(f16, 7)->state == TaskState::Canceled);
    CHECK(w.tasks(f16).size() == 1);
    CHECK(w.removeTask(f16, 7) == Reason::None);
    CHECK_FALSE(w.taskStatus(f16, 7).has_value());
    CHECK(std::string_view(taskStateName(TaskState::ExecutionPending)) == "execution_pending");
}

TEST_CASE("tasks: its runs are one activity's, each its interval after the one before completes", "[task]") {
    session::World w(options("tasks-runs"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    REQUIRE(w.storeTask(f16, 11, Command(aileronRoll()), {}, {}, TaskRepetition{3, 2.0}) == Reason::None);
    const CommandResult r = w.commandTask(f16, 11);
    REQUIRE(r.accepted());
    const ActivityRecord* a = w.activity(r.activity);
    CHECK((a->run == 1 && a->runs == 3));
    // the first run done: the activity stays active, and the second begins its interval later
    std::uint16_t seen = 1;
    double percent = 0.0;
    for (int i = 0; i < 400 && w.activity(r.activity)->live(); ++i) {
        fly(w, 0.25);
        a = w.activity(r.activity);
        if (a->live()) CHECK(a->state == ActivityState::Active);
        const auto s = w.taskStatus(f16, 11);
        CHECK(s->percent >= percent - 1e-9); // (it only grows)
        percent = s->percent;
        seen = std::max(seen, a->run);
    }
    a = w.activity(r.activity);
    CHECK(a->state == ActivityState::Completed);
    CHECK(seen == 3);
    const auto s = w.taskStatus(f16, 11);
    CHECK(s->state == TaskState::Completed);
    CHECK((s->run == 3 && s->runs == 3));
    CHECK(s->percent == 100.0);
}

TEST_CASE("suggestions: a command refused that Clamp would fly, and a waiting one failing as it would start, name a task", "[task]") {
    session::World w(options("tasks-suggest"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    HsaCommand fast;
    fast.headingRad = 1.5, fast.speed = 600.0, fast.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
    const CommandResult refused = w.submit(f16, fast, reject);
    CHECK(refused.reason == Reason::PerformanceLimit);
    const TaskId suggestion = w.commandDetails(f16)->suggestion;
    REQUIRE(suggestion != 0);
    CHECK((suggestion & kSuggestedTask) != 0);
    auto s = w.taskStatus(f16, suggestion);
    REQUIRE(s.has_value());
    CHECK(s->suggested);
    CHECK(s->state == TaskState::AwaitingExecution);
    // flown as it suggests, with the speed held to the most the aircraft flies
    const CommandResult flown = w.commandTask(f16, suggestion, reject);
    REQUIRE(flown.accepted());
    CHECK(w.commandDetails(f16)->suggestion == 0);
    // one no clamp mends (an unknown field's value) is suggested nothing; nor is a validation
    reject.validateOnly = true;
    CHECK(w.submit(f16, fast, reject).reason == Reason::PerformanceLimit);
    CHECK(w.commandDetails(f16)->suggestion == 0);
    reject.validateOnly = false;

    // a waiting route whose climb is too steep from where the aircraft has flown by its start: it fails, naming its suggestion
    const sim::VehicleState& st = *w.vehicleState(f16);
    const std::vector<Waypoint> up = {at(st, 0, 6000, 3400)};
    CommandOptions later = reject;
    later.window.startNotBefore = w.simTime() + 20.0;
    const CommandResult waits = w.submit(f16, RouteCommand{}, up, later);
    REQUIRE(waits.accepted());
    REQUIRE((waits.flags & kDeferred));
    fly(w, 21.0);
    const ActivityRecord* failed = w.activity(waits.activity);
    REQUIRE(failed != nullptr);
    CHECK(failed->state == ActivityState::Failed);
    CHECK(failed->reason == Reason::PerformanceLimit);
    REQUIRE(failed->suggestion != 0);
    s = w.taskStatus(f16, failed->suggestion);
    REQUIRE(s.has_value());
    CHECK(s->suggested);
    // what it suggests flies (clamped: the climb at the most the aircraft climbs)
    CHECK(w.commandTask(f16, failed->suggestion).accepted());
}
