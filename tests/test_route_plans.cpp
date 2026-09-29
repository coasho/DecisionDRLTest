// Route plans (docs/flight-autonomy.md, 4.39; ADR-29 FA-7a): A-GRA's route
// plans kept by id and version and taken through the VI's plan activation
// states (1.2.5) - converted and uploaded, prepared for activation, activated,
// deactivated - their execution reported (1.2.6.6), FA's own deactivation
// (1.2.5.7), the queries for their ids and content (1.2.4.2, 1.2.6.4), and their
// planning metadata read back as given (WPT-23).
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <string_view>
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

/// A plan east of where the aircraft is - 3 km, then 6 km, at 1,500 m - with metadata on both points and on its one path.
RoutePlan eastPlan(const sim::VehicleState& s, PlanId id, std::uint32_t version) {
    RoutePlan p;
    p.id = id, p.version = version;
    p.waypoints = {at(s, 0.0, 3000.0, 1500.0), at(s, 0.0, 6000.0, 1500.0)};
    p.detailed = true, p.remarksName = "east", p.remarks = "two points east, level";
    PointMetadata a;
    a.point = 0, a.source = PointSource::OperatorDefined, a.locked = true, a.remarks = "the first";
    a.fixKey = "ALPHA", a.fixSystem = "DAFIF";
    PointMetadata b;
    b.point = 1, b.modified = true, b.remarksName = "last";
    p.pointMetadata = {a, b};
    PathMetadata path;
    path.initial.latitudeRad = s.latitudeRad, path.initial.longitudeRad = s.longitudeRad, path.initial.altitudeM = 1500.0;
    path.initial.yawRad = 1.5, path.enduranceS = 7200.0, path.fuelKg = 80.0, path.grossWeightKg = 1000.0, path.transitionPlan = 9;
    p.pathMetadata = {path};
    return p;
}

/// Prepared for upload, published and uploaded.
void upload(session::World& w, std::uint32_t v, const RoutePlan& p) {
    REQUIRE(w.planCommand(v, p.id, PlanCommand::PrepareForUpload).completed);
    REQUIRE(w.publishPlan(v, p) == Reason::None);
    const PlanCommandResult r = w.planCommand(v, p.id, PlanCommand::Upload);
    REQUIRE(r.completed);
    REQUIRE(r.state == PlanState::Uploaded);
}

} // namespace

TEST_CASE("route plans: converted and uploaded, queried, their metadata read back as given (VI 1.2.5.2, 1.2.4.2, 1.2.6.4)", "[plan]") {
    session::World w(options("plans-upload"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const RoutePlan p = eastPlan(*w.vehicleState(v), 17, 3);
    // FA listens for those prepared for upload alone
    CHECK(w.publishPlan(v, p) == Reason::WrongPlanState);
    CHECK(w.planCommand(v, 17, PlanCommand::Upload).reason == Reason::UnknownPlan);
    PlanCommandResult r = w.planCommand(v, 17, PlanCommand::PrepareForUpload);
    CHECK((r.completed && r.state == PlanState::ReadyForUpload && r.plan == 17 && r.command == PlanCommand::PrepareForUpload));
    auto st = w.planStatus(v, 17);
    REQUIRE(st.has_value());
    CHECK((st->state == PlanState::ReadyForUpload && st->revision == 0 && st->version == 0 && st->execution == PlanExecution::None));
    // what it refuses to take: FA listens on
    RoutePlan bad = p;
    bad.id = 0;
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.pointMetadata[1].point = 2; // (a point the route has not)
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.pointMetadata[1].point = 0; // (twice for one)
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.remarks = "tab\there"; // (not printable)
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.pointMetadata[0].remarksName = std::string(33, 'x'); // (longer than A-GRA's DisplayName)
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.pathMetadata[0].path = 1; // (without paths, the route is one)
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    bad = p, bad.pathMetadata[0].fuelKg = -1.0;
    CHECK(w.publishPlan(v, bad) == Reason::InvalidParameter);
    CHECK(w.planStatus(v, 17)->state == PlanState::ReadyForUpload);
    // taken, published twice: the later one kept at its upload
    RoutePlan draft = p;
    draft.remarks = "an earlier draft";
    CHECK(w.publishPlan(v, draft) == Reason::None);
    CHECK(w.publishPlan(v, p) == Reason::None);
    CHECK_FALSE(w.plan(v, 17).has_value()); // (nothing kept until its upload)
    r = w.planCommand(v, 17, PlanCommand::Upload);
    CHECK((r.completed && r.state == PlanState::Uploaded));
    CHECK(w.publishPlan(v, p) == Reason::WrongPlanState); // (FA listens no longer)
    r = w.planCommand(v, 17, PlanCommand::Upload);
    CHECK((!r.completed && r.state == PlanState::Uploaded && r.reason == Reason::WrongPlanState));
    st = w.planStatus(v, 17);
    CHECK((st->state == PlanState::Uploaded && st->version == 3 && st->revision == 1 && !st->forPlanningUseOnly));
    // read back as given (A-GRA's query for the plan)
    const auto kept = w.plan(v, 17);
    REQUIRE(kept.has_value());
    CHECK((kept->id == 17 && kept->version == 3 && kept->detailed && kept->remarksName == "east" && kept->remarks == p.remarks));
    REQUIRE(kept->waypoints.size() == 2);
    CHECK(kept->waypoints[1].longitudeRad == p.waypoints[1].longitudeRad);
    REQUIRE(kept->pointMetadata.size() == 2);
    const PointMetadata& a = kept->pointMetadata[0];
    CHECK((a.point == 0 && a.source == PointSource::OperatorDefined && a.locked && !a.modified && a.remarks == "the first"));
    CHECK((a.fixKey == "ALPHA" && a.fixSystem == "DAFIF" && a.remarksName.empty()));
    const PointMetadata& b = kept->pointMetadata[1];
    CHECK((b.point == 1 && b.source == PointSource::AutoRouted && !b.locked && b.modified && b.remarksName == "last"));
    REQUIRE(kept->pathMetadata.size() == 1);
    const PathMetadata& path = kept->pathMetadata[0];
    CHECK((path.path == 0 && path.initial.altitudeM == 1500.0 && path.initial.yawRad == 1.5 && std::isnan(path.initial.timeS)));
    CHECK((path.enduranceS == 7200.0 && path.fuelKg == 80.0 && path.grossWeightKg == 1000.0 && path.transitionPlan == 9));
    // a new version: FA prepared for its upload again, the one kept until then; its revision one more
    RoutePlan next = p;
    next.version = 4, next.remarks = "version four";
    REQUIRE(w.planCommand(v, 17, PlanCommand::PrepareForUpload).completed);
    CHECK(w.plan(v, 17)->version == 3);
    REQUIRE(w.publishPlan(v, next) == Reason::None);
    REQUIRE(w.planCommand(v, 17, PlanCommand::Upload).completed);
    st = w.planStatus(v, 17);
    CHECK((st->version == 4 && st->revision == 2));
    CHECK(w.plan(v, 17)->remarks == "version four");
    // an upload with no plan received fails; the ids kept (A-GRA's query for identifiers only)
    REQUIRE(w.planCommand(v, 18, PlanCommand::PrepareForUpload).completed);
    r = w.planCommand(v, 18, PlanCommand::Upload);
    CHECK((!r.completed && r.state == PlanState::UploadFailed && r.reason == Reason::PlanNotReceived));
    const auto all = w.plans(v);
    REQUIRE(all.size() == 2);
    CHECK((all[0].id == 17 && all[0].state == PlanState::Uploaded && all[0].revision == 2));
    CHECK((all[1].id == 18 && all[1].state == PlanState::UploadFailed && all[1].reason == Reason::PlanNotReceived && all[1].revision == 0));
    CHECK_FALSE(w.plan(v, 18).has_value());
    CHECK(std::string_view(planStateName(PlanState::PreparationForActivationFailed)) == "preparation_for_activation_failed");
    CHECK(std::string_view(planCommandName(PlanCommand::PrepareForUpload)) == "prepare_for_upload");
    CHECK(std::string_view(planExecutionName(PlanExecution::Superseded)) == "superseded");
    CHECK(std::string_view(pointSourceName(PointSource::OperatorDefined)) == "operator_defined");
    CHECK(std::string_view(reasonName(Reason::PlanNotReceived)) == "plan_not_received");
}

TEST_CASE("route plans: prepared for activation, activated, flown to completion (VI 1.2.5.3, 1.2.5.1, 1.2.6.6)", "[plan]") {
    session::World w(options("plans-activate"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState s = *w.vehicleState(v);
    upload(w, v, eastPlan(s, 21, 1));
    CHECK(w.planCommand(v, 21, PlanCommand::Activate).reason == Reason::WrongPlanState); // (not ready for activation)
    // a point it cannot fly: its preparation fails, naming the point
    RoutePlan bad = eastPlan(s, 22, 1);
    bad.waypoints[1].altitudeMinM = 2000.0, bad.waypoints[1].altitudeMaxM = 1800.0; // (a block upside down)
    upload(w, v, bad);
    PlanCommandResult r = w.planCommand(v, 22, PlanCommand::PrepareForActivation);
    CHECK((!r.completed && r.state == PlanState::PreparationForActivationFailed && r.reason == Reason::InvalidWaypoint && r.check.index == 1));
    CHECK(w.planStatus(v, 22)->reason == Reason::InvalidWaypoint);
    CHECK(w.planCommand(v, 22, PlanCommand::Activate).reason == Reason::WrongPlanState);
    // its final checks, then flown
    r = w.planCommand(v, 21, PlanCommand::PrepareForActivation);
    CHECK((r.completed && r.state == PlanState::ReadyForActivation && r.check.status == CommandStatus::Valid));
    CHECK(w.planStatus(v, 21)->execution == PlanExecution::None);
    CommandOptions o;
    o.commandId = 44;
    r = w.planCommand(v, 21, PlanCommand::Activate, o);
    REQUIRE(r.completed);
    CHECK((r.state == PlanState::Activated && r.check.accepted() && r.check.activity != 0 && r.check.commandId == 44));
    const ActivityId flying = r.check.activity;
    auto st = w.planStatus(v, 21);
    CHECK((st->state == PlanState::Activated && st->activity == flying && st->commandId == 44 && std::isfinite(st->startTime)));
    CHECK(st->execution == PlanExecution::Pending); // (not flown yet)
    fly(w, 20.0);
    st = w.planStatus(v, 21);
    CHECK((st->execution == PlanExecution::Executing && st->percent > 0.0 && std::isnan(st->endTime)));
    CHECK(w.planCommand(v, 21, PlanCommand::PrepareForUpload).reason == Reason::WrongPlanState); // (it flies)
    CHECK(w.planCommand(v, 21, PlanCommand::PrepareForActivation).reason == Reason::WrongPlanState);
    CHECK(w.removePlan(v, 21) == Reason::WrongPlanState);
    for (int i = 0; i < 300 && w.activity(flying)->live(); ++i) fly(w, 1.0);
    st = w.planStatus(v, 21);
    CHECK((st->execution == PlanExecution::Complete && st->state == PlanState::Activated && st->reason == Reason::GoalReached));
    CHECK((st->percent == 100.0 && std::isfinite(st->endTime)));
    // done: prepared and flown again, or taken back
    CHECK(w.planCommand(v, 21, PlanCommand::Activate).reason == Reason::WrongPlanState); // (prepared again first)
    r = w.planCommand(v, 21, PlanCommand::Deactivate);
    CHECK((r.completed && r.state == PlanState::Deactivated));
    CHECK(w.planStatus(v, 21)->execution == PlanExecution::Complete); // (as it ended)
    CHECK(w.planCommand(v, 21, PlanCommand::PrepareForActivation).completed);
    CHECK(w.removePlan(v, 21) == Reason::None);
    CHECK_FALSE(w.planStatus(v, 21).has_value());
}

TEST_CASE("route plans: taken back before they execute, not after; superseded (VI 1.2.5.4, 1.2.6.6)", "[plan]") {
    session::World w(options("plans-deactivate"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    upload(w, v, eastPlan(*w.vehicleState(v), 31, 1));
    REQUIRE(w.planCommand(v, 31, PlanCommand::PrepareForActivation).completed);
    PlanCommandResult r = w.planCommand(v, 31, PlanCommand::Deactivate); // ready: taken back
    CHECK((r.completed && r.state == PlanState::Deactivated));
    CHECK(w.planStatus(v, 31)->execution == PlanExecution::None);
    CHECK(w.planCommand(v, 31, PlanCommand::Deactivate).reason == Reason::WrongPlanState);
    // activated to start a minute from now: pending, then taken back - its activity canceled
    REQUIRE(w.planCommand(v, 31, PlanCommand::PrepareForActivation).completed);
    CommandOptions later;
    later.window.startNotBefore = w.simTime() + 60.0;
    r = w.planCommand(v, 31, PlanCommand::Activate, later);
    REQUIRE(r.completed);
    CHECK((r.check.flags & kDeferred) != 0);
    fly(w, 2.0);
    CHECK(w.planStatus(v, 31)->execution == PlanExecution::Pending);
    r = w.planCommand(v, 31, PlanCommand::Deactivate);
    CHECK((r.completed && r.state == PlanState::Deactivated && r.check.status == CommandStatus::Canceled));
    auto st = w.planStatus(v, 31);
    CHECK((st->execution == PlanExecution::Canceled && st->reason == Reason::Requested));
    CHECK(w.activity(st->activity)->state == ActivityState::Canceled);
    // activated at once: executing, so its deactivation fails and it flies on
    REQUIRE(w.planCommand(v, 31, PlanCommand::PrepareForActivation).completed);
    r = w.planCommand(v, 31, PlanCommand::Activate);
    REQUIRE(r.completed);
    fly(w, 2.0);
    const PlanCommandResult refused = w.planCommand(v, 31, PlanCommand::Deactivate);
    CHECK((!refused.completed && refused.state == PlanState::Activated && refused.reason == Reason::PlanExecuting));
    CHECK(w.activity(r.check.activity)->state == ActivityState::Active);
    CHECK(w.planStatus(v, 31)->execution == PlanExecution::Executing);
    // another command takes its axes: superseded, the plan still activated
    HsaCommand h;
    h.headingRad = 0.0, h.speed = 55.0, h.altitudeM = 1500.0;
    REQUIRE(w.submit(v, h).accepted());
    st = w.planStatus(v, 31);
    CHECK((st->execution == PlanExecution::Superseded && st->reason == Reason::Preempted && st->state == PlanState::Activated));
    CHECK(w.planCommand(v, 31, PlanCommand::Deactivate).completed); // (its execution done)
}

TEST_CASE("route plans: FA's own deactivation, before and after they start (VI 1.2.5.7)", "[plan]") {
    session::World w(options("plans-abort"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    const sim::VehicleState s = *w.vehicleState(v);
    upload(w, v, eastPlan(s, 41, 1));
    CHECK(w.abortPlan(v, 41).reason == Reason::WrongPlanState); // (uploaded: not in the activation's way)
    CHECK(w.abortPlan(v, 99).reason == Reason::UnknownPlan);
    REQUIRE(w.planCommand(v, 41, PlanCommand::PrepareForActivation).completed);
    PlanCommandResult r = w.abortPlan(v, 41);
    CHECK((r.completed && r.state == PlanState::Deactivated));
    auto st = w.planStatus(v, 41);
    CHECK((st->execution == PlanExecution::Canceled && st->reason == Reason::Restricted));
    // after it starts: its activity canceled with the platform's reason
    REQUIRE(w.planCommand(v, 41, PlanCommand::PrepareForActivation).completed);
    r = w.planCommand(v, 41, PlanCommand::Activate);
    REQUIRE(r.completed);
    fly(w, 2.0);
    const PlanCommandResult aborted = w.abortPlan(v, 41, Reason::CollisionAvoidance);
    CHECK((aborted.completed && aborted.state == PlanState::Deactivated));
    const ActivityRecord* a = w.activity(r.check.activity);
    CHECK((a->state == ActivityState::Canceled && a->reason == Reason::CollisionAvoidance));
    st = w.planStatus(v, 41);
    CHECK((st->state == PlanState::Deactivated && st->execution == PlanExecution::Canceled && st->reason == Reason::CollisionAvoidance));
    // the platform ending its activity itself - its grant revoked - deactivates it too
    upload(w, v, eastPlan(s, 42, 1));
    REQUIRE(w.setControlMode(v, ControlMode::Granted) == Reason::None);
    REQUIRE(w.requestControl(v, "fsim.guidance.route") == Reason::None);
    REQUIRE(w.planCommand(v, 42, PlanCommand::PrepareForActivation).completed);
    REQUIRE(w.planCommand(v, 42, PlanCommand::Activate).completed);
    fly(w, 2.0);
    REQUIRE(w.revokeControl(v, "fsim.guidance.route") == Reason::None);
    st = w.planStatus(v, 42);
    CHECK((st->state == PlanState::Deactivated && st->execution == PlanExecution::Canceled && st->reason == Reason::Revoked));
}

TEST_CASE("route plans: for planning use only, never activated; the store's room (RPL-11, RPL-01)", "[plan]") {
    session::World w(options("plans-store"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    RoutePlan planning = eastPlan(*w.vehicleState(v), 51, 1);
    planning.forPlanningUseOnly = true;
    upload(w, v, planning);
    CHECK(w.planStatus(v, 51)->forPlanningUseOnly);
    const PlanCommandResult r = w.planCommand(v, 51, PlanCommand::PrepareForActivation);
    CHECK((!r.completed && r.state == PlanState::PreparationForActivationFailed && r.reason == Reason::PlanningOnly));
    CHECK(w.planCommand(v, 51, PlanCommand::Activate).reason == Reason::WrongPlanState);
    // 32 kept: the 33rd fails, and is not kept; one forgotten makes room
    for (PlanId id = 100; id < 131; ++id) REQUIRE(w.planCommand(v, id, PlanCommand::PrepareForUpload).completed);
    CHECK(w.plans(v).size() == 32);
    const PlanCommandResult full = w.planCommand(v, 200, PlanCommand::PrepareForUpload);
    CHECK((!full.completed && full.state == PlanState::PreparationForUploadFailed && full.reason == Reason::PlanStoreFull));
    CHECK_FALSE(w.planStatus(v, 200).has_value());
    CHECK(w.planCommand(v, 0, PlanCommand::PrepareForUpload).reason == Reason::InvalidParameter);
    CHECK(w.removePlan(v, 100) == Reason::None);
    CHECK(w.planCommand(v, 200, PlanCommand::PrepareForUpload).completed);
    CHECK(w.removePlan(v, 999) == Reason::UnknownPlan);
    CHECK(w.planCommand(9999, 51, PlanCommand::Upload).reason == Reason::UnknownVehicle);
    // the support rows
    CHECK(w.supportTable(v)->find("fsim.plan/store")->support == Support::Supported);
    CHECK(w.supportTable(v)->find("fsim.guidance.route/metadata")->support == Support::Supported);
}
