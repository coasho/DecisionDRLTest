// The names and descriptions of the control contracts' enumerations
// (fsim/Capability.h): what the SDKs, the logs and the reports print. Kept
// apart from the runtime's code, whose layout they would otherwise move.
#include "fsim/Capability.h"
#include "fsim/Control.h"

namespace fsim::control {

const char* reasonName(Reason reason) noexcept {
    switch (reason) {
    case Reason::None: return "none";
    case Reason::UnknownVehicle: return "unknown_vehicle";
    case Reason::UnknownCapability: return "unknown_capability";
    case Reason::UnknownActivity: return "unknown_activity";
    case Reason::Unavailable: return "unavailable";
    case Reason::VersionUnsupported: return "version_unsupported";
    case Reason::InvalidParameter: return "invalid_parameter";
    case Reason::OutOfRange: return "out_of_range";
    case Reason::InvalidAxes: return "invalid_axes";
    case Reason::AuthorityHeld: return "authority_held";
    case Reason::ControllerNotAxisAware: return "controller_not_axis_aware";
    case Reason::ActivityEnded: return "activity_ended";
    case Reason::NotUpdatable: return "not_updatable";
    case Reason::WrongCommandType: return "wrong_command_type";
    case Reason::GoalReached: return "goal_reached";
    case Reason::Requested: return "requested";
    case Reason::Preempted: return "preempted";
    case Reason::TargetLost: return "target_lost";
    case Reason::BehaviorFailed: return "behavior_failed";
    case Reason::CapabilityLost: return "capability_lost";
    case Reason::Diverged: return "diverged";
    case Reason::InvalidWaypoint: return "invalid_waypoint";
    case Reason::InvalidCurve: return "invalid_curve";
    case Reason::PerformanceLimit: return "performance_limit";
    case Reason::NotGranted: return "not_granted";
    case Reason::NotAllowed: return "not_allowed";
    case Reason::Revoked: return "revoked";
    case Reason::Released: return "released";
    case Reason::CollisionAvoidance: return "collision_avoidance";
    case Reason::Restricted: return "restricted";
    case Reason::NotSupported: return "not_supported";
    case Reason::NotImplemented: return "not_implemented";
    case Reason::OnGround: return "on_ground";
    case Reason::Airborne: return "airborne";
    case Reason::TimeConstraint: return "time_constraint";
    case Reason::QueueFull: return "queue_full";
    case Reason::NotInteractive: return "not_interactive";
    case Reason::UnknownTask: return "unknown_task";
    case Reason::TaskActive: return "task_active";
    case Reason::InsufficientEndurance: return "insufficient_endurance";
    case Reason::TerrainConflict: return "terrain_conflict";
    case Reason::UnknownPlan: return "unknown_plan";
    case Reason::WrongPlanState: return "wrong_plan_state";
    case Reason::PlanExecuting: return "plan_executing";
    case Reason::PlanNotReceived: return "plan_not_received";
    case Reason::PlanningOnly: return "planning_only";
    case Reason::PlanStoreFull: return "plan_store_full";
    case Reason::ReadOnlyPlan: return "read_only_plan";
    case Reason::SafetyCriticalPlan: return "safety_critical_plan";
    case Reason::UnknownAirfield: return "unknown_airfield";
    default: return "?";
    }
}

const char* reasonDescription(Reason reason) noexcept {
    switch (reason) {
    case Reason::None: return "";
    case Reason::UnknownVehicle: return "no such vehicle";
    case Reason::UnknownCapability: return "no platform defines such a capability";
    case Reason::UnknownActivity: return "no such activity";
    case Reason::Unavailable: return "not available now";
    case Reason::VersionUnsupported: return "the capability is older than the version asked for";
    case Reason::InvalidParameter: return "a value is missing or malformed";
    case Reason::OutOfRange: return "a value is outside the capability's range";
    case Reason::InvalidAxes: return "the capability cannot own those axes";
    case Reason::AuthorityHeld: return "a higher source holds the axes";
    case Reason::ControllerNotAxisAware: return "a controller that is not axis-aware would fly axes owned apart";
    case Reason::ActivityEnded: return "the activity has ended";
    case Reason::NotUpdatable: return "the capability takes no update";
    case Reason::WrongCommandType: return "the command is not the capability's";
    case Reason::GoalReached: return "the activity reached its goal";
    case Reason::Requested: return "canceled on request";
    case Reason::Preempted: return "another activity took its axes";
    case Reason::TargetLost: return "the vehicle it follows is gone";
    case Reason::BehaviorFailed: return "the behaviour could not do what it was asked";
    case Reason::CapabilityLost: return "the capability became unavailable";
    case Reason::Diverged: return "the flight model diverged";
    case Reason::InvalidWaypoint: return "a route point the aircraft cannot fly";
    case Reason::InvalidCurve: return "a curve segment the aircraft cannot fly";
    case Reason::PerformanceLimit: return "beyond what the aircraft can do";
    case Reason::NotGranted: return "the policy holds no grant for it";
    case Reason::NotAllowed: return "the policy may not have it";
    case Reason::Revoked: return "the platform revoked the grant";
    case Reason::Released: return "the policy released its grant";
    case Reason::CollisionAvoidance: return "the platform is avoiding a collision";
    case Reason::Restricted: return "the platform restricts it";
    case Reason::NotSupported: return "a physical exception on this aircraft: its support table names the rule and the evidence";
    case Reason::NotImplemented: return "applicable to this aircraft, not built yet: its support table names the stage";
    case Reason::OnGround: return "the aircraft is on the ground: the airborne guidance waits for it to fly";
    case Reason::Airborne: return "the aircraft is in the air: the ground modes wait for it to land";
    case Reason::TimeConstraint: return "a time window it must meet cannot be met, or was missed";
    case Reason::QueueFull: return "as many activities as can wait already do";
    case Reason::NotInteractive: return "its command said it takes no activity commands";
    case Reason::UnknownTask: return "no task is kept by that id";
    case Reason::TaskActive: return "the task's activity is live";
    case Reason::InsufficientEndurance: return "its flight needs more fuel or charge than the vehicle has above its reserve";
    case Reason::TerrainConflict: return "its path goes below the terrain";
    case Reason::UnknownPlan: return "no route plan is kept by that id";
    case Reason::WrongPlanState: return "the plan is not in a state that takes it";
    case Reason::PlanExecuting: return "the plan executes";
    case Reason::PlanNotReceived: return "no plan by its id was published since FA was prepared for its upload";
    case Reason::PlanningOnly: return "the plan is for planning use only: it is never activated";
    case Reason::PlanStoreFull: return "the vehicle keeps as many plans, or airfields, as it can";
    case Reason::ReadOnlyPlan: return "the plan is FA's own: MA activates it, never replaces or removes it";
    case Reason::SafetyCriticalPlan: return "a takeoff's, a departure's, an approach's or a landing's plan is FA's own alone";
    case Reason::UnknownAirfield: return "a takeoff's or a landing's path names an airfield or a runway the vehicle does not keep";
    default: return "";
    }
}

const char* availabilityName(Availability availability) noexcept {
    switch (availability) {
    case Availability::Available: return "available";
    case Availability::TemporarilyUnavailable: return "temporarily_unavailable";
    case Availability::Faulted: return "faulted";
    case Availability::Disabled: return "disabled";
    case Availability::Unavailable: return "unavailable";
    case Availability::Expended: return "expended";
    }
    return "?";
}

const char* constraintName(Constraint constraint) noexcept {
    switch (constraint) {
    case Constraint::None: return "none";
    case Constraint::MinAirspeed: return "min_airspeed";
    case Constraint::MaxAirspeed: return "max_airspeed";
    case Constraint::MinAltitude: return "min_altitude";
    case Constraint::MaxAltitude: return "max_altitude";
    case Constraint::MinAcceleration: return "min_acceleration";
    case Constraint::MaxAcceleration: return "max_acceleration";
    case Constraint::MaxOrientation: return "max_orientation";
    case Constraint::MaxOrientationRate: return "max_orientation_rate";
    case Constraint::MaxTurnRate: return "max_turn_rate";
    case Constraint::MaxClimbRate: return "max_climb_rate";
    case Constraint::MaxDescentRate: return "max_descent_rate";
    default: return "?";
    }
}

const char* flightModeName(FlightMode mode) noexcept {
    switch (mode) {
    case FlightMode::None: return "none";
    case FlightMode::HsaCsa: return "hsa_csa";
    case FlightMode::WaypointFollowing: return "waypoint_following";
    case FlightMode::CurveFollowing: return "curve_following";
    case FlightMode::Loiter: return "loiter";
    case FlightMode::Formation: return "formation";
    case FlightMode::MustFly: return "must_fly";
    case FlightMode::AltitudeStackedMarshall: return "altitude_stacked_marshall";
    case FlightMode::Launch: return "launch";
    case FlightMode::Recovery: return "recovery";
    case FlightMode::RouteIntercept: return "route_intercept";
    default: return "?";
    }
}

const char* requirementKindName(RequirementKind kind) noexcept {
    switch (kind) {
    case RequirementKind::None: return "none";
    case RequirementKind::Effect: return "effect";
    case RequirementKind::Action: return "action";
    case RequirementKind::Task: return "task";
    case RequirementKind::Command: return "command";
    default: return "?";
    }
}

const char* activityStateName(ActivityState state) noexcept {
    switch (state) {
    case ActivityState::Pending: return "pending";
    case ActivityState::Active: return "active";
    case ActivityState::Completed: return "completed";
    case ActivityState::Failed: return "failed";
    case ActivityState::Canceled: return "canceled";
    case ActivityState::Disabled: return "disabled";
    case ActivityState::Deleted: return "deleted";
    default: return "?";
    }
}

const char* timeCriticalityName(TimeCriticality criticality) noexcept {
    switch (criticality) {
    case TimeCriticality::None: return "none";
    case TimeCriticality::Start: return "start";
    case TimeCriticality::End: return "end";
    case TimeCriticality::StartAndEnd: return "start_and_end";
    default: return "?";
    }
}

const char* activityWaitName(ActivityWait wait) noexcept {
    switch (wait) {
    case ActivityWait::None: return "none";
    case ActivityWait::Scheduled: return "scheduled";
    case ActivityWait::Queued: return "queued";
    default: return "?";
    }
}

const char* activityBasisName(ActivityBasis basis) noexcept {
    switch (basis) {
    case ActivityBasis::Actual: return "actual";
    case ActivityBasis::Sensed: return "sensed";
    case ActivityBasis::Predicted: return "predicted";
    case ActivityBasis::Planned: return "planned";
    default: return "?";
    }
}

const char* activityCommandName(ActivityCommand command) noexcept {
    switch (command) {
    case ActivityCommand::Disable: return "disable";
    case ActivityCommand::Enable: return "enable";
    case ActivityCommand::Reset: return "reset";
    case ActivityCommand::Delete: return "delete";
    case ActivityCommand::ChangeRank: return "change_rank";
    case ActivityCommand::Unassign: return "unassign";
    default: return "?";
    }
}

const char* endPointKindName(EndPointKind kind) noexcept {
    switch (kind) {
    case EndPointKind::Waypoint: return "waypoint";
    case EndPointKind::TurnPoint: return "turn_point";
    case EndPointKind::LoiterPoint: return "loiter_point";
    default: return "?";
    }
}

const char* taskStateName(TaskState state) noexcept {
    switch (state) {
    case TaskState::AwaitingExecution: return "awaiting_execution";
    case TaskState::ExecutionPending: return "execution_pending";
    case TaskState::Executing: return "executing";
    case TaskState::Completed: return "completed";
    case TaskState::Dropped: return "dropped";
    case TaskState::Failed: return "failed";
    case TaskState::Canceled: return "canceled";
    default: return "?";
    }
}

const char* planCommandName(PlanCommand command) noexcept {
    switch (command) {
    case PlanCommand::PrepareForUpload: return "prepare_for_upload";
    case PlanCommand::Upload: return "upload";
    case PlanCommand::PrepareForActivation: return "prepare_for_activation";
    case PlanCommand::Activate: return "activate";
    case PlanCommand::Deactivate: return "deactivate";
    default: return "?";
    }
}

const char* planStateName(PlanState state) noexcept {
    switch (state) {
    case PlanState::Inactive: return "inactive";
    case PlanState::ReadyForUpload: return "ready_for_upload";
    case PlanState::PreparationForUploadFailed: return "preparation_for_upload_failed";
    case PlanState::UploadFailed: return "upload_failed";
    case PlanState::Uploaded: return "uploaded";
    case PlanState::PreparationForActivationFailed: return "preparation_for_activation_failed";
    case PlanState::ReadyForActivation: return "ready_for_activation";
    case PlanState::ActivationFailed: return "activation_failed";
    case PlanState::Activated: return "activated";
    case PlanState::Deactivated: return "deactivated";
    default: return "?";
    }
}

const char* planExecutionName(PlanExecution execution) noexcept {
    switch (execution) {
    case PlanExecution::None: return "none";
    case PlanExecution::Pending: return "pending";
    case PlanExecution::Executing: return "executing";
    case PlanExecution::Complete: return "complete";
    case PlanExecution::Superseded: return "superseded";
    case PlanExecution::Canceled: return "canceled";
    case PlanExecution::Failed: return "failed";
    default: return "?";
    }
}

const char* pointSourceName(PointSource source) noexcept {
    switch (source) {
    case PointSource::AutoRouted: return "auto_routed";
    case PointSource::OperatorDefined: return "operator_defined";
    default: return "?";
    }
}

const char* energyName(Energy e) noexcept {
    switch (e) {
    case Energy::Unknown: return "unknown";
    case Energy::Fuel: return "fuel";
    case Energy::Battery: return "battery";
    default: return "?";
    }
}

const char* contingencyName(Contingency c) noexcept {
    switch (c) {
    case Contingency::Normal: return "NORMAL";
    case Contingency::MissionCritical: return "MISSION_CRITICAL";
    case Contingency::FlightCritical: return "FLIGHT_CRITICAL";
    case Contingency::LostComms: return "LOST_COMMS";
    default: return "?";
    }
}

} // namespace fsim::control
