"""A-GRA's vocabulary for what the platform answers (docs/vehicle-interface.md, 5.4 and Appendix B).

The platform speaks its own terms: a command's status and reason, an
activity's state and constraint flags, a capability. A consumer that reasons in
A-GRA ASK 6.0a's terms - a mission autonomy - translates them here into the
names the Vehicle Interface volume and its schema use. There are no messages:
these are names only (ADR-28, decision D1).

    >>> from fsim import agra
    >>> agra.activity_state(activity.info)          # 'ACTIVE_UNCONSTRAINED'
    >>> agra.cannot_comply(rejected.reason)         # 'INVALID_WAYPOINT'
    >>> agra.flight_capabilities(vehicle)           # {'HSA_CSA': ['fsim.guidance.hsa'], 'LOITER': [...], ...}
"""
import datetime
import math

from . import _native
from .world import ActivityBasis, ActivityState, ActivityWait, Energy, NurbsSegment, Rank, TimeCriticality, TimeWindow

#: CommandStatus (0 accepted, 1 rejected, 2 canceled, 3 valid) -> CommandProcessingStateEnum. RECEIVED is never
#: needed: every command is answered at once. A validation that would be accepted is ACCEPTED, and its
#: validation result FLIGHT_COMMAND_VALID (validation_results).
COMMAND_PROCESSING_STATE = {0: "ACCEPTED", 1: "REJECTED", 2: "CANCELED", 3: "ACCEPTED"}

#: fsim.RequirementKind names -> the element of A-GRA's RequirementInstanceID_ChoiceType a Traceability names
REQUIREMENT = {"effect": "EffectID", "action": "ActionID", "task": "TaskID", "command": "CapabilityCommandID"}

#: Constraint flags an active activity carries: the demand limited, a value clamped, a support axis taken, a route off
#: its required navigation performance (partly constrained); an effector at its stop, a limit exceeded (fully constrained).
_PARTLY = 2 | 8 | 16 | 32
_FULLY = 1 | 4

#: reason names (fsim.Rejected.reason, ActivityInfo.reason) -> CannotComplyEnum
CANNOT_COMPLY = {
    "unknown_capability": "CAPABILITY_UNAVAILABLE",
    "unknown_vehicle": "UNKNOWN_ID",
    "unknown_activity": "UNKNOWN_ID",
    "unavailable": "CAPABILITY_UNAVAILABLE",
    "version_unsupported": "INPUT_OTHER",
    "invalid_parameter": "INVALID_INPUT_PARAMETER",
    "out_of_range": "CAPABILITY_PERFORMANCE",
    "invalid_axes": "INPUT_OTHER",
    "authority_held": "CAPABILITY_PRECEDENCE",
    "controller_not_axis_aware": "SYSTEM_CONFLICT",
    "activity_ended": "STATE_OR_SETTINGS",
    "not_updatable": "INPUT_OTHER",
    "wrong_command_type": "INPUT_OTHER",
    "requested": "CANCELED",
    "preempted": "CAPABILITY_PRECEDENCE",
    "target_lost": "MISSION_EVENT",
    "behavior_failed": "CAPABILITY_FAULT",
    "capability_lost": "CAPABILITY_FAULT",
    "diverged": "SYSTEM_FAULT",
    "invalid_waypoint": "INFEASIBLE_ROUTE",
    "invalid_curve": "INFEASIBLE_ROUTE",
    "performance_limit": "CAPABILITY_PERFORMANCE",
    "not_granted": "INELIGIBLE_CONTROL_SOURCE",
    "not_allowed": "INELIGIBLE_CONTROL_SOURCE",
    "revoked": "CANCELED",
    "released": "CANCELED",
    "collision_avoidance": "CONSTRAINT_COLLISION_AVOIDANCE",
    "restricted": "CONSTRAINT_OP",
    "not_supported": "CAPABILITY_UNAVAILABLE",
    "not_implemented": "CAPABILITY_UNAVAILABLE",
    "on_ground": "STATE_OR_SETTINGS",
    "airborne": "STATE_OR_SETTINGS",
    "time_constraint": "CONSTRAINT_TIME",
    "queue_full": "INSUFFICIENT_RESOURCES",
    "not_interactive": "STATE_OR_SETTINGS",
    "unknown_task": "UNKNOWN_ID",
    "task_active": "STATE_OR_SETTINGS",
    "insufficient_endurance": "CONSTRAINT_ENDURANCE",
    "terrain_conflict": "CONSTRAINT_SAFETY",
    "unknown_plan": "UNKNOWN_ID",
    "wrong_plan_state": "STATE_OR_SETTINGS",
    "plan_executing": "STATE_OR_SETTINGS",
    "plan_not_received": "INIT_CRITERIA_NOT_MET",
    "planning_only": "STATE_OR_SETTINGS",
    "plan_store_full": "INSUFFICIENT_RESOURCES",
    "read_only_plan": "INELIGIBLE_CONTROL_SOURCE",
    "safety_critical_plan": "CONSTRAINT_SAFETY",
    "unknown_airfield": "UNKNOWN_ID",
}

#: fsim.PlanState -> A-GRA's PlanActivationStateEnum (docs/flight-autonomy.md, 4.39): the states FA reaches
PLAN_ACTIVATION_STATE = {0: "INACTIVE", 1: "READY_FOR_UPLOAD", 2: "PREPARATION_FOR_UPLOAD_FAILED", 3: "UPLOAD_FAILED", 4: "UPLOADED",
                         5: "PREPARATION_FOR_ACTIVATION_FAILED", 6: "READY_FOR_ACTIVATION", 7: "ACTIVATION_FAILED", 8: "ACTIVATED",
                         9: "DEACTIVATED"}

#: fsim.PlanExecution -> A-GRA's PlanExecutionStateEnum (its spelling, SUPERCEDED); a plan never activated has none
PLAN_EXECUTION_STATE = {0: None, 1: "PENDING", 2: "EXECUTING", 3: "COMPLETE", 4: "SUPERCEDED", 5: "CANCELED", 6: "FAILED"}

#: fsim.PlanCommand -> A-GRA's PlanActivationCommandEnum
PLAN_ACTIVATION_COMMAND = {0: "PREPARE_FOR_UPLOAD", 1: "UPLOAD", 2: "PREPARE_FOR_ACTIVATION", 3: "ACTIVATE", 4: "DEACTIVATE"}

#: fsim.TaskState -> A-GRA's RequirementExecutionStateEnum (a task kept, not commanded, awaits approval to execute)
REQUIREMENT_EXECUTION_STATE = {0: "AWAITING_EXECUTION_APPROVAL", 1: "EXECUTION_PENDING", 2: "EXECUTING", 3: "COMPLETED", 4: "DROPPED",
                               5: "FAILED", 6: "CANCELED"}

#: fsim.TimeCriticality names <-> A-GRA's SchedulingCriticalityEnum (its omission: none is critical)
SCHEDULING_CRITICALITY = {"none": None, "start": "START_TIME_CRITICAL", "end": "END_TIME_CRITICAL",
                          "start_and_end": "START_AND_END_TIME_CRITICAL"}

#: fsim.ActivityBasis names -> A-GRA's ActivityBasisEnum
ACTIVITY_BASIS = {"actual": "ACTUAL", "sensed": "SENSED", "predicted": "PREDICTED", "planned": "PLANNED"}

#: reason names -> MA_ValidationResultEnum, for a flight command's rejection (CannotComplyDetails)
VALIDATION_RESULT = {
    "unknown_capability": "CAPABILITY_NOT_SUPPORTED",
    "not_supported": "CAPABILITY_NOT_SUPPORTED",
    "not_implemented": "CAPABILITY_NOT_SUPPORTED",
    "invalid_waypoint": "INVALID_WAYPOINT",
    "invalid_curve": "INVALID_CURVE",
    "out_of_range": "PERFORMANCE_LIMIT_EXCEEDED",
    "performance_limit": "PERFORMANCE_LIMIT_EXCEEDED",
    "insufficient_endurance": "VIOLATION_ENDURANCE",
    "terrain_conflict": "VIOLATION_TERRAIN",
}

#: constraint names (fsim.Rejected.constraint) -> MA_PerformanceConstraintEnum
PERFORMANCE_CONSTRAINT = {
    "min_airspeed": "MIN_AIRSPEED_EXCEEDED",
    "max_airspeed": "MAX_AIRSPEED_EXCEEDED",
    "min_altitude": "MIN_ALTITUDE_EXCEEDED",
    "max_altitude": "MAX_ALTITUDE_EXCEEDED",
    "min_acceleration": "MIN_ACCELERATION_LIMIT_EXCEEDED",
    "max_acceleration": "MAX_ACCELERATION_LIMIT_EXCEEDED",
    "max_orientation": "MAX_ORIENTATION_LIMIT_EXCEEDED",
    "max_orientation_rate": "MAX_ORIENTATION_RATE_LIMIT_EXCEEDED",
    "max_turn_rate": "MAX_TURN_RATE_EXCEEDED",
    "max_climb_rate": "MAX_CLIMB_RATE_EXCEEDED",
    "max_descent_rate": "MAX_DESCENT_RATE_EXCEEDED",
}

#: Capability.mode -> MA_FlightCapabilityEnum
FLIGHT_CAPABILITY = {
    "hsa_csa": "HSA_CSA",
    "waypoint_following": "WAYPOINT_FOLLOWING",
    "curve_following": "CURVE_FOLLOWING",
    "loiter": "LOITER",
    "formation": "FORMATION",
    "must_fly": "MUST_FLY",
    "altitude_stacked_marshall": "ALTITUDE_STACKED_MARSHALL",
    "launch": "LAUNCH",
    "recovery": "RECOVERY",
    "route_intercept": "ROUTE_INTERCEPT",
}

#: SpeedReference codes (ActivityProgress.speed_reference) -> A-GRA's SpeedReferenceEnum, or MachType
SPEED_REFERENCE = {0: "TRUE_AIRSPEED", 1: "CALIBRATED_AIRSPEED", 2: "GROUNDSPEED", 3: "MACH"}

#: SpeedOptimization codes (an hsa's or a pattern's speed_optimization) -> A-GRA's SpeedOptimizationEnum
SPEED_OPTIMIZATION = {0: "LONG_RANGE_CRUISE", 1: "MAX_ENDURANCE"}

#: AltitudeReference codes (CommandedState.altitude_reference, EndPoint.altitude_reference) -> A-GRA's AltitudeReferenceEnum
ALTITUDE_REFERENCE = {0: "MSL", 1: "AGL", 2: "WGS_HAE", 3: "ALTITUDE_BAROMETRIC"}

#: DirectionReference codes (an hsa's direction_reference) -> A-GRA's MA_HeadingReferenceEnum
HEADING_REFERENCE = {0: "TRUE_NORTH", 1: "MAGNETIC_NORTH"}

#: fsim.EndPointKind -> the element of A-GRA's MA_EndPointType choice
END_POINT = {0: "WayPoint", 1: "TurnPoint", 2: "LoiterPoint"}

#: TurnType codes (a turn point's) -> A-GRA's TurnPointTypeEnum
TURN_POINT_TYPE = {0: "TURN_SHORT", 1: "FLY_OVER"}

#: A quintic Bezier as A-GRA's NURBS has it: its six control points weighted 1, and this clamped knot vector
KNOT_VECTOR = (0.0,) * 6 + (1.0,) * 6


def command_processing_state(status):
    """A command's status (a result's first field, or fsim's CommandStatus value) as A-GRA's
    CommandProcessingStateEnum."""
    return COMMAND_PROCESSING_STATE[int(status)]


def activity_state(info, now=None):
    """An activity's record (ActivityInfo) as A-GRA's ActivityStateEnum. Pending is ENABLED - but one queued
    for its axes whose start window is open is ACTIVE_FULLY_CONSTRAINED: what triggers it holds, and what it
    may not interrupt keeps it from flying (docs/flight-autonomy.md, 4.9; ``now``, the simulation time, else
    its window is taken as open). An active one is unconstrained, partly constrained (its demand limited, a
    value clamped, a support axis taken) or fully constrained (an effector at its stop, a limit exceeded); a
    canceled one FAILED, with the reason CANCELED (cannot_comply)."""
    state = ActivityState(info.state)
    if state == ActivityState.PENDING:
        if info.waiting == ActivityWait.QUEUED and (now is None or not now < info.window.start_not_before):
            return "ACTIVE_FULLY_CONSTRAINED"
        return "ENABLED"
    if state == ActivityState.ACTIVE:
        if info.constraints & _FULLY:
            return "ACTIVE_FULLY_CONSTRAINED"
        if info.constraints & _PARTLY:
            return "ACTIVE_PARTIALLY_CONSTRAINED"
        return "ACTIVE_UNCONSTRAINED"
    if state == ActivityState.COMPLETED:
        return "COMPLETED"
    if state == ActivityState.DISABLED:
        return "DISABLED"
    if state == ActivityState.DELETED:
        return "DELETED"
    return "FAILED"


def activity_basis(info):
    """An activity's record as A-GRA's ActivityBasisEnum: ACTUAL, or PLANNED while it waits to start."""
    return ACTIVITY_BASIS[ActivityBasis(info.basis).name.lower()]


def command_options(ranking=None, temporal=None, override_rejection=None):
    """A flight command's A-GRA ranking and temporal constraints as fsim's submit keywords (docs/flight-autonomy.md,
    4.9). ``ranking``: {"Rank": (priority, precedence), "InterruptOtherActivities": bool or None,
    "CapabilityPrecedenceOverride": int or None} - InterruptOtherActivities left out is False, as A-GRA has it (fsim's
    own default interrupts). ``temporal``: {"StartTimeWindow": (begin, end), "EndTimeWindow": (begin, end),
    "TemporalCriticality": "START_TIME_CRITICAL" ...}, times in simulation seconds, None for a bound left out."""
    out = {}
    if ranking is not None:
        out["rank"] = Rank(*ranking.get("Rank", (0, 0)))
        out["interrupt"] = bool(ranking.get("InterruptOtherActivities") or False)
        if ranking.get("CapabilityPrecedenceOverride") is not None:
            out["precedence_override"] = int(ranking["CapabilityPrecedenceOverride"])
    if temporal is not None:
        start = temporal.get("StartTimeWindow") or (None, None)
        end = temporal.get("EndTimeWindow") or (None, None)
        critical = {v: k for k, v in SCHEDULING_CRITICALITY.items() if v}.get(temporal.get("TemporalCriticality"), "none")
        nan = float("nan")
        out["window"] = TimeWindow(*(nan if t is None else float(t) for t in (*start, *end)), TimeCriticality[critical.upper()])
    if override_rejection is not None:
        out["override_rejection"] = bool(override_rejection)
    return out


def altitude_reference(code):
    """An AltitudeReference code as A-GRA's AltitudeReferenceEnum; None for NaN (none commanded)."""
    return None if code != code else ALTITUDE_REFERENCE[int(code)]


def end_point(point):
    """An end point (fsim.EndPoint, Activity.end_points) as A-GRA's MA_EndPointType choice: its element
    ("WayPoint", "TurnPoint", "LoiterPoint") and, for a turn point, its TurnPointTypeEnum (else None)."""
    kind = END_POINT[int(point.kind)]
    return kind, (TURN_POINT_TYPE[int(point.turn)] if kind == "TurnPoint" else None)


def flyout_curve(setpoint):
    """A curve's setpoint (Activity.setpoint()) as A-GRA's FlyoutCurve (docs/flight-autonomy.md, 4.12): one
    MA_NURBS_PointType per segment it flies, appended ones too - {"CenterReference": (latitude_rad, longitude_rad,
    altitude_m above sea level), "ControlPoints": [((north, east, down), weight)], "KnotVector": [knots]}: a Bezier
    segment's six points weighted 1.0 and KNOT_VECTOR, a segment given as A-GRA's schema gives it (fsim.NurbsSegment,
    4.26) as it was given, with its "Curvature", "InitialControlPointIndex" and "FinalControlPointIndex" where said."""
    if setpoint is None or setpoint.method != "submit_curve":
        raise ValueError("only a curve's setpoint has a flyout curve")
    k = setpoint.kwargs
    reference = (k["latitude_rad"], k["longitude_rad"], k["altitude_m"])
    out = []
    for s in setpoint.args[0]:
        if isinstance(s, NurbsSegment):
            weights = s.weights if s.weights is not None else [1.0] * len(s.north)
            segment = {"CenterReference": reference, "ControlPoints": [((n, e, d), w) for n, e, d, w in zip(s.north, s.east, s.down, weights)],
                       "KnotVector": list(s.knots)}
            for key, value in (("Curvature", s.curvature), ("InitialControlPointIndex", s.first_index),
                               ("FinalControlPointIndex", s.last_index)):
                if value == value:  # (said)
                    segment[key] = int(value) if key != "Curvature" else value
            out.append(segment)
        else:
            out.append({"CenterReference": reference, "ControlPoints": [((n, e, d), 1.0) for n, e, d in zip(s.north, s.east, s.down)],
                        "KnotVector": KNOT_VECTOR})
    return out


def navigation_report(report):
    """A navigation report (fsim.NavigationReport, Vehicle.navigation_report()) as A-GRA's MA_NavigationReport
    (docs/flight-autonomy.md, 4.14): {"Endurance": {"Fuel": kg, "Percent": its fuel's or charge's, "Duration": s},
    "Playtime": s, "ContingencyLevel": its SystemContingencyLevelEnum}. Fuel is 0 for a battery; a value the vehicle
    has none of - Playtime without a recovery point, all of Endurance where it flies on neither - is None."""
    def known(x):
        return None if x != x else x
    flies = report.energy != Energy.UNKNOWN
    return {"Endurance": {"Fuel": report.fuel_kg if flies else None, "Percent": known(report.percent),
                          "Duration": known(report.endurance_s)},
            "Playtime": known(report.playtime_s), "ContingencyLevel": report.contingency.name}


def performance_profile(profile, capacity=None):
    """A performance profile (fsim.PerformanceProfile, Vehicle.performance_profile()) as A-GRA's
    MA_FlightControlModesPerformanceProfileType (docs/flight-autonomy.md, 4.15). Each point carries its airspeed
    ({"Value", "Reference": "TRUE_AIRSPEED"}), its altitude ({"AltitudeReference": "MSL", "Altitude"}) and its weight
    (kg), None where it has none. A-GRA 6.0a's gaps are filled, and named so:
    - MA_SpeedType (MaxDescentRate, ExcessPowerMaxClimb) carries no value: the rate is under "Value";
    - FuelBurnRate's Endurance: a burn as what an hour uses, Duration 3600 s, with its Fuel (kg), or a battery's Percent
      of `capacity` (J: NavigationReport.capacity);
    - the control system's climb limit, which the VI names but the type does not carry, is "MaxClimbRate"."""
    def known(x):
        return None if x != x else x

    def speed(v):
        return None if v != v else {"Value": v, "Reference": "TRUE_AIRSPEED"}

    def altitude(h):
        return None if h != h else {"AltitudeReference": "MSL", "Altitude": h}

    def airspeeds(points):
        return [{"AirspeedLimit": speed(q.value), "AltitudePair": altitude(q.altitude_msl_m), "WeightPair": known(q.weight_kg)} for q in points]

    def acceleration(x, y, z, mach, tas, h, w):
        return {"AccelerationLimit": {"X_Accel": known(x), "Y_Accel": known(y), "Z_Accel": known(z)},
                "AccelerationLimitValue": {"MachValue": known(mach)}, "Airspeed": speed(tas), "Altitude": altitude(h), "Weight": known(w)}

    def accelerations(points):
        return [acceleration(a.x_ms2, a.y_ms2, a.z_ms2, a.mach, a.tas_ms, a.altitude_msl_m, a.weight_kg) for a in points]

    def rate(value, tas, h, w):
        return {"Value": known(value), "Airspeed": speed(tas), "Altitude": altitude(h), "Weight": known(w)}

    burn = []
    for q in profile.burn:
        if profile.energy == Energy.FUEL:
            endurance = {"Fuel": q.value * 3600.0, "Duration": 3600.0}
        elif profile.energy == Energy.BATTERY and capacity:
            endurance = {"Percent": 100.0 * q.value * 3600.0 / capacity, "Duration": 3600.0}
        else:
            endurance = {"Duration": 3600.0}
        burn.append({"Endurance": endurance, "Airspeed": speed(q.tas_ms), "Altitude": altitude(q.altitude_msl_m), "Weight": known(q.weight_kg)})
    power = [{"ExcessPowerMaxClimb": rate(e.climb_ms, e.tas_ms, e.altitude_msl_m, e.weight_kg),
              "ExcessPowerMaxAcceleration": acceleration(e.acceleration_ms2, float("nan"), float("nan"), float("nan"), e.tas_ms,
                                                         e.altitude_msl_m, e.weight_kg)} for e in profile.excess_power]
    return {
        "MinAirspeed": airspeeds(profile.min_airspeed), "MaxAirspeed": airspeeds(profile.max_airspeed),
        "BestEnduranceAirspeed": airspeeds(profile.best_endurance_airspeed), "BestRangeAirspeed": airspeeds(profile.best_range_airspeed),
        "MinAltitude": altitude(profile.min_altitude_msl_m), "MaxAltitude": altitude(profile.max_altitude_msl_m),
        "MinAccelerationLimits": accelerations(profile.min_acceleration), "MaxAccelerationLimits": accelerations(profile.max_acceleration),
        "ExcessPowerOrAcceleration": {"ExcessPower": power} if power else None,
        "MaxOrientationLimits": [{"OrientationLimits": {"Yaw": known(o.yaw_rad), "Pitch": known(o.pitch_rad), "Roll": known(o.roll_rad)},
                                  "Airspeed": speed(o.tas_ms), "Altitude": altitude(o.altitude_msl_m), "Weight": known(o.weight_kg)}
                                 for o in profile.max_orientation],
        "MaxOrientationRateLimits": [{"OrientationRateLimits": {"RollRate": known(r.roll_rad_s), "PitchRate": known(r.pitch_rad_s),
                                                                "YawRate": known(r.yaw_rad_s)}, "AirspeedPair": speed(r.tas_ms)}
                                     for r in profile.max_orientation_rate],
        "MaxTurnRate": known(profile.max_turn_rate_rad_s),
        "MaxClimbRate": known(profile.max_climb_rate_ms),
        "MaxDescentRate": [rate(q.value, q.tas_ms, q.altitude_msl_m, q.weight_kg) for q in profile.max_descent_rate],
        "MaxDeceleration": accelerations(profile.max_deceleration),
        "FuelBurnRate": burn,
    }


def task_state(status):
    """A task's status (fsim.TaskStatus) as A-GRA's RequirementExecutionStateEnum (docs/flight-autonomy.md, 4.11)."""
    return REQUIREMENT_EXECUTION_STATE[int(status.state)]


def plan_activation_status(answer):
    """A plan command's answer (fsim.PlanCommandResult) as A-GRA's MA_MissionPlanActivationCommandStatus's parts
    (docs/flight-autonomy.md, 4.39): its PlanActivationCommandEnum, the plan's PlanActivationStateEnum after it, its
    CommandStatus (COMPLETED or FAILED) and, failed, its CannotComplyEnum."""
    return {"CommandType": PLAN_ACTIVATION_COMMAND[int(answer.command)], "PlanActivationCommandState": PLAN_ACTIVATION_STATE[int(answer.state)],
            "CommandStatus": "COMPLETED" if answer.completed else "FAILED",
            "Reason": None if answer.completed else CANNOT_COMPLY.get(answer.reason, "INPUT_OTHER")}


def plan_execution_state(status):
    """A route plan's status (fsim.PlanStatus) as A-GRA's PlanExecutionStateEnum: None for one never activated."""
    return PLAN_EXECUTION_STATE[int(status.execution)]


def airfield_report(airfield):
    """An airfield (fsim.Airfield) as A-GRA's AirfieldReportMDT's parts (docs/flight-autonomy.md, 4.40): its AirfieldID, its
    Information's ICAO_Code and QNH_Setting (Pa), and its runways - each its RunwayID, Direction, AvailableLength and its
    TakeoffCoordinates and LandingCoordinates (Start, Threshold, Limit: Point3D_Type's Latitude, Longitude, Altitude and
    AltitudeReference). What is left out is None."""
    def known(v):
        return None if v is None or (isinstance(v, float) and math.isnan(v)) else v

    def point(q):
        if math.isnan(q.latitude_rad):
            return None
        ref = known(q.altitude_reference)
        return {"Latitude": q.latitude_rad, "Longitude": q.longitude_rad, "Altitude": q.altitude_m,
                "AltitudeReference": None if ref is None else ALTITUDE_REFERENCE.get(int(ref))}

    def coordinates(c):
        start = point(c.start)
        return None if start is None else {"Start": start, "Threshold": point(c.threshold), "Limit": point(c.limit)}

    runways = [{"RunwayID": r.id, "Direction": known(r.direction_rad), "AvailableLength": known(r.available_length_m),
                "TakeoffCoordinates": coordinates(r.takeoff), "LandingCoordinates": coordinates(r.landing)} for r in airfield.runways]
    return {"AirfieldID": airfield.id, "Information": {"ICAO_Code": airfield.icao or None, "QNH_Setting": known(airfield.qnh_pa), "Runway": runways}}


def insufficient_endurance(endurance, capacity=None):
    """An fsim.Endurance - a rejection's ("insufficient_endurance"), or an activity's accepted over it - as A-GRA's
    MA_InsufficientEnduranceType (docs/flight-autonomy.md, 4.18): EnduranceRemaining, what the vehicle has above its
    reserve, LESS_THAN what the flight needs, EnduranceRequired. Each an EnduranceType: the fuel (kg; a battery's has
    none), the duration (s) and, given the capacity (the navigation report's: fuel kg or charge J), the percent. None
    for None."""
    if endurance is None:
        return None

    def one(amount, seconds):
        out = {"Duration": seconds}
        if endurance.energy == Energy.FUEL:
            out["Fuel"] = amount
        if capacity:
            out["Percent"] = 100.0 * amount / capacity
        return out

    return {"EnduranceRemaining": {"EnduranceRemaining": one(endurance.remaining, endurance.remaining_s), "LogicalOperator": "LESS_THAN"},
            "EnduranceRequired": one(endurance.required, endurance.required_s)}


def _timestamp(utc_s):
    """Unix seconds as A-GRA's DateTimeType (UTC, ending Z)."""
    return datetime.datetime.fromtimestamp(utc_s, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def terrain_constraint(point, now_utc_s=None):
    """An fsim.TerrainPoint - a rejection's ("terrain_conflict") - as A-GRA's TerrainConstraint, a Point4D_Type
    (docs/flight-autonomy.md, 4.19): the place (radians), the path's altitude there above the WGS-84 ellipsoid (the
    Point4D's own reference when none is named) and, given ``now_utc_s`` - the world's UTC as Unix seconds
    (World.environment's epoch_utc_seconds plus World.time) - its Timestamp: when the aircraft would be there. None for
    None."""
    if point is None:
        return None
    out = {"Latitude": point.latitude_rad, "Longitude": point.longitude_rad, "Altitude": point.altitude_msl_m}
    if now_utc_s is not None and math.isfinite(point.time_s):
        out["Timestamp"] = _timestamp(now_utc_s + point.time_s)
    return out


def elevation_request_status(latitudes_rad, longitudes_rad, heights):
    """World.terrain's heights for an ElevationRequest's points as A-GRA's ElevationRequestStatus (VI 1.2.6.9;
    docs/flight-autonomy.md, 4.19): ElevationReturned's RequestPoint list in the request's order, each a Point2D with
    its Altitude above the WGS-84 ellipsoid where the platform has the ground there, without one where it has none;
    RequestProcessingState COMPLETED, or FAILED where it has none of them."""
    points = []
    for lat, lon, h in zip(latitudes_rad, longitudes_rad, heights):
        p = {"Latitude": lat, "Longitude": lon}
        if h is not None:
            p["Altitude"] = h
        points.append(p)
    known = not points or any(h is not None for h in heights)
    return {"RequestProcessingState": "COMPLETED" if known else "FAILED", "ElevationReturned": {"RequestPoint": points}}


def air_data(state, data):
    """A vehicle's state (fsim.VehicleState) and its state data (fsim.StateData) as A-GRA's MA_AirDataType
    (docs/flight-autonomy.md, 4.20): IndicatedBaroAltitude (m), BarometricAltitudeRate (m/s), Kollsman (hPa),
    TrueAirspeed and CalibratedAirspeed (m/s), Mach, Alpha and Beta (rad)."""
    return {"IndicatedBaroAltitude": data.indicated_altitude_m, "BarometricAltitudeRate": data.indicated_altitude_rate_ms,
            "Kollsman": data.kollsman_hpa, "TrueAirspeed": state.airspeed_true_ms, "CalibratedAirspeed": state.airspeed_calibrated_ms,
            "Mach": state.mach, "Alpha": state.alpha_rad, "Beta": state.beta_rad}


def orientation_rate(data):
    """An fsim.StateData's Euler angles' rates as A-GRA's OrientationRateType (docs/flight-autonomy.md, 4.21): YawRate,
    PitchRate, RollRate (rad/s)."""
    return {"YawRate": data.yaw_rate_rad_s, "PitchRate": data.pitch_rate_rad_s, "RollRate": data.roll_rate_rad_s}


def orientation_acceleration(data):
    """And their accelerations as A-GRA's OrientationAccelerationType: YawAccel, PitchAccel, RollAccel (rad/s2)."""
    return {"YawAccel": data.yaw_acceleration_rad_s2, "PitchAccel": data.pitch_acceleration_rad_s2, "RollAccel": data.roll_acceleration_rad_s2}


def wind_data(data):
    """The wind a vehicle measures (fsim.StateData) as A-GRA's WindDataType (VI 1.2.6.8; its source, the vehicle, is
    Other): its WindChoice's WindVelocity - NorthSpeed, EastSpeed, DownSpeed (m/s)."""
    return {"WindChoice": {"WindVelocity": {"NorthSpeed": data.wind_north_ms, "EastSpeed": data.wind_east_ms, "DownSpeed": data.wind_down_ms}}}


def apply_qnh_setting(vehicle, qnh_kpa):
    """A-GRA's MA_SystemManagementRequest of VehicleSettings with a QNH_Setting (kPa; VI 1.2.6.5), applied to a vehicle
    (fsim.Vehicle.set_qnh): its status's RequestProcessingState - "COMPLETED", or "FAILED" with its
    RequestProcessingStateReason where the setting is not one an altimeter takes (850 to 1,100 hPa)."""
    try:
        vehicle.set_qnh(1000.0 * float(qnh_kpa))
    except _native.Error as e:
        return {"RequestProcessingState": "FAILED", "RequestProcessingStateReason": str(e)}
    return {"RequestProcessingState": "COMPLETED"}


def cannot_comply(reason):
    """A reason name as A-GRA's CannotComplyEnum; None for "none" and "goal_reached" (nothing went wrong)."""
    return CANNOT_COMPLY.get(reason)


def validation_result(reason):
    """A rejection's reason as A-GRA's MA_ValidationResultEnum; None where the reason is not a validation's."""
    return VALIDATION_RESULT.get(reason)


def validation_results(answer):
    """A validation (fsim.Validation) or a rejection (fsim.Rejected) as A-GRA's MA_ValidationResultEnum list,
    "select all that apply" (docs/flight-autonomy.md, 4.8): ["FLIGHT_COMMAND_VALID"] for a valid one, else
    each finding's validation result once, in order; [] where none is a validation's."""
    if getattr(answer, "valid", False):
        return ["FLIGHT_COMMAND_VALID"]
    out = []
    for reason in [f.reason for f in answer.findings] or [answer.reason]:
        result = VALIDATION_RESULT.get(reason)
        if result and result not in out:
            out.append(result)
    return out


#: constraint names -> A-GRA's RouteValidationErrorEnum, for a route's findings (route_validation_error)
_ROUTE_ERROR = {"max_orientation": "BANK_ANGLE_ERROR", "max_airspeed": "SPEED_ERROR", "min_airspeed": "SPEED_ERROR",
                "max_altitude": "ALTITUDE_ERROR", "min_altitude": "ALTITUDE_ERROR", "max_climb_rate": "ALTITUDE_ERROR",
                "max_descent_rate": "ALTITUDE_ERROR", "max_turn_rate": "TURN_ERROR"}


def route_validation_error(finding):
    """A route's finding (fsim.Finding) as A-GRA's RouteValidationErrorEnum, for its invalid segment (the
    finding's index): a fly-by turn its legs cannot hold TURN_ERROR, a bank BANK_ANGLE_ERROR, a speed
    SPEED_ERROR, an altitude or a climb ALTITUDE_ERROR, a malformed point REQUIRED_INPUT_ERROR."""
    if finding.reason == "invalid_parameter" or (finding.reason == "invalid_waypoint" and finding.constraint == "none" and finding.index < 0):
        return "REQUIRED_INPUT_ERROR"
    if finding.reason == "invalid_waypoint" and finding.constraint == "none":
        return "TURN_ERROR"
    return _ROUTE_ERROR.get(finding.constraint, "OTHER_ERROR")


def performance_constraint(constraint):
    """A constraint name (fsim.Rejected.constraint) as A-GRA's MA_PerformanceConstraintEnum; None for "none"."""
    return PERFORMANCE_CONSTRAINT.get(constraint)


def flight_capability(capability):
    """A Capability's A-GRA flight capability type (MA_FlightCapabilityEnum), or None."""
    return FLIGHT_CAPABILITY.get(capability.mode)


def flight_capabilities(vehicle):
    """What a mission autonomy would be offered: A-GRA flight capability type -> the ids of the vehicle's
    capabilities of that type (docs/vehicle-interface.md, 4.1)."""
    out = {}
    for c in vehicle.capabilities():
        kind = flight_capability(c)
        if kind:
            out.setdefault(kind, []).append(c.id)
    return out
