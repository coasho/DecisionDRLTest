"""The object model: worlds, vehicles, commands at every control level,
environment, effects, communication and scenario files (docs/sdk/world.md,
control.md, environment.md for the concepts; python.md for this API).

Per vehicle, everything is one native call. For many vehicles, use the
batched calls - ``World.states`` reads every vehicle's state into one
structured array and ``World.command`` commands every vehicle at one level
from one array - so a step costs the same few calls however many vehicles
fly.
"""
import collections
import enum
import math
import weakref

import numpy as np

from . import _native
from ._convert import as_array
from ._state import VehicleState, vehicle_state_dtype

HOLD = _native.HOLD


class Level(enum.IntEnum):
    """Control levels, lowest (direct actuators) to highest (behaviours)."""

    ACTUATOR = _native.LEVEL_ACTUATOR
    ATTITUDE = _native.LEVEL_ATTITUDE
    ACCELERATION = _native.LEVEL_ACCELERATION
    VELOCITY = _native.LEVEL_VELOCITY
    POSITION = _native.LEVEL_POSITION
    BEHAVIOR = _native.LEVEL_BEHAVIOR


#: Field order of each level's command: the columns of World.command's values.
COMMAND_FIELDS = {
    Level.ACTUATOR: ("aileron", "elevator", "rudder", "throttle", "flaps", "gear_down", "brake_left", "brake_right"),
    Level.ATTITUDE: ("roll_rad", "pitch_rad", "heading_rad", "max_bank_rad", "throttle", "airspeed_ms"),
    Level.ACCELERATION: ("load_factor_g", "roll_rate_rad_s", "longitudinal_ms2", "throttle"),
    Level.VELOCITY: ("airspeed_ms", "vertical_speed_ms", "heading_rad", "turn_rate_rad_s"),
    Level.POSITION: ("latitude_rad", "longitude_rad", "altitude_msl_m", "airspeed_ms", "capture_radius_m"),
}

#: What each field is when not given - the C++ command structs' defaults.
#: HOLD (NaN) means "keep the current value / let the controller decide".
COMMAND_DEFAULTS = {
    Level.ACTUATOR: (0.0, 0.0, 0.0, 0.0, 0.0, HOLD, 0.0, 0.0),
    Level.ATTITUDE: (0.0, 0.0, HOLD, 0.785, HOLD, HOLD),
    Level.ACCELERATION: (1.0, 0.0, HOLD, HOLD),
    Level.VELOCITY: (HOLD, 0.0, HOLD, HOLD),
    Level.POSITION: (0.0, 0.0, 0.0, HOLD, 200.0),
}

#: Field order of each level's command in the capability calls (Vehicle.submit,
#: Activity.update, World.update): COMMAND_FIELDS', then the fields a rotorcraft
#: flies with (docs/rotorcraft.md, 3.4) - a wing marks them unsupported
#: (Parameter.supported), as a rotorcraft does what it has nothing for.
SETPOINT_FIELDS = dict(COMMAND_FIELDS)
SETPOINT_FIELDS[Level.ACCELERATION] = COMMAND_FIELDS[Level.ACCELERATION] + ("pitch_rate_rad_s", "yaw_rate_rad_s")
SETPOINT_FIELDS[Level.VELOCITY] = COMMAND_FIELDS[Level.VELOCITY] + ("north_ms", "east_ms")
SETPOINT_FIELDS[Level.POSITION] = COMMAND_FIELDS[Level.POSITION] + ("heading_rad",)
SETPOINT_DEFAULTS = {level: COMMAND_DEFAULTS[level] + (HOLD,) * (len(SETPOINT_FIELDS[level]) - len(COMMAND_FIELDS[level]))
                     for level in COMMAND_FIELDS}
#: the widths of the rows only SETPOINT_FIELDS has (World.update sends those as wide as they are)
_FULL_WIDTHS = frozenset(len(f) for level, f in SETPOINT_FIELDS.items() if len(f) != len(COMMAND_FIELDS[level]))

for _level, _fields in COMMAND_FIELDS.items():
    if len(_fields) != _native.command_field_count(int(_level)):
        raise ImportError("fsim: the %s command has %d fields in the library" % (_level.name, _native.command_field_count(int(_level))))
    if len(SETPOINT_FIELDS[_level]) != _native.command_field_count_full(int(_level)):
        raise ImportError("fsim: the %s command has %d fields in the library" % (_level.name, _native.command_field_count_full(int(_level))))

Message = collections.namedtuple("Message", "sender recipient channel format time_sent time_delivered data")
Message.__doc__ = "A delivered message: who from and to, channel, format, when sent and delivered, the bytes."

BROADCAST = 0xFFFFFFFF


# --- capabilities and activities (docs/sdk/control.md, "Capabilities and activities") ---

class Source(enum.IntEnum):
    """Who commands, in rising priority: a newer command preempts an activity
    of its own or a lower source and is refused by a higher one."""

    POLICY = 0
    AUTOPILOT = 1
    OVERRIDE = 2


class RangePolicy(enum.IntEnum):
    """What happens to a value outside a capability's advertised range."""

    CLAMP = 0
    REJECT = 1
    NONE = 2


class Axis(enum.IntFlag):
    """What an activity owns: the primary axes, flown through the cascade, and
    the support axes, set directly. Above the actuators a wing's roll and yaw go
    together (LATERAL), so a flight command may own any of LATERAL, PITCH and
    THRUST apart; a rotorcraft's roll and pitch (CYCLIC), so any of CYCLIC, YAW
    and THRUST."""

    ROLL = 1
    PITCH = 2
    YAW = 4
    THRUST = 8
    FLAPS = 16
    GEAR = 32
    BRAKES = 64
    SPEEDBRAKE = 128
    PITCH_TRIM = 256
    LATERAL = ROLL | YAW
    CYCLIC = ROLL | PITCH
    PRIMARY = ROLL | PITCH | YAW | THRUST


class VehicleDefault(enum.IntEnum):
    """What flies the primary axes nobody owns."""

    NEUTRAL = 0  #: surfaces centred, throttle 0: every vehicle's default
    HOLD = 1     #: the heading, airspeed and height each had when it was let go


class ProtectionMode(enum.IntEnum):
    """Envelope protection: LIMIT (the default for an aircraft with an
    envelope) limits the demand to it and reports what crosses it; REPORT only
    reports; OFF (the default without one) does neither. It never keeps the
    aircraft inside: a crossing is reported, not prevented."""

    OFF = 0
    REPORT = 1
    LIMIT = 2


class ActivityState(enum.IntEnum):
    PENDING = 0
    ACTIVE = 1
    COMPLETED = 2
    FAILED = 3
    CANCELED = 4
    DISABLED = 5  #: live: kept, flying nothing, until enabled (docs/flight-autonomy.md, 4.10)
    DELETED = 6   #: a sticky disable: ended


class RequirementKind(enum.IntEnum):
    """What a requirement a command comes from is (A-GRA's Traceability; docs/flight-autonomy.md, 4.8)."""
    NONE = 0
    EFFECT = 1
    ACTION = 2
    TASK = 3
    COMMAND = 4


class ActivityWait(enum.IntEnum):
    """Why a pending activity has not started (docs/flight-autonomy.md, 4.9): NONE (it starts at the next world
    step, or has started), SCHEDULED (its start window has not opened), QUEUED (its axes are held by what it may not
    interrupt: ActivityInfo.waiting_for)."""
    NONE = 0
    SCHEDULED = 1
    QUEUED = 2


class ActivityBasis(enum.IntEnum):
    """What an activity's record rests on (A-GRA's ActivityBasisEnum): ACTUAL, flown; PLANNED, waiting to start."""
    ACTUAL = 0
    SENSED = 1
    PREDICTED = 2
    PLANNED = 3


class ActivityFlag(enum.IntFlag):
    """What held an activity back (ActivityInfo.constraints: its last world step's; constraints_seen: every one since it
    started): an effector it drives at its travel limit, protection reducing its demand, the state beyond a limit on its
    axes, a setpoint clamped to its range, a support axis another took; a route farther off its path than its segment's
    required navigation performance (docs/flight-autonomy.md, 4.35)."""
    SATURATED = 1
    DEMAND_LIMITED = 2
    EXCEEDED = 4
    CLAMPED = 8
    AXES_REDUCED = 16
    NAVIGATION_PERFORMANCE = 32


class Comparison(enum.IntEnum):
    """How a value is compared with the one given (A-GRA's EqualityExpressionEnum; docs/flight-autonomy.md, 4.37): the value
    under test on the left."""
    GREATER = 0
    GREATER_EQUAL = 1
    LESS = 2
    LESS_EQUAL = 3
    EQUAL = 4
    NOT_EQUAL = 5


class PathTerminator(enum.IntEnum):
    """A-GRA's civil path terminators (CivilPathTerminatorType; docs/flight-autonomy.md, 4.38): the ARINC 424 leg type of the
    leg into a waypoint (fsim.Waypoint's ``terminator``), in the schema's order - its ARINC 424 code an alias of each (TF,
    RF, ...). Flown: TRACK_TO_FIX (the great circle from the point before), INITIAL_FIX and DIRECT_TO_FIX (straight to the
    point from where the aircraft is as the leg begins), COURSE_TO_FIX (its course into the point) and RADIUS_TO_FIX (an arc
    round its centre from the point before) - their data in an fsim.RouteTerminator; the legs to an altitude or an intercept
    and a track for a distance, on the planned leg's course (FA-6f2a); the legs to a manual termination (ended by
    Activity.command_branch on a branch at the point with operator_input=1) and the holds (FA-6f2b). Refused invalid_waypoint, a
    leg its
    segment does not define (A-GRA 6.0a gives no navaid, nor a procedure turn's data): AF, CD, CR, FD, PI, VD, VR."""
    ARC_TO_FIX = 0
    COURSE_TO_ALTITUDE = 1
    COURSE_TO_DME_DISTANCE = 2
    COURSE_TO_FIX = 3
    COURSE_TO_INTERCEPT = 4
    COURSE_TO_RADIAL = 5
    DIRECT_TO_FIX = 6
    TRACK_TO_ALTITUDE = 7
    TRACK_FROM_FIX_TO_DISTANCE_ALONG_TRACK = 8
    TRACK_FROM_FIX_TO_DME_DISTANCE = 9
    FIX_TO_MANUAL_TERMINATION = 10
    HOLDING_WITH_ALTITUDE_TERMINATION = 11
    HOLDING_WITH_FIX_TERMINATION = 12
    HOLDING_WITH_MANUAL_TERMINATION = 13
    INITIAL_FIX = 14
    PROCEDURE_TURN_TO_INTERCEPT = 15
    RADIUS_TO_FIX = 16
    TRACK_TO_FIX = 17
    HEADING_TO_ALTITUDE = 18
    HEADING_TO_DME_DISTANCE_TERMINATION = 19
    HEADING_TO_INTERCEPT = 20
    HEADING_TO_MANUAL = 21
    HEADING_TO_RADIAL_TERMINATION = 22
    # ARINC 424's codes
    AF = 0
    CA = 1
    CD = 2
    CF = 3
    CI = 4
    CR = 5
    DF = 6
    FA = 7
    FC = 8
    FD = 9
    FM = 10
    HA = 11
    HF = 12
    HM = 13
    IF = 14
    PI = 15
    RF = 16
    TF = 17
    VA = 18
    VD = 19
    VI = 20
    VM = 21
    VR = 22


class PathType(enum.IntEnum):
    """What a route's path is for (A-GRA's MA_PathTypeEnum; docs/flight-autonomy.md, 4.36): a label, reported back."""
    PRIMARY = 0
    ALTERNATE = 1
    LOSS_OF_COMM = 2
    RETURN_TO_BASE = 3
    SOFT_DITCH = 4
    HARD_DITCH = 5
    INGRESS = 6
    EGRESS = 7
    TAKEOFF = 8
    LANDING = 9
    EMERGENCY_LANDING = 10
    TAXI = 11
    AIRBORNE = 12
    ARCING = 13
    BREAKING = 14
    ON_DEPARTURE_RADIAL = 15
    INITIAL_APPROACH = 16
    INTERMEDIATE_APPROACH = 17
    FINAL_APPROACH = 18
    BOLTER_WAVEOFF = 19


class EndPointKind(enum.IntEnum):
    """What an activity's end point is (A-GRA's MA_EndPointType): a point, a turn flown by or over it, a loiter."""
    WAYPOINT = 0
    TURN_POINT = 1
    LOITER_POINT = 2


class TimeCriticality(enum.IntEnum):
    """Which of a command's time windows must be met (A-GRA's SchedulingCriticalityEnum): one missed fails the
    activity (time_constraint); the others are guidance."""
    NONE = 0
    START = 1
    END = 2
    START_AND_END = 3


class TaskState(enum.IntEnum):
    """A flight task's execution state (A-GRA's RequirementExecutionStateEnum; docs/flight-autonomy.md, 4.11)."""
    AWAITING_EXECUTION = 0  #: kept, not commanded
    EXECUTION_PENDING = 1   #: commanded: its activity waits, has not flown yet, or is disabled
    EXECUTING = 2           #: its activity flies
    COMPLETED = 3           #: every run completed
    DROPPED = 4             #: its activity lost its axes or its authority
    FAILED = 5
    CANCELED = 6


class PlanCommand(enum.IntEnum):
    """A plan activation command (A-GRA's PlanActivationCommandEnum: the five the VI's route plan behaviours use;
    docs/flight-autonomy.md, 4.39)."""
    PREPARE_FOR_UPLOAD = 0      #: FA listens for the plan
    UPLOAD = 1                  #: FA keeps the plan it received
    PREPARE_FOR_ACTIVATION = 2  #: its final checks: its route checked as its NEW would be now, flying nothing
    ACTIVATE = 3                #: fly it: its route's NEW
    DEACTIVATE = 4              #: take it back, unless it executes


class PlanState(enum.IntEnum):
    """A plan's activation state (A-GRA's PlanActivationStateEnum): the states FA reaches, answering each command at
    once and asking no approval."""
    INACTIVE = 0                           #: not kept
    READY_FOR_UPLOAD = 1                   #: FA listens for it
    PREPARATION_FOR_UPLOAD_FAILED = 2      #: (a new id's, answered: not kept)
    UPLOAD_FAILED = 3
    UPLOADED = 4
    PREPARATION_FOR_ACTIVATION_FAILED = 5
    READY_FOR_ACTIVATION = 6
    ACTIVATION_FAILED = 7
    ACTIVATED = 8
    DEACTIVATED = 9


class PlanExecution(enum.IntEnum):
    """A plan's execution (A-GRA's PlanExecutionStateEnum): its activity's, since it was activated last."""
    NONE = 0        #: never activated
    PENDING = 1     #: its activity waits to start, or is disabled
    EXECUTING = 2   #: its activity flies
    COMPLETE = 3
    SUPERSEDED = 4  #: another command took its axes
    CANCELED = 5    #: canceled on request or deleted, or ended by the platform (FA's abort)
    FAILED = 6


class PointSource(enum.IntEnum):
    """Who made a route plan's point (A-GRA's PathSegmentSourceEnum)."""
    AUTO_ROUTED = 0
    OPERATOR_DEFINED = 1


class Availability(enum.IntEnum):
    """Whether a capability can be commanded now (A-GRA's CapabilityAvailabilityEnum). DISABLED means switched off
    by the platform; a capability the vehicle does not offer is UNAVAILABLE, with the reason why
    (docs/flight-autonomy.md, 4.1)."""
    AVAILABLE = 0
    TEMPORARILY_UNAVAILABLE = 1
    FAULTED = 2
    DISABLED = 3
    UNAVAILABLE = 4
    EXPENDED = 5


class Support(enum.IntEnum):
    """Whether the aircraft can do a public feature at all (docs/flight-autonomy.md, 4.1): SUPPORTED; PARTIAL (what
    is missing named); NOT_IMPLEMENTED (applicable, with the stage that builds it); NOT_SUPPORTED (a physical
    exception, with its rules and the aircraft's evidence)."""
    SUPPORTED = 0
    PARTIAL = 1
    NOT_IMPLEMENTED = 2
    NOT_SUPPORTED = 3


SupportInfo = collections.namedtuple("SupportInfo", "feature support rules stage capability missing evidence")
SupportInfo.__doc__ = ("One public feature's support on a vehicle (docs/flight-autonomy.md, 4.2): ``support`` (fsim.Support); "
                       "``rules``, the applicability rules (\"R1\" ... \"R13\") that exclude it when not supported, else those that "
                       "govern it; ``stage``, the stage that builds it when partial or not implemented (FA-n: n; 0 none); the "
                       "``capability`` that carries it; what is ``missing`` when partial; the aircraft's ``evidence`` when not "
                       "supported - its declarations and their sources.")

AvailabilityInfo = collections.namedtuple("AvailabilityInfo", "availability reason reasons description associated next_available_s ranges")
AvailabilityInfo.__doc__ = ("A capability's availability now, as a policy is answered (docs/flight-autonomy.md, 4.1 and 4.6): "
                            "``availability`` (fsim.Availability); ``reason``, the first of ``reasons`` (every reason that holds, "
                            "by name); ``description`` in words; the id it is ``associated`` with (0 none); ``next_available_s``, the "
                            "simulation time it is expected back (NaN: not known); ``ranges``, (parameter, min, max) of the "
                            "parameters a placard narrows now - a NEW outside one is refused with ``reason``.")

#: Every public feature identifier, in the order a vehicle's support table lists them (docs/flight-autonomy.md, 4.2).
SUPPORT_FEATURES = _native.support_features()


def _rules(mask):
    return tuple(_native.rule_name(r) for r in range(1, 16) if mask & (1 << r) and _native.rule_name(r))


def _reasons(mask):
    return tuple(_native.reason_name(r) for r in range(64) if mask & (1 << r))


def _reason_code(reason):
    """A reason by name ("revoked") or code, as the library numbers it."""
    if not isinstance(reason, str):
        return int(reason)
    for code in range(256):
        name = _native.reason_name(code)
        if name == "?":
            break
        if name == reason:
            return code
    raise ValueError("no reason named %r" % reason)


class ControlMode(enum.IntEnum):
    """Who may command a vehicle as a policy (docs/vehicle-interface.md, 6.1): OPEN, as ever - any capability,
    arbitrated by source and axes; GRANTED - only the capabilities a grant covers (request_control)."""
    OPEN = 0
    GRANTED = 1


Performance = collections.namedtuple(
    "Performance", "revision hovers min_cas_ms max_cas_ms max_mach max_tas_ms cruise_tas_ms max_ground_speed_ms ceiling_m max_bank_rad "
                   "min_pitch_rad max_pitch_rad max_roll_rate_rad_s min_load_factor max_load_factor max_tilt_rad max_acceleration_ms2 "
                   "max_deceleration_ms2 max_climb_ms max_descent_ms altitude_gain_per_s heading_gain heading_reference_tas_ms "
                   "bank_rate_rad_s velocity_bandwidth_rad_s")
Performance.__doc__ = ("What a vehicle can do, as its guidance plans with it (docs/vehicle-interface.md, 7.1; A-GRA's performance "
                       "profile): speeds (calibrated from the envelope, the fastest true airspeed, the cruise a mode flies given "
                       "none, a rotorcraft's fastest over the ground), the ceiling, attitude and rate limits, a rotorcraft's tilt and "
                       "accelerations, the climb and descent guidance asks for, and how fast its loops answer. NaN where the aircraft's "
                       "profile and loops say nothing; ``revision`` counts recomputations (its loops changed).")

ControlStatus = collections.namedtuple("ControlStatus", "allowed granted holder", defaults=(0,))
ControlStatus.__doc__ = ("A capability's standing with a vehicle's policy: whether it may be requested, whether a grant is held, "
                         "and the controller whose grant it is (docs/flight-autonomy.md, 4.12).")


class Rejected(_native.Error):
    """A command the vehicle refused. ``reason`` says why ("authority_held",
    "activity_ended", "invalid_parameter", "invalid_waypoint", ...); ``other``
    is the activity that holds the authority, for "authority_held". What it was
    about (docs/vehicle-interface.md, 5.1): ``index``, the field (in the
    command's order), route point or curve segment, -1 if none; ``constraint``,
    the performance limit its value broke ("max_airspeed", ... or "none");
    ``section``, a curve segment's (from, to) parameters, or None. The command
    envelope's (docs/flight-autonomy.md, 4.8): ``description``, the reason in
    words; ``associated``, an id it is about; ``command_id``, the command's;
    ``findings``, every reason it cannot be flown as asked (fsim.Finding), the
    first this one; ``adjustments``, the values it would have been flown with
    other than asked (fsim.Adjustment); ``endurance``, for
    "insufficient_endurance", what its flight needs against what the vehicle
    has (fsim.Endurance; docs/flight-autonomy.md, 4.18), else None;
    ``terrain``, for "terrain_conflict", where its path first goes below the
    ground and when (fsim.TerrainPoint; 4.19), else None."""

    def __init__(self, reason, other=0, index=-1, constraint="none", section=None, *, description="", associated=0, command_id=0,
                 findings=(), adjustments=(), suggestion=0, endurance=None, terrain=None):
        about = "" if index < 0 else " (%s %d%s)" % ("item", index, "" if constraint == "none" else ", " + constraint)
        super().__init__("command refused: %s%s" % (reason, about))
        self.reason = reason
        self.other = other
        self.index = index
        self.constraint = constraint
        self.section = section
        self.description = description
        self.associated = associated
        self.command_id = command_id
        self.findings = list(findings)
        self.adjustments = list(adjustments)
        #: a task the platform keeps with what it can fly in this command's place - every value held to the
        #: aircraft's limits (docs/flight-autonomy.md, 4.11; vehicle.command_task flies it); 0 none
        self.suggestion = suggestion
        #: what its flight needs against what the vehicle has above its reserve (fsim.Endurance), where it needs more
        self.endurance = endurance
        #: where its path first goes below the ground, and when (fsim.TerrainPoint), where it does
        self.terrain = terrain


MagneticField = collections.namedtuple("MagneticField", "north_nt east_nt down_nt horizontal_nt total_nt declination_rad inclination_rad")
MagneticField.__doc__ = ("The Earth's magnetic field at a place and date (docs/flight-autonomy.md, 4.22; the World Magnetic Model 2025 of "
                         "NOAA NCEI and the British Geological Survey): north, east and down, horizontal and total (nT), its declination "
                         "(east of true north) and inclination (below the horizontal).")


def magnetic_field(latitude_rad, longitude_rad, height_m, decimal_year):
    """The World Magnetic Model 2025's field at a geodetic place, a height above the WGS-84 ellipsoid and a decimal year
    (fsim.MagneticField; outside 2025.0 to 2030.0 carried on at its rates of change). World.magnetic_year is a world's
    date as the platform reads it."""
    return MagneticField(*_native.magnetic_field(float(latitude_rad), float(longitude_rad), float(height_m), float(decimal_year)))


def decimal_year(unix_seconds):
    """A UTC time (Unix seconds) as a decimal year."""
    return _native.decimal_year(float(unix_seconds))


Rank = collections.namedtuple("Rank", "priority precedence", defaults=(0, 0))
Rank.__doc__ = ("A command's rank (A-GRA's ComparableRankingType; docs/flight-autonomy.md, 4.9): lower first - its priority, then "
                "its precedence within it. Rank(0, 0), every command's without one, ranks first.")

TimeWindow = collections.namedtuple("TimeWindow", "start_not_before start_not_after end_not_before end_not_after criticality",
                                    defaults=(math.nan, math.nan, math.nan, math.nan, TimeCriticality.NONE))
TimeWindow.__doc__ = ("When a command may start and should end (A-GRA's TemporalConstraints; docs/flight-autonomy.md, 4.9), in "
                      "simulation seconds (World.time); NaN, no bound. It starts no earlier than ``start_not_before``; "
                      "``criticality`` (fsim.TimeCriticality or its name) says which bounds must be met - a start window closed "
                      "while it waits, an end missed: failed, \"time_constraint\". A persistent activity (a hold, a level) is done "
                      "when its end window closes.")

ActivityInfo = collections.namedtuple(
    "ActivityInfo", "id vehicle capability source axes state reason by constraints constraints_seen start_time end_time "
    "command_id interactive trace waiting basis rank precedence waiting_for interrupt window suggestion run runs controller")
ActivityInfo.__doc__ = ("An activity's record: its capability (an index into Vehicle.capabilities()), who commanded it, "
                        "its state and why it ended, flags (1 saturated, 8 clamped, ...) and when it ran; the command it came "
                        "from (docs/flight-autonomy.md, 4.8): its id, whether it takes activity commands, and the requirements "
                        "it traces to, as (fsim.RequirementKind, id) pairs; how it is arbitrated and scheduled (4.9): why it waits "
                        "to start (fsim.ActivityWait), what its record rests on (fsim.ActivityBasis: planned while it waits), its "
                        "fsim.Rank, its capability's precedence, what it waits for (queued), whether its command interrupts, and "
                        "its fsim.TimeWindow; the task the platform suggests in its place, had it failed as it would start "
                        "(4.11), and a task's runs: the run flying, of how many (0, 0 none); the policy's controller its "
                        "command came from (4.12).")

TaskStatus = collections.namedtuple(
    "TaskStatus", "id state reason suggested activity run runs percent start_time end_time command_id")
TaskStatus.__doc__ = ("A flight task's status (A-GRA's TaskStatus; docs/flight-autonomy.md, 4.11): its fsim.TaskState and why it "
                      "failed, was dropped or canceled; whether it is the platform's suggestion; its activity (every run's), the "
                      "run flying or flown last of how many, the percent of the whole done, when it was commanded and ended, "
                      "its task command's id.")

PlanStatus = collections.namedtuple(
    "PlanStatus", "id state version revision execution reason for_planning_use_only activity percent start_time end_time command_id fa_owned")
PlanStatus.__doc__ = ("A route plan's status (A-GRA's plan activation status and route plan execution status; "
                      "docs/flight-autonomy.md, 4.39): its fsim.PlanState; the version kept (0 before an upload) and its "
                      "``revision``, its uploads kept; its fsim.PlanExecution, its activity's since it was activated last; why "
                      "its last command failed or why its execution ended; whether it is for planning use only; its activity "
                      "(0 before it was activated), the percent of its route flown, when it was activated and its activity "
                      "ended, and its activation's command id; ``fa_owned``, FA's own - loaded by the platform, read only to "
                      "MA (4.40).")

PlanValidationResult = collections.namedtuple("PlanValidationResult", "valid validation index")
PlanValidationResult.__doc__ = ("A route plan's validation answer (A-GRA's RoutePlanValidation; docs/flight-autonomy.md, 4.41): "
                                "``valid`` - A-GRA's VALID, over a patch's parts where they are given - and the route's "
                                "``validation`` as it answered (a fsim.Validation: its reason, findings and adjustments; "
                                "fsim.agra.route_plan_validation gives A-GRA's form), and the ``index`` of the waypoint a "
                                "refusal names (-1 none).")

PlanCommandResult = collections.namedtuple("PlanCommandResult", "plan command completed state reason activity index findings")
PlanCommandResult.__doc__ = ("A plan command's answer (A-GRA's MA_MissionPlanActivationCommandStatus; docs/flight-autonomy.md, "
                             "4.39): ``completed`` (A-GRA's COMPLETED; else FAILED), the plan's fsim.PlanState after it (INACTIVE: "
                             "not kept) and why it failed; an activation's fsim.Activity (None otherwise); the waypoint a failed "
                             "preparation for activation or activation names (-1 none), and its findings.")

Finding = collections.namedtuple("Finding", "reason index constraint section associated description")
Finding.__doc__ = ("One reason a command cannot be flown as asked (A-GRA's ValidationResult; docs/flight-autonomy.md, 4.8): the "
                   "reason, the field, route point or curve segment (-1 none), the performance limit it breaks, a curve "
                   "segment's section (from, to) or None, an id it is about, and the reason in words.")

Adjustment = collections.namedtuple("Adjustment", "index field constraint requested adjusted")
Adjustment.__doc__ = ("A value a command is flown with other than asked, held to what the aircraft can do: the command's field, a "
                      "route point or curve segment; a route point's field (fsim.Waypoint's order) or -1; the limit; what was "
                      "asked and what is flown (NaN where it is not one number: a fly-by turn flown smaller).")

Validation = collections.namedtuple("Validation", "valid reason clamped command_id findings adjustments deferred endurance terrain",
                                    defaults=(False, None, None))
Validation.__doc__ = ("A validation's answer (validate_only=True; A-GRA's FLIGHT_COMMAND_VALID): whether a NEW would be accepted, "
                      "why not, whether a value would be clamped, the command's id, every finding and every value it would be "
                      "flown with other than asked, whether it would wait to start (docs/flight-autonomy.md, 4.9), where its "
                      "flight needs more than the vehicle has, by how much (fsim.Endurance, 4.18), and where its path goes below "
                      "the ground (fsim.TerrainPoint, 4.19). Nothing flies.")

Endurance = collections.namedtuple("Endurance", "energy remaining required remaining_s required_s")
Endurance.__doc__ = ("What a flight with an end needs against what the vehicle has above its reserve (docs/flight-autonomy.md, 4.18; "
                     "A-GRA's MA_InsufficientEnduranceType): its ``energy`` (fsim.Energy: FUEL in kg, BATTERY in J), what the vehicle "
                     "has ``remaining`` and what the flight ``required`` - flown level at each leg's speed and altitude - and how long "
                     "each lasts: ``remaining_s`` at what it consumes now, ``required_s`` the flight's. Given where the flight needs "
                     "more: refused \"insufficient_endurance\", or accepted over it with override_rejection.")

TerrainPoint = collections.namedtuple("TerrainPoint", "latitude_rad longitude_rad altitude_msl_m ground_m time_s index")
TerrainPoint.__doc__ = ("Where a commanded path first goes below the ground (docs/flight-autonomy.md, 4.19; A-GRA's TerrainConstraint, "
                        "a Point4D): the place, the path's altitude and the ground's there - both above the WGS-84 ellipsoid - "
                        "when it would be there (seconds from the command, at its planned speeds), and the route point it flies "
                        "to there or the curve segment (-1 otherwise). Given where refused \"terrain_conflict\", which nothing "
                        "overrides.")

#: The envelope's limits, in the platform's order: "load_factor_max", "alpha_max", "cas_min", ...
LIMITS = tuple(_native.limit_name(i) for i in range(10))
LimitStatus = collections.namedtuple("LimitStatus", "limited_updates exceeded_updates exceeded_s worst_excess")
LimitStatus.__doc__ = ("One limit since the last read: the control updates in which the demand was limited for it, those "
                       "in which the state was beyond it, for how long (s) and by how much at most (g, rad, rad/s, m/s "
                       "calibrated or Mach).")
Envelope = collections.namedtuple("Envelope", "mode limits")
Envelope.__doc__ = "What envelope protection saw: its ProtectionMode, and a LimitStatus by limit name (fsim.LIMITS)."

Parameter = collections.namedtuple("Parameter", "name unit min max default optional supported", defaults=(True,))
Parameter.__doc__ = ("A field of a capability's command, or a behaviour's parameter: its range for this aircraft, its "
                     "default, whether it takes HOLD, and whether the aircraft has anything it moves (not ``supported``: "
                     "a command that sets it other than to HOLD or its default is refused, invalid_parameter).")
Capability = collections.namedtuple(
    "Capability", "id version kind interactions level axes terminating needs_target behavior parameters axis_groups mode accepted superseded",
    defaults=("none", 0, ""))
Capability.__doc__ = ("What a vehicle offers: e.g. fsim.flight.attitude (a level) or fsim.guidance.hold (a behaviour). "
                      "``axis_groups``: what it may own apart - 1 lateral (roll and yaw), 2 pitch, 4 thrust, 8 any primary "
                      "axis, 16 cyclic (roll and pitch), 32 yaw; 0 all of its axes or none. A wing's flight capabilities "
                      "own 1, 2 and 4 apart, a rotorcraft's 16, 32 and 4. ``mode``: the A-GRA flight capability type it is "
                      "(\"hsa_csa\", \"waypoint_following\", \"loiter\", \"formation\", ... or \"none\"; fsim.agra). "
                      "``accepted``: how it is controlled (A-GRA's AcceptedInterface) - 1 a command starts it, 2 its activities "
                      "change or end on command, 4 a flight task, 8 it acts on its own. ``superseded``: the A-GRA capability "
                      "that does a platform behaviour's job better (\"fsim.guidance.hold\": \"fsim.guidance.hsa\"), else \"\".")

ActivityProgress = collections.namedtuple(
    "ActivityProgress", "segment segments laps segment_id percent segment_percent distance_to_go_m time_to_go_s cross_track_m "
    "course_rad heading_rad altitude_msl_m speed_ms speed_reference arrival_s arrival_delta_s")
ActivityProgress.__doc__ = ("How far an activity has got and what it commands, as its behaviour reported it after the last "
                            "world step (docs/vehicle-interface.md, 5.3): the waypoint, curve segment or pattern leg flown now "
                            "(of ``segments``; 0: nothing segmented), laps, percent of the whole and of the segment, the "
                            "distance and time to the end, the cross-track distance (+ right of the path), and the course, "
                            "heading, altitude and speed it asks for (``speed_reference``: 0 true airspeed, 1 calibrated, 2 "
                            "ground speed, 3 Mach); a route's next point with an arrival window (docs/flight-autonomy.md, "
                            "4.33): ``arrival_s``, when it is estimated to arrive there (World.time's clock), and "
                            "``arrival_delta_s``, that against its window (+ late, - early, 0 within). NaN where it says nothing.")

class Energy(enum.IntEnum):
    """What a vehicle flies on (NavigationReport.energy)."""
    UNKNOWN = 0  # its flight model tells of neither
    FUEL = 1
    BATTERY = 2


class Contingency(enum.IntEnum):
    """A-GRA's SystemContingencyLevelEnum: the platform reports a low fuel state (FLIGHT_CRITICAL: at or below the
    reserve, or an engine starved); it models no subsystem failures and no communications."""
    NORMAL = 0
    MISSION_CRITICAL = 1
    FLIGHT_CRITICAL = 2
    LOST_COMMS = 3


NavigationReport = collections.namedtuple(
    "NavigationReport", "energy fuel_kg remaining capacity percent consumption endurance_s reserve playtime_s return_distance_m "
    "return_tas_ms return_consumption contingency starved")
NavigationReport.__doc__ = (
    "A-GRA's navigation report (docs/flight-autonomy.md, 4.14): what the vehicle flies on (fsim.Energy), its fuel (kg; 0 for a "
    "battery), what is left and its capacity (fuel in kg, a battery's charge in J), the percent left, its consumption now "
    "(the engines' fuel flow, kg/s, or the power the battery gives, W), its endurance at that (s; infinite while it consumes "
    "nothing, 0 with nothing left), the reserve it keeps, its playtime to its recovery point (s: what is left less the reserve and the return; NaN "
    "without a recovery point), the return's distance, speed and consumption, its contingency (fsim.Contingency) and whether its "
    "engines have nothing left - a fuel burner's tanks empty, or the battery spent.")

ProfilePoint = collections.namedtuple("ProfilePoint", "value tas_ms altitude_msl_m weight_kg")
ProfilePoint.__doc__ = ("A value at a true airspeed, an altitude (above sea level) and a weight: an airspeed or a climb or descent rate "
                        "(m/s), or a burn (kg/s of fuel, W of a battery); NaN where it does not depend on one.")
ProfileAcceleration = collections.namedtuple("ProfileAcceleration", "x_ms2 y_ms2 z_ms2 mach tas_ms altitude_msl_m weight_kg")
ProfileAcceleration.__doc__ = ("Acceleration limits in body axes (x forward, y right, z down) as the specific force the aircraft can make "
                               "(1 g of lift is -9.81 in z), at a Mach number, airspeed, altitude and weight; NaN where none.")
ProfileExcessPower = collections.namedtuple("ProfileExcessPower", "climb_ms acceleration_ms2 tas_ms altitude_msl_m weight_kg")
ProfileExcessPower.__doc__ = "Specific excess power at full power: the climb holding the speed, the acceleration holding the height."
ProfileOrientation = collections.namedtuple("ProfileOrientation", "yaw_rad pitch_rad pitch_min_rad roll_rad tas_ms altitude_msl_m weight_kg")
ProfileOrientation.__doc__ = "Attitude limits: the most nose-up pitch, the most nose-down (pitch_min_rad), the bank; NaN: none (the yaw)."
ProfileRates = collections.namedtuple("ProfileRates", "roll_rad_s pitch_rad_s yaw_rad_s tas_ms")
ProfileRates.__doc__ = "Attitude rate limits at an airspeed, body axes; NaN: none known."
PerformanceProfile = collections.namedtuple(
    "PerformanceProfile", "mode energy clean flaps_out gear_down time_s altitude_msl_m weight_kg tas_ms min_altitude_msl_m max_altitude_msl_m "
    "max_turn_rate_rad_s max_climb_rate_ms min_airspeed max_airspeed best_endurance_airspeed best_range_airspeed min_acceleration "
    "max_acceleration max_deceleration excess_power max_descent_rate burn max_orientation max_orientation_rate")
PerformanceProfile.__doc__ = (
    "A flight mode's performance profile (docs/flight-autonomy.md, 4.15; A-GRA's MA_FlightControlModesPerformanceProfileType) at "
    "the vehicle's condition when asked: the mode's name; what `burn` is (fsim.Energy); whether it was clean (flaps and gear up: "
    "only then are the tables' values - flown clean - in); the condition (time, altitude, weight, true airspeed); the least and "
    "most altitude; the fastest turn and climb guidance flies; and lists: the airspeeds (fsim.ProfilePoint, true) against "
    "altitude at the weight now; the accelerations (fsim.ProfileAcceleration), excess power (fsim.ProfileExcessPower), idle "
    "descents, decelerations and burn against airspeed and altitude; the attitude (fsim.ProfileOrientation) and rate "
    "(fsim.ProfileRates) limits at the condition now. fsim.agra.performance_profile gives it in A-GRA's names.")

#: flight mode names (fsim.Capability.mode) -> their codes
_FLIGHT_MODES = {}
for _code in range(64):
    _name = _native.flight_mode_name(_code)
    if _name == "?":
        break
    _FLIGHT_MODES[_name] = _code

NavigationSettings = collections.namedtuple("NavigationSettings", "recovery latitude_deg longitude_deg altitude_msl_m reserve_fraction")
StateData = collections.namedtuple("StateData", "indicated_altitude_m indicated_altitude_rate_ms kollsman_hpa static_pressure_pa "
                                   "static_temperature_k yaw_rate_rad_s pitch_rate_rad_s roll_rate_rad_s yaw_acceleration_rad_s2 "
                                   "pitch_acceleration_rad_s2 roll_acceleration_rad_s2 wander_angle_rad wind_north_ms wind_east_ms "
                                   "wind_down_ms magnetic_heading_rad declination_rad")
StateData.__doc__ = ("What A-GRA's detailed position report carries beyond the state (docs/flight-autonomy.md, 4.20 and 4.21): what "
                     "the vehicle's barometric altimeter reads - its IndicatedBaroAltitude, the standard atmosphere's height of the "
                     "static pressure above the QNH it is set to, and its rate - its Kollsman setting (hPa), and the air's static "
                     "pressure and temperature where the vehicle is (MA_AirDataType); how fast its Euler angles change and how that "
                     "changes (OrientationRate, OrientationAcceleration; NaN pitched straight up or down); its WanderAngle (0: its "
                     "navigation frame is north's); the wind where it is, the air's velocity over the ground (north, east, down); its "
                     "MagneticHeading and the declination that turns it true (4.22).")
NavigationSettings.__doc__ = "Where the vehicle recovers to (if ``recovery``) and the fraction of its capacity it keeps for the end."

CommandedState = collections.namedtuple(
    "CommandedState", "top_level latitude_rad longitude_rad altitude_msl_m heading_rad turn_rate_rad_s airspeed_ms "
    "vertical_speed_ms north_ms east_ms roll_rad pitch_rad load_factor_g roll_rate_rad_s pitch_rate_rad_s yaw_rate_rad_s throttle "
    "north_acceleration_ms2 east_acceleration_ms2 down_acceleration_ms2 altitude_m altitude_reference")
CommandedState.__doc__ = ("What the vehicle is commanded (A-GRA's VehicleCommandState): what the cascade asked for in its last "
                          "control update, level by level - the position level's point, the velocity level's heading, turn "
                          "rate, airspeed, vertical speed and velocity over the ground, the attitude level's roll and pitch, "
                          "the acceleration level's load factor and rates, the throttle - then the acceleration it commands, "
                          "north, east and down (a wing's: its longitudinal acceleration and load factor at the attitude it "
                          "flies, over the Earth), and the altitude as its mode commanded it, in its reference (fsim."
                          "AltitudeReference; docs/flight-autonomy.md, 4.12). NaN where none is commanded.")

EndPoint = collections.namedtuple("EndPoint", "kind latitude_rad longitude_rad altitude_m altitude_reference turn id index")
EndPoint.__doc__ = ("Where an activity flies to (A-GRA's ActualEndPoint; docs/flight-autonomy.md, 4.12): its kind "
                    "(fsim.EndPointKind), the point, its altitude in ``altitude_reference`` (fsim.AltitudeReference; a "
                    "waiting route's as given, NaN continuing the point before's), a turn point's ``turn`` (fsim.TurnType; "
                    "NaN otherwise), a route waypoint's ``id`` (0 none) and its waypoint's or curve segment's ``index`` (0 a "
                    "pattern's or the position level's point).")


def _info(t):
    return ActivityInfo(t[0], t[1], t[2], Source(t[3]), t[4], ActivityState(t[5]), _native.reason_name(t[6]), t[7], t[8], t[9],
                        t[10], t[11], t[12], t[13], tuple((RequirementKind(k), i) for k, i in t[14]), ActivityWait(t[15]),
                        ActivityBasis(t[16]), Rank(*t[17]), t[18], t[19], bool(t[20]), TimeWindow(*t[21][:4], TimeCriticality(t[21][4])),
                        t[22], t[23], t[24], t[25])


def _task(t):
    return TaskStatus(t[0], TaskState(t[1]), _native.reason_name(t[2]), bool(t[3]), t[4], t[5], t[6], t[7], t[8], t[9], t[10])


def _airfield(t):
    def place(v, k):
        return RunwayPoint(*v[k:k + 4])

    runways = tuple(Runway(int(r[0]), r[1], r[2], RunwayCoordinates(place(r, 3), place(r, 7), place(r, 11)),
                           RunwayCoordinates(place(r, 15), place(r, 19), place(r, 23))) for r in t[4])
    return Airfield(t[0], t[1], t[2], runways, t[3])


def _plan_status(t):
    return PlanStatus(t[0], PlanState(t[1]), t[2], t[3], PlanExecution(t[4]), _native.reason_name(t[5]), bool(t[6]), t[7], t[8], t[9],
                      t[10], t[11], bool(t[12]))


def _findings(result, h):
    """The last command's findings and adjustments (docs/flight-autonomy.md, 4.8), read only where there are some."""
    findings = [Finding(_native.reason_name(f[0]), f[1], _native.constraint_name(f[2]), None if math.isnan(f[3]) else (f[3], f[4]), f[5], f[6])
                for f in h.last_findings()] if h is not None and result[13] else []
    adjustments = [Adjustment(*a[:2], _native.constraint_name(a[2]), a[3], a[4]) for a in h.last_adjustments()] if h is not None and result[14] else []
    return findings, adjustments


def _endurance(h):
    """The last command's endurance (docs/flight-autonomy.md, 4.18) as an fsim.Endurance, or None where none was short."""
    e = h.last_endurance() if h is not None else None
    return None if e is None else Endurance(Energy(e[0]), *e[1:])


def _terrain(h):
    """The last command's terrain point (docs/flight-autonomy.md, 4.19) as an fsim.TerrainPoint, or None where its path clears it."""
    t = h.last_terrain() if h is not None else None
    return None if t is None else TerrainPoint(*t[1:], t[0])


def _rejected(result, h=None):
    """A refused command's result tuple as the Rejected it raises."""
    section = None if math.isnan(result[7]) else (result[7], result[8])
    findings, adjustments = _findings(result, h)
    reason = _native.reason_name(result[1])
    endurance = _endurance(h) if reason == "insufficient_endurance" else None
    terrain = _terrain(h) if reason == "terrain_conflict" else None
    return Rejected(reason, result[3], result[5], _native.constraint_name(result[6]), section, description=result[12],
                    associated=result[11], command_id=result[10], findings=findings, adjustments=adjustments, suggestion=result[16],
                    endurance=endurance, terrain=terrain)


def _checked(result, h=None):
    """A result tuple (status, reason, activity, other, clamped, index, constraint, from, to, new_activity,
    command_id, associated, description, finding_count, adjustment_count): raise Rejected unless accepted,
    canceled or valid. ``h``, the world's handle, reads the findings."""
    if result[0] == 1:
        raise _rejected(result, h)
    return result


def _validation(result, h):
    """A validation's answer (validate_only)."""
    findings, adjustments = _findings(result, h)
    reason = _native.reason_name(result[1])
    short = reason == "insufficient_endurance" or result[17]
    return Validation(result[0] == 3, reason, bool(result[4]), result[10], findings, adjustments, bool(result[15]),
                      _endurance(h) if short else None, _terrain(h) if reason == "terrain_conflict" else None)


def _envelope(command_id, trace, interactive, validate_only, rank=None, interrupt=True, precedence_override=None, window=None,
              override_rejection=False, controller=0):
    """The command envelope's native form (docs/flight-autonomy.md, 4.8, 4.9 and 4.12), or None where it is all left out."""
    scheduled = rank is not None or not interrupt or precedence_override is not None or window is not None or override_rejection or controller
    if not command_id and not trace and interactive and not validate_only and not scheduled:
        return None
    pairs = []
    for kind, rid in trace:
        kind = RequirementKind[kind.upper()] if isinstance(kind, str) else RequirementKind(kind)
        pairs.append((int(kind), int(rid)))
    envelope = (int(command_id), tuple(pairs), bool(interactive), bool(validate_only))
    if not scheduled:
        return envelope
    rank = Rank(*rank) if rank is not None else Rank()
    if window is None:
        window = TimeWindow()
    elif isinstance(window, dict):
        window = TimeWindow(**window)
    criticality = window.criticality
    criticality = TimeCriticality[criticality.upper()] if isinstance(criticality, str) else TimeCriticality(criticality)
    bounds = tuple(math.nan if b is None else float(b) for b in window[:4])
    return envelope + (bool(interrupt), bool(override_rejection), (int(rank.priority), int(rank.precedence)),
                       None if precedence_override is None else int(precedence_override), bounds, int(criticality), int(controller))


#: What is set directly beside the cascade (docs/sdk/control.md): the support
#: effectors and the engines' throttles, in the platform's order, each
#: command's fields and what they are when not given (HOLD keeps an engine's throttle).
SUPPORT_KINDS = ("gear", "flaps", "wheel_brakes", "speedbrake", "pitch_trim", "engines")
SUPPORT_FIELDS = {"gear": ("down",), "flaps": ("position",), "wheel_brakes": ("left", "right"), "speedbrake": ("position",),
                  "pitch_trim": ("position",), "engines": ("throttle_1", "throttle_2", "throttle_3", "throttle_4")}
SUPPORT_DEFAULTS = {"gear": (1.0,), "flaps": (0.0,), "wheel_brakes": (0.0, 0.0), "speedbrake": (0.0,), "pitch_trim": (0.0,),
                    "engines": (HOLD, HOLD, HOLD, HOLD)}


class SpeedReference(enum.IntEnum):
    """What a mode's speed is measured against (A-GRA's SpeedReferenceEnum and MachType)."""
    TRUE_AIRSPEED = 0
    CALIBRATED_AIRSPEED = 1
    GROUND_SPEED = 2
    MACH = 3


class AltitudeReference(enum.IntEnum):
    """What a mode's altitude is measured from (A-GRA's AltitudeReferenceEnum). The simulation's sea level is the
    WGS-84 ellipsoid, so MSL and ELLIPSOID are one; ABOVE_GROUND follows the terrain under the aircraft; BAROMETRIC is
    what the vehicle's altimeter reads, set to its QNH (Vehicle.set_qnh; docs/flight-autonomy.md, 4.20): an isobar,
    flown as the air and the setting move it - an hsa's or a pattern's (a route's is FA-6's)."""
    MSL = 0
    ABOVE_GROUND = 1
    ELLIPSOID = 2
    BAROMETRIC = 3


class FrameOrigin(enum.IntEnum):
    """A reference frame's origin (A-GRA's ReferenceFrameOriginChoiceType; docs/flight-autonomy.md, 4.21): a place; a place
    moving at a constant velocity from a time; a world vehicle, followed."""
    FIXED = 0
    MOVING = 1
    VEHICLE = 2


class FrameRotation(enum.IntEnum):
    """How a point's offsets in a frame are turned (A-GRA's RotationEnum): x north, y east, z down; by the origin's yaw; by
    its body's axes (x forward, y right, z down, as it is pitched and rolled too); by its track over the ground."""
    UNROTATED = 0
    YAW = 1
    ATTITUDE = 2
    HEADING = 3


class FrameOffsets(enum.IntEnum):
    """How they are laid out on the Earth (A-GRA's OffsetXY_Enum): in the plane square to the vertical at the origin; along
    the great circle their way; along the rhumb line."""
    CARTESIAN = 0
    GREAT_CIRCLE = 1
    RHUMB = 2


class CurveZ(enum.IntEnum):
    """How a curve's control points' third value reads (A-GRA's Z_ChoiceType; docs/flight-autonomy.md, 4.27): metres
    down from its reference; metres up from it; the altitude itself, in the curve's altitude reference."""
    DOWN = 0
    ALTITUDE_OFFSET = 1
    ABSOLUTE_ALTITUDE = 2


class SpeedOptimization(enum.IntEnum):
    """The speed a mode varies by itself (A-GRA's SpeedOptimizationEnum; docs/flight-autonomy.md, 4.17): the
    performance tables' best-range speed (the most distance for the fuel) or best-endurance speed (the most time),
    at the altitude and the weight now, flown as a true airspeed."""
    LONG_RANGE_CRUISE = 0
    MAX_ENDURANCE = 1


class DirectionReference(enum.IntEnum):
    """What an hsa's heading or course is measured from (A-GRA's MA_HeadingReferenceEnum; docs/flight-autonomy.md, 4.22):
    true north, or magnetic north - flown turned by the World Magnetic Model's declination where the aircraft is, at
    the world's date."""
    TRUE_NORTH = 0
    MAGNETIC_NORTH = 1


class WaypointType(enum.IntEnum):
    """What a waypoint is for (A-GRA's WaypointTypeEnum; docs/flight-autonomy.md, 4.29): nav only and passive are flown, the
    end of a path at its path's last point (4.36: a route without paths, its last; elsewhere refused "invalid_waypoint"); the
    others not yet (refused "not_implemented", naming the point)."""
    NAV_ONLY = 0
    TAXI = 1
    RUNWAY_START = 2
    RUNWAY_THRESHOLD = 3
    RUNWAY_LIMIT = 4
    APPROACH = 5
    APPROACH_INITIAL_POINT = 6
    APPROACH_FINAL_POINT = 7
    TAKEOFF = 8
    TAKEOFF_INITIAL_POINT = 9
    TAKEOFF_FINAL_POINT = 10
    TOUCHDOWN = 11
    PASSIVE = 12
    HARD_DITCH = 13
    END_OF_PATH = 14


class TurnType(enum.IntEnum):
    """How a route passes a waypoint (A-GRA's TurnType): a fly-by turn begins before it, on a circle tangent to
    both legs; a fly-over point is passed, then the next leg intercepted. A-GRA's others (docs/flight-autonomy.md, 4.30): a
    capture flies over the point and captures its course, the next point along it; a start begins an arc to the next point
    (ARINC 424's radius to fix), tangent to its course there; an end ends it."""
    FLY_BY = 0
    FLY_OVER = 1
    CAPTURE_OUTBOUND_COURSE = 2
    START_TURN = 3
    END_TURN = 4


class Projection(enum.IntEnum):
    """What a route's legs are on the Earth."""
    GREAT_CIRCLE = 0
    RHUMB = 1


class EndBehavior(enum.IntEnum):
    """What a route or curve does after its end: on along the last leg or course (its altitude and speed), or
    loiter there - a wing orbits the point; after a route a rotorcraft stops and hovers over it, after a curve it circles
    it too (A-GRA's CIRCULAR_LOITER)."""
    CONTINUE = 0
    LOITER = 1


class PatternKind(enum.IntEnum):
    """A loiter pattern (A-GRA's LOITER): an orbit round its centre; a racetrack, two half circles joined by legs,
    the inbound one ending at the fix; a figure-eight, two circles meeting at the centre; ATC's hold, a racetrack
    on the fix with a minute's legs, entered direct to the fix; a rotorcraft's hover over its point, its duration from
    its arrival (docs/flight-autonomy.md, 4.25)."""
    ORBIT = 0
    RACETRACK = 1
    FIGURE_EIGHT = 2
    HOLD = 3
    HOVER = 4


class HoldTurn(enum.IntEnum):
    """A hold's turns by type (A-GRA's MA_HoldTurnTypeEnum; docs/flight-autonomy.md, 4.24): rate one at most 25
    degrees of bank (a hold's default); the tightest the pattern flies (80 % of the aircraft's bank); half rate one at
    most 15 degrees."""
    STANDARD = 0
    MIL_POWER = 1
    RELAX = 2


class HoldEntry(enum.IntEnum):
    """How a racetrack or a hold is entered (A-GRA's MA_HoldEntryTypeEnum, and ATC's; docs/flight-autonomy.md, 4.24):
    where it is nearest; at the fix by ATC's entry for the side the aircraft comes from; along the inbound or the
    outbound leg; ATC's parallel or teardrop entry. Left out: direct to the fix, then round."""
    DIRECT = 0
    ANCHOR = 1
    INBOUND = 2
    OUTBOUND = 3
    PARALLEL = 4
    TEARDROP = 5


class ClimbOptimization(enum.IntEnum):
    """How a route's segment climbs or descends, its rate chosen by the aircraft (A-GRA's ClimbOptimizationEnum;
    docs/flight-autonomy.md, 4.32)."""
    BEST_RATE = 0
    EXTENDED_RANGE = 1


class HoldContext(enum.IntEnum):
    """A hold's operational context (A-GRA's MA_HoldContextEnum): the defaults it implies are ATC's for every one."""
    ADMIN = 0
    TACTICAL = 1
    ATC = 2


class MustFlyLocation(enum.IntEnum):
    """Where a must fly goes (A-GRA's MustFlyLocationType; docs/flight-autonomy.md, 4.42): a point, another vehicle, an
    operational point by its id (World.set_op_point)."""
    POINT = 0
    ENTITY = 1
    OP_POINT = 2


#: The Vehicle Interface's modes (docs/vehicle-interface.md): their fixed-size setpoints' fields, in order. HOLD leaves
#: one out: a NEW continues what a live hsa commanded (else what the aircraft flies now) and takes a route's, a
#: pattern's or a curve's default; an UPDATE keeps it.
MODE_KINDS = ("hsa", "route", "pattern", "curve", "must_fly")
MODE_FIELDS = {"hsa": ("heading_rad", "course_rad", "speed", "speed_reference", "altitude_m", "altitude_reference", "speed_optimization",
                       "direction_reference"),
               "route": ("projection", "repeat", "end", "start"),
               "pattern": ("pattern", "latitude_rad", "longitude_rad", "altitude_m", "altitude_reference", "radius_m", "clockwise",
                           "course_rad", "leg_m", "speed", "speed_reference", "duration_s", "speed_optimization", "direction_reference",
                           "heading_rad", "leg_s", "bank_rad", "orbits", "latitude2_rad", "longitude2_rad", "radius2_m", "entry_latitude_rad",
                           "entry_longitude_rad", "exit_latitude_rad", "exit_longitude_rad", "turn_rate_rad_s", "turn_type", "hold_entry",
                           "hold_context", "frame", "frame_rotation", "frame_offsets", "frame_x_m", "frame_y_m", "frame_z_m"),
               "curve": ("latitude_rad", "longitude_rad", "altitude_m", "speed_min_ms", "speed_max_ms", "duration_s", "end", "append",
                         "altitude_reference", "altitude_min_m", "altitude_max_m", "point_rotation", "point_offsets", "point_z", "frame",
                         "frame_rotation", "frame_offsets", "frame_x_m", "frame_y_m", "frame_z_m"),
               "must_fly": ("location", "latitude_rad", "longitude_rad", "altitude_m", "altitude_reference", "target", "ingress_min_rad",
                            "ingress_max_rad", "speed", "speed_reference")}
MODE_DEFAULTS = {"hsa": (HOLD,) * 8, "route": (HOLD,) * 4, "pattern": (HOLD,) * 35, "curve": (HOLD,) * 20, "must_fly": (HOLD,) * 10}
_REFERENCES = {"speed_reference": SpeedReference, "altitude_reference": AltitudeReference, "speed_optimization": SpeedOptimization,
               "direction_reference": DirectionReference,
               "projection": Projection, "end": EndBehavior,
               "turn": TurnType, "kind": EndPointKind, "waypoint_type": WaypointType, "pattern": PatternKind, "turn_type": HoldTurn, "hold_entry": HoldEntry, "hold_context": HoldContext,
               "climb_optimization": ClimbOptimization,
               "frame_rotation": FrameRotation, "frame_offsets": FrameOffsets, "point_rotation": FrameRotation, "point_offsets": FrameOffsets,
               "point_z": CurveZ, "terminator": PathTerminator, "location": MustFlyLocation}

Waypoint = collections.namedtuple(
    "Waypoint", "latitude_rad longitude_rad altitude_m altitude_reference speed speed_reference turn max_bank_rad climb_rate_ms id "
                "altitude_min_m altitude_max_m kind waypoint_type frame frame_rotation frame_offsets frame_x_m frame_y_m frame_z_m "
                "course_rad turn_radius_m speed_optimization climb_optimization acceleration_ms2 arrival_begin_s arrival_end_s rnp_m next "
                "terminator",
    defaults=(HOLD, HOLD, HOLD, HOLD, 0, HOLD, HOLD, 0) + (HOLD,) * 20)
Waypoint.__doc__ = ("One waypoint of a route (A-GRA's), and the segment that ends at it: reached at ``altitude_m`` above "
                    "``altitude_reference`` along a straight profile (or climbing at ``climb_rate_ms``, then level), flown at "
                    "``speed`` in ``speed_reference``, passed by ``turn`` (fsim.TurnType: 'fly_by', 'fly_over') with "
                    "``max_bank_rad`` for its turn; ``id`` comes back in the progress. HOLD (the default) continues the "
                    "previous point's; the first point's is the aircraft's own now, and a rotorcraft given no speed flies its "
                    "cruise speed over the ground. References may be given by name. As A-GRA's schema gives it "
                    "(docs/flight-autonomy.md, 4.29): its altitude block ``altitude_min_m``/``altitude_max_m`` (an altitude left "
                    "out held within it), its ``kind`` (fsim.EndPointKind: a turn point, or a waypoint - no turn, flown over) "
                    "and a waypoint's ``waypoint_type`` (fsim.WaypointType), a point in a ``frame`` (World.create_frame's id) "
                    "at its offsets ``frame_rotation``, ``frame_offsets``, ``frame_x_m``, ``frame_y_m``, ``frame_z_m`` - "
                    "placed where the frame is, a moving one's as it is flown. A turn point's (4.30): ``turn`` "
                    "'capture_outbound_course', 'start_turn', 'end_turn', its ``course_rad`` and ``turn_radius_m``. A loiter "
                    "point (``kind`` 'loiter_point', 4.31) flies the fsim.RouteLoiter naming it. Its segment's performance "
                    "(4.32): ``speed_optimization`` (fsim.SpeedOptimization: the tables' best now, its speed replaced; left out "
                    "with the speed, the point before's), ``climb_optimization`` (fsim.ClimbOptimization) and "
                    "``acceleration_ms2`` (the speed change into the segment at it). Its required time of arrival (4.33): "
                    "``arrival_begin_s`` and ``arrival_end_s``, the window it is to arrive in (World.time's clock; either "
                    "side left out, open) - its speed scheduled over the ground to arrive in it. Its required navigation "
                    "performance (4.35): ``rnp_m``, how far off its path the segment may be flown (left out, none); farther, its "
                    "ActivityInfo's ``constraints`` carry fsim.ActivityFlag.NAVIGATION_PERFORMANCE. The point flown after it "
                    "(4.36): ``next``, its index (-1: the route ends there; left out, the next in its path, or as given). Its "
                    "civil path terminator (4.38): ``terminator`` (fsim.PathTerminator or its name - 'tf', 'rf', ...), the ARINC 424 "
                    "leg type of the leg into it, its data in an fsim.RouteTerminator.")

RouteLoiter = collections.namedtuple("RouteLoiter", ("point",) + MODE_FIELDS["pattern"] + ("end_time_s",), defaults=(0,) + (HOLD,) * 36)
RouteLoiter.__doc__ = ("The loiter a route's loiter point flies (A-GRA's LoiterPoint; docs/flight-autonomy.md, 4.31): at waypoint "
                       "``point`` (its ``kind`` 'loiter_point'), the pattern its fields give - submit_pattern's, by name or member "
                       "('orbit', 'racetrack', 'figure_eight', 'hold', 'hover'; ``orbits``, ``duration_s``, a hold's, its entry and "
                       "exit points) - and ``end_time_s``, when it ends (World.time's clock). Its place is its point's: its latitude, "
                       "longitude, altitude, reference and frame left out (HOLD); its speed left out, the point's segment's. It ends "
                       "when its duration or laps are flown (on round to its exit point) or at its end time, the first; with none, "
                       "it is the route's end (its last point's only). Flown where the leg meets it - an orbit along the tangent from "
                       "a radius outside its circle, anything else at its point (a rotorcraft stops for a hover) - then on to the "
                       "next point from where it leaves: an orbit where its tangent runs to it, a hold at its fix.")

RouteState = collections.namedtuple(
    "RouteState", "point latitude_rad longitude_rad altitude_m altitude_reference time_s frame frame_rotation frame_offsets frame_x_m frame_y_m "
                  "frame_z_m uncertainty_m ground_north_ms ground_east_ms domain_north_ms domain_east_ms domain_down_ms relative_north_ms "
                  "relative_east_ms relative_down_ms acceleration_north_ms2 acceleration_east_ms2 acceleration_down_ms2 yaw_rad pitch_rad roll_rad "
                  "yaw_rate_rad_s pitch_rate_rad_s roll_rate_rad_s",
    defaults=(0,) + (HOLD,) * 29)
RouteState.__doc__ = ("A planned inertial state inside a route's segment (A-GRA's InertialState; docs/flight-autonomy.md, 4.34): on "
                      "the segment ending at waypoint ``point``, where the aircraft is to be - ``latitude_rad``, ``longitude_rad``, "
                      "``altitude_m`` in ``altitude_reference`` (left out, its point's) - and when, ``time_s`` (World.time's clock); or "
                      "a place in a ``frame`` at its offsets (``frame_rotation``, ``frame_offsets``, ``frame_x_m``, ``frame_y_m``, "
                      "``frame_z_m``), where the frame is at its time. The segment's first lap flies through its altitude and "
                      "arrives at it at its time; the rest A-GRA's plan gives there - ``uncertainty_m``, its ground, air and "
                      "relative velocities, its acceleration, its orientation and their rates - is kept and read back. At most 64, "
                      "in order along the route, each on its leg within its uncertainty (50 m at least, or 1 % of the leg).")

RouteBranch = collections.namedtuple(
    "RouteBranch", "point next altitude_min_m altitude_max_m altitude_reference time_begin_s time_end_s captures captures_comparison "
                   "operator_input endurance_comparison fuel_kg endurance_s endurance_end_s percent contingency",
    defaults=(0,) + (HOLD,) * 15)
RouteBranch.__doc__ = ("A route's conditional branch (A-GRA's ConditionalPathSegment; docs/flight-autonomy.md, 4.37): as the aircraft "
                       "comes to waypoint ``point``, the point flown after it is ``next`` (-1: the route's end there) where every "
                       "condition given holds - its altitude between ``altitude_min_m`` and ``altitude_max_m`` (in "
                       "``altitude_reference``; left out, above mean sea level), the time between ``time_begin_s`` and "
                       "``time_end_s`` (World.time's clock), the times it has come to the point, this one too, compared by "
                       "``captures_comparison`` (fsim.Comparison or its name) with ``captures``; with ``operator_input`` 1, only once "
                       "the operator has commanded it (Activity.command_branch); what it has left, compared by "
                       "``endurance_comparison`` with each of ``fuel_kg``, ``endurance_s``, ``endurance_end_s`` and ``percent`` given; "
                       "its ``contingency`` (fsim.Contingency or its name) - as its navigation report says. A mission critical or "
                       "lost comms contingency is refused not_implemented until FA-16. At most 16; those at one point tried in their "
                       "order, the first that holds taken, the route planned again from there.")

RouteTerminator = collections.namedtuple(
    "RouteTerminator", "point course_rad center_latitude_rad center_longitude_rad radius_m course_in_rad course_out_rad "
                       "initial_latitude_rad initial_longitude_rad end_latitude_rad end_longitude_rad arc_m chord_m clockwise",
    defaults=(0,) + (HOLD,) * 13)
RouteTerminator.__doc__ = ("A civil path terminator's data (A-GRA's CF_CourseToFixType, RF_RadiusToFixType; docs/flight-autonomy.md, "
                           "4.38): the leg into waypoint ``point``, whose ``terminator`` names it. A course to fix's ``course_rad`` "
                           "(into its point, from true north); a radius to fix's arc - its centre ``center_latitude_rad``, "
                           "``center_longitude_rad`` and way round ``clockwise`` (1 or True: right; 0: left) required, and where given "
                           "its ``radius_m``, its courses at its start and end ``course_in_rad``, ``course_out_rad``, its ends "
                           "``initial_*`` (the point before) and ``end_*`` (its point), its length ``arc_m`` and ``chord_m`` must be "
                           "its own (a metre or half a percent; a degree). At most 64, one a point.")

RoutePath = collections.namedtuple("RoutePath", "id type first count", defaults=(0, HOLD, 0, 0))
RoutePath.__doc__ = ("One of a route's paths (A-GRA's MA_RoutePathType; docs/flight-autonomy.md, 4.36): ``count`` of its waypoints "
                     "from ``first``, flown in order unless a point's ``next`` says otherwise - its last the route's end unless its "
                     "``next`` goes on - with its ``id`` and ``type`` (fsim.PathType or its name). At most 16, tiling the waypoints in "
                     "order; the route begins at the first path's first point.")

PointMetadata = collections.namedtuple("PointMetadata", "point source locked modified remarks_name remarks fix_key fix_system",
                                       defaults=(0, PointSource.AUTO_ROUTED, False, False, "", "", "", ""))
PointMetadata.__doc__ = ("A route plan's point's planning metadata (A-GRA's MA_PathSegmentType's Source, Locked, Modified, Remarks "
                         "and Fix_Identifier; docs/flight-autonomy.md, 4.39), for waypoint ``point``: its ``source`` "
                         "(fsim.PointSource or its name), ``locked``, ``modified``, its remarks (``remarks_name``: 32 characters at "
                         "most; ``remarks``: 1,024) and its fix's identifier (``fix_key``, ``fix_system``: 256 each). Printable ASCII. "
                         "Kept with its plan and read back; nothing flies by it.")

PathMetadata = collections.namedtuple("PathMetadata", "path initial endurance_s fuel_kg gross_weight_kg transition_plan airfield runway",
                                      defaults=(0, None, HOLD, HOLD, HOLD, 0, 0, 0))
PathMetadata.__doc__ = ("A route plan's path's planning metadata (A-GRA's MA_RoutePathType.InitialConditions; "
                        "docs/flight-autonomy.md, 4.39), for path ``path`` (0 without paths): the aircraft's state as planned or "
                        "assessed where it begins - ``initial``, a fsim.RouteState (its point not used; None: left out), its "
                        "endurance (``endurance_s``, ``fuel_kg``), ``gross_weight_kg``, and ``transition_plan``, a route plan's id "
                        "(0 none); its ``airfield`` and ``runway`` (A-GRA's AirfieldID, RunwayID; 0 none), which a takeoff's or a "
                        "landing's path on FA's own plan names from those the vehicle keeps (4.40). Kept with its plan and read back; "
                        "nothing flies by it.")

RunwayPoint = collections.namedtuple("RunwayPoint", "latitude_rad longitude_rad altitude_m altitude_reference", defaults=(HOLD,) * 4)
RunwayPoint.__doc__ = ("A place on a runway (A-GRA's Point3D_Type; docs/flight-autonomy.md, 4.40): above the WGS-84 ellipsoid "
                       "unless ``altitude_reference`` (fsim.AltitudeReference or its name) says otherwise; its latitude left out, none.")

RunwayCoordinates = collections.namedtuple("RunwayCoordinates", "start threshold limit", defaults=(RunwayPoint(),) * 3)
RunwayCoordinates.__doc__ = ("A runway's takeoff or landing coordinates (A-GRA's RunwayCoordinatesType): its ``start`` - required where "
                             "any is given - its ``threshold`` and ``limit``, its nearest and furthest points (fsim.RunwayPoint).")

Runway = collections.namedtuple("Runway", "id direction_rad available_length_m takeoff landing",
                                defaults=(0, HOLD, HOLD, RunwayCoordinates(), RunwayCoordinates()))
Runway.__doc__ = ("A runway (A-GRA's AirfieldRunwayType): its ``id`` (not 0), ``direction_rad`` (from true north, 0 to 2 pi), "
                  "``available_length_m``, and its ``takeoff`` and ``landing`` coordinates (fsim.RunwayCoordinates), at least one.")

Airfield = collections.namedtuple("Airfield", "id icao qnh_pa runways revision", defaults=(0, "", HOLD, (), 0))
Airfield.__doc__ = ("An airfield (A-GRA's AirfieldReportMDT; docs/flight-autonomy.md, 4.40): FA's, loaded by the platform "
                    "(Vehicle.load_airfield) and read only to MA - its ``id`` (not 0), ``icao`` code (four capitals, or none), "
                    "``qnh_pa`` (850 to 1,100 hPa), its ``runways`` (fsim.Runway, 16 at most) and, read back, its ``revision``.")

RoutePlan = collections.namedtuple(
    "RoutePlan", "id route version for_planning_use_only detailed remarks_name remarks point_metadata path_metadata",
    defaults=(0, False, False, "", "", (), ()))
RoutePlan.__doc__ = ("A route plan (A-GRA's MA_RoutePlanMT; docs/flight-autonomy.md, 4.39): a route FA keeps by its ``id`` (not 0) "
                     "and ``version``, taken through the plan activation states before it flies (Vehicle.plan_command). Its "
                     "``route`` a fsim.BatchCommand(\"submit_route\", ...) - its waypoints and their extras, its options (its "
                     "command options not kept: an activation gives them); ``for_planning_use_only`` never activated; its planning "
                     "metadata - ``detailed``, its remarks (``remarks_name``, ``remarks``), fsim.PointMetadata and "
                     "fsim.PathMetadata, one a point and one a path at most - kept and read back, flown by nothing.")

OpPoint = collections.namedtuple(
    "OpPoint", "id latitude_rad longitude_rad altitude_m altitude_reference frame frame_rotation frame_offsets frame_x_m frame_y_m "
               "frame_z_m ingress_min_rad ingress_max_rad revision", defaults=(HOLD,) * 12 + (0,))
OpPoint.__doc__ = ("An operational point (A-GRA's OpPoint; docs/flight-autonomy.md, 4.42), kept by the world by its ``id`` (not 0) "
                   "for a must fly to name (World.set_op_point): a place on the Earth - ``latitude_rad``, ``longitude_rad`` - or in "
                   "a frame (``frame``, World.create_frame's id, and its offsets as a route point's: ``frame_rotation``, "
                   "``frame_offsets``, ``frame_x_m``, ``frame_y_m``, ``frame_z_m``); its ``altitude_m`` in ``altitude_reference`` "
                   "(left out, none: flown at the aircraft's); the window of bearings from it it is approached from, "
                   "``ingress_min_rad`` clockwise to ``ingress_max_rad`` (both or neither); read back, its ``revision``.")


BezierSegment = collections.namedtuple("BezierSegment", "north east down")
BezierSegment.__doc__ = ("One segment of a curve (A-GRA's): a quintic Bezier by its six control points - ``north``, ``east`` "
                         "and ``down``, six metres each, from the curve's reference - with weights 1 and the clamped knots "
                         "[0,0,0,0,0,0,1,1,1,1,1,1]. Each segment starts where the one before ends (within a metre).")


NurbsSegment = collections.namedtuple("NurbsSegment", "north east down knots weights curvature first_index last_index",
                                      defaults=(None, HOLD, HOLD, HOLD))
NurbsSegment.__doc__ = ("One segment of a curve as A-GRA's schema gives it (MA_NURBS_PointType; docs/flight-autonomy.md, "
                        "4.26): a clamped rational B-spline - its 4 to 10 control points ``north``, ``east`` and ``down`` in "
                        "metres from the curve's reference, each with its ``weights`` (None: all 1), and its 4 to 14 ``knots`` "
                        "(its degree: knots - points - 1), from 0 and never decreasing, the first and the last each as many "
                        "times as the degree and once more. ``curvature`` (1/m; HOLD: not said) is the most it turns, checked "
                        "against it; ``first_index`` and ``last_index`` (HOLD: not said) its first and last control points, "
                        "0 and its last. A BezierSegment is one: six points, weights 1, knots [0]*6 + [1]*6.")


def _nurbs_of(s):
    """A segment (fsim.NurbsSegment, dicts of its fields, or an fsim.BezierSegment) as fsim.NurbsSegment."""
    if isinstance(s, dict):
        s = NurbsSegment(**s) if "knots" in s else BezierSegment(**s)
    if isinstance(s, NurbsSegment):
        return s
    b = s if isinstance(s, BezierSegment) else BezierSegment(*s)
    return NurbsSegment(list(b.north), list(b.east), list(b.down), [0.0] * 6 + [1.0] * 6)


def _is_nurbs(segments):
    """Whether any of the segments is given as A-GRA's schema gives it (fsim.NurbsSegment, or a dict with knots)."""
    return any(isinstance(s, NurbsSegment) or (isinstance(s, dict) and "knots" in s) for s in segments)


def _nurbs(segments):
    """Segments as the native rows of 59 numbers: points, knots, north[10], east[10], down[10], weight[10], knot[14],
    curvature, first_index, last_index."""
    rows = []
    for s in segments:
        s = _nurbs_of(s)
        points, knots = len(s.north), len(s.knots)
        weights = [1.0] * points if s.weights is None else list(s.weights)
        if len(s.east) != points or len(s.down) != points or len(weights) != points:
            raise ValueError("a segment's north, east, down and weights are one for each control point")
        if points > 10 or knots > 14:
            raise ValueError("a segment has at most 10 control points and 14 knots")

        def pad(v, n, fill=0.0):
            return [float(x) for x in v] + [fill] * (n - len(v))
        rows.append(tuple([float(points), float(knots)] + pad(s.north, 10) + pad(s.east, 10) + pad(s.down, 10) + pad(weights, 10, 1.0) +
                          pad(s.knots, 14) + [float(s.curvature), float(s.first_index), float(s.last_index)]))
    return rows


def _nurbs_segment(row):
    """A native row of 59 numbers as fsim.NurbsSegment."""
    p, k = int(row[0]), int(row[1])
    return NurbsSegment(list(row[2:2 + p]), list(row[12:12 + p]), list(row[22:22 + p]), list(row[42:42 + k]), list(row[32:32 + p]), row[56],
                        row[57], row[58])


def _segments(segments):
    """Segments (fsim.BezierSegment, dicts of its fields, or (north, east, down) triples) as the native rows."""
    rows = []
    for s in segments:
        if isinstance(s, dict):
            s = BezierSegment(**s)
        elif not isinstance(s, BezierSegment):
            s = BezierSegment(*s)
        row = tuple(float(v) for axis in s for v in axis)
        if len(row) != 18:
            raise ValueError("a segment is six control points each north, east and down")
        rows.append(row)
    return rows


def _op_point(t):
    """An operational point from its native tuple (id, revision, then its fields in order)."""
    return OpPoint(t[0], *t[2:14], revision=t[1])


def _waypoints(points):
    """Waypoints (fsim.Waypoint, dicts of its fields, or rows in its order) as the native rows."""
    rows = []
    for p in points:
        if isinstance(p, dict):
            p = Waypoint(**p)
        elif not isinstance(p, Waypoint):
            p = Waypoint(*p)
        values = [_REFERENCES[k][v.upper()] if isinstance(v, str) and k in _REFERENCES else v for k, v in zip(Waypoint._fields, p)]
        rows.append(tuple(float(v) for v in values[:9]) + (int(values[9]),) + tuple(float(v) for v in values[10:]))
    return rows


def _loiters(loiters):
    """Loiters (fsim.RouteLoiter or dicts of its fields) as the native rows: point, the pattern's 35 fields, end_time_s."""
    rows = []
    for l in loiters:
        if isinstance(l, dict):
            l = RouteLoiter(**l)
        elif not isinstance(l, RouteLoiter):
            l = RouteLoiter(*l)
        values = [_REFERENCES[k][v.upper()] if isinstance(v, str) and k in _REFERENCES else v for k, v in zip(RouteLoiter._fields, l)]
        rows.append((int(values[0]),) + tuple(float(v) for v in values[1:]))
    return rows


def _states(states):
    """States (fsim.RouteState or dicts of its fields) as the native rows: point, then the 29 fields."""
    rows = []
    for s in states:
        if isinstance(s, dict):
            s = RouteState(**s)
        elif not isinstance(s, RouteState):
            s = RouteState(*s)
        values = [_REFERENCES[k][v.upper()] if isinstance(v, str) and k in _REFERENCES else v for k, v in zip(RouteState._fields, s)]
        rows.append((int(values[0]),) + tuple(float(v) for v in values[1:]))
    return rows


def _paths(paths):
    """Paths (fsim.RoutePath or dicts of its fields) as the native rows: id, type, first, count."""
    rows = []
    for r in paths:
        if isinstance(r, dict):
            r = RoutePath(**r)
        elif not isinstance(r, RoutePath):
            r = RoutePath(*r)
        kind = PathType[r.type.upper()] if isinstance(r.type, str) else r.type
        rows.append((int(r.id), float(kind), int(r.first), int(r.count)))
    return rows


def _branches(branches):
    """Branches (fsim.RouteBranch or dicts of its fields) as the native rows: point, then the 15 fields - codes by name or
    member."""
    codes = {"altitude_reference": AltitudeReference, "captures_comparison": Comparison, "endurance_comparison": Comparison,
             "contingency": Contingency}
    rows = []
    for b in branches:
        if isinstance(b, dict):
            b = RouteBranch(**b)
        elif not isinstance(b, RouteBranch):
            b = RouteBranch(*b)
        values = [codes[k][v.upper()] if isinstance(v, str) and k in codes else v for k, v in zip(RouteBranch._fields, b)]
        rows.append((int(values[0]),) + tuple(float(v) for v in values[1:]))
    return rows


def _terminators(terminators):
    """Terminators' data (fsim.RouteTerminator or dicts of its fields) as the native rows: point, then the 13 fields."""
    rows = []
    for t in terminators:
        if isinstance(t, dict):
            t = RouteTerminator(**t)
        elif not isinstance(t, RouteTerminator):
            t = RouteTerminator(*t)
        rows.append((int(t[0]),) + tuple(float(v) for v in t[1:]))
    return rows


def _row(level, values, fields):
    """A level's (or support kind's, or mode's) fields: all of them in order - a level's COMMAND_FIELDS, or its
    SETPOINT_FIELDS with the rotorcraft's - or some by name with the rest as a new command's defaults."""
    if isinstance(level, str) and level in MODE_FIELDS:
        # a reference by name ("mach") or enum member
        fields = {k: (_REFERENCES[k][v.upper()] if isinstance(v, str) and k in _REFERENCES else v) for k, v in fields.items()}
        names, defaults, what = MODE_FIELDS[level], MODE_DEFAULTS[level], level
        counts = (len(names),)
    elif isinstance(level, str):
        names, defaults, what = SUPPORT_FIELDS[level], SUPPORT_DEFAULTS[level], level
        counts = (len(names),)
    else:
        names, defaults, what = SETPOINT_FIELDS[level], SETPOINT_DEFAULTS[level], level.name
        counts = (len(COMMAND_FIELDS[level]), len(names))
    if values:
        if fields or len(values) not in counts:
            raise TypeError("%s takes %s values in order (%s) or fields by name" % (what, " or ".join(str(c) for c in sorted(set(counts))),
                                                                                   ", ".join(names)))
        return tuple(float(v) for v in values)
    row = list(defaults)
    for key, value in fields.items():
        if key not in names:
            raise TypeError("%s has no field %r (fields: %s)" % (what, key, ", ".join(names)))
        row[names.index(key)] = float(value)
    return tuple(row)


class Activity:
    """A command a vehicle accepted: it runs until it completes, fails or is
    canceled. ``update`` gives it a new setpoint - its per-step path - and
    ``cancel`` ends it, handing its axes to the vehicle default. Both declare
    the ``source`` it was submitted with: under ControlMode.GRANTED a source
    below an activity's may not address it (fsim.Rejected "authority_held"),
    so a policy cannot change or end what the platform's own sources fly."""

    __slots__ = ("world", "id", "level", "clamped", "source", "command_id", "deferred", "controller", "overridden", "endurance")

    def __init__(self, world, activity_id, level, clamped=False, source=Source.POLICY, command_id=0, deferred=False, controller=0,
                 overridden=False, endurance=None):
        self.world = world
        self.id = activity_id
        self.level = level
        self.clamped = clamped  #: a value of the command was clamped to its range (World.last_command_details: which, and to what)
        self.source = Source(source)  #: the source it was submitted with, which its update and cancel declare
        self.command_id = command_id  #: the caller's id for its command (docs/flight-autonomy.md, 4.8), 0 none
        #: it was accepted to wait (docs/flight-autonomy.md, 4.9): for its start window, or for axes held by what it may
        #: not interrupt (``info.waiting``, ``info.waiting_for``)
        self.deferred = deferred
        #: the policy's controller it was submitted by (docs/flight-autonomy.md, 4.12), which its update, cancel and
        #: activity commands declare as they do its source
        self.controller = int(controller)
        #: accepted over a soft rejection with override_rejection (docs/flight-autonomy.md, 4.18): its flight needs more
        #: than the vehicle has, by ``endurance`` (fsim.Endurance)
        self.overridden = overridden
        self.endurance = endurance

    @property
    def vehicle(self):
        return self.world._vehicle(self.id >> 32)

    def update(self, *values, **fields):
        """A new setpoint: the level's fields by name (fsim.COMMAND_FIELDS),
        the others as a new command's defaults, or all of them in order - a
        mode's fields given, the others kept (a route's options, its waypoints
        kept and flown afresh). Returns True if a value was clamped; raises
        fsim.Rejected if the activity has ended (preempted, completed,
        canceled) or takes no updates (a behaviour: a new target is a new
        submit_behavior)."""
        h = self.world._h
        if self.level == Level.BEHAVIOR:  # the library answers: not_updatable, or why not
            return bool(_checked(h.activity_update(self.id, (), int(self.source), self.controller), h)[4])
        return bool(_checked(h.activity_update(self.id, _row(self.level, values, fields), int(self.source), self.controller), h)[4])

    def update_route(self, waypoints=None, loiters=None, states=None, paths=None, branches=None, terminators=None, **options):
        """UPDATE of a route: new ``waypoints`` with their ``loiters`` (fsim.RouteLoiter), ``states`` (fsim.RouteState),
        ``paths`` (fsim.RoutePath), ``branches`` (fsim.RouteBranch) and ``terminators`` (fsim.RouteTerminator) - None: those it
        has, and theirs - and the options
        given (the others kept); checked as a NEW's, then flown afresh from its start, from where the aircraft is (what the
        operator commanded forgotten). Returns True if a value was clamped; raises fsim.Rejected (``index`` the waypoint at
        fault)."""
        rows = [] if waypoints is None else _waypoints(waypoints)
        h = self.world._h
        return bool(_checked(h.activity_update_route(self.id, _row("route", (), options), rows, int(self.source), self.controller,
                                                     None if loiters is None else _loiters(loiters), None if states is None else _states(states),
                                                     None if paths is None else _paths(paths), None if branches is None else _branches(branches),
                                                     None if terminators is None else _terminators(terminators)),
                             h)[4])

    def command_branch(self, branch, commanded=True):
        """The operator's input to its route's conditional branch ``branch`` (its index; one with ``operator_input`` 1;
        docs/flight-autonomy.md, 4.37): commanded - or, ``commanded`` False, no longer - as it flies or waits; taken the
        next time its point is come to, where its other conditions hold. Declares the source it was submitted with, as
        update does; raises fsim.Rejected (invalid_parameter, ``index`` the branch, for one it has not or one that takes no
        operator input)."""
        h = self.world._h
        _checked(h.activity_command_branch(self.id, int(branch), 1 if commanded else 0, int(self.source), self.controller), h)

    def update_curve(self, segments=None, **options):
        """UPDATE of a curve: new ``segments`` (fsim.BezierSegment or fsim.NurbsSegment; None: those it has) and the options given (the
        others kept) - with ``append=1`` the segments go after its end, from the same reference; else they are a new
        curve, flown afresh. Returns True if a value was clamped; raises fsim.Rejected (``index`` the segment at
        fault; ``section`` where a segment is too tight)."""
        h = self.world._h
        if segments is not None and _is_nurbs(segments):  # (as A-GRA's schema gives them)
            return bool(_checked(h.activity_update_nurbs(self.id, _row("curve", (), options), _nurbs(segments), int(self.source), self.controller), h)[4])
        rows = [] if segments is None else _segments(segments)
        return bool(_checked(h.activity_update_curve(self.id, _row("curve", (), options), rows, int(self.source), self.controller), h)[4])

    def append(self, segments, **options):
        """A curve's segments after its end, from the same reference: flown on to, the activity the same."""
        return self.update_curve(segments, append=1, **options)

    def cancel(self):
        """End it: its axes fly the vehicle default. Raises fsim.Rejected if it had already ended."""
        _checked(self.world._h.activity_cancel(self.id, int(self.source), self.controller), self.world._h)

    # Activity commands (docs/flight-autonomy.md, 4.10; A-GRA's ActivityCommandBaseType), declaring the source it was
    # submitted with. Each raises fsim.Rejected: "not_interactive" where its command said interactive=False,
    # "activity_ended" once it has ended, "queue_full" where a flying one has no room to be kept.
    def _command(self, command, rank=(0, 0)):
        rank = Rank(*rank)
        _checked(self.world._h.activity_command(self.id, command, int(rank.priority), int(rank.precedence), int(self.source), self.controller),
                 self.world._h)

    def disable(self):
        """It stops flying and is kept, DISABLED (live), until enabled; its axes go to the vehicle default or to
        what waits for them."""
        self._command(0)

    def enable(self):
        """A disabled activity waits to start again, and starts when it may - a route at the point it flew to.
        One live and enabled stays so."""
        self._command(1)

    def reset(self):
        """Over from its beginning: a flying one's behaviour starts afresh (a route from its first point)."""
        self._command(2)

    def delete(self):
        """A sticky disable: it ends DELETED, and cannot be enabled."""
        self._command(3)

    def change_rank(self, rank):
        """Its rank (fsim.Rank or (priority, precedence)) changes: what it contests is arbitrated afresh."""
        self._command(4, rank)

    def unassign(self):
        """It gives up its axes and waits for them again, behind what waits."""
        self._command(5)

    def setpoint(self):
        """What it flies now, or waits to fly (A-GRA's last flight command; docs/flight-autonomy.md, 4.12), as the
        fsim.BatchCommand that would command it: a mode's fields as merged, a level's all of them, a support command's, a
        behaviour's, a route's waypoints (fsim.Waypoint), a curve's segments with the appended ones (fsim.BezierSegment:
        its flyout curve, fsim.agra.flyout_curve). A waiting one's is as given. None once it is not live."""
        return self.world.activity_setpoint(self.id)

    def end_points(self, max=16):
        """Where it flies to (A-GRA's ActualEndPoint): the point it flies to now, then those after it - a route's
        waypoints (a repeating route's round again), a curve's segment ends, a pattern's fix, the position level's
        point - ``max`` at most (fsim.EndPoint). Empty once it is not live, and for an hsa or a behaviour."""
        return self.world.end_points(self.id, max)

    @property
    def info(self):
        """Its record (ActivityInfo), or None once the vehicle no longer remembers it."""
        return self.world.activity(self.id)

    @property
    def progress(self):
        """How far it has got and what it commands (ActivityProgress), or None once the vehicle no longer
        remembers it. A guidance mode's; NaN fields for the others."""
        return self.world.activity_progress(self.id)

    @property
    def state(self):
        info = self.info
        return None if info is None else info.state

    @property
    def live(self):
        return self.state in (ActivityState.PENDING, ActivityState.ACTIVE, ActivityState.DISABLED)

    def __repr__(self):
        info = self.info
        what = self.level if isinstance(self.level, str) else self.level.name
        return "Activity(%#x, %s, %s)" % (self.id, what, info.state.name if info else "forgotten")


def _options(**given):
    """Only what was given: the library keeps its own defaults for the rest."""
    return {k: (int(v) if isinstance(v, bool) else v) for k, v in given.items() if v is not None}


class BatchCommand:
    """One command of Vehicle.submit_batch: a submit method's name ("submit", "submit_behavior", "submit_support",
    "submit_hsa", "submit_pattern", "submit_must_fly", "submit_route", "submit_curve") and the arguments it takes."""

    __slots__ = ("method", "args", "kwargs")
    _KINDS = {"submit": 0, "submit_behavior": 1, "submit_support": 2, "submit_hsa": 3, "submit_pattern": 3, "submit_must_fly": 3, "submit_route": 4,
              "submit_curve": 5}

    def __init__(self, method, *args, **kwargs):
        if method not in self._KINDS:
            raise ValueError("a batch command is one of %s" % ", ".join(sorted(self._KINDS)))
        self.method, self.args, self.kwargs = method, args, dict(kwargs)

    def _native(self, vehicle):
        """(the native item, (level, source, validate_only, controller)) - as the method would make its NEW."""
        k = dict(self.kwargs)
        source = Source(k.pop("source", Source.POLICY))
        axes = k.pop("axes", None)
        range_ = k.pop("range", RangePolicy.CLAMP)
        min_version = k.pop("min_version", 0)
        validate = bool(k.get("validate_only", False))
        controller = int(k.pop("controller", 0))
        envelope = _envelope(k.pop("command_id", 0), k.pop("trace", ()), k.pop("interactive", True), k.pop("validate_only", False),
                             k.pop("rank", None), k.pop("interrupt", True), k.pop("precedence_override", None), k.pop("window", None),
                             k.pop("override_rejection", False), controller)
        options = (int(source), None if axes is None else int(axes), int(range_), int(min_version), envelope)
        kind, args = self._KINDS[self.method], list(self.args)
        if self.method == "submit":
            level = Level(args.pop(0))
            return (kind, int(level), _row(level, args, k), None, None, None, options), (level, source, validate, controller)
        if self.method == "submit_behavior":
            behavior = args.pop(0)
            target = args.pop(0) if args else k.pop("target", None)
            points = args.pop(0) if args else k.pop("points", None)
            t = target.id if isinstance(target, Vehicle) else int(target or 0)
            rows = None if points is None else [tuple(float(x) for x in p) for p in points]
            return (kind, 0, (), (behavior, t, k or None, rows), None, None, options), (Level.BEHAVIOR, source, validate, controller)
        if self.method == "submit_support":
            what = args.pop(0)
            return (kind, SUPPORT_KINDS.index(what), _row(what, args, k), None, None, None, options), (what, source, validate, controller)
        if self.method in ("submit_hsa", "submit_pattern", "submit_must_fly"):
            mode = self.method[len("submit_"):]
            if "target" in k:  # (a must fly's vehicle as itself or its id)
                k["target"] = getattr(k["target"], "id", k["target"])
            return (kind, MODE_KINDS.index(mode), _row(mode, args, k), None, None, None, options), (mode, source, validate, controller)
        if self.method == "submit_route":
            waypoints = args.pop(0) if args else k.pop("waypoints")
            route = {"projection": k.pop("projection", Projection.GREAT_CIRCLE), "repeat": 1.0 if k.pop("repeat", False) else 0.0,
                     "end": k.pop("end", EndBehavior.CONTINUE), "start": k.pop("start", 0)}
            loiters, states, paths, branches = k.pop("loiters", None), k.pop("states", None), k.pop("paths", None), k.pop("branches", None)
            terminators = k.pop("terminators", None)
            return ((kind, 0, _row("route", (), route), None, _waypoints(waypoints), None, options, None if loiters is None else _loiters(loiters),
                     None if states is None else _states(states), None if paths is None else _paths(paths),
                     None if branches is None else _branches(branches), None if terminators is None else _terminators(terminators)),
                    ("route", source, validate, controller))
        segments = args.pop(0) if args else k.pop("segments")
        if _is_nurbs(segments):  # (as A-GRA's schema gives them: its own kind, ABI 1.24)
            return (6, 0, _row("curve", (), k), None, None, _nurbs(segments), options), ("curve", source, validate, controller)
        return (kind, 0, _row("curve", (), k), None, None, _segments(segments), options), ("curve", source, validate, controller)


def _setpoint(t):
    """A native setpoint (activity_setpoint) as the fsim.BatchCommand that would command it."""
    kind, code, fields, behavior, waypoints, segments, loiters, states, paths, branches, terminators = t
    if kind == 0:
        level = Level(code)
        return BatchCommand("submit", level, **dict(zip(SETPOINT_FIELDS[level], fields)))
    if kind == 1:
        name, target, params, points = behavior
        return BatchCommand("submit_behavior", name, target, points or None, **params)
    if kind == 2:
        what = SUPPORT_KINDS[code]
        return BatchCommand("submit_support", what, **dict(zip(SUPPORT_FIELDS[what], fields)))
    if kind == 3:
        mode = MODE_KINDS[code]
        return BatchCommand("submit_" + mode, **dict(zip(MODE_FIELDS[mode], fields)))
    if kind == 4:
        route = dict(zip(MODE_FIELDS["route"], fields))
        more = {"loiters": [RouteLoiter(*l) for l in loiters]} if loiters else {}  # (its loiters, complete: 4.31)
        if states:  # (its planned states, as placed: 4.34)
            more["states"] = [RouteState(int(s[0]), *s[1:]) for s in states]
        if paths:  # (its paths: 4.36)
            more["paths"] = [RoutePath(*r) for r in paths]
        if branches:  # (its conditional branches: 4.37)
            more["branches"] = [RouteBranch(int(b[0]), *b[1:]) for b in branches]
        if terminators:  # (its civil path terminators' data: 4.38)
            more["terminators"] = [RouteTerminator(int(t[0]), *t[1:]) for t in terminators]
        return BatchCommand("submit_route", [Waypoint(*w) for w in waypoints], projection=route["projection"], repeat=route["repeat"] == 1.0,
                            end=route["end"], start=route["start"], **more)
    if kind == 6:  # a curve's segments as A-GRA's schema gives them
        return BatchCommand("submit_curve", [_nurbs_segment(s) for s in segments], **dict(zip(MODE_FIELDS["curve"], fields)))
    return BatchCommand("submit_curve", [BezierSegment(s[0:6], s[6:12], s[12:18]) for s in segments], **dict(zip(MODE_FIELDS["curve"], fields)))


class Vehicle:
    """A vehicle in a World; valid until removed. Cheap: an id and a world."""

    __slots__ = ("_world", "_h", "id", "_state", "_sensed", "__weakref__")

    def __init__(self, world, vehicle_id):
        self._world = world
        self._h = world._h
        self.id = vehicle_id
        self._state = None
        self._sensed = None

    # --- identity -------------------------------------------------------------
    @property
    def name(self):
        return self._h.vehicle_info(self.id)[0]

    @property
    def type(self):
        return self._h.vehicle_info(self.id)[1]

    @property
    def world(self):
        return self._world

    def __repr__(self):
        return "Vehicle(%d, %r)" % (self.id, self.name)

    def __eq__(self, other):
        return isinstance(other, Vehicle) and other._h is self._h and other.id == self.id

    def __hash__(self):
        return hash((id(self._h), self.id))

    # --- state ------------------------------------------------------------------
    @property
    def state(self):
        """Truth, live: a VehicleState over the platform's own snapshot, rewritten
        in place every step (copy it - ``VehicleState.from_buffer_copy(v.state)`` -
        to keep one)."""
        s = self._state
        if s is None:
            s = self._state = VehicleState.from_buffer(self._h.state_buffer(self.id, 0))
        return s

    @property
    def sensed(self):
        """The state as the sensor effects report it, live like ``state``."""
        s = self._sensed
        if s is None:
            s = self._sensed = VehicleState.from_buffer(self._h.state_buffer(self.id, 1))
        return s

    def get_property(self, path):
        """A JSBSim property, e.g. "atmosphere/wind-north-fps"."""
        return self._h.get_property(self.id, path)

    def set_property(self, path, value):
        self._h.set_property(self.id, path, value)

    # --- control -----------------------------------------------------------------
    def command_actuator(self, aileron=0.0, elevator=0.0, rudder=0.0, throttle=0.0, flaps=0.0, gear_down=HOLD,
                         brake_left=0.0, brake_right=0.0):
        """Surfaces in [-1, 1], throttle/flaps/brakes in [0, 1]."""
        self._h.command(self.id, 0, (aileron, elevator, rudder, throttle, flaps, gear_down, brake_left, brake_right))

    def command_attitude(self, roll_rad=0.0, pitch_rad=0.0, heading_rad=HOLD, max_bank_rad=0.785, throttle=HOLD,
                         airspeed_ms=HOLD):
        self._h.command(self.id, 1, (roll_rad, pitch_rad, heading_rad, max_bank_rad, throttle, airspeed_ms))

    def command_acceleration(self, load_factor_g=1.0, roll_rate_rad_s=0.0, longitudinal_ms2=HOLD, throttle=HOLD):
        self._h.command(self.id, 2, (load_factor_g, roll_rate_rad_s, longitudinal_ms2, throttle))

    def command_velocity(self, airspeed_ms=HOLD, vertical_speed_ms=0.0, heading_rad=HOLD, turn_rate_rad_s=HOLD):
        self._h.command(self.id, 3, (airspeed_ms, vertical_speed_ms, heading_rad, turn_rate_rad_s))

    def command_position(self, latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms=HOLD, capture_radius_m=200.0):
        self._h.command(self.id, 4, (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m))

    def fly_to(self, latitude_deg, longitude_deg, altitude_msl_m, airspeed_ms=HOLD, capture_radius_m=200.0):
        """command_position in degrees."""
        self.command_position(math.radians(latitude_deg), math.radians(longitude_deg), altitude_msl_m, airspeed_ms,
                              capture_radius_m)

    def command_behavior(self, behavior, target=None, points=None, **params):
        """A behaviour by id - "hold", "waypoints", "loiter", "pursuit", "evade",
        "formation", "aerobatics" or a registered one - with its numeric
        parameters as keywords. ``target`` is a Vehicle or id for behaviours
        that follow one; ``points`` a route for "waypoints", as rows of
        (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m)."""
        t = target.id if isinstance(target, Vehicle) else int(target or 0)
        rows = None if points is None else [tuple(float(x) for x in p) for p in points]
        self._h.command_behavior(self.id, behavior, t, params or None, rows)

    def submit(self, level, *values, source=Source.POLICY, axes=None, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
               interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
               override_rejection=False, controller=0, **fields):
        """NEW: a command at ``level`` - its fields by name (fsim.COMMAND_FIELDS),
        the others as the command's defaults, or all of them in order -
        becomes an Activity, answered at once. ``axes`` (fsim.Axis) owns part
        of the vehicle - Axis.LATERAL, Axis.PITCH, Axis.THRUST or a union (a
        rotorcraft's Axis.CYCLIC, Axis.YAW, Axis.THRUST), any primary axis at
        the actuator level - and the rest keep their owners;
        None owns them all. Raises fsim.Rejected if the vehicle refuses it (a
        higher source holds its axes, a value out of range with
        RangePolicy.REJECT, a required field left as HOLD, axes it cannot own
        apart, a controller that is not axis-aware).

        The command envelope (docs/flight-autonomy.md, 4.8), on every submit: ``command_id``, the caller's id,
        echoed and kept with the activity; ``trace``, the requirements it comes from as (fsim.RequirementKind or
        its name, id) pairs; ``interactive`` False: its activity takes no activity commands; ``validate_only``
        True: answered as a NEW would be, flying nothing - a fsim.Validation, never an Activity or Rejected.

        How it is arbitrated and scheduled (4.9), on every submit: ``rank`` (fsim.Rank or (priority, precedence);
        lower first) - a policy's command takes contested axes from what it ranks at or ahead of and waits for
        the rest; ``interrupt`` False: it waits until its axes are free (the platform's sources: it defers to
        rank); ``precedence_override`` (the platform's sources only) its capability's precedence for this command;
        ``window`` (fsim.TimeWindow or a dict of its fields) when it may start and should end;
        ``override_rejection`` over a soft rejection (none exists yet). One that waits is an Activity with
        ``deferred`` True, pending until it starts."""
        level = Level(level)
        if level == Level.BEHAVIOR:
            raise TypeError("submit_behavior() takes behaviours")
        r = self._h.submit(self.id, int(level), _row(level, values, fields), int(source), None if axes is None else int(axes), int(range),
                           int(min_version), _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, level, source, validate_only, controller)

    def _answer(self, r, level, source, validate_only, controller=0):
        """A NEW's answer: an Activity (fsim.Rejected raised), or a validation's fsim.Validation."""
        if validate_only:
            return _validation(r, self._h)
        r = _checked(r, self._h)
        return Activity(self._world, r[2], level, bool(r[4]), source, r[10], bool(r[15]), controller, bool(r[17]),
                        _endurance(self._h) if r[17] else None)

    def submit_behavior(self, behavior, target=None, points=None, *, source=Source.POLICY, range=RangePolicy.CLAMP,
                        min_version=0, command_id=0, trace=(), interactive=True, validate_only=False, rank=None, interrupt=True,
                        precedence_override=None, window=None, override_rejection=False, controller=0, **params):
        """NEW for a behaviour (see command_behavior for its arguments): an
        Activity, or fsim.Rejected. Behaviours that follow a vehicle need
        ``target``; a new target is a new submit. The command envelope as submit's."""
        t = target.id if isinstance(target, Vehicle) else int(target or 0)
        rows = None if points is None else [tuple(float(x) for x in p) for p in points]
        r = self._h.submit_behavior(self.id, behavior, t, params or None, rows, int(source), None, int(range), int(min_version),
                                    _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, Level.BEHAVIOR, source, validate_only, controller)

    def submit_hsa(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(), interactive=True,
                   validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None, override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.hsa, A-GRA's HSA/CSA (docs/vehicle-interface.md, 4.4): hold ``heading_rad`` or
        ``course_rad``, a ``speed`` in ``speed_reference`` (fsim.SpeedReference or its name: "true_airspeed",
        "calibrated_airspeed", "ground_speed", "mach") and ``altitude_m`` above ``altitude_reference``
        (fsim.AltitudeReference: "msl", "above_ground", "ellipsoid"); or, for its speed, ``speed_optimization``
        (fsim.SpeedOptimization: "long_range_cruise", "max_endurance"): the performance tables' best speed, flown at the
        altitude and weight now (a speed replaces it, it a speed). What it leaves out continues what a live hsa
        commanded, else what the aircraft flies now; a reference alone takes the aircraft's own value in it. An
        Activity whose ``update(**fields)`` changes only the fields given; fsim.Rejected if refused. The command
        envelope as submit's."""
        r = self._h.submit_mode(self.id, MODE_KINDS.index("hsa"), _row("hsa", values, fields), int(source), None, int(range), int(min_version),
                                _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, "hsa", source, validate_only, controller)

    def submit_route(self, waypoints, *, projection=Projection.GREAT_CIRCLE, repeat=False, end=EndBehavior.CONTINUE, start=0, loiters=None, states=None, paths=None,
                     branches=None, terminators=None, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                     interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                     override_rejection=False, controller=0):
        """NEW for fsim.guidance.route, A-GRA's waypoint following (docs/vehicle-interface.md, 4.5): fly
        ``waypoints`` (fsim.Waypoint, dicts of its fields, or rows in its order; at most 256) as legs - great circles
        or rhumb lines (``projection``: fsim.Projection or "great_circle", "rhumb") - from where the aircraft is to
        the point ``start``, with fly-by turns or fly-over points; again from the first point if ``repeat``; after
        the last point, ``end`` (fsim.EndBehavior: "continue", "loiter"). An Activity that completes after the last
        point (unless it repeats), whose progress names the point flown to; ``update_route`` gives it new waypoints
        or options. Its loiter points' ``loiters`` (fsim.RouteLoiter, at most 16; docs/flight-autonomy.md, 4.31) beside
        them: while one flies its progress names its point, done, with the pattern's time to go. Its planned
        ``states`` (fsim.RouteState, at most 64; 4.34): its segments' first lap flies through their altitudes, at their
        times. Its ``paths`` (fsim.RoutePath, at most 16; 4.36): runs of its waypoints with ids and types, flown along each
        point's ``next``. Its conditional ``branches`` (fsim.RouteBranch, at most 16; 4.37): at a point, another next where
        their conditions hold. Its civil path terminators' data (``terminators``: fsim.RouteTerminator, at most 64; 4.38): each
        point's leg as its ``terminator`` says. fsim.Rejected if refused: ``index`` names the waypoint (a loiter's, a state's,
        a branch's or a terminator's, its point),
        ``constraint`` the limit it breaks, and ``findings`` every waypoint at fault. The command envelope as submit's."""
        options = {"projection": projection, "repeat": 1.0 if repeat else 0.0, "end": end, "start": start}
        r = self._h.submit_route(self.id, _row("route", (), options), _waypoints(waypoints), int(source), None, int(range), int(min_version),
                                 _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller), None if loiters is None else _loiters(loiters), None if states is None else _states(states),
                                 None if paths is None else _paths(paths), None if branches is None else _branches(branches),
                                 None if terminators is None else _terminators(terminators))
        return self._answer(r, "route", source, validate_only, controller)

    def submit_pattern(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                       interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                       override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.pattern, A-GRA's loiter (docs/vehicle-interface.md, 4.6): ``pattern``
        (fsim.PatternKind or "orbit", "racetrack", "figure_eight", "hold", "hover") round ``latitude_rad``, ``longitude_rad``
        (its centre or fix) at ``altitude_m``, with ``radius_m``, ``clockwise``, ``course_rad`` (the inbound course, a
        figure-eight's axis), ``leg_m``, a ``speed`` in ``speed_reference`` (or ``speed_optimization``, as submit_hsa's)
        and ``duration_s`` (then it completes). What it
        leaves out takes its default: an orbit here, as the aircraft flies now, right turns, the radius its speed and 80 %
        of its bank give (a hold's: rate one), a hold's minute-long legs. A-GRA's orbit as its schema gives it
        (docs/flight-autonomy.md, 4.23): ``direction_reference`` (fsim.DirectionReference of the course or heading),
        ``heading_rad`` (or the inbound heading: the course it makes good on it), ``leg_s`` (or the legs by the inbound
        leg's time), ``bank_rad`` (or the turns by bank), ``orbits`` (laps: then it completes; with a duration, the first),
        ``latitude2_rad``, ``longitude2_rad``, ``radius2_m`` (a racetrack's or a figure-eight's second circle),
        ``entry_latitude_rad``, ``entry_longitude_rad`` (where it joins, flown to directly) and ``exit_latitude_rad``,
        ``exit_longitude_rad`` (where it leaves, its duration or laps flown, out along its course). A hold's (4.24):
        ``turn_rate_rad_s`` or ``turn_type`` (fsim.HoldTurn) for its radius, ``hold_entry`` (fsim.HoldEntry: a racetrack's
        or a hold's way in) and ``hold_context`` (fsim.HoldContext: ATC's defaults for every one); its entry and exit
        times are the command's ``window``. A rotorcraft's hover (4.25) over its point, its duration from its arrival
        (a wing's refused not_supported). Its point in a frame (A-GRA's relative point): ``frame`` (World.create_frame's
        id), ``frame_rotation`` (fsim.FrameRotation), ``frame_offsets`` (fsim.FrameOffsets), ``frame_x_m``,
        ``frame_y_m``, ``frame_z_m`` (z down; left out, its own altitude) - carried as the frame moves; a point replaces
        a frame in an update, and a frame a point. Given more than one way to give the course, the legs or the radius,
        the first flies; an update of any one replaces them all. An Activity whose ``update(**fields)`` changes only what it
        gives; fsim.Rejected if refused. The command envelope as submit's."""
        r = self._h.submit_mode(self.id, MODE_KINDS.index("pattern"), _row("pattern", values, fields), int(source), None, int(range),
                                int(min_version), _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, "pattern", source, validate_only, controller)

    def submit_must_fly(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                        interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                        override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.must_fly, A-GRA's must fly (docs/flight-autonomy.md, 4.42): ``location``
        (fsim.MustFlyLocation or "point", "entity", "op_point") - a point at ``latitude_rad``, ``longitude_rad``; another
        vehicle, ``target`` (a Vehicle or its id); an operational point, ``target`` its id (World.set_op_point) - flown over
        at ``altitude_m`` in ``altitude_reference`` (left out: a point's the aircraft's; over a vehicle as far above it as
        the aircraft is, at least 500 ft; an operational point's its own), approached from within a window of bearings from
        it, ``ingress_min_rad`` clockwise to ``ingress_max_rad`` (both or neither; left out, an operational point's own), at
        a ``speed`` in ``speed_reference``. Laid out as a route from where the aircraft is - the points it approaches
        through, then the location, flown over - it completes as the location is passed and flies on along its course.
        An Activity whose ``update(**fields)`` merges what it gives (a location another than it was replacing the
        location's own); fsim.Rejected if refused ("unknown_geometry": an operational point the world does not keep). The
        command envelope as submit's."""
        if "target" in fields:
            fields["target"] = getattr(fields["target"], "id", fields["target"])
        r = self._h.submit_mode(self.id, MODE_KINDS.index("must_fly"), _row("must_fly", values, fields), int(source), None, int(range),
                                int(min_version), _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window,
                                                            override_rejection, controller))
        return self._answer(r, "must_fly", source, validate_only, controller)

    def submit_curve(self, segments, *, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                     interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                     override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.curve, A-GRA's curve following (docs/vehicle-interface.md, 4.7): fly ``segments``
        (fsim.BezierSegment, dicts of its fields, or (north, east, down) triples; 1 to 10), quintic Beziers in metres
        from ``latitude_rad``, ``longitude_rad``, ``altitude_m`` (left out: the aircraft now) - or as A-GRA's schema gives
        them, clamped rational B-splines (fsim.NurbsSegment; docs/flight-autonomy.md, 4.26). Within the ground speeds
        ``speed_min_ms`` to ``speed_max_ms`` (a wing holds its airspeed within them, a rotorcraft flies its ground
        speed), or so as to take ``duration_s``; left out, as it flies now. After its end, ``end``
        (fsim.EndBehavior: "continue", "loiter"). Its reference as A-GRA's schema gives it (4.27): ``altitude_reference``
        (fsim.AltitudeReference), ``altitude_min_m``/``altitude_max_m`` (its range: left out, the aircraft's altitude held within
        it), in a frame (``frame`` and its offsets, as submit_pattern's); its points' ``point_rotation`` (fsim.FrameRotation:
        turned with its frame), ``point_offsets`` (fsim.FrameOffsets: great circles are A-GRA's layout) and ``point_z``
        (fsim.CurveZ). Where a curve is changes only with a new curve's segments (appended ones go on from its reference,
        read as its). An Activity that completes at its end, whose progress names the
        segment flown; ``append`` adds segments while it flies, ``update_curve`` gives it a new curve or options.
        fsim.Rejected if refused: ``index`` names the segment, ``section`` where it is too tight, ``findings``
        every segment at fault. The command envelope as submit's."""
        submit = self._h.submit_nurbs if _is_nurbs(segments) else self._h.submit_curve
        rows = _nurbs(segments) if _is_nurbs(segments) else _segments(segments)
        r = submit(self.id, _row("curve", (), fields), rows, int(source), None, int(range), int(min_version),
                   _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection, controller))
        return self._answer(r, "curve", source, validate_only, controller)

    def submit_support(self, kind, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                       interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                       override_rejection=False, controller=0, **fields):
        """NEW for a support effector the vehicle has - "gear" (down), "flaps"
        (position), "wheel_brakes" (left, right), "speedbrake" (position),
        "pitch_trim" (position) - set directly beside the flight activity; or
        "engines" (throttle_1 .. throttle_4, HOLD keeps one), a throttle per
        engine that owns thrust, where the aircraft has more than one. An
        Activity (gear and flaps complete when they are there), or fsim.Rejected:
        "unknown_capability" if the aircraft has no such effector, "unavailable"
        for the placards (gear up on the ground, gear or flaps out too fast). The command envelope as submit's."""
        if kind not in SUPPORT_FIELDS:
            raise ValueError("support kind must be one of %s" % ", ".join(SUPPORT_KINDS))
        r = self._h.submit_support(self.id, SUPPORT_KINDS.index(kind), _row(kind, values, fields), int(source), None, int(range),
                                   int(min_version), _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, kind, source, validate_only, controller)

    def submit_batch(self, commands):
        """Several NEWs at once (A-GRA's several command instances in one message; docs/flight-autonomy.md,
        4.8): ``commands`` are fsim.BatchCommand("submit_hsa", heading_rad=1.0, command_id=1), naming a submit
        method and its arguments. Made in order at this simulation time and each answered on its own: a list of
        an Activity, a fsim.Validation (validate_only) or a fsim.Rejected - returned, not raised - per command.
        The last one's findings are World.last_command_details()'s."""
        items, kinds = [], []
        for c in commands:
            item, answer = c._native(self)
            items.append(item)
            kinds.append(answer)
        results = self._h.submit_batch(self.id, items)
        out = []
        for r, (level, source, validate, controller) in zip(results, kinds):
            if validate:
                out.append(_validation(r, None))
            elif r[0] == 1:
                out.append(_rejected(r, None))
            else:
                out.append(Activity(self._world, r[2], level, bool(r[4]), source, r[10], controller=controller))
        return out

    def activities(self):
        """The live activities (ActivityInfo), then the ended ones the vehicle remembers, newest first."""
        return [_info(t) for t in self._h.vehicle_activities(self.id)]

    # --- Flight tasks (docs/flight-autonomy.md, 4.11): a command kept by id, flown on a task command ----------
    def store_task(self, task_id, command, attempts=1, interval_s=None):
        """Keep a task: ``command`` a fsim.BatchCommand naming the submit it would be ("submit_hsa", "submit_route",
        "submit_behavior", ...; its options are the task command's, not kept here), flown ``attempts`` times - each
        run ``interval_s`` after the one before completes (None: at once), its one activity active between them.
        Raises fsim.Rejected: "invalid_parameter" (id 0 or a suggestion's, runs of what never completes),
        "task_active" (it flies), why the vehicle cannot command it."""
        item, (level, _source, _validate, _controller) = command._native(self)
        reason = self._h.store_task(self.id, int(task_id), item, int(attempts), math.nan if interval_s is None else float(interval_s))
        if reason:
            raise Rejected(_native.reason_name(reason))
        self._world._task_levels[(self.id, int(task_id))] = level

    def command_task(self, task_id, *, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(), interactive=True,
                     validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None, override_rejection=False, controller=0):
        """Fly a task: the NEW of its command, the task among the requirements it traces to (``trace`` adds to it),
        with the options a submit takes. An Activity (a fsim.Validation with ``validate_only``), or fsim.Rejected:
        "unknown_task", "task_active", or why its NEW is refused."""
        r = self._h.command_task(self.id, int(task_id), int(source), None, int(range), int(min_version),
                                 _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window,
                                           override_rejection, controller))
        return self._answer(r, self._world._task_levels.get((self.id, int(task_id)), "task"), source, validate_only, controller)

    def cancel_task(self, task_id, source=Source.POLICY, controller=0):
        """Cancel a task: its live activity ends (CANCEL, declaring ``source`` and ``controller``); one never commanded
        will not be. fsim.Rejected "unknown_task"."""
        _checked(self._h.cancel_task(self.id, int(task_id), int(source), int(controller)), self._h)

    def remove_task(self, task_id):
        """Forget a task: fsim.Rejected "unknown_task", or "task_active" while it flies."""
        reason = self._h.remove_task(self.id, int(task_id))
        if reason:
            raise Rejected(_native.reason_name(reason))
        self._world._task_levels.pop((self.id, int(task_id)), None)

    def task_status(self, task_id):
        """A task's fsim.TaskStatus, or None for one not kept."""
        t = self._h.task_status(self.id, int(task_id))
        return None if t is None else _task(t)

    def tasks(self):
        """Every task kept - the caller's and the platform's suggestions - in the order they were made."""
        return [_task(t) for t in self._h.tasks(self.id)]

    # --- Route plans (docs/flight-autonomy.md, 4.39): A-GRA's, kept by id and taken through the activation states --------
    def publish_plan(self, plan):
        """Publish a route plan (fsim.RoutePlan; A-GRA's MA_RoutePlanMT): taken where FA listens for its id - prepared
        for upload (plan_command) - A-GRA's notification CONFIRMED. fsim.Rejected: "wrong_plan_state" where FA does not
        listen for it; "invalid_parameter" for id 0, or metadata at no point or path, twice for one, or a text that is not
        printable ASCII or is longer than A-GRA's."""
        reason = self._h.publish_plan(self.id, *self._plan_native(plan))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def load_plan(self, plan):
        """FA's own route plan (fsim.RoutePlan; docs/flight-autonomy.md, 4.40), the platform's: kept uploaded and read only to
        MA, in place of any plan by its id not flying - MA prepares it for activation, activates and deactivates it, but never
        replaces or removes it ("read_only_plan"). A takeoff's or a landing's path names, in its fsim.PathMetadata, an airfield
        and a runway the vehicle keeps. fsim.Rejected: "invalid_parameter" (as publish_plan's), "unknown_airfield",
        "wrong_plan_state" (the plan it replaces flies), "plan_store_full"."""
        reason = self._h.load_plan(self.id, *self._plan_native(plan))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def _plan_native(self, plan):
        """A fsim.RoutePlan as the native publish_plan and load_plan take it (after the vehicle's id)."""
        route = plan.route
        if not isinstance(route, BatchCommand) or route.method != "submit_route":
            raise ValueError("a plan's route is a fsim.BatchCommand('submit_route', ...)")
        item, _ = route._native(self)
        points = []
        for m in plan.point_metadata:
            m = PointMetadata(**m) if isinstance(m, dict) else PointMetadata(*m)
            source = PointSource[m.source.upper()] if isinstance(m.source, str) else PointSource(m.source)
            points.append((int(m.point), int(source), bool(m.locked), bool(m.modified), str(m.remarks_name), str(m.remarks), str(m.fix_key),
                           str(m.fix_system)))
        paths = []
        for m in plan.path_metadata:
            m = PathMetadata(**m) if isinstance(m, dict) else PathMetadata(*m)
            initial = _states([RouteState() if m.initial is None else m.initial])[0]
            paths.append((int(m.path), initial, float(m.endurance_s), float(m.fuel_kg), float(m.gross_weight_kg), int(m.transition_plan),
                          int(m.airfield), int(m.runway)))
        return (int(plan.id), int(plan.version), bool(plan.for_planning_use_only), bool(plan.detailed), str(plan.remarks_name), str(plan.remarks),
                item, points, paths)

    def load_airfield(self, airfield):
        """An airfield (fsim.Airfield; A-GRA's AirfieldReportMT; docs/flight-autonomy.md, 4.40), the platform's: kept in place
        of any by its id, its revision one more, read only to MA. fsim.Rejected: "invalid_parameter" for one A-GRA's schema
        would not take - an id 0, a runway's id 0 or twice, a point off the Earth or not whole, a set of coordinates without its
        start, a runway with neither, a direction outside 0 to 2 pi, a length not above 0, an ICAO code not four capitals, a
        QNH outside 850 to 1,100 hPa; "plan_store_full" (32 kept)."""
        a = Airfield(**airfield) if isinstance(airfield, dict) else airfield

        def place(q):
            q = RunwayPoint(*q) if not isinstance(q, RunwayPoint) else q
            ref = q.altitude_reference
            ref = float(_REFERENCES["altitude_reference"][ref.upper()]) if isinstance(ref, str) else float(ref)
            return (float(q.latitude_rad), float(q.longitude_rad), float(q.altitude_m), ref)

        rows = []
        for r in a.runways:
            r = Runway(**r) if isinstance(r, dict) else r
            takeoff, landing = RunwayCoordinates(*r.takeoff), RunwayCoordinates(*r.landing)
            row = (int(r.id), float(r.direction_rad), float(r.available_length_m))
            for q in (takeoff.start, takeoff.threshold, takeoff.limit, landing.start, landing.threshold, landing.limit):
                row += place(q)
            rows.append(row)
        reason = self._h.load_airfield(self.id, int(a.id), a.icao or None, float(a.qnh_pa), rows)
        if reason:
            raise Rejected(_native.reason_name(reason))

    def validate_plan(self, plan, *, wind=None, gust_ms=None, origin=None, modify=False, parts=()):
        """Validate a route plan without flying it (A-GRA's route plan validation; docs/flight-autonomy.md, 4.41): ``plan``
        a fsim.RoutePlan - kept or not - or a kept plan's id (as uploaded last). Its route is checked as its NEW would be:
        in the ``wind`` given - (north, east) m/s, where it blows to - and its ``gust_ms``, its turns flown with them behind
        the aircraft (None: what its air data measure now); from ``origin`` - (latitude_rad, longitude_rad[,
        altitude_m]) - where the aircraft is if None; ``modify`` (A-GRA's ModifyToValidate) holding values beyond the
        aircraft's limits to them rather than refusing them; its verdict over ``parts`` (path types, fsim.PathType or their
        names: a patch's), the whole plan if none. Nothing is kept or flown. A fsim.PlanValidationResult; its validation's
        reason "unknown_plan" or "wrong_plan_state" for an id not kept or not uploaded, "invalid_parameter" for inputs
        that are not."""
        wn, we = (math.nan, math.nan) if wind is None else (float(wind[0]), float(wind[1]))
        lat, lon, alt = math.nan, math.nan, math.nan
        if origin is not None:
            lat, lon = float(origin[0]), float(origin[1])
            alt = float(origin[2]) if len(origin) > 2 else math.nan
        mask = 0
        for part in parts:
            mask |= 1 << int(PathType[part.upper()] if isinstance(part, str) else PathType(part))
        validation = (wn, we, math.nan if gust_ms is None else float(gust_ms), lat, lon, alt, bool(modify), mask)
        if isinstance(plan, RoutePlan):
            valid, result = self._h.validate_plan(self.id, *self._plan_native(plan), validation)
        else:
            valid, result = self._h.validate_stored_plan(self.id, int(plan), validation)
        return PlanValidationResult(bool(valid), _validation(result, self._h), result[5])

    def airfields(self):
        """Every airfield kept (fsim.Airfield), as loaded, in the order they were first loaded (A-GRA's query for the
        airfields, VI 1.2.6.3)."""
        return [_airfield(t) for t in self._h.airfields(self.id)]

    def airfield(self, airfield_id):
        """An airfield kept (fsim.Airfield), or None."""
        t = self._h.get_airfield(self.id, int(airfield_id))
        return None if t is None else _airfield(t)

    def plan_command(self, plan_id, command, *, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                     interactive=True, rank=None, interrupt=True, precedence_override=None, window=None, override_rejection=False, controller=0):
        """A plan activation command (fsim.PlanCommand or its name: "prepare_for_upload", "upload",
        "prepare_for_activation", "activate", "deactivate"), answered at once: a fsim.PlanCommandResult. A failure is an
        answer, not an error - ``completed`` False, why, and the plan's state. An activation is its route's NEW with the
        options a submit takes; a preparation for activation its validation with them; a deactivation cancels, as their
        source and controller, an activity not flying yet (one that flies fails "plan_executing")."""
        command = PlanCommand[command.upper()] if isinstance(command, str) else PlanCommand(command)
        t = self._h.plan_command(self.id, int(plan_id), int(command), int(source), None, int(range), int(min_version),
                                 _envelope(command_id, trace, interactive, False, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._plan_answer(t, source, controller)

    def _plan_answer(self, t, source, controller=0):
        completed, plan, command, state, reason, check = t
        activity, index, findings = None, -1, []
        if check is not None:
            index = check[5]
            if command == PlanCommand.ACTIVATE and completed:
                activity = Activity(self._world, check[2], "route", bool(check[4]), source, check[10], bool(check[15]), controller,
                                    bool(check[17]), _endurance(self._h) if check[17] else None)
            elif not completed:
                findings, _ = _findings(check, self._h)
        return PlanCommandResult(plan, PlanCommand(command), bool(completed), PlanState(state), _native.reason_name(reason), activity, index,
                                 findings)

    def abort_plan(self, plan_id, reason="restricted"):
        """FA's own deactivation of a plan (VI 1.2.5.7), the platform's: one ready for activation or activated is
        deactivated, its live activity canceled with ``reason`` (a name), its execution canceled. A fsim.PlanCommandResult:
        failed "unknown_plan", or "wrong_plan_state" in any other state."""
        return self._plan_answer(self._h.abort_plan(self.id, int(plan_id), _reason_code(reason)), Source.OVERRIDE)

    def remove_plan(self, plan_id):
        """Forget a plan: fsim.Rejected "unknown_plan", or "wrong_plan_state" while its activity is live."""
        reason = self._h.remove_plan(self.id, int(plan_id))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def plan_status(self, plan_id):
        """A route plan's fsim.PlanStatus, or None for one not kept."""
        t = self._h.plan_status(self.id, int(plan_id))
        return None if t is None else _plan_status(t)

    def plans(self):
        """Every route plan kept, in the order they were first prepared for upload (A-GRA's query for identifiers only)."""
        return [_plan_status(t) for t in self._h.plans(self.id)]

    def plan(self, plan_id):
        """A route plan's content as uploaded last, its metadata with it (A-GRA's query for a route plan): a
        fsim.RoutePlan, its route the fsim.BatchCommand that would fly it; None for one not kept, or not yet uploaded."""
        t = self._h.get_plan(self.id, int(plan_id))
        if t is None:
            return None
        pid, version, planning, detailed, remarks_name, remarks, route, points, paths = t
        return RoutePlan(pid, _setpoint(route), version, planning, detailed, remarks_name, remarks,
                         tuple(PointMetadata(p[0], PointSource(p[1]), *p[2:]) for p in points),
                         tuple(PathMetadata(q[0], RouteState(int(q[1][0]), *q[1][1:]), *q[2:]) for q in paths))

    @property
    def commanded(self):
        """What the vehicle is commanded (CommandedState, A-GRA's VehicleCommandState): what the cascade asked for in
        its last control update - an altitude, a heading, an airspeed, an attitude, rates, a throttle - with the
        acceleration it commands in north, east and down, and the altitude as its mode commanded it, in its reference.
        NaN where none is."""
        t = self._h.commanded(self.id)
        return CommandedState(Level(t[0]), *t[1:])

    def navigation_report(self):
        """A-GRA's navigation report (NavigationReport): what the vehicle flies on - fuel or a battery - how much is left
        (and its percent of capacity), its endurance at its consumption now, its playtime to its recovery point
        (set_recovery) and its contingency level."""
        t = self._h.navigation_report(self.id)
        return NavigationReport(Energy(t[0]), *t[1:12], Contingency(t[12]), bool(t[13]))

    def set_recovery(self, latitude_deg, longitude_deg, altitude_msl_m, reserve_fraction=None):
        """Where the vehicle recovers to: its navigation report's playtime counts the return (at its best-range speed,
        from its performance tables, else its cruise) and the reserve - ``reserve_fraction`` of its capacity (as it was:
        0.1 by default). Raises fsim.Error for a point off the Earth or a reserve outside [0, 1)."""
        reserve = self.navigation().reserve_fraction if reserve_fraction is None else float(reserve_fraction)
        self._h.set_navigation(self.id, True, float(latitude_deg), float(longitude_deg), float(altitude_msl_m), reserve)

    def clear_recovery(self, reserve_fraction=None):
        """No recovery point (its playtime unreported); the reserve as it was, or ``reserve_fraction``."""
        reserve = self.navigation().reserve_fraction if reserve_fraction is None else float(reserve_fraction)
        nan = float("nan")
        self._h.set_navigation(self.id, False, nan, nan, nan, reserve)

    def navigation(self):
        """Its recovery point and reserve (NavigationSettings)."""
        return NavigationSettings(*self._h.navigation(self.id))

    def set_qnh(self, qnh_pa):
        """Set its barometric altimeter to ``qnh_pa`` (A-GRA's QNH setting; docs/flight-autonomy.md, 4.20): what its
        barometric altitudes are read and flown at, from the next step. Raises fsim.Error outside 850 to 1,100 hPa
        (nothing changes). Until set, 1013.25 hPa: it reads the pressure altitude."""
        self._h.set_qnh(self.id, float(qnh_pa))

    @property
    def qnh(self):
        """What its barometric altimeter is set to, Pa."""
        return self._h.qnh(self.id)

    def state_data(self):
        """What its altimeter reads now, and the air it reads it in (fsim.StateData; A-GRA's MA_AirDataType)."""
        return StateData(*self._h.state_data(self.id))

    def performance_profile(self, mode="hsa_csa"):
        """A flight mode's performance profile at the vehicle's condition now (fsim.PerformanceProfile; docs/flight-autonomy.md,
        4.15): "hsa_csa", "waypoint_following" or "curve_following" (A-GRA profiles those three). Raises fsim.Rejected -
        "invalid_parameter" for another mode, "not_supported" or "not_implemented" for one the vehicle does not offer."""
        code = _FLIGHT_MODES.get(mode, -1) if isinstance(mode, str) else int(mode)
        reason, t = self._h.performance_profile(self.id, code)
        if reason:
            raise Rejected(_native.reason_name(reason))
        return PerformanceProfile(_native.flight_mode_name(t[0]), Energy(t[1]), bool(t[2]), bool(t[3]), bool(t[4]), *t[5:13],
                                  *([ProfilePoint(*r) for r in rows] for rows in t[13:17]),
                                  *([ProfileAcceleration(*r) for r in rows] for rows in t[17:20]),
                                  [ProfileExcessPower(*r) for r in t[20]], [ProfilePoint(*r) for r in t[21]],
                                  [ProfilePoint(*r) for r in t[22]], [ProfileOrientation(*r) for r in t[23]],
                                  [ProfileRates(*r) for r in t[24]])

    def capabilities(self):
        """What the vehicle offers: its levels and behaviours (Capability, with their Parameters and A-GRA mode)."""
        return [Capability(c[0], c[1], c[2], c[3], Level(c[4]), c[5], c[6], c[7], c[8], [Parameter(*p) for p in c[9]], c[10],
                           _native.flight_mode_name(c[11]), c[12], c[13])
                for c in self._h.capabilities(self.id)]

    def support(self, feature):
        """Whether the aircraft can do a public feature at all (fsim.SupportInfo; docs/flight-autonomy.md, 4.2): by its
        identifier ("fsim.guidance.hover", "fsim.guidance.hsa/direction/magnetic_north") or a behaviour's id. Raises
        fsim.Error for an unknown feature."""
        t = self._h.support(self.id, feature)
        return SupportInfo(t[0], Support(t[1]), _rules(t[2]), t[3], t[4], t[5], t[6])

    def support_table(self):
        """Every public feature's support on the vehicle ([fsim.SupportInfo], in fsim.SUPPORT_FEATURES' order)."""
        return [self.support(f) for f in SUPPORT_FEATURES]

    def availability(self, capability):
        """A capability's availability now, as a policy is answered (fsim.AvailabilityInfo): every reason that holds,
        the id it is about, when it is expected back, and the ranges a placard narrows now. One the vehicle does not
        offer is UNAVAILABLE with "not_supported", "not_implemented" or "unknown_capability"."""
        t = self._h.capability_status_info(self.id, capability)
        names = {}
        if t[6]:
            caps = {c.id: c for c in self.capabilities()}
            params = caps[capability].parameters if capability in caps else []
            names = {i: p.name for i, p in enumerate(params)}
        ranges = tuple((names.get(i, i), lo, hi) for i, lo, hi in t[6])
        return AvailabilityInfo(Availability(t[0]), _native.reason_name(t[1]), _reasons(t[2]), t[3], t[4], t[5], ranges)

    def capability_limits(self, capability):
        """The parameters a placard narrows now, as (name, min, max): the gear down on the ground, the flaps in above
        their speed. A NEW outside one is refused with the reason capability_status gives."""
        return self.availability(capability).ranges

    def profile_value(self, path):
        """A field of the vehicle's profile by its path as the aircraft file
        names it, in the unit its name gives: "envelope/clean/n_max",
        "envelope/clean/alpha_max_deg", "plant/roll/tau_s", "identity/class",
        "control/pid_attitude/pitch/kp"; NaN if unknown (docs/control-architecture.md, 7)."""
        return self._h.profile_value(self.id, path)

    def profile_section(self, section):
        """(version, provenance) of a profile section: version 0 if the aircraft
        has none; provenance 0 default, 1 hangar, 2 identified, 3 user, 4 derived."""
        return self._h.profile_section(self.id, section)

    def set_vehicle_default(self, mode):
        """What flies the primary axes nobody owns: VehicleDefault.NEUTRAL (or
        "neutral"; every vehicle's default) or VehicleDefault.HOLD ("hold"): the
        heading, airspeed and height each had when it was let go. Raises
        fsim.Rejected ("controller_not_axis_aware") if the hold would fly beside
        another owner through a controller that is not axis-aware."""
        mode = VehicleDefault[mode.upper()] if isinstance(mode, str) else VehicleDefault(mode)
        reason = self._h.set_vehicle_default(self.id, int(mode))
        if reason:
            raise Rejected(_native.reason_name(reason))

    @property
    def vehicle_default(self):
        return VehicleDefault(self._h.vehicle_default(self.id))

    def set_protection(self, mode):
        """Envelope protection: ProtectionMode.LIMIT ("limit"), REPORT or OFF."""
        mode = ProtectionMode[mode.upper()] if isinstance(mode, str) else ProtectionMode(mode)
        self._h.set_protection(self.id, int(mode))

    @property
    def protection(self):
        return ProtectionMode(self._h.protection(self.id))

    def envelope(self):
        """What envelope protection saw since the last call (an Envelope: the
        mode, and a LimitStatus per limit name); each call starts a new count.
        A step's exceedances make a reward term or an episode's end - the
        platform only reports them."""
        mode, rows = self._h.envelope(self.id)
        return Envelope(ProtectionMode(mode), {name: LimitStatus(*row) for name, row in zip(LIMITS, rows)})

    def capability_status(self, capability):
        """(Availability, reason) of a capability by id, e.g. "fsim.guidance.hold"."""
        availability, reason = self._h.capability_status(self.id, capability)
        return Availability(availability), _native.reason_name(reason)

    @property
    def performance(self):
        """What the vehicle can do, as its guidance plans with it (fsim.Performance): computed afresh when its loops change."""
        return Performance(*self._h.performance(self.id))

    @property
    def control_revision(self):
        """Counts every change to the grants, what is allowed, the control mode, availability and the performance: poll it."""
        return self._h.control_revision(self.id)

    @property
    def control_mode(self):
        """fsim.ControlMode: OPEN (as ever) or GRANTED (a policy needs a grant for each capability it commands)."""
        return ControlMode(self._h.control_mode(self.id))

    def set_control_mode(self, mode):
        """OPEN or GRANTED (fsim.ControlMode or its name). Switching to GRANTED ends what the policy flies without a
        grant: its activities end canceled, "not_granted"; the platform's own sources are never gated."""
        self._h.set_control_mode(self.id, int(ControlMode[mode.upper()] if isinstance(mode, str) else mode))

    def request_control(self, capability, controller=0):
        """A policy's ``controller`` (docs/flight-autonomy.md, 4.12; 0 the default policy) asks for control of a
        capability by id (A-GRA's ACQUIRE). Returns if granted; raises fsim.Rejected with "not_allowed", the reason it
        is unavailable ("restricted", "collision_avoidance", "diverged"), or "authority_held" while another
        controller holds it."""
        reason = self._h.request_control(self.id, capability, int(controller))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def release_control(self, capability, controller=0):
        """A controller lets go: its grant ends, and its live activities of the capability end canceled, "released".
        fsim.Rejected "not_granted", and nothing changes, where another controller holds it."""
        reason = self._h.release_control(self.id, capability, int(controller))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def revoke_control(self, capability, reason="revoked"):
        """The platform takes it back: the grant ends, and the policy's live activities of the capability end canceled
        with ``reason`` (a name, e.g. "revoked", "collision_avoidance", "restricted")."""
        self._h.revoke_control(self.id, capability, _reason_code(reason))

    def set_allowed(self, capability, allowed=True):
        """Whether the policy may request the capability (all may, by default); a grant for one no longer allowed is revoked."""
        self._h.set_allowed(self.id, capability, 1 if allowed else 0)

    def control_status(self, capability):
        """fsim.ControlStatus(allowed, granted, holder) of a capability by id."""
        return ControlStatus(*self._h.control_status(self.id, capability))

    def set_capability_precedence(self, capability, precedence):
        """A capability's precedence (docs/flight-autonomy.md, 4.9; lower first, 0 until set): the platform's setting,
        by which two activities of one source contest axes before their ranks are compared. What waits may start."""
        self._h.set_capability_precedence(self.id, capability, int(precedence))

    def capability_precedence(self, capability):
        """A capability's precedence by id (set_capability_precedence)."""
        return self._h.capability_precedence(self.id, capability)

    def set_availability(self, capability, availability, reason="restricted", associated=0, next_available_s=None):
        """The platform restricts a capability (fsim.Availability or its name; AVAILABLE lifts it): a policy's NEW for it
        is refused with ``reason``, and so is a request; what flies goes on. capability_status reports it, and
        availability() the id it is ``associated`` with (the vehicle avoided) and when it is expected back."""
        a = Availability[availability.upper()] if isinstance(availability, str) else availability
        nxt = float("nan") if next_available_s is None else float(next_available_s)
        self._h.set_availability_ex(self.id, capability, int(a), _reason_code(reason), int(associated), nxt)

    @property
    def active_level(self):
        return Level(self._h.active_level(self.id))

    @property
    def behavior_finished(self):
        return self._h.behavior_finished(self.id)

    def use_controller(self, level, controller_id):
        """Fly `level` with a registered controller instead of the built-in one."""
        self._h.use_controller(self.id, int(level), controller_id)

    def set_controller_parameter(self, level, name, value):
        """E.g. set_controller_parameter(Level.ATTITUDE, "roll.kp", 2.5)."""
        self._h.set_controller_parameter(self.id, int(level), name, value)

    def controller_parameter(self, level, name):
        """A parameter of the controller at `level` as the vehicle flies it: its
        default, the aircraft's own setting, or the last one set."""
        return self._h.controller_parameter(self.id, int(level), name)

    # --- effects --------------------------------------------------------------------
    def add_effect(self, effect, **params):
        """A built-in effect by id - "gaussian_sensor_noise", "sensor_latency",
        "constant_force", "wind_gusts", "gnss_degradation" - with its parameters."""
        self._h.add_effect(self.id, effect, params or None)

    def clear_effects(self):
        self._h.clear_effects(self.id)

    # --- communication -------------------------------------------------------------
    def send(self, to, data, channel=0, format=0):
        """Send bytes to a vehicle, a node address, or BROADCAST."""
        recipient = to.id if isinstance(to, Vehicle) else int(to)
        self._h.comm_send(self.id, recipient, channel, format, data)

    def inbox(self):
        """Messages delivered to this vehicle since the last step."""
        return [Message(*m) for m in self._h.comm_inbox(self.id)]

    def attach_protocol(self, protocol, **params):
        """E.g. attach_protocol("beacon", period_s=1.0, channel=1)."""
        self._h.comm_attach_protocol(self.id, protocol, params or None)

    # --- lifecycle -------------------------------------------------------------------
    def reset(self, **initial):
        """Back to its initial conditions, or to new ones (latitude_deg,
        longitude_deg, altitude_msl_m, heading_deg, pitch_deg, roll_deg,
        airspeed_ms, on_ground)."""
        self._h.reset_vehicle(self.id, _options(**initial) or None)

    def remove(self):
        self._h.remove_vehicle(self.id)
        self._state = self._sensed = None
        self._world._vehicles.pop(self.id, None)


class World:
    """A world of vehicles, stepped in lockstep (fsim_world; fsim::World in C++).

    ``World(name, **options)``: options are those of fsim_world_options -
    dt, frame_skip, workers, pin_workers, seed, capacity, publish,
    publish_interval_s, jsbsim_root, terrain, terrain_url, terrain_zoom,
    record_path, record_interval_s. What is not given keeps the platform's
    default. Viewers (flightsim-viewer.exe) attach by name.

    ``world.step(n=1)`` is the native call itself: no Python in between.
    """

    def __init__(self, name=None, *, dt=None, frame_skip=None, workers=None, pin_workers=None, seed=None,
                 capacity=None, publish=None, publish_interval_s=None, jsbsim_root=None, terrain=None,
                 terrain_url=None, terrain_zoom=None, record_path=None, record_interval_s=None, _handle=None):
        if _handle is not None:
            self._h = _handle
        else:
            self._h = _native.World(_options(
                name=name, dt=dt, frame_skip=frame_skip, workers=workers, pin_workers=pin_workers, seed=seed,
                capacity=capacity, publish=publish, publish_interval_s=publish_interval_s, jsbsim_root=jsbsim_root,
                terrain=terrain, terrain_url=terrain_url, terrain_zoom=terrain_zoom, record_path=record_path,
                record_interval_s=record_interval_s))
        self.name = name or "default"
        # Held weakly: a Vehicle holds its World, so a strong cache would make
        # a cycle, and a world - its threads, its flight models - would live on
        # until the garbage collector found it rather than go when its last
        # reference did.
        self._vehicles = weakref.WeakValueDictionary()
        self._task_levels = {}  # (vehicle, task id) -> the level or mode its command is: its Activity's
        self.step = self._h.step  # step(n=1)

    @classmethod
    def from_scenario(cls, scenario, **overrides):
        """A world as a scenario file describes it, with its vehicles."""
        if not isinstance(scenario, Scenario):
            scenario = Scenario(scenario)
        options = scenario.world_options
        options.update(overrides)
        world = cls(**options)
        world.apply_scenario(scenario)
        return world

    # --- time ----------------------------------------------------------------------
    @property
    def time(self):
        """Simulation seconds since creation."""
        return self._h.time()

    @property
    def step_seconds(self):
        """dt * frame_skip: what one step() advances."""
        return self._h.info()["step_seconds"]

    @property
    def vehicle_steps(self):
        """FDM vehicle-steps so far (for throughput)."""
        return self._h.info()["vehicle_steps"]

    @property
    def published(self):
        """Viewers can see this world."""
        return self._h.info()["published"]

    # --- vehicles --------------------------------------------------------------------
    def create_vehicle(self, name=None, type="jsbsim:c172x", *, latitude_deg=None, longitude_deg=None,
                       altitude_msl_m=None, heading_deg=None, pitch_deg=None, roll_deg=None, airspeed_ms=None,
                       on_ground=None, model=None, control_divider=None):
        """Load a vehicle: "<flight model>:<aircraft>", e.g. "jsbsim:f16". Raises
        fsim.Error if it cannot be loaded (unknown aircraft, duplicate name)."""
        vid = self._h.create_vehicle(_options(
            name=name, type=type, latitude_deg=latitude_deg, longitude_deg=longitude_deg,
            altitude_msl_m=altitude_msl_m, heading_deg=heading_deg, pitch_deg=pitch_deg, roll_deg=roll_deg,
            airspeed_ms=airspeed_ms, on_ground=on_ground, model=model, control_divider=control_divider))
        return self._vehicle(vid)

    def _vehicle(self, vid):
        v = self._vehicles.get(vid)
        if v is None:
            v = self._vehicles[vid] = Vehicle(self, vid)
        return v

    def vehicle(self, key):
        """A vehicle by id or name."""
        vid = self._h.find_vehicle(key) if isinstance(key, str) else int(key)
        if not vid or not self._h.vehicle_info(vid)[0]:
            raise KeyError(key)
        return self._vehicle(vid)

    def vehicle_ids(self):
        """Every vehicle's id, as a uint32 array - ready for the batched calls."""
        return np.frombuffer(self._h.vehicle_ids(), dtype=np.uint32).copy()

    def vehicles(self):
        return [self._vehicle(int(i)) for i in np.frombuffer(self._h.vehicle_ids(), dtype=np.uint32)]

    def __len__(self):
        return self._h.info()["vehicle_count"]

    def __iter__(self):
        return iter(self.vehicles())

    def __contains__(self, key):
        try:
            self.vehicle(key)
            return True
        except KeyError:
            return False

    @staticmethod
    def ids(vehicles):
        """uint32 ids for the batched calls, from Vehicles, ids or an id array.
        Build it once and reuse it: the batched calls then convert nothing."""
        if isinstance(vehicles, np.ndarray) and vehicles.dtype == np.uint32 and vehicles.flags.c_contiguous:
            return vehicles
        if isinstance(vehicles, (Vehicle, int, np.integer)):
            vehicles = (vehicles,)
        return np.fromiter((v.id if isinstance(v, Vehicle) else int(v) for v in vehicles), dtype=np.uint32)

    # --- batched calls -----------------------------------------------------------------
    def states(self, vehicles=None, *, sensed=False, out=None):
        """Every vehicle's state (or those given) in one structured array
        (fsim.vehicle_state_dtype), copied in one call: ``s["altitude_msl_m"]``
        is a column over all of them. Pass ``out`` to fill an array you keep."""
        if out is not None and vehicles is not None:
            try:  # an id array and an array you keep: nothing to do but the call
                self._h.gather_states(vehicles, out, sensed)
                return out
            except TypeError:
                pass
        ids = self.vehicle_ids() if vehicles is None else self.ids(vehicles)
        if out is None:
            out = np.empty(len(ids), dtype=vehicle_state_dtype)
        self._h.gather_states(ids, out, 1 if sensed else 0)
        return out

    def command(self, level, vehicles, values):
        """Command many vehicles at one level in one call. ``values`` has a row
        per vehicle and a column per field of the level (fsim.COMMAND_FIELDS),
        float64; HOLD (NaN) in a field works as in the per-vehicle commands.
        A C-contiguous float64 array and a uint32 id array (World.ids) pass
        straight through; anything else is converted first, torch tensors on
        any device included."""
        try:  # a uint32 id array and a float64 array: nothing to do but the call
            self._h.command_batch(level, vehicles, values)
        except TypeError:
            self._h.command_batch(int(level), self.ids(vehicles), as_array(values, np.float64))

    def activity(self, activity):
        """An activity's record (ActivityInfo) by Activity or id, or None if its vehicle no longer remembers it."""
        t = self._h.activity_info(activity.id if isinstance(activity, Activity) else int(activity))
        return None if t is None else _info(t)

    def last_command_details(self):
        """Everything the checks found for the last NEW, validation or UPDATE made through this World
        (docs/flight-autonomy.md, 4.8): (findings, adjustments) - every reason it cannot be flown as asked
        (fsim.Finding), the answer's the first, and every value flown other than asked (fsim.Adjustment)."""
        findings = [Finding(_native.reason_name(f[0]), f[1], _native.constraint_name(f[2]), None if math.isnan(f[3]) else (f[3], f[4]), f[5], f[6])
                    for f in self._h.last_findings()]
        adjustments = [Adjustment(*a[:2], _native.constraint_name(a[2]), a[3], a[4]) for a in self._h.last_adjustments()]
        return findings, adjustments

    def activity_progress(self, activity):
        """An activity's progress (ActivityProgress) by Activity or id, or None if its vehicle no longer remembers it."""
        t = self._h.activity_progress(activity.id if isinstance(activity, Activity) else int(activity))
        return None if t is None else ActivityProgress(*t)

    def activity_setpoint(self, activity):
        """What a live activity (by Activity or id) flies now, or waits to fly, as the fsim.BatchCommand that would
        command it (Activity.setpoint); None for one not live."""
        t = self._h.activity_setpoint(activity.id if isinstance(activity, Activity) else int(activity))
        return None if t is None else _setpoint(t)

    def end_points(self, activity, max=16):
        """Where a live activity (by Activity or id) flies to, ``max`` points at most (Activity.end_points)."""
        rows = self._h.activity_end_points(activity.id if isinstance(activity, Activity) else int(activity), int(max))
        return [EndPoint(EndPointKind(p[0]), *p[1:]) for p in rows]

    def update(self, activities, values):
        """UPDATE many activities of one level in one call: their ids (a uint64
        array, or Activities) and a float64 row per activity in the level's
        field order - COMMAND_FIELDS', or SETPOINT_FIELDS' with the
        rotorcraft's. Raises fsim.Error naming the first one refused."""
        if not (isinstance(activities, np.ndarray) and activities.dtype == np.uint64 and activities.flags.c_contiguous):
            activities = np.fromiter((a.id if isinstance(a, Activity) else int(a) for a in activities), dtype=np.uint64)
        rows = np.ascontiguousarray(as_array(values, np.float64))
        stride = rows.shape[-1] if rows.ndim > 1 else (rows.size // max(len(activities), 1))
        if stride in _FULL_WIDTHS:  # a row with the rotorcraft's fields: as wide as it is
            self._h.activity_update_batch(activities, rows, int(stride), int(stride))
        else:
            self._h.activity_update_batch(activities, rows, int(stride))

    @property
    def magnetic_year(self):
        """The date the World Magnetic Model is read at for this world now (docs/flight-autonomy.md, 4.22): its UTC's
        decimal year, held within 2025.0 to 2030.0 - a world whose clock was never set reads 2025.0."""
        return self._h.magnetic_year()

    # --- the terrain -----------------------------------------------------------------
    def terrain(self, latitude_rad, longitude_rad):
        """The ground's height above the WGS-84 ellipsoid (m) at a place - the
        physics' own, which commanded paths are checked against
        (docs/flight-autonomy.md, 4.19; VI 1.2.6.9, A-GRA's elevation request) -
        or None where it has no data (a terrain tile it cannot load). Given
        sequences of latitudes and longitudes, a list."""
        if np.ndim(latitude_rad) == 0 and np.ndim(longitude_rad) == 0:
            return self._h.terrain([float(latitude_rad)], [float(longitude_rad)])[0]
        return self._h.terrain([float(x) for x in latitude_rad], [float(x) for x in longitude_rad])

    # --- reference frames ------------------------------------------------------------
    def create_frame(self, origin=FrameOrigin.FIXED, *, latitude_rad=0.0, longitude_rad=0.0, altitude_msl_m=0.0, yaw_rad=0.0,
                     pitch_rad=0.0, roll_rad=0.0, north_ms=0.0, east_ms=0.0, down_ms=0.0, time_s=0.0, vehicle=None):
        """A reference frame (A-GRA's ReferenceFrame; docs/flight-autonomy.md, 4.21), its id: ``origin`` (fsim.FrameOrigin or its
        name) "fixed" at a place and orientation; "moving" from that place at ``time_s`` (World.time) at a constant velocity;
        "vehicle", following ``vehicle`` (a Vehicle or its id). Raises fsim.Error where refused (a value not finite, a
        latitude off the Earth, an unknown vehicle)."""
        origin = FrameOrigin[origin.upper()] if isinstance(origin, str) else FrameOrigin(origin)
        vid = 0 if vehicle is None else int(getattr(vehicle, "id", vehicle))
        return self._h.create_frame(int(origin), vid, float(latitude_rad), float(longitude_rad), float(altitude_msl_m), float(yaw_rad),
                                    float(pitch_rad), float(roll_rad), float(north_ms), float(east_ms), float(down_ms), float(time_s))

    def remove_frame(self, frame):
        """Remove a frame; False if there was none."""
        return self._h.remove_frame(int(frame))

    def set_op_point(self, point):
        """Keep an operational point (fsim.OpPoint; docs/flight-autonomy.md, 4.42) in place of any by its id, its revision one
        more; its codes by name or member. fsim.Rejected("invalid_parameter") for a malformed one: id 0; neither a place nor
        a frame, or both; a latitude off the Earth, a value not finite; a frame the world does not have, or offsets without
        one; an altitude reference without its altitude; a window given one way alone, or beyond half a turn."""
        p = point if isinstance(point, OpPoint) else OpPoint(**point) if isinstance(point, dict) else OpPoint(*point)
        codes = {"altitude_reference": AltitudeReference, "frame_rotation": FrameRotation, "frame_offsets": FrameOffsets}
        values = [codes[k][v.upper()] if k in codes and isinstance(v, str) else v for k, v in zip(OpPoint._fields, p)]
        reason = self._h.set_op_point(int(values[0]), *(float(v) for v in values[1:13]))
        if reason:
            raise Rejected(_native.reason_name(reason))

    def op_points(self):
        """The operational points kept (fsim.OpPoint), by id."""
        return [_op_point(t) for t in self._h.op_points()]

    def op_point(self, point_id):
        """An operational point kept (fsim.OpPoint), or None."""
        t = self._h.op_point(int(point_id))
        return None if t is None else _op_point(t)

    def remove_op_point(self, point_id):
        """Forget an operational point; False if there was none."""
        return self._h.remove_op_point(int(point_id))

    def frame_point(self, frame, x=0.0, y=0.0, z=0.0, *, rotation=FrameRotation.UNROTATED, offsets=FrameOffsets.CARTESIAN, time_s=None):
        """Where a point in a frame is (A-GRA's relative point): ``x``, ``y``, ``z`` metres (z down) turned as ``rotation``
        (fsim.FrameRotation or its name) says and laid out as ``offsets`` (fsim.FrameOffsets) says, at ``time_s`` (None: now;
        a vehicle's frame carried on at its velocity to another time). (latitude_rad, longitude_rad, altitude_msl_m), or None
        for an unknown frame or one whose vehicle is gone."""
        rotation = FrameRotation[rotation.upper()] if isinstance(rotation, str) else FrameRotation(rotation)
        offsets = FrameOffsets[offsets.upper()] if isinstance(offsets, str) else FrameOffsets(offsets)
        t = math.nan if time_s is None else float(time_s)
        return self._h.frame_point(int(frame), int(rotation), int(offsets), float(x), float(y), float(z), t)

    # --- environment ---------------------------------------------------------------------
    @property
    def environment(self):
        """The environment now, as a dict (time, atmosphere, wind, weather)."""
        return self._h.get_environment()

    def set_environment(self, **changes):
        """Change any of: epoch_utc_seconds, time_factor, temperature_sl_k,
        pressure_sl_pa, humidity, wind_direction_deg, wind_speed_ms,
        wind_gust_ms, turbulence, visibility_m, cloud_base_m, cloud_cover,
        precipitation. Takes effect at the next step."""
        self._h.set_environment(changes)

    def set_wind(self, direction_deg, speed_ms, gust_ms=0.0, turbulence=0.0):
        """Wind blowing FROM direction_deg (true)."""
        self.set_environment(wind_direction_deg=direction_deg, wind_speed_ms=speed_ms, wind_gust_ms=gust_ms,
                             turbulence=turbulence)

    # --- effects and communication ---------------------------------------------------------
    def add_effect(self, effect, **params):
        """An effect on every vehicle, present and future."""
        self._h.add_effect(0, effect, params or None)

    def set_comm_medium(self, medium, **params):
        """"ideal", or "link" with range_m, latency_s, jitter_s, loss_probability."""
        self._h.comm_set_medium(medium, params or None)

    def create_comm_node(self, address):
        """A node that is not a vehicle (a ground station), by address."""
        self._h.comm_create_node(address)

    def send(self, sender, recipient, data, channel=0, format=0):
        s = sender.id if isinstance(sender, Vehicle) else int(sender)
        r = recipient.id if isinstance(recipient, Vehicle) else int(recipient)
        self._h.comm_send(s, r, channel, format, data)

    def inbox(self, node):
        n = node.id if isinstance(node, Vehicle) else int(node)
        return [Message(*m) for m in self._h.comm_inbox(n)]

    def attach_udp_bridge(self, node, local_port, remote_host, remote_port):
        """Messages to `node` go out as UDP datagrams; datagrams arriving on
        local_port come in as messages from it (format in fsim/Comm.h)."""
        n = node.id if isinstance(node, Vehicle) else int(node)
        self._h.comm_attach_udp_bridge(n, local_port, remote_host, remote_port)

    # --- scenarios and lifetime -----------------------------------------------------------------
    def apply_scenario(self, scenario):
        """Apply a scenario's environment, effects and vehicles; returns the vehicles."""
        if not isinstance(scenario, Scenario):
            scenario = Scenario(scenario)
        return [self._vehicle(int(i)) for i in np.frombuffer(self._h.apply_scenario(scenario._h), dtype=np.uint32)]

    def close(self):
        """Release the world. Its memory lives on only while arrays or states
        viewing it do."""
        self._vehicles.clear()
        self._h = None
        self.step = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self):
        return "World(%r, %d vehicles, t=%.2f s)" % (self.name, len(self), self.time) if self._h else "World(closed)"


class Scenario:
    """A scenario file (docs/sdk/scenarios.md): ``Scenario(path)`` or
    ``Scenario(json=text)``. Build a world from it with World.from_scenario,
    or apply it to one you have with World.apply_scenario."""

    def __init__(self, path=None, *, json=None, source="inline"):
        if path is None and json is None:
            raise TypeError("Scenario(path) or Scenario(json=...)")
        self._h = _native.Scenario(str(path)) if path is not None else _native.Scenario(None, json, source)

    @property
    def world_options(self):
        """The "world" section, as World keyword arguments."""
        o = self._h.world_options()
        o.pop("struct_size", None)
        for flag in ("pin_workers", "publish", "terrain"):
            if flag in o:
                o[flag] = bool(o[flag])
        return {k: v for k, v in o.items() if v is not None}

    @property
    def vehicle_count(self):
        """Vehicle instances it creates (counting "count")."""
        return self._h.vehicle_count()
