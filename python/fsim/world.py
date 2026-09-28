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
    "course_rad heading_rad altitude_msl_m speed_ms speed_reference")
ActivityProgress.__doc__ = ("How far an activity has got and what it commands, as its behaviour reported it after the last "
                            "world step (docs/vehicle-interface.md, 5.3): the waypoint, curve segment or pattern leg flown now "
                            "(of ``segments``; 0: nothing segmented), laps, percent of the whole and of the segment, the "
                            "distance and time to the end, the cross-track distance (+ right of the path), and the course, "
                            "heading, altitude and speed it asks for (``speed_reference``: 0 true airspeed, 1 calibrated, 2 "
                            "ground speed, 3 Mach). NaN where it says nothing.")

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


class TurnType(enum.IntEnum):
    """How a route passes a waypoint (A-GRA's TurnType): a fly-by turn begins before it, on a circle tangent to
    both legs; a fly-over point is passed, then the next leg intercepted."""
    FLY_BY = 0
    FLY_OVER = 1


class Projection(enum.IntEnum):
    """What a route's legs are on the Earth."""
    GREAT_CIRCLE = 0
    RHUMB = 1


class EndBehavior(enum.IntEnum):
    """What a route or curve does after its end: on along the last leg or course (its altitude and speed), or
    loiter there - a wing orbits the point, a rotorcraft stops and hovers over it."""
    CONTINUE = 0
    LOITER = 1


class PatternKind(enum.IntEnum):
    """A loiter pattern (A-GRA's LOITER): an orbit round its centre; a racetrack, two half circles joined by legs,
    the inbound one ending at the fix; a figure-eight, two circles meeting at the centre; ATC's hold, a racetrack
    on the fix with a minute's legs, entered direct to the fix."""
    ORBIT = 0
    RACETRACK = 1
    FIGURE_EIGHT = 2
    HOLD = 3


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


class HoldContext(enum.IntEnum):
    """A hold's operational context (A-GRA's MA_HoldContextEnum): the defaults it implies are ATC's for every one."""
    ADMIN = 0
    TACTICAL = 1
    ATC = 2


#: The Vehicle Interface's modes (docs/vehicle-interface.md): their fixed-size setpoints' fields, in order. HOLD leaves
#: one out: a NEW continues what a live hsa commanded (else what the aircraft flies now) and takes a route's, a
#: pattern's or a curve's default; an UPDATE keeps it.
MODE_KINDS = ("hsa", "route", "pattern", "curve")
MODE_FIELDS = {"hsa": ("heading_rad", "course_rad", "speed", "speed_reference", "altitude_m", "altitude_reference", "speed_optimization",
                       "direction_reference"),
               "route": ("projection", "repeat", "end", "start"),
               "pattern": ("pattern", "latitude_rad", "longitude_rad", "altitude_m", "altitude_reference", "radius_m", "clockwise",
                           "course_rad", "leg_m", "speed", "speed_reference", "duration_s", "speed_optimization", "direction_reference",
                           "heading_rad", "leg_s", "bank_rad", "orbits", "latitude2_rad", "longitude2_rad", "radius2_m", "entry_latitude_rad",
                           "entry_longitude_rad", "exit_latitude_rad", "exit_longitude_rad", "turn_rate_rad_s", "turn_type", "hold_entry",
                           "hold_context"),
               "curve": ("latitude_rad", "longitude_rad", "altitude_m", "speed_min_ms", "speed_max_ms", "duration_s", "end", "append")}
MODE_DEFAULTS = {"hsa": (HOLD,) * 8, "route": (HOLD,) * 4, "pattern": (HOLD,) * 29, "curve": (HOLD,) * 8}
_REFERENCES = {"speed_reference": SpeedReference, "altitude_reference": AltitudeReference, "speed_optimization": SpeedOptimization,
               "direction_reference": DirectionReference,
               "projection": Projection, "end": EndBehavior,
               "turn": TurnType, "pattern": PatternKind, "turn_type": HoldTurn, "hold_entry": HoldEntry, "hold_context": HoldContext}

Waypoint = collections.namedtuple(
    "Waypoint", "latitude_rad longitude_rad altitude_m altitude_reference speed speed_reference turn max_bank_rad climb_rate_ms id",
    defaults=(HOLD, HOLD, HOLD, HOLD, 0, HOLD, HOLD, 0))
Waypoint.__doc__ = ("One waypoint of a route (A-GRA's), and the segment that ends at it: reached at ``altitude_m`` above "
                    "``altitude_reference`` along a straight profile (or climbing at ``climb_rate_ms``, then level), flown at "
                    "``speed`` in ``speed_reference``, passed by ``turn`` (fsim.TurnType: 'fly_by', 'fly_over') with "
                    "``max_bank_rad`` for its turn; ``id`` comes back in the progress. HOLD (the default) continues the "
                    "previous point's; the first point's is the aircraft's own now, and a rotorcraft given no speed flies its "
                    "cruise speed over the ground. References may be given by name.")


BezierSegment = collections.namedtuple("BezierSegment", "north east down")
BezierSegment.__doc__ = ("One segment of a curve (A-GRA's): a quintic Bezier by its six control points - ``north``, ``east`` "
                         "and ``down``, six metres each, from the curve's reference - with weights 1 and the clamped knots "
                         "[0,0,0,0,0,0,1,1,1,1,1,1]. Each segment starts where the one before ends (within a metre).")


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


def _waypoints(points):
    """Waypoints (fsim.Waypoint, dicts of its fields, or rows in its order) as the native rows."""
    rows = []
    for p in points:
        if isinstance(p, dict):
            p = Waypoint(**p)
        elif not isinstance(p, Waypoint):
            p = Waypoint(*p)
        values = [_REFERENCES[k][v.upper()] if isinstance(v, str) and k in _REFERENCES else v for k, v in zip(Waypoint._fields, p)]
        rows.append(tuple(float(v) for v in values[:9]) + (int(values[9]),))
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

    def update_route(self, waypoints=None, **options):
        """UPDATE of a route: new ``waypoints`` (None: those it has) and the options given (the others kept);
        checked as a NEW's, then flown afresh from its start, from where the aircraft is. Returns True if a value
        was clamped; raises fsim.Rejected (``index`` the waypoint at fault)."""
        rows = [] if waypoints is None else _waypoints(waypoints)
        h = self.world._h
        return bool(_checked(h.activity_update_route(self.id, _row("route", (), options), rows, int(self.source), self.controller), h)[4])

    def update_curve(self, segments=None, **options):
        """UPDATE of a curve: new ``segments`` (fsim.BezierSegment; None: those it has) and the options given (the
        others kept) - with ``append=1`` the segments go after its end, from the same reference; else they are a new
        curve, flown afresh. Returns True if a value was clamped; raises fsim.Rejected (``index`` the segment at
        fault; ``section`` where a segment is too tight)."""
        rows = [] if segments is None else _segments(segments)
        h = self.world._h
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
    "submit_hsa", "submit_pattern", "submit_route", "submit_curve") and the arguments it takes."""

    __slots__ = ("method", "args", "kwargs")
    _KINDS = {"submit": 0, "submit_behavior": 1, "submit_support": 2, "submit_hsa": 3, "submit_pattern": 3, "submit_route": 4, "submit_curve": 5}

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
        if self.method in ("submit_hsa", "submit_pattern"):
            mode = self.method[len("submit_"):]
            return (kind, MODE_KINDS.index(mode), _row(mode, args, k), None, None, None, options), (mode, source, validate, controller)
        if self.method == "submit_route":
            waypoints = args.pop(0) if args else k.pop("waypoints")
            route = {"projection": k.pop("projection", Projection.GREAT_CIRCLE), "repeat": 1.0 if k.pop("repeat", False) else 0.0,
                     "end": k.pop("end", EndBehavior.CONTINUE), "start": k.pop("start", 0)}
            return (kind, 0, _row("route", (), route), None, _waypoints(waypoints), None, options), ("route", source, validate, controller)
        segments = args.pop(0) if args else k.pop("segments")
        return (kind, 0, _row("curve", (), k), None, None, _segments(segments), options), ("curve", source, validate, controller)


def _setpoint(t):
    """A native setpoint (activity_setpoint) as the fsim.BatchCommand that would command it."""
    kind, code, fields, behavior, waypoints, segments = t
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
        return BatchCommand("submit_route", [Waypoint(*w) for w in waypoints], projection=route["projection"], repeat=route["repeat"] == 1.0,
                            end=route["end"], start=route["start"])
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

    def submit_route(self, waypoints, *, projection=Projection.GREAT_CIRCLE, repeat=False, end=EndBehavior.CONTINUE, start=0,
                     source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(), interactive=True, validate_only=False,
                     rank=None, interrupt=True, precedence_override=None, window=None, override_rejection=False, controller=0):
        """NEW for fsim.guidance.route, A-GRA's waypoint following (docs/vehicle-interface.md, 4.5): fly
        ``waypoints`` (fsim.Waypoint, dicts of its fields, or rows in its order; at most 256) as legs - great circles
        or rhumb lines (``projection``: fsim.Projection or "great_circle", "rhumb") - from where the aircraft is to
        the point ``start``, with fly-by turns or fly-over points; again from the first point if ``repeat``; after
        the last point, ``end`` (fsim.EndBehavior: "continue", "loiter"). An Activity that completes after the last
        point (unless it repeats), whose progress names the point flown to; ``update_route`` gives it new waypoints
        or options. fsim.Rejected if refused: ``index`` names the waypoint, ``constraint`` the limit it breaks, and
        ``findings`` every waypoint at fault. The command envelope as submit's."""
        options = {"projection": projection, "repeat": 1.0 if repeat else 0.0, "end": end, "start": start}
        r = self._h.submit_route(self.id, _row("route", (), options), _waypoints(waypoints), int(source), None, int(range), int(min_version),
                                 _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, "route", source, validate_only, controller)

    def submit_pattern(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                       interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                       override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.pattern, A-GRA's loiter (docs/vehicle-interface.md, 4.6): ``pattern``
        (fsim.PatternKind or "orbit", "racetrack", "figure_eight", "hold") round ``latitude_rad``, ``longitude_rad``
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
        times are the command's ``window``. Given more than one way to give the course, the legs or the radius, the
        first flies; an update of any one replaces them all. An Activity whose ``update(**fields)`` changes only what it
        gives; fsim.Rejected if refused. The command envelope as submit's."""
        r = self._h.submit_mode(self.id, MODE_KINDS.index("pattern"), _row("pattern", values, fields), int(source), None, int(range),
                                int(min_version), _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
        return self._answer(r, "pattern", source, validate_only, controller)

    def submit_curve(self, segments, *, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                     interactive=True, validate_only=False, rank=None, interrupt=True, precedence_override=None, window=None,
                     override_rejection=False, controller=0, **fields):
        """NEW for fsim.guidance.curve, A-GRA's curve following (docs/vehicle-interface.md, 4.7): fly ``segments``
        (fsim.BezierSegment, dicts of its fields, or (north, east, down) triples; 1 to 10), quintic Beziers in metres
        from ``latitude_rad``, ``longitude_rad``, ``altitude_m`` (left out: the aircraft now). Within the ground speeds
        ``speed_min_ms`` to ``speed_max_ms`` (a wing holds its airspeed within them, a rotorcraft flies its ground
        speed), or so as to take ``duration_s``; left out, as it flies now. After its end, ``end``
        (fsim.EndBehavior: "continue", "loiter"). An Activity that completes at its end, whose progress names the
        segment flown; ``append`` adds segments while it flies, ``update_curve`` gives it a new curve or options.
        fsim.Rejected if refused: ``index`` names the segment, ``section`` where it is too tight, ``findings``
        every segment at fault. The command envelope as submit's."""
        r = self._h.submit_curve(self.id, _row("curve", (), fields), _segments(segments), int(source), None, int(range), int(min_version),
                                 _envelope(command_id, trace, interactive, validate_only, rank, interrupt, precedence_override, window, override_rejection,
                                           controller))
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
