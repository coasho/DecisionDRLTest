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


class RequirementKind(enum.IntEnum):
    """What a requirement a command comes from is (A-GRA's Traceability; docs/flight-autonomy.md, 4.8)."""
    NONE = 0
    EFFECT = 1
    ACTION = 2
    TASK = 3
    COMMAND = 4


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

ControlStatus = collections.namedtuple("ControlStatus", "allowed granted")
ControlStatus.__doc__ = "A capability's standing with a vehicle's policy: whether it may be requested, whether a grant is held."


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
    other than asked (fsim.Adjustment)."""

    def __init__(self, reason, other=0, index=-1, constraint="none", section=None, *, description="", associated=0, command_id=0,
                 findings=(), adjustments=()):
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


ActivityInfo = collections.namedtuple(
    "ActivityInfo", "id vehicle capability source axes state reason by constraints constraints_seen start_time end_time "
    "command_id interactive trace")
ActivityInfo.__doc__ = ("An activity's record: its capability (an index into Vehicle.capabilities()), who commanded it, "
                        "its state and why it ended, flags (1 saturated, 8 clamped, ...) and when it ran; the command it came "
                        "from (docs/flight-autonomy.md, 4.8): its id, whether it takes activity commands, and the requirements "
                        "it traces to, as (fsim.RequirementKind, id) pairs.")

Finding = collections.namedtuple("Finding", "reason index constraint section associated description")
Finding.__doc__ = ("One reason a command cannot be flown as asked (A-GRA's ValidationResult; docs/flight-autonomy.md, 4.8): the "
                   "reason, the field, route point or curve segment (-1 none), the performance limit it breaks, a curve "
                   "segment's section (from, to) or None, an id it is about, and the reason in words.")

Adjustment = collections.namedtuple("Adjustment", "index field constraint requested adjusted")
Adjustment.__doc__ = ("A value a command is flown with other than asked, held to what the aircraft can do: the command's field, a "
                      "route point or curve segment; a route point's field (fsim.Waypoint's order) or -1; the limit; what was "
                      "asked and what is flown (NaN where it is not one number: a fly-by turn flown smaller).")

Validation = collections.namedtuple("Validation", "valid reason clamped command_id findings adjustments")
Validation.__doc__ = ("A validation's answer (validate_only=True; A-GRA's FLIGHT_COMMAND_VALID): whether a NEW would be accepted, "
                      "why not, whether a value would be clamped, the command's id, every finding and every value it would be "
                      "flown with other than asked. Nothing flies.")

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

CommandedState = collections.namedtuple(
    "CommandedState", "top_level latitude_rad longitude_rad altitude_msl_m heading_rad turn_rate_rad_s airspeed_ms "
    "vertical_speed_ms north_ms east_ms roll_rad pitch_rad load_factor_g roll_rate_rad_s pitch_rate_rad_s yaw_rate_rad_s throttle")
CommandedState.__doc__ = ("What the cascade asked for in its last control update, level by level (A-GRA's commanded state): "
                          "the position level's point, the velocity level's heading, turn rate, airspeed, vertical speed and "
                          "velocity over the ground, the attitude level's roll and pitch, the acceleration level's load factor "
                          "and rates, the throttle. NaN where no level set it.")


def _info(t):
    return ActivityInfo(t[0], t[1], t[2], Source(t[3]), t[4], ActivityState(t[5]), _native.reason_name(t[6]), t[7], t[8], t[9],
                        t[10], t[11], t[12], t[13], tuple((RequirementKind(k), i) for k, i in t[14]))


def _findings(result, h):
    """The last command's findings and adjustments (docs/flight-autonomy.md, 4.8), read only where there are some."""
    findings = [Finding(_native.reason_name(f[0]), f[1], _native.constraint_name(f[2]), None if math.isnan(f[3]) else (f[3], f[4]), f[5], f[6])
                for f in h.last_findings()] if h is not None and result[13] else []
    adjustments = [Adjustment(*a[:2], _native.constraint_name(a[2]), a[3], a[4]) for a in h.last_adjustments()] if h is not None and result[14] else []
    return findings, adjustments


def _rejected(result, h=None):
    """A refused command's result tuple as the Rejected it raises."""
    section = None if math.isnan(result[7]) else (result[7], result[8])
    findings, adjustments = _findings(result, h)
    return Rejected(_native.reason_name(result[1]), result[3], result[5], _native.constraint_name(result[6]), section, description=result[12],
                    associated=result[11], command_id=result[10], findings=findings, adjustments=adjustments)


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
    return Validation(result[0] == 3, _native.reason_name(result[1]), bool(result[4]), result[10], findings, adjustments)


def _envelope(command_id, trace, interactive, validate_only):
    """The command envelope's native form (docs/flight-autonomy.md, 4.8), or None where it is all left out."""
    if not command_id and not trace and interactive and not validate_only:
        return None
    pairs = []
    for kind, rid in trace:
        kind = RequirementKind[kind.upper()] if isinstance(kind, str) else RequirementKind(kind)
        pairs.append((int(kind), int(rid)))
    return (int(command_id), tuple(pairs), bool(interactive), bool(validate_only))


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
    WGS-84 ellipsoid, so MSL and ELLIPSOID are one; ABOVE_GROUND follows the terrain under the aircraft."""
    MSL = 0
    ABOVE_GROUND = 1
    ELLIPSOID = 2


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


#: The Vehicle Interface's modes (docs/vehicle-interface.md): their fixed-size setpoints' fields, in order. HOLD leaves
#: one out: a NEW continues what a live hsa commanded (else what the aircraft flies now) and takes a route's, a
#: pattern's or a curve's default; an UPDATE keeps it.
MODE_KINDS = ("hsa", "route", "pattern", "curve")
MODE_FIELDS = {"hsa": ("heading_rad", "course_rad", "speed", "speed_reference", "altitude_m", "altitude_reference"),
               "route": ("projection", "repeat", "end", "start"),
               "pattern": ("pattern", "latitude_rad", "longitude_rad", "altitude_m", "altitude_reference", "radius_m", "clockwise",
                           "course_rad", "leg_m", "speed", "speed_reference", "duration_s"),
               "curve": ("latitude_rad", "longitude_rad", "altitude_m", "speed_min_ms", "speed_max_ms", "duration_s", "end", "append")}
MODE_DEFAULTS = {"hsa": (HOLD,) * 6, "route": (HOLD,) * 4, "pattern": (HOLD,) * 12, "curve": (HOLD,) * 8}
_REFERENCES = {"speed_reference": SpeedReference, "altitude_reference": AltitudeReference, "projection": Projection, "end": EndBehavior,
               "turn": TurnType, "pattern": PatternKind}

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

    __slots__ = ("world", "id", "level", "clamped", "source", "command_id")

    def __init__(self, world, activity_id, level, clamped=False, source=Source.POLICY, command_id=0):
        self.world = world
        self.id = activity_id
        self.level = level
        self.clamped = clamped  #: a value of the command was clamped to its range (World.last_command_details: which, and to what)
        self.source = Source(source)  #: the source it was submitted with, which its update and cancel declare
        self.command_id = command_id  #: the caller's id for its command (docs/flight-autonomy.md, 4.8), 0 none

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
            return bool(_checked(h.activity_update(self.id, (), int(self.source)), h)[4])
        return bool(_checked(h.activity_update(self.id, _row(self.level, values, fields), int(self.source)), h)[4])

    def update_route(self, waypoints=None, **options):
        """UPDATE of a route: new ``waypoints`` (None: those it has) and the options given (the others kept);
        checked as a NEW's, then flown afresh from its start, from where the aircraft is. Returns True if a value
        was clamped; raises fsim.Rejected (``index`` the waypoint at fault)."""
        rows = [] if waypoints is None else _waypoints(waypoints)
        h = self.world._h
        return bool(_checked(h.activity_update_route(self.id, _row("route", (), options), rows, int(self.source)), h)[4])

    def update_curve(self, segments=None, **options):
        """UPDATE of a curve: new ``segments`` (fsim.BezierSegment; None: those it has) and the options given (the
        others kept) - with ``append=1`` the segments go after its end, from the same reference; else they are a new
        curve, flown afresh. Returns True if a value was clamped; raises fsim.Rejected (``index`` the segment at
        fault; ``section`` where a segment is too tight)."""
        rows = [] if segments is None else _segments(segments)
        h = self.world._h
        return bool(_checked(h.activity_update_curve(self.id, _row("curve", (), options), rows, int(self.source)), h)[4])

    def append(self, segments, **options):
        """A curve's segments after its end, from the same reference: flown on to, the activity the same."""
        return self.update_curve(segments, append=1, **options)

    def cancel(self):
        """End it: its axes fly the vehicle default. Raises fsim.Rejected if it had already ended."""
        _checked(self.world._h.activity_cancel(self.id, int(self.source)), self.world._h)

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
        return self.state in (ActivityState.PENDING, ActivityState.ACTIVE)

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
        """(the native item, (level, source, validate_only)) - as the method would make its NEW."""
        k = dict(self.kwargs)
        source = Source(k.pop("source", Source.POLICY))
        axes = k.pop("axes", None)
        range_ = k.pop("range", RangePolicy.CLAMP)
        min_version = k.pop("min_version", 0)
        validate = bool(k.get("validate_only", False))
        envelope = _envelope(k.pop("command_id", 0), k.pop("trace", ()), k.pop("interactive", True), k.pop("validate_only", False))
        options = (int(source), None if axes is None else int(axes), int(range_), int(min_version), envelope)
        kind, args = self._KINDS[self.method], list(self.args)
        if self.method == "submit":
            level = Level(args.pop(0))
            return (kind, int(level), _row(level, args, k), None, None, None, options), (level, source, validate)
        if self.method == "submit_behavior":
            behavior = args.pop(0)
            target = args.pop(0) if args else k.pop("target", None)
            points = args.pop(0) if args else k.pop("points", None)
            t = target.id if isinstance(target, Vehicle) else int(target or 0)
            rows = None if points is None else [tuple(float(x) for x in p) for p in points]
            return (kind, 0, (), (behavior, t, k or None, rows), None, None, options), (Level.BEHAVIOR, source, validate)
        if self.method == "submit_support":
            what = args.pop(0)
            return (kind, SUPPORT_KINDS.index(what), _row(what, args, k), None, None, None, options), (what, source, validate)
        if self.method in ("submit_hsa", "submit_pattern"):
            mode = self.method[len("submit_"):]
            return (kind, MODE_KINDS.index(mode), _row(mode, args, k), None, None, None, options), (mode, source, validate)
        if self.method == "submit_route":
            waypoints = args.pop(0) if args else k.pop("waypoints")
            route = {"projection": k.pop("projection", Projection.GREAT_CIRCLE), "repeat": 1.0 if k.pop("repeat", False) else 0.0,
                     "end": k.pop("end", EndBehavior.CONTINUE), "start": k.pop("start", 0)}
            return (kind, 0, _row("route", (), route), None, _waypoints(waypoints), None, options), ("route", source, validate)
        segments = args.pop(0) if args else k.pop("segments")
        return (kind, 0, _row("curve", (), k), None, None, _segments(segments), options), ("curve", source, validate)


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
               interactive=True, validate_only=False, **fields):
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
        True: answered as a NEW would be, flying nothing - a fsim.Validation, never an Activity or Rejected."""
        level = Level(level)
        if level == Level.BEHAVIOR:
            raise TypeError("submit_behavior() takes behaviours")
        r = self._h.submit(self.id, int(level), _row(level, values, fields), int(source), None if axes is None else int(axes), int(range),
                           int(min_version), _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, level, source, validate_only)

    def _answer(self, r, level, source, validate_only):
        """A NEW's answer: an Activity (fsim.Rejected raised), or a validation's fsim.Validation."""
        if validate_only:
            return _validation(r, self._h)
        r = _checked(r, self._h)
        return Activity(self._world, r[2], level, bool(r[4]), source, r[10])

    def submit_behavior(self, behavior, target=None, points=None, *, source=Source.POLICY, range=RangePolicy.CLAMP,
                        min_version=0, command_id=0, trace=(), interactive=True, validate_only=False, **params):
        """NEW for a behaviour (see command_behavior for its arguments): an
        Activity, or fsim.Rejected. Behaviours that follow a vehicle need
        ``target``; a new target is a new submit. The command envelope as submit's."""
        t = target.id if isinstance(target, Vehicle) else int(target or 0)
        rows = None if points is None else [tuple(float(x) for x in p) for p in points]
        r = self._h.submit_behavior(self.id, behavior, t, params or None, rows, int(source), None, int(range), int(min_version),
                                    _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, Level.BEHAVIOR, source, validate_only)

    def submit_hsa(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(), interactive=True,
                   validate_only=False, **fields):
        """NEW for fsim.guidance.hsa, A-GRA's HSA/CSA (docs/vehicle-interface.md, 4.4): hold ``heading_rad`` or
        ``course_rad``, a ``speed`` in ``speed_reference`` (fsim.SpeedReference or its name: "true_airspeed",
        "calibrated_airspeed", "ground_speed", "mach") and ``altitude_m`` above ``altitude_reference``
        (fsim.AltitudeReference: "msl", "above_ground", "ellipsoid"). What it leaves out continues what a live hsa
        commanded, else what the aircraft flies now; a reference alone takes the aircraft's own value in it. An
        Activity whose ``update(**fields)`` changes only the fields given; fsim.Rejected if refused. The command
        envelope as submit's."""
        r = self._h.submit_mode(self.id, MODE_KINDS.index("hsa"), _row("hsa", values, fields), int(source), None, int(range), int(min_version),
                                _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, "hsa", source, validate_only)

    def submit_route(self, waypoints, *, projection=Projection.GREAT_CIRCLE, repeat=False, end=EndBehavior.CONTINUE, start=0,
                     source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(), interactive=True, validate_only=False):
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
                                 _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, "route", source, validate_only)

    def submit_pattern(self, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                       interactive=True, validate_only=False, **fields):
        """NEW for fsim.guidance.pattern, A-GRA's loiter (docs/vehicle-interface.md, 4.6): ``pattern``
        (fsim.PatternKind or "orbit", "racetrack", "figure_eight", "hold") round ``latitude_rad``, ``longitude_rad``
        (its centre or fix) at ``altitude_m``, with ``radius_m``, ``clockwise``, ``course_rad`` (the inbound course, a
        figure-eight's axis), ``leg_m``, a ``speed`` in ``speed_reference`` and ``duration_s`` (then it completes). What it
        leaves out takes its default: an orbit here, as the aircraft flies now, right turns, the radius its speed and 80 %
        of its bank give (a hold's: rate one), a hold's minute-long legs. An Activity whose ``update(**fields)`` changes
        only what it gives; fsim.Rejected if refused. The command envelope as submit's."""
        r = self._h.submit_mode(self.id, MODE_KINDS.index("pattern"), _row("pattern", values, fields), int(source), None, int(range),
                                int(min_version), _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, "pattern", source, validate_only)

    def submit_curve(self, segments, *, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                     interactive=True, validate_only=False, **fields):
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
                                 _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, "curve", source, validate_only)

    def submit_support(self, kind, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, command_id=0, trace=(),
                       interactive=True, validate_only=False, **fields):
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
                                   int(min_version), _envelope(command_id, trace, interactive, validate_only))
        return self._answer(r, kind, source, validate_only)

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
        for r, (level, source, validate) in zip(results, kinds):
            if validate:
                out.append(_validation(r, None))
            elif r[0] == 1:
                out.append(_rejected(r, None))
            else:
                out.append(Activity(self._world, r[2], level, bool(r[4]), source, r[10]))
        return out

    def activities(self):
        """The live activities (ActivityInfo), then the ended ones the vehicle remembers, newest first."""
        return [_info(t) for t in self._h.vehicle_activities(self.id)]

    @property
    def commanded(self):
        """What the cascade asked for in its last control update (CommandedState): an altitude, a heading, an
        airspeed, an attitude, rates, a throttle - NaN where no level set it."""
        t = self._h.commanded(self.id)
        return CommandedState(Level(t[0]), *t[1:])

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

    def request_control(self, capability):
        """A policy asks for control of a capability by id (A-GRA's ACQUIRE). Returns if granted; raises fsim.Rejected
        with "not_allowed", or the reason it is unavailable ("restricted", "collision_avoidance", "diverged")."""
        reason = self._h.request_control(self.id, capability)
        if reason:
            raise Rejected(_native.reason_name(reason))

    def release_control(self, capability):
        """The policy lets go: its grant ends, and its live activities of the capability end canceled, "released"."""
        self._h.release_control(self.id, capability)

    def revoke_control(self, capability, reason="revoked"):
        """The platform takes it back: the grant ends, and the policy's live activities of the capability end canceled
        with ``reason`` (a name, e.g. "revoked", "collision_avoidance", "restricted")."""
        self._h.revoke_control(self.id, capability, _reason_code(reason))

    def set_allowed(self, capability, allowed=True):
        """Whether the policy may request the capability (all may, by default); a grant for one no longer allowed is revoked."""
        self._h.set_allowed(self.id, capability, 1 if allowed else 0)

    def control_status(self, capability):
        """fsim.ControlStatus(allowed, granted) of a capability by id."""
        return ControlStatus(*self._h.control_status(self.id, capability))

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
