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

for _level, _fields in COMMAND_FIELDS.items():
    if len(_fields) != _native.command_field_count(int(_level)):
        raise ImportError("fsim: the %s command has %d fields in the library" % (_level.name, _native.command_field_count(int(_level))))

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


class ActivityState(enum.IntEnum):
    PENDING = 0
    ACTIVE = 1
    COMPLETED = 2
    FAILED = 3
    CANCELED = 4


class Availability(enum.IntEnum):
    AVAILABLE = 0
    TEMPORARILY_UNAVAILABLE = 1
    FAULTED = 2
    DISABLED = 3


class Rejected(_native.Error):
    """A command the vehicle refused. ``reason`` says why ("authority_held",
    "activity_ended", "invalid_parameter", ...); ``other`` is the activity that
    holds the authority, for "authority_held"."""

    def __init__(self, reason, other=0):
        super().__init__("command refused: %s" % reason)
        self.reason = reason
        self.other = other


ActivityInfo = collections.namedtuple(
    "ActivityInfo", "id vehicle capability source axes state reason by constraints constraints_seen start_time end_time")
ActivityInfo.__doc__ = ("An activity's record: its capability (an index into Vehicle.capabilities()), who commanded it, "
                        "its state and why it ended, flags (1 saturated, 8 clamped, ...) and when it ran.")

Parameter = collections.namedtuple("Parameter", "name unit min max default optional")
Capability = collections.namedtuple("Capability", "id version kind interactions level axes terminating needs_target behavior parameters")
Capability.__doc__ = "What a vehicle offers: e.g. fsim.flight.attitude (a level) or fsim.guidance.hold (a behaviour)."


def _info(t):
    return ActivityInfo(t[0], t[1], t[2], Source(t[3]), t[4], ActivityState(t[5]), _native.reason_name(t[6]), t[7], t[8], t[9],
                        t[10], t[11])


def _checked(result):
    """A result tuple (status, reason, activity, other, clamped): raise Rejected unless accepted or canceled."""
    if result[0] == 1:
        raise Rejected(_native.reason_name(result[1]), result[3])
    return result


#: Support effectors (docs/sdk/control.md): their kinds, in the platform's
#: order, each command's fields and what they are when not given.
SUPPORT_KINDS = ("gear", "flaps", "wheel_brakes", "speedbrake", "pitch_trim")
SUPPORT_FIELDS = {"gear": ("down",), "flaps": ("position",), "wheel_brakes": ("left", "right"), "speedbrake": ("position",),
                  "pitch_trim": ("position",)}
SUPPORT_DEFAULTS = {"gear": (1.0,), "flaps": (0.0,), "wheel_brakes": (0.0, 0.0), "speedbrake": (0.0,), "pitch_trim": (0.0,)}


def _row(level, values, fields):
    """A level's (or support kind's) fields: all of them in order, or some by name with the rest as a new command's defaults."""
    if isinstance(level, str):
        names, defaults, what = SUPPORT_FIELDS[level], SUPPORT_DEFAULTS[level], level
    else:
        names, defaults, what = COMMAND_FIELDS[level], COMMAND_DEFAULTS[level], level.name
    if values:
        if fields or len(values) != len(names):
            raise TypeError("%s takes %d values in order (%s) or fields by name" % (what, len(names), ", ".join(names)))
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
    ``cancel`` ends it, handing the vehicle to its neutral default."""

    __slots__ = ("world", "id", "level", "clamped")

    def __init__(self, world, activity_id, level, clamped=False):
        self.world = world
        self.id = activity_id
        self.level = level
        self.clamped = clamped  #: a value of the command was clamped to its range

    @property
    def vehicle(self):
        return self.world._vehicle(self.id >> 32)

    def update(self, *values, **fields):
        """A new setpoint: the level's fields by name (fsim.COMMAND_FIELDS),
        the others as a new command's defaults, or all of them in order.
        Returns True if a value was clamped; raises fsim.Rejected if the
        activity has ended (preempted, completed, canceled)."""
        return bool(_checked(self.world._h.activity_update(self.id, _row(self.level, values, fields)))[4])

    def cancel(self):
        """End it: the vehicle flies its neutral default. Raises fsim.Rejected if it had already ended."""
        _checked(self.world._h.activity_cancel(self.id))

    @property
    def info(self):
        """Its record (ActivityInfo), or None once the vehicle no longer remembers it."""
        return self.world.activity(self.id)

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

    def submit(self, level, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, **fields):
        """NEW: a command at ``level`` - its fields by name (fsim.COMMAND_FIELDS),
        the others as the command's defaults, or all of them in order -
        becomes an Activity, answered at once. Raises fsim.Rejected if the
        vehicle refuses it (a higher source holds its axes, a value out of
        range with RangePolicy.REJECT, a required field left as HOLD)."""
        level = Level(level)
        if level == Level.BEHAVIOR:
            raise TypeError("submit_behavior() takes behaviours")
        r = _checked(self._h.submit(self.id, int(level), _row(level, values, fields), int(source), None, int(range), int(min_version)))
        return Activity(self._world, r[2], level, bool(r[4]))

    def submit_behavior(self, behavior, target=None, points=None, *, source=Source.POLICY, range=RangePolicy.CLAMP,
                        min_version=0, **params):
        """NEW for a behaviour (see command_behavior for its arguments): an
        Activity, or fsim.Rejected. Behaviours that follow a vehicle need
        ``target``; a new target is a new submit."""
        t = target.id if isinstance(target, Vehicle) else int(target or 0)
        rows = None if points is None else [tuple(float(x) for x in p) for p in points]
        r = _checked(self._h.submit_behavior(self.id, behavior, t, params or None, rows, int(source), None, int(range), int(min_version)))
        return Activity(self._world, r[2], Level.BEHAVIOR, bool(r[4]))

    def submit_support(self, kind, *values, source=Source.POLICY, range=RangePolicy.CLAMP, min_version=0, **fields):
        """NEW for a support effector the vehicle has - "gear" (down), "flaps"
        (position), "wheel_brakes" (left, right), "speedbrake" (position),
        "pitch_trim" (position) - set directly beside the flight activity. An
        Activity (gear and flaps complete when they are there), or fsim.Rejected:
        "unknown_capability" if the aircraft has no such effector, "unavailable"
        for the placards (gear up on the ground, gear or flaps out too fast)."""
        if kind not in SUPPORT_FIELDS:
            raise ValueError("support kind must be one of %s" % ", ".join(SUPPORT_KINDS))
        r = _checked(self._h.submit_support(self.id, SUPPORT_KINDS.index(kind), _row(kind, values, fields), int(source), None, int(range),
                                            int(min_version)))
        return Activity(self._world, r[2], kind, bool(r[4]))

    def activities(self):
        """The live activities (ActivityInfo), then the ended ones the vehicle remembers, newest first."""
        return [_info(t) for t in self._h.vehicle_activities(self.id)]

    def capabilities(self):
        """What the vehicle offers: its levels and behaviours (Capability, with their Parameters)."""
        return [Capability(c[0], c[1], c[2], c[3], Level(c[4]), c[5], c[6], c[7], c[8], [Parameter(*p) for p in c[9]])
                for c in self._h.capabilities(self.id)]

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

    def capability_status(self, capability):
        """(Availability, reason) of a capability by id, e.g. "fsim.guidance.hold"."""
        availability, reason = self._h.capability_status(self.id, capability)
        return Availability(availability), _native.reason_name(reason)

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

    def update(self, activities, values):
        """UPDATE many activities of one level in one call: their ids (a uint64
        array, or Activities) and a float64 row per activity in the level's
        field order. Raises fsim.Error naming the first one refused."""
        if not (isinstance(activities, np.ndarray) and activities.dtype == np.uint64 and activities.flags.c_contiguous):
            activities = np.fromiter((a.id if isinstance(a, Activity) else int(a) for a in activities), dtype=np.uint64)
        rows = np.ascontiguousarray(as_array(values, np.float64))
        stride = rows.shape[-1] if rows.ndim > 1 else (rows.size // max(len(activities), 1))
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
