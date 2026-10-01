"""The aircraft's profile for the platform (docs/control-architecture.md,
section 7): what differs between aircraft, as data, in sections the platform
reads from the JSBSim file's flight control section - fsim/<section>/<field>,
each section with its version and provenance.

hangar writes only what it knows: the design's own statements (its category,
its flight control law and that law's limits, its effectors and engines),
what its flight tests measured (stall, speed, ceiling, climb) and what the
autopilot stage identified (the responses at its reference condition). A
field it does not know it leaves out, and the platform treats it as unknown
- never a guess dressed as a limit.
"""
import json
import math
import os

VERSION = 1
PROVENANCE_HANGAR = 1

#: identity/class codes (include/fsim/VehicleProfile.h, AircraftClass)
CLASSES = {"light_ga": 1, "fighter": 2, "attack": 3, "bomber": 4, "transport": 5, "tanker": 6, "aew": 7, "reconnaissance": 8,
           "electronic warfare": 9, "uav": 10, "trainer": 11}
#: propulsion/type codes (EngineType)
ENGINE_TYPES = {"piston": 0, "turboprop": 1, "turbofan": 2, "turbojet": 3, "electric": 4}

KT = 0.514444  # m/s per knot
SPEED_OF_SOUND_SL = 340.294  # m/s, ISA sea level

#: the flap kinematics flight_control_xml writes: four steps of 2 s each
FLAPS_TRANSIT_S = 8.0


def fly_results(directory):
    """The design's flight-test results (out/fly.json), or {} before it has flown."""
    path = os.path.join(directory, "out", "fly.json")
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("results", {}).get("design", {})


def performance_tables(directory):
    """The design's performance tables (out/performance.json's), or {} before they are flown."""
    path = os.path.join(directory, "out", "performance.json")
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("tables", {})


def spawn_stall(tables):
    """The performance tables' stall (calibrated, m/s) at their lowest altitude, at the weight the aircraft spawns at
    (linear between the weights flown); None without one."""
    if not tables or not tables.get("stall_cas_ms"):
        return None
    points = [(float(w), float(x)) for w, x in zip(tables["weight_kg"], tables["stall_cas_ms"][0]) if x is not None and math.isfinite(float(x))]
    if not points:
        return None
    w = float(tables.get("spawn_weight_kg", points[-1][0]))
    for (w0, x0), (w1, x1) in zip(points, points[1:]):
        if w0 <= w <= w1:
            return x0 + (x1 - x0) * (w - w0) / (w1 - w0)
    return points[0][1] if w < points[0][0] else points[-1][1]


def table_fields(tables):
    """The tables section's fields (docs/flight-autonomy.md, SUB-02): the axes - altitude_m/h<i>, weight_kg/w<j>,
    speed_fraction/v<k> - and each table's cells, <name>/h<i>/w<j> or <name>/h<i>/w<j>/v<k>; a cell not flown is
    left out (NaN when read)."""
    if not tables:
        return {}
    out = {}
    for axis, tag in (("altitude_m", "h"), ("weight_kg", "w"), ("speed_fraction", "v")):
        for i, x in enumerate(tables[axis]):
            out["%s/%s%d" % (axis, tag, i)] = float(x)
    for name in ("fuel_capacity_kg", "battery_capacity_j"):
        if tables.get(name) is not None:
            out[name] = float(tables[name])
    for name, value in sorted(tables.items()):
        # (the speed, throttle and angle of attack each level point flew - a rotorcraft's pitch attitude, its engines'
        # shaft power and its rotor's speed - stay in the report: the platform reads each point at its fraction of the
        # band, and the rest)
        if name in ("altitude_m", "weight_kg", "speed_fraction", "tas_ms", "throttle", "alpha_deg", "pitch_deg", "shaft_power_w",
                    "rotor_rpm") or not isinstance(value, list):
            continue
        for i, row in enumerate(value):
            for j, cell in enumerate(row):
                if isinstance(cell, list):
                    for k, x in enumerate(cell):
                        if x is not None and math.isfinite(float(x)):
                            out["%s/h%d/w%d/v%d" % (name, i, j, k)] = float(x)
                elif cell is not None and math.isfinite(float(cell)):
                    out["%s/h%d/w%d" % (name, i, j)] = float(cell)
    return out


def sections(aircraft, fbw, reference, identified, flown, tables=None, tail_down=None, turn_radius=None, yaw_accel=None):
    """{section: {field: value}} for the JSBSim file.

    aircraft   the design (geometry.aircraft.Aircraft)
    fbw        fcs.design()'s result, or None for surfaces on the stick
    reference  autopilot.toml's [reference] (tas_ms, eas_ms, altitude_m), or {}
    identified autopilot.toml's [identified]: the responses measured there, or {}
    flown      the flight tests' results (fly_results), or {}
    tables     the performance tables (performance_tables), or {}
    tail_down  the pitch attitude its tail touches the ground at (jsbsim.tail_down_deg), deg, or None
    turn_radius  its tightest turn on its wheels (the ground stage's, else jsbsim.turn_radius_m), m, or None
    yaw_accel  the yaw acceleration its steering gives it on its wheels (the ground stage's), rad/s^2, or None
    """
    spec = aircraft.spec
    control = spec.get("flight_control", {})
    out = {}

    # identity: what it is and how the controls reach the surfaces
    identity = {"family": 2 if fbw is not None else 1}
    category = spec.get("aircraft", {}).get("category")
    if category in CLASSES:
        identity["class"] = CLASSES[category]
    out["identity"] = identity

    # effectors: what the stick means, and the support effectors it has
    channels = set(aircraft.channels())
    effectors = {
        "pitch": 1 if fbw is not None else 0,       # a load-factor demand, or the elevator
        "roll": 1 if fbw is not None else 0,        # a roll-rate demand, or the ailerons
        "yaw": 0,
        "neutral": 1 if fbw is not None else 0,     # centred, the law holds the flight path
        "flaps": 1 if "flap" in channels else 0,
        "retractable_gear": 1 if any(g.retractable for g in aircraft.gear) else 0,
        "wheel_brakes": 1 if aircraft.gear else 0,
        "speedbrake": 1 if aircraft.spec.get("drag_device") else 0,  # its drag devices (4.55)...
        "pitch_trim": 1 if fbw is None and "elevator" in channels else 0,  # the elevator channel sums fcs/pitch-trim-cmd-norm
    }
    if "flap" in channels:
        effectors["flaps_transit_s"] = FLAPS_TRANSIT_S
    # ...and whether they are opened on an approach: airbrakes and surfaces, not spoilers that dump lift - one speedbrake opens
    # them all, so all of them (the U-2S's airbrakes open its spoilers too: 4.55)
    from .drag import drag_devices
    brakes = drag_devices(aircraft)
    if brakes and all(d.in_flight for d in brakes):
        effectors["speedbrake_approach"] = 1
    out["effectors"] = effectors

    # envelope: the law's limits as the design states them, and the stall as flown;
    # a fly-by-wire law enforces its own (fcs.py), so the platform's protection
    # clamps setpoints to them but adds no feedback limiter to fight it
    clean, law = {}, {}
    for key, field, flag in (("n_max", "n_max", "law_load_factor"), ("n_min", "n_min", "law_load_factor"),
                             ("alpha_max_deg", "alpha_max_deg", "law_alpha"), ("roll_rate_deg_s", "roll_rate_max_deg_s", "law_roll_rate")):
        if key in control:
            clean[field] = float(control[key])
            if fbw is not None:
                law[flag] = 1
    stall = flown.get("stall") or {}
    if "stall_kcas" in stall:
        clean["cas_min_ms"] = float(stall["stall_kcas"]) * KT
        if "alpha_max_deg" not in clean and "alpha_at_stall" in stall:
            clean["alpha_max_deg"] = float(stall["alpha_at_stall"])
    elif spawn_stall(tables) is not None:
        # no stall flown (a fly-by-wire law will not let one be): the performance tables' - idle, the height held, to
        # the law's angle of attack - at the lowest altitude, as the aircraft spawns (ADR-29 FA-3d: the least it is
        # flown at, which energy management keeps a margin over)
        clean["cas_min_ms"] = spawn_stall(tables)
    envelope = dict({"clean/" + k: v for k, v in clean.items()}, **law)
    # on the wheels: the pitch attitude the tail touches at, pivoting on the aftmost
    # wheels (a launch rotates short of it)
    if tail_down is not None and 0.0 < tail_down < 90.0:
        envelope["ground_pitch_max_deg"] = round(tail_down, 2)
    # and its tightest turn there (a taxi's corners are drawn wider: 4.51)
    if turn_radius is not None and 0.0 < turn_radius < 1000.0:
        envelope["ground_turn_radius_m"] = round(turn_radius, 2)
    if yaw_accel is not None and 0.0 < yaw_accel < 100.0:
        envelope["ground_yaw_accel_rad_s2"] = round(yaw_accel, 4)
    # the 90 deg crosswind it lands and takes off in at most: its type's published one, else its flying qualities'
    # requirement - the design's [operations], its source beside it (docs/flight-autonomy.md, 4.54)
    crosswind = spec.get("operations", {}).get("crosswind_kt")
    if crosswind is not None and 0.0 < float(crosswind) < 200.0:
        envelope["crosswind_max_ms"] = round(float(crosswind) * KT, 2)
    # its placards, published, else none (docs/flight-autonomy.md, 4.57): the most it moves its gear at, and the most with
    # its flaps out beyond flaps_above - the platform refuses gear and flaps above them, and its protection holds them
    operations = spec.get("operations", {})
    gear = operations.get("gear_kt")
    if gear is not None and 0.0 < float(gear) < 1000.0:
        envelope["gear_cas_max_ms"] = round(float(gear) * KT, 2)
    flaps = operations.get("flaps_kt")
    if flaps is not None and 0.0 < float(flaps) < 1000.0:
        envelope["flaps/cas_max_ms"] = round(float(flaps) * KT, 2)
        envelope["flaps_threshold"] = float(operations.get("flaps_above", 0.05))
    if envelope:
        out["envelope"] = envelope

    # propulsion
    engines = aircraft.engines
    if engines:
        prop = {"engines": sum(len(e.copies()) for e in engines)}
        kind = engines[0].type
        if kind in ENGINE_TYPES:
            prop["type"] = ENGINE_TYPES[kind]
        prop["afterburner"] = 1 if any(getattr(e, "thrust_wet_kn", None) for e in engines) else 0
        speed = identified.get("speed") or {}
        if "lag_s" in speed:
            prop["spool_s"] = float(speed["lag_s"])  # the thrust's response, identified
        out["propulsion"] = prop

    # plant: the responses the autopilot stage identified, where it did
    if reference and identified:
        plant = {"tas_ms": float(reference["tas_ms"]), "eas_ms": float(reference["eas_ms"]), "altitude_m": float(reference["altitude_m"])}
        for axis in ("roll", "pitch", "yaw", "speed"):
            r = identified.get(axis) or {}
            if "gain" in r:
                plant[axis + "/gain"] = float(r["gain"])
            if "lag_s" in r:
                plant[axis + "/tau_s"] = float(r["lag_s"])
        # the fits of level flight (autopilot.fits): the platform designs the loops from these
        if "throttle_trim" in identified:
            plant["throttle_trim"] = float(identified["throttle_trim"])
        if "elevator_trim" in identified:
            plant["elevator_trim"] = float(identified["elevator_trim"])
            plant["elevator_trim_lift"] = float(identified.get("elevator_trim_lift", 0.0))
        if "alpha_zero_lift_rad" in identified:
            plant["alpha_zero_lift_deg"] = math.degrees(float(identified["alpha_zero_lift_rad"]))
        out["plant"] = plant

    # performance, as flown
    perf = {}
    if "stall_kcas" in stall:
        perf["stall_cas_ms"] = float(stall["stall_kcas"]) * KT
    fighter = flown.get("fighter") or {}
    if "max_speed_ms" in flown:
        perf["max_tas_ms"] = float(flown["max_speed_ms"])
    elif "max_mach_sl" in fighter:
        perf["max_tas_ms"] = float(fighter["max_mach_sl"]) * SPEED_OF_SOUND_SL
    climb = flown.get("climb") or {}
    ceiling = climb.get("service_ceiling_m", fighter.get("service_ceiling_m"))
    if ceiling is not None and math.isfinite(float(ceiling)):
        perf["ceiling_m"] = float(ceiling)
    if "climb_rate_ms" in fighter:
        perf["climb_ms"] = float(fighter["climb_rate_ms"])
    if perf:
        out["performance"] = perf

    # the performance tables, as flown (performance.py)
    fields = table_fields(tables or {})
    if fields:
        out["tables"] = fields

    # applicability: the physical characteristics the design declares, each with its source
    # (docs/flight-autonomy.md, 5.2); the sources go into the file header (jsbsim.aircraft_xml)
    from .applicability import declared
    fields, _ = declared(spec)
    if fields:
        out["applicability"] = fields
    return out


def _num(x):
    x = float(x)
    return "0.0" if x == 0.0 else ("%.6g" % x if "e" not in "%.6g" % x else "%.6e" % x)


def properties_xml(profile, indent="      "):
    """The sections as JSBSim property declarations: fsim/<section>/<field>,
    each section with its version and provenance (hangar)."""
    if not profile:
        return ""
    lines = [indent + "<!-- the aircraft's profile for the platform (docs/control-architecture.md, section 7): what",
             indent + "     hangar knows of it - its design, its flight tests, its identified responses -->"]
    for section in ("identity", "effectors", "envelope", "propulsion", "plant", "performance", "applicability", "tables"):
        fields = profile.get(section)
        if not fields:
            continue
        lines.append(indent + "<property value=\"%d\">fsim/%s/version</property>" % (VERSION, section))
        lines.append(indent + "<property value=\"%d\">fsim/%s/provenance</property>" % (PROVENANCE_HANGAR, section))
        for field in sorted(fields):
            lines.append(indent + "<property value=\"%s\">fsim/%s/%s</property>" % (_num(fields[field]), section, field))
    return "\n".join(lines) + "\n"
