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


def sections(aircraft, fbw, settings, reference, identified, flown):
    """{section: {field: value}} for the JSBSim file.

    aircraft   the design (geometry.aircraft.Aircraft)
    fbw        fcs.design()'s result, or None for surfaces on the stick
    settings   the autopilot's settings {controller: {parameter: value}}
    reference  autopilot.toml's [reference] (tas_ms, eas_ms, altitude_m), or {}
    identified autopilot.toml's [identified]: the responses measured there, or {}
    flown      the flight tests' results (fly_results), or {}
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
        "speedbrake": 0,
        "pitch_trim": 1 if fbw is None and "elevator" in channels else 0,  # the elevator channel sums fcs/pitch-trim-cmd-norm
    }
    if "flap" in channels:
        effectors["flaps_transit_s"] = FLAPS_TRANSIT_S
    out["effectors"] = effectors

    # envelope: the law's limits as the design states them, and the stall as flown
    clean = {}
    for key, field in (("n_max", "n_max"), ("n_min", "n_min"), ("alpha_max_deg", "alpha_max_deg"), ("roll_rate_deg_s", "roll_rate_max_deg_s")):
        if key in control:
            clean[field] = float(control[key])
    stall = flown.get("stall") or {}
    if "stall_kcas" in stall:
        clean["cas_min_ms"] = float(stall["stall_kcas"]) * KT
        if "alpha_max_deg" not in clean and "alpha_at_stall" in stall:
            clean["alpha_max_deg"] = float(stall["alpha_at_stall"])
    if clean:
        out["envelope"] = {"clean/" + k: v for k, v in clean.items()}

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
        attitude = settings.get("pid_attitude", {})
        if "pitch.trim" in attitude:
            plant["elevator_trim"] = attitude["pitch.trim"]
            plant["elevator_trim_lift"] = attitude.get("pitch.trim_lift", 0.0)
        alpha0 = settings.get("pid_velocity", {}).get("vertical_speed.alpha_zero_lift")
        if alpha0 is not None:
            plant["alpha_zero_lift_deg"] = math.degrees(alpha0)
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
    for section in ("identity", "effectors", "envelope", "propulsion", "plant", "performance"):
        fields = profile.get(section)
        if not fields:
            continue
        lines.append(indent + "<property value=\"%d\">fsim/%s/version</property>" % (VERSION, section))
        lines.append(indent + "<property value=\"%d\">fsim/%s/provenance</property>" % (PROVENANCE_HANGAR, section))
        for field in sorted(fields):
            lines.append(indent + "<property value=\"%s\">fsim/%s/%s</property>" % (_num(fields[field]), section, field))
    return "\n".join(lines) + "\n"
