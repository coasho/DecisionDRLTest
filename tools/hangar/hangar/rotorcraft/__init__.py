"""hangar's rotorcraft pipeline: helicopters and multirotors from design files that cite their
sources (aircraft/<name>/<name>.toml with [aircraft] kind = "helicopter" or "multirotor").

    python -m hangar <name> [build model fly performance report]

build        the JSBSim aircraft (<name>.xml, Engines/) and its profile for the platform
model        the viewer's model (<name>.glb): the airframe, the rotors on nodes the viewer turns
fly          flight tests in the platform's own JSBSim - trim, steps, the hover plant identified
             into hover.toml beside the design, which the build writes into the profile's hover
             section (fly builds again after it)
performance  the performance tables (ADR-29 FA-3b; rotorcraft/performance.py): the power, fuel or
             charge and climb against altitude, weight and speed, into the profile's tables
             section (it builds again after them)
report       out/report.html: the checks, the tables and the plots

docs/rotorcraft.md is the design record; docs/hangar.md the tool's manual.
"""
import json
import math
import os
import time
import tomllib

from .. import __version__

KINDS = ("helicopter", "multirotor")
STAGES = ("build", "model", "fly", "performance", "report")

#: profile codes (include/fsim/VehicleProfile.h)
CLASS = {"helicopter": 12, "multirotor": 13}
FAMILY = {"helicopter": 3, "multirotor": 4}
ENGINE_TYPE = {"turboshaft": 5, "electric": 4}
KT = 0.514444


def kind_of(path):
    """The design's [aircraft] kind if it is a rotorcraft's, else None."""
    with open(path, "rb") as f:
        spec = tomllib.load(f)
    kind = spec.get("aircraft", {}).get("kind")
    return kind if kind in KINDS else None


class Rotorcraft:
    def __init__(self, path, log=print):
        self.path = path
        self.dir = os.path.dirname(path)
        with open(path, "rb") as f:
            self.spec = tomllib.load(f)
        self.name = self.spec["aircraft"]["name"]
        self.kind = self.spec["aircraft"]["kind"]
        self.out = os.path.join(self.dir, "out")
        os.makedirs(self.out, exist_ok=True)
        self.log = log

    # -- results ---------------------------------------------------------------------------------
    def save(self, stage, data):
        data = dict(data, stage=stage, hangar=__version__, time=time.strftime("%Y-%m-%d %H:%M:%S"))
        with open(os.path.join(self.out, stage + ".json"), "w", encoding="utf-8") as f:
            json.dump(data, f, indent=1, default=float)
        return data

    def load(self, stage):
        p = os.path.join(self.out, stage + ".json")
        if not os.path.isfile(p):
            return None
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    # -- the profile -----------------------------------------------------------------------------
    def profile(self):
        """{section: (version, {field: value})} for the JSBSim file."""
        s, heli = self.spec, self.kind == "helicopter"
        out = {"identity": (2, {"class": CLASS[self.kind], "family": FAMILY[self.kind]})}
        wheels = heli and s.get("ground", {}).get("kind") == "wheels"
        out["effectors"] = (2, {
            "pitch": 3 if heli else 4,    # PitchControl::Cyclic / Mixer
            "roll": 2 if heli else 3,     # RollControl::Cyclic / Mixer
            "yaw": 2 if heli else 3,      # YawControl::TailRotor / Mixer
            "thrust": 1 if heli else 2,   # ThrustControl::Collective / RotorThrust
            "neutral": 0, "flaps": 0, "retractable_gear": 0, "wheel_brakes": 1 if wheels else 0,
            "speedbrake": 0, "pitch_trim": 0})
        env = s.get("envelope", {})
        clean = {}
        for key, field in (("bank_max_deg", "bank_max_deg"), ("pitch_min_deg", "pitch_min_deg"), ("pitch_max_deg", "pitch_max_deg"),
                           ("roll_rate_max_deg_s", "roll_rate_max_deg_s")):
            if key in env:
                clean["clean/" + field] = float(env[key])
        if "cas_max_kt" in env:
            clean["clean/cas_max_ms"] = float(env["cas_max_kt"]) * KT
        # the wind it hovers, lands and lifts in at most, from any side: its type's published one, else the platform's own
        # (docs/flight-autonomy.md, 4.66) - the design's [operations], its source beside it
        wind = s.get("operations", {}).get("wind_kt")
        if wind is not None and 0.0 < float(wind) < 200.0:
            clean["crosswind_max_ms"] = round(float(wind) * KT, 2)
        if clean:
            out["envelope"] = (1, clean)
        if heli:
            out["propulsion"] = (2, {"engines": int(s["engine"].get("count", 1)), "type": ENGINE_TYPE[s["engine"]["type"]]})
        else:
            n = len(s["rotors"].get("layout", s["rotors"].get("positions", [])))
            out["propulsion"] = (2, {"engines": n, "type": ENGINE_TYPE["electric"],
                                     "spool_s": float(s["rotors"]["lag_up_s"])})
        hover = self.hover()
        if hover:
            out["hover"] = (1, hover)
        perf = (self.load("fly") or {}).get("performance") or {}
        if perf:
            out["performance"] = (1, {k: float(v) for k, v in perf.items()})
        # the performance tables, as flown (rotorcraft/performance.py)
        from ..profile import table_fields
        tables = table_fields((self.load("performance") or {}).get("tables") or {})
        if tables:
            out["tables"] = (1, tables)
        # the physical characteristics the design declares (docs/flight-autonomy.md, 5.2); their
        # sources go into the file header (heli.write, multi.write)
        from ..applicability import VERSION, declared
        fields, _ = declared(s)
        if fields:
            out["applicability"] = (VERSION, fields)
        return out

    # -- the identified hover ----------------------------------------------------------------------
    HOVER_AXES = ("roll", "pitch", "yaw", "heave")

    def hover(self):
        """The hover section's fields from hover.toml beside the design (what the fly stage
        identified); {} without one."""
        path = os.path.join(self.dir, "hover.toml")
        if not os.path.isfile(path):
            return {}
        with open(path, "rb") as f:
            h = tomllib.load(f)
        out = {"altitude_m": h["reference"]["altitude_m"], "mass_kg": h["reference"]["mass_kg"]}
        for key, value in h["trim"].items():
            out[key if key.endswith("_deg") else key + "_trim"] = value
        for axis in self.HOVER_AXES:
            for key, value in h["identified"][axis].items():
                out["%s/%s" % (axis, key)] = value
        return {k: float(v) for k, v in out.items()}

    def write_hover(self, section):
        """hover.toml from the fly stage's hover section (fly._hover_section), dated now - unless
        the file already holds that hover: then it keeps its date, and git sees no change."""
        from ..pipeline import WRITTEN_DATE, keep_date
        g = lambda v: "%.6g" % v  # noqa: E731
        lines = ["# Written by hangar fly (%s) for %s.toml: its hover, identified in the platform's own JSBSim" % (
                     time.strftime("%Y-%m-%d %H:%M"), self.name),
                 "# (docs/rotorcraft.md, 3.5) - the commands that hold it, the attitude it hovers at, and each axis's",
                 "# response: the body acceleration per unit of the platform's command (the heave's vertical, m/s2;",
                 "# signed, the platform's senses), its damping (1/s) and the actuator's lag (s). The build writes",
                 "# it into the aircraft's hover section; the platform designs its rotorcraft loops from it. Delete",
                 "# this file to fly those loops' own defaults.",
                 "[reference]",
                 "altitude_m = %s" % g(section["altitude_m"]),
                 "mass_kg = %s" % g(section["mass_kg"]),
                 "",
                 "[trim]   # the commands and the attitude of the hover",
                 ]
        for key in ("throttle", "aileron", "elevator", "rudder"):
            lines.append("%s = %s" % (key, g(section[key + "_trim"])))
        for key in ("roll_attitude_deg", "pitch_attitude_deg"):
            lines.append("%s = %s" % (key, g(section[key])))
        lines += ["", "[identified]   # per unit aileron, elevator, rudder (body rates p, q, r) and throttle (heave)"]
        for axis in self.HOVER_AXES:
            lines.append("%s = { power = %s, damping = %s, lag_s = %s }" % (
                axis, g(section[axis + "/power"]), g(section[axis + "/damping"]), g(section[axis + "/lag_s"])))
        path = os.path.join(self.dir, "hover.toml")
        text = keep_date(path, "\n".join(lines) + "\n", WRITTEN_DATE)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

    def profile_xml(self, indent="    "):
        lines = [indent + "<!-- the aircraft's profile for the platform (docs/control-architecture.md, section 7, and",
                 indent + "     docs/rotorcraft.md): what hangar knows of it - its design and its flight tests -->"]
        for section, (version, fields) in self.profile().items():
            lines.append(indent + '<property value="%d">fsim/%s/version</property>' % (version, section))
            lines.append(indent + '<property value="1">fsim/%s/provenance</property>' % section)
            for field in sorted(fields):
                v = float(fields[field])
                lines.append(indent + '<property value="%s">fsim/%s/%s</property>' % ("%.6g" % v, section, field))
        return "\n".join(lines) + "\n"

    # -- stages ----------------------------------------------------------------------------------
    def build(self):
        if self.kind == "helicopter":
            from . import heli
            info = heli.write(self.spec, self.dir, self.profile_xml())
        else:
            from . import multi
            info = multi.write(self.spec, self.dir, self.profile_xml())
        # a declared characteristic the design itself contradicts (docs/flight-autonomy.md, 5.2)
        from ..applicability import contradictions
        ground = self.spec.get("ground", {})
        stands = "legs" if "leg_height_m" in ground else {"skid": "skids", "wheels": "wheels"}.get(ground.get("kind"))
        checks = [{"name": "applicability", "value": "contradicts the design", "unit": "", "expected": "", "status": "fail", "note": c}
                  for c in contradictions(self.spec, retractable_gear=False, rotorcraft=True, ground_contact=stands)]
        if self.kind == "multirotor":
            t = self.spec.get("targets", {})
            checks.append(_check("thrust to weight", info["thrust_to_weight"], t.get("thrust_to_weight")))
            target_rpm = t.get("hover_rpm", t["hover_rads"] * 60.0 / (2.0 * math.pi) if "hover_rads" in t else None)
            checks.append(_check("hover rpm", info["hover_rpm"], target_rpm))
            # its contacts, sized for the step (multi.contact_set), let the legs sink under its weight: by a
            # share of their length, or it stands lower than it is drawn
            from . import multi
            most = multi.LEG_SAG_SHARE * ground["leg_height_m"]
            checks.append({"name": "legs' static deflection", "value": 1e3 * info["leg_sag_m"], "unit": "mm",
                           "expected": "<= %.3g" % (1e3 * most), "status": "pass" if info["leg_sag_m"] <= most else "warn",
                           "note": "the contacts sized for the platform's step"})
        return self.save("build", {"info": info, "checks": checks})

    def model(self):
        from . import model
        path = os.path.join(self.dir, self.name + ".glb")
        report = model.write(self.spec, path, self.path)
        return self.save("model", {"path": path, "report": report, "checks": model.checks(self.spec, report, path)})

    def fly(self):
        from . import fly
        results = fly.run(self)
        self.save("fly", results)
        if results.get("hover"):
            self.write_hover(results["hover"])
        self.build()  # the identified hover plant into the profile
        return results

    def performance(self):
        from . import performance as P
        from ..report import plots
        flown = self.load("fly")
        if not flown:
            raise SystemExit("%s: fly first (the tables' checks compare with its trims)" % self.name)
        t = P.fly_tables(self, log=self.log)
        checks = P.checks(self, t, flown)
        plots.performance(t, os.path.join(self.out, "performance.png"), self.name)
        out = self.save("performance", {"tables": t, "checks": checks, "images": ["performance.png"]})
        self.build()  # (the aircraft file carries them)
        return out

    def report(self):
        from . import report
        return {"path": report.write(self)}


def _check(name, value, target, tol=0.03, unit=""):
    status = "info"
    expected = ""
    if target is not None:
        expected = "%.4g" % target
        status = "pass" if abs(value - target) <= tol * abs(target) else "warn"
    return {"name": name, "value": value, "unit": unit, "expected": expected, "status": status, "note": ""}


def run(path, stages, log=print):
    r = Rotorcraft(path, log=log)
    done = {}
    for st in stages:
        if st not in STAGES:
            raise SystemExit("stage %r is not a rotorcraft's (%s)" % (st, " ".join(STAGES)))
        t0 = time.time()
        log("%s: %s" % (r.name, st))
        done[st] = getattr(r, st)()
        for c in done[st].get("checks", []) if isinstance(done[st], dict) else []:
            v = c["value"]
            log("  %s %-44s %10s %s" % ({"pass": "ok  ", "warn": "WARN", "fail": "FAIL", "info": "    "}[c["status"]], c["name"],
                                          v if isinstance(v, str) else "%.4g" % v, ("[%s]" % c["expected"]) if c["expected"] else ""))
        log("  (%.1f s)" % (time.time() - t0))
    return r, done
