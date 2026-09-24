"""The stages, in order, each with its outputs and checks.

    geometry    drawings and the numbers a designer checks first
    aero        section polars, the lattice, the full-envelope tables
    mass        component weights, CG, inertia, static margin
    propulsion  propeller tables, engine
    build       the JSBSim aircraft (+ a glTF model), found by the platform as jsbsim:<name>
    verify      JSBSim flies exactly the tables (forces at random states)
    fly         flight tests: trim, stall, climb, speed, modes, robustness
    report      one HTML page with everything

Outputs go to aircraft/<name>/out/; the product (the JSBSim aircraft and the
model) to aircraft/<name>/. Every stage writes <stage>.json with its numbers
and checks; a check is pass / warn / fail against an expected range, or info.
"""
import glob
import hashlib
import json
import math
import os
import pickle

import time

import numpy as np

from . import __version__
from .geometry import Aircraft

STAGES = ("geometry", "aero", "mass", "propulsion", "build", "verify", "fly", "calibrate", "report")
DEFAULT = ("geometry", "aero", "mass", "propulsion", "build", "verify", "fly", "report")
KT = 0.514444


def check(name, value, lo=None, hi=None, unit="", level="fail", note="", fmt="%.3g"):
    """One check: value within [lo, hi] passes, else `level` (warn/fail)."""
    ok = value is not None and not np.isnan(value) and (lo is None or value >= lo) and (hi is None or value <= hi)
    rng = ("%s .. %s" % (fmt % lo if lo is not None else "", fmt % hi if hi is not None else "")) if (lo is not None or hi is not None) else ""
    return {"name": name, "value": None if value is None or np.isnan(value) else float(value), "unit": unit,
            "expected": rng.strip(), "status": "pass" if ok else level, "note": note}


def info(name, value, unit="", note=""):
    return {"name": name, "value": value if isinstance(value, str) else (None if value is None else float(value)),
            "unit": unit, "expected": "", "status": "info", "note": note}


def _json(obj):
    if isinstance(obj, dict):
        return {str(k): _json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _json(obj.tolist())
    if isinstance(obj, (np.floating, float)):
        return None if not np.isfinite(obj) else float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, complex):
        return [obj.real, obj.imag]
    return obj


def find_design(arg, root=None):
    """A design by name (aircraft/<name>/<name>.toml) or path."""
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    root = root or repo_root()
    p = os.path.join(root, "aircraft", arg, arg + ".toml")
    if os.path.isfile(p):
        return p
    raise SystemExit("no design %r (looked for %s)" % (arg, p))


def repo_root():
    here = os.path.dirname(os.path.abspath(__file__))
    for up in (os.path.join(here, "..", "..", ".."), os.path.join(here, "..", "..", "..", "..")):
        up = os.path.abspath(up)
        if os.path.isdir(os.path.join(up, "aircraft")) and os.path.isfile(os.path.join(up, "fsim.cmd")):
            return up
    return os.getcwd()


def shown(path):
    """A path relative to the repository when it is inside it."""
    root = repo_root()
    try:
        rel = os.path.relpath(path, root)
    except ValueError:          # another drive
        return path
    return path if rel.startswith("..") else rel


class Design:
    def __init__(self, path, log=print):
        self.path = path
        self.dir = os.path.dirname(path)
        self.aircraft = Aircraft.load(path)
        self.name = self.aircraft.name
        self.out = os.path.join(self.dir, "out")
        os.makedirs(self.out, exist_ok=True)
        self.log = log
        self._aero_model = None
        self.targets = self.aircraft.spec.get("targets", {})

    @property
    def quick(self):
        """[analysis] quick = true (or hangar --quick): coarse tables and
        shorter flight tests - a first look in a fraction of the time."""
        return bool(self.aircraft.spec.get("analysis", {}).get("quick", False))

    # -- helpers --------------------------------------------------------------------------------
    def save(self, stage, data):
        data = dict(data, stage=stage, hangar=__version__, time=time.strftime("%Y-%m-%d %H:%M:%S"))
        with open(os.path.join(self.out, stage + ".json"), "w", encoding="utf-8") as f:
            json.dump(_json(data), f, indent=1)
        return data

    def load(self, stage):
        p = os.path.join(self.out, stage + ".json")
        if not os.path.isfile(p):
            return None
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    def img(self, name):
        return os.path.join(self.out, name)

    def spec_hash(self, *sections):
        spec = self.aircraft.spec
        blob = json.dumps({k: spec.get(k) for k in sections}, sort_keys=True, default=str) + __version__
        # and the code that computes them: a change to the methods is a change to the tables
        here = os.path.dirname(os.path.abspath(__file__))
        for f in sorted(glob.glob(os.path.join(here, "aero", "*.py")) + glob.glob(os.path.join(here, "geometry", "*.py"))):
            with open(f, "rb") as fh:
                blob += hashlib.sha1(fh.read()).hexdigest()
        # airfoil files referenced by path change the result too
        for f in sorted(glob.glob(os.path.join(self.dir, "*.dat"))):
            blob += open(f, "rb").read().decode("latin-1")
        return hashlib.sha1(blob.encode()).hexdigest()[:16]

    # -- geometry -------------------------------------------------------------------------------
    def geometry(self):
        from .report import render
        a = self.aircraft
        s = a.summary()
        marks = {"ARP": (a.aero_point, "#2b6cb0")}
        mass = self.load("mass")
        if mass:
            marks["CG"] = (mass["loaded_cg_m"], "#e53e3e")
        aero = self.load("aero")
        if aero and aero.get("neutral_point_m"):
            marks["NP"] = (aero["neutral_point_m"], "#38a169")
        refs = self._references()
        render.three_view(a, self.img("threeview.png"), marks=marks, references=refs)
        render.views_png(a, self.img("views.png"))
        checks = []
        t = self.targets
        for key, value, label in (("wing_area_m2", a.S, "wing area"), ("wing_span_m", a.b, "wing span")):
            if key in t:
                checks.append(check(label, value, 0.97 * t[key], 1.03 * t[key], "m2" if "area" in key else "m",
                                    note="target %.3g" % t[key]))
        checks.append(check("wing aspect ratio", a.b**2 / a.S, 3.0, 30.0, level="warn"))
        for name, d in s["surfaces"].items():
            if d["kind"] in ("htail", "canard") and "volume_coefficient" in d:
                checks.append(check("%s volume coefficient" % name, abs(d["volume_coefficient"]), 0.35, 1.1, level="warn",
                                    note="typical 0.5-0.9 (Raymer 6.4)"))
            if d["kind"] in ("fin", "vtail") and "volume_coefficient" in d:
                checks.append(check("%s volume coefficient" % name, d["volume_coefficient"], 0.02, 0.1, level="warn",
                                    note="typical 0.03-0.08"))
        checks.append(info("mean aerodynamic chord", a.c, "m"))
        return self.save("geometry", {"summary": s, "checks": checks, "images": ["threeview.png", "views.png"]})

    def _references(self):
        """Reference drawings from the spec, for the three-view: each view's
        image with two points of known position, to place and scale it."""
        refs = {}
        for r in self.aircraft.spec.get("reference_image", []):
            try:
                import matplotlib.image as mpimg
                img = mpimg.imread(os.path.join(self.dir, r["file"]))
            except Exception as e:  # noqa: BLE001
                self.log("reference image %s: %s" % (r.get("file"), e))
                continue
            (u1, v1), (u2, v2) = r["pixels"]
            (x1, y1), (x2, y2) = r["points"]   # page coordinates of the view (metres)
            sx = (x2 - x1) / (u2 - u1)
            sy = (y2 - y1) / (v2 - v1)
            h, w = img.shape[:2]
            left, top = x1 - u1 * sx, y1 - v1 * sy
            refs[r["view"]] = (img, (left, left + w * sx, top + h * sy, top))
        return refs

    # -- aero -----------------------------------------------------------------------------------
    def aero_model(self):
        if self._aero_model is None:
            from .aero.model import AeroModel
            self._aero_model = AeroModel(self.aircraft)
        return self._aero_model

    def tables(self, force=False):
        from .aero import tables as T
        key = self.spec_hash("surface", "body", "reference", "analysis", "gear", "engine")
        cache = os.path.join(self.out, "tables_quick.pkl" if self.quick else "tables.pkl")
        if not force and os.path.isfile(cache):
            with open(cache, "rb") as f:
                saved = pickle.load(f)
            if saved.get("_key") == key:
                return saved
        m = self.aero_model()
        t0 = time.time()
        last = [0.0]

        def progress(stage, i, n):
            if time.time() - last[0] > 4:
                self.log("  aero: %s %d/%d (%.0f s)" % (stage, i, n, time.time() - t0))
                last[0] = time.time()
        tabs = T.build(m, progress=progress)
        tabs["_key"] = key
        with open(cache, "wb") as f:
            pickle.dump(tabs, f)
        self.log("  aero: tables built in %.0f s" % (time.time() - t0))
        return tabs

    def aero(self, force=False):
        from .aero import tables as T
        from .report import plots
        a = self.aircraft
        tabs = self.tables(force)
        d = T.derivatives(tabs)
        m = self.aero_model()
        # neutral point: where the pitching moment stops changing with lift
        np_x = a.aero_point[0] - d["Cma"] / d["CLa"] * a.c
        base = tabs["base"]
        j0 = int(np.argmin(np.abs(tabs["beta"])))
        al = tabs["alpha"]
        band = (al > -5) & (al < 30)
        clmax = float(np.max(base["CL"][band, j0]))
        a_clmax = float(al[band][np.argmax(base["CL"][band, j0])])
        polars = {}
        for k, label in ((0, "wing root"), (int(np.argmax(m.lat.eta * (m.lat.surface_index == 0))), "wing tip")):
            polars["%s (%s, Re %.1e)" % (label, m.lat.airfoils[k].name.split("~")[0], m.polars.polars[k].re)] = m.polars.polars[k]
        plots.polars(polars, self.img("polars.png"), title="section polars of the wing")
        plots.coefficients(tabs, self.img("coefficients.png"), a.name)
        plots.derivatives_vs_alpha(tabs, self.img("derivatives.png"), a.name)
        smooth = _roughness(tabs)
        checks = [
            check("lift-curve slope CL_alpha", d["CLa"], 3.5, 6.8, "/rad"),
            check("CL max (untrimmed, clean)", clmax, 1.1, 2.1, note="at %.0f deg" % a_clmax),
            check("pitch stiffness Cm_alpha (about ARP)", d["Cma"], None, -0.1, "/rad", note="negative: stable"),
            check("weathercock Cn_beta", d["Cnb"], 0.01, None, "/rad", note="positive: stable"),
            check("dihedral effect Cl_beta", d["Clb"], None, -0.005, "/rad", note="negative: stable"),
            check("roll damping Cl_p", d["Clp"], None, -0.1, "/rad"),
            check("pitch damping Cm_q", d["Cmq"], None, -1.0, "/rad"),
            check("yaw damping Cn_r", d["Cnr"], None, -0.01, "/rad"),
            check("table smoothness (largest jump between neighbouring entries)", smooth["worst"], None, 0.35,
                  level="warn", note=smooth["where"]),
        ]
        for ch, key, lo, hi, label in (("elevator", "Cm_elevator", None, -0.2, "elevator power Cm_de"),
                                       ("aileron", "Cl_aileron", 0.05, None, "aileron power Cl_da"),
                                       ("rudder", "Cn_rudder", None, -0.01, "rudder power Cn_dr")):
            if key in d:
                checks.append(check(label, d[key], lo, hi, "/rad"))
        if self.quick:
            checks.append(info("tables", "quick", note="coarse grids: a first look; run without --quick for the product"))
        checks.append(info("CD at zero lift (CD0 estimate)", float(np.min(base["CD"][band, j0]))))
        checks.append(info("neutral point", (np_x - a.mac_le[0]) / a.c * 100, "% MAC"))
        return self.save("aero", {"derivatives": d, "cl_max": clmax, "alpha_cl_max_deg": a_clmax,
                                  "neutral_point_m": [np_x, 0.0, a.aero_point[2]], "drag_area_m2": m.drag_area,
                                  "lattice": m.lat.summary(), "checks": checks,
                                  "images": ["coefficients.png", "derivatives.png", "polars.png"]})

    # -- mass -----------------------------------------------------------------------------------
    def mass(self):
        from .mass import MassModel
        a = self.aircraft
        mm = MassModel(a)
        e = mm.empty()
        m, cg = mm.loaded()
        g = mm.gyration()
        aero = self.load("aero")
        checks = []
        if mm.empty_target is not None:
            checks.append(check("empty mass", e["mass"], 0.99 * mm.empty_target, 1.01 * mm.empty_target, "kg"))
        sm = None
        if aero:
            np_x = aero["neutral_point_m"][0]
            sm = (np_x - cg[0]) / a.c
            checks.append(check("static margin (loaded)", sm * 100, 5.0, 40.0, "% MAC",
                                note="neutral point %.3f m, CG %.3f m" % (np_x, cg[0])))
            m_e, cg_e = mm.loaded(fuel_fraction=0.0, payload=False)
            checks.append(check("static margin (empty)", (np_x - cg_e[0]) / a.c * 100, 0.0, 45.0, "% MAC", level="warn"))
        checks += [check("roll gyration radius R_x", g["Rx"], 0.15, 0.45, level="warn", note="Roskam: GA 0.2-0.35"),
                   check("pitch gyration radius R_y", g["Ry"], 0.2, 0.5, level="warn", note="Roskam: GA 0.3-0.45"),
                   check("yaw gyration radius R_z", g["Rz"], 0.25, 0.6, level="warn", note="Roskam: GA 0.35-0.5"),
                   info("loaded mass", m, "kg")]
        breakdown = [{"name": n, "mass_kg": mk, "kind": k} for n, mk, k in mm.breakdown()]
        return self.save("mass", {"empty": e, "loaded_mass_kg": m, "loaded_cg_m": cg, "gyration": g,
                                  "static_margin": sm, "breakdown": breakdown, "checks": checks})

    # -- propulsion -----------------------------------------------------------------------------
    def propulsion(self):
        from .propulsion import Propeller, static_numbers
        from .report import plots
        a = self.aircraft
        out = []
        checks = []
        mass = self.load("mass") or self.mass()
        W = mass["loaded_mass_kg"] * 9.80665
        tabs = {}
        for e in a.engines:
            p = Propeller(e)
            t = p.tables()
            n = static_numbers(t, p.D, p.rpm, e.power_kw * 1000.0)
            tabs[e.name] = t
            out.append({"engine": e.name, "power_kw": e.power_kw, "rpm": e.rpm, "diameter_m": p.D, "pitch_m": p.pitch,
                        "activity_factor": p.activity_factor, "tables": t, **n})
            checks.append(check("%s peak propeller efficiency" % e.name, n["peak_efficiency"], 0.7, 0.92))
            checks.append(check("%s zero-thrust advance ratio" % e.name, n["zero_thrust_J"], 0.6, 1.8, level="warn",
                                note="pitch/diameter %.2f" % (p.pitch / p.D)))
        total_static = sum(o["static_thrust_n"] * len(e.copies()) for o, e in zip(out, a.engines))
        if a.engines:
            checks.append(check("static thrust / weight", total_static / W, 0.15, 0.8, level="warn"))
            plots.propeller(tabs, self.img("propeller.png"))
        return self.save("propulsion", {"engines": out, "checks": checks, "images": ["propeller.png"] if a.engines else []})

    # -- build ----------------------------------------------------------------------------------
    def build(self):
        from . import jsbsim, model3d
        from .mass import MassModel
        from .propulsion import Propeller, electric_xml, piston_xml, propeller_xml
        a = self.aircraft
        tabs = self.calibrated(self.tables())
        mm = MassModel(a)
        eng_dir = os.path.join(self.dir, "Engines")
        os.makedirs(eng_dir, exist_ok=True)
        files = []
        for i, e in enumerate(a.engines):
            ename = "%s_engine%d" % (a.name, i)
            pname = "%s_prop%d" % (a.name, i)
            with open(os.path.join(eng_dir, ename + ".xml"), "w", encoding="utf-8") as f:
                f.write(piston_xml(e) if e.type == "piston" else electric_xml(e))
            p = Propeller(e)
            mass_e = float(e.mass) if e.mass is not None else 30.0
            with open(os.path.join(eng_dir, pname + ".xml"), "w", encoding="utf-8") as f:
                f.write(propeller_xml(p, p.tables(), 0.1 * mass_e, "%s propeller %d" % (a.name, i)))
            files.append((ename, pname))
        xml_path = os.path.join(self.dir, a.name + ".xml")
        with open(xml_path, "w", encoding="utf-8") as f:
            f.write(jsbsim.aircraft_xml(a, tabs, mm, files))
        e = mm.empty()
        glb = os.path.join(self.dir, a.name + ".glb")
        model3d.write_glb(a, glb, origin=e["cg"])
        # the platform finds it where it is (io::AssetResolver: the source tree's
        # aircraft/ in a development build (before any staged copy), share/flightsim/aircraft
        # in a package, or FSIM_AIRCRAFT_PATH): jsbsim:<name> in any trainer, the viewer, Python
        checks = [info("JSBSim aircraft", shown(xml_path), note="type jsbsim:%s" % a.name),
                  info("3D model", shown(glb), note="fsim demo --aircraft %s" % a.name)]
        return self.save("build", {"xml": xml_path, "glb": glb, "checks": checks})

    def calibrated(self, tabs):
        """The tables with the calibration's extra drag area (drag along the
        flow, the same at every attitude) added to the base drag."""
        extra = float(self.aircraft.calibration.get("extra_drag_area_m2", 0.0))
        if not extra:
            return tabs
        out = dict(tabs)
        out["base"] = dict(tabs["base"])
        out["base"]["CD"] = tabs["base"]["CD"] + extra / self.aircraft.S
        return out

    # -- verify ---------------------------------------------------------------------------------
    def verify(self, n=None):
        from . import flight as F
        from . import verify as V
        a = self.aircraft
        n = n or (40 if self.quick else 150)
        tabs = self.calibrated(self.tables())
        f = F.Flight(a.name, name="hangar-verify-" + a.name)
        try:
            rows = V.sample_states(f, n=n)
        finally:
            f.close()
        res = V.compare(rows, tabs, a)
        Fe = np.array([r["err"]["F"] for r in res])
        Me = np.array([r["err"]["M"] for r in res])
        checks = [check("JSBSim force vs tables (max coefficient error)", float(np.abs(Fe).max()), None, 2e-3, fmt="%.1e"),
                  check("JSBSim moment vs tables (max coefficient error)", float(np.abs(Me).max()), None, 2e-3, fmt="%.1e"),
                  info("states compared", len(res))]
        al = np.array([r["alpha_deg"] for r in res])
        be = np.array([r["beta_deg"] for r in res])
        return self.save("verify", {"force_error_max": np.abs(Fe).max(0), "moment_error_max": np.abs(Me).max(0),
                                    "alpha_range": [al.min(), al.max()], "beta_range": [be.min(), be.max()],
                                    "checks": checks})

    # -- fly ------------------------------------------------------------------------------------
    def fly(self, reference=None):
        from . import flight as F
        from .report import plots
        a = self.aircraft
        reference = reference or self.targets.get("reference")
        results = {}
        hists = {}
        for label, kind in [("design", a.name)] + ([("reference", reference)] if reference else []):
            self.log("  fly: %s (%s)" % (label, kind))
            f = F.Flight(kind, name="hangar-fly-%s-%s" % (a.name, label))
            try:
                r, h = self._fly_one(F, f, design=label == "design")
            finally:
                f.close()
            results[label] = r
            hists[label] = h
        # plots: design vs reference
        tags = {lab: "%s (%s)" % (lab, results[lab]["type"]) for lab in results}
        plots.trim_sweep({tags[k]: v["trim_sweep"] for k, v in results.items()}, self.img("fly_trim.png"))
        plots.histories({tags[k]: h["stall"] for k, h in hists.items()}, self.img("fly_stall.png"),
                        "stall: idle, height held, speed bleeding off", ("alt", "kcas", "alpha", "theta", "de", "vs", "cl", "phi"))
        plots.histories({tags[k]: h["long"] for k, h in hists.items()}, self.img("fly_longitudinal.png"),
                        "longitudinal 3-2-1-1 (elevator)", ("tas", "alpha", "q", "theta", "de", "alt"))
        plots.histories({tags[k]: h["lat"] for k, h in hists.items()}, self.img("fly_lateral.png"),
                        "lateral 3-2-1-1 (aileron, then rudder)", ("beta", "p", "r", "phi", "da", "dr"))
        plots.climb({tags[k]: v["climb"] for k, v in results.items()}, self.img("fly_climb.png"))
        checks = self._flight_checks(results)
        return self.save("fly", {"results": results, "checks": checks,
                                 "images": ["fly_trim.png", "fly_climb.png", "fly_stall.png", "fly_longitudinal.png", "fly_lateral.png"]})

    def _aircraft_file(self, kind):
        """The JSBSim file of the design, or of a reference aircraft."""
        name = kind.split(":", 1)[-1]
        if name == self.aircraft.name:
            return os.path.join(self.dir, name + ".xml")
        return os.path.join(repo_root(), "third_party", "jsbsim", "aircraft", name, name + ".xml")

    def _stall_estimate(self):
        """The stall speed the tables predict (sea level, loaded, trimmed CL
        about 85 % of the clean maximum): the scale every test flies at."""
        aero, mass = self.load("aero"), self.load("mass")
        if not aero or not mass:
            return 26.0
        W = mass["loaded_mass_kg"] * 9.80665
        return float(np.sqrt(2 * W / (1.225 * self.aircraft.S * 0.85 * aero["cl_max"])))

    def _fly_one(self, F, f, design=True):
        t0 = time.time()
        r = {"type": f.type}
        vs0 = self._stall_estimate()
        stall_hi, h_stall = F.stall(f, altitude_m=1500.0, start_ms=1.7 * vs0 / 0.93)
        stall_sl, _ = F.stall(f, altitude_m=100.0, start_ms=1.7 * vs0)
        vs = stall_sl["stall_tas_ms"] if np.isfinite(stall_sl["stall_tas_ms"]) else vs0
        speeds = np.linspace(1.15 * vs, 3.6 * vs, 7 if self.quick else 13)
        r["trim_sweep"] = F.trim_sweep(f, speeds, altitude_m=100.0)
        r["max_speed_ms"] = F.max_level_speed(r["trim_sweep"])
        r["stall"] = {k: v for k, v in stall_sl.items() if k != "trim"}
        r["stall_1500"] = {k: v for k, v in stall_hi.items() if k != "trim"}
        r["climb"] = F.climb_performance(f, vs, altitudes=(0.0, 3000.0) if self.quick else (0.0, 1500.0, 3000.0, 4500.0))
        m = F.modes(f, altitude_m=1500.0, speed_ms=1.9 * vs)
        r["modes"] = {k: v for k, v in m.items() if not k.startswith("hist")}
        if design:
            # the linear model from the tables at the same condition: the
            # prediction the checks use (identification needs an oscillation
            # to fit, and a well-damped mode has none)
            from . import linear
            from .mass import MassModel
            lin = linear.model(self.calibrated(self.tables()), self.aircraft, MassModel(self.aircraft), 1.9 * vs, 1500.0)
            r["modes_linear"] = linear.modes(lin)
            r["linear_trim"] = {k: lin[k] for k in ("alpha_trim_deg", "CL", "speed_ms", "altitude_m", "Cma_cg", "Cnb_cg", "Clb_cg")}
        r["robustness"] = F.robustness(f, n=12 if self.quick else 40)
        r["crashes"] = F.crash_tests(f, vs, F.contact_points(self._aircraft_file(f.type)))
        r["seconds"] = time.time() - t0
        return r, {"stall": h_stall, "long": m["hist_long"], "lat": m["hist_lat"]}

    def _flight_checks(self, results):
        t = self.targets
        d = results["design"]
        ref = results.get("reference")
        checks = []

        def vs_target(label, value, key, tol, unit, conv=lambda x: x):
            refv = None
            if ref is not None:
                refv = conv_ref(ref, key)
            note = ("reference %.3g" % refv) if refv is not None and np.isfinite(refv) else ""
            if key in t:
                checks.append(check(label, value, t[key] * (1 - tol), t[key] * (1 + tol), unit,
                                    note=("target %.3g" % t[key]) + ("; " + note if note else "")))
            else:
                checks.append(info(label, value, unit, note))

        def conv_ref(r, key):
            return {"stall_speed_kcas": r["stall"]["stall_kcas"], "max_speed_ktas": r["max_speed_ms"] / KT,
                    "climb_rate_fpm": r["climb"]["rows"][0]["rate_ms"] / 0.3048 * 60 if r["climb"]["rows"] else None,
                    "service_ceiling_ft": r["climb"]["service_ceiling_m"] / 0.3048}.get(key)

        vs_target("stall speed, clean, sea level", d["stall"]["stall_kcas"], "stall_speed_kcas", 0.07, "KCAS")
        vs_target("maximum level speed, sea level", d["max_speed_ms"] / KT, "max_speed_ktas", 0.06, "KTAS")
        rows = d["climb"]["rows"]
        vs_target("best rate of climb, sea level", rows[0]["rate_ms"] / 0.3048 * 60 if rows else float("nan"),
                  "climb_rate_fpm", 0.15, "ft/min")
        vs_target("service ceiling", d["climb"]["service_ceiling_m"] / 0.3048, "service_ceiling_ft", 0.15, "ft")
        top = d["climb"].get("highest_climb_m")
        if top is not None and checks[-1]["value"] is not None:
            checks[-1]["note"] = ("climbed at up to %.0f ft" % (top / 0.3048)) + ("; " + checks[-1]["note"] if checks[-1]["note"] else "")
        m = d.get("modes_linear", d["modes"])
        idm = d["modes"]
        sp, ph, dr = m["short_period"], m["phugoid"], m["dutch_roll"]

        def seen(key, field, fmt="%.3g"):
            v = idm[key].get(field)
            return ("JSBSim response: " + fmt % v) if v is not None and np.isfinite(v) else "JSBSim response: no oscillation to fit"
        # MIL-F-8785C level 1, class I, category B; predicted by the linear model
        checks += [check("short period damping", sp["zeta"], 0.3, 2.0, note="MIL-F-8785C level 1: 0.3-2.0; " + seen("short_period", "zeta")),
                   info("short period frequency", sp["omega_n"], "rad/s", note=seen("short_period", "omega_n")),
                   check("phugoid damping", ph["zeta"], 0.04, None, level="warn", note="MIL-F-8785C level 1: > 0.04; " + seen("phugoid", "zeta")),
                   info("phugoid period", ph["period_s"], "s", note=seen("phugoid", "period_s")),
                   check("dutch roll damping", dr["zeta"], 0.08, None, note="MIL-F-8785C level 1: > 0.08; " + seen("dutch_roll", "zeta")),
                   check("dutch roll frequency", dr["omega_n"], 0.4, None, "rad/s", note="MIL-F-8785C level 1: > 0.4; " + seen("dutch_roll", "omega_n")),
                   check("roll mode time constant", m["roll"]["time_constant_s"], None, 1.4, "s",
                         note="MIL-F-8785C level 1: < 1.4; " + seen("roll", "time_constant_s")),
                   check("spiral time to double", m["spiral"]["time_to_double_s"], 12.0, None, "s", level="warn",
                         note="MIL-F-8785C level 1: > 12 (infinite: spirally stable)"),
                   check("random-state runs that diverged", d["robustness"]["diverged"], None, 0,
                         note="of %d: attitudes, rates, speeds and controls at random" % d["robustness"]["runs"])]
        checks += self._crash_checks(d, ref)
        return checks

    def _crash_checks(self, d, ref):
        """The crash tests: blown up, energy gained on impact, how deep."""
        crashes = d.get("crashes")
        if not crashes:
            return []

        def blown(r):
            return sorted(k for k, c in r["crashes"].items() if c["blew_up"]) if r and r.get("crashes") else []

        def gain(c):
            return c["fastest_after_ms"] / c["impact_ms"] if np.isfinite(c["impact_ms"]) and c["impact_ms"] > 1.0 else 1.0
        cases = ", ".join(crashes)
        mine = blown(d)
        note = "; ".join(filter(None, [", ".join(mine), ("reference: %s" % (", ".join(blown(ref)) or "none")) if ref else ""]))
        worst = max(crashes, key=lambda k: gain(crashes[k]) if not crashes[k]["blew_up"] else 0.0)
        deep = max(crashes, key=lambda k: crashes[k]["deepest_m"])
        return [check("crash tests that blew up", len(mine), None, 0, level="warn",
                      note=(note + "; " if note else "") + "into the ground at idle: " + cases),
                check("crash tests: speed gained on impact", gain(crashes[worst]), None, 1.3, "x", level="warn",
                      note="largest ratio of speed after impact to speed at it (%s); above 1 the contacts add energy, "
                           "as JSBSim's wheels do when one lands sideways" % worst),
                info("crash tests: deepest point below ground", crashes[deep]["deepest_m"], "m", note=deep)]


def _calibrate(d):
    """Fit what the physics cannot know to published performance: an extra
    drag area (struts, cooling, gear legs, antennas, gaps) to the maximum level
    speed and the propeller pitch to the best rate of climb. Damped Newton on
    the two, finite-difference sensitivities from quick flight tests of the
    rebuilt aircraft. The result goes to calibration.toml beside the design -
    reviewable, and deleted to return to the pure estimate."""
    from . import flight as F
    t = d.targets
    a = d.aircraft
    if not a.engines or not ("max_speed_ktas" in t and "climb_rate_fpm" in t):
        return d.save("calibrate", {"checks": [info("nothing to calibrate", "needs max_speed_ktas and climb_rate_fpm targets")]})
    fly = d.load("fly")
    if not fly:
        raise SystemExit("calibrate needs a fly stage first (it takes the climb speed from it)")
    vy = fly["results"]["design"]["climb"]["rows"][0]["speed_ms"]
    target = np.array([t["max_speed_ktas"], t["climb_rate_fpm"]])
    x = np.array([float(a.calibration.get("extra_drag_area_m2", 0.0)), float(a.engines[0].prop_pitch or 0.75 * a.engines[0].prop_diameter)])
    D = a.engines[0].prop_diameter
    lo, hi = np.array([0.0, 0.45 * D]), np.array([0.25 * a.S, 1.3 * D])
    history = []

    def measure(x):
        _write_calibration(d, {"extra_drag_area_m2": float(x[0]), "propeller_pitch_m": [float(x[1])] * len(a.engines)})
        d.aircraft = Aircraft.load(d.path)
        d.build()
        f = F.Flight(a.name, name="hangar-cal-" + a.name)
        try:
            v0 = target[0] * KT
            rows = F.trim_sweep(f, v0 * np.array([0.8, 0.9, 0.97, 1.03, 1.1]), altitude_m=100.0)
            vmax = F.max_level_speed(rows) / KT
            c, _ = F.climb(f, 30.0, vy)
            climb = c["rate_ms"] / 0.3048 * 60
        finally:
            f.close()
        y = np.array([vmax, climb])
        history.append({"extra_drag_area_m2": float(x[0]), "propeller_pitch_m": float(x[1]), "max_speed_kt": vmax, "climb_fpm": climb})
        d.log("  calibrate: drag area %+.3f m2, pitch %.3f m -> %.1f kt, %.0f ft/min" % (x[0], x[1], vmax, climb))
        return y

    y = measure(x)
    steps = np.array([0.02, 0.04])
    trust = np.array([0.12 * a.S * 0.05 / 0.05, 0.12 * D])   # largest step: drag area (m2), pitch (m)
    trust[0] = max(0.05, 0.02 * a.S)
    for _ in range(8):
        err = (target - y) / target
        if np.all(np.isfinite(y)) and np.all(np.abs(err) < 0.015):
            break
        J = np.empty((2, 2))
        for k in range(2):
            xk = x.copy()
            xk[k] += steps[k]
            J[:, k] = (measure(xk) - y) / steps[k]
        if not np.all(np.isfinite(J)):
            break
        step, *_ = np.linalg.lstsq(J, target - y, rcond=None)
        # trust region: no step larger than a physically small change
        scale = np.max(np.abs(step) / trust)
        if scale > 1.0:
            step = step / scale
        x_new = np.clip(x + step, lo, hi)
        y_new = measure(x_new)
        # accept only an improvement (both measured); else shrink the region
        if np.all(np.isfinite(y_new)) and np.sum(((target - y_new) / target) ** 2) < np.sum(((target - y) / target) ** 2):
            x, y = x_new, y_new
        else:
            trust *= 0.5
    measure(x)   # leave the aircraft built at the best point
    _write_calibration(d, {"extra_drag_area_m2": float(x[0]), "propeller_pitch_m": [float(x[1])] * len(a.engines)},
                       note="fitted: max speed %.1f kt (target %g), climb %.0f ft/min (target %g)" % (y[0], target[0], y[1], target[1]))
    d.aircraft = Aircraft.load(d.path)
    d.build()
    checks = [check("maximum level speed after calibration", y[0], 0.985 * target[0], 1.015 * target[0], "KTAS"),
              check("rate of climb after calibration", y[1], 0.97 * target[1], 1.03 * target[1], "ft/min"),
              info("extra drag area", x[0], "m2", note="CD %+.4f: what the build-up does not see" % (x[0] / a.S)),
              info("propeller pitch", x[1], "m", note="pitch/diameter %.2f" % (x[1] / D))]
    return d.save("calibrate", {"calibration": {"extra_drag_area_m2": x[0], "propeller_pitch_m": x[1]}, "history": history,
                                "checks": checks})


def _write_calibration(d, cal, note=""):
    lines = ["# Written by hangar calibrate (%s) to meet the [targets] of %s:" % (time.strftime("%Y-%m-%d %H:%M"),
                                                                              os.path.basename(d.path)),
             "# corrections for what the physics estimate does not see. Delete this file to",
             "# go back to the pure estimate.", "[calibration]",
             "extra_drag_area_m2 = %.4f   # drag along the flow, added to the estimate (m2)" % cal["extra_drag_area_m2"],
             "propeller_pitch_m = [%s]   # geometric pitch at 75 %% radius, per engine" % ", ".join("%.4f" % p for p in cal["propeller_pitch_m"])]
    if note:
        lines.append("# " + note)
    with open(os.path.join(d.dir, "calibration.toml"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def _roughness(tabs):
    """The largest jump between neighbouring table entries in the base
    tables, after removing the local trend - a sign of a numerical glitch."""
    worst, where = 0.0, ""
    a = tabs["alpha"]
    for k, t in tabs["base"].items():
        for j in range(t.shape[1]):
            col = t[:, j]
            d2 = np.abs(col[2:] - 2 * col[1:-1] + col[:-2])
            i = int(np.argmax(d2))
            if d2[i] > worst:
                worst, where = float(d2[i]), "%s at alpha %.0f, beta %.0f" % (k, a[i + 1], tabs["beta"][j])
    return {"worst": worst, "where": where}


def run(design, stages, log=print, reference=None, force=False, quick=False):
    d = Design(design, log=log)
    if quick:
        d.aircraft.spec.setdefault("analysis", {})["quick"] = True
    done = {}
    for st in stages:
        t0 = time.time()
        log("%s: %s" % (d.name, st))
        if st == "aero":
            done[st] = d.aero(force=force)
        elif st == "fly":
            done[st] = d.fly(reference)
        elif st == "report":
            from .report import html
            done[st] = {"path": html.write(d)}
        elif st == "calibrate":
            done[st] = _calibrate(d)
        else:
            done[st] = getattr(d, st)()
        if st != "report":
            _print_checks(log, done[st].get("checks", []))
        log("  (%.1f s)" % (time.time() - t0))
    return d, done


def _print_checks(log, checks):
    marks = {"pass": "ok  ", "warn": "WARN", "fail": "FAIL", "info": "    "}
    for c in checks:
        v = c["value"]
        val = v if isinstance(v, str) else ("-" if v is None else ("%.4g" % v))
        exp = ("  [%s]" % c["expected"]) if c["expected"] else ""
        note = ("  %s" % c["note"]) if c["note"] else ""
        log("  %s %-52s %10s %-6s%s%s" % (marks[c["status"]], c["name"], val, c["unit"], exp, note))
