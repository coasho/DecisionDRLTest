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

STAGES = ("geometry", "aero", "mass", "propulsion", "build", "model", "verify", "fly", "calibrate", "report")
DEFAULT = ("geometry", "aero", "mass", "propulsion", "build", "model", "verify", "fly", "report")
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
        fighter = a.spec.get("aircraft", {}).get("category") == "fighter"
        checks.append(check("wing aspect ratio", a.b**2 / a.S, 1.5 if fighter else 3.0, 30.0, level="warn",
                            note="fighters 2-3.5, deltas 2-2.5" if fighter else ""))
        for name, d in s["surfaces"].items():
            if d["kind"] == "htail" and "volume_coefficient" in d:
                checks.append(check("%s volume coefficient" % name, abs(d["volume_coefficient"]), 0.2 if fighter else 0.35, 1.1,
                                    level="warn", note="typical 0.5-0.9 (Raymer 6.4); fighters with fly-by-wire less"))
            if d["kind"] == "canard" and "volume_coefficient" in d:
                checks.append(check("%s volume coefficient" % name, abs(d["volume_coefficient"]), 0.05, 0.4, level="warn",
                                    note="close-coupled canards 0.05-0.2"))
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
        key = self.spec_hash("surface", "body", "intake", "reference", "analysis", "gear", "engine")
        cache = os.path.join(self.out, "tables_quick.pkl" if self.quick else "tables.pkl")
        if not force and os.path.isfile(cache):
            with open(cache, "rb") as f:
                saved = pickle.load(f)
            if saved.get("_key") == key:
                return self._with_wave_drag(saved)
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
        return self._with_wave_drag(tabs)

    def _with_wave_drag(self, tabs):
        """The zero-lift drag over Mach with the calibration's wave-drag
        efficiency (cheap; the calibration is not part of the tables' key)."""
        if tabs.get("mach") is not None and "E_WD" in tabs["mach"]:
            from .aero.mach import _drag
            tabs["mach"].update(_drag(self.aircraft, tabs, tabs["mach"]))
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
        band = (al > -5) & (al < 50)
        clmax = float(np.max(base["CL"][band, j0]))
        a_clmax = float(al[band][np.argmax(base["CL"][band, j0])])
        polars = {}
        for k, label in ((0, "wing root"), (int(np.argmax(m.lat.eta * (m.lat.surface_index == 0))), "wing tip")):
            polars["%s (%s, Re %.1e)" % (label, m.lat.airfoils[k].name.split("~")[0], m.polars.polars[k].re)] = m.polars.polars[k]
        plots.polars(polars, self.img("polars.png"), title="section polars of the wing")
        plots.coefficients(tabs, self.img("coefficients.png"), a.name)
        plots.derivatives_vs_alpha(tabs, self.img("derivatives.png"), a.name)
        smooth = _roughness(tabs)
        images = ["coefficients.png", "derivatives.png", "polars.png"]
        fighter = a.spec.get("aircraft", {}).get("category") == "fighter"
        tailless = not any(s.kind in ("htail", "canard", "vtail") for s in a.surfaces)
        # DATCOM 4.1.3.2: C_L_alpha = 2 pi A / (2 + sqrt(A^2 (1 + tan^2 sweep_c/2) + 4))
        sweep_c2 = a.wing.sweep_deg(0.5)
        cla_est = 2 * math.pi * a.wing.aspect_ratio / (2 + math.sqrt(a.wing.aspect_ratio**2 * (1 + math.tan(math.radians(sweep_c2))**2) + 4))
        checks = [
            check("lift-curve slope CL_alpha", d["CLa"], 0.7 * cla_est, 1.45 * cla_est, "/rad",
                  note="the wing alone by DATCOM's formula (aspect ratio %.1f, half-chord sweep %.0f deg): %.2f"
                  % (a.wing.aspect_ratio, sweep_c2, cla_est)),
            check("CL max (untrimmed, clean)", clmax, 1.1, 2.3 if fighter else 2.1,
                  note="at %.0f deg" % a_clmax + ("; leading-edge extensions take a fighter's past 2" if fighter else "")),
            check("pitch stiffness Cm_alpha (about ARP)", d["Cma"], None, -0.1, "/rad", level="warn" if fighter else "fail",
                  note="negative: stable" + ("; a fighter's flight controls make up for relaxed stability" if fighter else "")),
            check("weathercock Cn_beta", d["Cnb"], 0.01, None, "/rad", note="positive: stable"),
            check("dihedral effect Cl_beta", d["Clb"], None, -0.005, "/rad", note="negative: stable"),
            check("roll damping Cl_p", d["Clp"], None, -0.1, "/rad"),
            check("pitch damping Cm_q", d["Cmq"], None, -0.05 if tailless else (-0.5 if fighter else -1.0), "/rad",
                  note="a tailless wing's own" if tailless else ""),
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
        mt = tabs.get("mach")
        if mt is not None:
            plots.mach_effects(mt, self.img("mach.png"), a.name)
            images.append("mach.png")
            i2 = int(np.argmin(np.abs(mt["mach"] - 2.0)))
            checks.append(info("zero-lift drag at Mach %.1f" % mt["mach"][i2], float(mt["CD0"] + mt["dCD0"][i2]),
                               note="drag divergence at Mach %.2f; wave drag from A_max %.2f m2 over %.1f m"
                               % (mt["M_dd"], mt["A_max_m2"], mt["length_m"])))
            checks.append(info("neutral point move to Mach %.1f" % mt["mach"][-1], (mt["x_np"][-1] - mt["x_np"][0]) / a.c * 100,
                               "% MAC"))
        checks += self._reference_aero(tabs, images)
        return self.save("aero", {"derivatives": d, "cl_max": clmax, "alpha_cl_max_deg": a_clmax,
                                  "neutral_point_m": [np_x, 0.0, a.aero_point[2]], "drag_area_m2": m.drag_area,
                                  "lattice": m.lat.summary(), "checks": checks, "images": images})

    def _reference_aero(self, tabs, images):
        """The design's coefficients against its reference aircraft's (a
        JSBSim model from wind-tunnel data), plotted over alpha."""
        from .reference import compare
        from .report import plots
        ref = self.targets.get("reference")
        if not ref or not ref.startswith("jsbsim:"):
            return []
        path = self._aircraft_file(ref)
        if not os.path.isfile(path):
            return [info("reference aerodynamics", "missing", note=shown(path))]
        cmp = compare(tabs, self.aircraft, path, scale=self.targets.get("reference_control_scale"))
        plots.reference_comparison(cmp, self.img("reference.png"), self.aircraft.name)
        images.append("reference.png")
        R, D, al = cmp["reference"], cmp["design"], cmp["alpha"]
        out = []
        for lo, hi in ((0.0, 15.0), (15.0, 40.0)):
            k = (al >= lo) & (al <= hi)
            # the attached range is checked; past it, references are often guesses
            level = "warn" if hi <= 15.0 else "info"
            for key, label in (("CL", "lift"), ("CD", "drag")):
                err = float(np.mean(np.abs(D[key][k] - R[key][k]) / np.maximum(np.abs(R[key][k]), 0.05))) * 100
                name = "%s against %s, alpha %.0f-%.0f deg (mean error)" % (label, ref, lo, hi)
                out.append(check(name, err, None, 15.0, "%", level="warn", fmt="%.1f") if level == "warn" else info(name, err, "%"))
            dcm = float(np.max(np.abs(D["Cm"][k] - R["Cm"][k])))
            name = "pitching moment against %s, alpha %.0f-%.0f deg (largest difference)" % (ref, lo, hi)
            out.append(check(name, dcm, None, 0.08, level="warn", fmt="%.3f") if level == "warn" else info(name, dcm))
        return out

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
            fbw = self.fbw_options() is not None
            checks.append(check("static margin (loaded)", sm * 100, -15.0 if fbw else 5.0, 40.0, "% MAC",
                                level="warn" if fbw else "fail",
                                note="neutral point %.3f m, CG %.3f m%s" % (np_x, cg[0], "; fly-by-wire: relaxed stability allowed"
                                                                            if fbw else "")))
            m_e, cg_e = mm.loaded(fuel_fraction=0.0, payload=False)
            checks.append(check("static margin (empty)", (np_x - cg_e[0]) / a.c * 100, 0.0, 45.0, "% MAC", level="warn"))
        if mm.spec.get("gyration") is not None:
            checks += [info("radii of gyration R_x, R_y, R_z", "%.3f, %.3f, %.3f" % (g["Rx"], g["Ry"], g["Rz"]),
                            note="given in [mass] gyration")]
        else:
            checks += [check("roll gyration radius R_x", g["Rx"], 0.15, 0.45, level="warn", note="Roskam: GA 0.2-0.35, fighters ~0.25"),
                       check("pitch gyration radius R_y", g["Ry"], 0.2, 0.5, level="warn", note="Roskam: GA 0.3-0.45, fighters ~0.38"),
                       check("yaw gyration radius R_z", g["Rz"], 0.25, 0.6, level="warn", note="Roskam: GA 0.35-0.5, fighters ~0.5")]
        checks.append(info("loaded mass", m, "kg"))
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
        jets = [e for e in a.engines if e.type == "turbofan"]
        if jets:
            from .propulsion import turbofan_tables
            thrust = 0.0
            for e in jets:
                length, diameter = e.jet_size()
                out.append({"engine": e.name, "type": "turbofan", "thrust_dry_kn": e.thrust_dry_kn,
                            "thrust_wet_kn": e.thrust_wet_kn, "bypass_ratio": e.bypass_ratio,
                            "throttle_ratio": e.throttle_ratio, "length_m": length, "diameter_m": diameter,
                            "tables": turbofan_tables(e)})
                thrust += (e.thrust_wet_kn or e.thrust_dry_kn) * 1000.0 * len(e.copies())
            checks.append(check("thrust / weight (maximum thrust, loaded)", thrust / W, 0.3, 1.6, level="warn",
                                note="fighters 0.9-1.3"))
            plots.turbofan({e.name: o["tables"] for e, o in zip(jets, out)}, self.img("propulsion.png"))
            return self.save("propulsion", {"engines": out, "checks": checks, "images": ["propulsion.png"]})
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
        return self.build_with(self.tables())

    def build_with(self, tables):
        from . import jsbsim
        from .mass import MassModel
        from .propulsion import Propeller, electric_xml, nozzle_xml, piston_xml, propeller_xml, turbofan_xml
        a = self.aircraft
        tabs = self.calibrated(tables)
        mm = MassModel(a)
        eng_dir = os.path.join(self.dir, "Engines")
        os.makedirs(eng_dir, exist_ok=True)
        files = []
        for i, e in enumerate(a.engines):
            ename = "%s_engine%d" % (a.name, i)
            if e.type == "turbofan":
                pname = "%s_nozzle%d" % (a.name, i)
                with open(os.path.join(eng_dir, ename + ".xml"), "w", encoding="utf-8") as f:
                    f.write(turbofan_xml(e))
                with open(os.path.join(eng_dir, pname + ".xml"), "w", encoding="utf-8") as f:
                    f.write(nozzle_xml(e.name))
                files.append((ename, pname))
                continue
            pname = "%s_prop%d" % (a.name, i)
            with open(os.path.join(eng_dir, ename + ".xml"), "w", encoding="utf-8") as f:
                f.write(piston_xml(e) if e.type == "piston" else electric_xml(e))
            p = Propeller(e)
            mass_e = float(e.mass) if e.mass is not None else 30.0
            with open(os.path.join(eng_dir, pname + ".xml"), "w", encoding="utf-8") as f:
                f.write(propeller_xml(p, p.tables(), 0.1 * mass_e, "%s propeller %d" % (a.name, i)))
            files.append((ename, pname))
        from . import fcs
        fbw = fcs.design(tabs, a, mm)
        xml_path = os.path.join(self.dir, a.name + ".xml")
        with open(xml_path, "w", encoding="utf-8") as f:
            f.write(jsbsim.aircraft_xml(a, tabs, mm, files, fbw=fbw))
        # the platform finds it where it is (io::AssetResolver: the source tree's
        # aircraft/ in a development build (before any staged copy), share/flightsim/aircraft
        # in a package, or FSIM_AIRCRAFT_PATH): jsbsim:<name> in any trainer, the viewer, Python
        checks = [info("JSBSim aircraft", shown(xml_path), note="type jsbsim:%s" % a.name)]
        out = {"xml": xml_path, "checks": checks}
        if fbw is not None:
            checks += self._fbw_checks(fbw)
            from .report import plots
            plots.fbw_gains(fbw, self.img("fbw.png"), a.name)
            out["images"] = ["fbw.png"]
            out["fbw"] = {"options": fbw["options"], "qbar_psf": fbw["qbar_psf"], "mach": fbw["mach"],
                          "gains": fbw["gains"]}
        return self.save("build", out)

    # -- the 3D model ---------------------------------------------------------------------------
    def model_key(self):
        """What the model is made from: the design's geometry and mass (the CG
        is its origin), its paint (paint.toml), the code that shapes, meshes
        and paints it."""
        from .shape import meshkit
        blob = self.spec_hash("surface", "body", "intake", "gear", "engine", "mass", "reference", "aircraft", "dimensions")
        here = os.path.dirname(os.path.abspath(__file__))
        paint = os.path.join(self.aircraft.dir, "paint.toml")
        for f in (sorted(glob.glob(os.path.join(here, "shape", "*.py"))) + [os.path.join(here, "model3d.py"),
                  os.path.join(here, "livery.py")] + ([paint] if os.path.isfile(paint) else [])):
            with open(f, "rb") as fh:
                blob += hashlib.sha1(fh.read()).hexdigest()
        lib = meshkit.library()
        if lib is not None:
            with open(lib._name, "rb") as fh:
                blob += hashlib.sha1(fh.read()).hexdigest()
        return hashlib.sha1(blob.encode()).hexdigest()[:16]

    def model(self, force=False):
        """The design as the viewer's .glb. With hangar's mesher built, the
        airframe is one closed solid, filleted where its parts meet, and each
        control surface a closed solid on its hinge; the checks see that no
        mesh has a crack, a pinch or a loose piece (shape/, native/meshkit)."""
        from . import model3d
        from .mass import MassModel
        a = self.aircraft
        glb = os.path.join(self.dir, a.name + ".glb")
        key = self.model_key()
        old = self.load("model")
        if not force and old and old.get("key") == key and os.path.isfile(glb):
            # the model as it was; its checks as they are now
            self.log("  model: unchanged (%s)" % shown(glb))
            old["checks"] = self._model_checks(old["report"], glb, old.get("seconds", 0.0))
            return self.save("model", old)
        report = {}
        t0 = time.time()
        model3d.write_glb(a, glb, origin=MassModel(a).empty()["cg"], report=report)
        seconds = time.time() - t0
        checks = self._model_checks(report, glb, seconds)
        return self.save("model", {"glb": glb, "key": key, "report": report, "checks": checks, "seconds": seconds})

    def _model_checks(self, report, glb, seconds):
        a = self.aircraft
        checks = [info("3D model", shown(glb), note="fsim demo --aircraft %s" % a.name)]
        if "airframe" not in report:
            checks.append(info("3D model: parts drawn as separate primitives", "-",
                               note="hangar_meshkit is not built: cmake --build --preset ucrt64-release"))
            return checks
        af = report["airframe"]
        defects = af["boundary_edges"] + af["nonmanifold_edges"] + af["misoriented_edges"]
        checks.append(check("3D model: airframe defects (open, pinched or misturned edges)", defects, None, 0,
                            note="%d open, %d pinched, %d misturned" % (af["boundary_edges"], af["nonmanifold_edges"],
                                                                        af["misoriented_edges"])))
        checks.append(check("3D model: airframe pieces", af["components"], 1, 1,
                            note="one closed solid: no loose part, no part floating off the body"))
        checks += self._dimension_checks(af, report["pieces"])
        specks = af.get("fragments_removed", 0) + sum(p.get("fragments_removed", 0) for p in report["pieces"])
        if specks:
            checks.append(info("3D model: specks dropped", specks,
                               note="pieces a few mesh cells across, where two cuts almost meet"))
        bad = [p["label"] for p in report["pieces"]
               if p["boundary_edges"] + p["nonmanifold_edges"] + p["misoriented_edges"] or p["components"] != 1]
        checks.append(check("3D model: control surfaces not closed and whole", len(bad), None, 0,
                            note=", ".join(bad) or "%d, each one closed solid" % len(report["pieces"])))
        gear = report.get("gear", [])
        if gear:
            bad = [g["label"] for g in gear
                   if g["boundary_edges"] + g["nonmanifold_edges"] + g["misoriented_edges"] or g["components"] != 1]
            checks.append(check("3D model: landing gear parts not closed and whole", len(bad), None, 0,
                                note=", ".join(bad) or "%d struts, oleos, wheels and doors, each one closed solid" % len(gear)))
            legs = [g for g in gear if "protrusion" in g]
            if legs:
                worst = max(legs, key=lambda g: g["protrusion"])
                checks.append(check("3D model: stowed gear outside the skin", 100.0 * worst["protrusion"], None, 3.0, "cm",
                                    level="warn", note="%s; the leg must fold into the airframe" % worst["label"]))
            pairs = report.get("door_clearance", [])
            if pairs:
                # a door never touches a leg: open while the leg swings, closed
                # over it stowed
                w_open = min(pairs, key=lambda p: p["open_m"])
                w_shut = min(pairs, key=lambda p: p["closed_m"])
                checks.append(check("3D model: open gear doors' clearance from the legs", 1000.0 * w_open["open_m"], 0.0,
                                    None, "mm", note="%s and %s, %.0f %% of the way up" % (
                                        w_open["door"], w_open["leg"], 100.0 * w_open["at"])))
                checks.append(check("3D model: closed gear doors' clearance from the stowed legs",
                                    1000.0 * w_shut["closed_m"], 0.0, None, "mm",
                                    note="%s and %s" % (w_shut["door"], w_shut["leg"])))
        tris = af["triangles"] + sum(p["triangles"] for p in report["pieces"] + gear)
        degen = af["degenerate_triangles"] + sum(p["degenerate_triangles"] for p in report["pieces"] + gear)
        checks.append(check("3D model: degenerate triangles", 100.0 * degen / max(tris, 1), None, 0.2, "%",
                            level="warn", note="%d of %d" % (degen, tris)))
        checks.append(info("3D model: triangles", tris, note="%.1f MB, meshed in %.0f s" % (os.path.getsize(glb) / 1e6, seconds)))
        return checks

    def _dimension_checks(self, af, pieces=()):
        """The model's overall length, span and height (gear down, parked:
        on its wheels, each strut compressed by its static deflection, its
        control surfaces - an all-moving fin - at rest) against the published
        ones in [dimensions]: within 3 %."""
        dims = self.aircraft.spec.get("dimensions", {})
        if not dims or "bounds" not in af:
            return []
        boxes = [af["bounds"]] + [p["bounds"] for p in pieces if "bounds" in p]
        lo = np.min([b[0] for b in boxes], axis=0)
        hi = np.max([b[1] for b in boxes], axis=0)
        ground = min((float(p[2]) + g.static_deflection for g in self.aircraft.gear for _, p in g.positions()),
                     default=float(lo[2]))
        got = {"length": hi[0] - lo[0], "span": hi[1] - lo[1], "height": hi[2] - ground}
        out = []
        for key in ("length", "span", "height"):
            if key in dims:
                want = float(dims[key])
                out.append(check("3D model: overall %s" % key, got[key], 0.97 * want, 1.03 * want, "m", level="warn",
                                 note="published %.2f m (%+.1f %%)" % (want, 100.0 * (got[key] / want - 1.0)), fmt="%.2f"))
        return out

    def _fbw_checks(self, fbw):
        """The fly-by-wire's short period where the airframe can fly (1 g
        trim below the angle-of-attack limit): damping and frequency."""
        zetas, omegas, unstable = [], [], 0
        for row in fbw["points"]:
            for p in row:
                if p["alpha_deg"] > fbw["options"]["alpha_max_deg"] - 2.0:
                    continue
                e = np.array(p["eig_sp"])
                w = np.abs(e)
                z = -e.real / np.maximum(w, 1e-9)
                zetas.append(float(z.min()))
                omegas.append(float(w.min()))
                unstable += int(p["open_loop_Ma"] > 0)
        return [check("fly-by-wire short period damping (worst design point)", min(zetas), 0.35, 1.3,
                      note="MIL-F-8785C level 1, category A"),
                info("fly-by-wire short period frequency", "%.1f-%.1f" % (min(omegas), max(omegas)), "rad/s"),
                info("design points where the airframe alone is unstable in pitch", unstable,
                     note="of %d" % len(zetas))]

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
        opts = self.fbw_options()
        for label, kind in [("design", a.name)] + ([("reference", reference)] if reference else []):
            self.log("  fly: %s (%s)" % (label, kind))
            f = F.Flight(kind, name="hangar-fly-%s-%s" % (a.name, label))
            try:
                if opts is not None:
                    results[label] = self._fly_fighter(F, f, opts)
                    continue
                r, h = self._fly_one(F, f, design=label == "design")
            finally:
                f.close()
            results[label] = r
            hists[label] = h
        if opts is not None:
            tags = {lab: "%s (%s)" % (lab, results[lab]["type"]) for lab in results}
            plots.fighter({tags[k]: v["fighter"] for k, v in results.items()}, self.img("fly_fighter.png"), a.name)
            checks = self._fighter_checks(results)
            return self.save("fly", {"results": results, "checks": checks, "images": ["fly_fighter.png"]})
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

    def _fly_fighter(self, F, f, opts):
        """The fighter tests (flight.fighter_tests) and the crash and random
        state runs every design flies."""
        t0 = time.time()
        r = {"type": f.type, "fighter": F.fighter_tests(f, opts, quick=self.quick)}
        r["robustness"] = F.robustness(f, n=12 if self.quick else 40)
        vs = 70.0
        r["crashes"] = F.crash_tests(f, vs, F.contact_points(self._aircraft_file(f.type)))
        r["seconds"] = time.time() - t0
        return r

    def _fighter_checks(self, results):
        t = self.targets
        d = results["design"]["fighter"]
        ref = results.get("reference", {}).get("fighter")
        checks = []

        def vs(label, value, key, tol, unit, fmt="%.3g", refv=None):
            note = ("reference %s" % (fmt % refv)) if refv is not None and np.isfinite(refv) else ""
            if key in t:
                checks.append(check(label, value, t[key] * (1 - tol), t[key] * (1 + tol), unit, level="warn",
                                    note=("target %g" % t[key]) + ("; " + note if note else ""), fmt=fmt))
            else:
                checks.append(info(label, value, unit, note))
        g = lambda k, r=ref: r.get(k) if r else None  # noqa: E731
        vs("maximum Mach number at 36,000 ft (full afterburner)", d["max_mach_36k"], "max_mach", 0.05, "", refv=g("max_mach_36k"))
        vs("maximum level speed at sea level", d["max_mach_sl"] * 661.47, "max_speed_ktas", 0.08, "KTAS", "%.0f",
           refv=g("max_mach_sl") * 661.47 if ref else None)
        vs("best rate of climb at sea level (peak excess power)", d["climb_rate_ms"] / 0.3048 * 60, "climb_rate_fpm", 0.25, "ft/min",
           "%.0f", refv=g("climb_rate_ms") / 0.3048 * 60 if ref else None)
        vs("service ceiling", d["service_ceiling_m"] / 0.3048, "service_ceiling_ft", 0.15, "ft", "%.0f",
           refv=g("service_ceiling_m") / 0.3048 if ref else None)
        vs("sustained turn rate, Mach 0.9 at 15,000 ft", d["turn"]["rate_deg_s"], "sustained_turn_deg_s", 0.15, "deg/s",
           refv=ref["turn"]["rate_deg_s"] if ref else None)
        h = d["handling"]
        o = self.fbw_options()
        checks += [check("full aft stick at 350 kt: angle of attack reached", h["pull"]["alpha_max"], None, o["alpha_max_deg"] + 4.0,
                         "deg", note="the limiter holds %g deg" % o["alpha_max_deg"]),
                   info("full aft stick at 350 kt: load factor and turn rate", "%.1f g, %.1f deg/s" % (h["pull"]["n_max"],
                                                                                                     h["pull"]["rate_deg_s"])),
                   check("3 g step: overshoot", h["step"]["overshoot"] * 100, None, 40.0, "%", level="warn",
                         note="rise to 90 %% in %.2f s" % h["step"]["rise_s"]),
                   info("full-stick roll at 350 kt", "%.0f deg/s, 90 deg in %.2f s" % (h["roll"]["p_max"], h["roll"]["time_to_90_s"]),
                        note="sideslip up to %.1f deg" % h["roll"]["beta_max"]),
                   check("random-state runs that diverged", results["design"]["robustness"]["diverged"], None, 0,
                         note="of %d: attitudes, rates, speeds and controls at random" % results["design"]["robustness"]["runs"])]
        checks += self._crash_checks(results["design"], results.get("reference"))
        return checks

    def fbw_options(self):
        from .fcs import options
        return options(self.aircraft)

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
                      note="largest ratio of speed after impact to speed at it (%s); above 1 the contacts add energy" % worst),
                info("crash tests: deepest point below ground", crashes[deep]["deepest_m"], "m", note=deep)]


def _calibrate_fighter(d):
    """A fighter's engine and supersonic drag to its published top speed.
    First the engines' throttle ratio TR (where the turbine reaches its
    temperature limit: how much thrust is left at high Mach), between 1.0
    and 1.5; if even 1.5 falls short, then the wave drag's E_WD (Raymer
    12.46), from the area distribution's 2 down to 1.2, a smooth one's.
    Each is a bisection on the maximum level Mach number at the target
    altitude, the aircraft rebuilt and accelerated at full afterburner every
    step."""
    from . import flight as F
    from .fcs import options
    t = d.targets
    a = d.aircraft
    if "max_mach" not in t:
        return d.save("calibrate", {"checks": [info("nothing to calibrate", "needs a max_mach target")]})
    alt = float(t.get("max_mach_altitude_ft", 36000.0)) * 0.3048
    target = float(t["max_mach"])
    opts = options(a)
    history = []
    ewd0 = float(a.spec.get("analysis", {}).get("wave_drag_efficiency", 2.0))

    def measure(tr, ewd):
        _write_calibration(d, {"throttle_ratio": float(tr), "wave_drag_efficiency": float(ewd)})
        d.aircraft = Aircraft.load(d.path)
        d.build()
        f = F.Flight(a.name, name="hangar-cal-" + a.name)
        try:
            pilot = F.FighterPilot(f.dt, opts["n_max"], opts["n_min"]) if opts else F.FighterPilot(f.dt)
            m, _ = F.max_level_mach(f, pilot, alt, start_mach=0.9, seconds=420.0)
        finally:
            f.close()
        history.append({"throttle_ratio": float(tr), "wave_drag_efficiency": float(ewd), "max_mach": float(m)})
        d.log("  calibrate: TR %.3f, E_WD %.2f -> Mach %.3f at %.0f ft" % (tr, ewd, m, alt / 0.3048))
        return m

    def bisect(f_of, lo, hi, m_lo, m_hi):
        x, m = (lo, m_lo) if abs(m_lo - target) < abs(m_hi - target) else (hi, m_hi)
        if (m_lo - target) * (m_hi - target) >= 0:
            return x, m
        for _ in range(8):
            x = 0.5 * (lo + hi)
            m = f_of(x)
            if abs(m - target) < 0.01:
                break
            if (m - target) * (m_lo - target) > 0:
                lo, m_lo = x, m
            else:
                hi, m_hi = x, m
        return x, m

    tr_lo, tr_hi = 1.0, 1.5
    m_lo, m_hi = measure(tr_lo, ewd0), measure(tr_hi, ewd0)
    tr, m = bisect(lambda x: measure(x, ewd0), tr_lo, tr_hi, m_lo, m_hi)
    ewd = ewd0
    if m_hi < target - 0.01:
        # the engine alone cannot: less wave drag, down to a smooth area distribution's
        tr = tr_hi
        m_e = measure(tr, 1.2)
        ewd, m = bisect(lambda x: measure(tr, x), 1.2, ewd0, m_e, m_hi)
    _write_calibration(d, {"throttle_ratio": float(tr), "wave_drag_efficiency": float(ewd)},
                       note="fitted: maximum Mach %.3f at %.0f ft (target %g)" % (m, alt / 0.3048, target))
    d.aircraft = Aircraft.load(d.path)
    d.build()
    # a miss is a warning: both corrections at their bounds is an engine or a
    # drag estimate that falls short, which the fly stage's checks then show
    checks = [check("maximum Mach number after calibration", m, target - 0.03, target + 0.03, level="warn",
                    note="at %.0f ft" % (alt / 0.3048)),
              check("engine throttle ratio TR", tr, 1.0, 1.5, level="warn",
                    note="theta0 where the turbine reaches its limit: Mach %.2f at 36,000 ft" % math.sqrt(max((tr / 0.7519 - 1.0) / 0.2, 0.0))),
              check("wave-drag efficiency E_WD", ewd, 1.2, 3.5, level="warn", note="Raymer: 1.2 a smooth area distribution, 2-3 typical")]
    return d.save("calibrate", {"calibration": {"throttle_ratio": tr, "wave_drag_efficiency": ewd}, "history": history,
                                "checks": checks})


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
    if any(e.type == "turbofan" for e in a.engines):
        return _calibrate_fighter(d)
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
             "# go back to the pure estimate.", "[calibration]"]
    if "extra_drag_area_m2" in cal:
        lines.append("extra_drag_area_m2 = %.4f   # drag along the flow, added to the estimate (m2)" % cal["extra_drag_area_m2"])
    if "propeller_pitch_m" in cal:
        lines.append("propeller_pitch_m = [%s]   # geometric pitch at 75 %% radius, per engine"
                     % ", ".join("%.4f" % p for p in cal["propeller_pitch_m"]))
    if "wave_drag_efficiency" in cal:
        lines.append("wave_drag_efficiency = %.4f   # Raymer's E_WD: the wave drag over Sears-Haack's" % cal["wave_drag_efficiency"])
    if "throttle_ratio" in cal:
        lines.append("throttle_ratio = %.4f   # the engines' TR: the compressor-face temperature ratio at the turbine's limit"
                     % cal["throttle_ratio"])
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
