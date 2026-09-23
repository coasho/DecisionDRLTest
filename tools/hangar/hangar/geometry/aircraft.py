"""An aircraft design: its spec file and the geometry built from it.

The spec is TOML (aircraft/<name>/<name>.toml). Coordinates are metres in the
design frame - x aft, y right, z up - which is JSBSim's structural frame, so
positions go into the JSBSim file unchanged (as unit="M").
"""
import os
import tomllib

import numpy as np

from .body import Body
from .surface import Surface


class Gear:
    def __init__(self, spec):
        self.name = spec.get("name", "gear")
        if "position" not in spec:
            raise ValueError("gear %r: 'position' (the wheel's ground contact point) is required" % self.name)
        self.position = np.asarray(spec["position"], float)
        self.mirror = bool(spec.get("mirror", False))
        self.wheel_diameter = float(spec.get("wheel_diameter", 0.38))
        self.wheel_width = float(spec.get("wheel_width", 0.13))
        self.steerable = bool(spec.get("steerable", False))
        self.max_steer_deg = float(spec.get("max_steer", 10.0 if self.steerable else 0.0))
        self.brake = spec.get("brake", "auto")  # auto: mains brake, nose/tail wheels do not
        self.retractable = bool(spec.get("retractable", False))
        self.fairing = bool(spec.get("fairing", False))
        self.attach = np.asarray(spec["attach"], float) if "attach" in spec else None
        self.static_deflection = float(spec.get("static_deflection", 0.10))

    def positions(self):
        if self.mirror:
            p = self.position.copy()
            q = p * np.array([1.0, -1.0, 1.0])
            return [("Left " + self.name, q), ("Right " + self.name, p)] if p[1] > 0 else [(self.name, p)]
        return [(self.name, self.position)]


class Engine:
    def __init__(self, spec):
        self.name = spec.get("name", "engine")
        self.type = spec.get("type", "piston")
        if self.type not in ("piston", "electric"):
            raise ValueError("engine %r: type must be piston or electric (turbines are not built in yet)" % self.name)
        if "power_kw" not in spec:
            raise ValueError("engine %r: power_kw is required" % self.name)
        self.power_kw = float(spec["power_kw"])
        self.rpm = float(spec.get("rpm", 2700.0))
        self.position = np.asarray(spec.get("position", [0.0, 0.0, 0.0]), float)
        self.mass = spec.get("mass")
        self.mirror = bool(spec.get("mirror", False))
        prop = spec.get("propeller", {})
        self.prop_position = np.asarray(prop.get("position", self.position), float)
        self.prop_diameter = float(prop.get("diameter", 1.8))
        self.prop_blades = int(prop.get("blades", 2))
        self.prop_pitch = prop.get("pitch")  # geometric pitch (m) at 75 % radius, fixed pitch
        self.prop_sense = prop.get("rotation", "cw")  # seen from behind: cw = JSBSim sense 1
        self.prop_orient = np.radians(np.asarray(prop.get("orient", [0.0, 0.0, 0.0]), float))  # roll, pitch, yaw
        self.prop_spec = prop

    def copies(self):
        if self.mirror:
            flip = np.array([1.0, -1.0, 1.0])
            return [(self.name + " L", self.position * flip, self.prop_position * flip, "ccw" if self.prop_sense == "cw" else "cw"),
                    (self.name + " R", self.position, self.prop_position, self.prop_sense)]
        return [(self.name, self.position, self.prop_position, self.prop_sense)]


class Aircraft:
    """The parsed design. `spec` keeps the raw TOML for the later stages
    (mass, propulsion, targets)."""

    def __init__(self, spec, path=None):
        self.path = path
        self.dir = os.path.dirname(os.path.abspath(path)) if path else os.getcwd()
        self.spec = spec
        a = spec.get("aircraft", {})
        self.name = a.get("name") or (os.path.splitext(os.path.basename(path))[0] if path else "design")
        self.description = a.get("description", "")
        self.category = a.get("category", "light_ga")
        self.surfaces = [Surface(s, self.dir) for s in spec.get("surface", [])]
        self.bodies = [Body(b) for b in spec.get("body", [])]
        self.gear = [Gear(g) for g in spec.get("gear", [])]
        self.engines = [Engine(e) for e in spec.get("engine", [])]
        if not self.surfaces:
            raise ValueError("%s: no [[surface]] defined" % self.name)
        wings = [s for s in self.surfaces if s.kind == "wing"]
        self.wing = wings[0] if wings else self.surfaces[0]
        ref = spec.get("reference", {})
        mac, mac_le = self.wing.mac
        self.S = float(ref.get("area", self.wing.area))
        self.b = float(ref.get("span", self.wing.span))
        self.c = float(ref.get("chord", mac))
        self.mac_le = mac_le
        default_rp = np.array([mac_le[0] + 0.25 * mac, 0.0, mac_le[2]])
        self.aero_point = np.asarray(ref.get("aero_point", default_rp), float)
        self.calibration = {}

    @classmethod
    def load(cls, path):
        with open(path, "rb") as f:
            spec = tomllib.load(f)
        a = cls(spec, path)
        cal = os.path.join(os.path.dirname(os.path.abspath(path)), "calibration.toml")
        a.calibration = {}
        if os.path.isfile(cal):
            with open(cal, "rb") as f:
                a.calibration = tomllib.load(f).get("calibration", {})
            for e, pitch in zip(a.engines, a.calibration.get("propeller_pitch_m", [])):
                e.prop_pitch = float(pitch)
        return a

    # -- components -----------------------------------------------------------------------------
    def surface(self, kind):
        return [s for s in self.surfaces if s.kind == kind]

    def controls(self):
        """(surface, control) pairs of every control surface."""
        return [(s, c) for s in self.surfaces for c in s.controls]

    def channels(self):
        return sorted({c.channel for _, c in self.controls()})

    def summary(self):
        """The geometric numbers a designer checks first."""
        out = {
            "reference": {"area_m2": self.S, "span_m": self.b, "chord_m": self.c,
                          "aero_point_m": [round(v, 4) for v in self.aero_point]},
            "surfaces": {},
            "bodies": {},
        }
        for s in self.surfaces:
            mac, le = s.mac
            out["surfaces"][s.name] = {
                "kind": s.kind, "area_m2": s.area, "span_m": s.span, "aspect_ratio": s.aspect_ratio,
                "taper": s.taper, "mac_m": mac, "mac_le_m": [round(v, 4) for v in le],
                "sweep_le_deg": s.sweep_deg(0.0), "sweep_c4_deg": s.sweep_deg(0.25), "dihedral_deg": s.dihedral_deg,
                "thickness": s.thickness_ratio,
                "airfoils": sorted({sec.airfoil.name for sec in s.sections}),
                "controls": {c.name: {"channel": c.channel, "span": [c.eta0, c.eta1],
                                      "chord_fraction": [c.cf0, c.cf1], "limits_deg": [c.min_deg, c.max_deg]}
                             for c in s.controls},
            }
        for b in self.bodies:
            top, side = b.planform_area
            out["bodies"][b.name] = {"length_m": b.length, "max_width_m": b.max_width, "max_height_m": b.max_height,
                                     "volume_m3": b.volume, "fineness": b.fineness, "top_area_m2": top,
                                     "side_area_m2": side, "copies": len(b.copies())}
        # tail volume coefficients: the classic sizing check
        xw = self.aero_point[0]
        for s in self.surfaces:
            if s.kind in ("htail", "canard"):
                mac, le = s.mac
                arm = (le[0] + 0.25 * mac) - xw
                out["surfaces"][s.name]["arm_m"] = arm
                out["surfaces"][s.name]["volume_coefficient"] = s.area * arm / (self.S * self.c)
            elif s.kind in ("fin", "vtail"):
                mac, le = s.mac
                arm = (le[0] + 0.25 * mac) - xw
                out["surfaces"][s.name]["arm_m"] = arm
                out["surfaces"][s.name]["volume_coefficient"] = s.area * arm / (self.S * self.b)
        return out

    def mesh(self, fine=True):
        """Every component's skin: list of (name, kind, vertices, triangles, tags)."""
        parts = []
        for s in self.surfaces:
            v, t, g = s.skin(48 if fine else 20, 28 if fine else 14)
            parts.append((s.name, s.kind, v, t, g))
        for b in self.bodies:
            v, t, g = b.skin(72 if fine else 32, 40 if fine else 20)
            parts.append((b.name, b.kind, v, t, g))
        return parts

    def extent(self):
        pts = np.vstack([p[2] for p in self.mesh(fine=False)])
        return pts.min(axis=0), pts.max(axis=0)
