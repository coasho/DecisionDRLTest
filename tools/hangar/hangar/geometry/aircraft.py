"""An aircraft design: its spec file and the geometry built from it.

The spec is TOML (aircraft/<name>/<name>.toml). Coordinates are metres in the
design frame - x aft, y right, z up - which is JSBSim's structural frame, so
positions go into the JSBSim file unchanged (as unit="M").
"""
import os
import tomllib

import numpy as np

from .body import Body, Intake
from .surface import Surface


class Strut:
    """A bracing strut ([[strut]]): a streamlined bar from one point to
    another, a high wing's lift strut. Drawn only: its drag is part of the
    design's extra drag area."""

    def __init__(self, spec):
        self.name = spec.get("name", "strut")
        if "from" not in spec or "to" not in spec:
            raise ValueError("strut %r: 'from' and 'to' (its ends) are required" % self.name)
        self.a = np.asarray(spec["from"], float)
        self.b = np.asarray(spec["to"], float)
        self.chord = float(spec.get("chord", 0.15))
        self.thickness = float(spec.get("thickness", 0.35 * self.chord))
        self.mirror = bool(spec.get("mirror", True))
        if np.linalg.norm(self.b - self.a) < 1e-6 or not 0.0 < self.thickness <= self.chord:
            raise ValueError("strut %r: its ends must differ, its thickness lie in (0, chord]" % self.name)


class Gear:
    def __init__(self, spec):
        self.spec = spec
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
    """A piston engine or electric motor with its propeller, or a turbofan
    (with or without afterburner) and its nozzle. For a turbofan the
    "propeller" fields describe the nozzle: where the thrust acts and which
    way it points, and there is no disc."""

    def __init__(self, spec):
        self.name = spec.get("name", "engine")
        self.type = spec.get("type", "piston")
        if self.type not in ("piston", "electric", "turbofan"):
            raise ValueError("engine %r: type must be piston, electric or turbofan" % self.name)
        self.position = np.asarray(spec.get("position", [0.0, 0.0, 0.0]), float)
        self.mass = spec.get("mass")
        self.mirror = bool(spec.get("mirror", False))
        if self.type == "turbofan":
            if "thrust_dry_kn" not in spec:
                raise ValueError("engine %r: a turbofan needs thrust_dry_kn (sea-level static, military power)" % self.name)
            self.thrust_dry_kn = float(spec["thrust_dry_kn"])
            self.thrust_wet_kn = float(spec["thrust_wet_kn"]) if "thrust_wet_kn" in spec else None  # afterburner
            if self.thrust_wet_kn is not None and self.thrust_wet_kn <= self.thrust_dry_kn:
                raise ValueError("engine %r: thrust_wet_kn must exceed thrust_dry_kn" % self.name)
            self.bypass_ratio = float(spec.get("bypass_ratio", 0.5))
            self.tsfc_dry = float(spec.get("tsfc_dry", 0.8))    # kg/(kgf h) = lb/(lbf h)
            self.tsfc_wet = float(spec.get("tsfc_wet", 2.0))
            self.throttle_ratio = float(spec.get("throttle_ratio", 1.2))   # Mattingly's TR: where the lapse turns
            self.design_mach = float(spec.get("design_mach", 2.0))
            self.inlet_x = float(spec["inlet_x"]) if "inlet_x" in spec else None  # where the intake captures its air
            self.power_kw = 0.0
            self.rpm = 0.0
            nozzle = spec.get("nozzle", {})
            self.prop_position = np.asarray(nozzle.get("position", self.position), float)
            self.prop_diameter = 0.0
            self.prop_blades = 0
            self.prop_pitch = None
            self.prop_sense = "cw"
            self.prop_orient = np.radians(np.asarray(nozzle.get("orient", [0.0, 0.0, 0.0]), float))
            self.prop_spec = nozzle
            # thrust vectoring: the nozzle turns up to `vectoring` deg each way in
            # one plane, canted `vectoring_cant` deg outboard from the vertical
            # (0: pitch only, the F-22's; the Su-57's 32 deg give yaw and roll too)
            self.vectoring = float(nozzle.get("vectoring", 0.0))
            self.vectoring_cant = float(nozzle.get("vectoring_cant", 0.0))
            if not 0.0 <= self.vectoring <= 45.0:
                raise ValueError("engine %r: nozzle vectoring must be 0-45 deg" % self.name)
            if not 0.0 <= self.vectoring_cant < 90.0:
                raise ValueError("engine %r: nozzle vectoring_cant must be 0-90 deg" % self.name)
            return
        self.vectoring = 0.0
        self.vectoring_cant = 0.0
        self.inlet_x = None
        if "power_kw" not in spec:
            raise ValueError("engine %r: power_kw is required" % self.name)
        self.power_kw = float(spec["power_kw"])
        self.rpm = float(spec.get("rpm", 2700.0))
        prop = spec.get("propeller", {})
        self.prop_position = np.asarray(prop.get("position", self.position), float)
        self.prop_diameter = float(prop.get("diameter", 1.8))
        self.prop_blades = int(prop.get("blades", 2))
        self.prop_pitch = prop.get("pitch")  # geometric pitch (m) at 75 % radius, fixed pitch
        self.prop_sense = prop.get("rotation", "cw")  # seen from behind: cw = JSBSim sense 1
        self.prop_orient = np.radians(np.asarray(prop.get("orient", [0.0, 0.0, 0.0]), float))  # roll, pitch, yaw
        self.prop_spec = prop

    @property
    def has_propeller(self):
        return self.type != "turbofan"

    def jet_size(self):
        """Length and diameter (m) of an afterburning turbofan from its
        thrust (Raymer 10.1-10.2, scaled: L = 0.255 T^0.4 M^0.2 ft, D = 0.024
        T^0.5 e^(0.04 BPR) ft, T in lbf)."""
        t = (self.thrust_wet_kn or self.thrust_dry_kn) * 1000.0 / 4.448222
        return (0.255 * t ** 0.4 * self.design_mach ** 0.2 * 0.3048,
                0.024 * t ** 0.5 * np.exp(0.04 * self.bypass_ratio) * 0.3048)

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
        self.bodies = [Body(b) for b in spec.get("body", [])] + [Intake(i) for i in spec.get("intake", [])]
        self.gear = [Gear(g) for g in spec.get("gear", [])]
        self.engines = [Engine(e) for e in spec.get("engine", [])]
        self.struts = [Strut(s) for s in spec.get("strut", [])]
        if not self.surfaces:
            raise ValueError("%s: no [[surface]] defined" % self.name)
        schedules = sorted({tuple(d.schedule) for _, d in self.leading_devices()})
        if len(schedules) > 1:
            raise ValueError("%s: the leading-edge devices follow one schedule (the flight controls' "
                             "fcs/lef-pos-deg), not %s" % (self.name, " and ".join(map(str, schedules))))
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
            if "throttle_ratio" in a.calibration:
                for e in a.engines:
                    if e.type == "turbofan":
                        e.throttle_ratio = float(a.calibration["throttle_ratio"])
        return a

    # -- components -----------------------------------------------------------------------------
    def surface(self, kind):
        return [s for s in self.surfaces if s.kind == kind]

    def controls(self):
        """(surface, control) pairs of every control surface."""
        return [(s, c) for s in self.surfaces for c in s.controls]

    def leading_devices(self):
        """(surface, device) pairs of every leading-edge flap or slat."""
        return [(s, d) for s in self.surfaces for d in s.leading]

    @property
    def span_overall(self):
        """Tip-to-tip span of the widest mirrored surface (m)."""
        return max([s.span for s in self.surfaces if s.mirror] or [self.b])

    @property
    def length_overall(self):
        """Nose to tail, over the bodies and surfaces (m)."""
        xs = [b.x[0] for b in self.bodies] + [b.x[-1] for b in self.bodies]
        for s in self.surfaces:
            xs += [sec.le[0] for sec in s.sections] + [sec.le[0] + sec.chord for sec in s.sections]
        return float(max(xs) - min(xs)) if xs else 5.0 * self.c

    def channels(self):
        return sorted({ch for _, c in self.controls() for ch in c.channels})

    def channel_limits(self, channel):
        """(min, max) deflection (deg) of a channel: as far as the controls it
        drives first follow it (one with gain g, to its limits / g; each stops
        at its own limits), else as far as those that mix it in."""
        own = [c for _, c in self.controls() if c.channel == channel]
        mixed = [c for _, c in self.controls() if channel in c.mix]
        spans = [sorted((c.min_deg / c.channels[channel], c.max_deg / c.channels[channel]))
                 for c in own or mixed if abs(c.channels[channel]) > 1e-9]
        return min(s[0] for s in spans), max(s[1] for s in spans)

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
