"""A JSBSim aircraft's aerodynamic coefficients, read from its file - the
reference a design is checked against. JSBSim's f16 carries NASA TP-1538's
wind-tunnel data, so a hangar F-16 can be held against measurement.

The <aerodynamics> section is evaluated as JSBSim does: every function of an
axis summed, tables interpolated linearly and clamped at their ends. With the
dynamic pressure, wing area, span and chord set to 1 the sums are the
coefficients themselves (DRAG, SIDE, LIFT: CD, CY, CL; ROLL, PITCH, YAW: Cl,
Cm, Cn - JSBSim's axes, the same hangar writes).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

AXES = {"DRAG": "CD", "SIDE": "CY", "LIFT": "CL", "ROLL": "Cl", "PITCH": "Cm", "YAW": "Cn"}


class _Table:
    def __init__(self, el):
        self.vars = [(iv.get("lookup", "row"), iv.text.strip()) for iv in el.findall("independentVar")]
        datas = el.findall("tableData")
        if len(self.vars) == 1:
            rows = np.array([[float(x) for x in line.split()] for line in datas[0].text.strip().splitlines() if line.strip()])
            self.x, self.y = rows[:, 0], rows[:, 1]
            self.dim = 1
        elif len(self.vars) == 2:
            self.dim = 2
            self.cols, self.rows, self.data = self._grid(datas[0].text)
        else:
            self.dim = 3
            self.breaks = np.array([float(d.get("breakPoint")) for d in datas])
            self.grids = [self._grid(d.text) for d in datas]

    @staticmethod
    def _grid(text):
        lines = [line.split() for line in text.strip().splitlines() if line.strip()]
        cols = np.array([float(x) for x in lines[0]])
        rows = np.array([float(line[0]) for line in lines[1:]])
        data = np.array([[float(x) for x in line[1:]] for line in lines[1:]])
        return cols, rows, data

    @staticmethod
    def _interp2(cols, rows, data, r, c):
        r = min(max(r, rows[0]), rows[-1])
        c = min(max(c, cols[0]), cols[-1])
        i = int(np.clip(np.searchsorted(rows, r) - 1, 0, len(rows) - 2)) if len(rows) > 1 else 0
        j = int(np.clip(np.searchsorted(cols, c) - 1, 0, len(cols) - 2)) if len(cols) > 1 else 0
        tr = (r - rows[i]) / (rows[i + 1] - rows[i]) if len(rows) > 1 else 0.0
        tc = (c - cols[j]) / (cols[j + 1] - cols[j]) if len(cols) > 1 else 0.0
        at = lambda a, b: data[min(a, len(rows) - 1), min(b, len(cols) - 1)]  # noqa: E731
        return ((1 - tr) * (1 - tc) * at(i, j) + tr * (1 - tc) * at(i + 1, j)
                + (1 - tr) * tc * at(i, j + 1) + tr * tc * at(i + 1, j + 1))

    def value(self, lookup):
        order = {k: lookup(name) for k, name in self.vars}
        if self.dim == 1:
            return float(np.interp(order["row"], self.x, self.y))
        if self.dim == 2:
            return float(self._interp2(self.cols, self.rows, self.data, order["row"], order["column"]))
        t = min(max(order["table"], self.breaks[0]), self.breaks[-1])
        k = int(np.clip(np.searchsorted(self.breaks, t) - 1, 0, len(self.breaks) - 2)) if len(self.breaks) > 1 else 0
        v0 = self._interp2(*self.grids[k], order["row"], order["column"])
        if len(self.breaks) == 1:
            return float(v0)
        v1 = self._interp2(*self.grids[k + 1], order["row"], order["column"])
        w = (t - self.breaks[k]) / (self.breaks[k + 1] - self.breaks[k])
        return float((1 - w) * v0 + w * v1)


class JSBSimAero:
    """The coefficients of a JSBSim aircraft file at a given state."""

    UNARY = {"abs": abs, "sin": math.sin, "cos": math.cos, "tan": math.tan, "sqrt": math.sqrt, "exp": math.exp,
             "floor": math.floor, "ceil": math.ceil}

    def __init__(self, path):
        root = ET.parse(path).getroot()
        aero = root.find("aerodynamics")
        self.name = root.get("name", path)
        self.functions = {f.get("name"): f for f in aero.findall("function")}
        self.axes = {a.get("name"): a.findall("function") for a in aero.findall("axis")}
        self._tables = {}
        m = root.find("metrics")
        self.S = float(m.find("wingarea").text) * 0.09290304
        self.b = float(m.find("wingspan").text) * 0.3048
        self.c = float(m.find("chord").text) * 0.3048

    def _table(self, el):
        t = self._tables.get(id(el))
        if t is None:
            t = self._tables[id(el)] = _Table(el)
        return t

    def _eval(self, el, v):
        tag = el.tag
        kids = [k for k in el if k.tag not in ("description", "independentVar", "tableData")]
        if tag == "function":
            return self._eval(kids[0], v) if kids else 0.0
        if tag == "product":
            out = 1.0
            for k in kids:
                out *= self._eval(k, v)
            return out
        if tag == "sum":
            return sum(self._eval(k, v) for k in kids)
        if tag == "difference":
            vals = [self._eval(k, v) for k in kids]
            return vals[0] - sum(vals[1:])
        if tag == "quotient":
            a, b = (self._eval(k, v) for k in kids[:2])
            return a / b if b else 0.0
        if tag == "pow":
            a, b = (self._eval(k, v) for k in kids[:2])
            return a ** b
        if tag in ("max", "min"):
            vals = [self._eval(k, v) for k in kids]
            return max(vals) if tag == "max" else min(vals)
        if tag in self.UNARY:
            return self.UNARY[tag](self._eval(kids[0], v))
        if tag == "value":
            return float(el.text)
        if tag == "property":
            return self._prop(el.text.strip(), v)
        if tag == "table":
            return self._table(el).value(lambda name: self._prop(name, v))
        raise ValueError("%s: JSBSim function element <%s> is not read" % (self.name, tag))

    def _prop(self, name, v):
        if name in v:
            return v[name]
        f = self.functions.get(name)
        if f is not None:
            return self._eval(f, v)
        return 0.0

    def coefficients(self, alpha_deg, beta_deg=0.0, mach=0.2, p=0.0, q=0.0, r=0.0, props=None):
        """The six coefficients; p, q, r non-dimensional (p b/2V, q c/2V,
        r b/2V); props: other property values (surface positions...)."""
        a, b = math.radians(alpha_deg), math.radians(beta_deg)
        v = {"aero/alpha-rad": a, "aero/alpha-deg": alpha_deg, "aero/beta-rad": b, "aero/beta-deg": beta_deg,
             "aero/mag-beta-rad": abs(b), "velocities/mach": mach, "aero/qbar-psf": 1.0, "metrics/Sw-sqft": 1.0,
             "aero/qbar-area": 1.0, "aero/qbarUW-psf": 1.0, "aero/qbarUV-psf": 1.0, "aero/qbar-induced-psf": 1.0,
             "metrics/bw-ft": 1.0, "metrics/cbarw-ft": 1.0, "aero/bi2vel": 0.5, "aero/ci2vel": 0.5,
             "velocities/p-aero-rad_sec": 2.0 * p, "velocities/q-aero-rad_sec": 2.0 * q,
             "velocities/r-aero-rad_sec": 2.0 * r, "aero/h_b-mac-ft": 100.0, "aero/h_b-cg-ft": 100.0,
             "gear/gear-pos-norm": 0.0}
        v.update(props or {})
        return {AXES[axis]: sum(self._eval(f, v) for f in fs) for axis, fs in self.axes.items() if axis in AXES}

    def derivatives(self, alpha_deg, mach=0.2, props=None, controls=None):
        """Stability derivatives (per rad) at an angle of attack, by central
        differences; controls: {channel: property} to differentiate too."""
        h = 1.0
        c0 = self.coefficients
        out = {}
        up, dn = c0(alpha_deg + h, 0.0, mach, props=props), c0(alpha_deg - h, 0.0, mach, props=props)
        for k in ("CL", "CD", "Cm"):
            out[k + "a"] = (up[k] - dn[k]) / math.radians(2 * h)
        up, dn = c0(alpha_deg, h, mach, props=props), c0(alpha_deg, -h, mach, props=props)
        for k in ("CY", "Cl", "Cn"):
            out[k + "b"] = (up[k] - dn[k]) / math.radians(2 * h)
        d = 0.01
        for rate, keys in (("p", ("CY", "Cl", "Cn")), ("r", ("CY", "Cl", "Cn")), ("q", ("CL", "Cm"))):
            up = c0(alpha_deg, 0.0, mach, props=props, **{rate: d})
            dn = c0(alpha_deg, 0.0, mach, props=props, **{rate: -d})
            for k in keys:
                out[k + rate] = (up[k] - dn[k]) / (2 * d)
        for ch, prop in (controls or {}).items():
            dd = math.radians(2.0)
            up = c0(alpha_deg, 0.0, mach, props=dict(props or {}, **{prop: dd}))
            dn = c0(alpha_deg, 0.0, mach, props=dict(props or {}, **{prop: -dd}))
            for k in ("CL", "Cm", "Cl", "Cn", "CY"):
                out["%s_%s" % (k, ch)] = (up[k] - dn[k]) / (2 * dd)
        return out


# -- a design against its reference ------------------------------------------------------------
CONTROL_KEYS = {"elevator": ("elevator-pos",), "aileron": ("left-aileron-pos", "aileron-pos"), "rudder": ("rudder-pos",)}


def _control_properties(path):
    """The surface positions a reference's aerodynamics read, by channel:
    {channel: (property, radians per unit)}."""
    import re
    text = open(path, encoding="utf-8").read()
    aero = text[text.find("<aerodynamics"):text.find("</aerodynamics>")]
    used = set(re.findall(r"fcs/[a-z0-9_-]+-pos-(?:rad|deg)", aero))
    out = {}
    for ch, keys in CONTROL_KEYS.items():
        for key in keys:
            hit = sorted(p for p in used if p.split("/")[-1].startswith(key) and "mag-" not in p)
            if hit:
                prop = hit[0]
                out[ch] = (prop, 1.0 if prop.endswith("-rad") else math.degrees(1.0))
                break
    return out


def compare(tabs, aircraft, ref_path, moment_point=None, alphas=None, mach=0.2, scale=None):
    """hangar's tables and a JSBSim reference side by side over alpha, both
    in the design's reference geometry (area, span, chord, moment point):
    CL, CD, Cm at beta 0; the sideslip derivatives; damping; control power
    (per rad). moment_point: where the reference's moments are taken, in the
    design frame (default: the design's aerodynamic reference point). scale:
    {channel: factor} for a reference whose control tables are per some
    other unit than its property says."""
    ref = JSBSimAero(ref_path)
    ctl = _control_properties(ref_path)
    alphas = np.arange(-10.0, 50.0 + 1e-9, 1.0) if alphas is None else np.asarray(alphas, float)
    S, b, c = aircraft.S, aircraft.b, aircraft.c
    kf, kl, km = ref.S / S, ref.S * ref.b / (S * b), ref.S * ref.c / (S * c)
    dx = aircraft.aero_point[0] - (aircraft.aero_point[0] if moment_point is None else moment_point[0])

    def coeffs(a, beta=0.0, props=None, **rates):
        v = ref.coefficients(a, beta, mach, props=props, **rates)
        ar = math.radians(a)
        cn = v["CL"] * math.cos(ar) + v["CD"] * math.sin(ar)
        return {"CL": kf * v["CL"], "CD": kf * v["CD"], "CY": kf * v["CY"],
                "Cm": km * v["Cm"] + kf * cn * dx / c, "Cl": kl * v["Cl"], "Cn": kl * v["Cn"] + kf * v["CY"] * dx / b}

    out = {"alpha": alphas, "reference": {}, "design": {}, "name": ref.name}
    R = out["reference"]
    for k in ("CL", "CD", "Cm", "CYb", "Clb", "Cnb", "Clp", "Cnr", "Cmq", "Cm_elevator", "Cl_aileron", "Cn_rudder"):
        R[k] = np.full(len(alphas), np.nan)
    hb, hr, dd = 2.0, 0.02, math.radians(2.0)
    for i, a in enumerate(alphas):
        v = coeffs(a)
        for k in ("CL", "CD", "Cm"):
            R[k][i] = v[k]
        up, dn = coeffs(a, hb), coeffs(a, -hb)
        for k in ("CY", "Cl", "Cn"):
            R[k + "b"][i] = (up[k] - dn[k]) / math.radians(2 * hb)
        for rate, k in (("p", "Cl"), ("r", "Cn"), ("q", "Cm")):
            R[k + rate][i] = (coeffs(a, **{rate: hr})[k] - coeffs(a, **{rate: -hr})[k]) / (2 * hr)
        for ch, k in (("elevator", "Cm"), ("aileron", "Cl"), ("rudder", "Cn")):
            if ch not in ctl:
                continue
            prop, unit = ctl[ch]
            mag = prop.replace("fcs/", "fcs/mag-")
            up = coeffs(a, props={prop: dd * unit, mag: dd * unit})
            dn = coeffs(a, props={prop: -dd * unit, mag: dd * unit})
            R["%s_%s" % (k, ch)][i] = (up[k] - dn[k]) / (2 * dd) * float((scale or {}).get(ch, 1.0))
    # the design, from its tables
    D = out["design"]
    ta, tb, base = tabs["alpha"], tabs["beta"], tabs["base"]
    j0 = int(np.argmin(np.abs(tb)))
    jp, jm = int(np.argmin(np.abs(tb - 3.0))), int(np.argmin(np.abs(tb + 3.0)))
    for k in ("CL", "CD", "Cm"):
        D[k] = np.interp(alphas, ta, base[k][:, j0])
    for k in ("CY", "Cl", "Cn"):
        D[k + "b"] = np.interp(alphas, ta, (base[k][:, jp] - base[k][:, jm]) / math.radians(tb[jp] - tb[jm]))
    for rate, k in (("p", "Cl"), ("r", "Cn"), ("q", "Cm")):
        D[k + rate] = np.interp(alphas, ta, tabs["rates"][rate][k])
    for ch, k in (("elevator", "Cm"), ("aileron", "Cl"), ("rudder", "Cn")):
        t = tabs["controls"].get(ch)
        if t is None or k not in t:
            continue
        d = t["deflection"]
        ip, im = int(np.argmin(np.abs(d - 5.0))), int(np.argmin(np.abs(d + 5.0)))
        D["%s_%s" % (k, ch)] = np.interp(alphas, ta, (t[k][:, ip] - t[k][:, im]) / math.radians(d[ip] - d[im]))
    return out
