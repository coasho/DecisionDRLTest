"""Lifting surfaces: wings, tails, canards, fins.

A surface is a chain of sections (leading-edge point, chord, twist, airfoil)
in the design frame (metres; x aft, y right, z up - JSBSim's structural frame).
Sections are streamwise: the chord lies along +x before twist, the section
plane is tilted by the dihedral of the leading-edge line, and twist rotates the
section about its leading edge. A mirrored surface is defined for y >= 0 and
copied to y <= 0.

From that it builds:
- strips: spanwise elements with their chord, frame and airfoil, the unit
  every aerodynamic model works on;
- a vortex lattice (panels in the chord plane, camber through the normals);
- a skin mesh for drawing, wetted area and mass distribution.

Per strip frame: c (chord, leading to trailing edge), u (normal, the upper
surface side), e = u x c (the bound-vortex direction: a positive circulation
means lift towards u when the air flows along c).
"""
import numpy as np

from . import airfoil as af

CHANNELS = ("aileron", "elevator", "rudder", "flap")


class Control:
    """A control surface on part of a lifting surface: a trailing-edge flap
    (chord_fraction < 1) or the whole section turning about a spindle
    (chord_fraction = 1: an all-moving tail or canard; pivot = the spindle's
    chord fraction, for drawing). It follows its channel times gain, plus
    any channels in mix = {channel = gain}: a stabilator that also rolls
    (mix = { aileron = 0.3 }), a flaperon (mix = { flap = 1 }), an elevon."""

    def __init__(self, spec, surface_name):
        where = "surface %r control %r" % (surface_name, spec.get("name", "?"))
        self.name = spec.get("name") or spec.get("channel")
        self.channel = spec.get("channel", self.name)
        if self.channel not in CHANNELS:
            raise ValueError("%s: channel must be one of %s" % (where, ", ".join(CHANNELS)))
        span = spec.get("span")
        if not span or len(span) != 2:
            raise ValueError("%s: span = [eta_start, eta_end] (0 root .. 1 tip) is required" % where)
        self.eta0, self.eta1 = float(span[0]), float(span[1])
        cf = spec.get("chord_fraction", 0.25)
        self.cf0, self.cf1 = (float(cf), float(cf)) if np.isscalar(cf) else (float(cf[0]), float(cf[1]))
        lim = spec.get("limits", [-25.0, 25.0])
        self.min_deg, self.max_deg = float(lim[0]), float(lim[1])
        self.gain = float(spec.get("gain", 1.0))
        self.mix = {str(k): float(v) for k, v in spec.get("mix", {}).items()}
        for k in self.mix:
            if k not in CHANNELS:
                raise ValueError("%s: mix channel %r must be one of %s" % (where, k, ", ".join(CHANNELS)))
        self.channels = dict(self.mix)
        self.channels[self.channel] = self.gain
        self.pivot = float(spec.get("pivot", 0.35))
        self.kind = spec.get("kind", "plain")  # plain | slotted | fowler (flaps)
        if self.kind not in ("plain", "slotted", "fowler"):
            raise ValueError("%s: kind must be plain, slotted or fowler" % where)
        if not (0.0 <= self.eta0 < self.eta1 <= 1.0) or not (0.0 < self.cf0 <= 1.0 and 0.0 < self.cf1 <= 1.0):
            raise ValueError("%s: span within [0, 1] and chord_fraction within (0, 1]" % where)
        self.all_moving = self.cf0 >= 0.999 and self.cf1 >= 0.999
        if (self.cf0 >= 0.999) != (self.cf1 >= 0.999):
            raise ValueError("%s: an all-moving surface has chord_fraction 1 all along" % where)

    def chord_fraction(self, eta):
        t = (eta - self.eta0) / (self.eta1 - self.eta0)
        return (1 - t) * self.cf0 + t * self.cf1

    def covers(self, eta):
        return self.eta0 - 1e-9 <= eta <= self.eta1 + 1e-9


class LeadingEdge:
    """A leading-edge flap or slat on part of a lifting surface (leading =
    [...] on the surface): its front chord_fraction turns down about a hinge
    behind it, on the fighter's schedule - schedule = [a, m, b]: a alpha(deg)
    - m qbar/p + b degrees (qbar/p = 0.7 M^2; the F-16's 1.38, 9.05, 1.45) -
    held within limits. It moves in the 3D model (fsim:lef); the aerodynamic
    model does not see it (its tables stand for the scheduled wing)."""

    channel = "lef"
    all_moving = False
    mix = {}

    def __init__(self, spec, surface_name):
        where = "surface %r leading-edge device %r" % (surface_name, spec.get("name", "?"))
        self.name = spec.get("name", "leading-edge flap")
        span = spec.get("span")
        if not span or len(span) != 2:
            raise ValueError("%s: span = [eta_start, eta_end] (0 root .. 1 tip) is required" % where)
        self.eta0, self.eta1 = float(span[0]), float(span[1])
        cf = spec.get("chord_fraction", 0.15)
        self.cf0, self.cf1 = (float(cf), float(cf)) if np.isscalar(cf) else (float(cf[0]), float(cf[1]))
        lim = spec.get("limits", [-2.0, 25.0])
        self.min_deg, self.max_deg = float(lim[0]), float(lim[1])
        self.schedule = [float(v) for v in spec.get("schedule", [1.38, 9.05, 1.45])]
        if not (0.0 <= self.eta0 < self.eta1 <= 1.0) or not (0.0 < self.cf0 <= 1.0 and 0.0 < self.cf1 <= 1.0):
            raise ValueError("%s: span within [0, 1] and chord_fraction within (0, 1] (1: the whole section, hinged "
                             "at its trailing edge - a vortex controller)" % where)
        if len(self.schedule) != 3 or not self.min_deg < self.max_deg:
            raise ValueError("%s: schedule = [a, m, b] and limits = [lo, hi] with lo < hi" % where)
        self.channels = {"lef": 1.0}

    def chord_fraction(self, eta):
        t = (eta - self.eta0) / (self.eta1 - self.eta0)
        return (1 - t) * self.cf0 + t * self.cf1

    def covers(self, eta):
        return self.eta0 - 1e-9 <= eta <= self.eta1 + 1e-9


class Section:
    def __init__(self, le, chord, twist_deg, foil):
        self.le = np.asarray(le, float)
        self.chord = float(chord)
        self.twist = np.radians(float(twist_deg))
        self.airfoil = foil


class Surface:
    def __init__(self, spec, base_dir=None):
        self.spec = spec
        self.name = spec.get("name", "surface")
        self.kind = spec.get("kind", "wing")
        self.mirror = bool(spec.get("mirror", self.kind not in ("fin", "vtail")))
        origin = np.asarray(spec.get("origin", [0.0, 0.0, 0.0]), float)
        default_foil = spec.get("airfoil", "naca0012")
        rows = spec.get("sections")
        if not rows or len(rows) < 2:
            raise ValueError("surface %r: at least two sections are required" % self.name)
        self.sections = []
        for i, r in enumerate(rows):
            if "le" not in r or "chord" not in r:
                raise ValueError("surface %r section %d: 'le' = [x, y, z] and 'chord' are required" % (self.name, i))
            foil = af.get(r.get("airfoil", default_foil), base_dir)
            self.sections.append(Section(origin + np.asarray(r["le"], float), r["chord"], r.get("twist", 0.0), foil))
        self.controls = [Control(c, self.name) for c in spec.get("controls", [])]
        self.leading = [LeadingEdge(c, self.name) for c in spec.get("leading", [])]
        # spanwise coordinate: arc length of the leading edge in the y-z plane
        yz = np.array([[s.le[1], s.le[2]] for s in self.sections])
        seg = np.hypot(*np.diff(yz, axis=0).T)
        if np.any(seg <= 1e-9):
            raise ValueError("surface %r: consecutive sections must differ in y or z" % self.name)
        self._arc = np.concatenate([[0.0], np.cumsum(seg)])
        self.eta_sections = self._arc / self._arc[-1]
        self.nominal_nc = int(spec.get("chordwise_panels", 0)) or None
        if "root_reflection" in spec:
            self.root_reflection = float(spec["root_reflection"])
        self.nominal_ns = int(spec.get("spanwise_panels", 0)) or None

    # -- interpolation along the span -----------------------------------------------------------
    def _locate(self, eta):
        i = int(np.clip(np.searchsorted(self.eta_sections, eta, side="right") - 1, 0, len(self.sections) - 2))
        e0, e1 = self.eta_sections[i], self.eta_sections[i + 1]
        return i, (eta - e0) / (e1 - e0)

    def station(self, eta):
        """Leading edge, chord, twist (rad) and airfoil at a spanwise fraction."""
        i, w = self._locate(eta)
        a, b = self.sections[i], self.sections[i + 1]
        le = (1 - w) * a.le + w * b.le
        return le, (1 - w) * a.chord + w * b.chord, (1 - w) * a.twist + w * b.twist, a.airfoil.blend(b.airfoil, w)

    def sweep_at(self, eta):
        """Leading-edge and quarter-chord sweep (rad) of the section chain at
        eta, measured in the surface's plane."""
        i, _ = self._locate(eta)
        a, b = self.sections[i], self.sections[i + 1]
        d = b.le - a.le
        run = float(np.hypot(d[1], d[2]))
        return (float(np.arctan2(d[0], run)), float(np.arctan2(d[0] + 0.25 * (b.chord - a.chord), run)))

    @property
    def sweep_le_deg_max(self):
        return max(float(np.degrees(self.sweep_at(e)[0])) for e in 0.5 * (self.eta_sections[1:] + self.eta_sections[:-1]))

    def span_direction(self, eta):
        """Unit spanwise direction (root to tip) of the leading-edge line in
        the y-z plane at eta: the axis the section plane is tilted about."""
        i, _ = self._locate(eta)
        d = self.sections[i + 1].le - self.sections[i].le
        d = np.array([0.0, d[1], d[2]])
        return d / np.linalg.norm(d)

    def frame(self, eta):
        """Chord direction c and upper-surface normal u (right-hand side),
        twist included (a positive twist raises the leading edge)."""
        s = self.span_direction(eta)
        u0 = np.cross([1.0, 0.0, 0.0], s)
        _, _, tw, _ = self.station(eta)
        c = np.cos(tw) * np.array([1.0, 0.0, 0.0]) - np.sin(tw) * u0
        u = np.sin(tw) * np.array([1.0, 0.0, 0.0]) + np.cos(tw) * u0
        return c, u

    def breakpoints(self):
        pts = set(np.round(self.eta_sections, 9))
        for c in self.controls:
            pts.update([round(c.eta0, 9), round(c.eta1, 9)])
        return np.array(sorted(pts))

    def span_stations(self, n):
        """Strip boundaries: n strips, denser towards the tip, with a
        boundary at every section and control-surface edge."""
        base = np.sin(0.5 * np.pi * np.linspace(0.0, 1.0, n + 1))
        if self.kind in ("fin", "vtail") or not self.mirror:
            base = 0.5 * (1 - np.cos(np.pi * np.linspace(0.0, 1.0, n + 1)))  # both ends
        brk = self.breakpoints()
        pts = list(brk)
        for x in base:
            gap = np.min(np.abs(brk - x))
            if gap > 0.35 / n:
                pts.append(x)
        return np.unique(np.round(pts, 9))

    # -- summary geometry -----------------------------------------------------------------------
    def _integrate(self, f, n=400):
        eta = np.linspace(0.0, 1.0, n + 1)
        mid = 0.5 * (eta[1:] + eta[:-1])
        ds = np.diff(eta) * self._arc[-1]
        return np.sum([f(e) * d for e, d in zip(mid, ds)], axis=0)

    @property
    def half_arc(self):
        return float(self._arc[-1])

    @property
    def area(self):
        """Reference (planform) area: chord integrated along the span arc, both
        halves of a mirrored surface."""
        a = self._integrate(lambda e: self.station(e)[1])
        return float(a * (2 if self.mirror else 1))

    @property
    def span(self):
        """Tip-to-tip span of a mirrored surface (along y); the height of a
        single one (its arc length)."""
        return float(2 * self.sections[-1].le[1]) if self.mirror else self.half_arc

    @property
    def aspect_ratio(self):
        b = 2 * self.half_arc if self.mirror else self.half_arc
        return b * b / self.area

    @property
    def mac(self):
        """Mean aerodynamic chord and its leading-edge point."""
        half = self.area / (2 if self.mirror else 1)
        c2 = self._integrate(lambda e: self.station(e)[1] ** 2)
        le = self._integrate(lambda e: self.station(e)[0] * self.station(e)[1])
        return float(c2 / half), le / half  # for a mirrored surface, the station on the y > 0 half

    @property
    def taper(self):
        return self.sections[-1].chord / self.sections[0].chord

    def sweep_deg(self, fraction=0.25):
        """Sweep of the given chord-fraction line, root to tip."""
        a, b = self.sections[0], self.sections[-1]
        pa = a.le + fraction * a.chord * np.array([1, 0, 0])
        pb = b.le + fraction * b.chord * np.array([1, 0, 0])
        d = pb - pa
        return float(np.degrees(np.arctan2(d[0], np.hypot(d[1], d[2]))))

    @property
    def dihedral_deg(self):
        d = self.sections[-1].le - self.sections[0].le
        return float(np.degrees(np.arctan2(d[2], abs(d[1])))) if self.mirror else 90.0

    @property
    def thickness_ratio(self):
        return float(np.mean([s.airfoil.thickness_ratio for s in self.sections]))

    # -- strips and lattice ---------------------------------------------------------------------
    def strips(self, n_span=None, n_chord=None):
        """Spanwise strips of the defined half (and its mirror), each with a
        row of chordwise VLM panels. Returns a dict of arrays; see Lattice."""
        ns = n_span or self.nominal_ns or (24 if self.kind == "wing" else 12)
        nc = n_chord or self.nominal_nc or (8 if self.kind == "wing" else 6)
        eta = self.span_stations(ns)
        rows = []
        for e0, e1 in zip(eta[:-1], eta[1:]):
            em = 0.5 * (e0 + e1)
            ctrl = next((c for c in self.controls if c.covers(e0) and c.covers(e1)), None)
            rows.append(self._strip(e0, e1, em, nc, ctrl))
        out = {k: np.array([r[k] for r in rows]) for k in rows[0] if k not in ("airfoil", "control")}
        out["airfoil"] = [r["airfoil"] for r in rows]
        out["control"] = [r["control"] for r in rows]
        out["eta"] = np.array([r["eta"] for r in rows])
        out["side"] = np.ones(len(rows))
        if self.mirror:
            m = {k: v.copy() for k, v in out.items() if isinstance(v, np.ndarray)}
            flip = np.array([1.0, -1.0, 1.0])
            for k in ("le_in", "le_out", "c4", "ref"):
                m[k] = out[k] * flip
            for k in ("c", "u"):
                m[k] = out[k] * flip
            # panel corner points and control points
            for k in ("p_a", "p_b", "cp", "in_pts", "out_pts"):
                m[k] = out[k] * flip
            # keep the bound vortex along e = u x c: on the mirror it runs tip to root
            m["p_a"], m["p_b"] = out["p_b"] * flip, out["p_a"] * flip
            m["t_a"], m["t_b"] = out["t_b"] * flip, out["t_a"] * flip
            m["n"] = out["n"] * flip
            m["side"] = -np.ones(len(rows))
            m["airfoil"] = list(out["airfoil"])
            m["control"] = list(out["control"])
            out = {k: (np.concatenate([out[k], m[k]]) if isinstance(out[k], np.ndarray) else out[k] + m[k]) for k in out}
        out["e"] = np.cross(out["u"], out["c"])
        out["surface"] = self.name
        return out

    def _strip(self, e0, e1, em, nc, ctrl):
        le0, ch0, tw0, _ = self.station(e0)
        le1, ch1, tw1, _ = self.station(e1)
        lem, chm, twm, foil = self.station(em)
        c, u = self.frame(em)
        # chordwise stations, cosine spaced, with one moved onto the hinge
        xs = 0.5 * (1 - np.cos(np.pi * np.linspace(0.0, 1.0, nc + 1)))
        cf = None
        if ctrl is not None:
            cf = ctrl.chord_fraction(em)
            k = int(np.argmin(np.abs(xs[1:-1] - (1 - cf)))) + 1
            xs[k] = 1 - cf
        c_in, u_in = self.frame(e0)
        c_out, u_out = self.frame(e1)
        # panel corners along the chord at the inboard and outboard edges (chord plane)
        in_pts = le0 + np.outer(xs * ch0, c_in)
        out_pts = le1 + np.outer(xs * ch1, c_out)
        # horseshoe: bound vortex at each panel's quarter chord, control point at three quarters
        f_b = xs[:-1] + 0.25 * np.diff(xs)
        f_c = xs[:-1] + 0.75 * np.diff(xs)
        p_a = le0 + np.outer(f_b * ch0, c_in)
        p_b = le1 + np.outer(f_b * ch1, c_out)
        cp = 0.5 * ((le0 + np.outer(f_c * ch0, c_in)) + (le1 + np.outer(f_c * ch1, c_out)))
        slope = foil.camber_slope(f_c)
        n = u[None, :] - slope[:, None] * c[None, :]
        n /= np.linalg.norm(n, axis=1)[:, None]
        width = np.linalg.norm((le1 - le0)[1:])
        return {
            "eta": em,
            "le_in": le0, "le_out": le1,
            "chord": chm, "width": width, "area": chm * width,
            "c4": lem + 0.25 * chm * c, "ref": lem + 0.25 * chm * c,
            "c": c, "u": u,
            "twist": twm,
            "p_a": p_a, "p_b": p_b, "cp": cp, "n": n,
            # the trailing edge behind each end of the bound vortices (where the wake starts)
            "t_a": np.repeat(in_pts[-1][None, :], len(p_a), axis=0),
            "t_b": np.repeat(out_pts[-1][None, :], len(p_b), axis=0),
            "xc": f_c,                 # chordwise position of each control point (fraction)
            "hinge": (1 - cf) if cf is not None else 2.0,
            "airfoil": foil,
            "control": ctrl,
            "in_pts": in_pts, "out_pts": out_pts,
        }

    # -- skin mesh ------------------------------------------------------------------------------
    def skin(self, n_span=40, n_chord=24):
        """Closed triangle mesh of the surface (both halves if mirrored).
        Returns vertices (N, 3), triangles (M, 3) and a per-triangle tag:
        0 fixed surface, k > 0 the k-th control (1-based), -1 an end cap."""
        eta = np.unique(np.concatenate([np.linspace(0, 1, n_span + 1), self.breakpoints()]))
        xs = af.cosine_spacing(n_chord + 1)
        rings = []
        for e in eta:
            le, ch, tw, foil = self.station(e)
            c, u = self.frame(e)
            up = foil.upper(xs)
            lo = foil.lower(xs)
            # contour: TE -> upper -> LE -> lower -> TE (without repeating the LE)
            xx = np.concatenate([xs[::-1], xs[1:]])
            yy = np.concatenate([up[::-1], lo[1:]])
            rings.append(le + np.outer(xx * ch, c) + np.outer(yy * ch, u))
        rings = np.array(rings)  # (n_eta, m, 3)
        verts, tris, tags = _loft(rings, eta, xs, self.controls)
        if self.mirror:
            mv = verts * np.array([1.0, -1.0, 1.0])
            mt = tris[:, ::-1] + len(verts)
            verts = np.vstack([verts, mv])
            tris = np.vstack([tris, mt])
            tags = np.concatenate([tags, tags])
        return verts, tris, tags


def _loft(rings, eta, xs, controls):
    n_eta, m, _ = rings.shape
    verts = rings.reshape(-1, 3)
    tris, tags = [], []
    xx = np.concatenate([xs[::-1], xs[1:]])  # chord fraction of every contour point
    for i in range(n_eta - 1):
        emid = 0.5 * (eta[i] + eta[i + 1])
        ctrl_idx = next((k + 1 for k, c in enumerate(controls) if c.covers(eta[i]) and c.covers(eta[i + 1])), 0)
        hinge = 1 - controls[ctrl_idx - 1].chord_fraction(emid) if ctrl_idx else 2.0
        for j in range(m - 1):
            a, b = i * m + j, i * m + j + 1
            c, d = (i + 1) * m + j, (i + 1) * m + j + 1
            tag = ctrl_idx if 0.5 * (xx[j] + xx[j + 1]) > hinge else 0
            tris += [(a, c, b), (b, c, d)]
            tags += [tag, tag]
    # close the trailing edge gap and cap both ends (fans around the ring centroid)
    base = len(verts)
    extra = []
    for k, i in enumerate((0, n_eta - 1)):
        ring = rings[i]
        centre = ring.mean(axis=0)
        extra.append(centre)
        ci = base + k
        for j in range(m - 1):
            a, b = i * m + j, i * m + j + 1
            tris.append((ci, a, b) if k == 0 else (ci, b, a))
            tags.append(-1)  # caps: drawn, but not wetted area
    verts = np.vstack([verts, extra])
    return verts, np.array(tris, int), np.array(tags, int)
