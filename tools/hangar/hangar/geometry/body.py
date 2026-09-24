"""Bodies: fuselages, nacelles, pods, floats, booms.

A body is a chain of cross-sections along x. Each station gives the width,
the top and bottom z of the section and a superellipse exponent (2 = ellipse,
larger = boxier, below 2 = diamond-like); stations are joined with a smooth
monotone interpolation. A mirrored body (nacelles, booms) is defined on the
y > 0 side and copied.

    stations = [ { x = 0.0, w = 0.0, top = 1.0, bottom = 1.0 },
                 { x = 0.4, w = 0.9, top = 1.35, bottom = 0.65, n = 2.5 }, ... ]

A section's two halves can differ: chine is the height of its widest line
(default half way), and n_top, n_bottom the exponents above and below it -
a flat belly under a round back (n_bottom = 5, n_top = 2), or a chined nose
(n = 1.6, the chine a sharp edge along the side).

A canopy (a pod named "canopy...") is glass; frames = [x, ...] puts a frame
round it at each x (frame_width wide, default 6 cm): a windscreen's arch, the
bow between a canopy's panes.
"""
import numpy as np


def _pchip(x, y, xq):
    """Monotone cubic (Fritsch-Carlson) interpolation: smooth, no overshoot,
    so a body never bulges past its stations."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    h = np.diff(x)
    d = np.diff(y) / h
    m = np.zeros_like(y)
    if len(x) > 2:
        w1, w2 = 2 * h[1:] + h[:-1], h[1:] + 2 * h[:-1]
        same = d[:-1] * d[1:] > 0
        m[1:-1] = np.where(same, (w1 + w2) / (w1 / np.where(same, d[:-1], 1) + w2 / np.where(same, d[1:], 1)), 0.0)
    m[0], m[-1] = d[0], d[-1]
    i = np.clip(np.searchsorted(x, xq, side="right") - 1, 0, len(x) - 2)
    t = (xq - x[i]) / h[i]
    h00, h10, h01, h11 = 2 * t**3 - 3 * t**2 + 1, t**3 - 2 * t**2 + t, -2 * t**3 + 3 * t**2, t**3 - t**2
    return h00 * y[i] + h10 * h[i] * m[i] + h01 * y[i + 1] + h11 * h[i] * m[i + 1]


class Body:
    def __init__(self, spec):
        self.name = spec.get("name", "body")
        self.kind = spec.get("kind", "fuselage")  # fuselage | nacelle | pod | boom | float
        self.mirror = bool(spec.get("mirror", False))
        origin = np.asarray(spec.get("origin", [0.0, 0.0, 0.0]), float)
        rows = spec.get("stations")
        if not rows or len(rows) < 2:
            raise ValueError("body %r: at least two stations are required" % self.name)
        try:
            self.x = np.array([r["x"] for r in rows], float) + origin[0]
            self.w = np.array([r["w"] for r in rows], float)
            self.top = np.array([r["top"] for r in rows], float) + origin[2]
            self.bottom = np.array([r["bottom"] for r in rows], float) + origin[2]
        except KeyError as e:
            raise ValueError("body %r: every station needs x, w, top and bottom (missing %s)" % (self.name, e))
        self.y = np.array([r.get("y", 0.0) for r in rows], float) + origin[1]
        self.n = np.array([r.get("n", 2.0) for r in rows], float)
        self.frames = [float(x) + origin[0] for x in spec.get("frames", [])]
        # aero = false: drawn and weighed, left out of the aerodynamic tables
        # (a launcher rail on a wing tip is part of the tip, not a slender body)
        self.aero = bool(spec.get("aero", True))
        self.frame_width = float(spec.get("frame_width", 0.06))
        self.n_top = np.array([r.get("n_top", r.get("n", 2.0)) for r in rows], float)
        self.n_bottom = np.array([r.get("n_bottom", r.get("n", 2.0)) for r in rows], float)
        # the widest line, as a fraction of the height from the bottom
        span = np.maximum(self.top - self.bottom, 1e-9)
        self.chine = np.array([np.clip((r["chine"] + origin[2] - b) / h, 0.0, 1.0) if "chine" in r else 0.5
                               for r, b, h in zip(rows, self.bottom, span)], float)
        if np.any(self.n_top < 1.0) or np.any(self.n_bottom < 1.0) or np.any(self.n < 1.0):
            raise ValueError("body %r: the section exponents must be 1 or more" % self.name)
        if np.any(np.diff(self.x) <= 0):
            raise ValueError("body %r: stations must be in increasing x" % self.name)
        if np.any(self.top < self.bottom - 1e-9) or np.any(self.w < 0):
            raise ValueError("body %r: top must be >= bottom and w >= 0 at every station" % self.name)

    # -- sections along x -----------------------------------------------------------------------
    def section(self, xq):
        """Width, top, bottom, lateral centre and (mean) exponent at x (arrays
        ok)."""
        xq = np.clip(xq, self.x[0], self.x[-1])
        w = np.maximum(_pchip(self.x, self.w, xq), 0.0)
        top = _pchip(self.x, self.top, xq)
        bot = _pchip(self.x, self.bottom, xq)
        top = np.maximum(top, bot)
        n = 0.5 * (np.interp(xq, self.x, self.n_top) + np.interp(xq, self.x, self.n_bottom))
        return w, top, bot, np.interp(xq, self.x, self.y), n

    def halves(self, xq):
        """The height of the widest line and the exponents above and below it
        at x (arrays ok)."""
        xq = np.clip(xq, self.x[0], self.x[-1])
        _, top, bot, _, _ = self.section(xq)
        zc = bot + np.interp(xq, self.x, self.chine) * (top - bot)
        return zc, np.interp(xq, self.x, self.n_top), np.interp(xq, self.x, self.n_bottom)

    @property
    def length(self):
        return float(self.x[-1] - self.x[0])

    def _grid(self, n=200):
        xs = np.linspace(self.x[0], self.x[-1], n + 1)
        return xs, self.section(xs)

    @property
    def max_width(self):
        return float(self._grid()[1][0].max())

    @property
    def max_height(self):
        _, (w, t, b, _, _) = self._grid()
        return float((t - b).max())

    def area_at(self, xq):
        """Cross-section area: a superellipse of semi-axes a, b and exponent n
        has area 4ab G(1+1/n)^2 / G(1+2/n); each half its half."""
        w, t, b, _, _ = self.section(xq)
        zc, nt, nb = self.halves(xq)
        from math import gamma

        def k(n):
            return np.array([2 * gamma(1 + 1 / ni) ** 2 / gamma(1 + 2 / ni) for ni in np.atleast_1d(n)]).reshape(np.shape(w))
        return (w / 2) * ((t - zc) * k(nt) + (zc - b) * k(nb))

    @property
    def volume(self):
        xs = np.linspace(self.x[0], self.x[-1], 401)
        return float(np.trapezoid(self.area_at(xs), xs)) * (2 if self.mirror else 1)

    @property
    def max_area(self):
        xs = np.linspace(self.x[0], self.x[-1], 401)
        return float(self.area_at(xs).max())

    @property
    def fineness(self):
        d = np.sqrt(4 * self.max_area / np.pi)
        return self.length / max(d, 1e-9)

    @property
    def planform_area(self):
        """Top-view (for vertical cross-flow) and side-view (lateral) areas
        of one body."""
        xs, (w, t, b, _, _) = self._grid(400)
        return float(np.trapezoid(w, xs)), float(np.trapezoid(t - b, xs))

    # -- mesh -----------------------------------------------------------------------------------
    def skin(self, n_x=64, n_around=32):
        """Closed triangle mesh (both copies if mirrored): vertices, triangles,
        tags (0 = skin)."""
        # stations: cluster at the ends where the shape turns fastest
        xs = self.x[0] + self.length * 0.5 * (1 - np.cos(np.pi * np.linspace(0, 1, n_x + 1)))
        xs = np.unique(np.concatenate([xs, self.x]))
        w, t, b, yc, _ = self.section(xs)
        zc, nt, nb = self.halves(xs)
        th = np.linspace(0.0, 2 * np.pi, n_around, endpoint=False)
        c, s = np.cos(th), np.sin(th)
        rings = []
        for i in range(len(xs)):
            e = np.where(s >= 0.0, 2.0 / nt[i], 2.0 / nb[i])
            yy = yc[i] + (w[i] / 2) * np.sign(c) * np.abs(c) ** e
            zz = zc[i] + np.where(s >= 0.0, t[i] - zc[i], zc[i] - b[i]) * np.sign(s) * np.abs(s) ** e
            rings.append(np.column_stack([np.full(n_around, xs[i]), yy, zz]))
        rings = np.array(rings)
        verts = rings.reshape(-1, 3)
        m = n_around
        tris = []
        for i in range(len(xs) - 1):
            for j in range(m):
                a, bb = i * m + j, i * m + (j + 1) % m
                cc, d = (i + 1) * m + j, (i + 1) * m + (j + 1) % m
                tris += [(a, bb, cc), (bb, d, cc)]
        # caps at blunt ends (zero-area ends are already closed)
        extra = []
        for k, i in enumerate((0, len(xs) - 1)):
            if w[i] > 1e-6 and t[i] - b[i] > 1e-6:
                centre = rings[i].mean(axis=0)
                ci = len(verts) + len(extra)
                extra.append(centre)
                for j in range(m):
                    a, bb = i * m + j, i * m + (j + 1) % m
                    tris.append((ci, bb, a) if k == 0 else (ci, a, bb))
        if extra:
            verts = np.vstack([verts, extra])
        tris = np.array(tris, int)
        tags = np.zeros(len(tris), int)
        if self.mirror:
            mv = verts * np.array([1.0, -1.0, 1.0])
            verts, tris = np.vstack([verts, mv]), np.vstack([tris, tris[:, ::-1] + len(verts)])
            tags = np.concatenate([tags, tags])
        return verts, tris, tags

    def copies(self):
        """Lateral offsets of the body's copies (one, or +/- for mirrored)."""
        return [1.0, -1.0] if self.mirror else [1.0]


class Intake(Body):
    """An air intake ([[intake]]): its cowl as a body whose first station is
    the lip, open there onto a duct that runs dark into the airframe.

        lip = 0.03       the lip's thickness (m)
        rake = 0         deg: the lip plane leaned so its top is ahead of its bottom
        sweep = 0        deg: leaned so its outboard edge is behind its inboard one
        duct = 1.5       m: how far the duct is seen into, behind the lip
        duct_rise = 0    m: how far its centre climbs over that (towards the engine)
        duct_taper = 1   its size at the end over its size at the lip
    """

    def __init__(self, spec):
        spec = dict(spec)
        spec.setdefault("kind", "intake")
        super().__init__(spec)
        if self.w[0] <= 0.0 or self.top[0] <= self.bottom[0]:
            raise ValueError("intake %r: its first station is the lip, and must be open (w > 0, top > bottom)" % self.name)
        self.lip = float(spec.get("lip", 0.03))
        self.rake = float(spec.get("rake", 0.0))
        self.sweep = float(spec.get("sweep", 0.0))
        self.duct = float(spec.get("duct", 1.5))
        self.duct_rise = float(spec.get("duct_rise", 0.0))
        self.duct_taper = float(spec.get("duct_taper", 1.0))
        if not 0.0 < self.lip < 0.25 * min(self.w[0], self.top[0] - self.bottom[0]):
            raise ValueError("intake %r: lip must be positive and under a quarter of the opening" % self.name)

    def lip_plane(self):
        """A point on the lip plane (the opening's centre) and its unit normal,
        forward out of the cowl."""
        zc, _, _ = self.halves(self.x[0])
        p = np.array([self.x[0], self.y[0], float(zc)])
        n = np.array([-1.0, np.tan(np.radians(self.sweep)), -np.tan(np.radians(self.rake))])
        return p, n / np.linalg.norm(n)
