"""Bodies: fuselages, nacelles, pods, floats, booms.

A body is a chain of cross-sections along x. Each station gives the width,
the top and bottom z of the section and a superellipse exponent (2 = ellipse,
larger = boxier, below 2 = diamond-like); stations are joined with a smooth
monotone interpolation. A mirrored body (nacelles, booms) is defined on the
y > 0 side and copied.

    stations = [ { x = 0.0, w = 0.0, top = 1.0, bottom = 1.0 },
                 { x = 0.4, w = 0.9, top = 1.35, bottom = 0.65, n = 2.5 }, ... ]
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
        if np.any(np.diff(self.x) <= 0):
            raise ValueError("body %r: stations must be in increasing x" % self.name)
        if np.any(self.top < self.bottom - 1e-9) or np.any(self.w < 0):
            raise ValueError("body %r: top must be >= bottom and w >= 0 at every station" % self.name)

    # -- sections along x -----------------------------------------------------------------------
    def section(self, xq):
        """Width, top, bottom, lateral centre and exponent at x (arrays ok)."""
        xq = np.clip(xq, self.x[0], self.x[-1])
        w = np.maximum(_pchip(self.x, self.w, xq), 0.0)
        top = _pchip(self.x, self.top, xq)
        bot = _pchip(self.x, self.bottom, xq)
        top = np.maximum(top, bot)
        return w, top, bot, np.interp(xq, self.x, self.y), np.interp(xq, self.x, self.n)

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
        has area 4ab G(1+1/n)^2 / G(1+2/n)."""
        w, t, b, _, n = self.section(xq)
        from math import gamma
        k = np.array([4 * gamma(1 + 1 / ni) ** 2 / gamma(1 + 2 / ni) for ni in np.atleast_1d(n)])
        return k.reshape(np.shape(w)) * (w / 2) * ((t - b) / 2)

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
        w, t, b, yc, n = self.section(xs)
        th = np.linspace(0.0, 2 * np.pi, n_around, endpoint=False)
        c, s = np.cos(th), np.sin(th)
        rings = []
        for i in range(len(xs)):
            e = 2.0 / n[i]
            yy = yc[i] + (w[i] / 2) * np.sign(c) * np.abs(c) ** e
            zz = 0.5 * (t[i] + b[i]) + ((t[i] - b[i]) / 2) * np.sign(s) * np.abs(s) ** e
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
