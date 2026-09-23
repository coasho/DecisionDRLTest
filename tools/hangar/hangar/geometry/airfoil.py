"""Airfoil sections: coordinates, mean camber line and thickness.

NACA 4-digit and 5-digit sections are generated from their published
equations (Abbott & von Doenhoff, "Theory of Wing Sections", 1959); any other
section is read from a coordinate file (Selig or Lednicer format). Everything
is normalised to unit chord, x from the leading edge (0) to the trailing
edge (1), y up.
"""
import os
import re

import numpy as np

# NACA 5-digit standard (non-reflexed) mean lines for a design cl of 0.3,
# keyed by the position digit: (m, k1). Abbott & von Doenhoff, table p. 115.
_NACA5_MEANLINES = {1: (0.0580, 361.4), 2: (0.1260, 51.64), 3: (0.2025, 15.957), 4: (0.2900, 6.643), 5: (0.3910, 3.230)}


def cosine_spacing(n):
    """n points from 0 to 1, clustered at both ends."""
    return 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, n)))


def naca_thickness(x, t, closed_te=True):
    """Half-thickness of the NACA 4-digit family at x (unit chord)."""
    a4 = -0.1036 if closed_te else -0.1015
    x = np.asarray(x, float)
    return 5.0 * t * (0.2969 * np.sqrt(np.maximum(x, 0.0)) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 + a4 * x**4)


class Airfoil:
    """A section at unit chord. Built from a camber line and a thickness
    distribution (the NACA families) or from surface coordinates, from which
    both are recovered."""

    def __init__(self, name, x, camber, thickness):
        self.name = name
        self._x = np.asarray(x, float)          # stations, 0..1
        self._yc = np.asarray(camber, float)    # mean line
        self._yt = np.asarray(thickness, float) # half thickness
        self._slope = np.gradient(self._yc, self._x)

    # -- the section as functions of x --------------------------------------------------------
    def camber(self, x):
        return np.interp(x, self._x, self._yc)

    def camber_slope(self, x):
        return np.interp(x, self._x, self._slope)

    def half_thickness(self, x):
        return np.interp(x, self._x, self._yt)

    def upper(self, x):
        """Upper surface y at x (thickness applied normal to the chord: close
        enough to the true NACA construction for drawing and areas)."""
        return self.camber(x) + self.half_thickness(x)

    def lower(self, x):
        return self.camber(x) - self.half_thickness(x)

    # -- summary numbers ----------------------------------------------------------------------
    @property
    def thickness_ratio(self):
        return 2.0 * float(self._yt.max())

    @property
    def x_max_thickness(self):
        return float(self._x[np.argmax(self._yt)])

    @property
    def max_camber(self):
        i = np.argmax(np.abs(self._yc))
        return float(self._yc[i])

    @property
    def x_max_camber(self):
        return float(self._x[np.argmax(np.abs(self._yc))])

    @property
    def area(self):
        """Cross-section area at unit chord."""
        return float(np.trapezoid(2.0 * self._yt, self._x))

    @property
    def perimeter(self):
        xs = np.concatenate([self._x[::-1], self._x[1:]])
        ys = np.concatenate([self.upper(self._x)[::-1], self.lower(self._x)[1:]])
        return float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))

    @property
    def leading_edge_sharpness(self):
        """DATCOM's leading-edge sharpness parameter: the upper-surface
        ordinate difference between 6 % and 0.15 % chord, in percent chord."""
        return 100.0 * float(self.upper(0.06) - self.upper(0.0015))

    @property
    def trailing_edge_angle_deg(self):
        """Included angle at the trailing edge (from the last 5 % of chord)."""
        dt = self.half_thickness(0.95) - self.half_thickness(1.0)
        return float(np.degrees(2.0 * np.arctan2(dt, 0.05)))

    def coordinates(self, n=81):
        """Closed contour, Selig order: trailing edge, upper surface to the
        leading edge, lower surface back. Returns (2n-1, 2)."""
        x = cosine_spacing(n)
        xu, yu = x[::-1], self.upper(x[::-1])
        xl, yl = x[1:], self.lower(x[1:])
        return np.column_stack([np.concatenate([xu, xl]), np.concatenate([yu, yl])])

    def blend(self, other, w, name=None):
        """Linear blend (w = 0: self, 1: other), for sections between two
        defined ones."""
        if w <= 0.0 or other is self:
            return self
        if w >= 1.0:
            return other
        x = self._x
        return Airfoil(name or "%s~%s" % (self.name, other.name), x,
                       (1 - w) * self._yc + w * other.camber(x), (1 - w) * self._yt + w * other.half_thickness(x))

    def __repr__(self):
        return "Airfoil(%s, t/c %.3f, camber %.3f at %.2f)" % (self.name, self.thickness_ratio, self.max_camber,
                                                               self.x_max_camber)


def naca4(code, n=201, closed_te=True):
    """NACA MPTT: M max camber (% chord), P its position (tenths), TT thickness (%)."""
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    x = cosine_spacing(n)
    yc = np.zeros_like(x)
    if m > 0.0 and p > 0.0:
        front = x < p
        yc[front] = m / p**2 * (2 * p * x[front] - x[front] ** 2)
        yc[~front] = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x[~front] - x[~front] ** 2)
    return Airfoil("naca" + code, x, yc, naca_thickness(x, t, closed_te))


def naca5(code, n=201, closed_te=True):
    """NACA LPQTT with a standard mean line: L design lift (x0.15), P max
    camber position (x0.05), Q = 0, TT thickness (%)."""
    design_cl = 0.15 * int(code[0])
    pos = int(code[1])
    if code[2] != "0" or pos not in _NACA5_MEANLINES:
        raise ValueError("naca%s: only standard (non-reflexed) 5-digit mean lines P=1..5 are built in" % code)
    m, k1 = _NACA5_MEANLINES[pos]
    k1 *= design_cl / 0.3
    t = int(code[3:]) / 100.0
    x = cosine_spacing(n)
    yc = np.where(x < m, k1 / 6.0 * (x**3 - 3 * m * x**2 + m**2 * (3 - m) * x), k1 * m**3 / 6.0 * (1 - x))
    return Airfoil("naca" + code, x, yc, naca_thickness(x, t, closed_te))


def flat_plate(t=0.02, n=101):
    x = cosine_spacing(n)
    return Airfoil("plate", x, np.zeros_like(x), naca_thickness(x, t))


def from_coordinates(name, xy, n=201):
    """Recover camber and thickness from surface coordinates (a closed
    contour in any common order)."""
    xy = np.asarray(xy, float)
    le = int(np.argmin(xy[:, 0]))
    a, b = xy[: le + 1], xy[le:]
    # one side runs TE->LE, the other LE->TE; orient both LE->TE
    a = a[::-1]
    if np.mean(a[:, 1]) < np.mean(b[:, 1]):
        a, b = b, a
    chord = max(a[:, 0].max(), b[:, 0].max()) - xy[le, 0]
    x0, y0 = xy[le]
    a = (a - [x0, y0]) / chord
    b = (b - [x0, y0]) / chord
    x = cosine_spacing(n)
    ua = np.interp(x, *_increasing(a))
    lb = np.interp(x, *_increasing(b))
    return Airfoil(name, x, 0.5 * (ua + lb), np.maximum(0.5 * (ua - lb), 0.0))


def _increasing(p):
    order = np.argsort(p[:, 0], kind="stable")
    xs, ys = p[order, 0], p[order, 1]
    keep = np.concatenate([[True], np.diff(xs) > 1e-9])
    return xs[keep], ys[keep]


def read_dat(path):
    """Selig (one contour, TE-upper-LE-lower-TE) or Lednicer (counts line,
    then upper and lower from the LE) coordinate files."""
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = [l.strip() for l in f if l.strip()]
    name = lines[0]
    rows = []
    for l in lines[1:]:
        parts = l.replace(",", " ").split()
        try:
            rows.append([float(parts[0]), float(parts[1])])
        except (ValueError, IndexError):
            continue
    xy = np.array(rows)
    if len(xy) and xy[0, 0] > 1.5:  # Lednicer: first row holds the point counts
        nu, nl = int(xy[0, 0]), int(xy[0, 1])
        up, lo = xy[1 : 1 + nu], xy[1 + nu : 1 + nu + nl]
        xy = np.vstack([up[::-1], lo[1:]])
    return from_coordinates(os.path.splitext(os.path.basename(path))[0] or name, xy)


_cache = {}


def get(spec, base_dir=None):
    """An airfoil from a spec string: "naca2412", "naca23012", "plate", or a
    coordinate file path (relative to base_dir)."""
    key = (spec, base_dir)
    if key in _cache:
        return _cache[key]
    s = spec.strip().lower()
    m = re.fullmatch(r"naca\s*(\d{4,5})", s)
    if m:
        digits = m.group(1)
        foil = naca4(digits) if len(digits) == 4 else naca5(digits)
    elif s == "plate":
        foil = flat_plate()
    else:
        path = spec if os.path.isabs(spec) or base_dir is None else os.path.join(base_dir, spec)
        if not os.path.isfile(path):
            raise ValueError("airfoil %r: not a NACA 4/5-digit code and no such file %s" % (spec, path))
        foil = read_dat(path)
    _cache[key] = foil
    return foil
