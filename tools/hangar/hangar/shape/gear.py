"""Landing gear for the 3D model.

Each leg is a strut with its oleo and wheel(s), built with the wheel on the
ground and the strut up to its attachment, where it hinges: as the gear
retracts the viewer swings it up about its trunnion and, at once, twists it
about the strut (two nested fsim:gear nodes), as real legs fold. A
retractable leg folds into a bay cut into the airframe, and doors - the skin
cut out over the bay - open before it comes down and close after it has gone
up.

Per gear in the design ([[gear]]):
    retract = "aft"         the way the leg swings up: forward, aft, inward or outward
                            (default: a steerable nose gear aft, main gear forward)
    retract_deg = 90        how far, and
    wheel_turn = 0          degrees the wheel twists about the strut on the way up
                            ("flat": lying flat); without both, hangar fits them - and
                            cants the hinge up to 40 deg, as real trunnions are - to
                            stow the leg inside the airframe, as near a plain 90 deg
                            swing as it can
    wheels = 1              side by side on one axle
    door_deg = 90           how far the doors open
    fairing = true          (fixed gear) a spat over the wheel
"""
import numpy as np

from .airframe import GAP, METAL, SKIN, STRUT, TYRE, union

DOORS = 0.2  # the doors move while the gear position runs 0..0.2, the legs 0.2..1


def _rotation(axis, deg):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    th = np.radians(deg)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def _axis_angle(R):
    """The unit axis and angle (degrees) of the rotation R."""
    th = float(np.arccos(np.clip(0.5 * (np.trace(R) - 1.0), -1.0, 1.0)))
    if th < 1e-9:
        return np.array([1.0, 0.0, 0.0]), 0.0
    if np.pi - th < 1e-6:
        w, v = np.linalg.eigh(0.5 * (R + R.T))
        return v[:, int(np.argmax(w))], float(np.degrees(th))
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(th))
    return k / np.linalg.norm(k), float(np.degrees(th))


def _swing(direction, side):
    """The axis about which a positive turn swings a hanging leg that way
    (design frame: x aft, y right, z up)."""
    if direction == "forward":
        return np.array([0.0, 1.0, 0.0])
    if direction == "aft":
        return np.array([0.0, -1.0, 0.0])
    s = -1.0 if side > 0 else 1.0   # inward: the right leg about -x
    return np.array([s if direction == "inward" else -s, 0.0, 0.0])


class Leg:
    """One leg: where it hinges, the turn that stows it, and its parts."""

    def __init__(self, gear, name, contact, side):
        spec = gear.spec
        self.name, self.side = name, side
        self.retractable = gear.retractable
        self.fairing = gear.fairing
        self.direction = spec.get("retract", "aft" if gear.steerable else "forward")
        if self.direction not in ("forward", "aft", "inward", "outward"):
            raise ValueError("gear %r: retract must be forward, aft, inward or outward" % gear.name)
        self.spec = spec
        self.wheels = int(spec.get("wheels", 1))
        self.door_deg = float(spec.get("door_deg", 90.0))
        self.r = 0.5 * gear.wheel_diameter
        self.w = gear.wheel_width
        self.axle = contact + np.array([0.0, 0.0, self.r])
        top = gear.attach.copy() if gear.attach is not None else contact + np.array([0.0, 0.0, 2.0 * self.r + 0.8])
        if side < 0 < top[1] or side > 0 > top[1]:
            top[1] = -top[1]
        self.hinge = top
        self.strut_r = 0.035 + 0.06 * gear.wheel_diameter
        # the stowing turn: the swing about the trunnion and the twist about
        # the strut, together
        self.strut = (self.hinge - self.axle) / np.linalg.norm(self.hinge - self.axle)
        axis = _swing(self.direction, side)
        angle = float(spec.get("retract_deg", 90.0))
        turn = spec.get("wheel_turn", 0.0)
        if turn == "flat":
            # the twist that leaves the stowed axle nearest vertical
            y = np.array([0.0, 1.0, 0.0])
            swing = _rotation(axis, angle)
            turn = float(max(range(-90, 91), key=lambda a: (abs((swing @ _rotation(self.strut, a) @ y)[2]), -abs(a))))
        self.set_stow(axis, angle, float(turn) * (1.0 if side >= 0 else -1.0))  # the mirror image twists the other way
        self.fitted = False

    def set_stow(self, axis, angle, twist):
        """The swing (unit axis through the hinge, degrees) and the twist
        about the strut (degrees) that stow the leg."""
        self.swing_axis = np.asarray(axis, float) / np.linalg.norm(axis)
        self.swing_deg, self.twist_deg = float(angle), float(twist)
        self.R = _rotation(self.swing_axis, self.swing_deg) @ _rotation(self.strut, self.twist_deg)

    def fit(self, probe):
        """Fits the stowing turn to the airframe (a meshkit.Probe): the swing's
        angle, its hinge axis canted - leaned about z, tilted towards z - and
        the wheel's twist: the one that leaves the stowed leg furthest inside,
        the plainest where several do. A main leg stays on its own side of
        the centre line. Returns how far the stowed leg stands out of the
        skin (m)."""
        pts = self.points()
        if "retract_deg" in self.spec and "wheel_turn" in self.spec:
            return float(max(probe(self.turned(pts))[0].max(), 0.0))
        base = _swing(self.direction, self.side)
        sign = 1.0 if self.side >= 0 else -1.0
        tilt_axis = np.cross(base, [0.0, 0.0, 1.0])  # turns the hinge axis towards z
        cands = []
        for lean in (0.0, -15.0, 15.0, -30.0, 30.0):
            for tilt in (0.0, 20.0, -20.0, 40.0, -40.0):
                axis = _rotation([0.0, 0.0, 1.0], lean * sign) @ _rotation(tilt_axis, tilt) @ base
                for angle in (90.0, 80.0, 100.0, 110.0, 70.0, 120.0):
                    swing = _rotation(axis, angle)
                    for twist in (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 75.0, -75.0, 90.0, -90.0):
                        cands.append((swing @ _rotation(self.strut, twist * sign),
                                      (abs(lean) + abs(tilt)) / 30.0 + abs(angle - 90.0) / 30.0 + abs(twist) / 90.0,
                                      (axis, angle, twist * sign)))
        stowed = np.array([self.hinge + (pts - self.hinge) @ R.T for R, _, _ in cands])
        d, _ = probe(stowed.reshape(-1, 3))
        out = np.maximum(d.reshape(len(cands), -1).max(axis=1), 0.0)
        if self.side != 0:  # not across the centre line, into the other leg's bay
            out += np.maximum(0.05 - self.side * stowed[:, :, 1], 0.0).max(axis=1)
        # inside (to a centimetre) first, then the plainest turn
        score = 100.0 * np.maximum(out - 0.01, 0.0) + np.array([p for _, p, _ in cands])
        k = int(np.argmin(score))
        self.set_stow(*cands[k][2])
        self.fitted = True
        return float(out[k])

    def mirrored(self, other):
        """Takes the mirror image of other's stowing turn (the pair's other
        leg): M R(a, t) M = R(-M a, t) for the reflection M."""
        M = np.diag([1.0, -1.0, 1.0])
        self.set_stow(-M @ other.swing_axis, other.swing_deg, -other.twist_deg)
        self.fitted = other.fitted

    def wheel_centres(self):
        n, w = self.wheels, self.w
        span = n * w + (n - 1) * 0.04
        return [self.axle + np.array([0.0, -0.5 * span + 0.5 * w + k * (w + 0.04), 0.0]) for k in range(n)], span

    def parts(self):
        """The leg's solids: tyre(s) and hub(s) on the axle, the strut down
        from the hinge, the oleo, and a fork (a nose wheel, a pair) or a stub
        axle (a single main wheel, from inboard)."""
        r, w, rs = self.r, self.w, self.strut_r
        y = np.array([0.0, 1.0, 0.0])
        centres, span = self.wheel_centres()
        nodes = []
        for c in centres:
            nodes.append({"prim": "cylinder", "material": TYRE, "a": c - 0.5 * w * y, "b": c + 0.5 * w * y, "r": r,
                          "round": min(0.45 * w, 0.3 * r)})
            nodes.append({"prim": "cylinder", "material": METAL, "a": c - (0.5 * w + 0.01) * y,
                          "b": c + (0.5 * w + 0.01) * y, "r": 0.55 * r, "round": 0.008})
        if self.side == 0 or self.wheels > 1:
            half = 0.5 * span + 0.7 * rs
            foot = self.axle + np.array([0.0, 0.0, r + 0.08 + 0.6 * rs])
            for s in (-1.0, 1.0):
                nodes.append({"prim": "capsule", "material": STRUT, "a": foot + s * half * y, "b": self.axle + s * half * y,
                              "r": 0.45 * rs})
            nodes.append({"prim": "capsule", "material": STRUT, "a": foot - half * y, "b": foot + half * y, "r": 0.5 * rs})
            nodes.append({"prim": "cylinder", "material": METAL, "a": self.axle - half * y, "b": self.axle + half * y,
                          "r": 0.3 * rs, "round": 0.004})
        else:
            foot = self.axle - np.sign(self.side) * (0.5 * span + 1.3 * rs) * y
            nodes.append({"prim": "cylinder", "material": METAL, "a": self.axle, "b": foot, "r": 0.4 * rs, "round": 0.004})
        mid = foot + 0.45 * (self.hinge - foot)
        nodes.append({"prim": "capsule", "material": STRUT, "a": mid, "b": self.hinge, "r": rs})     # the cylinder
        nodes.append({"prim": "capsule", "material": METAL, "a": foot, "b": mid, "r": 0.72 * rs})  # the oleo
        if self.fairing and not self.retractable:
            nodes.append(self._spat(span))
        return union(nodes)

    def _spat(self, span):
        """A wheel spat: a streamlined shell over the wheel, open underneath."""
        r = self.r
        c = self.axle + np.array([0.1 * r, 0.0, 0.1 * r])
        shell = {"prim": "ellipsoid", "material": SKIN, "centre": c, "radii": [1.75 * r, 0.5 * span + 0.06, 1.05 * r]}
        return {"op": "subtract", "k": 0.0, "a": shell, "cut_material": SKIN,
                "b": {"prim": "box", "material": SKIN, "centre": self.axle - np.array([0.0, 0.0, 1.35 * r]),
                      "half": [3.0 * r, 0.5 * span + 0.2, 0.6 * r]}}

    def points(self):
        """Points on the leg as built: its tyres' rims and its strut."""
        th = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        ring = np.stack([np.cos(th), np.zeros_like(th), np.sin(th)], axis=1) * self.r
        centres, _ = self.wheel_centres()
        pts = [c + np.array([0.0, s * 0.5 * self.w, 0.0]) + ring for c in centres for s in (-1.0, 1.0)]
        pts.append(np.array([self.hinge + t * (self.axle - self.hinge) for t in np.linspace(0.0, 1.0, 12)]))
        return np.vstack(pts)

    def turned(self, p, frac=1.0):
        """Points p with the leg turned frac of the way up."""
        R = _rotation(self.swing_axis, frac * self.swing_deg) @ _rotation(self.strut, frac * self.twist_deg)
        return self.hinge + (np.asarray(p, float) - self.hinge) @ R.T


def legs(aircraft):
    out = []
    for g in aircraft.gear:
        for name, pos in g.positions():
            side = 0 if abs(pos[1]) < 1e-6 else (1 if pos[1] > 0 else -1)
            out.append(Leg(g, name, np.asarray(pos, float), side))
    return out


def leg_scene(leg):
    cell = float(np.clip(leg.r / 40.0, 0.002, 0.01))
    return {"cell": cell, "error": 0.1 * cell, "safety": 2.0, "sharp_deg": 45.0, "max_triangles": 4000,
            "root": leg.parts()}


def skin_below(probe, x, y, z_top, depth=4.0):
    """The height where a vertical line down through (x, y) from z_top last
    leaves the solid (a meshkit.Probe) - the lowest skin there - or None when
    it misses."""
    zs = np.linspace(z_top, z_top - depth, 800)
    d, _ = probe(np.stack([np.full_like(zs, x), np.full_like(zs, y), zs], axis=1))
    inside = d < 0.0
    idx = np.flatnonzero(inside[:-1] & ~inside[1:])
    if len(idx) == 0:
        return None
    i = idx[-1]
    return float(zs[i] + (zs[i + 1] - zs[i]) * d[i] / (d[i] - d[i + 1]))


def _door(solid, foils, lo, hi, z_lo, z_hi, hinge, axis, deg, label):
    """A door: the skin inside the box (lo..hi in x-y, z_lo..z_hi), shrunk by
    half the gap, as a shell 2 cm thick."""
    c = np.array([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (z_lo + z_hi)])
    half = np.array([0.5 * (hi[0] - lo[0]) - 0.5 * GAP, 0.5 * (hi[1] - lo[1]) - 0.5 * GAP, 0.5 * (z_hi - z_lo)])
    box = {"prim": "box", "material": SKIN, "centre": c, "half": half, "round": 0.03}
    shell = {"op": "subtract", "k": 0.0, "cut_material": SKIN,
             "a": {"op": "intersect", "k": 0.0, "children": [solid, box]},
             "b": {"op": "offset", "r": -0.02, "child": solid}}
    return {"scene": {"cell": 0.005, "error": 0.002, "safety": 3.0, "sharp_deg": 40.0, "max_triangles": 1500,
                      "foils": foils, "root": shell},
            "hinge": np.asarray(hinge, float), "axis": np.asarray(axis, float), "deg": deg, "label": label}


def bays(aircraft, solid, foils):
    """Per retractable leg: the bay it folds into (a cut for the airframe),
    the doors over it, and how far the stowed leg would stand out of the skin
    (m; the airframe's own, before the bay is cut)."""
    from . import meshkit
    todo = [leg for leg in legs(aircraft) if leg.retractable]
    if not todo:
        return []
    with meshkit.Probe({"foils": foils, "root": solid}) as probe:
        fitted = {}
        for leg in sorted(todo, key=lambda l: l.side < 0):
            twin = fitted.get(leg.name.replace("Left ", "Right ", 1))
            if leg.side < 0 and twin is not None:
                leg.mirrored(twin)
            else:
                leg.fit(probe)
                fitted[leg.name] = leg
        return [_bay(leg, probe, solid, foils) for leg in todo]


def _bay(leg, probe, solid, foils):
    """One leg's bay, doors and protrusion (bays)."""
    pts = leg.points()
    stowed = leg.turned(pts)
    d, _ = probe(stowed)
    protrusion = float(max(d.max(), 0.0))
    c = stowed.mean(axis=0)
    z0 = skin_below(probe, c[0], c[1], stowed[:, 2].max() + 0.5)
    if z0 is None:
        z0 = float(stowed[:, 2].min())
    # the opening: under the stowed leg, and where the leg passes the skin
    near = [q for f in (0.0, 0.25, 0.5, 0.75) for q in leg.turned(pts, f) if q[2] > z0 - 0.15]
    xy = np.vstack([stowed] + ([np.array(near)] if near else []))[:, :2]
    lo, hi = xy.min(axis=0) - 0.04, xy.max(axis=0) + 0.04
    # the skin's heights over it: the doors' depth
    gx, gy = np.linspace(lo[0], hi[0], 9), np.linspace(lo[1], hi[1], 9)
    samples = [(x, y, skin_below(probe, x, y, stowed[:, 2].max() + 0.5)) for x in gx for y in gy]
    zs = [z for _, _, z in samples if z is not None] or [z0]
    # doors in the skin near the bay's own height: where the skin over the
    # footprint climbs further (onto a wing), the bay stays closed inside
    z_lo, z_hi = min(zs) - 0.05, min(max(zs) + 0.06, z0 + 0.25)
    cut_xy = np.array([(x, y) for x, y, z in samples if z is not None and z <= z_hi - 0.03])
    if len(cut_xy):
        step = np.array([gx[1] - gx[0], gy[1] - gy[0]])
        dlo, dhi = np.maximum(cut_xy.min(axis=0) - 0.5 * step, lo), np.minimum(cut_xy.max(axis=0) + 0.5 * step, hi)
    else:
        dlo, dhi = lo, hi
    top = max(stowed[:, 2].max() + 0.06, z_hi + 0.05)
    # the bay: the opening through the lower skin, and above it the space
    # the stowed leg needs - hollowed only where the airframe is at least
    # 3 cm thick, so it never breaks through another skin
    centre = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])]
    half = [0.5 * (hi[0] - lo[0]) + 0.5 * GAP, 0.5 * (hi[1] - lo[1]) + 0.5 * GAP]
    opening = {"prim": "box", "material": SKIN, "centre": [0.5 * (dlo[0] + dhi[0]), 0.5 * (dlo[1] + dhi[1]),
                                                             0.5 * (z_lo - 0.1 + z_hi + 0.03)],
               "half": [0.5 * (dhi[0] - dlo[0]) + 0.5 * GAP, 0.5 * (dhi[1] - dlo[1]) + 0.5 * GAP,
                        0.5 * (z_hi + 0.03 - z_lo + 0.1)], "round": 0.03 + 0.5 * GAP}
    space = {"op": "intersect", "k": 0.0, "children": [
        {"prim": "box", "material": SKIN, "centre": centre + [0.5 * (z_lo - 0.1 + top)],
         "half": half + [0.5 * (top - z_lo + 0.1)], "round": 0.03 + 0.5 * GAP},
        {"op": "offset", "r": -0.03, "child": solid}]}
    cut = {"op": "union", "k": 0.0, "children": [opening, space]}
    # the doors: a pair split along a nose leg's centre line, else one;
    # hinged along the outer edge, or for a leg swinging sideways the edge
    # it stows against
    lo, hi = dlo, dhi
    if leg.side == 0:
        ym = 0.5 * (lo[1] + hi[1])
        panes = [(lo, np.array([hi[0], ym]), lo[1]), (np.array([lo[0], ym]), hi, hi[1])]
    elif leg.direction in ("inward", "outward"):
        far = lo[1] if abs(stowed[:, 1].mean() - lo[1]) < abs(stowed[:, 1].mean() - hi[1]) else hi[1]
        panes = [(lo, hi, far)]
    else:
        panes = [(lo, hi, hi[1] if leg.side > 0 else lo[1])]
    doors = []
    for k, (a, b, ye) in enumerate(panes):
        yc = 0.5 * (a[1] + b[1])
        xm = 0.5 * (a[0] + b[0])
        ze = skin_below(probe, xm, ye - np.sign(ye - yc) * 0.02, z_hi + 0.5) or z0
        axis = np.array([np.sign(ye - yc) or 1.0, 0.0, 0.0])
        doors.append(_door(solid, foils, a, b, z_lo, z_hi, [xm, ye, ze], axis, leg.door_deg,
                           "%s door%s" % (leg.name, "" if len(panes) == 1 else " %d" % (k + 1))))
    return {"leg": leg, "cut": cut, "doors": doors, "protrusion": protrusion}
