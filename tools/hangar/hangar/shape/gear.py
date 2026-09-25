"""Landing gear for the 3D model.

Each leg is a strut with its oleo and wheel(s), built with the wheel on the
ground and the strut up to its attachment, where it hinges: as the gear
retracts the viewer swings it up about its trunnion and, at once, twists it
about the strut (two nested fsim:gear nodes), as real legs fold. A
retractable leg folds into a bay cut into the airframe, and doors - the skin
cut out over the bay - open before it comes down and close after it has gone
up. The opening is as wide as the leg needs to pass: its two doors, hinged
along the edges either side of the leg's swing, hang clear of it all the way
(model3d measures that).

Per gear in the design ([[gear]]):
    retract = "aft"         the way the leg swings up: forward, aft, inward or outward
                            (default: a steerable nose gear aft, main gear forward)
    retract_deg = 90        how far, and
    wheel_turn = 0          degrees the wheel twists about the strut on the way up
                            ("flat": lying flat); hangar fits what is not given - the
                            angle, the twist, and a hinge canted up to 40 deg, as real
                            trunnions are - to stow the leg inside the airframe through
                            as small an opening as it can, as near a plain 90 deg swing
                            as it can
    retract_axis = [x, y, z] the trunnion's axis, when it is known (the leg at the
                            gear's own position; its mirror image turns about the
                            mirror image): a positive turn about it folds the leg
    wheels = 1              side by side on one axle
    door_deg = 90           how far the doors open
    door_reach = 0.8        how far out (m) a door's hinge may move to clear the leg: more
                            for a wide multi-wheel truck (the B-52's)
    doors = false           an open well, without doors: the stowed wheel may stand out
                            of the skin up to its radius (the A-10's main wheels, half
                            out of their pods; the B-52's outriggers in its thin wingtips)
    fairing = true          (fixed gear) a spat over the wheel

A single leg (not mirrored) is a centre leg - a fork round its wheel, free to
fold across the centre line - also where it stands beside it (the A-10's and
the Su-25's nose wheels, clear of the gun).
"""
import numpy as np

from .airframe import GAP, METAL, SKIN, STRUT, TYRE, union

DOORS = 0.2  # the doors move while the gear position runs 0..0.2, the legs 0.2..1
DOOR = 0.02  # a door's thickness (m)
CLEAR = 0.04  # the open doors' clearance from the moving leg, and the stowed leg's from the closed doors (m)
DOOR_REACH = 0.8  # how far out a door's hinge may move to clear the leg (m; door_reach for a wide truck)


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
        self.steerable = gear.steerable
        self.fairing = gear.fairing
        self.direction = spec.get("retract", "aft" if gear.steerable else "forward")
        if self.direction not in ("forward", "aft", "inward", "outward"):
            raise ValueError("gear %r: retract must be forward, aft, inward or outward" % gear.name)
        self.spec = spec
        self.wheels = int(spec.get("wheels", 1))
        self.door_deg = float(spec.get("door_deg", 90.0))
        self.door_reach = float(spec.get("door_reach", DOOR_REACH))
        self.doors = bool(spec.get("doors", True))
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
        if "retract_axis" in spec:
            axis = np.asarray(spec["retract_axis"], float)
            if axis.shape != (3,) or np.linalg.norm(axis) < 1e-9:
                raise ValueError("gear %r: retract_axis = [x, y, z], not zero" % gear.name)
            if side < 0 < gear.position[1] or side > 0 > gear.position[1]:
                axis = -np.diag([1.0, -1.0, 1.0]) @ axis  # the mirror image's
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
        the wheel's twist: one that stows the leg inside, clear of the closed
        doors, and of those the one that needs the smallest opening on its
        way, as plain as it can. A main leg stays on its own side of the
        centre line all the way. Returns how far the stowed leg stands out of
        the skin (m)."""
        pts = self.points()
        given = [k for k in ("retract_axis", "retract_deg", "wheel_turn") if k in self.spec]
        if len(given) == 3 or (len(given) == 2 and "retract_axis" not in given):
            return float(max(probe(self.turned(pts))[0].max(), 0.0))
        base = self.swing_axis if "retract_axis" in self.spec else _swing(self.direction, self.side)
        sign = 1.0 if self.side >= 0 else -1.0
        tilt_axis = np.cross(base, [0.0, 0.0, 1.0])  # turns the hinge axis towards z
        cants = [(0.0, 0.0)] if "retract_axis" in self.spec else \
            [(lean, tilt) for lean in (0.0, -15.0, 15.0, -30.0, 30.0) for tilt in (0.0, 20.0, -20.0, 40.0, -40.0)]
        angles = [float(self.spec["retract_deg"])] if "retract_deg" in self.spec else [90.0, 80.0, 100.0, 110.0, 70.0, 120.0]
        turn = self.spec.get("wheel_turn")
        cands = []
        for lean, tilt in cants:
            axis = _rotation([0.0, 0.0, 1.0], lean * sign) @ _rotation(tilt_axis, tilt) @ base
            for angle in angles:
                swing = _rotation(axis, angle)
                if turn == "flat":  # the twist that leaves the stowed axle nearest vertical
                    y = np.array([0.0, 1.0, 0.0])
                    twists = [float(max(range(-90, 91), key=lambda a: (abs((swing @ _rotation(self.strut, a) @ y)[2]), -abs(a))))]
                    signed = True
                elif turn is not None:
                    twists, signed = [float(turn) * sign], True
                else:
                    twists, signed = [0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 75.0, -75.0, 90.0, -90.0], False
                for twist in twists:
                    t = twist if signed else twist * sign
                    cands.append((swing @ _rotation(self.strut, t),
                                  (abs(lean) + abs(tilt)) / 30.0 + abs(angle - 90.0) / 30.0 + abs(twist) / 90.0,
                                  (axis, angle, t)))
        stowed = np.array([self.hinge + (pts - self.hinge) @ R.T for R, _, _ in cands])
        d, _ = probe(stowed.reshape(-1, 3))
        # inside, clear of the closed doors
        out = np.maximum(d.reshape(len(cands), -1).max(axis=1) + DOOR + 0.5 * CLEAR, 0.0)
        # on its way: a quarter, half and three quarters up
        mids = np.array([[self.hinge + (pts - self.hinge) @ (_rotation(a, f * g) @ _rotation(self.strut, f * t)).T
                          for f in (0.25, 0.5, 0.75)] for _, _, (a, g, t) in cands]).reshape(len(cands), -1, 3)
        dm, _ = probe(mids.reshape(-1, 3))
        dm = dm.reshape(len(cands), -1)
        if self.side != 0:  # not across the centre line, into the other leg's bay
            y = np.concatenate([stowed[:, :, 1], mids[:, :, 1]], axis=1)
            out += np.maximum(0.05 - self.side * y, 0.0).max(axis=1)
        # the opening it needs: the leg's extent where it passes the skin or
        # hangs below it, near it (m2)
        z_skin = skin_below(probe, self.hinge[0], self.hinge[1], self.hinge[2] + 0.3)
        z_skin = self.axle[2] + self.r if z_skin is None else z_skin
        near = (dm > -0.05) & (mids[:, :, 2] > z_skin - 0.4)
        big = 1e9
        ext = [np.where(near, mids[:, :, i], -big).max(axis=1) - np.where(near, mids[:, :, i], big).min(axis=1)
               for i in (0, 1)]
        area = np.where(near.any(axis=1), np.clip(ext[0], 0.0, None) * np.clip(ext[1], 0.0, None), 0.0)
        # inside (to a centimetre) first, then the smallest opening and the plainest turn
        score = 100.0 * np.maximum(out - 0.01, 0.0) + area / 0.25 + np.array([p for _, p, _ in cands])
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
        """The leg's solids, all in one (part_groups)."""
        return union([n for g in self.part_groups().values() for n in (g["children"] if g.get("op") == "union" else [g])])

    def part_groups(self):
        """The leg's solids in the three pieces that move apart: the strut
        (the cylinder, down from the hinge), the oleo (its piston, and a fork
        - a nose wheel, a pair - or a stub axle - a single main wheel, from
        inboard - with a fixed gear's spat), which slides up the strut as it
        compresses and turns with the steering, and the wheel(s) on their
        axle, which roll."""
        r, w, rs = self.r, self.w, self.strut_r
        y = np.array([0.0, 1.0, 0.0])
        centres, span = self.wheel_centres()
        wheel, oleo = [], []
        for c in centres:
            wheel.append({"prim": "cylinder", "material": TYRE, "a": c - 0.5 * w * y, "b": c + 0.5 * w * y, "r": r,
                          "round": min(0.45 * w, 0.3 * r)})
            wheel.append({"prim": "cylinder", "material": METAL, "a": c - (0.5 * w + 0.01) * y,
                          "b": c + (0.5 * w + 0.01) * y, "r": 0.55 * r, "round": 0.008})
        foot = self.foot()
        if self.side == 0 or self.wheels > 1:
            half = 0.5 * span + 0.7 * rs
            for s in (-1.0, 1.0):
                oleo.append({"prim": "capsule", "material": STRUT, "a": foot + s * half * y, "b": self.axle + s * half * y,
                             "r": 0.45 * rs})
            oleo.append({"prim": "capsule", "material": STRUT, "a": foot - half * y, "b": foot + half * y, "r": 0.5 * rs})
            # the axle through the fork, turning with the wheels it joins
            wheel.append({"prim": "cylinder", "material": METAL, "a": self.axle - half * y, "b": self.axle + half * y,
                          "r": 0.3 * rs, "round": 0.004})
        else:
            oleo.append({"prim": "cylinder", "material": METAL, "a": self.axle, "b": foot, "r": 0.4 * rs, "round": 0.004})
        mid = foot + 0.45 * (self.hinge - foot)
        oleo.append({"prim": "capsule", "material": METAL, "a": foot, "b": mid, "r": 0.72 * rs})  # the piston
        if self.fairing and not self.retractable:
            oleo.append(self._spat(span))
        return {"strut": union([{"prim": "capsule", "material": STRUT, "a": mid, "b": self.hinge, "r": rs}]),
                "oleo": union(oleo), "wheel": union(wheel)}

    def foot(self):
        """The bottom of the oleo: above a fork, or inboard of a single wheel."""
        rs, y = self.strut_r, np.array([0.0, 1.0, 0.0])
        if self.side == 0 or self.wheels > 1:
            return self.axle + np.array([0.0, 0.0, self.r + 0.08 + 0.6 * rs])
        _, span = self.wheel_centres()
        return self.axle - np.sign(self.side) * (0.5 * span + 1.3 * rs) * y

    def slide(self):
        """The way the oleo slides as it compresses (up the strut), and how far
        along it per metre JSBSim compresses the unit (vertically)."""
        k = self.hinge - self.foot()
        k = k / np.linalg.norm(k)
        return k, 1.0 / max(abs(float(k[2])), 0.2)

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

    def surface(self, step=0.03):
        """Points all over the leg's skin (its parts' surfaces, about step
        apart): what must stay clear of the doors."""
        return _surface_points(self.parts(), step)

    def turned(self, p, frac=1.0):
        """Points p with the leg turned frac of the way up."""
        R = _rotation(self.swing_axis, frac * self.swing_deg) @ _rotation(self.strut, frac * self.twist_deg)
        return self.hinge + (np.asarray(p, float) - self.hinge) @ R.T


def _surface_points(node, step):
    """Points on the surfaces of a union of capsules and cylinders (a leg's
    parts), about step apart; other shapes are left out."""
    if node.get("op") == "union":
        return np.vstack([_surface_points(c, step) for c in node["children"]] or [np.zeros((0, 3))])
    if node.get("prim") not in ("capsule", "cylinder"):
        return np.zeros((0, 3))
    a, b, r = np.asarray(node["a"], float), np.asarray(node["b"], float), float(node["r"])
    k = b - a
    L = float(np.linalg.norm(k))
    k = k / L if L > 1e-9 else np.array([0.0, 0.0, 1.0])
    e1 = np.cross(k, [1.0, 0.0, 0.0] if abs(k[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(k, e1)
    th = np.linspace(0.0, 2.0 * np.pi, max(8, int(np.ceil(2.0 * np.pi * r / step))), endpoint=False)
    ring = np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2)
    pts = [a + t * k + r * ring for t in np.linspace(0.0, L, max(2, int(np.ceil(L / step)) + 1))]
    for end, s in ((a, -1.0), (b, 1.0)):  # the ends: a cap, or a flat face
        for f in np.linspace(0.0, 1.0, max(2, int(np.ceil(r / step)) + 1))[:-1]:
            if node["prim"] == "capsule":
                pts.append(end + s * r * np.sqrt(1.0 - f * f) * k + r * f * ring)
            else:
                pts.append(end + r * f * ring)
    return np.vstack(pts)


def legs(aircraft):
    out = []
    for g in aircraft.gear:
        for name, pos in g.positions():
            side = 0 if abs(pos[1]) < 1e-6 or not g.mirror else (1 if pos[1] > 0 else -1)
            out.append(Leg(g, name, np.asarray(pos, float), side))
    return out


def leg_scene(leg, part=None):
    """The mesher's scene of the leg, or of one of its part_groups."""
    cell = float(np.clip(leg.r / 40.0, 0.002, 0.01))
    return {"cell": cell, "error": 0.1 * cell, "safety": 2.0, "sharp_deg": 45.0, "max_triangles": 4000 if part is None else 2000,
            "root": leg.parts() if part is None else leg.part_groups()[part]}


def skin_below(probe, x, y, z_top, depth=4.0):
    """The height where a vertical line down through (x, y) from z_top last
    leaves the solid (a meshkit.Probe) - the lowest skin there - or None when
    it misses."""
    h = skin_heights(probe, [x], [y], z_top, depth)[0, 0]
    return None if np.isnan(h) else float(h)


def skin_heights(probe, xs, ys, z_top, depth=4.0, step=0.01):
    """The lowest skin's height over the grid xs x ys ([iy, ix]; nan where a
    vertical line down from z_top misses the solid)."""
    zs = np.arange(z_top, z_top - depth, -step)
    X, Y = np.meshgrid(np.asarray(xs, float), np.asarray(ys, float))
    n = X.size
    d, _ = probe(np.stack([np.repeat(X.ravel(), len(zs)), np.repeat(Y.ravel(), len(zs)), np.tile(zs, n)], axis=1))
    d = d.reshape(n, len(zs))
    leave = (d[:, :-1] < 0.0) & (d[:, 1:] >= 0.0)  # out of the solid, going down
    out = np.full(n, np.nan)
    for k in np.flatnonzero(leave.any(axis=1)):
        i = np.flatnonzero(leave[k])[-1]
        out[k] = zs[i] + (zs[i + 1] - zs[i]) * d[k, i] / (d[k, i] - d[k, i + 1])
    return out.reshape(X.shape)


def swept(leg, pts, steps=30):
    """Points pts of the leg at every step of its swing, down to stowed."""
    return np.vstack([leg.turned(pts, f) for f in np.linspace(0.0, 1.0, steps + 1)])


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
    from . import meshkit
    pts = leg.surface()
    stowed = leg.turned(pts)
    d, _ = probe(stowed)
    protrusion = float(max(d.max(), 0.0))
    # the leg on its way down: where it is inside the airframe (the bay's
    # space) and where it passes through the skin (the opening)
    moving = swept(leg, pts)
    dm, _ = probe(moving)
    inside = moving[dm < 0.0] if (dm < 0.0).any() else stowed
    through = moving[np.abs(dm) < 0.05] if (np.abs(dm) < 0.05).any() else stowed
    # across the swing (u) and along it (v): a leg swinging fore and aft
    # passes between doors hinged along x, one swinging sideways between
    # doors hinged along y
    base = _swing(leg.direction, leg.side)
    iu, iv = (1, 0) if abs(base[1]) >= abs(base[0]) else (0, 1)
    lo, hi = through[:, :2].min(axis=0) - CLEAR, through[:, :2].max(axis=0) + CLEAR
    # the lowest skin around it, 2.5 cm apart, with room for the doors to move out
    reach = np.array([0.1, 0.1])
    reach[iu] = leg.door_reach
    z_top = float(inside[:, 2].max()) + 0.3
    xs = np.arange(lo[0] - reach[0], hi[0] + reach[0] + 1e-9, 0.025)
    ys = np.arange(lo[1] - reach[1], hi[1] + reach[1] + 1e-9, 0.025)
    H = skin_heights(probe, xs, ys, z_top)
    X, Y = np.meshgrid(xs, ys)
    # the doors: a pair either side of the middle of the leg's swing, each
    # hinged on its outer edge and opening outwards, down; each edge moves
    # out until its open door clears the leg all the way
    um = 0.5 * (lo[iu] + hi[iu])
    UV = [X, Y]
    edges = [lo[iu], hi[iu]]
    limit = reach[iu] + 0.5 * (hi[iu] - lo[iu])
    doors_at = []
    with meshkit.Probe({"root": leg.parts()}) as legp:
        for k, sgn in ((0, -1.0), (1, 1.0)) if leg.doors else ():
            while True:
                door = _door_frame(H, UV, iu, iv, lo, hi, um, edges[k], sgn)
                if door is None or _door_clear(leg, legp, door) >= CLEAR or abs(edges[k] - um) > limit:
                    break
                edges[k] += sgn * 0.025
            doors_at.append(door)
    lo[iu], hi[iu] = edges
    within = (X >= lo[0]) & (X <= hi[0]) & (Y >= lo[1]) & (Y <= hi[1]) & ~np.isnan(H)
    z_lo = float(H[within].min()) if within.any() else float(through[:, 2].min())
    top = float(inside[:, 2].max()) + 0.06
    # each door is the skin over its pane, from its lowest point up to its
    # highest - its hinge, on a side that curves up - and no skin above that
    panes = []
    for door in doors_at:
        if door is None:
            continue
        r = door["rect"]
        cell = (X >= r[0]) & (X <= r[2]) & (Y >= r[1]) & (Y <= r[3]) & ~np.isnan(H)
        z_top = max(float(door["hinge"][2]), float(H[cell].max()) if cell.any() else z_lo) + 0.05
        panes.append((door, z_top))
    if not leg.doors:
        # an open well: the opening is just where the leg passes the skin
        cell = within
        panes.append(({"rect": np.array([lo[0], lo[1], hi[0], hi[1]])},
                      (float(H[cell].max()) if cell.any() else z_lo) + 0.05))
    # the bay: the opening through the skin under the doors, and above it the
    # space the leg passes through and stows in - hollowed only where the
    # airframe is at least 3 cm thick, so it never breaks through another skin
    ilo = np.minimum(inside[:, :2].min(axis=0) - CLEAR, lo)
    ihi = np.maximum(inside[:, :2].max(axis=0) + CLEAR, hi)
    opening = union([{"prim": "box", "material": SKIN,
                      "centre": [0.5 * (d["rect"][0] + d["rect"][2]), 0.5 * (d["rect"][1] + d["rect"][3]),
                                 0.5 * (z_lo - 0.1 + zt + 0.03)],
                      "half": [0.5 * (d["rect"][2] - d["rect"][0]) + 0.5 * GAP, 0.5 * (d["rect"][3] - d["rect"][1]) + 0.5 * GAP,
                               0.5 * (zt + 0.03 - z_lo + 0.1)], "round": 0.02} for d, zt in panes])
    space = {"op": "intersect", "k": 0.0, "children": [
        {"prim": "box", "material": SKIN, "centre": [0.5 * (ilo[0] + ihi[0]), 0.5 * (ilo[1] + ihi[1]), 0.5 * (z_lo - 0.1 + top)],
         "half": [0.5 * (ihi[0] - ilo[0]) + 0.5 * GAP, 0.5 * (ihi[1] - ilo[1]) + 0.5 * GAP, 0.5 * (top - z_lo + 0.1)],
         "round": 0.03 + 0.5 * GAP},
        {"op": "offset", "r": -0.03, "child": solid}]}
    cut = {"op": "union", "k": 0.0, "children": [opening, space]}
    doors = [_door(solid, foils, d["rect"][:2], d["rect"][2:], z_lo - 0.1, zt, d["hinge"], d["axis"], leg.door_deg,
                   "%s door %d" % (leg.name, k + 1)) for k, (d, zt) in enumerate(panes)] if leg.doors else []
    # how far out the stowed leg may stand: an open well's wheel up to its radius
    return {"leg": leg, "cut": cut, "doors": doors, "protrusion": protrusion, "allowed": 0.0 if leg.doors else leg.r}


def _door(solid, foils, lo, hi, z_lo, z_hi, hinge, axis, deg, label):
    """A door: the skin inside the box (lo..hi in x-y, z_lo..z_hi), shrunk by
    half the gap, as a shell DOOR thick."""
    c = np.array([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (z_lo + z_hi)])
    half = np.array([0.5 * (hi[0] - lo[0]) - 0.5 * GAP, 0.5 * (hi[1] - lo[1]) - 0.5 * GAP, 0.5 * (z_hi - z_lo)])
    box = {"prim": "box", "material": SKIN, "centre": c, "half": half, "round": 0.02}
    shell = {"op": "subtract", "k": 0.0, "cut_material": SKIN,
             "a": {"op": "intersect", "k": 0.0, "children": [solid, box]},
             "b": {"op": "offset", "r": -DOOR, "child": solid}}
    return {"scene": {"cell": 0.005, "error": 0.002, "safety": 3.0, "sharp_deg": 40.0, "max_triangles": 1500,
                      "foils": foils, "root": shell},
            "hinge": np.asarray(hinge, float), "axis": np.asarray(axis, float), "deg": deg, "label": label}


def _door_frame(H, UV, iu, iv, lo, hi, um, edge, sgn):
    """A door from um out to edge (sgn: the side, -1 towards lower u): its
    rect, hinge (on the skin at the edge's middle), axis and points on it -
    or None where there is no skin."""
    U, V = UV[iu], UV[iv]
    a, b = (edge, um) if sgn < 0 else (um, edge)
    inpane = (U >= a) & (U <= b) & (V >= lo[iv]) & (V <= hi[iv]) & ~np.isnan(H)
    if not inpane.any():
        return None
    rect = np.zeros(4)
    rect[iu], rect[2 + iu], rect[iv], rect[2 + iv] = a, b, lo[iv], hi[iv]
    # the hinge: on the skin, at the outermost skin across the middle of the door
    vm = 0.5 * (lo[iv] + hi[iv])
    vs = np.unique(V[inpane])
    row = inpane & (V == vs[np.argmin(np.abs(vs - vm))])
    cand = np.flatnonzero(row.ravel())
    k = cand[np.argmax(sgn * U.ravel()[cand])]
    hinge = np.zeros(3)
    hinge[iu], hinge[iv], hinge[2] = U.ravel()[k], vm, H.ravel()[k]
    # the turn that takes the door's free edge (towards um) down
    axis = np.zeros(3)
    axis[iv] = sgn if iu == 1 else -sgn
    P = np.stack([UV[0][inpane], UV[1][inpane], H[inpane]], axis=1)
    P = np.vstack([P, P + [0.0, 0.0, DOOR]])
    return {"rect": rect, "hinge": hinge, "axis": axis, "points": P}


def _door_clear(leg, legp, door, steps=24):
    """How far the door, open, stays from the leg as it swings (m)."""
    R = _rotation(door["axis"], leg.door_deg)
    opened = door["hinge"] + (door["points"] - door["hinge"]) @ R.T
    frames = []
    for f in np.linspace(0.0, 1.0, steps + 1):
        Rl = _rotation(leg.swing_axis, f * leg.swing_deg) @ _rotation(leg.strut, f * leg.twist_deg)
        frames.append(leg.hinge + (opened - leg.hinge) @ Rl)
    d, _ = legp(np.vstack(frames))
    return float(d.min())
