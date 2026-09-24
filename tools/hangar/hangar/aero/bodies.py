"""Fuselages, nacelles and booms: forces from slender-body theory and
viscous cross flow.

- Potential part (Munk; Multhopp's fuselage method is this with the wing's
  flow field): the force per unit length is rho U d/dx(S_a v_c), with S_a the
  apparent-mass area of the section for the cross-flow direction and v_c the
  local cross-flow velocity - free stream, rotation, and the up- and downwash
  the lifting surfaces induce at the body (from the vortex lattice). A closed
  body gets no net lift from it, only the destabilising Munk moment, reduced
  ahead of the wing's upwash and behind its downwash as Multhopp describes.
- Viscous cross flow (Allen & Perkins, NACA TR 1048): each section is a
  cylinder in the cross flow, drag q_c eta C_dc times its width across the
  flow - the lift and moment a body makes at high angles, and in sideslip.
- Friction along the axis: turbulent skin friction on the wetted area times
  a fineness form factor (Raymer 12.30).

Sections inside a wing root (where the lattice's carry-through panels carry
the load) have no potential or cross-flow force, as in Multhopp.

The fuselage and the intakes, booms and nacelles that stand against it are
one body to the air: at each station their sections' union - its width and
height across the flow, its area - and only their exposed skin rubs. An
intake swallows the air ahead of it rather than pushing it aside: its mouth
adds no apparent mass. The air it captures is turned into the duct instead,
the inlet's normal force rho U S_c v_c at the lip (S_c its capture area).
"""
import numpy as np

from .section import skin_friction

ATTACHED = ("intake", "boom", "nacelle")  # kinds that merge with a fuselage they overlap


def _outline(body, x, side, n=96):
    """Points round a body's section at x (its copy on side)."""
    w, top, bot, yc, _ = body.section(x)
    zc, nt, nb = body.halves(x)
    th = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    e = np.where(s >= 0.0, 2.0 / nt, 2.0 / nb)
    y = float(yc) * side + 0.5 * float(w) * np.sign(c) * np.abs(c) ** e
    z = float(zc) + np.where(s >= 0.0, float(top - zc), float(zc - bot)) * np.sign(s) * np.abs(s) ** e
    return y, z


def _inside(body, x, side, y, z):
    """Whether points (y, z) lie inside a body's section at x."""
    w, top, bot, yc, _ = body.section(x)
    zc, nt, nb = body.halves(x)
    if w <= 0.0 or top <= bot:
        return np.zeros(np.shape(y), bool)
    up = z >= zc
    b = np.where(up, max(float(top - zc), 1e-9), max(float(zc - bot), 1e-9))
    n = np.where(up, float(nt), float(nb))
    return np.abs((y - float(yc) * side) / (0.5 * float(w))) ** n + np.abs((z - float(zc)) / b) ** n < 1.0 - 1e-9


def _covered(spans):
    """The length the union of intervals (a, b) covers."""
    total, end = 0.0, -np.inf
    for a, b in sorted(spans):
        if b <= end:
            continue
        total += b - max(a, end)
        end = b
    return total


class _Union:
    """A fuselage and the bodies merged with it, as one body: at each station
    the union's width, height, centre, area and exposed perimeter."""

    def __init__(self, members, stations):
        self.members = members
        self.name = members[0][0].name
        self.kind = "fuselage"
        x0 = min(b.x[0] for b, _ in members)
        x1 = max(b.x[-1] for b, _ in members)
        self.x = np.linspace(x0, x1, stations + 1)
        self.w, self.h, self.yc, self.zc, self.area, self.perimeter = (np.zeros(len(self.x)) for _ in range(6))
        self.w_pot, self.h_pot = np.zeros(len(self.x)), np.zeros(len(self.x))
        # each intake copy's mouth: (lip point, capture area)
        self.inlets = []
        for b, side in members:
            if b.kind == "intake":
                w, top, bot, yc, _ = b.section(b.x[0])
                zc, _, _ = b.halves(b.x[0])
                lip = getattr(b, "lip", 0.0)
                capture = float(b.area_at(b.x[0])) * max(1.0 - 2.0 * lip / max(float(min(w, top - bot)), 1e-9), 0.0) ** 2
                self.inlets.append((np.array([b.x[0], float(yc) * side, float(zc)]), capture))
        from .mach import bodies_area
        unique = []
        for b, _ in members:
            if all(b is not o for o in unique):
                unique.append(b)
        self.area = bodies_area(unique, self.x)
        for i, x in enumerate(self.x):
            here = [(b, s) for b, s in members if b.x[0] <= x <= b.x[-1]]
            ys, zs, per = [], [], 0.0
            for b, s in here:
                y, z = _outline(b, x, s)
                if len(y) == 0:
                    continue
                exposed = np.ones(len(y), bool)
                for o, so in here:
                    if o is not b or so != s:
                        exposed &= ~_inside(o, x, so, y, z)
                seg = np.hypot(np.diff(np.append(y, y[0])), np.diff(np.append(z, z[0])))
                per += float(np.sum(seg * exposed))
                ys.append(y)
                zs.append(z)
            if ys:
                # the width and height the cross flow meets: what the parts cover
                # across it, not the gaps between them (nacelles apart from the
                # fuselage)
                self.w[i] = _covered([(y.min(), y.max()) for y in ys])
                self.h[i] = _covered([(z.min(), z.max()) for z in zs])
                solid = [k for k, (b, _) in enumerate(here) if b.kind != "intake"]
                self.w_pot[i] = _covered([(ys[k].min(), ys[k].max()) for k in solid])
                self.h_pot[i] = _covered([(zs[k].min(), zs[k].max()) for k in solid])
                y, z = np.concatenate(ys), np.concatenate(zs)
                self.yc[i], self.zc[i] = 0.5 * (y.max() + y.min()), 0.5 * (z.max() + z.min())
                self.perimeter[i] = per
        self.length = float(x1 - x0)
        d = np.sqrt(4.0 * max(float(self.area.max()), 1e-9) / np.pi)
        self.fineness = self.length / max(d, 1e-9)
        self.n = np.array([np.mean([float(b.section(x)[4]) for b, _ in members if b.x[0] <= x <= b.x[-1]] or [2.0])
                           for x in self.x])

    @property
    def wetted(self):
        return float(np.trapezoid(self.perimeter, self.x))


def merged_groups(bodies):
    """The bodies as the air sees them: each fuselage with the intakes,
    booms and nacelles that overlap it (as (body, side) copies), and every
    other body alone."""
    fuselages = [b for b in bodies if b.kind == "fuselage"]
    taken = set()
    groups = []
    for f in fuselages:
        members = [(f, 1.0)]
        for b in bodies:
            if b is f or b.kind not in ATTACHED or id(b) in taken:
                continue
            lo, hi = max(f.x[0], b.x[0]), min(f.x[-1], b.x[-1])
            if hi <= lo:
                continue
            xs = np.linspace(lo, hi, 12)
            touching = False
            for x in xs:
                for side in b.copies():
                    y, z = _outline(b, x, side, 48)
                    if np.any(_inside(f, x, 1.0, y, z)):
                        touching = True
                        break
                if touching:
                    break
            if touching:
                members += [(b, side) for side in b.copies()]
                taken.add(id(b))
        groups.append(members)
        taken.add(id(f))
    groups += [[(b, None)] for b in bodies if id(b) not in taken]
    return groups


class BodyAero:
    def __init__(self, aircraft, re_speed, nu=1.46e-5, mach=0.0, stations=60):
        self.parts = []
        self.inlets = []  # (lip point, capture area): the captured air's momentum turned
        for group in merged_groups([b for b in aircraft.bodies if b.aero]):
            if len(group) > 1:
                self._merged(aircraft, _Union(group, stations), re_speed, nu, mach)
                continue
            body = group[0][0]
            for side in body.copies():
                xs = np.linspace(body.x[0], body.x[-1], stations + 1)
                w, top, bot, yc, n = body.section(xs)
                h = np.maximum(top - bot, 0.0)
                y = yc * side
                z = 0.5 * (top + bot)
                pts = np.column_stack([xs, y, z])
                # apparent-mass areas: a section moving vertically carries a
                # cylinder of air on its width, sideways on its height
                s_v = np.pi * w**2 / 4.0
                s_h = np.pi * h**2 / 4.0
                # cross-flow drag coefficient: circle 1.2, boxier sections up to ~1.8
                cdc = np.clip(1.2 + 0.2 * (n - 2.0), 1.0, 1.8)
                fine = body.fineness
                eta = float(np.clip(0.55 + 0.02 * fine, 0.55, 0.85))  # Allen & Perkins' finite-length factor
                inside = np.zeros(len(xs), bool)
                for s in aircraft.surfaces:
                    root = s.sections[0]
                    if abs(root.le[1] - np.interp(root.le[0], xs, y)) > 0.5 * np.interp(root.le[0], xs, w) + 1e-6:
                        continue
                    zc = np.interp(root.le[0], xs, z)
                    hc = np.interp(root.le[0], xs, h)
                    if abs(root.le[2] - zc) > 0.5 * hc + 0.05 * root.chord or s.kind in ("fin", "vtail"):
                        continue
                    inside |= (xs >= root.le[0]) & (xs <= root.le[0] + root.chord)
                # potential flow holds from the nose to where the flow separates:
                # x0 = 0.378 l + 0.527 x1, x1 where the section shrinks fastest (DATCOM 4.2.1.1)
                area = np.pi * w * h / 4.0
                x1 = xs[int(np.argmin(np.gradient(area, xs)))]
                x0 = xs[0] + 0.378 * body.length + 0.527 * (x1 - xs[0])
                potential = xs <= x0
                re = re_speed * body.length / nu
                ff = 1.0 + 60.0 / fine**3 + fine / 400.0 if body.kind not in ("nacelle", "pod", "intake") else 1.0 + 0.35 / fine
                wet = self._wetted(body)
                self.parts.append(dict(name=body.name, x=xs, pts=pts, w=w, h=h, s_v=s_v, s_h=s_h, cdc=cdc, eta=eta,
                                       inside=inside, potential=potential, x0=x0,
                                       friction=skin_friction(re, mach) * ff * wet, wet=wet))

    def _merged(self, aircraft, u, re_speed, nu, mach):
        """A fuselage merged with the bodies against it: one part, its sections
        the union's."""
        xs, w, h = u.x, u.w, u.h
        pts = np.column_stack([xs, u.yc, u.zc])
        # the apparent mass of what pushes the air aside (not an intake's mouth)
        s_v = np.pi * u.w_pot**2 / 4.0
        s_h = np.pi * u.h_pot**2 / 4.0
        cdc = np.clip(1.2 + 0.2 * (u.n - 2.0), 1.0, 1.8)
        eta = float(np.clip(0.55 + 0.02 * u.fineness, 0.55, 0.85))
        inside = np.zeros(len(xs), bool)
        for s in aircraft.surfaces:
            root = s.sections[0]
            if abs(root.le[1] - np.interp(root.le[0], xs, u.yc)) > 0.5 * np.interp(root.le[0], xs, w) + 1e-6:
                continue
            zc = np.interp(root.le[0], xs, u.zc)
            hc = np.interp(root.le[0], xs, h)
            if abs(root.le[2] - zc) > 0.5 * hc + 0.05 * root.chord or s.kind in ("fin", "vtail"):
                continue
            inside |= (xs >= root.le[0]) & (xs <= root.le[0] + root.chord)
        x1 = xs[int(np.argmin(np.gradient(u.area, xs)))]
        x0 = xs[0] + 0.378 * u.length + 0.527 * (x1 - xs[0])
        re = re_speed * u.length / nu
        fine = u.fineness
        ff = 1.0 + 60.0 / fine**3 + fine / 400.0
        wet = u.wetted
        self.parts.append(dict(name=u.name, x=xs, pts=pts, w=w, h=h, s_v=s_v, s_h=s_h, cdc=cdc, eta=eta,
                               inside=inside, potential=xs <= x0, x0=x0,
                               friction=skin_friction(re, mach) * ff * wet, wet=wet))
        for lip, capture in u.inlets:
            self.inlets.append((lip, capture))

    @staticmethod
    def _wetted(body):
        v, t, _ = body.skin(48, 32)
        a = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1)
        return float(a.sum()) / len(body.copies())

    def points(self):
        pts = [p["pts"] for p in self.parts] + [np.array([lip]) for lip, _ in self.inlets]
        return np.vstack(pts) if pts else np.zeros((0, 3))

    def forces(self, v, w, ref, induced=None, rho=1.0):
        """Force and moment (design frame, about ref) for air velocity v and
        rotation w; induced: (n_points, 3) lattice-induced velocity at
        points() (the wing's up- and downwash), or None."""
        F = np.zeros(3)
        M = np.zeros(3)
        k = 0
        for p in self.parts:
            pts = p["pts"]
            n = len(pts)
            vel = v[None, :] - np.cross(w[None, :], pts - ref)
            if induced is not None:
                vel = vel + induced[k : k + n]
            k += n
            U = vel[:, 0]
            vy, vz = vel[:, 1], vel[:, 2]
            dx = np.gradient(p["x"])
            live = ~p["inside"]
            # potential (slender body): f = rho U d/dx(S_a v_c), up to the separation station
            fz = rho * U * np.gradient(p["s_v"] * vz, p["x"]) * p["potential"]
            fy = rho * U * np.gradient(p["s_h"] * vy, p["x"]) * p["potential"]
            # viscous cross flow
            vc = np.hypot(vy, vz)
            width = np.sqrt((p["w"] * vz) ** 2 + (p["h"] * vy) ** 2) / np.maximum(vc, 1e-12)
            dc = 0.5 * rho * vc * p["eta"] * p["cdc"] * width  # times the cross-flow velocity vector
            fy = fy + dc * vy
            fz = fz + dc * vz
            f = np.column_stack([np.zeros(n), fy, fz]) * (dx * live)[:, None]
            # friction along the axis, spread over the length
            fax = 0.5 * rho * U * np.abs(U) * p["friction"] / (p["x"][-1] - p["x"][0])
            f[:, 0] += fax * dx
            F += f.sum(axis=0)
            M += np.cross(pts - ref, f).sum(axis=0)
        # each intake turns the air it captures into its duct: rho U S_c times
        # the cross flow at its lip
        for lip, capture in self.inlets:
            vel = v - np.cross(w, lip - ref)
            if induced is not None:
                vel = vel + induced[k]
            k += 1
            f = rho * vel[0] * capture * np.array([0.0, vel[1], vel[2]])
            F += f
            M += np.cross(lip - ref, f)
        return F, M
