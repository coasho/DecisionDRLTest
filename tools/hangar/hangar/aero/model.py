"""The aircraft's aerodynamics at any flight condition.

Lifting surfaces, in two steps.

1. The linear lattice with viscous lift slopes. The vortex lattice (vlm.py)
   is decambered (Mukherjee & Gopalarathnam, J. Aircraft 43(5), 2006) so that
   every strip's lift follows the attached-flow line of its section polar -
   a linear problem, one solve. It gives each strip's effective angle and,
   from Biot-Savart, the part of it every lifting surface induces there (the
   wing's own downwash, the wing's downwash at the tail, ...).

2. Nonlinear strips. Each strip reads its full section polar (section.py:
   stall, post-stall, reversed flow, flaps) at its geometric angle plus the
   induced angles, each surface's contribution scaled by that surface's lift
   relative to its linear lift. Those few scale factors - one per surface
   half - are the only unknowns, where decambering every strip is ill-posed
   past the stall. In the linear range the result is exactly the lattice's;
   through stall the induced field collapses with the lift that made it - the
   tail sees less downwash as the wing stalls, a stalled half wing stops
   washing down the other. Past 30 deg of flow angle the induced angles fade
   out (by 60 deg): strip theory on the section polars, which cover the whole
   circle.

   An aircraft whose surfaces have the vortex regime (below) flies two flows
   past the stall, each solved on its own: the vortex flow, a lifting surface
   in the lattice's induced flow in full, and the separated flow, strip
   theory on flat plates. A strip's forces are the two flows' in proportion
   to its share of the vortex flow: a wing strip's falls from 1 at 45 deg of
   its angle of attack to 0 at 70, and the tails, canards and bodies, in the
   flow the wing makes, share the wing's. The lattice's induced flow fades
   out over the same angles of sideslip. Each flow decides before it solves
   where its sections leave the vortex regime - the vortex flow at the angles
   its lattice gives them. Decided at the angle being solved for, as a lone
   section decides it, the hand-over made the two flows two solutions of one
   problem - a section washed far down and still in the vortex regime, or a
   plate hardly washed down at all - and the tables jumped between them
   between 50 and 60 deg.

Forces per strip: lift normal to the effective flow (so induced drag is the
lift's tilt), section drag along it, section moment about the span axis.
Each strip's effective angle comes from its lift through a thin section's
2 pi slope and the neighbouring strips' chordwise vortices, so the tilt
overstates the induced drag of the loading the lattice gives - by about
15 % on plain wings of aspect ratio 6 to 10, and more where the lattice's few
chordwise panels resolve a cambered line poorly (a 6A section's uniform-load
line: 0.006 CL more drag). [analysis] induced_drag = "trefftz" takes it from
the Trefftz plane instead (trefftz_correction): the far-field downwash of
every surface's trailing vortices at each strip, half of it at the bound
vortex, from the spanwise loading alone, as potential flow has it. The
designs made before it keep the strips' own tilt. The
lift a strip's angle of attack makes acts where the lattice's own loading
puts it on the chord (vlm.py), not at the section's quarter chord: the other
strips' loading moves it - aft over a wing root behind a strake or a canard,
whose trailing vortices wash the front of the root down, forward towards a
swept wing's tips. So in the linear range the strips give the lattice's
moment as well as its lift. At their quarter chords, the F-16C's strakes
kept their own lift ahead of the reference point but not the load they move
aft on the wing root: its neutral point came out 2.8 % of the MAC ahead of
NASA's.

Swept surfaces - the strips on them:

- in sideslip a swept wing's windward half is less swept than its leeward
  half and lifts more (simple sweep theory: lift ~ cos of the sweep the flow
  sees), the dihedral effect that grows with lift on fighters' wings;
- thin swept sections have the vortex regime (section.py), coming in with
  the leading edge's sweep - none of it at 25 deg, all of it from 35 (a
  strake, and a wing behind one, always): a thin tail swept 34 deg does not
  stall like a two-dimensional section while one swept 36 deg keeps its
  lift, as a line drawn at 35 deg made them. Past the attached-flow limit the lost
  leading-edge suction turns into vortex lift, 1/cos(sweep) times the part of
  it realised - all of it for a sharp edge, less for a round one. An edge
  beside a body has less suction to lose: 1 - (a/d)^4 of it, d from the axis
  of a body a wide (Bryson's slender wing-body), none inside the body - a
  strake makes little vortex lift where it grows out of the fuselage's side,
  and the part of a surface inside the fuselage none. The vortex
  bursts at an angle of attack that rises with the sweep (Earnshaw & Lawford,
  ARC R&M 3424, 1964: at the trailing edge by about 8 deg at 50 deg of sweep,
  40 deg at 76), windward first in sideslip; the flow behind a burst vortex
  keeps most of the potential lift (Polhamus again). A wing fed by a strake or
  leading-edge extension shares the strake's vortex system, which holds to the
  strake's breakdown angle (the double delta).

Bodies: bodies.py, in the lattice's (scaled) induced flow. Fixed gear and
other excrescences: a drag area.

Inputs: alpha, beta, the non-dimensional rates p b/2V, q c/2V, r b/2V and
control channel deflections; outputs JSBSim's coefficients - CD, CY, CL in
wind axes, Cl, Cm, Cn in body axes about the aerodynamic reference point.
"""
import math

import numpy as np

from .bodies import BodyAero
from .section import PolarSet, SectionPolar, linear_part, smoothstep, vortex_held, wrap
from .vlm import VLM, Lattice

RHO0, NU0 = 1.225, 1.46e-5

VORTEX_SWEEP = (math.radians(25.0), math.radians(35.0))  # leading-edge sweep over which a thin section's edge vortex comes to hold
BURST_WIDTH = math.radians(18.0)        # the burst moves from the trailing edge to the apex over this
BURST_LOSS = 0.15                       # circulation lost behind a burst vortex
BURST_KEEP = 0.4                        # of its lift a burst vortex keeps: a 60 deg delta's lift rises
                                        # 15 deg past the burst reaching the trailing edge (Wentz & Kohlman)
FED_BURST_EARLIER = 4.0                 # deg of strake sweep: a strake-fed wing's vortex bursts earlier
BURST_SIDESLIP = 0.5                    # of the sideslip that moves the breakdown (NASA's F-16 keeps its dihedral effect)
CANARD_BURST_LATER = 8.0                # deg of angle of attack a close-coupled canard delays the wing's vortex burst
BLUNT_VORTEX_SWEEP = (math.radians(45.0), math.radians(60.0))  # a blunt edge's vortex forms from, and fully by, these sweeps
VORTEX_FLOW = (45.0, 70.0)              # deg of angle of attack (and sideslip) over which the vortex flow gives way
CL_FLOOR = 0.1                          # the least lift coefficient a lift ratio is measured against
TREFFTZ_CORE = 0.02                     # of a piece's width: the core of its own surface's trailing vortices
                                        # (another surface's are a strip wide, as the lattice's)
TREFFTZ_SPLIT = 4                       # pieces per strip, the loading linear between the strips' centres


def trefftz_correction(L, group, U, v, un, lift, lift_dir, wc):
    """Per strip, the force (n_s, 3) that turns the induced drag of its
    lift's tilt into the Trefftz plane's. Each strip's circulation (its lift
    normal to the local flow and the span, over its speed and width) is laid
    along its surface half, linear between the strips' centres and falling
    to nothing at a free tip, in TREFFTZ_SPLIT pieces a strip; each piece
    sheds a vortex at either end. In the plane normal to the free stream v
    they induce a far-field normal wash at every piece, and half of it at
    the bound vortex makes the piece's induced drag - summing to the whole
    system's (Munk's stagger theorem), from the spanwise loading alone: an
    elliptic loading's pi Gamma^2 / 8 within half a percent. U: the local air
    velocity at the strips, un its speed in their section planes, lift the
    circulation's force (signed, along lift_dir), group the surface half of
    each strip, wc the lattice's coupling (the correction fades with it)."""
    Uh = U / np.maximum(np.linalg.norm(U, axis=1), 1e-12)[:, None]
    F = lift[:, None] * lift_dir
    own = np.einsum("ij,ij->i", F, Uh)                   # the tilt's drag, along the local flow
    n = np.cross(Uh, L.e)
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    g = np.einsum("ij,ij->i", F, n) / np.maximum(un * L.width, 1e-12)   # circulation (rho = 1)
    # the pieces: each strip cut along its span, its circulation interpolated
    # along the surface half from root (eta 0) to tip, nothing at the tip
    m = TREFFTZ_SPLIT
    frac = (np.arange(m) + 0.5) / m
    pc, pw, pg, pe, pn, owner = [], [], [], [], [], []
    for gi in np.unique(group):
        idx = np.flatnonzero(group == gi)
        idx = idx[np.argsort(L.eta[idx])]
        w = L.width[idx]
        t0 = np.concatenate([[0.0], np.cumsum(w)[:-1]])
        tc = t0 + 0.5 * w
        ts = (t0[:, None] + frac[None, :] * w[:, None]).ravel()
        gs = np.interp(ts, np.concatenate([tc, [t0[-1] + w[-1]]]), np.concatenate([g[idx], [0.0]]))
        # the pieces' centres, along each strip's span line from its root side
        s = 1.0 if len(idx) < 2 or float(L.e[idx[0]] @ (L.c4[idx[-1]] - L.c4[idx[0]])) >= 0.0 else -1.0
        start = L.c4[idx] - (0.5 * s * w)[:, None] * L.e[idx]
        pc.append((start[:, None, :] + (s * frac[None, :] * w[:, None])[:, :, None] * L.e[idx][:, None, :]).reshape(-1, 3))
        pw.append(np.repeat(w / m, m))
        pg.append(gs)
        pe.append(np.repeat(L.e[idx], m, axis=0))
        pn.append(np.repeat(n[idx], m, axis=0))
        owner.append(np.repeat(idx, m))
    pc, pw, pg, pe, pn, owner = (np.concatenate(x) for x in (pc, pw, pg, pe, pn, owner))
    surf = L.surface_index[owner]
    # each piece's ends, the bound vortex running along n x U for positive g
    half = 0.5 * pw[:, None] * pe
    flip = np.einsum("ij,ij->i", pe, np.cross(pn, Uh[owner])) < 0.0
    a = np.where(flip[:, None], pc + half, pc - half)
    b = np.where(flip[:, None], pc - half, pc + half)
    # the Trefftz plane: p1, p2 normal to the free stream, p1 x p2 along it
    vh = v / np.linalg.norm(v)
    p1 = np.array([0.0, 1.0, 0.0]) - vh[1] * vh
    p1 /= np.linalg.norm(p1)
    p2 = np.cross(vh, p1)
    P = lambda x: np.stack([x @ p1, x @ p2], axis=-1)  # noqa: E731
    pos = np.concatenate([P(b), P(a)])                   # the trailing vortices, +g at b, -g at a
    gam = np.concatenate([pg, -pg])
    r = P(pc)[:, None, :] - pos[None, :, :]
    same = surf[:, None] == np.concatenate([surf, surf])[None, :]
    core = np.where(same, TREFFTZ_CORE * pw[:, None], L.width[owner][:, None])
    k = gam[None, :] / (2.0 * math.pi * (np.einsum("ijk,ijk->ij", r, r) + core ** 2))
    w = -(k * r[:, :, 1]).sum(axis=1) * (pn @ p1) + (k * r[:, :, 0]).sum(axis=1) * (pn @ p2)   # far-field normal wash
    span = np.linalg.norm(P(b) - P(a), axis=1)
    trefftz = np.bincount(owner, -0.5 * pg * w * span, L.n_strips)   # each strip's share of the induced drag
    return (np.asarray(wc) * (trefftz - own))[:, None] * Uh


def burst_alpha(sweep):
    """Angle of attack (rad) at which a leading-edge vortex bursts at the
    trailing edge, from the leading edge's sweep (rad): Earnshaw & Lawford's
    trend for thin delta wings, 8 deg at 50 deg of sweep, +1.2 deg per deg."""
    return np.radians(np.clip(8.0 + 1.2 * (np.degrees(sweep) - 50.0), 2.0, 50.0))


def realised_vortex(sharpness, sweep=0.0):
    """The part of the lost leading-edge suction that a vortex realises, from
    the edge's sharpness (DATCOM's Delta-y, % chord) and sweep (rad): all of
    it for a sharp edge (Polhamus), none for a round one of moderate sweep -
    whose edge holds its attainable suction and loses the rest to a diffuse
    separated flow. Calibrated on NASA TP-1538's F-16: its 4 %-thick cambered
    wing (Delta-y 1.2, 40 deg) makes no vortex lift of its own; its sharp
    strakes do. Past 45 deg of sweep a blunt edge's separation rolls up into a
    vortex too, fully by 60 deg, as on the blunt-edged 65 deg delta of the
    second vortex flow experiment (VFE-2; Luckring, Aerosp. Sci. Tech. 2013)."""
    sharp = np.clip((1.0 - np.asarray(sharpness, float)) / 0.45, 0.0, 1.0)
    swept = np.clip((np.asarray(sweep, float) - BLUNT_VORTEX_SWEEP[0]) / (BLUNT_VORTEX_SWEEP[1] - BLUNT_VORTEX_SWEEP[0]), 0.0, 1.0)
    return np.maximum(sharp, swept)


def edge_shielding(aircraft, points):
    """How much of its leading-edge suction the bodies leave an edge at each
    of points (n, 3): 1 - (a/d)^4 for an edge d from a body's axis, a the
    body's half-width at the edge's height, none inside it - the edge's
    singularity in the slender theory of a wing on a body (Bryson, J. Aero.
    Sci. 21(6), 1954): the cross flow round the body takes it out of an edge
    at the body's side, and with it the edge's vortex."""
    f = np.ones(len(points))
    for b in aircraft.bodies:
        if not b.aero:
            continue
        for side in b.copies():
            for k, (x, y, z) in enumerate(points):
                if not b.x[0] <= x <= b.x[-1]:
                    continue
                w, top, bot, yc, _ = (float(v) for v in b.section(x))
                zc, nt, nb = (float(v) for v in b.halves(x))
                h, n = (top - zc, nt) if z >= zc else (zc - bot, nb)
                if w <= 0.0 or h <= 0.0 or abs(z - zc) >= h:
                    continue
                a = 0.5 * w * (1.0 - (abs(z - zc) / h) ** n) ** (1.0 / n)
                d = abs(y - yc * side)
                f[k] *= 0.0 if d <= a else 1.0 - (a / d) ** 4
    return f


def wind_axes(alpha, beta):
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    xw = np.array([ca * cb, sb, sa * cb])
    yw = np.array([-ca * sb, cb, -sa * sb])
    zw = np.array([-sa, 0.0, ca])
    return xw, yw, zw


def to_body(vec_design):
    return np.array([-vec_design[0], vec_design[1], -vec_design[2]])


class AeroModel:
    def __init__(self, aircraft, speed=None, mach=0.0, density=1.0, wake_deg=None):
        a = self.aircraft = aircraft
        an = a.spec.get("analysis", {})
        self.speed = float(speed or an.get("speed", 50.0))       # m/s, for Reynolds numbers
        self.mach = mach
        self.lat = Lattice(a, density * float(an.get("density", 1.0)))
        self.vlm = VLM(self.lat, mach) if wake_deg is None else VLM(self.lat, mach, wake_deg)
        L = self.lat
        laminar = float(an.get("laminar_fraction", 0.0))
        overrides = an.get("airfoil", {})
        ar = {s.name: s.aspect_ratio for s in a.surfaces}
        self.geo = strip_geometry(a, L)
        self.vx = self._vortex_setup()
        polars = []
        for k in range(L.n_strips):
            foil = L.airfoils[k]
            surf = L.surfaces[L.surface_index[k]]
            ctrl = L.controls[k]
            cf = (1.0 - L.hinge[k]) if L.hinge[k] < 1.0 else None
            kind = getattr(ctrl, "kind", "plain") if ctrl is not None else "plain"
            o = dict(overrides.get(foil.name.split("~")[0], {}))
            o.update(overrides.get(surf.name, {}))
            polars.append(SectionPolar(foil, re=self.speed * L.chord[k] / NU0, mach=mach, aspect_ratio=ar[surf.name],
                                       laminar=laminar, overrides=o, flap_chord=cf, flap_kind=kind,
                                       vortex=float(self.vx["on"][k])))
        self.polars = PolarSet(polars)
        # groups: one per surface half; the induced field of each scales on its own
        group = np.zeros(L.n_strips, int)
        names = []
        for si, surf in enumerate(L.surfaces):
            idx = np.flatnonzero(L.surface_index == si)
            halves = [idx[: len(idx) // 2], idx[len(idx) // 2 :]] if surf.mirror else [idx]
            for h, part in zip(halves, ("R", "L") if surf.mirror else ("",)):
                group[h] = len(names)
                names.append(surf.name + (" " + part if part else ""))
        self.group, self.group_names = group, names
        self.n_groups = len(names)
        self.group_surface = np.array([L.surface_index[np.flatnonzero(group == g)[0]] for g in range(len(names))])
        self.surf_area = np.array([s.area for s in L.surfaces])
        self.surf_ar = np.array([s.aspect_ratio for s in L.surfaces])
        # a strake is part of its wing's planform, not one of its own: its
        # vortex keeps the strips' suction (the F-16's strakes calibrate it)
        self.planform_suction = np.array([L.surfaces[i].kind != "strake" for i in L.surface_index])
        # the wing's planform, strakes and all
        self.wing_strip = np.array([s is a.wing or s.kind == "strake" for s in L.surfaces])[L.surface_index]
        self.panel_group = group[L.strip]
        self.bodies = BodyAero(a, self.speed, NU0, mach)
        self._body_pts = self.bodies.points()
        if len(self._body_pts):
            self.vlm.set_body_points(self._body_pts)
        misc = an.get("drag_area", None)   # extra drag area, m^2 (gear, antennas, cooling...)
        self.drag_area = float(misc) if misc is not None else self._default_drag_area()
        induced = str(an.get("induced_drag", "strips")).lower()
        if induced not in ("strips", "trefftz"):
            raise ValueError("[analysis] induced_drag must be \"strips\" or \"trefftz\", not %r" % induced)
        self.trefftz = induced == "trefftz"
        self.S, self.b, self.c = a.S, a.b, a.c
        self.ref = L.ref
        self.clb_wing_body = self._wing_body_dihedral()

    def _vortex_setup(self):
        """How much of the vortex regime each strip has (0 to 1, with the
        sweep), the part of the suction their vortex realises and the sweep
        that sets its breakdown. A surface's spec may set vortex = false, or
        vortex_realised = 0..1."""
        L = self.lat
        n = L.n_strips
        on = np.zeros(n)
        r = np.zeros(n)
        bd = self.geo["sweep_le"].copy()
        strakes = [s for s in self.aircraft.surfaces if s.kind == "strake"]
        feed = max((max(s.sweep_le_deg_max, 0.0) for s in strakes), default=0.0)
        # a wing behind a strake shares its vortex system, which bursts a little
        # before the strake's alone would (the wing's pressure rise downstream)
        feed_bd = max(feed - FED_BURST_EARLIER, 0.0)
        # a close-coupled canard (its trailing edge within a wing MAC of the
        # wing's root leading edge) washes the wing's inboard flow down and
        # steadies its vortex: the vortex bursts later (the Rafale's, the
        # Typhoon's, the J-10's lift keeps rising past 30 deg)
        wing = self.aircraft.wing
        w_le = wing.sections[0].le[0]
        canard_delay = 0.0
        for c in self.aircraft.surfaces:
            if c.kind == "canard":
                te = max(sec.le[0] + sec.chord for sec in c.sections)
                if w_le - te < wing.mac[0]:
                    canard_delay = CANARD_BURST_LATER
        # an edge beside a body loses its suction, and its vortex, to the body
        shield = edge_shielding(self.aircraft, L.c4 - 0.25 * L.chord[:, None] * L.c)
        for si, surf in enumerate(L.surfaces):
            idx = np.flatnonzero(L.surface_index == si)
            spec = surf.spec
            if surf.kind in ("fin", "vtail") or spec.get("vortex", True) is False:
                continue
            sharp = np.array([L.airfoils[k].leading_edge_sharpness for k in idx])
            rr = realised_vortex(sharp, self.geo["sweep_le"][idx]) if surf.kind != "strake" else np.ones(len(idx))
            if "vortex_realised" in spec:
                rr = np.full(len(idx), float(spec["vortex_realised"]))
            fed = surf.kind == "wing" and feed > 0.0
            lo, hi = VORTEX_SWEEP
            swept = smoothstep((self.geo["sweep_le"][idx] - lo) / (hi - lo))
            on[idx] = 1.0 if surf.kind == "strake" or fed else swept
            r[idx] = on[idx] * rr * shield[idx]
            if fed:
                bd[idx] = np.maximum(bd[idx], math.radians(feed_bd))
            if surf.kind == "wing" and canard_delay:
                bd[idx] = bd[idx] + math.radians(canard_delay / 1.2)   # as that much more sweep (burst_alpha: 1.2 deg/deg)
        return {"on": on, "r": r, "sweep_bd": bd}

    def _wing_body_dihedral(self):
        """The rolling moment in sideslip that a wing's vertical position on
        the fuselage adds (DATCOM 5.1.2.1): 1.2 sqrt(A) (z_w/b)(2 d/b) per rad,
        z_w the root quarter chord's height below the fuselage centre line - a
        high wing is more stable. The lattice has no fuselage to make it."""
        wing = self.aircraft.wing
        bodies = [b for b in self.aircraft.bodies if b.kind == "fuselage"]
        if not wing.mirror or not bodies:
            return 0.0
        root = wing.sections[0]
        x = root.le[0] + 0.25 * root.chord
        body = bodies[0]
        if not body.x[0] <= x <= body.x[-1]:
            return 0.0
        w, top, bot, _, _ = body.section(x)
        z_w = 0.5 * (top + bot) - root.le[2]
        d = float(np.sqrt(w * (top - bot)))
        return float(1.2 * np.sqrt(wing.aspect_ratio) * (z_w / self.b) * (2.0 * d / self.b))

    def _default_drag_area(self):
        """Gear, cooling, leakage and protuberances when the spec gives none
        (Raymer 12.5.6-12.5.8)."""
        area = 0.0
        for g in self.aircraft.gear:
            if g.retractable:
                continue
            n = 2 if g.mirror else 1
            area += n * (0.25 if not g.fairing else 0.13) * g.wheel_diameter * g.wheel_width
            area += n * 0.05 * 0.04 * max(0.3, g.attach[2] - g.position[2] if g.attach is not None else 0.6)  # strut
        for e in self.aircraft.engines:
            if e.type == "piston":
                area += 0.0012 * self.aircraft.S * (e.power_kw / 100.0) ** 0.5  # cooling
        if any(e.type == "turbofan" for e in self.aircraft.engines):
            # a jet fighter's leakage, protuberances, inlet spill and nozzle base:
            # CD 0.002, some 10-15 % of its skin friction (Raymer 12.5.8)
            area += 0.002 * self.aircraft.S
        return area

    # -- one condition --------------------------------------------------------------------------
    def evaluate(self, alpha, beta=0.0, p=0.0, q=0.0, r=0.0, controls=None, detail=False):
        """Coefficients at alpha, beta (rad), non-dimensional rates and
        control channel deflections {channel: rad}."""
        controls = controls or {}
        L = self.lat
        xw, yw, zw = wind_axes(alpha, beta)
        v = np.array([xw[0], -xw[1], xw[2]])               # air velocity, design frame (V = 1)
        w = np.array([-2.0 * p / self.b, 2.0 * q / self.c, -2.0 * r / self.b])  # body rates -> design frame
        theta = math.degrees(math.acos(max(-1.0, min(1.0, v[0]))))  # flow angle off the x axis
        w_ind = 1.0 - float(smoothstep((theta - 30.0) / 30.0))
        self._theta = theta
        # the wakes leave along the free stream, seen in the plane of symmetry
        self.vlm.select(math.atan2(v[2], max(v[0], 1e-6)))
        own = L.deflections(controls) if controls else None
        delta = L.strip_deflections(own) if controls else np.zeros(L.n_strips)
        F, M, info = self._surfaces(v, w, delta, own, w_ind)
        Fb, Mb = self.bodies.forces(v, w, self.ref, info["body_induced"])
        info["body_moment"] = Mb
        F = F + Fb
        M = M + Mb
        F = F + 0.5 * self.drag_area * v   # excrescences, along the flow
        # wing-body interference in sideslip (a body-axis rolling moment)
        M = M - np.array([self.clb_wing_body * math.sin(beta) * math.cos(alpha) * w_ind * 0.5 * self.S * self.b, 0.0, 0.0])
        return self._coefficients(F, M, alpha, beta, info if detail else None)

    def _coefficients(self, F, M, alpha, beta, info):
        qS = 0.5 * self.S
        Fb, Mb = to_body(F), to_body(M)
        xw, yw, zw = wind_axes(alpha, beta)
        out = {"CD": -Fb @ xw / qS, "CY": Fb @ yw / qS, "CL": -Fb @ zw / qS,
               "Cl": Mb[0] / (qS * self.b), "Cm": Mb[1] / (qS * self.c), "Cn": Mb[2] / (qS * self.b),
               "CX": Fb[0] / qS, "CZ": Fb[2] / qS}
        if info is not None:
            out["detail"] = info
        return out

    def _surfaces(self, v, w, delta, own, w_ind):
        L, P, V = self.lat, self.polars, self.vlm
        two_pi = 2.0 * math.pi
        # strip velocities (free stream and rotation) in their section planes
        U = v[None, :] - np.cross(w[None, :], L.c4 - self.ref)
        Un = U - np.einsum("ij,ij->i", U, L.e)[:, None] * L.e
        un = np.maximum(np.linalg.norm(Un, axis=1), 1e-9)
        f = Un / un[:, None]
        a_geo = np.arctan2(np.einsum("ij,ij->i", f, L.u), np.einsum("ij,ij->i", f, L.c))
        # sideslip each strip sees along its span (positive: from the tip, windward)
        G = self.geo
        beta_k = np.arctan2(-np.einsum("ij,ij->i", U, G["outboard"]), np.einsum("ij,ij->i", U, L.c))
        beta_k = np.where(G["side"] != 0, beta_k, 0.0)
        # simple sweep theory: lift ~ cos(sweep - beta) / (cos sweep cos beta); the
        # cos beta is in the strip's in-plane speed already
        bc = np.clip(beta_k, -math.radians(25.0), math.radians(25.0))
        # (the attached flow's effect: it fades as the flow separates at high alpha)
        fade = (1.0 - smoothstep((np.abs(beta_k) - math.radians(30.0)) / math.radians(20.0))) *             (1.0 - smoothstep((np.abs(a_geo) - math.radians(20.0)) / math.radians(20.0)))
        f_sweep = 1.0 + fade * (np.cos(G["sweep_c4"] - bc) / (np.cos(G["sweep_c4"]) * np.cos(bc)) - 1.0)
        # vortex regime: strength from the sweep, breakdown from the sweep the flow sees
        VX = self.vx
        sweep_v = np.clip(G["sweep_le"], 0.0, math.radians(85.0))
        B = 1.0 - smoothstep((np.abs(a_geo) - burst_alpha(VX["sweep_bd"] - BURST_SIDESLIP * beta_k)) / BURST_WIDTH)
        vortex = (VX["r"] / np.cos(sweep_v) * (BURST_KEEP + (1.0 - BURST_KEEP) * B), 1.0 - BURST_LOSS * (1.0 - B))
        # 1. the lattice, decambered onto the sections' attached-flow lines (linear)
        g0 = V.circulation(v, w, L.lattice_deflections(own, a_geo))
        gs0 = L.S @ g0
        H = V.decamber_response(v, w)
        qfac = 2.0 / (un * L.chord)
        qH = qfac[:, None] * H
        a0_thin = L.alpha0_thin - L.tau * delta
        a_vis, al0_vis = linear_part(P, delta)
        J = a_vis[:, None] * (qH / two_pi + np.eye(L.n_strips)) - qH
        rhs = -(a_vis * (a0_thin + qfac * gs0 / two_pi - al0_vis) - qfac * gs0)
        dec = np.linalg.solve(J, rhs)
        gamma = g0 + V.decamber_panels(v, w) @ dec
        cl_lin = qfac * (L.S @ gamma)
        a_lin = a0_thin + cl_lin / two_pi + dec
        # induced angles at each strip, by the group (surface half) inducing them
        gg = np.zeros((L.n_panels, self.n_groups))
        gg[np.arange(L.n_panels), self.panel_group] = gamma
        wk = np.einsum("snk,ng->sgk", V.W_strip, gg)               # (n_s, groups, 3)
        ind = np.einsum("sgk,sk->sg", wk, L.u) / un[:, None]           # upwash angle per group
        eps = a_lin - (a_geo + ind.sum(axis=1))                        # what Biot-Savart does not account for
        # 2. nonlinear strips: each group's induced angles scaled by its lift
        # over its linear lift (_scales), with the surfaces that have the
        # vortex regime flying two flows (the module's docstring). The wing's
        # strips (its strakes' too) have their share of the vortex flow from
        # their own angle of attack; the other surfaces, and the bodies, fly in
        # the flow the wing makes, and share the wing's. Within the vortex flow
        # a tail or a canard leaves the vortex regime at the angle its lattice
        # gives it (the wing's downwash, its turn); within the separated one the
        # wing is a flat plate, and a turned canard or tail keeps its own
        # regime, as strip theory would. The lattice's induced flow fades out in
        # sideslip, as the flow leaves the plane of symmetry its wake is laid in.
        lin = {"a_geo": a_geo, "ind": ind, "eps": eps, "cl_lin": cl_lin, "gamma": gamma, "un": un, "delta": delta,
               "f_sweep": f_sweep, "U": U, "v": v}
        turn = np.where(P.flap_chord >= 0.999, delta, 0.0)
        on = VX["on"]
        coupling = np.full(L.n_strips, w_ind)
        if on.any():
            lo, hi = (math.radians(x) for x in VORTEX_FLOW)
            sideslip = abs(math.asin(max(-1.0, min(1.0, v[1]))))
            c_vx = 1.0 - float(smoothstep((sideslip - lo) / (hi - lo)))
            share = 1.0 - smoothstep((np.abs(wrap(a_geo + turn)) - lo) / (hi - lo))
            mean = float(np.average(share[self.wing_strip], weights=L.area[self.wing_strip]))
            share = np.where(self.wing_strip, share, mean)
            a_vx = a_geo + c_vx * (a_lin - a_geo)        # the lattice's angle at that coupling
            plate = np.where(self.wing_strip, 0.0, vortex_held(wrap(a_geo + turn)))
            flows = []
            if share.max() > 0.0:                        # the vortex flow
                flows.append((share, mean, on * c_vx + (1.0 - on) * coupling, vortex + (vortex_held(wrap(a_vx + turn)),)))
            if share.min() < 1.0:                        # the separated flow
                flows.append((1.0 - share, 1.0 - mean, coupling, vortex + (plate,)))
        else:
            mean = 1.0
            flows = [(np.ones(L.n_strips), 1.0, coupling, vortex)]
        Fk = Mk = 0.0
        body_ind = None
        parts = []
        for part, part_bodies, wc, vx in flows:
            s, a_eff, res = self._scales(lin, wc, vx)
            f_k, m_k, b_k, info = self._strips(lin, s, a_eff, wc, vx)
            Fk, Mk = Fk + part[:, None] * f_k, Mk + part[:, None] * m_k
            if b_k is not None:
                body_ind = part_bodies * b_k if body_ind is None else body_ind + part_bodies * b_k
            info["residual"] = res
            parts.append((part_bodies, info))
        info = dict(max(parts, key=lambda x: x[0])[1])    # the larger flow's strips
        info.update({"alpha_geo": a_geo, "cl_lin": cl_lin, "burst": B, "beta_local": beta_k, "eps": eps, "a_lin": a_lin,
                     "dec": dec, "a0_thin": a0_thin, "vortex_share": mean, "body_induced": body_ind, "strip_moment": Mk,
                     "residual": max(p[1]["residual"] for p in parts), "flows": [(p, i["scale"]) for p, i in parts]})
        return Fk.sum(axis=0), Mk.sum(axis=0), info

    def _scales(self, lin, wc, vortex):
        """The groups' scale factors s = F(s): each group's lift over its
        linear lift, its induced angles scaled so, at coupling wc. Newton on
        the few unknowns (a numerical Jacobian) with each group's step held
        short; a group on the rising side of a stall's corner steps towards its
        F instead. Where that wanders, Newton again from its best point, each
        step cut back until it brings the residual down."""
        L, P = self.lat, self.polars
        a_geo, eps, ind, delta, cl_lin = lin["a_geo"], lin["eps"], lin["ind"], lin["delta"], lin["cl_lin"]
        weight = L.chord**2 * L.width * cl_lin
        # the lift ratio is measured against a lift coefficient of at least
        # CL_FLOOR: a group with less linear lift than that (a fin at small
        # sideslip, a wing half at no incidence in a sideslip) washes too little
        # down to solve for, and keeps its linear induced flow (s -> 1)
        floor = CL_FLOOR**2 * np.bincount(self.group, L.chord**2 * L.width, self.n_groups)
        lin_norm = np.bincount(self.group, weight * cl_lin, self.n_groups) + floor
        # a group's own induced flow scales with its lift where it washes its
        # strips down; where it washes them up (a loading of mixed sign - a
        # tail in a large sideslip - its one part's upwash on the other) it
        # keeps its linear value: scaled, it would feed the lift it came from
        own = self.group[:, None] == np.arange(self.n_groups)[None, :]
        up = own * np.clip(np.sign(ind) * cl_lin[:, None] / CL_FLOOR, 0.0, 1.0)
        fixed = (up * ind).sum(axis=1)
        ind = ind * (1.0 - up)

        def lift_ratio(s):
            a_eff = a_geo + wc * (eps + fixed + ind @ s)
            cl = P.evaluate(a_eff, delta, vortex)[3]
            return (np.bincount(self.group, weight * cl, self.n_groups) + floor) / lin_norm, a_eff

        def newton_step(s, res):
            h = 1e-6
            A = np.empty((self.n_groups, self.n_groups))
            for g in range(self.n_groups):
                sp = s.copy()
                sp[g] += h
                A[:, g] = (lift_ratio(sp)[0] - (res + s)) / h
            A -= np.eye(self.n_groups)
            # a group on the rising side of its stall's corner feeds itself
            # (its F climbs faster than its s): Newton would lead it away from
            # its root, so it steps towards its F instead
            A[np.diag_indices(self.n_groups)] = np.minimum(np.diag(A), -0.25)
            try:
                return np.linalg.solve(A, -res)
            except np.linalg.LinAlgError:
                return res

        # |F| is bounded (the floor): the lift ratio of a group with a small
        # linear lift of one sign and a real one of the other is legitimately
        # large and negative
        lo, hi, tol = -30.0, 30.0, 1e-7
        s = np.ones(self.n_groups)
        res = lift_ratio(s)[0] - s
        best = (np.linalg.norm(res), s, res)
        # Newton, each group's step held short where its residual is (the
        # stall's corners make the map rough) ...
        for it in range(40):
            if np.max(np.abs(best[2])) < tol:
                break
            limit = np.maximum(0.3, 0.5 * np.abs(res))
            s = np.clip(s + np.clip(newton_step(s, res), -limit, limit), lo, hi)
            res = lift_ratio(s)[0] - s
            if np.linalg.norm(res) < best[0]:
                best = (np.linalg.norm(res), s, res)
        # ... and where that wanders, from the best point with each step cut
        # back until it brings the residual down
        _, s, res = best
        for it in range(20):
            if np.max(np.abs(res)) < tol:
                break
            step = np.clip(newton_step(s, res), -0.3, 0.3)
            for cut in range(8):
                s_new = np.clip(s + step, lo, hi)
                res_new = lift_ratio(s_new)[0] - s_new
                if np.linalg.norm(res_new) < np.linalg.norm(res):
                    break
                step *= 0.5
            else:
                break
            s, res = s_new, res_new
        return s, lift_ratio(s)[1], float(np.max(np.abs(res)))

    def _strips(self, lin, s, a_eff, wc, vortex):
        """Each strip's force and moment at its solved angle, and the flow
        the lifting surfaces induce at the bodies."""
        L, P = self.lat, self.polars
        a_geo, ind, delta, un, gamma = lin["a_geo"], lin["ind"], lin["delta"], lin["un"], lin["gamma"]
        # the leading edge's suction sees the angle less the surface's own
        # downwash as Polhamus's analogy takes it - CL / (pi A) of its planform:
        # the 3D suction CL (alpha - alpha_i), which a low-aspect-ratio wing's
        # strip angles (set by their lift) understate - and the other surfaces'
        # induced flow
        cl_circ = P.evaluate(a_eff, delta, vortex)[3]
        lift = np.bincount(L.surface_index, cl_circ * L.area * un**2, len(L.surfaces)) / self.surf_area
        own = self.group_surface[None, :] == L.surface_index[:, None]
        other = np.where(own, 0.0, ind * s[None, :]).sum(axis=1)
        a_s = a_geo + wc * (other - lift[L.surface_index] / (math.pi * self.surf_ar[L.surface_index]))
        cl, cd, cm, _ = P.evaluate(a_eff, delta, vortex, np.where(self.planform_suction, a_s, a_eff), self.vlm.x_ac)
        # forces: lift normal to the effective flow, drag along it
        fe = np.cos(a_eff)[:, None] * L.c + np.sin(a_eff)[:, None] * L.u
        lift_dir = np.cross(fe, L.e)
        # Kutta-Joukowski with the local velocity: the induced velocity is normal
        # to the chord, so the air's speed along the chord stays that of the free
        # stream, and the force of the circulation is its cos(alpha_geo) /
        # cos(alpha_eff) times the section value (the 3D potential normal force
        # K_p sin(alpha) cos(alpha) of Polhamus; 1 at small angles)
        a_i = a_geo - a_eff
        k_loc = np.clip(np.cos(a_i) - np.tan(np.clip(a_eff, -1.4, 1.4)) * np.sin(a_i), 0.5, 1.2)
        qa = 0.5 * un**2 * L.area * lin["f_sweep"] * k_loc
        Fk = (qa * cl)[:, None] * lift_dir + (qa * cd)[:, None] * fe
        if self.trefftz:
            Fk = Fk + trefftz_correction(L, self.group, lin["U"], lin["v"], un, qa * cl, lift_dir, wc)
        Mk = np.cross(L.c4 - self.ref, Fk) + (qa * L.chord * cm)[:, None] * L.e
        body_ind = None
        if self.vlm.body_W is not None:
            scaled = gamma * (wc[L.strip] * s[self.panel_group])
            body_ind = np.einsum("mnk,n->mk", self.vlm.body_W, scaled)
        info = {"alpha_eff": a_eff, "cl": cl, "scale": dict(zip(self.group_names, s)), "induced": ind * s[None, :],
                "w_ind": wc}
        return Fk, Mk, body_ind, info


def strip_geometry(aircraft, L):
    """Per strip: the leading-edge and quarter-chord sweep (rad) of its
    surface there, which half of a mirrored lifting surface it is on (+1
    right, -1 left, 0 a single surface or a fin) and its outboard direction."""
    n = L.n_strips
    sweep_le, sweep_c4 = np.zeros(n), np.zeros(n)
    side = np.zeros(n)
    out = np.zeros((n, 3))
    for k in range(n):
        surf = L.surfaces[L.surface_index[k]]
        sweep_le[k], sweep_c4[k] = surf.sweep_at(L.eta[k])
        if surf.mirror and surf.kind not in ("fin", "vtail"):
            side[k] = 1.0 if L.c4[k, 1] >= 0.0 else -1.0
            out[k] = side[k] * L.e[k]
    return {"sweep_le": sweep_le, "sweep_c4": sweep_c4, "side": side, "outboard": out}
