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
   half - are the only unknowns, and each equation is monotone, so the
   solution is unique where decambering every strip is not (post-stall
   sections make that ill-posed). In the linear range the result is exactly
   the lattice's; through stall the induced field collapses with the lift that
   made it - the tail sees less downwash as the wing stalls, a stalled half
   wing stops washing down the other. Past 30 deg of flow angle the induced
   angles fade out (by 60 deg): strip theory on the section polars, which
   cover the whole circle.

Forces per strip: lift normal to the effective flow (so induced drag is the
lift's tilt), section drag along it, section moment about the span axis.

Swept surfaces - the strips on them:

- in sideslip a swept wing's windward half is less swept than its leeward
  half and lifts more (simple sweep theory: lift ~ cos of the sweep the flow
  sees), the dihedral effect that grows with lift on fighters' wings;
- thin swept sections (leading edge swept 35 deg or more, strakes always)
  have the vortex regime (section.py): past the attached-flow limit the lost
  leading-edge suction turns into vortex lift, 1/cos(sweep) times the part of
  it realised - all of it for a sharp edge, less for a round one. The vortex
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
from .section import PolarSet, SectionPolar, linear_part, smoothstep
from .vlm import VLM, Lattice

RHO0, NU0 = 1.225, 1.46e-5

VORTEX_MIN_SWEEP = math.radians(35.0)   # leading-edge sweep from which a thin section's edge vortex holds
BURST_WIDTH = math.radians(10.0)        # the burst moves from the trailing edge to the apex over this
BURST_LOSS = 0.15                       # circulation lost behind a burst vortex
FED_BURST_EARLIER = 4.0                 # deg of strake sweep: a strake-fed wing's vortex bursts earlier
BURST_SIDESLIP = 0.5                    # of the sideslip that moves the breakdown (NASA's F-16 keeps its dihedral effect)


def burst_alpha(sweep):
    """Angle of attack (rad) at which a leading-edge vortex bursts at the
    trailing edge, from the leading edge's sweep (rad): Earnshaw & Lawford's
    trend for thin delta wings, 8 deg at 50 deg of sweep, +1.2 deg per deg."""
    return np.radians(np.clip(8.0 + 1.2 * (np.degrees(sweep) - 50.0), 2.0, 50.0))


def realised_vortex(sharpness):
    """The part of the lost leading-edge suction that a vortex realises, from
    the edge's sharpness (DATCOM's Delta-y, % chord): all of it for a sharp
    edge (Polhamus), none for a round one - whose edge holds its attainable
    suction and loses the rest to a diffuse separated flow. Calibrated on
    NASA TP-1538's F-16: its 4 %-thick cambered wing (Delta-y 1.2) makes no
    vortex lift of its own; its sharp strakes do."""
    return np.clip((1.0 - np.asarray(sharpness, float)) / 0.45, 0.0, 1.0)


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
                                       vortex=bool(self.vx["on"][k])))
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
        self.panel_group = group[L.strip]
        self.bodies = BodyAero(a, self.speed, NU0, mach)
        self._body_pts = self.bodies.points()
        if len(self._body_pts):
            self.vlm.set_body_points(self._body_pts)
        misc = an.get("drag_area", None)   # extra drag area, m^2 (gear, antennas, cooling...)
        self.drag_area = float(misc) if misc is not None else self._default_drag_area()
        self.S, self.b, self.c = a.S, a.b, a.c
        self.ref = L.ref
        self.clb_wing_body = self._wing_body_dihedral()

    def _vortex_setup(self):
        """Which strips have the vortex regime, the part of the suction their
        vortex realises and the sweep that sets its breakdown. A surface's
        spec may set vortex = false, or vortex_realised = 0..1."""
        L = self.lat
        n = L.n_strips
        on = np.zeros(n, bool)
        r = np.zeros(n)
        bd = self.geo["sweep_le"].copy()
        strakes = [s for s in self.aircraft.surfaces if s.kind == "strake"]
        feed = max((max(s.sweep_le_deg_max, 0.0) for s in strakes), default=0.0)
        # a wing behind a strake shares its vortex system, which bursts a little
        # before the strake's alone would (the wing's pressure rise downstream)
        feed_bd = max(feed - FED_BURST_EARLIER, 0.0)
        for si, surf in enumerate(L.surfaces):
            idx = np.flatnonzero(L.surface_index == si)
            spec = surf.spec
            if surf.kind in ("fin", "vtail") or spec.get("vortex", True) is False:
                continue
            sharp = np.array([L.airfoils[k].leading_edge_sharpness for k in idx])
            rr = realised_vortex(sharp) if surf.kind != "strake" else np.ones(len(idx))
            if "vortex_realised" in spec:
                rr = np.full(len(idx), float(spec["vortex_realised"]))
            fed = surf.kind == "wing" and feed > 0.0
            swept = (self.geo["sweep_le"][idx] >= VORTEX_MIN_SWEEP) | (surf.kind == "strake") | fed
            on[idx] = swept
            r[idx] = np.where(on[idx], rr, 0.0)
            if fed:
                bd[idx] = np.maximum(bd[idx], math.radians(feed_bd))
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
        delta = np.zeros(L.n_strips)
        for ch, d in controls.items():
            if ch in L.gain:
                delta = delta + L.gain[ch] * d
        F, M, info = self._surfaces(v, w, delta, controls, w_ind)
        Fb, Mb = self.bodies.forces(v, w, self.ref, info["body_induced"])
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

    def _surfaces(self, v, w, delta, controls, w_ind):
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
        vortex = (VX["r"] / np.cos(sweep_v) * B, 1.0 - BURST_LOSS * (1.0 - B))
        # the induced flow of vortex-lifting strips holds to higher angles
        w_ind = np.where(VX["on"], 1.0 - float(smoothstep((self._theta - 45.0) / 25.0)), w_ind)
        # 1. the lattice, decambered onto the sections' attached-flow lines (linear)
        g0 = V.circulation(v, w, controls)
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
        # 2. nonlinear strips: scale each group's induced angles by its lift ratio
        weight = L.chord**2 * L.width * cl_lin
        lin_norm = np.bincount(self.group, weight * cl_lin, self.n_groups)
        # groups whose linear lift is a trace of the largest (a fin at small
        # sideslip) wash nothing down worth solving for: they keep s = 1
        live = lin_norm > 1e-3 * max(lin_norm.max(), 1e-30)
        s = np.ones(self.n_groups)

        def lift_ratio(s):
            a_eff = a_geo + w_ind * (eps + ind @ s)
            cl = P.evaluate(a_eff, delta, vortex)[3]
            num = np.bincount(self.group, weight * cl, self.n_groups)
            return np.where(live, num / np.where(live, lin_norm, 1.0), 1.0), a_eff

        for it in range(40):
            f_s, _ = lift_ratio(s)
            res = f_s - s
            if np.max(np.abs(res)) < 1e-7:
                break
            # Newton on the few scale factors (numerical Jacobian of a monotone map)
            h = 1e-6
            Jf = np.empty((self.n_groups, self.n_groups))
            for g in range(self.n_groups):
                sp = s.copy()
                sp[g] += h
                Jf[:, g] = (lift_ratio(sp)[0] - f_s) / h
            try:
                step = np.linalg.solve(Jf - np.eye(self.n_groups), -res)
            except np.linalg.LinAlgError:
                step = 0.5 * res
            m = np.max(np.abs(step))
            if m > 0.3:
                step *= 0.3 / m
            s = np.clip(s + step, -0.5, 2.0)
        _, a_eff = lift_ratio(s)
        cl, cd, cm, _ = P.evaluate(a_eff, delta, vortex)
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
        qa = 0.5 * un**2 * L.area * f_sweep * k_loc
        Fk = (qa * cl)[:, None] * lift_dir + (qa * cd)[:, None] * fe
        Mk = np.cross(L.c4 - self.ref, Fk) + (qa * L.chord * cm)[:, None] * L.e
        body_ind = None
        if self.vlm.body_W is not None:
            scaled = gamma * (w_ind[L.strip] * s[self.panel_group])
            body_ind = np.einsum("mnk,n->mk", self.vlm.body_W, scaled)
        info = {"alpha_eff": a_eff, "alpha_geo": a_geo, "cl": cl, "cl_lin": cl_lin, "scale": dict(zip(self.group_names, s)),
                "burst": B, "beta_local": beta_k,
                "residual": float(np.max(np.abs(res))), "body_induced": body_ind}
        return Fk.sum(axis=0), Mk.sum(axis=0), info


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
