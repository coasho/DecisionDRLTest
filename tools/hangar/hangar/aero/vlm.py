"""Vortex lattice: the linear aerodynamics of all lifting surfaces together.

Horseshoe vortices (Katz & Plotkin, "Low-Speed Aerodynamics", 12.3): each
panel carries a bound vortex on its quarter-chord line and two trailing legs
running aft along +x (a body-fixed wake, as in AVL); flow tangency holds at
the three-quarter-chord control points, whose normals carry the camber slope.
Forces are Kutta-Joukowski on the bound vortices with the local velocity
(free stream, rotation and every induced velocity), so induced drag and the
interference between surfaces - the wing's downwash on the tail - come out of
the same solution. Compressibility: the Prandtl-Glauert transformation
(stretch x by 1/beta, keep the local angles).

Everything a flight condition needs is linear in the free stream (V), the
rotation (omega), the control deflections and the strips' decambering angles
(the nonlinear model's unknowns, nonlinear.py), so the solutions for unit
values of each are computed once and a condition is a weighted sum.

Frame: the design frame (x aft, y right, z up), moments about the aerodynamic
reference point. The "air velocity" is the air's velocity relative to the
aircraft: (V, 0, 0) is level flight at zero incidence.
"""
import numpy as np

from .section import flap_tau, thin_airfoil

FOUR_PI = 4.0 * np.pi


def _segments(x, p1, p2, rc2):
    """Velocity at points x (M, 3) induced by unit vortex segments p1->p2
    (N, 3); rc2 (M, N) squared core radius. Returns (M, N, 3)."""
    r1 = x[:, None, :] - p1[None, :, :]
    r2 = x[:, None, :] - p2[None, :, :]
    r0 = p2 - p1
    cr = np.cross(r1, r2)
    cr2 = np.einsum("mnk,mnk->mn", cr, cr)
    l0 = np.einsum("nk,nk->n", r0, r0)
    n1 = np.sqrt(np.einsum("mnk,mnk->mn", r1, r1))
    n2 = np.sqrt(np.einsum("mnk,mnk->mn", r2, r2))
    dot = np.einsum("nk,mnk->mn", r0, r1 / np.maximum(n1, 1e-12)[..., None] - r2 / np.maximum(n2, 1e-12)[..., None])
    # Vatistas (n = 2) core of radius f |r0|: 1/h^2 becomes 1/sqrt(h^4 + rc^4),
    # exact far from the line, smooth through it; with |r1 x r2| = h |r0|
    return cr * (dot / (FOUR_PI * np.sqrt(cr2 * cr2 + (rc2 * (l0 * l0)[None, :]) ** 2)))[..., None]


def _legs(x, p, rc2, width2):
    """Velocity at x induced by unit semi-infinite vortices from p to +x
    infinity. Returns (M, N, 3)."""
    r = x[:, None, :] - p[None, :, :]
    h2 = r[..., 1] ** 2 + r[..., 2] ** 2
    nr = np.sqrt(h2 + r[..., 0] ** 2)
    coef = (1.0 + r[..., 0] / np.maximum(nr, 1e-12)) / (FOUR_PI * np.sqrt(h2 * h2 + (rc2 * width2[None, :]) ** 2))
    out = np.zeros(r.shape)
    out[..., 1] = -r[..., 2] * coef   # d x r with d = +x: (0, -rz, ry)
    out[..., 2] = r[..., 1] * coef
    return out


def reflect(points, z0):
    out = np.array(points, float, copy=True)
    out[..., 2] = 2.0 * z0 - out[..., 2]
    return out


def horseshoe_velocity(x, a, b, same, chunk=256):
    """Velocity at points x (M, 3) induced by unit horseshoes a->b with
    legs to +x (N). `same` (M, N) bool: the point lies on the horseshoe's own
    surface (a thin core) or on another - there the core is as wide as the
    horseshoe, so the trailing legs act as the continuous vortex sheet they
    stand for (whose normal velocity is smooth through the sheet: a tail in
    the wing's wake plane sees downwash, not the spikes of discrete lines).
    Returns (M, N, 3)."""
    width2 = np.einsum("nk,nk->n", b - a, b - a)
    out = np.empty((len(x), len(a), 3))
    for s in range(0, len(x), chunk):
        sl = slice(s, s + chunk)
        rc2 = np.where(same[sl], 0.02**2, 1.0)
        out[sl] = _segments(x[sl], a, b, rc2) + _legs(x[sl], b, rc2, width2) - _legs(x[sl], a, rc2, width2)
    return out


class Lattice:
    """Every surface's strips and panels, flattened into arrays.

    Strips (n_s): chord, width, area, quarter-chord point c4, frame c/u/e,
    airfoil, control, the channel gain of their control, surface index.
    Panels (n_p): bound vortex a->b, control point, normal, strip index,
    whether it lies behind a hinge (moves with the control)."""

    def __init__(self, aircraft, density=1.0):
        self.aircraft = aircraft
        self.ref = np.asarray(aircraft.aero_point, float)
        rows = []
        for si, surf in enumerate(aircraft.surfaces):
            base_ns = surf.nominal_ns or (24 if surf.kind == "wing" else 12)
            base_nc = surf.nominal_nc or (8 if surf.kind == "wing" else 6)
            d = surf.strips(max(4, int(round(base_ns * density))), max(3, int(round(base_nc * density))))
            rows.append((si, surf, d))
        self.surfaces = [r[1] for r in rows]
        # a fin standing on a body sees the body as a partial wall at its root:
        # image horseshoes (mirrored about the root plane) with a strength k raise
        # its effective aspect ratio as DATCOM's body end-plate factor (~1.5) does
        self.reflect = {si: _root_reflection(aircraft, surf) for si, surf, _ in rows}
        cat = lambda k: np.concatenate([d[k] for _, _, d in rows])  # noqa: E731
        self.chord, self.width, self.area = cat("chord"), cat("width"), cat("area")
        self.c4, self.c, self.u, self.e = cat("c4"), cat("c"), cat("u"), cat("e")
        self.eta, self.twist, self.hinge = cat("eta"), cat("twist"), cat("hinge")
        self.surface_index = np.concatenate([np.full(len(d["chord"]), si) for si, _, d in rows])
        self.airfoils = [f for _, _, d in rows for f in d["airfoil"]]
        self.controls = [c for _, _, d in rows for c in d["control"]]
        n_s = len(self.chord)
        nc = np.array([d["p_a"].shape[1] for _, _, d in rows for _ in range(len(d["chord"]))])
        self.n_strips = n_s
        # panels
        self.pa = np.concatenate([d["p_a"].reshape(-1, 3) for _, _, d in rows])
        self.pb = np.concatenate([d["p_b"].reshape(-1, 3) for _, _, d in rows])
        self.cp = np.concatenate([d["cp"].reshape(-1, 3) for _, _, d in rows])
        self.normal = np.concatenate([d["n"].reshape(-1, 3) for _, _, d in rows])
        self.xc = np.concatenate([d["xc"].reshape(-1) for _, _, d in rows])
        self.strip = np.repeat(np.arange(n_s), nc)
        self.n_panels = len(self.pa)
        self.panel_surface = self.surface_index[self.strip]
        self.moving = self.xc > self.hinge[self.strip]  # behind a hinge line
        # strip-sum operator
        self.S = np.zeros((n_s, self.n_panels))
        self.S[self.strip, np.arange(self.n_panels)] = 1.0
        # thin-airfoil zero-lift angle of each strip's camber line, and the flap
        # effectiveness of its control (the linear VLM's own 2D behaviour)
        self.alpha0_thin = np.array([thin_airfoil(f)[0] for f in self.airfoils])
        self.tau = np.array([flap_tau(1 - h) if h < 1 else 0.0 for h in self.hinge])
        # channel gains: local deflection (TE towards -u positive) per unit channel
        # deflection, by the sign conventions JSBSim uses (see channel_gain)
        self.gain = {ch: np.array([channel_gain(ch, ctrl, self.u[k], self.c4[k]) for k, ctrl in enumerate(self.controls)])
                     for ch in ("aileron", "elevator", "rudder", "flap")}

    def summary(self):
        return {"strips": self.n_strips, "panels": self.n_panels}


def _root_reflection(aircraft, surf):
    """(root plane z, strength) for a fin whose root lies inside a body, else None."""
    if surf.mirror or surf.kind not in ("fin", "vtail"):
        return None
    root = surf.sections[0].le
    for body in aircraft.bodies:
        if not body.x[0] <= root[0] <= body.x[-1]:
            continue
        w, top, bot, yc, _ = body.section(root[0])
        if abs(root[1] - yc) <= 0.5 * w + 1e-6 and bot - 0.05 <= root[2] <= top + 0.05:
            return float(root[2]), float(getattr(surf, "root_reflection", 0.55))
    return None


def channel_gain(channel, ctrl, u, where):
    """Local deflection of a control per unit deflection of a channel.

    JSBSim's conventions: elevator and flaps positive trailing edge down;
    aileron positive = left aileron trailing edge down (right roll); rudder
    positive = trailing edge left. A local deflection is positive when the
    trailing edge moves towards -u."""
    if ctrl is None or ctrl.channel != channel:
        return 0.0
    g = ctrl.gain
    if channel in ("elevator", "flap"):
        return g * np.sign(u[2]) if abs(u[2]) > 1e-6 else 0.0
    if channel == "aileron":
        side = -1.0 if where[1] > 0 else 1.0  # left (y < 0) down for a positive command
        return g * side * np.sign(u[2]) if abs(u[2]) > 1e-6 else 0.0
    if channel == "rudder":
        return g * np.sign(u[1]) if abs(u[1]) > 1e-6 else 0.0
    return 0.0


class VLM:
    """The lattice solved for unit inputs at one Mach number."""

    def __init__(self, lattice, mach=0.0):
        L = self.lattice = lattice
        self.mach = mach
        beta = self.beta = np.sqrt(max(1.0 - mach * mach, 0.05))
        stretch = np.array([1.0 / beta, 1.0, 1.0])
        ref = L.ref
        pa = self._pa = (L.pa - ref) * stretch + ref
        pb = self._pb = (L.pb - ref) * stretch + ref
        cp = (L.cp - ref) * stretch + ref
        mid = 0.5 * (pa + pb)
        same_cp = L.panel_surface[:, None] == L.panel_surface[None, :]
        vcp = self._velocity(cp, same_cp)
        self.A = np.einsum("mnk,mk->mn", vcp, L.normal)
        del vcp
        self.Ainv = np.linalg.inv(self.A)
        # induced velocity at every bound-vortex midpoint (forces)
        self.W = self._velocity(mid, same_cp)
        self.bound = pb - pa               # stretched bound-vortex vectors
        self.mid = 0.5 * (L.pa + L.pb)     # physical application points
        self.r_mid = self.mid - ref
        self.r_cp = L.cp - ref
        N = L.normal
        # unit solutions: free stream (m = x, y, z) and rotation (omega components)
        self.g_v = self.Ainv @ (-N)                            # (n_p, 3)
        self.g_w = self.Ainv @ np.cross(self.r_cp, N)          # (n_p, 3)
        # rotating a panel's normal nose-up by d changes it by d (e x n)
        E = np.cross(L.e[L.strip], N)
        self.E = E
        # controls, per channel: the moving panels' normals rotated by the channel's gain
        self.g_ctrl = {}
        for ch, gain in L.gain.items():
            if not np.any(gain):
                continue
            D = E * (gain[L.strip] * L.moving)[:, None]
            self.g_ctrl[ch] = (self.Ainv @ (-D), self.Ainv @ np.cross(self.r_cp, D))
        # decambering: strip k's panels rotated nose-down by Delta_k
        n_s, n_p = L.n_strips, L.n_panels
        self.dec_v = np.empty((3, n_p, n_s))
        self.dec_w = np.empty((3, n_p, n_s))
        rxE = np.cross(self.r_cp, E)
        for m in range(3):
            Bv = np.zeros((n_p, n_s))
            Bw = np.zeros((n_p, n_s))
            Bv[np.arange(n_p), L.strip] = E[:, m]
            Bw[np.arange(n_p), L.strip] = -rxE[:, m]
            self.dec_v[m] = self.Ainv @ Bv
            self.dec_w[m] = self.Ainv @ Bw
        self.S_dec_v = np.einsum("sp,mpk->msk", L.S, self.dec_v)
        self.S_dec_w = np.einsum("sp,mpk->msk", L.S, self.dec_w)

    # -- a condition ----------------------------------------------------------------------------
    def circulation(self, v, w, deflections=None, decamber=None):
        """Panel circulations for air velocity v (3,), rotation w (3,, rad/s,
        design frame), channel deflections {channel: rad} and strip decambering
        angles (n_s,)."""
        g = self.g_v @ v + self.g_w @ w
        for ch, d in (deflections or {}).items():
            if d and ch in self.g_ctrl:
                gv, gw = self.g_ctrl[ch]
                g = g + d * (gv @ v + gw @ w)
        if decamber is not None:
            G = np.tensordot(v, self.dec_v, 1) + np.tensordot(w, self.dec_w, 1)
            g = g + G @ decamber
        return g

    def decamber_response(self, v, w):
        """d(strip circulation)/d(decambering), (n_s, n_s), for a condition."""
        return np.tensordot(v, self.S_dec_v, 1) + np.tensordot(w, self.S_dec_w, 1)

    def decamber_panels(self, v, w):
        return np.tensordot(v, self.dec_v, 1) + np.tensordot(w, self.dec_w, 1)

    def air_velocity(self, points, v, w):
        """Free stream plus rotation at points: v - omega x r."""
        return v[None, :] - np.cross(w[None, :], points - self.lattice.ref)

    def forces(self, gamma, v, w, rho=1.0):
        """Kutta-Joukowski force on every bound vortex (n_p, 3) and the total
        force and moment about the reference point."""
        vel = self.air_velocity(self.mid, v, w) + np.einsum("mnk,n->mk", self.W, gamma)
        f = rho * gamma[:, None] * np.cross(vel, self.bound)
        return f, f.sum(axis=0), np.cross(self.r_mid, f).sum(axis=0)

    def _velocity(self, pts, same):
        """Influence of every horseshoe (and the images of reflected fins) at
        stretched points."""
        L = self.lattice
        out = horseshoe_velocity(pts, self._pa, self._pb, same)
        for si, refl in L.reflect.items():
            if refl is None:
                continue
            z0, k = refl
            idx = np.flatnonzero(L.panel_surface == si)
            # the image of horseshoe a->b is b'->a': its legs turn the other way
            img = horseshoe_velocity(pts, reflect(self._pb[idx], z0), reflect(self._pa[idx], z0), same[:, idx])
            out[:, idx] += k * img
        return out

    def strip_influence(self):
        """Velocity at every strip's quarter-chord point per unit circulation of
        every horseshoe, (n_s, n_p, 3) - without the strip's own bound
        vortices, whose effect is the two-dimensional one its section polar
        already carries. What is left is the strip's induced (3D) flow."""
        L = self.lattice
        stretch = np.array([1.0 / self.beta, 1.0, 1.0])
        pts = (L.c4 - L.ref) * stretch + L.ref
        same = L.surface_index[:, None] == L.panel_surface[None, :]
        W = self._velocity(pts, same)
        for k in range(L.n_strips):
            idx = np.flatnonzero(L.strip == k)
            W[k, idx] -= _segments(pts[k : k + 1], self._pa[idx], self._pb[idx], np.full((1, len(idx)), 0.02**2))[0]
        return W

    def influence(self, points):
        """Velocity at points per unit circulation of every horseshoe,
        (M, n_p, 3) - for points off the lattice (bodies), wide cores."""
        L = self.lattice
        stretch = np.array([1.0 / self.beta, 1.0, 1.0])
        pts = (points - L.ref) * stretch + L.ref
        return self._velocity(pts, np.zeros((len(points), L.n_panels), bool))
