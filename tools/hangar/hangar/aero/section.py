"""Section (2D) aerodynamics over the whole circle of angles of attack.

A flight simulator's aircraft can end up at any attitude - an RL agent will
make sure of it - so every section needs cl, cd and cm from -180 to 180 deg,
smooth, and physically sensible everywhere. The polar is built as a blend of
three regimes, each from published theory or correlations:

- attached flow, forward: thin-airfoil theory for the zero-lift angle and
  moment (Glauert), a lift slope with thickness and viscous corrections, a
  maximum lift from the leading-edge sharpness and Reynolds number (DATCOM
  4.1.1.4 trends), soft-clipped so the curve rounds over at cl_max, and a
  profile-drag bucket from turbulent skin friction and a thickness form factor
  (Raymer 12.5);
- separated flow: a flat plate - normal force CD_max sin(alpha), centre of
  pressure moving from the quarter to the half chord (Viterna & Corrigan's
  CD_max with the parent surface's aspect ratio);
- attached flow, reversed (trailing edge first), around 180 deg.

Weights move smoothly (C1 smoothstep) between them, so stall is where the
attached regime hands over to the plate; its sharpness follows the stall type
(leading-edge stall for thin sections, trailing-edge stall for thick ones).

Trailing-edge flaps enter as thin-airfoil increments (Glauert's flap
effectiveness tau) reduced at large deflections (DATCOM's K' trend), with the
drag increment of Raymer 12.61 and a matching change of maximum lift.

Swept thin sections (fighters' wings, strakes, canards) do not stall that
way: past the attached-flow limit the flow leaves the leading edge and rolls
up into a vortex over the surface. Those sections have a vortex regime in
place of the stall (Polhamus' leading-edge suction analogy, NASA TN D-3767,
1966): the circulation keeps its potential value a0 sin(alpha); the suction
it would carry at the leading edge, cl sin(alpha), is lost as the edge
separates - and reappears as normal force, the vortex lift, in the
proportion the aerodynamic model passes in (it depends on the sweep, and on
whether the vortex has burst: model.py). Past about 45 deg the regime hands
over to the flat plate.
"""
import math

import numpy as np

# DATCOM-style basic maximum lift vs leading-edge sharpness (Delta y, % chord)
# at Re 9e6, smooth: a digitised trend (symmetric sections).
_CLMAX_DY = np.array([0.8, 1.2, 1.6, 2.0, 2.5, 3.0, 3.5, 4.0, 6.0])
_CLMAX_V = np.array([0.72, 0.80, 0.95, 1.15, 1.36, 1.55, 1.65, 1.70, 1.70])


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def thin_airfoil(foil, n=400):
    """Glauert: zero-lift angle (rad) and moment about the quarter chord of a
    camber line."""
    th = np.linspace(0.0, np.pi, n + 1)
    thm = 0.5 * (th[1:] + th[:-1])
    dth = np.diff(th)
    x = 0.5 * (1.0 - np.cos(thm))
    s = foil.camber_slope(x)
    a_l0 = -np.sum(s * (np.cos(thm) - 1.0) * dth) / np.pi
    a1 = 2.0 / np.pi * np.sum(s * np.cos(thm) * dth)
    a2 = 2.0 / np.pi * np.sum(s * np.cos(2 * thm) * dth)
    return float(a_l0), float(np.pi / 4.0 * (a2 - a1))


def flap_tau(cf):
    """Glauert's flap effectiveness d(alpha_L0)/d(delta) for a flap of chord
    fraction cf (thin-airfoil theory)."""
    th = math.acos(2.0 * cf - 1.0)
    return 1.0 - (th - math.sin(th)) / math.pi


def flap_moment_slope(cf):
    """Thin-airfoil d(cm_c/4)/d(delta) of a flap (per rad)."""
    th = math.acos(2.0 * cf - 1.0)
    return 0.5 * math.sin(th) * (math.cos(th) - 1.0)


def flap_efficiency(delta_deg, kind="plain"):
    """Fraction of the thin-airfoil flap effect realised at a deflection
    (DATCOM's K' trend for plain flaps; slotted and Fowler flaps keep the flow
    attached to larger angles)."""
    d = np.abs(delta_deg)
    if kind == "plain":
        return 0.40 + 0.60 / (1.0 + (d / 22.0) ** 2.5)
    return 0.55 + 0.45 / (1.0 + (d / 32.0) ** 2.5)  # slotted, fowler


def skin_friction(re, mach=0.0, laminar=0.0):
    """Flat-plate skin friction: turbulent (Schlichting, with the Raymer
    compressibility term) blended with laminar (Blasius) by laminar fraction."""
    re = max(re, 1e4)
    turb = 0.455 / (math.log10(re) ** 2.58 * (1.0 + 0.144 * mach * mach) ** 0.65)
    lam = 1.328 / math.sqrt(re)
    return laminar * lam + (1.0 - laminar) * turb


class SectionPolar:
    """cl, cd, cm (about the quarter chord) of one section at one Reynolds
    and Mach number, for any angle of attack and flap deflection."""

    def __init__(self, foil, re=5e6, mach=0.0, aspect_ratio=8.0, laminar=0.0, overrides=None,
                 flap_chord=None, flap_kind="plain", vortex=False):
        o = overrides or {}
        self.foil = foil
        self.vortex = 0.0                      # set below: the clip levels are the attached polar's
        # the suction a leading edge can hold, as a fraction of what it carries at
        # the attached-flow limit: none for a sharp edge, all for a round one
        self.suction_k = float(np.clip((foil.leading_edge_sharpness - 0.4) / 0.8, 0.0, 1.0))
        t = foil.thickness_ratio
        self.re = re
        self.alpha0, self.cm0_thin = thin_airfoil(foil)
        self.alpha0 = math.radians(o["alpha0_deg"]) if "alpha0_deg" in o else self.alpha0
        # lift slope: thin theory x thickness (1 + 0.77 t/c) x a viscous factor that
        # falls with Reynolds number; compressibility is left to the 3D model
        visc = float(np.clip(0.905 + 0.025 * math.log10(re / 6e6), 0.80, 0.95))
        self.a0 = o.get("cl_alpha", 2.0 * math.pi * (1.0 + 0.77 * t) * visc)
        self.cm0 = o.get("cm0", 0.9 * self.cm0_thin)
        # maximum lift: leading-edge sharpness trend, Reynolds and camber
        dy = foil.leading_edge_sharpness
        base = float(np.interp(dy, _CLMAX_DY, _CLMAX_V)) * (min(re, 2e7) / 9e6) ** 0.08
        base -= 1.5 * max(t - 0.15, 0.0)
        camber_gain = -self.a0 * self.alpha0  # lift at zero angle from camber
        # (the sharpness is read off the upper surface, so it already carries
        # most of the camber's effect)
        self.clmax = o.get("clmax", base + 0.25 * camber_gain)
        self.clmin = o.get("clmin", -(base - 0.45 * camber_gain) * 0.95)
        # stall sharpness from the stall type
        # (and how much lift survives it: Viterna's A2 term, scaled)
        if dy < 1.9 or t < 0.09:
            self.stall_width, self.retention = math.radians(2.0), 0.45  # leading-edge (thin-airfoil) stall
        elif t <= 0.13:
            self.stall_width, self.retention = math.radians(3.5), 0.75  # mixed
        else:
            self.stall_width, self.retention = math.radians(6.0), 1.0   # trailing-edge stall: gentle
        self.retention = o.get("post_stall_retention", self.retention)
        self.stall_width = math.radians(o["stall_width_deg"]) if "stall_width_deg" in o else self.stall_width
        # profile drag: two surfaces of turbulent friction x thickness form factor
        ff = 1.0 + 0.6 / max(foil.x_max_thickness, 0.2) * t + 100.0 * t**4
        self.cd0 = o.get("cd0", 2.0 * skin_friction(re, mach, laminar) * ff * 1.03)
        self.cl_dmin = o.get("cl_cdmin", 0.7 * camber_gain)
        self.k_drag = o.get("k_drag", 0.010 * (6e6 / max(re, 1e5)) ** 0.2)
        # post-stall plate (Viterna & Corrigan): CD_max from the aspect ratio
        self.cd_max = o.get("cd_max", 1.11 + 0.018 * min(aspect_ratio, 50.0))
        # reversed flow: a sharp leading edge, early stall
        self.a0_rev = 0.8 * self.a0
        self.clmax_rev = 0.55
        self.flap_chord = flap_chord or 0.0
        self.flap_kind = flap_kind
        # flap parameters as numbers, so a PolarSet can stack them
        self.tau = flap_tau(flap_chord) if flap_chord else 0.0
        self.cm_flap = flap_moment_slope(flap_chord) if flap_chord else 0.0
        self.eff_a, self.eff_b, self.eff_c = (0.40, 0.60, 22.0) if flap_kind == "plain" else (0.55, 0.45, 32.0)
        self.ext_k = 0.5 if flap_kind == "fowler" else 0.0
        self.cd_flap_k = 1.0 if flap_kind == "plain" else 0.6
        # the soft clip and the hand-over to the plate round the peak off below
        # the clip level: raise the internal levels until the curve's own
        # extremes are cl_max and cl_min
        self._clmax_int, self._clmin_int = self.clmax, self.clmin
        reach = 3.0 * self.stall_width + math.radians(1.0)  # the attached peak, not the plate's hump at 45 deg
        hi = np.linspace(self.alpha0, self.alpha0 + self.clmax / self.a0 + reach, 800)
        lo = np.linspace(self.alpha0 + self.clmin / self.a0 - reach, self.alpha0, 800)
        for _ in range(4):
            self._clmax_int += self.clmax - self.evaluate(hi)[0].max()
            self._clmin_int += self.clmin - self.evaluate(lo)[0].min()
        self.vortex = 1.0 if vortex else 0.0   # the vortex regime replaces the stall

    def evaluate(self, alpha, delta=None, vortex=None):
        """cl, cd, cm at angles of attack alpha (rad, any range) and flap
        deflection delta (rad, TE down positive; scalar or matching alpha).
        vortex: (gain, degradation) of the vortex regime (see evaluate)."""
        return evaluate(self, alpha, delta, vortex)[:3]

    def cl_slope(self, alpha, delta=None, h=1e-4):
        return (self.evaluate(alpha + h, delta)[0] - self.evaluate(alpha - h, delta)[0]) / (2 * h)

    def summary(self):
        a_stall = self.alpha0 + self.clmax / self.a0
        return {"alpha0_deg": math.degrees(self.alpha0), "cl_alpha_per_rad": self.a0, "cm0": self.cm0,
                "clmax": self.clmax, "clmin": self.clmin, "alpha_clmax_deg": math.degrees(a_stall),
                "cd0": self.cd0, "cd_max": self.cd_max, "re": self.re}


# every parameter evaluate() reads: a PolarSet stacks them into arrays
_PARAMS = ("alpha0", "a0", "cm0", "_clmax_int", "_clmin_int", "stall_width", "retention", "cd0", "cl_dmin", "k_drag", "cd_max",
           "a0_rev", "clmax_rev", "flap_chord", "tau", "cm_flap", "eff_a", "eff_b", "eff_c", "ext_k", "cd_flap_k", "vortex",
           "suction_k")


class PolarSet:
    """Many sections evaluated together: parameters stacked into arrays, so
    one call gives every strip's coefficients (alpha and delta shaped
    (..., n))."""

    def __init__(self, polars):
        self.polars = list(polars)
        for name in _PARAMS:
            setattr(self, name, np.array([getattr(p, name) for p in self.polars], float))

    def __len__(self):
        return len(self.polars)

    def evaluate(self, alpha, delta=None, vortex=None, alpha_suction=None):
        return evaluate(self, alpha, delta, vortex, alpha_suction)


# the vortex regime: where the vortex force acts (chord fraction), and the
# leading-edge angles over which it hands over to the flat plate
VORTEX_X = 0.10
VORTEX_END = (math.radians(40.0), math.radians(55.0))


def evaluate(p, alpha, delta=None, vortex=None, alpha_suction=None):
    """The polar of a SectionPolar or PolarSet `p` (attributes scalar or
    arrays broadcasting against alpha): cl, cd, cm and the circulation's
    lift - cl without vortex lift, what washes the flow down behind.

    vortex = (gain, degradation), broadcasting against alpha, for sections
    with the vortex regime: the vortex normal force is gain times the lost
    leading-edge suction (Polhamus: 1/cos of the leading-edge sweep, times
    the part realised, times what is left after vortex breakdown), and the
    circulation is scaled by the degradation once the edge has separated
    (the burst vortex's separated flow). None: no vortex lift.

    alpha_suction: the angle the leading-edge suction sees, where it differs
    from alpha's (a wing's: its angle less the downwash of the trailing
    vortices - the local cl times it is the 3D suction, CL (alpha - alpha_i),
    which a low-aspect-ratio wing's strip angle, set by its lift, understates)."""
    a = np.asarray(alpha, float)
    d = np.zeros_like(a) if delta is None else np.broadcast_to(np.asarray(delta, float), a.shape)
    # an all-moving surface (flap chord 1) turns the whole section: its
    # deflection is angle of attack, with no flap increments
    whole = p.flap_chord >= 0.999
    a_s = None if alpha_suction is None else wrap(np.asarray(alpha_suction, float) + np.where(whole, d, 0.0))
    a = wrap(a + np.where(whole, d, 0.0))
    d = np.where(whole, 0.0, d)
    # flap increments (all zero where there is no flap: tau = cm_flap = flap_chord = 0)
    ddeg = np.abs(np.degrees(d))
    eff = p.eff_a + p.eff_b / (1.0 + (ddeg / p.eff_c) ** 2.5)
    ext = 1.0 + p.ext_k * p.flap_chord * np.clip(ddeg / 40.0, 0.0, 1.0)
    dal0 = -p.tau * eff * d * ext
    dcl0 = -p.a0 * dal0
    dclmax = 0.62 * dcl0
    dcd = 0.9 * p.flap_chord**1.38 * np.sin(d) ** 2 * p.cd_flap_k
    dcm = p.cm_flap * eff * d * ext
    a_l0 = p.alpha0 + dal0
    clmax = p._clmax_int + dclmax
    clmin = p._clmin_int + 0.3 * dcl0
    # forward attached: linear, soft-clipped to [clmin, clmax]
    lin = p.a0 * (a - a_l0)
    k = 12.0
    cl_f = -np.logaddexp(-k * lin, -k * clmax) / k           # smooth min(lin, clmax)
    cl_f = np.logaddexp(k * cl_f, k * clmin) / k              # smooth max(., clmin)
    cd_f = p.cd0 + p.k_drag * (cl_f - p.cl_dmin - 0.5 * dcl0) ** 2 + dcd
    cm_f = p.cm0 + dcm + np.zeros_like(a)
    # separated: Viterna & Corrigan - a flat plate (A1 = CD_max / 2) plus the
    # A2 term that starts from the stall point and dies away by 90 deg, so the
    # lift left after stall depends on how the section stalls
    sa, ca = np.sin(a), np.cos(a)
    a1 = 0.5 * p.cd_max
    fwd = ca > 0.0
    s_hi = np.clip(a_l0 + clmax / p.a0, 0.05, 1.2)            # stall angles, positive and negative side
    s_lo = np.clip(-(a_l0 + clmin / p.a0), 0.05, 1.2)
    a2_hi = (clmax - a1 * np.sin(2 * s_hi)) * np.sin(s_hi) / np.cos(s_hi) ** 2
    a2_lo = (-clmin - a1 * np.sin(2 * s_lo)) * np.sin(s_lo) / np.cos(s_lo) ** 2
    a2 = p.retention * np.where(sa >= 0.0, np.maximum(a2_hi, 0.0), -np.maximum(a2_lo, 0.0))
    visc = np.where(fwd, a2 * ca * ca / np.where(np.abs(sa) > 0.05, np.abs(sa), 0.05), 0.0)
    cl_p = a1 * np.sin(2 * a) + visc + 0.5 * dcl0 * ca * ca   # flaps keep a little of their lift
    cd_p = p.cd_max * sa * sa + p.cd0 * ca * ca + dcd
    cn = cl_p * ca + cd_p * sa
    xcp = 0.5 - 0.25 * ca * np.abs(ca)
    cm_p = -cn * (xcp - 0.25) + 0.5 * dcm * ca * ca          # ...and of their moment
    # reversed attached flow around 180 deg (the trailing edge leads)
    ar = wrap(a - np.pi)
    lin_r = p.a0_rev * ar
    cl_r = -np.logaddexp(-k * lin_r, -k * p.clmax_rev) / k
    cl_r = np.logaddexp(k * cl_r, -k * p.clmax_rev) / k
    cd_r = 1.5 * p.cd0 + 2 * p.k_drag * cl_r**2 + dcd
    cm_r = -(cl_r * ca + cd_r * sa) * 0.5                    # its normal force acts at the three-quarter chord
    # weights: attached-forward inside the stall angles, reversed near 180 deg
    w = p.stall_width
    a_hi = a_l0 + clmax / p.a0 + 0.5 * w
    a_lo = a_l0 + clmin / p.a0 - 0.5 * w
    w_f = smoothstep((a_hi + w - a) / (2 * w)) * smoothstep((a - (a_lo - w)) / (2 * w))
    rw = math.radians(4.0)
    a_rs = p.clmax_rev / p.a0_rev + rw
    w_r = smoothstep((a_rs + rw - np.abs(ar)) / (2 * rw))
    w_p = np.clip(1.0 - w_f - w_r, 0.0, 1.0)
    cl = w_f * cl_f + w_r * cl_r + w_p * cl_p
    cd = w_f * cd_f + w_r * cd_r + w_p * cd_p
    cm = w_f * cm_f + w_r * cm_r + w_p * cm_p
    if not np.any(p.vortex):
        return cl, cd, cm, cl
    # the vortex regime: the potential circulation (degraded behind a burst
    # vortex); the leading edge holds the suction it can - at most what it
    # carries at the attached-flow limit (Carlson's attainable thrust, NASA
    # TP-1500) - and the rest is lost, part of it to the vortex as normal force
    g, dg = (0.0, 1.0) if vortex is None else vortex
    an = a - a_l0
    lim = np.where(an >= 0.0, clmax, -clmin)
    held = p.suction_k * lim * lim / p.a0                 # the attainable suction
    beyond = smoothstep((np.abs(an) - lim / p.a0 + w) / (2 * w))
    circ = p.a0 * np.sin(an) * (1.0 - beyond * (1.0 - dg))
    suction = circ * sa                                  # the suction the circulation would carry
    T = held * np.tanh(suction / np.maximum(held, 1e-6))
    # what the edge loses goes to the vortex as the planform loses it
    # (Polhamus); the section's own balance keeps its own suction
    s3 = suction if a_s is None else circ * np.sin(a_s)
    T3 = T if a_s is None else held * np.tanh(s3 / np.maximum(held, 1e-6))
    n_v = g * np.abs(s3 - T3) * np.sign(circ)            # the vortex lift, normal to the chord
    N = circ * ca + n_v
    cl_v = N * ca + T * sa
    cd_v = N * sa - T * ca + p.cd0 + p.k_drag * (np.clip(circ, clmin, clmax) - p.cl_dmin) ** 2 + dcd
    cm_v = p.cm0 + dcm + n_v * (0.25 - VORTEX_X)
    lo, hi = VORTEX_END
    w_v = np.where(ca > 0.0, smoothstep((hi - np.abs(a)) / (hi - lo)), 0.0) * (1.0 - w_r)
    rest = 1.0 - w_v
    w_pr = np.clip(1.0 - w_r, 0.0, 1.0)
    cl_c = w_r * cl_r + w_pr * cl_p
    cd_c = w_r * cd_r + w_pr * cd_p
    cm_c = w_r * cm_r + w_pr * cm_p
    on = p.vortex > 0.0
    cl2 = np.where(on, w_v * cl_v + rest * cl_c, cl)
    cd2 = np.where(on, w_v * cd_v + rest * cd_c, cd)
    cm2 = np.where(on, w_v * cm_v + rest * cm_c, cm)
    circ2 = np.where(on, w_v * circ + rest * cl_c, cl)
    return cl2, cd2, cm2, circ2


def linear_part(p, delta=None):
    """The attached-flow line of a polar (or PolarSet) at a flap deflection:
    lift slope and zero-lift angle, cl = a0 (alpha - alpha_L0)."""
    d = np.zeros_like(np.asarray(p.a0, float)) if delta is None else np.asarray(delta, float)
    ddeg = np.abs(np.degrees(d))
    eff = p.eff_a + p.eff_b / (1.0 + (ddeg / p.eff_c) ** 2.5)
    ext = 1.0 + p.ext_k * p.flap_chord * np.clip(ddeg / 40.0, 0.0, 1.0)
    whole = np.asarray(p.flap_chord) >= 0.999
    return p.a0, p.alpha0 - np.where(whole, d, p.tau * eff * d * ext)
