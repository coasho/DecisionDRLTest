"""Compressibility: how the coefficients change with Mach number.

The tables are built for incompressible flow. A fighter flies from Mach 0.2
to 2, and its aerodynamics change a lot in between. This module works out
factors and increments on the low-speed tables, per Mach number:

- subsonic (to Mach 0.9): the model itself, its lattice under the
  Prandtl-Glauert transformation - lift slopes grow roughly as
  1/sqrt(1 - M^2) (less on a low aspect ratio), the neutral point moves;
- supersonic (from Mach 1.2): linear theory per lifting surface - Ackeret's
  4/sqrt(M^2 - 1) with the loss in the tips' Mach cones where the leading
  edge is supersonic, Stewart's delta-wing slope 2 pi cot(sweep)/E(k) where
  it is subsonic - each surface's lift acting at its area centroid (Ackeret:
  mid-chord), a trailing-edge flap as effective as its chord fraction (not
  Glauert's more); taken relative to the same estimate at Mach 0.9 - each
  surface's lift where the model's lattice puts it there - and applied to
  the model's own values there, so the two ends meet;
- transonic: faired between them;
- drag: skin friction falling with Mach (Raymer 12.27), the wave drag of the
  area distribution - Sears-Haack's D/q = 9 pi A_max^2 / (2 l^2) times
  Raymer's empirical E_WD (Raymer 12.46), from Korn's drag-divergence Mach
  number (Raymer 12.5.10) - and the loss of leading-edge suction once the
  leading edge is supersonic (induced drag CL^2 / CL_alpha).

The result, over Mach: K_L multiplies lift (and the damping), K_Y the
lateral coefficients, K_<channel> each channel's control power; dCm_dCL is
the pitching moment per unit of lift from the neutral point's move; dCD0 the
zero-lift drag's change; dK the change of the induced-drag factor.
"""
import math

import numpy as np

from .section import flap_tau

MACH_SUB = (0.0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9)
MACH_SUP = (1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6)
KORN_KAPPA = 0.87   # conventional (not supercritical) sections
E_WD = 2.0          # Raymer: 1.2 for a smooth Sears-Haack-like area distribution, 2-3 for poor ones


def ellipe(k):
    """Complete elliptic integral of the second kind E(k) (modulus k)."""
    x, w = np.polynomial.legendre.leggauss(32)
    th = 0.25 * math.pi * (x + 1.0)
    return float(0.25 * math.pi * np.sum(w * np.sqrt(1.0 - (k * np.sin(th)) ** 2)))


def _surface_numbers(surf):
    """Aspect ratio (a fin's doubled by its reflection in the body, as
    DATCOM's end-plate effect roughly does), leading-edge and half-chord
    sweep, quarter-chord MAC point and area centroid (x) of a surface."""
    a, b = surf.sections[0], surf.sections[-1]
    ar = surf.aspect_ratio * (1.55 if surf.kind in ("fin", "vtail") and not surf.mirror else 1.0)
    le = math.radians(surf.sweep_deg(0.0))
    half = math.radians(surf.sweep_deg(0.5))
    mac, mac_le = surf.mac
    area = surf.area / (2 if surf.mirror else 1)
    xc = surf._integrate(lambda e: (surf.station(e)[0][0] + 0.5 * surf.station(e)[1]) * surf.station(e)[1]) / area
    del a, b
    return {"ar": ar, "sweep_le": le, "sweep_half": half, "x_quarter": float(mac_le[0] + 0.25 * mac),
            "x_centroid": float(xc), "area": surf.area}


def lift_slope(num, mach):
    """Lift slope (per rad, on the surface's own area) by linear theory:
    DATCOM's subsonic formula below Mach 1, Ackeret/Stewart above."""
    ar, le, half = num["ar"], num["sweep_le"], num["sweep_half"]
    if mach < 1.0:
        b = math.sqrt(1.0 - mach * mach)
        return 2 * math.pi * ar / (2 + math.sqrt(4 + (ar * b / 0.95) ** 2 * (1 + math.tan(half) ** 2 / (b * b))))
    b = math.sqrt(mach * mach - 1.0)
    m = b / max(math.tan(le), 1e-6)         # beta cot(sweep): the leading edge supersonic at m >= 1
    if m >= 1.0:
        return 4.0 / b * max(1.0 - 1.0 / (2.0 * ar * b), 0.5)
    # a subsonic leading edge: Stewart's delta of the same sweep, scaled to the
    # surface's aspect ratio (that delta's is 4 cot(sweep))
    return 2 * math.pi / math.tan(le) / ellipe(math.sqrt(1.0 - m * m)) * min(1.0, ar * math.tan(le) / 4.0)


def _combine(nums, mach, downwash):
    """The aircraft's lift slope (per rad, on the reference area) and its
    neutral point's x from its horizontal surfaces by linear theory: each
    surface's lift at its area centroid supersonic, and subsonic where the
    model's lattice puts it (x_lattice; else the quarter chord of its MAC)."""
    cla = xs = 0.0
    for n, eps in zip(nums, downwash):
        a = lift_slope(n, mach) * n["share"] * (1.0 - eps)
        x = n.get("x_lattice", n["x_quarter"]) if mach < 1.0 else n["x_centroid"]
        cla += a
        xs += a * x
    return cla, xs / cla


def lattice_centres(model, alpha_deg=2.0):
    """Where each lifting surface's lift from angle of attack acts in a
    model's lattice, x in the design frame: the centroid of its panels'
    loading in an upward flow. At Mach 0.9 it is well behind a low aspect
    ratio surface's quarter chord already (its Prandtl-Glauert image is more
    slender), so the supersonic move is taken from there."""
    L, V = model.lat, model.vlm
    V.select(math.radians(alpha_deg))
    f = V.g_v[:, 2] * np.hypot(*(L.pb - L.pa)[:, 1:].T)
    x = 0.5 * (L.pa[:, 0] + L.pb[:, 0])
    out = {}
    for si, s in enumerate(L.surfaces):
        k = L.panel_surface == si
        if abs(f[k].sum()) > 1e-9:
            out[id(s)] = float((f[k] * x[k]).sum() / f[k].sum())
    return out


def _fair(xs, ys, x):
    """Monotone cubic (Fritsch-Carlson) through the points, at x."""
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    d = np.diff(ys) / np.diff(xs)
    m = np.concatenate([[d[0]], 0.5 * (d[1:] + d[:-1]), [d[-1]]])
    for i in range(len(d)):
        if d[i] == 0.0:
            m[i] = m[i + 1] = 0.0
        else:
            a, b = m[i] / d[i], m[i + 1] / d[i]
            s = a * a + b * b
            if s > 9.0:
                t = 3.0 / math.sqrt(s)
                m[i], m[i + 1] = t * a * d[i], t * b * d[i]
    x = np.clip(np.asarray(x, float), xs[0], xs[-1])
    i = np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2)
    h = xs[i + 1] - xs[i]
    t = (x - xs[i]) / h
    return ((2 * t**3 - 3 * t**2 + 1) * ys[i] + (t**3 - 2 * t**2 + t) * h * m[i]
            + (-2 * t**3 + 3 * t**2) * ys[i + 1] + (t**3 - t**2) * h * m[i + 1])


def linear_numbers(model, alpha_deg=2.0):
    """The derivatives the Mach factors follow, from a model at its Mach
    number: CL_alpha, Cm_alpha, the sideslip derivatives and each channel's
    control power (per rad)."""
    a, h, hb = math.radians(alpha_deg), math.radians(1.0), math.radians(2.0)
    up, dn = model.evaluate(a + h), model.evaluate(a - h)
    out = {"CLa": (up["CL"] - dn["CL"]) / (2 * h), "Cma": (up["Cm"] - dn["Cm"]) / (2 * h)}
    bp, bm = model.evaluate(a, hb), model.evaluate(a, -hb)
    for k in ("CY", "Cl", "Cn"):
        out[k + "b"] = (bp[k] - bm[k]) / (2 * hb)
    main = {"elevator": "Cm", "aileron": "Cl", "rudder": "Cn", "flap": "CL"}
    for ch in model.aircraft.channels():
        cp = model.evaluate(a, controls={ch: hb})
        cm = model.evaluate(a, controls={ch: -hb})
        out["ctl_" + ch] = (cp[main[ch]] - cm[main[ch]]) / (2 * hb)
    return out


def mach_effects(aircraft, build_model, base, progress=None):
    """The Mach factors and increments (see the module's notes), over the
    Mach numbers the aircraft reaches. build_model(mach) makes an AeroModel
    at a Mach number; base: the low-speed tables (for the zero-lift and
    induced drag)."""
    a = aircraft
    top = float(a.spec.get("analysis", {}).get("max_mach", 0.9 if not any(e.type == "turbofan" for e in a.engines) else 2.6))
    sub = [m for m in MACH_SUB if m <= max(top, 0.3) + 1e-9]
    # subsonic: the model at each Mach number
    lin = {}
    centres = {}
    for m in sub:
        model = build_model(m)
        lin[m] = linear_numbers(model)
        if m == 0.9:
            centres = lattice_centres(model)
        if progress:
            progress("mach %.2f" % m, len(lin), len(sub))
    ref = lin[0.0]
    x_ref, c = float(a.aero_point[0]), a.c
    x_np = {m: x_ref - v["Cma"] / v["CLa"] * c for m, v in lin.items()}
    channels = a.channels()
    rows = {m: {"K_L": v["CLa"] / ref["CLa"], "K_Y": v["CYb"] / ref["CYb"] if abs(ref["CYb"]) > 1e-6 else 1.0,
                "x_np": x_np[m], **{"K_" + ch: v["ctl_" + ch] / ref["ctl_" + ch] if abs(ref["ctl_" + ch]) > 1e-9 else 1.0
                                   for ch in channels}}
            for m, v in lin.items()}
    mach = list(sub)
    # supersonic: linear theory, relative to the same estimate at Mach 0.9
    if top > 1.0 and 0.9 in rows:
        horiz = [s for s in a.surfaces if s.kind not in ("fin", "vtail")]
        nums = [dict(_surface_numbers(s), share=s.area / a.S) for s in horiz]
        for n, s in zip(nums, horiz):
            if id(s) in centres:
                n["x_lattice"] = centres[id(s)]
        eps0 = [_tail_downwash(a, s) for s in horiz]
        wing = next(i for i, s in enumerate(horiz) if s is a.wing)
        fins = [_surface_numbers(s) for s in a.surfaces if s.kind in ("fin", "vtail")]
        cla9, xnp9 = _combine(nums, 0.9, eps0)
        w9 = lift_slope(nums[wing], 0.9)
        fin9 = lift_slope(fins[0], 0.9) if fins else 1.0
        for m in (x for x in MACH_SUP if x <= top + 1e-9):
            # the downwash behind the wing falls with the wing's lift slope
            eps = [e * lift_slope(nums[wing], m) / w9 for e in eps0]
            cla, xnp = _combine(nums, m, eps)
            row = {"K_L": rows[0.9]["K_L"] * cla / cla9, "x_np": rows[0.9]["x_np"] + (xnp - xnp9),
                   "K_Y": rows[0.9]["K_Y"] * (lift_slope(fins[0], m) / fin9 if fins else 1.0)}
            for ch in channels:
                surfs = [(s, cc) for s, cc in a.controls() if ch in cc.channels]
                s, cc = surfs[0]
                n = _surface_numbers(s)
                k = lift_slope(n, m) / lift_slope(n, 0.9)
                if not cc.all_moving:
                    cf = 0.5 * (cc.cf0 + cc.cf1)
                    k *= cf / flap_tau(cf)     # Ackeret: a flap's lift is its chord fraction of the whole
                row["K_" + ch] = rows[0.9]["K_" + ch] * k
            rows[m] = row
            mach.append(m)
        # transonic: fair through, with the values held flat at 0.9 and 1.2
        for m in (0.95, 1.0, 1.05, 1.1):
            rows[m] = None
        mach = sorted(set(mach) | {0.95, 1.0, 1.05, 1.1})
    known = [m for m in mach if rows[m] is not None]
    keys = [k for k in rows[0.0]]
    table = {"mach": np.array(mach)}
    for k in keys:
        table[k] = np.array([_fair(known, [rows[m][k] for m in known], m) for m in mach])
    # pitching moment per unit lift from the neutral point's move: at Mach M,
    # Cm = Cm0 + CL (x_ref - x_np(M))/c with CL = K_L CL_base, where the base
    # has CL_base (x_ref - x_np(0))/c: the increment per unit CL_base
    table["dCm_dCL"] = (table["K_L"] * (x_ref - table["x_np"]) - (x_ref - x_np[0.0])) / c
    table.update(_drag(a, base, table))
    return table


def _tail_downwash(aircraft, surf):
    """The downwash gradient at a surface from the wing ahead of it (DATCOM's
    2 CL_alpha,w / (pi A) at the tail, halved with the distance): 0 for the
    wing itself, a strake or a canard."""
    w = aircraft.wing
    if surf is w or surf.kind in ("strake", "canard"):
        return 0.0
    num = _surface_numbers(w)
    cla = lift_slope(num, 0.0)
    arm = _surface_numbers(surf)["x_quarter"] - num["x_quarter"]
    return float(2.0 * cla / (math.pi * w.aspect_ratio) * (1.0 - 0.25 * min(arm / (0.5 * w.span), 1.0)))


def _drag(a, base, table):
    """dCD0 (friction and wave drag) and dK (the induced-drag factor's
    change) over Mach."""
    mach = table["mach"]
    al = base["alpha"]
    j0 = int(np.argmin(np.abs(base["beta"])))
    k = (al >= -2.0) & (al <= 8.0)
    CL, CD = base["base"]["CL"][k, j0], base["base"]["CD"][k, j0]
    K0, CD0 = np.polyfit(CL ** 2, CD, 1)
    # friction falls with Mach (most of the zero-lift drag is friction)
    friction = 0.85 * CD0 * ((1.0 + 0.144 * mach ** 2) ** -0.65 - 1.0)
    # wave drag: Sears-Haack from the area distribution, Raymer's E_WD
    area, length = area_distribution(a)
    amax = float(area.max())
    ewd = float(getattr(a, "calibration", {}).get("wave_drag_efficiency",
                                                     a.spec.get("analysis", {}).get("wave_drag_efficiency", E_WD)))
    le = a.wing.sweep_deg(0.0)
    sh = 4.5 * math.pi * (amax / length) ** 2 / a.S
    w = a.wing
    t = w.thickness_ratio
    lam = math.radians(w.sweep_deg(0.25))
    m_dd = KORN_KAPPA / math.cos(lam) - t / math.cos(lam) ** 2 - 0.2 / (10 * math.cos(lam) ** 3)
    m_cr = m_dd - (0.1 / 80.0) ** (1.0 / 3.0)

    def wave(m):
        if m >= 1.2:
            return ewd * (1.0 - 0.2 * (m - 1.2) ** 0.57 * (1.0 - math.pi * le ** 0.77 / 100.0)) * sh
        peak = ewd * sh
        if m <= m_cr:
            return 0.0
        pts_m = [m_cr, m_dd, 1.05, 1.2]
        pts_v = [0.0, 0.002, peak, peak]
        return float(_fair(pts_m, pts_v, m))
    dcd0 = friction + np.array([wave(float(m)) for m in mach])
    # induced drag: the base carries K0 CL_base^2; at Mach M, with the lift K_L
    # times the base's, the suction the leading edge keeps goes as it turns
    # supersonic (M cos(sweep) = 1): K(M) from K0 to 1/CL_alpha(M)
    le_r = math.radians(le)
    cla = table["K_L"] * base_cla(base)
    lost = np.clip((mach - 1.0) / max(1.0 / math.cos(le_r) - 1.0, 0.05), 0.0, 1.0)
    lost = lost * lost * (3.0 - 2.0 * lost)
    K = (1.0 - lost) * K0 + lost / cla
    dK = K - K0 / table["K_L"] ** 2
    return {"dCD0": dcd0, "dK": dK, "K0": float(K0), "CD0": float(CD0), "M_dd": float(m_dd), "M_cr": float(m_cr),
            "A_max_m2": amax, "length_m": float(length), "E_WD": ewd}


def base_cla(base):
    al = base["alpha"]
    j0 = int(np.argmin(np.abs(base["beta"])))
    CL = base["base"]["CL"][:, j0]
    return float((np.interp(3.0, al, CL) - np.interp(-1.0, al, CL)) / math.radians(4.0))


def bodies_area(bodies, xs, cells=192):
    """The area (m^2) of the union of the bodies' sections at each x: where
    an intake stands against the fuselage or a boom on a nacelle, the
    overlap counts once. Each section is laid on a y-z grid."""
    area = np.zeros(len(xs))
    parts = [(b, side) for b in bodies for side in b.copies()]
    for i, x in enumerate(xs):
        secs = []
        for b, side in parts:
            if not b.x[0] <= x <= b.x[-1]:
                continue
            w, top, bot, yc, _ = b.section(x)
            zc, nt, nb = b.halves(x)
            if w <= 0.0 or top <= bot:
                continue
            secs.append((float(yc) * side, 0.5 * float(w), float(zc), float(top), float(bot), float(nt), float(nb)))
        if not secs:
            continue
        y0 = min(s[0] - s[1] for s in secs)
        y1 = max(s[0] + s[1] for s in secs)
        z0 = min(s[4] for s in secs)
        z1 = max(s[3] for s in secs)
        h = max(y1 - y0, z1 - z0) / cells
        ys = np.arange(y0 + 0.5 * h, y1, h)
        zs = np.arange(z0 + 0.5 * h, z1, h)
        Y, Z = np.meshgrid(ys, zs)
        covered = np.zeros(Y.shape, bool)
        for yc, a, zc, top, bot, nt, nb in secs:
            up = Z >= zc
            b_ = np.where(up, max(top - zc, 1e-9), max(zc - bot, 1e-9))
            n_ = np.where(up, nt, nb)
            covered |= np.abs((Y - yc) / a) ** n_ + np.abs((Z - zc) / b_) ** n_ <= 1.0
        area[i] = covered.sum() * h * h
    return area


def area_distribution(aircraft, n=160):
    """Cross-sectional area (m^2) of the whole aircraft over x, and its
    length (m): the bodies' sections, plus the surfaces' thickness where they
    stand outside the fuselage, less the air the engines swallow (their
    capture area, from the inlet to the nozzle: Raymer 12.5.10)."""
    lo = min([b.x[0] for b in aircraft.bodies] + [min(sec.le[0] for sec in s.sections) for s in aircraft.surfaces])
    hi = max([b.x[-1] for b in aircraft.bodies] + [max(sec.le[0] + sec.chord for sec in s.sections) for s in aircraft.surfaces])
    xs = np.linspace(lo, hi, n + 2)[1:-1]
    area = bodies_area(aircraft.bodies, xs)
    fuselage = [b for b in aircraft.bodies if b.kind == "fuselage"]
    for surf in aircraft.surfaces:
        etas = np.linspace(0.0, 1.0, 201)
        mids = 0.5 * (etas[1:] + etas[:-1])
        ds = np.diff(etas) * surf.half_arc
        for e, d in zip(mids, ds):
            le, chord, _, foil = surf.station(e)
            xi = (xs - le[0]) / chord
            on = (xi > 0.0) & (xi < 1.0)
            if not np.any(on):
                continue
            t = 2.0 * foil.half_thickness(xi[on]) * chord
            # not inside the fuselage's section there
            if fuselage:
                w, top, bot, yc, _ = fuselage[0].section(xs[on])
                hidden = (np.abs(le[1] - yc) < 0.5 * w) & (le[2] > bot) & (le[2] < top)
                t = np.where(hidden, 0.0, t)
            area[on] += t * d * (2 if surf.mirror else 1)
    for e in aircraft.engines:
        if e.type != "turbofan":
            continue
        _, diameter = e.jet_size()
        cap = 0.6 * math.pi * diameter ** 2 / 4.0 * len(e.copies())
        inlet = e.inlet_x if e.inlet_x is not None else lo + 0.3 * (hi - lo)
        area -= cap * ((xs >= inlet) & (xs <= e.prop_position[0]))
    return np.maximum(area, 0.0), hi - lo
