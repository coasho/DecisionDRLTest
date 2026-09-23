"""The aerodynamic model as JSBSim tables.

The build-up JSBSim aircraft use, every term a table over the angles it
depends on:

  C = C_base(alpha, beta)                       all six coefficients
    + dC_channel(alpha, delta)                  per control channel (beta = 0)
    + C_p(alpha) p b/2V + C_r(alpha) r b/2V     lateral rates
    + C_q(alpha) q c/2V + C_alphadot alphadot c/2V

alpha covers the whole circle and beta +/-90 deg - the attitudes an agent in
training will reach - with 1 deg steps through the stall. Rates are finite
differences of the nonlinear model at +/-0.03, so damping changes through the
stall (a wing's roll damping turns to autorotation) are in the tables.
"""
import math

import numpy as np

COEFFS = ("CD", "CY", "CL", "Cl", "Cm", "Cn")
CHANNEL_COEFFS = {
    "elevator": ("CD", "CL", "Cm"),
    "flap": ("CD", "CL", "Cm"),
    "aileron": ("CD", "CY", "Cl", "Cn"),
    "rudder": ("CD", "CY", "Cl", "Cn"),
}


def alpha_grid(fine=1.0):
    coarse_neg = [-180, -165, -150, -135, -120, -105, -90, -75, -60, -50, -45, -40, -35, -30, -27, -24, -22]
    coarse_pos = [32, 35, 38, 42, 46, 50, 55, 60, 70, 80, 90, 105, 120, 135, 150, 165, 180]
    mid = list(np.arange(-20.0, 30.0 + 1e-9, fine))
    return np.array(coarse_neg + mid + coarse_pos, float)


def beta_grid():
    return np.array([-90, -70, -50, -40, -30, -25, -20, -15, -10, -6, -3, 0, 3, 6, 10, 15, 20, 25, 30, 40, 50, 70, 90], float)


def control_grid(lo, hi, step=5.0):
    pts = set(np.round(np.arange(math.floor(lo / step) * step, hi + 1e-9, step), 6))
    pts.update([lo, hi, 0.0])
    return np.array(sorted(p for p in pts if lo - 1e-9 <= p <= hi + 1e-9), float)


def build(model, alpha=None, beta=None, progress=None):
    """Every table, as {name: dict(axes..., data)} with angles in degrees.
    [analysis] quick = true builds coarse grids (tests, first looks)."""
    quick = bool(model.aircraft.spec.get("analysis", {}).get("quick", False))
    a_deg = (alpha_grid(4.0 if quick else 1.0)) if alpha is None else np.asarray(alpha, float)
    if beta is not None:
        b_deg = np.asarray(beta, float)
    else:
        b_deg = np.array([-90, -40, -15, -5, 0, 5, 15, 40, 90], float) if quick else beta_grid()
    a_rad, b_rad = np.radians(a_deg), np.radians(b_deg)
    out = {"alpha": a_deg, "beta": b_deg}
    total = len(a_deg) * len(b_deg)
    done = 0
    # base: all six over alpha x beta
    base = {k: np.zeros((len(a_deg), len(b_deg))) for k in COEFFS}
    for j, b in enumerate(b_rad):
        for i, a in enumerate(a_rad):
            c = model.evaluate(a, b)
            for k in COEFFS:
                base[k][i, j] = c[k]
            done += 1
        if progress:
            progress("base tables", done, total)
    # exact symmetry about beta = 0 (the numerics are symmetric to round-off)
    jb = {round(b, 6): j for j, b in enumerate(b_deg)}
    for j, b in enumerate(b_deg):
        m = jb.get(round(-b, 6))
        if m is None or m < j:
            continue
        for k in ("CD", "CL", "Cm"):
            avg = 0.5 * (base[k][:, j] + base[k][:, m])
            base[k][:, j] = base[k][:, m] = avg
        for k in ("CY", "Cl", "Cn"):
            odd = 0.5 * (base[k][:, j] - base[k][:, m])
            base[k][:, j], base[k][:, m] = odd, -odd
    out["base"] = base
    zero = {k: base[k][:, jb[0.0]] for k in COEFFS}
    # controls: increments over alpha x deflection
    a_ctl = a_deg
    ctrl = {}
    for ch in model.aircraft.channels():
        lims = [(c.min_deg, c.max_deg) for _, c in model.aircraft.controls() if c.channel == ch]
        lo, hi = min(l[0] for l in lims), max(l[1] for l in lims)
        d_deg = control_grid(lo, hi, (12.0 if quick else 5.0) if hi - lo > 12 else 2.5)
        tab = {k: np.zeros((len(a_ctl), len(d_deg))) for k in CHANNEL_COEFFS[ch]}
        for jd, d in enumerate(d_deg):
            for i, a in enumerate(np.radians(a_ctl)):
                if abs(d) < 1e-9:
                    continue
                c = model.evaluate(a, 0.0, controls={ch: math.radians(d)})
                for k in tab:
                    tab[k][i, jd] = c[k] - zero[k][i]
            if progress:
                progress("control " + ch, jd + 1, len(d_deg))
        ctrl[ch] = {"deflection": d_deg, **tab}
    out["controls"] = ctrl
    # rates: derivatives over alpha, by central differences
    h = 0.03
    rates = {}
    for name, kw, keys in (("p", "p", ("CY", "Cl", "Cn")), ("r", "r", ("CY", "Cl", "Cn")), ("q", "q", ("CL", "Cm", "CD"))):
        tab = {k: np.zeros(len(a_ctl)) for k in keys}
        for i, a in enumerate(np.radians(a_ctl)):
            cp = model.evaluate(a, 0.0, **{kw: h})
            cm = model.evaluate(a, 0.0, **{kw: -h})
            for k in keys:
                tab[k][i] = (cp[k] - cm[k]) / (2 * h)
        rates[name] = tab
        if progress:
            progress("rate " + name, 1, 1)
    out["rates"] = rates
    out["alphadot"] = alphadot(model)
    out["ground_effect"] = ground_effect(model)
    return out


def alphadot(model):
    """CL and Cm per unit alpha-dot c/2V: the lag of the wing's downwash at
    the tail (DATCOM 7.1.2.3): Cm_ad = -2 CLa_t (S_t/S)(l_t/c)^2 deps/dalpha
    with the tail's lift slope and the downwash gradient taken from the
    lattice solution."""
    a = model.aircraft
    tails = [s for s in a.surfaces if s.kind == "htail"]
    if not tails:
        return {"CL": 0.0, "Cm": 0.0, "deps_dalpha": 0.0}
    tail = tails[0]
    L = model.lat
    ti = [i for i, s in enumerate(L.surfaces) if s is tail][0]
    wi = [i for i, s in enumerate(L.surfaces) if s is a.wing][0]
    idx = np.flatnonzero(L.surface_index == ti)
    # downwash angle at the tail strips from the wing groups, per radian of alpha
    eps = []
    for adeg in (0.0, 2.0):
        c = model.evaluate(math.radians(adeg), detail=True)["detail"]
        eps.append(c)
    d_ind = []
    for adeg in (0.0, 2.0):
        info = _induced_by(model, math.radians(adeg), wi)
        d_ind.append(np.average(info[idx], weights=L.area[idx]))
    deps = -(d_ind[1] - d_ind[0]) / math.radians(2.0)
    mac, le = tail.mac
    arm = le[0] + 0.25 * mac - a.aero_point[0]
    cla_t = _tail_lift_slope(model, idx)
    cm = -2.0 * cla_t * (tail.area / a.S) * (arm / a.c) ** 2 * deps
    cl = 2.0 * cla_t * (tail.area / a.S) * (arm / a.c) * deps
    return {"CL": float(cl), "Cm": float(cm), "deps_dalpha": float(deps), "tail_lift_slope": float(cla_t)}


def _induced_by(model, alpha, surface_index):
    """Induced (upwash) angle at every strip from one surface's vortices."""
    from .model import wind_axes
    L, V = model.lat, model.vlm
    xw, _, _ = wind_axes(alpha, 0.0)
    v = np.array([xw[0], -xw[1], xw[2]])
    w = np.zeros(3)
    g = V.circulation(v, w, {})
    mask = (L.panel_surface == surface_index).astype(float)
    wk = np.einsum("snk,n->sk", model.W_strip, g * mask)
    U = v[None, :] - np.cross(w[None, :], L.c4 - model.ref)
    return np.einsum("sk,sk->s", wk, L.u) / np.linalg.norm(U, axis=1)


def _tail_lift_slope(model, idx):
    """Lift slope of the tail strips' own normal force per radian of their
    angle, referred to the tail's area, from two nearby conditions."""
    L = model.lat
    vals = []
    for adeg in (0.0, 2.0):
        info = model.evaluate(math.radians(adeg), detail=True)["detail"]
        vals.append((info["cl"][idx] * L.area[idx]).sum() / L.area[idx].sum())
    return (vals[1] - vals[0]) / math.radians(2.0) * 1.0


def ground_effect(model):
    """Lift and induced-drag factors vs height/span: McCormick's
    (16 h/b)^2 / (1 + (16 h/b)^2) for the induced drag, and a lift gain
    rising to ~20 % at the ground (the trend JSBSim's GA models use)."""
    hb = np.array([0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1])
    k = (16 * hb) ** 2
    drag = np.where(hb > 0, k / (1 + k), 0.0)
    lift = 1.0 + 0.2 * np.exp(-hb / 0.12) * (hb < 1.05)
    return {"h_b": hb, "lift": lift, "drag": drag}


def derivatives(tables, alpha_deg=2.0):
    """Stability derivatives (per rad) near a reference alpha, read off the
    tables - the numbers a designer checks and the linear model uses."""
    a = tables["alpha"]
    b = tables["beta"]
    base = tables["base"]
    j0 = int(np.argmin(np.abs(b)))

    def at(tab, x):
        return float(np.interp(x, a, tab))

    da = 1.0
    out = {}
    for k in ("CL", "CD", "Cm"):
        out[k + "0"] = at(base[k][:, j0], 0.0)
        out[k + "a"] = (at(base[k][:, j0], alpha_deg + da) - at(base[k][:, j0], alpha_deg - da)) / math.radians(2 * da)
    jp = int(np.argmin(np.abs(b - 3.0)))
    jm = int(np.argmin(np.abs(b + 3.0)))
    for k in ("CY", "Cl", "Cn"):
        out[k + "b"] = (at(base[k][:, jp], alpha_deg) - at(base[k][:, jm], alpha_deg)) / math.radians(b[jp] - b[jm])
    for rate, ks in tables["rates"].items():
        for k, v in ks.items():
            out[k + rate] = at(v, alpha_deg)
    for ch, t in tables["controls"].items():
        d = t["deflection"]
        i0 = int(np.argmin(np.abs(d)))
        ip = min(i0 + 1, len(d) - 1)
        im = max(i0 - 1, 0)
        for k in t:
            if k == "deflection":
                continue
            out[k + "_" + ch] = (at(t[k][:, ip], alpha_deg) - at(t[k][:, im], alpha_deg)) / math.radians(d[ip] - d[im])
    out["CLad"] = tables["alphadot"]["CL"]
    out["Cmad"] = tables["alphadot"]["Cm"]
    return out
