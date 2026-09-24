"""Does the JSBSim aircraft fly the model hangar computed?

Two independent checks:

1. TableModel evaluates the tables exactly as the JSBSim file does
   (bilinear, clamped at the ends, the same superposition). Flown in JSBSim
   through random states, the aerodynamic forces and moments JSBSim reports
   must equal TableModel's at the same alpha, beta, rates and surface
   positions - that checks the writer: units, signs, property names, axes.
2. TableModel against the full AeroModel at the same states measures what
   the tables' build-up (superposition, grids) costs.
"""
import math

import numpy as np

from .aero.model import wind_axes
from .aero.tables import COEFFS

FT, LBF, SLUG_FT2 = 0.3048, 4.448222, 1.35582
PSF = 47.880259


def _interp2(rows, cols, data, r, c):
    r = min(max(r, rows[0]), rows[-1])
    c = min(max(c, cols[0]), cols[-1])
    i = int(np.clip(np.searchsorted(rows, r) - 1, 0, len(rows) - 2))
    j = int(np.clip(np.searchsorted(cols, c) - 1, 0, len(cols) - 2))
    tr = (r - rows[i]) / (rows[i + 1] - rows[i])
    tc = (c - cols[j]) / (cols[j + 1] - cols[j])
    return ((1 - tr) * (1 - tc) * data[i, j] + tr * (1 - tc) * data[i + 1, j]
            + (1 - tr) * tc * data[i, j + 1] + tr * tc * data[i + 1, j + 1])


class TableModel:
    """The coefficient build-up of the JSBSim file, evaluated in Python."""

    def __init__(self, tables):
        self.t = tables

    def evaluate(self, alpha_deg, beta_deg, p=0.0, q=0.0, r=0.0, adot=0.0, controls_deg=None, mach=0.0, cl2=None):
        """p, q, r, adot non-dimensional (p b/2V, ...); controls in degrees;
        cl2: the lift coefficient squared the induced-drag terms use (JSBSim:
        the last step's; default: this state's)."""
        t = self.t
        a, b = t["alpha"], t["beta"]
        mt = t.get("mach")
        K = (lambda key: float(np.interp(mach, mt["mach"], mt[key]))) if mt is not None else (lambda key: 1.0 if key[0] == "K" else 0.0)
        out = {k: _interp2(a, b, t["base"][k], alpha_deg, beta_deg) for k in COEFFS}
        cl_base = out["CL"]
        out["CL"] *= K("K_L")
        for k in ("CY", "Cl", "Cn"):
            out[k] *= K("K_Y")
        out["Cm"] += cl_base * K("dCm_dCL")
        for ch, d in (controls_deg or {}).items():
            if ch not in t["controls"]:
                continue
            tab = t["controls"][ch]
            for k in tab:
                if k != "deflection":
                    out[k] += _interp2(a, tab["deflection"], tab[k], alpha_deg, d) * K("K_" + ch)
        for rate, val in (("p", p), ("r", r), ("q", q)):
            for k, data in t["rates"][rate].items():
                if k != "CD":
                    out[k] += float(np.interp(alpha_deg, a, data)) * val * K("K_L")
        out["CL"] += t["alphadot"]["CL"] * adot * K("K_L")
        out["Cm"] += t["alphadot"]["Cm"] * adot * K("K_L")
        out["CD"] += K("dCD0") + (out["CL"] ** 2 if cl2 is None else cl2) * K("dK")
        return out


def body_forces(coef, alpha, beta, qbar, S, b, c):
    """Force and moment (about the aero reference point) in body axes from
    JSBSim-convention coefficients."""
    xw, yw, zw = wind_axes(alpha, beta)
    F = qbar * S * (-coef["CD"] * xw + coef["CY"] * yw - coef["CL"] * zw)
    M = qbar * S * np.array([coef["Cl"] * b, coef["Cm"] * c, coef["Cn"] * b])
    return F, M


def sample_states(f, n=120, seed=3, seconds=0.6):
    """Random states flown in JSBSim, with what JSBSim computed there."""
    rng = np.random.default_rng(seed)
    rows = []
    for i in range(n):
        v = f.spawn(float(rng.uniform(1000, 2500)), float(rng.uniform(25, 80)), heading_deg=float(rng.uniform(0, 360)),
                    pitch_deg=float(rng.uniform(-60, 60)), roll_deg=float(rng.uniform(-90, 90)))
        cmd = rng.uniform(-1, 1, 3)
        flaps = float(rng.choice([0.0, 0.0, 0.5, 1.0]))
        v.command_actuator(aileron=float(cmd[0]), elevator=float(cmd[1]), rudder=float(cmd[2]), throttle=0.5, flaps=flaps)
        f.world.step(int(seconds / f.dt))
        g = lambda name: f.prop(v, name)  # noqa: E731
        rows.append({
            "alpha": g("aero/alpha-rad"), "beta": g("aero/beta-rad"), "vt": g("velocities/vt-fps") * FT,
            "p": g("velocities/p-aero-rad_sec"), "q": g("velocities/q-aero-rad_sec"), "r": g("velocities/r-aero-rad_sec"),
            "adot": g("aero/alphadot-rad_sec"), "qbar": g("aero/qbar-psf") * PSF,
            "de": g("fcs/elevator-pos-deg"), "da": g("fcs/left-aileron-pos-deg"), "dr": g("fcs/rudder-pos-deg"),
            "df": g("fcs/flap-pos-deg"), "hb": g("aero/h_b-mac-ft"), "mach": g("velocities/mach"),
            "cl2": g("aero/cl-squared"),
            "F": np.array([g("forces/fbx-aero-lbs"), g("forces/fby-aero-lbs"), g("forces/fbz-aero-lbs")]) * LBF,
            "M": np.array([g("moments/l-aero-lbsft"), g("moments/m-aero-lbsft"), g("moments/n-aero-lbsft")]) * LBF * FT,
            "cg": np.array([g("inertia/cg-x-in"), g("inertia/cg-y-in"), g("inertia/cg-z-in")]) * 0.0254,
        })
        v.remove()
    return rows


def compare(rows, tables, aircraft, model=None):
    """JSBSim vs the tables (and the tables vs the full model), as
    coefficient errors per state."""
    tm = TableModel(tables)
    S, b, c = aircraft.S, aircraft.b, aircraft.c
    arp = aircraft.aero_point
    out = []
    for s in rows:
        if s["hb"] < 1.2 or not np.isfinite(s["alpha"]):
            continue  # in ground effect: skip
        V = max(s["vt"], 1.0)
        nd = dict(p=s["p"] * b / (2 * V), q=s["q"] * c / (2 * V), r=s["r"] * b / (2 * V), adot=s["adot"] * c / (2 * V))
        ctl = {"elevator": s["de"], "aileron": s["da"], "rudder": s["dr"], "flap": s["df"]}
        coef = tm.evaluate(math.degrees(s["alpha"]), math.degrees(s["beta"]), controls_deg=ctl, mach=s["mach"], cl2=s["cl2"], **nd)
        F, M = body_forces(coef, s["alpha"], s["beta"], s["qbar"], S, b, c)
        # JSBSim's moments are about the CG: move ours from the aero reference point
        d = arp - s["cg"]
        d_body = np.array([-d[0], d[1], -d[2]])
        M_cg = M + np.cross(d_body, F)
        qs = s["qbar"] * S
        err = {"F": (F - s["F"]) / qs, "M": (M_cg - s["M"]) / (qs * np.array([b, c, b]))}
        row = {"alpha_deg": math.degrees(s["alpha"]), "beta_deg": math.degrees(s["beta"]), "err": err}
        if model is not None:
            full = model.evaluate(s["alpha"], s["beta"], nd["p"], nd["q"], nd["r"],
                                  controls={k: math.radians(v) for k, v in ctl.items()})
            row["superposition"] = {k: coef[k] - t_adot(full, k, tables, nd["adot"]) for k in COEFFS}
        out.append(row)
    return out


def t_adot(full, k, tables, adot):
    """The full model has no alpha-dot term: add the tables' to compare."""
    v = full[k]
    if k == "CL":
        v += tables["alphadot"]["CL"] * adot
    if k == "Cm":
        v += tables["alphadot"]["Cm"] * adot
    return v
