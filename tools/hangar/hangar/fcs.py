"""Fly-by-wire: the control laws a fighter flies with, and their gains from
the aircraft's own linear model.

A modern fighter is not flown by its surfaces but by its flight control
computers, and many are unstable without them. With [flight_control]
type = "fbw" in the design, the JSBSim aircraft gets these laws in place of
the direct stick-to-surface channels; the command properties stay the same
(fcs/elevator-cmd-norm, fcs/aileron-cmd-norm, fcs/rudder-cmd-norm), so an
agent or a pilot flies it like any other aircraft:

- pitch: a load factor command. Neutral stick holds the flight path (the
  load factor that balances gravity, cos(theta) cos(phi)); full aft stick
  commands n_max, full forward n_min - no more than the lift at the angle
  of attack limits gives beyond that reference, through a 0.2 s prefilter
  and a 12 g/s onset limit. An inner loop feeds back angle of
  attack and pitch rate to place the short period - frequency from the
  Control Anticipation Parameter (CAP 1, the middle of MIL-F-8785C's level 1
  for category A), damping 0.8 - which makes an unstable airframe fly like a
  stable one; a feedforward gives the command, an integrator on the error
  from the response it should give (a 0.4 s lag of the command) trims.
  Approaching alpha_max the command is limited to the load factor the
  aircraft pulls plus what the angle of attack left gives (counting its
  rise over the next 0.35 s as spent), so the aircraft cannot be pulled
  past it: at the limit the feedforward asks for what the airframe gives
  there, and the integrator, five times as fast on the angle of attack
  left, trims the elevator that holds it - a steady pull on a stable
  airframe, a push on an unstable one. Past the limit a push back in
  proportion gives all the nose-down travel 4 deg beyond, so an unstable
  airframe pitching up fast is caught at once. The pitching moment's
  departures from a straight line through the angle-of-attack envelope (a
  tail in the wing's wake, vortex lift, a break) are cancelled by the
  elevator from a table over angle of attack and Mach number, and the
  gains are designed on the line: an airframe unstable in one band of
  angle of attack and stable in the next flies as the design expects
  through both;
- roll: a roll-rate command about the flight path (stability axes), the
  roll mode's time constant placed at 0.2 s, the rate limited by the
  rolling moment available; an integrator holds the bank angle at neutral
  stick. A roll about the flight path at angle of attack is also a yaw
  (r = p sin(alpha)), and the rudder has to give what the ailerons' own yaw
  does not: the command is limited to what the rudder can coordinate - its
  size to the rate whose yaw half the rudder's power holds while the
  aircraft pitches under the roll (the pitch rate's inertia coupling, a
  rising angle of attack, the yaw damping), and while it grows, its rate
  of change to the roll acceleration whose yaw all of it gives (unless
  even an uncoordinated roll could not leave 2 deg of sideslip). At low
  speed and high angle of attack the rudder's yaw acceleration is a
  twentieth to a fifth of the ailerons' roll acceleration, and a roll it
  cannot follow turns the angle of attack into sideslip;
- yaw: a yaw damper on the sideslip's rate (zero in a steady coordinated
  turn, in a roll about the flight path and in a steady sideslip, so it
  coordinates those without being washed out) and sideslip feedback where
  the airframe's own weathercock stability is short of a dutch roll of
  1.5 rad/s at damping 0.5; the pedals command sideslip.

Gains are tables over dynamic pressure and Mach number, from pole placement
on the linear model (linear.py's derivatives with the compressibility
factors, as the JSBSim file applies them) at 1 g trim at each of those
conditions.

Thrust vectoring ([engine.nozzle] vectoring, its plane canted
vectoring_cant outboard from the vertical): the nozzles turn with the
surfaces - through their travel as the elevator goes through its own,
differentially half their travel with the ailerons, and with the rudder
where the plane is canted (the Su-57's) - so they add their power to the
surfaces'. That power grows with thrust, the surfaces' with dynamic
pressure: the gains are designed as moments and divided at run time by the
power the surfaces and the nozzles give together at the thrust the engines
make, so the loops respond the same at idle and in full afterburner, and
the nozzles add authority where the surfaces run out of it - at low speed
and high angle of attack.
"""
import math

import numpy as np

from .aero import tables as T
from .linear import G0, loaded_inertia

PSF = 47.880259
QBAR_PSF = (10.0, 20.0, 40.0, 80.0, 150.0, 300.0, 600.0, 1200.0, 2400.0)
MACH = (0.15, 0.4, 0.7, 0.9, 1.1, 1.4, 2.0)
A_SOUND = 320.0          # m/s: the speed of sound at mid altitudes, to turn Mach into speed
LIMIT_INTEGRAL = 5.0     # how much faster the pitch integrator trims the angle of attack left at a limit
LBF = 4.4482216          # N per lbf: JSBSim reports thrust in lbf
ROLL_SHARE = 0.5         # of a vectoring nozzle's travel the ailerons (and the rudder) use differentially
PULL_YAW = 0.5           # of the rudder's yaw a roll's rate may spend while the aircraft pitches under it
SHARE_FLOOR = 0.01       # the least rudder's share of a roll's yaw the roll acceleration is limited by
ENTRY_SIDESLIP = math.radians(2.0)   # rad: the sideslip a change of roll rate may leave uncoordinated
SHARE_ALPHA = tuple(range(-20, 55, 5))     # deg: where the rudder's share of a roll's yaw is tabulated

DEFAULTS = {"n_max": 9.0, "n_min": -3.0, "alpha_max_deg": 25.0, "alpha_min_deg": -10.0, "roll_rate_deg_s": 300.0,
            "sideslip_deg": 10.0, "cap": 1.0, "short_period_zeta": 0.8, "roll_time_constant_s": 0.2,
            "dutch_roll_omega": 1.5, "dutch_roll_zeta": 0.5, "pitch_integral_s": 1.0, "bank_hold_s": 1.5}


def options(aircraft):
    """The design's [flight_control] settings over the defaults, or None for
    an aircraft without fly-by-wire."""
    spec = aircraft.spec.get("flight_control", {})
    if spec.get("type", "direct") != "fbw":
        return None
    out = dict(DEFAULTS)
    for k in DEFAULTS:
        if k in spec:
            out[k] = float(spec[k])
    return out


def vectoring(aircraft, inertia=None):
    """The thrust-vectoring nozzles, in JSBSim's engine order: each one's
    deflection per unit of the surfaces' positions (rad per rad: the
    elevator's through its whole travel, the aileron's and the rudder's -
    canted only - differentially through ROLL_SHARE of it), its travel and
    cant, its side (+1 left, -1 right: the left nozzle's deflection is the
    one the aileron's position adds to) and base orientation. With inertia
    (linear.loaded_inertia's) also the control power per lbf of its thrust
    about each axis: 1/s^2 per rad of the surface, signed as the surface's
    own (JSBSim's thruster: a positive pitch angle pushes the tail up, a
    positive yaw angle pushes it right). [] without vectoring nozzles."""
    ch = aircraft.channels()

    def travel(k):
        return math.radians(max(abs(x) for x in aircraft.channel_limits(k))) if k in ch else 0.0
    de, da, dr = travel("elevator"), travel("aileron"), travel("rudder")
    out = []
    i = 0
    for e in aircraft.engines:
        for name, _, nozzle, _ in e.copies():
            if e.vectoring > 0.0:
                vec, cant = math.radians(e.vectoring), math.radians(e.vectoring_cant)
                side = -float(np.sign(round(float(nozzle[1]), 6)))
                v = {"engine": i, "name": name, "nozzle": [float(x) for x in nozzle], "travel": vec, "cant": cant,
                     "side": side, "orient": [float(x) for x in e.prop_orient],
                     "k_elevator": vec / de if de > 0 else 0.0,
                     "k_aileron": ROLL_SHARE * vec / da if da > 0 and side else 0.0,
                     "k_rudder": ROLL_SHARE * vec / dr if dr > 0 and side and cant > 0 else 0.0}
                if inertia is not None:
                    _, cg, Ixx, Iyy, Izz, _ = inertia
                    lx = float(nozzle[0] - cg[0])            # behind the centre of gravity
                    ly = abs(float(nozzle[1] - cg[1]))
                    # the pitch deflection is the elevator's, nose down; the differential
                    # one rolls right with the left nozzle's tail pushed up; the canted
                    # yaw deflection is the rudder's, nose left
                    v["md"] = -LBF * v["k_elevator"] * math.cos(cant) * lx / Iyy
                    v["lda"] = LBF * v["k_aileron"] * math.cos(cant) * ly / Ixx
                    v["ndr"] = -LBF * v["k_rudder"] * math.sin(cant) * lx / Izz
                out.append(v)
            i += 1
    return out


def _mach_factor(mt, key, mach):
    if mt is None:
        return 1.0 if key.startswith("K") else 0.0
    return float(np.interp(mach, mt["mach"], mt[key]))


def derivatives_at(tabs, alpha_deg, mach):
    """Stability and control derivatives (per rad) at an angle of attack and
    a Mach number: the low-speed tables' with the compressibility factors, as
    the JSBSim file applies them (lift, damping and the lateral coefficients
    scaled, the neutral point's move as Cm per unit lift)."""
    d = T.derivatives(tabs, alpha_deg)
    mt = tabs.get("mach")
    KL, KY = _mach_factor(mt, "K_L", mach), _mach_factor(mt, "K_Y", mach)
    out = dict(d)
    out["CLa"] = d["CLa"] * KL
    out["Cma"] = d["Cma"] + d["CLa"] * _mach_factor(mt, "dCm_dCL", mach)
    for k in ("CLq", "Cmq", "CLad", "Cmad", "Clp", "Cnp", "CYp", "Clr", "Cnr", "CYr"):
        if k in d:
            out[k] = d[k] * KL
    for k in ("CYb", "Clb", "Cnb"):
        out[k] = d[k] * KY
    for ch in tabs["controls"]:
        K = _mach_factor(mt, "K_" + ch, mach)
        for key in d:
            if key.endswith("_" + ch):
                out[key] = d[key] * K
    return out


def _trim_alpha(tabs, CL, mach):
    """Angle of attack (deg) for a lift coefficient at a Mach number, capped
    at the lift's maximum."""
    mt = tabs.get("mach")
    CLb = CL / _mach_factor(mt, "K_L", mach)
    a = tabs["alpha"]
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    k = (a > -10) & (a < 45)
    cl = tabs["base"]["CL"][k, j0]
    i = int(np.argmax(cl))
    return float(np.interp(min(CLb, cl[i]), cl[: i + 1], a[k][: i + 1]))


def moment_line(tabs, aircraft, cg, opt):
    """The pitching moment about the centre of gravity over the angle-of-
    attack envelope (low speed, no sideslip), the straight line fitted
    through it, and the elevator power at each angle of attack: the control
    law cancels the moment's departures from the line with the elevator and
    is designed on the line. None without an elevator."""
    t = tabs["controls"].get("elevator")
    if t is None:
        return None
    a = aircraft
    al = tabs["alpha"]
    r = np.radians(al)
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    base = tabs["base"]
    dx = (cg[0] - a.aero_point[0]) / a.c
    cn = base["CL"][:, j0] * np.cos(r) + base["CD"][:, j0] * np.sin(r)
    cm = base["Cm"][:, j0] + cn * dx
    k = (al >= opt["alpha_min_deg"] - 1e-9) & (al <= opt["alpha_max_deg"] + 1e-9)
    slope, icpt = np.polyfit(r[k], cm[k], 1)
    d = t["deflection"]
    i0 = int(np.argmin(np.abs(d)))
    ip, im = min(i0 + 1, len(d) - 1), max(i0 - 1, 0)
    step = math.radians(d[ip] - d[im])
    cm_de = (t["Cm"][:, ip] - t["Cm"][:, im]) / step
    if "CL" in t:
        cm_de = cm_de + (t["CL"][:, ip] - t["CL"][:, im]) / step * dx
    return {"alpha_deg": al[k], "slope": float(slope), "departure": (cm - icpt - slope * r)[k], "cm_de": cm_de[k]}


def nose_down_reach(tabs, aircraft, cg, opt, search=20.0):
    """How far past alpha_max full nose-down control still brings the nose
    down: the pitching moment about the centre of gravity with the pitch
    channel at its nose-down stop (low speed, no sideslip), and the first
    angle of attack past alpha_max where it is no longer negative - None if
    it stays negative `search` deg past, or without an elevator. Past it an
    aircraft the limiter lets through would hang there."""
    t = tabs["controls"].get("elevator")
    if t is None:
        return None
    a = aircraft
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    base = tabs["base"]
    i = int(np.argmax(t["deflection"]))     # nose-down elevator is positive
    al = np.arange(opt["alpha_max_deg"], opt["alpha_max_deg"] + search + 1e-9, 0.25)
    r = np.radians(al)

    def at(tab):
        return np.interp(al, tabs["alpha"], tab)
    cl = at(base["CL"][:, j0]) + (at(t["CL"][:, i]) if "CL" in t else 0.0)
    cd = at(base["CD"][:, j0]) + (at(t["CD"][:, i]) if "CD" in t else 0.0)
    cm = at(base["Cm"][:, j0]) + at(t["Cm"][:, i]) + (cl * np.cos(r) + cd * np.sin(r)) * (cg[0] - a.aero_point[0]) / a.c
    up = np.nonzero(cm >= 0.0)[0]
    return {"alpha_deg": float(al[up[0]]) if len(up) else None, "cm_at_limit": float(cm[0]),
            "deflection_deg": float(t["deflection"][i])}


def moment_compensation(tabs, line, machs, de_lo, de_hi):
    """The elevator (rad) that cancels the pitching moment's departure from
    its line, over angle of attack (rows) and Mach number (columns): the
    departure over the elevator's power there, which Mach scales."""
    mt = tabs.get("mach")
    out = np.zeros((len(line["alpha_deg"]), len(machs)))
    for j, mach in enumerate(machs):
        power = line["cm_de"] * _mach_factor(mt, "K_elevator", mach)
        ok = power < -1e-3          # nose-down elevator is positive, its moment negative
        out[ok, j] = -line["departure"][ok] / power[ok]
    return np.clip(out, de_lo, de_hi)


def roll_yaw_share(tabs, inertia, alphas=SHARE_ALPHA):
    """The rudder's share of the yaw a roll about the flight path needs, per
    unit of its roll acceleration, over angle of attack (deg). A stability-
    axis roll acceleration a turns the body at a cos(alpha) about its x axis
    and a sin(alpha) about its z axis: the moments Ixx pdot - Ixz rdot and
    Izz rdot - Ixz pdot. The ailerons give the rolling moment, and with it
    the yaw mu = Cn_da / Cl_da of it (the tables' slopes at neutral); the
    rudder the rest of the yaw - per unit of a, as a yaw acceleration,
    sin(alpha) - eta cos(alpha) - (Ixz / Izz)(cos(alpha) - mu sin(alpha)),
    eta = mu Ixx / Izz. Almost none at a cruising angle of attack, where the
    ailerons' yaw and a principal axis inclined to the flight path roll the
    aircraft about the flight path by themselves; half and more past 30 deg."""
    _, _, Ixx, _, Izz, Ixz = inertia
    out = []
    for al in alphas:
        d = T.derivatives(tabs, al)
        cl, cn = d.get("Cl_aileron", 0.0), d.get("Cn_aileron", 0.0)
        mu = cn / cl if cl > 1e-3 else 0.0
        r = math.radians(al)
        out.append(math.sin(r) - mu * Ixx / Izz * math.cos(r) - Ixz / Izz * (math.cos(r) - mu * math.sin(r)))
    return np.array(alphas, float), np.array(out)


def design_point(tabs, aircraft, inertia, qbar_pa, mach, opt, cma_line=None):
    """The gains at one flight condition (dynamic pressure, Mach number),
    and the closed loop's modes there. With cma_line (the slope of
    moment_line) the airframe's pitch stiffness is the line's, with Mach's
    changes on it - the moment's departures from it are compensated."""
    m, cg, Ixx, Iyy, Izz, _ = inertia
    a = aircraft
    S, b, c = a.S, a.b, a.c
    V = max(mach * A_SOUND, 30.0)
    Q = qbar_pa
    CL = m * G0 / (Q * S)
    # 1 g trim - or, too slow for 1 g, the angle-of-attack limit it flies at
    alpha = min(_trim_alpha(tabs, CL, mach), opt["alpha_max_deg"])
    d = derivatives_at(tabs, alpha, mach)
    ar = math.radians(alpha)
    # moments about the centre of gravity
    dx = cg[0] - a.aero_point[0]
    Cma = d["Cma"] + d["CLa"] * dx / c
    if cma_line is not None:
        low = T.derivatives(tabs, alpha)
        Cma += cma_line - (low["Cma"] + low["CLa"] * dx / c)
    Cmq = d["Cmq"] + d.get("Cmad", 0.0) + d["CLq"] * dx / c
    Cmde = d.get("Cm_elevator", 0.0) + d.get("CL_elevator", 0.0) * dx / c
    CLde = d.get("CL_elevator", 0.0)
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    CD = float(np.interp(alpha, tabs["alpha"], tabs["base"]["CD"][:, j0]))
    # -- pitch: short period, alpha and pitch rate feedback --------------------------------------
    Za = -Q * S * (d["CLa"] + CD) / (m * V)
    Zd = -Q * S * CLde / (m * V)
    Ma = Q * S * c * Cma / Iyy
    Mq = Q * S * c * c * Cmq / (2 * Iyy * V)
    Md = Q * S * c * Cmde / Iyy
    n_alpha = max(Q * S * d["CLa"] / (m * G0), 1.0)             # g per rad (lift still rising)
    omega = float(np.clip(math.sqrt(max(opt["cap"] * n_alpha, 0.0)), 1.5, 6.0))
    zeta = opt["short_period_zeta"]
    # characteristic polynomial of A - B K, K = [ka, kq], is linear in the gains:
    # trace = Za + Mq - ka Zd - kq Md, det = (Za Mq - Ma) + ka (Md - Zd Mq) + kq (Zd Ma - Za Md)
    M2 = np.array([[-Zd, -Md], [Md - Zd * Mq, Zd * Ma - Za * Md]])
    rhs = np.array([-2 * zeta * omega - (Za + Mq), omega * omega - (Za * Mq - Ma)])
    try:
        ka, kq = np.linalg.solve(M2, rhs)
    except np.linalg.LinAlgError:
        ka, kq = 0.0, 0.0
    A = np.array([[Za, 1.0], [Ma, Mq]])
    B = np.array([Zd, Md])
    Acl = A - np.outer(B, [ka, kq])
    try:
        xss = -np.linalg.solve(Acl, B)                      # alpha, q per unit v
        dn_per_v = V * xss[1] / G0
    except np.linalg.LinAlgError:
        dn_per_v = float("nan")
    k_ff = 1.0 / dn_per_v if np.isfinite(dn_per_v) and abs(dn_per_v) > 1e-6 else 0.0
    k_i = k_ff / opt["pitch_integral_s"]
    eig_sp = np.linalg.eigvals(Acl)
    # -- roll: rate command, feedforward and feedback --------------------------------------------
    Lp = Q * S * b * b * d["Clp"] / (2 * Ixx * V)
    Lda = Q * S * b * d.get("Cl_aileron", 0.0) / Ixx
    tau = opt["roll_time_constant_s"]
    if abs(Lda) > 1e-9:
        kp = max((1.0 / tau + Lp) / Lda, 0.0)
        kff = -Lp / Lda
    else:
        kp = kff = 0.0
    da_max = math.radians(max(abs(x) for x in aircraft.channel_limits("aileron"))) if "aileron" in aircraft.channels() else 0.0
    p_avail = Lda * da_max / max(-Lp, 1e-6) if Lda > 0 else 0.0
    p_max = float(min(math.radians(opt["roll_rate_deg_s"]), 0.8 * p_avail))
    # the bank hold: an integrator on the roll-rate error (the bank angle's),
    # stiff enough to hold the bank against the airframe in bank_hold_s
    kip = 1.0 / (Lda * opt["bank_hold_s"] ** 2) if Lda > 1e-9 else 0.0
    # -- yaw: dutch roll damping and stiffness ---------------------------------------------------
    Yb = Q * S * d["CYb"] / m
    Nb = Q * S * b * d["Cnb"] / Izz
    Lb = Q * S * b * d["Clb"] / Ixx
    Nr = Q * S * b * b * d["Cnr"] / (2 * Izz * V)
    Ndr = Q * S * b * d.get("Cn_rudder", 0.0) / Izz
    Nb_s = Nb * math.cos(ar) - Lb * math.sin(ar)       # dynamic directional stability (stability axes)
    w_dr = max(opt["dutch_roll_omega"], math.sqrt(max(Nb_s, 0.0)))
    kb = (Nb_s - w_dr * w_dr) / Ndr if abs(Ndr) > 1e-9 and Nb_s < w_dr * w_dr else 0.0
    kr = (2 * opt["dutch_roll_zeta"] * w_dr + Yb / V + Nr) / Ndr if abs(Ndr) > 1e-9 else 0.0
    if kr * Ndr <= 0:       # the airframe damps enough by itself
        kr = 0.0
    beta_max = math.radians(opt["sideslip_deg"])
    dr_max = math.radians(max(abs(x) for x in aircraft.channel_limits("rudder"))) if "rudder" in aircraft.channels() else 0.0
    kped = float(np.clip(-(Nb * beta_max) / Ndr if abs(Ndr) > 1e-9 else 0.0, 0.0, dr_max)) if Ndr < 0 else 0.0
    # the gains as moments - each gain times its surface's power, what the design
    # asks of the airframe - for thrust vectoring, which adds power at run time
    moments = {"k_alpha_m": float(ka * Md), "k_q_m": float(kq * Md), "k_ff_m": float(k_ff * Md),
               "k_i_m": float(k_i * Md), "k_roll_m": max(1.0 / tau + Lp, 0.0), "k_roll_ff_m": -Lp,
               "k_roll_i_m": 1.0 / opt["bank_hold_s"] ** 2, "p_per": 0.8 * da_max / max(-Lp, 1e-6),
               "k_yaw_r_m": max(2 * opt["dutch_roll_zeta"] * w_dr + Yb / V + Nr, 0.0),
               "k_yaw_beta_m": min(Nb_s - w_dr * w_dr, 0.0), "k_pedal_m": -Nb * beta_max,
               "md": Md, "lda": Lda, "ndr": Ndr}
    out = {"qbar_psf": Q / PSF, "mach": mach, "speed_ms": V, "alpha_deg": alpha, "n_alpha": n_alpha,
           "k_alpha": float(ka), "k_q": float(kq), "k_ff": float(k_ff), "k_i": float(k_i),
           "omega_sp": omega, "eig_sp": [complex(e) for e in eig_sp], "open_loop_Ma": Ma,
           "k_roll": float(kp), "k_roll_ff": float(kff), "k_roll_i": float(kip), "p_max": p_max,
           "k_yaw_r": float(kr), "k_yaw_beta": float(kb), "k_pedal": kped,
           # what the rudder can coordinate a roll with: its yaw acceleration at full
           # travel (1/s^2), and the yaw damping a roll's yaw rate meets (1/s)
           "yaw_accel": float(abs(Ndr) * dr_max), "yaw_damping": float(max(-Nr, 0.0))}
    out.update(moments)
    return out


GAINS = ("k_alpha", "k_q", "k_ff", "k_i", "n_alpha", "k_roll", "k_roll_ff", "k_roll_i", "p_max", "k_yaw_r", "k_yaw_beta",
         "k_pedal", "yaw_accel", "yaw_damping")
LIMITS = {"k_alpha": 6.0, "k_q": 3.0, "k_ff": 0.3, "k_i": 0.6, "k_roll": 1.5, "k_roll_ff": 1.5, "k_roll_i": 1.0,
          "k_yaw_r": 3.0, "k_yaw_beta": 3.0}
# with thrust vectoring: each axis's gains as moments, and the surfaces' power they are divided by
AXES = {"pitch": (("k_alpha", "k_q", "k_ff", "k_i"), "md"), "roll": (("k_roll", "k_roll_ff", "k_roll_i"), "lda"),
        "yaw": (("k_yaw_r", "k_yaw_beta", "k_pedal"), "ndr")}


def design(tabs, aircraft, mass_model):
    """The gain tables (dynamic pressure x Mach number) and the options, or
    None for an aircraft without fly-by-wire."""
    opt = options(aircraft)
    if opt is None:
        return None
    inertia = loaded_inertia(mass_model)
    top = float(tabs["mach"]["mach"][-1]) if tabs.get("mach") is not None else 0.9
    machs = [m for m in MACH if m <= top + 1e-9]
    line = moment_line(tabs, aircraft, inertia[1], opt)
    slope = line["slope"] if line is not None else None
    points = [[design_point(tabs, aircraft, inertia, q * PSF, m, opt, slope) for m in machs] for q in QBAR_PSF]
    tables = {k: np.array([[p[k] for p in row] for row in points]) for k in GAINS}
    for k, lim in LIMITS.items():
        tables[k] = np.clip(tables[k], -lim, lim)
    out = {"qbar_psf": np.array(QBAR_PSF), "mach": np.array(machs), "gains": tables, "options": opt, "points": points,
           "roll_yaw_share": roll_yaw_share(tabs, inertia),
           # the yaw acceleration per unit roll and pitch rate: (Iyy - Ixx) / Izz
           "yaw_coupling": float((inertia[3] - inertia[2]) / inertia[4])}
    vec = vectoring(aircraft, inertia)
    if vec:
        out["vectoring"] = vec
        keys = [k + "_m" for gains, _ in AXES.values() for k in gains] + [p for _, p in AXES.values()] + ["p_per"]
        for k in keys:
            tables[k] = np.array([[p[k] for p in row] for row in points])
    if line is not None:
        de_lo, de_hi = (math.radians(x) for x in aircraft.channel_limits("elevator"))
        out["moment"] = {"alpha_deg": line["alpha_deg"], "slope": slope, "departure": line["departure"],
                         "elevator": moment_compensation(tabs, line, machs, de_lo, de_hi)}
    return out


# -- the JSBSim channels -------------------------------------------------------------------------
def _gain_table(name, fbw, key, indent=8, fmt="%10.5f"):
    pad = " " * indent
    g = fbw["gains"][key]
    head = pad + "          " + "".join("%10.3f" % m for m in fbw["mach"])
    body = "\n".join(pad + "%10.1f" % q + "".join(fmt % v for v in g[i]) for i, q in enumerate(fbw["qbar_psf"]))
    return """%s<fcs_function name="fcs/fbw/%s">
%s  <function>
%s    <table>
%s      <independentVar lookup="row">aero/qbar-psf</independentVar>
%s      <independentVar lookup="column">velocities/mach</independentVar>
%s      <tableData>
%s
%s
%s      </tableData>
%s    </table>
%s  </function>
%s</fcs_function>""" % (pad, name, pad, pad, pad, pad, pad, head, body, pad, pad, pad, pad)


def _moment_table(fbw, indent=8, name="moment-comp"):
    pad = " " * indent
    mo = fbw["moment"]
    head = pad + "          " + "".join("%10.3f" % m for m in fbw["mach"])
    body = "\n".join(pad + "%10.5f" % math.radians(al) + "".join("%10.5f" % v for v in mo["elevator"][i])
                     for i, al in enumerate(mo["alpha_deg"]))
    return """%s<!-- the pitching moment's departures from a straight line through the
%s     angle-of-attack envelope (%+.3f per rad about the CG), cancelled by the
%s     elevator: the gains are designed on the line -->
%s<fcs_function name="fcs/fbw/%s">
%s  <function>
%s    <table>
%s      <independentVar lookup="row">aero/alpha-rad</independentVar>
%s      <independentVar lookup="column">velocities/mach</independentVar>
%s      <tableData>
%s
%s
%s      </tableData>
%s    </table>
%s  </function>
%s</fcs_function>""" % (pad, pad, mo["slope"], pad, pad, name, pad, pad, pad, pad, pad, head, body, pad, pad, pad, pad)


POWER_NOTE = {"md": "pitch", "lda": "roll", "ndr": "yaw"}
POWER_SIGN = {"md": -1.0, "lda": 1.0, "ndr": -1.0}     # a surface's power as it normally is


def _thrust_power(fbw, key):
    """fcs/fbw/<key>-aero, the surfaces' power about one axis (a table,
    1/s^2 per rad), and fcs/fbw/<key>-total, with the vectoring nozzles' at
    the thrust the engines make - never less than 0.01 the usual way round,
    so that the gains divided by it stay finite."""
    terms = "\n".join("                <product><value>%.6e</value><max><value>0</value>"
                      "<property>propulsion/engine[%d]/thrust-lbs</property></max></product>" % (v[key], v["engine"])
                      for v in fbw["vectoring"] if v[key] != 0.0)
    s = POWER_SIGN[key]
    return """        <!-- the %s power (1/s^2 per rad of the surface): the surfaces', and with the
             vectoring nozzles' at the thrust the engines make -->
%s
        <fcs_function name="fcs/fbw/%s-total">
          <function>
            <%s>
              <sum>
                <property>fcs/fbw/%s-aero</property>
%s
              </sum>
              <value>%.2f</value>
            </%s>
          </function>
        </fcs_function>""" % (POWER_NOTE[key], _gain_table(key + "-aero", fbw, key, fmt=" %13.6e"), key,
                              "max" if s > 0 else "min", key, terms, 0.01 * s, "max" if s > 0 else "min")


def _moment_gain(name, fbw, key, power, lo, hi):
    """A gain as its moment (a table) over the power about its axis now."""
    return """%s
        <fcs_function name="fcs/fbw/%s">
          <function>
            <quotient><property>fcs/fbw/%s-m</property><property>fcs/fbw/%s-total</property></quotient>
          </function>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </fcs_function>""" % (_gain_table(name + "-m", fbw, key + "_m", fmt=" %13.6e"), name, name, power, lo, hi)


def _integrator(name, rate, trigger, lim, power=None, sign=1.0, fmt="%.5f"):
    """An integrator of a surface rate (rad/s) clipped to +-lim rad - or,
    with the power about its axis (fcs/fbw/<power>-total), of the moment
    that rate gives, clipped to the moment of +-lim rad, and its output that
    moment over the power: what it holds stays when the vectoring nozzles'
    share changes with thrust. (JSBSim clips the output, not the state: the
    anti-windup is the trigger's.)"""
    if power is None:
        return """        <integrator name="fcs/fbw/%s">
          <input>fcs/fbw/%s</input>
          <c1>1.0</c1>
          <trigger>%s</trigger>
          <clipto> <min>%s</min> <max>%s</max> </clipto>
        </integrator>""" % (name, rate, trigger, fmt % -lim, fmt % lim)
    return """        <!-- held as a moment: the nozzles' share changes with thrust, the trim does not -->
        <fcs_function name="fcs/fbw/%s-moment-rate">
          <function><product><property>fcs/fbw/%s</property><property>fcs/fbw/%s-total</property></product></function>
        </fcs_function>
        <fcs_function name="fcs/fbw/%s-limit">
          <function><product><value>%.5f</value><property>fcs/fbw/%s-total</property></product></function>
        </fcs_function>
        <integrator name="fcs/fbw/%s-moment">
          <input>fcs/fbw/%s-moment-rate</input>
          <c1>1.0</c1>
          <trigger>%s</trigger>
          <clipto> <min>-fcs/fbw/%s-limit</min> <max>fcs/fbw/%s-limit</max> </clipto>
        </integrator>
        <fcs_function name="fcs/fbw/%s">
          <function><quotient><property>fcs/fbw/%s-moment</property><property>fcs/fbw/%s-total</property></quotient></function>
        </fcs_function>""" % (name, rate, power, name, sign * lim, power, name, name, trigger, name, name, name, name, power)


def _roll_yaw_limit(aircraft, fbw, yaw_vectoring):
    """fcs/fbw/p-cmd: the stick's roll rate (fcs/fbw/p-stick) limited to
    what the rudder can coordinate. A roll about the flight path is also a
    yaw, r = p sin(alpha); what the ailerons' own yaw does not give, the
    rudder must. Its yaw acceleration at full travel Y (the design points',
    with the vectoring nozzles' at the thrust there is now) bounds:

    - the rate, to PULL_YAW of Y over the yaw each unit of roll rate needs
      while the aircraft pitches under it: the pitch rate's inertia
      coupling ((Iyy - Ixx) / Izz p q), the angle of attack rising
      (p alphadot cos(alpha)) and the yaw damping the roll's yaw rate meets;
    - the roll acceleration while the rate grows, to Y over the rudder's
      share c of the roll's yaw (roll_yaw_share). An uncoordinated roll to
      the rate p leaves at worst c^2 p^2 / (8 Y) of sideslip (rolled in at
      twice that acceleration); one that cannot leave ENTRY_SIDESLIP needs
      no limit - a small correction, or a fast roll at a low angle of
      attack, where the ailerons' yaw and a principal axis inclined to the
      flight path do most of it. A roll is stopped as fast as the ailerons
      can: the yaw damper and the weathercock take the yaw rate out.

    Without a rudder the stick's rate as it is."""
    if "rudder" not in aircraft.channels():
        return """        <pure_gain name="fcs/fbw/p-cmd">
          <input>fcs/fbw/p-stick</input>
          <gain>1</gain>
        </pure_gain>"""
    dr_max = math.radians(max(abs(x) for x in aircraft.channel_limits("rudder")))
    if yaw_vectoring:
        accel = """        <fcs_function name="fcs/fbw/yaw-accel">
          <function><product><value>%.5f</value><property>fcs/fbw/ndr-total</property></product></function>
        </fcs_function>""" % -dr_max
    else:
        accel = _gain_table("yaw-accel", fbw, "yaw_accel")
    alphas, share = fbw["roll_yaw_share"]
    rows = "\n".join("                  %9.5f %9.5f" % (math.radians(a), s) for a, s in zip(alphas, share))
    return """        <!-- the rudder's yaw acceleration at full travel (1/s^2), and the yaw damping (1/s) -->
%s
%s
        <!-- the rudder's share of the yaw a roll about the flight path needs, per unit of
             its roll acceleration: sin(alpha), less the ailerons' own yaw and what a
             principal axis inclined to the body axis gives -->
        <fcs_function name="fcs/fbw/roll-yaw-share">
          <function>
            <table>
              <independentVar lookup="row">aero/alpha-rad</independentVar>
              <tableData>
%s
              </tableData>
            </table>
          </function>
        </fcs_function>
        <!-- the yaw acceleration each rad/s of roll rate needs while the aircraft pitches
             under it: the pitch rate's inertia coupling, the angle of attack rising, and
             the yaw damping the roll's yaw rate meets -->
        <fcs_function name="fcs/fbw/p-yaw-need">
          <function>
            <abs>
              <sum>
                <product>
                  <sum>
                    <property>aero/alphadot-rad_sec</property>
                    <product><value>%.5f</value><property>velocities/q-rad_sec</property></product>
                  </sum>
                  <cos><property>aero/alpha-rad</property></cos>
                </product>
                <product><property>fcs/fbw/yaw-damping</property><sin><property>aero/alpha-rad</property></sin></product>
              </sum>
            </abs>
          </function>
        </fcs_function>
        <!-- no more roll rate than %g of the rudder's yaw holds: a roll the rudder cannot
             follow turns the angle of attack into sideslip -->
        <fcs_function name="fcs/fbw/p-yaw-max">
          <function>
            <quotient>
              <product><value>%g</value><property>fcs/fbw/yaw-accel</property></product>
              <max><value>0.001</value><property>fcs/fbw/p-yaw-need</property></max>
            </quotient>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/p-limited">
          <function>
            <max>
              <min><property>fcs/fbw/p-stick</property><property>fcs/fbw/p-yaw-max</property></min>
              <product><value>-1</value><property>fcs/fbw/p-yaw-max</property></product>
            </max>
          </function>
        </fcs_function>
        <!-- and while the rate grows, no faster roll acceleration than all of it
             coordinates, unless the roll asked for could not leave %.1f deg of sideslip
             even uncoordinated: at worst c^2 p^2 / (8 Y), c the rudder's share, Y its yaw
             acceleration, p the rate asked for -->
        <fcs_function name="fcs/fbw/p-accel">
          <function>
            <product>
              <quotient>
                <property>fcs/fbw/yaw-accel</property>
                <max><value>%g</value><abs><property>fcs/fbw/roll-yaw-share</property></abs></max>
              </quotient>
              <max>
                <value>1</value>
                <quotient>
                  <product><value>%.5f</value><property>fcs/fbw/yaw-accel</property></product>
                  <sum>
                    <value>1e-9</value>
                    <product>
                      <property>fcs/fbw/roll-yaw-share</property><property>fcs/fbw/roll-yaw-share</property>
                      <property>fcs/fbw/p-limited</property><property>fcs/fbw/p-limited</property>
                    </product>
                  </sum>
                </quotient>
              </max>
            </product>
          </function>
        </fcs_function>
        <!-- a roll is stopped as fast as the ailerons can -->
        <fcs_function name="fcs/fbw/p-accel-up">
          <function>
            <sum><property>fcs/fbw/p-accel</property>
              <product><value>1000</value><lt><property>fcs/fbw/p-cmd</property><value>0</value></lt></product></sum>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/p-accel-down">
          <function>
            <sum><property>fcs/fbw/p-accel</property>
              <product><value>1000</value><gt><property>fcs/fbw/p-cmd</property><value>0</value></gt></product></sum>
          </function>
        </fcs_function>
        <actuator name="fcs/fbw/p-cmd">
          <input>fcs/fbw/p-limited</input>
          <rate_limit sense="incr">fcs/fbw/p-accel-up</rate_limit>
          <rate_limit sense="decr">fcs/fbw/p-accel-down</rate_limit>
        </actuator>""" % (accel, _gain_table("yaw-damping", fbw, "yaw_damping"), rows, fbw["yaw_coupling"], PULL_YAW,
                          PULL_YAW, math.degrees(ENTRY_SIDESLIP), SHARE_FLOOR, 8.0 * ENTRY_SIDESLIP)


def vectoring_xml(aircraft):
    """The Thrust Vectoring channel: each vectoring nozzle's deflection from
    the surfaces' positions (vectoring()), turning its engine's thrust
    (JSBSim's propulsion/engine[i]/pitch-angle-rad and yaw-angle-rad) in its
    plane. None without vectoring nozzles."""
    vec = vectoring(aircraft)
    if not vec:
        return None
    src = (("k_elevator", 1.0, "fcs/elevator-pos-rad"), ("k_aileron", 1.0, "fcs/left-aileron-pos-rad"),
           ("k_rudder", -1.0, "fcs/rudder-pos-rad"))
    parts = ["""      <channel name="Thrust Vectoring">
        <!-- hangar: each nozzle turns with the surfaces (hangar/fcs.py): through its
             travel as the elevator goes through its own (a positive deflection
             pushes the tail up, nose down, as the elevator's does), differentially
             through half of it with the ailerons (the left nozzle's tail up rolls
             right) and, in a canted plane, with the rudder -->"""]
    for v in vec:
        i = v["engine"]
        terms = "\n".join("              <product><value>%.6f</value><property>%s</property></product>"
                          % (sign * (v["side"] if k != "k_elevator" else 1.0) * v[k], prop)
                          for k, sign, prop in src if v[k] != 0.0)
        parts.append("""        <fcs_function name="fcs/vectoring/nozzle-%d-rad">
          <function>
            <sum>
%s
            </sum>
          </function>
          <clipto> <min>%.6f</min> <max>%.6f</max> </clipto>
        </fcs_function>
        <fcs_function name="fcs/vectoring/pitch-%d-rad">
          <function>
            <sum><value>%.6f</value><product><value>%.6f</value><property>fcs/vectoring/nozzle-%d-rad</property></product></sum>
          </function>
          <output>propulsion/engine[%d]/pitch-angle-rad</output>
        </fcs_function>""" % (i, terms, -v["travel"], v["travel"], i, v["orient"][1], math.cos(v["cant"]), i, i))
        if v["cant"] > 0.0 and v["side"]:
            # canted outboard: the left nozzle's tail pushed up also goes left
            parts.append("""        <fcs_function name="fcs/vectoring/yaw-%d-rad">
          <function>
            <sum><value>%.6f</value><product><value>%.6f</value><property>fcs/vectoring/nozzle-%d-rad</property></product></sum>
          </function>
          <output>propulsion/engine[%d]/yaw-angle-rad</output>
        </fcs_function>""" % (i, v["orient"][2], -v["side"] * math.sin(v["cant"]), i, i))
    parts.append("      </channel>")
    return "\n".join(parts)


def channels_xml(aircraft, fbw):
    """The fly-by-wire Pitch, Roll and Yaw channels (JSBSim <channel>s) with
    their gain tables, writing the same surface positions the direct
    channels do."""
    o = fbw["options"]
    rad = math.radians
    de_lo, de_hi = (rad(x) for x in aircraft.channel_limits("elevator"))
    dr_max = rad(max(abs(x) for x in aircraft.channel_limits("rudder"))) if "rudder" in aircraft.channels() else 0.0
    parts = []
    hold = """        <!-- the integrators are reset below 60 kt (on the ground, taking off) -->
        <switch name="fcs/fbw/reset">
          <default value="0"/>
          <test value="-1">
            velocities/vc-kts lt 60
          </test>
        </switch>"""
    # thrust vectoring: the axes the nozzles add power about, their gains as moments
    # over the power there is now
    vec = fbw.get("vectoring") or []
    need = {"pitch": "elevator", "roll": "aileron", "yaw": "rudder"}
    powered = {axis: need[axis] in aircraft.channels() and any(v[p] != 0.0 for v in vec) for axis, (_, p) in AXES.items()}
    limits = dict(LIMITS, k_pedal=(0.0, dr_max))
    for axis, (_, p) in AXES.items():
        if powered[axis]:
            hold += "\n" + _thrust_power(fbw, p)

    def gain(name, key):
        axis = next(a for a, (keys, _) in AXES.items() if key in keys)
        if not powered[axis]:
            return _gain_table(name, fbw, key)
        lim = limits[key]
        lo, hi = lim if isinstance(lim, tuple) else (-lim, lim)
        return _moment_gain(name, fbw, key, AXES[axis][1], lo, hi)
    moment = _moment_table(fbw)
    if powered["pitch"]:
        # the elevator the departures ask for, with the nozzles turning with it
        moment = _moment_table(fbw, name="moment-comp-aero") + """
        <fcs_function name="fcs/fbw/moment-comp">
          <function>
            <product><property>fcs/fbw/moment-comp-aero</property>
              <quotient><property>fcs/fbw/md-aero</property><property>fcs/fbw/md-total</property></quotient></product>
          </function>
        </fcs_function>"""
    parts.append("""      <channel name="Pitch (fly-by-wire)">
        <!-- hangar: load factor command, alpha and pitch-rate inner loop (hangar/fcs.py) -->
        <summer name="fcs/pitch-trim-sum">
          <input>fcs/elevator-cmd-norm</input>
          <input>fcs/pitch-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
%s
%s
%s
%s
%s
%s
%s
        <!-- the gravity reference: the load factor that holds the flight path -->
        <fcs_function name="fcs/fbw/g-ref">
          <function>
            <product>
              <cos><property>attitude/theta-rad</property></cos>
              <cos><property>attitude/phi-rad</property></cos>
            </product>
          </function>
        </fcs_function>
        <!-- stick to load factor beyond the gravity reference, aft (negative) to n_max,
             no more than the lift at the angle-of-attack limits gives - beyond the
             gravity reference of the moment, so that full aft stick still reaches
             the limit in a steep climb or inverted - through a 0.2 s prefilter and
             a 12 g/s onset limit (a full pull at once pitches an agile airframe
             faster than its nose-down control can stop at the angle-of-attack
             limit) -->
        <fcs_function name="fcs/fbw/dn-stick">
          <function>
            <max>
              <min>
                <table>
                  <independentVar lookup="row">fcs/pitch-trim-sum</independentVar>
                  <tableData>
                    -1.0  %.3f
                     0.0  0.0
                     1.0  %.3f
                  </tableData>
                </table>
                <difference><product><property>fcs/fbw/n-alpha</property><value>%.5f</value></product><property>fcs/fbw/g-ref</property></difference>
              </min>
              <difference><product><property>fcs/fbw/n-alpha</property><value>%.5f</value></product><property>fcs/fbw/g-ref</property></difference>
            </max>
          </function>
        </fcs_function>
        <lag_filter name="fcs/fbw/dn-lag">
          <input>fcs/fbw/dn-stick</input>
          <c1>5.0</c1>
        </lag_filter>
        <actuator name="fcs/fbw/dn-cmd">
          <input>fcs/fbw/dn-lag</input>
          <rate_limit>12.0</rate_limit>
        </actuator>
        <!-- the response the command should produce (the short period's rise): the
             integrator trims away what differs from it, not the rise itself -->
        <lag_filter name="fcs/fbw/dn-model">
          <input>fcs/fbw/dn-cmd</input>
          <c1>2.5</c1>
        </lag_filter>
        <fcs_function name="fcs/fbw/dn">
          <function>
            <difference><property>accelerations/Nz</property><property>fcs/fbw/g-ref</property></difference>
          </function>
        </fcs_function>
        <!-- the angle of attack with its rise over the next 0.35 s counted as spent
             (the rise, not the pitch rate: in a steady pull the nose turns with the
             flight path and the angle of attack holds), and the load factor the
             angle of attack left to each limit gives -->
        <fcs_function name="fcs/fbw/alpha-ahead">
          <function>
            <sum>
              <property>aero/alpha-rad</property>
              <product><value>0.35</value><property>aero/alphadot-rad_sec</property></product>
            </sum>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/dn-room-up">
          <function>
            <product><property>fcs/fbw/n-alpha</property>
              <difference><value>%.5f</value><property>fcs/fbw/alpha-ahead</property></difference></product>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/dn-room-down">
          <function>
            <product><property>fcs/fbw/n-alpha</property>
              <difference><value>%.5f</value><property>fcs/fbw/alpha-ahead</property></difference></product>
          </function>
        </fcs_function>
        <!-- the command limited to the load factor the aircraft pulls now plus what
             the angle of attack left gives: at a limit the feedforward asks for the
             load factor the airframe gives there, whatever elevator holds it - a
             steady pull on a stable airframe, a push on an unstable one (a
             feedforward faded out past the limit instead took about 10 deg of
             elevator per deg of alpha, on one side of it only: the actuator's rate
             limit could not follow, and the MiG-29A cycled between 24 and 29 deg) -->
        <fcs_function name="fcs/fbw/dn-limited">
          <function>
            <max>
              <min>
                <property>fcs/fbw/dn-cmd</property>
                <sum><property>fcs/fbw/dn</property><property>fcs/fbw/dn-room-up</property></sum>
              </min>
              <sum><property>fcs/fbw/dn</property><property>fcs/fbw/dn-room-down</property></sum>
            </max>
          </function>
        </fcs_function>
        <!-- the integrator's error: the load factor's departure from the model
             response, and at a limit the angle of attack left, counted five times -
             it trims what the airframe needs there beyond the linear design (vortex
             lift, the tail's power at high alpha) in a fraction of a second, not
             over the pull -->
        <fcs_function name="fcs/fbw/pitch-error">
          <function>
            <max>
              <min>
                <difference><property>fcs/fbw/dn-model</property><property>fcs/fbw/dn</property></difference>
                <product><value>%.1f</value><property>fcs/fbw/dn-room-up</property></product>
              </min>
              <product><value>%.1f</value><property>fcs/fbw/dn-room-down</property></product>
            </max>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/pitch-error-rate">
          <function><product><property>fcs/fbw/k-i</property><property>fcs/fbw/pitch-error</property></product></function>
        </fcs_function>
        <!-- the integrator: reset below 60 kt, held while the elevator is at a stop
             it would push further into (anti-windup) -->
        <switch name="fcs/fbw/pitch-hold">
          <default value="0"/>
          <test value="-1">
            velocities/vc-kts lt 60
          </test>
          <test logic="AND" value="1">
            fcs/fbw/elevator-raw gt %.5f
            fcs/fbw/pitch-error-rate gt 0
          </test>
          <test logic="AND" value="1">
            fcs/fbw/elevator-raw lt %.5f
            fcs/fbw/pitch-error-rate lt 0
          </test>
        </switch>
%s
        <!-- past an angle-of-attack limit (the next 0.35 s counted) a push back
             in proportion, all the travel 4 deg beyond: an unstable airframe
             pitching up fast needs it at once, not when an integrator winds -->
        <fcs_function name="fcs/fbw/alpha-push">
          <function>
            <difference>
              <product><value>%.5f</value>
                <max><value>0</value><difference><property>fcs/fbw/alpha-ahead</property><value>%.5f</value></difference></max></product>
              <product><value>%.5f</value>
                <max><value>0</value><difference><value>%.5f</value><property>fcs/fbw/alpha-ahead</property></difference></max></product>
            </difference>
          </function>
        </fcs_function>
        <!-- the elevator: the push, the moment compensation, angle-of-attack and
             pitch-rate feedback, the feedforward of the limited command, the
             integrator -->
        <fcs_function name="fcs/fbw/elevator-raw">
          <function>
            <sum>
              <property>fcs/fbw/alpha-push</property>
              <property>fcs/fbw/moment-comp</property>
              <product><value>-1</value><property>fcs/fbw/k-alpha</property><property>aero/alpha-rad</property></product>
              <product><value>-1</value><property>fcs/fbw/k-q</property><property>velocities/q-rad_sec</property></product>
              <product><property>fcs/fbw/k-ff</property><property>fcs/fbw/dn-limited</property></product>
              <property>fcs/fbw/pitch-integral</property>
            </sum>
          </function>
        </fcs_function>
        <pure_gain name="fcs/fbw/elevator">
          <input>fcs/fbw/elevator-raw</input>
          <gain>1.0</gain>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </pure_gain>
        <!-- full travel in 0.83 s: 60 deg/s for a +-25 deg tail, faster for a
             canard's longer throw -->
        <actuator name="fcs/elevator-actuator">
          <input>fcs/fbw/elevator</input>
          <lag>40</lag>
          <rate_limit>%.3f</rate_limit>
          <output>fcs/elevator-pos-rad</output>
        </actuator>
      </channel>""" % (hold, gain("k-alpha", "k_alpha"), gain("k-q", "k_q"), gain("k-ff", "k_ff"), gain("k-i", "k_i"),
                       _gain_table("n-alpha", fbw, "n_alpha"), moment,
                       o["n_max"] - 1.0, o["n_min"] - 1.0, rad(o["alpha_max_deg"]), rad(o["alpha_min_deg"]),
                       rad(o["alpha_max_deg"]), rad(o["alpha_min_deg"]), LIMIT_INTEGRAL, LIMIT_INTEGRAL, de_hi, de_lo,
                       _integrator("pitch-integral", "pitch-error-rate", "fcs/fbw/pitch-hold", 0.5,
                                   "md" if powered["pitch"] else None, POWER_SIGN["md"], fmt="%g"),
                       de_hi / rad(4.0), rad(o["alpha_max_deg"]), -de_lo / rad(4.0), rad(o["alpha_min_deg"]),
                       de_lo, de_hi, max(1.05, (de_hi - de_lo) / 0.83)))
    if "aileron" in aircraft.channels():
        da = rad(max(abs(x) for x in aircraft.channel_limits("aileron")))
        p_max = _gain_table("p-max", fbw, "p_max")
        if powered["roll"]:
            # the rate asked for, or what the rolling moment there is now gives
            p_max = _gain_table("p-per", fbw, "p_per", fmt=" %13.6e") + """
        <fcs_function name="fcs/fbw/p-max">
          <function>
            <min>
              <value>%.5f</value>
              <product><property>fcs/fbw/p-per</property><property>fcs/fbw/lda-total</property></product>
            </min>
          </function>
        </fcs_function>""" % rad(o["roll_rate_deg_s"])
        parts.append("""      <channel name="Roll (fly-by-wire)">
        <!-- hangar: roll-rate command about the flight path, limited to what the rudder can
             coordinate; bank hold at neutral stick -->
        <summer name="fcs/roll-trim-sum">
          <input>fcs/aileron-cmd-norm</input>
          <input>fcs/roll-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
%s
%s
%s
%s
        <!-- the stick's rate halves towards alpha_max -->
        <fcs_function name="fcs/fbw/p-stick">
          <function>
            <product>
              <property>fcs/roll-trim-sum</property>
              <property>fcs/fbw/p-max</property>
              <table>
                <independentVar lookup="row">aero/alpha-rad</independentVar>
                <tableData>
                  %.5f 1.0
                  %.5f 0.5
                </tableData>
              </table>
            </product>
          </function>
        </fcs_function>
%s
        <fcs_function name="fcs/fbw/p-stability">
          <function>
            <sum>
              <product><property>velocities/p-rad_sec</property><cos><property>aero/alpha-rad</property></cos></product>
              <product><property>velocities/r-rad_sec</property><sin><property>aero/alpha-rad</property></sin></product>
            </sum>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/roll-error-rate">
          <function>
            <product><property>fcs/fbw/k-roll-i</property>
              <difference><property>fcs/fbw/p-cmd</property><property>fcs/fbw/p-stability</property></difference></product>
          </function>
        </fcs_function>
%s
        <fcs_function name="fcs/fbw/aileron">
          <function>
            <sum>
              <product><property>fcs/fbw/k-roll-ff</property><property>fcs/fbw/p-cmd</property></product>
              <product><property>fcs/fbw/k-roll</property>
                <difference><property>fcs/fbw/p-cmd</property><property>fcs/fbw/p-stability</property></difference></product>
              <property>fcs/fbw/roll-integral</property>
            </sum>
          </function>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </fcs_function>
        <actuator name="fcs/left-aileron-actuator">
          <input>fcs/fbw/aileron</input>
          <lag>40</lag>
          <rate_limit>1.4</rate_limit>
          <output>fcs/left-aileron-pos-rad</output>
        </actuator>
        <pure_gain name="fcs/right-aileron">
          <input>fcs/left-aileron-pos-rad</input>
          <gain>-1</gain>
          <output>fcs/right-aileron-pos-rad</output>
        </pure_gain>
      </channel>""" % (gain("k-roll", "k_roll"), gain("k-roll-ff", "k_roll_ff"), gain("k-roll-i", "k_roll_i"),
                       p_max, rad(o["alpha_max_deg"] * 0.4), rad(o["alpha_max_deg"]),
                       _roll_yaw_limit(aircraft, fbw, powered["yaw"]),
                       _integrator("roll-integral", "roll-error-rate", "fcs/fbw/reset", 0.5 * da,
                                   "lda" if powered["roll"] else None, POWER_SIGN["lda"]), -da, da))
    if "rudder" in aircraft.channels():
        dr = rad(max(abs(x) for x in aircraft.channel_limits("rudder")))
        parts.append("""      <channel name="Yaw (fly-by-wire)">
        <!-- hangar: yaw damper, sideslip feedback, pedals command sideslip -->
        <summer name="fcs/yaw-trim-sum">
          <input>fcs/rudder-cmd-norm</input>
          <input>fcs/yaw-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
%s
%s
%s
        <!-- the yaw damper damps the sideslip's rate, the stability-axis yaw rate less
             the one that coordinates the bank and the side force: in the dutch roll the
             yaw rate itself, and zero in a steady coordinated turn, in a roll about the
             flight path and in a steady sideslip - it coordinates those as long as they
             last (a damper on the washed-out yaw rate stopped coordinating a sustained
             roll, and at low speed the roll turned the angle of attack into sideslip) -->
        <fcs_function name="fcs/fbw/rudder">
          <function>
            <sum>
              <product><property>fcs/fbw/k-pedal</property><property>fcs/yaw-trim-sum</property></product>
              <product><value>-1</value><property>fcs/fbw/k-yaw-beta</property>
                <difference><property>aero/beta-rad</property>
                  <product><value>%.5f</value><property>fcs/yaw-trim-sum</property></product></difference></product>
              <product><property>fcs/fbw/k-yaw-r</property><property>aero/betadot-rad_sec</property></product>
            </sum>
          </function>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </fcs_function>
        <actuator name="fcs/rudder-actuator">
          <input>fcs/fbw/rudder</input>
          <lag>40</lag>
          <rate_limit>1.4</rate_limit>
          <output>fcs/rudder-pos-rad</output>
        </actuator>
        <!-- the nose wheel steers with the pedals (JSBSim steers from fcs/steer-cmd-norm,
             positive right; the rudder command is positive left) -->
        <pure_gain name="fcs/steer-from-rudder">
          <input>fcs/yaw-trim-sum</input>
          <gain>-1</gain>
          <output>fcs/steer-cmd-norm</output>
        </pure_gain>
      </channel>""" % (gain("k-yaw-r", "k_yaw_r"), gain("k-yaw-beta", "k_yaw_beta"), gain("k-pedal", "k_pedal"),
                       rad(o["sideslip_deg"]), -dr, dr))
    return parts
