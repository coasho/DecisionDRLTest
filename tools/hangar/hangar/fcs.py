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
  of attack limits gives, through a 0.2 s prefilter and a 12 g/s onset
  limit. An inner loop feeds back angle of
  attack and pitch rate to place the short period - frequency from the
  Control Anticipation Parameter (CAP 1, the middle of MIL-F-8785C's level 1
  for category A), damping 0.8 - which makes an unstable airframe fly like a
  stable one; a feedforward gives the command, an integrator on the error
  from the response it should give (a 0.4 s lag of the command) trims.
  Approaching alpha_max the error is limited by the angle of attack left
  (counting its rise over the next 0.35 s as spent), so the aircraft cannot
  be pulled past it; past it a push back in proportion gives all the
  nose-down travel 4 deg beyond, so an unstable airframe pitching up fast
  is caught at once. The pitching moment's departures from a straight line
  through the angle-of-attack envelope (a tail in the wing's wake, vortex
  lift, a break) are cancelled by the elevator from a table over angle of
  attack and Mach number, and the gains are designed on the line: an
  airframe unstable in one band of angle of attack and stable in the next
  flies as the design expects through both;
- roll: a roll-rate command about the flight path (stability axes), the
  roll mode's time constant placed at 0.2 s, the rate limited by the
  rolling moment available; an integrator holds the bank angle at neutral
  stick;
- yaw: a yaw damper on the washed-out stability-axis yaw rate (which also
  coordinates rolls at angle of attack) and sideslip feedback where the
  airframe's own weathercock stability is short of a dutch roll of 1.5 rad/s
  at damping 0.5; the pedals command sideslip.

Gains are tables over dynamic pressure and Mach number, from pole placement
on the linear model (linear.py's derivatives with the compressibility
factors, as the JSBSim file applies them) at 1 g trim at each of those
conditions.
"""
import math

import numpy as np

from .aero import tables as T
from .linear import G0, loaded_inertia

PSF = 47.880259
QBAR_PSF = (10.0, 20.0, 40.0, 80.0, 150.0, 300.0, 600.0, 1200.0, 2400.0)
MACH = (0.15, 0.4, 0.7, 0.9, 1.1, 1.4, 2.0)
A_SOUND = 320.0          # m/s: the speed of sound at mid altitudes, to turn Mach into speed

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
    return {"qbar_psf": Q / PSF, "mach": mach, "speed_ms": V, "alpha_deg": alpha, "n_alpha": n_alpha,
            "k_alpha": float(ka), "k_q": float(kq), "k_ff": float(k_ff), "k_i": float(k_i),
            "omega_sp": omega, "eig_sp": [complex(e) for e in eig_sp], "open_loop_Ma": Ma,
            "k_roll": float(kp), "k_roll_ff": float(kff), "k_roll_i": float(kip), "p_max": p_max,
            "k_yaw_r": float(kr), "k_yaw_beta": float(kb), "k_pedal": kped}


GAINS = ("k_alpha", "k_q", "k_ff", "k_i", "n_alpha", "k_roll", "k_roll_ff", "k_roll_i", "p_max", "k_yaw_r", "k_yaw_beta",
         "k_pedal")
LIMITS = {"k_alpha": 6.0, "k_q": 3.0, "k_ff": 0.3, "k_i": 0.6, "k_roll": 1.5, "k_roll_ff": 1.5, "k_roll_i": 1.0,
          "k_yaw_r": 3.0, "k_yaw_beta": 3.0}


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
    out = {"qbar_psf": np.array(QBAR_PSF), "mach": np.array(machs), "gains": tables, "options": opt, "points": points}
    if line is not None:
        de_lo, de_hi = (math.radians(x) for x in aircraft.channel_limits("elevator"))
        out["moment"] = {"alpha_deg": line["alpha_deg"], "slope": slope, "departure": line["departure"],
                         "elevator": moment_compensation(tabs, line, machs, de_lo, de_hi)}
    return out


# -- the JSBSim channels -------------------------------------------------------------------------
def _gain_table(name, fbw, key, indent=8):
    pad = " " * indent
    g = fbw["gains"][key]
    head = pad + "          " + "".join("%10.3f" % m for m in fbw["mach"])
    body = "\n".join(pad + "%10.1f" % q + "".join("%10.5f" % v for v in g[i]) for i, q in enumerate(fbw["qbar_psf"]))
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


def _moment_table(fbw, indent=8):
    pad = " " * indent
    mo = fbw["moment"]
    head = pad + "          " + "".join("%10.3f" % m for m in fbw["mach"])
    body = "\n".join(pad + "%10.5f" % math.radians(al) + "".join("%10.5f" % v for v in mo["elevator"][i])
                     for i, al in enumerate(mo["alpha_deg"]))
    return """%s<!-- the pitching moment's departures from a straight line through the
%s     angle-of-attack envelope (%+.3f per rad about the CG), cancelled by the
%s     elevator: the gains are designed on the line -->
%s<fcs_function name="fcs/fbw/moment-comp">
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
%s</fcs_function>""" % (pad, pad, mo["slope"], pad, pad, pad, pad, pad, pad, pad, head, body, pad, pad, pad, pad)


def channels_xml(aircraft, fbw):
    """The fly-by-wire Pitch, Roll and Yaw channels (JSBSim <channel>s) with
    their gain tables, writing the same surface positions the direct
    channels do."""
    o = fbw["options"]
    rad = math.radians
    de_lo, de_hi = (rad(x) for x in aircraft.channel_limits("elevator"))
    parts = []
    hold = """        <!-- the integrators are reset below 60 kt (on the ground, taking off) -->
        <switch name="fcs/fbw/reset">
          <default value="0"/>
          <test value="-1">
            velocities/vc-kts lt 60
          </test>
        </switch>"""
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
        <!-- stick to load factor beyond the gravity reference, aft (negative) to n_max,
             no more than the lift at the angle-of-attack limits gives, through a
             0.2 s prefilter and a 12 g/s onset limit (a full pull at once pitches
             an agile airframe faster than its nose-down control can stop at
             the angle-of-attack limit) -->
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
                <difference><product><property>fcs/fbw/n-alpha</property><value>%.5f</value></product><value>1.0</value></difference>
              </min>
              <difference><product><property>fcs/fbw/n-alpha</property><value>%.5f</value></product><value>1.0</value></difference>
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
            <difference>
              <property>accelerations/Nz</property>
              <product>
                <cos><property>attitude/theta-rad</property></cos>
                <cos><property>attitude/phi-rad</property></cos>
              </product>
            </difference>
          </function>
        </fcs_function>
        <!-- the load factor error, limited by the angle of attack left - with its
             rise over the next 0.35 s counted as spent, so the limit is not
             overshot (the rise, not the pitch rate: in a steady pull the nose
             turns with the flight path and the angle of attack holds) -->
        <fcs_function name="fcs/fbw/alpha-ahead">
          <function>
            <sum>
              <property>aero/alpha-rad</property>
              <product><value>0.35</value><property>aero/alphadot-rad_sec</property></product>
            </sum>
          </function>
        </fcs_function>
        <fcs_function name="fcs/fbw/pitch-error">
          <function>
            <max>
              <min>
                <difference><property>fcs/fbw/dn-model</property><property>fcs/fbw/dn</property></difference>
                <product><property>fcs/fbw/n-alpha</property>
                  <difference><value>%.5f</value><property>fcs/fbw/alpha-ahead</property></difference></product>
              </min>
              <product><property>fcs/fbw/n-alpha</property>
                <difference><value>%.5f</value><property>fcs/fbw/alpha-ahead</property></difference></product>
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
        <integrator name="fcs/fbw/pitch-integral">
          <input>fcs/fbw/pitch-error-rate</input>
          <c1>1.0</c1>
          <trigger>fcs/fbw/pitch-hold</trigger>
          <clipto> <min>-0.5</min> <max>0.5</max> </clipto>
        </integrator>
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
        <!-- the feedforward (of the limited command) fades out past alpha_max, with
             the angle of attack's next 0.35 s counted: an unstable airframe
             pitching up fast needs its nose-down control before it gets there -->
        <fcs_function name="fcs/fbw/elevator-raw">
          <function>
            <sum>
              <property>fcs/fbw/alpha-push</property>
              <property>fcs/fbw/moment-comp</property>
              <product><value>-1</value><property>fcs/fbw/k-alpha</property><property>aero/alpha-rad</property></product>
              <product><value>-1</value><property>fcs/fbw/k-q</property><property>velocities/q-rad_sec</property></product>
              <product><property>fcs/fbw/k-ff</property><property>fcs/fbw/dn-cmd</property>
                <table>
                  <independentVar lookup="row">fcs/fbw/alpha-ahead</independentVar>
                  <tableData>
                    %.5f 0.0
                    %.5f 1.0
                    %.5f 1.0
                    %.5f 0.0
                  </tableData>
                </table>
              </product>
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
      </channel>""" % (hold, _gain_table("k-alpha", fbw, "k_alpha"), _gain_table("k-q", fbw, "k_q"),
                       _gain_table("k-ff", fbw, "k_ff"), _gain_table("k-i", fbw, "k_i"),
                       _gain_table("n-alpha", fbw, "n_alpha"), _moment_table(fbw),
                       o["n_max"] - 1.0, o["n_min"] - 1.0, rad(o["alpha_max_deg"]), rad(o["alpha_min_deg"]),
                       rad(o["alpha_max_deg"]), rad(o["alpha_min_deg"]), de_hi, de_lo,
                       de_hi / rad(4.0), rad(o["alpha_max_deg"]), -de_lo / rad(4.0), rad(o["alpha_min_deg"]),
                       rad(o["alpha_min_deg"] - 5.0), rad(o["alpha_min_deg"]), rad(o["alpha_max_deg"]),
                       rad(o["alpha_max_deg"] + 5.0), de_lo, de_hi, max(1.05, (de_hi - de_lo) / 0.83)))
    if "aileron" in aircraft.channels():
        da = rad(max(abs(x) for x in aircraft.channel_limits("aileron")))
        parts.append("""      <channel name="Roll (fly-by-wire)">
        <!-- hangar: roll-rate command about the flight path, bank hold at neutral stick -->
        <summer name="fcs/roll-trim-sum">
          <input>fcs/aileron-cmd-norm</input>
          <input>fcs/roll-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
%s
%s
%s
%s
        <!-- the commanded rate halves towards alpha_max -->
        <fcs_function name="fcs/fbw/p-cmd">
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
        <integrator name="fcs/fbw/roll-integral">
          <input>fcs/fbw/roll-error-rate</input>
          <c1>1.0</c1>
          <trigger>fcs/fbw/reset</trigger>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </integrator>
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
      </channel>""" % (_gain_table("k-roll", fbw, "k_roll"), _gain_table("k-roll-ff", fbw, "k_roll_ff"),
                       _gain_table("k-roll-i", fbw, "k_roll_i"), _gain_table("p-max", fbw, "p_max"),
                       rad(o["alpha_max_deg"] * 0.4), rad(o["alpha_max_deg"]), -0.5 * da, 0.5 * da, -da, da))
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
        <fcs_function name="fcs/fbw/r-stability">
          <function>
            <difference>
              <product><property>velocities/r-rad_sec</property><cos><property>aero/alpha-rad</property></cos></product>
              <product><property>velocities/p-rad_sec</property><sin><property>aero/alpha-rad</property></sin></product>
            </difference>
          </function>
        </fcs_function>
        <washout_filter name="fcs/fbw/r-washout">
          <input>fcs/fbw/r-stability</input>
          <c1>1.0</c1>
        </washout_filter>
        <fcs_function name="fcs/fbw/rudder">
          <function>
            <sum>
              <product><property>fcs/fbw/k-pedal</property><property>fcs/yaw-trim-sum</property></product>
              <product><value>-1</value><property>fcs/fbw/k-yaw-beta</property>
                <difference><property>aero/beta-rad</property>
                  <product><value>%.5f</value><property>fcs/yaw-trim-sum</property></product></difference></product>
              <product><value>-1</value><property>fcs/fbw/k-yaw-r</property><property>fcs/fbw/r-washout</property></product>
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
      </channel>""" % (_gain_table("k-yaw-r", fbw, "k_yaw_r"), _gain_table("k-yaw-beta", fbw, "k_yaw_beta"),
                       _gain_table("k-pedal", fbw, "k_pedal"), rad(o["sideslip_deg"]), -dr, dr))
    return parts
