"""The linear small-perturbation model about trimmed level flight, from the
tables, and its modes (Etkin & Reid, "Dynamics of Flight", ch. 4 and 5;
Nelson, "Flight Stability and Automatic Control", ch. 4 and 5).

Longitudinal states (u, alpha, q, theta), lateral (beta, p, r, phi) in body
axes, with the trim angle of attack's kinematic terms (lateral_matrix).
Derivatives come from the tables at the trim angle of attack
and are moved from the aerodynamic reference point to the centre of gravity;
the propeller's thrust varies with speed at constant power. This is the
prediction the handling-qualities checks use; the flight tests identify the
same modes from JSBSim's own response as a cross-check.
"""
import math

import numpy as np

from .aero import tables as T

G0 = 9.80665


def atmosphere(h):
    T0, p0, L = 288.15, 101325.0, 0.0065
    t = T0 - L * min(h, 11000.0)
    return p0 * (t / T0) ** 5.2559 / (287.053 * t)


def loaded_inertia(mass_model):
    """The loaded aircraft's inertia about its CG (payload and fuel as point
    masses, as JSBSim adds them)."""
    e = mass_model.empty()
    m, cg = mass_model.loaded()
    J = np.zeros((3, 3))
    items = [(e["mass"], e["cg"])] + [(p["mass"], np.asarray(p["position"], float)) for p in mass_model.payload] + \
        [(mass_model.fuel(t), np.asarray(t["position"], float)) for t in mass_model.tanks]
    for mi, p in items:
        r = p - cg
        J += mi * np.outer(r, r)
    Ixx = e["ixx"] + J[1, 1] + J[2, 2]
    Iyy = e["iyy"] + J[0, 0] + J[2, 2]
    Izz = e["izz"] + J[0, 0] + J[1, 1]
    Ixz = e["ixz"] + J[0, 2]
    return m, cg, Ixx, Iyy, Izz, Ixz


def trim_alpha(tabs, CL):
    a = tabs["alpha"]
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    k = (a > -10) & (a < 20)
    cl = tabs["base"]["CL"][k, j0]
    i = int(np.argmax(cl))
    return float(np.interp(CL, cl[: i + 1], a[k][: i + 1]))


def model(tabs, aircraft, mass_model, speed, altitude=1500.0):
    """A_long, A_lat and the trim numbers."""
    a = aircraft
    rho = atmosphere(altitude)
    m, cg, Ixx, Iyy, Izz, Ixz = loaded_inertia(mass_model)
    Q = 0.5 * rho * speed * speed
    S, b, c = a.S, a.b, a.c
    W = m * G0
    CL0 = W / (Q * S)
    a0 = trim_alpha(tabs, CL0)
    d = T.derivatives(tabs, alpha_deg=a0)
    j0 = int(np.argmin(np.abs(tabs["beta"])))
    CD0 = float(np.interp(a0, tabs["alpha"], tabs["base"]["CD"][:, j0]))
    # moments about the CG, not the reference point
    dx = (cg[0] - a.aero_point[0])
    dz = (a.aero_point[2] - cg[2])
    Cma = d["Cma"] + d["CLa"] * dx / c
    Cmq = d["Cmq"] + d["CLq"] * dx / c
    Cnb = d["Cnb"] + d["CYb"] * dx / b
    Clb = d["Clb"] + d["CYb"] * dz / b
    Cnr = d["Cnr"] + d["CYr"] * dx / b
    Clp, Cnp, Clr = d["Clp"], d["Cnp"], d["Clr"]
    U = speed
    # longitudinal (thrust = drag in trim, constant power: dT/du = -T/U)
    D0 = CD0 * Q * S
    Xu = (-2 * CD0 * Q * S - D0) / (m * U)
    Xa = -(d["CDa"] - CL0) * Q * S / m
    Zu = -2 * CL0 * Q * S / (m * U)
    Za = -(d["CLa"] + CD0) * Q * S / m
    Zq = -d["CLq"] * Q * S * c / (2 * m * U)
    Zad = -d["CLad"] * Q * S * c / (2 * m * U)
    Ma = Cma * Q * S * c / Iyy
    Mq = Cmq * Q * S * c * c / (2 * Iyy * U)
    Mad = d["Cmad"] * Q * S * c * c / (2 * Iyy * U)
    k = 1.0 / (1.0 - Zad / U)
    ad_u, ad_a, ad_q = k * Zu / U, k * Za / U, k * (1 + Zq / U)
    A_lon = np.array([
        [Xu, Xa, 0.0, -G0],
        [ad_u, ad_a, ad_q, 0.0],
        [Mad * ad_u, Ma + Mad * ad_a, Mq + Mad * ad_q, 0.0],
        [0.0, 0.0, 1.0, 0.0]])
    der = {"CYb": d["CYb"], "CYp": d["CYp"], "CYr": d["CYr"], "Clb": Clb, "Clp": Clp, "Clr": Clr,
           "Cnb": Cnb, "Cnp": Cnp, "Cnr": Cnr}
    A_lat = lateral_matrix(der, Q * S, b, m, U, (Ixx, Izz, Ixz), math.radians(a0))
    return {"A_lon": A_lon, "A_lat": A_lat, "alpha_trim_deg": a0, "CL": CL0, "speed_ms": speed, "altitude_m": altitude,
            "Cma_cg": Cma, "Cnb_cg": Cnb, "Clb_cg": Clb, "inertia": [Ixx, Iyy, Izz, Ixz], "mass": m}


def lateral_matrix(d, QS, b, m, U, inertia, alpha0):
    """The lateral small-perturbation matrix, states (beta, p, r, phi), in
    body axes - the tables' derivatives and the inertia are body-axis ones -
    about level flight at angle of attack alpha0 (rad): the trim velocity
    has a component along the body's z axis, so a roll rate turns it into
    sideslip (sin alpha0 p in beta's rate, Etkin & Reid 4.9) and a yaw rate
    tilts the bank (phi' = p + tan theta0 r). Where a swept wing's large
    dihedral effect rolls the aircraft through its dutch roll these terms
    matter: at 4.5 deg they take a transport's predicted damping from a third
    of what JSBSim's own response shows to what it shows. The inertia product
    enters through primed derivatives. d: CYb, CYp, CYr, Clb, Clp, Clr, Cnb,
    Cnp, Cnr (per rad, rates as p b / 2U); inertia: Ixx, Izz, Ixz."""
    Ixx, Izz, Ixz = inertia
    Yb = d["CYb"] * QS / m
    Yp = d["CYp"] * QS * b / (2 * m * U)
    Yr = d["CYr"] * QS * b / (2 * m * U)
    Lb, Lp, Lr = (d["Clb"] * QS * b / Ixx, d["Clp"] * QS * b * b / (2 * Ixx * U), d["Clr"] * QS * b * b / (2 * Ixx * U))
    Nb, Np, Nr = (d["Cnb"] * QS * b / Izz, d["Cnp"] * QS * b * b / (2 * Izz * U), d["Cnr"] * QS * b * b / (2 * Izz * U))
    den = 1.0 - Ixz * Ixz / (Ixx * Izz)
    Lb_, Lp_, Lr_ = ((Lb + Ixz / Ixx * Nb) / den, (Lp + Ixz / Ixx * Np) / den, (Lr + Ixz / Ixx * Nr) / den)
    Nb_, Np_, Nr_ = ((Nb + Ixz / Izz * Lb) / den, (Np + Ixz / Izz * Lp) / den, (Nr + Ixz / Izz * Lr) / den)
    sa, ca = math.sin(alpha0), math.cos(alpha0)   # level flight: theta0 = alpha0
    return np.array([
        [Yb / U, Yp / U + sa, Yr / U - ca, G0 * ca / U],
        [Lb_, Lp_, Lr_, 0.0],
        [Nb_, Np_, Nr_, 0.0],
        [0.0, 1.0, sa / ca, 0.0]])


def modes(lin):
    from .flight import longitudinal_modes, modes_from_eigs
    el = np.linalg.eigvals(lin["A_lon"])
    sp, ph = longitudinal_modes(list(el))
    ea = np.linalg.eigvals(lin["A_lat"])
    cplx = [e for e in ea if e.imag > 1e-6]
    real = sorted([e.real for e in ea if abs(e.imag) <= 1e-6])
    dr = modes_from_eigs([cplx[0]])[0] if cplx else {"omega_n": float("nan"), "zeta": float("nan"), "period_s": float("nan")}
    roll = {"time_constant_s": float(-1.0 / real[0]) if real and real[0] < 0 else float("nan")}
    s = real[-1] if len(real) > 1 else float("nan")
    spiral = {"eigenvalue": float(s), "time_to_double_s": float(math.log(2) / s) if np.isfinite(s) and s > 0 else float("inf")}
    return {"short_period": sp, "phugoid": ph, "dutch_roll": dr, "roll": roll, "spiral": spiral,
            "long_eigs": [complex(e) for e in el], "lat_eigs": [complex(e) for e in ea]}
