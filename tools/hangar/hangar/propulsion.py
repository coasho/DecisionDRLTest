"""Propellers and engines.

A fixed-pitch propeller's thrust and power coefficients over advance ratio
come from blade-element momentum theory (Glauert; with Prandtl's tip and
hub losses), solved for the induced velocities directly so it holds from the
static case (J = 0) through zero thrust into windmilling. The blade sections
use the same full-circle section polars as the wings (section.py), so a
blade stalled at low advance ratio behaves like one.

Blade geometry, when the spec gives only diameter, blade count and pitch: a
constant-pitch helix at the given geometric pitch, a typical general-aviation
chord distribution (activity factor ~85) and a NACA 4412 section (close to
the Clark Y most metal propellers use).

JSBSim's propeller wants C_THRUST(J) and C_POWER(J) with CT = T/(rho n^2 D^4),
CP = P/(rho n^3 D^5), n in rev/s; its piston engine a handful of numbers.
"""
import math

import numpy as np

from .aero.section import SectionPolar
from .geometry import airfoil as af

RHO0, NU0 = 1.225, 1.46e-5
HP = 745.7


class Propeller:
    def __init__(self, engine, n_elements=30):
        spec = engine.prop_spec
        self.D = engine.prop_diameter
        self.R = 0.5 * self.D
        self.B = engine.prop_blades
        pitch = engine.prop_pitch
        if pitch is None:
            pitch = 0.75 * self.D  # a cruise-ish default
        self.pitch = float(pitch)
        self.rpm = engine.rpm * float(spec.get("gear_ratio", 1.0))
        self.hub = float(spec.get("hub_fraction", 0.15))
        x = np.linspace(self.hub, 1.0, n_elements + 1)
        self.x = 0.5 * (x[1:] + x[:-1])       # element centres, r/R
        self.dx = np.diff(x)
        chord = spec.get("chord")              # [[r/R, c/R], ...] optional
        if chord:
            c = np.asarray(chord, float)
            self.c_R = np.interp(self.x, c[:, 0], c[:, 1])
        else:
            self.c_R = 0.16 - 0.08 * self.x**2
        twist = spec.get("twist")              # [[r/R, deg], ...] optional, else a constant-pitch helix
        if twist:
            t = np.asarray(twist, float)
            self.beta = np.radians(np.interp(self.x, t[:, 0], t[:, 1]))
        else:
            self.beta = np.arctan(self.pitch / (2 * math.pi * self.x * self.R))
        foil = af.get(spec.get("airfoil", "naca4412"))
        vtip = 2 * math.pi * self.rpm / 60 * self.R
        ar = 1.0 / np.mean(self.c_R)
        self.polars = [SectionPolar(foil, re=max(vtip * xi * ci * self.R / NU0, 1e5), aspect_ratio=ar)
                       for xi, ci in zip(self.x, self.c_R)]
        self.activity_factor = float(100000.0 / 16.0 * np.sum(0.5 * self.c_R * self.x**3 * self.dx))
        self.ixx = float(spec.get("ixx", 0.0)) or None

    def element_forces(self, J):
        """Thrust and power coefficients at advance ratio J (BEMT)."""
        n = self.rpm / 60.0
        omega = 2 * math.pi * n
        V = J * n * self.D
        r = self.x * self.R
        c = self.c_R * self.R
        dr = self.dx * self.R
        vi = np.full_like(r, 0.05 * omega * self.R)   # axial induced velocity
        vt = np.zeros_like(r)                         # swirl
        for _ in range(200):
            va = V + vi
            ut = omega * r - vt
            W = np.hypot(va, ut)
            phi = np.arctan2(va, ut)
            alpha = self.beta - phi
            cl = np.empty_like(r)
            cd = np.empty_like(r)
            for k, p in enumerate(self.polars):
                cl[k], cd[k], _ = p.evaluate(alpha[k])
            cn = cl * np.cos(phi) - cd * np.sin(phi)
            ct = cl * np.sin(phi) + cd * np.cos(phi)
            # Prandtl tip and hub losses
            sphi = np.maximum(np.abs(np.sin(phi)), 0.05)
            f_tip = self.B / 2.0 * (self.R - r) / (r * sphi)
            f_hub = self.B / 2.0 * (r - self.hub * self.R) / (self.hub * self.R * sphi)
            F = (2 / math.pi) ** 2 * np.arccos(np.exp(-np.clip(f_tip, 0, 50))) * np.arccos(np.exp(-np.clip(f_hub, 0, 50)))
            F = np.maximum(F, 0.05)
            # momentum balance per annulus: element thrust = 4 pi r rho (V + vi) vi F dr
            dT_be = 0.5 * RHO0 * W**2 * self.B * c * cn
            dQ_be = 0.5 * RHO0 * W**2 * self.B * c * ct * r
            # solve 4 pi r F (V + vi) vi = dT_be / dr for vi (the root with the flow's sign)
            k = dT_be / (4 * math.pi * r * RHO0 * F)
            disc = V * V + 4 * k
            vi_new = np.where(disc >= 0, 0.5 * (-V + np.sqrt(np.maximum(disc, 0.0))), -0.5 * V)
            vt_new = dQ_be / (4 * math.pi * r**3 * RHO0 * F * np.maximum(np.abs(va), 1e-3)) * r
            vt_new = np.clip(vt_new, -0.5 * omega * r, 0.5 * omega * r)
            dv = np.max(np.abs(vi_new - vi)) + np.max(np.abs(vt_new - vt))
            vi = 0.6 * vi + 0.4 * vi_new
            vt = 0.6 * vt + 0.4 * vt_new
            if dv < 1e-6 * omega * self.R:
                break
        T = float(np.sum(dT_be * dr))
        Q = float(np.sum(dQ_be * dr))
        P = Q * omega
        return T / (RHO0 * n**2 * self.D**4), P / (RHO0 * n**3 * self.D**5)

    def tables(self, j_max=2.4, dj=0.1):
        J = np.round(np.arange(0.0, j_max + 1e-9, dj), 3)
        ct, cp = zip(*[self.element_forces(j) for j in J])
        return {"J": J, "CT": np.array(ct), "CP": np.array(cp)}

    def inertia(self, mass):
        """Spin inertia (kg m^2) of the blades: a blade is roughly a slender
        rod, (1/3) m R^2 over the blades' share of the propeller mass."""
        return self.ixx or (0.7 * mass) * self.R**2 / 3.0


def static_numbers(tab, D, rpm, power_w):
    """Static thrust and the peak efficiency, for checks."""
    n = rpm / 60.0
    eff = np.where(tab["CP"] > 1e-6, tab["J"] * tab["CT"] / tab["CP"], 0.0)
    i = int(np.argmax(eff))
    zero = float(np.interp(0.0, -tab["CT"], tab["J"])) if np.any(tab["CT"] < 0) else float("nan")
    # static thrust at the power the engine gives: find n where CP rho n^3 D^5 = P
    n_static = (power_w / (tab["CP"][0] * RHO0 * D**5)) ** (1 / 3)
    return {"static_thrust_n": float(tab["CT"][0] * RHO0 * min(n, n_static) ** 2 * D**4),
            "peak_efficiency": float(eff[i]), "peak_efficiency_J": float(tab["J"][i]), "zero_thrust_J": float(zero)}


def piston_xml(engine):
    """A JSBSim piston engine scaled from the given power and rpm."""
    hp = engine.power_kw * 1000.0 / HP
    disp = float(engine.prop_spec.get("displacement_in3", 0.0)) or 2.0 * hp
    return """<?xml version="1.0"?>
<!-- Generated by hangar: a piston engine scaled from %.0f hp at %.0f rpm. -->
<piston_engine name="%s">
  <minmp unit="INHG">        10.0 </minmp>
  <maxmp unit="INHG">        28.5 </maxmp>
  <displacement unit="IN3"> %.1f </displacement>
  <maxhp>                   %.1f </maxhp>
  <bsfc>                     0.45 </bsfc>
  <cycles>                    4.0 </cycles>
  <idlerpm>                 %.0f </idlerpm>
  <maxrpm>                  %.0f </maxrpm>
  <maxthrottle>               1.0 </maxthrottle>
  <minthrottle>               0.1 </minthrottle>
  <sparkfaildrop>             0.1 </sparkfaildrop>
</piston_engine>
""" % (hp, engine.rpm, engine.name, disp, hp, 0.22 * engine.rpm, engine.rpm)


def electric_xml(engine):
    """A JSBSim brushless DC motor (Drela's first-order motor model: current
    (V - rpm/Kv)/R, torque from the current above the no-load current) sized
    for the given power at the given rpm: the battery voltage steps with power,
    Kv puts the rated rpm at 85 % of the no-load speed, and the coil resistance
    gives the rated power there. Its torque is finite at standstill, so it
    spins a propeller up stably - JSBSim's plain "electric" engine divides
    power by rpm and does not."""
    P = engine.power_kw * 1000.0
    V = float(engine.prop_spec.get("battery_volts", 0.0)) or (11.1 if P < 600 else 22.2 if P < 2500 else 44.4 if P < 8000 else 88.8)
    kv = engine.rpm / (0.85 * V)
    R = 0.1275 * V * V / P
    i0 = max(0.3, 0.012 * 0.15 * V / R)
    return """<?xml version="1.0"?>
<!-- Generated by hangar: a brushless DC motor for %.2f kW at %.0f rpm on %.1f V. -->
<brushless_dc_motor name="%s">
  <velocityconstant> %.2f </velocityconstant>
  <coilresistance unit="OHMS"> %.5f </coilresistance>
  <noloadcurrent unit="AMPERES"> %.3f </noloadcurrent>
  <maxvolts unit="VOLTS"> %.2f </maxvolts>
</brushless_dc_motor>
""" % (engine.power_kw, engine.rpm, V, engine.name, kv, R, i0, V)


# -- turbofans ----------------------------------------------------------------------------------
_MACH = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6)
_ALT_FT = (-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 36089.0, 40000.0, 50000.0, 60000.0, 70000.0)
SIGMA_TROPOPAUSE = 0.29708   # ISA density ratio at 11 km


def _isa(h_m):
    """Temperature (K) and pressure (Pa) of the standard atmosphere."""
    if h_m <= 11000.0:
        t = 288.15 - 0.0065 * h_m
        return t, 101325.0 * (t / 288.15) ** 5.2559
    p11 = 101325.0 * (216.65 / 288.15) ** 5.2559
    return 216.65, p11 * np.exp(-(h_m - 11000.0) / 6341.6)


def high_bypass_lapse(mach, h_m, tr=1.2):
    """Installed thrust over sea-level-static thrust of a high-bypass
    turbofan (Mattingly, Heiser & Pratt, "Aircraft Engine Design", 2002,
    sec. 2.3.2): delta0 (1 - 0.49 sqrt(M)), less 3 (theta0 - TR) / (1.5 + M)
    once the total temperature at the fan face, theta0, passes the throttle
    ratio TR - its fan's thrust falls fast with speed, a fifth of its static
    thrust left at Mach 0.8 and 35,000 ft."""
    t, p = _isa(h_m)
    f = 1.0 + 0.2 * mach * mach
    delta0 = p / 101325.0 * f ** 3.5
    theta0 = t / 288.15 * f
    lapse = 1.0 - 0.49 * math.sqrt(max(mach, 0.0))
    if theta0 > tr:
        lapse -= 3.0 * (theta0 - tr) / (1.5 + mach)
    return max(delta0 * lapse, 0.0)


def turbofan_lapse(mach, h_m, tr=1.2, wet=False, bypass=0.0):
    """Installed thrust over sea-level-static thrust (dry, or wet with the
    afterburner) of a low-bypass afterburning turbofan: the density lapse
    - sigma^0.7 with the afterburner lit, sigma^1 without - times a ram gain
    of (1 + 0.2 M^2), cut back once the total temperature at the compressor
    face, theta0, passes the throttle ratio TR where the turbine reaches its
    temperature limit (Mattingly, Heiser & Pratt, "Aircraft Engine Design",
    2002, sec. 2.3: the bracket (1 - k (theta0 - TR) / theta0)). Mattingly's
    own lapse, delta0 below TR, gives an afterburning engine 1.7 times its
    static thrust at Mach 0.9 at sea level and none past Mach 2.2 at 40,000
    ft for a TR of 1.07; engines like the F100 have neither.

    Above the tropopause the air's temperature holds, so at a given Mach
    number the engine runs at the same corrected point and its thrust goes
    as the pressure - as the density there (Mattingly: thrust ~ delta0 at a
    fixed theta0); the afterburner's sigma^0.7 runs only up to 11 km.

    A dry engine of bypass ratio above 1 moves towards the high-bypass
    lapse (high_bypass_lapse), all of it from a bypass ratio of 3: a
    TF33's (1.4) a fifth of the way, a CFM56's or a TF34's all of it."""
    w = 0.0 if wet else float(np.clip((bypass - 1.0) / 2.0, 0.0, 1.0))
    if w >= 1.0:
        return high_bypass_lapse(mach, h_m, tr)
    t, p = _isa(h_m)
    sigma = (p / 101325.0) / (t / 288.15)
    n = 0.7 if wet else 1.0
    density = sigma ** n if sigma >= SIGMA_TROPOPAUSE else SIGMA_TROPOPAUSE ** n * sigma / SIGMA_TROPOPAUSE
    f = 1.0 + 0.2 * mach * mach
    theta0 = t / 288.15 * f
    over = max(theta0 - tr, 0.0) / theta0
    low = density * f * max(1.0 - (2.5 if wet else 3.0) * over, 0.0)
    return low if w <= 0.0 else (1.0 - w) * low + w * high_bypass_lapse(mach, h_m, tr)


def turbofan_tables(engine):
    """JSBSim's three thrust tables (fractions of the sea-level-static
    military and maximum thrust) over Mach and density altitude, from the
    lapse model; idle a few percent of military thrust, falling with the ram
    drag of the airflow it swallows."""
    tr = engine.throttle_ratio
    bpr = engine.bypass_ratio
    mil0 = turbofan_lapse(0.0, 0.0, tr, bypass=bpr)
    wet0 = turbofan_lapse(0.0, 0.0, tr, wet=True)
    out = {"IdleThrust": [], "MilThrust": [], "AugThrust": []}
    for m in _MACH:
        idle, mil, aug = [], [], []
        for h_ft in _ALT_FT:
            h = h_ft * 0.3048
            _, p = _isa(h)
            delta = p / 101325.0
            idle.append(max(delta * (0.05 - 0.08 * m), -0.12))
            mil.append(max(turbofan_lapse(m, h, tr, bypass=bpr) / mil0, 0.0))
            aug.append(max(turbofan_lapse(m, h, tr, wet=True) / wet0, 0.0))
        out["IdleThrust"].append(idle)
        out["MilThrust"].append(mil)
        out["AugThrust"].append(aug)
    return out


def _thrust_table(name, rows):
    head = "        " + "".join("%10.0f" % h for h in _ALT_FT)
    body = "\n".join("   %6.2f " % m + "".join("%10.4f" % v for v in row) for m, row in zip(_MACH, rows))
    return """  <function name="%s">
   <table>
    <independentVar lookup="row">velocities/mach</independentVar>
    <independentVar lookup="column">atmosphere/density-altitude</independentVar>
    <tableData>
%s
%s
    </tableData>
   </table>
  </function>""" % (name, head, body)


def turbofan_xml(engine):
    """A JSBSim turbine for the given thrusts. The afterburner (augmethod 2)
    is engaged by throttle positions above 1 (the flight control system maps
    the throttle lever onto 0..2, military power at the detent)."""
    tabs = turbofan_tables(engine)
    lbf = 1000.0 / 4.448222
    mil = engine.thrust_dry_kn * lbf
    wet = (engine.thrust_wet_kn or engine.thrust_dry_kn) * lbf
    aug = engine.thrust_wet_kn is not None
    parts = ["""<?xml version="1.0"?>
<!-- Generated by hangar: a turbofan of %.1f kN dry%s, bypass ratio %.2f.
     Thrust lapse: density lapse and ram gain, cut back past the throttle
     ratio %.2f (Mattingly, Heiser & Pratt, Aircraft Engine Design, 2002). -->
<turbine_engine name="%s">
  <milthrust> %.1f </milthrust>
  <maxthrust> %.1f </maxthrust>
  <bypassratio> %.3f </bypassratio>
  <tsfc> %.3f </tsfc>
  <atsfc> %.3f </atsfc>
  <idlen1> 30.0 </idlen1>
  <idlen2> 60.0 </idlen2>
  <maxn1> 100.0 </maxn1>
  <maxn2> 100.0 </maxn2>
  <augmented> %d </augmented>
  <augmethod> 2 </augmethod>
  <injected> 0 </injected>""" % (engine.thrust_dry_kn, (", %.1f kN with afterburner" % engine.thrust_wet_kn) if aug else "",
                                engine.bypass_ratio, engine.throttle_ratio, engine.name, mil, wet, engine.bypass_ratio,
                                engine.tsfc_dry, engine.tsfc_wet, 1 if aug else 0)]
    parts.append(_thrust_table("IdleThrust", tabs["IdleThrust"]))
    parts.append(_thrust_table("MilThrust", tabs["MilThrust"]))
    if aug:
        parts.append(_thrust_table("AugThrust", tabs["AugThrust"]))
    parts.append("</turbine_engine>\n")
    return "\n".join(parts)


def nozzle_xml(name):
    """JSBSim's "direct" thruster: the engine's thrust along the nozzle axis."""
    return """<?xml version="1.0"?>
<!-- Generated by hangar: the thrust of %s acts along the nozzle axis. -->
<direct name="%s nozzle">
</direct>
""" % (name, name)


def propeller_xml(prop, tab, mass, name):
    rows = "\n".join("      %5.2f  %9.5f" % (j, v) for j, v in zip(tab["J"], tab["CT"]))
    rowp = "\n".join("      %5.2f  %9.5f" % (j, v) for j, v in zip(tab["J"], tab["CP"]))
    ixx = prop.inertia(mass) * 0.73756  # kg m2 -> slug ft2
    return """<?xml version="1.0"?>
<!-- Generated by hangar: blade-element momentum theory for a %d-blade fixed-pitch
     propeller, %.3f m diameter, %.3f m geometric pitch (activity factor %.0f). -->
<propeller name="%s">
  <ixx> %.3f </ixx>
  <diameter unit="M"> %.4f </diameter>
  <numblades> %d </numblades>
  <table name="C_THRUST" type="internal">
    <tableData>
%s
      5.00  %9.5f
    </tableData>
  </table>
  <table name="C_POWER" type="internal">
    <tableData>
%s
      5.00  %9.5f
    </tableData>
  </table>
  <table name="CT_MACH" type="internal">
    <tableData>
      0.85   1.0
      1.05   0.8
    </tableData>
  </table>
  <table name="CP_MACH" type="internal">
    <tableData>
      0.85   1.0
      1.05   1.8
      2.00   1.4
    </tableData>
  </table>
</propeller>
""" % (prop.B, prop.D, prop.pitch, prop.activity_factor, name, ixx, prop.D, prop.B, rows, tab["CT"][-1], rowp,
       tab["CP"][-1])
