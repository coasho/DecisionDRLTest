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
_PITCH_TABLES = {}   # a constant-speed propeller's tables, made once per process (calibrate rebuilds often)


class Propeller:
    def __init__(self, engine, n_elements=30):
        spec = engine.prop_spec
        self.D = engine.prop_diameter
        self.R = 0.5 * self.D
        self.B = engine.prop_blades
        # a constant-speed propeller's blades turn between the stops of their
        # range (deg at 75 % radius); a fixed-pitch one's are set at its pitch
        self.blade_angles = getattr(engine, "blade_angle", None)
        pitch = engine.prop_pitch
        if pitch is None and self.blade_angles is not None:
            # the blades' own twist: a helix at the middle of their range
            pitch = 2 * math.pi * 0.75 * self.R * math.tan(math.radians(0.5 * sum(self.blade_angles)))
        if pitch is None:
            pitch = 0.75 * self.D  # a cruise-ish default
        self.pitch = float(pitch)
        self.rpm = engine.prop_rpm
        self.gear_ratio = engine.gear_ratio
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
        self.thickness = foil.thickness_ratio
        self._spec = dict(spec)

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

    # -- a constant-speed propeller: the blades turned together --------------------------------
    def forces(self, J, blade_angle, tip_mach=0.0):
        """Thrust and power coefficients at advance ratios J (array), the
        blades turned to blade_angle (deg at 75 % radius): element_forces'
        blade-element momentum theory, every advance ratio at once. tip_mach:
        the rotational tip speed's Mach number (0: incompressible) - each
        element's drag then rises past its critical Mach number (Lock's
        fourth-power law, from Korn's drag divergence of the section, swept
        by the blade's tip_sweep)."""
        from .aero.section import PolarSet
        J = np.atleast_1d(np.asarray(J, float))
        mt = np.broadcast_to(np.asarray(tip_mach, float), J.shape)[:, None]
        polars = PolarSet(self.polars)
        n = self.rpm / 60.0
        omega = 2 * math.pi * n
        V = (J * n * self.D)[:, None]
        r = self.x * self.R
        c = self.c_R * self.R
        dr = self.dx * self.R
        beta = self.beta + (math.radians(blade_angle) - float(np.interp(0.75, self.x, self.beta)))
        a_inv = mt / (omega * self.R)                  # 1 / the speed of sound
        vi = np.full((len(J), len(r)), 0.05 * omega * self.R)
        vt = np.zeros_like(vi)
        for _ in range(300):
            va = V + vi
            ut = omega * r - vt
            W = np.hypot(va, ut)
            phi = np.arctan2(va, ut)
            cl, cd, _, _ = polars.evaluate(beta - phi)
            cd = cd + self._wave_drag(W * a_inv, cl)
            cn = cl * np.cos(phi) - cd * np.sin(phi)
            ct = cl * np.sin(phi) + cd * np.cos(phi)
            sphi = np.maximum(np.abs(np.sin(phi)), 0.05)
            f_tip = self.B / 2.0 * (self.R - r) / (r * sphi)
            f_hub = self.B / 2.0 * (r - self.hub * self.R) / (self.hub * self.R * sphi)
            F = (2 / math.pi) ** 2 * np.arccos(np.exp(-np.clip(f_tip, 0, 50))) * np.arccos(np.exp(-np.clip(f_hub, 0, 50)))
            F = np.maximum(F, 0.05)
            dT_be = 0.5 * RHO0 * W**2 * self.B * c * cn
            dQ_be = 0.5 * RHO0 * W**2 * self.B * c * ct * r
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
        T = np.sum(dT_be * dr, axis=1)
        P = np.sum(dQ_be * dr, axis=1) * omega
        return T / (RHO0 * n**2 * self.D**4), P / (RHO0 * n**3 * self.D**5)

    def _wave_drag(self, mach, cl):
        """A blade section's compressibility drag at its Mach number: Lock's
        20 (M - M_crit)^4 past the critical Mach number, M_crit = M_dd - 0.108
        (Lock's rise reaches 0.002 at M_dd), and Korn's drag divergence
        kappa/cos L - t/cos^2 L - cl/(10 cos^3 L), kappa 0.87 for a
        conventional section (Mason's form). Only the outer blade flies that
        fast: its thickness t is the tip's ([engine.propeller] tip_thickness,
        else the blade section's) and L the tip's sweep (tip_sweep, deg)."""
        spec = getattr(self, "_spec", {})
        cs = math.cos(math.radians(float(spec.get("tip_sweep", 0.0))))
        t = float(spec.get("tip_thickness", self.thickness))
        mdd = 0.87 / cs - t / cs**2 - np.abs(cl) / (10.0 * cs**3)
        return 20.0 * np.maximum(mach - (mdd - 0.108), 0.0) ** 4

    def pitch_tables(self, j_max=4.0, dj=0.1, step=5.0):
        """C_THRUST and C_POWER of a constant-speed propeller over advance
        ratio (rows) and blade angle (columns, deg at 75 % radius, every
        `step` across the blades' range), incompressible."""
        key = (self._key(), j_max, dj, step)
        if key not in _PITCH_TABLES:
            lo, hi = self.blade_angles
            angles = np.round(np.linspace(lo, hi, int(math.ceil((hi - lo) / step - 1e-9)) + 1), 3)
            J = np.round(np.arange(0.0, j_max + 1e-9, dj), 3)
            CT = np.empty((len(J), len(angles)))
            CP = np.empty_like(CT)
            for k, b in enumerate(angles):
                CT[:, k], CP[:, k] = self.forces(J, b)
            _PITCH_TABLES[key] = {"J": J, "blade_angle": angles, "CT": CT, "CP": CP}
        return _PITCH_TABLES[key]

    def mach_tables(self, tab, machs=(0.0, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2)):
        """JSBSim's CT_MACH and CP_MACH for a constant-speed propeller: its
        thrust and power coefficients over the helical tip Mach number, as
        fractions of the incompressible ones, where its blades work best
        (the peak of the efficiency over its tables: its cruise)."""
        key = (self._key(), tuple(machs), tab["CT"].shape)
        if key not in _PITCH_TABLES:
            eff = np.where(tab["CP"] > 1e-4, tab["J"][:, None] * tab["CT"] / np.maximum(tab["CP"], 1e-4), 0.0)
            i, k = np.unravel_index(int(np.argmax(eff)), eff.shape)
            J, b = float(tab["J"][i]), float(tab["blade_angle"][k])
            helical = math.sqrt(1.0 + (J / math.pi) ** 2)      # the helical over the rotational tip speed
            ct0, cp0 = (float(v[0]) for v in self.forces([J], b))
            mh = np.asarray(machs, float)
            ct, cp = self.forces(np.full(len(mh), J), b, tip_mach=mh / helical)
            _PITCH_TABLES[key] = {"mach": mh, "CT": ct / ct0, "CP": cp / cp0, "J": J, "blade_angle": b}
        return _PITCH_TABLES[key]

    def _key(self):
        """What the tables are made from (the design's propeller and the
        propeller's own numbers), for the cache of this process."""
        return repr((sorted(self._spec.items(), key=lambda kv: kv[0]), self.D, self.B, self.rpm, self.pitch, self.hub,
                     self.blade_angles, len(self.x)))

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
    lapse (high_bypass_lapse), all of it from a bypass ratio of 2: a
    TF33's (1.42) two fifths of the way - 0.29 of its static thrust at
    35,000 ft and Mach 0.82, as the JT3D's cruise ratings give it (0.27-0.29)
    - a CFM56's or a TF34's all of it."""
    w = 0.0 if wet else float(np.clip(bypass - 1.0, 0.0, 1.0))
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
    drag of the airflow it swallows. A flat-rated engine (its core's
    thermodynamic_thrust_kn above its rating) keeps its rating's thrust at
    each Mach number until its core's, lapsing with height, falls below
    it."""
    tr = engine.throttle_ratio
    bpr = engine.bypass_ratio
    flat = engine.thermo_thrust_kn / engine.thrust_dry_kn
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
            dry = turbofan_lapse(m, h, tr, bypass=bpr)
            if flat > 1.0:  # never below the rated engine's own lapse (below sea level, past TR)
                dry = max(dry, min(flat * dry, turbofan_lapse(m, 0.0, tr, bypass=bpr)))
            mil.append(max(dry / mil0, 0.0))
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


# -- turboprops ---------------------------------------------------------------------------------
# JSBSim's turboprop (FGTurboProp) runs its gas generator's speed N1 from
# idle to 100 % with the throttle, and its shaft power from a table over the
# output shaft's rpm and N1, times EnginePowerVC for the flight condition. The
# tables here come from these stated physics (and these numbers, estimates
# for a turboprop of today):
TP_IDLE_N1 = 60.0       # flight idle, % of the gas generator's full speed
TP_SUSTAIN_N1 = 50.0    # its self-sustaining speed: the core has no power to spare
TP_SPOOL_S = 1.0        # N1's time constant, s (JSBSim's own default)
TP_ITT_C = (500.0, 800.0)  # the inter-turbine temperature at idle and full power, deg C
TP_IDLE_FUEL = 0.20     # fuel flow at no shaft power, over that at full power (the Willans line's intercept)
TP_MACH = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
TP_N1 = (0.0, 20.0, 40.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0, 110.0)
TP_RPM = (0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.3)  # of the design speed


def blades_mass(engine):
    """The mass (kg) whose spin inertia a turboprop's propeller has: its
    own ([engine.propeller] mass), or a tenth of the engine's (about 4 kW of
    rated power per kg, mass.MassModel's estimate)."""
    if engine.prop_mass is not None:
        return float(engine.prop_mass)
    return 0.1 * (float(engine.mass) if engine.mass is not None else engine.power_kw / 4.0)


def core_power(n1):
    """The gas generator's shaft power over its full power at N1 (%): its
    air flow goes as its speed and the work each kilogram of it does as the
    speed squared, so the turbine's power goes as N1^3; what is left over the
    compressor's own need is none at the self-sustaining speed."""
    s = TP_SUSTAIN_N1 / 100.0
    return np.maximum(((np.asarray(n1, float) / 100.0) ** 3 - s**3) / (1.0 - s**3), 0.0)


def turbine_speed_power(x, omega_prop):
    """The free power turbine's power at a fraction x of its design speed,
    over its design power: its torque falls in a straight line from twice
    the design torque at a standstill to none at twice the design speed, so
    the power is x (2 - x). At a standstill JSBSim divides the power by
    1 rad/s instead of the propeller's speed (omega_prop, rad/s, at the design
    speed): the stall torque there is 2 / omega_prop of the design power."""
    x = np.asarray(x, float)
    return np.where(x > 0.0, x * (2.0 - x), 2.0 / omega_prop)


def _lapse_cut(mach):
    """How fast the power falls past the throttle ratio, per unit of theta0
    over it, at Mach M: Mattingly's turboprop thrust lapse loses 3 (theta0 -
    TR) / (8.13 (M - 0.1)) of the static thrust there, out of the 1 - 0.96
    (M - 0.1)^0.25 the propeller keeps at that speed - the same fraction of
    the power. Singular just past Mach 0.1 (none below it): taken from Mach
    0.2, faired in from none at 0.1."""
    m = np.maximum(np.asarray(mach, float), 0.2)
    cut = 3.0 / (8.13 * (m - 0.1) * (1.0 - 0.96 * (m - 0.1) ** 0.25))
    return np.where(np.asarray(mach) <= 0.1, 0.0, cut * np.clip((np.asarray(mach) - 0.1) / 0.1, 0.0, 1.0))


def turboprop_power_lapse(mach, h_m=None, tr=1.0, t_k=None, p_pa=None):
    """Shaft power over its sea-level-static value (Mattingly, Heiser & Pratt,
    "Aircraft Engine Design", 2002, sec. 2.3.2). Their turboprop's installed
    thrust lapse is delta0 up to Mach 0.1, delta0 (1 - 0.96 (M - 0.1)^0.25)
    beyond - the propeller's thrust at a given power falling with speed - and
    that less 3 (theta0 - TR) / (8.13 (M - 0.1)) once the total temperature
    ratio theta0 passes the throttle ratio TR. JSBSim's propeller makes the
    thrust from the power (its own tables), so the power keeps the engine's
    part: the total pressure ratio delta0, and past TR the same fraction cut
    (_lapse_cut). The standard atmosphere at h_m, or the air's own t_k, p_pa."""
    if t_k is None or p_pa is None:
        t_k, p_pa = _isa(h_m)
    f = 1.0 + 0.2 * np.asarray(mach, float) ** 2
    delta0 = p_pa / 101325.0 * f ** 3.5
    theta0 = t_k / 288.15 * f
    lapse = delta0 * np.maximum(1.0 - np.maximum(theta0 - tr, 0.0) * _lapse_cut(mach), 0.0)
    return float(lapse) if np.ndim(lapse) == 0 else lapse


def turboprop_tables(engine):
    """JSBSim's EnginePowerRPM_N1 (shaft power, hp, over the output shaft's
    rpm and N1), ITT_N1 (deg C, over N1, not burning and burning) and
    CombustionEfficiency_N1 (the full-power specific fuel consumption over
    the part-power one, over N1)."""
    hp = engine.thermo_power_kw * 1000.0 / HP
    omega = 2 * math.pi * engine.prop_rpm / 60.0
    n1 = np.asarray(TP_N1)
    rpm = np.asarray(TP_RPM) * engine.rpm
    power = hp * turbine_speed_power(np.asarray(TP_RPM), omega)[:, None] * core_power(n1)[None, :]
    # the inter-turbine temperature rises with the power, from idle's to full power's
    p = core_power(n1)
    p_idle = float(core_power(TP_IDLE_N1))
    t_idle, t_max = TP_ITT_C
    itt = np.where(n1 >= TP_IDLE_N1, t_idle + (t_max - t_idle) * (p - p_idle) / (1.0 - p_idle),
                   15.0 + (t_idle - 15.0) * n1 / TP_IDLE_N1)
    # the Willans line: fuel flow c + (1 - c) p of the full-power flow at a
    # power fraction p, so the specific consumption rises at part power by
    # (c + (1 - c) p) / p; JSBSim divides its psfc by this "efficiency"
    c = TP_IDLE_FUEL
    efficiency = np.maximum(p / (c + (1.0 - c) * p), 0.02)
    return {"rpm": rpm, "n1": n1, "EnginePowerRPM_N1": power, "ITT_N1": itt, "CombustionEfficiency_N1": efficiency}


def power_lapse_xml(engine, indent):
    """turboprop_power_lapse as a JSBSim function's <product>, of the air the
    aircraft flies in: the engine's EnginePowerVC, and the flight controls'
    estimate of its power (jsbsim.governor_xml)."""
    mach2 = "<sum> <value>1</value> <product> <value>0.2</value> <property>velocities/mach</property> " \
            "<property>velocities/mach</property> </product> </sum>"
    cut = "\n".join("              %4.2f %9.4f" % (m, v) for m, v in zip(TP_MACH, _lapse_cut(np.asarray(TP_MACH))))
    text = """<product>
  <quotient> <property>atmosphere/P-psf</property> <value>2116.228</value> </quotient>
  <pow> %s <value>3.5</value> </pow>
  <max>
    <value>0</value>
    <difference>
      <value>1</value>
      <product>
        <max>
          <value>0</value>
          <difference>
            <product> <quotient> <property>atmosphere/T-R</property> <value>518.67</value> </quotient> %s </product>
            <value>%.4f</value>
          </difference>
        </max>
        <table>
          <independentVar lookup="row">velocities/mach</independentVar>
          <tableData>
%s
          </tableData>
        </table>
      </product>
    </difference>
  </max>
</product>""" % (mach2, mach2, engine.throttle_ratio, cut)
    return "\n".join(" " * indent + line for line in text.split("\n"))


def turboprop_xml(engine):
    """A JSBSim turboprop (FGTurboProp) for the given shaft power and specific
    fuel consumption: maxpower the rating (it holds the power there), the
    tables from turboprop_tables, and EnginePowerVC the lapse of
    turboprop_power_lapse as a JSBSim function of the air it flies in."""
    t = turboprop_tables(engine)
    rating = engine.power_kw * 1000.0 / HP
    psfc = engine.psfc * 1.644                          # kg/(kW h) -> lb/(hp h)
    head = "          " + "".join("%9.1f" % n for n in t["n1"])
    power = "\n".join("   %9.1f " % r + "".join("%9.1f" % v for v in row) for r, row in zip(t["rpm"], t["EnginePowerRPM_N1"]))
    itt = "\n".join("   %6.1f %7.1f %7.1f" % (n, 15.0, v) for n, v in zip(t["n1"], t["ITT_N1"]))
    eff = "\n".join("   %6.1f %8.4f" % (n, v) for n, v in zip(t["n1"], t["CombustionEfficiency_N1"]))
    return """<?xml version="1.0"?>
<!-- Generated by hangar: a turboprop of %.0f kW (%.0f hp)%s, psfc %.3f kg/(kW h),
     its propeller at %.0f rpm through a %.3g:1 gearbox. Power lapse: Mattingly, Heiser &
     Pratt, Aircraft Engine Design, 2002, sec. 2.3.2 (throttle ratio %.3f). -->
<turboprop_engine name="%s">
  <maxpower> %.1f </maxpower>
  <psfc> %.4f </psfc>
  <idlen1> %.1f </idlen1>
  <maxn1> 100.0 </maxn1>
  <n1idle_max_delay> %.2f </n1idle_max_delay>
  <function name="EnginePowerVC">
    <description>Shaft power over sea-level static: the total pressure ratio delta0, cut once
      the total temperature ratio theta0 passes the throttle ratio</description>
%s
  </function>
  <!-- shaft power (hp) over the output shaft's rpm (rows) and N1 (columns, %%) -->
  <table name="EnginePowerRPM_N1" type="internal">
    <tableData>
%s
%s
    </tableData>
  </table>
  <!-- inter-turbine temperature (deg C) over N1: not burning (0), burning (1) -->
  <table name="ITT_N1" type="internal">
    <tableData>
                 0       1
%s
    </tableData>
  </table>
  <table name="CombustionEfficiency_N1" type="internal">
    <tableData>
%s
    </tableData>
  </table>
</turboprop_engine>
""" % (engine.power_kw, rating, (", its core %.0f kW" % engine.thermo_power_kw) if engine.thermo_power_kw > engine.power_kw else "",
       engine.psfc, engine.prop_rpm, engine.gear_ratio, engine.throttle_ratio, engine.name, rating, psfc, TP_IDLE_N1,
       TP_SPOOL_S, power_lapse_xml(engine, 4), head, power, itt, eff)


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
    gear = "  <gearratio> %.4f </gearratio>\n" % prop.gear_ratio if prop.gear_ratio != 1.0 else ""
    return """<?xml version="1.0"?>
<!-- Generated by hangar: blade-element momentum theory for a %d-blade fixed-pitch
     propeller, %.3f m diameter, %.3f m geometric pitch (activity factor %.0f). -->
<propeller name="%s">
  <ixx> %.3f </ixx>
  <diameter unit="M"> %.4f </diameter>
  <numblades> %d </numblades>
%s  <table name="C_THRUST" type="internal">
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
""" % (prop.B, prop.D, prop.pitch, prop.activity_factor, name, ixx, prop.D, prop.B, gear, rows, tab["CT"][-1], rowp,
       tab["CP"][-1])


# The governor's speed range: JSBSim's constant-speed propeller aims at
# minrpm + (maxrpm - minrpm) x its advance command, which the flight controls
# set (jsbsim.governor_xml) so the propeller holds its governed speed
GOVERNOR_RANGE = (0.8, 1.1)   # minrpm, maxrpm over the governed speed
GOVERNOR_CP = tuple(np.round(np.arange(0.0, 1.2001, 0.05), 3))   # the power coefficients of blade_angles_for_power


def blade_angles_for_power(tab, cp=GOVERNOR_CP):
    """Where a constant-speed propeller's governor sets its blades at its
    speed: at each advance ratio of its tables (rows), the smallest blade
    angle (deg at 75 % radius) at which it takes each power coefficient
    (columns) - the low stop if it takes more there already (windmilling),
    the high stop if it cannot take that much."""
    angles, CP = np.asarray(tab["blade_angle"], float), np.asarray(tab["CP"], float)
    out = np.empty((len(tab["J"]), len(cp)))
    for i, row in enumerate(CP):
        for k, target in enumerate(cp):
            if row[0] >= target:
                out[i, k] = angles[0]
                continue
            up = np.nonzero((row[:-1] < target) & (row[1:] >= target))[0]
            if not len(up):
                out[i, k] = angles[-1]
                continue
            j = int(up[0])
            out[i, k] = angles[j] + (target - row[j]) / (row[j + 1] - row[j]) * (angles[j + 1] - angles[j])
    return out


def constant_speed_propeller_xml(prop, tab, mach, mass, name):
    """A JSBSim constant-speed propeller: C_THRUST and C_POWER over advance
    ratio and blade angle (Propeller.pitch_tables), the blades' range as its
    minpitch and maxpitch, the governor's rpm range (GOVERNOR_RANGE) and the
    gearbox; CT_MACH and CP_MACH over the helical tip Mach number
    (Propeller.mach_tables)."""
    J, angles = tab["J"], tab["blade_angle"]

    def table(key):
        head = "            " + "".join("%9.2f" % b for b in angles)
        body = "\n".join("      %5.2f " % j + "".join("%9.5f" % v for v in row) for j, row in zip(J, tab[key]))
        # past the last advance ratio the coefficients stay as they are there
        last = "      %5.2f " % (J[-1] + 2.0) + "".join("%9.5f" % v for v in tab[key][-1])
        return "%s\n%s\n%s" % (head, body, last)

    def factors(key):
        return "\n".join("      %5.2f %8.4f" % (m, v) for m, v in zip(mach["mach"], mach[key]))
    ixx = prop.inertia(mass) * 0.73756  # kg m2 -> slug ft2
    lo, hi = GOVERNOR_RANGE
    return """<?xml version="1.0"?>
<!-- Generated by hangar: blade-element momentum theory for a %d-blade constant-speed
     propeller, %.3f m diameter, governed at %.0f rpm, its blades %.1f-%.1f deg at 75 %%
     radius (activity factor %.0f); compressibility from the same blades at J %.2f and
     %.1f deg (Lock's drag rise past Korn's drag divergence). -->
<propeller name="%s">
  <ixx> %.3f </ixx>
  <diameter unit="M"> %.4f </diameter>
  <numblades> %d </numblades>
  <gearratio> %.4f </gearratio>
  <minpitch> %.2f </minpitch>
  <maxpitch> %.2f </maxpitch>
  <minrpm> %.1f </minrpm>
  <maxrpm> %.1f </maxrpm>
  <table name="C_THRUST" type="internal">
    <tableData>
%s
    </tableData>
  </table>
  <table name="C_POWER" type="internal">
    <tableData>
%s
    </tableData>
  </table>
  <table name="CT_MACH" type="internal">
    <tableData>
%s
    </tableData>
  </table>
  <table name="CP_MACH" type="internal">
    <tableData>
%s
    </tableData>
  </table>
</propeller>
""" % (prop.B, prop.D, prop.rpm, angles[0], angles[-1], prop.activity_factor, mach["J"], mach["blade_angle"], name, ixx,
       prop.D, prop.B, prop.gear_ratio, angles[0], angles[-1], lo * prop.rpm, hi * prop.rpm, table("CT"), table("CP"),
       factors("CT"), factors("CP"))
