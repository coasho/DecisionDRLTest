"""The JSBSim aircraft: <name>.xml and its engine and propeller files.

Everything is written in SI with JSBSim's unit attributes (metres, kg,
N/m...), in the design frame, which is JSBSim's structural frame. Control
conventions are JSBSim's (and the platform's): elevator positive trailing
edge down, aileron positive = left aileron down (right roll), rudder positive
= trailing edge left, flaps positive down; the flight control system maps the
normalised commands fcs/*-cmd-norm the platform writes onto the surfaces the
tables are indexed by.
"""
import datetime
import math
import os

import numpy as np

from . import __version__
from .mass import G0

CHANNEL_PROPERTY = {"elevator": "fcs/elevator-pos-deg", "aileron": "fcs/left-aileron-pos-deg",
                    "rudder": "fcs/rudder-pos-deg", "flap": "fcs/flap-pos-deg"}
AXES = {"CD": "DRAG", "CY": "SIDE", "CL": "LIFT", "Cl": "ROLL", "Cm": "PITCH", "Cn": "YAW"}
REF_LENGTH = {"Cl": "metrics/bw-ft", "Cm": "metrics/cbarw-ft", "Cn": "metrics/bw-ft"}


def _loc(p, indent):
    pad = " " * indent
    return "%s<x> %.4f </x>\n%s<y> %.4f </y>\n%s<z> %.4f </z>" % (pad, p[0], pad, p[1], pad, p[2])


def _table2(rows, cols, data, row_prop, col_prop, indent):
    pad = " " * indent
    head = pad + "          " + "".join("%10.2f" % c for c in cols)
    body = "\n".join(pad + "%10.2f" % r + "".join("%10.5f" % v for v in data[i]) for i, r in enumerate(rows))
    return ("%s<table>\n%s  <independentVar lookup=\"row\">%s</independentVar>\n%s  <independentVar lookup=\"column\">%s</independentVar>\n"
            "%s  <tableData>\n%s\n%s\n%s  </tableData>\n%s</table>") % (pad, pad, row_prop, pad, col_prop, pad, head, body, pad, pad)


def _table1(rows, data, row_prop, indent):
    pad = " " * indent
    body = "\n".join(pad + "    %10.2f %10.5f" % (r, v) for r, v in zip(rows, data))
    return "%s<table>\n%s  <independentVar lookup=\"row\">%s</independentVar>\n%s  <tableData>\n%s\n%s  </tableData>\n%s</table>" % (
        pad, pad, row_prop, pad, body, pad, pad)


def _function(name, description, factors, table, indent=6):
    pad = " " * indent
    props = "\n".join("%s    <property>%s</property>" % (pad, f) if not f.startswith("value:") else
                      "%s    <value>%s</value>" % (pad, f[6:]) for f in factors)
    return ("%s<function name=\"aero/coefficient/%s\">\n%s  <description>%s</description>\n%s  <product>\n%s\n%s\n%s  </product>\n%s</function>"
            % (pad, name, pad, description, pad, props, table, pad, pad))


def aerodynamics_xml(tables, aircraft, ge_e=0.85):
    """The <aerodynamics> element from the tables (tables.build)."""
    a = tables["alpha"]
    b = tables["beta"]
    out = {axis: [] for axis in AXES.values()}
    qs = ["aero/qbar-psf", "metrics/Sw-sqft"]
    ge = tables["ground_effect"]
    ge_lift = _table1(ge["h_b"], ge["lift"], "aero/h_b-mac-ft", 8)
    for k, axis in AXES.items():
        factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else [])
        tab = _table2(a, b, tables["base"][k], "aero/alpha-deg", "aero/beta-deg", 8)
        if k == "CL":
            tab = tab + "\n" + ge_lift
        out[axis].append(_function("%s_base" % k, "%s over alpha and beta%s (hangar)" % (k, ", ground effect" if k == "CL" else ""),
                                   factors, tab))
    for ch, t in tables["controls"].items():
        prop = CHANNEL_PROPERTY[ch]
        for k in t:
            if k == "deflection":
                continue
            factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else [])
            tab = _table2(a, t["deflection"], t[k], "aero/alpha-deg", prop, 8)
            if k == "CL" and ch == "flap":
                tab = tab + "\n" + ge_lift
            out[AXES[k]].append(_function("%s_%s" % (k, ch), "%s increment from %s (deg), over alpha" % (k, ch), factors, tab))
    rate_prop = {"p": ("aero/bi2vel", "velocities/p-aero-rad_sec"), "r": ("aero/bi2vel", "velocities/r-aero-rad_sec"),
                 "q": ("aero/ci2vel", "velocities/q-aero-rad_sec")}
    for rate, ks in tables["rates"].items():
        for k, data in ks.items():
            if k == "CD":
                continue
            factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else []) + list(rate_prop[rate])
            out[AXES[k]].append(_function("%s%s" % (k, rate), "%s per %s (non-dimensional), over alpha" % (k, rate), factors,
                                          _table1(a, data, "aero/alpha-deg", 8)))
    ad = tables["alphadot"]
    for k in ("CL", "Cm"):
        factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else []) + ["aero/ci2vel", "aero/alphadot-rad_sec", "value:%.5f" % ad[k]]
        pad = " " * 6
        props = "\n".join("%s    <property>%s</property>" % (pad, f) if not f.startswith("value:") else "%s    <value>%s</value>" % (pad, f[6:])
                          for f in factors)
        out[AXES[k]].append("%s<function name=\"aero/coefficient/%sadot\">\n%s  <description>%s per alpha-dot (downwash lag at the tail)</description>\n"
                            "%s  <product>\n%s\n%s  </product>\n%s</function>" % (pad, k, pad, k, pad, props, pad, pad))
    # ground effect on induced drag: (factor - 1) CL^2 / (pi A e)
    A = aircraft.b**2 / aircraft.S
    ge_drag = ge["drag"] - 1.0
    pad = " " * 6
    out["DRAG"].append("%s<function name=\"aero/coefficient/CD_ground\">\n%s  <description>Induced drag change in ground effect</description>\n"
                       "%s  <product>\n%s    <property>aero/qbar-psf</property>\n%s    <property>metrics/Sw-sqft</property>\n"
                       "%s    <property>aero/cl-squared</property>\n%s    <value>%.5f</value>\n%s\n%s  </product>\n%s</function>"
                       % (pad, pad, pad, pad, pad, pad, pad, 1.0 / (math.pi * A * ge_e),
                          _table1(ge["h_b"], ge_drag, "aero/h_b-mac-ft", 8), pad, pad))
    parts = ["    <aerodynamics>"]
    for axis in ("DRAG", "SIDE", "LIFT", "ROLL", "PITCH", "YAW"):
        parts.append("      <axis name=\"%s\">" % axis)
        parts.extend("\n".join("  " + line for line in f.split("\n")) for f in out[axis])
        parts.append("      </axis>")
    parts.append("    </aerodynamics>")
    return "\n".join(parts)


def gear_loads(aircraft, mass, cg):
    """Static load (N) on each wheel with the aircraft at rest: pitch balance
    between the centre-line wheel(s) and the pairs."""
    wheels = [(g, name, pos) for g in aircraft.gear for name, pos in g.positions()]
    if not wheels:
        return {}
    W = mass * G0
    centre = [w for w in wheels if abs(w[2][1]) < 0.1]
    pairs = [w for w in wheels if abs(w[2][1]) >= 0.1]
    if not centre or not pairs:
        return {name: W / len(wheels) for _, name, _ in wheels}
    xc = np.mean([w[2][0] for w in centre])
    xp = np.mean([w[2][0] for w in pairs])
    fc = W * (xp - cg[0]) / (xp - xc)
    fp = W - fc
    out = {name: fc / len(centre) for _, name, _ in centre}
    out.update({name: fp / len(pairs) for _, name, _ in pairs})
    return out


def ground_reactions_xml(aircraft, mass_model):
    m, cg = mass_model.loaded()
    loads = gear_loads(aircraft, m, cg)
    parts = ["    <ground_reactions>"]
    for g in aircraft.gear:
        for name, pos in g.positions():
            f = max(loads.get(name, m * G0 / 3), 100.0)
            k = f / g.static_deflection
            c = 2 * 0.5 * math.sqrt(k * f / G0)
            if g.brake == "auto":
                brake = "NONE" if g.steerable or abs(pos[1]) < 0.1 else ("LEFT" if pos[1] < 0 else "RIGHT")
            else:
                brake = g.brake.upper()
            parts.append("""      <contact type="BOGEY" name="%s">
        <location unit="M">
%s
        </location>
        <static_friction> 0.8 </static_friction>
        <dynamic_friction> 0.5 </dynamic_friction>
        <rolling_friction> 0.02 </rolling_friction>
        <spring_coeff unit="N/M"> %.0f </spring_coeff>
        <damping_coeff unit="N/M/SEC"> %.0f </damping_coeff>
        <damping_coeff_rebound unit="N/M/SEC"> %.0f </damping_coeff_rebound>
        <max_steer unit="DEG"> %.1f </max_steer>
        <brake_group> %s </brake_group>
        <retractable> %d </retractable>
      </contact>""" % (name, _loc(pos, 10), k, c, 2 * c, g.max_steer_deg if g.steerable else 0.0, brake, int(g.retractable)))
    # structure: the points that touch first in a crash
    k_s = 20 * m * G0 / 0.1
    c_s = 2 * 0.7 * math.sqrt(k_s * m)
    for name, p in structure_points(aircraft):
        parts.append("""      <contact type="STRUCTURE" name="%s">
        <location unit="M">
%s
        </location>
        <static_friction> 0.8 </static_friction>
        <dynamic_friction> 0.6 </dynamic_friction>
        <spring_coeff unit="N/M"> %.0f </spring_coeff>
        <damping_coeff unit="N/M/SEC"> %.0f </damping_coeff>
      </contact>""" % (name, _loc(p, 10), k_s, c_s))
    parts.append("    </ground_reactions>")
    return "\n".join(parts)


def structure_points(aircraft):
    pts = []
    w = aircraft.wing
    tip = w.sections[-1]
    for side, s in (("LEFT", -1), ("RIGHT", 1)):
        if w.mirror:
            p = tip.le.copy()
            p[1] *= s
            pts.append(("%s WING TIP" % side, p + np.array([0.5 * tip.chord, 0.0, 0.0])))
    for b in aircraft.bodies:
        if b.kind != "fuselage":
            continue
        pts.append(("NOSE", np.array([b.x[0], 0.0, float(b.bottom[0])])))
        pts.append(("TAIL", np.array([b.x[-1], 0.0, float(b.bottom[-1])])))
        i = int(np.argmin(b.bottom))
        pts.append(("BELLY", np.array([b.x[i], 0.0, float(b.bottom[i])])))
    for s in aircraft.surfaces:
        if s.kind in ("fin", "vtail") and not s.mirror:
            t = s.sections[-1]
            pts.append(("%s TOP" % s.name.upper(), t.le + np.array([0.5 * t.chord, 0.0, 0.0])))
    for e in aircraft.engines:
        for name, _, prop, _ in e.copies():
            pts.append(("%s PROP TIP" % name.upper(), prop - np.array([0.0, 0.0, 0.5 * e.prop_diameter])))
    return pts


def flight_control_xml(aircraft):
    ch = {c.channel: c for _, c in aircraft.controls()}
    lim = {k: (min(c.min_deg for _, c in aircraft.controls() if c.channel == k),
               max(c.max_deg for _, c in aircraft.controls() if c.channel == k)) for k in ch}
    parts = ["    <flight_control name=\"%s\">" % aircraft.name]
    if "elevator" in ch:
        lo, hi = lim["elevator"]
        parts.append("""      <channel name="Pitch">
        <summer name="fcs/pitch-trim-sum">
          <input>fcs/elevator-cmd-norm</input>
          <input>fcs/pitch-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
        <aerosurface_scale name="fcs/elevator-control">
          <input>fcs/pitch-trim-sum</input>
          <range> <min>%.2f</min> <max>%.2f</max> </range>
          <gain>0.0174533</gain>
        </aerosurface_scale>
        <actuator name="fcs/elevator-actuator">
          <input>fcs/elevator-control</input>
          <rate_limit>2.0</rate_limit>
          <output>fcs/elevator-pos-rad</output>
        </actuator>
      </channel>""" % (lo, hi))
    if "aileron" in ch:
        lo, hi = lim["aileron"]
        parts.append("""      <channel name="Roll">
        <summer name="fcs/roll-trim-sum">
          <input>fcs/aileron-cmd-norm</input>
          <input>fcs/roll-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
        <aerosurface_scale name="fcs/left-aileron-control">
          <input>fcs/roll-trim-sum</input>
          <range> <min>%.2f</min> <max>%.2f</max> </range>
          <gain>0.0174533</gain>
        </aerosurface_scale>
        <actuator name="fcs/left-aileron-actuator">
          <input>fcs/left-aileron-control</input>
          <rate_limit>2.0</rate_limit>
          <output>fcs/left-aileron-pos-rad</output>
        </actuator>
        <pure_gain name="fcs/right-aileron">
          <input>fcs/left-aileron-pos-rad</input>
          <gain>-1</gain>
          <output>fcs/right-aileron-pos-rad</output>
        </pure_gain>
      </channel>""" % (lo, hi))
    if "rudder" in ch:
        lo, hi = lim["rudder"]
        parts.append("""      <channel name="Yaw">
        <summer name="fcs/yaw-trim-sum">
          <input>fcs/rudder-cmd-norm</input>
          <input>fcs/yaw-trim-cmd-norm</input>
          <clipto> <min>-1</min> <max>1</max> </clipto>
        </summer>
        <aerosurface_scale name="fcs/rudder-control">
          <input>fcs/yaw-trim-sum</input>
          <range> <min>%.2f</min> <max>%.2f</max> </range>
          <gain>0.0174533</gain>
        </aerosurface_scale>
        <actuator name="fcs/rudder-actuator">
          <input>fcs/rudder-control</input>
          <rate_limit>2.0</rate_limit>
          <output>fcs/rudder-pos-rad</output>
        </actuator>
        <!-- the nose or tail wheel steers with the rudder pedals (JSBSim steers from
             fcs/steer-cmd-norm, positive right; the rudder command is positive left) -->
        <pure_gain name="fcs/steer-from-rudder">
          <input>fcs/yaw-trim-sum</input>
          <gain>-1</gain>
          <output>fcs/steer-cmd-norm</output>
        </pure_gain>
      </channel>""" % (lo, hi))
    if "flap" in ch:
        lo, hi = lim["flap"]
        steps = [0.0, 0.25, 0.5, 0.75, 1.0]
        settings = "\n".join("            <setting> <position>%.1f</position> <time>%.1f</time> </setting>" % (s * hi, 2.0 if s else 0.0)
                             for s in steps)
        parts.append("""      <channel name="Flaps">
        <kinematic name="fcs/flaps-control">
          <input>fcs/flap-cmd-norm</input>
          <traverse>
%s
          </traverse>
          <output>fcs/flap-pos-deg</output>
        </kinematic>
      </channel>""" % settings)
    pistons = sum(len(e.copies()) for e in aircraft.engines if e.type == "piston")
    if pistons:
        # JSBSim's piston engine dies of a rich mixture at altitude: lean it with
        # the ambient pressure (mixture = P / P_sl, as JSBSim's own C172 does)
        outs = "\n".join("          <output>fcs/mixture-cmd-norm[%d]</output>" % i for i in range(pistons))
        parts.append("""      <channel name="Automatic Mixture Control">
        <fcs_function name="fcs/auto-mixture">
          <function>
            <table>
              <independentVar lookup="row">atmosphere/P-psf</independentVar>
              <tableData>
                   0  0.0
                2117  1.0
              </tableData>
            </table>
          </function>
%s
        </fcs_function>
      </channel>""" % outs)
    if any(g.retractable for g in aircraft.gear):
        parts.append("""      <channel name="Landing Gear">
        <kinematic name="fcs/gear-control">
          <input>gear/gear-cmd-norm</input>
          <traverse>
            <setting> <position>0</position> <time>0</time> </setting>
            <setting> <position>1</position> <time>5</time> </setting>
          </traverse>
          <output>gear/gear-pos-norm</output>
        </kinematic>
      </channel>""")
    parts.append("    </flight_control>")
    return "\n".join(parts)


def propulsion_xml(aircraft, mass_model, engine_files):
    parts = ["    <propulsion>"]
    n_tanks = len(mass_model.tanks)
    for e, (eng_file, prop_file) in zip(aircraft.engines, engine_files):
        for name, pos, prop, sense in e.copies():
            feeds = "\n".join("        <feed>%d</feed>" % i for i in range(n_tanks)) if e.type == "piston" else ""
            orient = np.degrees(e.prop_orient)
            parts.append("""      <engine file="%s">
        <location unit="M">
%s
        </location>
%s
        <thruster file="%s">
          <location unit="M">
%s
          </location>
          <orient unit="DEG"> <roll>%.2f</roll> <pitch>%.2f</pitch> <yaw>%.2f</yaw> </orient>
          <sense>%d</sense>
        </thruster>
      </engine>""" % (eng_file, _loc(pos, 10), feeds, prop_file, _loc(prop, 12), orient[0], orient[1], orient[2],
                      -1 if sense == "cw" else 1))
    for t in mass_model.tanks:
        cap = float(t.get("capacity", 0.0))
        parts.append("""      <tank type="FUEL">
        <location unit="M">
%s
        </location>
        <capacity unit="KG"> %.2f </capacity>
        <contents unit="KG"> %.2f </contents>
      </tank>""" % (_loc(np.asarray(t["position"], float), 10), cap, cap * float(t.get("fill", 1.0))))
    parts.append("    </propulsion>")
    return "\n".join(parts)


def aircraft_xml(aircraft, tables, mass_model, engine_files, notes=""):
    e = mass_model.empty()
    a = aircraft
    htail = [s for s in a.surfaces if s.kind == "htail"]
    fins = [s for s in a.surfaces if s.kind in ("fin", "vtail")]
    arm = lambda s: s.mac[1][0] + 0.25 * s.mac[0] - a.aero_point[0]  # noqa: E731
    pilot = next((p for p in mass_model.payload if "pilot" in p["name"].lower()), None)
    eye = np.asarray(pilot["position"], float) + np.array([0.0, 0.0, 0.7]) if pilot else a.aero_point
    points = "\n".join("""      <pointmass name="%s">
        <weight unit="KG"> %.2f </weight>
        <location unit="M">
%s
        </location>
      </pointmass>""" % (p["name"], p["mass"], _loc(np.asarray(p["position"], float), 10)) for p in mass_model.payload)
    header = """<?xml version="1.0"?>
<?xml-stylesheet type="text/xsl" href="http://jsbsim.sourceforge.net/JSBSim.xsl"?>
<!-- Generated by hangar %s from %s on %s. Edit the design and rebuild; changes
     made here are overwritten. %s -->
<fdm_config name="%s" version="2.0" release="ALPHA"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:noNamespaceSchemaLocation="http://jsbsim.sourceforge.net/JSBSim.xsd">
    <fileheader>
      <author>hangar</author>
      <filecreationdate>%s</filecreationdate>
      <version>%s</version>
      <description>%s</description>
      <note>Aerodynamics: vortex lattice with viscous section polars, full-envelope
        strip model, slender-body fuselage (hangar). Mass: Raymer component
        weights on the geometry. Propeller: blade-element momentum theory.</note>
    </fileheader>
""" % (__version__, os.path.basename(a.path or a.name), datetime.date.today().isoformat(), notes, a.name,
       datetime.date.today().isoformat(), __version__, a.description or a.name)
    metrics = """    <metrics>
      <wingarea unit="M2"> %.4f </wingarea>
      <wingspan unit="M"> %.4f </wingspan>
      <chord unit="M"> %.4f </chord>
      <htailarea unit="M2"> %.4f </htailarea>
      <htailarm unit="M"> %.4f </htailarm>
      <vtailarea unit="M2"> %.4f </vtailarea>
      <vtailarm unit="M"> %.4f </vtailarm>
      <location name="AERORP" unit="M">
%s
      </location>
      <location name="EYEPOINT" unit="M">
%s
      </location>
      <location name="VRP" unit="M">
%s
      </location>
    </metrics>
""" % (a.S, a.b, a.c, htail[0].area if htail else 0.0, arm(htail[0]) if htail else 0.0,
       fins[0].area if fins else 0.0, arm(fins[0]) if fins else 0.0, _loc(a.aero_point, 8), _loc(eye, 8), _loc(e["cg"], 8))
    mass = """    <mass_balance negated_crossproduct_inertia="false">
      <ixx unit="KG*M2"> %.2f </ixx>
      <iyy unit="KG*M2"> %.2f </iyy>
      <izz unit="KG*M2"> %.2f </izz>
      <ixy unit="KG*M2"> %.3f </ixy>
      <ixz unit="KG*M2"> %.3f </ixz>
      <iyz unit="KG*M2"> %.3f </iyz>
      <emptywt unit="KG"> %.2f </emptywt>
      <location name="CG" unit="M">
%s
      </location>
%s
    </mass_balance>
""" % (e["ixx"], e["iyy"], e["izz"], e["ixy"], e["ixz"], e["iyz"], e["mass"], _loc(e["cg"], 8), points)
    return "".join([header, metrics, mass, ground_reactions_xml(a, mass_model), "\n",
                    propulsion_xml(a, mass_model, engine_files), "\n", flight_control_xml(a), "\n",
                    aerodynamics_xml(tables, a), "\n</fdm_config>\n"])
