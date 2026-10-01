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
from .applicability import references_xml
from .fcs import YAW_DAMPER_WASHOUT_S
from .mass import G0

CHANNEL_PROPERTY = {"elevator": "fcs/elevator-pos-deg", "aileron": "fcs/left-aileron-pos-deg",
                    "rudder": "fcs/rudder-pos-deg", "flap": "fcs/flap-pos-deg"}
PLATFORM_THROTTLES = 4   # the throttles the platform commands (ControlInputs::kMaxEngines)
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


MACH_LIFT, MACH_SIDE, MACH_DCM = "aero/function/mach-lift", "aero/function/mach-side", "aero/function/mach-dCm-dCL"
MACH_CD0, MACH_DK = "aero/function/mach-dCD0", "aero/function/mach-dK"
CL_COEFFICIENT = "aero/function/CL-base"


def _mach_function(name, description, mach, values):
    rows = "\n".join("            %8.3f %12.6f" % (m, v) for m, v in zip(mach, values))
    return """      <function name="%s">
        <description>%s</description>
        <table>
          <independentVar lookup="row">velocities/mach</independentVar>
          <tableData>
%s
          </tableData>
        </table>
      </function>""" % (name, description, rows)


def mach_channel(ch):
    return "aero/function/mach-" + ch


def aerodynamics_xml(tables, aircraft, ge_e=0.85):
    """The <aerodynamics> element from the tables (tables.build)."""
    a = tables["alpha"]
    b = tables["beta"]
    out = {axis: [] for axis in AXES.values()}
    qs = ["aero/qbar-psf", "metrics/Sw-sqft"]
    ge = tables["ground_effect"]
    ge_lift = _table1(ge["h_b"], ge["lift"], "aero/h_b-mac-ft", 8)
    mt = tables.get("mach")
    pre = []
    if mt is not None:
        m = mt["mach"]
        pre.append(_mach_function(MACH_LIFT, "lift, and damping, over its low-speed value (hangar: aero/mach.py)", m, mt["K_L"]))
        pre.append(_mach_function(MACH_SIDE, "side force, rolling and yawing moment over their low-speed values", m, mt["K_Y"]))
        pre.append(_mach_function(MACH_DCM, "pitching moment per unit low-speed lift: the neutral point's move", m, mt["dCm_dCL"]))
        pre.append(_mach_function(MACH_CD0, "zero-lift drag's change: friction and wave drag", m, mt["dCD0"]))
        pre.append(_mach_function(MACH_DK, "induced-drag factor's change (per CL^2)", m, mt["dK"]))
        for ch in tables["controls"]:
            pre.append(_mach_function(mach_channel(ch), "%s power over its low-speed value" % ch, m, mt["K_" + ch]))
        pad = " " * 6
        pre.append("%s<function name=\"%s\">\n%s  <description>CL over alpha and beta at low speed</description>\n%s\n%s</function>"
                   % (pad, CL_COEFFICIENT, pad, _table2(a, b, tables["base"]["CL"], "aero/alpha-deg", "aero/beta-deg", 8), pad))
    lift_k = [MACH_LIFT] if mt is not None else []
    side_k = [MACH_SIDE] if mt is not None else []
    for k, axis in AXES.items():
        factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else [])
        if k == "CL" and mt is not None:
            out[axis].append(_function("CL_base", "CL over alpha and beta, ground effect, compressibility (hangar)",
                                       factors + [CL_COEFFICIENT, MACH_LIFT], ge_lift))
            continue
        tab = _table2(a, b, tables["base"][k], "aero/alpha-deg", "aero/beta-deg", 8)
        if k == "CL":
            tab = tab + "\n" + ge_lift
        out[axis].append(_function("%s_base" % k, "%s over alpha and beta%s (hangar)" % (k, ", ground effect" if k == "CL" else ""),
                                   factors + (side_k if k in ("CY", "Cl", "Cn") else []), tab))
    if mt is not None:
        pad = " " * 6
        props = "\n".join("%s    <property>%s</property>" % (pad, f)
                           for f in qs + [REF_LENGTH["Cm"], CL_COEFFICIENT, MACH_DCM])
        out["PITCH"].append("%s<function name=\"aero/coefficient/Cm_mach\">\n%s  <description>Cm from the neutral point's move with Mach"
                            "</description>\n%s  <product>\n%s\n%s  </product>\n%s</function>" % (pad, pad, pad, props, pad, pad))
        for name, desc, fs in (("CD_mach", "zero-lift drag's change with Mach: friction, wave drag", qs + [MACH_CD0]),
                               ("CD_induced_mach", "induced drag's change with Mach (leading-edge suction lost)",
                                qs + ["aero/cl-squared", MACH_DK])):
            props = "\n".join("%s    <property>%s</property>" % (pad, f) for f in fs)
            out["DRAG"].append("%s<function name=\"aero/coefficient/%s\">\n%s  <description>%s</description>\n"
                               "%s  <product>\n%s\n%s  </product>\n%s</function>" % (pad, name, pad, desc, pad, props, pad, pad))
    for ch, t in tables["controls"].items():
        prop = CHANNEL_PROPERTY[ch]
        for k in t:
            if k == "deflection":
                continue
            factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else []) + ([mach_channel(ch)] if mt is not None else [])
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
            factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else []) + list(rate_prop[rate]) + lift_k
            out[AXES[k]].append(_function("%s%s" % (k, rate), "%s per %s (non-dimensional), over alpha" % (k, rate), factors,
                                          _table1(a, data, "aero/alpha-deg", 8)))
    ad = tables["alphadot"]
    for k in ("CL", "Cm"):
        factors = qs + ([REF_LENGTH[k]] if k in REF_LENGTH else []) + ["aero/ci2vel", "aero/alphadot-rad_sec", "value:%.5f" % ad[k]] + lift_k
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
    parts = ["    <aerodynamics>"] + ["\n".join("  " + line for line in f.split("\n")) for f in pre]
    for axis in ("DRAG", "SIDE", "LIFT", "ROLL", "PITCH", "YAW"):
        parts.append("      <axis name=\"%s\">" % axis)
        parts.extend("\n".join("  " + line for line in f.split("\n")) for f in out[axis])
        parts.append("      </axis>")
    parts.append("    </aerodynamics>")
    return "\n".join(parts)


def gear_loads(aircraft, mass, cg):
    """Static load (N) on each wheel with the aircraft at rest: pitch balance
    between the centre-line wheel(s) and the pairs. A single leg beside the
    centre line (the A-10's nose wheel, clear of its gun) is a centre wheel."""
    wheels = [(g, name, pos) for g in aircraft.gear for name, pos in g.positions()]
    if not wheels:
        return {}
    W = mass * G0
    single = [abs(w[2][1]) < 0.1 or not w[0].mirror for w in wheels]
    centre = [w for w, s in zip(wheels, single) if s]
    pairs = [w for w, s in zip(wheels, single) if not s]
    if not pairs:
        # a bicycle gear (the U-2's): the lever rule between the wheels ahead
        # of the CG and those behind it
        ahead = [w for w in centre if w[2][0] <= cg[0]]
        behind = [w for w in centre if w[2][0] > cg[0]]
        if ahead and behind:
            xa = np.mean([w[2][0] for w in ahead])
            xb = np.mean([w[2][0] for w in behind])
            fa = W * (xb - cg[0]) / (xb - xa)
            out = {name: fa / len(ahead) for _, name, _ in ahead}
            out.update({name: (W - fa) / len(behind) for _, name, _ in behind})
            return out
    if not centre or not pairs:
        return {name: W / len(wheels) for _, name, _ in wheels}
    xc = np.mean([w[2][0] for w in centre])
    xp = np.mean([w[2][0] for w in pairs])
    fc = W * (xp - cg[0]) / (xp - xc)
    fp = W - fc
    out = {name: fc / len(centre) for _, name, _ in centre}
    out.update({name: fp / len(pairs) for _, name, _ in pairs})
    return out


def gear_balance(aircraft, cg):
    """A tricycle gear's balance at rest (Raymer 11.2): (its parked pitch attitude,
    deg, nose up; the nose wheel's share of the weight, %; the tip-back angle, deg -
    off the vertical from the main wheels' contact to the CG) with the struts at their
    static deflection, or None for a tail wheel, a bicycle or no gear."""
    wheels = [(g, pos + np.array([0.0, 0.0, g.static_deflection])) for g in aircraft.gear for _, pos in g.positions()]
    single = [abs(p[1]) < 0.1 or not g.mirror for g, p in wheels]
    centre = [p for (_, p), s in zip(wheels, single) if s]
    pairs = [p for (_, p), s in zip(wheels, single) if not s]
    if not centre or not pairs:
        return None
    n, m = np.mean(centre, axis=0), np.mean(pairs, axis=0)
    if n[0] >= m[0]:
        return None
    th = math.atan2(m[2] - n[2], m[0] - n[0])  # nose up when the nose wheel's contact is the lower

    def level(p):  # (aft, up) in the ground's frame, the aircraft at its parked attitude
        return p[0] * math.cos(th) + p[2] * math.sin(th), p[2] * math.cos(th) - p[0] * math.sin(th)
    (xn, _), (xm, zm), (xc, zc) = level(n), level(m), level(np.asarray(cg, float))
    return math.degrees(th), (xm - xc) / (xm - xn) * 100.0, math.degrees(math.atan2(xm - xc, zc - zm))


def tail_down_deg(aircraft):
    """The pitch attitude (deg, nose up) at which the aircraft, pivoting on its
    aftmost wheels with the struts at their static deflection, touches the ground
    with the structure behind them - the most a rotation may take (the E-7A's
    7.3 deg). Structure already on the ground's level there (a skid) is not the
    tail. None for no gear, or nothing behind the wheels."""
    wheels = [pos + np.array([0.0, 0.0, g.static_deflection]) for g in aircraft.gear for _, pos in g.positions()]
    if not wheels:
        return None
    aft = max(p[0] for p in wheels)
    pivot = np.mean([p for p in wheels if p[0] > aft - 0.3], axis=0)
    angles = [math.degrees(math.atan2(p[2] - pivot[2], p[0] - pivot[0])) for _, p in structure_points(aircraft)
              if p[0] > pivot[0] + 0.1 and p[2] - pivot[2] > 0.02]
    return min(angles) if angles else None


def turn_radius_m(aircraft):
    """The tightest turn on its wheels (m), at the main wheels' middle: the wheelbase
    from the steerable wheel(s) to the others over the tangent of its steering limit.
    None for no steerable wheel, or a castering one (it turns by its brakes)."""
    steer = [(g, pos) for g in aircraft.gear for _, pos in g.positions() if g.steerable and 0.0 < g.max_steer_deg < 360.0]
    fixed = [pos for g in aircraft.gear for _, pos in g.positions() if not g.steerable]
    if not steer or not fixed:
        return None
    base = abs(np.mean([p[0] for _, p in steer]) - np.mean([p[0] for p in fixed]))
    limit = math.radians(min(g.max_steer_deg for g, _ in steer))
    return base / math.tan(limit) if base > 0.0 else None


def ground_reactions_xml(aircraft, mass_model):
    m, cg = mass_model.loaded()
    loads = gear_loads(aircraft, m, cg)
    parts = ["    <ground_reactions>"]
    for g in aircraft.gear:
        for name, pos in g.positions():
            f = max(loads.get(name, m * G0 / 3), 100.0)
            k = f / g.static_deflection
            # a steerable wheel behind the CG (a tail wheel) turns the other way to the
            # pedal's: the aircraft yaws the way the rudder yaws it (JSBSim's steer
            # angle is the command times max_steer; the U-2S's tail wheel, turning as a
            # nose wheel does, steered it off the runway against its rudder)
            steer = (g.max_steer_deg if pos[0] < cg[0] or g.max_steer_deg >= 360.0 else -g.max_steer_deg) if g.steerable else 0.0
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
      </contact>""" % (name, _loc(pos, 10), k, c, 2 * c, steer, brake, int(g.retractable)))
    # structure: the points that touch first in a crash, whatever the attitude
    for name, p, k_s, c_s in structure_contacts(aircraft, mass_model):
        parts.append("""      <contact type="STRUCTURE" name="%s">
        <location unit="M">
%s
        </location>
        <static_friction> 0.8 </static_friction>
        <dynamic_friction> 0.6 </dynamic_friction>
        <spring_coeff unit="N/M"> %.0f </spring_coeff>
        <damping_coeff unit="N/M/SEC"> %.0f </damping_coeff>
        <damping_coeff_rebound unit="N/M/SEC"> %.0f </damping_coeff_rebound>
      </contact>""" % (name, _loc(p, 10), k_s, c_s, c_s))
    parts.append("    </ground_reactions>")
    return "\n".join(parts)


# Structure contacts: stiff enough to hold the aircraft up, soft enough for
# the step to integrate. JSBSim integrates body rates with forward Euler at
# the platform's 120 Hz, and a contact far from the centre of gravity turns
# the aircraft about it: on a light airframe a stiff wing-tip spring makes a
# mode far faster than the step (the energy it gains throws the aircraft
# back into the air). So each contact's own mode is held to STRUCTURE_OMEGA
# rad/s (omega dt = 0.25 at 120 Hz), with the stiffness sized from the
# apparent mass at the point, and contacts close together (the nose, a gear
# leg and the spinner) share that budget: on one mode their springs and
# dampers add up.
STRUCTURE_OMEGA = 30.0
STRUCTURE_ZETA = 0.5
STRUCTURE_SHARE = 0.10  # contacts closer than this times the aircraft's size share


def structure_points(aircraft, directions=1500, depth=0.02, more=()):
    """The points that touch the ground first, whatever the attitude.

    For directions spread over the sphere, the airframe's farthest point in
    that direction is a support point of its convex hull; a flat ground
    meets the hull there first. A point is kept unless one already kept is
    within `depth` (times the aircraft's largest dimension) of being as far
    out in every direction it serves: a fin top 0.3 m above the boom end
    stays, the corners of a thin wing tip merge. Points along the keel and
    top of every body and the edges of every surface are added where they
    lie on the hull (flat or straight there, so few directions find them).
    A symmetric aircraft is done for its right half and mirrored. Wheels are
    the gear's contacts; a propeller counts as its disc. `more` - [(name,
    point)] - are more of the airframe's points (a rotorcraft's hubs, the
    ends of its struts).
    Returns [(name, point)]."""
    verts, labels = [], []
    for name, _, v, _, _ in aircraft.mesh(fine=False):
        verts.append(v)
        labels += [name] * len(v)
    for name, p in more:
        verts.append(np.asarray(p, float)[None, :])
        labels.append(name)
    for e in aircraft.engines:
        if not e.has_propeller:
            continue
        _, pitch, yaw = e.prop_orient
        axis = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), -np.sin(pitch)])
        a = np.cross(axis, [0.0, 0.0, 1.0])
        a /= np.linalg.norm(a)
        b = np.cross(axis, a)
        th = np.linspace(0.0, 2 * np.pi, 4, endpoint=False)
        for name, _, prop, _ in e.copies():
            disc = prop + 0.5 * e.prop_diameter * (np.cos(th)[:, None] * a + np.sin(th)[:, None] * b)
            verts.append(disc)
            labels += ["%s propeller" % name] * len(disc)
    V = np.vstack(verts)
    size = float(np.ptp(V, axis=0).max())
    tol = depth * size
    k = np.arange(directions) + 0.5
    polar = np.arccos(1.0 - 2.0 * k / directions)
    azimuth = np.pi * (1.0 + 5.0 ** 0.5) * k
    D = np.stack([np.cos(azimuth) * np.sin(polar), np.sin(azimuth) * np.sin(polar), np.cos(polar)], axis=1)
    support = V @ D.T
    reach = support.max(axis=0)  # the hull's extent in each direction
    symmetric = (all(s.mirror or abs(s.sections[0].le[1]) < 1e-6 for s in aircraft.surfaces)
                 and all(b.mirror or abs(float(np.max(np.abs(b.y)))) < 1e-6 for b in aircraft.bodies)
                 and all(e.mirror or abs(e.prop_position[1]) < 1e-6 for e in aircraft.engines)
                 and all(abs(float(np.asarray(p, float)[1])) < 1e-6 for _, p in more))
    serve = D[:, 1] >= -1e-9 if symmetric else np.ones(len(D), bool)  # the right half's directions
    best = np.argmax(support, axis=0)
    chosen, counts = np.unique(best[serve], return_counts=True)
    kept = []
    for i in chosen[np.argsort(-counts, kind="stable")]:  # the most-used first
        i = int(i)
        mine = serve & (best == i)
        if all(float(np.max(support[i, mine] - support[j, mine])) > tol for j in kept):
            kept.append(i)
    pts = [(labels[i], V[i]) for i in kept]
    extra = []
    for body in aircraft.bodies:
        for side in body.copies()[:1]:
            for x in np.linspace(body.x[0], body.x[-1], 7)[1:-1]:
                w, top, bottom, yc, _ = body.section(x)
                y = side * abs(yc) if body.mirror else yc
                extra += [("%s bottom" % body.name, np.array([x, y, bottom])), ("%s top" % body.name, np.array([x, y, top]))]
    for surf in aircraft.surfaces:
        for eta in (0.35, 0.7):
            le, chord, _, _ = surf.station(eta)
            c, _ = surf.frame(eta)
            extra += [(surf.name, le), (surf.name, le + chord * c)]
    for label, q in extra:
        gain = D @ q - reach  # how far short of the hull, per direction
        best_dir = int(np.argmax(gain))
        if gain[best_dir] < -0.005 * size:
            continue  # inside the hull: never touches first
        if all(float(D[best_dir] @ (q - r)) > tol for _, r in pts):
            pts.append((label, q))
    if symmetric:
        flip = np.array([1.0, -1.0, 1.0])
        pts = [(label, q * np.array([1.0, 0.0, 1.0]) if abs(q[1]) < tol else q) for label, q in pts]
        pts += [(label, q * flip) for label, q in pts if q[1] > 0.0]
    centre = 0.5 * (V.min(axis=0) + V.max(axis=0))
    named, seen = [], {}
    for label, q in sorted(pts, key=lambda lp: (lp[0], lp[1][1], lp[1][0], lp[1][2])):
        d = q - centre
        words = [label]
        if abs(q[1]) > 0.02 * size:
            words.append("left" if q[1] < 0 else "right")
        words.append(("front" if d[0] < 0 else "rear") if abs(d[0]) >= abs(d[2]) else ("top" if d[2] > 0 else "bottom"))
        name = " ".join(words)
        seen[name] = seen.get(name, 0) + 1
        named.append((name if seen[name] == 1 else "%s %d" % (name, seen[name]), q))
    return named


def apparent_mass(r, mass, inertia):
    """The smallest mass a force at r (from the centre of gravity) meets, over
    all directions: 1 / the largest eigenvalue of I/m - [r]x J^-1 [r]x."""
    R = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
    A = np.eye(3) / mass - R @ np.linalg.solve(inertia, R)
    return 1.0 / float(np.linalg.eigvalsh(0.5 * (A + A.T)).max())


def structure_contacts(aircraft, mass_model):
    """[(name, point, spring N/m, damping N s/m)], sized on the empty
    aircraft (the lightest, so the fastest)."""
    e = mass_model.empty()
    J = np.array([[e["ixx"], -e["ixy"], -e["ixz"]], [-e["ixy"], e["iyy"], -e["iyz"]], [-e["ixz"], -e["iyz"], e["izz"]]])
    points = structure_points(aircraft)
    wheels = [pos for g in aircraft.gear for _, pos in g.positions()]
    lo, hi = aircraft.extent()
    near = STRUCTURE_SHARE * float(np.max(hi - lo))
    everything = np.array([p for _, p in points] + wheels)
    out = []
    for name, p in points:
        m_eff = apparent_mass(p - e["cg"], e["mass"], J)
        share = int(np.sum(np.linalg.norm(everything - p, axis=1) < near))  # itself included
        out.append((name, p, m_eff * STRUCTURE_OMEGA ** 2 / share, 2.0 * STRUCTURE_ZETA * m_eff * STRUCTURE_OMEGA / share))
    return out


def _yaw_damper_xml(yd, lo, hi):
    """The yaw damper's part of the Yaw channel: the yaw rate washed out,
    times the gain over dynamic pressure, against it; half the rudder's
    travel at most, the pilot's command added to it."""
    rows = "\n".join("              %8.1f  %9.5f" % (q, k) for q, k in zip(yd["qbar_psf"], yd["k"]))
    half = 0.5 * max(abs(lo), abs(hi)) * 0.0174533
    return """        <!-- the yaw damper (hangar/fcs.py yaw_damper): the yaw rate, washed out so a
             steady turn's is left alone, against itself through the rudder - the
             dutch roll damped to %.2f at 1 g across the speed range -->
        <washout_filter name="fcs/yaw-damper-washout">
          <input>velocities/r-rad_sec</input>
          <c1>%.3f</c1>
        </washout_filter>
        <fcs_function name="fcs/yaw-damper">
          <function>
            <product>
              <value>-1</value>
              <table>
                <independentVar lookup="row">aero/qbar-psf</independentVar>
                <tableData>
%s
                </tableData>
              </table>
              <property>fcs/yaw-damper-washout</property>
            </product>
          </function>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </fcs_function>
        <summer name="fcs/rudder-sum">
          <input>fcs/rudder-control</input>
          <input>fcs/yaw-damper</input>
          <clipto> <min>%.5f</min> <max>%.5f</max> </clipto>
        </summer>""" % (yd["zeta"], 1.0 / YAW_DAMPER_WASHOUT_S, rows, -half, half, lo * 0.0174533, hi * 0.0174533)


def _centre_brake_xml():
    """The platform brakes left and right: a centre wheel (a bicycle gear's
    main, brake = "center") takes both pedals."""
    return """      <channel name="Brakes">
        <!-- the platform brakes left and right: the centre wheel takes both -->
        <fcs_function name="fcs/center-brake">
          <function>
            <product>
              <value>0.5</value>
              <sum>
                <property>fcs/left-brake-cmd-norm</property>
                <property>fcs/right-brake-cmd-norm</property>
              </sum>
            </product>
          </function>
          <output>fcs/center-brake-cmd-norm</output>
        </fcs_function>
      </channel>"""


def flight_control_xml(aircraft, fbw=None, yaw_damper=None, autopilot=None, profile=None):
    ch = set(aircraft.channels())
    lim = {k: aircraft.channel_limits(k) for k in ch}
    parts = ["    <flight_control name=\"%s\">" % aircraft.name]
    if profile:
        from .profile import properties_xml as profile_xml
        parts.append(profile_xml(profile).rstrip("\n"))
    if autopilot:
        from .autopilot import properties_xml
        parts.append(properties_xml(autopilot).rstrip("\n"))
    if any(g.brake == "center" for g in aircraft.gear):
        parts.append(_centre_brake_xml())
    if fbw is not None:
        from .fcs import channels_xml
        parts.extend(channels_xml(aircraft, fbw))
        ch = ch - {"elevator", "aileron", "rudder"}
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
        </aerosurface_scale>%s
        <actuator name="fcs/rudder-actuator">
          <input>%s</input>
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
      </channel>""" % (lo, hi, "\n" + _yaw_damper_xml(yaw_damper, lo, hi) if yaw_damper is not None else "",
                       "fcs/rudder-sum" if yaw_damper is not None else "fcs/rudder-control"))
    if "flap" in ch:
        lo, hi = lim["flap"]
        steps = [0.0, 0.25, 0.5, 0.75, 1.0]
        settings = "\n".join("            <setting> <position>%.1f</position> <time>%.1f</time> </setting>" % (s * hi, 2.0 if s else 0.0)
                             for s in steps)
        # and the position as a fraction of the travel, JSBSim's fcs/flap-pos-norm (which JSBSim
        # declares but does not derive): what fsim reads to know the flaps are where they were told
        parts.append("""      <channel name="Flaps">
        <kinematic name="fcs/flaps-control">
          <input>fcs/flap-cmd-norm</input>
          <traverse>
%s
          </traverse>
          <output>fcs/flap-pos-deg</output>
        </kinematic>
        <pure_gain name="fcs/flaps-normalized">
          <input>fcs/flap-pos-deg</input>
          <gain>%.6g</gain>
          <output>fcs/flap-pos-norm</output>
        </pure_gain>
      </channel>""" % (settings, 1.0 / hi if hi > 0.0 else 0.0))
    devices = [d for _, d in aircraft.leading_devices()]
    if devices:
        # the leading-edge flaps on their schedule, a alpha - m qbar/p + b deg,
        # through an actuator as fast as the F-16's (a 0.136 s lag, 25 deg/s;
        # NASA TP-1538) - fcs/lef-pos-deg, which the simulation reports and the
        # 3D model's flaps follow (each within its own stops)
        a, m, b = devices[0].schedule
        lo, hi = min(d.min_deg for d in devices), max(d.max_deg for d in devices)
        parts.append("""      <channel name="Leading-Edge Flaps">
        <fcs_function name="fcs/lef-schedule">
          <function>
            <sum>
              <product> <value>%.6g</value> <property>aero/alpha-deg</property> </product>
              <product> <value>%.6g</value>
                <quotient> <property>aero/qbar-psf</property> <property>atmosphere/P-psf</property> </quotient>
              </product>
              <value>%.6g</value>
            </sum>
          </function>
          <clipto> <min>%.6g</min> <max>%.6g</max> </clipto>
        </fcs_function>
        <actuator name="fcs/lef-actuator">
          <input>fcs/lef-schedule</input>
          <lag>7.35</lag>
          <rate_limit>25</rate_limit>
          <output>fcs/lef-pos-deg</output>
        </actuator>
      </channel>""" % (a, -m, b, lo, hi))
    # the throttle lever of an afterburning turbofan: military power at the
    # detent (80 %), the afterburner above it - JSBSim's turbine (augmethod 2)
    # takes positions 1..2 for that. The platform commands PLATFORM_THROTTLES
    # engines: an engine past them (the B-52's fifth to eighth) follows the
    # lever of the one that many before it - the same side's, as the engines
    # are numbered left and right in turn
    for i, (e, _) in enumerate(_engine_units(aircraft)):
        lever = i % PLATFORM_THROTTLES
        if e.type == "turbofan" and e.thrust_wet_kn:
            parts.append("""      <channel name="Throttle %d">
        <fcs_function name="fcs/throttle-lever-%d">
          <function>
            <table>
              <independentVar lookup="row">fcs/throttle-cmd-norm[%d]</independentVar>
              <tableData>
                0.0  0.0
                0.8  1.0
                1.0  2.0
              </tableData>
            </table>
          </function>
          <output>fcs/throttle-pos-norm[%d]</output>
        </fcs_function>
      </channel>""" % (i, i, lever, i))
        elif i >= PLATFORM_THROTTLES:
            parts.append("""      <channel name="Throttle %d">
        <pure_gain name="fcs/throttle-lever-%d">
          <input>fcs/throttle-cmd-norm[%d]</input>
          <gain>1</gain>
          <output>fcs/throttle-pos-norm[%d]</output>
        </pure_gain>
      </channel>""" % (i, i, lever, i))
    parts.extend(battery_xml(aircraft))
    parts.extend(governor_xml(aircraft))
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
    from .fcs import vectoring_xml
    tvc = vectoring_xml(aircraft)   # after the surfaces: the nozzles turn with them
    if tvc:
        parts.append(tvc)
    parts.append("    </flight_control>")
    return "\n".join(parts)


def battery_xml(aircraft):
    """The battery of an electric aircraft ([battery] capacity_wh): the power
    it gives - each motor's volts (the battery's, times its throttle) times its
    current, Drela's model's electrical power with the coil's and friction's
    losses in it, and none back into the battery - integrated into the energy
    used (fsim/battery/used-j), the charge left (fsim/battery/charge-j, of
    fsim/battery/capacity-j); the motors stop once it is spent
    (fsim/battery/supply, exactly 1 until then: a flight is the same bit for
    bit before). The power is the one the motors drew in the step before -
    the throttle they were given then and the current they took - as the
    flight controls run before the engines."""
    battery = aircraft.spec.get("battery")
    units = [(i, e) for i, (e, _) in enumerate(_engine_units(aircraft)) if e.type == "electric"]
    if not battery or not units:
        return []
    from .propulsion import motor_volts
    cap = float(battery["capacity_wh"]) * 3600.0
    terms = ["""            <product> <value>%.4g</value> <property>fcs/throttle-supplied-%d</property>
              <max> <value>0</value> <property>propulsion/engine[%d]/current-amperes</property> </max> </product>"""
             % (motor_volts(e), i, i) for i, e in units]
    power = terms[0] if len(terms) == 1 else "            <sum>\n%s\n            </sum>" % "\n".join(terms)
    supplied = "\n".join("""        <fcs_function name="fcs/throttle-supplied-%d">
          <function> <product> <property>fcs/throttle-pos-norm[%d]</property> <property>fsim/battery/supply</property> </product> </function>
          <output>fcs/throttle-pos-norm[%d]</output>
        </fcs_function>""" % (i, i, i) for i, _ in units)
    return ["""      <property value="%.6g">fsim/battery/capacity-j</property>
      <channel name="Battery">
        <!-- %.0f Wh: the power the motors drew, the energy used, the charge left, their supply while there is some -->
        <fcs_function name="fsim/battery/power-w">
          <function>
%s
          </function>
        </fcs_function>
        <integrator name="fsim/battery/used-j"> <input> fsim/battery/power-w </input> <c1> 1 </c1> </integrator>
        <fcs_function name="fsim/battery/charge-j">
          <function> <max> <value> 0 </value> <difference> <property> fsim/battery/capacity-j </property>
            <property> fsim/battery/used-j </property> </difference> </max> </function>
        </fcs_function>
        <fcs_function name="fsim/battery/supply">
          <function> <gt> <property> fsim/battery/charge-j </property> <value> 0 </value> </gt> </function>
        </fcs_function>
%s
      </channel>""" % (cap, float(battery["capacity_wh"]), power, supplied)]


def _engine_units(aircraft):
    """Every engine JSBSim sees, in its order: (engine, copy name)."""
    return [(e, name) for e in aircraft.engines for name, _, _, _ in e.copies()]


# The propeller governor. JSBSim's constant-speed propeller turns its blades
# 1 deg/s for every rpm it runs from the speed it aims at, minrpm + (maxrpm -
# minrpm) x its advance command: an integral governor. On a heavy propeller
# that alone rings - the blades' power against the propeller's inertia - so
# the command aims GOVERNOR_LEAD_S times the speed's rate of change below the
# governed speed: the proportional part that damps it. The rate is the power
# the engine gives less the power the propeller takes, over its inertia.
#
# When time stands still, JSBSim finds the engines' steady state by marching
# them in half-second steps (FGPropulsion::GetSteadyState: the start, every
# reset, every trim), where the same governor overshoots from stop to stop
# and leaves the propeller anywhere (a C-130J's at twice its speed, its
# thrust reversed). So while the simulation's dt is 0 the propeller is held
# at a blade angle, the one at which it takes the engine's power at its
# governed speed there (propulsion.blade_angles_for_power): the steady state
# is then the governed one, and the governor takes over as time runs.
GOVERNOR_LEAD_S = 0.15


def governor_xml(aircraft):
    """The channels that hold each turboprop's propeller at its governed
    speed, through JSBSim's fcs/advance-cmd-norm (propulsion.GOVERNOR_RANGE),
    and set its blades while time stands still; and its ground range
    (propulsion.ground_range): on the ground, at the bottom of the throttle,
    the blades set by the throttle and N1 holding the propeller's speed."""
    from .propulsion import (GOVERNOR_CP, GOVERNOR_RANGE, HP, TP_IDLE_N1, TP_SPOOL_S, TP_SUSTAIN_N1, Propeller,
                             blade_angles_for_power, blades_mass, ground_range, power_lapse_xml)
    lo, hi = GOVERNOR_RANGE
    out = []
    for i, (e, name) in enumerate(_engine_units(aircraft)):
        if e.type != "turboprop":
            continue
        p = Propeller(e)
        tab = p.pitch_tables()
        angles = blade_angles_for_power(tab)
        # spin inertia (slug ft2) x (2 pi / 60)^2: power over it and the rpm is rpm/s
        inertia = p.inertia(blades_mass(e)) * 0.73756 * (2 * math.pi / 60.0) ** 2
        n = p.rpm / 60.0
        d_ft = p.D / 0.3048
        s3 = (TP_SUSTAIN_N1 / 100.0) ** 3
        head = "                " + "".join("%8.2f" % c for c in GOVERNOR_CP)
        rows = "\n".join("            %6.2f " % j + "".join("%8.2f" % b for b in row) for j, row in zip(tab["J"], angles))
        g = ground_range(e)
        n1_rows = "\n".join("                      %8.2f %9.4f" % (b, v) for b, v in zip(g["blade_angle_deg"], g["n1"]))
        out.append("""      <channel name="Propeller Governor %(i)d">
        <!-- %(name)s: the propeller governed at %(rpm).0f rpm. The ground range: with weight
             on the wheels and time running, a throttle below %(beta_t)g sets the blades, from
             ground idle's %(gi).2f deg (no thrust standing still) to the low stop, %(lo_deg).1f,
             and N1 holds the propeller's speed - not below %(min_n1).1f %%, nor above the flight
             range's N1 at the throttle; past %(os_rpm).0f rpm the blades open further -->
        <fcs_function name="fcs/propeller-ground-range-%(i)d">
          <function>
            <and>
              <gt> <property>simulation/dt</property> <value>0</value> </gt>
              <gt> <property>gear/wow</property> <value>0</value> </gt>
              <lt> <property>fcs/throttle-cmd-norm[%(i)d]</property> <value>%(beta_t)g</value> </lt>
            </and>
          </function>
        </fcs_function>
        <fcs_function name="fcs/propeller-beta-blade-%(i)d">
          <function>
            <sum>
              <value>%(gi).4f</value>
              <product>
                <value>%(beta_gain).6g</value>
                <max> <value>0</value> <property>fcs/throttle-cmd-norm[%(i)d]</property> </max>
              </product>
            </sum>
          </function>
          <clipto> <min>%(gi).4f</min> <max>%(lo_deg).4f</max> </clipto>
        </fcs_function>
        <!-- the fuel control: the N1 that holds the governed speed standing still with the
             blades there (sea level), less %(k_n1).4g %% for each rpm the propeller runs fast -->
        <fcs_function name="fcs/propeller-beta-n1-%(i)d">
          <function>
            <min>
              <sum>
                <value>%(idle_n1).4f</value>
                <product>
                  <value>%(n1_span).4f</value>
                  <max> <value>0</value> <property>fcs/throttle-cmd-norm[%(i)d]</property> </max>
                </product>
              </sum>
              <max>
                <value>%(min_n1).4f</value>
                <sum>
                  <table>
                    <independentVar>fcs/propeller-beta-blade-%(i)d</independentVar>
                    <tableData>
%(n1_rows)s
                    </tableData>
                  </table>
                  <product>
                    <value>%(k_n1).6g</value>
                    <difference> <value>%(rpm).4f</value> <property>propulsion/engine[%(i)d]/propeller-rpm</property> </difference>
                  </product>
                </sum>
              </max>
            </min>
          </function>
        </fcs_function>
        <!-- JSBSim's turboprop drops its N1 to idle as time runs again after standing still (a
             reset, a trim; its power reads 0 then): in that step the throttle puts N1 back where
             it was, or in the ground range where the fuel control wants it -->
        <fcs_function name="fcs/propeller-throttle-%(i)d">
          <function>
            <ifthen>
              <gt> <property>fcs/propeller-ground-range-%(i)d</property> <value>0</value> </gt>
              <quotient>
                <difference>
                  <ifthen>
                    <lt> <property>propulsion/engine[%(i)d]/power-hp</property> <value>0.5</value> </lt>
                    <ifthen>
                      <gt> <property>fcs/propeller-beta-n1-%(i)d</property> <value>%(idle_n1).4f</value> </gt>
                      <sum>
                        <value>%(idle_n1).4f</value>
                        <quotient>
                          <difference> <property>fcs/propeller-beta-n1-%(i)d</property> <value>%(idle_n1).4f</value> </difference>
                          <difference>
                            <value>1</value>
                            <exp> <quotient> <property>simulation/dt</property> <value>%(neg_spool).4f</value> </quotient> </exp>
                          </difference>
                        </quotient>
                      </sum>
                      <quotient>
                        <difference>
                          <property>fcs/propeller-beta-n1-%(i)d</property>
                          <product>
                            <value>%(idle_n1).4f</value>
                            <exp> <quotient> <property>simulation/dt</property> <value>%(neg_spool_down).4f</value> </quotient> </exp>
                          </product>
                        </difference>
                        <difference>
                          <value>1</value>
                          <exp> <quotient> <property>simulation/dt</property> <value>%(neg_spool_down).4f</value> </quotient> </exp>
                        </difference>
                      </quotient>
                    </ifthen>
                    <property>fcs/propeller-beta-n1-%(i)d</property>
                  </ifthen>
                  <value>%(idle_n1).4f</value>
                </difference>
                <value>%(n1_span).4f</value>
              </quotient>
              <ifthen>
                <and>
                  <gt> <property>simulation/dt</property> <value>0</value> </gt>
                  <lt> <property>propulsion/engine[%(i)d]/power-hp</property> <value>0.5</value> </lt>
                </and>
                <quotient>
                  <difference> <property>propulsion/engine[%(i)d]/n1</property> <value>%(idle_n1).4f</value> </difference>
                  <product>
                    <value>%(n1_span).4f</value>
                    <difference>
                      <value>1</value>
                      <exp> <quotient> <property>simulation/dt</property> <value>%(neg_spool).4f</value> </quotient> </exp>
                    </difference>
                  </product>
                </quotient>
                <property>fcs/throttle-cmd-norm[%(i)d]</property>
              </ifthen>
            </ifthen>
          </function>
          <output>fcs/throttle-pos-norm[%(i)d]</output>
        </fcs_function>
        <!-- the speed's rate of change: the power the engine gives the propeller less the
             power it takes, over its spin inertia (rpm/s); none read while the engine's
             power reads 0 (time stood still) -->
        <fcs_function name="fcs/propeller-rpm-rate-%(i)d">
          <function>
            <product>
              <ge> <property>propulsion/engine[%(i)d]/power-hp</property> <value>0.5</value> </ge>
              <quotient>
                <difference>
                  <product> <property>propulsion/engine[%(i)d]/power-hp</property> <value>550</value> </product>
                  <property>propulsion/engine[%(i)d]/propeller-power-ftlbps</property>
                </difference>
                <max>
                  <value>%(inertia_min).6g</value>
                  <product> <value>%(inertia).6g</value> <property>propulsion/engine[%(i)d]/propeller-rpm</property> </product>
                </max>
              </quotient>
            </product>
          </function>
        </fcs_function>
        <fcs_function name="fcs/propeller-advance-%(i)d">
          <function>
            <quotient>
              <difference>
                <value>%(a0).1f</value>
                <product>
                  <gt> <property>simulation/dt</property> <value>0</value> </gt>
                  <value>%(lead).4f</value>
                  <property>fcs/propeller-rpm-rate-%(i)d</property>
                </product>
              </difference>
              <value>%(span).1f</value>
            </quotient>
          </function>
          <clipto> <min>0</min> <max>1</max> </clipto>
          <output>fcs/advance-cmd-norm[%(i)d]</output>
        </fcs_function>
        <!-- time standing still: the blades held where they take the engine's power; in the
             ground range: set -->
        <fcs_function name="fcs/propeller-governing-%(i)d">
          <function>
            <and>
              <gt> <property>simulation/dt</property> <value>0</value> </gt>
              <lt> <property>fcs/propeller-ground-range-%(i)d</property> <value>1</value> </lt>
            </and>
          </function>
          <output>propulsion/engine[%(i)d]/constant-speed-mode</output>
        </fcs_function>
        <fcs_function name="fcs/propeller-advance-ratio-%(i)d">
          <function>
            <max> <value>0</value> <quotient> <property>velocities/u-aero-fps</property> <value>%(nd).4f</value> </quotient> </max>
          </function>
        </fcs_function>
        <!-- the engine's power coefficient at the governed speed: its core at the throttle's
             N1 (the full one at a start: JSBSim starts engines at full throttle), the lapse -->
        <fcs_function name="fcs/propeller-power-coefficient-%(i)d">
          <function>
            <quotient>
              <min>
                <value>%(rating).1f</value>
                <product>
                  <value>%(thermo).1f</value>
                  <max>
                    <value>0</value>
                    <quotient>
                      <difference>
                        <pow>
                          <sum>
                            <value>%(idle).4f</value>
                            <product>
                              <value>%(span_n1).4f</value>
                              <ifthen>
                                <gt> <property>fcs/throttle-pos-norm[%(i)d]</property> <value>0.001</value> </gt>
                                <property>fcs/throttle-pos-norm[%(i)d]</property>
                                <value>1</value>
                              </ifthen>
                            </product>
                          </sum>
                          <value>3</value>
                        </pow>
                        <value>%(s3).6f</value>
                      </difference>
                      <value>%(s3c).6f</value>
                    </quotient>
                  </max>
%(lapse)s
                </product>
              </min>
              <product> <property>atmosphere/rho-slugs_ft3</property> <value>%(scale).6g</value> </product>
            </quotient>
          </function>
        </fcs_function>
        <fcs_function name="fcs/propeller-blade-angle-%(i)d">
          <function>
            <ifthen>
              <gt> <property>simulation/dt</property> <value>0</value> </gt>
              <ifthen>
                <gt> <property>fcs/propeller-ground-range-%(i)d</property> <value>0</value> </gt>
                <min>
                  <value>%(hi_deg).4f</value>
                  <sum>
                    <property>fcs/propeller-beta-blade-%(i)d</property>
                    <product>
                      <value>%(k_blade).6g</value>
                      <max>
                        <value>0</value>
                        <difference> <property>propulsion/engine[%(i)d]/propeller-rpm</property> <value>%(os_rpm).4f</value> </difference>
                      </max>
                    </product>
                  </sum>
                </min>
                <property>propulsion/engine[%(i)d]/blade-angle</property>
              </ifthen>
              <table>
                <independentVar lookup="row">fcs/propeller-advance-ratio-%(i)d</independentVar>
                <independentVar lookup="column">fcs/propeller-power-coefficient-%(i)d</independentVar>
                <tableData>
%(head)s
%(rows)s
                </tableData>
              </table>
            </ifthen>
          </function>
          <output>propulsion/engine[%(i)d]/blade-angle</output>
        </fcs_function>
      </channel>""" % {
            "i": i, "name": name, "rpm": e.prop_rpm, "lead": GOVERNOR_LEAD_S, "idle_n1": TP_IDLE_N1,
            "n1_span": 100.0 - TP_IDLE_N1, "neg_spool": -TP_SPOOL_S,
            "inertia": inertia, "inertia_min": inertia * 0.1 * e.prop_rpm,
            "a0": (1.0 - lo) * e.prop_rpm, "span": (hi - lo) * e.prop_rpm,
            "nd": n * d_ft, "rating": e.power_kw * 1000.0 / HP, "thermo": e.thermo_power_kw * 1000.0 / HP,
            "idle": TP_IDLE_N1 / 100.0, "span_n1": 1.0 - TP_IDLE_N1 / 100.0, "s3": s3, "s3c": 1.0 - s3,
            "lapse": power_lapse_xml(e, 18), "scale": n**3 * d_ft**5 / 550.0, "head": head, "rows": rows,
            "beta_t": g["throttle"], "gi": g["ground_idle_deg"], "lo_deg": g["low_stop_deg"], "hi_deg": g["high_stop_deg"],
            "beta_gain": (g["low_stop_deg"] - g["ground_idle_deg"]) / g["throttle"], "min_n1": g["min_n1"],
            "k_n1": g["k_n1"], "k_blade": g["k_blade"], "os_rpm": g["overspeed_rpm"], "n1_rows": n1_rows,
            "neg_spool_down": -2.4 * TP_SPOOL_S})
    return out


def propulsion_xml(aircraft, mass_model, engine_files):
    parts = ["    <propulsion>"]
    n_tanks = len(mass_model.tanks)
    for e, (eng_file, prop_file) in zip(aircraft.engines, engine_files):
        for name, pos, prop, sense in e.copies():
            feeds = "\n".join("        <feed>%d</feed>" % i for i in range(n_tanks)) if e.type in ("piston", "turbofan", "turboprop") else ""
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
%s
        </thruster>
      </engine>""" % (eng_file, _loc(pos, 10), feeds, prop_file, _loc(prop, 12), orient[0], orient[1], orient[2],
                      "" if e.type == "turbofan" else "          <sense>%d</sense>" % (-1 if sense == "cw" else 1)))
    for t in mass_model.tanks:
        cap = float(t.get("capacity", 0.0))
        parts.append("""      <tank type="FUEL">
        <location unit="M">
%s
        </location>
        <capacity unit="KG"> %.2f </capacity>
        <contents unit="KG"> %.2f </contents>
      </tank>""" % (_loc(np.asarray(t["position"], float), 10), cap, mass_model.fuel(t)))
    parts.append("    </propulsion>")
    return "\n".join(parts)


def aircraft_xml(aircraft, tables, mass_model, engine_files, notes="", fbw=None, yaw_damper=None, autopilot=None, profile=None):
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
%s    </fileheader>
""" % (__version__, os.path.basename(a.path or a.name), datetime.date.today().isoformat(), notes, a.name,
       datetime.date.today().isoformat(), __version__, a.description or a.name, references_xml(a.spec, "      "))
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
                    propulsion_xml(a, mass_model, engine_files), "\n", flight_control_xml(a, fbw, yaw_damper, autopilot, profile), "\n",
                    aerodynamics_xml(tables, a), "\n</fdm_config>\n"])
