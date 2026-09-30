"""A multirotor as a JSBSim aircraft, from a design file (kind = "multirotor"): <name>.xml and
its Engines/, flying the published motor and rotor equations.

Each motor's speed is a state of the flight control system, w' = (w_cmd - w)/tau, with the
design's lags up and down; its thrust k_T w^2 is a JSBSim direct thruster's (an electric engine,
whose thrust is its power in ft lb/s, metered by fcs/throttle-pos-norm[i] = thrust / maximum);
the reaction torques k_Q w^2, the rotor drag and the ground effect are aerodynamic functions.
The battery ([battery] capacity_wh, hover_endurance_min): the power it gives is the published hover's
(its capacity over its flight time, taken as a hover's) scaled by the rotors' physics - a rotor's
power grows as its speed cubed - integrated into the energy used (fsim/battery/used-j), the charge
left (fsim/battery/charge-j, of fsim/battery/capacity-j); the motors stop once it is spent
(fsim/battery/supply, exactly 1 until then: a flight is the same bit for bit before).
The command: each motor's thrust (0..1 of its maximum; throttle[i]), roll, pitch and yaw mixed in
(aileron + right, elevator + nose down, rudder + nose left), so a policy flies the motors directly
or through the mixer. The speed each motor commands is sqrt(thrust): the thrust is linear in the
command, as an ESC with thrust linearisation gives it. The rotors' speeds are reported as
propulsion/engine[i]/rotor-rpm, which the platform shows as the engines' rpm.
It stands on its feet (BOGEY contacts) and, turned over, on its rotors' hubs and its top (STRUCTURE
contacts), their springs and dampers sized for the platform's step (contact_set).
"""
import math
import os

import numpy as np

from ..applicability import references_xml

N_PER_LBF, M_PER_FT, KG_PER_SLUG, LB_PER_KG = 4.448222, 0.3048, 14.593903, 2.204623
RPM_PER_RADS = 60 / (2 * math.pi)
#: a full roll, pitch or yaw command moves each motor's thrust by this much (of its maximum)
MIX = 0.25

# The ground contacts, sized for the step. JSBSim applies a contact's spring and damper, as they are at the
# step's start, for the platform's whole step, and integrates what they do with Adams-Bashforth 2 (the
# airframe's velocity) and forward Euler (its body rates). Too stiff or too damped for that and a contact adds
# energy: the Crazyflie's feet, 60 N/m and 1.2 N s/m each on 27 g, made it hop parked (docs/rotorcraft.md, 7).
# The contacts that stand on the ground together - the feet, or upside down the tops - move the airframe as
# one set of modes: heave, roll and pitch. Their springs put the set's fastest mode at CONTACT_OMEGA_DT
# (omega dt: 72 rad/s at 120 Hz), inside Adams-Bashforth 2's stable range with margin (with damping it ends
# near 0.7; Euler's, damped at CONTACT_ZETA, near 1.4), and their dampers damp that mode at CONTACT_ZETA:
# less leaves a bounce, and more, applied for a whole step, reverses the speed it damps (the Crazyflie,
# damped at 1.0, left the ground faster than it struck). The slower modes are damped less, as the dampers
# follow the springs. Measured (docs/rotorcraft.md, 10): at 0.65 the Crazyflie left the ground as fast as
# it struck it, and softer than 0.6 the IRIS+ fallen on its side kept rocking.
STEP_S = 1.0 / 120.0  # the platform's JSBSim step (fly.FDM_DT)
CONTACT_OMEGA_DT = 0.6
CONTACT_ZETA = 0.7
#: the legs' static deflection a design may reach before the build warns: a quarter of their length
LEG_SAG_SHARE = 0.25


def _f(x):
    return "%.8g" % x


def rotors(spec):
    """[(label, x forward m, y left m, sense +1 counter-clockwise from above)], k_T and k_Q
    (N and N m per (rad/s)^2), the maximum speed (rad/s)."""
    r = spec["rotors"]
    if "layout" in r:
        a = r["arm_m"] / math.sqrt(2)
        rows = [(lab, sx * a, sy * a, 1 if sense == "ccw" else -1) for lab, sx, sy, sense in r["layout"]]
    else:
        rows = [(lab, x, y, 1 if sense == "ccw" else -1) for lab, x, y, sense in r["positions"]]
    if "thrust_n_per_rpm2" in r:
        kT = r["thrust_n_per_rpm2"] * RPM_PER_RADS ** 2
        kQ = r["torque_nm_per_rpm2"] * RPM_PER_RADS ** 2
        wmax = r["max_rpm"] / RPM_PER_RADS
    else:
        kT = r["thrust_n_per_rads2"]
        kQ = kT * r["moment_constant_m"]
        wmax = r["max_rads"]
    return rows, kT, kQ, wmax


def feet(spec):
    """Where it stands: [(x forward, y left)] (m), leg_height_m below the c.g. - the drawn feet
    ([ground] feet_m, in the rotors' order), or under the rotors, leg_spread of the way out."""
    g = spec["ground"]
    if "feet_m" in g:
        rows = spec["ground"]["feet_m"]
        if len(rows) != len(rotors(spec)[0]):
            raise ValueError("[ground] feet_m: one foot per rotor")
        return [(float(x), float(y)) for x, y in rows]
    spread = g.get("leg_spread", 0.8)  # a share of the way out to the rotors
    return [(xf * spread, yl * spread) for _, xf, yl, _ in rotors(spec)[0]]


def contact_set(spec, points):
    """(spring N/m, damping N s/m) for each of `points` - (x forward, y left, z up) m from the c.g. - standing
    on the ground together: the set's fastest mode (heave, roll or pitch: M^-1 K) at CONTACT_OMEGA_DT, damped
    at CONTACT_ZETA."""
    m = spec["mass"]
    M = np.diag([m["mass_kg"], m["ixx"], m["iyy"]])
    # a point's height changes by z + y roll - x pitch
    G = sum(np.outer(j, j) for j in (np.array([1.0, y, -x]) for x, y, _ in points))
    fastest = float(np.max(np.linalg.eigvals(np.linalg.solve(M, G)).real))  # omega^2 per N/m of each spring
    omega = CONTACT_OMEGA_DT / STEP_S
    k = omega * omega / fastest
    return k, 2.0 * CONTACT_ZETA * k / omega


def contacts(spec):
    """{"feet": [(x, y, z)], "tops": [...], ...}: where it stands and what it lands on upside down - the rotors'
    hubs and the top of the airframe ([ground] top_m above the c.g.), so a tumble on the ground comes to rest
    instead of sinking through it - each set with its spring and damper (contact_set) and the legs' static
    deflection under the aircraft's weight. The feet's contacts are that deflection below the drawn feet, so
    that at rest the feet the viewer draws stand on the ground."""
    g, r = spec["ground"], spec["rotors"]
    old = [key for key in ("spring_n_per_m", "damping_n_per_mps") if key in g]
    if old:
        raise ValueError("[ground] %s: hangar sizes a multirotor's contacts for the platform's step (docs/hangar.md, "
                         "Rotorcraft); leave them out" % ", ".join(old))
    rows = rotors(spec)[0]
    feet_pts = [(xf, yl, -g["leg_height_m"]) for xf, yl in feet(spec)]
    tops = [(xf, yl, r.get("height_m", 0.0)) for _, xf, yl, _ in rows] + [(0.0, 0.0, g.get("top_m", r.get("height_m", 0.0)))]
    k_feet, c_feet = contact_set(spec, feet_pts)
    k_tops, c_tops = contact_set(spec, tops)
    sag = spec["mass"]["mass_kg"] * 9.80665 / (len(feet_pts) * k_feet)
    stands = [(xf, yl, zu - sag) for xf, yl, zu in feet_pts]  # (sunk by the sag, on the drawn feet)
    return {"feet": stands, "feet_spring_n_per_m": k_feet, "feet_damping_n_per_mps": c_feet,
            "tops": tops, "tops_spring_n_per_m": k_tops, "tops_damping_n_per_mps": c_tops,
            "leg_sag_m": sag, "leg_height_m": g["leg_height_m"]}


def write(spec, out_dir, profile_xml=""):
    name = spec["aircraft"]["name"]
    os.makedirs(os.path.join(out_dir, "Engines"), exist_ok=True)
    rows, kT, kQ, wmax = rotors(spec)
    r, m = spec["rotors"], spec["mass"]
    t_max = kT * wmax ** 2
    weight = m["mass_kg"] * 9.80665
    with open(os.path.join(out_dir, "Engines", name + "_motor.xml"), "w", encoding="utf-8", newline="\n") as f:
        f.write("""<?xml version="1.0"?>
<!-- a motor and its rotor as JSBSim sees them: an electric engine whose power is the thrust (a
     direct thruster's thrust is its engine's power in ft lb/s), metered by the flight control
     system with fcs/throttle-pos-norm[i] = thrust / %s N -->
<electric_engine name="%s motor">
  <power unit="WATTS"> %s </power>
</electric_engine>
""" % (_f(t_max), name, _f(t_max / N_PER_LBF * 745.7 / 550.0)))
    with open(os.path.join(out_dir, "Engines", name + "_rotor.xml"), "w", encoding="utf-8", newline="\n") as f:
        f.write('<?xml version="1.0"?>\n<direct name="%s rotor"/>\n' % name)
    battery = spec.get("battery")
    if battery:
        # the hover's power (its capacity over its flight time, taken as a hover's), at each rotor's hover speed
        capacity_j = float(battery["capacity_wh"]) * 3600.0
        p_hover = capacity_j / (float(battery["hover_endurance_min"]) * 60.0)
        w_hover = math.sqrt(weight / len(rows) / kT)
    engines, motors, yaw, speeds = [], [], [], []
    for i, (label, xf, yl, sense) in enumerate(rows):
        x_in, y_in, z_in = -xf / 0.0254, -yl / 0.0254, r.get("height_m", 0.0) / 0.0254
        engines.append("""    <engine file="%(n)s_motor">
      <thruster file="%(n)s_rotor">
        <location unit="IN"> <x> %(x)s </x> <y> %(y)s </y> <z> %(z)s </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> 90 </pitch> <yaw> 0 </yaw> </orient>
      </thruster>
    </engine>""" % dict(n=name, x=_f(x_in), y=_f(y_in), z=_f(z_in)))
        # + roll right raises the left rotors, + pitch down the rear ones, + yaw left the clockwise
        # ones (their reaction turns the airframe counter-clockwise)
        mix = dict(r=MIX if yl > 0 else -MIX, p=MIX if xf < 0 else -MIX, y=MIX if sense < 0 else -MIX)
        motors.append("""      <!-- motor %(i)d: %(label)s, %(dir)s -->
      <fcs_function name="fcs/%(n)s/cmd[%(i)d]">
        <function> %(sup_open)s<max> <value> 0 </value> <min> <value> 1 </value> <sum>
          <property> fcs/throttle-cmd-norm[%(i)d] </property>
          <product> <value> %(r)s </value> <property> fcs/aileron-cmd-norm </property> </product>
          <product> <value> %(p)s </value> <property> fcs/elevator-cmd-norm </property> </product>
          <product> <value> %(y)s </value> <property> fcs/rudder-cmd-norm </property> </product>
        </sum> </min> </max>%(sup_close)s </function>
      </fcs_function>
      <fcs_function name="fcs/%(n)s/omega-cmd[%(i)d]">
        <function> <product> <value> %(wmax)s </value> <sqrt> <property> fcs/%(n)s/cmd[%(i)d] </property> </sqrt> </product> </function>
      </fcs_function>
      <fcs_function name="fcs/%(n)s/motor-c1[%(i)d]">
        <function> <ifthen> <gt> <property> fcs/%(n)s/omega-cmd[%(i)d] </property> <property> fcs/%(n)s/omega[%(i)d] </property> </gt>
          <value> %(cu)s </value> <value> %(cd)s </value> </ifthen> </function>
      </fcs_function>
      <lag_filter name="fcs/%(n)s/omega[%(i)d]"> <input> fcs/%(n)s/omega-cmd[%(i)d] </input> <c1> fcs/%(n)s/motor-c1[%(i)d] </c1> </lag_filter>
      <fcs_function name="fcs/%(n)s/thrust-norm[%(i)d]">
        <function> <pow> <quotient> <property> fcs/%(n)s/omega[%(i)d] </property> <value> %(wmax)s </value> </quotient> <value> 2 </value> </pow> </function>
        <output> fcs/throttle-pos-norm[%(i)d] </output>
      </fcs_function>
      <fcs_function name="fcs/%(n)s/rpm[%(i)d]">
        <function> <product> <property> fcs/%(n)s/omega[%(i)d] </property> <value> %(rpmk)s </value> </product> </function>
        <output> propulsion/engine[%(i)d]/rotor-rpm </output>
      </fcs_function>
""" % dict(mix, i=i, n=name, label=label, dir="counter-clockwise" if sense > 0 else "clockwise", wmax=_f(wmax),
           cu=_f(1 / r["lag_up_s"]), cd=_f(1 / r["lag_down_s"]), rpmk=_f(RPM_PER_RADS),
           sup_open="<product> <property> fsim/battery/supply </property> " if battery else "",
           sup_close=" </product>" if battery else ""))
        # a counter-clockwise rotor's reaction turns the airframe clockwise: nose right, + N
        yaw.append("<product> <value> %s </value> <pow> <property> fcs/%s/omega[%d] </property> <value> 2 </value> </pow> </product>"
                   % (_f(sense * kQ / (N_PER_LBF * M_PER_FT)), name, i))
        speeds.append("<property> fcs/%s/omega[%d] </property>" % (name, i))
    ground_effect, z_axis = "", []  # (the body Z axis's forces: its rotor drag, the ground effect)
    if "ground_effect" in r:
        # gym-pybullet-drones: each rotor's thrust grows by k_ge (r / 4 h)^2, h its height - held at
        # least GND_EFF_H_CLIP = r/4 sqrt(15 k_ge / 4), so the gain is never over 4/15 - as a force
        # along the body's -z
        h_min = 0.25 * r["radius_m"] * math.sqrt(15.0 * r["ground_effect"] / 4.0)
        k = r["ground_effect"] * (r["radius_m"] / 4.0) ** 2 * kT / N_PER_LBF
        ground_effect = """    <function name="aero/%(n)s/ground-effect-scale">
      <description> k_ge (r / 4)^2 / h^2, h the rotors' height above the ground (m) </description>
      <quotient> <value> %(k)s </value> <pow> <max> <value> %(hmin)s </value>
        <sum> <product> <property> position/h-agl-ft </property> <value> 0.3048 </value> </product> <value> %(hr)s </value> </sum> </max> <value> 2 </value> </pow> </quotient>
    </function>
""" % dict(n=name, k=_f(k), hmin=_f(h_min), hr=_f(r.get("height_m", 0.0)))
        z_axis.append("""<function name="aero/force/%(n)s-ground-effect">
      <product> <value> -1 </value> <property> aero/%(n)s/ground-effect-scale </property>
        <sum> %(w2)s </sum> </product> </function>""" % dict(
            n=name, w2=" ".join("<pow> <property> fcs/%s/omega[%d] </property> <value> 2 </value> </pow>" % (name, i) for i in range(len(rows)))))
    if "drag_xy" in r:
        # gym-pybullet-drones: F = -k (sum of the rotors' speeds, rad/s) v, per body axis
        kx, kz = r["drag_xy"], r["drag_z"]
    else:
        # PX4's motor model: each rotor's drag -w c v in the rotor's plane (none along its axis)
        kx, kz = r["rotor_drag"], 0.0
    drag = """    <function name="aero/%(n)s/rotor-speed-sum"> <sum> %(ws)s </sum> </function>
    <axis name="X"> <function name="aero/force/%(n)s-rotor-drag-x"> <product> <value> %(kx)s </value> <property> aero/%(n)s/rotor-speed-sum </property> <property> velocities/u-aero-fps </property> </product> </function> </axis>
    <axis name="Y"> <function name="aero/force/%(n)s-rotor-drag-y"> <product> <value> %(kx)s </value> <property> aero/%(n)s/rotor-speed-sum </property> <property> velocities/v-aero-fps </property> </product> </function> </axis>
""" % dict(n=name, ws=" ".join(speeds), kx=_f(-kx * M_PER_FT / N_PER_LBF))
    if kz:
        z_axis.insert(0, """<function name="aero/force/%(n)s-rotor-drag-z"> <product> <value> %(kz)s </value> <property> aero/%(n)s/rotor-speed-sum </property> <property> velocities/w-aero-fps </property> </product> </function>"""
                      % dict(n=name, kz=_f(-kz * M_PER_FT / N_PER_LBF)))
    # one <axis> per name: JSBSim keeps the functions of an axis's last element only (FGAerodynamics::Load
    # assigns each element's, it does not add them) - a second Z axis once dropped the rotors' drag along it
    if z_axis:
        ground_effect += "    <axis name=\"Z\"> %s </axis>\n" % "\n      ".join(z_axis)
    # (PX4's rolling moment, 1e-6 N m per rad/s per m/s, is left out: a few per cent of the rotor drag's)
    kgm2 = KG_PER_SLUG * M_PER_FT ** 2
    ground = contacts(spec)
    legs = []
    for i, (xf, yl, zu) in enumerate(ground["feet"]):
        legs.append("""    <contact type="BOGEY" name="leg %d">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> 0.8 </static_friction> <dynamic_friction> 0.6 </dynamic_friction> <rolling_friction> 0.6 </rolling_friction>
      <spring_coeff unit="N/M"> %s </spring_coeff> <damping_coeff unit="N/M/SEC"> %s </damping_coeff>
      <max_steer unit="DEG"> 0 </max_steer> <brake_group> NONE </brake_group> <retractable> 0 </retractable>
    </contact>""" % (i, _f(-xf / 0.0254), _f(-yl / 0.0254), _f(zu / 0.0254),
                     _f(ground["feet_spring_n_per_m"]), _f(ground["feet_damping_n_per_mps"])))
    for i, (xf, yl, zu) in enumerate(ground["tops"]):
        legs.append("""    <contact type="STRUCTURE" name="top %d">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> 0.8 </static_friction> <dynamic_friction> 0.6 </dynamic_friction>
      <spring_coeff unit="N/M"> %s </spring_coeff> <damping_coeff unit="N/M/SEC"> %s </damping_coeff>
    </contact>""" % (i, _f(-xf / 0.0254), _f(-yl / 0.0254), _f(zu / 0.0254),
                     _f(ground["tops_spring_n_per_m"]), _f(ground["tops_damping_n_per_mps"])))
    span_ft = 2 * max(math.hypot(x, y) for _, x, y, _ in rows) / M_PER_FT
    xml = """<?xml version="1.0"?>
<fdm_config name="%(n)s" version="2.0" release="BETA">
  <fileheader>
    <author> hangar (tools/hangar), from aircraft/%(n)s/%(n)s.toml </author>
    <description> %(desc)s </description>
    <note> Written by hangar's rotorcraft pipeline from the design file, whose comments cite every number's source. </note>
%(refs)s  </fileheader>
  <metrics>
    <wingarea unit="FT2"> %(area)s </wingarea>
    <wingspan unit="FT"> %(span)s </wingspan>
    <chord unit="FT"> %(span)s </chord>
    <htailarea unit="FT2"> 0 </htailarea> <htailarm unit="FT"> 0 </htailarm>
    <vtailarea unit="FT2"> 0 </vtailarea> <vtailarm unit="FT"> 0 </vtailarm>
    <location name="AERORP" unit="IN"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
    <location name="EYEPOINT" unit="IN"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
    <location name="VRP" unit="IN"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2"> %(ixx)s </ixx>
    <iyy unit="SLUG*FT2"> %(iyy)s </iyy>
    <izz unit="SLUG*FT2"> %(izz)s </izz>
    <emptywt unit="LBS"> %(w)s </emptywt>
    <location name="CG" unit="IN"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
  </mass_balance>
  <ground_reactions>
%(legs)s
  </ground_reactions>
  <propulsion>
%(engines)s
  </propulsion>
  <flight_control name="%(n)s mixer and motors">
%(profile)s%(battery)s    <channel name="motors">
%(motors)s    </channel>
  </flight_control>
  <aerodynamics>
    <axis name="YAW">
      <function name="aero/moment/%(n)s-rotor-torque">
        <description> the rotors' reaction torques, k_Q w^2 each </description>
        <sum> %(yaw)s </sum>
      </function>
    </axis>
%(drag)s%(ge)s  </aerodynamics>
</fdm_config>
""" % dict(n=name, desc=spec["aircraft"]["description"], refs=references_xml(spec, "    "), area=_f(math.pi * (span_ft / 2) ** 2), span=_f(span_ft),
           ixx=_f(m["ixx"] / kgm2), iyy=_f(m["iyy"] / kgm2), izz=_f(m["izz"] / kgm2), w=_f(m["mass_kg"] * LB_PER_KG),
           legs="\n".join(legs), engines="\n".join(engines), profile=profile_xml, motors="".join(motors),
           battery=("""    <property value="%(cap)s">fsim/battery/capacity-j</property>
    <channel name="battery">
      <!-- the power the battery gives: the hover's (%(ph)s W) as the rotors' speed cubed over their hover
           speed's (%(wh)s rad/s); the energy used, the charge left, the motors' supply while there is some -->
      <fcs_function name="fsim/battery/power-w">
        <function> <product> <value> %(k)s </value> <sum> %(w3)s </sum> </product> </function>
      </fcs_function>
      <integrator name="fsim/battery/used-j"> <input> fsim/battery/power-w </input> <c1> 1 </c1> </integrator>
      <fcs_function name="fsim/battery/charge-j">
        <function> <max> <value> 0 </value> <difference> <property> fsim/battery/capacity-j </property>
          <property> fsim/battery/used-j </property> </difference> </max> </function>
      </fcs_function>
      <fcs_function name="fsim/battery/supply">
        <function> <gt> <property> fsim/battery/charge-j </property> <value> 0 </value> </gt> </function>
      </fcs_function>
    </channel>
""" % dict(cap=_f(capacity_j), ph=_f(p_hover), wh=_f(w_hover), k=_f(p_hover / (len(rows) * w_hover ** 3)),
           w3=" ".join("<pow> <property> fcs/%s/omega[%d] </property> <value> 3 </value> </pow>" % (name, i) for i in range(len(rows))))
                    ) if battery else "",
           yaw=" ".join(yaw), drag=drag, ge=ground_effect)
    with open(os.path.join(out_dir, name + ".xml"), "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)
    return dict(kT=kT, kQ=kQ, omega_max=wmax, thrust_max_n=t_max, hover=weight / 4 / t_max,
                hover_rpm=math.sqrt(weight / 4 / kT) * RPM_PER_RADS, thrust_to_weight=4 * t_max / weight,
                **{key: ground[key] for key in ("feet_spring_n_per_m", "feet_damping_n_per_mps", "tops_spring_n_per_m",
                                                "tops_damping_n_per_mps", "leg_sag_m")})
