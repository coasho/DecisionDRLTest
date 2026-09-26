"""A multirotor as a JSBSim aircraft, from a design file (kind = "multirotor"): <name>.xml and
its Engines/, flying the published motor and rotor equations.

Each motor's speed is a state of the flight control system, w' = (w_cmd - w)/tau, with the
design's lags up and down; its thrust k_T w^2 is a JSBSim direct thruster's (an electric engine,
whose thrust is its power in ft lb/s, metered by fcs/throttle-pos-norm[i] = thrust / maximum);
the reaction torques k_Q w^2, the rotor drag and the ground effect are aerodynamic functions.
The command: each motor's thrust (0..1 of its maximum; throttle[i]), roll, pitch and yaw mixed in
(aileron + right, elevator + nose down, rudder + nose left), so a policy flies the motors directly
or through the mixer. The speed each motor commands is sqrt(thrust): the thrust is linear in the
command, as an ESC with thrust linearisation gives it. The rotors' speeds are reported as
propulsion/engine[i]/rotor-rpm, which the platform shows as the engines' rpm.
"""
import math
import os

N_PER_LBF, M_PER_FT, KG_PER_SLUG, LB_PER_KG = 4.448222, 0.3048, 14.593903, 2.204623
RPM_PER_RADS = 60 / (2 * math.pi)
#: a full roll, pitch or yaw command moves each motor's thrust by this much (of its maximum)
MIX = 0.25


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
        <function> <max> <value> 0 </value> <min> <value> 1 </value> <sum>
          <property> fcs/throttle-cmd-norm[%(i)d] </property>
          <product> <value> %(r)s </value> <property> fcs/aileron-cmd-norm </property> </product>
          <product> <value> %(p)s </value> <property> fcs/elevator-cmd-norm </property> </product>
          <product> <value> %(y)s </value> <property> fcs/rudder-cmd-norm </property> </product>
        </sum> </min> </max> </function>
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
           cu=_f(1 / r["lag_up_s"]), cd=_f(1 / r["lag_down_s"]), rpmk=_f(RPM_PER_RADS)))
        # a counter-clockwise rotor's reaction turns the airframe clockwise: nose right, + N
        yaw.append("<product> <value> %s </value> <pow> <property> fcs/%s/omega[%d] </property> <value> 2 </value> </pow> </product>"
                   % (_f(sense * kQ / (N_PER_LBF * M_PER_FT)), name, i))
        speeds.append("<property> fcs/%s/omega[%d] </property>" % (name, i))
    ground_effect = ""
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
    <axis name="Z"> <function name="aero/force/%(n)s-ground-effect">
      <product> <value> -1 </value> <property> aero/%(n)s/ground-effect-scale </property>
        <sum> %(w2)s </sum> </product> </function> </axis>
""" % dict(n=name, k=_f(k), hmin=_f(h_min), hr=_f(r.get("height_m", 0.0)),
           w2=" ".join("<pow> <property> fcs/%s/omega[%d] </property> <value> 2 </value> </pow>" % (name, i) for i in range(len(rows))))
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
        drag += """    <axis name="Z"> <function name="aero/force/%(n)s-rotor-drag-z"> <product> <value> %(kz)s </value> <property> aero/%(n)s/rotor-speed-sum </property> <property> velocities/w-aero-fps </property> </product> </function> </axis>
""" % dict(n=name, kz=_f(-kz * M_PER_FT / N_PER_LBF))
    # (PX4's rolling moment, 1e-6 N m per rad/s per m/s, is left out: a few per cent of the rotor drag's)
    kgm2 = KG_PER_SLUG * M_PER_FT ** 2
    g = spec["ground"]
    legs = []
    for i, (xf, yl) in enumerate(feet(spec)):
        legs.append("""    <contact type="BOGEY" name="leg %d">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> 0.8 </static_friction> <dynamic_friction> 0.6 </dynamic_friction> <rolling_friction> 0.6 </rolling_friction>
      <spring_coeff unit="N/M"> %s </spring_coeff> <damping_coeff unit="N/M/SEC"> %s </damping_coeff>
      <max_steer unit="DEG"> 0 </max_steer> <brake_group> NONE </brake_group> <retractable> 0 </retractable>
    </contact>""" % (i, _f(-xf / 0.0254), _f(-yl / 0.0254), _f(-g["leg_height_m"] / 0.0254),
                     _f(g["spring_n_per_m"]), _f(g["damping_n_per_mps"])))
    # what it lands on upside down: the rotors' hubs and the top of the airframe ([ground] top_m
    # above the c.g.) - so a tumble on the ground comes to rest instead of sinking through it
    tops = [(xf, yl, r.get("height_m", 0.0)) for _, xf, yl, _ in rows] + [(0.0, 0.0, g.get("top_m", r.get("height_m", 0.0)))]
    for i, (xf, yl, zu) in enumerate(tops):
        legs.append("""    <contact type="STRUCTURE" name="top %d">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> 0.8 </static_friction> <dynamic_friction> 0.6 </dynamic_friction>
      <spring_coeff unit="N/M"> %s </spring_coeff> <damping_coeff unit="N/M/SEC"> %s </damping_coeff>
    </contact>""" % (i, _f(-xf / 0.0254), _f(-yl / 0.0254), _f(zu / 0.0254), _f(g["spring_n_per_m"]), _f(g["damping_n_per_mps"])))
    span_ft = 2 * max(math.hypot(x, y) for _, x, y, _ in rows) / M_PER_FT
    xml = """<?xml version="1.0"?>
<fdm_config name="%(n)s" version="2.0" release="BETA">
  <fileheader>
    <author> hangar (tools/hangar), from aircraft/%(n)s/%(n)s.toml </author>
    <description> %(desc)s </description>
    <note> Written by hangar's rotorcraft pipeline from the design file, whose comments cite every number's source. </note>
  </fileheader>
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
%(profile)s    <channel name="motors">
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
""" % dict(n=name, desc=spec["aircraft"]["description"], area=_f(math.pi * (span_ft / 2) ** 2), span=_f(span_ft),
           ixx=_f(m["ixx"] / kgm2), iyy=_f(m["iyy"] / kgm2), izz=_f(m["izz"] / kgm2), w=_f(m["mass_kg"] * LB_PER_KG),
           legs="\n".join(legs), engines="\n".join(engines), profile=profile_xml, motors="".join(motors),
           yaw=" ".join(yaw), drag=drag, ge=ground_effect)
    with open(os.path.join(out_dir, name + ".xml"), "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)
    return dict(kT=kT, kQ=kQ, omega_max=wmax, thrust_max_n=t_max, hover=weight / 4 / t_max,
                hover_rpm=math.sqrt(weight / 4 / kT) * RPM_PER_RADS, thrust_to_weight=4 * t_max / weight)
