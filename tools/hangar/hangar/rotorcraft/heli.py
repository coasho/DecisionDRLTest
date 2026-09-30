"""A helicopter as a JSBSim aircraft, from a design file (aircraft/<name>/<name>.toml, kind =
"helicopter"): <name>.xml and its Engines/.

The rotors are JSBSim's FGRotor (with flightsim's patch: cmake/JsbsimPatches.cmake), their force
model the design's - "classical", NASA TM-73254's blade-element forms, or "heffley", JSBSim's own.
The rotor speed is a state of the flight control system: the engine's power less the rotors'
drives it through the drive's inertia, a governor holding the design's rpm with the load fed
forward (the main rotor's rpm is given to it; the tail rotor follows through its gearing). The
flight control system carries the aircraft's mechanical controls - linkages, rigging, the Bell
stabilizer bar, a mixing unit and rate feedbacks, a scheduled stabilator, a pitch bias actuator -
and the airframe's forces are one of two published models: "tm73254" (the UH-1H report's
fuselage, stabilizer and fin, in its shaft axes) or "tm85890" (the UH-60 report's wind-tunnel
fuselage and TM-84281's stabilizer and fin through 360 deg). The airframe takes the engine's
torque, not the rotors' (an external moment: JSBSim applies theirs as though the airframe held
their speed).

On the ground it stands on its skids or wheels ([ground]) and, struck hard or turned over, on its
airframe: structure contacts on its drawn hull and its hubs, as a fixed wing's (structure_contacts).

Senses, the platform's: fcs/elevator-cmd-norm + nose down (stick forward), aileron + right,
rudder + nose left (left pedal: JSBSim's, trailing edge left), throttle-cmd-norm the collective
(every entry). The design file's stick is + forward and + right, its pedal + right.

The fuel ([fuel] capacity_lb, [engine] sfc_lb_per_shp_h): a tank at the c.g., full as the aircraft
spawns and part of its weight (so it spawns as its report flies it, the c.g. and inertia unmoved),
feeding the engines, which burn their specific fuel consumption times the power they give
(JSBSim's electric engine, patched: cmake/JsbsimPatches.cmake) and give none once it is empty.
"""
import math
import os

from ..applicability import references_xml

RHO0 = 0.002377  # slug/ft3
HP = 550.0       # ft lb/s
W_PER_HP = 745.7
IN = 0.0254      # m
LB = 0.45359237  # kg
SLUG_FT2 = 1.3558179  # kg m2


def _f(x):
    return "%.8g" % x


def _write(path, text):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _sum(terms):
    """JSBSim function terms added (a single term as it is: a one-term <sum> is a no-op it warns of)."""
    return terms[0] if len(terms) == 1 else "<sum> %s </sum>" % " ".join(terms)


def fuel_of(spec):
    """The design's fuel: {capacity_lb, sfc} - its tank's capacity (full as it spawns) and the engines'
    specific fuel consumption (lb per shaft horsepower-hour) - or None without both."""
    fuel, eng = spec.get("fuel", {}), spec.get("engine", {})
    if "capacity_lb" not in fuel or "sfc_lb_per_shp_h" not in eng:
        return None
    return {"capacity_lb": float(fuel["capacity_lb"]), "sfc": float(eng["sfc_lb_per_shp_h"])}


# Structure contacts: what meets the ground, in any attitude, other than the gear - the airframe's
# convex hull (the fuselage, the tail boom, the pylon and the stabilizer, a tail skid) and the rotors'
# hubs - sized as a fixed wing's are for the 120 Hz step (jsbsim.STRUCTURE_OMEGA on the apparent mass
# at each point). A part that reaches within GEAR_CLEARANCE_M of the ground the aircraft stands on is
# the gear (a skid, its cross tubes), which gives way with it: the hull is made of the rest - so the
# belly the skids stand above is part of it - and the gear's contacts stand for the gear.
ROTOR_MIN_RPM = 50.0    # FGRotor's rpm limits: the least, and the most as a share of the design's
ROTOR_MAX_SHARE = 1.3
GEAR_CLEARANCE_M = 0.15


def skid_shares(contacts_in, cg_in):
    """Each skid contact's share of the weight standing, by the lever rule along the skids: the contacts stand at
    two stations, fore and aft, each carrying the weight in proportion to the other's distance from the c.g., its
    contacts alike (the skids side by side, the c.g. between them)."""
    xs = sorted({float(c[0]) for c in contacts_in})
    if len(xs) != 2:
        raise ValueError("[ground] contacts_in: a skid's contacts stand at two stations, fore and aft")
    fore, aft = xs
    x = float(cg_in[0])
    if not fore < x < aft:
        raise ValueError("[ground] contacts_in: the c.g. (station %g) must lie between the skid's stations" % x)
    per = {fore: (aft - x) / (aft - fore), aft: (x - fore) / (aft - fore)}
    n = {st: sum(1 for c in contacts_in if float(c[0]) == st) for st in xs}
    return [per[float(c[0])] / n[float(c[0])] for c in contacts_in]


def hubs(spec):
    """The rotors' hubs as the model draws them (rotorcraft/model.py, design frame, m): the main
    rotor's at the top of what turns on its mast (its hub, or a vibration absorber above it), the
    tail rotor's at its hub - on their shafts, `offset_m` along them. [(name, point)]."""
    look = spec.get("model", {})
    out = []
    for key, name in (("main", "main rotor hub"), ("tail", "tail rotor hub")):
        r = spec["rotor"][key]
        lk = look.get("%s_rotor" % key, {})
        if key == "main":
            tilt = math.radians(r.get("mast_tilt_deg", 0.0))
            t = (-math.sin(tilt), 0.0, math.cos(tilt))
            top = 0.7 * lk.get("hub_radius", 0.08 * r["radius_ft"] * 0.3048)
            if "absorber" in lk:
                b = lk["absorber"]
                top = max(top, b.get("above_m", 0.35) + 0.7 * b["weight_m"])
        else:
            side = 1.0 if r.get("thrust", "right") == "right" else -1.0
            cant = math.radians(r.get("cant_deg", 0.0))
            t = (0.0, side * math.cos(cant), math.sin(cant))
            top = 0.0
        along = lk.get("offset_m", 0.0) + top
        out.append((name, [c * IN + along * d for c, d in zip(r["hub_in"], t)]))
    return out


def structure_contacts(spec):
    """[(name, point m, spring N/m, damping N s/m)]: the helicopter's structure contacts (above), sized
    on the empty aircraft (the lightest, so the fastest; its fuel is at the c.g.)."""
    import numpy as np
    from .. import jsbsim
    from ..geometry.aircraft import Aircraft
    shape = Aircraft.shape(spec)
    # the ground the aircraft stands on: the plane through its gear's contacts
    gear = np.array(spec["ground"]["contacts_in"], float) * IN
    A = np.column_stack([np.ones(len(gear)), gear[:, 0], gear[:, 1]])
    plane = np.linalg.lstsq(A, gear[:, 2], rcond=None)[0]

    def clear(P):  # the least height of points above that ground
        P = np.atleast_2d(np.asarray(P, float))
        return float(np.min(P[:, 2] - (plane[0] + plane[1] * P[:, 0] + plane[2] * P[:, 1])))

    shape.bodies = [b for b in shape.bodies if clear(b.skin(32, 20)[0]) >= GEAR_CLEARANCE_M]
    shape.surfaces = [f for f in shape.surfaces if clear(f.skin(20, 14)[0]) >= GEAR_CLEARANCE_M]
    shape.struts = [s for s in shape.struts if clear([s.a, s.b]) >= GEAR_CLEARANCE_M]
    more = hubs(spec)
    for s in shape.struts:  # drawn bars - a tail skid: their ends
        for q in (s.a, s.b):
            more.append((s.name, q))
            if s.mirror and abs(q[1]) > 1e-6:
                more.append((s.name, q * np.array([1.0, -1.0, 1.0])))
    points = jsbsim.structure_points(shape, more=more)
    m = spec["mass"]
    fuel = fuel_of(spec)
    mass = (m["weight_lb"] - (fuel["capacity_lb"] if fuel else 0.0)) * LB
    J = np.array([[m["ixx"], 0.0, -m.get("ixz", 0.0)], [0.0, m["iyy"], 0.0], [-m.get("ixz", 0.0), 0.0, m["izz"]]]) * SLUG_FT2
    cg = np.array(m["cg_in"], float) * IN
    lo, hi = shape.extent()
    near = jsbsim.STRUCTURE_SHARE * float(np.max(hi - lo))
    everything = np.array([p for _, p in points] + list(gear))
    w = jsbsim.STRUCTURE_OMEGA
    out = []
    for name, p in points:
        m_eff = jsbsim.apparent_mass(p - cg, mass, J)
        share = int(np.sum(np.linalg.norm(everything - p, axis=1) < near))  # itself included
        out.append((name, p, m_eff * w ** 2 / share, 2.0 * jsbsim.STRUCTURE_ZETA * m_eff * w / share))
    return out


def rotor_data(spec):
    """The rotors' derived numbers: speeds, inertias, the drive's inertia."""
    mr, tr = spec["rotor"]["main"], spec["rotor"]["tail"]
    omega = mr["rpm"] * 2 * math.pi / 60
    R = mr["radius_ft"]
    if "lock" in mr:
        lock = mr["lock"]
    else:
        lock = 16.0 / (mr["flap_lag_s"] * omega)  # the flapping lag the design gives: 16/(gamma Omega)
    ib = RHO0 * mr["lift_slope"] * mr["chord_ft"] * R ** 4 / lock
    sb = 1.5 * ib / R  # a uniform blade's first mass moment about the hinge
    tr_R = tr["radius_ft"]
    tr_omega = tr["tip_speed_fps"] / tr_R if "tip_speed_fps" in tr else tr["rpm"] * 2 * math.pi / 60
    tr_chord = tr.get("chord_ft", tr.get("solidity", 0.0) * math.pi * tr_R / tr["blades"])
    tr_lock = tr.get("lock", 4.0)
    tr_ib = RHO0 * tr["lift_slope"] * tr_chord * tr_R ** 4 / tr_lock
    j = 1.10 * mr["blades"] * ib + tr["blades"] * tr_ib * (tr_omega / omega) ** 2  # + 10 % for the hub, bar and drive
    return dict(omega=omega, rpm=mr["rpm"], lock=lock, ib=ib, sb=sb, flap_lag=16.0 / (lock * omega),
                tr_omega=tr_omega, tr_rpm=tr_omega * 60 / (2 * math.pi), tr_chord=tr_chord, tr_ib=tr_ib,
                tr_flap_lag=16.0 / (tr_lock * tr_omega), j=j)


def write(spec, out_dir, profile_xml=""):
    """Write <name>.xml and Engines/ into out_dir; returns the derived numbers."""
    name = spec["aircraft"]["name"]
    os.makedirs(os.path.join(out_dir, "Engines"), exist_ok=True)
    rd = rotor_data(spec)
    mr, tr, eng = spec["rotor"]["main"], spec["rotor"]["tail"], spec["engine"]
    p_max = eng["power_shp"] * HP

    fuel = fuel_of(spec)
    _write(os.path.join(out_dir, "Engines", name + "_engine.xml"), """<?xml version="1.0"?>
<!-- the engines as one governed power source: the flight control system meters it
     (fcs/throttle-pos-norm[0] = shaft power / the design's limit, %.0f shp)%s -->
<electric_engine name="%s engine">
  <power unit="WATTS"> %.0f </power>
%s</electric_engine>
""" % (eng["power_shp"], "; it burns its specific fuel\n     consumption times the power it gives (JSBSim patched: cmake/JsbsimPatches.cmake)" if fuel else "",
       name, eng["power_shp"] * W_PER_HP, ('  <bsfc unit="LBS/HP*HR"> %s </bsfc>\n' % _f(fuel["sfc"])) if fuel else ""))
    _write(os.path.join(out_dir, "Engines", name + "_tail_drive.xml"), """<?xml version="1.0"?>
<!-- carries the tail rotor, which turns with the main rotor; the flight control system takes its power from the engine -->
<electric_engine name="%s tail rotor drive">
  <power unit="WATTS"> 0 </power>
</electric_engine>
""" % name)
    ge = ""
    if "ground_effect_g1" in mr:
        # TM-73254 eq. 10a: K_G = 1 - exp(-(Z/D)/G1), Z the hub's height: JSBSim's factor exp(-(h + shift) k)
        hub_above_cg = (mr["hub_in"][2] - spec["mass"]["cg_in"][2]) / 12.0
        ge = "  <groundeffectexp> %s </groundeffectexp>\n  <groundeffectshift unit=\"FT\"> %s </groundeffectshift>\n" % (
            _f(1.0 / (2 * mr["radius_ft"] * mr["ground_effect_g1"])), _f(hub_above_cg))
    hinge = mr.get("hinge_offset_ft", 0.0)
    _write(os.path.join(out_dir, "Engines", name + "_rotor.xml"), """<?xml version="1.0"?>
<rotor name="%(n)s main rotor">
  <diameter unit="FT"> %(d)s </diameter>
  <numblades> %(b)d </numblades>
  <gearratio> 1.0 </gearratio>
  <nominalrpm> %(rpm)s </nominalrpm>
  <minrpm> %(minrpm)s </minrpm>
  <maxrpm> %(maxrpm)s </maxrpm>
  <chord unit="FT"> %(c)s </chord>
  <liftcurveslope Xunit="1/RAD"> %(a)s </liftcurveslope>
  <twist unit="DEG"> %(tw)s </twist>
  <hingeoffset unit="FT"> %(e)s </hingeoffset>
  <flappingmoment unit="SLUG*FT2"> %(ib)s </flappingmoment>
  <massmoment> %(sb)s </massmoment>
  <polarmoment unit="SLUG*FT2"> %(jp)s </polarmoment>
  <inflowlag> %(lag)s </inflowlag>
  <tiplossfactor> %(tl)s </tiplossfactor>
  <ExternalRPM> -1 </ExternalRPM>
  <model> %(model)s </model>
%(ge)s</rotor>
""" % dict(n=name, d=_f(2 * mr["radius_ft"]), b=mr["blades"], rpm=_f(rd["rpm"]), maxrpm=_f(ROTOR_MAX_SHARE * rd["rpm"]), minrpm=_f(ROTOR_MIN_RPM),
           c=_f(mr["chord_ft"]), a=_f(mr["lift_slope"]), tw=_f(mr["twist_deg"]), e=_f(hinge), ib=_f(rd["ib"]),
           sb=_f(rd["sb"]), jp=_f(mr["blades"] * rd["ib"]), lag=_f(mr.get("inflow_lag_s", 0.1)),
           tl=_f(mr.get("tip_loss", 1.0)), model=mr.get("force_model", "heffley"), ge=ge))
    _write(os.path.join(out_dir, "Engines", name + "_tail_rotor.xml"), """<?xml version="1.0"?>
<rotor name="%(n)s tail rotor">
  <diameter unit="FT"> %(d)s </diameter>
  <numblades> %(b)d </numblades>
  <gearratio> %(g)s </gearratio>
  <nominalrpm> %(rpm)s </nominalrpm>
  <minrpm> %(minrpm)s </minrpm>
  <maxrpm> %(maxrpm)s </maxrpm>
  <chord unit="FT"> %(c)s </chord>
  <liftcurveslope Xunit="1/RAD"> %(a)s </liftcurveslope>
  <twist unit="DEG"> %(tw)s </twist>
  <hingeoffset unit="FT"> 0.0 </hingeoffset>
  <flappingmoment unit="SLUG*FT2"> %(ib)s </flappingmoment>
  <polarmoment unit="SLUG*FT2"> %(jp)s </polarmoment>
  <inflowlag> %(lag)s </inflowlag>
  <tiplossfactor> %(tl)s </tiplossfactor>
  <controlmap> TAIL </controlmap>
  <ExternalRPM> 0 </ExternalRPM>
  <model> %(model)s </model>
</rotor>
""" % dict(n=name, d=_f(2 * tr["radius_ft"]), b=tr["blades"], g=_f(rd["rpm"] / rd["tr_rpm"]), rpm=_f(rd["tr_rpm"]),
           maxrpm=_f(ROTOR_MAX_SHARE * rd["tr_rpm"]), minrpm=_f(ROTOR_MIN_RPM), c=_f(rd["tr_chord"]), a=_f(tr["lift_slope"]), tw=_f(tr.get("twist_deg", 0.0)),
           ib=_f(rd["tr_ib"]), jp=_f(tr["blades"] * rd["tr_ib"]), lag=_f(tr.get("inflow_lag_s", 0.1)),
           tl=_f(tr.get("tip_loss", 1.0)), model=tr.get("force_model", "heffley")))

    m, g = spec["mass"], spec["ground"]
    cg = m["cg_in"]
    contacts = []
    springs = g["spring_lbf_per_ft"] if isinstance(g["spring_lbf_per_ft"], list) else [g["spring_lbf_per_ft"]] * len(g["contacts_in"])
    dampers = g["damping_lbf_per_fps"] if isinstance(g["damping_lbf_per_fps"], list) else [g["damping_lbf_per_fps"]] * len(g["contacts_in"])
    skid = g.get("kind") == "skid"
    if skid and not isinstance(g["spring_lbf_per_ft"], list):
        # a skid's contacts sized as a fixed wing's wheels are: each its share of the weight (skid_shares), the
        # design's spring and damper their mean - standing, each sinks alike and the aircraft sits as drawn, and
        # a level strike stays level (equal ones, the c.g. 15 in ahead of the rear pair, pitched a UH-1H up
        # onto its tail: docs/rotorcraft.md, 7)
        shares = skid_shares(g["contacts_in"], cg)
        springs = [len(shares) * share * g["spring_lbf_per_ft"] for share in shares]
        dampers = [len(shares) * share * g["damping_lbf_per_fps"] for share in shares]
    for i, (pt, k, c) in enumerate(zip(g["contacts_in"], springs, dampers)):
        side = "left" if pt[1] < 0 else ("right" if pt[1] > 0 else "centre")
        contacts.append("""    <contact type="BOGEY" name="%s %d">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> %s </static_friction>
      <dynamic_friction> 0.5 </dynamic_friction>
      <rolling_friction> %s </rolling_friction>
      <spring_coeff unit="LBS/FT"> %s </spring_coeff>
      <damping_coeff unit="LBS/FT/SEC"> %s </damping_coeff>
      <max_steer unit="DEG"> %s </max_steer>
      <brake_group> %s </brake_group>
      <retractable> 0 </retractable>
    </contact>""" % ("skid" if skid else "wheel", i, _f(pt[0]), _f(pt[1]), _f(pt[2]), "0.6" if skid else "0.8",
                     "0.5" if skid else "0.02", _f(k), _f(c), "0" if skid or side != "centre" else "360",
                     "NONE" if skid or side == "centre" else side.upper()))
    for name_s, p, k, c in structure_contacts(spec):
        contacts.append("""    <contact type="STRUCTURE" name="%s">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <static_friction> 0.8 </static_friction>
      <dynamic_friction> 0.6 </dynamic_friction>
      <spring_coeff unit="N/M"> %.0f </spring_coeff>
      <damping_coeff unit="N/M/SEC"> %.0f </damping_coeff>
      <damping_coeff_rebound unit="N/M/SEC"> %.0f </damping_coeff_rebound>
    </contact>""" % (name_s, _f(p[0] / IN), _f(p[1] / IN), _f(p[2] / IN), k, c, c))
    tr_yaw = 90 if tr.get("thrust", "right") == "right" else -90
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
    <chord unit="FT"> %(chord)s </chord>
    <htailarea unit="FT2"> 0 </htailarea> <htailarm unit="FT"> 0 </htailarm>
    <vtailarea unit="FT2"> 0 </vtailarea> <vtailarm unit="FT"> 0 </vtailarm>
    <location name="AERORP" unit="IN"> <x> %(cgx)s </x> <y> %(cgy)s </y> <z> %(cgz)s </z> </location>
    <location name="EYEPOINT" unit="IN"> <x> %(cgx)s </x> <y> 0 </y> <z> %(cgz)s </z> </location>
    <location name="VRP" unit="IN"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
  </metrics>
  <mass_balance negated_crossproduct_inertia="false">
    <ixx unit="SLUG*FT2"> %(ixx)s </ixx>
    <iyy unit="SLUG*FT2"> %(iyy)s </iyy>
    <izz unit="SLUG*FT2"> %(izz)s </izz>
    <ixz unit="SLUG*FT2"> %(ixz)s </ixz>
    <emptywt unit="LBS"> %(empty)s </emptywt>
    <location name="CG" unit="IN"> <x> %(cgx)s </x> <y> %(cgy)s </y> <z> %(cgz)s </z> </location>
  </mass_balance>
  <ground_reactions>
%(gear)s
  </ground_reactions>
  <external_reactions>
    <!-- The airframe feels the rotors' drive, not their blades. JSBSim applies each rotor's aerodynamic
         torque to the airframe, as though the airframe held the rotor's speed; here the rotor speed is a
         state of the flight control system - the engine's torque less the rotors' spins them up - and the
         engine drives through a freewheel. What the drive does not carry (the rotors' spin-up, and all of
         an air-driven rotor's torque) is given back about the main rotor's shaft. -->
    <moment name="rotor-drive" frame="BODY">
      <function> <product> <value> %(sense)d </value> <property> fcs/%(n)s/armed </property>
        <difference> <property> fcs/%(n)s/drive-torque </property>
          <sum> <property> propulsion/engine[0]/torque-lbsft </property>
            <product> <property> propulsion/engine[1]/torque-lbsft </property> <value> %(trk)s </value> </product> </sum>
        </difference> </product> </function>
      <direction> <x> %(sx)s </x> <y> 0 </y> <z> %(sz)s </z> </direction>
    </moment>
  </external_reactions>
  <propulsion>
    <engine file="%(n)s_engine">
%(feed)s      <thruster file="%(n)s_rotor">
        <location unit="IN"> <x> %(hx)s </x> <y> %(hy)s </y> <z> %(hz)s </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> %(mast)s </pitch> <yaw> 0 </yaw> </orient>
        <sense> %(sense)d </sense>
      </thruster>
    </engine>
    <engine file="%(n)s_tail_drive">
      <thruster file="%(n)s_tail_rotor">
        <location unit="IN"> <x> %(tx)s </x> <y> %(ty)s </y> <z> %(tz)s </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> %(cant)s </pitch> <yaw> %(tyaw)d </yaw> </orient>
        <sense> 1 </sense>
      </thruster>
    </engine>
%(tank)s  </propulsion>
  <flight_control name="%(n)s">
%(profile)s%(fcs)s  </flight_control>
  <aerodynamics>
%(aero)s  </aerodynamics>
</fdm_config>
""" % dict(n=name, desc=spec["aircraft"]["description"], refs=references_xml(spec, "    "), area=_f(math.pi * mr["radius_ft"] ** 2), span=_f(2 * mr["radius_ft"]),
           chord=_f(mr["chord_ft"]), cgx=_f(cg[0]), cgy=_f(cg[1]), cgz=_f(cg[2]), ixx=_f(m["ixx"]), iyy=_f(m["iyy"]),
           izz=_f(m["izz"]), ixz=_f(m.get("ixz", 0.0)), empty=_f(m["weight_lb"] - (fuel["capacity_lb"] if fuel else 0.0)),
           feed="        <feed>0</feed>\n" if fuel else "",
           tank=("""    <tank type="FUEL">
      <location unit="IN"> <x> %s </x> <y> %s </y> <z> %s </z> </location>
      <capacity unit="LBS"> %s </capacity>
      <contents unit="LBS"> %s </contents>
    </tank>
""" % (_f(cg[0]), _f(cg[1]), _f(cg[2]), _f(fuel["capacity_lb"]), _f(fuel["capacity_lb"]))) if fuel else "",
           gear="\n".join(contacts),
           hx=_f(mr["hub_in"][0]), hy=_f(mr["hub_in"][1]), hz=_f(mr["hub_in"][2]), mast=_f(90.0 - mr.get("mast_tilt_deg", 0.0)),
           sense=1 if mr.get("sense", "ccw") == "ccw" else -1, tx=_f(tr["hub_in"][0]), ty=_f(tr["hub_in"][1]),
           tz=_f(tr["hub_in"][2]), cant=_f(tr.get("cant_deg", 0.0)), tyaw=tr_yaw, profile=profile_xml,
           fcs=_fcs(spec, rd, p_max), aero=_aero(spec, rd), trk=_f(rd["tr_omega"] / rd["omega"]),
           # the main rotor's shaft, down along it (JSBSim's body axes: x forward, z down), which its torque turns about
           sx=_f(-math.sin(math.radians(mr.get("mast_tilt_deg", 0.0)))), sz=_f(math.cos(math.radians(mr.get("mast_tilt_deg", 0.0)))))
    _write(os.path.join(out_dir, name + ".xml"), xml)
    return dict(rd, p_max=p_max)


# --- the flight control system -------------------------------------------------------------------

def _fcs(spec, rd, p_max):
    name = spec["aircraft"]["name"]
    c, tr = spec["controls"], spec["rotor"]["tail"]
    t = c["travel_in"]
    p = "fcs/" + name + "/"
    out = []
    out.append("""    <channel name="pilot">
      <!-- the controls in inches: stick and pedals from the centre, + forward, + right, + right
           pedal; the collective from the bottom -->
      <pure_gain name="%(p)slong-in"> <input> fcs/elevator-cmd-norm </input> <gain> %(lo)s </gain> </pure_gain>
      <pure_gain name="%(p)slat-in"> <input> fcs/aileron-cmd-norm </input> <gain> %(la)s </gain> </pure_gain>
      <pure_gain name="%(p)spedal-in"> <input> fcs/rudder-cmd-norm </input> <gain> -%(pe)s </gain> </pure_gain>
      <pure_gain name="%(p)scoll-in"> <input> fcs/throttle-cmd-norm[0] </input> <gain> %(co)s </gain>
        <clipto> <min> 0 </min> <max> %(co)s </max> </clipto> </pure_gain>
    </channel>
""" % dict(p=p, lo=_f(t["longitudinal"] / 2), la=_f(t["lateral"] / 2), pe=_f(t["pedal"] / 2), co=_f(t["collective"])))
    long_terms = ["<property> %slong-in </property>" % p]
    lat_terms = ["<property> %slat-in </property>" % p]
    pedal_terms = ["<property> %spedal-in </property>" % p]
    # the Bell stabilizer bar (TM-73254 eq. 29-30): the shaft-axis rates, lagged, into the cyclic
    bar = c.get("stabilizer_bar")
    if bar:
        tilt = math.radians(spec["rotor"]["main"].get("mast_tilt_deg", 0.0))
        out.append("""    <channel name="stabilizer bar">
      <fcs_function name="%(p)sp-shaft">
        <function> <sum> <product> <value> %(ct)s </value> <property> velocities/p-rad_sec </property> </product>
          <product> <value> %(st)s </value> <property> velocities/r-rad_sec </property> </product> </sum> </function>
      </fcs_function>
      <lag_filter name="%(p)sbar-p"> <input> %(p)sp-shaft </input> <c1> %(c1)s </c1> </lag_filter>
      <lag_filter name="%(p)sbar-q"> <input> velocities/q-rad_sec </input> <c1> %(c1)s </c1> </lag_filter>
    </channel>
""" % dict(p=p, ct=_f(math.cos(tilt)), st=_f(math.sin(tilt)), c1=_f(1.0 / bar["tau_s"])))
    # the pitch bias actuator (TM-85890 figure 11)
    if c.get("pitch_bias_actuator") == "uh60":
        out.append("""    <channel name="pitch bias actuator">
      <!-- TM-85890 figure 11: pitch attitude, pitch rate and airspeed (80 to 180 kt) to an actuator
           of 15 %% authority, 3 %%/s, added to the longitudinal cyclic (0.1 in per %% aft) -->
      <fcs_function name="%(p)sq-deg"> <function> <todegrees> <property> velocities/q-rad_sec </property> </todegrees> </function> </fcs_function>
      <lag_filter name="%(p)sairspeed-kt"> <input> %(p)su-kt </input> <c1> 2.5126 </c1> </lag_filter>
      <fcs_function name="%(p)spba-cmd-pct">
        <function> <sum>
          <product> <value> -0.375 </value> <max> <value> -20 </value> <min> <value> 20 </value> <property> attitude/theta-deg </property> </min> </max> </product>
          <product> <value> -0.13125 </value> <max> <value> -34.5 </value> <min> <value> 34.5 </value> <property> %(p)sq-deg </property> </min> </max> </product>
          <product> <value> 0.0525 </value> <max> <value> 0 </value> <min> <value> 100 </value> <difference> <property> %(p)sairspeed-kt </property> <value> 80 </value> </difference> </min> </max> </product>
          <value> -4.9 </value>
        </sum> </function>
      </fcs_function>
      <actuator name="%(p)spba-pct"> <input> %(p)spba-cmd-pct </input>
        <lag> 10 </lag> <rate_limit> 3 </rate_limit> <clipto> <min> -15 </min> <max> 15 </max> </clipto> </actuator>
      <pure_gain name="%(p)spba-in"> <input> %(p)spba-pct </input> <gain> -0.1000122 </gain> </pure_gain>
    </channel>
""" % dict(p=p))
        long_terms.append("<property> %spba-in </property>" % p)
    # the stabilator (TM-85890 figure 10)
    if c.get("stabilator") == "uh60":
        if c.get("pitch_bias_actuator") != "uh60":
            raise ValueError("the uh60 stabilator shares the pitch bias actuator's airspeed filter")
        out.append("""    <channel name="stabilator">
      <!-- TM-85890 figure 10: the incidence (deg, + trailing edge down) from pitch rate, lateral
           acceleration, airspeed and collective, through the servo, a rate and a position limit -->
      <second_order_filter name="%(p)say-sensed"> <input> accelerations/a-pilot-y-ft_sec2 </input>
        <c1> 0 </c1> <c2> 0 </c2> <c3> 1 </c3> <c4> 0.0254 </c4> <c5> 0.233 </c5> <c6> 1 </c6> </second_order_filter>
      <fcs_function name="%(p)scoll-pct"> <function> <product> <property> %(p)scoll-in </property> <value> %(pct)s </value> </product> </function> </fcs_function>
      <fcs_function name="%(p)sstab-cmd-deg">
        <function> <sum>
          <value> 42 </value>
          <product> <value> 0.16 </value> <property> %(p)sq-deg </property> </product>
          <product> <value> -0.0433 </value> <property> %(p)say-sensed </property>
            <max> <value> 0 </value> <min> <value> 30 </value> <difference> <property> %(p)sairspeed-kt </property> <value> 30 </value> </difference> </min> </max> </product>
          <product> <value> -1 </value>
            <max> <value> 0 </value> <min> <value> 50 </value> <difference> <property> %(p)sairspeed-kt </property> <value> 30 </value> </difference> </min> </max>
            <sum> <value> 0.67 </value>
              <product> <value> 0.003286 </value> <max> <value> 0 </value> <min> <value> 50 </value> <difference> <value> 50 </value> <property> %(p)scoll-pct </property> </difference> </min> </max> </product>
              <product> <value> 0.003286 </value> <max> <value> 0 </value> <min> <value> 20 </value> <difference> <value> 70 </value> <property> %(p)scoll-pct </property> </difference> </min> </max> </product>
            </sum> </product>
          <product> <value> -0.1119 </value>
            <max> <value> 0 </value> <min> <value> 67 </value> <difference> <property> %(p)sairspeed-kt </property> <value> 80 </value> </difference> </min> </max> </product>
        </sum> </function>
      </fcs_function>
      <second_order_filter name="%(p)sstab-servo"> <input> %(p)sstab-cmd-deg </input>
        <c1> 0 </c1> <c2> 0 </c2> <c3> 1 </c3> <c4> 0.0254 </c4> <c5> 0.223 </c5> <c6> 1 </c6> </second_order_filter>
      <actuator name="%(p)sstab-deg"> <input> %(p)sstab-servo </input>
        <rate_limit> 7 </rate_limit> <clipto> <min> -8 </min> <max> 39 </max> </clipto> </actuator>
      <fcs_function name="%(p)sstab-rad"> <function> <toradians> <property> %(p)sstab-deg </property> </toradians> </function>
        <output> fcs/elevator-pos-rad </output> <!-- the platform reports it (VehicleState::elevatorRad); the model's stabilator follows -->
      </fcs_function>
    </channel>
""" % dict(p=p, pct=_f(100.0 / t["collective"])))
    mix = c.get("mixing", {})
    for key, target, source in (("collective_to_longitudinal", long_terms, "coll-in"), ("pedal_to_longitudinal", long_terms, "pedal-in"),
                                ("collective_to_lateral", lat_terms, "coll-in"), ("collective_to_pedal", pedal_terms, "coll-in")):
        if key in mix:
            target.append("<product> <value> %s </value> <property> %s%s </property> </product>" % (_f(mix[key]), p, source))
    if "roll_rate_to_longitudinal" in mix:
        long_terms.append("<product> <value> %s </value> <property> velocities/p-rad_sec </property> </product>" % _f(mix["roll_rate_to_longitudinal"]))
    if "pitch_rate_to_lateral" in mix:
        lat_terms.append("<product> <value> %s </value> <property> velocities/q-rad_sec </property> </product>" % _f(mix["pitch_rate_to_lateral"]))
    # the swashplate: the cyclic from the mixed stick (and the bar), the control axis following it with the
    # rotor's flapping lag, the rigging added after (a constant); the collective and the tail rotor's pitch
    b1 = ["<product> <value> %s </value> <property> %slong-mixed </property> </product>" % (_f(c["longitudinal_rad_per_in"]), p)]
    a1 = ["<product> <value> %s </value> <property> %slat-mixed </property> </product>" % (_f(c["lateral_rad_per_in"]), p)]
    if bar:
        b1.append("<product> <value> %s </value> <property> %sbar-q </property> </product>" % (_f(bar["gain_s"]), p))
        a1.append("<product> <value> %s </value> <property> %sbar-p </property> </product>" % (_f(-bar["gain_s"]), p))
    flap_lag = c.get("flap_lag_s", rd["flap_lag"])
    mr = spec["rotor"]["main"]
    if "collective_root_rad" in c:
        coll = """      <fcs_function name="%(p)stheta0-root-rad">
        <function> <sum> <value> %(c0)s </value> <product> <value> %(cs)s </value> <property> %(p)scoll-in </property> </product> </sum> </function>
        <output> propulsion/engine[0]/collective-ctrl-rad </output>
      </fcs_function>
""" % dict(p=p, c0=_f(c["collective_root_rad"]), cs=_f(c["collective_rad_per_in"]))
    else:
        # the 3/4-radius pitch (the report's untwisted blade) and its lag; the root is 3/4 of the twist above it
        coll = """      <pure_gain name="%(p)stheta75-cmd"> <input> %(p)scoll-in </input> <gain> %(cs)s </gain> </pure_gain>
      <lag_filter name="%(p)stheta75-rad"> <input> %(p)stheta75-cmd </input> <c1> %(cc)s </c1> </lag_filter>
      <fcs_function name="%(p)stheta0-root-rad">
        <function> <sum> <property> %(p)stheta75-rad </property> <value> %(off)s </value> </sum> </function>
        <output> propulsion/engine[0]/collective-ctrl-rad </output>
      </fcs_function>
""" % dict(p=p, cs=_f(c["collective_rad_per_in"]), cc=_f(1.0 / c.get("collective_lag_s", 0.01)),
           off=_f(-0.75 * math.radians(mr["twist_deg"])))
    out.append("""    <channel name="swashplate">
      <fcs_function name="%(p)slong-mixed"> <function> %(lt)s </function> </fcs_function>
      <fcs_function name="%(p)slat-mixed"> <function> %(at)s </function> </fcs_function>
      <fcs_function name="%(p)spedal-mixed"> <function> %(pt)s </function> </fcs_function>
      <fcs_function name="%(p)sb1c-rad"> <function> %(b1)s </function> </fcs_function>
      <fcs_function name="%(p)sa1c-rad"> <function> %(a1)s </function> </fcs_function>
      <lag_filter name="%(p)sb1s-lag"> <input> %(p)sb1c-rad </input> <c1> %(cf)s </c1> </lag_filter>
      <lag_filter name="%(p)sa1s-lag"> <input> %(p)sa1c-rad </input> <c1> %(cf)s </c1> </lag_filter>
      <fcs_function name="%(p)sb1s-rad">
        <function> <sum> <property> %(p)sb1s-lag </property> <value> %(br)s </value> </sum> </function>
        <output> propulsion/engine[0]/longitudinal-ctrl-rad </output>
      </fcs_function>
      <fcs_function name="%(p)sa1s-rad">
        <function> <sum> <property> %(p)sa1s-lag </property> <value> %(ar)s </value> </sum> </function>
        <output> propulsion/engine[0]/lateral-ctrl-rad </output>
      </fcs_function>
%(coll)s      <fcs_function name="%(p)stheta-tr-rad">
        <function> <sum> <value> %(p0)s </value> <product> <value> %(ps)s </value> <property> %(p)spedal-mixed </property> </product> </sum> </function>
        <output> propulsion/engine[1]/antitorque-ctrl-rad </output>
      </fcs_function>
    </channel>
""" % dict(p=p, lt=_sum(long_terms), at=_sum(lat_terms), pt=_sum(pedal_terms), b1=_sum(b1), a1=_sum(a1),
           cf=_f(1.0 / flap_lag), br=_f(c.get("longitudinal_rigging_rad", 0.0)), ar=_f(c.get("lateral_rigging_rad", 0.0)),
           coll=coll, p0=_f(c["pedal_zero_rad"]), ps=_f(c["pedal_rad_per_in"])))
    if spec["airframe"]["model"] == "tm73254":
        rows = "\n".join("            %8.3f %9.5f" % tuple(r) for r in spec["airframe"]["stabilizer_incidence"])
        out.append("""    <channel name="stabilizer">
      <fcs_function name="%(p)sstab-rad">
        <!-- the stabilizer is linked to the longitudinal stick (TM-73254 table 1) -->
        <function> <table> <independentVar> %(p)slong-in </independentVar> <tableData>
%(rows)s
          </tableData> </table> </function>
        <output> fcs/elevator-pos-rad </output> <!-- the platform reports it (VehicleState::elevatorRad); the model's elevator follows -->
      </fcs_function>
    </channel>
""" % dict(p=p, rows=rows))
    out.append(_engine_channel(spec, rd, p_max, p))
    return "".join(out)


def _engine_channel(spec, rd, p_max, p):
    gov = spec["engine"].get("governor", {})
    fuel = fuel_of(spec)
    # with fuel, the power reaches the rotor only while the tank has some (1 exactly: a flight with fuel is the
    # same bit for bit); empty, the engine is starved (JSBSim patched) and the rotor is left to the air
    fed = ("""      <fcs_function name="%(p)sfed">
        <function> <gt> <property> propulsion/total-fuel-lbs </property> <value> 0 </value> </gt> </function>
      </fcs_function>
""" % dict(p=p)) if fuel else ""
    fed_term = (" <property> %sfed </property>" % p) if fuel else ""
    trk = rd["tr_omega"] / rd["omega"]
    lag = gov.get("lag_s", 0.0)
    governor_out = "%sgovernor" % p
    lag_xml = ""
    if lag > 0:
        lag_xml = """      <lag_filter name="%(p)sgovernor-power"> <input> %(p)sgovernor </input> <c1> %(c)s </c1> </lag_filter>
""" % dict(p=p, c=_f(1.0 / lag))
        governor_out = "%sgovernor-power" % p
    return """    <channel name="engine">
      <!-- The rotor speed is a state here: the engine's power less the rotors' drives it through the
           drive's inertia; a governor holds the design's rpm, the load fed forward. 'armed' is 0 on
           the first frame after a load or a reset, 1 after: JSBSim resets the components' states but
           not their output properties, nor the rotors' torques, so what the channel reads from the
           previous frame is gated by it then. -->
      <fcs_function name="%(p)sone"> <function> <value> 1 </value> </function> </fcs_function>
      <lag_filter name="%(p)sarmed-lag"> <input> %(p)sone </input> <c1> 1000 </c1> </lag_filter>
      <fcs_function name="%(p)sarmed"> <function> <gt> <property> %(p)sarmed-lag </property> <value> 0.5 </value> </gt> </function> </fcs_function>
      <fcs_function name="%(p)sdelta-omega-prev">
        <function> <product> <property> %(p)sarmed </property> <property> %(p)sdelta-omega </property> </product> </function>
      </fcs_function>
      <fcs_function name="%(p)somega">
        <function> <sum> <value> %(om)s </value> <property> %(p)sdelta-omega-prev </property> </sum> </function>
      </fcs_function>
      <fcs_function name="%(p)srpm-error"> <function> <product> <value> -1 </value> <property> %(p)sdelta-omega-prev </property> </product> </function> </fcs_function>
      <pid name="%(p)sgovernor"> <input> %(p)srpm-error </input>
        <kp> %(kp)s </kp> <ki> %(ki)s </ki> <kd> %(kd)s </kd>
        <trigger> %(p)spower-saturated </trigger> </pid>
%(lag)s      <fcs_function name="%(p)sload-power">
        <function> <quotient> <product> <property> %(p)sarmed </property> <property> %(p)somega </property> <sum>
          <property> propulsion/engine[0]/torque-lbsft </property>
          <product> <property> propulsion/engine[1]/torque-lbsft </property> <value> %(trk)s </value> </product>
        </sum> </product> <value> %(pmax)s </value> </quotient> </function>
      </fcs_function>
      <fcs_function name="%(p)spower-demand">
        <function> <sum> <property> %(p)sload-power </property> <property> %(gout)s </property> </sum> </function>
      </fcs_function>
      <fcs_function name="%(p)spower">
        <function> <max> <value> 0 </value> <min> <value> 1 </value> <property> %(p)spower-demand </property> </min> </max> </function>
        <output> fcs/throttle-pos-norm[0] </output>
      </fcs_function>
      <fcs_function name="%(p)spower-saturated">
        <function> <gt> <abs> <difference> <property> %(p)spower-demand </property> <property> %(p)spower </property> </difference> </abs> <value> 1e-9 </value> </gt> </function>
      </fcs_function>
      <pure_gain name="%(p)stail-drive-throttle"> <input> %(p)spower </input> <gain> 0 </gain>
        <output> fcs/throttle-pos-norm[1] </output> </pure_gain>
%(fed)s      <fcs_function name="%(p)snet-torque">
        <function> <product> <property> %(p)sarmed </property> <difference>
          <quotient> <product> <property> %(p)spower </property> <value> %(pmax)s </value>%(fed_term)s </product>
            <max> <value> 1 </value> <property> %(p)somega </property> </max> </quotient>
          <sum> <property> propulsion/engine[0]/torque-lbsft </property>
            <product> <property> propulsion/engine[1]/torque-lbsft </property> <value> %(trk)s </value> </product> </sum>
        </difference> </product> </function>
      </fcs_function>
      <!-- the rotor speed stops where the rotors' do (FGRotor's minimum and 130 %%): past it, and pushed further,
           it is held there, the integration stopped -->
      <fcs_function name="%(p)somega-stop">
        <function> <or>
          <and> <ge> <property> %(p)sdelta-omega </property> <value> %(hi)s </value> </ge>
            <gt> <property> %(p)snet-torque </property> <value> 0 </value> </gt> </and>
          <and> <le> <property> %(p)sdelta-omega </property> <value> %(lo)s </value> </le>
            <lt> <property> %(p)snet-torque </property> <value> 0 </value> </lt> </and>
        </or> </function>
      </fcs_function>
      <pid name="%(p)sdelta-omega"> <input> %(p)snet-torque </input>
        <kp> 0 </kp> <ki type="trap"> %(jinv)s </ki> <kd> 0 </kd>
        <trigger> %(p)somega-stop </trigger> <clipto> <min> %(lo)s </min> <max> %(hi)s </max> </clipto> </pid>
      <!-- the engine's torque on the main rotor's shaft: what the airframe feels of the rotors (the
           external reaction rotor-drive) -->
      <fcs_function name="%(p)sdrive-torque">
        <function> <quotient> <product> <property> %(p)spower </property> <value> %(pmax)s </value>%(fed_term)s </product>
          <max> <value> 1 </value> <property> %(p)somega </property> </max> </quotient> </function>
      </fcs_function>
      <fcs_function name="%(p)srotor-rpm">
        <function> <product> <sum> <value> %(om)s </value> <property> %(p)sdelta-omega </property> </sum> <value> %(rpmk)s </value> </product> </function>
        <output> propulsion/engine[0]/x-rpm-dict </output>
      </fcs_function>
    </channel>
""" % dict(p=p, om=_f(rd["omega"]), kp=_f(gov.get("kp", 0.3)), ki=_f(gov.get("ki", 0.3)), kd=_f(gov.get("kd", 0.0)),
           lag=lag_xml, gout=governor_out, trk=_f(trk), pmax=_f(p_max), jinv=_f(1.0 / rd["j"]), rpmk=_f(60 / (2 * math.pi)),
           fed=fed, fed_term=fed_term, hi=_f((ROTOR_MAX_SHARE - 1.0) * rd["omega"]), lo=_f(ROTOR_MIN_RPM * 2 * math.pi / 60 - rd["omega"]))


# --- the airframe -------------------------------------------------------------------------------

def _aero(spec, rd):
    model = spec["airframe"]["model"]
    if model == "tm73254":
        return _aero_tm73254(spec, rd)
    if model == "tm85890":
        return _aero_tm85890(spec, rd)
    raise ValueError("unknown airframe model %r (tm73254, tm85890)" % model)


def _tail_rotor_damping(spec, rd):
    """The tail rotor's damping beyond the isolated rotor's, where the design fits it to flight
    (TM-73254 eq. 37-38): the isolated rotor's own, dT/dv in hover by momentum and blade element,
    is what FGRotor supplies; the difference acts at the tail rotor. Returns (k, l, h) or None."""
    tr = spec["rotor"]["tail"]
    if "damping_lb_per_fps" not in tr:
        return None
    R, sigma = tr["radius_ft"], tr["blades"] * rd["tr_chord"] / (math.pi * tr["radius_ft"])
    vt = rd["tr_omega"] * R
    rho_a_vt2 = RHO0 * math.pi * R ** 2 * vt ** 2
    # the hover's tail rotor thrust: the main rotor's torque over the arm, from the hover power at 60 % of the limit
    l_tr, h_tr = tr["damping_arm_ft"]
    thrust = 0.5 * spec["engine"]["power_shp"] * HP / rd["omega"] / l_tr
    lam = math.sqrt(thrust / rho_a_vt2 / 2)
    a = tr["lift_slope"]
    own = rho_a_vt2 / vt * 2 * a * sigma * lam / (16 * lam + a * sigma)
    return tr["damping_lb_per_fps"] - own, l_tr, h_tr


def _aero_tm73254(spec, rd):
    af = spec["airframe"]
    fu, st, fi = af["fuselage"], af["stabilizer"], af["fin"]
    name = spec["aircraft"]["name"]
    a = "aero/" + name + "/"
    t = math.radians(spec["rotor"]["main"].get("mast_tilt_deg", 0.0))
    ct, s_t = math.cos(t), math.sin(t)
    h2 = st["h1"] * math.tan(math.radians(st["stall_deg"]))
    k2 = fi["k1"] * math.tan(math.radians(fi["stall_deg"]))
    tr_damp = _tail_rotor_damping(spec, rd)
    d = dict(a=a, p="fcs/" + name + "/", ct=_f(ct), st=_f(s_t), nst=_f(-s_t), d1=_f(fu["d1"]), d2=_f(fu["d2"]), d3=_f(fu["d3"]),
             l1=_f(fu["l1"]), y1=_f(fu["y1"]), m1=_f(fu["m1"]), n1=_f(fu["n1"]), h1=_f(st["h1"]), h2=_f(h2), h4=_f(st["h4"]),
             lhs=_f(st["arm_ft"]), k1=_f(fi["k1"]), k2=_f(k2), f1=_f(fi["f1"]), lvf=_f(fi["arm_ft"]),
             sa=_f(math.radians(st["stall_deg"])), sb=_f(math.pi - math.radians(st["stall_deg"])),
             fa=_f(math.radians(fi["stall_deg"])), fb=_f(math.pi - math.radians(fi["stall_deg"])))
    s = """    <!-- TM-73254 eq. 42-60, in the report's body axes - the shaft's - the velocities turned into them
         and the forces and moments back -->
    <function name="%(a)su-s">
      <sum> <product> <value> %(ct)s </value> <property> velocities/u-aero-fps </property> </product>
        <product> <value> %(st)s </value> <property> velocities/w-aero-fps </property> </product> </sum>
    </function>
    <function name="%(a)sw-s">
      <sum> <product> <value> %(nst)s </value> <property> velocities/u-aero-fps </property> </product>
        <product> <value> %(ct)s </value> <property> velocities/w-aero-fps </property> </product> </sum>
    </function>
    <function name="%(a)sr-s">
      <sum> <product> <value> %(nst)s </value> <property> velocities/p-aero-rad_sec </property> </product>
        <product> <value> %(ct)s </value> <property> velocities/r-aero-rad_sec </property> </product> </sum>
    </function>
    <function name="%(a)ssign-u">
      <description> sgn u, smoothed over 1 ft/s so the force is continuous through hover </description>
      <quotient> <property> %(a)su-s </property>
        <sqrt> <sum> <product> <property> %(a)su-s </property> <property> %(a)su-s </property> </product> <value> 1 </value> </sum> </sqrt> </quotient>
    </function>
    <function name="%(a)sfus-x-s">
      <description> eq. 42: X_F = -D1 u|u| + L1 w^2 sgn u </description>
      <sum>
        <product> <value> -%(d1)s </value> <property> %(a)su-s </property> <abs> <property> %(a)su-s </property> </abs> </product>
        <product> <value> %(l1)s </value> <property> %(a)sw-s </property> <property> %(a)sw-s </property> <property> %(a)ssign-u </property> </product>
      </sum>
    </function>
    <function name="%(a)sfus-z-s">
      <description> eq. 44: Z_F = w (-D3 |w| - L1 |u|) </description>
      <product> <property> %(a)sw-s </property>
        <difference> <product> <value> -%(d3)s </value> <abs> <property> %(a)sw-s </property> </abs> </product>
          <product> <value> %(l1)s </value> <abs> <property> %(a)su-s </property> </abs> </product> </difference> </product>
    </function>
    <function name="%(a)sw-h">
      <description> eq. 54: the stabilizer's normal velocity: its incidence, the pitch rate and the rotor's downwash </description>
      <sum> <property> %(a)sw-s </property>
        <product> <property> %(a)su-s </property> <property> %(p)sstab-rad </property> </product>
        <product> <value> %(lhs)s </value> <property> velocities/q-aero-rad_sec </property> </product>
        <product> <value> -1 </value> <property> propulsion/engine[0]/vi-fps </property> </product> </sum>
    </function>
    <function name="%(a)salpha-h">
      <atan2> <property> %(a)sw-h </property> <property> %(a)su-s </property> </atan2>
    </function>
    <function name="%(a)sstab-z-s">
      <description> eq. 57-59: linear to the stall, and within it of reversed flow; stalled between </description>
      <sum>
        <product> <value> -%(h4)s </value> <property> %(a)sw-h </property> <abs> <property> %(a)sw-h </property> </abs> </product>
        <ifthen> <or> <le> <abs> <property> %(a)salpha-h </property> </abs> <value> %(sa)s </value> </le>
                      <ge> <abs> <property> %(a)salpha-h </property> </abs> <value> %(sb)s </value> </ge> </or>
          <product> <value> -%(h1)s </value> <property> %(a)sw-h </property> <property> %(a)su-s </property> </product>
          <product> <value> -%(h2)s </value> <property> %(a)su-s </property> <property> %(a)su-s </property>
            <sign> <property> %(a)salpha-h </property> </sign> </product>
        </ifthen>
      </sum>
    </function>
    <function name="%(a)sv-f">
      <description> eq. 48: the fin's side wind (+ from the left) </description>
      <sum> <product> <value> -1 </value> <property> velocities/v-aero-fps </property> </product>
        <product> <value> %(lvf)s </value> <property> %(a)sr-s </property> </product> </sum>
    </function>
    <function name="%(a)salpha-f">
      <atan2> <property> %(a)sv-f </property> <property> %(a)su-s </property> </atan2>
    </function>
    <function name="%(a)sfin-y">
      <description> eq. 50-52 </description>
      <sum>
        <product> <value> %(f1)s </value> <property> %(a)sv-f </property> <abs> <property> %(a)sv-f </property> </abs> </product>
        <ifthen> <or> <le> <abs> <property> %(a)salpha-f </property> </abs> <value> %(fa)s </value> </le>
                      <ge> <abs> <property> %(a)salpha-f </property> </abs> <value> %(fb)s </value> </ge> </or>
          <product> <value> %(k1)s </value> <property> %(a)sv-f </property> <property> %(a)su-s </property> </product>
          <product> <value> %(k2)s </value> <property> %(a)su-s </property> <property> %(a)su-s </property>
            <sign> <property> %(a)salpha-f </property> </sign> </product>
        </ifthen>
      </sum>
    </function>
    <function name="%(a)sfus-n-s">
      <description> eq. 46: N_F = -N1 v u </description>
      <product> <value> -%(n1)s </value> <property> velocities/v-aero-fps </property> <property> %(a)su-s </property> </product>
    </function>
    <function name="%(a)sn-s">
      <description> the yawing moment about the shaft: eq. 46 and 53 </description>
      <sum> <property> %(a)sfus-n-s </property>
        <product> <value> -%(lvf)s </value> <property> %(a)sfin-y </property> </product> </sum>
    </function>
""" % d
    y_terms = ["<property> %sfin-y </property>" % a]
    l_terms = ["<product> <value> %(nst)s </value> <property> %(a)sn-s </property> </product>" % d]
    n_terms = ["<product> <value> %(ct)s </value> <property> %(a)sn-s </property> </product>" % d]
    if tr_damp:
        k, l_tr, h_tr = tr_damp
        s += """    <function name="%(a)str-damping-y">
      <description> the tail rotor's damping beyond the isolated rotor's: the design's fit to flight (TM-73254
        eq. 37-38) less what the rotor model supplies, at the tail rotor </description>
      <product> <value> %(k)s </value>
        <sum> <property> velocities/v-aero-fps </property>
          <product> <value> %(nl)s </value> <property> velocities/r-aero-rad_sec </property> </product>
          <product> <value> %(h)s </value> <property> velocities/p-aero-rad_sec </property> </product> </sum> </product>
    </function>
""" % dict(a=a, k=_f(-k), nl=_f(-l_tr), h=_f(h_tr))
        y_terms.append("<property> %str-damping-y </property>" % a)
        l_terms.append("<product> <value> %s </value> <property> %str-damping-y </property> </product>" % (_f(h_tr), a))
        n_terms.append("<product> <value> %s </value> <property> %str-damping-y </property> </product>" % (_f(-l_tr), a))
    s += """    <axis name="X">
      <function name="aero/force/%(n)s-x">
        <sum>
          <product> <value> %(ct)s </value> <property> %(a)sfus-x-s </property> </product>
          <product> <value> %(nst)s </value> <sum> <property> %(a)sfus-z-s </property> <property> %(a)sstab-z-s </property> </sum> </product>
        </sum>
      </function>
    </axis>
    <axis name="Y">
      <function name="aero/force/%(n)s-y">
        <description> eq. 43, the fin and the tail rotor's damping </description>
        <sum>
          <product> <property> velocities/v-aero-fps </property>
            <difference> <product> <value> -%(d2)s </value> <abs> <property> velocities/v-aero-fps </property> </abs> </product>
              <product> <value> %(y1)s </value> <abs> <property> %(a)su-s </property> </abs> </product> </difference> </product>
          %(y)s
        </sum>
      </function>
    </axis>
    <axis name="Z">
      <function name="aero/force/%(n)s-z">
        <sum>
          <product> <value> %(st)s </value> <property> %(a)sfus-x-s </property> </product>
          <product> <value> %(ct)s </value> <sum> <property> %(a)sfus-z-s </property> <property> %(a)sstab-z-s </property> </sum> </product>
        </sum>
      </function>
    </axis>
    <axis name="ROLL">
      <function name="aero/moment/%(n)s-l"> <sum> %(l)s </sum> </function>
    </axis>
    <axis name="PITCH">
      <function name="aero/moment/%(n)s-m">
        <description> eq. 45 and 60 </description>
        <sum>
          <product> <value> %(m1)s </value> <property> %(a)sw-s </property> <abs> <property> %(a)su-s </property> </abs> </product>
          <product> <value> %(lhs)s </value> <property> %(a)sstab-z-s </property> </product>
        </sum>
      </function>
    </axis>
    <axis name="YAW">
      <function name="aero/moment/%(n)s-n"> <sum> %(nn)s </sum> </function>
    </axis>
""" % dict(d, n=name, y="\n          ".join(y_terms), l=" ".join(l_terms), nn=" ".join(n_terms))
    return s


def _surface(a, which, sf, cg, hub, R, stall_tr_wash=None):
    """TM-84281 appendix E and transformations T5-T8: the horizontal stabilator or the fin through
    360 deg, in the local wind; the main rotor's wash where the wake reaches it."""
    aft = (sf["sta"] - cg[0]) / 12.0
    above = (sf["wl"] - cg[2]) / 12.0
    lift_slope = 2 * math.pi / (1 + 2 / sf["aspect"])
    q = "%s%s-" % (a, which)
    if which == "hs":
        v = """<sum> <property> velocities/v-aero-fps </property> <product> <value> %s </value> <property> velocities/r-aero-rad_sec </property> </product> </sum>""" % _f(-aft)
        w = """<sum> <property> velocities/w-aero-fps </property> <product> <value> %s </value> <property> velocities/q-aero-rad_sec </property> </product>
        <product> <value> %s </value> <property> %swake-hs </property> <property> propulsion/engine[0]/vi-fps </property> </product> </sum>""" % (_f(aft), _f(-sf["rotor_wash"]), a)
    else:
        v = """<sum> <property> velocities/v-aero-fps </property> <product> <value> %s </value> <property> velocities/r-aero-rad_sec </property> </product>
        <product> <value> %s </value> <property> propulsion/engine[1]/vi-fps </property> </product> </sum>""" % (_f(-aft), _f(sf["tail_rotor_wash"] * stall_tr_wash))
        w = """<sum> <property> velocities/w-aero-fps </property> <product> <value> %s </value> <property> velocities/q-aero-rad_sec </property> </product>
        <product> <value> -1 </value> <property> %swake-vf </property> <property> propulsion/engine[0]/vi-fps </property> </product> </sum>""" % (_f(aft), a)
    s = """    <function name="%(q)su"> <property> velocities/u-aero-fps </property> </function>
    <function name="%(q)sv"> %(v)s </function>
    <function name="%(q)sw"> %(w)s </function>
    <function name="%(q)svmag">
      <max> <value> 0.1 </value> <sqrt> <sum> <pow> <property> %(q)su </property> <value> 2 </value> </pow>
        <pow> <property> %(q)sv </property> <value> 2 </value> </pow> <pow> <property> %(q)sw </property> <value> 2 </value> </pow> </sum> </sqrt> </max>
    </function>
""" % dict(q=q, v=v, w=w)
    if which == "hs":
        s += """    <function name="%(q)salpha-raw">
      <description> + the stabilator's incidence (leading edge up: trailing edge down) </description>
      <sum> <atan2> <property> %(q)sw </property> <property> %(q)su </property> </atan2> <property> %(inc)s </property> </sum>
    </function>
    <function name="%(q)salpha"> <atan2> <sin> <property> %(q)salpha-raw </property> </sin> <cos> <property> %(q)salpha-raw </property> </cos> </atan2> </function>
    <function name="%(q)sbeta"> <asin> <quotient> <property> %(q)sv </property> <property> %(q)svmag </property> </quotient> </asin> </function>
    <function name="%(q)sa">
      <product> <value> %(ls)s </value> <pow> <cos> <property> %(q)sbeta </property> </cos> <value> 2 </value> </pow> </product>
    </function>
""" % dict(q=q, inc=sf["incidence_property"], ls=_f(lift_slope))
    else:
        s += """    <function name="%(q)salpha"> <atan2> <property> %(q)sv </property> <property> %(q)su </property> </atan2> </function>
    <function name="%(q)sbeta"> <asin> <quotient> <property> %(q)sw </property> <property> %(q)svmag </property> </quotient> </asin> </function>
    <function name="%(q)sa">
      <product> <value> %(ls)s </value> <pow> <cos> <sum> <property> %(q)sbeta </property> <value> %(sw)s </value> </sum> </cos> <value> 2 </value> </pow> </product>
    </function>
""" % dict(q=q, ls=_f(lift_slope), sw=_f(sf["sweep_rad"]))
    s += """    <function name="%(q)salpha-s">
      <min> <quotient> <value> %(clm)s </value> <property> %(q)sa </property> </quotient> <value> 0.785398 </value> </min>
    </function>
    <function name="%(q)sclm"> <product> <property> %(q)salpha-s </property> <property> %(q)sa </property> </product> </function>
    <function name="%(q)salpha-i">
      <description> the angle folded into 0..90 deg </description>
      <ifthen> <le> <abs> <property> %(q)salpha </property> </abs> <value> 1.570796 </value> </le>
        <abs> <property> %(q)salpha </property> </abs>
        <difference> <value> 3.141593 </value> <abs> <property> %(q)salpha </property> </abs> </difference>
      </ifthen>
    </function>
    <function name="%(q)scl0">
      <ifthen> <lt> <property> %(q)salpha-i </property> <property> %(q)salpha-s </property> </lt>
        <product> <property> %(q)sa </property> <property> %(q)salpha-i </property> </product>
        <ifthen> <lt> <property> %(q)salpha-i </property> <product> <value> 1.2 </value> <property> %(q)salpha-s </property> </product> </lt>
          <difference> <property> %(q)sclm </property>
            <product> <property> %(q)sa </property> <difference> <property> %(q)salpha-i </property> <property> %(q)salpha-s </property> </difference> </product> </difference>
          <product> <value> 0.8 </value> <property> %(q)sclm </property>
            <difference> <value> 1 </value>
              <pow> <quotient>
                <difference> <property> %(q)salpha-i </property> <product> <value> 1.2 </value> <property> %(q)salpha-s </property> </product> </difference>
                <difference> <value> 1.570796 </value> <product> <value> 1.2 </value> <property> %(q)salpha-s </property> </product> </difference>
              </quotient> <value> 2 </value> </pow> </difference> </product>
        </ifthen>
      </ifthen>
    </function>
    <function name="%(q)scl">
      <product> <property> %(q)scl0 </property>
        <ifthen> <le> <abs> <property> %(q)salpha </property> </abs> <value> 1.570796 </value> </le>
          <sign> <property> %(q)salpha </property> </sign>
          <product> <value> -0.8 </value> <sign> <property> %(q)salpha </property> </sign> </product>
        </ifthen> </product>
    </function>
    <function name="%(q)scd">
      <sum>
        <ifthen> <le> <property> %(q)salpha-i </property> <value> 0.35 </value> </le>
          <sum> <value> 0.009 </value> <product> <value> 0.11 </value> <property> %(q)salpha-i </property> <property> %(q)salpha-i </property> </product> </sum>
          <sum> <value> -0.1254 </value> <product> <value> 0.9415 </value> <property> %(q)salpha-i </property> </product>
            <product> <value> 0.977525 </value> <pow> <sin> <property> %(q)salpha-i </property> </sin> <value> 2 </value> </pow> </product> </sum>
        </ifthen>
        <quotient> <product> <property> %(q)scl </property> <property> %(q)scl </property> </product> <value> %(k)s </value> </quotient>
      </sum>
    </function>
    <function name="%(q)sf">
      <product> <value> 0.5 </value> <property> atmosphere/rho-slugs_ft3 </property> <pow> <property> %(q)svmag </property> <value> 2 </value> </pow> <value> %(s)s </value> </product>
    </function>
""" % dict(q=q, clm=_f(sf["cl_max"]), k=_f(0.8 * math.pi * sf["aspect"]), s=_f(sf["area_ft2"] * sf["q_ratio"]))
    if which == "hs":
        s += """    <function name="%(q)sx">
      <product> <property> %(q)sf </property> <sum>
        <product> <value> -1 </value> <property> %(q)scd </property> <cos> <property> %(q)sbeta </property> </cos> <cos> <property> %(q)salpha </property> </cos> </product>
        <product> <property> %(q)scl </property> <sin> <property> %(q)salpha </property> </sin> </product> </sum> </product>
    </function>
    <function name="%(q)sy"> <product> <value> -1 </value> <property> %(q)sf </property> <property> %(q)scd </property> <sin> <property> %(q)sbeta </property> </sin> </product> </function>
    <function name="%(q)sz">
      <product> <property> %(q)sf </property> <sum>
        <product> <value> -1 </value> <property> %(q)scl </property> <cos> <property> %(q)salpha </property> </cos> </product>
        <product> <value> -1 </value> <property> %(q)scd </property> <cos> <property> %(q)sbeta </property> </cos> <sin> <property> %(q)salpha </property> </sin> </product> </sum> </product>
    </function>
""" % dict(q=q)
    else:
        s += """    <function name="%(q)sx">
      <product> <property> %(q)sf </property> <sum>
        <product> <value> -1 </value> <property> %(q)scd </property> <cos> <property> %(q)sbeta </property> </cos> <cos> <property> %(q)salpha </property> </cos> </product>
        <product> <property> %(q)scl </property> <sin> <property> %(q)salpha </property> </sin> </product> </sum> </product>
    </function>
    <function name="%(q)sy">
      <product> <property> %(q)sf </property> <sum>
        <product> <value> -1 </value> <property> %(q)scl </property> <cos> <property> %(q)salpha </property> </cos> </product>
        <product> <value> -1 </value> <property> %(q)scd </property> <cos> <property> %(q)sbeta </property> </cos> <sin> <property> %(q)salpha </property> </sin> </product> </sum> </product>
    </function>
    <function name="%(q)sz"> <product> <value> -1 </value> <property> %(q)sf </property> <property> %(q)scd </property> <sin> <property> %(q)sbeta </property> </sin> </product> </function>
""" % dict(q=q)
    return s, aft, above


# TM-85890 p. 2-3: the fuselage's wind-tunnel regressions (drag, lift, side force; pitch, roll, yaw
# moments over q, ft2 and ft3), in the tunnel's angles: alpha_f and psi_w (deg where a power of it)
_FUSELAGE_85890 = """    <function name="%(a)sfus-d">
      <description> D/q = 90.0555 sin^2 a - 41.5604 cos a + 2.94684 cos 4p - 103.141 cos 2p - 0.535350e-6 p^4 + 160.2049 </description>
      <product> <property> %(a)sfus-qbar </property> <sum>
        <product> <value> 90.0555 </value> <pow> <sin> <property> %(a)sfus-a </property> </sin> <value> 2 </value> </pow> </product>
        <product> <value> -41.5604 </value> <cos> <property> %(a)sfus-a </property> </cos> </product>
        <product> <value> 2.94684 </value> <cos> <product> <value> 4 </value> <property> %(a)sfus-p </property> </product> </cos> </product>
        <product> <value> -103.141 </value> <cos> <product> <value> 2 </value> <property> %(a)sfus-p </property> </product> </cos> </product>
        <product> <value> -0.535350e-6 </value> <pow> <property> %(a)spsi-w-deg </property> <value> 4 </value> </pow> </product>
        <value> 160.2049 </value> </sum> </product>
    </function>
    <function name="%(a)sfus-lift">
      <description> L/q = 29.3616 sin a + 43.4680 sin 2a - 81.8924 sin^2 a - 84.1469 cos a - 0.0821406 p + 3.00102 sin 4p + 0.0323477 p^2 + 85.3496 </description>
      <product> <property> %(a)sfus-qbar </property> <sum>
        <product> <value> 29.3616 </value> <sin> <property> %(a)sfus-a </property> </sin> </product>
        <product> <value> 43.4680 </value> <sin> <product> <value> 2 </value> <property> %(a)sfus-a </property> </product> </sin> </product>
        <product> <value> -81.8924 </value> <pow> <sin> <property> %(a)sfus-a </property> </sin> <value> 2 </value> </pow> </product>
        <product> <value> -84.1469 </value> <cos> <property> %(a)sfus-a </property> </cos> </product>
        <product> <value> -0.0821406 </value> <property> %(a)spsi-w-deg </property> </product>
        <product> <value> 3.00102 </value> <sin> <product> <value> 4 </value> <property> %(a)sfus-p </property> </product> </sin> </product>
        <product> <value> 0.0323477 </value> <pow> <property> %(a)spsi-w-deg </property> <value> 2 </value> </pow> </product>
        <value> 85.3496 </value> </sum> </product>
    </function>
    <function name="%(a)sfus-side">
      <description> Y/q = 35.3999 sin p + 71.8019 sin 2p - 8.04823 sin 4p </description>
      <product> <property> %(a)sfus-qbar </property> <sum>
        <product> <value> 35.3999 </value> <sin> <property> %(a)sfus-p </property> </sin> </product>
        <product> <value> 71.8019 </value> <sin> <product> <value> 2 </value> <property> %(a)sfus-p </property> </product> </sin> </product>
        <product> <value> -8.04823 </value> <sin> <product> <value> 4 </value> <property> %(a)sfus-p </property> </product> </sin> </product> </sum> </product>
    </function>
    <function name="%(a)sfus-pitch">
      <description> M/q = 2.37925 a(deg) + 728.026 sin 2a + 426.760 sin^2 a + 348.072 cos a - 510.581 cos^3 p + 56.111 </description>
      <product> <property> %(a)sfus-qbar </property> <sum>
        <product> <value> 2.37925 </value> <property> %(a)salpha-f-deg </property> </product>
        <product> <value> 728.026 </value> <sin> <product> <value> 2 </value> <property> %(a)sfus-a </property> </product> </sin> </product>
        <product> <value> 426.760 </value> <pow> <sin> <property> %(a)sfus-a </property> </sin> <value> 2 </value> </pow> </product>
        <product> <value> 348.072 </value> <cos> <property> %(a)sfus-a </property> </cos> </product>
        <product> <value> -510.581 </value> <pow> <cos> <property> %(a)sfus-p </property> </cos> <value> 3 </value> </pow> </product>
        <value> 56.111 </value> </sum> </product>
    </function>
    <function name="%(a)sfus-roll">
      <description> l/q by |p|: 0 to 10 deg; to 25: sgn p (455.707 cos^4 p - 428.639); to 90: 614.797 sin p + sgn p (-47.7213 cos 4p - 290.504 cos^3 p + 735.507 cos^4 p - 669.266) </description>
      <product> <property> %(a)sfus-qbar </property>
        <ifthen> <le> <abs> <property> %(a)spsi-w-deg </property> </abs> <value> 10 </value> </le> <value> 0 </value>
          <ifthen> <le> <abs> <property> %(a)spsi-w-deg </property> </abs> <value> 25 </value> </le>
            <product> <sign> <property> %(a)sfus-p </property> </sign>
              <sum> <product> <value> 455.707 </value> <pow> <cos> <property> %(a)sfus-p </property> </cos> <value> 4 </value> </pow> </product> <value> -428.639 </value> </sum> </product>
            <sum> <product> <value> 614.797 </value> <sin> <property> %(a)sfus-p </property> </sin> </product>
              <product> <sign> <property> %(a)sfus-p </property> </sign> <sum>
                <product> <value> -47.7213 </value> <cos> <product> <value> 4 </value> <property> %(a)sfus-p </property> </product> </cos> </product>
                <product> <value> -290.504 </value> <pow> <cos> <property> %(a)sfus-p </property> </cos> <value> 3 </value> </pow> </product>
                <product> <value> 735.507 </value> <pow> <cos> <property> %(a)sfus-p </property> </cos> <value> 4 </value> </pow> </product>
                <value> -669.266 </value> </sum> </product> </sum>
          </ifthen>
        </ifthen> </product>
    </function>
    <function name="%(a)sfus-yaw">
      <description> N/q: |p| to 20 deg: -278.133 sin 2p + 422.644 sin 4p - 1.83172; beyond: 220.0 sin 2p + sgn p (671.0 cos^4 p - 429.0) </description>
      <product> <property> %(a)sfus-qbar </property>
        <ifthen> <le> <abs> <property> %(a)spsi-w-deg </property> </abs> <value> 20 </value> </le>
          <sum> <product> <value> -278.133 </value> <sin> <product> <value> 2 </value> <property> %(a)sfus-p </property> </product> </sin> </product>
            <product> <value> 422.644 </value> <sin> <product> <value> 4 </value> <property> %(a)sfus-p </property> </product> </sin> </product>
            <value> -1.83172 </value> </sum>
          <sum> <product> <value> 220.0 </value> <sin> <product> <value> 2 </value> <property> %(a)sfus-p </property> </product> </sin> </product>
            <product> <sign> <property> %(a)sfus-p </property> </sign>
              <sum> <product> <value> 671.0 </value> <pow> <cos> <property> %(a)sfus-p </property> </cos> <value> 4 </value> </pow> </product> <value> -429.0 </value> </sum> </product> </sum>
        </ifthen> </product>
    </function>
"""


def _wake(a, tag, aft_hub, below_hub, R):
    """1 in the main rotor's wake, 0 out of it (a simplification of TM-84281's interpolated wake
    charts): the wake's column, skewed back by chi = atan(mu / -lambda), spans the hub's x +- R at a
    depth h; smoothed over 2 ft at its edges."""
    h = max(below_hub, 0.5)
    return """    <function name="%(a)swake-%(t)s">
      <description> %(t)s: %(l).1f ft aft of the hub, %(h).1f ft below it </description>
      <product>
        <max> <value> 0 </value> <min> <value> 1 </value>
          <quotient> <difference> <sum> <value> %(r)s </value> <product> <value> %(h)s </value> <property> %(a)swake-tan-chi </property> </product> </sum> <value> %(l)s </value> </difference> <value> 2 </value> </quotient> </min> </max>
        <max> <value> 0 </value> <min> <value> 1 </value>
          <quotient> <difference> <value> %(l)s </value> <difference> <product> <value> %(h)s </value> <property> %(a)swake-tan-chi </property> </product> <value> %(r)s </value> </difference> </difference> <value> 2 </value> </quotient> </min> </max>
      </product>
    </function>
""" % dict(a=a, t=tag, l=aft_hub, h=h, r=_f(R))


def _aero_tm85890(spec, rd):
    af = spec["airframe"]
    name = spec["aircraft"]["name"]
    a = "aero/" + name + "/"
    cg = spec["mass"]["cg_in"]
    mr = spec["rotor"]["main"]
    hub = mr["hub_in"]
    R = mr["radius_ft"]
    fref = af["fuselage_ref_in"]
    f_aft, f_above = (fref[0] - cg[0]) / 12.0, (fref[1] - cg[2]) / 12.0
    hs = dict(af["horizontal"], incidence_property="fcs/%s/stab-rad" % name)
    vf = dict(af["fin"])
    cant = math.radians(spec["rotor"]["tail"].get("cant_deg", 0.0))
    s = """    <!-- the main rotor's wake: its skew angle chi from the vertical, tan chi = mu / -lambda -->
    <function name="%(a)swake-tan-chi">
      <quotient> <property> propulsion/engine[0]/advance-ratio </property>
        <max> <value> 0.002 </value> <abs> <property> propulsion/engine[0]/inflow-ratio </property> </abs> </max> </quotient>
    </function>
""" % dict(a=a)
    s += _wake(a, "fus", (fref[0] - hub[0]) / 12.0, (hub[2] - fref[1]) / 12.0, R)
    s += _wake(a, "hs", (hs["sta"] - hub[0]) / 12.0, (hub[2] - hs["wl"]) / 12.0, R)
    s += _wake(a, "vf", (vf["sta"] - hub[0]) / 12.0, (hub[2] - vf["wl"]) / 12.0, R)
    s += """    <function name="fcs/%(n)s/u-kt"> <product> <property> velocities/u-aero-fps </property> <value> 0.592484 </value> </product> </function>
    <!-- TM-85890's fuselage: its angles with the rotor's downwash and the rates at its reference point -->
    <function name="%(a)sfus-w">
      <sum> <property> velocities/w-aero-fps </property>
        <product> <value> %(fa)s </value> <property> velocities/q-aero-rad_sec </property> </product>
        <product> <value> -1 </value> <property> %(a)swake-fus </property> <property> propulsion/engine[0]/vi-fps </property> </product> </sum>
    </function>
    <function name="%(a)sfus-v">
      <sum> <property> velocities/v-aero-fps </property> <product> <value> %(nfa)s </value> <property> velocities/r-aero-rad_sec </property> </product> </sum>
    </function>
    <function name="%(a)sfus-vmag">
      <max> <value> 1 </value> <sqrt> <sum> <pow> <property> velocities/u-aero-fps </property> <value> 2 </value> </pow>
        <pow> <property> %(a)sfus-v </property> <value> 2 </value> </pow> <pow> <property> %(a)sfus-w </property> <value> 2 </value> </pow> </sum> </sqrt> </max>
    </function>
    <function name="%(a)sfus-qbar">
      <product> <value> 0.5 </value> <property> atmosphere/rho-slugs_ft3 </property> <pow> <property> %(a)sfus-vmag </property> <value> 2 </value> </pow> </product>
    </function>
    <function name="%(a)salpha-f-deg">
      <todegrees> <atan2> <property> %(a)sfus-w </property> <abs> <property> velocities/u-aero-fps </property> </abs> </atan2> </todegrees>
    </function>
    <function name="%(a)spsi-w-deg">
      <product> <value> -1 </value> <todegrees> <atan2> <property> %(a)sfus-v </property>
        <sqrt> <sum> <pow> <property> velocities/u-aero-fps </property> <value> 2 </value> </pow> <pow> <property> %(a)sfus-w </property> <value> 2 </value> </pow> </sum> </sqrt> </atan2> </todegrees> </product>
    </function>
    <function name="%(a)sfus-a"> <toradians> <property> %(a)salpha-f-deg </property> </toradians> </function>
    <function name="%(a)sfus-p"> <toradians> <property> %(a)spsi-w-deg </property> </toradians> </function>
""" % dict(a=a, n=name, fa=_f(f_aft), nfa=_f(-f_aft))
    s += _FUSELAGE_85890 % dict(a=a)
    s += """    <!-- wind to body: drag along the local wind, lift normal to it in the plane of symmetry, the side force along y -->
    <function name="%(a)sfus-x">
      <sum> <product> <value> -1 </value> <property> %(a)sfus-d </property> <quotient> <property> velocities/u-aero-fps </property> <property> %(a)sfus-vmag </property> </quotient> </product>
        <product> <property> %(a)sfus-lift </property> <sin> <property> %(a)sfus-a </property> </sin> </product> </sum>
    </function>
    <function name="%(a)sfus-y">
      <sum> <product> <value> -1 </value> <property> %(a)sfus-d </property> <quotient> <property> %(a)sfus-v </property> <property> %(a)sfus-vmag </property> </quotient> </product>
        <property> %(a)sfus-side </property> </sum>
    </function>
    <function name="%(a)sfus-z">
      <sum> <product> <value> -1 </value> <property> %(a)sfus-d </property> <quotient> <property> %(a)sfus-w </property> <property> %(a)sfus-vmag </property> </quotient> </product>
        <product> <value> -1 </value> <property> %(a)sfus-lift </property> <cos> <property> %(a)sfus-a </property> </cos> </product> </sum>
    </function>
""" % dict(a=a)
    hs_xml, hs_aft, hs_above = _surface(a, "hs", hs, cg, hub, R)
    vf_xml, vf_aft, vf_above = _surface(a, "vf", vf, cg, hub, R, stall_tr_wash=math.cos(cant))
    s += hs_xml + vf_xml

    def moments(aft, above, prefix):
        # a point `aft` ft behind the c.g. and `above` ft above it: L = h Y, M = -h X + a Z, N = -a Y
        return ("<product> <value> %s </value> <property> %sy </property> </product>" % (_f(above), prefix),
                "<product> <value> %s </value> <property> %sx </property> </product> <product> <value> %s </value> <property> %sz </property> </product>"
                % (_f(-above), prefix, _f(aft), prefix),
                "<product> <value> %s </value> <property> %sy </property> </product>" % (_f(-aft), prefix))
    fl, fm, fn = moments(f_aft, f_above, a + "fus-")
    hl, hm, hn = moments(hs_aft, hs_above, a + "hs-")
    vl, vm, vn = moments(vf_aft, vf_above, a + "vf-")
    s += """    <axis name="X"> <function name="aero/force/%(n)s-x"> <sum> <property> %(a)sfus-x </property> <property> %(a)shs-x </property> <property> %(a)svf-x </property> </sum> </function> </axis>
    <axis name="Y"> <function name="aero/force/%(n)s-y"> <sum> <property> %(a)sfus-y </property> <property> %(a)shs-y </property> <property> %(a)svf-y </property> </sum> </function> </axis>
    <axis name="Z"> <function name="aero/force/%(n)s-z"> <sum> <property> %(a)sfus-z </property> <property> %(a)shs-z </property> <property> %(a)svf-z </property> </sum> </function> </axis>
    <axis name="ROLL"> <function name="aero/moment/%(n)s-l"> <sum> <property> %(a)sfus-roll </property> %(fl)s %(hl)s %(vl)s </sum> </function> </axis>
    <axis name="PITCH"> <function name="aero/moment/%(n)s-m"> <sum> <property> %(a)sfus-pitch </property> %(fm)s %(hm)s %(vm)s </sum> </function> </axis>
    <axis name="YAW"> <function name="aero/moment/%(n)s-n"> <sum> <property> %(a)sfus-yaw </property> %(fn)s %(hn)s %(vn)s </sum> </function> </axis>
""" % dict(n=name, a=a, fl=fl, fm=fm, fn=fn, hl=hl, hm=hm, hn=hn, vl=vl, vm=vm, vn=vn)
    return s
