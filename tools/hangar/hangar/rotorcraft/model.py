"""A rotorcraft's model for the viewer (<name>.glb), built as hangar builds a fixed wing's
(model3d.py, shape/airframe.py):

- the airframe - the design's [[body]], [[surface]], [[strut]] and [[gear]], measured from a
  three-view - is one closed, filleted solid painted with the design's livery (paint.toml),
  each control surface cut out on its hinge (fsim:<channel>), the gear on its own nodes;
- each rotor comes from the flight model's own rotor data ([rotor.main], [rotor.tail] or
  [rotors]: its hub, shaft, radius, blades, chord, twist and sense), so what is drawn is where
  and how the simulation's rotor is, with the look of its blades, hub and blur in
  [model.<rotor>]. A rotor is a node the viewer turns at its engine's rpm as the simulation
  reports it (fsim:propeller:<engine>): its origin at the hub, its x axis along the shaft,
  pointing the way the rotor turns (right-handed). Inside it, its blades (fsim:blades:
  <engine>:<rpm>) and a blur disc (fsim:disc:<engine>:<rpm>) stand in for each other past the
  rpm where a blade moves a third of the way to the next between frames at 60 Hz. A look's
  offset_m draws the hub that far along its shaft from the flight model's point (the same line
  of thrust), when the source puts it on the centre line and the drawings beside the pylon.

Frames: hangar's design frame is metres, x aft, y right, z up. A helicopter's flight data are
stations, butt lines and water lines in inches (the same axes); a multirotor's x forward, y left
(turned here). The glTF's origin is the c.g., the point the simulation reports.
"""
import math

import numpy as np

from .. import model3d as m3
from ..geometry import airfoil as af
from ..geometry.aircraft import Aircraft
from ..shape import airframe as sh
from ..shape import meshkit

IN = 0.0254
FRAME_HZ = 60.0


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rgb(c, default):
    if c is None:
        return default
    if isinstance(c, str):
        c = c.lstrip("#")
        return tuple(int(c[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    return tuple(float(x) for x in c)


# -- the rotors, as the flight model has them ------------------------------------------------------

def _sense(rotation):
    """+1 for a rotor turning counter-clockwise seen from the side its thrust points to."""
    return {"ccw": 1.0, "cw": -1.0}[rotation]


def rotors(spec):
    """Each rotor as drawn: its engine, hub (design frame, m), thrust direction, sense (+1:
    counter-clockwise seen from the thrust side), and the blades' plan - from the flight
    data, with their look from [model.<rotor>]."""
    kind = spec["aircraft"]["kind"]
    out = []
    look = spec.get("model", {})
    if kind == "helicopter":
        mr, tr = spec["rotor"]["main"], spec["rotor"]["tail"]
        tilt = math.radians(mr.get("mast_tilt_deg", 0.0))
        m = look.get("main_rotor", {})
        out.append(dict(engine=0, label="main rotor", hub=np.asarray(mr["hub_in"], float) * IN,
                        thrust=np.array([-math.sin(tilt), 0.0, math.cos(tilt)]), sense=_sense(mr.get("sense", mr.get("rotation", "ccw"))),
                        radius=mr["radius_ft"] * 0.3048, blades=int(mr["blades"]), chord=mr["chord_ft"] * 0.3048,
                        twist_deg=mr.get("twist_deg", 0.0), precone_deg=m.get("precone_deg", 0.0), look=m))
        t = look.get("tail_rotor", {})
        side = 1.0 if tr.get("thrust", "right") == "right" else -1.0
        cant = math.radians(tr.get("cant_deg", 0.0))
        thrust = np.array([0.0, side * math.cos(cant), math.sin(cant)])  # canted: its thrust lifts too
        # which way it turns: its top blade moving aft or forward (the look's; the flight model does not care)
        top = np.cross(thrust, [0.0, 0.0, 1.0])  # where the top blade goes when turning ccw about the thrust
        top_aft = t.get("top_blade", "aft") == "aft"
        sense = 1.0 if (top[0] > 0.0) == top_aft else -1.0
        chord = tr["chord_ft"] if "chord_ft" in tr else tr["solidity"] * math.pi * tr["radius_ft"] / tr["blades"]
        out.append(dict(engine=1, label="tail rotor", hub=np.asarray(tr["hub_in"], float) * IN, thrust=thrust, sense=sense,
                        radius=tr["radius_ft"] * 0.3048, blades=int(tr["blades"]), chord=chord * 0.3048,
                        twist_deg=tr.get("twist_deg", 0.0), precone_deg=t.get("precone_deg", 0.0), look=t))
    else:
        from . import multi
        rows, *_ = multi.rotors(spec)
        r = spec["rotors"]
        p = look.get("propeller", {})
        for i, (label, x, y, sense) in enumerate(rows):
            out.append(dict(engine=i, label=label + " rotor", hub=np.array([-x, -y, r.get("height_m", 0.0)]),
                            thrust=np.array([0.0, 0.0, 1.0]), sense=float(sense), radius=r["radius_m"], blades=int(p.get("blades", 2)),
                            chord=p.get("chord_m", 0.12 * r["radius_m"]), twist_deg=0.0, precone_deg=0.0, look=p))
    for rot in out:
        rot["spin"] = rot["sense"] * _unit(rot["thrust"])  # the node's x axis: the way it turns, right-handed
        rot["blur_rpm"] = FRAME_HZ * 60.0 / (3.0 * rot["blades"])
        # drawn where the rotor is along its own shaft: a flight model may place a hub anywhere on
        # the thrust's line (TM-85890's tail rotor: where its shaft crosses the plane of symmetry)
        rot["shaft_hub"] = rot["hub"]
        rot["hub"] = rot["hub"] + float(rot["look"].get("offset_m", 0.0)) * _unit(rot["thrust"])
    return out


def _frame(rot):
    """The rotor's canonical frame in the design frame, as columns: x (a blade's chord, aft
    of its leading edge when the rotor turns ccw about z), y (blade 0's span: fore and aft,
    as the drawings show it; or up, for a rotor on a vertical shaft... never), z (thrust)."""
    t = _unit(rot["thrust"])
    ref = np.array([-1.0, 0.0, 0.0]) if abs(t[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    y = _unit(ref - t * np.dot(ref, t))
    x = np.cross(y, t)
    return np.stack([x, y, t], axis=1)


def _pitch_deg(rot, rr):
    """The blade's pitch at r/R: a propeller's from its geometric pitch, a rotor's from its
    three-quarter-radius pitch and linear twist."""
    look = rot["look"]
    if "pitch_m" in look:  # a propeller: a helix of that pitch
        return np.degrees(np.arctan2(look["pitch_m"], 2.0 * math.pi * np.maximum(rr, 0.05) * rot["radius"]))
    return look.get("pitch_75_deg", 8.0) + rot["twist_deg"] * (rr - 0.75)


def _chord(rot, rr):
    look = rot["look"]
    if "chords" in look:  # [r/R, chord/c] pairs
        a = np.asarray(look["chords"], float)
        return rot["chord"] * np.interp(rr, a[:, 0], a[:, 1])
    return np.full_like(rr, rot["chord"])


def blade(rot):
    """One blade in the rotor's canonical frame (span along +y, the rotor turning ccw about
    +z): (vertices, triangles, is-tip per triangle). Airfoil sections lofted along the span,
    each pitched about its quarter chord, the span coned up by the precone, the quarter-chord
    line swept aft past `tip.from`; closed at the root and at a rounded tip."""
    look = rot["look"]
    R = rot["radius"]
    foil = af.get(look.get("airfoil", "naca0012"))
    root = look.get("root", 0.15)
    tip_paint = look.get("tip_fraction", 0.04)
    sweep = look.get("tip", {})
    s_from, s_deg = sweep.get("from", 1.0), sweep.get("sweep_deg", 0.0)
    n = int(look.get("sections", 28))
    rr = np.concatenate([np.linspace(root, 0.985, n), [0.995, 1.0]])
    xc = 0.5 * (1.0 - np.cos(np.linspace(0.0, math.pi, 25)))  # chordwise, dense at the edges
    # the section loop: the upper surface from the trailing edge to the leading edge, the lower back
    sx = np.concatenate([xc[::-1], xc[1:-1]])
    sz = np.concatenate([foil.upper(xc[::-1]), foil.lower(xc[1:-1])])
    chords = _chord(rot, rr)
    chords[-2:] *= [0.8, 0.4]  # the tip rounded off
    pitch = np.radians(_pitch_deg(rot, rr))
    beta = math.radians(rot["precone_deg"])
    loops = []
    for r, c, th in zip(rr, chords, pitch):
        x = (sx - 0.25) * c
        z = sz * c
        # pitched nose up about the quarter chord: the leading edge (x < 0) rises
        xp, zp = x * math.cos(th) + z * math.sin(th), z * math.cos(th) - x * math.sin(th)
        xp = xp + max(0.0, r - s_from) * R * math.tan(math.radians(s_deg))
        y = np.full_like(xp, r * R)
        # coned: the span tilts up about the hub
        yc, zc = y * math.cos(beta) - zp * math.sin(beta), y * math.sin(beta) + zp * math.cos(beta)
        loops.append(np.stack([xp, yc, zc], axis=1))
    m = len(sx)
    V = np.vstack(loops)
    tris, tips = [], []
    for j in range(len(loops) - 1):
        for i in range(m):
            a, b = j * m + i, j * m + (i + 1) % m
            c, d = a + m, b + m
            tris += [(a, c, b), (b, c, d)]  # outward: the loop runs trailing edge, upper surface, leading edge
            tips += [rr[j] >= 1.0 - tip_paint] * 2
    # the caps: a fan about each end loop's centre
    for j, root_end in ((0, True), (len(loops) - 1, False)):
        centre = len(V)
        V = np.vstack([V, loops[j].mean(axis=0)])
        for i in range(m):
            a, b = j * m + i, j * m + (i + 1) % m
            tris.append((centre, a, b) if root_end else (centre, b, a))
            tips.append(not root_end)
    return V, np.array(tris), np.array(tips)


def rotor_mesh(rot):
    """Every blade of the rotor in the design frame, about its hub: (vertices, triangles,
    tip flags)."""
    V0, T0, tips = blade(rot)
    if rot["sense"] < 0:  # turning cw about its thrust: the blade mirrored, its leading edge the other way
        V0 = V0 * [-1.0, 1.0, 1.0]
        T0 = T0[:, ::-1]
    F = _frame(rot)
    Vs, Ts, tip = [], [], []
    n = 0
    for k in range(rot["blades"]):
        a = 2.0 * math.pi * k / rot["blades"]
        Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        Vs.append(rot["hub"] + (V0 @ Rz.T) @ F.T)
        Ts.append(T0 + n)
        tip.append(tips)
        n += len(V0)
    return np.vstack(Vs), np.vstack(Ts), np.concatenate(tip)


def disc_mesh(rot, n=72):
    """The blur: a flat annulus in the rotor's plane from the hub's radius to the tips."""
    F = _frame(rot)
    r0 = rot["look"].get("hub_radius", 0.08 * rot["radius"])
    th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    ring = np.stack([np.cos(th), np.sin(th), np.zeros(n)], axis=1)
    V = rot["hub"] + np.vstack([ring * r0, ring * rot["radius"]]) @ F.T
    T = []
    for i in range(n):
        j = (i + 1) % n
        T += [(i, n + i, j), (j, n + i, n + j)]
    return V, np.array(T)


def hub_scene(rot):
    """The rotor's hub - what turns with it at its middle - as meshkit primitives (design
    frame): a helicopter's hub, its grips, a stabilizer bar or a vibration absorber, or a
    propeller's spinner."""
    look = rot["look"]
    F = _frame(rot)
    hub, t = rot["hub"], F[:, 2]
    R = rot["radius"]
    style = look.get("hub", "propeller")
    nodes = []
    size = look.get("hub_radius", 0.08 * R)
    root = look.get("root", 0.15) * R

    def along(k, f):  # blade k's span direction, times f
        a = 2.0 * math.pi * k / rot["blades"]
        return F @ np.array([-math.sin(a), math.cos(a), 0.0]) * f

    if style == "propeller":
        nodes.append({"prim": "revolve", "material": sh.METAL, "origin": (hub - t * 0.5 * size).tolist(), "axis": t.tolist(),
                      "s": [0.0, 0.4 * size, 1.0 * size, 1.5 * size], "r": [size, size, 0.7 * size, 0.0]})
        for k in range(rot["blades"]):
            nodes.append({"prim": "capsule", "material": sh.DARK, "a": hub.tolist(), "b": (hub + along(k, root * 1.05)).tolist(),
                          "r": 0.35 * size})
    else:
        grip = look.get("grip_radius", 0.09 * rot["chord"])
        for k in range(rot["blades"]):
            nodes.append({"prim": "capsule", "material": sh.METAL, "a": hub.tolist(), "b": (hub + along(k, root * 1.03)).tolist(),
                          "r": grip})
        nodes.append({"prim": "cylinder", "material": sh.METAL, "a": (hub - t * 0.7 * size).tolist(),
                      "b": (hub + t * 0.7 * size).tolist(), "r": size, "round": 0.2 * size})
        if "bar" in look:  # a stabilizer bar across the blades, weights at its tips (Bell's)
            b = look["bar"]
            half, below = 0.5 * b["span_m"], b.get("below_m", 0.25)
            ax = F @ np.array([1.0, 0.0, 0.0])  # across blade 0, in the plane
            c = hub - t * below
            nodes.append({"prim": "capsule", "material": sh.METAL, "a": (c - ax * half).tolist(), "b": (c + ax * half).tolist(),
                          "r": b.get("rod_radius_m", 0.02)})
            for s in (-1.0, 1.0):
                w = b.get("weight_length_m", 0.35)
                nodes.append({"prim": "capsule", "material": sh.DARK, "a": (c + ax * s * (half - w)).tolist(),
                              "b": (c + ax * s * half).tolist(), "r": b.get("weight_radius_m", 0.045)})
            nodes.append({"prim": "cylinder", "material": sh.METAL, "a": (c - t * 0.06).tolist(), "b": (hub - t * 0.5 * size).tolist(),
                          "r": 0.45 * size})
        if "absorber" in look:  # a bifilar vibration absorber above the hub (Sikorsky's): arms between the blades
            b = look["absorber"]
            c = hub + t * b.get("above_m", 0.35)
            nodes.append({"prim": "cylinder", "material": sh.METAL, "a": hub.tolist(), "b": c.tolist(), "r": 0.45 * size})
            for k in range(rot["blades"]):
                a = 2.0 * math.pi * (k + 0.5) / rot["blades"]
                d = F @ np.array([-math.sin(a), math.cos(a), 0.0])
                tip = c + d * b["arm_m"]
                nodes.append({"prim": "capsule", "material": sh.METAL, "a": c.tolist(), "b": tip.tolist(), "r": b.get("arm_radius_m", 0.04)})
                nodes.append({"prim": "ellipsoid", "material": sh.DARK, "centre": tip.tolist(),
                              "axes": [d.tolist(), np.cross(t, d).tolist(), t.tolist()],
                              "radii": [b["weight_m"], b["weight_m"], 0.7 * b["weight_m"]]})
    cell = float(np.clip(size / 12.0, 0.0002, 0.02))
    return {"cell": cell, "error": 0.1 * cell, "safety": 3.0, "sharp_deg": 50.0, "max_triangles": 12000,
            "root": {"op": "union", "k": 0.5 * cell, "children": nodes}}


def _rotor_node(B, rot, origin, mats):
    """The rotor's node - its hub - and in it the blades and the blur disc (glTF frame)."""
    pivot = m3._to_gltf(rot["hub"], origin)
    q = m3._quat_from_x(m3._gltf_direction(rot["spin"]))
    Rm = m3._quat_matrix(q)

    def local(v):
        return (m3._to_gltf(v, origin) - pivot) @ Rm

    look = rot["look"]
    hub = meshkit.build(hub_scene(rot))
    hub_parts = [(local(hub["positions"]), hub["triangles"][hub["materials"] == k], mats[int(k)],
                  m3._to_gltf(hub["normals"], np.zeros(3)) @ Rm) for k in np.unique(hub["materials"])]
    V, T, tips = rotor_mesh(rot)
    colour = _rgb(look.get("colour"), (0.12, 0.12, 0.13))
    blade_m = B.material(rot["label"] + " blades", colour, 0.25, 0.55)
    tip_m = B.material(rot["label"] + " tips", _rgb(look.get("tip_colour"), colour), 0.25, 0.55)
    blades = B.mesh(rot["label"] + " blades", [(local(V), T[~tips], blade_m), (local(V), T[tips], tip_m)])
    DV, DT = disc_mesh(rot)
    disc_m = B.material(rot["label"] + " disc", _rgb(look.get("disc_colour"), (0.16, 0.16, 0.17)), 0.0, 0.9,
                        alpha=float(look.get("disc_alpha", 0.3)))
    disc = B.mesh(rot["label"] + " disc", [(local(DV), DT, disc_m)])
    e, rpm = rot["engine"], rot["blur_rpm"]
    kids = [B.node("fsim:blades:%d:%.0f" % (e, rpm), blades, translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0, 1.0]),
            B.node("fsim:disc:%d:%.0f" % (e, rpm), disc, translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0, 1.0])]
    node = B.node("fsim:propeller:%d" % e, B.mesh(rot["label"] + " hub", hub_parts), translation=pivot, rotation=q, children=kids)
    return node, {"label": rot["label"], "engine": e, "blur_rpm": rpm, "hub_triangles": int(len(hub["triangles"])),
                  "hub_components": int(hub["components"]), "hub_boundary_edges": int(hub["boundary_edges"]),
                  "blade_triangles": int(len(T))}


def origin_of(spec):
    """The c.g. in the design frame (m): the glTF's origin."""
    if spec["aircraft"]["kind"] == "helicopter":
        return np.asarray(spec["mass"]["cg_in"], float) * IN
    return np.zeros(3)


def write(spec, path, design_path=None):
    """The .glb (and its manifest) at path; returns the report: the airframe's and the
    rotors' meshes, what the checks read."""
    shape = Aircraft.shape(spec, design_path)
    origin = origin_of(spec)
    B = m3._Builder()
    top = []
    report = {}
    m3._solid_parts(shape, B, top, origin, report)
    mats = m3._paints(shape, B)
    report["rotors"] = []
    for rot in rotors(spec):
        node, info = _rotor_node(B, rot, origin, mats)
        top.append(node)
        report["rotors"].append(info)
    m3.finish(B, top, spec["aircraft"]["name"], path)
    return report


# -- the checks -------------------------------------------------------------------------------------

def _check(name, value, unit, expected, ok, note=""):
    return {"name": name, "value": value if isinstance(value, str) else float(value), "unit": unit, "expected": expected,
            "status": "pass" if ok else "fail", "note": note}


def _glb_nodes(path):
    """The .glb's nodes (its JSON chunk)."""
    import json
    import struct
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[12:16])[0]
    return json.loads(data[20:20 + n])["nodes"]


def _quat_rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * np.asarray(v, float))


def checks(spec, report, path):
    """The model against its design: closed and in one piece, as long as the three-view says,
    the rotors where and as the flight model has them, the ground contacts on the drawn gear."""
    out = []
    a = report["airframe"]
    closed = a["boundary_edges"] == 0 and a["nonmanifold_edges"] == 0 and a["misoriented_edges"] == 0
    out.append(_check("airframe closed", a["boundary_edges"] + a["nonmanifold_edges"] + a["misoriented_edges"], "edges", "0", closed,
                      "open, pinched or misturned edges"))
    out.append(_check("airframe in one piece", a["components"], "", "1", a["components"] == 1))
    for p in report.get("pieces", []):
        out.append(_check("%s closed" % p["label"], p["boundary_edges"], "edges", "0", p["boundary_edges"] == 0))
    for r in report["rotors"]:
        out.append(_check("%s hub closed" % r["label"], r["hub_boundary_edges"], "edges", "0", r["hub_boundary_edges"] == 0))
    dims = spec.get("dimensions", {})
    if "length_m" in dims:
        lo, hi = (np.asarray(b, float) for b in a["bounds"])
        length = float(hi[0] - lo[0])
        out.append(_check("length", length, "m", "%.3f +- 3 %%" % dims["length_m"], abs(length / dims["length_m"] - 1.0) <= 0.03))
    rots = rotors(spec)
    keys = ("main_rotor_diameter_m", "tail_rotor_diameter_m") if spec["aircraft"]["kind"] == "helicopter" else \
        ("rotor_diameter_m",) * len(rots)
    for rot, key in zip(rots, keys):
        if key in dims:
            d = 2.0 * rot["radius"]
            out.append(_check("%s diameter" % rot["label"], d, "m", "%.4f +- 3 %%" % dims[key], abs(d / dims[key] - 1.0) <= 0.03))
    # the bindings: each rotor's node at its hub, its x axis along the shaft the way it turns -
    # from the flight data and the glTF frame's definition (x = -y, y = z, z = -x of the design
    # frame; the origin the c.g.), not from the code that wrote them
    cg = origin_of(spec)
    nodes = _glb_nodes(path)
    by_name = {n["name"]: n for n in nodes}
    for rot in rots:
        n = by_name.get("fsim:propeller:%d" % rot["engine"])
        if n is None:
            out.append(_check("%s node" % rot["label"], "missing", "", "fsim:propeller:%d" % rot["engine"], False))
            continue
        d = rot["hub"] - cg
        want_t = np.array([-d[1], d[2], -d[0]])
        s = rot["sense"] * _unit(rot["thrust"])
        want_x = np.array([-s[1], s[2], -s[0]])
        got_t = np.asarray(n.get("translation", [0.0, 0.0, 0.0]), float)
        got_x = _quat_rotate(np.asarray(n.get("rotation", [0.0, 0.0, 0.0, 1.0]), float), np.array([1.0, 0.0, 0.0]))
        err = float(np.linalg.norm(got_t - want_t))
        ang = math.degrees(math.acos(float(np.clip(np.dot(got_x, want_x), -1.0, 1.0))))
        off = float(rot["look"].get("offset_m", 0.0))
        out.append(_check("%s at its hub" % rot["label"], err, "m", "0 +- 0.001", err <= 0.001,
                          "the flight model's hub, from the c.g." if off == 0.0 else
                          "the flight model's hub moved %.3f m along its shaft, from the c.g." % off))
        # wherever it is drawn, on the flight model's line of thrust: the node (back in the design
        # frame) off the line through the flight data's hub along the thrust by no more than 1 mm
        p = cg + np.array([-got_t[2], -got_t[0], got_t[1]])
        v = p - rot["shaft_hub"]
        miss = float(np.linalg.norm(v - np.dot(v, _unit(rot["thrust"])) * _unit(rot["thrust"])))
        out.append(_check("%s on its shaft" % rot["label"], miss, "m", "0 +- 0.001", miss <= 0.001,
                          "the node's distance from the flight model's line of thrust"))
        out.append(_check("%s turns about its shaft" % rot["label"], ang, "deg", "0 +- 0.1", ang <= 0.1,
                          "the node's x axis: the thrust's, the way the rotor turns"))
        kids = [nodes[k]["name"] for k in n.get("children", [])]
        blur = all(any(k.startswith(p % rot["engine"]) for k in kids) for p in ("fsim:blades:%d:", "fsim:disc:%d:"))
        out.append(_check("%s blades and blur disc" % rot["label"], "yes" if blur else "no", "", "yes", blur))
    # the ground contacts on the drawn skids (a helicopter's) at their bottoms
    ground = spec.get("ground", {})
    if ground.get("kind") == "skid":
        skids = [b for b in spec.get("body", []) if b.get("name") == "skid"]
        worst = 0.0
        for c in ground["contacts_in"]:
            p = np.asarray(c, float) * IN
            best = math.inf
            for b in skids:
                st = b["stations"]
                xs = [r["x"] for r in st]
                if not (xs[0] <= p[0] <= xs[-1]):
                    continue
                bottom = float(np.interp(p[0], xs, [r["bottom"] for r in st]))
                y = float(np.interp(p[0], xs, [r.get("y", 0.0) for r in st]))
                best = min(best, math.hypot(abs(p[1]) - abs(y), p[2] - bottom))
            worst = max(worst, best)
        out.append(_check("ground contacts on the skids", worst, "m", "0 +- 0.02", worst <= 0.02,
                          "the flight model's contact points against the drawn skids' bottoms"))
    elif ground.get("kind") == "wheels":
        # the wheels: each contact where a drawn leg's tyre meets the ground (its [[gear]] position),
        # in the order JSBSim lists the units, which is the order the viewer's gear joints count
        from ..shape import gear as sg
        legs = sg.legs(Aircraft.shape(spec))
        pts = [np.asarray(c, float) * IN for c in ground["contacts_in"]]
        worst = max((float(np.linalg.norm(p - leg.axle + np.array([0.0, 0.0, leg.r]))) for p, leg in zip(pts, legs)),
                    default=math.inf)
        ok = len(legs) == len(pts) and worst <= 0.02
        out.append(_check("ground contacts on the wheels", worst if len(legs) == len(pts) else len(legs), "m",
                          "0 +- 0.02", ok, "the flight model's contact points, in order, against the drawn tyres' bottoms"))
    elif spec["aircraft"]["kind"] == "multirotor" and "leg_height_m" in ground:
        # the feet: each point the flight model stands on (multi.feet: the drawn feet, or under the
        # rotors; leg_height below the c.g.) on the drawn airframe's skin - its signed distance there
        from . import multi
        from ..shape import meshkit as mk
        pts = np.array([[-x, -y, -ground["leg_height_m"]] for x, y in multi.feet(spec)])
        d, _ = mk.evaluate(sh.airframe(Aircraft.shape(spec)), pts)
        worst = float(np.max(np.abs(d)))
        out.append(_check("ground contacts on the feet", worst, "m", "0 +- 0.002", worst <= 0.002,
                          "the airframe's signed distance at the flight model's contact points"))
    return out
