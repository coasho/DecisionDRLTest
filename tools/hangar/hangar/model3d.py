"""The design as a glTF 2.0 binary (.glb) for the platform's viewer.

Written with the numbers the viewer's manifest describes (forward -y, up +z
after its Y-up to Z-up import, like the Cesium model): glTF x = -y (left),
y = z (up), z = -x (forward), a proper rotation of the design frame, with the
origin at the centre of gravity - the point the simulation reports. Each
component is its own node; propellers spin with a glTF animation, which the
viewer plays.

Every control surface is a node of its own that the viewer turns with the
vehicle's deflection (docs/sdk/viewer.md, "Moving control surfaces"): named
fsim:<channel>[:<gain>] - aileron, elevator, rudder or flaps - with its
origin on the hinge line and its local x axis along the hinge, turned so
that a positive rotation about x is the deflection JSBSim's channel means by
a positive value (aileron: the left one trailing edge down). A surface that
several channels move - a stabilator that also rolls, a flaperon - is
fsim:<channel>[:<gain>]+<channel>[:<gain>]..., turned by the sum; an
all-moving surface turns about its spindle.
"""
import json
import math
import struct

import numpy as np

from .aero.vlm import channel_gain
from .geometry import airfoil as af
from .report.render import COLOURS, CONTROL_COLOURS

CHANNEL_NODE = {"aileron": "aileron", "elevator": "elevator", "rudder": "rudder", "flap": "flaps"}


def _to_gltf(p, origin):
    q = np.asarray(p, float) - origin
    return np.stack([-q[..., 1], q[..., 2], -q[..., 0]], axis=-1)


def _normals(v, t):
    n = np.zeros_like(v)
    fn = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    for k in range(3):
        np.add.at(n, t[:, k], fn)
    ln = np.linalg.norm(n, axis=1)
    return n / np.maximum(ln, 1e-12)[:, None]


def _cylinder(centre, axis, radius, length, n=16):
    """A closed cylinder along axis (unit), centred."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    a = np.cross(axis, [0.0, 0.0, 1.0] if abs(axis[2]) < 0.9 else [1.0, 0.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(axis, a)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.cos(th)[:, None] * a + np.sin(th)[:, None] * b
    v = np.vstack([centre - 0.5 * length * axis + radius * ring, centre + 0.5 * length * axis + radius * ring,
                   [centre - 0.5 * length * axis, centre + 0.5 * length * axis]])
    t = []
    for i in range(n):
        j = (i + 1) % n
        t += [(i, j, n + i), (j, n + j, n + i), (2 * n, j, i), (2 * n + 1, n + i, n + j)]
    return v, np.array(t)


def _box(p0, p1, width, thick):
    """A thin box from p0 to p1 (a strut or a blade)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = np.linalg.norm(d)
    ax = d / max(L, 1e-9)
    a = np.cross(ax, [0.0, 0.0, 1.0] if abs(ax[2]) < 0.9 else [1.0, 0.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(ax, a)
    corners = []
    for s in (0.0, 1.0):
        for u, w in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            corners.append(p0 + s * d + 0.5 * width * u * a + 0.5 * thick * w * b)
    v = np.array(corners)
    q = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    t = [(a_, b_, c_) for a_, b_, c_, d_ in q] + [(a_, c_, d_) for a_, b_, c_, d_ in q]
    return v, np.array(t)


class _Builder:
    def __init__(self):
        self.bin = bytearray()
        self.accessors, self.views, self.meshes, self.nodes, self.materials = [], [], [], [], []
        self._mat = {}

    def material(self, name, rgb, metallic=0.1, rough=0.6):
        if name not in self._mat:
            self._mat[name] = len(self.materials)
            self.materials.append({"name": name, "doubleSided": True,
                                   "pbrMetallicRoughness": {"baseColorFactor": [float(c) for c in rgb] + [1.0],
                                                            "metallicFactor": metallic, "roughnessFactor": rough}})
        return self._mat[name]

    def _view(self, data, target=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        off = len(self.bin)
        self.bin += data
        view = {"buffer": 0, "byteOffset": off, "byteLength": len(data)}
        if target:
            view["target"] = target
        self.views.append(view)
        return len(self.views) - 1

    def accessor(self, arr, kind):
        arr = np.ascontiguousarray(arr)
        if kind == "index":
            data = arr.astype(np.uint32).tobytes()
            acc = {"bufferView": self._view(data, 34963), "componentType": 5125, "count": int(arr.size), "type": "SCALAR"}
        elif kind == "vec3":
            a = arr.astype(np.float32)
            acc = {"bufferView": self._view(a.tobytes(), 34962), "componentType": 5126, "count": int(len(a)), "type": "VEC3",
                   "min": a.min(axis=0).tolist(), "max": a.max(axis=0).tolist()}
        elif kind == "vec4":
            a = arr.astype(np.float32)
            acc = {"bufferView": self._view(a.tobytes()), "componentType": 5126, "count": int(len(a)), "type": "VEC4"}
        else:  # scalar floats (animation times)
            a = arr.astype(np.float32)
            acc = {"bufferView": self._view(a.tobytes()), "componentType": 5126, "count": int(len(a)), "type": "SCALAR",
                   "min": [float(a.min())], "max": [float(a.max())]}
        self.accessors.append(acc)
        return len(self.accessors) - 1

    def mesh(self, name, parts):
        """parts: [(vertices (gltf frame), triangles, material index)]."""
        prims = []
        for v, t, mat in parts:
            if len(t) == 0:
                continue
            used = np.unique(t)
            remap = np.full(len(v), -1)
            remap[used] = np.arange(len(used))
            vv, tt = v[used], remap[t]
            prims.append({"attributes": {"POSITION": self.accessor(vv, "vec3"), "NORMAL": self.accessor(_normals(vv, tt), "vec3")},
                          "indices": self.accessor(tt.reshape(-1), "index"), "material": mat})
        self.meshes.append({"name": name, "primitives": prims})
        return len(self.meshes) - 1

    def node(self, name, mesh=None, translation=None, children=None, rotation=None):
        n = {"name": name}
        if mesh is not None:
            n["mesh"] = mesh
        if translation is not None:
            n["translation"] = [float(x) for x in translation]
        if rotation is not None:
            n["rotation"] = [float(x) for x in rotation]
        if children:
            n["children"] = children
        self.nodes.append(n)
        return len(self.nodes) - 1


def _gltf_direction(d):
    d = np.asarray(d, float)
    return np.array([-d[1], d[2], -d[0]])


def _quat_from_x(a):
    """The shortest rotation taking +x to the unit vector a, as glTF's
    (x, y, z, w)."""
    d = float(a[0])
    if d < -0.999999:
        return np.array([0.0, 0.0, 1.0, 0.0])
    q = np.array([0.0, -a[2], a[1], 1.0 + d])  # (x cross a, 1 + x.a)
    return q / np.linalg.norm(q)


def _quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def display_skin(surface, n_span=48, n_chord=28):
    """The skin for drawing with moving control surfaces: the fixed part,
    cut away behind every hinge, and each control surface as a closed
    piece of its own, both halves of a mirrored surface.

    Every ring that carries a hinge has its chordwise stations stretched
    so one of them lies on the hinge, so the cut is exact. The fixed part
    and the piece both get a face along the hinge line and a wall at each
    end of the cut, so a deflected surface shows no hollow wing.

    Returns (vertices, triangles) of the fixed part and a list of pieces:
    dicts with the control index (1-based), side (+1 the defined half, -1
    its mirror), vertices, triangles, the hinge line (two points, root to
    tip, on the camber line), a trailing-edge point, the centre and the
    upper-surface normal u."""
    xs0 = af.cosine_spacing(n_chord + 1)
    n = len(xs0)
    m = 2 * n - 1
    # each control's hinge lands on one chordwise station, the same in every
    # ring; an all-moving control's "hinge" is the leading edge (station 0)
    station = [0 if c.all_moving else int(np.argmin(np.abs(xs0[1:-1] - (1 - 0.5 * (c.cf0 + c.cf1))))) + 1
               for c in surface.controls]
    etas = np.unique(np.concatenate([np.linspace(0, 1, n_span + 1), surface.breakpoints()]))
    rings = []  # (eta, k): k the control whose hinge the ring carries, 0 none
    for i, e in enumerate(etas):
        inside = [k for k, c in enumerate(surface.controls, 1) if c.eta0 + 1e-9 < e < c.eta1 - 1e-9]
        if inside:
            rings.append((e, inside[0]))
            continue
        ends = [k for k, c in enumerate(surface.controls, 1) if abs(c.eta1 - e) <= 1e-9]
        starts = [k for k, c in enumerate(surface.controls, 1) if abs(c.eta0 - e) <= 1e-9]
        seq = [ends[0] if ends else 0, starts[0] if starts else 0]
        if i == 0:
            seq = seq[1:]
        elif i == len(etas) - 1 or seq[0] == seq[1]:
            seq = seq[:1]
        rings += [(e, k) for k in seq]

    def contour(e, k):
        le, ch, tw, foil = surface.station(e)
        c, u = surface.frame(e)
        x = xs0
        if k and station[k - 1] > 0:
            j = station[k - 1]
            h = 1 - surface.controls[k - 1].chord_fraction(e)
            x = np.where(xs0 <= xs0[j], xs0 * h / xs0[j], h + (xs0 - xs0[j]) * (1 - h) / (1 - xs0[j]))
        xx = np.concatenate([x[::-1], x[1:]])
        yy = np.concatenate([foil.upper(x)[::-1], foil.lower(x)[1:]])
        return le + np.outer(xx * ch, c) + np.outer(yy * ch, u)

    pts = [contour(e, k) for e, k in rings]

    def hinge_index(k):
        j = station[k - 1]
        return n - 1 - j, n - 1 + j  # upper, lower contour index

    fixed = _MeshBuilder()
    pieces = {}
    for r in range(len(rings) - 1):
        (e0, k0), (e1, k1) = rings[r], rings[r + 1]
        if e1 - e0 < 1e-12:
            continue
        a, b = pts[r], pts[r + 1]
        if k0 and k0 == k1:
            iu, il = hinge_index(k0)
            piece = pieces.setdefault(k0, {"mesh": _MeshBuilder(), "rings": []})
            if not piece["rings"]:
                piece["rings"].append(r)
            piece["rings"].append(r + 1)
            if iu == il:   # all-moving: the whole section turns
                piece["mesh"].band(r, a, r + 1, b, range(m - 1))
                continue
            fixed.band(r, a, r + 1, b, range(iu, il))
            piece["mesh"].band(r, a, r + 1, b, list(range(0, iu)) + list(range(il, m - 1)))
            for mesh in (fixed, piece["mesh"]):
                mesh.quad(a[iu], b[iu], b[il], a[il])
        else:
            fixed.band(r, a, r + 1, b, range(m - 1))
    # caps and walls
    for k, piece in pieces.items():
        iu, il = hinge_index(k)
        aft = list(range(0, iu + 1)) + list(range(il, m))
        for r in (piece["rings"][0], piece["rings"][-1]):
            piece["mesh"].fan(pts[r][aft])
            e = rings[r][0]
            if 1e-9 < e < 1 - 1e-9:  # a wall where the cut ends inside the span
                fixed.fan(pts[r][aft])
    for r in ([0] if not surface.mirror else []) + [len(rings) - 1]:  # a mirrored root is inside
        k = rings[r][1]
        if k:
            iu, il = hinge_index(k)
            if il > iu:
                fixed.fan(pts[r][iu:il + 1])
        else:
            fixed.fan(pts[r])
    out = []
    for k, piece in pieces.items():
        v, t = piece["mesh"].arrays()
        first, last = pts[piece["rings"][0]], pts[piece["rings"][-1]]
        mid = pts[piece["rings"][len(piece["rings"]) // 2]]
        iu, il = hinge_index(k)
        e_mid = rings[piece["rings"][len(piece["rings"]) // 2]][0]
        ctrl = surface.controls[k - 1]
        if ctrl.all_moving:
            # the spindle: the pivot chord fraction of the first and last rings
            ends = []
            for r in (piece["rings"][0], piece["rings"][-1]):
                le, ch, _, _ = surface.station(rings[r][0])
                c, _ = surface.frame(rings[r][0])
                ends.append(le + ctrl.pivot * ch * c)
            hinge_line = (ends[0], ends[1])
        else:
            hinge_line = (0.5 * (first[iu] + first[il]), 0.5 * (last[iu] + last[il]))
        p = {"control": k, "side": 1, "vertices": v, "triangles": t,
             "hinge": hinge_line,
             "te": 0.5 * (mid[0] + mid[m - 1]), "centre": v.mean(axis=0), "u": surface.frame(e_mid)[1]}
        out.append(p)
        if surface.mirror:
            flip = np.array([1.0, -1.0, 1.0])
            out.append({"control": k, "side": -1, "vertices": v * flip, "triangles": t[:, ::-1],
                        "hinge": (p["hinge"][0] * flip, p["hinge"][1] * flip), "te": p["te"] * flip,
                        "centre": p["centre"] * flip, "u": p["u"] * flip})
    v, t = fixed.arrays()
    if surface.mirror:
        v, t = np.vstack([v, v * np.array([1.0, -1.0, 1.0])]), np.vstack([t, t[:, ::-1] + len(v)])
    return v, t, out


class _MeshBuilder:
    """Triangles with shared vertices along the skin (smooth shading across
    the span) and vertices of their own for flat faces (caps, hinge faces)."""

    def __init__(self):
        self.v, self.t, self.n = [], [], 0
        self._rings = {}

    def _add(self, verts, tris):
        verts = np.asarray(verts, float)
        self.v.append(verts)
        self.t.append(np.asarray(tris, int).reshape(-1, 3) + self.n)
        self.n += len(verts)

    def _ring(self, key, points):
        if key not in self._rings:
            self._rings[key] = self.n
            self.v.append(np.asarray(points, float))
            self.n += len(points)
        return self._rings[key]

    def band(self, ka, a, kb, b, segments):
        """Two triangles per contour segment j -> j+1 between rings a and b
        (keyed ka, kb: a ring's points are added once)."""
        segments = list(segments)
        if not segments:
            return
        i, k = self._ring(ka, a), self._ring(kb, b)
        tris = []
        for j in segments:
            tris += [(i + j, k + j, i + j + 1), (i + j + 1, k + j, k + j + 1)]
        self.t.append(np.array(tris, int))

    def quad(self, p0, p1, p2, p3):
        self._add([p0, p1, p2, p3], [(0, 1, 2), (0, 2, 3)])

    def fan(self, polygon):
        """A closed polygon as a fan around its centroid."""
        polygon = np.asarray(polygon, float)
        k = len(polygon)
        self._add(np.vstack([polygon, polygon.mean(axis=0)]), [(k, j, (j + 1) % k) for j in range(k)])

    def arrays(self):
        if not self.v:
            return np.zeros((0, 3)), np.zeros((0, 3), int)
        return np.vstack(self.v), np.vstack(self.t)


def hinge(surface, piece):
    """A control piece's hinge in the design frame: (point, unit axis, gain)
    such that turning the piece by gain * delta about the axis (right hand)
    is what JSBSim's channel means by +delta - the convention the
    aerodynamic model uses (aero.vlm.channel_gain) - with gain >= 0."""
    ctrl = surface.controls[piece["control"] - 1]
    p0, p1 = piece["hinge"]
    axis = (p1 - p0) / np.linalg.norm(p1 - p0)
    # the trailing edge's motion for a positive turn about the axis: towards -u
    # is a positive local deflection
    if np.dot(np.cross(axis, piece["te"] - p0), -piece["u"]) < 0:
        axis = -axis
    g = channel_gain(ctrl.channel, ctrl, piece["u"], piece["centre"])
    sign = -1.0 if g < 0 else 1.0
    if g < 0:
        axis, g = -axis, -g
    mixed = {ch: sign * channel_gain(ch, ctrl, piece["u"], piece["centre"]) for ch in ctrl.mix}
    return p0, axis, float(g), mixed


def control_nodes(aircraft, origin):
    """Every control piece as the node the glb carries: name, the glTF
    translation and rotation, and its vertices in the node's own frame."""
    out = []
    for s in aircraft.surfaces:
        _, _, pieces = display_skin(s)
        for piece in pieces:
            ctrl = s.controls[piece["control"] - 1]
            p0, axis, g, mixed = hinge(s, piece)
            q = _quat_from_x(_gltf_direction(axis))
            pivot = _to_gltf(p0, origin)
            name = "fsim:" + CHANNEL_NODE[ctrl.channel] + ("" if abs(g - 1.0) < 1e-9 else ":%.6g" % g)
            name += "".join("+%s:%.6g" % (CHANNEL_NODE[ch], gm) for ch, gm in sorted(mixed.items()) if abs(gm) > 1e-9)
            y = piece["centre"][1]
            side = "" if abs(y) < 1e-6 else (" right" if y > 0 else " left")
            out.append({"name": name, "label": "%s %s%s" % (s.name, ctrl.name, side),
                        "channel": ctrl.channel, "translation": pivot, "rotation": q,
                        "vertices": (_to_gltf(piece["vertices"], origin) - pivot) @ _quat_matrix(q),
                        "triangles": piece["triangles"], "design_axis": axis, "gain": g})
    return out


def write_glb(aircraft, path, origin):
    origin = np.asarray(origin, float)
    B = _Builder()
    top = []
    for s in aircraft.surfaces:
        v, t, _ = display_skin(s)
        top.append(B.node(s.name, B.mesh(s.name, [(_to_gltf(v, origin), t, B.material(s.kind, COLOURS.get(s.kind, (0.85, 0.85, 0.88))))])))
    for c in control_nodes(aircraft, origin):
        mat = B.material("control_" + c["channel"], CONTROL_COLOURS.get(c["channel"], (0.9, 0.7, 0.2)))
        top.append(B.node(c["name"], B.mesh(c["label"], [(c["vertices"], c["triangles"], mat)]),
                          translation=c["translation"], rotation=c["rotation"]))
    for b in aircraft.bodies:
        v, t, _ = b.skin(72, 40)
        top.append(B.node(b.name, B.mesh(b.name, [(_to_gltf(v, origin), t, B.material(b.kind, COLOURS.get(b.kind, (0.85, 0.85, 0.88))))])))
    # landing gear: wheels and struts
    dark = B.material("tyre", (0.08, 0.08, 0.09), 0.0, 0.9)
    strut_m = B.material("strut", (0.55, 0.56, 0.60), 0.6, 0.4)
    for gear in aircraft.gear:
        for name, pos in gear.positions():
            r = 0.5 * gear.wheel_diameter
            centre = pos + np.array([0.0, 0.0, r])
            wv, wt = _cylinder(centre, [0.0, 1.0, 0.0], r, gear.wheel_width)
            parts = [(_to_gltf(wv, origin), wt, dark)]
            if gear.attach is not None:
                att = gear.attach.copy()
                if pos[1] < 0 < att[1] or pos[1] > 0 > att[1]:
                    att[1] = -att[1]
                sv, st = _box(centre, att, 0.06, 0.03)
                parts.append((_to_gltf(sv, origin), st, strut_m))
            top.append(B.node(name, B.mesh(name, parts)))
    # propellers: spinner and blades on a node that spins
    blade_m = B.material("propeller", (0.12, 0.12, 0.13), 0.3, 0.5)
    spin_m = B.material("spinner", (0.85, 0.2, 0.15), 0.4, 0.4)
    anims = []
    for e in aircraft.engines:
        if not e.has_propeller:
            continue
        for name, _, prop, _ in e.copies():
            R = 0.5 * e.prop_diameter
            parts = []
            for k in range(e.prop_blades):
                ang = 2 * math.pi * k / e.prop_blades
                tip = np.array([0.0, R * math.cos(ang), R * math.sin(ang)])
                bv, bt = _box(0.08 * tip, tip, 0.08 * R, 0.015 * R)
                parts.append((_to_gltf(bv + prop, prop), bt, blade_m))
            sv, st = _cylinder(prop + np.array([-0.05, 0.0, 0.0]), [1.0, 0.0, 0.0], 0.1 * R, 0.25 * R, 20)
            parts.append((_to_gltf(sv, prop), st, spin_m))
            hub = _to_gltf(prop, origin)
            idx = B.node(name + " propeller", B.mesh(name + " propeller", parts), translation=hub)
            top.append(idx)
            # one turn in 0.2 s, about the propeller axis (gltf +z, forward)
            times = np.array([0.0, 0.05, 0.1, 0.15, 0.2])
            sense = -1.0 if e.prop_sense == "cw" else 1.0
            quats = np.array([[0.0, 0.0, math.sin(sense * a / 2), math.cos(sense * a / 2)] for a in (0, math.pi / 2, math.pi, 1.5 * math.pi, 2 * math.pi)])
            anims.append({"name": name + " spin",
                          "samplers": [{"input": B.accessor(times, "scalar"), "output": B.accessor(quats, "vec4"), "interpolation": "LINEAR"}],
                          "channels": [{"sampler": 0, "target": {"node": idx, "path": "rotation"}}]})
    root = B.node(aircraft.name, children=top)
    doc = {"asset": {"version": "2.0", "generator": "hangar"}, "scene": 0, "scenes": [{"nodes": [root]}],
           "nodes": B.nodes, "meshes": B.meshes, "materials": B.materials, "accessors": B.accessors,
           "bufferViews": B.views, "buffers": [{"byteLength": len(B.bin)}]}
    if anims:
        doc["animations"] = anims
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    while len(B.bin) % 4:
        B.bin.append(0)
    total = 12 + 8 + len(js) + 8 + len(B.bin)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack("<II", len(B.bin), 0x004E4942))
        f.write(bytes(B.bin))
    with open(path + ".manifest", "w", encoding="utf-8") as f:
        f.write("# Generated by hangar: glTF x = -y, y = up, z = forward (nose); origin at the CG.\n"
                "forward -y\nup +z\nscale 1.0\n")
    return path
