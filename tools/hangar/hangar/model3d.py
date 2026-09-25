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
all-moving surface turns about its spindle. A piece its channels can drive
past its own limits - a canard that travels further than the elevons on its
channel, an elevon asked for pitch and roll at once - ends in @<lo>,<hi>, its
stops in degrees.
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
        self.images, self.textures = [], []
        self._mat = {}
        self.livery = None  # the paint (livery.Livery) a skin part's texture coordinates refer to

    def texture(self, jpeg):
        """An image (JPEG bytes) for materials to use; its index."""
        self.images.append({"bufferView": self._view(jpeg), "mimeType": "image/jpeg"})
        self.textures.append({"sampler": 0, "source": len(self.images) - 1})
        return len(self.textures) - 1

    def painted(self, name, texture, metallic=0.2, rough=0.6):
        """A material coloured by a texture."""
        if name not in self._mat:
            self._mat[name] = len(self.materials)
            self.materials.append({"name": name, "doubleSided": True,
                                   "pbrMetallicRoughness": {"baseColorTexture": {"index": texture},
                                                            "baseColorFactor": [1.0, 1.0, 1.0, 1.0],
                                                            "metallicFactor": metallic, "roughnessFactor": rough}})
        return self._mat[name]

    def material(self, name, rgb, metallic=0.1, rough=0.6, emissive=None, alpha=1.0):
        if name not in self._mat:
            self._mat[name] = len(self.materials)
            m = {"name": name, "doubleSided": True,
                 "pbrMetallicRoughness": {"baseColorFactor": [float(c) for c in rgb] + [float(alpha)],
                                          "metallicFactor": metallic, "roughnessFactor": rough}}
            if emissive is not None:
                m["emissiveFactor"] = [float(c) for c in emissive]
            if alpha < 1.0:
                m["alphaMode"] = "BLEND"
            self.materials.append(m)
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
            # 16-bit where the vertices allow: half the index data
            small = arr.size == 0 or int(arr.max()) < 65535
            data = arr.astype(np.uint16 if small else np.uint32).tobytes()
            acc = {"bufferView": self._view(data, 34963), "componentType": 5123 if small else 5125,
                   "count": int(arr.size), "type": "SCALAR"}
        elif kind == "vec3":
            a = arr.astype(np.float32)
            acc = {"bufferView": self._view(a.tobytes(), 34962), "componentType": 5126, "count": int(len(a)), "type": "VEC3",
                   "min": a.min(axis=0).tolist(), "max": a.max(axis=0).tolist()}
        elif kind == "vec2":
            a = arr.astype(np.float32)
            acc = {"bufferView": self._view(a.tobytes(), 34962), "componentType": 5126, "count": int(len(a)), "type": "VEC2"}
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
        """parts: [(vertices (gltf frame), triangles, material index[, normals[, texture coordinates]])]."""
        prims = []
        for part in parts:
            v, t, mat = part[:3]
            if len(t) == 0:
                continue
            used = np.unique(t)
            remap = np.full(len(v), -1)
            remap[used] = np.arange(len(used))
            vv, tt = v[used], remap[t]
            nn = part[3][used] if len(part) > 3 and part[3] is not None else _normals(vv, tt)
            attrs = {"POSITION": self.accessor(vv, "vec3"), "NORMAL": self.accessor(nn, "vec3")}
            if len(part) > 4 and part[4] is not None:
                attrs["TEXCOORD_0"] = self.accessor(part[4][used], "vec2")
            prims.append({"attributes": attrs, "indices": self.accessor(tt.reshape(-1), "index"), "material": mat})
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


def leading_hinge(piece):
    """A leading-edge device's hinge: (point, unit axis, 1, {}), the axis
    turned so that a positive turn moves its leading edge down (towards -u)
    on either side."""
    p0, p1 = piece["hinge"]
    axis = (p1 - p0) / np.linalg.norm(p1 - p0)
    if np.dot(np.cross(axis, piece["te"] - p0), -piece["u"]) < 0:
        axis = -axis
    return p0, axis, 1.0, {}


def stops(aircraft, ctrl, piece):
    """"@lo,hi" (degrees, the node's sense) when the piece's channels can drive
    it past its own limits, else "". The node turns sign * orient times the
    control's own deflection (trailing edge down, or left for a rudder)."""
    lo, hi = aircraft.channel_limits(ctrl.channel)
    g = ctrl.channels[ctrl.channel]
    reach = sorted((g * lo, g * hi))
    if not ctrl.mix and reach[0] >= ctrl.min_deg - 1e-6 and reach[1] <= ctrl.max_deg + 1e-6:
        return ""
    sign = -1.0 if channel_gain(ctrl.channel, ctrl, piece["u"], piece["centre"]) < 0 else 1.0
    orient = np.sign(piece["u"][1 if ctrl.channel == "rudder" else 2]) or 1.0
    if sign * orient > 0:
        return "@%.6g,%.6g" % (ctrl.min_deg, ctrl.max_deg)
    return "@%.6g,%.6g" % (-ctrl.max_deg, -ctrl.min_deg)


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
            name += stops(aircraft, ctrl, piece)
            y = piece["centre"][1]
            side = "" if abs(y) < 1e-6 else (" right" if y > 0 else " left")
            out.append({"name": name, "label": "%s %s%s" % (s.name, ctrl.name, side),
                        "channel": ctrl.channel, "translation": pivot, "rotation": q,
                        "vertices": (_to_gltf(piece["vertices"], origin) - pivot) @ _quat_matrix(q),
                        "triangles": piece["triangles"], "design_axis": axis, "gain": g})
    return out


def node_name(aircraft, surface, ctrl, piece):
    """The glb name of a control piece: its channels and gains, and its stops."""
    _, _, g, mixed = hinge(surface, piece)
    name = "fsim:" + CHANNEL_NODE[ctrl.channel] + ("" if abs(g - 1.0) < 1e-9 else ":%.6g" % g)
    name += "".join("+%s:%.6g" % (CHANNEL_NODE[ch], gm) for ch, gm in sorted(mixed.items()) if abs(gm) > 1e-9)
    return name + stops(aircraft, ctrl, piece)


# the solid model's paint: air-superiority greys, a dark glossy canopy, a
# jet nozzle's heat-darkened metal (dull: bright metal there read as chrome)
PAINT = {"skin": ((0.60, 0.62, 0.65), 0.25, 0.55), "control": ((0.55, 0.57, 0.60), 0.25, 0.55),
         "glass": ((0.07, 0.08, 0.10), 0.9, 0.08), "dark": ((0.04, 0.04, 0.05), 0.0, 0.9),
         "metal": ((0.38, 0.37, 0.36), 0.8, 0.35), "tyre": ((0.08, 0.08, 0.09), 0.0, 0.9),
         "strut": ((0.55, 0.56, 0.60), 0.6, 0.4), "nozzle": ((0.26, 0.245, 0.23), 0.6, 0.62)}


def _paints(aircraft, B):
    """The paints by hangar's material index - the nozzle's only for an
    aircraft with a round nozzle, so a model without one carries no paint it
    never uses (a two-dimensional nozzle is the airframe's metal)."""
    from .shape import airframe as sh
    rounds = any(e.type == "turbofan" and (e.prop_spec or {}).get("shape", "round") == "round" for e in aircraft.engines)
    return {i: B.material(n, *PAINT[n]) for i, n in enumerate(sh.MATERIALS) if rounds or n != "nozzle"}


def _stats(m):
    return {k: m[k] for k in ("boundary_edges", "nonmanifold_edges", "misoriented_edges", "components",
                              "degenerate_triangles", "raw_triangles", "fragments_removed", "volume", "area",
                              "seconds")} | \
        {"triangles": int(len(m["triangles"]))}


def parked_ground(aircraft):
    """The ground the aircraft stands on, parked, in the design frame's x-z
    plane: the line through its foremost and aftmost wheels' ground points
    (each contact a static deflection below it) - (x0, z0, slope), level for
    a tricycle whose wheels stand level, nose-high for a bicycle gear whose
    tail wheel stands higher (the U-2's). None without gear."""
    pts = sorted((float(p[0]), float(p[2]) + g.static_deflection) for g in aircraft.gear for _, p in g.positions())
    if not pts:
        return None
    (x0, z0), (x1, z1) = pts[0], pts[-1]
    return x0, z0, (z1 - z0) / (x1 - x0) if x1 - x0 > 1e-6 else 0.0


def parked_height(aircraft, positions):
    """The height (m) of the points (design frame) over the parked ground, and
    the parked attitude's pitch (deg, nose up); None without gear."""
    ground = parked_ground(aircraft)
    if ground is None:
        return None
    x0, z0, s = ground
    c = 1.0 / np.sqrt(1.0 + s * s)
    h = max(float(np.max((p[:, 2] - z0 - s * (p[:, 0] - x0)) * c)) for p in positions)
    return h, float(np.degrees(np.arctan(s)))


def _solid_parts(aircraft, B, top, origin, report):
    """The airframe as one closed solid and every control piece as a solid of
    its own on its hinge (shape/airframe.py, native/meshkit)."""
    from .shape import airframe as sh
    from .shape import meshkit
    plan, flaps = [], []
    m = meshkit.build(sh.airframe(aircraft, gear=plan, flaps=flaps))
    report["airframe"] = _stats(m)
    report["airframe"]["bounds"] = [m["positions"].min(axis=0).tolist(), m["positions"].max(axis=0).tolist()]
    tops = [m["positions"]]
    mats = _materials(aircraft, B, report["airframe"]["bounds"])
    m = _painted(B, m)
    v, n = _to_gltf(m["positions"], origin), _to_gltf(m["normals"], np.zeros(3))
    parts = [(v, m["triangles"][m["materials"] == k], mats.get(int(k), mats[sh.SKIN]), n, m.get("uv"))
             for k in np.unique(m["materials"])]
    top.append(B.node("airframe", B.mesh("airframe", parts)))
    report["pieces"] = []
    for surf, ctrl, side, scene in sh.control_pieces(aircraft):
        pm = _painted(B, meshkit.build(scene))
        piece = sh.hinge_piece(surf, ctrl, side)
        p0, axis, _, _ = leading_hinge(piece) if piece["leading"] else hinge(surf, piece)
        q = _quat_from_x(_gltf_direction(axis))
        pivot = _to_gltf(p0, origin)
        R = _quat_matrix(q)
        label = "%s %s%s" % (surf.name, ctrl.name, {1: " right", -1: " left"}.get(side, ""))
        pv = (_to_gltf(pm["positions"], origin) - pivot) @ R
        pn = _to_gltf(pm["normals"], np.zeros(3)) @ R
        name = ("fsim:lef@%.6g,%.6g" % (ctrl.min_deg, ctrl.max_deg)
                if piece["leading"] else node_name(aircraft, surf, ctrl, piece))
        top.append(B.node(name, B.mesh(label, [(pv, pm["triangles"], mats[sh.CONTROL], pn, pm.get("uv"))]),
                          translation=pivot, rotation=q))
        report["pieces"].append(dict(_stats(pm), label=label, bounds=[pm["positions"].min(axis=0).tolist(),
                                                                      pm["positions"].max(axis=0).tolist()]))
        tops.append(pm["positions"])
    parked = parked_height(aircraft, tops)
    if parked is not None:
        report["airframe"]["parked_height_m"], report["airframe"]["parked_pitch_deg"] = parked
    # a vectoring two-dimensional nozzle's flaps, cut from the airframe: in the
    # airframe's frame here, gathered on their hinge by _vectoring()
    for name, scene in flaps:
        pm = _painted(B, meshkit.build(scene))
        label = "%s nozzle flaps" % name
        pv, pn = _to_gltf(pm["positions"], origin), _to_gltf(pm["normals"], np.zeros(3))
        top.append(B.node("%s vectoring" % name, B.mesh(label, [(pv, pm["triangles"][pm["materials"] == k],
                                                                  mats.get(int(k), mats[sh.SKIN]), pn, pm.get("uv"))
                                                             for k in np.unique(pm["materials"])])))
        report["pieces"].append(dict(_stats(pm), label=label, bounds=[pm["positions"].min(axis=0).tolist(),
                                                                      pm["positions"].max(axis=0).tolist()]))
    _gear_parts(aircraft, B, top, origin, report, plan, mats)


def _materials(aircraft, B, bounds):
    """The solid model's materials by hangar's material index: the skin and
    the control surfaces painted with the design's livery (paint.toml) when
    PIL is there to draw it, flat grey otherwise; the glass tinted."""
    from . import livery
    from .shape import airframe as sh
    mats = _paints(aircraft, B)
    try:
        import PIL  # noqa: F401
    except ImportError:
        return mats
    B.livery = livery.Livery(aircraft, bounds)
    tex = B.texture(B.livery.image())
    rough = 1.0 - 0.6 * B.livery.gloss
    mats[sh.SKIN] = B.painted("paint", tex, 0.15, rough)
    mats[sh.CONTROL] = B.painted("paint", tex, 0.15, rough)
    mats[sh.GLASS] = B.material("canopy", *B.livery.canopy())
    return mats


def _painted(B, m):
    """The mesh m (a meshkit mesh, design frame) with texture coordinates on
    the livery (m["uv"]; its vertices split where its faces look different
    ways), or m as it is without a livery."""
    if B.livery is None or len(m["triangles"]) == 0:
        return m
    t = m["triangles"]
    v, n, t2, uv = B.livery.split(m["positions"], m["normals"], t)
    out = dict(m)
    out.update(positions=v, normals=n, triangles=t2, uv=uv)
    return out


def _hinged(B, name, label, m, mats, point, axis, origin, inner=None):
    """A node turning about the design-frame axis through point: its origin
    there, its local x along the axis, the mesh in its own frame. inner:
    (name, axis) of a second joint through the same point, nested in it
    (a leg's twist inside its swing)."""
    pivot = _to_gltf(point, origin)
    q = _quat_from_x(_gltf_direction(axis))
    R = _quat_matrix(q)
    if inner is not None:
        local = R.T @ _gltf_direction(inner[1])
        qi = _quat_from_x(local / np.linalg.norm(local))
        R = R @ _quat_matrix(qi)
    v = (_to_gltf(m["positions"], origin) - pivot) @ R
    n = _to_gltf(m["normals"], np.zeros(3)) @ R
    mesh = B.mesh(label, [(v, m["triangles"][m["materials"] == k], mats[int(k)], n, m.get("uv"))
                          for k in np.unique(m["materials"])])
    if inner is None:
        return B.node(name, mesh, translation=pivot, rotation=q)
    return B.node(name, translation=pivot, rotation=q, children=[B.node(inner[0], mesh, rotation=qi)])


def _tree(B, mats, origin, joint, parent=None):
    """A chain of nodes from design-frame joints: joint = {"name", "point",
    "axis", "mesh": (label, meshkit mesh) or None, "children": [joints]} - each
    node's origin at its point, its x along its axis, placed in its parent's
    frame, its mesh in its own."""
    q = _quat_from_x(_gltf_direction(joint["axis"]))
    R, t = _quat_matrix(q), _to_gltf(joint["point"], origin)
    Rp, tp = parent if parent is not None else (np.eye(3), np.zeros(3))
    kids = [_tree(B, mats, origin, c, (R, t)) for c in joint.get("children", [])]
    mesh = None
    if joint.get("mesh") is not None:
        label, m = joint["mesh"]
        v = (_to_gltf(m["positions"], origin) - t) @ R
        n = _to_gltf(m["normals"], np.zeros(3)) @ R
        mesh = B.mesh(label, [(v, m["triangles"][m["materials"] == k], mats[int(k)], n) for k in np.unique(m["materials"])])
    return B.node(joint["name"], mesh, translation=Rp.T @ (t - tp), rotation=_quat_from_matrix(Rp.T @ R), children=kids)


def _gear_parts(aircraft, B, top, origin, report, plan, mats):
    """Each landing gear leg as solids that move as the simulation has them:
    a retractable one turns about its hinge with the gear position
    (fsim:gear:<deg>:1:<f>, from down at 1 to stowed at f) and its doors open
    before it moves (fsim:gear:<deg>:0:<f>); on every leg the oleo slides up
    the strut as the unit compresses (fsim:oleo:<wheel>:<gain>), a steerable
    one turns with the steering (fsim:steer:<wheel>) and the wheels roll
    (fsim:wheel:<wheel>:<radius>) - <wheel> the leg's place among the
    aircraft's wheeled units, as the JSBSim file lists them."""
    from .shape import gear as sg
    from .shape import meshkit
    doors = {p["leg"].name: p for p in plan}
    report["gear"] = []
    moving, panels = [], []
    for k, leg in enumerate(sg.legs(aircraft)):
        leg = doors[leg.name]["leg"] if leg.name in doors else leg  # fitted to its bay
        pieces = {part: meshkit.build(sg.leg_scene(leg, part))
                  for part in ["strut", "oleo"] + (["bogie"] if leg.bogie_turns else []) + leg.wheel_parts()}
        for part, m in pieces.items():
            report["gear"].append(dict(_stats(m), label="%s %s" % (leg.name, part)))
        slide, gain = leg.slide()
        # each axle's wheels roll about it (a bogie's several)
        lower = [{"name": "fsim:wheel:%d:%.6g" % (k, leg.r), "point": p, "axis": [0.0, -1.0, 0.0],
                  "mesh": ("%s %s" % (leg.name, part), pieces[part])} for part, p in zip(leg.wheel_parts(), leg.axle_points())]
        if leg.bogie_turns:  # the bogie turns on its pivot, at the oleo's foot, as the leg swings
            lower = [{"name": "fsim:gear:%.6g:1:%.6g" % (leg.bogie_deg, sg.DOORS), "point": leg.foot(),
                      "axis": [0.0, 1.0, 0.0], "mesh": ("%s bogie" % leg.name, pieces["bogie"]), "children": lower}]
        if leg.steerable:  # about the strut, pointing down: a positive turn steers right
            lower = [{"name": "fsim:steer:%d" % k, "point": leg.axle, "axis": -leg.strut,
                      "mesh": ("%s oleo" % leg.name, pieces["oleo"]), "children": lower}]
            oleo = {"name": "fsim:oleo:%d:%.6g" % (k, gain), "point": leg.foot(), "axis": slide, "children": lower}
        else:
            oleo = {"name": "fsim:oleo:%d:%.6g" % (k, gain), "point": leg.foot(), "axis": slide,
                    "mesh": ("%s oleo" % leg.name, pieces["oleo"]), "children": lower}
        strut = ("%s strut" % leg.name, pieces["strut"])
        if leg.retractable:
            chain = {"name": "fsim:gear:%.6g:1:%.6g" % (leg.twist_deg, sg.DOORS), "point": leg.hinge, "axis": leg.strut,
                     "mesh": strut, "children": [oleo]}
            top.append(_tree(B, mats, origin, {"name": "fsim:gear:%.6g:1:%.6g" % (leg.swing_deg, sg.DOORS),
                                               "point": leg.hinge, "axis": leg.swing_axis, "children": [chain]}))
            report["gear"][-len(pieces)]["protrusion"] = doors[leg.name]["protrusion"]  # on the strut's entry
            report["gear"][-len(pieces)]["allowed"] = doors[leg.name]["allowed"]
            moving.append(leg)
            for d in doors[leg.name]["doors"]:
                dm = _painted(B, meshkit.build(d["scene"]))
                top.append(_hinged(B, "fsim:gear:%.6g:0:%.6g" % (d["deg"], sg.DOORS), d["label"], dm, mats,
                                   d["hinge"], d["axis"], origin))
                report["gear"].append(dict(_stats(dm), label=d["label"]))
                panels.append((d, dm))
        else:
            top.append(_tree(B, mats, origin, {"name": leg.name, "point": leg.hinge, "axis": leg.strut, "mesh": strut,
                                               "children": [oleo]}))
    report["door_clearance"] = door_clearance(moving, panels)


def _surface_samples(m, step=0.012):
    """Points all over a mesh's triangles, about step apart."""
    v, t = m["positions"], m["triangles"]
    out = [v]
    for tri in v[t]:
        n = int(np.ceil(max(np.linalg.norm(tri[1] - tri[0]), np.linalg.norm(tri[2] - tri[0]),
                            np.linalg.norm(tri[2] - tri[1])) / step))
        if n < 2:
            out.append(tri.mean(axis=0)[None])
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = i + j <= n
        a, b = i[keep] / n, j[keep] / n
        out.append(tri[0] + np.outer(a, tri[1] - tri[0]) + np.outer(b, tri[2] - tri[0]))
    return np.vstack(out)


def door_clearance(legs, panels, steps=30):
    """How far each landing gear door stays from each leg (m; negative: into
    it): open, all the way as the leg swings from down to stowed; closed,
    from the stowed leg."""
    from .shape import gear as sg
    out = []
    samples = []
    for d, dm in panels:
        pts = _surface_samples(dm)
        R = sg._rotation(d["axis"], d["deg"])
        samples.append((d["label"], d["hinge"] + (pts - d["hinge"]) @ R.T, pts))
    for leg in legs:
        with leg.probes() as probes:
            for label, opened, closed in samples:
                worst, at = np.inf, 0.0
                for f in np.linspace(0.0, 1.0, steps + 1):
                    dist = leg.distance(probes, opened, f)  # the door in the frame of the leg as built (down)
                    if dist.min() < worst:
                        worst, at = float(dist.min()), float(f)
                shut = leg.distance(probes, closed, 1.0)
                out.append({"door": label, "leg": leg.name, "open_m": worst, "at": at, "closed_m": float(shut.min())})
    return out


def _primitive_parts(aircraft, B, top, origin):
    """The primitive model: each surface and body a closed skin of its own
    (when the mesher is not built)."""
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


def _revolve_mesh(s, r, n=32):
    """A solid of revolution about the local x axis: profile radii r at
    stations s, closed on the axis where r is 0."""
    th = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    rings = [np.stack([np.full(n, si), ri * np.cos(th), ri * np.sin(th)], axis=1) for si, ri in zip(s, r)]
    v = np.vstack(rings + [[[s[0], 0.0, 0.0]], [[s[-1], 0.0, 0.0]]])
    t = []
    for k in range(len(s) - 1):
        for i in range(n):
            j = (i + 1) % n
            a, b, c, d = k * n + i, k * n + j, (k + 1) * n + i, (k + 1) * n + j
            t += [(a, c, b), (b, c, d)]
    c0, c1 = len(s) * n, len(s) * n + 1
    last = (len(s) - 1) * n
    for i in range(n):
        j = (i + 1) % n
        t += [(c0, i, j), (c1, last + j, last + i)]
    return v, np.array(t)


def _engine_joint(i):
    """The engine a part's joint follows: the platform reports the first
    PLATFORM_THROTTLES engines, and each one past them runs on throttle i % 4
    (jsbsim.py), so it shows that engine's state."""
    from .jsbsim import PLATFORM_THROTTLES
    return i % PLATFORM_THROTTLES


def _propellers(aircraft, B, top, origin):
    """Each propeller as a solid - spinner and twisted-in-pitch blades - on a
    node the viewer turns at its engine's rpm as the simulation reports it
    (fsim:propeller:<engine>), its x axis the way it turns (cw seen from
    behind: forward)."""
    from .jsbsim import _engine_units
    from .shape import airframe as sh
    from .shape import meshkit
    mats = _paints(aircraft, B)
    units = [name for _, name in _engine_units(aircraft)]
    for e in aircraft.engines:
        if not e.has_propeller:
            continue
        R = 0.5 * e.prop_diameter
        for name, _, prop, sense in e.copies():
            hub = np.asarray(prop, float)
            nodes = [{"prim": "revolve", "material": sh.METAL, "origin": hub, "axis": [-1.0, 0.0, 0.0],
                      "s": [-0.06 * R, 0.0, 0.12 * R, 0.24 * R, 0.32 * R],
                      "r": [0.11 * R, 0.12 * R, 0.10 * R, 0.06 * R, 0.0]}]
            # it turns about the forward axis (cw seen from behind) or the aft one;
            # each blade's chord leans from the way it moves towards the thrust
            axis = np.array([-1.0, 0.0, 0.0]) if sense == "cw" else np.array([1.0, 0.0, 0.0])
            pitch = np.radians(22.0)
            for k in range(e.prop_blades):
                th = 2.0 * np.pi * k / e.prop_blades
                span = np.array([0.0, np.cos(th), np.sin(th)])
                chord = np.cos(pitch) * np.cross(axis, span) + np.sin(pitch) * np.array([-1.0, 0.0, 0.0])
                nodes.append({"prim": "ellipsoid", "material": sh.TYRE, "centre": hub + span * 0.55 * R,
                              "axes": [span, chord, np.cross(span, chord)], "radii": [0.46 * R, 0.075 * R, 0.013 * R]})
            cell = float(np.clip(R / 90.0, 0.001, 0.01))
            m = meshkit.build({"cell": cell, "error": 0.1 * cell, "safety": 3.0, "sharp_deg": 50.0,
                               "max_triangles": 3000, "root": {"op": "union", "k": 0.02 * R, "children": nodes}})
            top.append(_hinged(B, "fsim:propeller:%d" % _engine_joint(units.index(name)), name + " propeller", m, mats,
                               hub, axis, origin))


def petal_mesher(d):
    """The mesher's settings for a petal of a round nozzle of diameter d (m):
    cells fine enough for its edges to come out straight and the seal on its
    inside (half a wall thick) as a plate, not a row of teeth; its flat
    faces then thinned out to some 2,000 triangles a petal. There are twelve
    to a nozzle: at twice that they came to a quarter of a single-engined
    fighter's triangles, two fifths of a twin's."""
    cell = float(np.clip(d / 260.0, 0.002, 0.006))
    return {"cell": cell, "error": cell, "safety": 3.0, "sharp_deg": 45.0, "max_triangles": 600}


def _nozzle_petals(aircraft, B, top, origin):
    """Each round nozzle's petals, each on a node that opens it as far as
    the engine's nozzle is open (fsim:nozzle:<engine>:<deg wide open>; shut
    at military power, open at idle and with the afterburner lit) under one
    that sets it round the nozzle; one mesh shared by them all. A fixed
    nozzle's (petal_open_deg 0) are plain nodes."""
    from .jsbsim import _engine_units
    from .shape import airframe as sh
    from .shape import meshkit
    mats = _paints(aircraft, B)
    M = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])  # the design frame to glTF
    meshes = {}
    for i, (e, name) in enumerate(_engine_units(aircraft)):
        if e.type != "turbofan":
            continue
        made = sh.petals(e)
        if made is None:
            continue
        scene, hinge_pt, hinge_axis = made
        if id(e) not in meshes:
            d, _ = sh.nozzle_size(e)
            m = meshkit.build(dict(petal_mesher(d), root=scene))
            # the petal in its hinge's frame (x along the hinge axis)
            qc = _quat_from_x(hinge_axis)
            Rc = _quat_matrix(qc)
            v = (m["positions"] - hinge_pt) @ Rc
            n = m["normals"] @ Rc
            parts = [(v, m["triangles"][m["materials"] == k], mats[int(k)], n) for k in np.unique(m["materials"])]
            meshes[id(e)] = (B.mesh(e.name + " nozzle petal", parts), qc)
        mesh, qc = meshes[id(e)]
        exit_ = np.asarray(e.prop_position, float)
        if e.mirror and name.endswith(" L"):
            exit_ = exit_ * np.array([1.0, -1.0, 1.0])
        opening = float((e.prop_spec or {}).get("petal_open_deg", 12.0))
        for k in range(sh.PETALS):
            phi = 2.0 * np.pi * k / sh.PETALS
            c, s = np.cos(phi), np.sin(phi)
            Rx = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
            q = _quat_from_matrix(M @ Rx)
            joint = "fsim:nozzle:%d:%.6g" % (_engine_joint(i), opening) if opening else "%s petal" % name
            petal = B.node(joint, mesh, translation=hinge_pt, rotation=qc)
            top.append(B.node("%s petal %d" % (name, k + 1), translation=_to_gltf(exit_, origin), rotation=q,
                              children=[petal]))


def _quat_from_matrix(R):
    """glTF's (x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def _plumes(aircraft, B, top, origin):
    """An afterburner flame behind each augmented jet: a glowing outer plume
    and a brighter core, on a node the viewer stretches with the engine's
    afterburner as the simulation reports it (fsim:afterburner:<engine>)."""
    from .jsbsim import _engine_units
    from .shape.airframe import nozzle_size
    outer = B.material("afterburner", (1.0, 0.55, 0.2), 0.0, 1.0, emissive=(1.0, 0.45, 0.12), alpha=0.35)
    core = B.material("afterburner core", (1.0, 0.85, 0.6), 0.0, 1.0, emissive=(1.0, 0.8, 0.5), alpha=0.7)
    q = _quat_from_x(_gltf_direction([1.0, 0.0, 0.0]))  # the jet runs aft
    R = _quat_matrix(q)
    for i, (e, name) in enumerate(_engine_units(aircraft)):
        if e.type != "turbofan" or not e.thrust_wet_kn:
            continue
        d, _ = nozzle_size(e)
        r, L = 0.5 * d, 4.0 * d
        exit_ = np.asarray(e.prop_position, float)
        if e.mirror and name.endswith(" L"):
            exit_ = exit_ * np.array([1.0, -1.0, 1.0])
        v1, t1 = _revolve_mesh([0.0, 0.15 * L, 0.5 * L, 0.85 * L, L], [0.9 * r, 1.0 * r, 0.8 * r, 0.35 * r, 0.02 * r])
        v2, t2 = _revolve_mesh([0.0, 0.1 * L, 0.3 * L, 0.55 * L], [0.7 * r, 0.72 * r, 0.45 * r, 0.02 * r])
        # the mesh is built along +x; the node turns +x onto the jet's direction
        top.append(B.node("fsim:afterburner:%d" % _engine_joint(i), B.mesh(name + " afterburner", [(v1 @ np.eye(3), t1, outer), (v2, t2, core)]),
                          translation=_to_gltf(exit_, origin), rotation=q))
    del R


def _vectoring_parts(aircraft, B, top, origin):
    """The parts that turn with a vectoring nozzle besides its petals and
    its flame (shape.airframe.vectoring_parts: a two-dimensional nozzle's
    flaps, a round one's gimbal seal), one mesh per engine, a node per
    nozzle named "<engine> vectoring"."""
    from .jsbsim import _engine_units
    from .shape import airframe as sh
    from .shape import meshkit
    mats = _paints(aircraft, B)
    M = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])  # the design frame to glTF
    meshes = {}
    for e, name in _engine_units(aircraft):
        if e.type != "turbofan":
            continue
        if id(e) not in meshes:
            scene = sh.vectoring_parts(e)
            if scene is None:
                meshes[id(e)] = None
                continue
            d, _ = sh.nozzle_size(e)
            cell = float(np.clip(d / 150.0, 0.002, 0.01))
            m = meshkit.build({"cell": cell, "error": 0.1 * cell, "safety": 3.0, "sharp_deg": 45.0,
                               "max_triangles": 3000, "root": scene})
            parts = [(m["positions"], m["triangles"][m["materials"] == k], mats[int(k)], m["normals"])
                     for k in np.unique(m["materials"])]
            meshes[id(e)] = B.mesh(e.name + " vectoring", parts)
        if meshes[id(e)] is None:
            continue
        exit_ = np.asarray(e.prop_position, float)
        if e.mirror and name.endswith(" L"):
            exit_ = exit_ * np.array([1.0, -1.0, 1.0])
        top.append(B.node("%s vectoring" % name, meshes[id(e)], translation=_to_gltf(exit_, origin),
                          rotation=_quat_from_matrix(M)))


def _vectoring(aircraft, B, top, origin):
    """Each vectoring nozzle's moving parts - its petals, its flame and
    what _vectoring_parts made - gathered on a node that turns them as the
    flight control computers turn the nozzle: they turn it with the
    surfaces (fcs.vectoring), so the node is a channel mix,
    fsim:elevator:<g>[+aileron:<g>][+rudder:<g>]@<-travel>,<travel>, its
    origin at the hinge and its x axis the one a positive deflection turns
    the jet about (down for a nozzle vectoring in pitch: the thrust pushes
    the tail up, nose down, as a positive elevator does)."""
    from . import fcs
    from .jsbsim import _engine_units
    from .shape import airframe as sh
    units = _engine_units(aircraft)
    by_name = {B.nodes[i]["name"]: i for i in top}
    for v in fcs.vectoring(aircraft):
        e, name = units[v["engine"]]
        exit_ = np.asarray(v["nozzle"], float)
        pivot = exit_ + sh.vectoring_hinge(e)
        # the plane the nozzle turns in: up, canted outboard; the jet turns about
        # the axis across it
        c, s = v["cant"], v["side"]
        axis = np.array([0.0, math.cos(c), s * math.sin(c)])
        mix = [(ch, g) for ch, g in (("elevator", v["k_elevator"]), ("aileron", s * v["k_aileron"]),
                                     ("rudder", -s * v["k_rudder"])) if g != 0.0]
        if not mix:
            continue
        travel = math.degrees(v["travel"])
        joint = "fsim:" + "+".join("%s:%.6g" % (ch, g) for ch, g in mix) + "@%.6g,%.6g" % (-travel, travel)
        q = _quat_from_x(_gltf_direction(axis))
        R, p = _quat_matrix(q), _to_gltf(pivot, origin)
        names = ["fsim:afterburner:%d" % _engine_joint(v["engine"]), "%s vectoring" % name] + \
                ["%s petal %d" % (name, k + 1) for k in range(sh.PETALS)]
        kids = []
        for n in names:
            i = by_name.get(n)
            if i is None:
                continue
            node = B.nodes[i]
            t = np.asarray(node.get("translation", [0.0, 0.0, 0.0]), float)
            Rn = _quat_matrix(np.asarray(node.get("rotation", [0.0, 0.0, 0.0, 1.0]), float))
            node["translation"] = [float(x) for x in R.T @ (t - p)]
            node["rotation"] = [float(x) for x in _quat_from_matrix(R.T @ Rn)]
            top.remove(i)
            kids.append(i)
        if kids:
            top.append(B.node(joint, translation=p, rotation=q, children=kids))


def write_glb(aircraft, path, origin, report=None, solid=None):
    """The design as a .glb. solid: the airframe as one closed, filleted solid
    (the default when hangar's mesher is built) or each part a primitive
    skin; report, a dict, gets the meshes' numbers the checks read."""
    from .shape import meshkit
    origin = np.asarray(origin, float)
    B = _Builder()
    top = []
    if solid is None:
        solid = meshkit.library() is not None
    if solid:
        _solid_parts(aircraft, B, top, origin, report if report is not None else {})
    else:
        _primitive_parts(aircraft, B, top, origin)
    if solid:
        _plumes(aircraft, B, top, origin)
        _nozzle_petals(aircraft, B, top, origin)
        _vectoring_parts(aircraft, B, top, origin)
        _vectoring(aircraft, B, top, origin)
        _propellers(aircraft, B, top, origin)
    # landing gear: wheels and struts (the solid model has its own)
    dark = B.material("tyre", (0.08, 0.08, 0.09), 0.0, 0.9)
    strut_m = B.material("strut", (0.55, 0.56, 0.60), 0.6, 0.4)
    for gear in ([] if solid else aircraft.gear):
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
    for e in ([] if solid else aircraft.engines):
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
    # the binary chunk and the buffer it holds both end on a 4-byte boundary:
    # a loader may insist the two lengths agree (vsgXchange loads no geometry
    # at all when they do not)
    while len(B.bin) % 4:
        B.bin.append(0)
    doc = {"asset": {"version": "2.0", "generator": "hangar"}, "scene": 0, "scenes": [{"nodes": [root]}],
           "nodes": B.nodes, "meshes": B.meshes, "materials": B.materials, "accessors": B.accessors,
           "bufferViews": B.views, "buffers": [{"byteLength": len(B.bin)}]}
    if B.textures:
        doc.update(images=B.images, textures=B.textures,
                   samplers=[{"magFilter": 9729, "minFilter": 9987, "wrapS": 33071, "wrapT": 33071}])
    if anims:
        doc["animations"] = anims
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(B.bin)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack("<II", len(B.bin), 0x004E4942))
        f.write(bytes(B.bin))
    with open(path + ".manifest", "w", encoding="utf-8", newline="\n") as f:  # LF, as git keeps it
        f.write("# Generated by hangar: glTF x = -y, y = up, z = forward (nose); origin at the CG.\n"
                "forward -y\nup +z\nscale 1.0\n")
        f.write("".join(line + "\n" for line in surface_lines(aircraft, origin)))
    return path


def surface_lines(aircraft, origin):
    """The manifest's lines for the lifting surfaces the viewer's airflow
    effects hang on - vapour over a wing, a vortex off a strake or a wing
    tip: `<kind> <x> <y> <z> <chord> ...`, one line per mirrored wing,
    strake or canard, its right half's sections root to tip, each a leading
    edge in body axes from the origin (x forward, y right, z down; m) and a
    chord."""
    lines = []
    for s in aircraft.surfaces:
        if s.kind not in ("wing", "strake", "canard") or not s.mirror:
            continue
        values = []
        for sec in s.sections:
            q = np.asarray(sec.le, float) - np.asarray(origin, float)
            values += [-q[0], q[1], -q[2], sec.chord]
        if not lines:
            lines.append("# lifting surfaces, right half, root to tip: <kind> then per section its leading edge "
                         "(body axes from the origin, m) and chord")
        lines.append(s.kind + " " + " ".join("%.4f" % (v if abs(v) >= 5e-5 else 0.0) for v in values))
    return lines
