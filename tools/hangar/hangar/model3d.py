"""The design as a glTF 2.0 binary (.glb) for the platform's viewer.

Written with the numbers the viewer's manifest describes (forward -y, up +z
after its Y-up to Z-up import, like the Cesium model): glTF x = -y (left),
y = z (up), z = -x (forward), a proper rotation of the design frame, with the
origin at the centre of gravity - the point the simulation reports. Each
component is its own node and every control surface its own primitive
(coloured as in the drawings); propellers spin with a glTF animation, which
the viewer plays.
"""
import json
import math
import struct

import numpy as np

from .report.render import COLOURS, CONTROL_COLOURS


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

    def node(self, name, mesh=None, translation=None, children=None):
        n = {"name": name}
        if mesh is not None:
            n["mesh"] = mesh
        if translation is not None:
            n["translation"] = [float(x) for x in translation]
        if children:
            n["children"] = children
        self.nodes.append(n)
        return len(self.nodes) - 1


def write_glb(aircraft, path, origin):
    origin = np.asarray(origin, float)
    B = _Builder()
    top = []
    for name, kind, v, t, g in aircraft.mesh(fine=True):
        vg = _to_gltf(v, origin)
        base = B.material(kind, COLOURS.get(kind, (0.85, 0.85, 0.88)))
        parts = [(vg, t[g <= 0], base)]
        surf = next((s for s in aircraft.surfaces if s.name == name), None)
        if surf is not None:
            for k, ctrl in enumerate(surf.controls, start=1):
                parts.append((vg, t[g == k], B.material("control_" + ctrl.channel, CONTROL_COLOURS.get(ctrl.channel, (0.9, 0.7, 0.2)))))
        top.append(B.node(name, B.mesh(name, parts)))
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
