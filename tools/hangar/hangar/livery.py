"""The paint on a design's 3D model: its colours as a texture.

aircraft/<name>/paint.toml (optional; a plain light grey without it):

    [paint]
    scheme = "camo"            # single | two-tone | camo | stripes
    colours = ["#8c98a4", "#5f6d7c"]   # the base, then the pattern (camo, stripes) or
                               # the upper and lower tones (two-tone)
    underside = "#b9c3cd"      # the belly (optional; the base otherwise)
    scale = 3.0                # m: the camouflage's patch size (camo)
    boundary = -0.2            # z (m) where a two-tone side turns to the underside
    radome = "#6c7480"         # the nose cone, back to radome_x (m)
    radome_x = 1.6
    anti_glare = "#3d4249"     # the panel ahead of the windscreen, from/to (m)
    anti_glare_x = [1.6, 2.9]
    windows = "#1b2127"        # painted cockpit windows (a transport's, no canopy):
    windows_x = [2.4, 4.4]     # the side windows from/to x (m),
    windows_z = [1.95, 2.35]   # between these heights (m),
    windscreen_x = [2.3, 2.7]  # and the windscreen across the nose (seen from above
    windscreen_y = 0.95        # and ahead: its half width, m)
    canopy = "gold"            # the glass: clear | gold | dark
    panel_lines = 0.12         # how much darker the panel lines are (0: none)
    wear = 0.04                # a slight unevenness of the paint (0: none)
    gloss = 0.35               # 0 matte .. 1 glossy
    seed = 7

The model's skin is textured by where each triangle faces: seen from above,
from below, from the side (both sides alike) or from ahead - so a scheme is
drawn as it is in the plan and profile views of a real one."""
import io
import os
import tomllib

import numpy as np

TOP, BOTTOM, SIDE, FRONT = range(4)
SIZE = 2048


def _rgb(c):
    if isinstance(c, str):
        c = c.lstrip("#")
        return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], float) / 255.0
    return np.asarray(c, float)


class Livery:
    def __init__(self, aircraft, bounds):
        """bounds: the model's extent (design frame): [[x0, y0, z0], [x1, y1, z1]]."""
        spec = {}
        path = os.path.join(aircraft.dir, "paint.toml")
        if os.path.isfile(path):
            with open(path, "rb") as f:
                spec = tomllib.load(f).get("paint", {})
        self.spec = spec
        self.aircraft = aircraft
        self.scheme = spec.get("scheme", "single")
        if self.scheme not in ("single", "two-tone", "camo", "stripes"):
            raise ValueError("%s: paint scheme must be single, two-tone, camo or stripes" % path)
        cols = [_rgb(c) for c in spec.get("colours", ["#a4abb3"])]
        self.base = cols[0]
        self.pattern = cols[1] if len(cols) > 1 else cols[0] * 0.8
        self.under = _rgb(spec["underside"]) if "underside" in spec else self.base
        self.seed = int(spec.get("seed", sum(map(ord, aircraft.name))))
        self.gloss = float(spec.get("gloss", 0.35))
        lo, hi = np.asarray(bounds[0], float), np.asarray(bounds[1], float)
        pad = 0.05
        self.x0, self.x1 = lo[0] - pad, hi[0] + pad
        b = max(abs(lo[1]), abs(hi[1])) + pad
        self.y0, self.y1 = -b, b
        self.z0, self.z1 = lo[2] - pad, hi[2] + pad
        L, S, H = self.x1 - self.x0, self.y1 - self.y0, self.z1 - self.z0
        g = 0.2  # m between cells
        # the atlas: top and bottom (x across, y down), then the side (x, z)
        # and the front (y, z) side by side
        wm, hm = L + g + S, 2.0 * S + H + 2.0 * g
        self.ppm = SIZE / max(wm, hm)
        self.cells = {TOP: (0.0, 0.0), BOTTOM: (0.0, S + g), SIDE: (0.0, 2.0 * S + 2.0 * g),
                      FRONT: (L + g, 2.0 * S + 2.0 * g)}
        self.L, self.S, self.H = L, S, H

    # -- where a point of the skin is in the atlas ---------------------------------------------
    def regions(self, normals):
        """Which way each face (its normal, design frame) looks: TOP, BOTTOM, SIDE or FRONT."""
        a = np.abs(normals)
        r = np.full(len(normals), SIDE)
        r[(a[:, 2] >= a[:, 0]) & (a[:, 2] >= a[:, 1]) & (normals[:, 2] > 0)] = TOP
        r[(a[:, 2] >= a[:, 0]) & (a[:, 2] >= a[:, 1]) & (normals[:, 2] <= 0)] = BOTTOM
        r[(a[:, 0] > a[:, 1]) & (a[:, 0] > a[:, 2])] = FRONT
        return r

    def uv(self, p, region):
        """Texture coordinates (0..1, v down) of points p (design frame) drawn in their region."""
        p = np.asarray(p, float)
        u = np.empty(len(p))
        v = np.empty(len(p))
        for reg, (cu, cv) in self.cells.items():
            k = region == reg
            if reg in (TOP, BOTTOM):
                a, b = p[k, 0] - self.x0, p[k, 1] - self.y0
            elif reg == SIDE:
                a, b = p[k, 0] - self.x0, self.z1 - p[k, 2]
            else:
                a, b = p[k, 1] - self.y0, self.z1 - p[k, 2]
            u[k] = (cu + a) * self.ppm / SIZE
            v[k] = (cv + b) * self.ppm / SIZE
        return np.stack([u, v], axis=1)

    def split(self, v, n, t, face_normals=None):
        """The mesh (vertices v, normals n, triangles t; design frame) with its
        vertices split where its faces look different ways; returns (v, n, t, uv)."""
        if face_normals is None:
            e1, e2 = v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]
            face_normals = np.cross(e1, e2)
            face_normals /= np.maximum(np.linalg.norm(face_normals, axis=1), 1e-12)[:, None]
        reg = self.regions(face_normals)
        key = (t.astype(np.int64) * 4 + reg[:, None]).ravel()
        uniq, inv = np.unique(key, return_inverse=True)
        vi, ri = uniq // 4, uniq % 4
        return v[vi], n[vi], inv.reshape(-1, 3), self.uv(v[vi], ri)

    # -- the paint -----------------------------------------------------------------------------
    def _noise(self, shape, cells, rng):
        """A smooth random field over an image shape, about cells blotches across."""
        from PIL import Image
        h, w = shape
        cw, ch = max(2, int(round(cells[0]))), max(2, int(round(cells[1])))
        a = rng.random((ch, cw)).astype(np.float32)
        im = Image.fromarray(a).resize((w, h), Image.BICUBIC)
        return np.asarray(im)

    def image(self):
        """The atlas as JPEG bytes."""
        from PIL import Image
        rng = np.random.default_rng(self.seed)
        img = np.zeros((SIZE, SIZE, 3), np.float32)
        img[:] = self.under
        spec = self.spec
        scale = float(spec.get("scale", 3.0))
        radome, radome_x = spec.get("radome"), float(spec.get("radome_x", 0.0))
        glare, glare_x = spec.get("anti_glare"), spec.get("anti_glare_x", [0.0, 0.0])
        win = spec.get("windows")
        win_x, win_z = spec.get("windows_x", [0.0, 0.0]), spec.get("windows_z", [0.0, 0.0])
        screen_x, screen_y = spec.get("windscreen_x", [0.0, 0.0]), float(spec.get("windscreen_y", 0.95))
        lines = float(spec.get("panel_lines", 0.12))
        wear = float(spec.get("wear", 0.04))
        boundary = float(spec.get("boundary", 0.0))
        for reg, (cu, cv) in self.cells.items():
            w_m = self.L if reg != FRONT else self.S
            h_m = self.S if reg in (TOP, BOTTOM) else self.H
            c0, r0 = int(cu * self.ppm), int(cv * self.ppm)
            w, h = int(np.ceil(w_m * self.ppm)) + 1, int(np.ceil(h_m * self.ppm)) + 1
            w, h = min(w, SIZE - c0), min(h, SIZE - r0)
            # the design coordinates of the cell's texels
            a = np.arange(w) / self.ppm
            b = np.arange(h) / self.ppm
            A, B = np.meshgrid(a, b)
            if reg in (TOP, BOTTOM):
                X, Y, Z = self.x0 + A, self.y0 + B, None
            elif reg == SIDE:
                X, Y, Z = self.x0 + A, None, self.z1 - B
            else:
                X, Y, Z = None, self.y0 + A, self.z1 - B
            cell = np.zeros((h, w, 3), np.float32)
            lower = self.under if "underside" in spec else self.pattern
            below = Z < boundary if Z is not None else None
            if reg == BOTTOM:
                cell[:] = self.under if self.scheme != "two-tone" else lower
            elif self.scheme == "two-tone":
                cell[:] = self.base
                if below is not None:
                    cell[below] = lower
            else:
                cell[:] = self.base
                if self.scheme == "camo":
                    # patches with ragged edges: a coarse field and a finer one on it
                    f = (self._noise((h, w), (w_m / scale, h_m / scale), rng)
                         + 0.35 * self._noise((h, w), (2.5 * w_m / scale, 2.5 * h_m / scale), rng))
                    cell[f > 0.72] = self.pattern
                elif self.scheme == "stripes" and Z is not None:
                    cell[(Z > boundary - 0.12) & (Z < boundary)] = self.pattern
                if below is not None and "underside" in spec and self.scheme != "stripes":
                    cell[below] = self.under
            if radome is not None and X is not None:
                cell[X < radome_x] = _rgb(radome)
            if glare is not None and X is not None and reg == TOP:
                k = (X > glare_x[0]) & (X < glare_x[1]) & (np.abs(Y) < 0.35)
                cell[k] = _rgb(glare)
            if win is not None:
                if reg == SIDE:
                    k = (X > win_x[0]) & (X < win_x[1]) & (Z > win_z[0]) & (Z < win_z[1])
                elif reg == TOP:
                    k = (X > screen_x[0]) & (X < screen_x[1]) & (np.abs(Y) < screen_y)
                elif reg == FRONT:
                    k = (Z > win_z[0]) & (Z < win_z[1]) & (np.abs(Y) < screen_y)
                else:
                    k = np.zeros(cell.shape[:2], bool)
                cell[k] = _rgb(win)
            if lines > 0.0 and reg != FRONT:
                cell[self._joints(reg, X, Y, Z)] *= (1.0 - lines)
            if wear > 0.0:
                f = self._noise((h, w), (w_m / 0.8, h_m / 0.8), rng)
                cell *= (1.0 - wear * (f - 0.5))[:, :, None]
            img[r0:r0 + h, c0:c0 + w] = cell
        out = io.BytesIO()
        Image.fromarray((np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8)).save(out, format="JPEG", quality=88)
        return out.getvalue()

    # -- panel joints: frames round the fuselage, spars and ribs on the surfaces --------------
    WIDTH = 0.011  # m, half a joint's width

    def _joints(self, reg, X, Y, Z):
        """Where the panel joints are over a cell's texels (bool)."""
        a = self.aircraft
        out = np.zeros(X.shape, bool)
        covered = np.zeros(X.shape, bool)  # by a surface: no fuselage frames there
        for surf in a.surfaces:
            vertical = surf.kind in ("fin", "vtail") and not surf.mirror
            if (reg == SIDE) != vertical:
                continue
            span = Z if vertical else np.abs(Y)
            secs = surf.sections
            for s0, s1 in zip(secs[:-1], secs[1:]):
                k0, k1 = (s0.le[2], s1.le[2]) if vertical else (abs(s0.le[1]), abs(s1.le[1]))
                if abs(k1 - k0) < 1e-6:
                    continue
                t = (span - k0) / (k1 - k0)
                inside_span = (t >= 0.0) & (t <= 1.0)
                x_le = s0.le[0] + t * (s1.le[0] - s0.le[0])
                chord = s0.chord + t * (s1.chord - s0.chord)
                f = (X - x_le) / np.maximum(chord, 1e-6)
                inside = inside_span & (f >= 0.0) & (f <= 1.0)
                covered |= inside
                w = self.WIDTH / np.maximum(chord, 1e-6)
                spars = np.zeros(X.shape, bool)
                for c in (0.15, 0.65):
                    spars |= np.abs(f - c) < w
                ribs = np.abs(((span - k0) / 0.85) % 1.0 - 0.5) > 0.5 - self.WIDTH / 0.85
                out |= inside & (spars | (ribs & (f > 0.15) & (f < 0.65)))
        if reg in (TOP, BOTTOM):
            body = np.zeros(X.shape, bool)
            for b in a.bodies:
                if b.kind not in ("fuselage", "nacelle", "intake", "boom"):
                    continue
                hw = np.interp(X, b.x, 0.5 * b.w, left=0.0, right=0.0)
                yc = np.interp(X, b.x, b.y)
                body |= np.abs(Y - yc) < hw
                if b.mirror:
                    body |= np.abs(Y + yc) < hw
        frames = np.abs(((X - self.x0) / 1.1) % 1.0 - 0.5) > 0.5 - self.WIDTH / 1.1
        if reg in (TOP, BOTTOM):  # the fuselage's frames over it, the surfaces' joints beside it
            return (out & ~body) | (frames & body)
        return out | (frames & ~covered)

    def canopy(self):
        """The glass's colour, metallic and roughness."""
        tint = self.spec.get("canopy", "dark")
        return {"clear": ((0.35, 0.40, 0.45), 0.9, 0.05), "gold": ((0.26, 0.21, 0.10), 0.95, 0.05),
                "dark": ((0.07, 0.08, 0.10), 0.9, 0.08)}.get(tint, ((0.07, 0.08, 0.10), 0.9, 0.08))
