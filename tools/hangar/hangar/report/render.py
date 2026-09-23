"""Drawings of a design: three-view, perspective views and overlays.

A small painter's-algorithm renderer: every skin triangle is projected,
shaded by its normal and drawn far to near as a flat polygon. That is enough
for clean orthographic three-views (with reference drawings underneath) and
shaded perspective views, from matplotlib alone.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

COLOURS = {
    "fuselage": (0.86, 0.87, 0.90), "nacelle": (0.80, 0.81, 0.85), "pod": (0.80, 0.81, 0.85),
    "boom": (0.80, 0.81, 0.85), "float": (0.80, 0.81, 0.85),
    "wing": (0.93, 0.93, 0.95), "htail": (0.93, 0.93, 0.95), "canard": (0.93, 0.93, 0.95),
    "fin": (0.93, 0.93, 0.95), "vtail": (0.93, 0.93, 0.95),
}
CONTROL_COLOURS = {"aileron": (0.95, 0.55, 0.20), "elevator": (0.30, 0.60, 0.95), "rudder": (0.85, 0.30, 0.60),
                   "flap": (0.35, 0.75, 0.40)}

# view name -> (right, up, towards the viewer) in the design frame (x aft, y right, z up)
VIEWS = {
    "top": (np.array([0.0, 1.0, 0.0]), np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    "side": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([0.0, -1.0, 0.0])),
    "front": (np.array([0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0])),
}


def _triangles(aircraft, fine=True):
    """All skin triangles: (T, 3, 3) points, (T, 3) base colours."""
    pts, cols = [], []
    for name, kind, v, t, g in aircraft.mesh(fine):
        base = np.array(COLOURS.get(kind, (0.85, 0.85, 0.88)))
        c = np.tile(base, (len(t), 1))
        surf = next((s for s in aircraft.surfaces if s.name == name), None)
        if surf is not None:
            for k, ctrl in enumerate(surf.controls, start=1):
                c[g == k] = CONTROL_COLOURS.get(ctrl.channel, (0.9, 0.7, 0.2))
        pts.append(v[t])
        cols.append(c)
    return np.concatenate(pts), np.concatenate(cols)


def _shade(tri, cols, towards, light):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    n *= np.sign(n @ towards + 1e-12)[:, None]  # two-sided: face the viewer
    k = 0.55 + 0.45 * np.clip(n @ light, 0.0, 1.0)
    return np.clip(cols * k[:, None], 0.0, 1.0)


def draw_view(ax, aircraft, view, tri=None, cols=None, alpha=1.0, edge=None):
    """Draw one orthographic view into ax, in metres (x right, y up on the page)."""
    if tri is None:
        tri, cols = _triangles(aircraft)
    right, up, towards = VIEWS[view]
    light = towards * 0.8 + up * 0.5 + right * 0.3
    light /= np.linalg.norm(light)
    shaded = _shade(tri, cols, towards, light)
    depth = (tri @ towards).mean(axis=1)
    order = np.argsort(depth)
    poly = np.stack([tri @ right, tri @ up], axis=-1)[order]
    ax.add_collection(PolyCollection(poly, facecolors=shaded[order], edgecolors=edge or shaded[order],
                                     linewidths=0.15, alpha=alpha, zorder=2))
    ax.set_aspect("equal")
    ax.autoscale_view()
    return right, up


def _mark(ax, p, right, up, text, colour, marker="o"):
    x, y = p @ right, p @ up
    ax.plot([x], [y], marker=marker, color=colour, ms=7, mec="black", mew=0.6, zorder=5)
    if text:
        ax.annotate(text, (x, y), xytext=(6, 6), textcoords="offset points", fontsize=7, color="black", zorder=6)


def three_view(aircraft, path, marks=None, title=None, references=None):
    """Top, side and front views plus a perspective one on one sheet.
    marks: {label: (point, colour)} drawn in every view (CG, neutral point...).
    references: {view: (image array, extent)} drawn underneath."""
    tri, cols = _triangles(aircraft)
    fig = plt.figure(figsize=(15, 10.5), dpi=110)
    layout = {"top": (0.03, 0.30, 0.46, 0.64), "side": (0.53, 0.66, 0.45, 0.28),
              "front": (0.53, 0.36, 0.45, 0.26), "iso": (0.53, 0.02, 0.45, 0.31)}
    for view, rect in layout.items():
        ax = fig.add_axes(rect)
        if view == "iso":
            perspective(ax, aircraft, tri, cols)
            ax.set_title("perspective", fontsize=9)
            continue
        if references and view in references:
            img, extent = references[view]
            ax.imshow(img, extent=extent, zorder=1, alpha=0.9, cmap="gray" if img.ndim == 2 else None)
        right, up = draw_view(ax, aircraft, view, tri, cols, alpha=0.75 if references and view in references else 1.0)
        for label, (p, colour) in (marks or {}).items():
            _mark(ax, np.asarray(p, float), right, up, label, colour)
        ax.grid(True, lw=0.3, alpha=0.4)
        ax.tick_params(labelsize=7)
        ax.set_title(view + " (m)", fontsize=9)
    fig.text(0.03, 0.25, _summary_text(aircraft), fontsize=8, family="monospace", va="top")
    fig.suptitle(title or "%s - %s" % (aircraft.name, aircraft.description), fontsize=11)
    fig.savefig(path)
    plt.close(fig)


def perspective(ax, aircraft, tri=None, cols=None, azimuth=35.0, elevation=25.0):
    """A shaded perspective view from front-left-above (angles in degrees)."""
    if tri is None:
        tri, cols = _triangles(aircraft)
    az, el = np.radians(azimuth), np.radians(elevation)
    # camera direction from the aircraft towards the viewer (front-left, above)
    towards = np.array([-np.cos(el) * np.cos(az), -np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(np.array([0.0, 0.0, 1.0]), towards)
    right /= np.linalg.norm(right)
    up = np.cross(towards, right)
    centre = tri.reshape(-1, 3).mean(axis=0)
    size = np.ptp(tri.reshape(-1, 3), axis=0).max()
    eye = centre + towards * size * 2.2
    rel = tri - eye
    zc = -(rel @ towards)
    x = (rel @ right) / zc
    y = (rel @ up) / zc
    light = towards * 0.6 + up * 0.7 - right * 0.2
    light /= np.linalg.norm(light)
    shaded = _shade(tri, cols, towards, light)
    order = np.argsort(-zc.mean(axis=1))
    ax.add_collection(PolyCollection(np.stack([x, y], axis=-1)[order], facecolors=shaded[order],
                                     edgecolors=shaded[order], linewidths=0.1))
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.axis("off")


def _summary_text(a):
    s = a.summary()
    r = s["reference"]
    lines = ["reference  S %.2f m2   b %.2f m   c %.3f m   AR %.2f" % (r["area_m2"], r["span_m"], r["chord_m"],
                                                                          r["span_m"] ** 2 / r["area_m2"])]
    for name, d in s["surfaces"].items():
        extra = ""
        if "volume_coefficient" in d:
            extra = "  V %.3f (arm %.2f m)" % (d["volume_coefficient"], d["arm_m"])
        lines.append("%-10s %-6s S %5.2f  b %5.2f  AR %5.2f  taper %.2f  sweep(c/4) %5.1f  dihedral %4.1f  t/c %.3f%s"
                     % (name, d["kind"], d["area_m2"], d["span_m"], d["aspect_ratio"], d["taper"], d["sweep_c4_deg"],
                        d["dihedral_deg"], d["thickness"], extra))
    for name, d in s["bodies"].items():
        lines.append("%-10s body   L %5.2f  w %4.2f  h %4.2f  vol %5.2f m3  fineness %.1f"
                     % (name, d["length_m"], d["max_width_m"], d["max_height_m"], d["volume_m3"], d["fineness"]))
    return "\n".join(lines)


def views_png(aircraft, path, azimuths=(35, 150, 250), elevation=20):
    """Several perspective views side by side, for looking the shape over."""
    tri, cols = _triangles(aircraft)
    fig, axes = plt.subplots(1, len(azimuths), figsize=(5 * len(azimuths), 4.2), dpi=110)
    for ax, az in zip(np.atleast_1d(axes), azimuths):
        perspective(ax, aircraft, tri, cols, azimuth=az, elevation=elevation)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
