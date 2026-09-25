"""A design as signed-distance scenes for meshkit (native/meshkit.h).

The airframe is one solid: bodies and lifting surfaces joined by smooth
unions - the fillets real aircraft have where a wing, a fin or a nacelle
meets the fuselage - with the canopy set in hard, and every control surface
cut out of it with a small gap. Each control surface is a solid of its own,
cut from the same surface, so it can turn on its hinge.
"""
import math

import numpy as np

from ..geometry import airfoil as af

# materials: what the glTF writer paints each triangle with
MATERIALS = ("skin", "control", "glass", "dark", "metal", "tyre", "strut", "nozzle")
SKIN, CONTROL, GLASS, DARK, METAL, TYRE, STRUT, NOZZLE = range(len(MATERIALS))

GAP = 0.012  # m: between a control surface and what it is cut from


def cell_size(aircraft):
    """The mesher's grid: some 1,500 cells along the aircraft."""
    lo, hi = aircraft.extent()
    return float(np.clip(np.max(hi - lo) / 1500.0, 0.002, 0.02))


class Foils:
    """The airfoil tables of one scene, each sent once."""

    def __init__(self):
        self.entries, self._index = [], {}

    def index(self, foil):
        key = foil.name
        if key not in self._index:
            x = af.cosine_spacing(90)
            self._index[key] = len(self.entries)
            self.entries.append({"x": x, "upper": foil.upper(x), "lower": foil.lower(x)})
        return self._index[key]


def loft(body, material=SKIN, samples_per_m=60):
    """A body's stations, sampled densely along x (the loft between them is
    linear)."""
    n = max(40, int(body.length * samples_per_m))
    xs = np.unique(np.concatenate([np.linspace(body.x[0], body.x[-1], n + 1), body.x]))
    w, top, bot, yc, _ = body.section(xs)
    zc, nu, nl = body.halves(xs)
    node = {"prim": "loft", "material": material, "mirror": bool(body.mirror), "x": xs, "yc": yc, "zc": zc,
            "hw": 0.5 * w, "hu": top - zc, "hl": zc - bot, "nu": nu, "nl": nl}
    k = body.slant(xs)
    if np.any(k != 0.0):
        node["lean"] = k
    return node


class _Root:
    """A section added inboard of a surface's root, so the root reaches into
    the body it stands on."""

    def __init__(self, section, shift):
        self.le = section.le - shift
        self.chord, self.twist, self.airfoil = section.chord, section.twist, section.airfoil


def rooted(surface, bodies_scene):
    """The surface's sections, with one more inboard of the root when the root
    stops short of the bodies (a fin standing off the fuselage would float):
    the root section carried along the span axis until it lies 4 cm inside."""
    secs = list(surface.sections)
    if bodies_scene is None:
        return secs
    from . import meshkit
    root = secs[0]
    c, u = surface.frame(0.0)
    d = secs[1].le - root.le
    span = np.array([0.0, d[1], d[2]])
    span /= np.linalg.norm(span)
    chordwise = [root.le + c * f * root.chord + u * root.airfoil.camber(f) * root.chord for f in (0.25, 0.5, 0.75)]
    steps = np.linspace(0.0, 2.0, 81)
    pts = np.array([p - span * t for t in steps for p in chordwise])
    dist, _ = meshkit.evaluate(bodies_scene, pts)
    inside = dist.reshape(len(steps), len(chordwise)).max(axis=1) < -0.04
    if inside[0] or not inside.any():
        return secs  # already in (or no body along the span axis to reach)
    shift = steps[int(np.argmax(inside))]
    return [_Root(root, span * shift)] + secs


def segments(surface, foils, material=SKIN, inflate=0.0, mirror=None, sections=None):
    """The surface's segments between its defined sections."""
    out = []
    secs = sections or surface.sections
    for i in range(len(secs) - 1):
        a, b = secs[i], secs[i + 1]
        d = b.le - a.le
        span = np.array([0.0, d[1], d[2]])
        out.append({"prim": "wing", "material": material, "mirror": surface.mirror if mirror is None else mirror,
                    "le": [a.le, b.le], "chord": [a.chord, b.chord], "twist": [a.twist, b.twist],
                    "span": span / np.linalg.norm(span), "foils": [foils.index(a.airfoil), foils.index(b.airfoil)],
                    "inflate": inflate})
    return out


def union(children, k=0.0):
    children = [c for c in children if c is not None]
    if len(children) == 1:
        return children[0]
    return {"op": "union", "k": k, "children": children}


def control_regions(surface, ctrl, mirror=None, grow=0.0):
    """Where a control surface lies: per segment it spans, the stations and
    the chord fractions ahead of and behind it (all-moving: all of it)."""
    etas = surface.eta_sections
    out = []
    for i in range(len(etas) - 1):
        e0, e1 = max(ctrl.eta0, etas[i]), min(ctrl.eta1, etas[i + 1])
        if e1 <= e0 + 1e-9:
            continue
        a, b = surface.sections[i], surface.sections[i + 1]
        d = b.le - a.le
        span = np.array([0.0, d[1], d[2]])
        t = [(e0 - etas[i]) / (etas[i + 1] - etas[i]), (e1 - etas[i]) / (etas[i + 1] - etas[i])]
        # across a section inside the control, into the next segment's region by
        # 2 cm: regions that only touch would leave the piece in two
        over = 0.02 / max(float(np.linalg.norm(span)), 1e-6)
        if e0 > ctrl.eta0 + 1e-9:
            t[0] -= over
        if e1 < ctrl.eta1 - 1e-9:
            t[1] += over
        back = [1.2, 1.2]
        if ctrl.channel == "lef":  # a leading-edge device: ahead of its hinge
            front, back = [-0.2, -0.2], [ctrl.chord_fraction(e0), ctrl.chord_fraction(e1)]
        elif ctrl.all_moving:
            front = [-0.2, -0.2]
        else:
            front = [1.0 - ctrl.chord_fraction(e0), 1.0 - ctrl.chord_fraction(e1)]
        out.append({"prim": "wingregion", "mirror": surface.mirror if mirror is None else mirror,
                    "le": [a.le, b.le], "chord": [a.chord, b.chord], "twist": [a.twist, b.twist],
                    "span": span / np.linalg.norm(span), "t": t, "front": front, "back": back})
    region = union(out)
    return {"op": "offset", "r": grow, "child": region} if grow else region


def nozzle_size(engine):
    """Exit diameter and visible length of a jet's nozzle: given in
    [engine.nozzle], else 0.85 and 0.8 times the engine's diameter (Raymer's
    size from its thrust)."""
    spec = engine.prop_spec or {}
    d = float(spec.get("diameter", 0.85 * engine.jet_size()[1]))
    return d, float(spec.get("length", 0.8 * d))


PETALS = 12        # a round nozzle's divergent petals
PETAL_FRONT = 0.4  # of the nozzle's visible length ahead of the exit: where they hinge
PETAL_OPEN_DEG = 12.0  # how far a petal opens when the design does not say (petal_open_deg)


def _round(engine):
    """A round nozzle's numbers: exit (the defined engine's), diameter,
    visible length, wall thickness and the petals' hinge station (m, ahead
    of the exit: negative)."""
    d, length = nozzle_size(engine)
    return np.asarray(engine.prop_position, float), d, length, max(0.02, 0.03 * d), -PETAL_FRONT * length


def nozzles(aircraft):
    """Each jet's nozzle as a solid of revolution along x, and the cavity
    inside it down to the turbine face: (solids, cavities). A round nozzle's
    case stops where its petals hinge (petals())."""
    solids, cavities = [], []
    for e in aircraft.engines:
        if e.type != "turbofan":
            continue
        d, length = nozzle_size(e)
        r, wall = 0.5 * d, max(0.02, 0.03 * d)
        exit_ = np.asarray(e.prop_position, float)
        mirror = bool(e.mirror)
        shape = (e.prop_spec or {}).get("shape", "round")
        if shape == "2d":
            # a two-dimensional nozzle: the round case turning into a flat exit
            # (the F-22's), as a boxy loft
            xs = exit_[0] + np.array([-length - 0.4, -length, -0.5 * length, 0.0])
            hw = np.array([1.05 * r, 1.05 * r, 0.95 * r, 0.9 * r])
            hh = np.array([1.0 * r, 0.95 * r, 0.62 * r, 0.52 * r])
            n = np.array([2.2, 2.4, 5.0, 8.0])
            solids.append({"prim": "loft", "material": METAL, "mirror": mirror, "x": xs, "yc": np.full(4, exit_[1]),
                           "zc": np.full(4, exit_[2]), "hw": hw, "hu": hh, "hl": hh, "nu": n, "nl": n})
            xi = exit_[0] + np.array([-1.2 * length, -0.5 * length, 0.0, 0.3])
            cavities.append({"prim": "loft", "material": DARK, "mirror": mirror, "x": xi, "yc": np.full(4, exit_[1]),
                             "zc": np.full(4, exit_[2]), "hw": np.array([0.7, 0.8, 0.9, 0.9]) * r - wall,
                             "hu": np.array([0.6, 0.55, 0.52, 0.52]) * r - wall,
                             "hl": np.array([0.6, 0.55, 0.52, 0.52]) * r - wall,
                             "nu": np.array([2.2, 4.0, 8.0, 8.0]), "nl": np.array([2.2, 4.0, 8.0, 8.0])})
            continue
        # round: the case to the petals' hinge (the petals close the boattail)
        solids.append({"prim": "revolve", "material": METAL, "mirror": mirror, "origin": exit_, "axis": [1.0, 0.0, 0.0],
                       "s": [-length - 0.4, -length, -PETAL_FRONT * length],
                       "r": [1.08 * r, 1.1 * r, 1.05 * r]})
        # inside: the divergent flaps, the throat, the dark turbine face deep in
        cavities.append({"prim": "revolve", "material": DARK, "mirror": mirror, "origin": exit_, "axis": [1.0, 0.0, 0.0],
                         "s": [-1.3 * length, -1.2 * length, -0.45 * length, 0.0, 0.5],
                         "r": [0.72 * r, 0.72 * r, 0.8 * r, r - wall, r - wall]})
    return solids, cavities


def petal_clearances(aircraft):
    """The space each round nozzle's petals sweep, from just ahead of their
    hinges aft and as wide as they open, as solids to cut out of the
    airframe. A fuselage or nacelle that runs on past the hinges at about the
    nozzle's own size puts its skin where the petals are - two surfaces a
    millimetre apart that flicker through each other, and petals that open
    through the skin; the petals alone are the nozzle there."""
    out = []
    for e in aircraft.engines:
        if e.type != "turbofan" or (e.prop_spec or {}).get("shape", "round") != "round":
            continue
        exit_, d, length, wall, hinge = _round(e)
        r = 0.5 * d
        opening = np.radians(float((e.prop_spec or {}).get("petal_open_deg", PETAL_OPEN_DEG)))
        # the petals' tips wide open, their thickness and a gap
        reach = r + abs(hinge) * np.sin(opening) + 1.5 * wall + 0.01
        out.append({"prim": "revolve", "material": METAL, "mirror": bool(e.mirror), "origin": exit_,
                    "axis": [1.0, 0.0, 0.0], "s": [hinge - 0.01, 1.0], "r": [reach, reach]})
    return out


def nozzle_paint(aircraft):
    """How the airframe is painted round each round nozzle, as (material,
    region) pairs: the nozzle's burnt metal over its visible length - the
    case and whatever skin is near it, from its length ahead of the exit
    back to the petals - and ahead of that the buried front of the case in
    the skin's paint. The case stands proud of a fuselage or nacelle by a
    centimetre or two, or runs along inside it, and where the two surfaces
    were that close each point took the paint of whichever was a hair nearer:
    metal and skin in ragged patches, which reads as parts cutting through
    each other. Painted by where it is, the line between them is a ring
    round the nozzle."""
    out = []
    for e in aircraft.engines:
        if e.type != "turbofan" or (e.prop_spec or {}).get("shape", "round") != "round":
            continue
        exit_, d, length, _, hinge = _round(e)
        r = 0.5 * d
        ring = {"material": METAL, "mirror": bool(e.mirror), "origin": exit_, "axis": [1.0, 0.0, 0.0]}
        out.append((SKIN, dict(ring, prim="revolve", s=[-length - 0.45, -length], r=[1.1 * r + 0.015] * 2)))
        out.append((NOZZLE, dict(ring, prim="revolve", s=[-length, hinge + 0.02], r=[1.5 * r] * 2)))
    return out


def painted(solid, regions):
    """solid with its surface inside each region painted that region's
    material, its distance unchanged: the solid without the regions, and
    each part inside one painted - reaching 2 mm further, so the parts
    overlap and leave no face between them. regions: (material, node)."""
    rest = {"op": "subtract", "k": 0.0, "a": solid, "b": union([node for _, node in regions])}
    parts = [{"op": "paint", "material": m, "child": {"op": "intersect", "k": 0.0, "children": [
        solid, {"op": "offset", "r": 0.002, "child": node}]}} for m, node in regions]
    return {"op": "union", "k": 0.0, "children": [rest] + parts}


def petals(engine):
    """A round nozzle's petal - one of PETALS around its exit, the one on +y
    - as a scene in the nozzle's frame (origin at the exit, x aft), with its
    hinge: (scene, hinge point, hinge axis), the axis turned so a positive
    turn opens it. None for a two-dimensional nozzle."""
    if (engine.prop_spec or {}).get("shape", "round") != "round":
        return None
    _, d, length, wall, s0 = _round(engine)
    r = 0.5 * d

    def shell(r_out, r_in, material):
        """A shell of revolution from just ahead of the hinge to the exit,
        between two profiles (radii at the front, a little behind it and
        at the exit)."""
        s = [s0 - 0.02, -0.04, 0.0]
        return {"op": "subtract", "k": 0.0, "cut_material": DARK,
                "a": {"prim": "revolve", "material": material, "origin": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0],
                      "s": s, "r": r_out},
                "b": {"prim": "revolve", "material": DARK, "origin": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0],
                      "s": [s0 - 0.1] + s[1:] + [0.1], "r": [r_in[0]] + list(r_in[1:]) + [r_in[-1]]}}

    def slab(z0, z1):
        """Its sector's slice across the nozzle (z, the way round)."""
        return {"prim": "box", "material": NOZZLE, "centre": [0.5 * s0, r, 0.5 * (z0 + z1)],
                "half": [0.5 * abs(s0) + 0.05, 0.5 * r, 0.5 * (z1 - z0)]}

    # the petal: a plate that tucks in under the case's trailing edge (the
    # case ends at 1.05 r a centimetre ahead of the hinge, so the petal's
    # front, which dips as it opens, stays inside it), converging a little
    outer = np.array([1.0, 0.99, 0.98]) * r
    inner = outer - wall
    width = 2.0 * r * np.sin(np.pi / PETALS) * 0.97  # a thin gap between petals
    plate = {"op": "intersect", "k": 0.0, "children": [shell(outer, inner, NOZZLE), slab(-0.5 * width, 0.5 * width)]}
    # and a seal on its inside that reaches under the next petal round (+z),
    # joined to it by a step: without it the gaps the open petals leave
    # showed the sky through the nozzle. It clears the next petal's inside by
    # a few millimetres, closed or open (the next one swings out further).
    opening = np.radians(float((engine.prop_spec or {}).get("petal_open_deg", PETAL_OPEN_DEG)))
    gap = 2.0 * np.sin(np.pi / PETALS) * (r + abs(s0) * np.sin(opening)) - width + 0.01  # at the exit, wide open
    t = 0.5 * wall
    seal = {"op": "intersect", "k": 0.0, "children": [shell(inner - 0.004, inner - 0.004 - t, DARK),
                                                         slab(0.5 * width - 0.3 * gap, 0.5 * width + gap)]}
    step = {"op": "intersect", "k": 0.0, "children": [shell(inner + 0.5 * wall, inner - 0.004 - t, DARK),
                                                         slab(0.5 * width - 0.3 * gap, 0.5 * width)]}
    scene = {"op": "union", "k": 0.0, "children": [plate, seal, step]}
    # hinged across its front edge; a positive turn about +z swings its aft
    # end out (+y)
    return scene, np.array([s0, outer[0] - 0.5 * wall, 0.0]), np.array([0.0, 0.0, 1.0])


def strut(st):
    """A bracing strut as a bar of streamlined section between its ends: its
    chord along x as nearly as the strut's own direction allows, its edges
    rounded."""
    d = st.b - st.a
    length = float(np.linalg.norm(d))
    a = d / length
    c = np.array([1.0, 0.0, 0.0]) - a * a[0]
    if np.linalg.norm(c) < 1e-3:  # a strut along x: its chord across, level
        c = np.array([0.0, 1.0, 0.0]) - a * a[1]
    c /= np.linalg.norm(c)
    return {"prim": "box", "material": SKIN, "mirror": st.mirror, "centre": 0.5 * (st.a + st.b),
            "axes": [a, c, np.cross(a, c)], "half": [0.5 * length, 0.5 * st.chord, 0.5 * st.thickness],
            "round": 0.45 * st.thickness}


def fillet(surface, bodies):
    """Fillet radius where a surface meets the fuselage: half its root's
    thickness, for a fin a third; none for a surface that meets no body."""
    root = surface.sections[0]
    t = root.airfoil.thickness_ratio * root.chord
    if not any(b.x[0] - 0.2 <= root.le[0] <= b.x[-1] + 0.2 for b in bodies):
        return 0.0
    f = 0.35 if surface.kind in ("fin", "vtail") else 0.5
    if surface.kind == "strake":
        f = 1.5
    return float(np.clip(f * t, 0.02, 0.35))


def is_canopy(body):
    return body.kind == "pod" and "canopy" in body.name.lower()


def frames(body):
    """A canopy's frames: bands 12 mm proud of the glass round it at each of
    its frames' stations."""
    out = []
    for x in body.frames:
        w, top, bot, yc, _ = body.section(x)
        out.append({"op": "intersect", "k": 0.0, "children": [
            {"op": "offset", "r": 0.012, "child": loft(body, SKIN)},
            {"prim": "box", "material": SKIN, "mirror": bool(body.mirror), "round": 0.004,
             "centre": [x, float(yc), 0.5 * float(top + bot)],
             "half": [0.5 * body.frame_width, 0.5 * float(w) + 0.05, 0.5 * float(top - bot) + 0.05]}]})
    return out


def _extended(node, ahead):
    """A loft with its first section carried ahead by ahead (m)."""
    out = dict(node)
    for k in ("yc", "zc", "hw", "hu", "hl", "nu", "nl", "lean"):
        if k in node:
            out[k] = np.concatenate([[node[k][0]], node[k]])
    out["x"] = np.concatenate([[node["x"][0] - ahead], node["x"]])
    return out


def _behind(point, normal, length, mirror):
    """A box whose front face is the plane through point with the given
    forward normal, reaching length behind it - and as far across, so a
    steeply raked plane's box still holds a long cowl's far end."""
    a = -np.asarray(normal, float)
    u = np.cross(a, [0.0, 0.0, 1.0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    across = max(6.0, length)
    return {"prim": "box", "material": SKIN, "mirror": mirror, "centre": np.asarray(point, float) + a * 0.5 * length,
            "axes": [a, u, v], "half": [0.5 * length, across, across]}


def intake(body):
    """An intake's cowl, cut off at its lip plane, and its duct: (cowl, duct)."""
    p, n = body.lip_plane()
    # how far ahead of its centre the lip plane reaches, at the opening's edge
    height = body.top[0] - body.bottom[0]
    ahead = (0.05 + abs(np.tan(np.radians(body.rake))) * height
             + abs(np.tan(np.radians(body.sweep))) * (body.w[0] + abs(float(body.slant(body.x[0]))) * height))
    cowl = {"op": "intersect", "k": 0.0,
            "children": [_extended(loft(body), ahead + 0.05), _behind(p, n, body.length + ahead + 0.1, bool(body.mirror))]}
    # the duct: the opening less the lip, open ahead of the lip plane, then
    # climbing and narrowing towards the engine - along the cowl, as the cowl
    # moves sideways or leans
    f = np.linspace(0.0, 1.0, 24)
    xs = body.x[0] + body.duct * f
    w, top, bot, _, _ = body.section(body.x[0])
    zc, nu, nl = body.halves(body.x[0])
    t = body.lip
    k = 1.0 + (body.duct_taper - 1.0) * f
    duct = {"prim": "loft", "material": DARK, "mirror": bool(body.mirror), "x": xs, "yc": body.section(xs)[3],
            "zc": zc + body.duct_rise * f, "hw": np.maximum(0.5 * w - t, 0.01) * k,
            "hu": np.maximum(top - zc - t, 0.01) * k, "hl": np.maximum(zc - bot - t, 0.01) * k,
            "nu": np.full_like(f, nu), "nl": np.full_like(f, nl), "lean": body.slant(xs)}
    # open through the lip plane and no further: ahead of it may stand the
    # fuselage the intake hugs
    duct = {"op": "intersect", "k": 0.0,
            "children": [_extended(duct, ahead), _behind(p + 0.05 * n, n, body.duct + ahead + 0.2, bool(body.mirror))]}
    return cowl, duct


def airframe(aircraft, cell=None, error=None, gear=None):
    """The scene of the fixed airframe; gear, a list, gets the landing gear's
    bays and doors (shape/gear.py) - cut into it here."""
    h = cell or cell_size(aircraft)
    foils = Foils()
    structure = [b for b in aircraft.bodies if not is_canopy(b) and b.kind != "intake"]
    canopies = [b for b in aircraft.bodies if is_canopy(b)]
    intakes = [intake(b) for b in aircraft.bodies if b.kind == "intake"]
    solid = None
    for b in structure:
        node = loft(b)
        solid = node if solid is None else union([solid, node], 0.25 * min(b.max_width, b.max_height))
    # the intake cowls (a nacelle may be one) before the surfaces that stand on them
    for cowl, _ in intakes:
        solid = cowl if solid is None else union([solid, cowl], 0.12)
    structure = structure + [b for b in aircraft.bodies if b.kind == "intake"]
    body_solid = solid
    inflate = 0.35 * h
    bodies = {"root": solid} if solid is not None else None
    own = {}
    for s in aircraft.surfaces:
        node = union(segments(s, foils, inflate=inflate, sections=rooted(s, bodies)))
        own[id(s)] = node
        solid = node if solid is None else union([solid, node], fillet(s, structure))
    for st in aircraft.struts:
        solid = union([solid, strut(st)], 0.03)
    jets, cavities = nozzles(aircraft)
    cavities += [duct for _, duct in intakes]
    if jets:
        solid = union([solid] + jets, 0.06)
    clear = petal_clearances(aircraft)
    if clear:  # nothing of the airframe where the petals are, or where they open to
        solid = {"op": "subtract", "k": 0.0, "a": solid, "b": union(clear), "cut_material": DARK}
    paint = nozzle_paint(aircraft)
    if paint:
        solid = painted(solid, paint)
    if canopies:
        solid = union([solid] + [loft(b, GLASS) for b in canopies] + [f for b in canopies for f in frames(b)], 0.0)
    from .gear import bays
    plan = bays(aircraft, solid, foils.entries)  # the skin before the ducts are cut: the doors
    if cavities:
        solid = {"op": "subtract", "k": 0.01, "a": solid, "b": union(cavities), "cut_material": DARK}
    if gear is not None:
        gear.extend(plan)
    if plan:
        solid = {"op": "subtract", "k": 0.0, "a": solid, "b": union([p["cut"] for p in plan]), "cut_material": DARK}
    # each control's cut: its region of its own surface - near the surface
    # only (and its fillets), and never through a body (a boom through a flap)
    cuts = [{"op": "intersect", "k": 0.0, "children": [control_regions(s, c, grow=0.5 * GAP),
                                                       {"op": "offset", "r": 0.15, "child": own[id(s)]}]}
            for s in aircraft.surfaces for c in s.controls + s.leading]
    if cuts and body_solid is not None:
        cuts = [{"op": "subtract", "k": 0.0, "a": c, "b": body_solid} for c in cuts]
    if cuts:
        solid = {"op": "subtract", "k": 0.0, "a": solid, "b": union(cuts), "cut_material": SKIN}
    return {"cell": h, "error": error or 0.25 * h, "safety": 3.0, "sharp_deg": 48.0, "max_triangles": 75000,
            "foils": foils.entries, "root": solid}


def control_pieces(aircraft, cell=None, error=None):
    """A scene per control surface and side: (surface, control, side, scene);
    side +1 the defined half (y >= 0), -1 its mirror, 0 a single surface."""
    h = cell or cell_size(aircraft)
    out = []
    for s in aircraft.surfaces:
        for c in s.controls + s.leading:
            foils = Foils()
            wing = union(segments(s, foils, CONTROL, inflate=0.35 * h, mirror=False))
            piece = {"op": "intersect", "k": 0.0,
                     "children": [wing, control_regions(s, c, mirror=False, grow=-0.5 * GAP)]}
            sides = (1, -1) if s.mirror else (0,)
            for side in sides:
                root = piece if side >= 0 else {"op": "reflect", "child": piece}
                out.append((s, c, side, {"cell": h * 0.8, "error": error or 0.12 * h, "safety": 3.0,
                                         "sharp_deg": 48.0, "max_triangles": 4000, "foils": foils.entries,
                                         "root": root}))
    return out


def hinge_piece(surface, ctrl, side):
    """What model3d.hinge needs of a piece: its hinge line (root to tip, on
    the camber line), a trailing-edge point, its centre and the upper
    surface's normal - mirrored for the left one."""
    def point(eta, frac):
        le, chord, tw, foil = surface.station(eta)
        c, u = surface.frame(eta)
        return le + c * frac * chord + u * foil.camber(frac) * chord, c, u, le, chord
    leading = ctrl.channel == "lef"
    if leading:  # hinged behind the device, the leading edge the part that moves down
        frac0, frac1 = ctrl.chord_fraction(ctrl.eta0), ctrl.chord_fraction(ctrl.eta1)
    else:
        frac0 = ctrl.pivot if ctrl.all_moving else 1.0 - ctrl.chord_fraction(ctrl.eta0)
        frac1 = ctrl.pivot if ctrl.all_moving else 1.0 - ctrl.chord_fraction(ctrl.eta1)
    p0, _, _, _, _ = point(ctrl.eta0, frac0)
    p1, _, _, _, _ = point(ctrl.eta1, frac1)
    em = 0.5 * (ctrl.eta0 + ctrl.eta1)
    fm = 0.5 * (frac0 + frac1)
    centre, c, u, le, chord = point(em, fm)
    te = le if leading else le + c * chord
    piece = {"hinge": (p0, p1), "te": te, "centre": centre, "u": u, "leading": leading,
             "control": (surface.leading if leading else surface.controls).index(ctrl) + 1, "side": 1 if side >= 0 else -1}
    if side < 0:
        flip = np.array([1.0, -1.0, 1.0])
        piece["hinge"] = (p0 * flip, p1 * flip)
        piece["te"], piece["centre"], piece["u"] = te * flip, centre * flip, u * flip
    return piece
