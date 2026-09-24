"""The viewer's stand-ins: stock JSBSim aircraft drawn with a model designed
here.

A design names the stock aircraft it stands in for ([aircraft] stands_in_for
= ["f16"]). `python -m hangar register` parks each of those in the simulator,
finds where its main wheels touch relative to its centre of gravity, and
writes aircraft/models.txt: per stock type, the design and how far to move
the design's model (its origin is the design's empty CG) so its main wheels
stand where the stock aircraft's do. The viewer reads that file (world::
VehicleVisuals); needs the fsim package.
"""
import os

import numpy as np

from .geometry.aircraft import Aircraft
from .mass import MassModel

HEADER = """# The viewer's models for stock JSBSim aircraft - written by: fsim hangar register
# <stock type> <design> <forward> <right> <down>: the viewer draws jsbsim:<stock type> with
# aircraft/<design>/<design>.glb, moved that far (m, the aircraft's body axes) so the model's
# main wheels stand where the stock aircraft's do.
"""
IN = 0.0254


def stand_ins(root):
    """[(design, stock types)] of the designs under root that name any."""
    out = []
    for name in sorted(os.listdir(root)):
        toml = os.path.join(root, name, name + ".toml")
        if not os.path.isfile(toml):
            continue
        a = Aircraft.load(toml)
        stocks = a.spec.get("aircraft", {}).get("stands_in_for", [])
        if stocks:
            out.append((a, list(stocks)))
    return out


def design_mains(a):
    """The design's main wheels' contact point relative to its empty CG (the
    model's origin), in body axes (forward, right, down)."""
    cg = np.asarray(MassModel(a).empty()["cg"], float)
    mains = [np.asarray(p, float) for g in a.gear for _, p in g.positions() if abs(p[1]) > 0.3]
    if not mains:
        raise ValueError("%s has no main wheels off the centre line" % a.name)
    m = np.mean([p * [1.0, 0.0, 1.0] for p in mains], axis=0) - cg  # the pair's middle
    return np.array([-m[0], 0.0, -m[2]])


def stock_mains(world, stock):
    """A stock aircraft parked: its main wheels' contact point relative to its
    CG (body axes), from JSBSim's gear and CG (structural frame, inches)."""
    v = world.create_vehicle("register-" + stock, "jsbsim:" + stock, latitude_deg=0.0, longitude_deg=0.0,
                             altitude_msl_m=0.0, heading_deg=0.0, airspeed_ms=0.0, on_ground=True)
    v.command_actuator(throttle=0.0, brake_left=1.0, brake_right=1.0, gear_down=1.0)
    world.step(int(3.0 / world.step_seconds))
    cg = np.array([v.get_property("inertia/cg-%s-in" % k) for k in "xyz"]) * IN
    mains = []
    for i in range(64):
        try:
            p = np.array([v.get_property("gear/unit[%d]/%s-position" % (i, k)) for k in "xyz"]) * IN
        except Exception:
            break
        if abs(p[1]) > 0.3 and v.get_property("gear/unit[%d]/WOW" % i):
            mains.append(p)
    v.remove()
    if not mains:
        raise ValueError("jsbsim:%s has no main wheels on the ground when parked" % stock)
    m = np.mean([p * [1.0, 0.0, 1.0] for p in mains], axis=0) - cg * [1.0, 0.0, 1.0]
    return np.array([-m[0], 0.0, -m[2]])


def register(root, log=print):
    """Writes root/models.txt; returns its lines [(stock, design, offset)]."""
    import fsim
    rows = []
    pairs = stand_ins(root)
    if pairs:
        world = fsim.World("hangar-register", publish=False)
        for a, stocks in pairs:
            ours = design_mains(a)
            for stock in stocks:
                offset = stock_mains(world, stock) - ours
                rows.append((stock, a.name, offset))
                log("  jsbsim:%-8s drawn with %-10s moved %+.2f forward, %+.2f down" % (stock, a.name, offset[0], offset[2]))
    with open(os.path.join(root, "models.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(HEADER)
        for stock, design, o in rows:
            f.write("%s %s %.3f %.3f %.3f\n" % (stock, design, o[0], o[1], o[2]))
    return rows
