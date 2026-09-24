"""The methods, each against something it must reproduce: an analytic result,
a published value, or a symmetry. Fast; no platform needed."""
import math
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from hangar.aero.model import AeroModel
from hangar.aero.section import PolarSet, SectionPolar, flap_tau, thin_airfoil
from hangar.aero.vlm import VLM, Lattice
from hangar.geometry import Aircraft
from hangar.geometry import airfoil as af


def wing(ar, taper=1.0, sweep_deg=0.0, dihedral_deg=0.0, airfoil="plate", ns=30, nc=8):
    b = 10.0
    S = b * b / ar
    cr = 2 * S / (b * (1 + taper))
    tip_z = (b / 2) * math.tan(math.radians(dihedral_deg))
    xs = [-0.25 * cr, -0.25 * cr * taper + (b / 2) * math.tan(math.radians(sweep_deg))]
    spec = {"aircraft": {"name": "w"},
            "surface": [{"name": "wing", "kind": "wing", "airfoil": airfoil, "spanwise_panels": ns, "chordwise_panels": nc,
                         "sections": [{"le": [xs[0], 0.0, 0.0], "chord": cr}, {"le": [xs[1], b / 2, tip_z], "chord": cr * taper}]}],
            "reference": {"aero_point": [0.0, 0.0, 0.0]}}
    return Aircraft(spec)


def lifting_line(ar, n=24, a0=2 * math.pi):
    """Prandtl's lifting line for a rectangular wing by Glauert's Fourier
    series: (CL_alpha, Cl_p). Odd modes carry the symmetric load, even modes
    the rolling one; collocation on one half span."""
    mu = a0 / (4 * ar)                                   # a0 c / 4b
    th = np.arange(1, n + 1) * (math.pi / 2) / n
    k = np.arange(1, 2 * n, 2)
    a1 = np.linalg.solve(np.sin(np.outer(th, k)) * (k * mu + np.sin(th)[:, None]), mu * np.sin(th))[0]
    th = np.arange(1, n + 1) * (math.pi / 2) / (n + 1)
    k = np.arange(2, 2 * n + 1, 2)
    a2 = np.linalg.solve(np.sin(np.outer(th, k)) * (k * mu + np.sin(th)[:, None]), -mu * np.cos(th) * np.sin(th))[0]
    return math.pi * ar * a1, math.pi * ar / 4 * a2


class Airfoils(unittest.TestCase):
    def test_naca4(self):
        f = af.naca4("0012")
        self.assertAlmostEqual(f.thickness_ratio, 0.12, delta=0.001)
        self.assertAlmostEqual(f.x_max_thickness, 0.30, delta=0.02)
        self.assertAlmostEqual(f.area, 0.685 * 0.12, delta=0.002)   # 4-digit sections: 0.685 t c
        g = af.naca4("2412")
        self.assertAlmostEqual(g.max_camber, 0.02, delta=0.0005)
        self.assertAlmostEqual(g.x_max_camber, 0.40, delta=0.02)

    def test_naca5_and_coordinates(self):
        f = af.naca5("23012")
        self.assertAlmostEqual(f.thickness_ratio, 0.12, delta=0.001)
        self.assertAlmostEqual(f.x_max_camber, 0.15, delta=0.03)      # the 230 mean line's maximum
        back = af.from_coordinates("copy", f.coordinates(121))
        x = np.linspace(0.02, 0.98, 30)
        self.assertLess(np.max(np.abs(back.camber(x) - f.camber(x))), 5e-4)


class Sections(unittest.TestCase):
    def test_thin_airfoil_theory(self):
        # Abbott & von Doenhoff: NACA 2412 zero-lift angle about -2.1 deg, cm c/4 about -0.05
        a0, cm = thin_airfoil(af.naca4("2412"))
        self.assertAlmostEqual(math.degrees(a0), -2.08, delta=0.05)
        self.assertAlmostEqual(cm, -0.053, delta=0.004)
        self.assertAlmostEqual(flap_tau(0.25), 0.609, delta=0.002)     # Glauert, 25 % chord flap

    def test_polar_is_physical_all_round(self):
        p = SectionPolar(af.naca4("2412"), re=5e6, aspect_ratio=7.0, flap_chord=0.3)
        for delta in (0.0, math.radians(20), math.radians(-15)):
            a = np.radians(np.linspace(-180, 180, 1441))
            cl, cd, cm = p.evaluate(a, delta)
            self.assertTrue(np.all(np.isfinite(cl)) and np.all(cd > 0))
            self.assertLess(np.max(np.abs(np.diff(cl))), 0.2)      # no jumps between 0.25 deg steps
            self.assertLess(np.max(np.abs(np.diff(cm))), 0.1)
        cl, _, _ = p.evaluate(np.radians(np.linspace(-5, 25, 301)))
        self.assertGreater(cl.max(), 1.5)
        self.assertLess(cl.max(), 1.9)
        self.assertAlmostEqual(float(p.evaluate(math.pi)[0]), 0.0, delta=0.05)  # tail first, zero angle

    def test_polar_set_matches_single(self):
        ps = [SectionPolar(af.naca4("2412"), re=3e6, flap_chord=0.25), SectionPolar(af.naca4("0010"), re=1e6)]
        s = PolarSet(ps)
        a = np.radians(np.linspace(-180, 180, 91))[:, None] * np.ones(2)
        d = np.radians([12.0, 0.0])
        v = s.evaluate(a, d)
        for i, p in enumerate(ps):
            r = p.evaluate(a[:, i], d[i])
            for k in range(3):
                self.assertLess(np.max(np.abs(v[k][:, i] - r[k])), 1e-12)


class Lattice_(unittest.TestCase):
    def coeffs(self, a, vlm, alpha, beta=0.0, p=0.0):
        v = np.array([math.cos(alpha) * math.cos(beta), -math.sin(beta), math.sin(alpha) * math.cos(beta)])
        w = np.array([-2 * p / a.b, 0.0, 0.0])
        g = vlm.circulation(v, w)
        _, F, M = vlm.forces(g, v, w)
        lift = np.array([-math.sin(alpha), 0.0, math.cos(alpha)])
        return F @ lift / (0.5 * a.S), F @ v / (0.5 * a.S), -M[0] / (0.5 * a.S * a.b), F[1] / (0.5 * a.S)

    def test_lift_slope_and_efficiency(self):
        # rectangular AR 6: lifting-surface theory gives about 4.25/rad (the
        # lifting line 4.53); span efficiency close to 1
        a = wing(6.0)
        vlm = VLM(Lattice(a))
        cl, cdi, _, _ = self.coeffs(a, vlm, math.radians(4))
        self.assertAlmostEqual(cl / math.radians(4), 4.25, delta=0.12)
        self.assertGreater(cl**2 / (math.pi * 6 * cdi), 0.95)

    def test_sweep_lowers_the_slope(self):
        # DATCOM: 2 pi A / (2 + sqrt(A^2 (1 + tan^2 L) + 4)) = 3.14/rad at AR 4, 45 deg
        a = wing(4.0, sweep_deg=45.0)
        cl, _, _, _ = self.coeffs(a, VLM(Lattice(a)), math.radians(4))
        self.assertAlmostEqual(cl / math.radians(4), 3.14, delta=0.2)

    def test_lifting_line_limit(self):
        # at high aspect ratio the lattice must approach Prandtl's lifting line
        # (solved independently below); at low aspect ratio it must fall under
        # it, the lifting-surface correction - more so for the rolling load,
        # whose antisymmetric loading acts like a wing of half the aspect ratio
        ratios = {}
        for ar in (6.0, 20.0):
            a = wing(ar, ns=40)
            vlm = VLM(Lattice(a))
            cla = self.coeffs(a, vlm, math.radians(4))[0] / math.radians(4)
            clp = self.coeffs(a, vlm, 0.0, p=0.01)[2] / 0.01
            ll_cla, ll_clp = lifting_line(ar)
            ratios[ar] = (cla / ll_cla, clp / ll_clp)
        self.assertAlmostEqual(ratios[20.0][0], 1.0, delta=0.03)
        self.assertAlmostEqual(ratios[20.0][1], 1.0, delta=0.07)
        self.assertTrue(0.90 < ratios[6.0][0] < 0.97, ratios)
        self.assertTrue(0.80 < ratios[6.0][1] < ratios[6.0][0], ratios)

    def test_symmetry(self):
        a = wing(7.0, dihedral_deg=5.0)
        cl, cdi, roll, side = self.coeffs(a, VLM(Lattice(a)), math.radians(5))
        self.assertLess(abs(roll), 1e-10)
        self.assertLess(abs(side), 1e-10)


class Fighters(unittest.TestCase):
    def test_polhamus_delta(self):
        # a sharp 60 deg delta (aspect ratio 2.31): Polhamus' suction analogy,
        # K_p sin a cos^2 a + K_v sin^2 a cos a with K_p 2.45 and K_v 3.25,
        # which wind-tunnel data follow until the vortex bursts (about 20 deg)
        spec = {"aircraft": {"name": "delta"}, "analysis": {"speed": 60.0},
                "surface": [{"name": "wing", "kind": "wing", "airfoil": "plate", "spanwise_panels": 24, "chordwise_panels": 10,
                             "sections": [{"le": [0.0, 0.0, 0.0], "chord": 1.0}, {"le": [0.999, 0.57677, 0.0], "chord": 0.001}]}],
                "reference": {"aero_point": [0.6667, 0.0, 0.0]}}
        m = AeroModel(Aircraft(spec))
        for deg in (10.0, 20.0):
            a = math.radians(deg)
            polhamus = 2.45 * math.sin(a) * math.cos(a) ** 2 + 3.25 * math.sin(a) ** 2 * math.cos(a)
            self.assertAlmostEqual(m.evaluate(a)["CL"] / polhamus, 1.0, delta=0.12, msg="alpha %g" % deg)

    def test_each_control_stops_at_its_own_limits(self):
        # a canard delta: elevons +-25 deg, the canard (gain -1) 50 deg leading
        # edge down; the pitch channel runs as far as the canard follows it
        a = Aircraft.load(repo("aircraft/typhoon/typhoon.toml"))
        self.assertEqual(a.channel_limits("elevator"), (-25.0, 50.0))
        L = Lattice(a)
        own = np.degrees(L.deflections({"elevator": math.radians(50.0)}))
        for p, d in zip(L.pieces, own):
            self.assertAlmostEqual(d, -50.0 if p["control"].name == "canard" else 25.0 if p["control"].channel == "elevator" else 0.0)

    def test_all_moving_surface_turns_its_whole_section(self):
        # an all-moving wing turned 30 deg leading edge down at 30 deg flies as
        # an unturned one at 0 (the lattice's linear deflection would leave it
        # some 7 deg of incidence)
        spec = {"aircraft": {"name": "slab"}, "analysis": {"speed": 60.0},
                "surface": [{"name": "wing", "kind": "wing", "airfoil": "naca0006",
                             "sections": [{"le": [0.0, 0.0, 0.0], "chord": 1.0}, {"le": [0.0, 2.5, 0.0], "chord": 1.0}],
                             "controls": [{"name": "slab", "channel": "elevator", "span": [0.0, 1.0], "chord_fraction": 1.0,
                                           "limits": [-40, 40]}]}],
                "reference": {"aero_point": [0.25, 0.0, 0.0]}}
        m = AeroModel(Aircraft(spec))
        turned = m.evaluate(math.radians(30.0), controls={"elevator": math.radians(-30.0)})["CL"]
        self.assertLess(abs(turned), 0.05)

    def test_turbofan_lapse_in_the_stratosphere(self):
        # above 11 km the temperature holds: at a fixed Mach number thrust
        # goes as the density, continuous at the tropopause
        from hangar.propulsion import _isa, turbofan_lapse

        def thrust(h):
            return turbofan_lapse(0.9, h, 1.1, wet=True)

        def density(h):
            t, p = _isa(h)
            return p / t
        self.assertAlmostEqual(thrust(10999.0) / thrust(11001.0), 1.0, delta=0.001)
        self.assertAlmostEqual(thrust(18000.0) / thrust(13000.0), density(18000.0) / density(13000.0), delta=1e-6)


class Aircraft_(unittest.TestCase):
    def test_c172_derivative_signs_and_sizes(self):
        a = Aircraft.load(repo("aircraft/c172/c172.toml"))
        m = AeroModel(a)
        c0, c1 = m.evaluate(math.radians(2)), m.evaluate(math.radians(4))
        cla = (c1["CL"] - c0["CL"]) / math.radians(2)
        cma = (c1["Cm"] - c0["Cm"]) / math.radians(2)
        cb = m.evaluate(math.radians(3), math.radians(4))
        self.assertTrue(4.5 < cla < 5.8, cla)
        self.assertLess(cma, -0.8)                       # stable in pitch
        self.assertGreater(cb["Cn"], 0.0)                # weathercock stable
        self.assertLess(cb["Cl"], 0.0)                   # dihedral effect
        self.assertLess(cb["CY"], 0.0)
        # stall: a peak between 14 and 22 deg, lift kept afterwards (not a cliff to zero)
        cls = [m.evaluate(math.radians(x))["CL"] for x in range(10, 31, 2)]
        i = int(np.argmax(cls))
        self.assertTrue(14 <= 10 + 2 * i <= 22)
        self.assertGreater(cls[-1], 0.6 * max(cls))


def repo(rel):
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "..", "..", rel)


class Writer(unittest.TestCase):
    def test_jsbsim_file_parses(self):
        from hangar import jsbsim
        from hangar.aero import tables as T
        from hangar.mass import MassModel
        a = Aircraft.load(repo("aircraft/c172/c172.toml"))
        tabs = T.build(AeroModel(a), alpha=[-180, -90, -20, -10, 0, 10, 20, 90, 180], beta=[-90, -10, 0, 10, 90])
        root = ET.fromstring(jsbsim.aircraft_xml(a, tabs, MassModel(a), [("e", "p")]))
        axes = {ax.get("name") for ax in root.find("aerodynamics").findall("axis")}
        self.assertEqual(axes, {"DRAG", "SIDE", "LIFT", "ROLL", "PITCH", "YAW"})
        self.assertIsNotNone(root.find("mass_balance/ixx"))
        self.assertEqual(len(root.findall("ground_reactions/contact[@type='BOGEY']")), 3)


class Contacts(unittest.TestCase):
    def test_apparent_mass(self):
        # at the centre of gravity a push meets the whole mass; on the roll
        # axis at r, 1 / (1/m + r^2 / Ixx) (Ixx = 4, r = 2: 1 / (1/10 + 1) = 0.909)
        from hangar.jsbsim import apparent_mass
        J = np.diag([4.0, 6.0, 9.0])
        self.assertAlmostEqual(apparent_mass(np.zeros(3), 10.0, J), 10.0, places=9)
        self.assertAlmostEqual(apparent_mass(np.array([0.0, 2.0, 0.0]), 10.0, J), 1.0 / (0.1 + 4.0 / 4.0), places=9)

    def test_structure_contacts_cover_the_airframe(self):
        # the points a crash meets first, symmetric, and every one soft enough
        # for the step: its own mode at most STRUCTURE_OMEGA
        from hangar import jsbsim
        from hangar.mass import MassModel
        for name, expected in (("c172", [(7.25, 0.0, 2.25), (7.1, 1.73, 0.96), (1.88, 5.49, 1.69), (-0.96, 0.0, -0.28)]),
                               ("skua", [(2.3, 0.55, 0.45), (2.36, 0.55, 0.15), (0.82, 2.0, 0.23), (0.0, 0.0, 0.0)])):
            a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
            mm = MassModel(a)
            contacts = jsbsim.structure_contacts(a, mm)
            pts = np.array([p for _, p, _, _ in contacts])
            with self.subTest(design=name):
                self.assertGreaterEqual(len(contacts), 6)
                for q in expected:  # fin top, tail or boom end, wing tip, nose or propeller
                    self.assertLess(np.min(np.linalg.norm(pts - np.array(q), axis=1)), 0.05, q)
                mirrored = pts * np.array([1.0, -1.0, 1.0])
                for q in mirrored:
                    self.assertLess(np.min(np.linalg.norm(pts - q, axis=1)), 1e-6)
                e = mm.empty()
                J = np.array([[e["ixx"], -e["ixy"], -e["ixz"]], [-e["ixy"], e["iyy"], -e["iyz"]], [-e["ixz"], -e["iyz"], e["izz"]]])
                for _, p, k, c in contacts:
                    m_eff = jsbsim.apparent_mass(p - e["cg"], e["mass"], J)
                    self.assertLessEqual(math.sqrt(k / m_eff), jsbsim.STRUCTURE_OMEGA + 1e-9)
                    self.assertLessEqual(c / m_eff * (1.0 / 120.0), 0.3)  # damping well inside the step's limit


class Model3D(unittest.TestCase):
    def test_control_surfaces_hinge_the_way_jsbsim_deflects(self):
        # every piece, turned about its hinge axis by what +0.25 rad of each of
        # its channels means for it, moves its trailing edge the way JSBSim's
        # positive value means (design frame: x aft, y right, z up): elevator
        # and flaps down, the left aileron down and the right one up, the
        # rudder left - also for twin fins, all-moving tails that roll too,
        # and flaperons
        from hangar import model3d
        for name, expect in (("c172", 7), ("skua", 8), ("f16c", 5)):
            a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
            seen = 0
            for s in a.surfaces:
                _, _, pieces = model3d.display_skin(s)
                for piece in pieces:
                    p0, axis, g, mixed = model3d.hinge(s, piece)
                    ctrl = s.controls[piece["control"] - 1]
                    r = piece["te"] - p0
                    left = piece["centre"][1] < 0
                    for ch in ctrl.channels:
                        th = 0.25 * (g if ch == ctrl.channel else mixed[ch])
                        # Rodrigues: r turned about axis by th
                        moved = (r * math.cos(th) + np.cross(axis, r) * math.sin(th)
                                 + axis * np.dot(axis, r) * (1 - math.cos(th))) - r
                        with self.subTest(design=name, surface=s.name, control=ctrl.name, channel=ch, left=left):
                            self.assertGreater(g, 0.0)
                            if ch in ("elevator", "flap"):
                                self.assertLess(moved[2], 0.0)
                            elif ch == "aileron":
                                self.assertLess(moved[2] if left else -moved[2], 0.0)
                            else:
                                self.assertLess(moved[1], 0.0)
                    seen += 1
            self.assertEqual(seen, expect)

    def test_glb_names_the_moving_parts(self):
        import json
        import os
        import struct
        import tempfile
        from hangar import model3d
        from hangar.shape import meshkit
        a = Aircraft.load(repo("aircraft/skua/skua.toml"))
        surfaces = ["fsim:aileron"] * 2 + ["fsim:elevator"] * 2 + ["fsim:flaps"] * 2 + ["fsim:rudder"] * 2
        # the primitive model, and the solid one when the mesher is built: that
        # also has its propeller, which the viewer turns at the engine's rpm,
        # and its gear's moving parts - the oleos, the nose wheel's steering and
        # the wheels, by their places among the wheeled units (nose, left, right)
        wheels = ["fsim:oleo:0", "fsim:oleo:1", "fsim:oleo:2", "fsim:steer:0", "fsim:wheel:0", "fsim:wheel:1", "fsim:wheel:2"]
        for solid, expect in ((False, surfaces), (True, sorted(surfaces + ["fsim:propeller:0"] + wheels))):
            if solid and meshkit.library() is None:
                continue
            report = {}
            with tempfile.TemporaryDirectory() as d:
                path = model3d.write_glb(a, os.path.join(d, "skua.glb"), origin=np.zeros(3), report=report, solid=solid)
                with open(path, "rb") as f:
                    magic, version, total = struct.unpack("<III", f.read(12))
                    length, kind = struct.unpack("<II", f.read(8))
                    doc = json.loads(f.read(length))
                    bin_length, bin_kind = struct.unpack("<II", f.read(8))
            with self.subTest(solid=solid):
                self.assertEqual((magic, version, kind), (0x46546C67, 2, 0x4E4F534A))
                # the buffer is the whole binary chunk, on a 4-byte boundary:
                # a loader may refuse anything else (vsgXchange does)
                self.assertEqual((bin_kind, doc["buffers"][0]["byteLength"], bin_length % 4), (0x004E4942, bin_length, 0))
                names = [n["name"] for n in doc["nodes"] if n["name"].startswith("fsim:")]
                # an oleo's gain and a wheel's radius follow the leg's shape
                self.assertEqual(sorted(":".join(n.split(":")[:3]) if n.startswith(("fsim:oleo:", "fsim:wheel:")) else n
                                        for n in names), expect)
                for n in doc["nodes"]:
                    if n["name"].startswith("fsim:"):
                        self.assertEqual(len(n["translation"]), 3)
                        self.assertAlmostEqual(float(np.linalg.norm(n["rotation"])), 1.0, places=6)
                if solid:  # one closed solid, and every moving part one too
                    af = report["airframe"]
                    self.assertEqual((af["boundary_edges"], af["nonmanifold_edges"], af["components"]), (0, 0, 1))
                    for p in report["pieces"] + report["gear"]:
                        self.assertEqual((p["boundary_edges"], p["components"]), (0, 1), p["label"])

    def test_gear_folds_into_its_bay(self):
        # the F-16's legs stow inside the airframe (its intake, belly and wing
        # roots); each bay has a pair of doors, each hinged on its own outer
        # edge along the leg's (fore and aft) swing, opening down and out; and
        # no door ever touches a leg - open while the leg swings, closed over
        # it stowed
        from hangar import model3d
        from hangar.shape import airframe as sh
        from hangar.shape import gear as sg
        from hangar.shape import meshkit
        if meshkit.library() is None:
            self.skipTest("hangar_meshkit is not built")
        a = Aircraft.load(repo("aircraft/f16c/f16c.toml"))
        plan = []
        sh.airframe(a, gear=plan)
        self.assertEqual(sorted(p["leg"].name for p in plan), ["Left Main Gear", "Nose Gear", "Right Main Gear"])
        panels = []
        for p in plan:
            with self.subTest(leg=p["leg"].name):
                self.assertLess(p["protrusion"], 0.03)
                self.assertEqual(len(p["doors"]), 2)
                for d in p["doors"]:
                    box = d["scene"]["root"]["a"]["children"][1]  # the skin within it, a shell
                    (cx, cy, _), (hx, hy, _) = box["centre"], box["half"]
                    x0, y0, x1, y1 = cx - hx, cy - hy, cx + hx, cy + hy
                    # the hinge along x, on one of the door's long edges
                    self.assertEqual(abs(d["axis"][0]), 1.0)
                    self.assertLess(min(abs(d["hinge"][1] - y0), abs(d["hinge"][1] - y1)), 0.05)
                    # opening turns its free edge down, away from the bay
                    free = np.array([0.5 * (x0 + x1), y1 if abs(d["hinge"][1] - y0) < abs(d["hinge"][1] - y1) else y0,
                                     d["hinge"][2]])
                    moved = d["hinge"] + sg._rotation(d["axis"], d["deg"]) @ (free - d["hinge"])
                    self.assertLess(moved[2], d["hinge"][2] - 0.5 * abs(y1 - y0))
                    panels.append((d, meshkit.build(d["scene"])))
        for c in model3d.door_clearance([p["leg"] for p in plan], panels):
            with self.subTest(door=c["door"], leg=c["leg"]):
                self.assertGreater(c["open_m"], 0.0)
                self.assertGreater(c["closed_m"], 0.0)

    def test_leading_edge_flaps_turn_down(self):
        # a positive turn of a leading-edge device's node moves its leading
        # edge down on both sides
        from hangar import model3d
        from hangar.shape import airframe as sh
        a = Aircraft.load(repo("aircraft/f16c/f16c.toml"))
        wing = next(s for s in a.surfaces if s.name == "wing")
        self.assertEqual(len(wing.leading), 1)
        for side in (1, -1):
            piece = sh.hinge_piece(wing, wing.leading[0], side)
            p0, axis, _, _ = model3d.leading_hinge(piece)
            r = piece["te"] - p0  # the leading edge, ahead of the hinge
            th = 0.25
            moved = r * math.cos(th) + np.cross(axis, r) * math.sin(th) + axis * np.dot(axis, r) * (1 - math.cos(th)) - r
            with self.subTest(side=side):
                self.assertLess(moved[2], 0.0)

    def test_leading_edge_flaps_follow_the_flight_controls(self):
        # the flight controls move the flaps on their schedule (fcs/lef-pos-deg,
        # which the simulation reports), within the widest stops; each flap's
        # node follows that within its own
        from hangar import jsbsim
        a = Aircraft.load(repo("aircraft/su57/su57.toml"))
        root = ET.fromstring("<fdm>%s</fdm>" % jsbsim.flight_control_xml(a))
        ch = root.find("flight_control/channel[@name='Leading-Edge Flaps']")
        self.assertIsNotNone(ch)
        self.assertEqual(ch.find("actuator/output").text, "fcs/lef-pos-deg")
        clip = ch.find("fcs_function/clipto")
        self.assertEqual((float(clip.find("min").text), float(clip.find("max").text)), (-5.0, 30.0))
        values = [float(v.text) for v in ch.iter("value")]
        self.assertEqual(values, [1.38, -9.05, 1.45])
        # no flaps, no channel; two schedules on one aircraft are refused
        c172 = Aircraft.load(repo("aircraft/c172/c172.toml"))
        self.assertNotIn("fcs/lef-pos-deg", jsbsim.flight_control_xml(c172))
        import tomllib
        with open(repo("aircraft/rafale/rafale.toml"), "rb") as f:
            spec = tomllib.load(f)
        wing = next(s for s in spec["surface"] if s.get("leading"))
        wing["leading"][1]["schedule"] = [1.0, 5.0, 0.0]
        with self.assertRaisesRegex(ValueError, "one schedule"):
            Aircraft(spec, repo("aircraft/rafale/rafale.toml"))


if __name__ == "__main__":
    unittest.main()
