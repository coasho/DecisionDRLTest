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


if __name__ == "__main__":
    unittest.main()
