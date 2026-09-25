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

    def test_centres_of_pressure(self):
        # where on its chord a strip's lift acts: at the quarter chord on a
        # straight wing of aspect ratio 20 (but at its tips); on a swept one
        # aft of it at the root and ahead of it at the tip, Kuchemann's
        # centre and tip effects
        for ar, taper, sweep, root, tip in ((20.0, 1.0, 0.0, (-0.01, 0.01), (-0.1, -0.02)),
                                            (4.0, 0.4, 45.0, (0.05, 0.12), (-0.2, -0.1))):
            vlm = VLM(Lattice(wing(ar, taper=taper, sweep_deg=sweep)))
            vlm.select(math.radians(4.0))
            y = vlm.lattice.c4[:, 1]
            right = np.flatnonzero(y > 0.0)
            right = right[np.argsort(y[right])]
            with self.subTest(sweep=sweep):
                self.assertTrue(root[0] < vlm.x_ac[right[0]] < root[1], vlm.x_ac[right[0]])
                self.assertTrue(tip[0] < vlm.x_ac[right[-1]] < tip[1], vlm.x_ac[right[-1]])
                if not sweep:
                    self.assertLess(abs(vlm.x_ac[right[len(right) // 2]]), 0.01)


class InducedDrag(unittest.TestCase):
    """[analysis] induced_drag = "trefftz": the strips' induced drag from the
    Trefftz plane, not from the tilt of each strip's lift."""

    @staticmethod
    def model(mode, airfoil="naca0006", ar=10.0, taper=1.0):
        a = wing(ar, taper=taper, airfoil=airfoil, ns=24, nc=6)
        a.spec["analysis"] = {"speed": 50.0, "induced_drag": mode,
                              "airfoil": {"wing": {"cd0": 0.0, "k_drag": 0.0}}}   # the induced drag alone
        return AeroModel(Aircraft(a.spec))

    def test_trefftz_plane_drag_of_an_elliptic_loading(self):
        # an elliptic loading laid on a rectangle's strips: pi Gamma0^2 / 8
        from hangar.aero.model import trefftz_correction
        m = self.model("trefftz")
        L = m.lat
        y = L.c4[:, 1]
        g = np.sqrt(np.clip(1.0 - (2.0 * y / 10.0) ** 2, 0.0, 1.0))
        n = L.n_strips
        dF = trefftz_correction(L, m.group, np.tile([1.0, 0.0, 0.0], (n, 1)), np.array([1.0, 0.0, 0.0]), np.ones(n),
                                g * L.width, np.tile([0.0, 0.0, 1.0], (n, 1)), np.ones(n))
        self.assertAlmostEqual(dF[:, 0].sum() / (math.pi / 8.0), 1.0, delta=0.01)

    def test_strips_drag_follows_the_trefftz_plane(self):
        # a rectangle of aspect ratio 10: lifting-line theory's e is 0.93-0.95;
        # the strips' own tilt gives 0.78, the Trefftz plane's loading 0.95.
        # The lift and its moment stay the strips' own.
        e = {}
        for mode in ("strips", "trefftz"):
            m = self.model(mode)
            c = m.evaluate(math.radians(4.0))
            e[mode] = c["CL"] ** 2 / (math.pi * 10.0 * c["CD"])
            e[mode + " CL"] = c["CL"]
        self.assertTrue(0.92 < e["trefftz"] < 0.97, e)
        self.assertLess(e["strips"], 0.82)
        self.assertAlmostEqual(e["trefftz CL"], e["strips CL"], delta=1e-12)

    def test_a_swept_wing_keeps_its_span_efficiency(self):
        # 35 deg of sweep: the strips' pieces lie on the quarter-chord line, so
        # their trailing vortices meet in the Trefftz plane at any incidence
        # (laid across the stream from each strip's centre, their stagger
        # made a dipole at every boundary: e 0.57 at CL 0.15, 0.16 at 0.44)
        a = wing(8.5, taper=0.35, sweep_deg=35.0, airfoil="naca0010", ns=24, nc=6)
        a.spec["analysis"] = {"speed": 50.0, "induced_drag": "trefftz", "airfoil": {"wing": {"cd0": 0.0, "k_drag": 0.0}}}
        m = AeroModel(Aircraft(a.spec))
        for deg in (2.0, 4.0, 6.0):
            c = m.evaluate(math.radians(deg))
            self.assertTrue(0.94 < c["CL"] ** 2 / (math.pi * 8.5 * c["CD"]) < 0.99, deg)

    def test_a_drooped_wing_at_incidence_meets_its_mirror(self):
        # an elliptic loading on a drooped (anhedral) wing set at 8 deg: its
        # two halves' root vortices meet on the plane of symmetry, and the
        # drag is pi Gamma0^2 / 8 (twist turned the chords a little sideways,
        # and the halves' roots crossed the plane by millimetres: 10 % low)
        from hangar.aero.model import trefftz_correction
        a = wing(10.0, dihedral_deg=-3.0, airfoil="naca0006", ns=24, nc=6)
        for sec in a.spec["surface"][0]["sections"]:
            sec["twist"] = 8.0
        a.spec["analysis"] = {"speed": 50.0, "induced_drag": "trefftz"}
        m = AeroModel(Aircraft(a.spec))
        L = m.lat
        n = L.n_strips
        g = np.sqrt(np.clip(1.0 - (2.0 * L.c4[:, 1] / 10.0) ** 2, 0.0, 1.0))
        for deg in (0.0, -8.0):
            v = np.array([math.cos(math.radians(deg)), 0.0, math.sin(math.radians(deg))])
            U = np.tile(v, (n, 1))
            ld = np.cross(v[None, :], L.e)
            ld /= np.linalg.norm(ld, axis=1)[:, None]
            un = np.linalg.norm(U - np.einsum("ij,ij->i", U, L.e)[:, None] * L.e, axis=1)
            dF = trefftz_correction(L, m.group, U, v, un, g * L.width * un, ld, np.ones(n))
            self.assertAlmostEqual(float((dF @ v).sum()) / (math.pi / 8.0), 1.0, delta=0.01)

    def test_a_cambered_line_adds_no_induced_drag_of_its_own(self):
        # a 6A section's uniform-load mean line, which the lattice's few
        # chordwise panels resolve poorly: from the tilt it adds drag in
        # proportion to the lift (0.006 CL on this planform); the Trefftz
        # plane's drag follows the loading alone, CL^2 / (pi A e)
        m = self.model("trefftz", airfoil="naca63a409", ar=10.6, taper=0.23)
        for deg in (0.0, 2.0, 4.0):
            c = m.evaluate(math.radians(deg))
            self.assertAlmostEqual(c["CL"] ** 2 / (math.pi * 10.6 * c["CD"]), 0.975, delta=0.03)


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

    def test_strips_carry_the_lattices_moment(self):
        # a 75 deg strake ahead of a 45 deg wing: the strake's downwash moves
        # the wing root's load aft. The strips give the lattice's moment as
        # well as its lift - the neutral point within 0.5 % of the MAC of the
        # lattice's own (Kutta-Joukowski on every bound vortex); with each
        # strip's lift at its quarter chord it was 2.9 % ahead
        spec = {"aircraft": {"name": "double delta"}, "analysis": {"speed": 60.0},
                "surface": [{"name": "wing", "kind": "wing", "airfoil": "plate",
                             "sections": [{"le": [3.0, 0.0, 0.0], "chord": 4.0}, {"le": [7.0, 4.0, 0.0], "chord": 1.0}]},
                            {"name": "strake", "kind": "strake", "airfoil": "plate",
                             "sections": [{"le": [0.0, 0.0, 0.0], "chord": 3.0}, {"le": [3.0, 0.8, 0.0], "chord": 0.05}]}],
                "reference": {"aero_point": [5.0, 0.0, 0.0]}}
        a = Aircraft(spec)
        m = AeroModel(a)
        fit = np.radians(np.linspace(-2.0, 6.0, 9))
        model, lattice = [], []
        for al in fit:
            c = m.evaluate(al)
            model.append((c["CL"], c["Cm"]))
            v = np.array([math.cos(al), 0.0, math.sin(al)])
            m.vlm.select(al)
            _, F, M = m.vlm.forces(m.vlm.circulation(v, np.zeros(3)), v, np.zeros(3))
            lattice.append((F @ np.array([-math.sin(al), 0.0, math.cos(al)]), M[1] / a.c))
        points = [a.aero_point[0] - np.polyfit(fit, np.array(r)[:, 1], 1)[0] / np.polyfit(fit, np.array(r)[:, 0], 1)[0] * a.c
                  for r in (model, lattice)]
        self.assertLess(abs(points[0] - points[1]) / a.c, 0.005, points)

    def test_an_edge_beside_a_body_loses_its_suction(self):
        # Bryson's slender wing-body: an edge d from the axis of a body a
        # wide (at the edge's height) keeps 1 - (a/d)^4 of its suction, and so
        # of its vortex; none inside the body or at its side
        from hangar.aero.model import edge_shielding
        spec = {"aircraft": {"name": "tube"},
                "surface": [{"name": "wing", "kind": "wing", "sections": [{"le": [4.0, 0.0, 0.0], "chord": 2.0},
                                                                         {"le": [4.0, 4.0, 0.0], "chord": 2.0}]}],
                "body": [{"name": "tube", "kind": "fuselage", "stations": [{"x": 0.0, "w": 1.0, "top": 0.5, "bottom": -0.5},
                                                                          {"x": 10.0, "w": 1.0, "top": 0.5, "bottom": -0.5}]}]}
        f = edge_shielding(Aircraft(spec), np.array([[5.0, 0.3, 0.0], [5.0, 0.5, 0.0], [5.0, 1.0, 0.0], [5.0, -2.0, 0.0],
                                                     [5.0, 0.8, 0.3], [5.0, 3.0, 1.0], [11.0, 0.6, 0.0]]))
        np.testing.assert_allclose(f, [0.0, 0.0, 1.0 - 0.5**4, 1.0 - 0.25**4, 1.0 - 0.5**4, 1.0, 1.0], atol=1e-9)

    def test_f16c_pitch_follows_nasa(self):
        # the F-16C as its three-views shape it against NASA TP-1538 (JSBSim's
        # f16), both about NASA's moment reference: the neutral point within
        # 1.5 % of the MAC from the same -2..6 deg slope, the pitching moment
        # within 0.03 to 30 deg and 0.07 at 40. With every strip's lift at its
        # quarter chord and the strakes' vortex lift unshielded by the
        # fuselage, the neutral point was 2.8 % ahead and Cm 0.23 above at 40
        import os
        from hangar.reference import JSBSimAero
        path = repo("third_party/jsbsim/aircraft/f16/f16.xml")
        if not os.path.isfile(path):
            self.skipTest("JSBSim's aircraft are not checked out")
        a = Aircraft.load(repo("aircraft/f16c/f16c.toml"))
        m = AeroModel(a)
        ref = JSBSimAero(path)
        kf, km = ref.S / a.S, ref.S * ref.c / (a.S * a.c)

        def both(deg):
            c, r = m.evaluate(math.radians(deg)), ref.coefficients(deg, 0.0, 0.2)
            return c["CL"], c["Cm"], kf * r["CL"], km * r["Cm"]
        fit = np.linspace(-2.0, 6.0, 9)
        rows = np.array([both(x) for x in fit])
        points = [(a.aero_point[0] - np.polyfit(np.radians(fit), rows[:, i + 1], 1)[0]
                   / np.polyfit(np.radians(fit), rows[:, i], 1)[0] * a.c - a.mac_le[0]) / a.c for i in (0, 2)]
        self.assertLess(abs(points[0] - points[1]), 0.015, points)
        for deg, tol in ((0.0, 0.03), (10.0, 0.03), (20.0, 0.03), (30.0, 0.03), (40.0, 0.07)):
            cl, cm, cl_ref, cm_ref = both(deg)
            self.assertLess(abs(cm - cm_ref), tol, msg="alpha %g: Cm %.3f, NASA %.3f" % (deg, cm, cm_ref))

    def test_post_stall_solution_is_unique(self):
        # past the stall a vortex-lifting wing's induced flow had two
        # solutions - a section washed far down and still in the vortex
        # regime, or a plate hardly washed down at all - and the Rafale's CD
        # jumped by 0.5 between 55 and 60 deg in sideslip, each half on its
        # own; now every solve converges, sideslip either way mirrors, and the
        # coefficients change smoothly
        m = AeroModel(Aircraft.load(repo("aircraft/rafale/rafale.toml")))
        alphas = np.arange(40.0, 74.0, 2.0)
        for beta in (0.0, 30.0):
            c = {}
            for sign in (1.0, -1.0):
                rows = []
                for deg in alphas:
                    e = m.evaluate(math.radians(deg), math.radians(sign * beta), detail=True)
                    self.assertLess(e["detail"]["residual"], 1e-6, msg="alpha %g beta %g" % (deg, sign * beta))
                    rows.append([e[k] for k in ("CD", "CL", "Cm", "CY", "Cl", "Cn")])
                c[sign] = np.array(rows)
            self.assertLess(np.max(np.abs(c[1.0][:, :3] - c[-1.0][:, :3])), 2e-3)
            self.assertLess(np.max(np.abs(c[1.0][:, 3:] + c[-1.0][:, 3:])), 2e-3)
            self.assertLess(np.max(np.abs(np.diff(c[1.0], 2, axis=0))), 0.1)   # per 2 deg: 0.03; 0.69 before

    def test_vortex_regime_comes_in_with_sweep(self):
        # a thin swept section's vortex regime comes in between 25 and 35 deg
        # of leading-edge sweep, not at a line drawn at 35: turned 30 deg
        # nose-down at 20 deg, slabs swept 34 and 36 deg lift alike (the line
        # made the first stall like a two-dimensional section - the F-35A's
        # 34 deg stabilators lost their nose-down power past 20 deg), while
        # one swept 20 deg does stall
        def turned(sweep):
            b, c = 5.0, 1.5
            spec = {"aircraft": {"name": "slab"}, "analysis": {"speed": 60.0},
                    "surface": [{"name": "wing", "kind": "wing", "airfoil": "naca64a004",
                                 "sections": [{"le": [0.0, 0.0, 0.0], "chord": c},
                                              {"le": [2.5 * math.tan(math.radians(sweep)), b / 2, 0.0], "chord": 0.5 * c}],
                                 "controls": [{"name": "slab", "channel": "elevator", "span": [0.0, 1.0], "chord_fraction": 1.0,
                                               "pivot": 0.35, "limits": [-30, 30]}]}],
                    "reference": {"aero_point": [1.0, 0.0, 0.0]}}
            m = AeroModel(Aircraft(spec))
            return float(m.vx["on"].mean()), m.evaluate(math.radians(20.0), controls={"elevator": math.radians(30.0)})["CL"]
        (w20, cl20), (w30, _), (w34, cl34), (w36, cl36) = (turned(s) for s in (20.0, 30.0, 34.0, 36.0))
        self.assertEqual((w20, w36), (0.0, 1.0))
        self.assertAlmostEqual(w30, 0.5, places=6)
        self.assertGreater(w34, 0.95)
        self.assertLess(abs(cl34 - cl36), 0.05 * cl36)
        self.assertLess(cl20, 0.6 * cl36)

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


    def test_high_bypass_thrust_falls_with_speed(self):
        # a high-bypass fan's thrust falls fast with speed (Mattingly): a
        # fifth of it left at Mach 0.8 and 35,000 ft; a bypass ratio of 1 or
        # less keeps the low-bypass law, between them the lapse moves across
        from hangar.propulsion import high_bypass_lapse, turbofan_lapse
        self.assertAlmostEqual(high_bypass_lapse(0.0, 0.0, 1.08), 1.0)
        self.assertLess(high_bypass_lapse(0.4, 0.0, 1.08), 0.8)
        self.assertAlmostEqual(high_bypass_lapse(0.8, 10668.0, 1.08), 0.2, delta=0.02)
        for m, h in ((0.4, 0.0), (0.8, 10668.0)):
            low = turbofan_lapse(m, h, 1.08, bypass=0.3)
            self.assertEqual(turbofan_lapse(m, h, 1.08, bypass=1.0), low)
            self.assertEqual(turbofan_lapse(m, h, 1.08, bypass=6.0), high_bypass_lapse(m, h, 1.08))
            self.assertLess(turbofan_lapse(m, h, 1.08, bypass=2.0), low)
            self.assertGreater(turbofan_lapse(m, h, 1.08, bypass=2.0), high_bypass_lapse(m, h, 1.08))


class Transports(unittest.TestCase):
    def test_korn_kappa_moves_the_drag_divergence(self):
        # Korn (Raymer 12.5.10): M_dd = kappa / cos L - t / cos^2 L - CL / (10
        # cos^3 L); supercritical sections' kappa (0.95) diverge 0.08 / cos L
        # later than conventional ones' (0.87), the default; a subsonic jet's
        # calibration fits it (calibration.toml), a design may give it
        from hangar.aero import mach
        a = wing(8.0, taper=0.3, sweep_deg=30.0, airfoil="naca64a412")
        al = np.arange(-10.0, 21.0, 1.0)
        cl = 5.0 * np.radians(al)[:, None]
        base = {"alpha": al, "beta": np.array([0.0]), "base": {"CL": cl, "CD": 0.02 + 0.05 * cl**2}}
        table = {"mach": np.array([0.0, 0.5, 0.8, 0.9]), "K_L": np.ones(4)}
        conventional = mach._drag(a, base, table)
        self.assertEqual(conventional["korn_kappa"], mach.KORN_KAPPA)
        a.spec["analysis"] = {"korn_kappa": 0.91}
        self.assertEqual(mach.korn_kappa(a), 0.91)
        a.calibration = {"korn_kappa": mach.KORN_SUPERCRITICAL}
        supercritical = mach._drag(a, base, table)
        lam = math.radians(a.wing.sweep_deg(0.25))
        self.assertAlmostEqual(supercritical["M_dd"] - conventional["M_dd"], 0.08 / math.cos(lam), places=9)
        # no wave drag below the critical Mach number, more of it at 0.9 the earlier the divergence
        self.assertLess(supercritical["dCD0"][3], conventional["dCD0"][3])

    def test_calibration_fits_kappa_to_the_top_speed(self):
        # a subsonic jet's top speed rises with its drag divergence: the fit
        # bisects kappa to the target, or keeps the nearer end out of reach
        from hangar.pipeline import fit_kappa
        top = lambda k: 0.60 + 3.0 * (k - 0.87)  # noqa: E731 - Mach 0.60 conventional, 0.84 supercritical
        kappa, m = fit_kappa(top, 0.78)
        self.assertAlmostEqual(m, 0.78, delta=0.005)
        self.assertAlmostEqual(kappa, 0.87 + 0.18 / 3.0, delta=0.002)
        self.assertEqual(fit_kappa(top, 0.90), (0.95, top(0.95)))
        self.assertEqual(fit_kappa(top, 0.50), (0.87, top(0.87)))

    def test_fighter_tests_fit_a_transport(self):
        # a fighter keeps its tests; a transport flown through fly-by-wire
        # turns at Mach 0.6 within its load limit and rolls long enough to
        # pass 90 deg; a jet on direct controls climbs faster than a propeller
        from hangar import flight
        for opts in ({"n_max": 9.0, "roll_rate_deg_s": 308.0}, {"n_max": 7.5, "roll_rate_deg_s": 220.0}):
            self.assertEqual(flight.transport_plan(opts, 2.0),
                             {"turn_mach": 0.9, "loads": (3.0, 5.0, 6.0, 7.0, 8.0, 9.0), "roll_seconds": 3.0})
            self.assertEqual(flight.transport_plan(opts, 2.0, quick=True)["loads"], (4.0, 6.0, 8.0))
        plan = flight.transport_plan({"n_max": 2.5, "roll_rate_deg_s": 35.0}, 0.82)
        self.assertEqual(plan["turn_mach"], 0.6)
        self.assertEqual(plan["loads"], (1.5, 2.0, 2.5))
        self.assertGreater(plan["roll_seconds"] * 35.0, 90.0 * 1.5)
        self.assertEqual(flight.JET_CLIMB_FACTORS[:len(flight.CLIMB_FACTORS)], flight.CLIMB_FACTORS)
        self.assertGreater(max(flight.JET_CLIMB_FACTORS), 2.5)


class FlyByWire(unittest.TestCase):
    @staticmethod
    def tables(kink):
        # a pitching moment unstable in one band of angle of attack (a tail
        # passing through the wing's wake gives one), an elevator of
        # constant power; no sideslip dependence
        al = np.arange(-10.0, 31.0)
        r = np.radians(al)
        cl = 3.5 * r
        cm = -0.2 * r + kink * np.clip((al - 2.0) / 4.0, 0.0, 1.0)
        d = np.array([-20.0, 0.0, 20.0])
        col = lambda v: np.repeat(v[:, None], 3, axis=1)  # noqa: E731
        el = {"deflection": d, "Cm": np.outer(np.ones_like(r), -0.8 * np.radians(d)),
              "CL": np.outer(np.ones_like(r), 0.3 * np.radians(d))}
        return {"alpha": al, "beta": np.array([-4.0, 0.0, 4.0]),
                "base": {"CL": col(cl), "CD": col(0.02 + 0.1 * cl ** 2), "Cm": col(cm)}, "controls": {"elevator": el}}

    def test_moment_compensation_leaves_the_line(self):
        # the elevator the control law adds cancels the moment's departure
        # from its line: the airframe with it is straight in alpha about the
        # CG - here 10 % of the chord ahead of the aero point, where the
        # normal force's own curvature counts too - and one straight about
        # the CG needs none
        from types import SimpleNamespace
        from hangar import fcs
        a = SimpleNamespace(aero_point=[5.0, 0.0, 0.0], c=3.0)
        opt = {"alpha_min_deg": -10.0, "alpha_max_deg": 25.0}
        for kink, cg_x in ((0.0, 5.0), (0.03, 4.7)):
            tabs = self.tables(kink)
            line = fcs.moment_line(tabs, a, [cg_x, 0.0, 0.0], opt)
            self.assertEqual((line["alpha_deg"][0], line["alpha_deg"][-1]), (-10.0, 25.0))
            comp = fcs.moment_compensation(tabs, line, [0.4], math.radians(-25.0), math.radians(25.0))
            r = np.radians(line["alpha_deg"])
            k = np.isin(tabs["alpha"], line["alpha_deg"])
            cn = tabs["base"]["CL"][k, 1] * np.cos(r) + tabs["base"]["CD"][k, 1] * np.sin(r)
            flown = tabs["base"]["Cm"][k, 1] + cn * (cg_x - 5.0) / 3.0 + comp[:, 0] * line["cm_de"]
            with self.subTest(kink=kink):
                np.testing.assert_allclose(flown, np.polyval(np.polyfit(r, flown, 1), r), atol=1e-9)
                self.assertAlmostEqual(np.polyfit(r, flown, 1)[0], line["slope"], places=9)
                self.assertEqual(float(np.max(np.abs(comp))) > 1e-3, kink > 0)

    @staticmethod
    def pitch_channel(name):
        """A design's flight control XML with made-up gains and compensation,
        parsed, and the fly-by-wire tables it was written from."""
        from hangar import fcs, jsbsim
        a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
        line = {"alpha_deg": np.array([-10.0, 0.0, 10.0, 20.0]), "slope": 0.3, "departure": np.zeros(4)}
        fbw = {"qbar_psf": np.array(fcs.QBAR_PSF), "mach": np.array([0.4, 0.9]), "options": fcs.options(a),
               "gains": {k: np.full((len(fcs.QBAR_PSF), 2), 0.1) for k in fcs.GAINS},
               "moment": dict(line, elevator=np.array([[0.01, 0.02], [0.0, 0.0], [-0.03, -0.04], [0.05, 0.06]]))}
        return ET.fromstring("<fdm>%s</fdm>" % jsbsim.flight_control_xml(a, fbw)), fbw

    def test_pitch_channel_sums_the_compensation_and_the_push(self):
        # the pitch channel carries the compensation over angle of attack
        # and Mach number, and the elevator sums it with the push back past
        # the angle-of-attack limits
        root, fbw = self.pitch_channel("gripen")
        comp = root.find(".//fcs_function[@name='fcs/fbw/moment-comp']")
        self.assertEqual([v.text for v in comp.iter("independentVar")], ["aero/alpha-rad", "velocities/mach"])
        rows = [[float(x) for x in line.split()] for line in comp.find(".//tableData").text.strip().splitlines()]
        self.assertEqual(rows[0], [0.4, 0.9])
        np.testing.assert_allclose([r[0] for r in rows[1:]], np.radians([-10.0, 0.0, 10.0, 20.0]), atol=1e-5)
        np.testing.assert_allclose([r[1:] for r in rows[1:]], fbw["moment"]["elevator"], atol=1e-5)
        raw = root.find(".//fcs_function[@name='fcs/fbw/elevator-raw']/function/sum")
        summed = [p.text for p in raw.findall("property")]
        self.assertIn("fcs/fbw/alpha-push", summed)
        self.assertIn("fcs/fbw/moment-comp", summed)
        self.assertIsNotNone(root.find(".//fcs_function[@name='fcs/fbw/alpha-push']"))

    def test_limiter_limits_the_command(self):
        # at an angle-of-attack limit the feedforward acts on the command
        # limited to the load factor the aircraft pulls plus what the angle of
        # attack left gives - not on the command, faded out past the limit:
        # one-sided, about 10 deg of elevator per deg of alpha, that cycled
        # the MiG-29A 24-29 deg against its 26 deg limit. The integrator trims
        # the angle of attack left LIMIT_INTEGRAL times as fast as the load
        # factor, and full aft stick asks for the lift at the limit beyond the
        # gravity reference of the moment, not beyond 1 g
        from hangar import fcs
        root, fbw = self.pitch_channel("mig29a")
        o = fbw["options"]

        def fn(name):
            return root.find(".//fcs_function[@name='fcs/fbw/%s']/function" % name)

        def props(el):
            return [p.text for p in el.iter("property")]
        raw = fn("elevator-raw").find("sum")
        ff = [p for p in raw.findall("product") if "fcs/fbw/k-ff" in props(p)]
        self.assertEqual([props(p) for p in ff], [["fcs/fbw/k-ff", "fcs/fbw/dn-limited"]])
        self.assertIsNone(raw.find(".//table"))
        lim = fn("dn-limited").find("max")
        self.assertEqual(props(lim.find("min")), ["fcs/fbw/dn-cmd", "fcs/fbw/dn", "fcs/fbw/dn-room-up"])
        self.assertEqual(props(lim.find("sum")), ["fcs/fbw/dn", "fcs/fbw/dn-room-down"])
        for side, limit in (("up", o["alpha_max_deg"]), ("down", o["alpha_min_deg"])):
            room = fn("dn-room-" + side)
            with self.subTest(side=side):
                self.assertEqual(props(room), ["fcs/fbw/n-alpha", "fcs/fbw/alpha-ahead"])
                self.assertAlmostEqual(float(room.find(".//value").text), math.radians(limit), places=5)
        err = fn("pitch-error").find("max")
        self.assertEqual(props(err.find("min")), ["fcs/fbw/dn-model", "fcs/fbw/dn", "fcs/fbw/dn-room-up"])
        self.assertEqual([float(v.text) for v in err.iter("value")], [fcs.LIMIT_INTEGRAL] * 2)
        caps = fn("dn-stick").find("max").iter("difference")
        self.assertEqual([props(d) for d in caps], [["fcs/fbw/n-alpha", "fcs/fbw/g-ref"]] * 2)
        self.assertEqual(props(fn("dn")), ["accelerations/Nz", "fcs/fbw/g-ref"])


class ThrustVectoring(unittest.TestCase):
    def test_nozzles_turn_with_the_surfaces(self):
        # the F-22A's nozzles turn in pitch only, 20 deg: all their travel
        # with the elevator's, half of it differentially with the ailerons
        # (the left one's tail up rolls right); the Su-57's in planes canted
        # 32 deg outboard, so the rudder moves them too. Their power comes
        # signed as the surfaces' own: a nose-down elevator, a right roll, a
        # nose-left rudder (JSBSim: a positive pitch angle pushes the tail up,
        # a positive yaw angle pushes it right)
        from hangar import fcs
        from hangar.linear import loaded_inertia
        from hangar.mass import MassModel
        for name, travel, cant in (("f22a", 20.0, 0.0), ("su57", 15.0, 32.0)):
            a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
            vec = fcs.vectoring(a, loaded_inertia(MassModel(a)))
            de, da, dr = (math.radians(max(abs(x) for x in a.channel_limits(k))) for k in ("elevator", "aileron", "rudder"))
            with self.subTest(name=name):
                self.assertEqual([v["engine"] for v in vec], [0, 1])
                self.assertEqual([v["side"] for v in vec], [1.0, -1.0])        # the left copy first
                for v in vec:
                    self.assertAlmostEqual(v["travel"], math.radians(travel))
                    self.assertAlmostEqual(v["cant"], math.radians(cant))
                    self.assertAlmostEqual(v["k_elevator"], math.radians(travel) / de)
                    self.assertAlmostEqual(v["k_aileron"], fcs.ROLL_SHARE * math.radians(travel) / da)
                    self.assertAlmostEqual(v["k_rudder"], fcs.ROLL_SHARE * math.radians(travel) / dr if cant else 0.0)
                    self.assertLess(v["md"], 0.0)
                    self.assertGreater(v["lda"], 0.0)
                    self.assertTrue(v["ndr"] < 0.0 if cant else v["ndr"] == 0.0)
        self.assertEqual(fcs.vectoring(Aircraft.load(repo("aircraft/f16c/f16c.toml"))), [])

    def test_vectoring_channel_turns_the_thrust(self):
        # each nozzle's deflection from the surfaces' positions, within its
        # travel, turns its engine's thrust in its plane: the pitch angle the
        # deflection's upward part, the yaw angle its outboard part
        from hangar import fcs, jsbsim
        for name in ("f22a", "su57"):
            a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
            root = ET.fromstring("<fdm>%s</fdm>" % jsbsim.flight_control_xml(a))
            ch = root.find(".//channel[@name='Thrust Vectoring']")
            for v in fcs.vectoring(a):
                i, s, c = v["engine"], v["side"], v["cant"]
                with self.subTest(name=name, engine=i):
                    dfl = ch.find("fcs_function[@name='fcs/vectoring/nozzle-%d-rad']" % i)
                    terms = {p.find("property").text: float(p.find("value").text) for p in dfl.iter("product")}
                    want = {"fcs/elevator-pos-rad": v["k_elevator"], "fcs/left-aileron-pos-rad": s * v["k_aileron"]}
                    if c:
                        want["fcs/rudder-pos-rad"] = -s * v["k_rudder"]
                    self.assertEqual(set(terms), set(want))
                    for k, g in want.items():
                        self.assertAlmostEqual(terms[k], g, places=5)
                    self.assertAlmostEqual(float(dfl.find("clipto/max").text), v["travel"], places=5)
                    pitch = ch.find("fcs_function[@name='fcs/vectoring/pitch-%d-rad']" % i)
                    self.assertEqual(pitch.find("output").text, "propulsion/engine[%d]/pitch-angle-rad" % i)
                    self.assertAlmostEqual(float(pitch.findall(".//value")[1].text), math.cos(c), places=5)
                    yaw = ch.find("fcs_function[@name='fcs/vectoring/yaw-%d-rad']" % i)
                    if not c:
                        self.assertIsNone(yaw)
                        continue
                    self.assertEqual(yaw.find("output").text, "propulsion/engine[%d]/yaw-angle-rad" % i)
                    self.assertAlmostEqual(float(yaw.findall(".//value")[1].text), -s * math.sin(c), places=5)
        f16 = Aircraft.load(repo("aircraft/f16c/f16c.toml"))
        self.assertIsNone(ET.fromstring("<fdm>%s</fdm>" % jsbsim.flight_control_xml(f16)).find(
            ".//channel[@name='Thrust Vectoring']"))

    def test_gains_follow_the_power_thrust_adds(self):
        # with vectoring nozzles each axis's gains are moments over the power
        # there is now - the surfaces' (a table) and the nozzles' at each
        # engine's thrust - within the gains' limits, and the integrators hold
        # moments: what they trim stays as the throttle moves
        from hangar import fcs
        root, fbw = FlyByWire.pitch_channel("su57")
        self.assertIsNotNone(root.find(".//fcs_function[@name='fcs/fbw/k-alpha']/function/table"))  # none: tables
        a = Aircraft.load(repo("aircraft/su57/su57.toml"))
        from hangar.linear import loaded_inertia
        from hangar.mass import MassModel
        fbw["vectoring"] = fcs.vectoring(a, loaded_inertia(MassModel(a)))
        n = (len(fcs.QBAR_PSF), 2)
        for gains, power in fcs.AXES.values():
            fbw["gains"][power] = np.full(n, 5.0 * fcs.POWER_SIGN[power])
            for k in gains:
                fbw["gains"][k + "_m"] = np.full(n, 0.5)
        fbw["gains"]["p_per"] = np.full(n, 0.1)
        root = ET.fromstring("<fdm>%s</fdm>" % "\n".join(fcs.channels_xml(a, fbw)))

        def fn(name):
            return root.find(".//fcs_function[@name='fcs/fbw/%s']" % name)
        for axis, (gains, power) in fcs.AXES.items():
            total = fn(power + "-total").find("function")
            engines = [p.text for p in total.iter("property") if p.text.startswith("propulsion/")]
            with self.subTest(axis=axis):
                self.assertEqual(engines, ["propulsion/engine[0]/thrust-lbs", "propulsion/engine[1]/thrust-lbs"])
                self.assertEqual(total[0].tag, "max" if fcs.POWER_SIGN[power] > 0 else "min")
                for k in gains:
                    q = fn(k.replace("_", "-")).find("function/quotient")
                    self.assertEqual([p.text for p in q.iter("property")],
                                     ["fcs/fbw/%s-m" % k.replace("_", "-"), "fcs/fbw/%s-total" % power])
        for name, power in (("pitch-integral", "md"), ("roll-integral", "lda")):
            q = fn(name).find("function/quotient")
            self.assertEqual([p.text for p in q.iter("property")], ["fcs/fbw/%s-moment" % name, "fcs/fbw/%s-total" % power])
        comp = fn("moment-comp").find("function/product")
        self.assertEqual([p.text for p in comp.iter("property")],
                         ["fcs/fbw/moment-comp-aero", "fcs/fbw/md-aero", "fcs/fbw/md-total"])

    def test_nozzles_joint_turns_the_jet_the_way_the_thrust_turns(self):
        # the model's vectoring node: a channel mix over the nozzle's
        # surfaces, stopped at its travel, its x axis the one a positive
        # deflection turns the jet about - down, for the F-22A (the thrust
        # pushes the tail up); canted outboard, the Su-57's - and what hangs
        # on it stays where it was
        from hangar import fcs, model3d
        from hangar.shape import airframe as sh
        for name in ("f22a", "su57"):
            a = Aircraft.load(repo("aircraft/%s/%s.toml" % (name, name)))
            B = model3d._Builder()
            top = [B.node("fsim:afterburner:%d" % i, translation=[0.3 * i, 0.2, -1.0 - i], rotation=[0.0, 0.0, 0.0, 1.0])
                   for i in range(2)]
            before = [np.array(B.nodes[i]["translation"]) for i in top]
            origin = np.array([10.0, 0.0, 0.0])
            model3d._vectoring(a, B, top, origin)
            joints = [B.nodes[i] for i in top]
            for v, j in zip(fcs.vectoring(a), joints):
                with self.subTest(name=name, engine=v["engine"]):
                    mix = "fsim:elevator:%.6g+aileron:%.6g" % (v["k_elevator"], v["side"] * v["k_aileron"])
                    if v["cant"]:
                        mix += "+rudder:%.6g" % (-v["side"] * v["k_rudder"])
                    t = math.degrees(v["travel"])
                    self.assertEqual(j["name"], mix + "@%.6g,%.6g" % (-t, t))
                    R = model3d._quat_matrix(j["rotation"])
                    axis = np.array([-R[2, 0], -R[0, 0], R[1, 0]])       # its x, back in the design frame
                    np.testing.assert_allclose(axis, [0.0, math.cos(v["cant"]), v["side"] * math.sin(v["cant"])], atol=1e-9)
                    # turned by d about it, the jet (aft, +x) goes down and inboard: the
                    # thrust pushes the tail up and outboard
                    d = 0.2
                    jet = np.cos(d) * np.array([1.0, 0.0, 0.0]) + np.sin(d) * np.cross(axis, [1.0, 0.0, 0.0])
                    self.assertLess(jet[2], 0.0)
                    self.assertTrue(v["side"] * jet[1] >= 0.0 if v["cant"] else abs(jet[1]) < 1e-12)
                    pivot = model3d._to_gltf(np.array(v["nozzle"]) + sh.vectoring_hinge(a.engines[0]), origin)
                    np.testing.assert_allclose(j["translation"], pivot, atol=1e-9)
                    kid = B.nodes[j["children"][0]]
                    np.testing.assert_allclose(R @ kid["translation"] + j["translation"], before[v["engine"]], atol=1e-9)


class LargeAircraft(unittest.TestCase):
    def test_engines_past_the_platforms_throttles_follow_one(self):
        # the platform commands four throttles: an eighth engine follows the
        # fourth's lever, the fifth the first's
        from hangar import jsbsim
        a = Aircraft.load(repo("aircraft/c172/c172.toml"))
        e = a.spec["engine"][0]
        a.spec["engine"] = [dict(e, name="engine %d" % k, position=[1.0 * k, 0.0, 0.0]) for k in range(8)]
        a = Aircraft(a.spec)
        root = ET.fromstring("<fdm>%s</fdm>" % jsbsim.flight_control_xml(a))
        for i in range(jsbsim.PLATFORM_THROTTLES):
            self.assertIsNone(root.find(".//pure_gain[@name='fcs/throttle-lever-%d']" % i))
        for i in range(jsbsim.PLATFORM_THROTTLES, 8):
            g = root.find(".//pure_gain[@name='fcs/throttle-lever-%d']" % i)
            self.assertEqual(g.find("input").text, "fcs/throttle-cmd-norm[%d]" % (i % jsbsim.PLATFORM_THROTTLES))
            self.assertEqual(g.find("output").text, "fcs/throttle-pos-norm[%d]" % i)

    def test_a_tanks_fill_is_the_fuel_everywhere(self):
        # a tank's fill: the fuel the JSBSim file starts it with, the loaded
        # mass and CG, the inertia; fuel_fraction overrides it
        from hangar.linear import loaded_inertia
        from hangar.mass import MassModel
        a = Aircraft.load(repo("aircraft/c172/c172.toml"))
        full = MassModel(a)
        for t in a.spec["mass"]["tank"]:
            t["fill"] = 0.5
        half = MassModel(Aircraft(a.spec))
        fuel = sum(t["capacity"] for t in a.spec["mass"]["tank"])
        self.assertAlmostEqual(full.loaded()[0] - half.loaded()[0], 0.5 * fuel, places=6)
        self.assertAlmostEqual(half.loaded(fuel_fraction=1.0)[0], full.loaded()[0], places=6)
        self.assertAlmostEqual(half.loaded(fuel_fraction=0.0)[0], full.loaded(fuel_fraction=0.0)[0], places=6)
        self.assertNotAlmostEqual(loaded_inertia(half)[0], loaded_inertia(full)[0], places=3)
        with self.assertRaises(ValueError):
            a.spec["mass"]["tank"][0]["fill"] = 1.5
            MassModel(Aircraft(a.spec))

    def test_lateral_matrix_at_an_angle_of_attack(self):
        # body axes about level flight at alpha0: a roll rate turns the trim
        # velocity into sideslip, a yaw rate less of it, gravity's share of the
        # bank goes as cos alpha0 and a yaw rate tilts the bank; at alpha0 = 0
        # the stability-axis matrix
        from hangar.linear import G0, lateral_matrix
        d = {"CYb": -0.8, "CYp": 0.1, "CYr": 0.4, "Clb": -0.1, "Clp": -0.45, "Clr": 0.1, "Cnb": 0.12, "Cnp": -0.05,
             "Cnr": -0.15}
        args = (d, 0.5 * 1.225 * 100.0 ** 2 * 30.0, 12.0, 8000.0, 100.0, (3.0e4, 9.0e4, 1.0e3))
        A0, A = lateral_matrix(*args, 0.0), lateral_matrix(*args, math.radians(8.0))
        np.testing.assert_allclose(A0[3], [0.0, 1.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(A[1:3], A0[1:3])
        sa, ca = math.sin(math.radians(8.0)), math.cos(math.radians(8.0))
        self.assertAlmostEqual(A[0, 1] - A0[0, 1], sa)
        self.assertAlmostEqual(A[0, 2] - A0[0, 2], 1.0 - ca)
        self.assertAlmostEqual(A[0, 3], G0 * ca / 100.0)
        self.assertAlmostEqual(A[3, 2], sa / ca)


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
        for name, expected in (("c172", [(7.18, 0.0, 2.25), (6.54, 1.73, 0.96), (1.88, 5.49, 1.69), (-0.96, 0.0, -0.28)]),
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
                    # the skin painted: one JPEG texture, the airframe's skin and
                    # the control surfaces coloured by it through their own coordinates
                    self.assertEqual([i["mimeType"] for i in doc["images"]], ["image/jpeg"])
                    paint = [k for k, m in enumerate(doc["materials"])
                             if "baseColorTexture" in m["pbrMetallicRoughness"]]
                    self.assertEqual(len(paint), 1)
                    airframe = next(m for m in doc["meshes"] if m["name"] == "airframe")
                    skin = [q for q in airframe["primitives"] if q["material"] == paint[0]]
                    self.assertTrue(skin)
                    self.assertTrue(all("TEXCOORD_0" in q["attributes"] for q in skin))

    def test_livery_faces_and_seams(self):
        # each face is painted from the view it faces, its vertices split where
        # neighbouring faces face different ways, its coordinates in the atlas
        from hangar import livery
        a = Aircraft.load(repo("aircraft/skua/skua.toml"))
        L = livery.Livery(a, [[0.0, -2.0, -0.5], [2.0, 2.0, 0.5]])
        n = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.3, -0.9, 0.3]])
        self.assertEqual(L.regions(n).tolist(), [livery.TOP, livery.BOTTOM, livery.SIDE, livery.FRONT, livery.SIDE])
        # two triangles sharing an edge, one facing down and one sideways
        v = np.array([[0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.5, 0.0], [0.5, 0.0, -0.5]])
        t = np.array([[0, 2, 1], [0, 1, 3]])
        v2, n2, t2, uv = L.split(v, np.zeros_like(v), t)
        self.assertEqual(len(v2), 6)  # the shared edge's two vertices, once per view
        self.assertTrue(((uv >= 0.0) & (uv <= 1.0)).all())
        self.assertEqual(L.image()[:2], bytes([0xFF, 0xD8]))  # a JPEG

    def test_stand_in_wheels(self):
        # a stand-in's main wheels, relative to its model's origin (its empty
        # CG), in body axes: behind and below it
        from hangar import register
        a = Aircraft.load(repo("aircraft/f16c/f16c.toml"))
        forward, right, down = register.design_mains(a)
        self.assertLess(forward, -0.3)
        self.assertEqual(right, 0.0)
        self.assertGreater(down, 1.5)
        self.assertIn("f16", dict((d.name, s) for d, s in register.stand_ins(repo("aircraft")))["f16c"])

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

    def test_leaning_sections(self):
        # a leaning section is the upright one sheered: its centre moves
        # outboard tan(lean) per metre up, the same in the mesher's distance
        # field and the body's own skin; an intake's duct follows its cowl and
        # stops just ahead of the lip, never carving the body it hugs
        from hangar.geometry.body import Body, Intake
        from hangar.shape import airframe as sh
        from hangar.shape import meshkit
        if meshkit.library() is None:
            self.skipTest("hangar_meshkit is not built")
        rows = [{"x": x, "y": 1.2, "w": 0.8, "top": 0.0, "bottom": -1.0, "lean": 40} for x in (5.0, 7.0)]
        b = Body({"name": "wall", "mirror": True, "stations": rows})
        k = float(b.slant(6.0))
        self.assertAlmostEqual(k, np.tan(np.radians(40.0)))
        # the widest line at mid-height, 0.4 m out; 0.3 m above it the section
        # has moved 0.3 k outboard: points on the leaned ellipse lie on the skin
        z = np.array([-0.5, -0.2, -0.8])
        v = (z + 0.5) / 0.5
        y = 1.2 + k * (z + 0.5) + 0.4 * np.sqrt(1.0 - v * v)
        d, _ = meshkit.evaluate({"root": sh.loft(b)}, np.column_stack([np.full(3, 6.0), y, z]))
        self.assertTrue(np.all(np.abs(d) < 0.01), d)
        verts, _, _ = b.skin(16, 32)
        mid = np.abs(verts[:, 0] - 6.0) < 0.2
        top = verts[mid & (verts[:, 1] > 0)][:, 2].argmax()
        self.assertGreater(verts[mid & (verts[:, 1] > 0)][top, 1], 1.2 + 0.4 * k)  # the top leans out
        # the duct: open through the lip plane, gone 10 cm ahead of it
        inl = Intake({"name": "in", "mirror": True, "lip": 0.03, "rake": 30, "duct": 1.0,
                      "stations": [dict(r, lean=40) for r in rows]})
        _, duct = sh.intake(inl)
        p, n = inl.lip_plane()
        dd, _ = meshkit.evaluate({"root": duct}, np.array([p - 0.02 * n, p + 0.10 * n]))
        self.assertLess(dd[0], 0.0)
        self.assertGreater(dd[1], 0.0)

    def test_struts_brace_the_wing(self):
        # a strut is a streamlined bar from end to end: its chord along x,
        # its thickness across, mirrored to the left
        from hangar.geometry.aircraft import Strut
        from hangar.shape import airframe as sh
        from hangar.shape import meshkit
        if meshkit.library() is None:
            self.skipTest("hangar_meshkit is not built")
        st = Strut({"from": [1.0, 0.5, 0.15], "to": [1.2, 2.5, 1.55], "chord": 0.16, "thickness": 0.05})
        node = sh.strut(st)
        mid = 0.5 * (st.a + st.b)
        a = (st.b - st.a) / np.linalg.norm(st.b - st.a)
        t = np.cross(a, np.array(node["axes"][1]))
        pts = np.array([mid, mid * [1, -1, 1], mid + [0.07, 0.0, 0.0], mid + 0.04 * t, st.a - 0.02 * a])
        d, _ = meshkit.evaluate({"root": node}, pts)
        self.assertTrue(np.all(d[:3] < 0.0), d)   # on its axis, the mirror's, within its chord
        self.assertTrue(np.all(d[3:] > 0.0), d)   # beyond its thickness, past its end
        with self.assertRaises(ValueError):
            Strut({"from": [0.0, 0.0, 0.0], "to": [0.0, 1.0, 0.0], "chord": 0.1, "thickness": 0.2})

    def test_raked_intake_keeps_its_far_end(self):
        # the lip plane cuts a steeply raked intake's cowl, not its far end:
        # a long chin intake raked 55 deg keeps its belly 10 m behind the lip
        from hangar.geometry.body import Intake
        from hangar.shape import airframe as sh
        from hangar.shape import meshkit
        if meshkit.library() is None:
            self.skipTest("hangar_meshkit is not built")
        rows = [{"x": x, "w": 1.3, "top": 0.3, "bottom": -0.35, "n": 4.0} for x in (3.5, 8.0, 13.5)]
        cowl, _ = sh.intake(Intake({"name": "chin", "lip": 0.04, "rake": 55, "stations": rows}))
        d, _ = meshkit.evaluate({"root": cowl}, np.array([[13.0, 0.0, -0.3], [8.0, 0.5, -0.25]]))
        self.assertTrue(np.all(d < 0.0), d)

    def test_mesh_is_the_same_every_time(self):
        # a scene gives the same mesh, byte for byte, on one thread or on all
        # of them, so a design's .glb changes only when the design does: the
        # bricks are joined in their own order whichever thread made them,
        # and the decimator breaks its ties by index
        from hangar.shape import meshkit
        if meshkit.library() is None:
            self.skipTest("hangar_meshkit is not built")
        scene = {"cell": 0.016, "root": {"op": "union", "k": 0.05, "children": [
            {"prim": "ellipsoid", "centre": [0.0, 0.0, 0.0], "radii": [2.0, 0.5, 0.5]},
            {"prim": "box", "centre": [0.2, 0.0, 0.0], "half": [0.5, 2.0, 0.05], "round": 0.02, "material": 1},
            {"prim": "cylinder", "a": [1.6, 0.0, 0.2], "b": [1.9, 0.0, 0.9], "r": 0.1, "material": 2}]}}
        one = meshkit.build(dict(scene, threads=1))
        self.assertGreater(one["raw_triangles"], 400000)  # the decimator's parallel first pass runs too
        for _ in range(2):
            m = meshkit.build(scene)
            for k in ("positions", "normals", "triangles", "materials"):
                self.assertTrue(np.array_equal(m[k], one[k]), k)

    def test_model_cache_follows_the_shape_not_the_aerodynamics(self):
        # the model is re-made when what shapes it changes, and only then: an
        # edit to the aerodynamics leaves the model's key as it is (and changes
        # the tables'), an edit to the code that shapes the model changes it
        import os
        import shutil
        import subprocess
        import sys
        import tempfile

        import hangar
        probe = ("import sys, hangar; from hangar.pipeline import Design; d = Design(sys.argv[1], log=None); "
                 "print(d.model_key(), d.spec_hash('surface', 'body', 'intake', 'reference', 'analysis', 'gear', 'engine'), "
                 "hangar.__file__)")
        with tempfile.TemporaryDirectory() as tmp:
            pkg = shutil.copytree(os.path.dirname(hangar.__file__), os.path.join(tmp, "hangar"),
                                  ignore=shutil.ignore_patterns("__pycache__"))
            design = shutil.copytree(repo("aircraft/skua"), os.path.join(tmp, "skua"), ignore=shutil.ignore_patterns("out"))

            env = dict(os.environ, PYTHONPATH=os.pathsep.join([tmp, os.environ.get("PYTHONPATH", "")]))

            def keys():
                # run in tmp: its copy of hangar comes first on the path
                run = subprocess.run([sys.executable, "-c", probe, os.path.join(design, "skua.toml")], cwd=tmp, env=env,
                                     capture_output=True, text=True)
                self.assertEqual(run.returncode, 0, run.stderr)
                model, tables, where = run.stdout.split(maxsplit=2)
                self.assertTrue(os.path.samefile(os.path.dirname(where.strip()), pkg), where)
                return model, tables

            def edit(rel):
                with open(os.path.join(pkg, rel), "a", encoding="utf-8") as f:
                    f.write("\n# an edit\n")

            model, tables = keys()
            edit(os.path.join("aero", "vlm.py"))
            model_a, tables_a = keys()
            self.assertEqual(model_a, model)        # nothing to re-mesh
            self.assertNotEqual(tables_a, tables)   # the tables are rebuilt
            edit(os.path.join("shape", "airframe.py"))
            model_s, tables_s = keys()
            self.assertNotEqual(model_s, model)     # re-meshed
            self.assertEqual(tables_s, tables_a)    # the tables stay

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
