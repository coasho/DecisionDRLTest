"""A curve's segments as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.26): clamped rational
B-splines (fsim.NurbsSegment) submitted, read back, updated, in a batch and a task, a malformed one refused naming it."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def cessna(w, name="cessna"):
    return w.create_vehicle(name, "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                            heading_deg=0.0)


def cubic(n0, length, swing):
    """A rational cubic S along north from n0: seven points, three interior knots, uneven weights."""
    L = length
    return fsim.NurbsSegment(north=[n0, n0 + 0.15 * L, n0 + 0.3 * L, n0 + 0.5 * L, n0 + 0.7 * L, n0 + 0.85 * L, n0 + L],
                             east=[0.0, 0.0, swing, swing, -swing, 0.0, 0.0], down=[0.0] * 7,
                             knots=[0.0, 0.0, 0.0, 0.0, 0.2, 0.45, 0.7, 1.0, 1.0, 1.0, 1.0], weights=[1.0, 1.4, 0.8, 1.0, 1.25, 0.9, 1.0])


class NurbsTest(unittest.TestCase):
    def test_submitted_read_back_updated_batched_and_refused(self):
        w = make_world("py-nurbs")
        v = cessna(w)
        w.step(10)
        a = v.submit_curve([cubic(0.0, 8000.0, 400.0)])
        back = a.setpoint()
        self.assertEqual(len(back.args[0]), 1)
        s = back.args[0][0]
        self.assertIsInstance(s, fsim.NurbsSegment)
        self.assertEqual((len(s.north), len(s.knots), s.weights[1]), (7, 11, 1.4))
        self.assertTrue(math.isnan(s.curvature))
        flyout = fsim.agra.flyout_curve(back)[0]  # (A-GRA's form: as given)
        self.assertEqual(flyout["KnotVector"], [0.0, 0.0, 0.0, 0.0, 0.2, 0.45, 0.7, 1.0, 1.0, 1.0, 1.0])
        self.assertEqual(flyout["ControlPoints"][1][1], 1.4)
        self.assertNotIn("Curvature", flyout)
        w.step(int(round(20.0 / w.step_seconds)))
        self.assertEqual(a.state, fsim.ActivityState.ACTIVE)
        # appended: another S from where it ends, as a dict; a Bezier segment beside a NURBS one, made one
        after = cubic(8000.0, 8000.0, 400.0)._asdict()
        a.append([after])
        self.assertEqual(len(a.setpoint().args[0]), 2)
        # a malformed one refused naming it: its first knot too few times (not clamped)
        bad = cubic(0.0, 8000.0, 400.0)._replace(knots=[0.0, 0.0, 0.0, 0.1, 0.2, 0.45, 0.7, 1.0, 1.0, 1.0, 1.0])
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_curve([cubic(0.0, 8000.0, 400.0), bad._replace(north=[x + 8000.0 for x in bad.north])])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_curve", 1))
        # a curvature said too small: the section named
        tight = cubic(0.0, 8000.0, 400.0)._replace(curvature=1e-4)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_curve([tight])
        self.assertEqual(refused.exception.reason, "invalid_curve")
        self.assertIsNotNone(refused.exception.section)
        # a batch with a NURBS curve, and a task
        results = v.submit_batch([fsim.BatchCommand("submit_curve", [cubic(0.0, 8000.0, 400.0)])])
        self.assertIsInstance(results[0], fsim.Activity)
        self.assertIsInstance(results[0].setpoint().args[0][0], fsim.NurbsSegment)
        v.store_task(5, fsim.BatchCommand("submit_curve", [cubic(0.0, 8000.0, 400.0)]))  # (kept: raises if refused)
        t = v.command_task(5)
        self.assertIsInstance(t.setpoint().args[0][0], fsim.NurbsSegment)


if __name__ == "__main__":
    unittest.main()
