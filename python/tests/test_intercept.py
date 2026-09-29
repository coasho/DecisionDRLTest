"""A route intercept through Python (docs/flight-autonomy.md, 4.47): a plan of four points north, 2 km west of a C172 flying
north, the aircraft abeam 40 % of the leg into its third point; each method's join, its status as it flies, the plan activated
by it and complete; a batch item's; no UPDATE; refusals."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


class InterceptTest(unittest.TestCase):
    def setUp(self):
        self.w = fsim.World("py-intercept", publish=False, workers=1, pin_workers=False, seed=5, terrain=False)
        self.v = self.w.create_vehicle("i", "jsbsim:c172x", latitude_deg=37.6, longitude_deg=-122.4, altitude_msl_m=1500.0, airspeed_ms=55.0,
                                       heading_deg=0.0)
        self.w.step(10)
        s = self.v.state
        self.lat0, self.lon0 = s.latitude_rad, s.longitude_rad
        west = s.longitude_rad - 2000.0 / (R * math.cos(s.latitude_rad))
        points = [fsim.Waypoint(s.latitude_rad + (k - 2.4) * 3000.0 / R, west, 1500.0, id=100 + k) for k in range(4)]
        v = self.v
        self.assertTrue(v.plan_command(44, "prepare_for_upload").completed)
        v.publish_plan(fsim.RoutePlan(44, fsim.BatchCommand("submit_route", points)))
        self.assertTrue(v.plan_command(44, "upload").completed)

    def test_methods(self):
        v = self.v
        # the beginning; the nearest point; where the perpendicular meets the leg into point 3; the soonest, ahead of it
        expected = {None: (0, -1), "discrete": (2, -1), "shortest_distance": (3, 4)}
        for method, (joined, laid) in expected.items():
            a = v.submit_intercept(plan=44, **({} if method is None else {"method": method}))
            s = a.intercept_status()
            self.assertEqual((s.plan_id, s.joined, s.laid), (44, joined, laid), method)
        a = v.submit_intercept(plan=44, method=fsim.InterceptMethod.SOONEST)
        s = a.intercept_status()
        self.assertEqual((s.joined, s.laid), (3, 4))
        self.assertGreater((s.join_latitude_rad - self.lat0) * R, 50.0)  # (ahead of the foot: it turns toward the route as it goes on)
        self.assertEqual(v.plan_status(44).state, fsim.PlanState.ACTIVATED)
        self.assertEqual(a.setpoint().method, "submit_intercept")
        self.assertEqual(a.setpoint().kwargs["method"], float(fsim.InterceptMethod.SOONEST))
        # its status as it flies: the leg it joins, then each point in turn; the plan complete
        self.w.step(30)
        s = a.intercept_status()
        self.assertEqual(s.execution, fsim.PlanExecution.EXECUTING)
        self.assertIsNone(s.previous)
        self.assertEqual((s.current.point, s.current.point_id), (3, 103))
        self.assertGreater(s.current.capture_distance_m, 0.0)
        self.assertIsNone(s.next)  # (point 3 is the plan's last)
        for _ in range(120):
            self.w.step(30)
            if a.state == fsim.ActivityState.COMPLETED:
                break
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertEqual(v.plan_status(44).execution, fsim.PlanExecution.COMPLETE)
        self.assertEqual(a.intercept_status().execution, fsim.PlanExecution.COMPLETE)

    def test_batch_update_refusals(self):
        v = self.v
        (a,) = v.submit_batch([fsim.BatchCommand("submit_intercept", plan=44, method="discrete")])
        self.assertEqual(a.intercept_status().joined, 2)
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(method=2.0)
        self.assertEqual(refused.exception.reason, "not_updatable")
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_intercept(plan=45)
        self.assertEqual(refused.exception.reason, "unknown_plan")
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_intercept(plan=44, method=7.0)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 2))
        self.assertIsNone(self.w.intercept_status(12345))
        self.assertEqual(v.support("fsim.guidance.intercept").support, fsim.Support.SUPPORTED)


if __name__ == "__main__":
    unittest.main()
