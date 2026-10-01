"""What an activity flies, and where to, through Python (docs/flight-autonomy.md, 4.12): an activity's setpoint read
back as the command that would fly it, its end points, and what the vehicle is commanded - with A-GRA's names."""
import math
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-reports", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


def same(a, b):
    """Two setpoints' fields alike, a field left out (NaN) as one."""
    return a.keys() == b.keys() and all(a[k] == b[k] or (math.isnan(a[k]) and math.isnan(b[k])) for k in a)


def straight(n0, n1, e0=0.0, e1=0.0):
    return fsim.BezierSegment([n0 + (n1 - n0) * i / 5 for i in range(6)], [e0 + (e1 - e0) * i / 5 for i in range(6)], [0.0] * 6)


class ReportsTest(unittest.TestCase):
    def test_a_setpoint_read_back_flies_again(self):
        w = make_world()
        v = viper(w)
        w.step(5)
        a = v.submit_hsa(heading_rad=1.2, altitude_m=3200.0)
        read = a.setpoint()
        self.assertEqual(read.method, "submit_hsa")
        self.assertAlmostEqual(read.kwargs["heading_rad"], 1.2)
        self.assertEqual(read.kwargs["altitude_m"], 3200.0)
        self.assertEqual(read.kwargs["speed_reference"], fsim.SpeedReference.TRUE_AIRSPEED)  # (completed: what it flies now)
        again = v.submit_batch([read])[0]  # the command that would fly it, flown
        self.assertIsInstance(again, fsim.Activity)
        self.assertTrue(same(again.setpoint().kwargs, read.kwargs))
        self.assertIsNone(a.setpoint())  # (replaced: not live)
        flaps = v.submit_support("flaps", position=0.0)  # (its gear: above its placard, docs/flight-autonomy.md 4.57)
        self.assertEqual((flaps.setpoint().method, flaps.setpoint().args, flaps.setpoint().kwargs), ("submit_support", ("flaps",), {"position": 0.0}))
        level = v.submit(fsim.Level.VELOCITY, airspeed_ms=170.0, vertical_speed_ms=2.0, heading_rad=1.4)
        self.assertEqual(level.setpoint().args, (fsim.Level.VELOCITY,))
        self.assertEqual(level.setpoint().kwargs["vertical_speed_ms"], 2.0)
        roll = v.submit_behavior("aerobatics", manoeuvre=0.0)
        self.assertEqual((roll.setpoint().method, roll.setpoint().args[0], roll.setpoint().kwargs), ("submit_behavior", "aerobatics", {"manoeuvre": 0.0}))

    def test_end_points_and_the_flyout_curve(self):
        w = make_world()
        v = viper(w)
        w.step(5)
        s = v.state
        lat, lon = s.latitude_rad, s.longitude_rad
        points = [fsim.Waypoint(lat, lon + 0.0008, 3000.0, id=11), fsim.Waypoint(lat + 0.0006, lon + 0.0012, 3100.0, id=12, turn="fly_over"),
                  fsim.Waypoint(lat + 0.0012, lon + 0.0008, 3200.0, id=13)]
        r = v.submit_route(points, end=fsim.EndBehavior.LOITER)
        self.assertEqual([p.id for p in r.setpoint().args[0]], [11, 12, 13])
        e = r.end_points()
        self.assertEqual([p.kind for p in e], [fsim.EndPointKind.TURN_POINT, fsim.EndPointKind.TURN_POINT, fsim.EndPointKind.LOITER_POINT])
        self.assertEqual([agra.end_point(p) for p in e], [("TurnPoint", "TURN_SHORT"), ("TurnPoint", "FLY_OVER"), ("LoiterPoint", None)])
        self.assertEqual(agra.altitude_reference(e[2].altitude_reference), "MSL")
        self.assertEqual(len(r.end_points(max=1)), 1)
        k = v.submit_curve([straight(0.0, 2000.0)])
        w.step(5)
        k.append([straight(2000.0, 4000.0, 0.0, 500.0)])
        read = k.setpoint()
        curve = agra.flyout_curve(read)
        self.assertEqual(len(curve), 2)  # (the appended segment too)
        self.assertEqual(curve[1]["CenterReference"], (read.kwargs["latitude_rad"], read.kwargs["longitude_rad"], read.kwargs["altitude_m"]))
        self.assertEqual(curve[1]["ControlPoints"][5], ((4000.0, 500.0, 0.0), 1.0))
        self.assertEqual(curve[0]["KnotVector"], (0.0,) * 6 + (1.0,) * 6)
        ends = k.end_points()
        self.assertEqual([p.index for p in ends], [0, 1])
        self.assertAlmostEqual(ends[1].latitude_rad, read.kwargs["latitude_rad"] + 4000.0 / 6371000.0, places=6)
        with self.assertRaises(ValueError):
            agra.flyout_curve(r.setpoint())

    def test_what_the_vehicle_is_commanded(self):
        w = make_world()
        v = viper(w)
        w.step(5)
        v.submit_hsa(heading_rad=-1.0, altitude_m=2500.0, altitude_reference="above_ground")
        w.step(60)
        c = v.commanded
        self.assertEqual((c.altitude_m, agra.altitude_reference(c.altitude_reference)), (2500.0, "AGL"))
        self.assertGreater(math.hypot(c.north_acceleration_ms2, c.east_acceleration_ms2), 5.0)  # (turning)
        v.submit(fsim.Level.ACCELERATION, load_factor_g=1.0, roll_rate_rad_s=0.0, throttle=0.8)
        w.step(2)
        self.assertTrue(math.isnan(v.commanded.down_acceleration_ms2))  # (a throttle, not a longitudinal acceleration)


if __name__ == "__main__":
    unittest.main()
