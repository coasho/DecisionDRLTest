"""A must fly through Python (docs/flight-autonomy.md, 4.42): a point flown over at its altitude; an operational point kept by
the world, read back and flown from within its window of bearings; another vehicle flown over; refusals."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class MustFlyTest(unittest.TestCase):
    def setUp(self):
        self.w = make_world("py-must-fly")
        self.v = self.w.create_vehicle("flier", "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                                       airspeed_ms=55.0, heading_deg=90.0)
        self.w.step(10)

    def place(self, north, east):
        s = self.v.state
        return s.latitude_rad + north / R, s.longitude_rad + east / (R * math.cos(s.latitude_rad))

    def fly_over(self, activity, lat, lon, seconds, entity=None):
        """The least distance over the ground to the location as it was passed, the track then, how the activity ended."""
        closest, track = math.inf, 0.0
        for _ in range(int(seconds * 5)):
            self.w.step(6)  # (0.2 s)
            s = self.v.state
            if entity is not None:
                lat, lon = entity.state.latitude_rad, entity.state.longitude_rad
            d = math.hypot((s.latitude_rad - lat) * R, (s.longitude_rad - lon) * R * math.cos(lat))
            if d < closest:
                closest, track = d, math.atan2(s.velocity_ned_ms[1], s.velocity_ned_ms[0])
            if not activity.live:
                break
        return closest, track, activity.state

    def test_point(self):
        lat, lon = self.place(1000.0, 4000.0)
        a = self.v.submit_must_fly(location="point", latitude_rad=lat, longitude_rad=lon, altitude_m=1600.0)
        sp = a.setpoint()
        self.assertEqual((sp.method, sp.kwargs["location"], sp.kwargs["latitude_rad"], sp.kwargs["longitude_rad"]),
                         ("submit_must_fly", float(fsim.MustFlyLocation.POINT), lat, lon))
        self.assertEqual(len(a.end_points()), 1)  # (its route: the point alone)
        closest, _, state = self.fly_over(a, lat, lon, 150.0)
        self.assertEqual(state, fsim.ActivityState.COMPLETED)
        self.assertLess(closest, 40.0)

    def test_op_point_from_its_window(self):
        w, v = self.w, self.v
        lat, lon = self.place(0.0, 4000.0)
        w.set_op_point(fsim.OpPoint(7, lat, lon, 1500.0, ingress_min_rad=math.radians(170.0), ingress_max_rad=math.radians(-170.0)))
        back = w.op_point(7)
        self.assertEqual((back.id, back.revision, back.altitude_m), (7, 1, 1500.0))
        self.assertTrue(math.isnan(back.frame))
        self.assertEqual([p.id for p in w.op_points()], [7])
        with self.assertRaises(fsim.Rejected) as refused:
            w.set_op_point(fsim.OpPoint(8, lat))  # (half a place)
        self.assertEqual(refused.exception.reason, "invalid_parameter")
        a = v.submit_must_fly(location=fsim.MustFlyLocation.OP_POINT, target=7)
        self.assertEqual(len(a.end_points()), 2)  # (an approach from the south, then the point)
        closest, track, state = self.fly_over(a, lat, lon, 400.0)
        self.assertEqual(state, fsim.ActivityState.COMPLETED)
        self.assertLess(closest, 40.0)
        came_from = math.degrees(math.remainder(track + math.pi, 2.0 * math.pi)) % 360.0
        self.assertTrue(168.0 < came_from < 192.0, came_from)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(location="op_point", target=9)
        self.assertEqual(refused.exception.reason, "unknown_geometry")
        self.assertTrue(w.remove_op_point(7))
        self.assertFalse(w.remove_op_point(7))
        self.assertIsNone(w.op_point(7))

    def test_entity(self):
        w, v = self.w, self.v
        lat, lon = self.place(-3000.0, 6000.0)
        slow = w.create_vehicle("slow", "jsbsim:c172x", latitude_deg=math.degrees(lat), longitude_deg=math.degrees(lon), altitude_msl_m=1400.0,
                                airspeed_ms=45.0, heading_deg=0.0)
        slow.submit_hsa(course_rad=0.0, speed=45.0, altitude_m=1400.0)
        a = v.submit_must_fly(location="entity", target=slow)
        closest, _, state = self.fly_over(a, 0.0, 0.0, 300.0, entity=slow)
        self.assertEqual(state, fsim.ActivityState.COMPLETED)
        self.assertLess(closest, 100.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(location="entity", target=v)  # (itself)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 5))

    def test_refused_and_updated(self):
        v = self.v
        lat, lon = self.place(0.0, 4000.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(location="point", latitude_rad=lat)  # (no longitude)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 2))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(location="point", latitude_rad=lat, longitude_rad=lon, ingress_min_rad=1.0)  # (a window one way)
        self.assertEqual(refused.exception.index, 7)
        a = v.submit_must_fly(location="point", latitude_rad=lat, longitude_rad=lon, altitude_m=1500.0)
        lat2, lon2 = self.place(1000.0, 5000.0)
        a.update(latitude_rad=lat2, longitude_rad=lon2)
        kept = a.setpoint().kwargs
        self.assertEqual((kept["latitude_rad"], kept["longitude_rad"], kept["altitude_m"]), (lat2, lon2, 1500.0))  # (the altitude kept)
        self.assertEqual(v.support("fsim.guidance.must_fly").support, fsim.Support.PARTIAL)
        self.assertEqual(v.support("fsim.geometry").support, fsim.Support.PARTIAL)


if __name__ == "__main__":
    unittest.main()
