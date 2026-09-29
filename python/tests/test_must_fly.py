"""A must fly through Python (docs/flight-autonomy.md, 4.42 to 4.44): a point flown over at its altitude; an operational point
kept by the world, read back and flown from within its window of bearings; another vehicle flown over; zones entered; corridors
flown through; refusals."""
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

    def test_zone_given_and_kept(self):
        """A zone (docs/flight-autonomy.md, 4.43): an ellipse given with a must fly, entered; a polygon with a hole kept by the world,
        read back and entered by its id; one updated in place of its own; one malformed refused naming its field."""
        w, v = self.w, self.v
        lat, lon = self.place(0.0, 4000.0)
        a = v.submit_must_fly(zone=fsim.OpZone(0, "ellipse", latitude_rad=lat, longitude_rad=lon, semi_major_m=800.0, semi_minor_m=500.0))
        self.assertEqual(a.setpoint().kwargs["location"], float(fsim.MustFlyLocation.ZONE))
        for _ in range(600):
            w.step(6)
            if not a.live:
                break
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        s = v.state
        north, east = (s.latitude_rad - lat) * R, (s.longitude_rad - lon) * R * math.cos(lat)
        self.assertLessEqual((north / 800.0) ** 2 + (east / 500.0) ** 2, 1.01)  # (in it, as it completed)
        # a polygon with a hole, kept and read back
        square = [self.place(n, e) for n, e in ((-1000.0, 2000.0), (1000.0, 2000.0), (1000.0, 4000.0), (-1000.0, 4000.0))]
        hole = [self.place(n, e) for n, e in ((-200.0, 2500.0), (200.0, 2500.0), (0.0, 3000.0))]
        w.set_op_zone(fsim.OpZone(21, fsim.ZoneShape.POLYGON, square, [hole], altitude_min_m=1000.0, altitude_max_m=2000.0))
        back = w.op_zone(21)
        self.assertEqual((back.id, back.revision, back.shape, len(back.vertices), len(back.holes)), (21, 1, float(fsim.ZoneShape.POLYGON), 4, 1))
        self.assertEqual(back.vertices[1].latitude_rad, square[1][0])
        self.assertTrue(math.isnan(back.vertices[1].x_m))
        self.assertEqual([z.id for z in w.op_zones()], [21])
        b = v.submit_must_fly(location="op_zone", target=21)
        # moved: a zone given in place of its own
        lat2, lon2 = self.place(3000.0, 3000.0)
        b.update_must_fly(zone=fsim.OpZone(0, "rectangle", latitude_rad=lat2, longitude_rad=lon2, width_m=1000.0, height_m=1000.0), location="zone")
        self.assertEqual(b.setpoint().kwargs["location"], float(fsim.MustFlyLocation.ZONE))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(zone=fsim.OpZone(0, "ellipse", latitude_rad=lat, longitude_rad=lon, semi_major_m=100.0, semi_minor_m=200.0))
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 14))
        with self.assertRaises(fsim.Rejected) as refused:
            w.set_op_zone(fsim.OpZone(22, "polygon", square[:2]))  # (two vertices)
        self.assertEqual(refused.exception.reason, "invalid_parameter")
        self.assertTrue(w.remove_op_zone(21))
        self.assertFalse(w.remove_op_zone(21))
        self.assertIsNone(w.op_zone(21))

    def test_line_given_and_kept(self):
        """A corridor (docs/flight-autonomy.md, 4.44): a line with a right angle given with a must fly, flown through; one kept by
        the world, read back and flown by its id, then given a line in place of its own; one too narrow for its turn refused;
        one malformed refused naming its field."""
        w, v = self.w, self.v
        corners = [self.place(0.0, 2000.0), self.place(0.0, 5000.0), self.place(3000.0, 5000.0)]
        line = fsim.OpLine(0, corners, left_width_m=500.0, right_width_m=500.0)
        a = v.submit_must_fly(line=line)
        self.assertEqual(a.setpoint().kwargs["location"], float(fsim.MustFlyLocation.LINE))
        for _ in range(1500):
            w.step(6)
            if not a.live:
                break
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        s = v.state
        lat, lon = corners[-1]
        self.assertLess(math.hypot((s.latitude_rad - lat) * R, (s.longitude_rad - lon) * R * math.cos(lat)), 20.0)  # (done as it is passed)
        # kept, read back and flown by its id; then a line given in place of its own
        w.set_op_line(fsim.OpLine(41, [fsim.LineVertex(*self.place(0.0, 2000.0), altitude_m=1600.0), self.place(0.0, 5000.0)], projection="rhumb"))
        back = w.op_line(41)
        self.assertEqual((back.id, back.revision, len(back.vertices), back.vertices[0].altitude_m, back.projection),
                         (41, 1, 2, 1600.0, float(fsim.Projection.RHUMB)))
        self.assertEqual([l.id for l in w.op_lines()], [41])
        b = v.submit_must_fly(location="op_line", target=41)
        b.update_must_fly(line=line, location="line")
        self.assertEqual(b.setpoint().kwargs["location"], float(fsim.MustFlyLocation.LINE))
        # 20 m wide with a right angle: the turn cuts inside it by more
        narrow = fsim.OpLine(0, [self.place(0.0, 2000.0), self.place(0.0, 5000.0), self.place(3000.0, 5000.0)], left_width_m=20.0,
                             right_width_m=20.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(line=narrow)
        self.assertEqual(refused.exception.reason, "performance_limit")
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_must_fly(line=fsim.OpLine(0, corners[:1]))  # (one vertex)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 10))
        self.assertTrue(w.remove_op_line(41))
        self.assertFalse(w.remove_op_line(41))
        self.assertIsNone(w.op_line(41))

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
