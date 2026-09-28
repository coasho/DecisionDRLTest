"""A route's loiter points as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.31): an orbit of two
laps inside a route by name, read back complete and flown, then on; its time to go while it loiters; an UPDATE with new
waypoints and theirs; a batch's and a task's route with its loiters; a loiter point with none refused, naming it; a
radius tighter than it flies held to it, named by its point and its field from 100."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteLoitersTest(unittest.TestCase):
    def test_an_orbit_inside_a_route(self):
        w = make_world("py-route-loiters")
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                             heading_deg=0.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad  # (the state is a live view: its values now)
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        # north 3 km and 8 km - twice round an orbit there - then 3 km east of it
        points = [fsim.Waypoint(lat + 3000.0 / R, lon), fsim.Waypoint(lat + 8000.0 / R, lon, kind="loiter_point"),
                  fsim.Waypoint(lat + 8000.0 / R, lon + east(3000.0))]
        orbit = fsim.RouteLoiter(1, pattern="orbit", orbits=2)
        a = v.submit_route(points, loiters=[orbit])
        back = a.setpoint()
        self.assertEqual(len(back.kwargs["loiters"]), 1)
        held = back.kwargs["loiters"][0]
        self.assertEqual((held.point, held.pattern, held.orbits), (1, float(fsim.PatternKind.ORBIT), 2.0))
        self.assertTrue(math.isnan(held.latitude_rad) and math.isnan(held.altitude_m))  # (its place its point's)
        radius = held.radius_m
        self.assertGreater(radius, 100.0)
        self.assertEqual([p.kind for p in a.end_points(3)][1], fsim.EndPointKind.LOITER_POINT)
        # flown: round the point from where it met it, its time to go told while it loiters, then on east
        centre = (lat + 8000.0 / R, lon)
        loitered, time_to_go, swept, last, t0 = False, None, 0.0, None, w.time
        while w.time - t0 < 460.0 and a.state != fsim.ActivityState.COMPLETED:
            w.step(int(round(1.0 / w.step_seconds)))
            p = a.progress
            if p.segment == 1 and p.segment_percent == 100.0:
                loitered = True
                s = v.state
                n, e = (s.latitude_rad - centre[0]) * R, (s.longitude_rad - centre[1]) * R * math.cos(lat)
                bearing = math.atan2(e, n)
                if last is not None:
                    swept += math.remainder(bearing - last, 2.0 * math.pi)
                last = bearing
                if time_to_go is None and swept > math.pi:
                    time_to_go = p.time_to_go_s
        self.assertTrue(loitered)
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertGreater(swept / (2.0 * math.pi), 1.95)
        self.assertGreater(time_to_go, 60.0)
        # an UPDATE: new waypoints with their loiter once round; a batch's and a task's route with its loiters
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        points = [fsim.Waypoint(lat + 3000.0 / R, lon), fsim.Waypoint(lat + 8000.0 / R, lon, kind="loiter_point"),
                  fsim.Waypoint(lat + 8000.0 / R, lon + east(3000.0))]
        b = v.submit_route(points, loiters=[orbit])
        b.update_route(points, loiters=[orbit._replace(orbits=1)])
        self.assertEqual(b.setpoint().kwargs["loiters"][0].orbits, 1.0)
        b.update_route(projection="rhumb")  # (its options alone: its waypoints and their loiters kept)
        self.assertEqual(b.setpoint().kwargs["loiters"][0].orbits, 1.0)
        b.cancel()
        batched = v.submit_batch([fsim.BatchCommand("submit_route", points, loiters=[orbit])])[0]
        self.assertEqual(batched.setpoint().kwargs["loiters"][0].orbits, 2.0)
        batched.cancel()
        v.store_task(7, fsim.BatchCommand("submit_route", points, loiters=[{"point": 1, "pattern": "hold", "duration_s": 120.0}]))
        tasked = v.command_task(7)
        self.assertEqual(tasked.setpoint().kwargs["loiters"][0].pattern, float(fsim.PatternKind.HOLD))
        tasked.cancel()
        # a loiter point with none: refused, naming it; a radius of 20 m held to the tightest it flies, named by its point
        # and its field (100 + 5)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 1))
        c = v.submit_route(points, loiters=[orbit._replace(radius_m=20.0)])
        self.assertTrue(c.clamped)
        adjustment = w.last_command_details()[1][0]
        self.assertEqual((adjustment.index, adjustment.field, adjustment.requested), (1, 100 + 5, 20.0))


if __name__ == "__main__":
    unittest.main()
