"""A route's waypoints as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.29): an altitude block, a
waypoint and its type, a point in a frame - by name, read back - a ship's route flown over the ship, and what is not
built yet refused, naming the point."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def cessna(w, name="cessna"):
    return w.create_vehicle(name, "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1000.0, airspeed_ms=55.0,
                            heading_deg=0.0)


class RoutePointsTest(unittest.TestCase):
    def test_read_back_flown_over_a_ship_and_refused(self):
        w = make_world("py-route-points")
        v = cessna(w)
        w.step(10)
        lat, lon, alt = v.state.latitude_rad, v.state.longitude_rad, v.state.altitude_msl_m  # (the state is a live view: its values now)
        # a block, a waypoint by its type alone, a point in a fixed frame 1 km east of its origin
        here = w.create_frame("fixed", latitude_rad=lat + 6000.0 / R, longitude_rad=lon)
        a = v.submit_route([fsim.Waypoint(lat + 3000.0 / R, lon, altitude_min_m=alt + 100.0, altitude_max_m=alt + 300.0, waypoint_type="passive"),
                            fsim.Waypoint(0.0, 0.0, frame=here, frame_y_m=1000.0)])
        points = a.setpoint().args[0]
        self.assertEqual(points[0].altitude_m, alt + 100.0)  # (left out: the aircraft's, held up to its least)
        self.assertEqual((points[0].kind, points[0].waypoint_type), (float(fsim.EndPointKind.WAYPOINT), float(fsim.WaypointType.PASSIVE)))
        self.assertEqual(points[1].frame, float(here))
        self.assertAlmostEqual((points[1].longitude_rad - lon) * R * math.cos(lat + 6000.0 / R), 1000.0, delta=1.0)  # (placed there)
        # named at its point: a ditch, not built yet (FA-16); a loiter point with no loiter beside it, no point (4.31)
        for fields, why in (({"waypoint_type": fsim.WaypointType.HARD_DITCH}, "not_implemented"), ({"kind": "loiter_point"}, "invalid_waypoint")):
            with self.assertRaises(fsim.Rejected) as refused:
                v.submit_route([fsim.Waypoint(lat + 3000.0 / R, lon), fsim.Waypoint(lat + 6000.0 / R, lon, **fields)])
            self.assertEqual((refused.exception.reason, refused.exception.index), (why, 1), fields)
        with self.assertRaises(fsim.Rejected) as refused:  # (a frame the world does not have)
            v.submit_route([fsim.Waypoint(0.0, 0.0, frame=here + 100)])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 0))
        # round a ship moving north at 8 m/s, a box 3 km a side about it: flown over it, within its legs where it is
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        ship = w.create_frame("moving", latitude_rad=lat + 4000.0 / R, longitude_rad=lon, north_ms=8.0, time_s=w.time)
        box = [(1500.0, 1500.0), (-1500.0, 1500.0), (-1500.0, -1500.0), (1500.0, -1500.0)]
        b = v.submit_route([fsim.Waypoint(0.0, 0.0, frame=ship, frame_x_m=x, frame_y_m=y) for x, y in box], repeat=1)
        t0 = w.time
        worst = 0.0
        while w.time - t0 < 360.0:
            w.step(int(round(1.0 / w.step_seconds)))
            p = b.progress
            if p.laps < 1 or not 33.0 < p.segment_percent < 67.0:
                continue
            s = v.state
            north = (s.latitude_rad - (lat + 4000.0 / R + 8.0 * (w.time - t0) / R)) * R  # (the ship's origin now)
            east = (s.longitude_rad - lon) * R * math.cos(lat)
            (n1, e1), (n0, e0) = box[p.segment], box[(p.segment + 3) % 4]
            length = math.hypot(n1 - n0, e1 - e0)
            worst = max(worst, abs(-(north - n0) * (e1 - e0) / length + (east - e0) * (n1 - n0) / length))
        self.assertEqual(b.state, fsim.ActivityState.ACTIVE)
        self.assertLess(worst, 40.0)  # (off the leg where the ship is now: over the ship)
        self.assertGreater(worst, 0.0)  # (measured)


if __name__ == "__main__":
    unittest.main()
