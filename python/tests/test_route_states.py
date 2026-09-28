"""A route's planned inertial states as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.34): a
segment flown through a state's altitude at its time, what else the plan gives kept and read back (its altitude reference
completed, by name); in a batch, and gone with an UPDATE's new waypoints; refused as a point is, naming it: one off its
leg, a time on the stock C172x without performance tables."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteStatesTest(unittest.TestCase):
    def test_through_a_state_at_its_time(self):
        w = make_world("py-route-states")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        stock = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=37.7, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                                 heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad  # (the state is a live view: its values now)
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        t0 = w.time
        # a time on the stock C172x (no performance tables): not implemented, naming the point (before it has flown far on
        # its own)
        slat, slon = stock.state.latitude_rad, stock.state.longitude_rad
        timed = fsim.RouteState(1, slat, slon + east(5500.0), time_s=t0 + 300.0)
        with self.assertRaises(fsim.Rejected) as refused:
            stock.submit_route([fsim.Waypoint(slat, slon + east(3000.0)), fsim.Waypoint(slat, slon + east(8000.0))], states=[timed])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("not_implemented", 1))
        # east 3 km at 50 m/s, then 5 km on at 1500 m; 5.5 km east, 1550 m at 140 s (at 50 m/s it would be there at 110 s:
        # slowed); before it, 1.5 km east, a yaw alone
        points = [fsim.Waypoint(lat, lon + east(3000.0), 1500.0, speed=50.0), fsim.Waypoint(lat, lon + east(8000.0), 1500.0)]
        states = [fsim.RouteState(0, lat, lon + east(1500.0), yaw_rad=0.5 * math.pi),
                  fsim.RouteState(1, lat, lon + east(5500.0), 1550.0, "msl", time_s=t0 + 140.0)]
        a = v.submit_route(points, states=states)
        back = a.setpoint().kwargs["states"]
        self.assertEqual(len(back), 2)
        self.assertEqual(back[0].yaw_rad, 0.5 * math.pi)
        self.assertTrue(math.isnan(back[0].time_s))
        self.assertEqual((back[1].point, back[1].altitude_m, back[1].time_s), (1, 1550.0, t0 + 140.0))
        self.assertEqual(back[1].altitude_reference, float(fsim.AltitudeReference.MSL))
        # flown: past it within 2 s of its time and 25 m of its altitude, its estimate telling that time
        estimate, passed, high = None, None, None
        while w.time - t0 < 250.0 and a.state != fsim.ActivityState.COMPLETED:
            w.step(int(round(1.0 / w.step_seconds)) // 4 or 1)
            done = (v.state.longitude_rad - lon) * R * math.cos(lat)
            p = a.progress
            if estimate is None and done > 4000.0 and not math.isnan(p.arrival_s):
                estimate = p.arrival_s - t0
            if passed is None and done >= 5500.0:
                passed, high = w.time - t0, v.state.altitude_msl_m
        self.assertIsNotNone(passed)
        self.assertLess(abs(passed - 140.0), 2.0)
        self.assertLess(abs(high - 1550.0), 25.0)
        self.assertLess(abs(estimate - 140.0), 0.5)
        # from where it is now: a batch's, read back; an UPDATE's new waypoints come with theirs (none)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        points = [fsim.Waypoint(lat, lon + east(3000.0)), fsim.Waypoint(lat, lon + east(8000.0))]
        batched = v.submit_batch([fsim.BatchCommand("submit_route", points, states=[{"point": 1, "latitude_rad": lat, "longitude_rad": lon + east(5500.0),
                                                                                      "pitch_rad": 0.05}])])[0]
        self.assertEqual(batched.setpoint().kwargs["states"][0].pitch_rad, 0.05)
        batched.update_route(points)
        self.assertNotIn("states", batched.setpoint().kwargs)
        # refused, naming the point: a kilometre off its leg
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points, states=[fsim.RouteState(1, lat + 1000.0 / R, lon + east(5500.0))])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 1))


if __name__ == "__main__":
    unittest.main()
