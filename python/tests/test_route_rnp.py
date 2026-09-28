"""A route's required navigation performance as A-GRA's schema gives it, through Python (docs/flight-autonomy.md, 4.35): a
segment's rnp_m, read back; the route farther off its path than it, its activity's constraints say so
(fsim.ActivityFlag.NAVIGATION_PERFORMANCE) - A-GRA's ACTIVE_PARTIALLY_CONSTRAINED - and remember it; refused as a point
is, naming it."""
import math
import unittest

import fsim
from fsim import agra

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteRnpTest(unittest.TestCase):
    def test_off_its_rnp(self):
        w = make_world("py-route-rnp")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        # north 3 km, then 3 km on - a quarter turn left onto its first leg - within 20 m of its path
        points = [fsim.Waypoint(lat + 3000.0 / R, lon, rnp_m=20.0), fsim.Waypoint(lat + 6000.0 / R, lon, rnp_m=20.0)]
        a = v.submit_route(points)
        self.assertEqual(a.setpoint().args[0][0].rnp_m, 20.0)
        w.step(int(round(5.0 / w.step_seconds)))
        info = a.info
        self.assertTrue(info.constraints & fsim.ActivityFlag.NAVIGATION_PERFORMANCE)
        self.assertTrue(info.constraints_seen & fsim.ActivityFlag.NAVIGATION_PERFORMANCE)
        self.assertEqual(agra.activity_state(info), "ACTIVE_PARTIALLY_CONSTRAINED")
        # refused, naming the point: an RNP of 0
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route([fsim.Waypoint(lat, lon + east(3000.0)), fsim.Waypoint(lat, lon + east(8000.0), rnp_m=0.0)])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 1))


if __name__ == "__main__":
    unittest.main()
