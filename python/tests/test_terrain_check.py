"""The terrain through Python (docs/flight-autonomy.md, 4.19; A-GRA's VIOLATION_TERRAIN and TerrainConstraint, VI 1.2.6.9's
elevation request): a path into the ground is refused "terrain_conflict", with where it would meet it and when - never
overridden - and the ground the physics has is asked for."""
import math
import unittest

import fsim
from fsim import agra

R = 6371008.8  # the platform's mean radius


def make_world():
    return fsim.World("py-terrain", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class TerrainTest(unittest.TestCase):
    def test_a_path_into_the_ground(self):
        w = make_world()
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=500.0, airspeed_ms=55.0,
                             heading_deg=0.0)
        s = fsim.VehicleState.from_buffer_copy(v.state)
        # an hsa under the ground (flat, at sea level): where it is, at once
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(altitude_m=-50.0)
        t = refused.exception.terrain
        self.assertEqual(refused.exception.reason, "terrain_conflict")
        self.assertIsInstance(t, fsim.TerrainPoint)
        self.assertAlmostEqual(t.latitude_rad, s.latitude_rad, delta=1e-9)
        self.assertEqual((t.altitude_msl_m, t.ground_m, t.index), (-50.0, 0.0, -1))
        self.assertAlmostEqual(t.time_s, 0.0)
        # a route down into it, as gently as it descends: halfway along the second point's leg - 5.5 km north, 100 s on
        def route(north_m):
            return [fsim.Waypoint(s.latitude_rad + 500.0 / R, s.longitude_rad, altitude_m=500.0, speed=55.0, speed_reference="true_airspeed"),
                    fsim.Waypoint(s.latitude_rad + north_m / R, s.longitude_rad, altitude_m=-500.0)]
        points = route(10500.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points)
        t = refused.exception.terrain
        self.assertEqual((refused.exception.index, t.index), (1, 1))
        self.assertLess(t.altitude_msl_m, 0.0)
        self.assertGreater(t.altitude_msl_m, -0.1)  # (where it goes below, to a tenth of a metre)
        self.assertAlmostEqual((t.latitude_rad - s.latitude_rad) * R, 5500.0, delta=1.0)
        self.assertAlmostEqual(t.time_s, 5500.0 / 55.0, delta=0.05)
        # too steep to fly as asked: clamped to its steepest descent, it is still going down past the point, into it there
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(route(2500.0))
        steep = refused.exception.terrain
        self.assertEqual(steep.index, 1)
        self.assertGreater((steep.latitude_rad - s.latitude_rad) * R, 2600.0)
        self.assertGreater(steep.altitude_msl_m, -0.1)
        self.assertGreater(steep.time_s, 2500.0 / 55.0)
        # validated: the same answer; in A-GRA's terms
        check = v.submit_route(points, validate_only=True)
        self.assertEqual((check.valid, check.reason), (False, "terrain_conflict"))
        self.assertEqual(check.terrain, t)
        self.assertIsNone(check.endurance)
        self.assertEqual(agra.validation_results(check), ["VIOLATION_TERRAIN"])
        self.assertEqual(agra.cannot_comply(check.reason), "CONSTRAINT_SAFETY")
        now = w.environment["epoch_utc_seconds"] + w.time
        point = agra.terrain_constraint(t, now)
        self.assertEqual((point["Latitude"], point["Longitude"], point["Altitude"]), (t.latitude_rad, t.longitude_rad, t.altitude_msl_m))
        self.assertTrue(point["Timestamp"].endswith("Z"))
        self.assertNotIn("Timestamp", agra.terrain_constraint(t))
        # nothing overrides it
        with self.assertRaises(fsim.Rejected):
            v.submit_route(points, override_rejection=True)
        # clear of it: flown, and nothing said
        high = v.submit_hsa(altitude_m=300.0)
        self.assertIsNone(v.submit_hsa(altitude_m=300.0, validate_only=True).terrain)
        # an UPDATE into it: refused, flown on as it was
        with self.assertRaises(fsim.Rejected) as refused:
            high.update(altitude_m=-50.0)
        self.assertEqual(refused.exception.reason, "terrain_conflict")
        self.assertIsNotNone(refused.exception.terrain)

    def test_the_ground_asked_for(self):
        w = make_world()
        self.assertEqual(w.terrain(0.7, 0.1), 0.0)
        self.assertEqual(w.terrain([0.7, -0.2], [0.1, 2.0]), [0.0, 0.0])
        self.assertEqual(w.terrain([], []), [])
        status = agra.elevation_request_status([0.7, -0.2], [0.1, 2.0], [0.0, None])
        self.assertEqual(status["RequestProcessingState"], "COMPLETED")
        self.assertEqual(status["ElevationReturned"]["RequestPoint"][0], {"Latitude": 0.7, "Longitude": 0.1, "Altitude": 0.0})
        self.assertNotIn("Altitude", status["ElevationReturned"]["RequestPoint"][1])
        self.assertEqual(agra.elevation_request_status([0.7], [0.1], [None])["RequestProcessingState"], "FAILED")
        with self.assertRaises(ValueError):
            w.terrain([0.7, 0.1], [0.1])
        self.assertTrue(math.isfinite(w.terrain(-0.5, 3.0)))


if __name__ == "__main__":
    unittest.main()
