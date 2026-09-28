"""A flight's endurance through Python (docs/flight-autonomy.md, 4.18; A-GRA's VIOLATION_ENDURANCE): a flight with an end
needs no more than the vehicle has above its reserve - refused "insufficient_endurance", with what it needs against what
the vehicle has; a soft rejection, which override_rejection overrides."""
import math
import unittest

import fsim
from fsim import agra

R = 6371000.0


def make_world():
    return fsim.World("py-endurance", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class EnduranceTest(unittest.TestCase):
    def test_a_route_beyond_its_fuel(self):
        w = make_world()
        v = w.create_vehicle("cessna", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=50.0)
        w.step(30)
        s = v.state
        far = [fsim.Waypoint(s.latitude_rad + 3.0e6 / R, s.longitude_rad, altitude_m=1500.0, speed=50.0, speed_reference="true_airspeed")]
        near = [fsim.Waypoint(s.latitude_rad + 2.0e4 / R, s.longitude_rad, altitude_m=1500.0, speed=50.0, speed_reference="true_airspeed")]
        # refused, what it needs against what the vehicle has above its reserve
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(far)
        e = refused.exception.endurance
        self.assertEqual(refused.exception.reason, "insufficient_endurance")
        self.assertIsInstance(e, fsim.Endurance)
        self.assertEqual(e.energy, fsim.Energy.FUEL)
        nav = v.navigation_report()
        self.assertAlmostEqual(e.remaining, nav.remaining - nav.reserve, delta=1e-6 * nav.remaining)
        self.assertGreater(e.required, e.remaining)
        self.assertAlmostEqual(e.required_s, 3.0e6 / 50.0, delta=600.0)
        # validated: the same answer, and the same numbers
        check = v.submit_route(far, validate_only=True)
        self.assertEqual((check.valid, check.reason), (False, "insufficient_endurance"))
        self.assertEqual(check.endurance, e)
        self.assertEqual(agra.validation_results(check), ["VIOLATION_ENDURANCE"])
        self.assertEqual(agra.cannot_comply(check.reason), "CONSTRAINT_ENDURANCE")
        a = agra.insufficient_endurance(e, capacity=nav.capacity)
        self.assertEqual(a["EnduranceRemaining"]["LogicalOperator"], "LESS_THAN")
        self.assertAlmostEqual(a["EnduranceRequired"]["Fuel"], e.required)
        self.assertAlmostEqual(a["EnduranceRemaining"]["EnduranceRemaining"]["Percent"], 100.0 * e.remaining / nav.capacity)
        # overridden: flown, and said so
        flown = v.submit_route(far, override_rejection=True)
        self.assertTrue(flown.overridden)
        self.assertEqual(flown.endurance, e)
        # within its fuel: nothing said
        within = v.submit_route(near)
        self.assertFalse(within.overridden)
        self.assertIsNone(within.endurance)
        self.assertIsNone(v.submit_route(near, validate_only=True).endurance)

    def test_a_battery(self):
        w = make_world()
        v = w.create_vehicle("skua", "jsbsim:skua", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1000.0, airspeed_ms=25.0)
        v.submit(fsim.Level.VELOCITY, airspeed_ms=25.0, vertical_speed_ms=0.0, heading_rad=0.0)
        w.step(300)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(duration_s=36000.0)
        e = refused.exception.endurance
        self.assertEqual(e.energy, fsim.Energy.BATTERY)
        self.assertAlmostEqual(e.required_s, 36000.0)
        self.assertNotIn("Fuel", agra.insufficient_endurance(e)["EnduranceRequired"])
        self.assertTrue(math.isfinite(e.remaining_s))


if __name__ == "__main__":
    unittest.main()
