"""The navigation report through Python (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport): what a vehicle
flies on and how much it has, its endurance, its playtime to a recovery point and its contingency."""
import math
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-navigation", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def cessna(w, name="cessna"):
    v = w.create_vehicle(name, "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=50.0,
                         heading_deg=90.0)
    v.submit(fsim.Level.VELOCITY, airspeed_ms=50.0, vertical_speed_ms=0.0, heading_rad=math.pi / 2)
    return v


class NavigationTest(unittest.TestCase):
    def test_fuel_and_the_playtime_to_a_recovery_point(self):
        w = make_world()
        v = cessna(w)
        w.step(600)  # (20 s: some fuel burnt, below full)
        r = v.navigation_report()
        self.assertIsInstance(r, fsim.NavigationReport)
        self.assertEqual(r.energy, fsim.Energy.FUEL)
        self.assertEqual(r.fuel_kg, r.remaining)
        self.assertGreater(r.remaining, 0.0)
        self.assertAlmostEqual(r.percent, 100.0 * r.remaining / r.capacity)
        self.assertAlmostEqual(r.endurance_s, r.remaining / r.consumption, delta=1e-9 * r.endurance_s)
        self.assertAlmostEqual(r.reserve, 0.1 * r.capacity)
        self.assertTrue(math.isnan(r.playtime_s))
        self.assertEqual(r.contingency, fsim.Contingency.NORMAL)
        self.assertFalse(r.starved)
        # a recovery point behind it: the playtime to it, less than its endurance
        v.set_recovery(40.0, -0.35, 300.0)
        n = v.navigation()
        self.assertIsInstance(n, fsim.NavigationSettings)
        self.assertEqual((n.recovery, n.latitude_deg, n.longitude_deg, n.reserve_fraction), (True, 40.0, -0.35, 0.1))
        r = v.navigation_report()
        self.assertGreater(r.return_distance_m, 25000.0)
        self.assertGreater(r.return_tas_ms, 0.0)
        self.assertGreater(r.playtime_s, 0.0)
        self.assertLess(r.playtime_s, r.endurance_s)
        # in A-GRA's names
        m = agra.navigation_report(r)
        self.assertEqual(m["Endurance"], {"Fuel": r.fuel_kg, "Percent": r.percent, "Duration": r.endurance_s})
        self.assertEqual((m["Playtime"], m["ContingencyLevel"]), (r.playtime_s, "NORMAL"))
        # a reserve of all it has: flight critical, no playtime
        v.set_recovery(40.0, -0.35, 300.0, reserve_fraction=min(0.9999, r.remaining / r.capacity + 1e-4))
        r = v.navigation_report()
        self.assertEqual(r.contingency, fsim.Contingency.FLIGHT_CRITICAL)
        self.assertEqual(r.playtime_s, 0.0)
        # cleared: no playtime, the reserve as it was
        v.clear_recovery()
        self.assertFalse(v.navigation().recovery)
        self.assertTrue(math.isnan(v.navigation_report().playtime_s))
        self.assertIsNone(agra.navigation_report(v.navigation_report())["Playtime"])
        self.assertGreater(v.navigation().reserve_fraction, 0.99)
        v.clear_recovery(reserve_fraction=0.2)
        self.assertEqual(v.navigation().reserve_fraction, 0.2)

    def test_its_tanks_emptied(self):
        w = make_world()
        v = cessna(w)
        w.step(5)
        for i in range(2):
            v.set_property("propulsion/tank[%d]/contents-lbs" % i, 0.0)
        w.step(10)
        r = v.navigation_report()
        self.assertEqual(r.remaining, 0.0)
        self.assertTrue(r.starved)
        self.assertEqual(r.endurance_s, 0.0)
        self.assertEqual(r.contingency, fsim.Contingency.FLIGHT_CRITICAL)

    def test_a_battery(self):
        w = make_world()
        v = w.create_vehicle("cf2", "jsbsim:cf2", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=100.0, airspeed_ms=0.0)
        v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0, heading_rad=0.0)
        w.step(60)
        r = v.navigation_report()
        self.assertEqual(r.energy, fsim.Energy.BATTERY)
        self.assertEqual(r.fuel_kg, 0.0)
        self.assertGreater(r.capacity, 3000.0)  # J
        self.assertGreater(r.consumption, 5.0)  # W
        self.assertLess(r.consumption, 10.0)
        self.assertAlmostEqual(r.endurance_s, r.remaining / r.consumption, delta=1e-9 * r.endurance_s)

    def test_what_it_refuses(self):
        w = make_world()
        v = cessna(w)
        with self.assertRaises(fsim.Error):
            v.set_recovery(95.0, 0.0, 300.0)
        with self.assertRaises(fsim.Error):
            v.set_recovery(40.0, 0.0, 300.0, reserve_fraction=1.0)
        with self.assertRaises(fsim.Error):
            v.clear_recovery(reserve_fraction=-0.1)
        self.assertFalse(v.navigation().recovery)  # (as it was)
        self.assertEqual(v.navigation().reserve_fraction, 0.1)
        self.assertEqual(fsim.Contingency.LOST_COMMS, 3)
        self.assertEqual(fsim.Energy.UNKNOWN, 0)


if __name__ == "__main__":
    unittest.main()
