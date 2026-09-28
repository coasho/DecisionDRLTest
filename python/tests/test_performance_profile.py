"""A flight mode's performance profile through Python (docs/flight-autonomy.md, 4.15; A-GRA's
MA_FlightControlModesPerformanceProfileType): the airspeeds against altitude, the altitude limits, the accelerations,
excess power, descents, decelerations and burn, the attitudes, rates, turn and climb, at the vehicle's condition now."""
import math
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-profile", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class PerformanceProfileTest(unittest.TestCase):
    def test_a_designed_wing(self):
        w = make_world()
        v = w.create_vehicle("cessna", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=50.0,
                             heading_deg=90.0)
        v.submit(fsim.Level.VELOCITY, airspeed_ms=50.0, vertical_speed_ms=0.0, heading_rad=math.pi / 2)
        w.step(300)
        p = v.performance_profile()
        self.assertIsInstance(p, fsim.PerformanceProfile)
        self.assertEqual((p.mode, p.energy, p.clean), ("hsa_csa", fsim.Energy.FUEL, True))
        self.assertGreater(len(p.max_airspeed), 2)
        for lo, hi in zip(p.min_airspeed, p.max_airspeed):
            self.assertIsInstance(hi, fsim.ProfilePoint)
            self.assertEqual(lo.altitude_msl_m, hi.altitude_msl_m)
            self.assertLess(lo.value, hi.value)
            self.assertTrue(math.isnan(hi.tas_ms))
        self.assertGreater(p.max_altitude_msl_m, 3000.0)
        self.assertTrue(math.isnan(p.min_altitude_msl_m))
        self.assertIsInstance(p.excess_power[0], fsim.ProfileExcessPower)
        self.assertGreater(p.excess_power[0].climb_ms, 0.0)
        self.assertTrue(all(b.value > 0.0 for b in p.burn))
        self.assertIsInstance(p.max_orientation[0], fsim.ProfileOrientation)
        self.assertAlmostEqual(p.max_orientation[0].roll_rad, math.radians(45.0))
        self.assertGreater(p.max_turn_rate_rad_s, 0.0)
        for mode in ("waypoint_following", "curve_following"):
            self.assertEqual(v.performance_profile(mode).mode, mode)
        # in A-GRA's names: true airspeeds, altitudes above sea level, the schema's gaps filled
        a = agra.performance_profile(p)
        self.assertEqual(a["MaxAirspeed"][0]["AirspeedLimit"], {"Value": p.max_airspeed[0].value, "Reference": "TRUE_AIRSPEED"})
        self.assertEqual(a["MaxAirspeed"][0]["AltitudePair"], {"AltitudeReference": "MSL", "Altitude": p.max_airspeed[0].altitude_msl_m})
        self.assertIsNone(a["MinAltitude"])
        self.assertEqual(a["FuelBurnRate"][0]["Endurance"], {"Fuel": p.burn[0].value * 3600.0, "Duration": 3600.0})
        self.assertEqual(a["ExcessPowerOrAcceleration"]["ExcessPower"][0]["ExcessPowerMaxClimb"]["Value"], p.excess_power[0].climb_ms)
        self.assertEqual(a["MaxOrientationLimits"][0]["OrientationLimits"]["Roll"], p.max_orientation[0].roll_rad)
        self.assertIsNone(a["MaxOrientationLimits"][0]["OrientationLimits"]["Yaw"])

    def test_a_quadrotors_battery(self):
        w = make_world()
        v = w.create_vehicle("iris", "jsbsim:iris", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=100.0, airspeed_ms=0.0)
        v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0, heading_rad=0.0)
        w.step(60)
        p = v.performance_profile("waypoint_following")
        self.assertEqual(p.energy, fsim.Energy.BATTERY)
        self.assertEqual(p.min_airspeed[0].value, 0.0)  # (it hovers)
        self.assertGreater(p.burn[0].value, 100.0)      # W
        self.assertEqual(p.excess_power, [])
        capacity = v.navigation_report().capacity
        a = agra.performance_profile(p, capacity=capacity)
        self.assertAlmostEqual(a["FuelBurnRate"][0]["Endurance"]["Percent"], 100.0 * p.burn[0].value * 3600.0 / capacity)
        self.assertIsNone(a["ExcessPowerOrAcceleration"])

    def test_what_has_none(self):
        w = make_world()
        v = w.create_vehicle("cessna", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=50.0)
        for mode in ("loiter", "no such mode"):
            with self.assertRaises(fsim.Rejected) as caught:
                v.performance_profile(mode)
            self.assertEqual(caught.exception.reason, "invalid_parameter")


if __name__ == "__main__":
    unittest.main()
