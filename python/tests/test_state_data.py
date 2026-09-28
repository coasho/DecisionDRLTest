"""The state data and reference frames through Python (docs/flight-autonomy.md, 4.21; A-GRA's OrientationRate,
OrientationAcceleration, WanderAngle and wind data, VI 1.2.6.8, and its ReferenceFrame): the wind a vehicle measures,
its orientation's rates, and points in fixed, moving and vehicle frames."""
import math
import unittest

import fsim
from fsim import agra

R = 6371008.8  # the platform's mean radius


def make_world():
    return fsim.World("py-state-data", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class StateDataTest(unittest.TestCase):
    def test_the_wind_and_the_orientation(self):
        w = make_world()
        w.set_environment(wind_direction_deg=270.0, wind_speed_ms=10.0)  # (blowing east)
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=55.0)
        w.step(int(round(10.0 / w.step_seconds)))
        d = v.state_data()
        self.assertAlmostEqual(d.wind_north_ms, 0.0, delta=1e-6)
        self.assertAlmostEqual(d.wind_east_ms, 10.0, delta=1e-6)
        self.assertEqual(d.wander_angle_rad, 0.0)
        self.assertTrue(math.isfinite(d.yaw_rate_rad_s) and math.isfinite(d.roll_acceleration_rad_s2))
        self.assertEqual(agra.wind_data(d)["WindChoice"]["WindVelocity"]["EastSpeed"], d.wind_east_ms)
        self.assertEqual(set(agra.orientation_rate(d)), {"YawRate", "PitchRate", "RollRate"})
        self.assertEqual(agra.orientation_acceleration(d)["RollAccel"], d.roll_acceleration_rad_s2)

    def test_frames(self):
        w = make_world()
        f = w.create_frame("fixed", latitude_rad=0.6, longitude_rad=-2.1, altitude_msl_m=100.0)
        lat, lon, alt = w.frame_point(f, 1000.0, offsets="great_circle")
        self.assertAlmostEqual(lat, 0.6 + 1000.0 / R, delta=1e-12)
        self.assertEqual(alt, 100.0)
        m = w.create_frame(fsim.FrameOrigin.MOVING, latitude_rad=0.6, longitude_rad=-2.1, north_ms=10.0)
        self.assertAlmostEqual(w.frame_point(m, time_s=100.0)[0], 0.6 + 1000.0 / R, delta=1e-12)
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=55.0)
        fv = w.create_frame("vehicle", vehicle=v)
        lat, lon, alt = w.frame_point(fv)
        self.assertAlmostEqual(lat, v.state.latitude_rad, delta=1e-12)
        self.assertIsNotNone(w.frame_point(fv, 100.0, rotation="yaw", offsets="great_circle"))
        with self.assertRaises(fsim.Error):
            w.create_frame("vehicle", vehicle=999)
        self.assertTrue(w.remove_frame(f))
        self.assertFalse(w.remove_frame(f))
        self.assertIsNone(w.frame_point(f))


if __name__ == "__main__":
    unittest.main()
