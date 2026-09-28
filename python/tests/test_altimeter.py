"""The barometric altimeter through Python (docs/flight-autonomy.md, 4.20; A-GRA's QNH setting, VI 1.2.6.5, and
MA_AirDataType, VI 1.2.6.8): the QNH set or refused, what the altimeter reads, a barometric altitude flown on its
isobar in air that is not standard."""
import math
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-altimeter", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class AltimeterTest(unittest.TestCase):
    def test_the_setting_and_what_it_reads(self):
        w = make_world()
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=1500.0, airspeed_ms=55.0)
        self.assertEqual(v.qnh, 101325.0)  # until set: the pressure altitude
        d = v.state_data()
        self.assertIsInstance(d, fsim.StateData)
        self.assertEqual(d.kollsman_hpa, 1013.25)
        self.assertAlmostEqual(d.indicated_altitude_m, 1500.0, delta=1.0)  # (its geopotential height: 0.35 m less)
        # set in A-GRA's kPa: applied, or failed with a reason
        self.assertEqual(agra.apply_qnh_setting(v, 100.0), {"RequestProcessingState": "COMPLETED"})
        self.assertEqual(v.qnh, 100000.0)
        self.assertAlmostEqual(v.state_data().indicated_altitude_m, d.indicated_altitude_m - 110.9, delta=0.2)
        failed = agra.apply_qnh_setting(v, 80.0)
        self.assertEqual(failed["RequestProcessingState"], "FAILED")
        self.assertIn("out_of_range", failed["RequestProcessingStateReason"])
        self.assertEqual(v.qnh, 100000.0)
        with self.assertRaises(fsim.Error):
            v.set_qnh(float("nan"))
        a = agra.air_data(v.state, v.state_data())
        self.assertEqual(a["Kollsman"], 1000.0)
        self.assertAlmostEqual(a["TrueAirspeed"], v.state.airspeed_true_ms)
        self.assertTrue(math.isfinite(a["BarometricAltitudeRate"]))
        # a route's barometric altitude taken (ADR-29 FA-6a: flown on its isobar in test_route_points)
        s = fsim.VehicleState.from_buffer_copy(v.state)
        r = v.submit_route([fsim.Waypoint(s.latitude_rad + 0.01, s.longitude_rad, altitude_m=1500.0, altitude_reference="barometric")])
        self.assertEqual(r.setpoint().args[0][0].altitude_reference, float(fsim.AltitudeReference.BAROMETRIC))

    def test_flown_on_its_isobar(self):
        w = make_world()
        w.set_environment(temperature_sl_k=303.15, pressure_sl_pa=102000.0)  # warm, and high
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=2000.0, airspeed_ms=55.0)
        v.set_qnh(102000.0)
        v.submit_hsa(heading_rad=0.5 * math.pi, altitude_m=2000.0, altitude_reference="barometric")
        w.step(int(round(150.0 / w.step_seconds)))
        self.assertAlmostEqual(v.state_data().indicated_altitude_m, 2000.0, delta=10.0)
        self.assertGreater(v.state.altitude_msl_m - 2000.0, 80.0)  # (15 K warm: its isobar 100 m up)


if __name__ == "__main__":
    unittest.main()
