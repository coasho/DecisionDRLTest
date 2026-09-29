"""An altitude stacked marshall through Python (docs/flight-autonomy.md, 4.46): two aircraft round one point given the lowest two
slots and holding them; a third refused a full stack; an UPDATE that leaves the stack keeping its slot; the hover's row."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


class MarshallTest(unittest.TestCase):
    def setUp(self):
        self.w = fsim.World("py-marshall", publish=False, workers=1, pin_workers=False, seed=5, terrain=False)
        self.v = [self.w.create_vehicle("m%d" % i, "jsbsim:c172x", latitude_deg=37.6 + 0.01 * i, longitude_deg=-122.4,
                                        altitude_msl_m=1500.0 + 300.0 * i, airspeed_ms=55.0, heading_deg=90.0) for i in range(3)]
        self.w.step(10)

    def test_stack(self):
        w, v = self.w, self.v
        s = v[0].state
        lat, lon = s.latitude_rad + 3000.0 / R, s.longitude_rad
        stack = dict(latitude_rad=lat, longitude_rad=lon, radius_m=1500.0, altitude_min_m=1500.0, altitude_max_m=1900.0, separation_m=300.0)
        a = v[0].submit_marshall(**stack)
        b = v[1].submit_marshall(**stack)
        self.assertEqual((a.setpoint().kwargs["altitude_m"], b.setpoint().kwargs["altitude_m"]), (1500.0, 1800.0))
        self.assertEqual(a.setpoint().method, "submit_marshall")
        with self.assertRaises(fsim.Rejected) as refused:
            v[2].submit_marshall(**stack)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("stack_full", 3))
        # its speed alone: its stack left, its slot kept
        b.update(speed=60.0)
        self.assertEqual(b.setpoint().kwargs["altitude_m"], 1800.0)
        for _ in range(60):
            w.step(30)  # (a minute)
        self.assertEqual((a.state, b.state), (fsim.ActivityState.ACTIVE, fsim.ActivityState.ACTIVE))
        self.assertLess(abs(v[0].state.altitude_msl_m - 1500.0), 5.0)
        self.assertLess(abs(v[1].state.altitude_msl_m - 1800.0), 5.0)
        self.assertEqual(v[0].support("fsim.guidance.marshall").support, fsim.Support.SUPPORTED)
        # a hold is ATC's, not a marshall's
        with self.assertRaises(fsim.Rejected) as refused:
            v[2].submit_marshall(pattern="hold", latitude_rad=lat + 0.01, longitude_rad=lon, altitude_min_m=1500.0)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 0))


if __name__ == "__main__":
    unittest.main()
