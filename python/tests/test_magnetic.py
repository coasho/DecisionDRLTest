"""The magnetic model and reference through Python (docs/flight-autonomy.md, 4.22; A-GRA's MA_HeadingReferenceEnum and
MagneticHeading): the World Magnetic Model 2025 against its technical report's test values, a world's date, and an hsa
flown from magnetic north."""
import math
import unittest

import fsim


def make_world():
    return fsim.World("py-magnetic", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class MagneticTest(unittest.TestCase):
    def test_the_model(self):
        # the technical report's: 2025.0 at sea level, 80 N 0 E; and 2027.5 at 100 km, 80 S 240 E
        f = fsim.magnetic_field(math.radians(80.0), 0.0, 0.0, 2025.0)
        self.assertIsInstance(f, fsim.MagneticField)
        self.assertAlmostEqual(f.north_nt, 6521.6, delta=0.05)
        self.assertAlmostEqual(math.degrees(f.declination_rad), 1.28, delta=0.005)
        g = fsim.magnetic_field(math.radians(-80.0), math.radians(240.0), 100000.0, 2027.5)
        self.assertAlmostEqual(g.total_nt, 51825.7, delta=0.05)
        self.assertAlmostEqual(math.degrees(g.inclination_rad), -72.10, delta=0.005)
        with self.assertRaises(ValueError):
            fsim.magnetic_field(2.0, 0.0, 0.0, 2025.0)
        self.assertEqual(fsim.decimal_year(1735689600.0), 2025.0)

    def test_a_worlds_date_and_a_magnetic_heading(self):
        w = make_world()
        self.assertEqual(w.magnetic_year, 2025.0)  # (its clock never set)
        w.set_environment(epoch_utc_seconds=1767225600.0)  # 2026-01-01
        self.assertAlmostEqual(w.magnetic_year, 2026.0, delta=1e-9)
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                             heading_deg=90.0)
        v.submit_hsa(heading_rad=0.0, direction_reference=fsim.DirectionReference.MAGNETIC_NORTH)
        w.step(int(round(60.0 / w.step_seconds)))
        d = v.state_data()
        s = v.state
        expected = fsim.magnetic_field(s.latitude_rad, s.longitude_rad, s.altitude_msl_m, w.magnetic_year).declination_rad
        self.assertAlmostEqual(d.declination_rad, expected, delta=1e-12)
        self.assertGreater(math.degrees(d.declination_rad), 12.0)  # (off San Francisco)
        # (its velocity level's heading: the declination's, and beside it the trim its hsa takes out what its loops leave
        # with - 1.5 deg on the stock c172x)
        self.assertAlmostEqual(math.remainder(v.commanded.heading_rad - d.declination_rad, 2 * math.pi), 0.0, delta=math.radians(2.0))
        self.assertAlmostEqual(math.remainder(d.magnetic_heading_rad - (s.euler_rad[2] - d.declination_rad), 2 * math.pi), 0.0, delta=1e-9)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=0.0, direction_reference=2)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 7))


if __name__ == "__main__":
    unittest.main()
