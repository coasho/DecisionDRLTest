"""The performance tables' arithmetic (hangar/performance.py, profile.table_fields): what the flown numbers become,
each against a result it must give. Fast; no platform needed (the tables are flown by the performance stage)."""
import math
import unittest

import numpy as np

from hangar import performance as P
from hangar.profile import table_fields


def tables():
    """Two altitudes, two weights, three level points: a lift-and-drag aircraft by hand."""
    nan = float("nan")
    t = {"altitude_m": [100.0, 3000.0], "weight_kg": [900.0, 1100.0], "speed_fraction": [0.0, 0.5, 1.0], "fuel_capacity_kg": 200.0}
    t["max_climb_ms"] = [[5.0, 4.0], [1.0, 0.3]]
    t["min_tas_ms"] = [[30.0, 32.0], [35.0, nan]]
    t["max_tas_ms"] = [[60.0, 58.0], [55.0, nan]]
    t["tas_ms"] = [[[30.1, 45.0, 58.2], [32.0, 45.0, 56.3]], [[35.0, 45.0, 53.4], [nan, nan, nan]]]
    t["ps_full_ms"] = [[[3.0, 5.0, 1.0], [2.0, 4.0, 0.5]], [[0.5, 1.0, 0.2], [nan, nan, nan]]]
    return t


def run(start, v_max, held=True, seconds=1200.0):
    """A full-power run from below as the stage keeps it: level (its height constant), its speed rising steadily from
    start to v_max over `seconds`, recorded every half second - an excess power of v/g times its acceleration, a few
    m/s."""
    t = np.arange(0.0, seconds, 0.5)
    v = np.linspace(start, v_max, len(t))
    return {"t": list(t), "v": list(v), "vs": [0.0] * len(v), "h": [1000.0] * len(v), "start": start, "v_max": v_max, "held": held}


class PerformanceTest(unittest.TestCase):
    def test_the_extremum_between_its_neighbours(self):
        # a parabola's vertex, read off five samples of it: exact
        x = np.array([10.0, 20.0, 30.0, 40.0, 50.0])
        y = 2.0 + 0.01 * (x - 33.0) ** 2
        xm, ym = P._extremum(x, y, most=False)
        self.assertAlmostEqual(xm, 33.0, places=9)
        self.assertAlmostEqual(ym, 2.0, places=9)
        xm, ym = P._extremum(x, -y, most=True)
        self.assertAlmostEqual(xm, 33.0, places=9)
        # at an end of the speeds flown: that end
        self.assertEqual(P._extremum(x, x, most=False), (10.0, 10.0))

    def test_a_curve_read_to_its_ends(self):
        # linear between its points; within 3 % past either end, that end's value (a run is recorded from half a
        # second in); NaN beyond
        v, ps = np.array([100.0, 150.0, 200.0]), np.array([4.0, 6.0, 1.0])
        self.assertAlmostEqual(P._along_curve(125.0, v, ps), 5.0)
        self.assertEqual(P._along_curve(98.0, v, ps), 4.0)
        self.assertEqual(P._along_curve(205.0, v, ps), 1.0)
        self.assertTrue(math.isnan(P._along_curve(96.0, v, ps)))
        self.assertTrue(math.isnan(P._along_curve(207.0, v, ps)))

    def test_the_service_ceiling(self):
        t = tables()
        # linear between the altitudes where the best climb crosses 0.508 m/s: 5 -> 1 m/s over 100 -> 3000 m climbs
        # at both, so the highest two extended...
        self.assertAlmostEqual(P.ceiling(t, 0), 3000.0 + (1.0 - 0.508) / (5.0 - 1.0) * 2900.0, places=6)
        # ...and between them where it does: 4 -> 0.3 m/s
        self.assertAlmostEqual(P.ceiling(t, 1), 100.0 + (4.0 - 0.508) / (4.0 - 0.3) * 2900.0, places=6)
        # an altitude nothing held level at climbs nothing (as the flight tests count it): the ceiling below it...
        t["max_climb_ms"] = [[5.0, 4.0], [float("nan"), 0.3]]
        self.assertAlmostEqual(P.ceiling(t, 0), 100.0 + (5.0 - 0.508) / 5.0 * 2900.0, places=6)
        # ...never the line of the rows beneath extended past it; and none where it climbs at none
        t["max_climb_ms"] = [[0.2, 4.0], [float("nan"), 0.3]]
        self.assertTrue(math.isnan(P.ceiling(t, 0)))

    def test_the_altitudes_flown(self):
        # the ceiling's fractions, to 10 m, the lowest a little above the ground...
        self.assertEqual(P.altitudes(10000.0), [100.0, 2000.0, 4000.0, 6000.0, 7500.0, 8500.0, 9500.0])
        # ...and the height a top speed is published at among them, unless near a row or outside them
        self.assertEqual(P.altitudes(10000.0, (9144.0,)), [100.0, 2000.0, 4000.0, 6000.0, 7500.0, 8500.0, 9144.0, 9500.0])
        self.assertEqual(P.altitudes(10000.0, (7600.0, 100.0, 12000.0, float("nan"))), P.altitudes(10000.0))

    def test_lookups(self):
        t = tables()
        self.assertEqual(P.at(t, "max_tas_ms", 1550.0, 0), 57.5)
        self.assertTrue(math.isnan(P.at(t, "max_tas_ms", 5000.0, 0)))  # (above the altitudes flown)
        # in weight too: linear between the weights flown, none beyond them
        self.assertAlmostEqual(P.at_weight(t, "max_tas_ms", 100.0, 1000.0), 59.0)
        self.assertTrue(math.isnan(P.at_weight(t, "max_tas_ms", 100.0, 1200.0)))
        # the excess power along each altitude's band (each point at its fraction of [least, 97 % of top], as the
        # platform reads it: 30 .. 58.2 at 100 m), then in altitude; below the lowest, its line extended
        self.assertAlmostEqual(P.ps_at(t, 100.0, 37.05, 0), 4.0)
        band3000 = 35.0 + 0.5 * (0.97 * 55.0 - 35.0)
        self.assertAlmostEqual(P.ps_at(t, 3000.0, band3000, 0), 1.0)
        mid = 0.5 * (30.0 + 0.5 * (0.97 * 60.0 - 30.0)) + 0.5 * band3000
        self.assertTrue(np.isfinite(P.ps_at(t, 1550.0, mid, 0)))
        self.assertTrue(math.isnan(P.ps_at(t, 1550.0, 70.0, 0)))  # (faster than flown)
        # at a weight between those flown (as it spawns: a B-52H's tanks 40 % full): linear between them
        self.assertAlmostEqual(P.ps_at_weight(t, 100.0, 37.05, 900.0), P.ps_at(t, 100.0, 37.05, 0))
        mid = P.ps_at_weight(t, 100.0, 40.0, 1000.0)
        self.assertAlmostEqual(mid, 0.5 * (P.ps_at(t, 100.0, 40.0, 0) + P.ps_at(t, 100.0, 40.0, 1)))
        self.assertAlmostEqual(P.ceiling_at(t, 1000.0), 0.5 * (P.ceiling(t, 0) + P.ceiling(t, 1)))
        self.assertTrue(math.isnan(P.ceiling_at(t, 1200.0)))

    def test_short_runs_below_a_retry_and_past_a_drag_rise(self):
        # a retry began at 1.8 times a 100 m/s stall: short runs below it climb at 1.5 times (a best climb the run
        # missed) and not at all at 1.2; the least level speed falls to the slowest that flew with margin
        r = run(180.0, 300.0)
        c = P._curve(r, [(120.0, -1.0), (150.0, 6.0)], 115.0)
        self.assertEqual(c["lo"], 150.0)
        self.assertEqual(c["top"], 300.0)
        self.assertEqual(float(c["ps"][np.argmax(c["ps"])]), 6.0)

    def test_a_drag_rise_the_run_could_not_pass(self):
        # the run from below settled at 300 m/s; level runs begun past it: losing at 320 (the drag rise), climbing at
        # 380 and 420, losing again at 460 - full power holds level past the rise, up to where 420's +4 falls to 460's -12
        r = run(100.0, 300.0)
        c = P._curve(r, [(320.0, -2.0), (380.0, 3.0), (420.0, 4.0), (460.0, -12.0)], 90.0)
        self.assertAlmostEqual(c["top"], 420.0 + 4.0 / 16.0 * 40.0)
        self.assertEqual(c["reach"], 300.0)
        self.assertEqual(c["spots"], 4)
        # nothing past it holds more than it loses: the top is where the run got
        c = P._curve(r, [(320.0, -2.0), (380.0, -5.0)], 90.0)
        self.assertEqual(c["top"], 300.0)
        # only past it does full power hold level at all: the top from the runs there, none reached from below
        r["held"] = False
        c = P._curve(r, [(380.0, 3.0), (420.0, 4.0), (460.0, -12.0)], 90.0)
        self.assertAlmostEqual(c["top"], 430.0)
        self.assertTrue(math.isnan(c["reach"]))
        self.assertEqual(c["lo"], 380.0)

    def test_the_profile_carries_the_cells_flown(self):
        f = table_fields(tables())
        self.assertEqual(f["altitude_m/h1"], 3000.0)
        self.assertEqual(f["speed_fraction/v2"], 1.0)
        self.assertEqual(f["fuel_capacity_kg"], 200.0)
        self.assertEqual(f["max_tas_ms/h1/w0"], 55.0)
        self.assertNotIn("max_tas_ms/h1/w1", f)  # (not flown: left out, NaN when read)
        self.assertEqual(f["ps_full_ms/h0/w1/v1"], 4.0)
        self.assertNotIn("tas_ms/h0/w0/v0", f)  # (the speed each point held: the report's, not the platform's)
        # the names are the platform's: letters first, no index a JSBSim property would read as one
        self.assertTrue(all(all(part[0].isalpha() or part[0] == "_" for part in k.split("/")) for k in f))


if __name__ == "__main__":
    unittest.main()
