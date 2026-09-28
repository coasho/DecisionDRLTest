"""A route's segment performance as A-GRA's schema gives it, through Python (docs/flight-autonomy.md, 4.32): a segment at
the performance tables' best range speed now, its speed replaced, by name - read back and flown - reached at its
acceleration; refused as a point is, naming it: a speed or a climb optimisation without performance tables, an
acceleration of 0; an acceleration beyond the aircraft's held to it, named by its point and field 24; a climb at the most
the aircraft climbs holding its speed, by name."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteSegmentsTest(unittest.TestCase):
    def test_best_range_speed_reached_at_its_acceleration(self):
        w = make_world("py-route-segments")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        stock = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=37.7, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                                 heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad  # (the state is a live view: its values now)
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        # refused, naming the point: the stock C172x's best range speed (no performance tables), a climb optimisation,
        # an acceleration of 0
        slat, slon = stock.state.latitude_rad, stock.state.longitude_rad
        first = fsim.Waypoint(slat, slon + east(3000.0))
        for fields, why in (({"speed_optimization": "long_range_cruise"}, "not_implemented"), ({"climb_optimization": "best_rate"}, "not_implemented"),
                            ({"acceleration_ms2": 0.0}, "invalid_waypoint")):
            with self.assertRaises(fsim.Rejected) as refused:
                stock.submit_route([first, fsim.Waypoint(slat, slon + east(8000.0), **fields)])
            self.assertEqual((refused.exception.reason, refused.exception.index), (why, 1), fields)
        # east 3 km at 50 m/s; then to 13 km at the best range speed, reached at 0.1 m/s^2
        points = [fsim.Waypoint(lat, lon + east(3000.0), speed=50.0),
                  fsim.Waypoint(lat, lon + east(13000.0), speed_optimization="long_range_cruise", acceleration_ms2=0.1)]
        a = v.submit_route(points)
        back = a.setpoint().args[0]
        self.assertEqual(back[1].speed_optimization, float(fsim.SpeedOptimization.LONG_RANGE_CRUISE))
        self.assertEqual((back[1].speed_reference, back[1].acceleration_ms2), (float(fsim.SpeedReference.TRUE_AIRSPEED), 0.1))
        best = back[1].speed  # (its snapshot: the tables' best at its altitude and the fuel as it was sent)
        self.assertTrue(40.0 < best < 50.0)
        # flown: slowing through its middle at its rate, its progress telling the speed it flies now; then at the best
        t0, speeds, told, settled = w.time, [], [], None
        while w.time - t0 < 400.0 and a.state != fsim.ActivityState.COMPLETED:
            w.step(int(round(1.0 / w.step_seconds)))
            p = a.progress
            if p.segment == 1:
                tas = v.state.airspeed_true_ms
                if best + 0.2 * (50.0 - best) < tas < best + 0.8 * (50.0 - best):
                    speeds.append((w.time, tas))
                    told.append(p.speed_ms)
                if p.segment_percent > 60.0:
                    settled = v.state.airspeed_true_ms
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertGreater(len(speeds), 10)
        rate = (speeds[0][1] - speeds[-1][1]) / (speeds[-1][0] - speeds[0][0])
        self.assertAlmostEqual(rate, 0.1, delta=0.02)
        self.assertTrue(all(best < s < 50.0 for s in told))  # (the ramp's, not the segment's end)
        self.assertAlmostEqual(settled, best, delta=0.5)
        # slowing from 50 to 35 m/s at 5 m/s^2: more than its idle slows it between, held to that
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        c = v.submit_route([fsim.Waypoint(lat, lon + east(3000.0), speed=50.0), fsim.Waypoint(lat, lon + east(9000.0), speed=35.0, acceleration_ms2=5.0)])
        self.assertTrue(c.clamped)
        adjustment = w.last_command_details()[1][0]
        self.assertEqual((adjustment.index, adjustment.field, adjustment.requested), (1, 24, 5.0))
        self.assertTrue(0.1 < adjustment.adjusted < 5.0)
        self.assertEqual(c.setpoint().args[0][1].acceleration_ms2, adjustment.adjusted)
        # up 300 m at its best rate, by name: read back, and climbing at what its tables give holding its speed
        c.cancel()
        lat, lon, alt = v.state.latitude_rad, v.state.longitude_rad, v.state.altitude_msl_m
        d = v.submit_route([fsim.Waypoint(lat, lon + east(3000.0), altitude_m=alt, speed=50.0),
                            fsim.Waypoint(lat, lon + east(20000.0), altitude_m=alt + 300.0, climb_optimization="best_rate")])
        self.assertEqual(d.setpoint().args[0][1].climb_optimization, float(fsim.ClimbOptimization.BEST_RATE))
        rates, t0 = [], w.time
        while w.time - t0 < 400.0 and d.state != fsim.ActivityState.COMPLETED:
            w.step(int(round(1.0 / w.step_seconds)))
            if d.progress.segment == 1 and alt + 50.0 < v.state.altitude_msl_m < alt + 250.0:
                rates.append(-v.state.velocity_ned_ms[2])
        self.assertGreater(len(rates), 10)
        self.assertTrue(all(1.0 < r < 2.5 for r in rates), rates)  # (its tables' excess power at 50 m/s: 1.5 to 1.9 m/s here)


if __name__ == "__main__":
    unittest.main()
