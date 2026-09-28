"""A route's turn points as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.30): an arc from a start
turn point to an end turn point by name, read back and flown; a capture's course; a radius the arc does not have refused,
naming the point it does not reach."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class TurnPointsTest(unittest.TestCase):
    def test_an_arc_a_capture_and_a_refusal(self):
        w = make_world("py-turn-points")
        v = w.create_vehicle("cessna", "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                             heading_deg=0.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad  # (the state is a live view: its values now)
        east = lambda m: m / (R * math.cos(lat))  # noqa: E731
        r = 1200.0
        # north 3 km, a quarter circle of 1,200 m round to the right - east - then on east 3 km
        points = [fsim.Waypoint(lat + 3000.0 / R, lon, turn="start_turn", turn_radius_m=r),
                  fsim.Waypoint(lat + (3000.0 + r) / R, lon + east(r), turn=fsim.TurnType.END_TURN, course_rad=0.5 * math.pi),
                  fsim.Waypoint(lat + (3000.0 + r) / R, lon + east(r + 3000.0))]
        a = v.submit_route(points)
        back = a.setpoint().args[0]
        self.assertEqual((back[0].turn, back[0].turn_radius_m), (float(fsim.TurnType.START_TURN), r))
        self.assertEqual((back[1].turn, back[1].course_rad), (float(fsim.TurnType.END_TURN), 0.5 * math.pi))
        worst, t0 = 0.0, w.time
        while w.time - t0 < 250.0 and a.state != fsim.ActivityState.COMPLETED:
            w.step(int(round(1.0 / w.step_seconds)))
            p = a.progress
            if p.segment == 1 and 15.0 < p.segment_percent < 85.0:  # (off the arc: its centre 1,200 m east of its start)
                s = v.state
                n, e = (s.latitude_rad - (lat + 3000.0 / R)) * R, (s.longitude_rad - (lon + east(r))) * R * math.cos(lat)
                worst = max(worst, abs(math.hypot(n, e) - r))
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertLess(worst, 15.0)
        self.assertGreater(worst, 0.0)  # (measured)
        # a radius the arc does not have: refused at the point it does not reach
        bad = list(points)
        bad[0] = bad[0]._replace(turn_radius_m=1500.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(bad)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 1))
        # a capture: over a point, then its course captured - the next point along it
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        c = v.submit_route([fsim.Waypoint(lat, lon + east(3000.0), turn="capture_outbound_course", course_rad=0.5 * math.pi),
                            fsim.Waypoint(lat, lon + east(7000.0))])
        self.assertEqual(c.setpoint().args[0][0].course_rad, 0.5 * math.pi)


if __name__ == "__main__":
    unittest.main()
