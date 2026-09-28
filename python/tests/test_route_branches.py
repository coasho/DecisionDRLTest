"""A route's conditional branches as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.37): branches read
back, one taken once its point has been come to twice, one the operator commands; a malformed one refused, naming its
point; the operator's input to one that takes none refused, naming it."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteBranchesTest(unittest.TestCase):
    def test_taken_as_they_say(self):
        w = make_world("py-route-branches")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        at = lambda n, e: (lat + n * 3000.0 / R, lon + e * 3000.0 / (R * math.cos(lat)))  # noqa: E731
        # A: two points east, on into B; B: four round and round; C: out, south of B
        ne = [(0, 1), (0, 2), (1, 3), (2, 3), (2, 4), (1, 4), (-1, 4), (-1, 5)]
        points = [fsim.Waypoint(*at(n, e)) for n, e in ne]
        points[1] = points[1]._replace(next=2)
        points[5] = points[5]._replace(next=2)
        paths = [fsim.RoutePath(1, "primary", 0, 2), fsim.RoutePath(2, "primary", 2, 4), fsim.RoutePath(3, "egress", 6, 2)]
        out = fsim.RouteBranch(5, 6, captures=2, captures_comparison="greater_equal")
        asked = fsim.RouteBranch(3, 6, operator_input=1)
        a = v.submit_route(points, paths=paths, branches=[out, asked])
        back = a.setpoint().kwargs["branches"]
        self.assertEqual([(b.point, b.next) for b in back], [(5, 6.0), (3, 6.0)])
        self.assertEqual(back[0].captures_comparison, float(fsim.Comparison.GREATER_EQUAL))
        self.assertEqual(back[1].operator_input, 1.0)
        flown = []
        for _ in range(100):
            w.step(int(round(10.0 / w.step_seconds)))
            if a.progress.segments and (not flown or flown[-1] != a.progress.segment):
                flown.append(a.progress.segment)
            if a.info.state not in (fsim.ActivityState.ACTIVE, fsim.ActivityState.PENDING):
                break
        self.assertEqual(flown, [0, 1, 2, 3, 4, 5, 2, 3, 4, 5, 6, 7])  # (round twice, then out)
        # the operator's input: out at B's second point the next time it is come to
        b = v.submit_route(points, paths=paths, branches=[out, asked])
        b.command_branch(1)
        with self.assertRaises(fsim.Rejected) as none:
            b.command_branch(0)  # (one that takes no operator input)
        self.assertEqual((none.exception.reason, none.exception.index), ("invalid_parameter", 0))
        flown = []
        for _ in range(90):  # (from where the first ended, back to A)
            w.step(int(round(10.0 / w.step_seconds)))
            if not flown or flown[-1] != b.progress.segment:
                flown.append(b.progress.segment)
        self.assertEqual(flown[:6], [0, 1, 2, 3, 6, 7])
        # refused, naming its point: a branch on to its own point
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points, paths=paths, branches=[fsim.RouteBranch(5, 5)])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 5))


if __name__ == "__main__":
    unittest.main()
