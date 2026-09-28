"""A route's paths and links as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.36): paths with ids and
types by name, a point's next into another path and round a loop - read back, its end points and its flight in that
order; paths that do not tile the points refused, naming the point."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RoutePathsTest(unittest.TestCase):
    def test_along_its_links(self):
        w = make_world("py-route-paths")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        at = lambda n, e: (lat + n * 3000.0 / R, lon + e * 3000.0 / (R * math.cos(lat)))  # noqa: E731
        # A: two points east, on into B; B: four round and round; C: linked from nowhere
        ne = [(0, 1), (0, 2), (1, 3), (2, 3), (2, 4), (1, 4), (-1, 1), (-1, 2)]
        points = [fsim.Waypoint(*at(n, e), id=100 + i) for i, (n, e) in enumerate(ne)]
        points[1] = points[1]._replace(next=2)
        points[5] = points[5]._replace(next=2)
        paths = [fsim.RoutePath(10, "primary", 0, 2), fsim.RoutePath(20, "ingress", 2, 4), fsim.RoutePath(30, fsim.PathType.ALTERNATE, 6, 2)]
        a = v.submit_route(points, paths=paths)
        back = a.setpoint()
        self.assertEqual([p.id for p in back.kwargs["paths"]], [10, 20, 30])
        self.assertEqual(back.kwargs["paths"][1].type, float(fsim.PathType.INGRESS))
        self.assertEqual(back.args[0][5].next, 2.0)
        self.assertEqual([e.index for e in a.end_points(8)], [0, 1, 2, 3, 4, 5, 2, 3])
        flown = []
        for _ in range(40):
            w.step(int(round(10.0 / w.step_seconds)))
            if not flown or flown[-1] != a.progress.segment:
                flown.append(a.progress.segment)
        self.assertEqual(flown[:7], [0, 1, 2, 3, 4, 5, 2])  # (round B, back where it loops to)
        # refused, naming the point: a point no path holds
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points, paths=[paths[0], fsim.RoutePath(20, "ingress", 3, 3), paths[2]])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 2))


if __name__ == "__main__":
    unittest.main()
