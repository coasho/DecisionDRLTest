"""A route's civil path terminators as A-GRA's schema gives them, through Python (docs/flight-autonomy.md, 4.38): a radius to
fix's arc round its centre, by name, read back and flown within 20 m; a leg its segment does not define refused, naming its
point; a course to fix without its course refused; FA-6f2's legs not implemented."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RouteTerminatorsTest(unittest.TestCase):
    def test_flown_as_their_legs_say(self):
        w = make_world("py-route-terminators")
        v = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                             airspeed_ms=50.0, heading_deg=90.0)
        w.step(10)
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        at = lambda n, e: (lat + n / R, lon + e / (R * math.cos(lat)))  # noqa: E731
        # east 3 km, a quarter circle of 1,500 m round to the right (its centre 1,500 m south of its start), south 3 km
        r = 1500.0
        points = [fsim.Waypoint(*at(0, 3000)), fsim.Waypoint(*at(-r, 3000 + r), terminator="rf"), fsim.Waypoint(*at(-r - 3000, 3000 + r))]
        centre = at(-r, 3000)
        arc = fsim.RouteTerminator(1, center_latitude_rad=centre[0], center_longitude_rad=centre[1], radius_m=r, clockwise=True)
        a = v.submit_route(points, terminators=[arc])
        sp = a.setpoint()
        back = sp.kwargs
        self.assertEqual(sp.args[0][1].terminator, float(fsim.PathTerminator.RADIUS_TO_FIX))
        self.assertEqual(fsim.PathTerminator.RF, fsim.PathTerminator.RADIUS_TO_FIX)  # (ARINC 424's code, an alias)
        self.assertEqual([(t.point, t.radius_m, t.clockwise) for t in back["terminators"]], [(1, r, 1.0)])
        worst, samples = 0.0, 0
        for _ in range(300):
            w.step(int(round(1.0 / w.step_seconds)))
            p = a.progress
            if a.info.state == fsim.ActivityState.ACTIVE and p.segment == 1 and 5.0 < p.segment_percent < 95.0:
                s = v.state
                n, e = (s.latitude_rad - centre[0]) * R, (s.longitude_rad - centre[1]) * R * math.cos(lat)
                worst, samples = max(worst, abs(math.hypot(n, e) - r)), samples + 1
            if a.info.state not in (fsim.ActivityState.ACTIVE, fsim.ActivityState.PENDING):
                break
        self.assertGreater(samples, 20)
        self.assertLess(worst, 20.0)  # (FA-6's criterion)
        self.assertEqual(a.info.state, fsim.ActivityState.COMPLETED)
        # refused, naming its point: a leg its segment does not define (a procedure turn); a course to fix without its course
        turn = list(points)
        turn[2] = turn[2]._replace(terminator="procedure_turn_to_intercept")
        with self.assertRaises(fsim.Rejected) as undefined:
            v.submit_route(turn, terminators=[arc])
        self.assertEqual((undefined.exception.reason, undefined.exception.index), ("invalid_waypoint", 2))
        course = list(points)
        course[2] = course[2]._replace(terminator=fsim.PathTerminator.CF)
        with self.assertRaises(fsim.Rejected) as none:
            v.submit_route(course, terminators=[arc])
        self.assertEqual((none.exception.reason, none.exception.index), ("invalid_waypoint", 2))
        # its course given, flown; a manual termination mid-route that nothing ends refused, naming the point - one the
        # operator's branch ends taken
        c = v.submit_route(course, terminators=[arc, fsim.RouteTerminator(2, course_rad=math.pi)])
        self.assertEqual(c.setpoint().kwargs["terminators"][1].course_rad, math.pi)
        manual = list(points)
        manual[1] = manual[1]._replace(terminator="fm")
        with self.assertRaises(fsim.Rejected) as none:
            v.submit_route(manual)
        self.assertEqual((none.exception.reason, none.exception.index), ("invalid_waypoint", 1))
        v.submit_route(manual, branches=[fsim.RouteBranch(1, 2, operator_input=1)])


if __name__ == "__main__":
    unittest.main()
