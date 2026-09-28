"""A-GRA's orbit as its schema gives it, through Python (docs/flight-autonomy.md, 4.23): a racetrack by two circles
read back complete, a hold by its inbound heading, legs' time and bank, laps that complete it, an exit point it leaves
from, and a refusal naming its field."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def cessna(w, name="cessna"):
    return w.create_vehicle(name, "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0, airspeed_ms=55.0,
                            heading_deg=90.0)


class PatternShapesTest(unittest.TestCase):
    def test_two_circles_and_a_fix_point_orbit(self):
        w = make_world("py-pattern-shapes")
        v = cessna(w)
        w.step(10)
        s = v.state
        east = 1.0 / (R * math.cos(s.latitude_rad))
        a = v.submit_pattern(pattern="racetrack", latitude_rad=s.latitude_rad, longitude_rad=s.longitude_rad + 3000.0 * east, radius_m=900.0,
                             latitude2_rad=s.latitude_rad + 4000.0 / R, longitude2_rad=s.longitude_rad + 3000.0 * east, radius2_m=1200.0)
        given = a.setpoint().kwargs
        self.assertEqual((given["radius_m"], given["radius2_m"]), (900.0, 1200.0))
        self.assertTrue(math.isnan(given["course_rad"]) and math.isnan(given["leg_m"]))  # (the circles give the legs)
        # a hold by its inbound heading, its legs' time and its turns' bank, magnetic
        h = v.submit_pattern(pattern="hold", latitude_rad=s.latitude_rad, longitude_rad=s.longitude_rad + 5000.0 * east, heading_rad=0.0,
                             direction_reference="magnetic_north", leg_s=45.0, bank_rad=math.radians(20.0))
        held = h.setpoint().kwargs
        self.assertEqual((held["heading_rad"], held["leg_s"], held["direction_reference"]), (0.0, 45.0, fsim.DirectionReference.MAGNETIC_NORTH))
        self.assertTrue(held["course_rad"] == held["course_rad"] and held["leg_m"] > 0.0)  # filled in from them
        gusted = held["speed"] + 0.0  # (calm)
        self.assertAlmostEqual(held["radius_m"], gusted * gusted / (9.80665 * math.tan(math.radians(20.0))), delta=0.01 * held["radius_m"])
        # an update of the bank replaces the radius
        h.update(bank_rad=math.radians(25.0))
        self.assertAlmostEqual(h.setpoint().kwargs["radius_m"], gusted * gusted / (9.80665 * math.tan(math.radians(25.0))),
                               delta=0.01 * held["radius_m"])
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(latitude2_rad=s.latitude_rad, longitude2_rad=s.longitude_rad)  # an orbit is one circle
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 18))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(orbits=1.5)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 17))

    def test_laps_and_an_exit_point(self):
        w = make_world("py-pattern-ends")
        v = cessna(w)
        w.step(10)
        s = v.state
        centre = (s.latitude_rad, s.longitude_rad + 2000.0 / (R * math.cos(s.latitude_rad)))
        exit_point = (centre[0], centre[1] + 800.0 / (R * math.cos(s.latitude_rad)))  # its east
        a = v.submit_pattern(latitude_rad=centre[0], longitude_rad=centre[1], radius_m=800.0, orbits=1,
                             exit_latitude_rad=exit_point[0], exit_longitude_rad=exit_point[1])
        steps = 0
        while a.state in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE) and steps < int(400.0 / w.step_seconds):
            w.step()
            steps += 1
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertEqual(a.progress.laps, 1)
        s = v.state
        off = math.hypot((s.latitude_rad - exit_point[0]) * R, (s.longitude_rad - exit_point[1]) * R * math.cos(s.latitude_rad))
        self.assertLess(off, 150.0)  # completed at its exit point
        w.step(int(round(30.0 / w.step_seconds)))
        s = v.state
        self.assertLess(abs(math.degrees(math.remainder(math.atan2(s.velocity_ned_ms[1], s.velocity_ned_ms[0]) - math.pi, 2.0 * math.pi))), 5.0)


if __name__ == "__main__":
    unittest.main()
