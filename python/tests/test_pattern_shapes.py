"""A-GRA's orbit and hold as its schema gives them, through Python (docs/flight-autonomy.md, 4.23 to 4.25): a racetrack
by two circles read back complete, a hold by its inbound heading, legs' time and bank, a hold's turn type, entry and
context by name, laps that complete it, an exit point it leaves from, a rotorcraft's hover (a wing's refused), a point
in a moving frame, and refusals naming their field."""
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
        # a hold's turns by type, its entry and context (4.24), by name
        t = v.submit_pattern(pattern="hold", latitude_rad=s.latitude_rad, longitude_rad=s.longitude_rad + 5000.0 * east, turn_type="relax",
                             hold_entry="anchor", hold_context="atc")
        held = t.setpoint().kwargs
        self.assertEqual((held["turn_type"], held["hold_entry"], held["hold_context"]),
                         (fsim.HoldTurn.RELAX, fsim.HoldEntry.ANCHOR, fsim.HoldContext.ATC))
        speed = held["speed"]
        self.assertAlmostEqual(held["radius_m"], max(speed / math.radians(1.5), speed * speed / (9.80665 * math.tan(math.radians(15.0)))),
                               delta=1e-6 * held["radius_m"])
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(hold_entry="parallel")  # an orbit has no fix to enter by
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 27))
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

    def test_a_hover_and_a_point_in_a_frame(self):
        w = make_world("py-pattern-frames")
        v = cessna(w)
        declared = w.create_vehicle("hangar-cessna", "jsbsim:c172", latitude_deg=37.7, longitude_deg=-122.375, altitude_msl_m=1500.0,
                                    airspeed_ms=55.0, heading_deg=90.0)
        quad = w.create_vehicle("quad", "jsbsim:iris", latitude_deg=37.8, longitude_deg=-122.375, altitude_msl_m=100.0, airspeed_ms=0.0)
        w.step(10)
        # a hover is a rotorcraft's: a wing's refused, as its support table says - not supported where its design says it does not
        # fly on rotors, not implemented where nothing says (a stock model's)
        for wing, why in ((declared, "not_supported"), (v, "not_implemented")):
            with self.assertRaises(fsim.Rejected) as refused:
                wing.submit_pattern(pattern="hover", duration_s=20.0)
            self.assertEqual((refused.exception.reason, refused.exception.index), (why, 0))
        q = quad.state
        h = quad.submit_pattern(pattern=fsim.PatternKind.HOVER, latitude_rad=q.latitude_rad + 20.0 / R, longitude_rad=q.longitude_rad,
                                altitude_m=q.altitude_msl_m + 5.0, duration_s=20.0)
        hovered = h.setpoint().kwargs
        self.assertEqual(hovered["pattern"], fsim.PatternKind.HOVER)
        self.assertTrue(math.isnan(hovered["radius_m"]) and math.isnan(hovered["course_rad"]))  # (nothing round it)
        with self.assertRaises(fsim.Rejected) as refused:
            quad.submit_pattern(pattern="hover", radius_m=10.0)  # a circuit's
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 5))
        # an orbit round a point 500 m ahead of a ship moving north-east, its rotation and offsets by name
        s = v.state
        ship = w.create_frame("moving", latitude_rad=s.latitude_rad, longitude_rad=s.longitude_rad + 3000.0 / (R * math.cos(s.latitude_rad)),
                              yaw_rad=math.pi / 4.0, north_ms=6.0, east_ms=6.0, time_s=w.time)
        a = v.submit_pattern(radius_m=1000.0, frame=ship, frame_rotation="yaw", frame_offsets="cartesian", frame_x_m=500.0)
        given = a.setpoint().kwargs
        self.assertEqual((given["frame"], given["frame_rotation"], given["frame_offsets"], given["frame_x_m"]),
                         (float(ship), fsim.FrameRotation.YAW, fsim.FrameOffsets.CARTESIAN, 500.0))
        self.assertAlmostEqual(given["latitude_rad"], s.latitude_rad + 500.0 * math.cos(math.pi / 4.0) / R, delta=2.0 / R)  # (the frame's point)
        w.step(int(round(20.0 / w.step_seconds)))
        self.assertEqual(a.state, fsim.ActivityState.ACTIVE)
        # a point replaces the frame
        a.update(latitude_rad=s.latitude_rad, longitude_rad=s.longitude_rad)
        self.assertTrue(math.isnan(a.setpoint().kwargs["frame"]))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(frame=ship + 100)  # not the world's
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 29))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(frame_y_m=10.0)  # offsets without their frame
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 33))


if __name__ == "__main__":
    unittest.main()
