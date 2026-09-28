"""A curve's reference as A-GRA's schema gives it, through Python (docs/flight-autonomy.md, 4.27): within its altitude
range, its points laid out along great circles and read as offsets up - by name, read back, flown - in a moving frame
turned with it, turned in three dimensions by a frame's attitude, in a batch; where it is, given with its options alone,
refused naming the field, and what does not make a reference likewise; segments appended read as its."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def cessna(w, name="cessna"):
    return w.create_vehicle(name, "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1000.0, airspeed_ms=55.0,
                            heading_deg=90.0)


def straight(n0, e0, course, length, z0=0.0, z1=0.0):
    """A straight clamped cubic `length` metres along `course` (rad) from (n0, e0), its third from z0 to z1."""
    u = [i / 3.0 for i in range(4)]
    return fsim.NurbsSegment(north=[n0 + x * length * math.cos(course) for x in u], east=[e0 + x * length * math.sin(course) for x in u],
                             down=[z0 + x * (z1 - z0) for x in u], knots=[0.0] * 4 + [1.0] * 4)


class CurveReferenceTest(unittest.TestCase):
    def test_read_back_flown_in_a_frame_and_refused(self):
        w = make_world("py-curve-references")
        v = cessna(w)
        w.step(10)
        alt0, lat0 = v.state.altitude_msl_m, v.state.latitude_rad  # (the state is a live view: its values now)
        # within its range, laid out along great circles, its third read as offsets up: 60 m up by its end
        a = v.submit_curve([straight(0.0, 0.0, math.pi / 2.0, 6000.0, 0.0, 60.0)], altitude_min_m=alt0 + 50.0,
                           altitude_max_m=alt0 + 300.0, point_offsets="great_circle", point_z=fsim.CurveZ.ALTITUDE_OFFSET)
        given = a.setpoint().kwargs
        self.assertEqual((given["point_offsets"], given["point_z"]), (fsim.FrameOffsets.GREAT_CIRCLE, fsim.CurveZ.ALTITUDE_OFFSET))
        self.assertEqual(given["altitude_m"], alt0 + 50.0)  # (left out: the aircraft's, held up to its least)
        self.assertTrue(math.isnan(given["frame"]))
        # where it is changes only with a new curve's segments: given with its options alone, refused; segments appended go on
        # from its reference, read as its (A-GRA's append uses the preceding CenterReference) - a reading given with them unused
        with self.assertRaises(fsim.Rejected) as refused:
            a.update_curve(latitude_rad=lat0)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 0))
        a.append([straight(0.0, 6000.0, math.pi / 2.0, 3000.0, 60.0, 60.0)], point_z="down")
        self.assertEqual(a.setpoint().kwargs["point_z"], fsim.CurveZ.ALTITUDE_OFFSET)
        a.update_curve(speed_max_ms=60.0)  # (how it is flown: taken)
        w.step(int(round(190.0 / w.step_seconds)))
        self.assertEqual(a.state, fsim.ActivityState.COMPLETED)
        self.assertGreater(v.state.altitude_msl_m, alt0 + 105.0)  # (its reference 50 m up, its end 60 m above that: read as its)
        # in a moving frame, its points turned with it: 500 m ahead of a ship heading north-east, read back placed there
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        ship = w.create_frame("moving", latitude_rad=lat, longitude_rad=lon + 3000.0 / (R * math.cos(lat)),
                              yaw_rad=math.pi / 4.0, north_ms=6.0, east_ms=6.0, time_s=w.time)
        b = v.submit_curve([straight(0.0, 0.0, 0.0, 8000.0)], frame=ship, frame_rotation="yaw", frame_x_m=500.0, point_rotation="yaw")
        given = b.setpoint().kwargs
        self.assertEqual((given["frame"], given["frame_rotation"], given["frame_x_m"], given["point_rotation"]),
                         (float(ship), fsim.FrameRotation.YAW, 500.0, fsim.FrameRotation.YAW))
        self.assertAlmostEqual(given["latitude_rad"], lat + 500.0 * math.cos(math.pi / 4.0) / R, delta=2.0 / R)  # (the frame's point)
        end = b.end_points()[0]  # (8 km along the ship's heading from there)
        self.assertAlmostEqual((end.latitude_rad - given["latitude_rad"]) * R, 8000.0 * math.cos(math.pi / 4.0), delta=5.0)
        # its points turned in three dimensions, by a fixed frame's attitude: 6 km along a frame pitched 2 degrees up ends 209 m up
        pitched = w.create_frame("fixed", latitude_rad=lat, longitude_rad=lon, altitude_msl_m=1000.0, pitch_rad=math.radians(2.0))
        c = v.submit_curve([straight(0.0, 0.0, 0.0, 6000.0)], frame=pitched, point_rotation="attitude")
        given = c.setpoint().kwargs
        self.assertEqual(given["point_rotation"], fsim.FrameRotation.ATTITUDE)
        self.assertAlmostEqual(c.end_points()[0].altitude_m - given["altitude_m"], 6000.0 * math.sin(math.radians(2.0)), delta=0.5)
        # in a batch, its shape beside it
        results = v.submit_batch([fsim.BatchCommand("submit_curve", [straight(0.0, 0.0, 0.0, 8000.0)], frame=ship, frame_x_m=500.0)])
        self.assertIsInstance(results[0], fsim.Activity)
        self.assertEqual(results[0].setpoint().kwargs["frame"], float(ship))
        # what does not make a reference: refused, naming the field
        for fields, why, index in (({"point_rotation": "yaw"}, "invalid_parameter", 11),  # (turned with no frame to turn with)
                                   ({"frame": ship + 100}, "invalid_parameter", 14),  # (not the world's)
                                   ({"frame_y_m": 10.0}, "invalid_parameter", 18),  # (offsets without their frame)
                                   ({"altitude_m": 500.0, "altitude_min_m": 800.0}, "invalid_parameter", 2)):  # (outside its range)
            with self.assertRaises(fsim.Rejected) as refused:
                v.submit_curve([straight(0.0, 0.0, 0.0, 8000.0)], **fields)
            self.assertEqual((refused.exception.reason, refused.exception.index), (why, index), fields)
        with self.assertRaises(fsim.Rejected) as refused:  # (a reference given alone has no value to read)
            results[0].update_curve([straight(0.0, 0.0, 0.0, 8000.0)], altitude_reference="above_ground")  # (b: preempted by it)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 8))


if __name__ == "__main__":
    unittest.main()
