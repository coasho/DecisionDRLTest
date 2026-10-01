"""A launch through Python (docs/flight-autonomy.md, 4.49): a C172 parked on a runway takes off along it and climbs out, calm and
in a crosswind; a Crazyflie lifts to its hover; refused naming the field - an airfield or runway not the vehicle's - and in the
air."""
import math
import unittest

import fsim

R = 6371008.8  # the platform's mean radius


def world(name, wind=0.0):
    w = fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)
    if wind:
        w.set_wind(270.0, wind)
    return w


def parked(w, kind):
    v = w.create_vehicle(kind, "jsbsim:" + kind, latitude_deg=40.0, longitude_deg=0.0, heading_deg=0.0, on_ground=True, airspeed_ms=0.0)
    w.step(int(round(1.0 / w.step_seconds)))
    s = v.state
    ground = s.altitude_msl_m - s.altitude_agl_m
    start = fsim.RunwayPoint(s.latitude_rad, s.longitude_rad, ground)
    limit = fsim.RunwayPoint(s.latitude_rad + 3500.0 / R, s.longitude_rad, ground)
    v.load_airfield(fsim.Airfield(7, "", fsim.HOLD, (fsim.Runway(3, 0.0, 3500.0, fsim.RunwayCoordinates(start, start, limit)),)))
    return v, s.latitude_rad, s.longitude_rad


def fly(w, activity, seconds):
    for _ in range(int(seconds)):
        w.step(int(round(1.0 / w.step_seconds)))
        if activity.info.state not in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE):
            break


class LaunchTest(unittest.TestCase):
    def test_a_wing_takes_off_along_its_runway(self):
        for wind in (0.0, 10.0):
            with self.subTest(wind=wind):
                w = world("py-launch", wind)
                v, lat0, lon0 = parked(w, "c172")
                a = v.submit_behavior("launch", airfield=7, runway=3)
                self.assertEqual(v.support("fsim.guidance.launch").support, fsim.Support.SUPPORTED)
                worst = 0.0
                for _ in range(240):
                    w.step(int(round(1.0 / w.step_seconds)))
                    s = v.state
                    if s.on_ground:
                        worst = max(worst, abs((s.longitude_rad - lon0) * R * math.cos(lat0)))
                    if a.info.state not in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE):
                        break
                self.assertEqual(a.info.state, fsim.ActivityState.COMPLETED)
                self.assertLess(worst, 22.5)  # (half a runway: 0.2 m calm, 6.1 m in 10 m/s across it)
                self.assertGreater(v.state.altitude_agl_m, 400.0)

    def test_a_rotorcraft_lifts_to_its_hover(self):
        w = world("py-lift")
        v, lat0, lon0 = parked(w, "cf2")
        s0 = v.state
        a = v.submit_behavior("launch", airfield=7, runway=3, hover_agl_m=2.0)
        fly(w, a, 60)
        self.assertEqual(a.info.state, fsim.ActivityState.COMPLETED)
        s = v.state
        north, east = (s.latitude_rad - lat0) * R, (s.longitude_rad - lon0) * R * math.cos(lat0)
        up = (s.altitude_msl_m - s0.altitude_msl_m) - (2.0 - s0.altitude_agl_m)
        self.assertLess(math.hypot(math.hypot(north, east), up), 1.0)

    def test_a_rejected_takeoff(self):
        # a policy's CANCEL at half its rotation speed: FA's own activity stops it on the runway (4.50)
        w = world("py-rto")
        v, lat0, lon0 = parked(w, "f16c")
        a = v.submit_behavior("launch", airfield=7, runway=3)
        vr = 1.1 * v.profile_value("envelope/clean/cas_min_ms")
        while v.state.airspeed_calibrated_ms < 0.5 * vr:
            w.step(3)
        own = a.cancel()
        self.assertIsNotNone(own)
        self.assertEqual(own.source, fsim.Source.AUTOPILOT)
        fly(w, own, 120)
        self.assertEqual((a.info.state, own.info.state), (fsim.ActivityState.CANCELED, fsim.ActivityState.COMPLETED))
        s = v.state
        self.assertLess(math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1]), 0.5)
        self.assertLess((s.latitude_rad - lat0) * R, 3500.0)
        # its fuel gone below its decision speed: the launch fails "takeoff_rejected", stopped on the runway
        w = world("py-rto-engine")
        v, lat0, lon0 = parked(w, "c172")
        a = v.submit_behavior("launch", airfield=7, runway=3)
        while v.state.airspeed_calibrated_ms < 15.0:
            w.step(3)
        for _ in range(240):
            v.set_property("propulsion/tank[0]/contents-lbs", 0.0)
            v.set_property("propulsion/tank[1]/contents-lbs", 0.0)
            w.step(int(round(0.5 / w.step_seconds)))
            if a.info.state not in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE):
                break
        self.assertEqual((a.info.state, a.info.reason), (fsim.ActivityState.FAILED, "takeoff_rejected"))
        self.assertLess(math.hypot(v.state.velocity_ned_ms[0], v.state.velocity_ned_ms[1]), 0.5)

    def test_a_taxi(self):
        # north 150 m, east 200: within 2 m of its path, stopped at its end (4.51); a corner it cannot turn refused
        w = world("py-taxi")
        v, lat0, lon0 = parked(w, "c172")
        at = lambda n, e: (lat0 + n / R, lon0 + e / (R * math.cos(lat0)), 0.0, math.nan, 5.0)  # noqa: E731
        a = v.submit_behavior("taxi", points=[at(150.0, 0.0), at(150.0, 200.0)], speed_ms=8.0)
        fly(w, a, 120)
        self.assertEqual(a.info.state, fsim.ActivityState.COMPLETED)
        s = v.state
        self.assertLess(math.hypot((s.latitude_rad - lat0) * R - 150.0, (s.longitude_rad - lon0) * R * math.cos(lat0) - 200.0), 1.0)
        self.assertEqual(v.support("fsim.guidance.taxi").support, fsim.Support.SUPPORTED)
        with self.assertRaises(fsim.Rejected) as refused:  # back on itself at its first point
            v.submit_behavior("taxi", points=[at(300.0, 200.0), at(200.0, 200.0)])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 0))

    def test_refusals(self):
        w = world("py-launch-refused")
        v, _, _ = parked(w, "c172")
        for params, field in ((dict(airfield=8, runway=3), 0), (dict(airfield=7, runway=4), 1)):
            with self.assertRaises(fsim.Rejected) as refused:
                v.submit_behavior("launch", **params)
            self.assertEqual((refused.exception.reason, refused.exception.index), ("unknown_airfield", field))
        flying = w.create_vehicle("air", "jsbsim:c172", latitude_deg=41.0, longitude_deg=0.0, altitude_msl_m=1000.0, airspeed_ms=50.0,
                                  heading_deg=0.0)
        w.step(10)
        with self.assertRaises(fsim.Rejected) as refused:
            flying.submit_behavior("launch", airfield=7, runway=3)
        self.assertEqual(refused.exception.reason, "airborne")


if __name__ == "__main__":
    unittest.main()
