"""A launch through Python (docs/flight-autonomy.md, 4.49): a C172 parked on a runway takes off along it and climbs out, calm and
in a crosswind; a Crazyflie lifts to its hover; refused naming the field - an airfield or runway not the vehicle's - and in the
air. A rejected takeoff (4.50), a taxi (4.51), and a route that starts on the ground (4.52)."""
import math
import unittest

import fsim
import fsim.agra

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
        for wind in (0.0, 7.0):  # (the C172's crosswind limit: 15 kt, 7.72 m/s - 4.54)
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
                self.assertLess(worst, 22.5)  # (half a runway)
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

    def test_a_route_from_the_ground(self):
        # a route that starts on the ground (4.52): its taxi points to the runway's start, its runway points' takeoff, then its
        # points in the air - as points, and as FA's own plan's taxi and takeoff paths; on a runway too short, rejected and failed
        def layout(v, lat0, lon0, length):
            s = v.state
            ground = s.altitude_msl_m - s.altitude_agl_m
            at = lambda n, e, t=fsim.HOLD: fsim.Waypoint(lat0 + n / R, lon0 + e / (R * math.cos(lat0)), waypoint_type=t)  # noqa: E731
            up = lambda n, e: fsim.Waypoint(lat0 + n / R, lon0 + e / (R * math.cos(lat0)), ground + 600.0, "msl")  # noqa: E731
            start, limit = at(160.0, 150.0), at(160.0 + length, 150.0)
            v.load_airfield(fsim.Airfield(7, "", fsim.HOLD, (fsim.Runway(3, 0.0, length, fsim.RunwayCoordinates(
                fsim.RunwayPoint(start.latitude_rad, start.longitude_rad, ground), fsim.RunwayPoint(start.latitude_rad, start.longitude_rad, ground),
                fsim.RunwayPoint(limit.latitude_rad, limit.longitude_rad, ground))),)))
            T = fsim.WaypointType
            taxi = [at(60.0, 0.0, T.TAXI), at(60.0, 150.0, T.TAXI)]
            return taxi, [at(160.0, 150.0, T.RUNWAY_START), at(160.0 + length, 150.0, T.RUNWAY_LIMIT)], [up(5000.0, 150.0), up(5000.0, 9000.0)], ground

        def flown(w, v, a, lat0, lon0, ground):
            for _ in range(600):
                w.step(int(round(1.0 / w.step_seconds)))
                s = v.state
                if math.hypot((s.latitude_rad - lat0) * R - 5000.0, (s.longitude_rad - lon0) * R * math.cos(lat0) - 9000.0) < 300.0:
                    break
                if a.info.state not in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE):
                    break
            return v.state

        w = world("py-ground-route")
        v, lat0, lon0 = parked(w, "f16c")
        taxi, runway, air, ground = layout(v, lat0, lon0, 3000.0)
        a = v.submit_route(taxi + runway + air)
        s = flown(w, v, a, lat0, lon0, ground)
        self.assertEqual(a.info.state, fsim.ActivityState.ACTIVE)
        self.assertLess(abs(s.altitude_msl_m - ground - 600.0), 50.0)
        self.assertEqual(v.support("fsim.guidance.route/waypoint_type/runway").support, fsim.Support.SUPPORTED)
        self.assertEqual(v.support("fsim.guidance.route/waypoint_type/taxi").support, fsim.Support.PARTIAL)
        # FA's own plan: a taxi path, a takeoff path naming the runway, a primary path
        w = world("py-ground-plan")
        v, lat0, lon0 = parked(w, "f16c")
        taxi, _, air, ground = layout(v, lat0, lon0, 3000.0)
        points = [taxi[0], taxi[1]._replace(next=2), air[0]._replace(next=3), air[1]]
        paths = [fsim.RoutePath(1, "taxi", 0, 2), fsim.RoutePath(2, "takeoff", 2, 1), fsim.RoutePath(3, "primary", 3, 1)]
        v.load_plan(fsim.RoutePlan(41, fsim.BatchCommand("submit_route", points, paths=paths), path_metadata=[fsim.PathMetadata(1, airfield=7, runway=3)]))
        self.assertTrue(v.plan_command(41, "prepare_for_activation").completed)
        r = v.plan_command(41, "activate")
        self.assertTrue(r.completed)
        s = flown(w, v, r.activity, lat0, lon0, ground)
        self.assertEqual(r.activity.info.state, fsim.ActivityState.ACTIVE)
        self.assertLess(abs(s.altitude_msl_m - ground - 600.0), 50.0)
        # a runway too short: its takeoff rejected, the route failed, stopped on the runway
        w = world("py-ground-reject")
        v, lat0, lon0 = parked(w, "c172")
        taxi, runway, air, ground = layout(v, lat0, lon0, 250.0)
        a = v.submit_route(taxi + runway + air)
        flown(w, v, a, lat0, lon0, ground)
        self.assertEqual((a.info.state, a.info.reason), (fsim.ActivityState.FAILED, "takeoff_rejected"))
        self.assertTrue(v.state.on_ground)
        # in the air: a route that starts on the ground cannot
        flying = w.create_vehicle("air", "jsbsim:c172", latitude_deg=41.0, longitude_deg=0.0, altitude_msl_m=1000.0, airspeed_ms=50.0,
                                  heading_deg=0.0)
        w.step(10)
        with self.assertRaises(fsim.Rejected) as refused:
            flying.submit_route(taxi + runway + air)
        self.assertEqual(refused.exception.reason, "airborne")

    def test_a_speedbrake(self):
        # the F-15C's dorsal speed brake (4.55): supported, opened by the speedbrake, its drag felt; none on the C172
        w = world("py-speedbrake")
        v = w.create_vehicle("f15c", "jsbsim:f15c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, heading_deg=0.0, airspeed_ms=200.0)
        w.step(10)
        self.assertEqual(v.support("fsim.support.speedbrake").support, fsim.Support.SUPPORTED)
        self.assertEqual(v.profile_value("effectors/speedbrake_approach"), 1.0)
        v.submit_support("speedbrake", 1.0)
        w.step(int(round(4.0 / w.step_seconds)))
        self.assertAlmostEqual(v.get_property("fcs/speedbrake-pos-norm"), 1.0)
        self.assertGreater(v.get_property("aero/coefficient/CD_speed_brake"), 0.0)
        c = w.create_vehicle("c172", "jsbsim:c172", latitude_deg=40.1, longitude_deg=0.0, altitude_msl_m=1000.0, heading_deg=0.0, airspeed_ms=50.0)
        self.assertEqual(c.support("fsim.support.speedbrake").support, fsim.Support.NOT_SUPPORTED)

    def test_a_crosswind_limit(self):
        # beyond the C172's 15 kt (4.54): its launch refused crosswind_limit, A-GRA's CAPABILITY_PERFORMANCE; within it, flown
        w = world("py-crosswind", wind=10.0)
        v, _, _ = parked(w, "c172")
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_behavior("launch", airfield=7, runway=3)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("crosswind_limit", -1))
        self.assertEqual(fsim.agra.CANNOT_COMPLY["crosswind_limit"], "CAPABILITY_PERFORMANCE")
        self.assertEqual(fsim.agra.CANNOT_COMPLY["landing_abandoned"], "CONSTRAINT_ATTEMPTS")
        self.assertAlmostEqual(v.profile_value("envelope/crosswind_max_ms"), 7.72)
        self.assertEqual(v.support("fsim.guidance.recovery/go_around").support, fsim.Support.SUPPORTED)
        w = world("py-crosswind-within", wind=5.0)
        v, _, _ = parked(w, "c172")
        self.assertEqual(v.submit_behavior("launch", airfield=7, runway=3).info.state, fsim.ActivityState.PENDING)

    def test_a_recovery(self):
        # RECOVERY to a runway 8 km ahead (4.53): down the glide slope, touched down in the zone below its sink-rate limit, stopped
        w = world("py-recovery")
        v = w.create_vehicle("c172", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=300.0, heading_deg=0.0, airspeed_ms=45.0)
        w.step(10)
        s = v.state
        lat0, lon0, ground = s.latitude_rad + 8000.0 / R, s.longitude_rad, s.altitude_msl_m - s.altitude_agl_m
        start = fsim.RunwayPoint(lat0, lon0, ground)
        limit = fsim.RunwayPoint(lat0 + 3000.0 / R, lon0, ground)
        v.load_airfield(fsim.Airfield(7, "", fsim.HOLD, (fsim.Runway(3, 0.0, 3000.0, fsim.RunwayCoordinates(), fsim.RunwayCoordinates(start, start, limit)),)))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_behavior("recovery", airfield=7, runway=4)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("unknown_airfield", 1))
        a = v.submit_behavior("recovery", airfield=7, runway=3)
        self.assertEqual(v.support("fsim.guidance.recovery").support, fsim.Support.SUPPORTED)
        touched, sink = None, 0.0
        for _ in range(3000):
            w.step(int(round(0.5 / w.step_seconds)))
            s = v.state
            if touched is None and not s.on_ground:
                sink = s.velocity_ned_ms[2]
            if touched is None and s.on_ground:
                touched = (s.latitude_rad - lat0) * R
            if a.info.state not in (fsim.ActivityState.PENDING, fsim.ActivityState.ACTIVE):
                break
        self.assertEqual(a.info.state, fsim.ActivityState.COMPLETED)
        self.assertIsNotNone(touched)
        self.assertTrue(0.0 < touched < 900.0)  # (its touchdown zone: 279 m along)
        self.assertLess(sink, 3.2)               # (1.05 m/s)
        self.assertLess(math.hypot(v.state.velocity_ned_ms[0], v.state.velocity_ned_ms[1]), 0.5)

    def test_cleanup_and_dirtyup(self):
        # A-GRA's CleanUp and DirtyUp (4.58): an update of the recovery's configuration - the C172 at 44 m/s, above its flaps' 85 kt
        # (43.7 m/s): dirtying up refused; cleaning up and handing back taken, the recovery not started again
        w = world("py-configuration")
        v = w.create_vehicle("c172", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=300.0, heading_deg=0.0, airspeed_ms=45.0)
        w.step(10)
        s = v.state
        lat0, lon0, ground = s.latitude_rad + 8000.0 / R, s.longitude_rad, s.altitude_msl_m - s.altitude_agl_m
        start = fsim.RunwayPoint(lat0, lon0, ground)
        limit = fsim.RunwayPoint(lat0 + 3000.0 / R, lon0, ground)
        v.load_airfield(fsim.Airfield(7, "", fsim.HOLD, (fsim.Runway(3, 0.0, 3000.0, fsim.RunwayCoordinates(), fsim.RunwayCoordinates(start, start, limit)),)))
        self.assertEqual(v.support("fsim.guidance.recovery/configuration").support, fsim.Support.SUPPORTED)
        a = v.submit_behavior("recovery", airfield=7, runway=3)
        w.step(1)
        self.assertGreater(v.state.airspeed_calibrated_ms, 43.7)
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(configuration=2)
        self.assertEqual(refused.exception.reason, "unavailable")
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(runway=3)
        self.assertEqual(refused.exception.reason, "invalid_parameter")
        self.assertFalse(a.update(configuration=1))
        w.step(1)
        self.assertEqual(v.get_property("fcs/flap-cmd-norm"), 0.0)
        self.assertEqual(a.setpoint().kwargs["configuration"], 1.0)
        self.assertFalse(a.update(configuration=0))
        self.assertEqual(a.info.state, fsim.ActivityState.ACTIVE)
        held = v.submit_behavior("hold")
        with self.assertRaises(fsim.Rejected) as refused:
            held.update(radius_m=500.0)
        self.assertEqual(refused.exception.reason, "not_updatable")

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
