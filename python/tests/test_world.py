import math
import os
import tempfile
import unittest

import numpy as np

import fsim
from fsim import HOLD, Level


def make_world(**options):
    options.setdefault("publish", False)
    options.setdefault("workers", 1)
    options.setdefault("pin_workers", False)
    return fsim.World(options.pop("name", "py-test"), **options)


def fly(world, name, **ic):
    ic.setdefault("latitude_deg", 37.62)
    ic.setdefault("longitude_deg", -122.38)
    ic.setdefault("altitude_msl_m", 1500.0)
    ic.setdefault("airspeed_ms", 60.0)
    return world.create_vehicle(name, **ic)


class WorldTest(unittest.TestCase):
    def test_vehicles_state_and_commands(self):
        world = make_world()
        a = fly(world, "alpha", heading_deg=90.0)
        b = fly(world, "bravo", longitude_deg=-122.36)
        self.assertEqual(len(world), 2)
        self.assertEqual(world.vehicle("bravo"), b)
        self.assertEqual(world.vehicle(a.id).name, "alpha")
        self.assertEqual(a.type, "jsbsim:c172x")
        with self.assertRaises(KeyError):
            world.vehicle("charlie")
        with self.assertRaises(fsim.Error):
            fly(world, "alpha")  # duplicate name

        s = a.state
        self.assertAlmostEqual(s.altitude_msl_m, 1500.0, delta=1.0)
        a.command_attitude(roll_rad=0.3, pitch_rad=0.03, airspeed_ms=60.0)
        self.assertEqual(a.active_level, Level.ATTITUDE)
        b.command_behavior("pursuit", target=a, range_m=150.0)
        self.assertEqual(b.active_level, Level.BEHAVIOR)
        world.step(90)  # 3 s
        self.assertAlmostEqual(world.time, 3.0, places=9)
        self.assertAlmostEqual(s.sim_time, 3.0, places=6)  # the same view, updated in place
        self.assertGreater(s.euler_rad[0], 0.1)  # rolling right as commanded
        self.assertIn("VehicleState", repr(s))

    def test_every_level(self):
        world = make_world()
        v = fly(world, "v")
        v.command_actuator(throttle=0.8)
        self.assertEqual(v.active_level, Level.ACTUATOR)
        v.command_acceleration(load_factor_g=1.5)
        self.assertEqual(v.active_level, Level.ACCELERATION)
        v.command_velocity(airspeed_ms=65.0, vertical_speed_ms=2.0)
        self.assertEqual(v.active_level, Level.VELOCITY)
        v.fly_to(37.7, -122.4, 1600.0)
        self.assertEqual(v.active_level, Level.POSITION)
        v.command_behavior("waypoints", points=[(math.radians(37.7), math.radians(-122.4), 1600.0, 60.0, 300.0)])
        self.assertEqual(v.active_level, Level.BEHAVIOR)
        world.step(10)

    def test_state_views_survive_many_more_vehicles(self):
        world = make_world()
        first = fly(world, "first")
        view = first.state
        for i in range(20):
            fly(world, "more-%d" % i, longitude_deg=-122.38 + 0.001 * i)
        world.step(2)
        self.assertAlmostEqual(view.sim_time, world.time, places=9)

    def test_batched_states_and_commands(self):
        world = make_world()
        vehicles = [fly(world, "v%d" % i, longitude_deg=-122.38 + 0.01 * i) for i in range(5)]
        ids = world.ids(vehicles)
        self.assertEqual(ids.dtype, np.uint32)
        states = world.states(ids)
        self.assertEqual(states.shape, (5,))
        for i, v in enumerate(vehicles):
            self.assertEqual(states["altitude_msl_m"][i], v.state.altitude_msl_m)
            self.assertEqual(states["euler_rad"][i][2], v.state.euler_rad[2])

        rows = np.array([[-0.2, 0.02, HOLD, 0.6, HOLD, 55.0]] * 5)
        self.assertEqual(rows.shape[1], len(fsim.COMMAND_FIELDS[Level.ATTITUDE]))
        world.command(Level.ATTITUDE, ids, rows)
        self.assertTrue(all(v.active_level == Level.ATTITUDE for v in vehicles))
        world.step(30)
        out = np.empty(5, dtype=fsim.vehicle_state_dtype)
        self.assertIs(world.states(ids, out=out), out)
        self.assertTrue((out["euler_rad"][:, 0] < -0.05).all())  # every one banking left
        sensed = world.states(ids, sensed=True)
        self.assertEqual(sensed.shape, (5,))

        # Vehicles and plain lists work too: converted, then the same call.
        world.command(Level.VELOCITY, vehicles, [[60.0, 1.0, HOLD, HOLD]] * 5)
        self.assertTrue(all(v.active_level == Level.VELOCITY for v in vehicles))
        self.assertIs(world.states(vehicles, out=out), out)
        with self.assertRaises(ValueError):
            world.command(Level.ATTITUDE, ids, np.zeros((5, 4)))  # wrong number of fields
        with self.assertRaises(fsim.Error):
            world.command(Level.ATTITUDE, np.array([99999], np.uint32), rows[:1])

    def test_torch_tensors_as_command_values(self):
        try:
            import torch
        except ImportError:
            self.skipTest("torch is not installed")
        world = make_world()
        vehicles = [fly(world, "v%d" % i, longitude_deg=-122.38 + 0.01 * i) for i in range(3)]
        rows = torch.tensor([[-0.2, 0.02, HOLD, 0.6, HOLD, 55.0]] * 3, requires_grad=True)  # float32, HOLD kept
        world.command(Level.ATTITUDE, vehicles, rows)
        self.assertTrue(all(v.active_level == Level.ATTITUDE for v in vehicles))
        world.step(30)
        self.assertTrue((world.states(vehicles)["euler_rad"][:, 0] < -0.05).all())  # every one banking left

    def test_environment_effects_properties_comm(self):
        world = make_world()
        a = fly(world, "a")
        b = fly(world, "b", longitude_deg=-122.37)
        world.set_wind(180.0, 5.0)
        self.assertEqual(world.environment["wind_speed_ms"], 5.0)
        world.add_effect("wind_gusts")
        b.add_effect("gaussian_sensor_noise", position_sigma_m=2.0)
        with self.assertRaises(fsim.Error):
            b.add_effect("no_such_effect")
        a.attach_protocol("beacon")
        b.send(a, b"hello", channel=4)
        world.step(90)
        self.assertGreater(a.get_property("atmosphere/wind-north-fps"), 10.0)
        with self.assertRaises(fsim.Error):
            a.get_property("no/such/property")
        self.assertIsInstance(a.inbox(), list)
        b.clear_effects()
        a.set_controller_parameter(Level.ATTITUDE, "roll.kp", 2.5)
        self.assertEqual(a.controller_parameter(Level.ATTITUDE, "roll.kp"), 2.5)
        self.assertEqual(a.controller_parameter(Level.VELOCITY, "schedule.tas_ms"), 0.0)   # a stock aircraft: no schedule
        with self.assertRaises(fsim.Error):
            a.controller_parameter(Level.ATTITUDE, "nope")
        with self.assertRaises(fsim.Error):
            a.use_controller(Level.ATTITUDE, "nope")

    def test_reset_and_remove(self):
        world = make_world()
        v = fly(world, "v")
        world.step(30)
        v.reset(altitude_msl_m=2000.0)
        self.assertAlmostEqual(v.state.altitude_msl_m, 2000.0, delta=1.0)
        v.remove()
        self.assertEqual(len(world), 0)
        self.assertNotIn("v", world)

    def test_scenario(self):
        text = (
            '{ "world": { "name": "py-scenario", "publish": false, "workers": 1, "pin_workers": false },'
            '  "environment": { "wind": { "direction_deg": 180, "speed_ms": 5 } },'
            '  "vehicles": [ { "name": "v", "count": 2, "initial": { "alt_msl_m": 1200, "heading_deg": 45, "airspeed_ms": 60 },'
            '                   "command": { "level": "attitude", "pitch_deg": 3 } } ] }'
        )
        scenario = fsim.Scenario(json=text)
        self.assertEqual(scenario.vehicle_count, 2)
        self.assertEqual(scenario.world_options["name"], "py-scenario")
        world = fsim.World.from_scenario(scenario)
        self.assertEqual([v.name for v in world], ["v-1", "v-2"])
        world.step(10)
        self.assertAlmostEqual(world.vehicle("v-2").state.altitude_msl_m, 1200.0, delta=30.0)
        with self.assertRaises(fsim.Error):
            fsim.Scenario(json='{ "vehicles": [ { } ] }')

    def test_recording_round_trip(self):
        directory = os.environ.get("FSIM_TEST_OUTPUT") or tempfile.mkdtemp()
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, "py-record.fsrec")
        world = make_world(name="py-record", record_path=path)
        fly(world, "rec")
        world.step(30)
        world.close()
        del world
        import gc
        gc.collect()
        rec = fsim.Recording(path)
        self.assertEqual(rec.world_name, "py-record")
        self.assertGreater(len(rec), 10)
        samples = rec.samples(len(rec) - 1)
        self.assertEqual(samples.dtype, fsim.recorded_sample_dtype)
        self.assertGreater(samples["state"]["sim_time"][0], 0.5)
        self.assertEqual(rec.events(0)[0]["name"], "rec")

    def test_log_level(self):
        fsim.set_log_level("error")
        fsim.set_log_level("warning")
        with self.assertRaises(KeyError):
            fsim.set_log_level("loud")



class CapabilityTest(unittest.TestCase):
    """Capabilities and activities (docs/sdk/control.md)."""

    def test_discovery(self):
        world = make_world(name="py-caps")
        v = fly(world, "discover")
        caps = {c.id: c for c in v.capabilities()}
        for cid in ("fsim.flight.actuator", "fsim.flight.velocity", "fsim.guidance.hold", "fsim.guidance.waypoints"):
            self.assertIn(cid, caps)
        velocity = caps["fsim.flight.velocity"]
        self.assertEqual(velocity.level, Level.VELOCITY)
        self.assertEqual([p.name for p in velocity.parameters], list(fsim.SETPOINT_FIELDS[Level.VELOCITY]))
        self.assertEqual([p.supported for p in velocity.parameters], [True] * 4 + [False] * 2)  # a wing flies through the air
        self.assertTrue(caps["fsim.guidance.waypoints"].terminating)
        self.assertEqual(velocity.axis_groups, 1 | 2 | 4)  # lateral, pitch, thrust
        self.assertEqual(caps["fsim.flight.actuator"].axis_groups & 8, 8)  # any primary axis alone
        self.assertEqual(caps["fsim.guidance.hold"].axis_groups, 0)  # all or nothing
        self.assertTrue(caps["fsim.guidance.pursuit"].needs_target)
        self.assertEqual(v.capability_status("fsim.guidance.hold"), (fsim.Availability.AVAILABLE, "none"))
        self.assertEqual(v.capability_status("no.such.thing")[1], "unknown_capability")
        # the profile: a stock c172x carries no sections
        self.assertEqual(v.profile_section("control"), (0, 0))
        self.assertTrue(math.isnan(v.profile_value("envelope/clean/n_max")))
        with self.assertRaises(KeyError):
            v.profile_section("nonsense")

    def test_submit_update_cancel(self):
        world = make_world(name="py-activities")
        v = fly(world, "active")
        a = v.submit(Level.VELOCITY, airspeed_ms=60.0, vertical_speed_ms=1.0)
        self.assertIsInstance(a, fsim.Activity)
        self.assertEqual(a.id >> 32, v.id)
        self.assertEqual(a.vehicle, v)
        self.assertEqual(a.state, fsim.ActivityState.PENDING)
        world.step()
        self.assertEqual(a.state, fsim.ActivityState.ACTIVE)
        self.assertTrue(a.live)
        self.assertFalse(a.update(airspeed_ms=60.0, vertical_speed_ms=0.0))
        with self.assertRaises(TypeError):
            a.update(no_such_field=1.0)
        self.assertTrue(v.submit(Level.ATTITUDE, roll_rad=4.0).clamped)  # beyond +-pi: clamped, and it preempted `a`
        info = world.activity(a)
        self.assertEqual(info.state, fsim.ActivityState.CANCELED)
        self.assertEqual(info.reason, "preempted")
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(airspeed_ms=60.0)
        self.assertEqual(refused.exception.reason, "activity_ended")
        self.assertIsInstance(refused.exception, fsim.Error)
        hold = v.submit_behavior("hold")
        hold.cancel()
        self.assertEqual(hold.info.reason, "requested")
        self.assertGreaterEqual(len(v.activities()), 3)

    def test_behaviour_takes_no_update(self):
        world = make_world(name="py-behaviour-update")
        v = fly(world, "behaviour")
        hold = v.submit_behavior("hold")
        with self.assertRaises(fsim.Rejected) as refused:
            hold.update()  # a new target is a new submit_behavior
        self.assertEqual(refused.exception.reason, "not_updatable")
        hold.cancel()
        with self.assertRaises(fsim.Rejected) as refused:
            hold.update()
        self.assertEqual(refused.exception.reason, "activity_ended")

    def test_route_points_are_checked(self):
        world = make_world(name="py-route-check")
        v = fly(world, "route")
        s = v.state
        good = (s.latitude_rad + 0.001, s.longitude_rad, 1500.0, HOLD, 200.0)
        self.assertTrue(v.submit_behavior("waypoints", points=[good]).live)
        for bad in ((math.nan, s.longitude_rad, 1500.0, HOLD, 200.0), (2.0, s.longitude_rad, 1500.0, HOLD, 200.0),
                    (s.latitude_rad, s.longitude_rad, 1500.0, HOLD, 0.0), (s.latitude_rad, s.longitude_rad, 1500.0, -5.0, 200.0)):
            with self.assertRaises(fsim.Rejected) as refused:
                v.submit_behavior("waypoints", points=[good, bad])
            self.assertEqual(refused.exception.reason, "invalid_parameter")

    def test_progress_detail_and_commanded_state(self):
        world = make_world(name="py-interface")
        v = fly(world, "interface")
        s = v.state
        points = [(s.latitude_rad, s.longitude_rad + 0.0006, 1500.0, HOLD, 300.0),
                  (s.latitude_rad + 0.0005, s.longitude_rad + 0.0006, 1600.0, HOLD, 300.0)]
        route = v.submit_behavior("waypoints", points=points)
        world.step(10)
        p = route.progress
        self.assertIsInstance(p, fsim.ActivityProgress)
        self.assertEqual((p.segment, p.segments), (0, 2))
        self.assertGreater(p.distance_to_go_m, 1000.0)
        self.assertTrue(0.0 <= p.percent < 100.0)
        self.assertEqual(p.altitude_msl_m, 1500.0)
        c = v.commanded
        self.assertIsInstance(c, fsim.CommandedState)
        self.assertEqual(c.top_level, Level.BEHAVIOR)
        self.assertEqual(c.altitude_msl_m, 1500.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit(Level.ATTITUDE, roll_rad=4.0, range=fsim.RangePolicy.REJECT)
        self.assertEqual((refused.exception.index, refused.exception.constraint), (0, "max_orientation"))
        self.assertIsNone(refused.exception.section)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_behavior("waypoints", points=points + [(math.nan, 0.0, 0.0, HOLD, 1.0)])
        self.assertEqual(refused.exception.index, 2)
        # A-GRA's vocabulary
        from fsim import agra
        self.assertEqual(agra.performance_constraint("max_orientation"), "MAX_ORIENTATION_LIMIT_EXCEEDED")
        self.assertEqual(agra.cannot_comply("invalid_waypoint"), "INFEASIBLE_ROUTE")
        self.assertEqual(agra.validation_result("invalid_waypoint"), "INVALID_WAYPOINT")
        self.assertIn(agra.activity_state(route.info), ("ACTIVE_UNCONSTRAINED", "ACTIVE_PARTIALLY_CONSTRAINED", "ACTIVE_FULLY_CONSTRAINED"))
        self.assertEqual(agra.command_processing_state(1), "REJECTED")
        caps = {cap.id: cap for cap in v.capabilities()}
        self.assertEqual(caps["fsim.guidance.formation"].mode, "formation")
        self.assertEqual(caps["fsim.flight.velocity"].mode, "none")
        self.assertEqual(agra.flight_capabilities(v), {"ALTITUDE_STACKED_MARSHALL": ["fsim.guidance.marshall"],
                                                       "CURVE_FOLLOWING": ["fsim.guidance.curve"], "FORMATION": ["fsim.guidance.formation"],
                                                       "HSA_CSA": ["fsim.guidance.hsa"], "LOITER": ["fsim.guidance.pattern"],
                                                       "MUST_FLY": ["fsim.guidance.must_fly"], "ROUTE_INTERCEPT": ["fsim.guidance.intercept"],
                                                       "WAYPOINT_FOLLOWING": ["fsim.guidance.route"]})
        route.cancel()
        self.assertEqual(agra.activity_state(route.info), "FAILED")
        self.assertEqual(agra.cannot_comply(route.info.reason), "CANCELED")

    def test_hsa_mode(self):
        world = make_world(name="py-hsa")
        v = fly(world, "hsa")
        a = v.submit_hsa(heading_rad=math.pi, speed=55.0, speed_reference="true_airspeed", altitude_m=1600.0)
        self.assertEqual(a.level, "hsa")
        world.step()
        self.assertEqual(a.state, fsim.ActivityState.ACTIVE)
        a.update(altitude_m=1700.0)  # only the altitude: the rest as commanded
        world.step()
        p = a.progress
        self.assertEqual((p.altitude_msl_m, p.speed_ms), (1700.0, 55.0))
        self.assertAlmostEqual(abs(p.heading_rad), math.pi)
        self.assertEqual(p.speed_reference, fsim.SpeedReference.TRUE_AIRSPEED)
        a.update(speed=0.25, speed_reference=fsim.SpeedReference.MACH)
        world.step()
        self.assertEqual((a.progress.speed_ms, a.progress.speed_reference), (0.25, 3.0))
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(speed_reference="calibrated_airspeed")  # a reference needs its value in an UPDATE
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 2))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=1.0, course_rad=1.0)
        self.assertEqual(refused.exception.index, 1)
        caps = {c.id: c for c in v.capabilities()}
        self.assertEqual(caps["fsim.guidance.hsa"].mode, "hsa_csa")
        self.assertEqual([q.name for q in caps["fsim.guidance.hsa"].parameters], list(fsim.MODE_FIELDS["hsa"]))
        self.assertEqual(fsim.agra.flight_capabilities(v)["HSA_CSA"], ["fsim.guidance.hsa"])

    def test_route_mode(self):
        world = make_world(name="py-route")
        v = fly(world, "route")
        s = v.state
        lat, lon = float(s.latitude_rad), float(s.longitude_rad)
        step = 3000.0 / 6371008.8
        points = [fsim.Waypoint(lat, lon + step / math.cos(lat), altitude_m=1550.0, speed=55.0, speed_reference="true_airspeed", id=41),
                  {"latitude_rad": lat + step, "longitude_rad": lon + step / math.cos(lat), "turn": "fly_over", "id": 42},
                  (lat + step, lon + 2 * step / math.cos(lat))]  # a row in Waypoint's order: the rest as before
        a = v.submit_route(points, end="loiter")
        self.assertEqual(a.level, "route")
        world.step(10)
        p = a.progress
        self.assertEqual((p.segment, p.segments, p.segment_id), (0, 3, 41))
        self.assertEqual((p.speed_ms, p.speed_reference), (55.0, fsim.SpeedReference.TRUE_AIRSPEED))
        self.assertTrue(0.0 < p.percent < 100.0 and p.distance_to_go_m > 6000.0)
        # its options alone: its waypoints kept, flown afresh from the point it names
        a.update(start=2)
        world.step()
        self.assertEqual(a.progress.segment, 2)
        # new waypoints (from the first: the start it had is kept, as every option not given); one it cannot fly is refused, naming it
        with self.assertRaises(fsim.Rejected) as refused:
            a.update_route([points[0], (float("nan"), lon)], start=0)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 1))
        a.update_route(points[:2], projection="rhumb", start=0)
        world.step()
        self.assertEqual((a.progress.segment, a.progress.segments), (0, 2))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route([])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_waypoint", 0))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_route(points, start=5)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 3))
        caps = {c.id: c for c in v.capabilities()}
        self.assertEqual(caps["fsim.guidance.route"].mode, "waypoint_following")
        self.assertEqual([q.name for q in caps["fsim.guidance.route"].parameters], list(fsim.MODE_FIELDS["route"]))

    def test_pattern_mode(self):
        world = make_world(name="py-pattern")
        v = fly(world, "pattern")
        a = v.submit_pattern(pattern="racetrack", radius_m=900.0, leg_m=3000.0, course_rad=0.0, duration_s=600.0)
        self.assertEqual(a.level, "pattern")
        world.step(5)
        p = a.progress
        self.assertEqual(p.segments, 4)
        self.assertTrue(0.0 < p.percent < 5.0 and 590.0 < p.time_to_go_s <= 600.0)
        a.update(radius_m=1100.0)  # only the radius: the rest as commanded
        world.step()
        self.assertEqual(a.progress.segments, 4)
        with self.assertRaises(fsim.Rejected) as refused:
            a.update(speed_reference="mach")  # a reference needs its value in an UPDATE
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 9))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_pattern(pattern=7)
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 0))
        caps = {c.id: c for c in v.capabilities()}
        self.assertEqual(caps["fsim.guidance.pattern"].mode, "loiter")
        self.assertEqual([q.name for q in caps["fsim.guidance.pattern"].parameters], list(fsim.MODE_FIELDS["pattern"]))
        self.assertEqual(fsim.PatternKind.FIGURE_EIGHT, 2)

    def test_curve_mode(self):
        world = make_world(name="py-curve")
        v = fly(world, "curve")

        def straight(n0, e0, n1, e1):  # control points evenly along a line
            return fsim.BezierSegment([n0 + (n1 - n0) * k / 5 for k in range(6)], [e0 + (e1 - e0) * k / 5 for k in range(6)], [0.0] * 6)

        # east from where it is (the reference, left out), then a bend north; the second as a dict, the third as a triple
        first = [straight(0, 0, 0, 2000), {"north": [0, 0, 0, 200, 600, 1000], "east": [2000, 2400, 2800, 3000, 3000, 3000], "down": [0] * 6}]
        a = v.submit_curve(first, speed_max_ms=50.0, end="loiter")
        self.assertEqual(a.level, "curve")
        world.step(10)
        p = a.progress
        self.assertEqual((p.segment, p.segments), (0, 2))
        self.assertEqual(p.speed_reference, fsim.SpeedReference.TRUE_AIRSPEED)  # (a wing holds its airspeed)
        self.assertTrue(0.0 < p.percent < 100.0 and p.distance_to_go_m > 2000.0)
        # appended while it flies: the same activity, on to the new end
        a.append([([1000 + 400 * k for k in range(6)], [3000] * 6, [0] * 6)])
        world.step()
        self.assertEqual(a.progress.segments, 3)
        # where the curve does not end: refused, naming the segment; so is a hairpin too tight, with its section
        with self.assertRaises(fsim.Rejected) as refused:
            a.append([straight(0, 0, 500, 500)])
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_curve", 0))
        hairpin = fsim.BezierSegment([0, 30, 60, 60, 30, 0], [0, 0, 0, 20, 20, 20], [0] * 6)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_curve([hairpin])
        r = refused.exception
        self.assertEqual((r.reason, r.index, r.constraint), ("invalid_curve", 0, "max_turn_rate"))
        self.assertTrue(0.0 < r.section[0] < r.section[1] < 1.0)
        # its options alone: how it is flown; a new curve through update_curve, flown afresh
        a.update(speed_max_ms=45.0)
        a.update_curve([straight(0, 0, 3000, 0)])
        world.step()
        self.assertEqual((a.progress.segment, a.progress.segments), (0, 1))
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_curve(first, append=1)  # a NEW has nothing to append to
        self.assertEqual((refused.exception.reason, refused.exception.index), ("invalid_parameter", 7))
        caps = {c.id: c for c in v.capabilities()}
        self.assertEqual(caps["fsim.guidance.curve"].mode, "curve_following")
        self.assertEqual([q.name for q in caps["fsim.guidance.curve"].parameters], list(fsim.MODE_FIELDS["curve"]))

    def test_grants_and_performance(self):
        world = make_world(name="py-grants")
        v = fly(world, "granted")
        world.step()
        # the performance: its loops' bank, and computed afresh when they change, the revision counting it
        p = v.performance
        self.assertIsInstance(p, fsim.Performance)
        self.assertFalse(p.hovers)
        self.assertEqual(p.max_bank_rad, v.controller_parameter(fsim.Level.VELOCITY, "max_bank"))
        revision = v.control_revision
        v.set_controller_parameter(fsim.Level.VELOCITY, "max_bank", 0.4)
        self.assertEqual(v.performance.max_bank_rad, 0.4)
        self.assertEqual(v.performance.revision, p.revision + 1)
        self.assertEqual(v.control_revision, revision + 1)
        # Granted: a policy commands only what it holds a grant for
        self.assertEqual(v.control_mode, fsim.ControlMode.OPEN)
        v.set_control_mode("granted")
        self.assertEqual(v.control_mode, fsim.ControlMode.GRANTED)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=1.0)
        self.assertEqual(refused.exception.reason, "not_granted")
        v.request_control("fsim.guidance.hsa")
        self.assertEqual(v.control_status("fsim.guidance.hsa"), fsim.ControlStatus(allowed=True, granted=True))
        a = v.submit_hsa(heading_rad=1.0)
        world.step()
        v.release_control("fsim.guidance.hsa")  # it lets go: its activity ends
        self.assertEqual((a.info.state, a.info.reason), (fsim.ActivityState.CANCELED, "released"))
        # the platform's restriction: a request refused with its reason
        v.set_availability("fsim.guidance.hsa", "temporarily_unavailable", "collision_avoidance")
        self.assertEqual(v.capability_status("fsim.guidance.hsa"), (fsim.Availability.TEMPORARILY_UNAVAILABLE, "collision_avoidance"))
        with self.assertRaises(fsim.Rejected) as refused:
            v.request_control("fsim.guidance.hsa")
        self.assertEqual(refused.exception.reason, "collision_avoidance")
        v.set_availability("fsim.guidance.hsa", "available")
        # not allowed; revoked
        v.set_allowed("fsim.guidance.hsa", False)
        with self.assertRaises(fsim.Rejected) as refused:
            v.request_control("fsim.guidance.hsa")
        self.assertEqual(refused.exception.reason, "not_allowed")
        v.set_allowed("fsim.guidance.hsa", True)
        v.request_control("fsim.guidance.hsa")
        a = v.submit_hsa(heading_rad=1.0)
        v.revoke_control("fsim.guidance.hsa", "restricted")
        self.assertEqual(a.info.reason, "restricted")
        self.assertFalse(v.control_status("fsim.guidance.hsa").granted)
        with self.assertRaises(fsim.Error):
            v.request_control("fsim.guidance.nonsense")
        with self.assertRaises(fsim.Error):  # not a reason the platform gives: it would misreport the end
            v.revoke_control("fsim.guidance.hsa", "preempted")
        # an Activity declares the source it was submitted with: a policy's handle cannot touch the platform's
        o = v.submit_hsa(heading_rad=1.2, source=fsim.Source.OVERRIDE)
        self.assertEqual(o.source, fsim.Source.OVERRIDE)
        with self.assertRaises(fsim.Rejected) as refused:
            fsim.Activity(world, o.id, "hsa").cancel()  # (a handle of its id without its source: the policy's)
        self.assertEqual(refused.exception.reason, "authority_held")
        o.update(heading_rad=1.3)
        o.cancel()
        v.set_control_mode(fsim.ControlMode.OPEN)
        v.submit_hsa(heading_rad=1.0)

    def test_authority_and_refusals(self):
        world = make_world(name="py-authority")
        v = fly(world, "held")
        operator = v.submit(Level.VELOCITY, airspeed_ms=60.0, source=fsim.Source.OVERRIDE)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit(Level.ATTITUDE, roll_rad=0.1)
        self.assertEqual(refused.exception.reason, "authority_held")
        self.assertEqual(refused.exception.other, operator.id)
        with self.assertRaises(fsim.Error):
            v.command_attitude(roll_rad=0.1)  # the per-step path is refused too
        operator.cancel()
        v.command_attitude(roll_rad=0.1)
        with self.assertRaises(fsim.Rejected):
            v.submit(Level.ATTITUDE, roll_rad=4.0, range=fsim.RangePolicy.REJECT)
        with self.assertRaises(fsim.Error):
            v.command_behavior("no_such_behaviour")  # refused since 2026-09-26; it used to be ignored

    def test_support(self):
        world = make_world(name="py-support")
        v = fly(world, "support")
        flaps = v.submit_support("flaps", position=0.5)
        self.assertEqual(flaps.level, "flaps")
        world.step()
        self.assertEqual(flaps.state, fsim.ActivityState.ACTIVE)
        self.assertFalse(flaps.update(position=0.3))
        with self.assertRaises(TypeError):
            flaps.update(down=1.0)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_support("speedbrake", position=1.0)  # a c172x has none, and declares nothing: applicable, not built
        self.assertEqual(refused.exception.reason, "not_implemented")
        self.assertEqual(v.support("fsim.support.speedbrake").support, fsim.Support.NOT_IMPLEMENTED)
        with self.assertRaises(ValueError):
            v.submit_support("afterburner")
        brakes = v.submit_support("wheel_brakes", 0.2, 0.3)
        self.assertEqual(world.activity(brakes).axes, 1 << 6)  # the brakes axis
        flaps.cancel()
        self.assertEqual(flaps.info.reason, "requested")

    def test_support_and_availability(self):
        """Discovery that tells the truth (docs/flight-autonomy.md, 4): what an aircraft can do at all, and now."""
        world = make_world(name="py-discovery")
        viper = fly(world, "viper", type="jsbsim:f16c", altitude_msl_m=3000.0, airspeed_ms=160.0)
        self.assertIn("fsim.guidance.must_fly", fsim.SUPPORT_FEATURES)
        hover = viper.support("fsim.guidance.hover")
        self.assertEqual(hover.support, fsim.Support.NOT_SUPPORTED)
        self.assertEqual(hover.rules, ("R1",))
        self.assertTrue(hover.evidence.startswith("vertical_flight = false: USAF F-16 fact sheet"))
        with self.assertRaises(fsim.Rejected) as refused:
            viper.submit_behavior("hover")
        self.assertEqual(refused.exception.reason, "not_supported")
        discretized = viper.support("fsim.guidance.curve/discretized")  # (applicable, not built: the stage that builds it)
        self.assertEqual((discretized.support, discretized.stage), (fsim.Support.NOT_IMPLEMENTED, 17))
        self.assertEqual(viper.support("fsim.guidance.route/metadata").support, fsim.Support.SUPPORTED)  # (FA-7a)
        self.assertEqual(viper.support("fsim.guidance.route/path_terminators").support, fsim.Support.SUPPORTED)  # (FA-6f)
        self.assertEqual(viper.support("fsim.guidance.route/conditional_segment").support, fsim.Support.PARTIAL)  # (FA-6e2a)
        self.assertEqual(viper.support("fsim.guidance.route/paths").support, fsim.Support.SUPPORTED)  # (FA-6e1)
        self.assertEqual(viper.support("fsim.guidance.route/altitude/barometric").support, fsim.Support.SUPPORTED)  # (FA-6a)
        self.assertEqual(viper.support("fsim.guidance.hsa/direction/magnetic_north").support, fsim.Support.SUPPORTED)
        self.assertEqual(viper.support("hold").feature, "fsim.guidance.hold")  # a behaviour's id finds it
        table = viper.support_table()
        self.assertEqual(len(table), len(fsim.SUPPORT_FEATURES))
        with self.assertRaises(fsim.Error):
            viper.support("fsim.guidance.warp_drive")
        status = viper.availability("fsim.guidance.hover")
        self.assertEqual((status.availability, status.reason), (fsim.Availability.UNAVAILABLE, "not_supported"))
        self.assertEqual(status.reasons, ("not_supported",))
        viper.set_availability("fsim.flight.velocity", "temporarily_unavailable", "collision_avoidance", associated=7, next_available_s=30.0)
        avoiding = viper.availability("fsim.flight.velocity")
        self.assertEqual((avoiding.reason, avoiding.associated, avoiding.next_available_s), ("collision_avoidance", 7, 30.0))
        caps = {c.id: c for c in viper.capabilities()}
        self.assertEqual(caps["fsim.guidance.hold"].superseded, "fsim.guidance.hsa")
        self.assertEqual(caps["fsim.envelope.protection"].accepted, 8)
        self.assertEqual(caps["fsim.guidance.hsa"].accepted, 3)
        # on the ground: a policy's guidance waits (on_ground); the gear's placard narrows its range
        parked = world.create_vehicle("parked", type="jsbsim:f16c", latitude_deg=37.7, longitude_deg=-122.3, on_ground=True)
        for _ in range(60):
            world.step()
        self.assertEqual(parked.availability("fsim.guidance.hsa").reason, "on_ground")
        with self.assertRaises(fsim.Rejected) as grounded:
            parked.submit_hsa(heading_rad=1.0, speed=80.0, speed_reference="true_airspeed", altitude_m=500.0)
        self.assertEqual(grounded.exception.reason, "on_ground")
        self.assertEqual(parked.capability_limits("fsim.support.gear"), (("down", 0.5, 1.0),))
        with self.assertRaises(fsim.Rejected) as up:
            parked.submit_support("gear", 0.0)
        self.assertEqual(up.exception.reason, "unavailable")
        from fsim import agra
        self.assertEqual(agra.validation_result("not_supported"), "CAPABILITY_NOT_SUPPORTED")

    def test_axes_apart(self):
        world = make_world(name="py-axes")
        v = fly(world, "apart", heading_deg=90.0, airspeed_ms=55.0)
        hold = v.submit(Level.VELOCITY, airspeed_ms=55.0, vertical_speed_ms=0.0, source=fsim.Source.AUTOPILOT,
                        axes=fsim.Axis.PITCH | fsim.Axis.THRUST)
        bank = v.submit(Level.ATTITUDE, roll_rad=0.3, axes=fsim.Axis.ROLL)
        self.assertEqual(bank.info.axes, fsim.Axis.LATERAL)  # the loop that banks also coordinates
        self.assertTrue(hold.live)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit(Level.ATTITUDE, pitch_rad=0.1, axes=fsim.Axis.PITCH)
        self.assertEqual(refused.exception.reason, "authority_held")
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit(Level.ATTITUDE, axes=fsim.Axis.FLAPS)
        self.assertEqual(refused.exception.reason, "invalid_axes")
        altitude = v.state.altitude_msl_m
        world.step(450)
        self.assertLess(abs(v.state.euler_rad[0] - 0.3), 0.06)
        self.assertLess(abs(v.state.altitude_msl_m - altitude), 20.0)
        self.assertEqual(hold.state, fsim.ActivityState.ACTIVE)

    def test_vehicle_default(self):
        world = make_world(name="py-default")
        v = fly(world, "held", heading_deg=90.0, airspeed_ms=55.0)
        self.assertEqual(v.vehicle_default, fsim.VehicleDefault.NEUTRAL)
        v.set_vehicle_default("hold")
        self.assertEqual(v.vehicle_default, fsim.VehicleDefault.HOLD)
        self.assertEqual(v.active_level, Level.VELOCITY)
        altitude = v.state.altitude_msl_m
        world.step(600)
        self.assertLess(abs(v.state.altitude_msl_m - altitude), 15.0)
        self.assertLess(abs(v.state.airspeed_true_ms - 55.0), 3.0)
        v.set_vehicle_default(fsim.VehicleDefault.NEUTRAL)
        world.step()
        self.assertEqual(v.get_property("fcs/throttle-cmd-norm[0]"), 0.0)

    def test_engines(self):
        world = make_world(name="py-engines")
        twin = fly(world, "twin", type="jsbsim:a10c", altitude_msl_m=3000.0, airspeed_ms=140.0)
        caps = {c.id: c for c in twin.capabilities()}
        self.assertEqual([p.name for p in caps["fsim.flight.engines"].parameters], ["throttle_1", "throttle_2"])
        v = twin.submit(Level.ATTITUDE, pitch_rad=0.03, axes=fsim.Axis.LATERAL | fsim.Axis.PITCH)
        engines = twin.submit_support("engines", throttle_1=0.9, throttle_2=0.4)

        def throttles():  # what the flight model was given
            return [twin.get_property("fcs/throttle-cmd-norm[%d]" % i) for i in range(2)]

        world.step()
        self.assertEqual(throttles(), [0.9, 0.4])
        engines.update(throttle_1=0.7)  # the other held
        world.step()
        self.assertEqual(throttles(), [0.7, 0.4])
        self.assertTrue(v.live)
        single = fly(world, "single", longitude_deg=-122.3)
        with self.assertRaises(fsim.Rejected) as refused:
            single.submit_support("engines", 0.5, HOLD, HOLD, HOLD)
        self.assertEqual(refused.exception.reason, "not_implemented")  # a stock aircraft's profile says nothing of its engines

    def test_protection(self):
        world = make_world(name="py-protection")
        stock = fly(world, "stock")
        self.assertEqual(stock.protection, fsim.ProtectionMode.OFF)  # a stock aircraft has no envelope
        viper = fly(world, "viper", type="jsbsim:f16c", altitude_msl_m=3000.0, airspeed_ms=160.0, longitude_deg=-122.3)
        self.assertEqual(viper.protection, fsim.ProtectionMode.LIMIT)
        self.assertIn("fsim.envelope.protection", [c.id for c in viper.capabilities()])
        viper.set_protection("report")
        self.assertEqual(viper.protection, fsim.ProtectionMode.REPORT)
        world.step(10)
        e = viper.envelope()
        self.assertEqual(e.mode, fsim.ProtectionMode.REPORT)
        self.assertEqual(sorted(e.limits), sorted(fsim.LIMITS))
        self.assertEqual(fsim.LIMITS[:3], ("load_factor_max", "load_factor_min", "alpha_max"))
        self.assertEqual(e.limits["alpha_max"], fsim.LimitStatus(0, 0, 0.0, 0.0))  # level flight: well inside
        with self.assertRaises(ValueError):
            viper.set_protection(7)

    def test_batched_updates(self):
        world = make_world(name="py-batch-updates")
        vs = [fly(world, "b%d" % i, longitude_deg=-122.38 + 0.01 * i) for i in range(3)]
        acts = [v.submit(Level.VELOCITY, airspeed_ms=60.0) for v in vs]
        ids = np.array([a.id for a in acts], dtype=np.uint64)
        rows = np.tile(np.array([60.0, 1.0, HOLD, HOLD]), (3, 1))
        world.update(ids, rows)
        world.update(acts, rows)
        world.step()
        for a in acts:
            self.assertEqual(a.state, fsim.ActivityState.ACTIVE)
        acts[1].cancel()
        with self.assertRaises(fsim.Error):
            world.update(ids, rows)


class RotorcraftTest(unittest.TestCase):
    """The rotorcraft (docs/rotorcraft.md): their own controls' names, velocity over the ground, rows with the new fields."""

    def test_quadrotor_through_the_capability_calls(self):
        world = make_world(name="py-rotorcraft")
        iris = world.create_vehicle("iris", type="jsbsim:iris", latitude_deg=37.62, longitude_deg=-122.38, altitude_msl_m=100.0,
                                    airspeed_ms=0.0)
        caps = {c.id: c for c in iris.capabilities()}
        self.assertEqual([p.name for p in caps["fsim.flight.actuator"].parameters][:4], ["roll", "pitch", "yaw", "thrust"])
        self.assertEqual(caps["fsim.flight.velocity"].axis_groups, 16 | 32 | 4)  # cyclic, yaw, thrust
        self.assertIn("fsim.guidance.hover", caps)
        self.assertNotIn("fsim.guidance.aerobatics", caps)
        self.assertEqual([p.name for p in caps["fsim.flight.engines"].parameters], ["rotor_1", "rotor_2", "rotor_3", "rotor_4"])
        self.assertEqual(iris.profile_section("hover")[0], 1)
        with self.assertRaises(fsim.Rejected):  # a rotorcraft has no longitudinal acceleration of its own
            iris.submit(Level.ACCELERATION, longitudinal_ms2=1.0)
        go = iris.submit(Level.VELOCITY, north_ms=2.0, east_ms=0.0, heading_rad=0.0)
        world.step(300)  # 10 s
        s = iris.state
        self.assertAlmostEqual(s.velocity_ned_ms[0], 2.0, delta=0.2)
        self.assertAlmostEqual(s.altitude_msl_m, 100.0, delta=0.5)
        # the batch path, a row with the rotorcraft's fields: east now
        world.update(np.array([go.id], dtype=np.uint64), np.array([[HOLD, 0.0, 0.0, HOLD, 0.0, 2.0]]))
        world.step(300)
        self.assertAlmostEqual(iris.state.velocity_ned_ms[1], 2.0, delta=0.2)
        self.assertAlmostEqual(iris.state.velocity_ned_ms[0], 0.0, delta=0.2)
        # positional values: the legacy width or the full one
        go.update(HOLD, 0.0, 0.0, HOLD, 0.0, 0.0)
        with self.assertRaises(TypeError):
            go.update(HOLD, 0.0, 0.0)
        hover = iris.submit_behavior("hover")
        world.step(150)
        self.assertEqual(hover.state, fsim.ActivityState.ACTIVE)


if __name__ == "__main__":
    unittest.main()
