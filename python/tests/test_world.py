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
        self.assertEqual([p.name for p in velocity.parameters], list(fsim.COMMAND_FIELDS[Level.VELOCITY]))
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
            v.submit_support("speedbrake", position=1.0)  # a c172x has none
        self.assertEqual(refused.exception.reason, "unknown_capability")
        with self.assertRaises(ValueError):
            v.submit_support("afterburner")
        brakes = v.submit_support("wheel_brakes", 0.2, 0.3)
        self.assertEqual(world.activity(brakes).axes, 1 << 6)  # the brakes axis
        flaps.cancel()
        self.assertEqual(flaps.info.reason, "requested")

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
        self.assertEqual(refused.exception.reason, "unknown_capability")

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


if __name__ == "__main__":
    unittest.main()
