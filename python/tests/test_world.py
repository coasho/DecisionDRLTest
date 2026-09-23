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


if __name__ == "__main__":
    unittest.main()
