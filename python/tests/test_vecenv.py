import gc
import threading
import time
import unittest

import numpy as np

import fsim


def make(num_envs=4, **options):
    options.setdefault("publish", False)
    options.setdefault("workers", 1)
    options.setdefault("seed", 7)
    return fsim.VecEnv(num_envs, **options)


class VecEnvTest(unittest.TestCase):
    def test_results_are_views_rewritten_in_place(self):
        env = make()
        obs = env.reset()
        self.assertEqual(obs.shape, (4, env.observation_size))
        self.assertEqual(obs.dtype, np.float32)
        self.assertFalse(obs.flags.writeable)
        self.assertEqual(env.observation_names[0], "alt_msl_km")
        self.assertEqual(env.action_names, ["aileron", "elevator", "rudder", "throttle"])
        before = obs.copy()
        actions = np.zeros((4, env.action_size), np.float32)
        actions[:, 3] = 0.6
        result = env.step(actions)
        o, r, term, trunc = result
        self.assertIs(o, obs)  # the same array, not a copy
        self.assertIs(env.step(actions), result)  # nothing is built per step
        self.assertGreater(np.abs(obs - before).max(), 0.0)
        self.assertEqual(r.shape, (4,))
        self.assertEqual(term.dtype, np.bool_)
        self.assertEqual(trunc.dtype, np.bool_)

    def test_actions_in_any_form(self):
        env = make()
        env.reset()
        a32 = np.zeros((4, env.action_size), np.float32)
        env.step(a32)
        env.step(a32.astype(np.float64))  # narrowed natively
        env.step(a32.tolist())  # converted
        wide = np.zeros((4, env.action_size * 2), np.float32)
        env.step(wide[:, ::2])  # strided: converted
        env.step(np.zeros((4, env.action_size), np.int32))  # converted
        with self.assertRaises(ValueError):
            env.step(np.zeros(3, np.float32))

    def test_next_step_autoreset(self):
        env = make(max_episode_steps=5)
        env.reset()
        a = np.zeros((4, env.action_size), np.float32)
        for _ in range(5):
            _, _, _, trunc = env.step(a)
        self.assertTrue(trunc.all())
        self.assertTrue((env.episode_steps == 5).all())
        np.testing.assert_array_equal(env.final_observations, env.observations)
        _, r, _, trunc = env.step(a)  # the reset step: ignored action, zero reward
        self.assertFalse(trunc.any())
        self.assertTrue((r == 0).all())
        self.assertTrue((env.episode_steps == 0).all())

    def test_same_step_autoreset(self):
        env = make(max_episode_steps=5, autoreset="same_step")
        self.assertEqual(env.autoreset, "same_step")
        env.reset()
        a = np.zeros((4, env.action_size), np.float32)
        for _ in range(5):
            obs, _, _, trunc = env.step(a)
        self.assertTrue(trunc.all())
        self.assertTrue((env.episode_steps == 0).all())  # already the next episode
        self.assertGreater(np.abs(obs - env.final_observations).max(), 1e-3)
        env.step(a)
        self.assertTrue((env.episode_steps == 1).all())  # no step spent on the reset
        with self.assertRaises(ValueError):
            env.autoreset = "sometimes"

    def test_reset_with_a_seed_is_reproducible(self):
        env = make()
        first = env.reset(seed=3).copy()
        env.step(np.zeros((4, env.action_size), np.float32))
        np.testing.assert_allclose(env.reset(seed=3), first, atol=1e-6)

    def test_views_outlive_the_environment(self):
        env = make()
        obs = env.reset()
        expected = obs.copy()
        env.close()
        del env
        gc.collect()
        np.testing.assert_array_equal(obs, expected)  # still valid memory: the view keeps it alive

    def test_batch_states_and_world(self):
        env = make(num_envs=3, vehicles_per_env=2)
        env.reset()
        states = env.states()
        self.assertEqual(states.shape, (6,))
        self.assertEqual(states.dtype, fsim.vehicle_state_dtype)
        world = env.world
        names = [world.vehicle(int(i)).name for i in env.vehicle_ids]
        self.assertEqual(names, ["env0/0", "env0/1", "env1/0", "env1/1", "env2/0", "env2/1"])
        v = world.vehicle("env1/1")
        self.assertAlmostEqual(states["altitude_msl_m"][3], v.state.altitude_msl_m)

    def test_another_thread_gets_an_error_not_a_race(self):
        env = make(num_envs=8)
        env.reset()
        a = np.zeros((8, env.action_size), np.float32)
        started = threading.Event()
        errors = []

        def many_steps():
            started.set()
            for _ in range(400):
                try:
                    env.step(a)
                except RuntimeError as e:  # the other thread got there first
                    errors.append(str(e))

        worker = threading.Thread(target=many_steps)
        worker.start()
        started.wait()
        deadline = time.perf_counter() + 5.0
        while worker.is_alive() and time.perf_counter() < deadline:
            try:
                env.step(a)  # races the worker's steps
            except RuntimeError as e:
                errors.append(str(e))
                break
        worker.join()
        # The worker's steps ran without the GIL (this thread ran meanwhile)
        # and the overlap was refused rather than let through.
        self.assertTrue(errors, "no step overlapped: the GIL was not released")
        self.assertIn("another thread", errors[0])

    def test_registered_ids(self):
        self.assertIn("altitude_heading_hold", fsim.tasks())
        self.assertIn("state", fsim.observations())
        self.assertIn("velocity", fsim.actions())

    def test_bad_options_fail_loudly(self):
        with self.assertRaises(TypeError):
            make(no_such_option=1)
        with self.assertRaises(fsim.Error):
            make(aircraft="no_such_aircraft")


if __name__ == "__main__":
    unittest.main()
