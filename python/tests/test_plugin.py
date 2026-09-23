"""The C++ task plugin example (examples/python/plugin/climb_task.cpp)
loaded into Python: it registers, a VecEnv runs it, and its rewards are the
formula it implements - computed here from what the agent observes."""
import os
import unittest

import numpy as np

import fsim

PLUGIN = os.environ.get("FSIM_TEST_PLUGIN")


@unittest.skipUnless(PLUGIN and os.path.isfile(PLUGIN), "the plugin example was not built (FSIM_TEST_PLUGIN)")
class PluginTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.added = fsim.load_plugin(PLUGIN)

    def test_it_registers(self):
        self.assertEqual(self.added, {"tasks": ["climb"], "observations": [], "actions": []})
        self.assertIn("climb", fsim.tasks())
        self.assertEqual(fsim.load_plugin(PLUGIN)["tasks"], [])  # already loaded: nothing new

    def test_rewards_are_its_formula(self):
        env = fsim.VecEnv(8, task="climb", action="attitude", publish=False, workers=1, seed=3,
                          target_altitude_delta_m=200.0)
        alt_err = env.observation_names.index("alt_err_km")
        obs = env.reset()
        climb = obs[:, alt_err] * 1000.0
        self.assertTrue(((climb > 100.0 - 1e-3) & (climb < 400.0 + 1e-3)).all())  # 0.5 to 2 times the delta
        self.assertGreater(np.ptp(climb), 10.0)  # a target of its own for each aircraft

        actions = np.zeros((8, env.action_size), np.float32)
        actions[:, 1] = 0.3  # nose up: the error changes, and so does the reward
        rewards = []
        ended = np.zeros(8, bool)
        for _ in range(40):
            obs, r, terminated, truncated = env.step(actions)
            err_m = obs[:, alt_err].astype(np.float64) * 1000.0
            expected = 1.0 - np.minimum(np.abs(err_m) / 300.0, 1.0)
            live = ~(terminated | truncated | ended)  # no crash penalty, no reset step
            np.testing.assert_allclose(r[live], expected[live], atol=1e-5)
            rewards.append(r.copy())
            ended = terminated | truncated
        self.assertGreater(np.ptp(np.array(rewards)), 0.05)
