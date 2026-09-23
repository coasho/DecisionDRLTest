import importlib.util
import unittest

import numpy as np

HAVE_GYM = importlib.util.find_spec("gymnasium") is not None
HAVE_SB3 = HAVE_GYM and importlib.util.find_spec("stable_baselines3") is not None

OPTIONS = dict(publish=False, workers=1, seed=5, max_episode_steps=6)


@unittest.skipUnless(HAVE_GYM, "gymnasium is not installed")
class GymnasiumTest(unittest.TestCase):
    def test_next_step_vector_env(self):
        import fsim.gym
        from gymnasium.vector import AutoresetMode

        envs = fsim.gym.FsimVectorEnv(4, **OPTIONS)
        self.assertEqual(envs.num_envs, 4)
        self.assertEqual(envs.metadata["autoreset_mode"], AutoresetMode.NEXT_STEP)
        self.assertEqual(envs.single_action_space.shape, (4,))
        obs, info = envs.reset(seed=1)
        self.assertEqual(obs.shape, envs.observation_space.shape)
        self.assertTrue(envs.observation_space.contains(obs))
        for _ in range(6):
            obs, rewards, terminated, truncated, info = envs.step(envs.action_space.sample())
        self.assertTrue(truncated.all())
        self.assertIsNot(obs, envs.env.observations)  # copied: ours to keep
        obs, rewards, terminated, truncated, info = envs.step(envs.action_space.sample())
        self.assertFalse(truncated.any())
        self.assertTrue((rewards == 0).all())
        envs.close()

    def test_same_step_vector_env(self):
        import fsim.gym
        from gymnasium.vector import AutoresetMode

        envs = fsim.gym.FsimVectorEnv(3, autoreset_mode=AutoresetMode.SAME_STEP, copy=False, **OPTIONS)
        envs.reset(seed=2)
        for _ in range(6):
            obs, rewards, terminated, truncated, info = envs.step(np.zeros((3, 4), np.float32))
        self.assertTrue(truncated.all())
        self.assertTrue(info["_final_obs"].all())
        self.assertEqual(info["final_obs"][0].shape, (envs.env.observation_size,))
        self.assertGreater(np.abs(info["final_obs"][0] - obs[0]).max(), 1e-3)
        self.assertIs(obs, envs.env.observations)  # copy=False: the view itself


@unittest.skipUnless(HAVE_SB3, "stable-baselines3 is not installed")
class StableBaselines3Test(unittest.TestCase):
    def test_vec_env_protocol(self):
        import fsim.sb3

        env = fsim.sb3.FsimVecEnv(4, **OPTIONS)
        env.seed(3)
        obs = env.reset()
        self.assertEqual(obs.shape, (4, env.env.observation_size))
        for _ in range(6):
            env.step_async(np.zeros((4, 4), np.float32))
            obs, rewards, dones, infos = env.step_wait()
        self.assertTrue(dones.all())
        self.assertTrue(all(i["TimeLimit.truncated"] for i in infos))
        self.assertEqual(infos[0]["terminal_observation"].shape, obs[0].shape)
        self.assertGreater(np.abs(infos[0]["terminal_observation"] - obs[0]).max(), 1e-3)
        self.assertEqual(env.env_is_wrapped(object), [False] * 4)
        self.assertEqual(len(env.get_attr("num_envs")), 4)
        env.close()

    def test_ppo_learns_on_it(self):
        from stable_baselines3 import PPO

        import fsim.sb3

        env = fsim.sb3.FsimVecEnv(4, publish=False, workers=1, seed=1)
        model = PPO("MlpPolicy", env, n_steps=32, batch_size=64, n_epochs=1, seed=1, device="cpu", verbose=0)
        model.learn(total_timesteps=256)
        action, _ = model.predict(env.reset(), deterministic=True)
        self.assertEqual(action.shape, (4, 4))


if __name__ == "__main__":
    unittest.main()
