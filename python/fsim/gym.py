"""Gymnasium (1.x) vector environment over fsim.VecEnv.

``FsimVectorEnv`` is a ``gymnasium.vector.VectorEnv`` whose sub-environments
are the batch's vehicles (num_envs = M * K). The auto-reset is done by the
platform, in the mode ``metadata["autoreset_mode"]`` declares: NEXT_STEP by
default, as Gymnasium's own vector envs do; SAME_STEP on request, with the
final observations in ``info["final_obs"]``.

    import fsim.gym
    envs = fsim.gym.FsimVectorEnv(num_envs=64, task="altitude_heading_hold", seed=1)
    obs, info = envs.reset(seed=1)
    obs, rewards, terminations, truncations, info = envs.step(envs.action_space.sample())

With ``copy=True`` (the default, as in Gymnasium's own vector envs) the
arrays returned are yours; ``copy=False`` returns the platform's views, which
the next step rewrites - faster, if you consume them before stepping again.
"""
import gymnasium
import numpy as np
from gymnasium.vector import AutoresetMode, VectorEnv
from gymnasium.vector.utils import batch_space

from .vecenv import VecEnv


class FsimVectorEnv(VectorEnv):
    """fsim.VecEnv as a Gymnasium VectorEnv. Keyword arguments are VecEnv's;
    ``num_envs`` and ``vehicles_per_env`` give M and K."""

    def __init__(self, num_envs=1, vehicles_per_env=1, *, autoreset_mode=AutoresetMode.NEXT_STEP, copy=True, **options):
        mode = AutoresetMode(autoreset_mode)
        if mode == AutoresetMode.DISABLED:
            raise ValueError("fsim environments reset themselves: use NEXT_STEP or SAME_STEP")
        self.env = VecEnv(num_envs, vehicles_per_env,
                          autoreset="same_step" if mode == AutoresetMode.SAME_STEP else "next_step", **options)
        self.metadata = {"autoreset_mode": mode}
        self.num_envs = self.env.num_vehicles
        self.copy = copy
        self.single_observation_space = gymnasium.spaces.Box(-np.inf, np.inf, (self.env.observation_size,), np.float32)
        self.single_action_space = gymnasium.spaces.Box(-1.0, 1.0, (self.env.action_size,), np.float32)
        self.observation_space = batch_space(self.single_observation_space, self.num_envs)
        self.action_space = batch_space(self.single_action_space, self.num_envs)
        self._same_step = mode == AutoresetMode.SAME_STEP

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if isinstance(seed, (list, tuple)):
            seed = seed[0]
        obs = self.env.reset(seed)
        return (obs.copy() if self.copy else obs), {}

    def step(self, actions):
        obs, rewards, terminated, truncated = self.env.step(actions)
        info = {}
        if self._same_step:
            done = terminated | truncated
            if done.any():
                final = np.full(self.num_envs, None, dtype=object)
                for i in np.flatnonzero(done):
                    final[i] = self.env.final_observations[i].copy()
                info["final_obs"], info["_final_obs"] = final, done.copy()
                info["final_info"], info["_final_info"] = {}, done.copy()
        if self.copy:
            return obs.copy(), rewards.copy(), terminated.copy(), truncated.copy(), info
        return obs, rewards, terminated, truncated, info

    def close_extras(self, **kwargs):
        self.env.close()
