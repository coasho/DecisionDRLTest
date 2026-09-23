"""Stable-Baselines3 vector environment over fsim.VecEnv.

``FsimVecEnv`` is an SB3 ``VecEnv`` whose environments are the batch's
vehicles (num_envs = M * K), using the platform's same-step auto-reset: the
step that ends an episode already returns the next one's first observation,
with the last one in ``info["terminal_observation"]`` and time-limit
truncation in ``info["TimeLimit.truncated"]`` - SB3's conventions, without a
Python environment per vehicle.

    import fsim.sb3
    from stable_baselines3 import PPO
    env = fsim.sb3.FsimVecEnv(num_envs=64, task="altitude_heading_hold", seed=1)
    PPO("MlpPolicy", env).learn(1_000_000, callback=fsim.sb3.RolloutThreads())

``RolloutThreads`` gives torch one thread while SB3 collects rollouts, so
torch's threads do not take the cores the platform steps on, and its own
thread count while it trains: 17% more steps/s for PPO end to end.
"""
import gymnasium
import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env.base_vec_env import VecEnv as _Sb3VecEnv

from ._threads import short_openmp_spin
from .vecenv import VecEnv


class RolloutThreads(BaseCallback):
    """torch's CPU threads for SB3's two phases: ``threads`` (1) while it
    collects rollouts - a forward pass over one observation per vehicle, next
    to the platform's workers stepping every vehicle - and torch's own count
    while it trains on the whole buffer. The numbers: docs/sdk/python.md,
    "torch's threads"."""

    def __init__(self, threads=1, verbose=0):
        super().__init__(verbose)
        self.threads = threads
        self._training_threads = None

    def _on_rollout_start(self):
        short_openmp_spin()
        self._training_threads = torch.get_num_threads()
        torch.set_num_threads(self.threads)

    def _on_rollout_end(self):
        torch.set_num_threads(self._training_threads)

    def _on_step(self):
        return True


class FsimVecEnv(_Sb3VecEnv):
    """fsim.VecEnv as a Stable-Baselines3 VecEnv. Keyword arguments are VecEnv's;
    ``num_envs`` and ``vehicles_per_env`` give M and K."""

    def __init__(self, num_envs=1, vehicles_per_env=1, **options):
        options.pop("autoreset", None)
        self.env = VecEnv(num_envs, vehicles_per_env, autoreset="same_step", **options)
        observation_space = gymnasium.spaces.Box(-np.inf, np.inf, (self.env.observation_size,), np.float32)
        action_space = gymnasium.spaces.Box(-1.0, 1.0, (self.env.action_size,), np.float32)
        self.render_mode = None  # the viewer shows the batch; SB3 asks before __init__
        super().__init__(self.env.num_vehicles, observation_space, action_space)
        self._actions = None

    def reset(self):
        seed = self._seeds[0]
        obs = self.env.reset(seed)
        self._reset_seeds()
        self._reset_options()
        return obs.copy()

    def step_async(self, actions):
        self._actions = actions

    def step_wait(self):
        obs, rewards, terminated, truncated = self.env.step(self._actions)
        dones = terminated | truncated
        infos = [{} for _ in range(self.num_envs)]
        if dones.any():
            final = self.env.final_observations
            for i in np.flatnonzero(dones):
                infos[i]["terminal_observation"] = final[i].copy()
                infos[i]["TimeLimit.truncated"] = bool(truncated[i] and not terminated[i])
        return obs.copy(), rewards.copy(), dones, infos

    def close(self):
        self.env.close()

    # The platform's environments are not Python objects: attribute calls
    # reach this adapter, once per index asked for.
    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name)] * len(self._get_indices(indices))

    def set_attr(self, attr_name, value, indices=None):
        setattr(self, attr_name, value)

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        method = getattr(self, method_name)
        return [method(*method_args, **method_kwargs) for _ in self._get_indices(indices)]

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * len(self._get_indices(indices))

    def _get_indices(self, indices):
        if indices is None:
            return range(self.num_envs)
        if isinstance(indices, int):
            return [indices]
        return indices
