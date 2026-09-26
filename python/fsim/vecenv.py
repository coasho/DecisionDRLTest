"""The batch layer for vectorised RL (docs/sdk/vecenv.md) as numpy arrays.

``VecEnv`` is fsim_vecenv with nothing added per step: its results are
arrays viewing the environment's own buffers, created once, and ``step``
hands your action array to the platform in place. Those arrays are
therefore *rewritten by the next step*; copy what you keep. The Gymnasium
and Stable-Baselines3 adapters (fsim.gym, fsim.sb3) copy for you, as those
APIs expect.
"""
import numpy as np

from . import _native
from ._convert import as_array
from .world import World

_AUTORESET = {"next_step": _native.AUTORESET_NEXT_STEP, "same_step": _native.AUTORESET_SAME_STEP}


class VecEnv:
    """M environments x K vehicles stepped in lockstep.

    ``VecEnv(num_envs, vehicles_per_env, **options)``, where options are the
    fields of fsim_options (task, observation, action, aircraft, seed,
    workers, dt, frame_skip, max_episode_steps, the initial conditions and
    their jitters, target_altitude_delta_m, target_heading_delta_deg,
    world_name, publish, terrain, scenario_path, jsbsim_root) plus
    ``autoreset``: "next_step" (Gymnasium's default) or "same_step"
    (Stable-Baselines3's), and ``action_ranges``: "fixed" (the action's own)
    or "aircraft" (where the aircraft's profile narrows a command's range - a
    fighter's load factor to its n_min .. n_max - that range). Batch index is
    env * K + vehicle.

        env = fsim.VecEnv(64, task="altitude_heading_hold", action="surfaces", seed=1)
        obs = env.reset()                                    # (64, 20) float32, a view
        obs, rewards, terminated, truncated = env.step(actions)   # actions (64, 4) in [-1, 1]
    """

    def __init__(self, num_envs=1, vehicles_per_env=1, *, autoreset="next_step", action_ranges="fixed", **options):
        for key in list(options):
            if options[key] is None:
                del options[key]
            elif isinstance(options[key], bool):
                options[key] = int(options[key])
        options["num_envs"] = int(num_envs)
        options["vehicles_per_env"] = int(vehicles_per_env)
        self._h = h = _native.VecEnv(options)
        if action_ranges != "fixed":
            h.set_action_ranges(action_ranges)
        self.action_ranges = action_ranges
        self.autoreset = autoreset
        b = h.buffers()
        self.num_envs = b["num_envs"]
        self.vehicles_per_env = b["vehicles_per_env"]
        self.num_vehicles = n = self.num_envs * self.vehicles_per_env
        self.observation_size = o = b["observation_size"]
        self.action_size = b["action_size"]
        self.agent_step_seconds = b["agent_step_seconds"]
        self.observation_names, self.action_names = h.names()
        # Views of the environment's buffers, made once: they never move.
        self.observations = np.frombuffer(b["observations"], dtype=np.float32).reshape(n, o)
        self.rewards = np.frombuffer(b["rewards"], dtype=np.float32)
        self.terminated = np.frombuffer(b["terminated"], dtype=np.bool_)
        self.truncated = np.frombuffer(b["truncated"], dtype=np.bool_)
        self.final_observations = np.frombuffer(b["final_observations"], dtype=np.float32).reshape(n, o)
        self.episode_steps = np.frombuffer(b["episode_steps"], dtype=np.uint32)
        self._result = (self.observations, self.rewards, self.terminated, self.truncated)
        self._step = h.step
        self._world = None
        self._ids = None
        self._world_name = options.get("world_name", "vecenv")

    @property
    def autoreset(self):
        return "same_step" if self._h.autoreset() == _native.AUTORESET_SAME_STEP else "next_step"

    @autoreset.setter
    def autoreset(self, mode):
        if mode not in _AUTORESET:
            raise ValueError("autoreset is 'next_step' or 'same_step', not %r" % (mode,))
        self._h.set_autoreset(_AUTORESET[mode])

    def reset(self, seed=None):
        """Every environment starts a new episode; returns the observations
        (the view). ``seed`` restarts the episode sequence reproducibly."""
        self._h.reset(seed)
        return self.observations

    def step(self, actions):
        """One agent step for the whole batch: ``actions`` is (M*K, A), each
        element in [-1, 1]. Returns (observations, rewards, terminated,
        truncated) - views, rewritten by the next step. A C-contiguous float32
        array is read in place; float64 is narrowed natively; anything else is
        converted first - lists, strided arrays, and torch tensors on any
        device (a CUDA tensor is brought to the CPU, where the platform
        steps)."""
        try:
            self._step(actions)
        except (TypeError, BufferError):
            self._step(as_array(actions, np.float32))
        return self._result

    @property
    def vehicle_steps(self):
        """FDM vehicle-steps so far."""
        return self._h.vehicle_steps()

    @property
    def vehicle_ids(self):
        """The batch vehicles' world ids, in batch order (uint32)."""
        return np.frombuffer(self._h.vehicle_ids(), dtype=np.uint32).copy()

    @property
    def world(self):
        """The batch's world: its vehicles ("env<e>/<v>"), environment,
        effects and network - and states() for your own rewards."""
        if self._world is None:
            self._world = World(self._world_name, _handle=self._h.world())
        return self._world

    def states(self, *, sensed=False, out=None):
        """Every batch vehicle's full state, in batch order, in one call."""
        if self._ids is None:
            self._ids = self.vehicle_ids
        return self.world.states(self._ids, sensed=sensed, out=out)

    def close(self):
        """Release the environment; its memory lives on only while arrays
        viewing it do."""
        self._h = self._step = self._world = None
        self._result = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self):
        return "VecEnv(%d x %d, obs %d, act %d)" % (self.num_envs, self.vehicles_per_env, self.observation_size, self.action_size)
