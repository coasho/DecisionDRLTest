"""PPO on a task written in C++: examples/python/plugin/climb_task.cpp,
built into climb_task.dll, loaded into Python and named like a built-in.

    fsim python examples\\python\\train_plugin.py [--steps 400000] [--envs 64]

The plugin registers the task "climb": reach a target 150 to 600 m above
where the episode starts, and hold it. The platform steps the aircraft and
the plugin scores them, all in C++; Python only runs the policy. The batch
is published as "climb-ppo": `fsim viewer` shows it learning.
"""
import argparse
import os
import time

import numpy as np
from stable_baselines3 import PPO

import fsim
import fsim.sb3

# Where the build puts the plugin: build/<preset>/examples, beside the
# staged package's build/<preset>/python.
BUILT = os.path.join(os.path.dirname(fsim.__file__), "..", "..", "examples", "climb_task.dll")

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--plugin", default=os.path.normpath(BUILT), help="the plugin DLL")
parser.add_argument("--steps", type=int, default=400_000, help="environment steps to train for")
parser.add_argument("--envs", type=int, default=64, help="aircraft in the batch")
args = parser.parse_args()

added = fsim.load_plugin(args.plugin)
print("loaded %s: task %s" % (args.plugin, ", ".join(added["tasks"])))
print("tasks: %s" % ", ".join(fsim.tasks()))

options = dict(task="climb", action="attitude", target_altitude_delta_m=100.0)
env = fsim.sb3.FsimVecEnv(args.envs, seed=1, world_name="climb-ppo", **options)
alt_err = env.env.observation_names.index("alt_err_km")


def fly(model, steps=1500):
    """The policy on aircraft it has not seen: mean reward per step, and how
    far from their targets they end (m)."""
    test = fsim.sb3.FsimVecEnv(args.envs, seed=99, publish=False, **options)
    obs, total = test.reset(), 0.0
    for _ in range(steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, _, _ = test.step(action)
        total += reward.mean()
    test.close()
    return total / steps, np.abs(obs[:, alt_err]).mean() * 1000.0


model = PPO("MlpPolicy", env, n_steps=256, batch_size=4096, learning_rate=3e-4, verbose=0, seed=1, device="cpu")
print("untrained: reward %.2f per step, %4.0f m from the target" % fly(model))
t0 = time.perf_counter()
model.learn(total_timesteps=args.steps, callback=fsim.sb3.RolloutThreads())
wall = time.perf_counter() - t0
print("trained on %d steps in %.1f s (%.0f steps/s)" % (model.num_timesteps, wall, model.num_timesteps / wall))
print("trained:   reward %.2f per step, %4.0f m from the target" % fly(model))
