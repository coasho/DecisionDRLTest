"""PPO (Stable-Baselines3) on a batch of aircraft learning to hold an altitude
and a heading, each vehicle one of SB3's environments.

    fsim python examples\\python\\train_sb3.py [--steps 200000] [--envs 64]

The batch is published as "sb3-ppo": `fsim viewer` shows it training.
"""
import argparse
import time

from stable_baselines3 import PPO

import fsim.sb3

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--steps", type=int, default=200_000, help="environment steps to train for")
parser.add_argument("--envs", type=int, default=64, help="aircraft in the batch")
parser.add_argument("--save", default="ppo_fsim", help="where the policy is saved")
args = parser.parse_args()

env = fsim.sb3.FsimVecEnv(args.envs, task="altitude_heading_hold", action="attitude", seed=1, world_name="sb3-ppo")
print("%d aircraft; observations %s; actions %s" % (env.num_envs, env.env.observation_names, env.env.action_names))
model = PPO("MlpPolicy", env, n_steps=256, batch_size=4096, learning_rate=3e-4, verbose=1, seed=1, device="cpu")

t0 = time.perf_counter()
model.learn(total_timesteps=args.steps)
wall = time.perf_counter() - t0
steps = model.num_timesteps  # whole rollouts: at least --steps
print("%d steps in %.1f s: %.0f steps/s (policy and training included), %.0f FDM vehicle-steps/s"
      % (steps, wall, steps / wall, env.env.vehicle_steps / wall))
model.save(args.save)
print("policy saved to %s.zip" % args.save)
