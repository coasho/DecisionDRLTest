"""What the Python SDK costs, measured against the C ABI doing the same work.

    python overhead.py <path to fsim_python_baseline.exe> [--rounds 5]

Each case runs the same loop through the C ABI (benchmarks/baseline.c) and
through fsim, with the same configuration. Every run is a fresh process -
JSBSim's performance depends on how its many small allocations fall in the
heap, so a process that has already built and freed other worlds is not a
fair comparison with one that has not - and the two alternate for five
rounds, the best of each kept. The physics is identical, so the difference
is what Python adds per step.
"""
import math
import os
import subprocess
import sys
import time

ROUNDS = 5

# (label, C baseline arguments, this script's case arguments)
CASES = [
    ("VecEnv 1 env, frame skip 1, 1 worker", ["vecenv", 1, 1, 1, 4000], ["vecenv", 1, 1, 1, 4000]),
    ("VecEnv 64 envs, frame skip 4", ["vecenv", 64, 4, 0, 400], ["vecenv", 64, 4, 0, 400]),
    ("VecEnv 256 envs, frame skip 4", ["vecenv", 256, 4, 0, 100], ["vecenv", 256, 4, 0, 100]),
    ("World 1 vehicle, step only", ["step", 1, 0, 400], ["world", 1, 0, 400, "step"]),
    ("World 16 vehicles, step only", ["step", 16, 0, 400], ["world", 16, 0, 400, "step"]),
    ("World 1 vehicle, per-vehicle command + state", ["world", 1, 0, 400], ["world", 1, 0, 400, "each"]),
    ("World 1 vehicle, batched command + states", ["world", 1, 0, 400], ["world", 1, 0, 400, "batch"]),
    ("World 16 vehicles, per-vehicle command + state", ["world", 16, 0, 400], ["world", 16, 0, 400, "each"]),
    ("World 16 vehicles, batched command + states", ["world", 16, 0, 400], ["world", 16, 0, 400, "batch"]),
]


def run_case(kind, *args):
    """One measurement, in this (fresh) process: best of three runs, us/step."""
    import numpy as np

    import fsim
    from fsim import HOLD, Level

    fsim.set_log_level("warning")
    if kind == "vecenv":
        envs, frame_skip, workers, steps = map(int, args)
        env = fsim.VecEnv(envs, frame_skip=frame_skip, workers=workers, seed=1, publish=False)
        actions = np.zeros((env.num_vehicles, env.action_size), np.float32)
        actions[:, 3] = 0.6
        step = env.step
        best = math.inf
        for _ in range(3):
            env.reset(1)
            for _ in range(20):
                step(actions)
            t0 = time.perf_counter()
            for _ in range(steps):
                step(actions)
            best = min(best, (time.perf_counter() - t0) / steps)
        return best * 1e6

    vehicles, workers, steps = map(int, args[:3])
    mode = args[3]
    world = fsim.World("bench", workers=workers, pin_workers=False, publish=False)
    vs = [world.create_vehicle("v%d" % i, longitude_deg=-122.375 + 0.01 * i, altitude_msl_m=1500.0, airspeed_ms=60.0)
          for i in range(vehicles)]
    ids = world.ids(vs)
    rows = np.empty((vehicles, 6))
    rows[:] = (0.0, 0.02, HOLD, 0.785, HOLD, 60.0)
    states = np.empty(vehicles, dtype=fsim.vehicle_state_dtype)
    step = world.step
    sink = 0.0
    best = math.inf
    if mode == "step":  # commanded once, then the step alone
        for v in vs:
            v.command_attitude(0.1, 0.02, HOLD, 0.785, HOLD, 60.0)
    for _ in range(3):
        t0 = time.perf_counter()
        for k in range(steps):
            roll = 0.2 * math.sin(0.01 * k)
            if mode == "step":
                pass
            elif mode == "batch":
                world.states(ids, out=states)
                sink += float(states["altitude_msl_m"].sum())
                rows[:, 0] = roll
                world.command(Level.ATTITUDE, ids, rows)
            else:
                for v in vs:
                    sink += v.state.altitude_msl_m
                    v.command_attitude(roll, 0.02, HOLD, 0.785, HOLD, 60.0)
            step()
        best = min(best, (time.perf_counter() - t0) / steps)
    return best * 1e6


def fresh(command):
    return float(subprocess.run(command, capture_output=True, text=True, check=True).stdout.strip())


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--case":
        print("%.3f" % run_case(*sys.argv[2:]))
        return
    baseline = sys.argv[1]
    rounds = int(sys.argv[sys.argv.index("--rounds") + 1]) if "--rounds" in sys.argv else ROUNDS
    import fsim

    print("fsim %s, Python %s; microseconds per step, best of %d alternating fresh processes"
          % (fsim.__version__, sys.version.split()[0], rounds))
    print("%-48s %10s %10s %12s" % ("", "C ABI", "Python", "difference"))
    for label, c_args, py_args in CASES:
        c = py = math.inf
        for _ in range(rounds):
            c = min(c, fresh([baseline, *map(str, c_args)]))
            py = min(py, fresh([sys.executable, os.path.abspath(__file__), "--case", *map(str, py_args)]))
        print("%-48s %10.1f %10.1f %+8.2f (%+.1f%%)" % (label, c, py, py - c, 100.0 * (py - c) / c))


if __name__ == "__main__":
    main()
