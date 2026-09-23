# Python SDK

`import fsim` - the same platform as the C++ SDK (libfsim.dll: JSBSim
vehicles, the six control levels, effects, communication, recordings,
scenarios, the batch layer, cameras), from Python, at the speed of the C ABI.

```python
import fsim

world = fsim.World("my-experiment")                      # viewers attach by name
world.set_wind(270, 8, turbulence=0.2)
red = world.create_vehicle("red-1", latitude_deg=37.62, longitude_deg=-122.38,
                           altitude_msl_m=1500, heading_deg=90, airspeed_ms=60)
red.command_velocity(airspeed_ms=65, vertical_speed_ms=2, heading_rad=0.0)
red.add_effect("gaussian_sensor_noise")

s = red.state                                            # live: rewritten in place by every step
for _ in range(3000):
    world.step()
    # s.altitude_msl_m, s.euler_rad[0], s.velocity_ned_ms[2], red.sensed ...
```

Run the examples through the launcher, which uses the interpreter the SDK was
built for with the package on its path:

```
fsim python examples\python\world_tour.py    the object model, per vehicle and batched
fsim python examples\python\train_sb3.py     PPO (Stable-Baselines3) on a 64-aircraft batch
fsim python examples\python\cameras.py       nose and chase cameras, images as numpy arrays
fsim python examples\python\speed.py         what the SDK costs, measured against C
```

## Installing

`fsim dist` writes `dist/python/fsim-<version>-cp311-abi3-win_amd64.whl`:

```
pip install build\ucrt64-release\dist\python\fsim-0.1.0-cp311-abi3-win_amd64.whl
pip install "build\ucrt64-release\dist\python\fsim-0.1.0-cp311-abi3-win_amd64.whl[gym,sb3]"   # with Gymnasium and SB3
```

It installs on any 64-bit CPython from 3.11 on - python.org or conda, the
interpreter PyTorch, Gymnasium and Stable-Baselines3 run on. The wheel
carries the platform's DLLs (`fsim/bin`) and data (`fsim/share/flightsim`:
the JSBSim aircraft, vehicle models); it needs nothing else. numpy is its
only Python dependency.

Without installing, point `PYTHONPATH` at the build tree's staged package,
`build/ucrt64-release/python` - which is what `fsim python` does.

## World and vehicles

| | |
| --- | --- |
| `World(name, **options)` | `dt`, `frame_skip`, `workers`, `pin_workers`, `seed`, `capacity`, `publish`, `publish_interval_s`, `jsbsim_root`, `terrain`, `terrain_url`, `terrain_zoom`, `record_path`, `record_interval_s`: [world.md](world.md). What is not given keeps the platform's default |
| `world.step(n=1)` | the native call itself - nothing in between |
| `world.create_vehicle(name, type="jsbsim:c172x", latitude_deg=, longitude_deg=, altitude_msl_m=, heading_deg=, pitch_deg=, roll_deg=, airspeed_ms=, on_ground=, model=, control_divider=)` | a `Vehicle`; `fsim.Error` if it cannot be loaded |
| `world.vehicle(id or name)`, `world.vehicles()`, `len(world)`, `name in world` | |
| `world.time`, `step_seconds`, `vehicle_steps`, `published` | |
| `world.environment` / `set_environment(**changes)` / `set_wind(...)` | [environment.md](environment.md) |
| `world.add_effect(id, **params)` | on every vehicle, present and future |
| `set_comm_medium`, `create_comm_node`, `send`, `inbox`, `attach_udp_bridge` | [environment.md](environment.md) |
| `World.from_scenario(path or Scenario)`, `world.apply_scenario(...)` | [scenarios.md](scenarios.md) |
| `vehicle.state`, `vehicle.sensed` | a `fsim.VehicleState` (ctypes) over the platform's own snapshot: attribute reads are memory reads, and it is rewritten in place every step - `VehicleState.from_buffer_copy(v.state)` keeps one |
| `vehicle.command_actuator/attitude/acceleration/velocity/position(...)`, `fly_to(lat_deg, lon_deg, alt)`, `command_behavior(id, target=, points=, **params)` | [control.md](control.md); every field defaults as the C++ command struct does, `fsim.HOLD` (NaN) meaning "keep / let the controller decide" |
| `vehicle.active_level`, `behavior_finished`, `use_controller`, `set_controller_parameter`, `get_property` / `set_property`, `add_effect`, `clear_effects`, `send`, `inbox`, `attach_protocol`, `reset(**initial)`, `remove()` | |

## Many vehicles: the batched calls

A per-vehicle call is one native call - about 80 ns for a command, 36 ns for
a state read - so a Python loop over vehicles costs that many calls per
step. The batched calls cost one, however many vehicles fly:

```python
ids = world.ids(vehicles)                                  # a uint32 array: build it once
states = np.empty(len(ids), dtype=fsim.vehicle_state_dtype)
rows = np.tile(fsim.COMMAND_DEFAULTS[fsim.Level.ATTITUDE], (len(ids), 1))

for step in range(steps):
    world.states(ids, out=states)                          # every state, one memcpy each
    rows[:, 0] = policy(states["euler_rad"], states["altitude_msl_m"])   # roll column
    world.command(fsim.Level.ATTITUDE, ids, rows)          # every command, one call
    world.step()
```

`fsim.COMMAND_FIELDS[level]` names the columns. With a uint32 id array, a
C-contiguous float64 value array and an `out` array you keep, the Python
wrapper adds 30-40 ns to the native call; anything else (lists of Vehicles,
float32, strided arrays) is converted first.

## VecEnv: the batch layer

```python
env = fsim.VecEnv(64, task="altitude_heading_hold", action="attitude", seed=1)
obs = env.reset()                                          # (64, 20) float32
for _ in range(steps):
    obs, rewards, terminated, truncated = env.step(policy(obs))   # actions (64, 3) in [-1, 1]
```

Options are those of [vecenv.md](vecenv.md) (task, observation, action,
aircraft, seed, workers, dt, frame_skip, max_episode_steps, the initial
conditions and their jitters, world_name, publish, terrain, scenario_path),
plus `autoreset="next_step"` (Gymnasium) or `"same_step"`
(Stable-Baselines3). `fsim.tasks()`, `fsim.observations()` and
`fsim.actions()` list what can be named.

**The results are views of the environment's own buffers**, made once when
it is created: `env.step` returns the same four arrays every time, rewritten
in place, and `env.final_observations` / `env.episode_steps` are views too.
Copy what you keep past the next step. Actions are read in place when they
are a C-contiguous float32 array, narrowed natively from float64, and
converted from anything else - a torch CPU tensor included (`.cpu()` a CUDA
one first).

`env.world` is the batch's world (vehicles `env<e>/<v>`), and `env.states()`
reads every batch vehicle's full state in one call - for rewards or
observations of your own, computed with numpy over the whole batch.

## Gymnasium and Stable-Baselines3

```python
import fsim.gym                                  # needs gymnasium >= 1.0
envs = fsim.gym.FsimVectorEnv(64, task="altitude_heading_hold", seed=1)
obs, info = envs.reset(seed=1)
obs, rewards, terminations, truncations, info = envs.step(envs.action_space.sample())

import fsim.sb3                                  # needs stable-baselines3 >= 2.0
from stable_baselines3 import PPO
PPO("MlpPolicy", fsim.sb3.FsimVecEnv(64, task="altitude_heading_hold", seed=1)).learn(1_000_000)
```

Each is a vector environment whose sub-environments are the batch's
vehicles (num_envs = M * K), with the reset done by the platform in the
convention the library expects: `FsimVectorEnv` declares
`metadata["autoreset_mode"]` (NEXT_STEP by default; SAME_STEP with
`info["final_obs"]` on request), `FsimVecEnv` uses same-step reset with
`info["terminal_observation"]` and `info["TimeLimit.truncated"]`. Both copy
their results, as those APIs expect (`FsimVectorEnv(copy=False)` returns the
views).

## Cameras

```python
import fsim.vision                               # needs a Vulkan device; `import fsim` does not
vision = fsim.vision.Vision(world, segmentation=True)
nose = vision.add_camera(red, width=640, height=360, fov_deg=70, depth=True, segmentation=True)
world.step(); vision.render()                    # GIL released while it renders
rgb = nose.image()                               # (360, 640, 3) uint8 view, valid until the next render
depth, ids = nose.depth(), nose.segmentation()   # float32 metres, uint16 vehicle ids

batch = fsim.vision.VisionBatch(env, width=84, height=84)   # one camera per batch vehicle
pixels = batch.render()                          # (N, 84, 84, 3) uint8, one render for all of them
```

Options and camera specs are those of [vision.md](vision.md).

## Recordings

```python
rec = fsim.Recording("run.fsrec")                # written by World(record_path=...)
for t, samples in rec:                           # samples: fsim.recorded_sample_dtype, a view
    alt = samples["state"]["altitude_msl_m"]
```

## Performance

The SDK is a C extension module (`fsim._native`) over the C ABI, built for
the least cost per call rather than for convenience - the Python code in
the package sits on top only where it costs nothing in the loop:

- **No copies per step.** Every result is a numpy array or ctypes structure
  that views the platform's memory, made once. The views keep that memory
  alive: an array outliving its environment is still valid.
- **One call per step, not per vehicle.** `VecEnv.step`, `World.states` and
  `World.command` are single native calls whatever the batch size.
- **Fast calls.** `METH_FASTCALL`, positional arguments, the buffer protocol
  for arrays in and out; a native call costs about 10 ns plus its work.
- **The GIL is released** while the platform simulates, renders or loads, so
  other Python threads run meanwhile.

Measured by `python/benchmarks/overhead.py` (`fsim python examples\python\speed.py`),
which runs the same loops through the C ABI and through fsim in fresh
processes, best of five, on a Ryzen 7 9700X with Python 3.13:

| per step | C ABI | Python | difference |
| --- | --- | --- | --- |
| VecEnv, 1 aircraft, frame skip 1 | 6.4 us | 6.6 us | +0.2 us |
| VecEnv, 64 aircraft | 375.3 us | 375.4 us | +0.1 us |
| VecEnv, 256 aircraft | 1695.7 us | 1688.6 us | within noise |
| World, 16 aircraft, step only | 91.8 us | 94.0 us | +2.2 us |
| World, 16 aircraft, per-vehicle commands and states | 92.9 us | 98.0 us | +5.1 us |

Python adds a fraction of a microsecond per native call; the rest of any
difference is the Python in your own loop.

## Threads

Calls that simulate or render release the GIL, so different objects can be
driven from different threads in parallel. One object serves one call at a
time: a call on a World or VecEnv while another thread's call on it is still
running raises `RuntimeError` instead of racing it.

## Logging and errors

A call that fails raises `fsim.Error` with the platform's message. The
platform's own log goes to stderr at "warning" and above; `fsim.set_log_level`
(or `FSIM_LOG_LEVEL`) changes that for every loaded library.

## Building

`FSIM_BUILD_PYTHON` (on by default) builds `fsim._native` - and `fsim._vision`
with the renderer - with the project's own GCC against the stable ABI of a
python.org or conda CPython 3.11+: set `FSIM_PYTHON_EXECUTABLE`, or the `py`
launcher's default is used. Not MSYS2's Python: the libraries RL is done with
are not built for it. One build serves every CPython from 3.11 on. The
package is staged in `build/<preset>/python`, tested by `ctest`
(`python_sdk`), and installed as the `python` component.

What is not in Python: tasks, observations, actions, controllers and effects
implemented as plugins (C++ interfaces, [vecenv.md](vecenv.md),
[control.md](control.md)). From Python, compute rewards and observations
with numpy over `states()`, and command at whichever level suits.
