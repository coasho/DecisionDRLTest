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
fsim python examples\python\train_plugin.py  PPO on a task written in C++, loaded as a plugin DLL
fsim python examples\python\cameras.py       nose and chase cameras, images as numpy arrays
fsim python examples\python\control_surfaces.py   two hangar designs flying a slalom, for the viewer
fsim python examples\python\fighters.py      the fourteen fighters flying in formation, for the viewer
fsim python examples\python\vehicle_interface.py   routes, a hold and curves on wings, a fighter and two rotorcraft in wind, for the viewer
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
| `vehicle.profile_value(path)`, `profile_section(name)` | the aircraft's profile by path ("envelope/clean/n_max", NaN if unknown) and a section's (version, provenance) ([control.md](control.md#the-aircrafts-profile)) |
| `vehicle.capabilities()`, `capability_status(id)` | what the vehicle offers: `fsim.Capability(id, version, kind, interactions, level, axes, terminating, needs_target, behavior, parameters, axis_groups, mode)` - `mode` the A-GRA flight capability type it is ("formation", "loiter", ... or "none") - with `fsim.Parameter(name, unit, min, max, default, optional, supported)` - not `supported`: the aircraft has nothing the field moves, and a command that sets it is refused; ([control.md](control.md#capabilities-and-activities)) |
| `vehicle.submit(level, **fields, source=, axes=, range=, min_version=)`, `submit_behavior(id, target=, points=, **params)`, `submit_support(kind, **fields)` ("gear", "flaps", "wheel_brakes", "speedbrake", "pitch_trim", "engines"; `fsim.SUPPORT_FIELDS`) | NEW: an `fsim.Activity`, or `fsim.Rejected` (an `fsim.Error` with `.reason` - "authority_held", "out_of_range", "invalid_axes", ... - and `.other`); `source` is `fsim.Source.POLICY` / `AUTOPILOT` / `OVERRIDE`, `axes` an `fsim.Axis` (`LATERAL`, `PITCH`, `THRUST`, their union; None = every axis), `range` `fsim.RangePolicy.CLAMP` / `REJECT` / `NONE`; "engines" is a throttle per engine (`throttle_1` ..., `fsim.HOLD` keeps one) |
| `vehicle.submit_hsa(heading_rad= or course_rad=, speed=, speed_reference=, altitude_m=, altitude_reference=)` | the HSA/CSA mode ([control.md](control.md#hsacsa-a-heading-or-a-course-a-speed-an-altitude)): an `Activity` whose `update(**fields)` changes only what it gives; references as `fsim.SpeedReference` / `fsim.AltitudeReference` or their names ("mach", "ground_speed", "above_ground", ...); `fsim.MODE_FIELDS["hsa"]` |
| `vehicle.submit_route(waypoints, projection=, repeat=, end=, start=)` | waypoint following ([control.md](control.md#routes-waypoint-following)): `fsim.Waypoint(latitude_rad, longitude_rad, altitude_m=, altitude_reference=, speed=, speed_reference=, turn=, max_bank_rad=, climb_rate_ms=, id=)` (or a dict of its fields, or a row in its order; HOLD continues the previous point's), `fsim.Projection`, `fsim.EndBehavior`, `fsim.TurnType` or their names ("rhumb", "loiter", "fly_over"); an `Activity` whose `update_route(waypoints=None, **options)` replaces it and `update(**options)` changes its options alone. `examples/python/vehicle_interface.py` flies routes for the viewer |
| `vehicle.submit_pattern(pattern=, latitude_rad=, longitude_rad=, altitude_m=, radius_m=, clockwise=, course_rad=, leg_m=, speed=, speed_reference=, duration_s=)` | loiter patterns ([control.md](control.md#loiter-patterns-orbit-racetrack-figure-eight-hold)): `fsim.PatternKind` or its name ("orbit", "racetrack", "figure_eight", "hold"); what it leaves out takes its default (an orbit here, a hold's rate-one turns and minute-long legs); an `Activity` whose `update(**fields)` changes only what it gives; `fsim.MODE_FIELDS["pattern"]` |
| `vehicle.submit_curve(segments, latitude_rad=, longitude_rad=, altitude_m=, speed_min_ms=, speed_max_ms=, duration_s=, end=)` | curve following ([control.md](control.md#curves-curve-following)): `fsim.BezierSegment(north, east, down)`, six control points each in metres from the reference (or a dict of its fields, or a triple), 1 to 10; an `Activity` whose `append(segments)` adds segments after its end while it flies, `update_curve(segments=None, **options)` replaces it, and `update(**options)` changes how it is flown; `fsim.Rejected.index` names a segment refused, `.section` where one is too tight; `fsim.MODE_FIELDS["curve"]` |
| `vehicle.performance`, `vehicle.control_revision`; `vehicle.set_control_mode("granted")`, `request_control(capability)`, `release_control`, `revoke_control(capability, reason="revoked")`, `set_allowed(capability, allowed)`, `control_status(capability)`, `set_availability(capability, availability, reason="restricted")` | what the vehicle can do (`fsim.Performance`, NaN where unknown; computed afresh when its loops change) and grants over the priorities ([control.md](control.md#grants-who-may-command-a-vehicle)): under `fsim.ControlMode.GRANTED` a policy's NEW needs a grant, else `fsim.Rejected("not_granted")`; `request_control` raises `fsim.Rejected` ("not_allowed", "restricted", ...) when refused; `control_status` is `fsim.ControlStatus(allowed, granted)`; `control_revision` counts every change to them. An `Activity` remembers the `source` it was submitted with and declares it in its `update`, `update_route`, `update_curve`, `append` and `cancel`: under GRANTED a policy's handle cannot change or end what the platform's own sources fly ("authority_held"). `revoke_control` takes "revoked", "collision_avoidance" or "restricted"; `set_availability` "restricted", "collision_avoidance" or "unavailable", and `associated=` and `next_available_s=` |
| `activity.setpoint()`, `activity.end_points(max=16)`, `vehicle.commanded`; `controller=` on every submit, `vehicle.request_control(capability, controller=0)`, `release_control(capability, controller=0)` | the reports ([control.md](control.md#reports-what-an-activity-flies-and-where-to)): what a live activity flies as the `fsim.BatchCommand` that would command it (a curve's: its flyout curve, `fsim.agra.flyout_curve` A-GRA's NURBS form), where it flies to (`fsim.EndPoint`: `fsim.EndPointKind`, point, altitude and reference, turn, id, index; `fsim.agra.end_point`), and `vehicle.commanded`'s acceleration north, east and down and altitude in its reference (`fsim.agra.altitude_reference`). Named controllers ([control.md](control.md#named-controllers)): a grant is one controller's (`fsim.ControlStatus.holder`); `request_control` raises `fsim.Rejected("authority_held")` while another holds it, `release_control` `("not_granted")` for another's; an `Activity` keeps its `controller` and declares it in its update, cancel and activity commands; `ActivityInfo.controller` |
| `vehicle.store_task(task_id, fsim.BatchCommand(...), attempts=1, interval_s=None)`, `command_task(task_id, **options)`, `cancel_task`, `remove_task`, `task_status`, `tasks()` | flight tasks ([control.md](control.md#flight-tasks-and-suggestions)): a command kept by id and flown on a task command (an `Activity`, the task among its `info.trace`), as often as `attempts` says - its runs one activity's (`ActivityInfo.run`, `.runs`); `task_status` a `fsim.TaskStatus` (`fsim.TaskState`, reason, suggested, activity, run, runs, percent, start and end, command id; `fsim.agra.task_state`). A refusal Clamp would fly names a suggestion (`fsim.Rejected.suggestion`), a task `command_task` flies; so does a waiting activity that failed as it would start (`ActivityInfo.suggestion`) |
| `activity.disable()`, `.enable()`, `.reset()`, `.delete()`, `.change_rank(rank)`, `.unassign()` | activity commands ([control.md](control.md#activity-commands)): disabled, an activity is kept (`fsim.ActivityState.DISABLED`, still `live`) and flies nothing until enabled; deleted, it ends (`DELETED`). Each raises `fsim.Rejected` - `not_interactive` where it was submitted with `interactive=False`, `activity_ended` once it has ended - and declares the source it was submitted with |
| `rank=`, `interrupt=`, `precedence_override=`, `window=`, `override_rejection=` on every `submit*`; `vehicle.set_capability_precedence(capability, p)`, `capability_precedence(capability)` | ranks, queues and time windows ([control.md](control.md#ranks-queues-and-time-windows)): `rank` a `fsim.Rank(priority, precedence)` or pair, lower first; `interrupt=False` waits for what flies; `window` a `fsim.TimeWindow(start_not_before, start_not_after, end_not_before, end_not_after, criticality)` (simulation seconds, `World.time`; NaN no bound; `fsim.TimeCriticality` or its name) or a dict of its fields. A NEW accepted to wait is an `Activity` with `deferred` True (`Validation.deferred` likewise); its `ActivityInfo` has `waiting` (`fsim.ActivityWait`), `waiting_for`, `basis` (`fsim.ActivityBasis`), `rank`, `precedence`, `interrupt`, `window`. `fsim.agra.command_options(ranking, temporal)` turns A-GRA's Ranking and TemporalConstraints into these keywords (InterruptOtherActivities left out: False); `agra.activity_state(info, now)` names a queued one ACTIVE_FULLY_CONSTRAINED, `agra.activity_basis(info)` its basis |
| `command_id=`, `trace=`, `interactive=`, `validate_only=` on every `submit*`; `vehicle.submit_batch([fsim.BatchCommand(...)])`; `world.last_command_details()` | the command envelope ([control.md](control.md#the-command-envelope-ids-findings-validation-batches)): the caller's id, echoed (`Activity.command_id`, `fsim.Rejected.command_id`) and kept (`ActivityInfo.command_id`, `.trace` as `(fsim.RequirementKind, id)` pairs, `.interactive`); `validate_only=True` answers as a NEW would with a `fsim.Validation(valid, reason, clamped, command_id, findings, adjustments)` and flies nothing; a refusal's `fsim.Rejected.findings` names every reason (`fsim.Finding`), its `description` the first in words, and a clamp's `world.last_command_details()` every value held (`fsim.Adjustment`); `submit_batch` makes several NEWs at once, each answered on its own (an `Activity`, `Validation` or `Rejected`, returned). `fsim.agra.validation_results(answer)` and `route_validation_error(finding)` give A-GRA's names |
| `vehicle.support(feature)`, `vehicle.support_table()`, `fsim.SUPPORT_FEATURES`; `vehicle.availability(capability)`, `vehicle.capability_limits(capability)` | whether the aircraft can do a public feature at all (`fsim.SupportInfo`: `support` a `fsim.Support` - SUPPORTED, PARTIAL, NOT_IMPLEMENTED or NOT_SUPPORTED - `rules` ("R1" ...), `stage`, `capability`, `missing`, `evidence`), and whether a capability can be commanded now (`fsim.AvailabilityInfo`: `availability`, `reason`, every one of `reasons`, `description`, `associated`, `next_available_s`, and `ranges` a placard narrows now as (name, min, max)). A NEW for what the vehicle does not offer raises `fsim.Rejected("not_supported")` or `("not_implemented")`, a policy's guidance on the ground `("on_ground")` ([control.md](control.md#support-and-availability-what-a-vehicle-can-do-at-all-and-now)). `Capability.accepted` and `.superseded` say how a capability is controlled and what supersedes a platform behaviour; `fsim.agra` maps the new reasons |
| `activity.progress`, `vehicle.commanded`, `fsim.Rejected.index` / `.constraint` / `.section`, `fsim.agra` | the Vehicle Interface ([control.md](control.md#the-vehicle-interface)): a guidance activity's `fsim.ActivityProgress` (segment of segments, laps, percent, distance and time to go, cross-track, the course, heading, altitude and speed it commands); the `fsim.CommandedState` of the cascade's last update (with, since FA-2e, the acceleration it commands and the altitude in its reference); what a rejection was about (the field, route point or curve segment; the limit its value broke, "max_orientation", ...); and A-GRA's names for them (`fsim.agra.activity_state(info)`, `cannot_comply(reason)`, `performance_constraint(c)`, `flight_capabilities(vehicle)`) |
| `vehicle.set_vehicle_default("neutral" / "hold")`, `vehicle.vehicle_default` | what flies the axes nobody owns: idle, as always, or a hold of the heading, airspeed and height each had when it was let go - a rotorcraft's heading, height and velocity over the ground (`fsim.VehicleDefault`) |
| Rotorcraft ([control.md](control.md#rotorcraft)) | `submit` and `update` take the rotorcraft's fields by name - `pitch_rate_rad_s`, `yaw_rate_rad_s` (acceleration), `north_ms`, `east_ms` (velocity), `heading_rad` (position) - or rows as wide as `fsim.SETPOINT_FIELDS[level]` (`COMMAND_FIELDS`' rows still work); `world.update` sends either width. A multirotor's motors: `submit_support("engines", t1, t2, t3, t4)`, each motor's thrust (0..1); `vehicle.state.engine_rpm` is each rotor's speed, which the viewer turns its propeller at. A rotorcraft's flight commands own `fsim.Axis.CYCLIC` (roll with pitch), `YAW` and `THRUST` apart |
| `vehicle.set_protection("limit" / "report" / "off")`, `vehicle.protection`, `vehicle.envelope()` | envelope protection ([control.md](control.md#envelope-protection)): `fsim.ProtectionMode`, limit by default with an envelope; `envelope()` is an `fsim.Envelope(mode, limits)`, `limits` a `fsim.LimitStatus(limited_updates, exceeded_updates, exceeded_s, worst_excess)` per name in `fsim.LIMITS`, since the last call |
| `activity.update(**fields)`, `activity.cancel()`, `activity.state` / `live` / `info`; `world.activity(id)`, `vehicle.activities()`; `world.update(activities, rows)` | UPDATE (the per-step path; returns whether a value was clamped), CANCEL, and the records (`fsim.ActivityInfo`); `world.update` takes a uint64 array of activity ids and a float64 row each, in one call. The `command_*` calls refuse what the vehicle refuses (`fsim.Error`): a behaviour nobody registered, an axis a higher source holds |
| `vehicle.active_level`, `behavior_finished`, `use_controller`, `set_controller_parameter`, `controller_parameter(level, name)`, `get_property` / `set_property`, `add_effect`, `clear_effects`, `send`, `inbox`, `attach_protocol`, `reset(**initial)`, `remove()` | |

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
Copy what you keep past the next step.

Actions are read in place when they are a C-contiguous float32 array, and
narrowed natively from float64. Anything else is converted first: lists,
strided arrays, and torch tensors as a policy returns them - on the GPU, in
half precision, with a gradient attached - with no `.detach().cpu()` of your
own. A CPU float32 tensor is read without a copy, for 2.4 us more per step
than a numpy array (the conversion); a CUDA tensor is copied to the host once, which the
platform needs whatever the API, since it simulates on the CPU.
`World.command` takes tensors the same way. torch is not imported for this:
it stays optional.

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
model = PPO("MlpPolicy", fsim.sb3.FsimVecEnv(64, task="altitude_heading_hold", seed=1))
model.learn(1_000_000, callback=fsim.sb3.RolloutThreads())   # 17% faster: see "torch's threads"
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

## torch's threads

A VecEnv steps its aircraft on worker threads, one per core. torch runs its
CPU ops on an OpenMP team as large as the machine, and the OpenMP it ships
on Windows (Intel's) keeps that team spinning for 200 ms after every op. So
while experience is collected - one small forward pass per step - torch's
idle threads spin on the cores the aircraft are stepped on. Training on a
whole buffer is the other way round: there the threads pay. PPO as
`train_sb3.py` runs it (64 aircraft, 213k steps, an 8-core Ryzen 7 9700X;
fresh processes, best of three):

| torch threads | collecting | training | end to end |
| --- | --- | --- | --- |
| torch's default (8) | 83.6k steps/s | 1.64 s | 50.9k steps/s |
| 1 | 109.6k steps/s | 3.09 s | 42.3k steps/s |
| 2 | 104.4k steps/s | 2.06 s | 51.9k steps/s |
| 1 collecting, 8 training: `fsim.sb3.RolloutThreads()` | 110.6k steps/s | 1.64 s | **59.7k steps/s** |

`fsim.sb3.RolloutThreads(threads=1)` is the SB3 callback of the last row:
`threads` torch threads from the start of each rollout to its end, torch's
own count while it trains. It also shortens the OpenMP spin to 1 ms -
without that, the team that trained spins on through most of the next
rollout and the switch gains nothing (training measured no slower for it).
In a loop of your own, `fsim.torch_threads` does the same around a block:

```python
for update in range(updates):
    with fsim.torch_threads(1):                               # collecting
        for t in range(n_steps):
            with torch.no_grad():
                actions = policy(torch.tensor(obs))           # a copy: obs is rewritten by the next step
            obs, rewards, terminated, truncated = env.step(actions)
    train(policy, buffer)                                     # torch's own thread count
```

`KMP_BLOCKTIME=1` in the environment, set before torch is imported, is the
same 1 ms spin for every thread.

## Python threads

Calls that simulate or render release the GIL, so different objects can be
driven from different threads in parallel. One object serves one call at a
time: a call on a World or VecEnv while another thread's call on it is still
running raises `RuntimeError` instead of racing it.

## Logging and errors

A call that fails raises `fsim.Error` with the platform's message. The
platform's own log goes to stderr at "warning" and above; `fsim.set_log_level`
(or `FSIM_LOG_LEVEL`) changes that for every loaded library.

## Your own task, in C++, trained from Python

Tasks, observations, actions and controllers of your own are C++ - the
interfaces of [vecenv.md](vecenv.md#your-own-task-observation-or-action)
and [control.md](control.md) - in a DLL that registers them when it is
loaded; no platform rebuild. `fsim.load_plugin` loads one into Python, and
its ids are then named like the built-ins:

```python
fsim.load_plugin(r"build\ucrt64-release\examples\climb_task.dll")   # {"tasks": ["climb"], ...}
env = fsim.sb3.FsimVecEnv(64, task="climb", action="attitude", seed=1)
```

[examples/python/plugin/climb_task.cpp](../../examples/python/plugin/climb_task.cpp)
is one: a task that has each aircraft climb 0.5 to 2 times
`target_altitude_delta_m` and hold it, registered by a static initialiser
(`fsim::registerTask`). `fsim python examples\python\train_plugin.py`
trains PPO on it - on unseen aircraft, 400k steps (7 s) take the mean
reward from 0.52 to 0.83 per step and the distance to the target after 50 s
from 187 m to 39 m.

The plugin runs where the built-in tasks run - inside the step, in C++ - and
costs what they cost. Build it with the platform's toolchain (MSYS2 UCRT64
GCC) against the SDK of the same version (in-tree like the example, or
`find_package(fsim CONFIG)` of `dist/sdk`): the interface is C++, and the DLL
binds to the libfsim.dll the Python package has already loaded. Effects are
the exception - a C++ program attaches them as objects, there is no registry
to name one in - so they are not offered to Python this way.

## Building

`FSIM_BUILD_PYTHON` (on by default) builds `fsim._native` - and `fsim._vision`
with the renderer - with the project's own GCC against the stable ABI of a
python.org or conda CPython 3.11+: set `FSIM_PYTHON_EXECUTABLE`, or the `py`
launcher's default is used. Not MSYS2's Python: the libraries RL is done with
are not built for it. One build serves every CPython from 3.11 on. The
package is staged in `build/<preset>/python`, tested by `ctest`
(`python_sdk`), and installed as the `python` component.
