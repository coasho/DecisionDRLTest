# VecEnv: the batch layer for vectorised RL

`#include <fsim/VecEnv.h>`

`fsim::VecEnv` is a gym-style vectorised environment built entirely on the
object model: it creates M environments x K vehicles in one `World` (named
`worldName`, so the viewer shows the training batch), samples initial
conditions per episode, issues each action as a command at a chosen control
level, and returns observations, rewards and episode flags in flat buffers.

```cpp
fsim::VecEnvOptions opt;
opt.numEnvs = 64; opt.vehiclesPerEnv = 1; opt.seed = 7;
opt.task = "altitude_heading_hold";      // or "level_flight", or a task you register
opt.observation = "state";               // 20 named channels
opt.action = "surfaces";                 // "surfaces" | "attitude" | "acceleration" | "velocity"
opt.worldName = "ppo-run-3";             // what the viewer lists
fsim::VecEnv env(opt);

fsim::StepResult r = env.reset(seed);
std::vector<float> actions(env.numVehicles() * env.actionSize());
for (;;) {
    policy(r.observations, actions);     // [M*K][O] -> [M*K][A], each element in [-1, 1]
    r = env.step(actions);               // rewards[M*K], terminated[M*K], truncated[M*K], finalObservations, episodeSteps[M]
}
```

| Element | Meaning |
| --- | --- |
| Layout | vehicle-major: index `env * K + vehicle`; buffers are owned by the environment and returned as spans |
| Auto-reset | Gymnasium "next-step" semantics: after a terminal step the environment resets on the following `step()`, that action is ignored, reward is 0, flags are clear, the terminal observation is in `finalObservations()` |
| Seeding | `Rng::forVehicle(seed ^ episode, env, vehicle)`: reproducible per episode; `reset(seed)` restarts the episode counter |
| Actions | `"surfaces"`: aileron, elevator, rudder, throttle. `"attitude"`: roll (+-60 deg), pitch (+-25 deg), throttle. `"acceleration"`: load factor (-1..5 g), roll rate (+-3 rad/s), throttle. `"velocity"`: airspeed (20..120 m/s), vertical speed (+-10 m/s), turn rate (+-0.2 rad/s) |
| Observation `"state"` | `alt_msl_km, agl_km, tas_100ms, alpha, beta, roll, pitch, hdg_sin, hdg_cos, p, q, r, vz_down_100ms, ax_g, ay_g, az_g, alt_err_km, hdg_err_sin, hdg_err_cos, throttle` |
| Tasks | `altitude_heading_hold` (targets sampled per episode within `targetAltitudeDeltaM` / `targetHeadingDeltaDeg` of the initial state; shaped reward, -10 on crash), `level_flight` |

The C ABI mirrors it as `fsim_vecenv_*` ([c_abi.md](c_abi.md)). `examples/minimal_trainer` is a complete loop with a PD baseline.

Because the environments live in a `World`, everything in the object model
applies: `env.world()` (C++ internal API) lets you attach effects, set the
environment or inspect vehicles, and `flightsim-viewer.exe` shows the batch.
