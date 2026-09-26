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
opt.actionRanges = "fixed";              // or "aircraft": the ranges the aircraft's profile narrows
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
| Auto-reset | `opt.autoReset`. `NextStep` (default, Gymnasium's): after a terminal step the environment resets on the following `step()`, that action is ignored, reward is 0, flags are clear. `SameStep` (Stable-Baselines3's): the terminal step itself already returns the new episode's first observation, with that step's reward and flags. Either way the terminal observation is in `finalObservations()` |
| Seeding | `Rng::forVehicle(seed ^ episode, env, vehicle)`: reproducible per episode; `reset(seed)` restarts the episode counter |
| Actions | `"surfaces"`: aileron, elevator, rudder, throttle. `"attitude"`: roll (+-60 deg), pitch (+-25 deg), throttle. `"acceleration"`: load factor (-1..5 g), roll rate (+-3 rad/s), throttle. `"velocity"`: airspeed (20..120 m/s), vertical speed (+-10 m/s), turn rate (+-0.2 rad/s) |
| Action ranges | `opt.actionRanges` (1.4). `"fixed"` (default) keeps the ranges above. `"aircraft"` maps an action onto the aircraft's own range wherever its profile narrows one: the F-16C's load factor to -3..9 g and its roll rate to 308 deg/s. Elsewhere the fixed ranges stay (the platform's own bounds - a 180-degree roll - make poor action scales). From C, `fsim_vecenv_set_action_ranges(env, "aircraft")`; from Python, `VecEnv(..., action_ranges="aircraft")`; in a scenario file, `"vecenv": {"action_ranges": "aircraft"}`. Either way the aircraft's envelope protection limits what the loops are asked for ([control.md](control.md#envelope-protection)) |
| Observation `"state"` | `alt_msl_km, agl_km, tas_100ms, alpha, beta, roll, pitch, hdg_sin, hdg_cos, p, q, r, vz_down_100ms, ax_g, ay_g, az_g, alt_err_km, hdg_err_sin, hdg_err_cos, throttle` |
| Tasks | `altitude_heading_hold` (targets sampled per episode within `targetAltitudeDeltaM` / `targetHeadingDeltaDeg` of the initial state; shaped reward, -10 on crash), `level_flight` |

## Your own task, observation or action

`#include <fsim/VecEnvPlugins.h>`

The three things that make a batch an *experiment* - what it is rewarded for,
what the agent sees and what an action means - are small interfaces. Implement
one in your own code, register it under an id, and name that id in the
options: no platform rebuild, and the built-ins are implemented against
exactly the same interfaces.

| Interface | Implements | Registered with |
| --- | --- | --- |
| `fsim::Task` | `reset()` picks the episode's targets, `evaluate()` returns reward and termination for one vehicle | `registerTask(id, [](const TaskParams&) { ... })` |
| `fsim::ObservationBuilder` | `size()`, `names()`, `build()` - the vector the agent sees, roughly normalised to [-1, 1] | `registerObservation(id, [] { ... })` |
| `fsim::ActionMapper` | `size()`, `names()`, `level()`, `map()` - an action vector (each element in [-1, 1]) as a command at one control level; optionally `useRanges(capabilities)`, the aircraft's ranges for `"aircraft"` action ranges | `registerAction(id, [] { ... })` |

```cpp
class Climb final : public fsim::Task {
public:
    explicit Climb(const fsim::TaskParams& p) : ceiling_(p.targetAltitudeDeltaM) {}
    std::string_view name() const noexcept override { return "climb"; }

    void reset(std::size_t, const fsim::VehicleState& initial, fsim::Rng& rng, fsim::TaskState& t) override {
        t.targetAltitudeM = initial.altitudeMslM + rng.uniform(100.0, ceiling_);
        t.custom[0] = initial.altitudeMslM;              // yours to use
    }
    void evaluate(std::size_t, const fsim::VehicleState& s, fsim::TaskState& t, double& reward, bool& terminated) override {
        reward = (s.altitudeMslM - t.custom[0]) / 100.0;
        terminated = s.diverged || s.altitudeAglM < 30.0;
        if (terminated && !s.diverged) reward -= 10.0;
        t.lastReward = reward;
    }
private:
    double ceiling_;
};

fsim::registerTask("climb", [](const fsim::TaskParams& p) { return std::make_unique<Climb>(p); });

fsim::VecEnvOptions opt;
opt.task = "climb";                                      // and opt.observation / opt.action likewise
fsim::VecEnv env(opt);
```

- `TaskState` is the per-vehicle scratch the task keeps across an episode: the
  targets, `lastReward`, and `custom[8]` for anything else. It is **cleared
  before every `reset()`**, so episodes are independent, and the observation
  builder is handed the same struct - which is how the agent gets to see what
  it is being asked to do.
- `TaskParams` is what the scenario fixed for the batch:
  `targetAltitudeDeltaM`, `targetHeadingDeltaDeg`, `maxEpisodeSteps`,
  `aircraft` and `agentStepSeconds` (`dt * frameSkip`).
- An `ActionMapper` returns a `control::Command` at the `level()` it declares,
  so anything above the actuators is flown by the built-in loops
  ([control.md](control.md)); registering one is how a batch gets an action
  space the four built-ins do not cover.
- `taskIds()`, `observationIds()` and `actionIds()` list what is registered,
  built-ins included. Registration is process-wide, takes effect for
  environments created afterwards, and registering an existing id replaces it
  - so you can substitute your own `"state"` observation. An id nobody
  registered makes the `VecEnv` constructor throw, naming the ids that exist.
- The environment calls these on the caller's thread, once per vehicle per
  step, in vehicle order.

`tests/test_sdk.cpp` registers all three and drives a batch with them.

For a Python trainer, the same classes go into a DLL that registers them from
a static initialiser; `fsim.load_plugin` loads it, and the ids are named as
here. [examples/python/plugin/climb_task.cpp](../../examples/python/plugin/climb_task.cpp)
is one, trained with PPO by `examples/python/train_plugin.py`
([python.md](python.md#your-own-task-in-c-trained-from-python)).

A scenario file can define the batch instead of code: `fsim::vecEnvOptions(fsim::loadScenario(path))` ([scenarios.md](scenarios.md#vecenv-from-a-scenario)); its environment and world-wide effects then apply to the batch's world (`VecEnvOptions::scenarioPath`).

The C ABI mirrors it as `fsim_vecenv_*` ([c_abi.md](c_abi.md)). `examples/minimal_trainer` is a complete loop with a PD baseline; `examples/ppo_trainer` is a full PPO (GAE, clipped objective, running observation normalisation, truncation bootstrapping) with a dependency-free MLP that learns the `altitude_heading_hold` task at the attitude level in about two minutes and beats the PD baseline - the loop to copy when plugging in LibTorch or any other learner.

Because the environments live in a `World`, everything in the object model
applies: `env.world()` returns it as a `fsim::World&` (vehicles named
`env<e>/<v>` in batch order) so you can attach effects, set the environment,
inspect vehicles or mount cameras ([vision.md](vision.md)), and
`flightsim-viewer.exe` shows the batch.
