# World and vehicles

`#include <fsim/World.h>`

## `fsim::World`

One `World` is one simulation session: a set of vehicles stepped in lockstep,
one environment, one clock, one message fabric, and one shared-memory
publisher that makes the world visible to viewers.

```cpp
struct WorldOptions {
    std::string name = "default";          // viewers attach by this name; use one per experiment
    double dt = 1.0 / 120.0;               // flight-dynamics step (s)
    int frameSkip = 4;                     // FDM steps per World::step() -> 30 Hz world steps
    unsigned workers = 0;                  // simulation threads; 0 = physical cores - 2
    bool pinWorkers = true;                // one worker per physical core (stable throughput)
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;          // vehicle slots visible to viewers
    bool publish = true;                   // false: never visible to viewers
    double publishIntervalSeconds = 1/60.; // wall-clock rate limit of the viewer feed
    std::string jsbsimRoot;                // empty = auto-detect
    bool terrain = false;                  // physics ground from the public elevation tiles the viewer draws
    std::string terrainUrl;                // XYZ template; empty = AWS Terrarium
    unsigned terrainZoom = 12;             // ~38 m/px (14: ~10 m/px, 4x the tiles)
    std::shared_ptr<GroundProvider> ground;// or your own height model (overrides `terrain`)
    std::string recordPath;                // non-empty: record the run for flightsim-viewer --replay
    double recordIntervalSeconds = 0;      // simulation time between frames; 0 = every world step
};
```

With `terrain = true` the ground under every vehicle is the same relief the
viewer renders (AWS Terrarium tiles, sea clamped to 0 m): gear contact, AGL
and `onGround` spawns use it. Tiles are downloaded on first use into
`%LOCALAPPDATA%\flightsim\tilecache` (shared with the viewer), prefetched
around each new vehicle (blocking, a few km) and kept warm around moving
vehicles by background loaders, so training steps rarely wait. Without
network access and an empty cache the ground is flat at 0 m (a warning is
logged per missing tile) - fill the cache beforehand with
`tile_prefetch --lat .. --lon .. --radius-km ..` (elevation and imagery for
the whole level pyramid; the directory can be copied to an offline machine). Implement `fsim::GroundProvider` for a custom
height model (a single virtual, thread-safe call).

With `recordPath` set the world appends every vehicle's state (and each
creation, reset and removal) to a `.fsrec` file as it steps, independent of
whether a viewer is attached: `flightsim-viewer.exe --replay run.fsrec` plays
it back later with pause, single-step and time-factor controls. Recording
is a sequential write of ~150 bytes per vehicle per frame (a 60 s run of
5 vehicles at 30 Hz is ~5 MB); raise `recordIntervalSeconds` for long
trainings.

A recording is also data (`#include <fsim/Recording.h>`):
`fsim::Recording::load(path)` returns the frames - per frame the simulation
time, one `Sample{slot, state, inputs}` per live vehicle and the vehicle
events (creation, reset, control-level change, removal) - and
`rec.track(slot)` collects one vehicle's states and actuator inputs over
time: trajectories of the built-in behaviours for imitation learning,
evaluation logs, regression baselines.

| Call | Meaning |
| --- | --- |
| `Vehicle createVehicle(const VehicleSpec&)` | Load a vehicle; throws `fsim::Error` if the type is unknown, the aircraft fails to load or the name exists |
| `Vehicle vehicle(id)` / `vehicle("name")` / `vehicles()` / `vehicleCount()` | Look-ups; an invalid handle has `valid() == false` |
| `void step(unsigned n = 1)` | Advance every vehicle by `n` world steps (`frameSkip` FDM steps each). Controls, effects and messaging run inside |
| `double time()` / `stepSeconds()` | Simulation seconds since creation; seconds per world step |
| `Environment& environment()` | Real-time environment control ([environment.md](environment.md)) |
| `comm::Network& network()` | The message fabric ([environment.md](environment.md#communication)) |
| `addEffectToAll<E>(args...)` | Every vehicle, present and future, gets its own `E` instance |
| `published()`, `name()`, `vehicleSteps()`, `worldSteps()`, `publishNow()` | Introspection; `publishNow()` pushes the state to viewers immediately |

Several worlds may coexist in one process (each publishes under its own name).
Two processes publishing the same name: the second is not visible until the
first exits (a warning is logged).

## `fsim::VehicleSpec`

```cpp
struct VehicleSpec {
    std::string name;                      // unique in the world; empty = "<aircraft>-<id>"
    std::string type = "jsbsim:c172x";     // "<flight model>:<aircraft>" - any aircraft in the JSBSim tree
    InitialConditions initial;             // latitudeDeg, longitudeDeg, altitudeMslM, headingDeg, pitchDeg, rollDeg, airspeedTrueMs, onGround
    std::string model;                     // optional glTF for the viewer (default: models/<type>.glb, else the platform's aircraft)
    unsigned controlDivider = 1;           // run the control stack every N FDM steps
};
```

`onGround = true` places the vehicle on the terrain (JSBSim ground trim) with
brakes and idle power; otherwise it starts airborne at `airspeedTrueMs` with
engines running.

## `fsim::Vehicle`

A copyable handle (id + world). It stays valid until `remove()`.

| Call | Meaning |
| --- | --- |
| `id()`, `name()`, `type()`, `valid()` | Identity |
| `const VehicleState& state()` | Truth after the last step: ECEF position, quaternion + body->ECEF rotation, geodetic position, altitudes (MSL and AGL), Euler angles, body/NED velocities, rates, accelerations, air data (TAS, CAS, Mach, alpha, beta, load factor), control surface positions, engines, fuel, `onGround`, `diverged` |
| `const effects::SensedState& sensed()` | What the vehicle's sensors report: `state` after the sensor effects, plus `gnssValid`, `airDataValid`, `positionErrorM` |
| `const ControlInputs& inputs()` | The actuator inputs the control cascade produced last step |
| `command(cmd)` | Command at any level; the level of the command becomes the active level ([control.md](control.md)) |
| `controls()`, `activeLevel()`, `use(level, id)` | The control stack: swap a loop, tune gains, inspect derived commands |
| `reset()` / `reset(initial)` | Re-apply initial conditions; controllers and effects reset; viewers see a new "generation" |
| `remove()` | Remove from the world (its slot may be reused by a later vehicle of the same aircraft) |
| `addEffect<E>(args...)` / `addEffect(unique_ptr)` | Attach a disturbance or sensor model ([environment.md](environment.md#effects)) |
| `property("path")` | Raw flight-model property (JSBSim property tree), e.g. `property("propulsion/engine/thrust-lbs").get()` |
| `node()` | This vehicle's communication endpoint |

`state()` and `sensed()` return references into world-owned storage that are
overwritten every step: read what you need before the next `step()`.

## A vehicle's step, in order

For each of the `frameSkip` FDM sub-steps, on the worker thread that owns the vehicle:

1. **Effects** run with the truth state: they may add wind, push forces, write properties and perturb the *sensed* state.
2. **Control cascade** runs from the active level down to the actuators using the sensed state (so noisy sensors affect the built-in loops exactly as they affect your policy).
3. The **flight model** steps.

Then, once per world step on the caller's thread: the **network** delivers messages and the **publisher** copies the world to viewers if the publish interval has elapsed.

## Determinism and threads

Vehicles are partitioned across workers in a fixed order and share nothing:
results are bit-identical for any worker count. Each vehicle has its own
random stream (`Rng`, seeded from `WorldOptions::seed` and the vehicle id) used
by effects and JSBSim's turbulence. Behaviours that look at other vehicles
read the *previous* step's states, so they are deterministic too.
