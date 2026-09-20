# flightsim

A pure C++ flight-simulation platform for AI and reinforcement-learning training:
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics for many vehicles at once,
[VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) for full-Earth visualisation and
vision observations, and a C++ SDK with a stable C ABI as the primary interface.

The architecture, requirements, technology evaluation and roadmap are in
[docs/FlightSim_System_Architecture_and_Design.md](docs/FlightSim_System_Architecture_and_Design.md).

## How it is used

1. **Write a training application against the SDK** ([docs/sdk](docs/sdk/README.md)): create a
   `fsim::World`, create vehicles by name, type and initial state, command them at any level of the
   control stack (actuators, attitude, acceleration, velocity, position, behaviours), control the
   environment in real time, attach effects (sensor noise, gusts, forces, GNSS loss) and let vehicles
   talk over a modelled network. Step the world from your learner's loop.
2. **Start the prebuilt viewer whenever you like.** `flightsim-viewer.exe` discovers the world through
   shared memory and mirrors it - creations, resets, state, control levels, environment - without the
   trainer knowing it exists and without slowing it down (wait-free, rate-limited publish; measured within
   a few percent at 64 vehicles).
3. For vectorised RL, `fsim::VecEnv` gives the same world a gym-style batch interface with actions at
   the control level of your choice.

```cpp
#include <fsim/World.h>

fsim::World world({.name = "dogfight-01"});
world.environment().setWind({.directionDeg = 270, .speedMs = 8, .turbulence = 0.3});

fsim::VehicleSpec spec; spec.name = "red-1"; spec.type = "jsbsim:c172x";
spec.initial.altitudeMslM = 1500; spec.initial.headingDeg = 90; spec.initial.airspeedTrueMs = 60;
fsim::Vehicle red = world.createVehicle(spec);
spec.name = "blue-1"; spec.initial.longitudeDeg -= 0.03;
fsim::Vehicle blue = world.createVehicle(spec);

red.command(fsim::control::AttitudeCommand{.rollRad = 0.4, .pitchRad = 0.05, .airspeedMs = 60});
blue.command(fsim::control::BehaviorCommand{.id = "pursuit", .target = red.id(), .params = {{"range_m", 200}}});
for (int k = 0; k < 3000; ++k) {
    world.step();                           // all vehicles, lockstep, 30 Hz world steps by default
    red.command(policy(red.sensed()));      // any level, any time
}
```

## Status

Milestones **M0 (skeleton), viewer, M2 (RL environment + SDK) and M2b (vehicle SDK + transparent
viewer) done**:

- **Vehicle SDK** (`include/fsim/World.h`, `libfsim.dll`): `World` / `Vehicle` object model over a
  lockstep worker pool; vehicles by name and type; per-vehicle random streams; determinism for any
  worker count (tested).
- **Multi-level control stack** (`fsim/Control.h`): six strictly ordered levels, one command variant,
  controllers that cascade to any lower level, built-in PID loops (attitude, acceleration, velocity,
  position) and behaviours (hold, waypoints, loiter, pursuit, evade, formation, aerobatics), a registry to
  replace any of them, tunable gains, introspection of the derived commands.
- **Real-time environment** (time, atmosphere, wind and turbulence, weather) applied to every JSBSim
  instance per step and published to the viewer.
- **Effects** (`fsim/Effects.h`): a context with wind, force/moment (generic external reaction injected
  into every stock aircraft), property and sensed-state channels; built-in sensor noise, latency,
  constant force, gusts, GNSS degradation.
- **Communication** (`fsim/Comm.h`): nodes, messages, raw/JSON codecs, ideal medium and range/latency/loss
  link model, beacon protocol; all interfaces.
- **Transparent viewer**: `flightsim-viewer.exe` attaches to any published world (`--list`,
  `--world`), waits for one, follows vehicle creation/reset/removal, interpolates in wall time, lights the
  scene from the world's clock, shows names/types/control levels/environment/throughput; `--demo` keeps
  the built-in scenario. Full-Earth satellite imagery over real relief, glTF aircraft, chase/orbit/overview
  mouse cameras, sky, trails, labels, Dear ImGui panels.
- **`fsim::VecEnv`** on the object model: Gymnasium-style batch semantics, actions as `surfaces`,
  `attitude`, `acceleration` or `velocity` commands, visible in the viewer.
- **C ABI** (`fsim/fsim_c.h`): the whole object model and the batch layer, tested from C99.
- `sim::VehiclePool` (one worker per physical core), JSBSim 1.3.1 adapter with terrain ground callback,
  `flightsim.exe` headless benchmark.

Measured on an 8-core desktop (Release): 64 c172x with control cascades, effects and beacons ~760k
vehicle-steps/s (`multi_level_control --extra 59`); VecEnv 32 envs ~180k agent-steps/s (~6000x real time).

Not yet: per-type vehicle models in the viewer, a terrain `GroundProvider` inside the SDK (the viewer's
tiles for physics), animated control surfaces, vision observations, transport bridges for the comm layer,
offline tile pyramids (`tools/tile_builder`).

Imagery and elevation come from Esri World Imagery and AWS Terrain Tiles under their respective terms (attribution required); tiles are cached under `%LOCALAPPDATA%\flightsim\tilecache`.

## Build (Windows x64, MSYS2 UCRT64 / GCC)

The mandated toolchain is MSYS2 **UCRT64** at `D:\ENV\DevLanguages\Cpp\msys2\ucrt64`
(override with `-DFSIM_UCRT64_ROOT=...`). Packages needed in that environment:

```bash
pacman -S mingw-w64-ucrt-x86_64-{gcc,cmake,ninja,vulkan-headers,vulkan-loader,glslang,spirv-tools,assimp,curl}
```

Keep the installation consistent (`pacman -Syu`); MSYS2 does not support partial upgrades.

```bash
git clone --recursive https://github.com/coasho/DecisionDRLTest.git
cd DecisionDRLTest

# 1. VSG stack (vsg, vsgXchange, vsgImGui) -> build/deps-ucrt64/install   (once)
cmake -S deps --preset deps-ucrt64
cd deps && cmake --build --preset deps-ucrt64 && cd ..

# 2. flightsim (viewer + headless)
cmake --preset ucrt64-release
cmake --build --preset ucrt64-release --parallel
ctest --preset ucrt64-release
```

`ucrt64-headless` builds without Vulkan or the VSG stack.

```bash
cmake --build --preset ucrt64-release --target deploy   # copy the 27 runtime DLLs next to the executables
```

After `deploy`, `build/ucrt64-release/bin` runs standalone (no `ucrt64/bin` on `PATH` needed).

## Run

```bash
# 1. a training application (this one: five c172x at different control levels, wind, gusts, beacons)
build/ucrt64-release/bin/multi_level_control.exe --realtime --seconds 600
# 2. the viewer, in another terminal, whenever you like
build/ucrt64-release/bin/flightsim-viewer.exe
build/ucrt64-release/bin/flightsim-viewer.exe --list

# vectorised RL trainer skeleton (32 envs, PD baseline; --random for a random policy); world "vecenv"
build/ucrt64-release/bin/minimal_trainer.exe --envs 32 --steps 3000

# built-in demo scenario on the viewer's own simulation thread
build/ucrt64-release/bin/flightsim-viewer.exe --demo --vehicles 8
build/ucrt64-release/bin/flightsim-viewer.exe --demo --vehicles 6 --lat 37.72 --lon -119.55 --alt 3200 --spread 0.02
build/ucrt64-release/bin/flightsim-viewer.exe --demo --vehicles 3 --spread 0.004 --on-ground
build/ucrt64-release/bin/flightsim-viewer.exe --help

# headless benchmark
build/ucrt64-release/bin/flightsim.exe --vehicles 64 --steps 300 --benchmark
```

Viewer keys: `tab` next vehicle, `c` camera (chase / orbit / overview), `-`/`=` zoom, `r` reset view,
`l` vehicle list, `m` monitor, `n` labels, `t` trails, `esc` quit; in demo mode also `space` pause, `.` step,
`[`/`]` time factor. Mouse, with OpenSceneGraph's trackball feel: **left drag rotates** around the selected
vehicle (release while moving to throw), **middle drag pans**, **right drag zooms** (down = closer), **wheel** zooms;
`r` resets the view.

## Using the SDK

See [docs/sdk](docs/sdk/README.md): [world and vehicles](docs/sdk/world.md), [multi-level
control](docs/sdk/control.md), [environment, effects, communication](docs/sdk/environment.md),
[transparent visualisation](docs/sdk/viewer.md), [VecEnv](docs/sdk/vecenv.md), [C ABI](docs/sdk/c_abi.md).
Link against `fsim` (`libfsim.dll` + `libJSBSim.dll` at runtime); JSBSim's aircraft data is found
automatically next to the executable (`share/jsbsim`) or in the source tree.

## Layout

```
cmake/            warnings, dependencies
docs/             design document
src/platform/     OS isolation (the only module with Win32 includes)
src/core/         logging, registries, module lifecycle, RNG, profiler
src/io/           asset resolution (config, tile cache and recordings later)
src/sim/          FlightModel, JsbsimModel (JSBSim adapter), VehiclePool, GroundProvider
src/control/      multi-level control stack, built-in loops and behaviours, registry
src/effects/      effect pipeline and built-in effects
src/comm/         communication: network, media, codecs, protocols
src/ipc/          shared-memory world segment: publisher, mirror, registry
src/session/      World implementation (vehicles, stepping, environment, publisher)
src/env/          Scenario, Task, Observation/Action spaces, VecEnv (batch layer)
src/sdk/          libfsim.dll: C++ SDK (World, VecEnv) + C ABI
include/fsim/     public SDK headers
examples/         multi_level_control, minimal_trainer
docs/sdk/         SDK guide
src/render/       VSG window, viewer, render graph (viewer builds)
src/world/        Earth tiles (vsg::TileDatabase), vehicle visuals, cameras
src/ui/           Dear ImGui monitor / vehicle list, key bindings
src/app/          flightsim.exe, flightsim-viewer.exe
deps/             superbuild for the VSG stack
tests/            Catch2 unit + JSBSim integration tests
third_party/      JSBSim (LGPL-2.1, DLL), VSG, vsgXchange, vsgImGui submodules
```

## License

MIT — see [LICENSE](LICENSE). JSBSim is LGPL-2.1 and is linked as a shared library.
