# flightsim

A pure C++ flight-simulation platform for AI and reinforcement-learning training:
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics for many vehicles at once,
[VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) for full-Earth visualisation and
vision observations, and a C++ SDK with a stable C ABI as the primary interface.

The architecture, requirements, technology evaluation and roadmap are in
[docs/FlightSim_System_Architecture_and_Design.md](docs/FlightSim_System_Architecture_and_Design.md).

## Status

Milestones **M0 (skeleton), viewer, and M2 (RL environment + SDK) done**:

- `platform`, `core`, `io`, `sim` modules; JSBSim 1.3.1 adapter (`sim::JsbsimModel`) with a
  terrain `GroundProvider` hooked into JSBSim's ground callback.
- `sim::VehiclePool` steps N vehicles in lockstep on a worker pool; trajectories are bit-identical
  for any worker count (tested).
- `flightsim.exe` headless runner / benchmark.
- **Viewer** (`flightsim-viewer.exe`): full-Earth `vsg::TileDatabase` with Esri World Imagery (or
  OSM/Bing/custom XYZ) draped over **real relief** from the free AWS Terrarium elevation tiles, sun +
  ambient lighting from the current UTC time, N vehicles driven from a paced simulation thread through a
  lock-free snapshot buffer with render-side interpolation, chase/orbit/overview cameras, an eye-centred
  gradient sky dome with sun glow and night side, fading flight trails, on-screen vehicle labels, Dear ImGui
  monitor and vehicle list. Vehicles use a real glTF aircraft (the Apache-2.0 "Cesium Air" sample,
  downloaded at configure time) oriented by a per-model `.manifest`; any glTF/OBJ works via `--model`.
- **Terrain physics**: JSBSim's ground callback samples the *same* elevation tiles the renderer shows
  (`world::TileGroundProvider`, LRU + background prefetch); `--on-ground` spawns vehicles parked on the
  terrain via JSBSim's ground trim. Diverged vehicles are reset automatically.

Measured on a 16-thread desktop (Release, 64 × c172x, frame-skip 4):

| workers | vehicle-steps / s |
| ------: | ----------------: |
|       1 |           148,000 |
|       4 |           554,000 |
|       8 |           800,000 |
|      16 |         1,138,000 |

- **RL environment layer** (`env`): `env::VecEnv` runs M environments × K vehicles in lockstep with
  Gymnasium-style vectorised semantics (next-step auto-reset, `terminated`/`truncated`, final observations),
  seeded per (episode, env, vehicle) so trajectories are reproducible for any worker count (tested). Built-in
  tasks `altitude_heading_hold` and `level_flight`, the 20-channel `state` observation and the 4-channel
  `surfaces` action; new tasks/observations/actions plug in through small interfaces.
- **`fsim` SDK** (`libfsim.dll`): the public interface for trainers - `include/fsim/VecEnv.h` (C++) over
  `include/fsim/fsim_c.h` (versioned C ABI: opaque handle, `struct_size`-versioned option/buffer structs,
  library-owned buffers, error codes + `fsim_last_error()`). The C ABI is tested from a plain C99 file.
- `examples/minimal_trainer`: a complete training-loop skeleton (PD baseline / random policy) against the
  SDK, ~170k agent-steps/s on 32 envs (~5700× real time).

Not yet: animated control surfaces, vision observations (offscreen sensor cameras), shared-memory env server, offline tile pyramids (`tools/tile_builder`).

Imagery and elevation come from Esri World Imagery and AWS Terrain Tiles under their respective terms (attribution required); tiles are cached under `%LOCALAPPDATA%lightsim	ilecache`.

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
# Full-Earth viewer: 8 c172x over San Francisco, satellite imagery + terrain relief
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 8
# Yosemite, 6 vehicles at 3200 m
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 6 --lat 37.72 --lon -119.55 --alt 3200 --spread 0.02
# parked at KSFO
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 3 --spread 0.004 --on-ground
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 32 --imagery none --time-factor 4
build/ucrt64-release/bin/flightsim-viewer.exe --help

# Headless benchmark
build/ucrt64-release/bin/flightsim.exe --vehicles 64 --steps 300 --benchmark

# Example trainer over the SDK (32 envs, PD baseline; --random for a random policy)
build/ucrt64-release/bin/minimal_trainer.exe --envs 32 --steps 3000
```

## Using the SDK

```cpp
#include <fsim/VecEnv.h>

fsim::VecEnvOptions opt;
opt.numEnvs = 64;                 // M environments (x vehiclesPerEnv vehicles each)
opt.task = "altitude_heading_hold";
fsim::VecEnv env(opt);            // loads JSBSim aircraft, spins up the worker pool

fsim::StepResult r = env.reset(seed);
std::vector<float> actions(env.numVehicles() * env.actionSize());
for (;;) {
    policy(r.observations, actions); // [M*K][20] -> [M*K][4], all in [-1, 1]
    r = env.step(actions);           // rewards, terminated, truncated, finalObservations
}
```

Link against `fsim` (`libfsim.dll` + `libJSBSim.dll` at runtime); JSBSim's aircraft data is found
automatically next to the executable (`share/jsbsim`) or in the source tree. The same environment is
reachable from C - or any language with a C FFI - through `fsim_c.h`: `fsim_options_init`,
`fsim_vecenv_create`, `fsim_vecenv_step`, `fsim_vecenv_buffers`.

Viewer keys: `space` pause, `.` step, `tab` next vehicle, `c` camera (chase / orbit / overview),
`-`/`=` zoom, `r` reset view, `[`/`]` time factor, `l` vehicle list, `m` monitor, `n` labels, `t` trails, `esc` quit.
Mouse: **left drag orbits** around the selected vehicle, **wheel / right drag** changes distance, middle click resets.
Camera modes: *chase* keeps the view relative to the aircraft's heading, *orbit* keeps it north-referenced, *overview* looks straight down.

## Layout

```
cmake/            warnings, dependencies
docs/             design document
src/platform/     OS isolation (the only module with Win32 includes)
src/core/         logging, registries, module lifecycle, RNG, profiler
src/io/           asset resolution (config, tile cache and recordings later)
src/sim/          FlightModel, JsbsimModel, VehiclePool, GroundProvider
src/env/          Scenario, Task, Observation/Action spaces, VecEnv
src/sdk/          libfsim.dll: C ABI + C++ SDK (public headers in include/fsim)
include/fsim/     public SDK headers (fsim_c.h, VecEnv.h)
examples/         minimal_trainer
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
