# flightsim

A pure C++ flight-simulation platform for AI and reinforcement-learning training:
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics for many vehicles at once,
[VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) for full-Earth visualisation and
vision observations, and a C++ SDK with a stable C ABI as the primary interface.

The architecture, requirements, technology evaluation and roadmap are in
[docs/FlightSim_System_Architecture_and_Design.md](docs/FlightSim_System_Architecture_and_Design.md).

## Status

Milestone **M0 (skeleton) done; viewer (visualisation-first re-plan) in progress**:

- `platform`, `core`, `io`, `sim` modules; JSBSim 1.3.1 adapter (`sim::JsbsimModel`) with a
  terrain `GroundProvider` hooked into JSBSim's ground callback.
- `sim::VehiclePool` steps N vehicles in lockstep on a worker pool; trajectories are bit-identical
  for any worker count (tested).
- `flightsim.exe` headless runner / benchmark.
- **Viewer** (`flightsim-viewer.exe`): full-Earth `vsg::TileDatabase` imagery, N vehicles driven
  from a paced simulation thread through a lock-free snapshot buffer, chase/orbit/overview cameras,
  Dear ImGui monitor and vehicle list.

Measured on a 16-thread desktop (Release, 64 × c172x, frame-skip 4):

| workers | vehicle-steps / s |
| ------: | ----------------: |
|       1 |           148,000 |
|       4 |           554,000 |
|       8 |           800,000 |
|      16 |         1,138,000 |

Not yet: RL environment layer (`env`), `fsim` SDK / C ABI, elevation tiles + ground provider over real terrain, glTF vehicle manifests, vision observations.

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

`ucrt64-headless` builds without Vulkan or the VSG stack. Executables need `ucrt64/bin` on `PATH`
at runtime (the presets set it for tests); a deploy step that copies the runtime DLLs is planned.

## Run

```bash
# Full-Earth viewer: 8 c172x over San Francisco, OpenStreetMap imagery
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 8
build/ucrt64-release/bin/flightsim-viewer.exe --vehicles 32 --imagery none --time-factor 4
build/ucrt64-release/bin/flightsim-viewer.exe --help

# Headless benchmark
build/ucrt64-release/bin/flightsim.exe --vehicles 64 --steps 300 --benchmark
```

Viewer keys: `space` pause, `.` step, `tab` next vehicle, `c` camera (chase / orbit / overview),
`-`/`=` zoom, `[`/`]` time factor, `l` vehicle list, `m` monitor, `esc` quit.

## Layout

```
cmake/            warnings, dependencies
docs/             design document
src/platform/     OS isolation (the only module with Win32 includes)
src/core/         logging, registries, module lifecycle, RNG, profiler
src/io/           asset resolution (config, tile cache and recordings later)
src/sim/          FlightModel, JsbsimModel, VehiclePool, GroundProvider
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
