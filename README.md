# flightsim

A pure C++ flight-simulation platform for AI and reinforcement-learning training:
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics for many vehicles at once,
[VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) for full-Earth visualisation and
vision observations, and a C++ SDK with a stable C ABI as the primary interface.

The architecture, requirements, technology evaluation and roadmap are in
[docs/FlightSim_System_Architecture_and_Design.md](docs/FlightSim_System_Architecture_and_Design.md).

## Status

Milestone **M0 (skeleton) done, M1 (multi-vehicle FDM) in progress**:

- `platform`, `core`, `io`, `sim` modules; JSBSim 1.3.1 adapter (`sim::JsbsimModel`) with a
  terrain `GroundProvider` hooked into JSBSim's ground callback.
- `sim::VehiclePool` steps N vehicles in lockstep on a worker pool; trajectories are bit-identical
  for any worker count (tested).
- `flightsim.exe` headless runner / benchmark.

Measured on a 16-thread desktop (Release, 64 × c172x, frame-skip 4):

| workers | vehicle-steps / s |
| ------: | ----------------: |
|       1 |           148,000 |
|       4 |           554,000 |
|       8 |           800,000 |
|      16 |         1,138,000 |

Not yet: RL environment layer (`env`), `fsim` SDK / C ABI, viewer, terrain tiles, vision observations.

## Build (Windows x64, MSVC 2022)

Requirements: Visual Studio 2022 Build Tools (C++), CMake ≥ 3.25, Ninja, git.
The headless build needs no Vulkan SDK.

```bash
git clone --recursive https://github.com/coasho/DecisionDRLTest.git
cd DecisionDRLTest
cmake --preset msvc-release
cmake --build --preset msvc-release --parallel
ctest --preset msvc-release
```

Run from a *Developer PowerShell / Command Prompt for VS 2022* (or any shell where `cl` is on `PATH`).

## Run

```bash
build/msvc-release/bin/flightsim.exe --vehicles 64 --steps 300 --benchmark
build/msvc-release/bin/flightsim.exe --aircraft f16 --vehicles 8 --steps 120 --print-every 30
build/msvc-release/bin/flightsim.exe --help
```

The JSBSim data tree (aircraft, engines, systems) is the `third_party/jsbsim` submodule; pass
`--jsbsim-root <dir>` to use another one.

## Layout

```
cmake/            warnings, dependencies
docs/             design document
src/platform/     OS isolation (the only module with Win32 includes)
src/core/         logging, registries, module lifecycle, RNG, profiler
src/io/           asset resolution (config, tile cache and recordings later)
src/sim/          FlightModel, JsbsimModel, VehiclePool, GroundProvider
src/app/          flightsim.exe
tests/            Catch2 unit + JSBSim integration tests
third_party/      JSBSim submodule (LGPL-2.1, built as a DLL)
```

## License

MIT — see [LICENSE](LICENSE). JSBSim is LGPL-2.1 and is linked as a shared library.
