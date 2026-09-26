# Flight Simulator (C++ / VSG / JSBSim) — System Architecture & Design

2026-09-19 · @Someone

## 1. Purpose and scope

This document defines the architecture of a pure C++ flight-simulation platform for AI and reinforcement-learning (RL) training: JSBSim computes flight dynamics for many vehicles at once, VulkanSceneGraph (VSG) renders a full-Earth scene for visualisation and for vision-based observations, and a C++ SDK with a stable C ABI is the primary interface. It is the reference for structure, boundaries and major technical decisions; implementation details live in code and per-module design notes.

In scope: the application skeleton, module boundaries, runtime/threading model, integration of VSG and JSBSim, the RL environment API (C++ and C ABI), the debug viewer, VSG-native full-Earth terrain, build and packaging on Windows, and the extension mechanism. Out of scope for this revision: human-piloted operation (no flight-stick input, cockpit or instrument panels), networking/multiplayer, RL algorithms themselves, and certification-grade fidelity requirements. Project owner decisions of 2026-09-19 that shaped this revision: RL training is the primary purpose, full-Earth visualisation, no human pilot for now, powerful training hardware, open-source licence, Windows only, multiple vehicles, JSBSim used as-is, VSG mandatory, no Cesium, no Python. Owner workflow statement of 2026-09-20 (section 9): researchers build training applications against the vehicle SDK; a prebuilt viewer mirrors their world through shared memory transparently and without affecting training throughput; vehicles are created by name/type/initial state and controlled through a multi-level control stack; the environment is controllable in real time; effects and communication are abstract, extensible interfaces. Later owner requests added: an offline map package that ships with the viewer (2026-09-22); a Python SDK over the C ABI (2026-09-23), which relaxes "no Python" for trainers while the platform itself still embeds no language runtime (section 2); hangar (2026-09-22/23), a tool that turns an aircraft design into a flight-tested JSBSim aircraft with a 3D model, and a library of fighters and support aircraft made with it (2026-09-23 to 25); and engine exhaust and airflow effects in the viewer (2026-09-25).

Reading guide: sections 2–4 fix requirements and technology choices; sections 5–10 describe the design; sections 11–13 cover build, performance and cross-cutting concerns; sections 14–17 hold risks, roadmap, decisions and open questions.

## 2. Requirements and design drivers

Four drivers shape every decision below, in this priority order when they conflict: performance, portability, lightweight, extensibility. For an RL training platform "performance" means simulation throughput first and frame rate second, and "portability" means Windows x64 across GPU vendors today with a clean path to Linux servers later. Each driver has a measurable target so the architecture can be checked, not just argued.

| Driver | What it means here | Target (v1) | How it is enforced |
| --- | --- | --- | --- |
| Performance | Maximise simulated vehicle-steps per second; keep visualisation and vision rendering from slowing the simulation | ≥ 100,000 JSBSim vehicle-steps/s aggregate on a 16-core workstation (headless, stock c172/f16); a 64-vehicle scene renders at 60 fps at 1440p on an RTX-class GPU; vision observations at 128×128 for 64 vehicles ≥ 2,000 frames/s | parallel sim workers, zero per-step heap allocation, batched stepping API, offscreen render batching, throughput benchmark in CI |
| Portability | Windows 10/11 x64 on NVIDIA, AMD and Intel Vulkan drivers; no OS-specific code outside `platform/` so a Linux build is a port, not a rewrite | identical results on all three GPU vendors; `platform/` is the only module with Win32 includes (checked in CI) | C++17, VSG's own windowing and tile streaming, CMake, dependency isolation |
| Lightweight | Few dependencies, small install, low idle resource use, no language runtimes | headless library + viewer executable < 20 MB excluding terrain cache; ≤ 5 third-party libraries in the core; headless start < 1 s; idle RAM < 500 MB with 64 vehicles headless | static linking, dependency budget reviewed per ADR, VSG-native solutions preferred over third-party ones |
| Extensibility | New aircraft, tasks/rewards, observation types, sensors, cameras, data sinks and external trainers without touching the core | add a task or observation builder as a C++ module without core edits; bind the platform from any language through the C ABI; optional modules excluded at build time | module registry, property store, event bus, stable C++ SDK headers, versioned C ABI |

Secondary requirements that follow from the four drivers and the owner decisions:

- Determinism and reproducibility: given a seed, a scenario and a fixed step, every run produces the same trajectory; JSBSim guarantees this per instance when fed a fixed `dt`, and the platform never lets wall-clock time enter the simulation.
- Headless first: the simulation and environment API run with no window and no GPU; rendering is an optional module attached to a running simulation for visualisation or vision observations.
- Many vehicles: dozens to hundreds of JSBSim instances per process, stepped in parallel and in lockstep, grouped into independent environments.
- Full-Earth world on VSG alone: any latitude/longitude, WGS-84 ECEF double precision, imagery and elevation streamed by `vsg::TileDatabase` from a tile pyramid (self-hosted or public), and the same elevation tiles used for physics ground height, headless and offline once cached.
- Pure C++: no Python, Lua or other language runtime anywhere in the platform; external trainers use the C++ SDK or the C ABI. Still true since the Python SDK (2026-09-23, `python/`): it is a C extension over the C ABI loaded by the trainer's own interpreter, and hangar (`tools/hangar`), the aircraft design tool, is offline Python tooling that, like `tools/tile_builder`'s GDAL, is never part of the platform.
- Open source: the platform is released under MIT; every dependency is MIT, Apache-2.0, BSD, Zlib or LGPL (JSBSim), with JSBSim linked as a shared library.
- JSBSim as-is: stock aircraft, engines, systems and XML scripts from the JSBSim repository; flight-control automation uses JSBSim's own FCS/autopilot definitions; no custom FDM code in v1.

## 3. Technology stack baseline

Licensing consequence: JSBSim's LGPL-2.1 is the only copyleft component. Dynamic linking of JSBSim, or shipping object files to allow relinking, keeps the application's own licence free. All other core components are MIT/Zlib/Apache.

The fixed stack is C++17, VSG 1.1.x and JSBSim 1.3.x, with vsgXchange for model loading and HTTP tile fetching and vsgImGui for the viewer UI; nothing else is required at runtime. All components are actively maintained and MIT/LGPL licensed; CMake is the build system because every dependency already uses it. Status checked 2026-09-19.

| Component | Version | License | Last release / push | Role | Notes |
| --- | --- | --- | --- | --- | --- |
| C++ | C++17 (GCC 16.2, MSYS2 UCRT64 — mandated toolchain at D:\\ENV\\DevLanguages\\Cpp\\msys2\\ucrt64) | — | — | Implementation language | C++17 is VSG's requirement; code stays C++17 so a GCC/Clang Linux port needs no language changes |
| [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) | 1.1.16 | MIT | 2026-08-21 | Scene graph, Vulkan rendering, windowing, offscreen rendering, viewer, **full-Earth tile streaming (`vsg::TileDatabase` with `imageLayer` + `elevationLayer`)**, serialisation (`.vsgt/.vsgb`) | Requires Vulkan 1.1 SDK; glslang optional |
| [vsgXchange](https://github.com/vsg-dev/vsgXchange) | tracks VSG | MIT | 2026-08-24 | glTF loader (assimp), KTX/PNG/JPEG image readers, `curl` reader for HTTP tile sources | Only assimp, KTX, stb\_image and curl modules enabled; GDAL module off |
| [JSBSim](https://github.com/JSBSim-Team/jsbsim) | 1.3.1 | LGPL-2.1 | 2026-05-17 (push 2026-09-15) | Flight dynamics (`FGFDMExec`), stock aircraft/engines/systems, XML scripts, property tree | Linked as a DLL; one instance per vehicle |
| [vsgImGui](https://github.com/vsg-dev/vsgImGui) | 0.8.0 | MIT | 2026-08-21 | Dear ImGui + ImPlot debug/monitor UI inside the VSG frame | Viewer only |
| libcurl (via vsgXchange) | current | MIT-style | active | HTTP(S) fetch of imagery/elevation tiles when a remote pyramid is used | not needed for file-based pyramids |
| CMake + vcpkg | ≥ 3.25 / manifest mode | BSD-3 / MIT | — | Build and dependency management | vcpkg has ports for every dependency above |
| Offline tooling only: GDAL | current | MIT | active | `tools/tile_builder` converts DEM and imagery sources into the tile pyramid | never linked into the platform; runs at data-preparation time |

Compiler and platform for v1 (owner decision 2026-09-19): MSYS2 **UCRT64** — GCC 16.2, mingw-w64, UCRT — on Windows 10/11 x64; CMake 4.1 and Ninja from the same environment; Vulkan headers/loader, glslang, SPIRV-Tools, assimp and curl from MSYS2 packages. The MSYS2 installation must be kept consistent with `pacman -Syu` (partial upgrades break newly installed packages). MSVC is not used. A Linux build is not a v1 deliverable but nothing in the stack prevents it.

## 4. Entry point and primary interface evaluation

Recommendation: the primary interface is a C++ SDK (`fsim::VecEnv`, static or shared library, public headers under `include/fsim/`) with a versioned C ABI (`fsim_c.h`) exported from the same DLL so trainers in any compiled language can bind it; an optional shared-memory environment server serves out-of-process trainers. The graphical interface is an optional in-process viewer using VSG's native window and Dear ImGui (vsgImGui). No scripting runtime, no flight-stick input and no cockpit UI.

### 4.1 Primary interface: how RL code reaches the simulator

| # | Option | How it works | Step overhead (64 envs) | Extra dependencies | Verdict |
| --- | --- | --- | --- | --- | --- |
| I1 | C++ SDK, in-process | trainer links `fsim.lib`/`fsim.dll`, calls `VecEnv::step(actions) -> batch` on preallocated buffers | \~0 (function call) | none | **Selected as primary** — fastest possible, type-safe, the platform's own tests and tools use it |
| I2 | C ABI, in-process | `fsim_c.h`: opaque handles, plain structs, `fsim_vecenv_step(h, actions, n)`; buffers owned by the library | \~0 | none | **Selected as companion** — stable across compiler versions; Rust/C#/Julia/Go bind it with their standard FFI tools; an `extern "C"` layer over I1 |
| I3 | Shared-memory environment server | `flightsim.exe --serve` maps a ring of action/observation buffers; trainer process attaches; semaphore handshake per step | \~20–50 µs | none (Win32 named shared memory in `platform/`) | **Selected as optional module** `ext/env_server` — isolates trainer crashes from the simulator, allows a different toolchain per side |
| I4 | TCP/UDP flat-binary protocol | same message layout as I3 over a socket | 100–300 µs + network | none | variant of I3 for remote trainers; post-v1 |
| I5 | gRPC / protobuf service | RPC per step | 200–500 µs, serialisation | gRPC, protobuf (\~10 MB) | rejected: dependency weight and latency for no gain over I3/I4 |
| I6 | Embedded scripting runtime (Lua, Python) | scripts drive the loop in-process | — | runtime + bindings | rejected by owner decision |
| I7 | In-process training with LibTorch | RL algorithm in C++ using the LibTorch C++ API | 0 | LibTorch (CPU \~200 MB, CUDA \~2 GB) | not part of the platform; supported as a consumer of I1 (`examples/torch_ppo` shows it); never a core dependency |

### 4.2 Graphical interface (optional viewer)

| # | Option | UI toolkit | Extra dependencies | Approx. size | Verdict |
| --- | --- | --- | --- | --- | --- |
| A | VSG native window + vsgImGui | Dear ImGui + ImPlot | none beyond VSG | \~0.5 MB | **Selected** — monitoring, telemetry plots, property browser, camera control |
| C | SDL3 / GLFW window + adopted surface | Dear ImGui | SDL3 or GLFW | 0.3–2 MB | rejected: a second library owning the window for no benefit; joystick support is not needed |
| E | Qt 6 + vsgQt | Qt Widgets/QML | Qt6 (LGPL-3) | 20–35 MB | rejected for the viewer; possible future scenario editor as a separate tool |
| G | RmlUi | HTML/CSS documents | RmlUi + FreeType | 2–3 MB | rejected: no operator-facing UI to justify it |
| H | Web dashboard (browser) | HTML/JS served from the process | embedded HTTP server (\~0.3 MB) | small | deferred: remote monitoring of long training runs; post-v1 module |

### 4.3 Scoring against the requirements

Scale 1 (poor) to 5 (excellent); weights follow the driver priority in section 2.

| Criterion (weight) | I1+I2 C++ SDK & C ABI + A | I3 shared memory + A | I5 gRPC + A | I1 + E Qt viewer |
| --- | --- | --- | --- | --- |
| Performance (3) | 5 | 4 | 2 | 5 |
| Portability (3) | 5 | 4 | 5 | 4 |
| Lightweight (2) | 5 | 5 | 3 | 1 |
| Extensibility (2) | 5 | 5 | 4 | 5 |
| VSG / JSBSim integration (2) | 5 | 5 | 4 | 4 |
| Maintainability (2) | 5 | 4 | 4 | 4 |
| Weighted total (max 70) | 70 | 62 | 51 | 55 |

### 4.4 Decision and rationale

- Entry points: (1) the `fsim` library, linked by the trainer's own C++ program — the primary one; (2) `flightsim.exe` for headless batch runs, benchmarks, recording and `--serve` (shared-memory server); (3) `flightsim-viewer.exe`, or `attachViewer()` from the SDK, for the window. All construct the same `Application` object.
- Batched API: `VecEnv` holds M environments, each with K vehicles; `step()` consumes an `(M×K×A)` action span and fills `(M×K×O)` observation, reward and flag buffers owned by the library, so one call advances every vehicle and no data is copied. The C ABI exposes the same buffers as raw pointers with sizes and a layout version.
- Windowing and viewer: `vsg::Window` and `vsg::Viewer`, Dear ImGui through vsgImGui. The viewer never blocks the simulation; it renders the latest snapshot at display rate (section 6).
- Not included: Python (bindings over the C ABI followed on 2026-09-23 at the owner's request, section 15), Lua, Cesium/3D Tiles, SDL3, HUD, instruments, cockpit view. The event/property infrastructure stays, so a piloted mode can be added later as a module.

## 5. System architecture overview

The platform is a set of statically linked C++ modules in five layers with strictly downward dependencies, exposed through the `fsim` library (C++ SDK + C ABI) and two thin executables. VSG's object model (`vsg::Object`, `vsg::ref_ptr`, visitors, `vsg::Input/Output`) is the single object model for the codebase; the `env` layer, which turns simulations into RL environments, sits above `sim` and knows nothing about rendering.

```mermaid
flowchart TD
    SDK[fsim library<br/>C++ SDK + C ABI] --> ENV
    EXE[app<br/>flightsim.exe, viewer] --> ENV[env<br/>VecEnv, tasks, observations]
    EXE --> WORLD[world<br/>earth tiles, vehicles, cameras]
    EXE --> UI[ui<br/>ImGui viewer layers]
    ENV --> SIM[sim<br/>vehicle pool, JSBSim adapters]
    WORLD --> SIM
    WORLD --> RENDER[render<br/>VSG viewer, offscreen]
    UI --> RENDER
    SIM --> CORE[core<br/>registry, bus, properties, jobs]
    RENDER --> CORE
    CORE --> IO[io<br/>assets, config, tile cache]
    CORE --> PLATFORM[platform<br/>Win32 isolation]
    EXT[ext/*<br/>env_server, recorder, ...] -.-> CORE
```

Reading the diagram: the `fsim` SDK (object model in `sim`/`control`/`comm`, batch layer in `env`) is the only thing a trainer sees; `ipc` publishes the world for the viewer; `world` and `render` are optional and attach to a running `sim` to visualise it or to produce vision observations; `core` knows neither JSBSim nor Vulkan and runs headless.

| Module | Responsibility | Depends on | External libs |
| --- | --- | --- | --- |
| `fsim` (SDK) | Public C++ headers (`fsim/World.h`, `fsim/Vehicle.h`, `fsim/Control.h`, `fsim/Environment.h`, `fsim/Effects.h`, `fsim/Comm.h`, `fsim/VecEnv.h`) and the `extern "C"` layer `fsim_c.h`; the only exported symbols of `fsim.dll` | `env`, `control`, `comm`, `ipc` | — |
| `control` | Multi-level control stack: command types per level, `Controller`/`Behavior` interfaces, cascade, built-in PID loops and behaviours, controller registry | `sim`, `core` | — |
| `comm` | Communication abstractions: nodes, messages, codecs, protocols, delivery media | `core` | — |
| `ipc` | Shared-memory world segment: publisher (in `fsim.dll`) and read-only mirror (viewer), registry of live worlds | `sim`, `platform` | Win32 file mapping |
| `vision` | `fsim_vision.dll`: vehicle cameras rendered offscreen (headless `vsg::Viewer`, one framebuffer per camera, RGB + depth readback, own-vehicle masking) over the public `World`; C ABI `fsim_vision_c.h` | `world`, `render`, `io`, `sdk` | VSG, Vulkan |
| `app` | `flightsim.exe` (headless runs, benchmark, record, `--serve`) and `flightsim-viewer.exe`; command line, configuration, module wiring | all | — |
| `env` | `VecEnv` batching M environments over the object model; observation and action builders (actions are commands at a chosen control level); task/reward/termination interface; seeding; episode bookkeeping | `control`, `sim`, `core` | — |
| `session` | `World` implementation: vehicle registry by name/type over the pool, per-step environment application, effects pipeline and control cascades on the owning worker, message fabric, shared-memory publisher | `control`, `effects`, `comm`, `ipc`, `sim` | — |
| `effects` | Effect/EffectContext (wind, force, property, sensed-state channels), built-in effects | `sim`, `core` | — |
| `sim` | `VehiclePool` (N `FlightModel` instances stepped in parallel by a worker pool in lockstep, one worker per physical core, pre-step hook for cascades and effects), JSBSim adapter with wind/turbulence/atmosphere/external-force/seed channels, `GroundProvider` reading elevation tiles | `core`, `io` | JSBSim |
| `world` | Scene composition: `vsg::TileDatabase` Earth (imagery + elevation layers), vehicle visuals and part animation, cameras (chase, orbit, free, per-vehicle sensor cameras), sun and sky; maps sim snapshots to scene transforms | `sim`, `render`, `core` | VSG |
| `render` | Window and offscreen render targets, viewer, render/command graphs, shader sets, GPU→host readback for vision observations, frame statistics | `core` | VSG, vsgXchange |
| `ui` | ImGui layers: monitor (throughput, episode stats), telemetry plots, property browser, camera and vehicle selection, scenario controls | `render`, `core` | vsgImGui |
| `io` | Asset resolver, config, JSBSim aircraft path management, tile pyramid access and disk cache (shared by physics and rendering), recording files | `platform` | vsgXchange (curl, image readers) |
| `core` | Module registry and lifecycle, event bus, property store, deterministic RNG streams, job system, logging, profiler | `io`, `platform` | VSG core |
| `platform` | Win32 specifics: paths, high-resolution clock, thread naming/affinity, shared memory, HTTP, UDP, crash handler (minidump via dbghelp, installed by the executables only) | — | Win32 |
| `ext/*` | Optional modules: `env_server` (shared memory), `recorder`, `web_dashboard`, extra sensors, scripted traffic | `core` + what they need | per module |

Dependency rules, enforced by CMake target visibility and a CI check:

1. No module includes a header from a layer above it; `env` never includes `render` or `world`.
2. JSBSim headers appear only inside `sim`; Vulkan and `vsg::` GPU types only inside `render`, `world` and `ui`; Win32 headers only inside `platform`.
3. Hot-path communication (stepping, snapshots, observations) uses typed buffers described in section 6, never the event bus.
4. Optional modules are compiled in via CMake options and self-register through `core::ModuleRegistry`; the headless configuration (`sim` + `env` + `fsim`, no `render`/`world`/`ui`) builds and runs without the Vulkan SDK.
5. Only `fsim/*.h` and `fsim_c.h` are public; everything else may change without notice.

## 6. Runtime model

The simulation is driven in lockstep by the caller (`VecEnv::step()` from the trainer's C++ code, the C ABI, or the shared-memory server), not by a clock: each call advances every vehicle by a fixed number of JSBSim steps across a worker pool, then builds observations; the optional viewer runs on its own thread at display rate and reads immutable snapshots, so visualisation never slows training.

### 6.1 Process entry

1. The trainer constructs `fsim::VecEnv(scenarioPath, options)` (or `flightsim.exe` does) which builds `Application` in headless mode: `core` services, `io`, `sim`, `env`. The Vulkan SDK is not touched.
2. Scenario files (`.vsgt` text or JSON) define per-environment vehicles (JSBSim aircraft name, initial-condition distributions, JSBSim script if any), the task id, observation and action specs, the sensors and the tile pyramid to use for ground height.
3. `VehiclePool` creates one `JsbsimModel` per vehicle (M environments × K vehicles), each with its own `FGFDMExec`, `dt` and RNG stream derived from the seed.
4. `reset()` runs `RunIC()` per vehicle and fills the first observation batch.
5. `step(actions)` for each call: scatter actions to vehicles, step all vehicles `frameSkip` times in parallel, compute rewards/terminations in the task, gather observations into the output buffers, auto-reset finished environments, publish a snapshot for viewers.
6. `attachViewer()` (or `flightsim-viewer.exe --attach`) constructs `render`, `world` and `ui` on a separate thread with a window; detaching destroys them without affecting the simulation.
7. Shutdown is reverse order; `shutdown()` runs on every module even after an exception so JSBSim outputs and recordings are flushed.

### 6.2 Threads

| Thread | Driven by | Owns | Must never |
| --- | --- | --- | --- |
| Caller (trainer) | `step()` / `reset()` calls | `VecEnv`, task evaluation, observation assembly | touch `vsg::` GPU objects |
| Sim workers (`nWorkers`, default = physical cores − 2) | work items = one vehicle × `frameSkip` steps | `FGFDMExec` instances (each worker touches only its assigned vehicles for a given call) | allocate, log at info level, or share state across vehicles |
| Render (viewer only) | display refresh | `vsg::Viewer`, window events, ImGui, scene mutation | call into JSBSim or block the sim workers |
| Vision render (optional) | `step()` when a vision observation is requested | offscreen `vsg::View`s for sensor cameras, readback | — (synchronous with the step: the caller waits for readback) |
| Job pool | on demand | tile fetch/decode for `vsg::TileDatabase` and the ground provider, recording flush | mutate the live scene graph directly |
| Server (optional `ext/env_server`) | semaphore from the trainer process | shared-memory ring, one `VecEnv` | — |

### 6.3 Stepping and data flow

```mermaid
sequenceDiagram
    participant P as Trainer (C++ / C ABI)
    participant E as VecEnv
    participant W as Sim workers
    participant T as Task
    participant V as Viewer thread
    P->>E: step(actions M×K×A)
    E->>W: scatter actions, run frameSkip steps per vehicle
    W-->>E: VehicleState per vehicle
    E->>T: reward, terminated, truncated per environment
    E->>E: build observations into library-owned buffers
    E->>V: publish snapshot (triple buffer)
    E-->>P: spans over obs, reward, flags
    V->>V: interpolate, update scene, render at display rate
```

`VehicleState` is a fixed-size POD (ECEF position `dvec3`, attitude `dquat`, body and NED velocities, rates, accelerations, control positions, engine data, gear state, \~60 scalars) written by the worker that stepped the vehicle. Observation builders read these plus any cached JSBSim property handles declared in the observation spec; outputs are `float32` arrays owned by `VecEnv` and returned as `std::span` (C++) or pointer + length (C ABI) without copying. Actions map to JSBSim `fcs/*-cmd-norm` properties or, when the scenario says so, to JSBSim autopilot targets (heading, altitude, airspeed), which uses JSBSim's own FCS as-is.

### 6.4 Time

- Simulation time advances only inside `step()`; there is no real-time clock in the training path. `dt` (default 1/120 s) and `frameSkip` (default 4 → 30 Hz agent rate) are scenario parameters.
- Determinism: fixed `dt`, per-vehicle RNG streams seeded from `(seed, envIndex, vehicleIndex)`, and no dependence on thread scheduling (each vehicle's step is independent), so results are identical regardless of `nWorkers`.
- Viewer time: the render thread interpolates between the two latest snapshots using their simulation timestamps and shows the current wall-clock speed factor; a `--realtime` option throttles `step()` calls in the exe only.

### 6.5 Budget per `step()` (64 environments × 1 vehicle, frame\_skip 4, 16 workers)

Measured 2026-09-19 (M0 build, Release, 16-thread desktop, c172x, frame-skip 4): one JSBSim step costs \~6 µs, not the 40 µs assumed in the first draft. 64 vehicles run at 148,000 vehicle-steps/s on one worker and 1,138,000 on 16 workers (`step()` mean 0.225 ms), i.e. 11× the section 2 target before any tuning; the f16 runs at 757,000 on 6 workers. Vision observations add GPU render + readback time and will be the dominant cost when enabled (section 8.4).

## 7. Flight dynamics integration (JSBSim)

JSBSim is used as-is — stock aircraft, engines, systems, autopilots and XML scripts from the JSBSim repository — behind a `sim::FlightModel` interface implemented by `sim::JsbsimModel`, one instance per vehicle; the rest of the platform never sees JSBSim types, units or its property tree, and all conversion to SI units and the ECEF-metre world frame happens inside the adapter.

### 7.1 Interface

```cpp
class FlightModel : public vsg::Inherit<vsg::Object, FlightModel> {
public:
    virtual bool load(const AircraftSpec&, const InitialConditions&) = 0;
    virtual void step(const ControlInputs&, double dt) = 0;   // exactly one fixed step
    virtual void state(VehicleState&) const = 0;              // fill snapshot, SI units, ECEF metres
    virtual PropertyHandle property(std::string_view path) = 0; // cached, O(1) read/write after lookup
    virtual void reset(const InitialConditions&) = 0;
};
```

`step()` is the only method called at 120 Hz; `property()` returns a handle wrapping a cached `FGPropertyNode*` so hot reads (instrument values, control positions) never repeat a string lookup.

### 7.2 JSBSim adapter lifecycle

| Phase | JSBSim calls | Notes |
| --- | --- | --- |
| Construct | `FGFDMExec`, `SetRootDir`, `SetAircraftPath`, `SetEnginePath`, `SetSystemsPath`, `Setdt(dt)` | one instance per vehicle; JSBSim's own `aircraft/`, `engine/`, `systems/` and `scripts/` directories are shipped unchanged as the asset root |
| Load | `LoadModel(name)`, `GetIC()->...`, optionally `LoadScript(path)`, `RunIC()` | runs on a sim worker inside `try/catch`; XML errors become a `LoadFailed` result on the environment, never a crash. Initial conditions come from the scenario (lat/lon/alt, heading, speed) or a JSBSim IC file |
| Step | write commands via cached nodes (`fcs/*-cmd-norm`, `fcs/throttle-cmd-norm[n]`, gear, flaps, brakes, or autopilot targets `ap/*` when the aircraft defines them), `Run()` × `frame_skip`, read state | `Run()` advances exactly `dt`; JSBSim scripts, event triggers and `FGOutput` (CSV/socket) remain available and are enabled per scenario for logging |
| Reset | `ResetToInitialConditions(0)`, then re-apply randomised ICs from the environment's RNG stream | preserves the loaded model; a reset costs \~1 ms, so environments auto-reset in place |
| Destroy | destructor | JSBSim logging routed to `core::Log` via `FGLogger`, level warning and above only on workers |

### 7.3 Units and frames

| Quantity | JSBSim | Application (`VehicleState`) | Conversion |
| --- | --- | --- | --- |
| Position | ECEF, feet (`FGPropagate::GetLocation`), WGS-84 | ECEF, metres, `vsg::dvec3` | × 0.3048 |
| Attitude | body-to-local (NED) Euler/quaternion; body axes x forward, y right, z down | ECEF-to-body quaternion `vsg::dquat`; model axes x forward, y left, z up | compose with NED-to-ECEF at current lat/lon; body→model flips y and z |
| Velocities | body frame ft/s, NED ft/s | body and NED m/s | × 0.3048 |
| Angular rates | rad/s | rad/s | none |
| Mass, forces | slugs, lbf | kg, N | × 14.5939, × 4.44822 |
| Altitude | ft (`position/h-sl-ft`), AGL via terrain callback | m | × 0.3048 |

Terrain: JSBSim asks the host for ground elevation through `FGGroundCallback`; the adapter implements it against `sim::GroundProvider`, which samples the same elevation tile pyramid that `vsg::TileDatabase` renders (section 8.2), decoded on the CPU by `io::TilePyramid` at a configurable level (default: the pyramid's finest level, 30 m) with bilinear interpolation in double precision. Physics and rendering therefore read identical data, the provider works headless and offline once tiles are cached, and it is deterministic.

### 7.4 Rendering frame

The world frame is ECEF metres in double precision. `world` keeps a floating origin: the scene root is a `vsg::MatrixTransform` with a `dmat4` that re-bases the scene near the camera, so GPU floats never see coordinates above \~10 km. VSG performs the `dmat4` × `dmat4` multiplication on the CPU during record and uploads floats, which is what its `dmat4` transforms exist for.

### 7.5 Multiple vehicles

`sim::VehiclePool` owns N `FlightModel`s and steps them with a fixed worker pool: each `step()` call partitions vehicles across workers in a deterministic order, every worker steps its vehicles `frame_skip` times, and a barrier ends the call. Vehicles share nothing (separate `FGFDMExec`, separate ground-provider cache handles), so scaling is linear until memory bandwidth limits; JSBSim's \~5–10 MB per instance puts 256 vehicles at 1.5–2.5 GB, well within the training hardware. Interactions between vehicles (relative geometry for pursuit/evasion or formation tasks, simple collision detection) are computed in the `env` task layer from the snapshot batch after the step, not inside the FDM. A lighter `KinematicModel` implementing the same interface serves scripted traffic and replay without a JSBSim instance.

## 8. Visualization design (VSG)

Rendering is an optional module built entirely on VSG with two jobs: a full-Earth viewer for humans (one `vsg::Viewer`, one window, Earth streamed by VSG's own `vsg::TileDatabase`, ImGui overlay) and offscreen sensor cameras that produce vision observations for agents; both read the same immutable snapshots and share one scene graph, and neither is present in the headless training configuration.

### 8.1 Scene graph layout

```mermaid
flowchart TD
    ROOT[Group root] --> EARTH[vsg::TileDatabase<br/>imageLayer + elevationLayer]
    ROOT --> ORIGIN[MatrixTransform<br/>floating origin dmat4]
    ORIGIN --> SKY[Sky, sun, atmosphere]
    ORIGIN --> VEH[Group vehicles N]
    VEH --> V1[MatrixTransform<br/>vehicle pose]
    V1 --> MODEL[Aircraft model glTF]
    MODEL --> PARTS[Animated parts<br/>surfaces, gear, props]
    ORIGIN --> FX[Effects<br/>trails, markers]
    ROOT --> SENSORS[Offscreen Views<br/>per-vehicle cameras]
```

`vsg::TileDatabase` builds a `PagedLOD` quadtree over the WGS-84 `EllipsoidModel` in ECEF metres with its own precision handling; vehicle transforms above the model level are `dmat4`, model-local data is float.

### 8.2 Components

| Concern | Design | Why |
| --- | --- | --- |
| Full-Earth terrain and imagery | `vsg::TileDatabase` with `TileDatabaseSettings`: `imageLayer` and `elevationLayer` URL templates (`{z}/{x}/{y}`), `ellipsoidModel` = WGS-84, `maxLevel` per pyramid, `lodTransitionScreenHeightRatio` tuned per quality tier; tiles come from a **self-hosted tile pyramid** (files on disk or any HTTP server) produced by `tools/tile_builder`, or from public XYZ imagery servers (OpenStreetMap, Bing via VSG's `createBingMapsSettings()`) for quick starts | VSG-native, zero extra runtime dependency, streamed by VSG's `DatabasePager` on the job pool, cached on disk by `io` |
| Tile pyramid format | imagery: JPEG or KTX2 256×256 tiles; elevation: 16-bit PNG heightmaps (metres × scale + offset, documented in `pyramid.json`) decoded by vsgXchange's stb\_image reader and interpreted through `elevationLayerCallback`; global coverage from Copernicus GLO-30 (30 m) or GLO-90 built offline | one format for physics and rendering; PNG/JPEG readers already in the stack; no GDAL at runtime |
| Physics vs. visual ground | both read `io::TilePyramid`; the viewer's debug layer shows the residual between the CPU sample and the rendered mesh at the selected vehicle (mesh simplification + skirts explain sub-metre differences) | consistency by construction |
| Aircraft models | glTF 2.0 via vsgXchange; a `ModelManifest` (`.vsgt`) maps named nodes to `VehicleState` fields; stock JSBSim aircraft get simple placeholder models until proper ones exist | one loader, PBR materials |
| Part animation | `world::PartAnimator` resolves named transforms once at load and writes rotations each frame from the interpolated snapshot | no per-frame lookups |
| Sky / lighting | analytic sky shader; sun position from scenario date/time; one directional light + ambient; VSG PBR `ShaderSet`; shadows optional | no texture assets; time of day for free |
| Cameras | `CameraController` strategies: chase, orbit, free, tower, follow-selected; plus `SensorCamera`s attached to vehicles for vision observations | swap by key; sensors are data-defined in the scenario |
| Many vehicles | vehicle subgraphs share model geometry (one loaded model, N transforms); per-vehicle `vsg::Switch` for visibility; selection and labels via ImGui overlay | 64–256 vehicles at 60 fps with a few draw calls per model |
| Shaders | GLSL compiled to SPIR-V at build time; runtime glslang only in developer builds | size and startup |
| Frame statistics | VSG frame stamps + GPU timestamp queries in the ImGui monitor | needed to hold budgets |

### 8.3 Viewer update pass

Each frame the render thread pumps events (VSG visitors, then ImGui), acquires the latest snapshot batch and interpolates, writes vehicle `dmat4`s and part rotations, updates the floating origin when the camera moved more than 5 km, updates cameras, builds the ImGui frame, then `viewer->update()`, `recordAndSubmit()`, `present()`. `vsg::DatabasePager` loads and compiles tiles on its own threads and merges them during `update()`; the pager's memory and tile-request limits are set per quality tier.

### 8.4 Vision observations

When a scenario declares a camera observation (`rgb`, `depth`, `segmentation`; resolution, field of view, mount pose), `render` creates an offscreen `vsg::View` per sensor rendering into a shared image array; all sensors for a `step()` are recorded into one command buffer, submitted once, and read back with a single copy into a host-visible buffer exposed as an `(M×K×H×W×C)` `uint8`/`float16` span. Terrain tiles for sensor views are requested by the pager from each sensor camera, so agents see the same Earth the viewer does; for training on a fixed region the pyramid is prefetched by `tools/tile_builder --prefetch` so no I/O happens during episodes (today: `tools/tile_prefetch` fills the shared cache for a region from the public servers). Budget: 64 cameras at 128×128 RGB in under 30 ms per step on an RTX-class GPU; a GPU-resident path (no host readback) is a post-v1 optimisation. Delivered 2026-09-21 as `fsim_vision.dll` (`fsim::vision::Sensors`, `include/fsim/Vision.h`): a headless `vsg::Viewer` with one framebuffer and render graph per camera in a single command graph, colour attachments copied into cached host-visible images after the pass, own-vehicle hiding through view masks; 64 cameras measured at ~8 ms per step on an RTX 5070 Ti. Segmentation followed 2026-09-21: `CameraSpec::segmentation` adds a second pass per camera that draws an id-coloured copy of the vehicles over the depth the colour pass just wrote, so occlusion is exact, and reads back a `uint16` vehicle id per pixel (64 cameras: 13.2 ms without, 18.7 ms with). `rgb`/`depth`/`segmentation` are therefore all delivered; semantic classes beyond "which vehicle" are not.

### 8.5 Assets and cache

The install ships shaders, placeholder aircraft models, the JSBSim aircraft tree and the ImGui font (< 20 MB). The tile pyramid is separate data: a global 90 m elevation + low-resolution imagery pyramid is \~3 GB; 30 m elevation with 10 m imagery for a training region of 500×500 km is \~2 GB. Both are built once with `tools/tile_builder` and either copied locally or served from any static HTTP server; the runtime disk cache has a configurable size limit (default 20 GB). As built: the package ships the aircraft designed with hangar (`share/flightsim/aircraft/<name>`: the JSBSim files and the glTF model) and, since 2026-09-22, its own maps instead of a pyramid built with `tools/tile_builder` (not needed so far): `fetch-maps` downloads the public imagery and elevation (Esri World Imagery, AWS Terrain Tiles) that `assets/config/offline-map-plan.json` asks for - a global base to level 9, mountain ranges, airports and route corridors deeper, about 2.4 GB against the owner's 2.5 GB budget - and the packaged viewer reads only those files, opening no sockets.

## 9. User interface design: the vehicle SDK, transparent visualisation and the C ABI

Revised 2026-09-20 to the owner's workflow statement. Researchers write a **training application** against the `fsim` SDK; the platform ships a **prebuilt viewer** (`flightsim-viewer.exe`) that discovers the training application's world through shared memory and mirrors it. The two processes never talk directly, never wait for each other, and the training application contains no visualisation code. The SDK is organised around **vehicle instances** with a **multi-level control stack**, **real-time environment control**, and abstract **effect** and **communication** interfaces; the earlier vectorised `VecEnv` remains as a convenience layer built on top of it.

### 9.1 Researcher workflow

```mermaid
flowchart LR
    subgraph T[training application - researcher's process]
        P[policy / learner] --> W[fsim::World]
        W --> V1[Vehicle red-1]
        W --> V2[Vehicle blue-1]
        W --> ENV[Environment]
        W --> PUB[ipc::WorldPublisher]
    end
    PUB -- lock-free writes --> SHM[(shared memory<br/>Local\fsim.world.name)]
    SHM -- seqlock reads --> SUB[ipc::WorldMirror]
    subgraph Vw[flightsim-viewer.exe - prebuilt]
        SUB --> SCENE[world: Earth, models, cameras]
    end
```

1. The researcher reads `include/fsim/*.h` and `docs/sdk/*.md`, links `fsim.dll`, creates a `World`, creates vehicles by name, type and initial state, commands them at whichever control level the experiment needs, and steps the world.
2. Starting `flightsim-viewer.exe` at any time - before, during or after training starts - shows that world: the viewer lists live worlds, attaches to one, creates a visual for every vehicle it finds and follows creations, resets, state changes and control inputs from then on. Closing it changes nothing in the training process.
3. Visualisation costs the training application one bounded, lock-free memory copy per publish interval (section 9.7); it never allocates, blocks, or waits for a reader, so training throughput is the same with zero, one or several viewers attached.

### 9.2 Object model

```cpp
#include <fsim/World.h>

fsim::World world({.name = "dogfight-01", .dt = 1.0 / 120, .frameSkip = 4, .seed = 7});
world.environment().setWind({.directionDeg = 270, .speedMs = 8, .turbulence = 0.3});
world.environment().setTime(fsim::Utc{2026, 9, 20, 14, 30, 0});

fsim::Vehicle red  = world.createVehicle({.name = "red-1",  .type = "jsbsim:f16",
    .initial = {.latitudeDeg = 37.62, .longitudeDeg = -122.38, .altitudeMslM = 3000, .headingDeg = 90, .airspeedTrueMs = 220}});
fsim::Vehicle blue = world.createVehicle({.name = "blue-1", .type = "jsbsim:f16", .initial = /* ... */});

red.command(fsim::AttitudeCommand{.rollRad = 0.4, .pitchRad = 0.05, .throttle = 0.9});   // attitude loop
blue.command(fsim::PursuitBehavior{.target = red.id(), .rangeM = 500});                    // behaviour

for (int k = 0; k < 10'000; ++k) {
    world.step();                                    // all vehicles, lockstep, frameSkip FDM steps
    const fsim::VehicleState& s = red.state();       // truth
    const fsim::SensedState&  z = red.sensed();      // through the vehicle's sensor models
    red.command(policy(z));                          // any level, any time
}
```

| Type | Role | Notes |
| --- | --- | --- |
| `World` | The simulation session: owns vehicles, the environment, the clock, the worker pool and the shared-memory publisher | `createVehicle`, `removeVehicle`, `vehicle(id/name)`, `vehicles()`, `step(n = 1)`, `time()`, `environment()`, `addEffect` (world-wide), `comm()` (the medium), `reset()`; one `World` per training process is typical, several are allowed (each publishes under its own name) |
| `VehicleSpec` | What to create: `name` (unique in the world, shown by the viewer), `type` (`"jsbsim:<aircraft>"`; other prefixes map to other `FlightModel` implementations), `initial` state (position, attitude, velocity, on-ground), optional `model` (glTF override for the viewer), `sensors`, `controlRateHz` | The type string is the only thing the viewer needs to pick a model: `models/<aircraft>.glb` + manifest, or the default aircraft |
| `Vehicle` | A lightweight handle (id + world pointer) to a vehicle instance | `state()`, `sensed()`, `command(...)`, `controls()` (the stack), `reset(initial)`, `addEffect`, `comm()` (this vehicle's endpoint), `property(path)` for raw JSBSim access, `name()`, `type()`, `alive()` |
| `Environment` | Global conditions applied to every vehicle each step and published for illumination | section 9.4 |
| `Effect` | A disturbance or fidelity effect attached to a vehicle or the world | section 9.5 |
| `comm::Node` | A vehicle's or an external node's communication endpoint | section 9.6 |
| `VecEnv` | The gym-style batch layer: scenario -> vehicles, task -> rewards, spaces -> buffers | section 9.9; built entirely from the public `World`/`Vehicle` API, so it doubles as the reference example |

Rules: every call is on the caller's thread; `step()` is the only call that does work; handles stay valid until `removeVehicle`; nothing in the object model knows about rendering.

### 9.3 Multi-level control architecture

Every vehicle owns a **control stack**: an ordered set of levels, one controller per level, and one *active* command. Commanding a level makes it the active level; each step the stack runs the cascade from the active level down to the actuators, each controller translating its command into a command for a lower level, until an `ActuatorCommand` reaches the flight model. Nothing above the active level runs.

```mermaid
flowchart TD
    B[Behavior<br/>pursue, loiter, aerobatics, guidance] --> P
    P[Position<br/>lat/lon/alt or NED offset, speed] --> V
    V[Velocity<br/>NED velocity / airspeed + vertical speed + heading] --> A
    A[Acceleration<br/>body-axis specific accelerations, roll rate] --> T
    T[Attitude<br/>roll, pitch, yaw or heading, throttle] --> U
    U[Actuator<br/>aileron, elevator, rudder, throttle, flaps, gear, brakes] --> FDM[FlightModel]
```

| Concept | Definition |
| --- | --- |
| `ControlLevel` | `Actuator < Attitude < Acceleration < Velocity < Position < Behavior`; a strict order, so "lower" and "higher" are unambiguous |
| `Command` | One plain struct per level (`ActuatorCommand`, `AttitudeCommand`, `AccelerationCommand`, `VelocityCommand`, `PositionCommand`, `BehaviorCommand`) held in a `std::variant`; every field has a `hold`/`nan` = "keep current" convention so partial commands are natural |
| `Controller` | `level()` (the level it accepts) and `update(const ControlContext&, const Command& in) -> Command` returning a command at **any strictly lower** level; the stack keeps cascading from the returned level. A controller is plain C++ with a `reset()` and its own gains/state; built-ins are PID loops written against `VehicleState` only, so they work for any `FlightModel` |
| `Behavior` | A controller at the `Behavior` level with a lifecycle (`start`, `update`, `finished()`) and parameters, e.g. `Pursuit{target, range}`, `Loiter{centre, radius}`, `Waypoints{...}`, `Aerobatic{Loop, Roll, Immelmann}`; a finished behaviour holds its last output until replaced |
| `ControlStack` | Per vehicle: `command(Command)` sets the active level and command; `use(level, controllerId)` swaps the controller at a level; `controller(level)` for gain tuning; `activeLevel()`; `derived(level)` returns the command the cascade produced at any lower level last step (introspection, telemetry, observations); `rateHz` (defaults to the FDM rate) |
| `ControllerRegistry` | `registerController(id, level, factory)` and `registerBehavior(id, factory)` from trainer code or built-in modules; `VehicleSpec.controllers` and `use()` select by id, so a researcher swaps a loop without touching the rest of the stack |
| `ControlContext` | What a controller sees: truth `state`, `sensed` state, `dt`, the environment, the vehicle's `Rng` stream, the `World` (for behaviours that look at other vehicles) |

Choosing a level is one line (`vehicle.command(VelocityCommand{...})`); extending is one class (`struct MyLoop : fsim::Controller { ... }` + `registerController`); replacing a built-in loop is one call (`vehicle.controls().use(Level::Attitude, "my_attitude")`). Because RL actions are just commands, the same policy can act on surfaces, attitudes or velocities by changing one enum, and hierarchical RL maps to the stack directly (a high-level policy commands `Position`, a low-level policy owns `Attitude`).

Built-in controllers (v1): `Actuator` (identity, clamps), `pid_attitude` (roll/pitch/heading with throttle or airspeed hold), `pid_acceleration` (normal/longitudinal acceleration + roll rate to attitude), `pid_velocity` (airspeed, vertical speed, heading to acceleration), `pid_position` (guidance to a point / along a track to velocity), and behaviours `hold`, `waypoints`, `loiter`, `pursuit`, `evade`, `formation`, `aerobatics` (a sequence of attitude/rate segments). Gains are per aircraft type in `assets/control/<type>.json` with defaults that fly the stock JSBSim aircraft.

Per-aircraft gains (2026-09-26): the built-in loops' defaults suit the stock c172x, and flew most other aircraft badly - fly-by-wire fighters in a standing pitch oscillation, heavies drifting in height - so an aircraft may carry its own. They are JSBSim properties `fsim/control/<controller id>/<parameter>` in its flight control section, read once per aircraft type when it first loads (`FlightModel::properties`) and set by the `ControlStack` on every controller it creates by id (`setControllerSettings`); a trainer's own `setParameter` still wins, and an instance handed to `use()` is left as it is. The loops gained what makes one set of gains hold over an envelope: a schedule on true and equivalent airspeed (each channel's gains scaled by the inverse of how the aircraft's response to that control grows with speed), the elevator trim law of a surface-controlled aircraft, flight-path and 1 g angle-of-attack feedforwards, a lag on commanded vertical speed, and feedforwards of the stick a load factor or roll rate needs - all off by default, so a stock aircraft flies as before. hangar's `autopilot` stage identifies each design by small steps at a reference condition, places the loops' poles, writes the gains into the aircraft and flies it (docs/hangar.md, The autopilot).

### 9.4 Real-time environment control

`world.environment()` is a settable model applied to every vehicle's flight model at the start of each step (JSBSim: `FGAtmosphere` and `FGWinds` of every instance) and published to the viewer, which lights the scene accordingly.

| Parameter | API | Effect |
| --- | --- | --- |
| Time | `setTime(Utc)`, `setTimeFactor(x)`, `time()`; simulation time advances with `step()` | sun position and sky in the viewer; available to sensors and behaviours |
| Atmosphere | `setAtmosphere({temperatureSlK or deltaK, pressureSlPa, humidity})`, `setProperty(...)` for anything else | density, speed of sound, engine power through JSBSim's standard atmosphere |
| Wind | `setWind({directionDeg, speedMs, gustMs, turbulence [0..1], shearProfile})` and `setWindField(fn(position, time) -> WindNed)` for spatially varying wind | JSBSim `FGWinds` (steady wind, gusts, Dryden/Milspec turbulence with the vehicle's seed) |
| Visibility / precipitation | `setWeather({visibilityM, cloudBase, cloudCover, precipitation})` | viewer rendering and sensor models; no FDM effect |
| Ground | `setGround(GroundProvider)` (flat, terrain tiles, custom) | contact height for every vehicle |

All setters are immediate and take effect at the next step; changes are cheap (a few property writes per vehicle) and may be made every step. Per-vehicle overrides are effects (9.5), not environment settings.

### 9.5 Abstract vehicle simulation effects

`fsim::Effect` is the extension point for fidelity and disturbance: `apply(EffectContext&)` runs once per FDM step per vehicle it is attached to, before the flight model steps. The context exposes what an effect may touch:

| Channel | Context call | Implementation |
| --- | --- | --- |
| Local wind | `setWindNed(...)`, `addWindNed(...)` | per-instance `FGWinds`, overriding the global wind for that vehicle |
| External force / moment | `addForceBody(N)`, `addMomentBody(N·m)` | a generic external reaction injected into the JSBSim instance at load time (in-memory `<external_reactions>` element), so no aircraft XML edit is needed |
| Property | `property(path).set(...)` | any JSBSim/FDM property (mass, fuel, engine health, control-surface limits) |
| Sensors | `SensedState& sensed()` | noise, bias, drift, dropout, latency on what `vehicle.sensed()` returns; truth is untouched |
| Electromagnetic / GNSS | `gnss().degrade(...)`, `radio().jam(...)` | marks the sensed state and the communication medium (9.6) |
| Lifecycle | `onReset()`, `enabled` | effects survive vehicle resets |

Built-in effects (v1): `GaussianSensorNoise`, `SensorLatency`, `ConstantForce`, `WindGustField`, `GnssDegradation`; the interfaces are defined and tested even where an implementation is minimal, so researchers can add electromagnetic interference, icing, damage or actuator faults as effects without touching `sim`.

### 9.6 Abstract communication interfaces

`fsim::comm` models messages between vehicles and other nodes (ground stations, the trainer itself) without fixing a wire format or protocol:

| Concept | Role |
| --- | --- |
| `Node` | An endpoint with an address (`vehicle:<id>` or a named external node); `send(Message)`, `receive() -> span<Message>` for messages delivered up to the current step |
| `Message` | `from`, `to` (address or broadcast/group), `channel`, `timeSent`, `timeDelivered`, `Payload` (bytes + format id) |
| `Codec` | Encode/decode a typed struct to `Payload` under a format id (`raw`, `json`, `msgpack`, user formats); vehicles exchange typed state reports, commands, or arbitrary bytes |
| `Protocol` | Application-level rules on top of nodes (request/response, periodic beacons, group membership, ack/retry); registered by id; users add MAVLink-like or custom protocols |
| `Medium` | Delivery model for the whole world, stepped with the simulation: `IdealMedium` (instant, lossless), `LinkModel` (range, line-of-sight, latency, jitter, loss, bandwidth) |
| `Transport` / `BridgeProtocol` | A byte transport (`createUdpTransport`; users add serial, shared memory, queues) under a protocol that carries one node's traffic to an external process or device in a fixed little-endian wire format (`"FSMG"` header + payload), so external software is a node of the world behind the same medium |

Every part is a registry-backed interface; the v1 implementation ships `IdealMedium`, `LinkModel`, the `raw` and `json` codecs, a `beacon` protocol (periodic state reports) that `pursuit`/`formation` behaviours use to find their targets, and the UDP bridge (`examples/udp_peer` is a peer in plain sockets).

### 9.7 Transparent visualisation through shared memory

`ipc::WorldPublisher` (inside `fsim.dll`, created by every `World`) and `ipc::WorldMirror` (inside the viewer) share one named page-file-backed mapping, `Local\fsim.world.<name>`, plus a tiny `Local\fsim.registry` listing live worlds for discovery. No sockets, no daemons, no handshakes.

| Region | Content | Written by | Concurrency |
| --- | --- | --- | --- |
| Header | magic, layout version, world name, `dt`, frame skip, capacity, simulation time, wall clock of the last publish, time factor, environment (wind, atmosphere, weather, UTC), `readers` (attached viewers) | publisher (readers increment `readers`) | plain atomics |
| Vehicle table `[capacity]` | id, `name`, `type`, model override, `generation` (bumped on create/reset/remove), alive flag, control level, initial conditions | publisher on create/reset/remove | per-row seqlock |
| State slots `[3][capacity]` | `VehicleState` (appended-only layout) + `ControlInputs` + derived commands of the cascade + per-vehicle sensed summary | publisher, triple buffered, at most every `publishIntervalMs` (default 16 ms) | slot seqlock; writer never waits |
| Event ring | create, remove, reset, effect attached, message sent (metadata only), small fixed-size records | publisher | single-producer ring, readers catch up or skip |

Guarantees: the writer is wait-free (a bounded `memcpy` behind a sequence counter); publishing is rate limited by wall clock so a 5,000× real-time trainer copies at 60 Hz, not at 600 kHz; a reader that lags or dies is never noticed by the writer; a crashed publisher leaves a stale mapping the viewer reports as "not updating". The viewer interpolates between the two latest slots by wall time, exactly as it does today for its own sim thread, and renders vehicle creations/removals as the table's generations change. The same segment is what the optional out-of-process environment server (ADR-17) extends with an action/observation exchange; the mirror is read-only by design.

### 9.8 C ABI

`fsim_c.h` exports the object model as opaque handles and plain structs: `fsim_world_create/destroy/step`, `fsim_world_create_vehicle(world, const fsim_vehicle_spec*, uint32_t* id)`, `fsim_vehicle_state(world, id, fsim_vehicle_state*)`, `fsim_vehicle_command_<level>(world, id, const fsim_<level>_command*)`, `fsim_world_set_wind/atmosphere/time`, `fsim_comm_send/receive`, and the existing `fsim_vecenv_*` batch API. Rules: `struct_size`-versioned structs that only grow within a major version, library-owned buffers, integer return codes with `fsim_last_error()`, no exceptions and no C++ types. This is the binding surface for any language with a C FFI; the platform ships no bindings.

### 9.9 `VecEnv` batch layer

`fsim::VecEnv` is now a thin composition of the object model: a scenario declares vehicles per environment (spec templates + initial-condition ranges), a `Task` produces rewards/terminations from `Vehicle` handles, an `ObservationBuilder` reads `state()`/`sensed()`/derived commands, and an `ActionMapper` issues a command at a chosen level (`"surfaces"` -> `ActuatorCommand`, `"attitude"` -> `AttitudeCommand`, `"velocity"` -> `VelocityCommand`, ...). It keeps its vehicle-major buffers, Gymnasium auto-reset semantics and the `fsim_vecenv_*` C ABI, and gains the viewer for free because its `World` publishes like any other.

### 9.10 Command line

`flightsim-viewer.exe [--world <name>] [--list] [--demo ...]`: with no arguments the viewer lists live worlds in the registry, attaches to the only one (or the newest) and waits, showing "waiting for a training application" until one appears; `--demo` runs the built-in scenario with its own simulation thread as before (development and screenshots). `flightsim.exe` keeps the headless benchmark and `--serve` (ADR-17). Parsing uses `vsg::CommandLine`; every option maps to the same configuration schema as `config.json`.

### 9.11 Viewer layers (ImGui)

| Layer | Content | Default key |
| --- | --- | --- |
| Monitor | attached world, publish rate and age, vehicle-steps/s reported by the publisher, environment summary, GPU/CPU frame time | always on |
| Vehicle list | every mirrored vehicle: name, type, control level, state summary; click to follow; multi-select for labels and trails | `L` |
| Telemetry | ImPlot strip charts of any state field or derived command for the selected vehicles; CSV export | `T` |
| Control stack | the active level and the cascade's derived commands for the selected vehicle | `K` |
| Environment | wind, atmosphere, weather, time as published (read-only in mirror mode) | `E` |
| Sensor preview | thumbnails of vision observations for the selected vehicle | `V` |
| Debug | VSG statistics, tile streaming status, mirror status (age, dropped slots), thread states | `F3` |

### 9.12 Input and style

Keyboard and mouse only, delivered by VSG to a `ui::EventDispatcher` that offers events to ImGui first and then to the camera controller and key-binding table. One ImGui style, DPI scaling from the window's content scale, a single embedded font.

### 9.13 Scenario files

A scenario is a JSON document (`include/fsim/Scenario.h`) holding the `WorldOptions`, an optional environment (UTC time, wind, atmosphere, weather), world-wide effects, and the vehicles to create - each with its `VehicleSpec`, an optional instance `count` (placed abreast), an initial command at any level (behaviour targets by vehicle name, resolved after all vehicles exist) and its own effects. `fsim::loadScenario` parses it (errors carry `file:line:col`), `fsim::applyScenario(world, scenario)` performs the setup in order (environment, world effects, vehicles and their effects, commands) and `dumpScenario` writes one back. The parser is the platform's own `core::Json` (no dependency); the C ABI mirrors the calls (`fsim_scenario_*`). Scenarios describe the start of an experiment; the trainer's program is what happens next. `VecEnv` keeps its compact per-environment `env::Scenario` with sampled initial conditions.

## 10. Extensibility

Extension happens at four levels — data, SDK registration, built-in modules and (later) dynamic plugins — and every level reaches the same small set of core interfaces, so a task can start in the trainer's own code, be promoted to a built-in module, and be split into a plugin without changing the core.

### 10.1 Extension points

| Level | Mechanism | Examples | Requires platform rebuild |
| --- | --- | --- | --- |
| Data | `.vsgt`/JSON scenarios and manifests resolved through `io::AssetResolver`; JSBSim XML; tile pyramids | new aircraft (JSBSim XML + glTF + manifest), scenarios, IC distributions, sensor definitions, quality presets, training regions | no |
| SDK registration | trainer code implements `fsim::Controller`, `fsim::Behavior`, `fsim::Effect`, `fsim::comm::Codec`/`Protocol`/`Medium`, `fsim::Task`, `fsim::ObservationBuilder` or `fsim::ActionMapper` and registers it by id through the SDK | a new control loop or manoeuvre, sensor noise or damage model, a message format, curriculum, reward shaping, custom observation stacks | no (trainer rebuild only) |
| Registries | `core::Registry<T>` keyed by string id, populated in module `init()` | tasks, observation builders, action mappers, flight-model implementations, camera controllers, sensors, UI layers | yes (static) |
| Modules | `core::Module` interface (`init / start / update(phase) / stop / shutdown`), compiled in behind a CMake option, self-registering | `ext/env_server`, `ext/recorder`, `ext/web_dashboard`, `ext/lidar`, `ext/traffic` | yes |
| Dynamic plugins | same `Module` interface exported through `extern "C" Module* createModule()`, loaded from a plugin directory | third-party sensors or tasks without source access | no (post-v1) |

### 10.2 Core interfaces modules use

- `EventBus`: typed publish/subscribe (`bus.subscribe<VehicleLoaded>(...)`) for low-rate events: load, reset, pause, camera change, device hot-plug. Dispatch is on the render thread between frames; the sim thread posts, never dispatches.
- `PropertyStore`: string-addressed tree of scalars and strings mirroring the JSBSim tree plus application values (`/app/time-factor`, `/render/fps`). Handles are resolved once and read lock-free; this is how UI, telemetry, recorder and scripting reach any value without new APIs.
- `Registry<T>`: id → factory; unknown ids produce a logged error and a no-op default, never a crash.
- `JobSystem`: `submit(fn)` and `submitOnRender(fn)` for work that must land on the render thread.
- `Context`: what `init()` receives — the services above, the `vsg::Viewer` (may be null when headless), the asset resolver and configuration.

### 10.3 Stability rules

1. Headers under `include/fsim/` are the SDK; anything else is internal and may change without notice.
2. Modules never include each other's headers; they communicate through the bus, the property store or registries.
3. The core compiles with every `ext/*` option off, and CI builds that configuration.
4. `VehicleState` may only gain fields, appended, so recorders and network consumers stay compatible.

### 10.4 No scripting runtime

The platform embeds no scripting language. Behaviour that would be scripted elsewhere is either data (scenarios, JSBSim XML scripts and autopilots, which are used unchanged) or compiled C++ registered through the SDK. External orchestration — sweeps, curricula, evaluation — lives in the trainer program, which talks to the platform through the C++ SDK, the C ABI or the shared-memory server.

## 11. Build, dependencies and packaging

A single CMake super-project builds Windows x64 with MSVC 2022, resolves dependencies through a vcpkg manifest with pinned baseline, produces the `fsim` library (headers + DLL + import lib), two executables and the offline tile tool, and is released under MIT. As built: the toolchain is MSYS2 UCRT64 GCC (section 3; MSVC is not used) and there is no vcpkg - VSG, vsgXchange and vsgImGui are submodules built as static libraries by a CMake superbuild (`deps/`), JSBSim is a submodule built as a DLL by the main project, Catch2 is fetched at configure time and the rest comes from MSYS2 packages. The build also produces `fsim_vision.dll`, the examples and the Python package.

### 11.1 Repository layout

As built (2026-09-25; the README's Layout section says what each directory holds):

```
flightsim/
├─ CMakeLists.txt          options: FSIM_WITH_RENDER, FSIM_BUILD_TESTS, _EXAMPLES, _TOOLS, _PYTHON; presets in CMakePresets.json
├─ fsim.cmd                the front door: build, run, the demos, hangar, Python
├─ fetch-maps.cmd          the offline map tiles, into assets/maps
├─ cmake/                  toolchain, warnings, dependencies, assets, install and packaging
├─ deps/                   superbuild for the VSG stack (static)
├─ third_party/            submodules: JSBSim (DLL; its data tree too), VSG, vsgXchange, vsgImGui; stb
├─ include/fsim/           public C++ SDK headers + fsim_c.h (section 10.3)
├─ src/{platform,core,io,sim,control,effects,comm,ipc,session,env,sdk,vision,render,world,ui,app}/
├─ python/                 the Python SDK: C extension modules over the C ABI, the fsim package, tests, wheel
├─ tools/                  tile_prefetch (offline tile cache for a region); hangar (aircraft design, Python + a native mesher)
├─ aircraft/               designs made with hangar, and the JSBSim aircraft and glTF models it builds from them
├─ assets/                 viewer config and the offline map plan; maps/ (fetched, not in git)
├─ examples/               C++ trainers and tools, rust_trainer, python/, scenarios/ (JSON scenario files)
├─ tests/                  Catch2 unit + JSBSim integration tests, C ABI tests compiled as C, the package consumer
└─ docs/                   this document, hangar's guide, sdk/ (the SDK guide)
```

The first revision's `vcpkg.json`, `ext/`, top-level `shaders/` and `scenarios/` were not needed: dependencies are submodules and MSYS2 packages (above), `ext/`'s environment server became the world segment and the UDP comm bridges and its recorder the SDK's recording (section 15), the viewer's GLSL lives in its sources, and scenarios are JSON (9.13). `tools/tile_builder`, a scenario validator and an asset packer are not built.

### 11.2 Dependency budget

| Library | Configuration | Linked | Estimated release size |
| --- | --- | --- | --- |
| JSBSim (bundled expat) | headless + viewer | DLL (LGPL) | \~2.5 MB |
| VulkanSceneGraph (incl. `TileDatabase`, `DatabasePager`) | viewer | static | \~3 MB |
| vsgXchange (assimp glTF, KTX, stb\_image, curl) | viewer; `curl` + image readers also in headless for tile access | static | \~3–4 MB (assimp is the bulk; a glTF-only build cuts it to \~1 MB) |
| libcurl + zlib (via vsgXchange) | headless + viewer | static | \~0.7 MB |
| Dear ImGui + ImPlot (vsgImGui) | viewer | source | \~0.6 MB |
| Catch2 | tests | — | 0 |
| GDAL | `tools/tile_builder` only | separate exe | not shipped with the platform |
| Vulkan loader | system | `vulkan-1.dll` from the driver | 0 |

Headless `fsim.dll`: \~5 MB. Viewer executable: \~9–12 MB. Total install well under the 20 MB target; LTO and `/OPT:REF,ICF` on for release. M0 note: JSBSim is built from the submodule's own `src/` CMake tree (it must ship its data tree anyway) and Catch2 is fetched at configure time; the vcpkg manifest arrives with the VSG dependencies in M3.

### 11.3 Platform notes (Windows 10/11 x64)

| Topic | Decision |
| --- | --- |
| Toolchain | MSYS2 UCRT64: GCC 16.2, CMake ≥ 3.25, Ninja, MSYS2 pacman packages (vulkan-headers, vulkan-loader, glslang, spirv-tools, assimp, curl); toolchain file cmake/toolchains/ucrt64.cmake; LTO off (GCC LTO collides with dllexport vtables in JSBSim.dll) |
| CRT | dynamic UCRT, `/MD`; the C ABI carries no CRT types so trainers built with other compilers or runtimes can link `fsim.dll` |
| Distribution | zip with `bin/` (`fsim.dll`, `fsim_vision.dll`, `JSBSim.dll`, `flightsim.exe`, `flightsim-viewer.exe`, tools, the MinGW runtime), `include/fsim/`, `lib/` (import libraries), `share/flightsim/` (models, JSBSim data, scenarios), `share/doc/`, CMake package config for `find_package(fsim)` -> `fsim::sdk`, `fsim::vision`. Implemented in `cmake/Install.cmake` (`cmake --install`, `cpack` ZIP, ~27 MB); `tests/package_consumer` builds against it. As built since: `fsim dist` lays out five separate trees - `viewer` (standalone, with its maps), `sdk`, `python` (the package and a cp311-abi3 wheel), `tools`, `examples` - and the zip carries hangar's aircraft too, about 480 MB from CI (which has no maps) |
| GPU matrix | NVIDIA (RTX), AMD (RDNA), Intel Arc drivers tested in the viewer/vision CI job |
| Future Linux | no Win32 outside `platform/` (shared memory and semaphores get a POSIX implementation); every dependency supports Linux, so a port is a CI job plus `platform/linux` |

### 11.4 Continuous integration

GitHub Actions on `windows-2022`: configure and build headless and viewer configurations (Debug/Release); Catch2 suites; a 60-second headless golden-trajectory test per stock aircraft (tolerance 1e-6 relative); determinism test across worker counts; a throughput benchmark with regression gate (−10 % fails); C ABI test compiled as plain C; the `examples/minimal_trainer` built against the installed package; a 200-frame offscreen render with Vulkan validation layers on a software device (SwiftShader/Lavapipe) using a tiny bundled tile pyramid; binary size check against the budget.

As built (`.github/workflows/ci.yml`, on every push): one job on `windows-2022` with MSYS2 UCRT64, about 13-19 minutes. The VSG stack is built once and cached by its submodules' commits; the release configuration (viewer and headless together) is configured and built; `ctest` runs the Catch2 suites, the C ABI tests compiled as C, the Python SDK against a python.org CPython 3.12 and hangar's own tests; a headless smoke run and a 32-vehicle throughput benchmark follow (reported, not gated); then `cmake --install` and `cpack`, a smoke run of the installed copy, and the zip kept as the run's artifact (`flightsim-win64`) - a `v*` tag publishes it as a release. Not built yet: the Debug build, golden trajectories, the offscreen render test, the size check and the benchmark gate.

## 12. Performance design

Performance means training throughput: vehicle-steps per second with the trainer's loop in charge, then vision frames per second, then viewer frame rate. It comes from structure — lockstep batching, share-nothing vehicles, library-owned zero-copy buffers, no allocation in the step path — and from measurement built in from the first milestone.

### 12.1 Budgets and measurements

| Metric | Target | Measured by | Fails CI when |
| --- | --- | --- | --- |
| Headless throughput, state observations | ≥ 100,000 vehicle-steps/s on 16 physical cores (c172x); ≥ 60,000 (f16). Measured at M0: 1,138,000 (c172x, 16 workers), 757,000 (f16, 6 workers) | `flightsim.exe --benchmark`, CSV | −10 % vs. last release |
| `step()` overhead excluding JSBSim | < 2 µs per vehicle (task + observation assembly) | profiler scopes | > 4 µs |
| Worker scaling | ≥ 0.85 × linear from 1 to 16 workers | benchmark sweep | < 0.7 |
| Shared-memory server round trip | < 50 µs per step excluding simulation | benchmark with a dummy trainer process | > 100 µs |
| Vision observations | 64 × 128×128 RGB < 30 ms per step steady state on an RTX 4070-class GPU | GPU timestamps + readback timer | > 50 ms |
| Viewer frame time | < 8 ms CPU on the render thread with 256 vehicles and streaming terrain | frame profiler | tracked, not gated |
| Heap allocations in `step()` (steady state) | 0 | allocation counter in debug builds | > 0 |
| Memory | < 10 MB per JSBSim vehicle; tile cache within its configured limit | OS counters | > 15 MB per vehicle |
| Startup | headless `VecEnv(64)` ready < 2 s | wall clock | > 4 s |

### 12.2 Hot-path rules

- Vehicles are partitioned across workers once per `step()` in a deterministic order; workers write only their own `VehicleState` slots.
- Property access uses cached `FGPropertyNode*` handles resolved at load; no string lookups in `step()`.
- Observation and action buffers are preallocated per `VecEnv` and reused; the SDK returns spans and the C ABI returns pointers over the same memory, so no consumer ever receives a copy.
- Task evaluation runs vectorised over the snapshot batch (flat arrays), not per-vehicle virtual calls.
- Ground-height queries hit a per-worker tile cache (last tile + neighbours), so the common case is a bilinear sample with no lookup.
- Snapshots for viewers are published by index swap into a triple buffer; the render thread never blocks the caller.
- Sim workers are pinned to physical cores; the render thread and job pool use the remaining ones.

### 12.3 GPU-side

Vision sensors render into one image array with one command buffer and one submit per step, read back through a single copy into a persistently mapped host buffer. Vehicle models share geometry (instancing per model type); `DatabasePager` limits (max tiles per frame, memory budget) are set per quality tier so streaming does not stall the step, and training regions are prefetched. Reverse-Z depth handles the 2 m–200 km range of a full-Earth scene.

### 12.4 Scaling

One process scales to roughly `physical_cores × 8` vehicles before worker overhead dominates; beyond that, the trainer runs several `VecEnv` instances or several `--serve` processes and the design needs no change. A GPU-resident observation path (Vulkan external memory / CUDA interop) and asynchronous stepping (`stepAsync` / `stepWait`) are the two post-v1 optimisations already accommodated by the interfaces.

## 13. Cross-cutting concerns

Configuration, logging, errors and tests all follow one rule: no new dependency, one mechanism each, usable headless.

| Concern | Design |
| --- | --- |
| Configuration | One schema, versioned, stored as `.vsgt` or JSON; sources merged in order: built-in defaults → `config.vsgt` in `%LOCALAPPDATA%\flightsim` → `--config` / `VecEnvOptions` → command-line flags. Scenario files are separate from configuration (what to simulate vs. how to run) |
| Logging | `core::Log` with levels and per-module categories, lock-free ring buffer drained by a job-pool thread to stderr and a rotating file; JSBSim (`FGLogger`) and VSG (`vsg::Logger`) redirected into it; the SDK exposes a log callback so trainers can route records into their own logging |
| Errors | Exceptions only at load-time boundaries (scenario, JSBSim model, Vulkan device); `step()` never throws for simulation reasons — a diverged or crashed vehicle marks its environment `terminated` with an `info` code and auto-resets. The C ABI converts every exception to an error code. Vulkan validation layers in Debug; device-lost triggers a controlled viewer shutdown, the simulation continues |
| Determinism checks | CI replays a recorded episode with different `nWorkers` values and asserts bit-identical trajectories |
| Crash handling | Minimal Win32 crash handler writes the last 2 s of the log ring and the current snapshot batch to the config directory |
| Recording and replay | `ext/recorder` writes snapshots + actions at agent rate to a compact binary stream (`.fsrec`, VSG binary serialisation); replay feeds `KinematicModel`s, so the viewer shows recorded episodes through the same code path |
| Testing | Catch2 for `core`, `sim` conversions, `env` builders and the SDK; C ABI tests compiled as C; headless golden trajectories per stock aircraft; offscreen render + validation layers; throughput benchmark with regression gate |
| Licensing | Platform under MIT; `THIRD_PARTY_LICENSES.md` generated from vcpkg SPDX data at build time; JSBSim as a DLL satisfies LGPL-2.1 |
| Data access | Any property-store value can be observed, plotted or recorded without code changes |

## 14. Risks and mitigations

The largest risks are the ones the stack does not already solve: building and hosting the global tile pyramid, `vsg::TileDatabase`'s fitness at very low altitude, keeping the C ABI stable, and vision-observation cost.

| Risk | Likelihood | Impact | Mitigation |
| --- | --- | --- | --- |
| Global tile pyramid is large to build and host (30 m elevation worldwide ≈ tens of GB) | high | data logistics | 90 m global base + 30 m regional pyramids; `tools/tile_builder` is incremental; any static HTTP server or local disk serves tiles; public imagery servers for quick starts |
| `vsg::TileDatabase` shows seams, cracks or popping at low altitude / high LOD | medium | visual quality | tune `skirtRatio`, `lodTransitionScreenHeightRatio` and `maxLevel`; the tile subgraph is regenerated through `elevationLayerCallback`, so a custom mesh builder (finer subdivision near vehicles) is possible without leaving VSG |
| Physics ground vs. rendered mesh residual | low | visual only | same tiles for both; residual shown in debug layer; sub-metre by construction |
| C ABI drift breaks external trainers | medium | adoption | layout version in every buffer descriptor; ABI tests compiled as C in CI; additive changes only within a major version |
| Vision observations dominate step time | high when enabled | performance | shared image array, one submit per step, pager limits, region prefetch; GPU-resident path post-v1 |
| JSBSim instance memory at hundreds of vehicles | low | memory | measured per aircraft in M1; multiple processes above \~256 vehicles |
| JSBSim numerical divergence with extreme RL actions | medium | training stability | per-vehicle NaN/limit checks after each step → `terminated` with code, auto-reset; action rate limiting option |
| No Python means slower adoption by RL researchers | low since 2026-09-23 | adoption | the Python SDK (`import fsim`, Gymnasium and Stable-Baselines3 adapters) over the C ABI, at its speed: a 64-aircraft `VecEnv` step takes 375.4 us from Python, 375.3 us from C; `examples/` also show C++ and Rust trainers |
| Windows-only leaves Linux training clusters out | medium | adoption | strict `platform/` isolation; every dependency supports Linux; port planned as a post-v1 CI job |
| VSG API changes between 1.1.x releases | low | maintenance | pin by tag; quarterly upgrade with CI green |
| Dependency budget creeps | medium | size, build time | every new library needs an ADR and the size check must pass |

## 15. Roadmap and milestones

Six milestones take the project from an empty repository to a v1 release. Re-planned 2026-09-19 at the owner's direction: visualisation is the most important component and comes first (M0 skeleton, then the viewer), the RL API follows, vision observations and release last. Status (2026-09-20): M0 done; the viewer delivered (full-Earth satellite imagery over real relief, terrain physics, N vehicles, glTF models, chase/orbit/overview mouse cameras, sky, trails, labels, ImGui monitor, interpolated motion); M2 delivered (the Rust C ABI example, `examples/rust_trainer`, followed 2026-09-21; the PPO exit criterion is met by `examples/ppo_trainer`, a dependency-free C++ PPO that trains altitude/heading hold on the c172x through the SDK in ~2 minutes) (`env::VecEnv`, two tasks, `state`/`surfaces` spaces, `libfsim.dll` with the C++ SDK and C ABI v1, `examples/minimal_trainer` PD baseline at ~0.7 M vehicle-steps/s through the SDK, C ABI tested from C99). M2b delivered 2026-09-20: `World`/`Vehicle` object model, the six-level control stack with built-in loops and behaviours, real-time environment, effects and comm interfaces with v1 implementations, `ipc` world segment, viewer mirror mode with discovery, `VecEnv` re-based on the object model with actions at any level, world C ABI, `docs/sdk/`, `examples/multi_level_control`; 64 vehicles with cascades and effects at ~760k vehicle-steps/s, within a few percent with a viewer attached. Follow-ups delivered 2026-09-20: headless terrain physics in the SDK, recording/replay (`WorldOptions::recordPath`, `flightsim-viewer --replay` with timeline seek), per-type vehicle models in the viewer, scenario files (9.13), comm bridges over UDP (9.6) and `tools/tile_prefetch`. M4 (vision observations, 8.4) delivered 2026-09-21 as `fsim_vision.dll`: RGB + depth cameras on vehicles at 64 cameras per ~8 ms step, `BatchCameras` for VecEnv, a C ABI, the viewer's live camera preview through shared memory; the `ext/env_server` item is superseded by the world segment and the UDP comm bridges. M5 items done the same day: `cmake --install` + CPack zip + `find_package(fsim)`, third-party notices, crash handler, CI green with the package as an artifact and releases on `v*` tags. The 10.1 "SDK registration" row is complete for the environment layer since 2026-09-21: `fsim::Task`, `fsim::ObservationBuilder` and `fsim::ActionMapper` are public interfaces in `fsim/VecEnvPlugins.h`, registered by id through `registerTask()` / `registerObservation()` / `registerAction()`, and the built-ins are registered against the same interfaces. Each milestone ends with CI green and the section 12 metrics recorded. Durations assume one to two developers.

Status (2026-09-25), since then: the viewer's package carries its own maps and opens no sockets (2026-09-22, section 8.5). The Python SDK (2026-09-23, `python/`, [sdk/python.md](sdk/python.md)) offers the object model, the batch layer, cameras, recordings and scenarios through C extension modules over the C ABI, with Gymnasium and Stable-Baselines3 adapters, C++ plugins loaded from Python and one wheel for CPython 3.11+, at the C ABI's speed. hangar (2026-09-22/23, `tools/hangar`, [hangar.md](hangar.md)) turns a design file into a JSBSim aircraft with a glTF model and flight-tests it; a Cessna 172P rebuilt from public dimensions predicts its handbook's stall speed and ceiling within 4 %. With it came moving control surfaces, landing gear and doors, propellers, nozzles and wheels in the viewer and the vision cameras; fourteen fighters flown through their own fly-by-wire, the F-16C checked against NASA's wind-tunnel data (2026-09-23 to 25), with thrust vectoring on the F-22A and the Su-57 (2026-09-25); and fifteen support aircraft - bombers, tankers, transports, AEW&C, reconnaissance, electronic warfare and attack (2026-09-25). The viewer draws engine exhaust and airflow effects after DCS World from each vehicle's state (2026-09-25, [sdk/viewer.md](sdk/viewer.md#effects)). M5's release is still to come: no `v*` tag yet.

| # | Milestone | Deliverable | Exit criteria | Duration |
| --- | --- | --- | --- | --- |
| M0 | Skeleton | CMake + vcpkg project, `core` services, headless `Application`, CI on Windows | headless exe steps one JSBSim vehicle | 2 weeks |
| M1 | Multi-vehicle FDM + terrain data | `JsbsimModel`, `VehiclePool` with workers, unit conversions, `io::TilePyramid` + `GroundProvider`, `tools/tile_builder` (90 m global + one 30 m region), golden trajectories, recorder | 64 vehicles headless over real terrain; determinism across worker counts; throughput measured | 4 weeks |
| M2 | RL API | `env` layer (`VecEnv`, tasks, state observations, action mappers), `fsim` SDK + C ABI, install/export package, `examples/minimal_trainer`, benchmark mode | a C++ PPO (LibTorch example) trains altitude-hold on c172x through the SDK; the C ABI example steps from Rust; ≥ 100k vehicle-steps/s | 3 weeks |
| M2b | Vehicle SDK + transparent viewer | `World`/`Vehicle` object model, control stack (all six levels, built-in loops and behaviours), environment control, effects and comm interfaces with v1 implementations, `ipc` world segment, viewer mirror mode with discovery, `VecEnv` re-based on the object model, SDK docs (`docs/sdk/`), examples (`multi_level_control`, `dogfight_behaviors`) | a trainer creates vehicles and commands them at every level while the prebuilt viewer, started separately, mirrors them with no measurable change in trainer throughput | 4 weeks |
| M3 | Viewer | `render`, `world` with `vsg::TileDatabase` full-Earth from the pyramid, vehicle models driven by snapshots, cameras, ImGui monitor/vehicle list/telemetry, `attachViewer()` | watch 64 training vehicles anywhere on Earth at 60 fps without slowing `step()` | 4 weeks |
| M4 | Vision observations + server | offscreen sensor cameras, batched readback, `rgb`/`depth` observation types, sensor preview layer, `ext/env_server` shared-memory module | 64 × 128×128 RGB per step within budget; an out-of-process trainer steps through shared memory | 3 weeks |
| M5 | Release v1 | packaging (zip + CMake package), docs and examples, licence generation, crash handler | public MIT release; benchmarks within budget; SDK headers and C ABI v1 frozen | 3 weeks |

Post-v1 candidates, in priority order: Linux build, GPU-resident observations, asynchronous stepping, finer terrain meshing near vehicles, additional sensors (lidar/radar models), scripted traffic, TCP variant of the environment server, web dashboard, and a scenario editor.

## 16. Architecture decision records

Each major choice, its alternatives and the driver that decided it; status "accepted" reflects the project owner's decisions of 2026-09-19, the rest are proposed.

| ADR | Decision | Alternatives considered | Deciding driver | Status |
| --- | --- | --- | --- | --- |
| ADR-1 | C++ SDK (`fsim`) with a versioned C ABI is the primary interface; exe entry points share the same `Application` | Python bindings, gRPC service, C++-only without C ABI | owner decision (no Python), performance, extensibility | accepted; amended 2026-09-23 at the owner's request: Python bindings over the C ABI (the Python SDK) |
| ADR-2 | Lockstep, caller-driven stepping across a worker pool; no real-time clock in the training path | free-running sim thread with real-time clock | throughput, determinism | accepted |
| ADR-3 | One JSBSim instance per vehicle, share-nothing, JSBSim used as-is (stock aircraft, FCS, scripts) | custom FDM, shared JSBSim instance | owner decision, determinism | accepted |
| ADR-4 | Viewer = VSG native window + Dear ImGui (vsgImGui), optional module | Qt, SDL/GLFW window, RmlUi, web view | lightweight, single render pass | accepted |
| ADR-5 | No flight-stick input, HUD, instruments or cockpit view in v1 | SDL3 input module | owner decision | accepted |
| ADR-6 | Full-Earth rendering via VSG's own `vsg::TileDatabase` (imagery + elevation layers) from a self-hosted tile pyramid | vsgCs / Cesium 3D Tiles, own PagedLOD engine | owner decision (VSG mandatory, no Cesium), lightweight | accepted |
| ADR-7 | Physics ground from the same elevation tiles, sampled on the CPU by `io::TerrainTiles` (headless: WinHTTP + stb_image, disk cache shared with the renderer) | separate DEM source; height from rendered mesh | consistency, headless, determinism | accepted |
| ADR-8 | Vision observations via offscreen VSG views, batched, host readback | per-sensor windows, separate render process | performance | proposed |
| ADR-9 | VSG object model and serialisation (`.vsgt`) in `world`, `render`, `ui` and for scenarios/manifests; `core`, `sim` and `env` are plain C++ so the headless build has no VSG/Vulkan dependency (revised at M0) | own model, JSON everywhere | one model, VSG mandatory | proposed |
| ADR-10 | `FlightModel` interface hides JSBSim; SI + ECEF metres at the boundary | JSBSim types used directly | extensibility, testability | proposed |
| ADR-11 | ECEF double world frame; `TileDatabase` handles Earth precision; floating origin for vehicle subgraphs | flat local tangent plane | full-Earth correctness | proposed |
| ADR-12 | Windows x64 / MSYS2 UCRT64 (GCC) only for v1 (section 3; first written as MSVC 2022); Win32 confined to `platform/` | Linux from day one | owner decision | accepted |
| ADR-13 | vcpkg manifest as the dependency manager; static linking except JSBSim DLL and `fsim.dll` itself | `FetchContent`, system packages | Windows build reproducibility, LGPL | superseded: submodules (the VSG stack built static by the `deps/` superbuild, JSBSim as a DLL in the main build), Catch2 fetched at configure time, the rest from MSYS2 packages (section 11) |
| ADR-14 | MIT licence; JSBSim as DLL | GPL, Apache-2.0 | owner decision | accepted |
| ADR-15 | No scripting runtime anywhere; data + compiled C++ only | Lua/Python module | owner decision, lightweight | accepted |
| ADR-16 | Shaders compiled to SPIR-V at build time | runtime glslang | size, startup | proposed |
| ADR-17 | Optional shared-memory environment server for out-of-process trainers; no gRPC | gRPC, TCP-only | lightweight, latency | proposed |
| ADR-18 | Vehicle-instance SDK (`World`/`Vehicle`) is the primary API; `VecEnv` is a layer over it | gym-only API, scenario-file-only vehicle creation | owner workflow statement 2026-09-20 | accepted |
| ADR-19 | Visualisation mirrors the training process through a read-only shared-memory world segment written wait-free and rate-limited by the SDK; the viewer is a separate prebuilt process that discovers worlds | in-process viewer (`attachViewer()`), sockets/gRPC streaming, recording playback only | owner: transparent, independent, no training bottleneck | accepted |
| ADR-20 | Multi-level control stack with strictly ordered levels, one command variant, controllers that cascade to any lower level, behaviours as top-level controllers, registry for replacements | JSBSim autopilot XML only, flat "mode" switch per vehicle, separate APIs per level | owner: multi-level control, extensibility, cohesion | accepted |
| ADR-21 | Environment (time, atmosphere, wind, weather, ground) is a settable world-level model applied per step; per-vehicle disturbances are `Effect`s with a context that exposes wind, forces, properties and sensed state | scenario-file-only weather, per-vehicle environment objects | owner: real-time environmental control, abstract effects | accepted |
| ADR-22 | Communication as registry-backed abstractions (`Node`, `Message`, `Codec`, `Protocol`, `Medium`) with an ideal medium and a link model in v1 and transport bridges later | fixed MAVLink/UDP, no comm model | owner: abstract communication, future formats/protocols | accepted |
| ADR-23 | Scenario files as a JSON document parsed by the platform's own `core::Json` (no dependency), applied through the object model in a fixed order (environment, world effects, vehicles + effects, commands); the batch layer reads its own `vecenv` section | .vsgt files, a scripting language, YAML with a library | data over code for experiments; zero new dependencies; one loader for the SDK, the C ABI and VecEnv | accepted |
| ADR-24 | Vision observations in a separate library (`fsim_vision.dll`) over the public `World`: a headless `vsg::Viewer` with one framebuffer per camera in one command graph, cached host-visible readback, own-vehicle hiding by view masks, one Vulkan device per process | rendering inside `fsim.dll` (drags Vulkan into every trainer), a render service process with shared-memory images | the core SDK stays headless and dependency-free; trainers opt in; 64 cameras in ~8 ms meets the budget without a GPU-resident path | accepted |
| ADR-25 | Comm bridges as a `Protocol` on an external node over a `Transport` (UDP built in) with a fixed little-endian wire format, the peer becoming a node subject to the same medium | a special medium, MAVLink | keeps the medium/protocol split; any language can speak 36 bytes + payload | accepted |

## 17. Open questions

Owner decisions so far are recorded in section 1; the remaining questions below change scope or a proposed ADR.

- [x] Tile data: settled 2026-09-22 — public tiles (Esri World Imagery, AWS Terrain Tiles), cached on disk by the viewer and the SDK, and pre-fetched into the package for offline use by `fetch-maps` within the owner's 2.5 GB budget (8.5); `tools/tile_builder` has not been needed.
- [x] Trainer language: decided 2026-09-19 — C++. The first example trainer in M2 is C++ (\`examples/minimal\_trainer\`, then the dependency-free PPO); the Rust C ABI example (\`examples/rust\_trainer\`) landed 2026-09-21.
- [x] First tasks: M2 ships `altitude_heading_hold` (targets sampled per episode relative to the initial state) and `level_flight` on the stock c172x; waypoint following and pursuit–evasion are queued behind vision observations.
- [x] Vision in v1: delivered with M4 on 2026-09-21 (8.4).
- [ ] Linux timing: post-v1 (as planned) or before the public release, given that most training clusters run Linux?
- [x] Aircraft models: hangar (2026-09-22/23) writes a glTF model, with its moving parts, for every aircraft it designs - 31 in `aircraft/`; a stock JSBSim aircraft without a model draws the sample aircraft.

## Sources

- [VulkanSceneGraph repository](https://github.com/vsg-dev/VulkanSceneGraph) — platforms, C++17 requirement, dependencies, release 1.1.16
- [vsg/nodes/TileDatabase.h](https://github.com/vsg-dev/VulkanSceneGraph/blob/master/include/vsg/nodes/TileDatabase.h) — `TileDatabaseSettings` fields: `imageLayer`, `elevationLayer`, `detailLayer`, `ellipsoidModel`, `maxLevel`, `skirtRatio`, `lodTransitionScreenHeightRatio`, `createOpenStreetMapSettings()`, `createBingMapsSettings()`
- [vsgImGui repository](https://github.com/vsg-dev/vsgImGui) — ImGui and ImPlot integration, release 0.8.0
- [vsgXchange repository](https://github.com/vsg-dev/vsgXchange) — loader modules incl. curl and image readers
- [vsgCs repository](https://github.com/timoore/vsgCs) — considered and rejected by owner decision
- [vsgQt repository](https://github.com/vsg-dev/vsgQt) — Qt integration considered for tooling
- [JSBSim repository](https://github.com/JSBSim-Team/jsbsim) — FDM library, licence, release 1.3.1
- [SDL repository](https://github.com/libsdl-org/SDL) — release 3.4.16, considered and not used
- [GLFW repository](https://github.com/glfw/glfw) — release 3.5.1, considered and not used
- [Dear ImGui repository](https://github.com/ocornut/imgui) — release 1.92.9

Binary-size, latency and throughput figures in sections 4, 6, 11 and 12 are approximate estimates to be replaced by measurements in M1–M2.
