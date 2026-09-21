# fsim SDK guide

The `fsim` SDK is how a training application drives the platform. You write a
C++ program (or any language with a C FFI, through `fsim_c.h`), link
`libfsim.dll`, create a **World**, create **vehicles** by name, type and initial
state, command them at whichever **control level** your experiment needs, and
step the world. Visualisation is somebody else's job: start
`flightsim-viewer.exe` at any time and it mirrors your world through shared
memory without your program noticing (see [viewer.md](viewer.md)).

| Document | Contents |
| --- | --- |
| [world.md](world.md) | `World`, `Vehicle`, `VehicleSpec`, stepping, time, state and sensed state |
| [control.md](control.md) | the multi-level control stack: levels, commands, built-in loops and behaviours, writing your own |
| [environment.md](environment.md) | real-time environment control, effects (disturbances, sensor models), communication |
| [viewer.md](viewer.md) | transparent visualisation: how the prebuilt viewer discovers and mirrors a world |
| [scenarios.md](scenarios.md) | scenario files: the world, environment, vehicles, commands and effects as a JSON document |
| [vecenv.md](vecenv.md) | the batch (gym-style) layer for vectorised RL, built on the same object model |
| [c_abi.md](c_abi.md) | the C ABI for other languages |

## Ten-line tour

```cpp
#include <fsim/World.h>
#include <fsim/BuiltinEffects.h>

int main() {
    fsim::World world({.name = "my-experiment"});            // published for viewers under this name
    world.environment().setWind({.directionDeg = 270, .speedMs = 8, .turbulence = 0.2});

    fsim::VehicleSpec spec;
    spec.name = "red-1"; spec.type = "jsbsim:c172x";
    spec.initial.latitudeDeg = 37.62; spec.initial.longitudeDeg = -122.38;
    spec.initial.altitudeMslM = 1500; spec.initial.headingDeg = 90; spec.initial.airspeedTrueMs = 60;
    fsim::Vehicle red = world.createVehicle(spec);

    red.command(fsim::control::VelocityCommand{.airspeedMs = 65, .verticalSpeedMs = 2, .headingRad = 0.0});
    red.addEffect<fsim::effects::GaussianSensorNoise>();

    for (int k = 0; k < 3000; ++k) {                          // 100 s at the default 30 Hz world step
        world.step();
        const auto& s = red.state();                          // truth
        const auto& z = red.sensed();                         // through the sensor effects
        // ... your learner reads z, writes red.command(...) at any level ...
    }
}
```

## Building against the SDK

- Headers: `include/fsim/*.h` (C++17). Only these are stable; anything under `src/` may change.
- Library: `libfsim.dll` (+ import library) and `libJSBSim.dll` next to your executable.
- Data: the JSBSim aircraft tree is found automatically next to the executable (`share/jsbsim`) or in the source tree; override with `WorldOptions::jsbsimRoot`.
- CMake in this repository: `target_link_libraries(my_trainer PRIVATE fsim::sdk)` (see `examples/`).

## Conventions

- SI units, radians in the C++ API (degrees only where a field says `Deg`), WGS-84 geodetic latitude/longitude, altitudes above the ellipsoid ("MSL" in JSBSim's sense), body frame x forward / y right / z down, NED velocities.
- `fsim::control::kHold` (NaN) in an optional command field means "keep the current value / let the controller decide".
- Every `World` call runs on the caller's thread; `step()` is the only call that does work. Nothing is asynchronous, nothing is hidden.
- Load-time failures throw `fsim::Error`; runtime calls return `bool`/handles and log through the platform logger.
- Determinism: same seed, same calls, same trajectories, for any number of worker threads.
