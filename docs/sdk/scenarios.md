# Scenario files

`#include <fsim/Scenario.h>`

A scenario is a JSON document that describes an experiment's starting point
as data: the world options, the environment, the vehicles to create, the
command each starts with and the effects attached to them. Loading one and
applying it to a `World` replaces the setup code of a trainer, so the same
program can run any number of scenarios and a scenario can be shared,
versioned and reviewed like a config file.

```cpp
fsim::Scenario scenario = fsim::loadScenario("formation_and_pursuit.json"); // throws fsim::Error "<file>:<line>:<col>: ..."
fsim::World world(scenario.world);
std::vector<fsim::Vehicle> vehicles = fsim::applyScenario(world, scenario);
for (;;) world.step();
```

`examples/scenario_runner` is exactly this loop with `--realtime`, and
`examples/scenarios/*.json` are ready-to-run documents.

## Format

Every key is optional except a vehicle's `name`. Units are SI and degrees;
comments (`//`, `/* */`) and trailing commas are accepted.

```jsonc
{
  "world": {                       // fsim::WorldOptions
    "name": "scenario", "dt": 0.008333, "frame_skip": 4, "workers": 0, "pin_workers": true,
    "seed": 7, "capacity": 64, "publish": true, "publish_interval_s": 0.0167,
    "jsbsim_root": "", "terrain": false, "terrain_url": "", "terrain_zoom": 12,
    "record_path": "", "record_interval_s": 0
  },
  "environment": {                 // applied before the vehicles exist
    "utc": "2026-06-21T09:30:00Z", // or "unix_seconds": 1782034200; absent = the world's clock (now)
    "time_factor": 1,
    "wind":       { "direction_deg": 300, "speed_ms": 6, "gust_ms": 0, "turbulence": 0.15 },
    "atmosphere": { "temperature_sea_level_k": 288.15, "pressure_sea_level_pa": 101325, "humidity": 0 },
    "weather":    { "visibility_m": 30000, "cloud_base_m": 2000, "cloud_cover": 0.3, "precipitation": 0 }
  },
  "effects": [                     // every vehicle, present and future (World::addEffectToAll)
    { "id": "gaussian_sensor_noise", "position_sigma_m": 2, "altitude_sigma_m": 1 }
  ],
  "vehicles": [
    {
      "name": "echelon",           // required; with "count" > 1 the instances are "echelon-1", "echelon-2", ...
      "type": "jsbsim:c172x",
      "model": "",                 // optional glTF for the viewer
      "control_divider": 1,
      "count": 4, "spacing_m": 60, // instances abreast, to the right of the heading
      "initial": { "lat_deg": 37.62, "lon_deg": -122.40, "alt_msl_m": 1500, "heading_deg": 90,
                   "pitch_deg": 0, "roll_deg": 0, "airspeed_ms": 60, "on_ground": false },
      "command": { "level": "attitude", "roll_deg": 0, "pitch_deg": 2, "airspeed_ms": 60 },
      "effects": [ { "id": "sensor_latency", "delay_steps": 12 } ]
    }
  ]
}
```

### Commands

`command.level` selects the struct; every other key is a field of it and an
absent field keeps the struct's default (`kHold` where the field is optional,
see [control.md](control.md)). Angles are degrees in the file and radians in
the API.

| Level | Keys |
| --- | --- |
| `actuator` | `aileron`, `elevator`, `rudder`, `throttle`, `flaps`, `gear_down`, `brake_left`, `brake_right` |
| `attitude` | `roll_deg`, `pitch_deg`, `heading_deg`, `max_bank_deg`, `throttle`, `airspeed_ms` |
| `acceleration` | `load_factor_g`, `roll_rate_deg_s`, `longitudinal_ms2`, `throttle` |
| `velocity` | `airspeed_ms`, `vertical_speed_ms`, `heading_deg`, `turn_rate_deg_s` |
| `position` | `lat_deg`, `lon_deg`, `alt_msl_m`, `airspeed_ms`, `capture_radius_m` |
| `behavior` | `id` (`hold`, `waypoints`, `loiter`, `pursuit`, ...), `target` (another vehicle's **name**), `params` `{ ... }`, `points` `[ { position keys } ]` |

Behaviour targets are resolved by name after every vehicle in the file has
been created, so a chaser may be listed before its target.

### Effects

`{ "id": <built-in effect id>, <parameter>: <number>, ... }` with the ids and
snake_case parameters of `effects::createBuiltinEffect`
([environment.md](environment.md#effects)): `gaussian_sensor_noise`,
`sensor_latency`, `constant_force`, `wind_gusts`, `gnss_degradation`. Effects
written in trainer code are attached after `applyScenario` as usual.

## API

| Call | Meaning |
| --- | --- |
| `loadScenario(path)` | read and parse; relative `jsbsim_root` / `record_path` are taken relative to the file |
| `parseScenario(text, source)` | parse a document held in memory |
| `applyScenario(world, scenario)` | environment, world-wide effects, vehicles (with their effects), then commands; returns the vehicles in file order; throws `fsim::Error` on an unknown type, effect or target (vehicles created so far remain) |
| `dumpScenario(scenario)` | the document back as compact JSON (a `Scenario` can be built or edited in code) |
| `Scenario::world` | the `WorldOptions` to construct the `World` with; adjust in code before constructing (e.g. `publish = false` for a sweep) |

The C ABI mirrors it: `fsim_scenario_load/parse/destroy`,
`fsim_scenario_world_options`, `fsim_scenario_vehicle_count`,
`fsim_scenario_apply` ([c_abi.md](c_abi.md)).

Scenarios describe the **start**; what happens next is the trainer's program.
`VecEnv` keeps its own compact per-environment scenario (`env::Scenario`) for
batched episodes with sampled initial conditions.
