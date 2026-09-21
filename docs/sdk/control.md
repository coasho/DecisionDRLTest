# Multi-level control

`#include <fsim/Control.h>` (commands, interfaces), `<fsim/ControlStack.h>`,
`<fsim/ControllerRegistry.h>`, `<fsim/BuiltinControllers.h>`.

Every vehicle owns a **control stack**: six strictly ordered levels, one
controller per level, and one *active command*. You command a level; each
step the stack runs the cascade from that level down to the actuators. Nothing
above the active level runs.

```
Behavior   pursue / loiter / waypoints / formation / evade / aerobatics / hold / yours
  -> Position   fly to lat/lon/alt at an airspeed
    -> Velocity   airspeed, vertical speed, heading or turn rate
      -> Acceleration   load factor, roll rate, longitudinal acceleration
        -> Attitude   roll, pitch, heading, throttle or airspeed hold
          -> Actuator   aileron, elevator, rudder, throttle, flaps, gear, brakes
            -> flight model
```

## Choosing a level: one line

```cpp
using namespace fsim::control;
v.command(ActuatorCommand{.aileron = 0.1, .elevator = -0.05, .throttle = 0.7});
v.command(AttitudeCommand{.rollRad = 0.3, .pitchRad = 0.05, .airspeedMs = 60});
v.command(AttitudeCommand{.headingRad = 1.57, .maxBankRad = 0.5, .throttle = 0.8});   // heading mode
v.command(AccelerationCommand{.loadFactorG = 3.0, .rollRateRadS = 0.0, .throttle = 1.0}); // pull 3 g
v.command(VelocityCommand{.airspeedMs = 65, .verticalSpeedMs = 2.5, .headingRad = 0.0});
v.command(PositionCommand{.latitudeRad = lat, .longitudeRad = lon, .altitudeMslM = 1800, .airspeedMs = 60});
v.command(BehaviorCommand{.id = "pursuit", .target = other.id(), .params = {{"range_m", 300}}});
v.activeLevel();          // Level::Behavior
```

(Designated initialisers are C++20; in a C++17 trainer set the fields one by
one - the structs are plain aggregates with defaults.) Commands are plain
structs. A field set to `kHold` (NaN) means "keep the
current value / let the controller decide": `AttitudeCommand{.rollRad = 0.2}`
holds the current pitch, `VelocityCommand{.headingRad = kHold}` flies wings
level, a `throttle` of `kHold` keeps the last throttle. The last command
persists until replaced; a finished behaviour keeps producing its final
output.

| Command | Fields (defaults) |
| --- | --- |
| `ActuatorCommand` | `aileron`, `elevator` (+nose down, JSBSim), `rudder` (-1..1), `throttle` (0..1), `flaps` (0..1), `gearDown` (hold), `brakeLeft`, `brakeRight` |
| `AttitudeCommand` | `rollRad`, `pitchRad`, `headingRad` (hold; when set, roll follows the heading error within `maxBankRad`), `throttle` (hold), `airspeedMs` (hold; when set, throttle holds this speed) |
| `AccelerationCommand` | `loadFactorG` (1), `rollRateRadS` (0), `longitudinalMs2` (hold), `throttle` (hold). Works through any attitude, so it flies loops and rolls |
| `VelocityCommand` | `airspeedMs` (hold), `verticalSpeedMs` (0), `headingRad` (hold) or `turnRateRadS` (hold) |
| `PositionCommand` | `latitudeRad`, `longitudeRad`, `altitudeMslM`, `airspeedMs` (hold), `captureRadiusM` (200) |
| `BehaviorCommand` | `id`, `target` (vehicle id), `params` (name -> number), `points` (route) |

## Built-in behaviours

| id | Uses | Parameters (defaults) |
| --- | --- | --- |
| `hold` | - | `airspeed_ms`, `heading_deg`, `altitude_m` (all: current at start) |
| `waypoints` | `points` | `loop` (0), `airspeed_ms` (current) for points without one. `finished()` after the last capture |
| `loiter` | `target` or `lat_deg`/`lon_deg` | `radius_m` (1500), `altitude_m` (current), `clockwise` (1), `airspeed_ms` (current) |
| `pursuit` | `target` | `range_m` (300), `lead_s` (2), `min_airspeed_ms` (30), `max_airspeed_ms` (400) |
| `evade` | `target` | `altitude_delta_m` (-300), `airspeed_ms` (current) |
| `formation` | `target` (leader) | `ahead_m` (-100), `right_m` (60), `below_m` (0), `closure_gain` (0.1) |
| `aerobatics` | - | `manoeuvre` (0 aileron roll, 1 loop, 2 Immelmann, 3 split-S), `load_factor_g` (3.5), `roll_rate_rad_s` (1.5); `finished()` when done, then holds the entry altitude/heading |

Behaviours that need another vehicle read it through the world view
(`ctx.world->vehicleState(id)`), one step behind, deterministically.

## Built-in loops and their gains

| Level | id | Output | Tunables (`controller->setParameter("name", v)`) |
| --- | --- | --- | --- |
| Attitude | `pid_attitude` | actuators | `roll.kp/ki/kd`, `pitch.kp/ki/kd`, `airspeed.kp/ki`, `heading.gain`, `rudder.beta_gain`, `throttle.feedforward`, `roll.max_rate` |
| Acceleration | `pid_acceleration` | actuators | `load_factor.kp/ki/kd`, `roll_rate.kp/ki`, `longitudinal.kp/ki`, `rudder.beta_gain`, `throttle.feedforward` |
| Velocity | `pid_velocity` | attitude | `vertical_speed.kp/ki`, `max_bank` |
| Position | `pid_position` | velocity | `altitude.gain`, `max_vertical_speed` |
| Actuator | `actuator` | actuators | - |

Defaults fly the stock JSBSim c172x; faster aircraft usually want lower
attitude gains and a higher `max_bank`.

```cpp
auto& stack = v.controls();
stack.controller(Level::Attitude)->setParameter("roll.kp", 1.2);
stack.use(Level::Velocity, "my_velocity_loop");     // by registry id
stack.use(Level::Attitude, std::make_unique<MyAttitude>());
const Command* d = stack.derived(Level::Attitude);   // what the cascade produced at that level last step
```

`derived(level)` gives every intermediate command of the last cascade: use it
for telemetry, for observations (e.g. the attitude your velocity policy
implied), or to debug a behaviour.

## Writing a controller

A controller accepts a command at its level and returns a command at **any
strictly lower** level; the stack keeps cascading from there.

```cpp
struct BangBangAttitude final : fsim::control::Controller {
    const char* id() const noexcept override { return "bang_bang_attitude"; }
    fsim::control::Level level() const noexcept override { return fsim::control::Level::Attitude; }
    fsim::control::Command update(const fsim::control::ControlContext& ctx, const fsim::control::Command& in) override {
        const auto& c = std::get<fsim::control::AttitudeCommand>(in);
        fsim::control::ActuatorCommand out;
        out.aileron = c.rollRad > ctx.sensed.eulerRad[0] ? 0.5 : -0.5;
        out.throttle = fsim::control::orHold(c.throttle, 0.7);
        return out;
    }
};

fsim::control::ControllerRegistry::instance().add("bang_bang_attitude", fsim::control::Level::Attitude,
                                                  [] { return std::make_unique<BangBangAttitude>(); });
v.use(fsim::control::Level::Attitude, "bang_bang_attitude");
```

`ControlContext` gives you the vehicle id, truth `state`, `sensed` state, the
period `dt`, the `world` view (other vehicles, environment, time) and the
vehicle's deterministic `rng`. Override `reset()` to clear integrators and
`setParameter`/`parameter` to expose gains.

## Writing a behaviour

```cpp
struct Orbit final : fsim::control::Behavior {
    const char* id() const noexcept override { return "orbit_target"; }
    void start(const fsim::control::ControlContext&, const fsim::control::BehaviorCommand& c) override {
        target_ = c.target; radius_ = c.param("radius_m", 800.0);
    }
    fsim::control::Command update(const fsim::control::ControlContext& ctx, const fsim::control::Command&) override {
        const auto* t = ctx.world ? ctx.world->vehicleState(target_) : nullptr;
        fsim::control::PositionCommand p;   // or a VelocityCommand, AttitudeCommand, ... any lower level
        // ... aim at a point on the circle around t ...
        return p;
    }
    bool finished() const noexcept override { return false; }
    std::uint32_t target_ = 0; double radius_ = 800.0;
};
fsim::control::ControllerRegistry::instance().addBehavior("orbit_target", [] { return std::make_unique<Orbit>(); });
v.command(fsim::control::BehaviorCommand{.id = "orbit_target", .target = other.id()});
```

## RL at any level

Because actions are just commands, one policy can act on surfaces, attitudes
or velocities by changing which command you build; hierarchical RL maps to
the stack directly (a high-level policy commands `Position`, a low-level
policy owns the `Attitude` controller). The batch layer exposes this as the
`"surfaces"`, `"attitude"`, `"acceleration"` and `"velocity"` action spaces
([vecenv.md](vecenv.md)).
