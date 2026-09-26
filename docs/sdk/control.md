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
(`ctx.world->vehicleState(id)`) as the world step began, whichever worker
steps which vehicle: the same calls fly the same trajectories with any number
of workers.

## Capabilities and activities

`#include <fsim/Capability.h>`: the contract layer of
[ADR-26](../control-architecture.md). `command()` is the per-step path and
flies as it always has. Beside it, a vehicle says what it offers, answers
every command at once, and keeps what it was told as an *activity* with a
state and an end:

```cpp
using namespace fsim::control;
for (const auto& c : v.capabilities())      // fsim.flight.attitude, fsim.guidance.hold, ...
    std::printf("%s (v%u)\n", c.id.c_str(), c.version);

CommandResult r = v.submit(VelocityCommand{.airspeedMs = 60, .verticalSpeedMs = 2});   // NEW
if (!r.accepted()) std::printf("refused: %s\n", reasonName(r.reason));
world.step();
world.activity(r.activity)->state;          // ActivityState::Active
world.update(r.activity, VelocityCommand{.airspeedMs = 60, .verticalSpeedMs = 0});    // UPDATE
world.cancel(r.activity);                   // CANCEL: the vehicle flies its neutral default
```

**Capabilities.** One per level (`fsim.flight.actuator` ... `fsim.flight.position`)
and one per registered behaviour (`fsim.guidance.hold`, ...; your own are
`user.guidance.<id>`). A descriptor gives its id and version, what it
takes (`kCommand`, `kUpdate`, `kCancel`), the level it enters at, whether it
completes (`Persistence::Terminating`: `waypoints`, `aerobatics`) and its
parameters. A level's parameters are its command struct's fields in order,
with units and this aircraft's range. A behaviour's are its `params`.
`capabilityStatus(id)` says whether it can be commanded now; a diverged
vehicle's are `TemporarilyUnavailable` until it is reset.

**Answers.** `submit` (NEW), `update` (UPDATE) and `cancel` (CANCEL) return a
`CommandResult` at once:

| Field | What it holds |
| --- | --- |
| `status` | `Accepted`, `Rejected` or `Canceled` |
| `reason` | why it was rejected (`reasonName()`: `unknown_capability`, `invalid_parameter`, `out_of_range`, `authority_held`, `activity_ended`, `not_updatable`, `wrong_command_type`, ...) |
| `activity` | the activity it made or addressed |
| `other` | the activity that holds the authority |
| `kClamped` | set in `flags` when a value was clamped |

**Updates and parameters.**
- `update` is the per-step path of an activity. It writes the new setpoint, checked like its NEW was, and allocates nothing. A behaviour's parameters are heap data, so a behaviour takes no UPDATE; a new target is a new `submit`.
- Range policies: by default a value outside its advertised range is clamped (`RangePolicy::Clamp`, flagged `kClamped`). `RangePolicy::Reject` refuses the command instead. A required field left at `kHold` (a position's latitude) is refused as `invalid_parameter`.

**Activities.**
- An activity is `Pending` until the next step has flown it, then `Active`.
- It ends in one of three ways:
  - `Completed` (`goal_reached`): a route's last point, a manoeuvre flown;
  - `Failed`: `target_lost` when a followed vehicle is removed, `diverged`;
  - `Canceled`: `requested` by CANCEL, or `preempted` by a newer command.
- While it runs, its record carries flags: an effector saturated, a setpoint clamped (`constraints` for the last step, `constraintsSeen` since it started).
- A completed or failed behaviour keeps flying its last output, a hold, until another command takes over. CANCEL hands the axes to the vehicle's neutral default instead: surfaces centred, throttle 0, as every vehicle starts.
- `ActivityId` is the vehicle's id in its high 32 bits and a per-vehicle count, so the same calls give the same ids. A vehicle remembers its 16 latest ended activities (`v.activities()`).

**Authority.** A command has a `Source` in `CommandOptions`:
- `Policy` (the default and what `command()` uses);
- `Autopilot`: a mode a policy must not silently override;
- `Override`: an operator or script.

A newer command replaces an activity of its own or a lower source, which ends `preempted`. A higher source's activity refuses it with `authority_held`. For now a command owns every axis. Owning roll apart from pitch and thrust (a policy banking while an autopilot holds the height) comes with step 3 of the ADR.

**From C and Python.** The C ABI has the same calls (`fsim_vehicle_submit`, `fsim_activity_update`, `fsim_activity_cancel`, the capability and activity queries; [c_abi.md](c_abi.md)), and Python has `vehicle.submit(Level.VELOCITY, airspeed_ms=60)`, which returns an `fsim.Activity` with `update`, `cancel` and `state` ([python.md](python.md)).

**What `command()` does now.**
- At the level the vehicle's own activity flies, a new setpoint updates it: the same few nanoseconds as before.
- Any other level, or a behaviour, starts a new activity with no range or availability checks, exactly as before.
- It returns `false` when the command is refused. That covers a behaviour nobody registered (until 2026-09-26 it was logged, ignored and reported as success; the C ABI now returns `FSIM_INVALID_ARGUMENT`, Python raises), or an axis an `Autopilot` or `Override` activity holds.

## Built-in loops and their gains

| Level | id | Output | Tunables (`controller->setParameter("name", v)`) |
| --- | --- | --- | --- |
| Attitude | `pid_attitude` | actuators | `roll.kp/ki/kd`, `roll.max_rate`, `roll.integral_limit`, `pitch.kp/ki/kd`, `pitch.integral_limit`, `pitch.trim`, `pitch.trim_lift`, `airspeed.kp/ki`, `airspeed.integral_limit`, `heading.gain`, `rudder.beta_gain`, `throttle.feedforward`; the schedule: `schedule.tas_ms`, `schedule.eas_ms`, `roll.eas_exponent`, `roll.tas_exponent`, `pitch.eas_exponent`, `pitch.tas_exponent` |
| Acceleration | `pid_acceleration` | actuators | `load_factor.kp/ki/kd`, `load_factor.integral_limit`, `load_factor.feedforward`, `load_factor.path_hold`, `roll_rate.kp/ki`, `roll_rate.integral_limit`, `roll_rate.feedforward`, `pitch.trim`, `pitch.trim_lift`, `longitudinal.kp/ki`, `rudder.beta_gain`, `throttle.feedforward`; the schedule as above with `load_factor.*` and `roll_rate.*` exponents |
| Velocity | `pid_velocity` | attitude | `vertical_speed.kp/ki`, `vertical_speed.integral_limit`, `vertical_speed.feedforward`, `vertical_speed.command_lag`, `vertical_speed.alpha_zero_lift`, `pitch.min`, `pitch.max`, `max_bank`, `schedule.tas_ms` |
| Position | `pid_position` | velocity | `altitude.gain`, `max_vertical_speed` |
| Actuator | `actuator` | actuators | - |

The defaults fly the stock JSBSim c172x, and every new parameter's default
leaves that flight as it was. `controller->parameter("name")` reads one back
(C: `fsim_vehicle_controller_parameter`, Python:
`v.controller_parameter(Level.ATTITUDE, "pitch.kp")`).

What the others do:

- **The schedule.** At the reference true and equivalent airspeeds
  (`schedule.tas_ms`, `schedule.eas_ms`) a loop's gains are as set;
  elsewhere a channel's are multiplied by
  (eas_ref / eas)^a (tas_ref / tas)^b, where the aircraft's response to that
  control grows as eas^a tas^b - so the loop keeps its speed and its damping
  over the envelope (the calibrated airspeed stands in for the equivalent;
  the factor stays within 0.2 - 5). The heading gain grows with tas (a bank
  turns the heading at g tan(bank) / tas) and the vertical-speed gains fall
  with it (a pitch change moves the vertical speed by tas times as much).
  `schedule.tas_ms` 0 - the default - turns it off.
- **Feedforwards.** `pitch.trim` and `pitch.trim_lift` give the elevator
  that holds a surface-controlled aircraft in level flight at the reference
  speed and the part of it that goes with lift: the loops feed
  trim - lift + lift n (eas_ref / eas)^2 forward (n that of a level turn at
  the current bank, or the one commanded), so the integrator only trims what
  is left. `load_factor.feedforward` is the stick per g beyond what neutral
  stick gives - cos(pitch) cos(roll) when `load_factor.path_hold` is 1 (a
  fly-by-wire law that holds its flight path), 1 g when it is 0;
  `roll_rate.feedforward` the aileron per rad/s. With
  `vertical_speed.feedforward` 1 the pitch is the flight-path angle the
  vertical speed needs, asin(vz / tas), plus the angle of attack the wing
  would fly at 1 g, alpha0 + (alpha - alpha0) / n about its zero-lift angle
  `vertical_speed.alpha_zero_lift` (so a pull does not feed itself).
  `vertical_speed.command_lag` (s) follows a commanded vertical speed
  through a first-order lag, starting from the one flown, so a step asks for
  a climb the aircraft can enter without overshoot.
- A loop that missed a period - the stack flew another level meanwhile -
  starts again from the state (its bank setpoint, its lagged command).

## Per-aircraft gains

An aircraft can carry its own gains for the built-in loops: JSBSim
properties `fsim/control/<controller id>/<parameter>`, the parameter's dots
written as slashes, declared in its flight control section:

```xml
<flight_control name="f16c">
  <property value="1.11952">fsim/control/pid_attitude/pitch/kp</property>
  <property value="163.54">fsim/control/pid_attitude/schedule/tas_ms</property>
  ...
```

Every vehicle of that type gets them when it is created, and again whenever
the stack creates the controller by id (`use(level, "pid_attitude")`); an
instance you hand to `use()` is left as it is, and your own
`setParameter` wins. The aircraft designed with hangar all carry a set,
tuned for each by flight test ([hangar.md](../hangar.md#the-autopilot)); a
stock JSBSim aircraft flies the shared defaults. A setting no built-in
controller takes is logged once when the aircraft first loads, and kept for
a controller registered under that id.

![The same velocity commands with the shared gains and with each aircraft's own](../images/control-gains-before-after.png)

The same commands - hold speed, height and heading, climb, turn 90° - with
the shared gains (red) and each aircraft's own (blue): with the shared ones
the F-16C and the B-52H never stop oscillating in pitch, and the Su-25
departs in the turn.

Retuning or replacing them in code:

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
    bool finished() const noexcept override { return false; }       // true: its activity completes
    fsim::control::Reason failure() const noexcept override {         // not None: its activity fails
        return lost_ ? fsim::control::Reason::TargetLost : fsim::control::Reason::None;
    }
    std::uint32_t target_ = 0; double radius_ = 800.0; bool lost_ = false;
};
fsim::control::BehaviorTraits traits;   // optional: how consumers see it (user.guidance.orbit_target)
traits.parameters = {{"radius_m", "m", 100.0, 1e5, 800.0, true}};
traits.uses = {"fsim.flight.position"};
traits.needsTarget = true;              // submit() refuses it without a target
fsim::control::ControllerRegistry::instance().addBehavior("orbit_target", [] { return std::make_unique<Orbit>(); }, traits);
v.command(fsim::control::BehaviorCommand{.id = "orbit_target", .target = other.id()});
```

## RL at any level

Because actions are just commands, one policy can act on surfaces, attitudes
or velocities by changing which command you build; hierarchical RL maps to
the stack directly (a high-level policy commands `Position`, a low-level
policy owns the `Attitude` controller). The batch layer exposes this as the
`"surfaces"`, `"attitude"`, `"acceleration"` and `"velocity"` action spaces
([vecenv.md](vecenv.md)).
