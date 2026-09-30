# ADR-27: Rotorcraft in the capability architecture

| | |
| --- | --- |
| Status | Accepted and implemented 2026-09-26, from the owner's request of that day (section 10); the 3D models rebuilt from three-views the same day, at the owner's request |
| Extends | ADR-26 ([control-architecture.md](control-architecture.md)), which stays in force |
| Scope | Four real rotorcraft - two quadrotors, two conventional helicopters - built from published data, and what the control architecture, hangar and the viewer needed to fly and show them without fixed-wing assumptions |
| Related | [hangar.md](hangar.md#rotorcraft) (the pipeline); [sdk/control.md](sdk/control.md#rotorcraft); [sdk/viewer.md](sdk/viewer.md) |

## 1. Context

### 1.1 The request

The owner asked for four real, well-known rotorcraft - two quadrotors and two
conventional helicopters - with public specifications, flight-dynamics data
and open-source simulation references, the data verified before the models
were chosen. They are to test the capability-oriented control architecture
(ADR-26): every fixed-wing assumption they expose is to be resolved by a
reusable abstraction, not an aircraft-specific workaround, keeping every
existing aircraft, determinism and performance as they are. Then: their 3D
models were to be rebuilt as the fighters were - from three-views, with
materials and textures - their axes bound correctly, and the quadrotors'
propellers were to turn, driven by what the SDK's user code commands.

### 1.2 What assumes a fixed wing

An audit of the platform before any change. The core of ADR-26 - the axes, the
slots, the activities, the runtime's cascade - names no aircraft; the
assumptions sat at its edges.

| Where | Assumption | Consequence for a rotorcraft |
| --- | --- | --- |
| `VehicleProfile` identity (`include/fsim/VehicleProfile.h`) | `AircraftClass` and `ControlFamily` know surfaces and fly-by-wire laws only | no adapter for cyclic and collective, or for a mixer and motors |
| effectors section | pitch is a surface, a load factor or a pitch rate; yaw a surface or a sideslip; nothing says what the thrust control is | collective, cyclic, pedals and differential thrust have no names |
| plant section | load factor per elevator, sideslip per rudder, an elevator trim law, a zero-lift angle of attack | a rotorcraft's responses are rates per control and heave per collective, trimmed in the hover |
| envelope and performance sections | a stall speed, alpha limits, a ceiling | hover ceilings and airspeed near zero are normal flight |
| `EngineType` | no turboshaft | - |
| `AccelerationCommand` (`include/fsim/Control.h`) | load factor, roll rate, longitudinal acceleration: pitch follows the load factor and yaw is coordinated | a rotorcraft pitches and yaws independently of its thrust: body rates and collective thrust cannot be commanded |
| `VelocityCommand` | airspeed along the heading, a vertical speed, a heading or turn rate | no sideways flight, no ground velocity held against wind, the nose tied to the track |
| `PositionCommand` | fly to a point and pass it | nothing to hover at, no heading to face there |
| built-in loops (`src/control/Builtin.cpp`) | heading by bank, speed by throttle, height by pitch, load factor by elevator | reversed or meaningless for a rotorcraft: height by collective, speed by pitch, heading by yaw |
| laws designed from the plant (`src/control/Laws.cpp`) | fixed-wing responses and schedules | none designable |
| capability catalog (`src/control/Catalog.cpp`) | actuator parameters named for surfaces; the per-engine capability owns thrust only; every parameter supported by every aircraft | a multirotor's motors own all four axes; a fixed wing would accept rotor-only fields silently |
| authority groups (`src/control/Adapter.*`) | roll and yaw fly together (coordinated), pitch apart, thrust apart | a helicopter's cyclic flies roll and pitch together, its pedals the yaw apart |
| the vehicle default's hold | keeps a heading with the roll, a height with the pitch, an airspeed with the thrust | a rotorcraft keeps its heading with the yaw, its height with the thrust, its velocity over the ground with the cyclic |
| envelope protection | the wing's limits (alpha, load factor at speed) | nothing wingborne to protect in a hover |
| behaviours | `hold` keeps an airspeed | no hover |
| hangar (`tools/hangar`) | aerodynamics from lifting surfaces, fixed-wing flight tests, models whose smallest cell is 2 mm and whose joints are blended over 2-3 cm | no rotorcraft can be built or tested; a 9 cm quadrotor cannot be meshed |
| the viewer | a chase camera no nearer than 6 m, a near plane at 0.36 m; propellers turned at the engine's rpm | a 9 cm aircraft is a speck; a rotor's rpm is not an engine's |
| C ABI (`include/fsim/fsim_c.h`) | command structs without a size field | new fields need new functions (C3) |

What needed nothing: `ControlInputs` (its four channels are the roll, pitch,
yaw and thrust controls of any aircraft - section 3.2), and the runtime's
cascade and host.

## 2. The four aircraft

Chosen after the data were read, not before. Every number the flight models
use is cited in their design files (`aircraft/<name>/<name>.toml`); section 9
lists the sources.

| | Bitcraze Crazyflie 2.0 (`cf2`) | 3DR IRIS+ (`iris`) | Bell UH-1H Iroquois (`uh1h`) | Sikorsky UH-60A Black Hawk (`uh60`) |
| --- | --- | --- | --- | --- |
| Kind | nano quadrotor, brushed coreless motors | quadrotor, brushless motors | single main rotor, 2 blades, teetering, Bell stabilizer bar | single main rotor, 4 blades, articulated, canted tail rotor, stabilator |
| Mass | 27 g | 1.5 kg (PX4's model; 1,282 g as sold) | 2,793 kg (6,158 lb, as flight-tested) | 7,439 kg (16,400 lb) |
| Flight dynamics | Förster 2015 (ETH), as tabulated by Luis & Le Ny 2016: inertia, arm, thrust and torque coefficients, PWM-rpm map; motor lag from Kooi & Babuška 2021; rotor drag and ground effect from gym-pybullet-drones | PX4's Gazebo model: inertia, rotor positions and senses, rotor constants, motor time constants up and down, rotor drag | NASA TM-73254: rotor, stabilizer bar, fuselage and tail constants, inertias; flight records at hover and 60 kt | NASA TM-85890 (GENHEL): rotor, tail, fuselage regressions, controls, mixing, stabilator, T700 engine and governor; trim 1 to 140 kt; TM-84281's tail surfaces |
| 3D model from | Bitcraze's drawing of the board from the top; its product photograph | PX4's own model of it (`models/iris`, its mesh measured) | TM 55-1520-210-10's principal dimensions (three-view, public domain) | TM 1-1520-237-10's principal dimensions (side and front views) and FM 44-80's three-view (both public domain) |
| Open-source reference | gym-pybullet-drones `cf2x` (MIT) | PX4-SITL Gazebo `iris` (BSD-3-Clause) | FlightGear UH-1 (GPL); JSBSim's `ah1s` | FlightGear UH-60 (GPL) |

**Why these.** Two quadrotors three orders of magnitude apart in mass, with
brushed and brushless motors; two helicopters with the two rotor systems
JSBSim's rotor model represents (section 3.1) - a teetering two-blade rotor
with a mechanical stabilizer, and a four-blade articulated one with a canted
tail rotor and a scheduled stabilator - both with public NASA math models from
the same family (Talbot et al., NASA TM-84281) and flight or trim data to
check against.

**Considered and not chosen.**
- *MBB Bo 105*: its hingeless rotor's hub stiffness is not in JSBSim's rotor
  model, and its public data (NASA CR-3144) is a 20 MB report, beyond what
  could be read here without downloading it.
- *DJI F450*: a kit frame; its dynamics depend on the motors, propellers and
  battery fitted, with no single identified set.
- *AH-1*: JSBSim already ships a model of it (`ah1s`); it served as a
  reference instead (its numbers are not copied: LGPL).

**Conflicts in the sources, and what was used.**
- The Crazyflie 2.1 datasheet gives 29 g, the identified 2.0 was 27 g: the
  model is the identified aircraft, 27 g.
- Bitcraze gives the Crazyflie as "92 x 92 x 29 mm (motor-to-motor and
  including motor mount feet)"; Luis & Le Ny's arm is 39.73 mm (79.5 mm
  between opposite rotors). Bitcraze's drawing of the board, scaled by its
  expansion headers' 2 mm pitch, puts the arms' ends 40.0 mm from the centre:
  the rotors' axes are there, and the 92 mm spans the motor mounts' outsides.
- For the IRIS+, retailers disagree on the motors (850 or 920 kV) and the
  propellers (10 x 4.7 or 9.5 x 4.5 in). The model follows PX4's, whose
  1.5 kg is the 1,282 g aircraft with 218 g of its 400 g payload. Its feet were
  first estimated 0.10 m below the c.g.; PX4's model puts them 0.054 m below:
  the flight model stands on those.
- TM-73254 gives the UH-1H's tip speed as 760 ft/s in table 2, but every
  constant in its table 3 uses 33.9 rad/s (324 rpm, 818 ft/s): the model uses
  those. It halved the Lock-number term (R6) by mistake and kept it "because
  subjectively more realistic handling qualities resulted": the model uses the
  blade's true Lock number.
- TM-73254 puts the UH-1H's tail rotor at station 479.4, water line 137.5 -
  above the main rotor's hub - which no drawing of the aircraft shows. The
  operator's manual's three-view, registered at the main rotor's hub, puts it
  at station 469.7, water line 118.7 (10 ft 2.5 in over the ground, 18 in under
  the main rotor's hub), left of the pylon: the flight model flies the
  manual's, keeping TM's fitted damping and its arms.
- TM-85890 gives the UH-60A's tail rotor at station 732.0, water line 324.7
  and no butt line: its hub on the plane of symmetry. The operator's manual's
  front view draws it 18.8 in right of the centre line, on the canted shaft
  through TM's point - TM's point is where the shaft crosses the plane of
  symmetry, which leaves the thrust's line of action, and so its moments,
  where they are. The flight model keeps TM's point; the 3D model draws the
  hub along the shaft at the drawing's offset (section 4.1).
- The UH-60A's wheels were first estimated from general-arrangement
  proportions (the mains at station 247); the operator's manual's side view
  puts them at 294.2, 55.8 in out, the tail wheel 29 ft behind at 641.3: the
  contacts follow the drawing.
- TM-85890's table 4 is GENHEL's trim - the math model the report documents,
  itself checked against flight tests elsewhere - not flight data: section 6
  compares against it as such.

## 3. Decision

### 3.1 The flight dynamics: JSBSim, from published data

All four are JSBSim aircraft, written by hangar from design files that cite
their sources, so they share everything the platform has - state, recording,
ground contact, the viewer, determinism - and no new flight model enters the
platform.

- **Helicopters** fly JSBSim's rotor model (`FGRotor`, T. Kreitler, after
  Bailey NACA 716 and Talbot NASA TM-73254): blade flapping, uniform inflow
  with a lag, tip loss, ground effect, and the rotor's rpm as a state driven
  through the gearbox. The tail rotor is a second rotor tied to the main
  rotor's rpm; a cant is its orientation. The engine is a governed power
  source whose governor and lags are in the flight control system (the T700's
  in TM-85890 table 3). The mechanical parts of the control system are the
  flight control system too: the UH-1H's stabilizer bar (TM-73254 eq. 29-30)
  and its elevator linked to the stick (table 1); the UH-60A's mixing unit and
  rate feedbacks (table 2), its scheduled stabilator (figure 10) and pitch bias
  actuator (figure 11). The fuselage and tail: TM-73254's forms for the UH-1H;
  TM-85890's wind-tunnel regressions and TM-84281's stabilizer and fin through
  360 deg for the UH-60A. Each stands on its skids or wheels (JSBSim BOGEY
  contacts) and, struck hard or turned over, on its airframe: structure
  contacts on its convex hull and its hubs, as a fixed wing's. The airframe
  takes the engine's torque, not the rotors' aerodynamic torque, which JSBSim
  applies as though the airframe held their speed: an external moment gives
  back the difference (section 7).
- **The rotor patch** (`cmake/JsbsimPatches.cmake`, shipped, noted in
  THIRD_PARTY_NOTICES): a reset restarts a rotor as a load does instead of
  from what the previous run left, the first calculation converges the inflow
  (a start at lambda = -0.001 had divided the thrust by 0.002), and
  `<model> classical </model>` computes the in-plane forces and the torque from
  the blade-element expressions TM-73254 flies. Upstream's minimum-complexity
  forms leave a UH-1H unstable in angle of attack at 60 kt where the classical
  ones, and its flight records, are stable; the UH-60A, with 18 deg of twist
  and a 4.7 % hinge offset that TM-73254's forms leave out, flies upstream's,
  which match GENHEL's trims more closely.
- **Quadrotors**: each motor's speed is a state of the flight control system,
  w' = (w_cmd - w)/tau with the design's lags up and down; its thrust k_T w^2
  is a JSBSim direct thruster's; the reaction torques k_Q w^2, the rotor drag
  (gym-pybullet-drones' for the Crazyflie, PX4's in the rotors' plane for the
  IRIS+) and the Crazyflie's ground effect are aerodynamic functions, each
  body axis's in one element: JSBSim keeps only the last `<axis>` of a name,
  and a second Z axis dropped the Crazyflie's drag along its axis until
  ADR-29 FA-3b restored it (section 10). The
  command is each motor's thrust, 0..1 of its maximum (`throttle[i]`), roll,
  pitch and yaw mixed in; the speed a motor commands is the square root of it,
  as an ESC with thrust linearisation gives it. The rotors' speeds are the
  engines' rpm the platform reports (`VehicleState::engineRpm`). The ground
  effect is gym-pybullet-drones' k_ge (r/4h)^2 with its height clip
  (GND_EFF_H_CLIP = r/4 sqrt(15 k_ge/4)), so the gain is never over 4/15.
  Each stands on its feet (JSBSim BOGEY contacts) and, turned over, on its
  rotors' hubs and its top (STRUCTURE contacts), their springs and dampers
  sized by hangar for the platform's step (section 7).
- **Energy on board** (ADR-29 FA-3b):
  - **The helicopters' fuel.** Each carries a tank at the c.g., full as it
    spawns and part of the design's weight, so it spawns as its report flies
    it. The UH-60A's two tanks hold 360 gal, 2,340 lb (its operator's
    manual); the UH-1H's cells hold 1,400 lb. The engine burns its specific
    fuel consumption times the power it gives: the UH-60A's 1/K_E, 0.571 lb
    per shp-h (TM-85890 table 3), and the UH-1H's 0.623, a T53's on a UH-1B.
    Burning it takes JSBSim's electric engine patched (`cmake/JsbsimPatches.cmake`,
    noted in THIRD_PARTY_NOTICES). Once the tank is empty the engine gives no
    power and the rotor is left to the air.
  - **The quadrotors' batteries.** A battery's capacity is the IRIS+'s 56.6
    Wh or the Crazyflie 2.0's 0.888 Wh. The power it gives is the published
    hover's - its capacity over its flight time, taken as a hover's: 181.6 W
    and 7.6 W - scaled by the rotors' speed cubed. The charge falls with it,
    and once it is spent the motors stop: the IRIS+ after 18.7 minutes'
    hover, the Crazyflie after 7. Until then a flight is the same, bit for
    bit, as without the battery.
  - hangar's flight tests fly with the fuel frozen, so a helicopter's hover is
    identified at the weight its design names.
- **Performance tables** (ADR-29 FA-3b; [flight-autonomy.md](flight-autonomy.md),
  4.13): hangar's `performance` stage flies each rotorcraft's tables with the fly
  stage's hold.
  - The rows are 100 m to 3,000 m. The weights are a helicopter's tank a tenth,
    half and wholly full, and a quadrotor's one weight.
  - The level points run from the hover to 97 % of the top level speed. The
    top is the fastest trim within the power (a helicopter's engines short of
    their limit, the rotor governed) and the design's pitch limit.
  - Each point records the power (a helicopter's engines', a quadrotor's
    battery's) and a helicopter's fuel. From them come the best-endurance and
    best-range speeds.
  - A helicopter's full-power climb is flown at each speed. A quadrotor has no
    climb in the tables: its thrust ignores a climb's inflow.
  - The stage checks the tables against the fly stage's hover and trims, the
    lighter against the heavier, and a burn flown at the best-endurance speed.
- **Limitations** of the models are in section 7.

### 3.2 What the controls mean: families and adapters

Two control families join `Stock`, `Direct` and `FlyByWire`:
`Helicopter` and `Multirotor`, each with an adapter (`jsbsim.helicopter`,
`jsbsim.multirotor`). `ControlInputs` does not change: its four channels are
the aircraft's roll, pitch, yaw and thrust controls, and the family says what
they move.

| Channel | Fixed wing | Helicopter | Multirotor |
| --- | --- | --- | --- |
| `aileron` (roll) | ailerons | lateral cyclic | roll into the mixer |
| `elevator` (pitch) | elevator | longitudinal cyclic | pitch into the mixer |
| `rudder` (yaw) | rudder | pedals (tail rotor collective) | yaw into the mixer |
| `throttle[i]` (thrust) | engine *i* | collective (all entries) | motor *i*'s thrust, the mixer adding roll, pitch and yaw |

An adapter says, for its family: the authority groups its axes are owned in
(a wing: roll with yaw, pitch, thrust; a rotorcraft: the cyclic - roll with
pitch -, the yaw, the thrust), how a command's fields merge onto the axes, what
the vehicle default's hold keeps with each axis (section 1.2), the features
its aircraft have (`kFeatureWingborne`, `kFeatureHover`), and which parameters
it supports.

### 3.3 Capabilities

- The actuator capability's parameters take the family's names (a
  helicopter's are `lateral_cyclic`, `longitudinal_cyclic`, `pedals`,
  `collective`). Parameters are a command's fields in order, so commands are
  unchanged.
- A multirotor's per-engine capability, `fsim.flight.engines`, owns all four
  primary axes, since its motors fly roll, pitch and yaw too: user code can
  set each motor's thrust directly (`submit_support("engines", ...)`,
  `FSIM_SUPPORT_ENGINES`). A helicopter offers none: its engines are governed.
- A parameter can be unsupported by an aircraft (`ParameterInfo::supported`,
  C `fsim_parameter_info.unsupported`). A NEW or UPDATE that sets one other
  than to hold or its default is refused `invalid_parameter`; the older entry
  points ignore it, as they always ignored what a loop did not use.
- `hover`, a new behaviour, holds a position and heading - where it started,
  or `lat_deg`, `lon_deg`, `altitude_m`, `heading_deg`. A behaviour may need a
  feature (`BehaviorTraits::features`); `hover` needs `kFeatureHover`, so it is
  offered only to aircraft that can hover.
- The behaviours both kinds are offered fly a rotorcraft as it flies
  (ADR-28's step VI-1, [vehicle-interface.md](vehicle-interface.md)): they
  see the vehicle's features (`ControlContext::features`). `hold` keeps the
  velocity over the ground it had, so a hover stays put in wind (it kept the
  airspeed it had along the nose, the wind's, and flew off at 7 m/s);
  `waypoints` and `loiter` given no airspeed fly the position loop's own
  speed (a route from a hover never moved: its speed was the hover's
  airspeed, none), and `loiter` circles down to a metre.
- The Vehicle Interface's modes fly a rotorcraft over the ground (VI-3, VI-4).
  `hsa` holds a ground speed along a heading or course, or an airspeed. A
  `route` flies its legs as a velocity over the ground, the nose along the
  track; it slows for a turn only as much as the turn's radius asks (its
  radius planned from the velocity loop's tilt), stops only at an end where
  it loiters, and hovers there.
- Protection's wing limits are for aircraft with `kFeatureWingborne`.

### 3.4 Commands: fields a rotorcraft needs

Appended, with `kHold` defaults, so every existing C++ command compiles and
means what it did:

| Command | New fields | For |
| --- | --- | --- |
| `AccelerationCommand` | `pitchRateRadS`, `yawRateRadS` | body rates and collective thrust (the load factor): the interface multirotor policies train on |
| `VelocityCommand` | `northMs`, `eastMs` | a ground velocity with the heading free: hovering, sideways flight, holding a point against wind |
| `PositionCommand` | `headingRad` | the heading to face at the point, where a rotorcraft hovers |

A fixed-wing aircraft marks them unsupported. The C ABI 1.5 takes the wider
rows in its capability calls (`fsim_command_field_count_full`,
`fsim_activity_update_batch_n`) beside the frozen command structs, and Python
names them (`SETPOINT_FIELDS`).

### 3.5 The profile

New versions of three sections and one new section, each read alongside the
older version (7.3 of ADR-26):
- **identity v2:** classes `Helicopter` and `Multirotor`, and the two families.
- **effectors v2:** the new control meanings, and what the thrust control is:
  throttle, collective or rotor thrust.
- **propulsion v2:** `EngineType::Turboshaft`.
- **`hover` v1 (new):** the rotorcraft plant, identified by hangar in the
  hover (`aircraft/<name>/hover.toml`): each axis's body acceleration per unit
  command, its damping and its lag, the heave per unit thrust control, and the
  hover's trims - its commands and its attitude.

The envelope and performance sections keep their meaning: a rotorcraft sets
the limits it has (bank, pitch, roll rate, never-exceed speed) and leaves stall
and alpha unknown.

### 3.6 Control laws for rotorcraft

Four built-in controllers (`src/control/Rotor.cpp`), chosen for rotorcraft by
the control section and designed from the hover section by the same pole
placement as ADR-26's step 5a (`designRotorLaws`):

| Level | Controller | Does |
| --- | --- | --- |
| Acceleration | `rotor_allocation` | body rates and the load factor to cyclic, pedals and collective, or to the mixer: rate loops on the identified responses, the hover trims fed forward; the load factor measured from the body's specific force (JSBSim's `Nlf` is aerodynamic only and leaves out the rotor's thrust) |
| Attitude | `rotor_attitude` | roll and pitch attitude to rates, the heading to a yaw rate (not a bank), the thrust control passed through |
| Velocity | `rotor_velocity` | a ground velocity - or an airspeed along the heading - to attitude within a tilt limit, the vertical speed to the thrust control, the heading held on its own; the integral's band smooth, not a hard band, which had left a dead zone. Its integral trims the whole tilt forward - the drag of a fast multirotor takes most of it (ADR-29 FA-3e: the Crazyflie's best range needs 22 of its 24 degrees, and had been flown 15 % short on half) - and half sideways, as an orbit needs. Asked beyond what it can fly, it holds its tilt limit and the fastest that gives (the Crazyflie 18.30 m/s along its nose); a checked command is held to the tables' top level speed first ([flight-autonomy.md](flight-autonomy.md), 4.48) |
| Position | `rotor_position` | the distance to go to a ground velocity that brings it to a stop there - what the velocity loop, a lag behind its command, can stop from - the height to a vertical speed, the heading on arrival |

The cascade, the runtime and protection are unchanged: the acceleration
level's pseudo-controls are still rates and a load factor.

### 3.7 hangar

A rotorcraft pipeline beside the fixed-wing one (`tools/hangar/hangar/rotorcraft`,
[hangar.md](hangar.md#rotorcraft)): `build` writes the JSBSim aircraft and its
profile, `model` the viewer's model, `fly` flies it in the platform's own
JSBSim - the hover's trim, the published trims, step responses, and the
identification that fills the hover section - `performance` flies its
performance tables (ADR-29 FA-3b: the profile's `tables` section), and
`report` draws it. The 3D
model is built as the fighters' are, from three-views (section 4), with what a
rotorcraft needed added to hangar in general rather than for these four:

- `[[part]]` shapes beside the bodies and surfaces - the mesher's primitives,
  among them a new `plate`, an outline extruded to a thickness with its edges
  rounded (a circuit board, a frame's arms, a landing leg) - with a colour of
  their own when the livery's is not theirs, and a fillet sized to the
  aircraft (a millimetre for a nano quadrotor);
- the mesher's cell set by the design (`[model] cell_m`) where 1,500 cells
  along the aircraft would be coarser than its parts;
- each rotor drawn from the flight model's own rotor data - its hub, shaft,
  radius, blades, chord, twist and sense - with its look from
  `[model.main_rotor]`, `[model.tail_rotor]` or `[model.propeller]`: airfoil,
  pitch, precone, swept tips, the hub (grips, a stabilizer bar, a bifilar
  absorber, a spinner), colours, and `offset_m` along the shaft (section 2);
- a multirotor's feet where they are drawn (`[ground] feet_m`) and its top
  contacts (`top_m`); a helicopter's structure contacts, found on its drawn
  airframe and hubs and sized as a fixed wing's, and the rotors' drive
  (section 7);
- the model stage's checks: the airframe closed and in one piece, its length
  and rotors' diameters against the drawings', each rotor node at its hub, on
  the flight model's line of thrust and turning about its shaft, its blades
  and blur disc present, and the flight model's ground contacts on the drawn
  skids, tyres or feet.

### 3.8 The viewer

- A rotor is a node the viewer turns (`fsim:propeller:<engine>`, its x axis
  the shaft, pointing the way the rotor turns): at the rpm the simulation
  reports for it - a helicopter's rotor speed, a multirotor's motor speed
  state, so what user code commands through the SDK is what turns. Its blades
  (`fsim:blades:<engine>:<rpm>`) and a translucent disc (`fsim:disc:...`)
  swap above the rpm where a blade moves a third of the way to the next
  between frames at 60 Hz (a two-blade propeller's 600 rpm, the UH-60A's main
  rotor's 300: its 258 rpm shows the blades turning).
- The chase camera's nearest distance and the eye's clearance scale with the
  model's size (9 cm to 64 m), and the near plane with the depth range.
- `--screenshot-frames n --screenshot-every s` saves a sequence of frames
  (section 6.4 used it).

## 4. The 3D models

Each is measured from the drawings named in section 2, in the browser - pixels
to stations, water lines and butt lines through the drawing's own scale and a
registration point the flight data give - then written as the design's bodies,
surfaces, struts, gear and parts. The drawings are not stored.

| | UH-1H | UH-60A | IRIS+ | Crazyflie 2.0 |
| --- | --- | --- | --- | --- |
| Views | side, top, front (one sheet) | side and front (TM 1-1520-237-10); top (FM 44-80) | PX4's mesh in three orthographic profiles | the board from the top (Bitcraze); the product photograph for the heights |
| Scale | 2.531 px/in, by its 41 ft 5 in (checked by the mast's and cabin's heights) | side 1.233 px/in, by its 50 ft 7.5 in (its wheel base and folded length agree); front 1.648 px/in, by its 5 ft 1 in and the hub's height | the mesh's metres | 9.11 px/mm, by the expansion headers' 2 mm pitch |
| Registered at | the main rotor's hub (TM-73254: station 133.5, water line 136.5) | the main rotor's hub (TM-85890: 341.2, 315.0) | PX4's c.g. | the board's centre |
| Airframe | fuselage (22 stations), cowling, tail boom, gearbox, skids and cross tubes, pylon, synchronized elevator (moves), mast and swashplate | fuselage (35 stations), deck fairing, T700 nacelles and exhausts, sponsons, swept pylon, tail rotor gearbox, stabilator (moves, 39 deg down to 8 up), wheels on legs with oleos, drag beams, mast, swashplate, IR jammer | shell, arms (blue front, black rear), motors in their mounts, tapered legs | the board (a plate of its traced outline), battery, headers, motors, motor mounts and their feet |
| Rotors | 2-blade main rotor with its stabilizer bar, 2-blade tail rotor (white and red) | 4 blades with swept tips (20 deg from 93 %) and the bifilar absorber; 4-blade tail rotor, canted 20 deg, drawn 20 in along its shaft | 2-blade propellers, 10 in (PX4's radius), 4.5 in pitch | 2-blade propellers, 46 mm (the identified radius), 17 mm pitch (Bitcraze's 47-17) |
| Paint | Army olive drab (FS 34087), glass where the three-view draws the windows | Army helicopter green (FS 34031), windows from both views, the inlets dark | black shell | the battery's grey; black board, white mounts, metal motors |

The checks of section 3.7 pass for all four (`aircraft/<name>/out/model.json`):
the UH-1H is 12.64 m long against 12.624, the UH-60A 15.50 against 15.431;
every rotor node is at its hub within 1 mm, on its line of thrust within 1 mm
and turns about its shaft within 0.1 deg; the UH-1H's skid contacts are 6 mm
from the drawn skids' bottoms, the UH-60A's contacts 1 mm from the tyres', the
quadrotors' feet within 0.1 mm of the drawn feet. Where the drawings disagree
with each other or with the flight data it is recorded in the design file: the
UH-60A's front view's dimension lines give 1.52 to 1.69 px/in (its scale is
the one the hub's height from the side view agrees with); FM 44-80's top view
is a recognition drawing, within about 5 %, so it gave the plan's shape and
Wikipedia's 7 ft 9 in its width; the manual draws the UH-60A's tail rotor 5 to
10 in lower than TM-85890, and the pylon's top is raised to it.

### 4.1 The axes

hangar's design frame is x aft, y right, z up; a helicopter's data are
stations, butt lines and water lines (the same axes, inches), a multirotor's x
forward, y left (turned). The glTF's frame is x = -y, y = z, z = -x of the
design frame, its origin the c.g. the simulation reports. A rotor's node is at
its hub, its x axis the thrust direction times the sense (right-handed), and
the check computes that from the flight data and the frame's definition, not
from the code that wrote the node. The UH-60A's tail rotor: its thrust
(0, cos 20 deg, sin 20 deg) - right and up; drawn 0.508 m out along it from
TM's point, 18.8 in right of the centre line and 6.6 in above TM's water line,
which is where the operator's manual's front view draws it; its node's x axis
is still the flight model's shaft.

## 5. Compatibility

Verified on 2026-09-26 with everything above in place:

- Every existing flight is bit-identical: the control digests
  (`fsim_control_bench digest`, protection on and off) equal step 5b's
  byte for byte, and the c172x checkpoints hold. Existing commands leave the
  new fields `kHold`, and the existing laws never read them.
- The fixed-wing models are byte-identical: five re-meshed with the changed
  mesher and model writer (C172, F-16C, A-10C, B-52H, Skua) leave git clean.
  A design without part colours carries no new material.
- The C ABI and the profile change additively (1.5); the structs shared with
  viewers and recordings do not change.
- The conformance suite covers the new aircraft and adapters as it covers
  every shipped aircraft; `ctest` passes 154 of 154, among them the hangar
  suite, the Python SDK's and the allocation gate (no allocation per step).

## 6. Validation

### 6.1 The quadrotors (hangar build and fly)

| | Crazyflie 2.0 | IRIS+ |
| --- | --- | --- |
| Thrust to weight | 2.24 (gym-pybullet-drones: 2.25) | 1.92 (PX4's constants: 1.92) |
| Hover rotor speed | 14,477 rpm (gym-pybullet-drones: 14,468) | 7,578 rpm, 793.6 rad/s (sqrt(mg/4k_T): 793.5) |
| Hover held steady | yes | yes |
| Identified control power (roll, pitch, yaw; heave) | 301, -293, -173 rad/s2; 21.4 m/s2 | 50.1, -31.0, -7.55 rad/s2; 18.3 m/s2 |

### 6.2 The UH-1H (against TM-73254's flight records)

| | Model | Flight |
| --- | --- | --- |
| Hover collective | 3.9 in | 5.5 in |
| Hover power | 571 shp | - |
| Hover pitch attitude | 4.6 deg | - |
| Longitudinal stick at 60 kt | 1.67 in | 1.2 in |

The model hovers with 1.6 in less collective than the flight records (section 7).

### 6.3 The UH-60A (against GENHEL's trim, TM-85890 table 4)

Model / GENHEL, this file's senses (stick + forward):

| kt | collective, in (+- 0.3) | longitudinal, in (+- 0.5) | pedal, in (+- 0.3) | pitch, deg (+- 1.5) | lateral, in (+- 1.2) |
| --- | --- | --- | --- | --- | --- |
| 1 | 5.80 / 5.72 | -0.21 / -0.13 | -1.40 / -1.28 | 4.77 / 5.05 | 0.14 / 0.23 |
| 20 | 5.35 / 5.36 | 0.22 / 0.37 | -1.14 / -1.07 | 5.32 / 5.83 | 0.04 / -1.00 |
| 40 | 4.55 / 4.58 | 0.09 / 0.21 | -0.70 / -0.58 | 4.35 / 4.34 | -0.06 / -0.76 |
| 60 | 4.11 / 4.19 | 0.52 / 0.42 | -0.66 / -0.58 | 4.21 / 3.49 | 0.00 / -0.23 |
| 100 | 4.28 / 4.42 | 1.13 / 1.06 | -0.27 / -0.26 | 3.52 / 2.47 | -0.23 / 0.18 |
| 140 | 5.40 / 5.72 (warning) | 1.69 / 1.80 | -0.09 / -0.01 | 1.05 / -0.30 | -0.51 / 0.40 |

All within the tolerances except the collective at 140 kt, 0.02 in outside
(section 7). Its identified hover control power: roll 6.43, pitch -1.73, yaw
-2.00 rad/s2, heave 26.7 m/s2.

### 6.4 Flown through the platform

- **The rotorcraft manoeuvre suite** (`tests/test_rotorcraft.cpp`, in the
  conformance label): each aircraft, ten vehicles at once, hovers 20 s from its
  hover's attitude, then a hover hold, a climb, a 90 deg heading step, a
  velocity step, a roll-rate step, a position capture with a heading, the
  `hover` behaviour, the thrust flown by a policy while an autopilot holds the
  cyclic and yaw, the vehicle default's hold, and a bank beyond the envelope,
  limited. Each step must reach 90 % with at most 50 % overshoot, each hold
  stay within the aircraft's drift (Crazyflie 0.3 m, IRIS+ 0.5 m, UH-1H 15 m,
  UH-60A 10 m), the capture arrive within 5 % of its distance, facing its
  heading. All four pass; two worlds fly the same commands to the same bits.
- **The propellers, from user code**: `submit_support("engines", ...)` set
  the Crazyflie's four motors to 1e-4, 4e-5, 1e-5 and 0 of full thrust; the
  simulation reported 216.7, 137.0, 68.5 and 0 rpm (sqrt of each, of its
  21,666 rpm), and a frame sequence from the viewer showed three propellers
  turning at those rates and the fourth still; at 0.35 all four ran at 12,818
  rpm and showed their discs. The IRIS+ the same (210, 133, 66, 0 rpm).

## 7. Limitations

- **The rotor model**: uniform inflow with a lag, no dynamic wake; no hub
  spring (so no hingeless rotor); no pitch-flap coupling (the UH-60A's tail
  rotor has it); a quasi-static fuselage.
- **The UH-1H** hovers with 3.9 in of collective where its flight records need
  5.5 in; its 60 kt stick is 0.5 in forward of the records'.
- **The UH-60A** needs 5.40 in of collective at 140 kt where GENHEL needs
  5.72: outside the 0.3 in tolerance by 0.02 in (a warning, not a failure).
- **The tail rotors' sense** - the top blade moving aft - is an assumption for
  both helicopters: no source here gives it. It is the look's; the flight
  model does not depend on it.
- **The UH-60A's tail rotor** is where TM-85890 puts it, on the centre line;
  drawn 20 in along its shaft. The thrust's moments are the same; the moments
  of the rotor's in-plane forces differ by that offset.
- **Striking the ground**: the contacts are JSBSim's springs and dampers,
  each applied for the platform's whole 120 Hz step. A quadrotor striking
  the ground faster than its contacts can take in a step (the Crazyflie at
  35 m/s moves 0.29 m a step on 18 mm legs) was sent back up faster than it
  came down: the IRIS+ bounced, and the Crazyflie, thrown up tumbling,
  diverged. Such an impact is now inelastic (flightsim's JSBSim,
  THIRD_PARTY_NOTICES.md): a quadrotor whose battery is spent, striking at
  40 to 50 m/s, or a Crazyflie tumbled into the ground at 150 rad/s (two
  motors at 90 % for 0.4 s), comes to rest where it strikes, within a hop of
  2.5 m (section 10).
- **Left uncommanded in the air**: under the neutral vehicle default
  (`VehicleDefault::Neutral`) a rotorcraft's thrust stands still and it
  falls. A Crazyflie let go at 150 m for 5 s is at 27 m falling at 49 m/s.
  Asked then for 10 m/s or more, it strikes the ground at 35 m/s. It
  diverged there, as a spent battery's did, until such an impact was made
  inelastic (above): it now leaves the ground no faster than it struck, and
  flies on or lies there. ADR-29 FA-3e took the divergence for its velocity
  loop's at 25 and 30 m/s ([flight-autonomy.md](flight-autonomy.md), 4.48).
  Settle it first (a velocity command, or `VehicleDefault::Hold`).
- **The quadrotors' contacts, sized for the step**: short of such an impact
  the contacts act as JSBSim gives them. The design files' springs and
  dampers - the Crazyflie's 60 N/m and 1.2 N s/m on each contact, on 27 g,
  and the IRIS+'s 2,000 N/m and 60 N s/m on 1.5 kg - were more than the
  120 Hz step integrates, and added energy:
  - parked, the Crazyflie hopped, its feet off the ground a quarter of the
    time at up to 0.37 m/s, and the IRIS+ rocked on its feet without end;
  - dropped on its back from 5 cm, the Crazyflie rose 12 cm, and from 10 cm
    it was still rocking on its back 10 s later.

  hangar now sizes them from the modes the contacts standing together make
  (docs/hangar.md, Rotorcraft). Parked, both stand still, their legs sunk
  2.9 mm and 4.1 mm - their contacts placed that far below the drawn feet,
  so the feet stand on the ground as drawn. Dropped from up to 30 cm, level
  or on the back, neither
  moves up faster than 0.81 of the speed it struck at, nor rises more than
  a centimetre off the ground, and both come to rest, the IRIS+ on its back
  within 4.4 s (section 10). The rule sizes the feet standing and the tops
  upside down. On its side an airframe rests on some of each, which it does
  not size: the IRIS+ fallen on its side comes to rest in 5.4 s, but with
  softer springs it kept rocking.
- **The helicopters on the ground** (found with the above). Let go from its
  hover at 150 m (the vehicle default's neutral: the collective down, the
  governor holding the rotors' speed), the UH-1H struck its skids at 16 m/s,
  pitched up and rolled over, and diverged 1.2 s later: the fleet test's
  three diverged flights. Dropped on its back or side, the UH-1H diverged and
  the UH-60A fell 440 m through the ground. Three things did it (section 10):
  - **The skids pitched it up.** Its four skid contacts have the same spring
    and damper, but its c.g. (station 140) is 15 in ahead of the rear pair
    (155) and 105 in behind the front pair (35). In a level strike the front
    pair's push has seven times the lever arm: at 7.7 m/s it pitched up at
    0.7 rad/s about the rear contacts, past their 14.6 deg tip-back angle,
    with nothing to catch its tail. It stood on its tail and rolled over. A
    strike at 4.4 m/s or more did it; 3.2 m/s did not. The rotor is not the
    cause: at flat pitch it pushes some 30 lb, and with the stabilizer bar,
    the ground effect or the skids' friction taken out in turn, the UH-1H
    tipped over as before. (Without the bar, the uncommanded UH-1H diverges
    in the air 6.5 s into its fall.)
  - **Its gear was all it touched with.** On its back or side, its skids or
    wheels point up, and the airframe sank into the ground.
  - **The rotor's torque gave it energy.** JSBSim applies a rotor's
    aerodynamic torque to the airframe, as though the airframe held the
    rotor's speed both ways. When the airframe rotates, the rotor's disc lags
    behind its shaft (the flapping lag, 0.14 s, times the rate). The torque
    then falls below zero as the square of the rates: the air drives the
    rotor, and the airframe took that torque too. Dropped on its back and
    sinking through the ground, touching nothing, the UH-1H gained its
    rotational energy from the work that torque did on it. The torque reached
    -1.1 million lb ft 2.6 s after the drop, with the rotor at its 130 % stop,
    and the airframe diverged at 3.2 s. The fall from the hover ended the
    same way: tumbling on its tail, then flung off by its skids, which came
    back up through the ground from metres below it.
  - **The change** (in hangar). Its
    airframe meets the ground: hangar gives a helicopter structure contacts
    as it does a fixed wing (docs/hangar.md), on its convex hull and its
    hubs, sized for the step. The UH-1H gets its tail skid, stabilizer tips,
    nose and cabin roof, belly (where its cross tubes pass under it), main
    rotor hub and tail rotor gearbox. The UH-60A gets its nose, deck and
    nacelles, drag beams, tail cone, stabilator, main rotor hub, and tail
    rotor hub and gearbox. And the airframe takes the rotors' drive: the
    flight control system's rotor speed already takes the difference between
    the engine's torque and the rotors'. An external moment about the main
    rotor's shaft now gives back what JSBSim applies beyond the engine's
    torque, which the freewheel keeps from falling below zero. Let go from
    its hover, the UH-1H tips back 11 deg onto its tail skid and settles on
    its skids. Dropped on their backs or sides, both come to rest on their
    sides.
  - **What is left.**
    - A strike pitched the UH-1H up onto its tail skid (8 deg at 3.2 m/s, 11
      at 16), and it sat 0.7 deg nose up. Its skid contacts now take springs
      and dampers in proportion to the load each carries, as hangar sizes a
      fixed wing's wheels: it sits level, and a level strike stays level
      (section 10).
    - After a crash the rotors kept turning at their governed speed. Now the
      blades strike the ground when the main rotor's disc reaches it, and
      then the engine gives nothing and the rotor stops (section 10).
    - The UH-1H lying on its side can still rock (at up to 1.5 m/s after
      20 s, in 4 of 16 drops). It was thought to rock under its tail rotor's
      thrust; it rocks as much with its rotors stopped. Its structure
      contacts are sized one by one, and several of them resting at once
      together damp a mode more than the step integrates: with their damping
      halved, a UH-1H dropped on its back came to rest, but in other drops it
      rocked more and bounced twice as high. Not changed; the contacts want
      sizing as a set, as the quadrotors' are.
    - Let go on its wheels, the UH-60A turns on them at 0.74 rad/s under its
      tail rotor (its rotor turning, as it should upright).
    - The flight control system's rotor speed had no stop of its own: past
      JSBSim's 130 % it wound on (to 1,800 rad/s in the old tumble), and the
      governor waited for it to come down. It now stops where the rotors'
      does, at FGRotor's least and 130 % (section 10).
- **Power and thrust at altitude**: the helicopters' engines give the
  design's rating (a transmission limit) at any height, and a quadrotor's
  thrust is its rotors' speed squared, whatever the air's density. None of
  the four has a ceiling in these models; their performance tables stop at
  3,000 m, and the ceiling the platform reads from them is none.
- **A quadrotor in forward flight**: its thrust neither falls with a climb's
  inflow nor gains translational lift.
  - The IRIS+'s drag is PX4's rotor drag, in the rotors' plane only. Tilted
    forward it holds the aircraft up too, and level flight needs the weight
    times the cosine of the tilt in thrust. Its power therefore falls all the
    way to its top speed: 181 W in the hover, 139 W at 15.9 m/s and 33 deg
    of tilt. Its tables' best-endurance and best-range speeds are its top.
  - The Crazyflie's drag acts along its axis too, so its power is least in
    the hover.
  - Neither has a full-power climb in the tables (above).
- **The UH-60A's top level speed** at full weight is 187 kt at 100 m, just
  below its 193 kt never-exceed speed, with 1,375 shp at 140 kt (GENHEL's
  trims fly the controls, not the power).
- **Spawning**: a rotorcraft spawned level at zero airspeed lurches until its
  loops take its hover attitude (the UH-1H lurches 0.7 m/s forward and still
  moves at 0.3 m/s after 20 s). To start in trim, spawn it at its profile's hover attitude
  (`profile_value("hover/pitch_attitude_deg")` and `.../roll_attitude_deg`),
  as the manoeuvre suite does.
- **Closing on a point** (investigated after ADR-29 FA-5c; the loops as they
  were): over a point 150 m ahead the UH-1H came within 1 m 85 s after its
  command, the UH-60A 17 s. Measured causes:
  - Not its position gain, the attitude floor or its pitch power. Its loops
    are half the UH-60A's because its flapping lag is twice as long (0.144 s,
    TM-73254's choice, against 0.072): rates 2.63 rad/s, attitude 0.66,
    velocity 0.22, position 0.0875/s. The 0.4 floor does not bind; the pitch
    power, -0.704 per unit, is the teetering rotor's (thrust x hub height /
    Iyy x cyclic per unit: 0.686) and does not enter the bandwidths.
  - The velocity loop's integral. It winds up following the position loop's
    deceleration and holds it on past the point (the UH-1H 2.8 deg of nose-up
    as it stopped), so the aircraft backs away (the UH-1H to 8.6 m; to 14 m
    from 300 m out) and returns on a closed-loop pole near a fifth of the
    velocity bandwidth: 22 s for the UH-1H, 12 s for the UH-60A, which
    overshoots 2.3 and 4.6 m the same way.
  - What the identified plant leaves out: the rotor's flapping with the speed
    a tilt builds, and the UH-1H's stabilizer bar (0.528 s through a 3.3 s
    lag: at low frequency 10.6 1/s of roll damping and 1.8 of pitch, where the
    1 s fit sees 2.7 and 0.45). Held against them, the UH-1H's roll attitude
    reaches 64 % of a 5 deg step, the UH-60A's 90 %.
  - Changes measured on all four rotorcraft and not taken: faster attitude or
    velocity ratios alone (the UH-1H then overshoots 7 to 9 m); the integral
    on a reference model's error (the Crazyflie's drag trim lags, 16 to 35 s
    to settle, and a helicopter lags in turns, 9 to 23 m off the fleet's
    curves and orbits); the same along the velocity only (the UH-1H there in
    21 s, but 6.9 m past it); the velocity's rate of change fed forward (the
    guidance already anticipates the loop's lag: the UH-60A 17 to 28 m off
    routes, curves and racetracks); the stabilizer bar cancelled in the rate
    loop (the flapping with speed still holds the attitude back). A third
    instead of a quarter of the rates' bandwidth for the attitude also leaves
    the UH-60A's protected bank 1 deg from the suite's limit.
  - What would: the rotor's speed derivatives and the stabilizer bar (the
    design gives the bar's gain and lag) in the hover section, and the loops
    designed against them; then the integral on the reference model along the
    velocity. With the attitude at a third, that one flew every route, curve
    and pattern case in the tests and the fleet, and halved the UH-60A's time
    to settle over a point (hover 35 to 25 s, 300 m out 47 to 23 s); only the
    UH-1H's overshoot kept it out.
- **The drawings** are operator's manuals' and a recognition manual's figures,
  within a few per cent (section 4); the IRIS+ is PX4's model of it rather than
  a 3DR drawing; the Crazyflie's heights are from a photograph. The Crazyflie's
  motor mounts are drawn white, not clear; the paint colours are the Federal
  Standard colours' usual approximations.

## 8. Migration order (done)

1. **R1: flight models.** The rotorcraft pipeline in hangar and the four
   aircraft, validated open-loop against their sources.
2. **R2: families, profile and capabilities.** Adapters, sections, native
   parameter names, support flags, the multirotor's engines owning four axes,
   `hover`.
3. **R3: commands.** The new fields, the C ABI 1.5 and Python.
4. **R4: rotorcraft laws.** The four controllers, designed from the hover
   section, which hangar identifies.
5. **R5: flight tests.** The rotorcraft manoeuvre suite, and the conformance
   suite and CI with the new aircraft.
6. **R6: documentation.**
7. **The models from three-views** (the owner's follow-up): the drawings
   measured, hangar's parts, plate, colours, cell and rotor looks, the binding
   and contact checks, the propellers turned from what user code commands, the
   Crazyflie's ground effect clipped as gym-pybullet-drones clips it and the
   quadrotors' top contacts.

## 9. Sources

Flight data:
- Bitcraze, *Datasheet Crazyflie 2.1*, rev. 3; the Crazyflie 2.0's specifications (27 g; 92 x 92 x 29 mm "motor-to-motor and including motor mount feet").
- J. Förster, R. D'Andrea, *System Identification of the Crazyflie 2.0 Nano Quadrocopter*, ETH Zürich, 2015.
- C. Luis, J. Le Ny, *Design of a Trajectory Tracking Controller for a Nanoquadcopter*, arXiv:1608.05786, 2016, table 2.3.1.
- J. E. Kooi, R. Babuška, *Inclined Quadrotor Landing using Deep Reinforcement Learning*, arXiv:2103.09043, 2021, eq. 5.
- J. Panerati et al., gym-pybullet-drones (MIT), `cf2x.urdf` and its ground effect. <https://github.com/utiasDSL/gym-pybullet-drones>
- 3D Robotics, IRIS+ specifications (as listed by Adafruit, product 2199).
- PX4, PX4-SITL_gazebo-classic (BSD-3-Clause), `models/iris/iris.sdf.jinja`.
- P. D. Talbot, L. D. Corliss, *A Mathematical Force and Moment Model of a UH-1H Helicopter for Flight Dynamics Simulations*, NASA TM-73254, 1977.
- P. D. Talbot, B. E. Tinling, W. A. Decker, R. T. N. Chen, *A Mathematical Model of a Single Main Rotor Helicopter for Piloted Simulation*, NASA TM-84281, 1982.
- K. B. Hilbert, *A Mathematical Model of the UH-60 Helicopter*, NASA TM-85890, 1984.
- US Army TM 1-1520-237-10, ch. 5 limits (UH-60 Vne 193 kt), as transcribed at oocities.org/ericdurb/limits.htm.
- Energy on board (ADR-29 FA-3b):
  - the UH-60A's two crashworthy tanks, 360 US gal (TM 1-1520-237-10, the UH-60A/L operator's manual);
  - the UH-1H's cells full at 1,400 lb (uh1ops.com, "UH-1D/H/V Fuel System", after TM 55-1520-210-23-2);
  - a T53's 514 lb/h at 75 % of its 1,100 shp on a UH-1B (AOPA Pilot, March 2014, "The Bell UH-1B Huey");
  - the IRIS+'s 3S 5.1 Ah battery and 16 to 22 minutes' flight (as listed by Adafruit, product 2199);
  - the Crazyflie 2.0's 240 mAh LiPo and 7 minutes' flight (Bitcraze store, "Crazyflie 2.0").

Drawings and shapes (viewed in a browser and measured; nothing stored):
- US Army TM 55-1520-210-10 (UH-1H operator's manual), principal dimensions, p. 29 - Wikimedia Commons "Bell UH-1H Iroquois 3-view line drawing.png", public domain.
- US Army TM 1-1520-237-10 (UH-60 operator's manual), fig. 1-1 principal dimensions - Commons "Sikorsky_UH-60_Black_Hawk_dimensions.png", public domain.
- US Army FM 44-80 (visual aircraft recognition), UH-60A three-view - Commons "Sikorsky UH-60A Black Hawk 3-view line drawing.png", public domain.
- Wikipedia, "Sikorsky UH-60 Black Hawk", specifications (fuselage width 7 ft 9 in).
- Bitcraze, *Getting started with the Crazyflie 2.0 or Crazyflie 2.1(+)*: "frontCF.png" (the board from the top), "cf2_props.png" (the propellers' senses); the Crazyflie 2.0 product photograph.
- PX4, PX4-SITL_gazebo-classic, `models/iris/meshes/iris.stl` (its profiles measured, not copied).

Considered: EASA TCDS R.011 (Bo 105); FlightGear FGAddon `UH-1` and `UH-60`; JSBSim `ah1s` (T. Kreitler).

## 10. Measurements

- Control digests (`fsim_control_bench digest`, 20 flights, protection on
  and off): identical to step 5b's.
- `ctest`: 154 of 154 (the rotorcraft contract, manoeuvre suite and
  determinism among them).
- The model stage: every check passes for the four (section 4); the
  helicopters' airframes mesh in 7 to 11 s, the quadrotors' in 2.5 to 3.5 s.
- ADR-29 FA-3b, a named, measured change to the Crazyflie: its drag along its
  axis (gym-pybullet-drones' 10.311e-7 per rad/s) now acts.
  - The identified hover's heave damping goes from 0 to 0.146 1/s, and its
    heave control power from 21.44 to 21.12 m/s2 per unit (-1.5 %). Roll,
    pitch and yaw are unchanged.
  - Through the platform's loops the hover is the same. At 1 m/s its climb
    and descent track as before (the vertical speed's error 0.134 m/s rms,
    against 0.139), with 3.6 % more power climbing and 3.5 % less
    descending.
  - At 6 m/s forward its vertical speed holds six times tighter, and it
    draws 2.3 % more power, where before it drew less the faster it flew.
  - The IRIS+'s file is unchanged, byte for byte.
- **The ground impact**, a named, measured change (flightsim's JSBSim:
  `FGAccelerations.cpp` in `cmake/JsbsimPatches.cmake`; section 7).
  - Found as a Crazyflie left 5 s with no command (its motors off) from
    150 m, then flown an HSA at 10 m/s: diverged. Traced a JSBSim step at a
    time, it strikes the ground at 35 m/s, 9 deg nose down, its motors
    spinning up.
    - In the step it first touches, its nine contacts are 5 to 32 mm deep.
      JSBSim's limit on the damping of a contact just touching down holds
      them to 32 N, whose moment pitches its 1.4e-5 kg m2 at 134 rad/s in
      that step.
    - A step later it is 0.29 m deeper, sixteen times its legs' 18 mm, and
      the contacts, 0.30 to 0.33 m deep, apply 350 N for the step: their
      springs 18 N each, their dampers the rest. The nine together damp at
      400 1/s on 27 g, 3.3 times the step's rate (120 1/s). Applied for a
      whole step, a damper beyond once the step's rate does not stop the
      speed it damps but reverses it, and beyond twice sends it back faster.
    - 350 N for a step is 2.9 N s against the 0.53 N s it carries, and
      JSBSim's translational integrator (Adams-Bashforth 2) applies it 1.5
      times over: from 19.8 m/s down to 138 m/s up. The step after, the
      contacts left behind, the integrator takes back half: it leaves at
      83 m/s with 95 J of the 16.6 J it struck with, tumbling at 50 to
      70 rad/s.
    - In the air, its motors fighting the tumble, its rates grow to
      1,800 rad/s in 1.1 s, and JSBSim's integration of them overflows at
      step 833.
  - The change: where the ground's forces would send a contact point back
    out more than 5 m/s faster than it came in, the step applies instead the
    forces that stop the points' approach, none leaving faster than 0.5 m/s.
    Short of that the forces are JSBSim's, bit for bit. The margin stays
    well above what contacts reach that the step integrates, or nearly
    does: a parked Crazyflie 0.4 m/s (it hops), one dropped on its back
    from 30 cm 4 m/s; strikes of 10 m/s and more reach tens of m/s.
  - The found case now strikes at 35.2 m/s, is never faster after it, and
    25 s on lies on its back, its motors (still flying the HSA) pressing it
    down.
  - Dropped with their motors off, level from 5, 20, 62 and 128 m (10 to
    50 m/s), and from 62 m tilted, nose first, on an edge and upside down:
    - The Crazyflie left the ground up to 5.5 times as fast as it struck,
      bounced up to 187 m, and 3 of the 9 diverged. Now none is faster after
      it strikes than as it struck, save its 10 m/s drop on its back (1.09
      times, as before: the limit does not engage); it bounces 0.3 to 2.0 m
      and 3 s later is at rest (0.23 m/s at most).
    - The IRIS+ left the ground up to 4.3 times as fast and bounced up to
      173 m (flown as the found case, 3.1 times and 260 m). Now none is
      faster after; it bounces 0.15 to 0.85 m and is at rest 3 s later
      (0.04 m/s).
    - The UH-60A's drops are the same as before; the UH-1H's diverge, as
      before, from another cause (section 7).
  - The Crazyflie tumbled into the ground from 5 to 50 m, two of its motors
    at 90 % for 0.4 s (154 to 158 rad/s), then all off: 7 of the 8
    diverged, 6 of them thrown back up faster than they struck (at up to
    144 m/s). Now all 8 come to rest.
  - `test_rotorcraft` gains both: the quadrotors' drops, five attitudes
    each, and the found case. Both fail on the unpatched JSBSim.
  - Control digests (`fsim_control_bench digest`, 20 flights): identical,
    with protection and without.
  - Its cost: a world of 30 vehicles of ten types, before and after in
    turn, three rounds each. Parked, where the check runs every step, a
    vehicle's step took 8.68 us before and 8.60 after (medians); in the air,
    where it does not, 7.28 and 7.16: within the rounds' spread.
  - The fleet test, flown before and after with every flight's end state
    printed (docs/flight-autonomy.md, section 14): 3,284 states, each flight
    as it is judged and every vehicle as its case ends. 3,245 are identical,
    every judged state among them. The 39 that change all strike the ground
    after their judgement, or are vehicles a case leaves uncommanded:
    - 27 quadrotor flights that diverged come to rest instead: 20
      Crazyflies and 7 IRIS+, 20 of them after their batteries ran out and
      7 left uncommanded in cases flown by other aircraft. Two IRIS+ and a
      Crazyflie whose batteries ran out near their cases' ends strike
      differently.
    - 9 wings a case leaves uncommanded dive into the ground at 33 to
      235 m/s: the A-10C and the Su-25 three times, the E-3G twice, the
      Su-57 once. Their wrecks come to rest differently; none diverged
      before.
    - The fleet's flights ended diverged 30 times; now 3 times, the UH-1H's
      left uncommanded (section 7), as before, bit for bit.
  - `ctest`: 328 of 328, the two new cases among them.
  - Merged onto main at FA-8d (2026-09-29), past FA-6f2a to FA-8d, which it
    was not written on, and measured there the same way. The fleet test
    has 3,992 states now: 1,752 as flights are judged, 2,240 as cases end.
    Every judged state is identical. 63 end states change, all after
    their judgement: 39 quadrotors', 36 of which diverged and now come to
    rest, and 24 of wings a case leaves uncommanded, whose wrecks come to
    rest differently (the A-10C and the Su-25 five times each, the E-3G
    four, the Su-57 three, the F-22A, J-10A and J-20A twice, the F/A-18C
    once). The fleet's flights ended diverged 39 times; now 3, the UH-1H's,
    as before. Control digests, with protection and without, and the
    route and curve probes are unchanged; `ctest`: 370 of 370.
- **The quadrotors' contacts sized for the step**, a named, measured change
  (hangar: `contact_set` in `hangar/rotorcraft/multi.py`, docs/hangar.md,
  Rotorcraft; section 7), measured on top of the ground impact's change
  (above).
  - Why they added energy. JSBSim applies a contact's force for the whole
    step. It integrates the velocity with Adams-Bashforth 2 (over a
    position by Adams-Bashforth 3), and the body rates and attitude with
    forward Euler. Standing together, a set of contacts moves the airframe
    in three modes, heave, roll and pitch. Worked out as one JSBSim step of
    those modes (the test in `tools/hangar/tests/test_methods.py`), the old
    contacts grew a disturbance each step:
    - the Crazyflie's feet by 1.21 a step: their heave at ω·dt = 0.79,
      damped at 0.94, beyond Adams-Bashforth 2's stable range. Its tops,
      upside down, by 1.38;
    - the IRIS+'s feet by 1.56: their roll at ω·dt = 0.89, so overdamped
      (1.61) that Euler's step overshoots. Its tops by 1.73. On its back it
      came to rest anyway: it lies on its top and two hubs, a set whose
      modes are slower.
  - The rule: each set's springs put its fastest mode at ω·dt = 0.6, its
    dampers damp that mode at ζ = 0.7. The Crazyflie's nine contacts get
    22.9 N/m and 0.445 N s/m each (its tops' fastest mode is its feet's, the
    same roll); the IRIS+'s feet 903 N/m and 17.6 N s/m, its tops 854 N/m
    and 16.6 N s/m. One step now shrinks a disturbance to 0.75 of itself
    (the Crazyflie) and 0.90 (the IRIS+, whose slowest mode, the pitch, is
    damped least). The legs sink 2.9 mm (the Crazyflie) and 4.1 mm (the
    IRIS+) under the weight, where they sank 1.1 and 1.8.
  - Chosen by flying candidates, with the probe below, from ω·dt = 0.5 to
    0.7 and ζ = 0.5 to 1.0 (variant files through FSIM_AIRCRAFT_PATH). At 0.6
    with ζ from 0.6 to 0.8 every case came to rest. At 0.65 and ζ = 0.7 the
    Crazyflie moved up as fast as it struck; at 0.7 and ζ = 1.0, 1.57 times
    as fast. At 0.5 the IRIS+ fallen on its side kept rocking, at 0.06 m/s
    30 s on.
  - Flown with its motors off, a JSBSim step at a time, parked for 10 s,
    and dropped with its lowest contact 1 cm to 30 cm up: level, on its
    back, tilted (20 deg of roll, 10 of pitch), on an edge and nose first
    (10 cm).
    - The Crazyflie parked. Before, it hopped, its feet off the ground in
      298 of 1,200 steps, at up to 0.37 m/s, and 10 s on still moved at
      1.7 rad/s. Now it stands still from the first step, its centre of
      gravity 15.1 mm up.
    - The Crazyflie dropped. Before, it moved up at up to 2.7 times the
      speed it struck at. It rose higher than it fell from: 23 mm from
      2 cm on its back, 12 cm from 5 cm, 17 cm from 10 cm, 13 cm from 10 cm
      nose first, 34 cm from 30 cm level (landing on its back). None was
      at rest 10 s on. Now it moves up at most at 0.81 of the speed it
      struck at, rises off the ground only from 30 cm (8 mm level, 4 mm
      on its back), and comes to rest in 0.2 to 0.7 s; from an edge it
      rolls onto its back and rests in 3.8 s.
    - The IRIS+ parked. Before, it rocked on its feet, at 0.34 rad/s 10 s
      on. Now it stands still from the first step, 49.9 mm up.
    - The IRIS+ dropped. Before, level from 5 and 10 cm and nose first it
      moved up at 1.06 to 1.07 times the speed it struck at, and upright it
      was never at rest; on its edge it came down on its side, still moving
      at 0.08 m/s 10 s on. Now it moves up at most at 0.38 of the speed it
      struck at and rises 9 mm from 30 cm, and comes to rest: upright in
      0.3 to 1.1 s, on its back in 4.0 to 4.4 s, on its side in 5.4 s.
  - The impacts the ground impact's change takes over (its drops from 5 to
    128 m and its tumbles, above): still none
    faster after it strikes than as it struck; the Crazyflie's 5 m drop on
    its back, 1.09 times as fast before, now 1.00. The hops change: the
    Crazyflie's 0.2 to 2.5 m (0.3 to 2.0 before), the IRIS+'s 0.2 to 1.2 m
    (0.15 to 0.85). 3 s later every one is at rest, at 0.008 m/s at most
    (the Crazyflie) and 0.03 (the IRIS+), where before the Crazyflie still
    moved at up to 0.23 m/s. The eight tumbles, still moving at up to
    0.19 m/s 8 s on, are at rest. The found case (a Crazyflie commanded an
    HSA after falling 124 m) lay on its back; now it comes down on its legs
    and flies on, as does the IRIS+ flown so.
  - Hover and flight tests: hangar's `fly` and `performance` stages give the
    same results, bit for bit (`out/fly.json` and `out/performance.json`,
    save the time they took), run on the old files and the new. The new
    aircraft files differ from the old only in their contacts' springs and
    dampers: `hover.toml`, the profile and the tables are unchanged.
  - Control digests (`fsim_control_bench digest`, 20 flights, protection on
    and off): identical. None of them flies a quadrotor.
  - The fleet test, flown before and after with every flight's end state
    printed (docs/flight-autonomy.md, section 14): 3,284 states, 3,249
    identical, every judged state among them but one.
    - 33 are quadrotors on the ground as their cases end - 22 Crazyflies
      and 11 IRIS+, fallen once their batteries ran out, or left
      uncommanded in cases other aircraft fly, or parked. Before, 29 of
      them still hopped or rocked (0.01 to 0.22 m/s, 0.08 to 9.8 rad/s)
      and 4 IRIS+ lay still on their backs. Now 32 of them are at rest -
      among them 11 Crazyflies on their backs that had hopped upright, and
      3 IRIS+ upright that had lain on their backs - and the 33rd, a
      Crazyflie whose battery ran out as its case ended, is still tumbling
      (0.10 m/s).
    - The other two are one Crazyflie hovering at 152 m in the `hold` case,
      as it is judged and as its case ends: they differ in their 15th and
      16th figures. It was spawned into the slot a parked Crazyflie had
      left, and a vehicle spawned into a used slot starts from bits that
      depend on what the slot last held (JSBSim's initial conditions keep
      the last vehicle's position): parked lower now, the Crazyflie left a
      start one unit in the last place away. The same spawn in a fresh
      world is unchanged, bit for bit.
  - `test_rotorcraft` gains the parked and dropped quadrotors: both parked
    never lift a foot and stand still; dropped 5 and 30 cm level and on
    their backs, they never rise half as high as they fell nor move up as
    fast as they struck, and are at rest 6 s on. It fails on the old
    contacts, 28 of its 48 checks. hangar's tests gain the rule's (each
    set's fastest mode and its damping, JSBSim's step shrinking every
    disturbance, the legs' deflection, the old keys refused).
  - `ctest`: 329 of 329, the new case among them.
  - Merged onto main at FA-8d (2026-09-29), on top of the ground impact's
    change as it was written, and measured there the same way; hangar and
    the two quadrotors' files on main were the ones it was written on. Of
    the fleet test's 3,992 states, 3,948 are identical, every judged state
    among them but the Crazyflie hovering in the `hold` case, spawned into
    a used slot (1.4 nm, as above; as its case ends too). The other 42 are
    quadrotors on the ground as their cases end, 31 Crazyflies and 11
    IRIS+: 37 still moved before (over 1 cm/s), 1 now, a Crazyflie
    tumbling as its battery ran out (0.11 m/s). Control digests, with
    protection and without, and the route and curve probes are unchanged;
    `ctest`: 371 of 371.
- **The helicopters on the ground** (section 7), traced with the Python
  SDK, left uncommanded (the vehicle default's neutral) in a world with no
  terrain. The change was held for the owner's word, and merged on the
  owner's instruction to merge the waiting work (2026-09-29).
  - Found in the fleet test: the UH-1H let go from its hover, in the
    aerobatics and formation cases, diverged at 22.97 s (released at 10 s).
    The same fall, flown alone, struck the skids at 16.3 m/s. Dropped level
    with its c.g. 5.1 m up, it strikes at 7.7 m/s, having rolled 3.4 deg
    right in the fall under its tail rotor's thrust.
    - At the first contact step the right skid's contacts are 161 and
      168 mm deep, the left's 30 and 36. Their normal forces pitch it nose
      up with 370,000 lb ft, and within five steps it turns at 0.72 rad/s,
      rotating about the rear contacts.
    - Carrying the c.g. over those contacts lifts it 0.05 m (985 ft lb), so
      any rate above 0.34 rad/s about them tips it over. It passes 14.5 deg
      at 1.31 s, stands at 70 deg at 2.5 s, rolls over, and its skids leave
      the ground at 2.66 s.
    - Upside down in the ground, its rates grow, and at 3.16 s the skids come
      back up through the ground, 1.2 to 4.4 m deep. They throw it off at
      48 m/s, then 189, and it diverges at 3.28 s.
  - Dropped level from lower: struck at 1.6 m/s it pitches 2.3 deg and at
    3.2 m/s 8.3 deg, and both stay upright. At 4.4 m/s (rolled 30 deg), 5.2
    and 7.7 m/s it tips over and diverges.
  - Suspects taken out one at a time (variant files through
    `FSIM_AIRCRAFT_PATH`), each flown in the 5.1 m and 3 m drops, the
    rolled drop and the fall from the hover:
    - Tipped over and diverged as before in both drops and the fall from
      the hover: without the stabilizer bar, the ground effect or the skids'
      friction; with the dampers four times as strong or a quarter; with the
      rear contacts at the skid's heel (station 158.7). Without the friction,
      with the stronger dampers or at the heel, the rolled drop stayed
      upright.
    - Level (0.1 deg): springs and dampers in proportion to each contact's
      static load, their totals kept. The fall from the hover (16 m/s) still
      diverged with them.
    - Upright after every case: a contact at the tail skid.
  - The energy, traced a step at a time on the UH-1H dropped on its back
    from 10 m (the old file): it touches nothing - its skids point up - and
    sinks through the ground. Its rotational energy grows from 900 ft lb
    (1.2 s) to 100,000 (2.2 s) and 55 million (3.0 s). No ground force acts:
    the rotors' moment does work on it at least as fast as it grows, and the
    airframe's aerodynamics take the rest.
    - The main rotor's torque goes from +3,630 lb ft at the drop to
      -120,000 at 2.0 s and -8.3 million at 2.8 s, the rotor at its
      420.8 rpm stop from 2.2 s.
    - The flight control system's rotor speed, which no stop holds, reaches
      1,800 rad/s.
  - The change, in hangar:
    - Structure contacts: the UH-1H gets 16, the UH-60A 18. Springs are
      0.12 to 0.70 MN/m (the UH-1H) and 0.18 to 1.8 MN/m (the UH-60A), each
      contact's own mode at 30 rad/s on its apparent mass, as a fixed
      wing's.
    - The rotors' drive: an external moment about the main rotor's shaft.
    - The UH-1H's skids and cross tubes reach within 0.15 m of the ground
      it stands on, so the hull is found without them.
  - The same cases after (`test_rotorcraft`'s new case). In each: the speed
    at the strike, the fastest after it, the c.g.'s least height after it,
    and the state at the case's end (45 s after the drop; 35 s after the
    release).

    | case | strike | fastest after | c.g. least | then |
    | --- | --- | --- | --- | --- |
    | UH-1H let go from its hover | 16.24 m/s | 16.25 | 1.30 m | on its skids, still |
    | UH-1H dropped on its back from 10 m | 10.53 | 10.56 | 1.00 | on its side, rocking at 1.2 m/s |
    | UH-1H dropped on its side from 5 m | 8.07 | 8.18 | 0.82 | on its side, still |
    | UH-60A let go from its hover | 23.05 | 23.05 | 1.15 | on its wheels, turning at 0.74 rad/s (as before) |
    | UH-60A dropped on its back from 10 m | 11.94 | 12.01 | 1.27 | on its side, still |
    | UH-60A dropped on its side from 5 m | 7.92 | 8.00 | 1.22 | on its side, still |

    Before, the UH-1H let go or dropped on its back diverged, and dropped on
    its side sank 13 m into the ground; the UH-60A fell 440 m and 160 m
    through the ground. With the structure contacts alone, the UH-1H dropped
    on its back still diverged, at 4.5 s: stood on its hub, the rotor's
    torque spun it. The new case fails on both older files. The probe's
    other drops all end whole and on the ground. Level or rolled, the UH-1H
    stays upright, tipped back onto its tail skid (8 to 11 deg) when it
    strikes at 3.2 m/s or more. Nose first, both end on their gear;
    dropped tail first, the UH-1H falls onto its side. Parked, both are as
    they were.
  - What the drive changes in flight, measured by flying hangar's fly and
    performance stages again:
    - The identified hover moves by up to 0.06 % on the UH-1H (its yaw
      damping) and 0.2 % on the UH-60A (its yaw damping, 0.46311 to
      0.462173).
    - At the same speeds, the tables' power and fuel move by at most 1e-5
      (the UH-1H) and 9e-5 (the UH-60A), and the full-power climb by
      1.5e-4 (the UH-60A's ringing climb at 93 m/s by 0.7 %).
    - In two of the UH-1H's rows the top level speed moves by one step of
      its bisection, 0.125 m/s. That moves those rows' speeds, and with them
      their values: the climb at the top speed goes from 3.10 to 2.99 m/s.
    - `hover.toml` and the files' profiles are written again. A second run
      leaves them as they are.
    - The alternative, the freewheel alone (the airframe never takes a
      negative main-rotor torque), leaves the UH-1H's flight tests bit for
      bit but not the UH-60A's. A collective drop dips the UH-60A's torque to
      -11,000 lb ft for a step or two while its engine still gives 60 %
      power, and the airframe's torque would still jump there.
  - The fleet test, flown three times with every flight's end state printed
    (docs/flight-autonomy.md, section 14): the committed files, the
    structure contacts alone, and both changes. Each run has 3,284 states.
    - The contacts alone change 9 states:
      - The three UH-1H flights that diverged now last to their cases' ends,
        on the ground.
      - The UH-60A let go in the same three cases strikes at 23 m/s, touches
        its new contacts, and rests 1.7 to 2.1 m and 41 deg of heading from
        where it did.
      - Let go after the engines case, the UH-60A strikes the ground a
        second before its state is printed, touches its new contacts, and
        is 1.5 m and 16 deg of heading from where it was.
      - Two parked helicopters end within 1e-8 m of where they did. JSBSim's
        ground trim, run as they spawn, sees the new contacts.
    - Both changes: every helicopter state changes (188), and every other
      aircraft's 3,096 are identical.
      - All 82 judged helicopter states pass their thresholds. They moved by
        a median of 1.4 cm; the largest are the UH-1H's loiter (4.0 m after
        158 s) and the UH-60A's (0.97 m).
      - Diverged endings: 3 before, none after.
  - Control digests (`fsim_control_bench digest`, 20 flights, protection on
    and off): identical. They fly no helicopter.
  - Cost: a world of 10 UH-1H and 10 UH-60A, before and after in turn, in
    fresh processes, seven rounds.
    - A vehicle's step, parked: 6.58 us before, 7.98 after (medians).
      Hovering under the velocity loop: 5.85 before, 7.38 after.
    - The contacts account for 1.3 us (0.08 us each), the drive's moment
      for 0.2 us.
    - The helicopters now carry as many structure contacts as the C-130J,
      the F-35A and the C-17A (the fleet's designs carry 5 to 18).
  - Tests: `hangar` gains the helicopters' contacts (on the airframe, clear
    of the gear, each soft enough for the step); `test_rotorcraft` gains the
    case above.
  - Merged onto main at FA-8d (2026-09-29), after the quadrotors' contacts,
    and measured there the same way. hangar's paths for the two do not
    meet (the legs' check is a multirotor's; `structure_points` takes more
    points for the helicopters alone), so each change's aircraft files
    stand as built. Of the fleet test's 3,992 states, the 232 that change
    are every helicopter state: 104 as flights are judged (1.4 cm apart at
    the median, 4.0 m at most, round an orbit) and 128 as cases end. The
    other 3,760 are identical. The fleet's flights ended diverged 3 times;
    now none. The route end stop's helicopters (flight-autonomy.md,
    section 14) read as they did to the printed precision, but for the
    UH-1H after a turn in its tailwind: within 1.31 m of its point, where
    1.30. Control digests, with protection and without, and the route and
    curve probes are unchanged; `ctest`: 372 of 372.
- **The helicopters' rotor speed stopped with the rotors'** (2026-09-30;
  hangar's `heli.py`, section 7). The flight control system integrates the
  rotor speed from the engine's torque less the rotors', and it drives the
  governor and the rotor-drive moment. FGRotor stopped at 50 rpm (hangar's
  least, since 1 rpm: the next block) and 130 %;
  the integration did not. Collective down and the cyclic forward from
  2,000 m, the air drives the rotor to FGRotor's stop, and the flight
  control system's rotor speed wound on: the UH-1H's to 895 % in 40 s, the
  UH-60A's to 480 %. Now, past a stop and pushed further, the integration is
  held and its output clipped there. Flown so again, both read the rotor's
  own most, to the hundredth of an rpm (420.81 and 335.18 rpm). No flight
  had reached the stop: the fleet test's 3,992 states are identical, bit
  for bit. `test_rotorcraft` gains the dive, which fails on the old files
  (2,152 and 1,238 rpm).
- **The blades strike the ground** (2026-09-30; hangar's `heli.py`, its FCS
  channel "blade strike"). When the main rotor's disc reaches the ground -
  the hub's height above it (the c.g.'s, less the hub's offset turned by
  the attitude) below the most the disc's edge drops under the hub at the
  shaft's tilt - the blades have struck, and stay so until a reset. The
  engine then gives nothing, and the rotor is braked to a stop in 2 s: its
  least speed is now FGRotor's own 1 rpm (hangar gave 50, at which the
  UH-1H's rotor still made 2.4 % of its hover thrust). The strike counts
  once the flight has begun: a start's ground trim, at time 0, searches
  through the ground (it first latched every parked helicopter).
  - Dropped on its side or back from 5 and 6 m, both strike, their engines
    cut and their rotors at 1 rpm; parked or hovering at 3 m, neither does.
    `test_rotorcraft` checks it (it fails on the old files).
  - No fleet flight strikes: its 3,992 states are identical, bit for bit;
    digests unchanged; `ctest`: 377 of 377.
  - The UH-1H on its side still rocks in some drops: its structure
    contacts', not its tail rotor's (section 7).
- **The quadrotors' feet on the ground as drawn** (2026-09-30; hangar's
  `multi.py`). Sized for the step, the feet's contacts sink 2.9 mm (the
  Crazyflie) and 4.1 mm (the IRIS+) under the weight, and the contacts were
  at the drawn feet: at rest the viewer drew the feet into the ground. The
  contacts are now that deflection lower, the springs and dampers the same
  (they are sized from the feet's places across the airframe, not their
  height). Parked, each stands its legs' height up, to 0.1 mm (it stood
  2.89 and 4.08 mm lower); `test_rotorcraft` checks it, and fails on the
  old files. The model stage's check now takes where the contacts settle
  (0.08 mm and 0 from the drawn skin). The fleet test's judged states are
  identical; 42 end states change, quadrotors on the ground as their cases
  end, those that fell coming to rest elsewhere (3 cm at the median,
  0.36 m at most). Control digests and the route and curve probes are
  unchanged; `ctest`: 376 of 376.
- **The UH-1H's skids sized by the load they carry** (2026-09-30; hangar's
  `heli.py`, `skid_shares`). Its four skid contacts had the design's spring
  and damper alike, but its c.g. stands 15 in ahead of the rear pair and
  105 in behind the front: the rear pair carries seven eighths of the
  weight. Each contact now takes its share by the lever rule, the design's
  values their mean: 5,000 lbf/ft and 500 lbf s/ft at the front, 35,000 and
  3,500 at the rear. Standing, each sinks alike. Parked 10 s it sits at
  -0.01 deg of pitch, where it sat at +0.67 deg. Dropped level,
  uncommanded, striking at 3.8, 6.1 and 12.3 m/s, it pitches up by 0.0 deg,
  where it pitched +9.5, +10.2 and +9.1 deg onto its tail skid. The UH-60A,
  on its wheels, is unchanged. The fleet test's judged states are
  identical; 4 end states change, the UH-1H on the ground as its cases end
  (let go, it comes to rest 0.6 m from where it did). `ctest`: 376 of 376.
