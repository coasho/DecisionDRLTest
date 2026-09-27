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
  360 deg for the UH-60A.
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
  rotors' hubs and its top (STRUCTURE contacts).
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
| Velocity | `rotor_velocity` | a ground velocity - or an airspeed along the heading - to attitude within a tilt limit, the vertical speed to the thrust control, the heading held on its own; the integral's band smooth, not a hard band, which had left a dead zone |
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
  contacts (`top_m`);
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
- **The Crazyflie at extreme rates**: tumbling into the ground at ~100 rad/s
  (two motors at 90 % for 0.4 s), its 1.4e-5 kg m2 inertia spins up beyond
  what the platform's fixed 120 Hz step can integrate; the platform reports
  the divergence (`VehicleState::diverged`). A gentler tumble comes to rest on
  its back, rocking a few degrees on its top contacts.
- **A spent battery**: a quadrotor whose battery is spent falls, and strikes
  the ground at 40 to 50 m/s from a few hundred metres. There its legs'
  contacts throw the IRIS+ back up, and the Crazyflie diverges: a crash the
  contact model does not end.
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
