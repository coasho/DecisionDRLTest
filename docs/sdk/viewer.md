# Transparent visualisation

`flightsim-viewer.exe` is a separate, prebuilt program. It finds your world
by itself, shows it, and follows every change. Your training application
contains no visualisation code and does not know whether a viewer is running.

## What you do

Nothing. Create a `World` (with `publish = true`, the default) and step it.
Start the viewer whenever you like, before or after the trainer, on the same
machine:

```bash
build/ucrt64-release/bin/flightsim-viewer.exe              # attach to the newest published world
build/ucrt64-release/bin/flightsim-viewer.exe --world my-experiment
build/ucrt64-release/bin/flightsim-viewer.exe --list       # print live worlds
```

The viewer is a GUI application: no console window appears when it is
started from Explorer (its log then goes to `%LOCALAPPDATA%\flightsim\viewer.log`);
started from a terminal it prints there (`--list`, `--help`, `--stats`).

Until a world exists the window says "waiting for a training application";
when your program starts, the vehicles appear with their names, types and
active control levels; when it exits, the viewer says so and waits for the
next one. Restarting the trainer re-attaches automatically.

## What the viewer shows

- Every live vehicle, as a 3D model over full-Earth satellite imagery (to level 19, ~0.3 m/px) and terrain (30 m data to level 15, smoothly refined below it so the imagery keeps sharpening at low altitude), with a label, a trail and a chase/orbit/overview camera (`tab` cycles vehicles; the mouse works like OpenSceneGraph's manipulators: the bindings follow osgEarth's EarthManipulator - left drag pans, middle drag rotates, the wheel zooms from 6 m to the whole Earth and changes nothing but the distance (`--zoom-to-cursor` for osgEarth's zoomToMouse); detached (camera mode "free", or before any vehicle exists) left drag moves the globe, including across the poles; the right button is not a camera control; the camera never enters the terrain).
- Vehicle creation, reset (new "generation") and removal, as they happen.
- Several training applications at once: the monitor lists every live world and switches between them (`--world <name>` picks one at start).
- Per-type models: a vehicle of type `jsbsim:f16` is drawn with `models/f16.glb` (or `.gltf`) if such a file exists in an asset directory (`<exe>/../share/flightsim/models`, the source tree's `assets/models`, or a `--assets` path), else with the platform's default aircraft; `VehicleSpec::model` names a file explicitly. Files are loaded once, on first use, and shared by every vehicle of that type; a `<file>.manifest` (`forward`, `up`, `scale`) fixes axes and size. Aircraft made with [hangar](../hangar.md) carry their own model (`aircraft/<name>/<name>.glb`), found the same way.
- Moving control surfaces, for models that mark them (see below): ailerons, elevator, rudder and flaps follow each vehicle's deflections, from the trainer or the replay.
- What the engines and the air do (see [Effects](#effects)): afterburner flames, glowing nozzles and hot exhaust; wing tip vortices, vapour over the wings and the transonic vapour cone; contrails high up; streaks of air flowing past the airframe that show the speed.
- The world's clock (sun position and sky), wind, atmosphere and weather.
- The trainer's throughput (vehicle-steps/s), simulation time and the age of the last update.
- Per vehicle: state summary and the active control level.
- The trainer's cameras (`fsim_vision`, [vision.md](vision.md)): a "cameras" window with every image the training application rendered last, live (`v` toggles it).

## How it works (design 9.7)

The `World` maps one named shared-memory segment (`Local\fsim.world.<name>`)
holding a header, a vehicle table and three snapshot slots, and registers the
name in a tiny registry segment for discovery. After each `step()` it copies
the vehicle states into the next slot **if at least `publishIntervalSeconds`
of wall time have passed** (16 ms by default), behind a sequence counter. That
is the whole cost: a bounded `memcpy` at most ~60 times per second, no locks,
no waiting, no allocation, no notice of readers. A trainer running 5,000x
real time publishes at 60 Hz, not at 600 kHz; a trainer that pauses is simply
shown as "not updating".

The viewer reads slots under a seqlock (retrying if a copy is overwritten
while it reads), interpolates between the two newest snapshots in wall time
so motion is smooth at any simulation pace, and re-reads the vehicle table
whenever its generation counter changes.

Measured on an 8-core desktop with 64 vehicles: 760k vehicle-steps/s alone,
within a few percent with a viewer attached (the difference is the viewer's
own rendering competing for CPU, not the publishing; the viewer is paced by
vsync and uses ~3% of one core while mirroring 64 vehicles).

## Options that matter to a trainer

| `WorldOptions` | Effect |
| --- | --- |
| `name` | what the viewer lists and attaches to; use a distinct name per concurrent experiment |
| `publish = false` | no segment at all (e.g. headless sweeps on a cluster) |
| `publishIntervalSeconds` | the copy rate limit; raise it to reduce the copy further, lower it for smoother motion at real-time pace |
| `capacity` | vehicle slots in the segment (256 by default); vehicles beyond it simulate but are not shown |
| `VehicleSpec::model` | optional glTF path shown instead of the type's model (`models/<type>.glb`, a design's `aircraft/<type>/<type>.glb`, or the design registered to stand in for a stock aircraft in `aircraft/models.txt`) or the default aircraft |
| `recordPath` | also write the run to a `.fsrec` file; `flightsim-viewer.exe --replay <file>` plays it back (space pauses, `.` steps one frame, `[` `]` change the time factor, the timeline slider seeks, `home` restarts, loops at the end) |

## Moving control surfaces

A glTF node named `fsim:<channel>` turns with the vehicle's deflection of that
channel. The channel is one of:

- `aileron` - the left aileron's position; give the right one a gain of `-1`,
  or turn its axis the other way;
- `elevator`;
- `rudder`;
- `flaps` (`flap` works too).

The node turns about its own x axis, by the deflection in radians. JSBSim's
sign conventions apply: elevator and flaps trailing edge down, aileron left
trailing edge down, rudder trailing edge left. An optional gain scales the
deflection: `fsim:aileron:-1`, `fsim:elevator:0.5`. A part that several
channels move adds them: `fsim:elevator+aileron:-1` (an elevon). A trailing
`@<lo>,<hi>` holds the turn within the part's own stops, in degrees:
`fsim:elevator@-20,50` for a canard that travels further than the elevons
sharing its channel.

Other moving parts follow the vehicle's state too - the same numbers your
code reads from `state()`, so what you see is what the simulation did:

- `fsim:gear:<deg>[:<g0>:<g1>]` turns by `deg` as the gear position (1 down,
  0 up) goes from `g0` to `g1` (default 1 to 0): a leg, or a door
  (`fsim:gear:90:0:0.2` opens in the first fifth of the way down). Nest two
  for a leg that swings and twists at once.
- `fsim:lef[:<gain>][@<lo>,<hi>]` is a leading-edge flap, turned by
  `leadingEdgeFlapRad` (where the aircraft's flight controls put them,
  `fcs/lef-pos-deg`; 0 for an aircraft without) times the gain, held within
  its stops.
- `fsim:propeller:<engine>` turns about its x axis at that engine's
  `engineRpm`; it stops with the engine and holds still while the clock does.
- `fsim:nozzle:<engine>:<deg>` turns a nozzle petal open by `deg` times the
  engine's `nozzlePosition` (JSBSim's turbine: shut at military power, open at
  idle and with the afterburner lit).
- `fsim:afterburner[:<engine>]` marks where the engine's jet leaves the
  nozzle: its origin at the exit, its x axis along the jet. The viewer draws
  the exhaust there (see [Effects](#effects)), as big as the node's own
  geometry is across its x axis, and turned with the node, so a nozzle that
  vectors takes its flame with it. With the effects off, the node's own mesh
  is the flame, stretched along x with the engine's `afterburner` (0 out ..
  1 full) and hidden when it is out.
- `fsim:oleo:<wheel>[:<gain>]` slides along its x axis by the unit's
  `wheelCompressionM` (times the gain): a strut's piston, so a parked
  aircraft stands on its wheels.
- `fsim:steer:<wheel>` turns about its x axis by `wheelSteerRad`.
- `fsim:wheel:<wheel>:<radius>` rolls about its x axis at `wheelSpeedMs`
  over the radius, and spins down in the air.

`<wheel>` is the unit's place among the aircraft's wheeled gear (JSBSim's
BOGEY contacts, in the aircraft file's order - the nose wheel, then the left
and right mains, for most).

To make a model's surface move:

1. Put the node's origin on the hinge line.
2. Turn the node so its x axis runs along the hinge, pointing the way that
   makes a positive rotation the positive deflection.
3. Put the surface's mesh under the node.

Every vehicle turns its own copy of these nodes; the geometry under them is
shared. Models written by hangar are built this way. The vision cameras and
their segmentation images show the same deflections.

## Effects

Drawn from each vehicle's state as the simulation reports it - nothing is
added to `VehicleState` for them - after the effects DCS World shows:

| Effect | When | From |
| --- | --- | --- |
| Afterburner flame: a white-hot core with shock diamonds, an orange envelope, blue at the nozzle while barely lit, flickering | afterburner lit; longer the harder | `afterburner`, `nozzlePosition` |
| Nozzle glow | dull red at military power by night; bright with the afterburner | `engineN2`, `afterburner` |
| Heat haze behind the nozzle | military power | `engineN2` |
| Wing tip vortices, left behind in the air | a hard pull: angle of attack at the tip (roll adds to it on the rising wing) and load factor; less in the drier air high up | `alphaRad`, `loadFactor`, `angularRateBodyRadS` |
| Vapour over the wings and off the leading-edge extensions, flickering | a harder pull still | `alphaRad`, `loadFactor` |
| Vapour cone round the airframe | Mach 0.93 - 1.03, low down | `mach`, `altitudeMslM` |
| Contrails, starting a little behind the nozzles, spreading and fading over a minute | air colder than -40 C (the standard atmosphere from the world's sea-level temperature: about 8.5 km and up on a standard day) and the engine working | `altitudeMslM`, `engineN2` |
| Air streaks flowing past the airframe | faster: longer and denser; a change of speed shows as a change in them | `airspeedTrueMs`, `alphaRad`, `betaRad` |

`e` switches them off and on, `--no-effects` starts with them off, and
`"effects": false` in `viewer.json` makes that the default. Off, the flames
are the models' own meshes, as before.

They cost little: the GPU shapes and animates everything from a few numbers
per vehicle and the clock (the simulation's, so a paused or replayed world
shows the same), one small mesh is shared by every engine, the trails are
rings of samples the vertex shader turns into ribbons facing the camera, and
vehicles far from the camera draw nothing. With the 14 fighters of
`examples/python/fighters.py` in view, uncapped, a frame took 1.1 ms without
them and 1.2 ms with them; all 14 in afterburner pulling 6 g at 10.5 km,
every effect showing at once, 1.4 ms (an RTX 5070 Ti).

The exhaust is part of the vehicle's model, so the vision cameras' colour
images show it too; the rest is the viewer's alone. Segmentation images
never include any of it: an id covers the vehicle itself.

A model made by hangar also tells the viewer where its wings and strakes are
(`wing`, `strake` and `canard` lines in its `.glb.manifest`: per section of
the right half, its leading edge in body axes from the model's origin and
its chord). A model without them gets the tip vortices from its widest point
and no vapour over the wings.

## Screenshots

`flightsim-viewer.exe --screenshot shot.png [--screenshot-after 5]` saves the
window as PNG after the given number of seconds and exits (any mode: mirror,
`--demo`, `--replay`); the copy is taken from the swapchain, so it works on a
locked desktop and in scripts. `--view lat,lon,alt,dist[,az,el]` starts with
the free camera looking at a point from `dist` metres (whole-Earth views:
`--view 20,-30,0,30000000,180,80`); above the atmosphere the sky fades to
black. `--camera chase --chase-distance 15` starts right behind the selected
vehicle, also when attached to a trainer's world (which otherwise starts with
a regional view); `--chase-azimuth` and `--chase-elevation` move the camera
around it.

## If it crashes

`flightsim-viewer.exe` and `flightsim.exe` install a crash handler: an
unhandled exception writes a minidump to `%LOCALAPPDATA%/flightsim/crash/`
(`<exe>-<date>-<time>.dmp`, openable in Visual Studio or WinDbg) and one line
to stderr / `viewer.log`. Libraries never install one - a trainer owns its own
process.

## Limits (current)

- One machine: the segment is local shared memory (a network relay is a planned `ext/` module).
- One sample aircraft model ships (Cesium Air, fixed surfaces) besides the hangar designs; drop `models/<type>.glb` files next to it for your own types.
- The trainer's physics ground is flat unless `WorldOptions::terrain` is on (then it is the same relief the viewer draws); with it off a low-flying vehicle can appear below hills.
