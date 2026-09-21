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

- Every live vehicle, as a 3D model over full-Earth satellite imagery and terrain, with a label, a trail and a chase/orbit/overview camera (`tab` cycles vehicles; the mouse works like OpenSceneGraph's manipulators: left drag rotates, middle drag pans, wheel zooms from 6 m to the whole Earth, right drag zooms while following a vehicle; detached (camera mode "free", or before any vehicle exists) the middle and right buttons drag the globe like osgGA's TerrainManipulator; the camera never enters the terrain).
- Vehicle creation, reset (new "generation") and removal, as they happen.
- Several training applications at once: the monitor lists every live world and switches between them (`--world <name>` picks one at start).
- Per-type models: a vehicle of type `jsbsim:f16` is drawn with `models/f16.glb` (or `.gltf`) if such a file exists in an asset directory (`<exe>/../share/flightsim/models`, the source tree's `assets/models`, or a `--assets` path), else with the platform's default aircraft; `VehicleSpec::model` names a file explicitly. Files are loaded once, on first use, and shared by every vehicle of that type; a `<file>.manifest` (`forward`, `up`, `scale`) fixes axes and size.
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
| `VehicleSpec::model` | optional glTF path shown instead of the type's model (`models/<type>.glb`) or the default aircraft |
| `recordPath` | also write the run to a `.fsrec` file; `flightsim-viewer.exe --replay <file>` plays it back (space pauses, `.` steps one frame, `[` `]` change the time factor, the timeline slider seeks, `home` restarts, loops at the end) |

## Screenshots

`flightsim-viewer.exe --screenshot shot.png [--screenshot-after 5]` saves the
window as PNG after the given number of seconds and exits (any mode: mirror,
`--demo`, `--replay`); the copy is taken from the swapchain, so it works on a
locked desktop and in scripts. `--view lat,lon,alt,dist[,az,el]` starts with
the free camera looking at a point from `dist` metres (whole-Earth views:
`--view 20,-30,0,30000000,180,80`); above the atmosphere the sky fades to
black.

## If it crashes

`flightsim-viewer.exe` and `flightsim.exe` install a crash handler: an
unhandled exception writes a minidump to `%LOCALAPPDATA%/flightsim/crash/`
(`<exe>-<date>-<time>.dmp`, openable in Visual Studio or WinDbg) and one line
to stderr / `viewer.log`. Libraries never install one - a trainer owns its own
process.

## Limits (current)

- One machine: the segment is local shared memory (a network relay is a planned `ext/` module).
- Only one sample aircraft model ships (Cesium Air); drop `models/<type>.glb` files next to it for your own types.
- The trainer's physics ground is flat unless `WorldOptions::terrain` is on (then it is the same relief the viewer draws); with it off a low-flying vehicle can appear below hills.
