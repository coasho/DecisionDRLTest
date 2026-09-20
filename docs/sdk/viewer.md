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

Until a world exists the window says "waiting for a training application";
when your program starts, the vehicles appear with their names, types and
active control levels; when it exits, the viewer says so and waits for the
next one. Restarting the trainer re-attaches automatically.

## What the viewer shows

- Every live vehicle, as a 3D model over full-Earth satellite imagery and terrain, with a label, a trail and a chase/orbit/overview camera (`tab` cycles vehicles, mouse orbits/zooms).
- Vehicle creation, reset (new "generation") and removal, as they happen.
- The world's clock (sun position and sky), wind, atmosphere and weather.
- The trainer's throughput (vehicle-steps/s), simulation time and the age of the last update.
- Per vehicle: state summary and the active control level.

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
own rendering competing for CPU, not the publishing; the viewer caps itself at
60 fps and uses ~4% of one core).

## Options that matter to a trainer

| `WorldOptions` | Effect |
| --- | --- |
| `name` | what the viewer lists and attaches to; use a distinct name per concurrent experiment |
| `publish = false` | no segment at all (e.g. headless sweeps on a cluster) |
| `publishIntervalSeconds` | the copy rate limit; raise it to reduce the copy further, lower it for smoother motion at real-time pace |
| `capacity` | vehicle slots in the segment (256 by default); vehicles beyond it simulate but are not shown |
| `VehicleSpec::model` | optional glTF path shown instead of the default aircraft model |

## Limits (current)

- One machine: the segment is local shared memory (a network relay is a planned `ext/` module).
- The viewer uses one aircraft model for every vehicle unless a `model` override is given (per-type models are next).
- The trainer's physics ground is flat unless it configures a `GroundProvider`; the viewer draws real terrain, so a low-flying vehicle can appear below hills. A terrain provider in the SDK (same tiles as the viewer) is on the roadmap.
