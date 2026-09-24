---
name: aircraft-design
description: Design, analyse and validate a new aircraft for JSBSim with hangar (tools/hangar), from a description, drawing or photo to a flight-tested jsbsim:<name>. Use when asked to model an aircraft that has no JSBSim model or wind-tunnel data, to change a design's geometry, mass or engine, or to explain how a design flies.
---

# Aircraft design with hangar

hangar turns one TOML file (`aircraft/<name>/<name>.toml`) into a JSBSim
aircraft, a glTF model and a validation report. docs/hangar.md covers the
stages, the file format and the methods. This skill is the working loop.

## Run it

```bash
./fsim.cmd hangar list
./fsim.cmd hangar new <name> --like skua     # or --like c172
./fsim.cmd hangar <name> geometry            # any stage, or several in order
./fsim.cmd hangar <name> --quick             # all stages, coarse tables: ~30 s
./fsim.cmd hangar <name>                     # all stages, full tables: ~1-2 min
./fsim.cmd hangar <name> calibrate           # only with real performance data
```

The outputs go to `aircraft/<name>/out/`: `<stage>.json` (the numbers and
checks), the PNGs, and `report.html`. A fail makes the exit code 1.

## 1. Gather, then write the design

- Collect the real numbers first: span, chords, sweep, dihedral, airfoils,
  tail sizes and positions, fuselage length and sections, empty and gross
  mass, CG range, engine power and rpm, propeller diameter and pitch.
- Collect the performance targets too: stall speed, top speed, climb rate
  and ceiling. They go in `[targets]`.
- **From a picture.** Read it, find one known dimension (span, length or
  wheelbase) and scale everything else from it. Note the scaling in a
  comment.
- Every number in the TOML gets a comment: its source, or "assumed" with the
  reason. The design file is the record.
- **Frame.** Metres, with x aft, y right and z up. Put the origin where the
  sources measure from; JSBSim uses the same frame. Give wings and tails for
  the right half; they are mirrored. Fins and bodies are mirrored only with
  `mirror = true`.

## 2. Look after every stage

Read the images with the Read tool and compare them with the sources. Don't
move on while a check fails or a picture looks wrong.

- **geometry.** Read `threeview.png` and `views.png` next to the reference
  drawing or photo: proportions, tail positions, gear, propeller disc. The
  usual ranges are:
  - horizontal tail volume 0.4–0.9, vertical 0.02–0.08;
  - wing loading 30–120 kg/m² (GA), 5–25 kg/m² (small UAV).
- **aero.** Read `coefficients.png`.
  - CL(α) should be linear with a slope of 4–6 per radian, break at 12–20°
    and keep 60–80 % of its peak after the stall.
  - Cm(α) should fall steadily through −10…+20°.
  - Every curve should be smooth. A kink or spike usually means one surface
    sits right in the wake plane of another; move it about 5 % of the chord
    up or down.
  - Read `derivatives.png` too: Cmq, Clp and Cnr stay negative before the
    stall.
- **mass.** Static margin 5–25 % MAC loaded. Radii of gyration outside
  Roskam's bands point at a mass item placed wrongly.
- **propulsion.** Read `propeller.png`. Peak efficiency should be 0.75–0.88.
  A zero-thrust advance ratio far from pitch/diameter means a wrong pitch.
- **verify.** This must pass: it checks the XML writer, not the design. If
  it fails, the fix is in `tools/hangar/hangar/jsbsim.py`.
- **fly.** Read `fly_trim.png`, `fly_stall.png`, `fly_climb.png`,
  `fly_longitudinal.png` and `fly_lateral.png`.
  - Throttle against speed is U-shaped. Trim elevator stays well inside its
    limits.
  - The stall shows a clear break, and the aircraft recovers.
  - The 3-2-1-1 responses settle.
  - Compare the linear model's modes with "JSBSim response" in the checks.
    A big difference is worth explaining.
  - The crash checks put the aircraft into the ground six ways. A speed gain
    in a cartwheel is JSBSim's wheel model (docs/hangar.md, "Limits"); in
    any other case, look at the ground contacts in `<name>.xml`.
- **calibrate.** Run it only against real published performance. It fits an
  extra drag area and the propeller pitch; say what it fitted. A GA aircraft
  should need well under 1 m², and the pitch should land near the real
  propeller's. A large correction means the geometry is wrong. Fix that
  instead.
- **In the viewer.** Take a screenshot, then Read the PNG:

  ```bash
  build/ucrt64-release/bin/flightsim-viewer.exe --demo --vehicles 2 --aircraft <name> --alt 400 --screenshot <scratchpad>/<name>.png --screenshot-after 10
  ```

  The log line "vehicle model: …<name>.glb" confirms the viewer used the
  design's own model, and "N moving control surface(s)" that its hinges
  were found. `ctest -R "control surface"` checks the hinge directions.
  To look at deflections, publish a world where the aircraft holds its
  controls, stop stepping it, then attach
  `--world <name> --camera chase --chase-distance 12`.

## 3. Usual fixes

| symptom | first thing to try |
|---|---|
| Cm_alpha positive, or margin too small | move the wing aft or the CG forward; a bigger tail or a longer tail arm |
| elevator can't trim the stall | more elevator chord or tail area; CG further aft |
| dutch roll damping < 0.08 | more fin area or fin arm |
| spiral divergent too fast | more dihedral, or less fin |
| roll too sluggish | longer aileron span, placed further out |
| trims in `fly.json` say `"method": "flown"` | nothing: JSBSim's own trim failed (common for small electric aircraft), so hangar flew the aircraft to trim |
| piston engine quits at altitude | the generated `fcs/auto-mixture` leans it with altitude; check that it drives `fcs/mixture-cmd-norm` |

## 4. Finish

- Run `ctest --test-dir build/ucrt64-release -R hangar`.
- Show the user three things: the three-view, a viewer screenshot, and the
  failed or warned checks with what they mean.
- Link `aircraft/<name>/out/report.html`.
- The aircraft is `jsbsim:<name>` everywhere on the platform, with no copying:
  in the viewer, the C++ and Python SDKs, and scenario files.
