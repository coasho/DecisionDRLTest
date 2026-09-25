---
name: aircraft-design
description: Design, analyse and validate a new aircraft for JSBSim with hangar (tools/hangar), from a description, drawing or photo to a flight-tested jsbsim:<name> - light aircraft, UAVs, jet fighters with fly-by-wire, and bombers, transports and other support aircraft. Use when asked to model an aircraft that has no JSBSim model or wind-tunnel data, to change a design's geometry, mass or engine, or to explain how a design flies.
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
    or a blow-up there means the ground contacts add energy: look at them in
    `<name>.xml`.
- **calibrate.** Run it only against real published performance. It fits an
  extra drag area and the propeller pitch; say what it fitted. A GA aircraft
  should need well under 1 m², and the pitch should land near the real
  propeller's. A large correction means the geometry is wrong. Fix that
  instead.
- **model.** The `.glb`. Its checks must pass: no open, pinched or
  misturned edge, the airframe in one piece, every moving part closed, the
  stowed gear inside the skin (under 3 cm), length, span and height within
  3 % of `[dimensions]`.
  - The stage re-meshes only when the geometry, the mass (the CG), the
    paint or the code that shapes the model changes, never for an aero
    edit. The mesher is deterministic: a `.glb` that git shows as modified
    has changed, and one that should not have is a bug to chase.
  - A loose piece is usually a fin or boom standing off its body. Give the
    surface a body under it (a tail boom), or move it onto one.
  - Gear outside the skin: move the `attach` (the trunnion) up inside the
    structure, where the real one is; a leg hinged below the skin cannot fold
    in. Give `retract` from the real aircraft.
  - Shape: intakes are `[[intake]]` bodies (lip, rake, sweep, duct). Bellies
    and chines come from `chine`, `n_top`, `n_bottom`. Canopies take `frames`.
    Leading-edge flaps and slats go in the surface's `leading` list.
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

## 4. Fighters

`aircraft/f16c/f16c.toml` is the worked example; copy its layout.

- **Names.** Don't reuse a JSBSim aircraft's name (f15, f16, f22): the design
  would shadow it, and it could no longer be the reference. Use f16c, f15c,
  and so on.
- **Surfaces.**
  - Strakes and LEX are `kind = "strake"`, `airfoil = "plate"`. Start the
    root at the fuselage centre line, as the wing does.
  - All-moving tails and canards take `chord_fraction = 1.0` and a `pivot`.
  - Surfaces that several channels move take `mix`: a stabilator that rolls
    `mix = { aileron = 0.25 }`, a flaperon `mix = { flap = 1.0 }`, an elevon
    `channel = "elevator", mix = { aileron = 1.0 }`, a canard a negative
    `gain`.
  - Every control stops at its own `limits`; the channel runs as far as its
    furthest control. Give a canard its leading-edge-down travel,
    `gain = -1.0, limits = [-50, 20]`: at high angle of attack it is the
    nose-down control.
- **Airfoils.** `naca64aXYZ` (NACA 6A), `biconvexN`, or `plate`. The
  leading edge's sharpness decides the vortex lift: sharp edges make it,
  round ones hold their suction.
- **Engine.** `type = "turbofan"` with `thrust_dry_kn`, `thrust_wet_kn`,
  `bypass_ratio`, TSFCs, a `[engine.nozzle]` position and an `inlet_x`.
  Mass and size come from Raymer if not given.
- **Flight controls.** `[flight_control] type = "fbw"` with `n_max`,
  `n_min`, `alpha_max_deg` and `roll_rate_deg_s` from the real aircraft.
  - Build checks the short period at every design point, and how far past
    `alpha_max_deg` full nose-down control still brings the nose down.
  - Fly checks the limiter (full aft stick), a 3 g step from trim and the
    roll.
  - A 3 g step that overshoots is a gain problem, not a balance one: the
    control law already cancels the pitching moment's kinks (fcs.py).
- **Balance.** Real CGs are rarely published. Place the CG from the neutral
  point the aero stage finds, not from a guessed % MAC: canard deltas 3-10 %
  MAC unstable, relaxed-stability fighters 0-5 %. The main wheels bound it:
  at least 15 deg tip-back (the CG's height against its distance ahead of
  the main wheels), 8-20 % of the weight on the nose wheel. Move
  `empty_cg`, the tanks and `aero_point` (at the CG) together. Build then
  checks the nose-down reach: full nose-down control must bring the nose
  down at least 2 deg past `alpha_max_deg`, or the limiter can let the
  aircraft hang there. If it does not, move the CG forward or check
  `alpha_max_deg` against the real aircraft's published limit. Say in the
  file that the CG comes from the neutral point: when a change to the
  aerodynamic model moves it, those CGs move with it, their `.glb` is made
  again (its origin is the CG) and `fsim hangar register` is run again.
  The main wheels still bound them: check the tip-back angle and the nose
  wheel's share again (the Mirage 2000's CG stops at 15 deg tip-back).
- **Targets.** Give `max_mach` and `max_mach_altitude_ft`: calibrate fits
  the engine's throttle ratio to them, then the wave drag if the engine
  alone cannot. `max_speed_ktas` (sea level), `climb_rate_fpm`,
  `service_ceiling_ft` and `sustained_turn_deg_s` stay checks. Published
  climb rates are loose, so don't fit to them. A published ceiling of
  50,000 ft is usually a clearance, not where the climb runs out: give it
  as `operational_ceiling_ft`, which the model must reach.
- **Reference.** `reference = "jsbsim:<name>"` plots that aircraft's
  coefficients against the design's (`reference.png`). It is worth doing
  only when the JSBSim model comes from wind-tunnel data: of those shipped,
  only `f16` does (the f15's and f22's headers say performance data). Check
  its control tables' units first: see the f16c's `reference_control_scale`.
  The comparison takes the reference's moments about the design's
  `aero_point`, so keep that at the reference's moment point (the f16c's:
  NASA's 0.35 MAC), not at the CG.
- **Read after aero:**
  - `reference.png`, if there is a reference.
  - `mach.png`: K_L should peak near Mach 0.9-1.0. The neutral point
    should move aft 15-30 % MAC supersonic.
  - `coefficients.png`: CL max 1.5-2.0 at 30-40° is normal for vortex-lift
    fighters.
- **Read after fly:** `fly_fighter.png`, for the excess power at sea level
  and at the top speed's height, the turn, and the best excess power over
  height (the ceiling).
- **The model's shape.** Build it from the real aircraft's drawings, not
  from a template: its own intakes (a chin intake, D-shaped side intakes,
  carets, nacelle mouths), its canopy's frames, its tail booms. Measure a
  public three-view: scale it by the published length (with the probe when
  the length includes one) and check its span and height against the
  published ones; read heights from its ground line or the published
  height, with the wing root's chord plane as z = 0; put the wheels where
  it draws them, then compare the model's silhouettes with the drawing's.
  Check the control layout against the type: a tailless canard delta has
  elevons along its whole trailing edge, not flaps; most fighters carry
  leading-edge flaps or slats. Read the four-view screenshots against
  photos of the real one.
- **Known model limits.** Tails on booms beside the engines (Su-27, MiG-29)
  come out too stable: the lattice carries the tail across the gap between
  them. High-thrust engines fitted to Mach 2.5 at altitude are too fast at
  sea level. Leading-edge devices move in the 3D model only; the tables do
  not model them.
- **Thrust vectoring.** `[engine.nozzle] vectoring` (deg each way) and
  `vectoring_cant` (deg its plane leans outboard; 0 pitch only): the
  nozzles turn with the surfaces and the fly-by-wire scales its gains with
  the thrust (the F-22A, the Su-57). The angle-of-attack limit stays the
  aerodynamic one.

## 5. Large aircraft (bombers, transports, tankers, AEW&C)

- **Induced drag.** Give `[analysis] induced_drag = "trefftz"`: the strips'
  own tilt makes a swept transport wing's lift-to-drag ratio a quarter low.
- **Wings.** A thick, round-nosed wing does not make vortex lift, however
  swept: give its surfaces `vortex = false`, or a 35 deg wing will not stall
  (CL max above 2).
- **Engines.** High-bypass turbofans: `type = "turbofan"`, no
  `thrust_wet_kn`, the real `bypass_ratio`. Pods on pylons: a nacelle body
  (or an `[[intake]]` with a lip) on `[[strut]]`s. More than four engines is
  fine; the platform's throttles 1-4 drive the rest.
- **Mass.** Give `empty`, `empty_cg`, `gyration` and every tank (a tank's
  `fill` is the fuel it starts with). hangar's structural estimates for jets
  are fighter equations: give component masses where they matter.
- **Controls.** Direct, unless the aircraft is fly-by-wire or its dutch
  roll needs it; then `type = "fbw"` with its own limits (n_max 2.5-3,
  alpha_max 12-15 deg, roll 30-60 deg/s).
- **Targets.** `max_mach` at `max_mach_altitude_ft` below 1: calibrate fits
  Korn's kappa_A (0.87 conventional sections, 0.95 supercritical), more
  wave drag if even 0.87 is too fast. A certified altitude is
  `operational_ceiling_ft`, reached or not.

## 6. Finish

- Run `ctest --test-dir build/ucrt64-release -R hangar`.
- Show the user three things: the three-view, a viewer screenshot, and the
  failed or warned checks with what they mean.
- Link `aircraft/<name>/out/report.html`.
- Check `git status`. A generated file shown as modified has changed: the
  build writes its `.xml` files as git keeps them (LF), a rebuild that
  changes nothing keeps the aircraft's date, and the mesher gives the same
  `.glb` every time. One that should not have changed is a bug to chase,
  not a file to check out. (calibrate dates `calibration.toml` every run.)
- The aircraft is `jsbsim:<name>` everywhere on the platform, with no copying:
  in the viewer, the C++ and Python SDKs, and scenario files.
