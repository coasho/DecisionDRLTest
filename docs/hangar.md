# hangar: JSBSim aircraft from a design file

JSBSim flies an aircraft from tables of aerodynamic coefficients. For an
aircraft that only exists on paper, nobody has those tables. With hangar you
describe the aircraft in one TOML file: its surfaces, bodies, engine, gear
and masses. hangar then:

- computes the tables, mass properties and propeller;
- writes a JSBSim aircraft and a 3D model of it;
- flies it in the platform's own JSBSim, checking it against performance
  targets and handling-quality criteria.

The finished aircraft is `jsbsim:<name>` everywhere on the platform: in the
viewer, the C++ and Python SDKs, and scenario files. In the viewer its
control surfaces move with the simulation, its gear folds away, its
afterburner lights and its propeller turns (see [the 3D model](#the-3d-model)).

It builds light aircraft and fighters. For a fighter it adds afterburning
turbofans, vortex lift, supersonic drag and a fly-by-wire flight control
system, so the aircraft flies like the real one: the stick asks for load
factor and roll rate, and the angle of attack stays within its limit.
Fourteen of the best-known fourth- and fifth-generation fighters are
included, built from public data (see [the fighter library](#the-fighter-library)).

![Skua, a hypothetical UAV designed with hangar, in the viewer](images/hangar-skua.jpg)

![The F-16C, built by hangar from public data](images/hangar-f16c.jpg)

![Its gear down: the legs out of their bays, the doors open](images/hangar-f16c-gear.jpg)

## Use

```bat
fsim hangar list                      the designs in aircraft\
fsim hangar new mine --like skua      start a design from another
fsim hangar mine geometry             one stage: look at aircraft\mine\out\threeview.png
fsim hangar mine --quick              every stage, coarse: a first look in half a minute
fsim hangar mine                      every stage, full tables (a few minutes)
fsim hangar mine calibrate            fit corrections to published performance
fsim demo --aircraft mine             watch it fly
fsim hangar f16c                      a fighter: NASA's wind-tunnel data as reference
```

`fsim python examples\python\control_surfaces.py` flies the C172 and the
Skua through a slalom. Watch it with
`fsim viewer --camera chase --chase-distance 20`; `tab` switches aircraft.
`fsim python examples\python\fighters.py` flies the fourteen fighters
together (`--chase-distance 60`). `fsim demo --aircraft typhoon` starts
eight of one type at a speed that suits it.

`aircraft\mine\out\report.html` collects everything on one page.

## The loop

Each stage reads the design and what the stages before it wrote. It writes
`out/<stage>.json`, which holds its numbers and its checks, plus any images.
Every check is marked pass, warn, fail or info.

The loop: read the images and the checks, change the design, and run that
stage again. A stage costs seconds, except the full tables, which take a few
minutes. The tables are cached and rebuilt only when the geometry, or the
code that computes them, changes.

| stage | produces | checked against |
|---|---|---|
| geometry | three-view and 3D views; areas, MAC, aspect ratio, tail volumes | usual ranges for the category |
| aero | coefficient tables over α ±180° and β ±90°, and factors on them over Mach; section polars; derivative plots; the reference aircraft's coefficients beside the design's | signs and sizes of the stability derivatives, CL max, smoothness; the reference's lift, drag and pitching moment |
| mass | component weights, CG, inertia, static margin | target empty mass; Roskam's radii of gyration; margin 5–40 % MAC (fly-by-wire: from −15 %, full and with the tanks dry) |
| propulsion | propeller thrust and power tables, engine | peak efficiency, static thrust / weight |
| build | `<name>.xml`, `Engines/`; the fly-by-wire's gains | the fly-by-wire's short period, MIL-F-8785C; full nose-down control reaching 2° past the angle-of-attack limit |
| model | `<name>.glb`: the airframe as one closed solid, its moving parts on their hinges | no crack, pinch or loose piece; gear stowed inside; length, span and height against `[dimensions]` |
| verify | JSBSim's forces and moments at 150 random states, compared with the tables | largest error below 0.002 in any coefficient |
| fly | trim across the speed range; stall; climb and ceiling; top speed; dynamic modes; 40 runs from random states; six crashes into the ground. A fighter instead: top speed at sea level and at the published height, excess power, the ceiling at the best climb speed, sustained turn, the angle-of-attack limiter, a 3 g step from trim, a full-stick roll | `[targets]`, MIL-F-8785C level 1, no diverged run; crashes that stop without blowing up |
| calibrate | `calibration.toml`: extra drag, and the propeller pitch unless the design gives the real one; for a jet, the throttle ratio and the wave drag | `[targets]` |
| report | `out/report.html` | |

The fly stage flies the same tests on a reference aircraft (`reference =
"jsbsim:c172x"` in `[targets]`) and prints its numbers beside the design's.

## The design file

Positions are in metres, in JSBSim's structural frame: x aft, y right, z up.
Wings and tails are given for the right half and mirrored. A fin or a body
is single unless `mirror = true`. Here is an excerpt of
`aircraft/skua/skua.toml`, with a `[targets]` section added:

```toml
[[surface]]
name = "wing"
kind = "wing"                    # wing, htail, canard, fin, vtail
airfoil = "naca4412"             # NACA 4/5-digit, or a .dat file beside the design
sections = [                     # leading edge, chord, twist about the leading edge (deg)
  { le = [0.550, 0.000, 0.120], chord = 0.360, twist = 2.0 },
  { le = [0.580, 2.000, 0.225], chord = 0.240, twist = 0.0 },
]
controls = [                     # span as a fraction of the half span
  { name = "aileron", channel = "aileron", span = [0.45, 0.95], chord_fraction = 0.25, limits = [-20, 20] },
]

[[body]]                         # superelliptic stations: width, top, bottom, exponent n
name = "pod"
stations = [ { x = 0.20, w = 0.26, top = 0.13, bottom = -0.13, n = 2.3 }, ... ]

[[engine]]
type = "electric"                # or "piston"
power_kw = 3.0
[engine.propeller]
diameter = 0.56
pitch = 0.30

[mass]
empty = 21.0                     # kg; components by name, or Raymer's statistics
[targets]                        # optional: what fly checks against
stall_speed_kcas = 27            # also max_speed_ktas, climb_rate_fpm, service_ceiling_ft;
                                 # operational_ceiling_ft for a published clearance (reached or not)
```

A fighter adds strakes, all-moving tails, surfaces that several channels
move, a turbofan and fly-by-wire. From `aircraft/f16c/f16c.toml`:

```toml
[[surface]]
name = "strake"
kind = "strake"                  # a sharp leading-edge extension: vortex lift
airfoil = "plate"                # also NACA 6A ("naca64a204"), "biconvex5"

controls = [                     # chord_fraction 1: the whole panel turns (pivot at 30 % chord);
                                 # mix: other channels it follows - here it rolls the aircraft too
  { name = "stabilator", channel = "elevator", mix = { aileron = 0.25 }, span = [0.27, 1.0], chord_fraction = 1.0, pivot = 0.30, limits = [-25, 25] },
]

[[engine]]
type = "turbofan"                # afterburner above 80 % throttle
thrust_dry_kn = 79.2
thrust_wet_kn = 129.4
bypass_ratio = 0.36

[flight_control]
type = "fbw"                     # load factor, roll rate and sideslip commands
n_max = 9.0
alpha_max_deg = 25.0

[targets]
max_mach = 2.05                  # at max_mach_altitude_ft: calibrates the wave drag
max_mach_altitude_ft = 40000
```

The 3D model has its own keys (all optional):

```toml
[[intake]]                       # a body whose first station is the lip, open onto a dark duct
name = "intake"
lip = 0.035                      # m
rake = 12                        # deg: the lip's top ahead of its bottom (negative: behind)
sweep = 0                        # deg: its outboard edge behind its inboard one
duct = 1.6                       # m seen into; duct_rise and duct_taper bend it towards the engine
stations = [ { x = 4.40, w = 0.96, top = -0.56, bottom = -1.16, chine = -0.82, n_top = 4.0, n_bottom = 2.4 }, ... ]
                                 # any body: chine is the widest line's height, n_top and n_bottom
                                 # the exponents above and below it (a flat belly, a chined nose);
                                 # lean = 40 (deg) tips a section's top outboard, its walls sloping
                                 # (a stealth fighter's caret intakes)
[[body]]
name = "canopy"                  # a pod named canopy... is glass
frames = [5.45]                  # a frame round it at each x

[[surface]]
leading = [                      # leading-edge flaps or slats, on the angle-of-attack schedule
  { name = "lef", span = [0.28, 0.97], chord_fraction = 0.17, limits = [-2, 25], schedule = [1.38, 9.05, 1.45] },
]

[aircraft]
stands_in_for = ["f16"]          # a stock JSBSim aircraft drawn with this model (fsim hangar register)

[[gear]]
retract = "forward"              # forward, aft, inward, outward; hangar fits the angle, the
wheels = 2                       # trunnion's cant and the wheel's twist that stow the leg inside
wheel_turn = "flat"              # or retract_deg, retract_axis: what is known of the real leg;
                                 # hangar fits only the rest
fairing = true                   # fixed gear: a spat over the wheel

[[strut]]                        # a high wing's lift strut, drawn only (its drag is in the
from = [0.99, 0.50, 0.15]        # extra drag area): its ends, mirrored unless mirror = false
to = [1.21, 2.50, 1.54]
chord = 0.16                     # m; thickness 0.35 of it unless given

[dimensions]                     # published: the model is checked against them
length = 15.06
span = 9.45
height = 5.09
```

Included designs:

- `aircraft/c172`: a Cessna 172P built from published dimensions, used to
  validate the methods.
- `aircraft/skua`: a hypothetical twin-boom pusher UAV.
- `aircraft/f16c`: an F-16C Block 52, checked against NASA's wind-tunnel
  data.
- Thirteen more fighters: [the fighter library](#the-fighter-library).

## Methods

Every method is a published one. The tests in `tools/hangar/tests` check the
section and lattice methods against theory and published data, and run every
stage end to end through the platform (`ctest -R hangar`).

- **Sections.** Thin-airfoil theory gives the zero-lift angle and the
  pitching moment. The lift slope is corrected for thickness and Reynolds
  number. CL max follows DATCOM's leading-edge-sharpness trend. Past the
  stall come Viterna and Corrigan's model and then flat-plate values,
  covering the whole ±180° range. Flaps use Glauert's flap effectiveness and
  Raymer's flap drag.
- **Lifting surfaces.** A vortex lattice (Katz & Plotkin, ch. 12) with
  Prandtl–Glauert compressibility and Vatistas vortex cores. The lattice is
  decambered onto the viscous section polars (after Mukherjee &
  Gopalarathnam, J. Aircraft 43(3), 2006). Each strip then reads its full
  polar at its own induced angle. Past 30–60° of flow angle this becomes
  plain strip theory.
- **Bodies.** Slender-body theory as far as the flow stays attached (DATCOM
  4.2.1.1), then Allen and Perkins' crossflow drag (NACA TR 1048), plus skin
  friction. The wing–body dihedral effect comes from DATCOM. A fuselage and
  the intakes, nacelles and booms against it are one body to the air, their
  sections' union at each station; an intake's mouth swallows its air rather
  than pushing it aside, and turns it into the duct: the inlet's normal force
  at the lip. The area rule counts overlapping bodies once.
- **Derivatives.** Rate derivatives are central differences of the full
  nonlinear model, so they change through the stall. The α̇ terms come from
  the lag of the downwash at the tail (Nelson, *Flight Stability and
  Automatic Control*, ch. 3).
- **Mass.** Component weights come from Raymer's general-aviation equations
  (ch. 15) or are given directly. Each component is spread over its own skin
  to give the inertia. Systems mass is placed to meet the empty CG.
- **Ground contacts.** Besides the wheels, the airframe gets contact
  points wherever a crash can meet the ground first: the extreme points of
  its convex hull in every direction, plus points along keels and surface
  edges. Each point's spring is sized from the mass it moves (small at a
  wingtip, where the aircraft rolls easily), so JSBSim's 120 Hz step
  integrates it stably. Points close together share that budget. The fly
  stage checks the result by crashing the aircraft six ways. The wheels
  stay JSBSim's; flightsim's build of JSBSim fixes their force when one
  lands on its side (THIRD_PARTY_NOTICES.md, "Changes to JSBSim").
- **Propeller.** Blade-element momentum theory with Prandtl's tip and hub
  losses.
- **Engines.** Piston engines use JSBSim's piston engine; hangar's control
  system adds mixture that follows the altitude. Electric motors use JSBSim's
  brushless DC motor.
- **Linear model.** Etkin & Reid's small-perturbation equations, built from
  the tables at the trim condition. The handling-quality checks use it. The
  3-2-1-1 flight tests identify the same modes from JSBSim's own response as
  a cross-check.

For fighters:

- **Vortex lift.** A thin swept section does not stall like a light
  aircraft's. Past its attached-flow limit the leading edge keeps the suction
  it can hold (Carlson's attainable thrust, NASA TP-1500). What it loses turns
  into vortex lift where the edge is sharp (Polhamus' suction analogy, NASA
  TN D-3767), and past 45° of sweep where it is blunt too (the blunt-edged
  65° delta of the VFE-2 experiment). The suction a wing loses to its vortex
  is Polhamus' own: its lift times the angle less its planform's downwash,
  CL/πA. A sharp 60° delta
  gets within 7 % of Polhamus' lift up to 20°. The vortex bursts at an angle
  that rises with the sweep (Earnshaw & Lawford, ARC R&M 3424) and takes 18°
  more to reach the apex; a burst vortex keeps 40 % of its lift, so a 60°
  delta's lift peaks at 1.27 near 30° (Wentz & Kohlman measured about 1.3 at
  35°). A wing behind a strake shares the strake's vortex; a close-coupled
  canard delays the wing's burst. The
  circulation's force uses the local velocity, so the potential lift goes as
  Polhamus' sin α cos² α.
- **Past the stall.** A vortex-lifting aircraft flies two flows there, each
  solved on its own: the vortex flow, a lifting surface in the lattice's
  induced flow, and the separated flow, flat plates in strip theory. The
  wing moves from the one to the other between 45° and 70° of its angle of
  attack, and its tails and canards, in the flow it makes, with it. Each
  flow decides before it solves where a section leaves the vortex regime,
  so each has one solution. Decided while solving, as a lone section decides
  it, a section washed far down and still in the vortex regime and a plate
  hardly washed down at all were both solutions, and the tables jumped
  between them (the Rafale's drag by 0.5 between 55° and 60° in a 30°
  sideslip).
- **Controls.** Each control turns as a whole and stops at its own limits,
  so a canard can travel further than the elevons on its channel. An
  all-moving surface turns its whole section: the lattice gets the deflection
  that gives it sin(α + δ), not the linear sin α + δ cos α.
- **Wake.** The lattice's wake leaves the trailing edge along the free
  stream. It is solved at wake angles 2° apart and interpolated, so a tail
  in the wing's plane sees the wake rise above it.
- **Sideslip.** Simple sweep theory: the windward half of a swept wing
  lifts more, the dihedral effect that grows with lift.
- **Mach number.** Prandtl–Glauert on the lattice to Mach 0.9. From 1.2,
  linear theory per surface: Ackeret, and Stewart's slope for subsonic
  leading edges. Transonic values are faired between. The results are
  factors on the tables: lift, the neutral point's move, control power.
  Drag adds skin friction falling with Mach, and wave drag from the area
  distribution (Sears–Haack times Raymer's E_WD, from Korn's
  drag-divergence Mach). Once the leading edge is supersonic, the edge's
  suction is lost.
- **Turbofans.** JSBSim's turbine. Thrust goes as the density to the 0.7
  (afterburner) or 1 (dry) up to 11 km, and as the density above it, where
  the temperature holds. It rises with the ram, 1 + 0.2 M², and is cut back
  once the compressor's inlet temperature passes the throttle ratio TR
  (Mattingly, Heiser & Pratt, *Aircraft Engine Design*, sec. 2.3). Weight
  and size come from Raymer. Calibration fits TR to the published top speed,
  and then the wave drag if TR alone cannot.
- **Fighter mass.** Raymer's fighter/attack weight equations. Radii of
  gyration are given per design (NASA's for the F-16).
- **Fly-by-wire.** Gains are placed from the linear model at each dynamic
  pressure and Mach number (`hangar/fcs.py`):
  - Pitch: angle-of-attack and pitch-rate feedback give the short period
    CAP 1 and damping 0.8. A load-factor command follows a model response,
    limited by the angle of attack left, counting its rise over the next
    0.35 s. Past the limit a push back gives all the nose-down travel 4°
    beyond it, so an unstable canard delta pitching up fast is caught. The
    command rises at most 12 g/s: a full pull at once would pitch an agile
    airframe (the MiG-29A) faster than its nose-down control can stop.
  - Pitching moment: the elevator cancels the moment's departures from a
    straight line through the angle-of-attack envelope (a table over α and
    Mach), and the gains are designed on that line. A band of local
    instability, such as the F-35A's tail passing through the wing's wake
    at 2–5°, then no longer throws a 3 g step up to 70 % past its target.
  - Roll: a roll-rate command with bank hold.
  - Yaw: a yaw damper, and sideslip from the pedals.

## The 3D model

The model stage writes `<name>.glb` from the same design:

- **Airframe.** One closed solid: every body and surface as a signed
  distance field, joined with fillets where a wing, fin or intake meets the
  fuselage, and meshed by hangar's native mesher (`tools/hangar/native`).
  Intakes open onto dark ducts, nozzles onto their turbine faces; canopies are
  glass in their frames.
- **Control surfaces.** Each one, and each leading-edge flap, is cut from its
  surface with a 12 mm gap and turns on its own hinge.
- **Landing gear.** Each leg swings about its trunnion and twists about its
  strut into a bay cut into the airframe. The bay opens where the leg passes
  through the skin; its two doors hinge on the edges either side of the leg's
  swing, open first and close behind it. hangar fits the swing to the
  airframe - through as small an opening as it can - from what the design
  gives (the direction, and any of the angle, the trunnion's axis and the
  wheel's twist). On every leg the oleo slides up the strut as the unit
  compresses, a steerable wheel turns with the steering, and the wheels roll.
- **Propulsion.** An afterburner flame behind each augmented jet, as the
  engine lights it; nozzle petals open as the engine opens them. A propeller
  turns at its engine's rpm.
- **Paint.** `paint.toml` beside the design gives its colours: a scheme
  (single, two-tone, camouflage or a cheat line), the radome, an anti-glare
  panel, the canopy's tint. hangar draws it as a texture, with panel joints
  where the airframe has them - frames round the fuselage, spars and ribs on
  the wings and fins - and a little wear.

The checks: no open, pinched or misturned edge in any mesh; the airframe in
one piece; every moving part a closed solid; the stowed gear inside the skin;
no gear door ever touching a leg, open or closed; length, span and height
within 3 % of `[dimensions]`.

A design that stands in for a stock JSBSim aircraft (`stands_in_for`) is
drawn for it too: `fsim hangar register` parks each stock aircraft, finds
where its main wheels touch, and writes `aircraft/models.txt` - which model
the viewer uses for each, moved so its wheels stand where the stock
aircraft's do.

The viewer's names for the moving nodes are in
[docs/sdk/viewer.md](sdk/viewer.md#moving-control-surfaces).

## Validation: the Cessna 172P

The C172P was built from public dimensions, not from JSBSim's c172x tables;
its tail stands where c172x's tail arm and a three-view put it, and its lift
struts and wheel spats are drawn. Its propeller has the real McCauley's
57 in (1.448 m) pitch, and calibration fits one number, to the top speed:

- **Extra drag area, 0.235 m².** This stands for the struts, cooling and
  gaps the estimate does not see.

The stall speed, the climb, the ceiling and the dynamics are predictions.
Top speeds are flown: full throttle, the height held, until the speed
settles.

| sea level, 2400 lb | hangar c172 | POH | JSBSim c172x (stock) |
|---|---|---|---|
| stall, clean | 49.5 KCAS | 51 | 41.6 |
| maximum level speed | 122.7 KTAS | 123 | 131 |
| best rate of climb | 715 ft/min | 700 | 870 |
| service ceiling | 12,970 ft | 13,000 | 25,100 |

The dynamic modes at 1500 m and 1.9 times the stall speed, first as the
linear model predicts them and then as identified from JSBSim's response:

| mode | predicted | JSBSim response |
|---|---|---|
| short period ζ | 0.64 | 0.57 |
| phugoid period | 25.3 s | 25.7 s |
| dutch roll ω, ζ | 1.66 rad/s, 0.16 | 1.72 rad/s, 0.19 |
| roll time constant | 0.21 s | 0.22 s |

All modes are MIL-F-8785C level 1, and the spiral mode is stable. JSBSim
reproduces the tables to within 4×10⁻⁴ in every coefficient. None of the 40
runs from random attitudes and rates diverged.

## Validation: the F-16C

The F-16C was built from public dimensions. JSBSim's own `f16` carries NASA
TP-1538's wind-tunnel data (Nguyen et al., 1979), a reference measured from
−20° to 90° angle of attack. It is in black, hangar's F-16C in blue:

![hangar's F-16C against NASA TP-1538](images/hangar-f16c-nasa.jpg)

Lift and drag follow NASA's to 40° angle of attack. The mean error to 15°
is 7.2 % in lift and 7.5 % in drag, and from 15° to 40° it is 8.6 % and
10.9 %. The dihedral effect, the weathercock stability to 25°, and the
damping in pitch and yaw at low α also agree.

The pitching moment agrees only at low α. Both moments are taken about
NASA's moment reference, 35 % of the MAC, which is also the design's
reference point. NASA's Cm stays within 0.015 of zero from 0° to 40°.
hangar's rises with α: it is 0.037 above NASA's at 15° and 0.23 above at
40°. The neutral point is at 30.7 % of the MAC, and NASA's at 33.5 % (both
from the slope between −2° and 6°).

This is a regression. It came when the F-16C was reshaped to its
three-views. Today's methods, run on the earlier shape, give Cm within 0.02
of NASA's to 15° and 0.07 to 40°, a mean lift error of 5.4 %, and the
neutral point at 34.4 %. The three-views gave the F-16C ogee strakes that
start 1.2 m further forward, a wider forebody, and a stabilator 0.6 m
further forward. Each of these adds nose-up moment. Put back one at a time,
the earlier strakes move the neutral point 1.1 % of the MAC aft, the
earlier fuselage and intake 0.85 %, and the earlier stabilator 0.6 %. The
new shape is the more faithful one, so the error is in the model, not in
the comparison. The strakes' vortex lift was calibrated on NASA's data
while the F-16C still had its earlier, shorter strakes.

hangar's F-16C also differs in these:

- At high angle of attack it keeps more directional stability than NASA's.
- Its all-moving tail is about 1.4 times as powerful below 20°.
- Its side force in sideslip is about half of NASA's.

JSBSim's `f16` applies Stevens & Lewis's aileron and rudder tables per
radian, although they are given per 20° and 30°. The comparison scales them
back (`reference_control_scale`).

Flown through its fly-by-wire:

| F-16C, clean | hangar | published |
|---|---|---|
| top speed, 40,000 ft | Mach 2.04 (calibrated: TR 1.22) | Mach 2.05 |
| sustained turn, Mach 0.9, 15,000 ft | 12.5 deg/s | about 13.5 deg/s |
| full aft stick, 350 kt | 7.1 g, α held at 25.0° | α limit 25° |
| full-stick roll, 350 kt | 264 deg/s | 308 deg/s (limit) |
| top speed, sea level | 874 kt | 795 kt |
| best rate of climb | 62,600 ft/min | 50,000 ft/min |
| service ceiling | 61,800 ft | 50,000+ ft |

The top speed at 40,000 ft is the calibration's one target. The rest are
predictions. The published sea-level speed is the airframe's limit, not
where thrust and drag meet.

## The fighter library

Fourteen fighters, each built the F-16C's way from public dimensions,
weights and engine data, and flown through its own fly-by-wire. Each is
`jsbsim:<name>` on the platform: in the viewer with moving control
surfaces, in the SDKs and in scenario files.

![The fourteen fighters in the viewer, each flying its fly-by-wire](images/hangar-fighters.jpg)

Each airframe is measured off a public three-view drawing and checked
against it silhouette by silhouette: side and top views overlap it 89-99 %,
front views 68-89 % (the drawings' pylons and stores count against them).
The landing gear stands where the drawings put the wheels, and folds the way
the real gear does.

Flown, beside the published figures (the J-20A's and the Su-57's are
estimates):

| aircraft | `jsbsim:` | top speed, Mach | climb, ft/min | ceiling, ft | α held (limit) |
|---|---|---|---|---|---|
| F-16C Block 52 | `f16c` | 2.04 (2.05) | 62,600 (50,000) | 61,800 (50,000+) | 25.0° (25°) |
| F-15C | `f15c` | 2.43 (2.5) | 63,200 (50,000) | 64,700 (65,000) | 30.2° (30°) |
| F/A-18C | `fa18c` | 1.81 (1.8) | 50,800 (45,000) | 60,100 (50,000+) | 35.0° (35°) |
| F-22A | `f22a` | 2.25 (2.25) | 62,100 | 61,600 (65,000) | 40.5° (40°) |
| F-35A | `f35a` | 1.59 (1.6) | 44,900 | 57,000 (50,000+) | 19.8° (20°) |
| Su-27S | `su27s` | 2.34 (2.35) | 64,300 (59,000) | 65,100 (60,700) | 26.3° (26°) |
| Su-57 | `su57` | 1.99 (2.0) | 56,300 | 57,500 (65,600) | 26.4° (26°) |
| MiG-29A | `mig29a` | 2.24 (2.25) | 64,000 (65,000) | 63,000 (59,000) | 26.6° (26°) |
| Typhoon | `typhoon` | 2.01 (2.0) | 72,500 (62,000) | 61,500 (55,000+) | 32.0° (30°) |
| Rafale C | `rafale` | 1.82 (1.8) | 60,100 (60,000) | 61,500 (50,000+) | 29.7° (29°) |
| JAS 39C Gripen | `gripen` | 1.99 (2.0) | 49,400 | 59,700 (50,000+) | 28.0° (28°) |
| Mirage 2000C | `mirage2000` | 2.19 (2.2) | 54,900 (56,000) | 57,900 (56,000) | 29.0° (29°) |
| J-10A | `j10a` | 2.20 (2.2) | 50,500 | 57,600 (59,000) | 30.2° (30°) |
| J-20A | `j20a` | 2.00 (2.0) | 50,900 | 57,800 (66,000) | 30.2° (30°) |

- The top speed at altitude is each design's one calibration target,
  flown where it is published: 40,000 ft for the five American designs,
  36,000 ft for the rest. Every other number is a prediction.
- The ceiling is where the best climb, at any Mach number the aircraft can
  hold level flight at, falls to 100 ft/min. A published 50,000 ft (and
  the Typhoon's 55,000) is a clearance, not where the climb runs out: the
  model must reach it (shown with a +).
- Handling at 350 kt, from trim: a 3 g step overshoots 3-25 % and reaches
  90 % in 0.6-1.0 s; full aft stick holds each limit within 2°.
- Engines fitted to Mach 2.3-2.5 keep too much thrust at sea level (see
  [Limits](#limits)).

## Limits

- **Speed range.** Subsonic tables, with Mach factors to about Mach 2.6.
  The factors come from linear theory, not from a transonic or supersonic
  solution of the flow.
- **Propeller wash.** The propeller's slipstream over the wing and tail is
  not modelled.
- **Stall and spin.** Past the stall the numbers are estimates. They come
  from empirical section data and, at high angles of attack, from strip
  theory. This is enough for an agent to meet the stall and recover, not for
  spin research.
- **Reynolds number.** The tables use one Reynolds number per strip, at
  one cruise speed (`[analysis] speed`), whatever the altitude. Laminar separation bubbles below
  Re ≈ 2×10⁵ are not modelled.
- **Engines.** Piston, electric, and afterburning turbofans. Thrust
  lapse comes from one published model, not from each engine's own data. An
  engine fitted to Mach 2.3-2.5 at altitude keeps too much thrust at sea
  level: the F-15C and Su-27S reach 990-1,070 kt there, where the real ones
  are held near 800 kt.
- **High angle of attack.** Forebody vortices, and the fin's shielding by
  the wing, are not modelled. Past about 30° a fighter keeps more
  directional stability than the real one. Thrust vectoring is not modelled
  either: the Su-57's limiter holds the 30° its aerodynamic controls can.
  The F-35A's holds 20°: past about 23° its stabilators, stalled, can no
  longer bring the nose down in this model (the real one flies to 50°).
- **Tails on booms.** The lattice carries a horizontal tail across the gap
  between two booms (Su-27, MiG-29) as if it were one surface, so those
  aircraft come out stable where the real ones are close to neutral.
- **Balance.** Real fighters' CGs are rarely published. The deltas' are
  placed 3-13 % of the MAC behind the neutral point the model finds, where
  their main wheels put them: a fifth or less of the weight on the nose
  wheel. Full nose-down control still reaches a few degrees past each
  angle-of-attack limit there (the build stage checks it).
- **Post-stall tables.** From 45° to 70° of angle of attack a fighter's
  tables blend two estimates, the vortex flow and the separated flow (see
  Methods): smoothly, and alike in sideslip either way. The vortex flow
  carries more normal force than the flat plates, so for most of the
  fighters it falls by a sixth to a third from a peak near 46° to a low near
  60-70°, and drag dips by up to 0.3 after a hump near 50° before it rises
  towards 90°. A fin in a sideslip of about 25° can still stall two ways;
  the tables average the two sideslips there.
- **Leading-edge devices.** The flight controls move them on the F-16's
  published schedule (NASA TP-1538), standing in for each type's own; the
  simulation reports where they are (`leadingEdgeFlapRad`) and the 3D model
  follows. The aerodynamic tables do not model them; they stand for the wing
  as its schedule flies it.
- **Gear kinematics.** Each leg folds the way the real one does where that
  is well documented (the F/A-18's main gear aft, the Typhoon's and the
  Mirage 2000's mains inward, every Rafale leg forward), and otherwise by
  hangar's default: a nose gear aft, a main gear forward. The swing itself
  is fitted to the airframe, not taken from drawings.
- **Layouts.** Swing wings and thrust-vectoring nozzles do not move. There
  is no flying wing in the library yet.

## For Claude

`.claude/skills/aircraft-design/SKILL.md` describes the working loop: which
images to read after each stage, which numbers to question, and how to fix
the usual problems.
