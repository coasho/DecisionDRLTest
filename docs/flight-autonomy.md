# ADR-29: Flight Autonomy — complete native coverage of A-GRA's Vehicle Interface

| | |
| --- | --- |
| Status | Accepted 2026-09-27. After the audit of section 1.2 the owner approved the goal and gave the directions of section 2, then approved this record with two clarifications (D13, D14). Implemented in the order of section 8: FA-1 done 2026-09-27 (section 14) |
| Extends | ADR-28 ([vehicle-interface.md](vehicle-interface.md)): its modes, path follower, grants, reports and gates stand, and its deferred work (ADR-28 section 10) becomes this record's scope. ADR-26 ([control-architecture.md](control-architecture.md)): three of its rules change. Its "no plan engine" narrows to a store of route plans and flight tasks with the VI's activation states, still with no workflow engine (section 8, FA-2 and FA-7). Commands may be ranked, queued and given time windows, with the sources' precedence underneath (FA-2). Availability gains flight phases, and discovery gains the support states of section 4. ADR-27 ([rotorcraft.md](rotorcraft.md)) for the rotorcraft |
| Scope | Every Flight Autonomy command, parameter, semantic and function that the VI volume and the 6.0a schema define, on each of the 35 aircraft to which it applies, in C++, the C ABI and Python. Also the supporting environment and subsystem models these need, kept minimal and deterministic |
| Reference | A-GRA ASK 6.0a, from the public repository open-arsenal/a-gra: the *VI L1 Interface Volume* v6.0a (sections 1.1-1.4 and its extension tables A-1-40 to A-1-170) and *A-GRA_MessageDefinitions_v6_0_a.xsd*. Read for semantics; neither is in this repository |
| Related | [sdk/control.md](sdk/control.md), [sdk/c_abi.md](sdk/c_abi.md), [sdk/python.md](sdk/python.md), [hangar.md](hangar.md), [design document](FlightSim_System_Architecture_and_Design.md) §9.3 |

## 1. Context

### 1.1 The goal

The owner, 2026-09-27: complete native SDK functional coverage of the Vehicle Interface for high-performance training and evaluation, with no XML bridge and no formal compliance claim. Every applicable command is implemented by Flight Autonomy (FA) and supported on all 35 aircraft. An exception is allowed only where a capability is physically inapplicable to an aircraft; a missing implementation is never an exception. Discovery must tell supported, unsupported and temporarily unavailable capabilities apart. It must never advertise a command that does not work, and it must give an explicit reason for every unsupported request.

### 1.2 What the audit found (main at caf03c9)

The audit read the VI volume in full, including section 1.4's allocation of capabilities to FA. It extracted every flight type from the schema, traced each item into the code, and ran a read-only probe on the built SDK over all 35 aircraft (Appendix D). Its inventory, revised here by the owner's directions, has 272 items (Appendix A):

| Group | Works | Partial | Defect | Missing | Out of scope | Items |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Core VI command interface | 52 | 56 | 7 | 105 | 0 | 220 |
| Platform behaviours | 3 | 1 | 4 | 0 | 0 | 8 |
| Flight Autonomy functions | 0 | 3 | 0 | 10 | 0 | 13 |
| Supporting environment | 0 | 2 | 0 | 8 | 0 | 10 |
| Supporting subsystems | 0 | 3 | 0 | 7 | 0 | 10 |
| Transport and message format | 0 | 0 | 0 | 0 | 11 | 11 |
| **All** | **55** | **65** | **11** | **130** | **11** | **272** |

- **Discovery advertised commands that did not work.**
  - `aerobatics` was offered to 14 aircraft that are not aerobatic types. It reported `goal_reached` while they stalled; the RQ-4B reached the ground.
  - Legacy `waypoints` never completed on 14 aircraft.
  - `evade` flew the four rotorcraft into the ground.
  - `pursuit` closed to within 0-33 m of its target on 30 aircraft.
- **On the ground, every airborne mode reported AVAILABLE.** An HSA commanded on the runway took off by itself on 10 aircraft.
- **Every unsupported request got the same answer.** Hover on a wing, gear on a fixed-gear aircraft and an id that does not exist were all refused `unknown_capability`, with a status of DISABLED. A-GRA reserves DISABLED for a capability switched off.
- **Five of the ten flight capability types are absent:** MUST_FLY, ALTITUDE_STACKED_MARSHALL, LAUNCH, RECOVERY and ROUTE_INTERCEPT.
- **The existing modes cover part of the schema.** Absent are:
  - multi-path routes, branches, required times of arrival, path terminators;
  - general NURBS curves and most hold options;
  - magnetic and barometric references;
  - speed optimisation.
- **The performance profile is thin.** Maximum CAS is unknown on all 31 wings and Mach on all 35. There are no tables against altitude or weight.
- **Complete and verified:** the lifecycle, grants, restrictions, rejection detail for routes and curves, progress and the commanded state. C++, the C ABI and Python are at parity for every item that exists.

### 1.3 Terms

| Term | Meaning |
| --- | --- |
| Item | One row of Appendix A: a command, a parameter or option, a semantic, a report, a function or a supporting model. Its id (CMD-02, HSA-03 ...) belongs to this record, for tracing; the SDK never uses it |
| Public feature identifier | The stable name the SDK gives something a consumer can use or check: a capability id (`fsim.guidance.hsa`) or a feature under it (`fsim.guidance.hsa/direction/magnetic_north`). Section 4.2 |
| Applicable | An item applies to an aircraft unless a rule of section 5 excludes it with that aircraft's evidence |
| Physical exception | An applicable-rule exclusion: the aircraft physically cannot do it (it cannot hover, has no wheels, no carrier hook, no stores). Never "not built yet" |
| Support state | Per item and aircraft: **supported**; **partial** (the item's `missing` column says what is not); **not implemented** (applicable, not built yet: the stage that builds it); **not supported** (a physical exception: the rule and the evidence) |
| Availability | Per capability, now: available; temporarily unavailable (a reason: on the ground, diverged, restricted, a placard); unavailable (return not known); expended; faulted; disabled (switched off) |
| Flight phase | On the ground or airborne; takeoff and landing join when LAUNCH and RECOVERY exist |
| Fleet acceptance test | The test that flies every supported item's case on every aircraft it applies to (section 9.3) |

## 2. Decisions (the owner, 2026-09-27)

| # | Decision | Consequence |
| --- | --- | --- |
| D1 | **The goal.** Complete native FA coverage of ASK 6.0a for all 35 aircraft; no XML transport, no compliance claim | ADR-28 D1 stands; transport items stay out (group XPT) |
| D2 | **Core VI commands apart from supporting functionality.** | Section 3: every item is core VI, a platform behaviour, an FA function, a supporting environment or subsystem model, or transport |
| D3 | **Keep and fix the platform's own behaviours.** | Section 6; each is advertised only where it works |
| D4 | **Stop advertising non-functional capabilities immediately.** | FA-1 comes first; the fleet acceptance test gates discovery from then on |
| D5 | **Tell physically unsupported, temporarily unavailable and not yet implemented apart.** | Section 4: support states, availability, and a refusal for each |
| D6 | **Per-aircraft evidence for every physical exception.** | Section 5.2: declarations with sources, checked against the model; no evidence, no exception |
| D7 | **Digests preserved** except where an intentional change of behaviour is measured and documented | Section 9.5 |
| D8 | **Minimal deterministic carrier and stores models**, appropriate for simulation, not comprehensive external systems | Section 7 |
| D9 | **State-only equipment settings claim no physical effect they do not have** | Section 7; FA-14 proves it |
| D10 | **The volume-only curve is an extension, built after the schema-defined functionality** | FA-17, last |
| D11 | **Order:** FA-1 first; then every applicable command and parameter gap, across C++, the C ABI and Python; then the FA functions and subsystems; then the extension | Section 8 |
| D12 | **This record, with the inventory, the stages, the applicability rules and the acceptance criteria, is approved before any functional code** | Approved 2026-09-27 |
| D13 | **Flight-phase gating never blocks FA's internal control paths** that takeoff, landing or another valid flight-phase operation needs | Section 4.5: the gate stands where the mission autonomy commands (a policy's NEW), not in the runtime or FA's own activities and sources |
| D14 | **Audit ids stay apart from the SDK's stable public identifiers. Dogfight tactical decision-making and manoeuvre sequencing belong to the mission autonomy and are outside this record; FA stays responsible for executing its VI flight capabilities** | Section 4.2 (identifiers), section 11 (non-goals) |

The audit page's earlier proposals (its D1-D9) are superseded by these.

## 3. The groups (D2)

| Group | What it holds | Items |
| --- | --- | ---: |
| **Core VI command interface** (VI) | What a mission autonomy commands, sets, is told and discovers: the flight commands, their parameters and semantics, validation, activity reports, discovery and availability, authority, route plans, flight tasks, vehicle settings and the state data the VI publishes | 220 |
| **Platform behaviours** (PLT) | The platform's own guidance behaviours, older than ADR-28 and outside A-GRA, which discovery also advertises: kept and fixed (D3) | 8 |
| **Flight Autonomy functions** (FA) | What FA does on its own behind the VI (section 1.4 of the VI volume): envelope and energy management, terrain, geofence and collision avoidance, fuel and contingency management, the failsafe | 13 |
| **Supporting environment** (ENV) | Models of the world the commands and FA functions need: terrain elevation, the magnetic field, the altimeter, reference frames, airfields, operational geometry, a ship, geofences, traffic, weather | 10 |
| **Supporting subsystems** (SUB) | Models of the aircraft's own systems: applicability declarations, performance and fuel, drag devices, tailhook and launch bar, catapult and arresting gear, stores, equipment state, faults, navigation solution | 10 |
| **Transport and message format** (XPT) | Out of scope (D1): no XML, messaging or compliance claim | 11 |

- A mission autonomy sees the **core VI**: every item there is something it commands, sets, reads or is told. Its items are what "every applicable command" means.
- **FA functions** act on their own behind the VI, and show through availability, reasons, reports and refusals. The VI volume allocates them to FA in section 1.4.
- **Supporting models** are the least of the world, or of the aircraft's systems, that makes a command or a function mean something. Each is built in the first stage that needs it, and only as far as that stage needs.
- The **platform's behaviours** are outside A-GRA. They are kept by D3, and discovery answers for them as for everything else.

## 4. Discovery that tells the truth (D4, D5; stage FA-1)

### 4.1 Two questions, answered apart

- **Support: can this aircraft, in this build, do it at all?** It does not change while a vehicle flies. The answers are:
  - supported;
  - partial: the item's missing part is named;
  - not implemented: applicable, with the stage that builds it;
  - not supported: a physical exception, with its rule and the aircraft's evidence.
- **Availability: can it be commanded now?** It changes with the flight. The answers are:
  - available;
  - temporarily unavailable, with a reason (`on_ground`, `airborne`, `diverged`, `restricted`, `collision_avoidance`, a placard);
  - unavailable, when return is not known;
  - expended;
  - faulted;
  - disabled, meaning switched off by the platform. It is never the answer for "not supported".

### 4.2 The support table

Each vehicle answers for every public feature, keyed by its **public feature identifier** (D14):
- its support state;
- the rule that excludes it (for "not supported"), and the evidence from the aircraft's declaration;
- the stage that builds it (for "not implemented" and "partial");
- the capability that carries it.

**Public feature identifiers are the SDK's, not this record's.**
- A capability keeps its id (`fsim.guidance.hsa`, `fsim.support.gear`).
- The capability types not built yet get theirs now, reserved and reported as not implemented until their stage: `fsim.guidance.must_fly`, `fsim.guidance.marshall`, `fsim.guidance.launch`, `fsim.guidance.recovery`, `fsim.guidance.intercept`, `fsim.guidance.taxi`.
- A feature finer than a capability (an option, a reference, a parameter's kind) is named under it after a slash, as the profile's paths are: `fsim.guidance.hsa/direction/magnetic_north`, `fsim.guidance.hsa/speed/long_range_cruise`, `fsim.guidance.pattern/entry/teardrop`, `fsim.guidance.route/required_time_of_arrival`.
- Features of the command interface itself sit under `fsim.command/`, `fsim.activity/` and `fsim.control/` (`fsim.command/time_window`, `fsim.activity/disable`).

Once published, an identifier is stable: it never changes meaning, and new features take new identifiers. FA-1 defines the list and its mapping to this record's items (Appendix A gains the column), and each stage extends both. An item with nothing a consumer uses or checks by name (a semantic its tests pin, a report field) has no identifier.

The table is how a consumer learns, before commanding anything, that for example HSA's magnetic reference is not implemented yet (FA-4) or that hover is not supported on an F-16C (R1, its evidence).

### 4.3 Refusals

| Case | Answer | Detail |
| --- | --- | --- |
| An id no platform defines | `unknown_capability` (as today) | — |
| A capability or option that is a physical exception on this aircraft | **`not_supported`** (new) | the rule; the field index for an option |
| An applicable capability or option not yet built | **`not_implemented`** (new) | the stage; the field index for an option |
| A capability temporarily unavailable | the availability's own reason (as ADR-28 7.2 does for restrictions): **`on_ground`**, **`airborne`** (new), `diverged`, `restricted`, `collision_avoidance` | — |
| A value outside the range allowed now (a placard) | as today (`unavailable` for the gear and flap placards) | the status reports the range now (4.6) |

- Existing reason codes keep their numbers and meanings. The four new ones are appended.
- A platform capability that an aircraft lacks is now refused `not_supported` instead of `unknown_capability`. This is an intended change to an answer (section 10).

### 4.4 What discovery never offers

- Every capability advertised on an aircraft has a case in the fleet acceptance test (9.3) that passes on that aircraft. A failing case fails the build, so a capability that stops working cannot stay advertised.
- A platform behaviour is advertised only where its rule admits the aircraft, and only once its fix (section 6) has passed.

Until FA-1 lands these hold by withdrawal; after it, by the test.

### 4.5 Flight phases

What a mission autonomy (a policy's NEW) is offered in each phase:

| Phase (FA-1) | Guidance modes and platform behaviours | Flight levels (actuator … position), support effectors | Ground modes (taxi, launch: FA-9) |
| --- | --- | --- | --- |
| On the ground | temporarily unavailable, `on_ground` | available | available |
| Airborne | available | available | temporarily unavailable, `airborne` |

- **The gate stands where the mission autonomy commands (D13).** It refuses a policy's NEW, and the status a policy reads reports it. It is not in the runtime, the loops, the path follower or the modes' behaviours.
- **It never blocks FA's own control paths.**
  - LAUNCH and RECOVERY (FA-9, FA-10) compose whatever loops, modes and followers they need through every phase: the takeoff roll, rotation and climb-out; the approach, flare, touchdown and rollout.
  - FA's own sources, `Autopilot` and `Override`, are not gated. FA answers for their use.
  - An activity already running when the phase changes goes on, as ADR-28 7.2 has it for restrictions: a touch-and-go, a rollout, a hover taxi. What an unsafe change of phase requires is FA's safety functions' business (FA-15).
- **The flight levels stay available in every phase**, so a policy may still fly a takeoff itself, as today.
- **FA-9 and FA-10 add the takeoff and landing phases.** LAUNCH takes a vehicle from the ground to airborne, and RECOVERY brings it back.

### 4.6 Status and admission agree

A capability's status carries the current range of any parameter that is narrower now than its advertised range:
- gear: `down` at least 1 on the ground;
- flaps: `position` at most their placard's allowance above the flap speed.

A NEW outside the current range is refused with the same reason the status gives. The status never says AVAILABLE for a command the admission would refuse.

### 4.7 Surfaces (as FA-1 built them)

| C++ | C ABI 1.7 | Python |
| --- | --- | --- |
| `enum class Support { Supported, Partial, NotImplemented, NotSupported }`; `SupportInfo {feature, support, rules, stage, capability, missing, evidence}` (`rules` a bit per `Rule`, `ruleBit`); `Vehicle::support(feature)` (a pointer, null for an unknown feature), `Vehicle::supportTable()`; `supportFeatureCount()`, `supportFeature(i)`; `supportName`, `ruleName`, `ruleDescription` | `fsim_support_info` (`struct_size`), `fsim_support_info_init`, `fsim_vehicle_support(world, id, "fsim.guidance.hsa/direction/magnetic_north", &info)`, `fsim_support_feature_count`, `fsim_support_feature(index)`, `fsim_support_name`, `fsim_rule_name`, `fsim_rule_description` | `Vehicle.support("fsim.guidance.hsa/direction/magnetic_north")` → `SupportInfo`, `Vehicle.support_table()`, `fsim.SUPPORT_FEATURES`, `fsim.Support` |
| `Availability` appends `Unavailable`, `Expended`; `Reason` appends `NotSupported`, `NotImplemented`, `OnGround`, `Airborne`; `availabilityName`, `reasonDescription` | the same values appended to `fsim_availability` and the reason codes; `fsim_availability_name`, `fsim_reason_description` | `Availability.UNAVAILABLE`, `.EXPENDED`; reason names |
| `CapabilityStatus` gains every reason that holds (`reasons`), a `description`, the id it is about (`associated`), when it is expected back (`nextAvailableS`) and the current ranges (`ranges`, `rangeCount`: 4.6); `setAvailability` takes the associated id and the time | `fsim_capability_status` (`struct_size`) through `fsim_vehicle_capability_status_info`; `fsim_vehicle_capability_limits`; `fsim_vehicle_set_availability_ex` | `Vehicle.availability(capability)` → `AvailabilityInfo`; `Vehicle.capability_limits(capability)`; `set_availability(..., associated=, next_available_s=)` |
| `CapabilityDescriptor` gains `accepted` (A-GRA's AcceptedInterface: capability command, activity command, task command, AUTO_MDF) and `superseded` (a platform behaviour's successor) | separate calls, `fsim_vehicle_capability_accepted` and `fsim_vehicle_capability_superseded`: `fsim_capability_info` has no `struct_size` to grow by | `Capability.accepted`, `.superseded` |

Everything is additive, reads allocate nothing, and structs carry their size, as ADR-26 C3 requires. The support table is built once per aircraft type (and per vehicle with a profile of its own), with its evidence; a read is a lookup.

## 5. Applicability (D6)

### 5.1 The rules

| Rule | Applies | Declaration | Evidence | Physical exceptions |
| --- | --- | --- | --- | --- |
| **R1** Hover | Holding a point in the air (hover loiter, the marshall's hover option, deck operations' vertical flight) applies to aircraft that fly on rotors. | `vertical_flight = true` | The family (fsim/identity/family 3 or 4) and the type: no fixed wing here is a powered-lift or VTOL type | 31: a10c, b52h, c130j, c172, c17a, e3g, e7a, ea18g, ec130h, f15c, f16c, f22a, f35a, fa18c, gripen, h6k, j10a, j20a, kc135r, kc46a, mig29a, mirage2000, rafale, rc135w, rq4b, skua, su25, su27s, su57, typhoon, u2s |
| **R2** Ground taxi | Taxi paths (PathType TAXI) apply to aircraft that roll on wheels. | `ground_contact = wheels` | The design's landing gear (wheels, skids or legs) and the type | 3: cf2, iris, uh1h |
| **R3** Catapult and arrested landing | Catapult launch, arrested recovery and their commands apply to carrier-capable fixed-wing types. | `carrier = catapult_arrested` | The type's service: a carrier type has a launch bar, a carrier-rated tailhook and gear for carrier sink rates; a land-based type's naval derivative is a different type | 33: a10c, b52h, c130j, c172, c17a, cf2, e3g, e7a, ec130h, f15c, f16c, f22a, f35a, gripen, h6k, iris, j10a, j20a, kc135r, kc46a, mig29a, mirage2000, rafale, rc135w, rq4b, skua, su25, su27s, su57, typhoon, u2s, uh1h, uh60 |
| **R4** Deck operations | Takeoff from and landing on a moving deck, with the carrier departure and recovery commands that apply to it, applies to aircraft that take off and land vertically. | `carrier = deck` | The family (rotorcraft) | 31: the fixed-wing aircraft (they cannot take off or land vertically); the F/A-18C and EA-18G fly carrier operations under R3 |
| **R5** Arrester hook | SetArresterHook and the hook in CleanUp and DirtyUp apply where R3 does. | `carrier = catapult_arrested` | As R3 | 33: a10c, b52h, c130j, c172, c17a, cf2, e3g, e7a, ec130h, f15c, f16c, f22a, f35a, gripen, h6k, iris, j10a, j20a, kc135r, kc46a, mig29a, mirage2000, rafale, rc135w, rq4b, skua, su25, su27s, su57, typhoon, u2s, uh1h, uh60 |
| **R6** Retractable gear | Gear retraction and extension (a vehicle action, CleanUp, DirtyUp, StayClean) applies to aircraft with retractable gear. | `retractable_gear = true` | fsim/effectors/retractable_gear in the aircraft file, and the design's gear | 6: c172, cf2, iris, skua, uh1h, uh60 |
| **R7** Trailing-edge flaps | Flaps in configuration commands apply to aircraft with a flap function. | `flaps = true` | fsim/effectors/flaps in the aircraft file, and the type's trailing-edge surfaces | 5: cf2, iris, mirage2000, uh1h, uh60; pending evidence: gripen, rafale, typhoon |
| **R8** Drag devices | Spoiler and airbrake actions, and deceleration by them, apply to aircraft that have spoilers, airbrakes or surfaces the flight control system deploys as a speedbrake. | `drag_devices != none` | The type's data | 8: c130j, c172, cf2, ec130h, iris, skua, uh1h, uh60; pending evidence: h6k |
| **R9** Releasable stores | The release envelope applies to aircraft that carry and release weapon stores: on stations, from a bay, as palletized munitions from the hold, or from dispensers. | `releasable_stores != none` | The type's data | 12: c172, cf2, e3g, e7a, ec130h, iris, kc135r, kc46a, rc135w, rq4b, skua, u2s |
| **R10** Aerobatic type (platform behaviour) | fsim.guidance.aerobatics is offered to fixed-wing types cleared for aerobatic manoeuvres. Not an A-GRA item. | `aerobatic = true (and n_max >= 6 g)` | The envelope's n_max in the aircraft file, and the type's category | 18: b52h, c130j, c172, c17a, cf2, e3g, e7a, ec130h, h6k, iris, kc135r, kc46a, rc135w, rq4b, skua, u2s, uh1h, uh60 |
| **R11** Wheel brakes (platform effector; FA-1) | fsim.support.wheel_brakes applies to aircraft that roll on wheels. | `ground_contact = wheels` | fsim/effectors/wheel_brakes in the aircraft file | 3: cf2, iris, uh1h |
| **R12** Pitch trim (platform effector; FA-1) | fsim.support.pitch_trim applies to aircraft whose controls move their surfaces directly: a fly-by-wire law and a rotorcraft's loops trim themselves. | The family (fsim/identity/family 1) | The aircraft file's family, from the design's flight control system | 20: c17a, cf2, ea18g, f15c, f16c, f22a, f35a, fa18c, gripen, iris, j10a, j20a, mig29a, mirage2000, rafale, su27s, su57, typhoon, uh1h, uh60 |
| **R13** Engine throttles (platform effector; FA-1) | fsim.flight.engines, a throttle per engine, applies to aircraft with more than one engine the pilot moves apart (a multirotor's rotors among them): a helicopter's governed engines follow its collective. | fsim/propulsion/engines above 1, not a helicopter | The aircraft file's engines and family | 11: c172, f16c, f35a, gripen, j10a, mirage2000, rq4b, skua, u2s, uh1h, uh60 |

Every item that no rule governs applies to all 35 aircraft. R11 to R13 were added in FA-1 for the platform's own effectors, which the support table answers for too; they rest on the same declarations (R11) or on the aircraft file's family and engines, which the design states from its sources.

### 5.2 Declarations and evidence

- **Each aircraft's design declares its physical characteristics, each with a source.** The declaration is a hangar TOML `[applicability]` section with `vertical_flight`, `ground_contact`, `carrier`, `retractable_gear`, `flaps`, `drag_devices`, `releasable_stores` and `aerobatic`. The source is a public document: a type certificate data sheet, a flight manual, a service fact sheet.
  - hangar writes the declarations into the aircraft file as a new profile section, `applicability`.
  - The vehicle's support table reads them.
- **A test checks each declaration against the model**, and a mismatch fails. The checks:
  - `vertical_flight` against the family (`fsim/identity/family`);
  - `ground_contact` against the design's landing gear kinds (hangar's build: wheels, skids or legs) and the wheel brakes (`tests/test_applicability.cpp`);
  - `retractable_gear` and `flaps` against `fsim/effectors/*`;
  - `aerobatic` against the envelope's `n_max` of at least 6 g;
  - `carrier`: a catapult and an arrested landing on a wing, a deck on a rotorcraft.
- **No evidence, no exception.** A characteristic without a source is not a declaration, and the items it governs stay applicable.
- **The model and the type must agree.** Where the model lacks a feature the type has (a flap function, an airbrake), the model is extended. The feature is never declared absent to match the model.

### 5.3 Evidence still to record

Appendix C gives, per aircraft, the model evidence already in the files and the type fact each exception rests on. FA-1 records a public source for every one before discovery reports the exception. Three cases are still open and stay applicable until type data decides them:
- the flap function of the JAS 39C, Rafale C and Typhoon, which the models omit (R7);
- the H-6K's drag devices (R8).

### 5.4 What applies to whom

Appendix B is the matrix: 35 aircraft against the capabilities the rules govern and the ones that apply everywhere.

## 6. Platform behaviours: kept and fixed (D3; stage FA-1)

| Id | Behaviour | Found (probe) | Fix | Status at caf03c9 |
| --- | --- | --- | --- | --- |
| PLT-01 | fsim.guidance.hold: heading, true airspeed and altitude | Probe: heading within 0.5 deg on 35 of 35; superseded by fsim.guidance.hsa. The fleet test (FA-1e): its fixed altitude gain (0.25/s) circled the Mirage 2000 about its new altitude, 25 m either way, for good | Marked as superseded in discovery; its altitude flown at the aircraft's own position-loop gain | partial |
| PLT-02 | fsim.guidance.waypoints completes where the aircraft can fly it | Probe: never completes on 14 aircraft whose turn radius exceeds the capture radius; stays active, never fails | Refuse a point it cannot capture (invalid_waypoint with the point and max_turn_rate), and fail an activity that stops closing; superseded by fsim.guidance.route | defect |
| PLT-03 | fsim.guidance.loiter: a circle round a point | Works; its 1,500 m default is clamped to the turn radius (7 to 12 km for the heavies). The fleet test (FA-1e): a vehicle that follows its carrot at once flies 17.5 % inside the circle | A default radius from the aircraft's performance; the circle flown trimmed to the one asked; superseded by fsim.guidance.pattern | partial |
| PLT-04 | fsim.guidance.pursuit keeps its stand-off from the target | Probe: closes to 0 to 33 m of the target on 30 aircraft | Hold range_m as the least range; close no faster than the aircraft can stop closing | defect |
| PLT-05 | fsim.guidance.evade stays above the terrain | Probe: the default 300 m descent flew the four rotorcraft into the ground; the activity stayed active. The fleet test (FA-1e): a rotorcraft evading from a hover kept the airspeed it had, none | A floor above the terrain (the state's height above ground); a descent that would cross it is clamped and flagged; a rotorcraft flees at its cruise at least | defect |
| PLT-06 | fsim.guidance.formation: a slot behind a leader | Probe: "converges on its slot" - but that measured only that the gap shrank. The fleet test (FA-1e): from 5 s of cruise behind, 33 of 35 end 48 m to 1.8 km off their slots after 150 s (its crosswise gain, 0.01 rad/m, outruns a heading loop; its closing allows for no lag), and a rotorcraft cannot close sideways | A closing it can stop, as pursuit's; the slot's line joined along a route's look-ahead; a rotorcraft's slot flown over the ground. A-GRA's formation builds on it (FRM, FA-12) | defect |
| PLT-07 | fsim.guidance.aerobatics flown only by aerobatic types, within the envelope, and reported as it went | Probe: offered to all 31 wings; 14 fell below their minimum CAS, the RQ-4B reached the ground and the EA-18G lost 1,946 m, all reported goal_reached | Offered by rule R10; an entry-energy check (refused performance_limit when too slow for the load factor); completed only if flown within the envelope, else failed | defect |
| PLT-08 | fsim.guidance.hover: hold a point | Probe: drift 0.00 to 0.10 m on the four rotorcraft | Kept; A-GRA's hover loiter builds on it (LTR-15, FA-5) | works |

- The four superseded behaviours stay under their ids, with `superseded` naming their successors (hold → hsa, waypoints → route, loiter → pattern, hover → the hover loiter).
- Their flights change only where they were wrong: waypoints, pursuit, evade and aerobatics (FA-1d); formation, the loiter's radius and the hold's altitude law, which the fleet test found (FA-1e). The changes are measured and listed (D7).

**As FA-1 fixed them** (the probe of Appendix D flown again, section 14):
- **Waypoints (PLT-02):** a checked NEW refuses a point inside a turn circle at its arrival, at the aircraft's full bank, by more than its capture radius (`invalid_waypoint`, the point, `max_turn_rate`); a point circled a full turn without closing fails the activity (`behavior_failed`), and it flies on straight and level. The 14 aircraft that circled for ever are refused; the 21 others complete as before.
- **Loiter (PLT-03):** left out, its radius is 1.25 times the circle the aircraft's bank and heading loop hold at its speed (at least 1,500 m): 2.3 to 4.1 km for the fighters, unchanged for the heavies, which already flew the widest circle they could. Near its circle it trims the circle of its carrot until the one flown is the one asked (FA-1e): a vehicle that follows the carrot, 0.6 rad ahead, at once flies the chord to it, R cos 0.6 = 0.83 R (the helicopters flew 17.5 % inside), and one whose turn lags flies wider (the heavies were 10 to 13 % inside, the fighters up to 3.5 % outside; FA-1d had put the heavies' shortfall down to their roll rate). Every aircraft now flies its circle within 0.1 % of its radius; a rotorcraft given no speed flies its cruise, no faster than it follows the circle.
- **Pursuit (PLT-04):** it aims `range_m` behind the target along its track and closes no faster than it can stop closing (0.25 m/s² for a wing, half a rotorcraft's deceleration, after a 10 s lag); inside the range it turns back out. The least range was 0 to 33 m on 30 aircraft. Now, of the 30 that start outside the 300 m range, 29 hold at least 318 m (the fighters settle at 341 to 427 m); the C172, whose speed answers in tens of seconds, dips once to 218 m (73 %) and settles at the range. Those that start inside it (the four rotorcraft, the Skua) open out to it.
- **Evade (PLT-05):** a floor `floor_agl_m` (150 m) above the terrain under it, or the height it started at if lower; the floor holding its descent is reported (`kActivityClamped`). The four rotorcraft no longer reach the ground. Given no speed, a rotorcraft flees at its cruise at least (FA-1e): from a hover it kept the airspeed it had, none, and drifted 150 m in 90 s; now the helicopters are 1.26 km further off.
- **Hold (PLT-01, FA-1e):** it flies to its altitude at the aircraft's own position-loop gain and vertical speeds, as the modes do, where it used 0.25/s for every aircraft: six times what the Mirage 2000's loops follow, which circled its new altitude 25 m either way for good. Now every aircraft settles as under HSA (the slowest, the B-52H, 8.4 m short after 120 s of a 200 m descent). The stock c172x's own gain is 0.25/s, so its flights do not change.
- **Formation (PLT-06, FA-1e):** it closes on its slot no faster than it could stop closing there, as pursuit does, at a gain half its speed loop's bandwidth (0.05/s for a wing); a wing flies the leader's airspeed plus that closing and joins the slot's line along the look-ahead a route's legs are flown with, and a rotorcraft flies the leader's velocity over the ground plus the closing, straight to its slot. From 5 s of cruise behind (a rotorcraft three times its slot's distance) every wing is within 9.3 m of its slot after 150 s, where 33 of 35 were 48 m to 1.8 km off, and every rotorcraft within 0.01 m.
- **Aerobatics (PLT-07):** offered by R10 (FA-1b). A checked NEW is refused `performance_limit` too slow where the least airspeed is known (the A-10C and Su-25; the fighters' comes with FA-3's performance tables) or a split-S too low. In flight it gives up below its least airspeed, past its angle of attack by more than 3° (a departure) or within 150 m of the ground, and completes only if flown within the envelope to its limiters' tolerance (0.5 g, 3° of angle of attack, 3 to 5 m/s; bank and pitch, which a loop passes by design, are not judged). The probe's loop from cruise: 14 of the 17 complete within the envelope; the A-10C, EA-18G and Mirage 2000 give up (they ran out of airspeed or departed over the top), where before every one reported `goal_reached`. None reaches the ground.
- New in the SDK for these: a behaviour's admission (`BehaviorTraits::admit`), its completion within the envelope (`BehaviorTraits::withinEnvelope`), the flags it reports (`Behavior::constraints`), and the envelope it flies to (`ControlContext::envelope`).

## 7. Supporting models: minimal and deterministic (D8, D9)

- **Minimal.** Each model is the least that gives its commands a meaning a mission autonomy can test against. Its assumptions are written in its stage's section of the documentation.
- **Deterministic.** The same seed and calls give the same result, independent of worker count, as ADR-26 C4 requires. Any variation (deck motion, gust fields) comes from the world's seed.
- **Built in.** No external system (ship simulators, weapons effects, air traffic services) and nothing fetched at run time.
- **Honest.** A state-only setting (lights, antennas, transponder, radio, RF transmit, IFF, comm allocation) is declared with no physical effect. Its descriptor says so, and FA-14's test shows that setting it changes no flight state (D9). A setting with an effect (survivability mode, a lost-comm timeout) has that effect, and it is documented.

| Id | Model | Minimal definition | For | Stage |
| --- | --- | --- | --- | --- |
| ENV-01 | Terrain elevation service | A query in the SDK and a look-ahead along a path | VI 1.2.6.9; for STS-11, VAL-06, CTG-03 | FA-4 |
| ENV-02 | Magnetic field model | The World Magnetic Model's published coefficients and equations: declination at a place and date | For HSA-03, STS-05 | FA-4 |
| ENV-03 | Barometric altimeter | Static pressure from the atmosphere; indicated altitude for a QNH setting per vehicle | For HSA-07, STS-04, STS-10 | FA-4 |
| ENV-04 | Reference frames, fixed or attached to a vehicle | Frames by id; a point's position in one at a time | For WPT-22, CRV-05, LTR-18, the carrier | FA-4 |
| ENV-05 | Airfields and runways | Runway ends, direction, length, elevation, takeoff and landing points, limits, QNH; loaded before a mission, read-only | VI 1.2.6.3; for RPL-04, LCH, RCV | FA-7 |
| ENV-06 | Operational geometry by id: points, lines, zones, volumes | A store the must-fly locations name | For MFY-03 | FA-8 |
| ENV-07 | A ship: a moving deck | Minimal and deterministic (D8): course and speed, a deck frame, optional pitch, roll and heave as sums of sines from the world's seed | For LCH-05 to 07, RCV-05 to 10 | FA-11 |
| ENV-08 | Geofences | Hard limits loaded before a mission: an altitude floor, keep-in and keep-out areas | VI 1.4 row 4.2; for CTG-02, VAL-04 | FA-15 |
| ENV-09 | Traffic picture | The world's other vehicles and their predicted paths | VI 1.4 row 4.9; for CTG-01, VAL-05 | FA-15 |
| ENV-10 | Weather for validation | Given with a plan to validate | VI 1.2.5.5; for RPL-06 | FA-7 |
| SUB-01 | Applicability declarations with per-aircraft evidence | Each aircraft's design declares its physical characteristics (section 5.2) with a source; built into its profile; a test checks them against the model | Owner direction D6; for CAP-31 | FA-1 |
| SUB-02 | Performance tables | hangar computes speeds, climb, excess power, turn and rate limits against altitude and weight from its own aerodynamics and engines | For CAP-04 to CAP-15 | FA-3 |
| SUB-03 | Fuel flow and endurance | Fuel flow tables, tank capacity, percent, endurance and playtime | For CAP-14, VAL-03, STS-07 | FA-3 |
| SUB-04 | Drag devices: spoilers, airbrakes, speedbrake surfaces | hangar models them where the type has them (rule R8) | For STS-16, CAP-13, RCV-08 | FA-10 |
| SUB-05 | Tailhook and launch bar | On the F/A-18C and EA-18G (rule R3) | For RCV-09, LCH-05 | FA-11 |
| SUB-06 | Catapult and arresting gear | Minimal and deterministic (D8): a launch force profile to an end speed, an arresting force profile to a stop | For LCH-05, RCV-05 | FA-11 |
| SUB-07 | Stores | Minimal and deterministic (D8): stations, mass and drag, release (rule R9) | VI 1.2.7.1; for WPN-01 | FA-13 |
| SUB-08 | Equipment state | Settings kept and reported; each declared with no physical effect (D9) | For STS-17 | FA-14 |
| SUB-09 | Faults | Injectable failures with built-in test and health | VI 1.2.1.4-5; for STS-09, CTG-10 | FA-16 |
| SUB-10 | Navigation solution | Solution source, state, figure of merit and covariance | For STS-08 | FA-16 |

## 8. Stages (D11)

The stages run one at a time. Each is committed and checked before the next, as ADR-26 to 28 were. Each delivers C++, the C ABI (one minor version per stage) and Python together, with documentation and tests, under the gates of section 9. Sizes are relative (S, M, L, XL).

| Stage | Title | Size | Items | Supporting models | Digests |
| --- | --- | --- | ---: | --- | --- |
| FA-1 | Discovery that tells the truth; platform behaviours fixed | M | 20 | Applicability declarations (SUB-01) | changes named and measured |
| FA-2 | Command envelope, activity commands, reports and flight tasks | L | 36 | - | identical |
| FA-3 | Performance, energy management, speed optimisation, endurance | L | 20 | Performance tables, fuel flow (SUB-02, SUB-03) | changes named and measured |
| FA-4 | References and state data | M | 14 | Terrain service, magnetic model, altimeter, frames (ENV-01 to ENV-04) | identical |
| FA-5 | Loiter and curves as the schema defines them | L | 17 | - | identical |
| FA-6 | Routes as the schema defines them | XL | 15 | - | identical |
| FA-7 | Route plans and airfields | M | 14 | Airfields, weather for validation (ENV-05, ENV-10) | identical |
| FA-8 | Must fly, stacked marshall, route intercept | L | 13 | Operational geometry (ENV-06) | identical |
| FA-9 | Launch: takeoff and taxi | L | 5 | - | identical |
| FA-10 | Recovery and drag devices | L | 8 | Drag devices (SUB-04) | changes named and measured |
| FA-11 | Carrier operations | XL | 12 | Ship, tailhook and launch bar, catapult and arresting gear (ENV-07, SUB-05, SUB-06), minimal and deterministic | identical |
| FA-12 | Formation as A-GRA defines it | M | 5 | - | identical |
| FA-13 | The release envelope | M | 2 | Stores (SUB-07), minimal and deterministic | identical |
| FA-14 | Vehicle settings | S | 3 | Equipment state (SUB-08) | identical |
| FA-15 | FA safety functions | L | 9 | Geofences, traffic (ENV-08, ENV-09) | changes named and measured |
| FA-16 | Faults and contingencies | L | 16 | Faults, navigation solution (SUB-09, SUB-10) | identical |
| FA-17 | Extension: the VI volume's discretized curve | M | 1 | - | identical |

### FA-1: Discovery that tells the truth; platform behaviours fixed (M)

Stop advertising what does not work, at once (D4); tell physically unsupported, temporarily unavailable and not implemented apart (D5); per-aircraft evidence for every exception (D6); fix the platform's behaviours (D3). Flight phases gate the guidance modes on the ground.

**Status:** done 2026-09-27 in five steps (FA-1a applicability, FA-1b the support table, FA-1c flight phases and placards, FA-1d the behaviours, FA-1e the fleet acceptance test), each measured in section 14.

**Supporting models:** Applicability declarations (SUB-01).

**Items (20):** VAL-07; CAP-03, CAP-17, CAP-18, CAP-19, CAP-20, CAP-21, CAP-22, CAP-23, CAP-24, CAP-29, CAP-30, CAP-31; AUT-05; PLT-02, PLT-03, PLT-04, PLT-05, PLT-07; SUB-01.

**Accepted when:**

- The fleet acceptance test (section 9.3) passes on all 35 aircraft: every advertised capability flies its case within its thresholds; no ground contact, no CAS below the envelope minimum, no completion reported for a manoeuvre flown outside the envelope.
- Every unsupported request is answered not_supported with its rule; every applicable but unbuilt item not_implemented with its stage; on the ground the airborne modes report TEMPORARILY_UNAVAILABLE (on_ground) to a policy and refuse its NEW with it, while FA's own sources and running activities are unaffected (D13).
- Every exception in Appendix C has its declaration and source in the aircraft's files, and the declaration test checks each against the model.
- The support table answers for every public feature identifier on every aircraft, identically in C++, the C ABI and Python; Appendix A maps each identifier to its items (D14).
- Digests identical except the flights this stage changes on purpose: the platform behaviours it fixes and any flight in which a policy commands a guidance mode on the ground (measured before and after, listed in section 14).

### FA-2: Command envelope, activity commands, reports and flight tasks (L)

The command and activity semantics A-GRA defines around every flight command.

**Items (36):** CMD-02, CMD-03, CMD-05, CMD-06, CMD-07, CMD-08, CMD-09, CMD-10, CMD-12, CMD-13, CMD-14, CMD-15, CMD-16, CMD-18, CMD-19, CMD-20; WPT-24; CRV-14; VAL-01, VAL-02, VAL-08, VAL-10, VAL-11, VAL-12; ACT-03, ACT-04, ACT-06, ACT-07, ACT-08, ACT-10, ACT-13, ACT-15; AUT-06; STS-14; TSK-01, TSK-02.

**Accepted when:**

- The random-sequence conformance test extended with ranks, time windows, queues and the activity commands, checked against its model, on every family.
- The per-step and UPDATE paths allocate nothing; digests identical.

### FA-3: Performance, energy management, speed optimisation, endurance (L)

A-GRA's per-mode performance profile from hangar's data; energy management in every mode; long-range-cruise and max-endurance speeds; endurance validation and the fuel report.

**Supporting models:** Performance tables, fuel flow (SUB-02, SUB-03).

**Items (20):** HSA-05, HSA-10; LTR-17; VAL-03; CAP-04, CAP-05, CAP-06, CAP-07, CAP-08, CAP-09, CAP-10, CAP-11, CAP-12, CAP-13, CAP-14, CAP-15; CTG-04; STS-07; SUB-02, SUB-03.

**Accepted when:**

- Each aircraft's profile values within 5 % of hangar's flight tests (level speeds, climb, stall).
- In the fleet climb case no mode lets CAS fall below 1.1 times the minimum.
- Best-range and best-endurance speeds within 5 % of hangar's optimum; endurance within 5 % of a flown burn.
- Changed flights (energy management) measured and listed; others identical.

### FA-4: References and state data (M)

Magnetic and barometric references in every mode and in the state; the QNH setting; reference frames; the terrain query; winds; orientation acceleration; terrain validation of commanded paths.

**Supporting models:** Terrain service, magnetic model, altimeter, frames (ENV-01 to ENV-04).

**Items (14):** HSA-03, HSA-07; LTR-16; VAL-06; STS-02, STS-04, STS-05, STS-06, STS-10, STS-11; ENV-01, ENV-02, ENV-03, ENV-04.

**Accepted when:**

- Declination against the World Magnetic Model's published test values; indicated altitude against the standard atmosphere at a non-standard QNH.
- Every mode accepts every reference; a path into terrain refused violation_terrain with the point. Digests identical.

### FA-5: Loiter and curves as the schema defines them (L)

Laps, entry and exit points, legs by time, turns by bank, rate or type, hold contexts and entries, two-circle patterns, the hover loiter with a duration; general NURBS curves with their references, curvature and indices.

**Items (17):** CRV-03, CRV-04, CRV-05, CRV-06, CRV-08, CRV-11; LTR-03, LTR-05, LTR-06, LTR-07, LTR-10, LTR-11, LTR-12, LTR-13, LTR-14, LTR-15, LTR-18.

**Accepted when:**

- Pattern and curve geometry within the bounds VI-5 and VI-6 set, per class; each hold entry flown as specified. Digests identical.

### FA-6: Routes as the schema defines them (XL)

Paths with ids and types, links and conditional branches, turn points, loiter points, per-segment optimisation, climb and acceleration, required times of arrival in 4D, altitude blocks, civil path terminators, planned states, RNP monitoring, relative points.

**Items (15):** WPT-04, WPT-06, WPT-08, WPT-10, WPT-11, WPT-12, WPT-13, WPT-14, WPT-15, WPT-17, WPT-18, WPT-19, WPT-20, WPT-21, WPT-22.

**Accepted when:**

- Branches taken as their conditions say; arrival within 2 s of a feasible RTA; RF arcs within 20 m; RNP alerts; per class. Digests identical.

### FA-7: Route plans and airfields (M)

Plans by id and version, their activation states, FA-owned read-only plans, airfields and runways, queries, validation and patches, execution status.

**Supporting models:** Airfields, weather for validation (ENV-05, ENV-10).

**Items (14):** WPT-23; RPL-01, RPL-02, RPL-03, RPL-04, RPL-05, RPL-06, RPL-07, RPL-08, RPL-09, RPL-10, RPL-11; ENV-05, ENV-10.

**Accepted when:**

- The VI's route-plan sequences (1.2.4 to 1.2.6) replayed as tests in SDK terms. Digests identical.

### FA-8: Must fly, stacked marshall, route intercept (L)

Three of the missing capability types.

**Supporting models:** Operational geometry (ENV-06).

**Items (13):** MFY-01, MFY-02, MFY-03, MFY-04, MFY-05, MFY-06, MFY-07; ASM-01; RIC-01, RIC-02, RIC-03; CAP-02; ENV-06.

**Accepted when:**

- Every must-fly location kind reached as specified; a marshall of four holds its separation; each intercept method picks the right segment; per class. Digests identical.

### FA-9: Launch: takeoff and taxi (L)

LAUNCH from airfields for every family, rejected takeoff, taxi paths; the flight phases switch the modes' availability.

**Items (5):** WPT-26; LCH-01, LCH-02, LCH-04; CAP-02.

**Accepted when:**

- Every wing takes off within half a runway width and rotates within 5 kt of Vr, calm and in a 10 m/s crosswind; aborts stop on the runway; rotorcraft lift off to within 1 m of their hover point; taxi paths within 2 m. Digests identical.

### FA-10: Recovery and drag devices (L)

RECOVERY at airfields for every family, go-around, missed approach, CleanUp and DirtyUp; hangar models drag devices where rule R8 applies.

**Supporting models:** Drag devices (SUB-04).

**Items (8):** RCV-01, RCV-02, RCV-03, RCV-04, RCV-08; CAP-02; STS-16; SUB-04.

**Accepted when:**

- Every aircraft lands in its touchdown zone below its sink-rate limit, calm and in its type's demonstrated crosswind; go-arounds from unstable approaches.
- Flights of aircraft that gain drag devices measured and listed; others identical.

### FA-11: Carrier operations (XL)

Carrier departure and recovery for the F/A-18C and EA-18G (catapult, arrested landing) and the four rotorcraft (deck).

**Supporting models:** Ship, tailhook and launch bar, catapult and arresting gear (ENV-07, SUB-05, SUB-06), minimal and deterministic.

**Items (12):** LCH-05, LCH-06, LCH-07; RCV-05, RCV-06, RCV-07, RCV-09, RCV-10; CAP-02; ENV-07; SUB-05, SUB-06.

**Accepted when:**

- Launches and traps (or deck landings) succeed in calm and moderate deck motion; the same seed gives the same result. Digests identical.

### FA-12: Formation as A-GRA defines it (M)

Templates, anchors, slots, keep-in zones, joins, package operation.

**Items (5):** FRM-01, FRM-02, FRM-03, FRM-04; CAP-28.

**Accepted when:**

- Four-ship formations of each class hold their slots within 10 % of spacing through 60 degree turns; joins from a dispersed start. Digests identical.

### FA-13: The release envelope (M)

The release envelope for a flight command traced to its task.

**Supporting models:** Stores (SUB-07), minimal and deterministic.

**Items (2):** WPN-01; SUB-07.

**Accepted when:**

- Accept and reject cases at each envelope edge on every aircraft rule R9 applies to. Digests identical.

### FA-14: Vehicle settings (S)

State-only settings, each declared without physical effect (D9); the survivability mode.

**Supporting models:** Equipment state (SUB-08).

**Items (3):** STS-17, STS-18; SUB-08.

**Accepted when:**

- Each setting round-trips; a test shows no flight state changes when one is set. Digests identical.

### FA-15: FA safety functions (L)

Terrain and ground collision avoidance, geofences, collision avoidance, the validations that depend on them, the best-effort simulation.

**Supporting models:** Geofences, traffic (ENV-08, ENV-09).

**Items (9):** FRM-05; VAL-04, VAL-05, VAL-09; CTG-01, CTG-02, CTG-03; ENV-08, ENV-09.

**Accepted when:**

- The probe's hazards cannot recur with protection on: no terrain or peer contact in the fleet's evade, aerobatic and pursuit cases; geofence breaches prevented; each refusal names its ids or point.
- Flights that protection changes measured and listed; others identical.

### FA-16: Faults and contingencies (L)

Fault injection and reports, emergency procedures, fuel monitoring, the policy's liveness contract and the failsafe, divert and return to base, FA health, the navigation solution.

**Supporting models:** Faults, navigation solution (SUB-09, SUB-10).

**Items (16):** LCH-03; CAP-17; CTG-05, CTG-06, CTG-07, CTG-08, CTG-09, CTG-10, CTG-11; STS-08, STS-09, STS-12, STS-13, STS-19; SUB-09, SUB-10.

**Accepted when:**

- Each fault injected per family gives its procedure and reports; the failsafe runs when the policy stops answering. Digests identical.

### FA-17: Extension: the VI volume's discretized curve (M)

Timed control points with velocities and attitudes, fitted by B-spline, Bezier, Catmull-Rom or natural cubic (D10). Documented only in the VI volume, not in the 6.0a schema; after everything the schema defines.

**Items (1):** CRV-15.

**Accepted when:**

- Each fit method's curve flown within the curve bounds, per class. Digests identical.


## 9. Acceptance criteria

### 9.1 This record is done when

1. Every applicable item of Appendix A is supported on every aircraft to which it applies, in C++, the C ABI and Python. Transport items (XPT) are outside it.
2. Every aircraft's support table reports "supported" or "not supported" for every item. Nothing remains "not implemented" or "partial".
3. Every "not supported" answer has its rule and a recorded, tested declaration with a source (5.2).
4. The fleet acceptance test (9.3) passes on all 35 aircraft for every supported item with a flight case.
5. Each stage's own criteria (section 8) passed when it landed, and its measurements are in section 14.
6. The documentation describes every item as built: the SDK guides (control, C ABI, Python), ADR-28 where its text changes, and this record.

### 9.2 Each stage is done when

- its criteria in section 8 pass;
- its items report their new support states in every aircraft's table;
- the fleet acceptance test has its new cases and passes;
- the standing gates (9.4) hold;
- digests follow 9.5.

### 9.3 The fleet acceptance test

- **Seeded by the audit's probe (Appendix D).** For every aircraft and every supported item with a flight case, it flies a case from a standard condition: a wing at 3,000 m at its reference airspeed, a rotorcraft hovering at 150 m, an aircraft on a runway for the ground cases.
- **It checks each case against thresholds set per class** (fixed-wing direct, fixed-wing FBW, helicopter, multirotor), including:
  - completion where the item terminates;
  - tracking error for its geometry;
  - no ground contact, and no CAS below the envelope minimum;
  - no manoeuvre reported complete that was flown outside the envelope.
- **It also checks discovery**:
  - every advertised capability has a passing case;
  - every "not supported" answer has its declaration;
  - every refusal carries the reason section 4.3 gives.
- **Where it runs.** It is a ctest with the label `fleet`, and runs in CI. A Python twin checks the same answers through the Python surface on one aircraft per class.
- **It grows every stage**, and a stage lands only with its own cases passing.
- **As FA-1e built it** (`tests/test_fleet.cpp`, `python/tests/test_fleet.py`):
  - The aircraft are hangar's designs as `aircraft/` holds them, all flying at once in one world, 44 km apart, made afresh for each case; a helper of the same type flies ahead where a case follows another vehicle.
  - The cases: the flight levels (velocity, attitude, acceleration, actuator, position, the engines' throttles); the support effectors at a speed for gear and flaps (gear up and down, flaps half out and in, pitch trim, speedbrake) and envelope protection's mode; parked (the flight phase, the gear's range, the flaps, the wheel brakes holding it); guidance (hold, HSA, waypoints, route, loiter, pattern, curve, hover, an aileron roll and a loop); another vehicle (pursuit, evade, formation).
  - Each checks its geometry's error, completion where the item terminates, no ground contact, no divergence and no CAS below the least; the thresholds are about twice the fleet's worst when they were set, per class, and the file gives the worst beside each.
  - It ends by checking that every capability each aircraft advertises had its case.

### 9.4 Standing gates

These are ADR-26's gates, as ADR-28 kept them:
- ctest green;
- control digests (9.5);
- no allocation on the per-step and UPDATE paths, extended to every new mode and command path;
- the interleaved A/B throughput benchmark: no regression beyond its noise on the existing cases, with the new cases' costs recorded;
- determinism across worker counts.

### 9.5 Digests (D7)

- Every existing flight keeps its digest, with protection on and off, unless the stage changes that flight's behaviour on purpose.
- A change is intentional only if all of these hold:
  - the stage names it in advance (section 8 says which do);
  - the flights that change are measured before and after, with what changed and by how much;
  - section 14 records the measurement;
  - the owner approves the new baseline for those flights.
- A change the stage did not name fails the stage.

## 10. Compatibility

- **Flight.** As 9.5.
- **The C++ SDK: additions.** Enums grow at their ends. `Command` gains alternatives for the new capability types, so a `switch` or `std::visit` over it must handle them. The platform's own code does.
- **The C ABI: additive, one minor version per stage from 1.7.** New calls, new structs with a `struct_size`, and appended enum values. A caller built against an older header keeps working.
- **Answers that change on purpose:**
  - a platform capability an aircraft lacks is refused `not_supported`, not `unknown_capability`, and its status is not supported rather than DISABLED;
  - a policy's NEW of a guidance mode on the ground is refused `on_ground` rather than accepted (FA's own sources and running activities are not affected, D13);
  - a behaviour withdrawn from an aircraft (section 6) is refused `not_supported` there.

  Each is listed in the stage that makes it, with the tests that pin it.
- **FA-1b and FA-1c changed these answers** (tests/test_discovery.cpp pins them; the older tests that pinned the old answers say why they changed):
  - a command for a feature the vehicle does not offer: `not_supported` (a physical exception), `not_implemented` (applicable, not built: on a stock aircraft, which declares nothing, every such effector), `unknown_capability` only for an id no platform defines. So do a request, release, revocation, `setAllowed` and `setAvailability` for one, and the C ABI's calls fail naming it;
  - its status: `Unavailable` with that reason, not `Disabled`; an unknown id's too;
  - a field the aircraft has nothing for (`ParameterInfo::supported` false): `not_supported` with the field's index, not `invalid_parameter`;
  - a diverged vehicle's NEW: `diverged`, the reason its status gives, not `unavailable`;
  - a policy's NEW of the platform's guidance (`fsim.guidance.*`) on the ground, checked (`RangePolicy` Clamp or Reject): `on_ground`; unchecked NEWs (the existing entry points) and FA's own sources as before;
  - the aerobatics behaviour is no longer offered to the 18 aircraft R10 excludes, nor the support effectors R11 to R13 exclude: the capability lists of those aircraft are shorter (a list's indices never change while a vehicle lives).
- **Python:** additions. `fsim.agra` maps the new reasons and states to A-GRA's vocabulary.

## 11. Non-goals

| Not done | Why |
| --- | --- |
| UCI or A-GRA messages, the XSD, a transport, heartbeats, message acknowledgments, SHA-2 product hashes, a compliance claim | D1: native SDK only. Their semantic equivalents are items (ids, revisions, synchronous answers) |
| Comprehensive ship, weapon-effects, air-traffic or weather systems | D8: minimal deterministic models only |
| Mission-system tasks (collection, electronic attack, communications relay …), classification labels, Link 16, voice | Not Flight Autonomy: mission systems or the C2 interface |
| A workflow or plan engine | ADR-26 5 stands: plans are stored and activated as the VI defines, not scheduled against each other |
| Dogfight tactical decision-making and manoeuvre sequencing: when to engage, pursue, evade or break off, which manoeuvre follows which | D14: the mission autonomy's (the VI volume's section 1.4, row 8.1: MA decomposes tactics into FA's flight capabilities). FA executes the capabilities it is given, including curves for dynamic manoeuvres. The platform's pursuit, evade and aerobatics stay as executable manoeuvres (section 6), not as tactics |

## 12. Alternatives considered

| Alternative | Why not |
| --- | --- |
| Keep advertising everything and document the defects | Violates D4: a consumer cannot know what works |
| Omit applicable-but-unbuilt items from discovery (today's behaviour) | Violates D5: omission cannot be told from a physical exception or an unknown id |
| Derive exceptions from the models alone | A model can lack a feature the type has (no aircraft here has an airbrake yet), which would turn a gap into an exception. D6 requires type evidence, checked against the model |
| Build the FA safety functions before the command gaps | D11 orders the command and parameter gaps first. FA-1's behaviour fixes (terrain floor, stand-off, aerobatic gating) remove the hazards the probe found in the meantime |
| Model carriers and stores in full (ship hydrodynamics, weapon ballistics) | D8 |

## 13. Consequences

- A consumer can read, per aircraft and item, what it can do now, what it will be able to do (and when), and what it never can (and why). Training can then be scoped honestly, per aircraft.
- The fleet acceptance test becomes the contract for discovery. A regression in any advertised capability on any aircraft fails the build.
- The aircraft files carry their physical characteristics with sources, which hangar and future aircraft designs must supply.
- The work is large: FA-2, FA-3, FA-5 and FA-9 to FA-16 are large or very large. The order puts the hazards first (FA-1), then what a mission autonomy commands.

## 14. Measurements

Filled as the stages land: each stage's criteria results, the digests (identical or changed with before/after), the allocation gate, the A/B throughput and the fleet test's results.

**FA-1a to FA-1c (applicability, support table, flight phases, placards).**
- The 35 designs declare 272 characteristics with their sources; 8 are left undeclared (5.3 and the J-20A's, RQ-4B's and Su-57's drag devices, on which no exception rests). Every rebuilt aircraft file differs only by the declarations and its date. `test_applicability` checks them against the models on all 35.
- The support table answers for 190 public features on every vehicle; `test_discovery` pins the refusals, the flight phase (a policy's HSA refused `on_ground` on the ground, an autopilot's accepted and updated), the placards (the gear's `down` range [0.5, 1] on the ground and gear-up refused with the status's reason; the flaps' range above their speed; the gear unavailable above its operating speed) and the status's reasons, associated id and expected return.
- Digests: identical to 5b's, with protection and without (`fsim_control_bench digest`).
- The allocation gate passes (`control_alloc`).
- A/B throughput against VI-6's build (`ab_vi.py micro`, 5 interleaved rounds): the medians within -2.3 % to +3.1 %, the noise of the VI-7 run (-2.0 % to +2.7 %); the minimums within 1.5 %. The per-step path is unchanged.

**FA-1d (the platform's behaviours fixed).**
- Section 6 has what each fix did, flown on all 35 aircraft (the probe of Appendix D again): 14 routes that circled for ever refused, no pursuit closer than 73 % of its range where it was 0 to 33 m, no rotorcraft evading into the ground, aerobatics completed within the envelope or honestly failed.
- Digests: `pursuit` and `evade` change, by design (the pursuit aims at its stand-off point; the evade's target is that pursuer, so its flight follows); every other flight identical, with protection and without.
- The conformance walk's rarer answers (a revocation for collision avoidance that ends what a policy flies, an append where a curve does not end) are now met over more seeded walks, the same every run, instead of on one seed's path, which any change to what a command draws moves.

**FA-1e (the fleet acceptance test).**
- `tests/test_fleet.cpp` (label `fleet`, two ctest cases, 38 s) passes on all 35 aircraft with 15,006 checks. Discovery: every capability's row agrees with what the aircraft offers; every "not supported" row has its rules and its evidence; every capability not offered is refused, and reported unavailable, with `not_supported` or `not_implemented`; an unknown one with `unknown_capability`. Flights: every capability each aircraft advertises flies its case, and the test checks that each had one.
- The Python twin (`python/tests/test_fleet.py`, part of `python_sdk`, 4.6 s): the C172, F-16C, UH-60 and Iris give the same discovery answers and ground rules through Python, and fly velocity, HSA, loiter and formation to the same thresholds.
- What it found, and FA-1 fixed:
  - **Flaps were never done on hangar's designs.** fsim reads JSBSim's `fcs/flap-pos-norm`, which JSBSim declares but does not derive and hangar's flap channel never wrote; a flaps command to anything but "up" stayed active for good on the 27 designs with flaps. hangar now writes it (a gain on `fcs/flap-pos-deg`); each rebuilt aircraft file gains those 5 lines only.
  - Formation, the hold's altitude, the loiter's radius and a rotorcraft's evade: section 6.
  - **A rotorcraft's turns.** Its default pattern circle, and the speed it flies a given circle at, now also keep the turn rate within a third of its velocity loop's bandwidth (the rule a route's look-ahead follows), besides 80 % of its acceleration. The UH-1H (0.22 rad/s) flew its 79 m default circle up to 30 m off; its circle is now 206 m, flown within 7.3 m. The Crazyflie's 0.3 m circle, flown 0.75 m off, is now 6.9 m, flown within 0.27 m. No digest flight is a rotorcraft's.
- What it found for later stages, not fixed in FA-1:
  - **No hangar design records its gear or flap speeds.** The envelope section hangar writes carries the flight-control limits and the stall, so the placards of 4.6 have nothing to act on across the fleet, and a policy may lower flaps at any speed. The B-52H's stabilizer, at 65 % of its nose-down travel in trimmed cruise, cannot hold the pitch-up of half flaps from about 1.5 times its least airspeed up: lowered at cruise (132 m/s CAS), it climbs away until its airspeed is 50 m/s (fixed in the design since: the next entry). Recording each design's speeds, with their sources, in that section is data work for hangar, proposed with FA-3's performance tables. Meanwhile the fleet test lowers the flaps where a flap speed usually is: 1.4 times the least airspeed (about 1.7 times the stall).
  - **A climb trades airspeed** (FA-3's energy management): asked by HSA or hold for 200 m more at 3,000 m, the C172 pulls up to its least airspeed at full throttle, then creeps at 0.1 m/s. Until FA-3 brings the fleet climb case, the wings' HSA and hold cases descend 200 m (rotorcraft climb).
  - hangar's turboprops idle at 5,000 to 11,000 lbf on the ground: parked, the C-130J rolls to 11 m/s in 5 s until braked. A model item for hangar; the fleet test judges the wheel brakes by the stop.
- Digests: `formation` and `loiter` change, by design; every other flight identical, with protection and without (the hold flights too: the c172x's own altitude gain is 0.25/s). The formation's c172x, joining from 2.9 km behind, ends 44 m from where it did; the loiter's ends 16.5 m off, on its 800 m circle now.
- The c172x's checkpoints (docs/control-architecture.md, 12.4): the loiter's rows from step 600 recorded again, as its trim takes it out to its circle (up to 47 m by step 1800; its height and airspeed within 0.1 m and 0.04 m/s); the other five flights' rows are byte-identical.
- The allocation gate passes. A/B throughput against VI-6's build (5 interleaved rounds): the medians within -2.9 % to +3.1 %, except waypoints at +5.3 % (7 ns a call; its minimum +5.1 %), the cost of FA-1d's check that a point is still being closed on, measured here.
- ctest: all 213 tests pass (the two fleet cases and the Python twin among them) in 217 s, as before: hangar's stage test sets the time.

**The B-52H's pitch authority with flaps (after FA-1e; fixed in the design, not in the loops).**
- **Why its cruise needed 6.5° of nose-down stabilizer.** The balance asks little of the tail: at the reference condition the wing's moment about the CG is −0.023, the bodies' −0.011 and the thrust line's +0.003, so the tail carries +0.031. But the design rigged its all-moving tail at 0° to a fuselage that cruises at −4.2° (its wing's 8° incidence), in 3° of the wing's downwash (hangar's lattice gives 0.49° per degree of alpha; DATCOM's handbook value is 0.34). Turning it to where it makes that load took 6.8° (6.5° flown). The other causes are small. The wing's camber changes nothing: symmetric sections trim within 0.05°, because the lift they lose lowers the tail's angle by as much. The 3° washout costs 0.65°, and the CG 0.13° per % MAC.
- **Why the flaps then ran out of travel.** At constant speed, half flaps add 5° of nose-down trim and full flaps 9°. The Fowler flap pitches the design up at a fixed alpha: Cm +0.08 per radian about the CG, because the tail's added downwash (2.3° half, 3.9° full) outweighs the flap lift's arm. On top of that, the lower alpha that keeps the lift the same costs the static margin times the flap's lift. Half flaps at cruise needed 11.7° against a 10° stop.
- **The real B-52H** (public sources). Its stabilizer trims through 13°, "9 up, 4 down" (J. Baugher's B-52 history; Wikipedia). The USAF's accident report on B-52H 60-0053 (Guam, 2008) puts the stabilizer at a normal 3–4° nose down around 240 KCAS at 14,000 ft, and cites Boeing's data for a recovery from 7° nose down. So "up" is the leading edge's, and nose down; the design had the travel the wrong way round (15° nose up, 10° down). Its flaps have two settings, up and fully down (35°, Baugher; the design's go to 30°), and B. Schiff (AOPA Pilot, 2015) reports the same. A B-52 pilot on PPRuNe writes that they "want to pitch the aircraft up" and that crews lower them in a descent; Wikipedia notes the "large pitch changes induced by flap application". The design's pitch-up is the real aircraft's, in kind.
- **The change** (`aircraft/b52h/b52h.toml`, the horizontal tail only):
  - The travel is 15° nose down and 10° nose up: the stabilizer's 9° and 4°, plus an estimated 6° of elevator either way.
  - The stabilizer's zero is rigged 3° leading edge up, turned about its spindle. Its rigging on the fuselage is not published, so this is fitted: level flight at 14,000 ft and 240 to 296 KCAS now trims at 2.9° to 3.6° nose down (it took 6.2° to 6.8°), the report's 3–4° within what the unknown weight moves it.
  - Cruise at 3,000 m now trims at 3.2° nose down; the middle of the travel is 2.5°.
- **Flown** (FA-1e's probe, 60 s at 3,000 m: the velocity loop holds level flight, then the flaps are commanded). Each cell gives the most nose-down stabilizer and the balloon, or how long the stabilizer sat at its stop:

  | lowered at | half flaps | full flaps | half flaps, before | full flaps, before |
  | --- | --- | --- | --- | --- |
  | 1.2 × least CAS | 5.1°, 99 m | 8.6°, 193 m | 6.8°, 91 m | at the stop 50 s, 829 m |
  | 1.3 × | 6.0°, 90 m | 9.4°, 207 m | 8.0°, 107 m | at the stop 52 s, 852 m |
  | 1.4 × | 6.7°, 85 m | 10.2°, 226 m | 9.0°, 127 m | at the stop 48 s, 875 m |
  | 1.7 × | 8.2°, 91 m | 11.9°, 304 m | at the stop 55 s, 1,284 m | at the stop 40 s, pitched to 41° |
  | 2.0 × | 9.1°, 107 m | 12.8°, 386 m | at the stop 56 s, 1,465 m | at the stop 44 s, pitched to 90° |
  | cruise, 132 m/s CAS | 9.7°, 121 m | 13.5°, 486 m | at the stop 38 s, pitched to 33° | at the stop 40 s, pitched to 90° |

  The stabilizer now never reaches its stop (15°; it was 10°). Up to 1.7 times the least CAS the airspeed stays within 1.3 m/s of the command. At twice the least CAS and at cruise speed, full flaps cost 5.4 and 10.7 m/s before the loop wins the speed back.
- **hangar's stages all pass.** Against the old design flown by today's build, the autopilot stage's manoeuvres at 0.7, 1 and 1.5 times the reference speed match within 1 % of overshoot and 0.3 s. The exception is the 1.5 g step at 1.5 times, which now overshoots 2.8 % (was 0 %). The stall is the wing's: 6° of nose-up stabilizer at the break, 4° left.
- **The profile.** Least CAS 58.38 m/s and alpha max 15.2°. Today's build gives the old design 58.00 m/s and 15.6°; its file had kept 58.34 m/s and 17.2° from 2026-09-25.
- **Digests.** `b52h_velocity` and `b52h_attitude` change, as this entry names them. The other 18 flights are identical, with protection and without. The two flights, replayed through the SDK to the same end states:
  - `b52h_velocity`: the stabilizer spans 0 to 5.4° (was 0 to 8.6°); heights within 2.7 m (end +1.2 m); pitch within 0.43°; airspeed within 0.06 m/s; ground track within 2.4 m.
  - `b52h_attitude`: the stabilizer spans 0 to 4.1° (was 0 to 7.4°); heights within 6.8 m (end +5.7 m); pitch within 0.29°; airspeed within 0.04 m/s; ground track within 1.3 m.

  The new baseline awaits the owner's approval.
- **ctest:** all 213 tests pass.


## Appendix A: the inventory

Status at caf03c9:
- **works**: implemented and verified;
- **partial**: its `Missing` says what is not;
- **defect**: implemented but misbehaves, per the probe;
- **missing**: applicable, not built;
- **out of scope**: transport.

`Rule` is the applicability rule that governs the item (section 5); blank means all 35 aircraft. `Stage` is the stage that closes it. Evidence cites files at caf03c9. The ids are this record's, for tracing only. `Public ids` are the SDK's public feature identifiers (4.2) that answer for each item, from FA-1 (`src/control/Features.cpp`); an item a consumer never selects by name (a semantic its tests pin, a report field) has none.

### A.1 Core VI command interface (220 items)

**Command envelope and lifecycle** (CMD)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CMD-01 | NEW, UPDATE and CANCEL of a flight command | VI 1.3.2.2 Command; XSD CommandStateEnum | works | submit/update/cancel answered at once (World.h:150-296; fsim_c.h fsim_vehicle_submit*, fsim_activity_update*, _cancel*; world.py submit_*, Activity.update/cancel) | - |  |  | - |
| CMD-02 | CommandID: the caller's id for a command, echoed in its status and in the activity's Source | XSD CommandBaseType.CommandID; ActivitySourceType | missing | The platform returns its own ActivityId (Capability.h:165) | A caller correlation id carried into the result, the ActivityRecord and progress |  | FA-2 | `fsim.command/command_id` |
| CMD-03 | Several command instances in one message, each answered on its own | XSD MA_FlightCommandMDT.Command[1..n] | partial | One command per call; UPDATE has a batch (fsim_activity_update_batch) | A batch NEW (each result separate) in C++, C and Python |  | FA-2 | `fsim.command/batch` |
| CMD-04 | CapabilityID addresses the capability commanded | XSD CapabilityCommandBaseType.CapabilityID | works | By capability id or command type (Catalog.cpp:332-347) | - |  |  | - |
| CMD-05 | Ranking.Rank: priority and precedence within priority | XSD CapabilityCommandRankingType, ComparableRankingType | partial | Three fixed sources, Policy < Autopilot < Override (Capability.h:171); within a source the newest command wins | A rank within a source, compared before preemption, reported on the activity |  | FA-2 | `fsim.command/rank` |
| CMD-06 | Ranking.InterruptOtherActivities = false: do not interrupt; schedule after | XSD CapabilityCommandRankingType | missing | A NEW always preempts what owns its axes at its source | Queued (Pending) activities that start when the axes free |  | FA-2 | `fsim.command/no_interrupt` |
| CMD-07 | Ranking.CapabilityPrecedenceOverride | XSD CapabilityCommandRankingType | missing | - | A per-command precedence override |  | FA-2 | `fsim.command/precedence_override` |
| CMD-08 | TemporalConstraints: start and end windows (no earlier / no later than, repetition), criticality | XSD CapabilityCommandTemporalConstraintsType | missing | Activities start at once; Pending exists (Capability.h:213) but is never timed | Scheduled start, a deadline, repetition, and CONSTRAINT_TIME when missed |  | FA-2 | `fsim.command/time_window` |
| CMD-09 | OverrideRejection: perform a command previously rejected | XSD CapabilityCommandBaseType.OverrideRejection | missing | - | Accept over soft rejections only (endurance, air traffic advisories); never over safety limits |  | FA-2 | `fsim.command/override_rejection` |
| CMD-10 | Traceability: the Requirement (Task, command) a flight command comes from | XSD TraceabilityType; VI 1.2.7.1 | missing | - | A requirement tag on the command, echoed in the activity (also serves the release envelope, WPN-01) |  | FA-2 | `fsim.command/traceability` |
| CMD-12 | Activity command ChangeActivityState: DISABLE, ENABLE, RESET | XSD MA_FlightActivityCommandType; CapabilityCommandStateEnum | missing | - | Disable (keep, stop flying: DISABLED), enable, reset (restart from the start) |  | FA-2 | `fsim.activity/disable`, `fsim.activity/enable`, `fsim.activity/reset` |
| CMD-13 | Activity command DeleteActivity (a sticky disable) | XSD MA_FlightActivityCommandType.DeleteActivity | partial | CANCEL ends the activity (Canceled/Requested) | The DELETED state name and its sticky semantics |  | FA-2 | `fsim.activity/delete` |
| CMD-14 | Activity command ChangeActivityRank | XSD MA_FlightActivityCommandType | missing | - | Re-rank a live activity (with CMD-05) |  | FA-2 | `fsim.activity/change_rank` |
| CMD-15 | Activity command UnassignActivity: free its resources, keep it | XSD MA_FlightActivityCommandType | missing | Axes can be owned apart (CommandOptions.axes) | Release an activity's axes without ending it |  | FA-2 | `fsim.activity/unassign` |
| CMD-16 | Interactive: whether an activity takes Activity commands | XSD MA_FlightActivityType.Interactive | partial | Per capability: the kUpdate interaction bit (Capability.h:344) | Per activity, in the record |  | FA-2 | - |
| CMD-17 | CommandProcessingState RECEIVED, ACCEPTED, REJECTED, CANCELED | XSD CommandProcessingStateEnum | works | Accepted, Rejected, Canceled answered at once (Capability.h:191); RECEIVED is never needed in-process | - |  |  | - |
| CMD-18 | Status.Activity[]: resulting activity with NewActivity | XSD ResultingActivityType | partial | CommandResult.activity (Capability.h:198) | NewActivity (false when an UPDATE or a merge keeps the activity) |  | FA-2 | - |
| CMD-19 | CommandProcessingStateReason: reason, description, associated id | XSD CannotComplyType | partial | Reason (29 values) and `other` for authority_held; fsim.agra maps to CannotComplyEnum | A text description; associated id for every reason that has one |  | FA-2 | - |
| CMD-20 | ACCEPTED with less than optimum results expected | XSD CommandProcessingStateReason annotation | partial | kClamped with the first clamped field and its limit (Catalog.cpp:63-78) | Every adjusted field and its adjusted value |  | FA-2 | - |
| CMD-21 | After acceptance, changes go through the activity, not the command | XSD CommandProcessingStateEnum annotations | works | UPDATE and CANCEL address activities (World.h:266-296) | - |  |  | - |
| CMD-22 | A new command supersedes the active one (HSA, a curve without AppendCurve) | VI 1.2.2.1-1.2.2.3 | works | Preemption by axes at the same source (CapabilityHost) | - |  |  | - |

**HSA/CSA** (HSA)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| HSA-01 | Heading hold, true north | VI 1.2.2.2; XSD MA_HSA_CSA_Type.Direction.Heading | works | HsaCommand.headingRad (Control.h:140); tests VI-3; probe: heading within 3 deg on 35 of 35 after a 90 deg turn, altitude within 9 m on 34 (c172: HSA-10) | - |  |  | `fsim.guidance.hsa/direction/heading` |
| HSA-02 | Course hold (over the ground), true north | XSD MA_DirectionChoiceType.Course | works | HsaCommand.courseRad: a wing by the wind triangle, a rotorcraft by its ground velocity (GuidanceModes.h:41-51) | - |  |  | `fsim.guidance.hsa/direction/course` |
| HSA-03 | MAGNETIC_NORTH reference for heading and course | XSD MA_HeadingReferenceEnum | missing | Directions are true only | A declination model (World Magnetic Model, published coefficients) and the reference field |  | FA-4 | `fsim.guidance.hsa/direction/magnetic_north` |
| HSA-04 | Speed: TRUE_AIRSPEED, CALIBRATED_AIRSPEED, GROUNDSPEED, Mach | XSD SpeedReferenceEnum, MachType | works | SpeedReference (Control.h:127); trueAirspeedOf (GuidanceModes.h:32) | - |  |  | `fsim.guidance.hsa/speed/true_airspeed`, `fsim.guidance.hsa/speed/calibrated_airspeed`, `fsim.guidance.hsa/speed/ground_speed`, `fsim.guidance.hsa/speed/mach` |
| HSA-05 | SpeedOptimization LONG_RANGE_CRUISE, MAX_ENDURANCE | XSD SpeedOptimizationEnum; VI 1.2.2.2 | missing | - | Best-range and best-endurance speeds from a performance model (CAP-05), flown and updated with weight and altitude |  | FA-3 | `fsim.guidance.hsa/speed/long_range_cruise`, `fsim.guidance.hsa/speed/max_endurance` |
| HSA-06 | Altitude MSL, AGL, WGS_HAE | XSD AltitudeReferenceEnum | works | AltitudeReference Msl, AboveGround, Ellipsoid (Control.h:131); MSL and HAE coincide in the simulation | - |  |  | `fsim.guidance.hsa/altitude/msl`, `fsim.guidance.hsa/altitude/agl`, `fsim.guidance.hsa/altitude/hae` |
| HSA-07 | Altitude ALTITUDE_BAROMETRIC | XSD AltitudeReferenceEnum; VI 1.2.6.5 | missing | - | Indicated barometric altitude from a QNH setting (STS-10) |  | FA-4 | `fsim.guidance.hsa/altitude/barometric` |
| HSA-08 | Partial commands update the most recent commanded values | VI 1.2.2.2 | works | UPDATE keeps what is not given; a NEW keeps what the live hsa commanded (Control.h:133-139) | - |  |  | `fsim.guidance.hsa` |
| HSA-09 | Persistent: no completion state | VI 1.2.2.2 | works | Persistence::Persistent | - |  |  | `fsim.guidance.hsa` |
| HSA-10 | Tracked subject to the envelope: speed is not traded below the minimum for altitude | VI 1.2.2.2; 1.4 row 2.5 | defect | Probe: c172 at 3000 m asked +200 m kept climbing at the cost of speed, 50 to 30 m/s TAS, CAS below the envelope minimum; altitude 100 m short at 90 s | Energy management: airspeed margin first; report the climb as performance limited |  | FA-3 | `fsim.guidance.hsa` |

**Waypoint following and taxi** (WPT)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| WPT-01 | Fly a route's waypoints in order and report completion | VI 1.2.2.3 | works | fsim.guidance.route (GuidanceModes.h:78-143); probe: completed on 35 of 35 | - |  |  | `fsim.guidance.route` |
| WPT-02 | Legs on great circles or rhumb lines (RouteProjection) | XSD MA_RouteType.RouteProjection | works | RouteCommand.projection (Control.h:185) | - |  |  | `fsim.guidance.route/projection/great_circle`, `fsim.guidance.route/projection/rhumb_line` |
| WPT-03 | Fly-by (TURN_SHORT) and FLY_OVER turn points | XSD TurnPointTypeEnum | works | Waypoint.turn FlyBy, FlyOver (Control.h:150-154) | - |  |  | `fsim.guidance.route/turn/fly_by`, `fsim.guidance.route/turn/fly_over` |
| WPT-04 | TurnPoint CAPTURE_OUTBOUND_COURSE, START_TURN, END_TURN; course at the point; turn radius or bank | XSD TurnPointType | partial | Waypoint.maxBankRad sizes a fly-by turn | The three turn types, a course at the point, an explicit turn radius |  | FA-6 | `fsim.guidance.route/turn/capture_outbound_course`, `fsim.guidance.route/turn/start_turn`, `fsim.guidance.route/turn/end_turn`, `fsim.guidance.route/turn/radius`, `fsim.guidance.route/course_at_point` |
| WPT-05 | Per-segment speed (value and reference, or Mach) | XSD MA_PathSegmentType.Speed | works | Waypoint.speed and speedReference | - |  |  | `fsim.guidance.route/segment_speed` |
| WPT-06 | Per-segment speed optimisation (long-range cruise, max endurance) | XSD PathSegmentSpeedType.SpeedOptimization | missing | - | As HSA-05, per segment |  | FA-6 | `fsim.guidance.route/speed/long_range_cruise`, `fsim.guidance.route/speed/max_endurance` |
| WPT-07 | Per-segment climb rate | XSD ClimbType.ClimbRate | works | Waypoint.climbRateMs | - |  |  | `fsim.guidance.route/climb_rate` |
| WPT-08 | Per-segment climb optimisation BEST_RATE, EXTENDED_RANGE | XSD ClimbOptimizationEnum | missing | - | Best-rate and cruise-climb schedules from the performance model |  | FA-6 | `fsim.guidance.route/climb/best_rate`, `fsim.guidance.route/climb/extended_range` |
| WPT-09 | Per-segment maximum roll | XSD MA_PathSegmentType.MaximumRoll | works | Waypoint.maxBankRad | - |  |  | `fsim.guidance.route/max_roll` |
| WPT-10 | Per-segment acceleration | XSD MA_PathSegmentType.Acceleration | missing | - | The speed change's acceleration along the segment |  | FA-6 | `fsim.guidance.route/acceleration` |
| WPT-11 | Required time of arrival (time window or duration) at a point | XSD RequiredTimeOfArrival, TurnPoint timestamp; VI 1.4 row 4.1 | missing | - | 4D guidance: speed scheduling to arrive within the window; ETA against RTA in the progress |  | FA-6 | `fsim.guidance.route/required_time_of_arrival` |
| WPT-12 | Waypoint altitude with reference, and altitude ranges (blocks) | XSD Point2D_Type Altitude/AltitudeRange/AltitudeReference | partial | Waypoint.altitudeM and altitudeReference | Altitude ranges; the barometric reference |  | FA-6 | `fsim.guidance.route/altitude/msl`, `fsim.guidance.route/altitude/agl`, `fsim.guidance.route/altitude/hae`, `fsim.guidance.route/altitude/barometric`, `fsim.guidance.route/altitude_block` |
| WPT-13 | Paths: several per route (PRIMARY, ALTERNATE, LOSS_OF_COMM, RETURN_TO_BASE, ditch, ingress, egress, TAKEOFF, LANDING, approach, BOLTER_WAVEOFF ...) with ids | XSD MA_RoutePathType, MA_PathTypeEnum | missing | One sequence of up to 256 points (PathStore, Control.h:220) | A route of paths with PathID and PathType; the first path and first segment |  | FA-6 | `fsim.guidance.route/paths` |
| WPT-14 | NextPathSegment: explicit sequencing, jumps to another path | XSD NextPathSegmentType | missing | Points are flown in order; `start` picks the first | Segment links across paths |  | FA-6 | `fsim.guidance.route/next_segment` |
| WPT-15 | ConditionalPathSegment: branch on altitude, time window, capture count, operator input, endurance, contingency level | XSD ConditionalPathSegmentType, PathSegmentConditionType | missing | - | Branch evaluation at each segment end |  | FA-6 | `fsim.guidance.route/conditional_segment` |
| WPT-16 | Segment ids reported back | XSD PathSegmentID | works | Waypoint.id echoed in the progress (Capability.h:232) | - |  |  | `fsim.guidance.route` |
| WPT-17 | WayPoint (no turn) and its WaypointType (NAV_ONLY, PASSIVE, END_OF_PATH, runway, approach, takeoff, touchdown, HARD_DITCH) | XSD WayPointType, WaypointTypeEnum | missing | - | The end-point kinds and what FA does at each (the runway and approach types drive LCH/RCV) |  | FA-6 | `fsim.guidance.route/waypoint_type` |
| WPT-18 | LoiterPoint: a loiter pattern at a point in the route, with an end time | XSD MA_LoiterPointType | partial | A route may end in a loiter (EndBehavior::Loiter) | Loiters inside a route: pattern, duration, laps or end time, then on |  | FA-6 | `fsim.guidance.route/loiter_point` |
| WPT-19 | Civil path terminators (RF, CF, TF, DF, CA, VA, FA, FC, FD, FM, CD, VD, VM, VR, CI, VI, CR, AF, PI, IF, HA, HF, HM) | XSD CivilPathTerminatorType | missing | - | Leg types as ARINC 424 defines them (RF and CF carry data in the schema; the others are empty elements) |  | FA-6 | `fsim.guidance.route/path_terminators` |
| WPT-20 | InertialState[]: planned 4D states inside a segment | XSD MA_PathSegmentType.InertialState | missing | - | Accept and use as the reference trajectory within the segment |  | FA-6 | `fsim.guidance.route/inertial_states` |
| WPT-21 | Required navigation performance (containment, m) | XSD RequiredNavigationPerformanceInMeters | missing | - | Monitor cross-track against RNP; report and alert |  | FA-6 | `fsim.guidance.route/required_navigation_performance` |
| WPT-22 | Relative points (reference frames, including moving ones) | XSD Point2D_RelativeType, ReferenceFrameID | missing | - | Points relative to a frame: fixed or attached to a vehicle (a ship, a tanker) |  | FA-6 | `fsim.guidance.route/relative_points` |
| WPT-23 | Planning metadata: Detailed, Source, Locked, Modified, Remarks, Fix_Identifier, InitialConditions | XSD MA_RouteType, MA_PathSegmentType | missing | - | Kept with the stored route and reported back (no flight effect) |  | FA-7 | `fsim.guidance.route/metadata` |
| WPT-24 | Validation per segment: the invalid segment and why (turn, heading change, bank, roll, speed, altitude, orientation, weight, fuel, input) | VI 1.2.2.3; XSD RouteValidationInvalidPathType, RouteValidationErrorEnum | partial | InvalidWaypoint or PerformanceLimit with the point's index and the limit (Capability.h:126-128) | Every invalid segment with its RouteValidationError reason; the path id |  | FA-2 | `fsim.guidance.route` |
| WPT-25 | UPDATE of a route (new waypoints or options) | VI 1.2.2.3 | works | update_route: flown afresh from its start (World.h:279) | - |  |  | `fsim.guidance.route` |
| WPT-26 | Taxi routes: PathType TAXI, validated against taxi speed and turn rate, flown on the ground | VI 1.2.2.3; 1.4 row 10.1 | missing | Brakes and gear are support effectors; no ground guidance | Ground guidance (nose-wheel steering, differential braking, taxi speed), taxi validation, obstruction braking | R2 | FA-9 | `fsim.guidance.taxi` |

**Curve following** (CRV)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CRV-01 | Quintic Bezier segments: six control points, weights 1, knots [0x6, 1x6] | VI 1.2.2.1; XSD MA_NURBS_PointType | works | BezierSegment (Control.h:195); tests VI-6 | - |  |  | `fsim.guidance.curve/bezier` |
| CRV-02 | 1 to 10 segments, C0 continuity | VI 1.2.2.1; XSD CurveSegments[1..10] | works | Checked at NEW (World.h:171) | - |  |  | `fsim.guidance.curve` |
| CRV-03 | General rational NURBS: 4 to 10 weighted control points, 4 to 14 knots, other degrees | XSD MA_NURBS_PointType (ControlPoints[4..10], KnotVector[4..14], Weight) | missing | Only the quintic Bezier profile the VI text describes | Evaluation, arc length and validation of any clamped NURBS the schema admits |  | FA-5 | `fsim.guidance.curve/nurbs` |
| CRV-04 | CenterReference as a geodetic point (lat, lon, altitude with reference) | XSD WayPointPointChoiceType.Point2D | partial | CurveCommand latitude, longitude, altitude MSL (Control.h:206) | Altitude reference and altitude range on the centre |  | FA-5 | `fsim.guidance.curve/reference/geodetic` |
| CRV-05 | CenterReference relative to a reference frame (a moving ownship or platform) | XSD Point2D_RelativeType | missing | - | Frames (WPT-22) |  | FA-5 | `fsim.guidance.curve/reference/frame` |
| CRV-06 | Control point offsets: rotation (UNROTATED, 2D, 3D, HEADING), XY interpretation (CARTESIAN, GEODETIC), Z (down, altitude offset, absolute altitude) | XSD RelativeOffset2D_Type, RotationEnum, OffsetXY_Enum, Z_ChoiceType | partial | North, east, down metres (Cartesian, unrotated) | The rotations, geodetic offsets and altitude choices |  | FA-5 | `fsim.guidance.curve/offsets` |
| CRV-07 | Traversal by speed range or by duration | XSD MA_CurveTraversingType | works | speedMinMs, speedMaxMs, durationS (Control.h:209-211) | - |  |  | `fsim.guidance.curve/traversal/speed_range`, `fsim.guidance.curve/traversal/duration` |
| CRV-08 | Curvature (smoothness) and Initial/Final control point indices | XSD MA_NURBS_PointType | missing | - | Accept, validate against the curve, use the indices for continuity |  | FA-5 | `fsim.guidance.curve/curvature` |
| CRV-09 | AppendCurve: only while a curve is executing, same centre | VI 1.2.2.1 | works | CurveCommand.append in an UPDATE (Control.h:213) | - |  |  | `fsim.guidance.curve/append` |
| CRV-10 | EndOfCurveBehavior CSA | XSD MA_EndOfCurveBehaviorEnum | works | EndBehavior::Continue: on along the last course, at its altitude and speed | - |  |  | `fsim.guidance.curve/end/csa` |
| CRV-11 | EndOfCurveBehavior CIRCULAR_LOITER at the endpoint | XSD MA_EndOfCurveBehaviorEnum | partial | A wing orbits the end; a rotorcraft hovers over it | A rotorcraft circle when a circle is what is asked |  | FA-5 | `fsim.guidance.curve/end/circular_loiter` |
| CRV-12 | Rejection naming the invalid section (segment, u start, u end) | XSD MA_CurveSectionType | works | InvalidCurve with index, from, to (Capability.h:206-207); probe: 16 correct rejections | - |  |  | `fsim.guidance.curve` |
| CRV-13 | Curve status START, IN_PROGRESS, COMPLETED; segment; percent | XSD MA_CurveFollowingStatusType | works | Progress segment, segmentPercent, percent; state | - |  |  | `fsim.guidance.curve` |
| CRV-14 | FlyoutCurve: the curve being flown, reported back | XSD MA_CurveFollowingControlType.FlyoutCurve | missing | - | Report the stored curve (with appended segments) by activity |  | FA-2 | `fsim.guidance.curve` |
| CRV-15 | Discretized 4D control points with a fit method (B_SPLINE, BEZIER, CATMULL_ROM, NATURAL_CUBIC) | VI tables A-1-59, A-1-61 to A-1-64 (absent from the 6.0a XSD) | missing | - | Fit and fly a trajectory through timed points with velocities, attitudes and rates. Documented in the VI volume only; no schema element carries it |  | FA-17 | `fsim.guidance.curve/discretized` |

**Loiter** (LTR)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LTR-01 | Orbit CIRCLE: centre, radius, direction | XSD MA_CircleOrbitType, CircleType | works | PatternCommand Orbit (Control.h:248); probe: 35 of 35 | - |  |  | `fsim.guidance.pattern/orbit` |
| LTR-02 | RACETRACK | XSD OrbitEnum | works | PatternKind::Racetrack (fix, inbound course, legs, radius) | - |  |  | `fsim.guidance.pattern/racetrack` |
| LTR-03 | RACETRACK and FIGURE_EIGHT given as two circles (own centres and radii) | XSD MA_CircleOrbitType.Circle[1..2] | partial | Racetrack and figure-eight from a centre, an axis and one radius | The two-circle form |  | FA-5 | `fsim.guidance.pattern/two_circles` |
| LTR-04 | FIGURE_EIGHT | XSD OrbitEnum | works | PatternKind::FigureEight | - |  |  | `fsim.guidance.pattern/figure_eight` |
| LTR-05 | Fix-point orbit: inbound direction as heading or course, leg as distance or duration, turn as radius or bank angle | XSD MA_FixOrbitType | partial | Inbound course, leg distance, radius | Inbound heading, leg duration, turn by bank angle |  | FA-5 | `fsim.guidance.pattern/fix_point` |
| LTR-06 | Orbit duration by time or number of orbits | XSD OrbitDurationType | partial | durationS; laps are counted and reported | A lap count that completes it |  | FA-5 | `fsim.guidance.pattern/duration/time`, `fsim.guidance.pattern/duration/orbits` |
| LTR-07 | Entry point and exit point on the pattern | XSD MA_OrbitType.EntryPoint/ExitPoint | missing | - | Enter at a named point; leave at one |  | FA-5 | `fsim.guidance.pattern/entry_point`, `fsim.guidance.pattern/exit_point` |
| LTR-08 | Hold: anchor, inbound orientation, turn direction, speed | XSD MA_HoldType | works | PatternKind::Hold (fix, course, clockwise, speed) | - |  |  | `fsim.guidance.pattern/hold` |
| LTR-09 | Hold defaults: right turns; inbound = arrival course; 60 s legs at or below 14,000 ft, 90 s above | VI 1.2.2.3 | works | PatternCommand defaults (Control.h:240-247) | - |  |  | `fsim.guidance.pattern` |
| LTR-10 | Hold leg as time or length | XSD MA_HoldLegSpecificationType | partial | legM; time only as the default | An explicit leg time |  | FA-5 | `fsim.guidance.pattern/hold/leg_time` |
| LTR-11 | Hold turns as radius, rate, or type STANDARD, MIL_POWER, RELAX | XSD MA_HoldTurnSpecificationType, MA_HoldTurnTypeEnum | partial | radiusM; rate one by default | Turn rate and the three turn types |  | FA-5 | `fsim.guidance.pattern/hold/turn_rate`, `fsim.guidance.pattern/hold/turn_type` |
| LTR-12 | Hold duration by time, orbits, or entry and exit times | XSD MA_OrbitDurationType | partial | durationS | Orbits; entry and exit times |  | FA-5 | `fsim.guidance.pattern/hold/duration` |
| LTR-13 | Hold entry DIRECT, ANCHOR, INBOUND, OUTBOUND (and the parallel and teardrop entries ATC flies) | XSD MA_HoldEntryTypeEnum | partial | Direct to the fix | The other entries |  | FA-5 | `fsim.guidance.pattern/entry/direct`, `fsim.guidance.pattern/entry/anchor`, `fsim.guidance.pattern/entry/inbound`, `fsim.guidance.pattern/entry/outbound`, `fsim.guidance.pattern/entry/parallel`, `fsim.guidance.pattern/entry/teardrop` |
| LTR-14 | HoldContext ADMIN, TACTICAL, ATC: the defaults it implies | XSD MA_HoldContextEnum | missing | - | Context defaults (ATC's are implemented as the only ones) |  | FA-5 | `fsim.guidance.pattern/hold/context` |
| LTR-15 | Hover loiter at a 3D point for a duration | XSD HoverType | partial | fsim.guidance.hover holds a point (Rotor.cpp:340-349); probe drift 0.00-0.10 m | Duration and completion; the 3D point with altitude reference; a mode setpoint that takes UPDATE | R1 | FA-5 | `fsim.guidance.pattern/hover` |
| LTR-16 | Loiter altitude with reference | XSD Point2D_Type AltitudeReference | partial | MSL, AGL, HAE | Barometric |  | FA-4 | `fsim.guidance.pattern/altitude/msl`, `fsim.guidance.pattern/altitude/agl`, `fsim.guidance.pattern/altitude/hae`, `fsim.guidance.pattern/altitude/barometric` |
| LTR-17 | Loiter speed optimisation | XSD MA_HoldType.Speed.SpeedOptimization | missing | - | Max endurance hold speed (HSA-05) |  | FA-3 | `fsim.guidance.pattern/speed/long_range_cruise`, `fsim.guidance.pattern/speed/max_endurance` |
| LTR-18 | Relative (frame) points for centres and fixes | XSD PointChoiceType.RelativePoint | missing | - | Frames (WPT-22) |  | FA-5 | `fsim.guidance.pattern/relative_points` |

**Must fly** (MFY)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| MFY-01 | Must fly a geodetic point (3D) | XSD MustFlyType.Location.Point | missing | - | Fly over the point at its altitude; complete when passed |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-02 | Must fly over an entity (another vehicle, a track) | XSD MustFlyLocationType.EntityID | missing | - | Overfly a moving vehicle in the world |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-03 | Must fly an OpPoint, OpLine, OpZone or OpVolume by id | XSD MustFlyLocationType | missing | - | A store of operational geometry by id |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-04 | Must enter a zone (polygon, ellipse, rectangle, slant-range area; altitude band) | XSD ZoneExternalType | missing | - | Enter the area and report |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-05 | Must fly through a corridor (line with left and right widths, altitude band) | XSD LineTargetType | missing | - | Transit the corridor end to end within its widths |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-06 | Must enter a volume (sphere, dome, cylinder, cone ...) | XSD OpVolumeType | missing | - | Enter the volume (orbital volumes: not applicable in the atmosphere) |  | FA-8 | `fsim.guidance.must_fly` |
| MFY-07 | Ingress constraint: the bearing window at the location | XSD MustFlyType.IngressConstraint | missing | - | Arrive within the bearing window |  | FA-8 | `fsim.guidance.must_fly` |

**Formation** (FRM)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FRM-01 | Formation slot relative to an anchor platform | XSD MA_FormationType.FormationAnchor/FormationSlot | partial | fsim.guidance.formation: an offset (ahead, right, below) behind a leader (Builtin.cpp:590-624); probe converges | Slots by index from a template; a virtual anchor point |  | FA-12 | `fsim.guidance.formation/anchor` |
| FRM-02 | Formation templates AGILE, CONTAINER, TRAIL | XSD MA_FormationTemplateEnum | missing | - | Slot geometry per template and flight size |  | FA-12 | `fsim.guidance.formation/templates` |
| FRM-03 | Keep-in zone for the formation | XSD MA_FormationType.ZoneTarget | missing | - | Hold the formation inside the zone |  | FA-12 | `fsim.guidance.formation/keep_in_zone` |
| FRM-04 | Several formation entries in one command; package operation | XSD Formation[0..n]; PackageOperationEnum | missing | - | Several constraints; advertise package coordination |  | FA-12 | `fsim.guidance.formation/package` |

**Altitude stacked marshall** (ASM)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ASM-01 | Altitude stacked marshall: loiter kinematics (orbit or hover), minimum and maximum altitude, separation | XSD MA_AltitudeStackedMarshallType | missing | - | Each vehicle takes and keeps a slot in the stack with its peers | R1 (its hover option) | FA-8 | `fsim.guidance.marshall`, `fsim.guidance.marshall/hover` |

**Route intercept** (RIC)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RIC-01 | Route intercept of a stored route plan (path, earliest and latest points) | XSD MA_RoutePlanInterceptType | partial | RouteCommand.start picks the first point of the route given | Intercept a stored plan (RPL-01) between bounds |  | FA-8 | `fsim.guidance.intercept` |
| RIC-02 | Intercept methods DISCRETE, SHORTEST_DISTANCE, SOONEST (default: the plan's beginning) | XSD MA_RouteInterceptEnum | missing | - | The three methods |  | FA-8 | `fsim.guidance.intercept` |
| RIC-03 | Intercept status: plan execution state, previous, current and next segment, capture time and distance, loiter progress, segment kinematics | XSD MA_RoutePlanInterceptStatusType | partial | Route progress: current point, distance and time to go, laps, cross-track | Previous and next segments, capture estimates, along and across speeds |  | FA-8 | `fsim.guidance.intercept` |

**Launch** (LCH)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LCH-01 | Airfield takeoff from a runway by AirfieldID and RunwayID | XSD MA_AirfieldTakeoffType; VI 1.4 row 10.2 | missing | Probe: an HSA on the runway either lifts off on its own (10 aircraft, gear left down) or rolls on the runway with the wings rocking up to 42 deg (21) | Line-up, takeoff roll with runway tracking, rotation at Vr, climb-out, gear and flap retraction, exit criteria (time, waypoint, state, gear up) |  | FA-9 | `fsim.guidance.launch`, `fsim.guidance.launch/runway` |
| LCH-02 | Rejected takeoff and abort | VI 1.4 row 10.2 | missing | - | Abort below V1: idle, brakes, stop on the runway; the aborted-takeoff taxi path |  | FA-9 | `fsim.guidance.launch/rejected_takeoff` |
| LCH-04 | Rotorcraft vertical (or running) takeoff to a hover | XSD MA_DepartureType (LAUNCH) | missing | - | Lift-off, climb to a hover height, transition |  | FA-9 | `fsim.guidance.launch/vertical` |
| LCH-05 | Carrier departure: Departure (CarrierID) - catapult launch | XSD MA_CarrierDepartureDepartureType | missing | - | Ship model, catapult, launch bar and hold-back | R3 | FA-11 | `fsim.guidance.launch/carrier_catapult` |
| LCH-06 | Carrier departure: ProceedOnCourse (kill overrides, exit the departure orbit, shortcut) | XSD MA_CarrierDepartureChoiceType | missing | - | Departure procedure state machine | R3 or R4 | FA-11 | `fsim.guidance.launch/carrier_proceed` |
| LCH-07 | Carrier departure: JoinUp with a formation anchor | XSD MA_CarrierDepartureJoinUpType | missing | - | Join-up (FRM) | R3 or R4 | FA-11 | `fsim.guidance.launch/carrier_join_up` |

**Recovery** (RCV)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RCV-01 | Airfield landing on a runway by AirfieldID and RunwayID | XSD MA_AirfieldLandType; VI 1.4 row 10.2 | missing | - | Approach, landing configuration, glide slope, crab and de-crab, flare, touchdown, rollout, braking |  | FA-10 | `fsim.guidance.recovery`, `fsim.guidance.recovery/runway` |
| RCV-02 | Go-around and automatic wave-off (crosswind) | VI 1.4 row 10.2 | missing | - | Go-around on unstable approach or crosswind above the aircraft's limit |  | FA-10 | `fsim.guidance.recovery/go_around` |
| RCV-03 | Missed approach (a path chained to the approach) | VI 1.2.6.3 | missing | - | Fly the chained missed-approach path (WPT-15) |  | FA-10 | `fsim.guidance.recovery/missed_approach` |
| RCV-04 | Rotorcraft approach to a hover and vertical landing | XSD MA_RecoveryType (RECOVERY) | missing | - | Approach, hover, descend, touchdown |  | FA-10 | `fsim.guidance.recovery/vertical` |
| RCV-05 | Carrier recovery: Recovery (CarrierID, expected final bearing, expected approach time, marshal radial, range, altitude) | XSD MA_CarrierRecoveryRecoveryType | missing | - | Case I/III marshal and approach to a moving deck | R3 or R4 | FA-11 | `fsim.guidance.recovery/carrier` |
| RCV-06 | Carrier recovery: Delta (hold a time if feasible) | XSD MA_DeltaType | missing | - | Delay in the marshal | R3 or R4 | FA-11 | `fsim.guidance.recovery/carrier_delta` |
| RCV-07 | Carrier recovery: CallTheBall, ClearedDownwind, InterceptFinalBearing, DiscontinueApproach | XSD MA_CarrierRecoveryChoiceType | missing | - | Approach phase commands | R3 or R4 | FA-11 | `fsim.guidance.recovery/carrier_calls` |
| RCV-08 | Carrier recovery: CleanUp, DirtyUp (gear, flaps, hook) | XSD MA_CarrierRecoveryChoiceType | missing | Gear and flaps are support capabilities | Configuration commands over the effectors the aircraft has | R6, R7 or R5 | FA-10 | `fsim.guidance.recovery/configuration` |
| RCV-09 | Carrier recovery: SetArresterHook STOW, DEPLOY | XSD MA_SetArresterHookEnum | missing | - | A tailhook effector (hangar) and its command | R5 | FA-11 | `fsim.guidance.recovery/arrester_hook` |
| RCV-10 | Carrier recovery overrides: SetSpeed (minimum approach), StayClean | XSD MA_CarrierRecoveryOverrideType | missing | - | Approach overrides | R3 or R4 | FA-11 | `fsim.guidance.recovery/carrier_overrides` |

**Validation and rejection detail** (VAL)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| VAL-01 | ValidationResult: every applicable reason (select all that apply) | XSD MA_CannotComplyDetailsType.ValidationResult[0..9] | partial | One Reason per result; fsim.agra maps it to MA_ValidationResultEnum | A list of reasons with a detail record each |  | FA-2 | - |
| VAL-02 | PERFORMANCE_LIMIT_EXCEEDED with every constraint broken | XSD MA_PerformanceConstraintEnum[0..11] | partial | Constraint enum complete (11 values, Capability.h:143); one per result | All constraints broken |  | FA-2 | - |
| VAL-03 | VIOLATION_ENDURANCE with remaining and required endurance | XSD MA_InsufficientEnduranceType | missing | - | Endurance prediction for a command (fuel burn model, CAP-14) |  | FA-3 | - |
| VAL-04 | VIOLATION_GEOFENCE with the geofence ids | XSD GeofenceConstraintID | missing | - | Geofences (CTG-02) |  | FA-15 | - |
| VAL-05 | VIOLATION_AIR_TRAFFIC with the entity ids | XSD EntityConstraintID | missing | - | Predicted conflicts with other vehicles |  | FA-15 | - |
| VAL-06 | VIOLATION_TERRAIN with the point expected to hit | XSD TerrainConstraint (Point4D) | missing | - | Terrain clearance along the commanded path |  | FA-4 | - |
| VAL-07 | CAPABILITY_NOT_SUPPORTED with the capability | XSD ValidationResultReason.CapabilityID | defect | Probe: every unsupported request answers unknown_capability, the same as a nonexistent id; status DISABLED | A distinct reason and why (see CAP-22) |  | FA-1 | - |
| VAL-08 | Plain-text description | XSD ValidationResultReason.Description | missing | - | A human-readable detail string |  | FA-2 | - |
| VAL-09 | Validation phases: pre-check, best-effort simulation, post-check | XSD MA_ValidationResultEnum annotation | partial | Pre-checks at NEW (ranges, points, curve sections) | A best-effort simulation of the command against the envelope, terrain and fuel |  | FA-15 | - |
| VAL-10 | Best effort: a suggested flight task with adjusted parameters | VI 1.2.2.1-3; XSD MA_TaskMT.Flight | partial | RangePolicy::Clamp accepts with values clamped (kClamped) | Reject-and-suggest: return the adjusted command for the caller to resubmit |  | FA-2 | - |
| VAL-11 | A FAILED activity names the suggestion (ActivityReason.AssociatedID) | VI 1.2.2.1 | missing | - | Link a failed activity to a suggestion |  | FA-2 | - |
| VAL-12 | Validate a command without flying it | VI 1.4 row 9.2; 1.2.5.5 | missing | - | A dry-run check that answers as NEW would |  | FA-2 | `fsim.command/validate` |

**Activity reporting** (ACT)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ACT-01 | ActivityID and CapabilityID | XSD MA_FlightActivityType | works | ActivityRecord id, capability (Capability.h:246) | - |  |  | - |
| ACT-02 | States ENABLED, ACTIVE_UNCONSTRAINED, ACTIVE_PARTIALLY_CONSTRAINED, COMPLETED, FAILED | XSD ActivityStateEnum | works | Pending, Active with constraint flags, Completed, Failed; fsim.agra maps them | - |  |  | - |
| ACT-03 | ACTIVE_FULLY_CONSTRAINED: triggering conditions exist but the activity is not performed | XSD ActivityStateEnum | partial | fsim.agra maps saturation or an exceedance to it (python/fsim/agra.py) | The A-GRA meaning: pending on a constraint (queued, blocked, restricted) |  | FA-2 | - |
| ACT-04 | States DISABLED and DELETED | XSD ActivityStateEnum | missing | Canceled | With CMD-12 and CMD-13 |  | FA-2 | `fsim.activity/disable`, `fsim.activity/enable`, `fsim.activity/reset` |
| ACT-05 | ActivityReason (reason, description, associated id) | XSD CannotComplyType | works | reason and `by` (the preempting activity) | - |  |  | - |
| ACT-06 | Basis ACTUAL, SENSED, PREDICTED, PLANNED | XSD ActivityBasisEnum | partial | Always actual | Predicted and planned records (queued activities) |  | FA-2 | - |
| ACT-07 | ActivityRank | XSD ComparableRankingType | missing | - | With CMD-05 |  | FA-2 | `fsim.command/rank` |
| ACT-08 | Source: the command or task it came from | XSD ActivitySourceType | partial | Source (Policy, Autopilot, Override) | The command id or requirement (CMD-02, CMD-10) |  | FA-2 | - |
| ACT-09 | Estimated start, estimated completion, estimated percent complete | XSD MA_FlightActivityType | works | startTime; progress timeToGoS and percent (Capability.h:228-244) | - |  |  | - |
| ACT-10 | ActualEndPoint[]: the point flown to, then the following ones | XSD MA_EndPointType[0..n] | partial | The segment flown and its waypoint id | The following points (and loiter points) |  | FA-2 | - |
| ACT-11 | Actual completion time | XSD ActualCompletionTime | works | endTime | - |  |  | - |
| ACT-12 | VehicleCommandState: altitude, heading, orientation, orientation rate, power level, speed | XSD MA_VehicleCommandStateType; VI 1.2.6.8 | works | Vehicle.commanded() (World.h:178), fsim_vehicle_commanded, Vehicle.commanded; progress course, heading, altitude, speed | - |  |  | - |
| ACT-13 | VehicleCommandState acceleration (NED) and the altitude's reference | XSD MA_VehicleCommandStateType | partial | Load factor and longitudinal acceleration; altitude MSL | NED acceleration; the reference commanded |  | FA-2 | - |
| ACT-14 | Published periodically while active | VI 1.2.3.1 | works | Records and progress refreshed each world step and polled (by design, ADR-26) | - |  |  | - |
| ACT-15 | The last flight command, in the activity report | VI 1.2.3.1 | missing | - | Read an activity's setpoint back (its current fixed-size setpoint) |  | FA-2 | - |

**Discovery, performance and availability** (CAP)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CAP-01 | Capabilities advertised with id and flight capability type | VI 1.2.2.4; XSD MA_FlightCapabilityType | works | capabilities() with FlightMode (Capability.h:348-363); fsim_vehicle_capability_flight_mode; fsim.agra.flight_capabilities | - |  |  | - |
| CAP-02 | Every one of the ten types offered where it applies | XSD MA_FlightCapabilityEnum | partial | HSA_CSA, WAYPOINT_FOLLOWING, CURVE_FOLLOWING, LOITER, FORMATION (partial) | MUST_FLY, ALTITUDE_STACKED_MARSHALL, LAUNCH, RECOVERY, ROUTE_INTERCEPT |  | FA-8, FA-9, FA-10, FA-11 | - |
| CAP-03 | AcceptedInterface per capability (capability command, activity command, task command ...) | XSD CapabilityControlInterfacesEnum | partial | Interaction bits command, update, cancel, settings, status (Capability.h:344) | The A-GRA interface list per capability (and in the control status) |  | FA-1 | - |
| CAP-04 | Performance profile per control mode (HSA, waypoint, curve) | XSD MA_FlightCapabilityPerformanceProfileType | partial | One Performance per vehicle (Capability.h:383-417) | Per mode, as the modes fly (a curve's limits are its own) |  | FA-3 | - |
| CAP-05 | Min and max airspeed, best-endurance and best-range airspeed, each against altitude (and weight) | XSD MA_AirspeedLimitType | partial | minCasMs, maxCasMs, maxTasMs, cruiseTasMs (single values) | Tables against altitude and weight; probe: max CAS unknown on all 31 wings, min CAS unknown on the 16 FBW aircraft, max Mach unknown on all 35 |  | FA-3 | - |
| CAP-06 | Min and max altitude now | XSD MinAltitude, MaxAltitude | partial | ceilingM (wings with a performance section) | Minimum altitude; the ceiling for the weight now; rotorcraft ceilings |  | FA-3 | - |
| CAP-07 | Acceleration limits (body axes, against Mach or body rates; time-indexed for curves) | XSD MA_AccelerationLimitsType | partial | Load factor min and max; a rotorcraft's acceleration and deceleration | Three-axis limits against Mach; deceleration for wings |  | FA-3 | - |
| CAP-08 | Specific excess power, or acceleration and climb-rate pairs | XSD MA_ExcessPowerOrAccelerationType | partial | maxClimbMs | Ps against speed, altitude and weight |  | FA-3 | - |
| CAP-09 | Maximum orientation against airspeed, altitude and weight | XSD MA_OrientationLimitType | partial | Bank; pitch where the envelope has it (probe: none of the 31 wings) | Pitch limits; tables |  | FA-3 | - |
| CAP-10 | Maximum orientation rates (roll, pitch, yaw) against airspeed | XSD MA_OrientationRateLimitsType | partial | Roll rate (probe: unknown on the 15 direct wings) | Pitch and yaw rates; tables |  | FA-3 | - |
| CAP-11 | Maximum turn rate | XSD MaxTurnRate | missing | Derivable from bank and speed | Reported |  | FA-3 | - |
| CAP-12 | Maximum descent rate against airspeed, altitude and weight | XSD MaxDescentRate | partial | maxDescentMs (single value) | Tables |  | FA-3 | - |
| CAP-13 | Maximum deceleration | XSD MaxDeceleration | partial | Rotorcraft only | Wings (idle, drag devices) |  | FA-3 | - |
| CAP-14 | Fuel burn rate against airspeed, altitude and weight | XSD MA_FuelBurnRateType | missing | - | Fuel flow tables from the engine model |  | FA-3 | - |
| CAP-15 | Profile updated with configuration and flight condition | VI 1.2.6.7 | partial | Recomputed when the loops change; controlRevision (World.h:188) | Recomputed with flaps, gear, weight and altitude |  | FA-3 | - |
| CAP-16 | Availability AVAILABLE and TEMPORARILY_UNAVAILABLE | XSD CapabilityAvailabilityEnum | works | Diverged; platform restrictions (setAvailability) | - |  |  | - |
| CAP-17 | Availability UNAVAILABLE, EXPENDED, FAULTED | XSD CapabilityAvailabilityEnum | missing | Faulted exists but is never produced (grep); Unavailable and Expended do not exist | Produced by faults, fuel and configuration |  | FA-1, FA-16 | - |
| CAP-18 | DISABLED means switched off | XSD CapabilityAvailabilityEnum | defect | DISABLED is returned for an id the aircraft does not have (src/session/World.cpp:515) | Unsupported answered as such (CAP-22) |  | FA-1 | - |
| CAP-19 | AvailabilityReason: reasons, description, associated id; optional when it returns | XSD MA_AvailabilityInfoType; VI 1.2.2.6 | partial | One Reason | Several reasons, text, associated id |  | FA-1 | - |
| CAP-20 | NextAvailableWindow | XSD MA_AvailabilityInfoType | missing | - | When it is expected back |  | FA-1 | - |
| CAP-21 | Ready signal: FA offers control only when it can take it | VI 1.2.2.3-4 | defect | Probe: on the ground every airborne mode reports AVAILABLE, and an HSA flies away down the runway | Availability by flight phase (ground, takeoff, airborne, landing) |  | FA-1 | - |
| CAP-22 | Unsupported requests answered with an explicit reason | User requirement; XSD CAPABILITY_NOT_SUPPORTED | defect | Probe: 9 kinds of unsupported request, all unknown_capability; parameters the aircraft lacks: invalid_parameter with the field | not_supported with why: needs hover, not aerobatic, no such effector, not on this aircraft |  | FA-1 | - |
| CAP-23 | No capability advertised that does not work | User requirement | defect | Probe: aerobatics on 14 non-aerobatic aircraft (stalls; rq4b fell to the ground); waypoints circling on 14; evade into terrain on 4 rotorcraft; pursuit into the target on 30 | Withdraw or fix each (FA-1), and a test that flies every advertised capability on all 35 |  | FA-1 | - |
| CAP-24 | Status and admission agree | User requirement | defect | Probe: gear AVAILABLE on the ground while gear-up is refused unavailable (Adapter.cpp:232-242) | Placards reflected in the status |  | FA-1 | - |
| CAP-25 | Modify capabilities: FA restricts and restores with reasons | VI 1.2.2.6 | works | setAvailability with restricted, collision_avoidance, unavailable (World.h:208); controlRevision | - |  |  | `fsim.control/restrict` |
| CAP-26 | Parameters the aircraft has nothing for are marked | (platform) | works | ParameterInfo.supported (Capability.h:435); probe: refused with the field index | - |  |  | - |
| CAP-28 | PackageOperation and CapabilityPrecedence | XSD MA_FlightCapabilityType, MA_CapabilityStatusType | missing | - | Package coordination (formation); precedence |  | FA-12 | `fsim.guidance.formation/package` |
| CAP-29 | Not-yet-implemented told apart from physically unsupported and temporarily unavailable | Owner direction D5 | missing | - | A support state per item (supported, partial, not implemented, not supported) and the refusals not_implemented and not_supported |  | FA-1 | - |
| CAP-30 | A support table per vehicle: every public feature with its state, rule, evidence and stage | Owner direction D5 | missing | - | Queried by public feature identifier (D14: never by this record's ids) in C++, the C ABI and Python |  | FA-1 | - |
| CAP-31 | Every physical exception carries per-aircraft evidence | Owner direction D6 | missing | - | Declarations with their sources in each aircraft's files, checked against the model (SUB-01) |  | FA-1 | - |

**Control authority** (AUT)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| AUT-01 | ACQUIRE capability secondary control | VI 1.2.2.7; XSD ControlRequestMDT | works | requestControl (World.h:197); tests VI-7 | - |  |  | `fsim.control/grants` |
| AUT-02 | RELEASE | XSD ControlRequestEnum | works | releaseControl | - |  |  | `fsim.control/grants` |
| AUT-03 | APPROVED or REJECTED with a reason (PENDING not needed in-process) | XSD ApprovalStatusEnum | works | Reason::None or NotAllowed/restriction | - |  |  | `fsim.control/grants` |
| AUT-04 | Revocation with a reason; remaining capabilities reported | VI 1.2.2.7-8 | works | revokeControl; controlStatus per capability | - |  |  | `fsim.control/grants` |
| AUT-05 | Control status per capability: primary FA, secondary MA, accepted interfaces | VI 1.2.6.2; XSD ControlStatusCapabilityControlType | partial | ControlStatus {allowed, granted} | The accepted interfaces (CAP-03) |  | FA-1 | `fsim.control/grants` |
| AUT-06 | Controller identity: which MA service holds control (several services) | XSD SystemServiceType | missing | One policy per vehicle; sources are declared, not authenticated | Named controllers per vehicle, each with its grants |  | FA-2 | `fsim.control/controller_identity` |
| AUT-07 | C2 designations: the capabilities MA may command | VI 1.2.2.9 | works | setAllowed | - |  |  | `fsim.control/grants` |
| AUT-08 | FA is always primary and may take over | VI 1.2.6.2 | works | Autopilot and Override sources | - |  |  | `fsim.control/grants` |
| AUT-09 | When MA relinquishes control the vehicle continues safely | VI 1.2.2.1-3 | works | Vehicle default Hold (D5; review fix caf03c9) | - |  |  | `fsim.control/grants` |

**Route plans** (RPL)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RPL-01 | A store of route plans by id and version | VI 1.2.5; XSD MA_RoutePlanMDT | missing | The path store holds the active route only (Control.h:220) | Plans by id with versions, capacity fixed per vehicle |  | FA-7 | `fsim.plan/store` |
| RPL-02 | Activation states: prepare for upload, upload, prepare for activation, ready, activate, deactivate | VI 1.2.5.1-3 | missing | - | The state machine and its failures |  | FA-7 | `fsim.plan/store` |
| RPL-03 | FA-owned read-only plans: takeoff, departure, approach, landing, missed approach, aborted takeoff | VI 1.2.5.2, 1.2.6.3-4 | missing | - | Pre-loaded plans MA may activate but not replace |  | FA-7 | `fsim.plan/fa_plans` |
| RPL-04 | Airfields and runways (coordinates, direction, length, takeoff and landing points, limits, QNH) | VI 1.2.6.3; XSD AirfieldReportMDT | missing | - | An airfield store the launch and recovery modes use |  | FA-7 | `fsim.plan/airfields` |
| RPL-05 | Query plans and airfields; list ids; content revision (for checksums) | VI 1.2.4, 1.2.6.4 | missing | - | Queries; a revision number per plan |  | FA-7 | `fsim.plan/store` |
| RPL-06 | Validate a route plan (with weather) without flying it | VI 1.2.5.5 | missing | - | Plan validation (VAL-12) |  | FA-7 | `fsim.plan/validate` |
| RPL-07 | Verify a plan patch | VI 1.2.5.6 | missing | - | Validate a partial replacement |  | FA-7 | `fsim.plan/validate` |
| RPL-08 | Deactivate a plan (an executing one fails to deactivate) | VI 1.2.5.4 | missing | - | As the VI specifies |  | FA-7 | `fsim.plan/store` |
| RPL-09 | FA aborts a route before or after it starts (DEACTIVATED, execution CANCELED) | VI 1.2.5.7 | partial | Activity canceled or failed with a reason | The plan's state and execution status |  | FA-7 | `fsim.plan/store` |
| RPL-10 | Plan execution status (PENDING, EXECUTING, COMPLETE, SUPERCEDED, CANCELED, FAILED) | VI 1.2.6.6; XSD PlanExecutionStateEnum | partial | Activity state and progress | Per plan |  | FA-7 | `fsim.plan/store` |
| RPL-11 | ForPlanningUseOnly plans never activate | XSD MA_RoutePlanMDT | missing | - | Refused at activation |  | FA-7 | `fsim.plan/store` |

**Status and state data** (STS)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| STS-01 | Position, velocity, acceleration, orientation, orientation rate | VI 1.2.6.8; XSD MA_DetailedKinematicsType | works | VehicleState (VehicleState.h:16-39) | - |  |  | - |
| STS-02 | Orientation acceleration | XSD OrientationAccelerationType | missing | - | Angular acceleration |  | FA-4 | - |
| STS-03 | Air data TAS, CAS, Mach, alpha, beta | XSD MA_AirDataType | works | VehicleState airspeedTrueMs ... betaRad | - |  |  | - |
| STS-04 | Indicated barometric altitude, its rate, Kollsman | XSD MA_AirDataType | missing | - | Barometric altimeter (STS-10) |  | FA-4 | - |
| STS-05 | Magnetic heading | VI 1.2.6.8 | missing | - | Declination model (HSA-03) |  | FA-4 | - |
| STS-06 | Winds measured by the vehicle, with time and place | VI 1.2.6.8; XSD WindDataType | partial | WindEstimate inside guidance (GuidanceModes.h:20) | Reported |  | FA-4 | - |
| STS-07 | Navigation report: fuel mass, percent, endurance duration, playtime, contingency level | VI 1.2.3.1; XSD MA_NavigationReportMDT | partial | fuelKg | Percent (capacity), endurance, playtime (with a return estimate), contingency level |  | FA-3 | - |
| STS-08 | Navigation solution: source, state, figure of merit, covariance | XSD MA_PositionReportDataType | partial | Sensed state through effects (GNSS degradation) | Solution state, FOM and covariance reported |  | FA-16 | - |
| STS-09 | Faults: id, code, severity NOMINAL to FAILED, raised or cleared, affected capabilities, detection time | VI 1.2.1.4-5, 1.2.6.1; XSD MA_SubsystemFaultType | missing | - | Fault records and their effect on availability |  | FA-16 | `fsim.fault` |
| STS-10 | QNH setting (kPa): applied or failed with a reason | VI 1.2.6.5 | missing | - | Altimeter setting per vehicle |  | FA-4 | `fsim.setting/qnh` |
| STS-11 | Terrain data from FA for requested points | VI 1.2.6.9 | missing | GroundProvider has the elevation inside (GroundProvider.h) | An elevation query in the SDK |  | FA-4 | `fsim.query/terrain` |
| STS-12 | Component status (engines) | VI 1.2.6.8 | partial | Engine rpm, N2, thrust, afterburner in the state | A component state per engine |  | FA-16 | - |
| STS-13 | FA health (subsystem status) | VI 1.2.6.1, 1.2.6.10 | missing | - | FA state nominal, degraded, failed |  | FA-16 | - |
| STS-14 | Execution status of executing and queued activities, routes and plans | VI 1.2.6.6 | partial | Activities and progress | Queued activities (CMD-06) and plans (RPL-10) |  | FA-2 | - |
| STS-15 | Vehicle settings: gear extend and retract | XSD VehicleActionEnum | works | fsim.support.gear (29 aircraft with retractable gear) | - | R6 |  | `fsim.support.gear` |
| STS-16 | Vehicle settings: spoilers enable and disable | XSD VehicleActionEnum | missing | The speedbrake axis exists; no aircraft declares a drag device (all 35: speedbrake 0) | Spoilers and airbrakes in hangar where the type has them; the command | R8 | FA-10 | `fsim.support.speedbrake` |
| STS-17 | Vehicle settings: lights, antennas, transponder, radio channels, RF transmit, IFF, comm allocation | XSD VehicleActionEnum, MA_VehicleCommandDataType | missing | - | Settings kept and reported (no flight effect) |  | FA-14 | `fsim.setting/lights`, `fsim.setting/antennas`, `fsim.setting/transponder`, `fsim.setting/radio`, `fsim.setting/rf_transmit`, `fsim.setting/iff`, `fsim.setting/comm_allocation` |
| STS-18 | Vehicle settings: survivability mode | XSD VehicleSurvivabilityModeEnum | missing | - | A mode that constrains configuration use (flaps, gear, bay doors) |  | FA-14 | `fsim.setting/survivability` |
| STS-19 | Vehicle settings: lost-comm timeout, LOS backup, loss-of-link processing | XSD MA_VehicleCommandDataType | missing | - | With CTG-09 |  | FA-16 | `fsim.setting/lost_comm` |

**Weapon employment** (WPN)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| WPN-01 | Validate release envelope: accept the strike manoeuvre only where the release criteria are met | VI 1.2.7.1 | missing | No stores or weapons model | Stores per aircraft, release envelope checks, the command's traceability to its task | R9 | FA-13 | `fsim.command/release_envelope` |

**Flight tasks** (TSK)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TSK-01 | Flight tasks kept by id and executed on a task command; task command status | VI 1.2.2.5; XSD MA_TaskMT.Flight, MA_TaskCommandMT | missing | - | A flight-task store and execution |  | FA-2 | `fsim.command/task` |
| TSK-02 | Task status: execution state, percent, timing, traceability | XSD MA_TaskStatusMDT | missing | - | Reported per task |  | FA-2 | `fsim.command/task` |

### A.2 Platform behaviours (8 items)

**Platform behaviours** (PLT)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| PLT-01 | fsim.guidance.hold: heading, true airspeed and altitude | Platform (older than ADR-28) | partial | Probe: heading within 0.5 deg on 35 of 35; superseded by fsim.guidance.hsa. The fleet test (FA-1e): its fixed altitude gain circled the Mirage 2000 about its new altitude, 25 m either way | Marked as superseded in discovery; its altitude flown at the aircraft's own position-loop gain |  | FA-1 | `fsim.guidance.hold` |
| PLT-02 | fsim.guidance.waypoints completes where the aircraft can fly it | Platform | defect | Probe: never completes on 14 aircraft whose turn radius exceeds the capture radius; stays active, never fails | Refuse a point it cannot capture (invalid_waypoint with the point and max_turn_rate), and fail an activity that stops closing; superseded by fsim.guidance.route |  | FA-1 | `fsim.guidance.waypoints` |
| PLT-03 | fsim.guidance.loiter: a circle round a point | Platform | partial | Works; its 1,500 m default is clamped to the turn radius (7 to 12 km for the heavies). The fleet test (FA-1e): a vehicle that follows its carrot at once flies 17.5 % inside the circle | A default radius from the aircraft's performance; the circle flown trimmed to the one asked; superseded by fsim.guidance.pattern |  | FA-1 | `fsim.guidance.loiter` |
| PLT-04 | fsim.guidance.pursuit keeps its stand-off from the target | Platform | defect | Probe: closes to 0 to 33 m of the target on 30 aircraft | Hold range_m as the least range; close no faster than the aircraft can stop closing |  | FA-1 | `fsim.guidance.pursuit` |
| PLT-05 | fsim.guidance.evade stays above the terrain | Platform | defect | Probe: the default 300 m descent flew the four rotorcraft into the ground; the activity stayed active. The fleet test (FA-1e): a rotorcraft evading from a hover kept the airspeed it had, none | A floor above the terrain (the state's height above ground); a descent that would cross it is clamped and flagged; a rotorcraft flees at its cruise at least |  | FA-1 | `fsim.guidance.evade` |
| PLT-06 | fsim.guidance.formation: a slot behind a leader | Platform | defect | Probe: "converges on its slot" - but that measured only that the gap shrank. The fleet test (FA-1e): 33 of 35 end 48 m to 1.8 km off their slots after 150 s, and a rotorcraft cannot close sideways | A closing it can stop, as pursuit's; the slot's line joined along a route's look-ahead; a rotorcraft's slot flown over the ground. A-GRA's formation builds on it (FRM, FA-12) |  | FA-1 | `fsim.guidance.formation` |
| PLT-07 | fsim.guidance.aerobatics flown only by aerobatic types, within the envelope, and reported as it went | Platform | defect | Probe: offered to all 31 wings; 14 fell below their minimum CAS, the RQ-4B reached the ground and the EA-18G lost 1,946 m, all reported goal_reached | Offered by rule R10; an entry-energy check (refused performance_limit when too slow for the load factor); completed only if flown within the envelope, else failed | R10 | FA-1 | `fsim.guidance.aerobatics` |
| PLT-08 | fsim.guidance.hover: hold a point | Platform | works | Probe: drift 0.00 to 0.10 m on the four rotorcraft | Kept; A-GRA's hover loiter builds on it (LTR-15, FA-5) | R1 | FA-1 | `fsim.guidance.hover` |

### A.3 Flight Autonomy functions (13 items)

**Formation** (FRM)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FRM-05 | Tight formation safety (FA protection during close formation) | VI 1.4 rows 4.6, 5.4 | missing | No separation protection (probe: pursuit to 0-33 m) | Collision avoidance (CTG-01) under formation |  | FA-15 | `fsim.fa/tight_formation` |

**Launch** (LCH)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LCH-03 | Emergency divert on takeoff | VI 1.4 row 10.2 | missing | - | Divert to a stored alternate after takeoff |  | FA-16 | `fsim.guidance.launch/emergency_divert` |

**FA safety functions and contingencies** (CTG)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CTG-01 | Collision avoidance: detect, evade, capabilities TEMPORARILY_UNAVAILABLE with CONSTRAINT_COLLISION_AVOIDANCE, report, resume | VI 1.2.1.1; 1.4 row 4.9 | partial | The reason and the restriction exist (setAvailability); nothing detects or evades (probe: pursuit to 0-33 m) | Detection among world vehicles, an avoidance manoeuvre by the Override source, the reports |  | FA-15 | `fsim.fa/collision_avoidance` |
| CTG-02 | Geofences (hard limits, an altitude floor), stored and enforced by FA | VI 1.1; 1.4 row 4.2 | missing | - | Geofence store, enforcement, VIOLATION_GEOFENCE |  | FA-15 | `fsim.fa/geofence` |
| CTG-03 | Terrain avoidance and ground collision avoidance | VI 1.2.6.9; 1.4 row 4.5 | missing | Probe: evade flew 4 rotorcraft into the ground; aerobatics took rq4b to the ground | Terrain look-ahead, automatic recovery, VIOLATION_TERRAIN |  | FA-15 | `fsim.fa/terrain_avoidance` |
| CTG-04 | Envelope management: g, rate and speed limits; energy management; stall prevention | VI 1.4 row 2.5 | partial | Protection limits the demand and reports exceedances (Protection.h) | Energy management (HSA-10); stall margin held by every mode |  | FA-3 | `fsim.envelope.protection`, `fsim.fa/envelope_management` |
| CTG-05 | Vehicle fuel management: fuel state, abnormal flow, low-fuel procedure when MA is unresponsive | VI 1.4 row 2.4 | partial | fuelKg in the state | Fuel monitoring, leaks (faults), low-fuel procedure |  | FA-16 | `fsim.fa/fuel_management` |
| CTG-06 | Vehicle contingency management: emergency procedures; tell MA the contingency, remedy and capability changes | VI 1.1; 1.4 row 7.4 | missing | - | Procedures per family; reports; availability changes |  | FA-16 | `fsim.fa/contingency_management` |
| CTG-07 | Emergency divert or return to base when MA is unresponsive | VI 1.4 row 7.5 | missing | - | Auto-route to a stored location |  | FA-16 | `fsim.fa/emergency_divert` |
| CTG-08 | MA failsafe plan: transit routes plus a stored recovery route, executed on MA failure | VI 1.2.1.3 | missing | - | Failsafe designation, update by version, execution |  | FA-16 | `fsim.fa/failsafe_plan` |
| CTG-09 | Loss of MA (comms failure): a liveness timeout, loss-of-link processing, LOS backup | VI 1.2.1.2; VehicleSettings | missing | - | A liveness contract for the policy and the contingency it triggers |  | FA-16 | `fsim.fa/lost_link` |
| CTG-10 | Mechanical damage and sensor failure reported as faults | VI 1.2.1.4-5 | missing | Sensor degradation effects exist (BuiltinEffects.h); nothing reports them | Fault injection and reports (STS-09) |  | FA-16 | `fsim.fa/faults` |
| CTG-11 | Contingency level NORMAL, MISSION_CRITICAL, FLIGHT_CRITICAL, LOST_COMMS | XSD SystemContingencyLevelEnum | missing | - | Reported, and a branch condition (WPT-15) |  | FA-16 | `fsim.fa/contingency_management` |

### A.4 Supporting environment (10 items)

**Supporting environment** (ENV)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ENV-01 | Terrain elevation service | VI 1.2.6.9; for STS-11, VAL-06, CTG-03 | partial | The ground provider has elevation inside (GroundProvider.h) | A query in the SDK and a look-ahead along a path |  | FA-4 | `fsim.query/terrain` |
| ENV-02 | Magnetic field model | For HSA-03, STS-05 | missing | - | The World Magnetic Model's published coefficients and equations: declination at a place and date |  | FA-4 | - |
| ENV-03 | Barometric altimeter | For HSA-07, STS-04, STS-10 | missing | - | Static pressure from the atmosphere; indicated altitude for a QNH setting per vehicle |  | FA-4 | `fsim.setting/qnh` |
| ENV-04 | Reference frames, fixed or attached to a vehicle | For WPT-22, CRV-05, LTR-18, the carrier | missing | - | Frames by id; a point's position in one at a time |  | FA-4 | `fsim.frame` |
| ENV-05 | Airfields and runways | VI 1.2.6.3; for RPL-04, LCH, RCV | missing | - | Runway ends, direction, length, elevation, takeoff and landing points, limits, QNH; loaded before a mission, read-only |  | FA-7 | `fsim.plan/airfields` |
| ENV-06 | Operational geometry by id: points, lines, zones, volumes | For MFY-03 | missing | - | A store the must-fly locations name |  | FA-8 | `fsim.geometry` |
| ENV-07 | A ship: a moving deck | For LCH-05 to 07, RCV-05 to 10 | missing | - | Minimal and deterministic (D8): course and speed, a deck frame, optional pitch, roll and heave as sums of sines from the world's seed |  | FA-11 | `fsim.ship` |
| ENV-08 | Geofences | VI 1.4 row 4.2; for CTG-02, VAL-04 | missing | - | Hard limits loaded before a mission: an altitude floor, keep-in and keep-out areas |  | FA-15 | `fsim.fa/geofence` |
| ENV-09 | Traffic picture | VI 1.4 row 4.9; for CTG-01, VAL-05 | missing | - | The world's other vehicles and their predicted paths |  | FA-15 | - |
| ENV-10 | Weather for validation | VI 1.2.5.5; for RPL-06 | partial | The environment's wind, gusts and atmosphere | Given with a plan to validate |  | FA-7 | `fsim.plan/validate` |

### A.5 Supporting subsystems (10 items)

**Supporting subsystems** (SUB)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SUB-01 | Applicability declarations with per-aircraft evidence | Owner direction D6; for CAP-31 | missing | - | Each aircraft's design declares its physical characteristics (section 5.2) with a source; built into its profile; a test checks them against the model |  | FA-1 | - |
| SUB-02 | Performance tables | For CAP-04 to CAP-15 | partial | The profile's envelope and performance sections (single values) | hangar computes speeds, climb, excess power, turn and rate limits against altitude and weight from its own aerodynamics and engines |  | FA-3 | - |
| SUB-03 | Fuel flow and endurance | For CAP-14, VAL-03, STS-07 | partial | Fuel mass in the state | Fuel flow tables, tank capacity, percent, endurance and playtime |  | FA-3 | - |
| SUB-04 | Drag devices: spoilers, airbrakes, speedbrake surfaces | For STS-16, CAP-13, RCV-08 | missing | The speedbrake axis exists; no aircraft declares one | hangar models them where the type has them (rule R8) | R8 | FA-10 | `fsim.support.speedbrake` |
| SUB-05 | Tailhook and launch bar | For RCV-09, LCH-05 | missing | - | On the F/A-18C and EA-18G (rule R3) | R3 | FA-11 | - |
| SUB-06 | Catapult and arresting gear | For LCH-05, RCV-05 | missing | - | Minimal and deterministic (D8): a launch force profile to an end speed, an arresting force profile to a stop | R3 | FA-11 | - |
| SUB-07 | Stores | VI 1.2.7.1; for WPN-01 | missing | - | Minimal and deterministic (D8): stations, mass and drag, release (rule R9) | R9 | FA-13 | `fsim.stores` |
| SUB-08 | Equipment state | For STS-17 | missing | - | Settings kept and reported; each declared with no physical effect (D9) |  | FA-14 | - |
| SUB-09 | Faults | VI 1.2.1.4-5; for STS-09, CTG-10 | missing | Sensor degradation effects (BuiltinEffects.h) | Injectable failures with built-in test and health |  | FA-16 | `fsim.fault` |
| SUB-10 | Navigation solution | For STS-08 | partial | The sensed state through effects | Solution source, state, figure of merit and covariance |  | FA-16 | - |

### A.6 Transport and message format (11 items)

**Command envelope and lifecycle** (CMD)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CMD-11 | Classification label for resulting data | XSD SecurityInformationType | out of scope | - | Message labelling; nothing to fly |  |  | - |

**Discovery, performance and availability** (CAP)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CAP-27 | CommsRequired, DomainPairing, ProductOutput, MDFs | XSD MA_FlightCapabilityType | out of scope | - | Constant for this platform (no comms needed, air domain, no products) |  |  | - |

**FA safety functions and contingencies** (CTG)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CTG-12 | Zeroize | VI 1.4 row 7.1 | out of scope | - | Data handling, nothing to fly |  |  | - |

**Transport and message format** (XPT)

| Id | Item | Source | Status | Exists today | Missing | Rule | Stage | Public ids |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| XPT-01 | UCI 2.3 and A-GRA XML messages, schema validation, headers, UUIDs | VI 1.1, 1.3; XSD | out of scope | Native types and ids | - |  |  | - |
| XPT-02 | OMS isolator, publish and subscribe, message rates | VI 1.1, 1.2.3 | out of scope | Polling of records, progress and revisions | - |  |  | - |
| XPT-03 | ServiceStatus and SubsystemStatus heartbeats, data requests | VI 1.2.1.2, 1.2.6.1, 1.2.6.10 | out of scope | In-process calls; FA health is STS-13 | - |  |  | - |
| XPT-04 | System notifications acknowledging receipt | VI 1.2.1.3, 1.2.2.5, 1.2.5.2 | out of scope | Synchronous results | - |  |  | - |
| XPT-05 | Malformed message response (MA-L1-020) | VI 1.2.1.6 | out of scope | Typed calls; invalid input refused invalid_parameter | - |  |  | - |
| XPT-06 | QueryDataRequest states QUEUED and PROCESSING; results in native format | VI 1.2.4, 1.2.6.11 | out of scope | Synchronous queries | - |  |  | - |
| XPT-07 | ProductLocation and ProductMetadata SHA-2 hashes | VI 1.2.4.1 | out of scope | A revision per stored plan stands for the hash (RPL-05) | - |  |  | - |
| XPT-08 | System management requests outside flight: SetMode, SetIdentity, Link 16 metadata, voice control, sensor reporting | XSD MA_SystemManagementRequestType | out of scope | Mission systems, not flight | - |  |  | - |


## Appendix B: applicability by aircraft

Key: **W** works, **P** partial, **M** missing (not implemented yet), **X** physically inapplicable (the rule in the column; evidence in Appendix C), **?** evidence pending (applicable until decided), **D** a defect FA-1 fixes.

| Aircraft | HSA | Route | Taxi R2 | Curve | Patterns | Hold | Hover R1 | Must fly | Form. | Marshall | Intercept | Takeoff | Landing | Carrier launch R3/R4 | Carrier recovery R3/R4 | Hook R5 | Gear R6 | Flaps R7 | Drag R8 | Release R9 | Aerobatics R10 | Baro, magnetic | Speed opt. | Perf. tables | Terrain, geofence, collision | Faults | Route plans | Probe findings (Appendix D) |
| --- | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | --- |
| a10c (A-10C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | waypoints circle; aerobatics at 24 m/s CAS (min 56); HSA on the runway lifts off, gear down |
| b52h (B-52H) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | X | M | M | P | M | M | M | waypoints circle; pursuit to 6.9 m; aerobatics at 51 m/s CAS (min 58); HSA on the runway rolls |
| c130j (C-130J) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | X | M | X | M | M | P | M | M | M | waypoints circle; pursuit to 16.0 m; aerobatics at 42 m/s CAS (min 57); HSA on the runway lifts off, gear down |
| c172 (Cessna 172P) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | X | M | X | X | X | M | M | P | M | M | M | aerobatics at 9 m/s CAS (min 25); HSA on the runway lifts off, gear down |
| e3g (E-3G) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 2.2 m; aerobatics at 7 m/s CAS (min 81); HSA on the runway lifts off, gear down |
| e7a (E-7A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 5.4 m; aerobatics at 51 m/s CAS (min 62); HSA on the runway rolls |
| ec130h (EC-130H) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | X | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 33.1 m; aerobatics at 20 m/s CAS (min 55); HSA on the runway lifts off, gear down |
| h6k (H-6K) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | ? | M | X | M | M | P | M | M | M | waypoints circle; pursuit to 5.5 m; aerobatics at 43 m/s CAS (min 56); HSA on the runway rolls |
| kc135r (KC-135R) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 2.8 m; aerobatics at 29 m/s CAS (min 71); HSA on the runway rolls |
| kc46a (KC-46A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 3.5 m; aerobatics at 13 m/s CAS (min 83); HSA on the runway rolls |
| rc135w (RC-135W) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 4.7 m; aerobatics at 26 m/s CAS (min 75); HSA on the runway lifts off, gear down |
| rq4b (RQ-4B) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 1.6 m; aerobatics at 0 m/s CAS (min 39); aerobatics lost 2999 m; HSA on the runway lifts off, gear down |
| skua (Skua (hypothetical)) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | X | M | X | X | X | M | M | P | M | M | M | aerobatics at 4 m/s CAS (min 14); HSA on the runway lifts off, gear down |
| su25 (Su-25) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | waypoints circle; pursuit to 4.8 m; HSA on the runway lifts off, gear down |
| u2s (U-2S) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | X | X | M | M | P | M | M | M | waypoints circle; pursuit to 1.9 m; aerobatics at 42 m/s CAS (min 44); HSA on the runway lifts off, gear down |
| c17a (C-17A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | X | M | M | P | M | M | M | waypoints circle; pursuit to 4.8 m; HSA on the runway rolls |
| ea18g (EA-18G) | P | P | M | P | P | P | X | M | P | M | M | M | M | M | M | M | W | M | M | M | D | M | M | P | M | M | M | pursuit to 1.8 m; aerobatics lost 1946 m; HSA on the runway rolls, wings to 23 deg |
| f15c (F-15C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 6.3 m; HSA on the runway rolls |
| f16c (F-16C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 8.1 m; HSA on the runway rolls, wings to 27 deg |
| f22a (F-22A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 5.5 m; HSA on the runway rolls, wings to 19 deg |
| f35a (F-35A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 9.9 m; HSA on the runway rolls |
| fa18c (F/A-18C) | P | P | M | P | P | P | X | M | P | M | M | M | M | M | M | M | W | M | M | M | W | M | M | P | M | M | M | pursuit to 9.1 m; HSA on the runway rolls |
| gripen (JAS 39C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | ? | M | M | W | M | M | P | M | M | M | pursuit to 2.9 m; HSA on the runway rolls, wings to 34 deg |
| j10a (J-10A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 3.4 m; HSA on the runway rolls, wings to 39 deg |
| j20a (J-20A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 3.9 m; HSA on the runway rolls, wings to 36 deg |
| mig29a (MiG-29A) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 2.1 m; HSA on the runway rolls, wings to 31 deg |
| mirage2000 (Mirage 2000C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | X | M | M | W | M | M | P | M | M | M | pursuit to 7.2 m; HSA on the runway rolls, wings to 42 deg |
| rafale (Rafale C) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | ? | M | M | W | M | M | P | M | M | M | pursuit to 6.5 m; HSA on the runway rolls, wings to 27 deg |
| su27s (Su-27S) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 7.6 m; HSA on the runway rolls, wings to 19 deg |
| su57 (Su-57) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | M | M | M | W | M | M | P | M | M | M | pursuit to 2.0 m; HSA on the runway rolls, wings to 29 deg |
| typhoon (Typhoon) | P | P | M | P | P | P | X | M | P | M | M | M | M | X | X | X | W | ? | M | M | W | M | M | P | M | M | M | pursuit to 8.2 m; HSA on the runway rolls, wings to 27 deg |
| uh1h (UH-1H) | P | P | X | P | P | P | P | M | P | M | M | M | M | M | M | X | X | X | X | M | X | M | M | P | M | M | M | evade into the ground |
| uh60 (UH-60A) | P | P | M | P | P | P | P | M | P | M | M | M | M | M | M | X | X | X | X | M | X | M | M | P | M | M | M | evade into the ground |
| cf2 (Crazyflie 2.0) | P | P | X | P | P | P | P | M | P | M | M | M | M | M | M | X | X | X | X | X | X | M | M | P | M | M | M | pursuit to 1.7 m; evade into the ground |
| iris (IRIS+) | P | P | X | P | P | P | P | M | P | M | M | M | M | M | M | X | X | X | X | X | X | M | M | P | M | M | M | pursuit to 0.0 m; evade into the ground |

## Appendix C: per-aircraft evidence for the physical exceptions

Model evidence cites the aircraft files at caf03c9. "Not probative" means the model cannot show the exception, because no aircraft models the feature yet. The type facts are what FA-1 records with a public source in each aircraft's `[applicability]` declaration (5.2). Until a source is recorded, discovery reports the item as applicable (not implemented), never as not supported. "Pending" rows are not exceptions yet: type data decides them.

120 exceptions established on the evidence below, 4 pending.

| Aircraft | Rule | Excluded | Model evidence | Type fact (FA-1 records its source) |
| --- | --- | --- | --- | --- |
| a10c (A-10C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/a10c/a10c.xml:361` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | USAF land-based close-support type |
| b52h (B-52H) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/b52h/b52h.xml:597` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | USAF land-based bomber |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Bomber: load factor limit below 6 g, not cleared for aerobatics |
| c130j (C-130J) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/c130j/c130j.xml:469` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based transport (the 1963 KC-130F carrier trials used neither catapult nor hook) |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | No spoilers or airbrakes |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Transport: load factor limit below 6 g, not cleared for aerobatics |
| c172 (Cessna 172P) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/c172/c172.xml:333` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Civil light aircraft |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/c172/c172.xml:341` retractable_gear 0 | Fixed tricycle gear |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | No spoilers or airbrakes |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Civil type with no stores provisions |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Light aircraft (utility category): load factor limit below 6 g, not cleared for aerobatics |
| e3g (E-3G) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/e3g/e3g.xml:456` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based 707 derivative |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed AEW&C |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | AEW&C aircraft: load factor limit below 6 g, not cleared for aerobatics |
| e7a (E-7A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/e7a/e7a.xml:350` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based 737 derivative |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed AEW&C |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | AEW&C aircraft: load factor limit below 6 g, not cleared for aerobatics |
| ec130h (EC-130H) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/ec130h/ec130h.xml:469` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based C-130H derivative |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | No spoilers or airbrakes (C-130H airframe) |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | EW mission equipment fills the hold; no stations |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | EW aircraft: load factor limit below 6 g, not cleared for aerobatics |
| h6k (H-6K) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/h6k/h6k.xml:347` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based bomber |
|  | R8 (pending) | STS-16 spoilers and airbrakes | Not probative | To decide from type data. Applicable until then |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Bomber: load factor limit below 6 g, not cleared for aerobatics |
| kc135r (KC-135R) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/kc135r/kc135r.xml:499` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based tanker |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed tanker |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Tanker: load factor limit below 6 g, not cleared for aerobatics |
| kc46a (KC-46A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/kc46a/kc46a.xml:372` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based 767 derivative |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed tanker |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Tanker: load factor limit below 6 g, not cleared for aerobatics |
| rc135w (RC-135W) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/rc135w/rc135w.xml:481` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based C-135 derivative |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed reconnaissance |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Reconnaissance aircraft: load factor limit below 6 g, not cleared for aerobatics |
| rq4b (RQ-4B) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/rq4b/rq4b.xml:249` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based UAV |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed reconnaissance UAV |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Reconnaissance UAV: load factor limit below 6 g, not cleared for aerobatics |
| skua (Skua (hypothetical)) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/skua/skua.xml:270` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Hypothetical land-based UAV (its design file defines no hook or launch bar) |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/skua/skua.xml:278` retractable_gear 0 | Fixed gear (its design) |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | Hypothetical; its design file defines flaps only |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Hypothetical; its design file defines no stations |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | UAV: load factor limit below 6 g, not cleared for aerobatics |
| su25 (Su-25) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/su25/su25.xml:336` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (the Su-25UTG naval trainer is a different type) |
| u2s (U-2S) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/u2s/u2s.xml:307` family 1 (direct) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (the 1960s carrier trials used modified U-2G and U-2R aircraft) |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | Unarmed reconnaissance |
|  | R10 (platform) | PLT-07 aerobatics | No n_max in the aircraft file's envelope | Reconnaissance aircraft: load factor limit below 6 g, not cleared for aerobatics |
| c17a (C-17A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/c17a/c17a.xml:506` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based transport |
|  | R10 (platform) | PLT-07 aerobatics | `aircraft/c17a/c17a.xml:522` n_max 2.5 g | Transport: load factor limit below 6 g, not cleared for aerobatics |
| ea18g (EA-18G) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/ea18g/ea18g.xml:395` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
| f15c (F-15C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/f15c/f15c.xml:336` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | USAF land-based (runway emergency hook only) |
| f16c (F-16C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/f16c/f16c.xml:317` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | USAF land-based (runway emergency hook only) |
| f22a (F-22A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/f22a/f22a.xml:380` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | USAF land-based (runway emergency hook only) |
| f35a (F-35A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/f35a/f35a.xml:395` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering (the F-35B, not the A, is the STOVL variant) |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | CTOL land-based variant (the F-35C is the carrier variant) |
| fa18c (F/A-18C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/fa18c/fa18c.xml:348` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
| gripen (JAS 39C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/gripen/gripen.xml:293` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (runway hook only; the Sea Gripen was not built) |
|  | R7 (pending) | The flaps in RCV-08 and RCV-10 | `aircraft/gripen/gripen.xml:296` flaps 0 (the model omits a flap function) | To decide from type data: canard delta whose trailing-edge surfaces may act as flaperons. Applicable until then |
| j10a (J-10A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/j10a/j10a.xml:315` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based |
| j20a (J-20A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/j20a/j20a.xml:384` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based |
| mig29a (MiG-29A) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/mig29a/mig29a.xml:362` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (the MiG-29K is the naval type) |
| mirage2000 (Mirage 2000C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/mirage2000/mirage2000.xml:257` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based |
|  | R7 | The flaps in RCV-08 and RCV-10 | `aircraft/mirage2000/mirage2000.xml:260` flaps 0 | Tailless delta with elevons: no flap function |
| rafale (Rafale C) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/rafale/rafale.xml:325` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Rafale C: land-based (the Rafale M is the naval variant) |
|  | R7 (pending) | The flaps in RCV-08 and RCV-10 | `aircraft/rafale/rafale.xml:328` flaps 0 (the model omits a flap function) | To decide from type data: canard delta whose trailing-edge surfaces may act as flaperons. Applicable until then |
| su27s (Su-27S) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/su27s/su27s.xml:350` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (the Su-33 is the naval type) |
| su57 (Su-57) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/su57/su57.xml:372` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based |
| typhoon (Typhoon) | R1 | LTR-15 hover loiter; ASM-01's hover option; vertical deck operations (R4) | `aircraft/typhoon/typhoon.xml:337` family 2 (fly-by-wire) | Conventional takeoff and landing type: no rotors, lift fans or vectored lift for hovering |
|  | R3, R5 | LCH-05 to LCH-07, RCV-05 to RCV-07, RCV-09, RCV-10 (catapult, arrested landing, arrester hook) | Not probative: no aircraft models a hook or launch bar yet | Land-based (runway hook only) |
|  | R7 (pending) | The flaps in RCV-08 and RCV-10 | `aircraft/typhoon/typhoon.xml:340` flaps 0 (the model omits a flap function) | To decide from type data: canard delta whose trailing-edge surfaces may act as flaperons. Applicable until then |
| uh1h (UH-1H) | R2 | WPT-26 taxi paths | `aircraft/uh1h/uh1h.toml:108 kind = "skid"` | Skids: it cannot roll |
|  | R3, R5 | Catapult launch, arrested landing, arrester hook (deck operations under R4 still apply) | `aircraft/uh1h/uh1h.xml:94` family 3 (rotorcraft) | Rotorcraft: takes off and lands vertically; no launch bar or arrester hook |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/uh1h/uh1h.xml:101` retractable_gear 0; `aircraft/uh1h/uh1h.toml:108 kind = "skid"` | Skids |
|  | R7 | The flaps in RCV-08 and RCV-10 | `aircraft/uh1h/uh1h.xml:97` flaps 0 | Rotorcraft: no flaps |
|  | R5, R6, R7 | RCV-08 CleanUp and DirtyUp as a whole (nothing to retract or deploy) | `aircraft/uh1h/uh1h.xml:101` retractable_gear 0, `aircraft/uh1h/uh1h.xml:97` flaps 0 | No retractable gear, flaps or hook |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | Rotorcraft: no drag devices |
|  | R10 (platform) | PLT-07 aerobatics | `aircraft/uh1h/uh1h.xml:94` family 3 (rotorcraft) | Rotorcraft: the manoeuvres are a wing's |
| uh60 (UH-60A) | R3, R5 | Catapult launch, arrested landing, arrester hook (deck operations under R4 still apply) | `aircraft/uh60/uh60.xml:83` family 3 (rotorcraft) | Rotorcraft: takes off and lands vertically; no launch bar or arrester hook |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/uh60/uh60.xml:90` retractable_gear 0; `aircraft/uh60/uh60.toml:106 kind = "wheels" (fixed)` | Fixed wheeled gear |
|  | R7 | The flaps in RCV-08 and RCV-10 | `aircraft/uh60/uh60.xml:86` flaps 0 | Rotorcraft: no flaps |
|  | R5, R6, R7 | RCV-08 CleanUp and DirtyUp as a whole (nothing to retract or deploy) | `aircraft/uh60/uh60.xml:90` retractable_gear 0, `aircraft/uh60/uh60.xml:86` flaps 0 | No retractable gear, flaps or hook |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | Rotorcraft: no drag devices |
|  | R10 (platform) | PLT-07 aerobatics | `aircraft/uh60/uh60.xml:83` family 3 (rotorcraft) | Rotorcraft: the manoeuvres are a wing's |
| cf2 (Crazyflie 2.0) | R2 | WPT-26 taxi paths | `aircraft/cf2/cf2.toml:43 leg_height_m (legs, no wheels)` | Legs, no wheels: it cannot roll |
|  | R3, R5 | Catapult launch, arrested landing, arrester hook (deck operations under R4 still apply) | `aircraft/cf2/cf2.xml:108` family 4 (rotorcraft) | Rotorcraft: takes off and lands vertically; no launch bar or arrester hook |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/cf2/cf2.xml:115` retractable_gear 0; `aircraft/cf2/cf2.toml:43 leg_height_m (legs, no wheels)` | Fixed legs |
|  | R7 | The flaps in RCV-08 and RCV-10 | `aircraft/cf2/cf2.xml:111` flaps 0 | Rotorcraft: no flaps |
|  | R5, R6, R7 | RCV-08 CleanUp and DirtyUp as a whole (nothing to retract or deploy) | `aircraft/cf2/cf2.xml:115` retractable_gear 0, `aircraft/cf2/cf2.xml:111` flaps 0 | No retractable gear, flaps or hook |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | Rotorcraft: no drag devices |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | No stations or release mechanism |
|  | R10 (platform) | PLT-07 aerobatics | `aircraft/cf2/cf2.xml:108` family 4 (rotorcraft) | Rotorcraft: the manoeuvres are a wing's |
| iris (IRIS+) | R2 | WPT-26 taxi paths | `aircraft/iris/iris.toml:38 leg_height_m (legs, no wheels)` | Legs, no wheels: it cannot roll |
|  | R3, R5 | Catapult launch, arrested landing, arrester hook (deck operations under R4 still apply) | `aircraft/iris/iris.xml:108` family 4 (rotorcraft) | Rotorcraft: takes off and lands vertically; no launch bar or arrester hook |
|  | R6 | STS-15 gear action; the gear in RCV-08 and RCV-10 | `aircraft/iris/iris.xml:115` retractable_gear 0; `aircraft/iris/iris.toml:38 leg_height_m (legs, no wheels)` | Fixed legs |
|  | R7 | The flaps in RCV-08 and RCV-10 | `aircraft/iris/iris.xml:111` flaps 0 | Rotorcraft: no flaps |
|  | R5, R6, R7 | RCV-08 CleanUp and DirtyUp as a whole (nothing to retract or deploy) | `aircraft/iris/iris.xml:115` retractable_gear 0, `aircraft/iris/iris.xml:111` flaps 0 | No retractable gear, flaps or hook |
|  | R8 | STS-16 spoilers and airbrakes; deceleration by them | Not probative: no aircraft models a drag device yet (all speedbrake 0) | Rotorcraft: no drag devices |
|  | R9 | WPN-01 release envelope | Not probative: no aircraft models stores yet | No stations or release mechanism |
|  | R10 (platform) | PLT-07 aerobatics | `aircraft/iris/iris.xml:108` family 4 (rotorcraft) | Rotorcraft: the manoeuvres are a wing's |

## Appendix D: the discovery probe (2026-09-27)

Run on the built SDK at caf03c9, seed 11, one worker. It changed no file in the repository. Its script, `probe_discovery.py`, and its results are kept outside the repository with the audit.

**What discovery advertised**

| Capability | A-GRA type | Aircraft |
| --- | --- | ---: |
| `fsim.envelope.protection` | - | 35 |
| `fsim.flight.acceleration` | - | 35 |
| `fsim.flight.actuator` | - | 35 |
| `fsim.flight.attitude` | - | 35 |
| `fsim.flight.engines` | - | 24 |
| `fsim.flight.position` | - | 35 |
| `fsim.flight.velocity` | - | 35 |
| `fsim.guidance.aerobatics` | - | 31 |
| `fsim.guidance.curve` | CURVE_FOLLOWING | 35 |
| `fsim.guidance.evade` | - | 35 |
| `fsim.guidance.formation` | FORMATION | 35 |
| `fsim.guidance.hold` | - | 35 |
| `fsim.guidance.hover` | LOITER | 4 |
| `fsim.guidance.hsa` | HSA_CSA | 35 |
| `fsim.guidance.loiter` | - | 35 |
| `fsim.guidance.pattern` | LOITER | 35 |
| `fsim.guidance.pursuit` | - | 35 |
| `fsim.guidance.route` | WAYPOINT_FOLLOWING | 35 |
| `fsim.guidance.waypoints` | - | 35 |
| `fsim.support.flaps` | - | 27 |
| `fsim.support.gear` | - | 29 |
| `fsim.support.pitch_trim` | - | 15 |
| `fsim.support.wheel_brakes` | - | 32 |

**What an unsupported request was told**

| Requested | Status | NEW answer | Aircraft |
| --- | --- | --- | ---: |
| `fsim.flight.engines` | DISABLED / unknown_capability | `unknown_capability` | 11 |
| `fsim.guidance.aerobatics` | DISABLED / unknown_capability | `unknown_capability` | 4 |
| `fsim.guidance.hover` | DISABLED / unknown_capability | `unknown_capability` | 31 |
| `fsim.guidance.no_such_thing` | DISABLED / unknown_capability | `unknown_capability` | 35 |
| `fsim.support.flaps` | DISABLED / unknown_capability | `unknown_capability` | 8 |
| `fsim.support.gear` | DISABLED / unknown_capability | `unknown_capability` | 6 |
| `fsim.support.pitch_trim` | DISABLED / unknown_capability | `unknown_capability` | 20 |
| `fsim.support.speedbrake` | DISABLED / unknown_capability | `unknown_capability` | 35 |
| `fsim.support.wheel_brakes` | DISABLED / unknown_capability | `unknown_capability` | 3 |

**Smoke flights** (90 to 150 s; a wing at 3,000 m at its reference airspeed, a rotorcraft hovering at 150 m)

| Capability | Result |
| --- | --- |
| `hsa`, `hold` | Turn 90 degrees and climb: heading within 3 degrees on 35 of 35; altitude within 9 m on 34. The C172 at 3,000 m bled from 50 to 30 m/s TAS, below its minimum CAS, 100 m short (HSA-10) |
| `route` | Two legs with a 90 degree turn: completed on 35 of 35 |
| `curve` | One 20 s S segment: completed on 19; refused `invalid_curve`, segment 0, `max_turn_rate` on the 16 that cannot turn so tightly |
| `pattern` | Default orbit flown on 35 of 35 (radius from speed and bank: 1 m to 11.6 km) |
| `hover` | Drift 0.00 to 0.10 m on the 4 rotorcraft |
| `formation` | Converged on its slot (fighters: 1.5 km to 100-220 m in 90 s) |
| `waypoints` | Not completed in 150 s on 14 aircraft: circling a point it cannot capture, active, no failure |
| `loiter` | Its 1,500 m default clamped to the turn radius (7 to 12 km for the heavies) |
| `aerobatics` | All 31 reported completed; 14 fell below the envelope minimum CAS; the RQ-4B reached the ground; the EA-18G lost 1,946 m |
| `evade` | The 4 rotorcraft descended into the ground from 150 m |
| `pursuit` | Closed to 0 to 33 m of the target on 30 aircraft |

**On the ground** (all 31 wings on a runway): `hsa`, `route` and `velocity` reported AVAILABLE; `fsim.support.gear` AVAILABLE while gear-up was refused `unavailable`. An HSA to 60 m/s and 300 m up along the runway: 10 aircraft lifted off within 60 s with the gear down and climbed 107 to 395 m (a10c, c130j, c172, e3g, ec130h, rc135w, rq4b, skua, su25, u2s); the other 21 stayed on the runway, 12 of them rolling their wings 19 to 42 degrees on the wheels.

**Performance unknown** (`Vehicle.performance`): maximum CAS on all 31 wings; Mach on all 35; minimum CAS on the 16 fly-by-wire aircraft; pitch limits on all wings; roll rate on the 15 direct-control wings; deceleration on all wings; ceiling on the rotorcraft.
