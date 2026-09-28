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

### 4.8 The command envelope (as FA-2 builds it)

What a command carries beside its setpoint, and what its answer carries beside its reason: A-GRA's CapabilityCommandBaseType, ResultingActivityType, CannotComplyType and MA_CannotComplyDetails, natively.

- **Its id and requirements (CMD-02, CMD-10, ACT-08).** `CommandOptions::commandId` is the caller's id for the command. Every answer about it echoes it (NEW, UPDATE and CANCEL, refused ones too), and its activity's record keeps it. `trace` names up to four requirements the command comes from (an effect, an action, a task or another command), kept likewise. The platform's activity id stays the handle.
- **Interactive (CMD-16).** Whether its activity takes activity commands, kept in the record.
- **New or not (CMD-18).** `CommandResult::newActivity` is true for a NEW that made an activity. It is false for an UPDATE, a CANCEL, a command the live activity takes (the existing entry points) and a validation.
- **Why, and about what (CMD-19, VAL-08).** `reasonDescription(reason)` puts the reason in words; `other` is the id it is about (the activity holding the authority). The C ABI and Python carry both in the answer's detail.
- **Every finding (VAL-01, VAL-02, WPT-24).** Under `RangePolicy::Reject` a command is checked as Clamp would check it. Each value beyond the aircraft is held there so that checking goes on, and each is a finding. The answer is refused with the first, which is where checking used to stop: every earlier answer and every digest is unchanged. `World::commandDetails` has them all: every point and field of a route at fault (the climb its held altitude still asks, too), every section of a curve too tight for the aircraft, an hsa's altitude and speed. A malformed command still stops at its first fault. `fsim.agra` maps them to A-GRA's ValidationResult list and a route's to its RouteValidationErrorEnum.
- **Every value flown other than asked (CMD-20).** Under Clamp each value held is an adjustment: the field (a route point's field), the limit, what was asked and what flies. The answer's `kClamped` and first-clamp detail are as before.
- **Validation (VAL-12).** With `validateOnly`, a command gets every check a NEW makes, authority and axes included. The answer is `Valid` or a refusal with every finding; nothing flies and no record changes. The conformance walks validate a quarter of their NEWs first and hold each NEW to the same answer.
- **Batches (CMD-03).** `World::submitBatch` makes several NEWs in order at one simulation time and answers each on its own, with each one's details.

| C++ | C ABI 1.8 | Python |
| --- | --- | --- |
| `CommandOptions` gains `commandId`, `trace` (`Requirement {kind, id}` × 4; `RequirementKind`), `interactive`, `validateOnly`; `CommandStatus::Valid` | `fsim_command_options` (its `struct_size` grows) gains `command_id`, `trace` (`fsim_requirement`), `interactive`, `validate_only`; `FSIM_COMMAND_VALID` | every `submit*` takes `command_id=`, `trace=[("task", 7), ...]`, `interactive=`, `validate_only=` (then a `fsim.Validation`) |
| `CommandResult` gains `newActivity`, `commandId` (`other` is the id a reason is about; `reasonDescription(reason)` the words) | `fsim_command_detail` gains `new_activity`, `finding_count`, `adjustment_count`, `command_id`, `associated`, `description` | `Activity.command_id`; `fsim.Rejected` gains `description`, `associated`, `command_id`, `findings`, `adjustments` |
| `CommandDetails {findings[16], adjustments[16]}` (`Finding`, `Adjustment`) by `World::commandDetails(id)`, `Vehicle::commandDetails()` | `fsim_last_command_finding(world, i, &f)`, `fsim_last_command_adjustment(world, i, &a)` | `World.last_command_details()` → (`[fsim.Finding]`, `[fsim.Adjustment]`) |
| `ActivityRecord` gains `commandId`, `trace`, `interactive` | `fsim_activity_get_envelope(world, activity, &e)` (`fsim_activity_info` has no `struct_size`) | `ActivityInfo.command_id`, `.interactive`, `.trace` |
| `World::submitBatch(id, Span<const BatchCommand>, &details)`; `BatchCommand {command, waypoints, segments, options}` | `fsim_vehicle_submit_batch(world, id, batch, count, results, details)` (`fsim_batch_command`: the kind, its call's arguments) | `vehicle.submit_batch([fsim.BatchCommand("submit_hsa", heading_rad=1.0), ...])` → an `Activity`, `Validation` or `Rejected` each |

### 4.9 Ranks, queues and time windows (as FA-2 builds it)

How contested axes are arbitrated, and when a command flies: A-GRA's CapabilityCommandRankingType, ComparableRankingType, CapabilityCommandTemporalConstraintsType and the activity's rank, basis and constrained state, natively.

- **Ranks (CMD-05, ACT-07).** `CommandOptions::rank` is `{priority, precedence}`, lower first; the activity keeps it. `{0, 0}` is every command's without one, and ranks first of all.
- **Capability precedence (CMD-07).** A capability has a precedence, lower first and 0 until the platform sets one (`World::setCapabilityPrecedence`). The schema says nothing about its direction, so it is read as a rank's precedence. A command may override it for itself (`precedenceOverride`), from the platform's own sources only; a policy's override is refused `not_allowed`. The activity keeps the precedence it is arbitrated by.
- **Who takes contested axes (CMD-06).** A NEW contests each live activity on its axes:
  - a higher source's activity refuses an interrupting command `authority_held`, as ADR-26 has it, and is waited for by one that does not interrupt;
  - the platform's interrupting command takes any rank (A-GRA: the primary controller "interrupts activities of any Rank");
  - a policy's command that does not interrupt waits for whatever flies (A-GRA's "nice" command);
  - otherwise (a policy's interrupting command, the platform's deferring one) the capability's precedence decides, then the rank: ahead of it takes, behind it waits. Equal, the newer command takes: a NEW always, as ADR-26 had it. What waited (scheduled, unassigned, enabled) does not take axes from a newer one.

  `CommandOptions::interrupt` is true by default, today's behaviour; A-GRA's omission means false, which `fsim.agra.command_options` maps. With every rank and precedence left as they are, every answer and every flight is as before.
- **Waiting (ACT-03, ACT-06, STS-14).** A NEW that may not take its axes, or whose start window has not opened, is accepted to wait: `kDeferred`, with `other` naming what it waits for. Its activity is pending, `waiting` `scheduled` or `queued` (and `waitingFor`), its basis `planned`. It is listed and addressed as what flies: an UPDATE changes what it will fly, checked as its NEW was, and a CANCEL ends it. It starts as soon as it may: after each world step, and after anything that frees axes or changes a precedence. Starts go in order of source, precedence, rank, then the order they began to wait, and a start that frees what an earlier one waited for lets that one start too. At its start it is prepared afresh from where the aircraft is then (an hsa's omitted fields, a route's plan, a behaviour's admission, a placard). A refusal there fails it, with that reason. At most 16 wait per vehicle; one more is refused `queue_full`. The existing entry points never wait: where they may not take the axes, they are refused. `fsim.agra.activity_state` names a queued activity whose start window is open ACTIVE_FULLY_CONSTRAINED.
- **Time windows (CMD-08).** `CommandOptions::window` gives a start window, an end window (simulation seconds; NaN for no bound) and which must be met (`TimeCriticality`). A window that cannot be met is refused `time_constraint`: an end already past, or a critical start already past. One out of order is refused `invalid_parameter`. The rest act after each world step:
  - it starts no earlier than its start window opens;
  - a critical start window closing while it waits fails it (`time_constraint`); a start window that is not critical lets it start late;
  - the end window closing while it waits fails it;
  - a persistent activity (a level, a hold) is done when its end window closes (`completed`, `goal_reached`);
  - a terminating one still flying then fails if its end is critical, and goes on late if not;
  - one done before a critical end window opens fails.

  A-GRA puts repetition on tasks, not flight commands: it comes with the flight tasks (FA-2d).
- **Overriding a rejection (CMD-09).** `overrideRejection` is carried and kept. The first soft rejection is endurance (FA-3e, 4.18); air traffic's comes in FA-15. None of the safety limits is ever overridden.

| C++ | C ABI 1.9 | Python |
| --- | --- | --- |
| `CommandOptions` gains `rank` (`Rank`), `interrupt`, `precedenceOverride`, `window` (`TimeWindow`, `TimeCriticality`), `overrideRejection`; `kDeferred` | `fsim_command_options` grows: `interrupt` (1 from init), `override_rejection`, `rank_priority`, `rank_precedence`, `precedence_override` (`FSIM_NO_PRECEDENCE_OVERRIDE`), the four window bounds (NaN from init), `criticality`; `FSIM_COMMAND_DEFERRED` | every `submit*` takes `rank=`, `interrupt=`, `precedence_override=`, `window=` (`fsim.TimeWindow` or a dict), `override_rejection=`; `Activity.deferred`, `Validation.deferred` |
| `ActivityRecord` gains `rank`, `precedence`, `interrupt`, `waiting` (`ActivityWait`), `waitingFor`, `window`, `basis()` | `fsim_activity_envelope` grows: `waiting`, `basis`, `rank_priority`, `rank_precedence`, `precedence`, `waiting_for`, `interrupt`, `criticality`, the window's bounds; `fsim_activity_wait_name`, `fsim_activity_basis_name`, `fsim_time_criticality_name` | `ActivityInfo` gains `waiting` (`fsim.ActivityWait`), `basis` (`fsim.ActivityBasis`), `rank` (`fsim.Rank`), `precedence`, `waiting_for`, `interrupt`, `window` |
| `World::setCapabilityPrecedence`, `capabilityPrecedence`; `Vehicle::` likewise | `fsim_vehicle_set_capability_precedence`, `fsim_vehicle_capability_precedence` | `vehicle.set_capability_precedence(capability, p)`, `capability_precedence(capability)`; `fsim.agra.command_options(ranking, temporal)`, `activity_basis(info)`, `activity_state(info, now)` |
| `Reason::TimeConstraint`, `QueueFull` | `"time_constraint"`, `"queue_full"` | `fsim.agra`: `CONSTRAINT_TIME`, `INSUFFICIENT_RESOURCES` |

### 4.10 Activity commands (as FA-2 builds it)

What may be done to an activity once its command is accepted (A-GRA's ActivityCommandBaseType: ChangeActivityState, DeleteActivity, ChangeActivityRank, UnassignActivity), natively: `World::activityCommand(caller, activity, command, rank)`.

- **Disable and enable (CMD-12, ACT-04).** A disabled activity is kept, `disabled` and live, and flies nothing; its axes go to the vehicle default or to what waits for them. Enabled, it waits to start again and starts when it may. A route resumes at the point it flew to; the others start afresh. Disable or enable of one already so is accepted, and changes nothing. Its end window still ends it (`time_constraint`).
- **Reset.** Over from its beginning: a flying activity's behaviour starts afresh from its setpoint (a route from its first point as commanded, wherever it resumed). One that waits, or is disabled, forgets where it would resume.
- **Delete (CMD-13).** A sticky disable: the activity ends `deleted` (reason `requested`) and nothing enables it again.
- **Change rank (CMD-14, ACT-07).** Its rank changes, and what it contests is arbitrated afresh at once: what waited may take its axes, or it theirs.
- **Unassign (CMD-15).** It gives up its axes and waits for them again, behind what waits: among equals, the order they began to wait decides, and an activity that waited never takes axes from a newer command of equal rank (4.9). So what waited for its axes gets them, and it waits until they are free.
- **Interactive (CMD-16).** An activity whose command said `interactive` false refuses all six (`not_interactive`); UPDATE and CANCEL, the command's own, it still takes.
- They answer as UPDATE does: the addressed activity's command id; `activity_ended`, `unknown_activity`, `authority_held` (under Granted, a caller below the activity's source); `queue_full` where a flying activity kept out of its slot finds no room (the 16 of 4.9). An activity kept out of its slot keeps its behaviour, its setpoint, and a route's waypoints or a curve's segments.

| C++ | C ABI 1.10 | Python |
| --- | --- | --- |
| `ActivityCommand` (`Disable`, `Enable`, `Reset`, `Delete`, `ChangeRank`, `Unassign`), `World::activityCommand(caller, activity, command, rank)`; `ActivityState::Disabled`, `Deleted`; `Reason::NotInteractive` | `fsim_activity_command(world, activity, command, rank_priority, rank_precedence, source, &result)`, `fsim_activity_command_name`; `FSIM_ACTIVITY_DISABLED`, `_DELETED` | `Activity.disable()`, `.enable()`, `.reset()`, `.delete()`, `.change_rank(rank)`, `.unassign()`; `fsim.ActivityState.DISABLED`, `.DELETED`; `fsim.agra.activity_state`: DISABLED, DELETED |

### 4.11 Flight tasks and suggestions (as FA-2 builds it)

A command kept by id and flown on a task command (A-GRA's MA_TaskMT with Flight, MA_TaskCommandMT, TaskStatus), and a best effort suggested in place of a command refused (VI 1.2.2.1). There is no workflow engine (section 11): a task is one command, flown as often as its repetition says.

- **Kept and flown (TSK-01).** `storeTask(vehicle, id, command, waypoints, segments, repetition)` keeps a flight or guidance command. It refuses:
  - id 0, or one with `kSuggestedTask` (the platform's own ids): `invalid_parameter`;
  - no runs, a negative interval, or runs of a capability that never completes: `invalid_parameter`;
  - a task whose activity is live: `task_active`;
  - a command the vehicle cannot fly: the reason why, as a NEW's.

  `commandTask(vehicle, id, options)` is the NEW of its command, with `{task, id}` among the requirements it traces to, answered as that NEW is; a task not kept is `unknown_task`, one flying `task_active`. `cancelTask` cancels its activity; `removeTask` forgets it.
- **Its runs (A-GRA's finite repetition, which A-GRA puts on tasks).** `TaskRepetition {attempts, intervalS}`: each run begins `intervalS` after the one before completes (NaN: at once). The runs are one activity's (`ActivityRecord::run`, `runs`). Between runs it stays active and flies what its behaviour flies when done (a route's end, a manoeuvre's recovery), then begins afresh (a route from its first point); the activity completes after the last run. No run allocates anything in the step.
- **Its status (TSK-02).** `taskStatus` gives A-GRA's execution state:
  - `awaiting_execution` while it is kept and not commanded;
  - `execution_pending` while its activity waits, has not flown yet, or is disabled; `executing` while it flies;
  - `completed`, or `failed`, as its activity ended;
  - `dropped` if its activity lost its axes or its authority (preempted, revoked, released, not granted, collision avoidance, restricted), `canceled` if canceled or deleted on request;

  with the reason, its activity, the run of how many, the percent of the whole, its start and end, and its task command's id.
- **Suggestions (VAL-10).** A NEW refused under `RangePolicy::Reject` only for values Clamp would hold (a value beyond a limit, a turn flown smaller, a climb flown at the aircraft's rate) names a suggestion: a task the platform keeps with the command its checks left, every value held to the aircraft's limits (a route's points as held). It is in `CommandDetails::suggestion`, the C ABI's detail, and `fsim.Rejected.suggestion`, and a task command flies it. A finding no clamp mends (a curve section too tight, a manoeuvre's entry) suggests nothing; nor does a validation. Sixteen suggestions are kept, the oldest not flying making room.
- **A failed activity names one (VAL-11).** A waiting activity refused as it would start, for what Clamp would fly, fails, and its record's `suggestion` names the task kept in its place. Nothing is allocated in the step: the waiting entry keeps the suggestion until the next call that may allocate makes it a task.

| C++ | C ABI 1.11 | Python |
| --- | --- | --- |
| `World::storeTask`, `commandTask`, `cancelTask`, `removeTask`, `taskStatus`, `tasks`; `Vehicle::` likewise; `TaskId`, `kSuggestedTask`, `TaskRepetition`, `TaskState`, `TaskStatus`; `Reason::UnknownTask`, `TaskActive` | `fsim_vehicle_store_task` (the command as an `fsim_batch_command`), `_command_task`, `_cancel_task`, `_remove_task`, `_task_status`, `_task_count`, `_task_at`; `fsim_task_status`, `fsim_task_state_name`, `FSIM_SUGGESTED_TASK` | `vehicle.store_task(id, fsim.BatchCommand(...), attempts, interval_s)`, `command_task`, `cancel_task`, `remove_task`, `task_status`, `tasks`; `fsim.TaskState`, `fsim.TaskStatus`; `fsim.agra.task_state` |
| `CommandDetails::suggestion`; `ActivityRecord::suggestion`, `run`, `runs` | `fsim_command_detail.suggestion`; `fsim_activity_envelope.suggestion`, `run`, `runs` | `fsim.Rejected.suggestion`; `ActivityInfo.suggestion`, `.run`, `.runs` |

### 4.12 Reports and named controllers (as FA-2 builds it)

What an activity flies and where to, read back as A-GRA's activity report carries them (VI 1.2.3.1; MA_CurveFollowingControlType, MA_EndPointType, MA_VehicleCommandStateType), and the several mission autonomy services that may hold control of one vehicle (SystemServiceType).

- **Its setpoint read back (ACT-15).** `activitySetpoint(activity)` gives what a live activity flies now:
  - a mode's setpoint as its NEW completed it and its UPDATEs merged (an hsa's heading, speed and altitude, with their references), a level's, a support command's, a behaviour's;
  - a route's waypoints as completed (a field left out filled from the point before);
  - a curve's segments, the appended ones too.

  A waiting activity's is its command as given: it is completed as it starts, from where the aircraft is then. None once it has ended.
- **Its flyout curve (CRV-14).** A curve's setpoint is its flyout curve: the reference its segments are from (fixed at its NEW; where the aircraft was, if left out) and every segment it flies. `fsim.agra.flyout_curve` gives A-GRA's form: one MA_NURBS_PointType per segment, its six control points weighted 1, the knots [0 x6, 1 x6].
- **Its end points (ACT-10).** `endPoints(activity, max)` says where it flies to, from the point it flies to now, `max` at most:
  - a route's waypoints, from the one its progress names (a waiting route's: from its start). Each is a turn point (A-GRA's TurnPoint: fly-by TURN_SHORT, or FLY_OVER) but the last, a waypoint - or a loiter point where the route loiters after it. A repeating route's go round again;
  - a curve's segment ends, offset from its reference; the last a loiter point where it loiters. None for a waiting curve whose reference is left out: where it will start is not known yet;
  - a pattern's centre or fix, a loiter point; the position level's point, a waypoint;
  - none for an hsa, a behaviour, or a route or curve past its end that flies on.
- **What the vehicle is commanded (ACT-13).** `commandState(vehicle)` is A-GRA's VehicleCommandState: the cascade's levels, as before (CommandedState), and:
  - the acceleration commanded, north, east and down. A wing's is its longitudinal acceleration along its flight path and its load factor's lift normal to the path (JSBSim's lift load factor, in the plane of symmetry), with gravity's pull, at the attitude, angle of attack and sideslip it flies. In the F-16C's 60-degree turn it is the acceleration flown to within 1 m/s^2 on each axis once rolled in (test_reports). NaN where no longitudinal acceleration is commanded (a throttle given instead), and for a rotorcraft, whose thrust is not its acceleration (its drag is not modelled here);
  - the altitude as commanded, in its reference: a live hsa's or pattern's, a route's point flown to's; a curve's (as its progress has it) and the position level's above sea level.
- **Named controllers (AUT-06).** A vehicle's policy may be several services, each named by a `ControllerId` (0, the default policy; A-GRA's SystemServiceType). A NEW's `CommandOptions::controller` names its controller, kept with its activity (`ActivityRecord::controller`). UPDATE, CANCEL and the activity commands declare a `Caller {source, controller}`; a `Source` alone is the default policy's.
  - Under `ControlMode::Granted` a grant is one controller's (`ControlStatus::holder`). A request while another controller holds it is refused `authority_held`; a release by one that does not hold it is refused `not_granted`, and nothing changes. A release ends the releasing controller's activities of the capability.
  - Only the holder's NEW flies the capability (another's is refused `not_granted`), and a controller may not address another's activity (`authority_held`, naming it), as a lower source may not address a higher one's.
  - Switching to Granted ends what flies without its own controller's grant. The platform's revocation, and a capability no longer allowed, end every controller's.
  - Under `ControlMode::Open`, as ADR-26: any controller commands and addresses any activity. Arbitration is by source, precedence and rank, never by controller.
  - Controllers, like sources, are declared, not authenticated.

| C++ | C ABI 1.12 | Python |
| --- | --- | --- |
| `World::activitySetpoint`, `endPoints`, `commandState`; `Setpoint`, `EndPoint`, `EndPointKind`, `VehicleCommandState`; `Vehicle::commanded()`, a `VehicleCommandState` now | `fsim_activity_get_setpoint` (the `fsim_batch_command` that would command it, its arrays the library's), `fsim_activity_end_points`, `fsim_end_point`, `fsim_end_point_kind_name`; `fsim_commanded_state` grown (`north_acceleration_ms2`, `east_`, `down_`, `altitude_m`, `altitude_reference`) | `activity.setpoint()` (a `fsim.BatchCommand`), `activity.end_points(max)`, `fsim.EndPoint`, `fsim.EndPointKind`; `vehicle.commanded`'s new fields; `fsim.agra.flyout_curve`, `end_point`, `altitude_reference` |
| `ControllerId`, `Caller`; `CommandOptions::controller`, `ActivityRecord::controller`, `ControlStatus::holder`; `requestControl` and `releaseControl` with a controller | `fsim_command_options.controller` (1.9's reserved word); `fsim_activity_update_by`, `_cancel_by`, `_update_route_by`, `_update_curve_by`; a controller on `fsim_activity_command` and `fsim_vehicle_cancel_task`; `fsim_vehicle_request_control_by`, `_release_control_by`, `fsim_vehicle_control_holder`; `fsim_activity_envelope.controller` | `controller=` on every submit and `command_task`; `Activity.controller`; `request_control(capability, controller)`, `release_control(capability, controller)`; `fsim.ControlStatus.holder`; `ActivityInfo.controller` |

### 4.13 The performance tables (as FA-3 builds them)

The performance tables (SUB-02) record what an aircraft flies level, climbs and descends at, and the fuel it burns, against altitude, weight and speed. FA-3's performance profile, energy management, speed optimisation and endurance all stand on them.

- **Flown by hangar.** Its `performance` stage ([hangar.md](hangar.md), "Performance and fuel") flies every fixed-wing design in the platform's own JSBSim, through its velocity loop with the envelope protection off:
  - It flies at seven altitudes: 0, 0.2, 0.4, 0.6, 0.75, 0.85 and 0.95 of the service ceiling its flight tests found, the lowest at 100 m, plus the height the design publishes its top speed at. It flies three weights, the tanks a tenth, half and wholly full of their capacity, plus the weight the aircraft file starts at where that is none of them (the B-52H's 40 %). An aircraft without fuel flies one weight.
  - **Full power, level:** the excess power at each speed, the top level speed and the best climb; the run's first 8 s, the engines spooling up, are left out.
  - **Short runs where that run did not fly.** Each is flown as the flight tests fly their ceiling runs: 10 s level at its speed, then 20 s at full power, counted only where it held its height at 1 g. They go where the long run left gaps:
    - below a retry's start: near its ceiling an aircraft cannot fly level at 1.2 times the stall, and the retry begins at 1.8;
    - for a supersonic design, past a drag rise the run could not accelerate through: loaded at 14 km, the F-35A stops at Mach 1.16 yet holds Mach 1.45 once there.
  - **Idle, level, from the top:** the excess power at idle (the descent rate and the deceleration) and the stall, as the flight tests read it. Each weight's stall comes from the highest lift coefficient any weight's run reached at that altitude.
  - **Level at sixteen speeds:** the fuel flow, and from it the best-endurance and best-range speeds. Each point counts only where it held its speed within 2 % and its height within 1 m/s. The band runs from the slowest speed full power flew with margin, never below the envelope's least speed, to 97 % of the top. A band narrower than 5 % is none: at its ceiling.

  Each condition flies at its weight: the fuel is frozen except in the level points' last ten seconds. The vertical speed asked of the loop follows the height, since the loop holds a vertical speed, not a height. A fighter accelerating through Mach 1 at 100 m sank into the ground. Any run that comes within 10 m of the ground is refused: a fighter's gear is up, and on its belly its wheels report nothing.
- **Checked against the flight tests** within 5 %, read as the platform reads the tables, at the same altitude and at the weight the aircraft spawns at, as the tests fly. Four items are checked:
  - the top level speeds and the best climb; a fighter's top Mach number and best climb are read at the weight its run had burned down to (up to six minutes at full afterburner), found by flying its runs again;
  - the stall;
  - the service ceiling, for a straight wing against its flight test's two highest climbs extended;
  - the climb, against a climb flown again at a held rate, since the flight test's airspeed hold counts speed bled off as climb.

  The tables are also checked against themselves where no test reaches: at every altitude flown at every weight, the lighter aircraft climbs better and flies higher. Section 14 has the measurements.
- **Carried in the profile.** The `tables` section ([control-architecture.md](control-architecture.md), 7.2) carries:
  - the axes;
  - per condition: the least and top level speeds; the top a level acceleration reaches, short of the top past a drag rise it cannot pass; the stall; the best-endurance and best-range speeds with their fuel flows; the best climb and its speed;
  - per level point: its fuel flow (where it held) and the excess power at full power and at idle;
  - the fuel capacity.

  A stock JSBSim aircraft has none.
- **Looked up** (`tablesAt`, `tablesCeilingM`, fsim/VehicleProfile.h). Values are linear in altitude and weight between the conditions flown, and along each condition's band at the same fraction of it. At a speed (FA-3e), each altitude row is read at the same equivalent airspeed, where the drag changes little with height: at 3,000 m, between the Su-27S's rows at 100 m and 4,000 m, the same fraction had read its burn 12 % high. A condition or point is read at its own value, whatever its neighbour's. Below the lowest altitude a lookup takes that row's values, and in weight it runs on down to the tanks empty. A lookup is NaN above the altitudes flown, or where a condition it lies between was not flown (above that weight's ceiling). The ceiling is where the best climb first falls below 0.5 m/s; an altitude nothing held level at climbs nothing, as the flight tests count it. Still climbing at the highest row, the highest two's line is extended, but no further than as high again: the fixed wings' reached at most 42 % above their rows, a rotorcraft's tens of times. An axis that is not strictly rising is refused.
- **Energy: fuel or a battery** (FA-3b). An aircraft that flies on a battery has the power its battery gives where one that burns fuel has its fuel flow (`powerW`, `bestEndurancePowerW`, `bestRangePowerW`, `batteryCapacityJ`). Its best speeds are the least power's and the most distance per joule's. The Skua's band begins at its least speed, 1.15 times the stall: an electric motor gives full power at once, and the full-power run is past its slow speeds within the second it is recorded from.
- **The rotorcraft** (FA-3b; [hangar.md](hangar.md), "Rotorcraft") fly theirs with the fly stage's hold:
  - four rows, 100 m to 3,000 m (their models' power does not fall with the air's density: no ceiling within reach);
  - a helicopter's three weights, a multirotor's one;
  - level from the hover to 97 % of the top level speed, the top found within the power and the tilt the platform's velocity loop flies within (FA-3e: the multirotors had flown to the design's whole pitch limit, to speeds the platform does not fly);
  - their speeds along the nose, as their velocity loop flies an airspeed (a multirotor flies nose down: its true airspeed is up to 12 % more);
  - a helicopter's full-power climb at each speed;
  - no stall, no idle, and no multirotor climb (their thrust ignores a climb's inflow).

  They are checked against the fly stage's hover and trims, and against a burn flown at the best-endurance speed.

### 4.14 The navigation report (as FA-3 builds it)

The navigation report (STS-07; A-GRA's MA_NavigationReport) says what a vehicle flies on, how much it has left and for how long, what it can spend before it turns back, and its contingency level.

- **Energy on board, all 35 aircraft.**
  - The fixed wings burn fuel from their JSBSim tanks, and so do the UH-60A and UH-1H: their engines are JSBSim's electric engine, patched to burn their specific fuel consumption times the power they give.
  - The IRIS+, the Crazyflie and the Skua carry batteries. Their flight controls integrate the power the motors draw into the charge left (`fsim/battery/*`), and the motors stop once it is spent.
  - A stock JSBSim aircraft reports its tanks as they are. One with neither tanks nor a battery, such as JSBSim's gliders, reports `Energy::Unknown` and nothing more.
- **What it reports.**
  - The fuel (A-GRA's Fuel), what is left and the capacity: kilograms for fuel, joules for a battery. The percent left (Percent).
  - The consumption now: the engines' fuel flow, or the power the battery gives.
  - The endurance (Duration): what is left over the consumption now, as A-GRA defines it. It is infinite while the vehicle consumes nothing and 0 once nothing is left.
  - The contingency level: FLIGHT_CRITICAL at or below the reserve (a tenth of capacity unless set) or with the engines starved, otherwise NORMAL. The platform models no subsystem failures and no communications, so it never reports MISSION_CRITICAL or LOST_COMMS.
  - With a recovery point set (`setNavigation`), the playtime (Playtime): what is left less the reserve and the return, over the consumption now; 0 once past it. The return is flown at the best-range speed and its consumption (fuel flow, or a battery's power) from the performance tables (4.13), at the vehicle's altitude and weight. Without tables (a stock aircraft, or above the altitudes flown) it is flown at the cruise speed and the consumption now. The distance is the great circle over the ground; the wind, the climb and the descent are not counted.
- **Asked for, never stepped.** The report is worked out when asked, from the flight model's tanks, engines and battery. Nothing in the step reads it, so no flight changes.
- **Checked on all 35** (section 14):
  - the consumption it gives, summed step by step, is what left the tanks or the battery;
  - flown level at cruise (the rotorcraft in the hover), the time to use a fifth of what the vehicle has, or two hours' worth, is 0 to 4.1 % longer than the report said. The difference is the weight the vehicle loses as it burns; a battery's weight does not change, and it matches.
- **Surfaces.**
  - C++: `World::navigationReport`, `setNavigation` and `navigation` (session); `Vehicle::navigationReport()`, `setNavigation()` and `navigation()` (fsim/World.h), with the types in fsim/Control.h.
  - C: `fsim_vehicle_navigation_report`, `fsim_vehicle_set_navigation` and `fsim_vehicle_get_navigation` (ABI 1.13).
  - Python: `Vehicle.navigation_report()`, `set_recovery()`, `clear_recovery()` and `navigation()`; `fsim.agra.navigation_report(report)` gives A-GRA's names.

### 4.15 The performance profile (as FA-3 builds it)

A flight mode's performance profile (CAP-04 to CAP-15; A-GRA's MA_FlightControlModesPerformanceProfileType, VI 1.2.6.7) is the set of guard rails a mission autonomy shapes its commands within.

- **Per mode, worked out when asked.**
  - A-GRA profiles three modes: HSA/CSA, waypoint following and curve following. The platform gives each one the vehicle offers.
  - The modes share the vehicle's performance ([vehicle-interface.md](vehicle-interface.md), 7.1), so the three profiles carry the same values, each under its own mode.
  - Another mode is refused `invalid_parameter`. One the vehicle does not offer is refused with its support table's reason, `not_supported` or `not_implemented`.
  - The profile is worked out at the vehicle's condition now: its altitude, weight and airspeed, its flaps and gear. Nothing in the step reads it.
- **What it carries**, under A-GRA's fields:
  - *Airspeeds against altitude* (true airspeeds, at the weight now; CAP-05). The least and the most are what the aircraft flies within what it may:
    - it flies from the tables' least to their top level speed, a rotorcraft's from the hover;
    - it may fly within the envelope's calibrated and Mach limits and the gear's placard.

    With them come the best-endurance and best-range speeds.
  - *Altitudes* (CAP-06): the most is the service ceiling at the weight now. There is no least: the platform keeps no floor but the ground.
  - *Against airspeed and altitude*, at the weight now:
    - acceleration limits in body axes (CAP-07): x from the excess power at full power and at idle over the speed, z from the load factors; a rotorcraft's from its tilt and its loops' deceleration;
    - specific excess power (CAP-08): the climb holding the speed, and the acceleration holding the height;
    - the steepest descent holding the speed at idle, no faster than guidance asks (CAP-12);
    - the deceleration at idle (CAP-13), clean: the tables fly no drag devices;
    - the burn (CAP-14): the fuel flow, or a battery's power.
  - *At the condition now*:
    - the attitude limits (CAP-09): pitch up and down, and bank; no yaw limit;
    - the rate limits (CAP-10): roll, where the aircraft has one;
    - the fastest turn guidance flies at the airspeed now (CAP-11), and the fastest climb it asks.
- **Updated with the configuration and the condition** (CAP-15). Asked again, the profile is worked out afresh.
  - The flaps count as out when commanded beyond their threshold, as protection judges them; the gear counts as down when not fully up.
  - Then what the tables give is left out, since they are flown clean, and the flaps' and gear's placards bound the airspeeds.
  - The weight is the vehicle's now; the altitude limits and attitudes are the condition's now.
- **Where the tables are silent, it says nothing.**
  - A stock JSBSim aircraft's airspeeds come from its Performance at the altitude now, and it has no excess power and no burn.
  - A quadrotor has no climb.
  - A rotorcraft has no ceiling within the altitudes flown.
- **A rotorcraft's airspeeds are along its nose**, as its loops fly an airspeed and its tables have it (4.13).
- **A-GRA's schema has two gaps**, which the platform fills (`fsim.agra.performance_profile`):
  - MA_SpeedType, used by MaxDescentRate and ExcessPowerMaxClimb, carries an airspeed, an altitude and a weight, but no speed;
  - the VI names a maximum climb rate, which the type has no field for.
- **Checked on all 35**, through the SDK (section 14). At a weight the tables flew, each of the profile's points is the table's cell, to the six digits the aircraft file keeps. FA-3a checked those tables against the flight tests, within 5 %.
- **Surfaces.**
  - C++: `World::performanceProfile(id, mode, out)` and `Vehicle::performanceProfile(mode, out)`, with the types in fsim/PerformanceProfile.h. A profile asked again into the same one allocates nothing.
  - C: `fsim_vehicle_performance_profile` (ABI 1.14).
  - Python: `Vehicle.performance_profile(mode)`; `fsim.agra.performance_profile(profile, capacity=None)` gives A-GRA's names.

### 4.16 Energy management (as FA-3 builds it)

Energy management (HSA-10, CTG-04) keeps a wing from trading its airspeed for height. Asked to climb faster than its power allows, it climbs at what its airspeed's margin over its least airspeed affords, and the height waits.

- **The rule**, on a wing's vertical speed, at its calibrated airspeed now:
  - above 1.1 times its least airspeed, a climb of at most 0.5 m/s for each m/s above that;
  - between its least and 1.1 times it, no climb: the height is held;
  - below its least, a descent of 0.5 m/s for each m/s short.

  A climb asked for more than the power gives settles where the power holds the climb that the margin affords. The C172 at full power climbs at 0.8 m/s at 1.16 times its least. The 1.1 is FA-3's criterion.
- **Where it acts.** It is envelope protection's, in `Limit`, the default with an envelope. It limits the vertical speed at the velocity level, as the flight path the pitch limits allow already does ([control-architecture.md](control-architecture.md), 11.3). So:
  - every mode a wing flies through its velocity level has it: HSA and hold, routes, patterns and curves, pursuit, evasion and formation, and a velocity command itself;
  - an attitude, acceleration or actuator command is flown as given, within the attitude limits;
  - a rotorcraft has no least airspeed, so no rule: its climb costs no airspeed;
  - hangar's flight tests and performance tables fly with protection off, as before.
- **Reported as the least airspeed's limit** (HSA-10's "performance limited"). While the rule holds a climb back:
  - the envelope report counts the update under `cas_min`;
  - the activity is flagged demand-limited: A-GRA's ACTIVE_PARTIALLY_CONSTRAINED, or ACTIVE_FULLY_CONSTRAINED where its throttle is also at its stop (the C172's climb).
- **A least airspeed on all 31 wings.**
  - The 15 wings without a limiting law have the stall their flight tests flew.
  - The 16 fly-by-wire designs had none (CAP-05's probe), since their law will not let a stall be flown. hangar now writes theirs from the performance tables (`profile.py`, `spawn_stall`): the tables' stall (idle, the height held, to the law's angle of attack) at the lowest altitude, at the weight the aircraft spawns at.
  - With it they are treated as the other wings are:
    - protection holds their airspeed setpoints at their least or above;
    - a command's speed below it is clamped or rejected (`min_airspeed`), and the profile's least airspeed is never below it (4.15);
    - a loop needs twice it to be entered, and one flown below it is given up (section 6: the fighters' least airspeed was to come with FA-3's tables).
- **Checked** on all 31 wings by the fleet climb case, and the changed flights measured and listed (section 14).

### 4.17 Speed optimisation (as FA-3 builds it)

A speed optimisation (HSA-05, LTR-17; A-GRA's SpeedOptimizationEnum, PathSegmentSpeedType.SpeedOptimization) is the speed a mode varies by itself: LONG_RANGE_CRUISE, the most fuel-efficient, and MAX_ENDURANCE, the most time in the air. The platform flies the performance tables' best-range speed (the most distance per kilogram, or per joule) and best-endurance speed (the least fuel flow, or power) (4.13).

- **Where.** `fsim.guidance.hsa` and `fsim.guidance.pattern` take `speedOptimization`, their setpoints' last field. A speed replaces an optimisation, and an optimisation a speed. A route's, per segment (WPT-06), comes with FA-6.
- **How it is flown.**
  - The host resolves it as a speed, as the command is given: the optimum's true airspeed at the altitude flown to. The checks judge it, and a pattern's radius and legs are planned with it.
  - The mode flies the optimum afresh each update, at the altitude and the weight now: the fuel on board on the tables' weight with the tanks empty. It follows as the aircraft climbs and burns fuel. Above the altitudes the tables fly, it holds the speed resolved.
  - A rotorcraft's is along its nose, as its loops fly an airspeed and its tables have it.
- **Reported.** The setpoint keeps the optimisation and the speed resolved. The progress gives the speed flown now, a true airspeed.
- **Without tables** - a stock JSBSim aircraft, which hangar has flown none for - it is not implemented. Its support table says so (the tables are FA-3's), and a command asking for one is refused `not_implemented`, the field named.
- **Two things it found, fixed** (section 14):
  - The multirotors' tables had been flown to their design's whole pitch limit (35° and 30°), past the 28° and 24° their velocity loop flies within. Their best range lay at the top of speeds the platform does not fly. hangar now flies every rotorcraft's tables within its loop's tilt ([hangar.md](hangar.md)).
  - The rotor velocity loop's integral trimmed half its tilt, so a speed whose drag needs more was flown short: the Crazyflie's best range needs 22 of its 24°, and was flown 15 % short along the nose. The integral now trims the whole tilt forward and half sideways, as an orbit needs ([rotorcraft.md](rotorcraft.md)).
- **Surfaces.**
  - C++: `HsaCommand::speedOptimization`, `PatternCommand::speedOptimization`, `SpeedOptimization`; `optimalTasMs` (fsim/GuidanceModes.h); `ControlContext::tables`.
  - C ABI 1.15: an hsa's seventh field and a pattern's thirteenth, `fsim_speed_optimization`.
  - Python: `speed_optimization=` ("long_range_cruise", "max_endurance"), `fsim.SpeedOptimization`, `fsim.agra.SPEED_OPTIMIZATION`.

### 4.18 Endurance validation (as FA-3 builds it)

A flight command whose flight has an end is checked at its NEW against the vehicle's endurance (VAL-03; A-GRA's VIOLATION_ENDURANCE with MA_InsufficientEnduranceType): it needs no more fuel, or battery charge, than the vehicle has above its reserve.

- **Which flights.** A route that does not repeat, to its last point; a pattern with a duration; a curve, to its end. An hsa, a repeating route or an untimed pattern has no end, and nothing is judged.
- **What it needs** (a minimal model):
  - each leg flown level at its speed and altitude - a route's from where the aircraft is, with the turns between them;
  - the performance tables' burn there (at the same equivalent airspeed between their rows, 4.13), at the weight it will have by then, a minute at a time;
  - where the tables are silent (a stock aircraft), what it consumes now.

  Climbs, descents and wind are not counted.
- **What it has:** its fuel, or its battery's charge, above its reserve (the navigation settings', 4.14; a tenth of its capacity unless set).
- **Refused `insufficient_endurance`** (A-GRA's CONSTRAINT_ENDURANCE and VIOLATION_ENDURANCE) where it needs more. This is the first **soft rejection**:
  - `overrideRejection` flies it anyway, flagged `kOverridden`;
  - the details carry both sides (`CommandDetails::endurance`): what it has and what it needs, in kg or J, and how long each lasts - at what it consumes now, and the flight's own time;
  - a validation answers the same.
- **Checked** on all 35 against a flown burn (section 14).
- **Surfaces.**
  - C++: `Reason::InsufficientEndurance`, `kOverridden`, `CommandDetails::Endurance`.
  - C ABI 1.16: `fsim_command_endurance`, `fsim_last_command_endurance`, `FSIM_COMMAND_OVERRIDDEN`.
  - Python: `fsim.Endurance` on `fsim.Rejected`, on a `Validation`, and on an overridden `Activity` (`overridden`); `fsim.agra.insufficient_endurance`.

### 4.19 The terrain (as FA-4 builds it)

A flight command's path is checked against the ground (VAL-06; A-GRA's VIOLATION_TERRAIN with its TerrainConstraint), and the ground can be asked for (STS-11, ENV-01; VI 1.2.6.9's elevation request).

- **The ground** is the physics' own: the world's `GroundProvider` - flat at sea level unless set, the public elevation tiles with `terrain`, or the caller's own. What a path is checked against is what the aircraft would meet.
- **The query:** the height above the WGS-84 ellipsoid at a place, or nothing where the provider has no data (a terrain tile it cannot load, which the physics reads as sea level). A provider tells the two apart through `knownHeightAboveEllipsoidM`.
- **Which paths** (a minimal model):
  - a route: each leg from where the aircraft is, its altitude in its own reference as the route flies it - straight to its point, or at its climb rate and then level - with its fly-by turns. A descent too steep to fly as asked is clamped to the aircraft's steepest, and goes on down past its point. A route that repeats is checked once round again; one that does not, for what it flies after its last point: until its altitude has settled, and then a minute on (a lap, round its point);
  - a pattern: its lap at its altitude, and a racetrack's or a hold's entry to its fix;
  - a curve: its segments, then a minute on along its last course (a lap round its end);
  - an hsa: its line ahead, level at its altitude, for a minute at its speed.

  A path above the ground follows it. The climb or descent to a pattern's or an hsa's altitude, and the wind, are not modelled.
- **Sampled** at the ground's own spacing: a tile's pixel (38 m at zoom 12), 30 m for a provider that does not say, and the ends alone for flat ground. A curve's segment is sampled 32 times at least, since its height is not straight. Where a sample is below the ground, the first place the path goes below is found between it and the sample before, to a tenth of a metre.
- **Refused `terrain_conflict`** where the path goes below the ground: at a NEW, a validation or an UPDATE that is checked, whatever the range policy. It is never overridden.
  - The details (`CommandDetails::terrain`) name the place, the path's altitude and the ground's there, when it would be there (seconds from the command, at its planned speeds), and the route point it flies to there or the curve segment.
  - In A-GRA's terms: the validation result VIOLATION_TERRAIN, the cannot-comply CONSTRAINT_SAFETY.
- **Surfaces.**
  - C++: `Reason::TerrainConflict`, `CommandDetails::Terrain`, `World::terrainHeightM`; `GroundProvider::knownHeightAboveEllipsoidM` and `resolutionM`.
  - C ABI 1.17: `fsim_command_terrain`, `fsim_last_command_terrain`, `fsim_world_terrain`.
  - Python: `World.terrain`, `fsim.TerrainPoint` on `fsim.Rejected` and on a `Validation`; `fsim.agra.terrain_constraint`, `fsim.agra.elevation_request_status`.

### 4.20 The barometric altimeter (as FA-4 builds it)

Each vehicle has a barometric altimeter (ENV-03), set to a QNH (STS-10; VI 1.2.6.5). It reports what it reads (STS-04; A-GRA's IndicatedBaroAltitude, BarometricAltitudeRate and Kollsman), and hsa and patterns fly a barometric altitude on it (HSA-07, LTR-16).

- **The air** is the world's: the 1976 standard atmosphere's layers with a uniform temperature bias and the sea-level pressure its environment sets. This is JSBSim's standard atmosphere, which the flight models fly in. The platform computes it itself, and matches JSBSim's pressure to 2e-9 of it and its temperature to 1e-7 K: JSBSim keeps its layers' bases in feet and their temperatures in Rankine.
- **The altimeter** reads the ICAO standard atmosphere's height of the static pressure above the pressure it is set to. That is how an altimeter's subscale works: set 13.25 hPa low (1000 hPa), it reads 110.9 m low at every height. It reads geopotential metres, as the standard atmosphere is laid out in them.
- **The setting.** A QNH from 850 to 1,100 hPa is applied at once; outside it, the request fails with `out_of_range` and nothing changes. Until set, it is the standard 1013.25 hPa, and the altimeter reads the pressure altitude.
- **A barometric altitude** in an hsa or a pattern is flown on the isobar the altimeter reads it on. As the air and the setting change, the isobar moves, and the mode follows it (15 K warm, 2,000 m on the altimeter is 102 m higher). The host checks it as the altimeter would read the ceiling. A route's barometric altitude is FA-6's (WPT-12) and is refused `not_implemented`, as its support table says.
- **The state data** (`StateData`, beyond the vehicle's state): the reading, its rate (the ratio of the air's temperature to the standard atmosphere's at that pressure, climbing), the Kollsman in hPa, and the air's static pressure and temperature.
- **Surfaces.**
  - C++: `fsim/Altimeter.h` (`Air`, `Altimeter`, the air's pressure and temperature, the altimeter's reading and its isobar); `AltitudeReference::Barometric`; `World::setQnh`, `qnh`, `stateData` (`Vehicle::setQnh`, `qnh`, `stateData`); `StateData`.
  - C ABI 1.18: `FSIM_ALTITUDE_BAROMETRIC`, `fsim_vehicle_set_qnh`, `fsim_vehicle_qnh`, `fsim_state_data`, `fsim_vehicle_state_data`.
  - Python: `fsim.AltitudeReference.BAROMETRIC`, `Vehicle.set_qnh`, `Vehicle.qnh`, `Vehicle.state_data` (`fsim.StateData`); `fsim.agra.apply_qnh_setting`, `fsim.agra.air_data`.

### 4.21 The state data and reference frames (as FA-4 builds them)

The state data carries what A-GRA's detailed position report and the vehicle's weather observation hold beyond the vehicle's state (STS-02, STS-06; VI 1.2.6.8). Reference frames (ENV-04) are by id.

- **Orientation rates and accelerations** (A-GRA's OrientationRate and OrientationAcceleration): how fast the Euler angles change - yaw, pitch and roll over the local north, east and down - and how that changes. The rates come from the body's rates through the kinematic equations; the accelerations from the body's angular accelerations, the flight model's. Pitched within 0.06° of straight up or down, where yaw and roll are one, they are NaN.
- **The wander angle** is 0: the platform's navigation frame is north's.
- **The wind** where the vehicle is, as its air data measures it: its velocity over the ground less its velocity through the air (its true airspeed along its angle of attack and sideslip, turned from its body's axes to north, east and down). It is A-GRA's WindData, whose source is Other: the vehicle.
- **Reference frames** (A-GRA's ReferenceFrame) have an id and an origin: fixed at a place and orientation, moving from a place at a constant velocity from a time, or following a vehicle.
  - A point in one (A-GRA's relative point) is offsets turned as A-GRA's RotationEnum says - unrotated (x north, y east, z down), by the origin's yaw, by its body's axes, or by its track - and laid out as its OffsetXY_Enum says: in the plane square to the vertical at the origin, along a great circle, or along a rhumb line, on the platform's sphere.
  - At a time: a moving origin is carried on at its velocity; a vehicle's from its state now, carried on at its velocity. A frame whose vehicle is gone answers nothing.
  - The modes' relative points (WPT-22, CRV-05, LTR-18) use them in FA-5 and FA-6.
- **Surfaces.**
  - C++: `StateData`'s orientation rates and accelerations, `wanderAngleRad`, wind; `fsim/Frames.h` (`FrameSpec`, `FrameOffset`, `FramePose`, `GeoPoint`; `framePose`, `framePoint`, `carried`); `World::createFrame`, `removeFrame`, `frame`, `framePose`, `framePoint`.
  - C ABI 1.19: `fsim_state_data`'s new fields; `fsim_frame_spec`, `fsim_frame_offset`, `fsim_world_create_frame`, `fsim_world_remove_frame`, `fsim_world_frame_point`.
  - Python: `fsim.StateData`'s new fields; `fsim.FrameOrigin`, `fsim.FrameRotation`, `fsim.FrameOffsets`; `World.create_frame`, `remove_frame`, `frame_point`; `fsim.agra.orientation_rate`, `orientation_acceleration`, `wind_data`.

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
- **Aerobatics (PLT-07):** offered by R10 (FA-1b). A checked NEW is refused `performance_limit` too slow where the least airspeed is known (the A-10C and Su-25; the fighters' comes with FA-3's performance tables) or a split-S too low. In flight it gives up below its least airspeed, past its angle of attack by more than 3° (a departure) or within 150 m of the ground, and completes only if flown within the envelope to its limiters' tolerance (0.5 g, 3° of angle of attack, 3 to 5 m/s; bank and pitch, which a loop passes by design, are not judged). The probe's loop from cruise: 14 of the 17 complete within the envelope; the A-10C, EA-18G and Mirage 2000 give up (they ran out of airspeed or departed over the top), where before every one reported `goal_reached`. None reaches the ground. With FA-3d the fighters' least airspeeds are known (4.16). The fleet test's loops from cruise: 7 of the 17 complete; 9 give up, 7 of them fighters that float over the top below their least airspeed; the Mirage 2000 is refused for its entry speed (section 14).
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

**Status:** done 2026-09-27 in five steps, each measured in section 14:
- FA-2a, the command envelope (4.8);
- FA-2b, ranks, queues and time windows (4.9);
- FA-2c, the activity commands (4.10);
- FA-2d, flight tasks and suggestions (4.11);
- FA-2e, named controllers and the reports (4.12; AUT-06 moved from FA-2d so that each step stays one reviewable change), with the fleet cases.

**Items (36):** CMD-02, CMD-03, CMD-05, CMD-06, CMD-07, CMD-08, CMD-09, CMD-10, CMD-12, CMD-13, CMD-14, CMD-15, CMD-16, CMD-18, CMD-19, CMD-20; WPT-24; CRV-14; VAL-01, VAL-02, VAL-08, VAL-10, VAL-11, VAL-12; ACT-03, ACT-04, ACT-06, ACT-07, ACT-08, ACT-10, ACT-13, ACT-15; AUT-06; STS-14; TSK-01, TSK-02.

**Accepted when:**

- The random-sequence conformance test extended with ranks, time windows, queues and the activity commands, checked against its model, on every family.
- The per-step and UPDATE paths allocate nothing; digests identical.

### FA-3: Performance, energy management, speed optimisation, endurance (L)

A-GRA's per-mode performance profile from hangar's data; energy management in every mode; long-range-cruise and max-endurance speeds; endurance validation and the fuel report.

**Status:** done 2026-09-27 in five steps:
- FA-3a, the performance tables (4.13), done 2026-09-27 and measured in section 14;
- FA-3b, energy on board and the fuel report: fuel and batteries on all 35 aircraft, the navigation report (STS-07, 4.14), endurance against a flown burn and the rotorcraft's tables, done 2026-09-27 and measured in section 14;
- FA-3c, the performance profile per mode (CAP-04 to CAP-15, 4.15), updated with the condition and configuration, done 2026-09-27 and measured in section 14;
- FA-3d, energy management in every mode (HSA-10, CTG-04, 4.16): the fleet climb case, done 2026-09-27 and measured in section 14;
- FA-3e, speed optimisation (HSA-05, LTR-17, 4.17) and endurance validation (VAL-03, 4.18), the first soft rejection override_rejection overrides, done 2026-09-27 and measured in section 14.

**Supporting models:** Performance tables, fuel flow (SUB-02, SUB-03).

**Items (20):** HSA-05, HSA-10; LTR-17; VAL-03; CAP-04, CAP-05, CAP-06, CAP-07, CAP-08, CAP-09, CAP-10, CAP-11, CAP-12, CAP-13, CAP-14, CAP-15; CTG-04; STS-07; SUB-02, SUB-03.

**Accepted when:**

- Each aircraft's profile values within 5 % of hangar's flight tests (level speeds, climb, stall).
- In the fleet climb case no mode lets CAS fall below 1.1 times the minimum.
- Best-range and best-endurance speeds within 5 % of hangar's optimum; endurance within 5 % of a flown burn.
- Changed flights (energy management) measured and listed; others identical.

### FA-4: References and state data (M)

Magnetic and barometric references in every mode and in the state; the QNH setting; reference frames; the terrain query; winds; orientation acceleration; terrain validation of commanded paths.

**Status:** in progress, in four steps:
- FA-4a, the terrain: the query and the paths checked against it (ENV-01, STS-11, VAL-06; 4.19), done 2026-09-27 and measured in section 14;
- FA-4b, the barometric altimeter: the QNH setting, what the altimeter reads in the state data, the barometric reference in the hsa and the patterns (ENV-03, STS-10, STS-04, HSA-07, LTR-16; 4.20), done 2026-09-27 and measured in section 14;
- FA-4c, the state data and frames: orientation rates and accelerations, the wind, reference frames (STS-02, STS-06, ENV-04; 4.21), done 2026-09-27 and measured in section 14;
- FA-4d, the magnetic model: declination, the magnetic reference and heading (ENV-02, HSA-03, STS-05). It needs the World Magnetic Model's published coefficients built in, which are not on this machine.

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
  - **A climb trades airspeed** (FA-3's energy management): asked by HSA or hold for 200 m more at 3,000 m, the C172 pulls up to its least airspeed at full throttle, then creeps at 0.1 m/s. Until FA-3 brings the fleet climb case, the wings' HSA and hold cases descend 200 m (rotorcraft climb). **Fixed in FA-3d** (4.16): the C172 climbs at 0.8 m/s at 1.16 times its least airspeed and reaches its height, and the wings' HSA and hold cases climb 200 m (section 14).
  - hangar's turboprops idle at 5,000 to 11,000 lbf on the ground: parked, the C-130J rolls to 11 m/s in 5 s until braked. A model item for hangar; the fleet test judges the wheel brakes by the stop. **Fixed in hangar since** (its ground range, docs/hangar.md): governed at flight idle on their low stop, the propellers made 3,440 lbf each standing still (the 11,000 was engine[0] at the spawn, where JSBSim starts every engine at full throttle). Now, with weight on the wheels, the throttle's first fifth sets the blades from ground idle, where they make no thrust standing still, and the engines hold the propellers' speed. Parked at idle, the C-130J makes -1 lbf and the EC-130H -84 lbf from the first step, and neither moves. Their flight tests' reports are identical before and after.
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

**FA-2a (the command envelope: CMD-02, CMD-03, CMD-10, CMD-16's record, CMD-18, CMD-19, CMD-20; VAL-01, VAL-02, VAL-08, VAL-12; WPT-24; ACT-08).**
- What it built is 4.8, in C++, the C ABI (1.8) and Python. `fsim.command/command_id`, `/batch`, `/traceability` and `/validate` are supported on every vehicle. The interactive flag is kept now; the activity commands it refuses come in FA-2c.
- `test_envelope` (4 cases, 92 checks) and its Python twin (4 tests):
  - a command's id comes back from its NEW, its UPDATEs and its CANCEL, refused ones too, and its activity keeps it with its requirements and interactive flag;
  - only a NEW that made an activity says so;
  - an hsa too fast and too high is refused with both findings, the altitude first as before, and flown with both values held under Clamp, each an adjustment;
  - a route's every point at fault is named with its field, and a curve's every section too tight;
  - a validation is answered as its NEW is, and nothing changes;
  - a batch is answered item by item, each with its own details.

  `test_c_abi` covers the 1.8 calls.
- The conformance walk (1,987,301 checks, one aircraft per adapter) validates a quarter of its NEWs first, drawn apart from the walk so that the walk takes the same path. Each validation must leave every record as it was and answer as the NEW then does: Valid exactly when the NEW is accepted, with the same reason and flags. Every adapter meets both a valid and a refused one. Every accepted NEW must say it made an activity.
- No earlier answer changes: under Reject a command is refused with the first finding, as before, and only the details list the others. Digests: identical, with protection and without.
- The allocation gate passes.
- A/B throughput against FA-1e's build (`fsim_control_bench`, interleaved):
  - per step (`micro`, 5 rounds): the medians within -2.1 % to +1.9 %;
  - the existing entry points' per-step command, the same level again (`command`, two runs of 9 rounds): +0.0 % and +2.9 %, the noise;
  - a NEW: 6.6 to 8.0 % more for a level switch (3.5 ns of 49), 4.4 to 7.0 % for a behaviour (5 ns of 109);
  - a checked UPDATE: 8.5 to 9.3 % more (2 ns of 22.5).

  That is the envelope's cost. Each activity keeps its id, requirements and flag, so its record is 232 bytes (was 160). Each NEW or UPDATE resets the details, and the answer is 48 bytes (was 40). The first version cost 12.6 % a NEW: its answer also carried the description and the associated id, which are now read from its reason and `other`. Its details had also parted the per-step path's members; they now sit at the end of the host.
- ctest: all 217 tests pass (the four envelope cases added).

**FA-2b (ranks, queues and time windows: CMD-05, CMD-06, CMD-07, CMD-08, CMD-09's carrying; ACT-03, ACT-06, ACT-07; STS-14).**
- What it built is 4.9, in C++, the C ABI (1.9) and Python. `fsim.command/rank`, `/no_interrupt`, `/precedence_override` and `/time_window` are supported on every vehicle; `/override_rejection` is partial until a soft rejection exists (FA-3; supported since FA-3e, 4.18).
- `test_schedule` (6 cases, 141 checks) and its Python twin (3 tests):
  - a policy's command ranked behind what flies waits, named in the answer, planned and queued; it starts when what it waited for ends; one ranked at or ahead takes;
  - a command that does not interrupt waits; the platform's interrupts any rank, or, deferring, lets the rank decide;
  - a capability's precedence decides before the rank, and a change to it starts what waited;
  - a start window delays the start; an end window ends a level; a critical start window missed, and a terminating activity done too early or too late for its critical end, fail it;
  - what waits is updated and canceled as what flies, a validation says it would wait, sixteen can, and the policy's end with its authority.

  `test_c_abi` covers the 1.9 calls.
- **The conformance walk checks a model of 4.9's rules on every adapter** (2,009,930 checks). A separately seeded stream gives 45 % of its NEWs a rank, 40 % of those no interrupting, some a precedence override (a policy's 5 %), half a time window, now and then out of order or already over. A new operation sets a capability's precedence. After every operation:
  - what waits is pending and within the windows it can still meet; a scheduled one before its start window, a queued one behind something on its axes it may not take, which it names. Nothing waits that could start.
  - What flies past its end window is only a terminating one whose end is not critical. A persistent activity completes only at its end window; a `time_constraint` failure is always one a window explains; any other failure of one that waited is a start refused.
  - A preemption is one the rules allow; a new record waits exactly when its NEW said so; nothing waits after a step unless it just started or still may not.

  Every adapter meets deferred NEWs, scheduled, queued and started activities. Across the walks: precedence changes, `not_allowed`, `time_constraint` refusals and failures, completions at an end window, failures of what waited.

  Two findings made the host right: a start can free axes an earlier one in the order waited for, so the scheduler passes again after a start. The existing entry points must never wait: a platform's precedence would have queued a policy's per-step commands until the queue filled.

  A family's first walk now and then misses a marker the draws moved past (the UH-1H's `active->pending` did); up to three more seeded walks of that family are run until it has them, the same every run.
- No earlier answer or flight changes: with every rank and precedence left as they are, the arbitration is ADR-26's. Digests: identical, with protection and without.
- The allocation gate passes, with a new case in which the steps counted end a level at its end window, start a route that waited for its start window, fail it at its critical end, and start what was queued behind it. Each is checked, with no allocation. What a start needs is made at its NEW: its behaviour, the route's scratch plan and path store, room for its waypoints or segments.
- A/B throughput against FA-2a's build (interleaved):
  - per step (`micro`, 5 rounds): the medians within -4.4 % to +2.7 %. A first build read +4 to 6 % on the hsa, route and pattern cases, which run only the runtime. The name functions this stage added to the runtime's file had moved its code; in the host's file, the runtime's cases read as before.
  - the existing entry points' per-step command and a checked UPDATE (`command`, 9 rounds): -4.3 % and -0.4 %.
  - a NEW: +18.8 % at a level switch (10 ns of 52) and +3 to 7 % with a behaviour. That is the arbitration (4 ns: precedence and rank against each holder, and the window's check) and a record of 296 bytes (was 232). The first version cost a level switch 26 %; building the record in place and passing the launch's setpoint by reference took most of it back.

  Against FA-1e's build, all of FA-2 so far costs a level switch +27 % (48.5 to 61.7 ns), a behaviour's NEW +17 %, a checked UPDATE +8.5 %, and nothing per step.
- ctest: all 223 tests pass (the six schedule cases added).

**FA-2c (activity commands: CMD-12, CMD-13, CMD-14, CMD-15, CMD-16's refusal; ACT-04).**
- What it built is 4.10, in C++, the C ABI (1.10) and Python. `fsim.activity/disable`, `/enable`, `/reset`, `/delete`, `/change_rank` and `/unassign` are supported on every vehicle.
- `test_activities` (3 cases, 62 checks) and its Python twin (3 tests):
  - a route disabled partway is kept and flies nothing; the vehicle default flies. Enabled, it resumes at the point it flew to; reset, it flies from its first point.
  - Unassigned, an hsa gives its axes to the command that waited behind it and waits in its turn. Re-ranked ahead, that command takes the axes back at once. Deleted, it ends and cannot be enabled.
  - An activity whose command takes none refuses all six (with its command id), and still takes UPDATE and CANCEL. Under Granted, a policy may not command what the platform flies.

  `test_c_abi` covers the 1.10 calls.
- **The conformance walk** (1,744,607 checks, one aircraft per adapter) draws an activity command for 7 % of its operations, from the separate stream: one of the six, at a live activity mostly, from a drawn caller. It draws `interactive` false for 10 % of its NEWs. The model:
  - the answer is refused exactly as the rules say (`unknown_activity`, `activity_ended`, `authority_held`, `not_interactive`, `queue_full` only for a flying one kept out of its slot);
  - an accepted command does what it says: disabled, deleted, re-ranked, enabled out of `disabled`, reset or unassigned out of `active`;
  - a record becomes disabled or deleted only by that command, and waits again only when unassigned or enabled;
  - a disabled activity is live, flies nothing, and outlives no end window.

  Every adapter meets all six accepted, and a disabled activity; across the walks, `not_interactive`, `active->disabled` and `disabled->pending`.
- **A rule 4.9 needed, found as the tests were written.** Among equals, "the newest takes" had let an unassigned, enabled or scheduled activity take its axes straight back from the newer command that had just got them. Equal ranks now go to the newer command: a NEW always, so no earlier answer changes; what waited never.
- No earlier answer or flight changes. Digests: identical, with protection and without. The allocation gate passes.
- **Code placement.** The names of the contracts' enumerations moved out of the runtime's file into their own (`src/control/Names.cpp`, last in the library). The case lines this step added to them had moved the per-step cases that run only the runtime by -7 % to +7 % against FA-2b's build. With them apart, per step (`micro`, 5 rounds) is within -7.4 % to +1.4 %: the one outlier, "apart, pseudo", is faster, back where FA-2a had it.
- A/B against FA-2b's build (`command`, 9 rounds): a level switch's NEW +2.7 %, a behaviour's -2.3 %, the per-step command +1.5 %, a checked UPDATE 0.0 %.
- ctest: all 226 tests pass (the three activity cases added).

**FA-2d (flight tasks and suggestions: TSK-01, TSK-02, VAL-10, VAL-11; CMD-08's repetition).**
- What it built is 4.11, in C++, the C ABI (1.11) and Python. `fsim.command/task` is supported on every vehicle.
- `test_tasks` (3 cases, 156 checks) and its Python twin (2 tests):
  - a task is refused for a reserved id, runs of what never completes, a capability the aircraft lacks. Kept, it is flown on its command with the task in its trace; it reports pending, executing, completed (100 %), canceled; it is refused while it flies, and forgotten.
  - An aileron roll kept for three runs, two seconds apart, flies them as one activity, active throughout. Its percent never falls, and it completes after the third.
  - An hsa refused for its speed names a suggestion that flies at the most the F-16C flies, and a validation suggests nothing. A route waiting 20 s, its climb fine at its NEW, is too steep from where the aircraft has flown by then. It fails, naming a suggestion that flies.

  `test_c_abi` covers the 1.11 calls.
- **The conformance walk** (6,998,600 checks, every adapter). It draws a task operation for 5 % of its operations: a task kept (a command drawn as a NEW's, some with runs and intervals), flown on a task command (held to the NEW's rules), or canceled. After every operation:
  - each task's state agrees with its activity: executing while it flies, pending while it waits, its activity's end once it ended, its runs its activity's;
  - its activity traces to it;
  - a refusal's suggestion, and a failure's, name a suggested task kept.

  Across the walks: tasks kept, flown, refused `task_active`, executing, completed, canceled, and refusals naming suggestions.
- **What the walk found and fixed:**
  - a task command whose activity was taken at once by what waited left the task unaware of its end;
  - an unassigned or enabled activity was still held to its first start's critical window, long past. A resumed activity answers to its end window only;
  - a behaviour done reports so at every step, which put a task's next run off step after step. The next run's time is now set once.
- No earlier answer or flight changes. Digests: identical, with protection and without.
- The allocation gate passes, with two new cases in the steps counted, each checked:
  - a task's route begins its second run;
  - a waiting route refused as it would start is kept as the suggestion its record names.
- **Code placement, again.** The per-step cases that run only the runtime read +6 to +10 % against FA-2c's build ("axes apart", "default hold", "apart, pseudo"), though FA-2d changed none of their code. The task code in the host's file, which links ahead of the runtime's, had moved it. In its own file, last in the library (`src/control/Tasks.cpp`), per step (`micro`, 5 rounds) is within -5.5 % to +0.7 %, the outliers faster.
- A/B against FA-2c's build (`command`, 9 rounds): the per-step command +3.1 %, a level switch's NEW +2.7 %, a behaviour's +2.2 %, a checked UPDATE +1.2 %.
- ctest: all 229 tests pass (the three task cases added).

**FA-2e (named controllers and the reports: AUT-06, ACT-10, ACT-13, ACT-15, CRV-14; the fleet cases).**
- What it built is 4.12, in C++, the C ABI (1.12) and Python. `fsim.control/controller_identity` is supported on every vehicle.
- `test_reports` (4 cases, 137 checks) and its Python twin (3 tests; one reads an hsa's setpoint back and flies it again as a batch item):
  - a setpoint read back completed, merged, appended, and as given while it waits; none once ended;
  - end points of a route (turn points, the last, round again, a loiter), a curve (from its reference), a pattern and the position level; none for an hsa, nor for a waiting curve whose reference is left out;
  - the commanded acceleration against the flown one in the F-16C's 60-degree turn, within 1 m/s^2 on each axis; a 2 g pull's -9.8 m/s^2 down; none where a throttle is given, none for the UH-60;
  - the altitude in its reference: an hsa's above the ground, a route's point, a curve's and the position level's above sea level.
- The named controllers' case in `test_grants` (51 checks), its Python twin (`test_controllers`), and `test_c_abi`'s 1.12 calls. One controller holds a grant. Another's request is refused `authority_held`, its NEW `not_granted`, its UPDATE, CANCEL and activity command `authority_held`, its release `not_granted`. The holder's release ends its activity, released. Open: any controller. Back to Granted: what flies without its own grant ends. A task command keeps its controller, and a revocation ends every controller's.
- **What the probe found.** The load factor, put along the body's normal, leaked n g sin(alpha) along the flight path: -3.3 m/s^2 at 2.4 g and 8 degrees of angle of attack, where the aircraft flew -0.1. The load factor is the lift's, normal to the path (JSBSim's Nlf): in wind axes the along-path part is what the aircraft flies (-0.09 against -0.06).
- **The conformance walk** (7,533,377 checks, every adapter). Its authority model knows controllers: every NEW, UPDATE, CANCEL, activity command, task command and cancel, request and release is drawn from three. Its rules:
  - only the holder's NEW flies; another's call on its activity is refused under Granted;
  - a request while another holds the grant is refused; a release of another's is refused, and ends nothing; a release ends the releaser's activities only;
  - Granted ends what flies without its own grant;
  - every record keeps its controller.

  Across the walks: requests refused `authority_held`, releases refused `not_granted`, and calls refused for another controller's activity.
- **The fleet** (all 35 aircraft at once, 1,976 checks):
  - validated, nothing flying; flown and read back, its setpoint completed; its commanded altitude in its reference, and an acceleration for every wing, none for every rotorcraft;
  - queued behind what flies and started as it ends; scheduled 2 s ahead, started, taking the axes; disabled and enabled;
  - a task executing, then canceled; named controllers;
  - a route's end points, and its commanded altitude the point flown to's;
  - no ground contact, no divergence, no calibrated airspeed below the envelope's least.
- No earlier answer or flight changes. Digests: identical, with protection and without. The record and the options keep their sizes (304 and 136 bytes): the controller fills a hole in each.
- The allocation gate passes, with a new case in the steps counted: a named controller's activity under Granted, updated every step by it, and what the vehicle is commanded read every step.
- Not-found answers are quiet: `fsim_vehicle_task_status` for a task not kept and `fsim_activity_get_setpoint` for an activity not live say so in `fsim_last_error`, and log nothing.
- **Code placement.** The reports' code is in its own file at the end of the library (`src/control/Reports.cpp`, between the tasks' and the names'). The runtime's file is unchanged: the commanded acceleration and altitude are worked out beside it (`CapabilityHost::commandState`), not in `ControlStack::commanded()`.
- A/B against FA-2d's build (the scratchpad worktree's): per step (`micro`, 5 rounds) within -0.7 % to +2.2 % ("attitude" +2.2 %, "apart, pseudo" +1.9 %, the rest within 1 %). The commands (`command`, 9 rounds): the per-step command -2.9 %, a level switch's NEW +1.9 %, a behaviour's -1.1 %, a checked UPDATE +1.2 %.
- ctest: all 235 tests pass (the report and fleet cases added).

**FA-3a (the performance tables: SUB-02, and SUB-03's fuel flow for the fixed wings).**
- What it built is 4.13: hangar's `performance` stage, the profile's `tables` section, the lookups. No flight uses them yet (FA-3c to FA-3e will): digests identical, with protection and without.
- The lookups are C++ (`tablesAt`, `tablesCeilingM`). The C ABI and Python read the tables' cells through the profile (`fsim_vehicle_profile_value`, `Vehicle.profile_value("tables/max_tas_ms/h0/w2")`). FA-3c brings them to both as the performance profile, A-GRA's per-mode limits.
- **The fleet** (31 fixed-wing designs, about 20 to 50 s each; 7 or 8 altitudes by 3 weights - 4 where the aircraft file starts elsewhere, 1 without fuel - flown where the aircraft flies level). The tables against the flight tests, in percent:

| Aircraft | Top speed % | Climb % | Stall % | Ceiling % | Conditions | Checks |
| --- | --- | --- | --- | --- | --- | --- |
| a10c | +0.0 .. +0.0 | -2.0 .. -0.3 | -0.5 | +0.5 | 21 of 21 | pass |
| b52h | +0.1 .. +0.2 | -0.8 .. -0.3 | +1.1 | -0.2 | 25 of 28 | pass |
| c130j | +0.5 | -4.0 .. -0.8 | +1.9 | +0.0 | 24 of 24 | pass |
| c172 | -0.0 | -3.8 .. -2.0 | -0.9 | -0.7 | 21 of 21 | pass |
| c17a | -0.3 .. +0.1 | -0.3 | - | -0.7 | 21 of 21 | pass |
| e3g | -0.8 .. +0.1 | -4.0 .. -0.7 | -0.8 | -0.9 | 21 of 21 | pass |
| e7a | -0.0 .. +0.1 | -1.7 .. -0.1 | -3.1 | +2.2 | 24 of 24 | pass |
| ea18g | -2.3 .. +0.1 | -2.1 | - | +0.1 | 24 of 24 | pass |
| ec130h | +0.0 | -1.3 .. -0.7 | +0.8 | +0.1 | 27 of 28 | pass |
| f15c | +0.0 .. +0.2 | -2.0 | - | -0.1 | 24 of 24 | pass |
| f16c | +0.0 .. +0.1 | -2.9 | - | -1.3 | 24 of 24 | pass |
| f22a | +0.0 .. +0.2 | -2.6 | - | +0.1 | 24 of 24 | pass |
| f35a | -1.0 .. -0.0 | -0.9 | - | -0.4 | 24 of 24 | pass |
| fa18c | +0.0 .. +0.2 | -1.7 | - | +1.4 | 24 of 24 | pass |
| gripen | -1.2 .. +0.1 | -3.3 | - | +1.2 | 21 of 21 | pass |
| h6k | +0.0 .. +0.1 | -0.7 .. -0.1 | -3.1 | +1.6 | 23 of 24 | pass |
| j10a | -0.2 .. +0.0 | -2.0 | - | +0.2 | 21 of 21 | pass |
| j20a | -0.5 .. +0.0 | -1.5 | - | +0.2 | 21 of 21 | pass |
| kc135r | +0.0 .. +0.1 | -2.3 .. +1.8 | -1.3 | +0.9 | 27 of 28 | pass |
| kc46a | -0.3 .. +0.1 | -0.9 .. -0.6 | -0.5 | +2.9 | 21 of 21 | pass |
| mig29a | -0.0 .. +0.1 | -3.0 | - | -1.1 | 24 of 24 | pass |
| mirage2000 | -4.1 .. +0.1 | -4.4 | - | +0.2 | 21 of 21 | pass |
| rafale | -0.7 .. +0.1 | -2.9 | - | +0.4 | 24 of 24 | pass |
| rc135w | +0.1 .. +0.1 | -3.3 .. -0.4 | +0.8 | +1.9 | 27 of 28 | pass |
| rq4b | +0.0 | -2.6 .. +0.4 | -1.9 | +0.2 | 27 of 28 | pass |
| skua | +0.4 | - | -3.1 | -0.3 | 7 of 7 | pass |
| su25 | +1.3 .. +1.3 | -2.8 .. -1.0 | +0.7 | +0.3 | 21 of 21 | pass |
| su27s | -0.0 .. +0.1 | -2.3 | - | -0.3 | 24 of 24 | pass |
| su57 | -0.5 .. +0.0 | -2.4 | - | +0.3 | 24 of 24 | pass |
| typhoon | +0.0 .. +0.1 | -3.1 | - | +0.1 | 24 of 24 | pass |
| u2s | -0.6 .. +0.2 | -1.1 .. +1.7 | +0.5 | +2.1 | 31 of 32 | pass |

All 183 comparisons are within 5 %: 57 top speeds, 80 climbs, 15 stalls and 31 ceilings. The worst are the Mirage 2000's top Mach number at -4.1 % and its best climb at -4.4 %, a stall at -3.1 % and the KC-46A's ceiling at +2.9 %.
- The Mirage 2000's flight test extrapolates its Mach 2.20 from the Mach 2.106 it flew to in six minutes. The tables' settled Mach 2.111 at that weight agrees with what it flew to within 0.2 %.
- The Skua's climbs above 1,500 m are flown below the envelope's least speed, which the platform's velocity loop flies no slower than. They are shown as beyond the tables and not compared.
- **Past a drag rise.** 12 conditions on 9 fighters (EA-18G, F-35A and Su-57 2 each; F-15C, FA-18C, J-20A, MiG-29A, Mirage 2000 and Su-27S 1 each): in those conditions, full power holds level faster than a level acceleration reaches.
- **Level only in short runs, near a ceiling.** 25 conditions on 15 designs, the most on the KC-135R and Mirage 2000 (4 each).
- **Fuel-flow points.** 10,914 of 11,456 level points (95.3 %) held their speed and height, and only those carry a fuel flow. The excess power at full power is known at every point, at idle at all but 55.
- **Size.** The tables add 41,728 properties to the 31 aircraft files: 405 on the one-weight Skua, up to 1,829. Nothing else in them changed.
- **What the checks found, each fixed in the stage before the tables were kept:**
  - The velocity loop holds a vertical speed, not a height. Accelerating through Mach 1 at 100 m, the F-15C and FA-18C sank into the ground and slid along it, and the slide read as a settled top speed of 90 m/s. The height is now held.
  - A fighter's gear is up, so on its belly its wheels report nothing. The light and half-weight Su-27S slid at 1 m and were accepted, until any run within 10 m of the ground was refused.
  - Loaded at 14 km, the F-35A stops in the transonic drag rise. The short runs past it found the top where full power holds level beyond it, and the supersonic best climbs that set the fighters' ceilings. Those ceilings were 5 to 11 % low before.
  - The fighters' flight tests read their top Mach number and best climb after minutes at full afterburner, hundreds to thousands of kilograms lighter. They are now compared at that weight, found by flying the tests again. The re-flown tests match the originals to four figures.
  - Eleven speed points lost up to 9 % of the excess power near the top speed, where the curve falls steeply (the Su-25). Sixteen points keep it within about 1 %.
  - The straight wings' flight test fits its ceiling through its three highest climbs, the third often far below: the RQ-4B's read 45,152 ft. Against its two highest climbs extended, the reference the tables use, the RQ-4B agrees within 0.2 %.
  - The B-52H's file starts its tanks 40 % full. The stage had taken that as full capacity, which would have reported 100 % on a B-52H at 40 %. The capacity is now read from the tanks.
  - A full-power run's first seconds record the engines spooling up: the EC-130H's excess power climbs from 7 to 11 m/s over 6 s, and a band starting there read its lowest climb 6 % low. Each run's first 8 s are left out.
  - The light C-17A's velocity loop oscillates at 98 m/s and 3° of angle of attack, which read as a stall at 91 m/s. The stall now comes from the highest lift coefficient at each altitude, and the angle-of-attack break counts only near the limit.
- Digests are identical, with protection and without.
- `control_alloc` passes.
- The interleaved A/B against FA-2e's bench shows no change beyond noise. The micro-benchmark medians range from -1.6 % to +2.8 %; the largest, attitude, is +0.7 % on its minimum. The command medians range from -1.5 % to +1.8 %.
- ctest: all 240 tests pass (the five tables cases added; hangar's performance unit tests 8).

**FA-3b, energy on board (SUB-03: the rotorcraft's fuel and batteries).**
- **The helicopters burn fuel.** Each has a tank at the c.g., full as it spawns and part of its design weight: the UH-60A's 2,340 lb and the UH-1H's 1,400 lb. Its engine burns its specific fuel consumption times the power it gives: 0.571 lb per shp-h (TM-85890's 1/K_E) and 0.623 (a T53's on a UH-1B). JSBSim's electric engine, their power source, is patched to burn (`cmake/JsbsimPatches.cmake`, THIRD_PARTY_NOTICES). Once the tank is empty the engine gives no power and the rotor is left to the air: the UH-60A's rotor falls from 258 to 139 rpm in 3 s, and it sinks. An electric engine without a consumption burns nothing, as before.
- **The quadrotors carry batteries.** The power a battery gives is the published hover's (its capacity over its flight time, taken as a hover's), scaled by the rotors' speed cubed. Once it is spent the motors stop: the IRIS+ after 18.7 minutes' hover at 181.2 W, the Crazyflie 2.0 after 7.0 at 7.59 W.
- **The named change, measured.** The helicopters now lighten as they fly:
  - held ten minutes, the UH-60A burns 161 lb at a hover (1.0 % of its weight) and 90 lb at 30 m/s; the UH-1H burns 59 and 41 lb;
  - their loops hold them as before: height within 0.25 m and position within 0.1 m of the flights without fuel, the collective 0.4 to 1.2 % lower as they lighten;
  - hangar's flight tests fly with the fuel frozen, so their identified hover, and the loops the platform designs from it, are unchanged;
  - the quadrotors fly the same, bit for bit, until the battery is spent;
  - the fixed wings are untouched: digests identical, with protection and without.
- ctest: all 240 tests pass.

**FA-3b, the navigation report (STS-07) and the Skua's battery.**
- **The Skua carries a battery.** `[battery] capacity_wh = 1170` is its 6.5 kg pack at 180 Wh/kg, 12S Li-ion: like everything about the Skua, a design choice. The power it gives is the motor's: the battery's 44.4 V times the throttle, times the current Drela's motor model draws.
  - At 22 m/s and 3,000 m it gives 456 W. The report said 151.8 minutes, and the pack was spent after 152.0 minutes' flight.
  - The motor then stops, and the Skua glides at 16 m/s, sinking 1.3 m/s.
  - Until the pack is spent it flies the same, bit for bit: ten minutes' cruise and ten minutes' climb, compared with the aircraft file from before.
- **The report on all 35.** Each aircraft flew level at its cruise speed at 3,000 m (the rotorcraft hovered at 300 m), after 120 s to settle.
  - *Drain*: the consumption the report gives, summed step by step over 60 s, over what left the tanks or battery. It is 1 to within 1.1 × 10⁻⁶ on every aircraft.
  - *Flown / report*: the time to use a fifth of what the aircraft had (or two hours' worth), over the report's time for that amount at its consumption then. It is 1.000 to 1.041, all within the plan's 5 %. The aircraft flies longer because it lightens as it burns. A battery's weight does not change, and it matches.
  - The RQ-4B's flow stays flat. At 3,000 m it needs so little thrust that its fuel flow sits on JSBSim's idle floor for its turbine (MilThrust^0.2 × 107 lb/h: 639 lb/h), which does not change as it lightens.

| Aircraft | Flies on | Left (%) | Consumption | Endurance (min) | Drain | Flown (min) | Report (min) | Flown / report |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| a10c | fuel | 98.9 | 0.2954 kg/s | 278.5 | 1.00000 | 55.9 | 55.7 | 1.004 |
| b52h | fuel | 39.8 | 1.491 kg/s | 630.0 | 1.00000 | 121.8 | 120.0 | 1.015 |
| c130j | fuel | 99.4 | 0.6636 kg/s | 519.9 | 1.00000 | 104.9 | 104.0 | 1.009 |
| c172 | fuel | 99.0 | 0.006366 kg/s | 305.9 | 1.00000 | 61.6 | 61.2 | 1.006 |
| c17a | fuel | 99.7 | 1.577 kg/s | 1133.9 | 1.00000 | 120.9 | 120.0 | 1.008 |
| cf2 | battery | 57.2 | 7.6 W | 4.0 | 1.00000 | 0.8 | 0.8 | 1.000 |
| e3g | fuel | 99.2 | 2.902 kg/s | 363.5 | 1.00000 | 73.7 | 72.7 | 1.013 |
| e7a | fuel | 59.6 | 0.5196 kg/s | 399.1 | 1.00000 | 80.3 | 79.8 | 1.006 |
| ea18g | fuel | 98.1 | 0.6491 kg/s | 159.3 | 1.00000 | 32.6 | 31.9 | 1.022 |
| ec130h | fuel | 59.5 | 0.5797 kg/s | 356.1 | 1.00000 | 71.8 | 71.2 | 1.009 |
| f15c | fuel | 98.5 | 0.5177 kg/s | 193.4 | 1.00000 | 39.7 | 38.7 | 1.026 |
| f16c | fuel | 98.1 | 0.3273 kg/s | 158.6 | 1.00000 | 32.3 | 31.7 | 1.019 |
| f22a | fuel | 98.5 | 0.6637 kg/s | 202.9 | 1.00000 | 41.6 | 40.6 | 1.026 |
| f35a | fuel | 98.8 | 0.5528 kg/s | 246.6 | 1.00000 | 50.5 | 49.3 | 1.024 |
| fa18c | fuel | 98.3 | 0.4498 kg/s | 178.5 | 1.00000 | 36.5 | 35.7 | 1.022 |
| gripen | fuel | 97.8 | 0.2964 kg/s | 131.9 | 1.00000 | 27.1 | 26.4 | 1.026 |
| h6k | fuel | 59.7 | 0.574 kg/s | 624.1 | 1.00000 | 121.9 | 120.0 | 1.016 |
| iris | battery | 84.0 | 181.2 W | 15.7 | 1.00000 | 3.1 | 3.1 | 1.000 |
| j10a | fuel | 98.5 | 0.3993 kg/s | 203.6 | 1.00000 | 42.2 | 40.7 | 1.035 |
| j20a | fuel | 98.5 | 0.8369 kg/s | 206.1 | 1.00000 | 42.6 | 41.2 | 1.034 |
| kc135r | fuel | 49.9 | 0.7217 kg/s | 1044.5 | 1.00000 | 121.4 | 120.0 | 1.012 |
| kc46a | fuel | 99.7 | 1.367 kg/s | 1170.9 | 1.00000 | 121.7 | 120.0 | 1.014 |
| mig29a | fuel | 97.8 | 0.4222 kg/s | 135.2 | 1.00000 | 27.5 | 27.0 | 1.019 |
| mirage2000 | fuel | 97.7 | 0.4002 kg/s | 128.6 | 1.00000 | 26.8 | 25.7 | 1.041 |
| rafale | fuel | 98.3 | 0.4328 kg/s | 178.0 | 1.00000 | 36.8 | 35.6 | 1.034 |
| rc135w | fuel | 49.8 | 0.7807 kg/s | 631.1 | 1.00000 | 121.4 | 120.0 | 1.011 |
| rq4b | fuel | 29.8 | 0.08052 kg/s | 484.3 | 1.00000 | 96.9 | 96.9 | 1.000 |
| skua | battery | 97.0 | 684.7 W | 99.5 | 1.00000 | 19.9 | 19.9 | 1.000 |
| su25 | fuel | 98.0 | 0.3382 kg/s | 144.8 | 1.00000 | 29.3 | 29.0 | 1.012 |
| su27s | fuel | 98.0 | 0.5821 kg/s | 147.9 | 1.00000 | 30.1 | 29.6 | 1.017 |
| su57 | fuel | 98.5 | 0.8278 kg/s | 204.3 | 1.00000 | 42.4 | 40.9 | 1.038 |
| typhoon | fuel | 98.1 | 0.516 kg/s | 158.3 | 1.00000 | 32.8 | 31.7 | 1.035 |
| u2s | fuel | 49.6 | 0.1718 kg/s | 419.7 | 1.00000 | 84.8 | 83.9 | 1.010 |
| uh1h | fuel | 98.7 | 0.04461 kg/s | 234.3 | 1.00000 | 47.8 | 46.9 | 1.021 |
| uh60 | fuel | 97.9 | 0.1222 kg/s | 141.8 | 1.00000 | 28.8 | 28.4 | 1.017 |

- **Tests.**
  - `test_navigation` (C++):
    - fuel on a designed wing, a stock aircraft and a fighter, each against its drain;
    - the UH-60A's fuel, and the Crazyflie's battery flown until spent, within 1 % of its endurance;
    - a glider that flies on neither;
    - the playtime to a recovery point against its terms, the reserve's contingency, the tanks emptied, the refusals.
  - The C ABI's 1.13 block, `python/tests/test_navigation.py`, and hangar's test of the battery channel.
- **Digests:** identical to FA-2e's and FA-3a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-2e's build (5 interleaved rounds of `micro`, 9 of `command`): the medians are within -2.1 % to +3.0 % and -2.8 % to +0.2 %. That is the noise; FA-3a's were -1.6 % to +2.8 % and -1.5 % to +1.8 %. The report is worked out only when asked.
- ctest: all 244 tests pass.

**FA-3b, the rotorcraft's tables (SUB-02, SUB-03), a battery's power in the tables, and two named changes.**
- **The rotorcraft's tables** (4.13; [hangar.md](hangar.md), "Rotorcraft"), flown for all four. Loaded, at 100 m:

| Aircraft | Weight | Level speeds (m/s) | Best endurance (m/s) | Best range (m/s) | Best climb (m/s) |
| --- | --- | --- | --- | --- | --- |
| uh60 | 7,439 kg | 0 to 96.2 | 37.9 (241 kg/h) | 62.9 (303 kg/h) | 16.0 at 24.3 m/s |
| uh1h | 2,793 kg | 0 to 56.2 | 25.2 (109 kg/h) | 40.3 (133 kg/h) | 13.7 at 17.6 m/s |
| iris | 1.5 kg | 0 to 16.6 | 15.9 (138.6 W) | 15.9 (138.6 W) | none flown |
| cf2 | 0.027 kg | 0 to 21.6 | 0 (7.60 W) | 20.8 (9.59 W) | none flown |

  - **Checks, all passed.**
    - The hover's power against the fly stage's: the UH-60A's 1,698 shp, the UH-1H's 571 shp. For the quadrotors, against the design's hover draw: 181.4 W against 181.6 W, and 7.60 W against 7.60 W.
    - The UH-60A's power at the six published trims' speeds, against the fly stage's trims: all within 1 %.
    - The lighter burns less at every altitude and speed.
    - A burn flown at the best-endurance speed against the tables: −0.2 % for both helicopters, 0.0 % for the IRIS+, +1.5 % for the Crazyflie (190 s, half its battery).
  - **The helicopters' full-power climbs.** The power rises to the limit over 15 s, since stepped at once the UH-60A at 130 kt pitched 48° nose down. Each climb is averaged over half a minute.
    - Between 95 and 120 kt the hold rings in a UH-60A's climb. In each row up to three of those speeds, and its fastest, have no stationary average, and those cells are not flown.
    - The UH-1H's table is complete.
  - **The quadrotors fly no climb.** Their thrust ignores a climb's inflow.
  - **What the tables show of the models** (the owner's to weigh; [rotorcraft.md](rotorcraft.md), section 7):
    - None of the four loses power or thrust with height, so their ceilings lie far above the rows.
    - The IRIS+'s drag is PX4's, in the rotors' plane only, so tilted forward it lifts too. Its power falls all the way to its top speed, and its best speeds are its top.
    - The Crazyflie's power is least in the hover: neither quadrotor model has translational lift.
- **The ceiling's extension is capped at as high again.** A rotorcraft still climbing 15 m/s at 3,000 m would have had its ceiling extended to 50 to 300 km. The fixed wings' extensions reached at most 42 % above their rows, so every fixed wing's ceiling is unchanged.
- **The navigation report's return** now flies a rotorcraft's best range from its tables. The UH-60A returns at 123 kt, not the loops' default 29 kt. The IRIS+'s playtime over 5.6 km is 12.2 minutes, where the default speed made it none.
- **A named change: the Crazyflie's drag along its axis.** gym-pybullet-drones' `drag_z`, which the design gives, was dropped: JSBSim keeps only the last `<axis>` of a name, and the ground effect's Z axis came after it. Now one axis carries both. No other aircraft repeats an axis.
  - The identified hover's heave damping goes from 0 to 0.146 1/s, and its heave control power from 21.44 to 21.12 m/s² per unit (−1.5 %).
  - Through the platform's loops the hover is the same, and a 1 m/s climb draws 3.6 % more power.
  - At 6 m/s its vertical speed holds six times tighter, and it draws 2.3 % more power where it drew less.
  - The fleet test judged a rotorcraft's orbit from its first lap of bearing round the centre, which for a rotorcraft starting at the centre is its way out to the circle. The Crazyflie's way out now crossed the lap's mark 1.3 m short of the circle; its tracking is no worse, 0.03 m off the circle from the second lap against 0.04 m before. A rotorcraft is now judged from its second lap. The worst is 3.0 % of the radius, the UH-60A's, against the 10 % allowed; before, the IRIS+ in a replica read 13.9 % from the first.
- **A named change: the Skua's tables.** They now carry its battery's power, and its band begins at 1.15 times the stall.
  - An electric motor needs no spool-up, so its full-power run leaves out 1 s, not 8. The Skua passes its slow speeds within those 8 s, so its band had begun at 31 m/s, past the design's 22 m/s cruise.
  - Its best endurance is now 16.1 m/s (370 W) and its best range 19.2 m/s (405 W).
  - Its FA-3a checks still pass. Its ceiling is 34,810 ft, +2.0 % against the flight test (−0.3 % before): its low speeds' climbs now count.
  - Its full-power climb below 19.8 m/s stays unflown, as the flight tests' climbs there were beyond the tables before.
- **Digests:** identical to FA-2e's and FA-3a's, with protection and without. No digested flight is a rotorcraft's or the Skua's. The allocation gate passes.
- **A/B throughput.**
  - Micro, against FA-2e's build (5 rounds): −2.6 % to +0.4 %.
  - Command, against FA-3b's navigation report build (15 rounds): −0.5 % to +1.6 %, but for a behaviour's NEW at +5.4 % (7 ns). That is code placement. Nothing on its path changed, and the path neither copies, destroys nor reads the tables.
    - The larger tables section changed the profile code's inlining, and the path's functions moved 0.8 to 2.6 KB.
    - A 448-byte unused function added to the bench moved the same case by 4 ns.
- ctest: all 244 tests pass.

**FA-3c, the performance profile (CAP-04 to CAP-15).**
- **The profile on all 35, against its tables.** Each aircraft flew its cruise level at 3,000 m for 15 s (the rotorcraft hovered at 300 m), its tanks full and the fuel frozen: a weight every table flies.
  - At that weight each of the profile's points is a table's cell. That covers the airspeeds against altitude (within the envelope's limits, which bind none of the designs), the best speeds, the ceiling, and every excess-power and burn point with its speed, the band's fraction.
  - The profile matched cell for cell on all 35, 15,479 values in all, to within 4.9e-06: the six digits the aircraft file keeps.
  - FA-3a checked the fixed wings' tables against their flight tests (level speeds, climb, stall and ceiling) within 5 %. FA-3b checked the rotorcraft's against their fly stage's hover and trims and a flown burn. All 230 of those checks pass, and the profile carries them: FA-3's first acceptance criterion.

| Aircraft | Rows | Points | Worst | Ceiling at full tanks (m) | Tables' checks |
| --- | --- | --- | --- | --- | --- |
| a10c | 7 | 477 | 3.8e-06 | 9,165 | 10 of 10 |
| b52h | 5 | 255 | 4.1e-06 | 13,486 | 9 of 9 |
| c130j | 8 | 545 | 4.6e-06 | 9,886 | 8 of 8 |
| c172 | 7 | 477 | 4.4e-06 | 3,936 | 7 of 7 |
| c17a | 7 | 477 | 4.6e-06 | 11,135 | 5 of 5 |
| cf2 | 4 | 145 | 3.5e-06 | none | 2 of 2 |
| e3g | 7 | 477 | 4.4e-06 | 10,341 | 9 of 9 |
| e7a | 8 | 545 | 4.6e-06 | 11,800 | 10 of 10 |
| ea18g | 8 | 539 | 4.7e-06 | 17,449 | 5 of 5 |
| ec130h | 6 | 409 | 4.8e-06 | 6,571 | 8 of 8 |
| f15c | 8 | 507 | 4.5e-06 | 19,635 | 5 of 5 |
| f16c | 8 | 535 | 4.5e-06 | 18,730 | 5 of 5 |
| f22a | 8 | 509 | 4.4e-06 | 18,430 | 5 of 5 |
| f35a | 8 | 543 | 4.0e-06 | 17,470 | 5 of 5 |
| fa18c | 8 | 539 | 4.9e-06 | 18,540 | 5 of 5 |
| gripen | 7 | 443 | 4.6e-06 | 18,487 | 5 of 5 |
| h6k | 7 | 443 | 4.8e-06 | 11,628 | 9 of 9 |
| iris | 4 | 145 | 3.5e-06 | none | 2 of 2 |
| j10a | 7 | 435 | 4.7e-06 | 18,109 | 5 of 5 |
| j20a | 7 | 447 | 4.3e-06 | 17,929 | 5 of 5 |
| kc135r | 6 | 409 | 4.7e-06 | 10,723 | 10 of 10 |
| kc46a | 7 | 477 | 4.6e-06 | 9,269 | 9 of 9 |
| mig29a | 8 | 541 | 4.2e-06 | 18,913 | 5 of 5 |
| mirage2000 | 7 | 361 | 4.4e-06 | 17,901 | 5 of 5 |
| rafale | 8 | 539 | 3.9e-06 | 18,962 | 5 of 5 |
| rc135w | 6 | 409 | 4.6e-06 | 10,925 | 11 of 11 |
| rq4b | 6 | 409 | 4.3e-06 | 12,110 | 10 of 10 |
| skua | 7 | 447 | 4.7e-06 | 10,611 | 4 of 4 |
| su25 | 7 | 477 | 3.3e-06 | 16,012 | 10 of 10 |
| su27s | 8 | 539 | 4.8e-06 | 19,934 | 5 of 5 |
| su57 | 8 | 501 | 3.9e-06 | 17,748 | 5 of 5 |
| typhoon | 8 | 479 | 3.9e-06 | 18,964 | 5 of 5 |
| u2s | 7 | 477 | 4.1e-06 | 18,447 | 10 of 10 |
| uh1h | 4 | 273 | 4.9e-06 | none | 3 of 3 |
| uh60 | 4 | 249 | 4.5e-06 | none | 9 of 9 |

- **Configuration and condition** (`test_performance_profile`):
  - An F-16C given flap and gear placards, with its flaps out and gear down, has its airspeed bounded by the gear's placard at the altitude now, and the tables' values left out.
  - Spawned in the air, its gear retracting, it is not clean until the gear is up.
  - A profile asked again into the same one reuses its vectors, and its weight follows the fuel burned.
- **A-GRA's schema**: MA_SpeedType carries an airspeed, an altitude and a weight, but no speed. So MaxDescentRate and ExcessPowerMaxClimb cannot carry the rate in the schema; `fsim.agra.performance_profile` adds it as "Value".
- **Digests:** identical to FA-2e's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-3b's rotorcraft build (8edf5da), built in the scratch worktree (5 rounds of `micro`, 9 of `command`):
  - the micro cases are within −3.2 % to +0.9 %, and the command cases −1.5 % to 0.0 %;
  - a behaviour's NEW reads −16.4 %, because that baseline, built in another tree, runs it at 161.7 ns where the same source built in this tree ran 134.3 ns. It is +0.7 % against that.
  - The profile's code is not linked into the bench: it is asked for, never stepped.
- ctest: all 247 tests pass.

**FA-3d, energy management (HSA-10, CTG-04).**
- **The fleet climb case** (`test_fleet`), FA-3's second acceptance criterion:
  - Every wing is asked, by hold and by HSA, for 200 m more and a quarter turn at 3,000 m, and given 200 s. Until now the case descended.
  - Each keeps its calibrated airspeed at 1.1 times its least or more. The lowest is the C172 at 1.16 times, then the EC-130H at 1.56.
  - Each ends within 4.6 m of its height (the B-52H) and within 0.14° of its heading (the C172).
- **The C172's climb** (the probe `energy_report.py`), asked by HSA for 200 m at 3,000 m and 50 m/s:
  - At full power, its airspeed settles at 1.16 times its least while it climbs at 0.8 m/s. It reaches its height in 150 s and gathers its speed again.
  - FA-1e found it pulling up to its least airspeed and creeping at 0.1 m/s, 100 m short.
  - While the rule holds the climb back, the envelope report counts every update under `cas_min`, and the activity is flagged demand-limited and saturated (its throttle at its stop): A-GRA's ACTIVE_FULLY_CONSTRAINED.
- **Changed flights, measured.** The fleet test was flown twice with the same cases: once by FA-3c's rules (no energy rule, the fly-by-wire designs without a least airspeed), once by FA-3d's. Each flight's end state was compared bit for bit: 611 of the 622 flights are identical. The 11 that change:
  - **The C172's climb, by hold and by HSA.** Its least airspeed goes from 24.6 to 29.5 m/s (its least is 25.4), and it ends 88 m higher, at its height.
  - **The A-10C's loop**, given up below its least airspeed as before. In its recovery the rule asks for a descent of 8.0 to 8.7 m/s where the recovery asked for 8, and its saturated recovery flies the same. Its end state differs in the 14th digit of its velocity.
  - **The loops of seven fly-by-wire fighters** (F-22A, F/A-18C, Gripen, J-10A, J-20A, Rafale, Su-57). They float over the top 3 to 26 % below their new least airspeed.
    - They are now given up there (failed, `behavior_failed`), where they completed.
    - Their recoveries then slow further, by up to 10.6 m/s: the Gripen's lowest airspeed goes from 48.6 to 38.0 m/s.
    - The give-up is FA-1d's rule (section 6), now that their least airspeed is known. Pulling through would lose less speed than levelling out from the top; that is an open finding for the aerobatics behaviour.
  - **The Mirage 2000's loop** is refused for its entry speed (`performance_limit`). A loop needs twice its least airspeed, 117 m/s, and it flies at 110. Before, the loop was accepted and failed, falling to 12.6 m/s.
  - The other loops complete as before: the F-15C, F-16C, F-35A, MiG-29A, Su-25, Su-27S and Typhoon. The EA-18G's is given up as before.
- **Tests that change with the fighters' least airspeed:**
  - `test_envelope`:
    - The F-16C's route finds a fourth fault. Its last point, 25 km up, asks for 200 m/s, which is under its least there. Clamped, that point is flown at its least at the ceiling it is held to, 215 m/s.
    - The UPDATE's speed is now one it flies at that ceiling, 300 m/s.
  - `test_conformance`: the Mirage 2000 is asked for a loop at its reference condition and refused for its speed. The test then asks for an aileron roll at 2 g, as it asks for the gear up where the gear's placard refuses it down.
- **The support table:** `fsim.fa/envelope_management` (CTG-04) is supported: protection's limits, and now energy management. `fsim.guidance.hsa` stays partial for its speed optimisation (FA-3e) and its magnetic and barometric references (FA-4).
- **The performance profile** (the FA-3c check, again): cell for cell with its tables on all 35, at the weight each is spawned at with full tanks; the worst difference is 4.9e-06.
- **Digests:** identical to FA-3c's (and FA-2e's), with protection and without. The allocation gate passes.
- **A/B throughput** against FA-3c (8913081), built in the scratch worktree: 5 rounds of `micro` and 9 of `command`, twice each.
  - The three velocity-level cases with protection limiting (`limit`, `design`, `pseudo`) are +1.0 to +2.0 %, 0.9 to 1.6 ns on their medians: the rule's arithmetic.
  - Over both runs, the other micro cases are within −1.2 % to +2.8 % on their medians, and the command cases within −0.3 % to +2.2 %. No case's rise repeated: the attitude's +2.8 % (its minimum unchanged) read −0.9 % in the second run.
  - World throughput (3 rounds): 100.0 to 100.8 % of FA-3c's. With protection on, the F-16C and B-52H fly at 99.6 % of their throughput with it off (the gate: 97 %).
- ctest: all 247 tests pass.

**FA-3e, speed optimisation (HSA-05, LTR-17).**
- **The fleet** (`test_fleet`), FA-3's acceptance for the best speeds:
  - Every aircraft, its heading and height held, flies each optimisation for 240 s (a rotorcraft for 90 s).
  - Each flies its optimum within 5 %. The 31 wings are within 0.01 %, their height within 1.1 m (the Mirage 2000); the helicopters within 0.3 %; the multirotors, along the nose, within 0.84 % (the Crazyflie's best range). The Crazyflie's best endurance is its hover.
- **HSA and pattern** (`test_modes`, `test_patterns`):
  - The resolved speed is the optimum at the altitude flown to. Flown, it is the optimum now: the F-16C's within 2 %, the C172's orbit within 3 % once level.
  - A speed replaces it, and a fraction is refused `invalid_parameter`.
  - The stock c172x refuses it `not_implemented`, as its support table says.
  - The C ABI (1.15) and Python do the same.
- **The conformance walks.**
  - They draw an optimisation only in walks of their own, one per adapter, held to the same rules: flown where there are tables, refused `not_implemented` on the stock aircraft. So the other walks draw what they drew before, and meet the rarer answers they met; a field more to draw had moved every walk, and no task then completed in 60 seeds.
  - The moved walks found three gaps in the test's model, not in the platform, now closed:
    - a task command whose task's activity is live is refused `task_active` before its NEW;
    - an activity that waited, started and was done before its critical end window opened fails `time_constraint` within one operation;
    - an activity command that sets a live activity waiting (unassign, disable) past its end window fails it at once.
- **Named changes, measured.**
  - **The multirotors' tables**, flown again within their loop's tilt:
    - the IRIS's top level speed 16.6 → 13.1 m/s, its best endurance and range 15.9 → 12.5, its steepest pitch 33.5 → 26.7°;
    - the Crazyflie's top 21.5 → 18.0 m/s, its best range 20.8 → 17.4 (its best endurance the hover, as before).
  - **The helicopters' tables**, within the 20° they never approach: their level tables are identical. 17 of the UH-1H's 192 full-power climb cells at the top speeds move, by 1.18 % at most, and 9 of the UH-60's are flown where none were. Every table check passes.
  - **The rotor velocity loop's forward integral.** 4 of the fleet's 656 flights change: the IRIS's and the Crazyflie's optimised HSAs (new), and the IRIS's waypoints, by 6e-9 m. The orbits are unchanged: sideways it trims half, as before. The whole tilt sideways had run the UH-60 up to 9 m off its 150 m circle.
- **The performance profile** (FA-3c's check, again): cell for cell with the tables on all 35, the rotorcraft's re-flown ones too; the worst difference is 5.1e-06.
- **Digests:** identical to FA-3d's, with protection and without; no digest flight is a rotorcraft's. The allocation gate passes.
- **A/B throughput** against FA-3d (e2e48b4), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` twice.
  - The micro cases are within −1.3 % to +1.8 %.
  - The command cases are within −1.4 % to +3.0 % (0.2 ns), except a behaviour's NEW at −10 %. That baseline, built in another tree, runs it at 135 ns where this one runs 122: the layout's, as FA-3c found.
  - World throughput is 100.2 to 101.7 % of FA-3d's. Protection costs at most 0.5 % (the gate: 97 %).
- ctest: all 249 tests pass.

**FA-3e, endurance validation (VAL-03) and the first soft rejection (CMD-09).**
- **A route against its flown burn** (the probe `endurance_burn.py`), FA-3's acceptance:
  - Every aircraft flies at its cruise: a straight route of two legs, ten minutes long (for the Crazyflie, 40 % of the seven its battery lasts).
  - Its prediction is read under a reserve of 99.99 %, so that the check reports what the flight needs; then the route is flown with the reserve as it was.
  - All 35 are within 5 %: from −3.7 % (the Mirage 2000, burning less than predicted) to +2.3 % (the Crazyflie). The flights' times are within 4 s of those predicted.
- **The fleet** (`test_fleet`):
  - Every aircraft refuses a timed pattern three times what it lasts, counted in its own energy, and validates it when overridden.
  - It flies five minutes ahead within 5 % over its prediction. On the cautious side, the Mirage 2000 is 5.5 % under: its tanks full, slow on the back of its power curve, where its tables' level points read high.
- **Named changes.**
  - **The B-52H's fuel flow** (FA-3a's tables). hangar had summed the engines the state reports, four at most, and the B-52H has eight. Its three fuel-flow tables double, exactly; nothing else changes (its best speeds, its checks). The performance profile's burn and the navigation report's burn home (its playtime) follow.
  - **The tables' lookup at a speed between rows** (4.13): each row is now read at the same equivalent airspeed. The Su-27S's prediction went from 12.1 % high to 1.9 %. At the rows nothing changes: the profile is the same cell for cell, and no flight reads it.
  - **The conformance tests.**
    - The Crazyflie's route in the catalog walk is overridden (8 km on a battery that lasts 7 minutes).
    - The random walks' model admits `insufficient_endurance`.
    - The walks moved, and met a fourth gap in the model: a timed pattern completes at its duration.
- **Digests:** identical to the speed optimisation's (0c63475), with protection and without. The allocation gate passes.
- **A/B throughput** against the speed optimisation (0c63475), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` three times.
  - The micro cases are within −1.1 % to +0.9 %.
  - A behaviour's NEW read +2.2 % while the host called the model to learn that a hold has no end. The host now asks only for a route, a pattern or a curve, and it reads +0.2 % and +1.0 %.
  - The other command cases are within −4.3 % to +1.7 % (0.4 ns), on paths the check is not on.
  - World throughput is 99.9 to 100.8 % of the speed optimisation's. Protection costs at most 0.2 % (the gate: 97 %).
- ctest: all 250 tests pass.

**FA-4a, the terrain (ENV-01, STS-11, VAL-06).**
- **The checks** (`test_terrain_check`, over a ridge 1,500 m high laid across the way):
  - A route, a pattern, a curve and an hsa into the ridge are refused `terrain_conflict`. Each names the ridge's edge to a tenth of a metre, the time at its planned speeds (within half a second of distance over speed), and the route point or curve segment.
  - Over it, or above the ground, they fly.
  - A route that ends short of the ridge is refused for flying on into it; loitering round its last point, it flies.
  - A validation answers the same. Clamp refuses it too, with no suggestion; unchecked (`RangePolicy::None`), it flies.
  - An UPDATE into the ridge is refused, and the activity flies on as it was: a route's, a pattern's and an hsa's.
  - The query answers the ridge, sea level, and nothing where the ground has no data. The tiles tell no data apart from sea level (`test_terrain`).
  - Python: a descent the aircraft can fly crosses sea level halfway along its leg, as asked. One too steep is clamped to its steepest descent and meets the ground past its point.
- **The fleet** (`test_fleet`): every aircraft refuses an hsa under the ground at once, where it is, and a route to a point under the ground, on its way down. Both are refused with override_rejection set.
- **The walks**: the random-sequence conformance draws altitudes 200 m either side of a multirotor's 100 m and meets `terrain_conflict`; its model admits it for a NEW and an UPDATE.
- **Digests:** identical to FA-3e's, with protection and without; no digest flight goes below the ground. The allocation gate passes.
- **A/B throughput** against FA-3e (02ee658), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` twice.
  - The micro cases are within −1.3 % to +0.5 %.
  - A behaviour's NEW reads −2.4 % and −2.8 %: that baseline, built in another tree, runs it slower (as FA-3c and FA-3e found).
  - The other command cases are within −1.2 % to +2.0 % (0.5 ns), on paths the check is not on.
  - World throughput is 99.2 to 100.1 % of FA-3e's. Protection costs at most 0.1 % (the gate: 97 %).
- ctest: all 254 tests pass.

**FA-4b, the barometric altimeter (ENV-03, STS-10, STS-04, HSA-07, LTR-16).**
- **The air against the flight model's** (`test_altimeter`): in three atmospheres (the standard, 15 K warm at 1020 hPa, 25 K cold at 990 hPa), from sea level to 25 km:
  - the platform's static pressure is within 2.1e-9 of JSBSim's and its temperature within 7.8e-8 K;
  - the isobar of JSBSim's pressure is within 15 µm of the height it was read at.

  The residue is JSBSim's own rounding: its layers' bases are kept in feet to 1e-4 ft, their temperatures in Rankine to 0.01.
- **The altimeter against the standard atmosphere:**
  - at 1013.25 hPa it reads the standard's layer bases from their published pressures (11, 20 and 32 km) within a centimetre;
  - at 995, 1013.25 and 1030 hPa it reads the height between the two pressures, as the standard's troposphere gives it, within a micrometre;
  - set to 1000 hPa, it reads 110.9 m low at every height, within 5 cm (the flight model's gas constant differs from the standard's in its sixth figure).
- **Flown** (a C172 in air 15 K warm, at a sea-level pressure of 1020 hPa):
  - set to 1020 hPa and holding 2,000 m on its altimeter, it reads 1,999.8 m at 2,101.9 m above sea level: the isobar is 102 m up;
  - set to 1000 hPa, it reads 166.9 m less at once, and settles reading 2,000.3 m at 2,278.0 m;
  - its rate matches what its reading does over a second of a climb within 10 %;
  - a reference given alone holds the reading it had;
  - an orbit at 2,300 m on the altimeter settles within 15 m of it.
- **The fleet** (`test_fleet`):
  - at the standard setting, every aircraft's altimeter reads its geopotential height, within 5 cm;
  - set 20 hPa low, every aircraft's hsa climbs on its altimeter as high as the climb case climbs, flying the isobar it reads. The worst is the Mirage 2000, 3.3 m high and still closing; the next the Typhoon, 1.2 m; every rotorcraft within a millimetre.
- **The setting and the refusals:**
  - a QNH outside 850 to 1,100 hPa, or NaN, is refused `out_of_range` and nothing changes; an unknown vehicle is refused;
  - a route's barometric waypoint is refused `not_implemented`, naming the point;
  - in A-GRA's terms, the setting is COMPLETED, or FAILED with its reason.
- **Digests:** identical to FA-4a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-4a (66fc223), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` twice.
  - The micro cases are within −1.4 % to +0.3 %; the command cases within −1.5 % to +1.4 %.
  - World throughput is 99.5 to 100.6 % of FA-4a's. Protection costs at most 0.8 % (the gate: 97 %).
- ctest: all 258 tests pass.

**FA-4c, the state data and reference frames (STS-02, STS-06, ENV-04).**
- **The wind** (`test_state_data`): in a steady wind of 10 m/s, a C172 flying and an IRIS hovering each measure it within 1e-6 m/s; when it drops, they measure none.
- **The fleet** (`test_fleet`): in calm air every aircraft measures no wind. The worst is the Crazyflie, 7.4e-5 m/s: its air data at a few metres a second. Every other aircraft is within 1e-6. Its wander angle is 0, and its orientation's rates and accelerations are given.
- **The orientation's rates and accelerations:** a C172 rolling in, stepped a flight-model step at a time.
  - The Euler rates match the angles' central differences within 1 % of their peak, beyond the difference's own error where the roll jumps in: 0.0104 rad/s, half a step's worth of 2.5 rad/s².
  - The accelerations match the rates' differences within 4 % of theirs.
- **The frames:**
  - Points in fixed, moving and vehicle frames - every rotation and every layout - match the sphere's geometry, worked out in the test, within 1e-12 rad and a micrometre.
  - A vehicle's frame, carried on 10 s, moves 10 s of its ground speed within a centimetre.
  - One whose vehicle is gone answers nothing; a latitude off the Earth, a value not finite and an unknown vehicle are refused.
- **Digests:** identical to FA-4b's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-4b (ffa9bec), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` twice.
  - The micro cases are within −1.8 % to +0.9 %; the command cases within −1.5 % to +0.4 %.
  - World throughput is 99.1 to 100.4 % of FA-4b's. Protection costs at most 0.5 % (the gate: 97 %).
- ctest: all 262 tests pass.

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
