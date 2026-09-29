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
- **Asked for, never stepped.** The report is worked out when asked, from the flight model's tanks, engines and battery. Nothing in the step reads it, so no flight changes - but a route's branch on what it has left or its contingency, which reads its own vehicle's as the branch is decided (since FA-6e2b: 4.37).
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
- **A rotorcraft's airspeeds are along its nose**, as its loops fly an airspeed and its tables have it (4.13). Its most airspeed at each altitude is what its airspeed commands are held to there (4.48).
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

### 4.22 The magnetic model (as FA-4 builds it)

An hsa's heading or course may be measured from magnetic north (HSA-03), and the state data gives the magnetic heading (STS-05), both from the Earth's magnetic field (ENV-02).

- **The model** is the World Magnetic Model 2025 of NOAA's National Centers for Environmental Information and the British Geological Survey, a work of the US Government in the public domain.
  - Its published coefficients (`WMM.COF` of 11/13/2024) are built in, evaluated with the equations its technical report publishes: degree 12, Schmidt semi-normalized, a place's geodetic latitude and height above the WGS-84 ellipsoid turned geocentric.
  - It gives the field north, east and down, its horizontal and total intensity, its declination and its inclination.
- **The date** is the world's UTC as a decimal year, held within the model's five years, 2025.0 to 2030.0. A world whose clock was never set (1970: a C ABI or Python world until its environment's epoch is set) reads the model's epoch. The model itself, asked for another date, is carried on at its rates of change, as NOAA's software does.
- **A magnetic heading or course** (`DirectionReference::MagneticNorth`, the hsa's `direction_reference`) is flown turned by the declination where the aircraft is: true = magnetic + declination. The declination is refreshed every 10 s of simulation time, never every step.
  - A reference given alone holds the heading the aircraft flies now, from that north; a heading or course given alone continues the reference it replaces; in an UPDATE, a reference needs its value.
  - The progress reports the direction from the north it was commanded from.
  - The terrain check turns the hsa's line ahead the same way.
- **The state data** gives the magnetic heading (A-GRA's MagneticHeading) and the declination.
- **Checked** against NOAA's published test values (section 14).
- **Surfaces.**
  - C++: `fsim/Magnetic.h` (`magneticField`, `declinationRad`, `decimalYear`, `magneticYear`); `DirectionReference`, `HsaCommand::directionReference`; `StateData`'s `magneticHeadingRad` and `declinationRad`.
  - C ABI 1.20: the hsa's eighth field and `fsim_direction_reference`; `fsim_state_data`'s `magnetic_heading_rad` and `declination_rad`; `fsim_magnetic_field`, `fsim_magnetic_field_at`, `fsim_decimal_year`, `fsim_world_magnetic_year`.
  - Python: `fsim.DirectionReference`, `submit_hsa(direction_reference=)`; `fsim.StateData`'s new fields; `fsim.magnetic_field` (`fsim.MagneticField`), `fsim.decimal_year`, `World.magnetic_year`; `fsim.agra.HEADING_REFERENCE`.

### 4.23 A-GRA's orbit as its schema gives it (as FA-5a builds it)

A-GRA's orbit (MA_OrbitType) gives what VI-5's pattern does not: a racetrack or a figure-eight by two circles (LTR-03), a fix-point orbit's inbound heading, legs by time and turns by bank (LTR-05; a hold's leg time, LTR-10, is the same), a number of laps (LTR-06), and where the pattern is joined and left (LTR-07).

- **The shape** (`PatternShape`) goes beside the pattern, as a route's waypoints do, and the path store keeps the shape of the pattern that flies. The setpoint every command is kept in stays the size it was: grown by the shape's twelve fields, it made every command's NEW 4 % slower (section 14).
  - In the C ABI and Python the pattern's fields and its shape's are one list, the shape's from index 13; a shape field at fault is named by that index.
- **Another way to give one thing.** Given both, the pattern's own field flies and the shape's stays as it was given; in an UPDATE, either replaces both.
  - A heading for the course: the course it makes good on that heading, in the wind when the pattern is planned.
  - A time for the legs: legs as long as the inbound leg is flown in that time, at the pattern's speed in the wind then.
  - A bank for the radius: the radius at which it banks no more, at its speed plus the wind, as the default radius has it.
  - A course or heading may be from magnetic north (`directionReference`, as the hsa's in 4.22): turned true by the declination at the pattern's point when it is planned. A magnetic course left out is the course now, from that north.
- **Two circles**, a racetrack's or a figure-eight's (an orbit or a hold is refused, field 18):
  - the first round the pattern's point at its radius, the second round its own centre at `radius2M` (the first's if left out);
  - a racetrack's legs touch both circles on the outside, both flown the pattern's way round; a figure-eight's cross between them, the second circle flown the other way;
  - the circles give the course and the legs: given with them, a course, heading, legs or leg time is refused, and a second circle given in an UPDATE replaces them;
  - a racetrack's circles may not lie one within the other, nor a figure-eight's overlap (field 18); a figure-eight's that touch make VI-5's own;
  - joined at the pattern's nearest point, as an orbit is.
- **Laps** (`orbits`): the pattern completes after so many, counted where it was joined; with a duration, whichever comes first. Its progress runs through them.
- **The entry point:** the aircraft flies directly to the pattern's nearest point to it, and its laps are counted from there.
- **The exit point:** its duration or laps flown, the aircraft goes on round to the pattern's nearest point to it, completes there, and flies on straight along its course there at its altitude and speed. Without a duration or laps it never leaves. The terrain check walks that line for a minute.
- **Checked** as the pattern's fields are:
  - the shape's fields whole and in range: a direction reference whole, a bank above 0 and below 90°, laps whole from 0, a point's latitude and longitude together; `InvalidParameter` naming the field;
  - a bank steeper than the aircraft's full bank is clamped to it, the radius with it, or refused `performance_limit` under Reject; a second radius as the first;
  - the endurance check (4.18) counts the laps, and the way in and out.
- **Surfaces.**
  - C++: `PatternShape`; `World::submit(id, PatternCommand, PatternShape)` and `update(activity, PatternCommand, PatternShape)` (and their `Vehicle` and `Caller` forms), `storeTask(..., shape)`, `BatchCommand::shape`, `Setpoint::shape`; `ControlStack::command(PatternCommand, PatternShape)`; `PathStore::pattern`.
  - C ABI 1.21: `FSIM_MODE_PATTERN` takes 25 fields (12 and 13 still), and reads back 25.
  - Python: `submit_pattern` and `Activity.update` take the twelve by name (`fsim.MODE_FIELDS["pattern"]`).

### 4.24 A-GRA's hold as its schema gives it (as FA-5b builds it)

A-GRA's hold (MA_HoldType) gives what a hold on its fix did not: its turns by rate or by type (LTR-11), its duration by entry and exit times (LTR-12), how it is entered (LTR-13) and its context (LTR-14). The shape (4.23) carries them.

- **Turns by rate or type.** A turn rate (`turnRateRadS`): the radius it turns at that rate on, at its speed plus the wind. A type (`HoldTurn`):
  - STANDARD: rate one, at most 25° of bank (a hold's default);
  - MIL_POWER: the tightest the pattern flies, 80 % of the aircraft's bank (an orbit's default);
  - RELAX: half rate one, at most 15° of bank.
  Given more than one of the radius, a bank, a rate and a type, the first flies; in an UPDATE any one replaces them all. A rate faster than the aircraft turns at its full bank is clamped to that (the radius with it), or refused `performance_limit`.
- **Entry and exit times** (A-GRA's EntryExitTime) are the command's time window (4.9): it waits for its start, and completes when its end window closes.
- **Entries** (`HoldEntry`), a racetrack's or a hold's on its fix (not with two circles or an entry point):
  - left out: direct to the fix, then round, whichever way the aircraft comes, as before;
  - DIRECT: where the pattern is nearest, joined as an orbit is;
  - ANCHOR: at the fix, by ATC's entry for the side the aircraft comes from, by its way to the fix against the inbound course (right turns; left mirrored). From 70° left of it to 110° right, ATC's direct entry: to the fix, then round. Beyond 110° right, the teardrop; beyond 70° left, the parallel. These are the AIM's sectors;
  - INBOUND and OUTBOUND: onto the leg's start, on its course, by the shortest turn, straight and turn (Dubins's) from where the aircraft is on its track;
  - PARALLEL and TEARDROP, ATC's: over the fix, then turned onto a leg out, a leg long, and round back to the fix. The parallel's leg runs along the inbound course and ends on the non-holding side; it turns back a half circle toward the holding side. The teardrop's leg runs 30° into the holding side; it turns back 210° the pattern's way.
  - The way in turns at the pattern's radius, tangent where it turns, so the follower flies its curvature: joined at corners, a heavy's legs were flown half a radius off (section 14).
- **The context** (`HoldContext`: ADMIN, TACTICAL, ATC) is accepted and reported. The defaults it implies are ATC's for every context, the only ones implemented (Appendix A, LTR-14): rate-one turns at most 25°, a minute's legs (90 s above 14,000 ft), right turns.
- **Checked:** a turn rate above 0; a type, entry and context among theirs; an entry only a racetrack's or a hold's on its fix without a second circle or an entry point (field 27); a context only a hold's (field 28).
- **Surfaces.**
  - C++: `HoldTurn`, `HoldEntry`, `HoldContext`; `PatternShape`'s `turnRateRadS`, `turnType`, `holdEntry`, `holdContext`.
  - C ABI 1.22: `FSIM_MODE_PATTERN` takes 29 fields; `fsim_hold_turn`, `fsim_hold_entry`, `fsim_hold_context`.
  - Python: `fsim.HoldTurn`, `fsim.HoldEntry`, `fsim.HoldContext`; `submit_pattern(turn_rate_rad_s=, turn_type=, hold_entry=, hold_context=)`, by name or member.

### 4.25 A-GRA's hover and relative points (as FA-5c builds them)

A-GRA's loiter gives two things the patterns did not: a rotorcraft's hover over a point for a duration (LTR-15, MA_LoiterType's Hover), and a pattern's point in a reference frame (LTR-18, its RelativePoint), which moves as the frame does. Both are the pattern's: a hover is a kind of it, and the frame goes in its shape (4.23).

- **The hover** (`PatternKind::Hover`), a rotorcraft's:
  - over its point at its altitude (in its reference), flown by the position loop at its speed there at most, slowing to stop over it. That speed is a ground speed: the pattern's speed, a rotorcraft's cruise by default; an airspeed is taken as its true airspeed in calm air;
  - its duration counts from its arrival: within a metre of the point and 2 m of its height. Then it completes and hovers on; without a duration it hovers until replaced. An UPDATE that moves the point keeps the count from the first arrival; one that makes another pattern a hover starts the count at its arrival;
  - it has its point, its altitude, its speed there and its duration, nothing else. Given a radius or a way to give it, a way round, a course or heading, legs, laps, a second circle, an entry or exit point, or a hold's entry or context, it is refused `invalid_parameter` naming the field, in a NEW or an UPDATE. Its setpoint reads back without them (NaN); an UPDATE that makes another pattern a hover drops that pattern's circuit;
  - its progress: 0 % on its way there, then through its duration (untimed, 100 %); on its way, its time to go is the distance at its speed plus the duration;
  - the endurance check (4.18) counts its way there at its speed, then its duration at the tables' burn in the hover (their least speed);
  - a wing's is refused, naming the pattern field (0), as its support table says:
    - `not_supported` where its design declares it does not fly on rotors (R1, with that evidence);
    - `not_implemented` where nothing declares it and its model does not hover: a stock JSBSim model's, the same answer the hover capability gives for it.
- **Relative points** (`frame`, `frameRotation`, `frameOffsets`, `frameXM`, `frameYM`, `frameZM`): the pattern's point is a point in one of the world's frames (4.21), its offsets as a `FrameOffset` gives them. That point is an orbit's centre, a racetrack's or a hold's fix, two circles' first centre, or a hover's point. Given a z, the pattern flies at the frame's height there (above sea level); left out, at its own altitude.
  - At a NEW or an UPDATE, the host places the point where the frame is then, in the pattern's latitude and longitude (and its altitude, given a z); the setpoint reads it back so. A frame it cannot place - one the world does not have, or a vehicle's whose vehicle is gone - is refused `invalid_parameter` (field 29); offsets without a frame are refused naming the first given (30 to 34).
  - The behaviour places the point again at every update, as the step began: a fixed or moving frame at the world's time (a vehicle's own clock starts at its creation), a vehicle's frame as the world read that vehicle then. The pattern is laid out from the point, so all of it moves with the frame: a second circle and entry and exit points too, placed from the point when the pattern is planned.
  - Flown over the frame where the frame moves. A frame moving steadily is ground like any other: the aircraft flies the pattern at its velocity over the frame (its ground velocity less the frame origin's), through air moving over the frame at the wind less the frame's velocity. A rotorcraft's velocity is given back to it over the ground. A hover over a point a moving frame carries flies the frame's velocity and a closing on the point it could stop closing, as a formation closes on its slot, no faster than its speed there; given a z, it climbs as the frame does.
  - The velocity fed forward is the frame origin's. A point turned with its origin swings as the origin turns; the feedback follows that.
  - A vehicle's frame whose vehicle is gone: the activity fails `target_lost`, and the aircraft flies on as it was (a wing straight and level, a rotorcraft hovering).
  - In an UPDATE a point (a latitude and longitude) replaces the frame, and a frame replaces the point.
- **Surfaces.**
  - C++: `PatternKind::Hover`; `PatternShape`'s `frame`, `frameRotation`, `frameOffsets`, `frameXM`, `frameYM`, `frameZM` and `frameOffset()`; `PathStore::patternFrame`; `vehiclePose()` (fsim/Frames.h).
  - C ABI 1.23: `FSIM_PATTERN_HOVER`; `FSIM_MODE_PATTERN` takes 35 fields.
  - Python: `fsim.PatternKind.HOVER`; `submit_pattern(frame=, frame_rotation=, frame_offsets=, frame_x_m=, frame_y_m=, frame_z_m=)`, the rotation and offsets by name or member.

### 4.26 A-GRA's curve segments as its schema gives them (as FA-5d builds them)

A-GRA gives a curve's segments as NURBS (MA_NURBS_PointType): 4 to 10 weighted control points and 4 to 14 knots, of any degree they make (CRV-03), with a curvature and the first and last control points' indices (CRV-08). The VI's quintic Bezier is one of them. FA-5d1 flies them all.

- **The segment** (`NurbsSegment`): its control points, metres north, east and down from the curve's reference as a Bezier's are, each with its weight; its knots; its degree `knots - points - 1`. The schema asks for curves "terminated at both endpoints", so a segment is a clamped B-spline: its first knot comes as many times as its degree and once more, and its last likewise. It starts at its first control point and ends at its last.
- **Evaluated** by the published mathematics: the B-spline basis and its first two derivatives by the Cox-de Boor recursion (the triangle of knot differences), and a rational curve's derivatives by the quotient rule. Its parameter 0 to 1 runs over its knots' domain. Everything a curve does with a segment - its length tables, the nearest point by Newton steps, the curvature ahead, the checks, the terrain walk - takes its point and derivatives there.
- **A Bezier's form** - six points, weights 1, knots [0 x6, 1 x6] - is flown by Bernstein's basis, as a `BezierSegment` is. The curve notes each segment's form when it measures it, so a Bezier curve flies bit for bit as before, however it was given. A `BezierSegment` is made one exactly (`NurbsSegment::of`); the path store, the waiting store and tasks hold the general form.
- **Well formed**, or refused `invalid_curve` naming the segment:
  - 4 to 10 points, 4 to 14 knots, a degree of 1 or more, every value finite;
  - weights above 0; knots from 0, never decreasing;
  - clamped: its first and its last knot each exactly degree + 1 times;
  - no knot within the domain more often than the degree. More would break the curve there. As often as the degree makes a corner, which is allowed: NURBS's circle joins its quarters so, and the turn checks refuse a corner nothing can fly.
- **Its curvature**, given: the most it turns over the ground, 1/m. A curve turning more than 1 % tighter anywhere is refused `invalid_curve`, naming the segment and the section (`from`, `to`), whatever the policy. The curvature must be above 0.
- **Its indices**, given: its first and last control points, which a clamped curve starts and ends at - 0, and its last. Others are refused as ill formed. The continuity check joins each segment's last point to the next one's first, within a metre, as before.
- **Surfaces.**
  - C++: `NurbsSegment` (`of`, `bezier`, `asBezier`, `degree`); `World::submit(id, CurveCommand, Span<const NurbsSegment>)` and `update(activity, CurveCommand, Span<const NurbsSegment>)` (their `Vehicle` and `Caller` forms), `storeTask(id, task, BatchCommand)`; `BatchCommand::nurbs`; `Setpoint::nurbs` (a curve's every segment; `segments` its Bezier ones, where each is one); `ControlStack::command(CurveCommand, Span<const NurbsSegment>)`. A curve's UPDATE given `{}` for its segments is ambiguous now: give the span's type, or `update(activity, curve)` for its options alone.
  - C ABI 1.24: `fsim_nurbs_segment` (`fsim_nurbs_segment_init`), `fsim_vehicle_submit_nurbs`, `fsim_activity_update_nurbs` (`_as`, `_by`), `FSIM_BATCH_NURBS` and `fsim_batch_command::nurbs` (read only where the caller's struct has it), and `fsim_activity_get_setpoint` answering `FSIM_BATCH_NURBS` for a curve not all of Bezier segments.
  - Python: `fsim.NurbsSegment` (`north`, `east`, `down`, `knots`, `weights`, `curvature`, `first_index`, `last_index`); `submit_curve`, `update_curve`, `append`, batches and tasks take them, beside `fsim.BezierSegment`s; `setpoint()` gives them back; `fsim.agra.flyout_curve` gives A-GRA's form as given.

### 4.27 A-GRA's curve reference and control points as its schema gives them (as FA-5d2 builds them)

A-GRA places a curve by its CenterReference: a geodetic point with an altitude in a reference and an altitude range (CRV-04), or a point in a reference frame (CRV-05). Its control points are offsets from it (CRV-06, RelativeOffset2D_Type): turned as its RotationEnum says, laid out as its OffsetXY_Enum says, their third read as its Z_ChoiceType says. FA-5d2 flies them all.

- **Its reference** (`latitudeRad`, `longitudeRad`, `altitudeM`, as before):
  - `altitudeReference`: its altitude above sea level, above the ground, above the ellipsoid, or barometric (4.22). The curve's heights, down from the reference, are read in it: above the ground, over the ground under the aircraft as it flies; barometric, on the isobar;
  - `altitudeMinM`, `altitudeMaxM`: its range. Left out, the altitude is the aircraft's in the reference, held within the range. Given outside it, it is refused `invalid_parameter` naming the altitude (field 2); a range whose most is below its least is refused naming the most (10);
  - a new curve in an UPDATE given a reference without its altitude is refused naming the reference (8): the UPDATE has no aircraft's altitude to read in it. A NEW takes the aircraft's.
- **In a frame** (`CurveShape`: `frame`, `frameRotation`, `frameOffsets`, `frameXM`, `frameYM`, `frameZM`, as a pattern's point in 4.25):
  - the reference is the frame's point, placed where the frame is at a NEW and again at every update as the step began, so all of the curve moves with the frame. Given a z, the reference is at the frame's height there, above sea level;
  - flown over a moving frame as a pattern is (4.25): its velocity and the wind over the frame, a rotorcraft's given back; with a z, the frame's climb fed forward;
  - a vehicle's frame whose vehicle goes: the activity fails `target_lost`, and the aircraft flies on as it was;
  - refused `invalid_parameter` as a pattern's are, at fields 14 to 19: a frame the world does not have (14), offsets without a frame (the first given).
- **Its points' axes** (`pointRotation`, A-GRA's RotationEnum), turned as its frame is, so only with one (else refused naming the field, 11):
  - `Yaw` (ROTATION_2D): by the frame's yaw; `Heading`: by its track over the ground (its yaw when still). Both turn the plane's axes, not the points;
  - `Attitude` (ROTATION_3D): x, y and z are the frame's body axes (forward, right, down), turned by its roll, pitch and yaw into north, east and down, as a frame's point is (4.21). A NURBS curve is unchanged in form when its control points are transformed alike, weights kept, so turning the points turns the curve exactly: the curve tilts with the frame. A fixed or moving frame's attitude does not change, so its curve is turned once; a vehicle's frame's is turned afresh at every update, as the vehicle turns.
- **Their layout on the Earth** (`pointOffsets`, A-GRA's OffsetXY_Enum):
  - `Cartesian`: in the plane at the reference every local path is laid out in (north along the meridian, east by the latitude's cosine), as before;
  - `GreatCircle`: A-GRA's azimuthal equidistant layout, the one its schema documents for a curve's control points. A point x north and y east of the reference is along the great circle from it on (x, y)'s bearing, as far as (x, y) is long, so a straight line through the reference is a great circle. A bridge from A-GRA's messages gives it; `Cartesian` stays the default, as every local path is laid out;
  - `Rhumb`: north along the meridian, east as the rhumb line's departure (as a frame's point, 4.21).
- **Their third** (`pointZ`, `CurveZ`, A-GRA's Z_ChoiceType): `Down`, metres down from the reference (as before); `AltitudeOffset`, metres up from it; `AbsoluteAltitude`, the altitude itself, in the curve's reference. Each is made metres down from the reference when the curve is planned, so the same curve given the three ways flies alike. Turned in three dimensions, an absolute altitude stays one: only its x and y are turned.
- **Where a curve is changes only with a new curve's segments.**
  - Its reference, range, points' reading and frame, given in an UPDATE of its options alone, are refused `invalid_parameter` naming the first given (0, 1, 2, 8 to 19). Before FA-5d2 such an UPDATE took a latitude, longitude or altitude: the setpoint and the end points read it, but the curve was never flown from it (section 14).
  - Segments appended go on from the curve's reference, their points read as its: A-GRA's append "will use the ownship CenterReference of the preceding curve following command", though its schema gives every segment a CenterReference. Those given with them are not used; the setpoint reads the curve's back.
  - A new curve's segments in an UPDATE place it afresh: a point replaces a frame, and a frame a point.
- **Past its end**, a curve not in the plain layout, or in a frame, flies on (or orbits) from its end on the Earth, its last course turned as its axes are.
- **Checked and reported as flown.** The host plans the curve as the behaviour flies it: placed, read and turned. The turn, climb and terrain checks and the endurance check see that curve, and its end points (4.13) are placed so, the frame as it is now.
- **Surfaces.**
  - C++: `CurveCommand`'s `altitudeReference`, `altitudeMinM`, `altitudeMaxM`, `pointRotation`, `pointOffsets` and `pointZ`; `CurveZ`; `CurveShape` (its six fields, `fields`, `frameOffset`, `empty`); the curve's `submit` and `update`, Bezier and NURBS, in their `Vehicle` and `Caller` forms, take a trailing `const CurveShape*`; `BatchCommand::curveShape`; `Setpoint::curveShape`; `PathStore::curveShape` and `curveFrame`.
  - C ABI 1.25: `FSIM_MODE_CURVE` takes 20 fields. Eight are still taken; the reference's follow them, then the frame's (14 to 19). `enum fsim_curve_z`. A curve's setpoint reads back all 20.
  - Python: `submit_curve`, `update_curve`, `append`, batches and tasks take `altitude_reference`, `altitude_min_m`, `altitude_max_m`, `point_rotation`, `point_offsets`, `point_z` (`fsim.CurveZ`), `frame`, `frame_rotation`, `frame_offsets`, `frame_x_m`, `frame_y_m` and `frame_z_m`, codes by name or member.

### 4.28 A-GRA's circular loiter at a curve's end (as FA-5d3 builds it)

A-GRA ends a curve by its EndOfCurveBehavior: on at its course, speed and altitude (CSA), or "a circular loiter pattern centered around the curve's endpoint" (CIRCULAR_LOITER, CRV-11). A wing orbited the end point; a rotorcraft stopped and hovered over it. FA-5d3 makes a rotorcraft circle it too.

- **Round its end** (`EndBehavior::Loiter` after a curve): each aircraft passes its end at the curve's pace, completes there, and circles the end point in right turns.
  - A wing's radius is as before: its turn radius (80 % of its bank) at its airspeed with the wind behind it.
  - A rotorcraft's is its turn radius at its ground speed: its acceleration's, and no tighter than its velocity loop follows (the orbit's own when its radius is left out, 4.23).
  - A rotorcraft no longer slows to stop at the end: it flies it at the curve's pace, as a wing does, and a duration now brings it there on time at that pace.
- **A route's end is unchanged:** a rotorcraft stops at its last point and hovers there. Routes are FA-6's.
- **The terrain walk** circles a rotorcraft's end at its radius, as a wing's, where it walked the point alone.
- **Surfaces:** none new. `EndBehavior::Loiter`, `FSIM_END_LOITER` and `fsim.EndBehavior.LOITER` say what it does after a curve and after a route. `fsim.guidance.curve/end/circular_loiter` is supported, and with it the curve as a whole.

### 4.29 A-GRA's waypoint as its schema gives it (as FA-6a builds it)

A-GRA's route segment ends at an end point that is one of three: a WayPoint ("a point in the route where no turning occurs", with its WaypointType), a TurnPoint (its type, a course and a turn's geometry) or a LoiterPoint. Each is a geodetic point with an altitude, its reference and an altitude range, or a point relative to a reference frame. FA-6a builds the altitude range and the barometric reference, the waypoint and its type, and points in frames (WPT-12, WPT-17, WPT-22). FA-6b builds the turn points' other types and loiter points.

- **Its altitude block** (`altitudeMinM`, `altitudeMaxM`, in the point's reference): an altitude left out is held within it - the point before's, or the aircraft's for the first. One given outside it, or a block whose most is below its least, is refused `invalid_waypoint` naming the point.
- **A barometric altitude** on a route is flown on its isobar, as the hsa's (4.22): the behaviour reads it through the altimeter at every update, and the host's checks compare heights above sea level, the terrain walk too. Until FA-6a a route refused it `not_implemented`.
- **Its kind** (`kind`, `EndPointKind`): left out, a turn point as `turn` says, as before. A waypoint is flown over, its next leg joined after it. A loiter point flies the loiter beside it (4.31); until FA-6b2 it was refused `not_implemented`, naming the point.
- **A waypoint's type** (`waypointType`, `WaypointType`, A-GRA's WaypointTypeEnum): given alone, it makes the point a waypoint; given with another kind, it is refused `invalid_waypoint`.
  - Nav only and passive are flown. The end of a path is taken at its path's last point: the route's last until FA-6e1, then its path's as given (4.36) - a route without paths is one, and a point whose next is -1 ends it there. Where its path goes on it is none, refused `invalid_waypoint` naming the point (until FA-6e1, `not_implemented`).
  - The others ask for an action FA does not fly yet. Each is answered as its row in the support table says, naming the point: `not_supported` where the aircraft cannot, else `not_implemented`. They are a taxi's points (FA-9, where the aircraft taxies), a runway's and a takeoff's (FA-9), an approach's and a touchdown (FA-10), and a hard ditch (FA-16).
- **A point in a frame** (`frame`, `frameRotation`, `frameOffsets`, `frameXM`, `frameYM`, `frameZM`, as a pattern's point in 4.25): its latitude and longitude are where the frame puts it, and those given are not read. Given a z, its altitude is the frame's there, above sea level.
  - Refused `invalid_waypoint`, naming the point: a frame the world does not have; its fields out of range; offsets without a frame; a seventeenth frame in one route (the path store keeps 16).
  - The host places the points at the NEW, where the frames are then; its checks (turns, gradients, terrain, endurance) see the route so placed. The path store keeps each frame as it was then; a moving one goes on at its velocity from there.
  - A fixed frame's points stay where they were placed. A moving frame's, or a vehicle's, are placed again as the route is flown: at every update, as the step began, the point flown to and those either side of it. The legs into and out of the point flown to, and its fly-by turn, are planned again from where they are (`route::replan`): the entry keeps its start, and the turn keeps its share of the legs as `plan()` gives it.
  - A leg whose two ends are in one moving frame is flown over the frame: the aircraft's velocity and the wind less the frame's, a rotorcraft's given back, as a pattern's (4.25). Any other leg is flown over the ground, so a leg to a moving point pursues it.
  - A rotorcraft that loiters at a route's end over a moving point hovers over it as the hover pattern does: the frame's velocity, and a closing it could stop.
  - A vehicle's frame whose vehicle is gone: the activity fails `target_lost`, and the aircraft flies on as it was.
  - The progress measures along the legs as they are when flown. The end points report each point where its frame is now.
- **Surfaces.**
  - C++: `Waypoint`'s `altitudeMinM`, `altitudeMaxM`, `kind`, `waypointType`, `frame`, `frameRotation`, `frameOffsets`, `frameXM`, `frameYM`, `frameZM` and `frameOffset()`; `WaypointType`; `PathStore::routeFrames`, `routeFrameIds`, `routeFrameCount`.
  - C ABI 1.26: `fsim_waypoint`'s ten new fields, read where the caller's `struct_size` has them (`fsim_waypoint_init` leaves them out); `enum fsim_waypoint_type`.
  - Python: `fsim.Waypoint`'s ten new fields, codes by name or member; `fsim.WaypointType`.

### 4.30 A-GRA's turn points as its schema gives them (as FA-6b builds them)

A-GRA's TurnPoint gives a turn type (TURN_SHORT, FLY_OVER, CAPTURE_OUTBOUND_COURSE, START_TURN, END_TURN), a course at the point, and a turn's geometry (a radius or a bank). Its schema says only that turn points "will generally come in groups of at least two in linked PathSegments", all but the last with a radius; its ICD and its VI volume say nothing more. FA-6b takes the geometry from the published leg type the groups describe (ARINC 424's radius to fix) and from what each type's words ask (WPT-04).

- **Its type** (`turn`, `TurnType`): a fly-by (TURN_SHORT) and a fly-over as before, and:
  - **a capture** (`CaptureOutboundCourse`): the point is flown over and its course (`courseRad`, required) captured. The leg out of it is that course, so the next point must lie along it within a degree; else it is refused `invalid_waypoint`, naming the point;
  - **a start** (`StartTurn`): an arc begins at the point and runs to the next, ARINC 424's radius to fix. It leaves the point on the point's course, or, left out, on the course the leg into it arrives on (the entry's, at a route's own start). It is the circle tangent to that course there through the next point: its sweep is twice the chord's angle from the course, its radius half the chord over that angle's sine;
  - **an end** (`EndTurn`): the arc from the point before, which must be a start, ends here, and the route goes on straight. A course given must be the arc's there, within a degree.
- **An arc's checks.** A radius given (`turnRadiusM`, A-GRA's TurnGeometry) must be the arc's, within a metre or half a percent. An arc sweeping more than 170 degrees is refused too: give a start between. Both are refused `invalid_waypoint`, naming the point the arc does not reach. An arc the aircraft cannot turn at the next point's speed and its full bank (a rotorcraft's, its tilt) is refused `invalid_waypoint` naming its end, whatever the policy: no clamp moves a point.
- **Corners.** Where an arc's end is not tangent to the leg beside it (a course given across the leg in, or a leg out across the arc's end), the corner is flown over: the aircraft reaches the point, then turns.
- **A fly-by's radius** (`turnRadiusM`): its turn's radius, in place of the one its speed and bank give. One tighter than its full-bank turns at its speed is clamped to that (flagged), or refused `performance_limit` under `RangePolicy::Reject`. A bank is a fly-by's `maxBankRad`, as before.
- **Refused `invalid_waypoint`, naming the point**, for what does not make a turn point: a course where its type has none (a fly-by's, a fly-over's); a radius where it has none (a fly-over's, a capture's, an end's); a capture without its course; a start with no point after it; an end not after a start; a waypoint (no turn) given a turn type; a type that is not one.
- **Flown as a leg.** An arc is a leg (`route::Leg` holds it), so it is flown, measured and walked as one. Its length is its arc's, and its courses at its ends are the arc's tangents. The follower flies it as it flies a turn, its curvature fed forward, and looks ahead to the curvature after it: a rotorcraft slows in time for an arc and flies it no faster than it allows. The terrain walk walks it round its centre.
- **Surfaces.**
  - C++: `TurnType::CaptureOutboundCourse`, `StartTurn`, `EndTurn`; `Waypoint::courseRad`, `turnRadiusM`; `route::makeArc`.
  - C ABI 1.27: `FSIM_TURN_CAPTURE_OUTBOUND_COURSE`, `FSIM_TURN_START_TURN`, `FSIM_TURN_END_TURN`; `fsim_waypoint`'s `course_rad` and `turn_radius_m`.
  - Python: `fsim.TurnType`'s three; `fsim.Waypoint`'s `course_rad` and `turn_radius_m`.

### 4.31 A-GRA's loiter points inside a route (as FA-6b2 builds them)

A-GRA's third end point, the LoiterPoint, gives a loiter - an orbit, a hover or a hold (MA_LoiterType), each as the loiter command gives it - and a time it ends. Its schema says only that it "specifies a Loiter"; its ICD, that waypoint behaviour may include a loiter or a hold. Neither says where a loiter in a route begins, nor how the route leaves it. FA-6b2 flies it as a flight management system flies a hold or an orbit on a route (WPT-18).

- **Its loiter** (`RouteLoiter`, beside the route's waypoints as they go beside its RouteCommand; 16 a route at most): at the waypoint `point`, of kind `EndPointKind::LoiterPoint`, a `PatternCommand` and its `PatternShape` - every pattern FA-5 flies: an orbit, a racetrack, a figure-eight, a hold with its ways in and turn types, a rotorcraft's hover, two circles, laps, entry and exit points - and `endTimeS`, A-GRA's EndTime, in simulation seconds.
- **Its place is its point's.** The loiter takes the point's latitude, longitude, altitude, altitude reference and frame; any of its own is refused `invalid_waypoint`, naming the point. Its speed left out is the point's segment's. A point in a moving frame carries its loiter with it (4.25).
- **Checked as a pattern NEW is**, at its point:
  - what it leaves out is filled in as for an aircraft arriving there along the leg in: the course it tracks, and the radius its speed, the wind and 80 % of its bank give;
  - its speed and radius are limited as a pattern's are;
  - a field a pattern refuses is refused `invalid_waypoint`, naming the point;
  - a hover where the aircraft does not hover, and an optimisation with no tables, are refused as a pattern's are (`not_supported`, `not_implemented`), naming the point;
  - a limit names the point, and the loiter's field numbered from 100, past the waypoint's own with room left for them: the pattern's fields are 100 to 112, its shape's 113 to 134, its end time 135 (a radius held to the full bank's: field 105).
- **Refused `invalid_waypoint`, naming the point:** a loiter point without a loiter; a loiter at a point that is not a loiter point, or past the route; two loiters for one point; an end time not finite.
- **Its end:**
  - its duration or its laps flown (then on round to its exit point, if it has one), or its end time, whichever comes first;
  - a loiter with no end is the route's end: only its last point's, in a route that does not repeat, else refused `invalid_waypoint`;
  - an end time already past ends it as it begins.
- **Flown.** A loiter point has no turn of its own: a fly-by or a fly-over flies over it, and the other turn types, or a turn radius, are refused. The leg into it is flown until its pattern takes the aircraft on:
  - an orbit (or two circles, the first), a radius outside its circle. It is entered along the tangent from there, and joins the circle on its course;
  - a hold, a racetrack or a figure-eight, at its point, which is on the pattern. It is flown into as a pattern from there: a hold by its way in, direct or ATC's for the side the aircraft comes from;
  - a rotorcraft's hover, where the aircraft would stop from its speed, its velocity loop's lag counted. Its position loop then makes the approach, as the hover pattern's does, but its nose held on the course the leg arrives on.
  On the leg, a rotorcraft slows for the loiter's radius, or to stop for a hover. Where a turn just before a hover leaves less than its stopping distance, it slows on the turn, and before it as far back as the stop reaches: no faster than stops it at the point, along what is left of the route.
- **Left** for the next point, unless its exit point is given: an orbit where its circle's tangent runs to the next point; a hold, a racetrack or a figure-eight at its point, as a hold is left at its fix. The leg to the next point then runs from where the aircraft left, as the route's entry runs from where the route began, and the turn at the next point is planned again.
- **The last point's** loiter: the route completes at its end (with no end, as it begins), and the pattern flies on, round or out along its exit. The route's `end` does not apply.
- **The route's own end stop** (`EndBehavior::Loiter`, a rotorcraft's, its last point no loiter point) is flown as a hover's too, since the task FA-6b2 raised (section 14). It is handed to the position loop where the aircraft would stop from its segment's speed, and the path slows for it with the velocity loop's lag counted. The position loop hovers it over the point, no faster than that speed. The route still completes within a metre of its point along its last leg, then hovers on. Handed over within a metre, as it was, a UH-1H swung 42 m past its point.
  - A turn just before the point that leaves less than the stopping distance slows the aircraft as it does for a loiter point's hover (above), since the task the end stop raised (section 14).
  - Both stops hold the nose on the course the leg into the point arrives on. The position loop turns a nose to its point from beyond 20 m of it, as the hover pattern's still does. Coming out of a turn off its leg, a UH-1H's nose swung up to 75° with that bearing, its velocity loop's trims with it, and it swung past its point.
  - A last leg that ends where the aircraft is (4.38: an altitude's) is not stopped from where the aircraft would stop: it ends where it ends, perhaps past its point. The position loop then takes the aircraft over the point as before, its nose turned to it - not held on the leg, which would fly it back tail first.
  - A UH-1H still leaves a fly-by turn at its cruise some 25 m off its leg, and passes a point just after it up to 22 m to its side (section 14).
- **Progress** while it loiters:
  - its point's segment, flown (100 %);
  - the pattern's cross-track, course, altitude and speed;
  - the time to go: the pattern's (its duration's, its laps', its end time's) and the legs after it.
  Its end point is a `LoiterPoint`.
- **The checks:** the terrain walk flies each loiter's way in and a lap from where it begins, at its point's altitude, naming the point. The endurance adds each loiter's duration, laps or time to its end time from its arrival, whichever is least; its laps with its way in from where it begins and on round to where it is left - an orbit's from two radii out, or from where a leg shorter than that begins (section 14, FA-6g2's finding closed).
- **Kept and read back** complete, but with its place left out, since that is its point's: a loiter read back and submitted again is taken again. An UPDATE with new waypoints takes their loiters; its options alone keep both.
- **Surfaces.**
  - C++: `RouteLoiter`; `PathStore::routeLoiters`; `BatchCommand::loiters`, `Setpoint::loiters`; the World's route `submit` and `update` take a `Span<const RouteLoiter>`; `ControlStack::command(route, waypoints, loiters)`.
  - C ABI 1.28: `fsim_route_loiter` (its point, the pattern's 35 fields, `end_time_s`) and `fsim_route_loiter_init`; `fsim_vehicle_submit_route_loiters`, `fsim_activity_update_route_loiters`; `fsim_batch_command`'s `loiters` and `loiter_count`, which `fsim_activity_get_setpoint` fills.
  - Python: `fsim.RouteLoiter(point, <the pattern's fields>, end_time_s)`; `submit_route(..., loiters=)`, `update_route(waypoints, loiters=)`, and a batch's or a task's `loiters=`; read back in the setpoint's `loiters`.

### 4.32 A-GRA's segment performance (as FA-6c builds it)

A-GRA's path segment gives its speed as a value in a reference or as an optimisation (PathSegmentSpeedType), its climb as a rate or as an optimisation (ClimbType), and an acceleration in m/s² (MA_PathSegmentType.Acceleration). Its schema says an optimisation lets the vehicle "vary its speed autonomously", and that with a climb optimisation the vehicle climbs or descends "at an autonomously chosen rate": BEST_RATE at "the best rate achievable by the vehicle", EXTENDED_RANGE "as efficiently as possible". It does not say when the acceleration applies. FA-6c takes it as the rate of the speed change into the segment's speed. FA-6c1 builds the speed optimisation and the acceleration (WPT-06, WPT-10); FA-6c2 builds the climb optimisation (WPT-08).

- **A segment's speed optimisation** (`speedOptimization`, `SpeedOptimization`) flies the segment at the performance tables' best speed, as an hsa's (4.17):
  - the host resolves it as the route is given: the optimum's true airspeed at the point's altitude and the fuel on board. It replaces the point's speed, in true airspeed, and the checks (turns, gradients, terrain, endurance) judge that speed. The setpoint keeps the optimisation and the speed resolved;
  - the behaviour flies the optimum afresh each update, at the altitude and the weight now; above the altitudes the tables fly, the speed resolved. A rotorcraft flies it as it flies any airspeed on a route: the ground speed it makes along the path in the wind;
  - a point that leaves out its speed continues the point before's, and with it its optimisation. A speed given replaces it;
  - without tables - a stock JSBSim aircraft - it is not implemented: refused `not_implemented`, naming the point, as its support rows say (`fsim.guidance.route/speed/long_range_cruise`, `.../max_endurance`);
  - one that is not an optimisation is refused `invalid_waypoint`, naming the point.
- **A segment's acceleration** (`accelerationMs2`, above 0) sets the rate of the speed change into the segment:
  - the speed flown starts at the aircraft's speed as the segment begins: its true airspeed, or its ground speed where the segment's speed is over the ground. It runs at the acceleration to the segment's speed, or to its optimum as it is now, then holds it. Left out, the loops change the speed as they did before;
  - the progress reports the speed flown now, the ramp's;
  - refused `invalid_waypoint`, naming the point: 0 or less, or not finite;
  - held to what the aircraft can, where that is known, judged from the point before's speed (the first point's from the aircraft's now) at the point's altitude and the fuel on board. A rotorcraft's is its performance (`maxAccelerationMs2` faster, `maxDecelerationMs2` slower). A wing's comes from its tables: g Ps / V, with full power's excess power faster and idle's slower, the least at nine speeds through the change within the level speeds. The tables read each altitude row at one equivalent airspeed, so a speed at the slowest level speed can fall outside the row below; such a speed is passed over. An acceleration beyond that is held to it, flagged, and named by its point and field 24 (`MaxAcceleration`, `MinAcceleration`); under `RangePolicy::Reject` it is refused `performance_limit`. A wing without tables has no known limit, and its acceleration is taken as given;
  - the loops fly the ramp as they fly any speed setpoint. A wing's airspeed loop follows with its lag, and catches up on what it lost as the ramp began: a C172 still settling from its start fell 0.9 m/s behind and closed with a 14 s time constant. In a 40 s ramp's middle the fleet's wings slowed 9 to 13 % faster than asked, and its rotorcraft reached their cruise within 15 % of the rate asked (section 14).
- **A segment's climb optimisation** (`climbOptimization`, `ClimbOptimization`) is A-GRA's choice beside a climb rate: a point with both is refused `invalid_waypoint`, naming it. The segment's speed is its own; the optimisation chooses how its altitude changes:
  - **the rate:** the most the aircraft climbs or descends holding the speed it flies. Climbing, it is its tables' full power excess power (at a constant speed the height changes at the excess power's rate); descending, a wing's idle's. Where the tables do not read the speed flown (outside the level speeds there, or at a row's edge), it is read at the nearest speed they do. A rotorcraft's tables have no idle: it descends at its guidance's most. Either is held within the guidance's most, which it is where the tables read nothing at all, and read afresh each update at the altitude, speed and fuel now;
  - **BestRate** changes the altitude at that rate from the segment's start, then holds it;
  - **ExtendedRange** climbs at that rate too, timed by the tables. A full-power climb costs the same fuel wherever along the segment it is flown, and the rest of the segment is level flight, whose fuel per metre at the speed flown the tables give at each altitude. So it climbs at once to the altitude between the two ends where that is least (the ends and the tables' rows between; ties toward the point's), holds it, and makes the rest of the climb at the end. That rest starts where it needs the distance left at the rate halfway through it, and as long again as its altitude loop lags (the vertical speed law's time constant). Where the higher is the cheaper it flies as BestRate: at a fixed speed the most climb is then also the most efficient. Where the lower is (the C172 at 40 m/s), it climbs late;
  - **an efficient descent** is along the segment's gradient, as without an optimisation. Measured on five aircraft, a late descent at idle cost 0.6 to 7.0 % more fuel than one along the gradient on four, and 2.3 % less on the C-130J (section 14). At idle an engine burns fuel it does not turn into flight, and the tables have no idle fuel flow to weigh that by;
  - without tables (a stock JSBSim aircraft) it is not implemented: refused `not_implemented`, naming the point, as its support rows say (`fsim.guidance.route/climb/best_rate`, `.../extended_range`);
  - a segment too short for its change at its rate goes on changing past the point, as a climb rate's does, and the gradient check passes it by;
  - **checked:** the terrain walk flies the lowest profile it could fly. That is a best rate climb at the least rate its tables give through the change, an efficient climb from the latest it could start (at the most), and a best rate descent at the most. So an efficient climb toward terrain may be refused where a best rate one clears it;
  - read back as given, its climb rate left out.
- **Surfaces.**
  - C++: `Waypoint::speedOptimization`, `climbOptimization`, `accelerationMs2` (its fields 22 to 24); `ClimbOptimization`.
  - C ABI 1.29: `fsim_waypoint`'s `speed_optimization`, `climb_optimization` and `acceleration_ms2`, read where the caller's `struct_size` has them (`fsim_waypoint_init` leaves them out); `enum fsim_climb_optimization`.
  - Python: `fsim.Waypoint`'s three, codes by name or member; `fsim.ClimbOptimization`.

### 4.33 A-GRA's required time of arrival (as FA-6d1 builds it)

A-GRA's path segment gives a required time of arrival at its end point (MA_PathSegmentType.RequiredTimeOfArrival, a TimeWindowType): "a constraint on the timing of the waypoint", which "allows the operator to see a delta between the required and estimated arrival times that would allow for speed up/slow down determinations". A TimeWindowType is a range (a begin and an end; an end alone starts now, a begin alone runs on) and a duration. FA-6d1 builds it (WPT-11); FA-6d2 builds the planned inertial states and FA-6d3 the required navigation performance.

- **Its window** (`arrivalBeginS`, `arrivalEndS`, the Waypoint's fields 25 and 26): when it is to arrive at the point, in the world's simulation seconds, as a loiter's end time. Either side left out is open: a begin alone, no earlier; an end alone, no later. A duration is the caller's to add to its begin. Arriving is reaching the point's segment's end, its turn's middle, where the progress's segment moves on.
- **Refused `invalid_waypoint`, naming the point:** a time not finite; a begin after its end; an end already past.
- **Its speed scheduled** to the next point with a window (a repeating route's are its first lap's):
  - the distance to it along the route, as the route measures its segments;
  - its arrival at the speed planned: its segment's, over the ground in the wind now along its course;
  - within the window, flown as planned. Beyond it, from then until the point, the ground speed that arrives a quarter of the window's width inside it (5 s at most, and 5 s past a side given alone; a point window, at it), re-planned each update, so it closes on its own error. In its last second it holds the speed it asked;
  - within the speeds it flies level: a wing's from 1.2 times its envelope's least up to its tables' fastest at the altitude and fuel on board, less 3 %, and its most. A wing's tables' slowest level speed is narrower than what it flies (a Typhoon at 139 m/s where they read 168), and energy management begins to act at 1.1 times its least. A rotorcraft's is from a metre a second up to its fastest over the ground;
  - since FA-6d2, a climb steeper than the aircraft climbs at its fastest level speed is flown no faster than it climbs it, and the rest paced to make up for it (4.34).
  The point after the window is flown at its own speed again.
- **Checked:** the time it would arrive at the fastest and the slowest of those speeds (in calm air) against the window: since FA-6d2, from the earliest and the latest it can be at the timed target before it - a window, a planned state's time - the first from now, its climbs no faster than it climbs them (4.34). A window it cannot make is refused `performance_limit` naming the point, whatever the range policy: `MaxAirspeed` where even its fastest arrives after it, `MinAirspeed` where even a wing's slowest arrives before it. Unchecked (`RangePolicy::None`), it is flown at its limit.
- **Through a loiter** (FA-6g2):
  - **A window at a loiter point** is met where its loiter begins, the leg flown to where it meets the loiter (4.31).
  - **Past one,** the schedule counts the loiter's own time from when it begins: the first of its ends to fall due - its laps flown, its duration, its end time - then on round its pattern to where it is left, where the leg on begins (an orbit's exit toward the next point, a hold's fix). An orbit due between its exits flies on to the next. The leg on is measured from where the loiter is left: an orbit's exit tangent is nearer the next point than its centre is. A rotorcraft's hover counts its duration from its arrival over its point, taken to slow evenly from where it began to stop. One speed is scheduled for the legs before the loiter and after it; the loiter flies its own, and the legs after it make up what it took. A loiter's laps and duration take the same time whenever it begins. For an end time, where the aircraft will be round its lap at that time is known only once the loiter has begun: until then the loiter is taken to be left at its end time, and the legs after it make up the rest of the way to its exit.
  - **A loiter whose end is not known ahead** - a hold to an altitude, or one the operator ends (4.38) - is not scheduled through: the legs before it are flown as planned, no estimate is given, and the schedule begins where the loiter ends.
  - **The estimate through a loiter it flies:** what the loiter has left, as it began (its way in, its laps, its exit), then the legs on at the speed last asked (the next point's, where none was), and the loiters after it. A hover's duration is known once it is over its point; until then, at least its duration is left.
  - **How near the estimate comes** is the loiter's pieces' measure against how the aircraft flies them (section 14, FA-6g2):
    - where the route joins an orbit two radii out, within about a second as planned (the B-52H, C-130J and F-16C); scheduled, every aircraft of the fleet arrived within 0.06 s of its aim;
    - a route that begins inside its orbit's join turns onto its way in more sharply than it can: the E-3G, KC-46A and C-17A, 1.45 radii out, took 10 to 12 s longer, and missed a window after it by what the legs after could not make up (up to 6.4 s). A C172 on a 206 m orbit took 6.6 s longer, rolling into and out of it;
    - through a hover, the Crazyflie and the IRIS came within 0.7 s. The UH-60A and UH-1H took 23 and 72 s longer to settle over the point: their position loops close slowly from where they would stop (FA-6b2 found the UH-1H over its point 95 s into its loiter).
    The host's check measures a loiter as its pieces do.
  - **Checked** as before, each window from the earliest and the latest the aircraft can be at the timed target before it, a loiter's own time between (one whose end is not known ahead, from none to without end).
  - Until FA-6g2, a window at or after a loiter point was not implemented: the time a loiter takes is its end time's to set (4.31).
- **Not implemented** (as its support row, `fsim.guidance.route/required_time_of_arrival`, says): an aircraft without performance tables, whose speeds it would have no floor for (a stock JSBSim aircraft), as the speed optimisation's is.
- **Reported:** the estimated arrival at the next point with a window (the aim, once scheduled; a planned state's time, 4.34), and that against its window: + late, − early, 0 within (to a microsecond). C++ asks the route for it (`World::activityArrival`, an `ArrivalEstimate`), apart from the progress: every activity record carries a progress, and the host holds 26 records a vehicle, so 16 bytes more there cost a level switch's NEW 9 % (section 14). The C ABI's and Python's progress carry it, NaN where there is none. The speed the progress reports is the ground speed its schedule asks.
- **Surfaces.**
  - C++: `Waypoint::arrivalBeginS`, `arrivalEndS`; `World::activityArrival` and `ArrivalEstimate`; `Behavior::arrival`.
  - C ABI 1.30: `fsim_waypoint`'s `arrival_begin_s` and `arrival_end_s`, and `fsim_activity_progress`'s `arrival_s` and `arrival_delta_s`, each where the caller's `struct_size` has them.
  - Python: `fsim.Waypoint`'s `arrival_begin_s` and `arrival_end_s`; `fsim.ActivityProgress`'s `arrival_s` and `arrival_delta_s`.

### 4.34 A-GRA's planned inertial states (as FA-6d2 builds them)

A-GRA's path segment carries planned inertial states (MA_PathSegmentType.InertialState, an InertialStateRelativeType, any number of them): "detailed inertial state information at user defined increments within a path segment", data a route's pre-mission analysis may need. Each is a place - a point, or one relative to a reference frame - with the time it is to be reached, and the velocities, acceleration, orientation and rates planned there. ADR-29 plans them as the segment's reference trajectory (WPT-20); FA-6d2 builds that.

- **A state** (`RouteState`, beside the waypoints as the loiters are; 64 a route at most, in order along it):
  - its segment, `point`: the one ending at that waypoint;
  - its place: a latitude and longitude, or a place in a frame (a frame and its offsets, as a waypoint's) where the frame is at the state's time - a fixed frame's place, a moving one's at its velocity, a vehicle's where its velocity now carries it; without a time, where it is now;
  - its altitude, in its point's reference (a reference left out takes it), and its time (`timeS`, the world's simulation seconds). Either may be left out;
  - the rest A-GRA gives - its position's uncertainty, its ground velocity, its velocity through the air and in its frame, its acceleration, its orientation and its rates - kept and read back.
- **Flown** on the first lap (a repeating route's later laps fly their segments as without them):
  - its altitude: the segment's profile runs straight from where it climbs from through each state's altitude, at its place along the leg, to its point's, in place of its gradient;
  - its time: a window of none at its place (4.33). The next timed target is the nearer of the next point with a window and the next timed state not yet passed, and its schedule aims at the state's time;
  - a climb: where a piece of the first lap's profile - a segment's gradient, or a piece between its states' altitudes - is steeper than the aircraft climbs at its fastest level speed, the fastest speed it climbs it at is found from its tables' full power excess power (4.32's rate), down from that speed. The schedule flies no faster there, and paces the rest to make up for it: the ground speed that arrives on time with its climbs no faster. This holds for a route with a time to arrive at; a segment at a climb rate or optimisation, or above the ground, has none. First scheduled at one ground speed throughout, a C172 climbing 150 m to a state held 49.4 m/s at full power while its schedule asked up to 59, and passed it 1.4 s late (section 14).
- **Refused `invalid_waypoint`, naming the point:**
  - 65 or more;
  - on a segment past the route, or one its first lap does not fly (before its start);
  - out of order along the route;
  - a field not finite, or a code not whole;
  - no place (neither a latitude and longitude nor a frame), or a frame the session has not;
  - an altitude in another reference than its point's;
  - a time past, or not after the one before;
  - an altitude beside a climb rate or a climb optimisation: two profiles;
  - checked (a range policy given), a place off its leg as planned by more than its uncertainty (50 m at least, or 1 % of the leg), or behind the state before it on the leg.
- **Refused `performance_limit`, naming the point, whatever the range policy:**
  - a piece of its profile steeper than the aircraft climbs or descends (`MaxClimbRate`, `MaxDescentRate`): at its point's speed, or between two times in the time between them;
  - a time it cannot make (`MaxAirspeed`, `MinAirspeed`), as a window's (4.33).
- **Checked** by the terrain walk too: its first lap through its states' altitudes.
- **At and after a loiter point** (FA-6g3a):
  - A state on a loiter point's own leg is flown before its loiter; one past where the loiter begins (4.31) is refused `invalid_waypoint`, naming the point.
  - The leg after a loiter point is flown from where the loiter is left (4.31): an orbit's exit toward the next point, a hold's fix, a hover's point. A state on it is placed on that leg, where it will be laid, and placed again on the leg as laid once the loiter is left. It is checked against that leg, within its uncertainty, as any state is on its own.
  - Its time is scheduled through the loiters on its way, as a window's is (4.33): each loiter's own time counted, the legs' speed paced round it.
  - Until FA-6g3a, a state at or after a loiter point was not implemented.
- **Beside points in moving frames** (FA-6g3b): its legs move with their points (4.29).
  - A state with a time is checked on its leg where the leg will be at that time, its ends where their frames carry them - a moving frame's at its velocity, a vehicle's where its velocity now carries it - within its uncertainty. One without a time is checked where the leg is now.
  - In flight it is placed again on its leg as the leg is planned again, each update.
  - Until FA-6g3b it was not implemented.
- **Partial** (as its support row, `fsim.guidance.route/inertial_states`, says, where the aircraft has no performance tables): a timed state there is not implemented, as a window is (4.33). With tables the row is supported, since FA-6g3b.
- **Kept** as the loiters are: an UPDATE's new waypoints come with their states, and without new waypoints it keeps its own. A route kept waiting (disabled, unassigned) resumes at the point it flew to, with the states beyond it. Reset, it flies all its states again, those it flew past too (since FA-6e1: 4.36).
- **Reported:** the arrival estimate (4.33) is the next timed target's, a state's time too. The setpoint reads its states back as placed: one in a frame with its latitude, longitude and altitude where the frame was at its time.
- **A route's times taken together** (a named change to 4.33): FA-6d1 checked each window from now alone. A route's windows and its states' times are now checked in turn along its first lap, each from the earliest and the latest the aircraft can be at the one before, and its climbs no faster than it climbs them. So two windows it could make each alone, but not both, are refused (`MaxAirspeed` at the second). The first is checked as before.
- **Surfaces.**
  - C++: `RouteState` (`fsim/Control.h`); a `Span<const RouteState>` beside the loiters in `World::submit` and `World::update` (a route's), `BatchCommand::states`, `Setpoint::states`, `ControlStack::command`; `PathStore::routeStates`.
  - C ABI 1.31: `fsim_route_state` (`fsim_route_state_init`: every field left out), its 29 fields in `RouteState`'s order (`fsim/fsim_c.h` lists them); `fsim_vehicle_submit_route_states`, `fsim_activity_update_route_states`; `fsim_batch_command`'s `states` and `state_count`, where the caller's struct has them, filled by `fsim_activity_get_setpoint`.
  - Python: `fsim.RouteState`, by name or member; `states=` beside `loiters=` in `submit_route` and `update_route`, and in a batch's or a task's `BatchCommand("submit_route", ...)`; read back in the setpoint's `states`.

### 4.35 A-GRA's required navigation performance (as FA-6d3 builds it)

A-GRA's path segment gives a required navigation performance (MA_PathSegmentType.RequiredNavigationPerformanceInMeters, a distance): "the navigation performance accuracy required in an airspace", in metres rather than RNP's nautical miles. ADR-29 plans it as a monitor (WPT-21): the cross-track against it, reported and alerted. FA-6d3 builds that.

- **Its RNP** (`rnpM`, the Waypoint's field 27): how far off its path its segment may be flown, in metres. Left out, the segment has none; unlike most of a point's fields, it does not continue the previous point's (an airspace's RNP is the segment's own).
- **Refused `invalid_waypoint`, naming the point:** an RNP not above 0, or not finite.
- **Monitored** each control update, on every lap:
  - the route's cross-track against the RNP of the segment it flies. Its path is its own: its legs, its fly-by turns' arcs and a start turn's arc (4.30), so a fly-by cutting its corner is on it;
  - the platform navigates exactly: its total system error is its flight technical error, the cross-track the progress reports;
  - not in a loiter point's loiter, which is no segment, nor past the route's end.
- **Alerted:** farther off than its RNP, the route's behaviour says so (`Behavior::constraints`, `kActivityNavigationPerformance`). Its activity's record carries the flag for each world step in which it was (`constraints`) and every one since it started (`constraintsSeen`), as the platform's other constraint flags (docs/vehicle-interface.md). The record does not grow: its constraints have room. A-GRA reads it as ACTIVE_PARTIALLY_CONSTRAINED: performed, its performance limited.
- **Reported:** the cross-track in the progress, as before; its RNP read back with its point.
- **Nothing else is done:** a route off its RNP flies on as it would. What to do about it (a missed approach, a new clearance) is its policy's.
- **The support row** (`fsim.guidance.route/required_navigation_performance`) is supported on every aircraft: the monitor needs nothing an aircraft may lack.
- **Surfaces.**
  - C++: `Waypoint::rnpM`; `kActivityNavigationPerformance` (`fsim/Capability.h`).
  - C ABI 1.32: `fsim_waypoint`'s `rnp_m`, where the caller's `struct_size` has it (`fsim_waypoint_init` leaves it out); the activity info's `constraints` bit 32.
  - Python: `fsim.Waypoint`'s `rnp_m`; `fsim.ActivityFlag` (its NAVIGATION_PERFORMANCE and the other five) for `ActivityInfo.constraints`; `fsim.agra.activity_state` reads it as partly constrained.

### 4.36 A-GRA's paths and links (as FA-6e1 builds them)

A-GRA's route is a set of paths (MA_RouteType.Path, each an MA_RoutePathType): a primary path, "alternate routes and contingency routes which branch from the primary path", and paths standing alone. A path has an id and a type (MA_PathTypeEnum: primary, alternate, loss of comm, return to base, ingress, egress, a takeoff's, a landing's, a ditch's and the carrier's departure and recovery, twenty in all). Its segments are not given in flight order: each segment's NextPathSegment names the one flown after it, in its own path or another, and its ConditionalPathSegment one flown when its conditions hold. ADR-29 plans the paths and their links (WPT-13, WPT-14) and the branches (WPT-15) as FA-6e. FA-6e1 builds the paths and their links; FA-6e2 builds the branches.

- **A path** (`RoutePath`, beside the waypoints as the loiters are; 16 a route at most): its id (`id`, A-GRA's PathID), its type (`type`, `PathType`: A-GRA's twenty; left out, primary) and its points, `count` of the waypoints from `first`. The paths tile the waypoints in order: the first from point 0, each from where the one before ends, the last to the route's last point. Given none, a route is one path, as before. A path's type is a label, read back: what a path does is its points' (a landing's points are FA-10's, 4.29).
- **A link** (`next`, the Waypoint's field 28; A-GRA's NextPathSegment): the index of the point flown after it, in its path or another; -1, the route's end there. Left out, the next point in its path, and after its path's last the route's end; without paths, the next as they are, as before. A-GRA's segments, given in any order, are these points in some order, each with its next; its route's first path (FirstInRoutePathID) is the one its start is on.
- **Its flight order:** from its start (`RouteCommand::start`, point 0 by default) along each point's next, until the route's end or a point it has flown before. Its laps go round from there, on and on. A route given `repeat` whose flight ends goes back to its first point (0), as an unlinked one does (until FA-6e2, to where it began: 4.37).
  - Its points are flown in that order, as any route's are: each turn by the legs into and out of its point in that order, and a lap's last leg from its last point back to the point it goes round from, with the turn there.
  - A point it never comes to is not flown, but it is checked as a point is, in their order as given. A loiter on one is kept as given: checked as a loiter, not completed - completed and flown where a branch takes the route there (4.37). An arrival window there is due only so. A planned state there is refused, as one on a segment its first lap does not fly (4.34).
  - Its loiters (4.31), arrival windows (4.33) and planned states (4.34) are at their points as flown: a window or a state at or after a loiter point in that order is scheduled through it (FA-6g2, FA-6g3a).
- **Named as given:** a point is named everywhere by its index as given - a refusal's, a finding's and an adjustment's index, its progress's segment (its segments the points given), its end points, and its loiters' and states' points read back. Its end points come in its flight order, round its laps.
- **The end of a path** (A-GRA's END_OF_PATH; a named change to 4.29): taken at its path's last point - a route without paths, its last - and at a point whose next is -1. Where its path goes on it is none, refused `invalid_waypoint` naming the point. Until FA-6e1 it was `not_implemented`, the end of a path before the route's end.
- **Refused `invalid_waypoint`, naming the point:**
  - 17 paths or more (point 0);
  - paths that do not tile the points - one of none, one that does not begin where the one before ends, one that runs past the route, points no path holds after the last - at the first point at fault: where the one before ended, or its own first;
  - a type not one of the twenty, or an id twice (the path's first point);
  - a next that is neither a point's index nor -1;
  - a lap round one point: that is a loiter point's loiter (4.31).
- **A start turn at the point its links go round from** (4.30), its course left out (FA-6g1): its arc leaves on the course the leg into it arrives on, and its laps come to it from another point than its first lap does - so each flies its own arc: the first lap's on from the point before, a later lap's on from the last. So does each start turn after it whose course is left out (its tangent the arc before's end), and the fly-by turn at the point after them, fitted to its legs as a turn is. A later lap's own are laid in place of the first lap's once it has flown them.
  - **Checked** as the first lap's are, what the first lap's found not found again: a later lap's arc beyond 170 degrees, or other than its radius given, refused `invalid_waypoint` naming the point it reaches (so is an end whose course is not its arc's there); one the aircraft cannot turn, a finding; the turn after them, too big for its legs, flown smaller, and a gradient steeper than the aircraft climbs, flown at its rate - each a finding under `RangePolicy::Reject`, as the first lap's. The terrain walk walks a later lap's own.
  - **Where they run on to the last point,** the leg back from there is the first lap's, which every lap flies on from: a later lap comes to the last point on its own arc, and the corner there is flown over (4.30's corners). A loiter point ends them: after it, the leg on is laid from where its loiter ended, on every lap (4.31).
  - Until FA-6g1 it was not implemented, as its support row said.
- **Kept** as the loiters are: an UPDATE's new waypoints come with their paths (none given: one path), and without new waypoints it keeps its own. A route kept waiting (disabled, unassigned: 4.10) resumes at the point it flew to along its links, with the states beyond it in its flight order.
- **Reset** (4.10; a named change to 4.34): over from its first start, a linked route along its links from there, with all its states - those it flew past before it was kept waiting too. Until FA-6e1 a route reset after it had resumed flew without those, and a linked one would have gone on from where it resumed.
- **A stack on its own** (`ControlStack::command`, unchecked) takes its paths too, and flies along its links from its start; links it cannot fly leave it nothing to fly, and its behaviour fails.
- **The support rows,** `fsim.guidance.route/paths` and `fsim.guidance.route/next_segment` supported (the start turn above since FA-6g1), are the same on every aircraft: they need nothing an aircraft may lack.
- **Surfaces.**
  - C++: `RoutePath` and `PathType` (`fsim/Control.h`); `Waypoint::next`; a `Span<const RoutePath>` after the states in `World::submit` and `World::update` (a route's), `ControlStack::command`, `BatchCommand::paths`, `Setpoint::paths`; `PathStore::routePaths`, and the flight order beside the points as given.
  - C ABI 1.33: `fsim_route_path` (`fsim_route_path_init`: none of it given) and `enum fsim_path_type`; `fsim_waypoint`'s `next`, where the caller's `struct_size` has it (`fsim_waypoint_init` leaves it out); `fsim_route_extras` (a route's loiters, states and paths together; `fsim_route_extras_init`: none), taken by `fsim_vehicle_submit_route_extras` and `fsim_activity_update_route_extras`; `fsim_batch_command`'s `paths` and `path_count`, filled by `fsim_activity_get_setpoint`.
  - Python: `fsim.RoutePath(id, type, first, count)`, its type by name or `fsim.PathType`; `fsim.Waypoint`'s `next`; `paths=` beside `states=` in `submit_route` and `update_route`, and in a batch's or a task's `BatchCommand("submit_route", ...)`; read back in the setpoint's `paths`.

### 4.37 A-GRA's conditional branches (as FA-6e2 builds them)

A-GRA's path segment may carry conditional segments (MA_PathSegmentType.ConditionalPathSegment): a segment of any path, flown next when its condition holds - "a logical AND of all fields present (except OperatorInput)". Its conditions (PathSegmentConditionType) are the altitude at the segment's end within a range, the time within a window, the segment captured so many times, the operator's input, the endurance remaining, and the contingency level. ADR-29 plans them as WPT-15, FA-6e2. FA-6e2a built the branches with their altitude, time, capture and operator-input conditions; FA-6e2b the endurance and contingency ones.

- **A branch** (`RouteBranch`, beside the waypoints as the loiters are; 16 a route at most): at waypoint `point`, on to `next` - another point's index, in its path or another; -1, the route's end there - and its conditions, each left out or given:
  - `altitudeMinM`, `altitudeMaxM` in `altitudeReference` (left out, above mean sea level): its altitude within them, a side left out open;
  - `timeBeginS`, `timeEndS` (the world's simulation seconds): the time within them;
  - `captures` with `capturesComparison` (`Comparison`, A-GRA's EqualityExpressionEnum): the times it has come to the point, this one too, compared so with the count;
  - `operatorInput` 1: only once the operator has commanded it;
  - what it has left, compared by `enduranceComparison` with each of `fuelKg`, `enduranceS`, `enduranceEndS` (the world's simulation seconds: now and its endurance) and `percent` given;
  - its `contingency`: NORMAL, or FLIGHT_CRITICAL (at or below its reserve, or its engines starved).
  What it has left and its contingency are as its navigation report says (4.14), read as the branch is decided: the behaviour asks the step's world for its own vehicle's (`WorldView::navigation`; its flight model idle while its cascade runs). A battery's fuel is 0; where the flight model tells of no energy, an endurance never holds; a stack on its own has no report, and neither holds.
  None given, it holds.
- **Decided** as the aircraft comes to its point: where its turn there begins (a fly-by's lead), at the point (flown over), or as a loiter point's loiter ends. The point is captured then. Its branches are tried in their order and the first that holds is taken; none, its own next, as the links say.
- **Taken:** the route goes on from the branch's next along the links; one that repeats goes back from its end to its first point, as ever. It is planned again from where the aircraft is. A branch at a fly-by is flown to its point, the turn there toward its next; at a point flown over, or as a loiter ends, on from the aircraft to its next. What was kept of the points it now flies is flown - their loiters, completed now, and their arrival windows - and the first lap's planned states are left behind. Its laps, the distance it has flown and the captures go on; its distance to go is the rest of the new flight.
- **The operator's input** (`World::commandBranch`): a branch that takes it is commanded, or no longer, while its route flies or waits, addressed as an UPDATE is. It holds until the route is flown afresh - an UPDATE, a Reset. Refused `invalid_parameter` (its index: the branch) for one it has not, or one that takes no operator input; `wrong_command_type` for an activity that is no route.
- **Reported:** its branches read back as given. A flying route's end points are as its behaviour flies it: a branch taken, on from there; one not yet decided, as the links say.
- **Refused `invalid_waypoint`, naming its point:**
  - 17 or more (the 17th's point);
  - at a point it has not; a next that is none, or its own point;
  - a field not finite, a code that is none; a range or a window upside down, a reference with no range; captures without their comparison or the other way round, or not whole; an operator input not 0 or 1; an endurance without its comparison or the other way round, a percent outside 0 to 100, a fuel or an endurance below 0;
  - a flight on from it that is none: round one point.
- **Not implemented** (as its support row, `fsim.guidance.route/conditional_segment`, says: partial): a contingency the platform does not reach yet - MISSION_CRITICAL (a subsystem's failure) and LOST_COMMS (the policy's link), FA-16's.
- **A stack on its own** takes them too, unchecked; it has no operator's input to take.
- **A named change to 4.36:** a repeating linked route whose flight ends goes back to its first point (0), as an unlinked one does. FA-6e1's went back to where it began, which differs where that is not the first.
- **Surfaces.**
  - C++: `RouteBranch` and `Comparison` (`fsim/Control.h`); a `Span<const RouteBranch>` after the paths in `World::submit`, `World::update` (a route's) and `ControlStack::command`; `BatchCommand::branches`, `Setpoint::branches`; `World::commandBranch`; `PathStore::routeBranches` and `routeCommanded`; `Behavior::ahead`, the points a route flies from here; `WorldView::navigation`, a vehicle's own report as its cascade runs.
  - C ABI 1.34: `fsim_route_branch` (`fsim_route_branch_init`: every field left out), its 15 fields in `RouteBranch`'s order; `enum fsim_comparison`; `fsim_route_extras`'s `branches` and `branch_count`, where the caller's `struct_size` has them; `fsim_batch_command`'s `branches` and `branch_count`, filled by `fsim_activity_get_setpoint`; `fsim_activity_command_branch`.
  - Python: `fsim.RouteBranch` (its codes by name or member) and `fsim.Comparison`; `branches=` in `submit_route`, `update_route` and a batch's or a task's `BatchCommand("submit_route", ...)`; read back in the setpoint's `branches`; `Activity.command_branch(branch, commanded=True)`.

### 4.38 A-GRA's civil path terminators (as FA-6f builds them)

A-GRA's path segment may name its civil path terminator (MA_PathSegmentType.CivilPathTerminator, a CivilPathTerminatorType): "the Civil Path Terminator information associated with the End Point" - the ARINC 424 leg type of the leg into its end point, one of 23. Its schema gives data for two - a course to fix's course (CF_CourseToFixType) and a radius to fix's arc (RF_RadiusToFixType: its centre, radius, courses in and out, initial and end points, arc and direct distances, and turn direction) - and leaves the other 21 empty, their "children elements ... subject to change over the course of development". Its ICD and its VI volume say nothing more. ADR-29 plans them as WPT-19. FA-6f flies each as ARINC 424 defines it (the path terminators FAA Order 8260.58 and ICAO's PANS-OPS publish) where its segment carries what defines it - in its terminator, or in its end point - in two steps: FA-6f1 the legs that end at their fix, FA-6f2 those that end at an altitude, an intercept, a distance or the operator's hand, and the holds.

- **Its type** (`terminator`, the Waypoint's field 29; `PathTerminator`, in the schema's order, ARINC 424's codes beside them): the leg into the point. Left out, the route's own leg, as before. Its data, where it has any, in a `RouteTerminator` beside the waypoints, as the loiters are: 64 a route at most, one a point.
- **Flown** (FA-6f1):
  - **a track to fix (TF):** the great circle from the point before - on a rhumb route too;
  - **a direct to fix (DF):** straight to the point from where the aircraft is as the leg begins. Its leg is made again from where the aircraft is at each update, the turn at its point planned again with it, until its track is within a degree of the point; that leg is flown on. After a fly-by at the point before, whose turn is planned onto the leg from that point, the aircraft ends the turn on course; after a point flown over, a loiter, a branch taken or as the route begins, it turns straight to its point. Flown over a point and then direct, a C172 was 248 m east of the line from that point halfway to the next, where the route's own leg had brought it back to within 28 m. A point it comes abeam of as it turns to it, too near to turn to - within twice the turn at its speed and 80 % of its bank (a rotorcraft's, its tilt) - is passed there, as a point is passed abeam (since FA-6f2a: first built, six aircraft never passed a point two orbit radii to the side of where their course ended);
  - **an initial fix (IF):** where a procedure begins - A-GRA's end point has a leg into it all the same, flown as a direct to fix's;
  - **a course to fix (CF):** its course's great circle into the point (`courseRad`, required), joined from abeam where the leg begins - the point before, or where the aircraft is (as the route begins, after a loiter, after a branch taken at a point flown over). Where there is past the point along its course, the leg there is direct; a point before past it is refused;
  - **a radius to fix (RF):** the arc round its centre (`centerLatitudeRad`, `centerLongitudeRad`, required) from the point before to the point, its way round (`clockwise`, required: A-GRA's TurnDirection). It is laid out in the plane at its centre - a circle round it on the Earth, its radius the point before's distance from it - where a start turn point's arc is laid out at its start (4.30); the follower flies it as it flies those, its curvature fed forward and looked ahead for.
- **Flown** (FA-6f2a) - the legs whose data A-GRA leaves out entirely, from what their segment gives: a course or a heading is the planned leg's, from the point before to the point (A-GRA's end point is where its planner has the leg end, along its course), and an altitude the point's:
  - **a course to an altitude (CA):** that course, from where the leg begins, until the aircraft is at the point's altitude (within 10 m, or past it) - before its point, or on past it; its climb as the segment's (its gradient to its point, its climb rate or its optimisation), held at the altitude past its point;
  - **a track from a fix to an altitude (FA):** likewise, from the point before;
  - **a heading to an altitude (VA):** that course held as a heading, what the wind does to its track left to it, as the hsa flies a heading (a wing's trimmed on what its loops leave: 4.12);
  - **a course or a heading to an intercept (CI, VI):** flown until it meets the next leg - a course to fix's line, required: its cross-track to it within what the turn onto it takes at its speed and 80 % of its bank (a rotorcraft's, its tilt), closing on it, or crossed - and that leg joined from there;
  - **a track from a fix for a distance (FC):** as a track to fix: its point is where its distance along its track ends.
  These end where the aircraft is (but FC): their point has no turn, and the leg after one begins where it ended - a course, a heading, a direct or a course to fix, or the route's own leg from there, as after a loiter (4.31).
- **Flown** (FA-6f2b) - the legs the operator's hand ends, and the holds:
  - **a track from a fix, or a heading, to a manual termination (FM, VM):** its course from the point before (a heading's held) on past its point until the operator ends it - A-GRA's only operator's hand in a route is a branch that takes the operator's input (4.37): once one at its point is commanded, and holds, it is taken from where the aircraft is. A manual termination no branch at its point ends is refused, but at the last point of a route that does not repeat: there the route completes as the leg begins and flies on along it, a heading's on its heading - its end its manual leg's, whatever its end behaviour;
  - **a hold (HA, HF, HM):** its point a loiter point whose loiter is a hold, with no end of its own (4.31) - its terminator ends it, at its fix: once round (HF: its entry, a lap, its fix); at its altitude, once within 10 m of it, and not held at all where the aircraft comes to it there (HA); once the operator has commanded a branch at its point that holds (HM: then taken, as a loiter's end decides one). A hold's terminator on a hold with an end of its own, on another loiter, or at no loiter point, is refused.
- **Checked** as 4.30's turn points are - under a range policy; unchecked (`RangePolicy::None`), flown as given - refused `invalid_waypoint` naming the point:
  - an arc: its point off its circle (by more than a metre, or half a percent of its radius), sweeping more than 170 degrees, what else its data gives not its own (its radius and its ends, a metre or half a percent; its courses in and out, a degree; its length and its chord, a metre or half a percent); at the start of a route that does not repeat (the route comes to it from where the aircraft is), at a point a later lap comes back to from its last (two points before it), its point or the one before in a moving frame (its centre does not move), after a loiter point whose loiter does not leave at its point (an orbit's, two circles', one with an exit point), and a branch's next (the branch comes to it from where it is taken: named at the branch's point). One the aircraft cannot turn at its point's speed and its full bank (a rotorcraft's, its tilt), whatever the policy;
  - after a start turn point, whose arc is the leg, anything but an arc - its tangent at its start the start's course (given, or the leg in's) within a degree; after a capture, whose course is the leg, anything but a track to fix or a course to fix on its course (within a degree);
  - a course to fix whose point before is past it along its course.
- **Refused `invalid_waypoint`, naming the point, whatever the policy:** a leg to an altitude without the point's altitude (a hold's to an altitude too); a manual termination - a leg's or a hold's - no branch at its point that takes the operator's input ends, but at a route's last point; a hold's terminator at no loiter point, on a loiter that is no hold, or on one with an end of its own (a hold once round given 1 lap alone); a leg that ends where the aircraft is at a loiter point; after one, a leg that begins at the point before (a track or radius to fix, a track from a fix, a hold), named at its point; an intercept whose next leg is no course to fix, or that ends the route (named at the intercept's point); a type that is not one; data at a point past the route, two for one point, a field not finite, a way round not 0 or 1, a place's latitude without its longitude, a length not above 0; data its leg has none of (a course on an arc, an arc's data on a course to fix, any on the others); a course to fix without its course; an arc without its centre or its way round.
- **Not defined by its segment:** a leg to or along a navaid's DME distance, radial or arc (AF, CD, CR, FD, VD, VR: A-GRA gives no navaid, and the world has none) and a procedure turn (PI: its outbound course, its side and the distance it stays within, which A-GRA does not give). Their elements are empty in A-GRA 6.0a: refused `invalid_waypoint` naming the point, as a capture without its course is (4.30). No stage builds them; where a later A-GRA gives their data, a stage will.
- **The support row,** `fsim.guidance.route/path_terminators`, is supported on every aircraft since FA-6f2b: every leg A-GRA 6.0a's segment defines is flown.
- **Kept** as the loiters are: an UPDATE's new waypoints come with their terminators; without new waypoints it keeps its own. Read back as given.
- **A stack on its own** takes them too, unchecked.
- **A named change to 4.37:** a branch refused on a linked route whose flight order is not as given is named by its point as given. FA-6e2a named its place in the flight order.
- **Surfaces.**
  - C++: `PathTerminator`, `RouteTerminator` (`fsim/Control.h`); `Waypoint::terminator`; a `Span<const RouteTerminator>` after the branches in `World::submit`, `World::update` (a route's) and `ControlStack::command`; `BatchCommand::terminators`, `Setpoint::terminators`; `PathStore::routeTerminators`.
  - C ABI 1.35: `enum fsim_path_terminator`; `fsim_waypoint`'s `terminator`, where the caller's `struct_size` has it (`fsim_waypoint_init` leaves it out); `fsim_route_terminator` (`fsim_route_terminator_init`: every field left out), its 13 fields in `RouteTerminator`'s order; `fsim_route_extras`'s and `fsim_batch_command`'s `terminators` and `terminator_count`, where the caller's struct has them, filled by `fsim_activity_get_setpoint`.
  - Python: `fsim.PathTerminator` (A-GRA's names, ARINC 424's codes their aliases: by name either way, "tf" or "track_to_fix"); `fsim.Waypoint`'s `terminator`; `fsim.RouteTerminator`; `terminators=` in `submit_route`, `update_route` and a batch's or a task's `BatchCommand("submit_route", ...)`; read back in the setpoint's `terminators`.

### 4.39 A-GRA's route plans (as FA-7a builds them)

A-GRA's route plan (MA_RoutePlanMT) is a route FA keeps by its id and version, taken through the plan activation state machine before it flies (VI 1.2.5). MA prepares FA for its upload (PREPARE_FOR_UPLOAD: FA listens for it), publishes it, has FA keep it (UPLOAD), make its final checks (PREPARE_FOR_ACTIVATION) and fly it (ACTIVATE), or takes it back (DEACTIVATE, which fails once it executes). FA may abort it before or after it starts: DEACTIVATED, its execution CANCELED. FA reports its execution (PlanExecutionStateEnum). A plan for planning use only is never activated. MA queries the ids FA keeps, and each plan (1.2.4.2, 1.2.6.4). ADR-29 plans these as RPL-01, 02, 05 and 08 to 11, with the route's planning metadata (WPT-23), as FA-7a. FA's own plans and the airfields are FA-7b; validation with weather and patches, FA-7c.

- **A plan** (`RoutePlan`):
  - its id (`id`, the caller's, not 0: A-GRA's RoutePlanID) and its `version`;
  - whether it is for planning use only (`forPlanningUseOnly`);
  - its route: its options (`route`, a `RouteCommand`), its waypoints, and their loiters, planned states, paths, branches and civil path terminators' data, as `World::submit` takes a route's;
  - its planning metadata (WPT-23), none of which flies:
    - the route's `detailed` and its remarks (`remarksName`, `remarks`: A-GRA's RemarksType's DisplayName and Detail);
    - each point's (`PointMetadata`, for waypoint `point`): its `source` (`PointSource`, auto-routed or operator-defined), `locked`, `modified`, its remarks, and its fix's identifier (`fixKey`, `fixSystem`: Fix_Identifier's Key and SystemName);
    - each path's (`PathMetadata`, for path `path`; 0 without paths): its initial conditions - `initial`, a planned state's fields (4.34), its point not used; `enduranceS`, `fuelKg`, `grossWeightKg`; `transitionPlan`, the plan it transitions from.
  It is kept with the plan and read back.
- **The store:** 32 plans a vehicle. `World::planCommand(vehicle, plan, command, options)` takes the VI's five commands (`PlanCommand`) and answers each at once (`PlanCommandResult`): completed or failed, the plan's state after it (`PlanState`: A-GRA's PlanActivationStateEnum, the states FA reaches without approvals; `Inactive`, not kept), and the reason. Its `check` is PrepareForActivation's validation and Activate's NEW as they answered (the point a refusal names, its activity), or Deactivate's CANCEL.
  - **PrepareForUpload:** FA listens for the plan (`ReadyForUpload`). A new id takes room in the store; with none left it fails `plan_store_full` (`PreparationForUploadFailed`), and nothing is kept. A plan activated whose activity is live is refused `wrong_plan_state`.
  - **Publishing it** (`World::publishPlan(vehicle, plan)`, A-GRA's MA_RoutePlanMT): taken for an id FA listens for (`Reason::None`, A-GRA's notification CONFIRMED). Refused, and not taken, `wrong_plan_state` for an id FA does not listen for; `invalid_parameter` for id 0, or metadata at no point or path, twice for one, or a text not printable ASCII or longer than A-GRA's (32, 1,024 and 256 characters), or a number neither left out nor finite, or an endurance, a fuel or a weight below 0. Published again, the later one is taken.
  - **Upload:** from `ReadyForUpload`, the plan received kept (`Uploaded`), its revision (`PlanStatus::revision`) one more, its execution none. None received: failed `plan_not_received` (`UploadFailed`). Until an upload, the version kept before stays.
  - **PrepareForActivation:** its final checks. Its route is checked as its NEW would be now, with the options given, flying nothing (`CommandOptions::validateOnly`, 4.8): `ReadyForActivation`, or failed with the check's reason (`PreparationForActivationFailed`: `invalid_waypoint` naming its point, ...). Taken from a plan kept whose activity is not live: `Uploaded`, `ReadyForActivation` (checked again), either failed state after it, `Deactivated`, or `Activated` once its execution has ended.
  - **Activate:** from `ReadyForActivation`, the NEW of its route with the options given: accepted, `Activated`, its activity in `check`; refused, `ActivationFailed`, with the NEW's reason. A-GRA's traceability names no plan, so its activity traces to none; its status names its activity.
  - **Deactivate:** `Deactivated` - from `ReadyForActivation`; from `Activated` while its activity has not flown (waiting for its axes or its start window, or disabled: that activity canceled, as the options' source and controller), or once its execution has ended. One whose activity flies fails `plan_executing`, and it flies on (VI 1.2.5.4).
  - **For planning use only** (RPL-11): prepared for activation, it fails `planning_only`; so it is never activated.
  - Any other command, in any other state, is refused `wrong_plan_state`, the plan unchanged. An id the store does not keep: `unknown_plan` (but PrepareForUpload, which makes it); id 0, `invalid_parameter`.
- **Its execution** (`PlanExecution`, A-GRA's PlanExecutionStateEnum; RPL-10), its activity's since it was activated last, as a task's is (4.11): none until then; `Pending` while its activity waits, is disabled or has not flown; `Executing` while it flies; then `Complete`, `Failed`, `Superseded` (another command took its axes: preempted), or `Canceled` (on request, deleted, or ended by the platform). `PlanStatus` gives it with the reason, the percent of its route, when it was activated and ended, and its activation's command id.
- **FA's own deactivation** (VI 1.2.5.7; RPL-09): `World::abortPlan(vehicle, plan, reason)`, the platform's. A plan ready for activation or activated is `Deactivated`, and its execution, unless done, `Canceled`: its activity, if live, canceled with the reason (`restricted` by default). So is one whose activity the platform itself ends: its grant revoked, or for collision avoidance or a restriction.
- **Queries** (RPL-05): `World::plans(vehicle)` gives every plan kept, in the order they were first prepared for upload - its id, version, revision, state and execution (A-GRA's query for identifiers only); `planStatus(vehicle, plan)` one; `plan(vehicle, plan)` its content as uploaded last, its metadata with it. The revision stands for A-GRA's hash of a plan's content (XPT-07).
- `World::removePlan(vehicle, plan)` forgets a plan whose activity is not live (`unknown_plan`, `wrong_plan_state`).
- **FA-7b** keeps a plan whose paths are a takeoff's, a departure's, an approach's or a landing's FA's own, refused to MA, and adds the airfields (4.40). FA-7c validates plans, in weather, and patches (4.41).
- **The support rows,** `fsim.plan/store` and `fsim.guidance.route/metadata`, are supported on every aircraft: they need nothing an aircraft may lack. The route capability's pending list now names FA-16's contingencies alone.
- **Surfaces.**
  - C++: `PlanId`, `PlanCommand`, `PlanState`, `PlanExecution`, `PlanStatus`, `PlanCommandResult` (`fsim/Capability.h`); `PointSource`, `PointMetadata`, `PathMetadata`, `RoutePlan` (`fsim/Control.h`); `World::publishPlan`, `planCommand`, `abortPlan`, `removePlan`, `planStatus`, `plans`, `plan`, and `Vehicle::` likewise; `Reason::UnknownPlan`, `WrongPlanState`, `PlanExecuting`, `PlanNotReceived`, `PlanningOnly`, `PlanStoreFull`; `planCommandName`, `planStateName`, `planExecutionName`, `pointSourceName`.
  - C ABI 1.36: `fsim_route_plan` (`fsim_route_plan_init`: its `route` a `FSIM_BATCH_ROUTE` item), `fsim_point_metadata`, `fsim_path_metadata` (their inits: none of it given), `enum fsim_point_source`; `fsim_vehicle_publish_plan`; `fsim_vehicle_plan_command` (`enum fsim_plan_command`) and `fsim_vehicle_abort_plan`, answered in an `fsim_plan_command_result` (`enum fsim_plan_state`); `fsim_vehicle_remove_plan`; `fsim_vehicle_plan_status`, `_plan_count`, `_plan_at` (`fsim_plan_status`, `enum fsim_plan_execution`); `fsim_vehicle_get_plan`, its arrays and texts the library's; `fsim_plan_command_name`, `fsim_plan_state_name`, `fsim_plan_execution_name`, `fsim_point_source_name`.
  - Python: `fsim.RoutePlan(id, fsim.BatchCommand("submit_route", ...), ...)`, `fsim.PointMetadata`, `fsim.PathMetadata`, `fsim.PointSource`; `vehicle.publish_plan(plan)`; `vehicle.plan_command(plan_id, command, **options)` (`fsim.PlanCommand` or its name) and `abort_plan(plan_id, reason)`, each a `fsim.PlanCommandResult` (`fsim.PlanState`; an activation's `Activity`; the point a failed check names, and its findings); `remove_plan`; `plan_status`, `plans()` (`fsim.PlanStatus`, `fsim.PlanExecution`); `plan(plan_id)`; `fsim.agra.plan_activation_status`, `plan_execution_state`.

### 4.40 A-GRA's airfields, and FA's own route plans (as FA-7b builds them)

FA keeps what is safety critical: the airfields and the route plans for takeoff, departure, approach and landing, "loaded on FA pre-mission ... read only" (VI 1.2.6.3, 1.2.6.4). An airfield is A-GRA's AirfieldReportMT: its runways, each with its takeoff and landing coordinates and limits. MA queries them, and activates one of FA's plans; it never sends one of its own for those phases (1.2.5.2). A takeoff's or a landing's path names its airfield and runway, which must be FA's. ADR-29 plans these as RPL-03, RPL-04 and ENV-05, FA-7b.

- **An airfield** (`Airfield`, A-GRA's AirfieldReportMDT's): its id (`id`, not 0: A-GRA's AirfieldID), its ICAO code (`icao`: four capitals, or none), its QNH (`qnhPa`: A-GRA's QNH_Setting; left out, none) and its runways (`Runway`, 16 at most):
  - a runway's id (`id`, not 0: RunwayID), its direction (`directionRad`: from true north, 0 to 2 pi; left out, none) and its available length (`availableLengthM`, above 0; left out, none);
  - its takeoff and landing coordinates (`takeoff`, `landing`: A-GRA's RunwayCoordinatesType). Each set is its `start`, `threshold` and `limit`, each a `RunwayPoint`: A-GRA's Point3D_Type - latitude, longitude, altitude, and the altitude's reference, above the WGS-84 ellipsoid when left out. A set's start is required where any of it is given; each runway gives at least one set.
  Its elevation is its points' altitudes; its ends, a set's start and limit.
- **Loaded by the platform** (`World::loadAirfield(vehicle, airfield)`), before a mission or as FA's database changes: kept by its id - one loaded again in its place, its revision one more (`Airfield::revision`, read back). 32 a vehicle; with no room left, `plan_store_full`. Refused `invalid_parameter`:
  - id 0, a runway's id 0 or twice, more than 16 runways;
  - a point off the Earth or not whole (its latitude given without its longitude or altitude), a reference that is none;
  - a set without its start, a runway with neither set;
  - a direction outside 0 to 2 pi, a length not above 0;
  - an ICAO code not four capitals, a QNH outside 850 to 1,100 hPa (the altimeter's: 4.20).
- **Queried** (VI 1.2.6.3; RPL-05): `World::airfields(vehicle)` gives every airfield kept, as loaded, in the order they were first loaded; `airfield(vehicle, id)` one.
- **FA's own plans** (RPL-03): `World::loadPlan(vehicle, plan)`, the platform's, keeps a route plan `Uploaded` and FA's (`PlanStatus::faOwned`), its revision one more. It takes the place of any plan by its id whose activity is not live (else `wrong_plan_state`); with no room, `plan_store_full`; malformed, `invalid_parameter`, as a published plan. MA prepares it for activation, activates and deactivates it as its own. Its preparation for upload and its removal are refused `read_only_plan`, and FA listens for no plan by its id. The platform deactivates it as any other (4.39).
- **A takeoff's or a landing's path** (A-GRA's TAKEOFF, LANDING, EMERGENCY_LANDING), on FA's own plan, names its airfield and runway (`PathMetadata::airfield`, `runway`: A-GRA's MA_RoutePathType.AirfieldID and RunwayID). They must be ones the vehicle keeps: with none, or one it does not keep, the plan is refused `unknown_airfield`. On any other path they are kept and read back.
- **MA's plans for those phases** (VI 1.2.5.2) are refused as published, `safety_critical_plan`, and FA listens on. They are plans with a path of a takeoff's, a departure's, an approach's or a landing's type:
  - A-GRA's TAKEOFF, LANDING and EMERGENCY_LANDING;
  - a departure's: AIRBORNE, ARCING, BREAKING, ON_DEP_RADIAL (CV Admin's);
  - an approach's: INITIAL_APP, INTERMEDIATE_APP, FINAL_APP and BOLTER_WAVEOFF (CV Admin's recovery).
  A taxi route (TAXI) is MA's too: an aborted takeoff's is FA's as part of its takeoff plan.
- **What flies:** FA's plans fly as their routes, as MA's do (4.39). A path's type is a label (4.36): a takeoff's or a landing's path is flown as its points, not as a takeoff or a landing, until FA-9 and FA-10 build those (their support rows say so).
- **A named change to 4.39:** a plan with a takeoff's, a departure's, an approach's or a landing's path, published by MA, is refused. FA-7a kept it as any other.
- **The support rows,** `fsim.plan/fa_plans` and `fsim.plan/airfields`, are supported on every aircraft.
- **Surfaces.**
  - C++: `AirfieldId`, `RunwayId`, `RunwayPoint`, `RunwayCoordinates`, `Runway`, `Airfield` (`fsim/Control.h`); `PathMetadata::airfield`, `runway`; `PlanStatus::faOwned`; `World::loadAirfield`, `airfields`, `airfield`, `loadPlan`, and `Vehicle::` likewise; `Reason::ReadOnlyPlan`, `SafetyCriticalPlan`, `UnknownAirfield`.
  - C ABI 1.37: `fsim_runway_point`, `fsim_runway_coordinates`, `fsim_runway` (`fsim_runway_init`: every place and value left out), `fsim_airfield` (`fsim_airfield_init`); `fsim_vehicle_load_airfield`, `_airfield_count`, `_get_airfield_at`, `_get_airfield`, its runways and ICAO code the library's; `fsim_vehicle_load_plan`; `fsim_path_metadata`'s `airfield` and `runway`, where the caller's `struct_size` has them; `fsim_plan_status`'s `fa_owned` (1.36's reserved word).
  - Python: `fsim.Airfield`, `fsim.Runway`, `fsim.RunwayCoordinates`, `fsim.RunwayPoint`; `vehicle.load_airfield(airfield)`, `airfields()`, `airfield(id)`, `load_plan(plan)`; `fsim.PathMetadata`'s `airfield` and `runway`, `fsim.PlanStatus.fa_owned`; `fsim.agra.airfield_report`.

### 4.41 A-GRA's route plan validation, in weather, and patches (as FA-7c builds it)

MA may have FA validate a route plan without flying it (VI 1.2.5.5: RoutePlanValidationCommand, answered by RoutePlanValidation). It gives the weather (WeatherAreaData) and where the plan would be flown from (Origin). It may instead verify a patch, the parts of a plan (PlanPart: path types) it would replace (1.2.5.6). ADR-29 plans these as RPL-06, RPL-07 and ENV-10, FA-7c.

- **Validated** (`World::validatePlan(vehicle, plan, validation)`): its route is checked as its NEW would be - its points, their extras and its options - flying nothing and keeping nothing. The plan may be kept or not; a kept one's is by its id, as uploaded last (`unknown_plan`; `wrong_plan_state` before its first upload). A plan for planning use only validates as any other.
- **The validation** (`PlanValidation`; each part left out, as the vehicle is now):
  - **the weather** (A-GRA's WeatherAreaData.WindData; ENV-10): its wind - `windNorthMs`, `windEastMs`, where it blows to - and its gusts (`gustMs`, along it). The route's turns are checked in them: each turn's radius at its speed with the wind and its gusts behind it, as a NEW's turns are in the wind the air data measure. A-GRA's other weather - temperature, pressure, precipitation, visibility, icing - is not used: the checks fly in the air the aircraft is in (a minimal model, as D8 has the supporting models).
  - **the origin** (A-GRA's Origin): where it is validated from - `originLatitudeRad`, `originLongitudeRad`, and `originAltitudeM` (left out, the aircraft's altitude). The route's first leg is from there, the aircraft's own motion as it is.
  - **modify to validate** (A-GRA's ModifyToValidate): a value beyond the aircraft's limits is held to them, an adjustment (`RangePolicy::Clamp`). Left false, each is a finding (`RangePolicy::Reject`).
  - **its parts** (`parts`, A-GRA's PlanPart: bit i for PathType i), a patch's: the verdict is theirs alone, valid where every finding lies on a path of another type. A refusal the checks stop at - a point they look no further past - cannot be vouched for, and is invalid. Left 0, the whole plan.
- **Answered** (`PlanValidationResult`, A-GRA's RoutePlanValidation): `valid` (VALID; else INVALID), and `check`, the route's validation as it answered - Valid, or Rejected with its reason and the point it names. The vehicle's command details (`World::commandDetails`) have its findings and adjustments; `fsim.agra.route_plan_validation` gives A-GRA's form - InvalidPath, InvalidSegment, and each finding's RouteValidationErrorEnum, by the limit it breaks.
- **Refused `invalid_parameter`:** a plan a publish would refuse (4.39); a wind given one way alone, or not finite; a gust below 0; an origin's latitude without its longitude, or off the Earth, or an altitude without its place; a part that is no path type.
- **The support row,** `fsim.plan/validate`, is supported on every aircraft. With it, stage 7 has no row left: FA-7 is done.
- **Surfaces.**
  - C++: `PlanValidation`, `PlanValidationResult` (`fsim/Capability.h`); `World::validatePlan` (a plan, or a kept one's id), and `Vehicle::` likewise.
  - C ABI 1.38: `fsim_plan_validation` (`fsim_plan_validation_init`: as the vehicle is now), `fsim_plan_validation_result`; `fsim_vehicle_validate_plan`, `fsim_vehicle_validate_stored_plan`.
  - Python: `vehicle.validate_plan(plan_or_id, wind=, gust_ms=, origin=, modify=, parts=)`, a `fsim.PlanValidationResult` (`valid`, its `fsim.Validation`, the `index` a refusal names); `fsim.agra.route_plan_validation`.

### 4.42 A-GRA's must fly: points, entities, operational points and the ingress window (as FA-8a builds them)

A-GRA's MUST_FLY (MA_FlightCapabilityEnum; the flight command's MustFly, MustFlyType) is a location the aircraft must fly: a point, an entity, an operational point, line, zone or volume by id, or a zone, line or volume given with it - approached, where it says so, from within a window of bearings (its IngressConstraint: "the acceptable range of bearing values at the ... Location that the System must approach from. Bearing from is defined as the true heading from the ... Location to the System"). ADR-29 plans it as MFY-01 to MFY-07, with operational geometry (ENV-06), FA-8. FA-8a builds the points, entities, operational points and the ingress window (MFY-01, MFY-02, MFY-07; MFY-03 and ENV-06 for points). FA-8b builds zones (FA-8b1: 4.43), corridors (FA-8b2: 4.44) and volumes (FA-8b3: 4.45).

- **The mode** (`MustFlyCommand`, fsim.guidance.must_fly): `location` (`MustFlyLocation`: a point, an entity, an operational point), a point's `latitudeRad` and `longitudeRad`, the `altitudeM` it flies over the location at and its `altitudeReference`, a `target` (an entity's vehicle id, or an operational point's id), `ingressMinRad` and `ingressMaxRad`, a `speed` and its `speedReference`. It joins the command variant no larger than its largest (4.23): ten fields. It takes NEW, UPDATE and CANCEL, as the other modes do.
- **Laid out as a route** from where the aircraft is, when it is commanded (or starts, where it waits). The route has the points it approaches through, then the location, which is flown over (a waypoint). It is flown by the route's follower, and checked as a route is: its turns, gradients, terrain and endurance, by point (0 and 1 its approaches, the last its location). It completes as the location is passed, and flies on along its course there. Its setpoint reads back as commanded; its route is in its setpoint's waypoints and its end points.
- **A point** (A-GRA's Point3D): flown over at its altitude; left out, the aircraft's.
- **An entity** (A-GRA's EntityID): another vehicle in the world, flown over as it moves - a point in its own frame (4.21, 4.29).
  - Its altitude is the one given; left out, as far above it as the aircraft is when commanded, and never less than 500 ft (A-GRA leaves it to "the service design"). That height is kept as the vehicle climbs or descends.
  - The leg to its first point is flown to where that vehicle will be as the aircraft gets there - carried on at its velocity, met at the aircraft's speed over the ground - so the aircraft flies a collision course that closes on the vehicle itself. Pursuing where the vehicle was instead left a jet 540 m off it (section 14).
  - The vehicle gone, the activity fails `target_lost`, as a route's frame does. The aircraft itself, or a vehicle the world does not have, is refused `invalid_parameter` (field 5).
- **An operational point** (A-GRA's OpPointID; ENV-06): one the world keeps by its id, its place, altitude and window used where the command gives none. One the world does not keep is refused `unknown_geometry`.
- **The ingress window** (A-GRA's IngressConstraint): the bearings from the location to the aircraft, from the least clockwise to the most (it may wrap), each within half a turn.
  - Where the aircraft's bearing from the location lies within it when commanded, the aircraft flies straight in.
  - Else it approaches through a point on the window's nearer edge, moved in by 5 degrees (or half the window). That point is 3 turn radii out - the radius the route plans its turns with, at the speed commanded (else the aircraft's, a rotorcraft's cruise) plus the wind - and at least 10 seconds of flight out. The leg from it to the location is flown on that bearing's reciprocal.
  - A turn onto that leg of more than 120 degrees is split by a point 2.5 radii to the aircraft's side of it. So a window behind the aircraft is flown round to, not reversed into.
  - A moving location's approach points are in its frame: north and east of its origin, unturned, as the location lay when commanded.
- **Operational points** (ENV-06; A-GRA's OpPoint): `OpPoint` - an id, a place on the Earth or in a frame (its offsets as a route point's), its altitude and reference, its window - kept by the world (`World::setOpPoint`) in place of any by its id, its revision one more; `opPoints`, `opPoint`, `removeOpPoint`.
  - Refused `invalid_parameter`: id 0; neither a place nor a frame, or both; a latitude off the Earth, or a value that is not finite; a frame the world does not have, or offsets without one; an altitude reference without its altitude; a window given one way alone, or beyond half a turn.
  - A-GRA's other OpPoint data (its category, schedule, turn direction, safe altitudes) are not kept: nothing flies by them (D8).
- **Refused `invalid_parameter`,** naming the field: a code that is not one; a point without its place, or naming a target; an entity or operational point with a place, or without a whole id; a window given one way alone, or beyond half a turn; a negative speed. In an UPDATE, the fields given merge into those kept. A location given, other than it was, replaces the location's own fields (a place, an id).
- **The support rows:** `fsim.guidance.must_fly` is partial on every aircraft - zones, corridors and volumes, given or by id, are FA-8b's - and so is `fsim.geometry` (lines, zones and volumes).
- **Surfaces.**
  - C++: `MustFlyCommand`, `MustFlyLocation`, `OpPoint`, `OpPointId` (`fsim/Control.h`); `SetpointKind::MustFly`; `Reason::UnknownGeometry`; `World::setOpPoint`, `removeOpPoint`, `opPoints`, `opPoint`; `MustFlyBehavior` (`fsim/GuidanceModes.h`).
  - C ABI 1.39: `FSIM_MODE_MUST_FLY` (10 fields), `enum fsim_must_fly_location`; `fsim_op_point` (`fsim_op_point_init`), `fsim_world_set_op_point`, `fsim_world_remove_op_point`, `fsim_world_op_point_count`, `fsim_world_get_op_point_at`, `fsim_world_get_op_point`.
  - Python: `vehicle.submit_must_fly(location=, latitude_rad=, longitude_rad=, altitude_m=, target=, ingress_min_rad=, ingress_max_rad=, speed=, ...)`, the location by name or member (`fsim.MustFlyLocation`), a target by vehicle or id; `fsim.OpPoint`; `World.set_op_point`, `op_points`, `op_point`, `remove_op_point`.

### 4.43 A-GRA's must fly: zones, given or by id (as FA-8b1 builds them)

A must fly may name an operational zone (A-GRA's OpZoneID) or be given a zone with it (its ZoneTarget). A zone (A-GRA's ZoneType) is a polygon with holes, an ellipse, a rectangle or a slant range area. It lies within a band of altitudes, on the Earth or in a frame, and may move. ADR-29 plans the must fly into a zone as MFY-04, and zones among the operational geometry (ENV-06). FA-8b1 builds both. Corridors are FA-8b2's; volumes are FA-8b3's.

- **A zone** (`OpZone`; `shape` a `ZoneShape`):
  - A polygon: 3 to 32 vertices in order (`ZoneVertex`: a latitude and longitude, or x and y in its frame), simple, with up to 4 holes of 3 to 32 vertices each, inside it and apart (A-GRA's bounding and interior rings).
  - An ellipse: its centre, its semi-major and semi-minor axes, and the major axis's bearing (`orientationRad`, within a quarter turn of north).
  - A rectangle: its centre, its `widthM` across and `heightM` along its bearing.
  - A slant range area: a point, its greatest range from it and its least (left out, 0), and its bearings from the point, from the least clockwise to the most (both, each within half a turn), turned by its orientation.
  - Its band: `altitudeMinM` and `altitudeMaxM`, in its reference (MSL where none is given). Either may be left out: open that way.
  - On the Earth, or in a frame (`frame`, `frameRotation`). In a frame, its places are x and y metres along the frame's axes from its origin, and the axes are turned by the frame's yaw (or its track, or not at all). A zone is flat.
  - Or moving: a velocity north and east, from `timeS`. Left out, `timeS` is when the zone is set or commanded.
  - Checked as A-GRA's schema restricts it. A fault is refused `invalid_parameter`, naming the field from 10 on, after the must fly's ten: 10 the shape, 11 the vertices (too few or many, off the Earth, not simple), 12 the holes (too many, not simple, not inside it, crossing), 13 the centre, 14 the dimensions (not above 0, a semi-minor axis above the semi-major, a least range not below the greatest), 15 the bearings, 16 the band (upside down, or a reference without it), 17 the frame (one the world does not have, or a rotation without one), 18 the velocity (one way alone, with a frame, or a time without one). A field another shape uses, given, is refused as that field.
- **Laid out** in a plane at the zone's reference: a polygon's vertices' mean (in a frame, the frame's origin), the other shapes' centre. Its coordinates are north and east metres, or along the frame's axes. A zone given is kept as it was laid out when commanded. An operational zone is looked up by id when commanded (or when it starts, where it waits), and read where the world keeps it, never copied. The plane is fixed-size (32 vertices, 4 holes) and kept in the path store beside the route, so nothing allocates as it flies.
- **Where it goes in:**
  - Without a window: the nearest point of the zone's edge to the aircraft. Over one of its holes, that hole's edge.
  - With a window of bearings (4.42): where a bearing from the zone's centre leaves it - the aircraft's own bearing where that lies within the window, else the window's nearer edge moved in by 5 degrees or half the window. It is approached from within the window as a point is.
  - Either way it aims a fifth of the way across the zone further in (200 m at most), and flies there as a route, checked as one, the aim flown over. A moving zone's aim is where the zone will be as the aircraft gets there; a framed zone's is in its frame.
  - The altitude is the one given. Where the zone has a band, that altitude must lie within it, in the band's reference (left out, it takes the band's): otherwise the zone would never be entered, and the command is refused `invalid_parameter` (field 3, or 4 for the reference). Given none, the altitude is the aircraft's where that lies within the band, or the band's nearer edge moved a tenth of the band in (30 m at most, and never past its middle).
  - Already over the zone, the aircraft flies on along its track for 10 s, to that altitude, and completes at once where it is within the band.
- **Completed** once the aircraft is in it: over its area where it is then (moving, or in its frame as the frame is), and within its band. The behaviour tests this each step. Completion does not wait for the route's end, and an UPDATE looks afresh. The route ends in a loiter at its aim - a rotorcraft stops over it, a wing orbits it - so an aircraft that reaches the aim outside the band climbs or descends to it there, and completes once in the zone (from FA-8b3, 4.45; before, it flew on along its course).
- **Operational zones** (ENV-06; A-GRA's OpZone): `World::setOpZone` keeps one by its id, in place of any with that id, its revision one more. With it come `opZones`, `opZone` and `removeOpZone`.
  - Refused `invalid_parameter`: id 0, or any fault above.
  - A moving zone's time left out is when it was set.
  - A must fly naming one the world does not keep, or one whose frame is gone, is refused `unknown_geometry` (field 5).
  - A-GRA's other OpZone data (its category, schedule and the rest) are not kept: nothing flies by them (D8).
- **The command:** its `location` is Zone (the zone given with it: `World::submit(vehicle, MustFlyCommand, OpZone)`) or OpZone (`target` the id).
  - A Zone without a zone given is refused (field 0). Neither takes a place (fields 1 and 2).
  - An UPDATE may give a zone in place of its own. Given none, a Zone keeps the one it was given.
  - A batch's must fly gives it by `BatchCommand::zone`. A task keeps it as it was laid out.
- **The support rows:** `fsim.guidance.must_fly` is partial on every aircraft - corridors and volumes are FA-8b2's and FA-8b3's - and so is `fsim.geometry` (operational lines and volumes).
- **Surfaces.**
  - C++: `OpZone`, `OpZoneId`, `ZoneShape`, `ZoneVertex`, `MustFlyArea`, and `MustFlyLocation::Zone` and `OpZone` (`fsim/Control.h`); `World::submit(vehicle, MustFlyCommand, OpZone)`, `World::update(activity, MustFlyCommand, OpZone)`; `World::setOpZone`, `removeOpZone`, `opZones`, `opZone`.
  - C ABI 1.40: `FSIM_MUST_FLY_ZONE` and `FSIM_MUST_FLY_OP_ZONE`; `enum fsim_zone_shape`, `fsim_zone_vertex`, `fsim_op_zone` (`fsim_op_zone_init`); `fsim_world_set_op_zone`, `fsim_world_remove_op_zone`, `fsim_world_op_zone_count`, `fsim_world_get_op_zone_at`, `fsim_world_get_op_zone`; `fsim_vehicle_submit_must_fly`, `fsim_activity_update_must_fly`, `fsim_activity_update_must_fly_by`; `fsim_batch_command.zone`.
  - Python: `vehicle.submit_must_fly(zone=fsim.OpZone(...), ...)` (its location Zone where a zone is given), `activity.update_must_fly(zone=None, **fields)`; `fsim.OpZone`, `fsim.ZoneVertex`, `fsim.ZoneShape`; `World.set_op_zone`, `op_zones`, `op_zone`, `remove_op_zone`.

### 4.44 A-GRA's must fly: corridors, given or by id (as FA-8b2 builds them)

A must fly may name an operational line (A-GRA's OpLineID) or be given a corridor with it (its LineTarget: "Use of the Line Target implies that a vehicle must fly through a corridor"). A line (A-GRA's LineType) is two vertices or more, on the Earth or in a frame. Each vertex may carry an altitude, an altitude range and widths either side for the segment from it. The line has a projection (great circles or rhumb lines), widths and a band, and an operational line may move. ADR-29 plans the must fly through a corridor as MFY-05, and lines among the operational geometry (ENV-06). FA-8b2 builds both; volumes are FA-8b3's.

- **A line** (`OpLine`):
  - 2 to 32 vertices in the order flown (`LineVertex`): a latitude and longitude, or x and y in its frame; its `altitudeM`; its range `altitudeMinM` to `altitudeMaxM`, in place of the line's band there; their `altitudeReference`; and the `leftWidthM` and `rightWidthM` of the segment from it, in place of the line's.
  - Its `projection` (a route's: great circles, left out, or rhumb lines); its widths left and right of it as it is flown (left out: none); its band and its reference.
  - On the Earth, or in a frame (`frame`, `frameRotation`), as a zone is (4.43); or moving, at a velocity from `timeS`.
  - Checked as A-GRA's schema restricts it. A fault is refused `invalid_parameter`, naming the field from 10 on: 10 the vertices (too few or many; a place not finite, off the Earth or given both ways; two in a row at one place; a vertex's altitude, range, reference or widths, or its altitude outside the band there where both are in one reference), 11 the projection, 12 the widths (negative), 13 the band, 14 the frame, 15 the velocity.
  - A-GRA's vertex Timestamp, and an OpLine's category and schedule, are not kept: nothing flies by them (D8).
- **Laid out** in the plane as a zone is, at its vertices' mean (in a frame, the frame's origin), with the altitude, band and widths at each vertex. It is fixed-size, and kept in the path store, the waiting entry and the task as a zone is.
- **Flown through** as a route, on the line's projection:
  - Its entry is onto its first segment from behind its first vertex: straight in from within 10 degrees of the segment's line there, else through an approach point, as a point's window is flown (4.42). Given a window of bearings from its first vertex, it comes from within that instead.
  - Then every vertex is flown by, and the last flown over. It completes as the last vertex is passed, and flies on along its course.
  - The altitude at each vertex is the one given with the command, within the band there and in its reference (else refused, field 3 or 4, as a zone's). Given none, it is the vertex's own; else the aircraft's, held within the band there (4.43's rule). The approach points fly at the first vertex's.
  - A moving line's vertices are placed where they will be as the aircraft gets there, at the speed it approaches at. A framed line's are in its frame.
- **Its widths:** each turn inside it is checked against them, at the radius the route plans it with.
  - A turn flown by cuts inside by r (1 - cos(a/2)). One flown over (beyond 150 degrees) swings out by r (1 - cos a).
  - Either is held against the lesser width on that side of the segments either side of its vertex (at the first vertex, the first segment's).
  - Beyond it, the command is refused `performance_limit` (`max_turn_rate`), naming the turn's point in the route (its approach points come first). A line without widths is not checked.
  - Between turns the route's follower holds the line. Nothing checks the aircraft against the widths as it flies.
- **Operational lines** (ENV-06; A-GRA's OpLine): `World::setOpLine` keeps one by its id as a zone is kept, its revision one more; with it come `opLines`, `opLine` and `removeOpLine`. Refused `invalid_parameter`: id 0, or any fault above. A must fly naming one the world does not keep, or one whose frame is gone, is refused `unknown_geometry` (field 5).
- **The command:** its `location` is Line (the line given with it: `World::submit(vehicle, MustFlyCommand, OpLine)`) or OpLine (`target` the id).
  - A Line without a line given is refused (field 0).
  - An UPDATE may give a line in place of its own. Given none, a Line keeps the one it was given.
  - A batch's must fly gives it by `BatchCommand::line`, never with a zone. A task keeps it as it was laid out.
- **The support rows:** `fsim.guidance.must_fly` is partial on every aircraft - volumes are FA-8b3's - and so is `fsim.geometry` (operational volumes).
- **Surfaces.**
  - C++: `OpLine`, `OpLineId`, `LineVertex`, and `MustFlyLocation::Line` and `OpLine` (`fsim/Control.h`); `World::submit(vehicle, MustFlyCommand, OpLine)`, `World::update(activity, MustFlyCommand, OpLine)`; `World::setOpLine`, `removeOpLine`, `opLines`, `opLine`.
  - C ABI 1.41: `FSIM_MUST_FLY_LINE` and `FSIM_MUST_FLY_OP_LINE`; `fsim_line_vertex` (`fsim_line_vertex_init`), `fsim_op_line` (`fsim_op_line_init`); `fsim_world_set_op_line`, `fsim_world_remove_op_line`, `fsim_world_op_line_count`, `fsim_world_get_op_line_at`, `fsim_world_get_op_line`; `fsim_vehicle_submit_must_fly_line`, `fsim_activity_update_must_fly_line`, `fsim_activity_update_must_fly_line_by`; `fsim_batch_command.line`.
  - Python: `vehicle.submit_must_fly(line=fsim.OpLine(...), ...)` (its location Line where a line is given), `activity.update_must_fly(line=...)`; `fsim.OpLine`, `fsim.LineVertex`; `World.set_op_line`, `op_lines`, `op_line`, `remove_op_line`.

### 4.45 A-GRA's must fly: volumes, given or by id (as FA-8b3 builds them)

A must fly may name an operational volume (A-GRA's OpVolumeID) or be given a volume with it (its VolumeTarget: "Use of the Volume Target implies that a vehicle must enter the volume"). A volume (A-GRA's OpVolumeType) is one of three kinds. It may be geometric: a sphere, a dome, an ellipsoid, a cylinder, a cone or a rectangular cone at a point, moving with it. It may be geocentric: between latitudes, longitudes and altitudes. Or it may be orbital. ADR-29 plans the must fly into a volume as MFY-06, and volumes among the operational geometry (ENV-06). FA-8b3 builds both; with them the must fly and the operational geometry are whole on every aircraft.

- **The volumes of the atmosphere** (`OpVolume`; `shape` a `VolumeShape`):
  - A sphere (`radiusM`) round its point; a dome (`radiusM`), the half of a sphere above its point's level.
  - An ellipsoid (`semiAxisAM`, `semiAxisBM`, `semiAxisCM`) round its point, along its x, y and z.
  - A cylinder (`radiusM`; `lengthM`, left out without end) from its point along its x.
  - A cone (`halfAngleRad`) and a rectangular cone (`lengthHalfAngleRad` in its x-y plane, `widthHalfAngleRad` in its x-z plane): their vertex its point, their axis its x, out to `rangeM` from it (left out, without end).
  - Its axes are turned by `yawRad`, `pitchRad` and `rollRad` from north-east-down at its point (in a frame, from the frame's turned axes). A-GRA gives an ellipsoid its attitude as a quaternion, and a cone's axis only through orbital kinematics; here each shape with an axis is turned by its own. A vertical column is a cylinder pitched 90 degrees.
  - Its point is on the Earth, at its `altitudeM` in its `altitudeReference`. Or it is in a frame (`frame`, `frameRotation`): x and y along the frame's turned axes, and, its altitude left out, the frame's origin's - so a volume in the frame that follows another vehicle moves with it. Or it moves, at a velocity north, east and down from `timeS`.
  - A geocentric volume: `latitudeMinRad` to `latitudeMaxRad`, `longitudeMinRad` clockwise to `longitudeMaxRad`, `altitudeMinM` to `altitudeMaxM` (either left out: open that way).
  - Checked as A-GRA's schema restricts it. A fault is refused `invalid_parameter`, naming the field from 10 on: 10 the shape, 11 the point (off the Earth, or on it without its altitude), 12 the dimensions (not above 0, a half angle not below a quarter turn, or another shape's), 13 the attitude (on a sphere, a dome or a geocentric volume), 14 the geocentric bounds, 15 the frame, 16 the velocity.
- **Not supported:** A-GRA's orbital volumes lie outside the atmosphere every aircraft here flies in. These are its ArcVolume and IncRaPeriodVolume shapes, orbital kinematics and a body's local position, orbit regimes, orbit altitudes and its qualitative regions. The C++, C and Python surfaces do not express them.
- **Laid out** in the plane at its point as a zone is (a geocentric volume's at its bounds' middle), with its axes, and kept as a zone's is.
- **Where it goes in.** Every shape is convex, so a containment test finds it.
  - Its height is the altitude given, which must lie inside it over a point well inside it, in its reference (else field 3 or 4). That point is its centre, a dome's halfway up, a cylinder's or a cone's along its axis. Given none, it is the aircraft's, held between its top and bottom over that point as a zone's band holds it: a tenth of the way in, 30 m at most, or its middle where it is thinner.
  - At that height it aims toward that point from the aircraft - or along a bearing from it within a window of bearings - finds its edge, and goes a fifth of the way across it further in (200 m at most, never past that point). Over it already, the aircraft flies on as it flies for ten seconds.
  - It is flown there as a route, as a zone's is. A moving volume's aim is where it will be, its height too.
- **Completed** once the aircraft is in it, tested each step in three dimensions as the volume is then: moving, or where its frame's vehicle is.
- **Its route ends in a loiter at its aim** - a rotorcraft stops over it, a wing orbits it - so an aircraft that reaches the aim before it is in (still climbing to it) goes on until it is. A zone's route ends so too now (4.43).
- **Operational volumes** (ENV-06; A-GRA's OpVolume): `World::setOpVolume` keeps one by its id as a zone is kept; with it come `opVolumes`, `opVolume` and `removeOpVolume`. A must fly naming one the world does not keep, or one whose frame is gone, is refused `unknown_geometry` (field 5).
- **The command:** its `location` is Volume (the volume given with it: `World::submit(vehicle, MustFlyCommand, OpVolume)`) or OpVolume (`target` the id). The rest is as a zone's: a Volume without a volume is refused (field 0); an UPDATE may give one in place of its own, and given none keeps it; a batch's must fly gives it by `BatchCommand::volume`; a task keeps it as it was laid out.
- **The support rows:** `fsim.guidance.must_fly` and `fsim.geometry` are supported on every aircraft.
- **Surfaces.**
  - C++: `OpVolume`, `OpVolumeId`, `VolumeShape`, and `MustFlyLocation::Volume` and `OpVolume` (`fsim/Control.h`); `World::submit(vehicle, MustFlyCommand, OpVolume)`, `World::update(activity, MustFlyCommand, OpVolume)`; `World::setOpVolume`, `removeOpVolume`, `opVolumes`, `opVolume`.
  - C ABI 1.42: `FSIM_MUST_FLY_VOLUME` and `FSIM_MUST_FLY_OP_VOLUME`; `enum fsim_volume_shape`, `fsim_op_volume` (`fsim_op_volume_init`); `fsim_world_set_op_volume`, `fsim_world_remove_op_volume`, `fsim_world_op_volume_count`, `fsim_world_get_op_volume_at`, `fsim_world_get_op_volume`; `fsim_vehicle_submit_must_fly_volume`, `fsim_activity_update_must_fly_volume`, `fsim_activity_update_must_fly_volume_by`; `fsim_batch_command.volume`.
  - Python: `vehicle.submit_must_fly(volume=fsim.OpVolume(...), ...)` (its location Volume where a volume is given), `activity.update_must_fly(volume=...)`; `fsim.OpVolume`, `fsim.VolumeShape`; `World.set_op_volume`, `op_volumes`, `op_volume`, `remove_op_volume`.

### 4.46 A-GRA's altitude stacked marshall (as FA-8c builds it)

A-GRA's ALTITUDE_STACKED_MARSHALL (MA_AltitudeStackedMarshallType) is a loiter flown by each aircraft of a stack at an altitude of its own. The loiter is an orbit - a circle, or a racetrack or figure-eight by two circles, with a turn direction and a duration by time or laps - or a hover at a point for a duration. The stack has a minimum altitude, and optionally a maximum and a minimum separation. The inventory reads it as "each vehicle takes and keeps a slot in the stack with its peers" (ASM-01). ADR-29 plans it as ASM-01, FA-8c.

- **The mode** (`MarshallCommand`, fsim.guidance.marshall) has thirteen fields, then the pattern's shape. It is not one of the command variant's (below): its activity flies its pattern.
  - Its pattern (`PatternKind`): an orbit, left out; a racetrack or a figure-eight, their second circle in the pattern's shape as A-GRA gives it; or a rotorcraft's hover.
  - Its centre, its circle's radius and turn, a speed and its reference, and a duration (its laps in the shape's `orbits`).
  - Its stack: `altitudeMinM` (given), `altitudeMaxM` (left out: none) and `separationM` (left out: 1,000 ft).
  - Its slot reads back in `altitudeM`. Given in a NEW, it is the slot asked for.
  - Refused `invalid_parameter`, naming the field: a hold, which is ATC's, not a marshall's (0); a racetrack or a figure-eight without its second circle (18); a reference that is not one (4); the stack's least left out or not finite (10); a most below it (11); a separation not above 0 (12); a slot asked for outside them (3).
- **Its slot** is the world's choice, made between steps on the caller's thread, so the same commands choose the same slots.
  - It is the lowest from the stack's least in steps of its separation that is clear by the separation of every other aircraft's marshall round the same point - their centres within 100 m, live or waiting, in the same altitude reference - and no higher than its most. One asked for must be clear of the others.
  - None clear: refused `stack_full`, a reason of its own, naming field 3.
  - An UPDATE that moves the stack - its least, most, separation, centre or reference, or a slot asked for - chooses afresh. One that leaves it (a speed, a radius) keeps its slot. A slot freed as a marshall ends goes to the next NEW, and a waiting marshall holds its slot.
- **Flown** as its pattern at its slot, by fsim.guidance.pattern's follower: a wing orbits there, a rotorcraft hovers there (the hover a rotorcraft's, R1, as the pattern's). Its NEW and UPDATE are checked as a pattern's, the pattern's fields named back as the marshall's. With a duration or laps it completes, and flies on.
- **Beside the command variant, not in it.** The activity's command is its pattern - a `PatternCommand` at its slot, the one its follower flies - and its stack (`MarshallStack`: its least, most and separation) is kept beside it, in the path store while it flies and in its waiting entry while it waits, as a must fly's zone is (4.43).
  - Why: libstdc++ visits a variant of eleven alternatives or fewer through a switch, and one of more through a table of calls. The command variant has eleven since the must fly (4.42). A twelfth made every copy, move and visit of a command dispatch through that table: measured against FA-8b3, the step cases that run the control stack's merged path 2 to 4 % slower from three copies each, and a NEW of another level 4 % slower, in both builds. `ControlStack::flyMerged` itself compiled 31 instructions shorter, the variant's move-assignment inlined through the table where it had been a call.
  - As built, the variant is FA-8b3's: `flyMerged` the same instruction for instruction, and a static assertion holds the variant to eleven. A mode after it flies as one of the eleven, with its own fields beside it.
  - A marshall's pattern takes an UPDATE through the marshall's alone: a `PatternCommand`'s is refused `wrong_command_type`.
  - Read back: the setpoint's `command` is its pattern at its slot, and its `marshall` the marshall, its stack with it (none for any other activity). A batch item or a task gives one in place of its command (`BatchCommand::marshall`).
- **Never a task:** its slot is the stack's at its NEW, so a marshall is not kept as a task (refused `invalid_parameter`), nor suggested as one.
- **On the way to its slot** the aircraft climbs or descends from where it is; nothing keeps it apart from the others on the way. A stack's entry is its tasking's.
- **The support rows:** `fsim.guidance.marshall` is supported on every aircraft, and `fsim.guidance.marshall/hover` as `fsim.guidance.pattern/hover` is (R1).
- **Surfaces.**
  - C++: `MarshallCommand`, `MarshallStack` and `kMarshallSeparationM` (`fsim/Control.h`), `Reason::StackFull` and `SetpointKind::Marshall` (`fsim/Capability.h`); `World::submit(vehicle, MarshallCommand, PatternShape)`, `World::update(activity, MarshallCommand[, PatternShape])`; `Setpoint::marshall`, `BatchCommand::marshall`, `PathStore::marshall`.
  - C ABI 1.43: `FSIM_MODE_MARSHALL` (35 fields: its 13, then the pattern's shape's), through `fsim_vehicle_submit_mode` and `fsim_activity_update`; the reason `stack_full`.
  - Python: `vehicle.submit_marshall(...)` with its fields by name (`fsim.MODE_FIELDS["marshall"]`), `activity.update(**fields)`.

### 4.47 A-GRA's route intercept (as FA-8d builds it)

A-GRA's ROUTE_INTERCEPT (MA_RoutePlanInterceptType) joins a route plan the vehicle keeps. It names the plan (RoutePlanID), the path to intercept (PathID), the method (MA_RouteInterceptEnum) and the earliest and latest segments it may join (ActivationPathSegmentType: a path and a segment). The schema defines the three methods by a line each: DISCRETE, "interception limited to entering at individual Endpoints"; SHORTEST_DISTANCE, "intercept point with the shortest path length"; SOONEST, "intercept point with the shortest time to intercept". Left out, it intercepts "at the beginning of the RoutePlan". Its activity reports its status (MA_RoutePlanInterceptStatusType): the plan's execution, and the previous, current and next segments - each its plan, path and segment, the estimated capture time and distance, the loiter's progress and the aircraft's heading and velocity relative to it. A-GRA's ICD and VI volume say nothing more. ADR-29 plans it as RIC-01 to RIC-03, FA-8d.

- **The mode** (`InterceptCommand`, fsim.guidance.intercept) has five fields. Like the marshall (4.46) it is not one of the command variant's: its activity flies the plan's route, a `RouteCommand`, and keeps the intercept beside it.
  - `plan`: the plan's id (a whole number up to 2^53, as the modes' ids are).
  - `path`: the path to join, its id as the plan gives it (`RoutePath::id`); left out, any.
  - `method` (`InterceptMethod`): `Discrete`, `ShortestDistance` or `Soonest`; left out, the plan's beginning.
  - `earliest`, `latest`: the first and last segments it may join, each by its end point's index as given. A-GRA's segment is the leg into its end point, and a point is named by its index everywhere (4.36).
- **Its candidates:** the plan's points in their flight order (4.36) - from its start, once round; on a path given, from that path's first point until the path is left - from the earliest to the latest. A candidate's segment is the leg into it from the point before it in that order. The first point in that order has none.
- **The join,** from where the aircraft is:
  - left out: the first candidate, flown to directly - the plan's beginning;
  - `Discrete`: the candidate point nearest the aircraft over the ground (on a tie, the earlier), flown to directly;
  - `ShortestDistance`: the point on a candidate's segment nearest the aircraft - where the perpendicular from it meets the leg, or the leg's end;
  - `Soonest`: the point on a candidate's segment the aircraft reaches first, turning from its track toward it at its speed and 80 % of its bank (a rotorcraft at once), then straight to it at its ground speed in the wind.
  - A join on a leg, short of its end point, is laid into the route as one point more after the plan's, its index the plan's count: flown by, at the leg's speed and at its altitude along the profile there, its next the leg's end point. The plan's last point, where its next was left out and the plan has no paths, ends the route there (next -1). With paths, the point is in a path of its own.
  - Only a straight leg between two points is joined short of its end point. An arc, a leg a civil path terminator lays, a leg out of a loiter point and a leg in a frame are joined at their end points. So is every leg of a plan with 256 points or 16 paths, which leaves no room for the point.
- **Flown** from the join as the plan's activation flies it from its start (4.39): the plan's route, with its loiters, planned states, paths, branches and civil path terminators, checked as its NEW is.
- **The plan** must be kept (`unknown_plan`), uploaded, and in a state its activation may leave from: Uploaded, ReadyForActivation, either failed state, Deactivated, or Activated (else `wrong_plan_state`). A plan for planning use only is never flown (`planning_only`).
  - Its activity may be live - its activation's, or an intercept's: the new intercept replaces it, as any NEW does. That is the rejoin: a plan superseded by a diversion, or one flying, joined again from where the aircraft is.
  - Accepted, the plan is Activated, its activity the intercept's, and its execution is reported as an activation's (4.39). Refused or validated, the plan is unchanged.
- **Refused `invalid_parameter`, naming the field:** a plan id that is not a whole number above 0 (0); a path the plan does not have (1); a method that is not one (2); an earliest or a latest that is not a point in the flight order, or not on the path given (3, 4); an earliest after the latest (4).
- **Waiting,** it keeps the plan's route as it was laid at its NEW, and chooses its join afresh as it starts, from where the aircraft is then, in room its NEW made (a start allocates nothing). A resumed one (disabled, unassigned: 4.10) resumes as a route does.
- **No UPDATE** (`not_updatable`): a new intercept replaces it. It is never kept as a task (`not_implemented`), nor suggested as one.
- **Its status** (`World::interceptStatus`, A-GRA's MA_RoutePlanInterceptStatusType):
  - the plan and its execution - an ended one's, its plan while it is still that plan's last activation;
  - the join: the segment it joined and, where a point was laid in for it, that point and its place;
  - the previous, current and next segments. Each has its path's id, its end point (index and id), the estimated capture time and distance (along the route at the ground speed now; the previous: when it was captured), its loiter's orbits completed and end time where it loiters there, and the segment's heading (from the point before its end point) with the aircraft's ground velocity along and across it.
  - Flying to a point laid in for the join, the current segment is the leg it joins.
- **The support row:** `fsim.guidance.intercept` is supported on every aircraft: it needs nothing a route does not.
- **Surfaces.**
  - C++: `InterceptCommand`, `InterceptMethod` and `InterceptJoin` (`fsim/Control.h`); `InterceptStatus`, `SegmentStatus`, `SegmentEstimate` and `SetpointKind::Intercept` (`fsim/Capability.h`); `World::submit(vehicle, InterceptCommand)`, `World::interceptStatus(activity)`; `Setpoint::intercept` and `Setpoint::join`, `BatchCommand::intercept`, `PathStore::intercept`; `Behavior::segments`, a route's.
  - C ABI 1.44: `FSIM_MODE_INTERCEPT` (5 fields), through `fsim_vehicle_submit_mode`; `fsim_activity_intercept_status` (`fsim_intercept_status`, `fsim_segment_status`); `enum fsim_intercept_method`.
  - Python: `vehicle.submit_intercept(plan=, path=, method=, earliest=, latest=)`, `fsim.InterceptMethod`; `activity.intercept_status()` (`fsim.InterceptStatus`, `fsim.SegmentStatus`). A batch item's `BatchCommand("submit_intercept", ...)`: its method name is positional alone, as an intercept's field is its `method`.

### 4.48 A rotorcraft's airspeed within its top level speed (after FA-3e)

A rotorcraft asked for an airspeed it cannot fly is held to its fastest, as a wing is held to its envelope and its performance section's top speed. Before, a multirotor's airspeed was held to nothing, and a helicopter's only to its never-exceed speed, which lies above its top level speed (the UH-60A's 193 kt against 187).

- **The bound** is the performance tables' top level speed (4.13) at the altitude the command flies at, and at the weight now: the fuel on board, weighed as a speed optimisation weighs it (`topTasMs`, fsim/GuidanceModes.h).
  - It is along the nose, as a rotorcraft flies an airspeed and its tables have it.
  - A calibrated airspeed or a Mach number is converted there through the standard atmosphere, as a wing's limits are.
  - An altitude above the ground counts as sea level, as a wing's does: the tables' lowest row.
  - Above the altitudes the tables fly (3,000 m) they give no top, and nothing bounds the airspeed. The performance profile gives no most there either.
- **Where it applies:**
  - an hsa's speed and a pattern's, at the altitude flown to;
  - each route point's, at its own altitude;
  - the velocity level's airspeed, at the altitude now.

  A ground speed stays within the position loop's fastest, and a curve's speeds are ground speeds, as before.
- **Checked as every limit is** (4.8):
  - under Clamp, held to the bound and flagged `kClamped`, with `max_airspeed`;
  - under Reject, refused `performance_limit` with `max_airspeed`, the field or point named, and with the suggestion of any clampable finding (4.11);
  - unchecked (`RangePolicy::None`, as a trainer's per-step commands are), flown as asked.
- **The performance profile agrees.** Its most airspeed at each of the tables' altitudes (4.15) is what a command there is held to, within a millionth on all four. The difference is the profile's weight, which is the flight model's.
- **Flown at the bound**, each holds it: after four minutes all four are within 0.011 % of it and within 2 mm of their height (the fleet test). On the way:
  - The Crazyflie's velocity loop takes over two minutes to reach it. At its top the loop's integral carries its whole tilt (4.17), and the integral is slow far from its target.
  - The UH-60A sags 96 m at full collective and is back on its height 70 s after the command. It does the same accelerating to any speed near its top: 59 m to 90 m/s, none to 70.
- **Unchecked beyond the bound:**
  - A multirotor's velocity loop saturates at its tilt and holds its fastest, a little past its tables' top: the Crazyflie 18.30 m/s along its nose against 18.0, the IRIS+ 13.23 against 13.09. Asked less, it comes back.
  - A helicopter's flies past its top and its never-exceed speed, and cannot hold its height. The UH-60A, asked 150 m/s, flew 99.7 and strayed 128 m from its height. The UH-1H, asked 100, flew 55 to 59 m/s and strayed 19.5 m.
- **What FA-3e had taken for a divergence of the Crazyflie's loops.** Asked for 25 and 30 m/s, it had diverged; its loops were not the cause.
  - The probe let the Crazyflie go for 5 s before the command. Under the neutral vehicle default (`VehicleDefault::Neutral`) its motors stood still, and it fell 124 m.
  - The hsa began 27 m up, falling at 49 m/s. At full thrust it struck the ground at 35 m/s, was thrown up at 80 m/s tumbling at 50 to 70 rad/s, and diverged. This is the crash the contact model does not end ([rotorcraft.md](rotorcraft.md), 7).
  - Asked for 10 m/s, it diverges the same way. Settled first, it flies its fastest at 25, 30, 40 and 60 m/s and never diverges.
- **Surfaces.** C++: `topTasMs`. No C ABI or Python change: the answers are the existing `kClamped`, `performance_limit` and `max_airspeed` (Python: `Activity.clamped`, `fsim.Rejected`, `World.last_command_details`).

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
- FA-3e, speed optimisation (HSA-05, LTR-17, 4.17) and endurance validation (VAL-03, 4.18), the first soft rejection override_rejection overrides, done 2026-09-27 and measured in section 14;
- after FA-3e, a rotorcraft's airspeed held to its tables' top level speed (4.48), done 2026-09-27, merged 2026-09-29 and measured in section 14. It closes what FA-3e found: a Crazyflie asked beyond its top had seemed to diverge.

**Supporting models:** Performance tables, fuel flow (SUB-02, SUB-03).

**Items (20):** HSA-05, HSA-10; LTR-17; VAL-03; CAP-04, CAP-05, CAP-06, CAP-07, CAP-08, CAP-09, CAP-10, CAP-11, CAP-12, CAP-13, CAP-14, CAP-15; CTG-04; STS-07; SUB-02, SUB-03.

**Accepted when:**

- Each aircraft's profile values within 5 % of hangar's flight tests (level speeds, climb, stall).
- In the fleet climb case no mode lets CAS fall below 1.1 times the minimum.
- Best-range and best-endurance speeds within 5 % of hangar's optimum; endurance within 5 % of a flown burn.
- Changed flights (energy management) measured and listed; others identical.

### FA-4: References and state data (M)

Magnetic and barometric references in every mode and in the state; the QNH setting; reference frames; the terrain query; winds; orientation acceleration; terrain validation of commanded paths.

**Status:** done 2026-09-27 in four steps, each measured in section 14:
- FA-4a, the terrain: the query and the paths checked against it (ENV-01, STS-11, VAL-06; 4.19), done 2026-09-27 and measured in section 14;
- FA-4b, the barometric altimeter: the QNH setting, what the altimeter reads in the state data, the barometric reference in the hsa and the patterns (ENV-03, STS-10, STS-04, HSA-07, LTR-16; 4.20), done 2026-09-27 and measured in section 14;
- FA-4c, the state data and frames: orientation rates and accelerations, the wind, reference frames (STS-02, STS-06, ENV-04; 4.21), done 2026-09-27 and measured in section 14;
- FA-4d, the magnetic model: declination, the magnetic reference and heading (ENV-02, HSA-03, STS-05; 4.22), done 2026-09-27 and measured in section 14. The World Magnetic Model 2025's coefficients were downloaded from NOAA NCEI with the owner's approval.

**Supporting models:** Terrain service, magnetic model, altimeter, frames (ENV-01 to ENV-04).

**Items (14):** HSA-03, HSA-07; LTR-16; VAL-06; STS-02, STS-04, STS-05, STS-06, STS-10, STS-11; ENV-01, ENV-02, ENV-03, ENV-04.

**Accepted when:**

- Declination against the World Magnetic Model's published test values; indicated altitude against the standard atmosphere at a non-standard QNH.
- Every mode accepts every reference; a path into terrain refused violation_terrain with the point. Digests identical.

### FA-5: Loiter and curves as the schema defines them (L)

Laps, entry and exit points, legs by time, turns by bank, rate or type, hold contexts and entries, two-circle patterns, the hover loiter with a duration; general NURBS curves with their references, curvature and indices.

**Status:** done 2026-09-28 in four steps, each measured in section 14:
- FA-5a, A-GRA's orbit: two circles, the fix-point orbit's heading, leg time and bank, laps, entry and exit points (LTR-03, LTR-05, LTR-06, LTR-07, LTR-10; 4.23), done 2026-09-28 and measured in section 14;
- FA-5b, the hold: its turns by rate and type, its duration by entry and exit times, its entries and context (LTR-11 to LTR-14; 4.24), done 2026-09-28 and measured in section 14;
- FA-5c, the hover loiter and relative points (LTR-15, LTR-18; 4.25), done 2026-09-28 and measured in section 14;
- FA-5d, curves as the schema gives them, in three steps:
  - FA-5d1, general NURBS with their curvature and indices (CRV-03, CRV-08; 4.26), done 2026-09-28 and measured in section 14;
  - FA-5d2, the curve's reference and its control points' offsets: an altitude reference and range, a frame, rotations, geodetic offsets and altitude choices (CRV-04, CRV-05, CRV-06; 4.27), done 2026-09-28 and measured in section 14;
  - FA-5d3, a rotorcraft's circular loiter at the curve's end (CRV-11; 4.28), done 2026-09-28 and measured in section 14.

**Items (17):** CRV-03, CRV-04, CRV-05, CRV-06, CRV-08, CRV-11; LTR-03, LTR-05, LTR-06, LTR-07, LTR-10, LTR-11, LTR-12, LTR-13, LTR-14, LTR-15, LTR-18.

**Accepted when:**

- Pattern and curve geometry within the bounds VI-5 and VI-6 set, per class; each hold entry flown as specified. Digests identical.

### FA-6: Routes as the schema defines them (XL)

Paths with ids and types, links and conditional branches, turn points, loiter points, per-segment optimisation, climb and acceleration, required times of arrival in 4D, altitude blocks, civil path terminators, planned states, RNP monitoring, relative points.

**Status:** done 2026-09-29 in seven steps, each measured in section 14:
- FA-6a, the waypoint as the schema gives it: altitude blocks and the barometric reference, waypoints and their types, points in frames (WPT-12, WPT-17, WPT-22; 4.29), done 2026-09-28 and measured in section 14;
- FA-6b, turn points and loiter points, in two steps:
  - FA-6b1, turn points: capturing the outbound course, starting and ending a turn, a course at the point, a turn's radius (WPT-04; 4.30), done 2026-09-28 and measured in section 14;
  - FA-6b2, a loiter inside a route, then on (WPT-18; 4.31), done 2026-09-28 and measured in section 14;
- FA-6c, per-segment performance, in two steps:
  - FA-6c1, speed optimisation and acceleration (WPT-06, WPT-10; 4.32), done 2026-09-28 and measured in section 14;
  - FA-6c2, climb optimisation (WPT-08; 4.32), done 2026-09-28 and measured in section 14;
- FA-6d, 4D, in three steps, done:
  - FA-6d1, required times of arrival (WPT-11; 4.33), done 2026-09-28 and measured in section 14;
  - FA-6d2, planned inertial states (WPT-20; 4.34), done 2026-09-28 and measured in section 14;
  - FA-6d3, required navigation performance (WPT-21; 4.35), done 2026-09-28 and measured in section 14;
- FA-6e, paths, in two steps, done:
  - FA-6e1, paths with ids and types, and their links (WPT-13, WPT-14; 4.36), done 2026-09-28 and measured in section 14;
  - FA-6e2, conditional branches (WPT-15), in two steps:
    - FA-6e2a, the branches, and their altitude, time, capture and operator-input conditions (4.37), done 2026-09-28 and measured in section 14;
    - FA-6e2b, their endurance and contingency conditions (4.37), done 2026-09-28 and measured in section 14; FA-6e done;
- FA-6f, civil path terminators (WPT-19; 4.38), in two steps:
  - FA-6f1, the legs that end at their fix - a track, a direct, an initial fix, a course and a radius to fix - done 2026-09-28 and measured in section 14;
  - FA-6f2, the legs A-GRA gives no data for, in two steps:
    - FA-6f2a, the legs to an altitude, an intercept and a distance, and the heading legs, done 2026-09-28 and measured in section 14;
    - FA-6f2b, the legs to a manual termination, and the holds, done 2026-09-28 and measured in section 14; FA-6f done;
- FA-6g, what FA-6 has left, in three steps:
  - FA-6g1, a start turn where its links loop back, its course left out (WPT-14; 4.36), done 2026-09-28 and measured in section 14;
  - FA-6g2, a time of arrival at or after a loiter point (WPT-11; 4.33), done 2026-09-28 and measured in section 14;
  - FA-6g3, planned states (WPT-20; 4.34), in two steps:
    - FA-6g3a, at or after a loiter point, done 2026-09-29 and measured in section 14;
    - FA-6g3b, beside points in moving frames, done 2026-09-29 and measured in section 14; FA-6g done, and FA-6 with it.

**Items (15):** WPT-04, WPT-06, WPT-08, WPT-10, WPT-11, WPT-12, WPT-13, WPT-14, WPT-15, WPT-17, WPT-18, WPT-19, WPT-20, WPT-21, WPT-22.

**Accepted when:**

- Branches taken as their conditions say; arrival within 2 s of a feasible RTA; RF arcs within 20 m; RNP alerts; per class. Digests identical.

### FA-7: Route plans and airfields (M)

Plans by id and version, their activation states, FA-owned read-only plans, airfields and runways, queries, validation and patches, execution status.

**Status:** done 2026-09-29 in three steps, each measured in section 14:
- FA-7a, the plan store: its activation states, execution, queries and the route's planning metadata (RPL-01, RPL-02, RPL-05, RPL-08, RPL-09, RPL-10, RPL-11, WPT-23; 4.39), done 2026-09-29 and measured in section 14;
- FA-7b, FA's own plans and the airfields (RPL-03, RPL-04, ENV-05; 4.40), done 2026-09-29 and measured in section 14;
- FA-7c, validation with weather, and patches (RPL-06, RPL-07, ENV-10; 4.41), done 2026-09-29 and measured in section 14; FA-7 done.

**Supporting models:** Airfields, weather for validation (ENV-05, ENV-10).

**Items (14):** WPT-23; RPL-01, RPL-02, RPL-03, RPL-04, RPL-05, RPL-06, RPL-07, RPL-08, RPL-09, RPL-10, RPL-11; ENV-05, ENV-10.

**Accepted when:**

- The VI's route-plan sequences (1.2.4 to 1.2.6) replayed as tests in SDK terms. Digests identical.

### FA-8: Must fly, stacked marshall, route intercept (L)

Three of the missing capability types.

**Supporting models:** Operational geometry (ENV-06).

**Items (13):** MFY-01, MFY-02, MFY-03, MFY-04, MFY-05, MFY-06, MFY-07; ASM-01; RIC-01, RIC-02, RIC-03; CAP-02; ENV-06.

**Status:** in six steps:
- FA-8a, must fly a point, an entity or an operational point, from within a window of bearings; operational points (MFY-01, MFY-02, MFY-07; MFY-03 and ENV-06 for points; 4.42), done 2026-09-29 and measured in section 14;
- FA-8b1, must fly a zone, given or by id; operational zones (MFY-04; MFY-03 and ENV-06 for zones; 4.43), done 2026-09-29 and measured in section 14;
- FA-8b2, must fly a corridor, given or by id; operational lines (MFY-05; MFY-03 and ENV-06 for lines; 4.44), done 2026-09-29 and measured in section 14;
- FA-8b3, must fly a volume, given or by id; operational volumes (MFY-06; MFY-03 and ENV-06 for volumes; 4.45), done 2026-09-29 and measured in section 14: FA-8b done, and with it the must fly and the operational geometry;
- FA-8c, the altitude stacked marshall (ASM-01; 4.46), done 2026-09-29 and measured in section 14;
- FA-8d, the route intercept (RIC-01 to RIC-03; CAP-02; 4.47), done 2026-09-29 and measured in section 14: FA-8 done.

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

**FA-4d, the magnetic model (ENV-02, HSA-03, STS-05).** It closes FA-4, whose acceptance it completes: the declination against the World Magnetic Model's published test values.
- **The model against NOAA's published test values** (`test_magnetic`):
  - the technical report's twelve (2025.0 and 2027.5, at sea level and 100 km, 80° N, the equator and 80° S) are within the rounding they are given to: 0.05 nT for every component and intensity, 0.005° for the declination and inclination;
  - the coefficient package's hundred (2025.0 to 2029.5, heights to 94 km, the whole Earth) are within 7.2e-4 nT, where they are given to a millionth, and within their 0.005° rounding.
- **Flown** (a C172x off San Francisco, where the declination is 12.9° in 2026):
  - told magnetic north, it commands a true heading within 0.01° of the declination, and its state data's magnetic heading is its heading turned back;
  - its heading hold flies it as closely as it flies a true heading: 1.6° off after this left turn, the same told true north. That is a separate finding, set aside as a task of its own (closed at the end of this section: the hsa trims its heading);
  - an UPDATE's heading alone continues the magnetic reference; a reference alone in an UPDATE is refused; a reference given alone in a NEW holds the heading now, from that north;
  - a course from magnetic north is flown over the ground within 1° of it turned true.
- **The fleet** (`test_fleet`): every aircraft told a magnetic heading a quarter turn right commands its true heading within 0.008° of it turned by the declination now (the worst, the Su-25's, moving between the 10 s it is refreshed at), and flies it within 0.17° (the Su-25).
- **The date:** a world whose clock was never set reads the model's epoch; the 2026 clock reads 2026; 2033 reads 2030.0.
- **Found in passing:** `ALTITUDE_REFERENCE` in `fsim.agra` had no name for FA-4b's barometric reference; reading one back would have failed. It has one now.
- **Digests:** identical to FA-4c's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-4c (338f948), built in the scratch worktree: 5 rounds of `micro`, and 9 of `command` three times.
  - The `hsa` micro case read +7.6 % while the mode wrapped every direction, true ones too. It now turns and wraps only a magnetic one, and reads +0.0 %. The other micro cases are within −3.9 % to +1.2 %.
  - The command cases are within −3.1 % to +2.0 %.
  - World throughput, over 7 rounds on a loaded machine (both builds' F-16C a sixth below the earlier runs'), is 98.4 to 100.0 % of FA-4c's. Protection costs at most 2.5 % (the gate: 97 %).
- ctest: all 267 tests pass.

**A rotorcraft's airspeed within its top level speed (after FA-3e; 4.48).**
- **FA-3e's finding, flown again.** FA-3e's probe let the Crazyflie go at 150 m for 5 s, then asked it for 25 m/s; it diverged, and so it did at 30. The flights below are 60 s each, asked for 0, 10, 25, 30 and 40 m/s by heading and by course.
  - Let go first, under the neutral default its motors stood still. The command found it 27 m up, falling at 49 m/s. The Crazyflie struck the ground at 35 m/s and diverged at every speed but 0, including 10 m/s. The IRIS+ struck it too, and flew on.
  - Settled first on its velocity loop for 10 s, neither quadrotor diverged at any speed. Each flew the fastest its tilt gives: the Crazyflie 18.29 m/s along its nose (20.0 true), the IRIS+ 13.22.
  - The velocity, attitude and allocation loops needed no change.
- **The bound** (`test_modes`, and a probe at 150 m). On all four, an hsa, a pattern, a route point and a velocity command beyond the top are refused `performance_limit` with `max_airspeed` under Reject, the field or point named, and clamped to the top otherwise. An UPDATE is checked the same way.
  - Asked at 150 m: the Crazyflie 30 m/s → 18.0; the IRIS+ 40 → 13.09; the UH-1H 100 → 64.25 (its never-exceed speed there), then 56.28; the UH-60A 150 → 100.0, then 96.28.
  - A calibrated airspeed and a Mach number are converted at the altitude asked.
  - The F-16C's 600 m/s is still held to its 446, and a wing's velocity level is checked as before.
- **The performance profile** (`test_performance_profile`, and the Python twin's case): at each of the tables' four altitudes, a command asking twice the profile's most is clamped to it within a millionth, on all four.
- **Flown at the bound.**
  - The fleet's new case: every rotorcraft asked for 1.5 times its top is refused under Reject and clamped otherwise. After 240 s each flies within 0.011 % of its top (the IRIS+ and the UH-1H the furthest) and within 2 mm of its height.
  - On the way there (a probe): the IRIS+ is at its top in 40 s, the Crazyflie at 90 % in 60 s and 99.9 % in 120 s, the UH-1H at 99.9 % in 140 s. The UH-60A sags 96 m at full collective and is back on its height 70 s after the command. Asked for 90 m/s it sags 59 m; asked for 70, not at all.
- **Flown unchecked beyond it** (`test_rotorcraft`):
  - The Crazyflie holds 18.30 m/s along its nose and the IRIS+ 13.23, pitched at their 24° and 28° tilt, within 0.1 mm of their height. Asked for half their top, each is back within 2 % of it in 30 s.
  - The helicopters (a probe): the UH-60A asked for 150 m/s flies 99.7 and strays 128 m from its height; the UH-1H asked for 100 flies 55 to 59 m/s and strays 19.5 m.
- **Changed flights:** the fleet's 765 flights, compared with the bound switched off and on: none of the 761 that were flown before changes. The 4 that do are the new case's.
- **Digests:** identical to FA-4d's (ceaec64), with protection and without; no digest flight is a rotorcraft's. The allocation gate passes.
- **A/B throughput** against FA-4d (ceaec64), built in the same worktree: 5 rounds of `micro` twice, 9 of `command` twice, 5 of `world`. A check every 15 s found no other session's tests running meanwhile.
  - The micro cases are within −0.8 % to +1.7 %; no rise repeated, and every minimum is within 0.9 ns.
  - The command cases are within −1.4 % to +1.5 %, a checked UPDATE +0.8 and +0.0 %.
  - A behaviour's NEW reads +2.6 and +2.4 % (1.5 and 2.5 ns on its minimum). Its path, unchecked, meets only one test of a flag, and on FA-4c, before the rebase, the same change read it −1.8 to −2.2 %: layout, as FA-3c and FA-3e found.
  - Layout moved a NEW further while the change was made. With the velocity level's check in `prepare()` written the other way round (the variant tested first), a level switch read +14.9 % and a behaviour's NEW +8.0 %, 10 ns on their minimums.
  - World throughput is 99.5 to 100.3 % of FA-4d's. Protection costs at most 1.2 % (the gate: 97 %).
- ctest: all 270 tests pass.
- **Merged onto main at FA-8d** (2026-09-29), past FA-5a to FA-8d, which it was not written on. Its section, 4.23 there, is 4.48 here: main's 4.23 is the orbit's.
  - The limits it gives the weight now have callers since: a route loiter's pattern (4.31) and a pattern's UPDATE, now in a file of its own. Each holds a rotorcraft's airspeed to its top as the rest do.
  - The fleet's 1,752 flights, compared with the bound switched off and on: the 4 that change are this case's, and the other 1,748 are identical to the bit.
  - `test_modes`, `test_performance_profile` and `test_rotorcraft` pass as they did. The digests, with protection and without, and the route and curve probes are unchanged. The allocation gate passes.
  - ctest: all 368 tests pass.

**FA-5a, A-GRA's orbit (LTR-03, LTR-05, LTR-06, LTR-07, LTR-10).**
- **Two circles, per class** (`test_pattern_shapes`), off their geometry computed in the test (each line checked to touch both circles), once joined, over 1.2 laps:

  | | Circles | Apart | Racetrack | Figure-eight |
  | --- | --- | --- | --- | --- |
  | c172x, 8 m/s wind | 800 and 1,100 m; 700 and 900 m | 4 km; 3 km | 2.2 m | 4.3 m |
  | UH-60A, calm | 150 and 250 m; 150 and 200 m | 800 m; 600 m | 9.4 m | 10.6 m |
  | IRIS, calm | 5 and 8 m; 5 and 5 m | 30 m; 20 m | 0.36 m | 0.30 m |

  - The c172x holds VI-5's bounds (its racetrack 15 m, its figure-eight 21 m, in the same wind) with room.
  - The UH-60A, whose orbit holds 1.4 m, follows the steps in curvature between circles and legs as it follows a route's turns (9.6 m at VI-4) and a curve (7.4 to 11.9 m at VI-6).
  - Each flies both lines and turns the ways the pattern does: a racetrack once round and more to the right, a figure-eight more than half a circle each way.
- **A fix-point orbit** (a c172x in an 8 m/s wind from the west):
  - by its inbound heading north: the course it makes good, 8.95°, flown at 9.00°, heading 0.08°;
  - by its legs' time, a minute: 59.9 s each inbound leg;
  - by its turns' bank, 20°: at most 20.6° flown, 969 m circles;
  - 2.0 m off its racetrack.
- **A magnetic course:** a racetrack inbound on magnetic north flies its inbound leg at 13.00° true, where the declination is 12.96°; 1.7 m off it.
- **Laps, entry and exit points:**
  - two laps of an 800 m orbit complete at 207 s (a lap 91 s), 50 % through after the first;
  - an orbit's entry point is passed 1.5 m off before any lap is counted;
  - a racetrack once round completes 1.1 m from its exit point and flies out along its outbound course (−179.9°, 0.1 m off the line); a minute's orbit completes 2.1 m from its exit point at 65 s and flies out along its course;
  - an IRIS flies to its entry point (0.03 m), once round, and out at its exit point (0.10 m) on its course (−0.1°).
- **Semantics:** refusals name the field (an orbit or a hold with a second circle, a course or leg time with two circles, circles inside or over one another, a bank of 0 or 90°, laps of 1.5, a point without its longitude); a bank beyond the c172x's full bank is refused under Reject, or clamped with the radius it gives; in an UPDATE a heading, bank or leg time replaces the course, radius or legs and is filled in again, and the other way round; a direction reference alone is refused.
- **The fleet** (`test_fleet`): every aircraft flies a racetrack between two circles of its orbit's radius, three radii apart, once round from where it joins it, and completes at its exit point. The worst: a wing 10.0 m from it (the Gripen, 0.56 % of its radius; the Skua 3.6 % of 132 m), a rotorcraft 0.46 m (the IRIS, 6.1 % of 7.5 m).
- **The shape beside the pattern, measured.** Its fields were first the PatternCommand's own. The Command variant that holds every setpoint grew from 120 to 216 bytes, and every command's NEW was 4 % slower (a level switch +8.1 %, a behaviour's NEW +7.1 %). FA-4d's build with only twelve unused doubles added to its PatternCommand read the same, so the size alone was the cause. Carried beside the pattern, as waypoints are, the setpoint keeps its size.
- **A bench's reading depends on where it runs.** The same binary read `attitude, pseudo` at 86 ns from the build tree and 77 ns from a directory of its own, run after run. The A/B now runs both builds from directories whose paths are the same length.
- **Digests:** identical to FA-4d's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-4d (ceaec64), built in the scratch worktree, both run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 3 of `world`.
  - The micro cases are within −1.3 % to +1.1 %; the `pattern` case +1.1 % (2 ns: the path store's revision compared each update, and the ways in and out).
  - The command cases are within −2.9 % to +1.2 %.
  - World throughput is 100.7 to 101.5 % of FA-4d's; protection costs nothing measurable (the gate: 97 %).
- ctest: all 271 tests pass.

**FA-5b, A-GRA's hold (LTR-11, LTR-12, LTR-13, LTR-14).**
- **Turns** (`test_pattern_shapes`, a c172x, calm): each radius the formula's to a millionth.
  - STANDARD: 1,017 m, at most 16.6° of bank;
  - MIL_POWER: 398 m, 38.2°;
  - RELAX: 2,034 m, 8.5°;
  - 0.04 rad/s: 1,331 m, 12.8°.
- **Entry and exit times:** a hold whose window starts in 30 s and ends in 150 s waited, started at 30.0 s and completed at 150.0 s.
- **Entries** (a c172x, calm, a fix 5 km ahead; radius 1,017 m, legs 3,195 m). Each passed the place and course only its entry flies:

  | Entry | Its mark | Off | Track |
  | --- | --- | --- | --- |
  | inbound | the inbound leg's start | 0.9 m | 0.6° |
  | outbound | the outbound leg's start | 0.4 m | 1.0° |
  | parallel | halfway out its leg | 0.6 m | 0.1° |
  | teardrop | halfway out its leg | 1.5 m | 0.2° |
  | direct, left turns | the outbound leg's end | 29.9 m | 2.5° |
  | anchor, the direct sector | the fix, inbound | 0.2 m | 0.7° |
  | anchor, the teardrop sector | the teardrop's leg | 0.8 m | 0.0° |
  | anchor, the parallel sector | the parallel's leg | 2.1 m | 0.2° |

  - The direct entry joined the outbound leg where the hold was nearest: first over the fix at 209 s, where the others were at 89 s.
  - Each flew the hold after within 1.1 to 4.3 m.
  - The direct entry's 30 m is its convergence onto the outbound leg, joined at an angle: it has no way in of its own.
- **The entries, first flown at corners.** The way in was first straight pieces meeting at corners. The follower anticipates a change of curvature, not a corner, so a wing whose rate-one radius is as long as its legs came out of the corner at the fix half a leg off. On the fleet, the parallel leg was passed 13 % to 50 % of the radius off (the heavies up to 2.5 km, 49°). Turned tangent (over the fix, and Dubins's onto a leg), it is passed within 0.42 %.
- **The fleet** (`test_fleet`): every aircraft holds on a fix two turns ahead, entered by ATC's parallel entry, its legs half a minute by time, once round, and completes.
  - The parallel leg is passed within 29 m and 1.3° by a wing (the E-3G, 0.41 % of its radius; the B-52H 21 m), and within 5.1 m and 3.4° by a rotorcraft (the UH-1H, 1.8 %).
  - The Crazyflie's is refused `insufficient_endurance`: its battery does not last a hold at its cruise of a metre a second. Flown anyway, it flies the entry and the lap, then falls: the endurance check is right.
- **Digests:** identical to FA-5a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5a, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 3 of `world`.
  - The micro cases are within −0.6 % to +1.3 %; the command cases within −0.4 % to +2.0 %.
  - World throughput is 99.2 to 100.2 % of FA-5a's; protection costs at most 0.3 %.
- ctest: all 273 tests pass.

**FA-5c, A-GRA's hover and relative points (LTR-15, LTR-18).**
- **The hover** (`test_pattern_shapes`, calm):
  - an IRIS over a point 50 m away and 15 m up, for 20 s: there at 13.8 s, completed 20.00 s after, then within 0.01 m;
  - a UH-60A, its point moved 100 m further at 3 s, before it got there: there at 61.4 s, completed 20.00 s after, then within 0.19 m;
  - a UH-1H orbiting at 60 m, made a hover over the centre for 10 s by an UPDATE at 30 s: there 57 s later, completed 10.00 s after, then within 0.63 m;
  - each at 0 % until it was there; each setpoint without a radius, a way round, a course or legs;
  - a radius, a way round, a course, laps, an exit point and a turn type refused naming their fields (5, 6, 7, 17, 23, 26), and a radius in an UPDATE;
  - the hangar's C172 refused `not_supported` at field 0, the stock C172x `not_implemented`.
- **Relative points** (`test_pattern_shapes`, calm):
  - a C172x round a ship's point (a frame moving north-east at 8.5 m/s, the point 500 m ahead along its heading), an orbit of 1 km: 2.9 m off its moving circle over two laps, where round a still point it was 1.5 m off. Flown as though the frame were still, it was 67 m off;
  - round a fixed frame's point 2 km east and 350 m up: 1.4 m off, and within 0.3 m of the point's height. That height is 0.31 m above the origin's plus 350 m: a Cartesian offset lies in the plane at the origin, and 2 km out that plane is 0.31 m above the sphere;
  - an IRIS 15 m east of a UH-1H flying north at 2 m/s, in the UH-1H's frame: there at 5.4 s, then within 0.12 m. Flown without the frame's velocity, it never came within a metre. The UH-1H removed, it failed `target_lost` and hovered (0.07 m/s over the ground);
  - an UPDATE's point left the frame, and a frame replaced a point.
- **The fleet** (`test_fleet`):
  - the hover: each rotorcraft over a point ten seconds of its cruise ahead and 10 m up, for 20 s. Each completed 20.00 to 20.03 s after its arrival and hovered on within 0.65 m (the UH-60A's). Every wing was refused `not_supported` at field 0, as its support table says;
  - the UH-1H arrived 85 s after its command, the UH-60A 17 s: it closed its last 17 m in 65 s. That is the rotorcraft loops' design, not the hover's. The separate look found it is not the position loop's gain (0.0875/s, not the 0.053/s first reckoned here: the attitude floor does not bind, and the pitch power does not enter the bandwidths) but the velocity loop's integral winding up with the deceleration, on a UH-1H whose identified plant leaves out its stabilizer bar and its rotor's flapping with speed ([rotorcraft.md](rotorcraft.md), section 7: closing on a point);
  - relative points: every aircraft round the origin of a frame moving across its heading at a tenth of its speed, measured from its first lap. The wings were within 16.8 m (the C172's, 4.7 % of its 357 m; the jets 0.1 to 0.6 %), the rotorcraft within 4.3 m (the UH-1H's, 2.1 %);
  - after it is judged, the Crazyflie flies on until the longest aircraft's case ends. Its battery is spent at about 450 s, and it falls and diverges, as in the cases before ([rotorcraft.md](rotorcraft.md), section 7: a spent battery).
- **The support table:** an option governed by R1 alone, on an aircraft nothing declares and whose model does not hover (a stock JSBSim model), now reads `not_implemented`, as the hover capability does for it; it had read supported. The 35 designs declare R1: their answers are unchanged.
- **Conformance:** a refused UPDATE may now answer `not_implemented`, as a refused NEW could: an UPDATE making a stock C172x's pattern a hover drew it in the optimise walks. A pattern's activity may fail `target_lost`, its frame's vehicle gone.
- **Digests:** identical to FA-5b's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5b, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 3 of `world`.
  - The micro cases are within −0.9 % to +0.7 %; the command cases within −2.9 % to 0.0 %.
  - World throughput is 99.3 to 99.6 % of FA-5b's; protection costs at most 0.6 %.
- ctest: all 275 tests pass.

**FA-5d1, A-GRA's curve segments (CRV-03, CRV-08).**
- **Evaluated** (`test_nurbs`), against the Cox-de Boor recursion evaluated in the test:
  - a rational cubic with three interior knots and uneven weights, and a quartic with a double knot: their points within 9.1e-13 m;
  - their derivatives against Richardson-extrapolated central differences: the first within 8.3e-10 of its size (at step 1e-3, 1.3e-8: sixteen times as much, the extrapolation's own error of order h^4); the second within 1.5e-8, rounding's floor at either step;
  - a semicircle as a rational quadratic (NURBS's exact circle), 700 m: within 3.4e-13 m of its radius, its curvature within 1.1e-15 of 1/700;
  - a Bezier's form through the rational basis: within 9.1e-13 m of Bernstein's; flown by Bernstein's, identical.
- **Bezier curves fly as before, to the last bit:** a C172x's two-segment S with a segment appended, and an IRIS's curve ending in a hover, flown by this build and by FA-5c's. Their states and progress over 80 s, 64 lines at full precision, are identical. The digests fly no curve; this does.
- **Refused:**
  - each of fifteen malformed segments: 3 or 11 points, 15 knots, no degree, a point not finite, a weight of 0, a negative knot, knots falling, not clamped at either end, the first knot once too often, a break (an interior knot degree + 1 times), a curvature of 0, a first index of 1, a last index not its last point;
  - a curve's second segment not clamped: `invalid_curve` at 1;
  - a curvature said as 0.8 of the most it turns (5.07e-4 1/m where it turns 6.34e-4): `invalid_curve`, the section 0.688 to 0.719, where it turns tighter; said as 1.02 of it, accepted.
- **Flown** (`test_nurbs`, calm): a rational cubic S twelve radii long, a semicircle of 1.6 radii round to the right as a rational quadratic, and a cubic straight on.
  - a C172x (radius 425 m): 7.1 m off the curve, 2.3 m off the semicircle's radius; completed;
  - an IRIS (20 m): 0.12 m off the curve and the radius.
  - Its setpoint reads the segments back as given, and no Bezier ones.
- **The fleet** (`test_fleet`): every aircraft flies the same curve scaled to its orbit's radius, and completes. From a fifth along it, the wings keep within 3.8 % of their radius (the Skua's 5.1 m; in metres, the KC-46A's 287 m, 3.2 % of its 8.9 km) and the rotorcraft within 5.6 % (the Crazyflie's 0.38 m).
- **Conformance:** the optimise walks give a curve now and then as cubic NURBS - its rational path - with the same faults the Bezier ones carry.
- **Surfaces:** the C ABI's 1.24 block (a rational cubic submitted, read back as `FSIM_BATCH_NURBS`, appended, one not clamped refused, a batch); Python's `test_nurbs` (submitted, read back, `flyout_curve`, appended as a dict, refused, a batch, a task).
- **Memory:** the path store holds 32 segments of 464 bytes where it held 144 (10 KB more a vehicle); the host's plan and each curve behaviour's likewise.
- **Digests:** identical to FA-5c's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5c, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 3 of `world`.
  - The micro cases are within −0.8 % to +1.3 %, but the curve's 4.8 % faster (273.8 ns: its Bernstein sums now inline into the plan's point); the command cases within −1.2 % to +1.3 %.
  - World throughput is 99.9 to 100.2 % (7 rounds; 3 read 98.7 to 100.6 %) of FA-5c's; protection costs at most 0.5 %.
- ctest: all 278 tests pass.

**FA-5d2, A-GRA's curve reference and control points (CRV-04, CRV-05, CRV-06).**
- **Its reference** (`test_curve_references`, calm):
  - above the ground: a C172x's level curve 10 km east at 400 m, over ground rising 2 % to the east. After its first minute it kept within 4.7 m of 400 m over the ground, the ground 186 m higher at its end;
  - its range: left out at 500 m, with a range of 1,000 to 1,200 m, the altitude was 1,000 m; given 800 m it was refused at field 2; a range whose least was above its most was refused at 10;
  - its third three ways: the same curve, climbing 150 m in 6 km, given as down, as an altitude offset up and as absolute altitudes. After 90 s all three were at 623.129 m: the first two bit for bit, the third within a millimetre (made down from its reference by a subtraction).
- **Laid out on the Earth** (a C172x each):
  - along a great circle, 30 km east: its end 0.00 m from the great circle's end, where the plane's (the parallel) was 54.4 m from it; flown within 1.8 m of the great circle past its first 5 km;
  - along a rhumb line, 25 km north-east: its end 0.00 m from the rhumb line's, where the plane's was 19.0 m from it; flown within 1.2 m.
- **In a frame:**
  - a ship heading 60° at 8 m/s, the curve 8 km abeam to starboard along its y axis (150°), turned with its yaw, the aircraft 2 km short of its start. From 80 s on it was within 2.8 m of the line where the ship was then, its track over the ship 0.12° off it; by the curve's end the ship had carried the line 1.5 km;
  - this case found a defect in FA-5d2's own code before the commit: a curve whose axes are turned had its cross-track read as though they pointed north. On the curve's extension, the aircraft read 1.7 km off it, turned away, and was still 579 m off at 80 s. Read along its axes (the course turned after), 2.8 m. An unturned curve runs the same operations as before;
  - an IRIS along a curve from 20 m east of a UH-1H flying north at 2 m/s, in the UH-1H's frame: within 0.03 m of 20 m east of it. The UH-1H removed before the curve's end: the activity failed `target_lost`, and the IRIS held still (0.00 m/s).
- **In three dimensions:**
  - a fixed frame heading east and pitched 2° up: a curve 6 km along its x read back ending 5,996.3 m east and 209.40 m up (6 km's cosine and sine of 2°), and was flown to 209.6 m up. Given as absolute altitudes 50 m above its reference, its end stayed 50.000 m up, only its x and y turned (5,996.3 m east);
  - an IRIS along a line from 20 m to a UH-1H's right along its nose, in the UH-1H's body axes, turned afresh every step as the UH-1H flew north at 2 m/s. In the UH-1H's axes it kept within 0.02 m of 20 m to its right and 0.84 m of its level, the UH-1H pitched up to 4.7°: a degree swings the line a metre 60 m along it.
- **Refused, naming the field:** a reference that is not one (8), offsets (12) and a third (13) out of range, a turn with no frame (11), a frame the world does not have (14), offsets without a frame (18); an altitude outside its range (2), and a range upside down (10). In the frame 2 km north of its origin and 200 m up, the reference was read back there, 0.31 m higher: a Cartesian offset lies in the plane at the origin (as a pattern's, FA-5c).
- **Where a curve is changes only with a new curve's segments.** Given with its options alone: an altitude (2), offsets (12) and a frame (14) were refused; its speed alone was taken. A new curve's point left the frame. A new curve's reference given without its altitude was refused (8). Segments appended with another latitude and another reading went on from the curve's reference, read as its: the setpoint read the curve's back, and a segment appended 60 m up, read as offsets up (the curve's reading, not the down given with it), was flown 60 m up.
  - **A fix, named:** before FA-5d2 a curve's UPDATE of its options alone took a latitude, longitude or altitude. The setpoint and end points read it, but the curve was never flown from it. It is now refused. None of the platform's own calls gave one. The conformance walks' curves that did are now answered `invalid_parameter` where they were taken: their answers change, not their draws.
  - Built first, an append given a reference was refused too. A-GRA's ICD says the appended curve uses the preceding command's CenterReference, and its schema gives every segment one: an A-GRA append always carries one. So an append goes on from the curve's, as before.
- **The fleet** (`test_fleet`): every aircraft flies a straight cubic twelve orbit radii long from the origin of a frame a radius ahead. The curve runs along the frame's heading, 30° right of the aircraft's, its points turned with the frame and their third read as offsets up, climbing 1 %; the frame moves to its right at a tenth of the aircraft's speed. All 35 complete.
  - From a third along it, the wings keep within 4.3 % of their radius (the KC-46A's 385 m of 8.9 km) and the rotorcraft within 2.3 % (the UH-1H's 4.7 m); off its climb, a wing within 8.3 m (the Mirage 2000's) and a rotorcraft 0.01 m.
  - From a fifth along it, the Skua was 21.5 m off (16 % of its 132 m): the tail of its join from a radius to the side at 30°, which overshoots 23 m and settles to 2.5 m. From a third, 10.1 m. The join is the follower's, as flown before.
- **The support table:** `curve/reference/geodetic`, `curve/reference/frame` and `curve/offsets` are supported; the curve's capability names only a rotorcraft's circle at the end (FA-5d3) as missing.
- **Conformance:** the walks draw as they did. The optimise walks also draw the reference, its range, the points' reading, rotations and layouts, and frames (the session's first few, which it does not have), with one of the shape's fields out of its range now and then; the segments' third is given as the reading says, the same curve.
- **Bezier curves fly as before, to the last bit:** FA-5d1's probe (a C172x's S with a segment appended, an IRIS's curve ending in a hover), flown by this build: its 64 lines at full precision are identical to FA-5c's.
- **Surfaces:** the C ABI's 1.25 block (20 fields - a range, great circles, offsets up - read back; an UPDATE of its options alone given a latitude or a layout refused; in a frame, read back placed; a third out of range refused; 21 fields malformed; a batch with its shape); an ABI 1.6 curve reads back 20 fields; Python's `test_curve_references` (by name and member, read back, flown, refused, appended, in a frame, in three dimensions, a batch).
- **Digests:** identical to FA-5d1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5d1, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 3 of `world`.
  - The micro cases are within −1.6 % to +0.6 % (the curve's −1.0 %); the command cases within −1.3 % to +0.4 %, but a same-level update's +3.0 % in both runs (6.8 ns where it was 6.6).
  - That update is `World::command` with the host's `updateLegacy` inlined. Its code is the same instruction for instruction, prologue included; it sits 0x110 bytes later, 16 bytes into a cache line where it began one. Built before a four-line change elsewhere in the host (the append's refusal, taken out), it read −1.5 %: placement, not its path.
  - World throughput is 100.0 to 100.2 % of FA-5d1's; protection costs at most 1.3 % (the B-52H's; the run before read 0.4 %).
- ctest: all 282 tests pass.

**FA-5d3, A-GRA's circular loiter at a curve's end (CRV-11).**
- **Round its end** (`test_curves`, calm): an IRIS's curve at 4 m/s ending in a loiter. From a minute after it completed, it circled its end at 7.49 to 7.51 m, its radius 7.50 m, at 4.00 m/s. Until now it stopped over the end and hovered (0.01 m). The C172x beside it orbits its end at 397 to 400 m as before.
- **On time:** an IRIS given a duration over a short curve ending in a loiter reached its end in 34.5 s of 34.5 (0.1 %). Braking to stop there, it read 34.2 s (−0.8 %).
- **The fleet** (`test_fleet`): every aircraft flies a straight cubic four orbit radii along its heading and loiters at its end. All 35 complete and circle it at the radius their pace gives. From half a lap after completing, the rotorcraft keep within 0.928 to 1.007 of it (the Crazyflie's 6.39 of 6.89 m at a metre a second); from a lap and a half, the wings within 0.982 to 1.010 (the Skua's 129 of 132 m, the C172's 361 of 357 m).
  - A wing joins its circle from the centre, as it always has. Half a lap on, the heavies were still out at 1.43 of their radius, settling within the next lap.
- **Changed, named:** FA-5d1's probe, flown by this build. Its 42 lines of the C172x's curve, and of the IRIS's up to its end, are identical to FA-5c's. The IRIS's 22 lines from its end at 25 s circle the end where they hovered over it. The digests fly no curve.
- **The support table:** `fsim.guidance.curve/end/circular_loiter` is supported, and with it `fsim.guidance.curve`: the curve has nothing missing.
- **Digests:** identical to FA-5d2's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5d2, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −0.4 % to +0.9 %; the command cases within −1.2 % to +0.1 %.
  - World throughput is 99.8 to 100.0 % of FA-5d2's (3 rounds read 98.8 to 100.4 %); protection costs at most 0.6 %.
- ctest: all 282 tests pass.

**FA-6a, A-GRA's waypoint as its schema gives it (WPT-12, WPT-17, WPT-22).**
- **A block** (`test_route_points`, calm): a C172x east at 1,500 m, then a point whose block of 1,700 to 1,900 m was left to choose. It read back 1,700 m (the point before's, held up to its least) and was flown over at 1,700.2 m. An altitude of 1,600 m given there was refused at the point, and so was a block upside down.
- **Barometric** (warm, high air, QNH 1,020 hPa): on along its last leg at 2,000.2 m on the altimeter, 2,102.3 m above sea level against its isobar's 2,102.1 m.
- **A waypoint:** at a corner 4 km out, a turn point was passed 173 m off (turned short) and a waypoint 0.4 m (flown over).
- **Types:**
  - nav only and passive were flown, and the end of a path on the last point; a type given alone made the point a waypoint, reported so;
  - the twelve not built were refused at their point, as the stock C172x's support table rows say: not implemented, since it declares nothing. So were the end of a path before the last point and a loiter point;
  - a type with a turn point, a kind of 3 and a type of 15 were refused as invalid.
- **Frames:**
  - a fixed frame's point was placed 4 km along its x, turned with its yaw (east), within a metre;
  - a C172x round a box 3 km a side about a ship moving north at 8 m/s: from its second lap, within 4.0 m of the legs where the ship was, and 0.95° off them over the ship. Its end point was 0.31 m from the ship's corner as it was then;
  - a chaser: a point 2 km east, then two of another ship's points flown over, the legs to them pursuing them. It passed within 17.8 and 6.7 m of them, and completed;
  - an IRIS loitering at a route's end over a point a frame carries east at 2 m/s held within 0.03 m of it;
  - an IRIS round points in a UH-1H's frame, the UH-1H removed: failed `target_lost`;
  - refused, naming the point: a frame the world does not have, offsets without a frame, a rotation of 4, a seventeenth frame (sixteen were taken).
- **The fleet** (`test_fleet`): every aircraft flies three points in a frame at it, turned with its heading and moving to its right at a tenth of its speed: five orbit radii ahead, five to the right, five ahead again. All 35 complete.
  - In the middle third of its two legs in the frame, the wings kept within 5.1 % of their radius (the Skua's 6.7 m; in metres the F-35A's 49 m, 1.6 %; the heavies 0.1 to 0.6 %). The rotorcraft kept within 8.3 % (the Crazyflie's 0.57 m).
  - First flown on legs three radii long, the rotorcraft read 10.7 %. With the frame still they read as much (the Crazyflie 12 %, the UH-60A 8.5 %): short legs read mostly the turns' joins, and legs five radii long read the frame.
  - First measured along the frame's parallel, the heavies' legs 90 km out read 530 m off. The points are laid out in the plane at the frame's origin, where great circles are straight: measured there, 14 to 30 m.
- **Unchanged, to the last bit:**
  - a route probe, 120 lines of states and progress at full precision, identical to FA-5d3's build: a C172x's fly-by turns, a climb by rate, a fly-over and a loiter; an F-16C's repeating rhumb lines from its second point; an IRIS's short legs to a hover;
  - the curve probe, identical to FA-5d3's;
  - the pattern tests' hovers and frames print the same numbers. The hover's closing is now `route::hoverOver`, shared with the route.
- **Changed, named:** a barometric altitude on a route, refused `not_implemented` until now, is flown. The C ABI, Python and C++ tests that pinned the refusal now fly or read it.
- **The support table:** `route/altitude/barometric`, `route/altitude_block`, `route/waypoint_type` and `route/relative_points` are supported. `route/waypoint_type/taxi` (FA-9, rule R2), `/runway` and `/takeoff` (FA-9), `/landing` (FA-10) and `/hard_ditch` (FA-16) are new, not implemented.
- **Conformance:** the walks draw as they did. The optimise walks give points blocks, both built kinds, the built types and one that is not one, and frames (the session's first few, which it does not have).
- **Surfaces:** the C ABI's 1.26 block (a block, a waypoint by its type, a point in a frame read back; a touchdown refused at its point; a type with a turn point refused as invalid); Python's `test_route_points` (by name and member, read back, refused, round a ship).
- **Memory:** a waypoint is 160 bytes where it was 80. The path store's 256 take 20 KB more, as do the host's plan and each route behaviour's; the frames 1.8 KB in the path store and each plan.
- **Digests:** identical to FA-5d3's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-5d3, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −0.4 % to +1.7 %, but the pattern's −6.0 %: FA-5d3's build read 192 ns this run where it read 181 in the others, and this build read 180.9 as before. The command cases are within −1.5 % to +2.2 %.
  - Built first with a route's frames in the host (1.7 KB more a vehicle), a same-level update read +5.9 % and +9.0 %, and a behaviour's NEW +3.7 % and +4.2 %: the host is each vehicle's, and it grew. The frames went into the host's route plan, allocated at its first route: +0.0 % and +1.1 %.
  - World throughput is 99.1 to 99.8 % of FA-5d3's; protection costs at most 0.8 %.
- ctest: all 286 tests pass.

**FA-6b1, A-GRA's turn points (WPT-04).**
- **Arcs** (`test_turn_points`, calm): east 3 km, then a quarter circle of 1,200 m round to the right from a start to an end turn point, then south 3 km. A C172x flew the arc within 3.8 m, and an IRIS its 30 m one within 0.10 m; both completed. Read back: the types, the start's radius and the end's course as given.
- **A capture:** a point 3 km east flown over at 0.7 m, then its course (north-east) held within 1.15°.
- **A fly-by's radius:** a corner turned on a 2 km circle given was passed 829 m off it (the geometry's 828). Given 100 m, tighter than its full bank at its speed, it was clamped (flagged, read back over 150 m); under Reject, refused `performance_limit` at the point.
- **Refused, naming the point:** a radius the arc does not have (1,500 m where it is 1,200) and an end's course across the arc's; a start with no point after it; an end nothing began; a course on a fly-by and a radius on a fly-over; a capture without its course, and one whose next point is not along it; a waypoint given a turn type; a type of 5; an arc of more than 170 degrees; an arc of 150 m a C172x cannot turn at its speed.
- **The fleet** (`test_fleet`): every aircraft flies two orbit radii ahead, a quarter circle of twice its radius round to the right from a start to an end turn point, and two radii on. All 35 complete. In the arc's middle the wings kept within 3.2 % of its radius (the Skua's 8.4 m; in metres the F-16C's 25 m, 0.5 %), the rotorcraft within 1.7 % (the Crazyflie's 0.23 m; the UH-1H's 6.5 m).
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe, identical to FA-6a's build. An arc is a leg (`route::Leg`): a straight leg's path runs the operations it did, one branch aside.
- **The support table:** `route/turn/capture_outbound_course`, `route/turn/start_turn`, `route/turn/end_turn`, `route/turn/radius` and `route/course_at_point` are supported.
- **Conformance:** the walks draw as they did. The optimise walks give points each turn type, a course and a radius now and then: most of what they make is refused as invalid, and what is not is flown.
- **Surfaces:** the C ABI's 1.27 block (an arc from a start to an end turn point read back; a radius the arc does not have refused at its end); Python's `test_turn_points` (by name, read back, flown, refused, a capture).
- **Digests:** identical to FA-6a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6a, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −1.3 % to +1.2 %, but the curve's +2.2 % (its minimum 6 ns more). Its update, `onCurve`, `curvatureAhead`, `rational`, `follow` and `verticalSpeedTo` are each the same instructions in both builds. Each case's stack holds every mode's behaviour, and a route's plan grew 10 KB, a leg holding its arc: where the curve's structures fall has moved. Aligned to a cache line, `follow` read the same, and its alignment is not the cause.
  - The route's look ahead for arcs runs only where its plan has one (`Plan::arcs`): the route's case read +1.4 %, then +1.2 %.
  - The command cases are within −1.2 % to +1.5 %. World throughput is 100.0 to 100.6 % of FA-6a's; protection costs at most 0.2 %.
- ctest: all 289 tests pass.

**FA-6b2, A-GRA's loiter points inside a route (WPT-18).**
- **An orbit** (`test_route_loiters`, calm): a C172x east at 1,500 m, 4 km, then 10 km on a loiter point with an orbit of two laps, then 4 km north. Read back complete, its place left out: an orbit, right turns, R 417 m (its speed's and 80 % of its bank's), the point's segment's speed.
  - Met 833 m from its point, a radius outside its circle, it was entered along the tangent from there. It flew 3.02 laps within 10.7 m of its circle once joined: two from where it joined, then on round to where its tangent runs to the next point.
  - Half a lap in, its time to go read 152 s: its laps and the leg after it. It left 318 s in, and the leg on, direct from where it left, was 0.5 m off in its second half. The route completed.
  - Handed over a radius out and left to the pattern's own join, the Skua, whose lookahead exceeds its radius, was still 21 % off its circle half a lap in (the fleet). Entered along the tangent, it read 12.8 m.
  - Left wherever its laps ended, the leg on first intercepted the planned leg from the point: 28 m off in its second half. Flown direct from where it left, 20.6 m, after a turn onto it. Leaving where its tangent runs to the next point, 0.5 m.
- **A hover** (an IRIS): north 50 m; east 50 m, with a 15 s hover there; south 50 m, ending in a hover until canceled.
  - It arrived at 25.7 s and left at 40.7 s: 15.0 s there, counted from its arrival, within 0.95 m of its point.
  - It completed on arriving at its last point, and hovered on within 0.60 m of it.
  - A UH-1H handed over within a metre of its point, as the route's end stop is, arrived at 12.4 m/s and swung 49 m past it. Braking with its velocity loop's lag counted, it swung 16 m. Handed over where it would stop from its speed, its position loop brought it in within 3.0 m, as the hover pattern's does.
  - The route's own end stop (`EndBehavior::Loiter`) still swings a UH-1H 49 m past its last point: raised as a task of its own, since done (the end stop's block, after FA-6c1's).
- **A hold** (a C172x): at a fix 10 km east until 300 s, its exit at its fix; then an orbit 5 km north that ends the route.
  - Its inbound course was the leg's (east), its legs 3,269 m and its turns 1,040 m. It flew 1,034 m past its fix, 4,309 m back and 2,080 m to its right: the racetrack.
  - It left 424 s in (its end time was 300 s, then on round to its fix), 0.6 m from its fix.
  - The route completed as the aircraft joined the orbit, 506 s in, and it orbited on at 415 to 418 m (R 417).
- **A moving point:** once round an orbit at a point 500 m west of a ship moving north at 6 m/s: 15.4 m off its circle round the ship's point (R 417), then on.
- **Refused, naming the point:** a loiter point with no loiter; a loiter at a waypoint, past the route, or two for one point; a loiter's own latitude, altitude or frame offsets; no end before the route's end, and none in a route that repeats; an end time not finite; a kind of 7 and a radius of −5 m; a start turn type and a turn radius at a loiter point; a seventeenth loiter.
  - A stock C172x's hover: `not_implemented`, as its hover pattern's row says (it declares nothing). The hangar's C172 declares it cannot hover: `not_supported`.
  - An optimisation on the stock C172x (no tables): `not_implemented`. On the hangar's C172, it is planned at the tables' best speed at its point's altitude.
  - A radius of 50 m, tighter than its full bank flies at its speed: clamped (flagged), the adjustment naming the point and field 105. Under Reject, refused `performance_limit` at the point.
- **Kept:** read back as given while waiting for its start window, complete once started; kept by an UPDATE of the options alone, replaced with new waypoints (none for a loiter point: refused); taken by a task and by a batch, read back with it; validated.
- **Terrain** (`test_terrain_check`): a C172x at 1,000 m with an orbit of 3 km at a point 8 km east, where a ridge rises at 10 km, was refused `terrain_conflict` at the point, on its circle. An orbit of 800 m was flown.
- **The fleet** (`test_fleet`): every aircraft flies three orbit radii ahead, then a loiter point eight ahead, and on to a point two radii to its right. A wing orbits the point once; a rotorcraft hovers over it for 10 s. All 35 complete.
  - The wings swept 1.58 laps each: once round, and on to where they leave for the next point.
  - From half a lap in, the wings kept within 2.7 % of their radius: the C172's 9.5 m, the Skua's 12.8 m of 132, and in metres the F-16C's 18.7 m (0.8 %).
  - The rotorcraft kept within 1.0 m of their point once there. The UH-1H got there 95 s into its loiter: its position loop's approach from where it would stop.
- **Unchanged, to the last bit:**
  - the route probe (120 lines) and the curve probe (64), identical to FA-6b1's build;
  - the hover's closing on a moving point (`route::hoverOver`) has its stopping speed factored out as `route::stoppingLimit`, the expression unchanged.
- **Changed, named:** a loiter point without its loiter was refused `not_implemented`; it is now refused `invalid_waypoint`. The C++ and Python tests that pinned the old refusal now expect the new one.
- **The support table:** `route/loiter_point` is supported, and the route capability's pending list no longer names loiter points.
- **Conformance:** the walks draw as they did. The optimise walks give points a loiter now and then, of every kind the vehicle flies. It ends by its time, its laps or its end time, and now and then it is one the vehicle cannot fly.
- **Surfaces:** the C ABI's 1.28 block (an orbit read back; none for its point refused; an UPDATE with its waypoints and theirs; a radius clamped, named by its point and field 105; a batch's route with its loiter); Python's `test_route_loiters` (by name, read back, flown, updated, batched, a task, refused, clamped).
- **Memory:** a loiter is 296 bytes. Sixteen take 4.6 KB in the path store, in the host's route plan and in each route behaviour's plan. A route behaviour also holds a pattern behaviour, made with it.
- **Digests:** identical to FA-6b1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6b1, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −2.4 % to +1.7 %: the route's −2.4 %, the pattern's +0.8 %.
  - The command cases are within −0.8 % to +0.8 %, but a behaviour's NEW: +6.2 % and +5.8 % (its minimum 125 and 126 ns, then 133).
    - Every function only that path runs is the same instructions in both builds: the registry's `create`, the hold behaviour's `start`, `launch`, `takeOver`, `start`, `end`, `makeRecord`, `noteEnd`.
    - `submitWith` and `prepare` pass one more argument. A level switch's NEW, which runs them too, read −0.6 % and +0.6 %.
    - `submitWith` grew 448 bytes, its waiting branch's loiters: where the code falls has moved, as FA-3b recorded for this case.
  - World throughput is 100.1 to 101.1 % of FA-6b1's; protection costs at most 0.8 %.
- ctest: all 295 tests pass.

**FA-6c1, A-GRA's segment speed optimisation and acceleration (WPT-06, WPT-10).**
- **A best range speed** (`test_route_segments`, calm): the hangar's C172 east at 1,500 m: 3 km at 50 m/s, to 15 km at its best range speed, on to 22 km with its speed left out, then to 28 km at 45 m/s.
  - Read back: point 1's optimisation, its speed the tables' best at the aircraft's altitude and fuel as it was sent (43.488 m/s), a true airspeed; point 2's the same, continued; point 3's its own.
  - In its segments' middles it flew within 0.0004 m/s of the best at the altitude and fuel then (43.422 m/s by the end, the fuel burned), its progress telling that speed within 1.1e-6 m/s. Then 45.00 m/s, and it completed.
- **An acceleration:**
  - a C172x east at 55 m/s, then 12 km on at 40 m/s reached at 0.2 m/s²: in the ramp's middle it slowed at 0.198 m/s², and from 53 to 42 m/s at 0.198 on average. Given no acceleration, its loops slowed it through the same speeds at 1.03 m/s². Its progress told the ramp's speed;
  - an IRIS from its hover to 6 m/s over the ground at 0.5 m/s²: 0.498.
- **Held to what the aircraft can:**
  - the hangar's C172 slowing from 50 to 35 m/s at 5 m/s²: held to 0.987 m/s², idle's least between (flagged; its point 1, field 24, `MinAcceleration`; read back held);
  - an IRIS's 30 m/s² from its hover: its own most (`maxAccelerationMs2`, `MaxAcceleration`). Under Reject, refused `performance_limit` at its point;
  - first looked up at the speed changed to, held within the level speeds: there, at the slowest, the tables gave no excess power, and 5 m/s² was taken as given. The tables read each altitude row at one equivalent airspeed, and the row below the C172's 1,477 m lay outside its own slowest (a fraction of about −0.04, past the 0.02 read beyond an edge). Now the least at nine speeds through the change, those it cannot read passed over.
- **Refused, naming the point:** an acceleration of 0 and of −1, and an optimisation of 2 (`invalid_waypoint`); a climb optimisation (`not_implemented`, FA-6c2's); the stock C172x's best endurance, which has no tables (`not_implemented`, as its row says; the hangar's C172's row reads supported).
- **The fleet** (`test_fleet`): all 35 complete.
  - A wing flies to its best range speed and on at it (its speed left out), then slows to 90 % of it over 40 s. In the second segment's middle the wings kept within 0.24 % of the best at the altitude and fuel then, but the C-130J's 0.98 %.
  - In the ramp's middle every wing slowed 9 to 13 % faster than asked (the RQ-4B's 13.2 %; none was held). A wing's airspeed loop lags the ramp as it begins and catches up in its middle. A C172 that began its ramp still settling from its start fell 0.9 m/s behind the ramp, and closed with a 14 s time constant. Begun on a wing just started, 20 s in, the wings read 9 to 14 % too: the lag is the ramp's own start.
  - A rotorcraft goes from its hover to its cruise over 20 s, over the ground: the IRIS 0.1 % faster than asked, the UH-1H 11.5 %, the UH-60A 14.9 %, the Crazyflie 7.0 % slower. Then at its best range speed, within 0.09 % (the Crazyflie's) in the third segment's middle.
  - First sized by its cruise, the Crazyflie's best range segment was 126 m long, and it could not reach its 17.4 m/s there: its segments are now sized by the best range speed.
  - First judged 390 s in, the Crazyflie had fallen: a rotorcraft flies on at its best range speed once its route completes, and its battery, which lasted the route's 220 s, was spent about 360 s in. The route's endurance check covers the route; the case now judges soon after it.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6b2's build. The ramp and the optimisation run only where a segment gives them.
- **The support table:** `route/speed/long_range_cruise` and `route/speed/max_endurance` are supported where the aircraft has tables, else not implemented; `route/acceleration` is supported. `route/climb/best_rate` and `route/climb/extended_range` stay not implemented (FA-6c2). The route capability's pending list names climb optimisation.
- **Conformance:** the walks draw as they did. The optimise walks give points a speed optimisation (or one that is not one), now and then a climb optimisation, and an acceleration, now and then beyond the aircraft's, or 0.
- **Surfaces:** the C ABI's 1.29 block (an acceleration read back; the stock C172x's best range speed and a climb optimisation refused `not_implemented`, an acceleration of 0 `invalid_waypoint`, each at its point); Python's `test_route_segments` (by name, read back, a ramp to the best range speed asked at 0.1 m/s², flown at 0.109 in its middle; refused, held).
- **Memory:** a waypoint is 200 bytes where it was 176: the path store's 256, the host's route plan and each route behaviour's take 6 KB more. A route behaviour holds four more doubles, its ramp's.
- **Digests:** identical to FA-6b2's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6b2, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −1.8 % to +0.7 %: the route's +0.7 % (its minimum 2 ns more), a check each update for an optimisation or a ramp.
  - The command cases are within −1.7 % to +1.5 %, but a checked update's −13.0 % in both runs (its minimum 28.1 ns, now 24.2). Every function on its path is the same instructions in both builds. The activity lookup's padding before its loop differs: it now starts on a 64-byte line, where it sat 32 bytes past one. That is layout, as FA-6b1 recorded.
  - World throughput is 98.9 to 99.2 % of FA-6b2's (a first run: 98.9 to 100.1 %); protection costs at most 0.9 %. On the per-step path, too, only the padding before loops differs (`afterStep`'s). The functions that copy waypoints, a suggestion's and a task's, copy 24 bytes more.
- ctest: all 298 tests pass.

**FA-6c2, A-GRA's climb optimisation (WPT-08).**
- **What the tables say** (their level fuel, or power, per metre at a fixed true airspeed, at the middle weight and at 10, 50 and 90 % of the speeds every row flies level): it falls with altitude at every one of those speeds on 12 aircraft, rises at the slowest on the C172 and the UH-60A, and is mixed on the rest (it falls at the fastest). So no fixed rule: an efficient climb is timed by the tables, per segment.
- **Fuel** (five wings, each flying the same route three ways side by side: a climb and a descent through 500 m, a C172's, or 1,000 m, each over a 400 s segment):
  - climbing at once at full power used 1.9 % less fuel over its segment than a climb along the gradient on the F-16C, 4.5 % on the B-52H, 3.3 % on the KC-135R and 0.9 % on the C-130J; on the C172, the same within 0.06 %;
  - descending late at idle, as first built, cost 0.6 % more than along the gradient on the C172, 0.7 % on the F-16C, 2.2 % on the B-52H and 7.0 % on the KC-135R; the C-130J's saved 2.3 %. An engine at idle burns fuel it does not turn into flight, and the tables have no idle fuel flow to weigh that by: an efficient descent is along the gradient now, and costs what one does (within 0.1 %);
  - a best rate descent, at idle at once, cost 1.9 to 14.5 % more than along the gradient: it is the fastest, not the cheapest.
- **Flown** (`test_route_segments`, calm):
  - a C172 at 50 m/s, from 1,500 m up to 2,000 m at its best rate over 20 km: its vertical speed within 0.2 % of its tables' excess power as that fell with the height (1.97 to 1.48 m/s), its speed within 1.8 m/s of 50. It was level at 2,000 m 76 % into the segment. Down again at its best rate: within 2.6 % of its most, its guidance's 5.05 m/s (idle's is more), and at 1,500 m at its point;
  - a C172 at 40 m/s, where its tables have level flight cheaper low at every altitude they fly, the same route efficiently: still at 1,500 m halfway, then up at its best rate, at 2,000 m at its point (0.4 m low). Its descent was within 0.1 m of its gradient, and at 1,500 m at its point (0.1 m low);
  - an IRIS from its hover, up 40 m and down again at its best rate: up at 2.00 m/s, within 5.2 % of the most its guidance allows (its tables give more), down at its guidance's most, within 5.5 %;
  - two KC-135Rs up 1,000 m over 68 km and down again, one efficiently, one along the gradients: the efficient climb 325.3 kg, the other 336.3 kg (3.3 % less); the descents 229.1 and 229.0 kg.
- **Refused, naming the point:** a climb rate beside a climb optimisation, and a climb optimisation of 2 (`invalid_waypoint`); the stock C172x's, which has no tables (`not_implemented`, as its rows say).
- **Terrain** (`test_terrain_check`): the hangar's C172 at 1,400 m, 3 km east, then on to 20 km east at 1,600 m over a ridge 1,500 m high at 10 km. An efficient climb was refused `terrain_conflict` at its point: walked from the latest it could start, it met the ridge's edge at 1,400 m. A best rate climb, walked at the least rate its tables give through the change, was flown, and crossed the ridge no lower than 1,599.8 m.
- **The fleet** (`test_fleet`): every aircraft flies a minute on (a rotorcraft 20 s), then a best rate climb, an efficient climb and a best rate descent through both, each change sized to take some 40 s at the rate its tables give (a wing's 300 to 812 m, a rotorcraft's 10 to 40 m); then level. All 35 complete.
  - From halfway up to nine tenths of its best rate climb, the wings climbed within 6.2 % of their tables' rate (the Typhoon's), but the Mirage 2000 12.4 % and the RQ-4B 11.4 % faster: their vertical loops still catching up on the lag the climb began with. A Mirage's loop ran 15 m/s on a 12.7 m/s command while behind its target. The rotorcraft were within 1.1 %.
  - Through the middle half of the best rate descent, within 4.3 % (the Mirage's); a rotorcraft within 2.8 %.
  - The efficient climb reached its point's altitude within 9.1 m (the J-20A's, over it; the Mirage 2000's 3.4 m); every other within 1.7 m.
  - First judged at its worst instant, a climb read up to 36 % off (the EC-130H's): an E-3G lags the rising target some 2 s, overshoots to 20 m/s catching up, then climbs within 2 to 3 % of its tables' rate, and overshoots its top by 39 m, as a route's climb at a rate does. Through its middle half, still up to 16.7 %; from halfway up, as above.
  - The U-2S, whose tables do not read its 92 m/s near 3,600 m, was first held to its guidance's most descent, 9.14 m/s, which idle cannot give holding its speed: it gained 12 m/s. Read at the nearest speed its tables do read, it descends at their 4.6 to 6.1 m/s, within 0.1 %.
  - The Mirage 2000's efficient climb held its tables' cheapest row between its ends, 3,570 m, and arrived 31 m low at its point: its vertical loop lags a few seconds. Its late climb now starts that much earlier (its altitude loop's time constant), and it arrived 3.4 m over.
  - First sized as the wings, the Crazyflie's route outlasted its battery: refused `insufficient_endurance`, as it should be. A rotorcraft's changes now take some 20 s.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6c1's build. The climb profile runs only where a segment has a climb optimisation; the terrain walk's profile gains a late change, the rest of it the same expression.
- **The support table:** `route/climb/best_rate` and `route/climb/extended_range` are supported where the aircraft has tables, else not implemented. The route capability's pending list no longer names climb optimisation.
- **Conformance:** the walks draw as FA-6c1's. Their climb optimisations are now flown where the aircraft has tables.
- **Surfaces:** the C ABI's 1.29 block (the stock C172x's climb optimisation refused `not_implemented` at its point); Python's `test_route_segments` (a best rate climb by name, read back, climbing at 1 to 2.5 m/s).
- **Memory:** a route behaviour holds three more doubles and a flag, its climb profile's.
- **Digests:** identical to FA-6c1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6c1, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −1.2 % to +2.1 %, run on a quiet machine. Two runs beside another session's benchmarks (it was measuring its own world gates) swung ±5 % in their medians, their minimums within ±2 %.
  - The command cases are within −1.5 % to +1.6 %, but a behaviour's NEW: +1.3 % and +2.2 % (its minimum 128 ns, now 131; beside the other session's benchmarks, +4 %). Every function on its path is the same instructions in both builds but for their padding: layout, as FA-3b and FA-6b2 recorded for this case.
  - World throughput is 99.0 to 101.0 % of FA-6c1's; protection costs at most 1.4 %.
- ctest: all 301 tests pass.

**The route's end stop, flown as a loiter point's hover (the task FA-6b2 raised).**
- **The change:** a rotorcraft whose route ends in a loiter (`EndBehavior::Loiter`) braked along its last leg at 80 % of its deceleration, its velocity loop's lag not counted (`route::brakingLimit`), and was handed to its position loop within a metre of its point. Now, as a loiter point's hover (4.31):
  - it is handed over where it would stop from its segment's speed (`route::stoppingDistanceM`), the leg held to the stopping speed with the lag counted (`route::stoppingLimit`);
  - the position loop takes it over the point, no faster than the segment's speed, as the hover pattern's does;
  - the route still completes within a metre of its point along its last leg.
- **Compared** (`test_route_loiters`, calm): each rotorcraft flies to a point eight turn radii east at its cruise over the ground, three ways: the hover pattern, a route to a loiter point with a hover, and a route that ends in a loiter. How far past its point the end stop swung, before and after (the pattern's; a loiter point's):
  - the Crazyflie 1.40 → 0.78 m (0.61; 0.78), the IRIS 1.20 → 0.61 m (0.61; 0.61);
  - the UH-60A 12.4 → 0.0 m (0.0; 0.0) and the UH-1H 42.2 → 0.0 m (0.0; 0.0). Once within a metre of its point, the UH-1H swung out to 49.1 m; now 1.0 m. Handed over within a metre, the two passed their points at 8.4 and 12.1 m/s.
  - The end stop now flies as the loiter point's hover does, the same to the printed precision. The loiter point's route completes as its loiter begins (the UH-1H's 101 s in, 194 m out); the end stop's as it gets within a metre of its point, as before.
  - The helicopters complete later: the UH-1H at 196.4 s where it passed its point at 114.4 s, the UH-60A at 91.4 s where at 61.7 s. Their position loop's approach is slow (FA-5c's finding: the UH-1H closed the last 17 m in 65 s).
  - Within a metre for good, the helicopters are there a little later than their swing came back: the UH-1H at 196.4 s against 188.0 s, the UH-60A at 91.5 s against 84.5 s. The multirotors are there sooner: the Crazyflie at 56.6 s against 67.0 s, the IRIS at 16.5 s against 19.3 s.
- **The fleet** (`test_fleet`): each rotorcraft flies three orbit radii ahead, then to its last point eight ahead, its route ending in a loiter. All four complete.
  - Past its point, before and after: the Crazyflie 1.41 → 0.78 m, the IRIS 1.21 → 0.62 m, the UH-60A 12.4 → 0.0 m, the UH-1H 42.3 → 0.0 m (then out to 49.0 m, now 1.0 m).
  - Completed 0.98 to 1.01 m from its point, then held within 1.01 m. The UH-1H completes 82 s later than it did, the UH-60A 30 s, the multirotors 0.2 and 0.3 s.
  - Flown on the old code with looser first thresholds (5 % of its scale), the case failed the UH-1H's two checks. Its thresholds, twice the fleet's worst now, would fail the UH-60A's too.
- **Where it still swings past** (a probe; the tests fly calm air): `examples/python/vehicle_interface.py`'s UH-60A hops, 400 m legs at 20 m/s over the ground in an 8 m/s wind from the north, the last leg south after a fly-by turn, flown by a UH-60A and a UH-1H on flat ground.
  - Swung past its last point along its last leg, before and after: the UH-60A 15.3 → 3.1 m; the UH-1H 54.6 → 0.0 m, but 24 m to the leg's side over its last 100 m, and out to 14.1 m from the point once past it. A loiter point's hover there flies the same, to the printed precision. (First read as 9.1 and 25.9 m, measured over the whole flight: the first leg runs east along the last point's latitude, and the wind's drift there as they set off read as a swing past it. Corrected by the next block.)
  - The turn before the point leaves less than the stopping distance from 20 m/s: the UH-60A's 272 m after 255 m, the UH-1H's 315 m after 200 m (its turns shrunk to R 200 m by the 400 m legs). The path's speed on the turn does not count the stop after it. So the aircraft is handed over as the turn ends. (That this carried them past did not hold here: the next block.)
  - With a last leg of 800 m, where the stop fits after the turn, the UH-60A swings 0.0 m past, but the UH-1H 12.2 m. The tailwind carried it to 21.4 m/s over the ground as it was handed over, whose stopping distance is 354 m. Slowing through no airspeed, its nose swung from 180° to 149° and it drifted 17 m sideways. The hover pattern, flown from a kilometre out in the same wind, swung none.
  - Passing its point abeam with such a swing, the route completes a few metres from it: 3.1 and 10.8 m (before, 2.3 and 21.7 m). Within a metre for good, the UH-60A was there at 89.0 s (101.5 s), the UH-1H at 170.1 s (169.1 s).
  - Not changed here: both would change a loiter point's hover as well. Raised as a task of their own, since done (the next block).
- **Changed, named:**
  - the route probe's IRIS, whose short legs end in a hover: its 26 lines from 48 s. It flew the same to 42 s, 10 m before its point. It was then at most 0.32 m from where it had been (at 48 s: 0.58 m from its hover, where it was 0.90 m), and hovers where it did, within a millimetre, from 66 s. Its cross-track as it completed reads 0.0125 m where it read 0.0027. The wing's and the F-16C's lines are identical;
  - every rotorcraft route that ends in a loiter. The tests' flights print what they did: the IRIS's 40 m hop (`test_routes`, 0.00 m from its point and still) and the IRIS over a point a frame carries (`test_route_points`, within 0.03 m). The Python example's UH-60A hops (`examples/python/vehicle_interface.py`) end in one.

  The new baseline awaits the owner's approval.
- **Unchanged, to the last bit:** the digests, with protection and without; the curve probe (64 lines). A loiter point's hover is reached and flown by the same expressions as before. The allocation gate passes.
- **A/B throughput** against main at FA-6c1, both builds from this change's worktree, each run from its own directory: 5 rounds of `micro` twice, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −2.6 % to +1.3 % (the route's +0.2 % in both runs), but one: "apart, pseudo" read −5.8 % and −6.3 %, faster. It runs no guidance. The worktree's build of FA-6c1 read it at 93 ns where FA-6c1's own build read 87, and this build reads 87: layout, as these runtime-only cases move.
  - The command cases are within −1.2 % to +3.0 %. The +3.0 % is a same-level update's 6.9 ns against 6.7, its minimum the same, then +0.0 %. A level switch's NEW, which runs no route, read +1.6 % and +2.2 %.
  - World throughput is 99.2 to 101.2 % of FA-6c1's in three runs, two of them watched: nothing else used the processor. Protection costs at most 1.9 %. A first run read the F-16C at 96.5 % (the others 98.2 and 101.5 %), with nothing else seen running before or after. In another, the same build read protection as costing 7 to 9 %, where it costs at most 1.9 % in every other run: something else was running then, and it read 83 to 94 %.
- ctest: all 299 tests pass.

**A stop just after a turn: slowed on the turn, the nose held on the leg (the task the end stop raised).**
- **Measured again first,** on the last leg alone (above): in the hops the end stop left the UH-60A 3.1 m past its point and the UH-1H none, but 24 m to the leg's side. Calm, the UH-60A swung 2.8 m past and the UH-1H 9.8 m, then out to 24.8 m from its point. The 800 m leg's 0.0 and 12.2 m stand; before the end stop, 11.0 and 59.7 m.
- **Why, traced second by second:**
  - the stop after the turn left the position loop room in the hops. It stops on its full deceleration, the UH-60A from 20 m/s in some 160 m, and the UH-1H flies its turn at 14.6 m/s (a turn rate a third of its velocity loop's bandwidth, on its shrunk 200 m radius), which it stops from within the 200 m left. A shorter leg after the turn puts the stop into the turn: with a last leg of 200 m, calm, the UH-60A swung 17.4 m past, the UH-1H 62.8 m;
  - the nose: the position loop turns it to its point from beyond 20 m (`RotorPosition`). An aircraft that leaves a turn off its leg swings its nose with that bearing - the UH-1H's from 180° to 149° on the 800 m leg - and the velocity loop's integrals, kept along the nose and to its right, turn with it. The hover pattern, flown straight at its point, keeps its bearing and its nose;
  - a UH-1H leaves a fly-by turn at its cruise some 25 m wide of its leg, calm or not, and its velocity loop (0.22 rad/s) overshoots a speed step after a turn: 21.4 m/s on a 20 m/s leg, carried through the stop.
- **The change:**
  - a rotorcraft whose stop - its route's end where it loiters, or a loiter point's hover - lies beyond the point it flies to, nearer than it stops from its pace, flies no faster than stops it there along what is left of the route: its velocity loop's lag counted, at half its deceleration (`route::stoppingLimit`; `route::Plan::stopsAt`, `toStopM`). It binds only where the stop reaches back past the leg into its point, so the straight approaches fly as before, to the last bit;
  - both stops hold the nose on the course the leg into the point arrives on (`PositionCommand::headingRad`; a leg under a metre has none, and leaves it to the position loop). The hover pattern still turns its nose to its point, which it may start from facing anywhere.
- **Compared** (`test_route_loiters`, new): every rotorcraft 4R east at its cruise over the ground, a fly-by turn right, then its last point R and half its stopping distance south, calm and in a tailwind at 0.4 of its cruise: a route that ends in a loiter, and one to a loiter point with a hover, which flew alike. How far past its point, then from it, and its nose's angle off the course over it - before, with the turn slowed alone, and with the nose held too:
  - the UH-1H, calm: 14.4 m, 17.5 m, 75°; 10.4 m, 18.6 m, 81°; 0.0 m, 1.73 m, 0°. In its tailwind: 0.0 m, 1.03 m, 74°; 0.0, 1.03, 40°; 0.0, 1.30, 0°;
  - the UH-60A, calm: 0.21 m, 6.33 m, 9°; 0.0, 1.26, 1°; 0.0, 1.22, 0°. In its tailwind: 0.60, 4.44, 15°; 0.0, 1.29, 4°; 0.0, 1.27, 0°;
  - the IRIS 0.75 → 0.41 m past calm and 1.12 → 0.87 m in its tailwind, then within 1.02 → 0.99 and 1.18 → 1.11 m; the Crazyflie 1.23 → 0.99 and 1.59 → 1.40 m, then within 1.23 → 1.08 and 1.62 → 1.42 m.
  - Its thresholds are about twice the worst now: 3.0 m past, 3.5 m from its point, the nose within 2°. Before, the UH-1H fails all three and the UH-60A two, and only the IRIS's nose was within 2° of the course; with the turn slowed alone, the UH-1H in calm air still fails all three.
- **The hops** (the UH-60A, then the UH-1H; before, and after):
  - as the example flies them (400 m legs, 8 m/s from the north): 3.1 → 2.9 m and 0.0 → 3.1 m past; out to 3.8 → 3.5 m and 14.1 → 13.4 m from the point; completed 3.1 → 2.6 m and 10.8 → 4.2 m from it; within a metre for good at 89.0 → 90.1 s and 170.1 → 151.8 s. A loiter point's hover there flies alike;
  - calm: 2.8 → 2.7 m and 9.8 → 3.1 m past; out to 3.9 → 4.1 m and 24.8 → 22.1 m;
  - the 800 m leg: 0.0 → 0.0 m and 12.2 → 3.4 m past; out to 1.0 → 1.0 m and 25.4 → 16.1 m;
  - a last leg of 200 m, calm: 17.4 → 0.0 m and 62.8 → 11.4 m past; out to 17.6 → 1.1 m and 76.1 → 16.2 m. At 30 m/s: 6.1 → 5.4 m and 20.8 → 1.9 m past.
- **Not taken, measured:**
  - handing over where it would stop from its ground speed, where that is above the segment's: the UH-1H's 12.19 m became 12.20 m. Between the two distances the leg and the position loop both command the segment's speed;
  - handing over only once its velocity points at the point, or within a metre of it: the UH-60A no worse, the UH-1H worse in two of three hops (19.6 m past on the 800 m leg, where it read 12.2).
- **Still:** after a turn the UH-1H passes its point up to 22 m to its side (the hops, calm). It leaves the turn some 25 m wide, and its position loop closes that at its velocity loop's pace. Its velocity loop's overshoot after a speed step is the rest. Both are its loops' (FA-5c's finding, left to a separate look), not the stop's.
- **Completion** is unchanged: within a metre of the point along the last leg. After a turn the UH-1H completes 4 to 18 m from its point over the hops' variants (4.2 m as the example flies them), the UH-60A 1.0 to 3.3 m; straight at it, both within 1.01 m. Asked whether a stop handed to the position loop should complete within a metre of the point itself instead, the owner kept it along the leg (2026-09-28).
- **Changed, named:**
  - the route probe's IRIS, the same 26 lines from 48 s the end stop changed: within 2.3 mm of where it was, its nose on its last leg's course over its point (153.44°, where it read 153.29°), its cross-track as it completed 0.0146 m where it read 0.0125. The wing's and the F-16C's lines are identical;
  - every rotorcraft's route stop holds its nose on its last leg's course, and a stop just after a turn slows on the turn. The end stop's comparison (`test_route_loiters`) reads as before to its printed precision, the UH-1H completing at 196.5 s where 196.4, the UH-60A at 91.3 s where 91.4. The Python example's hops end in such a stop.

  The owner approved the new baseline, the nose held on the leg with it (2026-09-28).
- **Unchanged, to the last bit:** the digests, with protection and without; the curve probe (64 lines). The allocation gate passes. The end stop rebased onto FA-6c2 (1ca8534) reads its digests and probes as on FA-6c1, and ctest 302.
- **Memory:** a pattern behaviour holds one more double, its hover's heading as a route gives it.
- **A/B throughput** against the end stop on FA-6c2, both builds run from their own directories, nothing else running (watched): 5 rounds of `micro` twice, 9 of `command` four times, 7 of `world`.
  - The micro cases are within −0.5 % to +3.1 %, then −3.6 % to +1.4 %: the route's +0.4 %, then −1.7 %; the curve's +2.2 %, then −0.2 %.
  - The command cases are within −1.5 % to +0.8 %, but a behaviour's NEW: +7.1 % and +8.5 % in the first two runs (its minimum 128 ns, now 138), +1.9 % and +1.2 % in the next two (129 ns). Of the 220 functions on its path, six calls deep, all but one are the same instructions in both builds, and 130 of them moved (the 2 KB these add sits before them). The one is the guidance modes' registration, which runs once. Layout, as FA-3b, FA-6b2 and FA-6c2 recorded for this case.
  - World throughput is 100.3 to 100.9 % of the end stop's; protection costs at most 0.7 %.
- ctest: all 303 tests pass.
- **Merged onto main at FA-8d** (2026-09-29), both blocks, past FA-6d to FA-8d, which they were not written on:
  - `test_route_loiters` reads as above, to the printed precision; the route probe's IRIS the same 26 lines from 48 s, as the blocks name them (its nose 153.44° over its point, its cross-track as it completed 0.0146 m), its other lines FA-8d's to the bit; the digests, with protection and without, and the curve probe unchanged. The allocation gate passes.
  - A last leg a terminator ends where the aircraft is (4.38, since) ends as it did: the aircraft is taken over its point from there, its nose turned to it by the position loop. With the nose held on the leg as the stop holds it, an IRIS whose course to an altitude ended 52 m past its point flew back to it tail first (a probe; no test flies it).
  - The fleet's routes with a branch (FA-6e2's two cases, since) gave a rotorcraft 330 and 250 s. The UH-1H now completes at 340.9 and 241.9 s, the UH-60A at 308.2 and 202.5 s. A helicopter's time there now adds its position loop's approach, 20 s over its velocity bandwidth (the UH-1H's 91 s, the UH-60A's 48 s), as the end stop's own case does. The multirotors' is as it was: the Crazyflie, done at 282.8 s, has 19 % of its charge left at 330 s, and given 520 s it came down.
  - ctest: all 365 tests pass.
**FA-6d1, A-GRA's required time of arrival (WPT-11).**
- **Flown** (`test_route_arrivals`, calm):
  - three C172s east, 3 km, then 12 km on to a point with a window, some 273 s away at 55 m/s. Given 330 to 340 s, one was slowed and arrived at 332.5 s, its aim (a quarter of the window inside it); its estimate, from a third of the way on, was its aim. One at 45 m/s given 280 to 290 s was sped up and arrived at 287.5 s. One given 260 to 300 s flew as planned, at its own speed, and arrived at 273.5 s;
  - an IRIS, 300 m at 5 m/s over the ground, given 90 to 100 s: slowed, it arrived at 92.5 s (aimed 92.5);
  - a C172 through two windows: 200 to 205 s at 201.3 s (aimed 201.25); the next point at its own 55 m/s again; 370 to 400 s, as planned, at 384.4 s;
  - first released to its planned speed once the arrival that speed gave came within the window, a C172 sped up in its last 10 s and arrived 1.1 s before its aim: a schedule once begun now holds to its point. In its last second it first asked its fastest (its estimate read 202.32 s, aiming at 202.5): it now holds the speed it asked.
- **Refused, naming the point:** a begin after its end, an end not finite, an end already past (`invalid_waypoint`); a window too soon for the fastest it flies level (`performance_limit`, `MaxAirspeed`, clamped too) or too late for its slowest (`MinAirspeed`); unchecked, flown; a window after a loiter point, and the stock C172x's, `not_implemented`.
  - The stock C172x first had no speeds to bound its schedule (no tables, no envelope): its windows went unchecked, and a far one would have slowed it below its stall. Its row now reads not implemented, as the speed optimisation's does.
- **The fleet** (`test_fleet`): every aircraft flies to a point three minutes on at its speed, given a window of 10 s a tenth later (slowed), or, where the slowest it flies level cannot take that, a tenth sooner (sped up). All 35 complete; 34 were slowed, and the Crazyflie, whose slowest is its 1 m/s cruise, was sped up.
  - Every one arrived 0.03 s after its aim, a control period: its schedule closes on its own error each update. Its estimate halfway was its aim.
  - First bounded below by its tables' slowest level speed, the Typhoon's window was refused: they read 167.5 m/s at 3,000 m, where it flies 139 m/s. Its floor is now 1.2 times its envelope's least.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6c2's build. The schedule runs only where a point has a window.
- **The support table:** `route/required_time_of_arrival` is partial where the aircraft has tables (at or after a loiter point, not implemented), else not implemented. The route capability's pending list names a time of arrival at or after a loiter point.
- **Conformance:** the optimise walks give points windows now and then: one it may make, one too soon or too late for its speeds, one past, one upside down, one side of it alone - never at or after a loiter point.
- **Surfaces:** the C ABI's 1.30 block (a hangar C172's window read back; its progress's estimate its aim, 202.5 s, and its delta 0, at a ground speed under 50 m/s; the stock C172x's refused `not_implemented` and one upside down `invalid_waypoint`, at its point); Python's `test_route_segments` (by name, read back, arrived within 2 s of its aim, its estimate within 0.5 s; the stock C172x's refused).
- **Memory:** a waypoint is 216 bytes where it was 200: the path store's 256, the host's route plan and each route behaviour's take 4 KB more. A route behaviour holds six more numbers, its schedule's. The activity record is as it was.
- **Digests:** identical to FA-6c2's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6c2, both builds run from their own directories: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`. Another session's builds, tests and benchmarks ran beside the first runs; each was run again once two minutes had passed with none.
  - The micro cases are within −0.9 % to +2.0 %: the hsa's +2.0 % (its minimum 1.6 ns more).
  - The command cases are within −1.6 % to +1.5 %.
  - First carried in the progress, the estimate made every activity record 16 bytes longer (304 to 320), and the host holds 26 records a vehicle, its 10 slots' and 16 ended (416 bytes more): a level switch's NEW read +7.8 % and +7.9 % (its minimum 67 ns, then 73). The estimate is now asked of the route (`World::activityArrival`), and the record is 304 bytes again.
  - Then a behaviour's NEW read +8.6 % and +9.5 % (its minimum 130 ns, then 142), every function on its path the same instructions in both builds. The arrival checks inside `checkRoute`, before them in their file, had moved `prepare` and `submitWith` some 600 bytes on. Outlined into their own translation unit, last in the library (`Arrival.cpp`, with the estimate's plumbing), `checkRoute` is 192 bytes longer than FA-6c2's, they move 64 bytes, and the NEW reads +0.7 % and +1.4 %.
  - World throughput is 97.7 to 100.9 % of FA-6c2's (another session's process ran once beside it; the runs before read 99.6 to 101.9 %); protection costs at most 1.6 %.
- ctest: all 304 tests pass.

**FA-6d2, A-GRA's planned inertial states (WPT-20).**
- **Flown** (`test_route_states`, calm):
  - a C172 east, 3 km at 55 m/s, then 12 km on at 1,500 m, through two states: 1,650 m at 7 km, due at 140 s (at 55 m/s it would be there at 127 s: slowed), and 1,550 m at 11 km, due at 210 s (sped up). It passed them at 140.2 s, 1,650.1 m high, and at 210.2 s, 1,550.1 m; no higher than 1,651.8 m. After 5 km its estimate read 140.12 s: its climb paced no faster than it climbs it;
  - first scheduled at one ground speed throughout, it held 49.4 m/s at full power through the climb while its schedule asked up to 59, and passed the first state 1.4 s late, its estimate reading 140.00 s all the while. Then, catching up in the descent, it ran to 64 m/s on a command of 59 and passed the second 1.7 s early. Its climbs are now paced (4.34): the level entry flown at 51 to 52 m/s where it had been 50, the climb at 49.07, the fastest its tables' full power climbs that gradient at;
  - an IRIS north 300 m at 5 m/s over the ground: 20 m up at a moving frame's origin at 50 s - the frame 2 m/s north from where the IRIS was, the state placed 100 m north (within 0.5 m) - and 30 m up 200 m north at 75 s. It passed them at 50.0 s, 20.0 m up, and at 75.0 s, 30.0 m up.
- **Read back as placed:** what else a state gives (a ground speed, a yaw, an uncertainty, a roll rate of 0) as given; its altitude reference its point's; a state in a frame with its latitude and altitude where the frame was at its time, its frame kept.
- **Refused, naming the point:** past the route, out of order, a field not finite, no place, a frame the session has not, another altitude reference, a time past, a time not after the one before, an altitude beside a climb rate, a segment its first lap does not fly, 65 states, 1 km off a 12 km leg (1 % of it is 120 m), behind the state before (`invalid_waypoint`); 500 m up in a kilometre (`performance_limit`, `MaxClimbRate`, clamped too), 4 km in 10 s (`MaxAirspeed`), a time too late for its slowest (`MinAirspeed`); at or after a loiter point, beside a point in a moving frame, and the stock C172x's time (`not_implemented`). Flown: 300 m off its leg within its own 400 m uncertainty; unchecked, 1 km off; the stock C172x's altitude without a time.
- **Kept:** an UPDATE of its options alone keeps its states; one with new waypoints and none has none.
- **A route's times taken together:** a C172's windows at 8 km (240 to 245 s) and 13 km (250 to 260 s) - each it could make alone - are refused `performance_limit`, `MaxAirspeed`, at the second: after the first, 5 km on takes it to 317 s at the earliest. FA-6d1's check, each from now alone, passed them. With the second at 330 to 340 s, flown.
- **The fleet** (`test_fleet`): every aircraft flies to a point half a minute on at its speed, then two and a half minutes on; between them two states - up a minute on, by what it climbs in 20 s at a third of its most (60 m at the most: the C172's 33.7 m, the Skua's 20 m, a rotorcraft's 3.5 to 33.3 m), and back down a minute later - each timed a tenth later than its speed makes it (slowed), or where its slowest cannot take that, a tenth sooner (sped up). All 35 complete; 34 were slowed, and the Crazyflie sped up.
  - Every one passed each state 0.03 s after its time, a control period.
  - Every one passed each within 9.3 m of its altitude: the Mirage 2000 9.2 m over at the top, where its profile turns from climb to descent, and the RQ-4B 9.2 m under; every other within 4.2 m (the Typhoon's), a rotorcraft within 0.01 m.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6d1's build. The states' profile, their schedule and the climbs' pace run only where a route has states or times to arrive at; the terrain walk's profile runs through states only on a segment with their altitudes.
- **The support table:** `route/inertial_states` is partial on every aircraft (at or after a loiter point, beside points in moving frames, not implemented); a time without tables is refused as a window's, by `route/required_time_of_arrival`'s row. The route capability's pending list names a planned state at or after a loiter point or beside points in moving frames, where it named 4D states.
- **Conformance:** the optimise walks give routes one to three states now and then, halfway along their legs before any loiter point: up or down, a time it may make or not; one off its leg, one not finite, some out of order.
- **Surfaces:** the C ABI's 1.31 block (a hangar C172's states read back, its altitude reference completed; its progress's estimate the state's time, 140 s, and its delta 0; an UPDATE's new waypoints with none; one off its leg refused `invalid_waypoint` at its point, in a batch too; the stock C172x's time `not_implemented`); Python's `test_route_states` (by name, read back, passed within 2 s and 25 m, its estimate within 0.5 s; in a batch; refused).
- **Memory:** a state is 240 bytes. The path store holds 64 (15 KB); the host's route plan and each route behaviour's 64 placed, where each is, and the first lap's climbs (24 KB more); a waiting activity keeps room for 64 (15 KB, reserved at its NEW). A route behaviour holds an index and two flags more. The activity record is as it was.
- **Digests:** identical to FA-6d1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6d1, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −6.3 % to +1.2 %: the curve's −6.3 % (its minimum 18 ns less), the hsa's +1.2 %.
  - The command cases are within −2.0 % to +1.5 %: a behaviour's NEW −2.0 % in both runs (its minimum 130.5 ns, now 128.1), a level switch's −0.4 % and −1.6 %.
  - First passed down beside the loiters as a second span, the states made `submitWith` 544 bytes longer (58 instructions): Windows x64 passes a 16-byte struct by a hidden reference, copied as the function begins, and its registers were allocated afresh throughout. A behaviour's NEW read +2.7 % and +3.5 % (its minimum 131 ns, then 135), a level switch's +3.1 % and +1.9 %. A route's loiters and states now go down the host's private calls as one pointer (`RouteExtras`), null for anything not a route: `submitWith` is 48 bytes shorter than FA-6d1's, `prepare` too, and the NEWs read as above.
  - World throughput is 100.2 to 100.5 % of FA-6d1's; protection costs at most 0.6 %.
- ctest: all 308 tests pass.

**FA-6d3, A-GRA's required navigation performance (WPT-21).**
- **Flown** (`test_route_rnp`, calm):
  - three C172s east at 1,500 m, each given a route north - a quarter turn left onto its first leg, 3 km, then 5 km on - at an RNP of 20 m, 5 km and none. Turning onto its leg each was 386.1 m off at most. At 20 m its activity said so at 1,896 of the 1,908 steps it flew that segment: at every step that ended more than 20 m off (none missed), at none that ended under 18 m. On its second leg, still closing on it, 24.5 m off at most: said so for 184 steps, and no longer at its end, its `constraintsSeen` still holding it. At 5 km, and with none, never;
  - a C172's fly-by turn, east 3 km then north 4 km, 17.5 m off its path at most: at 30 m it never said so, its arc on its path;
  - an IRIS north 100 m, flown over, then east 100 m at 5 m/s over the ground: past the corner 3.2 m off its next leg at most. At 0.5 m it said so at 166 of 632 steps, none missed, none that ended well within it; its first segment, with none, never.
- **Refused, naming the point:** an RNP of 0, −5 m, not finite (`invalid_waypoint`).
- **The fleet** (`test_fleet`): every aircraft flies half a minute straight on at an RNP of a fifth of its scale; then a fly-over point, a quarter turn right and half a minute on at 0.1 m. All 35:
  - straight on, none said so: the worst the UH-1H, 5.86 m off of 60; every wing within 0.95 m;
  - past the corner every one did, at every step that ended more than 0.1 m off and at none that ended under 0.09 m: a wing 150 m (the Skua) to 7.6 km (the C-17A) off, turning as it can after flying over its point; a rotorcraft 1.4 m (the Crazyflie) to 60 m.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6d2's build: the monitor reads the cross-track and sets a flag; it flies nothing.
- **The support table:** `route/required_navigation_performance` is supported on every aircraft. The route capability's pending list no longer names navigation performance.
- **Conformance:** the optimise walks give points an RNP now and then: 1 m to 3 km, now and then 0.
- **Surfaces:** the C ABI's 1.32 block (read back; a hangar C172 turning onto a leg 3 km to its right at 20 m, its activity's `constraints` and `constraints_seen` carrying 32; 0 refused `invalid_waypoint` at its point; `fsim_waypoint_init` leaving it out); Python's `test_route_rnp` (read back; `fsim.ActivityFlag.NAVIGATION_PERFORMANCE` in its constraints and those seen; `fsim.agra.activity_state` ACTIVE_PARTIALLY_CONSTRAINED; 0 refused).
- **Memory:** a waypoint is 224 bytes where it was 216: the path store's 256, the host's route plan and each route behaviour's take 2 KB more. A route behaviour holds its last update's flags. The activity record is as it was.
- **Digests:** identical to FA-6d2's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6d2, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −3.2 % to +1.6 %: a route's −0.4 %.
  - The command cases are within −0.0 % to +3.0 %: the same level's update +3.0 % in one run (its minimum 0.1 ns more) and 0.0 % in the other.
  - World throughput is 100.1 to 100.9 % of FA-6d2's; protection costs at most 0.7 %.
- ctest: all 310 tests pass.

**FA-6e1, A-GRA's paths and links (WPT-13, WPT-14).**
- **Flown** (`test_route_paths`, calm): three paths - A, two points east; B, four points round a square, its last linked back to its first; C, two points behind, linked from nowhere - flown by a C172 at 3 km a unit and an IRIS at 20 m a unit (3 m/s over the ground), for 900 s:
  - the C172 flew 0 1 2 3 4 5 2 3 4 5 2 3 4 5 2: A, then three laps round B. It passed within 27 m of A's points and 132 m of B's (its fly-by arcs cut B's corners), and never came nearer C than 3,000 m;
  - the IRIS flew A, then 37 laps round B, within 2.36 m of B's points, and never nearer C than 20.0 m (A's first point is that far from it);
  - their end points came in that order, round B (0 1 2 3 4 5 2 3), and the setpoint read their points, links and paths back as given.
- **Refused `invalid_waypoint`, naming the point** as given: a point no path holds (2), a path of none (6), a type that is none (2), an id twice (6), a next of 2.5 (3) or past the route (4), a lap round one point (5). A link to -1 ends the route there (end points 0 1 2 3); links without paths take the points as they are (0 1 4 5 2 3 4 5).
- **A path it does not fly, held as given:** C's last a loiter point with its hold - accepted, the hold read back as given (its radius left out: not completed), never flown. Without its hold, refused at 7; a planned state on C, refused at 6 (4.34).
- **The end of a path** (a named change to 4.29): taken at A's last and at B's (which links back round); in B's middle refused `invalid_waypoint` at 3, where FA-6a answered `not_implemented` (`test_route_points` now says so).
- **Not implemented:** a start turn at B's first, where its links go round, its course left out: at 2. Its course given, not so.
- **Kept waiting and reset** (a named change to 4.34): a route from 0 on to 4, 5, then 2 and 3 (0 1 4 5 2 3, round from 4), a state on the leg into 4 and one on the leg into 2. Disabled flying to 5 and enabled, it went on at 5 - end points 5 2 3 4 5 2 3 4 - with the state into 2, not the one into 4 it had passed (as given 4 comes after 2; FA-6d2 kept those beyond by their index). Reset as it flew, and again reset as it waited: from 0 along its links (0 1 4 5 2 3 4 5), with both states. Before FA-6e1 a reset after a resume flew without the states passed, and a linked route would have gone on from where it resumed: without the new reset step, three of the test's checks fail.
- **A stack on its own** (`ControlStack::command`): three paths, A's last linked to -1: 6 km to go, not the 20 its points make in their order. A lap round one point: its behaviour fails.
- **The fleet** (`test_fleet`): every aircraft's route of three paths - one point half a minute ahead, linked into a square of half-minute sides linked round and round; one point a minute behind to the left, linked from nowhere. All 35 flew 0 1 2 3 4 1 2, and on round the square; none came nearer the third path's point than its start, a minute from it (2.99 of its scale at the worst, the EC-130H).
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6d3's build: a route without paths or links plans and flies as before.
- **The support table:** `route/paths` supported, `route/next_segment` partial (the start turn above), on every aircraft; `route/conditional_segment` not implemented (FA-6e2). The route capability's pending list names the start turn, conditional branches and path terminators.
- **Conformance:** the optimise walks give routes paths now and then (one to three, of types drawn; now and then a type that is none, paths that do not tile the points, an id twice) and links (the last point back to one before, a jump, the route's end, a next that is none) - links only where what they may reorder is built, so those walks answer `not_implemented` nowhere on aircraft with tables, as before. Most of what they draw is refused, as their other routes are: the walks hold the refusals to the lifecycle's rules, the fleet and `test_route_paths` fly.
- **Surfaces:** the C ABI's 1.33 block (two paths, the second round and round: read back, end points in its flight order 0 1 2 3 4 2; a point no path holds refused `invalid_waypoint` at 2); Python's `test_route_paths` (paths by name and by member, read back; end points and its flight in that order; refused at 2).
- **Memory:** a waypoint is 232 bytes where it was 224. The path store is 2.9 KB larger (its waypoints, 16 paths, the flight order), and the host's route plan and each route behaviour's plan 4.6 KB (their waypoints, the flight order and where each point is in it, 16 paths, a later lap's leg and turn). A route behaviour holds the leg and turn it flies to (16 bytes). The activity record is as it was (304 bytes); a batch item grows 16 bytes, a setpoint 24.
- **Digests:** identical to FA-6d3's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6d3, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`.
  - The micro cases are within −1.1 % to +2.4 %: the attitude case's median +2.4 % (its least 0.2 ns more), a route's +1.8 %. That route figure is where its copy ran from: the same executable copied to another directory as long read 1.4 % faster. Run from three copies of each build, their medians' median, every micro case is within −0.7 % to +1.1 %, a route's −0.2 %.
  - The route follower takes the leg to its point and the turn there once, as it aims (`RouteBehavior::legTo_`, `turnAt_`). Before, it asked the plan at every call, each ask now minding a later lap's own leg and turn, and its update was 576 bytes longer than FA-6d3's; now 240, and its `locate` 144 shorter.
  - The command cases are within −4.5 % to +3.0 %: the same level's update +3.0 % and +1.5 % (its least the same 6.6 ns in one run, 0.2 ns more in the other), a behaviour's NEW −4.1 % and −4.5 %.
  - World throughput is 99.3 to 100.7 % of FA-6d3's; protection costs at most 1.1 %.
- ctest: all 315 tests pass.

**FA-6e2a, A-GRA's conditional branches: altitude, time, captures, the operator's input (WPT-15).**
- **Flown** (`test_route_branches`, calm), on three paths as FA-6e1's - A, two points east, on into B, a square round and round; C, two points south of B, the way out:
  - a C172 (3 km a unit), with a branch at B's last on to C once it has come there twice, and at A's last one at an altitude it is not at (above 2,500 m) before one it is (1,000 to 2,000 m) on into B. It flew 0 1 2 3 4 5 2 3 4 5 6 7 and completed, its end points as it flew out 6 7. A second, branching to the route's end at B's second point, flew 0 1 2 3 and completed;
  - IRISes (20 m a unit, 3 m/s over the ground): commanded out at B's second point once round (99 s on), one flew 0 1 2 3 4 5 2 3 4 5 2 3 6 7 - held there twice, the operator's input not given, out the next time; one with a time window a minute and a half on went round until it opened, then out (0 1 2 3 4 5 2 3 4 5 2 3 4 5 2 3 4 5 6 7); one whose A's last hovered 20 s went out as the hover ended (0 1 6 7); one commanded as it waited, disabled, then enabled, went out the first time (0 1 2 3 6 7); one commanded and then updated went round and round, the input forgotten.
- **Refused `invalid_waypoint`, naming its point:** at a point it has not (8); at 5 - a next past the route, of 2.5, or its own point; a range or a window upside down, a reference with no range; a count with no comparison, or not whole; an operator input of 2; a contingency of 7; an endurance with no comparison; 17 branches; a flight on from it round one point. **Not implemented** at 5: an endurance, a contingency. The operator's input to a branch that takes none, or one it has not: `invalid_parameter` naming it; to an hsa, `wrong_command_type`; to a route ended, `activity_ended`.
- **A stack on its own:** its end points as it will fly them.
- **The fleet** (`test_fleet`): every aircraft on the paths case's first two paths - a point half a minute ahead, into a square of half-minute sides linked round - with a branch at the square's last out to a point half a minute to its right once it has come there twice, the route ending in a loiter. All 35 flew 0 1 2 3 4 1 2 3 4 5 and completed: 256 s after the NEW (the fighters) to 500 s (the C-17A, whose turns are wider than the square's sides). A rotorcraft's run is 330 s, inside the Crazyflie's battery (it falls at 600).
- **A named change to 4.36** (`test_route_paths`): a repeating linked route begun at point 2 goes back at its end to its first point, 2 3 4 5 0 1 2 3; FA-6e1's went back to 2.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6e1's build: a route without branches plans and flies as before, its distance to go counted from a lap's start the same.
- **The support table:** `route/conditional_segment` partial (an endurance or a contingency: FA-6e2b); the route capability's pending list names those in place of conditional branches. The C ABI's, discovery's and Python's example of a row not built is `route/path_terminators` (FA-6f).
- **Conformance:** the optimise walks give routes one or two branches now and then - at points drawn, on to one drawn or the end, with conditions drawn from those built, now and then one malformed - and a fifth of their UPDATEs are the operator's input to a branch, held to an UPDATE's rules. Those walks still answer `not_implemented` nowhere on aircraft with tables.
- **Surfaces:** the C ABI's 1.34 block (read back; the operator's input to a branch that takes it, and to one that takes none refused, `reserved` 1; one on to its own point refused `invalid_waypoint`, `reserved` 5; an endurance `not_implemented`); Python's `test_route_branches` (read back, codes by name; round twice, then out; commanded, out the next time; refusals).
- **Memory:** the path store is 2.0 KB larger (16 branches and the operator's commands), the host's route plan and each route behaviour's 2.1 KB (their branches, as given, and the times each one's point has been come to). A waypoint is as it was (232 bytes), and so is the activity record (304); a batch item grows 16 bytes, a setpoint 24.
- **Digests:** identical to FA-6e1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6e1, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −6.8 % to +0.3 % and the command cases within −0.8 % to +6.9 %: a behaviour's NEW +6.0 % and +6.9 %. From three copies of each, their medians' median, every micro case is within −0.8 % to +1.0 % and every command case within 0.0 % to +1.2 %: a behaviour's NEW +1.2 %.
  - Before, a behaviour's NEW was 12.9 % slower on all three copies, with every instruction on its path the same except those of `submitWith`'s block for a NEW that waits, which it never runs. The branches' lines there had moved the rest of `submitWith` 80 bytes. That block now keeps what goes beside a waiting route out of line (`CapabilityHost::holdExtras`, in `Branches.cpp`), and the NEW is back to +1.2 %.
  - World throughput is 99.2 to 100.7 % of FA-6e1's; protection costs at most 0.6 %.
- ctest: all 319 tests pass.

**FA-6e2b, A-GRA's conditional branches: endurance and contingency (WPT-15); FA-6e done.**
- **Flown** (`test_route_branches`, calm): four C172s on FA-6e2a's three paths (3 km a unit), each with branches at B's last:
  - out once its percent is at or below what it had at the NEW, less a millionth: out the first time it came there, 0 1 2 3 4 5 6 7 (99.994 % at the NEW, 95.860 % at the end);
  - out once it has burned 90 %: never, round and round (0 1 2 3 4 5 2 3 4 5 2 3 4);
  - its reserve set at 99 % of its capacity, so flight critical: a branch for NORMAL not taken, the next, for FLIGHT_CRITICAL, taken to C's last (0 1 2 3 4 5 7);
  - normal: a branch for FLIGHT_CRITICAL not taken, the next, on its endurance's end (within 10⁹ s of now) and its fuel (below what it had), taken (0 1 2 3 4 5 6 7).
- **Refused `not_implemented` at its point:** a MISSION_CRITICAL or a LOST_COMMS contingency (FA-16). An endurance with a NORMAL contingency is accepted.
- **The fleet** (`test_fleet`): every aircraft on the paths case's square, out at its last once its fuel or charge is below what it had at the NEW. All 35 went out the first time (0 1 2 3 4 5) and completed, 161 s after the NEW (the fighters) to 229 s (the B-52H, the E-3G), each with less than it had.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6e2a's build: no branch reads the report but one with such a condition, as it is decided.
- **The support table:** `route/conditional_segment` partial, now FA-16's: a mission critical or lost comms contingency. The route capability's pending list names it under FA-16.
- **Conformance:** the optimise walks' branches now and then carry what it has left (a percent, an endurance, a comparison drawn - now and then none) or a contingency (normal, flight critical); those walks answer `not_implemented` nowhere on aircraft with tables.
- **Surfaces:** the C ABI's 1.34 block (a LOST_COMMS contingency `not_implemented`, `reserved` 5); Python's `test_route_branches` (an endurance and a contingency by name, read back; LOST_COMMS refused `not_implemented` at its point).
- **Memory:** as FA-6e2a's: its fields were there.
- **Digests:** identical to FA-6e2a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6e2a, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −16.5 % to +0.5 %: FA-6e2a's copy ran its attitude cases slow (54.8 ns, where its three copies ran 46.1 to 46.3). From three copies of each, their medians' median, every micro case is within −1.0 % to +1.3 %.
  - The command cases are within −2.9 % to +0.8 % from one copy, and −1.5 % to +0.8 % from three: a behaviour's NEW +0.5 % (one of FA-6e2a's copies ran it at 143.2 ns, where the other two ran 126).
  - World throughput is 100.0 to 101.2 % of FA-6e2a's; protection costs at most 0.4 %.
- ctest: all 320 tests pass.

**FA-4d's finding closed: the stock C172x's heading (HSA-01, HSA-03).**
- **Why it was off.** The stock C172x flies the shared loops. Their bank loop (`pid_attitude`: roll `ki` 0) has no integral, and at cruise power its propeller rolls it right: it holds about −0.07 aileron.
  - A proportional-derivative loop makes that aileron from a standing bank error, 2.0° short of its command. The heading law (1.5 rad of bank per rad of heading) asks for that bank only from a 1.6° heading error.
  - So it settled 1.6° right of every heading: short of one turned onto left, past one turned onto right, off one held from the start, and the same from magnetic north. Its sideslip stayed within 0.1°.
  - The hangar designs bank through the acceleration level's integral and never had it. The path follower's course integral already took it out on routes ([vehicle-interface.md](vehicle-interface.md), 4.8).
- **Fixed in the mode, not the loops.** The shared loops are what the digests and the stock checkpoints pin, so the bank loop keeps no integral. The hsa trims its heading as it trims a course.
  - The integral is of a quarter of the turn its heading law asks (the course bandwidth times the error) less the turn made; its zero is at a quarter of the bandwidth, as the path follower's.
  - Turning onto the heading as the law has it, that is nothing; held off it, all of the error. It counts within 0.2 rad of the heading while the bank holds (rolling slower than 0.5°/s), and holds at most 0.1 rad.
  - Tried first, on the fleet: an integral on the error alone wound up in every turn onto a heading (at a quarter of the bandwidth the fighters overshot 1.3 to 1.6°, the E-3G 2.3°; at the course trim's slower gain the C172x took some 75 s to come within 0.3°, and the fighters still overshot 0.5°). The turn asked less the turn made, counted without the bank's gate, took a slow roll-out's lag for a standing error: the B-52H stopped 2.4° short a minute on. Twice the gain under-damped the RQ-4B.
- **Flown** (`test_modes`: four C172x's for four minutes after a left turn, a right turn, a heading held and a magnetic heading): each within 0.20° from a minute on, where all four were 1.6° off. Off San Francisco (`test_magnetic`): 0.11° after its left turn, 0.07° after its right turn.
  - A course between two headings: the second counts the turns it makes from its own start, its trim kept, and holds within 0.3°. First built, it counted the 60° turned on the course as one update's turn, which threw the trim to its most: 6.5° off within 30 s.
- **The fleet** (`test_fleet`, the 35 designs, against the same build with the trim's gain 0):
  - A quarter turn right and 200 m up: the heading within 0.11° (the C172, still climbing), where it was 0.14°. The hold behaviour flies the same case unchanged, 0.14°.
  - A magnetic heading: the magnetic heading asked flown within 0.009°, where it was 0.051°.
  - A heading held while a speed optimisation flies: within 0.002°, where it was 0.22° (the C172).
  - The altitudes, speeds and airspeed margins are as they were, within a millimetre and a hundredth of a percent.
  - Its transients, every wing design turning a quarter right (off the test): the most any got worse from 45 s to 200 s was 0.08°, the RQ-4B's at a minute; the worst at 90 s is 0.09°.
- **The commanded state** is the cascade's (4.12): its heading is the velocity level's, the trim beside the heading asked. That is 1.5° on the C172x, and 0.057° at most on the designs (the F-35A's). The mode's progress still reports the heading asked. FA-4d's checks read the commanded heading within 0.01° (the fleet's within 0.02°); they now read it within the trim, and `test_magnetic` checks the heading flown within 0.3°, where it allowed 2°.
- **Unchanged:** the digests, with protection and without (the digest flights fly no hsa), and the stock C172x's checkpoints; an hsa's course and a rotorcraft's heading; the `hold` behaviour and the velocity level, which fly a heading through the loops as before. The stock C172x's `hold` still settles 1.6° off its heading: its flights are pinned by the checkpoints.
- **Memory:** an hsa holds two more doubles, its heading trim and the heading it last saw.
- **A/B throughput** against FA-6e2b (the same tree with its two files as they were), both benches run from their own directories, 9 interleaved rounds of `micro`:
  - The cases are within −1.8 % to +2.3 % in their medians and −1.0 % to +1.1 % in their minimums; the `hsa` case, a course, −0.2 %.
  - `route` read +1.4 % (its minimum +1.1 %). Its functions are the same instructions in both builds but for the padding before a loop in `restart`, so that is placement. Against FA-6c2, `pattern`'s +1.2 % was the same.
  - None of the bench's cases flies a heading. A temporary case that did, measured against FA-6c2, read +0.9 %, 1 ns: the trim's comparisons and multiply-adds, with no division or call.
- ctest: all 321 tests pass.

**FA-6f1, A-GRA's civil path terminators: the legs that end at their fix (WPT-19).**
- **Flown** (`test_route_terminators`, calm; C172s east at 1,500 m and 55 m/s):
  - a track to fix on a rhumb route at 60 degrees north, 40 km east: its great circle 54.4 m poleward of the rhumb line in its middle (the geometry's 54.3 m), 0.0 m off it; the route's own leg on the rhumb line, 0.0 m off it;
  - a direct to fix after a waypoint flown over, 6 km on to the north: halfway, 248 m east of the line from the point flown over and never west of it; the route's own leg, back to within 28 m of that line;
  - a course to fix due north into a point 1 km east of the line on from the point before: within 11.0 m of its course's line and 0.52 degrees of its course in its last 2 km;
  - a radius to fix's quarter circle of 1,500 m, every field of its arc given: a C172 within 7.5 m of it, an IRIS within 0.06 m of its own of 50 m. Read back as given.
- **Refused `invalid_waypoint`, naming the point:** a code of 23; data at a point past the route, two for one point, a radius not finite, a way round of 2, a course on an arc, an arc's data on a course to fix, data on a track to fix, a course to fix without its course, an arc without its centre or its way round, a centre's latitude alone; the seven legs A-GRA 6.0a does not define (AF, CD, CR, FD, PI, VD, VR); its point 96 m inside its circle, the long way round, a radius of 1,450 m, a course in or out not its, a start not its, a length or a chord not its; an arc at a route's start; a branch to an arc (named at the branch's point); a track to fix after a start turn point; a direct to fix after a capture; a course to fix whose point before is past it; an arc after an orbit; an arc of 100 m, which the C172 cannot turn; a 65th (the 65th's point). On a linked route whose flight order is not as given, an arc's data and its radius refused at their point as given.
- **Not implemented** (FA-6f2): the legs to an altitude, an intercept, a distance or a manual termination, and the holds (CA, CI, FA, FC, FM, HA, HF, HM, VA, VI, VM).
- **The fleet** (`test_fleet`): every aircraft two orbit radii ahead, then a radius to fix's quarter circle of three radii round to the right, near 20 degrees of bank, and two radii on. All 35 complete, each within 20 m of its circle round its centre, over the ground, all along it: the wings from 5.6 m (the Skua) to 18.3 m (the E-3G, on its 22.4 km arc), the rotorcraft from 0.15 m (the IRIS) to 4.4 m (the UH-1H). Their routes' own cross-tracks read the same, to a centimetre.
  - First laid out in the plane at its start, as a start turn point's arc is, and read in the flat plane the fleet test lays points out in, the heavies' arcs of 15 to 28 km read 21 m (the B-52H) to 82 m (the KC-46A) off. A radius to fix's arc is now laid out in the plane at its centre, a circle round it on the Earth, and read by the great circle's distance from its centre.
- **A fix to FA-6e2a:** a branch refused on a linked route whose flight order is not as given named its point's place in that order - a branch at point 3 read 2. It now names its point as given (`test_route_branches`).
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6e2b's build (the hsa trim flies no route): a route without terminators lays its legs out as before, and a start turn point's arc keeps its plane at its start.
- **The support table:** `route/path_terminators` partial (the legs to an altitude, an intercept, a distance or a manual termination, and the holds: FA-6f2); the route capability's pending list names those in its place. The C ABI's, discovery's and Python's example of a row not built is now `route/metadata` (FA-7).
- **Conformance:** the optimise walks give points now and then a track, a direct or a course to fix, an initial fix, a radius to fix round a centre on its chord's bisector either way round, a leg not defined, or a code that is none.
- **Surfaces:** the C ABI's 1.35 block (a radius to fix read back; a procedure turn refused `invalid_waypoint` and a course to altitude `not_implemented`, `reserved` 3); Python's `test_route_terminators` (an arc by name, within 20 m, read back; refusals by name).
- **Memory:** a waypoint 240 bytes (232), a leg 192 (176), the path store 108,648 (99,432), a route's plan 169,864 (156,504), a setpoint 544 (520), a batch command 408 (392); an activity record unchanged, 304.
- **Digests:** identical to FA-6e2b's (and 84862b2's), with protection and without. The allocation gate passes.
- **A/B throughput** against the build before it - FA-6e2b with the hsa's heading trim (84862b2), built from its own tree with the same JSBSim - both run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −0.7 % to +1.2 %, but `default hold`'s +5.6 %, which reads +0.2 % from three copies: placement, as FA-6e2b's single copies showed. From three copies of each, their medians' median, every micro case is within −1.0 % to +0.3 %; `route` −0.6 %.
  - The command cases are within −5.7 % to +0.8 % from one copy, and −5.3 % to +0.4 % from three: a behaviour's NEW reads 125 ns where 84862b2's read 132 in all three copies.
  - World throughput is 99.8 to 100.7 % of 84862b2's; protection costs at most 1.2 %.
- ctest: all 325 tests pass.

**FA-6f2a, A-GRA's civil path terminators: the legs to an altitude, an intercept and a distance, and the heading legs (WPT-19).**
- **Flown** (`test_route_terminators`; C172s east at 1,500 m and 55 m/s, a 10 m/s wind from the north across their legs):
  - a course to an altitude, 6 km east to 1,650 m at 3 m/s: ended 4,963 m east (its point 9,000) at 1,640.0 m, within 3.2 m of its course; the direct to fix after it, from there;
  - a heading to the same altitude: ended 5,072 m east, 541 m south of its course, the wind's way;
  - a course to an intercept, its point where it meets a course to fix's line due north (6,000 m east, 1,000 m north): met it 5,872 m east and 958 m north, then within 2.8 m of the line and 0.30 degrees of its course.
- **Refused `invalid_waypoint`, naming the point:** a course to an altitude without the point's altitude; a track to fix after one (at the track's point: its start is not where the leg before ends) - a direct to fix after it accepted; a course to an intercept whose next is a track to fix (at the intercept's point); a heading to an intercept that ends the route; a course to an altitude at a loiter point.
- **Not implemented** (FA-6f2b): the legs to a manual termination, and the holds (FM, VM, HA, HF, HM).
- **The fleet** (`test_fleet`): every aircraft two orbit radii ahead, then a course to an altitude - 30 s of its best climb, 300 m at most - its point four radii on, and a direct to fix two radii to the right of that. All 35 complete, each course ended within 10 m below its altitude: the fighters' and the heavies' 300 m 3 to 5 % short of their points (the C-17A's 36.5 km on, where its point was 37.7), the C172's 151 m and the Skua's 90 m past theirs (2.9 km on where it was 1.4; 764 m where it was 527), the rotorcraft's - the IRIS's 60 m, the Crazyflie's 16 m, the UH-60's and the UH-1H's 150 m - as they climbed.
  - First built, a direct to fix ended only once the aircraft's track was on its point. Its course ended short of its point and its fix two orbit radii to the side, the Skua and five fighters (the Mirage 2000, the Typhoon, the Gripen, the F-15C, the Rafale) came to the end of their case's time still flying it: the point came abeam as they turned, and was not passed. A point come abeam as the aircraft turns to it is now passed there, as a point is passed abeam, and all six complete.
  - First asked to climb 300 m, the C172 and the IRIS were still climbing at their case's end, and the Crazyflie's battery ran out first (its best climb 0.52 m/s): it came down. The climb is now 30 s of the aircraft's best, 300 m at most.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6f1's build.
- **The support table:** `route/path_terminators` partial (a manual termination, a hold: FA-6f2b).
- **Conformance:** the optimise walks draw a course, a track or a heading to an altitude (now and then without it), an intercept (refused but before a course to fix), and a track for a distance beside FA-6f1's legs.
- **Surfaces:** the C ABI's 1.35 block and Python's `test_route_terminators` answer a manual termination `not_implemented` where they answered a course to an altitude.
- **Memory:** a route's behaviour holds a leg's end, an intercept's last cross-track and a heading leg's trim; a waypoint, a leg and the stores as FA-6f1's.
- **Digests:** identical to FA-6f1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6f1, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −0.7 % to +3.2 % (`hold`). From three copies of each, their medians' median, every micro case is within −1.3 % to +0.6 %: one of FA-6f1's copies ran most cases 3 to 8 % slow, as copies do.
  - The command cases are within −1.5 % to +2.4 % from one copy, and −1.0 % to +1.5 % from three.
  - World throughput is 99.4 to 101.5 % of FA-6f1's; protection costs at most 1.0 %.
- ctest: all 326 tests pass.

**FA-6f2b, A-GRA's civil path terminators: manual terminations and holds (WPT-19); FA-6f done.**
- **Flown** (`test_route_terminators`; C172s east at 1,500 m and 55 m/s, calm):
  - a track from a fix to a manual termination, its point 3 km on, and a branch there that takes the operator's input: flown on past its point until the operator commanded it, 3 km past, 175 s in; then direct to the branch's next, and completed;
  - a heading to a manual termination that ends the route: completed as its leg began (60 s in), then on its heading within 0.0 degrees a minute later;
  - holds at a point 6 km east: once round, 241 s held; to 1,700 m, its leg climbing to it at 0.5 m/s, 252 s held, left at 1,700.2 m; to 1,500 m, the altitude it came at, not held; until the operator ended it, commanded 400 s in, left at its fix 81 s later.
- **Refused `invalid_waypoint`, naming the point:** a track or a heading to a manual termination mid-route with no branch at its point taking the operator's input - at the route's end it is accepted; a hold's at no loiter point, at an orbit, once round given 2 laps, to an altitude given a duration or without its altitude, to a manual termination mid-route with no such branch.
- **The fleet** (`test_fleet`): every aircraft a loiter point three orbit radii ahead with a hold once round, then on two radii to its right. All 35 held and completed: a rate-one lap's 240 s (the rotorcraft, the Skua, the C172) to the C-17A's 628 s, its turns at 25 degrees of bank.
  - First placed eight radii ahead, the UH-1H was still holding when a rotorcraft's case time (330 s, the Crazyflie's battery's) ran out: the hold is at the route's first point.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6f2a's build.
- **The support table:** `route/path_terminators` supported on every aircraft; the route capability's pending list no longer names it. FA-6f is done; FA-6g (the stage's rows still partial) is next.
- **Conformance:** the optimise walks draw manual terminations (refused but where an operator's branch or the route's end ends them) and holds' terminators (refused but at a hold's loiter point) beside the other legs.
- **Surfaces:** the C ABI's 1.35 block refuses a manual termination mid-route that nothing ends (`reserved` 2); Python's `test_route_terminators` refuses one and accepts it with a branch that takes the operator's input; discovery's and Python's support checks read the row supported.
- **Memory:** a pattern holds one more flag, ended by the route that flies it.
- **Digests:** identical to FA-6f2a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6f2a, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −1.5 % to +2.1 %. From three copies of each, first −1.3 % to +2.2 % (`attitude, pseudo`, a flight level's case nothing here touches; one copy of each ran a case 6 to 15 % slow), then, run again, −1.5 % to +0.6 % (`attitude, pseudo` −0.1 %).
  - The command cases are within −2.0 % to +0.6 % from one copy; from three, first 0.0 to +2.1 % (a level switch's NEW: one copy read 77.4 ns where the others read 68.6), then +0.1 % to +1.5 %.
  - World throughput is 99.8 to 100.4 % of FA-6f2a's; protection costs at most 1.3 %.
- ctest: all 327 tests pass.

**FA-6g1, a start turn where its links loop back, its course left out (WPT-14).**
- **Flown** (`test_route_paths`; a C172 east at 1,500 m and 55 m/s, calm): a path of one point, 3 km east, on into a loop of five. The loop's first point, 6 km east, is a start turn, its course left out, whose arc runs to a point 2 km north and 2 km further east. The first lap comes to it heading east; the later laps come heading north, from the loop's last point 3 km south:
  - each lap began its arc on the course it came on: 90.0 degrees on the first lap, 0.0 on the next two. It was at most 5.0 m off the arc on each lap (the first 60 % of the segment, before the first lap's turn at its end);
  - each lap passed through its own arc's middle: the first lap within 3 m, the next two within 1 m, and a later lap more than a kilometre from the first lap's middle;
  - checked once and not kept: flying the first lap's arc on every lap, the later laps were 357 m off it and passed its middle 1,020 m away.
- **Refused `invalid_waypoint`, naming the point:** the paths test's loop, with B's first point a start turn whose course is left out. Its first lap's arc sweeps 90 degrees and its later laps' 180, so it is refused as any arc beyond 170 degrees is, naming the point the arc reaches. Until FA-6g1 it was refused `not_implemented`, naming the start.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6f2b's build.
- **The support table:** `route/next_segment` is supported on every aircraft, and the route capability's pending list no longer names the start turn.
- **Conformance:** the optimise walks draw links where a start turn's course is left out too.
- **Memory:** a route's plan holds one more index, where its later laps' legs differ; its behaviour lays them in place of the first lap's.
- **Digests:** identical to FA-6f2b's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6f2b, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, the micro cases are within −6.1 % to +0.5 %: FA-6f2b's own actuator median read high (34.5 ns, its least 32.4), and everything else is within ±1.4 %. From three copies of each, they are within −1.5 % to +0.4 %. Another session's build began in that run's last 16 s.
  - The command cases are within −1.5 % to 0.0 % from one copy, and −1.5 % to +1.6 % from three (a level switch's NEW, which nothing here touches: one copy read 66.8 ns where the others read 67.9 and 68.1).
  - World throughput is 98.9 to 100.4 % of FA-6f2b's; protection costs at most 1.3 %.
- ctest: all 328 tests pass.

**FA-6g2, a time of arrival at or after a loiter point (WPT-11).**
- **Flown** (`test_route_arrivals`; C172s east at 1,500 m and 55 m/s, calm): 3 km east, then a loiter point 8 km east with an orbit for 90 s, then a point 17 km east.
  - As planned (a window it was inside), its orbit began 130.3 s in and it arrived 407.1 s in.
  - Given a window 50 to 60 s later, it was slowed round the orbit and arrived at 459.6 s, aiming at 459.6 s. Its estimate through the orbit read 459.6 s, within 0.01 s, its delta 0.
  - Given a window at the loiter point 20 to 30 s later than it began its orbit as planned, it began at 152.8 s, aiming at 152.8 s.
  - A hold the operator ended: no estimate before it ended. The operator commanded its end 206 s in; it left the hold at 386 s and arrived at 562.1 s, inside its window of 557 to 857 s.
- **Refused `performance_limit`, naming the point**, as before (a C172, a one-minute orbit 3 km east, then 12 km on): a window after the loiter too soon for its fastest (`MaxAirspeed`) or too late for its slowest (`MinAirspeed`). A window it can make is taken; until FA-6g2 it was refused `not_implemented`.
- **The fleet** (`test_fleet`): each aircraft's loiter point was a minute ahead at its speed - or, where its orbit is joined two radii out, half a minute past that - with an orbit of 1.25 times its turn's radius for 30 s, then a point two minutes on. It flew first as planned. Once it told when it would arrive, the route was given again with a window 10 s wide: later by 8 % of its legs' time (slowed), or as much sooner where its slowest cannot take a tenth (the Crazyflie). All 35 arrived within 0.06 s of their aim, and their estimates as the orbit began were within as much of when they arrived.
  - First, nine heavies' windows were refused `performance_limit`. The host's check had measured an orbit joined where a leg too short for its join begins as if it were joined at its centre; it is now measured from where the leg begins.
  - With every loiter point a minute ahead, the E-3G's, KC-46A's and C-17A's routes began inside their orbit's join, 1.45 radii out. Turning 44 degrees onto their way in, more sharply than their turn allows, they took 10 to 12 s longer than the orbit's pieces measure, and arrived 4.7 to 6.4 s after their aim: the legs after could not make up the rest.
- **The estimate as planned, before the loiter** (a minute ahead, the host's default orbit, a hover for a rotorcraft):
  - Where the route joins an orbit two radii out, it came within 1.1 s (the B-52H, C-130J and F-16C). That is once the leg on is measured from the orbit's tangent: before that, the C-130J, H-6K, B-52H and KC-135R read up to 3.3 s late, and the E-3G's leg on was 7.3 s shorter than from the centre.
  - A C172 on a 206 m orbit came within 6.6 s, rolling into and out of it.
  - Through a hover, the Crazyflie and IRIS came within 0.7 s. The UH-60A and UH-1H took 23 and 72 s longer to settle over the point: their position loops close slowly from where they would stop, as FA-6b2 found.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6g1's build.
- **The support table:** `route/required_time_of_arrival` is supported wherever the aircraft has performance tables, and not implemented where it has none, as before. The route capability's pending list no longer names a time of arrival.
- **Conformance:** the optimise walks draw windows at and after loiter points, and links beside them.
- **Memory:** a route's plan holds a measure for each of its 16 loiters (6 numbers each); its behaviour holds three more numbers.
- **Digests:** identical to FA-6g1's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6g1, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - From one copy of each, with no other session's work in either run, the micro cases are within −0.7 % to +0.9 %. The one exception is the curve's −9.4 %: FA-6g1's own median read 303.6 ns where its copies read 274.6 to 275.8. From three copies of each, with another session's tests running, they are within −0.9 % to +0.8 %.
  - The command cases, from one copy with no other session's work, are within −0.4 % to +1.5 %, but for a behaviour's NEW: +3.1 % and +1.6 %. From three copies, with another session's tests running, they are within −1.5 % to +0.6 % and the NEW +4.1 %.
    - That NEW commands the built-in hold behaviour, which touches nothing FA-6g2 built. Every function on its path disassembles the same in both builds, addresses aside: the 12 functions that differ are the route's own, its behaviour's constructor and factory (its plan is larger), and two that differ only in a constant's label and alignment padding. Its reading is where its code now lies, not its work.
  - World throughput is 99.1 to 99.9 % of FA-6g1's, and protection costs at most 1.1 %. Another session's build began partway through the world run.
- ctest: all 329 tests pass.

**FA-6g2's finding closed: the endurance through an orbit that a short leg leads to (4.18, 4.31).**
- **Why it was off.** The endurance check times an orbit of so many laps as the route flies it: its way in from where its loiter begins, its laps, and on round to where it is left. Where the leg into its point is shorter than the orbit's join (two radii), the route begins the loiter where that leg begins (4.31). The check planned it from the circle's centre instead: no way in, and its laps counted from due north of the centre. FA-6g2 found the same in the schedule's measure of a loiter and fixed it there; the check kept its own copy of the old plan.
  - How far off that was turned on where the orbit is left. Flying north on to a point north, the lap from due north round to its exit ran 290 degrees and the check read long; flying east on to a point east, it ran 19 degrees and the check read short of the flight.
- **Fixed in the check**, as in the schedule: such an orbit is planned from where its leg begins, on the course that leg leaves on. Nothing flies differently.
- **Measured** (an E-3G at 3,000 m heading north at 180.55 m/s, calm, its orbit the host's 7,466 m; its loiter point a minute ahead, 1.45 radii, then a point two minutes on; each read by validating under a reserve of 0.9999, then flown):

  | its loiter point | laps | the check, before | the check, now | flown |
  | --- | --- | --- | --- | --- |
  | the first, its leg from the aircraft | 1 | 649.2 s | 529.3 s | 481 s |
  | the first | 2 | 909.1 s | 789.1 s | 741 s |
  | the second, its leg from a point 30 s ahead | 1 | 679.2 s | 559.3 s | 511 s |
  | the second | 2 | 939.1 s | 819.1 s | 771 s |
  | two radii and 30 s ahead, its leg longer than its join | 1 | 600.3 s | 600.3 s | 513 s |
  | the same | 2 | 860.2 s | 860.2 s | 773 s |

  - Here each orbit that a short leg leads to reads 119.9 s less, and its fuel with it: once round from the first point, 1,530.6 kg where it read 1,876.8 (flown, 1,439.2). Where the leg is longer than its join, the estimate is the same to the last bit.
  - A stock C172x east at 1,500 m and 55 m/s, a 1 km orbit once round 1,450 m ahead, then 3 km on, flew it in 205.5 s. The check reads 234.3 s; it read 201.3 s, short of the flight (its orbit's own time 32.9 s less, as the test below measures it).
  - What is left over is the model's (4.18): it counts each leg to its point, and the leg on from its point. So it counts the part of the leg into an orbit that the route leaves for its loiter (two radii of it, or a short leg's whole), and the leg on from the point rather than from where the orbit is left. The E-3G's reads 48 s (10 %) over its flight where it begins inside its join, and 87 s (17 %) where it joins two radii out; the C172x's 29 s (14 %). All read long.
- **Tested** (`test_route_loiters`; the C172x's geometry, validated only): each orbit's own time - the route's with it less without - against its pieces: the tangent from where it is joined, a lap, and on round to its exit's tangent. Two radii out, 161.429 s (its pieces 161.430 s); where a leg of 1.45 radii begins, from the aircraft and from the point before, 153.346 s each (its pieces 153.346 s). Built against the check as it was, those two read 120.418 s, and the test fails.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6g2's build.
- **Digests:** identical to FA-6g2's, with protection and without.
- **Code:** only `Endurance.cpp` is compiled again, and its code is 112 bytes smaller. What `libfsim.dll` places after it - the files the control library links after it (`Terrain.cpp` to `Schedule.cpp`), then every file's template and inline functions kept out of line - lies 112 bytes earlier, from the same objects. The check runs at a NEW, never stepped; no A/B throughput was run for it.
- ctest: all 330 tests pass.

**FA-6g3a, planned states at and after a loiter point (WPT-20).**
- **Flown** (`test_route_states`; a C172 east at 1,500 m and 55 m/s, calm): 3 km east, then a loiter point 8 km east with an orbit of 1 km for 90 s, then a point 17 km east. A state 5 km east is on the loiter point's own leg. A state 12 km east is after the loiter, within a kilometre of its leg: the leg on is laid from the orbit's exit, 0.55 km north of the state's place.
  - As planned, it crossed 12 km east 338.4 s in.
  - Given that state a time 30 s later and 1,600 m, it crossed 12 km east at 367.1 s, 1,598.7 m high, and stayed below 1,625 m on that leg. Its estimate as its orbit began read 368.4 s, the state's time. On the leg flown, which runs from the orbit's exit at 6.4 degrees from its centre, the state's place falls 61 m past 12 km east: that is the 1.3 s it crossed 12 km early.
- **Refused `invalid_waypoint`, naming the point:** a state on a loiter point's own leg past where its loiter begins, two radii out; one before it is taken. Until FA-6g3a, a state at or after a loiter point was refused `not_implemented`.
- **Its time through the loiter** is scheduled as a window's is (FA-6g2), a state's time a window of none; the fleet measured that schedule through an orbit on every aircraft.
- **Its climbs after a loiter** are measured from where the leg on begins, not from the loiter's point.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6g2's build.
- **The support table:** `route/inertial_states` is partial on every aircraft, now only beside points in moving frames (FA-6g3b). The route capability's pending list names only those.
- **Conformance:** the optimise walks draw states on any segment, at and after loiter points too, and links beside them.
- **Memory:** each loiter's measure holds its exit's place, and a route's plan holds the points its loiters have been left before.
- **Digests:** identical to FA-6g2's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6g2, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each.
  - No other session's work ran during any of it. From one copy of each, the micro cases are within −1.3 % to +0.3 %; from three copies, −1.5 % to +1.9 % (the waypoints' copies read 152.0 to 157.9 either way).
  - The command cases, from one copy, are within −2.8 % to +3.0 % (the same level's update, 6.6 ns, moves 0.2 ns); from three copies, −1.5 % to +0.4 %.
  - World throughput is 98.2 to 99.0 % of FA-6g2's, and protection costs at most 1.8 %. The world benchmark flies attitude commands alone. Every function on its path disassembles the same in both builds, addresses aside: the 20 that differ are the route's own and its plan's allocation. Its reading is where its code now lies, not its work.
- ctest: all 330 tests pass.

**FA-6g3b, planned states beside points in moving frames (WPT-20); FA-6 done.**
- **Flown** (`test_route_states`; a C172 east at 1,500 m and 55 m/s, calm): 3 km east, then on to a ship 12 km east moving north at 3 m/s. A state halfway along the leg to the ship is due 157 s in, 20 s later than 55 m/s would make it, at 1,600 m, placed where the leg's middle will be then (the ship 471 m north).
  - It passed the state's place at 157.0 s, 1,600.1 m high; its estimate 5 km east read 157.0 s.
- **Checked where the leg will be at the state's time:** beside a point drifting north at 1 m/s, the leg's middle is 100 m off the state's place by then, within its uncertainty: taken. At 10 m/s it is a kilometre off: refused `invalid_waypoint`, naming the point, though the leg passes through the place now. Until FA-6g3b a state beside a moving point was refused `not_implemented`.
- **Placed again** each update, as the leg is planned again: the altitude profile and the schedule follow the state's place on the leg as it moves.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6g3a's build.
- **The support table:** `route/inertial_states` is supported on every aircraft with performance tables, and partial on the stock C172x, which has none: a planned state's time, as an arrival window's (the feature table's new `withoutTables`). FA-6 is done; its route capability's pending list names FA-7's metadata and FA-16's contingencies.
- **Memory:** unchanged.
- **Digests:** identical to FA-6g3a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6g3a, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command` from three copies of each; later 7 rounds of `world` again, and 3 from three copies of each.
  - No other session's work ran during any of it, but for one more run of `world`, which another session's build overlapped: it is left out. From one copy of each, the micro cases are within −3.6 % to +1.9 %; from three copies, −1.5 % to +1.8 %.
  - The command cases, from one copy, are within −1.5 % to +2.8 %; from three copies, −0.6 % to +2.2 %.
  - World throughput from one copy of each read 100.9 to 108.9 % of FA-6g3a's, twice: the FA-6g3a build itself read 2 to 8 % below what it read an hour before, in its own A/B. From three copies of each, it is 99.0 to 100.6 %. FA-6g3a's copies read 857,036 to 859,521 vehicle-steps a second on the C172x, as FA-6g3b's do (856,116 to 864,264). Protection costs at most 1.0 %.
  - Addresses aside, these functions differ between the builds: the host's state checks (`checkStates`, `limitStates`), the new `legAt`, the support table's two, and the route behaviour's update. That last one's new call is on a moving route's path alone. On every other path it differs by one register's name, and by where two blocks it does not run lie. Four more functions differ only in their padding. None is on the path of the curve case (+1.9 % and +1.8 %, a curve's behaviour), the behaviour case (+2.8 % and +2.2 %, a hold) or the world benchmark (attitude commands). Their readings are where their code now lies, not their work.
- ctest: all 331 tests pass.

**FA-7a, A-GRA's route plans: the store, its activation states, execution and queries, and the route's planning metadata (RPL-01, RPL-02, RPL-05, RPL-08 to RPL-11, WPT-23).**
- What it built is 4.39, in C++, the C ABI (1.36) and Python. `fsim.plan/store` and `fsim.guidance.route/metadata` are supported on every aircraft; the route capability's pending list names FA-16's contingencies alone. The examples of a row not built (the C ABI's, discovery's and Python's) are now `curve/discretized`, FA-17's.
- **The VI's sequences replayed** (`test_route_plans`, 5 cases, 178 checks; its Python twin, 2 tests; `test_c_abi`'s 1.36 block), a C172 flying at 1,500 m:
  - Convert and upload (1.2.5.2): a plan published before FA listens for it is refused. Prepared for upload, it is taken - published twice, the later one kept - uploaded, and read back as given: its route, and every field of its metadata. A new version replaces it at its upload, its revision one more. An upload with nothing received fails `plan_not_received`. Malformed metadata is refused, and FA listens on: at no point, twice for one, a text not printable or too long, a fuel below 0.
  - Prepare for activation (1.2.5.3): a point it cannot fly fails its preparation, naming the point, as its route's validation does.
  - Activate (1.2.5.1) and its execution (1.2.6.6): the plan flies to its end - pending until its first step, executing, then complete (100 %) - its activation's command id kept.
  - Deactivate (1.2.5.4): ready, it is taken back. Activated to start a minute later, it is taken back and its activity canceled. Flying, its deactivation fails `plan_executing`, and it flies on. Another command taking its axes supersedes it.
  - FA's own deactivation (1.2.5.7), before it starts and after: its activity canceled with the platform's reason. Its grant revoked deactivates it too.
  - Queries (1.2.4.2, 1.2.6.4): every plan's id, version, revision and state, in the order they were made.
  - For planning use only (RPL-11): its preparation for activation fails `planning_only`. 32 plans are kept; the 33rd fails `plan_store_full`, and one forgotten makes room.
- **The fleet** (`test_fleet`): the route case's square, kept as a route plan on every aircraft - prepared for upload, published, uploaded, prepared for activation, activated - flies as the route does, within its class's thresholds; its plan's execution is complete at its end, on all 35 aircraft. The Python twin flies a plan straight ahead on one aircraft of each class.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-6g3b's build.
- **Memory:** a vehicle's plans are made at its first plan prepared for upload, behind one pointer at the host's end, and a flag in the hole after its vehicle's id says whether they are: the host grows by 8 bytes, and nothing before its end moves.
- **Digests:** identical to FA-6g3b's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-6g3b, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks - twice, the second after the fix below: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command`, and 3 of `world`, from three copies of each.
  - **The first run found the NEWs slower, and nothing else:** a level switch 8.0 % and a behaviour 12.0 %, from three copies of each. Every function on their path disassembles the same in both builds, addresses aside, but the end of an activity (`noteEnd`). That asked the plans' pointer, at the host's far end - a line nothing else in a NEW reads. With its flag moved to the hole after the vehicle's id, on the host's first line, the run again read a level switch 6.4 to 7.5 % slower and a behaviour 9.9 to 11.4 %: that line was little of it.
  - **Where the code lies was the rest.** The hot functions are the same, instruction for instruction, but not where they were: the host's `admits`, `checkAwareness` and `start`, and the World's `command`, `commandResult` and `levelChanged`, each begin 16 bytes further into its cache line. Built again with every function aligned to 64 bytes - 9 rounds of `command` twice, and 5 from three copies of each - the two builds read a level switch within −0.3 % to +1.5 % of each other, and a behaviour within +0.7 % to +2.4 % (124 to 128 ns). FA-6g3b's own build reads as the aligned ones do. FA-7a's lands its unchanged code 5 ns worse for a level switch and 13 ns for a behaviour, until another change moves it again.
  - **The rest is even.** The updates: the same level's 6.6 to 6.9 ns either way, a checked one within 1.6 %. The micro cases, from three copies of each, are within −1.3 % to +1.0 %. From one copy, some rounds' interference lifted four cases' medians 5 to 8 %, their least readings unchanged; the watcher saw no other session's work, but twice a bench whose command line it could not read. World throughput is 99.2 to 99.9 % of FA-6g3b's (99.1 to 99.8 % from three copies), and protection costs at most 0.6 %.
- ctest: all 336 tests pass.

**FA-7b, A-GRA's airfields and FA's own route plans (RPL-03, RPL-04, ENV-05).**
- What it built is 4.40, in C++, the C ABI (1.37) and Python. `fsim.plan/fa_plans` and `fsim.plan/airfields` are supported on every aircraft; of stage 7's rows, `fsim.plan/validate` is left (FA-7c).
- **The VI's sequences replayed** (`test_route_plans`, 2 cases more, 98 checks; its Python twin, 1 test more; `test_c_abi`'s 1.37 block), a C172 flying at 1,500 m:
  - Query Airfield Update (1.2.6.3): an airfield 20 km north, with a runway whose takeoff and landing coordinates are both given and one with its landing's start alone, is loaded and read back as given. Loaded again, it takes its own place, its revision one more. Each of 14 malformations A-GRA's schema would not take is refused, from a runway's id given twice to a QNH outside the altimeter's range. 32 airfields are kept; the 33rd fails `plan_store_full`.
  - Query Route Plan (1.2.6.4) and FA's own (1.2.5.2): FA's landing plan - an approach's path, then a landing's naming runway 2 of airfield 5 - is refused `unknown_airfield` until the airfield is loaded, and again for a runway it has not. Loaded, it is FA's and read only to MA: its preparation for upload and its removal are refused `read_only_plan`, and FA listens for no plan by its id. Its metadata reads back with its airfield and runway.
  - MA's own plan with a takeoff's, a departure's, an approach's or a landing's path - each of the eleven types - is refused as published, `safety_critical_plan`, FA listening on; one with a taxi route is taken.
  - FA's departure plan, activated by MA, flies. It is refused a reload while it flies, and, deactivated by FA, reloads, its revision 2.
- **The fleet** (`test_fleet`'s command interface): on every aircraft, an airfield where it began and FA's departure plan over the route it flies, loaded by the platform. MA is refused its upload, and flies it once activated: executing 2 s later, on all 35 aircraft.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-7a's build.
- **Memory:** the airfields live in the plans' store, made at the first loaded or prepared for upload; the host does not grow.
- **Digests:** identical to FA-7a's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-7a, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command`, and 3 of `world`, from three copies of each; and 9 rounds of `command` twice, and 5 from three copies of each, between the two built with every function aligned to 64 bytes.
  - No other session's work ran during any of it. From one copy of each, the micro cases are within −3.5 % to +0.4 %; from three copies, −0.6 % to +0.4 %.
  - The command cases are within −3.2 % to +3.0 % from one copy - the same level's update 6.6 to 6.8 ns either way - and within −3.1 % to +0.4 % from three copies, the NEWs a little faster. Built with every function aligned to 64 bytes, a level switch reads within −1.3 % to +0.5 %, and a behaviour −3.2 % to −1.8 %.
  - World throughput is 99.7 to 100.1 % of FA-7a's (100.0 to 100.4 % from three copies), and protection costs at most 1.0 %.
- ctest: all 338 tests pass.

**FA-7c, A-GRA's route plan validation, in weather, and patches (RPL-06, RPL-07, ENV-10).**
- What it built is 4.41, in C++, the C ABI (1.38) and Python. `fsim.plan/validate` is supported on every aircraft. Stage 7 has no row left: FA-7 is done.
- **The VI's sequences replayed** (`test_route_plans`, 1 case more, 28 checks; its Python twin, 1 test more; `test_c_abi`'s 1.38 block), a C172 flying at 1,500 m at 55 m/s:
  - Route Plan Validation (1.2.5.5): two corners 1,200 m apart. The C172's turns at 55 m/s fit its legs in calm air, the wind its air data measure. With 20 m/s of wind behind them, or 20 m/s of gusts, they do not: invalid, its first point named (`invalid_waypoint`), with a finding for each. Modified to validate, the turns are flown smaller: valid, with adjustments. Nothing flies, and nothing is kept.
  - From an origin 1,400 m below it, the climb to its first point is steeper than the aircraft climbs: invalid. From 50 m below, valid.
  - Plan Patch Verification (1.2.5.6): the corners on the primary path, an alternate's leg straight on from them. In the wind, the alternate's part is valid; the primary's is not, nor is the whole plan.
  - A kept plan - one for planning use only - validates by its id, as uploaded. An unknown id is refused `unknown_plan`, and one prepared for upload with nothing uploaded `wrong_plan_state`. Each of five malformed validations is refused `invalid_parameter`, from a wind given one way alone to a part that is no path type.
  - A-GRA's form (`fsim.agra.route_plan_validation`): INVALID, its first path's first segment named; VALID once modified.
- **The fleet** (`test_fleet`'s route plan case): on every aircraft, the plan is validated first, in calm air and modified to validate, as MA would before sending it: valid on all 35, then flown as before. Unmodified, the UH-1H's alone is not: its legs, 300 m (20 s at its 15 m/s cruise), are too short for its fly-by turns at the first two points, which its route flies smaller - under Reject, a turn error each.
- **The route checks' wind:** the validation's wind reaches the checks as the wind itself. The four places that took it from the air data now ask the host (`checkWind`), which gives the validation's while one runs, else the measurement as before. Moving the aircraft's ground velocity by the wind instead read a hovering UH-1H as flying.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-7b's build.
- **Memory:** the validation's wind is held in the plans' store (made at the first validation) while it runs; the host does not grow.
- **Digests:** identical to FA-7b's, with protection and without. The allocation gate passes.
- **A/B throughput** against FA-7b, both builds run from their own directories once two minutes had passed with no other session's builds, tests or benchmarks: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command`, and 3 of `world`, from three copies of each; and 9 rounds of `command` twice, and 5 from three copies of each, between the two built with every function aligned to 64 bytes. Two earlier runs, which another session's helicopter flight tests disturbed, were discarded; the run now stops and starts again whenever another session's work appears.
  - The micro cases are within −2.5 % to +2.4 % from one copy, and −2.1 % to +3.1 % from three.
  - The NEWs read faster: a level switch by 5.0 to 7.4 % (5.9 % from three copies), a behaviour by 5.5 to 7.8 % (7.3 %). The same level's update and a checked update are within −1.5 % to +1.5 %.
  - Built with every function aligned, a level switch reads within −0.6 % to +0.1 %, but a behaviour +9.0 % and +9.7 % from one copy - and +2.6 % from three. There, one of FA-7b's own copies read 139.5 ns against its siblings' 124.3 and 124.8: where a copy loads moves that case by 12 %.
  - No instruction on the NEW path changed: `submitWith` is instruction for instruction FA-7b's. The one function there that changed is the command variant's destructor, where GCC now inlines part of a map's teardown (31 to 52 instructions); an empty map skips it for two more pushes and pops. The rest is where the code lies, as FA-7a's was.
  - World throughput is 99.4 to 100.7 % of FA-7b's, and 100.1 to 101.1 % run again apart. From three copies of each, a first run drifted as it went - FA-7c's later copies reading lower, 96.3 to 99.4 % - and run again read 98.6 to 101.0 %. On the step path too, the command variant's destructor is the one function that changed. Protection costs at most 1.5 %.
- ctest: all 339 tests pass.

**FA-8a, A-GRA's must fly: points, entities, operational points and the ingress window (MFY-01, MFY-02, MFY-07; MFY-03 and ENV-06 for points).**
- What it built is 4.42, in C++, the C ABI (1.39) and Python. `fsim.guidance.must_fly` and `fsim.geometry` are partial on every aircraft: zones, corridors and volumes, and operational lines, zones and volumes, are FA-8b's.
- **Flown** (`test_must_fly`, 5 cases, 121 checks; its Python twin, 4 tests; `test_c_abi`'s 1.39 block):
  - A point ahead and to the side, 100 m up (a rotorcraft's 20 m), flown over and completed as it is passed, per class: a C172 within 40 m of it and 20 m of its height, an F-16C within 80 m and 30 m, a UH-60A within 15 m and 5 m, an IRIS within 5 m and 3 m. Through Python a C172 passed 5.1 m off.
  - A window of bearings: from the south (the C172 heading east, through one approach point), from the east (behind it: round through a point abeam, then one on the window's edge), from the west (straight in), and a jet from the south. Each came from within its window.
  - Another vehicle: an F-16C 10 km north of a C172 flying east flew over it 37 m off and 163 m above - 500 ft, given no altitude. With that vehicle removed, the must fly over it failed `target_lost`. The aircraft itself, or a vehicle the world does not have, is refused.
  - An operational point (OpPoint 7) kept, read back and set again (revision 2), each of nine malformed ones refused, and flown with its own altitude and window. One the world does not keep, or one removed, is refused `unknown_geometry`.
  - Each malformed field is refused naming it. An UPDATE moves the point, keeping the rest, and flies it afresh.
- **The fleet** (`test_fleet`, a case of its own): on every aircraft, a point a leg ahead and half a leg to the right at its altitude, approached from the east - the aircraft coming from the south-west, round through the points laid out - was flown over on all 35. They passed it 0.06 m (the IRIS) to 34.5 m (the Gripen) off: the fly-by-wire jets 14 to 34.5 m, the direct wings at most 9.3 m, the helicopters 2.3 and 3.3 m, the multirotors 0.06 and 0.17 m. Each came from 93.4 to 95.5 degrees, the window 80 to 100, aimed at 95. Each completed within 39 s (the IRIS) to 456 s (the C-17A).
- **Entities:** pursuing where the vehicle was - the route's own way with a moving point - left the F-16C 540 m off the C172, its cross-track growing as the leg swung. The must fly now aims its first leg where the vehicle will be, and closes on it.
- **Conformance:** a must fly answers NEW, UPDATE and CANCEL as its descriptor says on every aircraft shipped. The random walks draw it only in the walks of their own, so the others draw what they drew before: drawn in all of them, it moved every walk, and the rare `task:completed` went unmet.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-7c's build.
- **Digests:** identical to FA-7c's, with protection and without. The allocation gate passes, with two cases more: a must fly's point moved every step through UPDATE, and another vehicle flown over, its leg aimed anew every step.
- **A/B throughput** against FA-7c, both builds run from their own directories in a quiet window held throughout (no other session's builds, tests or benchmarks): 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command`, and 3 of `world`, from three copies of each; and 9 rounds of `command` twice, and 5 from three copies of each, between the two built with every function aligned to 64 bytes.
  - The micro cases are within −1.3 % to +2.2 % from one copy, and −0.4 % to +2.4 % from three: the axes apart and the default hold 2.3 and 2.4 % (1.7 and 2.2 ns), cases that move with where the runtime's code lies.
  - The NEWs read slower in the default builds: a level switch by 7.6 to 9.4 % (8.8 % from three copies), a behaviour by 12.3 to 12.9 %. The same level's update and a checked update are within 0.0 to +3.0 %.
  - Built with every function aligned, a level switch reads within +0.0 % to +0.4 %, and a behaviour −4.3 % and −5.1 % from one copy and +0.4 % from three.
  - The NEW path gains what a new mode asks: a case more in the command's copy, and the must fly's index compared in `indexOf` (2 instructions more), `prepare` (12) and `submitWith`. A NEW of anything else runs a handful of them. `axesOf` and `arbitrate` are unchanged and `launch` one instruction apart; the rest of `submitWith`'s difference is its blocks laid out afresh. The rest of the default builds' difference is where the code lies, as FA-7a's was.
  - World throughput is 99.6 to 100.1 % of FA-7c's, and 100.0 to 100.8 % from three copies. Protection costs at most 0.9 %.
- ctest: all 344 tests pass.

**FA-8b1, A-GRA's must fly into a zone; operational zones (MFY-04; MFY-03 and ENV-06 for zones).**
- What it built is 4.43, in C++, the C ABI (1.40) and Python. `fsim.guidance.must_fly` and `fsim.geometry` stay partial on every aircraft: corridors and volumes, and operational lines and volumes, are FA-8b2's and FA-8b3's.
- **Flown** (`test_must_fly_zones`, 4 cases, 78 checks; its Python twin, a test more; `test_c_abi`'s 1.40 block):
  - A zone of each shape, per class: an ellipse 5 km east of a C172, a rectangle turned 30 degrees 10 km north-east of an F-16C, a polygon with a hole ahead of a UH-60A, and a slant range area's quarter ring 200 m north of an IRIS. Each completed in its zone, within a metre.
  - A band above the aircraft - 1,700 to 1,900 m, a C172 at 1,500 m - climbed into, and completed in it. A window from the north (bearings 350 to 10 degrees from the zone's centre), the aircraft to its west: it came round and went in heading within 12 degrees of south.
  - An operational zone (12) kept, set again (revision 2) and flown into by its id. A rectangle in a frame turned 90 degrees, and a circle moving north at 15 m/s: each entered where it was then. One the world does not keep, or one removed, is refused `unknown_geometry`.
  - Ten malformed zones are refused naming their fields, 10 to 18. So are an altitude given outside the zone's band, or in another reference (fields 3 and 4), a Zone given no zone, and a zone kept with id 0.
  - An UPDATE's zone, moved 3 km north, was flown to and entered. A must fly queued behind a start window with its zone started 20 s later and entered it.
- **The fleet** (`test_fleet`, a case of its own): on every aircraft, a square half a leg across - its centre a leg ahead and half a leg to the right, spanning 100 m below the aircraft to 100 m above (a rotorcraft's 20 m) - was entered on all 35. Each completed as it crossed the edge: from 0.02 m inside it (the CF2) to 34.4 m (the KC-46A), an update's flight past it. None was more than 19.4 m from the altitude it began at (the Mirage 2000). Each completed within 17 s (the IRIS) to 114 s (the C-17A).
- **Found and fixed:**
  - A must fly queued behind a window never wrote its route as it started, and failed `behavior_failed` - FA-8a's, found by the queued zone. It now writes it, as a route does.
  - An altitude given outside a zone's band was flown as given, so the zone was never entered and the must fly never completed. It is now refused.
  - The world's zone is read where it is kept. A copy would have allocated for a polygon at every UPDATE by its id, and within a step where a waiting must fly starts.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-8a's build.
- **Digests:** identical to FA-8a's, with protection and without. The allocation gate passes, with three cases more: a polygon with a hole given and moved every step through UPDATE; a polygon kept by the world, its must fly updated every step by its id; and a moving ellipse flown into by its id.
- **A/B throughput** against FA-8a, both builds run from their own directories in a quiet window held throughout: 5 rounds of `micro`, 9 of `command` twice, 7 of `world`; then 5 rounds of `micro` and of `command`, and 3 of `world`, from three copies of each; and 9 rounds of `command` twice, and 5 from three copies of each, between the two built with every function aligned to 64 bytes.
  - The micro cases are within −1.1 % to +0.8 % from one copy, and −1.0 % to +0.5 % from three.
  - The NEWs read faster in the default builds: a level switch by 7.9 and 8.3 % (7.0 % from three copies), a behaviour by 10.5 and 12.5 % (11.5 %) - FA-8a's rise from where the code lay, undone. The same level's update and a checked update are within −4.3 % to +1.6 %.
  - Built with every function aligned, a level switch reads within −0.7 % to +0.3 %, and a behaviour −1.7 % and −1.5 % from one copy. From three copies a behaviour reads −8.3 %: FA-8a's copies spread from 129.7 to 146.2 ns, FA-8b1's from 126.9 to 128.5.
  - World throughput is 99.5 to 99.9 % of FA-8a's, and 100.3 to 100.8 % from three copies. Protection costs at most 0.9 %.
- ctest: all 348 tests pass.

**FA-8b2, A-GRA's must fly through a corridor; operational lines (MFY-05; MFY-03 and ENV-06 for lines).**
- What it built is 4.44, in C++, the C ABI (1.41) and Python. `fsim.guidance.must_fly` and `fsim.geometry` stay partial on every aircraft: volumes, and operational volumes, are FA-8b3's.
- **Flown** (`test_must_fly_lines`, 4 cases, 89 checks; its Python twin, a test more; `test_c_abi`'s 1.41 block):
  - A corridor with turns, per class: an L ahead of a C172 (a right angle, 500 m either side), a Z ahead of an F-16C (two 45-degree turns, 3 km either side), and an S ahead of a UH-60A (150 m) and of an IRIS (25 m). Past its first vertex each stayed within its widths: the C172 122 m off its line at its turn, the F-16C 170 m, the UH-60A 15 m, the IRIS 0.9 m. Each completed 0.4 to 6.5 m from its last vertex.
  - A band above the aircraft (1,700 to 1,800 m) was climbed into, and the vertices' own altitudes flown (the last's 1,650 m, within 15 m). A corridor was entered from a window to its west, coming in within 20 degrees of east. A right angle in a corridor 20 m wide is refused `performance_limit` (`max_turn_rate`), naming its point.
  - An operational line (31) kept, set again (revision 2) and flown by its id; one in a frame turned 90 degrees; one moving north at 10 m/s: each done within 40 m of its last vertex where it was then. One the world does not keep, or one removed, is refused `unknown_geometry`.
  - Eight malformed lines are refused naming their fields, 10 to 15. So are an altitude given outside its band, or in another reference (fields 3 and 4), a Line given no line, and a line kept with id 0.
  - An UPDATE's line, moved 2 km north, was flown through; an UPDATE of its speed alone kept its line. A must fly queued behind a start window with its line started 20 s later and flew it.
- **The fleet** (`test_fleet`, a case of its own): on every aircraft, a corridor from half a leg ahead, a leg on, then a leg turned 60 degrees right, half a leg wide either side, was flown through on all 35. Past its first vertex, none was further off its line than 0.052 of a leg (the EA-18G) - a rotorcraft 0.125 (the UH-1H) - the cut of its turn. Each completed 0.03 m (the CF2) to 22.8 m (the KC-46A) from its last vertex, within 51 s (the IRIS) to 353 s (the C-17A).
- **Found and fixed:**
  - FA-8b1's zone in a turned frame read a window of bearings - and, over the zone already, the aircraft's track - along the frame's axes rather than from true north. A window from the north on a frame turned 90 degrees sent the aircraft in from −82 degrees. Both are turned into the zone's plane now, and a test flies it from within 12 degrees of north.
  - A must fly's UPDATE refused what its checks found, and checked its terrain, only under a range policy. A NEW and a route's UPDATE do both whatever it is, and now so does it.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-8b1's build.
- **Digests:** identical to FA-8b1's, with protection and without. The allocation gate passes, with two cases more: a corridor with a turn given and moved every step through UPDATE, and one kept by the world, its must fly's speed updated every step by its id.
- **A/B throughput** against FA-8b1, both builds run from their own directories in a quiet window held throughout, as FA-8b1's were measured.
  - The micro cases are within −1.5 % to +1.1 % from one copy, and −1.3 % to +1.7 % from three.
  - The NEWs in the default builds: a level switch +1.2 % and −1.3 % (+0.6 % from three copies), a behaviour −1.6 % twice (+1.0 %); the same level's update and a checked update within −1.6 % to +1.5 %. Built with every function aligned, all four within −2.9 % to +3.0 %.
  - World throughput is 98.4 to 100.8 % of FA-8b1's, and 100.0 to 100.3 % from three copies. Protection costs at most 1.5 %.
- ctest: all 352 tests pass.

**FA-8b3, A-GRA's must fly into a volume; operational volumes (MFY-06; MFY-03 and ENV-06 for volumes). FA-8b done.**
- What it built is 4.45, in C++, the C ABI (1.42) and Python. `fsim.guidance.must_fly` and `fsim.geometry` are supported on every aircraft. A-GRA's orbital volumes lie outside the atmosphere, and the surfaces do not express them.
- **Flown** (`test_must_fly_volumes`, 4 cases, 80 checks; its Python twin, a test more; `test_c_abi`'s 1.42 block):
  - A volume of each kind, per class: a sphere 5 km east of a C172; a cone 6 km ahead of an F-16C, opening away from it 10 degrees each side (entered just past its vertex); a dome beside a UH-60A; a column 30 m across beside an IRIS. Each completed in its volume, within a metre.
  - A sphere whose lowest point was 500 m above a C172, climbed into. An ellipsoid entered from the north through its window. A geocentric box, 1,700 to 1,900 m, climbed into within its bounds. A sphere moving north at 10 m/s, entered where it was then. A sphere 300 m round another aircraft flying north, in the frame that follows it, flown into.
  - An operational volume (51) kept, set again (revision 2), read back and entered by its id. One the world does not keep, or one removed, is refused `unknown_geometry`.
  - Nine malformed volumes are refused naming their fields, 10 to 16. So are an altitude outside the volume over its inner point, or in another reference (fields 3 and 4), a Volume given no volume, and a volume kept with id 0.
  - An UPDATE's volume, moved 3 km north, was entered. A must fly queued behind a start window with its volume started 20 s later and entered it.
- **The fleet** (`test_fleet`, a case of its own): on every aircraft, a sphere a quarter of a leg in radius - its centre a leg ahead, half a leg to the right and 100 m above the aircraft (a rotorcraft's 20 m) - was entered on all 35. Each completed inside it, from 0.01 m (the CF2, after climbing 14.8 m into its 5 m sphere) to 7.95 m (the E-3G), within 21 s (the IRIS) to 126 s (the C-17A).
- **Found and fixed:**
  - The multirotors reached their aim before they had climbed into their spheres, flew on along their course, and never entered them. A zone's or a volume's route now ends in a loiter at its aim - a rotorcraft stops over it, a wing orbits it - so the aircraft goes on until it is in. FA-8b1's zones end so too; their tests and fleet case read as before.
  - The IRIS's height lay exactly at its sphere's lowest point, which counts as inside, so it went in there with no room and took 52 s to complete. A volume's height is now held between its top and bottom over its inner point as a zone's band holds it: a tenth of the way in, 30 m at most. The IRIS then went in 3.8 m up, in 21 s.
  - FA-8a's test named location 7 as a code that was not one; 7 is now a volume's.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-8b2's build.
- **Digests:** identical to FA-8b2's, with protection and without. The allocation gate passes, with three cases more: an ellipsoid given and moved every step through UPDATE; a column kept by the world, its must fly's speed updated every step by its id; and a sphere round another vehicle, tested every step where that vehicle is.
- **A/B throughput** against FA-8b2, both builds run from their own directories in a quiet window held throughout, as FA-8b2's were measured.
  - The micro cases are within −1.3 % to +1.6 % from one copy, and −1.3 % to +1.3 % from three.
  - The NEWs in the default builds: a level switch +0.6 % and −0.9 % (−1.0 % from three copies), a behaviour +0.7 % and +1.4 % (−2.2 %); the same level's update and a checked update within −1.5 % to +1.2 %. Built with every function aligned, all four within −3.6 % to +2.9 %.
  - World throughput from three copies first read 98.0 to 99.4 % of FA-8b2's, one directory's drifting as before (FA-7c's). Run again apart, it read 99.4 to 100.2 % from one copy and 99.5 to 100.3 % from three. Protection costs at most 1.1 %.
- ctest: all 356 tests pass.

**FA-8c, A-GRA's altitude stacked marshall (ASM-01).**
- What it built is 4.46, in C++, the C ABI (1.43) and Python. `fsim.guidance.marshall` is supported on every aircraft, its hover as the pattern's is (R1).
- **Flown** (`test_marshall`, 3 cases, 1,192 checks; its Python twin; `test_c_abi`'s 1.43 block):
  - A stack of four per class, each NEW given the lowest slot left, then flown 150 s and measured for a minute more:
    - four C172s orbiting from 1,500 m, 1,000 ft apart: at worst 0.43 m off their slots, every two at least 304.05 m apart;
    - four F-16Cs from 3,000 m: 1.62 m, and 304.62 m;
    - four UH-60As hovering 30 m apart, and four IRIS 10 m apart: under a millimetre off, their separations held.
  - The slots of a stack of three, 1,500 to 2,100 m and 300 m apart: the top asked for and given; the others the lowest left; a fourth refused `stack_full` naming field 3, as was one asking for a slot taken. Another point 5 km away had a stack of its own. A slot freed by a CANCEL went to the next NEW. An UPDATE that moved the stack to the other point chose afresh (1,800 m: its 1,500 m taken). One that left it (a speed) kept its slot. A `PatternCommand` sent to it was refused `wrong_command_type`.
  - Refused, naming their fields: a hold (0), a racetrack without its second circle (18), the least left out (10), a most below it (11), a separation of 0 (12), a reference that is not one (4). A hover on the hangar's C172, which declares it hovers not, was refused `not_supported`, as its row says. A racetrack by two circles was accepted, its second circle read back. A marshall kept as a task was refused `invalid_parameter`. One queued behind a start window held its slot as it waited - the next NEW took 1,804.8 m - and started 20 s later.
  - In Python, two C172s were given 1,500 and 1,800 m of a stack that stops at 1,900 m, and a third was refused `stack_full` at field 3 (A-GRA's INSUFFICIENT_RESOURCES). An UPDATE of the speed kept its slot, and both flew within 5 m of their slots after a minute. Through the C ABI, the two lowest slots read back, and a third was refused `stack_full` with its slot named (reserved 4).
- **The fleet** (`test_fleet`, a case of its own): every aircraft alone in a stack round where it is, its least 100 m above it (a rotorcraft's 20 m, hovering), was given that slot on all 35, climbed to it and held it. Over its last 30 s it was at worst 9.09 m off (the Gripen), then 5.17 m (the Mirage 2000) and 4.03 m (the Typhoon); every rotorcraft under a centimetre.
- **Found and fixed:**
  - The marshall first joined the command variant as its twelfth alternative. The A/B read the step cases that run the control stack's merged path 2 to 4 % slower from three copies - twelve of them, the design's, the pseudo-controls', the limits' and the reports' among them - and a NEW of another level 4.0 % slower, 4.6 % built with every function aligned. `ControlStack::flyMerged`, whose source had not changed, compiled differently: libstdc++ visits a variant of eleven alternatives or fewer through a switch, and one of more through a table of calls. The marshall now flies as its pattern with its stack beside it (4.46), and the variant is FA-8b3's again: `flyMerged` the same instruction for instruction. A static assertion holds the variant to eleven.
  - The catalog named the marshall's capability `user.guidance.marshall` until the built-in modes' list took it.
  - A malformed stack was refused `stack_full` by the world's slot choice before the host saw it. The world now passes such a marshall on, and the host names its field.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-8b3's build. The flights are the same after the rework: every stack's and every fleet aircraft's numbers above to the last digit printed.
- **Digests:** identical to FA-8b3's, with protection and without. The allocation gate passes, with a case more: a marshall round a point ahead, its speed updated every step - its stack left, its slot kept.
- **A/B throughput** against FA-8b3, both builds run from their own directories in a quiet window held throughout, as FA-8b3's were measured:
  - The micro cases are within −1.7 % to +1.4 % from one copy, and −1.5 % to +1.2 % from three.
  - The NEWs in the default builds: a level switch +1.9 % and +2.0 % (+0.4 % from three copies), a behaviour −1.1 % and −1.2 % (−3.7 %); the same level's update and a checked update within −4.3 % to +0.0 %. Built with every function aligned: a level switch +1.9 % and +3.3 % (+1.6 % from three), a behaviour +3.8 % and +3.0 % (+1.9 %), the other two within −1.5 % to +0.4 %.
  - World throughput is 99.7 to 100.4 % of FA-8b3's, and 98.6 to 100.5 % from three copies. Protection costs at most 0.5 %.
- ctest: all 359 tests pass.

**FA-8d, A-GRA's route intercept (RIC-01 to RIC-03; CAP-02). FA-8 done.**
- What it built is 4.47, in C++, the C ABI (1.44) and Python. `fsim.guidance.intercept` is supported on every aircraft. With it and FA-8's must fly and marshall, eight of A-GRA's ten flight capability types are offered; LAUNCH and RECOVERY are FA-9 to FA-11's (CAP-02).
- **Flown** (`test_intercept`, 3 cases, 829 checks; its Python twin, 2 tests; `test_c_abi`'s 1.44 block):
  - Per class - a C172, an F-16C, a UH-60A and an IRIS - a plan of six points along the aircraft's course to its left, the aircraft abeam 40 % of the leg into point 3. Each method joined where it should: the beginning at point 0; Discrete at point 2, the nearest; ShortestDistance on the leg into point 3, where the perpendicular meets it (within a metre); Soonest on that leg, a wing's ahead of that place, a rotorcraft's at it. Each flew the plan from there to its end, capturing each point in turn, the plan Activated, Executing, then Complete. The points after the join were passed within 30 m (the C172), 250 m (the F-16C: 192 m at worst, at the point after it turned back onto the plan's beginning), 10 m (the UH-60A: 7.4 m) and 3 m (the IRIS: 2.5 m) - the route follower's, as any route's.
  - Bounds and a path: from point 4 on, Discrete joined point 4; up to point 1, ShortestDistance joined point 1; point 3 alone, Soonest joined its leg. On a path of its own (an alternate of three points 4 km to the aircraft's right), Discrete joined its first.
  - Its status: flying to the point laid in, the current segment was the leg it joins (point 3, its id 103, its path the primary's), its heading the leg's course within half a degree, none before it; the next segment 2.9 km or more beyond. On the leg, having captured point 3: the previous segment's capture time within 1.5 s of when it did, its distance 0; the current's velocity along it the ground speed within 2 m/s, across it under 2 m/s; its capture time now plus its distance at the ground speed. A loiter of two orbits at point 4 was reported as it flew it, its orbits counted.
  - Refused: a plan it does not keep (`unknown_plan`); one prepared for upload, never uploaded (`wrong_plan_state`); plan 0 (0), a method that is not one (2), a path the plan has not (1), an earliest that is no point (3), a latest before it (4); a plan for planning use only (`planning_only`). Validated alone, nothing flew and the plan stayed Uploaded. An UPDATE, its own or its route's, was refused `not_updatable`; a task, `not_implemented`. Its setpoint read back the intercept, its join, and the plan's six points with the one laid in. Intercepted again while it flew, the new one replaced it; diverted by an hsa, the plan's execution was Superseded; intercepted again, Executing, the plan's activity the new one's. One queued behind a start window joined the leg into point 3 at its NEW and, laid afresh as it started 45 s later, the leg into point 4.
  - In Python the four ways joined as in C++, the soonest ahead of the perpendicular's foot; a batch item joined point 2; an UPDATE was refused `not_updatable`, an unknown plan `unknown_plan`, a method of 7 at field 2. Through the C ABI: joined on the leg into point 3, a point laid in (4); its status and setpoint read back; no UPDATE; a method of 7 refused at field 2 (reserved 3); an unknown plan refused.
- **The fleet** (`test_fleet`, a case of its own): on every aircraft a plan of four points along its heading, half a leg to its right from half a leg behind it, was joined by Soonest on its leg into point 1 on all 35 - each rotorcraft where the perpendicular meets it, each wing ahead of that, from 132 m (the Skua) to 9.4 km (the C-17A) - and flown to its end: completed, the plan's execution Complete, its last point passed at worst 8.1 m off (the Mirage 2000), a helicopter's 0.46 m, a multirotor's 0.05 m.
- **Found and fixed:**
  - Abeam a point exactly, ShortestDistance chose the leg after it over the point by a floating point hair, and laid a join 0.24 m from the point. A tie is now within a billionth of the score, the earlier taken, and a join within a metre of a leg's start is that point's.
  - A second intercept of a plan in flight was first refused `plan_executing`. The rejoin is the intercept's use, so it now replaces the plan's live activity as any NEW does.
  - The conformance held every mode to take UPDATE; the intercept, which takes none, is named its exception.
  - Python's batch item named its submit method `method`, as the intercept names a field: the item's is now positional alone.
- **Unchanged, to the last bit:** the route probe (120 lines) and the curve probe (64), identical to FA-8c's build.
- **Digests:** identical to FA-8c's, with protection and without. The allocation gate passes, with a case more: an intercept queued behind a start window, started in a counted step - its join laid then, in the room its NEW made.
- **A/B throughput** against FA-8c, both builds run from their own directories in a quiet window held throughout, as FA-8c's were measured:
  - The micro cases are within −0.4 % to +1.2 % from one copy, and −1.7 % to +1.5 % from three.
  - The NEWs in the default builds: a level switch −1.0 % and −0.1 % (−0.6 % from three copies), a behaviour +2.6 % and +2.9 % (+1.1 %); the same level's update and a checked update within +0.0 % to +1.5 % (+0.0 %). Built with every function aligned: a level switch −1.9 % and −1.2 % (−1.5 % from three), a behaviour +1.2 % and +1.4 % (+1.6 %), the other two within −1.5 % to +1.5 %.
  - World throughput is 99.0 to 100.0 % of FA-8c's, and 100.0 to 100.4 % from three copies. Protection costs at most 1.0 %.
- ctest: all 362 tests pass.

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
