# C ABI

`#include <fsim/fsim_c.h>` - the binding surface for any language with a C
FFI (Rust `bindgen`, C#, Julia, Go, ...). The platform's own Python SDK
([python.md](python.md)) is built on it.

Rules: opaque handles, plain structs with a leading `struct_size` (call the
`*_init` function, then override fields), buffers and strings owned by the
library, integer return codes (`FSIM_OK` = 0, negative = error, text in
`fsim_last_error()`), no exceptions, no C++ types. Struct layouts only grow
within a major ABI version (`fsim_abi_version()`).

`examples/rust_trainer` is a complete Rust client (no bindgen): `src/ffi.rs` mirrors the structs, `src/main.rs` runs the object model and a VecEnv batch.

## World and vehicles

```c
fsim_world_options wo; fsim_world_options_init(&wo);
wo.name = "rust-trainer";
wo.terrain = 1;                       /* optional: real relief; wo.record_path = "run.fsrec" records for --replay */
fsim_world* world; if (fsim_world_create(&wo, &world) != FSIM_OK) { puts(fsim_last_error()); return 1; }

fsim_vehicle_spec spec; fsim_vehicle_spec_init(&spec);
spec.name = "red-1"; spec.type = "jsbsim:f16"; spec.altitude_msl_m = 3000; spec.airspeed_ms = 200;
uint32_t red; fsim_world_create_vehicle(world, &spec, &red);

fsim_attitude_command att = {0.3, 0.05, fsim_hold(), 0.785, fsim_hold(), 200.0};
fsim_vehicle_command_attitude(world, red, &att);

for (int k = 0; k < 3000; ++k) {
    fsim_world_step(world, 1);
    const fsim_vehicle_state* s = fsim_vehicle_state_ptr(world, red);   /* valid until removal, updated per step */
    /* s->altitude_msl_m, s->euler_rad[0..2], s->velocity_ned_ms[...] ... same layout as fsim::VehicleState */
}
fsim_world_destroy(world);
```

| Group | Functions |
| --- | --- |
| Lifecycle | `fsim_world_create/destroy/step/time/step_seconds/vehicle_steps/published` |
| VecEnv world | `fsim_vecenv_world(env)`: the batch's world as a world handle (owned by the environment; vehicles `env<e>/<v>`) |
| Vision | `fsim_vision_c.h` (`fsim_vision.dll`): `fsim_vision_create(world, options, &v)` / `destroy`, `fsim_vision_add_camera(v, vehicle_id, spec, &cam)`, `fsim_vision_render`, `fsim_vision_image` / `fsim_vision_depth` / `fsim_vision_segmentation` (library-owned buffers, valid until the next render), `fsim_vision_segmentation_id(v, vehicle_id)`, `fsim_vision_save_png` / `fsim_vision_save_segmentation_png`, `fsim_vision_settle`; `fsim_vision_batch_create(env, spec, options, &b)` / `render` / `rgb` / `depth` / `segmentation` for one camera per batch vehicle as tensors; `fsim_camera_spec` and `fsim_vision_options` mirror [vision.md](vision.md) |
| Recordings | `fsim_recording_load/destroy`, `fsim_recording_frame_count/frame_time`, `fsim_recording_samples(r, frame, &n)` (`fsim_recorded_sample{slot, state, inputs}`), `fsim_recording_events`, `world_name/dt/frame_skip` ([world.md](world.md), recordings) |
| Scenarios | `fsim_scenario_load/parse/destroy`, `fsim_scenario_world_options` (fills an `fsim_world_options`; strings live with the scenario), `fsim_scenario_vehicle_count`, `fsim_scenario_apply(world, scenario, ids, capacity, &count)` ([scenarios.md](scenarios.md)) |
| Vehicles | `fsim_world_create_vehicle/remove_vehicle/reset_vehicle/find_vehicle/vehicle_count/vehicle_ids`, `fsim_vehicle_name/type` |
| State | `fsim_vehicle_state_ptr`, `fsim_vehicle_sensed_ptr` (`fsim_vehicle_state`, layout-checked against the C++ struct), `fsim_vehicle_get_property/set_property` |
| Control | `fsim_vehicle_command_actuator/attitude/acceleration/velocity/position/behavior`, `fsim_vehicle_active_level`, `fsim_vehicle_behavior_finished`, `fsim_vehicle_use_controller`, `fsim_vehicle_set_controller_parameter`, `fsim_vehicle_controller_parameter` (1.3: the value a vehicle flies with - the default, the aircraft's own gain, or the last one set); `fsim_hold()` for optional fields |
| Capabilities and activities (1.4) | [control.md](control.md#capabilities-and-activities). `fsim_vehicle_capability_count/capability/capability_parameter` describe what a vehicle offers (`fsim_capability_info`, `fsim_parameter_info`; strings live with the world), `fsim_vehicle_capability_status`. `fsim_vehicle_submit(world, id, level, fields, n, options, &result)`, `fsim_vehicle_submit_behavior` and `fsim_vehicle_submit_support(world, id, FSIM_SUPPORT_GEAR/FLAPS/WHEEL_BRAKES/SPEEDBRAKE/PITCH_TRIM/ENGINES, fields, n, ...)` (NEW; `ENGINES` is a throttle per engine, 1 to 4 fields, owning thrust), `fsim_activity_update(world, activity, fields, n, &result)` (UPDATE, the per-step path) and `fsim_activity_update_batch(world, activities, n, values, stride)`, `fsim_activity_cancel` (CANCEL). Each answers in its `fsim_command_result` (status, reason, activity, other, clamped) and returns `FSIM_OK`; a non-OK code means the call itself was malformed. `fsim_activity_get`, `fsim_vehicle_activity_count/activity` (`fsim_activity_info`), `fsim_reason_name`, `fsim_activity_state_name`; the vehicle's profile by path, `fsim_vehicle_profile_value(world, id, "envelope/clean/n_max", &v)`, and `fsim_vehicle_profile_section` (version, provenance); `fsim_command_options_init` first (a policy's command, values clamped). `fsim_command_options.axes` owns part of the vehicle: bits roll 1, pitch 2, yaw 4, thrust 8, ... (`fsim_capability_info.axis_groups` says what a capability may own apart: 1 roll and yaw, 2 pitch, 4 thrust, 8 any primary axis). `fsim_vehicle_set_default(world, id, FSIM_DEFAULT_NEUTRAL or FSIM_DEFAULT_HOLD, &reason)` and `fsim_vehicle_get_default` choose what flies the axes nobody owns. Envelope protection ([control.md](control.md#envelope-protection)): `fsim_vehicle_set_protection(world, id, FSIM_PROTECTION_OFF/REPORT/LIMIT)`, `fsim_vehicle_get_protection`, and `fsim_vehicle_envelope(world, id, &status)` - per limit (`fsim_limit_name(i)`), the updates limited, the updates and seconds beyond it and the worst excess since the last call; `fsim_vecenv_set_action_ranges(env, "aircraft")` for a batch's actions in the aircraft's own ranges. Since 1.4 the `fsim_vehicle_command_*` calls refuse what the vehicle refuses: a behaviour nobody registered (it used to be ignored and reported as `FSIM_OK`), an axis an autopilot or override activity holds - `FSIM_INVALID_ARGUMENT` with the reason in `fsim_last_error()` |
| Rotorcraft (1.5) | [control.md](control.md#rotorcraft). The capability calls take a level's rotorcraft fields after its others: `fsim_command_field_count_full(level)` values (acceleration 6: ... `pitch_rate_rad_s`, `yaw_rate_rad_s`; velocity 6: ... `north_ms`, `east_ms`; position 6: ... `heading_rad`) or `fsim_command_field_count(level)` as before, the rest `fsim_hold()`; `fsim_activity_update_batch_n(world, activities, n, values, stride, fields)` sends rows of either width. `fsim_parameter_info.unsupported` (was `reserved`, 0) is 1 for a field the aircraft has nothing for, which a command may not set. `fsim_capability_info.axis_groups` adds 16 (the cyclic: roll with pitch) and 32 (yaw), a rotorcraft's; `FSIM_SUPPORT_ENGINES` on a multirotor sets each motor's thrust and owns all four axes. The frozen `fsim_*_command` structs do not change |
| Vehicle Interface (1.6) | [control.md](control.md#the-vehicle-interface), [ADR-28](../vehicle-interface.md). `fsim_command_result.reserved` is the index of the field, route point or curve segment an answer was about, plus one (0 none); `fsim_last_command_detail(world, &detail)` (`fsim_command_detail_init` first) gives the world's last answer's reason, index, the performance limit its value broke (`fsim_constraint_name`) and a curve's section. `fsim_activity_get_progress(world, activity, &progress)` (`fsim_activity_progress`: segment of segments, laps, percent, distance and time to go, cross-track, and the course, heading, altitude and speed the activity commands). `fsim_vehicle_commanded(world, id, &state)` (`fsim_commanded_state`: what the cascade asked for in its last update, level by level). `fsim_vehicle_submit_mode(world, id, FSIM_MODE_HSA, fields, 6, &options, &result)` commands the HSA/CSA mode: heading_rad or course_rad, speed, speed_reference (`FSIM_SPEED_TRUE_AIRSPEED`, `_CALIBRATED_AIRSPEED`, `_GROUND_SPEED`, `_MACH`), altitude_m, altitude_reference (`FSIM_ALTITUDE_MSL`, `_ABOVE_GROUND`, `_ELLIPSOID`), `fsim_hold()` leaving one out; `fsim_activity_update` takes the same six fields and keeps the ones left out (`fsim_mode_field_count`). `fsim_vehicle_submit_route(world, id, fields, 4, waypoints, n, &options, &result)` commands a route (waypoint following): its fields projection (`FSIM_PROJECTION_GREAT_CIRCLE`, `_RHUMB`), repeat, end (`FSIM_END_CONTINUE`, `_LOITER`) and start, and at most 256 `fsim_waypoint`s (`fsim_waypoint_init` leaves every optional field out; `turn` `FSIM_TURN_FLY_BY` or `_FLY_OVER`), read `waypoints[0].struct_size` apart so an older header's array works; `fsim_activity_update_route` replaces a route's waypoints (none: it keeps them) and the options given, and `fsim_activity_update` with a route's four fields its options alone - either flies it afresh from its start. `fsim_vehicle_submit_mode(world, id, FSIM_MODE_PATTERN, fields, 12, &options, &result)` commands a loiter pattern: pattern (`FSIM_PATTERN_ORBIT`, `_RACETRACK`, `_FIGURE_EIGHT`, `_HOLD`), latitude_rad, longitude_rad, altitude_m, altitude_reference, radius_m, clockwise, course_rad, leg_m, speed, speed_reference, duration_s; `fsim_hold()` takes a default in a NEW and keeps a value in an UPDATE. `fsim_vehicle_submit_curve(world, id, fields, 8, segments, n, &options, &result)` commands curve following: its fields latitude_rad, longitude_rad, altitude_m (the reference; `fsim_hold()`: the aircraft), speed_min_ms, speed_max_ms (ground speeds), duration_s, end and append, and 1 to 10 `fsim_bezier_segment`s (six control points north, east and down, metres from the reference; `fsim_bezier_segment_init`), read `segments[0].struct_size` apart; a segment refused is named in `reserved`, a section too tight in the detail's `from` and `to`. `fsim_activity_update_curve` appends segments (append 1) or replaces the curve, and `fsim_activity_update` with its eight fields changes its options alone. `fsim_vehicle_capability_flight_mode(world, id, capability)` is the A-GRA type a capability is (`FSIM_FLIGHT_MODE_HSA_CSA`, `_WAYPOINT_FOLLOWING`, `_CURVE_FOLLOWING`, `_LOITER`, `_FORMATION`, ...; `fsim_flight_mode_name`). These structs are filled up to the `struct_size` their `_init` sets, so a caller built against an older header keeps working |
| Batched (1.2) | `fsim_world_gather_states(world, ids, n, sensed, out)` copies n states in one call; `fsim_world_command_batch(world, level, ids, n, values, stride)` commands n vehicles at one level from rows of doubles in the field order of the level's `fsim_*_command` (`fsim_command_field_count(level)`: 8, 6, 4, 4, 5). For bindings, whose cost is per call: a step then costs the same number of calls however many vehicles fly |
| Environment | `fsim_world_get_environment/set_environment` (`fsim_environment`) |
| Effects | `fsim_vehicle_add_effect(world, id or 0 for all, "gaussian_sensor_noise" ..., names, values, n)`, `fsim_vehicle_clear_effects` |
| Communication | `fsim_comm_create_node/send/inbox_count/inbox_get/set_medium/attach_protocol`, `fsim_comm_attach_udp_bridge(world, node, local_port, remote_host, remote_port)` |
| Batch layer | `fsim_options_init`, `fsim_vecenv_create/destroy/reset/step/buffers`, `fsim_vecenv_observation_name/action_name/vehicle_steps`; `fsim_vecenv_set_autoreset(env, FSIM_AUTORESET_NEXT_STEP` (Gymnasium) `| FSIM_AUTORESET_SAME_STEP` (Stable-Baselines3)`)`; `fsim_vecenv_vehicle_ids` (world ids in batch order); `fsim_registered_id(FSIM_REGISTRY_TASK/OBSERVATION/ACTION, i)` |

Behaviour parameters and effect parameters are passed as parallel name/value
arrays (`const char* const* names, const double* values, uint32_t count`),
routes as an array of `fsim_position_command`.

`tests/test_c_abi.c` exercises the whole surface from a C99 translation unit
and doubles as a reference.
