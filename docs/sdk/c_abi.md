# C ABI

`#include <fsim/fsim_c.h>` - the binding surface for any language with a C
FFI (Rust `bindgen`, C#, Julia, Go, ...). The platform ships no bindings.

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
| Control | `fsim_vehicle_command_actuator/attitude/acceleration/velocity/position/behavior`, `fsim_vehicle_active_level`, `fsim_vehicle_behavior_finished`, `fsim_vehicle_use_controller`, `fsim_vehicle_set_controller_parameter`; `fsim_hold()` for optional fields |
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
