//! Hand-written declarations for the part of `include/fsim/fsim_c.h` this
//! example uses. Layouts mirror the C structs field for field (the header is
//! the contract; it only grows within a major ABI version, and every struct
//! starts with `struct_size`, filled by its `_init` function).
#![allow(non_camel_case_types, dead_code)]

use std::os::raw::{c_char, c_double, c_int, c_void};

pub const FSIM_OK: c_int = 0;

pub type fsim_vecenv = c_void;
pub type fsim_world = c_void;

#[repr(C)]
pub struct fsim_options {
    pub struct_size: u32,
    pub num_envs: u32,
    pub vehicles_per_env: u32,
    pub workers: u32,
    pub seed: u64,
    pub aircraft: *const c_char,
    pub jsbsim_root: *const c_char,
    pub task: *const c_char,
    pub observation: *const c_char,
    pub action: *const c_char,
    pub dt: c_double,
    pub frame_skip: i32,
    pub max_episode_steps: u32,
    pub latitude_deg: c_double,
    pub longitude_deg: c_double,
    pub altitude_m: c_double,
    pub heading_deg: c_double,
    pub airspeed_ms: c_double,
    pub latitude_jitter_deg: c_double,
    pub longitude_jitter_deg: c_double,
    pub altitude_jitter_m: c_double,
    pub heading_jitter_deg: c_double,
    pub airspeed_jitter_ms: c_double,
    pub target_altitude_delta_m: c_double,
    pub target_heading_delta_deg: c_double,
    pub world_name: *const c_char,
    pub publish: i32,
    pub terrain: i32,
    pub scenario_path: *const c_char,
}

#[repr(C)]
pub struct fsim_buffers {
    pub struct_size: u32,
    pub num_envs: u32,
    pub vehicles_per_env: u32,
    pub observation_size: u32,
    pub action_size: u32,
    pub observations: *const f32,
    pub rewards: *const f32,
    pub terminated: *const u8,
    pub truncated: *const u8,
    pub final_observations: *const f32,
    pub episode_steps: *const u32,
    pub agent_step_seconds: c_double,
}

#[repr(C)]
pub struct fsim_world_options {
    pub struct_size: u32,
    pub name: *const c_char,
    pub dt: c_double,
    pub frame_skip: i32,
    pub workers: u32,
    pub pin_workers: i32,
    pub seed: u64,
    pub capacity: u32,
    pub publish: i32,
    pub publish_interval_s: c_double,
    pub jsbsim_root: *const c_char,
    pub terrain: i32,
    pub terrain_url: *const c_char,
    pub terrain_zoom: u32,
    pub record_path: *const c_char,
    pub record_interval_s: c_double,
}

#[repr(C)]
pub struct fsim_vehicle_spec {
    pub struct_size: u32,
    pub name: *const c_char,
    pub r#type: *const c_char,
    pub latitude_deg: c_double,
    pub longitude_deg: c_double,
    pub altitude_msl_m: c_double,
    pub heading_deg: c_double,
    pub pitch_deg: c_double,
    pub roll_deg: c_double,
    pub airspeed_ms: c_double,
    pub on_ground: i32,
    pub model: *const c_char,
    pub control_divider: u32,
}

pub const FSIM_MAX_ENGINES: usize = 4;

#[repr(C)]
pub struct fsim_vehicle_state {
    pub sim_time: c_double,
    pub position_ecef: [c_double; 3],
    pub attitude_ecef_to_body: [c_double; 4],
    pub latitude_rad: c_double,
    pub longitude_rad: c_double,
    pub altitude_msl_m: c_double,
    pub altitude_agl_m: c_double,
    pub euler_rad: [c_double; 3],
    pub velocity_body_ms: [c_double; 3],
    pub velocity_ned_ms: [c_double; 3],
    pub angular_rate_body_rad_s: [c_double; 3],
    pub acceleration_body_ms2: [c_double; 3],
    pub airspeed_true_ms: c_double,
    pub airspeed_calibrated_ms: c_double,
    pub mach: c_double,
    pub alpha_rad: c_double,
    pub beta_rad: c_double,
    pub load_factor: c_double,
    pub aileron_rad: c_double,
    pub elevator_rad: c_double,
    pub rudder_rad: c_double,
    pub flaps_rad: c_double,
    pub gear_position: c_double,
    pub engine_count: i32,
    pub throttle_position: [c_double; FSIM_MAX_ENGINES],
    pub thrust_n: [c_double; FSIM_MAX_ENGINES],
    pub fuel_kg: c_double,
    pub step_count: u32,
    pub on_ground: u8,
    pub diverged: u8,
    pub rotation_body_to_ecef: [c_double; 9],
}

#[repr(C)]
pub struct fsim_environment {
    pub struct_size: u32,
    pub epoch_utc_seconds: c_double,
    pub time_factor: c_double,
    pub temperature_sl_k: c_double,
    pub pressure_sl_pa: c_double,
    pub humidity: c_double,
    pub wind_direction_deg: c_double,
    pub wind_speed_ms: c_double,
    pub wind_gust_ms: c_double,
    pub turbulence: c_double,
    pub visibility_m: c_double,
    pub cloud_base_m: c_double,
    pub cloud_cover: c_double,
    pub precipitation: c_double,
}

#[repr(C)]
pub struct fsim_attitude_command {
    pub roll_rad: c_double,
    pub pitch_rad: c_double,
    pub heading_rad: c_double,
    pub max_bank_rad: c_double,
    pub throttle: c_double,
    pub airspeed_ms: c_double,
}

#[repr(C)]
pub struct fsim_velocity_command {
    pub airspeed_ms: c_double,
    pub vertical_speed_ms: c_double,
    pub heading_rad: c_double,
    pub turn_rate_rad_s: c_double,
}

extern "C" {
    pub fn fsim_abi_version() -> u32;
    pub fn fsim_version() -> *const c_char;
    pub fn fsim_last_error() -> *const c_char;
    pub fn fsim_hold() -> c_double;

    // VecEnv batch layer
    pub fn fsim_options_init(options: *mut fsim_options);
    pub fn fsim_vecenv_create(options: *const fsim_options, out: *mut *mut fsim_vecenv) -> c_int;
    pub fn fsim_vecenv_destroy(env: *mut fsim_vecenv);
    pub fn fsim_vecenv_reset(env: *mut fsim_vecenv, seed: u64) -> c_int;
    pub fn fsim_vecenv_step(env: *mut fsim_vecenv, actions: *const f32, count: usize) -> c_int;
    pub fn fsim_vecenv_buffers(env: *const fsim_vecenv, out: *mut fsim_buffers) -> c_int;
    pub fn fsim_vecenv_observation_name(env: *const fsim_vecenv, index: u32) -> *const c_char;
    pub fn fsim_vecenv_vehicle_steps(env: *const fsim_vecenv) -> u64;
    pub fn fsim_vecenv_world(env: *mut fsim_vecenv) -> *mut fsim_world;

    // World / vehicles
    pub fn fsim_world_options_init(options: *mut fsim_world_options);
    pub fn fsim_vehicle_spec_init(spec: *mut fsim_vehicle_spec);
    pub fn fsim_world_create(options: *const fsim_world_options, out: *mut *mut fsim_world) -> c_int;
    pub fn fsim_world_destroy(world: *mut fsim_world);
    pub fn fsim_world_step(world: *mut fsim_world, steps: u32) -> c_int;
    pub fn fsim_world_time(world: *const fsim_world) -> c_double;
    pub fn fsim_world_step_seconds(world: *const fsim_world) -> c_double;
    pub fn fsim_world_create_vehicle(world: *mut fsim_world, spec: *const fsim_vehicle_spec, id: *mut u32) -> c_int;
    pub fn fsim_world_vehicle_count(world: *const fsim_world) -> u32;
    pub fn fsim_vehicle_state_ptr(world: *const fsim_world, id: u32) -> *const fsim_vehicle_state;
    pub fn fsim_vehicle_command_attitude(world: *mut fsim_world, id: u32, command: *const fsim_attitude_command) -> c_int;
    pub fn fsim_vehicle_command_velocity(world: *mut fsim_world, id: u32, command: *const fsim_velocity_command) -> c_int;
    pub fn fsim_world_get_environment(world: *const fsim_world, out: *mut fsim_environment) -> c_int;
    pub fn fsim_world_set_environment(world: *mut fsim_world, environment: *const fsim_environment) -> c_int;
}

/// The library's last error as a String.
pub fn last_error() -> String {
    unsafe {
        let p = fsim_last_error();
        if p.is_null() {
            String::new()
        } else {
            std::ffi::CStr::from_ptr(p).to_string_lossy().into_owned()
        }
    }
}
