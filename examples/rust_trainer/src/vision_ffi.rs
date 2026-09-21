//! `include/fsim/fsim_vision_c.h`: cameras on vehicles (feature `vision`, links fsim_vision.dll).
#![allow(non_camel_case_types, dead_code)]

use crate::ffi::fsim_world;
use std::os::raw::{c_char, c_double, c_int, c_void};

pub type fsim_vision = c_void;

#[repr(C)]
pub struct fsim_vision_options {
    pub struct_size: u32,
    pub earth: i32,
    pub imagery: i32,
    pub elevation: i32,
    pub imagery_url: *const c_char,
    pub elevation_url: *const c_char,
    pub max_level: u32,
    pub sky: i32,
    pub max_vehicles: u32,
    pub asset_dir: *const c_char,
    pub debug_layer: i32,
}

#[repr(C)]
pub struct fsim_camera_spec {
    pub struct_size: u32,
    pub width: u32,
    pub height: u32,
    pub fov_deg: c_double,
    pub offset_body_m: [c_double; 3],
    pub yaw_deg: c_double,
    pub pitch_deg: c_double,
    pub roll_deg: c_double,
    pub hide_own_vehicle: i32,
    pub depth: i32,
}

extern "C" {
    pub fn fsim_vision_options_init(options: *mut fsim_vision_options);
    pub fn fsim_camera_spec_init(spec: *mut fsim_camera_spec);
    pub fn fsim_vision_create(world: *mut fsim_world, options: *const fsim_vision_options, out: *mut *mut fsim_vision) -> c_int;
    pub fn fsim_vision_destroy(vision: *mut fsim_vision);
    pub fn fsim_vision_add_camera(vision: *mut fsim_vision, vehicle_id: u32, spec: *const fsim_camera_spec, out_camera: *mut u32) -> c_int;
    pub fn fsim_vision_remove_camera(vision: *mut fsim_vision, camera: u32);
    pub fn fsim_vision_camera_count(vision: *const fsim_vision) -> u32;
    pub fn fsim_vision_render(vision: *mut fsim_vision) -> c_int;
    pub fn fsim_vision_image(vision: *const fsim_vision, camera: u32, width: *mut u32, height: *mut u32) -> *const u8;
    pub fn fsim_vision_depth(vision: *const fsim_vision, camera: u32, width: *mut u32, height: *mut u32) -> *const f32;
    pub fn fsim_vision_save_png(vision: *const fsim_vision, camera: u32, path: *const c_char) -> c_int;
    pub fn fsim_vision_settle(vision: *mut fsim_vision, frames: u32) -> c_int;
    pub fn fsim_vision_last_render_ms(vision: *const fsim_vision) -> c_double;
}
