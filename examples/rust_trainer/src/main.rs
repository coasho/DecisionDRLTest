//! fsim from Rust, through the C ABI and nothing else (no bindgen, no crates).
//!
//! Two demonstrations:
//!   `rust_trainer world`   - the object model: a world, two aircraft commanded at
//!                            the attitude and velocity levels, wind, state readout.
//!                            Start flightsim-viewer.exe at any time to watch.
//!   `rust_trainer vecenv`  - the batch layer: 32 environments stepped with the
//!                            same PD baseline as examples/minimal_trainer.
//!   `rust_trainer camera`  - (feature `vision`) a nose camera with depth on a
//!                            vehicle over Yosemite, images saved as PNG.
//!
//! Build: `cargo run --release -- world` from this directory after the CMake
//! build (fsim.dll and JSBSim.dll are copied next to the executable; set
//! FSIM_BUILD_DIR for another build tree). Toolchain: the MinGW-built DLLs
//! work with the `stable-x86_64-pc-windows-gnu` target.

mod ffi;
#[cfg(feature = "vision")]
mod vision_ffi;

use std::ffi::{CStr, CString};
use std::mem::MaybeUninit;

/// Safe wrapper over a world handle for this example.
struct World(*mut ffi::fsim_world);

impl World {
    fn new(name: &str) -> Result<World, String> {
        let name = CString::new(name).unwrap();
        unsafe {
            let mut o = MaybeUninit::<ffi::fsim_world_options>::uninit();
            ffi::fsim_world_options_init(o.as_mut_ptr());
            let mut o = o.assume_init();
            o.name = name.as_ptr();
            let mut handle = std::ptr::null_mut();
            if ffi::fsim_world_create(&o, &mut handle) != ffi::FSIM_OK {
                return Err(ffi::last_error());
            }
            Ok(World(handle))
        }
    }

    fn create_vehicle(&mut self, name: &str, lat: f64, lon: f64, alt: f64, heading: f64, tas: f64) -> Result<u32, String> {
        let name = CString::new(name).unwrap();
        let kind = CString::new("jsbsim:c172x").unwrap();
        unsafe {
            let mut s = MaybeUninit::<ffi::fsim_vehicle_spec>::uninit();
            ffi::fsim_vehicle_spec_init(s.as_mut_ptr());
            let mut s = s.assume_init();
            s.name = name.as_ptr();
            s.r#type = kind.as_ptr();
            s.latitude_deg = lat;
            s.longitude_deg = lon;
            s.altitude_msl_m = alt;
            s.heading_deg = heading;
            s.airspeed_ms = tas;
            let mut id = 0u32;
            if ffi::fsim_world_create_vehicle(self.0, &s, &mut id) != ffi::FSIM_OK {
                return Err(ffi::last_error());
            }
            Ok(id)
        }
    }

    fn step(&mut self, n: u32) {
        unsafe {
            ffi::fsim_world_step(self.0, n);
        }
    }

    fn time(&self) -> f64 {
        unsafe { ffi::fsim_world_time(self.0) }
    }

    fn state(&self, id: u32) -> Option<&ffi::fsim_vehicle_state> {
        unsafe { ffi::fsim_vehicle_state_ptr(self.0, id).as_ref() }
    }

    #[allow(dead_code)] // used by the `vision` feature
    fn set_time_utc(&mut self, unix_seconds: f64) {
        unsafe {
            let mut e = MaybeUninit::<ffi::fsim_environment>::zeroed().assume_init();
            e.struct_size = std::mem::size_of::<ffi::fsim_environment>() as u32;
            ffi::fsim_world_get_environment(self.0, &mut e);
            e.epoch_utc_seconds = unix_seconds - self.time();
            ffi::fsim_world_set_environment(self.0, &e);
        }
    }

    fn set_wind(&mut self, from_deg: f64, speed_ms: f64) {
        unsafe {
            let mut e = MaybeUninit::<ffi::fsim_environment>::zeroed().assume_init();
            e.struct_size = std::mem::size_of::<ffi::fsim_environment>() as u32;
            ffi::fsim_world_get_environment(self.0, &mut e);
            e.wind_direction_deg = from_deg;
            e.wind_speed_ms = speed_ms;
            ffi::fsim_world_set_environment(self.0, &e);
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe { ffi::fsim_world_destroy(self.0) }
    }
}

fn version() -> String {
    unsafe { CStr::from_ptr(ffi::fsim_version()).to_string_lossy().into_owned() }
}

fn demo_world(seconds: f64) -> Result<(), String> {
    let mut world = World::new("rust")?;
    let hold = unsafe { ffi::fsim_hold() };
    let leader = world.create_vehicle("leader", 37.62, -122.40, 1500.0, 90.0, 60.0)?;
    let wing = world.create_vehicle("wing", 37.615, -122.405, 1500.0, 90.0, 60.0)?;
    world.set_wind(270.0, 6.0);

    // Attitude level: 15 degrees of bank, a little nose-up, hold the airspeed.
    let att = ffi::fsim_attitude_command { roll_rad: 0.26, pitch_rad: 0.03, heading_rad: hold, max_bank_rad: 0.8, throttle: hold, airspeed_ms: 60.0 };
    // Velocity level: climb at 2 m/s towards heading 045 at 65 m/s.
    let vel = ffi::fsim_velocity_command { airspeed_ms: 65.0, vertical_speed_ms: 2.0, heading_rad: 45.0_f64.to_radians(), turn_rate_rad_s: hold };
    unsafe {
        if ffi::fsim_vehicle_command_attitude(world.0, leader, &att) != ffi::FSIM_OK {
            return Err(ffi::last_error());
        }
        if ffi::fsim_vehicle_command_velocity(world.0, wing, &vel) != ffi::FSIM_OK {
            return Err(ffi::last_error());
        }
    }
    let step_s = unsafe { ffi::fsim_world_step_seconds(world.0) };
    let steps = (seconds / step_s) as u32;
    let report_every = (5.0 / step_s) as u32;
    for k in 1..=steps {
        world.step(1);
        if k % report_every == 0 {
            println!("--- t = {:5.1} s", world.time());
            for (name, id) in [("leader", leader), ("wing", wing)] {
                let s = world.state(id).ok_or("vehicle gone")?;
                println!(
                    "  {:7} alt {:7.1} m  hdg {:5.1}  tas {:5.1} m/s  roll {:6.1}  pitch {:5.1}  vz {:5.1}{}",
                    name,
                    s.altitude_msl_m,
                    s.euler_rad[2].to_degrees().rem_euclid(360.0),
                    s.airspeed_true_ms,
                    s.euler_rad[0].to_degrees(),
                    s.euler_rad[1].to_degrees(),
                    -s.velocity_ned_ms[2],
                    if s.diverged != 0 { "  DIVERGED" } else { "" }
                );
            }
        }
    }
    Ok(())
}

/// The observation channel index by name (names are stable, resolve once).
fn channel(env: *const ffi::fsim_vecenv, name: &str, count: u32) -> usize {
    for i in 0..count {
        let n = unsafe { CStr::from_ptr(ffi::fsim_vecenv_observation_name(env, i)) };
        if n.to_bytes() == name.as_bytes() {
            return i as usize;
        }
    }
    panic!("no observation channel {name}");
}

fn demo_vecenv(steps: usize) -> Result<(), String> {
    unsafe {
        let mut o = MaybeUninit::<ffi::fsim_options>::uninit();
        ffi::fsim_options_init(o.as_mut_ptr());
        let mut o = o.assume_init();
        let world_name = CString::new("rust-vecenv").unwrap();
        o.num_envs = 32;
        o.max_episode_steps = 1000;
        o.world_name = world_name.as_ptr();
        let mut env = std::ptr::null_mut();
        if ffi::fsim_vecenv_create(&o, &mut env) != ffi::FSIM_OK {
            return Err(ffi::last_error());
        }
        let mut b = MaybeUninit::<ffi::fsim_buffers>::zeroed().assume_init();
        b.struct_size = std::mem::size_of::<ffi::fsim_buffers>() as u32;
        ffi::fsim_vecenv_reset(env, 0);
        ffi::fsim_vecenv_buffers(env, &mut b);
        let (n, obs_size, act_size) = ((b.num_envs * b.vehicles_per_env) as usize, b.observation_size as usize, b.action_size as usize);
        let ch = |name: &str| channel(env, name, b.observation_size);
        let (alt_err, hdg_sin, hdg_cos, roll, pitch, p, q, r, vz) = (
            ch("alt_err_km"), ch("hdg_err_sin"), ch("hdg_err_cos"), ch("roll"), ch("pitch"), ch("p"), ch("q"), ch("r"), ch("vz_down_100ms"),
        );
        println!("fsim {}: {} vehicle(s), obs {}, act {} - PD baseline", version(), n, obs_size, act_size);

        let mut actions = vec![0.0f32; n * act_size];
        let mut returns = vec![0.0f64; n];
        let mut finished: Vec<f64> = Vec::new();
        let t0 = std::time::Instant::now();
        for step in 0..steps {
            let obs = std::slice::from_raw_parts(b.observations, n * obs_size);
            for i in 0..n {
                let o = &obs[i * obs_size..(i + 1) * obs_size];
                let alt_err_m = o[alt_err] * 1000.0;
                let hdg_err = o[hdg_sin].atan2(o[hdg_cos]);
                let roll_target = (hdg_err * 1.2).clamp(-0.6, 0.6);
                let pitch_target = (alt_err_m * 0.0015 - o[vz] * -0.02).clamp(-0.25, 0.25);
                let a = &mut actions[i * act_size..(i + 1) * act_size];
                a[0] = ((roll_target - o[roll]) * 1.5 - o[p] * 0.3).clamp(-1.0, 1.0);
                a[1] = ((pitch_target - o[pitch]) * -3.0 + o[q] * 0.5).clamp(-1.0, 1.0);
                a[2] = (-o[r] * 0.5).clamp(-1.0, 1.0);
                a[3] = (0.2 + alt_err_m * 0.002).clamp(-1.0, 1.0);
            }
            if ffi::fsim_vecenv_step(env, actions.as_ptr(), actions.len()) != ffi::FSIM_OK {
                return Err(ffi::last_error());
            }
            ffi::fsim_vecenv_buffers(env, &mut b);
            let rewards = std::slice::from_raw_parts(b.rewards, n);
            let terminated = std::slice::from_raw_parts(b.terminated, n);
            let truncated = std::slice::from_raw_parts(b.truncated, n);
            for i in 0..n {
                returns[i] += rewards[i] as f64;
                if terminated[i] != 0 || truncated[i] != 0 {
                    finished.push(returns[i]);
                    returns[i] = 0.0;
                }
            }
            if (step + 1) % 500 == 0 {
                let mean = if finished.is_empty() { 0.0 } else { finished.iter().sum::<f64>() / finished.len() as f64 };
                let obs = std::slice::from_raw_parts(b.observations, n * obs_size);
                println!(
                    "step {:5}  episodes {:4}  mean return {:8.2}  alt_err {:+7.1} m  hdg_err {:+5.1} deg",
                    step + 1,
                    finished.len(),
                    mean,
                    obs[alt_err] * 1000.0,
                    obs[hdg_sin].atan2(obs[hdg_cos]).to_degrees()
                );
            }
        }
        let wall = t0.elapsed().as_secs_f64();
        let vsteps = ffi::fsim_vecenv_vehicle_steps(env) as f64;
        println!("{wall:.2} s: {:.0} agent-steps/s, {:.2}M vehicle-steps/s", (steps * n) as f64 / wall, vsteps / wall / 1e6);
        ffi::fsim_vecenv_destroy(env);
    }
    Ok(())
}

#[cfg(feature = "vision")]
fn demo_camera(shots: u32) -> Result<(), String> {
    use std::ffi::CString;
    let mut world = World::new("rust-camera")?;
    world.set_time_utc(1782063000.0); // 2026-06-21T17:30:00Z: afternoon sun over Yosemite
    let lead = world.create_vehicle("lead", 37.72, -119.55, 3200.0, 90.0, 55.0)?;
    let hold = unsafe { ffi::fsim_hold() };
    let att = ffi::fsim_attitude_command { roll_rad: 0.0, pitch_rad: 0.03, heading_rad: hold, max_bank_rad: 0.8, throttle: hold, airspeed_ms: hold };
    unsafe {
        ffi::fsim_vehicle_command_attitude(world.0, lead, &att);
        let mut o = MaybeUninit::<vision_ffi::fsim_vision_options>::uninit();
        vision_ffi::fsim_vision_options_init(o.as_mut_ptr());
        let o = o.assume_init();
        let mut vision = std::ptr::null_mut();
        if vision_ffi::fsim_vision_create(world.0, &o, &mut vision) != ffi::FSIM_OK {
            return Err(ffi::last_error());
        }
        let mut c = MaybeUninit::<vision_ffi::fsim_camera_spec>::uninit();
        vision_ffi::fsim_camera_spec_init(c.as_mut_ptr());
        let mut c = c.assume_init();
        c.width = 320;
        c.height = 240;
        c.fov_deg = 70.0;
        c.offset_body_m[0] = 2.0;
        c.depth = 1;
        let mut cam = 0u32;
        if vision_ffi::fsim_vision_add_camera(vision, lead, &c, &mut cam) != ffi::FSIM_OK {
            return Err(ffi::last_error());
        }
        let step_s = ffi::fsim_world_step_seconds(world.0);
        let steps_per_shot = (2.0 / step_s) as u32;
        std::fs::create_dir_all("captures").map_err(|e| e.to_string())?;
        // Let the terrain tiles around the start stream in before the first image.
        vision_ffi::fsim_vision_render(vision);
        vision_ffi::fsim_vision_settle(vision, 60);
        for shot in 0..shots {
            world.step(steps_per_shot);
            if vision_ffi::fsim_vision_render(vision) != ffi::FSIM_OK {
                return Err(ffi::last_error());
            }
            let (mut w, mut h) = (0u32, 0u32);
            let rgb = vision_ffi::fsim_vision_image(vision, cam, &mut w, &mut h);
            let pixels = std::slice::from_raw_parts(rgb, (w * h * 3) as usize);
            let depth = vision_ffi::fsim_vision_depth(vision, cam, &mut w, &mut h);
            let centre_depth = *depth.add((h / 2 * w + w / 2) as usize);
            let mean: f64 = pixels.iter().map(|&p| p as f64).sum::<f64>() / pixels.len() as f64;
            let path = CString::new(format!("captures/rust_nose_{shot:03}.png")).unwrap();
            vision_ffi::fsim_vision_save_png(vision, cam, path.as_ptr());
            println!(
                "t={:5.1} s  {}x{}  mean intensity {:5.1}  depth at centre {:8.1} m  render {:.1} ms",
                world.time(),
                w,
                h,
                mean,
                centre_depth,
                vision_ffi::fsim_vision_last_render_ms(vision)
            );
        }
        vision_ffi::fsim_vision_destroy(vision);
    }
    Ok(())
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "world".to_string());
    println!("fsim {} (C ABI {}) from Rust", version(), unsafe { ffi::fsim_abi_version() });
    let result = match mode.as_str() {
        "world" => demo_world(30.0),
        "vecenv" => demo_vecenv(3000),
        #[cfg(feature = "vision")]
        "camera" => demo_camera(5),
        other => Err(format!("unknown mode '{other}' (world | vecenv | camera)")),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}
