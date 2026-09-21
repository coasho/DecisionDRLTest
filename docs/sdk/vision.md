# Vision: cameras on vehicles

`#include <fsim/Vision.h>` - link `fsim::vision` (`fsim_vision.dll`) as well as `fsim::sdk`.

Agents that learn from pixels get cameras mounted on their vehicles,
rendered offscreen with the same Earth (satellite imagery over real
relief), sky, sun and vehicle models the viewer draws. The core SDK stays
headless; the vision library needs a Vulkan device but no window, so it
runs on a training box without a display attached (it is a separate DLL for
exactly that reason).

```cpp
fsim::World world(options);
fsim::vision::Sensors sensors(world);                  // Vulkan device + scene, no window

fsim::vision::CameraSpec nose;                         // body frame: x forward, y right, z down
nose.width = 128; nose.height = 128; nose.fovDeg = 70; // vertical field of view
nose.offsetBodyM[0] = 2.0;                             // 2 m ahead of the body origin
const unsigned cam = sensors.addCamera(vehicle, nose); // before the first render()

for (;;) {
    world.step();
    sensors.render();                                  // every camera, one GPU submission, blocks until read back
    fsim::vision::Image img = sensors.image(cam);      // img.rgb: height x width x 3 bytes, top row first
}
```

| Element | Meaning |
| --- | --- |
| `CameraSpec` | `width`, `height`, `fovDeg`, mount `offsetBodyM[3]` and attitude `yawDeg` (right), `pitchDeg` (up), `rollDeg` (right) relative to the body; `hideOwnVehicle` (default true) keeps the carrier's own model out of its cameras; `depth` also delivers metres per pixel |
| `Options` | `earth` / `imagery` / `elevation` (and URL templates, `maxLevel`), `sky` (dome + sun from the world's clock), `maxVehicles` drawn, `assetDir` for `models/<type>.glb`, `debugLayer` |
| `addCamera(vehicle, spec)` / `removeCamera(i)` | at any time; a change recompiles the command graph at the next `render()` (a few ms) |
| `render()` | draws every camera from the world's current state; explicit, so a trainer pays for images only when it wants them (e.g. every agent step, not every FDM step) |
| `image(i)` | the last frame of camera `i`, valid until the next `render()` |
| `depth(i)` | `DepthImage`: float metres along the view axis per pixel (cameras with `depth`); the sky reads as the far plane (hundreds of km) |
| `saveDepthPng(i, path, farM)` | log-scaled 8-bit depth for a look |
| `savePng(i, path)` | debugging and datasets |
| `timing()` | where the last render went: bookkeeping, tile merges, recording, GPU, host copy |
| `settle(n)` | run `n` frames without reading back, to let tiles stream in after a jump to a new region |

Cameras of a removed vehicle keep rendering from its last pose; up to 31
vehicles can hide themselves from their own cameras. Several `Sensors` in
one process share one Vulkan device.

## With VecEnv

`BatchCameras` mounts one camera per batch vehicle (batch order, index =
`env * K + vehicle`) and packs the images as tensors - the pixel observation
next to `StepResult::observations`:

```cpp
fsim::VecEnv env(opt);
fsim::vision::BatchCameras cams(env, nose);          // nose.depth = true for a depth tensor too
for (;;) {
    auto r = env.step(actions);
    cams.render();
    learner.see(cams.rgb(), cams.depth());           // [N][H][W][3] bytes, [N][H][W] metres
}
```

`ppo_trainer --depth 8x6` is this loop: the state observation plus 48
log-depth pixels from a forward camera per vehicle, through the same PPO.

`VecEnv::world()` is the same world through the object model, so cameras
can also be placed by hand (`Sensors`, `env.world().vehicles()`, vehicles
named `env<e>/<v>`); `cams.sensors()` is the underlying `Sensors` for
`savePng`, timing or extra cameras. In C: `fsim_vision_batch_create(env, &spec,
&options, &batch)`, `fsim_vision_batch_render`, `fsim_vision_batch_rgb/depth`.

## From C

`fsim_vision_c.h` mirrors the API for other languages: `fsim_vision_create`
on any world handle (including `fsim_vecenv_world(env)`),
`fsim_vision_add_camera(vision, vehicle_id, &spec, &cam)`,
`fsim_vision_render`, `fsim_vision_image` / `fsim_vision_depth` returning
library-owned buffers ([c_abi.md](c_abi.md)).

## Performance

One `render()` records all cameras into one command buffer, submits once,
waits, and copies the images back through cached host memory. Measured on an
RTX 5070 Ti (`examples/vision_capture --fleet 60`): 64 cameras (60 x 128x128
+ 4 x 320x240) in ~8 ms per step, ~1 ms for four 320x240 cameras - the design
budget (8.4) is met. The first frames at a new place show coarse tiles while
the pager streams the pyramid; run `tile_prefetch` for the training region
so nothing downloads during episodes and the cache is warm.

## Example

`examples/vision_capture` flies three aircraft over Yosemite with nose,
chase, downward and side cameras on the lead and writes PNGs:

```bash
build/ucrt64-release/bin/vision_capture.exe --seconds 20 --every 2 --out captures
```

## Limits

- RGB and depth; segmentation (per-vehicle ids) is next.
- Whether the vehicle's own model occludes the view depends on the mount: a
  camera 2 m ahead of the origin sits inside the c172's cowling, which is why
  `hideOwnVehicle` defaults to on.
