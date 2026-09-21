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
| `CameraSpec` | `width`, `height`, `fovDeg`, mount `offsetBodyM[3]` and attitude `yawDeg` (right), `pitchDeg` (up), `rollDeg` (right) relative to the body; `hideOwnVehicle` (default true) keeps the carrier's own model out of its cameras |
| `Options` | `earth` / `imagery` / `elevation` (and URL templates, `maxLevel`), `sky` (dome + sun from the world's clock), `maxVehicles` drawn, `assetDir` for `models/<type>.glb`, `debugLayer` |
| `render()` | draws every camera from the world's current state; explicit, so a trainer pays for images only when it wants them (e.g. every agent step, not every FDM step) |
| `image(i)` | the last frame of camera `i`, valid until the next `render()` |
| `savePng(i, path)` | debugging and datasets |
| `timing()` | where the last render went: bookkeeping, tile merges, recording, GPU, host copy |
| `settle(n)` | run `n` frames without reading back, to let tiles stream in after a jump to a new region |

Cameras of a removed vehicle keep rendering from its last pose; up to 31
vehicles can hide themselves from their own cameras. Several `Sensors` in
one process share one Vulkan device.

## With VecEnv

`VecEnv::world()` is the batch's world through the object model, so cameras
go on batch vehicles the same way; images are read after `step()`:

```cpp
fsim::VecEnv env(opt);
fsim::vision::Sensors sensors(env.world());
std::vector<unsigned> cams;
for (fsim::Vehicle v : env.world().vehicles()) cams.push_back(sensors.addCamera(v, nose)); // "env<e>/<v>", batch order
for (;;) {
    auto r = env.step(actions);
    sensors.render();
    for (unsigned c : cams) learner.see(sensors.image(c)); // alongside r.observations
}
```

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

- RGB only (depth and segmentation follow the same path and are next).
- Cameras are added before the first `render()`; the scene is compiled once.
- Whether the vehicle's own model occludes the view depends on the mount: a
  camera 2 m ahead of the origin sits inside the c172's cowling, which is why
  `hideOwnVehicle` defaults to on.
