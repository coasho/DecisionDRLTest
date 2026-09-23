import unittest

import numpy as np

import fsim

try:
    import fsim.vision

    # A device, not just the DLLs: CI runners have the Vulkan loader but no GPU.
    fsim.vision.Vision(fsim.World("py-vision-probe", publish=False, workers=1, pin_workers=False), earth=False, publish=False)
    VISION = None
except (ImportError, fsim.Error) as e:  # built without the renderer, or no Vulkan device
    VISION = str(e)

# Sky and vehicles only: no terrain tiles, so no network and no waiting.
QUIET = dict(earth=False, publish=False)


@unittest.skipIf(VISION is not None, "fsim.vision unavailable: %s" % VISION)
class VisionTest(unittest.TestCase):
    def test_camera_images_are_views(self):
        world = fsim.World("py-vision", publish=False, workers=1, pin_workers=False)
        a = world.create_vehicle("a", altitude_msl_m=1500.0, airspeed_ms=60.0, heading_deg=0.0)
        world.create_vehicle("b", altitude_msl_m=1500.0, airspeed_ms=60.0, heading_deg=0.0, latitude_deg=37.6188 + 0.0015)
        vision = fsim.vision.Vision(world, segmentation=True, **QUIET)
        cam = vision.add_camera(a, width=96, height=64, fov_deg=60, depth=True, segmentation=True)
        world.step()
        vision.render()
        rgb = cam.image()
        self.assertEqual(rgb.shape, (64, 96, 3))
        self.assertEqual(rgb.dtype, np.uint8)
        self.assertGreater(int(rgb.max()) - int(rgb.min()), 0)  # a picture, not a blank
        depth = cam.depth()
        self.assertEqual(depth.shape, (64, 96))
        seg = cam.segmentation()
        self.assertEqual(seg.shape, (64, 96))
        self.assertGreater(vision.last_render_ms, 0.0)

    def test_batch_tensors(self):
        env = fsim.VecEnv(3, publish=False, workers=1, seed=1)
        env.reset()
        batch = fsim.vision.VisionBatch(env, width=64, height=48, depth=True, **QUIET)
        self.assertEqual(batch.count, 3)
        rgb = batch.render()
        self.assertEqual(rgb.shape, (3, 48, 64, 3))
        self.assertEqual(batch.depth.shape, (3, 48, 64))
        env.step(np.zeros((3, env.action_size), np.float32))
        self.assertIs(batch.render(), rgb)  # the same view, re-rendered


if __name__ == "__main__":
    unittest.main()
