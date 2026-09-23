"""Cameras on vehicles (docs/sdk/vision.md): offscreen RGB, depth and
segmentation images of the same Earth the viewer draws, as numpy arrays
viewing the renderer's own buffers.

Imported on its own (``import fsim.vision``) because it opens a Vulkan
device: ``import fsim`` works on a machine without a GPU driver, this does
not.

    import fsim, fsim.vision
    world = fsim.World("cams")
    red = world.create_vehicle("red", altitude_msl_m=800, airspeed_ms=60)
    vision = fsim.vision.Vision(world)
    nose = vision.add_camera(red, width=256, height=192, fov_deg=70)
    world.step(); vision.render()
    rgb = nose.image()            # (192, 256, 3) uint8, valid until the next render
"""
import numpy as np

from . import _native, _vision
from .vecenv import VecEnv
from .world import Vehicle, World, _options


_vision.set_log_level(_native.log_level())  # the level fsim.set_log_level chose


def _camera_spec(width, height, fov_deg, offset_body_m, yaw_deg, pitch_deg, roll_deg, hide_own_vehicle, depth, segmentation):
    spec = _options(width=width, height=height, fov_deg=fov_deg, yaw_deg=yaw_deg, pitch_deg=pitch_deg, roll_deg=roll_deg,
                    hide_own_vehicle=hide_own_vehicle, depth=depth, segmentation=segmentation)
    if offset_body_m is not None:
        spec["offset_body_m"] = tuple(float(x) for x in offset_body_m)
    return spec


class Camera:
    """A camera mounted on a vehicle. Images are views valid until the next
    render; copy what you keep."""

    __slots__ = ("_vision", "index", "vehicle")

    def __init__(self, vision, index, vehicle):
        self._vision = vision
        self.index = index
        self.vehicle = vehicle

    def image(self):
        """(height, width, 3) uint8 RGB, top row first."""
        data, h, w = self._vision._h.image(self.index)
        return None if data is None else np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)

    def depth(self):
        """(height, width) float32 metres along the view axis, if asked for."""
        data, h, w = self._vision._h.depth(self.index)
        return None if data is None else np.frombuffer(data, dtype=np.float32).reshape(h, w)

    def segmentation(self):
        """(height, width) uint16 vehicle ids (Vision.segmentation_id), 0 where none."""
        data, h, w = self._vision._h.segmentation(self.index)
        return None if data is None else np.frombuffer(data, dtype=np.uint16).reshape(h, w)

    def save_png(self, path):
        self._vision._h.save_png(self.index, str(path))

    def save_segmentation_png(self, path):
        self._vision._h.save_segmentation_png(self.index, str(path))

    def remove(self):
        self._vision._h.remove_camera(self.index)


class Vision:
    """Cameras on the vehicles of one World.

    Options (fsim_vision_options): earth, imagery, elevation, imagery_url,
    elevation_url, max_level, sky, max_vehicles, asset_dir, debug_layer,
    publish, segmentation (needed before any camera asks for segmentation).
    """

    def __init__(self, world, *, earth=None, imagery=None, elevation=None, imagery_url=None, elevation_url=None,
                 max_level=None, sky=None, max_vehicles=None, asset_dir=None, debug_layer=None, publish=None,
                 segmentation=None, _handle=None):
        if not isinstance(world, World) and _handle is None:
            raise TypeError("Vision(world): a fsim.World")
        self._world = world
        self._h = _handle if _handle is not None else _vision.Vision(world._h, _options(
            earth=earth, imagery=imagery, elevation=elevation, imagery_url=imagery_url, elevation_url=elevation_url,
            max_level=max_level, sky=sky, max_vehicles=max_vehicles, asset_dir=asset_dir, debug_layer=debug_layer,
            publish=publish, segmentation=segmentation))

    def add_camera(self, vehicle, *, width=None, height=None, fov_deg=None, offset_body_m=None, yaw_deg=None,
                   pitch_deg=None, roll_deg=None, hide_own_vehicle=None, depth=None, segmentation=None):
        """Mount a camera: body-frame offset (x forward, y right, z down) and
        attitude relative to the body. Defaults: 128 x 128, 60 degrees."""
        vid = vehicle.id if isinstance(vehicle, Vehicle) else int(vehicle)
        index = self._h.add_camera(vid, _camera_spec(width, height, fov_deg, offset_body_m, yaw_deg, pitch_deg, roll_deg,
                                                     hide_own_vehicle, depth, segmentation))
        return Camera(self, index, vehicle)

    def render(self):
        """Draw every camera from the world's current state (GIL released)."""
        self._h.render()

    def settle(self, frames=30):
        """Let terrain stream in after a jump to a new region, without reading back."""
        self._h.settle(frames)

    def segmentation_id(self, vehicle):
        """The id a vehicle is painted with in segmentation images."""
        return self._h.segmentation_id(vehicle.id if isinstance(vehicle, Vehicle) else int(vehicle))

    @property
    def last_render_ms(self):
        return self._h.last_render_ms()

    @property
    def camera_count(self):
        return self._h.camera_count()


class VisionBatch:
    """One camera per batch vehicle of a VecEnv, images as tensors:
    ``rgb`` (N, H, W, 3) uint8, ``depth`` (N, H, W) float32 and
    ``segmentation`` (N, H, W) uint16 - views of the renderer's buffers,
    rewritten by every render."""

    def __init__(self, env, *, width=None, height=None, fov_deg=None, offset_body_m=None, yaw_deg=None, pitch_deg=None,
                 roll_deg=None, hide_own_vehicle=None, depth=None, segmentation=None, **vision_options):
        if not isinstance(env, VecEnv):
            raise TypeError("VisionBatch(env): a fsim.VecEnv")
        spec = _camera_spec(width, height, fov_deg, offset_body_m, yaw_deg, pitch_deg, roll_deg, hide_own_vehicle, depth,
                            segmentation)
        if segmentation:
            vision_options.setdefault("segmentation", True)
        self._h = _vision.VisionBatch(env._h, spec, _options(**vision_options))
        self.count = self._h.count()
        self.sensors = Vision(env.world, _handle=self._h.sensors())
        _, self.height, self.width = self.sensors._h.image(0)
        self._views()

    def _views(self):
        rgb, depth, seg = self._h.tensors()
        n, h, w = self.count, self.height, self.width
        self.rgb = np.frombuffer(rgb, dtype=np.uint8).reshape(n, h, w, 3)
        self.depth = None if depth is None else np.frombuffer(depth, dtype=np.float32).reshape(n, h, w)
        self.segmentation = None if seg is None else np.frombuffer(seg, dtype=np.uint16).reshape(n, h, w)

    def render(self):
        """Draw every batch camera; returns ``rgb``."""
        self._h.render()
        return self.rgb
