"""Cameras from Python: a pair of aircraft over Yosemite, a nose camera and a
chase camera on the lead, images as numpy arrays, saved as PNGs.

    fsim python examples\\python\\cameras.py [output directory]

The Earth's imagery and relief stream in from the public tile servers (or
the tile cache), as they do for the viewer.
"""
import datetime
import os
import sys

import fsim
import fsim.vision

out = sys.argv[1] if len(sys.argv) > 1 else "python-cameras"
os.makedirs(out, exist_ok=True)

world = fsim.World("python-cameras")
sun = datetime.datetime(2026, 6, 21, 17, 30, tzinfo=datetime.timezone.utc)  # the sun high over the valley
world.set_environment(epoch_utc_seconds=sun.timestamp())


def spawn(name, north_deg, east_deg):
    v = world.create_vehicle(name, latitude_deg=37.72 + north_deg, longitude_deg=-119.55 + east_deg,
                             altitude_msl_m=3200, heading_deg=90, airspeed_ms=55)
    v.command_attitude(roll_rad=0.0, pitch_rad=0.03)
    return v


lead = spawn("lead", 0.0, 0.0)
wing = spawn("wing", -0.0008, 0.0003)  # right of the lead and a little behind

vision = fsim.vision.Vision(world, segmentation=True)
nose = vision.add_camera(lead, width=640, height=360, fov_deg=70, offset_body_m=(2.0, 0.0, -0.3), depth=True,
                         segmentation=True)
chase = vision.add_camera(lead, width=640, height=360, fov_deg=70, offset_body_m=(-25.0, 0.0, -6.0), pitch_deg=-10,
                          hide_own_vehicle=False, segmentation=True)
vision.render()
vision.settle(60)  # let the terrain around the start stream in

for k in range(5):
    world.step(30)
    vision.render()
    for name, cam in (("nose", nose), ("chase", chase)):
        cam.save_png(os.path.join(out, "%s-%d.png" % (name, k)))
    seg = chase.segmentation()  # views of the renderer's buffers, valid until the next render
    lead_px = int((seg == vision.segmentation_id(lead)).sum())
    wing_px = int((nose.segmentation() == vision.segmentation_id(wing)).sum())
    ground = nose.depth()[nose.depth().shape[0] - 1].mean()
    print("t=%4.1f s  chase view: lead %5d px   nose view: wing %4d px, ground ahead %5.0f m   render %.1f ms"
          % (world.time, lead_px, wing_px, ground, vision.last_render_ms))
print("images in %s" % os.path.abspath(out))
