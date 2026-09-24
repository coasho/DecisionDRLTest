"""The fighters built with hangar (aircraft/, docs/hangar.md) flying together
for the viewer: line abreast at 3,000 m, turning left and right in step and
easing up and down at 220 m/s, each through its own fly-by-wire. Five
minutes.

    fsim python examples\\python\\fighters.py
    fsim viewer --camera chase --chase-distance 60       (a second terminal; tab switches aircraft)

--fast runs it as fast as it goes, without the viewer in mind.
"""
import math
import sys
import time

import fsim

FIGHTERS = ("f16c", "f15c", "fa18c", "f22a", "f35a", "su27s", "su57", "mig29a",
            "typhoon", "rafale", "gripen", "mirage2000", "j10a", "j20a")

real_time = "--fast" not in sys.argv
world = fsim.World("fighters")
fleet = {}
for i, name in enumerate(FIGHTERS):
    # line abreast heading east, 90 m apart
    fleet[name] = world.create_vehicle(name, "jsbsim:" + name, latitude_deg=37.60 + i * 0.0008, longitude_deg=-122.38,
                                       altitude_msl_m=3000, heading_deg=90, airspeed_ms=220)

t0 = time.perf_counter()
last = -1
while world.time < 300.0:
    t = world.time
    # 60 degrees of bank each way, 15 s at a time, the nose easing up and down
    roll = math.radians(60.0) * math.sin(2 * math.pi * t / 30.0)
    pitch = math.radians(3.0 + 4.0 * math.sin(2 * math.pi * t / 47.0))
    for v in fleet.values():
        v.command_attitude(roll_rad=roll, pitch_rad=pitch, max_bank_rad=math.radians(65.0), airspeed_ms=220.0)
    world.step()
    if int(t // 20) != last:
        last = int(t // 20)
        print("t=%3.0f s  " % t + "  ".join("%s %3.0f m/s %4.0f m" % (name, v.state.airspeed_true_ms, v.state.altitude_msl_m)
                                          for name, v in fleet.items()), flush=True)
    if real_time:
        ahead = world.time - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
