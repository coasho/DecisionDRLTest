"""The object model from Python: vehicles, commands at several control levels,
live state, the batched calls, the environment and effects.

    fsim python examples\\python\\world_tour.py        (and `fsim viewer` to watch it)
"""
import math
import time

import numpy as np

import fsim
from fsim import Level

world = fsim.World("python-tour")  # published: flightsim-viewer.exe lists it
world.set_wind(270, 8, turbulence=0.2)

red = world.create_vehicle("red-1", latitude_deg=37.62, longitude_deg=-122.38, altitude_msl_m=1500,
                           heading_deg=90, airspeed_ms=60)
blue = world.create_vehicle("blue-1", "jsbsim:c172x", latitude_deg=37.63, longitude_deg=-122.40,
                            altitude_msl_m=1600, heading_deg=90, airspeed_ms=60)
red.command_velocity(airspeed_ms=65, vertical_speed_ms=2, heading_rad=0.0)  # climb away north
blue.command_behavior("pursuit", target=red, range_m=300)  # and chase it
red.add_effect("gaussian_sensor_noise")

# Eight more, flown with the batched calls: one call reads all their states,
# one commands all of them, however many there are.
wing = [world.create_vehicle("wing-%d" % i, latitude_deg=37.60 - 0.004 * i, longitude_deg=-122.36,
                             altitude_msl_m=1400 + 20 * i, heading_deg=0, airspeed_ms=55) for i in range(8)]
ids = world.ids(wing)  # build once, reuse every step
rows = np.tile(fsim.COMMAND_DEFAULTS[Level.ATTITUDE], (len(wing), 1))  # roll, pitch, heading, max bank, throttle, airspeed
rows[:, 5] = 55.0
states = np.empty(len(wing), dtype=fsim.vehicle_state_dtype)

s = red.state  # a live view of red's state, rewritten in place by every step
t0 = time.perf_counter()
for k in range(1800):  # 60 s of simulated time
    world.states(ids, out=states)
    rows[:, 0] = 0.3 * np.sin(0.02 * k + np.arange(len(wing)))  # each banks in its own rhythm
    world.command(Level.ATTITUDE, ids, rows)
    world.step()
    if k % 300 == 0:
        gap = math.dist(red.state.position_ecef, blue.state.position_ecef)
        print("t=%5.1f s  red %6.0f m, heading %5.1f deg   blue %5.0f m behind   wing mean %6.0f m"
              % (world.time, s.altitude_msl_m, math.degrees(s.heading_rad) % 360, gap, states["altitude_msl_m"].mean()))

wall = time.perf_counter() - t0
print("%.0f s simulated in %.2f s (%.0fx real time), %d vehicles, %.0f vehicle-steps/s"
      % (world.time, wall, world.time / wall, len(world), world.vehicle_steps / wall))
