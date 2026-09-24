"""The two aircraft made with hangar (tools/hangar) flying a slalom for the
viewer: banked left, banked right, nose up and down, so the platform's
attitude controller works their ailerons, rudder and elevator. Ten minutes.

    fsim python examples\\python\\control_surfaces.py
    fsim viewer --camera chase --chase-distance 20       (a second terminal; tab switches aircraft)

--fast runs it as fast as it goes, without the viewer in mind.
"""
import math
import sys
import time

import fsim

real_time = "--fast" not in sys.argv
world = fsim.World("control-surfaces")
fleet = {  # name: (vehicle, throttle)
    "c172": (world.create_vehicle("c172", "jsbsim:c172", latitude_deg=37.62, longitude_deg=-122.38,
                                  altitude_msl_m=1500, heading_deg=90, airspeed_ms=55), 1.0),
    "skua": (world.create_vehicle("skua", "jsbsim:skua", latitude_deg=37.623, longitude_deg=-122.38,
                                  altitude_msl_m=1500, heading_deg=90, airspeed_ms=26), 0.8),
}

t0 = time.perf_counter()
last = -1
while world.time < 600.0:
    t = world.time
    # 35 degrees of bank each way, 5 s at a time; the nose drifts up and down
    roll = math.radians(35.0) * (1.0 if math.sin(2 * math.pi * t / 10.0) >= 0.0 else -1.0)
    pitch = math.radians(5.0 + 4.0 * math.sin(2 * math.pi * t / 23.0))
    for v, throttle in fleet.values():
        v.command_attitude(roll_rad=roll, pitch_rad=pitch, max_bank_rad=math.radians(45.0), throttle=throttle)
    world.step()
    if int(t // 37) != last:  # not a multiple of the slalom's period: a different moment each time
        last = int(t // 37)
        print("t=%4.0f s  " % t + "   ".join("%s %4.0f m %3.0f m/s  aileron %+5.1f elevator %+5.1f rudder %+5.1f deg"
              % (name, v.state.altitude_msl_m, v.state.airspeed_true_ms, math.degrees(v.state.aileron_rad),
                 math.degrees(v.state.elevator_rad), math.degrees(v.state.rudder_rad)) for name, (v, _) in fleet.items()),
              flush=True)
    if real_time:
        ahead = world.time - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
