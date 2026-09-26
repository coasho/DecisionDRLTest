"""Two things the control architecture added (docs/sdk/control.md), flying
for the viewer: axes owned apart, and envelope protection. Two minutes.

- The F-16C. An autopilot holds its vertical speed at zero and its airspeed
  - a velocity activity on pitch and thrust. A scripted "policy" flies only
  its bank, a new setpoint every step; the runtime merges the two. When the
  policy reaches for the pitch as well it is refused: the autopilot has it.
- Two pairs of hangar designs, a U-2S (its wing stalls at 8 degrees) and a
  KC-46A. Every 20 s the same aggressive pull is asked of each pair - 3 g
  from the U-2S, 2.5 g from the KC-46A - then wings level again. The first
  of each pair flies with envelope protection on, as every hangar design
  does; the second only reports. Protection eases the demand as the wing
  nears its limit; what still crosses - inertia, overshoot - it reports,
  with how far and how long. It promises nothing about the aircraft, and
  it does nothing else: what to do about a crossing is yours.

    fsim python examples\\python\\authority_and_envelope.py
    fsim viewer --camera chase --chase-distance 40       (a second terminal; tab switches aircraft)

--fast runs it as fast as it goes, without the viewer in mind.
"""
import math
import sys
import time

import fsim

real_time = "--fast" not in sys.argv
world = fsim.World("authority-and-envelope")
viper = world.create_vehicle("viper", "jsbsim:f16c", latitude_deg=37.62, longitude_deg=-122.38,
                             altitude_msl_m=3000, heading_deg=90, airspeed_ms=160)

# The F-16C: the autopilot owns pitch and thrust, the policy the bank (roll and yaw).
hold = viper.submit(fsim.Level.VELOCITY, airspeed_ms=160.0, vertical_speed_ms=0.0,
                    source=fsim.Source.AUTOPILOT, axes=fsim.Axis.PITCH | fsim.Axis.THRUST)
policy = viper.submit(fsim.Level.ATTITUDE, roll_rad=0.0, axes=fsim.Axis.LATERAL)
try:
    viper.submit(fsim.Level.ATTITUDE, pitch_rad=0.2, axes=fsim.Axis.PITCH)
except fsim.Rejected as refused:
    print("the policy reached for the pitch as well: refused, %s (the autopilot's activity %#x)"
          % (refused.reason, refused.other))

# The pairs: (vehicle, pull in g, protection).
pairs = []
for i, (aircraft, speed, pull) in enumerate((("u2s", 70.0, 3.0), ("kc46a", 120.0, 2.5))):
    for j, mode in enumerate(("limit", "report")):
        v = world.create_vehicle("%s-%s" % (aircraft, mode), "jsbsim:" + aircraft, latitude_deg=37.64 + 0.02 * i,
                                 longitude_deg=-122.38 + 0.01 * j, altitude_msl_m=3000, heading_deg=90, airspeed_ms=speed)
        v.set_protection(mode)
        v.submit(fsim.Level.ATTITUDE, roll_rad=0.0, pitch_rad=0.05, airspeed_ms=speed)
        v.envelope()
        pairs.append((v, pull, speed, mode))
print("the F-16C's autopilot holds %.0f m/s and level flight; " % viper.state.airspeed_true_ms
      + ", ".join("%s alpha_max %.1f deg" % (v.name, v.profile_value("envelope/clean/alpha_max_deg")) for v, _, _, _ in pairs[::2]))

t0 = time.perf_counter()
pulling = False
peaks = {v.name: -90.0 for v, _, _, _ in pairs}
next_report = 20.0
while world.time < 120.0:
    t = world.time
    policy.update(roll_rad=math.radians(45.0) * math.sin(2.0 * math.pi * t / 30.0))  # the policy's per-step path
    phase = t % 20.0
    if 5.0 <= phase < 9.0 and not pulling:  # the pull: the same demand for both of a pair
        pulling = True
        for v, pull, _, _ in pairs:
            v.submit(fsim.Level.ACCELERATION, load_factor_g=pull, roll_rate_rad_s=0.0, throttle=1.0)
    elif phase >= 9.0 and pulling:  # and wings level again
        pulling = False
        for v, _, speed, _ in pairs:
            v.submit(fsim.Level.ATTITUDE, roll_rad=0.0, pitch_rad=0.05, airspeed_ms=speed)
    world.step()
    for v, _, _, _ in pairs:
        peaks[v.name] = max(peaks[v.name], math.degrees(v.state.alpha_rad))
    if world.time >= next_report:
        next_report += 20.0
        s = viper.state
        print("t=%4.0f s  F-16C bank %+5.1f deg, vertical speed %+4.1f m/s, speed %+4.1f m/s"
              % (world.time, math.degrees(s.euler_rad[0]), -s.velocity_ned_ms[2], s.airspeed_true_ms - 160.0))
        for v, _, _, mode in pairs:
            e = v.envelope().limits["alpha_max"]
            print("        %-13s alpha up to %5.1f deg; beyond alpha_max %4.1f s, by up to %4.1f deg; demand eased in %4d updates"
                  % (v.name, peaks[v.name], e.exceeded_s, math.degrees(e.worst_excess), e.limited_updates))
            peaks[v.name] = -90.0
        sys.stdout.flush()
    if real_time:
        ahead = world.time - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)

print("the autopilot's activity is %s and the policy's %s: nobody ended them" % (hold.state.name.lower(), policy.state.name.lower()))
