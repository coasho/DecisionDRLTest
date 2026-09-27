"""The Vehicle Interface's waypoint following (docs/vehicle-interface.md), flying
for the viewer in an 8 m/s wind from the north. Four minutes.

- A Cessna 172 flies a triangle of 3 km legs round and round: fly-by turns
  begun before each point, on circles sized for its speed, the wind and 80 %
  of its bank; its progress names the point it flies to and counts the laps.
- An F-16C climbs at 10 m/s to 4000 m on its way, passes one point flown over
  and then intercepts the next leg, and orbits its last point when it gets there.
- A UH-60A flies 400 m legs at 20 m/s without stopping at its points, and stops
  and hovers over the last one.
- An IRIS flies 30 m legs round and round, its nose along its track.

Each flies what its own loops can do: the turns are planned from the
aircraft's performance, the legs flown by one path follower - a wing turns
at the rate that holds the path, a rotorcraft flies its velocity over the
ground. Every 20 s it prints where each has got.

    fsim python examples\\python\\vehicle_interface.py
    fsim viewer --camera chase --chase-distance 60       (a second terminal; tab switches aircraft)

--fast runs it as fast as it goes, without the viewer in mind.
"""
import math
import sys
import time

import fsim

real_time = "--fast" not in sys.argv
world = fsim.World("vehicle-interface")
world.set_wind(0.0, 8.0)  # from the north

R = 6371008.8


def at(vehicle, north, east, **fields):
    """A waypoint `north` and `east` metres from where the vehicle is now."""
    s = vehicle.state
    lat, lon = float(s.latitude_rad), float(s.longitude_rad)
    return fsim.Waypoint(lat + north / R, lon + east / (R * math.cos(lat)), **fields)


def hover(name, aircraft, lat, lon):
    """A rotorcraft in a hover at 100 m, at the attitude its profile hovers at, held still over the ground."""
    probe = world.create_vehicle("probe-" + aircraft, "jsbsim:" + aircraft, latitude_deg=lat + 1.0, longitude_deg=lon, altitude_msl_m=100.0)
    pitch, roll = (probe.profile_value("hover/%s_attitude_deg" % axis) for axis in ("pitch", "roll"))
    probe.remove()
    v = world.create_vehicle(name, "jsbsim:" + aircraft, latitude_deg=lat, longitude_deg=lon, altitude_msl_m=100.0, heading_deg=0.0,
                             airspeed_ms=0.0, pitch_deg=0.0 if math.isnan(pitch) else pitch, roll_deg=0.0 if math.isnan(roll) else roll)
    v.submit(fsim.Level.VELOCITY, vertical_speed_ms=0.0, north_ms=0.0, east_ms=0.0)
    return v


cessna = world.create_vehicle("c172-triangle", "jsbsim:c172x", latitude_deg=37.62, longitude_deg=-122.38, altitude_msl_m=1500,
                              heading_deg=90, airspeed_ms=55)
viper = world.create_vehicle("f16-climb-orbit", "jsbsim:f16c", latitude_deg=37.66, longitude_deg=-122.38, altitude_msl_m=3000,
                             heading_deg=90, airspeed_ms=160)
hawk = hover("uh60-hops", "uh60", 37.6, -122.36)
quad = hover("iris-square", "iris", 37.6, -122.358)
for v in (cessna, viper):  # on east while they get a feel for the wind (the turns are planned with it)
    v.submit_hsa(heading_rad=math.pi / 2, speed=v.state.airspeed_true_ms, altitude_m=v.state.altitude_msl_m)
world.step(int(10.0 / world.step_seconds))

routes = {
    cessna: cessna.submit_route([at(cessna, 0, 3000, speed=55.0, id=1), at(cessna, 2600, 1500, id=2), at(cessna, 0, 0, id=3)],
                                repeat=True),
    viper: viper.submit_route([at(viper, 0, 15000, altitude_m=4000.0, climb_rate_ms=10.0, id=11),
                               at(viper, 12000, 15000, turn="fly_over", id=12), at(viper, 12000, 30000, id=13)],
                              end="loiter"),
    hawk: hawk.submit_route([at(hawk, 0, 400, speed=20.0, speed_reference="ground_speed", id=21), at(hawk, 400, 400, id=22),
                             at(hawk, 400, 800, id=23), at(hawk, 0, 800, id=24)],
                            end="loiter"),
    quad: quad.submit_route([at(quad, 0, 30, speed=5.0, speed_reference="ground_speed", id=31), at(quad, 30, 30, id=32),
                             at(quad, 30, 0, id=33), at(quad, 0, 0, id=34)],
                            repeat=True),
}
for v, a in routes.items():
    print("%-16s its route accepted: %s" % (v.name, fsim.agra.activity_state(a.info)))

t0 = time.perf_counter()
start = world.time
next_report = start
while world.time < start + 240.0:
    world.step()
    if world.time >= next_report:
        next_report += 20.0
        print("t=%4.0f s" % (world.time - start))
        for v, a in routes.items():
            p, info = a.progress, a.info
            left = "" if math.isnan(p.distance_to_go_m) else ", %5.0f m to go" % p.distance_to_go_m
            print("    %-16s %-24s to point %d (id %2d) of %d, lap %d, %5.1f %%%s, cross-track %+6.1f m, %5.1f m/s over the ground"
                  % (v.name, fsim.agra.activity_state(info), p.segment, p.segment_id, p.segments, p.laps, p.percent, left,
                     p.cross_track_m, math.hypot(v.state.velocity_ned_ms[0], v.state.velocity_ned_ms[1])))
        sys.stdout.flush()
    if real_time:
        ahead = (world.time - start) - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
