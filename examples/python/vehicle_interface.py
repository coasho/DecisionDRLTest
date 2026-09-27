"""The Vehicle Interface's waypoint following, loiter patterns and curve
following (docs/vehicle-interface.md), flying for the viewer in an 8 m/s wind
from the north. Four minutes.

- A Cessna 172 flies a triangle of 3 km legs round and round: fly-by turns
  begun before each point, on circles sized for its speed, the wind and 80 %
  of its bank; its progress names the point it flies to and counts the laps.
  At 150 s the platform revokes its route for collision avoidance: the route
  ends, and its vehicle default - a hold - flies on as it was.
- An F-16C climbs at 10 m/s to 4000 m on its way, passes one point flown over
  and then intercepts the next leg, and orbits its last point when it gets there.
- A UH-60A flies 400 m legs at 20 m/s without stopping at its points, and stops
  and hovers over the last one.
- An IRIS flies 30 m legs round and round, its nose along its track.
- A second Cessna holds over a fix 4 km ahead, ATC's holding pattern from
  its defaults alone: right turns, the inbound course the way it arrived,
  rate-one turns, a minute's legs. Its policy commands it by grant: it asks
  for control of the pattern capability before it submits the hold.
- A third Cessna flies a slalom of Bezier curves, holding its airspeed while
  its ground speed stays within 45 to 65 m/s; a minute in, more of the slalom
  is appended while it flies, and at the end it orbits the last point.
- A second IRIS climbs 20 m along an S of Bezier curves in exactly a minute,
  and hovers at its end.

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


class Pen:
    """Draws a curve as fsim.BezierSegment pieces from where it last ended, in metres from the curve's reference:
    straights and arcs, each a quintic Hermite of its ends (position, tangent, curvature)."""

    def __init__(self, course_rad):
        self.north = self.east = self.down = 0.0
        self.course = course_rad

    def straight(self, length, climb=0.0):
        return self._piece(0.0, 0.0, length, climb)

    def arc(self, radius, degrees, climb=0.0):  # + right
        return self._piece(radius, math.radians(degrees), radius * math.radians(abs(degrees)), climb)

    def _piece(self, radius, sweep, length, climb):
        chi0, kappa = self.course, (math.copysign(1.0 / radius, sweep) if radius else 0.0)
        chi1 = chi0 + (sweep if radius else 0.0)
        if radius:  # round its centre, on its right for a right turn
            side = 1.0 if sweep > 0 else -1.0
            cn, ce = self.north - side * radius * math.sin(chi0), self.east + side * radius * math.cos(chi0)
            n1, e1 = cn + side * radius * math.sin(chi1), ce - side * radius * math.cos(chi1)
        else:
            n1, e1 = self.north + length * math.cos(chi0), self.east + length * math.sin(chi0)
        p0, p1 = (self.north, self.east, self.down), (n1, e1, self.down - climb)
        v0 = (length * math.cos(chi0), length * math.sin(chi0), -climb)
        v1 = (length * math.cos(chi1), length * math.sin(chi1), -climb)
        a0 = (-length * length * kappa * math.sin(chi0), length * length * kappa * math.cos(chi0), 0.0)
        a1 = (-length * length * kappa * math.sin(chi1), length * length * kappa * math.cos(chi1), 0.0)
        axes = [[p0[k], p0[k] + v0[k] / 5, p0[k] + 2 * v0[k] / 5 + a0[k] / 20, p1[k] - 2 * v1[k] / 5 + a1[k] / 20, p1[k] - v1[k] / 5, p1[k]]
                for k in range(3)]
        self.north, self.east, self.down, self.course = n1, e1, self.down - climb, chi1
        return fsim.BezierSegment(*axes)


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
holder = world.create_vehicle("c172-hold", "jsbsim:c172x", latitude_deg=37.64, longitude_deg=-122.42, altitude_msl_m=1200,
                              heading_deg=90, airspeed_ms=55)
slalom = world.create_vehicle("c172-slalom", "jsbsim:c172x", latitude_deg=37.68, longitude_deg=-122.38, altitude_msl_m=1500,
                              heading_deg=90, airspeed_ms=55)
hawk = hover("uh60-hops", "uh60", 37.6, -122.36)
quad = hover("iris-square", "iris", 37.6, -122.358)
climber = hover("iris-climb", "iris", 37.6, -122.356)
for v in (cessna, viper, holder, slalom):  # on east while they get a feel for the wind (the turns are planned with it)
    v.submit_hsa(heading_rad=math.pi / 2, speed=v.state.airspeed_true_ms, altitude_m=v.state.altitude_msl_m)
world.step(int(10.0 / world.step_seconds))

fix = at(holder, 0, 4000)
holder.set_control_mode("granted")               # its policy commands only what it is granted
try:
    holder.submit_pattern(pattern="hold")
except fsim.Rejected as refused:
    print("c172-hold        a hold without a grant: %s" % refused.reason)
holder.request_control("fsim.guidance.pattern")  # granted: allowed, and available
cessna.set_vehicle_default("hold")               # what flies its axes if its route is taken away
activities = {
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
    holder: holder.submit_pattern(pattern="hold", latitude_rad=fix.latitude_rad, longitude_rad=fix.longitude_rad),
}
# the slalom from where the Cessna is (the curve's reference, left out): 900 m arcs either way; more of it a minute in
pen = Pen(math.pi / 2)
activities[slalom] = slalom.submit_curve([pen.straight(1000), pen.arc(900, 90), pen.arc(900, -90), pen.arc(900, 90), pen.arc(900, -90)],
                                         speed_min_ms=45.0, speed_max_ms=65.0, end="loiter")
more = [pen.arc(900, -90), pen.arc(900, 90), pen.straight(1000)]
# the IRIS's climbing S, 15 m turns, in a minute
up = Pen(0.0)
activities[climber] = climber.submit_curve([up.straight(10, 2), up.arc(15, 180, 8), up.arc(15, -180, 8), up.straight(10, 2)],
                                           duration_s=60.0, end="loiter")
for v, a in activities.items():
    print("%-16s its %s accepted: %s" % (v.name, a.level, fsim.agra.activity_state(a.info)))

t0 = time.perf_counter()
start = world.time
revoke = True
next_report = start
while world.time < start + 240.0:
    world.step()
    if revoke and world.time >= start + 150.0:
        cessna.revoke_control("fsim.guidance.route", "collision_avoidance")
        info = activities[cessna].info
        print("t=%4.0f s  c172-triangle: the platform revoked its route (%s, %s); its hold flies on"
              % (world.time - start, fsim.agra.activity_state(info), info.reason))
        revoke = False
    if more and world.time >= start + 60.0:
        activities[slalom].append(more)  # on from where its curve ends, the same activity
        print("t=%4.0f s  c172-slalom: %d more segments appended" % (world.time - start, len(more)))
        more = None
    if world.time >= next_report:
        next_report += 20.0
        print("t=%4.0f s" % (world.time - start))
        for v, a in activities.items():
            p, info = a.progress, a.info
            left = "" if math.isnan(p.distance_to_go_m) else ", %5.0f m to go" % p.distance_to_go_m
            where = {"route": "to point %d (id %2d)" % (p.segment, p.segment_id), "curve": "on segment %d" % p.segment}.get(a.level,
                                                                                                                   "on piece %d" % p.segment)
            print("    %-16s %-24s %s of %d, lap %d, %5.1f %%%s, cross-track %+6.1f m, %5.1f m/s over the ground"
                  % (v.name, fsim.agra.activity_state(info), where, p.segments, p.laps, p.percent, left, p.cross_track_m,
                     math.hypot(v.state.velocity_ned_ms[0], v.state.velocity_ned_ms[1])))
        sys.stdout.flush()
    if real_time:
        ahead = (world.time - start) - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
