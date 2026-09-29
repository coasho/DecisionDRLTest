"""The fleet acceptance test's Python twin (docs/flight-autonomy.md, 9.3): one
aircraft of each class gives, through the Python surface, the answers
tests/test_fleet.cpp checks in C++ - what it offers and why not, the reasons
its refusals carry, what the ground allows - and flies some of its cases to
the same thresholds."""
import math
import unittest

import fsim
from fsim import Level

#: one aircraft of each class the thresholds are set for
CLASSES = {"c172": "fixed-wing direct", "f16c": "fixed-wing fly-by-wire", "uh60": "helicopter", "iris": "multirotor"}
ROTOR = {"uh60", "iris"}
EARTH_M = 6371000.0
SUPPORT_VALUES = {"gear": (1.0,), "flaps": (0.0,), "wheel_brakes": (0.0, 0.0), "speedbrake": (0.0,), "pitch_trim": (0.0,),
                  "engines": (0.5, fsim.HOLD, fsim.HOLD, fsim.HOLD)}


def make_world():
    # flat ground at sea level, as the C++ test's world (no terrain: 150 m over the Pyrenees is underground)
    return fsim.World("py-fleet", publish=False, workers=1, pin_workers=False, seed=29, terrain=False)


def steps(w, seconds):
    return int(round(seconds / w.step_seconds))


def ground_distance(a, b):
    north = (a.latitude_rad - b.latitude_rad) * EARTH_M
    east = (a.longitude_rad - b.longitude_rad) * EARTH_M * math.cos(b.latitude_rad)
    return math.hypot(north, east)


def heading_off_deg(s, heading_rad):
    return abs(math.degrees(math.remainder(s.euler_rad[2] - heading_rad, 2.0 * math.pi)))


class Plane:
    """One aircraft at its standard condition (fleet()): a wing at 3,000 m at its reference airspeed heading north, a
    rotorcraft hovering at 150 m; with ``helper_ahead_s``, another of its type that many seconds of its cruise ahead,
    flying on straight (a rotorcraft's hovering)."""

    def __init__(self, w, kind, latitude_deg, helper_ahead_s=0.0, parked=False):
        self.kind, self.rotor = kind, kind in ROTOR
        probe = w.create_vehicle(kind + "-probe", "jsbsim:" + kind, latitude_deg=latitude_deg, longitude_deg=0.0, altitude_msl_m=3000.0,
                                 airspeed_ms=0.0)
        self.tas = probe.profile_value("plant/tas_ms")
        self.cruise = probe.performance.cruise_tas_ms
        pitch, roll = probe.profile_value("hover/pitch_attitude_deg"), probe.profile_value("hover/roll_attitude_deg")
        self.hover = (pitch if math.isfinite(pitch) else 0.0, roll if math.isfinite(roll) else 0.0)
        probe.remove()
        if parked:
            self.v = w.create_vehicle(kind + "-parked", "jsbsim:" + kind, latitude_deg=latitude_deg, longitude_deg=0.0, on_ground=True,
                                      airspeed_ms=0.0)
            self.helper = None
            return
        self.v = self._spawn(w, kind, latitude_deg)
        self.helper = self._spawn(w, kind + "-helper", latitude_deg + math.degrees(helper_ahead_s * self.cruise / EARTH_M)) if helper_ahead_s else None
        self.start = None

    def _spawn(self, w, name, latitude_deg):
        if self.rotor:
            return w.create_vehicle(name, "jsbsim:" + self.kind, latitude_deg=latitude_deg, longitude_deg=0.0, altitude_msl_m=150.0,
                                    airspeed_ms=0.0, pitch_deg=self.hover[0], roll_deg=self.hover[1])
        return w.create_vehicle(name, "jsbsim:" + self.kind, latitude_deg=latitude_deg, longitude_deg=0.0, altitude_msl_m=3000.0,
                                airspeed_ms=self.tas)

    def still(self, v):
        """Straight and level where it is: a wing's velocity loop at its speed and heading; a rotorcraft still over the ground."""
        s = v.state
        if self.rotor:
            return v.submit(Level.VELOCITY, vertical_speed_ms=0.0, north_ms=0.0, east_ms=0.0, heading_rad=s.euler_rad[2])
        return v.submit(Level.VELOCITY, airspeed_ms=s.airspeed_true_ms, vertical_speed_ms=0.0, heading_rad=s.euler_rad[2])

    @property
    def scale(self):
        """How far a case's geometry reaches: 20 s at its cruise."""
        return 20.0 * self.cruise


def fleet(w, helper_ahead_s=None):
    """One aircraft of each class at its standard condition, all settled on their velocity loops for 10 s together, then
    let go (their helpers fly on). ``helper_ahead_s(kind)``: a helper that many seconds of its cruise ahead."""
    planes = [Plane(w, kind, 40.0 + i, helper_ahead_s(kind) if helper_ahead_s else 0.0) for i, kind in enumerate(CLASSES)]
    held = [p.still(p.v) for p in planes]
    for p in planes:
        if p.helper:
            p.still(p.helper)
    w.step(steps(w, 10.0))
    for p, a in zip(planes, held):
        a.cancel()
        p.start = fsim.VehicleState.from_buffer_copy(p.v.state)  # (the state is live: rewritten every step)
    return planes


def refusal(v, feature):
    """The reason a NEW of a capability the vehicle does not offer is refused with, as tests/test_fleet.cpp submits it."""
    kind = feature.rsplit(".", 1)[1]
    try:
        if feature.startswith("fsim.guidance."):
            v.submit_behavior(kind)
        elif feature.startswith("fsim.support.") or feature == "fsim.flight.engines":
            v.submit_support(kind, *SUPPORT_VALUES[kind])
        else:
            return None
    except fsim.Rejected as e:
        return e.reason
    return "accepted"


class FleetTwinTest(unittest.TestCase):
    def test_discovery_tells_the_truth(self):
        w = make_world()
        planes = fleet(w)
        for p in planes:
            offered = {c.id for c in p.v.capabilities()}
            for row in p.v.support_table():
                with self.subTest(aircraft=p.kind, feature=row.feature):
                    # an offered capability is one the table says works; what it does not offer, the table says why
                    if row.feature == row.capability:
                        self.assertEqual(row.feature in offered, row.support in (fsim.Support.SUPPORTED, fsim.Support.PARTIAL))
                    if row.support == fsim.Support.NOT_SUPPORTED:
                        self.assertTrue(row.rules)
                        self.assertTrue(row.evidence)
                    if row.support == fsim.Support.PARTIAL:
                        self.assertTrue(row.missing)
                    if row.feature != row.capability or row.feature in offered:
                        continue
                    # every refusal the reason section 4.3 gives, and the status to match
                    expected = "not_supported" if row.support == fsim.Support.NOT_SUPPORTED else "not_implemented"
                    reason = refusal(p.v, row.feature)
                    if reason is not None:
                        self.assertEqual(reason, expected)
                    status = p.v.availability(row.feature)
                    self.assertEqual(status.availability, fsim.Availability.UNAVAILABLE)
                    self.assertEqual(status.reason, expected)
            with self.assertRaises(fsim.Rejected) as refused:
                p.v.submit_behavior("warp_drive")
            self.assertEqual(refused.exception.reason, "unknown_capability")

    def test_on_the_ground(self):
        w = make_world()
        planes = [Plane(w, kind, 40.0 + i, parked=True) for i, kind in enumerate(CLASSES)]
        w.step(steps(w, 5.0))
        for p in planes:
            with self.subTest(aircraft=p.kind):
                self.assertTrue(p.v.state.on_ground)
                offered = {c.id for c in p.v.capabilities()}
                self.assertEqual(p.v.availability("fsim.flight.velocity").availability, fsim.Availability.AVAILABLE)
                if "fsim.guidance.hsa" in offered:
                    # a policy's airborne guidance waits for the air; the platform's own paths do not
                    status = p.v.availability("fsim.guidance.hsa")
                    self.assertEqual((status.availability, status.reason), (fsim.Availability.TEMPORARILY_UNAVAILABLE, "on_ground"))
                    with self.assertRaises(fsim.Rejected) as refused:
                        p.v.submit_hsa(heading_rad=1.0)
                    self.assertEqual(refused.exception.reason, "on_ground")
                    p.v.submit_hsa(heading_rad=1.0, source=fsim.Source.AUTOPILOT).cancel()
                if "fsim.support.gear" in offered:
                    self.assertEqual(p.v.capability_limits("fsim.support.gear"), (("down", 0.5, 1.0),))
                    with self.assertRaises(fsim.Rejected) as refused:
                        p.v.submit_support("gear", 0.0)
                    self.assertEqual(refused.exception.reason, "unavailable")

    def test_flight_cases(self):
        """Velocity, HSA, loiter, a route plan and formation on each class, to the C++ test's thresholds."""
        # the velocity level: its heading, height and speed (a rotorcraft: its point)
        w = make_world()
        planes = fleet(w)
        for p in planes:
            p.still(p.v)
        w.step(steps(w, 30.0))
        for p in planes:
            with self.subTest(case="velocity", aircraft=p.kind):
                s = p.v.state
                self.assertLess(heading_off_deg(s, p.start.euler_rad[2]), 1.0)
                self.assertLess(abs(s.altitude_msl_m - p.start.altitude_msl_m), 1.0 if p.rotor else 50.0)
                if p.rotor:
                    self.assertLess(ground_distance(s, p.start), max(0.5, 0.1 * p.cruise))
                else:
                    self.assertLess(abs(s.airspeed_true_ms - p.start.airspeed_true_ms), 1.0)

        # HSA: a quarter turn right, a wing 200 m lower, a rotorcraft a tenth of its scale higher (2 to 20 m)
        w = make_world()
        planes = fleet(w)
        change = {p.kind: min(max(0.1 * p.scale, 2.0), 20.0) if p.rotor else -200.0 for p in planes}
        for p in planes:
            p.v.submit_hsa(heading_rad=math.remainder(p.start.euler_rad[2] + 0.5 * math.pi, 2.0 * math.pi),
                           altitude_m=p.start.altitude_msl_m + change[p.kind])
        w.step(steps(w, 120.0))
        for p in planes:
            with self.subTest(case="hsa", aircraft=p.kind):
                s = p.v.state
                self.assertLess(heading_off_deg(s, p.start.euler_rad[2] + 0.5 * math.pi), 1.0)
                self.assertLess(abs(s.altitude_msl_m - (p.start.altitude_msl_m + change[p.kind])), 0.5 if p.rotor else 20.0)
                self.assertFalse(s.on_ground)

        # loiter round where it is: flown on its circle (a wing's 1.25 times its least orbit and at least 1,500 m; here 3.5 km
        # for the F-16C, whose least is 3.1 km at its speed; a rotorcraft's half its scale)
        w = make_world()
        planes = fleet(w)
        radius = {"c172": 1500.0, "f16c": 3500.0}
        for p in planes:
            radius.setdefault(p.kind, max(3.0, 0.5 * p.scale))
            p.v.submit_behavior("loiter", radius_m=radius[p.kind])
        w.step(steps(w, max((1.0 + 2.5 * math.pi) * radius[p.kind] / p.cruise + 60.0 for p in planes)))
        for p in planes:
            with self.subTest(case="loiter", aircraft=p.kind):
                self.assertLess(abs(ground_distance(p.v.state, p.start) - radius[p.kind]), 0.02 * radius[p.kind])

        # a route kept as A-GRA's route plan (docs/flight-autonomy.md, 4.39): uploaded, prepared for activation, activated,
        # and flown to its end straight ahead (a wing's legs 2 km, a rotorcraft's its scale) - its plan's execution complete
        w = make_world()
        planes = fleet(w)
        legs = {p.kind: p.scale if p.rotor else 2000.0 for p in planes}
        for p in planes:
            s, d = p.start, legs[p.kind]
            psi = s.euler_rad[2]
            points = [fsim.Waypoint(s.latitude_rad + k * d * math.cos(psi) / EARTH_M,
                                    s.longitude_rad + k * d * math.sin(psi) / (EARTH_M * math.cos(s.latitude_rad)), s.altitude_msl_m)
                      for k in (1, 2)]
            self.assertTrue(p.v.plan_command(1, "prepare_for_upload").completed)
            p.v.publish_plan(fsim.RoutePlan(1, fsim.BatchCommand("submit_route", points)))
            self.assertTrue(p.v.plan_command(1, "upload").completed)
            self.assertTrue(p.v.plan_command(1, "prepare_for_activation").completed)
            self.assertTrue(p.v.plan_command(1, "activate").completed)
        w.step(steps(w, max(2.0 * legs[p.kind] / p.cruise * 1.5 + 60.0 for p in planes)))
        for p in planes:
            with self.subTest(case="route plan", aircraft=p.kind):
                s = p.v.plan_status(1)
                self.assertEqual((s.state, s.execution, s.reason), (fsim.PlanState.ACTIVATED, fsim.PlanExecution.COMPLETE, "goal_reached"))

        # formation: a wing 100 m behind its leader and 60 m right, from 5 s of its cruise behind; a rotorcraft its
        # cruise's metres behind and right, from three times as far
        w = make_world()
        planes = fleet(w, lambda kind: 3.0 if kind in ROTOR else 5.0)
        slot = {p.kind: (-p.cruise, p.cruise) if p.rotor else (-100.0, 60.0) for p in planes}
        for p in planes:
            p.v.submit_behavior("formation", target=p.helper, ahead_m=slot[p.kind][0], right_m=slot[p.kind][1])
        w.step(steps(w, 150.0))
        for p in planes:
            with self.subTest(case="formation", aircraft=p.kind):
                leader, me = p.helper.state, p.v.state
                psi = leader.euler_rad[2]
                ahead, right = slot[p.kind]
                north = (me.latitude_rad - leader.latitude_rad) * EARTH_M
                east = (me.longitude_rad - leader.longitude_rad) * EARTH_M * math.cos(leader.latitude_rad)
                off = math.hypot(north - (ahead * math.cos(psi) - right * math.sin(psi)), east - (ahead * math.sin(psi) + right * math.cos(psi)))
                self.assertLess(off, 0.5 if p.rotor else 20.0)


if __name__ == "__main__":
    unittest.main()
