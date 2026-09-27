"""Ranks, queues and time windows through Python (docs/flight-autonomy.md, 4.9): a command outranked by what flies
waits and starts when it may; a start window delays a start; a capability's precedence is the platform's."""
import math
import unittest

import fsim
from fsim import Level, agra


def make_world():
    return fsim.World("py-schedule", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


def seconds(w, s):
    w.step(max(1, round(s / w.step_seconds)))


class ScheduleTest(unittest.TestCase):
    def test_outranked_waits_then_starts(self):
        w = make_world()
        v = viper(w)
        first = v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=1.5, rank=(1, 0))
        self.assertFalse(first.deferred)
        waits = v.submit_hsa(heading_rad=1.0, rank=fsim.Rank(5, 2))
        self.assertTrue(waits.deferred)
        info = waits.info
        self.assertEqual(info.state, fsim.ActivityState.PENDING)
        self.assertEqual(info.waiting, fsim.ActivityWait.QUEUED)
        self.assertEqual(info.waiting_for, first.id)
        self.assertEqual(info.basis, fsim.ActivityBasis.PLANNED)
        self.assertEqual(info.rank, fsim.Rank(5, 2))
        self.assertTrue(info.interrupt)
        self.assertTrue(math.isnan(info.window.start_not_before))
        self.assertEqual(agra.activity_state(info), "ACTIVE_FULLY_CONSTRAINED")
        self.assertEqual(agra.activity_basis(info), "PLANNED")
        # a validation of one that would wait says so
        check = v.submit_hsa(heading_rad=1.0, rank=(6, 0), validate_only=True)
        self.assertTrue(check.valid)
        self.assertTrue(check.deferred)
        # what it waits for ends: it starts
        first.cancel()
        self.assertEqual(waits.info.waiting, fsim.ActivityWait.NONE)
        seconds(w, 0.5)
        self.assertEqual(waits.info.state, fsim.ActivityState.ACTIVE)
        self.assertEqual(agra.activity_basis(waits.info), "ACTUAL")

    def test_a_start_window_and_a_nice_command(self):
        w = make_world()
        v = viper(w)
        later = v.submit_hsa(heading_rad=1.0, window=fsim.TimeWindow(start_not_before=w.time + 1.0, criticality="start"))
        self.assertTrue(later.deferred)
        self.assertEqual(later.info.waiting, fsim.ActivityWait.SCHEDULED)
        self.assertEqual(agra.activity_state(later.info, w.time), "ENABLED")
        self.assertEqual(later.info.window.criticality, fsim.TimeCriticality.START)
        seconds(w, 1.2)
        self.assertEqual(later.info.waiting, fsim.ActivityWait.NONE)
        # a "nice" command waits for what flies, whatever its rank
        nice = v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=1.5, interrupt=False)
        self.assertTrue(nice.deferred)
        self.assertFalse(nice.info.interrupt)
        # an end window already closed cannot be met
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=1.0, window={"end_not_after": w.time})
        self.assertEqual(refused.exception.reason, "time_constraint")
        self.assertEqual(agra.cannot_comply("time_constraint"), "CONSTRAINT_TIME")

    def test_capability_precedence_is_the_platforms(self):
        w = make_world()
        v = viper(w)
        self.assertEqual(v.capability_precedence("fsim.flight.velocity"), 0)
        v.set_capability_precedence("fsim.flight.velocity", 2)
        self.assertEqual(v.capability_precedence("fsim.flight.velocity"), 2)
        velocity = v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=1.5)
        self.assertEqual(velocity.info.precedence, 2)
        hsa = v.submit_hsa(heading_rad=1.0, rank=(9, 9))  # its capability comes first
        self.assertFalse(hsa.deferred)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=1.0, precedence_override=0)
        self.assertEqual(refused.exception.reason, "not_allowed")
        # A-GRA's ranking and temporal constraints as submit keywords: InterruptOtherActivities left out is False
        options = agra.command_options({"Rank": (3, 1)}, {"StartTimeWindow": (w.time + 5.0, None),
                                                          "TemporalCriticality": "START_TIME_CRITICAL"})
        self.assertEqual(options["rank"], fsim.Rank(3, 1))
        self.assertFalse(options["interrupt"])
        self.assertEqual(options["window"].criticality, fsim.TimeCriticality.START)
        waits = v.submit_hsa(heading_rad=2.0, **options)
        self.assertTrue(waits.deferred)
        self.assertEqual(waits.info.waiting, fsim.ActivityWait.SCHEDULED)


if __name__ == "__main__":
    unittest.main()
