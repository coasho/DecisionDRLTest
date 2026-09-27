"""The command envelope through Python (docs/flight-autonomy.md, 4.8): a command's id and requirements kept with
its activity, a validation that flies nothing, every finding named, several NEWs at once."""
import math
import unittest

import fsim
from fsim import Level


def make_world():
    return fsim.World("py-envelope", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


class EnvelopeTest(unittest.TestCase):
    def test_id_and_requirements_go_with_the_activity(self):
        w = make_world()
        v = viper(w)
        a = v.submit_hsa(heading_rad=0.5, command_id=42, trace=[("task", 7), (fsim.RequirementKind.COMMAND, 9)], interactive=False)
        self.assertEqual(a.command_id, 42)
        info = a.info
        self.assertEqual(info.command_id, 42)
        self.assertFalse(info.interactive)
        self.assertEqual(info.trace, ((fsim.RequirementKind.TASK, 7), (fsim.RequirementKind.COMMAND, 9)))
        # a refused UPDATE says which command it was about
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(speed=600.0, speed_reference="true_airspeed", range=fsim.RangePolicy.REJECT, command_id=5)
        self.assertEqual(refused.exception.command_id, 5)
        self.assertEqual(refused.exception.reason, "performance_limit")
        self.assertTrue(refused.exception.description)
        # the existing commands keep their records as they were
        self.assertEqual(v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=0.5).info.command_id, 0)

    def test_every_finding_and_every_adjustment(self):
        w = make_world()
        v = viper(w)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(speed=600.0, speed_reference="true_airspeed", altitude_m=25000.0, range=fsim.RangePolicy.REJECT)
        e = refused.exception
        self.assertEqual((e.index, e.constraint), (4, "max_altitude")) # the first checked, as it was
        self.assertEqual([(f.reason, f.index, f.constraint) for f in e.findings],
                         [("performance_limit", 4, "max_altitude"), ("performance_limit", 2, "max_airspeed")])
        # clamped instead: flown, with what it asked and what it flies
        a = v.submit_hsa(speed=600.0, speed_reference="true_airspeed", altitude_m=25000.0)
        self.assertTrue(a.clamped)
        findings, adjustments = w.last_command_details()
        self.assertEqual(findings, [])
        self.assertEqual([(x.index, x.constraint, x.requested) for x in adjustments], [(4, "max_altitude", 25000.0), (2, "max_airspeed", 600.0)])
        self.assertLess(adjustments[1].adjusted, 450.0)

    def test_a_validation_flies_nothing(self):
        w = make_world()
        v = viper(w)
        before = len(v.activities())
        ok = v.submit_hsa(heading_rad=1.0, validate_only=True, command_id=3)
        self.assertIsInstance(ok, fsim.Validation)
        self.assertTrue(ok.valid)
        self.assertEqual(ok.command_id, 3)
        fast = v.submit_hsa(speed=600.0, speed_reference="true_airspeed", validate_only=True)
        self.assertTrue(fast.valid and fast.clamped)
        self.assertEqual([x.index for x in fast.adjustments], [2])
        refused = v.submit_hsa(speed=600.0, speed_reference="true_airspeed", validate_only=True, range=fsim.RangePolicy.REJECT)
        self.assertFalse(refused.valid)
        self.assertEqual(refused.reason, "performance_limit")
        self.assertEqual([f.index for f in refused.findings], [2])
        self.assertEqual(len(v.activities()), before)

    def test_several_news_at_once(self):
        w = make_world()
        v = viper(w)
        now = w.time
        results = v.submit_batch([
            fsim.BatchCommand("submit_hsa", heading_rad=1.0, command_id=1),
            fsim.BatchCommand("submit_support", "gear", 2.0, range=fsim.RangePolicy.REJECT, command_id=2),
            fsim.BatchCommand("submit", Level.VELOCITY, airspeed_ms=160.0, heading_rad=0.5, command_id=3),
            fsim.BatchCommand("submit_hsa", heading_rad=2.0, validate_only=True),
        ])
        self.assertIsInstance(results[0], fsim.Activity)
        self.assertIsInstance(results[1], fsim.Rejected)
        self.assertEqual((results[1].reason, results[1].command_id), ("out_of_range", 2))
        self.assertIsInstance(results[2], fsim.Activity)
        self.assertIsInstance(results[3], fsim.Validation)
        # made in order at one moment: the velocity took the hsa's axes
        first = results[0].info
        self.assertEqual((first.state, first.reason, first.by), (fsim.ActivityState.CANCELED, "preempted", results[2].id))
        self.assertEqual(first.start_time, now)
        self.assertEqual(results[2].command_id, 3)


if __name__ == "__main__":
    unittest.main()
