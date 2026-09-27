"""Flight tasks and suggestions through Python (docs/flight-autonomy.md, 4.11): a command kept by id and flown on a
task command, as often as its repetition says; a command refused that Clamp would fly suggested as a task."""
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-tasks", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


def seconds(w, s):
    w.step(max(1, round(s / w.step_seconds)))


class TasksTest(unittest.TestCase):
    def test_a_task_kept_and_flown(self):
        w = make_world()
        v = viper(w)
        roll = fsim.BatchCommand("submit_behavior", "aerobatics", manoeuvre=0.0)
        v.store_task(7, roll, attempts=2, interval_s=1.0)
        s = v.task_status(7)
        self.assertEqual(s.state, fsim.TaskState.AWAITING_EXECUTION)
        self.assertEqual(s.runs, 2)
        self.assertEqual(agra.task_state(s), "AWAITING_EXECUTION_APPROVAL")
        a = v.command_task(7, command_id=4)
        self.assertEqual(a.command_id, 4)
        self.assertIn((fsim.RequirementKind.TASK, 7), a.info.trace)
        with self.assertRaises(fsim.Rejected) as active:
            v.command_task(7)
        self.assertEqual(active.exception.reason, "task_active")
        for _ in range(120):
            seconds(w, 0.5)
            if v.task_status(7).state == fsim.TaskState.COMPLETED:
                break
        s = v.task_status(7)
        self.assertEqual(s.state, fsim.TaskState.COMPLETED)
        self.assertEqual((s.run, s.runs), (2, 2))
        self.assertEqual(s.percent, 100.0)
        self.assertEqual([t.id for t in v.tasks()], [7])
        v.remove_task(7)
        self.assertIsNone(v.task_status(7))
        with self.assertRaises(fsim.Rejected) as unknown:
            v.command_task(7)
        self.assertEqual(unknown.exception.reason, "unknown_task")

    def test_a_suggestion(self):
        w = make_world()
        v = viper(w)
        with self.assertRaises(fsim.Rejected) as refused:
            v.submit_hsa(heading_rad=1.5, speed=600.0, speed_reference="true_airspeed", range=fsim.RangePolicy.REJECT)
        suggestion = refused.exception.suggestion
        self.assertTrue(suggestion & (1 << 63))
        s = v.task_status(suggestion)
        self.assertTrue(s.suggested)
        a = v.command_task(suggestion, range=fsim.RangePolicy.REJECT)
        self.assertTrue(a.live)


if __name__ == "__main__":
    unittest.main()
