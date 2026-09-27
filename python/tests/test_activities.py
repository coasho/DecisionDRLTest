"""Activity commands through Python (docs/flight-autonomy.md, 4.10): disabled and kept, enabled, reset, deleted for good,
re-ranked, unassigned; one whose command takes none refuses them."""
import unittest

import fsim
from fsim import Level, agra


def make_world():
    return fsim.World("py-activities", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


class ActivitiesTest(unittest.TestCase):
    def test_disable_enable_delete(self):
        w = make_world()
        v = viper(w)
        a = v.submit_hsa(heading_rad=1.0)
        a.disable()
        self.assertEqual(a.info.state, fsim.ActivityState.DISABLED)
        self.assertTrue(a.live)
        self.assertEqual(agra.activity_state(a.info), "DISABLED")
        a.enable()
        self.assertEqual(a.info.state, fsim.ActivityState.PENDING)
        a.change_rank((2, 1))
        self.assertEqual(a.info.rank, fsim.Rank(2, 1))
        a.reset()
        a.delete()
        self.assertEqual(a.info.state, fsim.ActivityState.DELETED)
        self.assertFalse(a.live)
        self.assertEqual(agra.activity_state(a.info), "DELETED")
        with self.assertRaises(fsim.Rejected) as ended:
            a.enable()
        self.assertEqual(ended.exception.reason, "activity_ended")

    def test_unassign_gives_the_axes_to_what_waits(self):
        w = make_world()
        v = viper(w)
        hsa = v.submit_hsa(heading_rad=1.0)
        nice = v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=1.5, interrupt=False)
        self.assertTrue(nice.deferred)
        hsa.unassign()
        self.assertEqual(nice.info.waiting, fsim.ActivityWait.NONE)
        self.assertEqual(hsa.info.waiting, fsim.ActivityWait.QUEUED)
        self.assertEqual(hsa.info.waiting_for, nice.id)

    def test_not_interactive(self):
        w = make_world()
        v = viper(w)
        a = v.submit(Level.VELOCITY, airspeed_ms=160.0, heading_rad=1.5, interactive=False)
        for command in (a.disable, a.enable, a.reset, a.delete, a.unassign):
            with self.assertRaises(fsim.Rejected) as refused:
                command()
            self.assertEqual(refused.exception.reason, "not_interactive")
        self.assertEqual(agra.cannot_comply("not_interactive"), "STATE_OR_SETTINGS")
        a.update(airspeed_ms=160.0, heading_rad=1.0)  # (UPDATE and CANCEL are the command's, and still taken)
        a.cancel()


if __name__ == "__main__":
    unittest.main()
