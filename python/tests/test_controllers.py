"""Named controllers through Python (docs/flight-autonomy.md, 4.12; A-GRA's several MA services): a grant is one
controller's, and under ControlMode.GRANTED only its holder commands the capability and addresses what it flies."""
import unittest

import fsim

HSA = "fsim.guidance.hsa"


def make_world():
    return fsim.World("py-controllers", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class ControllersTest(unittest.TestCase):
    def test_a_grant_is_one_controllers(self):
        w = make_world()
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                             heading_deg=90.0)
        w.step(5)
        v.set_control_mode(fsim.ControlMode.GRANTED)
        v.request_control(HSA, controller=1)
        self.assertEqual(v.control_status(HSA), fsim.ControlStatus(True, True, 1))
        with self.assertRaises(fsim.Rejected) as held:
            v.request_control(HSA, controller=2)
        self.assertEqual(held.exception.reason, "authority_held")
        with self.assertRaises(fsim.Rejected) as none:
            v.submit_hsa(heading_rad=1.0, controller=2)
        self.assertEqual(none.exception.reason, "not_granted")
        a = v.submit_hsa(heading_rad=1.0, controller=1)
        self.assertEqual((a.controller, a.info.controller), (1, 1))
        a.update(altitude_m=3100.0)  # (its own controller's: the Activity declares it)
        other = fsim.Activity(w, a.id, "hsa", controller=2)
        with self.assertRaises(fsim.Rejected) as addressed:
            other.cancel()
        self.assertEqual(addressed.exception.reason, "authority_held")
        with self.assertRaises(fsim.Rejected) as released:
            v.release_control(HSA, controller=2)
        self.assertEqual(released.exception.reason, "not_granted")
        v.release_control(HSA, controller=1)
        self.assertEqual(a.info.reason, "released")
        batch = v.submit_batch([fsim.BatchCommand("submit_hsa", heading_rad=1.0, controller=3)])
        self.assertIsInstance(batch[0], fsim.Rejected)  # (controller 3 holds no grant)
        v.set_control_mode(fsim.ControlMode.OPEN)
        b = v.submit_batch([fsim.BatchCommand("submit_hsa", heading_rad=1.0, controller=3)])[0]
        self.assertEqual((b.controller, b.info.controller), (3, 3))


if __name__ == "__main__":
    unittest.main()
