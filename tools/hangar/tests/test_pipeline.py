"""Every stage in order on a copy of a design, through the platform's own
JSBSim: the tool works end to end. Quick mode (coarse tables, short flight
tests), so the numbers are a first look; what is asserted is that each stage
produces its outputs and that the aircraft it builds flies. Needs the fsim
package on the path; skipped without it."""
import glob
import json
import math
import os
import re
import shutil
import tempfile
import unittest

try:
    import fsim
except ImportError:
    fsim = None

from hangar import pipeline
from hangar.report import html

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


@unittest.skipIf(fsim is None, "the fsim package is not importable")
class Stages(unittest.TestCase):
    def setUp(self):
        out = os.environ.get("FSIM_TEST_OUTPUT")
        if out:
            os.makedirs(out, exist_ok=True)
        self.base = tempfile.mkdtemp(prefix="hangar-", dir=out or None)
        self.name = "hangartest"      # not a name the platform knows otherwise
        folder = os.path.join(self.base, self.name)
        os.makedirs(folder)
        with open(os.path.join(ROOT, "aircraft", "skua", "skua.toml"), encoding="utf-8") as f:
            text = f.read().replace('name = "skua"', 'name = "%s"' % self.name, 1)
        self.toml = os.path.join(folder, self.name + ".toml")
        with open(self.toml, "w", encoding="utf-8") as f:
            f.write(text)
        self.old_path = os.environ.get("FSIM_AIRCRAFT_PATH")
        os.environ["FSIM_AIRCRAFT_PATH"] = self.base      # jsbsim:hangartest is found here

    def tearDown(self):
        if self.old_path is None:
            os.environ.pop("FSIM_AIRCRAFT_PATH", None)
        else:
            os.environ["FSIM_AIRCRAFT_PATH"] = self.old_path
        if not os.environ.get("FSIM_TEST_OUTPUT"):
            shutil.rmtree(self.base, ignore_errors=True)

    def stage(self, d, name, *args):
        r = getattr(d, name)(*args)
        self.assertTrue(os.path.isfile(os.path.join(d.out, name + ".json")), name)
        for img in r.get("images", []):
            self.assertTrue(os.path.isfile(d.img(img)), img)
        return r

    def failed(self, r):
        return [c["name"] for c in r.get("checks", []) if c["status"] == "fail"]

    @staticmethod
    def measured(d):
        """What the fly stage measured (out/fly.json's results), without the seconds each took."""
        with open(os.path.join(d.out, "fly.json"), encoding="utf-8") as f:
            results = json.load(f)["results"]
        for r in results.values():
            r.pop("seconds", None)
        return json.dumps(results, sort_keys=True)

    def test_every_stage(self):
        d = pipeline.Design(self.toml, log=lambda *a: None)
        d.aircraft.spec.setdefault("analysis", {})["quick"] = True
        self.assertEqual(self.failed(self.stage(d, "geometry")), [])
        aero = self.stage(d, "aero")
        self.assertEqual(self.failed(aero), [])
        self.assertLess(aero["derivatives"]["Cma"], 0.0)
        self.assertEqual(self.failed(self.stage(d, "mass")), [])
        self.assertEqual(self.failed(self.stage(d, "propulsion")), [])
        build = self.stage(d, "build")
        self.assertTrue(os.path.isfile(build["xml"]))
        # the files git keeps, with LF line ends as it keeps them
        for p in [build["xml"]] + glob.glob(os.path.join(d.dir, "Engines", "*.xml")):
            with open(p, "rb") as f:
                self.assertEqual(f.read().count(b"\r"), 0, p)
        # a rebuild that changes nothing leaves the aircraft as it was, date and all
        with open(build["xml"], encoding="utf-8", newline="") as f:
            built = f.read()
        today = re.search(r"<filecreationdate>(.*)</filecreationdate>", built).group(1)
        old = built.replace(today, "2000-01-01")
        with open(build["xml"], "w", encoding="utf-8", newline="\n") as f:
            f.write(old)
        self.stage(d, "build")
        with open(build["xml"], encoding="utf-8", newline="") as f:
            self.assertEqual(f.read(), old)
        # one that changes anything else writes it anew, dated today
        with open(build["xml"], "w", encoding="utf-8", newline="\n") as f:
            f.write(old.replace("</fdm_config>", "<!-- edited -->\n</fdm_config>"))
        self.stage(d, "build")
        with open(build["xml"], encoding="utf-8", newline="") as f:
            rebuilt = f.read()
        self.assertNotIn("<!-- edited -->", rebuilt)
        self.assertNotIn("2000-01-01", rebuilt)
        # the 3D model: one closed solid when the mesher is built, else primitives
        model = self.stage(d, "model")
        self.assertTrue(os.path.isfile(model["glb"]))
        self.assertEqual(self.failed(model), [])
        # meshed: slivers within bounds (the decimator collapses edges shorter
        # than 0.1 mm; before it, this airframe had 0.25 %)
        degenerate = [c for c in model["checks"] if c["name"] == "3D model: degenerate triangles"]
        for c in degenerate:
            self.assertEqual(c["status"], "pass", "%.3f %% (%s)" % (c["value"], c["note"]))
        self.assertTrue(os.path.isfile(os.path.join(d.dir, "Engines", self.name + "_engine0.xml")))
        # JSBSim flies exactly the tables
        self.assertEqual(self.failed(self.stage(d, "verify")), [])
        fly = self.stage(d, "fly")["results"]["design"]
        self.assertTrue(any(r["ok"] for r in fly["trim_sweep"]), "no trimmed level flight")
        self.assertGreater(fly["stall"]["stall_kcas"], 15.0)
        self.assertGreater(fly["climb"]["rows"][0]["rate_ms"], 1.0)
        self.assertEqual(fly["robustness"]["diverged"], 0)
        first = self.measured(d)
        # its plant identified: written beside the design and into the JSBSim
        # aircraft, and every vehicle of the type flies the loops the platform
        # designs from it
        from hangar import autopilot
        ap = autopilot.stage(d)
        self.assertTrue(os.path.isfile(os.path.join(d.out, "autopilot.json")))
        self.assertEqual(self.failed(ap), [])
        self.assertEqual(autopilot.load_settings(os.path.join(d.dir, "autopilot.toml")), {})
        reference, identified = autopilot.load_identification(os.path.join(d.dir, "autopilot.toml"))
        self.assertIn("lag_s", identified["roll"])
        self.assertIn("throttle_trim", identified)
        with open(build["xml"], encoding="utf-8") as f:
            xml = f.read()
        self.assertNotIn("fsim/control/", xml)   # no gains: the platform designs them
        # and its profile: the identification the stage's rebuild wrote in
        self.assertIn("fsim/plant/roll/tau_s", xml)
        self.assertIn("fsim/performance/stall_cas_ms", xml)
        w = fsim.World("hangar-autopilot-test", publish=False, workers=1)
        try:
            v = w.create_vehicle("t", type="jsbsim:" + self.name, altitude_msl_m=1000.0, airspeed_ms=25.0)
            self.assertEqual(v.profile_section("control"), (1, 4))  # designed from the plant (provenance: derived)
            self.assertAlmostEqual(v.controller_parameter(fsim.Level.ATTITUDE, "schedule.tas_ms"), reference["tas_ms"], places=9)
            self.assertAlmostEqual(v.controller_parameter(fsim.Level.ACCELERATION, "throttle.feedforward"), identified["throttle_trim"],
                                   places=9)
            self.assertEqual(v.profile_section("plant"), (1, 1))  # version 1, from hangar
            self.assertAlmostEqual(v.profile_value("plant/roll/tau_s"), identified["roll"]["lag_s"], places=5)
            self.assertEqual(v.profile_value("identity/family"), 1)
            # the stall the fly stage flew is the envelope now: protected, the elevator eases near it
            self.assertAlmostEqual(v.profile_value("envelope/clean/alpha_max_deg"), fly["stall"]["alpha_at_stall"], places=3)
            self.assertEqual(v.protection, fsim.ProtectionMode.LIMIT)
        finally:
            w.close()
        # the flight tests measure the aircraft, not that envelope (flight.Flight): flown
        # again on the profile the last run wrote, they find what they found
        self.stage(d, "fly")
        self.assertEqual(self.measured(d), first)
        self.assertTrue(os.path.isfile(html.write(d)))


@unittest.skipIf(fsim is None, "the fsim package is not importable")
class TurbopropStart(unittest.TestCase):
    """The platform starts every engine running (JsbsimModel: InitRunning),
    and JSBSim then marches the engines to their steady state in half-second
    steps - at a spawn, a reset and in its trim. A turboprop must come out
    of each with its propeller at the governed speed and its engine at the
    power it had, and fly on without a thrust transient; parked, it idles in
    its ground range. One build of a copy of the C-130J serves every test."""

    @classmethod
    def setUpClass(cls):
        out = os.environ.get("FSIM_TEST_OUTPUT")
        if out:
            os.makedirs(out, exist_ok=True)
        cls.base = tempfile.mkdtemp(prefix="hangar-tp-", dir=out or None)
        cls.name = "hangartp"
        folder = os.path.join(cls.base, cls.name)
        os.makedirs(folder)
        with open(os.path.join(ROOT, "aircraft", "c130j", "c130j.toml"), encoding="utf-8") as f:
            text = f.read().replace('name = "c130j"', 'name = "%s"' % cls.name, 1)
        cls.toml = os.path.join(folder, cls.name + ".toml")
        with open(cls.toml, "w", encoding="utf-8") as f:
            f.write(text)
        cls.old_path = os.environ.get("FSIM_AIRCRAFT_PATH")
        os.environ["FSIM_AIRCRAFT_PATH"] = cls.base
        cls.d = pipeline.Design(cls.toml, log=lambda *a: None)
        cls.d.aircraft.spec.setdefault("analysis", {})["quick"] = True
        for stage in ("aero", "mass", "propulsion", "build"):
            getattr(cls.d, stage)()

    @classmethod
    def tearDownClass(cls):
        if cls.old_path is None:
            os.environ.pop("FSIM_AIRCRAFT_PATH", None)
        else:
            os.environ["FSIM_AIRCRAFT_PATH"] = cls.old_path
        if not os.environ.get("FSIM_TEST_OUTPUT"):
            shutil.rmtree(cls.base, ignore_errors=True)

    def test_runs_from_spawn_reset_and_trim(self):
        d = self.d
        e = d.aircraft.engines[0]
        world = fsim.World("hangar-test-turboprop", publish=False, workers=1)
        try:
            v = world.create_vehicle("tp", type="jsbsim:" + self.name, latitude_deg=37.6, longitude_deg=-122.4,
                                     altitude_msl_m=3000.0, heading_deg=0.0, airspeed_ms=130.0)
            n = sum(len(x.copies()) for x in d.aircraft.engines)

            def fly(label, throttle, seconds=2.0):
                thrust, rpm = [], []
                for _ in range(int(seconds / world.step_seconds)):
                    v.command_actuator(throttle=throttle)
                    world.step()
                    thrust.append(sum(v.get_property("propulsion/engine[%d]/thrust-lbs" % i) for i in range(n)))
                    rpm += [v.get_property("propulsion/engine[%d]/propeller-rpm" % i) for i in range(n)]
                with self.subTest(label):
                    # the propellers at their governed speed, within 1 %
                    self.assertLess(max(abs(r / e.prop_rpm - 1.0) for r in rpm), 0.01, label)
                    # the thrust steady: no step of more than 5 % of it (the engines
                    # spool down to the throttle smoothly)
                    steps = [abs(b - a) / max(abs(a), 1.0) for a, b in zip(thrust, thrust[1:])]
                    self.assertLess(max(steps), 0.05, label)
                    self.assertGreater(min(thrust), 0.0, label)
                    self.assertGreater(v.get_property("propulsion/engine[0]/power-hp"), 100.0, label)
            fly("spawn", 0.6)
            v.reset()
            fly("reset", 0.6)
            v.set_property("simulation/do_simple_trim", 1)
            fly("trim", v.get_property("fcs/throttle-cmd-norm"))
        finally:
            world.close()

    def test_ground_idle(self):
        """The ground range (propulsion.py): parked at idle from its spawn on,
        the blades at ground idle make next to no thrust and the aircraft
        stands still, its engines holding the propellers' speed (it rolled
        away at 3,400 lbf a propeller, governed at flight idle on its low
        stop); braked, the thrust rises with the throttle through the ground
        range into the flight range's, without a step; rolling at idle the
        flat blades brake, the propellers held near their speed."""
        from hangar.propulsion import TP_BETA_THROTTLE
        e = self.d.aircraft.engines[0]
        n = sum(len(x.copies()) for x in self.d.aircraft.engines)
        world = fsim.World("hangar-test-turboprop-ground", publish=False, workers=1, terrain=False)
        try:
            def spawn(name, speed):
                return world.create_vehicle(name, type="jsbsim:" + self.name, latitude_deg=37.6, longitude_deg=-122.4,
                                            heading_deg=0.0, on_ground=True, airspeed_ms=speed)

            def fly(v, throttle, seconds, brakes=0.0):
                most, rpm = -math.inf, []
                for _ in range(int(seconds / world.step_seconds)):
                    v.command_actuator(throttle=throttle, brake_left=brakes, brake_right=brakes)
                    world.step()
                    most = max(most, sum(v.get_property("propulsion/engine[%d]/thrust-lbs" % i) for i in range(n)))
                    rpm += [v.get_property("propulsion/engine[%d]/propeller-rpm" % i) / e.prop_rpm for i in range(n)]
                s = v.state
                return most, math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1]), min(rpm), max(rpm)

            v = spawn("parked", 0.0)
            weight = v.get_property("inertia/weight-lbs")
            most, speed, lo, hi = fly(v, 0.0, 20.0)
            self.assertLess(most, 0.002 * weight)            # -1 lbf (was 46,500 falling to 14,000)
            self.assertLess(speed, 0.05)
            self.assertLess(max(1.0 - lo, hi - 1.0), 0.01)
            thrust = []
            for k in (0.25, 0.5, 0.75, 0.999, 1.0, 1.5):
                fly(v, k * TP_BETA_THROTTLE, 15.0, brakes=1.0)
                thrust.append(sum(v.get_property("propulsion/engine[%d]/thrust-lbs" % i) for i in range(n)))
            self.assertTrue(all(b > a > 0.0 for a, b in zip(thrust, thrust[1:])), thrust)
            self.assertLess(abs(thrust[4] / thrust[3] - 1.0), 0.02, thrust)
            v.remove()
            v = spawn("rolling", 50.0)
            fly(v, 0.0, 2.0)
            most, speed, lo, hi = fly(v, 0.0, 20.0)
            self.assertLess(most, 0.0)                      # the flat blades brake
            self.assertLess(speed, 47.0)
            self.assertLess(max(1.0 - lo, hi - 1.0), 0.04)  # the C-130's 96-103 %
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
