"""Every stage in order on a copy of a design, through the platform's own
JSBSim: the tool works end to end. Quick mode (coarse tables, short flight
tests), so the numbers are a first look; what is asserted is that each stage
produces its outputs and that the aircraft it builds flies. Needs the fsim
package on the path; skipped without it."""
import glob
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
        self.assertTrue(os.path.isfile(html.write(d)))


if __name__ == "__main__":
    unittest.main()
