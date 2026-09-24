"""hangar: design, analyse and validate aircraft for JSBSim.

    python -m hangar <design> [stage ...]      run stages (default: all)
    python -m hangar <design> --quick          the same, coarse and short: a first look
    python -m hangar new <name> [--like c172]  start a design from another
    python -m hangar list                      the designs in aircraft/
    python -m hangar register                  the viewer's stand-ins for stock aircraft (aircraft/models.txt)

<design> is a name (aircraft/<name>/<name>.toml) or a path to a .toml.
Stages: geometry aero mass propulsion build verify fly report (in that order;
"all" runs these), and calibrate: fit extra drag and propeller pitch to the
[targets] (after fly; writes calibration.toml, rebuilds). Outputs: aircraft/<name>/out/ - report.html, the
stage JSON files and the images; the product - <name>.xml, Engines/, the .glb
model - in aircraft/<name>/, where the platform finds it as jsbsim:<name>.
"""
import argparse
import os
import shutil
import sys

from . import pipeline


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "list":
        root = os.path.join(pipeline.repo_root(), "aircraft")
        for name in sorted(os.listdir(root)):
            toml = os.path.join(root, name, name + ".toml")
            if os.path.isfile(toml):
                built = os.path.isfile(os.path.join(root, name, name + ".xml"))
                print("%-16s %s" % (name, "built" if built else "not built"))
        return 0
    if argv and argv[0] == "register":
        from . import register
        root = os.path.join(pipeline.repo_root(), "aircraft")
        rows = register.register(root)
        print("%s: %d stand-in(s)" % (os.path.join(root, "models.txt"), len(rows)))
        return 0
    if argv and argv[0] == "new":
        p = argparse.ArgumentParser(prog="hangar new")
        p.add_argument("name")
        p.add_argument("--like", default="c172", help="design to start from")
        a = p.parse_args(argv[1:])
        root = os.path.join(pipeline.repo_root(), "aircraft")
        src = os.path.join(root, a.like, a.like + ".toml")
        dst_dir = os.path.join(root, a.name)
        if os.path.exists(dst_dir):
            raise SystemExit("aircraft/%s exists already" % a.name)
        os.makedirs(dst_dir)
        with open(src, encoding="utf-8") as f:
            text = f.read().replace('name = "%s"' % a.like, 'name = "%s"' % a.name, 1)
        with open(os.path.join(dst_dir, a.name + ".toml"), "w", encoding="utf-8") as f:
            f.write(text)
        print("aircraft/%s/%s.toml (from %s): edit it, then python -m hangar %s geometry" % (a.name, a.name, a.like, a.name))
        return 0
    p = argparse.ArgumentParser(prog="hangar", description=__doc__.splitlines()[0])
    p.add_argument("design")
    p.add_argument("stages", nargs="*", default=["all"])
    p.add_argument("--reference", help="a JSBSim aircraft to fly the same tests on (e.g. jsbsim:c172x)")
    p.add_argument("--force", action="store_true", help="rebuild the aerodynamic tables even if cached")
    p.add_argument("--quick", action="store_true", help="coarse tables and short flight tests: a first look")
    a = p.parse_args(argv)
    stages = list(pipeline.DEFAULT) if a.stages == ["all"] else a.stages
    for s in stages:
        if s not in pipeline.STAGES:
            raise SystemExit("unknown stage %r (stages: %s)" % (s, " ".join(pipeline.STAGES)))
    path = pipeline.find_design(a.design)
    d, done = pipeline.run(path, stages, reference=a.reference, force=a.force, quick=a.quick)
    if "report" in done:
        print("report: %s" % done["report"]["path"])
    fails = sum(1 for r in done.values() for c in r.get("checks", []) if c["status"] == "fail")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
