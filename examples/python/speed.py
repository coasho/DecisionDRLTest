"""What the Python SDK costs: the same loops through the C ABI and through
fsim, side by side, in fresh processes (python/benchmarks/overhead.py).

    fsim python examples\\python\\speed.py [--rounds 3]

Needs a build tree: the benchmark script and the C baseline it compares with.
"""
import os
import runpy
import sys

import fsim

here = os.path.dirname(os.path.abspath(__file__))
bench = os.path.normpath(os.path.join(here, "..", "..", "python", "benchmarks", "overhead.py"))
baseline = os.path.normpath(os.path.join(os.path.dirname(fsim.__file__), "..", "..", "bin", "fsim_python_baseline.exe"))
for needed in (bench, baseline):
    if not os.path.exists(needed):
        sys.exit("speed.py needs a build tree; missing %s" % needed)

sys.argv = [bench, baseline] + sys.argv[1:]
runpy.run_path(bench, run_name="__main__")
