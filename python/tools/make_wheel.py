"""Build the fsim wheel from an installed package directory, stdlib only.

    python make_wheel.py <prefix>/fsim <version> <output dir>

Writes fsim-<version>-cp311-abi3-win_amd64.whl: the package as installed
(native modules, the platform's DLLs in bin/, its data in share/), with the
METADATA, WHEEL and RECORD pip needs. cp311-abi3: the native modules use
CPython's stable ABI, so one wheel installs on every CPython from 3.11 on.
"""
import base64
import hashlib
import os
import sys
import zipfile

TAG = "cp311-abi3-win_amd64"


def record_hash(data):
    digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode("ascii")
    return "sha256=" + digest


def main():
    package, version, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    package = os.path.abspath(package)
    dist_info = "fsim-%s.dist-info" % version
    wheel = os.path.join(out_dir, "fsim-%s-%s.whl" % (version, TAG))

    metadata = "\n".join([
        "Metadata-Version: 2.1",
        "Name: fsim",
        "Version: %s" % version,
        "Summary: Flight simulation platform for reinforcement learning (JSBSim, multi-level control, cameras)",
        "License: MIT",
        "Requires-Python: >=3.11",
        "Requires-Dist: numpy>=1.24",
        "Provides-Extra: gym",
        "Requires-Dist: gymnasium>=1.0; extra == \"gym\"",
        "Provides-Extra: sb3",
        "Requires-Dist: stable-baselines3>=2.0; extra == \"sb3\"",
        "",
        "The fsim platform's Python SDK: see docs/sdk/python.md.",
        "",
    ])
    wheel_file = "\n".join([
        "Wheel-Version: 1.0",
        "Generator: flightsim make_wheel.py",
        "Root-Is-Purelib: false",
        "Tag: %s" % TAG,
        "",
    ])

    records = []
    with zipfile.ZipFile(wheel, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for root, dirs, files in os.walk(package):
            dirs[:] = sorted(d for d in dirs if d != "__pycache__")
            for name in sorted(files):
                path = os.path.join(root, name)
                arc = "fsim/" + os.path.relpath(path, package).replace(os.sep, "/")
                with open(path, "rb") as f:
                    data = f.read()
                z.writestr(arc, data)
                records.append("%s,%s,%d" % (arc, record_hash(data), len(data)))
        for name, text in (("METADATA", metadata), ("WHEEL", wheel_file)):
            data = text.encode("utf-8")
            arc = "%s/%s" % (dist_info, name)
            z.writestr(arc, data)
            records.append("%s,%s,%d" % (arc, record_hash(data), len(data)))
        records.append("%s/RECORD,," % dist_info)
        z.writestr("%s/RECORD" % dist_info, "\n".join(records) + "\n")
    print("python: wrote %s (%d files)" % (wheel, len(records)))


if __name__ == "__main__":
    main()
