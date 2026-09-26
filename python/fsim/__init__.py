"""fsim: the flight simulation platform's Python SDK.

The same platform as the C++ SDK - libfsim.dll, JSBSim vehicles, the
multi-level control stack, effects, communication, the batch layer and
cameras - through a native module built on the C ABI, with the cost of
Python kept out of the way:

* results are numpy arrays that *view* the platform's memory: nothing is
  copied per step, and a VecEnv step is one native call;
* many-vehicle operations are one call (``World.states``, ``World.command``);
* the GIL is released while the platform simulates or renders.

    import fsim
    world = fsim.World("my-experiment")
    red = world.create_vehicle("red-1", latitude_deg=37.62, longitude_deg=-122.38,
                               altitude_msl_m=1500, heading_deg=90, airspeed_ms=60)
    red.command_velocity(airspeed_ms=65, vertical_speed_ms=2, heading_rad=0.0)
    for _ in range(3000):
        world.step()
        s = red.state                 # live, zero-copy: s.altitude_msl_m, s.euler_rad[0], ...

See docs/sdk/python.md.
"""
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))

# The platform's DLLs live in bin/ beside this file (a package) or wherever
# FSIM_DLL_DIR points (a development tree). Python 3.8+ no longer searches
# PATH for an extension module's dependencies, so they are named here.
_dll_directories = []
if _sys.platform == "win32":
    for _dir in (_os.environ.get("FSIM_DLL_DIR"), _os.path.join(_HERE, "bin")):
        if _dir and _os.path.isdir(_dir):
            _dll_directories.append(_os.add_dll_directory(_dir))

from . import _native  # noqa: E402

Error = _native.Error
HOLD = _native.HOLD

_LOG_LEVELS = {"trace": 0, "debug": 1, "info": 2, "warn": 3, "warning": 3, "error": 4, "off": 5}


def set_log_level(level):
    """The platform's own log, which goes to stderr: "trace", "debug", "info",
    "warning", "error" or "off". Quiet by default in Python ("warning"); the
    FSIM_LOG_LEVEL environment variable sets the starting level."""
    value = _LOG_LEVELS[level.lower()] if isinstance(level, str) else int(level)
    _native.set_log_level(value)
    vision = _sys.modules.get(__name__ + "._vision")
    if vision is not None:
        vision.set_log_level(value)


set_log_level(_os.environ.get("FSIM_LOG_LEVEL", "warning"))

from ._state import (  # noqa: E402
    ControlInputs,
    RecordedSample,
    VehicleState,
    control_inputs_dtype,
    recorded_sample_dtype,
    vehicle_state_dtype,
)
from .world import (  # noqa: E402
    COMMAND_DEFAULTS,
    COMMAND_FIELDS,
    SUPPORT_FIELDS,
    SUPPORT_KINDS,
    Activity,
    ActivityInfo,
    ActivityState,
    Availability,
    Capability,
    Level,
    Message,
    Parameter,
    RangePolicy,
    Rejected,
    Scenario,
    Source,
    Vehicle,
    World,
)
from .vecenv import VecEnv  # noqa: E402
from .recording import Recording  # noqa: E402
from ._threads import torch_threads  # noqa: E402

__version__, abi_version = _native.version()


def tasks():
    """Task ids a VecEnv can be given (built-ins and registered ones)."""
    return _native.registered_ids(_native.REGISTRY_TASK)


def observations():
    """Observation ids a VecEnv can be given."""
    return _native.registered_ids(_native.REGISTRY_OBSERVATION)


def actions():
    """Action ids a VecEnv can be given."""
    return _native.registered_ids(_native.REGISTRY_ACTION)


_plugins = []  # loaded for good: the registries hold their code


def load_plugin(path):
    """Load a C++ plugin: a DLL that links the SDK and registers tasks,
    observations, actions or controllers from its static initialisers
    (examples/python/plugin). They land in the registries of the libfsim.dll
    this package runs, so a VecEnv created afterwards can name them. Build it
    with the platform's toolchain (MSYS2 UCRT64 GCC) against this version of
    the SDK: the plugin interface is C++.

    Returns the ids the plugin added, as a dict of "tasks", "observations"
    and "actions" (an id it replaced is not new, and loading the same DLL
    again adds nothing). The plugin stays loaded until the process ends."""
    import ctypes

    before = tasks(), observations(), actions()
    _plugins.append(ctypes.CDLL(_os.path.abspath(_os.fspath(path))))
    after = tasks(), observations(), actions()
    return {kind: sorted(set(a) - set(b)) for kind, b, a in zip(("tasks", "observations", "actions"), before, after)}


__all__ = [
    "COMMAND_DEFAULTS",
    "COMMAND_FIELDS",
    "ControlInputs",
    "Error",
    "HOLD",
    "Level",
    "Message",
    "RecordedSample",
    "Recording",
    "Scenario",
    "VecEnv",
    "Vehicle",
    "VehicleState",
    "World",
    "abi_version",
    "actions",
    "control_inputs_dtype",
    "load_plugin",
    "observations",
    "recorded_sample_dtype",
    "set_log_level",
    "tasks",
    "torch_threads",
    "vehicle_state_dtype",
]
