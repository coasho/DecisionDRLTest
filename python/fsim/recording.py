"""Recordings (docs/sdk/world.md): the .fsrec files a World writes with
``record_path``, read back frame by frame without copying."""
import numpy as np

from . import _native
from ._state import recorded_sample_dtype


class Recording:
    """``Recording(path)``: frames of samples (one per live vehicle: slot, state,
    control inputs) and vehicle events (created, removed, renamed)."""

    def __init__(self, path):
        self._h = _native.Recording(str(path))
        info = self._h.info()
        self.frame_count = info["frame_count"]
        self.world_name = info["world_name"]
        self.dt = info["dt"]
        self.frame_skip = info["frame_skip"]

    def __len__(self):
        return self.frame_count

    def frame_time(self, frame):
        """Simulation seconds of a frame."""
        return self._h.frame_time(frame)

    def samples(self, frame):
        """The frame's samples as a structured array (fsim.recorded_sample_dtype)
        viewing the recording's memory: ``s["state"]["altitude_msl_m"]``."""
        return np.frombuffer(self._h.samples(frame), dtype=recorded_sample_dtype)

    def events(self, frame):
        """Vehicle events of a frame, as dicts."""
        return self._h.events(frame)

    def __iter__(self):
        """(time, samples) for every frame."""
        for f in range(self.frame_count):
            yield self.frame_time(f), self.samples(f)
