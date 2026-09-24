"""The C ABI's state structs, mirrored for zero-copy access.

A ``VehicleState`` is a ctypes structure laid over the platform's own
snapshot of a vehicle: attribute access reads that memory directly, and the
snapshot is rewritten in place by every step. The same layouts as numpy
dtypes let many states be gathered into one structured array in one call.
The mirrors are checked against the loaded library's layout at import, so a
mismatch fails loudly instead of reading the wrong bytes.
"""
import ctypes
import math

import numpy as np

from . import _native

_d = ctypes.c_double
_MAX_ENGINES = 4


class VehicleState(ctypes.Structure):
    """A vehicle's state (fsim_vehicle_state; fsim::VehicleState in C++).

    SI units and radians; WGS-84 geodetic latitude/longitude; altitudes above
    the ellipsoid; body frame x forward, y right, z down; NED velocities.
    """

    _fields_ = [
        ("sim_time", _d),
        ("position_ecef", _d * 3),
        ("attitude_ecef_to_body", _d * 4),  # quaternion w, x, y, z
        ("latitude_rad", _d),
        ("longitude_rad", _d),
        ("altitude_msl_m", _d),
        ("altitude_agl_m", _d),
        ("euler_rad", _d * 3),  # roll, pitch, yaw
        ("velocity_body_ms", _d * 3),
        ("velocity_ned_ms", _d * 3),
        ("angular_rate_body_rad_s", _d * 3),
        ("acceleration_body_ms2", _d * 3),
        ("airspeed_true_ms", _d),
        ("airspeed_calibrated_ms", _d),
        ("mach", _d),
        ("alpha_rad", _d),
        ("beta_rad", _d),
        ("load_factor", _d),
        ("aileron_rad", _d),
        ("elevator_rad", _d),
        ("rudder_rad", _d),
        ("flaps_rad", _d),
        ("gear_position", _d),
        ("engine_count", ctypes.c_int32),
        ("throttle_position", _d * _MAX_ENGINES),
        ("thrust_n", _d * _MAX_ENGINES),
        ("fuel_kg", _d),
        ("step_count", ctypes.c_uint32),
        ("on_ground", ctypes.c_uint8),
        ("diverged", ctypes.c_uint8),
        ("rotation_body_to_ecef", _d * 9),
        ("engine_rpm", _d * _MAX_ENGINES),  # propeller rpm; 0 for a jet
        ("engine_n2", _d * _MAX_ENGINES),  # turbine core speed, %; 0 otherwise
        ("afterburner", _d * _MAX_ENGINES),  # 0 off .. 1 full
        ("nozzle_position", _d * _MAX_ENGINES),  # 0 shut .. 1 wide open
        ("leading_edge_flap_rad", _d),  # + leading edge down
    ]

    @property
    def latitude_deg(self):
        return math.degrees(self.latitude_rad)

    @property
    def longitude_deg(self):
        return math.degrees(self.longitude_rad)

    @property
    def roll_rad(self):
        return self.euler_rad[0]

    @property
    def pitch_rad(self):
        return self.euler_rad[1]

    @property
    def heading_rad(self):
        """Yaw, radians from true north."""
        return self.euler_rad[2]

    def __repr__(self):
        return (
            "VehicleState(t=%.2f s, lat=%.5f, lon=%.5f, alt=%.1f m, roll=%.1f, pitch=%.1f, hdg=%.1f deg, tas=%.1f m/s)"
            % (
                self.sim_time,
                self.latitude_deg,
                self.longitude_deg,
                self.altitude_msl_m,
                math.degrees(self.euler_rad[0]),
                math.degrees(self.euler_rad[1]),
                math.degrees(self.euler_rad[2]) % 360.0,
                self.airspeed_true_ms,
            )
        )


class ControlInputs(ctypes.Structure):
    """Actuator inputs the control cascade produced (fsim_control_inputs)."""

    _fields_ = [
        ("aileron", _d),
        ("elevator", _d),
        ("rudder", _d),
        ("throttle", _d * _MAX_ENGINES),
        ("flaps", _d),
        ("gear_down", _d),
        ("brake_left", _d),
        ("brake_right", _d),
    ]


class RecordedSample(ctypes.Structure):
    """One vehicle in one frame of a recording (fsim_recorded_sample)."""

    _fields_ = [("slot", ctypes.c_uint32), ("state", VehicleState), ("inputs", ControlInputs)]


vehicle_state_dtype = np.dtype(VehicleState)
control_inputs_dtype = np.dtype(ControlInputs)
recorded_sample_dtype = np.dtype(RecordedSample)


def _check_layout():
    layout = _native.layout()
    for struct, name in ((VehicleState, "vehicle_state"), (ControlInputs, "control_inputs"), (RecordedSample, "recorded_sample")):
        expected = layout[name]
        if ctypes.sizeof(struct) != expected["size"]:
            raise ImportError("fsim: %s is %d bytes here but %d in the library" % (name, ctypes.sizeof(struct), expected["size"]))
        for field, offset in expected.items():
            if field != "size" and getattr(struct, field).offset != offset:
                raise ImportError("fsim: %s.%s is at %d here but %d in the library" % (name, field, getattr(struct, field).offset, offset))


_check_layout()
