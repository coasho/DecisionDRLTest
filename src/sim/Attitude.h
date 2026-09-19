#pragma once

#include <algorithm>
#include <cmath>

namespace fsim::sim::attitude {

/// Quaternion helpers on (w, x, y, z) arrays as stored in VehicleState.

/// Spherical interpolation along the short arc; `out` is normalised.
inline void slerp(const double* a, const double* b, double t, double* out) noexcept {
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    double sign = 1.0;
    if (dot < 0.0) {
        dot = -dot;
        sign = -1.0;
    }
    double wa, wb;
    if (dot > 0.9995) {
        wa = 1.0 - t;
        wb = t;
    } else {
        const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
        const double s = std::sin(theta);
        wa = std::sin((1.0 - t) * theta) / s;
        wb = std::sin(t * theta) / s;
    }
    double n = 0.0;
    for (int i = 0; i < 4; ++i) {
        out[i] = wa * a[i] + sign * wb * b[i];
        n += out[i] * out[i];
    }
    n = std::sqrt(n);
    if (n > 0.0)
        for (int i = 0; i < 4; ++i) out[i] /= n;
}

/// Body -> ECEF rotation (row-major 3x3) from the ECEF -> body quaternion,
/// matching JSBSim: Tec2b is the rotation matrix of q, Tb2ec its transpose.
inline void bodyToEcefFromQuaternion(const double* q, double* R) noexcept {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double m00 = 1 - 2 * (y * y + z * z), m01 = 2 * (x * y + w * z), m02 = 2 * (x * z - w * y);
    const double m10 = 2 * (x * y - w * z), m11 = 1 - 2 * (x * x + z * z), m12 = 2 * (y * z + w * x);
    const double m20 = 2 * (x * z + w * y), m21 = 2 * (y * z - w * x), m22 = 1 - 2 * (x * x + y * y);
    R[0] = m00; R[1] = m10; R[2] = m20; // transpose of [m]
    R[3] = m01; R[4] = m11; R[5] = m21;
    R[6] = m02; R[7] = m12; R[8] = m22;
}

} // namespace fsim::sim::attitude
