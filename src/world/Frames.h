#pragma once

#include "sim/VehicleState.h"

#include <cstddef>

#include <vsg/maths/mat4.h>
#include <vsg/maths/vec3.h>

namespace fsim::world {

/// Conversions between VehicleState (ECEF metres, body x-fwd/y-right/z-down)
/// and VSG's double-precision matrices. VSG matrices are column-major and
/// `m(col, row)`; a vsg::dmat4 built here maps body-frame points to ECEF.

inline vsg::dvec3 positionEcef(const sim::VehicleState& s) noexcept {
    return vsg::dvec3(s.positionEcef[0], s.positionEcef[1], s.positionEcef[2]);
}

/// Body axis `i` (0 = forward, 1 = right, 2 = down) expressed in ECEF.
inline vsg::dvec3 bodyAxisEcef(const sim::VehicleState& s, unsigned i) noexcept {
    const double* R = s.rotationBodyToEcef; // row-major, columns are body axes
    return vsg::dvec3(R[0 * 3 + i], R[1 * 3 + i], R[2 * 3 + i]);
}

/// Body -> ECEF rigid transform (rotation + translation).
inline vsg::dmat4 bodyToEcef(const sim::VehicleState& s) noexcept {
    const double* R = s.rotationBodyToEcef;
    vsg::dmat4 m;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c) m(c, r) = R[r * 3 + c];
    m(3, 0) = s.positionEcef[0];
    m(3, 1) = s.positionEcef[1];
    m(3, 2) = s.positionEcef[2];
    m(0, 3) = m(1, 3) = m(2, 3) = 0.0;
    m(3, 3) = 1.0;
    return m;
}

/// Local "up" at a position: the ellipsoid normal is close enough to the
/// geocentric direction for camera purposes.
inline vsg::dvec3 localUp(const vsg::dvec3& ecef) noexcept { return vsg::normalize(ecef); }

} // namespace fsim::world
