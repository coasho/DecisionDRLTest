#pragma once

// Simplification by edge collapse ordered by quadric error (Garland &
// Heckbert, SIGGRAPH 1997): each vertex carries the planes of the triangles
// it has absorbed, and the cheapest collapse goes first. Edges between
// materials and sharp edges (a trailing edge, a chine) add planes across
// them, so they stay where they are; a collapse that would pinch the surface
// (the link condition) or fold a triangle over is refused.

#include "polygonize.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace meshkit {

struct DecimateOptions {
    double maxError = 0.001;      ///< m: the RMS distance to the planes a vertex stands for
    std::size_t targetFaces = 0;  ///< stop at this many triangles (0: the error decides)
    double sharpDeg = 50.0;       ///< dihedral past which an edge is a feature
    double featureWeight = 400.0; ///< of the planes that hold features
    bool blocks = true;           ///< a parallel first pass on large meshes
    bool serial = true;           ///< the final pass over the whole mesh
};

/// Simplifies m in place; material holds one entry per triangle and follows it.
void decimate(Mesh& m, std::vector<std::uint16_t>& material, const DecimateOptions& options);

} // namespace meshkit
