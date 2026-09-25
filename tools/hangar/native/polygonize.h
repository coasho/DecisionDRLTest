#pragma once

// A scene's zero surface as a closed triangle mesh: the grid is refined only
// where the surface can pass (a cell whose centre lies further from it than
// its half-diagonal holds none), and the cells that remain are cut into six
// tetrahedra each (Kuhn's split, the same diagonal everywhere), whose
// marching - unlike marching cubes' - has no ambiguous cases: the mesh is
// closed and manifold wherever the samples are.

#include "sdf.h"

#include <array>
#include <cstdint>
#include <vector>

namespace meshkit {

struct Mesh {
    std::vector<V3> v;
    std::vector<std::array<std::uint32_t, 3>> f;
};

struct PolygonizeOptions {
    double cell = 0.01;    ///< leaf cell size, m
    double safety = 2.0;   ///< how far past its half-diagonal a cell is still refined
    unsigned threads = 0;  ///< 0: all cores; the mesh is the same on any number
};

Mesh polygonize(const Node& root, const Box& domain, const PolygonizeOptions& options);

} // namespace meshkit
