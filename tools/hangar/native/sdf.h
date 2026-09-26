#pragma once

// Signed-distance scenes: primitives and the operators that join them.
//
// A scene is a tree. Its leaves are solids given by a signed distance
// (negative inside) - bodies lofted from superelliptic sections, wing
// segments lofted from airfoil sections, solids of revolution, capsules,
// boxes, ellipsoids, tori - and its inner nodes join them: unions (smooth
// ones make fillets), subtractions (openings, gaps) and intersections. Every
// solid carries a material, and a point takes the material of the solid it
// lies nearest to.
//
// The distances are exact where that is cheap and first-order estimates
// elsewhere; what matters is that the sign is right everywhere and the
// magnitude close near the surface, within the blend radii and a few cells
// of the mesher's grid.

#include "vec.h"

#include <memory>
#include <vector>

namespace fsim::core {
class Json;
}

namespace meshkit {

struct Sample {
    double d = 1e30;
    int material = 0;
};

class Node {
public:
    virtual ~Node() = default;
    virtual Sample eval(V3 p) const = 0;
    /// eval(p) where its distance is below cutoff; elsewhere any sample
    /// whose distance is at or above cutoff. An operator skips what cannot
    /// bring its distance below the cutoff - a cutter asked whether it
    /// reaches past the surface it cuts stops at its first part that does
    /// not; a primitive just evaluates.
    virtual Sample below(V3 p, double /*cutoff*/) const { return eval(p); }
    Box box; ///< holds the solid
};

using NodePtr = std::unique_ptr<Node>;

/// An airfoil's signed distance at unit chord, tabulated over X (0 leading
/// edge, 1 trailing edge) and Z (normal to the chord).
class FoilTable {
public:
    FoilTable(const std::vector<double>& x, const std::vector<double>& upper, const std::vector<double>& lower);
    double sdf(double X, double Z) const;

private:
    static constexpr double kX0 = -0.25, kX1 = 1.25, kZ0 = -0.3, kZ1 = 0.3, kStep = 0.0015;
    int nx_ = 0, nz_ = 0;
    std::vector<float> d_;
};

struct Scene {
    std::vector<std::unique_ptr<FoilTable>> foils;
    NodePtr root;
};

/// Builds a scene from its description (see meshkit.h); throws
/// std::runtime_error naming what is wrong.
Scene buildScene(const fsim::core::Json& description);

/// Polynomial smooth minimum and maximum over a radius k (0: plain min/max).
inline double smin(double a, double b, double k) {
    if (k <= 0.0) return std::min(a, b);
    const double h = std::max(k - std::fabs(a - b), 0.0) / k;
    return std::min(a, b) - h * h * k * 0.25;
}
inline double smax(double a, double b, double k) { return -smin(-a, -b, k); }

} // namespace meshkit
