#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <vector>

namespace fsim::world {

/// Per-vehicle flight-path trails (design 8.2 "Effects"): a line strip over the
/// last N sampled positions, fading out along its length. Positions are kept
/// relative to a per-trail origin so float vertices stay precise.
class Trails {
public:
    Trails(std::size_t vehicles, std::size_t points, double sampleIntervalS, vsg::ref_ptr<const vsg::Options> options);

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Append a sample per vehicle when `simTime` has advanced by the sample
    /// interval; rewrites the GPU arrays (cheap: N points per vehicle).
    void update(Span<const sim::VehicleState> states, double simTime);

    void setSelected(int index) { selected_ = index; }
    void setVisible(bool on) { root_->setAllChildren(on); }

private:
    struct Trail {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec4Array> colors;
        vsg::ref_ptr<vsg::VertexDraw> draw;
        std::vector<vsg::dvec3> history; ///< oldest .. newest
        vsg::dvec3 origin;
        bool originSet = false;
    };
    void rewrite(Trail& t, bool selected);

    vsg::ref_ptr<vsg::Switch> root_;
    std::vector<Trail> trails_;
    std::size_t points_;
    double interval_;
    double lastSample_ = -1.0;
    int selected_ = -1;
};

} // namespace fsim::world
