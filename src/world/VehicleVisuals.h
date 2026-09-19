#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <string>
#include <vector>

namespace fsim::world {

/// One shared model, N vsg::MatrixTransforms (design 8.2 "Many vehicles").
/// The model is a glTF file when given, else a procedural placeholder built in
/// the JSBSim body frame (x forward, y right, z down) so no axis conversion is
/// needed for the placeholder; glTF models get a fixed model->body rotation.
class VehicleVisuals {
public:
    struct Settings {
        std::string modelPath;           ///< glTF/OBJ path; empty = placeholder
        double modelScale = 1.0;
        double placeholderLengthM = 8.3; ///< c172-ish
        double placeholderSpanM = 11.0;
    };

    VehicleVisuals(std::size_t count, const Settings& settings, vsg::ref_ptr<vsg::Options> options);

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Write the latest snapshot into the transforms (render thread, per frame).
    void update(Span<const sim::VehicleState> states);

    /// Highlight one vehicle (or none with -1).
    void setSelected(int index);
    int selected() const noexcept { return selected_; }

    std::size_t count() const { return transforms_.size(); }

private:
    vsg::ref_ptr<vsg::Node> buildPlaceholder(const Settings& s, const vsg::vec4& color) const;
    vsg::ref_ptr<vsg::Node> loadModel(const Settings& s, vsg::ref_ptr<vsg::Options> options) const;

    vsg::ref_ptr<vsg::Group> root_;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> transforms_;
    std::vector<vsg::ref_ptr<vsg::Switch>> highlight_;
    int selected_ = -1;
};

} // namespace fsim::world
