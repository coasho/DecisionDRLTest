#include "world/VehicleVisuals.h"

#include "core/Log.h"
#include "world/Frames.h"

namespace fsim::world {

namespace {

vsg::ref_ptr<vsg::Node> box(vsg::Builder& builder, const vsg::vec3& centre, const vsg::vec3& size, const vsg::vec4& color) {
    vsg::GeometryInfo geom;
    geom.position = centre;
    geom.dx = vsg::vec3(size.x, 0.0f, 0.0f);
    geom.dy = vsg::vec3(0.0f, size.y, 0.0f);
    geom.dz = vsg::vec3(0.0f, 0.0f, size.z);
    geom.color = color;
    vsg::StateInfo state;
    state.lighting = true;
    return builder.createBox(geom, state);
}

} // namespace

VehicleVisuals::VehicleVisuals(std::size_t count, const Settings& settings, vsg::ref_ptr<vsg::Options> options) {
    root_ = vsg::Group::create();

    vsg::ref_ptr<vsg::Node> model;
    if (!settings.modelPath.empty()) model = loadModel(settings, options);
    if (!model) model = buildPlaceholder(settings, vsg::vec4(0.85f, 0.85f, 0.9f, 1.0f));
    auto highlightModel = buildPlaceholder(settings, vsg::vec4(1.0f, 0.55f, 0.1f, 1.0f));

    transforms_.reserve(count);
    highlight_.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        auto transform = vsg::MatrixTransform::create();
        // Shared subgraph under N transforms: one copy of the geometry on the GPU.
        auto sw = vsg::Switch::create();
        sw->addChild(true, model);
        sw->addChild(false, highlightModel);
        transform->addChild(sw);
        root_->addChild(transform);
        transforms_.push_back(transform);
        highlight_.push_back(sw);
    }
}

vsg::ref_ptr<vsg::Node> VehicleVisuals::loadModel(const Settings& s, vsg::ref_ptr<vsg::Options> options) const {
    auto node = vsg::read_cast<vsg::Node>(s.modelPath, options);
    if (!node) {
        LOG_WARN("world") << "could not load vehicle model '" << s.modelPath << "', using placeholder";
        return {};
    }
    // glTF convention (+Y up, -Z forward, +X right) -> body (x fwd, y right, z down).
    // Rows of the matrix express body axes in model coordinates.
    vsg::dmat4 modelToBody(0.0, 0.0, -1.0, 0.0,   // model +X -> body +Y (right)
                           0.0, -1.0, 0.0, 0.0,   // model +Y (up) -> body -Z
                           -1.0, 0.0, 0.0, 0.0,   // model +Z (back) -> body -X
                           0.0, 0.0, 0.0, 1.0);
    auto xf = vsg::MatrixTransform::create(vsg::scale(s.modelScale, s.modelScale, s.modelScale) * modelToBody);
    xf->addChild(node);
    LOG_INFO("world") << "vehicle model: " << s.modelPath;
    return xf;
}

vsg::ref_ptr<vsg::Node> VehicleVisuals::buildPlaceholder(const Settings& s, const vsg::vec4& color) const {
    // Simple aircraft silhouette in the body frame: x forward, y right, z down.
    vsg::Builder builder;
    const float L = static_cast<float>(s.placeholderLengthM * s.modelScale);
    const float span = static_cast<float>(s.placeholderSpanM * s.modelScale);
    const vsg::vec4 dark(color.r * 0.6f, color.g * 0.6f, color.b * 0.6f, 1.0f);

    auto group = vsg::Group::create();
    group->addChild(box(builder, vsg::vec3(0.0f, 0.0f, 0.0f), vsg::vec3(L, L * 0.14f, L * 0.16f), color));          // fuselage
    group->addChild(box(builder, vsg::vec3(L * 0.05f, 0.0f, -L * 0.02f), vsg::vec3(L * 0.18f, span, L * 0.02f), color)); // wing
    group->addChild(box(builder, vsg::vec3(-L * 0.42f, 0.0f, 0.0f), vsg::vec3(L * 0.12f, span * 0.32f, L * 0.015f), dark)); // tailplane
    group->addChild(box(builder, vsg::vec3(-L * 0.42f, 0.0f, -L * 0.12f), vsg::vec3(L * 0.12f, L * 0.015f, L * 0.18f), dark)); // fin (up = -z)
    group->addChild(box(builder, vsg::vec3(L * 0.5f, 0.0f, 0.0f), vsg::vec3(L * 0.06f, L * 0.06f, L * 0.06f), dark));  // nose
    return group;
}

void VehicleVisuals::update(Span<const sim::VehicleState> states) {
    const std::size_t n = std::min(states.size(), transforms_.size());
    for (std::size_t i = 0; i < n; ++i) transforms_[i]->matrix = bodyToEcef(states[i]);
}

void VehicleVisuals::setSelected(int index) {
    if (selected_ == index) return;
    if (selected_ >= 0 && static_cast<std::size_t>(selected_) < highlight_.size()) {
        highlight_[static_cast<std::size_t>(selected_)]->setAllChildren(false);
        highlight_[static_cast<std::size_t>(selected_)]->setSingleChildOn(0);
    }
    selected_ = index;
    if (selected_ >= 0 && static_cast<std::size_t>(selected_) < highlight_.size()) {
        highlight_[static_cast<std::size_t>(selected_)]->setSingleChildOn(1);
    }
}

} // namespace fsim::world
