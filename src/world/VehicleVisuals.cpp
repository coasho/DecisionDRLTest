#include "world/VehicleVisuals.h"

#include "core/Log.h"
#include "world/Frames.h"

#include <fstream>
#include <sstream>

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

    vsg::ref_ptr<vsg::Node> model, highlightModel;
    if (!settings.modelPath.empty()) model = loadModel(settings, options);
    if (model) {
        highlightModel = model; // a loaded model keeps its look; the label marks the selection
    } else {
        model = buildPlaceholder(settings, vsg::vec4(0.85f, 0.85f, 0.9f, 1.0f));
        highlightModel = buildPlaceholder(settings, vsg::vec4(1.0f, 0.55f, 0.1f, 1.0f));
    }

    transforms_.reserve(count);
    highlight_.reserve(count);
    visible_.assign(count, 1);
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

bool VehicleVisuals::applyManifest(Settings& settings) {
    if (settings.modelPath.empty()) return false;
    std::ifstream in(settings.modelPath + ".manifest");
    if (!in) return false;
    auto axis = [](const std::string& a, vsg::dvec3& out) {
        if (a.size() != 2) return;
        const double sgn = a[0] == '-' ? -1.0 : 1.0;
        if (a[1] == 'x') out = vsg::dvec3(sgn, 0.0, 0.0);
        else if (a[1] == 'y') out = vsg::dvec3(0.0, sgn, 0.0);
        else if (a[1] == 'z') out = vsg::dvec3(0.0, 0.0, sgn);
    };
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string key, value;
        if (!(ls >> key >> value) || key.empty() || key[0] == '#') continue;
        if (key == "forward") axis(value, settings.modelForward);
        else if (key == "up") axis(value, settings.modelUp);
        else if (key == "scale") settings.modelScale = std::stod(value);
        else LOG_WARN("world") << "manifest: unknown key '" << key << "' in " << settings.modelPath << ".manifest";
    }
    LOG_INFO("world") << "model manifest: " << settings.modelPath << ".manifest";
    return true;
}

vsg::ref_ptr<vsg::Node> VehicleVisuals::loadModel(const Settings& s, vsg::ref_ptr<vsg::Options> options) const {
    auto node = vsg::read_cast<vsg::Node>(s.modelPath, options);
    if (!node) {
        LOG_WARN("world") << "could not load vehicle model '" << s.modelPath << "', using placeholder";
        return {};
    }
    // Model axes -> body axes (x fwd, y right, z down). Body x maps to the model's
    // forward vector, body z to minus its up vector, body y = down x forward
    // completes a right-handed frame. Columns of bodyToModel are those vectors.
    const vsg::dvec3 f = vsg::normalize(s.modelForward);
    const vsg::dvec3 d = -vsg::normalize(s.modelUp);
    const vsg::dvec3 r = vsg::normalize(vsg::cross(d, f));
    vsg::dmat4 bodyToModel(f.x, f.y, f.z, 0.0,   // column 0: body x in model space
                           r.x, r.y, r.z, 0.0,   // column 1: body y
                           d.x, d.y, d.z, 0.0,   // column 2: body z
                           0.0, 0.0, 0.0, 1.0);
    const vsg::dmat4 modelToBody = vsg::inverse(bodyToModel);
    auto xf = vsg::MatrixTransform::create(modelToBody * vsg::scale(s.modelScale, s.modelScale, s.modelScale));
    xf->addChild(node);

    // Animated parts (propellers, ...): collected so the viewer can play them.
    auto finder = vsg::visit<vsg::FindAnimations>(node);
    for (auto& anim : finder.animations) anim->mode = vsg::Animation::REPEAT;
    const_cast<VehicleVisuals*>(this)->animations_ = finder.animations;
    LOG_INFO("world") << "vehicle model: " << s.modelPath << " (" << finder.animations.size() << " animation(s))";
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

void VehicleVisuals::applySwitch(std::size_t index) {
    auto& sw = highlight_[index];
    sw->setAllChildren(false);
    if (visible_[index]) sw->setSingleChildOn(static_cast<int>(index) == selected_ ? 1 : 0);
}

void VehicleVisuals::setSelected(int index) {
    if (selected_ == index) return;
    const int previous = selected_;
    selected_ = index;
    if (previous >= 0 && static_cast<std::size_t>(previous) < highlight_.size()) applySwitch(static_cast<std::size_t>(previous));
    if (selected_ >= 0 && static_cast<std::size_t>(selected_) < highlight_.size()) applySwitch(static_cast<std::size_t>(selected_));
}

void VehicleVisuals::setVisible(std::size_t index, bool visible) {
    if (index >= visible_.size() || (visible_[index] != 0) == visible) return;
    visible_[index] = visible ? 1 : 0;
    applySwitch(index);
}

} // namespace fsim::world
