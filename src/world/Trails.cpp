#include "world/Trails.h"

#include "world/FlatGeometry.h"
#include "world/Frames.h"

#include <algorithm>

namespace fsim::world {

Trails::Trails(std::size_t vehicles, std::size_t points, double sampleIntervalS, vsg::ref_ptr<const vsg::Options> options)
    : points_(std::max<std::size_t>(points, 2)), interval_(sampleIntervalS) {
    root_ = vsg::Switch::create();

    FlatGeometrySettings settings;
    settings.topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
    settings.blending = true;
    settings.depthWrite = false; // translucent lines never occlude
    settings.cullBackFaces = false;
    auto state = createFlatStateGroup(settings, options);
    if (!state) return;

    trails_.resize(vehicles);
    for (auto& t : trails_) {
        t.vertices = vsg::vec3Array::create(static_cast<std::uint32_t>(points_), vsg::vec3(0.0f, 0.0f, 0.0f));
        t.colors = vsg::vec4Array::create(static_cast<std::uint32_t>(points_), vsg::vec4(1.0f, 1.0f, 1.0f, 0.0f));
        t.draw = createFlatDraw(t.vertices, t.colors, /*dynamic=*/true);
        t.draw->vertexCount = 0; // nothing until two samples exist
        t.transform = vsg::MatrixTransform::create();
        auto group = vsg::StateGroup::create();
        group->stateCommands = state->stateCommands; // share the pipeline binding
        group->prototypeArrayState = state->prototypeArrayState;
        group->addChild(t.draw);
        t.transform->addChild(group);
        root_->addChild(true, t.transform);
        t.history.reserve(points_);
    }
}

void Trails::update(Span<const sim::VehicleState> states, double simTime) {
    if (trails_.empty()) return;
    if (lastSample_ >= 0.0 && simTime - lastSample_ < interval_) return;
    if (simTime < lastSample_) { // simulation was reset: start over
        for (auto& t : trails_) { t.history.clear(); t.originSet = false; }
    }
    lastSample_ = simTime;

    const std::size_t n = std::min(states.size(), trails_.size());
    for (std::size_t i = 0; i < n; ++i) {
        Trail& t = trails_[i];
        if (!t.enabled) continue;
        if (states[i].diverged) {
            t.history.clear();
            t.originSet = false;
            t.draw->vertexCount = 0;
            continue;
        }
        const vsg::dvec3 p = positionEcef(states[i]);
        if (!t.originSet) {
            t.origin = p;
            t.originSet = true;
            t.transform->matrix = vsg::translate(p);
        }
        if (t.history.size() == points_) t.history.erase(t.history.begin());
        t.history.push_back(p);
        rewrite(t, static_cast<int>(i) == selected_);
    }
}

void Trails::rewrite(Trail& t, bool selected) {
    const std::size_t count = t.history.size();
    const vsg::vec3 base = selected ? vsg::vec3(1.0f, 0.65f, 0.2f) : vsg::vec3(0.55f, 0.85f, 1.0f);
    for (std::size_t k = 0; k < count; ++k) {
        const vsg::dvec3 rel = t.history[k] - t.origin;
        t.vertices->set(static_cast<std::uint32_t>(k), vsg::vec3(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z)));
        const float age = count > 1 ? static_cast<float>(k) / static_cast<float>(count - 1) : 1.0f; // 0 oldest .. 1 newest
        t.colors->set(static_cast<std::uint32_t>(k), vsg::vec4(base.x, base.y, base.z, 0.08f + 0.85f * age * age));
    }
    t.vertices->dirty();
    t.colors->dirty();
    t.draw->vertexCount = static_cast<std::uint32_t>(count >= 2 ? count : 0);
}

void Trails::setEnabled(std::size_t index, bool enabled) {
    if (index >= trails_.size()) return;
    Trail& t = trails_[index];
    if (t.enabled == enabled) return;
    t.enabled = enabled;
    if (!enabled) {
        t.history.clear();
        t.originSet = false;
        t.draw->vertexCount = 0;
    }
}

} // namespace fsim::world
