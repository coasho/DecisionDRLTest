#include "world/ElevatedTile.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::world {

namespace {

/// Finds the elevation texture (binding 7) under a tile subgraph and reports
/// its value range in metres (VSG displaces by the raw float with scale 1).
class ElevationRange : public vsg::ConstVisitor {
public:
    float minM = std::numeric_limits<float>::max(), maxM = std::numeric_limits<float>::lowest();
    bool found = false;

    void apply(const vsg::Object& object) override { object.traverse(*this); }
    void apply(const vsg::StateGroup& sg) override {
        for (const auto& sc : sg.stateCommands) sc->accept(*this);
        sg.traverse(*this);
    }
    void apply(const vsg::BindDescriptorSet& bds) override {
        if (!bds.descriptorSet) return;
        for (const auto& d : bds.descriptorSet->descriptors) {
            if (d->dstBinding != 7) continue;
            if (auto* image = d->cast<vsg::DescriptorImage>()) {
                for (const auto& info : image->imageInfoList) {
                    if (!info || !info->imageView || !info->imageView->image) continue;
                    if (auto elevation = info->imageView->image->data.cast<vsg::floatArray2D>()) {
                        for (const float v : *elevation) {
                            minM = std::min(minM, v);
                            maxM = std::max(maxM, v);
                        }
                        found = elevation->size() > 0;
                    }
                }
            }
        }
    }
    // PagedLOD external children are not loaded yet; only the inline tile matters.
    void apply(const vsg::PagedLOD& plod) override {
        for (const auto& child : plod.children)
            if (child.node) child.node->accept(*this);
    }
};

void lift(vsg::dsphere& bound, float minM, float maxM) {
    if (bound.radius <= 0.0 || !(maxM >= minM)) return;
    const vsg::dvec3 up = vsg::normalize(bound.center);
    const double mid = 0.5 * (static_cast<double>(minM) + static_cast<double>(maxM));
    const double half = 0.5 * (static_cast<double>(maxM) - static_cast<double>(minM));
    bound.center += up * mid;
    bound.radius += half + 50.0; // plus the mesh-vs-texel sampling slack
}

/// Visits PagedLOD and CullGroup nodes and corrects their bounds.
class FixBounds : public vsg::Visitor {
public:
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::PagedLOD& plod) override {
        ElevationRange range;
        for (auto& child : plod.children)
            if (child.node) child.node->accept(range);
        if (range.found) lift(plod.bound, range.minM, range.maxM);
        for (auto& child : plod.children)
            if (child.node) child.node->accept(*this);
    }
    void apply(vsg::CullGroup& group) override {
        ElevationRange range;
        group.traverse(range);
        if (range.found) lift(group.bound, range.minM, range.maxM);
        group.traverse(*this);
    }
};

} // namespace

vsg::ref_ptr<vsg::Object> ElevatedTile::read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options) const {
    auto result = vsg::tile::read(filename, options);
    if (result && settings && settings->elevationLayer) fixBounds(result.get());
    return result;
}

void ElevatedTile::fixBounds(vsg::Object* node) {
    if (!node) return;
    FixBounds fix;
    node->accept(fix);
}

bool readElevatedDatabase(vsg::TileDatabase& database, vsg::ref_ptr<const vsg::Options> options,
                          const std::vector<vsg::ref_ptr<vsg::ReaderWriter>>& extraReaders) {
    if (!database.settings || database.child) return false;
    if (database.settings->ellipsoidModel) database.setObject("EllipsoidModel", database.settings->ellipsoidModel);

    // The tile reader needs options that carry the extra readers too: the
    // subtiles it loads go through vsg::read(paths, options) with the options
    // it was created with.
    auto local = options ? vsg::clone(options) : vsg::Options::create();
    for (auto it = extraReaders.rbegin(); it != extraReaders.rend(); ++it) local->readerWriters.insert(local->readerWriters.begin(), *it);
    auto reader = ElevatedTile::create(database.settings, local);
    local->readerWriters.insert(local->readerWriters.begin(), reader);

    auto result = vsg::read("root.tile", local);
    database.child = result.cast<vsg::Node>();
    if (!database.child) {
        if (auto error = result.cast<vsg::ReadError>()) LOG_ERROR("world") << "tile database root failed: " << error->message;
        return false;
    }
    return true;
}

} // namespace fsim::world
