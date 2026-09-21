#include "world/VehicleVisuals.h"

#include "core/Log.h"
#include "world/FlatGeometry.h"
#include "world/Frames.h"

#include <cctype>
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

/// Flattens a loaded model to transform + draw pairs with white vertex
/// colours. The segmentation pass paints a whole vehicle one colour, so the
/// model's own state (pipelines, textures, materials) has to go, and so do
/// its vertex colours - the flat shader writes `vsg_Color * material`, and
/// anything but white would corrupt the id.
class StripState : public vsg::ConstVisitor {
public:
    vsg::ref_ptr<vsg::Group> result = vsg::Group::create();
    std::size_t skipped = 0; ///< draws whose arrays the flat pipeline cannot bind

    void apply(const vsg::Object& o) override { o.traverse(*this); }

    void apply(const vsg::Transform& t) override {
        const vsg::dmat4 outer = matrix_;
        matrix_ = t.transform(matrix_);
        t.traverse(*this);
        matrix_ = outer;
    }

    void apply(const vsg::VertexIndexDraw& d) override {
        vsg::DataList arrays;
        if (!whiteArrays(d.arrays, arrays)) {
            ++skipped;
            return;
        }
        auto draw = vsg::VertexIndexDraw::create();
        draw->assignArrays(arrays);
        if (d.indices && d.indices->data) draw->assignIndices(d.indices->data);
        draw->indexCount = d.indexCount;
        draw->instanceCount = d.instanceCount;
        draw->firstIndex = d.firstIndex;
        draw->vertexOffset = d.vertexOffset;
        draw->firstInstance = d.firstInstance;
        add(draw);
    }

    void apply(const vsg::VertexDraw& d) override {
        vsg::DataList arrays;
        if (!whiteArrays(d.arrays, arrays)) {
            ++skipped;
            return;
        }
        auto draw = vsg::VertexDraw::create();
        draw->assignArrays(arrays);
        draw->vertexCount = d.vertexCount;
        draw->instanceCount = d.instanceCount;
        draw->firstVertex = d.firstVertex;
        draw->firstInstance = d.firstInstance;
        add(draw);
    }

private:
    /// The flat pipeline binds vsg_Vertex (vec3), vsg_Normal (vec3),
    /// vsg_TexCoord0 (vec2) and vsg_Color (vec4), in that order. A draw that
    /// supplies anything else is left out rather than drawn with a wrong id.
    ///
    /// The colours are replaced by one white entry per *vertex*: a glTF model
    /// with no COLOR_0 carries a single-element colour array bound per
    /// instance, and reading that per vertex would feed the shader whatever
    /// lies past its end.
    static bool whiteArrays(const vsg::BufferInfoList& in, vsg::DataList& out) {
        if (in.size() != 4) return false;
        for (const auto& a : in) {
            if (!a || !a->data) return false;
            out.push_back(a->data);
        }
        auto vertices = out[0].cast<vsg::vec3Array>();
        if (!vertices || !out[1].cast<vsg::vec3Array>() || !out[2].cast<vsg::vec2Array>() || !out[3].cast<vsg::vec4Array>()) {
            out.clear();
            return false;
        }
        out[3] = vsg::vec4Array::create(vertices->size(), vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f));
        return true;
    }

    void add(const vsg::ref_ptr<vsg::Node>& draw) {
        auto xf = vsg::MatrixTransform::create(matrix_);
        xf->addChild(draw);
        result->addChild(xf);
    }

    vsg::dmat4 matrix_;
};

} // namespace

VehicleVisuals::VehicleVisuals(std::size_t count, const Settings& settings, vsg::ref_ptr<vsg::Options> options)
    : settings_(settings), options_(options) {
    root_ = vsg::Group::create();

    if (!settings.modelPath.empty()) default_.normal = loadModel(settings, options);
    if (default_.normal) {
        default_.highlighted = default_.normal; // a loaded model keeps its look; the label marks the selection
    } else {
        default_.normal = buildPlaceholder(settings, vsg::vec4(0.85f, 0.85f, 0.9f, 1.0f));
        default_.highlighted = buildPlaceholder(settings, vsg::vec4(1.0f, 0.55f, 0.1f, 1.0f));
    }

    if (settings.segmentation) {
        default_.geometry = stripState(default_.normal);
        segRoot_ = vsg::Group::create();
        segTransforms_.reserve(count);
        segSwitch_.reserve(count);
        segState_.reserve(count);
    }

    transforms_.reserve(count);
    highlight_.reserve(count);
    visible_.assign(count, 1);
    onMask_.assign(count, vsg::MASK_ALL);
    slotModel_.assign(count, std::string());
    for (std::size_t i = 0; i < count; ++i) {
        auto transform = vsg::MatrixTransform::create();
        // Shared subgraph under N transforms: one copy of the geometry on the GPU.
        auto sw = vsg::Switch::create();
        sw->addChild(true, default_.normal);
        sw->addChild(false, default_.highlighted);
        transform->addChild(sw);
        root_->addChild(transform);
        transforms_.push_back(transform);
        highlight_.push_back(sw);

        if (!segRoot_) continue;
        // The id colour is per slot, so the StateGroup is too; the stripped
        // geometry under it is shared with every other vehicle on that model.
        FlatGeometrySettings flat;
        flat.diffuse = segmentationColour(i);
        flat.depthWrite = false;                            // the main pass already wrote it
        flat.depthCompare = VK_COMPARE_OP_GREATER_OR_EQUAL; // ... including these very fragments
        auto state = createFlatStateGroup(flat, options_);
        if (!state) { // no flat pipeline: no segmentation rather than a broken scene
            segRoot_.reset();
            segTransforms_.clear();
            segSwitch_.clear();
            segState_.clear();
            continue;
        }
        if (default_.geometry) state->addChild(default_.geometry);
        auto segSw = vsg::Switch::create();
        segSw->addChild(true, state);
        auto segTransform = vsg::MatrixTransform::create();
        segTransform->addChild(segSw);
        segRoot_->addChild(segTransform);
        segTransforms_.push_back(segTransform);
        segSwitch_.push_back(segSw);
        segState_.push_back(state);
    }
}

vsg::vec4 VehicleVisuals::segmentationColour(std::size_t index) {
    // id = slot + 1, little end in red, high end in green: an 8-bit UNORM
    // attachment stores k/255 exactly, so the readback is lossless.
    const unsigned id = static_cast<unsigned>(index) + 1;
    return vsg::vec4(static_cast<float>(id & 0xFFu) / 255.0f, static_cast<float>((id >> 8) & 0xFFu) / 255.0f, 0.0f, 1.0f);
}

vsg::ref_ptr<vsg::Node> VehicleVisuals::stripState(const vsg::ref_ptr<vsg::Node>& model) {
    if (!model) return {};
    auto strip = vsg::visit<StripState>(model);
    if (strip.skipped > 0)
        LOG_WARN("world") << strip.skipped << " model draw(s) left out of the segmentation copy (unexpected vertex arrays)";
    if (strip.result->children.empty()) {
        LOG_WARN("world") << "no geometry could be stripped for segmentation";
        return {};
    }
    return strip.result;
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

std::string VehicleVisuals::resolveModel(const std::string& modelPath, const std::string& type) const {
    namespace fs = std::filesystem;
    if (!modelPath.empty()) {
        if (fs::exists(modelPath)) return modelPath;
        for (const auto& dir : settings_.modelDirs)
            if (fs::exists(dir / modelPath)) return (dir / modelPath).string();
        return modelPath; // let the loader report it once
    }
    // "jsbsim:f16" -> f16; only characters safe in a file name.
    const auto colon = type.find(':');
    std::string name = type.substr(colon == std::string::npos ? 0 : colon + 1);
    for (char& c : name)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.')) c = '_';
    if (name.empty()) return {};
    for (const auto& dir : settings_.modelDirs)
        for (const char* ext : {".glb", ".gltf"})
            if (fs::exists(dir / (name + ext))) return (dir / (name + ext)).string();
    return {};
}

const VehicleVisuals::Model& VehicleVisuals::modelFor(const std::string& key) {
    if (key.empty() || key == settings_.modelPath) return default_;
    auto it = library_.find(key);
    if (it == library_.end()) {
        Settings s; // glTF conventions, then the file's manifest
        s.modelPath = key;
        applyManifest(s);
        Model m;
        m.normal = loadModel(s, options_);
        if (m.normal) {
            m.highlighted = m.normal;
            if (compiler_ && !compiler_(m.normal)) LOG_WARN("world") << "could not compile vehicle model " << key;
            if (segRoot_) {
                m.geometry = stripState(m.normal);
                if (m.geometry && compiler_ && !compiler_(m.geometry))
                    LOG_WARN("world") << "could not compile the segmentation copy of " << key;
            }
        }
        it = library_.emplace(key, std::move(m)).first;
    }
    return it->second.normal ? it->second : default_;
}

void VehicleVisuals::setModel(std::size_t index, const std::string& modelPath, const std::string& type) {
    if (index >= transforms_.size()) return;
    const std::string key = resolveModel(modelPath, type);
    if (slotModel_[index] == key) return;
    slotModel_[index] = key;
    const Model& m = modelFor(key);
    auto& children = highlight_[index]->children;
    children[0].node = m.normal;
    children[1].node = m.highlighted;
    if (index < segState_.size()) {
        auto geometry = m.geometry ? m.geometry : default_.geometry;
        segState_[index]->children.clear();
        if (geometry) segState_[index]->addChild(geometry);
    }
}

const std::string& VehicleVisuals::modelOf(std::size_t index) const {
    static const std::string none;
    return index < slotModel_.size() ? slotModel_[index] : none;
}

void VehicleVisuals::update(Span<const sim::VehicleState> states) {
    const std::size_t n = std::min(states.size(), transforms_.size());
    for (std::size_t i = 0; i < n; ++i) {
        const vsg::dmat4 m = bodyToEcef(states[i]);
        transforms_[i]->matrix = m;
        if (i < segTransforms_.size()) segTransforms_[i]->matrix = m;
    }
}

void VehicleVisuals::applySwitch(std::size_t index) {
    auto& sw = highlight_[index];
    const std::size_t on = static_cast<int>(index) == selected_ ? 1 : 0;
    for (std::size_t i = 0; i < sw->children.size(); ++i) sw->children[i].mask = (visible_[index] && i == on) ? onMask_[index] : vsg::MASK_OFF;
    // The segmentation copy is never highlighted, but it hides and masks alike.
    if (index < segSwitch_.size()) segSwitch_[index]->children[0].mask = visible_[index] ? onMask_[index] : vsg::MASK_OFF;
}

void VehicleVisuals::setMask(std::size_t index, vsg::Mask mask) {
    if (index >= onMask_.size() || onMask_[index] == mask) return;
    onMask_[index] = mask;
    applySwitch(index);
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
