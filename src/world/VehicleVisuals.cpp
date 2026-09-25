#include "world/VehicleVisuals.h"

#include "core/Log.h"
#include "world/Exhaust.h"
#include "world/FlatGeometry.h"
#include "world/Frames.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
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

bool isJointName(const std::string& name) { return name.rfind("fsim:", 0) == 0; }

bool isExhaustNode(const vsg::Node& node) {
    std::string name;
    VehicleVisuals::Joint joint;
    return node.getValue("name", name) && VehicleVisuals::Joint::parse(name, joint) && joint.kind == VehicleVisuals::Joint::Afterburner;
}

/// The widest point of a node's children across its own x axis (their
/// geometry in its frame): how big a jet an afterburner node's flame mesh is
/// drawn for.
double radiusAcrossX(const vsg::Group& node) {
    vsg::ComputeBounds bounds;
    for (const auto& child : node.children) child->accept(bounds);
    if (!bounds.bounds.valid()) return 0.0;
    const auto& b = bounds.bounds;
    return std::max({std::abs(b.min.y), std::abs(b.max.y), std::abs(b.min.z), std::abs(b.max.z)});
}

/// Every node named fsim:<channel>[:<gain>], and the nodes on the paths down
/// to them (the "spine" a vehicle needs its own copy of).
class FindJoints : public vsg::Visitor {
public:
    std::vector<VehicleVisuals::Joint> joints;
    std::set<const vsg::Object*> spine;
    std::vector<std::string> malformed;

    void apply(vsg::Object& o) override { descend(o); }

    void apply(vsg::MatrixTransform& t) override {
        const vsg::dmat4 outer = matrix_;
        matrix_ = matrix_ * t.matrix;
        std::string name;
        if (t.getValue("name", name) && isJointName(name)) {
            VehicleVisuals::Joint joint;
            if (VehicleVisuals::Joint::parse(name, joint)) {
                joint.node = &t;
                joint.rest = t.matrix;
                joint.inModel = matrix_;
                if (joint.kind == VehicleVisuals::Joint::Afterburner) joint.radius = radiusAcrossX(t);
                joints.push_back(joint);
                spine.insert(path_.begin(), path_.end());
                spine.insert(&t);
            } else {
                malformed.push_back(name);
            }
        }
        descend(t);
        matrix_ = outer;
    }

private:
    void descend(vsg::Object& o) {
        path_.push_back(&o);
        o.traverse(*this);
        path_.pop_back();
    }
    std::vector<const vsg::Object*> path_;
    vsg::dmat4 matrix_;
};

/// A model's extent in its own frame, and its widest point to the right: the
/// wing tip, for a model that does not say where its wing is. The exhaust
/// flames' meshes are left out - they are not the airframe.
class Extent : public vsg::ConstVisitor {
public:
    vsg::dbox box;

    /// The tip: of the points furthest out to the right (within 3 cm), the
    /// one furthest aft - where a trailing edge ends.
    vsg::dvec3 widest() const {
        vsg::dvec3 best(0.0, 0.0, 0.0);
        bool found = false;
        for (const auto& p : right_)
            if (p.y > box.max.y - 0.03 && (!found || p.x < best.x)) {
                best = p;
                found = true;
            }
        return best;
    }

    void apply(const vsg::Object& o) override { o.traverse(*this); }
    void apply(const vsg::Transform& t) override {
        const vsg::dmat4 outer = matrix_;
        matrix_ = t.transform(matrix_);
        t.traverse(*this);
        matrix_ = outer;
    }
    void apply(const vsg::MatrixTransform& t) override {
        if (isExhaustNode(t)) return;
        apply(static_cast<const vsg::Transform&>(t));
    }
    void apply(const vsg::VertexIndexDraw& d) override { points(d.arrays); }
    void apply(const vsg::VertexDraw& d) override { points(d.arrays); }
    void apply(const vsg::Geometry& g) override { points(g.arrays); }

private:
    void points(const vsg::BufferInfoList& arrays) {
        if (arrays.empty() || !arrays[0]) return;
        auto vertices = arrays[0]->data.cast<vsg::vec3Array>();
        if (!vertices) return;
        for (const auto& v : *vertices) {
            const vsg::dvec3 p = matrix_ * vsg::dvec3(v.x, v.y, v.z);
            box.add(p);
            if (p.y > 0.0) right_.push_back(p);
        }
    }
    vsg::dmat4 matrix_;
    std::vector<vsg::dvec3> right_;
};

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

    /// A moving part stays a transform of its own, named as in the model and
    /// holding everything above it, so each vehicle's copy can still turn it.
    /// An exhaust's flame is left out: it is gas, not the vehicle, and a
    /// vehicle's id covers what a camera would see of the vehicle itself.
    void apply(const vsg::MatrixTransform& t) override {
        if (isExhaustNode(t)) return;
        std::string name;
        if (!t.getValue("name", name) || !isJointName(name)) {
            apply(static_cast<const vsg::Transform&>(t));
            return;
        }
        auto joint = vsg::MatrixTransform::create(t.transform(matrix_));
        joint->setValue("name", name);
        result->addChild(joint);
        const vsg::dmat4 outer = matrix_;
        const auto outerResult = result;
        matrix_ = vsg::dmat4();
        result = joint;
        t.traverse(*this);
        result = outerResult;
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
    /// lies past its end. Normals and texture coordinates a model leaves out
    /// come the same way (one vec3Value / vec2Value per instance, from
    /// vsgXchange's glTF reader) and get one default entry per vertex too.
    static bool whiteArrays(const vsg::BufferInfoList& in, vsg::DataList& out) {
        if (in.size() != 4) return false;
        for (const auto& a : in) {
            if (!a || !a->data) return false;
            out.push_back(a->data);
        }
        auto vertices = out[0].cast<vsg::vec3Array>();
        if (!vertices) {
            out.clear();
            return false;
        }
        const std::size_t n = vertices->size();
        auto normals = out[1].cast<vsg::vec3Array>();
        auto texcoords = out[2].cast<vsg::vec2Array>();
        const bool known = (normals || out[1].cast<vsg::vec3Value>()) && (texcoords || out[2].cast<vsg::vec2Value>()) &&
                           (out[3].cast<vsg::vec4Array>() || out[3].cast<vsg::vec4Value>());
        if (!known) {
            out.clear();
            return false;
        }
        if (!normals || normals->size() != n) out[1] = vsg::vec3Array::create(n, vsg::vec3(0.0f, 0.0f, 1.0f));
        if (!texcoords || texcoords->size() != n) out[2] = vsg::vec2Array::create(n, vsg::vec2(0.0f, 0.0f));
        out[3] = vsg::vec4Array::create(n, vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f));
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
    for (const auto& dir : settings.modelDirs) readStandIns(dir / "models.txt", standIns_); // the first directory wins
    if (settings.exhaust) {
        exhaust_ = std::make_unique<Exhaust>(count, options);
        exhaustOn_ = exhaust_->valid();
        flameCompiled_.assign(count * Exhaust::kPerSlot, 0);
    }

    bool loaded = false;
    if (!settings.modelPath.empty()) default_.normal = loadModel(settings, options);
    if (default_.normal) {
        default_.highlighted = default_.normal; // a loaded model keeps its look; the label marks the selection
        loaded = true;
    } else {
        default_.normal = buildPlaceholder(settings, vsg::vec4(0.85f, 0.85f, 0.9f, 1.0f));
        default_.highlighted = buildPlaceholder(settings, vsg::vec4(1.0f, 0.55f, 0.1f, 1.0f));
    }

    default_.rig = Rig::find(default_.normal);
    default_.shape = measure(default_.normal, default_.rig, settings);
    default_.shape.valid = loaded;
    if (settings.segmentation) {
        default_.geometry = stripState(default_.normal);
        default_.geometryRig = Rig::find(default_.geometry);
        segRoot_ = vsg::Group::create();
        segTransforms_.reserve(count);
        segSwitch_.reserve(count);
        segState_.reserve(count);
    }

    transforms_.reserve(count);
    highlight_.reserve(count);
    poses_.resize(count);
    visible_.assign(count, 1);
    onMask_.assign(count, vsg::MASK_ALL);
    slotModel_.assign(count, std::string());
    slotModels_.assign(count, &default_);
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
    if (!default_.rig.joints.empty())
        for (std::size_t i = 0; i < count; ++i) assign(i, default_);
}

VehicleVisuals::~VehicleVisuals() = default;

vsg::vec4 VehicleVisuals::segmentationColour(std::size_t index) {
    // id = slot + 1, little end in red, high end in green: an 8-bit UNORM
    // attachment stores k/255 exactly, so the readback is lossless.
    const unsigned id = static_cast<unsigned>(index) + 1;
    return vsg::vec4(static_cast<float>(id & 0xFFu) / 255.0f, static_cast<float>((id >> 8) & 0xFFu) / 255.0f, 0.0f, 1.0f);
}

namespace {
/// One `<channel>[:<gain>]` term of a joint's name.
bool parseTerm(std::string term, VehicleVisuals::Joint::Channel& channel, double& gain) {
    using J = VehicleVisuals::Joint;
    std::string value;
    if (const auto colon = term.find(':'); colon != std::string::npos) {
        value = term.substr(colon + 1);
        term.resize(colon);
    }
    if (term == "aileron") channel = J::Aileron;
    else if (term == "elevator") channel = J::Elevator;
    else if (term == "rudder") channel = J::Rudder;
    else if (term == "flaps" || term == "flap") channel = J::Flaps;
    else return false;
    gain = 1.0;
    if (!value.empty()) {
        char* end = nullptr;
        const double g = std::strtod(value.c_str(), &end);
        if (end == value.c_str() || *end != '\0' || !std::isfinite(g)) return false;
        gain = g;
    }
    return true;
}
} // namespace

namespace {
/// The numbers of `<a>[:<b>...]`, all finite, or false.
bool parseNumbers(const std::string& text, std::vector<double>& out) {
    out.clear();
    if (text.empty()) return true;
    std::size_t begin = 0;
    for (;;) {
        const auto colon = text.find(':', begin);
        const std::string item = text.substr(begin, colon == std::string::npos ? std::string::npos : colon - begin);
        char* end = nullptr;
        const double v = std::strtod(item.c_str(), &end);
        if (item.empty() || end == item.c_str() || *end != '\0' || !std::isfinite(v)) return false;
        out.push_back(v);
        if (colon == std::string::npos) return true;
        begin = colon + 1;
    }
}
} // namespace

bool VehicleVisuals::Joint::parse(const std::string& name, Joint& joint) {
    if (!isJointName(name)) return false;
    std::string terms = name.substr(5);
    joint.mix.clear();
    joint.kind = Surface;
    joint.phase = 0.0;
    joint.phaseTime = std::numeric_limits<double>::quiet_NaN();
    // a landing gear's leg or door: fsim:gear:<deg>[:<g0>:<g1>]
    if (terms.rfind("gear:", 0) == 0) {
        std::vector<double> v;
        if (!parseNumbers(terms.substr(5), v) || (v.size() != 1 && v.size() != 3)) return false;
        joint.kind = Gear;
        joint.gearRad = v[0] * 3.14159265358979323846 / 180.0;
        joint.g0 = v.size() == 3 ? v[1] : 1.0;
        joint.g1 = v.size() == 3 ? v[2] : 0.0;
        return joint.g0 != joint.g1;
    }
    // a leading-edge flap: fsim:lef[:<gain>][@<lo>,<hi>], as the aircraft's
    // flight controls move it (VehicleState::leadingEdgeFlapRad)
    if (terms == "lef" || terms.rfind("lef:", 0) == 0 || terms.rfind("lef@", 0) == 0) {
        joint.lo = -std::numeric_limits<double>::infinity();
        joint.hi = std::numeric_limits<double>::infinity();
        if (const auto at = terms.find('@'); at != std::string::npos) {
            std::vector<double> stops;
            std::string s = terms.substr(at + 1);
            std::replace(s.begin(), s.end(), ',', ':');
            if (!parseNumbers(s, stops) || stops.size() != 2 || !(stops[0] < stops[1])) return false;
            joint.lo = stops[0] * 3.14159265358979323846 / 180.0;
            joint.hi = stops[1] * 3.14159265358979323846 / 180.0;
            terms.resize(at);
        }
        std::vector<double> v;
        if (!parseNumbers(terms.size() > 3 ? terms.substr(4) : std::string(), v) || v.size() > 1) return false;
        joint.kind = LeadingEdge;
        joint.gain = v.empty() ? 1.0 : v[0];
        return true;
    }
    // a wheeled gear unit's parts, by its index among the aircraft's wheels
    // (VehicleState::wheel*): fsim:oleo:<wheel>[:<gain>] slides along its x
    // axis by the strut's compression (times the gain), fsim:steer:<wheel>
    // turns about it by the steering angle, fsim:wheel:<wheel>:<radius> rolls
    // about it at the wheel's speed
    if (terms.rfind("oleo:", 0) == 0 || terms.rfind("steer:", 0) == 0 || terms.rfind("wheel:", 0) == 0) {
        const auto colon = terms.find(':');
        const std::string what = terms.substr(0, colon);
        std::vector<double> v;
        if (!parseNumbers(terms.substr(colon + 1), v) || v.empty()) return false;
        if (v[0] < 0.0 || v[0] >= sim::VehicleState::kMaxWheels || v[0] != std::floor(v[0])) return false;
        joint.wheel = static_cast<int>(v[0]);
        if (what == "oleo") {
            if (v.size() > 2) return false;
            joint.kind = Oleo;
            joint.gain = v.size() == 2 ? v[1] : 1.0;
        } else if (what == "steer") {
            if (v.size() != 1) return false;
            joint.kind = Steer;
        } else {
            if (v.size() != 2 || !(v[1] > 0.0)) return false;
            joint.kind = Wheel;
            joint.wheelRadius = v[1];
        }
        return true;
    }
    // a nozzle petal: fsim:nozzle:<engine>:<deg wide open>
    if (terms.rfind("nozzle:", 0) == 0) {
        std::vector<double> v;
        if (!parseNumbers(terms.substr(7), v) || v.size() != 2) return false;
        if (v[0] < 0.0 || v[0] >= sim::VehicleState::kMaxEngines || v[0] != std::floor(v[0])) return false;
        joint.kind = Nozzle;
        joint.engine = static_cast<int>(v[0]);
        joint.nozzleRad = v[1] * 3.14159265358979323846 / 180.0;
        return true;
    }
    // a propeller: fsim:propeller:<engine>, at that engine's rpm
    if (terms.rfind("propeller:", 0) == 0) {
        std::vector<double> v;
        if (!parseNumbers(terms.substr(10), v) || v.size() != 1) return false;
        if (v[0] < 0.0 || v[0] >= sim::VehicleState::kMaxEngines || v[0] != std::floor(v[0])) return false;
        joint.kind = Propeller;
        joint.engine = static_cast<int>(v[0]);
        return true;
    }
    // an exhaust plume: fsim:afterburner[:<engine>]
    if (terms == "afterburner" || terms.rfind("afterburner:", 0) == 0) {
        std::vector<double> v;
        if (!parseNumbers(terms.size() > 11 ? terms.substr(12) : std::string(), v) || v.size() > 1) return false;
        const double e = v.empty() ? 0.0 : v[0];
        if (e < 0.0 || e >= sim::VehicleState::kMaxEngines || e != std::floor(e)) return false;
        joint.kind = Afterburner;
        joint.engine = static_cast<int>(e);
        return true;
    }
    joint.lo = -std::numeric_limits<double>::infinity();
    joint.hi = std::numeric_limits<double>::infinity();
    if (const auto at = terms.find('@'); at != std::string::npos) {
        // @<lo>,<hi>: the part's stops, degrees
        const std::string stops = terms.substr(at + 1);
        terms.resize(at);
        const char* p = stops.c_str();
        char* end = nullptr;
        const double lo = std::strtod(p, &end);
        if (end == p || *end != ',') return false;
        p = end + 1;
        const double hi = std::strtod(p, &end);
        if (end == p || *end != '\0' || !std::isfinite(lo) || !std::isfinite(hi) || !(lo < hi)) return false;
        constexpr double kRad = 3.14159265358979323846 / 180.0;
        joint.lo = lo * kRad;
        joint.hi = hi * kRad;
    }
    std::size_t begin = 0;
    for (bool first = true;; first = false) {
        const auto plus = terms.find('+', begin);
        Channel channel = Aileron;
        double gain = 1.0;
        if (!parseTerm(terms.substr(begin, plus == std::string::npos ? std::string::npos : plus - begin), channel, gain))
            return false;
        if (first) {
            joint.channel = channel;
            joint.gain = gain;
        } else {
            joint.mix.emplace_back(channel, gain);
        }
        if (plus == std::string::npos) return true;
        begin = plus + 1;
    }
}

double VehicleVisuals::Joint::turn(double perSecond, double simTime) {
    // over the sim's time since the last frame; held still when the clock
    // stops or jumps (pause, a reset, a replay's seek)
    if (std::isfinite(simTime)) {
        const double dt = std::isfinite(phaseTime) ? simTime - phaseTime : 0.0;
        if (dt > 0.0 && dt < 1.0 && std::isfinite(perSecond)) phase += perSecond * dt;
        phase -= std::floor(phase);
        phaseTime = simTime;
    }
    return phase;
}

vsg::dmat4 VehicleVisuals::Joint::matrix(const sim::VehicleState& s) {
    if (kind == Gear) {
        const double g = std::isfinite(s.gearPosition) ? s.gearPosition : 1.0;
        const double t = std::clamp((g - g0) / (g1 - g0), 0.0, 1.0);
        return rest * vsg::rotate(gearRad * t, vsg::dvec3(1.0, 0.0, 0.0));
    }
    if (kind == LeadingEdge) {
        const double d = std::isfinite(s.leadingEdgeFlapRad) ? s.leadingEdgeFlapRad : 0.0;
        return rest * vsg::rotate(std::clamp(gain * d, lo, hi), vsg::dvec3(1.0, 0.0, 0.0));
    }
    const bool engineOk = engine < s.engineCount;
    if (kind == Propeller) {
        const double rpm = engineOk && std::isfinite(s.engineRpm[engine]) ? std::abs(s.engineRpm[engine]) : 0.0;
        return rest * vsg::rotate(2.0 * 3.14159265358979323846 * turn(rpm / 60.0, s.simTime), vsg::dvec3(1.0, 0.0, 0.0));
    }
    const bool wheelOk = wheel < s.wheelCount;
    if (kind == Oleo) {
        const double c = wheelOk && std::isfinite(s.wheelCompressionM[wheel]) ? std::max(s.wheelCompressionM[wheel], 0.0) : 0.0;
        return rest * vsg::translate(gain * c, 0.0, 0.0);
    }
    if (kind == Steer) {
        const double a = wheelOk && std::isfinite(s.wheelSteerRad[wheel]) ? s.wheelSteerRad[wheel] : 0.0;
        return rest * vsg::rotate(a, vsg::dvec3(1.0, 0.0, 0.0));
    }
    if (kind == Wheel) {
        const double v = wheelOk && std::isfinite(s.wheelSpeedMs[wheel]) ? s.wheelSpeedMs[wheel] : 0.0;
        const double perSecond = v / (2.0 * 3.14159265358979323846 * wheelRadius);
        return rest * vsg::rotate(2.0 * 3.14159265358979323846 * turn(perSecond, s.simTime), vsg::dvec3(1.0, 0.0, 0.0));
    }
    if (kind == Nozzle) {
        // the nozzle as the engine model has it: shut at military power, open
        // at idle and with the afterburner lit
        const double n = engineOk && std::isfinite(s.nozzlePosition[engine]) ? std::clamp(s.nozzlePosition[engine], 0.0, 1.0) : 0.0;
        return rest * vsg::rotate(nozzleRad * n, vsg::dvec3(1.0, 0.0, 0.0));
    }
    if (kind == Afterburner) {
        const double a = engineOk && std::isfinite(s.afterburner[engine]) ? std::clamp(s.afterburner[engine], 0.0, 1.0) : 0.0;
        if (a <= 0.0) return rest * vsg::scale(0.0, 0.0, 0.0);
        return rest * vsg::scale(0.35 + 0.65 * a, 0.6 + 0.4 * a, 0.6 + 0.4 * a);
    }
    const auto of = [&s](Channel c) {
        double d = 0.0;
        switch (c) {
        case Aileron: d = s.aileronRad; break;
        case Elevator: d = s.elevatorRad; break;
        case Rudder: d = s.rudderRad; break;
        case Flaps: d = s.flapsRad; break;
        }
        return std::isfinite(d) ? d : 0.0;
    };
    double angle = gain * of(channel);
    for (const auto& [c, g] : mix) angle += g * of(c);
    angle = std::clamp(angle, lo, hi);
    return rest * vsg::rotate(angle, vsg::dvec3(1.0, 0.0, 0.0));
}

VehicleVisuals::Rig VehicleVisuals::Rig::find(const vsg::ref_ptr<vsg::Node>& graph) {
    Rig rig;
    if (!graph) return rig;
    auto finder = vsg::visit<FindJoints>(graph);
    for (const auto& name : finder.malformed)
        LOG_WARN("world") << "model node '" << name
                          << "' is not a moving part (fsim:aileron|elevator|rudder|flaps[:gain][+...][@lo,hi], "
                             "fsim:gear:<deg>[:<g0>:<g1>], fsim:afterburner[:<engine>], fsim:lef[:<gain>][@lo,hi], "
                             "fsim:propeller:<engine>, fsim:nozzle:<engine>:<deg>, fsim:oleo:<wheel>[:<gain>], "
                             "fsim:steer:<wheel>, fsim:wheel:<wheel>:<radius>); left fixed";
    rig.joints = std::move(finder.joints);
    rig.spine.assign(finder.spine.begin(), finder.spine.end());
    return rig;
}

vsg::ref_ptr<vsg::Node> VehicleVisuals::Rig::copy(const vsg::ref_ptr<vsg::Node>& graph,
                                                  std::vector<vsg::ref_ptr<vsg::MatrixTransform>>& transforms) const {
    vsg::CopyOp copyop;
    copyop.duplicate = new vsg::Duplicate; // Duplicate has no create() of its own
    for (const auto* node : spine) copyop.duplicate->insert(node);
    auto out = copyop(graph);
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> copies;
    for (const auto& joint : joints) {
        const auto it = copyop.duplicate->find(joint.node);
        auto t = it != copyop.duplicate->end() ? it->second.cast<vsg::MatrixTransform>() : vsg::ref_ptr<vsg::MatrixTransform>();
        if (!t || t.get() == joint.node) return {}; // a node on the way down does not copy: no moving parts
        copies.push_back(t);
    }
    transforms.insert(transforms.end(), copies.begin(), copies.end());
    return out;
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
    settings.surfaces.clear();
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string key, value;
        if (!(ls >> key >> value) || key.empty() || key[0] == '#') continue;
        if (key == "forward") axis(value, settings.modelForward);
        else if (key == "up") axis(value, settings.modelUp);
        else if (key == "scale") settings.modelScale = std::stod(value);
        else if (key == "wing" || key == "strake" || key == "canard") {
            // <kind> then per section x y z (the leading edge, body axes) and the chord
            std::istringstream values(line.substr(line.find(key) + key.size()));
            std::vector<double> v;
            for (double d; values >> d;) v.push_back(d);
            Surface s;
            s.kind = key;
            for (std::size_t i = 0; i + 3 < v.size(); i += 4) s.sections.emplace_back(v[i], v[i + 1], v[i + 2], v[i + 3]);
            if (s.sections.size() >= 2 && v.size() % 4 == 0) settings.surfaces.push_back(std::move(s));
            else LOG_WARN("world") << "manifest: '" << key << "' needs sections of x y z chord, two or more, in " << settings.modelPath << ".manifest";
        } else LOG_WARN("world") << "manifest: unknown key '" << key << "' in " << settings.modelPath << ".manifest";
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
    auto xf = vsg::MatrixTransform::create(vsg::translate(s.modelOffset) * modelToBody *
                                           vsg::scale(s.modelScale, s.modelScale, s.modelScale));
    xf->addChild(node);

    // Animated parts (propellers, ...): collected so the viewer can play them.
    auto finder = vsg::visit<vsg::FindAnimations>(node);
    for (auto& anim : finder.animations) anim->mode = vsg::Animation::REPEAT;
    const_cast<VehicleVisuals*>(this)->animations_ = finder.animations;
    LOG_INFO("world") << "vehicle model: " << s.modelPath << " (" << finder.animations.size() << " animation(s))";
    return xf;
}

VehicleVisuals::Shape VehicleVisuals::measure(const vsg::ref_ptr<vsg::Node>& model, const Rig& rig, const Settings& s) {
    Shape shape;
    if (!model) return shape;
    Extent extent;
    model->accept(extent);
    if (extent.box.valid()) {
        shape.lo = extent.box.min;
        shape.hi = extent.box.max;
        shape.tip = extent.widest();
    }
    // the jets where the afterburner nodes stand at rest (the rig's matrices
    // are in the model's own frame, which is the body's: loadModel turns it)
    for (const auto& j : rig.joints) {
        if (j.kind != Joint::Afterburner) continue;
        Shape::Jet jet;
        jet.exit = j.inModel * vsg::dvec3(0.0, 0.0, 0.0);
        const vsg::dvec3 along = j.inModel * vsg::dvec3(1.0, 0.0, 0.0) - jet.exit;
        const vsg::dvec3 across = j.inModel * vsg::dvec3(0.0, 1.0, 0.0) - jet.exit;
        jet.direction = vsg::length(along) > 0.0 ? vsg::normalize(along) : vsg::dvec3(-1.0, 0.0, 0.0);
        jet.radius = j.radius * vsg::length(across);
        jet.engine = j.engine;
        shape.jets.push_back(jet);
    }
    // the manifest's surfaces, moved and scaled as the model is
    for (auto surface : s.surfaces) {
        for (auto& sec : surface.sections) {
            const vsg::dvec3 p = s.modelOffset + vsg::dvec3(sec.x, sec.y, sec.z) * s.modelScale;
            sec = vsg::dvec4(p.x, p.y, p.z, sec.w * s.modelScale);
        }
        shape.surfaces.push_back(std::move(surface));
    }
    for (const auto& surface : shape.surfaces)
        if (surface.kind == "wing") { // its tip's trailing edge
            const auto& t = surface.sections.back();
            shape.tip = vsg::dvec3(t.x - t.w, t.y, t.z);
            break;
        }
    return shape;
}

const VehicleVisuals::Shape& VehicleVisuals::shape(std::size_t index) const {
    return index < slotModels_.size() && slotModels_[index] ? slotModels_[index]->shape : default_.shape;
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

bool VehicleVisuals::readStandIns(const std::filesystem::path& file, std::map<std::string, StandIn>& out) {
    std::ifstream in(file);
    if (!in) return false;
    std::string line;
    int n = 0;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string type;
        StandIn s;
        if (!(ls >> type) || type[0] == '#') continue;
        if (!(ls >> s.design >> s.offset.x >> s.offset.y >> s.offset.z)) {
            LOG_WARN("world") << file.string() << ": '" << line << "' is not <type> <design> <forward> <right> <down>";
            continue;
        }
        if (out.emplace(type, s).second) ++n;
    }
    if (n) LOG_INFO("world") << "model stand-ins: " << n << " from " << file.string();
    return true;
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
    // a stock aircraft with a stand-in: that design's model, moved (the key
    // carries the offset: "<path>|<forward>,<right>,<down>")
    if (const auto it = standIns_.find(name); it != standIns_.end()) {
        const auto& s = it->second;
        for (const auto& dir : settings_.modelDirs)
            for (const char* ext : {".glb", ".gltf"}) {
                const auto path = dir / s.design / (s.design + ext);
                if (!fs::exists(path)) continue;
                std::ostringstream key;
                key << path.string() << '|' << s.offset.x << ',' << s.offset.y << ',' << s.offset.z;
                return key.str();
            }
    }
    for (const auto& dir : settings_.modelDirs)
        for (const char* ext : {".glb", ".gltf"}) {
            if (fs::exists(dir / (name + ext))) return (dir / (name + ext)).string();
            if (fs::exists(dir / name / (name + ext))) return (dir / name / (name + ext)).string(); // an aircraft folder
        }
    return {};
}

const VehicleVisuals::Model& VehicleVisuals::modelFor(const std::string& key) {
    if (key.empty() || key == settings_.modelPath) return default_;
    auto it = library_.find(key);
    if (it == library_.end()) {
        Settings s; // glTF conventions, then the file's manifest
        s.modelPath = key;
        if (const auto bar = key.rfind('|'); bar != std::string::npos) { // a stand-in, moved
            s.modelPath = key.substr(0, bar);
            std::string offset = key.substr(bar + 1);
            std::replace(offset.begin(), offset.end(), ',', ' ');
            std::istringstream(offset) >> s.modelOffset.x >> s.modelOffset.y >> s.modelOffset.z;
        }
        applyManifest(s);
        Model m;
        m.normal = loadModel(s, options_);
        if (m.normal) {
            m.highlighted = m.normal;
            m.rig = Rig::find(m.normal);
            m.shape = measure(m.normal, m.rig, s);
            m.shape.valid = true;
            if (!m.rig.joints.empty()) LOG_INFO("world") << "vehicle model: " << key << ": " << m.rig.joints.size() << " moving part(s)";
            if (compiler_ && !compiler_(m.normal)) LOG_WARN("world") << "could not compile vehicle model " << key;
            if (segRoot_) {
                m.geometry = stripState(m.normal);
                m.geometryRig = Rig::find(m.geometry);
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
    assign(index, modelFor(key));
}

void VehicleVisuals::assign(std::size_t index, const Model& m) {
    Pose& pose = poses_[index];
    pose = Pose{};
    slotModels_[index] = &m;
    vsg::ref_ptr<vsg::Node> normal = m.normal;
    if (!m.rig.joints.empty()) {
        if (auto copy = m.rig.copy(m.normal, pose.transforms)) {
            pose.normal = normal = copy;
            pose.joints = m.rig.joints;
        } else {
            LOG_WARN("world") << "vehicle model: its control surfaces could not be copied per vehicle; drawn fixed";
        }
    }
    // The exhaust hangs under this vehicle's own copy of each afterburner
    // node, beside the model's flame mesh, a switch choosing between them.
    pose.flames.assign(pose.transforms.size(), -1);
    pose.flameSwitches.assign(pose.transforms.size(), {});
    if (exhaust_ && exhaust_->valid()) {
        std::size_t k = 0;
        for (std::size_t j = 0; j < pose.joints.size() && k < Exhaust::kPerSlot; ++j) {
            if (pose.joints[j].kind != Joint::Afterburner) continue;
            auto flame = exhaust_->node(index, k);
            if (!flame) break;
            auto own = vsg::Group::create(); // the model's own flame, shown with the exhaust off
            own->children = pose.transforms[j]->children;
            auto sw = vsg::Switch::create();
            sw->addChild(!exhaustOn_, own);
            sw->addChild(false, flame); // update() shows it while it has something to draw
            pose.transforms[j]->children = {sw};
            // made after the scene was compiled: compiled now (once; without a
            // compiler it is compiled with the scene it is part of)
            const std::size_t f = index * Exhaust::kPerSlot + k;
            if (compiler_ && f < flameCompiled_.size() && !flameCompiled_[f]) {
                if (!compiler_(flame)) LOG_WARN("world") << "could not compile an engine's exhaust";
                flameCompiled_[f] = 1;
            }
            pose.flames[j] = static_cast<int>(k);
            pose.flameSwitches[j] = sw;
            ++k;
        }
    }
    auto& children = highlight_[index]->children;
    children[0].node = normal;
    children[1].node = m.highlighted == m.normal ? normal : m.highlighted;
    if (index < segState_.size()) {
        vsg::ref_ptr<vsg::Node> geometry = m.geometry ? m.geometry : default_.geometry;
        if (m.geometry && !m.geometryRig.joints.empty()) {
            if (auto copy = m.geometryRig.copy(m.geometry, pose.transforms)) {
                pose.geometry = geometry = copy;
                pose.joints.insert(pose.joints.end(), m.geometryRig.joints.begin(), m.geometryRig.joints.end());
            }
        }
        segState_[index]->children.clear();
        if (geometry) segState_[index]->addChild(geometry);
    }
    pose.flames.resize(pose.transforms.size(), -1); // the segmentation copy's joints carry none
    pose.flameSwitches.resize(pose.transforms.size());
}

const std::string& VehicleVisuals::modelOf(std::size_t index) const {
    static const std::string none;
    return index < slotModel_.size() ? slotModel_[index] : none;
}

void VehicleVisuals::update(Span<const sim::VehicleState> states) {
    const std::size_t n = std::min(states.size(), transforms_.size());
    double clock = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < n; ++i) {
        const vsg::dmat4 m = bodyToEcef(states[i]);
        transforms_[i]->matrix = m;
        if (i < segTransforms_.size()) segTransforms_[i]->matrix = m;
        Pose& pose = poses_[i];
        for (std::size_t j = 0; j < pose.transforms.size(); ++j) {
            const int flame = j < pose.flames.size() ? pose.flames[j] : -1;
            if (flame < 0 || !exhaustOn_) {
                pose.transforms[j]->matrix = pose.joints[j].matrix(states[i]);
                continue;
            }
            // the exhaust sizes itself: its node stays as the model placed it
            // (or as whatever turns it)
            const Joint& joint = pose.joints[j];
            pose.transforms[j]->matrix = joint.rest;
            const bool shows = visible_[i] && exhaust_->set(i, static_cast<std::size_t>(flame), joint.radius, states[i], joint.engine);
            pose.flameSwitches[j]->children[1].mask = shows ? vsg::MASK_ALL : vsg::MASK_OFF;
        }
        if (visible_[i] && !std::isfinite(clock) && std::isfinite(states[i].simTime)) clock = states[i].simTime;
    }
    if (exhaust_ && exhaustOn_) {
        exhaust_->setFrame(std::isfinite(clock) ? clock : 0.0, daylight_);
        exhaust_->commit();
    }
}

void VehicleVisuals::setExhaust(bool on) {
    on = on && exhaust_ && exhaust_->valid();
    if (on == exhaustOn_) return;
    exhaustOn_ = on;
    for (auto& pose : poses_)
        for (auto& sw : pose.flameSwitches)
            if (sw) {
                sw->children[0].mask = on ? vsg::MASK_OFF : vsg::MASK_ALL; // the model's own flame
                if (!on) sw->children[1].mask = vsg::MASK_OFF;
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
