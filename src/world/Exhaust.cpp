#include "world/Exhaust.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {

constexpr double kPi = 3.14159265358979323846;

// The shells of one exhaust, each a surface of revolution about the jet (x,
// aft from the exit) that the vertex shader shapes: u runs along it (0 at the
// nozzle), the ring's cos and sin go round it.
enum Shell { Envelope, Core, Diamonds, Glow, Haze, kShells };
constexpr int kRings[kShells] = {24, 16, 56, 8, 16};
constexpr int kSegments = 24;

const char* kVertex = R"(#version 450
layout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;
layout(std430, set = 0, binding = 0) readonly buffer Rows { vec4 rows[]; };
layout(set = 0, binding = 1) uniform Frame { vec4 frame; }; // x time (s), y daylight
layout(location = 0) in vec4 plume; // u along the shell, the ring's cos and sin, the shell
layout(location = 0) out vec3 viewPos;
layout(location = 1) out vec3 viewNormal;
layout(location = 2) out vec4 along; // u, x / diameter, shell, radius / nozzle radius
layout(location = 3) out vec2 ring;
layout(location = 4) flat out vec4 engine; // afterburner, dry power, nozzle, seed
out gl_PerVertex { vec4 gl_Position; };

const float PI = 3.14159265;

// Each shell's length along the jet and radius at u, for a nozzle of radius r
// (m). The flame grows with the afterburner (JSBSim's augmented turbine: its
// stage), from a short blue cone to some six diameters.
vec2 shape(int shell, float u, float r, float ab, float dry, float flicker) {
    float d = 2.0 * r;
    if (shell == 0) { // the envelope: a long cone that swells a little past the nozzle
        float L = d * (1.2 + 4.6 * ab) * flicker;
        return vec2(u * L, r * (1.0 + 0.45 * u) * pow(max(1.0 - u, 0.0), 0.85));
    }
    if (shell == 1) { // the core
        float L = d * (0.7 + 2.3 * ab) * flicker;
        return vec2(u * L, 0.62 * r * pow(max(1.0 - u, 0.0), 0.8));
    }
    if (shell == 2) { // four shock diamonds strung along the core
        float L = d * (0.45 + 2.6 * ab) * flicker;
        return vec2(u * L, 0.45 * r * pow(abs(sin(PI * 4.0 * u)), 0.7) * (1.0 - 0.55 * u));
    }
    if (shell == 3) // the nozzle's hot inside: a cone down towards the turbine
        return vec2(mix(-1.1 * d, -0.02 * d, u), r * mix(0.25, 0.97, u));
    // hot gas carried on behind it, spreading
    return vec2(u * d * (3.0 + 3.0 * dry), r * (0.95 + 1.1 * u) * sqrt(max(1.0 - u * u * u, 0.0)));
}

void main() {
    vec4 a = rows[2 * gl_InstanceIndex];
    vec4 b = rows[2 * gl_InstanceIndex + 1];
    float r = a.x, ab = a.y, dry = a.z, nozzle = a.w, seed = b.x;
    float t = frame.x;
    int shell = int(plume.w + 0.5);
    // nothing to show: every vertex of the shell on one point, no fragments
    float shown = shell <= 2 ? ab : shell == 3 ? max(ab, dry) : max(dry, ab);
    if (shown <= 0.0) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        viewPos = vec3(0.0); viewNormal = vec3(0.0, 0.0, 1.0); along = vec4(0.0); ring = vec2(0.0); engine = vec4(0.0);
        return;
    }
    float flicker = 1.0 + 0.04 * sin(t * 31.0 + seed * 40.0) + 0.025 * sin(t * 57.0 + seed * 13.0);
    float rx = r * (1.0 + 0.07 * nozzle); // the petals open with the afterburner
    vec2 p = shape(shell, plume.x, rx, ab, dry, flicker);
    vec2 ahead = shape(shell, max(plume.x - 0.01, 0.0), rx, ab, dry, flicker);
    vec2 behind = shape(shell, min(plume.x + 0.01, 1.0), rx, ab, dry, flicker);
    float slope = (behind.y - ahead.y) / max(behind.x - ahead.x, 1e-5);
    vec4 v = pc.modelView * vec4(p.x, p.y * plume.y, p.y * plume.z, 1.0);
    viewPos = v.xyz;
    viewNormal = mat3(pc.modelView) * normalize(vec3(-slope, plume.y, plume.z));
    along = vec4(plume.x, p.x / (2.0 * r), float(shell), p.y / max(rx, 1e-4));
    ring = plume.yz;
    engine = vec4(ab, dry, nozzle, seed);
    gl_Position = pc.projection * v;
}
)";

// Premultiplied colour: rgb is light added, a how much of what lies behind is
// hidden. By night a flame is all light (a = 0); by day it also takes some of
// the sky away, or its orange washes out to white against it.
const char* kFragment = R"(#version 450
layout(set = 0, binding = 1) uniform Frame { vec4 frame; };
layout(location = 0) in vec3 viewPos;
layout(location = 1) in vec3 viewNormal;
layout(location = 2) in vec4 along;
layout(location = 3) in vec2 ring;
layout(location = 4) flat in vec4 engine;
layout(location = 0) out vec4 outColor;

float hash(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}
float noise(vec3 x) {
    vec3 i = floor(x), f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash(i), hash(i + vec3(1, 0, 0)), f.x), mix(hash(i + vec3(0, 1, 0)), hash(i + vec3(1, 1, 0)), f.x), f.y),
               mix(mix(hash(i + vec3(0, 0, 1)), hash(i + vec3(1, 0, 1)), f.x), mix(hash(i + vec3(0, 1, 1)), hash(i + vec3(1, 1, 1)), f.x), f.y),
               f.z);
}

void main() {
    float u = along.x, xd = along.y;
    int shell = int(along.z + 0.5);
    float ab = engine.x, dry = engine.y, seed = engine.w;
    float t = frame.x, day = frame.y;
    // soft edges: a shell seen edge-on is a thin sliver of gas, seen face-on
    // the whole depth of it
    float facing = abs(dot(normalize(viewNormal), normalize(-viewPos)));
    // turbulence carried down the jet
    vec3 at = vec3(xd * 2.5 - t * 14.0, ring * 1.6 + seed * 9.0);
    float n = 0.6 * noise(at) + 0.4 * noise(at * 2.7 + 3.1);
    vec3 c = vec3(0.0);
    float a = 0.0;
    float lit = smoothstep(0.0, 0.08, ab);
    // by day a flame also hides some of the sky, or it washes out to white
    float cover = 0.15 + 0.55 * day;
    // (colours are linear: the window is sRGB, so they come out lighter)
    if (shell == 0) { // orange, cooling to red at its tail; blue at the nozzle while barely lit
        float heat = pow(1.0 - u, 1.2);
        vec3 col = mix(vec3(0.7, 0.1, 0.01), vec3(1.0, 0.32, 0.045), smoothstep(0.0, 0.5, heat));
        col = mix(col, vec3(1.0, 0.55, 0.15), smoothstep(0.65, 1.0, heat));
        col = mix(col, vec3(0.12, 0.2, 1.0), (1.0 - smoothstep(0.15, 0.6, ab)) * pow(1.0 - u, 2.0));
        float I = lit * smoothstep(0.0, 0.75, facing) * heat * (0.55 + 0.8 * n);
        c = col * I * 0.8;
        a = I * 0.75 * cover;
    } else if (shell == 1) { // white-hot at the nozzle
        float heat = pow(1.0 - u, 1.5);
        vec3 col = mix(vec3(1.0, 0.45, 0.1), vec3(1.0, 0.78, 0.45), heat);
        col = mix(col, vec3(0.3, 0.42, 1.0), (1.0 - smoothstep(0.1, 0.5, ab)) * 0.8);
        float I = lit * pow(facing, 1.5) * heat * (0.85 + 0.3 * n);
        c = col * I * 0.8;
        a = I * 0.2 * cover;
    } else if (shell == 2) { // the diamonds: a bead of light at each, brightest where it is widest
        float I = smoothstep(0.05, 0.4, ab) * pow(facing, 1.5) * (1.0 - 0.75 * u) * along.w * along.w * 1.8;
        c = vec3(1.0, 0.82, 0.55) * I;
        a = I * 0.4 * cover;
    } else if (shell == 3) { // with the afterburner, orange to white-hot deep in; dry, a dull red only by night
        float night = 1.0 - day;
        float glow = 0.25 * dry * dry * night * night + 0.55 * ab;
        vec3 hot = mix(vec3(1.0, 0.3, 0.05), vec3(1.0, 0.7, 0.4), 1.0 - u);
        vec3 col = mix(vec3(0.45, 0.03, 0.004), hot, clamp(ab * 1.5, 0.0, 1.0));
        c = col * glow * (0.55 + 0.45 * (1.0 - u)) * (0.9 + 0.2 * n);
    } else { // hot gas: a faint, fast-moving shimmer - pale, so it reads as heat, not smoke
        float s = (0.8 * dry + 0.4 * ab) * facing * pow(1.0 - u, 2.0) * smoothstep(0.0, 0.1, u);
        float grain = noise(vec3(xd * 5.0 - t * 22.0, ring * 3.0 + seed * 5.0));
        a = s * 0.03 * (0.2 + 1.6 * grain);
        c = vec3(0.62, 0.6, 0.56) * a * (0.3 + 0.7 * day);
    }
    outColor = vec4(c, a);
}
)";

} // namespace

double Exhaust::flameLength(double radius, double afterburner) {
    const double a = std::clamp(afterburner, 0.0, 1.0);
    return a > 0.0 ? 2.0 * radius * (1.2 + 4.6 * a) : 0.0;
}

double Exhaust::dryPower(double n2) {
    // JSBSim's turbines idle at about 60 % of their core speed: nothing to see
    // below three quarters, the hottest dry exhaust at full speed
    if (!std::isfinite(n2)) return 0.0;
    const double s = std::clamp((n2 - 75.0) / 25.0, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

Exhaust::Exhaust(std::size_t slots, vsg::ref_ptr<const vsg::Options> options) {
    (void)options;
    auto vertex = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", std::string(kVertex));
    auto fragment = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", std::string(kFragment));
    vsg::ShaderStages stages{vertex, fragment};
    // Compiled here rather than when the scene is: a shader that does not
    // build leaves the models their own flames instead of failing the scene.
    auto compiler = vsg::ShaderCompiler::create();
    if (!compiler->supported() || !compiler->compile(stages)) {
        LOG_WARN("world") << "exhaust shaders did not build; engines keep the model's own flames";
        return;
    }

    vsg::DescriptorSetLayoutBindings bindings{
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout},
                                              vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});

    auto input = vsg::VertexInputState::create();
    input->vertexBindingDescriptions.push_back(VkVertexInputBindingDescription{0, sizeof(vsg::vec4), VK_VERTEX_INPUT_RATE_VERTEX});
    input->vertexAttributeDescriptions.push_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0});
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_NONE; // both sides of every shell: the far wall of the gas counts too
    auto depth = vsg::DepthStencilState::create();
    depth->depthWriteEnable = VK_FALSE; // gas hides nothing behind it from the depth test
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{input, vsg::InputAssemblyState::create(), raster, vsg::MultisampleState::create(), blend, depth};
    auto pipeline = vsg::GraphicsPipeline::create(layout, stages, states);

    rows_ = vsg::vec4Array::create(static_cast<std::uint32_t>(std::max<std::size_t>(slots, 1) * kPerSlot * 2), vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    rows_->properties.dataVariance = vsg::DYNAMIC_DATA;
    frame_ = vsg::vec4Value::create(vsg::vec4(0.0f, 1.0f, 0.0f, 0.0f));
    frame_->properties.dataVariance = vsg::DYNAMIC_DATA;
    auto descriptors = vsg::DescriptorSet::create(
        setLayout, vsg::Descriptors{vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                    vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)});

    state_ = vsg::StateGroup::create();
    state_->add(vsg::BindGraphicsPipeline::create(pipeline));
    state_->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, descriptors));

    // The mesh: every shell's rings, u from the nozzle, one vertex per ring
    // point (the seam twice, so a ring closes), two triangles per quad.
    std::vector<vsg::vec4> points;
    std::vector<std::uint16_t> indices;
    for (int shell = 0; shell < kShells; ++shell) {
        const auto base = static_cast<std::uint16_t>(points.size());
        for (int i = 0; i < kRings[shell]; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(kRings[shell] - 1);
            for (int j = 0; j <= kSegments; ++j) {
                const double th = 2.0 * kPi * j / kSegments;
                points.emplace_back(u, static_cast<float>(std::cos(th)), static_cast<float>(std::sin(th)), static_cast<float>(shell));
            }
        }
        for (int i = 0; i + 1 < kRings[shell]; ++i)
            for (int j = 0; j < kSegments; ++j) {
                const auto a = static_cast<std::uint16_t>(base + i * (kSegments + 1) + j);
                const auto b = static_cast<std::uint16_t>(a + kSegments + 1);
                indices.insert(indices.end(), {a, b, static_cast<std::uint16_t>(a + 1), static_cast<std::uint16_t>(a + 1), b,
                                               static_cast<std::uint16_t>(b + 1)});
            }
    }
    auto vertices = vsg::vec4Array::create(static_cast<std::uint32_t>(points.size()));
    std::copy(points.begin(), points.end(), vertices->begin());
    auto index = vsg::ushortArray::create(static_cast<std::uint32_t>(indices.size()));
    std::copy(indices.begin(), indices.end(), index->begin());
    arrays_ = {vsg::BufferInfo::create(vertices)};
    indices_ = vsg::BufferInfo::create(index);
    indexCount_ = static_cast<std::uint32_t>(indices.size());

    flames_.resize(std::max<std::size_t>(slots, 1) * kPerSlot);
}

vsg::ref_ptr<vsg::Node> Exhaust::node(std::size_t slot, std::size_t k) {
    if (!state_ || k >= kPerSlot) return {};
    const std::size_t i = slot * kPerSlot + k;
    if (i >= flames_.size()) return {};
    Flame& f = flames_[i];
    if (!f.lod) {
        // Its own draw of the shared mesh, reading its own row of numbers: the
        // instance index is where the row is.
        auto draw = vsg::VertexIndexDraw::create();
        draw->arrays = arrays_;
        draw->indices = indices_;
        draw->indexCount = indexCount_;
        draw->instanceCount = 1;
        draw->firstInstance = static_cast<std::uint32_t>(i);
        auto group = vsg::StateGroup::create();
        group->stateCommands = state_->stateCommands;
        group->addChild(draw);
        f.lod = vsg::LOD::create();
        f.lod->addChild(vsg::LOD::Child{0.004, group}); // gone below a few pixels
        f.lod->bound = vsg::dsphere(0.0, 0.0, 0.0, 1.0);
    }
    return f.lod;
}

bool Exhaust::set(std::size_t slot, std::size_t k, double radius, const sim::VehicleState& s, int engine) {
    const std::size_t i = slot * kPerSlot + k;
    if (!state_ || k >= kPerSlot || i >= flames_.size()) return false;
    const bool ok = engine >= 0 && engine < s.engineCount && engine < sim::VehicleState::kMaxEngines;
    const double ab = ok && std::isfinite(s.afterburner[engine]) ? std::clamp(s.afterburner[engine], 0.0, 1.0) : 0.0;
    const double dry = ok ? dryPower(s.engineN2[engine]) : 0.0;
    const double nozzle = ok && std::isfinite(s.nozzlePosition[engine]) ? std::clamp(s.nozzlePosition[engine], 0.0, 1.0) : 0.0;
    const double r = std::isfinite(radius) && radius > 0.0 ? radius : 0.5;

    const vsg::vec4 a(static_cast<float>(r), static_cast<float>(ab), static_cast<float>(dry), static_cast<float>(nozzle));
    // a seed per engine, so two nozzles side by side do not flicker in step
    const float seed = static_cast<float>(std::fmod(static_cast<double>(i) * 0.618033988749895, 1.0));
    const vsg::vec4 b(seed, 0.0f, 0.0f, 0.0f);
    if (rows_->at(static_cast<std::uint32_t>(2 * i)) != a || rows_->at(static_cast<std::uint32_t>(2 * i + 1)) != b) {
        rows_->set(static_cast<std::uint32_t>(2 * i), a);
        rows_->set(static_cast<std::uint32_t>(2 * i + 1), b);
        changed_ = true;
    }
    Flame& f = flames_[i];
    if (f.lod && f.radius != r) {
        // Round everything it can draw: the flame at full stretch, some 6
        // diameters and its flicker.
        const double reach = 2.0 * r * 7.0;
        f.lod->bound = vsg::dsphere(0.5 * reach, 0.0, 0.0, 0.5 * reach + 3.0 * r);
        f.radius = r;
    }
    return ab > 0.0 || dry > 0.0;
}

void Exhaust::setFrame(double seconds, float daylight) {
    if (!frame_) return;
    // the shaders see seconds as a float: wrapped, so the flicker keeps its
    // speed however long the world has run
    const float t = static_cast<float>(std::fmod(std::isfinite(seconds) ? seconds : 0.0, 3600.0));
    const vsg::vec4 v(t, std::clamp(daylight, 0.0f, 1.0f), 0.0f, 0.0f);
    if (frame_->value() != v) {
        frame_->value() = v;
        frame_->dirty();
    }
}

void Exhaust::commit() {
    if (changed_ && rows_) rows_->dirty();
    changed_ = false;
}

} // namespace fsim::world
