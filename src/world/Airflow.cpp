#include "world/Airflow.h"

#include "core/Log.h"
#include "world/Exhaust.h"
#include "world/Frames.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace fsim::world {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kVortexLife = 1.6, kContrailLife = 45.0; // s; the shaders fade them over the same
constexpr double kVapourRange = 3000.0, kStreakRange = 2500.0, kVortexRange = 15000.0, kContrailRange = 60000.0; // m from the eye

double smoothstep(double e0, double e1, double x) {
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// Condensation needs moisture, and there is less of it the higher you go.
double humidity(double altitudeM) { return 1.0 - 0.65 * smoothstep(2000.0, 9000.0, altitudeM); }

double finite(double v, double fallback = 0.0) { return std::isfinite(v) ? v : fallback; }

// What every stage may read: VSG's matrices, the trail samples, the frame
// (time, daylight, sun), the vehicles' vapour rows and air-streak rows.
const char* kCommon = R"(
layout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;
layout(std430, set = 0, binding = 0) readonly buffer Samples { vec4 samples[]; };
layout(set = 0, binding = 1) uniform Frame { vec4 frame[3]; }; // [0] now (s), daylight; [1] the sun, view space; [2] sky light
layout(std430, set = 0, binding = 2) readonly buffer Rows { vec4 rows[]; };
layout(std430, set = 0, binding = 3) readonly buffer Streaks { vec4 streakRows[]; };

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
// Water droplets in the light: white by day, glowing towards the sun, grey
// by night. (Linear colour: the window is sRGB.)
vec3 vapourLight(vec3 towards) {
    float day = frame[0].y;
    float glow = pow(max(dot(normalize(towards), frame[1].xyz), 0.0), 6.0);
    return mix(vec3(0.035, 0.04, 0.06), frame[2].rgb, day) * (1.0 + 0.9 * glow * day);
}
// Unlit vapour is not seen: without the sun it thins out rather than
// turning into dark smoke.
float vapourSeen() { return 0.25 + 0.75 * frame[0].y; }
)";

const char* kRibbonVertex = R"(
layout(location = 0) out vec4 v_data; // across (-1 .. 1), where along the trail (its birth time), contrail, opacity
layout(location = 1) out vec3 v_view;
out gl_PerVertex { vec4 gl_Position; };
const int CAP = 128;

void main() {
    int src = gl_InstanceIndex / CAP, i = gl_InstanceIndex - src * CAP;
    int ia = src * CAP + i, ib = src * CAP + (i + 1) % CAP;
    vec4 a0 = samples[2 * ia], a1 = samples[2 * ia + 1]; // xyz from the eye, birth; strength, width, contrail, count
    vec4 b0 = samples[2 * ib], b1 = samples[2 * ib + 1];
    // a segment joins two samples taken one after the other: a gap in the
    // count is a gap in the trail (and the ring's newest-to-oldest seam)
    int corner = gl_VertexIndex;
    bool atB = corner == 1 || corner == 4 || corner == 5;
    float side = (corner == 2 || corner == 3 || corner == 5) ? 1.0 : -1.0;
    vec4 p0 = atB ? b0 : a0, p1 = atB ? b1 : a1;
    bool contrail = p1.z > 0.5;
    float age = max(frame[0].x - p0.w, 0.0);
    float life = contrail ? 45.0 : 1.6;
    float t = clamp(age / life, 0.0, 1.0);
    // a vortex's core swells as it decays; a contrail spreads for a minute,
    // starting a little way behind the nozzle, where the exhaust has cooled
    float width = contrail ? p1.y + 0.3 * age + 1.2 * sqrt(age) : p1.y * (1.0 + 2.5 * t) + 0.2 * age;
    float alpha = p1.x * (contrail ? smoothstep(0.05, 0.3, age) * pow(1.0 - t, 1.3) : smoothstep(0.0, 0.06, age) * pow(1.0 - t, 1.4));
    if (a1.w < 0.0 || b1.w != a1.w + 1.0 || max(a1.x, b1.x) <= 0.0 || t >= 1.0) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        v_data = vec4(0.0);
        v_view = vec3(0.0);
        return;
    }
    vec3 va = (pc.modelView * vec4(a0.xyz, 1.0)).xyz, vb = (pc.modelView * vec4(b0.xyz, 1.0)).xyz;
    vec3 me = atB ? vb : va;
    vec3 across = cross(vb - va, me); // square to the segment and to the line of sight: the ribbon faces the eye
    float l = length(across);
    across = l > 1e-9 ? across / l : vec3(0.0, 1.0, 0.0);
    vec3 v = me + across * side * 0.5 * width;
    v_view = v;
    v_data = vec4(side, p0.w, contrail ? 1.0 : 0.0, alpha);
    gl_Position = pc.projection * vec4(v, 1.0);
}
)";

const char* kRibbonFragment = R"(
layout(location = 0) in vec4 v_data;
layout(location = 1) in vec3 v_view;
layout(location = 0) out vec4 outColor;

void main() {
    float across = v_data.x;
    bool contrail = v_data.z > 0.5;
    // round in section: dense along its core, thin at its edges; torn along its length
    float profile = pow(max(1.0 - across * across, 0.0), contrail ? 1.0 : 2.0);
    float n = noise(vec3(v_data.y * (contrail ? 1.1 : 7.0), across * 1.7, contrail ? 7.0 : 0.0));
    float n2 = noise(vec3(v_data.y * (contrail ? 4.3 : 23.0), across * 4.1, 3.0));
    float a = v_data.w * profile * (0.45 + 0.8 * n * (0.6 + 0.8 * n2)) * (contrail ? 0.95 : 0.7) * vapourSeen();
    outColor = vec4(vapourLight(v_view) * a, a);
}
)";

// Vapour on and about the airframe, in its body frame: sheets over the wings
// (kind 0), ropes off the leading-edge extensions (1), the cone round it near
// Mach 1 (2). A vertex is a point on the sheet or the axis, and an offset the
// shader swells (its normal's way).
const char* kVapourVertex = R"(
layout(location = 0) in vec3 base;
layout(location = 1) in vec3 offset;
layout(location = 2) in vec4 info; // kind, s (along), t (across), side
layout(location = 0) out vec3 v_view;
layout(location = 1) out vec3 v_normal;
layout(location = 2) out vec4 v_info; // kind, s, t, strength
layout(location = 3) flat out float v_seed;
out gl_PerVertex { vec4 gl_Position; };

void main() {
    vec4 r0 = rows[2 * gl_InstanceIndex], r1 = rows[2 * gl_InstanceIndex + 1];
    int kind = int(info.x + 0.5);
    float strength = kind == 0 ? (info.w > 0.0 ? r0.x : r0.y) : kind == 1 ? r0.z : r0.w;
    if (strength <= 0.0) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        v_view = vec3(0.0); v_normal = vec3(0.0, 0.0, 1.0); v_info = vec4(0.0); v_seed = 0.0;
        return;
    }
    float time = frame[0].x;
    float swell = 1.0;
    if (kind == 1) // a vortex rope thickens along its length and bursts at its end
        swell = (0.35 + 1.1 * info.y) * (0.85 + 0.3 * noise(vec3(info.y * 9.0 - time * 12.0, info.z * 2.0, r1.x * 5.0)));
    else if (kind == 2)
        swell = 0.97 + 0.06 * noise(vec3(info.y * 3.0 - time * 2.0, info.z * 6.0, r1.x));
    vec4 v = pc.modelView * vec4(base + offset * swell, 1.0);
    v_view = v.xyz;
    v_normal = mat3(pc.modelView) * normalize(offset);
    v_info = vec4(info.x, info.y, info.z, strength);
    v_seed = r1.x;
    gl_Position = pc.projection * v;
}
)";

const char* kVapourFragment = R"(
layout(location = 0) in vec3 v_view;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec4 v_info;
layout(location = 3) flat in float v_seed;
layout(location = 0) out vec4 outColor;

void main() {
    int kind = int(v_info.x + 0.5);
    float s = v_info.y, t = v_info.z, strength = v_info.w;
    vec3 toEye = normalize(-v_view);
    float facing = abs(dot(normalize(v_normal), toEye));
    float time = frame[0].x;
    float a;
    if (kind == 0) { // over the wing: a sheet thickest behind the leading edge, streaked along the flow, flickering
        float streak = noise(vec3(s * 38.0 + v_seed * 17.0, t * 1.6 - time * 7.0, 0.0));
        float grain = noise(vec3(s * 11.0, t * 5.0 - time * 16.0, v_seed * 5.0 + time));
        float shape = pow(1.0 - t, 1.8) * smoothstep(0.0, 0.08, t) * smoothstep(0.02, 0.2, s) * (1.0 - smoothstep(0.65, 0.95, s));
        a = strength * shape * (0.35 + 0.65 * streak) * (0.6 + 0.4 * grain) * 0.75;
    } else if (kind == 1) { // a rope of vapour twisting off the leading-edge extension
        float twist = noise(vec3(s * 18.0 - time * 28.0, t * 4.0 + s * 6.0, v_seed * 9.0));
        float along = smoothstep(0.0, 0.12, s) * (1.0 - smoothstep(0.6, 1.0, s));
        a = strength * along * pow(facing, 1.2) * (0.3 + 0.9 * twist) * 0.6;
    } else { // the vapour cone: a shell of cloud, sharp at its front, streaked and torn up behind
        float n = noise(vec3(s * 7.0 - time * 3.0, t * 14.0, v_seed * 3.0));
        float n2 = noise(vec3(s * 17.0 - time * 7.0, t * 31.0, v_seed * 7.0 + 2.0));
        float body = smoothstep(0.0, 0.05, s) * pow(1.0 - s, 2.2);
        a = strength * body * (0.45 + 0.3 * (1.0 - facing)) * smoothstep(0.15, 0.75, n * (0.6 + 0.6 * n2)) * 0.85;
    }
    a *= vapourSeen();
    outColor = vec4(vapourLight(-toEye) * a, a);
}
)";

// Air streaks: a fixed cloud of seeds wrapped round each aircraft in a box
// that stays put in the air, so the aircraft flies through it; each seed is
// drawn as the streak it would leave in a short exposure.
const char* kStreakVertex = R"(
layout(location = 0) out vec3 v_data; // across, along (0 tail .. 1 head), opacity
layout(location = 1) out vec3 v_view;
out gl_PerVertex { vec4 gl_Position; };

uint pcg(uint v) {
    uint state = v * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}
vec3 seed3(uint i) { return vec3(pcg(i), pcg(i + 0x9e3779b9u), pcg(i + 0x7f4a7c15u)) * (1.0 / 4294967295.0); }

void main() {
    int particle = gl_VertexIndex / 6, corner = gl_VertexIndex - particle * 6;
    vec4 r0 = streakRows[3 * gl_InstanceIndex], r1 = streakRows[3 * gl_InstanceIndex + 1], r2 = streakRows[3 * gl_InstanceIndex + 2];
    float box = r0.w; // the aircraft at r0.xyz from the eye; its airspeed r1.xyz, strength r1.w; where it is in the box r2.xyz
    vec3 rel = (fract(seed3(uint(particle)) - r2.xyz / box) - 0.5) * box;
    float fade = 1.0 - smoothstep(0.28, 0.5, length(rel) / box);
    // the air flows back past the aircraft: a moment ago this bit of it was
    // further forward, so that is where its streak trails off to
    vec3 head = r0.xyz + rel, tail = head + r1.xyz * r2.w;
    bool atHead = corner == 1 || corner == 4 || corner == 5;
    float side = (corner == 2 || corner == 3 || corner == 5) ? 1.0 : -1.0;
    vec3 vh = (pc.modelView * vec4(head, 1.0)).xyz, vt = (pc.modelView * vec4(tail, 1.0)).xyz;
    vec3 me = atHead ? vh : vt;
    vec3 across = cross(vh - vt, me);
    float l = length(across);
    across = l > 1e-9 ? across / l : vec3(0.0, 1.0, 0.0);
    float dist = length(me);
    fade *= smoothstep(6.0, 20.0, dist); // none in the camera's face
    vec3 v = me + across * side * max(0.012, 0.0008 * dist);
    v_view = v;
    v_data = vec3(side, atHead ? 1.0 : 0.0, r1.w * fade);
    gl_Position = pc.projection * vec4(v, 1.0);
}
)";

const char* kStreakFragment = R"(
layout(location = 0) in vec3 v_data;
layout(location = 1) in vec3 v_view;
layout(location = 0) out vec4 outColor;

void main() {
    float a = v_data.z * pow(max(1.0 - v_data.x * v_data.x, 0.0), 1.5) * smoothstep(0.0, 0.6, v_data.y) * 0.18 * vapourSeen();
    outColor = vec4(vapourLight(v_view) * a, a);
}
)";

vsg::ref_ptr<vsg::ShaderStage> stage(VkShaderStageFlagBits kind, const char* body) {
    return vsg::ShaderStage::create(kind, "main", std::string("#version 450\n") + kCommon + body);
}

/// A pipeline over the shared layout: premultiplied blending, depth tested
/// but not written (vapour and gas hide nothing), both faces.
vsg::ref_ptr<vsg::BindGraphicsPipeline> pipeline(vsg::ref_ptr<vsg::PipelineLayout> layout, vsg::ShaderStages stages,
                                                 vsg::ref_ptr<vsg::VertexInputState> input) {
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_NONE;
    auto depth = vsg::DepthStencilState::create();
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{input, vsg::InputAssemblyState::create(), raster, vsg::MultisampleState::create(), blend, depth};
    return vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, stages, states));
}

} // namespace

double Airflow::vortexStrength(double tipAlpha, double nz, double altitudeM) {
    return smoothstep(0.14, 0.30, tipAlpha) * smoothstep(1.8, 4.0, nz) * humidity(altitudeM);
}

double Airflow::wingVapour(double alpha, double nz, double altitudeM) {
    return smoothstep(0.17, 0.33, alpha) * smoothstep(3.5, 7.0, nz) * humidity(altitudeM);
}

double Airflow::vapourCone(double mach, double altitudeM) {
    // it takes the humid air down low
    return smoothstep(0.94, 0.98, mach) * (1.0 - smoothstep(0.995, 1.035, mach)) * (1.0 - smoothstep(800.0, 3500.0, altitudeM));
}

double Airflow::contrail(double altitudeM, double seaLevelK, double power) {
    // the standard lapse to the tropopause, isothermal above; contrails form
    // in air colder than about -40 C (the Schmidt-Appleman criterion)
    const double t = seaLevelK - 0.0065 * std::clamp(altitudeM, 0.0, 11000.0);
    return smoothstep(236.0, 226.0, t) * (0.35 + 0.65 * std::clamp(power, 0.0, 1.0));
}

double Airflow::streaks(double airspeedMs) { return smoothstep(60.0, 320.0, airspeedMs); }

Airflow::Airflow(std::size_t slots, vsg::ref_ptr<const vsg::Options> options) {
    (void)options;
    root_ = vsg::Group::create();
    switch_ = vsg::Switch::create();
    root_->addChild(switch_);

    vsg::ShaderStages ribbon{stage(VK_SHADER_STAGE_VERTEX_BIT, kRibbonVertex), stage(VK_SHADER_STAGE_FRAGMENT_BIT, kRibbonFragment)};
    vsg::ShaderStages vapour{stage(VK_SHADER_STAGE_VERTEX_BIT, kVapourVertex), stage(VK_SHADER_STAGE_FRAGMENT_BIT, kVapourFragment)};
    vsg::ShaderStages streak{stage(VK_SHADER_STAGE_VERTEX_BIT, kStreakVertex), stage(VK_SHADER_STAGE_FRAGMENT_BIT, kStreakFragment)};
    auto compiler = vsg::ShaderCompiler::create();
    if (!compiler->supported() || !compiler->compile(ribbon) || !compiler->compile(vapour) || !compiler->compile(streak)) {
        LOG_WARN("world") << "airflow shaders did not build; no vapour, contrails or air streaks";
        return;
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
                                              {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr}};
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout},
                                              vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});

    samples_ = vsg::vec4Array::create(static_cast<std::uint32_t>(kSources * kSamples * 2), vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
    rows_ = vsg::vec4Array::create(static_cast<std::uint32_t>(std::max<std::size_t>(slots, 1) * 2), vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    streakRows_ = vsg::vec4Array::create(static_cast<std::uint32_t>(kStreakVehicles * 3), vsg::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    frame_ = vsg::vec4Array::create(3, vsg::vec4(0.0f, 1.0f, 0.0f, 0.0f));
    for (auto& d : {samples_, rows_, streakRows_, frame_}) d->properties.dataVariance = vsg::DYNAMIC_DATA;
    auto descriptors = vsg::DescriptorSet::create(
        setLayout, vsg::Descriptors{vsg::DescriptorBuffer::create(samples_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                    vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                                    vsg::DescriptorBuffer::create(rows_, 2, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                    vsg::DescriptorBuffer::create(streakRows_, 3, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)});
    auto bindSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, descriptors);

    // the trails and the streaks: no vertex input, the shaders make every vertex
    auto none = vsg::VertexInputState::create();
    auto trails = vsg::StateGroup::create();
    trails->add(pipeline(layout, ribbon, none));
    trails->add(bindSet);
    ribbonDraw_ = vsg::Draw::create(6, 0, 0, 0);
    trails->addChild(ribbonDraw_);
    ribbonAt_ = vsg::MatrixTransform::create();
    ribbonAt_->addChild(trails);

    auto air = vsg::StateGroup::create();
    air->add(pipeline(layout, streak, none));
    air->add(bindSet);
    streakDraw_ = vsg::Draw::create(static_cast<std::uint32_t>(6 * kStreaks), 0, 0, 0);
    air->addChild(streakDraw_);
    streakAt_ = vsg::MatrixTransform::create();
    streakAt_->addChild(air);

    // the vapour: a mesh per model (base, offset, info)
    auto input = vsg::VertexInputState::create();
    input->vertexBindingDescriptions = {VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
                                        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
                                        VkVertexInputBindingDescription{2, sizeof(vsg::vec4), VK_VERTEX_INPUT_RATE_VERTEX}};
    input->vertexAttributeDescriptions = {VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                          VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                          VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32B32A32_SFLOAT, 0}};
    vapourState_ = {pipeline(layout, vapour, input), bindSet};
    vapourRoot_ = vsg::Group::create();

    // The vapour first, then the trails, then the streaks: they blend over
    // one another in that order, drawn after the scene they sit in.
    switch_->addChild(true, vapourRoot_);
    switch_->addChild(true, ribbonAt_);
    switch_->addChild(true, streakAt_);
    state_ = trails;

    sources_.resize(kSources);
    vapour_.resize(slots);
}

void Airflow::setVisible(bool on) {
    if (on == visible_) return;
    visible_ = on;
    switch_->setAllChildren(on);
    if (!on)
        for (auto& s : sources_) s = Source{};
}

Airflow::Source* Airflow::source(int slot, int kind, int which, bool create) {
    Source* free = nullptr;
    for (auto& s : sources_) {
        if (s.slot == slot && s.kind == kind && s.which == which) return &s;
        if (!free && s.slot < 0) free = &s;
    }
    if (!free || !create) return nullptr;
    *free = Source{};
    free->slot = slot;
    free->kind = kind;
    free->which = which;
    free->points.assign(kSamples, vsg::dvec4(0.0, 0.0, 0.0, 0.0));
    free->values.assign(kSamples, vsg::dvec3(0.0, 0.0, -1.0));
    return free;
}

void Airflow::feed(Source& s, const vsg::dvec3& at, double time, double strength, double width, double interval) {
    s.used = true;
    const std::size_t newest = (s.head + kSamples - 1) % kSamples;
    const bool on = strength > 1e-3;
    const bool wasOn = s.count > 0 && s.values[newest].x > 1e-3;
    if (!on && !wasOn) return; // nothing drawn and nothing to draw: the trail stays broken off where it faded
    const bool restart = on && !wasOn;
    if (s.count == 0 || restart || time - s.lastCommit >= interval) {
        // a new sample, which then follows the emitter until the next one; a
        // trail that starts again skips a count, so it is not joined to
        // where it stopped
        s.sequence = s.count == 0 ? 0.0 : s.sequence + (restart ? 2.0 : 1.0);
        s.points[s.head] = vsg::dvec4(at.x, at.y, at.z, time);
        s.values[s.head] = vsg::dvec3(strength, width, s.sequence);
        s.head = (s.head + 1) % kSamples;
        s.count = std::min(s.count + 1, kSamples);
        s.lastCommit = time;
        return;
    }
    // the newest sample stands at the emitter: the trail reaches the aircraft
    s.points[newest] = vsg::dvec4(at.x, at.y, at.z, time);
    s.values[newest].x = strength;
    s.values[newest].y = width;
}

const Airflow::Mesh* Airflow::mesh(const VehicleVisuals::Shape& shape) {
    auto it = meshes_.find(&shape);
    if (it != meshes_.end()) return &it->second;
    std::vector<vsg::vec3> bases, offsets;
    std::vector<vsg::vec4> infos;
    std::vector<std::uint16_t> indices;
    // a grid of (rows x cols) vertices, two triangles per cell
    auto grid = [&](int rows, int cols, const auto& at) {
        const auto first = static_cast<std::uint16_t>(bases.size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j) {
                vsg::vec3 b, o;
                vsg::vec4 info;
                at(i, j, b, o, info);
                bases.push_back(b);
                offsets.push_back(o);
                infos.push_back(info);
            }
        for (int i = 0; i + 1 < rows; ++i)
            for (int j = 0; j + 1 < cols; ++j) {
                const auto a = static_cast<std::uint16_t>(first + i * cols + j), b = static_cast<std::uint16_t>(a + cols);
                indices.insert(indices.end(), {a, b, static_cast<std::uint16_t>(a + 1), static_cast<std::uint16_t>(a + 1), b,
                                               static_cast<std::uint16_t>(b + 1)});
            }
    };
    auto f = [](const vsg::dvec3& v) { return vsg::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)); };

    const VehicleVisuals::Surface* wing = nullptr;
    const VehicleVisuals::Surface* strake = nullptr;
    for (const auto& s : shape.surfaces) {
        if (s.kind == "wing" && !wing) wing = &s;
        if (s.kind == "strake" && !strake) strake = &s;
    }
    // along a surface's leading edge, by span fraction: the edge's point and the chord there
    auto edge = [](const VehicleVisuals::Surface& s, double fraction, vsg::dvec3& le, double& chord) {
        const double y0 = s.sections.front().y, y1 = s.sections.back().y;
        const double y = y0 + fraction * (y1 - y0);
        std::size_t k = 0;
        while (k + 2 < s.sections.size() && s.sections[k + 1].y < y) ++k;
        const auto& a = s.sections[k];
        const auto& b = s.sections[k + 1];
        const double u = std::abs(b.y - a.y) > 1e-9 ? std::clamp((y - a.y) / (b.y - a.y), 0.0, 1.0) : 0.0;
        le = vsg::dvec3(a.x + u * (b.x - a.x), y, a.z + u * (b.z - a.z));
        chord = a.w + u * (b.w - a.w);
    };
    const double span = std::max(shape.hi.y - shape.lo.y, 1.0);
    if (wing) {
        // over the upper surface (body z is down), a little clear of it
        for (const double side : {1.0, -1.0})
            grid(11, 7, [&](int i, int j, vsg::vec3& b, vsg::vec3& o, vsg::vec4& info) {
                const double s = i / 10.0, t = j / 6.0;
                vsg::dvec3 le;
                double chord = 0.0;
                edge(*wing, s, le, chord);
                const double lift = (0.04 + 0.05 * std::sin(kPi * t)) * chord;
                b = f(vsg::dvec3(le.x - t * chord, side * le.y, le.z - lift));
                o = vsg::vec3(0.0f, 0.0f, -0.02f); // its "normal", up
                info = vsg::vec4(0.0f, static_cast<float>(s), static_cast<float>(t), static_cast<float>(side));
            });
    }
    if (strake && wing) {
        // off the strake's leading edge, where it meets the wing, and on back
        // over the wing root, rising: a rope of 16 rings of 12
        const auto& apex = strake->sections[std::min<std::size_t>(1, strake->sections.size() - 1)];
        const auto& join = strake->sections.back();
        vsg::dvec3 rootLe;
        double rootChord = 0.0;
        edge(*wing, 0.0, rootLe, rootChord);
        const vsg::dvec3 p0(apex.x, apex.y, apex.z - 0.1), p1(join.x, join.y, join.z - 0.3);
        const vsg::dvec3 p2(join.x - 0.6 * rootChord, join.y * 1.1, join.z - 0.6);
        const double radius = std::clamp(0.025 * span, 0.08, 0.3);
        for (const double side : {1.0, -1.0})
            grid(16, 13, [&](int i, int j, vsg::vec3& b, vsg::vec3& o, vsg::vec4& info) {
                const double s = i / 15.0, th = 2.0 * kPi * j / 12.0;
                const double u = s * 2.0;
                const vsg::dvec3 axis = u < 1.0 ? p0 + (p1 - p0) * u : p1 + (p2 - p1) * (u - 1.0);
                b = f(vsg::dvec3(axis.x, side * axis.y, axis.z));
                o = f(vsg::dvec3(0.0, std::cos(th), std::sin(th)) * radius);
                info = vsg::vec4(1.0f, static_cast<float>(s), static_cast<float>(j / 12.0), static_cast<float>(side));
            });
    }
    {
        // the cone: from about the wing's root leading edge, flaring back past
        // its trailing edge, round the fuselage (body x through the middle)
        vsg::dvec3 rootLe(0.25 * shape.hi.x, 0.0, 0.0);
        double rootChord = 0.3 * (shape.hi.x - shape.lo.x);
        if (wing) edge(*wing, 0.0, rootLe, rootChord);
        const double half = 0.5 * span;
        const double x0 = rootLe.x + 0.15 * rootChord, x1 = rootLe.x - 1.25 * rootChord;
        const double zc = 0.5 * (shape.lo.z + shape.hi.z) * 0.3 + rootLe.z * 0.7;
        grid(14, 33, [&](int i, int j, vsg::vec3& b, vsg::vec3& o, vsg::vec4& info) {
            const double s = i / 13.0, th = 2.0 * kPi * j / 32.0;
            const double r = half * (0.28 + 0.5 * std::pow(s, 0.7));
            b = f(vsg::dvec3(x0 + s * (x1 - x0), 0.0, zc));
            o = f(vsg::dvec3(0.12 * r, std::cos(th) * r, std::sin(th) * r * 0.8));
            info = vsg::vec4(2.0f, static_cast<float>(s), static_cast<float>(j / 32.0), 1.0f);
        });
    }
    Mesh m;
    auto va = vsg::vec3Array::create(static_cast<std::uint32_t>(bases.size()));
    auto vo = vsg::vec3Array::create(static_cast<std::uint32_t>(offsets.size()));
    auto vi = vsg::vec4Array::create(static_cast<std::uint32_t>(infos.size()));
    std::copy(bases.begin(), bases.end(), va->begin());
    std::copy(offsets.begin(), offsets.end(), vo->begin());
    std::copy(infos.begin(), infos.end(), vi->begin());
    auto ix = vsg::ushortArray::create(static_cast<std::uint32_t>(indices.size()));
    std::copy(indices.begin(), indices.end(), ix->begin());
    m.arrays = {vsg::BufferInfo::create(va), vsg::BufferInfo::create(vo), vsg::BufferInfo::create(vi)};
    m.indices = vsg::BufferInfo::create(ix);
    m.count = static_cast<std::uint32_t>(indices.size());
    const vsg::dvec3 centre = (shape.lo + shape.hi) * 0.5;
    m.bound = vsg::dsphere(centre, 0.75 * vsg::length(shape.hi - shape.lo) + 2.0);
    return &meshes_.emplace(&shape, std::move(m)).first->second;
}

Airflow::Vapour* Airflow::vapour(std::size_t slot, const VehicleVisuals::Shape& shape) {
    if (slot >= vapour_.size()) return nullptr;
    Vapour& v = vapour_[slot];
    if (!v.transform) {
        v.transform = vsg::MatrixTransform::create();
        v.sw = vsg::Switch::create();
        v.lod = vsg::LOD::create();
        v.draw = vsg::VertexIndexDraw::create();
        v.draw->instanceCount = 1;
        v.draw->firstInstance = static_cast<std::uint32_t>(slot);
        auto group = vsg::StateGroup::create();
        group->stateCommands = vapourState_;
        group->addChild(v.draw);
        v.lod->addChild(vsg::LOD::Child{0.01, group});
        v.sw->addChild(false, v.lod);
        v.transform->addChild(v.sw);
    }
    if (v.shape != &shape) {
        const Mesh* m = mesh(shape);
        v.draw->arrays = m->arrays;
        v.draw->indices = m->indices;
        v.draw->indexCount = m->count;
        v.lod->bound = m->bound;
        const bool fresh = v.shape == nullptr;
        v.shape = &shape;
        if (fresh) vapourRoot_->addChild(v.transform);
        // made after the scene was compiled: compile it (a new model's mesh too)
        if (compiler_ && !compiler_(v.transform)) LOG_WARN("world") << "could not compile a vehicle's vapour";
    }
    return &v;
}

void Airflow::update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                     const vsg::dmat4& view, const vsg::dvec3& eye, const vsg::dvec3& sun, float daylight) {
    if (!state_ || !visible_) return;
    const std::size_t n = std::min({states.size(), alive.size(), visuals.count()});

    // the world's clock: from the vehicles, the same for all of them
    double now = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < n && !std::isfinite(now); ++i)
        if (alive[i] && std::isfinite(states[i].simTime)) now = states[i].simTime;
    if (!std::isfinite(now)) now = lastTime_ >= 0.0 ? lastTime_ : 0.0;
    if (now < lastTime_ - 0.5) // a reset or a replay's seek: the trails were somewhere else
        for (auto& s : sources_) s = Source{};
    lastTime_ = now;
    if (now - timeBase_ > 3000.0 || now < timeBase_) timeBase_ = now; // keep the shaders' float times small

    // the frame: time, daylight, the sun in view space, the sky's light
    {
        const vsg::dvec3 s = vsg::normalize(sun);
        const vsg::dvec4 turned = view * vsg::dvec4(s.x, s.y, s.z, 0.0); // a direction: turned, not moved
        const vsg::dvec3 sv = vsg::normalize(vsg::dvec3(turned.x, turned.y, turned.z));
        // low sun: the light goes warm, as the sky dome does
        const float warm = static_cast<float>(1.0 - smoothstep(0.1, 0.4, vsg::dot(s, vsg::normalize(eye))));
        frame_->set(0, vsg::vec4(static_cast<float>(now - timeBase_), daylight, 0.0f, 0.0f));
        frame_->set(1, vsg::vec4(static_cast<float>(sv.x), static_cast<float>(sv.y), static_cast<float>(sv.z), 0.0f));
        frame_->set(2, vsg::vec4(0.92f, 0.93f - 0.13f * warm, 0.95f - 0.3f * warm, 0.0f));
        frame_->dirty();
    }

    // nearest first: they get the trails when there are more than can be kept
    std::vector<std::pair<double, std::size_t>> order;
    for (std::size_t i = 0; i < n; ++i)
        if (alive[i] && visuals.visible(i) && !states[i].diverged && visuals.shape(i).valid)
            order.emplace_back(vsg::length(positionEcef(states[i]) - eye), i);
    std::sort(order.begin(), order.end());

    for (auto& s : sources_) s.used = false;
    std::vector<unsigned char> vapourShown(vapour_.size(), 0);
    std::size_t streaking = 0;
    for (const auto& [distance, i] : order) {
        const auto& st = states[i];
        const auto& shape = visuals.shape(i);
        const vsg::dmat4 m = bodyToEcef(st);
        const double v = std::max(finite(st.airspeedTrueMs), 1.0);
        const double alpha = finite(st.alphaRad), nz = finite(st.loadFactor, 1.0), altitude = finite(st.altitudeMslM);

        if (distance < kVortexRange && shape.tip.y > 0.5) {
            for (int side = 0; side < 2; ++side) {
                const double y = side == 0 ? shape.tip.y : -shape.tip.y;
                // rolling raises one wing into the air and the other out of it
                const double tipAlpha = alpha + finite(st.angularRateBodyRadS[0]) * y / std::max(v, 30.0);
                const double strength = vortexStrength(tipAlpha, nz, altitude);
                if (Source* src = source(static_cast<int>(i), 0, side, strength > 0.0))
                    feed(*src, m * vsg::dvec3(shape.tip.x, y, shape.tip.z), now, strength,
                         std::clamp(0.012 * (shape.hi.y - shape.lo.y), 0.08, 0.3), 1.0 / 30.0);
            }
        }
        if (distance < kContrailRange) {
            for (std::size_t k = 0; k < shape.jets.size(); ++k) {
                const auto& jet = shape.jets[k];
                const int e = jet.engine;
                const bool ok = e >= 0 && e < st.engineCount && e < sim::VehicleState::kMaxEngines;
                const double power = ok ? std::max(Exhaust::dryPower(st.engineN2[e]), finite(st.afterburner[e])) : 0.0;
                const double strength = contrail(altitude, seaLevelK_, power);
                if (Source* src = source(static_cast<int>(i), 1, static_cast<int>(k), strength > 0.0))
                    feed(*src, m * (jet.exit + jet.direction * 0.5), now, strength, 1.2 * jet.radius + 0.4, 0.35);
            }
        }
        if (distance < kVapourRange) {
            const double wr = wingVapour(alpha + finite(st.angularRateBodyRadS[0]) * 0.5 * shape.tip.y / std::max(v, 30.0), nz, altitude);
            const double wl = wingVapour(alpha - finite(st.angularRateBodyRadS[0]) * 0.5 * shape.tip.y / std::max(v, 30.0), nz, altitude);
            const double lex = smoothstep(0.24, 0.42, alpha) * smoothstep(1.5, 4.0, nz) * humidity(altitude);
            const double cone = vapourCone(finite(st.mach), altitude);
            const vsg::vec4 row(static_cast<float>(wr), static_cast<float>(wl), static_cast<float>(lex), static_cast<float>(cone));
            if (row != vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f) || (i < vapour_.size() && vapour_[i].shape)) {
                rows_->set(static_cast<std::uint32_t>(2 * i), row);
                rows_->set(static_cast<std::uint32_t>(2 * i + 1),
                           vsg::vec4(static_cast<float>(std::fmod(static_cast<double>(i) * 0.618033988749895, 1.0)), 0.0f, 0.0f, 0.0f));
            }
            if (row != vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f))
                if (Vapour* vp = vapour(i, shape)) {
                    vp->transform->matrix = m;
                    vapourShown[i] = 1;
                }
        }
        const double airflow = streaks(v);
        if (distance < kStreakRange && airflow > 0.0 && streaking < kStreakVehicles) {
            // the box: some three lengths of the aircraft, fixed in the air
            const double box = std::clamp(2.5 * vsg::length(shape.hi - shape.lo), 25.0, 90.0);
            const vsg::dvec3 p = positionEcef(st);
            const vsg::dvec3 phase(p.x - box * std::floor(p.x / box), p.y - box * std::floor(p.y / box), p.z - box * std::floor(p.z / box));
            // the air past the aircraft: its velocity through the air, in ECEF
            const vsg::dvec3 wind(std::cos(alpha) * std::cos(finite(st.betaRad)), std::sin(finite(st.betaRad)),
                                  std::sin(alpha) * std::cos(finite(st.betaRad)));
            const vsg::dvec3 air = (m * (wind * v)) - (m * vsg::dvec3(0.0, 0.0, 0.0));
            const vsg::dvec3 at = p - eye;
            const auto k = static_cast<std::uint32_t>(3 * streaking);
            streakRows_->set(k, vsg::vec4(static_cast<float>(at.x), static_cast<float>(at.y), static_cast<float>(at.z), static_cast<float>(box)));
            streakRows_->set(k + 1, vsg::vec4(static_cast<float>(air.x), static_cast<float>(air.y), static_cast<float>(air.z), static_cast<float>(airflow)));
            // a streak is the air's path over 1/80 s: its length reads as the speed
            streakRows_->set(k + 2, vsg::vec4(static_cast<float>(phase.x), static_cast<float>(phase.y), static_cast<float>(phase.z), 0.0125f));
            ++streaking;
        }
    }
    for (std::size_t i = 0; i < vapour_.size(); ++i)
        if (vapour_[i].sw) vapour_[i].sw->children[0].mask = vapourShown[i] ? vsg::MASK_ALL : vsg::MASK_OFF;
    rows_->dirty();
    streakRows_->dirty();
    streakDraw_->instanceCount = static_cast<std::uint32_t>(streaking);
    streakAt_->matrix = vsg::translate(eye);

    // the trails: a source not fed lives on until its trail has faded away
    std::size_t used = 0;
    for (std::size_t k = 0; k < sources_.size(); ++k) {
        Source& s = sources_[k];
        if (s.slot < 0) continue;
        const double life = s.kind == 1 ? kContrailLife : kVortexLife;
        const std::size_t newest = (s.head + kSamples - 1) % kSamples;
        if (!s.used && (s.count == 0 || now - s.points[newest].w > life)) {
            s = Source{};
            continue;
        }
        used = k + 1;
    }
    for (std::size_t k = 0; k < used; ++k) {
        const Source& s = sources_[k];
        for (std::size_t j = 0; j < kSamples; ++j) {
            const auto at = static_cast<std::uint32_t>(2 * (k * kSamples + j));
            if (s.slot < 0 || s.values.empty() || s.values[j].z < 0.0 || j >= s.points.size()) {
                samples_->set(at + 1, vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
                continue;
            }
            const vsg::dvec3 rel = vsg::dvec3(s.points[j].x, s.points[j].y, s.points[j].z) - eye;
            samples_->set(at, vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z),
                                        static_cast<float>(s.points[j].w - timeBase_)));
            samples_->set(at + 1, vsg::vec4(static_cast<float>(s.values[j].x), static_cast<float>(s.values[j].y),
                                            static_cast<float>(s.kind), static_cast<float>(s.values[j].z)));
        }
    }
    if (used > 0) samples_->dirty();
    ribbonDraw_->instanceCount = static_cast<std::uint32_t>(used * kSamples);
    ribbonAt_->matrix = vsg::translate(eye);
}

} // namespace fsim::world
