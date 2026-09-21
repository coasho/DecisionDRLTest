#include "world/FlatGeometry.h"

#include "core/Log.h"
#include "world/Scattering.h"

namespace fsim::world {

vsg::ref_ptr<vsg::StateGroup> createFlatStateGroup(const FlatGeometrySettings& settings, vsg::ref_ptr<const vsg::Options> options) {
    auto shaderSet = settings.lit ? vsg::createPhongShaderSet(options) : vsg::createFlatShadedShaderSet(options);
    if (!shaderSet) {
        LOG_ERROR("world") << (settings.lit ? "phong" : "flat shaded") << " ShaderSet unavailable";
        return {};
    }
    // Only the lit path goes through VSG's Phong shader, which is what the
    // scattering is written against.
    if (settings.lit && settings.aerialPerspective) addAerialPerspective(*shaderSet);
    auto config = vsg::GraphicsPipelineConfigurator::create(shaderSet);

    auto material = vsg::PhongMaterialValue::create();
    material->value().diffuse = settings.diffuse;
    if (settings.lit) {
        material->value().specular = vsg::vec4(0.0f, 0.0f, 0.0f, 1.0f); // matte: snow and ice, not plastic
        material->value().ambient = settings.ambient;
    }
    config->assignDescriptor("material", material);
    config->enableArray("vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    config->enableArray("vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    config->enableArray("vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, 8);
    config->enableArray("vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, 16);

    struct Apply : public vsg::Visitor {
        const FlatGeometrySettings& s;
        explicit Apply(const FlatGeometrySettings& in) : s(in) {}
        void apply(vsg::Object& o) override { o.traverse(*this); }
        void apply(vsg::RasterizationState& rs) override {
            rs.cullMode = s.cullBackFaces ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
        }
        void apply(vsg::InputAssemblyState& ias) override { ias.topology = s.topology; }
        void apply(vsg::DepthStencilState& ds) override {
            ds.depthTestEnable = s.depthTest ? VK_TRUE : VK_FALSE;
            ds.depthWriteEnable = s.depthWrite ? VK_TRUE : VK_FALSE;
            ds.depthCompareOp = s.depthCompare;
        }
        void apply(vsg::ColorBlendState& cbs) override { cbs.configureAttachments(s.blending); }
    } apply(settings);
    config->accept(apply);

    config->init();

    auto stateGroup = vsg::StateGroup::create();
    vsg::StateCommands commands;
    if (!config->copyTo(commands)) {
        LOG_ERROR("world") << "flat pipeline setup failed";
        return {};
    }
    stateGroup->stateCommands.swap(commands);
    stateGroup->prototypeArrayState = config->getSuitableArrayState();
    return stateGroup;
}

vsg::ref_ptr<vsg::VertexDraw> createFlatDraw(vsg::ref_ptr<vsg::vec3Array> vertices, vsg::ref_ptr<vsg::vec4Array> colors,
                                             bool dynamic, vsg::ref_ptr<vsg::vec3Array> normals) {
    const std::uint32_t n = static_cast<std::uint32_t>(vertices->size());
    if (!normals) normals = vsg::vec3Array::create(n, vsg::vec3(0.0f, 0.0f, 1.0f));
    auto texcoords = vsg::vec2Array::create(n, vsg::vec2(0.0f, 0.0f));
    if (dynamic) {
        vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
        colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    }
    auto draw = vsg::VertexDraw::create();
    draw->assignArrays({vertices, normals, texcoords, colors});
    draw->vertexCount = n;
    draw->instanceCount = 1;
    return draw;
}

} // namespace fsim::world
