#include "world/FlatGeometry.h"

#include "core/Log.h"

namespace fsim::world {

vsg::ref_ptr<vsg::StateGroup> createFlatStateGroup(const FlatGeometrySettings& settings, vsg::ref_ptr<const vsg::Options> options) {
    auto shaderSet = vsg::createFlatShadedShaderSet(options);
    if (!shaderSet) {
        LOG_ERROR("world") << "flat shaded ShaderSet unavailable";
        return {};
    }
    auto config = vsg::GraphicsPipelineConfigurator::create(shaderSet);

    if (const auto& materialBinding = shaderSet->getDescriptorBinding("material")) {
        config->assignDescriptor("material", materialBinding.data);
    }
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
                                             bool dynamic) {
    const std::uint32_t n = static_cast<std::uint32_t>(vertices->size());
    auto normals = vsg::vec3Array::create(n, vsg::vec3(0.0f, 0.0f, 1.0f));
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
