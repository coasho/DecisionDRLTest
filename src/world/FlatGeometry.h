#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Unlit, per-vertex-coloured geometry on VSG's flat-shaded ShaderSet: the
/// building block for the sky dome, trails and markers (design 8.2).
struct FlatGeometrySettings {
    VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    bool depthTest = true;
    bool depthWrite = true;
    bool blending = false;   ///< alpha blending (vertex colour alpha)
    bool cullBackFaces = true;
};

/// StateGroup binding a pipeline for `vsg_Vertex` (vec3), `vsg_Normal` (vec3),
/// `vsg_TexCoord0` (vec2) and per-vertex `vsg_Color` (vec4) arrays.
vsg::ref_ptr<vsg::StateGroup> createFlatStateGroup(const FlatGeometrySettings& settings, vsg::ref_ptr<const vsg::Options> options);

/// Draw command for the arrays above. Normals/texcoords are filled with
/// placeholders when not supplied. `dynamic` marks vertices and colours as
/// DYNAMIC_DATA so later edits (+ dirty()) reach the GPU.
vsg::ref_ptr<vsg::VertexDraw> createFlatDraw(vsg::ref_ptr<vsg::vec3Array> vertices, vsg::ref_ptr<vsg::vec4Array> colors,
                                             bool dynamic);

} // namespace fsim::world
