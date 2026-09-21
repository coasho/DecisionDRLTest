#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Unlit, per-vertex-coloured geometry on VSG's flat-shaded ShaderSet: the
/// building block for the sky dome, trails and markers (design 8.2).
struct FlatGeometrySettings {
    VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    bool depthTest = true;
    bool depthWrite = true;
    /// Reverse depth throughout (far = 0), so VSG's GREATER is the default. A
    /// pass that re-draws geometry an earlier pass already depth-wrote needs
    /// GREATER_OR_EQUAL, or every fragment fails against its own depth.
    VkCompareOp depthCompare = VK_COMPARE_OP_GREATER;
    bool blending = false;   ///< alpha blending (vertex colour alpha)
    bool cullBackFaces = true;
    /// Shade with the scene's lights instead of drawing at full brightness.
    /// An unlit surface among lit terrain reads as a hole cut in the world,
    /// which is what the polar caps used to look like.
    bool lit = false;
    /// Haze with distance like the terrain does (needs `lit`); see
    /// world/Scattering.h. Anything large and far that skips this reads as
    /// unnaturally clear next to ground that has it.
    bool aerialPerspective = false;
    /// Material colour; the default is VSG's. The flat shader writes
    /// `vsg_Color * diffuse`, so this is the whole output where the vertex
    /// colours are white - which is how the segmentation pass paints a
    /// vehicle its id colour.
    vsg::vec4 diffuse{0.9f, 0.9f, 0.9f, 1.0f};
    /// Ambient response, for the lit path. VSG's default is 1.0; the terrain
    /// tiles use 0.45, so anything meant to sit alongside them has to use the
    /// same or it glows where the sun is low - which at the poles is always.
    vsg::vec4 ambient{1.0f, 1.0f, 1.0f, 1.0f};
};

/// StateGroup binding a pipeline for `vsg_Vertex` (vec3), `vsg_Normal` (vec3),
/// `vsg_TexCoord0` (vec2) and per-vertex `vsg_Color` (vec4) arrays.
vsg::ref_ptr<vsg::StateGroup> createFlatStateGroup(const FlatGeometrySettings& settings, vsg::ref_ptr<const vsg::Options> options);

/// Draw command for the arrays above. Normals/texcoords are filled with
/// placeholders when not supplied. `dynamic` marks vertices and colours as
/// DYNAMIC_DATA so later edits (+ dirty()) reach the GPU.
vsg::ref_ptr<vsg::VertexDraw> createFlatDraw(vsg::ref_ptr<vsg::vec3Array> vertices, vsg::ref_ptr<vsg::vec4Array> colors,
                                             bool dynamic, vsg::ref_ptr<vsg::vec3Array> normals = {});

} // namespace fsim::world
