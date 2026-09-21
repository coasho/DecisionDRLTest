#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Eye-centred gradient sky (design 8.2 "Sky / atmosphere"): a sphere that
/// follows the camera, oriented to the local vertical, coloured from zenith
/// blue to horizon haze with a warm glow around the sun and a darkened
/// night side. Drawn without depth test/write as the first child of the scene
/// so everything else paints over it; no textures, no custom shaders.
class SkyDome {
public:
    SkyDome(vsg::ref_ptr<const vsg::Options> options, const vsg::dvec3& sunDirectionEcef, double radiusM = 60000.0);

    vsg::ref_ptr<vsg::Node> node() const { return transform_; }

    /// Move the dome to the eye and align it with the local up there.
    void update(const vsg::dvec3& eyeEcef);

    /// Change the sun (recolours on the next update).
    void setSun(const vsg::dvec3& sunDirectionEcef) {
        sun_ = vsg::normalize(sunDirectionEcef);
        coloured_ = false;
    }

private:
    void colour(const vsg::dvec3& upEcef, float space);

    vsg::ref_ptr<vsg::MatrixTransform> transform_;
    vsg::ref_ptr<vsg::vec3Array> vertices_; ///< unit sphere, local frame (z up)
    vsg::ref_ptr<vsg::vec4Array> colors_;
    vsg::dvec3 sun_;
    double radius_;
    vsg::dvec3 lastUp_;
    float lastSpace_ = -1.0f; ///< 0 = inside the atmosphere, 1 = black sky from orbit
    bool coloured_ = false;
};

} // namespace fsim::world
