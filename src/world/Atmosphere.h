#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// The shell of air around the globe (design 8.2 "Sky / atmosphere"), for the
/// view from outside it: a blue rim that thickens towards the limb, brightest
/// where it faces the sun and gone round the night side.
///
/// One low-poly sphere, drawn once, with the colour and opacity of every
/// vertex worked out on the CPU and only when the eye has actually moved. So
/// it costs a single draw of a few thousand vertices and nothing per pixel
/// beyond the blend - no extra pass, no depth read, no scattering integral.
///
/// It is the counterpart of SkyDome, which paints the sky from *inside* the
/// air. The two never both matter: this one fades in above `kFadeInStartM`,
/// by which altitude the dome has nearly finished fading to black.
class Atmosphere {
public:
    Atmosphere(vsg::ref_ptr<const vsg::Options> options, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
               const vsg::dvec3& sunDirectionEcef);

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Recolour for the eye's position (cheap, and skipped unless the view has
    /// moved enough to show).
    void update(const vsg::dvec3& eyeEcef);

    void setSun(const vsg::dvec3& sunDirectionEcef) {
        sun_ = vsg::normalize(sunDirectionEcef);
        coloured_ = false;
    }

private:
    void colour(const vsg::dvec3& eyeEcef, float strength);

    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::vec3Array> vertices_; ///< ECEF, on the shell
    vsg::ref_ptr<vsg::vec3Array> normals_;  ///< outward, = normalize(vertex)
    vsg::ref_ptr<vsg::vec4Array> colors_;
    vsg::dvec3 sun_;
    vsg::dvec3 lastEye_{0.0, 0.0, 0.0};
    float lastStrength_ = -1.0f;
    bool coloured_ = false;
};

} // namespace fsim::world
