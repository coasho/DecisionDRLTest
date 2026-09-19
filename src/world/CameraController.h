#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

namespace fsim::world {

/// Camera strategies (design 8.2 "Cameras"):
///  - Chase:  behind and above the selected vehicle, smoothed, up = local up
///  - Orbit:  VSG Trackball anchored on the ellipsoid; the user flies the camera
///  - Overview: high above the selected vehicle looking straight down
/// Switching modes is seamless: the trackball resumes from the current view.
class CameraController {
public:
    enum class Mode { Chase, Orbit, Overview };

    CameraController(vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::LookAt> lookAt,
                     vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

    /// The trackball is a VSG event handler; register it with the viewer.
    vsg::ref_ptr<vsg::Trackball> trackball() const { return trackball_; }

    void setMode(Mode mode);
    Mode mode() const noexcept { return mode_; }
    void cycleMode() { setMode(mode_ == Mode::Chase ? Mode::Orbit : mode_ == Mode::Orbit ? Mode::Overview : Mode::Chase); }

    /// Chase distance behind and height above the target, metres.
    void setChaseOffset(double behindM, double aboveM) noexcept { behind_ = behindM; above_ = aboveM; }
    void zoom(double factor) noexcept;

    /// Per-frame update with the followed vehicle's latest state.
    void update(const sim::VehicleState& target, double dtSeconds);

    static const char* modeName(Mode m) noexcept;

private:
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid_;
    vsg::ref_ptr<vsg::Trackball> trackball_;
    Mode mode_ = Mode::Chase;
    double behind_ = 40.0;
    double above_ = 12.0;
    vsg::dvec3 smoothedEye_;
    bool haveEye_ = false;
};

} // namespace fsim::world
