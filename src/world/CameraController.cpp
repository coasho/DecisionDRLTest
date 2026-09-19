#include "world/CameraController.h"

#include "world/Frames.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

CameraController::CameraController(vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::LookAt> lookAt,
                                   vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid)
    : camera_(camera), lookAt_(lookAt), ellipsoid_(ellipsoid) {
    trackball_ = vsg::Trackball::create(camera_, ellipsoid_);
    // Only the orbit mode listens to the mouse; keep the trackball's own
    // keyboard viewpoints out of the way of ours.
    trackball_->supportsThrow = false;
}

const char* CameraController::modeName(Mode m) noexcept {
    switch (m) {
    case Mode::Chase: return "chase";
    case Mode::Orbit: return "orbit";
    case Mode::Overview: return "overview";
    }
    return "?";
}

void CameraController::setMode(Mode mode) {
    if (mode == mode_) return;
    mode_ = mode;
    haveEye_ = false;
    if (mode_ == Mode::Orbit) {
        // Resume the trackball from wherever the chase camera left the view.
        trackball_->setViewpoint(vsg::LookAt::create(lookAt_->eye, lookAt_->center, lookAt_->up), 0.0);
    }
}

void CameraController::zoom(double factor) noexcept {
    behind_ = std::clamp(behind_ * factor, 8.0, 5000.0);
    above_ = std::clamp(above_ * factor, 2.0, 1500.0);
}

void CameraController::update(const sim::VehicleState& target, double dtSeconds) {
    const vsg::dvec3 pos = positionEcef(target);
    const vsg::dvec3 up = localUp(pos);

    switch (mode_) {
    case Mode::Orbit:
        return; // the trackball owns the view

    case Mode::Chase: {
        // Behind along the horizontal projection of the body x axis, above along local up.
        vsg::dvec3 fwd = bodyAxisEcef(target, 0);
        fwd = fwd - up * vsg::dot(fwd, up);
        if (vsg::length(fwd) < 1e-6) fwd = bodyAxisEcef(target, 1);
        fwd = vsg::normalize(fwd);

        const vsg::dvec3 desiredEye = pos - fwd * behind_ + up * above_;
        if (!haveEye_) {
            smoothedEye_ = desiredEye;
            haveEye_ = true;
        } else {
            // Critically damped-ish exponential smoothing, ~0.25 s time constant.
            const double a = 1.0 - std::exp(-dtSeconds / 0.12);
            smoothedEye_ = smoothedEye_ + (desiredEye - smoothedEye_) * a;
        }
        lookAt_->eye = smoothedEye_;
        lookAt_->center = pos;
        lookAt_->up = up;
        return;
    }

    case Mode::Overview: {
        const double height = std::max(500.0, behind_ * 40.0);
        lookAt_->eye = pos + up * height;
        lookAt_->center = pos;
        // "north-ish" as screen-up: project ECEF z onto the tangent plane.
        vsg::dvec3 north = vsg::dvec3(0.0, 0.0, 1.0) - up * up.z;
        lookAt_->up = vsg::length(north) > 1e-6 ? vsg::normalize(north) : bodyAxisEcef(target, 0);
        return;
    }
    }
}

} // namespace fsim::world
