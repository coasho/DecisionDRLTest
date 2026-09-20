#include "world/CameraController.h"

#include "world/Frames.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kMinDistance = 6.0;
constexpr double kMaxDistance = 200000.0;
constexpr double kMinElevation = -20.0 * kDeg; // slightly below the vehicle
constexpr double kMaxElevation = 89.0 * kDeg;

double wrapAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
} // namespace

CameraController::CameraController(vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::LookAt> lookAt,
                                   vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid)
    : camera_(camera), lookAt_(lookAt), ellipsoid_(ellipsoid) {}

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
    // Keep the view where it is when switching between chase and orbit: the
    // azimuth reference changes (heading vs north), so convert.
    if (mode_ == Mode::Chase && mode == Mode::Orbit && haveHeading_) azimuth_ = wrapAngle(azimuth_ + smoothedHeading_);
    if (mode_ == Mode::Orbit && mode == Mode::Chase && haveHeading_) azimuth_ = wrapAngle(azimuth_ - smoothedHeading_);
    mode_ = mode;
}

void CameraController::setChaseOffset(double distanceM, double elevationDeg, double azimuthDeg) {
    defaultDistance_ = distance_ = std::clamp(distanceM, kMinDistance, kMaxDistance);
    defaultElevation_ = elevation_ = std::clamp(elevationDeg * kDeg, kMinElevation, kMaxElevation);
    defaultAzimuth_ = azimuth_ = wrapAngle(azimuthDeg * kDeg);
}

void CameraController::zoom(double factor) noexcept { distance_ = std::clamp(distance_ * factor, kMinDistance, kMaxDistance); }

void CameraController::resetView() noexcept {
    azimuth_ = defaultAzimuth_;
    elevation_ = defaultElevation_;
    distance_ = defaultDistance_;
}

void CameraController::apply(vsg::ButtonPressEvent& e) {
    if (e.handled) return; // ImGui has the mouse
    lastX_ = e.x;
    lastY_ = e.y;
    if (e.button == 1) leftDown_ = true;
    else if (e.button == 3) rightDown_ = true;
    else if (e.button == 2) resetView();
    e.handled = true;
}

void CameraController::apply(vsg::ButtonReleaseEvent& e) {
    if (e.button == 1) leftDown_ = false;
    else if (e.button == 3) rightDown_ = false;
}

void CameraController::apply(vsg::MoveEvent& e) {
    if (e.handled) return;
    const int dx = e.x - lastX_, dy = e.y - lastY_;
    lastX_ = e.x;
    lastY_ = e.y;
    if (leftDown_ && (e.mask & vsg::BUTTON_MASK_1)) {
        // The camera follows the mouse: drag right -> the eye moves right around
        // the target; drag down (screen y grows downward) -> the eye rises and
        // looks down on the target, as if pulling the view over it.
        azimuth_ = wrapAngle(azimuth_ + dx * 0.4 * kDeg);
        elevation_ = std::clamp(elevation_ + dy * 0.3 * kDeg, kMinElevation, kMaxElevation);
        e.handled = true;
    } else if (rightDown_ && (e.mask & vsg::BUTTON_MASK_3)) {
        distance_ = std::clamp(distance_ * std::exp(dy * 0.01), kMinDistance, kMaxDistance); // drag down -> further
        e.handled = true;
    }
}

void CameraController::apply(vsg::ScrollWheelEvent& e) {
    if (e.handled) return;
    distance_ = std::clamp(distance_ * std::pow(0.85, static_cast<double>(e.delta.y)), kMinDistance, kMaxDistance);
    e.handled = true;
}

void CameraController::update(const sim::VehicleState& target, double dtSeconds) {
    const vsg::dvec3 pos = positionEcef(target);
    const vsg::dvec3 up = localUp(pos);

    // Local horizontal frame at the target: east, north.
    vsg::dvec3 east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    const vsg::dvec3 north = vsg::cross(up, east);

    // Heading of the body x axis projected onto the horizontal plane.
    vsg::dvec3 fwd = bodyAxisEcef(target, 0);
    fwd = fwd - up * vsg::dot(fwd, up);
    double heading = 0.0;
    if (vsg::length(fwd) > 1e-6) {
        fwd = vsg::normalize(fwd);
        heading = std::atan2(vsg::dot(fwd, east), vsg::dot(fwd, north)); // 0 = north, +east
    }
    if (!haveHeading_) {
        smoothedHeading_ = heading;
        haveHeading_ = true;
    } else {
        const double a = 1.0 - std::exp(-dtSeconds / 0.35);
        smoothedHeading_ = wrapAngle(smoothedHeading_ + wrapAngle(heading - smoothedHeading_) * a);
    }

    if (mode_ == Mode::Overview) {
        const double height = std::max(200.0, distance_ * 10.0);
        lookAt_->eye = pos + up * height;
        lookAt_->center = pos;
        lookAt_->up = north;
        return;
    }

    // Direction from target to eye: azimuth measured clockwise from the reference
    // (heading or north), elevation above the horizon.
    const double reference = (mode_ == Mode::Chase) ? smoothedHeading_ : 0.0;
    const double az = reference + azimuth_;
    const vsg::dvec3 horizontal = north * std::cos(az) + east * std::sin(az);
    const vsg::dvec3 dir = horizontal * std::cos(elevation_) + up * std::sin(elevation_);
    lookAt_->eye = pos + dir * distance_;
    lookAt_->center = pos;
    lookAt_->up = up;
}

} // namespace fsim::world
