#include "world/CameraController.h"

#include "world/Frames.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kMinDistance = 6.0;
constexpr double kMaxDistance = 200000.0;
constexpr double kMinElevation = -60.0 * kDeg; // well below the vehicle (terrain may occlude)
constexpr double kMaxElevation = 89.0 * kDeg;
constexpr double kRotateRadPerNdc = 1.25;     // osgGA trackball (size 0.8): ~72 deg per half window
constexpr double kPanPerNdc = 0.3;            // osgGA panModel scale

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
    panRight_ = panUp_ = 0.0;
    stopThrow();
}

// Window pixels -> normalised device coordinates in [-1, 1] (x right, y up),
// as osgGA does, so the response is independent of window size and DPI.
void CameraController::normalised(int x, int y, double& nx, double& ny) const {
    const VkViewport vp = camera_ ? camera_->getViewport() : VkViewport{};
    const double w = vp.width > 1.0f ? vp.width : 1.0, h = vp.height > 1.0f ? vp.height : 1.0;
    nx = (2.0 * x - w) / w;
    ny = (h - 2.0 * y) / h;
}

void CameraController::rotate(double dxNdc, double dyNdc) {
    // osgGA trackball of size 0.8: ~1.25 rad per normalised unit (half the
    // window). Drag right -> the scene turns right (the eye goes left, azimuth
    // grows clockwise); drag up -> the near side rolls up (the eye sinks).
    azimuth_ = wrapAngle(azimuth_ + dxNdc * kRotateRadPerNdc);
    elevation_ = std::clamp(elevation_ - dyNdc * kRotateRadPerNdc, kMinElevation, kMaxElevation);
}

void CameraController::apply(vsg::ButtonPressEvent& e) {
    if (e.handled) return; // ImGui has the mouse
    stopThrow();           // any click stops a thrown rotation (osgGA)
    lastX_ = e.x;
    lastY_ = e.y;
    lastMoveTime_ = e.time;
    lastMoveSeconds_ = 0.0;
    if (e.button == 1) leftDown_ = true;
    else if (e.button == 2) middleDown_ = true;
    else if (e.button == 3) rightDown_ = true;
    e.handled = true;
}

void CameraController::apply(vsg::ButtonReleaseEvent& e) {
    if (e.button == 1) {
        leftDown_ = false;
        // Throw (osgGA): released while still moving -> keep rotating at the
        // last measured rate until the next button press.
        const double sinceMove = std::chrono::duration<double>(e.time - lastMoveTime_).count();
        if (sinceMove < 0.08 && lastMoveSeconds_ > 0.0) {
            throwAzimuth_ = lastMoveAzimuth_ / lastMoveSeconds_;
            throwElevation_ = lastMoveElevation_ / lastMoveSeconds_;
            thrown_ = std::abs(throwAzimuth_) + std::abs(throwElevation_) > 0.05; // rad/s
        }
    } else if (e.button == 2) {
        middleDown_ = false;
    } else if (e.button == 3) {
        rightDown_ = false;
    }
}

void CameraController::apply(vsg::MoveEvent& e) {
    if (e.handled) return;
    double x0, y0, x1, y1;
    normalised(lastX_, lastY_, x0, y0);
    normalised(e.x, e.y, x1, y1);
    const double dx = x1 - x0, dy = y1 - y0; // y up, as in osgGA
    lastX_ = e.x;
    lastY_ = e.y;
    if (leftDown_ && (e.mask & vsg::BUTTON_MASK_1)) {
        const double before = azimuth_, beforeEl = elevation_;
        rotate(dx, dy);
        lastMoveAzimuth_ = wrapAngle(azimuth_ - before);
        lastMoveElevation_ = elevation_ - beforeEl;
        lastMoveSeconds_ = std::max(1e-3, std::chrono::duration<double>(e.time - lastMoveTime_).count());
        lastMoveTime_ = e.time;
        e.handled = true;
    } else if (middleDown_ && (e.mask & vsg::BUTTON_MASK_2)) {
        // Pan: the scene follows the mouse (osgGA panModel with scale 0.3 x distance).
        panRight_ -= dx * kPanPerNdc * distance_;
        panUp_ -= dy * kPanPerNdc * distance_;
        e.handled = true;
    } else if (rightDown_ && (e.mask & vsg::BUTTON_MASK_3)) {
        // Zoom (osgGA zoomModel): distance *= 1 + dy; drag down = closer.
        distance_ = std::clamp(distance_ * std::max(0.2, 1.0 + dy), kMinDistance, kMaxDistance);
        e.handled = true;
    }
}

void CameraController::apply(vsg::ScrollWheelEvent& e) {
    if (e.handled) return;
    stopThrow();
    distance_ = std::clamp(distance_ * std::pow(0.9, static_cast<double>(e.delta.y)), kMinDistance, kMaxDistance); // 10 % per notch
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

    // A thrown rotation keeps turning at the release rate (osgGA "throw").
    if (thrown_) {
        azimuth_ = wrapAngle(azimuth_ + throwAzimuth_ * dtSeconds);
        const double el = std::clamp(elevation_ + throwElevation_ * dtSeconds, kMinElevation, kMaxElevation);
        if (el == elevation_ && throwElevation_ != 0.0) throwElevation_ = 0.0; // hit the limit: stop that component
        elevation_ = el;
    }

    // Direction from target to eye: azimuth measured clockwise from the reference
    // (heading or north), elevation above the horizon.
    const double reference = (mode_ == Mode::Chase) ? smoothedHeading_ : 0.0;
    const double az = reference + azimuth_;
    const vsg::dvec3 horizontal = north * std::cos(az) + east * std::sin(az);
    const vsg::dvec3 dir = horizontal * std::cos(elevation_) + up * std::sin(elevation_);
    // Pan offset in the eye's screen plane (right = forward x up, screen up = right x forward).
    const vsg::dvec3 forward = -dir;
    vsg::dvec3 right = vsg::cross(forward, up);
    if (vsg::length(right) < 1e-6) right = east;
    right = vsg::normalize(right);
    const vsg::dvec3 screenUp = vsg::normalize(vsg::cross(right, forward));
    const vsg::dvec3 centre = pos + right * panRight_ + screenUp * panUp_;
    lookAt_->eye = centre + dir * distance_;
    lookAt_->center = centre;
    lookAt_->up = up;
}

} // namespace fsim::world
