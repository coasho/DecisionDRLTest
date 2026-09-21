#include "world/CameraController.h"

#include "world/Frames.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kMinDistance = 6.0;          // following a vehicle
constexpr double kMinDistanceDetached = 25.0; // orbiting a ground point: stay clear of the drawn mesh
constexpr double kMaxDistance = 5.0e7;        // 50,000 km: the whole Earth with room around it
constexpr double kMinElevation = -60.0 * kDeg; // well below the focus (terrain collision takes over)
constexpr double kMaxElevation = 89.0 * kDeg;
constexpr double kRotateRadPerNdc = 1.0;      // osgGA rotateYawPitch: 1 rad per normalised unit (57 deg per half window)
constexpr double kPanPerNdc = 0.3;            // osgGA panModel scale
constexpr double kZoomPerNotch = 0.88;        // 12 % per wheel notch
constexpr double kZoomTimeConstant = 0.12;    // s, wheel/drag zoom smoothing
constexpr double kEyeClearanceM = 4.0;        // eye height above the sampled terrain at close range ...
constexpr double kEyeClearanceRatio = 0.03;   // ... plus 3 % of the distance (the drawn LOD gets coarser with range) ...
constexpr double kEyeClearanceMaxM = 80.0;    // ... up to this
constexpr double kGlobeRampStartM = 1.0e6;    // beyond this distance the view tilts towards straight down ...
constexpr double kGlobeRampEndM = 1.5e7;      // ... and is fully top-down here (the whole Earth in view)

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
    case Mode::Free: return "free";
    }
    return "?";
}

void CameraController::setMode(Mode mode) {
    if (mode == mode_) return;
    // Keep the view where it is when switching between chase and orbit: the
    // azimuth reference changes (heading vs north), so convert.
    if (mode_ == Mode::Chase && mode != Mode::Chase && haveHeading_) azimuth_ = wrapAngle(azimuth_ + smoothedHeading_);
    if (mode_ != Mode::Chase && mode == Mode::Chase && haveHeading_) azimuth_ = wrapAngle(azimuth_ - smoothedHeading_);
    if (mode == Mode::Free && hasTarget_) focus_ = lastTargetPos_; // detach where the vehicle is
    mode_ = mode;
}

void CameraController::setFreeView(double latitudeDeg, double longitudeDeg, double altitudeM, double distanceM, double azimuthDeg, double elevationDeg) {
    mode_ = Mode::Free;
    focus_ = ellipsoid_->convertLatLongAltitudeToECEF(vsg::dvec3(latitudeDeg, longitudeDeg, altitudeM));
    lastTargetPos_ = focus_;
    distance_ = targetDistance_ = std::clamp(distanceM, kMinDistance, kMaxDistance);
    azimuth_ = wrapAngle(azimuthDeg * kDeg);
    elevation_ = std::clamp(elevationDeg * kDeg, kMinElevation, kMaxElevation);
    panRight_ = panUp_ = 0.0;
}

void CameraController::setChaseOffset(double distanceM, double elevationDeg, double azimuthDeg) {
    defaultDistance_ = distance_ = targetDistance_ = std::clamp(distanceM, kMinDistance, kMaxDistance);
    defaultElevation_ = elevation_ = std::clamp(elevationDeg * kDeg, kMinElevation, kMaxElevation);
    defaultAzimuth_ = azimuth_ = wrapAngle(azimuthDeg * kDeg);
}

void CameraController::zoom(double factor) noexcept { targetDistance_ = std::clamp(targetDistance_ * factor, kMinDistance, kMaxDistance); }

void CameraController::resetView() noexcept {
    azimuth_ = defaultAzimuth_;
    elevation_ = defaultElevation_;
    distance_ = targetDistance_ = defaultDistance_;
    panRight_ = panUp_ = 0.0;
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
    // osgGA trackball: drag right -> the scene turns right (the eye goes left,
    // azimuth grows clockwise); drag up -> the near side rolls up (the eye sinks).
    azimuth_ = wrapAngle(azimuth_ + dxNdc * kRotateRadPerNdc);
    elevation_ = std::clamp(elevation_ - dyNdc * kRotateRadPerNdc, kMinElevation, kMaxElevation);
}

// Free camera: drag the ground with the right button. The focus moves in the
// horizontal plane so that the terrain under the cursor follows the mouse,
// and is kept on the ellipsoid surface (at terrain height when known).
void CameraController::moveFocus(double dxNdc, double dyNdc) {
    vsg::dvec3 east, north, up;
    localFrame(focus_, east, north, up);
    vsg::dvec3 right = viewRight_ - up * vsg::dot(viewRight_, up);
    vsg::dvec3 ahead = viewForward_ - up * vsg::dot(viewForward_, up);
    if (vsg::length(right) < 1e-6) right = east;
    if (vsg::length(ahead) < 1e-6) ahead = north;
    right = vsg::normalize(right);
    ahead = vsg::normalize(ahead);
    // Screen units to metres: half a window at the focus spans ~ distance * tan(fov/2) * aspect;
    // 0.5 x distance per normalised unit is close to "the ground sticks to the cursor" at 30 deg fov.
    const double scale = 0.5 * distance_;
    vsg::dvec3 moved = focus_ - right * (dxNdc * scale) - ahead * (dyNdc * scale);
    // Back onto the globe: keep latitude/longitude; the altitude follows the
    // terrain when known and otherwise stays what it was (update() corrects it
    // as soon as the tile arrives) - never 0, which would sink the focus
    // under high ground.
    vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(moved);
    double altitude = ellipsoid_->convertECEFToLatLongAltitude(focus_).z;
    if (ground_)
        if (auto h = ground_(lla.x * kDeg, lla.y * kDeg)) altitude = *h;
    lla.z = altitude;
    focus_ = ellipsoid_->convertLatLongAltitudeToECEF(lla);
}

void CameraController::localFrame(const vsg::dvec3& pos, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& up) const {
    up = localUp(pos);
    east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    north = vsg::cross(up, east);
}

void CameraController::apply(vsg::ButtonPressEvent& e) {
    if (e.handled) return; // ImGui has the mouse
    lastX_ = e.x;
    lastY_ = e.y;
    if (e.button == 1) leftDown_ = true;
    else if (e.button == 2) middleDown_ = true;
    else if (e.button == 3) rightDown_ = true;
    e.handled = true;
}

void CameraController::apply(vsg::ButtonReleaseEvent& e) {
    if (e.button == 1) leftDown_ = false;
    else if (e.button == 2) middleDown_ = false;
    else if (e.button == 3) rightDown_ = false;
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
        rotate(dx, dy);
        e.handled = true;
    } else if (middleDown_ && (e.mask & vsg::BUTTON_MASK_2)) {
        if (detached()) {
            moveFocus(dx, dy); // osgGA TerrainManipulator: the middle button pans over the terrain
        } else {
            // Following a vehicle: offset the look-at point in the screen plane (osgGA panModel, 0.3 x distance).
            panRight_ -= dx * kPanPerNdc * distance_;
            panUp_ -= dy * kPanPerNdc * distance_;
        }
        e.handled = true;
    } else if (rightDown_ && (e.mask & vsg::BUTTON_MASK_3)) {
        if (detached()) moveFocus(dx, dy); // rotate the globe under the camera
        else targetDistance_ = std::clamp(targetDistance_ * std::max(0.2, 1.0 + dy), kMinDistance, kMaxDistance); // osgGA zoomModel: drag down = closer
        e.handled = true;
    }
}

void CameraController::apply(vsg::ScrollWheelEvent& e) {
    if (e.handled) return;
    targetDistance_ = std::clamp(targetDistance_ * std::pow(kZoomPerNotch, static_cast<double>(e.delta.y)),
                                 detached() ? kMinDistanceDetached : kMinDistance, kMaxDistance);
    e.handled = true;
}

void CameraController::update(const sim::VehicleState* target, double dtSeconds) {
    hasTarget_ = target != nullptr;
    if (target) lastTargetPos_ = positionEcef(*target);
    const bool followVehicle = target && mode_ != Mode::Free;
    if (!followVehicle && ground_) {
        // Keep the detached focus on the terrain surface (tiles arrive over time).
        vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(focus_);
        if (auto h = ground_(lla.x * kDeg, lla.y * kDeg); h && std::abs(*h - lla.z) > 0.5) {
            lla.z = *h;
            focus_ = ellipsoid_->convertLatLongAltitudeToECEF(lla);
        }
    }
    const vsg::dvec3 pos = followVehicle ? positionEcef(*target) : focus_;

    // Smooth zoom towards the wheel/drag target.
    if (distance_ != targetDistance_) {
        const double a = dtSeconds > 0.0 ? 1.0 - std::exp(-dtSeconds / kZoomTimeConstant) : 1.0;
        distance_ += (targetDistance_ - distance_) * a;
        if (std::abs(distance_ - targetDistance_) < 1e-3 * targetDistance_) distance_ = targetDistance_;
    }

    vsg::dvec3 east, north, up;
    localFrame(pos, east, north, up);

    // Heading of the body x axis projected onto the horizontal plane.
    if (followVehicle) {
        vsg::dvec3 fwd = bodyAxisEcef(*target, 0);
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
    }

    if (mode_ == Mode::Overview && followVehicle) {
        const double height = std::max(200.0, distance_ * 10.0);
        lookAt_->eye = pos + up * height;
        lookAt_->center = pos;
        lookAt_->up = north;
        viewRight_ = east;
        viewForward_ = -up;
        viewUp_ = north;
        return;
    }

    // Direction from focus to eye: azimuth measured clockwise from the reference
    // (heading or north), elevation above the horizon.
    const double reference = (mode_ == Mode::Chase && followVehicle) ? smoothedHeading_ : 0.0;
    const double az = reference + azimuth_;
    const vsg::dvec3 horizontal = north * std::cos(az) + east * std::sin(az);

    double elevation = elevation_;

    // Far out, tilt towards straight down so the globe is seen from above its
    // focus rather than from an angle that puts the eye past the horizon.
    if (distance_ > kGlobeRampStartM) {
        const double t = std::clamp((distance_ - kGlobeRampStartM) / (kGlobeRampEndM - kGlobeRampStartM), 0.0, 1.0);
        const double s = t * t * (3.0 - 2.0 * t);
        elevation = std::max(elevation, elevation + (kMaxElevation - elevation) * s);
    }
    // Beyond the terrain-collision range the eye must stay above the focus'
    // horizontal plane, or a negative elevation would put it under the globe.
    if (distance_ >= 2.0e5) elevation = std::max(elevation, 0.0);

    // The pan offset moves the eye too: probe the terrain from where the eye really orbits.
    vsg::dvec3 orbitCentre = pos;
    if (panRight_ != 0.0 || panUp_ != 0.0) {
        const vsg::dvec3 dir0 = horizontal * std::cos(elevation) + up * std::sin(elevation);
        vsg::dvec3 right0 = vsg::cross(-dir0, up);
        if (vsg::length(right0) < 1e-6) right0 = east;
        right0 = vsg::normalize(right0);
        const vsg::dvec3 screenUp0 = vsg::normalize(vsg::cross(right0, -dir0));
        orbitCentre = pos + right0 * panRight_ + screenUp0 * panUp_;
    }

    // Terrain collision: keep the eye (and the line of sight down to the focus)
    // above the ground by raising the elevation angle; the orbit stays
    // consistent and the view tilts down over the terrain instead of entering
    // it. The ground is sampled at several points between focus and eye so a
    // ridge in between is respected too.
    if (ground_ && distance_ < 2.0e5) {
        const double focusAltitude = ellipsoid_->convertECEFToLatLongAltitude(orbitCentre).z;
        const double clearance = std::min(kEyeClearanceMaxM, kEyeClearanceM + kEyeClearanceRatio * distance_);
        // Detached, the focus is on the ground: never look up at it from below.
        double minElevation = followVehicle ? -kPi / 2.0 : std::asin(std::clamp(clearance / distance_, -1.0, 1.0));
        for (double t : {0.15, 0.3, 0.5, 0.75, 1.0}) {
            const vsg::dvec3 probeDir = horizontal * std::cos(elevation) + up * std::sin(elevation);
            const vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(orbitCentre + probeDir * (distance_ * t));
            if (auto h = ground_(lla.x * kDeg, lla.y * kDeg)) {
                const double needed = (*h + clearance) - focusAltitude; // height to gain over the focus by that point
                minElevation = std::max(minElevation, std::asin(std::clamp(needed / (distance_ * t), -1.0, 1.0)));
            }
        }
        if (elevation < minElevation) elevation = std::min(minElevation, kMaxElevation);
    }
    // What is shown is what the next drag starts from: no dead zone after the
    // terrain or the globe view pushed the elevation up.
    elevation_ = elevation;

    const vsg::dvec3 dir = horizontal * std::cos(elevation) + up * std::sin(elevation);
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
    viewRight_ = right;
    viewForward_ = forward;
    viewUp_ = screenUp;
}

} // namespace fsim::world
