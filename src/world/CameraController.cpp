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
    // Starting from what was drawn rather than from elevation_ keeps the first
    // part of a drag from doing nothing whenever the distance tilt or the
    // terrain has raised the view above what was last asked for.
    azimuth_ = wrapAngle(azimuth_ + dxNdc * kRotateRadPerNdc);
    elevation_ = std::clamp(shownElevation_ - dyNdc * kRotateRadPerNdc, kMinElevation, kMaxElevation);
}

void CameraController::moveFocusTo(const vsg::dvec3& newFocus, Carry carry) {
    if (vsg::length(newFocus) < 1.0) return;
    vsg::dvec3 east, north, up;
    localFrame(focus_, east, north, up);
    // The direction from the focus out to the eye, in world terms.
    const vsg::dvec3 horizontal = north * std::cos(azimuth_) + east * std::sin(azimuth_);
    const vsg::dvec3 toEye = horizontal * std::cos(elevation_) + up * std::sin(elevation_);

    vsg::dvec3 wanted = toEye;
    if (carry == Carry::WithGround) {
        // Turn the view by the same angle the focus travelled over the globe.
        // The local frame is rebuilt from the Earth's axis rather than carried
        // along, so the two are not the same thing: near a pole the frame
        // swings through half a turn while the ground does not, and it is that
        // difference this removes.
        const vsg::dvec3 a = vsg::normalize(focus_), b = vsg::normalize(newFocus);
        const double cosTurn = std::clamp(vsg::dot(a, b), -1.0, 1.0);
        if (cosTurn < 1.0 - 1e-15) {
            const vsg::dvec3 axis = vsg::cross(a, b);
            if (vsg::length(axis) > 1e-12) wanted = vsg::rotate(std::acos(cosTurn), vsg::normalize(axis)) * toEye;
        }
    }

    // Re-measure whichever direction that is against the frame the focus has now.
    focus_ = newFocus;
    localFrame(focus_, east, north, up);
    elevation_ = std::clamp(std::asin(std::clamp(vsg::dot(wanted, up), -1.0, 1.0)), kMinElevation, kMaxElevation);
    const vsg::dvec3 flat = wanted - up * vsg::dot(wanted, up);
    if (vsg::length(flat) > 1e-9) azimuth_ = wrapAngle(std::atan2(vsg::dot(flat, east), vsg::dot(flat, north)));
}

// Free camera: drag the ground with the middle button, so the terrain under
// the cursor follows the mouse.
//
// This is a rotation of the focus about the centre of the Earth, not a
// displacement reprojected through latitude/longitude. The difference only
// shows at the poles, and there it is the whole story: a metre of easting at
// 89.5 deg is a hundred times more longitude than at the equator, so a
// reprojected drag swung the focus' longitude wildly, which swung the local
// east/north frame the view direction is built from, which changed what the
// next drag meant - the globe spun out of control. A rotation has no such
// singularity.
//
// The same rotation is applied to the view direction, so the camera keeps
// looking the way it was: dragging past a pole no longer spins the world.
void CameraController::moveFocus(double dxNdc, double dyNdc) {
    vsg::dvec3 east, north, up;
    localFrame(focus_, east, north, up);
    vsg::dvec3 right = viewRight_ - up * vsg::dot(viewRight_, up);
    vsg::dvec3 ahead = viewForward_ - up * vsg::dot(viewForward_, up);
    if (vsg::length(right) < 1e-6) right = east;
    if (vsg::length(ahead) < 1e-6) ahead = north;
    right = vsg::normalize(right);
    ahead = vsg::normalize(ahead);
    // osgEarth's ACTION_PAN scale, so dragging the globe moves it the same
    // distance per pixel as it does there.
    const double scale = kPanPerNdc * distance_;
    const vsg::dvec3 step = -right * (dxNdc * scale) - ahead * (dyNdc * scale);
    const double arc = vsg::length(step);
    const double radius = vsg::length(focus_);
    if (arc < 1e-9 || radius < 1.0) return;

    const vsg::dvec3 axis = vsg::normalize(vsg::cross(focus_, step));
    // Dragging: the view goes with the ground under the cursor.
    moveFocusTo(vsg::rotate(arc / radius, axis) * focus_, Carry::WithGround); // arc length -> angle at the centre

    // Back onto the surface: the altitude follows the terrain when known and
    // otherwise stays what it was (update() corrects it as soon as the tile
    // arrives) - never 0, which would sink the focus under high ground.
    vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(focus_);
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
    // Only the left (rotate) and middle (pan / drag the globe) buttons are
    // used; the right button is left to the window manager.
    if (e.button == 1) leftDown_ = true;
    else if (e.button == 2) middleDown_ = true;
    else return;
    e.handled = true;
}

void CameraController::apply(vsg::ButtonReleaseEvent& e) {
    if (e.button == 1) leftDown_ = false;
    else if (e.button == 2) middleDown_ = false;
}

void CameraController::apply(vsg::MoveEvent& e) {
    double x0, y0, x1, y1;
    normalised(lastX_, lastY_, x0, y0);
    normalised(e.x, e.y, x1, y1);
    const double dx = x1 - x0, dy = y1 - y0; // y up, as in osgGA
    // Track the cursor even when a panel took the event. ScrollWheelEvent
    // carries no position, so this is the only record of where the pointer
    // is, and zooming towards a position last seen before the mouse crossed
    // a panel sends the view somewhere nobody asked for.
    lastX_ = e.x;
    lastY_ = e.y;
    if (e.handled) return;
    if (leftDown_ && (e.mask & vsg::BUTTON_MASK_1)) {
        // osgEarth binds the left button to pan.
        if (detached()) {
            moveFocus(dx, dy); // drag the globe: the ground follows the cursor
        } else {
            // Following a vehicle: offset the look-at point in the screen plane (osgGA panModel, 0.3 x distance).
            panRight_ -= dx * kPanPerNdc * distance_;
            panUp_ -= dy * kPanPerNdc * distance_;
        }
        e.handled = true;
    } else if (middleDown_ && (e.mask & vsg::BUTTON_MASK_2)) {
        rotate(dx, dy); // osgEarth binds the middle button to rotate
        e.handled = true;
    }
}

std::optional<vsg::dvec3> CameraController::groundUnderCursor() const {
    if (!camera_ || !camera_->projectionMatrix || !ellipsoid_) return std::nullopt;
    double nx = 0.0, ny = 0.0;
    normalised(lastX_, lastY_, nx, ny);

    // Unproject the cursor to a ray. The eye is the origin; the near-plane
    // point gives the direction, which avoids the far plane, where an
    // ellipsoid-fitted projection is ill-conditioned.
    const vsg::dmat4 inverseViewProj =
        vsg::inverse(camera_->projectionMatrix->transform() * camera_->viewMatrix->transform());
    const vsg::dvec4 nearPoint = inverseViewProj * vsg::dvec4(nx, ny, 0.0, 1.0);
    if (std::abs(nearPoint.w) < 1e-12) return std::nullopt;
    const vsg::dvec3 onNearPlane(nearPoint.x / nearPoint.w, nearPoint.y / nearPoint.w, nearPoint.z / nearPoint.w);
    const vsg::dvec3 eye = lookAt_->eye;
    const vsg::dvec3 ray = onNearPlane - eye;
    if (vsg::length(ray) < 1e-9) return std::nullopt;
    const vsg::dvec3 direction = vsg::normalize(ray);

    // Ellipsoid intersection, done in the space where it is the unit sphere.
    const double a = ellipsoid_->radiusEquator(), b = ellipsoid_->radiusPolar();
    if (a <= 0.0 || b <= 0.0) return std::nullopt;
    const vsg::dvec3 o(eye.x / a, eye.y / a, eye.z / b);
    const vsg::dvec3 d(direction.x / a, direction.y / a, direction.z / b);
    const double qa = vsg::dot(d, d), qb = 2.0 * vsg::dot(o, d), qc = vsg::dot(o, o) - 1.0;
    const double discriminant = qb * qb - 4.0 * qa * qc;
    if (discriminant < 0.0 || qa < 1e-300) return std::nullopt; // the cursor is off the globe
    const double t = (-qb - std::sqrt(discriminant)) / (2.0 * qa);
    if (t <= 0.0) return std::nullopt;
    return eye + direction * t;
}

void CameraController::zoomTowardsCursor(double fromDistance, double toDistance) {
    zoomShare_ = 0.0;
    if (!zoomToCursor_ || !detached() || fromDistance <= 0.0 || toDistance <= 0.0) return;
    const auto ground = groundUnderCursor();
    if (!ground) return;

    // Two things have to be said about a point picked off the globe at range.
    //
    // Near the silhouette the ray only grazes the surface, and a pixel of
    // cursor movement slides the hit point hundreds of kilometres. Zooming
    // towards a point that unstable throws the globe about, so it is refused.
    const vsg::dvec3 surfaceNormal = vsg::normalize(*ground);
    const vsg::dvec3 rayDirection = vsg::normalize(*ground - lookAt_->eye);
    if (-vsg::dot(rayDirection, surfaceNormal) < 0.05) return; // within ~3 deg of the silhouette

    // And holding a far-off point exactly under the cursor means swinging the
    // whole planet under it: from a whole-Earth view a point near the edge is
    // tens of degrees away, and a notch of the wheel would turn the globe by
    // several of them. The pull falls away with the angle, so zooming into
    // what you are looking at works and zooming into the edge of the world
    // does not heave it across the screen.
    const double cosSeparation = std::clamp(vsg::dot(vsg::normalize(focus_), surfaceNormal), -1.0, 1.0);
    if (cosSeparation <= 0.0) return;
    zoomTarget_ = *ground;
    zoomShare_ = cosSeparation * cosSeparation * cosSeparation;
}

void CameraController::followZoomTarget(double fromDistance, double toDistance) {
    if (zoomShare_ <= 0.0 || fromDistance <= 0.0) return;

    // Keeping a point T fixed on screen while the distance goes from d to d'
    // is one step: the focus F scales towards T by the ratio the distance
    // did, F' = T + (F - T) * d'/d. It is exact, it composes - doing it every
    // frame with that frame's ratio lands where doing it once with the whole
    // ratio would - and it is its own inverse, so scrolling out undoes what
    // scrolling in did instead of ratcheting the globe one way.
    //
    // The share damps it towards no movement for targets far off the middle
    // of the view; the clamp is only there so a huge single step cannot fling
    // the focus past the target and out the other side.
    const double t = std::clamp((1.0 - toDistance / fromDistance) * zoomShare_, -1.0, 1.0);
    if (t == 0.0) return;
    moveFocusTo(focus_ + (zoomTarget_ - focus_) * t, Carry::WithGround);
}

void CameraController::apply(vsg::ScrollWheelEvent& e) {
    if (e.handled) return;
    const double from = targetDistance_;
    targetDistance_ = std::clamp(targetDistance_ * std::pow(kZoomPerNotch, static_cast<double>(e.delta.y)),
                                 detached() ? kMinDistanceDetached : kMinDistance, kMaxDistance);
    zoomTowardsCursor(from, targetDistance_);
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
        const double before = distance_;
        distance_ += (targetDistance_ - distance_) * a;
        if (std::abs(distance_ - targetDistance_) < 1e-3 * targetDistance_) distance_ = targetDistance_;
        followZoomTarget(before, distance_);
        if (distance_ == targetDistance_) zoomShare_ = 0.0;
    } else {
        zoomShare_ = 0.0;
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

    // Terrain collision, on osgEarth's rule: keep the *eye* out of the ground
    // and nothing more. EarthManipulator::collisionDetect drops a vertical
    // line through the eye, and if the eye is under the terrain by more than
    // the avoidance distance it lifts it straight out. A hill between the eye
    // and what it is looking at is not its business - the hill simply hides
    // the view, as hills do.
    //
    // This used to sweep the whole line of sight and raise the elevation until
    // nothing crossed it. That is a stronger promise than osgEarth makes, and
    // it is why the camera could not be levelled: 60 km across the Bernese
    // Alps with the eye at 18 km - above every summit in Europe - the peaks
    // between focus and eye still forced 14 degrees of tilt. Whatever is in
    // the way, if the eye is in clear air the angle asked for is the angle
    // drawn.
    if (ground_ && distance_ < 2.0e5) {
        const double focusAltitude = ellipsoid_->convertECEFToLatLongAltitude(orbitCentre).z;
        const double clearance = std::min(kEyeClearanceMaxM, kEyeClearanceM + kEyeClearanceRatio * distance_);
        // Detached, the focus is on the ground: never look up at it from below.
        double minElevation = followVehicle ? -kPi / 2.0 : std::asin(std::clamp(clearance / distance_, -1.0, 1.0));

        // Twice: lifting the eye moves it over different ground, which may be
        // higher than the ground it was over before.
        for (int pass = 0; pass < 2; ++pass) {
            const double probeElevation = std::max(elevation, minElevation);
            const vsg::dvec3 probeDir = horizontal * std::cos(probeElevation) + up * std::sin(probeElevation);
            const vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(orbitCentre + probeDir * distance_);
            if (auto h = ground_(lla.x * kDeg, lla.y * kDeg)) {
                const double needed = (*h + clearance) - focusAltitude; // height the eye must gain over the focus
                minElevation = std::max(minElevation, std::asin(std::clamp(needed / distance_, -1.0, 1.0)));
            }
        }
        if (elevation < minElevation) elevation = std::min(minElevation, kMaxElevation);
    }
    // Nothing here tilts the view as a function of distance. There used to
    // be a floor that kept the eye near enough overhead to stop the focus
    // drifting towards the limb, and whatever such a rule does, zooming does:
    // the tilt was a function of the distance, so every notch of the wheel
    // turned the view by the difference. That is the complaint it was meant
    // to fix. The eye is kept out of the globe by the elevation >= 0 rule
    // below, which is all that was ever needed for safety; where the user
    // points the camera beyond that is the user's business, as in osgEarth.
    // Beyond the terrain-collision range the eye must stay above the focus'
    // horizontal plane, or a negative elevation would put it under the globe.
    if (distance_ >= 2.0e5) elevation = std::max(elevation, 0.0);

    // What was drawn is where the next drag starts from, so raising the view
    // for the terrain or the distance leaves no dead zone at the top of it.
    shownElevation_ = elevation;

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
