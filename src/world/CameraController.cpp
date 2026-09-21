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
constexpr double kProbeSpacingM = 150.0;      // ground spacing between terrain-clearance samples
constexpr double kEyeClearanceM = 4.0;        // eye height above the sampled terrain at close range ...
constexpr double kEyeClearanceRatio = 0.03;   // ... plus 3 % of the distance (the drawn LOD gets coarser with range) ...
constexpr double kEyeClearanceMaxM = 80.0;    // ... up to this
constexpr double kMaxFocusOffset = 45.0 * kDeg; // how far from overhead the focus may sit before the eye climbs

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
    // Only meaningful when the focus is ours to move; following a vehicle the
    // camera belongs to the vehicle.
    if (!zoomToCursor_ || !detached() || fromDistance <= 0.0) return;
    const auto ground = groundUnderCursor();
    if (!ground) return;

    // Closing to a fraction k of the distance leaves what the cursor is over
    // in place if the focus comes the same fraction of the way to it: the
    // offset of that point from the focus then shrinks exactly as fast as the
    // distance does, so its angle at the eye is unchanged.
    const double t = std::clamp(1.0 - toDistance / fromDistance, -1.0, 1.0);

    // Straight at it, not round the curve. With the view held still, moving
    // the focus along the straight line to the target by exactly this
    // fraction is not an approximation: the whole offset from eye to target
    // scales by the same k the distance does, so its direction - and so its
    // place on screen - is unchanged. Going round the great circle instead
    // overshoots, and did so by 21 pixels over a six-fold zoom. The line
    // between two nearby points on the globe dips below it by centimetres,
    // and update() puts the focus back on the ground in any case.
    // Zooming: the camera must not rotate, or the point it is closing on
    // slides across the screen. With the view held still in world terms, the
    // whole offset from eye to target scales by the same fraction the
    // distance does, so its direction - and its place on screen - is exactly
    // unchanged. That is why this is a straight line to the target and not a
    // path over the globe: the line is what makes the scaling exact, and
    // between two nearby points it dips below the surface by centimetres.
    // osgEarth moves its centre and leaves the orientation alone, and so does
    // this: holding the view fixed in world terms instead swung the compass
    // bearing as the focus slid, which is the more disorienting of the two.
    moveFocusTo(focus_ + (*ground - focus_) * t, Carry::WithGround);
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

        // Sample the whole line of sight at a fixed spacing on the ground, not
        // at a few fractions of it. Fractions leave the gap that matters
        // unwatched: the first was at 15 % of the way out, so anything nearer
        // than that - which is most of what a low camera is about to hit -
        // was never looked at. A ridge 800 m from the focus of a 12 km view
        // went straight through the line of sight, 1463 m of it.
        const int probes = static_cast<int>(std::clamp(distance_ / kProbeSpacingM, 16.0, 96.0));
        // Twice: lifting the view swings the line of sight onto ground it was
        // not crossing before, which may be higher still.
        for (int pass = 0; pass < 2; ++pass) {
            const double probeElevation = std::max(elevation, minElevation);
            const vsg::dvec3 probeDir = horizontal * std::cos(probeElevation) + up * std::sin(probeElevation);
            for (int i = 1; i <= probes; ++i) {
                const double t = static_cast<double>(i) / probes;
                const vsg::dvec3 lla = ellipsoid_->convertECEFToLatLongAltitude(orbitCentre + probeDir * (distance_ * t));
                if (auto h = ground_(lla.x * kDeg, lla.y * kDeg)) {
                    const double needed = (*h + clearance) - focusAltitude; // height to gain over the focus by that point
                    minElevation = std::max(minElevation, std::asin(std::clamp(needed / (distance_ * t), -1.0, 1.0)));
                }
            }
        }
        if (elevation < minElevation) elevation = std::min(minElevation, kMaxElevation);
    }
    // Far out the focus drifts towards the limb unless the eye climbs, and
    // the view ends up grazing the globe edge-on instead of looking at it.
    // What is needed is that the eye stay roughly overhead, so that is what
    // is asked for: keep the focus within kMaxFocusOffset of the point the
    // eye is directly above, and raise the elevation by the least that does
    // it. Solving the triangle rather than fading towards vertical matters,
    // because whatever this does, zooming does - the tilt is a function of
    // distance, so every notch of the wheel turns the view by the difference.
    // The old smoothstep ran to 89 degrees and turned it 47 degrees over
    // eight notches from a whole-Earth view; this turns it about 8, and
    // leaves it alone entirely closer in than half an Earth radius, which is
    // every distance anyone flies at.
    const double focusRadius = vsg::length(pos);
    if (focusRadius > 1.0 && distance_ > 0.5 * focusRadius) {
        const double sinOffset = std::sin(kMaxFocusOffset);
        const double overhead = std::acos(std::clamp((focusRadius / distance_) * sinOffset, -1.0, 1.0));
        elevation = std::max(elevation, std::min(overhead - kMaxFocusOffset, kMaxElevation));
    }
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
