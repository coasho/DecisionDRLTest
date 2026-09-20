#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

namespace fsim::world {

/// Mouse-driven follow camera (design 8.2 "Cameras") with the feel of
/// OpenSceneGraph's TrackballManipulator: the eye sits on a sphere around the
/// selected vehicle (azimuth, elevation, distance) and the mouse works in
/// window-normalised coordinates, so the response is the same at any window
/// size or DPI:
///
///   left drag      rotate: dragging across half the window turns ~72 deg
///                  (drag right -> the scene turns right, i.e. the eye moves left;
///                   drag down -> the top of the scene comes towards you, the eye rises)
///   release while moving   "throw": the rotation keeps going until the next click
///   middle drag    pan: the scene follows the mouse (the look-at point shifts)
///   right drag     zoom: drag down = closer, drag up = farther (OSG convention)
///   wheel          zoom 10 % per notch
///   r / GUI        reset the view offset
///
/// Modes only change what the azimuth is measured against:
///  - Chase:    relative to the vehicle's heading (view turns with the aircraft)
///  - Orbit:    relative to north (view stays put while the aircraft turns)
///  - Overview: straight down from `distance x 10`, north up
class CameraController : public vsg::Inherit<vsg::Visitor, CameraController> {
public:
    enum class Mode { Chase, Orbit, Overview };

    CameraController(vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::LookAt> lookAt,
                     vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

    void setMode(Mode mode);
    Mode mode() const noexcept { return mode_; }

    /// Initial/default distance behind and above; also the "reset" state.
    void setChaseOffset(double distanceM, double elevationDeg = 14.0, double azimuthDeg = 180.0);
    void zoom(double factor) noexcept;
    void resetView() noexcept;

    /// Per-frame update with the followed vehicle's latest (interpolated) state.
    void update(const sim::VehicleState& target, double dtSeconds);

    // VSG event handling (render thread). Events already consumed by ImGui are ignored.
    void apply(vsg::ButtonPressEvent& e) override;
    void apply(vsg::ButtonReleaseEvent& e) override;
    void apply(vsg::MoveEvent& e) override;
    void apply(vsg::ScrollWheelEvent& e) override;

    static const char* modeName(Mode m) noexcept;

    double distance() const noexcept { return distance_; }
    double azimuthDeg() const noexcept { return azimuth_ * 57.29577951308232; }
    double elevationDeg() const noexcept { return elevation_ * 57.29577951308232; }
    bool thrown() const noexcept { return thrown_; }

private:
    void normalised(int x, int y, double& nx, double& ny) const;
    void rotate(double dxNdc, double dyNdc);
    void stopThrow() noexcept { thrown_ = false; throwAzimuth_ = throwElevation_ = 0.0; }

    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid_;
    Mode mode_ = Mode::Chase;

    // View offset in the local frame of the target.
    double azimuth_ = 3.14159265358979323846; ///< 0 = looking from ahead, pi = from behind
    double elevation_ = 0.244;                ///< radians above the horizontal (14 deg)
    double distance_ = 40.0;
    double defaultAzimuth_ = 3.14159265358979323846, defaultElevation_ = 0.244, defaultDistance_ = 40.0;
    double panRight_ = 0.0, panUp_ = 0.0;     ///< look-at offset in the eye's screen plane, metres

    // Heading smoothing for Chase so the view does not twitch with the aircraft.
    double smoothedHeading_ = 0.0;
    bool haveHeading_ = false;

    // Mouse state (window pixels; converted to normalised coordinates per event)
    bool leftDown_ = false, middleDown_ = false, rightDown_ = false;
    int lastX_ = 0, lastY_ = 0;
    // Throw: rotation rate at release (rad/s), kept until the next button press.
    vsg::clock::time_point lastMoveTime_{};
    double lastMoveAzimuth_ = 0.0, lastMoveElevation_ = 0.0, lastMoveSeconds_ = 0.0;
    double throwAzimuth_ = 0.0, throwElevation_ = 0.0;
    bool thrown_ = false;
};

} // namespace fsim::world
