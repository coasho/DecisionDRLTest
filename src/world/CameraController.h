#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

namespace fsim::world {

/// Mouse-driven follow camera (design 8.2 "Cameras"). The eye sits on a
/// sphere around the selected vehicle described by azimuth, elevation and
/// distance; the mouse edits those in every mode:
///
///   left drag        orbit (azimuth / elevation)
///   right drag       distance (drag up = closer)   wheel  distance
///   middle click     reset the view offset
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

private:
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid_;
    Mode mode_ = Mode::Chase;

    // View offset in the local frame of the target.
    double azimuth_ = 3.14159265358979323846; ///< 0 = looking from ahead, pi = from behind
    double elevation_ = 0.244;                ///< radians above the horizontal (14 deg)
    double distance_ = 40.0;
    double defaultAzimuth_ = 3.14159265358979323846, defaultElevation_ = 0.244, defaultDistance_ = 40.0;

    // Heading smoothing for Chase so the view does not twitch with the aircraft.
    double smoothedHeading_ = 0.0;
    bool haveHeading_ = false;

    // Mouse state
    bool leftDown_ = false, rightDown_ = false;
    int lastX_ = 0, lastY_ = 0;
};

} // namespace fsim::world
