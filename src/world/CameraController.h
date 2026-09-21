#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <functional>
#include <optional>

namespace fsim::world {

/// Mouse-driven camera (design 8.2 "Cameras") with the feel of
/// OpenSceneGraph's manipulators. The eye sits on a sphere (azimuth,
/// elevation, distance) around a focus: the followed vehicle, or, in Free
/// mode and while no vehicle exists, a point on the globe. Mouse deltas are
/// window-normalised, so the response is the same at any window size or DPI.
///
///   left drag      rotate the view around the focus (~72 deg per half window)
///   middle drag    pan the focus in the screen plane
///   right drag     following a vehicle: zoom (drag down = closer)
///                  free / no vehicle:   rotate the globe (the ground follows the mouse)
///   wheel          zoom 12 % per notch, smoothed; from 6 m up to the whole Earth
///   r / GUI        reset the view offset
///
/// The eye never goes below the terrain when a ground query is installed.
///
/// Modes:
///  - Chase:    azimuth relative to the vehicle's heading (view turns with the aircraft)
///  - Orbit:    azimuth relative to north (view stays put while the aircraft turns)
///  - Overview: straight down from `distance x 10`, north up
///  - Free:     detached: orbit a point on the globe that the right button moves
class CameraController : public vsg::Inherit<vsg::Visitor, CameraController> {
public:
    enum class Mode { Chase, Orbit, Overview, Free };
    static constexpr int kModeCount = 4;

    /// Terrain height (metres above the ellipsoid) at a position, or nullopt
    /// when unknown yet. Must not block (render thread).
    using GroundQuery = std::function<std::optional<double>(double latitudeRad, double longitudeRad)>;

    CameraController(vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::LookAt> lookAt,
                     vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

    void setMode(Mode mode);
    Mode mode() const noexcept { return mode_; }
    void setGroundQuery(GroundQuery query) { ground_ = std::move(query); }

    /// Initial/default distance behind and above; also the "reset" state.
    void setChaseOffset(double distanceM, double elevationDeg = 14.0, double azimuthDeg = 180.0);
    /// Detach and look at a geodetic point from a given distance and angles (scripted views, screenshots).
    void setFreeView(double latitudeDeg, double longitudeDeg, double altitudeM, double distanceM, double azimuthDeg = 180.0, double elevationDeg = 14.0);
    void zoom(double factor) noexcept;
    void resetView() noexcept;

    /// Where the camera looks while no vehicle is followed (Free mode / no vehicle).
    void setFocus(const vsg::dvec3& ecef) noexcept { focus_ = ecef; }
    const vsg::dvec3& focus() const noexcept { return focus_; }

    /// Per-frame update. `target` is the followed vehicle's latest state, or
    /// null when there is none (the camera then orbits its focus point).
    void update(const sim::VehicleState* target, double dtSeconds);

    // VSG event handling (render thread). Events already consumed by ImGui are ignored.
    void apply(vsg::ButtonPressEvent& e) override;
    void apply(vsg::ButtonReleaseEvent& e) override;
    void apply(vsg::MoveEvent& e) override;
    void apply(vsg::ScrollWheelEvent& e) override;

    static const char* modeName(Mode m) noexcept;

    double distance() const noexcept { return distance_; }
    double azimuthDeg() const noexcept { return azimuth_ * 57.29577951308232; }
    double elevationDeg() const noexcept { return elevation_ * 57.29577951308232; }
    /// Free-style input is active (Free mode, or no vehicle to follow).
    bool detached() const noexcept { return mode_ == Mode::Free || !hasTarget_; }

private:
    void normalised(int x, int y, double& nx, double& ny) const;
    void rotate(double dxNdc, double dyNdc);
    void moveFocus(double dxNdc, double dyNdc);
    void localFrame(const vsg::dvec3& pos, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& up) const;

    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid_;
    GroundQuery ground_;
    Mode mode_ = Mode::Chase;
    bool hasTarget_ = false;

    // View offset in the local frame of the focus.
    double azimuth_ = 3.14159265358979323846; ///< 0 = looking from ahead, pi = from behind
    double elevation_ = 0.244;                ///< radians above the horizontal (14 deg)
    double distance_ = 40.0, targetDistance_ = 40.0;
    double defaultAzimuth_ = 3.14159265358979323846, defaultElevation_ = 0.244, defaultDistance_ = 40.0;
    double panRight_ = 0.0, panUp_ = 0.0;     ///< look-at offset in the eye's screen plane, metres
    vsg::dvec3 focus_{6378137.0, 0.0, 0.0};   ///< ECEF focus for the detached camera
    vsg::dvec3 lastTargetPos_{};              ///< where the followed vehicle was (Free mode starts there)

    // Heading smoothing for Chase so the view does not twitch with the aircraft.
    double smoothedHeading_ = 0.0;
    bool haveHeading_ = false;

    // Mouse state (window pixels; converted to normalised coordinates per event)
    bool leftDown_ = false, middleDown_ = false, rightDown_ = false;
    int lastX_ = 0, lastY_ = 0;
    // Last frame's view vectors, for screen-plane operations.
    vsg::dvec3 viewRight_{1.0, 0.0, 0.0}, viewForward_{0.0, 1.0, 0.0}, viewUp_{0.0, 0.0, 1.0};
};

} // namespace fsim::world
