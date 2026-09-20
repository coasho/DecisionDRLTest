#pragma once

#include <vsg/all.h>

#include <cstdint>
#include <string>

namespace fsim::render {

struct ViewerSettings {
    std::string title = "flightsim";
    std::uint32_t width = 1600;
    std::uint32_t height = 900;
    bool fullscreen = false;
    bool debugLayer = false;   ///< VK_LAYER_KHRONOS_validation
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_4_BIT;
    double fieldOfViewDeg = 30.0;
    bool headlight = true;     ///< off when the scene brings its own lights (sun)
    double maxFps = 60.0;      ///< frame-rate cap (0 = uncapped); keeps a mirror viewer from competing with the trainer for CPU
};

/// One window, one vsg::Viewer, one camera, one command graph: scene first,
/// ImGui overlay last in the same render pass (design 8, ADR-4). Owns nothing
/// about what is drawn; `world` supplies the scene, `ui` the overlay.
class Viewer {
public:
    Viewer() = default;

    /// Create the window and Vulkan device. Returns false (and logs) on failure.
    bool create(const ViewerSettings& settings);

    /// Build the command graph for `scene` (+ optional ImGui overlay node), add
    /// event handlers and compile everything. Call once after create().
    bool setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
                  vsg::ref_ptr<vsg::Node> imguiOverlay = {});

    /// Add an event handler (key handler, trackball, ImGui event forwarder).
    /// ImGui's forwarder must be added before any other handler so it can
    /// consume events first (design 9.6).
    void addEventHandler(vsg::ref_ptr<vsg::Visitor> handler);

    /// Run one frame: events, update, record, submit, present. Returns false
    /// once the window has been closed.
    bool frame();

    bool active() const { return viewer_ && viewer_->active(); }
    void close() { if (viewer_) viewer_->close(); }

    vsg::ref_ptr<vsg::Viewer> viewer() const { return viewer_; }
    vsg::ref_ptr<vsg::Window> window() const { return window_; }
    vsg::ref_ptr<vsg::Camera> camera() const { return camera_; }
    vsg::ref_ptr<vsg::LookAt> lookAt() const { return lookAt_; }
    vsg::ref_ptr<vsg::Options> options() const { return options_; }

    /// Last frame's wall-clock duration and rolling frames-per-second.
    double frameSeconds() const { return frameSeconds_; }
    double fps() const { return fps_; }

private:
    ViewerSettings settings_;
    vsg::ref_ptr<vsg::Options> options_;
    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Window> window_;
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::Group> root_;

    double frameSeconds_ = 0.0;
    double fps_ = 0.0;
    double fpsAccumulator_ = 0.0;
    int fpsFrames_ = 0;
    vsg::clock::time_point lastFrame_{};
};

} // namespace fsim::render
