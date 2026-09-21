#pragma once

// "cameras" window: the images a training application's fsim_vision cameras
// rendered, read from the world's vision segment (design 8.4 "sensor
// preview"). One texture per camera, refreshed whenever the publisher
// finished a frame; the trainer never waits for us.

#include "ipc/VisionSegment.h"
#include "ui/ViewerControls.h"

#include <vsg/all.h>
#include <vsgImGui/Texture.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fsim::ui {

class CameraPreview : public vsg::Inherit<vsg::Command, CameraPreview> {
public:
    /// `compile` uploads a new texture (render::Viewer::compile); `deviceID` is the window's device.
    CameraPreview(std::shared_ptr<ViewerControls> controls, std::function<bool(vsg::ref_ptr<vsg::Object>)> compile, std::uint32_t deviceID);

    /// Follow a world: (re)open its segment when it appears, drop it when the world goes.
    void attach(const std::string& worldName);
    void detach();
    /// Poll the segment: new camera table -> new textures, new frame -> pixels. Call once per frame.
    void update();
    std::size_t cameraCount() const noexcept { return cameras_.size(); }
    bool attached() const noexcept { return mirror_.valid(); }

    void record(vsg::CommandBuffer&) const override;

private:
    struct Camera {
        ipc::VisionMirror::Camera info;
        vsg::ref_ptr<vsg::ubvec4Array2D> pixels;
        vsg::ref_ptr<vsgImGui::Texture> texture;
    };
    void rebuild();

    std::shared_ptr<ViewerControls> controls_;
    std::function<bool(vsg::ref_ptr<vsg::Object>)> compile_;
    std::uint32_t deviceID_ = 0;
    std::string world_;
    ipc::VisionMirror mirror_;
    std::vector<Camera> cameras_;
    std::vector<std::uint8_t> scratch_;
    std::uint64_t lastFrame_ = 0;
    double lastOpenAttempt_ = -1e9;
    double clock_ = 0.0;
};

} // namespace fsim::ui
