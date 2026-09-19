#include "render/Viewer.h"

#include "core/Log.h"
#include "platform/Paths.h"
#include "platform/Threads.h"

#include <vsgXchange/all.h>

namespace fsim::render {

bool Viewer::create(const ViewerSettings& settings) {
    settings_ = settings;
    platform::enableHighDpiAwareness(); // client rect in physical pixels == swapchain extent

    options_ = vsg::Options::create();
    options_->add(vsgXchange::all::create()); // glTF/KTX/PNG/JPEG readers + http tile fetching
    options_->fileCache = vsg::Path((platform::configDir() / "tilecache").string());
    options_->sharedObjects = vsg::SharedObjects::create();

    auto traits = vsg::WindowTraits::create();
    traits->windowTitle = settings.title;
    traits->width = settings.width;
    traits->height = settings.height;
    traits->fullscreen = settings.fullscreen;
    traits->debugLayer = settings.debugLayer;
    traits->samples = settings.samples;
    // Depth precision over the 2 m .. 200 km range of a full-Earth scene (design 12.3).
    traits->depthFormat = VK_FORMAT_D32_SFLOAT;

    viewer_ = vsg::Viewer::create();
    window_ = vsg::Window::create(traits);
    if (!window_) {
        LOG_ERROR("render") << "could not create a Vulkan window (is a Vulkan 1.1 driver installed?)";
        return false;
    }
    viewer_->addWindow(window_);

    const auto& physical = window_->getOrCreatePhysicalDevice();
    LOG_INFO("render") << "Vulkan device: " << physical->getProperties().deviceName << ", " << settings.width << "x"
                       << settings.height << (settings.debugLayer ? ", validation on" : "");
    return true;
}

bool Viewer::setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
                      vsg::ref_ptr<vsg::Node> imguiOverlay) {
    if (!viewer_ || !window_) return false;

    root_ = vsg::Group::create();
    root_->addChild(scene);

    // Initial camera: a few hundred km above the scene's centre, looking down.
    lookAt_ = vsg::LookAt::create(vsg::dvec3(0.0, -3.0e7, 0.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 0.0, 1.0));
    const double aspect = static_cast<double>(window_->extent2D().width) / static_cast<double>(window_->extent2D().height);
    vsg::ref_ptr<vsg::ProjectionMatrix> projection;
    if (ellipsoid) {
        // Far = distance to the horizon (+ mountains) from the eye's height above
        // the ellipsoid; near = far * ratio. At 1.5 km altitude far is ~250 km, so
        // a 1e-5 ratio puts near at ~2.5 m, which the D32 depth buffer handles and
        // which keeps a chase camera 40 m behind the aircraft outside the near plane.
        projection = vsg::EllipsoidPerspective::create(lookAt_, ellipsoid, settings_.fieldOfViewDeg, aspect, 1.0e-5, 2000.0);
    } else {
        projection = vsg::Perspective::create(settings_.fieldOfViewDeg, aspect, 1.0, 1.0e6);
    }
    camera_ = vsg::Camera::create(projection, lookAt_, vsg::ViewportState::create(window_->extent2D()));

    auto commandGraph = vsg::CommandGraph::create(window_);
    auto renderGraph = vsg::RenderGraph::create(window_);
    commandGraph->addChild(renderGraph);

    auto view = vsg::View::create(camera_, root_);
    view->addChild(vsg::createHeadlight());
    renderGraph->addChild(view);
    if (imguiOverlay) renderGraph->addChild(imguiOverlay); // same render pass, drawn last

    viewer_->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer_->addEventHandler(vsg::CloseHandler::create(viewer_));

    const auto result = viewer_->compile();
    if (!result) {
        LOG_ERROR("render") << "vsg::Viewer::compile failed";
        return false;
    }
    lastFrame_ = vsg::clock::now();
    return true;
}

void Viewer::addEventHandler(vsg::ref_ptr<vsg::Visitor> handler) {
    if (viewer_) viewer_->addEventHandler(handler);
}

bool Viewer::frame() {
    if (!viewer_ || !viewer_->advanceToNextFrame()) return false;

    viewer_->handleEvents();
    viewer_->update();
    viewer_->recordAndSubmit();
    viewer_->present();

    const auto now = vsg::clock::now();
    frameSeconds_ = std::chrono::duration<double>(now - lastFrame_).count();
    lastFrame_ = now;
    fpsAccumulator_ += frameSeconds_;
    if (++fpsFrames_ >= 30) {
        fps_ = fpsAccumulator_ > 0.0 ? fpsFrames_ / fpsAccumulator_ : 0.0;
        fpsAccumulator_ = 0.0;
        fpsFrames_ = 0;
    }
    return true;
}

} // namespace fsim::render
