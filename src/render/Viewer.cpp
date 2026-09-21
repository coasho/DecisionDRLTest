#include "render/Viewer.h"

#include <chrono>
#include <thread>

#include "core/Log.h"
#include "platform/Clock.h"
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
    // Vsync (FIFO) paces the loop for free; a frame cap needs a non-blocking present.
    if (settings_.maxFps > 0.0) traits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;

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
        // the ellipsoid; near = far * ratio. VSG renders reverse depth into a
        // 32-bit float buffer, so the ratio can be tiny: at 1e-5 the near plane
        // sat 3-4 km in front of a 10 km-high eye and clipped the nearby
        // terrain into flat grey slabs; 1e-6 keeps it under 1 m at any height
        // that shows the ground while the aircraft (>= 6 m away) is never cut.
        projection = vsg::EllipsoidPerspective::create(lookAt_, ellipsoid, settings_.fieldOfViewDeg, aspect, 1.0e-6, 9000.0);
    } else {
        projection = vsg::Perspective::create(settings_.fieldOfViewDeg, aspect, 1.0, 1.0e6);
    }
    camera_ = vsg::Camera::create(projection, lookAt_, vsg::ViewportState::create(window_->extent2D()));

    auto commandGraph = vsg::CommandGraph::create(window_);
    auto renderGraph = vsg::RenderGraph::create(window_);
    commandGraph->addChild(renderGraph);

    auto view = vsg::View::create(camera_, root_);
    if (settings_.headlight) view->addChild(vsg::createHeadlight());
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

bool Viewer::compile(vsg::ref_ptr<vsg::Object> object) {
    if (!viewer_ || !viewer_->compileManager || !object) return false;
    auto result = viewer_->compileManager->compile(object);
    if (result) vsg::updateViewer(*viewer_, result);
    return static_cast<bool>(result);
}

void Viewer::addEventHandler(vsg::ref_ptr<vsg::Visitor> handler) {
    if (viewer_) viewer_->addEventHandler(handler);
}

bool Viewer::frame() {
    if (!viewer_) return false;
    using seconds = std::chrono::duration<double>;
    auto t0 = vsg::clock::now();
    timing_.app = frameEnd_.time_since_epoch().count() ? seconds(t0 - frameEnd_).count() : 0.0;

    if (!viewer_->advanceToNextFrame()) return false;
    auto t1 = vsg::clock::now();
    timing_.advance = seconds(t1 - t0).count();

    viewer_->handleEvents();
    auto t2 = vsg::clock::now();
    timing_.events = seconds(t2 - t1).count();
    viewer_->update();
    auto t3 = vsg::clock::now();
    timing_.update = seconds(t3 - t2).count();
    viewer_->recordAndSubmit();
    auto t4 = vsg::clock::now();
    timing_.record = seconds(t4 - t3).count();
    viewer_->present();
    auto t5 = vsg::clock::now();
    timing_.present = seconds(t5 - t4).count();

    // Optional frame cap (a bare sleep_for() would be rounded up to the 15.6 ms
    // scheduler period and turn "60 fps" into 40; see platform::sleepUntil).
    // Only meaningful with a non-blocking present mode: with FIFO the cap and
    // the vsync beat against each other.
    if (settings_.maxFps > 0.0) {
        const auto target = lastFrame_ + std::chrono::duration_cast<vsg::clock::duration>(seconds(1.0 / settings_.maxFps));
        platform::sleepUntil(target);
    }
    const auto t6 = vsg::clock::now();
    timing_.sleep = seconds(t6 - t5).count();
    frameEnd_ = t6;

    const auto now = t6;
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
