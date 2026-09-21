#include "vision/Offscreen.h"

#include "core/Log.h"
#include "platform/Paths.h"

#include <vsgXchange/all.h>

#include <chrono>
#include <cstring>

namespace fsim::vision {

namespace {

constexpr VkFormat kColourFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

vsg::ref_ptr<vsg::Image> makeImage(VkFormat format, unsigned w, unsigned h, VkImageUsageFlags usage, VkImageTiling tiling) {
    auto image = vsg::Image::create();
    image->imageType = VK_IMAGE_TYPE_2D;
    image->format = format;
    image->extent = VkExtent3D{w, h, 1};
    image->mipLevels = 1;
    image->arrayLayers = 1;
    image->samples = VK_SAMPLE_COUNT_1_BIT;
    image->tiling = tiling;
    image->usage = usage;
    image->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image->sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    return image;
}

/// Colour attachment left in TRANSFER_SRC_OPTIMAL for the readback copy.
vsg::ref_ptr<vsg::RenderPass> makeRenderPass(vsg::Device* device) {
    auto colour = vsg::defaultColorAttachment(kColourFormat);
    colour.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    auto depth = vsg::defaultDepthAttachment(kDepthFormat);
    vsg::RenderPass::Attachments attachments{colour, depth};

    vsg::AttachmentReference colourRef{};
    colourRef.attachment = 0;
    colourRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    vsg::AttachmentReference depthRef{};
    depthRef.attachment = 1;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    vsg::SubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachments.push_back(colourRef);
    subpass.depthStencilAttachments.push_back(depthRef);

    vsg::SubpassDependency in{};
    in.srcSubpass = VK_SUBPASS_EXTERNAL;
    in.dstSubpass = 0;
    in.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    in.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    in.srcAccessMask = 0;
    in.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    in.dependencyFlags = 0;
    vsg::SubpassDependency out{};
    out.srcSubpass = 0;
    out.dstSubpass = VK_SUBPASS_EXTERNAL;
    out.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    out.dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    out.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    out.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    out.dependencyFlags = 0;
    return vsg::RenderPass::create(device, attachments, vsg::RenderPass::Subpasses{subpass}, vsg::RenderPass::Dependencies{in, out});
}

} // namespace

bool Offscreen::create(const Settings& settings, std::string* error) {
    vsg::Names instanceExtensions, layers;
    if (settings.debugLayer) {
        instanceExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        layers.push_back("VK_LAYER_KHRONOS_validation");
    }
    try {
        instance_ = vsg::Instance::create(instanceExtensions, layers, VK_API_VERSION_1_1);
        auto [physicalDevice, queueFamily] = instance_->getPhysicalDeviceAndQueueFamily(VK_QUEUE_GRAPHICS_BIT);
        if (!physicalDevice || queueFamily < 0) {
            if (error) *error = "no Vulkan device with a graphics queue";
            return false;
        }
        queueFamily_ = queueFamily;
        vsg::QueueSettings queueSettings{vsg::QueueSetting{queueFamily, {1.0}}};
        auto features = vsg::DeviceFeatures::create();
        features->get().samplerAnisotropy = VK_TRUE;
        device_ = vsg::Device::create(physicalDevice, queueSettings, layers, vsg::Names{}, features);
        LOG_INFO("vision") << "Vulkan device: " << physicalDevice->getProperties().deviceName << " (offscreen)";
    } catch (const std::exception& e) {
        if (error) *error = std::string("Vulkan initialisation failed: ") + e.what();
        return false;
    }
    options_ = vsg::Options::create();
    options_->add(vsgXchange::all::create());
    options_->fileCache = vsg::Path((platform::configDir() / "tilecache").string());
    options_->sharedObjects = vsg::SharedObjects::create();
    viewer_ = vsg::Viewer::create();
    scene_ = vsg::Group::create();
    return true;
}

void Offscreen::setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid) {
    scene_->children.clear();
    scene_->addChild(scene);
    ellipsoid_ = ellipsoid;
}

unsigned Offscreen::addCamera(unsigned width, unsigned height, double fovDeg, vsg::Mask viewMask) {
    Camera c;
    c.width = width;
    c.height = height;
    c.fovDeg = fovDeg;
    c.lookAt = vsg::LookAt::create(vsg::dvec3(0.0, -1.0e7, 0.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 0.0, 1.0));
    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    vsg::ref_ptr<vsg::ProjectionMatrix> projection;
    if (ellipsoid_) projection = vsg::EllipsoidPerspective::create(c.lookAt, ellipsoid_, fovDeg, aspect, 1.0e-6, 9000.0);
    else projection = vsg::Perspective::create(fovDeg, aspect, 1.0, 1.0e6);
    c.camera = vsg::Camera::create(projection, c.lookAt, vsg::ViewportState::create(VkExtent2D{width, height}));

    // Attachments: colour (copied out after the pass) and depth.
    c.colour = makeImage(kColourFormat, width, height, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_IMAGE_TILING_OPTIMAL);
    auto depth = makeImage(kDepthFormat, width, height, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_TILING_OPTIMAL);
    auto colourView = vsg::createImageView(device_, c.colour, VK_IMAGE_ASPECT_COLOR_BIT);
    auto depthView = vsg::createImageView(device_, depth, VK_IMAGE_ASPECT_DEPTH_BIT);
    auto renderPass = makeRenderPass(device_);
    auto framebuffer = vsg::Framebuffer::create(renderPass, vsg::ImageViews{colourView, depthView}, width, height, 1);

    c.renderGraph = vsg::RenderGraph::create();
    c.renderGraph->framebuffer = framebuffer;
    c.renderGraph->renderArea.offset = VkOffset2D{0, 0};
    c.renderGraph->renderArea.extent = VkExtent2D{width, height};
    c.renderGraph->setClearValues(VkClearColorValue{{0.55f, 0.70f, 0.90f, 1.0f}}, VkClearDepthStencilValue{0.0f, 0}); // reverse depth
    auto view = vsg::View::create(c.camera, scene_);
    view->mask = viewMask;
    c.renderGraph->addChild(view);

    // Host-readable copy: linear tiling, host-visible memory.
    c.capture = makeImage(kColourFormat, width, height, VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_TILING_LINEAR);
    c.capture->compile(device_);
    // Cached host memory: reading uncached (write-combined) memory back is ~100x slower.
    const VkMemoryPropertyFlags cached = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
    if (c.capture->allocateAndBindMemory(device_, cached) != VK_SUCCESS)
        c.capture->allocateAndBindMemory(device_, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    c.rgb.assign(static_cast<std::size_t>(width) * height * 3, 0);

    cameras_.push_back(std::move(c));
    return static_cast<unsigned>(cameras_.size() - 1);
}

bool Offscreen::compile(std::string* error) {
    if (compiled_ || cameras_.empty()) {
        if (error) *error = compiled_ ? "already compiled" : "no cameras";
        return false;
    }
    commandGraph_ = vsg::CommandGraph::create(device_, queueFamily_);
    for (auto& c : cameras_) {
        commandGraph_->addChild(c.renderGraph);

        // Readback: capture image -> TRANSFER_DST, copy, -> GENERAL for the host.
        auto commands = vsg::Commands::create();
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        auto toDst = vsg::ImageMemoryBarrier::create(0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                                     VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, c.capture, range);
        commands->addChild(vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, toDst));

        auto copy = vsg::CopyImage::create();
        copy->srcImage = c.colour;
        copy->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        copy->dstImage = c.capture;
        copy->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        VkImageCopy region{};
        region.srcSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.dstSubresource = region.srcSubresource;
        region.extent = VkExtent3D{c.width, c.height, 1};
        copy->regions.push_back(region);
        commands->addChild(copy);

        auto toHost = vsg::ImageMemoryBarrier::create(VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                                      VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, c.capture, range);
        commands->addChild(vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, toHost));
        commandGraph_->addChild(commands);
    }
    viewer_->assignRecordAndSubmitTaskAndPresentation({commandGraph_});
    const auto result = viewer_->compile();
    if (!result) {
        if (error) *error = "scene compile failed";
        return false;
    }
    compiled_ = true;
    return true;
}

void Offscreen::setView(unsigned camera, const vsg::dvec3& eye, const vsg::dvec3& centre, const vsg::dvec3& up) {
    if (camera >= cameras_.size()) return;
    auto& la = cameras_[camera].lookAt;
    la->eye = eye;
    la->center = centre;
    la->up = up;
}

void Offscreen::advance() {
    if (!compiled_) return;
    if (!viewer_->advanceToNextFrame()) return;
    viewer_->handleEvents();
    viewer_->update();
    viewer_->recordAndSubmit();
    viewer_->present();
    ++frames_;
}

void Offscreen::render() {
    if (!compiled_) return;
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    if (!viewer_->advanceToNextFrame()) return;
    const auto t1 = clock::now();
    viewer_->handleEvents();
    viewer_->update();
    const auto t2 = clock::now();
    viewer_->recordAndSubmit();
    viewer_->present();
    ++frames_;
    const auto t3 = clock::now();
    vkDeviceWaitIdle(*device_); // one submission per render(); the images are ready when it retires
    const auto t4 = clock::now();
    for (auto& c : cameras_) readback(c);
    const auto t5 = clock::now();
    using ms = std::chrono::duration<double, std::milli>;
    timing_.advance = ms(t1 - t0).count();
    timing_.update = ms(t2 - t1).count();
    timing_.record = ms(t3 - t2).count();
    timing_.wait = ms(t4 - t3).count();
    timing_.readback = ms(t5 - t4).count();
}

void Offscreen::readback(Camera& c) {
    auto* memory = c.capture->getDeviceMemory(device_->deviceID);
    if (!memory) return;
    VkImageSubresource sub{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout layout{};
    vkGetImageSubresourceLayout(*device_, c.capture->vk(device_->deviceID), &sub, &layout);
    void* data = nullptr;
    if (memory->map(layout.offset, layout.size, 0, &data) != VK_SUCCESS || !data) return;
    // Bulk-copy each row out of the mapped memory first (wide loads), then drop the alpha channel locally.
    const auto* src = static_cast<const std::uint8_t*>(data);
    const std::size_t rowBytes = static_cast<std::size_t>(c.width) * 4;
    if (c.staging.size() < rowBytes * c.height) c.staging.resize(rowBytes * c.height);
    for (unsigned y = 0; y < c.height; ++y) std::memcpy(c.staging.data() + y * rowBytes, src + y * layout.rowPitch, rowBytes);
    memory->unmap();
    std::uint8_t* dst = c.rgb.data();
    const std::uint8_t* rgba = c.staging.data();
    for (std::size_t i = 0, n = static_cast<std::size_t>(c.width) * c.height; i < n; ++i, rgba += 4, dst += 3) {
        dst[0] = rgba[0];
        dst[1] = rgba[1];
        dst[2] = rgba[2];
    }
}

} // namespace fsim::vision
