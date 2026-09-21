// Viewer screenshots: copy the swapchain image last presented into a
// host-visible image and write it as PNG (docs, bug reports, and a way to
// check the viewer without a screen).

#include "render/Viewer.h"

#include "core/Log.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC // vsgXchange carries its own copy
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <stb_image_write.h>
#pragma GCC diagnostic pop

#include <cstdio>
#include <cstring>
#include <vector>

namespace fsim::render {

bool Viewer::screenshot(const std::string& path) {
    if (!viewer_ || !window_) return false;
    auto device = window_->getOrCreateDevice();
    const auto extent = window_->extent2D();
    const VkFormat format = window_->surfaceFormat().format;
    const bool bgr = format == VK_FORMAT_B8G8R8A8_UNORM || format == VK_FORMAT_B8G8R8A8_SRGB;
    if (!bgr && format != VK_FORMAT_R8G8B8A8_UNORM && format != VK_FORMAT_R8G8B8A8_SRGB) {
        LOG_WARN("render") << "screenshot: unsupported swapchain format " << format;
        return false;
    }

    // The image presented last frame (imageIndex(1) once a frame has been presented).
    viewer_->deviceWaitIdle();
    const std::size_t index = window_->imageIndex(1) < window_->numFrames() ? window_->imageIndex(1) : window_->imageIndex(0);
    if (index >= window_->numFrames()) return false;
    auto source = window_->imageView(index)->image;

    auto capture = vsg::Image::create();
    capture->imageType = VK_IMAGE_TYPE_2D;
    capture->format = format;
    capture->extent = VkExtent3D{extent.width, extent.height, 1};
    capture->mipLevels = 1;
    capture->arrayLayers = 1;
    capture->samples = VK_SAMPLE_COUNT_1_BIT;
    capture->tiling = VK_IMAGE_TILING_LINEAR;
    capture->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    capture->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    capture->sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    capture->compile(device);
    if (capture->allocateAndBindMemory(device, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT) != VK_SUCCESS)
        capture->allocateAndBindMemory(device, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    auto commands = vsg::Commands::create();
    commands->addChild(vsg::PipelineBarrier::create(
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
        vsg::ImageMemoryBarrier::create(VK_ACCESS_MEMORY_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, source, range),
        vsg::ImageMemoryBarrier::create(0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                        VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, capture, range)));
    auto copy = vsg::CopyImage::create();
    copy->srcImage = source;
    copy->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    copy->dstImage = capture;
    copy->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    VkImageCopy region{};
    region.srcSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.dstSubresource = region.srcSubresource;
    region.extent = VkExtent3D{extent.width, extent.height, 1};
    copy->regions.push_back(region);
    commands->addChild(copy);
    commands->addChild(vsg::PipelineBarrier::create(
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
        vsg::ImageMemoryBarrier::create(VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                        VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, capture, range),
        vsg::ImageMemoryBarrier::create(VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, source, range)));

    if (viewer_->recordAndSubmitTasks.empty()) return false;
    auto queue = viewer_->recordAndSubmitTasks.front()->queue; // the graphics queue the frames use
    auto pool = vsg::CommandPool::create(device, queue->queueFamilyIndex());
    auto fence = vsg::Fence::create(device);
    const VkResult submitted = vsg::submitCommandsToQueue(pool, fence, 1000000000, queue, [&](vsg::CommandBuffer& cb) { commands->record(cb); });
    if (submitted != VK_SUCCESS) {
        LOG_WARN("render") << "screenshot: copy failed (" << submitted << ")";
        return false;
    }

    // Read the linear image back, convert to tightly packed RGB.
    auto* memory = capture->getDeviceMemory(device->deviceID);
    VkImageSubresource sub{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout layout{};
    vkGetImageSubresourceLayout(*device, capture->vk(device->deviceID), &sub, &layout);
    void* data = nullptr;
    if (!memory || memory->map(layout.offset, layout.size, 0, &data) != VK_SUCCESS || !data) return false;
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(extent.width) * extent.height * 3);
    const auto* src = static_cast<const std::uint8_t*>(data);
    for (uint32_t y = 0; y < extent.height; ++y) {
        const std::uint8_t* row = src + y * layout.rowPitch;
        std::uint8_t* out = rgb.data() + static_cast<std::size_t>(y) * extent.width * 3;
        for (uint32_t x = 0; x < extent.width; ++x, row += 4, out += 3) {
            out[0] = bgr ? row[2] : row[0];
            out[1] = row[1];
            out[2] = bgr ? row[0] : row[2];
        }
    }
    memory->unmap();

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* bytes, int size) { std::fwrite(bytes, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(extent.width), static_cast<int>(extent.height), 3, rgb.data(), static_cast<int>(extent.width) * 3);
    std::fclose(f);
    if (ok) LOG_INFO("render") << "screenshot " << extent.width << "x" << extent.height << " -> " << path;
    return ok != 0;
}

} // namespace fsim::render
