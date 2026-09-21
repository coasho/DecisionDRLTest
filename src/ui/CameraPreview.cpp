#include "ui/CameraPreview.h"

#include "core/Log.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace fsim::ui {

CameraPreview::CameraPreview(std::shared_ptr<ViewerControls> controls, std::function<bool(vsg::ref_ptr<vsg::Object>)> compile, std::uint32_t deviceID)
    : controls_(std::move(controls)), compile_(std::move(compile)), deviceID_(deviceID) {}

void CameraPreview::attach(const std::string& worldName) {
    if (world_ == worldName && mirror_.valid()) return;
    detach();
    world_ = worldName;
    lastOpenAttempt_ = -1e9;
}

void CameraPreview::detach() {
    mirror_.close();
    cameras_.clear();
    world_.clear();
    lastFrame_ = 0;
}

void CameraPreview::rebuild() {
    cameras_.clear();
    auto sampler = vsg::Sampler::create();
    sampler->magFilter = VK_FILTER_LINEAR;
    sampler->minFilter = VK_FILTER_LINEAR;
    for (const auto& info : mirror_.cameras()) {
        if (info.width == 0 || info.height == 0) continue;
        Camera c;
        c.info = info;
        c.pixels = vsg::ubvec4Array2D::create(info.width, info.height, vsg::ubvec4(0, 0, 0, 255), vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM});
        c.pixels->properties.dataVariance = vsg::DYNAMIC_DATA; // refreshed every published frame
        c.texture = vsgImGui::Texture::create(c.pixels, sampler);
        if (!compile_ || !compile_(c.texture)) {
            LOG_WARN("ui") << "camera preview: cannot compile a texture for " << info.label;
            continue;
        }
        cameras_.push_back(std::move(c));
    }
    lastFrame_ = 0; // fetch the pixels at the next update
    LOG_INFO("ui") << "camera preview: " << cameras_.size() << " camera(s) of world '" << world_ << "'";
}

void CameraPreview::update() {
    clock_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (world_.empty()) return;
    if (!mirror_.valid()) {
        if (clock_ - lastOpenAttempt_ < 1.0) return; // a trainer without cameras: keep looking, cheaply
        lastOpenAttempt_ = clock_;
        if (!mirror_.open(world_)) return;
        rebuild();
    }
    if (mirror_.pollTable()) rebuild();
    if (!mirror_.publisherAlive() && cameras_.empty()) return;
    const std::uint64_t frame = mirror_.frame();
    if (frame == lastFrame_) return;
    lastFrame_ = frame;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        auto& c = cameras_[i];
        if (!mirror_.read(static_cast<unsigned>(i), scratch_)) continue;
        const std::size_t n = static_cast<std::size_t>(c.info.width) * c.info.height;
        if (scratch_.size() < n * 3) continue;
        auto* dst = c.pixels->data();
        const std::uint8_t* src = scratch_.data();
        for (std::size_t p = 0; p < n; ++p, src += 3) dst[p] = vsg::ubvec4(src[0], src[1], src[2], 255);
        c.pixels->dirty();
    }
}

void CameraPreview::record(vsg::CommandBuffer&) const {
    if (cameras_.empty() || !controls_->showCameras.load(std::memory_order_relaxed)) return;
    // Bottom-right corner, away from the monitor and the vehicle list.
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(display.x - 20.0f, display.y - 20.0f), ImGuiCond_FirstUseEver, ImVec2(1.0f, 1.0f));
    bool open = true;
    if (ImGui::Begin("cameras", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextDisabled("what the training application's cameras see (%zu)", cameras_.size());
        const float cell = 160.0f;
        const int perRow = std::max(1, static_cast<int>(std::min<std::size_t>(cameras_.size(), 4)));
        int column = 0;
        for (const auto& c : cameras_) {
            if (column > 0) ImGui::SameLine();
            ImGui::BeginGroup();
            const float aspect = static_cast<float>(c.info.height) / static_cast<float>(std::max(1u, c.info.width));
            ImGui::Image(c.texture->id(deviceID_), ImVec2(cell, cell * aspect));
            ImGui::TextUnformatted(c.info.label.c_str());
            ImGui::EndGroup();
            if (++column >= perRow) column = 0;
        }
    }
    ImGui::End();
    if (!open) controls_->showCameras.store(false, std::memory_order_relaxed);
}

} // namespace fsim::ui
