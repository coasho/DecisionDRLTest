#pragma once

// Offscreen Vulkan rendering for vehicle cameras (design 8.4): a headless
// vsg::Viewer (device, no window) with one framebuffer, render graph and
// host-readable capture image per camera, all recorded and submitted once
// per render(). Shares the scene builders of the viewer (Earth, sky, vehicle
// visuals) so agents see what the viewer shows.

#include <vsg/all.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fsim::vision {

/// The process's Vulkan instance and device, shared by every Offscreen (VSG
/// allows one vsg::Device per process by default).
struct VulkanContext {
    vsg::ref_ptr<vsg::Instance> instance;
    vsg::ref_ptr<vsg::Device> device;
    int queueFamily = -1;
    vsg::ref_ptr<vsg::Options> options;
    /// The live context, or a new one; null (with `error`) when there is no device.
    static std::shared_ptr<VulkanContext> acquire(bool debugLayer, std::string* error);
};

class Offscreen {
public:
    struct Settings {
        bool debugLayer = false;
    };

    /// Opens the device; false (with `error`) when there is no Vulkan device.
    bool create(const Settings& settings, std::string* error);

    vsg::ref_ptr<vsg::Device> device() const { return device_; }
    vsg::ref_ptr<vsg::Options> options() const { return options_; }

    /// The scene every camera draws. Call once, before addCamera()/compile().
    void setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);

    /// A framebuffer + view + readback for one camera; returns its index.
    unsigned addCamera(unsigned width, unsigned height, double fovDeg, vsg::Mask viewMask = vsg::MASK_ALL);

    /// Build the command graph and compile; call after the cameras exist.
    /// Cameras added later require another compile() (not supported yet).
    bool compile(std::string* error);

    /// Aim a camera (ECEF).
    void setView(unsigned camera, const vsg::dvec3& eye, const vsg::dvec3& centre, const vsg::dvec3& up);

    /// Record, submit, wait, and copy every camera's pixels into rgb().
    void render();
    /// Run the frame loop without reading back (lets the pager stream tiles).
    void advance();

    const std::vector<std::uint8_t>& rgb(unsigned camera) const { return cameras_[camera].rgb; }
    unsigned width(unsigned camera) const { return cameras_[camera].width; }
    unsigned height(unsigned camera) const { return cameras_[camera].height; }
    std::size_t cameraCount() const { return cameras_.size(); }
    std::uint64_t frames() const { return frames_; }

    /// Where the last render() went (milliseconds).
    struct Timing {
        double advance = 0.0, update = 0.0, record = 0.0, wait = 0.0, readback = 0.0;
    };
    const Timing& timing() const { return timing_; }

private:
    struct Camera {
        unsigned width = 0, height = 0;
        double fovDeg = 60.0;
        vsg::ref_ptr<vsg::LookAt> lookAt;
        vsg::ref_ptr<vsg::Camera> camera;
        vsg::ref_ptr<vsg::Image> colour, capture;
        vsg::ref_ptr<vsg::RenderGraph> renderGraph;
        std::vector<std::uint8_t> rgb;
        std::vector<std::uint8_t> staging; ///< RGBA rows copied out of the mapped image
    };
    void readback(Camera& c);

    std::shared_ptr<VulkanContext> context_;
    vsg::ref_ptr<vsg::Device> device_;
    int queueFamily_ = -1;
    vsg::ref_ptr<vsg::Options> options_;
    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Group> scene_;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid_;
    vsg::ref_ptr<vsg::CommandGraph> commandGraph_;
    std::vector<Camera> cameras_;
    bool compiled_ = false;
    std::uint64_t frames_ = 0;
    Timing timing_;
};

} // namespace fsim::vision
