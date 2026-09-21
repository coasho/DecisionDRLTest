#pragma once

// fsim vision SDK (design 8.4 "Vision observations"): cameras mounted on
// vehicles, rendered offscreen with the same Earth, sky and vehicle models the
// viewer draws, read back as RGB images for learners that see. Lives in its
// own library (fsim_vision.dll) because it needs Vulkan; the core SDK stays
// headless.
//
//   fsim::World world(options);
//   fsim::vision::Sensors sensors(world);                    // opens a Vulkan device, no window
//   fsim::vision::CameraSpec cam; cam.width = 160; cam.height = 120; cam.fovDeg = 70;
//   const unsigned nose = sensors.addCamera(vehicle, cam);   // mounted on the vehicle
//   for (;;) {
//       world.step();
//       sensors.render();                                    // every camera, one submission
//       auto img = sensors.image(nose);                      // img.rgb: height x width x 3, top row first
//   }
//
// Rendering is explicit (render()) so a trainer pays for images only when it
// wants them. Terrain and imagery stream in from the shared tile cache; run
// tile_prefetch for the training region first so nothing downloads during
// episodes.

#include "fsim/Export.h"
#include "fsim/World.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace fsim::vision {

/// A camera rigidly mounted on a vehicle. Body frame: x forward, y right, z down.
struct CameraSpec {
    unsigned width = 128, height = 128;
    double fovDeg = 60.0;                    ///< vertical field of view
    double offsetBodyM[3] = {0.0, 0.0, 0.0}; ///< mount position relative to the body origin
    double yawDeg = 0.0, pitchDeg = 0.0, rollDeg = 0.0; ///< mount attitude relative to the body (yaw right, pitch up, roll right)
    bool hideOwnVehicle = true;              ///< do not draw the aircraft the camera is mounted on (up to 31 such vehicles)
};

struct Options {
    bool earth = true;                ///< the globe (false: sky and vehicles only; tests, synthetic tasks)
    bool imagery = true;              ///< satellite imagery (else a plain shaded globe)
    bool elevation = true;            ///< terrain relief
    std::string imageryUrl;           ///< XYZ template; empty = Esri World Imagery
    std::string elevationUrl;         ///< XYZ template; empty = AWS Terrarium
    unsigned maxLevel = 15;           ///< deepest tile level
    bool sky = true;                  ///< sky dome and sun light from the world's clock
    unsigned maxVehicles = 64;        ///< vehicles drawn (others are invisible to cameras)
    std::string assetDir;             ///< extra asset directory (models/<type>.glb); empty = the platform's
    bool debugLayer = false;          ///< Vulkan validation
};

/// One RGB image, tightly packed, top row first.
struct Image {
    const std::uint8_t* rgb = nullptr;
    unsigned width = 0, height = 0;
    std::size_t size() const noexcept { return static_cast<std::size_t>(width) * height * 3; }
};

class FSIM_VISION_API Sensors {
public:
    /// Opens a Vulkan device and builds the scene. Throws fsim::Error when no
    /// device is available.
    explicit Sensors(World& world, const Options& options = {});
    ~Sensors();
    Sensors(const Sensors&) = delete;
    Sensors& operator=(const Sensors&) = delete;

    /// Mount a camera; returns its index. Cameras of a removed vehicle keep
    /// rendering from its last known pose.
    unsigned addCamera(const Vehicle& vehicle, const CameraSpec& spec);
    std::size_t cameraCount() const noexcept;

    /// Draw every camera from the world's current state and read the images
    /// back. Blocks until they are ready (a few ms for a handful of small cameras).
    void render();
    /// The last rendered image of a camera (valid until the next render()).
    Image image(unsigned camera) const noexcept;

    /// Write the last image of a camera as PNG (debugging, datasets).
    bool savePng(unsigned camera, const std::string& path) const;

    double lastRenderMs() const noexcept;  ///< wall time of the last render()
    /// Where the last render() went (milliseconds): frame bookkeeping, tile
    /// merges, command recording + submit, GPU wait, host copy.
    struct Timing {
        double advance = 0.0, update = 0.0, record = 0.0, wait = 0.0, readback = 0.0;
    };
    Timing timing() const noexcept;
    std::uint64_t frames() const noexcept; ///< render() calls so far
    /// Advance tile streaming without rendering (call between episodes to let
    /// the pager settle after a jump to a new region); returns loaded tiles pending.
    void settle(unsigned frames = 30);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fsim::vision
