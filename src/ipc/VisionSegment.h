#pragma once

// Camera images of a world in shared memory (design 8.4 "sensor preview"):
// fsim_vision publishes what each camera rendered into "fsim.vision.<world>"
// and the viewer shows the frames next to the world it mirrors. Same
// principles as the world segment: the publisher never blocks, the reader
// polls, seqlocks per camera, nothing in the trainer waits for a viewer.
//
// Layout: VisionHeader | image bytes (RGB, tightly packed) at each camera's
// offset. The segment is created once per (world, capacity); a change of the
// camera set rewrites the table under a new generation.

#include "platform/SharedMemory.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim::ipc {

inline constexpr std::uint32_t kVisionMagic = 0x49565346u; // "FSVI"
inline constexpr std::uint32_t kVisionVersion = 1;
inline constexpr unsigned kMaxVisionCameras = 64;
inline constexpr unsigned kVisionLabelLength = 32;

struct VisionCameraRecord {
    std::uint32_t vehicleId = 0;
    std::uint32_t width = 0, height = 0;
    std::uint32_t channels = 3;
    std::uint64_t offset = 0;                ///< image bytes from the segment start
    std::atomic<std::uint64_t> sequence{0}; ///< odd while being written; even = consistent
    char label[kVisionLabelLength] = {};     ///< vehicle name (or "cam<i>")
};

struct VisionHeader {
    std::uint32_t magic = 0, version = 0;
    std::uint64_t totalBytes = 0;
    std::uint64_t pid = 0;
    std::atomic<std::uint64_t> generation{0}; ///< bumped when the camera table is rewritten (odd while rewriting)
    std::atomic<std::uint64_t> frame{0};      ///< bumped after every published frame
    std::uint32_t cameraCount = 0;
    VisionCameraRecord cameras[kMaxVisionCameras];
};

/// A camera as the publisher describes it.
struct VisionCameraDesc {
    std::uint32_t vehicleId = 0;
    unsigned width = 0, height = 0;
    std::string label;
};

class VisionPublisher {
public:
    /// Create "fsim.vision.<world>" with room for `capacityBytes` of images.
    bool create(const std::string& worldName, std::size_t capacityBytes = 32u << 20);
    void close();
    bool valid() const noexcept { return shm_.valid(); }

    /// Rewrite the camera table (drops cameras that do not fit); returns how many were kept.
    unsigned setCameras(const std::vector<VisionCameraDesc>& cameras);
    /// Write one camera's RGB pixels (width*height*3 bytes).
    void write(unsigned camera, const std::uint8_t* rgb, std::size_t bytes);
    /// Mark the end of a frame (the reader's "new images" signal).
    void endFrame();

private:
    VisionHeader* header() noexcept { return static_cast<VisionHeader*>(shm_.data()); }
    platform::SharedMemory shm_;
    unsigned count_ = 0;
};

/// Read side; polls without ever blocking the publisher.
class VisionMirror {
public:
    bool open(const std::string& worldName);
    void close();
    bool valid() const noexcept { return shm_.valid(); }

    struct Camera {
        std::uint32_t vehicleId = 0;
        unsigned width = 0, height = 0;
        std::string label;
    };
    /// Re-read the camera table if it changed; returns true when it did.
    bool pollTable();
    const std::vector<Camera>& cameras() const noexcept { return cameras_; }
    std::uint64_t frame() const noexcept;          ///< the publisher's frame counter
    bool publisherAlive() const noexcept;

    /// Copy one camera's pixels (consistent snapshot); false when it is being written right now.
    bool read(unsigned camera, std::vector<std::uint8_t>& rgb) const;

private:
    const VisionHeader* header() const noexcept { return static_cast<const VisionHeader*>(shm_.data()); }
    platform::SharedMemory shm_;
    std::vector<Camera> cameras_;
    std::uint64_t generation_ = 0;
};

std::string visionSegmentName(const std::string& worldName);

} // namespace fsim::ipc
