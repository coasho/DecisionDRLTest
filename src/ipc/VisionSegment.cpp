#include "ipc/VisionSegment.h"

#include "core/Log.h"

#include <algorithm>
#include <cstring>
#include <new>

namespace fsim::ipc {

std::string visionSegmentName(const std::string& worldName) { return "fsim.vision." + worldName; }

// --- publisher ---------------------------------------------------------------------------

bool VisionPublisher::create(const std::string& worldName, std::size_t capacityBytes) {
    close();
    const std::size_t total = sizeof(VisionHeader) + capacityBytes;
    if (!shm_.create(visionSegmentName(worldName), total)) {
        LOG_WARN("ipc") << "cannot create the camera segment for world '" << worldName << "'";
        return false;
    }
    auto* h = header();
    if (shm_.created()) {
        new (h) VisionHeader();
    } else if (h->magic != kVisionMagic) {
        new (h) VisionHeader(); // a stale mapping from a crashed publisher
    }
    h->magic = kVisionMagic;
    h->version = kVisionVersion;
    h->totalBytes = total;
    h->pid = platform::currentProcessId();
    count_ = 0;
    LOG_INFO("ipc") << "publishing camera images of world '" << worldName << "' (" << (capacityBytes >> 20) << " MiB) for viewers";
    return true;
}

void VisionPublisher::close() {
    if (shm_.valid()) {
        auto* h = header();
        h->generation.fetch_add(1, std::memory_order_release); // odd: rewriting
        h->cameraCount = 0;
        h->generation.fetch_add(1, std::memory_order_release);
    }
    shm_.close();
    count_ = 0;
}

unsigned VisionPublisher::setCameras(const std::vector<VisionCameraDesc>& cameras) {
    if (!shm_.valid()) return 0;
    auto* h = header();
    h->generation.fetch_add(1, std::memory_order_release); // odd while the table changes
    std::uint64_t offset = sizeof(VisionHeader);
    unsigned kept = 0;
    for (const auto& c : cameras) {
        if (kept >= kMaxVisionCameras) break;
        const std::uint64_t bytes = static_cast<std::uint64_t>(c.width) * c.height * 3;
        if (offset + bytes > h->totalBytes) {
            LOG_WARN("ipc") << "camera segment full: " << (cameras.size() - kept) << " camera(s) not published";
            break;
        }
        auto& r = h->cameras[kept];
        r.vehicleId = c.vehicleId;
        r.width = c.width;
        r.height = c.height;
        r.channels = 3;
        r.offset = offset;
        r.sequence.store(0, std::memory_order_relaxed);
        std::memset(r.label, 0, sizeof r.label);
        std::strncpy(r.label, c.label.c_str(), kVisionLabelLength - 1);
        offset += bytes;
        ++kept;
    }
    h->cameraCount = kept;
    count_ = kept;
    h->generation.fetch_add(1, std::memory_order_release); // even: consistent
    return kept;
}

void VisionPublisher::write(unsigned camera, const std::uint8_t* rgb, std::size_t bytes) {
    if (!shm_.valid() || camera >= count_ || !rgb) return;
    auto* h = header();
    auto& r = h->cameras[camera];
    const std::size_t expected = static_cast<std::size_t>(r.width) * r.height * 3;
    if (bytes < expected) return;
    r.sequence.fetch_add(1, std::memory_order_acq_rel); // odd: writing
    std::memcpy(static_cast<std::uint8_t*>(shm_.data()) + r.offset, rgb, expected);
    r.sequence.fetch_add(1, std::memory_order_release);  // even: consistent
}

void VisionPublisher::endFrame() {
    if (shm_.valid()) header()->frame.fetch_add(1, std::memory_order_release);
}

// --- mirror ---------------------------------------------------------------------------------

bool VisionMirror::open(const std::string& worldName) {
    close();
    platform::SharedMemory probe;
    if (!probe.open(visionSegmentName(worldName), sizeof(VisionHeader))) return false;
    const auto* h = static_cast<const VisionHeader*>(probe.data());
    if (h->magic != kVisionMagic || h->version != kVisionVersion || h->totalBytes < sizeof(VisionHeader)) return false;
    const std::size_t total = static_cast<std::size_t>(h->totalBytes);
    probe.close();
    if (!shm_.open(visionSegmentName(worldName), total)) return false;
    generation_ = ~std::uint64_t(0);
    pollTable();
    return true;
}

void VisionMirror::close() {
    shm_.close();
    cameras_.clear();
    generation_ = 0;
}

bool VisionMirror::pollTable() {
    if (!shm_.valid()) return false;
    const auto* h = header();
    const std::uint64_t g = h->generation.load(std::memory_order_acquire);
    if (g == generation_ || (g & 1u)) return false; // unchanged, or being rewritten
    std::vector<Camera> table;
    const unsigned n = std::min(h->cameraCount, kMaxVisionCameras);
    for (unsigned i = 0; i < n; ++i) {
        const auto& r = h->cameras[i];
        Camera c;
        c.vehicleId = r.vehicleId;
        c.width = r.width;
        c.height = r.height;
        c.label.assign(r.label, ::strnlen(r.label, kVisionLabelLength));
        table.push_back(std::move(c));
    }
    if (h->generation.load(std::memory_order_acquire) != g) return false; // changed under us; next poll
    cameras_ = std::move(table);
    generation_ = g;
    return true;
}

std::uint64_t VisionMirror::frame() const noexcept { return shm_.valid() ? header()->frame.load(std::memory_order_acquire) : 0; }

bool VisionMirror::publisherAlive() const noexcept { return shm_.valid() && platform::processAlive(header()->pid); }

bool VisionMirror::read(unsigned camera, std::vector<std::uint8_t>& rgb) const {
    if (!shm_.valid() || camera >= cameras_.size()) return false;
    const auto* h = header();
    const auto& r = h->cameras[camera];
    const std::size_t bytes = static_cast<std::size_t>(r.width) * r.height * 3;
    if (r.offset + bytes > h->totalBytes) return false;
    const std::uint64_t before = r.sequence.load(std::memory_order_acquire);
    if (before & 1u) return false;
    rgb.resize(bytes);
    std::memcpy(rgb.data(), static_cast<const std::uint8_t*>(shm_.data()) + r.offset, bytes);
    std::atomic_thread_fence(std::memory_order_acquire);
    return r.sequence.load(std::memory_order_acquire) == before;
}

} // namespace fsim::ipc
