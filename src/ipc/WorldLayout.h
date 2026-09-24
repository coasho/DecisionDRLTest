#pragma once

// Shared-memory layout of a published world (design 9.7, ADR-19). Plain
// data with atomics only where the seqlock/sequence protocol needs them; the
// same header is compiled into fsim.dll (publisher) and the viewer (mirror).
// The layout only grows within kLayoutVersion.

#include "sim/ControlInputs.h"
#include "sim/EnvironmentState.h"
#include "sim/VehicleState.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace fsim::ipc {

constexpr std::uint32_t kMagic = 0x4D495346u; // "FSIM"
constexpr std::uint32_t kLayoutVersion = 2; // 2: VehicleState grew (engines' and moving parts' state)
constexpr std::uint32_t kNameLength = 64;
constexpr std::uint32_t kTypeLength = 64;
constexpr std::uint32_t kPathLength = 256;
constexpr std::uint32_t kSlots = 3;

constexpr const char* kWorldPrefix = "fsim.world.";
constexpr const char* kRegistryName = "fsim.registry";

static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "shared-memory sequence counters must be lock-free");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "shared-memory seqlocks must be lock-free");

/// Seqlock protocol: the writer increments `seq` to odd, writes, increments to
/// even; a reader copies while `seq` is even and unchanged across the copy.
using SeqLock = std::atomic<std::uint32_t>;

/// Vehicle table row: identity and configuration; changes rarely.
struct VehicleRecord {
    SeqLock seq{0};
    std::uint32_t id = 0;                ///< world-unique id (never reused within a world)
    std::uint64_t generation = 0;        ///< bumped on create / reset / remove
    std::uint8_t alive = 0;
    std::uint8_t controlLevel = 0;       ///< control::Level of the active command
    std::uint8_t reserved[6] = {0, 0, 0, 0, 0, 0};
    char name[kNameLength] = {};
    char type[kTypeLength] = {};         ///< e.g. "jsbsim:c172x"
    char model[kPathLength] = {};        ///< optional visual override (glTF path)
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

/// Snapshot of every vehicle at one instant; triple buffered.
struct StateSlotHeader {
    SeqLock seq{0};
    std::uint32_t reserved = 0;
    std::uint64_t sequence = 0;          ///< publish counter (monotonic across slots)
    double simTime = 0.0;
    std::int64_t wallNs = 0;             ///< steady clock at publish, system-wide on Windows (QPC)
};

struct VehicleSample {
    sim::VehicleState state;
    sim::ControlInputs inputs;
    double derived[8] = {0, 0, 0, 0, 0, 0, 0, 0}; ///< cascade summary (attitude/velocity targets), level-dependent
};

struct alignas(64) WorldHeader {
    std::uint32_t magic = 0;
    std::uint32_t layoutVersion = 0;
    std::uint32_t headerSize = 0;
    std::uint32_t capacity = 0;          ///< vehicle slots
    std::uint64_t publisherPid = 0;
    char name[kNameLength] = {};
    double dt = 0.0;
    std::int32_t frameSkip = 1;
    std::uint32_t reserved0 = 0;
    std::uint64_t tableOffset = 0;       ///< VehicleRecord[capacity]
    std::uint64_t slotOffset = 0;        ///< StateSlotHeader + VehicleSample[capacity], x kSlots
    std::uint64_t slotStride = 0;
    std::uint64_t totalSize = 0;

    std::atomic<std::uint32_t> alive{0};             ///< 1 while the publisher exists
    std::atomic<std::uint32_t> readers{0};           ///< attached mirrors (informational)
    std::atomic<std::uint64_t> tableGeneration{0};   ///< bumped whenever any row changes
    std::atomic<std::uint32_t> latestSlot{0};        ///< index of the newest complete slot
    std::atomic<std::uint32_t> reserved1{0};
    std::atomic<std::uint64_t> publishSequence{0};
    std::atomic<std::int64_t> lastPublishWallNs{0};
    std::atomic<std::uint64_t> vehicleSteps{0};      ///< total FDM vehicle-steps so far
    std::atomic<std::uint64_t> worldSteps{0};        ///< world.step() calls so far

    SeqLock environmentSeq{0};
    std::uint32_t reserved2 = 0;
    sim::EnvironmentState environment;
};

/// Byte layout helpers shared by publisher and mirror.
struct Layout {
    static constexpr std::size_t align(std::size_t n) noexcept { return (n + 63) & ~static_cast<std::size_t>(63); }
    static std::size_t tableOffset() noexcept { return align(sizeof(WorldHeader)); }
    static std::size_t tableSize(std::uint32_t capacity) noexcept { return align(sizeof(VehicleRecord) * capacity); }
    static std::size_t slotStride(std::uint32_t capacity) noexcept {
        return align(sizeof(StateSlotHeader)) + align(sizeof(VehicleSample) * capacity);
    }
    static std::size_t slotOffset(std::uint32_t capacity) noexcept { return tableOffset() + tableSize(capacity); }
    static std::size_t totalSize(std::uint32_t capacity) noexcept { return slotOffset(capacity) + slotStride(capacity) * kSlots; }
};

/// Registry of live worlds for discovery (a tiny separate mapping).
constexpr std::uint32_t kRegistryEntries = 32;
struct RegistryEntry {
    SeqLock seq{0};
    std::uint32_t reserved = 0;
    std::uint64_t pid = 0;
    std::int64_t createdWallNs = 0;
    char name[kNameLength] = {};
};
struct alignas(64) RegistryHeader {
    std::uint32_t magic = 0;
    std::uint32_t layoutVersion = 0;
    std::atomic<std::uint32_t> lock{0};  ///< spin lock for registration (rare)
    std::uint32_t reserved = 0;
    RegistryEntry entries[kRegistryEntries];
};

} // namespace fsim::ipc
