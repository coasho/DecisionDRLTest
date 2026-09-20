#pragma once

#include "ipc/WorldLayout.h"
#include "platform/SharedMemory.h"
#include "sim/SnapshotBuffer.h"

#include <cstdint>
#include <string>
#include <vector>

namespace fsim::ipc {

/// A vehicle as the mirror sees it (decoded VehicleRecord).
struct MirroredVehicle {
    std::uint32_t slot = 0;
    std::uint32_t id = 0;
    std::uint64_t generation = 0;
    bool alive = false;
    std::uint8_t controlLevel = 0;
    std::string name, type, model;
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

/// Reader side of the world segment (design 9.7): read-only, never blocks the
/// publisher, tolerates a publisher that stalls, restarts or dies. The viewer
/// polls it once per frame.
class WorldMirror {
public:
    WorldMirror() = default;
    ~WorldMirror();
    WorldMirror(const WorldMirror&) = delete;
    WorldMirror& operator=(const WorldMirror&) = delete;

    /// Attach to a published world by name. Returns false if it does not exist
    /// or has an incompatible layout.
    bool open(const std::string& name);
    void close();
    bool valid() const noexcept { return header_ != nullptr; }

    const std::string& name() const noexcept { return name_; }
    std::uint32_t capacity() const noexcept { return header_ ? header_->capacity : 0; }
    double dt() const noexcept { return header_ ? header_->dt : 0.0; }
    int frameSkip() const noexcept { return header_ ? header_->frameSkip : 1; }
    std::uint64_t publisherPid() const noexcept { return header_ ? header_->publisherPid : 0; }

    /// Publisher still alive (flag + process check)?
    bool publisherAlive() const noexcept;
    /// Seconds since the last publish (wall clock).
    double ageSeconds() const noexcept;
    std::uint64_t vehicleSteps() const noexcept { return header_ ? header_->vehicleSteps.load(std::memory_order_relaxed) : 0; }
    std::uint64_t worldSteps() const noexcept { return header_ ? header_->worldSteps.load(std::memory_order_relaxed) : 0; }

    /// Re-read the vehicle table if it changed. Returns true when it did;
    /// `vehicles()` then reflects the new table (one entry per slot).
    bool pollTable();
    const std::vector<MirroredVehicle>& vehicles() const noexcept { return vehicles_; }
    std::uint64_t tableGeneration() const noexcept { return tableGeneration_; }

    /// Copy the newest snapshot into `out` if it is newer than the last one
    /// returned. `out.states` has `capacity()` entries; `inputs()` matches.
    bool pollSnapshot(sim::SnapshotBatch& out);
    const std::vector<sim::ControlInputs>& inputs() const noexcept { return inputs_; }
    std::int64_t lastWallNs() const noexcept { return lastWallNs_; }

    /// Latest environment (seqlock copy).
    sim::EnvironmentState environment() const noexcept;

private:
    const VehicleRecord* row(std::uint32_t index) const noexcept;
    const StateSlotHeader* slot(std::uint32_t index) const noexcept;
    const VehicleSample* samples(std::uint32_t index) const noexcept;

    platform::SharedMemory memory_;
    const WorldHeader* header_ = nullptr;
    const unsigned char* base_ = nullptr;
    std::string name_;
    std::vector<MirroredVehicle> vehicles_;
    std::vector<sim::ControlInputs> inputs_;
    std::uint64_t tableGeneration_ = ~0ull;
    std::uint64_t lastSequence_ = 0;
    std::int64_t lastWallNs_ = 0;
    bool attached_ = false;
};

} // namespace fsim::ipc
