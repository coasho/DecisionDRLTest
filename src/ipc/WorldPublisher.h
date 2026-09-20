#pragma once

#include "core/Span.h"
#include "ipc/WorldLayout.h"
#include "platform/SharedMemory.h"

#include <cstdint>
#include <string>

namespace fsim::ipc {

/// Writer side of the world segment (design 9.7): created by every World,
/// wait-free, rate limited by wall clock. Never blocks, allocates, or notices
/// readers. If the mapping cannot be created the publisher is inert and every
/// call is a no-op, so a training application never fails because of it.
class WorldPublisher {
public:
    struct Options {
        std::string name = "default";
        std::uint32_t capacity = 256;
        double dt = 1.0 / 120.0;
        int frameSkip = 1;
        double publishIntervalSeconds = 1.0 / 60.0; ///< minimum wall time between snapshot copies
    };

    explicit WorldPublisher(const Options& options);
    ~WorldPublisher();
    WorldPublisher(const WorldPublisher&) = delete;
    WorldPublisher& operator=(const WorldPublisher&) = delete;

    bool active() const noexcept { return header_ != nullptr; }
    const std::string& name() const noexcept { return options_.name; }
    std::uint32_t capacity() const noexcept { return options_.capacity; }

    /// Vehicle table: create/reset (`alive` = 1) or remove (`alive` = 0).
    void setVehicle(std::uint32_t index, const VehicleRecord& record) noexcept;
    void setControlLevel(std::uint32_t index, std::uint8_t level) noexcept;
    void clearVehicle(std::uint32_t index) noexcept;

    void setEnvironment(const sim::EnvironmentState& environment) noexcept;

    /// Copy the current samples into the next slot if the publish interval has
    /// elapsed (or `force`). `states`/`inputs` are indexed by vehicle slot and
    /// may be shorter than the capacity. Returns true when a copy was made.
    bool publish(double simTime, Span<const sim::VehicleState> states, Span<const sim::ControlInputs> inputs,
                 bool force = false) noexcept;

    /// Counters shown by the viewer's monitor.
    void addSteps(std::uint64_t vehicleSteps, std::uint64_t worldSteps = 1) noexcept;

private:
    VehicleRecord* row(std::uint32_t index) noexcept;
    StateSlotHeader* slot(std::uint32_t index) noexcept;
    VehicleSample* samples(std::uint32_t index) noexcept;

    Options options_;
    platform::SharedMemory memory_;
    WorldHeader* header_ = nullptr;
    unsigned char* base_ = nullptr;
    std::uint32_t writeSlot_ = 0;
    std::int64_t lastPublishNs_ = 0;
};

} // namespace fsim::ipc
