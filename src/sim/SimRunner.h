#pragma once

#include "sim/ControlInputs.h"
#include "sim/SnapshotBuffer.h"
#include "sim/VehiclePool.h"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace fsim::sim {

/// Runs a VehiclePool on its own thread, paced to wall-clock time, and
/// publishes snapshots for viewers (design 6.2, viewer configuration). The
/// training path never uses this class: there the caller drives step()
/// directly with no clock.
class SimRunner {
public:
    /// Called on the sim thread before every step to produce the controls.
    using Controller = std::function<void(const SnapshotBatch& previous, std::vector<ControlInputs>& out)>;

    SimRunner(std::unique_ptr<VehiclePool> pool, int frameSkip, Controller controller);
    ~SimRunner();

    SimRunner(const SimRunner&) = delete;
    SimRunner& operator=(const SimRunner&) = delete;

    void start();
    void stop();

    // Live controls (any thread).
    void setPaused(bool paused) noexcept { paused_.store(paused, std::memory_order_relaxed); }
    bool paused() const noexcept { return paused_.load(std::memory_order_relaxed); }
    void setTimeFactor(double factor) noexcept { timeFactor_.store(factor, std::memory_order_relaxed); }
    double timeFactor() const noexcept { return timeFactor_.load(std::memory_order_relaxed); }
    /// Advance one agent step while paused.
    void singleStep() noexcept { singleStep_.store(true, std::memory_order_relaxed); }

    SnapshotBuffer& snapshots() noexcept { return snapshots_; }
    const VehiclePool& pool() const noexcept { return *pool_; }
    std::size_t vehicles() const noexcept { return pool_->size(); }

    /// Achieved vehicle-steps per second over the last second (any thread).
    double throughput() const noexcept { return throughput_.load(std::memory_order_relaxed); }
    double agentStepSeconds() const noexcept { return agentStepSeconds_; }

private:
    void loop();

    std::unique_ptr<VehiclePool> pool_;
    int frameSkip_;
    double agentStepSeconds_;
    Controller controller_;
    SnapshotBuffer snapshots_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> singleStep_{false};
    std::atomic<double> timeFactor_{1.0};
    std::atomic<double> throughput_{0.0};
};

} // namespace fsim::sim
