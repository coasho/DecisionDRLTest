#include "sim/SimRunner.h"

#include "core/Log.h"
#include "platform/Clock.h"
#include "platform/Threads.h"

#include <algorithm>
#include <chrono>

namespace fsim::sim {

SimRunner::SimRunner(std::unique_ptr<VehiclePool> pool, int frameSkip, Controller controller)
    : pool_(std::move(pool)), frameSkip_(std::max(1, frameSkip)),
      agentStepSeconds_(pool_->size() ? pool_->vehicle(0).dt() * frameSkip_ : 1.0 / 30.0),
      controller_(std::move(controller)), snapshots_(pool_->size()) {
    // Publish the initial state so viewers have something before the first step.
    pool_->refreshStates();
    auto& batch = snapshots_.beginWrite();
    const auto states = pool_->states();
    std::copy(states.begin(), states.end(), batch.states.begin());
    batch.simTime = states.empty() ? 0.0 : states[0].simTime;
    batch.wallNs = platform::Clock::nanoseconds();
    snapshots_.publish();
}

SimRunner::~SimRunner() { stop(); }

void SimRunner::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread([this] { loop(); });
}

void SimRunner::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

void SimRunner::loop() {
    platform::setCurrentThreadName("fsim-sim-runner");
    platform::requestHighResolutionTimer();
    using clock = platform::Clock::clock;

    std::vector<ControlInputs> inputs(pool_->size());
    auto next = clock::now();
    auto windowStart = next;
    std::uint64_t windowSteps = 0;

    while (running_.load(std::memory_order_relaxed)) {
        const bool doStep = !paused_.load(std::memory_order_relaxed) || singleStep_.exchange(false);
        if (doStep) {
            const SnapshotBatch* previous = snapshots_.current();
            if (controller_ && previous) controller_(*previous, *pool_, inputs);

            pool_->step(Span<const ControlInputs>(inputs), frameSkip_);

            auto& batch = snapshots_.beginWrite();
            const auto states = pool_->states();
            std::copy(states.begin(), states.end(), batch.states.begin());
            batch.simTime = states[0].simTime;
            batch.wallNs = platform::Clock::nanoseconds();
            snapshots_.publish();

            windowSteps += pool_->size() * static_cast<std::uint64_t>(frameSkip_);
        }

        // Pace to wall time: one agent step per agentStepSeconds / timeFactor.
        const double factor = std::max(1e-3, timeFactor_.load(std::memory_order_relaxed));
        const auto period = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(agentStepSeconds_ / factor));
        next += period;
        const auto now = clock::now();
        if (next < now - period * 4) next = now; // fell far behind (breakpoint, heavy load): don't burst

        const double windowSeconds = platform::Clock::seconds(windowStart, now);
        if (windowSeconds >= 1.0) {
            throughput_.store(static_cast<double>(windowSteps) / windowSeconds, std::memory_order_relaxed);
            windowSteps = 0;
            windowStart = now;
        }
        // Windows sleeps are coarse (1-15 ms); sleep until ~2 ms before the
        // deadline, then spin, so snapshots land on an even 60 Hz grid.
        const auto spinFrom = next - std::chrono::milliseconds(2);
        if (clock::now() < spinFrom) std::this_thread::sleep_until(spinFrom);
        while (clock::now() < next) std::this_thread::yield();
    }
}

} // namespace fsim::sim
