#pragma once

#include "core/Profiler.h"
#include "core/Span.h"
#include "core/StableArray.h"
#include "sim/ControlInputs.h"
#include "sim/FlightModel.h"
#include "sim/VehicleState.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace fsim::sim {

/// Owns N flight models and steps them in lockstep across a fixed worker pool
/// (design 6.2, 7.5). Vehicles are partitioned across workers in a fixed
/// order and share nothing, so results are identical for any worker count.
class VehiclePool {
public:
    /// Most vehicles one pool can hold (slots of removed vehicles are reused):
    /// far beyond what fits in memory as flight models, and costing only
    /// address space - about 60 MB of it - until used.
    static constexpr std::size_t kMaxVehicles = std::size_t{1} << 17;

    /// @param workers number of worker threads; 0 or 1 = step on the calling thread
    /// @param pinWorkers pin worker i to physical core 1 + i (design 12.2)
    explicit VehiclePool(unsigned workers, bool pinWorkers = false);
    ~VehiclePool();

    VehiclePool(const VehiclePool&) = delete;
    VehiclePool& operator=(const VehiclePool&) = delete;

    /// Add a vehicle (already loaded or not). Returns its index. Not thread-safe
    /// with respect to step().
    std::size_t add(std::unique_ptr<FlightModel> model);

    /// Step every active vehicle `frameSkip` times with its inputs, in
    /// parallel, then refresh the state snapshots. Blocks until all vehicles
    /// are done. `inputs.size()` must equal size().
    void step(Span<const ControlInputs> inputs, int frameSkip = 1);

    /// Per-FDM-step hook run by the worker that owns the vehicle, before each
    /// `FlightModel::step`, with a private copy of the vehicle's inputs it may
    /// rewrite (control cascades, effects). Must not touch other vehicles'
    /// models, and may read only its own vehicle's entry of `states()`: each
    /// worker rewrites its vehicles' entries as it finishes them, so another
    /// vehicle's may be from this step or the last. A hook that needs other
    /// vehicles copies `states()` before step() (session::World::StepView).
    using PreStep = std::function<void(std::size_t vehicle, int subStep, FlightModel& model, ControlInputs& inputs)>;
    void setPreStep(PreStep hook) { preStep_ = std::move(hook); }

    /// Inactive vehicles are skipped by step() (removed vehicles keep their slot).
    void setActive(std::size_t i, bool active) noexcept { active_[i] = active ? 1 : 0; }
    bool active(std::size_t i) const noexcept { return active_[i] != 0; }

    /// Snapshots from the last step() (or from refreshStates()).
    Span<const VehicleState> states() const noexcept { return Span<const VehicleState>(states_.data(), states_.size()); }

    /// Recompute snapshots without stepping (after load/reset).
    void refreshStates();
    /// Recompute one vehicle's snapshot (after an individual reset).
    void refreshState(std::size_t i) { models_[i]->state(states_[i]); }

    std::size_t size() const noexcept { return models_.size(); }
    unsigned workers() const noexcept { return workerCount_; }
    FlightModel& vehicle(std::size_t i) noexcept { return *models_[i]; }
    const FlightModel& vehicle(std::size_t i) const noexcept { return *models_[i]; }

    /// Wall-clock statistics of step() calls (design 12.1).
    const TimingStats& stepTiming() const noexcept { return stepTiming_; }

private:
    struct Job {
        Span<const ControlInputs> inputs;
        int frameSkip = 1;
        bool refreshOnly = false;
    };

    void workerLoop(unsigned index);
    void runRange(std::size_t begin, std::size_t end, const Job& job);
    void dispatch(const Job& job);
    void rangeFor(unsigned worker, std::size_t& begin, std::size_t& end) const noexcept;

    std::vector<std::unique_ptr<FlightModel>> models_;
    // Never moves: the SDK hands out pointers into it (fsim_vehicle_state_ptr,
    // Vehicle::state(), the Python SDK's zero-copy views) and promises they
    // hold until the vehicle is removed. A vector would reallocate as
    // vehicles are added and leave every one of them dangling.
    core::StableArray<VehicleState> states_{kMaxVehicles};
    std::vector<unsigned char> active_;
    PreStep preStep_;

    unsigned workerCount_;
    bool pinWorkers_;
    std::vector<std::thread> threads_;

    // Generation-based dispatch: main bumps `generation_` and workers run the
    // current job once, then increment `done_`.
    std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable finished_;
    std::uint64_t generation_ = 0;
    unsigned done_ = 0;
    Job job_;
    bool stopping_ = false;

    TimingStats stepTiming_;
};

} // namespace fsim::sim
