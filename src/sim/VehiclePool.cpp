#include "sim/VehiclePool.h"

#include "core/Log.h"
#include "platform/Threads.h"

#include <cassert>
#include <string>

namespace fsim::sim {

VehiclePool::VehiclePool(unsigned workers, bool pinWorkers)
    : workerCount_(workers <= 1 ? 0u : workers), pinWorkers_(pinWorkers) {
    threads_.reserve(workerCount_);
    for (unsigned i = 0; i < workerCount_; ++i) threads_.emplace_back([this, i] { workerLoop(i); });
    LOG_DEBUG("sim") << "vehicle pool: " << (workerCount_ ? workerCount_ : 1u) << " worker(s)";
}

VehiclePool::~VehiclePool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    for (auto& t : threads_) t.join();
}

std::size_t VehiclePool::add(std::unique_ptr<FlightModel> model) {
    assert(model);
    models_.push_back(std::move(model));
    states_.emplace_back();
    return models_.size() - 1;
}

void VehiclePool::rangeFor(unsigned worker, std::size_t& begin, std::size_t& end) const noexcept {
    // Contiguous, deterministic partition: worker w owns [w*n/W, (w+1)*n/W).
    const std::size_t n = models_.size();
    const std::size_t w = workerCount_;
    begin = (n * worker) / w;
    end = (n * (worker + 1)) / w;
}

void VehiclePool::runRange(std::size_t begin, std::size_t end, const Job& job) {
    for (std::size_t i = begin; i < end; ++i) {
        FlightModel& m = *models_[i];
        if (!job.refreshOnly) {
            const ControlInputs& in = job.inputs[i];
            for (int k = 0; k < job.frameSkip; ++k) m.step(in);
        }
        m.state(states_[i]);
    }
}

void VehiclePool::dispatch(const Job& job) {
    if (workerCount_ == 0) {
        runRange(0, models_.size(), job);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job_ = job;
        done_ = 0;
        ++generation_;
    }
    wake_.notify_all();

    std::unique_lock<std::mutex> lock(mutex_);
    finished_.wait(lock, [this] { return done_ == workerCount_; });
}

void VehiclePool::workerLoop(unsigned index) {
    platform::setCurrentThreadName("fsim-sim-" + std::to_string(index));
    if (pinWorkers_) platform::pinCurrentThreadToCore(1 + index);

    std::uint64_t seen = 0;
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [&] { return stopping_ || generation_ != seen; });
            if (stopping_) return;
            seen = generation_;
            job = job_;
        }

        std::size_t begin, end;
        rangeFor(index, begin, end);
        runRange(begin, end, job);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++done_;
        }
        finished_.notify_one();
    }
}

void VehiclePool::step(Span<const ControlInputs> inputs, int frameSkip) {
    assert(inputs.size() == models_.size());
    assert(frameSkip >= 1);
    ScopedTimer timer(stepTiming_);
    Job job;
    job.inputs = inputs;
    job.frameSkip = frameSkip;
    dispatch(job);
}

void VehiclePool::refreshStates() {
    Job job;
    job.refreshOnly = true;
    dispatch(job);
}

} // namespace fsim::sim
