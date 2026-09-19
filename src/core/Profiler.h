#pragma once

#include "platform/Clock.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace fsim {

/// Running statistics for one timed scope (design 12.1). Lock-free by
/// construction: one instance per thread or per single-writer scope.
struct TimingStats {
    std::uint64_t samples = 0;
    double totalSeconds = 0.0;
    double minSeconds = std::numeric_limits<double>::infinity();
    double maxSeconds = 0.0;

    void add(double seconds) noexcept {
        ++samples;
        totalSeconds += seconds;
        minSeconds = std::min(minSeconds, seconds);
        maxSeconds = std::max(maxSeconds, seconds);
    }
    double meanSeconds() const noexcept { return samples ? totalSeconds / static_cast<double>(samples) : 0.0; }
    void reset() noexcept { *this = TimingStats{}; }
};

/// RAII scope timer feeding a TimingStats.
class ScopedTimer {
public:
    explicit ScopedTimer(TimingStats& stats) noexcept : stats_(stats), start_(platform::Clock::now()) {}
    ~ScopedTimer() { stats_.add(platform::Clock::seconds(start_, platform::Clock::now())); }
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    TimingStats& stats_;
    platform::Clock::time_point start_;
};

} // namespace fsim
