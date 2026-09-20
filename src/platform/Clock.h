#pragma once

#include <chrono>
#include <cstdint>

namespace fsim::platform {

/// Monotonic wall-clock time. Never used inside the simulation path
/// (design 6.4): only for benchmarks, the viewer and logging.
class Clock {
public:
    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;

    static time_point now() noexcept { return clock::now(); }

    /// Seconds elapsed between two time points.
    static double seconds(time_point from, time_point to) noexcept {
        return std::chrono::duration<double>(to - from).count();
    }

    /// Nanoseconds since an unspecified epoch; cheap and monotonic.
    static std::int64_t nanoseconds() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(now().time_since_epoch()).count();
    }
};

/// Sleep until `deadline` with sub-millisecond precision: a high-resolution
/// waitable timer on Windows (immune to the 15.6 ms scheduler period and to
/// background timer coalescing), then a short spin for the last ~100 us.
/// Use this instead of std::this_thread::sleep_until for pacing loops.
void sleepUntil(Clock::time_point deadline) noexcept;

/// Scoped stopwatch: `Stopwatch sw; ... sw.elapsedSeconds()`.
class Stopwatch {
public:
    Stopwatch() noexcept : start_(Clock::now()) {}
    void restart() noexcept { start_ = Clock::now(); }
    double elapsedSeconds() const noexcept { return Clock::seconds(start_, Clock::now()); }

private:
    Clock::time_point start_;
};

} // namespace fsim::platform
