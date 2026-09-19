#pragma once

#include <cmath>
#include <cstdint>

namespace fsim {

/// Deterministic per-vehicle random stream (design 6.4): a SplitMix64
/// generator seeded from (seed, envIndex, vehicleIndex). Independent of
/// thread scheduling because each vehicle owns its own instance.
class Rng {
public:
    explicit Rng(std::uint64_t seed = 0) noexcept : state_(seed) {}

    /// Derive a stream that is decorrelated from other (env, vehicle) pairs.
    static Rng forVehicle(std::uint64_t seed, std::uint64_t envIndex, std::uint64_t vehicleIndex) noexcept {
        Rng base(seed);
        std::uint64_t mixed = base.next() ^ (envIndex * 0x9E3779B97F4A7C15ull);
        mixed = mix(mixed) ^ (vehicleIndex * 0xD1B54A32D192ED03ull);
        return Rng(mix(mixed));
    }

    std::uint64_t next() noexcept {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
        return mix(z);
    }

    /// Uniform in [0, 1).
    double uniform() noexcept { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

    /// Uniform in [lo, hi).
    double uniform(double lo, double hi) noexcept { return lo + (hi - lo) * uniform(); }

    /// Standard normal via Box-Muller (two uniforms, one output; simple and
    /// deterministic, adequate for initial-condition sampling).
    double normal() noexcept;

    std::uint64_t state() const noexcept { return state_; }

private:
    static std::uint64_t mix(std::uint64_t z) noexcept {
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    std::uint64_t state_;
};

inline double Rng::normal() noexcept {
    double u1 = uniform();
    if (u1 < 1e-300) u1 = 1e-300; // avoid log(0)
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
}

} // namespace fsim
