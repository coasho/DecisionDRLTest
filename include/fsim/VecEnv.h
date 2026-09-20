#pragma once

// fsim C++ SDK (design 4.4, 9.1): the primary interface for trainers written
// in C++. Thin pimpl over the platform's environment layer; buffers are owned
// by the environment and handed out as read-only spans.

#include "fsim/fsim_c.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fsim {

/// Non-owning read-only view (mirrors std::span<const T> for C++17).
template <typename T>
struct ConstSpan {
    const T* data = nullptr;
    std::size_t size = 0;
    const T& operator[](std::size_t i) const noexcept { return data[i]; }
    const T* begin() const noexcept { return data; }
    const T* end() const noexcept { return data + size; }
};

/// Same fields as fsim_options, with C++ defaults.
struct VecEnvOptions {
    unsigned numEnvs = 1;
    unsigned vehiclesPerEnv = 1;
    unsigned workers = 0;
    std::uint64_t seed = 0;
    std::string aircraft = "c172x";
    std::string jsbsimRoot;
    std::string task = "altitude_heading_hold";
    std::string observation = "state";
    std::string action = "surfaces";
    double dt = 1.0 / 120.0;
    int frameSkip = 4;
    unsigned maxEpisodeSteps = 2000;
    double latitudeDeg = 37.6188, longitudeDeg = -122.375, altitudeM = 1500.0, headingDeg = 0.0, airspeedMs = 60.0;
    double latitudeJitterDeg = 0.02, longitudeJitterDeg = 0.02, altitudeJitterM = 150.0, headingJitterDeg = 180.0, airspeedJitterMs = 5.0;
    double targetAltitudeDeltaM = 300.0, targetHeadingDeltaDeg = 60.0;
    std::string worldName = "vecenv"; ///< viewers attach by this name
    bool publish = true;
};

struct StepResult {
    ConstSpan<float> observations;      ///< M*K*O
    ConstSpan<float> rewards;           ///< M*K
    ConstSpan<std::uint8_t> terminated; ///< M*K
    ConstSpan<std::uint8_t> truncated;  ///< M*K
    ConstSpan<float> finalObservations; ///< M*K*O, valid for environments that auto-reset this step
    ConstSpan<std::uint32_t> episodeSteps; ///< M
};

class FSIM_API VecEnv {
public:
    explicit VecEnv(const VecEnvOptions& options); ///< throws std::runtime_error on load failure
    ~VecEnv();
    VecEnv(const VecEnv&) = delete;
    VecEnv& operator=(const VecEnv&) = delete;

    StepResult reset(std::uint64_t seed = 0);
    StepResult step(const float* actions, std::size_t count);
    StepResult step(const std::vector<float>& actions) { return step(actions.data(), actions.size()); }

    unsigned numEnvs() const noexcept;
    unsigned vehiclesPerEnv() const noexcept;
    std::size_t numVehicles() const noexcept;
    std::size_t observationSize() const noexcept;
    std::size_t actionSize() const noexcept;
    const std::vector<std::string>& observationNames() const noexcept;
    const std::vector<std::string>& actionNames() const noexcept;
    double agentStepSeconds() const noexcept;
    std::uint64_t vehicleSteps() const noexcept;

    /// The C handle for the same environment (for mixed-language use).
    fsim_vecenv* handle() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

FSIM_API const char* version() noexcept;

} // namespace fsim
