#pragma once

#include "sim/SnapshotBuffer.h"
#include "sim/VehicleState.h"

#include <deque>
#include <vector>

namespace fsim::world {

/// Turns the simulation's irregular, lower-rate snapshot stream into smooth
/// per-frame states (design 6.4 "Viewer time").
///
/// A render-side simulation clock advances with wall time x time factor and is
/// kept a small, fixed delay behind the newest snapshot; each frame the two
/// snapshots bracketing that clock (from a short ring of recent ones) are
/// interpolated: position lerp, attitude slerp. If the clock runs past the
/// newest snapshot (sim thread stalled or starved) the state is extrapolated
/// with the body velocity, up to a limit.
class Interpolator {
public:
    explicit Interpolator(std::size_t vehicles);

    /// Feed a freshly acquired batch (render thread).
    void push(const sim::SnapshotBatch& batch);

    /// Advance the render clock by `frameSeconds` of wall time and produce the
    /// interpolated states. Returns false until two batches have been seen
    /// (then `states()` holds the newest raw batch).
    bool update(double frameSeconds, double timeFactor, bool paused);

    const std::vector<sim::VehicleState>& states() const noexcept { return out_; }
    double renderSimTime() const noexcept { return renderTime_; }

    /// Delay behind the newest snapshot, in agent steps (default 1.5).
    void setDelaySteps(double steps) noexcept { delaySteps_ = steps; }

private:
    struct Sample {
        double simTime = 0.0;
        std::int64_t wallNs = 0;
        std::vector<sim::VehicleState> states;
    };
    static void blend(const sim::VehicleState& a, const sim::VehicleState& b, double t, sim::VehicleState& out);
    static void extrapolate(const sim::VehicleState& b, double dt, sim::VehicleState& out);

    std::size_t vehicles_;
    std::deque<Sample> ring_;          ///< oldest .. newest, at most kRing entries
    static constexpr std::size_t kRing = 6;
    std::vector<sim::VehicleState> out_;
    double stepSeconds_ = 1.0 / 60.0;  ///< measured from snapshot spacing
    double renderTime_ = 0.0;
    double delaySteps_ = 1.5;
    double frameSeconds_ = 0.0;        ///< filtered frame period (the display cadence, without CPU-side jitter)
    bool clockStarted_ = false;
};

} // namespace fsim::world
