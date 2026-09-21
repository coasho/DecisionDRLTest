#pragma once

#include <atomic>

namespace fsim::sim {

/// Terrain height source for physics (design 7.3). Must be thread-safe: it is
/// called concurrently from every simulation worker, several times per step.
/// The v1 implementation samples the same elevation tile pyramid the viewer
/// renders (design 8.2); `FlatGround` is the M0 stand-in and the unit-test
/// fixture.
class GroundProvider {
public:
    virtual ~GroundProvider() = default;

    /// Terrain height above the WGS-84 ellipsoid, metres, at a geodetic position.
    virtual double heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const = 0;
};

/// Constant-elevation ground.
class FlatGround final : public GroundProvider {
public:
    explicit FlatGround(double elevationM = 0.0) noexcept : elevationM_(elevationM) {}

    double heightAboveEllipsoidM(double, double) const override { return elevationM_.load(std::memory_order_relaxed); }
    void setElevationM(double m) noexcept { elevationM_.store(m, std::memory_order_relaxed); }

private:
    std::atomic<double> elevationM_;
};

} // namespace fsim::sim

namespace fsim {
using sim::FlatGround;
using sim::GroundProvider;
}
