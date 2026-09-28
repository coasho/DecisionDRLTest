#pragma once

#include <atomic>
#include <limits>
#include <optional>

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
    /// The height where the provider has data, empty where it has none (a
    /// terrain tile it cannot load, which heightAboveEllipsoidM reads as sea
    /// level): what the terrain query answers (docs/flight-autonomy.md, 4.19).
    /// Every point, unless overridden.
    virtual std::optional<double> knownHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
        return heightAboveEllipsoidM(latitudeRad, longitudeRad);
    }
    /// The spacing of its data, m: a commanded path is checked against the
    /// ground this far apart (4.19). Infinite for ground that is flat.
    virtual double resolutionM() const noexcept { return 30.0; }
};

/// Constant-elevation ground.
class FlatGround final : public GroundProvider {
public:
    explicit FlatGround(double elevationM = 0.0) noexcept : elevationM_(elevationM) {}

    double heightAboveEllipsoidM(double, double) const override { return elevationM_.load(std::memory_order_relaxed); }
    double resolutionM() const noexcept override { return std::numeric_limits<double>::infinity(); }
    void setElevationM(double m) noexcept { elevationM_.store(m, std::memory_order_relaxed); }

private:
    std::atomic<double> elevationM_;
};

} // namespace fsim::sim

namespace fsim {
using sim::FlatGround;
using sim::GroundProvider;
}
