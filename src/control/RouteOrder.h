// A route's points in the order they are flown, and what each is on the ground (docs/flight-autonomy.md, 4.52 and 4.59): its
// start on the ground (GroundRoute.cpp) and its end there (LandingRoute.cpp) read it alike.
#pragma once

#include "fsim/Control.h"

#include <cstdint>

namespace fsim::control::order {

inline WaypointType typeOf(const Waypoint& w) noexcept {
    return isHold(w.waypointType) ? WaypointType::NavOnly : static_cast<WaypointType>(static_cast<int>(w.waypointType));
}

inline bool runwayType(WaypointType t) noexcept {
    return t == WaypointType::RunwayStart || t == WaypointType::RunwayThreshold || t == WaypointType::RunwayLimit;
}

/// The path point i is in (its index), or -1 for none.
inline int pathOf(Span<const RoutePath> paths, std::uint32_t i) noexcept {
    for (std::size_t k = 0; k < paths.size(); ++k)
        if (i >= paths[k].first && i < paths[k].first + paths[k].count) return static_cast<int>(k);
    return -1;
}

inline PathType pathTypeOf(Span<const RoutePath> paths, std::uint32_t i) noexcept {
    const int k = pathOf(paths, i);
    return k < 0 || isHold(paths[static_cast<std::size_t>(k)].type) ? PathType::Primary : static_cast<PathType>(static_cast<int>(paths[static_cast<std::size_t>(k)].type));
}

/// The point flown after point i (A-GRA's NextPathSegment, else the next in its path, else the next given), or -1: the end.
inline std::int64_t nextOf(Span<const Waypoint> points, Span<const RoutePath> paths, std::uint32_t i) noexcept {
    const Waypoint& w = points[i];
    if (!isHold(w.next)) return w.next < 0.0 || w.next >= static_cast<double>(points.size()) ? -1 : static_cast<std::int64_t>(w.next);
    if (!paths.empty()) {
        const int k = pathOf(paths, i);
        if (k < 0) return -1;
        const RoutePath& p = paths[static_cast<std::size_t>(k)];
        return i + 1 < p.first + p.count ? static_cast<std::int64_t>(i) + 1 : -1;
    }
    return i + 1 < points.size() ? static_cast<std::int64_t>(i) + 1 : -1;
}

inline bool taxiAt(Span<const Waypoint> points, Span<const RoutePath> paths, std::uint32_t i) noexcept {
    return typeOf(points[i]) == WaypointType::Taxi || pathTypeOf(paths, i) == PathType::Taxi;
}

/// Where a landing begins at point i (4.59): a runway's threshold or a touchdown point, or a point of a LANDING path.
inline bool landingAt(Span<const Waypoint> points, Span<const RoutePath> paths, std::uint32_t i) noexcept {
    const WaypointType t = typeOf(points[i]);
    return t == WaypointType::RunwayThreshold || t == WaypointType::Touchdown || pathTypeOf(paths, i) == PathType::Landing;
}

inline std::uint32_t startOf(const RouteCommand& route, std::size_t count) noexcept {
    const double s = isHold(route.start) ? 0.0 : route.start;
    return s >= 0.0 && s < static_cast<double>(count) ? static_cast<std::uint32_t>(s) : 0;
}

} // namespace fsim::control::order
