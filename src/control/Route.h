#pragma once

// A route's geometry (docs/vehicle-interface.md, 4.5 and 4.8): its legs on the
// sphere - great circles or rhumb lines - and its fly-by turns, arcs in the
// plane at their waypoint; where the aircraft is against them; and the plan
// the route's behaviour flies and the host checks a route against. The Earth
// is a sphere of the mean radius here, as for all local geometry (core/Geodesy.h).

#include "fsim/Control.h"
#include "fsim/VehicleState.h"

#include <cstdint>

namespace fsim::control::route {

/// A leg from point a to point b. A great circle is kept as a's unit vector
/// and its plane's normal, a rhumb line in the Mercator projection, where it
/// is straight.
struct Leg {
    double latA = 0.0, lonA = 0.0, latB = 0.0, lonB = 0.0;
    double lengthM = 0.0;
    double courseOutRad = 0.0; ///< leaving a
    double courseInRad = 0.0;  ///< arriving at b
    bool rhumb = false;
    double a[3] = {0.0, 0.0, 0.0}; ///< a great circle's: a on the unit sphere...
    double n[3] = {0.0, 0.0, 0.0}; ///< ...and its plane's normal, a x b normalised: the left of the way it runs
    double psiA = 0.0, dPsi = 0.0, dLon = 0.0; ///< a rhumb line's: a's stretched latitude, its extent in it and in longitude
};

/// Where the aircraft is against a piece of the path.
struct Fix {
    double alongM = 0.0;      ///< from the piece's start: a leg's a, an arc's entry
    double crossTrackM = 0.0; ///< + right of the path
    double courseRad = 0.0;   ///< the path's course there
    double curvature = 0.0;   ///< 1/m, + turning right
};

Leg makeLeg(double latA, double lonA, double latB, double lonB, bool rhumb) noexcept;
/// The aircraft at (lat, lon) against the leg, extended beyond its ends.
Fix onLeg(const Leg& leg, double lat, double lon) noexcept;

/// A fly-by turn at a waypoint: an arc tangent to the leg in and the leg out,
/// in the plane at the point.
struct Turn {
    double radiusM = 0.0;  ///< 0: no arc - the point is flown over, or the route runs straight on
    double leadM = 0.0;    ///< the tangent points' distance from the waypoint, back along the leg in and on along the leg out
    double angleRad = 0.0; ///< the course change, + right
    double centreNorthM = 0.0, centreEastM = 0.0; ///< from the waypoint
    double entryBearingRad = 0.0; ///< from the centre to the arc's start
    double inRad = 0.0;    ///< the leg in's course at the point
    bool shrunk = false;   ///< smaller than planned, to fit a short leg
};
/// The turn from course `in` to course `out` on a circle of `radiusM`; no arc
/// for a change under a degree or over 150 degrees (flown over).
Turn makeTurn(double inRad, double outRad, double radiusM) noexcept;
/// The aircraft at (lat, lon) against the turn at the waypoint (lat0, lon0);
/// alongM is the arc swept from its start, curvature the arc's.
Fix onArc(const Turn& turn, double lat0, double lon0, double lat, double lon) noexcept;

/// A speed in its reference as the speed a turn is planned at: a true
/// airspeed (a calibrated one or a Mach number through the standard
/// atmosphere at `altitudeMslM`), or a ground speed as it is.
double plannedSpeed(double speed, double reference, double altitudeMslM) noexcept;

/// What a route flies, planned from its complete waypoints: fixed arrays, so
/// the behaviour that holds it allocates nothing in flight.
struct Plan {
    static constexpr std::uint32_t kMax = static_cast<std::uint32_t>(PathStore::kWaypoints);
    std::uint32_t count = 0, start = 0;
    bool repeat = false, rhumb = false;
    EndBehavior end = EndBehavior::Continue;
    Waypoint points[kMax];
    /// legs[i]: to point i from the one before (legs[0]: from the last, when it repeats).
    Leg legs[kMax];
    /// turns[i]: at point i, from legs[i] to the leg after it.
    Turn turns[kMax];
    /// On the first lap, in place of legs[start] and turns[start]: from where
    /// the aircraft was when the route started, and the turn from there.
    Leg entry;
    Turn entryTurn;
    /// After the last point, a wing that loiters orbits it (its centre the point).
    Turn orbit;

    std::uint32_t last() const noexcept { return count - 1; }
    std::uint32_t next(std::uint32_t i) const noexcept { return i + 1 < count ? i + 1 : 0; }
    std::uint32_t prev(std::uint32_t i) const noexcept { return i > 0 ? i - 1 : count - 1; }
    /// A leg leaves point i: it is not the end of a route that does not repeat.
    bool leaves(std::uint32_t i) const noexcept { return i + 1 < count || repeat; }
    const Leg& leg(std::uint32_t i, bool firstLap) const noexcept { return firstLap && i == start ? entry : legs[i]; }
    const Turn& turn(std::uint32_t i, bool firstLap) const noexcept { return firstLap && i == start ? entryTurn : turns[i]; }
    /// The turn a lap flies at the start of leg i; null for the entry, or the first point of a route that does not repeat.
    const Turn* turnBefore(std::uint32_t i, bool firstLap) const noexcept {
        if ((firstLap && i == start) || (i == 0 && !repeat)) return nullptr;
        return &turn(prev(i), firstLap);
    }
    /// Leg i as a lap flies it: its length less the turns' leads at its ends.
    double pieceM(std::uint32_t i, bool firstLap) const noexcept;
    /// A lap's length: the first from the entry, the others the whole route.
    double lapM(bool firstLap) const noexcept;
};

/// What a route's waypoints leave out, filled in (docs/vehicle-interface.md,
/// 4.5): each field the previous point's, the first's the aircraft's own now
/// (a reference given alone, its value in that reference now; a rotorcraft's
/// speed its cruise over the ground); longitudes wrapped. InvalidWaypoint,
/// with the point's index in `bad`, for a point that cannot be flown: not
/// finite, out of range, a reference that is not one, a speed that is not
/// positive, a bank or climb rate out of range, a reference given alone after
/// the first point, the same place as the point before (within a metre).
Reason complete(Waypoint* out, const Waypoint* in, std::uint32_t count, bool repeat, const sim::VehicleState& state, const Performance& performance,
                bool hovers, std::int16_t& bad) noexcept;

/// The plan's geometry from its complete points (`p.points`, `p.count` and
/// the options set): the legs, the entry from (lat, lon), the fly-by turns
/// sized for their speeds plus the wind and their banks, shrunk to fit short
/// legs (Turn::shrunk). The entry's turn is flown over if the aircraft is too
/// near its point to make it.
void plan(Plan& p, double lat, double lon, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept;

} // namespace fsim::control::route
