#pragma once

// The Vehicle Interface's guidance modes (docs/vehicle-interface.md, ADR-28):
// behaviours that fly a mode's fixed-size setpoint - an HSA/CSA - the way the
// vehicle flies: a wing banks to turn and holds an airspeed, a rotorcraft flies
// its velocity over the ground. Plain classes, as the built-in loops are, so a
// trainer can subclass them or register its own under their ids.

#include "fsim/Control.h"
#include "fsim/Export.h"

namespace fsim::control {

class ControllerRegistry;

/// The wind as the aircraft's own air data see it: the ground velocity less
/// the air velocity (true airspeed along its angles of attack and sideslip,
/// turned to north-east-down), low-passed. Guidance never reads the
/// environment's wind, so sensor effects reach it as they would a real one.
struct FSIM_API WindEstimate {
    double northMs = 0.0, eastMs = 0.0; ///< where it blows to
    bool valid = false;
    double timeConstantS = 3.0;
    void update(const sim::VehicleState& s, double dt) noexcept;
    void reset() noexcept { northMs = eastMs = 0.0, valid = false; }
};

/// A speed in `reference` as a true airspeed here: a calibrated one by the
/// ratio of the two airspeeds the state has (the standard atmosphere's when
/// it is too slow to have one), a Mach number by the speed of sound here. NaN
/// for a ground speed, which no airspeed makes alone.
FSIM_API double trueAirspeedOf(double speed, SpeedReference reference, const sim::VehicleState& s) noexcept;
/// The aircraft's own speed now in `reference`.
FSIM_API double speedNow(SpeedReference reference, const sim::VehicleState& s) noexcept;
/// An altitude in `reference` as the height above sea level to fly now: above
/// the ground, over the terrain under the aircraft.
FSIM_API double altitudeMslOf(double altitudeM, AltitudeReference reference, const sim::VehicleState& s) noexcept;
/// The aircraft's own altitude now in `reference`.
FSIM_API double altitudeNow(AltitudeReference reference, const sim::VehicleState& s) noexcept;

/// "hsa": fsim.guidance.hsa, A-GRA's HSA/CSA (docs/vehicle-interface.md, 4.4).
/// Flies a complete HsaCommand (the host resolves what a command leaves out):
/// - a wing: the heading, or the heading that holds the course against the
///   wind estimate plus a slow trim on the course error; the airspeed its
///   reference asks, or that makes the ground speed along its track; the
///   altitude by a vertical speed at its position loop's gain and limits;
/// - a rotorcraft: a ground speed as its velocity over the ground along the
///   heading or course (the nose along the track), an airspeed along the nose
///   or, for a course, the air velocity whose track is the course; the same
///   altitude law.
/// Its output is a velocity command, flown by the vehicle's own loops.
class FSIM_API HsaBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "hsa"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    /// What it commands: the heading or course, the altitude (as the height
    /// above sea level to fly now), the speed and its reference.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    WindEstimate wind_;
    double courseTrim_ = 0.0;  ///< a wing's course error integral, rad of heading
    double speedTrim_ = 0.0;   ///< a wing's ground speed error integral, m/s of airspeed
    double lastTime_ = -1.0;   ///< the vehicle's time at the last update: a loop that missed a period starts again
    HsaCommand flown_{};       ///< the setpoint as last flown
    double altitudeMsl_ = kHold, headingFlown_ = kHold;
};

namespace route {
struct Plan;
struct Fix;
} // namespace route

/// "route": fsim.guidance.route, A-GRA's waypoint following
/// (docs/vehicle-interface.md, 4.5 and 4.8). Flies the waypoints of the
/// vehicle's path store (ControlContext::path) as legs - great circles or
/// rhumb lines, the first from where the aircraft is - with fly-by turns on
/// circles tangent to both legs, sized for the speeds, the wind and the bank,
/// and fly-over points passed abeam. One path follower for every family:
/// line-of-sight guidance on the path's course with its curvature fed
/// forward, from the vehicle's course bandwidth (Performance):
/// - a wing flies it as a turn rate, with a slow trim on the cross-track;
/// - a rotorcraft as its velocity over the ground, the nose along the track,
///   slowing for a turn only as much as its radius asks, and to stop at the
///   end when it is to loiter there.
/// The altitude runs straight from point to point, or climbs at a segment's
/// rate; the speed is each segment's, in its reference. It completes after
/// the last point unless the route repeats, and flies on: along the last
/// leg's course, or orbiting (a wing) or hovering over (a rotorcraft) the
/// last point. A new path store revision or new options fly it afresh.
class FSIM_API RouteBehavior final : public Behavior {
public:
    RouteBehavior();
    ~RouteBehavior() override;
    const char* id() const noexcept override { return "route"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool finished() const noexcept override { return finished_; }
    Reason failure() const noexcept override { return failure_; }
    /// The waypoint flown to and its id, the laps, how far along the segment
    /// and the route, the distance and time to go, the cross-track, and what
    /// it commands: the course, the altitude, the segment's speed.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    /// Plan the route from where the aircraft is, and fly it from its start.
    void restart(const ControlContext& ctx, const RouteCommand& command);
    /// Where the aircraft is on the route, passing the pieces it has finished.
    route::Fix locate(const sim::VehicleState& s, const Performance& performance);
    /// Point `k`'s segment begins: its length and the altitude it climbs from.
    void beginSegment(std::uint32_t k, const sim::VehicleState& s, double atM, double halfArcM, double leadM, bool firstLap, bool fromPoint);
    /// On to the next point, or the end.
    void advance(const sim::VehicleState& s, const Performance& performance);
    /// A rotorcraft that loiters at the end stops at the last point.
    bool stops() const noexcept;

    std::unique_ptr<route::Plan> plan_; ///< allocated with the behaviour: nothing in flight
    WindEstimate wind_;
    RouteCommand flown_{};             ///< the options it was planned with
    std::uint32_t revision_ = 0;       ///< the path store's it was planned from
    bool planned_ = false, hovers_ = false;
    // where it is
    std::uint32_t target_ = 0;         ///< the point flown to
    bool firstLap_ = true, onArc_ = false, midway_ = false, ended_ = false, finished_ = false;
    Reason failure_ = Reason::None;
    std::uint32_t laps_ = 0;
    double leadOut_ = 0.0;             ///< the lead of the turn the leg flown was entered from
    double finishedM_ = 0.0;           ///< the route's pieces behind it
    double inPieceM_ = 0.0;            ///< how far along the one it is on
    double lapStartM_ = 0.0, lapM_ = 0.0;
    // the segment flown: to point segment_, from the middle of the turn before it
    std::uint32_t segment_ = 0;
    double segmentStartM_ = 0.0, segmentM_ = 0.0, segmentStartS_ = 0.0, segmentFrom_ = 0.0;
    // trims and what it commands
    double courseTrim_ = 0.0;          ///< a wing's course error integral, rad/s of turn rate
    double speedTrim_ = 0.0, lastTime_ = -1.0;
    double crossTrack_ = kHold, course_ = kHold, heading_ = kHold, altitudeMsl_ = kHold, groundSpeed_ = 0.0;
};

/// Registers the modes' behaviours ("hsa", "route"); registerBuiltinControllers calls it.
void registerGuidanceModes(ControllerRegistry& registry);

} // namespace fsim::control
