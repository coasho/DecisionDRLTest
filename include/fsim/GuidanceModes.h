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
/// the ground, over the terrain under the aircraft; barometric, on the isobar
/// the altimeter reads it on (docs/flight-autonomy.md, 4.20; null: the
/// standard atmosphere's, at 1013.25 hPa).
FSIM_API double altitudeMslOf(double altitudeM, AltitudeReference reference, const sim::VehicleState& s,
                              const Altimeter* altimeter = nullptr) noexcept;
/// The aircraft's own altitude now in `reference` (barometric: what its altimeter reads).
FSIM_API double altitudeNow(AltitudeReference reference, const sim::VehicleState& s, const Altimeter* altimeter = nullptr) noexcept;
/// A speed optimisation's true airspeed (docs/flight-autonomy.md, 4.17): the
/// performance tables' best-range or best-endurance speed at an altitude and
/// at the weight `fuelKg` on board makes - the tables' weight with the tanks
/// empty (the heaviest they fly is with them full) and the fuel; one weight
/// flown (a battery's) is that one. NaN without tables, for an optimisation
/// code that is none, or where they give none (above the altitudes flown).
FSIM_API double optimalTasMs(const TablesSection* tables, double optimization, double altitudeMslM, double fuelKg) noexcept;

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
/// A speed optimisation is flown as that true airspeed: the performance
/// tables' best at the altitude and weight now (ControlContext::tables).
/// Its output is a velocity command, flown by the vehicle's own loops.
class FSIM_API HsaBehavior final : public Behavior {
public:
    const char* id() const noexcept override { return "hsa"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    /// What it commands: the heading or course (from the north it was
    /// commanded from), the altitude (as the height above sea level to fly
    /// now), the speed and its reference.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    WindEstimate wind_;
    double courseTrim_ = 0.0;  ///< a wing's course error integral, rad of heading
    double speedTrim_ = 0.0;   ///< a wing's ground speed error integral, m/s of airspeed
    double lastTime_ = -1.0;   ///< the vehicle's time at the last update: a loop that missed a period starts again
    HsaCommand flown_{};       ///< the setpoint as last flown
    double altitudeMsl_ = kHold, headingFlown_ = kHold;
    double speedFlown_ = kHold; ///< the speed flown at the last update (a speed optimisation's, now)
    /// A magnetic direction's declination where the aircraft was, and when (every 10 s: docs/flight-autonomy.md, 4.22).
    double declination_ = 0.0, declinationAt_ = kHold;
};

namespace route {
struct Plan;
struct Pattern;
struct Curve;
struct Fix;
struct Turn;
struct Ahead;
struct Steer;
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
    // what it commands
    double lastTime_ = -1.0;
    double crossTrack_ = kHold, course_ = kHold, heading_ = kHold, altitudeMsl_ = kHold, groundSpeed_ = 0.0;
    double lastCross_ = kHold; ///< off the leg flown last: what it reports past its end
};

/// "pattern": fsim.guidance.pattern, A-GRA's loiter (docs/vehicle-interface.md,
/// 4.6). Flies a complete PatternCommand (the host fills in what it leaves
/// out): an orbit round its centre, a racetrack or a hold - two half circles
/// joined by legs, the inbound one ending at the fix, entered direct to the fix -
/// or a figure-eight, two circles meeting at the centre - or one of A-GRA's
/// shapes (docs/flight-autonomy.md, 4.23 to 4.25): two circles, an entry
/// point flown to, an exit point left from, a hold's ways in, a point in a
/// frame carried as the frame moves; or a rotorcraft's hover over its point,
/// its duration from its arrival. The route's path follower flies it
/// (RouteBehavior). Its laps are counted from where it was joined; with a
/// duration or a number of laps it completes when the first has passed, and
/// flies on - round the pattern, or, given an exit point, on round to it and
/// out along its course there. A merged UPDATE flies the pattern it makes
/// afresh, the duration still counted from the start.
class FSIM_API PatternBehavior final : public Behavior {
public:
    PatternBehavior();
    ~PatternBehavior() override;
    const char* id() const noexcept override { return "pattern"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool finished() const noexcept override { return finished_; }
    /// TargetLost: the vehicle its frame follows is gone (docs/flight-autonomy.md, 4.25).
    Reason failure() const noexcept override { return failure_; }
    /// The piece flown (of the lap's), the laps, how far round the lap (or,
    /// timed, through the duration) and the time to go, the cross-track, and
    /// what it commands: the course, the altitude, the speed.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    /// Plan the pattern `c` from where the aircraft is.
    void plan(const ControlContext& ctx, const PatternCommand& c);
    /// Fly lap piece `i` from here: an arc's sweep counted from where the aircraft is round it.
    void startPiece(std::uint32_t i, const sim::VehicleState& s);
    /// Round an arc from here: its sweep counted from where the aircraft is (a little before its start: negative).
    void startArc(const route::Turn& turn, const sim::VehicleState& s);
    /// Where the aircraft is on the pattern, passing the pieces it has finished.
    route::Fix locate(const sim::VehicleState& s);
    /// Its duration or its laps are flown.
    bool due(double now) const noexcept;
    /// Its point where its frame is now, as the step began (a vehicle's as the world read it then), and the frame's
    /// velocity; false if the frame's vehicle is gone.
    bool placeInFrame(const ControlContext& ctx, const sim::VehicleState& s);
    /// The path flown over a frame that moves (4.25): the aircraft's velocity and the wind's over the frame, the frame's
    /// origin's velocity taken out - a rotorcraft's given back to it over the ground.
    VelocityCommand followInFrame(const ControlContext& ctx, const Performance& perf, const route::Fix& fix, const route::Ahead& ahead,
                                  const route::Steer& steer);
    /// A hover over a point a moving frame carries: the frame's origin's velocity, and a closing on the point it could
    /// stop closing (a formation's), no faster than `transit`.
    VelocityCommand hoverInFrame(const ControlContext& ctx, const Performance& perf, double transit) const noexcept;

    std::unique_ptr<route::Pattern> pattern_; ///< allocated with the behaviour
    WindEstimate wind_;
    PatternCommand flown_{};         ///< the setpoint it was planned from
    PatternCommand resolved_{};      ///< the same, complete
    PatternShape shape_{};           ///< its shape, the path store's when it was planned (completed)
    std::uint32_t pathRevision_ = 0; ///< the path store's revision then: a new shape is planned afresh
    FrameSpec frame_{};              ///< its frame, where its point is one's
    double frameAltitude_ = kHold;   ///< the frame's point's height, where the shape gives it a z
    bool frameMoves_ = false;        ///< its frame is not fixed: flown over it
    double frameNorthMs_ = 0.0, frameEastMs_ = 0.0, frameDownMs_ = 0.0; ///< its origin's velocity, as it was placed
    Reason failure_ = Reason::None;
    bool planned_ = false, hovers_ = false, entering_ = false, finished_ = false;
    bool leaving_ = false;           ///< past its exit point, out along its course there
    std::uint32_t piece_ = 0, laps_ = 0, entryPiece_ = 0;
    double swept_ = 0.0, lastBearing_ = 0.0; ///< round the arc flown
    double startS_ = -1.0;           ///< when it began (a duration counts from it)
    double lapDoneM_ = 0.0, inPieceM_ = 0.0;
    double lastTime_ = -1.0;
    double crossTrack_ = kHold, course_ = kHold, heading_ = kHold, altitudeMsl_ = kHold, groundSpeed_ = 0.0, simTime_ = 0.0;
    double speedFlown_ = kHold; ///< the speed flown at the last update (a speed optimisation's, now)
};

/// "curve": fsim.guidance.curve, A-GRA's curve following (docs/vehicle-interface.md,
/// 4.7). Flies the Bezier segments of the vehicle's path store: the nearest
/// point by Newton steps from where it was, over tables of their length; the
/// altitude along them; its speed - a wing's airspeed, a rotorcraft's ground
/// speed, as they were, the ground speed they make kept within the range
/// given - or the ground speed that takes the rest of it in the rest of its
/// duration. The route's path follower
/// flies it, the curvature fed forward from the curve ahead. Segments appended
/// (the store's curve the same, more segments) are flown on to; a new curve
/// is flown afresh. At its end it completes, and flies on along its last
/// course, or loiters: circles its end, a rotorcraft as a wing does (A-GRA's
/// CIRCULAR_LOITER).
class FSIM_API CurveBehavior final : public Behavior {
public:
    CurveBehavior();
    ~CurveBehavior() override;
    const char* id() const noexcept override { return "curve"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool finished() const noexcept override { return finished_; }
    Reason failure() const noexcept override { return failure_; }
    /// The segment flown (of how many), how far along it and the curve, the
    /// distance and time to go, the cross-track, and what it commands: the
    /// course, the altitude, the ground speed.
    bool progress(ActivityProgress& out) const noexcept override;

private:
    /// Fly the store's curve afresh, from its start.
    void restart(const ControlContext& ctx, const CurveCommand& c);
    /// The speed it flies now (speed_, reference_): the rest in the rest of its
    /// duration over the ground, or its own - a rotorcraft's ground speed, a
    /// wing's airspeed with the ground speed it makes along `fix` in the wind -
    /// within its range.
    void pace(const Performance& performance, const route::Fix& fix) noexcept;
    /// Past its end: on along its last course, or loitering.
    void end(const Performance& performance);
    /// Its reference where its frame is now, as the step began (a vehicle's as the world read it then), its axes turned
    /// with the frame as its points' rotation says, and the frame's velocity (docs/flight-autonomy.md, 4.27); false if
    /// the frame's vehicle is gone.
    bool placeInFrame(const ControlContext& ctx, const sim::VehicleState& s);
    /// Its points turned afresh from the store's in three dimensions by its frame's attitude (4.27: ROTATION_3D).
    void orient(const ControlContext& ctx, const FramePose& pose);
    /// A height along it (m, down from its reference in its plane) above sea level now, as its altitude reference reads.
    double altitudeOf(const ControlContext& ctx, const sim::VehicleState& s, double down) const noexcept;

    std::unique_ptr<route::Curve> curve_; ///< allocated with the behaviour
    WindEstimate wind_;
    CurveCommand flown_{};
    std::uint32_t generation_ = 0, stored_ = 0; ///< the store's curve, and how many of its segments it flies
    bool planned_ = false, hovers_ = false, ended_ = false, finished_ = false;
    Reason failure_ = Reason::None;
    std::uint32_t segment_ = 0;
    double t_ = 0.0;                 ///< the nearest point's parameter on segment_
    double startS_ = -1.0;           ///< when it began (a duration counts from it)
    double ownSpeed_ = 0.0;          ///< when it began: a rotorcraft's ground speed (its cruise, if still), a wing's airspeed
    SpeedReference reference_ = SpeedReference::GroundSpeed; ///< speed_'s
    double lastTime_ = -1.0;
    double crossTrack_ = kHold, course_ = kHold, heading_ = kHold, altitudeMsl_ = kHold, groundSpeed_ = 0.0, simTime_ = 0.0, speed_ = kHold;
    // its reference and points as A-GRA's schema gives them (4.27)
    CurveShape shape_{};             ///< its reference in a frame, the store's when it began
    FrameSpec frame_{};
    bool frameMoves_ = false;        ///< its frame is not fixed: flown over it
    double frameNorthMs_ = 0.0, frameEastMs_ = 0.0, frameDownMs_ = 0.0;
    FrameRotation rotation_ = FrameRotation::Unrotated;
    AltitudeReference altitudeReference_ = AltitudeReference::Msl;
    CurveZ z_ = CurveZ::Down;
    double absoluteBase_ = 0.0;      ///< its reference's height when its absolute altitudes were made down from it
    double turnedRoll_ = kHold, turnedPitch_ = kHold, turnedYaw_ = kHold; ///< the attitude its points were turned by
    double endNorth_ = 0.0, endEast_ = 0.0; ///< its end in its plane, past it
};

/// Registers the modes' behaviours ("hsa", "route", "pattern", "curve"); registerBuiltinControllers calls it.
void registerGuidanceModes(ControllerRegistry& registry);

} // namespace fsim::control
