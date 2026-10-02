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
class TaxiBehavior;
class LaunchBehavior;

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
/// The top level speed the performance tables give at an altitude, at the
/// weight `fuelKg` on board makes, as optimalTasMs weighs it: the fastest
/// full power holds level there - a rotorcraft's along its nose, within the
/// tilt its velocity loop flies (docs/flight-autonomy.md, 4.13). What bounds
/// a rotorcraft's airspeed commands (4.48). NaN without tables, or where they
/// give none (above the altitudes flown).
FSIM_API double topTasMs(const TablesSection* tables, double altitudeMslM, double fuelKg) noexcept;

/// "hsa": fsim.guidance.hsa, A-GRA's HSA/CSA (docs/vehicle-interface.md, 4.4).
/// Flies a complete HsaCommand (the host resolves what a command leaves out):
/// - a wing: the heading plus a slow trim on what its loops leave (the turn
///   its heading law asks less the turn made, while the bank holds), or the
///   heading that holds the course against the wind estimate plus a slow trim
///   on the course error; the airspeed its reference asks, or that makes the
///   ground speed along its track; the altitude by a vertical speed at its
///   position loop's gain and limits;
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
    double headingTrim_ = 0.0; ///< a wing's heading: the integral of the turn its law asks less the turn made, rad
    double lastHeading_ = kHold; ///< the heading at the last update, for the turn made since
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
struct Leg;
struct Turn;
struct Ahead;
struct Steer;
} // namespace route

class PatternBehavior;

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
///   slowing for a turn only as much as its radius asks, and for a stop - its
///   end where it loiters, a loiter point's hover - no faster than stops it
///   there along what is left of the route; it leaves the stop to its
///   position loop from where it would stop from its speed, as a loiter
///   point's hover, the nose held on the course the leg arrives on.
/// The altitude runs straight from point to point, or climbs at a segment's
/// rate; the speed is each segment's, in its reference - or the tables' best
/// at the altitude and weight now where it optimises it, reached at its
/// acceleration where it has one; a climb optimisation climbs or descends at
/// the most it can holding that speed (docs/flight-autonomy.md, 4.32); and
/// where a point has an arrival window, the ground speed that arrives in it
/// (4.33). It completes after
/// the last point unless the route repeats, and flies on: along the last
/// leg's course, or orbiting (a wing) or hovering over (a rotorcraft) the
/// last point. A new path store revision or new options fly it afresh.
/// A loiter point's loiter (docs/flight-autonomy.md, 4.31) is flown where the
/// leg meets it by a pattern's behaviour, from where the aircraft is; at its
/// end the route goes on to the next point from where it left it - or, its
/// last point's, completes.
class RecoveryBehavior;

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
    /// kActivityNavigationPerformance where, in its last update, it was farther off its path than the required navigation
    /// performance of the segment it flew (docs/flight-autonomy.md, 4.35).
    std::uint16_t constraints() const noexcept override { return constraints_; }
    /// The waypoint flown to and its id, the laps, how far along the segment
    /// and the route, the distance and time to go, the cross-track, and what
    /// it commands: the course, the altitude, the segment's speed.
    bool progress(ActivityProgress& out) const noexcept override;
    /// The points it flies from here (docs/flight-autonomy.md, 4.37): as planned, a branch taken included.
    std::uint32_t ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept override;
    /// Where it is along its segments (4.47; Intercept.cpp): the point flown from and when it was captured (its state's clock),
    /// the point flown to and how far along the path it is, the one after it and the leg on; its loiter's orbits there.
    bool segments(SegmentEstimate& out) const noexcept override;
    /// Its takeoff on the runway, a route's that starts on the ground (docs/flight-autonomy.md, 4.52): FA's own rest of it
    /// (4.50); its landing's rollout, a route's that ends in one (4.59), likewise.
    bool handOver(BehaviorCommand& out) const override;
    /// A route that ends in a landing (4.59; LandingRoute.cpp) sets its gear, flaps, brakes and speedbrake as its recovery does.
    bool configures() const noexcept override { return lands_ != 0; }
    /// Its recovery for a landing at its end allocated (4.59): by the host, as it installs one that lands - never in a step.
    void carryLanding();
    void configure(ActuatorCommand& out) const noexcept override;
    double speedbrake() const noexcept override;

private:
    /// Plan the route from where the aircraft is, and fly it from its start. `branchTo` given, a branch taken (4.37;
    /// Branches.cpp): on from point `branchTo` - or, `fromPoint`, from the point flown to, `branchTo` its next - its laps,
    /// the distance flown and the branches' captures kept.
    void restart(const ControlContext& ctx, const RouteCommand& command, double branchTo = kHold, bool fromPoint = true);
    /// Where the aircraft is on the route, passing the pieces it has finished - and the branches at a point as it comes to it.
    route::Fix locate(const ControlContext& ctx, const sim::VehicleState& s, const Performance& performance);
    /// Point `k`'s segment begins: its length and the altitude it climbs from.
    void beginSegment(std::uint32_t k, const sim::VehicleState& s, double atM, double halfArcM, double leadM, bool firstLap, bool fromPoint);
    /// On to the next point, or the end.
    void advance(const sim::VehicleState& s, const Performance& performance);
    /// A rotorcraft that loiters at the end stops at the last point.
    bool stops() const noexcept;
    /// Point i where its moving frame is now, as the step began (docs/flight-autonomy.md, 4.29): the frame's pose into
    /// `pose`; false if the frame's vehicle is gone. A point in no frame, or a fixed one, stays where the host put it.
    bool place(const ControlContext& ctx, std::uint32_t i, FramePose& pose);
    /// Fly to point k: its loiter, if it has one, and how far before it it is reached.
    void aim(std::uint32_t k, const Performance& performance) noexcept;
    /// Its loiter flown (4.31), begun where the leg met it (`begins`); at its end, on to the next point, or the route's end.
    Command loiter(const ControlContext& ctx, const Performance& performance, bool begins);
    /// The segment's speed as it chooses it (4.32): the tables' best now where it optimises it, reached at its acceleration.
    void chooseSpeed(const ControlContext& ctx, const Waypoint& segment, route::Steer& steer);
    /// Its climb optimisation's altitude now (4.32), `routeM` along the route: at the most it climbs or descends holding
    /// its speed, to where it holds (an efficient climb's cheapest altitude), then the rest to arrive at its point; its
    /// rate, while it changes, as `feedforward`.
    double climbProfile(const ControlContext& ctx, const Performance& performance, const Waypoint& segment, double routeM, double& feedforward);
    bool arrival(ArrivalEstimate& out) const noexcept override;
    /// Its speed scheduled to its next arrival window (4.33) or timed state (4.34), `routeM` along the route: as planned
    /// while that arrives within it; else, from then to it, the ground speed that arrives a little inside it (at a state's
    /// time), within the speeds it flies level. Its estimate kept.
    void scheduleArrival(const ControlContext& ctx, const Performance& performance, double routeM, route::Steer& steer);
    /// A loiter on its way to its next arrival window, or at the window's point (4.33; Schedule.cpp).
    bool loitersAhead() const noexcept;
    /// Its speed scheduled to its next arrival window through the loiters on its way (4.33; Schedule.cpp): each loiter's own
    /// time counted, its legs at one speed; flown as planned, and not estimated, where one's end is not known ahead.
    void scheduleThroughLoiters(const ControlContext& ctx, const Performance& performance, double routeM, route::Steer& steer);
    /// Its estimate through the loiter it flies (4.33; Schedule.cpp): what the loiter has left, then the legs on.
    void loiterArrival(const ControlContext& ctx, const Performance& performance);
    /// Its loiter begun (4.33; Schedule.cpp), `c` and `shape` as flown: a window at its point met - on to the next point's -
    /// and when it will be left, where one is ahead.
    void loiterBegun(const ControlContext& ctx, const PatternCommand& c, const PatternShape& shape);
    /// Its planned states from the path store (4.34; States.cpp): false for one without a place, or past the route.
    bool takeStates(const ControlContext& ctx) noexcept;
    /// A linked route's points from the path store in its flight order (docs/flight-autonomy.md, 4.36; Paths.cpp): true
    /// for one, its plan's count, repeat and loop the host's.
    bool takeOrder(const ControlContext& ctx) noexcept;
    /// The plan's points in the flight order from `start` (its next `startNext`, where given) along the path store's links
    /// (4.37; Branches.cpp): false where there is none.
    bool orderFrom(const ControlContext& ctx, std::uint32_t start, double startNext, bool repeat) noexcept;
    /// Its civil path terminators' data from the path store (4.38; Terminators.cpp), and whether a point it flies has one.
    void takeTerminators(const ControlContext& ctx) noexcept;
    /// A direct to fix's leg, from where the aircraft is (4.38; Terminators.cpp): made again from here - what it flew since
    /// counted - with the turn at its point, until its track is within a degree of it; then that leg. Its point come abeam
    /// first (too near to turn to it), it is passed, as a point is passed abeam.
    void direct(const sim::VehicleState& s, const Performance& performance);
    /// How the leg to point k is flown and ends, as its terminator says (4.38; Terminators.cpp): pursued, at an altitude,
    /// as it meets the next leg, on a heading.
    void terminated(std::uint32_t k) noexcept;
    /// A leg that ends where the aircraft is (4.38; Terminators.cpp), `f` the aircraft on it: at its altitude (within 10 m, or
    /// past it), or as it meets the next leg - its cross-track to it within the turn onto it, closing, or crossed.
    bool reached(const ControlContext& ctx, const sim::VehicleState& s, const Performance& performance, const route::Fix& f);
    /// Past such a leg's end (4.38; Terminators.cpp): what it flew counted, on to the next point, its leg from here.
    void passHere(const ControlContext& ctx, const sim::VehicleState& s, const Performance& performance, const route::Fix& f);
    /// A branch at the point flown to that takes the operator's input commanded, and holding (4.38, 4.37; Terminators.cpp):
    /// what ends a manual termination.
    bool commandedHere(const ControlContext& ctx) const noexcept;
    /// A hold's terminator at the point flown to (4.38; Terminators.cpp): as its loiter begins, `shape` flown once round
    /// (HF), and whether it ends; as it flies, ended at its fix once at its altitude (HA) or commanded (HM).
    bool holdBegins(PatternShape& shape) const noexcept;
    void holdEnds(const ControlContext& ctx, const sim::VehicleState& s);
    /// A hold to an altitude it is at as it comes to it (4.38; Terminators.cpp): not held, its fix passed.
    bool holdPassed(const ControlContext& ctx, const sim::VehicleState& s) const noexcept;
    /// A heading leg's command (4.38; Terminators.cpp): its heading as the hsa flies one - a wing's trimmed on what its
    /// loops leave - at the segment's speed, the route's vertical speed.
    VelocityCommand headingCommand(const ControlContext& ctx, const sim::VehicleState& s, const Performance& performance, const route::Steer& steer);
    /// The point flown to come to (4.37; Branches.cpp): captured, its branches tried in their order - the first that holds
    /// taken, the route planned again from here (`flownM` more of it flown), on from the point (`fromPoint`: where it has a
    /// turn ahead) or from the branch's next. True where one was; once each time it comes to it.
    bool branchAt(const ControlContext& ctx, const Performance& performance, bool fromPoint, double flownM);
    /// Branch `b`'s conditions hold now, the point come to `captures` times.
    bool holds(const ControlContext& ctx, const RouteBranch& b, std::uint32_t captures) const noexcept;
    /// Its next timed target this lap, from `lapM` along it (4.33, 4.34): the nearest of the next point with an arrival
    /// window and a timed state not passed; and whether the segment flown has states' altitudes. Its schedule kept while
    /// it is the one.
    void nextArrival(double lapM) noexcept;
    /// The segment's altitude through its states' (4.34), `routeM` along the route: straight from each to the next,
    /// from where it climbs from to its point; the gradient's rate as `feedforward`.
    double stateAltitude(double routeM, double& feedforward) const noexcept;
    /// The ground speed that takes it from `fromM` to `toM` along the first lap in `seconds`, its climbs no faster than it
    /// climbs them (4.34): within `lo` to `hi`, the least that makes it; `hi` where none does.
    double paceThroughClimbs(double fromM, double toM, double seconds, double lo, double hi, double alongMs) const noexcept;
    /// The fastest it climbs the piece `lapM` along the first lap is in (over the ground), or infinity where it is not in one.
    double climbMostHere(double lapM, double alongMs) const noexcept;

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
    std::uint16_t constraints_ = 0; ///< its last update's ActivityFlag bits (4.35)
    // the segment's speed as it chooses it (docs/flight-autonomy.md, 4.32)
    double rampFromMs_ = kHold, rampStartS_ = 0.0; ///< its acceleration's ramp: from this speed (true air, or a rotorcraft's ground), then
    double speedFlown_ = kHold, referenceFlown_ = kHold; ///< the speed it flies now, where it optimises or ramps it
    // its climb optimisation's profile (4.32), in the segment's reference
    double climbTarget_ = kHold;       ///< the altitude the profile has reached; kHold: the segment has none
    double climbMid_ = kHold;          ///< the altitude it holds between (an efficient climb's cheapest; else its end)
    double climbLastS_ = 0.0;          ///< when the profile last moved
    bool climbRest_ = false;           ///< the rest of the change, from where it held, begun
    // its next point with an arrival window (4.33), or timed state (4.34): the nearer
    std::int32_t arrivalPoint_ = -1;   ///< its index; -1: none ahead this lap (or a state first)
    std::int32_t arrivalState_ = -1;   ///< the state's; -1: none first
    bool segmentFirstLap_ = true;      ///< the segment flown is the first lap's: its states flown
    bool stateAltitudes_ = false;      ///< the segment flown has states' altitudes (4.34)
    double arrivalAimS_ = kHold;       ///< when it is to arrive there, once its schedule has begun; kHold: as planned
    double arrivalSpeedMs_ = kHold;    ///< the ground speed its schedule last asked (held through its last second)
    double arrivalS_ = kHold, arrivalDeltaS_ = kHold; ///< when it is estimated to arrive there, and that against its window
    double arrivalShiftM_ = 0.0;       ///< past a loiter on its first lap: the lap's measure less its own, where the leg on began
    double loiterEndsS_ = kHold;       ///< when the loiter it flies will be left (the world's seconds), a window ahead; kHold: not known
    double loiterShortM_ = 0.0;        ///< and how much shorter the leg on is from where it is left than from its point
    // its points in moving frames (4.29)
    bool moving_ = false;              ///< placed and planned again as it flies them
    bool overFrame_ = false;           ///< the piece flown is in one moving frame: flown over it
    double frameNorthMs_ = 0.0, frameEastMs_ = 0.0;
    // its loiter points' loiters (4.31)
    std::unique_ptr<PatternBehavior> loiter_; ///< flies them: allocated with the behaviour
    Command loiterCommand_{};          ///< the one flown's pattern
    const RouteLoiter* loiterAhead_ = nullptr; ///< the point flown to's (the plan's); null for none
    double reachM_ = 0.0;              ///< how far before the point flown to it is reached: a loiter's join, a hover's stopping distance
    bool loitering_ = false;           ///< its loiter flies
    // the leg to the point flown to and the turn there, as this lap flies them (the plan's: the entry, a later lap's own
    // where it comes back to - 4.36 - or the legs' and turns'): chosen as it aims, read as it flies
    const route::Leg* legTo_ = nullptr;
    const route::Turn* turnAt_ = nullptr;
    bool decided_ = false; ///< the point flown to's branches tried (4.37): once each time it comes to it
    bool pursuing_ = false; ///< the leg to the point flown to, a direct to fix's, made from where it is as it turns to it (4.38)
    std::uint8_t ends_ = 0; ///< how the leg to it ends (4.38; route::endOf): 0 at it, else where the aircraft is
    bool abeam_ = false;    ///< a direct to fix's point come abeam as it turned to it: passed (4.38)
    bool headed_ = false;   ///< the leg to it a heading's (4.38)
    double interceptM_ = kHold; ///< an intercept's: its cross-track to the next leg at the last update
    double headingTrim_ = 0.0, lastHeading_ = kHold; ///< a heading leg's trim and the heading it last saw, as the hsa's
    bool stopping_ = false;            ///< a rotorcraft's stop at the end: its position loop's, from where it would stop (until canceled)
    // its start on the ground (4.52; GroundRoute.cpp): its taxi to the runway and its takeoff, flown by their behaviours
    // before its first point in the air - allocated at its first
    void beginGround(const ControlContext& ctx);
    void beginTakeoff(const ControlContext& ctx);
    Command ground(const ControlContext& ctx, const Command& in);
    bool groundProgress(ActivityProgress& out) const noexcept;
    /// Whether point `i` as given is a taxi's (its type, or its path's): a branch to one is its rejected takeoff's, never taken
    /// in the air (4.52).
    static bool taxiPoint(const PathStore& store, std::uint32_t i) noexcept;
    std::uint8_t ground_ = 0; ///< 0 in the air (or none); 1 its taxi, 2 its takeoff, 3 off the runway rejected, 4/5 stopped so
    std::unique_ptr<TaxiBehavior> taxi_;     ///< allocated with the behaviour: nothing in a step
    std::unique_ptr<LaunchBehavior> launch_; ///< likewise
    // its landing at its end (4.59; LandingRoute.cpp): flown by its recovery after its last point in the air, then its taxi off
    // the runway
    Command flyRoute(const ControlContext& ctx, const Command& in); ///< its points in the air
    Command beginLanding(const ControlContext& ctx, const Command& in);
    Command landed(const ControlContext& ctx, const Command& in);
    bool landedProgress(ActivityProgress& out) const noexcept;
    std::uint8_t lands_ = 0; ///< 0 none; 1 its points in the air first, 2 landing, 3 its taxi after, 4 done
    std::unique_ptr<RecoveryBehavior> landing_; ///< allocated as the host installs one that lands (carryLanding)
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
    /// Flown inside a route, as a loiter point's loiter (docs/flight-autonomy.md, 4.31): its shape and its frame given,
    /// not the path store's, and ended at `endTimeS` too (the world's time; kHold: none); a hover with its nose held on
    /// `headingRad`, the course the leg into it arrives on (kHold: as the position loop turns it).
    friend class RouteBehavior;
    void embed(const PatternShape& shape, const FrameSpec& frame, double endTimeS, double headingRad) noexcept;
    /// Ended, inside a route (4.38: a hold's terminator): left as its exit point next comes, as its end had come.
    void finishAtExit() noexcept { exitNow_ = true; }

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
    // flown inside a route (4.31)
    bool embedded_ = false;
    PatternShape embeddedShape_{};   ///< its shape as the route gives it
    double endTimeS_ = kHold, worldNow_ = 0.0; ///< its end time, and the world's time at the last update (read only with one)
    bool exitNow_ = false;           ///< ended by the route that flies it (finishAtExit)
    double hoverHeadingRad_ = kHold; ///< a hover's heading as the route gives it (kHold: the position loop's)
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

/// "must_fly": fsim.guidance.must_fly, A-GRA's must fly (docs/flight-autonomy.md,
/// 4.42). The host lays the location out as a route in the path store - the
/// points it approaches through where it has a window of bearings to come
/// from, then the location, flown over - and the route's follower flies it
/// (RouteBehavior), a moving location's points in its frame as it moves. A
/// zone (4.43) is entered: the route aims into it, and it completes once the
/// aircraft is in it, over the ground and within its band. Over
/// another vehicle, the leg to the first point is flown to where that vehicle
/// will be as the aircraft gets there (a collision course), which closes on
/// the vehicle itself. It completes as the location is passed, and flies on
/// along its course there; the vehicle it flies over gone, it fails
/// `target_lost`.
class FSIM_API MustFlyBehavior final : public Behavior {
public:
    MustFlyBehavior();
    ~MustFlyBehavior() override;
    const char* id() const noexcept override { return "must_fly"; }
    void begin(const ControlContext& ctx, const Command& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool finished() const noexcept override;
    Reason failure() const noexcept override;
    std::uint16_t constraints() const noexcept override;
    /// The route's (4.42): the point flown to - an approach's, then the location's - the distance and time to go, the
    /// cross-track, what it commands.
    bool progress(ActivityProgress& out) const noexcept override;
    std::uint32_t ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept override;

private:
    bool arrival(ArrivalEstimate& out) const noexcept override;
    /// The route flown: over another vehicle, its first leg to where that vehicle will be (4.42).
    Command fly(const ControlContext& ctx, const Command& in);

    struct Lead; ///< the world as the route sees it: the vehicle flown over where it will be (MustFly.cpp)
    std::unique_ptr<RouteBehavior> route_; ///< flies the location's route: allocated with the behaviour
    std::unique_ptr<Lead> lead_;           ///< likewise
    Command options_;                      ///< the route's options (a RouteCommand): a great circle's legs, on at its end
    bool zoned_ = false, inside_ = false;  ///< a zone's (4.43), and in it: completed
    std::uint32_t areaRevision_ = 0;       ///< the path store's revision its zone was looked for in
};

/// "recovery": A-GRA's RECOVERY to an airfield's runway (docs/flight-autonomy.md, 4.53; ADR-29 FA-10a). params: airfield and
/// runway (their ids: the vehicle's, 4.40). The host resolves the runway's landing line at the NEW and lays out the approach as
/// a route in the path store. A wing flies it at its approach speed onto the extended centre line and down a 3 deg glide slope,
/// its gear and flaps out from the intermediate fix on; flares - its sink eased to its height over 3 s, its speed bled off -
/// touches down and rolls out as a rejected takeoff stops, on the centre line, then completes stopped. A rotorcraft flies to a
/// hover 10 m over the runway, descends straight down and completes on the ground, its collective down.
///
/// A wing goes around (4.54) from an approach not stable below 150 m, a crosswind beyond its limit down the glide slope, a
/// bounce of more than 3 m, or no touchdown by the touchdown zone's end: climbing straight ahead to the circuit's altitude, its
/// flaps back to a takeoff's and its gear up, then its missed approach - FA's own landing path's chained one where FA keeps one
/// for the runway, else the circuit back to its approach - and a second approach. Going around from that, it flies its missed
/// approach and fails: CrosswindLimit if the wind waved it off, else LandingAbandoned.
class FSIM_API RecoveryBehavior final : public Behavior {
public:
    enum class Phase : std::uint8_t { Approach, Flare, Rollout, Descent, Landed, GoAround, Missed };
    /// Why it went around last.
    enum class GoAround : std::uint8_t { None, Unstable, Crosswind, Bounce, Long };
    RecoveryBehavior();
    ~RecoveryBehavior() override;
    const char* id() const noexcept override { return "recovery"; }
    void start(const ControlContext& ctx, const BehaviorCommand& command) override;
    Command update(const ControlContext& ctx, const Command& in) override;
    void reset() override;
    bool finished() const noexcept override;
    Reason failure() const noexcept override;
    std::uint16_t constraints() const noexcept override;
    bool progress(ActivityProgress& out) const noexcept override;
    bool handOver(BehaviorCommand& out) const override;
    bool configures() const noexcept override { return true; }
    void configure(ActuatorCommand& out) const noexcept override;
    double speedbrake() const noexcept override;
    Reason amend(const BehaviorCommand& update) noexcept override;
    Phase phase() const noexcept { return phase_; }
    GoAround lastGoAround() const noexcept { return cause_; }
    std::uint32_t goArounds() const noexcept { return goArounds_; }

private:
    Command runway(const ControlContext& ctx, const Command& in, double dt);
    Command vertical(const ControlContext& ctx, double dt);
    Command goAround(const ControlContext& ctx, double alongM, double crossM);
    Command rotorGoAround(const ControlContext& ctx, double heading, double dt);
    bool arrived(const sim::VehicleState& s, double lat, double lon, double mslM, double scaleM) const noexcept;
    void goAroundFrom(GoAround cause) noexcept;
    void approachAgain(const ControlContext& ctx, std::uint32_t from);
    std::unique_ptr<RouteBehavior> route_;    ///< flies the approach: allocated with the behaviour
    std::unique_ptr<LaunchBehavior> rollout_; ///< a wing's rollout: likewise
    Command options_;                         ///< the approach route's options (a RouteCommand)
    Phase phase_ = Phase::Approach;
    bool hovers_ = false, configured_ = false;
    double thrLat_ = 0.0, thrLon_ = 0.0, courseRad_ = 0.0, lengthM_ = 0.0, elevationM_ = 0.0, vappMs_ = 0.0, aimM_ = 0.0;
    double airfield_ = 0.0, runway_ = 0.0, settledS_ = 0.0, lastS_ = -1.0, descentMslM_ = 0.0;
    double tailRad_ = kHold, flarePitchRad_ = 0.0, flareIntegral_ = 0.0; ///< its tail's touching attitude; its flare's
    double tailWheelRad_ = kHold; ///< a tail-wheel aircraft's attitude on its main and tail wheels (4.65); NaN: not one
    double vrMs_ = 0.0;           ///< its rotation speed (calibrated): its rollout's, where it would fly again
    double speedAddMs_ = 0.0; ///< its approach speed's change for the attitude it comes down at (calibrated)
    double flaps_ = 1.0;      ///< its flaps for landing, eased for that attitude
    double speedbrake_ = 0.0; ///< its drag devices down the glide slope (4.55)
    double gearMaxMs_ = kHold, flapsMaxMs_ = kHold, flapsAbove_ = 0.0; ///< its placards (4.57): NaN, none
    double casMs_ = 0.0;      ///< its calibrated airspeed as last updated
    bool gearOut_ = false;    ///< its gear lowered (below its placard)
    std::uint8_t configuration_ = 0; ///< commanded (4.58): 0 its own, 1 cleaned up, 2 dirtied up
    double placard(double flaps) const noexcept; ///< its flaps within their placard
    double casRateMs2_ = 0.0, lastCasMs_ = 0.0, slopeS_ = 0.0; ///< its calibrated airspeed's rate, smoothed; its time down the slope
    // its go-arounds (4.54)
    WindEstimate wind_;
    double crosswindMaxMs_ = kHold, circuitMslM_ = 0.0, zoneEndM_ = 0.0; ///< its limit; the go-around's altitude; touched down by
    double touchAglM_ = 0.0, unstableS_ = 0.0, crosswindS_ = 0.0;      ///< its height as it touched; how long each has held
    std::uint32_t missedFrom_ = 0;   ///< the approach route's first point of FA's chained missed approach; 0: the circuit
    double appOutM_ = 0.0, appMslM_ = 0.0, hoverMslM_ = 0.0; ///< a rotorcraft's approach point's way out and height; its hover's (4.66)
    bool intoWind_ = false; ///< a helicopter's hover and descent into the wind (4.66)
    double goAroundS_ = 0.0; ///< a rotorcraft's time in its go-around
    std::uint32_t goArounds_ = 0;
    GoAround cause_ = GoAround::None;
    bool gearUp_ = false, lastApproach_ = false, airbrakes_ = false;
    Reason failed_ = Reason::None;
};
/// Registers "recovery" (Recovery.cpp); registerGuidanceModes calls it.
void registerRecovery(ControllerRegistry& registry);

/// Registers the modes' behaviours ("hsa", "route", "pattern", "curve", "must_fly", "marshall", "intercept");
/// registerBuiltinControllers calls it.
void registerGuidanceModes(ControllerRegistry& registry);
/// Registers "must_fly" (MustFly.cpp); registerGuidanceModes calls it.
void registerMustFly(ControllerRegistry& registry);
/// Registers "marshall" (Marshall.cpp; docs/flight-autonomy.md, 4.46), its parameters its own and then its pattern's shape's:
/// flown by a PatternBehavior, its activity's command its pattern at its slot. registerGuidanceModes calls it.
void registerMarshall(ControllerRegistry& registry, const std::vector<ParameterInfo>& shape);
/// Registers "intercept" (Intercept.cpp; 4.47): flown by a RouteBehavior, its activity's command its plan's route, joined where
/// it chose. registerGuidanceModes calls it.
void registerIntercept(ControllerRegistry& registry);

} // namespace fsim::control
