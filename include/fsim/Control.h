#pragma once

// Multi-level control (design 9.3, ADR-20): commands, controller and
// behaviour interfaces. One plain struct per
// level; `kHold` (NaN) in a field means "keep the current value / let the
// controller decide", so partial commands are natural.

#include "fsim/Capability.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Frames.h"
#include "fsim/Rng.h"
#include "fsim/Span.h"
#include "fsim/VehicleState.h"

#include "fsim/Export.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <memory>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace fsim::control {

/// The control levels. Their values are the C ABI's and Python's; their order
/// in the cascade is levelRank's.
enum class Level : std::uint8_t { Actuator = 0, Attitude, Acceleration, Velocity, Position, Behavior, Count };

FSIM_API const char* levelName(Level level) noexcept;

/// Where a level sits in the cascade, from the bottom: actuator 0,
/// acceleration 1 (the roll rate and load factor an attitude is flown with),
/// attitude 2, velocity 3, position 4, behaviour 5. A controller returns a
/// command at a level of lower rank than its own.
constexpr int levelRank(Level l) noexcept {
    switch (l) {
    case Level::Actuator: return 0;
    case Level::Acceleration: return 1;
    case Level::Attitude: return 2;
    case Level::Velocity: return 3;
    case Level::Position: return 4;
    case Level::Behavior: return 5;
    default: return -1;
    }
}
/// The levels with a controller, from the top of the cascade down.
inline constexpr Level kCascadeOrder[] = {Level::Position, Level::Velocity, Level::Attitude, Level::Acceleration};

inline constexpr double kHold = std::numeric_limits<double>::quiet_NaN();
inline bool isHold(double v) noexcept { return std::isnan(v); }
/// `v` unless it is kHold, then `fallback`.
inline double orHold(double v, double fallback) noexcept { return std::isnan(v) ? fallback : v; }

/// Normalised actuator positions, JSBSim conventions: aileron +right,
/// elevator +nose-down, rudder +nose-left (trailing edge left), throttle
/// 0..1, flaps 0..1.
struct ActuatorCommand {
    double aileron = 0.0, elevator = 0.0, rudder = 0.0;
    double throttle = 0.0;
    double flaps = 0.0;
    double gearDown = kHold;
    double brakeLeft = 0.0, brakeRight = 0.0;
};

/// Attitude hold. With `headingRad` set the roll is derived from the heading
/// error (bounded by `maxBankRad`) and `rollRad` is ignored. Throttle either
/// directly or through an airspeed hold. A rotorcraft flies the heading with
/// its yaw, keeping the roll asked for (and without one holds the heading it
/// had); its thrust is the throttle alone (docs/rotorcraft.md, 3.4).
struct AttitudeCommand {
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double headingRad = kHold;
    double maxBankRad = 0.785;
    double throttle = kHold;
    double airspeedMs = kHold;
};

/// Manoeuvre-style command: normal load factor and roll rate (enough for
/// loops, rolls and hard turns), optional longitudinal acceleration hold.
/// A rotorcraft flies body rates and its thrust: the roll, pitch and yaw
/// rates, and the load factor its rotors' thrust gives (1 in the hover);
/// it has no longitudinal acceleration of its own. A wing has no pitch or
/// yaw rate of its own to fly (its pitch follows the load factor).
struct AccelerationCommand {
    double loadFactorG = 1.0;
    double rollRateRadS = 0.0;
    double longitudinalMs2 = kHold;
    double throttle = kHold;
    double pitchRateRadS = kHold; ///< body, + nose up (a rotorcraft)
    double yawRateRadS = kHold;   ///< body, + nose right (a rotorcraft)
};

/// Flight-path command: airspeed, vertical speed and either a heading to hold
/// or a turn rate to fly. A rotorcraft may be given a velocity over the
/// ground instead of the airspeed (north and east, both), which it flies
/// whatever its heading: hovering, sideways, holding a point against the wind.
struct VelocityCommand {
    double airspeedMs = kHold;
    double verticalSpeedMs = 0.0;
    double headingRad = kHold;
    double turnRateRadS = kHold;
    double northMs = kHold; ///< over the ground (a rotorcraft)
    double eastMs = kHold;
};

/// Fly to a geodetic point at an altitude; "captured" within the radius. A
/// rotorcraft stops there (its airspeed the most it flies at on the way),
/// facing `headingRad` - or, without one, the point while it is far, then
/// the way it came.
struct PositionCommand {
    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double altitudeMslM = 0.0;
    double airspeedMs = kHold;
    double captureRadiusM = 200.0;
    double headingRad = kHold; ///< (a rotorcraft)
};

// --- Guidance modes (docs/vehicle-interface.md, ADR-28): fixed-size setpoints,
// flown by a behaviour per activity, that take UPDATE ----------------------------------

/// What a mode's speed is measured against (A-GRA's SpeedReferenceEnum and MachType).
enum class SpeedReference : std::uint8_t { TrueAirspeed = 0, CalibratedAirspeed = 1, GroundSpeed = 2, Mach = 3, Count };
/// What a mode's altitude is measured from (A-GRA's AltitudeReferenceEnum). The
/// simulation's sea level is the WGS-84 ellipsoid (JSBSim's), so Msl and
/// Ellipsoid are one; AboveGround follows the terrain under the aircraft;
/// Barometric is what the vehicle's altimeter reads, set to its QNH
/// (fsim/Altimeter.h; docs/flight-autonomy.md, 4.20): an isobar, flown as the
/// air and the setting move it. A route's is FA-6's.
enum class AltitudeReference : std::uint8_t { Msl = 0, AboveGround = 1, Ellipsoid = 2, Barometric = 3, Count };
/// What a heading or a course is measured from (A-GRA's MA_HeadingReferenceEnum;
/// docs/flight-autonomy.md, 4.22): true north, or magnetic north - the World
/// Magnetic Model's declination where the aircraft is, at the world's date
/// (fsim/Magnetic.h), turns it to true.
enum class DirectionReference : std::uint8_t { TrueNorth = 0, MagneticNorth = 1, Count };
/// The speed a mode varies by itself (A-GRA's SpeedOptimizationEnum;
/// docs/flight-autonomy.md, 4.17): the performance tables' best-range speed
/// (the most distance for the fuel) or best-endurance speed (the most time),
/// at the altitude and the weight now, flown as a true airspeed.
enum class SpeedOptimization : std::uint8_t { LongRangeCruise = 0, MaxEndurance = 1, Count };
/// How a route's segment climbs or descends, its rate chosen by the aircraft (A-GRA's ClimbOptimizationEnum;
/// docs/flight-autonomy.md, 4.32): at the best rate it achieves, or as efficiently as it can.
enum class ClimbOptimization : std::uint8_t { BestRate = 0, ExtendedRange = 1, Count };

/// fsim.guidance.hsa (A-GRA's HSA/CSA): hold a heading or a course, a speed
/// and an altitude, until told otherwise. Each field may be left out (kHold):
/// a NEW keeps what a live hsa activity it replaces commanded, else what the
/// aircraft flies now; an UPDATE keeps what was commanded. A reference given
/// without its value takes the aircraft's own now (a Mach reference alone:
/// hold the Mach it flies). A heading replaces a course and a course a heading;
/// a speed replaces a speed optimisation and an optimisation a speed.
/// References are enum values carried as doubles, so kHold can mean "as before".
struct HsaCommand {
    double headingRad = kHold;        ///< the nose's direction, from true north (or magnetic: directionReference)
    double courseRad = kHold;         ///< or the track over the ground's
    double speed = kHold;             ///< m/s, or a Mach number
    double speedReference = kHold;    ///< SpeedReference
    double altitudeM = kHold;
    double altitudeReference = kHold; ///< AltitudeReference
    /// SpeedOptimization: the speed it varies by itself. Resolved, `speed` is
    /// the optimum's true airspeed at the altitude flown to, as the command
    /// was given; the mode flies it afresh at the altitude and weight now.
    double speedOptimization = kHold;
    /// DirectionReference of the heading or course: magnetic, it is flown
    /// turned by the declination where the aircraft is (refreshed every 10 s).
    double directionReference = kHold;
};

/// How a route passes a waypoint (A-GRA's TurnType).
enum class TurnType : std::uint8_t {
    FlyBy = 0,   ///< the turn begins before the point, on a circle tangent to both legs (A-GRA's TURN_SHORT)
    FlyOver = 1, ///< over the point, then the next leg is intercepted
    // A-GRA's other turn points (docs/flight-autonomy.md, 4.30)
    CaptureOutboundCourse = 2, ///< over the point, then its course captured: the next point along it
    StartTurn = 3,             ///< an arc begins at the point: to the next, tangent to its course here (ARINC 424's RF)
    EndTurn = 4,               ///< the arc from the point before ends here; on straight
    Count
};
/// What a route's legs are on the Earth.
enum class Projection : std::uint8_t { GreatCircle = 0, Rhumb = 1, Count };
/// What a route does after its last point.
enum class EndBehavior : std::uint8_t {
    Continue = 0, ///< the last leg's course, altitude and speed, on along the leg
    Loiter = 1,   ///< a wing orbits the point, at the radius its speed and 80 % of its bank give; after a route a rotorcraft stops
                  ///< and hovers there, after a curve it circles its end too (A-GRA's CIRCULAR_LOITER)
    Count
};

/// What a waypoint is for (A-GRA's WaypointTypeEnum; docs/flight-autonomy.md, 4.29): navigation alone, or the
/// action it marks - a taxi, a runway's points, an approach's, a takeoff's, a touchdown, a ditch, the end of its path.
enum class WaypointType : std::uint8_t {
    NavOnly = 0,
    Taxi,
    RunwayStart,
    RunwayThreshold,
    RunwayLimit,
    Approach,
    ApproachInitialPoint,
    ApproachFinalPoint,
    Takeoff,
    TakeoffInitialPoint,
    TakeoffFinalPoint,
    Touchdown,
    Passive, ///< (A-GRA gives it no meaning: flown as NavOnly)
    HardDitch,
    EndOfPath,
    Count
};

/// One waypoint of a route (A-GRA's), and the segment that ends at it: flown
/// to from the previous point, the first from where the aircraft is when the
/// route starts. A field left out (kHold) continues the previous point's; the
/// first point's is the aircraft's own now - a reference given alone, its
/// value in that reference now - and a rotorcraft given no speed flies its
/// cruise speed over the ground.
struct Waypoint {
    double latitudeRad = 0.0, longitudeRad = 0.0;
    double altitudeM = kHold;          ///< reached at the point, along a straight profile from the previous one
    double altitudeReference = kHold;  ///< AltitudeReference
    double speed = kHold;              ///< flown on the segment to the point: m/s, or a Mach number
    double speedReference = kHold;     ///< SpeedReference
    double turn = 0.0;                 ///< TurnType
    double maxBankRad = kHold;         ///< the bank its fly-by turn is planned with; kHold: 80 % of the aircraft's
    double climbRateMs = kHold;        ///< climb or descend at this rate, then level; kHold: along the segment's gradient
    std::uint64_t id = 0;              ///< the caller's, reported back in the progress
    // A-GRA's end point as its schema gives it (docs/flight-autonomy.md, 4.29)
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< its altitude block, in its reference: left out, the altitude held within it
    double kind = kHold;               ///< EndPointKind: left out, a turn point as `turn` says; a waypoint (no turn) is flown over;
                                       ///< a loiter point flies its RouteLoiter there (docs/flight-autonomy.md, 4.31)
    double waypointType = kHold;       ///< a waypoint's WaypointType (left out: NavOnly)
    double frame = kHold;              ///< a point in this frame (World::createFrame's id): its latitude and longitude where the frame puts it
    double frameRotation = kHold, frameOffsets = kHold; ///< its offsets' FrameRotation and FrameOffsets, as a pattern's point's
    double frameXM = kHold, frameYM = kHold, frameZM = kHold; ///< its offsets (z down: given, its altitude the frame's there)
    double courseRad = kHold;          ///< the course at the point (4.30): a start's arc's, an end's, the course a capture captures
    double turnRadiusM = kHold;        ///< its turn's radius: a fly-by's, a start's arc's (TurnGeometry)
    // A-GRA's segment performance (docs/flight-autonomy.md, 4.32)
    double speedOptimization = kHold;  ///< SpeedOptimization: the segment at the tables' best now, its speed replaced; left out with
                                       ///< the speed, the point before's
    double climbOptimization = kHold;  ///< ClimbOptimization: its climb or descent at a rate the aircraft chooses
    double accelerationMs2 = kHold;    ///< the speed change into the segment at this acceleration; kHold: as the loops change it
    // A-GRA's required time of arrival (docs/flight-autonomy.md, 4.33): the window it is to arrive at the point in, in the
    // world's simulation seconds - either side left out, open (a begin alone: no earlier; an end alone: no later)
    double arrivalBeginS = kHold;
    double arrivalEndS = kHold;
    // A-GRA's required navigation performance (docs/flight-autonomy.md, 4.35): how far off its path the segment may be flown,
    // m; left out, none. Farther, its activity says so (kActivityNavigationPerformance)
    double rnpM = kHold;
    // A-GRA's NextPathSegment (docs/flight-autonomy.md, 4.36): the waypoint flown after it, its index; -1, the route's end
    // there; left out, the next in its path (after its path's last, the route's end) - no paths given, the next as they are
    double next = kHold;
    // A-GRA's CivilPathTerminator (docs/flight-autonomy.md, 4.38): the ARINC 424 leg type of the leg into it
    // (PathTerminator), its data - a course to fix's, a radius to fix's - in a RouteTerminator beside it; left out, the
    // route's own leg, as before
    double terminator = kHold;
    /// Its point in its frame: the offsets left out, the frame's origin.
    FrameOffset frameOffset() const noexcept {
        FrameOffset o;
        o.rotation = isHold(frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(frameRotation));
        o.offsets = isHold(frameOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(frameOffsets));
        o.x = isHold(frameXM) ? 0.0 : frameXM, o.y = isHold(frameYM) ? 0.0 : frameYM, o.z = isHold(frameZM) ? 0.0 : frameZM;
        return o;
    }
};

/// fsim.guidance.route (A-GRA's waypoint following): its waypoints go beside
/// it (World::submit and update take a Span) into the vehicle's path store.
/// Completes after the last point, unless it repeats.
struct RouteCommand {
    double projection = 0.0; ///< Projection: 0 great circles, 1 rhumb lines
    double repeat = 0.0;     ///< 1: after the last point, fly the route again from its first
    double end = 0.0;        ///< EndBehavior after the last point
    double start = 0.0;      ///< the point to fly to first
};

/// One piece of a curve (A-GRA's): a quintic Bezier by its six control
/// points (weights 1, the clamped knots [0,0,0,0,0,0,1,1,1,1,1,1]), metres
/// north, east and down from the curve's reference point.
struct BezierSegment {
    double north[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double east[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double down[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

/// One piece of a curve as A-GRA's schema gives it (MA_NURBS_PointType;
/// docs/flight-autonomy.md, 4.26): a clamped rational B-spline - its 4 to 10
/// control points, metres north, east and down from the curve's reference,
/// each with its weight, and its 4 to 14 knots. Its degree is their counts'
/// difference less one (knots - points - 1, from 1). Clamped, it starts at its
/// first control point and ends at its last: its first knot comes as many
/// times as its degree and once more, and its last likewise. A BezierSegment
/// is one (NurbsSegment::of): six points, weights 1, knots [0 x6, 1 x6].
struct NurbsSegment {
    static constexpr std::size_t kPoints = 10, kKnots = 14;
    std::uint32_t points = 0; ///< its control points: 4 to 10
    std::uint32_t knots = 0;  ///< its knots: 4 to 14, and at least points + 2
    double north[kPoints] = {}, east[kPoints] = {}, down[kPoints] = {};
    double weight[kPoints] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; ///< each above 0
    double knot[kKnots] = {}; ///< from 0, never decreasing
    /// A-GRA's Curvature: the most it turns over the ground, 1/m - checked against it. kHold: not said.
    double curvature = kHold;
    /// A-GRA's Initial and FinalControlPointIndex: its first and last control points - 0 and its last, as a clamped
    /// curve starts and ends at them. kHold: not said.
    double firstIndex = kHold, lastIndex = kHold;

    /// A Bezier segment as one.
    static NurbsSegment of(const BezierSegment& b) noexcept {
        NurbsSegment s;
        s.points = 6, s.knots = 12;
        for (int i = 0; i < 6; ++i) s.north[i] = b.north[i], s.east[i] = b.east[i], s.down[i] = b.down[i];
        for (int i = 6; i < 12; ++i) s.knot[i] = 1.0;
        return s;
    }
    /// It is a BezierSegment's: six points, weights 1, knots [0 x6, 1 x6] - flown as one is.
    bool bezier() const noexcept {
        if (points != 6 || knots != 12) return false;
        for (int i = 0; i < 6; ++i)
            if (weight[i] != 1.0 || knot[i] != 0.0 || knot[i + 6] != 1.0) return false;
        return true;
    }
    /// Its first six points as a BezierSegment (what it is, when bezier()).
    BezierSegment asBezier() const noexcept {
        BezierSegment b;
        for (int i = 0; i < 6; ++i) b.north[i] = north[i], b.east[i] = east[i], b.down[i] = down[i];
        return b;
    }
    /// Its degree: knots - points - 1.
    int degree() const noexcept { return static_cast<int>(knots) - static_cast<int>(points) - 1; }
};

/// fsim.guidance.curve (A-GRA's curve following): its segments - 1 to 10 a
/// command, each starting where the one before ends - go beside it (World::submit
/// and update take a Span) into the path store. Flown at a ground speed within
/// a range, or so as to take a duration; completes at its end. A field left
/// out (kHold) takes its default in a NEW and keeps its value in an UPDATE.
struct CurveCommand {
    double latitudeRad = kHold, longitudeRad = kHold; ///< the reference its segments are from; kHold: where the aircraft is
    double altitudeM = kHold;          ///< the reference's height in altitudeReference; kHold: the aircraft's (within its range)
    double speedMinMs = kHold;         ///< the ground speed to fly it within; both kHold: as now (a rotorcraft's cruise)
    double speedMaxMs = kHold;
    double durationS = kHold;          ///< or the time to fly all of it, from the NEW
    double end = kHold;                ///< EndBehavior after its end; kHold: continue
    double append = kHold;             ///< in an UPDATE, 1: its segments after the curve's end, from the same reference
    // A-GRA's reference and control points as its schema gives them (docs/flight-autonomy.md, 4.27)
    double altitudeReference = kHold;  ///< AltitudeReference of altitudeM and its range; kHold: above sea level
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< the reference's altitude range (A-GRA's AltitudeRange), in that reference
    double pointRotation = kHold;      ///< FrameRotation of its control points' axes; kHold: north, east, down - turned as its frame is
    double pointOffsets = kHold;       ///< FrameOffsets of their north and east; kHold: the plane at its reference, as every local path's
    double pointZ = kHold;             ///< CurveZ of their third; kHold: down from the reference
};

/// How a curve's control points' third value reads (A-GRA's Z_ChoiceType; docs/flight-autonomy.md, 4.27).
enum class CurveZ : std::uint8_t {
    Down = 0,             ///< metres down from its reference (A-GRA's Z)
    AltitudeOffset = 1,   ///< metres up from it (AltitudeOffset)
    AbsoluteAltitude = 2, ///< the altitude itself, in the curve's altitude reference (AbsoluteAltitude)
    Count
};

/// A curve's reference as a point in a reference frame (A-GRA's RelativePoint; docs/flight-autonomy.md, 4.27): the
/// frame's id (World::createFrame) and the point's offsets from its origin - turned as `frameRotation` says
/// (FrameRotation), laid out as `frameOffsets` says (FrameOffsets), x and y, and z down (kHold: the curve's own
/// altitude). Beside the CurveCommand, as a pattern's shape is: the curve's reference is the frame's point, carried
/// with it as it moves (and its axes turned with it, as its pointRotation says); its latitude and longitude report
/// where it was when placed. Given a latitude and longitude in an UPDATE, the frame is left; given a frame, the point
/// is its.
struct CurveShape {
    double frame = kHold;
    double frameRotation = kHold, frameOffsets = kHold;
    double frameXM = kHold, frameYM = kHold, frameZM = kHold;

    /// Its fields in order (the C ABI's and Python's, after the CurveCommand's): pointers into it.
    static constexpr std::size_t kFields = 6;
    void fields(double* f[kFields]) noexcept {
        f[0] = &frame, f[1] = &frameRotation, f[2] = &frameOffsets, f[3] = &frameXM, f[4] = &frameYM, f[5] = &frameZM;
    }
    /// Its point in its frame: the offsets left out, the frame's origin.
    FrameOffset frameOffset() const noexcept {
        FrameOffset o;
        o.rotation = isHold(frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(frameRotation));
        o.offsets = isHold(frameOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(frameOffsets));
        o.x = isHold(frameXM) ? 0.0 : frameXM, o.y = isHold(frameYM) ? 0.0 : frameYM, o.z = isHold(frameZM) ? 0.0 : frameZM;
        return o;
    }
    /// Every field left out: the curve's reference as its CurveCommand gives it.
    bool empty() const noexcept {
        return isHold(frame) && isHold(frameRotation) && isHold(frameOffsets) && isHold(frameXM) && isHold(frameYM) && isHold(frameZM);
    }
};


/// A loiter pattern (A-GRA's LOITER).
enum class PatternKind : std::uint8_t {
    Orbit = 0,       ///< a circle round the centre
    Racetrack = 1,   ///< two half circles joined by straight legs, the inbound one ending at the fix
    FigureEight = 2, ///< two circles meeting at the centre, one flown each way round
    Hold = 3,        ///< ATC's holding pattern: a racetrack on the fix with a minute's legs, entered direct to the fix
    Hover = 4,       ///< a rotorcraft's hover over its point, at its altitude; its duration counted from its arrival there (4.25)
    Count
};

/// fsim.guidance.pattern (A-GRA's loiter): an orbit, a racetrack, a
/// figure-eight or a hold, until canceled or for a duration. A field left
/// out (kHold) takes its default in a NEW and keeps what was commanded in an
/// UPDATE. The defaults: an orbit, where the aircraft is, as it flies now (a
/// rotorcraft at its cruise speed over the ground), right turns, the course it
/// tracks now (a hold's: to its fix), the radius its speed, the wind and 80 %
/// of its bank give (a hold's: rate one, at most 25 degrees of bank), legs of
/// twice the radius (a hold's: a minute's flight, 90 s above 14,000 ft).
/// A-GRA's other ways to give a pattern go beside it, in a PatternShape.
struct PatternCommand {
    double pattern = kHold;            ///< PatternKind
    double latitudeRad = kHold, longitudeRad = kHold; ///< the centre, a racetrack's or a hold's fix, or the first of two circles' centre
    double altitudeM = kHold;
    double altitudeReference = kHold;  ///< AltitudeReference
    double radiusM = kHold;            ///< at least the turn radius at the aircraft's speed and full bank; a rotorcraft's a metre
    double clockwise = kHold;          ///< 1 right turns, 0 left (two circles': round the first)
    double courseRad = kHold;          ///< a racetrack's or a hold's inbound course, a figure-eight's axis
    double legM = kHold;               ///< a racetrack's or a hold's straight legs
    double speed = kHold;              ///< m/s, or a Mach number
    double speedReference = kHold;     ///< SpeedReference
    double durationS = kHold;          ///< then it completes (and flies on); kHold: until canceled
    double speedOptimization = kHold;  ///< SpeedOptimization, as an hsa's: a speed replaces it, it a speed
};

/// A hold's turns by type (A-GRA's MA_HoldTurnTypeEnum; docs/flight-autonomy.md, 4.24): the radius they give
/// at the pattern's speed plus the wind.
enum class HoldTurn : std::uint8_t {
    Standard = 0, ///< rate one, 3 degrees a second, at most 25 degrees of bank (a hold's default)
    MilPower = 1, ///< the tightest the pattern flies: 80 % of the aircraft's bank (an orbit's default)
    Relax = 2,    ///< half rate one, at most 15 degrees of bank
    Count
};
/// How a racetrack or a hold is entered (A-GRA's MA_HoldEntryTypeEnum, and ATC's entries; 4.24). Left out: direct
/// to the fix, then round from the turn there, wherever the aircraft comes from.
enum class HoldEntry : std::uint8_t {
    Direct = 0,   ///< where the pattern is nearest, joined as an orbit is
    Anchor = 1,   ///< at the fix, by ATC's entry for the side the aircraft comes from: direct, parallel or teardrop
    Inbound = 2,  ///< along the inbound leg: onto its course before it, then to the fix
    Outbound = 3, ///< along the outbound leg: onto its course before it, abeam the fix
    Parallel = 4, ///< ATC's parallel entry: over the fix, out along the inbound course a leg, round toward the holding side, back to the fix
    Teardrop = 5, ///< ATC's teardrop entry: over the fix, out 30 degrees into the holding side a leg, round the pattern's way, back to the fix
    Count
};
/// A hold's operational context (A-GRA's MA_HoldContextEnum; 4.24): the defaults it implies are ATC's for every one.
enum class HoldContext : std::uint8_t { Admin = 0, Tactical = 1, Atc = 2, Count };

/// A pattern as A-GRA's schema gives it beyond its PatternCommand
/// (docs/flight-autonomy.md, 4.23): beside it, as a route's waypoints are
/// (World::submit and update take it; the path store keeps the shape of the
/// pattern that flies), so that the setpoint every command is kept in stays
/// the size it was. Its fields follow the PatternCommand's in the C ABI's and
/// Python's one list (index 13 on). A field left out (kHold) is not used in a
/// NEW and keeps what was commanded in an UPDATE. Some give a PatternCommand
/// field another way: a heading for the course, a time for the legs, a bank,
/// a turn rate or a turn type for the radius. Given more than one, the first
/// flies - the PatternCommand's, then the bank, the rate, the type - and the
/// host fills it in from the shape's, which stay as they were given; in an
/// UPDATE, any one replaces the others, and a second circle replaces the
/// course and the legs.
struct PatternShape {
    /// DirectionReference of the course or the heading: magnetic, turned to
    /// true by the declination at the pattern's point when it is planned.
    double directionReference = kHold;
    /// Or the inbound heading: the course it makes good on that heading, in
    /// the wind when the pattern is planned.
    double headingRad = kHold;
    /// Or the legs by time: as long as the inbound leg is flown in so long,
    /// at the pattern's speed in the wind when it is planned.
    double legS = kHold;
    /// Or the turns by bank: the radius at which it banks no more, the wind
    /// with it (as the default radius has it).
    double bankRad = kHold;
    /// Laps: then it completes (and flies on); with a duration, the first
    /// reached. A lap is counted where the pattern was joined.
    double orbits = kHold;
    /// A racetrack or a figure-eight by two circles (A-GRA's): the second's
    /// centre and radius (kHold: the first's). A racetrack's straight legs
    /// are the lines touching both circles on the outside, a figure-eight's
    /// those crossing between them; the pattern's course and legs are the circles'.
    double latitude2Rad = kHold, longitude2Rad = kHold;
    double radius2M = kHold;
    /// Where it joins the pattern (A-GRA's EntryPoint): flown to directly,
    /// the pattern's nearest point to it, and on round from there.
    double entryLatitudeRad = kHold, entryLongitudeRad = kHold;
    /// Where it leaves (A-GRA's ExitPoint): its duration or laps flown, it
    /// goes on round to the pattern's nearest point to it, completes there
    /// and flies on along its course.
    double exitLatitudeRad = kHold, exitLongitudeRad = kHold;
    /// Or the turns by rate (a hold's TurnRate): the radius it turns at that
    /// rate on, at its speed plus the wind.
    double turnRateRadS = kHold;
    /// Or by type: HoldTurn.
    double turnType = kHold;
    /// HoldEntry: how a racetrack or a hold (with no second circle, nor an
    /// entry point) is entered.
    double holdEntry = kHold;
    /// HoldContext of a hold: its defaults ATC's, whichever it is.
    double holdContext = kHold;
    /// Its point in a reference frame (A-GRA's relative point; 4.25): the
    /// frame's id (World::createFrame), and its offsets from the frame's
    /// origin - turned as `frameRotation` says (FrameRotation), laid out as
    /// `frameOffsets` says (FrameOffsets), x and y, and z down (kHold: the
    /// pattern's own altitude). The pattern's point - its centre, fix, first
    /// circle's centre or hover's point - is the frame's, carried with it as
    /// it moves; the pattern's latitude and longitude report where it was
    /// when planned. Given a latitude and longitude in an UPDATE, the frame is
    /// left; given a frame, the point is its.
    double frame = kHold;
    double frameRotation = kHold, frameOffsets = kHold;
    double frameXM = kHold, frameYM = kHold, frameZM = kHold;

    /// Its fields in order (the C ABI's and Python's, after the PatternCommand's): pointers into it.
    static constexpr std::size_t kFields = 22;
    void fields(double* f[kFields]) noexcept {
        f[0] = &directionReference, f[1] = &headingRad, f[2] = &legS, f[3] = &bankRad, f[4] = &orbits, f[5] = &latitude2Rad;
        f[6] = &longitude2Rad, f[7] = &radius2M, f[8] = &entryLatitudeRad, f[9] = &entryLongitudeRad, f[10] = &exitLatitudeRad;
        f[11] = &exitLongitudeRad, f[12] = &turnRateRadS, f[13] = &turnType, f[14] = &holdEntry, f[15] = &holdContext, f[16] = &frame;
        f[17] = &frameRotation, f[18] = &frameOffsets, f[19] = &frameXM, f[20] = &frameYM, f[21] = &frameZM;
    }
    /// Its point in its frame: the offsets left out, the frame's origin.
    FrameOffset frameOffset() const noexcept {
        FrameOffset o;
        o.rotation = isHold(frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(frameRotation));
        o.offsets = isHold(frameOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(frameOffsets));
        o.x = isHold(frameXM) ? 0.0 : frameXM, o.y = isHold(frameYM) ? 0.0 : frameYM, o.z = isHold(frameZM) ? 0.0 : frameZM;
        return o;
    }
    /// Every field left out: the pattern as its PatternCommand alone gives it.
    bool empty() const noexcept {
        const double v[kFields] = {directionReference, headingRad, legS, bankRad, orbits, latitude2Rad, longitude2Rad, radius2M,
                                   entryLatitudeRad, entryLongitudeRad, exitLatitudeRad, exitLongitudeRad, turnRateRadS, turnType, holdEntry,
                                   holdContext, frame, frameRotation, frameOffsets, frameXM, frameYM, frameZM};
        for (const double x : v)
            if (!isHold(x)) return false;
        return true;
    }
    bool twoCircles() const noexcept { return !isHold(latitude2Rad) || !isHold(longitude2Rad); }
};

/// What a route's path is for (A-GRA's MA_PathTypeEnum; docs/flight-autonomy.md, 4.36): a label, reported back.
enum class PathType : std::uint8_t {
    Primary,
    Alternate,
    LossOfComm,
    ReturnToBase,
    SoftDitch,
    HardDitch,
    Ingress,
    Egress,
    Takeoff,
    Landing,
    EmergencyLanding,
    Taxi,
    Airborne,
    Arcing,
    Breaking,
    OnDepartureRadial,
    InitialApproach,
    IntermediateApproach,
    FinalApproach,
    BolterWaveoff,
    Count
};

/// One of a route's paths (A-GRA's MA_RoutePathType; docs/flight-autonomy.md, 4.36): `count` of its waypoints from
/// `first`, flown in order unless a point's `next` says otherwise - its last the route's end unless its `next` goes on -
/// with its id and type. Beside the waypoints, as the loiters are: 16 a route at most, tiling them in order; the route
/// begins at its first path's first point (RouteCommand::start picks another).
struct RoutePath {
    std::uint64_t id = 0;     ///< the caller's (A-GRA's PathID)
    double type = kHold;      ///< PathType; left out, Primary
    std::uint32_t first = 0;  ///< its first waypoint's index
    std::uint32_t count = 0;  ///< how many
};

/// How a value is compared with the one given (A-GRA's EqualityExpressionEnum; docs/flight-autonomy.md, 4.37): the value
/// under test on the left.
enum class Comparison : std::uint8_t { Greater, GreaterEqual, Less, LessEqual, Equal, NotEqual, Count };

/// A route's conditional branch (A-GRA's ConditionalPathSegment, its PathSegmentConditionType; docs/flight-autonomy.md,
/// 4.37): as the aircraft comes to waypoint `point`, the point flown after it is `next` when every condition given holds -
/// its altitude within a range, the time within a window, the point come to so many times, what it has left of its
/// endurance, its contingency - and, where it takes the operator's input, once the operator has commanded it
/// (World::commandBranch). Beside the waypoints, as the loiters are: 16 a route at most; those at one point tried in their
/// order, the first that holds taken. None given, it holds.
struct RouteBranch {
    std::uint32_t point = 0;          ///< the waypoint it branches at (its index as given)
    double next = kHold;              ///< the waypoint flown after it when taken (its index; -1: the route's end there)
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< AltitudeRange: its altitude there (a side left out: open)
    double altitudeReference = kHold; ///< AltitudeReference of the range (left out: above mean sea level)
    double timeBeginS = kHold, timeEndS = kHold; ///< TimeWindow: the world's simulation seconds (a side left out: open)
    double captures = kHold;          ///< SegmentCapture: the times it has come to the point, this one too...
    double capturesComparison = kHold; ///< ...compared so (Comparison) with this count
    double operatorInput = kHold;     ///< OperatorInput: 1, only once the operator has commanded it
    double enduranceComparison = kHold; ///< EnduranceRemaining: what it has left (NavigationReport) compared so (Comparison)...
    double fuelKg = kHold, enduranceS = kHold, enduranceEndS = kHold, percent = kHold; ///< ...with each given: its fuel, endurance, its end, percent
    double contingency = kHold;       ///< ContingencyLevel: it is in this Contingency

    /// Its fields after `point`, in order (the C ABI's and Python's): pointers into it.
    static constexpr std::size_t kFields = 15;
    void fields(double* f[kFields]) noexcept {
        double* all[kFields] = {&next,           &altitudeMinM,  &altitudeMaxM, &altitudeReference,   &timeBeginS,
                                &timeEndS,       &captures,      &capturesComparison, &operatorInput, &enduranceComparison,
                                &fuelKg,         &enduranceS,    &enduranceEndS, &percent,            &contingency};
        for (std::size_t k = 0; k < kFields; ++k) f[k] = all[k];
    }
};

/// A-GRA's civil path terminators (CivilPathTerminatorType; docs/flight-autonomy.md, 4.38): the ARINC 424 leg type of the
/// leg into a waypoint (Waypoint::terminator), in its schema's order - ARINC 424's codes beside them. A-GRA gives data for a
/// course to fix and a radius to fix (RouteTerminator) and leaves the others empty: a leg is flown where its segment carries
/// what defines it.
enum class PathTerminator : std::uint8_t {
    ArcToFix,                         ///< AF: a DME arc round a navaid - none given, not a leg its segment defines
    CourseToAltitude,                 ///< CA: its course until its altitude (FA-6f2)
    CourseToDmeDistance,              ///< CD: to a navaid's DME distance - not defined
    CourseToFix,                      ///< CF: its course into its point (RouteTerminator::courseRad)
    CourseToIntercept,                ///< CI: its course until it meets the next leg (FA-6f2)
    CourseToRadial,                   ///< CR: to a navaid's radial - not defined
    DirectToFix,                      ///< DF: straight to its point from where the aircraft is as the leg begins
    TrackToAltitude,                  ///< FA: from the point before on its course until its altitude (FA-6f2)
    TrackFromFixToDistanceAlongTrack, ///< FC: from the point before on its course for a distance (FA-6f2)
    TrackFromFixToDmeDistance,        ///< FD: to a navaid's DME distance - not defined
    FixToManualTermination,           ///< FM: from the point before on its course until the operator ends it (FA-6f2)
    HoldingWithAltitudeTermination,   ///< HA: a hold until its altitude (FA-6f2)
    HoldingWithFixTermination,        ///< HF: a hold once round, to its fix (FA-6f2)
    HoldingWithManualTermination,     ///< HM: a hold until the operator ends it (FA-6f2)
    InitialFix,                       ///< IF: where a procedure begins - flown to as a direct to fix is
    ProcedureTurnToIntercept,         ///< PI: a procedure turn - its outbound course, side and limit not given: not defined
    RadiusToFix,                      ///< RF: an arc round its centre from the point before (RouteTerminator)
    TrackToFix,                       ///< TF: the great circle from the point before
    HeadingToAltitude,                ///< VA: a heading until its altitude (FA-6f2)
    HeadingToDmeDistanceTermination,  ///< VD: to a navaid's DME distance - not defined
    HeadingToIntercept,               ///< VI: a heading until it meets the next leg (FA-6f2)
    HeadingToManual,                  ///< VM: a heading until the operator ends it (FA-6f2)
    HeadingToRadialTermination,       ///< VR: to a navaid's radial - not defined
    Count
};

/// A civil path terminator's data (A-GRA's CF_CourseToFixType and RF_RadiusToFixType; docs/flight-autonomy.md, 4.38): the
/// leg into waypoint `point`, whose `terminator` names its type - a course to fix's course, or a radius to fix's arc. Beside
/// the waypoints, as the loiters are: 64 a route at most, one a point. A field left out is not checked; a course to fix
/// needs its course, a radius to fix its centre and its way round, and what else it gives must be its arc's.
struct RouteTerminator {
    std::uint32_t point = 0;          ///< the waypoint whose leg it is (its index as given)
    double courseRad = kHold;         ///< CF's Course: the course into its point, from true north
    double centerLatitudeRad = kHold, centerLongitudeRad = kHold; ///< RF's RadiusCenterPoint
    double radiusM = kHold;           ///< RF's TurnRadius
    double courseInRad = kHold, courseOutRad = kHold; ///< RF's CourseIn and CourseOut: the arc's courses at its start and its end
    double initialLatitudeRad = kHold, initialLongitudeRad = kHold; ///< RF's ArcInitialPoint: where it starts, the point before
    double endLatitudeRad = kHold, endLongitudeRad = kHold;         ///< RF's ArcEndPoint: where it ends, its point
    double arcM = kHold;              ///< RF's ArcDistance: along the arc
    double chordM = kHold;            ///< RF's ArcDirectDistance: straight from its start to its end
    double clockwise = kHold;         ///< RF's TurnDirection: 1 right, 0 left (as a pattern's)

    /// Its fields after `point`, in order (the C ABI's and Python's): pointers into it.
    static constexpr std::size_t kFields = 13;
    void fields(double* f[kFields]) noexcept {
        double* all[kFields] = {&courseRad,          &centerLatitudeRad,   &centerLongitudeRad, &radiusM, &courseInRad,
                                &courseOutRad,       &initialLatitudeRad,  &initialLongitudeRad, &endLatitudeRad,
                                &endLongitudeRad,    &arcM,                &chordM,             &clockwise};
        for (std::size_t k = 0; k < kFields; ++k) f[k] = all[k];
    }
};

/// The loiter a route's loiter point flies (A-GRA's LoiterPoint, MA_LoiterPointType; docs/flight-autonomy.md, 4.31):
/// a pattern - an orbit, a racetrack, a figure-eight, a hold or a hover, with its shape - and the time it ends. Beside
/// the route's waypoints, as they go beside its RouteCommand (World::submit and update take a Span): 16 a route at most.
/// Its place is its point's - where the point is, in its frame if it has one, at its altitude - so the pattern's
/// latitude, longitude, altitude and reference, and the shape's frame, are left out; its speed left out, it flies the
/// point's segment's. It ends when its duration or its laps are flown (on round to its exit point, if it has one), or at
/// its end time, the first; with none, it is the route's end (its last point's only).
struct RouteLoiter {
    std::uint32_t point = 0; ///< its point: a waypoint of kind EndPointKind::LoiterPoint
    PatternCommand pattern{};
    PatternShape shape{};
    double endTimeS = kHold; ///< when it ends (A-GRA's EndTime), in simulation seconds; kHold: none

    /// Its pattern's fields then its shape's, in order (the C ABI's and Python's, as a pattern's): pointers into it.
    static constexpr std::size_t kFields = 13 + PatternShape::kFields;
    void fields(double* f[kFields]) noexcept {
        PatternCommand& c = pattern;
        f[0] = &c.pattern, f[1] = &c.latitudeRad, f[2] = &c.longitudeRad, f[3] = &c.altitudeM, f[4] = &c.altitudeReference;
        f[5] = &c.radiusM, f[6] = &c.clockwise, f[7] = &c.courseRad, f[8] = &c.legM, f[9] = &c.speed, f[10] = &c.speedReference;
        f[11] = &c.durationS, f[12] = &c.speedOptimization;
        shape.fields(f + 13);
    }
};

/// A planned inertial state inside a route's segment (A-GRA's InertialState, InertialStateRelativeType;
/// docs/flight-autonomy.md, 4.34): where and when the aircraft is to be - a place on the segment's leg, at its altitude and
/// time - and what else the plan gives there, kept for analysis and read back. Beside the route's waypoints, as its
/// loiters are (World::submit and update take a Span): 64 a route at most. Its place in a frame is where the frame is at
/// its time. The segment flies through the states' altitudes and arrives at each at its time.
struct RouteState {
    std::uint32_t point = 0;           ///< its segment: the one ending at waypoint `point`
    double latitudeRad = kHold, longitudeRad = kHold;
    double altitudeM = kHold;          ///< the altitude it passes it at (left out: the segment's own profile there)
    double altitudeReference = kHold;  ///< AltitudeReference (left out: its point's)
    double timeS = kHold;              ///< when it is to be there, the world's simulation seconds (left out: no time)
    double frame = kHold;              ///< a place in this frame (World::createFrame's id), where the frame is at its time
    double frameRotation = kHold, frameOffsets = kHold; ///< its offsets' FrameRotation and FrameOffsets, as a waypoint's
    double frameXM = kHold, frameYM = kHold, frameZM = kHold; ///< its offsets (z down)
    // what else A-GRA's InertialState gives: kept for analysis, and read back
    double uncertaintyM = kHold;       ///< PositionUncertainty: its radius
    double groundNorthMs = kHold, groundEastMs = kHold;                         ///< GroundVelocity
    double domainNorthMs = kHold, domainEastMs = kHold, domainDownMs = kHold;   ///< DomainVelocity (through the air)
    double relativeNorthMs = kHold, relativeEastMs = kHold, relativeDownMs = kHold; ///< RelativeVelocity (in its frame)
    double accelerationNorthMs2 = kHold, accelerationEastMs2 = kHold, accelerationDownMs2 = kHold; ///< DomainAcceleration
    double yawRad = kHold, pitchRad = kHold, rollRad = kHold;                   ///< Orientation
    double yawRateRadS = kHold, pitchRateRadS = kHold, rollRateRadS = kHold;    ///< OrientationRate

    /// Its fields after `point`, in order (the C ABI's and Python's): pointers into it.
    static constexpr std::size_t kFields = 29;
    void fields(double* f[kFields]) noexcept {
        double* all[kFields] = {&latitudeRad, &longitudeRad, &altitudeM, &altitudeReference, &timeS, &frame, &frameRotation, &frameOffsets, &frameXM,
                                &frameYM, &frameZM, &uncertaintyM, &groundNorthMs, &groundEastMs, &domainNorthMs, &domainEastMs, &domainDownMs,
                                &relativeNorthMs, &relativeEastMs, &relativeDownMs, &accelerationNorthMs2, &accelerationEastMs2, &accelerationDownMs2,
                                &yawRad, &pitchRad, &rollRad, &yawRateRadS, &pitchRateRadS, &rollRateRadS};
        for (std::size_t k = 0; k < kFields; ++k) f[k] = all[k];
    }
    /// Its place in its frame: the offsets left out, the frame's origin.
    FrameOffset frameOffset() const noexcept {
        FrameOffset o;
        o.rotation = isHold(frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(frameRotation));
        o.offsets = isHold(frameOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(frameOffsets));
        o.x = isHold(frameXM) ? 0.0 : frameXM, o.y = isHold(frameYM) ? 0.0 : frameYM, o.z = isHold(frameZM) ? 0.0 : frameZM;
        return o;
    }
};

/// Where a must fly goes (A-GRA's MustFlyLocationType; docs/flight-autonomy.md, 4.42).
enum class MustFlyLocation : std::uint8_t {
    Point = 0,   ///< a place (A-GRA's Point): flown over at its altitude
    Entity = 1,  ///< another vehicle in the world (EntityID): flown over as it moves
    OpPoint = 2, ///< an operational point by its id (OpPointID; World::setOpPoint)
    Zone = 3,    ///< a zone given with it (A-GRA's ZoneTarget; an OpZone beside the command): entered (4.43)
    OpZone = 4,  ///< an operational zone by its id (OpZoneID; World::setOpZone): entered
    Line = 5,    ///< a corridor given with it (A-GRA's LineTarget; an OpLine beside the command): flown through (4.44)
    OpLine = 6,  ///< an operational line by its id (OpLineID; World::setOpLine): flown through
    Volume = 7,  ///< a volume given with it (A-GRA's VolumeTarget; an OpVolume beside the command): entered (4.45)
    OpVolume = 8, ///< an operational volume by its id (OpVolumeID; World::setOpVolume): entered
    Count
};

/// fsim.guidance.must_fly (A-GRA's MUST_FLY; docs/flight-autonomy.md, 4.42):
/// a location the aircraft must fly over - a point at its altitude, another
/// vehicle as it moves, an operational point - approached from within a
/// window of bearings where one is given (A-GRA's IngressConstraint: the
/// bearing from the location to the aircraft, from the least clockwise to the
/// most). Flown as a route the host lays out from where the aircraft is: the
/// points it approaches through, then the location, flown over. Completes as
/// the location is passed, and flies on along its course there. A field left
/// out (kHold) takes its default in a NEW and keeps its value in an UPDATE.
struct MustFlyCommand {
    double location = kHold;           ///< MustFlyLocation; kHold: a point
    double latitudeRad = kHold, longitudeRad = kHold; ///< a point's
    double altitudeM = kHold;          ///< a point's, or the altitude it flies over the location at; kHold: 4.42
    double altitudeReference = kHold;  ///< AltitudeReference
    double target = kHold;             ///< an entity's vehicle id, or an operational point's, zone's, line's or volume's id
    double ingressMinRad = kHold, ingressMaxRad = kHold; ///< the bearings from the location it approaches from: both or neither
    double speed = kHold;              ///< m/s, or a Mach number; kHold: as it flies now (a rotorcraft its cruise)
    double speedReference = kHold;     ///< SpeedReference
};

/// fsim.guidance.marshall (A-GRA's ALTITUDE_STACKED_MARSHALL, its MA_AltitudeStackedMarshallType; docs/flight-autonomy.md,
/// 4.46): a loiter - an orbit, a racetrack or a figure-eight (A-GRA's orbit: a racetrack's or a figure-eight's second circle,
/// and its laps, in the pattern's shape beside it) or a rotorcraft's hover - flown by each aircraft of a stack at an altitude
/// of its own: the lowest from `altitudeMinM`, in steps of `separationM`, at least that apart from every other aircraft
/// marshalling round the same point, and no higher than `altitudeMaxM`. The world chooses it at the NEW (or an UPDATE that
/// moves the stack), and it reads back in `altitudeM`; flown as the pattern at it. A field left out (kHold) takes its default
/// in a NEW and keeps its value in an UPDATE. Not one of the command variant's: its activity flies its pattern, a
/// PatternCommand at its slot, and keeps its stack beside it (MarshallStack) - World::submit and World::update take it.
struct MarshallCommand {
    double pattern = kHold;            ///< PatternKind: an orbit (left out), a racetrack, a figure-eight or a hover (no hold: ATC's)
    double latitudeRad = kHold, longitudeRad = kHold; ///< its centre (a racetrack's or a figure-eight's first circle's), a hover's point
    double altitudeM = kHold;          ///< its slot, as the world chose it; given in a NEW, the slot it asks for
    double altitudeReference = kHold;  ///< AltitudeReference of its altitudes
    double radiusM = kHold;            ///< its (first) circle's (A-GRA's Radius)
    double clockwise = kHold;          ///< 1 right turns, 0 left (A-GRA's TurnDirection)
    double speed = kHold;              ///< m/s, or a Mach number
    double speedReference = kHold;     ///< SpeedReference
    double durationS = kHold;          ///< A-GRA's Duration by time (by laps: the shape's orbits), a hover's from its arrival; kHold: until canceled
    double altitudeMinM = kHold;       ///< A-GRA's MinimumAltitude: the stack's lowest (given in a NEW)
    double altitudeMaxM = kHold;       ///< MaximumAltitude: its highest (left out: none)
    double separationM = kHold;        ///< MinimumAltitudeSeparation (left out: 1,000 ft)
};
/// A marshall's separation left out: 1,000 ft (docs/flight-autonomy.md, 4.46).
inline constexpr double kMarshallSeparationM = 304.8;

/// A marshall's stack (docs/flight-autonomy.md, 4.46), kept beside the pattern its activity flies - its slot that pattern's
/// altitude - as its MarshallCommand gave it: a field left out, its default.
struct MarshallStack {
    double altitudeMinM = kHold, altitudeMaxM = kHold, separationM = kHold;
};

/// How a route intercept chooses where it joins its plan's route (A-GRA's MA_RouteInterceptEnum; docs/flight-autonomy.md, 4.47).
/// Left out: at the plan's beginning.
enum class InterceptMethod : std::uint8_t {
    Discrete,         ///< DISCRETE: at an end point alone - the one nearest the aircraft
    ShortestDistance, ///< SHORTEST_DISTANCE: the point of the route nearest the aircraft
    Soonest,          ///< SOONEST: the point of the route the aircraft reaches first, turning from its track
    Count
};

/// fsim.guidance.intercept (A-GRA's ROUTE_INTERCEPT, its MA_RoutePlanInterceptType; docs/flight-autonomy.md, 4.47): a route plan
/// the vehicle keeps (4.39), joined where `method` chooses - between its `earliest` and `latest` segments, on `path` - and flown
/// from there as the plan's activation flies it, the plan activated by it. Not one of the command variant's (4.46): its activity
/// flies the plan's route, a RouteCommand, and keeps the intercept beside it. A segment is the leg into its end point, named by
/// that point's index as given (4.36). A field left out is kHold.
struct InterceptCommand {
    double plan = kHold;     ///< the plan's id (A-GRA's RoutePlanID): a whole number up to 2^53
    double path = kHold;     ///< the path it joins, its id (A-GRA's PathID; RoutePath::id); left out, any
    double method = kHold;   ///< InterceptMethod; left out, the plan's beginning
    double earliest = kHold; ///< the first segment it may join, by its end point's index (A-GRA's EarliestInterceptPoint)
    double latest = kHold;   ///< the last (A-GRA's LatestInterceptPoint)
};

/// Where an intercept joined its plan's route (4.47), kept beside it: the segment's end point and - where it joined the leg into
/// it short of that point - the point laid into the route for the join (its index the plan's count) and its place.
struct InterceptJoin {
    std::int32_t point = -1;
    std::int32_t laid = -1;
    double latitudeRad = kHold, longitudeRad = kHold;
};

/// An operational zone's shape (A-GRA's AreaChoiceType; docs/flight-autonomy.md, 4.43).
enum class ZoneShape : std::uint8_t {
    Polygon = 0,    ///< its vertices, and holes inside it (A-GRA's PolygonType)
    Ellipse = 1,    ///< round its centre: its semi-axes, the major's bearing (LocatedEllipseType)
    Rectangle = 2,  ///< round its centre: its width across and height along its bearing (LocatedRectangleType)
    SlantRange = 3, ///< from its point: between two ranges, within two bearings (SlantRangeAreaType)
    Count
};

/// An operational zone's id (A-GRA's OpZoneID): not 0.
using OpZoneId = std::uint64_t;

/// A vertex of a zone's polygon: on the Earth, or - the zone in a frame - along the frame's x and y.
struct ZoneVertex {
    double latitudeRad = kHold, longitudeRad = kHold;
    double xM = kHold, yM = kHold;
};

/// An operational zone (A-GRA's OpZone, its ZoneType; docs/flight-autonomy.md, 4.43), kept by the world by its id for a
/// must fly to name (World::setOpZone), or given with one: its shape - on the Earth, or in a reference frame - the band of
/// altitudes it spans, and the velocity it moves at from a time. A field left out is kHold.
struct OpZone {
    OpZoneId id = 0;
    double shape = kHold;                        ///< ZoneShape
    std::vector<ZoneVertex> vertices;            ///< a polygon's: 3 to 32, in order - a simple polygon
    std::vector<std::vector<ZoneVertex>> holes;  ///< its interior polygons: 4 at most, 3 to 32 vertices each, inside it
    double latitudeRad = kHold, longitudeRad = kHold; ///< the others' centre (a slant range area's point), on the Earth
    double xM = kHold, yM = kHold;               ///< or in the frame
    double semiMajorM = kHold, semiMinorM = kHold; ///< an ellipse's
    double widthM = kHold, heightM = kHold;      ///< a rectangle's: across and along its bearing
    double rangeMinM = kHold, rangeMaxM = kHold; ///< a slant range area's, over the ground (the least left out: 0)
    double azimuthMinRad = kHold, azimuthMaxRad = kHold; ///< its bearings from its point, the least clockwise to the most
    double orientationRad = kHold;               ///< an ellipse's major axis's or a rectangle's height's bearing, within a quarter turn;
                                                 ///< a slant range area's bearings turned by it (left out: 0)
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< its band: left out, from the surface, and with no top
    double altitudeReference = kHold;            ///< AltitudeReference of the band
    double frame = kHold;                        ///< its points in this frame (World::createFrame's id)
    double frameRotation = kHold;                ///< FrameRotation: its x and y turned with the frame's yaw or track (2D)
    double northMs = kHold, eastMs = kHold;      ///< moving (A-GRA's Velocity): from where it is at `timeS`, at this velocity
    double timeS = kHold;                        ///< World::time; left out, when it is set
    std::uint32_t revision = 0;                  ///< read back: one more each time it is set
};

/// An operational line's id (A-GRA's OpLineID): not 0.
using OpLineId = std::uint64_t;

/// A vertex of a line (A-GRA's LinePoint2D_Type; docs/flight-autonomy.md, 4.44): on the Earth, or - the line in a frame -
/// along the frame's x and y; the altitude to fly it at and the band there, in their reference; the widths of the
/// segment from it to the next, in place of the line's.
struct LineVertex {
    double latitudeRad = kHold, longitudeRad = kHold;
    double xM = kHold, yM = kHold;
    double altitudeM = kHold;                          ///< A-GRA's Altitude
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< A-GRA's AltitudeRange: in place of the line's band here
    double altitudeReference = kHold;                  ///< AltitudeReference of both
    double leftWidthM = kHold, rightWidthM = kHold;    ///< the segment from it: left and right of it as it is flown
};

/// An operational line (A-GRA's OpLine, its LineType; docs/flight-autonomy.md, 4.44), kept by the world by its id for a must
/// fly to name (World::setOpLine), or given with one as a corridor to fly through: its vertices in order - on the Earth, or
/// in a reference frame - the projection its segments are drawn on, the widths either side of it, the band of altitudes
/// it spans, and the velocity it moves at from a time. A field left out is kHold.
struct OpLine {
    OpLineId id = 0;
    std::vector<LineVertex> vertices;            ///< 2 to 32, in the order it is flown
    double projection = kHold;                   ///< Projection of its segments: great circles (left out) or rhumb lines
    double leftWidthM = kHold, rightWidthM = kHold; ///< its widths, left and right of it as it is flown (left out: none)
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< its band: a vertex's range in place of it there
    double altitudeReference = kHold;            ///< AltitudeReference of the band
    double frame = kHold;                        ///< its vertices in this frame (World::createFrame's id)
    double frameRotation = kHold;                ///< FrameRotation: its x and y turned with the frame's yaw or track (2D)
    double northMs = kHold, eastMs = kHold;      ///< moving (A-GRA's Velocity): from where it is at `timeS`, at this velocity
    double timeS = kHold;                        ///< World::time; left out, when it is set
    std::uint32_t revision = 0;                  ///< read back: one more each time it is set
};

/// An operational volume's shape (A-GRA's Shape3D_ChoiceType and GeocentricVolumeType; docs/flight-autonomy.md, 4.45): those of
/// the atmosphere. A-GRA's orbital volumes - its ArcVolume and IncRaPeriodVolume, orbital kinematics, orbit regimes - are none of
/// an aircraft's.
enum class VolumeShape : std::uint8_t {
    Sphere = 0,          ///< round its point: its radius
    Dome = 1,            ///< the half of a sphere above its point's level: its radius
    Ellipsoid = 2,       ///< round its point: its semi-axes along its x, y and z
    Cylinder = 3,        ///< from its point along its x: its radius, and its length (left out: without end)
    Cone = 4,            ///< its vertex its point, its axis its x: its half angle, and its range (left out: without end)
    RectangularCone = 5, ///< likewise: its half angles in its x-y plane (its length's) and its x-z plane (its width's)
    Geocentric = 6,      ///< between two latitudes, two longitudes and two altitudes
    Count
};

/// An operational volume's id (A-GRA's OpVolumeID): not 0.
using OpVolumeId = std::uint64_t;

/// An operational volume (A-GRA's OpVolume, its OpVolumeType; docs/flight-autonomy.md, 4.45), kept by the world by its id for a
/// must fly to name (World::setOpVolume), or given with one: its shape at its point - on the Earth, or in a reference frame -
/// turned by its attitude, and the velocity it moves at from a time; or, geocentric, its bounds. A field left out is kHold.
struct OpVolume {
    OpVolumeId id = 0;
    double shape = kHold;                              ///< VolumeShape
    double latitudeRad = kHold, longitudeRad = kHold;  ///< its point, on the Earth
    double xM = kHold, yM = kHold;                     ///< or in the frame
    double altitudeM = kHold;                          ///< its point's altitude
    double altitudeReference = kHold;                  ///< AltitudeReference of its point's altitude, or of a geocentric volume's band
    double radiusM = kHold;                            ///< a sphere's, a dome's or a cylinder's
    double semiAxisAM = kHold, semiAxisBM = kHold, semiAxisCM = kHold; ///< an ellipsoid's, along its x, y and z
    double lengthM = kHold;                            ///< a cylinder's (left out: without end)
    double halfAngleRad = kHold;                       ///< a cone's, from its axis
    double lengthHalfAngleRad = kHold, widthHalfAngleRad = kHold; ///< a rectangular cone's
    double rangeM = kHold;                             ///< a cone's or a rectangular cone's, from its vertex (left out: without end)
    double yawRad = kHold, pitchRad = kHold, rollRad = kHold; ///< its axes from north-east-down at its point (in a frame, from the
                                                              ///< frame's turned axes): an ellipsoid's, a cylinder's or a cone's
    double latitudeMinRad = kHold, latitudeMaxRad = kHold;   ///< a geocentric volume's
    double longitudeMinRad = kHold, longitudeMaxRad = kHold; ///< from the least clockwise to the most
    double altitudeMinM = kHold, altitudeMaxM = kHold;       ///< its band: left out, from the surface, and with no top
    double frame = kHold;                              ///< its point in this frame (World::createFrame's id)
    double frameRotation = kHold;                      ///< FrameRotation: its x and y turned with the frame's yaw or track
    double northMs = kHold, eastMs = kHold, downMs = kHold; ///< moving (A-GRA's Velocity): from where it is at `timeS` (down left out: 0)
    double timeS = kHold;                              ///< World::time; left out, when it is set
    std::uint32_t revision = 0;                        ///< read back: one more each time it is set
};

/// A must fly's zone or corridor as the host laid it out (docs/flight-autonomy.md, 4.43 and 4.44), for the behaviour to find
/// the aircraft in at every update and for an UPDATE, a waiting start or a task to fly again: its shape in the plane at its
/// reference point - north and east metres, or x and y along its frame's axes - the band of altitudes a zone spans, a
/// corridor's altitudes, bands and widths at each vertex, and how its reference moves.
struct MustFlyArea {
    static constexpr std::size_t kVertices = 32, kHoles = 4;
    ZoneShape shape = ZoneShape::Count; ///< Count: none (a point's must fly)
    std::uint8_t vertexCount = 0, holeCount = 0;
    std::uint8_t holeSizes[kHoles] = {};
    double vertices[kVertices][2] = {};          ///< a polygon's, from its reference
    double holes[kHoles][kVertices][2] = {};     ///< its holes'
    double semiMajorM = 0.0, semiMinorM = 0.0;   ///< an ellipse's
    double widthM = 0.0, heightM = 0.0;          ///< a rectangle's: across and along its bearing
    double rangeMinM = 0.0, rangeMaxM = 0.0;     ///< a slant range area's
    double azimuthMinRad = 0.0, azimuthMaxRad = 0.0; ///< its bearings, the least clockwise to the most
    double orientationRad = 0.0;                 ///< an ellipse's major axis's or a rectangle's height's bearing; a slant range's turn
    double latitudeRad = 0.0, longitudeRad = 0.0; ///< its reference, where it is at `timeS` (not in a frame)
    double northMs = 0.0, eastMs = 0.0, timeS = 0.0; ///< its velocity from then (World::time)
    bool framed = false;                         ///< in the frame `frameId`: its reference `frameXM`, `frameYM` from the origin
    FrameId frameId = 0;
    FrameSpec frame{};
    FrameRotation rotation = FrameRotation::Unrotated;
    double frameXM = 0.0, frameYM = 0.0;
    double altitudeMinM = kHold, altitudeMaxM = kHold; ///< its band, in `altitudeReference`: left out, none that way
    AltitudeReference altitudeReference = AltitudeReference::Msl;
    // a corridor's (4.44; `shape` Count): its vertices in `vertices`, and at each the altitude to fly, the band there and the
    // widths of the segment from it (kHold: none given)
    std::uint8_t lineCount = 0;                  ///< its vertices: 0, none
    Projection projection = Projection::GreatCircle;
    double lineAltitudeM[kVertices] = {};
    AltitudeReference lineAltitudeReference[kVertices] = {};
    double lineMinM[kVertices] = {}, lineMaxM[kVertices] = {};
    AltitudeReference lineBandReference[kVertices] = {};
    double leftWidthM[kVertices] = {}, rightWidthM[kVertices] = {};
    // a volume's (4.45; `shape` Count, `lineCount` 0): its shape about its point - its reference, or in its frame at `frameXM`,
    // `frameYM` - at `pointAltitudeM` (in `altitudeReference`, moving at `downMs`), its axes `axes` in north-east-down (in a
    // frame, along the frame's turned axes and down); a geocentric volume's bounds, its band `altitudeMinM` to `altitudeMaxM`
    VolumeShape volume = VolumeShape::Count;     ///< Count: none
    double pointAltitudeM = 0.0, downMs = 0.0;
    double radiusM = 0.0, semiAxesM[3] = {}, lengthM = 0.0, halfAngleRad = 0.0, lengthHalfAngleRad = 0.0, widthHalfAngleRad = 0.0;
    double rangeM = 0.0;                         ///< a cone's (infinite: without end), as a cylinder's length
    double axes[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; ///< its x, y and z (rows)
    double latitudeMinRad = 0.0, latitudeMaxRad = 0.0, longitudeMinRad = 0.0, longitudeMaxRad = 0.0;

    /// A zone, a corridor or a volume laid out (false: a point's must fly, or none).
    bool laidOut() const noexcept { return shape != ZoneShape::Count || lineCount != 0 || volume != VolumeShape::Count; }
};

/// Where a vehicle's route, curve or pattern shape lives while it is flown
/// (docs/vehicle-interface.md, 4.2): allocated at its first and kept, written
/// by the host between steps, read by the mode's behaviour during them
/// (ControlContext::path).
struct PathStore {
    static constexpr std::size_t kWaypoints = 256;
    static constexpr std::size_t kSegments = 32;
    std::uint32_t revision = 0; ///< bumped on every write
    std::uint32_t count = 0;    ///< waypoints
    Waypoint waypoints[kWaypoints];
    std::uint32_t curve = 0;    ///< bumped when a curve is replaced (not appended to): it is flown afresh
    std::uint32_t segmentCount = 0;
    NurbsSegment segments[kSegments]; ///< a curve's: a Bezier's as NurbsSegment::of makes it
    PatternShape pattern;       ///< the shape of the pattern that flies (empty: its PatternCommand alone)
    FrameSpec patternFrame;     ///< its frame, where its point is one's (PatternShape::frame)
    CurveShape curveShape;      ///< the curve's reference in a frame (empty: its CurveCommand's; docs/flight-autonomy.md, 4.27)
    FrameSpec curveFrame;       ///< its frame, where its reference is one's
    /// The frames a route's waypoints are in (docs/flight-autonomy.md, 4.29), as they were at its NEW: 16 a route at most.
    static constexpr std::size_t kRouteFrames = 16;
    std::uint32_t routeFrameCount = 0;
    FrameId routeFrameIds[kRouteFrames] = {};
    FrameSpec routeFrames[kRouteFrames];
    /// The loiters its loiter points fly (4.31), as the host completed them: 16 a route at most.
    static constexpr std::size_t kRouteLoiters = 16;
    std::uint32_t routeLoiterCount = 0;
    RouteLoiter routeLoiters[kRouteLoiters];
    /// Its planned inertial states (4.34), as the host placed them (one in a frame where the frame is at its time): 64 a route at most.
    static constexpr std::size_t kRouteStates = 64;
    std::uint32_t routeStateCount = 0;
    RouteState routeStates[kRouteStates];
    /// Its paths as given (4.36): 16 a route at most. A linked route - paths, or a point's next - flown in its flight
    /// order: `routeOrder` the waypoints' indices in it, the `routeFlown` it flies first; after its last, on from
    /// `routeLoop` where it repeats (`routeRepeats`).
    static constexpr std::size_t kRoutePaths = 16;
    std::uint32_t routePathCount = 0;
    RoutePath routePaths[kRoutePaths];
    bool routeLinked = false, routeRepeats = false;
    std::uint32_t routeFlown = 0, routeLoop = 0;
    std::uint16_t routeOrder[kWaypoints] = {};
    /// Its conditional branches as given (4.37): 16 a route at most. `routeCommanded` has bit k while the operator has
    /// branch k commanded (World::commandBranch): written apart from `revision`, the flight going on.
    static constexpr std::size_t kRouteBranches = 16;
    std::uint32_t routeBranchCount = 0;
    RouteBranch routeBranches[kRouteBranches];
    std::uint32_t routeCommanded = 0;
    /// Its civil path terminators' data as given (4.38): 64 a route at most.
    static constexpr std::size_t kRouteTerminators = 64;
    std::uint32_t routeTerminatorCount = 0;
    RouteTerminator routeTerminators[kRouteTerminators];
    /// A must fly's area (docs/flight-autonomy.md, 4.43): entered, it completes. Its shape Count: none.
    MustFlyArea mustFlyArea;
    /// The marshall that flies (4.46): its stack, beside the pattern it flies at its slot.
    MarshallStack marshall;
    /// The route intercept that flies (4.47): as given, and where it joined the route beside it.
    InterceptCommand intercept;
    InterceptJoin interceptJoin;
};

/// A registered behaviour with its parameters (design 9.3 "Behavior").
struct BehaviorCommand {
    std::string id;                        ///< registry id: "hold", "waypoints", "loiter", "pursuit", ...
    std::uint32_t target = 0;              ///< another vehicle's id when the behaviour needs one
    std::map<std::string, double> params;  ///< behaviour-specific numbers (documented per behaviour)
    std::vector<PositionCommand> points;   ///< route for "waypoints"

    double param(const std::string& key, double fallback) const noexcept {
        const auto it = params.find(key);
        return it == params.end() ? fallback : it->second;
    }
};

/// A command: a level's, a behaviour's, or a guidance mode's setpoint. The
/// modes come after BehaviorCommand and enter at Level::Behavior (levelOf):
/// the variant's index is a level's only up to it.
using Command = std::variant<ActuatorCommand, AttitudeCommand, AccelerationCommand, VelocityCommand, PositionCommand, BehaviorCommand, HsaCommand,
                             RouteCommand, PatternCommand, CurveCommand, MustFlyCommand>;
static_assert(sizeof(Command) == 120, "a mode joins the command variant no larger than its largest (docs/flight-autonomy.md, 4.23)");
static_assert(std::variant_size_v<Command> <= 11,
              "libstdc++ visits a variant of eleven alternatives or fewer through a switch, and one of more through a table of calls - "
              "every copy of a command then slower: a mode past them flies as one of them, its own fields beside it (4.46)");

// Support effectors (docs/control-architecture.md, 8.2): set directly, not
// flown through the cascade; each its own capability (fsim.support.*) where
// the aircraft has the effector.

/// fsim.support.gear: completes when the gear is where it was told.
struct GearCommand {
    double down = 1.0; ///< 1 down, 0 up
};
/// fsim.support.flaps: completes when the flaps are where they were told.
struct FlapsCommand {
    double position = 0.0; ///< 0 up .. 1 full
};
/// fsim.support.wheel_brakes
struct WheelBrakesCommand {
    double left = 0.0, right = 0.0; ///< 0 .. 1
};
/// fsim.support.speedbrake
struct SpeedbrakeCommand {
    double position = 0.0; ///< 0 in .. 1 out
};
/// fsim.support.pitch_trim
struct PitchTrimCommand {
    double position = 0.0; ///< -1 .. 1, + nose down (JSBSim's sign)
};
/// fsim.flight.engines: a throttle per engine. A flight capability that owns
/// thrust, set directly like the support effectors (so beside the cascade,
/// which then flies roll and pitch without it).
struct EnginesCommand {
    double throttle[4] = {kHold, kHold, kHold, kHold}; ///< 0 .. 1 per engine; kHold keeps it
};

/// The commands set directly beside the cascade: the support effectors and per-engine throttles.
using SupportCommand = std::variant<GearCommand, FlapsCommand, WheelBrakesCommand, SpeedbrakeCommand, PitchTrimCommand, EnginesCommand>;

/// One command of a batch NEW (World::submitBatch; A-GRA's several command
/// instances in one message, each answered on its own; docs/flight-autonomy.md, 4.8):
/// a flight or guidance command - a route's waypoints or a curve's segments
/// beside it - or a support effector's, with its options.
struct BatchCommand {
    std::variant<Command, SupportCommand> command;
    Span<const Waypoint> waypoints;     ///< a RouteCommand's
    Span<const BezierSegment> segments; ///< a CurveCommand's
    CommandOptions options;
    Span<const NurbsSegment> nurbs;     ///< a CurveCommand's as A-GRA's schema gives them (instead of `segments`)
    const PatternShape* shape = nullptr; ///< a PatternCommand's or `marshall`'s (null: none)
    const CurveShape* curveShape = nullptr; ///< a CurveCommand's reference in a frame (null: none)
    Span<const RouteLoiter> loiters;     ///< a RouteCommand's: the loiters its loiter points fly (docs/flight-autonomy.md, 4.31)
    Span<const RouteState> states;       ///< a RouteCommand's: its planned inertial states (4.34)
    Span<const RoutePath> paths;         ///< a RouteCommand's: its paths (4.36)
    Span<const RouteBranch> branches;    ///< a RouteCommand's: its conditional branches (4.37)
    Span<const RouteTerminator> terminators; ///< a RouteCommand's: its civil path terminators' data (4.38)
    const OpZone* zone = nullptr;        ///< a MustFlyCommand's zone given with it (4.43; null: none)
    const OpLine* line = nullptr;        ///< a MustFlyCommand's corridor given with it (4.44; null: none)
    const OpVolume* volume = nullptr;    ///< a MustFlyCommand's volume given with it (4.45; null: none)
    const MarshallCommand* marshall = nullptr; ///< a marshall (4.46), in place of `command`, its pattern's shape `shape` (null: none)
    const InterceptCommand* intercept = nullptr; ///< a route intercept (4.47), in place of `command` (null: none)
};

/// What a live activity flies now, or waits to fly (A-GRA's last flight
/// command, in the activity report; docs/flight-autonomy.md, 4.12): its
/// setpoint as updated - a mode's merged, a level's, a support effector's -
/// and a route's waypoints or a curve's segments, appended ones too (a
/// curve's: A-GRA's FlyoutCurve, from the CurveCommand's reference).
struct Setpoint {
    std::variant<Command, SupportCommand> command;
    std::vector<Waypoint> waypoints;
    std::vector<BezierSegment> segments; ///< a curve's, where each is a Bezier segment's form
    PatternShape shape; ///< a pattern's, as it flies (docs/flight-autonomy.md, 4.23)
    std::vector<NurbsSegment> nurbs;     ///< a curve's, every segment, as it flies (docs/flight-autonomy.md, 4.26)
    CurveShape curveShape;               ///< a curve's reference in a frame, as it flies (4.27)
    std::vector<RouteLoiter> loiters;    ///< a route's loiters, as they fly (4.31)
    std::vector<RouteState> states;      ///< a route's planned inertial states, as placed (4.34)
    std::vector<RoutePath> paths;        ///< a route's paths (4.36)
    std::vector<RouteBranch> branches;   ///< a route's conditional branches (4.37)
    std::vector<RouteTerminator> terminators; ///< a route's civil path terminators' data (4.38)
    /// A marshall's (4.46): as it flies - `command` its pattern at its slot, `shape` its shape - with its stack; none for
    /// any other activity.
    std::optional<MarshallCommand> marshall;
    /// A route intercept's (4.47): as given - `command` and `waypoints` the plan's route it flies, the point laid in for its
    /// join among them - and where it joined; none for any other activity.
    std::optional<InterceptCommand> intercept;
    InterceptJoin join;
};

/// Where an activity flies to (A-GRA's ActualEndPoint, MA_EndPointType;
/// docs/flight-autonomy.md, 4.12): a point, a turn flown by or over it
/// (TurnType), or a loiter.
enum class EndPointKind : std::uint8_t { Waypoint, TurnPoint, LoiterPoint, Count };
struct EndPoint {
    EndPointKind kind = EndPointKind::Waypoint;
    double latitudeRad = kHold, longitudeRad = kHold;
    double altitudeM = kHold;          ///< in `altitudeReference` (a waiting route's as given: kHold continues the point before)
    double altitudeReference = kHold;  ///< AltitudeReference
    double turn = kHold;               ///< a turn point's TurnType
    std::uint64_t id = 0;              ///< a route waypoint's id; 0 none
    std::int32_t index = -1;           ///< its waypoint or curve segment; 0 a pattern's or the position level's point
};

inline Level levelOf(const Command& c) noexcept {
    return c.index() < static_cast<std::size_t>(Level::Behavior) ? static_cast<Level>(c.index()) : Level::Behavior;
}
/// The registered behaviour that flies a mode's setpoint ("hsa", "route", "pattern", "curve", "must_fly"); null for a level's
/// or a behaviour's command.
inline const char* modeBehavior(const Command& c) noexcept {
    if (std::holds_alternative<HsaCommand>(c)) return "hsa";
    if (std::holds_alternative<RouteCommand>(c)) return "route";
    if (std::holds_alternative<PatternCommand>(c)) return "pattern";
    if (std::holds_alternative<CurveCommand>(c)) return "curve";
    if (std::holds_alternative<MustFlyCommand>(c)) return "must_fly";
    return nullptr;
}

/// An airfield's id (A-GRA's AirfieldID) and a runway's (RunwayID): not 0.
using AirfieldId = std::uint64_t;
using RunwayId = std::uint64_t;

/// A place on a runway (A-GRA's Point3D_Type; docs/flight-autonomy.md, 4.40): above the WGS-84 ellipsoid unless its
/// reference says otherwise. Its latitude left out, none.
struct RunwayPoint {
    double latitudeRad = kHold, longitudeRad = kHold;
    double altitudeM = kHold;
    double altitudeReference = kHold; ///< AltitudeReference; left out, above the ellipsoid (A-GRA's Point3D's)
};

/// A runway's takeoff or landing coordinates (A-GRA's RunwayCoordinatesType): its start - required where any of them is
/// given - and its threshold and limit, its nearest and furthest points.
struct RunwayCoordinates {
    RunwayPoint start, threshold, limit;
};

/// A runway (A-GRA's AirfieldRunwayType): its takeoff coordinates or its landing coordinates, or both.
struct Runway {
    RunwayId id = 0;
    double directionRad = kHold;        ///< from true north, 0 to 2 pi (A-GRA's Direction)
    double availableLengthM = kHold;    ///< above 0 (A-GRA's AvailableLength)
    RunwayCoordinates takeoff, landing; ///< (A-GRA's TakeoffCoordinates, LandingCoordinates)
};

/// An airfield (A-GRA's AirfieldReportMDT; docs/flight-autonomy.md, 4.40): FA's, loaded by the platform before a mission and
/// read only to MA - its runways, their takeoff and landing coordinates and limits, and its QNH.
struct Airfield {
    static constexpr std::size_t kRunways = 16;
    AirfieldId id = 0;
    std::string icao;             ///< A-GRA's ICAO_Code: four capitals, or none
    double qnhPa = kHold;         ///< A-GRA's QNH_Setting: 850 to 1,100 hPa
    std::vector<Runway> runways;  ///< kRunways at most
    std::uint32_t revision = 0;   ///< its loads kept, read back (ignored as loaded)
};

/// Who made a route plan's point (A-GRA's PathSegmentSourceEnum).
enum class PointSource : std::uint8_t { AutoRouted, OperatorDefined, Count };
/// "auto_routed", "operator_defined".
FSIM_API const char* pointSourceName(PointSource source) noexcept;

/// A route plan's point's planning metadata (A-GRA's MA_PathSegmentType's Source, Locked, Modified, Remarks and
/// Fix_Identifier; docs/flight-autonomy.md, 4.39): kept with its plan and read back; nothing flies by it. Its texts are
/// printable ASCII (A-GRA's VisibleString), no longer than A-GRA's.
struct PointMetadata {
    std::uint32_t point = 0;       ///< the waypoint's index
    PointSource source = PointSource::AutoRouted;
    bool locked = false;           ///< not to be modified: operator-driven
    bool modified = false;         ///< modified in a modified route plan
    std::string remarksName;       ///< its Remarks' DisplayName: 32 characters at most
    std::string remarks;           ///< its Remarks' Detail: 1,024 at most
    std::string fixKey, fixSystem; ///< its Fix_Identifier's Key and SystemName: 256 each at most
};

/// A route plan's path's planning metadata (A-GRA's MA_RoutePathType.InitialConditions, a PlanningLocationType;
/// docs/flight-autonomy.md, 4.39): the aircraft's state as planned or assessed where the path begins - kept with its
/// plan and read back; nothing flies by it.
struct PathMetadata {
    std::uint32_t path = 0;       ///< the path's index among the route's paths (4.36); 0 without paths: the route's one
    RouteState initial{};         ///< its InertialState, as a planned state's fields (4.34): its `point` not used
    double enduranceS = kHold;    ///< its Endurance's Duration
    double fuelKg = kHold;        ///< its Endurance's Fuel
    double grossWeightKg = kHold; ///< its GrossWeight
    PlanId transitionPlan = 0;    ///< its TransitionRoute: a route plan's id; 0 none
    /// Its airfield and runway (A-GRA's MA_RoutePathType.AirfieldID and RunwayID; 4.40): a takeoff's or a landing's path, on
    /// FA's own plan, names ones the vehicle keeps. 0: none.
    AirfieldId airfield = 0;
    RunwayId runway = 0;
};

/// A route plan (A-GRA's MA_RoutePlanMT; docs/flight-autonomy.md, 4.39): a route FA keeps by its id and version,
/// taken through the plan activation states before it flies, and its planning metadata (A-GRA's MA_RouteType's and its
/// segments' and paths'), which nothing flies by. World::publishPlan takes one; World::plan reads one back.
struct RoutePlan {
    PlanId id = 0;
    std::uint32_t version = 0;       ///< A-GRA's RoutePlanID.Version
    bool forPlanningUseOnly = false; ///< never prepared for activation, or activated (A-GRA's ForPlanningUseOnly)
    RouteCommand route{};            ///< its route's options, as World::submit takes a route's
    std::vector<Waypoint> waypoints;
    std::vector<RouteLoiter> loiters;
    std::vector<RouteState> states;
    std::vector<RoutePath> paths;
    std::vector<RouteBranch> branches;
    std::vector<RouteTerminator> terminators;
    // its planning metadata (WPT-23)
    bool detailed = false;           ///< A-GRA's MA_RouteType.Detailed
    std::string remarksName;         ///< its route's Remarks' DisplayName: 32 characters at most
    std::string remarks;             ///< its route's Remarks' Detail: 1,024 at most
    std::vector<PointMetadata> pointMetadata; ///< one a point at most
    std::vector<PathMetadata> pathMetadata;   ///< one a path at most
};

/// An operational point's id (A-GRA's OpPointID): not 0.
using OpPointId = std::uint64_t;

/// An operational point (A-GRA's OpPoint; docs/flight-autonomy.md, 4.42), kept by the world by its id (World::setOpPoint)
/// for the commands that name it (a must fly): a place on the Earth (A-GRA's Point), or in a reference frame (its
/// RelativePoint: `frame` and its offsets, as a route point's), and the window of bearings it is approached from (its
/// IngressConstraint). A field left out is kHold.
struct OpPoint {
    OpPointId id = 0;
    double latitudeRad = kHold, longitudeRad = kHold; ///< on the Earth; with a frame, left out
    double altitudeM = kHold;          ///< in `altitudeReference`; left out, none (flown at the aircraft's)
    double altitudeReference = kHold;  ///< AltitudeReference
    double frame = kHold;              ///< in this frame (World::createFrame's id): where the frame puts its offsets
    double frameRotation = kHold, frameOffsets = kHold; ///< its offsets' FrameRotation and FrameOffsets, as a route point's
    double frameXM = kHold, frameYM = kHold, frameZM = kHold; ///< its offsets (z down: given, its altitude the frame's there)
    double ingressMinRad = kHold, ingressMaxRad = kHold; ///< the bearings from it it is approached from: both or neither
    std::uint32_t revision = 0;        ///< read back: one more each time it is set
};

/// Read-only view of the world for behaviours that look at other vehicles.
/// Implemented by the session; states are the previous step's snapshots.
struct TablesSection; // fsim/VehicleProfile.h
struct Altimeter;     // fsim/Altimeter.h
struct NavigationReport;

class WorldView {
public:
    virtual ~WorldView() = default;
    virtual const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept = 0;
    virtual double simTime() const noexcept = 0;
    virtual const sim::EnvironmentState& environment() const noexcept = 0;
    /// A vehicle's navigation report now (docs/flight-autonomy.md, 4.14): what it has left, its endurance, its contingency.
    /// During a step only the vehicle whose control update asks may ask for its own - its flight model idle while its
    /// cascade runs (a route's branch on them: 4.37). False for none (a vehicle it has not; a stack on its own).
    virtual bool navigation(std::uint32_t id, NavigationReport& out) const noexcept {
        (void)id, (void)out;
        return false;
    }
};

/// What a controller sees each update (design 9.3 "ControlContext").
struct ControlContext {
    std::uint32_t vehicleId = 0;
    const sim::VehicleState& state;     ///< truth, before this FDM step
    const sim::VehicleState& sensed;    ///< through the vehicle's sensor models (equals `state` without effects)
    double dt = 0.0;                    ///< controller period, seconds
    const WorldView* world = nullptr;   ///< null when the stack runs stand-alone
    Rng* rng = nullptr;                 ///< this vehicle's stream (deterministic)
    /// The primary axes this level's command drives this update
    /// (docs/control-architecture.md, 9.5): all of them unless the vehicle's
    /// axes are owned apart. A controller that says axisAware() leaves the
    /// others alone - no integrating, no output (kHold).
    AxisMask engaged = kPrimaryAxes;
    /// What the vehicle can do (Feature bits), for a behaviour whose guidance
    /// differs by it: a rotorcraft hovers (docs/vehicle-interface.md, 4.9).
    std::uint32_t features = kFeatureWingborne;
    /// What it can do in numbers, for a behaviour: its guidance plans with it
    /// (docs/vehicle-interface.md, 7.1). Null outside a vehicle's runtime.
    const Performance* performance = nullptr;
    /// Its clean configuration's limits (the profile's envelope; NaN where it
    /// has none), for a behaviour that flies to them: a manoeuvre that gives
    /// up when it departs. Null outside a vehicle's runtime.
    const EnvelopeLimits* envelope = nullptr;
    /// The vehicle's route, for the behaviour that flies it; null until one was given.
    const PathStore* path = nullptr;
    /// Its performance tables (the profile's; docs/flight-autonomy.md, 4.13),
    /// for a mode that flies their best speeds: null without them.
    const TablesSection* tables = nullptr;
    /// Its barometric altimeter (docs/flight-autonomy.md, 4.20): the world's air
    /// and the QNH it is set to, for a mode flying a barometric altitude. Null
    /// outside a vehicle's runtime: the standard atmosphere, at 1013.25 hPa.
    const Altimeter* altimeter = nullptr;
};

/// One level of the cascade: accepts a command at `level()` and returns a
/// command at any level below it, by levelRank (design 9.3 "Controller").
class Controller {
public:
    virtual ~Controller() = default;

    virtual const char* id() const noexcept = 0;
    virtual Level level() const noexcept = 0;

    /// Translate `in` (a command at `level()`) into a lower-level command.
    virtual Command update(const ControlContext& ctx, const Command& in) = 0;

    /// Forget integrators and internal state (vehicle reset, controller swap).
    virtual void reset() {}

    /// Gains and other tunables by name; unknown names return false / nullopt.
    virtual bool setParameter(std::string_view name, double value) { (void)name; (void)value; return false; }
    virtual std::optional<double> parameter(std::string_view name) const { (void)name; return std::nullopt; }

    /// True if it honours ControlContext::engaged: then a command whose axes
    /// are owned apart may pass through it (docs/control-architecture.md, 9.6).
    virtual bool axisAware() const noexcept { return false; }
};

/// A top-level controller with a lifecycle (design 9.3 "Behavior"). A finished
/// behaviour keeps producing its last output until replaced.
class Behavior : public Controller {
public:
    Level level() const noexcept final { return Level::Behavior; }

    /// Called once with the command that selected this behaviour, before the first update().
    virtual void start(const ControlContext& ctx, const BehaviorCommand& command) { (void)ctx; (void)command; }
    /// What the runtime calls first: with a BehaviorCommand, start() above; a
    /// guidance mode (docs/vehicle-interface.md) takes its setpoint here.
    virtual void begin(const ControlContext& ctx, const Command& command) {
        if (const auto* b = std::get_if<BehaviorCommand>(&command)) start(ctx, *b);
    }
    /// The goal is reached: its activity completes (docs/control-architecture.md, 10.3).
    virtual bool finished() const noexcept { return false; }
    /// Why it can no longer do what it was asked, e.g. Reason::TargetLost once
    /// the vehicle it follows is gone; None while it can. Its activity fails,
    /// and it keeps flying whatever it falls back to.
    virtual Reason failure() const noexcept { return Reason::None; }
    /// ActivityFlag bits for its last update, beside what the runtime sees of
    /// the loops: a setpoint it held back to keep the aircraft safe
    /// (kActivityClamped: an evade's descent stopped at its floor).
    virtual std::uint16_t constraints() const noexcept { return 0; }
    /// How far it has got and what it commands, as of its last update
    /// (docs/vehicle-interface.md, 5.3): the fields it knows into `out`, and
    /// true; false if it reports nothing. Asked between world steps (the
    /// activity's record carries it), never during one.
    virtual bool progress(ActivityProgress& out) const noexcept {
        (void)out;
        return false;
    }
    /// Its next point with a required time of arrival's estimate (docs/flight-autonomy.md, 4.33), as of its last update:
    /// true where it has one. Asked between world steps.
    virtual bool arrival(ArrivalEstimate& out) const noexcept {
        (void)out;
        return false;
    }
    /// The points a route flies from here, as of its last update (docs/flight-autonomy.md, 4.37): their indices as given
    /// into `points` - the one flown to first, at most `max` - their count returned (0: it flies no route); `ends` where
    /// the last is the route's end. Asked between world steps.
    virtual std::uint32_t ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept {
        (void)points, (void)max;
        ends = false;
        return 0;
    }
    /// Where a route is along its segments, as of its last update (docs/flight-autonomy.md, 4.47): the point flown from and
    /// when it was captured, the point flown to and how far it is, the one after it. True where it flies a route. Asked
    /// between world steps.
    virtual bool segments(SegmentEstimate& out) const noexcept {
        (void)out;
        return false;
    }
    /// What FA flies in its place when a policy cancels its activity (docs/flight-autonomy.md, 4.50): a takeoff on the
    /// runway, which must not be let go, hands on its rejection below its decision speed or its continuation above - into
    /// `out`, and true; false where its axes may simply go. Asked between world steps, as the CANCEL is answered.
    virtual bool handOver(BehaviorCommand& out) const {
        (void)out;
        return false;
    }
};

/// What the cascade asked for in its last control update, level by level
/// (docs/vehicle-interface.md, 5.3; A-GRA's VehicleCommandState): each field
/// from the level that sets it where that level ran, NaN where none did.
struct CommandedState {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    Level top = Level::Actuator;                                       ///< the highest level that ran
    double latitudeRad = kNone, longitudeRad = kNone, altitudeMslM = kNone; ///< the position level's point
    double headingRad = kNone;      ///< the velocity level's, else the attitude level's
    double turnRateRadS = kNone;    ///< the velocity level's
    double airspeedMs = kNone;      ///< true: the velocity level's, else the attitude level's
    double verticalSpeedMs = kNone; ///< the velocity level's
    double northMs = kNone, eastMs = kNone; ///< the velocity level's over the ground (a rotorcraft's)
    double rollRad = kNone, pitchRad = kNone; ///< the attitude level's
    double loadFactorG = kNone, rollRateRadS = kNone, pitchRateRadS = kNone, yawRateRadS = kNone; ///< the acceleration level's
    double throttle = kNone;        ///< what the actuators were given (the first engine's)
};

/// What a vehicle is commanded (A-GRA's VehicleCommandState;
/// docs/flight-autonomy.md, 4.12): the cascade's levels, the acceleration
/// they ask of the aircraft in north, east and down, and the altitude as its
/// mode commanded it, in the reference it was commanded in.
struct VehicleCommandState : CommandedState {
    /// A wing's: the acceleration level's longitudinal acceleration along its
    /// flight path and its load factor's lift normal to the path (JSBSim's
    /// lift load factor), with gravity's pull, at the attitude, angle of
    /// attack and sideslip it flies - its acceleration over the Earth. NaN
    /// where no longitudinal acceleration is commanded (a throttle given
    /// instead), and for a rotorcraft (whose thrust, its drag unmodelled, is
    /// not its acceleration).
    double northAccelerationMs2 = kNone, eastAccelerationMs2 = kNone, downAccelerationMs2 = kNone;
    /// A live hsa's or pattern's altitude, a route's point flown to's, in
    /// their reference; a curve's, and the position level's, above sea level.
    /// NaN where none is commanded.
    double altitudeM = kNone;
    double altitudeReference = kNone; ///< AltitudeReference
};

// --- The navigation report (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport) ---

/// What a vehicle flies on.
enum class Energy : std::uint8_t {
    Unknown = 0, ///< its flight model does not say (neither fuel nor a battery it knows)
    Fuel = 1,
    Battery = 2,
};

/// A-GRA's SystemContingencyLevelEnum: the vehicle's contingency. The
/// platform reports a low fuel state (FlightCritical); it models no
/// subsystem failures (MissionCritical) and no communications (LostComms).
enum class Contingency : std::uint8_t {
    Normal = 0,
    MissionCritical = 1,
    FlightCritical = 2, ///< at or below its reserve, or an engine starved
    LostComms = 3,
};

/// Where a vehicle recovers to, and what it keeps for the end: its playtime
/// counts the return and the reserve.
struct NavigationSettings {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    bool recovery = false;               ///< a recovery point set
    double latitudeDeg = kNone, longitudeDeg = kNone, altitudeMslM = kNone;
    double reserveFraction = 0.1;        ///< of its capacity, fuel or charge, kept for the end
};

/// A-GRA's MA_NavigationReport (its Endurance, Playtime and ContingencyLevel):
/// what the vehicle flies on, how much it has and for how long. Fuel in kg
/// (a battery's charge in J), its consumption now (kg/s, W): the engines'
/// fuel flow or the power the battery gives.
struct NavigationReport {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    Energy energy = Energy::Unknown;
    double fuelKg = kNone;       ///< A-GRA's Fuel: in its tanks (0 for a battery)
    double remaining = kNone;    ///< fuel (kg) or charge (J)
    double capacity = kNone;     ///< its tanks' (kg) or battery's (J), full
    double percent = kNone;      ///< A-GRA's Percent: remaining over capacity, %
    double consumption = kNone;  ///< now: fuel flow (kg/s) or power (W)
    /// A-GRA's Duration: remaining over consumption now, s (infinite while it
    /// consumes nothing: a helicopter's engines off, a multirotor at rest; 0
    /// with nothing left).
    double enduranceS = kNone;
    double reserve = kNone;      ///< kept for the end (NavigationSettings), kg or J
    /// A-GRA's Playtime, with a recovery point: what it can spend before it
    /// turns back - remaining less the reserve and the return, over its
    /// consumption now, s (0 once past it). NaN without a recovery point.
    double playtimeS = kNone;
    double returnDistanceM = kNone; ///< to the recovery point, over the ground
    double returnTasMs = kNone;     ///< it would fly back at: its best-range speed (the tables), else its cruise
    double returnConsumption = kNone; ///< it would burn back (kg/s or W)
    Contingency contingency = Contingency::Normal;
    bool starved = false;           ///< its engines have nothing left: a fuel burner's tanks empty, or the battery spent
};

/// What A-GRA's detailed position report carries beyond the vehicle's state
/// (VehicleState; docs/flight-autonomy.md, 4.20; MA_PositionReportDetailed's
/// AirData, VI 1.2.6.8): what its barometric altimeter reads, set to its QNH,
/// and the air it reads it in.
struct StateData {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double indicatedAltitudeM = kNone;      ///< A-GRA's IndicatedBaroAltitude: what the altimeter reads (fsim/Altimeter.h)
    double indicatedAltitudeRateMs = kNone; ///< A-GRA's BarometricAltitudeRate: how fast its reading changes
    double kollsmanHpa = kNone;             ///< A-GRA's Kollsman: the QNH the altimeter is set to, hPa
    double staticPressurePa = kNone;        ///< the air's pressure where the vehicle is
    double staticTemperatureK = kNone;      ///< and its temperature
    /// A-GRA's OrientationRate and OrientationAcceleration (docs/flight-autonomy.md,
    /// 4.21): how fast its Euler angles - yaw, pitch and roll, over the local
    /// north, east and down - change, and how that changes. NaN pitched within
    /// 0.06 degrees of straight up or down, where yaw and roll are one.
    double yawRateRadS = kNone, pitchRateRadS = kNone, rollRateRadS = kNone;
    double yawAccelerationRadS2 = kNone, pitchAccelerationRadS2 = kNone, rollAccelerationRadS2 = kNone;
    /// A-GRA's WanderAngle: its navigation frame's x axis from north. The
    /// platform's is north's: 0.
    double wanderAngleRad = kNone;
    /// A-GRA's MagneticHeading (docs/flight-autonomy.md, 4.22): its heading
    /// from magnetic north, and the declination that turns it true - the World
    /// Magnetic Model's where it is, at the world's date.
    double magneticHeadingRad = kNone, declinationRad = kNone;
    /// A-GRA's wind data (VI 1.2.6.8): the air's velocity over the ground where
    /// the vehicle is, as its air data measures it (its ground velocity less its
    /// velocity through the air), north, east and down, m/s.
    double windNorthMs = kNone, windEastMs = kNone, windDownMs = kNone;
};

FSIM_API const char* energyName(Energy e) noexcept;           ///< "unknown", "fuel", "battery"
FSIM_API const char* contingencyName(Contingency c) noexcept; ///< A-GRA's: "NORMAL", "MISSION_CRITICAL", "FLIGHT_CRITICAL", "LOST_COMMS"

} // namespace fsim::control
