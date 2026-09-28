#pragma once

// The guidance modes' paths (docs/vehicle-interface.md, 4.5 to 4.8): a
// route's legs on the sphere - great circles or rhumb lines - and its fly-by
// turns, arcs in the plane at their waypoint; where the aircraft is against
// them; the plan the route's behaviour flies and the host checks a route
// against; and the path follower every mode flies its path with. The Earth
// is a sphere of the mean radius here, as for all local geometry (core/Geodesy.h).

#include "fsim/Control.h"
#include "fsim/GuidanceModes.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <limits>

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

// --- The path follower (4.8) ------------------------------------------------------

/// What comes next along the path: a loop that lags flies the curvature
/// there early (a turn begun before its arc, ended before the arc does).
struct Ahead {
    double toChangeM = std::numeric_limits<double>::infinity(); ///< along the path to where its curvature changes
    double curvature = 0.0;  ///< its curvature from there, 1/m
    double turnRadiusM = 0.0; ///< the turn being flown or coming, for a wing's roll into it; 0: none
    /// A path whose curvature changes all along it (a curve): its curvature
    /// `aheadM` metres on from the fix, in place of the two above.
    double (*curvatureAt)(const void* path, double aheadM) noexcept = nullptr;
    const void* path = nullptr;
};

/// What the follower flies at a fix.
struct Steer {
    double speed = 0.0;                     ///< the path's, in `reference`
    SpeedReference reference = SpeedReference::TrueAirspeed;
    double speedLimitMs = std::numeric_limits<double>::infinity(); ///< a rotorcraft's path speed at most: a curve's, a stop
    double verticalSpeedMs = 0.0;           ///< what the altitude profile asks
};

/// What the follower integrates.
struct Trims {
    double courseRadS = 0.0; ///< the course error's integral: a wing's as a turn rate, a rotorcraft's as a course, rad
    double speedMs = 0.0;    ///< a wing's ground-speed error integral, as airspeed
    double taken = 0.0;      ///< a rotorcraft's: the acceleration its velocity loop's own integral has taken up (as modelled), m/s2
};

/// The path at `fix` flown by the vehicle (line of sight with the curvature
/// fed forward, from its course bandwidth): a wing as a turn rate and an
/// airspeed, a rotorcraft as its velocity over the ground, the nose along the
/// track. `course` and `heading` get what it commands (a wing's heading: none).
/// `s` is the aircraft as the path's ground sees it: its sensed state, or over
/// a frame the path moves with, its velocity over the frame (and `wind` the
/// wind's through it; docs/flight-autonomy.md, 4.25).
VelocityCommand follow(const ControlContext& ctx, const sim::VehicleState& s, const Performance& performance, const WindEstimate& wind, bool hovers,
                       const Fix& fix, const Ahead& ahead, const Steer& steer, Trims& trims, double& course, double& heading) noexcept;
inline VelocityCommand follow(const ControlContext& ctx, const Performance& performance, const WindEstimate& wind, bool hovers, const Fix& fix,
                              const Ahead& ahead, const Steer& steer, Trims& trims, double& course, double& heading) noexcept {
    return follow(ctx, ctx.sensed, performance, wind, hovers, fix, ahead, steer, trims, course, heading);
}

/// The vertical speed that flies to `altitudeMslM`: `feedforward` (a
/// profile's) plus the error at the position loop's gain, within the
/// aircraft's climb and descent.
double verticalSpeedTo(double altitudeMslM, double feedforward, const sim::VehicleState& s, const Performance& performance, bool hovers) noexcept;

/// A rotorcraft's path speed at most, for its lateral acceleration and its
/// braking (80 % of its performance's) and a turn rate its velocity loop
/// follows (a third of its bandwidth): on an arc of `radiusM`, and `toArcM`
/// before one (0: none).
double lateralLimit(const Performance& performance, double radiusM) noexcept;
double brakingLimit(const Performance& performance, double speedAfterMs, double toGoM) noexcept;

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
    Trims trims; ///< the follower's, flying it
    /// The frames its points are in, as the host placed them (docs/flight-autonomy.md, 4.29): the host's check's.
    std::uint32_t frameCount = 0;
    FrameId frameIds[PathStore::kRouteFrames] = {};
    FrameSpec frames[PathStore::kRouteFrames];

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

// --- Loiter patterns (4.6) ---------------------------------------------------------

/// A straight in the plane at a pattern's point: from (north, east), m, along its course.
struct Line {
    double northM = 0.0, eastM = 0.0;
    double courseRad = 0.0;
    double lengthM = 0.0;
};
/// The aircraft at (lat, lon) against the line in the plane at (lat0, lon0), extended beyond its ends.
Fix onLine(const Line& line, double lat0, double lon0, double lat, double lon) noexcept;

/// A pattern's loop round its point, in the plane there: pieces flown in
/// order and again - a lap begins where it was joined - after the pieces
/// that enter it: a racetrack's or a hold's line direct to the fix, a line to
/// its entry point. Its exit point ends a lap's piece: there, its duration or
/// laps flown, it leaves along a line (docs/flight-autonomy.md, 4.23).
struct Pattern {
    struct Piece {
        bool arc = false;
        Line line;              ///< a straight's
        Turn turn;              ///< an arc's: its centre from the point, its radius, its way round (the sign of angleRad), where it starts
        double sweepRad = 0.0;  ///< an arc's, up to a whole circle
        double lengthM() const noexcept { return arc ? turn.radiusM * sweepRad : line.lengthM; }
    };
    /// A lap's pieces: four at most (an orbit's 1, a figure-eight's 2, a racetrack's or a hold's 4, two circles' 4),
    /// and two more where one is cut to be joined and to be left.
    static constexpr std::uint32_t kPieces = 6;
    static constexpr std::uint32_t kEntryPieces = 6;
    PatternKind kind = PatternKind::Orbit;
    double lat0 = 0.0, lon0 = 0.0; ///< the centre, or the fix (two circles': the first's centre)
    double radiusM = 0.0;          ///< its tightest turn's
    std::uint32_t count = 0;       ///< pieces in a lap
    std::uint32_t first = 0;       ///< the piece a lap begins with (a racetrack's: the turn at the fix)
    Piece pieces[kPieces];
    std::uint32_t entryCount = 0;  ///< flown once, from where the aircraft is, before the lap from `first`
    Piece entry[kEntryPieces];
    std::int32_t exit = -1;        ///< the piece at whose start it leaves, its duration or laps flown; -1: it does not
    Line away;                     ///< where it leaves: on along its course from there
    Trims trims;                   ///< the follower's, flying it

    std::uint32_t next(std::uint32_t i) const noexcept { return i + 1 < count ? i + 1 : 0; }
    double pieceM(std::uint32_t i) const noexcept { return pieces[i].arc ? pieces[i].turn.radiusM * pieces[i].sweepRad : pieces[i].line.lengthM; }
    double lapM() const noexcept;
    double entryM() const noexcept;
    /// Along the lap from its first piece's start to the exit point (0: none).
    double toExitM() const noexcept;
};

/// Its track over the ground, rad: its heading when it barely moves (a metre a second).
inline double trackOf(const sim::VehicleState& s) noexcept {
    return std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) > 1.0 ? std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) : s.eulerRad[2];
}

/// A piece's point `m` along it (north and east, m, in the pattern's plane) and its course there.
void pointOn(const Pattern::Piece& piece, double m, double& north, double& east, double& courseRad) noexcept;

/// The pattern `c` sets (complete: its fields the host resolved) with its
/// shape (docs/flight-autonomy.md, 4.23), planned from where the aircraft is:
/// an orbit's laps counted from its bearing then, two circles' from their
/// nearest point, an entry point's from it; a magnetic course turned by the
/// declination at its point in `magneticYear` (fsim/Magnetic.h). `trackRad`,
/// the aircraft's track then, starts a hold's way in onto a leg (4.24).
void planPattern(Pattern& p, const PatternCommand& c, double lat, double lon, const PatternShape& shape = {}, double magneticYear = 2025.0,
                 double trackRad = kHold) noexcept;

/// What a pattern leaves out, filled in (docs/vehicle-interface.md, 4.6): an
/// orbit, here, the altitude and speed it flies now (a rotorcraft's speed its
/// cruise over the ground), right turns, its track now (a hold's course the
/// way to its fix), the radius its speed and the wind and 80 % of its bank
/// give (a hold's: rate one, at most 25 degrees of bank), legs of twice the
/// radius (a hold's: a minute's flight, 90 s above 14,000 ft). The angles
/// wrapped. What its shape gives another way (docs/flight-autonomy.md, 4.23)
/// fills its field: a course from a heading, the legs from their time, the
/// radius from a bank - in the wind given, north and east - and two circles'
/// second radius (the shape's) from the first; the radius by a turn rate or
/// type (4.24) too. A magnetic course left out is the course now, turned by
/// the declination at its point in `magneticYear`. A hover (4.25) has its
/// point, its altitude and its speed there alone: nothing round it.
void completePattern(PatternCommand& c, PatternShape& shape, const sim::VehicleState& state, const Performance& performance, bool hovers,
                     double windNorthMs, double windEastMs, const Altimeter* altimeter = nullptr, double magneticYear = 2025.0) noexcept;

/// Whether a pattern's shape fits the pattern (docs/flight-autonomy.md, 4.23, 4.24): a second circle only a
/// racetrack's or a figure-eight's, with no course, heading, legs or leg time (the circles give them), a racetrack's
/// two circles apart (neither inside the other), a figure-eight's clear of one another, both at least a metre in
/// radius; an entry only a racetrack's or a hold's on its fix (no second circle, no entry point); a context only a
/// hold's; a frame's offsets only with their frame. Its field at fault (its index in the fields of the PatternCommand
/// and then its shape), or -1. `c` complete.
int shapeFault(const PatternCommand& c, const PatternShape& shape) noexcept;
/// A hover's field at fault (docs/flight-autonomy.md, 4.25): one given that shapes a circuit - a radius or a way to
/// give it, a way round, a course or heading, legs, laps, a second circle, an entry or exit point, a hold's entry or
/// context. A hover has its point (or its frame's), its altitude, its speed there and its duration alone. -1 if none.
int hoverFault(const PatternCommand& c, const PatternShape& shape) noexcept;

// --- Curves (4.7) -----------------------------------------------------------------

/// A curve segment's point at a parameter (0 to 1 along it), and its first
/// and second derivatives by the parameter: north, east, down, m.
struct CurvePoint {
    double p[3] = {0.0, 0.0, 0.0};
    double d1[3] = {0.0, 0.0, 0.0};
    double d2[3] = {0.0, 0.0, 0.0};
    double courseRad() const noexcept;  ///< of its tangent over the ground
    double curvature() const noexcept;  ///< over the ground, 1/m, + turning right
    double gradient() const noexcept;   ///< its climb per metre over the ground
};
CurvePoint evaluate(const BezierSegment& s, double t) noexcept;
/// A segment as A-GRA's schema gives it (docs/flight-autonomy.md, 4.26): a Bezier's form by Bernstein's basis, as a
/// BezierSegment is; any other clamped rational B-spline by its basis (rational(), Nurbs.cpp). Its parameter 0 to 1 runs
/// over its knots' domain.
CurvePoint evaluate(const NurbsSegment& s, double t) noexcept;
/// A clamped rational B-spline's point (not a Bezier's form: evaluate); `s` well formed.
CurvePoint rational(const NurbsSegment& s, double t) noexcept;
/// Its counts, knots and weights make a clamped curve (docs/flight-autonomy.md, 4.26): 4 to 10 points and 4 to 14
/// knots, a degree of 1 or more, all finite; knots from 0 and never decreasing, the first and the last each as many
/// as the degree and once more, none within them more often than the degree (a break), and the domain longer than
/// nothing; weights above 0. Its curvature, given, above 0; its indices, given, its first point and its last.
bool wellFormed(const NurbsSegment& s) noexcept;
/// A curve's plane from its reference (lat0, lon0) (Nurbs.cpp): on the Earth as `offsets` says - in the plane every
/// local path is laid out in, along great circles (A-GRA's azimuthal equidistant layout) or rhumb lines (as a frame's
/// are, Frames.cpp) - its axes turned `psi` from north.
void earthToPlane(double lat0, double lon0, FrameOffsets offsets, double psi, double lat, double lon, double& north, double& east) noexcept;
void planeToEarth(double lat0, double lon0, FrameOffsets offsets, double psi, double north, double east, double& lat, double& lon) noexcept;
/// The turn a frame's pose gives a curve's axes (4.27): its yaw, or its track over the ground (its yaw when still) - 0 unrotated.
double frameTurn(const FramePose& pose, FrameRotation rotation) noexcept;
/// A frame's attitude (A-GRA's ROTATION_3D): what turns its body's axes - x forward, y right, z down - into north, east
/// and down, by its roll, pitch and yaw, as a frame's point is turned (Frames.cpp).
struct Attitude {
    double m[3][3];
    explicit Attitude(const FramePose& pose) noexcept;
    void turn(double x, double y, double z, double& north, double& east, double& down) const noexcept;
};
/// The first section of a segment turning more than 1 % tighter than `curvature` (1/m) over the ground - the most its
/// curvature says it turns (4.26): its parameters from..to. False if none.
bool sharperThan(const NurbsSegment& s, double curvature, double& from, double& to) noexcept;

/// A curve as flown: its segments, and the arc length over the ground along
/// them (a table per segment, the Newton steps' measure of how far).
struct Curve {
    static constexpr std::uint32_t kMax = static_cast<std::uint32_t>(PathStore::kSegments);
    static constexpr int kSamples = 32;
    double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0; ///< the reference
    std::uint32_t count = 0;
    NurbsSegment segments[kMax];
    bool bezier[kMax] = {};            ///< segment i is a Bezier's form: flown by Bernstein's basis (measure() sets it)
    double table[kMax][kSamples + 1];  ///< segment i's length from its start at t = k / kSamples, m
    double startM[kMax + 1];           ///< the curve's length at each segment's start; startM[count] its whole
    Trims trims;                       ///< the follower's, flying it
    Turn orbit;                        ///< after its end, a wing that loiters orbits its end point
    Line exit;                         ///< after its end, one that continues flies on along its last course
    double fromM = 0.0;                ///< where along it the aircraft is (for Ahead::curvatureAt)
    // its plane (docs/flight-autonomy.md, 4.27): its points laid out on the Earth as `offsets` says, their axes turned
    // `psi` from north - by default the plane at its reference every local path is laid out in, unturned
    FrameOffsets offsets = FrameOffsets::Cartesian;
    double psi = 0.0;

    double lengthM() const noexcept { return startM[count]; }
    /// Laid out as every local path is (Cartesian: north along the meridian, east by the latitudes' cosine), unturned.
    bool plain() const noexcept { return offsets == FrameOffsets::Cartesian && psi == 0.0; }
    /// (lat, lon) in its plane: north and east of its reference, along its axes.
    void toPlane(double lat, double lon, double& north, double& east) const noexcept;
    /// Its plane's (north, east) on the Earth.
    void fromPlane(double north, double east, double& lat, double& lon) const noexcept;
    /// Its segments from `from` on, their third as `z` reads it (4.27) made down from its reference (at alt0).
    void normalize(std::uint32_t from, CurveZ z) noexcept;
    /// Its segments from `from` on, normalized, their points turned from its frame's body axes into north, east and down
    /// (4.27: ROTATION_3D; an affine change of its points is its curve's); an absolute altitude's kept, its x and y turned.
    void orient(std::uint32_t from, const Attitude& attitude, CurveZ z) noexcept;
    /// The length along the curve to segment i's parameter t.
    double at(std::uint32_t i, double t) const noexcept;
    /// The segment and its parameter a length `s` along the curve (within it).
    void find(double s, std::uint32_t& i, double& t) const noexcept;
    /// Measure segments from `from` on: their form, their tables, and where each starts.
    void measure(std::uint32_t from) noexcept;
    /// Segment i's point at t (0 to 1): by its form, as measure() found it.
    CurvePoint point(std::uint32_t i, double t) const noexcept;
};

/// Its curvature `aheadM` on from where the aircraft is (Curve::fromM): Ahead::curvatureAt for a curve.
double curvatureAhead(const void* curve, double aheadM) noexcept;
/// The most a rotorcraft at `speedMs` flies along the curve from where it is:
/// what each section within its braking distance allows (lateralLimit) and
/// brakes to in time, and with `stops`, what stops it at the curve's end.
double speedLimitAhead(const Performance& performance, const Curve& c, double speedMs, bool stops) noexcept;

/// The aircraft at (lat, lon) against the curve: the nearest point over the
/// ground, by Newton steps from where it was last (segment, t), moving on into
/// the next segment past one's end. alongM is the length along the curve to it.
Fix onCurve(const Curve& c, std::uint32_t& segment, double& t, double lat, double lon) noexcept;
/// The first section of segment i curving tighter than `limit` (1/m) over the
/// ground: its parameters from..to. False if it has none.
bool tooTight(const Curve& c, std::uint32_t i, double limit, double& from, double& to) noexcept;
/// Segment i's steepest gradient (climb per metre over the ground, either way) and where.
double steepest(const Curve& c, std::uint32_t i, double& at) noexcept;

/// What a route's waypoints leave out, filled in (docs/vehicle-interface.md,
/// 4.5): each field the previous point's, the first's the aircraft's own now
/// (a reference given alone, its value in that reference now, the altimeter's
/// where barometric; a rotorcraft's speed its cruise over the ground);
/// longitudes wrapped; an altitude left out held within its point's block
/// (docs/flight-autonomy.md, 4.29), a type given alone making the point a
/// waypoint. InvalidWaypoint, with the point's index in `bad`, for a point
/// that cannot be flown: not finite, out of range, a reference that is not
/// one, a speed that is not positive, a bank or climb rate out of range, a
/// reference given alone after the first point, the same place as the point
/// before (within a metre); a kind or type that is not one, a type with
/// another kind, a block upside down or an altitude outside it, a frame's
/// fields out of range, or its offsets without it.
Reason complete(Waypoint* out, const Waypoint* in, std::uint32_t count, bool repeat, const sim::VehicleState& state, const Performance& performance,
                bool hovers, std::int16_t& bad, const Altimeter* altimeter = nullptr) noexcept;
/// A waypoint (A-GRA's WayPoint): no turn there - flown over (4.29).
inline bool noTurn(const Waypoint& w) noexcept { return w.kind == static_cast<double>(EndPointKind::Waypoint); }

/// The plan's geometry from its complete points (`p.points`, `p.count` and
/// the options set): the legs, the entry from (lat, lon), the fly-by turns
/// sized for their speeds plus the wind and their banks, shrunk to fit short
/// legs (Turn::shrunk). The entry's turn is flown over if the aircraft is too
/// near its point to make it.
void plan(Plan& p, double lat, double lon, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept;
/// A rotorcraft over a point a moving frame carries (docs/flight-autonomy.md, 4.25, 4.29): the frame's velocity and a
/// closing on the point it could stop closing - after its velocity loop's lag, at half its deceleration, at half its
/// bandwidth nearer, as a formation closes on its slot - no faster than `transit`; its vertical speed as given.
VelocityCommand hoverOver(const sim::VehicleState& s, const Performance& performance, double lat, double lon, double transit, double frameNorthMs,
                          double frameEastMs, double verticalSpeedMs) noexcept;
/// Point i's piece of the plan again from where its points are now (a point in a moving frame: docs/flight-autonomy.md,
/// 4.29) - the leg to it (on the first lap at its start, the entry's, from where the entry began), the leg out of it
/// and its fly-by turn between them, sized as plan() sizes it and made no longer than those legs leave it beside the
/// turns at their other ends.
void replan(Plan& p, std::uint32_t i, bool firstLap, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept;

} // namespace fsim::control::route
