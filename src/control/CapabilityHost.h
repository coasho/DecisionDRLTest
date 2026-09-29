#pragma once

// The contract layer's per-vehicle half (docs/control-architecture.md,
// sections 9 and 10): takes commands (NEW, UPDATE, CANCEL) and answers them
// at once, arbitrates authority, keeps the activities and their records, and
// writes the runtime's configuration - between steps, on the caller's thread.
// After each world step it reads the runtime's report into its activities.

#include "control/Adapter.h"
#include "control/Catalog.h"
#include "control/Checks.h"
#include "control/Runtime.h"
#include "fsim/Capability.h"
#include "fsim/ControlStack.h"
#include "fsim/Span.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace fsim::control {

struct WindEstimate; // fsim/GuidanceModes.h

namespace route {
struct Plan;
struct Curve;
struct Leg;
}

class SupportTable;

/// Where the support effectors are, for the activities that complete when they get there.
struct EffectorPositions {
    double gear = kUnknown;  ///< 0 up .. 1 down
    double flaps = kUnknown; ///< 0 .. 1
};

/// What a vehicle has on board now, for the endurance check
/// (docs/flight-autonomy.md, 4.18): its flight model's, through the session.
struct EnergyNow {
    Energy energy = Energy::Unknown;
    double remaining = kUnknown;   ///< kg of fuel, or a battery's J
    double reserve = kUnknown;     ///< kept back for the end, in the same unit (the navigation settings')
    double consumption = kUnknown; ///< now: kg/s, or W
    double massKg = kUnknown;      ///< the whole vehicle's
};
/// What the host asks of the session it runs in, when it checks a command -
/// never while it steps.
class SessionView {
public:
    virtual ~SessionView() = default;
    /// A vehicle's energy on board, for a flight with an end (docs/flight-autonomy.md, 4.18).
    virtual EnergyNow energyNow(std::uint32_t vehicle) const = 0;
    /// The ground's height above the WGS-84 ellipsoid at a place, as the
    /// physics has it, and the spacing of its data (GroundProvider): what a
    /// commanded path is checked against (docs/flight-autonomy.md, 4.19).
    virtual double groundM(double latitudeRad, double longitudeRad) const = 0;
    virtual double groundResolutionM() const = 0;
    /// The world's UTC now, Unix seconds: the date a magnetic direction is turned at (4.22).
    virtual double utcSeconds() const = 0;
    /// The world's simulation time now, s: a route's arrival windows' clock (4.33).
    virtual double simTimeS() const = 0;
    /// A reference frame, and where it is now (4.21, 4.25): false for one it does not have (or whose vehicle is gone). A
    /// vehicle's own frame is `kVehicleFrames` plus its id (a must fly over it: 4.42).
    virtual bool frame(FrameId id, FrameSpec& spec, FramePose& now) const = 0;
    /// An operational point the world keeps (4.42; World::setOpPoint): false for one it does not.
    virtual bool opPoint(OpPointId id, OpPoint& out) const {
        (void)id, (void)out;
        return false;
    }
    /// An operational zone the world keeps (4.43; World::setOpZone), read where it is kept - a polygon's vertices copied
    /// would allocate, at a waiting must fly's start within a step - until the world's next change to it: null for one it
    /// does not keep.
    virtual const OpZone* opZone(OpZoneId id) const {
        (void)id;
        return nullptr;
    }
    /// An operational line the world keeps (4.44; World::setOpLine), read where it is kept, as a zone is: null for one it
    /// does not keep.
    virtual const OpLine* opLine(OpLineId id) const {
        (void)id;
        return nullptr;
    }
    /// An operational volume the world keeps (4.45; World::setOpVolume), read where it is kept: null for one it does not keep.
    virtual const OpVolume* opVolume(OpVolumeId id) const {
        (void)id;
        return nullptr;
    }
};

/// A vehicle's own frame, as the session answers it (SessionView::frame): this plus the vehicle's id - whole, and within the
/// 2^53 a route point's frame is read to, where the world's own frames never reach. What a must fly flies over another
/// vehicle through (docs/flight-autonomy.md, 4.42).
inline constexpr FrameId kVehicleFrames = FrameId{1} << 52;

/// A must fly merged as an UPDATE gives it (docs/flight-autonomy.md, 4.42; MustFly.cpp): the fields given replace the kept
/// ones; a location given (another than it was) replaces the location's own fields - a point's place, an id - left out.
void mergeMustFly(MustFlyCommand& dst, const MustFlyCommand& src) noexcept;
/// A curve's shape merged as an UPDATE gives it (docs/flight-autonomy.md, 4.27): its frame's fields given replace
/// the kept ones; a point (a latitude and longitude) given leaves the frame, and a frame given the point (Nurbs.cpp).
void mergeCurveShape(CurveCommand& curve, CurveShape& shape, const CurveCommand& given, const CurveShape& givenShape) noexcept;
/// The first field an UPDATE gives that moves a curve - its reference, its points' reading, its frame (0, 1, 2, 8 to 13,
/// then its shape's 14 to 19) - or -1.
int curveWhereField(const CurveCommand& given, const CurveShape* shape) noexcept;

/// A route's own beside its waypoints, as the host's checks take them (docs/flight-autonomy.md, 4.31, 4.34): its loiter
/// points' loiters and its planned states - passed down by pointer, so what is not a route passes none.
struct RouteExtras {
    Span<const RouteLoiter> loiters;
    Span<const RouteState> states;
    Span<const RoutePath> paths; ///< its paths (4.36)
    Span<const RouteBranch> branches; ///< its conditional branches (4.37)
    Span<const RouteTerminator> terminators; ///< its civil path terminators' data (4.38)
    const MustFlyArea* area = nullptr;       ///< a must fly's zone, corridor or volume given with it, as it was laid out then (4.43 to 4.45)
};

class CapabilityHost {
public:
    CapabilityHost();
    ~CapabilityHost();
    CapabilityHost(CapabilityHost&&) noexcept;
    CapabilityHost& operator=(CapabilityHost&&) noexcept;

    /// Ended activities a vehicle remembers for queries.
    static constexpr std::size_t kRecent = 16;
    /// Activities that can wait to start at once (docs/flight-autonomy.md,
    /// 4.9): a NEW that would wait beyond them is refused QueueFull.
    static constexpr std::size_t kWaiting = 16;
    /// Where activities live: the cascade's slots, one per support axis, then
    /// the engines' throttles (fsim.flight.engines, thrust beside the cascade).
    static constexpr std::size_t kEnginesSlot = kSlotCount + kSupportAxisCount;
    static constexpr std::size_t kActivities = kEnginesSlot + 1;

    /// The vehicle this host serves, its runtime, catalog, adapter and profile
    /// (which outlive it), and how often its runtime updates: protection as
    /// the profile gives it (docs/control-architecture.md, 11).
    void bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog, const VehicleAdapter& adapter,
              const VehicleProfile& profile, double controlPeriodS = 1.0 / 120.0) noexcept;
    /// The session's answers (it outlives the host): without them, no endurance or terrain is checked.
    void setSessionView(const SessionView* view) noexcept { sessionView_ = view; }
    /// Its barometric altimeter (docs/flight-autonomy.md, 4.20): the world's air and the QNH it is set to, which a
    /// barometric altitude is checked and flown on. Written by the session between steps, once bound.
    void setAltimeter(const Altimeter& altimeter) noexcept {
        if (config_) config_->altimeter = altimeter;
    }
    Altimeter altimeter() const noexcept { return config_ ? config_->altimeter : Altimeter{}; }
    /// The vehicle's support for the public features (it outlives the host):
    /// a command for one the catalog does not offer is refused NotSupported or
    /// NotImplemented (docs/flight-autonomy.md, 4.3). Without it, UnknownCapability.
    void setSupport(const SupportTable* support) noexcept { support_ = support; }

    /// NEW. `state` is the vehicle's, for availability; `now` the simulation time.
    /// It starts at once, or waits (kDeferred; docs/flight-autonomy.md, 4.9):
    /// for its start window, or for axes held by what it may not interrupt.
    CommandResult submit(const Command& command, const CommandOptions& options, const sim::VehicleState& state, double now);
    /// NEW for a support effector (gear, flaps, brakes, speedbrake, pitch trim) or the engines' throttles.
    CommandResult submit(const SupportCommand& command, const CommandOptions& options, const sim::VehicleState& state, double now);
    /// NEW of a route (fsim.guidance.route; docs/vehicle-interface.md 4.5): its
    /// waypoints completed and checked against the aircraft, planned from
    /// where it is, then written into the vehicle's path store (allocated at
    /// its first route). A RouteCommand submitted as a Command has none: InvalidWaypoint.
    /// The loiters its loiter points fly beside them (docs/flight-autonomy.md, 4.31), its planned states (4.34), its paths
    /// (4.36), its conditional branches (4.37) and its civil path terminators' data (4.38).
    CommandResult submit(const RouteCommand& route, Span<const Waypoint> waypoints, const CommandOptions& options, const sim::VehicleState& state,
                         double now, Span<const RouteLoiter> loiters = {}, Span<const RouteState> states = {}, Span<const RoutePath> paths = {},
                         Span<const RouteBranch> branches = {}, Span<const RouteTerminator> terminators = {});
    /// NEW of a curve (fsim.guidance.curve; docs/vehicle-interface.md 4.7):
    /// its segments checked against the aircraft (InvalidCurve naming the
    /// segment, and a section too tight), then written into the path store.
    /// A CurveCommand submitted as a Command has none: InvalidCurve. Its
    /// segments as A-GRA's schema gives them (docs/flight-autonomy.md, 4.26),
    /// or Bezier segments, each made one (NurbsSegment::of).
    /// Its reference in a frame beside it, `shape` (docs/flight-autonomy.md, 4.27; null: none).
    CommandResult submit(const CurveCommand& curve, Span<const NurbsSegment> segments, const CommandOptions& options, const sim::VehicleState& state,
                         double now, const CurveShape* shape = nullptr);
    CommandResult submit(const CurveCommand& curve, Span<const BezierSegment> segments, const CommandOptions& options, const sim::VehicleState& state,
                         double now, const CurveShape* shape = nullptr);
    /// NEW of a pattern with its shape (fsim.guidance.pattern; docs/flight-autonomy.md, 4.23): the two checked and
    /// completed together, the shape then written into the path store (allocated at the first shape given).
    CommandResult submit(const PatternCommand& pattern, const PatternShape& shape, const CommandOptions& options, const sim::VehicleState& state,
                         double now);
    /// NEW of a must fly with its zone given (fsim.guidance.must_fly; docs/flight-autonomy.md, 4.43; MustFly.cpp): the zone
    /// checked (InvalidParameter naming its field from 10) and laid out as it is given, then kept with the activity.
    CommandResult submit(const MustFlyCommand& mustFly, const OpZone* zone, const CommandOptions& options, const sim::VehicleState& state, double now);
    /// UPDATE of a must fly with a zone given in place of its own (4.43): as submit's, then as update's.
    CommandResult update(ActivityId activity, const MustFlyCommand& mustFly, const OpZone* zone, const sim::VehicleState& state, Caller caller);
    /// NEW and UPDATE of a must fly with its corridor given (4.44): as a zone's, the line's fields named from 10.
    CommandResult submit(const MustFlyCommand& mustFly, const OpLine* line, const CommandOptions& options, const sim::VehicleState& state, double now);
    CommandResult update(ActivityId activity, const MustFlyCommand& mustFly, const OpLine* line, const sim::VehicleState& state, Caller caller);
    /// NEW and UPDATE of a must fly with its volume given (4.45): as a zone's, the volume's fields named from 10.
    CommandResult submit(const MustFlyCommand& mustFly, const OpVolume* volume, const CommandOptions& options, const sim::VehicleState& state, double now);
    CommandResult update(ActivityId activity, const MustFlyCommand& mustFly, const OpVolume* volume, const sim::VehicleState& state, Caller caller);
    /// UPDATE: a new setpoint for a live activity - the fast path; allocates
    /// nothing. `state` is the vehicle's: a route is planned afresh from it.
    /// `caller` is the source the caller declares, as a NEW's options do, and
    /// a policy's controller: under ControlMode::Granted one below the
    /// activity's may not address it (AuthorityHeld, naming it) - a policy
    /// cannot change or end what the platform's own sources fly - nor may a
    /// controller another's. Open, as ADR-26 10.1: any caller.
    /// A pattern's `shape`, if given, merged with it (as update(PatternCommand, PatternShape)).
    CommandResult update(ActivityId activity, const Command& setpoint, const sim::VehicleState& state, Caller caller,
                         const PatternShape* shape = nullptr) noexcept;
    CommandResult update(ActivityId activity, const SupportCommand& setpoint, Caller caller) noexcept;
    /// UPDATE of a pattern with its shape: the fields given in either (kHold keeps one) merged into both, and the
    /// pattern flown afresh (docs/flight-autonomy.md, 4.23).
    CommandResult update(ActivityId activity, const PatternCommand& pattern, const PatternShape& shape, const sim::VehicleState& state,
                         Caller caller) noexcept;
    /// UPDATE of a route: its options (a field left out, kHold, keeps its
    /// value) and its waypoints - none: those it has - checked as a NEW's,
    /// then flown afresh from its start, from where the aircraft is. New
    /// waypoints come with their loiters (4.31), states (4.34), paths (4.36), branches (4.37) and terminators (4.38); none,
    /// it keeps its own.
    CommandResult update(ActivityId activity, const RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state,
                         Caller caller, Span<const RouteLoiter> loiters = {}, Span<const RouteState> states = {},
                         Span<const RoutePath> paths = {}, Span<const RouteBranch> branches = {},
                         Span<const RouteTerminator> terminators = {}) noexcept;
    /// The operator's input to a route's conditional branch (docs/flight-autonomy.md, 4.37; Branches.cpp): branch `branch`,
    /// one that takes it, commanded (or no longer) - its route flying or waiting, addressed as an UPDATE is; held until the
    /// route is flown afresh (a NEW, an UPDATE, a Reset). InvalidParameter (index: the branch) for one it has not, or one that
    /// takes no operator input.
    CommandResult commandBranch(ActivityId activity, std::uint32_t branch, bool commanded, Caller caller) noexcept;
    /// UPDATE of a curve: the options given (kHold keeps one), and segments -
    /// with `append` 1, after its end, from the same reference; else a new
    /// curve, flown afresh. Options alone change how it is flown, not where.
    /// Where a curve is - its reference, its points' reading, its frame (4.27) - changes only with segments: given
    /// with its options alone, InvalidParameter naming the first. Its shape merged as a pattern's (a point replaces a
    /// frame, a frame a point); an append keeps its reference and reading.
    CommandResult update(ActivityId activity, const CurveCommand& curve, Span<const NurbsSegment> segments, const sim::VehicleState& state,
                         Caller caller, const CurveShape* shape = nullptr) noexcept;
    CommandResult update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments, const sim::VehicleState& state,
                         Caller caller, const CurveShape* shape = nullptr) noexcept;
    /// CANCEL: the activity ends and its axes return to the vehicle default
    /// (`caller` as for UPDATE); what waited for them may start.
    CommandResult cancel(ActivityId activity, const sim::VehicleState& state, double now, Caller caller) noexcept;
    /// An activity command (docs/flight-autonomy.md, 4.10) for a live activity
    /// - flying, waiting or disabled - `caller` as for UPDATE; `rank` for
    /// ChangeRank. Refused NotInteractive where its command said it takes
    /// none, QueueFull where a flying one has no room to be kept (Disable,
    /// Unassign). What waits may start after it. May allocate: an activity
    /// kept out of its slot keeps its route's waypoints and its curve's segments.
    CommandResult activityCommand(ActivityId activity, ActivityCommand command, Rank rank, const sim::VehicleState& state, double now, Caller caller);

    // --- Flight tasks (docs/flight-autonomy.md, 4.11): kept by id, flown on a task command ---
    /// Suggestions the platform keeps at once (the oldest not flying makes room).
    static constexpr std::size_t kSuggestions = 16;
    /// Keep a task: a flight or guidance command, a route's waypoints, a
    /// curve's segments, how often it flies. InvalidParameter for id 0 or one
    /// with kSuggestedTask, no runs, a negative interval, or runs of a
    /// capability that never completes; TaskActive while its activity is live;
    /// else why the vehicle cannot command the capability.
    Reason storeTask(TaskId id, const Command& command, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments, TaskRepetition repetition,
                     const PatternShape* shape = nullptr, const CurveShape* curveShape = nullptr, Span<const RouteLoiter> loiters = {},
                     Span<const RouteState> states = {}, Span<const RoutePath> paths = {}, Span<const RouteBranch> branches = {},
                     Span<const RouteTerminator> terminators = {}, const OpZone* zone = nullptr, const OpLine* line = nullptr,
                     const OpVolume* volume = nullptr);
    /// A task command: the NEW of its command with `options`, the task among
    /// the requirements it traces to, answered as the NEW is; its runs, as the
    /// task says. UnknownTask; TaskActive while its activity is live.
    CommandResult commandTask(TaskId id, CommandOptions options, const sim::VehicleState& state, double now);
    /// Its live activity canceled (`caller` as for CANCEL); one never commanded will not be. UnknownTask.
    CommandResult cancelTask(TaskId id, const sim::VehicleState& state, double now, Caller caller);
    /// Forget it: UnknownTask; TaskActive while its activity is live.
    Reason removeTask(TaskId id);
    /// Its status; false for a task not kept.
    bool taskStatus(TaskId id, TaskStatus& out);
    /// Every task kept - the caller's and the platform's suggestions - in the order they were made.
    std::vector<TaskStatus> tasks();

    // --- Route plans (docs/flight-autonomy.md, 4.39): kept by id, taken through the activation states (Plans.cpp) ---
    /// The plans a vehicle keeps.
    static constexpr std::size_t kPlans = 32;
    /// A plan published (A-GRA's MA_RoutePlanMT), taken where FA listens for
    /// its id: None (A-GRA's notification CONFIRMED). WrongPlanState where FA
    /// does not; InvalidParameter for id 0 or malformed metadata. May allocate.
    Reason publishPlan(const RoutePlan& plan);
    /// A plan activation command, answered at once. Activate is the NEW of its
    /// route with `options`; PrepareForActivation its validation with them;
    /// Deactivate cancels, as `options`' source and controller, an activity
    /// not flying yet. May allocate.
    PlanCommandResult planCommand(PlanId id, PlanCommand command, const CommandOptions& options, const sim::VehicleState& state, double now);
    /// FA's own deactivation (VI 1.2.5.7): a plan ready for activation, or
    /// activated, Deactivated; its live activity canceled with `reason`, its
    /// execution Canceled. UnknownPlan; WrongPlanState in any other state.
    PlanCommandResult abortPlan(PlanId id, Reason reason, const sim::VehicleState& state, double now);
    /// Forget a plan: UnknownPlan; WrongPlanState while it is activated and its activity is live.
    Reason removePlan(PlanId id);
    /// Its status; false for a plan not kept.
    bool planStatus(PlanId id, PlanStatus& out) const;
    /// Every plan kept, in the order they were first prepared for upload.
    std::vector<PlanStatus> plans() const;
    /// Its content as uploaded last, its metadata with it; false for a plan not kept, or kept before its first upload.
    bool plan(PlanId id, RoutePlan& out) const;
    // FA's own plans and the airfields (4.40)
    /// The airfields a vehicle keeps.
    static constexpr std::size_t kAirfields = 32;
    /// FA's own plan, the platform's: kept Uploaded and read only to MA, in
    /// place of any plan by its id whose activity is not live. InvalidParameter
    /// as a published plan's; UnknownAirfield for a takeoff's or a landing's
    /// path whose airfield or runway the vehicle does not keep; WrongPlanState
    /// while the plan it replaces flies; PlanStoreFull. May allocate.
    Reason loadPlan(const RoutePlan& plan);
    /// An airfield, the platform's: kept in place of any by its id, its
    /// revision one more. InvalidParameter for a malformed one; PlanStoreFull. May allocate.
    Reason loadAirfield(const Airfield& airfield);
    /// Every airfield kept, as loaded, in the order they were first loaded.
    std::vector<Airfield> airfields() const;
    /// One airfield as loaded; false for one not kept.
    bool airfield(AirfieldId id, Airfield& out) const;
    // Validation (4.41)
    /// A route plan validated without flying it: its route checked as its
    /// NEW would be, in `v`'s weather, from its origin, its verdict over its
    /// parts. The plan need not be kept; nothing is kept or flown. check's
    /// InvalidParameter for a malformed plan or validation. May allocate.
    PlanValidationResult validatePlan(const RoutePlan& plan, const PlanValidation& v, const sim::VehicleState& state, double now);
    /// A kept plan's, as uploaded last: UnknownPlan; WrongPlanState before its first upload.
    PlanValidationResult validatePlan(PlanId id, const PlanValidation& v, const sim::VehicleState& state, double now);

    // --- Reports (docs/flight-autonomy.md, 4.12): what an activity flies, and where to ---
    /// What a live activity flies now, or waits to fly: its setpoint as
    /// updated, a route's waypoints, a curve's segments (appended ones too).
    /// False for one not live.
    bool setpoint(ActivityId activity, Setpoint& out) const;
    /// A live route's next point with a required time of arrival, as its behaviour estimates it (4.33): false for none.
    bool arrival(ActivityId activity, ArrivalEstimate& out) const noexcept;
    /// Where a live activity flies to: the point it flies to now, then those
    /// after it - a route's waypoints (a repeating route's round again), a
    /// curve's segment ends, a pattern's fix, the position level's point -
    /// `max` at most.
    std::vector<EndPoint> endPoints(ActivityId activity, std::size_t max) const;
    /// What the vehicle is commanded (A-GRA's VehicleCommandState): the
    /// runtime's levels, the acceleration they command at the attitude
    /// `state` flies, the altitude as its mode commanded it.
    VehicleCommandState commandState(const sim::VehicleState& state) const noexcept;
    /// The existing entry points (docs/control-architecture.md, 10.7): an
    /// UPDATE of their live activity at the same capability, else a NEW with
    /// the legacy options.
    CommandResult command(const Command& command, const sim::VehicleState& state, double now);
    /// command()'s per-step path, inline: a new setpoint for the live legacy
    /// activity at the same level; false if command() has more to do.
    bool updateLegacy(const Command& command) noexcept {
        if (legacySlot_ < 0 || std::holds_alternative<BehaviorCommand>(command)) return false;
        SetpointSlot& slot = config_->slots[static_cast<std::size_t>(legacySlot_)];
        if (slot.command.index() != command.index()) return false;
        applySetpoint(slot.command, command); // a mode's setpoint merged, a level's replaced
        ++slot.revision;
        return true;
    }

    /// What the vehicle can do (docs/vehicle-interface.md, 7.1): its adapter's
    /// answer from the profile and the loops it flies with, which its guidance
    /// flies with - as of the last refreshPerformance().
    const Performance& performance() const noexcept { return performance_; }
    /// The performance afresh if the runtime's loops changed since it was
    /// computed (ControlStack::loopsRevision): a new revision, and the control
    /// revision counts it, if any of it changed. After each step and before a
    /// command is checked; a query calls it first.
    void refreshPerformance() noexcept;

    // --- Authority (docs/vehicle-interface.md, 6) and availability (7.2); between steps ---
    /// Open (as ADR-26), or Granted: a policy's NEW then needs a grant for its
    /// capability, and the policy's live activities no grant covers end
    /// Canceled(NotGranted).
    void setControlMode(ControlMode mode, const sim::VehicleState& state, double now) noexcept;
    ControlMode controlMode() const noexcept { return controlMode_; }
    /// A policy's controller asks for control of a capability (A-GRA's
    /// ACQUIRE): granted - Reason::None - if it is allowed and available, and
    /// no other controller holds it; else NotAllowed, the reason it is
    /// unavailable, or AuthorityHeld. UnknownCapability for one it cannot command.
    Reason requestControl(std::size_t capability, const sim::VehicleState& state, ControllerId controller = 0) noexcept;
    /// A controller lets go: its grant ends, and its live activities of the
    /// capability end Canceled(Released); their axes fly the vehicle default.
    /// NotGranted, and nothing changes, where another controller holds it.
    Reason releaseControl(std::size_t capability, const sim::VehicleState& state, double now, ControllerId controller = 0) noexcept;
    /// The platform takes it back: the grant ends, and the policy's live
    /// activities of the capability - every controller's - end Canceled with `reason`: Revoked (if
    /// None), CollisionAvoidance or Restricted; InvalidParameter for another,
    /// and nothing changes.
    Reason revokeControl(std::size_t capability, Reason reason, const sim::VehicleState& state, double now) noexcept;
    /// Whether the policy may request the capability (all may, by default); a
    /// grant for one no longer allowed is revoked.
    Reason setAllowed(std::size_t capability, bool allowed, const sim::VehicleState& state, double now) noexcept;
    ControlStatus controlStatus(std::size_t capability) const noexcept;
    /// The platform restricts a capability: a policy's NEW for it, and a
    /// request, are refused with `reason` - Restricted (if None),
    /// CollisionAvoidance or Unavailable; InvalidParameter for another - and
    /// live activities go on. Availability::Available lifts it (its reason unused).
    /// The status reports it, with the id it is about (`associated`: the
    /// vehicle avoided) and when it is expected back (simulation time; NaN: not known).
    Reason setAvailability(std::size_t capability, Availability availability, Reason reason, std::uint64_t associated = 0,
                           double nextAvailableS = kUnknown) noexcept;
    /// Counts every change to the grants, what is allowed, the control mode,
    /// availability and the performance (6.3): a consumer polls it.
    std::uint32_t controlRevision() const noexcept { return controlRevision_; }

    /// A capability's precedence (A-GRA's CapabilityPrecedence; lower first, 0
    /// for every capability until set): which of two activities of one source
    /// keeps contested axes before their ranks are compared
    /// (docs/flight-autonomy.md, 4.9). The platform's setting. Its live and
    /// waiting activities that do not override it are arbitrated by it from now;
    /// what waits may start. UnknownCapability for one it cannot command.
    Reason setPrecedence(std::size_t capability, std::uint32_t precedence, const sim::VehicleState& state, double now) noexcept;
    std::uint32_t precedence(std::size_t capability) const noexcept;

    /// Everything the checks found for the last NEW, validation or UPDATE this
    /// host answered (docs/flight-autonomy.md, 4.8): every finding - the first
    /// the answer's reason - and every value flown other than asked. The
    /// existing entry points' per-step path leaves it as it was.
    const CommandDetails& details() const noexcept { return details_; }

    /// Live (flying, or waiting to start) or recently ended; null if unknown.
    const ActivityRecord* activity(ActivityId activity) const noexcept;
    /// The live activities - those flying, then those waiting to start - then
    /// the ended ones it remembers, newest first.
    std::vector<ActivityRecord> activities() const;
    /// Its availability as a policy is answered (docs/flight-autonomy.md, 4.1,
    /// 4.5 and 4.6): every reason that holds - the vehicle's own (a diverged
    /// vehicle), the flight phase (the airborne guidance on the ground), the
    /// platform's restriction, a support effector's placard - the first of them
    /// as `reason`, and the ranges a placard narrows. A policy's NEW is refused
    /// with the same reason.
    CapabilityStatus status(std::size_t capability, const sim::VehicleState& state) const noexcept;

    /// What flies the primary axes nobody owns (docs/control-architecture.md,
    /// 6.2); between steps. Refused (ControllerNotAxisAware) if Hold would fly
    /// axes owned apart through a controller that is not axis-aware.
    Reason setVehicleDefault(VehicleDefault mode) noexcept;
    VehicleDefault vehicleDefault() const noexcept { return config_->vehicleDefault; }

    /// Envelope protection's mode (fsim.envelope.protection); between steps.
    void setProtection(ProtectionMode mode) noexcept;
    ProtectionMode protection() const noexcept;
    /// What protection saw since the last call: the limits it reduced the
    /// demand for, the state's exceedances (how long, how far). Starts a new count.
    EnvelopeStatus envelope() noexcept;

    /// After each world step: the runtime's report into the activities, then
    /// cleared; the time windows kept; what waits started when it may. True if
    /// something started (the level flown may have changed).
    bool afterStep(const sim::VehicleState& state, const EffectorPositions& positions, double now) noexcept;
    /// Whether afterStep needs the effectors' positions (a gear or flaps activity is under way).
    bool awaitsPosition() const noexcept {
        for (std::size_t s = kSlotCount; s < kActivities; ++s)
            if (slots_[s].live && slots_[s].target == slots_[s].target) return true; // a live goal (not NaN)
        return false;
    }
    /// Vehicle reset: live activities start again.
    void onReset() noexcept;

private:
    struct Slot {
        ActivityId activity = 0; ///< what this slot flies: a live activity, or an ended one's residual hold; 0 = free
        bool live = false;
        RangePolicy range = RangePolicy::Clamp;
        std::uint16_t flags = 0;  ///< host flags since the last world step (kActivityClamped, kActivityAxesReduced)
        double target = kUnknown; ///< a terminating support activity's goal (gear down 1 / up 0, a flap position)
        bool outside = false;     ///< a manoeuvre's: the state went past a limit by more than a limiter overshoots
        std::uint32_t precedenceOverride = kNoPrecedenceOverride; ///< its command's (CommandOptions::precedenceOverride)
        double firstStart = kHold; ///< a route's start as commanded: Reset flies from it (a route resumed flies from elsewhere)
        double restartAt = kUnknown; ///< a task's next run begins then (4.11); NaN: none due
    };

    /// An activity waiting to start (docs/flight-autonomy.md, 4.9): its record
    /// and its command as given, prepared afresh from the state when it starts.
    struct Waiting {
        bool used = false;
        bool support = false;           ///< a support effector's or the engines' command
        ActivityRecord record{};
        CommandOptions options{};
        Command command{};
        SupportCommand supportCommand{};
        std::unique_ptr<Behavior> behavior; ///< a guidance capability's, made at its NEW
        std::vector<Waypoint> waypoints;    ///< a route's (room for the path store's, reserved at its NEW)
        std::vector<NurbsSegment> segments; ///< a curve's (likewise)
        std::uint64_t queued = 0;           ///< when it began to wait (queueSerial_): its place among equals
        double firstStart = kHold;          ///< a route's start as commanded (Reset: kept out of its slot, it resumed elsewhere)
        /// A waiting activity that failed as it would start, kept as the
        /// suggestion its record names (4.11) until a call makes it a task
        /// (what a start in a step may not allocate).
        bool suggested = false;
        TaskId suggestion = 0;
        bool resumed = false; ///< it flew before (disabled, unassigned): its start window was its first start's
        PatternShape shape{};  ///< a pattern's (docs/flight-autonomy.md, 4.23)
        CurveShape curveShape{}; ///< a curve's reference in a frame (4.27)
        std::vector<RouteLoiter> loiters; ///< a route's (4.31; room for the path store's, reserved at its NEW)
        std::vector<RouteState> states;   ///< a route's planned states (4.34; likewise)
        std::vector<RouteState> passed;   ///< those a route resumed past (disabled, unassigned): Reset gives them back
        std::vector<RoutePath> paths;     ///< a route's paths (4.36)
        std::vector<RouteBranch> branches; ///< a route's conditional branches (4.37)
        std::uint32_t commanded = 0;      ///< the branches the operator has commanded (4.37: bit k, branch k)
        std::vector<RouteTerminator> terminators; ///< a route's civil path terminators' data (4.38)
        MustFlyArea area; ///< a must fly's zone, corridor or volume given with it, as laid out (4.43 to 4.45; laidOut() false: none)
    };
    /// A flight task (4.11): its command, and what became of it.
    struct Task {
        TaskId id = 0;
        bool suggested = false;             ///< the platform's
        Command command{};
        std::vector<Waypoint> waypoints;
        std::vector<NurbsSegment> segments;
        PatternShape shape{};               ///< a pattern's (docs/flight-autonomy.md, 4.23)
        CurveShape curveShape{};            ///< a curve's reference in a frame (4.27)
        std::vector<RouteLoiter> loiters;   ///< a route's (4.31)
        std::vector<RouteState> states;     ///< a route's planned states (4.34)
        std::vector<RoutePath> paths;       ///< a route's paths (4.36)
        std::vector<RouteBranch> branches;  ///< a route's conditional branches (4.37)
        std::vector<RouteTerminator> terminators; ///< a route's civil path terminators' data (4.38)
        TaskRepetition repetition{};
        ActivityId activity = 0;            ///< its activity (every run's), while it is commanded
        std::uint64_t commandId = 0;        ///< its task command's
        TaskState ended = TaskState::AwaitingExecution; ///< once its activity ended (or it was canceled before it was commanded)
        Reason reason = Reason::None;
        std::uint32_t run = 0, runs = 0;    ///< its activity's, as it ended
        double percent = kUnknown;          ///< likewise
        double startTime = kUnknown, endTime = kUnknown;
        MustFlyArea area;                   ///< a must fly's zone, corridor or volume given with it, as laid out (4.43 to 4.45; laidOut() false: none)
    };
    Task* findTask(TaskId id) noexcept;
    TaskStatus statusOf(const Task& t) const noexcept;
    /// An activity ended: the task it flew (if any) takes its end - in the step, allocating nothing.
    void noteEnd(const ActivityRecord& record) noexcept;
    /// The platform's suggestion (4.11): a task with the command the checks
    /// left, every value held to its limit (a route's points as planned). Its id.
    TaskId suggest(const Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments, const PatternShape* shape = nullptr,
                   const CurveShape* curveShape = nullptr, const RouteExtras* extras = nullptr);
    /// Room for a suggestion: the oldest not flying goes where kSuggestions are kept.
    Task& newSuggestion(TaskId id);
    /// Failed waiting activities kept as suggestions, made tasks now.
    void materialize();
    /// A task's interval between runs, by its activity; NaN if none.
    double repeatInterval(ActivityId activity) const noexcept;
    /// Slot s's activity flies its next run: its behaviour afresh.
    void restartRun(std::size_t slot) noexcept;
    /// A live activity's record, flying or waiting; null if none.
    ActivityRecord* liveRecord(ActivityId activity) noexcept;
    /// A flying activity leaves its slot, kept in the waiting store: Disabled,
    /// or Pending to wait for its axes again, behind what waits. Its behaviour
    /// is kept, its setpoint and a route's waypoints or a curve's segments too;
    /// a route resumes at the point it flew to. False if there is no room.
    bool retire(std::size_t slot, ActivityState state);
    /// The options slot `slot`'s activity was commanded with, as its record and slot keep them.
    CommandOptions optionsOf(std::size_t slot) const noexcept;
    /// What a support or engines slot demands now, as its command.
    SupportCommand supportCommandOf(std::size_t slot) const noexcept;
    /// What a contender may do about a live activity on its axes.
    enum class Standing : std::uint8_t { Takes, Waits, Refused };
    /// The rules of docs/flight-autonomy.md, 4.9: a higher source's activity
    /// refuses an interrupting command (AuthorityHeld) and is waited for by the
    /// rest; the platform's interrupting command takes any rank; a policy's
    /// that does not interrupt waits; otherwise precedence, then rank decide -
    /// ahead of it takes, and when equal the newer command: `id` the
    /// contender's (kNewest for a NEW's).
    static Standing standing(Source source, std::uint32_t precedence, Rank rank, bool interrupt, ActivityId id, const ActivityRecord& holder) noexcept;
    /// A contender against every live activity on `axes`: Takes if it may take
    /// them all; else Refused or Waits with `blocker` (Refused first).
    Standing arbitrate(AxisMask axes, Source source, std::uint32_t precedence, Rank rank, bool interrupt, ActivityId id,
                       ActivityId& blocker) const noexcept;
    /// A NEW's command, newer than every activity's.
    static constexpr ActivityId kNewest = ~ActivityId{0};
    /// A command's time window (docs/flight-autonomy.md, 4.9): each bound NaN
    /// or finite and in order (InvalidParameter), and one it can still meet at
    /// `now` - an end window not yet closed, a critical start window not yet
    /// closed (TimeConstraint).
    static Reason checkWindow(const TimeWindow& window, double now) noexcept;
    /// A NEW's command prepared to fly from `state`: completed (an hsa's, a
    /// pattern's), a route planned and a curve measured into their scratch,
    /// checked as its range policy says - the malformed returned at once, the
    /// rest logged - and the admission a behaviour asks. `setpoint` is what flies.
    Reason prepare(std::size_t index, Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments, const sim::VehicleState& state,
                   CheckLog& log, const PatternShape* shape = nullptr, const CurveShape* curveShape = nullptr, const RouteExtras* extras = nullptr);
    /// The axes a command owns: its own, else the capability's default, widened
    /// above the actuators to whole groups; InvalidAxes if not a flyable set.
    Reason axesOf(std::size_t index, const Command& command, const CommandOptions& options, AxisMask& axes) const noexcept;
    /// What a NEW or a waiting activity starts with: its record - `waited`'s,
    /// else one made from the NEW's options - and what its checks raised.
    struct Launch {
        const ActivityRecord* waited = nullptr;
        ActivityId id = 0;
        std::size_t capability = 0;
        AxisMask axes = 0;
        std::uint16_t flags = 0;
        double firstStart = kHold; ///< a route's start as commanded
    };
    /// What the NEW (or a waiting activity starting) flies, into a slot: it
    /// takes its axes, its setpoint and behaviour are installed, its record
    /// kept. A route's plan and a curve's segments go into the path store.
    void launch(const Launch& what, const CommandOptions& options, Command&& setpoint, std::unique_ptr<Behavior>&& behavior, bool route,
                Span<const NurbsSegment> curve, double now) noexcept;
    /// A support command's or the engines' NEW (or waiting activity) set directly.
    void launchDirect(const Launch& what, const CommandOptions& options, const SupportCommand& setpoint, double now) noexcept;
    /// A NEW's record, pending, from its options (docs/flight-autonomy.md, 4.8, 4.9), into `record`.
    void makeRecord(ActivityRecord& record, ActivityId id, std::size_t capability, const CommandOptions& options, AxisMask axes, double now) const noexcept;
    /// Room to wait: a free entry, else null (QueueFull).
    Waiting* freeWaiting();
    /// A waiting activity's entry; null if none waits by that id.
    Waiting* waitingEntry(ActivityId activity) noexcept;
    const Waiting* waitingEntry(ActivityId activity) const noexcept;
    /// A waiting activity ends: its record kept with the ended ones.
    void endWaiting(Waiting& w, ActivityState state, Reason reason, double now) noexcept;
    /// What waits and may start now starts, in order of source, precedence,
    /// rank, then age; what may not waits on (docs/flight-autonomy.md, 4.9).
    /// A waiting activity whose windows it can no longer meet fails. True if
    /// one started. Nothing waiting, nothing done.
    bool schedule(const sim::VehicleState& state, double now) noexcept { return waitingCount_ && scheduleWaiting(state, now); }
    bool scheduleWaiting(const sim::VehicleState& state, double now) noexcept;
    /// A waiting activity started, if what it is prepared into still flies: false and it failed if not.
    bool startWaiting(Waiting& w, const sim::VehicleState& state, double now) noexcept;
    /// UPDATE of a waiting activity: its command's new setpoint, checked as its NEW was, kept for its start.
    CommandResult updateWaiting(Waiting& w, const Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments,
                                const sim::VehicleState& state, Caller caller, const PatternShape* shape = nullptr,
                                const CurveShape* curveShape = nullptr, const RouteExtras* extras = nullptr) noexcept;
    CommandResult updateWaiting(Waiting& w, const SupportCommand& setpoint, Caller caller) noexcept;
    /// The live activities' time windows after a world step: a persistent one
    /// done at its end window's close, a terminating one late or early failed if its end is critical.
    void keepWindows(double now) noexcept;
    /// Its capability's precedence for an activity: its command's override, else its capability's.
    std::uint32_t precedenceFor(std::size_t capability, std::uint32_t override) const noexcept;

    /// A slot that flies through the cascade (else it is set directly: a support effector or the engines).
    static bool isCascade(std::size_t slot) noexcept { return slot < kSlotCount; }
    static bool isSupport(std::size_t slot) noexcept { return slot >= kSlotCount && slot < kEnginesSlot; }
    /// Where a directly set command lives.
    static std::size_t directSlot(const SupportCommand& command) noexcept;
    /// Its demand into the runtime's configuration.
    void writeDirect(std::size_t slot, const SupportCommand& command) noexcept;
    /// The axes a flight or guidance command may own (6.1, 9.1).
    static Reason checkAxes(const CapabilityDescriptor& d, AxisMask axes) noexcept;
    /// ControllerNotAxisAware (9.6): after `taken` goes to a new owner - one
    /// entering the cascade at `level`, or beside it if !cascade - would a
    /// controller that does not honour ControlContext::engaged fly axes owned apart?
    Reason checkAwareness(AxisMask taken, Level level, bool cascade) const noexcept;
    /// Every controller from Attitude up to `top` is axis-aware.
    Reason awareUpTo(int top) const noexcept;
    int liveSlot(ActivityId activity) const noexcept;
    /// Before activity `id` takes `axes` (9.3): a live activity that loses a
    /// primary axis ends, preempted; one that loses only support axes carries
    /// on without them. What keeps a primary axis flies on - a residual hold
    /// if it ended - and a slot left without one is freed.
    void takeOver(AxisMask axes, ActivityId id, double now) noexcept;
    /// Slot `slot` flies the activity `what` names (its command's options), its record made in place.
    ActivityRecord& start(std::size_t slot, const Launch& what, const CommandOptions& options, double now) noexcept;
    void end(std::size_t slot, ActivityState state, Reason reason, ActivityId by, double now) noexcept;
    void release(std::size_t slot) noexcept;
    CommandResult rejected(Reason reason, ActivityId activity = 0, ActivityId other = 0) const noexcept;
    /// A validation's answer (CommandOptions::validateOnly): Valid, with what the checks found.
    CommandResult valid(const CommandResult& detail) const noexcept;
    /// The setpoint of the live hsa activity, if one flies (a NEW continues what it commanded).
    const HsaCommand* liveHsa() const noexcept;
    /// A NEW hsa (docs/vehicle-interface.md, 4.4): its references checked, the
    /// fields it leaves out from the live hsa it replaces or the state, its
    /// angles wrapped. InvalidParameter (with the field in `detail`) if malformed.
    Reason resolveHsa(HsaCommand& c, const sim::VehicleState& state, CommandResult& detail) const noexcept;
    /// A speed optimisation the vehicle can fly (docs/flight-autonomy.md,
    /// 4.17): none asked for, or its performance tables to fly it from -
    /// without them NotImplemented, the field (`field`) in `detail`.
    Reason optimisable(double optimization, std::int16_t field, CommandResult& detail) const noexcept;
    /// What a flight with an end needs, against what the vehicle has above
    /// its reserve (docs/flight-autonomy.md, 4.18): a route that does not
    /// repeat, to its last point (its loiters' times too: 4.31); a timed pattern; a curve, to its end -
    /// complete, and checked: its route or curve in the scratch plan. Flown
    /// level, each leg at its speed and altitude, at the weight now: the
    /// performance tables' burn there, else what it consumes now. `energy` 0:
    /// nothing to judge (no end, no energy, no speed).
    CommandDetails::Endurance endurance(const Command& setpoint, const sim::VehicleState& state, double now = kUnknown) const noexcept;
    /// Where a commanded path first goes below the terrain (docs/flight-autonomy.md,
    /// 4.19): a route's legs and turns from where the aircraft is (its loiters' ways in and laps: 4.31), and what it
    /// flies after its last point; a pattern's lap and its entry; a curve; an
    /// hsa's line ahead for a minute - complete, its route or curve in the
    /// scratch plan. Sampled at the ground's spacing; `hit` 0: clear.
    CommandDetails::Terrain terrain(const Command& setpoint, const sim::VehicleState& state) const noexcept;
    /// The declination where the aircraft is, at the world's date (docs/flight-autonomy.md, 4.22): what a magnetic
    /// direction is turned by to be true.
    double declinationNow(const sim::VehicleState& state) const noexcept;
    /// The world's date as the magnetic model reads it (fsim/Magnetic.h).
    double yearNow() const noexcept;
    /// The terrain check of a route, pattern, curve or hsa that is checked:
    /// where it meets the ground, a finding (TerrainConflict) no range policy mends.
    void checkTerrain(const Command& setpoint, const sim::VehicleState& state, CheckLog& log) const noexcept;
    /// A speed optimisation's snapshot, as the command is given: the optimum's
    /// true airspeed at the altitude it flies to (as it flies, where the
    /// tables give none), into `speed` and `reference` - what the checks judge
    /// and a pattern is planned with. The mode flies the optimum afresh as the
    /// altitude and weight change. Nothing without an optimisation.
    void optimise(double& speed, double& reference, double optimization, double altitudeM, double altitudeReference,
                  const sim::VehicleState& state) const noexcept;
    /// An hsa's speed and altitude against the aircraft's performance: each
    /// held to it and logged - clamped (kClamped), or with Reject a
    /// PerformanceLimit finding naming the field and the limit.
    void limitHsa(HsaCommand& c, CheckLog& log) const noexcept;
    /// A speed and an altitude in their references against the performance,
    /// as limitHsa: named `speedIndex` or `altitudeIndex` (a field, or a
    /// waypoint - then `speedField` and `altitudeField`, its fields).
    void limitFlight(double& speed, double speedReference, double& altitude, double altitudeReference, CheckLog& log, std::int16_t speedIndex,
                     std::int16_t altitudeIndex, std::int16_t speedField = -1, std::int16_t altitudeField = -1) const noexcept;
    /// A route's options and waypoints (docs/vehicle-interface.md, 4.5 and
    /// 5.1), into the scratch plan: the options whole and in range
    /// (InvalidParameter), the waypoints completed (InvalidWaypoint); then,
    /// unless the range policy is None, each point's speed, altitude, bank and
    /// climb rate against the performance, the route planned from where the
    /// aircraft is, every fly-by turn too big for its legs, and every gradient
    /// steeper than the aircraft climbs - logged: clamped (the turn flown
    /// smaller, the gradient at its climb rate; kClamped) or, with Reject,
    /// findings (InvalidWaypoint, PerformanceLimit) naming each point. The
    /// malformed are returned at once, the point in the log's result. Its
    /// loiters (docs/flight-autonomy.md, 4.31) into the plan too: 16 at most,
    /// each at its loiter point (route::loiterFault), checked as a pattern
    /// NEW (checkLoiter), completed where it flies (completeLoiters) and,
    /// checked, limited as a pattern is (limitPattern at its point).
    Reason checkRoute(RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log,
                      const RouteExtras* extras = nullptr);
    /// A must fly's location laid out as a route from where the aircraft is (4.42; MustFly.cpp): its fields whole and in
    /// range, its location found - a point, another vehicle (a point in its own frame), an operational point the session
    /// keeps (UnknownGeometry) - the points it approaches through where it has a window of bearings to come from, then the
    /// location, flown over; checked as a route's (checkRoute), into the scratch plan.
    Reason prepareMustFly(MustFlyCommand& c, const sim::VehicleState& state, CheckLog& log, const MustFlyArea* given);
    /// A zone given with a must fly checked (InvalidParameter, its field from 10 in `detail`) and laid out into `out` as it is
    /// now: its frame where it is in one (4.43; MustFly.cpp).
    Reason layOutZone(const OpZone& zone, MustFlyArea& out, CommandResult& detail) const;
    /// A zone's must fly laid out as a route into it (4.43; MustFly.cpp): its nearest point to the aircraft - or, with a
    /// window of bearings, its edge that way from its centre - and a little further in, at an altitude within its band.
    Reason enterZone(const MustFlyCommand& c, MustFlyArea& area, double ingressMin, double ingressMax, const sim::VehicleState& state, CheckLog& log);
    /// A line given with a must fly checked and laid out, as a zone is (4.44; MustFly.cpp).
    Reason layOutLine(const OpLine& line, MustFlyArea& out, CommandResult& detail) const;
    /// A corridor's must fly laid out as a route through it (4.44; MustFly.cpp): onto its first segment from behind its first
    /// vertex, then every vertex, the last flown over; each turn checked against its widths (PerformanceLimit, by point).
    Reason flyLine(const MustFlyCommand& c, MustFlyArea& line, double ingressMin, double ingressMax, const sim::VehicleState& state, CheckLog& log);
    /// A volume given with a must fly checked and laid out, as a zone is (4.45; MustFly.cpp).
    Reason layOutVolume(const OpVolume& volume, MustFlyArea& out, CommandResult& detail) const;
    /// A volume's must fly laid out as a route into it (4.45; MustFly.cpp): at a height inside it over its inner point, toward
    /// that point from the aircraft - or from within a window of bearings - to its edge, and a little further in.
    Reason enterVolume(const MustFlyCommand& c, MustFlyArea& volume, double ingressMin, double ingressMax, const sim::VehicleState& state, CheckLog& log);
    /// A must fly's NEW or UPDATE with its zone, corridor or volume laid out (MustFly.cpp).
    CommandResult submitLaidOut(const MustFlyCommand& mustFly, const MustFlyArea& area, const CommandOptions& options, const sim::VehicleState& state,
                                double now);
    CommandResult updateLaidOut(ActivityId activity, const MustFlyCommand& mustFly, const MustFlyArea& area, const sim::VehicleState& state,
                                Caller caller);
    /// A must fly's speed (true, m/s) as it will approach - its own, or the aircraft's (a rotorcraft's cruise) - and the radius
    /// of the turn onto its last leg at it, the wind behind it.
    double approachSpeed(const MustFlyCommand& c, const sim::VehicleState& state, double altitudeMslM) const noexcept;
    double approachRadius(const MustFlyCommand& c, const sim::VehicleState& state, double altitudeMslM) const noexcept;
    /// A route's arrival windows as given (4.33; Arrival.cpp), its first lap's: one past refused invalid; one at or after
    /// a loiter point, or on an aircraft without the tables its speeds come from, not implemented - the point named.
    Reason checkArrivals(const route::Plan& plan, const sim::VehicleState& state, CommandResult& detail) const noexcept;
    /// Its windows and its states' times against what the aircraft can make, planned from where it is, in calm air (4.33,
    /// 4.34), each in turn along its first lap from the times it can make the ones before: a finding (PerformanceLimit)
    /// where even the fastest it flies level arrives after one (MaxAirspeed), or a wing's slowest before one
    /// (MinAirspeed) - no clamp mends either.
    void limitArrivals(route::Plan& plan, const sim::VehicleState& state, CheckLog& log) const noexcept;
    /// A route's planned states as given (4.34; States.cpp), into the plan placed and complete: 64 at most, each on a
    /// segment its first lap flies, in order, its fields finite, with a place (in a frame, where the frame is at its
    /// time), its altitude in its point's reference, a time not past and after the one before; an altitude beside a climb
    /// rate or optimisation - else invalid. At or after a loiter point, on a route with a point in a moving frame, or a
    /// time without the tables its speeds come from, not implemented. The point named.
    Reason checkStates(route::Plan& plan, Span<const RouteState> states, const sim::VehicleState& state, CommandResult& detail) const;
    /// Its states on their legs as planned from where the aircraft is (4.34): one off its leg by more than its
    /// uncertainty (50 m at least, or 1 % of the leg), or behind the one before, invalid - the point named; a piece of the
    /// profile through their altitudes steeper than the aircraft climbs or descends, a finding (PerformanceLimit) no
    /// clamp mends.
    Reason limitStates(route::Plan& plan, const sim::VehicleState& state, CheckLog& log) const noexcept;
    /// What goes beside a route's waypoints, kept as it waits (`extras`, null for none; room reserved for the path store's):
    /// its loiters, states, paths, branches and terminators (4.31, 4.34, 4.36, 4.37, 4.38; Branches.cpp), none of its
    /// branches commanded.
    void holdExtras(Waiting& w, const RouteExtras* extras) const;
    /// A route kept waiting (disabled, unassigned: 4.10), resumed at point `start` (-1: from its own): the states beyond it
    /// along its flight (a linked route's order: 4.36), those it flew past aside (`passed`: Reset gives them back - 4.34).
    void keepStates(Waiting& w, const PathStore& store, double start) const;
    /// The flying route reset (4.10), its `c` from its first start again: the states it resumed past back ahead of those
    /// it kept (4.34), a linked one's flight order from there (4.36; Paths.cpp). The path store rewritten where it changes.
    void resetRoute(const RouteCommand& c) noexcept;
    /// A linked route's flight order into the plan (docs/flight-autonomy.md, 4.36; Paths.cpp), `count` points given - its
    /// paths checked (route::pathFault), from its start along each point's next (route::flightOrder), its points reordered:
    /// those it flies first, in order, the rest after. A route with neither paths nor a point's next is left as it is.
    /// InvalidWaypoint naming the point for paths that do not tile the points, a next that is none, round one point.
    Reason linkRoute(route::Plan& plan, const RouteCommand& route, Span<const RoutePath> paths, std::uint32_t count, CommandResult& detail) const noexcept;
    /// What the checks after a linked route's flight order found, named by position, named by point as given (4.36): the
    /// answer's (where no finding named one before), and the findings and adjustments from those counts on.
    void nameAsGiven(const route::Plan& plan, CheckLog& log, int findingsFrom, int adjustmentsFrom, bool indexed) const noexcept;
    /// The route the plan holds as given (4.36): its points in their order as given, its loiters (unplaced) and states named
    /// by them, its paths, branches and terminators - a suggestion's, a waiting one's.
    void givenRoute(std::vector<Waypoint>& points, std::vector<RouteLoiter>& loiters, std::vector<RouteState>& states,
                    std::vector<RoutePath>& paths, std::vector<RouteBranch>& branches, std::vector<RouteTerminator>& terminators) const;
    /// A route's conditional branches checked and kept in the plan (docs/flight-autonomy.md, 4.37; Branches.cpp), `plan`
    /// holding its paths and `waypoints` as given: InvalidWaypoint naming a branch's point for one that is none - 17 or more,
    /// at a point it has not, a next that is none or its own point, a condition malformed, a flight on from it that is none
    /// (round one point: `repeat` the route's); NotImplemented for a mission critical or lost comms contingency (FA-16).
    Reason checkBranches(route::Plan& plan, Span<const Waypoint> waypoints, Span<const RouteBranch> branches, bool repeat,
                         CommandResult& detail) const noexcept;
    /// A route's civil path terminators checked and kept in the plan (docs/flight-autonomy.md, 4.38; Terminators.cpp),
    /// `waypoints` as given: InvalidWaypoint naming the point for data that is none - 65 or more, at a point it has not,
    /// two for one point, a field not finite or a code not whole, data its point's leg has none of, a course to fix without
    /// its course, a radius to fix without its centre or its way round - and for a leg its segment does not define (a
    /// navaid's, a procedure turn's); for a manual termination nothing ends but at a route's last point, and a hold's
    /// terminator at no hold (FA-6f2b).
    Reason checkTerminators(route::Plan& plan, Span<const Waypoint> waypoints, Span<const RouteTerminator> terminators, bool repeat,
                            CommandResult& detail) const noexcept;
    /// Its terminators' legs as laid out (4.38; Terminators.cpp; `plan` planned, as 4.30's turn points are checked):
    /// InvalidWaypoint naming the point for a leg that is none - an arc where the route never flies it from the point
    /// before (its entry's only lap, a lap back to a later point, a branch taken to it), its ends in a moving frame, after
    /// a loiter not left at its point, off its circle, sweeping more than 170 degrees, what else it gives not its own, not
    /// its start turn's; a course to fix whose point before is past it; after a start turn point anything but an arc,
    /// after a capture anything but a track or a course to fix on its course. An arc the aircraft cannot turn, a finding.
    Reason checkLegs(const route::Plan& plan, CheckLog& log) const noexcept;
    /// Its later laps' legs where they differ from its first's (docs/flight-autonomy.md, 4.36; Laps.cpp; `plan` planned, its
    /// first lap checked): the arcs from a start turn at the point it loops back to, its course left out, checked as the first
    /// lap's are (4.30) - InvalidWaypoint naming the point an arc reaches for one sweeping more than 170 degrees or other than
    /// its radius given, and naming an end whose course is not its arc's there; an arc the aircraft cannot turn a finding, the
    /// turn after them flown smaller than its legs allow as the first lap's, a gradient steeper than it climbs flown at its
    /// rate - what the first lap's check found not found again.
    Reason checkLaps(route::Plan& plan, const sim::VehicleState& state, double windMs, CheckLog& log, CommandResult& detail) const noexcept;
    /// The leg into point k at the world's time `timeS` (docs/flight-autonomy.md, 4.29, 4.34; Moving.cpp; `plan` placed now):
    /// its ends where their frames will be then - a moving frame's at its velocity, a vehicle's where its velocity now carries
    /// it; a point in no frame, or a fixed one, where it is - the route's start from where the aircraft is.
    route::Leg legAt(const route::Plan& plan, std::uint32_t k, double timeS, const sim::VehicleState& state) const noexcept;
    /// The most a route's segment accelerates from `fromMs` to `toMs` (4.32): a rotorcraft's (Performance), a wing's from
    /// its tables at the fuel on board - full power's excess faster, idle's slower, the least over the speeds between - as
    /// a rate; NaN where not known.
    double accelerationLimit(double fromMs, double toMs, double altitudeMslM, const sim::VehicleState& state) const noexcept;
    /// A route's loiter's pattern and shape (4.31) as a pattern NEW's are checked: InvalidWaypoint for a field
    /// checkPattern or checkShape refuses, or a hover's (route::hoverFault); a hover where the aircraft does not hover,
    /// and an optimisation it has no tables for, as the pattern's (NotSupported, NotImplemented). The caller names the point.
    Reason checkLoiter(const RouteLoiter& loiter) const noexcept;
    /// The plan's loiters completed where they fly (route::completeLoiters; `p` planned): an optimisation's speed at
    /// its point's altitude first, as a pattern's (optimise).
    void completeLoiters(route::Plan& p, const sim::VehicleState& state) const noexcept;
    /// A route's loiter's fields in a finding or an adjustment (4.31): named by its point, numbered from 100 - past the
    /// waypoint's own, room left for them - its pattern's then its shape's (100 to 134), its end time 135.
    static constexpr std::int16_t kLoiterField = 100;
    /// The route checkRoute left in the scratch plan, into the path store: flown afresh.
    void writeRoute();
    /// A pattern's fields (docs/vehicle-interface.md, 4.6): whole numbers for
    /// its kind, references and way round, the rest finite and in range (a
    /// radius and a speed above 0, legs from 0, a duration above 0); in an
    /// UPDATE (`merge`) a reference needs its value. InvalidParameter with the field.
    static Reason checkPattern(const PatternCommand& c, bool merge, CommandResult& detail) noexcept;
    /// A pattern's shape (docs/flight-autonomy.md, 4.23), its fields numbered after the PatternCommand's: its
    /// direction reference whole, a heading finite, a leg time from 0, a bank above 0 and below a right angle, laps
    /// whole from 0, points whole (a latitude with its longitude), a second radius above 0; in an UPDATE (`merge`) a
    /// direction reference needs a course or a heading. InvalidParameter with the field.
    static Reason checkShape(const PatternCommand& c, const PatternShape& shape, bool merge, CommandResult& detail) noexcept;
    /// A complete pattern against the performance, as limitHsa: its speed and
    /// altitude, and a radius no tighter than the aircraft's full bank flies
    /// at its speed (a rotorcraft's: a metre) - both circles' - or, the radius
    /// from a bank or a turn rate, one it can fly; named by `radiusFrom`, the
    /// field the radius came from (radiusField). A route's loiter's at
    /// `point` (4.31): named by the point, its field after kLoiterField.
    void limitPattern(PatternCommand& c, PatternShape& shape, CheckLog& log, std::int16_t radiusFrom = 5, std::int16_t point = -1) const noexcept;
    /// The field a pattern's radius comes from, as completion takes them: its own (5), else its shape's bank (16),
    /// turn rate (25) or turn type (26); none: its own, filled in by default.
    static std::int16_t radiusField(const PatternCommand& c, const PatternShape& shape) noexcept {
        return !isHold(c.radiusM) ? 5 : !isHold(shape.bankRad) ? 16 : !isHold(shape.turnRateRadS) ? 25 : !isHold(shape.turnType) ? 26 : 5;
    }
    /// A pattern's fields filled in with its shape's (route::completePattern) where the aircraft is, in the wind it
    /// flies in now, at the world's date.
    void completePattern(PatternCommand& c, PatternShape& shape, const sim::VehicleState& state) const noexcept;
    /// A live pattern's UPDATE in slot `s` (Patterns.cpp): `next` and its `shape`, if given, merged into what flies,
    /// completed and checked; then written, the shape into the path store.
    CommandResult updatePattern(std::size_t s, ActivityId activity, const PatternCommand& next, const PatternShape* shape, const sim::VehicleState& state,
                                CommandResult& result, CheckLog& log) noexcept;
    /// A live must fly's UPDATE in slot `s` (4.42; MustFly.cpp): `next`'s fields given merged into what flies, laid out
    /// afresh from where the aircraft is and checked; then written, its route into the path store, flown afresh.
    CommandResult updateMustFly(std::size_t s, ActivityId activity, const MustFlyCommand& next, const sim::VehicleState& state, CommandResult& result,
                                CheckLog& log, const MustFlyArea* given = nullptr) noexcept;
    /// The scratch shape into the path store, for the pattern that flies: where it has one, or one flew before - its
    /// frame with it.
    void writeShape();
    /// Where a pattern is (docs/flight-autonomy.md, 4.25): a hover only where the aircraft hovers (the support table's
    /// reason, the pattern field named), and given nothing that shapes a circuit (route::hoverFault, of what was given:
    /// `given` and `givenShape` in an UPDATE, whose merge's other fields are the pattern's it was, cleared); a point in a
    /// frame the session has, resolved where it is now into the pattern's latitude and longitude (and its altitude,
    /// given a z) and its frame into the scratch - an unknown one InvalidParameter, the frame field named.
    Reason placePattern(PatternCommand& c, PatternShape& shape, CommandResult& detail, const PatternCommand* given = nullptr,
                        const PatternShape* givenShape = nullptr) noexcept;
    /// A hover pattern's support here (4.25): None, or its row's NotSupported or NotImplemented.
    Reason hoverSupport() const noexcept;
    /// A curve's options and segments (docs/vehicle-interface.md, 4.7 and 5.1):
    /// the options whole and finite (InvalidParameter); in a NEW, the
    /// reference where the aircraft is if left out; 1 to 10 segments, finite,
    /// each starting within a metre of where the one before ends (appended:
    /// where the curve ends), a metre long over the ground, and room in the
    /// store (InvalidCurve) - returned at once; each well formed, and turning
    /// no tighter than its curvature, given (4.26: InvalidCurve with the
    /// section). Then, unless the range policy
    /// is None, logged: a speed range the aircraft can fly within (clamped, or
    /// PerformanceLimit), every section a wing's full bank cannot turn at the
    /// fastest it flies (InvalidCurve, whatever the policy, with the section),
    /// every segment steeper than it climbs (clamped, or PerformanceLimit, with the section).
    Reason checkCurve(CurveCommand& c, Span<const NurbsSegment> segments, bool appending, const sim::VehicleState& state, CheckLog& log);
    /// A NEW curve's reference (docs/flight-autonomy.md, 4.27), from the scratch curveShape_: its frame's fields whole
    /// and in range, offsets only with a frame (InvalidParameter, fields 14 to 19); its points turned only with a frame
    /// (field 11); in a frame the session has, placed where the frame is now (its frame, turn and pose into the scratch;
    /// an unknown one InvalidParameter, field 14); left out, where the aircraft is; its altitude, left out, the
    /// aircraft's in its reference, held within its range - given, within it (field 2).
    Reason placeCurve(CurveCommand& c, const sim::VehicleState& state, CommandResult& detail) noexcept;
    /// The turn of a curve's axes now (4.27): its frame's yaw or track, as its pointRotation says; 0 unturned.
    double curveTurn(const CurveCommand& c, const CurveShape& shape) const noexcept;
    /// A curve's points turned in three dimensions (4.27: ROTATION_3D): true, its frame's pose now into `pose`.
    bool curveAttitude(const CurveCommand& c, const CurveShape& shape, FramePose& pose) const noexcept;
    /// A curve's options whole and finite, as checkCurve: InvalidParameter with the field.
    Reason checkCurveOptions(const CurveCommand& c, bool appending, CommandResult& detail) const noexcept;
    /// Its speed range one the aircraft can fly within, as checkCurve.
    void limitCurveSpeeds(CurveCommand& c, CheckLog& log) const noexcept;
    /// The fastest a wing flies at `altitudeM` - its envelope's calibrated
    /// speed and Mach there, and its profile's airspeed - or a rotorcraft over
    /// the ground; NaN if nothing limits it.
    double fastest(double altitudeM) const noexcept;
    /// The segments into the path store: after the curve's end, or a new curve.
    void writeCurve(Span<const NurbsSegment> segments, bool appending);
    /// NEW: a command (with a route's waypoints, a curve's segments). One that
    /// may not wait (the existing entry points'): refused where it would.
    CommandResult submitWith(const Command& command, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments, const CommandOptions& options,
                             const sim::VehicleState& state, double now, bool mayWait = true, const PatternShape* shape = nullptr,
                             const CurveShape* curveShape = nullptr, const RouteExtras* extras = nullptr);

    /// A capability's standing with the vehicle's policy (6.2, 7.2).
    struct Authority {
        bool allowed = true, granted = false;
        ControllerId holder = 0;       ///< the controller whose grant it is, while granted
        std::uint32_t precedence = 0;  ///< the platform's precedence for it (setPrecedence; beside what a NEW reads first)
        CapabilityStatus restricted{}; ///< the platform's restriction (setAvailability)
    };
    /// The vehicle's own availability for a capability, apart from the platform's restrictions.
    CapabilityStatus vehicleStatus(std::size_t capability, const sim::VehicleState& state) const noexcept;
    /// The flight phase's answer to a policy (docs/flight-autonomy.md, 4.5):
    /// OnGround for the platform's airborne guidance on the ground, else None.
    /// FA's own sources and running activities never meet it.
    static Reason phase(const CapabilityDescriptor& d, const sim::VehicleState& state) noexcept;
    /// Why a command for a feature the catalog does not offer is refused.
    Reason missing(std::string_view feature) const noexcept;
    /// Its authority, the table grown with the catalog.
    Authority& authorityOf(std::size_t capability);
    /// Why `source` (a policy's `controller`) may not command the capability
    /// now - no grant of its own under Granted, the platform's restriction -
    /// or Reason::None. The platform's own sources are never stopped here.
    Reason admits(std::size_t capability, Source source, ControllerId controller = 0) const noexcept;
    /// The policy's live activities of the capability - flying or waiting;
    /// `only` one controller's, if given - end Canceled with `reason`; what waited may start.
    void endPolicy(std::size_t capability, Reason reason, const sim::VehicleState& state, double now, const ControllerId* only = nullptr) noexcept;
    /// Whether `caller` may UPDATE or CANCEL a live activity (flying or
    /// waiting): any caller under Open (ADR-26 10.1); under Granted a source no
    /// lower than the activity's - FA stays the primary controller - and, a
    /// policy's, only its own controller's. Else AuthorityHeld.
    Reason addresses(const ActivityRecord& record, Caller caller) const noexcept;

    std::uint32_t vehicle_ = 0;
    /// A plan has been prepared for upload: plans_ is made (4.39). Here, in the hole before controlPeriodS_, on the line
    /// every NEW reads: noteEnd tests it at each activity's end, where plans_, at the host's far end, would cost a line.
    bool planned_ = false;
    double controlPeriodS_ = 1.0 / 120.0;
    EnvelopeStatus envelope_{}; ///< since the last envelope()
    ControlStack* runtime_ = nullptr;
    // the existing entry points' per-step path (updateLegacy), together
    RuntimeConfig* config_ = nullptr; ///< the runtime's, held for the fast path
    ActivityId legacy_ = 0;           ///< the activity the existing entry points command
    int legacySlot_ = -1;             ///< its slot while it is live
    const CapabilityCatalog* catalog_ = nullptr;
    const VehicleAdapter* adapter_ = nullptr;
    const VehicleProfile* profile_ = nullptr;
    const SupportTable* support_ = nullptr;
    Performance performance_{};
    std::unique_ptr<route::Plan> routePlan_; ///< a route's scratch, allocated at the vehicle's first route
    std::unique_ptr<route::Curve> curvePlan_; ///< a curve's scratch, allocated at the vehicle's first curve
    std::uint32_t serial_ = 0;
    std::array<Slot, kActivities> slots_{};
    std::array<ActivityRecord, kActivities> records_{}; ///< per slot: its activity's record
    std::array<ActivityRecord, kRecent> recent_{};      ///< ended records, a ring
    std::size_t recentNext_ = 0, recentCount_ = 0;
    std::size_t waitingCount_ = 0;                      ///< activities waiting to start, or disabled
    std::uint64_t queueSerial_ = 0;                     ///< counts entries into the waiting store
    std::vector<Task> tasks_;                           ///< the flight tasks kept (4.11), in the order they were made
    std::uint64_t suggestionSerial_ = 0;                ///< numbers the platform's suggestions
    std::size_t pendingSuggestions_ = 0;                ///< waiting entries kept as suggestions, not yet tasks
    std::size_t windowed_ = 0;                          ///< live activities with an end window, flying
    std::vector<Authority> authority_;                  ///< per capability (sized at bind)
    ControlMode controlMode_ = ControlMode::Open;
    std::uint32_t controlRevision_ = 0;
    std::uint32_t loopsSeen_ = 0; ///< the runtime's loops revision performance_ is from
    bool divergedSeen_ = false;   ///< (a divergence changes every capability's availability: counted)
    CommandDetails details_{};    ///< the last answer's (details())
    std::unique_ptr<std::array<Waiting, kWaiting>> waiting_; ///< made when the first activity waits
    const SessionView* sessionView_ = nullptr; ///< the session's (setSessionView)
    PatternShape patternShape_{}; ///< a pattern's shape as prepare() completed it: its scratch, as routePlan_ is a route's
    FrameSpec patternFrame_{};    ///< its frame's, where its point is one's
    CurveShape curveShape_{};     ///< a curve's reference in a frame, as prepare() or an UPDATE merged it (4.27)
    FrameSpec curveFrame_{};      ///< its frame's, where its reference is one's
    double curveTurn_ = 0.0;      ///< its axes' turn as placed
    FramePose curvePose_{};       ///< its frame's pose as placed: its points turned in three dimensions by its attitude

    // route plans (docs/flight-autonomy.md, 4.39): Plans.cpp's, apart from the rest, last so that nothing moves
    struct PlanStore;
    struct PlanStoreFree {
        void operator()(PlanStore* store) const noexcept;
    };
    struct PlanEntry;
    /// An activity ended: the plan it flew (if any) takes its end - in the step, allocating nothing.
    void notePlanEnd(const ActivityRecord& record) noexcept;
    PlanEntry* findPlan(PlanId id) const noexcept;
    PlanStatus planStatusOf(const PlanEntry& e) const noexcept;
    /// Its live activity ended Canceled with `reason`, the platform's: no authority asked.
    void endPlanActivity(ActivityId activity, Reason reason, const sim::VehicleState& state, double now) noexcept;
    /// The wind a route's checks turn in (4.41): a plan validation's while one runs, else what the air data measure now.
    WindEstimate checkWind(const sim::VehicleState& state) const noexcept;
    std::unique_ptr<PlanStore, PlanStoreFree> plans_; ///< made at the first plan prepared for upload
};

} // namespace fsim::control
