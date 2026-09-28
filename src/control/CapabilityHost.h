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

namespace route {
struct Plan;
struct Curve;
}

class SupportTable;

/// Where the support effectors are, for the activities that complete when they get there.
struct EffectorPositions {
    double gear = kUnknown;  ///< 0 up .. 1 down
    double flaps = kUnknown; ///< 0 .. 1
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
    CommandResult submit(const RouteCommand& route, Span<const Waypoint> waypoints, const CommandOptions& options, const sim::VehicleState& state,
                         double now);
    /// NEW of a curve (fsim.guidance.curve; docs/vehicle-interface.md 4.7):
    /// its segments checked against the aircraft (InvalidCurve naming the
    /// segment, and a section too tight), then written into the path store.
    /// A CurveCommand submitted as a Command has none: InvalidCurve.
    CommandResult submit(const CurveCommand& curve, Span<const BezierSegment> segments, const CommandOptions& options, const sim::VehicleState& state,
                         double now);
    /// UPDATE: a new setpoint for a live activity - the fast path; allocates
    /// nothing. `state` is the vehicle's: a route is planned afresh from it.
    /// `caller` is the source the caller declares, as a NEW's options do, and
    /// a policy's controller: under ControlMode::Granted one below the
    /// activity's may not address it (AuthorityHeld, naming it) - a policy
    /// cannot change or end what the platform's own sources fly - nor may a
    /// controller another's. Open, as ADR-26 10.1: any caller.
    CommandResult update(ActivityId activity, const Command& setpoint, const sim::VehicleState& state, Caller caller) noexcept;
    CommandResult update(ActivityId activity, const SupportCommand& setpoint, Caller caller) noexcept;
    /// UPDATE of a route: its options (a field left out, kHold, keeps its
    /// value) and its waypoints - none: those it has - checked as a NEW's,
    /// then flown afresh from its start, from where the aircraft is.
    CommandResult update(ActivityId activity, const RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state,
                         Caller caller) noexcept;
    /// UPDATE of a curve: the options given (kHold keeps one), and segments -
    /// with `append` 1, after its end, from the same reference; else a new
    /// curve, flown afresh. Options alone change how it is flown, not where.
    CommandResult update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments, const sim::VehicleState& state,
                         Caller caller) noexcept;
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
    Reason storeTask(TaskId id, const Command& command, Span<const Waypoint> waypoints, Span<const BezierSegment> segments, TaskRepetition repetition);
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

    // --- Reports (docs/flight-autonomy.md, 4.12): what an activity flies, and where to ---
    /// What a live activity flies now, or waits to fly: its setpoint as
    /// updated, a route's waypoints, a curve's segments (appended ones too).
    /// False for one not live.
    bool setpoint(ActivityId activity, Setpoint& out) const;
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
        std::vector<BezierSegment> segments; ///< a curve's (likewise)
        std::uint64_t queued = 0;           ///< when it began to wait (queueSerial_): its place among equals
        double firstStart = kHold;          ///< a route's start as commanded (Reset: kept out of its slot, it resumed elsewhere)
        /// A waiting activity that failed as it would start, kept as the
        /// suggestion its record names (4.11) until a call makes it a task
        /// (what a start in a step may not allocate).
        bool suggested = false;
        TaskId suggestion = 0;
        bool resumed = false; ///< it flew before (disabled, unassigned): its start window was its first start's
    };
    /// A flight task (4.11): its command, and what became of it.
    struct Task {
        TaskId id = 0;
        bool suggested = false;             ///< the platform's
        Command command{};
        std::vector<Waypoint> waypoints;
        std::vector<BezierSegment> segments;
        TaskRepetition repetition{};
        ActivityId activity = 0;            ///< its activity (every run's), while it is commanded
        std::uint64_t commandId = 0;        ///< its task command's
        TaskState ended = TaskState::AwaitingExecution; ///< once its activity ended (or it was canceled before it was commanded)
        Reason reason = Reason::None;
        std::uint32_t run = 0, runs = 0;    ///< its activity's, as it ended
        double percent = kUnknown;          ///< likewise
        double startTime = kUnknown, endTime = kUnknown;
    };
    Task* findTask(TaskId id) noexcept;
    TaskStatus statusOf(const Task& t) const noexcept;
    /// An activity ended: the task it flew (if any) takes its end - in the step, allocating nothing.
    void noteEnd(const ActivityRecord& record) noexcept;
    /// The platform's suggestion (4.11): a task with the command the checks
    /// left, every value held to its limit (a route's points as planned). Its id.
    TaskId suggest(const Command& setpoint, Span<const Waypoint> waypoints, Span<const BezierSegment> segments);
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
    Reason prepare(std::size_t index, Command& setpoint, Span<const Waypoint> waypoints, Span<const BezierSegment> segments, const sim::VehicleState& state,
                   CheckLog& log);
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
                Span<const BezierSegment> curve, double now) noexcept;
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
    CommandResult updateWaiting(Waiting& w, const Command& setpoint, Span<const Waypoint> waypoints, Span<const BezierSegment> segments,
                                const sim::VehicleState& state, Caller caller) noexcept;
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
    /// malformed are returned at once, the point in the log's result.
    Reason checkRoute(RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log);
    /// The route checkRoute left in the scratch plan, into the path store: flown afresh.
    void writeRoute();
    /// A pattern's fields (docs/vehicle-interface.md, 4.6): whole numbers for
    /// its kind, references and way round, the rest finite and in range (a
    /// radius and a speed above 0, legs from 0, a duration above 0); in an
    /// UPDATE (`merge`) a reference needs its value. InvalidParameter with the field.
    static Reason checkPattern(const PatternCommand& c, bool merge, CommandResult& detail) noexcept;
    /// A complete pattern against the performance, as limitHsa: its speed and
    /// altitude, and a radius no tighter than the aircraft's full bank flies
    /// at its speed (a rotorcraft's: a metre).
    void limitPattern(PatternCommand& c, CheckLog& log) const noexcept;
    /// A curve's options and segments (docs/vehicle-interface.md, 4.7 and 5.1):
    /// the options whole and finite (InvalidParameter); in a NEW, the
    /// reference where the aircraft is if left out; 1 to 10 segments, finite,
    /// each starting within a metre of where the one before ends (appended:
    /// where the curve ends), a metre long over the ground, and room in the
    /// store (InvalidCurve) - returned at once. Then, unless the range policy
    /// is None, logged: a speed range the aircraft can fly within (clamped, or
    /// PerformanceLimit), every section a wing's full bank cannot turn at the
    /// fastest it flies (InvalidCurve, whatever the policy, with the section),
    /// every segment steeper than it climbs (clamped, or PerformanceLimit, with the section).
    Reason checkCurve(CurveCommand& c, Span<const BezierSegment> segments, bool appending, const sim::VehicleState& state, CheckLog& log);
    /// A curve's options whole and finite, as checkCurve: InvalidParameter with the field.
    Reason checkCurveOptions(const CurveCommand& c, bool appending, CommandResult& detail) const noexcept;
    /// Its speed range one the aircraft can fly within, as checkCurve.
    void limitCurveSpeeds(CurveCommand& c, CheckLog& log) const noexcept;
    /// The fastest a wing flies at `altitudeM` - its envelope's calibrated
    /// speed and Mach there, and its profile's airspeed - or a rotorcraft over
    /// the ground; NaN if nothing limits it.
    double fastest(double altitudeM) const noexcept;
    /// The segments into the path store: after the curve's end, or a new curve.
    void writeCurve(Span<const BezierSegment> segments, bool appending);
    /// NEW: a command (with a route's waypoints, a curve's segments). One that
    /// may not wait (the existing entry points'): refused where it would.
    CommandResult submitWith(const Command& command, Span<const Waypoint> waypoints, Span<const BezierSegment> segments, const CommandOptions& options,
                             const sim::VehicleState& state, double now, bool mayWait = true);

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
};

} // namespace fsim::control
