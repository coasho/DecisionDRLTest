#pragma once

// The contract layer's per-vehicle half (docs/control-architecture.md,
// sections 9 and 10): takes commands (NEW, UPDATE, CANCEL) and answers them
// at once, arbitrates authority, keeps the activities and their records, and
// writes the runtime's configuration - between steps, on the caller's thread.
// After each world step it reads the runtime's report into its activities.

#include "control/Adapter.h"
#include "control/Catalog.h"
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
    /// Where activities live: the cascade's slots, one per support axis, then
    /// the engines' throttles (fsim.flight.engines, thrust beside the cascade).
    static constexpr std::size_t kEnginesSlot = kSlotCount + kSupportAxisCount;
    static constexpr std::size_t kActivities = kEnginesSlot + 1;

    /// The vehicle this host serves, its runtime, catalog, adapter and profile
    /// (which outlive it), and how often its runtime updates: protection as
    /// the profile gives it (docs/control-architecture.md, 11).
    void bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog, const VehicleAdapter& adapter,
              const VehicleProfile& profile, double controlPeriodS = 1.0 / 120.0) noexcept;

    /// NEW. `state` is the vehicle's, for availability; `now` the simulation time.
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
    /// `caller` is the source the caller declares, as a NEW's options do:
    /// under ControlMode::Granted one below the activity's may not address it
    /// (AuthorityHeld, naming it) - a policy cannot change or end what the
    /// platform's own sources fly. Open, as ADR-26 10.1: any caller.
    CommandResult update(ActivityId activity, const Command& setpoint, const sim::VehicleState& state, Source caller) noexcept;
    CommandResult update(ActivityId activity, const SupportCommand& setpoint, Source caller) noexcept;
    /// UPDATE of a route: its options (a field left out, kHold, keeps its
    /// value) and its waypoints - none: those it has - checked as a NEW's,
    /// then flown afresh from its start, from where the aircraft is.
    CommandResult update(ActivityId activity, const RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state,
                         Source caller) noexcept;
    /// UPDATE of a curve: the options given (kHold keeps one), and segments -
    /// with `append` 1, after its end, from the same reference; else a new
    /// curve, flown afresh. Options alone change how it is flown, not where.
    CommandResult update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments, const sim::VehicleState& state,
                         Source caller) noexcept;
    /// CANCEL: the activity ends and its axes return to the vehicle default
    /// (`caller` as for UPDATE).
    CommandResult cancel(ActivityId activity, double now, Source caller) noexcept;
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
    void setControlMode(ControlMode mode, double now) noexcept;
    ControlMode controlMode() const noexcept { return controlMode_; }
    /// A policy asks for control of a capability (A-GRA's ACQUIRE): granted -
    /// Reason::None - if it is allowed and available; else NotAllowed, or the
    /// reason it is unavailable. UnknownCapability for one it cannot command.
    Reason requestControl(std::size_t capability, const sim::VehicleState& state) noexcept;
    /// The policy lets go: its grant ends, and its live activities of the
    /// capability end Canceled(Released); their axes fly the vehicle default.
    Reason releaseControl(std::size_t capability, double now) noexcept;
    /// The platform takes it back: the grant ends, and the policy's live
    /// activities of the capability end Canceled with `reason`: Revoked (if
    /// None), CollisionAvoidance or Restricted; InvalidParameter for another,
    /// and nothing changes.
    Reason revokeControl(std::size_t capability, Reason reason, double now) noexcept;
    /// Whether the policy may request the capability (all may, by default); a
    /// grant for one no longer allowed is revoked.
    Reason setAllowed(std::size_t capability, bool allowed, double now) noexcept;
    ControlStatus controlStatus(std::size_t capability) const noexcept;
    /// The platform restricts a capability: a policy's NEW for it, and a
    /// request, are refused with `reason` - Restricted (if None),
    /// CollisionAvoidance or Unavailable; InvalidParameter for another - and
    /// live activities go on. Availability::Available lifts it (its reason unused).
    Reason setAvailability(std::size_t capability, Availability availability, Reason reason) noexcept;
    /// Counts every change to the grants, what is allowed, the control mode,
    /// availability and the performance (6.3): a consumer polls it.
    std::uint32_t controlRevision() const noexcept { return controlRevision_; }

    /// Live or recently ended; null if unknown.
    const ActivityRecord* activity(ActivityId activity) const noexcept;
    /// The live activities, then the ended ones it remembers, newest first.
    std::vector<ActivityRecord> activities() const;
    /// Its availability: the vehicle's own (a diverged vehicle), else the platform's restriction (setAvailability).
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

    /// After each world step: the runtime's report into the activities, then cleared.
    void afterStep(const sim::VehicleState& state, const EffectorPositions& positions, double now) noexcept;
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
    };

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
    /// A live activity of a higher source on any of `axes`, else 0.
    ActivityId holder(AxisMask axes, Source source) const noexcept;
    /// Before activity `id` takes `axes` (9.3): a live activity that loses a
    /// primary axis ends, preempted; one that loses only support axes carries
    /// on without them. What keeps a primary axis flies on - a residual hold
    /// if it ended - and a slot left without one is freed.
    void takeOver(AxisMask axes, ActivityId id, double now) noexcept;
    ActivityRecord& start(std::size_t slot, ActivityId id, std::size_t capability, const CommandOptions& options, AxisMask axes,
                          std::uint16_t flags, double now) noexcept;
    void end(std::size_t slot, ActivityState state, Reason reason, ActivityId by, double now) noexcept;
    void release(std::size_t slot) noexcept;
    CommandResult rejected(Reason reason, ActivityId activity = 0, ActivityId other = 0) const noexcept;
    /// The setpoint of the live hsa activity, if one flies (a NEW continues what it commanded).
    const HsaCommand* liveHsa() const noexcept;
    /// A NEW hsa (docs/vehicle-interface.md, 4.4): its references checked, the
    /// fields it leaves out from the live hsa it replaces or the state, its
    /// angles wrapped. InvalidParameter (with the field in `detail`) if malformed.
    Reason resolveHsa(HsaCommand& c, const sim::VehicleState& state, CommandResult& detail) const noexcept;
    /// An hsa's speed and altitude against the aircraft's performance: clamped
    /// (kClamped) or, with Reject, PerformanceLimit - `detail` saying which field and limit.
    Reason limitHsa(HsaCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept;
    /// A speed and an altitude in their references against the performance,
    /// as limitHsa: `detail` names `speedIndex` or `altitudeIndex` (a field, or a waypoint).
    Reason limitFlight(double& speed, double speedReference, double& altitude, double altitudeReference, RangePolicy range, std::uint16_t& flags,
                       CommandResult& detail, std::int16_t speedIndex, std::int16_t altitudeIndex) const noexcept;
    /// A route's options and waypoints (docs/vehicle-interface.md, 4.5 and
    /// 5.1), into the scratch plan: the options whole and in range
    /// (InvalidParameter), the waypoints completed (InvalidWaypoint); then,
    /// unless the range policy is None, each point's speed, altitude, bank and
    /// climb rate against the performance, the route planned from where the
    /// aircraft is, a fly-by turn too big for its legs, and a gradient steeper
    /// than the aircraft climbs - clamped (the turn flown smaller, the
    /// gradient at its climb rate; kClamped) or, with Reject, refused
    /// (InvalidWaypoint, PerformanceLimit). `detail` names the point.
    Reason checkRoute(RouteCommand& route, Span<const Waypoint> waypoints, const sim::VehicleState& state, RangePolicy range, std::uint16_t& flags,
                      CommandResult& detail);
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
    Reason limitPattern(PatternCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept;
    /// A curve's options and segments (docs/vehicle-interface.md, 4.7 and 5.1):
    /// the options whole and finite (InvalidParameter); in a NEW, the
    /// reference where the aircraft is if left out; 1 to 10 segments, finite,
    /// each starting within a metre of where the one before ends (appended:
    /// where the curve ends), a metre long over the ground, and room in the
    /// store (InvalidCurve). Then, unless the range policy is None: a speed
    /// range the aircraft can fly within (clamped, or PerformanceLimit), a
    /// section a wing's full bank cannot turn at the fastest it flies
    /// (InvalidCurve, whatever the policy, with the section), a gradient
    /// steeper than it climbs (clamped, or PerformanceLimit, with the section).
    Reason checkCurve(CurveCommand& c, Span<const BezierSegment> segments, bool appending, const sim::VehicleState& state, RangePolicy range,
                      std::uint16_t& flags, CommandResult& detail);
    /// A curve's options whole and finite, as checkCurve: InvalidParameter with the field.
    Reason checkCurveOptions(const CurveCommand& c, bool appending, CommandResult& detail) const noexcept;
    /// Its speed range one the aircraft can fly within, as checkCurve.
    Reason limitCurveSpeeds(CurveCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept;
    /// The fastest a wing flies at `altitudeM` - its envelope's calibrated
    /// speed and Mach there, and its profile's airspeed - or a rotorcraft over
    /// the ground; NaN if nothing limits it.
    double fastest(double altitudeM) const noexcept;
    /// The segments into the path store: after the curve's end, or a new curve.
    void writeCurve(Span<const BezierSegment> segments, bool appending);
    /// NEW: a command (with a route's waypoints, a curve's segments).
    CommandResult submitWith(const Command& command, Span<const Waypoint> waypoints, Span<const BezierSegment> segments, const CommandOptions& options,
                             const sim::VehicleState& state, double now);

    /// A capability's standing with the vehicle's policy (6.2, 7.2).
    struct Authority {
        bool allowed = true, granted = false;
        CapabilityStatus restricted{}; ///< the platform's restriction (setAvailability)
    };
    /// The vehicle's own availability for a capability, apart from the platform's restrictions.
    CapabilityStatus vehicleStatus(std::size_t capability, const sim::VehicleState& state) const noexcept;
    /// Its authority, the table grown with the catalog.
    Authority& authorityOf(std::size_t capability);
    /// Why `source` may not command the capability now - no grant under
    /// Granted, the platform's restriction - or Reason::None. The platform's
    /// own sources are never stopped here.
    Reason admits(std::size_t capability, Source source) const noexcept;
    /// The policy's live activities of the capability end Canceled with `reason`.
    void endPolicy(std::size_t capability, Reason reason, double now) noexcept;
    /// Whether `caller` may UPDATE or CANCEL slot s's live activity: any caller
    /// under Open (ADR-26 10.1); under Granted a source no lower than the
    /// activity's - FA stays the primary controller. Else AuthorityHeld.
    Reason addresses(std::size_t s, Source caller) const noexcept;

    std::uint32_t vehicle_ = 0;
    double controlPeriodS_ = 1.0 / 120.0;
    EnvelopeStatus envelope_{}; ///< since the last envelope()
    ControlStack* runtime_ = nullptr;
    RuntimeConfig* config_ = nullptr; ///< the runtime's, held for the fast path
    const CapabilityCatalog* catalog_ = nullptr;
    const VehicleAdapter* adapter_ = nullptr;
    const VehicleProfile* profile_ = nullptr;
    Performance performance_{};
    std::unique_ptr<route::Plan> routePlan_; ///< a route's scratch, allocated at the vehicle's first route
    std::unique_ptr<route::Curve> curvePlan_; ///< a curve's scratch, allocated at the vehicle's first curve
    std::uint32_t serial_ = 0;
    std::array<Slot, kActivities> slots_{};
    std::array<ActivityRecord, kActivities> records_{}; ///< per slot: its activity's record
    std::array<ActivityRecord, kRecent> recent_{};      ///< ended records, a ring
    std::size_t recentNext_ = 0, recentCount_ = 0;
    ActivityId legacy_ = 0;                             ///< the activity the existing entry points command
    int legacySlot_ = -1;                               ///< its slot while it is live
    std::vector<Authority> authority_;                  ///< per capability (sized at bind)
    ControlMode controlMode_ = ControlMode::Open;
    std::uint32_t controlRevision_ = 0;
    std::uint32_t loopsSeen_ = 0; ///< the runtime's loops revision performance_ is from
    bool divergedSeen_ = false;   ///< (a divergence changes every capability's availability: counted)
};

} // namespace fsim::control
