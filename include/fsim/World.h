#pragma once

// fsim SDK: the vehicle-instance API (design 9.2, ADR-18). A training
// application creates a World, creates vehicles by name/type/initial state,
// commands them at any control level and steps the world. Visualisation is
// transparent: every World publishes itself for flightsim-viewer.exe through
// shared memory (design 9.7) at no cost to stepping.

#include "fsim/Capability.h"
#include "fsim/Comm.h"
#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/ControlStack.h"
#include "fsim/Effects.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Export.h"
#include "fsim/Frames.h"
#include "fsim/GroundProvider.h"
#include "fsim/InitialConditions.h"
#include "fsim/PerformanceProfile.h"
#include "fsim/Property.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace fsim {

namespace session {
class World;
}

/// Thrown for load-time failures (unknown aircraft, bad type, duplicate name).
class Error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct WorldOptions {
    std::string name = "default";     ///< viewers attach by this name
    double dt = 1.0 / 120.0;          ///< FDM step, seconds
    int frameSkip = 4;                ///< FDM steps per World::step()
    unsigned workers = 0;             ///< simulation threads; 0 = automatic (physical cores - 2)
    bool pinWorkers = true;           ///< one worker per physical core; disable when running several trainers on one machine
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;     ///< vehicle slots visible to viewers
    bool publish = true;              ///< false = never visible to viewers
    double publishIntervalSeconds = 1.0 / 60.0;
    std::string jsbsimRoot;           ///< empty = auto-detect
    bool terrain = false;             ///< physics ground from public elevation tiles (the ones the viewer draws); needs network or a warm cache
    std::string terrainUrl;           ///< XYZ template with {z}/{x}/{y}; empty = AWS Terrarium
    unsigned terrainZoom = 12;        ///< 12: ~38 m/px, 14: ~10 m/px (4x the tiles)
    std::shared_ptr<GroundProvider> ground; ///< your own ground model instead (overrides `terrain`)
    std::string recordPath;           ///< non-empty: record every vehicle's state for `flightsim-viewer --replay`
    double recordIntervalSeconds = 0.0; ///< simulation time between frames; 0 = every world step
};

struct VehicleSpec {
    std::string name;                 ///< unique; empty = "<aircraft>-<id>"
    std::string type = "jsbsim:c172x";///< "<flight model>:<aircraft>"
    InitialConditions initial;
    std::string model;                ///< optional glTF for the viewer
    unsigned controlDivider = 1;      ///< control stack every N FDM steps
    /// Profile sections to replace for this vehicle (those present(); see
    /// docs/control-architecture.md, 7.4): how a stock aircraft gets an
    /// envelope, say. Null = the aircraft's own.
    std::shared_ptr<const control::VehicleProfile> profile;
};

/// Calendar UTC for Environment::setTime.
struct Utc {
    int year = 2026, month = 1, day = 1, hour = 12, minute = 0;
    double second = 0.0;
};
FSIM_API double utcToUnixSeconds(const Utc& utc) noexcept;

struct Wind {
    double directionDeg = 0.0;  ///< blowing FROM, degrees true
    double speedMs = 0.0;
    double gustMs = 0.0;
    double turbulence = 0.0;    ///< 0..1
};
struct Atmosphere {
    double temperatureSeaLevelK = 288.15;
    double pressureSeaLevelPa = 101325.0;
    double humidity = 0.0;
};
struct Weather {
    double visibilityM = 50000.0;
    double cloudBaseM = 2000.0;
    double cloudCover = 0.0;
    double precipitation = 0.0;
};

class World;

/// Real-time environment control (design 9.4). Setters take effect at the next step.
class FSIM_API Environment {
public:
    void setTime(const Utc& utc);
    void setTime(double unixSeconds);
    void setTimeFactor(double factor);
    void setWind(const Wind& wind);
    void setAtmosphere(const Atmosphere& atmosphere);
    void setWeather(const Weather& weather);
    /// Everything at once (advanced: fields not covered by the setters).
    void set(const EnvironmentState& state);
    const EnvironmentState& state() const noexcept;
    /// Current UTC as Unix seconds (epoch + simulation time).
    double utcSeconds() const noexcept;

private:
    friend class World;
    explicit Environment(World& world) noexcept : world_(&world) {}
    World* world_;
};

/// Lightweight handle to a vehicle instance; copyable, valid until removed.
class FSIM_API Vehicle {
public:
    Vehicle() = default;
    bool valid() const noexcept;
    std::uint32_t id() const noexcept { return id_; }
    std::string name() const;
    std::string type() const;
    std::string model() const;  ///< the VehicleSpec::model override (empty = the type's model)

    // State (references are valid until the vehicle is removed)
    const VehicleState& state() const;            ///< truth
    const effects::SensedState& sensed() const;    ///< through sensor effects
    const ControlInputs& inputs() const;          ///< actuator inputs the cascade produced

    // Multi-level control (design 9.3): the per-step command. At the level
    // the vehicle's own activity flies it updates that activity, else it
    // starts a new one (docs/control-architecture.md, 10.7); false if the
    // command is rejected (a behaviour nobody registered, an axis an autopilot
    // or override activity holds).
    bool command(const control::Command& command);
    template <typename C>
    bool command(const C& c) { return command(control::Command(c)); }
    // Capability contracts (docs/sdk/control.md, "Capabilities and activities"):
    /// NEW: the command becomes an activity, or is rejected with a reason.
    control::CommandResult submit(const control::Command& command, const control::CommandOptions& options = {});
    /// NEW for a support effector (GearCommand, FlapsCommand, WheelBrakesCommand,
    /// SpeedbrakeCommand, PitchTrimCommand) or the engines' throttles (EnginesCommand).
    control::CommandResult submit(const control::SupportCommand& command, const control::CommandOptions& options = {});
    template <typename C>
    control::CommandResult submit(const C& c, const control::CommandOptions& options = {}) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return submit(control::SupportCommand(c), options);
        else
            return submit(control::Command(c), options);
    }
    /// NEW of a route (fsim.guidance.route, A-GRA's waypoint following;
    /// docs/sdk/control.md, "Routes"): at most PathStore::kWaypoints waypoints,
    /// each checked (InvalidWaypoint, PerformanceLimit: CommandResult::index
    /// names the point), then flown from where the aircraft is. Its loiter
    /// points' loiters beside them (docs/flight-autonomy.md, 4.31), its
    /// planned states (4.34): the segments fly through them, its paths
    /// (4.36): flown along its points' links, each point named as given, its
    /// conditional branches (4.37): taken as their conditions say, and its
    /// civil path terminators' data (4.38): each point's leg flown as its
    /// terminator (Waypoint::terminator) says.
    control::CommandResult submit(const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  const control::CommandOptions& options = {}, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    /// NEW of a curve (fsim.guidance.curve, A-GRA's curve following;
    /// docs/sdk/control.md, "Curves"): 1 to 10 quintic Bezier segments, each
    /// starting where the one before ends, checked (InvalidCurve names the
    /// segment, and a section too tight in CommandResult::from and to). Or its
    /// segments as A-GRA's schema gives them: clamped rational B-splines
    /// (docs/flight-autonomy.md, 4.26).
    control::CommandResult submit(const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    control::CommandResult submit(const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    /// NEW of a pattern (fsim.guidance.pattern) with its shape - A-GRA's orbit as
    /// its schema gives it: two circles, an inbound heading, legs by time, turns
    /// by bank, laps, entry and exit points (docs/sdk/control.md, "Patterns";
    /// docs/flight-autonomy.md, 4.23). A shape's field at fault is named by its
    /// index after the pattern's 13.
    control::CommandResult submit(const control::PatternCommand& pattern, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    /// NEW of an altitude stacked marshall with its pattern's shape (docs/flight-autonomy.md, 4.46): its slot chosen by the
    /// world - the lowest of its stack clear of every other aircraft marshalling round the same point, or the one asked for -
    /// and read back in its altitude; StackFull where none is. A racetrack's or a figure-eight's second circle, and laps, in
    /// the shape. Flown as its pattern, its stack beside it: its setpoint's `marshall` reads it back.
    control::CommandResult submit(const control::MarshallCommand& marshall, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    /// NEW of a must fly with its zone given (docs/flight-autonomy.md, 4.43): the zone checked (a field at fault named from
    /// 10, after the must fly's own) and laid out as it is given, then entered.
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpZone& zone, const control::CommandOptions& options = {});
    /// NEW of a must fly with its corridor given (4.44): the line checked (a field at fault named from 10) and laid out as it
    /// is given, then flown through from its first vertex to its last.
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpLine& line, const control::CommandOptions& options = {});
    /// NEW of a must fly with its volume given (4.45): the volume checked (a field at fault named from 10) and laid out as it is
    /// given, then entered.
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpVolume& volume, const control::CommandOptions& options = {});
    /// Several NEWs at once (docs/sdk/control.md, "The command envelope"),
    /// made in order at this simulation time and each answered on its own;
    /// `details`, if given, gets each answer's commandDetails() in order.
    std::vector<control::CommandResult> submitBatch(Span<const control::BatchCommand> batch, std::vector<control::CommandDetails>* details = nullptr);
    /// Everything the checks found for the vehicle's last NEW, validation
    /// (CommandOptions::validateOnly) or UPDATE: every finding - the answer's
    /// reason the first - and every value flown other than asked.
    control::CommandDetails commandDetails() const;
    /// The live activities, then the ended ones the vehicle remembers, newest first.
    /// A guidance mode's record carries its progress (docs/vehicle-interface.md, 5.3).
    std::vector<control::ActivityRecord> activities() const;
    /// What the vehicle is commanded (A-GRA's VehicleCommandState): what the
    /// cascade asked for in its last control update, level by level - an
    /// altitude, a heading, an airspeed, an attitude... - with the acceleration
    /// it commands in north, east and down, and the altitude as its mode
    /// commanded it, in its reference (docs/flight-autonomy.md, 4.12).
    control::VehicleCommandState commanded() const;
    /// A-GRA's navigation report (docs/flight-autonomy.md, 4.14): what the
    /// vehicle flies on - fuel or a battery - how much is left (and its percent
    /// of capacity), its endurance at its consumption now, its playtime to its
    /// recovery point (setNavigation) and its contingency level.
    control::NavigationReport navigationReport() const;
    /// Its recovery point and the reserve it keeps (NavigationSettings):
    /// Reason::OutOfRange for a point off the Earth or a reserve outside [0, 1).
    control::Reason setNavigation(const control::NavigationSettings& settings);
    control::NavigationSettings navigation() const;
    /// The QNH its barometric altimeter is set to (A-GRA's QNH setting; docs/flight-autonomy.md,
    /// 4.20), Pa: what its barometric altitudes are read and flown at. Reason::None applied,
    /// OutOfRange outside 850 to 1,100 hPa (nothing changes). Until set, 1013.25 hPa.
    control::Reason setQnh(double qnhPa);
    double qnh() const;
    /// What its altimeter reads now - its indicated altitude and rate, its Kollsman - and the
    /// air it reads it in (A-GRA's MA_AirDataType, beyond the state's airspeeds).
    control::StateData stateData() const;
    /// A flight mode's performance profile (docs/flight-autonomy.md, 4.15; A-GRA's
    /// MA_FlightControlModesPerformanceProfileType) at the vehicle's condition now: HSA/CSA,
    /// waypoint or curve following, into `out` (its vectors reused). Reason::InvalidParameter
    /// for another mode; NotSupported or NotImplemented for one the vehicle does not offer.
    control::Reason performanceProfile(control::FlightMode mode, control::PerformanceProfile& out) const;
    /// What the vehicle offers: its flight levels and behaviours.
    std::vector<control::CapabilityDescriptor> capabilities() const;
    /// A capability's availability now, as a policy is answered (docs/sdk/control.md,
    /// "Support and availability"): every reason that holds, the first as `reason`,
    /// and the ranges a placard narrows. One the vehicle does not offer is
    /// Unavailable, with not_supported, not_implemented or unknown_capability.
    control::CapabilityStatus capabilityStatus(std::string_view capability) const;
    /// Whether the aircraft can do a public feature at all (docs/flight-autonomy.md,
    /// 4.2): "fsim.guidance.hover", "fsim.guidance.hsa/direction/magnetic_north",
    /// or a behaviour's id. Null for an unknown feature; valid while the vehicle is.
    const control::SupportInfo* support(std::string_view feature) const;
    /// Every public feature's support, in control::supportFeature()'s order.
    std::vector<control::SupportInfo> supportTable() const;
    /// What the vehicle can do, as its guidance plans with it: speeds, ceiling,
    /// bank, climb, the loops' gains (docs/vehicle-interface.md, 7.1; A-GRA's
    /// performance profile). Computed afresh when its loops change.
    control::Performance performance() const;
    /// Counts every change to the grants, what is allowed, the control mode,
    /// availability and the performance: a consumer polls it (docs/sdk/control.md, "Grants").
    std::uint32_t controlRevision() const;
    // Authority (docs/sdk/control.md, "Grants"): grants over the priorities.
    /// Open (the default: as ever) or Granted: a policy's NEW then needs a
    /// grant for its capability; what the policy flies without one ends
    /// Canceled(NotGranted). The platform's own sources never need one.
    control::Reason setControlMode(control::ControlMode mode);
    control::ControlMode controlMode() const;
    /// A policy's controller (0, the default policy) asks for control of a
    /// capability (by id): Reason::None if granted, else NotAllowed, why it is
    /// unavailable, or AuthorityHeld while another controller holds it.
    control::Reason requestControl(std::string_view capability, control::ControllerId controller = 0);
    /// A controller lets go: its live activities of the capability end
    /// Canceled(Released). NotGranted, and nothing changes, for another's grant.
    control::Reason releaseControl(std::string_view capability, control::ControllerId controller = 0);
    /// The platform takes it back: the policy's live activities of it end Canceled(`reason`).
    control::Reason revokeControl(std::string_view capability, control::Reason reason = control::Reason::Revoked);
    /// Whether the policy may request the capability (all may, by default).
    control::Reason setAllowed(std::string_view capability, bool allowed);
    control::ControlStatus controlStatus(std::string_view capability) const;
    /// A capability's precedence (docs/sdk/control.md, "Ranks, queues and time
    /// windows"; lower first, 0 until set): the platform's setting, by which two
    /// activities of one source contest axes before their ranks are compared.
    control::Reason setCapabilityPrecedence(std::string_view capability, std::uint32_t precedence);
    std::uint32_t capabilityPrecedence(std::string_view capability) const;
    // Flight tasks (docs/sdk/control.md, "Flight tasks and suggestions"): a
    // command kept by id and flown on a task command, as often as its
    // repetition says; the platform's suggestions are tasks too (their ids
    // have kSuggestedTask).
    /// Keep a task: InvalidParameter for id 0, one of the platform's, no runs
    /// or runs of what never completes; TaskActive while it flies; else why
    /// the vehicle cannot command it.
    control::Reason storeTask(control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints = {},
                              Span<const control::BezierSegment> segments = {}, control::TaskRepetition repetition = {},
                              const control::PatternShape* shape = nullptr);
    /// Keep a task whose command is a batch item's: a curve's segments either way (its options not kept).
    control::Reason storeTask(control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition = {});
    /// Fly it: the NEW of its command, the task among the requirements it traces to.
    control::CommandResult commandTask(control::TaskId task, const control::CommandOptions& options = {});
    control::CommandResult cancelTask(control::TaskId task, control::Caller caller = {});
    control::Reason removeTask(control::TaskId task);
    std::optional<control::TaskStatus> taskStatus(control::TaskId task) const;
    std::vector<control::TaskStatus> tasks() const;
    // Route plans (docs/sdk/control.md, "Route plans"): A-GRA's route plans,
    // kept by id and taken through the plan activation states before they fly.
    /// A plan published (A-GRA's MA_RoutePlanMT): taken where FA listens for
    /// its id - prepared for upload - else WrongPlanState; InvalidParameter
    /// for id 0 or malformed metadata.
    control::Reason publishPlan(const control::RoutePlan& plan);
    /// A plan activation command, answered at once: completed or failed, the plan's state after it.
    control::PlanCommandResult planCommand(control::PlanId plan, control::PlanCommand command, const control::CommandOptions& options = {});
    /// FA's own deactivation (the platform's): Deactivated, its live activity canceled with `reason`.
    control::PlanCommandResult abortPlan(control::PlanId plan, control::Reason reason = control::Reason::Restricted);
    control::Reason removePlan(control::PlanId plan);
    std::optional<control::PlanStatus> planStatus(control::PlanId plan) const;
    std::vector<control::PlanStatus> plans() const;
    /// Its content as uploaded last, its planning metadata with it.
    std::optional<control::RoutePlan> plan(control::PlanId plan) const;
    // FA's own plans and the airfields (docs/sdk/control.md, "Route plans"): the platform's, read only to MA.
    /// FA's own plan, kept Uploaded: UnknownAirfield where a takeoff's or a landing's path names an airfield or runway not kept.
    control::Reason loadPlan(const control::RoutePlan& plan);
    control::Reason loadAirfield(const control::Airfield& airfield);
    std::vector<control::Airfield> airfields() const;
    std::optional<control::Airfield> airfield(control::AirfieldId airfield) const;
    /// A route plan validated without flying it - kept or not - in the weather given, from its origin, its verdict over its
    /// parts (a patch's); commandDetails() has its findings.
    control::PlanValidationResult validatePlan(const control::RoutePlan& plan, const control::PlanValidation& v = {});
    control::PlanValidationResult validatePlan(control::PlanId plan, const control::PlanValidation& v = {});
    /// The platform restricts a capability (collision avoidance, an
    /// operational restriction): a policy's NEW for it is refused with
    /// `reason`; what flies goes on. Availability::Available lifts it.
    control::Reason setAvailability(std::string_view capability, control::Availability availability,
                                    control::Reason reason = control::Reason::Restricted, std::uint64_t associated = 0,
                                    double nextAvailableS = std::numeric_limits<double>::quiet_NaN());
    /// What the vehicle knows about its aircraft: identity, effectors, envelope,
    /// propulsion, plant, performance, control (docs/control-architecture.md, 7).
    const control::VehicleProfile& profile() const;
    /// What flies the primary axes nobody owns (docs/sdk/control.md, "Owning
    /// axes apart"): Neutral, as always, or Hold - the heading, airspeed and
    /// height each had when it was let go. Reason::None if set.
    control::Reason setVehicleDefault(control::VehicleDefault mode);
    control::VehicleDefault vehicleDefault() const;
    /// Envelope protection (docs/sdk/control.md, "Envelope protection"): Limit
    /// by default for an aircraft with an envelope, else Off. It limits the
    /// demand; it does not keep the aircraft inside - what crosses a limit is
    /// reported in envelope(), which starts a new count at each call.
    control::Reason setProtection(control::ProtectionMode mode);
    control::ProtectionMode protection() const;
    control::EnvelopeStatus envelope();
    control::ControlStack& controls();
    control::Level activeLevel() const;
    bool use(control::Level level, std::string_view controllerId);

    // Lifecycle
    bool reset();
    bool reset(const InitialConditions& initial);
    bool remove();

    // Effects, properties, communication
    bool addEffect(std::unique_ptr<effects::Effect> effect);
    template <typename E, typename... Args>
    E* addEffect(Args&&... args) {
        auto e = std::make_unique<E>(std::forward<Args>(args)...);
        E* raw = e.get();
        return addEffect(std::move(e)) ? raw : nullptr;
    }
    PropertyHandle property(std::string_view path);
    comm::Node* node();

private:
    friend class World;
    Vehicle(World* world, std::uint32_t id) noexcept : world_(world), id_(id) {}
    World* world_ = nullptr;
    std::uint32_t id_ = 0;
};

class FSIM_API World {
public:
    explicit World(const WorldOptions& options = {});
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /// Load a vehicle. Throws fsim::Error on failure.
    Vehicle createVehicle(const VehicleSpec& spec);
    Vehicle vehicle(std::uint32_t id) noexcept;
    Vehicle vehicle(std::string_view name) noexcept;
    std::vector<Vehicle> vehicles();
    std::size_t vehicleCount() const noexcept;

    /// UPDATE: a new setpoint for a live activity (the per-step path; allocates nothing).
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    template <typename C>
    control::CommandResult update(control::ActivityId activity, const C& c) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return update(activity, control::SupportCommand(c));
        else
            return update(activity, control::Command(c));
    }
    /// UPDATE of a route: its options (a field kHold keeps its value) and new
    /// waypoints with their loiters, states, paths, branches and terminators - none: those it has - checked
    /// as a NEW's; flown afresh from its start. (A RouteCommand alone, through
    /// update() above, is the same with none.)
    control::CommandResult update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  Span<const control::RouteLoiter> loiters = {}, Span<const control::RouteState> states = {},
                                  Span<const control::RoutePath> paths = {}, Span<const control::RouteBranch> branches = {},
                                  Span<const control::RouteTerminator> terminators = {});
    /// UPDATE of a curve: its options (kHold keeps one), and segments - with
    /// `append` 1 after its end, from the same reference; else a new curve.
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    /// UPDATE of a pattern with its shape: the fields given in either (kHold keeps one) merged, the pattern flown afresh.
    control::CommandResult update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape);
    /// UPDATE of a marshall, with its pattern's shape or without: its slot chosen afresh where its stack moves (4.46). Its
    /// pattern takes an UPDATE through it alone (a PatternCommand's: wrong_command_type).
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape);
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall);
    /// UPDATE of a must fly with a zone given in place of its own (4.43), or a corridor (4.44).
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume);
    /// CANCEL: the activity ends; its axes fly the vehicle default.
    control::CommandResult cancel(control::ActivityId activity);
    /// UPDATE and CANCEL declaring the caller's source, as a NEW's options do,
    /// and a policy's controller (docs/sdk/control.md, "Grants" and "Named
    /// controllers"; a Source alone is the default policy's): under
    /// ControlMode::Granted a source below the activity's may not address it
    /// (AuthorityHeld, naming it) - a policy cannot change or end what the
    /// platform's own sources fly - nor may a controller another's. The calls
    /// above are the default policy's; in Open mode the caller changes nothing.
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::SupportCommand& setpoint);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::RouteCommand& route,
                                  Span<const control::Waypoint> waypoints, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                  Span<const control::BezierSegment> segments, const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                  Span<const control::NurbsSegment> segments, const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                  const control::PatternShape& shape);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                  const control::PatternShape& shape);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpZone& zone);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpLine& line);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpVolume& volume);
    control::CommandResult cancel(control::Caller caller, control::ActivityId activity);
    /// An activity command (docs/sdk/control.md, "Activity commands"): Disable
    /// (it stops flying and is kept), Enable, Reset (over from its beginning),
    /// Delete (a sticky disable: it ends), ChangeRank (`rank`), Unassign (it
    /// gives up its axes and waits for them again). The caller's source as
    /// UPDATE's; refused not_interactive where its command said it takes none.
    control::CommandResult activityCommand(control::ActivityId activity, control::ActivityCommand command, control::Rank rank = {},
                                           control::Caller caller = {});
    /// The operator's input to a route's conditional branch (docs/sdk/control.md; docs/flight-autonomy.md, 4.37): branch
    /// `branch`, one that takes it, commanded - or, `commanded` false, no longer. Held while the route flies on; flown
    /// afresh (an UPDATE, a Reset), it goes. The caller's source as UPDATE's; invalid_parameter (index: the branch) for a
    /// branch it has not, or one that takes no operator input.
    control::CommandResult commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded = true, control::Caller caller = {});
    /// A live or recently ended activity of any vehicle; empty if unknown.
    std::optional<control::ActivityRecord> activity(control::ActivityId activity) const;
    /// What a live activity flies now, or waits to fly (A-GRA's last flight
    /// command; docs/sdk/control.md, "Reports"): its setpoint as updated, a
    /// route's waypoints, a curve's segments - appended ones too, its flyout
    /// curve. Empty for an activity not live.
    std::optional<control::Setpoint> activitySetpoint(control::ActivityId activity) const;
    /// Where a live activity flies to (A-GRA's ActualEndPoint): the point it
    /// flies to now, then those after it - a route's waypoints (a repeating
    /// route's round again), a curve's segment ends, a pattern's fix, the
    /// position level's point - `max` at most.
    std::vector<control::EndPoint> endPoints(control::ActivityId activity, std::size_t max = 16) const;

    /// Advance every vehicle by n world steps (frameSkip FDM steps each).
    void step(unsigned n = 1);
    double time() const noexcept;            ///< simulation seconds since creation
    double stepSeconds() const noexcept;     ///< dt * frameSkip

    Environment& environment() noexcept { return environment_; }
    comm::Network& network();
    /// The ground's height above the WGS-84 ellipsoid at a place - the
    /// physics' own ground, which commanded paths are checked against
    /// (docs/flight-autonomy.md, 4.19; VI 1.2.6.9, A-GRA's elevation request).
    /// Empty where it has no data (a terrain tile it cannot load).
    std::optional<double> terrainHeightM(double latitudeRad, double longitudeRad) const;
    /// Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame): a frame by id -
    /// fixed, moving at a constant velocity from a time, or following a vehicle; 0 refused (a
    /// value not finite, a latitude off the Earth, an unknown vehicle).
    control::FrameId createFrame(const control::FrameSpec& spec);
    bool removeFrame(control::FrameId id);
    std::optional<control::FrameSpec> frame(control::FrameId id) const;
    /// A frame as it is at a time (NaN: now) - a vehicle's carried on at its velocity to another
    /// time - and where a point in it is then. Empty for an unknown frame, or one whose vehicle is gone.
    std::optional<control::FramePose> framePose(control::FrameId id, double timeS = std::numeric_limits<double>::quiet_NaN()) const;
    std::optional<control::GeoPoint> framePoint(control::FrameId id, const control::FrameOffset& offset,
                                                double timeS = std::numeric_limits<double>::quiet_NaN()) const;
    /// Operational points (docs/flight-autonomy.md, 4.42; A-GRA's OpPoint): kept by id in place of any by it, its revision one
    /// more - a place on the Earth or in a frame, and the window of bearings it is approached from - for a must fly to name.
    /// InvalidParameter for a malformed one (id 0, neither a place nor a frame or both, a frame the world does not have, ...).
    control::Reason setOpPoint(const control::OpPoint& point);
    bool removeOpPoint(control::OpPointId id);
    std::vector<control::OpPointId> opPoints() const;
    std::optional<control::OpPoint> opPoint(control::OpPointId id) const;
    /// Operational zones (docs/flight-autonomy.md, 4.43; A-GRA's OpZone): kept by id in place of any by it, its revision one
    /// more - a polygon with holes, an ellipse, a rectangle or a slant range area, on the Earth or in a frame, its band of
    /// altitudes, a velocity - for a must fly to name. InvalidParameter for one A-GRA's schema would not take.
    control::Reason setOpZone(const control::OpZone& zone);
    bool removeOpZone(control::OpZoneId id);
    std::vector<control::OpZoneId> opZones() const;
    std::optional<control::OpZone> opZone(control::OpZoneId id) const;
    /// Operational lines (docs/flight-autonomy.md, 4.44; A-GRA's OpLine): kept as zones are - 2 to 32 vertices on the Earth or
    /// in a frame, each with its altitude, range and widths, the line's projection, widths and band, a velocity - for a must
    /// fly to fly through. InvalidParameter for one A-GRA's schema would not take.
    control::Reason setOpLine(const control::OpLine& line);
    bool removeOpLine(control::OpLineId id);
    std::vector<control::OpLineId> opLines() const;
    std::optional<control::OpLine> opLine(control::OpLineId id) const;
    /// Operational volumes (docs/flight-autonomy.md, 4.45; A-GRA's OpVolume): kept as zones are - a sphere, a dome, an ellipsoid,
    /// a cylinder, a cone or a rectangular cone at its point (on the Earth or in a frame, turned, maybe moving), or a geocentric
    /// box - for a must fly to enter. InvalidParameter for one A-GRA's schema would not take.
    control::Reason setOpVolume(const control::OpVolume& volume);
    bool removeOpVolume(control::OpVolumeId id);
    std::vector<control::OpVolumeId> opVolumes() const;
    std::optional<control::OpVolume> opVolume(control::OpVolumeId id) const;

    /// Give every vehicle (present and future) its own effect instance.
    void addEffectToAll(std::function<std::unique_ptr<effects::Effect>()> factory);
    template <typename E, typename... Args>
    void addEffectToAll(Args... args) {
        addEffectToAll([=] { return std::make_unique<E>(args...); });
    }

    const std::string& name() const noexcept;
    bool published() const noexcept;         ///< viewers can see this world
    std::uint64_t vehicleSteps() const noexcept;
    std::uint64_t worldSteps() const noexcept;
    void publishNow();

    /// Internal object (for the platform's own tools; not part of the stable API).
    session::World& impl() noexcept { return *impl_; }
    /// Internal: a view of a world owned elsewhere (VecEnv::world, the C ABI); not part of the stable API.
    static World* borrowed(session::World& w) { return new World(Borrow{}, w); }

private:
    friend class Vehicle;
    friend class Environment;
    struct Borrow {};
    World(Borrow, session::World& borrowed);
    std::shared_ptr<session::World> impl_;
    Environment environment_;
};

FSIM_API const char* version() noexcept;

} // namespace fsim
