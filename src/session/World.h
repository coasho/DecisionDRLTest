#pragma once

// The simulation session behind fsim::World (design 9.2): vehicles by name
// and type, lockstep stepping through the vehicle pool, control cascades and
// effects on the worker that owns each vehicle, the environment applied per
// step, the message fabric, and the shared-memory publisher for viewers.

#include "comm/Comm.h"
#include "control/Adapter.h"
#include "control/CapabilityHost.h"
#include "control/Catalog.h"
#include "control/ControlStack.h"
#include "control/Features.h"
#include "fsim/VehicleProfile.h"
#include "effects/Effect.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Frames.h"
#include "fsim/InitialConditions.h"
#include "fsim/Rng.h"
#include "io/TerrainTiles.h"
#include "ipc/Recording.h"
#include "ipc/WorldPublisher.h"
#include "sim/GroundProvider.h"
#include "sim/VehiclePool.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace fsim::control {
struct PerformanceProfile; // fsim/PerformanceProfile.h
}

namespace fsim::session {

struct WorldOptions {
    std::string name = "default";       ///< shared-memory world name (viewers attach by it)
    double dt = 1.0 / 120.0;            ///< FDM step
    int frameSkip = 4;                  ///< FDM steps per world step
    unsigned workers = 0;               ///< 0 = physical cores - 2
    bool pinWorkers = true;             ///< one worker per physical core (throughput stability)
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;       ///< published vehicle slots
    bool publish = true;                ///< false = invisible to viewers
    double publishIntervalSeconds = 1.0 / 60.0;
    std::filesystem::path jsbsimRoot;   ///< empty = auto-detect
    std::shared_ptr<sim::GroundProvider> ground; ///< custom ground; null = `terrain` below or flat at 0 m
    bool terrain = false;               ///< physics ground from the public elevation tiles the viewer draws
    std::string terrainUrl;             ///< tile template; empty = AWS Terrarium
    unsigned terrainZoom = 12;          ///< ~38 m/px
    double terrainPrefetchRadiusM = 4000.0; ///< tiles loaded (blocking) around each new vehicle's position
    std::filesystem::path recordPath;   ///< non-empty: write every vehicle's state to this file for replay in the viewer
    double recordIntervalSeconds = 0.0; ///< simulation time between recorded frames; 0 = every world step
};

struct VehicleSpec {
    std::string name;                   ///< unique in the world; empty = "<type>-<id>"
    std::string type = "jsbsim:c172x";  ///< "<flight model>:<aircraft>"; a bare aircraft name means JSBSim
    sim::InitialConditions initial;
    std::string model;                  ///< optional visual override for the viewer (glTF path)
    unsigned controlDivider = 1;        ///< run the control stack every N FDM steps
    /// Sections of the aircraft's profile to replace for this vehicle (those
    /// present(); docs/control-architecture.md, 7.4); null = the aircraft's own.
    std::shared_ptr<const control::VehicleProfile> profile;
};

struct VehicleInfo {
    std::uint32_t id = 0;
    std::string name, type, aircraft, model;
    sim::InitialConditions initial;
    bool alive = false;
    std::uint64_t generation = 0;
};

class World final : public control::WorldView {
public:
    explicit World(const WorldOptions& options);
    ~World() override;
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // --- Vehicles ------------------------------------------------------------
    /// Create (load) a vehicle. Returns its id, or 0 on failure (logged).
    std::uint32_t createVehicle(const VehicleSpec& spec);
    bool removeVehicle(std::uint32_t id);
    /// Re-apply initial conditions (`ic` null = the spec's) and reset controls/effects.
    bool resetVehicle(std::uint32_t id, const sim::InitialConditions* ic = nullptr);
    bool alive(std::uint32_t id) const noexcept;
    const VehicleInfo* info(std::uint32_t id) const noexcept;
    std::uint32_t find(const std::string& name) const noexcept;
    std::vector<std::uint32_t> vehicleIds() const;
    std::size_t vehicleCount() const noexcept { return liveCount_; }

    // --- State ---------------------------------------------------------------
    const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
    const effects::SensedState* sensedState(std::uint32_t id) const noexcept;
    sim::FlightModel* model(std::uint32_t id) noexcept;
    const sim::ControlInputs* inputs(std::uint32_t id) const noexcept;

    // --- Control (design 9.3, docs/control-architecture.md) ---------------------
    /// What every existing entry point calls: an update of the vehicle's own
    /// activity at the same level, else a new one (10.7). False if the vehicle
    /// is unknown or the command rejected (a behaviour nobody registered, an
    /// axis an autopilot or override activity holds).
    bool command(std::uint32_t id, const control::Command& command);
    /// command() with its answer (the C ABI reports why a command was refused).
    control::CommandResult commandResult(std::uint32_t id, const control::Command& command);
    /// NEW: a command becomes an activity, or is rejected with a reason.
    control::CommandResult submit(std::uint32_t id, const control::Command& command, const control::CommandOptions& options = {});
    /// NEW for a support effector the vehicle has (gear, flaps, brakes, speedbrake, pitch trim) or its engines' throttles.
    control::CommandResult submit(std::uint32_t id, const control::SupportCommand& command, const control::CommandOptions& options = {});
    /// NEW of a route (fsim.guidance.route) with its waypoints (docs/vehicle-interface.md, 4.5), its loiter points'
    /// loiters (docs/flight-autonomy.md, 4.31), its planned states (4.34), its paths (4.36), its branches (4.37) and its
    /// civil path terminators' data (4.38).
    control::CommandResult submit(std::uint32_t id, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  const control::CommandOptions& options = {}, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    /// NEW of a curve (fsim.guidance.curve) with its segments (docs/vehicle-interface.md, 4.7): Bezier segments, or as
    /// A-GRA's schema gives them (docs/flight-autonomy.md, 4.26); its reference in a frame beside it, `shape` (4.27).
    control::CommandResult submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    control::CommandResult submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    /// NEW of a pattern (fsim.guidance.pattern) with its shape (docs/flight-autonomy.md, 4.23).
    /// NEW of a must fly with its zone given (docs/flight-autonomy.md, 4.43): the zone checked and laid out as it is given.
    control::CommandResult submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpZone& zone,
                                  const control::CommandOptions& options = {});
    control::CommandResult submit(std::uint32_t id, const control::PatternCommand& pattern, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    /// Several NEWs at once (docs/flight-autonomy.md, 4.8), made in order at
    /// this simulation time, each answered on its own; `details`, if given,
    /// gets each answer's details (commandDetails()) in the same order.
    std::vector<control::CommandResult> submitBatch(std::uint32_t id, Span<const control::BatchCommand> batch,
                                                    std::vector<control::CommandDetails>* details = nullptr);
    /// Everything the checks found for the vehicle's last NEW, validation or
    /// UPDATE (docs/flight-autonomy.md, 4.8): every finding and every value
    /// flown other than asked. Null for an unknown vehicle.
    const control::CommandDetails* commandDetails(std::uint32_t id) const noexcept;
    /// UPDATE: a new setpoint for a live activity (the fast path).
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    /// UPDATE of a route: new options (kHold keeps one) and waypoints with their loiters, states, paths, branches and
    /// terminators (none: those it has); flown afresh from its start.
    control::CommandResult update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  Span<const control::RouteLoiter> loiters = {}, Span<const control::RouteState> states = {},
                                  Span<const control::RoutePath> paths = {}, Span<const control::RouteBranch> branches = {},
                                  Span<const control::RouteTerminator> terminators = {});
    /// UPDATE of a curve: options, and segments appended (append 1) or a new curve.
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    /// UPDATE of a pattern with its shape: the fields given in either merged (kHold keeps one), flown afresh.
    control::CommandResult update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape);
    /// UPDATE of a must fly with a zone given in place of its own (4.43).
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone);
    /// CANCEL: the activity ends; its axes fly the vehicle default.
    control::CommandResult cancel(control::ActivityId activity);
    /// UPDATE and CANCEL declaring the caller's source, as a NEW's options do,
    /// and a policy's controller (docs/vehicle-interface.md, 6.1;
    /// docs/flight-autonomy.md, 4.12): under ControlMode::Granted a source
    /// below the activity's may not address it (AuthorityHeld, naming it), nor
    /// a controller another's. The calls above are the default policy's.
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
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpZone& zone);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                  const control::PatternShape& shape);
    control::CommandResult cancel(control::Caller caller, control::ActivityId activity);
    /// An activity command (docs/flight-autonomy.md, 4.10) - disable, enable,
    /// reset, delete, change its rank, unassign it - declaring the caller's
    /// source as UPDATE does; `rank` for ChangeRank.
    control::CommandResult activityCommand(control::Caller caller, control::ActivityId activity, control::ActivityCommand command,
                                           control::Rank rank = {});
    /// The operator's input to a route's conditional branch (docs/flight-autonomy.md, 4.37): branch `branch` commanded, or
    /// no longer - declaring the caller's source as UPDATE does.
    control::CommandResult commandBranch(control::Caller caller, control::ActivityId activity, std::uint32_t branch, bool commanded = true);
    control::CommandResult commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded = true) {
        return commandBranch(control::Source::Policy, activity, branch, commanded);
    }
    // --- Flight tasks (docs/flight-autonomy.md, 4.11): kept by id, flown on a task command ---
    control::Reason storeTask(std::uint32_t id, control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints = {},
                              Span<const control::BezierSegment> segments = {}, control::TaskRepetition repetition = {},
                              const control::PatternShape* shape = nullptr);
    /// A task whose command is a batch item's - a route's waypoints, a curve's segments either way, a pattern's shape
    /// beside it (its options not kept: a task command gives them). InvalidParameter for a support command's.
    control::Reason storeTask(std::uint32_t id, control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition = {});
    control::CommandResult commandTask(std::uint32_t id, control::TaskId task, const control::CommandOptions& options = {});
    control::CommandResult cancelTask(std::uint32_t id, control::TaskId task, control::Caller caller = {});
    control::Reason removeTask(std::uint32_t id, control::TaskId task);
    /// A task's status; empty for one not kept (or an unknown vehicle).
    std::optional<control::TaskStatus> taskStatus(std::uint32_t id, control::TaskId task);
    std::vector<control::TaskStatus> tasks(std::uint32_t id);
    // --- Route plans (docs/flight-autonomy.md, 4.39): kept by id, taken through the plan activation states ---
    /// A plan published (A-GRA's MA_RoutePlanMT): taken where FA listens for its id (None: A-GRA's notification
    /// CONFIRMED); WrongPlanState where it does not; InvalidParameter for id 0 or malformed metadata.
    control::Reason publishPlan(std::uint32_t id, const control::RoutePlan& plan);
    /// A plan activation command (A-GRA's), answered at once: completed or failed, and the plan's state after it.
    control::PlanCommandResult planCommand(std::uint32_t id, control::PlanId plan, control::PlanCommand command,
                                           const control::CommandOptions& options = {});
    /// FA's own deactivation (VI 1.2.5.7), the platform's: a plan ready for activation or activated is Deactivated, its
    /// live activity canceled with `reason`, its execution Canceled.
    control::PlanCommandResult abortPlan(std::uint32_t id, control::PlanId plan, control::Reason reason = control::Reason::Restricted);
    /// Forget a plan: UnknownPlan; WrongPlanState while its activity is live.
    control::Reason removePlan(std::uint32_t id, control::PlanId plan);
    /// A plan's status; empty for one not kept (or an unknown vehicle).
    std::optional<control::PlanStatus> planStatus(std::uint32_t id, control::PlanId plan) const;
    /// Every plan kept, in the order they were first prepared for upload (A-GRA's query for identifiers only).
    std::vector<control::PlanStatus> plans(std::uint32_t id) const;
    /// A plan's content as uploaded last, its metadata with it; empty for one not kept, or not yet uploaded.
    std::optional<control::RoutePlan> plan(std::uint32_t id, control::PlanId plan) const;
    // FA's own plans and the airfields (docs/flight-autonomy.md, 4.40): the platform's, read only to MA
    /// FA's own plan: kept Uploaded, in place of any plan by its id not flying. InvalidParameter, UnknownAirfield (a takeoff's
    /// or a landing's path naming an airfield or runway the vehicle does not keep), WrongPlanState, PlanStoreFull.
    control::Reason loadPlan(std::uint32_t id, const control::RoutePlan& plan);
    /// An airfield: kept in place of any by its id, its revision one more. InvalidParameter, PlanStoreFull.
    control::Reason loadAirfield(std::uint32_t id, const control::Airfield& airfield);
    /// Every airfield kept, as loaded (A-GRA's query for the airfields).
    std::vector<control::Airfield> airfields(std::uint32_t id) const;
    std::optional<control::Airfield> airfield(std::uint32_t id, control::AirfieldId airfield) const;
    /// A route plan validated without flying it (docs/flight-autonomy.md, 4.41): its route checked as its NEW would be, in
    /// the weather given, from its origin, its verdict over its parts (a patch's). Kept or not; nothing is kept or flown.
    /// commandDetails(id) has its findings and adjustments.
    control::PlanValidationResult validatePlan(std::uint32_t id, const control::RoutePlan& plan, const control::PlanValidation& v = {});
    /// A kept plan's, as uploaded last: UnknownPlan; WrongPlanState before its first upload.
    control::PlanValidationResult validatePlan(std::uint32_t id, control::PlanId plan, const control::PlanValidation& v = {});
    // --- Reports (docs/flight-autonomy.md, 4.12): what an activity flies, and where to ---
    /// What a live activity flies now, or waits to fly: its setpoint as
    /// updated, a route's waypoints, a curve's segments (appended ones too: its
    /// flyout curve). False for an activity not live.
    bool activitySetpoint(control::ActivityId activity, control::Setpoint& out) const;
    /// A live route's next point with a required time of arrival, as it estimates it now (docs/flight-autonomy.md, 4.33):
    /// when it will arrive, and that against its window; false where it has none.
    bool activityArrival(control::ActivityId activity, control::ArrivalEstimate& out) const;
    /// Where a live activity flies to (A-GRA's ActualEndPoint): the point it
    /// flies to now, then those after it, `max` at most.
    std::vector<control::EndPoint> endPoints(control::ActivityId activity, std::size_t max = 16) const;
    /// What the vehicle is commanded (A-GRA's VehicleCommandState): the
    /// cascade's levels, the acceleration they command in north, east and
    /// down, the altitude as its mode commanded it. NaN fields for an unknown vehicle.
    control::VehicleCommandState commandState(std::uint32_t id) const;
    // --- The navigation report (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport) ---
    /// What the vehicle flies on, how much is left and for how long, its
    /// playtime to its recovery point and its contingency (Navigation.cpp).
    /// Energy::Unknown for an unknown vehicle, or one its flight model says
    /// nothing of.
    control::NavigationReport navigationReport(std::uint32_t id) const;
    /// Its recovery point and reserve: Reason::OutOfRange for a point off the
    /// Earth or a reserve outside [0, 1).
    control::Reason setNavigation(std::uint32_t id, const control::NavigationSettings& settings);
    control::NavigationSettings navigation(std::uint32_t id) const noexcept;
    // --- The terrain (docs/flight-autonomy.md, 4.19; VI 1.2.6.9, A-GRA's elevation request) ---
    /// The ground's height above the WGS-84 ellipsoid at a place, as the
    /// physics has it; empty where it has no data (a terrain tile it cannot load).
    std::optional<double> terrainHeightM(double latitudeRad, double longitudeRad) const {
        return ground_->knownHeightAboveEllipsoidM(latitudeRad, longitudeRad);
    }
    // --- The barometric altimeter (docs/flight-autonomy.md, 4.20; VI 1.2.6.5 and 1.2.6.8) ---
    /// The QNH a vehicle's altimeter is set to (A-GRA's QNH setting), Pa - what
    /// its barometric altitudes are read and flown at: applied (Reason::None),
    /// or OutOfRange outside 850 to 1,100 hPa (nothing changes); UnknownVehicle.
    /// Until set, the standard 1013.25 hPa: the altimeter reads pressure altitude.
    control::Reason setQnh(std::uint32_t id, double qnhPa);
    double qnh(std::uint32_t id) const noexcept; ///< NaN for an unknown vehicle
    /// What its altimeter reads now, and the air it reads it in (StateData;
    /// A-GRA's MA_AirDataType). NaN fields for an unknown vehicle.
    control::StateData stateData(std::uint32_t id) const;
    /// The world's air now (its environment's sea-level temperature and pressure).
    control::Air air() const noexcept { return control::Air{environment_.temperatureSeaLevelK, environment_.pressureSeaLevelPa}; }
    // --- Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame) ---
    /// A frame by id (A-GRA's ReferenceFrameID): fixed, moving at a constant
    /// velocity from a time, or following a vehicle. 0: refused - a value not
    /// finite, a latitude off the Earth, an origin not one, an unknown vehicle.
    control::FrameId createFrame(const control::FrameSpec& spec);
    bool removeFrame(control::FrameId id);
    std::optional<control::FrameSpec> frame(control::FrameId id) const;
    /// A frame as it is at a time (NaN: now; simulation seconds, simTime()): a
    /// vehicle's from its state now, carried on at its velocity to another
    /// time. Empty for an unknown frame, or one whose vehicle is gone.
    std::optional<control::FramePose> framePose(control::FrameId id, double timeS = std::numeric_limits<double>::quiet_NaN()) const;
    /// Where a point in a frame is at a time (NaN: now).
    std::optional<control::GeoPoint> framePoint(control::FrameId id, const control::FrameOffset& offset,
                                                double timeS = std::numeric_limits<double>::quiet_NaN()) const;
    // --- Operational geometry (docs/flight-autonomy.md, 4.42; A-GRA's OpPoint) ---
    /// An operational point, kept by its id in place of any by it, its revision one more (Geometry.cpp). InvalidParameter:
    /// id 0; neither a place nor a frame, or both; a latitude off the Earth, a value not finite, a code not one; a frame
    /// the world does not have, or offsets without one; an altitude reference without its altitude; a window of bearings
    /// given one way alone, or beyond half a turn.
    control::Reason setOpPoint(const control::OpPoint& point);
    bool removeOpPoint(control::OpPointId id);
    /// The operational points kept, by id.
    std::vector<control::OpPointId> opPoints() const;
    std::optional<control::OpPoint> opPoint(control::OpPointId id) const;
    /// An operational zone (4.43; A-GRA's OpZone), kept by its id in place of any by it, its revision one more; a moving one's
    /// time left out, now (Geometry.cpp). InvalidParameter for one A-GRA's schema would not take, or a frame the world does not
    /// have.
    control::Reason setOpZone(const control::OpZone& zone);
    bool removeOpZone(control::OpZoneId id);
    std::vector<control::OpZoneId> opZones() const;
    std::optional<control::OpZone> opZone(control::OpZoneId id) const;
    // --- The performance profile (docs/flight-autonomy.md, 4.15; A-GRA's MA_FlightControlModesPerformanceProfileType) ---
    /// A flight mode's performance profile at the vehicle's condition now - HSA/CSA, waypoint or curve following -
    /// into `out`, its vectors reused (PerformanceProfile.cpp). InvalidParameter for another mode (A-GRA profiles
    /// those three), the support table's refusal for one the vehicle does not offer, UnknownVehicle.
    control::Reason performanceProfile(std::uint32_t id, control::FlightMode mode, control::PerformanceProfile& out);
    /// What flies a vehicle's primary axes nobody owns: the neutral actuator
    /// command (as always) or a hold of the heading, airspeed and height each
    /// had when it was let go. Reason::None if set.
    control::Reason setVehicleDefault(std::uint32_t id, control::VehicleDefault mode);
    control::VehicleDefault vehicleDefault(std::uint32_t id) const noexcept;
    /// Envelope protection (docs/control-architecture.md, 11): Limit by default
    /// for an aircraft with an envelope, else Off. Reason::None if set.
    control::Reason setProtection(std::uint32_t id, control::ProtectionMode mode);
    control::ProtectionMode protection(std::uint32_t id) const noexcept;
    /// What protection saw since the last call (each call starts a new count).
    control::EnvelopeStatus envelope(std::uint32_t id);
    /// A live or recently ended activity; null if unknown.
    const control::ActivityRecord* activity(control::ActivityId activity) const noexcept;
    /// A vehicle's live activities, then the ended ones it remembers, newest first.
    std::vector<control::ActivityRecord> activities(std::uint32_t id) const;
    /// What a vehicle offers (empty for an unknown vehicle).
    const std::vector<control::CapabilityDescriptor>& capabilities(std::uint32_t id);
    /// A capability's availability as a policy is answered (docs/flight-autonomy.md,
    /// 4.1): for one the vehicle does not offer, Unavailable with NotSupported,
    /// NotImplemented or UnknownCapability.
    control::CapabilityStatus capabilityStatus(std::uint32_t id, std::string_view capability) const;
    /// Whether the vehicle's aircraft can do a public feature at all
    /// (docs/flight-autonomy.md, 4.2), by its identifier or a behaviour's id;
    /// null for an unknown vehicle or feature.
    const control::SupportInfo* support(std::uint32_t id, std::string_view feature) const noexcept;
    /// Every public feature's support on the vehicle, in supportFeature()'s order; null for an unknown vehicle.
    const control::SupportTable* supportTable(std::uint32_t id) const noexcept;
    /// What the vehicle flies with: its aircraft's profile, with the spec's sections over it.
    const control::VehicleProfile* profile(std::uint32_t id) const noexcept;
    control::ControlStack* controls(std::uint32_t id) noexcept;
    const control::ControlStack* controls(std::uint32_t id) const noexcept;
    /// What the vehicle can do, as its guidance plans with it (docs/vehicle-interface.md, 7.1): computed
    /// afresh first if its loops changed. Null for an unknown vehicle.
    const control::Performance* performance(std::uint32_t id) noexcept;

    // --- Authority and availability (docs/vehicle-interface.md, 6 and 7.2) ---------
    /// Open (as ADR-26, the default) or Granted: a policy's NEW then needs a
    /// grant for its capability, and what the policy flies without one ends.
    control::Reason setControlMode(std::uint32_t id, control::ControlMode mode);
    control::ControlMode controlMode(std::uint32_t id) const noexcept;
    /// A policy's controller asks for control of a capability (by id):
    /// Reason::None if granted; else NotAllowed, the reason it is unavailable,
    /// AuthorityHeld (another controller holds it), UnknownCapability.
    control::Reason requestControl(std::uint32_t id, std::string_view capability, control::ControllerId controller = 0);
    /// A controller lets go: the grant ends, and its live activities of the
    /// capability end Canceled(Released). NotGranted for another's grant.
    control::Reason releaseControl(std::uint32_t id, std::string_view capability, control::ControllerId controller = 0);
    /// The platform takes it back: the grant ends, and the policy's live activities of it end Canceled(`reason`).
    control::Reason revokeControl(std::uint32_t id, std::string_view capability, control::Reason reason = control::Reason::Revoked);
    /// Whether the policy may request the capability; a grant for one no longer allowed is revoked.
    control::Reason setAllowed(std::uint32_t id, std::string_view capability, bool allowed);
    control::ControlStatus controlStatus(std::uint32_t id, std::string_view capability) const;
    /// A capability's precedence (docs/flight-autonomy.md, 4.9; lower first, 0 until set): the platform's
    /// setting, by which two activities of one source contest axes before their ranks. UnknownCapability, or why
    /// the vehicle does not offer it.
    control::Reason setCapabilityPrecedence(std::uint32_t id, std::string_view capability, std::uint32_t precedence);
    std::uint32_t capabilityPrecedence(std::uint32_t id, std::string_view capability) const;
    /// The platform restricts a capability (or, Available, lifts it): a policy's NEW for it is refused with
    /// `reason`. The status reports the id it is about and when it is expected back (NaN: not known).
    control::Reason setAvailability(std::uint32_t id, std::string_view capability, control::Availability availability,
                                    control::Reason reason = control::Reason::Restricted, std::uint64_t associated = 0,
                                    double nextAvailableS = control::kUnknown);
    /// Counts every change to grants, what is allowed, the control mode, availability and the performance (6.3).
    std::uint32_t controlRevision(std::uint32_t id) noexcept;

    // --- Effects ---------------------------------------------------------------
    bool addEffect(std::uint32_t id, std::unique_ptr<effects::Effect> effect);
    /// Give every vehicle (present and future) its own instance from `factory`.
    using EffectFactory = std::function<std::unique_ptr<effects::Effect>()>;
    void addEffectToAll(EffectFactory factory);
    void clearEffects(std::uint32_t id);

    // --- Environment, time, comm ---------------------------------------------
    const sim::EnvironmentState& environment() const noexcept override { return environment_; }
    bool navigation(std::uint32_t id, control::NavigationReport& out) const noexcept override;
    void setEnvironment(const sim::EnvironmentState& environment);
    double simTime() const noexcept override { return simTime_; }
    double dt() const noexcept { return options_.dt; }
    int frameSkip() const noexcept { return options_.frameSkip; }
    comm::Network& network() noexcept { return network_; }
    const comm::Network& network() const noexcept { return network_; }
    Rng& rng() noexcept { return rng_; }
    const std::string& name() const noexcept { return options_.name; }
    bool published() const noexcept { return publisher_ && publisher_->active(); }
    /// The elevation tiles used for physics when `terrain` is on (null otherwise).
    io::TerrainTiles* terrain() noexcept { return terrain_.get(); }

    // --- Stepping ------------------------------------------------------------
    /// Advance every vehicle by `n` world steps (frameSkip FDM steps each).
    void step(unsigned n = 1);
    std::uint64_t vehicleSteps() const noexcept { return vehicleSteps_; }
    std::uint64_t worldSteps() const noexcept { return worldSteps_; }
    /// Push the current state to viewers now, regardless of the publish interval.
    void publishNow();

private:
    struct Entry {
        VehicleInfo info;
        std::size_t slot = 0;
        control::ControlStack stack;       ///< the control runtime
        control::CapabilityHost host;      ///< its contract layer (bound to `stack`)
        control::ActivityId commanded = 0; ///< the activity the last accepted command made or updated
        control::Level level = control::Level::Actuator; ///< as last published and recorded
        std::shared_ptr<const control::VehicleProfile> profile;
        std::shared_ptr<control::CapabilityCatalog> catalog; ///< its aircraft type's (or its own, with a profile of its own)
        std::shared_ptr<const control::SupportTable> support; ///< likewise: what it can do at all
        sim::PropertyHandle flapsPosition;                   ///< for flap activities that complete in position
        /// Its body's angular accelerations (p, q, r dot), for its state data's orientation accelerations (StateData.cpp).
        std::array<sim::PropertyHandle, 3> angularAcceleration;
        sim::EffectorInputs effectors;                       ///< as last written to the flight model
        std::vector<std::unique_ptr<effects::Effect>> effects;
        effects::SensedState sensed;
        Rng rng;
        sim::ControlInputs inputs;         ///< last inputs the cascade produced
        sim::VehicleState working;         ///< per-sub-step state for control/effects
        unsigned controlDivider = 1;
        bool forceApplied = false;         ///< external force pushed last step (needs zeroing)
        bool windApplied = false;
        control::NavigationSettings navigation; ///< its recovery point and reserve
    };

    /// What the control cascades see of other vehicles during a step: each
    /// one's state as the step began (Control.h, WorldView), whichever worker
    /// steps which vehicle and however far it has got.
    class StepView final : public control::WorldView {
    public:
        explicit StepView(const World& world) noexcept : world_(world) {}
        const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
        double simTime() const noexcept override { return world_.simTime_; }
        const sim::EnvironmentState& environment() const noexcept override { return world_.environment_; }
        /// Its own report, for the vehicle whose control update asks (4.37: a route's branch).
        bool navigation(std::uint32_t id, control::NavigationReport& out) const noexcept override { return world_.navigation(id, out); }

    private:
        const World& world_;
    };
    /// What the vehicles' hosts ask of the world when they check a command
    /// (docs/flight-autonomy.md, 4.18, 4.19): the energy on board, the
    /// ground's height. Asked at a NEW or an UPDATE, never while stepping.
    class Answers final : public control::SessionView {
    public:
        explicit Answers(const World& world) noexcept : world_(world) {}
        control::EnergyNow energyNow(std::uint32_t id) const override;
        double groundM(double latitudeRad, double longitudeRad) const override { return world_.ground_->heightAboveEllipsoidM(latitudeRad, longitudeRad); }
        double groundResolutionM() const override { return world_.ground_->resolutionM(); }
        double utcSeconds() const override { return world_.environment_.epochUtcSeconds + world_.simTime_; }
        double simTimeS() const override { return world_.simTime_; }
        bool frame(control::FrameId id, control::FrameSpec& spec, control::FramePose& now) const override;
        bool opPoint(control::OpPointId id, control::OpPoint& out) const override;
        const control::OpZone* opZone(control::OpZoneId id) const override;

    private:
        const World& world_;
    };

    Entry* entry(std::uint32_t id) noexcept;
    const Entry* entry(std::uint32_t id) const noexcept;
    void preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs);
    void applyEnvironment(sim::FlightModel& model) const;
    void levelChanged(Entry& e);
    void publishVehicle(const Entry& e);
    static bool parseType(const std::string& type, std::string& family, std::string& aircraft);

    WorldOptions options_;
    std::filesystem::path jsbsimRoot_;
    std::shared_ptr<sim::GroundProvider> ground_;
    std::shared_ptr<io::TerrainTiles> terrain_;              ///< set when options.terrain (ground_ aliases it)
    std::unique_ptr<sim::VehiclePool> pool_;
    std::vector<std::unique_ptr<Entry>> entries_;               ///< by slot (null when never used)
    std::unordered_map<std::uint32_t, std::size_t> idToSlot_;
    std::vector<std::size_t> freeSlots_;
    std::vector<std::string> slotAircraft_;                     ///< loaded aircraft per slot (for reuse)
    /// Each loaded aircraft's profile (JSBSim properties under fsim/), read on its first load.
    std::unordered_map<std::string, std::shared_ptr<const control::VehicleProfile>> profiles_;
    /// What each aircraft type offers, from its profile through its adapter.
    std::unordered_map<std::string, std::shared_ptr<control::CapabilityCatalog>> catalogs_;
    /// What each aircraft type can do at all (docs/flight-autonomy.md, 4.2).
    std::unordered_map<std::string, std::shared_ptr<const control::SupportTable>> supports_;
    std::vector<sim::ControlInputs> poolInputs_;
    std::vector<sim::VehicleState> stepStates_;                 ///< by slot: every state as the current step began
    StepView stepView_{*this};
    std::vector<EffectFactory> worldEffects_;
    sim::EnvironmentState environment_;
    std::uint64_t appliedEnvironment_ = ~0ull;
    comm::Network network_;
    std::unique_ptr<ipc::WorldPublisher> publisher_;
    ipc::RecordWriter recorder_;
    double lastRecordedTime_ = -1.0;
    void recordVehicle(const Entry& e);
    void recordFrame();
    Rng rng_;
    std::uint32_t nextId_ = 1;
    std::size_t liveCount_ = 0;
    double simTime_ = 0.0;
    std::uint64_t vehicleSteps_ = 0, worldSteps_ = 0;
    Answers answers_{*this};
    std::map<control::FrameId, control::FrameSpec> frames_; ///< the reference frames, by id (4.21)
    control::FrameId lastFrame_ = 0;
    std::map<control::OpPointId, control::OpPoint> opPoints_; ///< the operational points, by id (4.42)
    std::map<control::OpZoneId, control::OpZone> opZones_;    ///< the operational zones, by id (4.43)
};

} // namespace fsim::session
