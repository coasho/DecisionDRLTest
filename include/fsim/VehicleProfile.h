#pragma once

// What differs between aircraft, as data (docs/control-architecture.md,
// section 7): an aggregate of small sections, each with its own version and
// provenance, produced and read independently. An aircraft carries them as
// JSBSim properties fsim/<section>/<field> in its flight control section (the
// gains of fsim/control are the control section); a trainer can supply
// sections for a vehicle of its own (VehicleSpec::profile).

#include "fsim/ControlStack.h"
#include "fsim/Export.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::control {

inline constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();

/// Where a section came from.
enum class Provenance : std::uint8_t {
    Default = 0,    ///< no carrier had it: the built-in defaults
    Hangar = 1,     ///< written by hangar from a design and its flight tests
    Identified = 2, ///< measured on the flying aircraft
    User = 3,       ///< written by hand, or given through VehicleSpec::profile
    Derived = 4,    ///< read from the loaded flight model
};

struct SectionHeader {
    std::uint16_t version = 0; ///< 0: no carrier had the section (defaults)
    Provenance provenance = Provenance::Default;
    bool present() const noexcept { return version != 0; }
};

// --- identity -----------------------------------------------------------------------

enum class AircraftClass : std::uint8_t {
    Unknown = 0, LightGa, Fighter, Attack, Bomber, Transport, Tanker, Aew, Reconnaissance, ElectronicWarfare, Uav, Trainer,
    Helicopter, Multirotor ///< identity version 2
};
/// How the controls reach what flies the aircraft: JSBSim's own FCS
/// (unknown), surfaces, a fly-by-wire law; a helicopter's cyclic, pedals
/// and collective; a multirotor's mixer and motors (docs/rotorcraft.md, 3.2).
enum class ControlFamily : std::uint8_t { Stock = 0, Direct = 1, FlyByWire = 2, Helicopter = 3, Multirotor = 4 };

/// Whether a family's aircraft flies on rotors rather than a wing.
constexpr bool isRotorcraft(ControlFamily f) noexcept { return f == ControlFamily::Helicopter || f == ControlFamily::Multirotor; }

struct IdentitySection {
    static constexpr std::uint16_t kVersion = 2; ///< 2: the rotorcraft classes and families
    SectionHeader header;
    AircraftClass aircraftClass = AircraftClass::Unknown;
    ControlFamily family = ControlFamily::Stock;
    std::uint32_t built = 0; ///< yyyymmdd
};

// --- effectors ------------------------------------------------------------------------

/// What each input moves. A rotorcraft's (effectors version 2): the cyclic
/// tilting its rotor, a mixer sharing the input among its rotors, the tail
/// rotor's pitch (the pedals).
enum class PitchControl : std::uint8_t { Surface = 0, LoadFactor = 1, PitchRate = 2, Cyclic = 3, Mixer = 4 };
enum class RollControl : std::uint8_t { Surface = 0, RollRate = 1, Cyclic = 2, Mixer = 3 };
enum class YawControl : std::uint8_t { Surface = 0, Sideslip = 1, TailRotor = 2, Mixer = 3 };
/// What the thrust input (ControlInputs::throttle) moves: each engine's
/// throttle, a helicopter's collective pitch, or a multirotor's rotors'
/// thrust (each rotor's own, the mixer adding roll, pitch and yaw).
enum class ThrustControl : std::uint8_t { Throttle = 0, Collective = 1, RotorThrust = 2 };
/// What the aircraft does with the stick centred.
enum class NeutralStick : std::uint8_t { Surface = 0, PathHold = 1, OneG = 2 };

struct EffectorsSection {
    static constexpr std::uint16_t kVersion = 2; ///< 2: the rotorcraft's controls and `thrust`
    SectionHeader header;
    PitchControl pitch = PitchControl::Surface; ///< what the elevator input means
    RollControl roll = RollControl::Surface;
    YawControl yaw = YawControl::Surface;
    ThrustControl thrust = ThrustControl::Throttle;
    NeutralStick neutral = NeutralStick::Surface;
    // the support effectors it has; without the section, those an actuator command has always set
    bool flaps = true;
    bool retractableGear = true;
    bool wheelBrakes = true;
    bool speedbrake = false;
    bool speedbrakeApproach = false; ///< its drag devices are opened on an approach: airbrakes, not spoilers that dump lift (4.55)
    bool pitchTrim = false;
    double flapsTransitS = kUnknown; ///< full travel, s
    double gearTransitS = kUnknown;
};

// --- envelope -------------------------------------------------------------------------

struct EnvelopeSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    EnvelopeLimits clean, flaps;   ///< flaps: beyond `flapsThreshold`
    double flapsThreshold = 0.05;  ///< normalised flap position
    double gearCasMaxMs = kUnknown;
    /// On its wheels: the pitch attitude its tail touches at, pivoting on its aftmost wheels (a launch rotates short of it).
    double groundPitchMaxRad = kUnknown;
    /// On its wheels, a tail-wheel aircraft's: the pitch attitude it rests at on its main and tail wheels together (its
    /// two-point landing's, docs/flight-autonomy.md 4.65). Unknown for any other.
    double tailWheelPitchRad = kUnknown;
    /// On its wheels: its tightest turn, m, at the main wheels' middle (a taxi's corners are drawn wider).
    double groundTurnRadiusM = kUnknown;
    /// On its wheels: the yaw acceleration its steering gives it, full over, rad/s^2 - how quickly a turn there builds (a
    /// heavy's 0.03 to 0.07, a fighter's 0.4 to 0.7).
    double groundYawAccelRadS2 = kUnknown;
    /// The 90 deg crosswind it lands and takes off in at most, m/s: its type's published one, else its flying qualities'
    /// requirement (its design's [operations]; docs/flight-autonomy.md, 4.54). Beyond it a recovery waves off, and a
    /// launch or a taxi is refused.
    double crosswindMaxMs = kUnknown;
    // Limits the aircraft's own flight control law already enforces (a
    // fly-by-wire law's g and alpha limiters): protection clamps setpoints to
    // them but adds no feedback limiter of its own, so the two cannot fight.
    bool lawLoadFactor = false; ///< n_min and n_max
    bool lawAlpha = false;      ///< alpha_max
    bool lawRollRate = false;   ///< roll_rate_max
};

// --- propulsion -------------------------------------------------------------------------

enum class EngineType : std::uint8_t { Piston = 0, Turboprop = 1, Turbofan = 2, Turbojet = 3, Electric = 4, Turboshaft = 5, Unknown = 255 };

struct PropulsionSection {
    static constexpr std::uint16_t kVersion = 2; ///< 2: turboshaft engines
    SectionHeader header;
    int engines = 0;
    EngineType type = EngineType::Unknown;
    bool afterburner = false;
    double afterburnerThrottle = kUnknown; ///< the throttle at which it lights
    bool reverse = false;
    double spoolS = kUnknown;              ///< time constant, s
};

// --- plant ---------------------------------------------------------------------------------

struct FirstOrder {
    double tauS = kUnknown; ///< time constant, s
    double gain = kUnknown; ///< steady response per unit of control
};

/// How the aircraft answers its controls at a reference condition (hangar's identification).
struct PlantSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    double altitudeM = kUnknown, tasMs = kUnknown, easMs = kUnknown, massKg = kUnknown;
    FirstOrder roll, pitch, yaw, speed;
    double elevatorTrim = kUnknown, elevatorTrimLift = kUnknown;
    double alphaZeroLiftRad = kUnknown;
    double throttleTrim = kUnknown; ///< the throttle of level flight at the reference condition
};

// --- hover ----------------------------------------------------------------------------------------

/// One axis of a rotorcraft in the hover: d(rate)/dt = -damping rate + power u,
/// u the platform's command through the actuator's lag (and a flight model step).
struct RotorAxis {
    double power = kUnknown;   ///< body acceleration per unit command (rad/s2; the heave's m/s2, + up): signed, the platform's senses
    double damping = kUnknown; ///< 1/s
    double lagS = kUnknown;    ///< the actuator's time constant, s
};

/// How a rotorcraft answers its controls in the hover (hangar's
/// identification; docs/rotorcraft.md, 3.5): what its loops are designed
/// from, as the plant section is for a fixed wing.
struct HoverSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    double altitudeM = kUnknown, massKg = kUnknown;
    /// the commands that hold the hover, and the attitude it hovers at
    double throttleTrim = kUnknown, aileronTrim = kUnknown, elevatorTrim = kUnknown, rudderTrim = kUnknown;
    double rollAttitudeRad = kUnknown, pitchAttitudeRad = kUnknown;
    RotorAxis roll, pitch, yaw; ///< body rates p, q, r per unit aileron, elevator, rudder
    RotorAxis heave;            ///< vertical acceleration per unit throttle (the collective, or every rotor's thrust)
};

// --- performance ------------------------------------------------------------------------------

struct PerformanceSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    double stallCasMs = kUnknown, stallFlapsCasMs = kUnknown; ///< calibrated; flaps: all out, its gear down
    double maxTasMs = kUnknown, ceilingM = kUnknown, climbMs = kUnknown;
    double approachCasMs = kUnknown; ///< a published final approach speed, calibrated (docs/flight-autonomy.md, 4.63)
};

// --- control ----------------------------------------------------------------------------------

/// The aircraft's own gains for the built-in controllers (docs/sdk/control.md, per-aircraft gains).
struct ControlSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    std::vector<ControllerSetting> settings;
    /// The controllers, by registry id, its levels fly with instead of their
    /// defaults: what the laws designed from the plant choose (the attitude
    /// loop over pseudo-controls). Not carried in aircraft files.
    std::vector<std::pair<Level, std::string>> controllers;
};

// --- applicability --------------------------------------------------------------------------

/// A characteristic as the aircraft's design declares it (docs/flight-autonomy.md,
/// 5.2). Unknown when it does not: what the characteristic governs then stays
/// applicable (no evidence, no exception). The file writes 0 or 1.
enum class Declared : std::uint8_t { Unknown = 0, No = 1, Yes = 2 };
/// What the aircraft stands on (rule R2).
enum class GroundContact : std::uint8_t { Unknown = 0, Wheels = 1, Skids = 2, Legs = 3 };
/// How it operates from a ship (rules R3 to R5): a catapult launch and an
/// arrested landing, or vertically from a deck.
enum class CarrierOperations : std::uint8_t { Unknown = 0, None = 1, CatapultArrested = 2, Deck = 3 };
/// What makes drag on demand (rule R8): spoilers or airbrakes, or surfaces
/// its flight control system deflects together as a speedbrake.
enum class DragDevices : std::uint8_t { Unknown = 0, None = 1, Devices = 2, Surfaces = 3 };
/// What it carries and releases (rule R9): weapons on stations or in a bay,
/// palletized munitions from the hold, or dispensers.
enum class ReleasableStores : std::uint8_t { Unknown = 0, None = 1, Weapons = 2, Palletized = 3, Dispensers = 4 };

/// The characteristics, in the order ApplicabilitySection::sources keeps
/// their sources.
enum class Characteristic : std::uint8_t { VerticalFlight, GroundContact, Carrier, RetractableGear, Flaps, DragDevices, ReleasableStores, Aerobatic };
inline constexpr std::size_t kCharacteristicCount = 8;

/// The aircraft's physical characteristics as its design declares them, each
/// with the public source it rests on: what discovery's physical exceptions
/// rest on, and the evidence it reports for them (docs/flight-autonomy.md, 5).
struct ApplicabilitySection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    Declared verticalFlight = Declared::Unknown;                   ///< holds a point in the air (R1)
    GroundContact groundContact = GroundContact::Unknown;          ///< (R2)
    CarrierOperations carrier = CarrierOperations::Unknown;        ///< (R3 to R5)
    Declared retractableGear = Declared::Unknown;                  ///< (R6)
    Declared flaps = Declared::Unknown;                            ///< a flap function (R7)
    DragDevices dragDevices = DragDevices::Unknown;                ///< (R8)
    ReleasableStores releasableStores = ReleasableStores::Unknown; ///< (R9)
    Declared aerobatic = Declared::Unknown;                        ///< cleared for aerobatic manoeuvres (R10)
    /// Each declared characteristic's source, by Characteristic. An aircraft
    /// file gives them in its header (<reference refID="fsim/applicability/<name>"
    /// title="<source>"/>). A characteristic without one is not declared.
    std::array<std::string, kCharacteristicCount> sources;
};

/// A characteristic's name as the file writes it ("vertical_flight",
/// "ground_contact", ...); "" out of range.
FSIM_API const char* characteristicName(Characteristic c) noexcept;
/// Whether the section declares the characteristic: a value, and its source.
FSIM_API bool declares(const ApplicabilitySection& section, Characteristic c) noexcept;

// --- performance tables (docs/flight-autonomy.md, SUB-02; ADR-29 FA-3a) --------------------------

/// What the aircraft flies level, climbs and descends at, and the fuel it
/// burns, against altitude, weight and speed: flown by hangar (its
/// performance stage), clean, the envelope protection off. An aircraft file
/// carries it as fsim/tables/<axis>/<h|w|v><i> and fsim/tables/<name>/h<i>/w<j>[/v<k>].
/// Empty for an aircraft hangar has not flown them for (a stock JSBSim aircraft).
///
/// An aircraft that flies on a battery (FA-3b) has the power it gives where
/// one that burns fuel has its fuel flow: its best speeds are the least power's
/// and the most distance per joule's. A rotorcraft's run from the hover (its
/// least speed 0, where full power holds it); it has no stall and no idle
/// (NaN), and a multirotor no full-power climb (its model's thrust does not
/// fall with a climb's inflow: docs/flight-autonomy.md, 4.13).
struct TablesSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    std::vector<double> altitudeM;     ///< the altitudes flown, rising (the lowest a little above the ground)
    std::vector<double> weightKg;      ///< the weights flown, rising: the tanks a tenth, half and wholly full
    std::vector<double> speedFraction; ///< each condition's level points, from its least level speed (0) to 97 % of its top (1)
    double fuelCapacityKg = kUnknown;
    double batteryCapacityJ = kUnknown; ///< an aircraft that flies on a battery
    // per condition (an altitude and a weight), at [altitude * weights + weight]; NaN where not flown (above its ceiling)
    std::vector<double> minTasMs;             ///< the least speed full power flew level at (never below 1.15 times the stall)
    std::vector<double> maxTasMs;             ///< full power, level: the fastest it holds - where the drag meets the thrust
    /// The fastest a level acceleration at full power reaches from below: maxTasMs, but past a drag rise it cannot
    /// accelerate through (a fighter high and heavy) the speed it stops at, below the top it holds beyond it; NaN
    /// where only beyond such a drag rise does full power hold level (a height-holding mode reaches it no other way).
    std::vector<double> reachTasMs;
    std::vector<double> stallCasMs;           ///< idle, the height held: the limit angle of attack, or where the height could no longer be held
    std::vector<double> bestEnduranceTasMs, bestEnduranceFuelKgS; ///< the least fuel flow, level (a battery's: the least power)
    std::vector<double> bestRangeTasMs, bestRangeFuelKgS;         ///< the most distance per kilogram, level (a battery's: per joule)
    std::vector<double> bestEndurancePowerW, bestRangePowerW;     ///< a battery's power at those speeds
    std::vector<double> maxClimbMs, climbTasMs;                   ///< the best excess power at full power, and its speed (up high, a fighter's beyond its drag rise)
    // per level point, at [(altitude * weights + weight) * points + point]
    std::vector<double> fuelKgS;  ///< the fuel flow, level
    std::vector<double> powerW;   ///< the power a battery gives, level
    std::vector<double> psFullMs; ///< the excess power at full power, level: the climb it would make, or the speed it would gain, as a rate of height
    std::vector<double> psIdleMs; ///< the excess power at idle (negative): the descent rate at the speed, or the deceleration
    bool empty() const noexcept { return altitudeM.empty() || weightKg.empty(); }
};

/// The tables at a condition: linear in altitude and in weight between the
/// conditions flown (below the lowest altitude, its values; in weight, on
/// down to the tanks empty); NaN above the altitudes flown, or where a
/// condition it lies between was not flown.
struct TablesAt {
    double minTasMs = kUnknown, maxTasMs = kUnknown, reachTasMs = kUnknown, stallCasMs = kUnknown;
    double bestEnduranceTasMs = kUnknown, bestEnduranceFuelKgS = kUnknown, bestEndurancePowerW = kUnknown;
    double bestRangeTasMs = kUnknown, bestRangeFuelKgS = kUnknown, bestRangePowerW = kUnknown;
    double maxClimbMs = kUnknown, climbTasMs = kUnknown;
};
FSIM_API TablesAt tablesAt(const TablesSection& tables, double altitudeM, double weightKg) noexcept;
/// ...and at a true airspeed within its level speeds (from its least to its
/// top): the fuel flow (a battery's power) level, the excess power at full
/// power and at idle. Each altitude row is read at the same equivalent
/// airspeed (the drag changes little with height at one), and within a row
/// each weight's points at the same fraction of its speeds.
struct TablesAtSpeed {
    double fuelKgS = kUnknown, powerW = kUnknown, psFullMs = kUnknown, psIdleMs = kUnknown;
};
FSIM_API TablesAtSpeed tablesAt(const TablesSection& tables, double altitudeM, double weightKg, double tasMs) noexcept;
/// The service ceiling at a weight: rising through the altitudes flown,
/// where the best climb first falls below 0.5 m/s (100 ft/min) - an altitude
/// nothing held level at climbing nothing - linear between it and the one
/// below; climbing at the highest, the highest two's line extended, no
/// further than as high again. NaN if it climbs at none, or beyond that (a
/// rotorcraft's ceiling lies far above the altitudes it is flown at).
FSIM_API double tablesCeilingM(const TablesSection& tables, double weightKg) noexcept;

struct VehicleProfile {
    std::string aircraft;
    IdentitySection identity;
    EffectorsSection effectors;
    EnvelopeSection envelope;
    PropulsionSection propulsion;
    PlantSection plant;
    PerformanceSection performance;
    ControlSection control;
    HoverSection hover;
    ApplicabilitySection applicability;
    TablesSection tables;
};

/// A section by name ("identity", "effectors", "envelope", "propulsion",
/// "plant", "performance", "control", "hover", "applicability", "tables");
/// null for another name.
FSIM_API const SectionHeader* sectionHeader(const VehicleProfile& profile, std::string_view section) noexcept;
/// A field by its path as the aircraft file names it, in the unit its name
/// gives: "envelope/clean/n_max", "envelope/clean/alpha_max_deg",
/// "plant/roll/tau_s", "hover/roll/power", "identity/class", "<section>/version",
/// "control/pid_attitude/pitch/kp", "applicability/carrier",
/// "tables/max_tas_ms/h0/w2", "tables/altitude_m/h3"; NaN if the profile has
/// no such field, or does not declare it.
FSIM_API double profileValue(const VehicleProfile& profile, std::string_view path) noexcept;

} // namespace fsim::control
