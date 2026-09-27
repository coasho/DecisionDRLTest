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
    double stallCasMs = kUnknown, stallFlapsCasMs = kUnknown; ///< calibrated
    double maxTasMs = kUnknown, ceilingM = kUnknown, climbMs = kUnknown;
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
};

/// A section by name ("identity", "effectors", "envelope", "propulsion",
/// "plant", "performance", "control", "hover", "applicability"); null for
/// another name.
FSIM_API const SectionHeader* sectionHeader(const VehicleProfile& profile, std::string_view section) noexcept;
/// A field by its path as the aircraft file names it, in the unit its name
/// gives: "envelope/clean/n_max", "envelope/clean/alpha_max_deg",
/// "plant/roll/tau_s", "hover/roll/power", "identity/class", "<section>/version",
/// "control/pid_attitude/pitch/kp", "applicability/carrier"; NaN if the
/// profile has no such field, or does not declare it.
FSIM_API double profileValue(const VehicleProfile& profile, std::string_view path) noexcept;

} // namespace fsim::control
