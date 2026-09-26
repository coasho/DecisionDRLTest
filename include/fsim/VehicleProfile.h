#pragma once

// What differs between aircraft, as data (docs/control-architecture.md,
// section 7): an aggregate of small sections, each with its own version and
// provenance, produced and read independently. An aircraft carries them as
// JSBSim properties fsim/<section>/<field> in its flight control section (the
// gains of fsim/control are the control section); a trainer can supply
// sections for a vehicle of its own (VehicleSpec::profile).

#include "fsim/ControlStack.h"
#include "fsim/Export.h"

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
    Unknown = 0, LightGa, Fighter, Attack, Bomber, Transport, Tanker, Aew, Reconnaissance, ElectronicWarfare, Uav, Trainer
};
/// How the controls reach the surfaces: JSBSim's own FCS (unknown), surfaces, a fly-by-wire law.
enum class ControlFamily : std::uint8_t { Stock = 0, Direct = 1, FlyByWire = 2 };

struct IdentitySection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    AircraftClass aircraftClass = AircraftClass::Unknown;
    ControlFamily family = ControlFamily::Stock;
    std::uint32_t built = 0; ///< yyyymmdd
};

// --- effectors ------------------------------------------------------------------------

enum class PitchControl : std::uint8_t { Surface = 0, LoadFactor = 1, PitchRate = 2 };
enum class RollControl : std::uint8_t { Surface = 0, RollRate = 1 };
enum class YawControl : std::uint8_t { Surface = 0, Sideslip = 1 };
/// What the aircraft does with the stick centred.
enum class NeutralStick : std::uint8_t { Surface = 0, PathHold = 1, OneG = 2 };

struct EffectorsSection {
    static constexpr std::uint16_t kVersion = 1;
    SectionHeader header;
    PitchControl pitch = PitchControl::Surface; ///< what the elevator input means
    RollControl roll = RollControl::Surface;
    YawControl yaw = YawControl::Surface;
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

enum class EngineType : std::uint8_t { Piston = 0, Turboprop = 1, Turbofan = 2, Turbojet = 3, Electric = 4, Unknown = 255 };

struct PropulsionSection {
    static constexpr std::uint16_t kVersion = 1;
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

struct VehicleProfile {
    std::string aircraft;
    IdentitySection identity;
    EffectorsSection effectors;
    EnvelopeSection envelope;
    PropulsionSection propulsion;
    PlantSection plant;
    PerformanceSection performance;
    ControlSection control;
};

/// A section by name ("identity", "effectors", "envelope", "propulsion",
/// "plant", "performance", "control"); null for another name.
FSIM_API const SectionHeader* sectionHeader(const VehicleProfile& profile, std::string_view section) noexcept;
/// A field by its path as the aircraft file names it, in the unit its name
/// gives: "envelope/clean/n_max", "envelope/clean/alpha_max_deg",
/// "plant/roll/tau_s", "identity/class", "<section>/version",
/// "control/pid_attitude/pitch/kp"; NaN if the profile has no such field.
FSIM_API double profileValue(const VehicleProfile& profile, std::string_view path) noexcept;

} // namespace fsim::control
