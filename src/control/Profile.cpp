#include "control/Profile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim::control {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kInf = std::numeric_limits<double>::infinity();

/// How a field is stored in memory.
enum class Kind : std::uint8_t {
    Real,  ///< double, scaled from the file's unit
    Flag,  ///< bool, 0 or 1
    Code,  ///< an enum over std::uint8_t
    Count, ///< int
    Date,  ///< std::uint32_t, yyyymmdd
};

/// One field of a section as the aircraft file names it (relative to
/// fsim/<section>), its valid range in the file's unit, and where it lives.
struct Field {
    const char* section;
    const char* name;
    Kind kind;
    double lo, hi;
    double scale; ///< file unit -> memory (degrees -> radians)
    void* (*at)(VehicleProfile&);
};

#define AT(member) [](VehicleProfile& p) -> void* { return &p.member; }

const Field kFields[] = {
    {"identity", "class", Kind::Code, 0, 11, 1, AT(identity.aircraftClass)},
    {"identity", "family", Kind::Code, 0, 2, 1, AT(identity.family)},
    {"identity", "built", Kind::Date, 19000101, 99991231, 1, AT(identity.built)},

    {"effectors", "pitch", Kind::Code, 0, 2, 1, AT(effectors.pitch)},
    {"effectors", "roll", Kind::Code, 0, 1, 1, AT(effectors.roll)},
    {"effectors", "yaw", Kind::Code, 0, 1, 1, AT(effectors.yaw)},
    {"effectors", "neutral", Kind::Code, 0, 2, 1, AT(effectors.neutral)},
    {"effectors", "flaps", Kind::Flag, 0, 1, 1, AT(effectors.flaps)},
    {"effectors", "retractable_gear", Kind::Flag, 0, 1, 1, AT(effectors.retractableGear)},
    {"effectors", "wheel_brakes", Kind::Flag, 0, 1, 1, AT(effectors.wheelBrakes)},
    {"effectors", "speedbrake", Kind::Flag, 0, 1, 1, AT(effectors.speedbrake)},
    {"effectors", "pitch_trim", Kind::Flag, 0, 1, 1, AT(effectors.pitchTrim)},
    {"effectors", "flaps_transit_s", Kind::Real, 0, 600, 1, AT(effectors.flapsTransitS)},
    {"effectors", "gear_transit_s", Kind::Real, 0, 600, 1, AT(effectors.gearTransitS)},

#define LIMITS(cfg)                                                                                           \
    {"envelope", #cfg "/n_min", Kind::Real, -20, 1, 1, AT(envelope.cfg.loadFactorMin)},                       \
    {"envelope", #cfg "/n_max", Kind::Real, 1, 20, 1, AT(envelope.cfg.loadFactorMax)},                        \
    {"envelope", #cfg "/alpha_max_deg", Kind::Real, 0, 90, kDeg, AT(envelope.cfg.alphaMaxRad)},               \
    {"envelope", #cfg "/bank_max_deg", Kind::Real, 0, 180, kDeg, AT(envelope.cfg.bankMaxRad)},                \
    {"envelope", #cfg "/pitch_min_deg", Kind::Real, -90, 0, kDeg, AT(envelope.cfg.pitchMinRad)},              \
    {"envelope", #cfg "/pitch_max_deg", Kind::Real, 0, 90, kDeg, AT(envelope.cfg.pitchMaxRad)},               \
    {"envelope", #cfg "/roll_rate_max_deg_s", Kind::Real, 0, 1440, kDeg, AT(envelope.cfg.rollRateMaxRadS)},    \
    {"envelope", #cfg "/cas_min_ms", Kind::Real, 0, 1000, 1, AT(envelope.cfg.casMinMs)},                       \
    {"envelope", #cfg "/cas_max_ms", Kind::Real, 0, 2000, 1, AT(envelope.cfg.casMaxMs)},                       \
    {"envelope", #cfg "/mach_max", Kind::Real, 0, 10, 1, AT(envelope.cfg.machMax)}
    LIMITS(clean),
    LIMITS(flaps),
#undef LIMITS
    {"envelope", "flaps_threshold", Kind::Real, 0, 1, 1, AT(envelope.flapsThreshold)},
    {"envelope", "gear_cas_max_ms", Kind::Real, 0, 2000, 1, AT(envelope.gearCasMaxMs)},

    {"propulsion", "engines", Kind::Count, 0, 16, 1, AT(propulsion.engines)},
    {"propulsion", "type", Kind::Code, 0, 4, 1, AT(propulsion.type)},
    {"propulsion", "afterburner", Kind::Flag, 0, 1, 1, AT(propulsion.afterburner)},
    {"propulsion", "afterburner_throttle", Kind::Real, 0, 1, 1, AT(propulsion.afterburnerThrottle)},
    {"propulsion", "reverse", Kind::Flag, 0, 1, 1, AT(propulsion.reverse)},
    {"propulsion", "spool_s", Kind::Real, 0, 100, 1, AT(propulsion.spoolS)},

    {"plant", "altitude_m", Kind::Real, -500, 40000, 1, AT(plant.altitudeM)},
    {"plant", "tas_ms", Kind::Real, 0, 2000, 1, AT(plant.tasMs)},
    {"plant", "eas_ms", Kind::Real, 0, 2000, 1, AT(plant.easMs)},
    {"plant", "mass_kg", Kind::Real, 0, 1e6, 1, AT(plant.massKg)},
    {"plant", "roll/tau_s", Kind::Real, 0, 100, 1, AT(plant.roll.tauS)},
    {"plant", "roll/gain", Kind::Real, -kInf, kInf, 1, AT(plant.roll.gain)},
    {"plant", "pitch/tau_s", Kind::Real, 0, 100, 1, AT(plant.pitch.tauS)},
    {"plant", "pitch/gain", Kind::Real, -kInf, kInf, 1, AT(plant.pitch.gain)},
    {"plant", "yaw/tau_s", Kind::Real, 0, 100, 1, AT(plant.yaw.tauS)},
    {"plant", "yaw/gain", Kind::Real, -kInf, kInf, 1, AT(plant.yaw.gain)},
    {"plant", "speed/tau_s", Kind::Real, 0, 1000, 1, AT(plant.speed.tauS)},
    {"plant", "speed/gain", Kind::Real, -kInf, kInf, 1, AT(plant.speed.gain)},
    {"plant", "elevator_trim", Kind::Real, -1, 1, 1, AT(plant.elevatorTrim)},
    {"plant", "elevator_trim_lift", Kind::Real, -1, 1, 1, AT(plant.elevatorTrimLift)},
    {"plant", "alpha_zero_lift_deg", Kind::Real, -30, 30, kDeg, AT(plant.alphaZeroLiftRad)},

    {"performance", "stall_cas_ms", Kind::Real, 0, 1000, 1, AT(performance.stallCasMs)},
    {"performance", "stall_flaps_cas_ms", Kind::Real, 0, 1000, 1, AT(performance.stallFlapsCasMs)},
    {"performance", "max_tas_ms", Kind::Real, 0, 2000, 1, AT(performance.maxTasMs)},
    {"performance", "ceiling_m", Kind::Real, 0, 40000, 1, AT(performance.ceilingM)},
    {"performance", "climb_ms", Kind::Real, 0, 1000, 1, AT(performance.climbMs)},
};

#undef AT

struct SectionInfo {
    const char* name;
    std::uint16_t version; ///< the newest this build reads
};

const SectionInfo kSections[] = {
    {"identity", IdentitySection::kVersion},     {"effectors", EffectorsSection::kVersion},
    {"envelope", EnvelopeSection::kVersion},     {"propulsion", PropulsionSection::kVersion},
    {"plant", PlantSection::kVersion},           {"performance", PerformanceSection::kVersion},
    {"control", ControlSection::kVersion},
};

SectionHeader* headerPtr(VehicleProfile& p, std::string_view section) noexcept {
    if (section == "identity") return &p.identity.header;
    if (section == "effectors") return &p.effectors.header;
    if (section == "envelope") return &p.envelope.header;
    if (section == "propulsion") return &p.propulsion.header;
    if (section == "plant") return &p.plant.header;
    if (section == "performance") return &p.performance.header;
    if (section == "control") return &p.control.header;
    return nullptr;
}

const Field* findField(std::string_view section, std::string_view name) noexcept {
    for (const auto& f : kFields)
        if (section == f.section && name == f.name) return &f;
    return nullptr;
}

bool store(const Field& f, VehicleProfile& p, double v) noexcept {
    if (!std::isfinite(v) || v < f.lo || v > f.hi) return false;
    void* at = f.at(p);
    switch (f.kind) {
    case Kind::Real: *static_cast<double*>(at) = v * f.scale; return true;
    case Kind::Flag:
        if (v != 0.0 && v != 1.0) return false;
        *static_cast<bool*>(at) = v != 0.0;
        return true;
    case Kind::Code:
        if (v != std::floor(v)) return false;
        *static_cast<std::uint8_t*>(at) = static_cast<std::uint8_t>(v);
        return true;
    case Kind::Count:
        if (v != std::floor(v)) return false;
        *static_cast<int*>(at) = static_cast<int>(v);
        return true;
    case Kind::Date:
        if (v != std::floor(v)) return false;
        *static_cast<std::uint32_t*>(at) = static_cast<std::uint32_t>(v);
        return true;
    }
    return false;
}

double load(const Field& f, const VehicleProfile& p) noexcept {
    void* at = f.at(const_cast<VehicleProfile&>(p));
    switch (f.kind) {
    case Kind::Real: return *static_cast<const double*>(at) / f.scale;
    case Kind::Flag: return *static_cast<const bool*>(at) ? 1.0 : 0.0;
    case Kind::Code: return *static_cast<const std::uint8_t*>(at);
    case Kind::Count: return *static_cast<const int*>(at);
    case Kind::Date: return *static_cast<const std::uint32_t*>(at);
    }
    return kUnknown;
}

std::string format(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%g", v);
    return buf;
}

/// Limits that contradict each other go back to unknown.
void checkLimits(EnvelopeLimits& l, const char* cfg, const std::string& aircraft, std::vector<std::string>& warnings) {
    auto pair = [&](double& lo, double& hi, const char* what) {
        if (!std::isnan(lo) && !std::isnan(hi) && !(lo < hi)) {
            warnings.push_back(aircraft + ": fsim/envelope/" + cfg + " " + what + ": the lower limit is not below the upper; both ignored");
            lo = hi = kUnknown;
        }
    };
    pair(l.loadFactorMin, l.loadFactorMax, "load factor");
    pair(l.pitchMinRad, l.pitchMaxRad, "pitch");
    pair(l.casMinMs, l.casMaxMs, "airspeed");
}

} // namespace

VehicleProfile readProfile(const std::string& aircraft, const PropertySource& properties, std::vector<std::string>& warnings) {
    VehicleProfile p;
    p.aircraft = aircraft;
    for (const auto& info : kSections) {
        const std::string prefix = std::string("fsim/") + info.name;
        const auto values = properties(prefix);
        if (values.empty()) continue;
        double version = 1.0, provenance = static_cast<double>(Provenance::User);
        for (const auto& [name, value] : values) {
            if (name == "version") version = value;
            else if (name == "provenance") provenance = value;
        }
        if (!(version >= 1.0) || version != std::floor(version)) {
            warnings.push_back(aircraft + ": " + prefix + "/version " + format(version) + " is not a version; the section is ignored");
            continue;
        }
        if (version > info.version) {
            // A newer section is never half-read (docs/control-architecture.md, 7.3).
            warnings.push_back(aircraft + ": " + prefix + " is version " + format(version) + ", this build reads up to " +
                               std::to_string(info.version) + "; its defaults are used");
            continue;
        }
        // (A version older than the newest would be converted here; version 1 is the first.)
        SectionHeader& header = *headerPtr(p, info.name);
        header.version = static_cast<std::uint16_t>(version);
        header.provenance = provenance >= 0.0 && provenance <= 4.0 && provenance == std::floor(provenance)
                                ? static_cast<Provenance>(static_cast<std::uint8_t>(provenance))
                                : Provenance::User;
        if (std::string_view(info.name) == "control") {
            // fsim/control/<controller id>/<parameter, its dots written as slashes>
            for (const auto& [path, value] : values) {
                const auto slash = path.find('/');
                if (slash == std::string::npos || slash + 1 >= path.size()) continue; // version, provenance
                std::string parameter = path.substr(slash + 1);
                std::replace(parameter.begin(), parameter.end(), '/', '.');
                p.control.settings.push_back({path.substr(0, slash), std::move(parameter), value});
            }
            continue;
        }
        for (const auto& [name, value] : values) {
            if (name == "version" || name == "provenance") continue;
            const Field* f = findField(info.name, name);
            if (!f) warnings.push_back(aircraft + ": " + prefix + "/" + name + " is not a field this build knows; ignored");
            else if (!store(*f, p, value))
                warnings.push_back(aircraft + ": " + prefix + "/" + name + " = " + format(value) + " is out of range [" + format(f->lo) + ", " +
                                   format(f->hi) + "]; its default is used");
        }
    }
    checkLimits(p.envelope.clean, "clean", aircraft, warnings);
    checkLimits(p.envelope.flaps, "flaps", aircraft, warnings);
    return p;
}

VehicleProfile mergeProfile(const VehicleProfile& base, const VehicleProfile& over) {
    VehicleProfile p = base;
    if (over.identity.header.present()) p.identity = over.identity;
    if (over.effectors.header.present()) p.effectors = over.effectors;
    if (over.envelope.header.present()) p.envelope = over.envelope;
    if (over.propulsion.header.present()) p.propulsion = over.propulsion;
    if (over.plant.header.present()) p.plant = over.plant;
    if (over.performance.header.present()) p.performance = over.performance;
    if (over.control.header.present()) p.control = over.control;
    return p;
}

const SectionHeader* sectionHeader(const VehicleProfile& profile, std::string_view section) noexcept {
    return headerPtr(const_cast<VehicleProfile&>(profile), section);
}

double profileValue(const VehicleProfile& profile, std::string_view path) noexcept {
    const auto slash = path.find('/');
    if (slash == std::string_view::npos) return kUnknown;
    const std::string_view section = path.substr(0, slash), name = path.substr(slash + 1);
    const SectionHeader* header = sectionHeader(profile, section);
    if (!header) return kUnknown;
    if (name == "version") return header->version;
    if (name == "provenance") return static_cast<double>(header->provenance);
    if (section == "control") {
        // control/<controller>/<parameter as a path>
        const auto next = name.find('/');
        if (next == std::string_view::npos) return kUnknown;
        const std::string_view controller = name.substr(0, next);
        std::string parameter(name.substr(next + 1));
        std::replace(parameter.begin(), parameter.end(), '/', '.');
        for (const auto& s : profile.control.settings)
            if (s.controller == controller && s.parameter == parameter) return s.value;
        return kUnknown;
    }
    const Field* f = findField(section, name);
    return f ? load(*f, profile) : kUnknown;
}

} // namespace fsim::control
