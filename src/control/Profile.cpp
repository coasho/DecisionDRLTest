#include "control/Profile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

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
    // a characteristic an aircraft declares (the applicability section): not declared is
    // 0 in memory, and reads back as NaN
    Declared, ///< Declared, the file's 0 or 1 its No or Yes
    Choice,   ///< an enum over std::uint8_t, the file's codes from 1
};

/// One field of a section as the aircraft file names it (relative to
/// fsim/<section>), its valid range in the file's unit, and where it lives;
/// and the versions of its section it belongs to - a code a newer version
/// added is out of range in an older one, which is read as it always was.
struct Field {
    const char* section;
    const char* name;
    Kind kind;
    double lo, hi;
    double scale; ///< file unit -> memory (degrees -> radians)
    void* (*at)(VehicleProfile&);
    std::uint16_t since = 1, until = 0xFFFF;
};

#define AT(member) [](VehicleProfile& p) -> void* { return &p.member; }

const Field kFields[] = {
    {"identity", "class", Kind::Code, 0, 11, 1, AT(identity.aircraftClass), 1, 1},
    {"identity", "class", Kind::Code, 0, 13, 1, AT(identity.aircraftClass), 2},       // + helicopter, multirotor
    {"identity", "family", Kind::Code, 0, 2, 1, AT(identity.family), 1, 1},
    {"identity", "family", Kind::Code, 0, 4, 1, AT(identity.family), 2},               // + helicopter, multirotor
    {"identity", "built", Kind::Date, 19000101, 99991231, 1, AT(identity.built)},

    {"effectors", "pitch", Kind::Code, 0, 2, 1, AT(effectors.pitch), 1, 1},
    {"effectors", "pitch", Kind::Code, 0, 4, 1, AT(effectors.pitch), 2},              // + cyclic, mixer
    {"effectors", "roll", Kind::Code, 0, 1, 1, AT(effectors.roll), 1, 1},
    {"effectors", "roll", Kind::Code, 0, 3, 1, AT(effectors.roll), 2},
    {"effectors", "yaw", Kind::Code, 0, 1, 1, AT(effectors.yaw), 1, 1},
    {"effectors", "yaw", Kind::Code, 0, 3, 1, AT(effectors.yaw), 2},                  // + tail rotor, mixer
    {"effectors", "thrust", Kind::Code, 0, 2, 1, AT(effectors.thrust), 2},
    {"effectors", "neutral", Kind::Code, 0, 2, 1, AT(effectors.neutral)},
    {"effectors", "flaps", Kind::Flag, 0, 1, 1, AT(effectors.flaps)},
    {"effectors", "retractable_gear", Kind::Flag, 0, 1, 1, AT(effectors.retractableGear)},
    {"effectors", "wheel_brakes", Kind::Flag, 0, 1, 1, AT(effectors.wheelBrakes)},
    {"effectors", "speedbrake", Kind::Flag, 0, 1, 1, AT(effectors.speedbrake)},
    {"effectors", "speedbrake_approach", Kind::Flag, 0, 1, 1, AT(effectors.speedbrakeApproach)},
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
    {"envelope", "ground_pitch_max_deg", Kind::Real, 0, 90, kDeg, AT(envelope.groundPitchMaxRad)},
    {"envelope", "ground_turn_radius_m", Kind::Real, 0, 1000, 1, AT(envelope.groundTurnRadiusM)},
    {"envelope", "ground_yaw_accel_rad_s2", Kind::Real, 0, 100, 1, AT(envelope.groundYawAccelRadS2)},
    {"envelope", "crosswind_max_ms", Kind::Real, 0, 100, 1, AT(envelope.crosswindMaxMs)},
    {"envelope", "law_load_factor", Kind::Flag, 0, 1, 1, AT(envelope.lawLoadFactor)},
    {"envelope", "law_alpha", Kind::Flag, 0, 1, 1, AT(envelope.lawAlpha)},
    {"envelope", "law_roll_rate", Kind::Flag, 0, 1, 1, AT(envelope.lawRollRate)},

    {"propulsion", "engines", Kind::Count, 0, 16, 1, AT(propulsion.engines)},
    {"propulsion", "type", Kind::Code, 0, 4, 1, AT(propulsion.type), 1, 1},
    {"propulsion", "type", Kind::Code, 0, 5, 1, AT(propulsion.type), 2},              // + turboshaft
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
    {"plant", "throttle_trim", Kind::Real, 0, 1, 1, AT(plant.throttleTrim)},

    {"performance", "stall_cas_ms", Kind::Real, 0, 1000, 1, AT(performance.stallCasMs)},
    {"performance", "stall_flaps_cas_ms", Kind::Real, 0, 1000, 1, AT(performance.stallFlapsCasMs)},
    {"performance", "approach_cas_ms", Kind::Real, 0, 1000, 1, AT(performance.approachCasMs)},
    {"performance", "max_tas_ms", Kind::Real, 0, 2000, 1, AT(performance.maxTasMs)},
    {"performance", "ceiling_m", Kind::Real, 0, 40000, 1, AT(performance.ceilingM)},
    {"performance", "climb_ms", Kind::Real, 0, 1000, 1, AT(performance.climbMs)},

    {"hover", "altitude_m", Kind::Real, -500, 40000, 1, AT(hover.altitudeM)},
    {"hover", "mass_kg", Kind::Real, 0, 1e6, 1, AT(hover.massKg)},
    {"hover", "throttle_trim", Kind::Real, 0, 1, 1, AT(hover.throttleTrim)},
    {"hover", "aileron_trim", Kind::Real, -1, 1, 1, AT(hover.aileronTrim)},
    {"hover", "elevator_trim", Kind::Real, -1, 1, 1, AT(hover.elevatorTrim)},
    {"hover", "rudder_trim", Kind::Real, -1, 1, 1, AT(hover.rudderTrim)},
    {"hover", "roll_attitude_deg", Kind::Real, -90, 90, kDeg, AT(hover.rollAttitudeRad)},
    {"hover", "pitch_attitude_deg", Kind::Real, -90, 90, kDeg, AT(hover.pitchAttitudeRad)},
#define AXIS(axis)                                                                                  {"hover", #axis "/power", Kind::Real, -1e6, 1e6, 1, AT(hover.axis.power)},                      {"hover", #axis "/damping", Kind::Real, 0, 1000, 1, AT(hover.axis.damping)},                     {"hover", #axis "/lag_s", Kind::Real, 0, 100, 1, AT(hover.axis.lagS)}
    AXIS(roll),
    AXIS(pitch),
    AXIS(yaw),
    AXIS(heave),
#undef AXIS

    {"applicability", "vertical_flight", Kind::Declared, 0, 1, 1, AT(applicability.verticalFlight)},
    {"applicability", "ground_contact", Kind::Choice, 1, 3, 1, AT(applicability.groundContact)},
    {"applicability", "carrier", Kind::Choice, 1, 3, 1, AT(applicability.carrier)},
    {"applicability", "retractable_gear", Kind::Declared, 0, 1, 1, AT(applicability.retractableGear)},
    {"applicability", "flaps", Kind::Declared, 0, 1, 1, AT(applicability.flaps)},
    {"applicability", "drag_devices", Kind::Choice, 1, 3, 1, AT(applicability.dragDevices)},
    {"applicability", "releasable_stores", Kind::Choice, 1, 4, 1, AT(applicability.releasableStores)},
    {"applicability", "aerobatic", Kind::Declared, 0, 1, 1, AT(applicability.aerobatic)},
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
    {"control", ControlSection::kVersion},       {"hover", HoverSection::kVersion},
    {"applicability", ApplicabilitySection::kVersion}, {"tables", TablesSection::kVersion},
};

SectionHeader* headerPtr(VehicleProfile& p, std::string_view section) noexcept {
    if (section == "identity") return &p.identity.header;
    if (section == "effectors") return &p.effectors.header;
    if (section == "envelope") return &p.envelope.header;
    if (section == "propulsion") return &p.propulsion.header;
    if (section == "plant") return &p.plant.header;
    if (section == "performance") return &p.performance.header;
    if (section == "control") return &p.control.header;
    if (section == "hover") return &p.hover.header;
    if (section == "applicability") return &p.applicability.header;
    if (section == "tables") return &p.tables.header;
    return nullptr;
}

/// The field as a section of `version` has it (0: any version - they all
/// live in the same place).
const Field* findField(std::string_view section, std::string_view name, std::uint16_t version = 0) noexcept {
    for (const auto& f : kFields)
        if (section == f.section && name == f.name && (version == 0 || (version >= f.since && version <= f.until))) return &f;
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
    case Kind::Declared:
        if (v != 0.0 && v != 1.0) return false;
        *static_cast<Declared*>(at) = v != 0.0 ? Declared::Yes : Declared::No;
        return true;
    case Kind::Choice:
        if (v != std::floor(v)) return false;
        *static_cast<std::uint8_t*>(at) = static_cast<std::uint8_t>(v);
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
    case Kind::Declared: {
        const auto d = *static_cast<const Declared*>(at);
        return d == Declared::Unknown ? kUnknown : d == Declared::Yes ? 1.0 : 0.0;
    }
    case Kind::Choice: {
        const auto code = *static_cast<const std::uint8_t*>(at);
        return code == 0 ? kUnknown : code;
    }
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

const char* const kCharacteristicNames[kCharacteristicCount] = {
    "vertical_flight", "ground_contact", "carrier", "retractable_gear", "flaps", "drag_devices", "releasable_stores", "aerobatic",
};

/// The characteristic's value as its field stores it: 0 when not declared.
std::uint8_t declaredCode(const ApplicabilitySection& s, Characteristic c) noexcept {
    switch (c) {
    case Characteristic::VerticalFlight: return static_cast<std::uint8_t>(s.verticalFlight);
    case Characteristic::GroundContact: return static_cast<std::uint8_t>(s.groundContact);
    case Characteristic::Carrier: return static_cast<std::uint8_t>(s.carrier);
    case Characteristic::RetractableGear: return static_cast<std::uint8_t>(s.retractableGear);
    case Characteristic::Flaps: return static_cast<std::uint8_t>(s.flaps);
    case Characteristic::DragDevices: return static_cast<std::uint8_t>(s.dragDevices);
    case Characteristic::ReleasableStores: return static_cast<std::uint8_t>(s.releasableStores);
    case Characteristic::Aerobatic: return static_cast<std::uint8_t>(s.aerobatic);
    }
    return 0;
}

void undeclare(ApplicabilitySection& s, Characteristic c) noexcept {
    switch (c) {
    case Characteristic::VerticalFlight: s.verticalFlight = Declared::Unknown; break;
    case Characteristic::GroundContact: s.groundContact = GroundContact::Unknown; break;
    case Characteristic::Carrier: s.carrier = CarrierOperations::Unknown; break;
    case Characteristic::RetractableGear: s.retractableGear = Declared::Unknown; break;
    case Characteristic::Flaps: s.flaps = Declared::Unknown; break;
    case Characteristic::DragDevices: s.dragDevices = DragDevices::Unknown; break;
    case Characteristic::ReleasableStores: s.releasableStores = ReleasableStores::Unknown; break;
    case Characteristic::Aerobatic: s.aerobatic = Declared::Unknown; break;
    }
}

void appendUtf8(std::string& out, unsigned long code) {
    if (code < 0x80) {
        out += static_cast<char>(code);
    } else if (code < 0x800) {
        out += static_cast<char>(0xC0 | (code >> 6));
        out += static_cast<char>(0x80 | (code & 0x3F));
    } else if (code < 0x10000) {
        out += static_cast<char>(0xE0 | (code >> 12));
        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (code & 0x3F));
    } else if (code < 0x110000) {
        out += static_cast<char>(0xF0 | (code >> 18));
        out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (code & 0x3F));
    }
}

/// An attribute value's text, its entity and character references decoded.
std::string decodeXml(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        const auto semi = s[i] == '&' ? s.find(';', i) : std::string_view::npos;
        if (semi == std::string_view::npos) {
            out += s[i];
            continue;
        }
        const std::string_view name = s.substr(i + 1, semi - i - 1);
        if (name == "amp") out += '&';
        else if (name == "lt") out += '<';
        else if (name == "gt") out += '>';
        else if (name == "quot") out += '"';
        else if (name == "apos") out += '\'';
        else if (name.size() > 1 && name[0] == '#') {
            const bool hex = name[1] == 'x' || name[1] == 'X';
            appendUtf8(out, std::strtoul(std::string(name.substr(hex ? 2 : 1)).c_str(), nullptr, hex ? 16 : 10));
        } else {
            out += s[i];
            continue;
        }
        i = semi;
    }
    return out;
}

/// The attributes of the element whose name ends at `at` in `text`, up to
/// its end (a quoted value may hold a '>'): (name, value) pairs.
std::vector<std::pair<std::string_view, std::string>> attributes(std::string_view text, std::size_t at) {
    std::vector<std::pair<std::string_view, std::string>> out;
    auto space = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
    while (at < text.size()) {
        while (at < text.size() && space(text[at])) ++at;
        if (at >= text.size() || text[at] == '>' || text[at] == '/') break;
        const std::size_t nameAt = at;
        while (at < text.size() && text[at] != '=' && !space(text[at]) && text[at] != '>') ++at;
        const std::string_view name = text.substr(nameAt, at - nameAt);
        while (at < text.size() && space(text[at])) ++at;
        if (at >= text.size() || text[at] != '=') break;
        ++at;
        while (at < text.size() && space(text[at])) ++at;
        if (at >= text.size() || (text[at] != '"' && text[at] != '\'')) break;
        const char quote = text[at++];
        const auto end = text.find(quote, at);
        if (end == std::string_view::npos) break;
        out.emplace_back(name, decodeXml(text.substr(at, end - at)));
        at = end + 1;
    }
    return out;
}

} // namespace

const char* characteristicName(Characteristic c) noexcept {
    const auto i = static_cast<std::size_t>(c);
    return i < kCharacteristicCount ? kCharacteristicNames[i] : "";
}

bool declares(const ApplicabilitySection& section, Characteristic c) noexcept {
    const auto i = static_cast<std::size_t>(c);
    return i < kCharacteristicCount && declaredCode(section, c) != 0 && !section.sources[i].empty();
}

void readApplicabilitySources(const std::filesystem::path& aircraftFile, ApplicabilitySection& section) {
    std::ifstream in(aircraftFile, std::ios::binary);
    if (!in) return;
    // the header opens the file: read to its end
    std::string text, line;
    while (text.size() < (1u << 20) && std::getline(in, line)) {
        text += line;
        text += '\n';
        if (line.find("</fileheader>") != std::string::npos) break;
    }
    static constexpr std::string_view kTag = "<reference", kPrefix = "fsim/applicability/";
    for (auto at = text.find(kTag); at != std::string::npos; at = text.find(kTag, at + kTag.size())) {
        std::string_view ref;
        std::string title;
        const auto attrs = attributes(text, at + kTag.size());
        for (const auto& [name, value] : attrs) {
            if (name == "refID") ref = value;
            else if (name == "title") title = value;
        }
        if (ref.substr(0, kPrefix.size()) != kPrefix) continue;
        ref.remove_prefix(kPrefix.size());
        for (std::size_t i = 0; i < kCharacteristicCount; ++i)
            if (ref == kCharacteristicNames[i]) section.sources[i] = title;
    }
}

void requireSources(const std::string& aircraft, ApplicabilitySection& section, std::vector<std::string>& warnings) {
    for (std::size_t i = 0; i < kCharacteristicCount; ++i) {
        const auto c = static_cast<Characteristic>(i);
        const bool value = declaredCode(section, c) != 0, source = !section.sources[i].empty();
        if (value && !source) {
            warnings.push_back(aircraft + ": fsim/applicability/" + kCharacteristicNames[i] +
                               " is declared without a source: not a declaration, and what it governs stays applicable");
            undeclare(section, c);
        } else if (!value && source) {
            warnings.push_back(aircraft + ": fsim/applicability/" + kCharacteristicNames[i] + " has a source but no value; ignored");
            section.sources[i].clear();
        }
    }
}

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
        // An older version is read as it was: its fields, its codes (every
        // version so far only added to the one before, so nothing converts).
        SectionHeader& header = *headerPtr(p, info.name);
        header.version = static_cast<std::uint16_t>(version);
        header.provenance = provenance >= 0.0 && provenance <= 4.0 && provenance == std::floor(provenance)
                                ? static_cast<Provenance>(static_cast<std::uint8_t>(provenance))
                                : Provenance::User;
        if (std::string_view(info.name) == "tables") { // (arrays: read apart, PerformanceTables.cpp)
            readTables(values, p.tables, aircraft + ": " + prefix, warnings);
            continue;
        }
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
            const Field* f = findField(info.name, name, header.version);
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
    if (over.hover.header.present()) p.hover = over.hover;
    if (over.applicability.header.present()) p.applicability = over.applicability;
    if (over.tables.header.present()) p.tables = over.tables;
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
    if (section == "tables") return tableValue(profile.tables, name);
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
