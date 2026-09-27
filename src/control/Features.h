#pragma once

// The public features and a vehicle's support for each (docs/flight-autonomy.md,
// 4.2 and 5): what the platform has built, and what the aircraft's
// applicability rules exclude, with the evidence its design declares.

#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/VehicleProfile.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::control {

class CapabilityCatalog;

inline constexpr std::size_t kRuleCount = static_cast<std::size_t>(Rule::Count);

/// Which rules exclude an aircraft, and on what evidence: its declarations
/// (docs/flight-autonomy.md, 5.2), or for the platform's own effectors
/// (R12, R13) its profile. A rule no evidence decides excludes nothing.
struct RuleVerdicts {
    std::uint16_t excluded = 0; ///< ruleBit
    std::array<std::string, kRuleCount> evidence;
    bool excludes(Rule rule) const noexcept { return (excluded & ruleBit(rule)) != 0; }
};
RuleVerdicts judgeRules(const VehicleProfile& profile);

/// The rules a capability's own feature needs (0: every aircraft), and
/// whether the excluded ones (RuleVerdicts::excluded) exclude it: a catalog
/// must not offer it then.
std::uint16_t capabilityRules(std::string_view capability) noexcept;
bool excluded(std::uint16_t rules, std::uint16_t excludedRules) noexcept;

/// The A-GRA capability a platform behaviour's is superseded by
/// ("fsim.guidance.hold" -> "fsim.guidance.hsa"); "" for the others.
const char* supersededBy(std::string_view capability) noexcept;

/// The feature a command asks for, where the vehicle's catalog has no
/// capability for it: a behaviour's ("hover" -> "fsim.guidance.hover"), a
/// support effector's; "" for the others.
std::string featureOf(const Command& command);
std::string featureOf(const SupportCommand& command);

/// A vehicle's support for every public feature: built once per aircraft
/// (and per vehicle with a profile of its own), read without allocating.
class SupportTable {
public:
    SupportTable(const VehicleProfile& profile, const CapabilityCatalog& catalog);
    SupportTable(const SupportTable&) = delete;
    SupportTable& operator=(const SupportTable&) = delete;

    std::size_t size() const noexcept { return rows_.size(); }
    const SupportInfo& at(std::size_t index) const noexcept { return rows_[index]; }
    const std::vector<SupportInfo>& rows() const noexcept { return rows_; }
    /// By its public identifier, or a behaviour's registry id ("hover"); null if none.
    const SupportInfo* find(std::string_view feature) const noexcept;
    /// The reason a command for a feature the catalog does not offer is
    /// refused: NotSupported, NotImplemented, else UnknownCapability.
    Reason refusal(std::string_view feature) const noexcept;

private:
    std::vector<SupportInfo> rows_;
    std::vector<std::string> evidence_; ///< per row: what its `evidence` points into
};

} // namespace fsim::control
