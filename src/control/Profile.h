#pragma once

// Reading a VehicleProfile from its carriers (docs/control-architecture.md,
// 7.3 and 7.4): the aircraft's properties under fsim/<section>, section by
// section, each checked against its version, field by field against its
// range; and a trainer's own sections laid over them.

#include "fsim/VehicleProfile.h"

#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::control {

/// (path below the prefix, value) for every leaf under a property prefix,
/// e.g. FlightModel::properties("fsim/envelope") -> ("clean/n_max", 9), ...
using PropertySource = std::function<std::vector<std::pair<std::string, double>>(std::string_view prefix)>;

/// Every section the aircraft carries; what it does not carry keeps its
/// defaults (version 0). A section newer than this build knows is skipped,
/// a field out of range or inconsistent falls back to its default, a field
/// nobody knows is ignored: each noted in `warnings`.
VehicleProfile readProfile(const std::string& aircraft, const PropertySource& properties, std::vector<std::string>& warnings);

/// The tables section's properties (relative to fsim/tables) into `out`
/// (PerformanceTables.cpp): the axes, then each table sized by them; a cell
/// out of the axes, or a name this build does not know, warned of (`where`
/// names the section) and ignored.
void readTables(const std::vector<std::pair<std::string, double>>& values, TablesSection& out, const std::string& where,
                std::vector<std::string>& warnings);
/// A tables field by its path below the section ("max_tas_ms/h0/w2", "altitude_m/h3", "fuel_capacity_kg"); NaN if none.
double tableValue(const TablesSection& tables, std::string_view path) noexcept;

/// `base` with every section `over` has (present()) replaced by `over`'s.
VehicleProfile mergeProfile(const VehicleProfile& base, const VehicleProfile& over);

/// The sources of the applicability section's declarations, from the header
/// of the aircraft file (<reference refID="fsim/applicability/<name>"
/// title="<source>"/>, as hangar writes them): each one found replaces the
/// section's. A file that cannot be read gives none.
void readApplicabilitySources(const std::filesystem::path& aircraftFile, ApplicabilitySection& section);

/// No evidence, no exception (docs/flight-autonomy.md, 5.2): a characteristic
/// declared without a source goes back to not declared, and a source without
/// a value is dropped; each noted in `warnings`.
void requireSources(const std::string& aircraft, ApplicabilitySection& section, std::vector<std::string>& warnings);

} // namespace fsim::control
