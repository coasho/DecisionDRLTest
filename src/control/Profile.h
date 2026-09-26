#pragma once

// Reading a VehicleProfile from its carriers (docs/control-architecture.md,
// 7.3 and 7.4): the aircraft's properties under fsim/<section>, section by
// section, each checked against its version, field by field against its
// range; and a trainer's own sections laid over them.

#include "fsim/VehicleProfile.h"

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

/// `base` with every section `over` has (present()) replaced by `over`'s.
VehicleProfile mergeProfile(const VehicleProfile& base, const VehicleProfile& over);

} // namespace fsim::control
