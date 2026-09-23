#pragma once

#include "fsim/Export.h"

#include <string>

class SGPropertyNode; // JSBSim property node; opaque outside the flight model

namespace fsim::sim {

/// Cached, O(1) read/write access to one property after a one-time lookup
/// (design 7.1). Null-safe: get() on an invalid handle returns 0.
class FSIM_API PropertyHandle {
public:
    PropertyHandle() = default;
    explicit PropertyHandle(SGPropertyNode* node) noexcept : node_(node) {}

    bool valid() const noexcept { return node_ != nullptr; }
    double get() const noexcept;
    /// Writes the property. A write can run model code (JSBSim's
    /// simulation/do_simple_trim trims the aircraft); if that fails, the error
    /// is logged and the write has no other effect.
    void set(double value) noexcept;
    /// The same, reporting failure: false, with the reason in `error`.
    bool trySet(double value, std::string& error) noexcept;

private:
    SGPropertyNode* node_ = nullptr;
};

} // namespace fsim::sim

namespace fsim {
using sim::PropertyHandle;
}
