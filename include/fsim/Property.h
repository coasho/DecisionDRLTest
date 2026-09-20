#pragma once

class SGPropertyNode; // JSBSim property node; opaque outside the flight model

namespace fsim::sim {

/// Cached, O(1) read/write access to one property after a one-time lookup
/// (design 7.1). Null-safe: get() on an invalid handle returns 0.
class PropertyHandle {
public:
    PropertyHandle() = default;
    explicit PropertyHandle(SGPropertyNode* node) noexcept : node_(node) {}

    bool valid() const noexcept { return node_ != nullptr; }
    double get() const noexcept;
    void set(double value) noexcept;

private:
    SGPropertyNode* node_ = nullptr;
};

} // namespace fsim::sim

namespace fsim {
using sim::PropertyHandle;
}
