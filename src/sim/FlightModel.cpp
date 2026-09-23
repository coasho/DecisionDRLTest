#include "sim/FlightModel.h"

#include "core/Log.h"

#include <simgear/props/props.hxx>

#include <exception>

namespace fsim::sim {

double PropertyHandle::get() const noexcept { return node_ ? node_->getDoubleValue() : 0.0; }

// A tied property's setter is model code and may throw (JSBSim's trim throws
// TrimFailureException); nothing may escape a noexcept function, or the
// process terminates.
bool PropertyHandle::trySet(double value, std::string& error) noexcept {
    if (!node_) {
        error = "invalid property";
        return false;
    }
    try {
        node_->setDoubleValue(value);
        return true;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown error";
    }
    return false;
}

void PropertyHandle::set(double value) noexcept {
    std::string error;
    if (node_ && !trySet(value, error)) LOG_WARN("sim") << "property " << node_->getPath() << " = " << value << ": " << error;
}

} // namespace fsim::sim
