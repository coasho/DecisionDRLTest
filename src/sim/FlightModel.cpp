#include "sim/FlightModel.h"

#include <simgear/props/props.hxx>

namespace fsim::sim {

double PropertyHandle::get() const noexcept { return node_ ? node_->getDoubleValue() : 0.0; }

void PropertyHandle::set(double value) noexcept {
    if (node_) node_->setDoubleValue(value);
}

} // namespace fsim::sim
