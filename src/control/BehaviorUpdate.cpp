// An UPDATE of a behaviour's updatable parameters (docs/flight-autonomy.md, 4.58): the recovery's configuration, A-GRA's
// CleanUp and DirtyUp. Apart from the host's command paths, which it would otherwise grow (a NEW's and an UPDATE's code
// moved with it: 4.58's A/B).
#include "control/CapabilityHost.h"
#include "fsim/ControlStack.h"

#include <algorithm>

namespace fsim::control {

Reason CapabilityHost::amendBehavior(std::size_t slot, std::size_t capability, BehaviorCommand& kept, const BehaviorCommand& next,
                                     CommandResult& result) noexcept {
    // its parameters alone, each one it declares updatable, within its range
    if (!next.id.empty() && next.id != kept.id) return Reason::WrongCommandType;
    if (next.target != 0 || !next.points.empty()) return Reason::InvalidParameter;
    const std::vector<ParameterInfo>& parameters = catalog_->descriptor(capability).parameters;
    for (const auto& [name, value] : next.params) {
        const auto p = std::find_if(parameters.begin(), parameters.end(), [&n = name](const ParameterInfo& q) { return q.name == n; });
        if (p == parameters.end() || !p->updatable) return Reason::InvalidParameter;
        result.index = static_cast<std::int16_t>(p - parameters.begin());
        if (!isHold(value) && !(value >= p->min && value <= p->max)) return Reason::OutOfRange;
    }
    result.index = -1;
    // the behaviour flying it told - it may refuse; one pending or waiting reads them as it starts
    if (Behavior* running = slot != kNotFlying && runtime_ ? runtime_->runningBehavior(slot) : nullptr)
        if (const Reason why = running->amend(next); why != Reason::None) return why;
    for (const auto& [name, value] : next.params) kept.params[name] = value; // (not started again: no new revision)
    return Reason::None;
}

} // namespace fsim::control
