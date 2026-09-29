// A-GRA's route intercept (docs/flight-autonomy.md, 4.47; ROUTE_INTERCEPT): the world's calls, each the vehicle's host's - its
// plan, its join and its checks are the host's, which keeps the plans.
#include "session/World.h"

namespace fsim::session {

control::CommandResult World::submit(std::uint32_t id, const control::InterceptCommand& intercept, const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(intercept, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::update(control::ActivityId activity, const control::InterceptCommand& intercept) {
    return update(control::Caller{}, activity, intercept);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::InterceptCommand& intercept) {
    Entry* e = entry(control::activityVehicle(activity));
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownActivity;
        r.activity = activity;
        return r;
    }
    control::CommandResult r = e->host.update(activity, intercept, pool_->states()[e->slot], caller);
    if (r.status == control::CommandStatus::Rejected && r.activity && !r.commandId) // (its command id echoed, as World.cpp's are)
        if (const control::ActivityRecord* a = e->host.activity(r.activity)) r.commandId = a->commandId;
    return r;
}

bool World::interceptStatus(control::ActivityId activity, control::InterceptStatus& out) const {
    const Entry* e = entry(control::activityVehicle(activity));
    return e && e->host.interceptStatus(activity, pool_->states()[e->slot], out);
}

} // namespace fsim::session
