// A route's required time of arrival, as its behaviour estimates it (docs/flight-autonomy.md, 4.33; ADR-29 FA-6d1): asked
// of the behaviour through the stack and the host, apart from the progress every activity record carries. Its own
// translation unit, last in the library: what is added before the runtime's code moves it (the A/B's lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Features.h"
#include "control/Route.h"
#include "fsim/ControlStack.h"
#include "fsim/GuidanceModes.h"

#include "fsim/VehicleProfile.h"

#include <algorithm>

namespace fsim::control {

namespace {

bool aboveGround(double reference) noexcept { return reference == static_cast<double>(AltitudeReference::AboveGround); }

} // namespace

bool ControlStack::arrival(std::size_t slot, ArrivalEstimate& out) const noexcept {
    if (slot >= kSlotCount || !behaviors_[slot] || started_[slot] != config_->slots[slot].generation) return false;
    return behaviors_[slot]->arrival(out);
}

bool CapabilityHost::arrival(ActivityId activity, ArrivalEstimate& out) const noexcept {
    const int s = liveSlot(activity);
    return s >= 0 && isCascade(static_cast<std::size_t>(s)) && runtime_->arrival(static_cast<std::size_t>(s), out);
}

Reason CapabilityHost::checkArrivals(const route::Plan& p, const sim::VehicleState& state, CommandResult& detail) const noexcept {
    const double worldNow = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    bool afterLoiter = false;
    for (std::uint32_t i = p.start; i < p.count; ++i) {
        const Waypoint& w = p.points[i];
        afterLoiter = afterLoiter || route::loiterPoint(w);
        if (isHold(w.arrivalBeginS) && isHold(w.arrivalEndS)) continue;
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        if (w.arrivalEndS < worldNow) return Reason::InvalidWaypoint;
        const SupportInfo* row = support_ ? support_->find("fsim.guidance.route/required_time_of_arrival") : nullptr;
        if (afterLoiter || !row || row->support == Support::NotImplemented) return Reason::NotImplemented;
        detail.index = -1;
    }
    return Reason::None;
}

void CapabilityHost::limitArrivals(const route::Plan& p, const sim::VehicleState& state, CheckLog& log) const noexcept {
    const double worldNow = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    for (std::uint32_t i = p.start; i < p.count; ++i) {
        const Waypoint& w = p.points[i];
        if (isHold(w.arrivalBeginS) && isHold(w.arrivalEndS)) continue;
        const double h = aboveGround(w.altitudeReference) ? state.altitudeMslM
                                                          : altitudeMslOf(w.altitudeM, static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)),
                                                                          state, &config_->altimeter);
        double least = kUnknown, most = kUnknown;
        route::levelSpeedsMs(config_->tables, performance_, hovers, h, state.fuelKg, least, most);
        const double alongM = p.arrivalM(i, true);
        const auto index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        if (most > 0.0 && w.arrivalEndS < worldNow + alongM / most - 1.0) log.find(Reason::PerformanceLimit, index, Constraint::MaxAirspeed);
        else if (least > 0.0 && w.arrivalBeginS > worldNow + alongM / least + 1.0) log.find(Reason::PerformanceLimit, index, Constraint::MinAirspeed);
    }
}

} // namespace fsim::control
