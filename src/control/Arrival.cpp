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
#include <limits>

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

void CapabilityHost::limitArrivals(route::Plan& p, const sim::VehicleState& state, CheckLog& log) const noexcept {
    const double worldNow = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    route::limitClimbs(p, state, config_->tables, performance_, hovers, &config_->altimeter); // (its climbs, no faster than it climbs them)
    // each timed target in turn along its first lap - its states' times (4.34), then its point's window - from the
    // earliest and the latest it can be at the one before (at first, where the aircraft is now), at the speeds it flies
    // level at the target's altitude: where it cannot make one, as early as it can, or as late
    double fromM = 0.0, earliest = worldNow, latest = worldNow;
    auto target = [&](double alongM, double begin, double end, double altitudeM, double reference, std::uint32_t point) {
        const double h = aboveGround(reference) ? state.altitudeMslM
                                                : altitudeMslOf(altitudeM, static_cast<AltitudeReference>(static_cast<int>(reference)), state, &config_->altimeter);
        double least = kUnknown, most = kUnknown;
        route::levelSpeedsMs(config_->tables, performance_, hovers, h, state.fuelKg, least, most);
        const double d = std::max(alongM - fromM, 0.0);
        const double first = !(most > 0.0) ? earliest : p.climbCount ? earliest + p.climbTimeS(fromM, alongM, most, 0.0) : earliest + d / most;
        const double last = least > 0.0 ? latest + d / least : std::numeric_limits<double>::infinity();
        const auto index = static_cast<std::int16_t>(std::min<std::uint32_t>(point, 0x7FFF));
        fromM = alongM;
        if (end < first - 1.0) {
            log.find(Reason::PerformanceLimit, index, Constraint::MaxAirspeed);
            earliest = latest = first;
        } else if (begin > last + 1.0) {
            log.find(Reason::PerformanceLimit, index, Constraint::MinAirspeed);
            earliest = latest = last;
        } else {
            earliest = isHold(begin) ? first : std::max(first, begin), latest = isHold(end) ? last : std::min(last, end);
            if (latest < earliest) latest = earliest; // (within its second's slack)
        }
    };
    std::uint32_t j = 0;
    for (std::uint32_t i = p.start; i < p.count; ++i) {
        const Waypoint& w = p.points[i];
        for (; j < p.stateCount && p.states[j].point <= i; ++j) {
            const RouteState& s = p.states[j];
            if (s.point == i && !isHold(s.timeS))
                target(p.stateLapM[j], s.timeS, s.timeS, isHold(s.altitudeM) ? w.altitudeM : s.altitudeM, w.altitudeReference, i);
        }
        if (!isHold(w.arrivalBeginS) || !isHold(w.arrivalEndS)) target(p.arrivalM(i, true), w.arrivalBeginS, w.arrivalEndS, w.altitudeM, w.altitudeReference, i);
    }
}

} // namespace fsim::control
