// The altitude stacked marshall's slots (docs/flight-autonomy.md, 4.46; A-GRA's ALTITUDE_STACKED_MARSHALL): the world, which sees
// every vehicle, chooses each aircraft's altitude in its stack - between steps, on the caller's thread, so the same commands
// choose the same slots - and hands the marshall to the vehicle's host with it.
#include "session/World.h"

#include "core/Geodesy.h"

#include <cmath>
#include <limits>

namespace fsim::session {

namespace {

/// Two marshalls round the same point: their centres within this.
constexpr double kSamePointM = 100.0;

control::CommandResult unknownActivity(control::ActivityId activity) {
    control::CommandResult r;
    r.reason = control::Reason::UnknownActivity;
    r.activity = activity;
    return r;
}

/// A refused UPDATE echoes the command id of the activity it addressed, as World.cpp's do.
void echo(const control::CapabilityHost& host, control::CommandResult& r) noexcept {
    if (r.status == control::CommandStatus::Rejected && r.activity && !r.commandId)
        if (const control::ActivityRecord* a = host.activity(r.activity)) r.commandId = a->commandId;
}

} // namespace

control::Reason World::slotMarshall(const Entry& e, control::MarshallCommand& m) const {
    using namespace control;
    // its stack as given: what is malformed the host refuses, naming its field, as it would any NEW
    if (isHold(m.altitudeMinM) || !std::isfinite(m.altitudeMinM)) return Reason::None;
    const double separation = isHold(m.separationM) ? kMarshallSeparationM : m.separationM;
    if (!std::isfinite(separation) || !(separation > 0.0)) return Reason::None;
    if (!isHold(m.altitudeMaxM) && !(m.altitudeMaxM >= m.altitudeMinM)) return Reason::None;
    if (!isHold(m.altitudeM) && !(m.altitudeM >= m.altitudeMinM && (isHold(m.altitudeMaxM) || m.altitudeM <= m.altitudeMaxM))) return Reason::None;
    const double most = isHold(m.altitudeMaxM) ? std::numeric_limits<double>::infinity() : m.altitudeMaxM;
    // its centre as given, else where the aircraft is (the pattern's own); its reference
    const sim::VehicleState& own = pool_->states()[e.slot];
    const double lat = isHold(m.latitudeRad) ? own.latitudeRad : m.latitudeRad, lon = isHold(m.longitudeRad) ? own.longitudeRad : m.longitudeRad;
    const double reference = isHold(m.altitudeReference) ? 0.0 : m.altitudeReference;
    // the others' slots: every other aircraft's live or waiting marshall round the same point, in the same reference (one
    // guidance activity flies on a vehicle at a time: its own are left out)
    std::vector<double> taken;
    MarshallCommand peer;
    for (const auto& other : entries_) {
        if (!other || other.get() == &e) continue;
        for (const ActivityRecord& a : other->host.activities()) {
            if (!a.live() || !other->host.marshall(a.id, peer)) continue;
            if (isHold(peer.altitudeM) || (isHold(peer.altitudeReference) ? 0.0 : peer.altitudeReference) != reference) continue;
            const sim::VehicleState& there = pool_->states()[other->slot];
            const double plat = isHold(peer.latitudeRad) ? there.latitudeRad : peer.latitudeRad;
            const double plon = isHold(peer.longitudeRad) ? there.longitudeRad : peer.longitudeRad;
            if (geo::distanceM(lat, lon, plat, plon) <= kSamePointM) taken.push_back(peer.altitudeM);
        }
    }
    auto clear = [&](double h) {
        for (const double t : taken)
            if (std::abs(h - t) < separation - 1e-6) return false;
        return true;
    };
    if (!isHold(m.altitudeM)) return clear(m.altitudeM) ? Reason::None : Reason::StackFull; // (the slot asked for)
    // the lowest clear from its least, in steps of its separation (each taken slot rules out two at most)
    for (std::size_t k = 0; k <= 2 * taken.size(); ++k) {
        const double h = m.altitudeMinM + static_cast<double>(k) * separation;
        if (h > most + 1e-6) break;
        if (clear(h)) {
            m.altitudeM = h;
            return Reason::None;
        }
    }
    return Reason::StackFull;
}

control::CommandResult World::submit(std::uint32_t id, const control::MarshallCommand& marshall, const control::PatternShape& shape,
                                     const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::MarshallCommand m = marshall;
    control::CommandResult r;
    if (const control::Reason why = slotMarshall(*e, m); why != control::Reason::None)
        r = e->host.refused(why, 3); // (its slot: none free, or the one asked for taken)
    else
        r = e->host.submit(m, shape, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape) {
    return updateMarshall(control::Source::Policy, activity, marshall, &shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                     const control::PatternShape& shape) {
    return updateMarshall(caller, activity, marshall, &shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::MarshallCommand& marshall) {
    return updateMarshall(control::Source::Policy, activity, marshall, nullptr);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall) {
    return updateMarshall(caller, activity, marshall, nullptr);
}

control::CommandResult World::updateMarshall(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                             const control::PatternShape* shape) {
    using namespace control;
    Entry* e = entry(activityVehicle(activity));
    if (!e) return unknownActivity(activity);
    MarshallCommand m = marshall;
    // its stack moved - its least, most or separation, its centre or reference given - or a slot asked for: chosen afresh, as the
    // marshall will be; else it keeps its own
    const bool moves = !isHold(m.altitudeMinM) || !isHold(m.altitudeMaxM) || !isHold(m.separationM) || !isHold(m.latitudeRad) ||
                       !isHold(m.longitudeRad) || !isHold(m.altitudeReference) || !isHold(m.altitudeM);
    if (MarshallCommand next; moves && e->host.marshall(activity, next)) { // (else it keeps its own: the host answers the rest)
        mergeMarshall(next, m);
        if (isHold(m.altitudeM)) next.altitudeM = kHold;
        if (const Reason why = slotMarshall(*e, next); why != Reason::None) {
            CommandResult r = e->host.refused(why, 3, activity);
            echo(e->host, r);
            return r;
        }
        m.altitudeM = next.altitudeM;
    }
    CommandResult r = e->host.update(activity, m, shape, pool_->states()[e->slot], caller);
    echo(e->host, r);
    return r;
}

} // namespace fsim::session
