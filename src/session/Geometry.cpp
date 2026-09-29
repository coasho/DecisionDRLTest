// Operational geometry (docs/flight-autonomy.md, 4.42; A-GRA's OpPoint): the world's store of it, by id, for the commands
// that name it (a must fly). Set and asked for between steps, never stepped.
#include "session/World.h"

#include "control/Zones.h"

#include <cmath>

namespace fsim::session {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool code(double v, double count) noexcept { return control::isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return control::isHold(v) || std::isfinite(v); }

} // namespace

control::Reason World::setOpPoint(const control::OpPoint& point) {
    using control::isHold;
    const control::OpPoint& p = point;
    const bool place = !isHold(p.latitudeRad) || !isHold(p.longitudeRad), framed = !isHold(p.frame);
    const bool bad =
        p.id == 0 || place == framed || !finiteOr(p.latitudeRad) || !finiteOr(p.longitudeRad) || !finiteOr(p.altitudeM) || !finiteOr(p.frameXM) ||
        !finiteOr(p.frameYM) || !finiteOr(p.frameZM) || !finiteOr(p.ingressMinRad) || !finiteOr(p.ingressMaxRad) ||
        (place && (isHold(p.latitudeRad) || isHold(p.longitudeRad) || std::abs(p.latitudeRad) > 0.5 * kPi)) ||
        (framed && !(p.frame == std::floor(p.frame) && p.frame >= 1.0 && frames_.count(static_cast<control::FrameId>(p.frame)))) ||
        (!framed && (!isHold(p.frameRotation) || !isHold(p.frameOffsets) || !isHold(p.frameXM) || !isHold(p.frameYM) || !isHold(p.frameZM))) ||
        !code(p.frameRotation, static_cast<double>(control::FrameRotation::Count)) ||
        !code(p.frameOffsets, static_cast<double>(control::FrameOffsets::Count)) ||
        !code(p.altitudeReference, static_cast<double>(control::AltitudeReference::Count)) || (!isHold(p.altitudeReference) && isHold(p.altitudeM)) ||
        isHold(p.ingressMinRad) != isHold(p.ingressMaxRad) || (!isHold(p.ingressMinRad) && std::abs(p.ingressMinRad) > kPi) ||
        (!isHold(p.ingressMaxRad) && std::abs(p.ingressMaxRad) > kPi);
    if (bad) return control::Reason::InvalidParameter;
    control::OpPoint& kept = opPoints_[p.id];
    const std::uint32_t revision = kept.revision + 1;
    kept = p;
    kept.revision = revision;
    return control::Reason::None;
}

bool World::removeOpPoint(control::OpPointId id) { return opPoints_.erase(id) > 0; }

std::vector<control::OpPointId> World::opPoints() const {
    std::vector<control::OpPointId> out;
    out.reserve(opPoints_.size());
    for (const auto& [id, p] : opPoints_) out.push_back(id);
    return out;
}

std::optional<control::OpPoint> World::opPoint(control::OpPointId id) const {
    const auto it = opPoints_.find(id);
    if (it == opPoints_.end()) return std::nullopt;
    return it->second;
}

bool World::Answers::opPoint(control::OpPointId id, control::OpPoint& out) const {
    const auto it = world_.opPoints_.find(id);
    if (it == world_.opPoints_.end()) return false;
    out = it->second;
    return true;
}

control::Reason World::setOpZone(const control::OpZone& zone) {
    const bool known = control::isHold(zone.frame) || (zone.frame == std::floor(zone.frame) && zone.frame >= 1.0 &&
                                                      frames_.count(static_cast<control::FrameId>(zone.frame)));
    if (zone.id == 0 || control::zones::fault(zone, known) >= 0) return control::Reason::InvalidParameter;
    control::OpZone& kept = opZones_[zone.id];
    const std::uint32_t revision = kept.revision + 1;
    kept = zone;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving zone's: from now)
    return control::Reason::None;
}

bool World::removeOpZone(control::OpZoneId id) { return opZones_.erase(id) > 0; }

std::vector<control::OpZoneId> World::opZones() const {
    std::vector<control::OpZoneId> out;
    out.reserve(opZones_.size());
    for (const auto& [id, z] : opZones_) out.push_back(id);
    return out;
}

std::optional<control::OpZone> World::opZone(control::OpZoneId id) const {
    const auto it = opZones_.find(id);
    if (it == opZones_.end()) return std::nullopt;
    return it->second;
}

const control::OpZone* World::Answers::opZone(control::OpZoneId id) const {
    const auto it = world_.opZones_.find(id);
    return it == world_.opZones_.end() ? nullptr : &it->second;
}

} // namespace fsim::session
