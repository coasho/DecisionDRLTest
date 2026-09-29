// Operational geometry (docs/flight-autonomy.md, 4.42 to 4.45; A-GRA's OpPoint, OpZone, OpLine and OpVolume): the world's store of it,
// by id, for the commands that name it (a must fly). Set and asked for between steps, never stepped.
#include "session/World.h"

#include "control/Lines.h"
#include "control/Volumes.h"
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

control::Reason World::setOpLine(const control::OpLine& line) {
    const bool known = control::isHold(line.frame) || (line.frame == std::floor(line.frame) && line.frame >= 1.0 &&
                                                      frames_.count(static_cast<control::FrameId>(line.frame)));
    if (line.id == 0 || control::lines::fault(line, known) >= 0) return control::Reason::InvalidParameter;
    control::OpLine& kept = opLines_[line.id];
    const std::uint32_t revision = kept.revision + 1;
    kept = line;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving line's: from now)
    return control::Reason::None;
}

bool World::removeOpLine(control::OpLineId id) { return opLines_.erase(id) > 0; }

std::vector<control::OpLineId> World::opLines() const {
    std::vector<control::OpLineId> out;
    out.reserve(opLines_.size());
    for (const auto& [id, l] : opLines_) out.push_back(id);
    return out;
}

std::optional<control::OpLine> World::opLine(control::OpLineId id) const {
    const auto it = opLines_.find(id);
    if (it == opLines_.end()) return std::nullopt;
    return it->second;
}

const control::OpLine* World::Answers::opLine(control::OpLineId id) const {
    const auto it = world_.opLines_.find(id);
    return it == world_.opLines_.end() ? nullptr : &it->second;
}

control::Reason World::setOpVolume(const control::OpVolume& volume) {
    const bool known = control::isHold(volume.frame) || (volume.frame == std::floor(volume.frame) && volume.frame >= 1.0 &&
                                                        frames_.count(static_cast<control::FrameId>(volume.frame)));
    if (volume.id == 0 || control::volumes::fault(volume, known) >= 0) return control::Reason::InvalidParameter;
    control::OpVolume& kept = opVolumes_[volume.id];
    const std::uint32_t revision = kept.revision + 1;
    kept = volume;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving volume's: from now)
    return control::Reason::None;
}

bool World::removeOpVolume(control::OpVolumeId id) { return opVolumes_.erase(id) > 0; }

std::vector<control::OpVolumeId> World::opVolumes() const {
    std::vector<control::OpVolumeId> out;
    out.reserve(opVolumes_.size());
    for (const auto& [id, v] : opVolumes_) out.push_back(id);
    return out;
}

std::optional<control::OpVolume> World::opVolume(control::OpVolumeId id) const {
    const auto it = opVolumes_.find(id);
    if (it == opVolumes_.end()) return std::nullopt;
    return it->second;
}

const control::OpVolume* World::Answers::opVolume(control::OpVolumeId id) const {
    const auto it = world_.opVolumes_.find(id);
    return it == world_.opVolumes_.end() ? nullptr : &it->second;
}

} // namespace fsim::session
