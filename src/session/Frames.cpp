// Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame): the world's store of them, and a frame as it
// is at a time. Set and asked for between steps, never stepped.
#include "session/World.h"

#include <cmath>

namespace fsim::session {

control::FrameId World::createFrame(const control::FrameSpec& spec) {
    const bool finite = std::isfinite(spec.latitudeRad) && std::abs(spec.latitudeRad) <= 0.5 * 3.14159265358979323846 &&
                        std::isfinite(spec.longitudeRad) && std::isfinite(spec.altitudeMslM) && std::isfinite(spec.yawRad) &&
                        std::isfinite(spec.pitchRad) && std::isfinite(spec.rollRad) && std::isfinite(spec.northMs) && std::isfinite(spec.eastMs) &&
                        std::isfinite(spec.downMs) && std::isfinite(spec.timeS);
    if (!finite || spec.origin >= control::FrameOrigin::Count) return 0;
    if (spec.origin == control::FrameOrigin::Vehicle && !entry(spec.vehicle)) return 0;
    frames_[++lastFrame_] = spec;
    return lastFrame_;
}

bool World::removeFrame(control::FrameId id) { return frames_.erase(id) > 0; }

std::optional<control::FrameSpec> World::frame(control::FrameId id) const {
    const auto it = frames_.find(id);
    if (it == frames_.end()) return std::nullopt;
    return it->second;
}

std::optional<control::FramePose> World::framePose(control::FrameId id, double timeS) const {
    const auto it = frames_.find(id);
    if (it == frames_.end()) return std::nullopt;
    const double t = std::isnan(timeS) ? simTime_ : timeS;
    const control::FrameSpec& spec = it->second;
    if (spec.origin != control::FrameOrigin::Vehicle) return control::framePose(spec, t);
    // a vehicle's: where it is now, as it is turned and moving, carried on to the time
    const Entry* e = entry(spec.vehicle);
    if (!e) return std::nullopt;
    const sim::VehicleState& s = pool_->states()[e->slot];
    control::FramePose pose;
    pose.latitudeRad = s.latitudeRad, pose.longitudeRad = s.longitudeRad, pose.altitudeMslM = s.altitudeMslM;
    pose.rollRad = s.eulerRad[0], pose.pitchRad = s.eulerRad[1], pose.yawRad = s.eulerRad[2];
    pose.northMs = s.velocityNedMs[0], pose.eastMs = s.velocityNedMs[1], pose.downMs = s.velocityNedMs[2];
    return control::carried(pose, t - simTime_);
}

std::optional<control::GeoPoint> World::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const {
    if (offset.rotation >= control::FrameRotation::Count || offset.offsets >= control::FrameOffsets::Count) return std::nullopt;
    const std::optional<control::FramePose> pose = framePose(id, timeS);
    if (!pose) return std::nullopt;
    return control::framePoint(*pose, offset);
}

} // namespace fsim::session
