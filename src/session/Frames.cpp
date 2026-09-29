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
    return control::carried(control::vehiclePose(pool_->states()[e->slot]), t - simTime_);
}

std::optional<control::GeoPoint> World::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const {
    if (offset.rotation >= control::FrameRotation::Count || offset.offsets >= control::FrameOffsets::Count) return std::nullopt;
    const std::optional<control::FramePose> pose = framePose(id, timeS);
    if (!pose) return std::nullopt;
    return control::framePoint(*pose, offset);
}

bool World::Answers::frame(control::FrameId id, control::FrameSpec& spec, control::FramePose& now) const {
    if (id > control::kVehicleFrames && id - control::kVehicleFrames <= 0xFFFFFFFFu) { // a vehicle's own (4.42): where it is now
        const Entry* e = world_.entry(static_cast<std::uint32_t>(id - control::kVehicleFrames));
        if (!e) return false;
        spec = control::FrameSpec{}, spec.origin = control::FrameOrigin::Vehicle, spec.vehicle = static_cast<std::uint32_t>(id - control::kVehicleFrames);
        now = control::vehiclePose(world_.pool_->states()[e->slot]);
        return true;
    }
    const std::optional<control::FrameSpec> found = world_.frame(id);
    const std::optional<control::FramePose> pose = found ? world_.framePose(id) : std::nullopt;
    if (!pose) return false;
    spec = *found, now = *pose;
    return true;
}

} // namespace fsim::session
