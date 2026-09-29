// A route's planned states beside points in moving frames (docs/flight-autonomy.md, 4.34; ADR-29 FA-6g3b): each checked on
// its leg where the leg will be at its time, its ends where their frames carry them. Its own translation unit, last in the
// library: what is added before the runtime's code moves it (the A/B's lesson).
#include "control/CapabilityHost.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Frames.h"

namespace fsim::control {

route::Leg CapabilityHost::legAt(const route::Plan& p, std::uint32_t k, double timeS, const sim::VehicleState& state) const noexcept {
    const double now = sessionView_ ? sessionView_->simTimeS() : state.simTime;
    auto placed = [&](std::uint32_t i, double& lat, double& lon) {
        const Waypoint& w = p.points[i];
        lat = w.latitudeRad, lon = w.longitudeRad; // (where it is placed now)
        FrameSpec spec;
        FramePose pose;
        if (isHold(w.frame) || !sessionView_ || !sessionView_->frame(static_cast<FrameId>(w.frame), spec, pose) || spec.origin == FrameOrigin::Fixed) return;
        if (spec.origin == FrameOrigin::Vehicle) { // (where its velocity now carries it)
            spec.origin = FrameOrigin::Moving;
            spec.latitudeRad = pose.latitudeRad, spec.longitudeRad = pose.longitudeRad, spec.altitudeMslM = pose.altitudeMslM;
            spec.yawRad = pose.yawRad, spec.pitchRad = pose.pitchRad, spec.rollRad = pose.rollRad;
            spec.northMs = pose.northMs, spec.eastMs = pose.eastMs, spec.downMs = pose.downMs;
            spec.timeS = now;
        }
        const GeoPoint at = framePoint(framePose(spec, timeS), w.frameOffset());
        lat = at.latitudeRad, lon = geo::wrapPi(at.longitudeRad);
    };
    double latB, lonB, latA = state.latitudeRad, lonA = state.longitudeRad;
    placed(k, latB, lonB);
    if (k > p.start || (p.repeat && k != p.start)) placed(p.prev(k), latA, lonA);
    return route::makeLeg(latA, lonA, latB, lonB, p.rhumb);
}

} // namespace fsim::control
