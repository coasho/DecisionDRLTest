// A pattern's setpoint and shape (docs/vehicle-interface.md, 4.6; docs/flight-autonomy.md, 4.23): merged, completed,
// checked and written as a pattern is commanded or updated, never stepped - in a file of its own.
#include "control/CapabilityHost.h"

#include "control/Adapter.h"
#include "control/ControlStack.h"
#include "control/Features.h"
#include "control/Route.h"
#include "control/Runtime.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"

#include <memory>

namespace fsim::control {

namespace {

/// `r` with what the check said the answer is about (docs/vehicle-interface.md, 5.1), as CapabilityHost.cpp's.
CommandResult about(CommandResult r, const CommandResult& detail) noexcept {
    r.index = detail.index;
    r.constraint = detail.constraint;
    r.from = detail.from;
    r.to = detail.to;
    return r;
}

} // namespace

void mergePattern(PatternCommand& dst, PatternShape& dstShape, const PatternCommand& src, const PatternShape& srcShape) noexcept {
    mergePattern(dst, src);
    PatternShape given = srcShape;
    double* to[PatternShape::kFields];
    double* from[PatternShape::kFields];
    dstShape.fields(to), given.fields(from);
    for (std::size_t i = 0; i < PatternShape::kFields; ++i)
        if (!isHold(*from[i])) *to[i] = *from[i];
    if (srcShape.twoCircles()) dst.courseRad = dst.legM = dstShape.headingRad = dstShape.legS = kHold; // (the circles give them)
    // (a point replaces a frame's, and a frame a point: filled in again from it)
    if (!isHold(src.latitudeRad) || !isHold(src.longitudeRad))
        dstShape.frame = dstShape.frameRotation = dstShape.frameOffsets = dstShape.frameXM = dstShape.frameYM = dstShape.frameZM = kHold;
    if (!isHold(srcShape.frame)) dst.latitudeRad = dst.longitudeRad = kHold;
    auto either = [](double& a, double& b, double givenA, double givenB) {
        if (!isHold(givenA) || !isHold(givenB)) a = givenA, b = givenB;
    };
    // (the radius: its own, a bank, a turn rate or a type - any given replaces them all)
    if (!isHold(src.radiusM) || !isHold(srcShape.bankRad) || !isHold(srcShape.turnRateRadS) || !isHold(srcShape.turnType))
        dst.radiusM = src.radiusM, dstShape.bankRad = srcShape.bankRad, dstShape.turnRateRadS = srcShape.turnRateRadS, dstShape.turnType = srcShape.turnType;
    either(dst.courseRad, dstShape.headingRad, src.courseRad, srcShape.headingRad);
    either(dst.legM, dstShape.legS, src.legM, srcShape.legS);
}

void CapabilityHost::completePattern(PatternCommand& c, PatternShape& shape, const sim::VehicleState& state) const noexcept {
    WindEstimate wind;
    wind.update(state, 0.0);
    const bool magnetic = shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth);
    route::completePattern(c, shape, state, performance_, (adapter_->features() & kFeatureHover) != 0, wind.northMs, wind.eastMs, &config_->altimeter,
                           magnetic ? yearNow() : 2025.0);
}

CommandResult CapabilityHost::submit(const PatternCommand& pattern, const PatternShape& shape, const CommandOptions& options,
                                     const sim::VehicleState& state, double now) {
    return submitWith(Command(pattern), {}, {}, options, state, now, true, &shape);
}

CommandResult CapabilityHost::update(ActivityId activity, const PatternCommand& pattern, const PatternShape& shape, const sim::VehicleState& state,
                                     Caller caller) noexcept {
    return update(activity, Command(pattern), state, caller, &shape);
}

CommandResult CapabilityHost::updatePattern(std::size_t s, ActivityId activity, const PatternCommand& next, const PatternShape* shape,
                                            const sim::VehicleState& state, CommandResult& result, CheckLog& log, bool marshall) noexcept {
    if (!marshall && catalog_->descriptor(records_[s].capability).setpoint == SetpointKind::Marshall) // (through its own UPDATE: 4.46)
        return rejected(Reason::WrongCommandType, activity);
    // a partial pattern (docs/vehicle-interface.md, 4.6): the fields given replace the commanded ones - and its
    // shape's, the path store's (docs/flight-autonomy.md, 4.23)
    const PatternShape given = shape ? *shape : PatternShape{};
    if (const Reason why = checkPattern(next, true, result); why != Reason::None) return about(rejected(why, activity), result);
    if (const Reason why = checkShape(next, given, true, result); why != Reason::None) return about(rejected(why, activity), result);
    if (const Reason why = optimisable(next.speedOptimization, 12, result); why != Reason::None) return about(rejected(why, activity), result);
    SetpointSlot& slot = config_->slots[s];
    PatternCommand merged = std::get<PatternCommand>(slot.command);
    PatternShape mergedShape = config_->path ? config_->path->pattern : PatternShape{};
    const bool shaped = shape || !mergedShape.empty();
    if (shaped) mergePattern(merged, mergedShape, next, given);
    else mergePattern(merged, next);
    if (const Reason why = placePattern(merged, mergedShape, result, &next, &given); why != Reason::None) return about(rejected(why, activity), result);
    merged.courseRad = geo::wrapPi(merged.courseRad), merged.longitudeRad = geo::wrapPi(merged.longitudeRad);
    optimise(merged.speed, merged.speedReference, merged.speedOptimization, merged.altitudeM, merged.altitudeReference, state);
    // what the merge left to be filled in again: a course from a heading, legs from their time, a radius from a bank, rate or type
    const std::int16_t radiusFrom = radiusField(merged, mergedShape);
    if (isHold(merged.radiusM) || (mergedShape.twoCircles() ? isHold(mergedShape.radius2M) : isHold(merged.courseRad) || isHold(merged.legM)))
        completePattern(merged, mergedShape, state);
    if (const int field = route::shapeFault(merged, mergedShape); field >= 0) {
        result.index = static_cast<std::int16_t>(field);
        return about(rejected(Reason::InvalidParameter, activity), result);
    }
    if (slots_[s].range != RangePolicy::None) {
        limitPattern(merged, mergedShape, state, log, radiusFrom);
        patternShape_ = mergedShape; // (the shape the terrain check walks)
        checkTerrain(Command(merged), state, log);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    std::get<PatternCommand>(slot.command) = merged;
    if (shaped) patternShape_ = mergedShape, writeShape();
    ++slot.revision;
    return result;
}

void CapabilityHost::writeShape() {
    // the pattern that flies: its shape, where it has one or one flew before (the store is made at a pattern's NEW)
    PathStore* store = config_->path.get();
    if (!store || (patternShape_.empty() && store->pattern.empty())) return;
    store->pattern = patternShape_;
    store->patternFrame = patternFrame_;
    ++store->revision;
}

Reason CapabilityHost::hoverSupport() const noexcept {
    const SupportInfo* row = support_ ? support_->find("fsim.guidance.pattern/hover") : nullptr;
    const Support support = row ? row->support : (adapter_->features() & kFeatureHover) ? Support::Supported : Support::NotSupported;
    return support == Support::NotSupported ? Reason::NotSupported : support == Support::NotImplemented ? Reason::NotImplemented : Reason::None;
}

Reason CapabilityHost::placePattern(PatternCommand& c, PatternShape& shape, CommandResult& detail, const PatternCommand* given,
                                    const PatternShape* givenShape) noexcept {
    if (c.pattern == static_cast<double>(PatternKind::Hover)) { // (a hover: where the aircraft hovers; its point, altitude, speed and duration)
        if (const Reason why = hoverSupport(); why != Reason::None) {
            detail.index = 0;
            return why;
        }
        if (const int field = route::hoverFault(given ? *given : c, givenShape ? *givenShape : shape); field >= 0) {
            detail.index = static_cast<std::int16_t>(field);
            return Reason::InvalidParameter;
        }
        if (given) { // (an UPDATE: the circuit the pattern it was had, gone - its frame kept)
            c.radiusM = c.clockwise = c.courseRad = c.legM = kHold;
            PatternShape kept;
            kept.frame = shape.frame, kept.frameRotation = shape.frameRotation, kept.frameOffsets = shape.frameOffsets;
            kept.frameXM = shape.frameXM, kept.frameYM = shape.frameYM, kept.frameZM = shape.frameZM;
            shape = kept;
        }
    }
    if (isHold(shape.frame)) return Reason::None;
    FramePose now;
    if (!sessionView_ || !sessionView_->frame(static_cast<FrameId>(shape.frame), patternFrame_, now)) {
        detail.index = 29;
        return Reason::InvalidParameter;
    }
    const GeoPoint at = framePoint(now, shape.frameOffset());
    c.latitudeRad = at.latitudeRad, c.longitudeRad = geo::wrapPi(at.longitudeRad);
    if (!isHold(shape.frameZM)) c.altitudeM = at.altitudeMslM, c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    return Reason::None;
}

Reason CapabilityHost::checkLoiter(const RouteLoiter& l) const noexcept {
    CommandResult field; // (the caller names the point)
    if (checkPattern(l.pattern, false, field) != Reason::None || checkShape(l.pattern, l.shape, false, field) != Reason::None)
        return Reason::InvalidWaypoint;
    if (l.pattern.pattern == static_cast<double>(PatternKind::Hover)) { // (where the aircraft hovers, given nothing that shapes a circuit)
        if (const Reason why = hoverSupport(); why != Reason::None) return why;
        if (route::hoverFault(l.pattern, l.shape) >= 0) return Reason::InvalidWaypoint;
    }
    return optimisable(l.pattern.speedOptimization, 0, field); // (no performance tables to fly one from: not implemented)
}

void CapabilityHost::completeLoiters(route::Plan& p, const sim::VehicleState& state) const noexcept {
    bool magnetic = false;
    for (std::uint32_t k = 0; k < p.loiterCount; ++k) { // an optimisation's speed first, at its point's altitude, as a pattern's
        RouteLoiter& l = p.loiters[k];
        if (l.point >= p.count) continue; // (on a point it does not fly: kept as given - 4.36)
        const Waypoint& w = p.points[l.point];
        optimise(l.pattern.speed, l.pattern.speedReference, l.pattern.speedOptimization, w.altitudeM, w.altitudeReference, state);
        magnetic = magnetic || l.shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth);
    }
    const WindEstimate wind = checkWind(state); // (a validation's, while one runs: 4.41)
    route::completeLoiters(p, state, performance_, (adapter_->features() & kFeatureHover) != 0, wind.northMs, wind.eastMs, &config_->altimeter,
                           magnetic ? yearNow() : 2025.0);
}

void ControlStack::command(const PatternCommand& pattern, const PatternShape& shape) {
    // its shape into the stack's own path store, as a World's host writes it
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    config_->path->pattern = shape;
    ++config_->path->revision;
    command(Command(pattern));
}

} // namespace fsim::control
