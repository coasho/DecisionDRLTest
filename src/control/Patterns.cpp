// A pattern's setpoint and shape (docs/vehicle-interface.md, 4.6; docs/flight-autonomy.md, 4.23): merged, completed,
// checked and written as a pattern is commanded or updated, never stepped - in a file of its own.
#include "control/CapabilityHost.h"

#include "control/Adapter.h"
#include "control/ControlStack.h"
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
    auto either = [](double& a, double& b, double givenA, double givenB) {
        if (!isHold(givenA) || !isHold(givenB)) a = givenA, b = givenB;
    };
    either(dst.radiusM, dstShape.bankRad, src.radiusM, srcShape.bankRad);
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
                                            const sim::VehicleState& state, CommandResult& result, CheckLog& log) noexcept {
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
    merged.courseRad = geo::wrapPi(merged.courseRad), merged.longitudeRad = geo::wrapPi(merged.longitudeRad);
    optimise(merged.speed, merged.speedReference, merged.speedOptimization, merged.altitudeM, merged.altitudeReference, state);
    // what the merge left to be filled in again: a course from a heading, legs from their time, a radius from a bank
    const bool radiusFromBank = isHold(merged.radiusM) && !isHold(mergedShape.bankRad);
    if (isHold(merged.radiusM) || (mergedShape.twoCircles() ? isHold(mergedShape.radius2M) : isHold(merged.courseRad) || isHold(merged.legM)))
        completePattern(merged, mergedShape, state);
    if (const int field = route::circlesFault(merged, mergedShape); field >= 0) {
        result.index = static_cast<std::int16_t>(field);
        return about(rejected(Reason::InvalidParameter, activity), result);
    }
    if (slots_[s].range != RangePolicy::None) {
        limitPattern(merged, mergedShape, log, radiusFromBank);
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
    ++store->revision;
}

void ControlStack::command(const PatternCommand& pattern, const PatternShape& shape) {
    // its shape into the stack's own path store, as a World's host writes it
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    config_->path->pattern = shape;
    ++config_->path->revision;
    command(Command(pattern));
}

} // namespace fsim::control
