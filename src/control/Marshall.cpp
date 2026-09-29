// The altitude stacked marshall (docs/flight-autonomy.md, 4.46; A-GRA's ALTITUDE_STACKED_MARSHALL): a pattern flown by each aircraft
// of a stack at an altitude of its own. The world chooses the altitude among the other aircraft marshalling round the same point
// (session/Marshalls.cpp); here the host checks the stack's fields and prepares, updates and flies its pattern as a pattern's are.
// Its activity's command is its pattern, a PatternCommand at its slot, and its stack is kept beside it - in the path store while it
// flies, in its waiting entry while it waits - not in the command variant (4.46).
#include "control/CapabilityHost.h"

#include "control/Checks.h"
#include "control/Runtime.h"
#include "fsim/ControllerRegistry.h"
#include "fsim/GuidanceModes.h"

#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }

/// Accepted, as CapabilityHost.cpp's: an UPDATE's answer before its checks.
CommandResult accepted(ActivityId activity) noexcept {
    CommandResult r;
    r.status = CommandStatus::Accepted;
    r.activity = activity;
    return r;
}

/// `r` with what the check said the answer is about, as CapabilityHost.cpp's.
CommandResult about(CommandResult r, const CommandResult& detail) noexcept {
    r.index = detail.index;
    r.constraint = detail.constraint;
    r.from = detail.from;
    r.to = detail.to;
    return r;
}

/// A pattern's field as a marshall's names it: its own the same, its speed's and duration's moved up, its course and legs its
/// second circle's (a racetrack's and a figure-eight's by two circles), its shape's the same (from 13).
std::int16_t marshallField(std::int16_t patternField) noexcept {
    switch (patternField) {
    case 7:
    case 8: return 18;  // (course, legs: its second circle)
    case 9: return 7;   // (speed)
    case 10: return 8;  // (speed reference)
    case 11: return 9;  // (duration)
    case 12: return 7;  // (a speed optimisation: its speed)
    default: return patternField;
    }
}

/// The findings and adjustments from `found` and `adjusted` on - a pattern's checks' - named as the marshall's.
void nameAsMarshall(CommandDetails& details, std::uint8_t found, std::uint8_t adjusted) noexcept {
    for (std::uint8_t i = found; i < std::min<std::uint8_t>(details.findingCount, CommandDetails::kMax); ++i)
        details.findings[i].index = marshallField(details.findings[i].index);
    for (std::uint8_t i = adjusted; i < std::min<std::uint8_t>(details.adjustmentCount, CommandDetails::kMax); ++i)
        details.adjustments[i].index = marshallField(details.adjustments[i].index);
}

/// The stack's fields an UPDATE gives, merged into the kept ones.
void mergeStack(MarshallStack& stack, const MarshallCommand& next) noexcept {
    if (!isHold(next.altitudeMinM)) stack.altitudeMinM = next.altitudeMinM;
    if (!isHold(next.altitudeMaxM)) stack.altitudeMaxM = next.altitudeMaxM;
    if (!isHold(next.separationM)) stack.separationM = next.separationM;
}

} // namespace

PatternCommand patternOf(const MarshallCommand& m) noexcept {
    PatternCommand p;
    p.pattern = isHold(m.pattern) ? static_cast<double>(PatternKind::Orbit) : m.pattern;
    p.latitudeRad = m.latitudeRad, p.longitudeRad = m.longitudeRad, p.altitudeM = m.altitudeM, p.altitudeReference = m.altitudeReference;
    p.radiusM = m.radiusM, p.clockwise = m.clockwise, p.speed = m.speed, p.speedReference = m.speedReference, p.durationS = m.durationS;
    return p;
}

void takeBack(MarshallCommand& m, const PatternCommand& p) noexcept {
    m.pattern = p.pattern, m.latitudeRad = p.latitudeRad, m.longitudeRad = p.longitudeRad, m.altitudeM = p.altitudeM;
    m.altitudeReference = p.altitudeReference, m.radiusM = p.radiusM, m.clockwise = p.clockwise, m.speed = p.speed;
    m.speedReference = p.speedReference, m.durationS = p.durationS;
}

MarshallStack stackOf(const MarshallCommand& m) noexcept { return MarshallStack{m.altitudeMinM, m.altitudeMaxM, m.separationM}; }

MarshallCommand marshallOf(const PatternCommand& pattern, const MarshallStack& stack) noexcept {
    MarshallCommand m;
    takeBack(m, pattern);
    m.altitudeMinM = stack.altitudeMinM, m.altitudeMaxM = stack.altitudeMaxM;
    m.separationM = isHold(stack.separationM) ? kMarshallSeparationM : stack.separationM;
    return m;
}

void mergeMarshall(MarshallCommand& dst, const MarshallCommand& src) noexcept {
    const double* from[] = {&src.pattern,  &src.latitudeRad,    &src.longitudeRad, &src.altitudeM,    &src.altitudeReference,
                            &src.radiusM,  &src.clockwise,      &src.speed,        &src.speedReference, &src.durationS,
                            &src.altitudeMinM, &src.altitudeMaxM, &src.separationM};
    double* to[] = {&dst.pattern,  &dst.latitudeRad,    &dst.longitudeRad, &dst.altitudeM,    &dst.altitudeReference,
                    &dst.radiusM,  &dst.clockwise,      &dst.speed,        &dst.speedReference, &dst.durationS,
                    &dst.altitudeMinM, &dst.altitudeMaxM, &dst.separationM};
    for (std::size_t i = 0; i < std::size(from); ++i)
        if (!isHold(*from[i])) *to[i] = *from[i];
}

int marshallFault(const MarshallCommand& m, const PatternShape* shape) noexcept {
    // its pattern an orbit, a racetrack, a figure-eight (by two circles, A-GRA's) or a hover - never a hold, ATC's
    if (!code(m.pattern, static_cast<double>(PatternKind::Count)) || m.pattern == static_cast<double>(PatternKind::Hold)) return 0;
    const bool twoCircles = m.pattern == static_cast<double>(PatternKind::Racetrack) || m.pattern == static_cast<double>(PatternKind::FigureEight);
    if (twoCircles && (!shape || !shape->twoCircles())) return 18; // (its second circle)
    if (!code(m.altitudeReference, static_cast<double>(AltitudeReference::Count))) return 4;
    // its stack: the least given, finite; the most not below it; the separation above 0; its slot within them
    if (isHold(m.altitudeMinM) || !std::isfinite(m.altitudeMinM)) return 10;
    if (!finiteOr(m.altitudeMaxM) || (!isHold(m.altitudeMaxM) && m.altitudeMaxM < m.altitudeMinM)) return 11;
    if (!finiteOr(m.separationM) || (!isHold(m.separationM) && !(m.separationM > 0.0))) return 12;
    if (!finiteOr(m.altitudeM) || (!isHold(m.altitudeM) && (m.altitudeM < m.altitudeMinM || (!isHold(m.altitudeMaxM) && m.altitudeM > m.altitudeMaxM))))
        return 3;
    return -1;
}

// --- The host ---------------------------------------------------------------------------------------

Reason CapabilityHost::prepareMarshall(Command& setpoint, const MarshallStack& stack, const sim::VehicleState& state, CheckLog& log,
                                       const PatternShape* shape) {
    CommandResult& detail = log.result;
    auto* pattern = std::get_if<PatternCommand>(&setpoint);
    if (!pattern) return Reason::WrongCommandType; // (a marshall flies its pattern)
    if (const int field = marshallFault(marshallOf(*pattern, stack), shape); field >= 0) {
        detail.index = static_cast<std::int16_t>(field);
        return Reason::InvalidParameter;
    }
    // its slot left out: its stack's least (the world chooses it among the others' at the NEW: 4.46)
    if (isHold(pattern->altitudeM)) pattern->altitudeM = stack.altitudeMinM;
    // its pattern prepared as a pattern's NEW - its fields named back as the marshall's
    const int index = catalog_->indexOf(SetpointKind::Pattern);
    if (index < 0) return Reason::UnknownCapability;
    const std::uint8_t found = details_.findingCount, adjusted = details_.adjustmentCount;
    const Reason why = prepare(static_cast<std::size_t>(index), setpoint, {}, {}, state, log, shape, nullptr, nullptr);
    detail.index = marshallField(detail.index);
    nameAsMarshall(details_, found, adjusted);
    return why;
}

CommandResult CapabilityHost::refused(Reason why, std::int16_t index, ActivityId activity) noexcept {
    details_.clear();
    CommandResult r = rejected(why, activity);
    r.index = index;
    return r;
}

CommandResult CapabilityHost::submit(const MarshallCommand& marshall, const PatternShape& shape, const CommandOptions& options, const sim::VehicleState& state,
                                     double now) {
    if (catalog_->indexOf(SetpointKind::Marshall) < 0) { // (what the vehicle cannot fly: why)
        details_.clear();
        return rejected(missing("fsim.guidance.marshall"));
    }
    const MarshallStack stack = stackOf(marshall);
    const RouteExtras extras{{}, {}, {}, {}, {}, nullptr, &stack};
    const CommandResult r = submitWith(Command(patternOf(marshall)), {}, {}, options, state, now, true, &shape, nullptr, &extras);
    if (r.accepted() && !(r.flags & kDeferred) && config_->path) config_->path->marshall = stack; // (it flies: its stack beside its pattern)
    return r;
}

CommandResult CapabilityHost::update(ActivityId activity, const MarshallCommand& next, const PatternShape* shape, const sim::VehicleState& state,
                                     Caller caller) noexcept {
    details_.clear();
    const int live = liveSlot(activity);
    if (live < 0) {
        if (Waiting* w = waitingEntry(activity)) return updateWaitingMarshall(*w, next, shape, state, caller);
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(live);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    const ActivityRecord& record = records_[s];
    const CapabilityDescriptor& d = catalog_->descriptor(record.capability);
    if (!(d.interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    const auto* flown = std::get_if<PatternCommand>(&config_->slots[s].command);
    if (d.setpoint != SetpointKind::Marshall || !flown || !config_->path) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    CheckLog log{result, slots_[s].range, &details_};
    // its stack's fields merged and checked with its pattern's as the UPDATE leaves them
    MarshallCommand merged = marshallOf(*flown, config_->path->marshall);
    mergeMarshall(merged, next);
    PatternShape given = config_->path->pattern;
    if (shape) {
        PatternCommand probe = patternOf(merged);
        mergePattern(probe, given, patternOf(next), *shape);
    }
    if (const int field = marshallFault(merged, &given); field >= 0) {
        result.index = static_cast<std::int16_t>(field);
        return about(rejected(Reason::InvalidParameter, activity), result);
    }
    // its pattern updated as a pattern's, its fields named back as the marshall's
    PatternCommand pattern = patternOf(next);
    if (isHold(next.pattern)) pattern.pattern = kHold; // (its kind kept)
    const std::uint8_t found = details_.findingCount, adjusted = details_.adjustmentCount;
    CommandResult r = updatePattern(s, activity, pattern, shape, state, result, log, true);
    r.index = marshallField(r.index);
    nameAsMarshall(details_, found, adjusted);
    if (r.accepted()) mergeStack(config_->path->marshall, next);
    return r;
}

CommandResult CapabilityHost::updateWaitingMarshall(Waiting& w, const MarshallCommand& next, const PatternShape* shape, const sim::VehicleState& state,
                                                    Caller caller) noexcept {
    const ActivityRecord& record = w.record;
    const ActivityId activity = record.id;
    if (const Reason why = addresses(record, caller); why != Reason::None) return rejected(why, activity, activity);
    if (w.support) return rejected(Reason::WrongCommandType, activity);
    const CapabilityDescriptor& d = catalog_->descriptor(record.capability);
    if (!(d.interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    const auto* kept = std::get_if<PatternCommand>(&w.command);
    if (d.setpoint != SetpointKind::Marshall || !kept) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    // what it will fly: its pattern and its stack merged, then checked as its NEW was, from where the aircraft is now
    PatternCommand pattern = *kept;
    PatternShape nextShape = w.shape;
    PatternCommand given = patternOf(next);
    if (isHold(next.pattern)) given.pattern = kHold; // (its kind kept)
    mergePattern(pattern, nextShape, given, shape ? *shape : PatternShape{});
    MarshallStack stack = w.marshall;
    mergeStack(stack, next);
    CheckLog log{result, w.options.range, &details_};
    Command probe = pattern;
    const RouteExtras extras{{}, {}, {}, {}, {}, nullptr, &stack};
    if (const Reason why = prepare(record.capability, probe, {}, {}, state, log, &nextShape, nullptr, &extras); why != Reason::None)
        return about(rejected(why, activity), result);
    if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
    // kept for its start, as given (it is prepared afresh as it starts)
    w.command = pattern, w.shape = nextShape, w.marshall = stack;
    return result;
}

bool CapabilityHost::marshall(ActivityId activity, MarshallCommand& out) const noexcept {
    if (const int live = liveSlot(activity); live >= 0) {
        const auto s = static_cast<std::size_t>(live);
        if (!isCascade(s) || !config_->path || catalog_->descriptor(records_[s].capability).setpoint != SetpointKind::Marshall) return false;
        const auto* flown = std::get_if<PatternCommand>(&config_->slots[s].command);
        if (!flown) return false;
        out = marshallOf(*flown, config_->path->marshall);
        return true;
    }
    const Waiting* w = waitingEntry(activity);
    if (!w || w->support || catalog_->descriptor(w->record.capability).setpoint != SetpointKind::Marshall) return false;
    const auto* kept = std::get_if<PatternCommand>(&w->command);
    if (!kept) return false;
    out = marshallOf(*kept, w->marshall);
    return true;
}

// --- The behaviour ----------------------------------------------------------------------------------

void registerMarshall(ControllerRegistry& r, const std::vector<ParameterInfo>& shape) {
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi, Constraint below = Constraint::None,
                Constraint above = Constraint::None) { return ParameterInfo{name, unit, lo, hi, def, true, below, above}; };
    // its setpoint's fields, in order (the C ABI's too), then its pattern's shape's, as a pattern's (from 13)
    BehaviorTraits marshall;
    marshall.persistence = Persistence::Persistent; // (as a pattern: with a duration or laps it completes, and flies on)
    marshall.parameters = {p("pattern", "", now, 0.0, static_cast<double>(PatternKind::Count) - 1.0),
                           p("latitude_rad", "rad", now, -0.5 * kPi, 0.5 * kPi),
                           p("longitude_rad", "rad", now, -inf, inf),
                           p("altitude_m", "m", now, -inf, inf, Constraint::MinAltitude, Constraint::MaxAltitude),
                           p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                           p("radius_m", "m", now, 0.0, inf, Constraint::MaxOrientation, Constraint::None),
                           p("clockwise", "", now, 0.0, 1.0),
                           p("speed", "m/s or Mach", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                           p("speed_reference", "", now, 0.0, static_cast<double>(SpeedReference::Count) - 1.0),
                           p("duration_s", "s", now, 0.0, inf),
                           p("altitude_min_m", "m", now, -inf, inf),
                           p("altitude_max_m", "m", now, -inf, inf),
                           p("separation_m", "m", kMarshallSeparationM, 0.0, inf)};
    marshall.parameters.insert(marshall.parameters.end(), shape.begin(), shape.end());
    marshall.uses = {"fsim.flight.velocity"};
    marshall.mode = FlightMode::AltitudeStackedMarshall;
    marshall.setpoint = SetpointKind::Marshall;
    // flown as its pattern: the slot's command is its pattern at its slot (its stack beside it, the host's)
    r.addBehavior("marshall", [] { return std::make_unique<PatternBehavior>(); }, std::move(marshall));
}

} // namespace fsim::control
