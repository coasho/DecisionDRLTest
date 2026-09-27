#pragma once

// What a command's checks find (docs/flight-autonomy.md, 4.8): every value
// held to what the aircraft can do, and every reason it cannot be flown as
// asked. Under RangePolicy::Clamp a value beyond a limit is held there and
// flown: an adjustment. Under Reject it is a finding, and checking goes on
// with it held, so every finding is named; the answer's reason and detail are
// the first finding's, as they were when checking stopped at it.

#include "fsim/Capability.h"

#include <cstdint>
#include <limits>

namespace fsim::control {

struct CheckLog {
    CommandResult& result;          ///< the answer: kClamped, and the first finding's detail (or the first clamp's)
    RangePolicy range;
    CommandDetails* details;        ///< every finding and adjustment; null: not kept
    Reason refused = Reason::None;  ///< the first finding's reason; None: the command may fly

    /// Something the command cannot be flown with, whatever the range policy
    /// (a curve section too tight for the aircraft); checking goes on.
    void find(Reason reason, std::int16_t index, Constraint constraint, float from = kNoSection, float to = kNoSection,
              std::uint64_t associated = 0) noexcept {
        if (refused == Reason::None) {
            refused = reason;
            result.index = index, result.constraint = constraint, result.from = from, result.to = to;
        }
        if (!details) return;
        if (details->findingCount < CommandDetails::kMax)
            details->findings[details->findingCount] = Finding{reason, index, constraint, from, to, associated, reasonDescription(reason)};
        if (details->findingCount < 255) ++details->findingCount;
    }

    /// `value` beyond a limit and held to `to`: flown so (Clamp: an
    /// adjustment, kClamped) or a finding with `reason` (Reject). `field` is
    /// a route point's field, else -1.
    void limit(double& value, double to, std::int16_t index, std::int16_t field, Constraint constraint, Reason reason) noexcept {
        if (range == RangePolicy::Reject) find(reason, index, constraint);
        else adjust(value, to, index, field, constraint);
        value = to;
    }

    /// Flown other than asked with no one value to hold (a fly-by turn flown
    /// smaller than its legs allow): as limit(), requested and adjusted NaN.
    void reshape(std::int16_t index, Constraint constraint, Reason reason) noexcept {
        double none = std::numeric_limits<double>::quiet_NaN();
        if (range == RangePolicy::Reject) find(reason, index, constraint);
        else adjust(none, none, index, -1, constraint);
    }

private:
    static constexpr float kNoSection = std::numeric_limits<float>::quiet_NaN();

    void adjust(double requested, double to, std::int16_t index, std::int16_t field, Constraint constraint) noexcept {
        if (!(result.flags & kClamped)) result.index = index, result.constraint = constraint; // (the first clamp's)
        result.flags |= kClamped;
        if (!details) return;
        if (details->adjustmentCount < CommandDetails::kMax)
            details->adjustments[details->adjustmentCount] = Adjustment{index, field, constraint, requested, to};
        if (details->adjustmentCount < 255) ++details->adjustmentCount;
    }
};

} // namespace fsim::control
