#pragma once

// A flight mode's performance profile (docs/flight-autonomy.md, 4.15; ADR-29
// FA-3c; A-GRA's MA_FlightControlModesPerformanceProfileType, VI 1.2.6.7):
// the guard rails a mission autonomy shapes its commands within - the
// airspeeds, altitudes, accelerations, climbs, descents, attitudes, rates and
// burn the aircraft flies to - worked out at its condition now (its
// altitude, weight and airspeed, its flaps and gear) from its performance
// tables (4.13), its envelope and the loops that fly it.

#include "fsim/Capability.h"
#include "fsim/Control.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace fsim::control {

/// A value at an airspeed, an altitude and a weight: a point of the profile
/// (A-GRA's MA_AirspeedLimitType, MA_SpeedType, MA_FuelBurnRateType).
/// Airspeeds are true (A-GRA's TRUE_AIRSPEED reference), altitudes above sea
/// level; NaN where the value does not depend on one (an airspeed limit's
/// own airspeed).
struct ProfilePoint {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double value = kNone; ///< m/s (an airspeed, a climb or descent rate), or a burn: kg/s of fuel, W of a battery
    double tasMs = kNone, altitudeMslM = kNone, weightKg = kNone;
};

/// Acceleration limits in body axes - x forward, y right, z down - as the
/// specific force the aircraft can make (lift, thrust and drag over its
/// mass, not gravity: 1 g of lift is -9.81 m/s2 in z), at a Mach number and
/// an airspeed, altitude and weight (A-GRA's MA_AccelerationLimitsType);
/// NaN where it has none.
struct ProfileAcceleration {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double xMs2 = kNone, yMs2 = kNone, zMs2 = kNone;
    double mach = kNone, tasMs = kNone, altitudeMslM = kNone, weightKg = kNone;
};

/// Specific excess power at full power (A-GRA's MA_SpecificExcessPowerType):
/// the climb it would make holding the speed, and the acceleration it would
/// make holding the height.
struct ProfileExcessPower {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double climbMs = kNone, accelerationMs2 = kNone;
    double tasMs = kNone, altitudeMslM = kNone, weightKg = kNone;
};

/// Attitude limits (A-GRA's MA_OrientationLimitType, yaw, pitch and roll):
/// the most nose-up pitch and bank; `pitchMinRad` the most nose-down, which
/// A-GRA's single orientation leaves out. NaN: no limit (the yaw).
struct ProfileOrientation {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double yawRad = kNone, pitchRad = kNone, pitchMinRad = kNone, rollRad = kNone;
    double tasMs = kNone, altitudeMslM = kNone, weightKg = kNone;
};

/// Attitude rate limits at an airspeed (A-GRA's MA_OrientationRateLimitsType), body axes; NaN: none known.
struct ProfileRates {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    double rollRadS = kNone, pitchRadS = kNone, yawRadS = kNone;
    double tasMs = kNone;
};

/// A flight mode's performance profile at the aircraft's condition now.
/// What the performance tables give - the airspeeds against altitude, the
/// excess power, the idle descents and decelerations, the burn against
/// speed and altitude, the ceiling - is its clean aircraft's (the tables are
/// flown clean): with its flaps or gear out, those are left out (`clean`
/// false) and the envelope's placards bound its airspeeds.
struct PerformanceProfile {
    static constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    FlightMode mode = FlightMode::None;
    // the condition it was worked out at
    double timeS = kNone, altitudeMslM = kNone, weightKg = kNone, tasMs = kNone;
    bool clean = true;                  ///< flaps and (retractable) gear up
    bool flapsOut = false, gearDown = false;
    Energy energy = Energy::Unknown;    ///< what `burn` is: fuel (kg/s) or a battery's power (W)
    // airspeeds (true), against altitude at the weight now: each value an airspeed, its altitude and weight
    std::vector<ProfilePoint> minAirspeed, maxAirspeed, bestEnduranceAirspeed, bestRangeAirspeed;
    double minAltitudeMslM = kNone;     ///< none: the platform keeps no floor but the ground
    double maxAltitudeMslM = kNone;     ///< the service ceiling at the weight now (the tables' clean one)
    // against airspeed and altitude, at the weight now
    std::vector<ProfileAcceleration> minAcceleration, maxAcceleration;
    std::vector<ProfileExcessPower> excessPower;
    std::vector<ProfilePoint> maxDescentRate;              ///< m/s, at the airspeed (idle; no faster than guidance asks)
    std::vector<ProfileAcceleration> maxDeceleration;      ///< xMs2 negative: idle, level
    std::vector<ProfilePoint> burn;                        ///< fuel flow or a battery's power, level
    // at the condition now
    std::vector<ProfileOrientation> maxOrientation;
    std::vector<ProfileRates> maxOrientationRate;
    double maxTurnRateRadS = kNone;     ///< the fastest turn guidance flies at the airspeed now
    double maxClimbRateMs = kNone;      ///< the fastest climb guidance asks
};

} // namespace fsim::control
