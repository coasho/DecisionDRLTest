#include "control/Features.h"

#include "control/Catalog.h"

#include <algorithm>
#include <cstring>

namespace fsim::control {

namespace {

constexpr Support S = Support::Supported, P = Support::Partial, N = Support::NotImplemented;

constexpr std::uint16_t R(Rule rule) noexcept { return ruleBit(rule); }
constexpr std::uint16_t R1 = R(Rule::Hover), R2 = R(Rule::GroundTaxi), R3 = R(Rule::CatapultArrested), R4 = R(Rule::DeckOperations),
                        R5 = R(Rule::ArresterHook), R6 = R(Rule::RetractableGear), R7 = R(Rule::Flaps), R8 = R(Rule::DragDevices),
                        R9 = R(Rule::ReleasableStores), R10 = R(Rule::Aerobatic), R11 = R(Rule::WheelBrakes), R12 = R(Rule::PitchTrim),
                        R13 = R(Rule::EngineThrottles);

/// A public feature (docs/flight-autonomy.md, 4.2): its identifier, the
/// capability that carries it (itself, for a capability), what the platform
/// has built of it, the stage that builds the rest, the rules that govern it
/// (it applies where any of them admits the aircraft; 0 everywhere) and,
/// when partial, what is missing. For a capability the stage is also where
/// an aircraft whose model lacks it (the flaps of a type still to be decided)
/// gets it. Once published an identifier keeps its meaning; new features
/// take new ones. The comments name the items of the flight autonomy record
/// each one answers for (its Appendix A); they are not identifiers.
struct FeatureDef {
    const char* id;
    const char* capability;
    Support built;
    std::uint8_t stage;
    std::uint16_t rules;
    const char* missing;
    /// Flown from the performance tables (a speed optimisation): not implemented
    /// on an aircraft hangar has flown none for - a stock one (FA-3's: stage 3).
    bool tables = false;
};

#define CAP(id, built, stage, rules, missing) {id, id, built, stage, rules, missing}

const FeatureDef kFeatures[] = {
    // --- the flight levels and support effectors (ADR-26) -----------------------------------
    CAP("fsim.flight.actuator", S, 0, 0, ""),
    CAP("fsim.flight.attitude", S, 0, 0, ""),
    CAP("fsim.flight.acceleration", S, 0, 0, ""),
    CAP("fsim.flight.velocity", S, 0, 0, ""),
    CAP("fsim.flight.position", S, 0, 0, ""),
    CAP("fsim.flight.engines", S, 0, R13, ""),
    CAP("fsim.support.gear", S, 0, R6, ""),         // STS-15
    CAP("fsim.support.flaps", S, 10, R7, ""),       // a type whose flaps are still to be decided: its model gets them with FA-10
    CAP("fsim.support.wheel_brakes", S, 0, R11, ""),
    CAP("fsim.support.speedbrake", S, 10, R8, ""),  // STS-16, SUB-04: the models get their drag devices in FA-10
    CAP("fsim.support.pitch_trim", S, 0, R12, ""),
    CAP("fsim.envelope.protection", S, 3, 0, ""),   // CTG-04 (an aircraft without an envelope: FA-3's performance)

    // --- the platform's own behaviours (section 6) ---------------------------------------------
    CAP("fsim.guidance.hold", S, 0, 0, ""),         // PLT-01, superseded by hsa
    CAP("fsim.guidance.waypoints", S, 0, 0, ""),    // PLT-02, superseded by route
    CAP("fsim.guidance.loiter", S, 0, 0, ""),       // PLT-03, superseded by pattern
    CAP("fsim.guidance.pursuit", S, 0, 0, ""),      // PLT-04
    CAP("fsim.guidance.evade", S, 0, 0, ""),        // PLT-05
    CAP("fsim.guidance.formation", S, 0, 0, ""),    // PLT-06
    CAP("fsim.guidance.aerobatics", S, 0, R10, ""), // PLT-07
    CAP("fsim.guidance.hover", S, 0, R1, ""),       // PLT-08, superseded by the hover loiter

    // --- HSA/CSA ---------------------------------------------------------------------------------
    CAP("fsim.guidance.hsa", S, 0, 0, ""), // HSA-08..10: whole since FA-4 (its barometric and magnetic references)
    {"fsim.guidance.hsa/direction/heading", "fsim.guidance.hsa", S, 0, 0, ""},              // HSA-01
    {"fsim.guidance.hsa/direction/course", "fsim.guidance.hsa", S, 0, 0, ""},               // HSA-02
    {"fsim.guidance.hsa/direction/magnetic_north", "fsim.guidance.hsa", S, 0, 0, ""},       // HSA-03 (FA-4d)
    {"fsim.guidance.hsa/speed/true_airspeed", "fsim.guidance.hsa", S, 0, 0, ""},            // HSA-04
    {"fsim.guidance.hsa/speed/calibrated_airspeed", "fsim.guidance.hsa", S, 0, 0, ""},
    {"fsim.guidance.hsa/speed/ground_speed", "fsim.guidance.hsa", S, 0, 0, ""},
    {"fsim.guidance.hsa/speed/mach", "fsim.guidance.hsa", S, 0, 0, ""},
    {"fsim.guidance.hsa/speed/long_range_cruise", "fsim.guidance.hsa", S, 3, 0, "", true},  // HSA-05
    {"fsim.guidance.hsa/speed/max_endurance", "fsim.guidance.hsa", S, 3, 0, "", true},
    {"fsim.guidance.hsa/altitude/msl", "fsim.guidance.hsa", S, 0, 0, ""},                   // HSA-06
    {"fsim.guidance.hsa/altitude/agl", "fsim.guidance.hsa", S, 0, 0, ""},
    {"fsim.guidance.hsa/altitude/hae", "fsim.guidance.hsa", S, 0, 0, ""},
    {"fsim.guidance.hsa/altitude/barometric", "fsim.guidance.hsa", S, 0, 0, ""},            // HSA-07 (FA-4b)

    // --- waypoint following ------------------------------------------------------------------------
    CAP("fsim.guidance.route", P, 7, 0,
        "climb optimisation, times of arrival, paths and branches, path terminators, "
        "4D states and navigation performance (FA-6); planning metadata (FA-7)"), // WPT-01, 16, 24, 25
    {"fsim.guidance.route/projection/great_circle", "fsim.guidance.route", S, 0, 0, ""},     // WPT-02
    {"fsim.guidance.route/projection/rhumb_line", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/turn/fly_by", "fsim.guidance.route", S, 0, 0, ""},                 // WPT-03
    {"fsim.guidance.route/turn/fly_over", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/turn/capture_outbound_course", "fsim.guidance.route", S, 0, 0, ""}, // WPT-04 (FA-6b)
    {"fsim.guidance.route/turn/start_turn", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/turn/end_turn", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/turn/radius", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/course_at_point", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/segment_speed", "fsim.guidance.route", S, 0, 0, ""},               // WPT-05
    {"fsim.guidance.route/speed/long_range_cruise", "fsim.guidance.route", S, 6, 0, "", true}, // WPT-06 (FA-6c1)
    {"fsim.guidance.route/speed/max_endurance", "fsim.guidance.route", S, 6, 0, "", true},
    {"fsim.guidance.route/climb_rate", "fsim.guidance.route", S, 0, 0, ""},                  // WPT-07
    {"fsim.guidance.route/climb/best_rate", "fsim.guidance.route", N, 6, 0, ""},             // WPT-08
    {"fsim.guidance.route/climb/extended_range", "fsim.guidance.route", N, 6, 0, ""},
    {"fsim.guidance.route/max_roll", "fsim.guidance.route", S, 0, 0, ""},                    // WPT-09
    {"fsim.guidance.route/acceleration", "fsim.guidance.route", S, 0, 0, ""},                // WPT-10 (FA-6c1)
    {"fsim.guidance.route/required_time_of_arrival", "fsim.guidance.route", N, 6, 0, ""},    // WPT-11
    {"fsim.guidance.route/altitude/msl", "fsim.guidance.route", S, 0, 0, ""},                // WPT-12
    {"fsim.guidance.route/altitude/agl", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/altitude/hae", "fsim.guidance.route", S, 0, 0, ""},
    {"fsim.guidance.route/altitude/barometric", "fsim.guidance.route", S, 0, 0, ""},          // (FA-6a)
    {"fsim.guidance.route/altitude_block", "fsim.guidance.route", S, 0, 0, ""},              // (FA-6a)
    {"fsim.guidance.route/paths", "fsim.guidance.route", N, 6, 0, ""},                       // WPT-13
    {"fsim.guidance.route/next_segment", "fsim.guidance.route", N, 6, 0, ""},                // WPT-14
    {"fsim.guidance.route/conditional_segment", "fsim.guidance.route", N, 6, 0, ""},         // WPT-15
    {"fsim.guidance.route/waypoint_type", "fsim.guidance.route", S, 0, 0, ""},               // WPT-17 (FA-6a: nav only, passive, end of path)
    {"fsim.guidance.route/waypoint_type/taxi", "fsim.guidance.route", N, 9, R2, ""},          //   the points each action flies
    {"fsim.guidance.route/waypoint_type/runway", "fsim.guidance.route", N, 9, 0, ""},
    {"fsim.guidance.route/waypoint_type/takeoff", "fsim.guidance.route", N, 9, 0, ""},
    {"fsim.guidance.route/waypoint_type/landing", "fsim.guidance.route", N, 10, 0, ""},
    {"fsim.guidance.route/waypoint_type/hard_ditch", "fsim.guidance.route", N, 16, 0, ""},
    {"fsim.guidance.route/loiter_point", "fsim.guidance.route", S, 0, 0, ""},                // WPT-18 (FA-6b2)
    {"fsim.guidance.route/path_terminators", "fsim.guidance.route", N, 6, 0, ""},            // WPT-19
    {"fsim.guidance.route/inertial_states", "fsim.guidance.route", N, 6, 0, ""},             // WPT-20
    {"fsim.guidance.route/required_navigation_performance", "fsim.guidance.route", N, 6, 0, ""}, // WPT-21
    {"fsim.guidance.route/relative_points", "fsim.guidance.route", S, 0, 0, ""},             // WPT-22 (FA-6a)
    {"fsim.guidance.route/metadata", "fsim.guidance.route", N, 7, 0, ""},                    // WPT-23
    CAP("fsim.guidance.taxi", N, 9, R2, ""),                                                  // WPT-26

    // --- curve following ----------------------------------------------------------------------------
    CAP("fsim.guidance.curve", S, 0, 0, ""), // CRV-02, 12..14
    {"fsim.guidance.curve/bezier", "fsim.guidance.curve", S, 0, 0, ""},                      // CRV-01
    {"fsim.guidance.curve/nurbs", "fsim.guidance.curve", S, 0, 0, ""},                       // CRV-03 (FA-5d1)
    {"fsim.guidance.curve/reference/geodetic", "fsim.guidance.curve", S, 0, 0, ""},          // CRV-04 (FA-5d2)
    {"fsim.guidance.curve/reference/frame", "fsim.guidance.curve", S, 0, 0, ""},             // CRV-05 (FA-5d2)
    {"fsim.guidance.curve/offsets", "fsim.guidance.curve", S, 0, 0, ""},                     // CRV-06 (FA-5d2)
    {"fsim.guidance.curve/traversal/speed_range", "fsim.guidance.curve", S, 0, 0, ""},       // CRV-07
    {"fsim.guidance.curve/traversal/duration", "fsim.guidance.curve", S, 0, 0, ""},
    {"fsim.guidance.curve/curvature", "fsim.guidance.curve", S, 0, 0, ""},                   // CRV-08 (FA-5d1: checked, the indices too)
    {"fsim.guidance.curve/append", "fsim.guidance.curve", S, 0, 0, ""},                      // CRV-09
    {"fsim.guidance.curve/end/csa", "fsim.guidance.curve", S, 0, 0, ""},                     // CRV-10
    {"fsim.guidance.curve/end/circular_loiter", "fsim.guidance.curve", S, 0, 0, ""},        // CRV-11 (FA-5d3)
    {"fsim.guidance.curve/discretized", "fsim.guidance.curve", N, 17, 0, ""},                // CRV-15

    // --- loiter -----------------------------------------------------------------------------------------
    CAP("fsim.guidance.pattern", S, 0, 0, ""),                                               // LTR-09
    {"fsim.guidance.pattern/orbit", "fsim.guidance.pattern", S, 0, 0, ""},                   // LTR-01
    {"fsim.guidance.pattern/racetrack", "fsim.guidance.pattern", S, 0, 0, ""},               // LTR-02
    {"fsim.guidance.pattern/figure_eight", "fsim.guidance.pattern", S, 0, 0, ""},            // LTR-04
    {"fsim.guidance.pattern/two_circles", "fsim.guidance.pattern", S, 0, 0, ""},             // LTR-03 (FA-5a)
    {"fsim.guidance.pattern/fix_point", "fsim.guidance.pattern", S, 0, 0, ""},               // LTR-05 (FA-5a)
    {"fsim.guidance.pattern/duration/time", "fsim.guidance.pattern", S, 0, 0, ""},           // LTR-06
    {"fsim.guidance.pattern/duration/orbits", "fsim.guidance.pattern", S, 0, 0, ""},         // (FA-5a)
    {"fsim.guidance.pattern/entry_point", "fsim.guidance.pattern", S, 0, 0, ""},             // LTR-07 (FA-5a)
    {"fsim.guidance.pattern/exit_point", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/hold", "fsim.guidance.pattern", S, 0, 0, ""},                    // LTR-08
    {"fsim.guidance.pattern/hold/leg_time", "fsim.guidance.pattern", S, 0, 0, ""},           // LTR-10 (FA-5a)
    {"fsim.guidance.pattern/hold/turn_rate", "fsim.guidance.pattern", S, 0, 0, ""},          // LTR-11 (FA-5b)
    {"fsim.guidance.pattern/hold/turn_type", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/hold/duration", "fsim.guidance.pattern", S, 0, 0, ""},           // LTR-12 (FA-5b: entry and exit times, the time window)
    {"fsim.guidance.pattern/entry/direct", "fsim.guidance.pattern", S, 0, 0, ""},            // LTR-13 (FA-5b)
    {"fsim.guidance.pattern/entry/anchor", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/entry/inbound", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/entry/outbound", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/entry/parallel", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/entry/teardrop", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/hold/context", "fsim.guidance.pattern", S, 0, 0, ""},            // LTR-14 (FA-5b: ATC's defaults for every context)
    {"fsim.guidance.pattern/hover", "fsim.guidance.pattern", S, 0, R1, ""},                  // LTR-15 (FA-5c)
    {"fsim.guidance.pattern/altitude/msl", "fsim.guidance.pattern", S, 0, 0, ""},            // LTR-16
    {"fsim.guidance.pattern/altitude/agl", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/altitude/hae", "fsim.guidance.pattern", S, 0, 0, ""},
    {"fsim.guidance.pattern/altitude/barometric", "fsim.guidance.pattern", S, 0, 0, ""},     // LTR-16 (FA-4b)
    {"fsim.guidance.pattern/speed/long_range_cruise", "fsim.guidance.pattern", S, 3, 0, "", true}, // LTR-17
    {"fsim.guidance.pattern/speed/max_endurance", "fsim.guidance.pattern", S, 3, 0, "", true},
    {"fsim.guidance.pattern/relative_points", "fsim.guidance.pattern", S, 0, 0, ""},         // LTR-18 (FA-5c)

    // --- the flight capability types not built yet (CAP-02) --------------------------------------------
    CAP("fsim.guidance.must_fly", N, 8, 0, ""),                                               // MFY-01..07
    CAP("fsim.guidance.marshall", N, 8, 0, ""),                                               // ASM-01
    {"fsim.guidance.marshall/hover", "fsim.guidance.marshall", N, 8, R1, ""},
    CAP("fsim.guidance.intercept", N, 8, 0, ""),                                              // RIC-01..03
    CAP("fsim.guidance.launch", N, 9, 0, ""),                                                 // LCH-01
    {"fsim.guidance.launch/runway", "fsim.guidance.launch", N, 9, 0, ""},
    {"fsim.guidance.launch/rejected_takeoff", "fsim.guidance.launch", N, 9, 0, ""},           // LCH-02
    {"fsim.guidance.launch/vertical", "fsim.guidance.launch", N, 9, R1, ""},                  // LCH-04
    {"fsim.guidance.launch/emergency_divert", "fsim.guidance.launch", N, 16, 0, ""},          // LCH-03
    {"fsim.guidance.launch/carrier_catapult", "fsim.guidance.launch", N, 11, R3, ""},         // LCH-05
    {"fsim.guidance.launch/carrier_proceed", "fsim.guidance.launch", N, 11, R3 | R4, ""},     // LCH-06
    {"fsim.guidance.launch/carrier_join_up", "fsim.guidance.launch", N, 11, R3 | R4, ""},     // LCH-07
    CAP("fsim.guidance.recovery", N, 10, 0, ""),                                              // RCV-01
    {"fsim.guidance.recovery/runway", "fsim.guidance.recovery", N, 10, 0, ""},
    {"fsim.guidance.recovery/go_around", "fsim.guidance.recovery", N, 10, 0, ""},             // RCV-02
    {"fsim.guidance.recovery/missed_approach", "fsim.guidance.recovery", N, 10, 0, ""},       // RCV-03
    {"fsim.guidance.recovery/vertical", "fsim.guidance.recovery", N, 10, R1, ""},             // RCV-04
    {"fsim.guidance.recovery/carrier", "fsim.guidance.recovery", N, 11, R3 | R4, ""},         // RCV-05
    {"fsim.guidance.recovery/carrier_delta", "fsim.guidance.recovery", N, 11, R3 | R4, ""},   // RCV-06
    {"fsim.guidance.recovery/carrier_calls", "fsim.guidance.recovery", N, 11, R3 | R4, ""},   // RCV-07
    {"fsim.guidance.recovery/configuration", "fsim.guidance.recovery", N, 10, R6 | R7 | R5, ""}, // RCV-08
    {"fsim.guidance.recovery/arrester_hook", "fsim.guidance.recovery", N, 11, R5, ""},        // RCV-09
    {"fsim.guidance.recovery/carrier_overrides", "fsim.guidance.recovery", N, 11, R3 | R4, ""}, // RCV-10

    // --- formation as A-GRA defines it (the platform's formation carries it) --------------------------
    {"fsim.guidance.formation/anchor", "fsim.guidance.formation", P, 12, 0,
     "slots by index from a template, a virtual anchor point; an offset behind a leader works"}, // FRM-01
    {"fsim.guidance.formation/templates", "fsim.guidance.formation", N, 12, 0, ""},          // FRM-02
    {"fsim.guidance.formation/keep_in_zone", "fsim.guidance.formation", N, 12, 0, ""},       // FRM-03
    {"fsim.guidance.formation/package", "fsim.guidance.formation", N, 12, 0, ""},            // FRM-04, CAP-28

    // --- the command interface ---------------------------------------------------------------------------
    {"fsim.command/command_id", "", S, 0, 0, ""},                                             // CMD-02
    {"fsim.command/batch", "", S, 0, 0, ""},                                                  // CMD-03
    {"fsim.command/rank", "", S, 0, 0, ""},                                                   // CMD-05, ACT-07
    {"fsim.command/no_interrupt", "", S, 0, 0, ""},                                           // CMD-06
    {"fsim.command/precedence_override", "", S, 0, 0, ""},                                    // CMD-07
    {"fsim.command/time_window", "", S, 0, 0, ""},                                            // CMD-08
    {"fsim.command/override_rejection", "", S, 0, 0, ""},                                   // CMD-09: over the soft rejections (endurance: FA-3e)
    {"fsim.command/traceability", "", S, 0, 0, ""},                                           // CMD-10
    {"fsim.command/validate", "", S, 0, 0, ""},                                               // VAL-12
    {"fsim.command/task", "", S, 0, 0, ""},                                                   // TSK-01, TSK-02
    {"fsim.command/release_envelope", "", N, 13, R9, ""},                                     // WPN-01
    {"fsim.activity/disable", "", S, 0, 0, ""},                                               // CMD-12, ACT-04
    {"fsim.activity/enable", "", S, 0, 0, ""},
    {"fsim.activity/reset", "", S, 0, 0, ""},
    {"fsim.activity/delete", "", S, 0, 0, ""},                                                // CMD-13
    {"fsim.activity/change_rank", "", S, 0, 0, ""},                                           // CMD-14
    {"fsim.activity/unassign", "", S, 0, 0, ""},                                              // CMD-15
    {"fsim.control/grants", "", S, 0, 0, ""},                                                 // AUT-01..05, 07..09
    {"fsim.control/restrict", "", S, 0, 0, ""},                                               // CAP-25
    {"fsim.control/controller_identity", "", S, 0, 0, ""},                                    // AUT-06

    // --- route plans, settings and the supporting models a consumer uses ------------------------------
    {"fsim.plan/store", "", N, 7, 0, ""},                        // RPL-01, 02, 05, 08..11
    {"fsim.plan/fa_plans", "", N, 7, 0, ""},                     // RPL-03
    {"fsim.plan/validate", "", N, 7, 0, ""},                     // RPL-06, RPL-07, ENV-10
    {"fsim.plan/airfields", "", N, 7, 0, ""},                    // RPL-04, ENV-05
    {"fsim.setting/qnh", "", S, 0, 0, ""},                       // STS-10, ENV-03: the altimeter's setting (FA-4b)
    {"fsim.setting/lights", "", N, 14, 0, ""},                   // STS-17
    {"fsim.setting/antennas", "", N, 14, 0, ""},
    {"fsim.setting/transponder", "", N, 14, 0, ""},
    {"fsim.setting/radio", "", N, 14, 0, ""},
    {"fsim.setting/rf_transmit", "", N, 14, 0, ""},
    {"fsim.setting/iff", "", N, 14, 0, ""},
    {"fsim.setting/comm_allocation", "", N, 14, 0, ""},
    {"fsim.setting/survivability", "", N, 14, 0, ""},            // STS-18
    {"fsim.setting/lost_comm", "", N, 16, 0, ""},                // STS-19
    {"fsim.query/terrain", "", S, 0, 0, ""},                     // STS-11, ENV-01: the query, and the paths checked against it (FA-4a)
    {"fsim.frame", "", S, 0, 0, ""},                             // ENV-04: frames by id, and their points (FA-4c)
    {"fsim.geometry", "", N, 8, 0, ""},                          // ENV-06
    {"fsim.ship", "", N, 11, 0, ""},                             // ENV-07
    {"fsim.stores", "", N, 13, R9, ""},                          // SUB-07
    {"fsim.fault", "", N, 16, 0, ""},                            // SUB-09, STS-09

    // --- FA's own functions (VI 1.4) --------------------------------------------------------------------------
    {"fsim.fa/envelope_management", "", S, 0, 0, ""},            // CTG-04: protection's limits, energy management (FA-3d)
    {"fsim.fa/collision_avoidance", "", P, 15, 0, "detection and the avoidance manoeuvre; the platform can restrict capabilities for it"}, // CTG-01
    {"fsim.fa/geofence", "", N, 15, 0, ""},                      // CTG-02, ENV-08
    {"fsim.fa/terrain_avoidance", "", N, 15, 0, ""},             // CTG-03
    {"fsim.fa/tight_formation", "", N, 15, 0, ""},               // FRM-05
    {"fsim.fa/fuel_management", "", P, 16, 0, "fuel monitoring, abnormal flow, the low-fuel procedure; the navigation report gives the fuel, its endurance and the low-fuel contingency"}, // CTG-05
    {"fsim.fa/contingency_management", "", N, 16, 0, ""},        // CTG-06, CTG-11
    {"fsim.fa/emergency_divert", "", N, 16, 0, ""},              // CTG-07
    {"fsim.fa/failsafe_plan", "", N, 16, 0, ""},                 // CTG-08
    {"fsim.fa/lost_link", "", N, 16, 0, ""},                     // CTG-09
    {"fsim.fa/faults", "", N, 16, 0, ""},                        // CTG-10
};

#undef CAP

constexpr std::size_t kFeatureCount = sizeof(kFeatures) / sizeof(kFeatures[0]);

const FeatureDef* definition(std::string_view feature) noexcept {
    for (const auto& f : kFeatures)
        if (feature == f.id) return &f;
    return nullptr;
}

const char* declaredValue(Characteristic c, const ApplicabilitySection& a) noexcept {
    switch (c) {
    case Characteristic::GroundContact:
        return a.groundContact == GroundContact::Wheels ? "wheels" : a.groundContact == GroundContact::Skids ? "skids" : "legs";
    case Characteristic::Carrier:
        return a.carrier == CarrierOperations::None ? "none" : a.carrier == CarrierOperations::CatapultArrested ? "catapult_arrested" : "deck";
    case Characteristic::DragDevices: return "none";
    case Characteristic::ReleasableStores: return "none";
    default: return "false";
    }
}

/// "ground_contact = skids: <its source>"
std::string declaration(Characteristic c, const ApplicabilitySection& a) {
    return std::string(characteristicName(c)) + " = " + declaredValue(c, a) + ": " + a.sources[static_cast<std::size_t>(c)];
}

const char* familyName(ControlFamily f) noexcept {
    switch (f) {
    case ControlFamily::Direct: return "direct";
    case ControlFamily::FlyByWire: return "fly-by-wire";
    case ControlFamily::Helicopter: return "helicopter";
    case ControlFamily::Multirotor: return "multirotor";
    default: return "stock";
    }
}

} // namespace

const char* supportName(Support support) noexcept {
    switch (support) {
    case Support::Supported: return "supported";
    case Support::Partial: return "partial";
    case Support::NotImplemented: return "not_implemented";
    case Support::NotSupported: return "not_supported";
    }
    return "?";
}

const char* ruleName(Rule rule) noexcept {
    static const char* const names[] = {"", "R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8", "R9", "R10", "R11", "R12", "R13"};
    const auto i = static_cast<std::size_t>(rule);
    return i < kRuleCount ? names[i] : "";
}

const char* ruleDescription(Rule rule) noexcept {
    switch (rule) {
    case Rule::Hover: return "holding a point in the air (hover, the vertical take-off and landing) applies to aircraft that fly on rotors";
    case Rule::GroundTaxi: return "taxi paths apply to aircraft that roll on wheels";
    case Rule::CatapultArrested: return "a catapult launch and an arrested landing apply to carrier-capable fixed-wing types";
    case Rule::DeckOperations: return "take-off from and landing on a moving deck apply to aircraft that take off and land vertically";
    case Rule::ArresterHook: return "the arrester hook applies to carrier-capable fixed-wing types";
    case Rule::RetractableGear: return "gear retraction and extension apply to aircraft with retractable gear";
    case Rule::Flaps: return "flaps apply to aircraft with a flap function";
    case Rule::DragDevices:
        return "spoilers, airbrakes and deceleration by them apply to aircraft with spoilers, airbrakes or surfaces their flight "
               "control system deploys as a speedbrake";
    case Rule::ReleasableStores: return "the release envelope and stores apply to aircraft that carry and release weapon stores";
    case Rule::Aerobatic: return "the aerobatic manoeuvres apply to fixed-wing types cleared for them";
    case Rule::WheelBrakes: return "wheel brakes apply to aircraft that roll on wheels";
    case Rule::PitchTrim:
        return "a pitch trim input applies to aircraft whose controls move their surfaces directly: a fly-by-wire law and a rotorcraft's "
               "loops trim themselves";
    case Rule::EngineThrottles:
        return "a throttle per engine applies to aircraft with more than one engine the pilot moves apart: a helicopter's governed "
               "engines follow its collective";
    default: return "";
    }
}

std::size_t supportFeatureCount() noexcept { return kFeatureCount; }

const char* supportFeature(std::size_t index) noexcept { return index < kFeatureCount ? kFeatures[index].id : nullptr; }

RuleVerdicts judgeRules(const VehicleProfile& profile) {
    RuleVerdicts v;
    const ApplicabilitySection& a = profile.applicability;
    auto exclude = [&v](Rule rule, std::string evidence) {
        v.excluded |= ruleBit(rule);
        v.evidence[static_cast<std::size_t>(rule)] = std::move(evidence);
    };
    // what the design declares (a characteristic without its source is not declared: requireSources)
    auto is = [&a](Characteristic c) { return declares(a, c); };
    if (is(Characteristic::VerticalFlight) && a.verticalFlight == Declared::No)
        exclude(Rule::Hover, declaration(Characteristic::VerticalFlight, a));
    if (is(Characteristic::GroundContact) && a.groundContact != GroundContact::Wheels) {
        exclude(Rule::GroundTaxi, declaration(Characteristic::GroundContact, a));
        exclude(Rule::WheelBrakes, declaration(Characteristic::GroundContact, a));
    }
    if (is(Characteristic::Carrier)) {
        if (a.carrier != CarrierOperations::CatapultArrested) {
            exclude(Rule::CatapultArrested, declaration(Characteristic::Carrier, a));
            exclude(Rule::ArresterHook, declaration(Characteristic::Carrier, a));
        }
        if (a.carrier != CarrierOperations::Deck) exclude(Rule::DeckOperations, declaration(Characteristic::Carrier, a));
    }
    if (is(Characteristic::RetractableGear) && a.retractableGear == Declared::No)
        exclude(Rule::RetractableGear, declaration(Characteristic::RetractableGear, a));
    if (is(Characteristic::Flaps) && a.flaps == Declared::No) exclude(Rule::Flaps, declaration(Characteristic::Flaps, a));
    if (is(Characteristic::DragDevices) && a.dragDevices == DragDevices::None)
        exclude(Rule::DragDevices, declaration(Characteristic::DragDevices, a));
    if (is(Characteristic::ReleasableStores) && a.releasableStores == ReleasableStores::None)
        exclude(Rule::ReleasableStores, declaration(Characteristic::ReleasableStores, a));
    if (is(Characteristic::Aerobatic) && a.aerobatic == Declared::No) exclude(Rule::Aerobatic, declaration(Characteristic::Aerobatic, a));
    // the platform's own effectors, on what the aircraft file says of its controls and engines
    const ControlFamily family = profile.identity.family;
    if (profile.identity.header.present() && (family == ControlFamily::FlyByWire || isRotorcraft(family)))
        exclude(Rule::PitchTrim, std::string("fsim/identity/family = ") + familyName(family) + ": the aircraft file");
    if (profile.identity.header.present() && family == ControlFamily::Helicopter)
        exclude(Rule::EngineThrottles, "fsim/identity/family = helicopter: the aircraft file (its engines are governed)");
    else if (profile.propulsion.header.present() && profile.propulsion.engines == 1 && family != ControlFamily::Multirotor)
        exclude(Rule::EngineThrottles, "fsim/propulsion/engines = 1: the aircraft file");
    return v;
}

std::uint16_t capabilityRules(std::string_view capability) noexcept {
    const FeatureDef* f = definition(capability);
    return f ? f->rules : 0;
}

bool excluded(std::uint16_t rules, std::uint16_t excludedRules) noexcept { return rules != 0 && (rules & ~excludedRules) == 0; }

const char* supersededBy(std::string_view capability) noexcept {
    if (capability == "fsim.guidance.hold") return "fsim.guidance.hsa";
    if (capability == "fsim.guidance.waypoints") return "fsim.guidance.route";
    if (capability == "fsim.guidance.loiter") return "fsim.guidance.pattern";
    if (capability == "fsim.guidance.hover") return "fsim.guidance.pattern/hover";
    return "";
}

std::string featureOf(const Command& command) {
    if (const auto* b = std::get_if<BehaviorCommand>(&command))
        return b->id.find('.') == std::string::npos ? "fsim.guidance." + b->id : b->id;
    if (std::holds_alternative<HsaCommand>(command)) return "fsim.guidance.hsa";
    if (std::holds_alternative<RouteCommand>(command)) return "fsim.guidance.route";
    if (std::holds_alternative<PatternCommand>(command)) return "fsim.guidance.pattern";
    if (std::holds_alternative<CurveCommand>(command)) return "fsim.guidance.curve";
    return "";
}

std::string featureOf(const SupportCommand& command) { return supportCapability(command.index()); }

SupportTable::SupportTable(const VehicleProfile& profile, const CapabilityCatalog& catalog) {
    const RuleVerdicts verdicts = judgeRules(profile);
    rows_.resize(kFeatureCount);
    evidence_.resize(kFeatureCount);
    // A feature is excluded where every rule governing it excludes the
    // aircraft; the evidence is each one's. Otherwise a capability is what the
    // catalog offers (built, or partial) or not built yet for this aircraft, and
    // a finer feature follows its capability where that one is not offered.
    auto judge = [&](const FeatureDef& f, SupportInfo& row, std::string& evidence) {
        row.feature = f.id;
        row.capability = f.capability;
        row.rules = f.rules;
        row.stage = f.built == Support::Supported ? 0 : f.stage;
        row.missing = f.built == Support::Partial ? f.missing : "";
        if (excluded(f.rules, verdicts.excluded)) {
            row.support = Support::NotSupported;
            row.stage = 0;
            row.missing = "";
            for (std::size_t r = 1; r < kRuleCount; ++r)
                if (f.rules & (1u << r)) {
                    const std::string& e = verdicts.evidence[r];
                    if (evidence.find(e) != std::string::npos) continue; // (R3 and R4 rest on the same declaration)
                    if (!evidence.empty()) evidence += "; ";
                    evidence += e;
                }
            return;
        }
        row.support = f.built;
        if (f.capability == f.id && f.built != Support::NotImplemented && catalog.find(f.id) < 0) {
            row.support = Support::NotImplemented; // applicable, and the aircraft's model lacks it
            row.stage = f.stage;
            row.missing = "";
        }
        if (f.capability != f.id && f.rules == R1 && f.built != Support::NotImplemented && !(catalog.features() & kFeatureHover)) {
            row.support = Support::NotImplemented; // a hover option, its rule undecided (no declaration), and a model that does not hover:
            row.stage = f.stage;                   // as the hover capability is for it (ADR-29 FA-5c)
            row.missing = "";
        }
        if (f.tables && profile.tables.empty() && row.support != Support::NotImplemented) {
            row.support = Support::NotImplemented; // flown from performance tables it has none of
            row.stage = f.stage;
            row.missing = "";
        }
    };
    for (std::size_t i = 0; i < kFeatureCount; ++i) judge(kFeatures[i], rows_[i], evidence_[i]);
    for (std::size_t i = 0; i < kFeatureCount; ++i) {
        const FeatureDef& f = kFeatures[i];
        if (f.capability == f.id || !*f.capability || rows_[i].support == Support::NotSupported) continue;
        const SupportInfo* parent = find(f.capability);
        if (!parent || parent->support == Support::Supported || parent->support == Support::Partial) continue;
        rows_[i].support = parent->support; // what cannot fly at all has none of its options
        rows_[i].stage = std::max(parent->stage, rows_[i].stage);
        rows_[i].missing = "";
        if (parent->support == Support::NotSupported) { // excluded as its capability is: the same rules and evidence
            rows_[i].rules = parent->rules;
            rows_[i].stage = 0;
            evidence_[i] = parent->evidence;
        }
    }
    for (std::size_t i = 0; i < kFeatureCount; ++i) rows_[i].evidence = evidence_[i].c_str();
}

const SupportInfo* SupportTable::find(std::string_view feature) const noexcept {
    static constexpr std::string_view kGuidance = "fsim.guidance.";
    const bool bare = feature.find('.') == std::string_view::npos && feature.find('/') == std::string_view::npos; // a behaviour's id
    for (const auto& row : rows_) {
        const std::string_view id = row.feature;
        if (id == feature) return &row;
        if (bare && id.size() == kGuidance.size() + feature.size() && id.substr(0, kGuidance.size()) == kGuidance &&
            id.substr(kGuidance.size()) == feature)
            return &row;
    }
    return nullptr;
}

Reason SupportTable::refusal(std::string_view feature) const noexcept {
    const SupportInfo* row = find(feature);
    if (!row) return Reason::UnknownCapability;
    if (row->support == Support::NotSupported) return Reason::NotSupported;
    if (row->support == Support::NotImplemented) return Reason::NotImplemented;
    return Reason::UnknownCapability; // (offered: the catalog has it)
}

} // namespace fsim::control
