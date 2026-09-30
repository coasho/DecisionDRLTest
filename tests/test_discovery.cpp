// Discovery that tells the truth (docs/flight-autonomy.md, section 4): a
// vehicle's support for every public feature - supported, partial, not
// implemented (with the stage), not supported (with the rules and the
// aircraft's evidence) - the refusals that say which, availability apart from
// support (never Disabled for what the aircraft cannot do), the flight phase
// a policy is answered in, and a status that agrees with admission.
#include "mode_flights.h"

#include "control/Features.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <set>
#include <string>
#include <string_view>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

session::VehicleSpec parked(const std::string& aircraft, const std::string& name) {
    session::VehicleSpec s;
    s.type = "jsbsim:" + aircraft;
    s.name = name;
    s.initial.onGround = true;
    return s;
}

bool offers(session::World& w, std::uint32_t v, std::string_view id) {
    for (const auto& d : w.capabilities(v))
        if (d.id == id) return true;
    return false;
}

const CapabilityDescriptor& descriptorOf(session::World& w, std::uint32_t v, std::string_view id) {
    static const CapabilityDescriptor none;
    for (const auto& d : w.capabilities(v))
        if (d.id == id) return d;
    FAIL("no capability " << id);
    return none;
}

/// A feature's support on the vehicle (it must have one).
const SupportInfo& supportOf(const session::World& w, std::uint32_t v, std::string_view feature) {
    static const SupportInfo none;
    const SupportInfo* s = w.support(v, feature);
    if (!s) FAIL("no feature " << feature);
    return s ? *s : none;
}

BehaviorCommand behavior(const char* id) {
    BehaviorCommand b;
    b.id = id;
    return b;
}

HsaCommand hsaNow(const sim::VehicleState& s) {
    HsaCommand h;
    h.headingRad = s.eulerRad[2], h.speed = std::max(s.airspeedTrueMs, 60.0), h.speedReference = 0.0, h.altitudeM = s.altitudeMslM + 300.0;
    return h;
}

} // namespace

TEST_CASE("discovery: the public features, each answered on every vehicle", "[discovery]") {
    std::set<std::string> ids;
    for (std::size_t i = 0; i < supportFeatureCount(); ++i) {
        const std::string id = supportFeature(i);
        INFO(id);
        CHECK(ids.insert(id).second); // once each
        CHECK(id.compare(0, 5, "fsim.") == 0);
        CHECK(id.find(' ') == std::string::npos);
        CHECK(id.find("-0") == std::string::npos); // (no audit id is a public identifier: D14)
    }
    CHECK(supportFeature(supportFeatureCount()) == nullptr);
    CHECK(ids.count("fsim.guidance.must_fly") == 1); // the flight capability types reserved (4.2)
    CHECK(ids.count("fsim.guidance.launch") == 1);
    CHECK(ids.count("fsim.guidance.hsa/direction/magnetic_north") == 1);

    session::World w(options("discovery-table"));
    const auto c172x = wing(w, "c172x", 1500.0, 55.0);
    const SupportTable* table = w.supportTable(c172x);
    REQUIRE(table != nullptr);
    CHECK(table->size() == supportFeatureCount());
    for (std::size_t i = 0; i < table->size(); ++i) {
        const SupportInfo& row = table->at(i);
        INFO(row.feature);
        CHECK(std::string_view(row.feature) == supportFeature(i));
        // an offered capability is supported or partial; a partial one says what is missing and where it comes
        if (row.support == Support::Partial) CHECK(*row.missing != '\0');
        if (row.support == Support::Partial) CHECK(row.stage > 0);
        if (std::string_view(row.feature) == row.capability && (row.support == Support::Supported || row.support == Support::Partial))
            CHECK(offers(w, c172x, row.feature));
    }
    // a stock aircraft declares nothing: no evidence, no exception
    for (const auto& row : table->rows()) CHECK(row.support != Support::NotSupported);
    CHECK(supportOf(w, c172x, "fsim.guidance.hsa").support == Support::Supported); // (whole since FA-4)
    CHECK(supportOf(w, c172x, "fsim.guidance.hsa/direction/magnetic_north").support == Support::Supported);
    CHECK(supportOf(w, c172x, "fsim.guidance.route/altitude/barometric").support == Support::Supported); // (FA-6a)
    CHECK(supportOf(w, c172x, "fsim.guidance.route/path_terminators").support == Support::Supported); // (FA-6f)
    CHECK(supportOf(w, c172x, "fsim.guidance.route/metadata").support == Support::Supported); // (FA-7a)
    CHECK(supportOf(w, c172x, "fsim.guidance.curve/discretized").stage == 17); // (not built: the stage that builds it)
    CHECK(supportOf(w, c172x, "fsim.guidance.route/paths").support == Support::Supported); // (FA-6e1)
    CHECK(supportOf(w, c172x, "fsim.guidance.route/conditional_segment").support == Support::Partial); // (FA-6e2a: endurance, contingency FA-6e2b's)
    CHECK(supportOf(w, c172x, "hold").support == Support::Supported); // a behaviour's id finds its feature
    CHECK(w.support(c172x, "fsim.guidance.warp_drive") == nullptr);
    CHECK(w.support(999, "fsim.guidance.hsa") == nullptr);
}

TEST_CASE("discovery: a physical exception is not supported, with its rule and the aircraft's evidence; never offered", "[discovery]") {
    session::World w(options("discovery-exceptions"));
    const auto viper = wing(w, "f16c", 3000.0, 160.0);
    const auto bomber = wing(w, "b52h", 6000.0, 200.0, 1);
    const auto cessna = wing(w, "c172", 1500.0, 55.0, 2);

    const SupportInfo& hover = supportOf(w, viper, "fsim.guidance.hover");
    CHECK(hover.support == Support::NotSupported);
    CHECK(hover.rules == ruleBit(Rule::Hover));
    CHECK(std::string_view(hover.evidence).find("vertical_flight = false: USAF F-16 fact sheet") == 0);
    CHECK_FALSE(offers(w, viper, "fsim.guidance.hover"));
    const CommandResult refused = w.submit(viper, behavior("hover"));
    CHECK(refused.reason == Reason::NotSupported);
    const CapabilityStatus status = w.capabilityStatus(viper, "fsim.guidance.hover");
    CHECK(status.availability == Availability::Unavailable); // not Disabled: that means switched off
    CHECK(status.reason == Reason::NotSupported);
    CHECK(status.reasons == reasonBit(Reason::NotSupported));
    CHECK(w.requestControl(viper, "fsim.guidance.hover") == Reason::NotSupported);

    // R10: aerobatics on an aerobatic fighter, never on a bomber
    CHECK(offers(w, viper, "fsim.guidance.aerobatics"));
    CHECK_FALSE(offers(w, bomber, "fsim.guidance.aerobatics"));
    CHECK(supportOf(w, bomber, "aerobatics").support == Support::NotSupported);
    CHECK(w.submit(bomber, behavior("aerobatics")).reason == Reason::NotSupported);
    // R8, R12, R13: the Cessna's drag devices, a fighter's trim and single engine
    CHECK(supportOf(w, cessna, "fsim.support.speedbrake").support == Support::NotSupported);
    CHECK(w.submit(cessna, SpeedbrakeCommand{1.0}).reason == Reason::NotSupported);
    CHECK(supportOf(w, viper, "fsim.support.pitch_trim").rules == ruleBit(Rule::PitchTrim));
    CHECK(supportOf(w, viper, "fsim.flight.engines").support == Support::NotSupported);
    CHECK(supportOf(w, bomber, "fsim.flight.engines").support == Support::Supported);
    // an option follows its capability's rules: the catalog launch is the F-16C's exception, not the F/A-18C's
    const SupportInfo& catapult = supportOf(w, viper, "fsim.guidance.launch/carrier_catapult");
    CHECK(catapult.support == Support::NotSupported);
    CHECK(std::string_view(catapult.evidence).find("carrier = none") == 0);
    // "R6, R7 or R5": the Cessna's flaps keep the recovery's configuration commands applicable
    CHECK(supportOf(w, cessna, "fsim.guidance.recovery/configuration").support == Support::NotImplemented);
    CHECK(supportOf(w, cessna, "fsim.guidance.recovery/configuration").rules == (ruleBit(Rule::RetractableGear) | ruleBit(Rule::Flaps) |
                                                                               ruleBit(Rule::ArresterHook)));
}

TEST_CASE("discovery: applicable but not built is not implemented, with the stage that builds it", "[discovery]") {
    session::World w(options("discovery-unbuilt"));
    const auto viper = wing(w, "f16c", 3000.0, 160.0);
    const SupportInfo& speedbrake = supportOf(w, viper, "fsim.support.speedbrake");
    CHECK(speedbrake.support == Support::NotImplemented); // the type has one, the model not yet
    CHECK(speedbrake.stage == 10);
    CHECK(w.submit(viper, SpeedbrakeCommand{1.0}).reason == Reason::NotImplemented);
    CHECK(w.capabilityStatus(viper, "fsim.support.speedbrake").reason == Reason::NotImplemented);
    CHECK(supportOf(w, viper, "fsim.guidance.launch").support == Support::Supported); // (built by FA-9a)
    CHECK(supportOf(w, viper, "fsim.guidance.launch/rejected_takeoff").support == Support::NotImplemented);
    CHECK(supportOf(w, viper, "fsim.guidance.launch/rejected_takeoff").stage == 9);
    CHECK(w.submit(viper, behavior("launch")).reason == Reason::Airborne); // a ground mode, the Viper flying
    // an id no platform defines is unknown, as before; its status is Unavailable, not Disabled
    CHECK(w.submit(viper, behavior("warp_drive")).reason == Reason::UnknownCapability);
    const CapabilityStatus unknown = w.capabilityStatus(viper, "fsim.guidance.warp_drive");
    CHECK(unknown.availability == Availability::Unavailable);
    CHECK(unknown.reason == Reason::UnknownCapability);
    CHECK(w.capabilityStatus(999, "fsim.guidance.hsa").reason == Reason::UnknownVehicle);
}

TEST_CASE("discovery: the descriptors say how a capability is controlled and what supersedes a platform behaviour", "[discovery]") {
    session::World w(options("discovery-descriptors"));
    const auto v = wing(w, "f16c", 3000.0, 160.0);
    CHECK(descriptorOf(w, v, "fsim.guidance.hsa").accepted == (kAcceptsCapabilityCommand | kAcceptsActivityCommand));
    CHECK(descriptorOf(w, v, "fsim.flight.attitude").accepted == (kAcceptsCapabilityCommand | kAcceptsActivityCommand));
    CHECK(descriptorOf(w, v, "fsim.envelope.protection").accepted == kAcceptsAutoMdf);
    CHECK(descriptorOf(w, v, "fsim.guidance.hold").superseded == "fsim.guidance.hsa");
    CHECK(descriptorOf(w, v, "fsim.guidance.waypoints").superseded == "fsim.guidance.route");
    CHECK(descriptorOf(w, v, "fsim.guidance.loiter").superseded == "fsim.guidance.pattern");
    CHECK(descriptorOf(w, v, "fsim.guidance.hsa").superseded.empty());
}

TEST_CASE("the flight phase: on the ground a policy's airborne guidance waits; FA's own sources and the flight levels never do", "[discovery]") {
    session::World w(options("discovery-phase"));
    const auto v = w.createVehicle(parked("f16c", "parked"));
    REQUIRE(v != 0);
    w.step(stepsFor(w, 2.0));
    REQUIRE(w.vehicleState(v)->onGround);
    const CapabilityStatus hsa = w.capabilityStatus(v, "fsim.guidance.hsa");
    CHECK(hsa.availability == Availability::TemporarilyUnavailable);
    CHECK(hsa.reason == Reason::OnGround);
    CHECK(std::string_view(hsa.description) == reasonDescription(Reason::OnGround));
    CHECK(w.capabilityStatus(v, "fsim.flight.attitude").availability == Availability::Available);
    CHECK(w.submit(v, hsaNow(*w.vehicleState(v))).reason == Reason::OnGround);
    CHECK(w.submit(v, behavior("hold")).reason == Reason::OnGround);
    CHECK(w.requestControl(v, "fsim.guidance.hsa") == Reason::OnGround);
    // the flight levels are offered in every phase: a policy may fly its own take-off
    CHECK(w.submit(v, AttitudeCommand{0.0, 0.0, kHold, 0.785, 1.0, kHold}).accepted());
    // FA's own control paths are never gated (D13): an autopilot's guidance starts, and its UPDATE goes through
    CommandOptions autopilot;
    autopilot.source = Source::Autopilot;
    const CommandResult own = w.submit(v, hsaNow(*w.vehicleState(v)), autopilot);
    REQUIRE(own.accepted());
    HsaCommand turn;
    turn.headingRad = 1.0;
    CHECK(w.update(Source::Autopilot, own.activity, turn).accepted());
    // the existing entry points keep their behaviour (RangePolicy::None: no availability checks)
    CommandOptions legacy;
    legacy.range = RangePolicy::None;
    legacy.source = Source::Override;
    CHECK(w.submit(v, behavior("hold"), legacy).accepted());
}

TEST_CASE("status and admission agree: the placards narrow a range, and a NEW outside it is refused with the status's reason", "[discovery]") {
    session::World w(options("discovery-placards"));
    const auto v = w.createVehicle(parked("f16c", "parked"));
    REQUIRE(v != 0);
    w.step(stepsFor(w, 2.0));
    REQUIRE(w.vehicleState(v)->onGround);
    const CapabilityStatus gear = w.capabilityStatus(v, "fsim.support.gear");
    CHECK(gear.availability == Availability::Available);
    REQUIRE(gear.rangeCount == 1);
    CHECK(gear.ranges[0].parameter == 0);
    CHECK(gear.ranges[0].min == 0.5); // down
    CHECK(gear.ranges[0].max == 1.0);
    CHECK(w.submit(v, GearCommand{0.0}).reason == Reason::Unavailable);
    CHECK(w.submit(v, GearCommand{1.0}).accepted());

    // above its speeds: the gear not operated at all, the flaps no further out than their threshold
    auto own = std::make_shared<VehicleProfile>(*w.profile(v));
    own->envelope.gearCasMaxMs = 45.0;
    own->envelope.flaps.casMaxMs = 50.0;
    session::VehicleSpec fastSpec;
    fastSpec.type = "jsbsim:f16c";
    fastSpec.name = "fast";
    fastSpec.initial.altitudeMslM = 3000.0;
    fastSpec.initial.airspeedTrueMs = 160.0;
    fastSpec.profile = own;
    const auto fast = w.createVehicle(fastSpec);
    REQUIRE(fast != 0);
    w.step();
    const CapabilityStatus fastGear = w.capabilityStatus(fast, "fsim.support.gear");
    CHECK(fastGear.availability == Availability::TemporarilyUnavailable);
    CHECK(fastGear.reason == Reason::Unavailable);
    CHECK(std::string_view(fastGear.description) == "above the gear's operating speed");
    CHECK(w.submit(fast, GearCommand{1.0}).reason == fastGear.reason);
    const CapabilityStatus flaps = w.capabilityStatus(fast, "fsim.support.flaps");
    CHECK(flaps.availability == Availability::Available);
    REQUIRE(flaps.rangeCount == 1);
    CHECK(flaps.ranges[0].max == own->envelope.flapsThreshold);
    CHECK(w.submit(fast, FlapsCommand{0.5}).reason == Reason::Unavailable);
    CHECK(w.submit(fast, FlapsCommand{0.0}).accepted());
}

TEST_CASE("availability: every reason that holds, what it is about and when it is expected back", "[discovery]") {
    session::World w(options("discovery-reasons"));
    const auto v = w.createVehicle(parked("f16c", "parked"));
    REQUIRE(v != 0);
    w.step(stepsFor(w, 2.0));
    REQUIRE(w.setAvailability(v, "fsim.guidance.hsa", Availability::TemporarilyUnavailable, Reason::CollisionAvoidance, 42, 125.0) ==
            Reason::None);
    const CapabilityStatus both = w.capabilityStatus(v, "fsim.guidance.hsa");
    CHECK(both.reason == Reason::OnGround); // the flight phase first
    CHECK(both.reasons == (reasonBit(Reason::OnGround) | reasonBit(Reason::CollisionAvoidance)));
    CHECK(both.associated == 0); // (the first reason's)
    const CapabilityStatus restricted = w.capabilityStatus(v, "fsim.flight.velocity");
    CHECK(restricted.availability == Availability::Available);
    REQUIRE(w.setAvailability(v, "fsim.flight.velocity", Availability::TemporarilyUnavailable, Reason::CollisionAvoidance, 42, 125.0) ==
            Reason::None);
    const CapabilityStatus avoiding = w.capabilityStatus(v, "fsim.flight.velocity");
    CHECK(avoiding.reason == Reason::CollisionAvoidance);
    CHECK(avoiding.associated == 42);
    CHECK(avoiding.nextAvailableS == 125.0);
    CHECK(std::string_view(avoiding.description) == reasonDescription(Reason::CollisionAvoidance));
    CHECK(w.submit(v, VelocityCommand{60.0, 0.0, kHold, kHold}).reason == Reason::CollisionAvoidance);
    // lifted
    REQUIRE(w.setAvailability(v, "fsim.flight.velocity", Availability::Available, Reason::None) == Reason::None);
    const CapabilityStatus lifted = w.capabilityStatus(v, "fsim.flight.velocity");
    CHECK(lifted.availability == Availability::Available);
    CHECK(lifted.reasons == 0);
    CHECK(std::isnan(lifted.nextAvailableS));
    // a capability the vehicle does not offer: why, not unknown
    CHECK(w.setAvailability(v, "fsim.guidance.hover", Availability::Disabled, Reason::Restricted) == Reason::NotSupported);
}
