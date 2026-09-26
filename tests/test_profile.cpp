// VehicleProfile (docs/control-architecture.md, section 7): sections read
// from an aircraft's fsim/<section> properties, each with its own version
// and provenance; a trainer's sections over the aircraft's.
#include "control/Adapter.h"
#include "control/Catalog.h"
#include "control/Profile.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

/// An aircraft file's fsim/ properties, as FlightModel::properties returns them.
struct Properties {
    std::map<std::string, double> values; // full paths, e.g. "fsim/envelope/clean/n_max"
    PropertySource source() const {
        return [this](std::string_view prefix) {
            std::vector<std::pair<std::string, double>> out;
            const std::string p = std::string(prefix) + "/";
            for (const auto& [path, value] : values)
                if (path.compare(0, p.size(), p) == 0) out.emplace_back(path.substr(p.size()), value);
            return out;
        };
    }
};

bool mentions(const std::vector<std::string>& warnings, const std::string& text) {
    for (const auto& w : warnings)
        if (w.find(text) != std::string::npos) return true;
    return false;
}

constexpr double kDeg = 3.14159265358979323846 / 180.0;

} // namespace

TEST_CASE("a profile's sections: absent ones keep their defaults, present ones their version and provenance", "[profile]") {
    Properties props;
    props.values = {
        {"fsim/identity/class", 2}, {"fsim/identity/family", 2}, {"fsim/identity/provenance", 1},
        {"fsim/envelope/clean/n_max", 9}, {"fsim/envelope/clean/n_min", -3}, {"fsim/envelope/clean/alpha_max_deg", 25},
        {"fsim/envelope/version", 1}, {"fsim/envelope/provenance", 1},
    };
    std::vector<std::string> warnings;
    const VehicleProfile p = readProfile("test", props.source(), warnings);
    CHECK(warnings.empty());
    CHECK(p.aircraft == "test");
    CHECK(p.identity.header.version == 1); // present without a version: 1
    CHECK(p.identity.header.provenance == Provenance::Hangar);
    CHECK(p.identity.aircraftClass == AircraftClass::Fighter);
    CHECK(p.identity.family == ControlFamily::FlyByWire);
    CHECK(p.envelope.header.present());
    CHECK(p.envelope.clean.loadFactorMax == 9.0);
    CHECK_THAT(p.envelope.clean.alphaMaxRad, Catch::Matchers::WithinRel(25.0 * kDeg, 1e-12));
    CHECK(std::isnan(p.envelope.flaps.loadFactorMax));
    CHECK_FALSE(p.effectors.header.present()); // absent: the defaults an actuator command has always had
    CHECK(p.effectors.header.provenance == Provenance::Default);
    CHECK(p.effectors.flaps);
    CHECK_FALSE(p.effectors.speedbrake);
    CHECK(p.control.settings.empty());
    // by path, in the unit the name gives
    CHECK(profileValue(p, "envelope/clean/alpha_max_deg") == 25.0);
    CHECK(profileValue(p, "envelope/clean/n_max") == 9.0);
    CHECK(profileValue(p, "identity/class") == 2.0);
    CHECK(profileValue(p, "envelope/version") == 1.0);
    CHECK(profileValue(p, "effectors/version") == 0.0);
    CHECK(std::isnan(profileValue(p, "envelope/no_such_field")));
    CHECK(std::isnan(profileValue(p, "no_section/x")));
    REQUIRE(sectionHeader(p, "identity") != nullptr);
    CHECK(sectionHeader(p, "nonsense") == nullptr);
}

TEST_CASE("a section newer than this build, a field out of range, one nobody knows, limits that contradict", "[profile]") {
    Properties props;
    props.values = {
        {"fsim/plant/version", 2}, {"fsim/plant/tas_ms", 150},                       // newer: never half-read
        {"fsim/propulsion/engines", 40}, {"fsim/propulsion/type", 2},                // 40 engines: out of range
        {"fsim/performance/stall_cas_ms", 50}, {"fsim/performance/warp_factor", 9},  // unknown field
        {"fsim/envelope/clean/n_min", 2}, {"fsim/envelope/clean/n_max", 3},         // n_min above 1: out of range
        {"fsim/envelope/flaps/cas_min_ms", 80}, {"fsim/envelope/flaps/cas_max_ms", 60}, // contradict
        {"fsim/effectors/speedbrake", 0.5},                                          // a flag is 0 or 1
    };
    std::vector<std::string> warnings;
    const VehicleProfile p = readProfile("odd", props.source(), warnings);
    CHECK_FALSE(p.plant.header.present());
    CHECK(std::isnan(p.plant.tasMs));
    CHECK(mentions(warnings, "fsim/plant is version 2"));
    CHECK(p.propulsion.header.present());
    CHECK(p.propulsion.engines == 0);
    CHECK(p.propulsion.type == EngineType::Turbofan);
    CHECK(mentions(warnings, "fsim/propulsion/engines = 40 is out of range"));
    CHECK(p.performance.stallCasMs == 50.0);
    CHECK(mentions(warnings, "warp_factor is not a field"));
    CHECK(std::isnan(p.envelope.clean.loadFactorMin));
    CHECK(p.envelope.clean.loadFactorMax == 3.0);
    CHECK(std::isnan(p.envelope.flaps.casMinMs));
    CHECK(std::isnan(p.envelope.flaps.casMaxMs));
    CHECK(mentions(warnings, "fsim/envelope/flaps airspeed"));
    CHECK_FALSE(p.effectors.speedbrake);
}

TEST_CASE("the control section is the aircraft's gains, fsim/control as before", "[profile]") {
    Properties props;
    props.values = {{"fsim/control/pid_attitude/pitch/kp", 1.5}, {"fsim/control/pid_velocity/vertical_speed/command_lag", 2.0},
                    {"fsim/control/version", 1}};
    std::vector<std::string> warnings;
    const VehicleProfile p = readProfile("gains", props.source(), warnings);
    CHECK(p.control.header.version == 1);
    REQUIRE(p.control.settings.size() == 2);
    CHECK(p.control.settings[0].controller == "pid_attitude");
    CHECK(p.control.settings[0].parameter == "pitch.kp");
    CHECK(profileValue(p, "control/pid_velocity/vertical_speed/command_lag") == 2.0);
    CHECK(std::isnan(profileValue(p, "control/pid_velocity/nothing")));
}

TEST_CASE("a trainer's sections replace the aircraft's, section by section", "[profile]") {
    VehicleProfile base;
    base.envelope.header = {1, Provenance::Hangar};
    base.envelope.clean.loadFactorMax = 9.0;
    base.identity.header = {1, Provenance::Hangar};
    base.identity.aircraftClass = AircraftClass::Fighter;
    VehicleProfile over;
    over.envelope.header = {1, Provenance::User};
    over.envelope.clean.loadFactorMax = 6.0;
    const VehicleProfile p = mergeProfile(base, over);
    CHECK(p.envelope.clean.loadFactorMax == 6.0);
    CHECK(p.envelope.header.provenance == Provenance::User);
    CHECK(p.identity.aircraftClass == AircraftClass::Fighter); // not in `over`: kept
}

TEST_CASE("every vehicle has its aircraft's profile; a spec's sections apply to that vehicle only", "[profile][world]") {
    session::WorldOptions o;
    o.name = "test-profile";
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    session::World w(o);
    session::VehicleSpec s;
    s.type = "jsbsim:c172x";
    s.initial.altitudeMslM = 1500.0;
    s.initial.airspeedTrueMs = 55.0;
    s.name = "stock";
    const auto stock = w.createVehicle(s);
    const VehicleProfile* p = w.profile(stock);
    REQUIRE(p != nullptr);
    CHECK(p->aircraft == "c172x");
    CHECK_FALSE(p->envelope.header.present());
    CHECK_FALSE(p->control.header.present());

    auto envelope = std::make_shared<VehicleProfile>();
    envelope->envelope.header = {1, Provenance::User};
    envelope->envelope.clean.loadFactorMax = 3.8;
    envelope->envelope.clean.loadFactorMin = -1.52;
    s.name = "limited";
    s.profile = envelope;
    const auto limited = w.createVehicle(s);
    CHECK(w.profile(limited)->envelope.clean.loadFactorMax == 3.8);
    CHECK(w.profile(limited)->aircraft == "c172x");
    CHECK(std::isnan(w.profile(stock)->envelope.clean.loadFactorMax)); // the aircraft's own is untouched

    s.name = "viper";
    s.type = "jsbsim:f16c";
    s.profile = nullptr;
    s.initial.altitudeMslM = 3000.0;
    s.initial.airspeedTrueMs = 160.0;
    const auto viper = w.createVehicle(s);
    REQUIRE(viper != 0);
    const VehicleProfile* f = w.profile(viper);
    CHECK(f->control.header.version == 1);
    CHECK(f->control.header.provenance == Provenance::Derived); // designed from its plant
    CHECK_FALSE(f->control.settings.empty());
    CHECK(profileValue(*f, "control/pid_attitude/schedule/tas_ms") == w.model(viper)->property("fsim/plant/tas_ms").get());
    CHECK(w.profile(999) == nullptr);
}

TEST_CASE("an adapter per family: what the family implies, and the envelope's ranges in the catalog", "[profile][adapter]") {
    CHECK(std::string(adapterFor(ControlFamily::Stock).family()) == "jsbsim.stock");
    CHECK(std::string(adapterFor(ControlFamily::Direct).family()) == "jsbsim.direct");
    CHECK(std::string(adapterFor(ControlFamily::FlyByWire).family()) == "jsbsim.fbw");

    // a fly-by-wire aircraft that did not say what its stick means
    VehicleProfile fbw;
    fbw.identity.family = ControlFamily::FlyByWire;
    adapterFor(ControlFamily::FlyByWire).complete(fbw);
    CHECK(fbw.effectors.pitch == PitchControl::LoadFactor);
    CHECK(fbw.effectors.roll == RollControl::RollRate);
    CHECK(fbw.effectors.neutral == NeutralStick::PathHold);
    CHECK(fbw.effectors.header.provenance == Provenance::Derived);
    // one that did keeps its word
    VehicleProfile said;
    said.effectors.header = {1, Provenance::Hangar};
    adapterFor(ControlFamily::FlyByWire).complete(said);
    CHECK(said.effectors.pitch == PitchControl::Surface);

    // the envelope narrows what a command may ask
    VehicleProfile p;
    p.envelope.header = {1, Provenance::Hangar};
    p.envelope.clean.loadFactorMin = -3.0;
    p.envelope.clean.loadFactorMax = 9.0;
    p.envelope.clean.bankMaxRad = 1.0;
    p.envelope.clean.rollRateMaxRadS = 5.0;
    const CapabilityCatalog catalog(p, adapterFor(ControlFamily::FlyByWire));
    const CapabilityCatalog wide;
    auto range = [](const CapabilityCatalog& c, const char* capability, const char* parameter) {
        for (const auto& q : c.descriptor(static_cast<std::size_t>(c.find(capability))).parameters)
            if (q.name == parameter) return std::make_pair(q.min, q.max);
        return std::make_pair(0.0, 0.0);
    };
    CHECK(range(catalog, "fsim.flight.acceleration", "load_factor_g") == std::make_pair(-3.0, 9.0));
    CHECK(range(catalog, "fsim.flight.acceleration", "roll_rate_rad_s") == std::make_pair(-5.0, 5.0));
    CHECK(range(catalog, "fsim.flight.attitude", "roll_rad") == std::make_pair(-1.0, 1.0));
    CHECK(range(catalog, "fsim.flight.attitude", "max_bank_rad") == std::make_pair(0.0, 1.0));
    CHECK(std::isinf(range(wide, "fsim.flight.acceleration", "load_factor_g").second));
}

TEST_CASE("a vehicle flies through its family's adapter and is offered its envelope's ranges", "[profile][adapter][world]") {
    session::WorldOptions o;
    o.name = "test-adapter";
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    session::World w(o);
    session::VehicleSpec s;
    s.type = "jsbsim:c172x";
    s.initial.altitudeMslM = 1500.0;
    s.initial.airspeedTrueMs = 55.0;
    s.name = "plain";
    const auto plain = w.createVehicle(s);
    auto own = std::make_shared<VehicleProfile>();
    own->identity.header = {1, Provenance::User};
    own->identity.family = ControlFamily::Direct;
    own->envelope.header = {1, Provenance::User};
    own->envelope.clean.loadFactorMin = -1.52;
    own->envelope.clean.loadFactorMax = 3.8; // normal category
    s.name = "limited";
    s.profile = own;
    const auto limited = w.createVehicle(s);
    CHECK(std::string(w.controls(plain)->adapter().family()) == "jsbsim.stock");
    CHECK(std::string(w.controls(limited)->adapter().family()) == "jsbsim.direct");
    AccelerationCommand pull{6.0, 0.0, kHold, 0.7};
    const CommandResult r = w.submit(limited, pull);
    REQUIRE(r.accepted());
    CHECK((r.flags & kClamped) != 0);
    CHECK(std::get<AccelerationCommand>(*w.controls(limited)->activeCommand()).loadFactorG == 3.8);
    const CommandResult free = w.submit(plain, pull);
    CHECK((free.flags & kClamped) == 0); // no envelope: the loops' own range
}
