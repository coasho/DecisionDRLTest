// The applicability section (docs/flight-autonomy.md, 5.2): an aircraft's
// physical characteristics as its design declares them, each with its public
// source, read from the aircraft file; every design's declarations checked
// against its model.
#include "control/Profile.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

/// An aircraft file's fsim/ properties, as FlightModel::properties returns them.
struct Properties {
    std::map<std::string, double> values; // full paths, e.g. "fsim/applicability/carrier"
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

constexpr Characteristic kAll[] = {
    Characteristic::VerticalFlight, Characteristic::GroundContact, Characteristic::Carrier,     Characteristic::RetractableGear,
    Characteristic::Flaps,          Characteristic::DragDevices,   Characteristic::ReleasableStores, Characteristic::Aerobatic,
};

const std::string& sourceOf(const ApplicabilitySection& a, Characteristic c) { return a.sources[static_cast<std::size_t>(c)]; }

session::World makeWorld() {
    session::WorldOptions o;
    o.name = "test-applicability";
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    return session::World(o);
}

session::VehicleSpec parked(const std::string& aircraft) {
    session::VehicleSpec s;
    s.type = "jsbsim:" + aircraft;
    s.name = aircraft;
    s.initial.onGround = true;
    return s;
}

} // namespace

TEST_CASE("the applicability section: flags and codes as the file writes them; not declared reads NaN", "[profile][applicability]") {
    Properties props;
    props.values = {
        {"fsim/applicability/version", 1},         {"fsim/applicability/provenance", 1},
        {"fsim/applicability/vertical_flight", 0}, {"fsim/applicability/ground_contact", 2},
        {"fsim/applicability/carrier", 3},         {"fsim/applicability/flaps", 1},
        {"fsim/applicability/drag_devices", 1},    {"fsim/applicability/releasable_stores", 4},
        {"fsim/applicability/aerobatic", 0},
    };
    std::vector<std::string> warnings;
    const VehicleProfile p = readProfile("test", props.source(), warnings);
    CHECK(warnings.empty());
    const ApplicabilitySection& a = p.applicability;
    REQUIRE(sectionHeader(p, "applicability") == &a.header);
    CHECK(a.header.version == 1);
    CHECK(a.header.provenance == Provenance::Hangar);
    CHECK(a.verticalFlight == Declared::No);
    CHECK(a.groundContact == GroundContact::Skids);
    CHECK(a.carrier == CarrierOperations::Deck);
    CHECK(a.retractableGear == Declared::Unknown); // not in the file
    CHECK(a.flaps == Declared::Yes);
    CHECK(a.dragDevices == DragDevices::None);
    CHECK(a.releasableStores == ReleasableStores::Dispensers);
    CHECK(a.aerobatic == Declared::No);
    CHECK(profileValue(p, "applicability/vertical_flight") == 0.0);
    CHECK(profileValue(p, "applicability/flaps") == 1.0);
    CHECK(profileValue(p, "applicability/ground_contact") == 2.0);
    CHECK(profileValue(p, "applicability/releasable_stores") == 4.0);
    CHECK(std::isnan(profileValue(p, "applicability/retractable_gear")));
    CHECK(profileValue(p, "applicability/version") == 1.0);
    for (const auto c : kAll) CHECK_FALSE(declares(a, c)); // values without their sources
    CHECK(std::string(characteristicName(Characteristic::DragDevices)) == "drag_devices");
    CHECK(std::string(characteristicName(static_cast<Characteristic>(kCharacteristicCount))).empty());

    SECTION("codes start at 1; flags are 0 or 1") {
        props.values["fsim/applicability/ground_contact"] = 0;
        props.values["fsim/applicability/carrier"] = 4;
        props.values["fsim/applicability/aerobatic"] = 2;
        warnings.clear();
        const VehicleProfile q = readProfile("test", props.source(), warnings);
        CHECK(mentions(warnings, "fsim/applicability/ground_contact = 0 is out of range"));
        CHECK(mentions(warnings, "fsim/applicability/carrier = 4 is out of range"));
        CHECK(mentions(warnings, "fsim/applicability/aerobatic = 2 is out of range"));
        CHECK(q.applicability.groundContact == GroundContact::Unknown);
        CHECK(q.applicability.carrier == CarrierOperations::Unknown);
        CHECK(q.applicability.aerobatic == Declared::Unknown);
        CHECK(std::isnan(profileValue(q, "applicability/carrier")));
    }
}

TEST_CASE("the sources come from the aircraft file's header; a characteristic without one is not declared", "[profile][applicability]") {
    const auto file = std::filesystem::temp_directory_path() / "fsim-test-applicability-sources.xml";
    {
        std::ofstream out(file, std::ios::binary);
        out << "<?xml version=\"1.0\"?>\n"
               "<fdm_config name=\"test\">\n"
               "  <fileheader>\n"
               "    <reference refID=\"Stevens\" author=\"Stevens, Lewis\" title=\"Aircraft Control and Simulation\" date=\"1992\"/>\n"
               "    <reference refID=\"fsim/applicability/flaps\" author=\"the design\" title=\"Jane's: &quot;flaperons&quot; &amp; "
               "flaps &#233;\" date=\"n/a\"/>\n"
               "    <reference author='the design' refID='fsim/applicability/carrier'\n"
               "               title='a land-based type (no \"hook\") &gt; a > b' date='n/a'/>\n"
               "    <reference refID=\"fsim/applicability/aerobatic\" author=\"the design\" title=\"+9 g\" date=\"n/a\"/>\n"
               "    <reference refID=\"fsim/applicability/wings\" author=\"the design\" title=\"not a characteristic\" date=\"n/a\"/>\n"
               "  </fileheader>\n"
               "  <reference refID=\"fsim/applicability/vertical_flight\" title=\"after the header: not read\"/>\n"
               "</fdm_config>\n";
    }
    ApplicabilitySection a;
    a.flaps = Declared::Yes;
    a.carrier = CarrierOperations::None;
    a.verticalFlight = Declared::No; // its source is not in the header
    readApplicabilitySources(file, a);
    CHECK(sourceOf(a, Characteristic::Flaps) == "Jane's: \"flaperons\" & flaps \xC3\xA9");
    CHECK(sourceOf(a, Characteristic::Carrier) == "a land-based type (no \"hook\") > a > b");
    CHECK(sourceOf(a, Characteristic::Aerobatic) == "+9 g");
    CHECK(sourceOf(a, Characteristic::VerticalFlight).empty());

    std::vector<std::string> warnings;
    requireSources("test", a, warnings);
    CHECK(declares(a, Characteristic::Flaps));
    CHECK(declares(a, Characteristic::Carrier));
    CHECK(a.verticalFlight == Declared::Unknown); // no evidence, no exception
    CHECK(mentions(warnings, "test: fsim/applicability/vertical_flight is declared without a source"));
    CHECK(sourceOf(a, Characteristic::Aerobatic).empty()); // a source without a value
    CHECK(mentions(warnings, "test: fsim/applicability/aerobatic has a source but no value"));
    CHECK(warnings.size() == 2);

    ApplicabilitySection none;
    readApplicabilitySources(file.parent_path() / "fsim-test-no-such-aircraft.xml", none); // nothing to read, nothing found
    for (const auto& s : none.sources) CHECK(s.empty());
    std::filesystem::remove(file);
}

TEST_CASE("a trainer's own declarations need their sources too; a stock aircraft declares nothing", "[profile][applicability][world]") {
    auto w = makeWorld();
    const auto stock = w.createVehicle(parked("c172x"));
    REQUIRE(stock != 0);
    const ApplicabilitySection& a = w.profile(stock)->applicability;
    CHECK_FALSE(a.header.present());
    for (const auto c : kAll) CHECK_FALSE(declares(a, c)); // everything stays applicable

    auto own = std::make_shared<VehicleProfile>();
    own->applicability.header = {1, Provenance::User};
    own->applicability.verticalFlight = Declared::Yes; // without a source
    own->applicability.flaps = Declared::No;
    own->applicability.sources[static_cast<std::size_t>(Characteristic::Flaps)] = "the trainer's own airframe";
    auto s = parked("c172x");
    s.name = "own";
    s.profile = own;
    const auto v = w.createVehicle(s);
    REQUIRE(v != 0);
    const ApplicabilitySection& b = w.profile(v)->applicability;
    CHECK(b.verticalFlight == Declared::Unknown);
    CHECK(declares(b, Characteristic::Flaps));
    CHECK(sourceOf(b, Characteristic::Flaps) == "the trainer's own airframe");
}

TEST_CASE("every design declares its characteristics with their sources, and its model agrees", "[applicability][world]") {
    // Not declared yet (docs/flight-autonomy.md, 5.2 and 5.3): what they govern
    // stays applicable until a public source decides them.
    const std::map<std::string, std::set<Characteristic>> open = {
        {"gripen", {Characteristic::Flaps}},     {"rafale", {Characteristic::Flaps}},     {"typhoon", {Characteristic::Flaps}},
        {"h6k", {Characteristic::DragDevices}},  {"j20a", {Characteristic::DragDevices}}, {"rq4b", {Characteristic::DragDevices}},
        {"su57", {Characteristic::DragDevices}},
    };
    auto w = makeWorld();
    std::size_t designs = 0;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(FSIM_TEST_AIRCRAFT_DIR, ec)) {
        const std::string name = entry.path().filename().string();
        if (!entry.is_directory(ec) || !std::filesystem::is_regular_file(entry.path() / (name + ".xml"), ec)) continue;
        ++designs;
        INFO(name);
        const auto v = w.createVehicle(parked(name));
        REQUIRE(v != 0);
        const VehicleProfile& p = *w.profile(v);
        const ApplicabilitySection& a = p.applicability;
        CHECK(a.header.version == ApplicabilitySection::kVersion);
        CHECK(a.header.provenance == Provenance::Hangar);
        const auto pending = open.find(name);
        for (const auto c : kAll) {
            INFO(characteristicName(c));
            CHECK(declares(a, c) == !(pending != open.end() && pending->second.count(c))); // with its source, or still open
        }
        const bool rotor = isRotorcraft(p.identity.family);
        CHECK((a.verticalFlight == Declared::Yes) == rotor); // R1: hovering is the rotorcraft's
        // R2: wheels have brakes; skids are a helicopter's, legs a multirotor's
        CHECK((a.groundContact == GroundContact::Wheels) == p.effectors.wheelBrakes);
        if (a.groundContact == GroundContact::Skids) CHECK(p.identity.family == ControlFamily::Helicopter);
        if (a.groundContact == GroundContact::Legs) CHECK(p.identity.family == ControlFamily::Multirotor);
        // R3 to R5: a catapult launch and an arrested landing are a wing's, a deck's vertical operations a rotorcraft's
        if (a.carrier == CarrierOperations::CatapultArrested) CHECK_FALSE(rotor);
        if (a.carrier == CarrierOperations::Deck) CHECK(rotor);
        // R6, R7: the model's effectors (a feature the type has is never declared absent to match the model)
        if (a.retractableGear != Declared::Unknown) CHECK((a.retractableGear == Declared::Yes) == p.effectors.retractableGear);
        if (a.flaps != Declared::Unknown) CHECK((a.flaps == Declared::Yes) == p.effectors.flaps);
        // R10: a wing cleared for aerobatics, built for their load factor
        if (a.aerobatic == Declared::Yes) {
            CHECK_FALSE(rotor);
            CHECK(p.envelope.clean.loadFactorMax >= 6.0);
        }
    }
    CHECK(designs >= 35);
}
