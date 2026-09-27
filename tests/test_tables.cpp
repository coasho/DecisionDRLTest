// The performance tables (docs/flight-autonomy.md, SUB-02; ADR-29 FA-3a): an
// aircraft hangar flew carries them in its profile, read as written, looked
// up linear in altitude and weight and along each condition's level speeds;
// an aircraft hangar has not flown them for has none. That the tables agree
// with the flight tests within 5 % is hangar's performance stage's check.
#include "mode_flights.h"

#include "control/Profile.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

const VehicleProfile& profileOf(session::World& w, const char* type, std::uint32_t& id) {
    id = wing(w, type, 3000.0, 150.0);
    const VehicleProfile* p = w.profile(id);
    REQUIRE(p != nullptr);
    return *p;
}

std::size_t cond(const TablesSection& t, std::size_t h, std::size_t w) { return h * t.weightKg.size() + w; }

} // namespace

TEST_CASE("tables: an aircraft hangar flew carries its performance tables, read and looked up as written", "[performance]") {
    session::World w(options("tables-read"));
    for (const char* type : {"c172", "b52h", "f16c"}) {
        INFO(type);
        std::uint32_t id = 0;
        const VehicleProfile& p = profileOf(w, type, id);
        const TablesSection& t = p.tables;
        REQUIRE_FALSE(t.empty());
        CHECK((t.header.version == 1 && t.header.provenance == Provenance::Hangar));
        CHECK((t.altitudeM.size() >= 7 && t.weightKg.size() >= 3 && t.speedFraction.size() == 16)); // (and the weight it spawns at: a B-52H's 40 %)
        CHECK(t.fuelCapacityKg > 0.0);
        CHECK(profileValue(p, "tables/altitude_m/h1") == t.altitudeM[1]);
        CHECK(profileValue(p, "tables/max_tas_ms/h0/w2") == t.maxTasMs[cond(t, 0, 2)]);
        const double cell = t.fuelKgS[cond(t, 0, 2) * t.speedFraction.size() + 5];
        CHECK((profileValue(p, "tables/fuel_kg_s/h0/w2/v5") == cell || (std::isnan(cell) && std::isnan(profileValue(p, "tables/fuel_kg_s/h0/w2/v5")))));
        CHECK(std::isnan(profileValue(p, "tables/max_tas_ms/h9/w0")));
        for (std::size_t h = 0; h < t.altitudeM.size(); ++h)
            for (std::size_t wt = 0; wt < t.weightKg.size(); ++wt) {
                const std::size_t c = cond(t, h, wt);
                if (!std::isfinite(t.maxTasMs[c])) continue; // (above its ceiling at that weight: not flown)
                INFO("h " << t.altitudeM[h] << " w " << t.weightKg[wt]);
                // at a condition flown, its own values
                const TablesAt at = tablesAt(t, t.altitudeM[h], t.weightKg[wt]);
                CHECK(at.maxTasMs == t.maxTasMs[c]);
                CHECK((at.bestRangeTasMs == t.bestRangeTasMs[c] || (std::isnan(at.bestRangeTasMs) && std::isnan(t.bestRangeTasMs[c]))));
                // what a level acceleration reaches: the top, or short of it past a drag rise it cannot pass
                if (std::isfinite(t.reachTasMs[c])) CHECK(t.reachTasMs[c] <= t.maxTasMs[c] * (1.0 + 1e-9));
                // the speeds in the order an aircraft has them
                // (each within the band of level speeds it was flown along, where its level points held)
                if (std::isfinite(t.bestRangeTasMs[c])) {
                    CHECK(t.minTasMs[c] <= t.bestRangeTasMs[c]);
                    CHECK(t.bestEnduranceTasMs[c] <= t.bestRangeTasMs[c] + 1e-9);
                    CHECK(t.bestRangeTasMs[c] < t.maxTasMs[c]);
                }
                CHECK(t.maxClimbMs[c] > 0.0);
                for (std::size_t k = 0; k < t.speedFraction.size(); ++k) {
                    const std::size_t i = c * t.speedFraction.size() + k;
                    if (!std::isfinite(t.fuelKgS[i])) continue; // (a level point that did not hold its speed and height)
                    CHECK(t.fuelKgS[i] > 0.0);
                    if (std::isfinite(t.psFullMs[i]) && std::isfinite(t.psIdleMs[i])) CHECK(t.psFullMs[i] > t.psIdleMs[i]);
                    // at a level point, its own values (the speed fraction it was flown at)
                    const double v = t.minTasMs[c] + t.speedFraction[k] * (0.97 * t.maxTasMs[c] - t.minTasMs[c]);
                    const TablesAtSpeed s = tablesAt(t, t.altitudeM[h], t.weightKg[wt], v);
                    CHECK(std::abs(s.fuelKgS - t.fuelKgS[i]) <= 1e-9 * t.fuelKgS[i]);
                }
            }
        // between conditions: linear in altitude and in weight
        const double hMid = 0.5 * (t.altitudeM[1] + t.altitudeM[2]), wMid = 0.5 * (t.weightKg[1] + t.weightKg[2]);
        const TablesAt mid = tablesAt(t, hMid, wMid);
        const double expected = 0.25 * (t.maxTasMs[cond(t, 1, 1)] + t.maxTasMs[cond(t, 1, 2)] + t.maxTasMs[cond(t, 2, 1)] + t.maxTasMs[cond(t, 2, 2)]);
        CHECK(std::abs(mid.maxTasMs - expected) < 1e-9 * expected);
        // below the lowest altitude: its values; above the highest: none
        CHECK(tablesAt(t, 0.0, t.weightKg.back()).maxTasMs == t.maxTasMs[cond(t, 0, t.weightKg.size() - 1)]);
        CHECK(std::isnan(tablesAt(t, t.altitudeM.back() + 500.0, t.weightKg.back()).maxTasMs));
        // lighter, higher: its ceiling rises as the fuel burns
        const double full = tablesCeilingM(t, t.weightKg.back()), light = tablesCeilingM(t, t.weightKg.front());
        CHECK(full > 0.0);
        CHECK(light > full);
        CHECK(std::isnan(tablesAt(t, 1000.0, t.weightKg.back() - t.fuelCapacityKg - 100.0).maxTasMs)); // (lighter than the tanks empty)
    }
}

TEST_CASE("tables: every fixed wing hangar ships carries them; the rotorcraft's come with their fuel (FA-3b)", "[performance]") {
    session::World w(options("tables-fleet"));
    std::error_code ec;
    int wings = 0;
    for (const auto& entry : std::filesystem::directory_iterator(FSIM_TEST_AIRCRAFT_DIR, ec)) {
        const std::string name = entry.path().filename().string();
        if (!entry.is_directory(ec) || !std::filesystem::is_regular_file(entry.path() / (name + ".xml"), ec)) continue;
        INFO(name);
        session::VehicleSpec s;
        s.name = name;
        s.type = "jsbsim:" + name;
        s.initial.latitudeDeg += 0.01 * wings;
        s.initial.altitudeMslM = 3000.0;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        const VehicleProfile& p = *w.profile(id);
        if (isRotorcraft(p.identity.family)) {
            CHECK(p.tables.empty());
            continue;
        }
        ++wings;
        REQUIRE_FALSE(p.tables.empty());
        // flown at its lowest altitude, loaded, at least: the conditions the flight tests are checked at
        CHECK(std::isfinite(p.tables.maxTasMs[p.tables.weightKg.size() - 1]));
        CHECK(std::isfinite(tablesCeilingM(p.tables, p.tables.weightKg.back())));
        REQUIRE(w.removeVehicle(id));
    }
    CHECK(wings == 31);
}

TEST_CASE("tables: an aircraft without them - a stock JSBSim aircraft - answers none", "[performance]") {
    session::World w(options("tables-none"));
    std::uint32_t id = 0;
    const VehicleProfile& p = profileOf(w, "c172x", id);
    CHECK(p.tables.empty());
    CHECK(std::isnan(tablesAt(p.tables, 1000.0, 1000.0).maxTasMs));
    CHECK(std::isnan(tablesAt(p.tables, 1000.0, 1000.0, 50.0).fuelKgS));
    CHECK(std::isnan(tablesCeilingM(p.tables, 1000.0)));
}

TEST_CASE("tables: the service ceiling - an altitude nothing held level at climbs nothing", "[performance]") {
    TablesSection t;
    t.altitudeM = {100.0, 3000.0, 6000.0};
    t.weightKg = {1000.0};
    t.speedFraction = {0.0, 1.0};
    t.maxClimbMs = {5.0, 1.0, 0.3};
    // between the altitudes where the best climb falls through 0.508 m/s
    CHECK(std::abs(tablesCeilingM(t, 1000.0) - (3000.0 + (1.0 - 0.508) / (1.0 - 0.3) * 3000.0)) < 1e-9);
    // a row not flown is a height it cannot fly level at: below it, never the rows beneath extended past it
    t.maxClimbMs = {5.0, kUnknown, 0.3};
    CHECK(std::abs(tablesCeilingM(t, 1000.0) - (100.0 + (5.0 - 0.508) / 5.0 * 2900.0)) < 1e-9);
    // climbing at the highest: the highest two's line extended
    t.maxClimbMs = {5.0, 3.0, 2.0};
    CHECK(std::abs(tablesCeilingM(t, 1000.0) - (6000.0 + (2.0 - 0.508) / 1.0 * 3000.0)) < 1e-9);
    // climbing at none
    t.maxClimbMs = {0.2, kUnknown, 0.1};
    CHECK(std::isnan(tablesCeilingM(t, 1000.0)));
}

TEST_CASE("tables: what an aircraft file writes wrongly is warned of and left out", "[performance]") {
    std::vector<std::pair<std::string, double>> values = {
        {"altitude_m/h0", 100.0},        {"altitude_m/h1", 2000.0},       {"weight_kg/w0", 900.0},   {"weight_kg/w1", 1100.0},
        {"speed_fraction/v0", 0.0},      {"speed_fraction/v1", 1.0},      {"fuel_capacity_kg", 150.0}, {"max_tas_ms/h0/w0", 60.0},
        {"max_tas_ms/h1/w1", 55.0},      {"max_tas_ms/h7/w0", 1.0},       {"warp_ms/h0/w0", 1.0},    {"fuel_kg_s/h0/w0/v1", 0.01},
        {"fuel_kg_s/h0/w0/v9", 1.0},
    };
    TablesSection t;
    std::vector<std::string> warnings;
    readTables(values, t, "test: fsim/tables", warnings);
    CHECK(warnings.size() == 3); // (a cell beyond the altitudes, a table it does not know, a point beyond the speeds)
    REQUIRE((t.altitudeM.size() == 2 && t.weightKg.size() == 2 && t.speedFraction.size() == 2));
    CHECK(t.maxTasMs[0] == 60.0);
    CHECK(t.maxTasMs[3] == 55.0);
    CHECK(std::isnan(t.maxTasMs[1]));
    CHECK(t.fuelKgS[1] == 0.01);
    // an axis out of order: nothing read
    values.emplace_back("altitude_m/h2", 1000.0);
    TablesSection bad;
    warnings.clear();
    readTables(values, bad, "test: fsim/tables", warnings);
    CHECK(bad.empty());
    CHECK(warnings.size() == 1);
    // nor one that repeats a point (three weights all the same: an aircraft without fuel flies one)
    values.pop_back();
    values.emplace_back("weight_kg/w2", 1100.0);
    TablesSection repeated;
    warnings.clear();
    readTables(values, repeated, "test: fsim/tables", warnings);
    CHECK(repeated.empty());
    CHECK(warnings.size() == 1);
}
