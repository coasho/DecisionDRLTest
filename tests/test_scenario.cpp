// Scenario files: JSON parsing, error reporting, dump round trip and
// application to a world (design 9.13).
#include "session/Scenario.h"

#include "core/Json.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <string>

using namespace fsim;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

TEST_CASE("json: parse, access, dump", "[json]") {
    const auto j = core::Json::parse(R"({"a": 1.5, "b": [true, null, "x\u00e9\n"], "c": {"d": -2e3}, // comment
        "e": false, /* block */ })", "t");
    REQUIRE(j.number("a", 0) == 1.5);
    REQUIRE(j.child("b").asArray().size() == 3);
    REQUIRE(j.child("b").asArray()[0].asBool());
    REQUIRE(j.child("b").asArray()[1].isNull());
    REQUIRE(j.child("b").asArray()[2].asString() == "x\xc3\xa9\n");
    REQUIRE(j.child("c").number("d", 0) == -2000.0);
    REQUIRE(j.number("missing", 7) == 7);
    REQUIRE_FALSE(j.boolean("e", true));
    REQUIRE_THROWS_WITH(j.number("e", 0), ContainsSubstring("must be a number"));
    REQUIRE(core::Json::parse(j.dump()).dump() == j.dump());
    REQUIRE_THROWS_WITH(core::Json::parse("{\"a\": }", "f.json"), ContainsSubstring("f.json:1:7"));
    REQUIRE_THROWS_WITH(core::Json::parse("[1, 2] x"), ContainsSubstring("trailing"));
}

namespace {

const char* kScenario = R"({
  "world": { "name": "test-scenario", "seed": 3, "capacity": 16, "publish": false, "workers": 1, "pin_workers": false },
  "environment": { "utc": "2026-06-21T09:30:00Z", "wind": { "direction_deg": 270, "speed_ms": 5 }, "weather": { "cloud_cover": 0.4 } },
  "effects": [ { "id": "gaussian_sensor_noise", "position_sigma_m": 2 } ],
  "vehicles": [
    { "name": "pair", "count": 2, "spacing_m": 100,
      "initial": { "lat_deg": 37.62, "lon_deg": -122.40, "alt_msl_m": 1500, "heading_deg": 90, "airspeed_ms": 60 },
      "command": { "level": "attitude", "roll_deg": 10, "pitch_deg": 2, "airspeed_ms": 60 } },
    { "name": "chaser",
      "initial": { "lat_deg": 37.63, "lon_deg": -122.42, "alt_msl_m": 1500, "heading_deg": 90, "airspeed_ms": 60 },
      "command": { "level": "behavior", "id": "pursuit", "target": "leader", "params": { "range_m": 150 } },
      "effects": [ { "id": "sensor_latency", "delay_steps": 4 } ] },
    { "name": "leader",
      "initial": { "lat_deg": 37.63, "lon_deg": -122.40, "alt_msl_m": 1500, "heading_deg": 90, "airspeed_ms": 60 },
      "command": { "level": "velocity", "airspeed_ms": 55, "heading_deg": 120 } }
  ]
})";

} // namespace

TEST_CASE("scenario: parse and dump round trip", "[scenario]") {
    const Scenario sc = parseScenario(kScenario, "s.json");
    REQUIRE(sc.world.name == "test-scenario");
    REQUIRE(sc.world.seed == 3);
    REQUIRE(sc.world.capacity == 16);
    REQUIRE_FALSE(sc.world.publish);
    REQUIRE(sc.hasEnvironment);
    REQUIRE(sc.environment.windDirectionDeg == 270.0);
    REQUIRE(sc.environment.cloudCover == 0.4);
    REQUIRE_THAT(sc.environment.epochUtcSeconds, WithinAbs(1782034200.0, 0.5)); // 2026-06-21T09:30:00Z
    REQUIRE(sc.effects.size() == 1);
    REQUIRE(sc.effects[0].params[0].first == "position_sigma_m");
    REQUIRE(sc.vehicles.size() == 3);
    REQUIRE(sc.vehicles[0].count == 2);
    REQUIRE(sc.vehicles[0].command.has_value());
    const auto& att = std::get<control::AttitudeCommand>(*sc.vehicles[0].command);
    REQUIRE_THAT(att.rollRad, WithinAbs(10.0 * 3.14159265358979 / 180.0, 1e-9));
    REQUIRE(std::isnan(att.throttle)); // absent = hold
    REQUIRE(sc.vehicles[1].commandTarget == "leader");
    REQUIRE(std::get<control::BehaviorCommand>(*sc.vehicles[1].command).param("range_m", 0) == 150.0);
    REQUIRE(sc.vehicles[1].effects.size() == 1);
    const auto& vel = std::get<control::VelocityCommand>(*sc.vehicles[2].command);
    REQUIRE(vel.airspeedMs == 55.0);

    // Dump -> parse gives the same scenario.
    const Scenario again = parseScenario(dumpScenario(sc), "dump");
    REQUIRE(dumpScenario(again) == dumpScenario(sc));
    REQUIRE(again.vehicles[1].commandTarget == "leader");
    REQUIRE(again.environment.windDirectionDeg == 270.0);
}

TEST_CASE("scenario: errors name the file and position", "[scenario]") {
    REQUIRE_THROWS_WITH(parseScenario("{ \"world\": { \"dt\": 0 } }", "bad.json"), ContainsSubstring("bad.json"));
    REQUIRE_THROWS_WITH(parseScenario("{ \"vehicles\": [ { \"type\": \"jsbsim:c172x\" } ] }", "v.json"), ContainsSubstring("without 'name'"));
    REQUIRE_THROWS_WITH(parseScenario("{ \"vehicles\": [ { \"name\": \"a\", \"command\": { \"level\": \"warp\" } } ] }", "v.json"),
                        ContainsSubstring("unknown command level 'warp'"));
    REQUIRE_THROWS_WITH(parseScenario("{ \"world\": { \"name\": 5 } }", "w.json"), ContainsSubstring("'name' must be a string"));
    REQUIRE_THROWS_WITH(parseScenario("{ \"world\": ", "trunc.json"), ContainsSubstring("trunc.json:1:"));
}

TEST_CASE("scenario: applied to a world", "[scenario][world]") {
    Scenario sc = parseScenario(kScenario, "s.json");
    sc.world.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    session::WorldOptions o;
    o.name = sc.world.name;
    o.jsbsimRoot = sc.world.jsbsimRoot;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = sc.world.seed;
    session::World w(o);
    const auto ids = session::applyScenario(w, sc);
    REQUIRE(ids.size() == 4);
    REQUIRE(w.find("pair-1") == ids[0]);
    REQUIRE(w.find("pair-2") == ids[1]);
    REQUIRE(w.find("chaser") == ids[2]);
    REQUIRE(w.find("leader") == ids[3]);
    // Second instance 100 m to the right of a heading of 090: due south.
    const auto* a = w.vehicleState(ids[0]);
    const auto* b = w.vehicleState(ids[1]);
    REQUIRE_THAT(b->longitudeRad, WithinAbs(a->longitudeRad, 1e-6));
    REQUIRE_THAT((a->latitudeRad - b->latitudeRad) * 6371000.0, WithinAbs(100.0, 2.0));
    REQUIRE(w.environment().windDirectionDeg == 270.0);
    REQUIRE_THAT(w.environment().epochUtcSeconds, WithinAbs(1782034200.0, 0.5));
    REQUIRE(w.controls(ids[0])->activeLevel() == control::Level::Attitude);
    REQUIRE(w.controls(ids[2])->activeLevel() == control::Level::Behavior);
    REQUIRE(w.controls(ids[3])->activeLevel() == control::Level::Velocity);
    w.step(30);
    REQUIRE_FALSE(w.vehicleState(ids[0])->diverged);
    REQUIRE(w.vehicleState(ids[0])->eulerRad[0] > 0.02); // rolling right as commanded

    // An unknown behaviour target is an error after the vehicles exist.
    Scenario bad = parseScenario(R"({ "vehicles": [ { "name": "x", "command": { "level": "behavior", "id": "pursuit", "target": "nobody" } } ] })", "bad.json");
    REQUIRE_THROWS_WITH(session::applyScenario(w, bad), ContainsSubstring("unknown target 'nobody'"));
    REQUIRE(w.vehicleCount() == 5);
}
