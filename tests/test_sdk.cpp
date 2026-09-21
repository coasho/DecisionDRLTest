// The public C++ SDK end to end (linked against fsim.dll like a trainer):
// a recorded world read back as data.
#include <fsim/Recording.h>
#include <fsim/World.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>

using namespace fsim;

TEST_CASE("a recording written by a World reads back through fsim::Recording", "[sdk][recording]") {
    const auto path = std::filesystem::temp_directory_path() / "fsim-sdk-test.fsrec";
    {
        WorldOptions o;
        o.name = "sdk-record";
        o.publish = false;
        o.workers = 1;
        o.pinWorkers = false;
        o.recordPath = path.string();
        World world(o);
        VehicleSpec s;
        s.name = "one";
        s.initial.altitudeMslM = 1500.0;
        s.initial.airspeedTrueMs = 60.0;
        Vehicle v = world.createVehicle(s);
        control::AttitudeCommand a;
        a.pitchRad = 0.05;
        v.command(a);
        world.step(20);
        VehicleSpec t = s;
        t.name = "two";
        t.initial.longitudeDeg += 0.01;
        world.createVehicle(t);
        world.step(10);
        v.remove();
        world.step(5);
    }
    REQUIRE_THROWS_AS(Recording::load(path.string() + ".missing"), Error);
    const Recording rec = Recording::load(path);
    REQUIRE(rec.worldName() == "sdk-record");
    REQUIRE(rec.frames().size() == 35);
    REQUIRE_THAT(rec.dt(), Catch::Matchers::WithinAbs(1.0 / 120.0, 1e-12));
    REQUIRE(rec.frameSkip() == 4);
    REQUIRE_THAT(rec.duration(), Catch::Matchers::WithinAbs(34.0 * 4.0 / 120.0, 1e-9));
    // Frame 0 carries the creation and the attitude command (two events on slot 0); frame 20 the second vehicle.
    REQUIRE(rec.frames()[0].events.size() == 2);
    REQUIRE(rec.frames()[0].events[0].name == "one");
    REQUIRE(rec.frames()[0].events[1].controlLevel == static_cast<std::uint8_t>(control::Level::Attitude));
    REQUIRE(rec.frames()[0].samples.size() == 1);
    REQUIRE(rec.frames()[20].events.size() == 1);
    REQUIRE(rec.frames()[20].events[0].name == "two");
    REQUIRE(rec.frames()[20].samples.size() == 2);
    REQUIRE(rec.frames()[30].events.size() == 1);
    REQUIRE_FALSE(rec.frames()[30].events[0].alive);
    REQUIRE(rec.frames()[30].samples.size() == 1);
    // A track follows one slot across the frames it lived in.
    const auto track = rec.track(0);
    REQUIRE(track.states.size() == 30);
    REQUIRE(track.simTime.front() < track.simTime.back());
    REQUIRE(track.states.back().altitudeMslM > 1400.0);
    REQUIRE(track.inputs.size() == 30);
    REQUIRE(rec.track(1).states.size() == 15);
    REQUIRE(rec.track(7).states.empty());
    std::filesystem::remove(path);
}
