// The public C++ SDK end to end (linked against fsim.dll like a trainer):
// a recorded world read back as data, and the environment extension points.
#include <fsim/Recording.h>
#include <fsim/VecEnv.h>
#include <fsim/VecEnvPlugins.h>
#include <fsim/World.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

// Trainer-side extension points (design 10.1 "SDK registration"): a task, an
// observation builder and an action mapper defined here - in the trainer, not
// the platform - registered by id and named in the options.
namespace {

/// Reward the altitude gained since the episode started. Keeps its working
/// values in TaskState::custom, which the observation builder below reads.
class ClimbTask final : public Task {
public:
    explicit ClimbTask(const TaskParams& p) : agentStep_(p.agentStepSeconds) {}

    std::string_view name() const noexcept override { return "climb"; }

    void reset(std::size_t, const VehicleState& initial, Rng&, TaskState& t) override {
        t.custom[0] = initial.altitudeMslM; // the altitude to beat
        t.custom[1] = 0.0;                  // agent steps this episode
        t.custom[2] = agentStep_;           // what TaskParams said the step was
    }

    void evaluate(std::size_t, const VehicleState& s, TaskState& t, double& reward, bool& terminated) override {
        t.custom[1] += 1.0;
        reward = (s.altitudeMslM - t.custom[0]) / 100.0;
        terminated = s.diverged;
        t.lastReward = reward;
    }

private:
    double agentStep_;
};

/// Three elements, two of them the task's own values.
class ClimbObservation final : public ObservationBuilder {
public:
    ClimbObservation() : names_{"alt_km", "episode_steps", "agent_step_s"} {}
    std::size_t size() const noexcept override { return names_.size(); }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    void build(const VehicleState& s, const TaskState& t, float* o) const override {
        o[0] = static_cast<float>(s.altitudeMslM / 1000.0);
        o[1] = static_cast<float>(t.custom[1]);
        o[2] = static_cast<float>(t.custom[2]);
    }

private:
    std::vector<std::string> names_;
};

/// One element: pitch. The built-in attitude loop holds the wings level and
/// flies it, so the agent commands nothing else.
class PitchAction final : public ActionMapper {
public:
    PitchAction() : names_{"pitch"} {}
    std::size_t size() const noexcept override { return 1; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    control::Level level() const noexcept override { return control::Level::Attitude; }
    control::Command map(const float* a) const override {
        control::AttitudeCommand c;
        c.rollRad = 0.0;
        c.pitchRad = static_cast<double>(a[0]) * 0.2;
        c.throttle = 1.0;
        return c;
    }

private:
    std::vector<std::string> names_;
};

bool has(const std::vector<std::string>& ids, const std::string& id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}

} // namespace

TEST_CASE("a trainer's task, observation and action are used by id", "[sdk][plugins]") {
    registerTask("climb", [](const TaskParams& p) { return std::make_unique<ClimbTask>(p); });
    registerObservation("climb_obs", [] { return std::make_unique<ClimbObservation>(); });
    registerAction("pitch", [] { return std::make_unique<PitchAction>(); });

    // The built-ins and the new ids come from the same registry.
    const auto tasks = taskIds();
    const auto observations = observationIds();
    auto actions = actionIds();
    REQUIRE(has(tasks, "altitude_heading_hold"));
    REQUIRE(has(tasks, "climb"));
    REQUIRE(has(observations, "state"));
    REQUIRE(has(observations, "climb_obs"));
    REQUIRE(has(actions, "surfaces"));
    REQUIRE(has(actions, "pitch"));

    // Registering an id again replaces it rather than adding a second entry.
    registerAction("pitch", [] { return std::make_unique<PitchAction>(); });
    actions = actionIds();
    REQUIRE(std::count(actions.begin(), actions.end(), "pitch") == 1);

    VecEnvOptions o;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.publish = false;
    o.workers = 1;
    o.numEnvs = 2;
    o.maxEpisodeSteps = 500; // no truncation during the runs below
    o.task = "climb";
    o.observation = "climb_obs";
    o.action = "pitch";

    // An id nobody registered is refused when the environment is built.
    VecEnvOptions bad = o;
    bad.task = "no_such_task";
    REQUIRE_THROWS_AS(VecEnv(bad), std::runtime_error);

    VecEnv env(o);
    REQUIRE(env.observationSize() == 3);
    REQUIRE(env.actionSize() == 1);
    REQUIRE(env.observationNames()[1] == "episode_steps");
    REQUIRE(env.actionNames()[0] == "pitch");

    // The spans point into the environment's own buffers, so read what this
    // run produced before starting the next one.
    const auto run = [&env](float pitch, double* altKm, double* steps) {
        auto r = env.reset(11);
        const std::vector<float> actionBuffer(env.numVehicles() * env.actionSize(), pitch);
        for (int i = 0; i < 60; ++i) r = env.step(actionBuffer);
        *altKm = static_cast<double>(r.observations[0]);
        *steps = static_cast<double>(r.observations[1]);
        return static_cast<double>(r.rewards[0]);
    };

    double upAltKm = 0.0, upSteps = 0.0, downAltKm = 0.0, downSteps = 0.0;
    const double upReward = run(1.0f, &upAltKm, &upSteps);
    const double downReward = run(-1.0f, &downAltKm, &downSteps);

    // The task ran once per step and saw the options' agent step.
    REQUIRE_THAT(upSteps, Catch::Matchers::WithinAbs(60.0, 1e-6));
    REQUIRE_THAT(downSteps, Catch::Matchers::WithinAbs(60.0, 1e-6));
    auto firstObs = env.reset(11);
    REQUIRE_THAT(static_cast<double>(firstObs.observations[2]),
                 Catch::Matchers::WithinRel(env.agentStepSeconds(), 1e-5));
    REQUIRE_THAT(static_cast<double>(firstObs.observations[1]), Catch::Matchers::WithinAbs(0.0, 1e-6));

    // The action mapper really commanded the attitude loop, and the reward is
    // the task's own: nose up ends higher and scores better than nose down.
    REQUIRE(upAltKm > downAltKm);
    REQUIRE(upReward > downReward);
}

TEST_CASE("capabilities and activities through the SDK: discover, submit, update, cancel", "[sdk][control]") {
    WorldOptions o;
    o.name = "sdk-capabilities";
    o.publish = false;
    o.workers = 1;
    o.pinWorkers = false;
    World world(o);
    VehicleSpec s;
    s.initial.altitudeMslM = 1500.0;
    s.initial.airspeedTrueMs = 55.0;
    Vehicle v = world.createVehicle(s);

    const auto caps = v.capabilities();
    REQUIRE(std::any_of(caps.begin(), caps.end(), [](const control::CapabilityDescriptor& c) { return c.id == "fsim.flight.velocity"; }));
    REQUIRE(v.capabilityStatus("fsim.guidance.hold").availability == control::Availability::Available);

    const control::CommandResult r = v.submit(control::VelocityCommand{55.0, 1.0, control::kHold, control::kHold});
    REQUIRE(r.accepted());
    REQUIRE(world.activity(r.activity)->state == control::ActivityState::Pending);
    world.step();
    REQUIRE(world.activity(r.activity)->state == control::ActivityState::Active);
    REQUIRE(world.update(r.activity, control::VelocityCommand{55.0, -1.0, control::kHold, control::kHold}).accepted());
    REQUIRE(world.cancel(r.activity).status == control::CommandStatus::Canceled);
    REQUIRE(world.activity(r.activity)->state == control::ActivityState::Canceled);
    REQUIRE(world.activity(r.activity)->reason == control::Reason::Requested);
    REQUIRE(std::string(control::reasonName(control::Reason::Requested)) == "requested");
    REQUIRE(v.activities().size() == 1);
    REQUIRE_FALSE(world.activity(control::activityId(v.id(), 42)).has_value());
}
