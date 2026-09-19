#include "core/Context.h"
#include "core/Log.h"
#include "core/Module.h"
#include "core/Registry.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

struct Task {
    virtual ~Task() = default;
    virtual int id() const = 0;
};
struct TaskA : Task {
    int id() const override { return 1; }
};
struct TaskB : Task {
    explicit TaskB(int v) : v_(v) {}
    int id() const override { return v_; }
    int v_;
};

} // namespace

TEST_CASE("registry creates by id and rejects unknown ids without crashing", "[core]") {
    fsim::log::setLevel(fsim::log::Level::Off);
    fsim::Registry<Task, int> reg;
    reg.add("a", [](int) { return std::make_unique<TaskA>(); });
    reg.add("b", [](int v) { return std::make_unique<TaskB>(v); });

    REQUIRE(reg.contains("a"));
    REQUIRE_FALSE(reg.contains("zzz"));
    CHECK(reg.create("a", 0)->id() == 1);
    CHECK(reg.create("b", 42)->id() == 42);
    CHECK(reg.create("zzz", 0) == nullptr);
    CHECK(reg.ids().size() == 2);
    fsim::log::setLevel(fsim::log::Level::Info);
}

namespace {

struct Trace {
    std::vector<std::string> events;
};

struct RecordingModule : fsim::Module {
    RecordingModule(std::string n, Trace& t, bool failOnStart = false)
        : name_(std::move(n)), trace_(t), fail_(failOnStart) {}
    std::string_view name() const noexcept override { return name_; }
    void init(fsim::Context&) override { trace_.events.push_back("init " + name_); }
    void start(fsim::Context&) override {
        if (fail_) throw std::runtime_error("boom");
        trace_.events.push_back("start " + name_);
    }
    void stop(fsim::Context&) noexcept override { trace_.events.push_back("stop " + name_); }
    void shutdown(fsim::Context&) noexcept override { trace_.events.push_back("shutdown " + name_); }
    std::string name_;
    Trace& trace_;
    bool fail_;
};

} // namespace

TEST_CASE("module registry starts in order and shuts down in reverse", "[core]") {
    Trace t;
    fsim::Context ctx;
    fsim::ModuleRegistry reg;
    reg.add(std::make_unique<RecordingModule>("a", t));
    reg.add(std::make_unique<RecordingModule>("b", t));
    reg.startAll(ctx);
    reg.shutdownAll(ctx);
    const std::vector<std::string> expected{"init a", "init b", "start a", "start b", "stop b", "stop a", "shutdown b",
                                            "shutdown a"};
    CHECK(t.events == expected);
}

TEST_CASE("module registry cleans up when a start fails", "[core]") {
    fsim::log::setLevel(fsim::log::Level::Off);
    Trace t;
    fsim::Context ctx;
    fsim::ModuleRegistry reg;
    reg.add(std::make_unique<RecordingModule>("a", t));
    reg.add(std::make_unique<RecordingModule>("b", t, /*failOnStart=*/true));
    REQUIRE_THROWS(reg.startAll(ctx));
    const std::vector<std::string> expected{"init a", "init b", "start a", "stop a", "shutdown b", "shutdown a"};
    CHECK(t.events == expected);
    fsim::log::setLevel(fsim::log::Level::Info);
}
