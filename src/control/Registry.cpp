#include "control/Registry.h"

#include <algorithm>

namespace fsim::control {

ControllerRegistry& ControllerRegistry::instance() {
    static ControllerRegistry registry;
    return registry;
}

ControllerRegistry::ControllerRegistry() { registerBuiltinControllers(*this); }

void ControllerRegistry::add(std::string id, Level level, Factory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++revision_;
    for (auto& e : entries_)
        if (e.first == id) {
            e.second = Entry{level, std::move(factory), {}};
            return;
        }
    entries_.emplace_back(std::move(id), Entry{level, std::move(factory), {}});
}

void ControllerRegistry::addBehavior(std::string id, std::function<std::unique_ptr<Behavior>()> factory) {
    addBehavior(std::move(id), std::move(factory), BehaviorTraits{});
}

void ControllerRegistry::addBehavior(std::string id, std::function<std::unique_ptr<Behavior>()> factory, BehaviorTraits traits) {
    add(id, Level::Behavior, [factory = std::move(factory)]() -> std::unique_ptr<Controller> { return factory(); });
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& e : entries_)
        if (e.first == id) e.second.traits = std::move(traits);
}

std::vector<std::pair<std::string, BehaviorTraits>> ControllerRegistry::behaviors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::pair<std::string, BehaviorTraits>> out;
    for (const auto& e : entries_)
        if (e.second.level == Level::Behavior) out.emplace_back(e.first, e.second.traits);
    return out;
}

std::uint64_t ControllerRegistry::revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return revision_;
}

std::unique_ptr<Controller> ControllerRegistry::create(std::string_view id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& e : entries_)
        if (e.first == id) return e.second.factory();
    return nullptr;
}

Level ControllerRegistry::levelOf(std::string_view id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& e : entries_)
        if (e.first == id) return e.second.level;
    return Level::Count;
}

std::vector<std::string> ControllerRegistry::ids(Level level) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    for (const auto& e : entries_)
        if (e.second.level == level) out.push_back(e.first);
    std::sort(out.begin(), out.end());
    return out;
}

const char* ControllerRegistry::defaultId(Level level) const noexcept {
    switch (level) {
    case Level::Actuator: return "actuator";
    case Level::Attitude: return "pid_attitude";
    case Level::Acceleration: return "pid_acceleration";
    case Level::Velocity: return "pid_velocity";
    case Level::Position: return "pid_position";
    default: return nullptr;
    }
}

} // namespace fsim::control
