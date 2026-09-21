#include "env/Registry.h"

#include <algorithm>

namespace fsim::env {

namespace {

/// Replace the entry with this id, or append it. The caller holds the lock.
template <typename Factory>
void put(std::vector<std::pair<std::string, Factory>>& table, std::string id, Factory factory) {
    for (auto& entry : table)
        if (entry.first == id) {
            entry.second = std::move(factory);
            return;
        }
    table.emplace_back(std::move(id), std::move(factory));
}

/// The factory registered under this id, or null. The caller holds the lock.
template <typename Factory>
const Factory* find(const std::vector<std::pair<std::string, Factory>>& table, std::string_view id) {
    for (const auto& entry : table)
        if (entry.first == id) return &entry.second;
    return nullptr;
}

template <typename Factory>
std::vector<std::string> keys(const std::vector<std::pair<std::string, Factory>>& table) {
    std::vector<std::string> out;
    out.reserve(table.size());
    for (const auto& entry : table) out.push_back(entry.first);
    std::sort(out.begin(), out.end());
    return out;
}

} // namespace

PluginRegistry& PluginRegistry::instance() {
    static PluginRegistry registry;
    return registry;
}

PluginRegistry::PluginRegistry() {
    registerBuiltinTasks(*this);
    registerBuiltinObservations(*this);
    registerBuiltinActions(*this);
}

void PluginRegistry::addTask(std::string id, TaskFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    put(tasks_, std::move(id), std::move(factory));
}

void PluginRegistry::addObservation(std::string id, ObservationFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    put(observations_, std::move(id), std::move(factory));
}

void PluginRegistry::addAction(std::string id, ActionFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    put(actions_, std::move(id), std::move(factory));
}

// The factory is copied out and called with the lock released: it runs user
// code, which may register further ids.
std::unique_ptr<Task> PluginRegistry::createTask(std::string_view id, const TaskParams& params) const {
    TaskFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const TaskFactory* found = find(tasks_, id);
        if (!found) return nullptr;
        factory = *found;
    }
    return factory(params);
}

std::unique_ptr<ObservationBuilder> PluginRegistry::createObservation(std::string_view id) const {
    ObservationFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const ObservationFactory* found = find(observations_, id);
        if (!found) return nullptr;
        factory = *found;
    }
    return factory();
}

std::unique_ptr<ActionMapper> PluginRegistry::createAction(std::string_view id) const {
    ActionFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const ActionFactory* found = find(actions_, id);
        if (!found) return nullptr;
        factory = *found;
    }
    return factory();
}

std::vector<std::string> PluginRegistry::taskIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keys(tasks_);
}

std::vector<std::string> PluginRegistry::observationIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keys(observations_);
}

std::vector<std::string> PluginRegistry::actionIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keys(actions_);
}

} // namespace fsim::env

// The SDK surface (fsim/VecEnvPlugins.h): free functions over the one registry.
namespace fsim {

void registerTask(const std::string& id, TaskFactory factory) {
    env::PluginRegistry::instance().addTask(id, std::move(factory));
}

void registerObservation(const std::string& id, ObservationFactory factory) {
    env::PluginRegistry::instance().addObservation(id, std::move(factory));
}

void registerAction(const std::string& id, ActionFactory factory) {
    env::PluginRegistry::instance().addAction(id, std::move(factory));
}

std::vector<std::string> taskIds() { return env::PluginRegistry::instance().taskIds(); }
std::vector<std::string> observationIds() { return env::PluginRegistry::instance().observationIds(); }
std::vector<std::string> actionIds() { return env::PluginRegistry::instance().actionIds(); }

} // namespace fsim
