#pragma once

// id -> factory for the three environment extension points (design 10.1
// "SDK registration"). Built-ins register themselves on first use; trainer
// code adds its own through fsim::registerTask() and friends in
// fsim/VecEnvPlugins.h. Thread-safe; registration is process-wide and takes
// effect for environments created afterwards.

#include "fsim/VecEnvPlugins.h"

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::env {

class PluginRegistry {
public:
    static PluginRegistry& instance();

    /// Register or replace a factory under `id`.
    void addTask(std::string id, TaskFactory factory);
    void addObservation(std::string id, ObservationFactory factory);
    void addAction(std::string id, ActionFactory factory);

    /// Create by id; null if unknown.
    std::unique_ptr<Task> createTask(std::string_view id, const TaskParams& params) const;
    std::unique_ptr<ObservationBuilder> createObservation(std::string_view id) const;
    std::unique_ptr<ActionMapper> createAction(std::string_view id) const;

    /// Registered ids, sorted.
    std::vector<std::string> taskIds() const;
    std::vector<std::string> observationIds() const;
    std::vector<std::string> actionIds() const;

private:
    PluginRegistry();

    template <typename Factory>
    using Table = std::vector<std::pair<std::string, Factory>>;

    mutable std::mutex mutex_;
    Table<TaskFactory> tasks_;
    Table<ObservationFactory> observations_;
    Table<ActionFactory> actions_;
};

/// Registered by the constructor (idempotent), one per source file that
/// defines built-ins.
void registerBuiltinTasks(PluginRegistry& registry);
void registerBuiltinObservations(PluginRegistry& registry);
void registerBuiltinActions(PluginRegistry& registry);

} // namespace fsim::env
