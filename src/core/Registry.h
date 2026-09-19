#pragma once

#include "core/Log.h"

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace fsim {

/// String-id -> factory registry (design 10.2). Unknown ids log an error and
/// return null; they never crash. One instance per extension point (tasks,
/// observation builders, flight models, camera controllers, ...).
template <typename Interface, typename... Args>
class Registry {
public:
    using Factory = std::function<std::unique_ptr<Interface>(Args...)>;

    /// Register `factory` under `id`. Re-registering an id replaces the
    /// previous factory and logs at debug level (modules may override defaults).
    void add(std::string id, Factory factory) {
        if (factories_.count(id)) LOG_DEBUG("registry") << "replacing '" << id << "'";
        factories_[std::move(id)] = std::move(factory);
    }

    bool contains(std::string_view id) const { return factories_.count(std::string(id)) != 0; }

    std::unique_ptr<Interface> create(std::string_view id, Args... args) const {
        auto it = factories_.find(std::string(id));
        if (it == factories_.end()) {
            LOG_ERROR("registry") << "unknown id '" << id << "'";
            return nullptr;
        }
        return it->second(std::forward<Args>(args)...);
    }

    std::vector<std::string> ids() const {
        std::vector<std::string> out;
        out.reserve(factories_.size());
        for (const auto& [id, _] : factories_) out.push_back(id);
        return out;
    }

private:
    std::unordered_map<std::string, Factory> factories_;
};

} // namespace fsim
