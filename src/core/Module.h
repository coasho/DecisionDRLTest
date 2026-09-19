#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fsim {

struct Context; // services handed to modules; grows with the platform

/// Lifecycle interface every optional or built-in module implements
/// (design 10.1). Registered with ModuleRegistry in dependency order; the
/// registry calls the hooks in that order and shuts down in reverse, even
/// after an exception.
class Module {
public:
    virtual ~Module() = default;

    virtual std::string_view name() const noexcept = 0;

    /// Acquire resources, register into registries. May throw on fatal misconfiguration.
    virtual void init(Context&) {}
    /// Start threads or begin work. Called after every module's init().
    virtual void start(Context&) {}
    /// Stop threads; must not throw.
    virtual void stop(Context&) noexcept {}
    /// Release resources; must not throw.
    virtual void shutdown(Context&) noexcept {}
};

class ModuleRegistry {
public:
    void add(std::unique_ptr<Module> module);

    /// init() then start() every module in registration order. If any hook
    /// throws, modules already started are stopped and shut down in reverse
    /// order and the exception is rethrown.
    void startAll(Context& ctx);

    /// stop() then shutdown() in reverse registration order. Safe to call twice.
    void shutdownAll(Context& ctx) noexcept;

    Module* find(std::string_view name) const noexcept;
    std::size_t size() const noexcept { return modules_.size(); }

private:
    std::vector<std::unique_ptr<Module>> modules_;
    std::size_t started_ = 0; ///< how many modules have completed start()
};

} // namespace fsim
