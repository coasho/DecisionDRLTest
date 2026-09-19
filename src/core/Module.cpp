#include "core/Module.h"

#include "core/Log.h"

namespace fsim {

void ModuleRegistry::add(std::unique_ptr<Module> module) {
    if (!module) return;
    LOG_DEBUG("core") << "module registered: " << module->name();
    modules_.push_back(std::move(module));
}

void ModuleRegistry::startAll(Context& ctx) {
    try {
        for (auto& m : modules_) {
            LOG_DEBUG("core") << "init " << m->name();
            m->init(ctx);
        }
        for (auto& m : modules_) {
            LOG_DEBUG("core") << "start " << m->name();
            m->start(ctx);
            ++started_;
        }
    } catch (...) {
        LOG_ERROR("core") << "module start failed; shutting down";
        shutdownAll(ctx);
        throw;
    }
}

void ModuleRegistry::shutdownAll(Context& ctx) noexcept {
    // stop() only those that started; shutdown() everything that was init'ed.
    for (std::size_t i = started_; i > 0; --i) modules_[i - 1]->stop(ctx);
    started_ = 0;
    for (auto it = modules_.rbegin(); it != modules_.rend(); ++it) (*it)->shutdown(ctx);
}

Module* ModuleRegistry::find(std::string_view name) const noexcept {
    for (const auto& m : modules_)
        if (m->name() == name) return m.get();
    return nullptr;
}

} // namespace fsim
