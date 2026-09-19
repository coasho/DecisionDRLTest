#include "sim/JsbsimLog.h"

#include "core/Log.h"

#include <input_output/FGLog.h>

#include <memory>
#include <mutex>
#include <string>

namespace fsim::sim {

namespace {

log::Level toLevel(JSBSim::LogLevel level) noexcept {
    using JL = JSBSim::LogLevel;
    switch (level) {
    case JL::BULK: return log::Level::Trace;
    case JL::DEBUG: return log::Level::Debug;
    case JL::INFO: return log::Level::Debug; // JSBSim "info" is chatty (banners, model dumps)
    case JL::WARN: return log::Level::Warn;
    case JL::ERROR: return log::Level::Error;
    case JL::FATAL: return log::Level::Error;
    case JL::STDOUT: return log::Level::Info;
    }
    return log::Level::Info;
}

/// FGLogging drives a logger as: SetLevel(level) -> Message(...)* -> Flush(),
/// all on the calling thread. Because the logger object is shared by every
/// FDM instance, per-message state lives in thread_local storage.
class LogBridge final : public JSBSim::FGLogger {
public:
    void SetLevel(JSBSim::LogLevel level) override { tls().level = level; }

    void Message(const std::string& message) override { tls().buffer += message; }

    void Format(JSBSim::LogFormat) override {} // colour codes are dropped

    void Flush() override {
        auto& t = tls();
        // Trim trailing newlines JSBSim tends to append.
        while (!t.buffer.empty() && (t.buffer.back() == '\n' || t.buffer.back() == '\r')) t.buffer.pop_back();
        if (!t.buffer.empty()) log::write(toLevel(t.level), "jsbsim", t.buffer);
        t.buffer.clear();
    }

private:
    struct ThreadState {
        JSBSim::LogLevel level = JSBSim::LogLevel::INFO;
        std::string buffer;
    };
    static ThreadState& tls() {
        thread_local ThreadState state;
        return state;
    }
};

} // namespace

void installJsbsimLogBridge() {
    static std::once_flag once;
    std::call_once(once, [] { JSBSim::SetLogger(std::make_shared<LogBridge>()); });
}

} // namespace fsim::sim
