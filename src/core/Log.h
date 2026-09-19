#pragma once

#include <functional>
#include <sstream>
#include <string>
#include <string_view>

namespace fsim::log {

enum class Level : int { Trace = 0, Debug, Info, Warn, Error, Off };

struct Record {
    Level level;
    std::string_view category; ///< module or subsystem name, e.g. "sim", "jsbsim"
    std::string_view message;
    std::int64_t timestampNs; ///< platform::Clock::nanoseconds()
};

/// Where records go. The default sink writes "[level] category: message" to
/// stderr. A trainer embedding the SDK may install its own (design 13).
using Sink = std::function<void(const Record&)>;

void setSink(Sink sink);
void setLevel(Level level) noexcept;
Level level() noexcept;

/// True if a record at `level` would be emitted. Check before formatting
/// anything expensive.
bool enabled(Level level) noexcept;

/// Emit a record. Thread-safe.
void write(Level level, std::string_view category, std::string_view message);

/// Stream-style helper: `LOG_INFO("sim") << "loaded " << name;`
class Stream {
public:
    Stream(Level level, std::string_view category) noexcept : level_(level), category_(category) {}
    ~Stream() {
        if (enabled(level_)) write(level_, category_, buffer_.str());
    }
    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;

    template <typename T>
    Stream& operator<<(const T& value) {
        if (enabled(level_)) buffer_ << value;
        return *this;
    }

private:
    Level level_;
    std::string_view category_;
    std::ostringstream buffer_;
};

const char* levelName(Level level) noexcept;

} // namespace fsim::log

#define FSIM_LOG(level, category) ::fsim::log::Stream(level, category)
#define LOG_TRACE(category) FSIM_LOG(::fsim::log::Level::Trace, category)
#define LOG_DEBUG(category) FSIM_LOG(::fsim::log::Level::Debug, category)
#define LOG_INFO(category) FSIM_LOG(::fsim::log::Level::Info, category)
#define LOG_WARN(category) FSIM_LOG(::fsim::log::Level::Warn, category)
#define LOG_ERROR(category) FSIM_LOG(::fsim::log::Level::Error, category)
