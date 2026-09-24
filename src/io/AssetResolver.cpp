#include "io/AssetResolver.h"

#include "core/Log.h"
#include "platform/Paths.h"

#include <cstdlib>
#include <string>

namespace fsim::io {

namespace fs = std::filesystem;

AssetResolver::AssetResolver() {
    // Beside the module first: inside Python, Rust or C# the executable is
    // the host's (python.exe), and the platform's files are wherever
    // libfsim.dll was installed. For our own programs the two are the same.
    const fs::path module = platform::moduleDir(), exe = platform::executableDir();
    paths_.push_back(module / ".." / "share" / "flightsim");
    paths_.push_back(module / "share");
    std::error_code ec;
    if (!fs::equivalent(module, exe, ec)) {
        paths_.push_back(exe / ".." / "share" / "flightsim");
        paths_.push_back(exe / "share");
    }
#ifdef FSIM_JSBSIM_DATA_DIR
    paths_.push_back(fs::path(FSIM_JSBSIM_DATA_DIR).parent_path()); // third_party/
#endif
}

void AssetResolver::addSearchPath(fs::path dir) { paths_.insert(paths_.begin(), std::move(dir)); }

std::optional<fs::path> AssetResolver::find(const fs::path& relative) const {
    std::error_code ec;
    for (const auto& base : paths_) {
        const fs::path candidate = base / relative;
        if (fs::exists(candidate, ec)) return fs::weakly_canonical(candidate, ec);
    }
    return std::nullopt;
}

bool AssetResolver::looksLikeJsbsimRoot(const fs::path& dir) {
    std::error_code ec;
    return fs::is_directory(dir / "aircraft", ec) && fs::is_directory(dir / "engine", ec) &&
           fs::is_directory(dir / "systems", ec);
}

namespace {

// FSIM_AIRCRAFT_PATH: directories of aircraft folders, ';'-separated.
std::vector<fs::path> environmentAircraftDirs() {
    std::vector<fs::path> out;
    const char* env = std::getenv("FSIM_AIRCRAFT_PATH");
    if (!env) return out;
    const std::string list(env);
    std::size_t start = 0;
    for (;;) {
        const std::size_t end = list.find(';', start);
        const std::string item = list.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!item.empty()) out.emplace_back(item);
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return out;
}

} // namespace

std::vector<fs::path> AssetResolver::aircraftDirs() const {
    std::vector<fs::path> out = environmentAircraftDirs();
    std::error_code ec;
#ifdef FSIM_AIRCRAFT_DIR
    // The source tree before any packaged copy: a development build stages
    // the designs into its Python package, and a design rebuilt with hangar
    // since would otherwise be shadowed by that stale copy. On a machine
    // without the source tree the directory does not exist.
    if (fs::is_directory(FSIM_AIRCRAFT_DIR, ec)) out.emplace_back(FSIM_AIRCRAFT_DIR);
#endif
    for (const auto& base : paths_)
        if (fs::is_directory(base / "aircraft", ec)) out.push_back(base / "aircraft");
    return out;
}

fs::path AssetResolver::findAircraft(const std::string& name, const fs::path& jsbsimRoot) const {
    std::error_code ec;
    auto has = [&](const fs::path& dir) { return fs::is_regular_file(dir / name / (name + ".xml"), ec); };
    // an explicit FSIM_AIRCRAFT_PATH may shadow a stock aircraft; the rest may not
    for (const auto& dir : environmentAircraftDirs())
        if (has(dir)) return fs::weakly_canonical(dir, ec);
    if (!jsbsimRoot.empty() && has(jsbsimRoot / "aircraft")) return {};
    for (const auto& dir : aircraftDirs())
        if (has(dir)) return fs::weakly_canonical(dir, ec);
    return {};
}

std::optional<fs::path> AssetResolver::jsbsimRoot(const fs::path& explicitRoot) const {
    std::error_code ec;
    if (!explicitRoot.empty()) {
        if (looksLikeJsbsimRoot(explicitRoot)) return fs::weakly_canonical(explicitRoot, ec);
        LOG_ERROR("io") << "not a JSBSim data root (needs aircraft/, engine/, systems/): " << explicitRoot.string();
        return std::nullopt;
    }
    for (const auto& base : paths_) {
        for (const fs::path& candidate : {base / "jsbsim", base}) {
            if (looksLikeJsbsimRoot(candidate)) return fs::weakly_canonical(candidate, ec);
        }
    }
#ifdef FSIM_JSBSIM_DATA_DIR
    if (looksLikeJsbsimRoot(FSIM_JSBSIM_DATA_DIR)) return fs::path(FSIM_JSBSIM_DATA_DIR);
#endif
    return std::nullopt;
}

} // namespace fsim::io
