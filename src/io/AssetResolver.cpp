#include "io/AssetResolver.h"

#include "core/Log.h"
#include "platform/Paths.h"

namespace fsim::io {

namespace fs = std::filesystem;

AssetResolver::AssetResolver() {
    const fs::path exe = platform::executableDir();
    paths_.push_back(exe / ".." / "share" / "flightsim");
    paths_.push_back(exe / "share");
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

std::optional<fs::path> AssetResolver::jsbsimRoot(const fs::path& explicitRoot) const {
    std::error_code ec;
    if (!explicitRoot.empty()) {
        if (looksLikeJsbsimRoot(explicitRoot)) return fs::weakly_canonical(explicitRoot, ec);
        LOG_ERROR("io") << "not a JSBSim data root (needs aircraft/, engine/, systems/): " << explicitRoot.string();
        return std::nullopt;
    }
    for (const auto& base : paths_) {
        for (const fs::path candidate : {base / "jsbsim", base}) {
            if (looksLikeJsbsimRoot(candidate)) return fs::weakly_canonical(candidate, ec);
        }
    }
#ifdef FSIM_JSBSIM_DATA_DIR
    if (looksLikeJsbsimRoot(FSIM_JSBSIM_DATA_DIR)) return fs::path(FSIM_JSBSIM_DATA_DIR);
#endif
    return std::nullopt;
}

} // namespace fsim::io
