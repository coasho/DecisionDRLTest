#pragma once

#include <filesystem>
#include <optional>
#include <vector>

namespace fsim::io {

/// Resolves asset paths through an ordered search-path list (design 8.5).
/// Default search order: explicit paths added by the caller, then
/// <module>/../share/flightsim - the module being libfsim.dll, or the
/// executable it is linked into - then <exe>/../share/flightsim when the
/// executable is a host elsewhere (python.exe), then the source tree
/// (development builds).
class AssetResolver {
public:
    AssetResolver();

    /// Prepend a directory to the search list (highest priority).
    void addSearchPath(std::filesystem::path dir);

    /// First existing `<searchPath>/<relative>` or nullopt.
    std::optional<std::filesystem::path> find(const std::filesystem::path& relative) const;

    /// Root of a JSBSim data tree (contains aircraft/, engine/, systems/):
    /// `explicitRoot` if given and valid, else the first search path that has
    /// a `jsbsim/aircraft` or `aircraft` directory, else the build-time
    /// submodule location.
    std::optional<std::filesystem::path> jsbsimRoot(const std::filesystem::path& explicitRoot = {}) const;

    const std::vector<std::filesystem::path>& searchPaths() const noexcept { return paths_; }

private:
    static bool looksLikeJsbsimRoot(const std::filesystem::path& dir);
    std::vector<std::filesystem::path> paths_;
};

} // namespace fsim::io
