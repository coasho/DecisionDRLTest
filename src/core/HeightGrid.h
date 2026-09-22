#pragma once

#include <cstdint>
#include <vector>

namespace fsim::core {

/// Box-filter a square height raster and resample it to `outSize` samples per
/// edge - the surface the terrain mesh is built from.
///
/// This lives in core because two modules have to agree on it exactly. `world`
/// builds the mesh the vertex shader displaces; `io` answers the camera's
/// question "how high is the ground here?". When those two disagreed the
/// camera cleared a hill that was not the one on screen: measured over 40
/// tiles of real relief, the drawn mesh sat up to 11.3 m above the heights the
/// camera was told, which is more than the clearance it keeps at close range,
/// so the eye ended up inside the hillside. One implementation, one surface.
///
/// The filter matters as much as the resampling. A vertex stands for its cell,
/// not for one raster sample: point-sampling a 4.8 m raster of a cliff every
/// 4 px turns the crest into a sawtooth of fins, while averaging over the cell
/// keeps the cliff and drops the aliasing. Past the raster's edge the window
/// continues the edge *slope* rather than the edge value, so the filter is
/// exact on a ramp and both sides of a tile seam estimate it the same way.
///
/// Sample k of `outSize` sits at tile fraction k/(outSize-1), reading the
/// raster where pixel centres are at (j + 0.5)/srcSize - so the first and last
/// samples fall half a pixel outside the outermost centres and are
/// extrapolated, again keeping seams closed to second order.
///
/// `outSize` of 0, or not smaller than `srcSize`, returns the raster unchanged.
std::vector<float> resampleHeightGrid(const float* src, std::uint32_t srcSize, std::uint32_t outSize);

} // namespace fsim::core
