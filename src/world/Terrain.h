#pragma once

#include <vsg/all.h>

#include <cstdint>
#include <string>

namespace fsim::world {

/// Elevation tile encodings the platform understands (design 8.2 "Tile
/// pyramid format").
enum class ElevationEncoding {
    Float,     ///< single-channel float metres (R32_SFLOAT / R16_SFLOAT), VSG native
    Terrarium, ///< RGB8: h = R*256 + G + B/256 - 32768 (AWS terrain tiles, Mapzen)
};

/// Public, key-free global elevation: AWS Terrain Tiles, Terrarium encoding,
/// zoom 0-15 (~5 m/px at z15, 30 m SRTM/derived data).
inline constexpr const char* kAwsTerrariumUrl = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";

/// Decode an elevation tile image into metres. Returns a floatArray2D
/// (R32_SFLOAT) or nullptr if the input is not understood. `maxDimension`
/// downsamples large tiles (VSG builds one mesh vertex per elevation texel,
/// so 256x256 tiles would cost 130k triangles each); 0 keeps full resolution.
vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension);

} // namespace fsim::world
