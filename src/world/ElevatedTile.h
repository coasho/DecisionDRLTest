#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// vsg::tile reader whose tiles carry culling bounds that include the relief.
///
/// VSG displaces tile meshes with the elevation texture in the vertex shader
/// but computes each PagedLOD/CullGroup bound from the flat, undisplaced
/// geometry at ellipsoid height 0. In mountains the drawn surface sits
/// kilometres above that sphere, so tiles close to the camera (whose flat
/// bound falls below the bottom of the view) are culled: grey holes with the
/// silhouette of the terrain behind. This reader post-processes every tile it
/// loads: it finds the tile's elevation data, takes its min/max and lifts and
/// grows the bound to enclose the displaced mesh. LOD selection is barely
/// affected because the growth is the tile's own relief, not a global margin.
class ElevatedTile : public vsg::Inherit<vsg::tile, ElevatedTile> {
public:
    ElevatedTile(vsg::ref_ptr<vsg::TileDatabaseSettings> tileSettings, vsg::ref_ptr<const vsg::Options> options)
        : Inherit(tileSettings, options) {}

    vsg::ref_ptr<vsg::Object> read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options = {}) const override;

    /// Correct the bounds of every PagedLOD / CullGroup under `node` in place.
    static void fixBounds(vsg::Object* node);
};

/// vsg::TileDatabase::readDatabase() with ElevatedTile as the tile reader and
/// `extraReaders` (e.g. ElevationUpsampler) tried before the stock ones.
bool readElevatedDatabase(vsg::TileDatabase& database, vsg::ref_ptr<const vsg::Options> options,
                          const std::vector<vsg::ref_ptr<vsg::ReaderWriter>>& extraReaders = {});

} // namespace fsim::world
