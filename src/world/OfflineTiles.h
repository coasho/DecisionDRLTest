#pragma once

// Reading a partial, offline tile pyramid.
//
// A map downloaded to a budget has different depths in different places and
// in different layers: a mountain range with elevation to level 11 and imagery
// only to 7, an airport with imagery to 14 and elevation to 12. VSG's tile
// reader cannot draw that as it stands, because it assumes both layers exist
// at every level it refines to:
//
//   - a tile with elevation but no imagery is not created at all, so elevation
//     downloaded for a mountain range was never drawn;
//   - a tile with imagery but no elevation is displaced by a 1x1 fallback of
//     0.0, so it is drawn at sea level - kilometre cliffs at its edges;
//   - if any of the four siblings fails, none of them refine.
//
// All three showed at once. The global base had imagery to level 7 and
// elevation to 6, so nearly every level-7 tile on Earth was flattened to sea
// level: terrain looked more detailed zoomed out than zoomed in, and tile
// edges broke into steps wherever there was relief.
//
// This reader fills the holes. A tile missing from one layer is made from its
// nearest real ancestor in that layer - the tile's share of it, resampled -
// provided that something real exists at that level among it and its three
// siblings, in either layer. Where nothing does, it declines, the four-sibling
// read fails, and the parent stays: refinement goes exactly as deep as the
// data does and no deeper, and a deeper tile is never worse than its parent.
//
// Tiles that exist on disk are left to the ordinary image readers.

#include "world/Terrain.h"

#include <vsg/all.h>

#include <string>

namespace fsim::world {

class OfflineTiles : public vsg::Inherit<vsg::ReaderWriter, OfflineTiles> {
public:
    enum class Layer { Imagery, Elevation };

    /// The two layers' {z}/{x}/{y} templates, as local file paths.
    OfflineTiles(std::string imageryTemplate, std::string elevationTemplate, ElevationEncoding encoding);

    vsg::ref_ptr<vsg::Object> read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options = {}) const override;

    /// Which layer a path was made from, and its tile address; false if neither.
    bool classify(const std::string& path, Layer& layer, unsigned& z, unsigned& x, unsigned& y) const;
    std::string path(Layer layer, unsigned z, unsigned x, unsigned y) const;

    /// Whether the tile is really on disk (rather than something this reader would make).
    bool exists(Layer layer, unsigned z, unsigned x, unsigned y) const;

private:
    std::string imagery_, elevation_;
    ElevationEncoding encoding_;
};

} // namespace fsim::world
