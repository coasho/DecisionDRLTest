#pragma once

// Elevation tiles below the pyramid's deepest level, made from the level
// above (design 8.2): the AWS Terrarium pyramid stops at level 15 (~4.8 m/px)
// while imagery goes to 19. Rather than capping the whole Earth at 15 (blurry
// ground at low altitude), this reader answers requests for deeper elevation
// tiles by cropping and bilinearly upsampling the deepest real ancestor, so
// the imagery keeps refining over a smooth version of the same relief and
// nothing pops.

#include "world/Terrain.h"

#include <vsg/all.h>

#include <string>

namespace fsim::world {

class ElevationUpsampler : public vsg::Inherit<vsg::ReaderWriter, ElevationUpsampler> {
public:
    /// `urlTemplate` is the elevation layer's XYZ template; levels above `maxSourceLevel` are synthesised.
    ElevationUpsampler(std::string urlTemplate, unsigned maxSourceLevel, ElevationEncoding encoding);

    vsg::ref_ptr<vsg::Object> read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options = {}) const override;

    /// Parse z/x/y out of a URL made from the template; false if it is not one.
    bool parse(const std::string& url, unsigned& z, unsigned& x, unsigned& y) const;
    std::string url(unsigned z, unsigned x, unsigned y) const;

private:
    std::string template_;
    unsigned maxSourceLevel_;
    ElevationEncoding encoding_;
};

} // namespace fsim::world
