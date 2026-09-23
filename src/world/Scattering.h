#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Aerial perspective: the air between the eye and the ground scatters
/// sunlight, so distance drains contrast out of the ground and replaces it
/// with sky. Without it every ridge to the horizon is as crisp and dark as
/// the one underfoot, which is the single thing that most makes a rendered
/// landscape look like a model.
///
/// The model is the standard one for real-time aerial perspective - Rayleigh
/// and Mie extinction with their phase functions, per Hoffman and Preetham
/// ("Rendering Outdoor Light Scattering in Real Time") and O'Neil in GPU Gems
/// 2, which is what osgEarth's sky is built on as well. Referenced, not
/// copied: osgEarth is LGPL and this is MIT, so what is taken is the
/// published mathematics.
///
///     L   = L0 * Fex + Lin
///     Fex = exp(-(bR + bM) * s)
///     Lin = (bR * (phaseR + multi) + bM * phaseM) / (bR + bM) * Esun * (1 - Fex)
///
/// It needs no uniform of its own and no camera altitude: the sun comes out
/// of the light data VSG binds per view, and the air is treated as a slab of
/// one scale height whose path is H/cos(zenith), bounded by the distance
/// actually travelled. The zenith is the ray's angle to the ellipsoid's up,
/// which the vertex stage hands on as `fsimUp` beside the normal it bends
/// with the relief; measured against the relief instead, every slope seen
/// edge-on hazes like a horizon, and ridges shine as if wet.
///
/// Patches VSG's Phong fragment shader, so it applies to anything drawn with
/// `vsg::createPhongShaderSet` - the terrain tiles and the polar caps alike,
/// which is what keeps the two shading the same.
///
/// The mechanism is particular. ShaderSet caches compiled variants, so
/// editing the source in place changes nothing: getShaderStages() hands back
/// what it compiled earlier. Clearing the prebuilt SPIR-V to force a
/// recompile is worse - VSG then quietly falls back to a shader built without
/// the caller's defines, which for a tile drops the imagery sampler and
/// renders it untextured, with nothing logged. Clearing `variants` and
/// handing over a fresh ShaderStage carrying source only is what works.
void addAerialPerspective(vsg::ShaderSet& shaderSet);

} // namespace fsim::world
