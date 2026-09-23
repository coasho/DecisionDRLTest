#include "world/Scattering.h"

#include "core/Log.h"

#include <string>

namespace fsim::world {

void addAerialPerspective(vsg::ShaderSet& shaderSet) {
    // The vertex stage hands on the ellipsoid's up alongside the normal it
    // bends with the relief: how much air a ray crosses depends on how steeply
    // it crosses the air, not on which way the slope it lands on faces.
    const std::string mainAnchor = "\nvoid main()";
    const std::string vertexDeclare = "\nlayout(location = 8) out vec3 fsimUp; // fsim: the ellipsoid's up, in view space\n";
    const std::string vertexAnchor = "normalDir = (mv * normal).xyz;";
    const std::string vertexUp = "normalDir = (mv * normal).xyz;\n    fsimUp = (mv * vec4(vsg_Normal, 0.0)).xyz; // before the relief bends it";
    const std::string fragmentDeclare = "\nlayout(location = 8) in vec3 fsimUp; // fsim: the ellipsoid's up, in view space\n";

    const std::string anchor = "outColor.rgb = (color * ambientOcclusion) + emissiveColor.rgb;";
    const std::string scattering = R"(// Aerial perspective (fsim); see addAerialPerspective().
    const vec3 fsimBetaR = vec3(5.8e-6, 13.5e-6, 33.1e-6); // Rayleigh, per metre at sea level
    const float fsimBetaM = 4.0e-6;                        // Mie, grey: clear air, little dust
    const float fsimScaleHeight = 8000.0;                  // the air as a slab this thick
    const float fsimG = 0.76;                              // Mie asymmetry: strongly forward
    // Single scattering alone leaves the horizon away from the sun almost
    // black, because the light that gets there has bounced more than once.
    // The usual stand-in is a constant added to the Rayleigh phase - but it is
    // some ten times the Rayleigh phase itself, so applying it to every ray
    // pours that much extra light into views that have hardly any air in them.
    // Looking down from 140 km over the Sahara it lifted the ground from a
    // mean of 144 to 197 and flattened what contrast the sand had from 17.5 to
    // 5.7: a white-out. The bounced light it stands for comes from the air
    // along the ray, so it is weighted by how much of the ray is air - full
    // for a horizon ray, nothing for one looking straight down.
    const float fsimMulti = 0.9;

    // The sun, straight out of the light data VSG binds for this view: skip
    // the ambient lights, then take the first directional one. Colour times
    // intensity is Esun, and the direction is already in view space.
    vec3 fsimSun = vec3(0.0);
    vec3 fsimEsun = vec3(0.0);
    if (int(lightData.values[0][1]) > 0)
    {
        int fsimAt = 1 + int(lightData.values[0][0]);
        vec4 fsimLightColor = lightData.values[fsimAt];
        fsimEsun = fsimLightColor.rgb * fsimLightColor.a;
        fsimSun = -lightData.values[fsimAt + 1].xyz;
    }

    // How steeply the ray crosses the air: against the ellipsoid's up, never
    // the relief's normal. Measured against the slope, every face seen edge-on
    // counted as a horizon ray - the whole slab of air and the full
    // multiple-scattering term - and glowed a bluish white, while the faces
    // turned to the eye stayed clear: a wet sheen on every ridge that made
    // mountains look like waves.
    vec3 fsimView = normalize(eyePos);                                   // eye -> ground
    float fsimCosZenith = max(abs(dot(fsimView, normalize(fsimUp))), 0.02);
    float fsimPath = min(length(eyePos), fsimScaleHeight / fsimCosZenith);

    vec3 fsimBetaT = fsimBetaR + vec3(fsimBetaM);
    vec3 fsimFex = exp(-fsimBetaT * fsimPath);

    float fsimCosTheta = dot(fsimView, fsimSun);
    float fsimPhaseR = 0.0596831 * (1.0 + fsimCosTheta * fsimCosTheta);
    float fsimGG = fsimG * fsimG;
    float fsimDenom = max(1.0 + fsimGG - 2.0 * fsimG * fsimCosTheta, 1e-4);
    float fsimPhaseM = 0.1193662 * (1.0 - fsimGG) / (fsimDenom * sqrt(fsimDenom));

    float fsimMultiHere = fsimMulti * (1.0 - fsimCosZenith);
    vec3 fsimIn = (fsimBetaR * (fsimPhaseR + fsimMultiHere) + vec3(fsimBetaM * fsimPhaseM)) / fsimBetaT * fsimEsun * (1.0 - fsimFex);
    outColor.rgb = (color * ambientOcclusion) * fsimFex + fsimIn;)";

    // Both stages or neither: a fragment input no vertex output feeds is
    // undefined, so if either anchor has moved the shaders stay VSG's own.
    vsg::ref_ptr<vsg::ShaderStage>* vertex = nullptr;
    vsg::ref_ptr<vsg::ShaderStage>* fragment = nullptr;
    for (auto& stage : shaderSet.stages) {
        if (!stage || !stage->module) continue;
        if (stage->stage == VK_SHADER_STAGE_VERTEX_BIT) vertex = &stage;
        if (stage->stage == VK_SHADER_STAGE_FRAGMENT_BIT) fragment = &stage;
    }
    std::string vertexSource = vertex ? (*vertex)->module->source : std::string();
    std::string fragmentSource = fragment ? (*fragment)->module->source : std::string();
    const auto vMain = vertexSource.find(mainAnchor), vAt = vertexSource.find(vertexAnchor);
    const auto fMain = fragmentSource.find(mainAnchor), fAt = fragmentSource.find(anchor);
    if (vMain == std::string::npos || vAt == std::string::npos || fMain == std::string::npos || fAt == std::string::npos) {
        LOG_WARN("world") << "tile shader has moved on; drawing without aerial perspective";
        return;
    }
    vertexSource.replace(vAt, vertexAnchor.size(), vertexUp);
    vertexSource.insert(vMain, vertexDeclare);
    fragmentSource.replace(fAt, anchor.size(), scattering);
    fragmentSource.insert(fMain, fragmentDeclare);

    // Anything already compiled was compiled from the shaders we are about to
    // replace, and getShaderStages() would hand those back instead of ours.
    shaderSet.variants.clear();
    // Fresh stages carrying only source: VSG then compiles them for whatever
    // defines a tile needs. Editing the existing modules in place instead
    // leaves their prebuilt SPIR-V in the way, and VSG quietly falls back to
    // it - the shader runs unchanged and without the imagery sampler.
    auto fresh = [](const vsg::ShaderStage& stage, const std::string& source) {
        auto s = vsg::ShaderStage::create(stage.stage, stage.entryPointName, source, stage.module->hints);
        s->specializationConstants = stage.specializationConstants;
        return s;
    };
    *vertex = fresh(**vertex, vertexSource);
    *fragment = fresh(**fragment, fragmentSource);
}

} // namespace fsim::world
