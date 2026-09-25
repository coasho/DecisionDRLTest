#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <cstddef>
#include <vector>

namespace fsim::world {

/// The engines' exhaust (design 8.2 "Effects"), drawn at a model's
/// `fsim:afterburner:<engine>` nodes instead of the flame mesh the model
/// carries: with the afterburner lit, a layered flame - a white-hot core
/// with shock diamonds in it, an orange envelope cooling to its tail, blue
/// at the nozzle while the afterburner is barely lit, flickering - and the
/// nozzle glowing inside; at military power without it the nozzle's dull
/// glow and a faint haze of hot gas; at idle nothing. Each drawing hangs
/// under its node, so it follows whatever turns the node (a vectoring
/// nozzle), and the node's own geometry gives its size.
///
/// The cost is kept off the CPU: one small mesh shared by every engine, a
/// shader that shapes and animates it from a handful of numbers per engine
/// (an 8-float row in one storage buffer) and a clock, one draw per engine
/// with something to show, culled with its bounding sphere and skipped
/// below a few pixels.
class Exhaust {
public:
    /// Engines per vehicle slot it can draw.
    static constexpr std::size_t kPerSlot = sim::VehicleState::kMaxEngines;

    Exhaust(std::size_t slots, vsg::ref_ptr<const vsg::Options> options);

    /// False when the shaders did not build: models keep their own flames.
    bool valid() const noexcept { return static_cast<bool>(state_); }

    /// The drawing of engine `k` of slot `slot`, made on first use and kept
    /// (the same node every time), or null past the limits.
    vsg::ref_ptr<vsg::Node> node(std::size_t slot, std::size_t k);

    /// What engine `engine` of `state` shows this frame, for a nozzle of
    /// this radius (m). False when it shows nothing (hide the node).
    bool set(std::size_t slot, std::size_t k, double radius, const sim::VehicleState& state, int engine);

    /// The clock the exhaust flickers by (s; the simulation's, so a paused
    /// or replayed world shows the same) and the daylight (0 night .. 1
    /// day): at night a flame is all light, by day it also hides a little
    /// of the sky behind it, or it washes out to white.
    void setFrame(double seconds, float daylight);

    /// Hands this frame's numbers to the GPU (once, after the set() calls).
    void commit();

    /// What a nozzle of this radius shows at this afterburner (0..1) and
    /// dry power (0 idle .. 1 military): the flame's length (m, 0 unlit).
    static double flameLength(double radius, double afterburner);
    /// Dry power from a turbine's core speed (VehicleState::engineN2, %).
    static double dryPower(double n2Percent);

private:
    struct Flame {
        vsg::ref_ptr<vsg::LOD> lod;
        double radius = -1.0;
    };
    std::vector<Flame> flames_; ///< slot * kPerSlot + k
    vsg::ref_ptr<vsg::StateGroup> state_; ///< pipeline and descriptors, shared
    vsg::BufferInfoList arrays_;          ///< the mesh, shared by every flame's own draw
    vsg::ref_ptr<vsg::BufferInfo> indices_;
    std::uint32_t indexCount_ = 0;
    vsg::ref_ptr<vsg::vec4Array> rows_;   ///< two per flame
    vsg::ref_ptr<vsg::vec4Value> frame_;  ///< x time (s), y daylight
    bool changed_ = false;
};

} // namespace fsim::world
