#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"
#include "world/VehicleVisuals.h"

#include <vsg/all.h>

#include <cstddef>
#include <map>
#include <vector>

namespace fsim::world {

/// What the air does around the aircraft, for the viewer (design 8.2
/// "Effects"), after the effects DCS World draws - each read off the
/// vehicle's state, so what shows is what the simulation did:
///
/// - wing tip vortices: thin white trails from the tips when the wing works
///   hard - angle of attack and load factor, and roll rate on the wing it
///   raises - left behind in the air, curling with the aircraft's path;
/// - vapour over the wings and off the leading-edge extensions at high angle
///   of attack and g, flickering;
/// - the transonic vapour cone round the airframe near Mach 1, low down;
/// - contrails behind the engines where the air is cold enough (below
///   -40 C), a gap behind the nozzle, spreading and fading over a minute;
/// - streaks of air flowing past the airframe, longer and denser the faster
///   it flies: the speed, and a change in it, at a glance.
///
/// The GPU does the work: the trails are rings of samples in one buffer,
/// each segment a quad the vertex shader turns to the camera, widens and
/// fades by its age; the vapour is one small mesh per model drawn with a row
/// of numbers per vehicle; the streaks are a fixed cloud of seeds the shader
/// wraps round each aircraft. The CPU samples a few points per vehicle per
/// frame. Vehicles far from the camera draw nothing, and trails are kept for
/// the nearest few.
class Airflow {
public:
    Airflow(std::size_t slots, vsg::ref_ptr<const vsg::Options> options);

    /// False when the shaders did not build (nothing is drawn).
    bool valid() const noexcept { return static_cast<bool>(state_); }
    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Compiles what is made after the scene was (a vehicle's vapour).
    void setCompiler(VehicleVisuals::Compiler compiler) { compiler_ = std::move(compiler); }

    /// Off: nothing drawn, and the trails are forgotten.
    void setVisible(bool on);
    bool visible() const noexcept { return visible_; }

    /// The air at sea level (K; the world's): where contrails start follows it.
    void setSeaLevelTemperature(double kelvin) noexcept { seaLevelK_ = kelvin; }

    /// Per frame, after VehicleVisuals::update(): `states` as drawn, `alive`
    /// per slot, the camera (its view matrix and eye), the sun (ECEF, towards
    /// it) and the daylight (0 night .. 1 day).
    void update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                const vsg::dmat4& view, const vsg::dvec3& eye, const vsg::dvec3& sun, float daylight);

    // The rules, 0 (nothing) .. 1 (full), for the tests and the docs.
    /// Condensation in a tip vortex: the angle of attack at the tip (rad;
    /// rolling adds to it on the rising wing) and the load factor; less in
    /// the drier air high up.
    static double vortexStrength(double tipAlphaRad, double loadFactor, double altitudeM);
    /// Vapour over the wing and off its leading-edge extensions: a harder
    /// pull than the tip vortices need.
    static double wingVapour(double alphaRad, double loadFactor, double altitudeM);
    /// The vapour cone: Mach 0.93 .. 1.03, strongest just under 1, low down.
    static double vapourCone(double mach, double altitudeM);
    /// A contrail: the air at the aircraft's height colder than -40 C
    /// (the standard atmosphere from the sea-level temperature), and the
    /// engine working (0 idle .. 1 military or more).
    static double contrail(double altitudeM, double seaLevelK, double power);
    /// The air streaks, by true airspeed (m/s).
    static double streaks(double airspeedMs);

    /// Trail samples per trail, trails kept, air streaks per aircraft.
    static constexpr std::size_t kSamples = 128, kSources = 64, kStreaks = 160, kStreakVehicles = 24;

private:
    struct Source {           ///< one trail: a tip vortex or a contrail
        int slot = -1;        ///< the vehicle; -1 free
        int kind = 0;         ///< 0 vortex, 1 contrail
        int which = 0;        ///< tip (0 right, 1 left) or jet
        std::vector<vsg::dvec4> points; ///< ring: position (ECEF) and birth time
        std::vector<vsg::dvec3> values; ///< ring: strength, width, sequence
        std::size_t head = 0, count = 0;
        double lastCommit = -1e9, sequence = 0.0;
        bool used = false;    ///< fed this frame
    };
    struct Vapour {           ///< a vehicle's vapour
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::ref_ptr<vsg::Switch> sw;
        vsg::ref_ptr<vsg::LOD> lod;
        vsg::ref_ptr<vsg::VertexIndexDraw> draw;
        const VehicleVisuals::Shape* shape = nullptr;
    };
    struct Mesh {
        vsg::BufferInfoList arrays;
        vsg::ref_ptr<vsg::BufferInfo> indices;
        std::uint32_t count = 0;
        vsg::dsphere bound;
    };
    /// The trail of that vehicle, kind and tip/jet; a free one when there is
    /// none and `create` (null when all are taken).
    Source* source(int slot, int kind, int which, bool create);
    void feed(Source& s, const vsg::dvec3& at, double time, double strength, double width, double interval);
    const Mesh* mesh(const VehicleVisuals::Shape& shape);
    Vapour* vapour(std::size_t slot, const VehicleVisuals::Shape& shape);

    VehicleVisuals::Compiler compiler_;
    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Switch> switch_;
    vsg::StateCommands vapourState_; ///< pipeline and descriptors, for each vehicle's vapour
    vsg::ref_ptr<vsg::StateGroup> state_;
    vsg::ref_ptr<vsg::MatrixTransform> ribbonAt_, streakAt_; ///< at the eye: the buffers are relative to it
    vsg::ref_ptr<vsg::Draw> ribbonDraw_, streakDraw_;
    vsg::ref_ptr<vsg::vec4Array> samples_, rows_, streakRows_, frame_;
    std::vector<Source> sources_;
    std::vector<Vapour> vapour_;
    vsg::ref_ptr<vsg::Group> vapourRoot_;
    std::map<const VehicleVisuals::Shape*, Mesh> meshes_;
    double seaLevelK_ = 288.15;
    bool visible_ = true;
    double timeBase_ = 0.0, lastTime_ = -1.0;
};

} // namespace fsim::world
