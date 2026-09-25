#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fsim::world {

class Exhaust;

/// Shared models, N vsg::MatrixTransforms (design 8.2 "Many vehicles").
/// The default model is a glTF file when given, else a procedural placeholder
/// built in the JSBSim body frame (x forward, y right, z down) so no axis
/// conversion is needed for the placeholder; glTF models get a fixed
/// model->body rotation. Per vehicle, setModel() picks a different file: the
/// vehicle's own `model` override or `<modelDir>/<type>.glb` for its type;
/// each file is loaded and compiled once and shared by every vehicle using it.
///
/// Moving control surfaces: a model node named `fsim:<channel>[:<gain>]`
/// (channel aileron, elevator, rudder or flaps) turns about its own x axis by
/// gain times the vehicle's deflection of that channel (VehicleState, radians;
/// aileron is the left one). A part several channels move - a stabilator
/// that also rolls, a flaperon - is named `fsim:<channel>[:<gain>]+<channel>
/// [:<gain>]...` and turns by the sum. A trailing `@<lo>,<hi>` (degrees) holds
/// the turn within the part's own stops: a canard that travels further than
/// the elevons on its channel. `fsim:gear:<deg>[:<g0>:<g1>]` turns a landing
/// gear leg or door by deg as the gear position (0 up, 1 down) goes from g0 to
/// g1; `fsim:afterburner[:<engine>]` marks where that engine's jet leaves the
/// nozzle (its origin at the exit, its x axis along the jet): the viewer draws
/// the exhaust there (world::Exhaust: flame, glow, heat haze), sized by the
/// node's own geometry, the model's flame mesh the node carries standing in
/// when that drawing is off - stretched along x with the afterburner (throttle
/// position past 1, JSBSim's augmented turbine) and hidden without it. Each
/// vehicle gets its own copy of those nodes and of the nodes above them; the
/// geometry stays shared.
class VehicleVisuals {
public:
    /// A lifting surface of the model, from its manifest (`wing`, `strake` or
    /// `canard` lines): its right half's sections root to tip, each a leading
    /// edge (body axes from the model's origin, m) and a chord. The viewer's
    /// airflow effects hang on them.
    struct Surface {
        std::string kind;
        std::vector<vsg::dvec4> sections; ///< x, y, z of the leading edge; w the chord
    };

    struct Settings {
        std::string modelPath;           ///< glTF/OBJ path; empty = placeholder
        double modelScale = 1.0;
        vsg::dvec3 modelForward{0.0, 0.0, -1.0}; ///< model-space nose direction (glTF default -Z)
        vsg::dvec3 modelUp{0.0, 1.0, 0.0};       ///< model-space up (glTF default +Y)
        double placeholderLengthM = 8.3; ///< c172-ish
        double placeholderSpanM = 11.0;
        std::vector<std::filesystem::path> modelDirs; ///< searched for `<type>.glb` / `.gltf` (type without its "jsbsim:" prefix)
        bool segmentation = false;       ///< also build the id-coloured copy (segmentation cameras)
        vsg::dvec3 modelOffset{0.0, 0.0, 0.0}; ///< where the model sits in the body frame (m: forward, right, down)
        /// Draw the engines' exhaust (world::Exhaust) at `fsim:afterburner`
        /// nodes instead of the flame mesh the model carries there.
        bool exhaust = true;
        std::vector<Surface> surfaces;   ///< from the manifest
    };

    /// A stock aircraft drawn with a model designed here: the design whose
    /// model stands in for it, and where that model sits (body axes, m) so its
    /// wheels stand where the stock aircraft's do.
    struct StandIn {
        std::string design;
        vsg::dvec3 offset{0.0, 0.0, 0.0};
    };
    /// Reads a register of stand-ins (`models.txt` in a model directory, as
    /// `fsim hangar register` writes it): lines `<type> <design> <forward>
    /// <right> <down>`, # comments. Entries already in `out` are kept.
    static bool readStandIns(const std::filesystem::path& file, std::map<std::string, StandIn>& out);

    /// Compiles a subgraph loaded after the scene was compiled (render::Viewer::compile).
    using Compiler = std::function<bool(vsg::ref_ptr<vsg::Node>)>;

    VehicleVisuals(std::size_t count, const Settings& settings, vsg::ref_ptr<vsg::Options> options);
    ~VehicleVisuals();

    /// Apply `<modelPath>.manifest` (key value lines: forward, up, scale, and
    /// wing/strake/canard surfaces) to `settings` if the file exists. Returns
    /// true when a manifest was read.
    static bool applyManifest(Settings& settings);

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Root of the id-coloured copy of every vehicle, or null unless
    /// `Settings::segmentation`. A view over this scene paints each vehicle
    /// one flat colour encoding its slot, so the pixels it covers name it.
    /// It follows the same transforms, visibility and masks as `node()`, so a
    /// camera that hides its own aircraft hides it here too.
    vsg::ref_ptr<vsg::Node> segmentationNode() const { return segRoot_; }

    /// Slot -> the colour its segmentation copy is painted, and back. Ids are
    /// 1-based over the low two bytes; 0 (black) means "no vehicle".
    static vsg::vec4 segmentationColour(std::size_t index);
    static unsigned segmentationId(std::uint8_t r, std::uint8_t g) noexcept {
        return static_cast<unsigned>(r) | (static_cast<unsigned>(g) << 8);
    }

    /// Write the latest snapshot into the transforms (render thread, per frame).
    void update(Span<const sim::VehicleState> states);

    void setCompiler(Compiler compiler) { compiler_ = std::move(compiler); }

    /// Choose the model drawn for one vehicle: `modelPath` (its VehicleSpec
    /// override) when it loads, else `<modelDir>/<type>.glb`, else the default.
    /// Cheap when the choice is unchanged; a file that fails to load falls back
    /// to the default and is not retried.
    void setModel(std::size_t index, const std::string& modelPath, const std::string& type);

    /// The file drawn for a vehicle (empty for the default model).
    const std::string& modelOf(std::size_t index) const;

    /// Highlight one vehicle (or none with -1).
    void setSelected(int index);

    /// Show or hide one vehicle (mirror mode: slots without a live vehicle).
    void setVisible(std::size_t index, bool visible);

    /// Traversal mask of a visible vehicle (default MASK_ALL): a vsg::View whose
    /// mask does not overlap it skips the vehicle (a camera hiding its own aircraft).
    void setMask(std::size_t index, vsg::Mask mask);
    bool visible(std::size_t index) const noexcept { return index < visible_.size() && visible_[index]; }
    int selected() const noexcept { return selected_; }

    std::size_t count() const { return transforms_.size(); }

    /// Animations found in the loaded model (e.g. propellers), empty for the placeholder.
    const vsg::Animations& animations() const { return animations_; }

    /// Draw the engines' exhaust (on by default when Settings::exhaust): off,
    /// the models' own flame meshes are shown as before.
    void setExhaust(bool on);
    bool exhaust() const noexcept { return exhaustOn_; }
    /// How bright the day is where the camera looks (0 night .. 1 day); the
    /// exhaust's flame reads differently against a dark and a bright sky.
    void setDaylight(float daylight) noexcept { daylight_ = daylight; }

    /// Where a model's exhausts and lifting surfaces are, and how big it is:
    /// what the viewer's airflow effects hang on (body axes from the vehicle's
    /// reference point, m, the model as it is drawn - moved and scaled).
    struct Shape {
        struct Jet {
            vsg::dvec3 exit, direction; ///< the jet's centre as it leaves the nozzle, and which way it runs
            double radius = 0.0;
            int engine = 0;
        };
        std::vector<Jet> jets;          ///< the fsim:afterburner nodes at rest
        std::vector<Surface> surfaces;  ///< from the manifest
        vsg::dvec3 lo, hi;              ///< its bounding box (the exhaust flames' meshes left out)
        vsg::dvec3 tip;                 ///< the right wing tip's trailing edge (the widest point, lacking a wing)
        bool valid = false;             ///< false for the placeholder and a model that did not load
    };
    const Shape& shape(std::size_t index) const;

    /// A moving part of a model: a node named fsim:<channel>[:<gain>][+...],
    /// fsim:gear:<deg>[:<g0>:<g1>], fsim:afterburner[:<engine>],
    /// fsim:lef[:<gain>][@<lo>,<hi>], fsim:propeller:<engine>,
    /// fsim:nozzle:<engine>:<deg>, fsim:oleo:<wheel>[:<gain>],
    /// fsim:steer:<wheel> or fsim:wheel:<wheel>:<radius> - each moved by what
    /// the simulation reports (VehicleState), never by a guess of its own.
    struct Joint {
        enum Channel { Aileron, Elevator, Rudder, Flaps };
        enum Kind { Surface, Gear, Afterburner, LeadingEdge, Propeller, Nozzle, Oleo, Steer, Wheel };
        Kind kind = Surface;
        const vsg::MatrixTransform* node = nullptr; ///< in the shared model
        vsg::dmat4 rest;                           ///< its matrix at zero deflection
        Channel channel = Aileron;                 ///< the first channel in the name
        double gain = 1.0;
        std::vector<std::pair<Channel, double>> mix; ///< the channels after it
        double lo = -std::numeric_limits<double>::infinity(); ///< its stops (rad), from `@<lo>,<hi>` (degrees)
        double hi = std::numeric_limits<double>::infinity();
        double gearRad = 0.0, g0 = 1.0, g1 = 0.0; ///< gear: turned gearRad as the position goes g0 -> g1
        int engine = 0;                            ///< afterburner: the engine whose plume it is
        /// afterburner: the jet's radius (m), the widest point of the node's
        /// own geometry across its x axis; 0 when it has none
        double radius = 0.0;
        vsg::dmat4 inModel;                        ///< its rest matrix in the model's (body) frame, all above it at rest
        /// propeller: the turn it has made (0..1) and the sim time of that, so it
        /// turns on at the engine's rpm; a positive turn is about the node's x axis
        double phase = 0.0;
        double phaseTime = std::numeric_limits<double>::quiet_NaN();
        /// nozzle petal: turned this far (rad) wide open (VehicleState::nozzlePosition 1)
        double nozzleRad = 0.0;
        /// oleo, steer, wheel: the wheeled gear unit (VehicleState::wheel*), and a
        /// wheel's radius (m), which turns its rolling speed into its spin
        int wheel = 0;
        double wheelRadius = 0.0;
        /// Parses a node name; false when it is not a joint (or malformed).
        static bool parse(const std::string& name, Joint& joint);
        /// The node's matrix for the vehicle's state (a propeller or a wheel
        /// also advances its turn).
        vsg::dmat4 matrix(const sim::VehicleState& state);
        /// Advances the turn (0..1) at this many turns a second over the sim
        /// time since the last call; held while the clock stops or jumps.
        double turn(double perSecond, double simTime);
    };

    /// The joints of one slot's model and their transforms in that slot's own
    /// copy (empty for a model without joints).
    struct Pose {
        vsg::ref_ptr<vsg::Node> normal, geometry; ///< the slot's copies (null: the shared graph)
        std::vector<Joint> joints;                ///< one per transform, both copies
        std::vector<vsg::ref_ptr<vsg::MatrixTransform>> transforms;
        /// per transform: the Exhaust drawing it carries (-1 none), and the
        /// switch between that drawing and the model's own flame
        std::vector<int> flames;
        std::vector<vsg::ref_ptr<vsg::Switch>> flameSwitches;
    };
    const Pose& pose(std::size_t index) const { return poses_.at(index); }

private:
    /// Every joint in a graph and the nodes on the paths down to them: the
    /// nodes a vehicle needs its own copy of.
    struct Rig {
        std::vector<Joint> joints;
        std::vector<const vsg::Object*> spine;
        static Rig find(const vsg::ref_ptr<vsg::Node>& graph);
        /// A copy of `graph` with the spine duplicated and the rest shared;
        /// `transforms` receives each joint's copy, in joint order.
        vsg::ref_ptr<vsg::Node> copy(const vsg::ref_ptr<vsg::Node>& graph,
                                     std::vector<vsg::ref_ptr<vsg::MatrixTransform>>& transforms) const;
    };
    struct Model {
        vsg::ref_ptr<vsg::Node> normal, highlighted;
        vsg::ref_ptr<vsg::Node> geometry; ///< state-free, white-vertex-colour copy for the segmentation pass
        Rig rig, geometryRig;
        Shape shape;
    };
    /// A loaded model's shape: its jets from its rig, its surfaces from the
    /// manifest (moved as the model is), its extent from its vertices.
    static Shape measure(const vsg::ref_ptr<vsg::Node>& model, const Rig& rig, const Settings& s);
    /// Points slot `index` at model `m`: the shared graph, or the slot's own
    /// copy when the model has joints.
    void assign(std::size_t index, const Model& m);
    std::vector<Pose> poses_;
    vsg::ref_ptr<vsg::Node> buildPlaceholder(const Settings& s, const vsg::vec4& color) const;
    /// Flatten a model to transform + draw pairs with white vertex colours,
    /// dropping its own pipelines, textures and materials so one flat
    /// pipeline can paint it a single id colour.
    static vsg::ref_ptr<vsg::Node> stripState(const vsg::ref_ptr<vsg::Node>& model);
    vsg::ref_ptr<vsg::Node> loadModel(const Settings& s, vsg::ref_ptr<vsg::Options> options) const;
    std::string resolveModel(const std::string& modelPath, const std::string& type) const;
    const Model& modelFor(const std::string& key);

    Settings settings_;
    vsg::ref_ptr<vsg::Options> options_;
    Compiler compiler_;
    Model default_;
    std::map<std::string, Model> library_; ///< by resolved path; an empty Model = failed to load
    std::map<std::string, StandIn> standIns_; ///< stock types drawn with a design's model (models.txt)
    std::vector<std::string> slotModel_;
    std::vector<const Model*> slotModels_; ///< what each slot draws (its shape)

    std::unique_ptr<Exhaust> exhaust_;
    bool exhaustOn_ = false;
    float daylight_ = 1.0f;
    std::vector<unsigned char> flameCompiled_; ///< per slot and engine: compiled by compiler_

    vsg::ref_ptr<vsg::Group> root_;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> transforms_;
    std::vector<vsg::ref_ptr<vsg::Switch>> highlight_;
    int selected_ = -1;
    std::vector<unsigned char> visible_;
    std::vector<vsg::Mask> onMask_;
    vsg::Animations animations_;
    void applySwitch(std::size_t index);

    // Segmentation copy: one transform per slot mirroring transforms_, each
    // holding a Switch over a StateGroup that paints the slot's id colour.
    vsg::ref_ptr<vsg::Group> segRoot_;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> segTransforms_;
    std::vector<vsg::ref_ptr<vsg::Switch>> segSwitch_;
    std::vector<vsg::ref_ptr<vsg::StateGroup>> segState_;
};

} // namespace fsim::world
