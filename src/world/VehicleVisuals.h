#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace fsim::world {

/// Shared models, N vsg::MatrixTransforms (design 8.2 "Many vehicles").
/// The default model is a glTF file when given, else a procedural placeholder
/// built in the JSBSim body frame (x forward, y right, z down) so no axis
/// conversion is needed for the placeholder; glTF models get a fixed
/// model->body rotation. Per vehicle, setModel() picks a different file: the
/// vehicle's own `model` override or `<modelDir>/<type>.glb` for its type;
/// each file is loaded and compiled once and shared by every vehicle using it.
class VehicleVisuals {
public:
    struct Settings {
        std::string modelPath;           ///< glTF/OBJ path; empty = placeholder
        double modelScale = 1.0;
        vsg::dvec3 modelForward{0.0, 0.0, -1.0}; ///< model-space nose direction (glTF default -Z)
        vsg::dvec3 modelUp{0.0, 1.0, 0.0};       ///< model-space up (glTF default +Y)
        double placeholderLengthM = 8.3; ///< c172-ish
        double placeholderSpanM = 11.0;
        std::vector<std::filesystem::path> modelDirs; ///< searched for `<type>.glb` / `.gltf` (type without its "jsbsim:" prefix)
        bool segmentation = false;       ///< also build the id-coloured copy (segmentation cameras)
    };

    /// Compiles a subgraph loaded after the scene was compiled (render::Viewer::compile).
    using Compiler = std::function<bool(vsg::ref_ptr<vsg::Node>)>;

    VehicleVisuals(std::size_t count, const Settings& settings, vsg::ref_ptr<vsg::Options> options);

    /// Apply `<modelPath>.manifest` (key value lines: forward, up, scale) to
    /// `settings` if the file exists. Returns true when a manifest was read.
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

private:
    struct Model {
        vsg::ref_ptr<vsg::Node> normal, highlighted;
        vsg::ref_ptr<vsg::Node> geometry; ///< state-free, white-vertex-colour copy for the segmentation pass
    };
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
    std::vector<std::string> slotModel_;

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
