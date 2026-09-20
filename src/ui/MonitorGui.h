#pragma once

#include "sim/SnapshotBuffer.h"
#include "ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>
#include <string>
#include <vector>

namespace fsim::ui {

/// ImGui monitor + vehicle list layers (design 9.5). A vsg::Command recorded
/// by vsgImGui::RenderImGui each frame; it reads the latest snapshot batch the
/// viewer loop has set and edits ViewerControls.
class MonitorGui : public vsg::Inherit<vsg::Command, MonitorGui> {
public:
    MonitorGui(std::shared_ptr<ViewerControls> controls, std::string aircraft);

    /// Called by the viewer loop before recording each frame.
    void setBatch(const sim::SnapshotBatch* batch) { batch_ = batch; }

    /// Where the vehicles come from: the built-in demo or a mirrored world.
    struct Source {
        bool mirror = false;
        bool attached = false;
        std::string world;              ///< world name (mirror) or aircraft (demo)
        double ageSeconds = 0.0;        ///< since the last publish
        bool publisherAlive = false;
        double simTime = 0.0;
        double vehicleStepsPerSecond = 0.0;
        std::string environment;        ///< one-line summary
        std::vector<std::string> available; ///< worlds seen in the registry while waiting
    };
    void setSource(Source source) { source_ = std::move(source); }

    /// Per-slot identity (mirror mode); empty = "v<i>" for every slot.
    struct VehicleMeta {
        std::string name, type;
        bool alive = true;
        const char* level = "";
    };
    void setVehicles(std::vector<VehicleMeta> vehicles) { vehicles_ = std::move(vehicles); }
    const std::vector<VehicleMeta>& vehicles() const noexcept { return vehicles_; }

    /// Screen-space vehicle labels for this frame (window pixels).
    struct Label {
        float x = 0.0f, y = 0.0f;
        std::string text;
        bool selected = false;
    };
    void setLabels(std::vector<Label> labels) { labels_ = std::move(labels); }
    void setShowLabels(bool on) { showLabels_ = on; }

    void record(vsg::CommandBuffer&) const override;

private:
    void drawMonitor() const;
    void drawVehicleList() const;

    void drawLabels() const;

    std::shared_ptr<ViewerControls> controls_;
    std::string aircraft_;
    const sim::SnapshotBatch* batch_ = nullptr;
    std::vector<Label> labels_;
    bool showLabels_ = true;
    Source source_;
    std::vector<VehicleMeta> vehicles_;
};

} // namespace fsim::ui
