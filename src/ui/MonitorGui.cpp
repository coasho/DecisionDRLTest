#include "ui/MonitorGui.h"

#include "core/Units.h"
#include "world/CameraController.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cstdio>

namespace fsim::ui {

MonitorGui::MonitorGui(std::shared_ptr<ViewerControls> controls, std::string aircraft)
    : controls_(std::move(controls)), aircraft_(std::move(aircraft)) {}

void MonitorGui::record(vsg::CommandBuffer&) const {
    if (showLabels_) drawLabels();
    if (controls_->showMonitor.load(std::memory_order_relaxed)) drawMonitor();
    if (controls_->showVehicleList.load(std::memory_order_relaxed)) drawVehicleList();
}

void MonitorGui::drawLabels() const {
    // Background draw list: labels sit under the panels, over the 3D scene.
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = ImGui::GetIO().FontGlobalScale;
    for (const Label& l : labels_) {
        const ImVec2 size = ImGui::CalcTextSize(l.text.c_str());
        const ImVec2 pos(l.x + 10.0f * k, l.y - size.y * 0.5f);
        const ImU32 bg = l.selected ? IM_COL32(230, 140, 30, 200) : IM_COL32(20, 30, 50, 170);
        dl->AddRectFilled(ImVec2(pos.x - 4.0f * k, pos.y - 2.0f * k), ImVec2(pos.x + size.x + 4.0f * k, pos.y + size.y + 2.0f * k), bg, 3.0f * k);
        dl->AddLine(ImVec2(l.x, l.y), ImVec2(pos.x - 4.0f * k, l.y), bg, 1.5f * k);
        dl->AddText(pos, IM_COL32(255, 255, 255, 255), l.text.c_str());
    }
}

void MonitorGui::drawMonitor() const {
    const float k = ImGui::GetIO().FontGlobalScale; // DPI scale applied by the app
    ImGui::SetNextWindowPos(ImVec2(12.0f * k, 12.0f * k), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400.0f * k, 0.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("flightsim monitor", nullptr, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    const double fps = controls_->fps.load(std::memory_order_relaxed);
    const double frameMs = controls_->frameMs.load(std::memory_order_relaxed);
    const double sps = controls_->simThroughput.load(std::memory_order_relaxed);
    const double simTime = controls_->simTime.load(std::memory_order_relaxed);

    std::size_t live = 0;
    if (!vehicles_.empty()) {
        for (const auto& v : vehicles_) live += v.alive ? 1 : 0;
    } else if (batch_) {
        live = batch_->states.size();
    }
    if (source_.mirror) {
        if (!source_.attached) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "waiting for a training application...");
            ImGui::TextDisabled("start any program that creates an fsim::World");
            if (!source_.available.empty()) {
                ImGui::Text("worlds in the registry:");
                for (const auto& w : source_.available) ImGui::BulletText("%s", w.c_str());
            }
        } else {
            ImGui::Text("world '%s'  x %zu vehicle(s)", source_.world.c_str(), live);
            if (!source_.publisherAlive) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "training application has exited");
            else if (source_.ageSeconds > 2.0) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "not updating (%.0f s)", source_.ageSeconds);
            else ImGui::TextDisabled("live, last update %.0f ms ago", source_.ageSeconds * 1e3);
        }
        ImGui::Separator();
        ImGui::Text("render   %6.1f fps   %6.2f ms/frame", fps, frameMs);
        ImGui::Text("trainer  %9.0f vehicle-steps/s", source_.vehicleStepsPerSecond);
        ImGui::Text("sim time %9.2f s", source_.simTime);
        if (!source_.environment.empty()) ImGui::TextWrapped("%s", source_.environment.c_str());
        ImGui::Separator();
        ImGui::TextDisabled("the training application sets the pace");
    } else {
        ImGui::Text("%s  x %zu", aircraft_.c_str(), live);
        ImGui::Separator();
        ImGui::Text("render   %6.1f fps   %6.2f ms/frame", fps, frameMs);
        ImGui::Text("sim      %9.0f vehicle-steps/s", sps);
        ImGui::Text("sim time %9.2f s", simTime);

        ImGui::Separator();
        bool paused = controls_->paused.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("pause  [space]", &paused)) controls_->paused.store(paused, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::Button("step  [.]")) controls_->singleStep.store(true, std::memory_order_relaxed);

        float factor = static_cast<float>(controls_->timeFactor.load(std::memory_order_relaxed));
        if (ImGui::SliderFloat("time factor", &factor, 0.1f, 32.0f, "%.2fx", ImGuiSliderFlags_Logarithmic))
            controls_->timeFactor.store(static_cast<double>(factor), std::memory_order_relaxed);
    }

    int mode = controls_->cameraMode.load(std::memory_order_relaxed);
    const char* modes[] = {"chase (follows heading)", "orbit (north-up)", "overview"};
    if (ImGui::Combo("camera  [c]", &mode, modes, 3)) controls_->cameraMode.store(mode, std::memory_order_relaxed);
    if (ImGui::Button("zoom -  [-]")) controls_->cameraZoom.store(1.25, std::memory_order_relaxed);
    ImGui::SameLine();
    if (ImGui::Button("zoom +  [=]")) controls_->cameraZoom.store(0.8, std::memory_order_relaxed);
    ImGui::SameLine();
    if (ImGui::Button("reset view  [r]")) controls_->cameraReset.store(true, std::memory_order_relaxed);
    ImGui::TextDisabled("mouse: left drag rotate (throw), middle drag pan, right drag / wheel zoom");

    bool list = controls_->showVehicleList.load(std::memory_order_relaxed);
    if (ImGui::Checkbox("vehicle list  [l]", &list)) controls_->showVehicleList.store(list, std::memory_order_relaxed);

    if (batch_ && !batch_->states.empty()) {
        const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0,
                                   static_cast<int>(batch_->states.size()) - 1);
        const auto& s = batch_->states[static_cast<std::size_t>(sel)];
        ImGui::Separator();
        const auto* meta = static_cast<std::size_t>(sel) < vehicles_.size() ? &vehicles_[static_cast<std::size_t>(sel)] : nullptr;
        if (meta && meta->alive) ImGui::Text("%s  (%s, %s)  [tab / shift+tab]", meta->name.c_str(), meta->type.c_str(), meta->level);
        else ImGui::Text("vehicle %d  [tab / shift+tab]", sel);
        ImGui::Text("alt %7.1f m   agl %7.1f m   tas %5.1f m/s (%4.0f kt)", s.altitudeMslM, s.altitudeAglM,
                    s.airspeedTrueMs, units::metresPerSecondToKnots(s.airspeedTrueMs));
        ImGui::Text("roll %6.1f  pitch %6.1f  hdg %6.1f  alpha %5.2f  n %4.2f", units::radiansToDegrees(s.eulerRad[0]),
                    units::radiansToDegrees(s.eulerRad[1]), units::radiansToDegrees(s.eulerRad[2]),
                    units::radiansToDegrees(s.alphaRad), s.loadFactor);
        ImGui::Text("lat %9.5f  lon %10.5f%s%s", units::radiansToDegrees(s.latitudeRad),
                    units::radiansToDegrees(s.longitudeRad), s.onGround ? "  [ground]" : "", s.diverged ? "  [DIVERGED]" : "");
    }
    ImGui::End();
}

void MonitorGui::drawVehicleList() const {
    if (!batch_) return;
    const float k = ImGui::GetIO().FontGlobalScale;
    ImGui::SetNextWindowPos(ImVec2(12.0f * k, 440.0f * k), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f * k, 320.0f * k), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("vehicles", nullptr)) {
        ImGui::End();
        return;
    }
    const int selected = controls_->selectedVehicle.load(std::memory_order_relaxed);
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("vehicles", 8, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("name");
        ImGui::TableSetupColumn("control");
        ImGui::TableSetupColumn("alt m");
        ImGui::TableSetupColumn("agl m");
        ImGui::TableSetupColumn("tas m/s");
        ImGui::TableSetupColumn("hdg");
        ImGui::TableSetupColumn("roll");
        ImGui::TableSetupColumn("state");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < batch_->states.size(); ++i) {
            const auto* meta = i < vehicles_.size() ? &vehicles_[i] : nullptr;
            if (meta && !meta->alive) continue;
            const auto& s = batch_->states[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char label[96];
            if (meta) std::snprintf(label, sizeof label, "%s##%zu", meta->name.c_str(), i);
            else std::snprintf(label, sizeof label, "v%zu", i);
            if (ImGui::Selectable(label, static_cast<int>(i) == selected, ImGuiSelectableFlags_SpanAllColumns))
                controls_->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(meta ? meta->level : "");
            if (s.diverged) {
                for (int c = 2; c <= 6; ++c) { ImGui::TableSetColumnIndex(c); ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(7);
                ImGui::TextUnformatted("DIVERGED");
                continue;
            }
            ImGui::TableSetColumnIndex(2); ImGui::Text("%7.0f", s.altitudeMslM);
            ImGui::TableSetColumnIndex(3); ImGui::Text("%7.0f", s.altitudeAglM);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%5.1f", s.airspeedTrueMs);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%5.0f", units::radiansToDegrees(s.eulerRad[2]));
            ImGui::TableSetColumnIndex(6); ImGui::Text("%5.0f", units::radiansToDegrees(s.eulerRad[0]));
            ImGui::TableSetColumnIndex(7);
            ImGui::TextUnformatted(s.onGround ? "ground" : "airborne");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace fsim::ui
