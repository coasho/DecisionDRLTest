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
    if (controls_->showMonitor.load(std::memory_order_relaxed)) drawMonitor();
    if (controls_->showVehicleList.load(std::memory_order_relaxed)) drawVehicleList();
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

    ImGui::Text("%s  x %zu", aircraft_.c_str(), batch_ ? batch_->states.size() : std::size_t{0});
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

    int mode = controls_->cameraMode.load(std::memory_order_relaxed);
    const char* modes[] = {"chase", "orbit (mouse)", "overview"};
    if (ImGui::Combo("camera  [c]", &mode, modes, 3)) controls_->cameraMode.store(mode, std::memory_order_relaxed);
    if (ImGui::Button("zoom -  [-]")) controls_->cameraZoom.store(1.25, std::memory_order_relaxed);
    ImGui::SameLine();
    if (ImGui::Button("zoom +  [=]")) controls_->cameraZoom.store(0.8, std::memory_order_relaxed);

    bool list = controls_->showVehicleList.load(std::memory_order_relaxed);
    if (ImGui::Checkbox("vehicle list  [l]", &list)) controls_->showVehicleList.store(list, std::memory_order_relaxed);

    if (batch_ && !batch_->states.empty()) {
        const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0,
                                   static_cast<int>(batch_->states.size()) - 1);
        const auto& s = batch_->states[static_cast<std::size_t>(sel)];
        ImGui::Separator();
        ImGui::Text("vehicle %d  [tab / shift+tab]", sel);
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
    if (ImGui::BeginTable("vehicles", 7, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("#");
        ImGui::TableSetupColumn("alt m");
        ImGui::TableSetupColumn("agl m");
        ImGui::TableSetupColumn("tas m/s");
        ImGui::TableSetupColumn("hdg");
        ImGui::TableSetupColumn("roll");
        ImGui::TableSetupColumn("state");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < batch_->states.size(); ++i) {
            const auto& s = batch_->states[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char label[16];
            std::snprintf(label, sizeof label, "%zu", i);
            if (ImGui::Selectable(label, static_cast<int>(i) == selected, ImGuiSelectableFlags_SpanAllColumns))
                controls_->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
            ImGui::TableSetColumnIndex(1); ImGui::Text("%7.0f", s.altitudeMslM);
            ImGui::TableSetColumnIndex(2); ImGui::Text("%7.0f", s.altitudeAglM);
            ImGui::TableSetColumnIndex(3); ImGui::Text("%5.1f", s.airspeedTrueMs);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%5.0f", units::radiansToDegrees(s.eulerRad[2]));
            ImGui::TableSetColumnIndex(5); ImGui::Text("%5.0f", units::radiansToDegrees(s.eulerRad[0]));
            ImGui::TableSetColumnIndex(6);
            ImGui::Text("%s", s.diverged ? "DIVERGED" : s.onGround ? "ground" : "airborne");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace fsim::ui
