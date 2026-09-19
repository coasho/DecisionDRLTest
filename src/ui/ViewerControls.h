#pragma once

#include <atomic>
#include <cstdint>

namespace fsim::ui {

/// State shared between the ImGui layers, the key handler and the viewer loop
/// (all on the render thread) and the sim runner (its own thread). Plain
/// atomics: UI actions never take a lock the simulation could hold (design 6.3).
struct ViewerControls {
    std::atomic<bool> paused{false};
    std::atomic<bool> singleStep{false};
    std::atomic<double> timeFactor{1.0};
    std::atomic<int> selectedVehicle{0};
    std::atomic<int> cameraMode{0};      ///< world::CameraController::Mode
    std::atomic<double> cameraZoom{1.0}; ///< multiplicative request, consumed each frame
    std::atomic<bool> showVehicleList{true};
    std::atomic<bool> showMonitor{true};
    std::atomic<bool> showLabels{true};
    std::atomic<bool> showTrails{true};
    std::atomic<bool> quit{false};

    // Read-only telemetry written by the loop for the GUI.
    std::atomic<double> fps{0.0};
    std::atomic<double> frameMs{0.0};
    std::atomic<double> simThroughput{0.0}; ///< vehicle-steps/s
    std::atomic<double> simTime{0.0};
    std::atomic<std::uint64_t> snapshotSequence{0};
};

} // namespace fsim::ui
