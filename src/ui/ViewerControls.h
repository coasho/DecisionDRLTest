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
    std::atomic<double> seekTo{-1.0};   ///< replay: jump to this simulation time (< 0 = none), consumed by the loop
    // Replay extent written by the loop for the GUI (both 0 when not replaying).
    std::atomic<double> replayStart{0.0}, replayEnd{0.0};
    std::atomic<int> selectedVehicle{0};
    std::atomic<int> attachWorld{-1};    ///< mirror: index into Source::available to attach to (consumed by the loop)
    std::atomic<int> selectStep{0};      ///< +1 / -1: move the selection to the next / previous live vehicle
    std::atomic<int> cameraMode{0};      ///< world::CameraController::Mode
    std::atomic<double> cameraZoom{1.0}; ///< multiplicative request, consumed each frame
    std::atomic<bool> cameraReset{false};
    // Camera telemetry written by the loop for the GUI.
    std::atomic<double> eyeLatDeg{0.0}, eyeLonDeg{0.0}, eyeAltM{0.0}, eyeDistanceM{0.0};
    std::atomic<bool> showVehicleList{true};
    std::atomic<bool> showCameras{true};   ///< the "cameras" window (trainer camera images), when a world publishes any
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
