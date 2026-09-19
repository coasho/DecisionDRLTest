#include "ui/KeyHandler.h"

#include <algorithm>

namespace fsim::ui {

void KeyHandler::apply(vsg::KeyPressEvent& e) {
    if (e.handled) return; // ImGui consumed it
    auto& c = *controls_;
    const bool shift = (e.keyModifier & vsg::MODKEY_Shift) != 0;

    switch (e.keyBase) {
    case vsg::KEY_Space:
        c.paused.store(!c.paused.load(std::memory_order_relaxed), std::memory_order_relaxed);
        break;
    case vsg::KEY_Period:
        c.singleStep.store(true, std::memory_order_relaxed);
        break;
    case vsg::KEY_Tab: {
        if (vehicles_ <= 0) break;
        int sel = c.selectedVehicle.load(std::memory_order_relaxed);
        sel = shift ? (sel + vehicles_ - 1) % vehicles_ : (sel + 1) % vehicles_;
        c.selectedVehicle.store(sel, std::memory_order_relaxed);
        break;
    }
    case vsg::KEY_c:
    case vsg::KEY_C:
        c.cameraMode.store((c.cameraMode.load(std::memory_order_relaxed) + 1) % 3, std::memory_order_relaxed);
        break;
    case vsg::KEY_Minus:
        c.cameraZoom.store(1.25, std::memory_order_relaxed);
        break;
    case vsg::KEY_Equals:
    case vsg::KEY_Plus:
        c.cameraZoom.store(0.8, std::memory_order_relaxed);
        break;
    case vsg::KEY_l:
    case vsg::KEY_L:
        c.showVehicleList.store(!c.showVehicleList.load(std::memory_order_relaxed), std::memory_order_relaxed);
        break;
    case vsg::KEY_m:
    case vsg::KEY_M:
        c.showMonitor.store(!c.showMonitor.load(std::memory_order_relaxed), std::memory_order_relaxed);
        break;
    case vsg::KEY_Leftbracket:
        c.timeFactor.store(std::max(0.05, c.timeFactor.load(std::memory_order_relaxed) * 0.5), std::memory_order_relaxed);
        break;
    case vsg::KEY_Rightbracket:
        c.timeFactor.store(std::min(64.0, c.timeFactor.load(std::memory_order_relaxed) * 2.0), std::memory_order_relaxed);
        break;
    case vsg::KEY_Escape:
        c.quit.store(true, std::memory_order_relaxed);
        break;
    default:
        return;
    }
    e.handled = true;
}

} // namespace fsim::ui
