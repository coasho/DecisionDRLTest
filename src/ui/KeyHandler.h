#pragma once

#include "ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>

namespace fsim::ui {

/// Keyboard bindings for the viewer (design 9.6). Registered after ImGui's
/// event forwarder so text fields keep their keys.
///
///   space        pause / resume        .          single step (while paused)
///   tab / s+tab  next / previous vehicle
///   c            cycle camera mode     - / =      zoom out / in
///   l            toggle vehicle list   m          toggle monitor
///   [ / ]        time factor / 2, x 2  esc        quit
class KeyHandler : public vsg::Inherit<vsg::Visitor, KeyHandler> {
public:
    KeyHandler(std::shared_ptr<ViewerControls> controls, int vehicleCount)
        : controls_(std::move(controls)), vehicles_(vehicleCount) {}

    void apply(vsg::KeyPressEvent& e) override;

private:
    std::shared_ptr<ViewerControls> controls_;
    int vehicles_;
};

} // namespace fsim::ui
