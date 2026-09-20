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
///   r            reset view offset     mouse      left drag orbit, wheel / right drag distance
///   l            toggle vehicle list   m          toggle monitor
///   n            toggle name labels    t          toggle trails
///   [ / ]        time factor / 2, x 2  esc        quit
class KeyHandler : public vsg::Inherit<vsg::Visitor, KeyHandler> {
public:
    explicit KeyHandler(std::shared_ptr<ViewerControls> controls) : controls_(std::move(controls)) {}

    void apply(vsg::KeyPressEvent& e) override;

private:
    std::shared_ptr<ViewerControls> controls_;
};

} // namespace fsim::ui
