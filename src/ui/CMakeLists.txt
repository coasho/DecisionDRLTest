# ui: Dear ImGui layers drawn inside the VSG frame (design 9.5).
add_library(fsim_ui STATIC
    ViewerControls.h
    MonitorGui.h
    MonitorGui.cpp
    KeyHandler.h
    KeyHandler.cpp)
add_library(fsim::ui ALIAS fsim_ui)

target_include_directories(fsim_ui PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_ui PUBLIC fsim::sim fsim::render fsim::world fsim::core vsg::vsg vsgImGui::vsgImGui)
fsim_set_warnings(fsim_ui)
fsim_relax_third_party_warnings(fsim_ui)
