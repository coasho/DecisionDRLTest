# render: window, viewer, render graph, ImGui integration point. VSG/Vulkan
# types live here and in world/ui only (design rule 5.2).
add_library(fsim_render STATIC
    Viewer.h
    Viewer.cpp)
add_library(fsim::render ALIAS fsim_render)

target_include_directories(fsim_render PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_render
    PUBLIC fsim::core vsg::vsg vsgXchange::vsgXchange vsgImGui::vsgImGui)
fsim_set_warnings(fsim_render)
fsim_relax_third_party_warnings(fsim_render)
