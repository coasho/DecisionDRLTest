# fsim_vision.dll: vehicle cameras rendered offscreen (design 8.4). Separate
# from fsim.dll because it needs Vulkan; links the public SDK for the world
# and the viewer's scene builders for the Earth, sky and vehicle models.
add_library(fsim_vision SHARED Vision.cpp Offscreen.h Offscreen.cpp fsim_vision_c.cpp)
add_library(fsim::vision ALIAS fsim_vision)
target_include_directories(fsim_vision
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(fsim_vision SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/stb)
target_link_libraries(fsim_vision PUBLIC fsim::sdk PRIVATE fsim::world fsim::io fsim::core fsim::platform vsg::vsg vsgXchange::vsgXchange)
target_compile_definitions(fsim_vision PRIVATE FSIM_VISION_BUILDING)
set_target_properties(fsim_vision PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON OUTPUT_NAME fsim_vision)
fsim_set_warnings(fsim_vision)
fsim_relax_third_party_warnings(fsim_vision)
