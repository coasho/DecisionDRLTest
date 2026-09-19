# world: scene composition - Earth tiles, vehicle visuals, cameras.
add_library(fsim_world STATIC
    Earth.h
    Earth.cpp
    VehicleVisuals.h
    VehicleVisuals.cpp
    CameraController.h
    CameraController.cpp
    Frames.h
    Interpolator.h
    Interpolator.cpp
    Sky.h
    Sky.cpp
    Terrain.h
    Terrain.cpp)
add_library(fsim::world ALIAS fsim_world)

target_include_directories(fsim_world PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_world PUBLIC fsim::sim fsim::render fsim::core vsg::vsg)
fsim_set_warnings(fsim_world)
fsim_relax_third_party_warnings(fsim_world)
