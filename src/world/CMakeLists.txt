# world: scene composition - Earth tiles, vehicle visuals, cameras.
add_library(fsim_world STATIC
    Earth.h
    Earth.cpp
    ElevatedTile.h
    ElevatedTile.cpp
    ElevationUpsampler.h
    ElevationUpsampler.cpp
    OfflineTiles.h
    OfflineTiles.cpp
    VehicleVisuals.h
    VehicleVisuals.cpp
    Exhaust.h
    Exhaust.cpp
    Airflow.h
    Airflow.cpp
    CameraController.h
    Atmosphere.h
    Atmosphere.cpp
    CameraController.cpp
    Scattering.h
    Scattering.cpp
    Frames.h
    Interpolator.h
    Interpolator.cpp
    FlatGeometry.h
    FlatGeometry.cpp
    Sky.h
    Sky.cpp
    SkyDome.h
    SkyDome.cpp
    Trails.h
    Trails.cpp
    Terrain.h
    Terrain.cpp)
add_library(fsim::world ALIAS fsim_world)

target_include_directories(fsim_world PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_world PUBLIC fsim::sim fsim::render fsim::core vsg::vsg)
fsim_set_warnings(fsim_world)
fsim_relax_third_party_warnings(fsim_world)
