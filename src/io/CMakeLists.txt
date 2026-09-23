# io: asset resolution, configuration, tile cache (later), recordings (later).
add_library(fsim_io STATIC
    AssetResolver.h
    AssetResolver.cpp
    TerrainTiles.h
    TerrainTiles.cpp)
add_library(fsim::io ALIAS fsim_io)

target_include_directories(fsim_io PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(fsim_io SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/stb)
target_link_libraries(fsim_io PUBLIC fsim::core)
target_compile_definitions(fsim_io PRIVATE FSIM_JSBSIM_DATA_DIR="${FSIM_JSBSIM_DATA_DIR}"
                                            FSIM_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft")
fsim_set_warnings(fsim_io)
