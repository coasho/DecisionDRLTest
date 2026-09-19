# io: asset resolution, configuration, tile cache (later), recordings (later).
add_library(fsim_io STATIC
    AssetResolver.h
    AssetResolver.cpp)
add_library(fsim::io ALIAS fsim_io)

target_include_directories(fsim_io PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_io PUBLIC fsim::core)
target_compile_definitions(fsim_io PRIVATE FSIM_JSBSIM_DATA_DIR="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_io)
