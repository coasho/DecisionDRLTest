# env: RL environments over the simulation (design 5, 9.1). Knows nothing
# about rendering.
add_library(fsim_env STATIC
    Observation.h
    Observation.cpp
    Scenario.h
    Task.h
    Task.cpp
    VecEnv.h
    VecEnv.cpp)
add_library(fsim::env ALIAS fsim_env)

target_include_directories(fsim_env PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_env PUBLIC fsim::sim fsim::io fsim::core)
fsim_set_warnings(fsim_env)
