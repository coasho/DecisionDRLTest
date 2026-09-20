# effects: abstract vehicle simulation effects (design 9.5).
add_library(fsim_effects STATIC
    Effect.h
    Effect.cpp
    Builtin.h
    Builtin.cpp)
add_library(fsim::effects ALIAS fsim_effects)
target_include_directories(fsim_effects PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_effects PUBLIC fsim::sim fsim::core)
fsim_set_warnings(fsim_effects)
