# core: services shared by every module. Knows neither JSBSim nor Vulkan.
add_library(fsim_core STATIC
    Context.h
    Log.h
    Log.cpp
    Module.h
    Module.cpp
    Profiler.h
    Registry.h
    Rng.h
    Span.h
    Units.h)
add_library(fsim::core ALIAS fsim_core)

target_include_directories(fsim_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_core PUBLIC fsim::platform)
fsim_set_warnings(fsim_core)
