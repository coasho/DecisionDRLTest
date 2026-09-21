# core: services shared by every module. Knows neither JSBSim nor Vulkan.
add_library(fsim_core STATIC
    Context.h
    Geodesy.h
    Json.h
    Json.cpp
    Time.h
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

target_include_directories(fsim_core PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(fsim_core PUBLIC fsim::platform)
# Internal modules are linked into fsim.dll: public classes they define are exported.
target_compile_definitions(fsim_core PUBLIC FSIM_BUILDING)
fsim_set_warnings(fsim_core)
