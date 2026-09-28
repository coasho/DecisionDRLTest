# control: multi-level control stack (design 9.3). Depends on sim for the
# state/input types only; no JSBSim.
add_library(fsim_control STATIC
    ControlStack.h
    ControlStack.cpp
    Runtime.h
    Catalog.h
    Catalog.cpp
    Features.h
    Features.cpp
    CapabilityHost.h
    CapabilityHost.cpp
    Profile.h
    Profile.cpp
    Protection.h
    Protection.cpp
    Laws.h
    Laws.cpp
    Adapter.h
    Adapter.cpp
    Registry.h
    Registry.cpp
    Pid.h
    Builtin.h
    Builtin.cpp
    Rotor.cpp
    Atmosphere.h
    Route.h
    Route.cpp
    Guidance.cpp
    Tasks.cpp
    Reports.cpp
    PerformanceTables.cpp
    Names.cpp
    Endurance.cpp
    Terrain.cpp)
add_library(fsim::control ALIAS fsim_control)

target_include_directories(fsim_control PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_control PUBLIC fsim::sim fsim::core)
fsim_set_warnings(fsim_control)
