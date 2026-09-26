# control: multi-level control stack (design 9.3). Depends on sim for the
# state/input types only; no JSBSim.
add_library(fsim_control STATIC
    ControlStack.h
    ControlStack.cpp
    Runtime.h
    Catalog.h
    Catalog.cpp
    CapabilityHost.h
    CapabilityHost.cpp
    Profile.h
    Profile.cpp
    Registry.h
    Registry.cpp
    Pid.h
    Builtin.h
    Builtin.cpp)
add_library(fsim::control ALIAS fsim_control)

target_include_directories(fsim_control PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_control PUBLIC fsim::sim fsim::core)
fsim_set_warnings(fsim_control)
