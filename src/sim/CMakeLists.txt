# sim: flight models, the vehicle pool and the ground provider. The only
# module that includes JSBSim headers (design rule 5.2), and only in .cpp files
# plus JsbsimModel.h.
add_library(fsim_sim STATIC
    ControlInputs.h
    FlightModel.h
    FlightModel.cpp
    GroundProvider.h
    JsbsimLog.h
    JsbsimLog.cpp
    JsbsimModel.h
    JsbsimModel.cpp
    SimRunner.h
    SimRunner.cpp
    SnapshotBuffer.h
    VehiclePool.h
    VehiclePool.cpp
    VehicleState.h)
add_library(fsim::sim ALIAS fsim_sim)

target_include_directories(fsim_sim PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_sim
    PUBLIC  fsim::core fsim::io
    PRIVATE fsim::jsbsim)
fsim_set_warnings(fsim_sim)

if(MSVC)
    # JSBSim headers trigger DLL-interface warnings for std members; not ours to fix.
    target_compile_options(fsim_sim PRIVATE /wd4251 /wd4275)
endif()
