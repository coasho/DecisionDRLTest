# Example trainers are demo code against the public SDK. They are kept out of
# bin/ (the applications) and out of the SDK package, so that what a consumer
# gets and what merely demonstrates it are never the same directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)

add_subdirectory(minimal_trainer)
add_subdirectory(multi_level_control)
add_subdirectory(ppo_trainer)
add_subdirectory(scenario_runner)
add_subdirectory(udp_peer)
if(FSIM_WITH_RENDER)
    add_subdirectory(vision_capture)
endif()
