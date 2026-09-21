include(Catch)

add_executable(fsim_tests
    test_units.cpp
    test_rng.cpp
    test_registry.cpp
    test_vehicle_pool.cpp
    test_vecenv.cpp
    test_control.cpp
    test_world.cpp
    test_terrain.cpp
    test_scenario.cpp)

target_link_libraries(fsim_tests PRIVATE fsim::env fsim::session fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_tests)

add_custom_command(TARGET fsim_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_tests>)

catch_discover_tests(fsim_tests)

# The public C ABI, consumed from a plain C translation unit against fsim.dll.
add_executable(fsim_c_abi_test test_c_abi.c)
target_link_libraries(fsim_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME c_abi COMMAND fsim_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
