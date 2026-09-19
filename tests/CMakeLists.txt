include(Catch)

add_executable(fsim_tests
    test_units.cpp
    test_rng.cpp
    test_registry.cpp
    test_vehicle_pool.cpp)

target_link_libraries(fsim_tests PRIVATE fsim::sim fsim::io fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_tests)

add_custom_command(TARGET fsim_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_tests>)

catch_discover_tests(fsim_tests)
