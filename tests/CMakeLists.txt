include(Catch)

# Test executables are not part of any shipped package, so they are kept out of
# bin/ (which holds the applications) and out of the dist/ trees entirely. The
# runtime DLLs they need are staged beside them by the `deploy` target.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

# Enumerate cases at test time rather than at build time: the executables are
# linked before `deploy` has staged the DLLs beside them, so running them to
# list their tests during the build would fail.
set(FSIM_CATCH_DISCOVERY DISCOVERY_MODE PRE_TEST)

add_executable(fsim_tests
    test_units.cpp
    test_rng.cpp
    test_registry.cpp
    test_vehicle_pool.cpp
    test_vecenv.cpp
    test_capabilities.cpp
    test_control.cpp
    test_control_compat.cpp
    test_profile.cpp
    test_world.cpp
    test_terrain.cpp
    test_scenario.cpp
    test_comm.cpp)

target_link_libraries(fsim_tests PRIVATE fsim::env fsim::session fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}"
                                                FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft"
                                                FSIM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data")
fsim_set_warnings(fsim_tests)

add_custom_command(TARGET fsim_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_tests>)

catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY})

# The control architecture's measurements (docs/control-architecture.md,
# section 12): timings, heap allocations and trajectory digests. The
# allocation count is a test (none in the steady state); the rest is run by
# hand at each step.
add_executable(fsim_control_bench bench_control.cpp)
target_link_libraries(fsim_control_bench PRIVATE fsim::session fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
target_compile_definitions(fsim_control_bench PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_control_bench)
add_custom_command(TARGET fsim_control_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_control_bench>)
add_test(NAME control_alloc COMMAND fsim_control_bench alloc)

# The public C++ SDK against fsim.dll (what a trainer links).
add_executable(fsim_sdk_tests test_sdk.cpp)
target_link_libraries(fsim_sdk_tests PRIVATE fsim::sdk Catch2::Catch2WithMain)
add_custom_command(TARGET fsim_sdk_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim> $<TARGET_FILE_DIR:fsim_sdk_tests>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_sdk_tests>)
target_compile_definitions(fsim_sdk_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_sdk_tests)
catch_discover_tests(fsim_sdk_tests ${FSIM_CATCH_DISCOVERY})

# The public C ABI, consumed from a plain C translation unit against fsim.dll.
add_executable(fsim_c_abi_test test_c_abi.c)
target_link_libraries(fsim_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME c_abi COMMAND fsim_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")

# The crash handler writes a minidump (the crasher is expected to die).
add_executable(fsim_crash_test test_crash.cpp)
target_link_libraries(fsim_crash_test PRIVATE fsim::platform)
fsim_set_warnings(fsim_crash_test)
add_test(NAME crash_handler COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:fsim_crash_test> -DDIR=${CMAKE_BINARY_DIR}/crash-test -P ${CMAKE_SOURCE_DIR}/tests/crash_check.cmake)

# The Rust example against the C ABI (when cargo is installed): builds and steps a batch.
find_program(FSIM_CARGO cargo)
find_program(FSIM_RUSTC rustc)
set(FSIM_RUST_GNU OFF)
if(FSIM_CARGO AND FSIM_RUSTC)
    execute_process(COMMAND ${FSIM_RUSTC} -vV OUTPUT_VARIABLE _rustc_info ERROR_QUIET)
    if(_rustc_info MATCHES "host: x86_64-pc-windows-gnu")
        set(FSIM_RUST_GNU ON) # the DLLs are MinGW builds; the MSVC toolchain cannot link their import library
    endif()
endif()
if(FSIM_RUST_GNU)
    add_test(NAME rust_example
             COMMAND ${FSIM_CARGO} run --release --quiet -- vecenv
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/rust_trainer)
    # FSIM_BUILD_DIR gives build.rs bin/ (the DLLs) and lib/ (the import library).
    set_tests_properties(rust_example PROPERTIES ENVIRONMENT "FSIM_BUILD_DIR=${CMAKE_BINARY_DIR}" TIMEOUT 600)
endif()

# Viewer-side terrain decoding (VSG data types; no Vulkan device needed).
if(FSIM_WITH_RENDER)
    add_executable(fsim_render_tests test_terrain_render.cpp test_vehicle_visuals.cpp)
    target_link_libraries(fsim_render_tests PRIVATE fsim::world fsim::core Catch2::Catch2WithMain)
    target_compile_definitions(fsim_render_tests PRIVATE FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft")
    fsim_set_warnings(fsim_render_tests)
    fsim_relax_third_party_warnings(fsim_render_tests)
    catch_discover_tests(fsim_render_tests ${FSIM_CATCH_DISCOVERY})
endif()

# Offscreen cameras through fsim_vision.dll (needs a Vulkan device).
if(FSIM_WITH_RENDER)
    add_executable(fsim_vision_test test_vision.cpp)
    target_link_libraries(fsim_vision_test PRIVATE fsim::sdk fsim::vision)
    add_custom_command(TARGET fsim_vision_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim_vision> $<TARGET_FILE_DIR:fsim_vision_test>)
    fsim_set_warnings(fsim_vision_test)
    add_test(NAME vision COMMAND fsim_vision_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision PROPERTIES SKIP_RETURN_CODE 77) # no Vulkan device (CI runners)
    add_executable(fsim_vision_c_abi_test test_vision_c_abi.c)
    target_link_libraries(fsim_vision_c_abi_test PRIVATE fsim::sdk fsim::vision)
    set_target_properties(fsim_vision_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
    add_test(NAME vision_c_abi COMMAND fsim_vision_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision_c_abi PROPERTIES SKIP_RETURN_CODE 77)
endif()
