cmake_minimum_required(VERSION 3.25)

project(flightsim
    VERSION 0.1.0
    DESCRIPTION "C++ / VulkanSceneGraph / JSBSim flight-simulation platform for RL training"
    LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Options (see docs/FlightSim_System_Architecture_and_Design.md, section 11)
# ---------------------------------------------------------------------------
option(FSIM_WITH_RENDER  "Build the viewer (render/world/ui; requires Vulkan SDK + VSG)" OFF)
option(FSIM_BUILD_TESTS  "Build unit and integration tests"                            ON)
option(FSIM_BUILD_EXAMPLES "Build example trainers (examples/)"                         ON)
option(FSIM_BUILD_TOOLS  "Build command-line tools (tools/tile_prefetch)"               ON)
option(FSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors in platform code"    ON)

# ---------------------------------------------------------------------------
# Global compile settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# One output directory so fsim.exe finds JSBSim.dll next to it.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(MSVC)
    # Dynamic UCRT everywhere (design 11.3); also required for the future C ABI consumers.
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_compile_options(/utf-8 /MP /Zc:__cplusplus /permissive- /EHsc)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# Link-time optimisation for release builds (design 11.2). Off for MinGW:
# GCC LTO and __declspec(dllexport) vtables collide when linking JSBSim.dll.
if(NOT MINGW)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT FSIM_IPO_SUPPORTED OUTPUT FSIM_IPO_MESSAGE)
    if(FSIM_IPO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Warnings)
include(Dependencies)
include(Assets)

# Location of the JSBSim data tree (aircraft/, engine/, systems/) used by default
# when no --jsbsim-root is given. Points at the submodule in development builds
# and at share/jsbsim in installed layouts (resolved at runtime, see io::Paths).
set(FSIM_JSBSIM_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsbsim)

add_subdirectory(src)
if(FSIM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(FSIM_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(FSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Staging the build tree (design 11.3 "Distribution"). Every directory that
# something is run from gets the DLLs it needs, so bin/ and tests/ both work
# without ucrt64/bin on PATH. Part of ALL because a build that produces
# executables which cannot start is not a finished build.
set(_fsim_app_exes "$<TARGET_FILE:flightsim>")
foreach(_t flightsim-viewer tile_prefetch)
    if(TARGET ${_t})
        set(_fsim_app_exes "${_fsim_app_exes};$<TARGET_FILE:${_t}>")
    endif()
endforeach()
add_custom_target(deploy ALL
    COMMAND ${CMAKE_COMMAND}
        "-DEXES=${_fsim_app_exes};$<TARGET_FILE:fsim>"
        "-DSEARCH=${FSIM_UCRT64_ROOT}/bin;${CMAKE_BINARY_DIR}/bin"
        "-DOBJDUMP=${FSIM_UCRT64_ROOT}/bin/objdump.exe"
        "-DDEST=${CMAKE_BINARY_DIR}/bin"
        "-DLABEL=deploy bin"
        -P ${CMAKE_SOURCE_DIR}/cmake/DeployRuntime.cmake
    COMMENT "Staging runtime DLLs in bin/"
    VERBATIM)
add_dependencies(deploy flightsim fsim)
foreach(_t flightsim-viewer tile_prefetch)
    if(TARGET ${_t})
        add_dependencies(deploy ${_t})
    endif()
endforeach()

if(FSIM_BUILD_TESTS)
    set(_fsim_test_exes "")
    foreach(_t fsim_tests fsim_sdk_tests fsim_c_abi_test fsim_crash_test
               fsim_render_tests fsim_vision_test fsim_vision_c_abi_test)
        if(TARGET ${_t})
            set(_fsim_test_exes "${_fsim_test_exes};$<TARGET_FILE:${_t}>")
        endif()
    endforeach()
    add_custom_target(deploy-tests ALL
        COMMAND ${CMAKE_COMMAND}
            "-DEXES=${_fsim_test_exes}"
            "-DSEARCH=${FSIM_UCRT64_ROOT}/bin;${CMAKE_BINARY_DIR}/bin"
            "-DOBJDUMP=${FSIM_UCRT64_ROOT}/bin/objdump.exe"
            "-DDEST=${CMAKE_BINARY_DIR}/tests"
            "-DLABEL=deploy tests"
            -P ${CMAKE_SOURCE_DIR}/cmake/DeployRuntime.cmake
        COMMENT "Staging runtime DLLs in tests/"
        VERBATIM)
    foreach(_t fsim_tests fsim_sdk_tests fsim_c_abi_test fsim_crash_test
               fsim_render_tests fsim_vision_test fsim_vision_c_abi_test)
        if(TARGET ${_t})
            add_dependencies(deploy-tests ${_t})
        endif()
    endforeach()
endif()

include(cmake/Install.cmake)
