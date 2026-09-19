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
option(FSIM_BUILD_TOOLS  "Build offline tools (tile_builder, ...)"                     OFF)
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

# Link-time optimisation for release builds (design 11.2).
include(CheckIPOSupported)
check_ipo_supported(RESULT FSIM_IPO_SUPPORTED OUTPUT FSIM_IPO_MESSAGE)
if(FSIM_IPO_SUPPORTED)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Warnings)
include(Dependencies)

# Location of the JSBSim data tree (aircraft/, engine/, systems/) used by default
# when no --jsbsim-root is given. Points at the submodule in development builds
# and at share/jsbsim in installed layouts (resolved at runtime, see io::Paths).
set(FSIM_JSBSIM_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsbsim)

add_subdirectory(src)

if(FSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
