# Third-party dependencies.
#
# Headless milestones (M0-M2) need only JSBSim, which ships as a git submodule
# because the platform also needs its data tree (aircraft/, engine/, systems/).
# The viewer milestones add VSG, vsgXchange and vsgImGui through vcpkg (see
# design section 11). Catch2 is fetched for tests only.

include(FetchContent)

# ---------------------------------------------------------------------------
# JSBSim (LGPL-2.1) - built as a shared library, one instance per vehicle.
# ---------------------------------------------------------------------------
set(FSIM_JSBSIM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsbsim)
if(NOT EXISTS ${FSIM_JSBSIM_SOURCE_DIR}/src/FGFDMExec.h)
    message(FATAL_ERROR
        "JSBSim submodule is missing. Run: git submodule update --init --recursive")
endif()

# Only libJSBSim's own subtree is added; JSBSim's top-level CMakeLists pulls in
# docs, Python, Julia, MATLAB and utilities we do not want. src/CMakeLists.txt
# is self-contained apart from PROJECT_VERSION, which it uses for a version string.
set(_fsim_saved_project_version ${PROJECT_VERSION})
set(PROJECT_VERSION 1.3.1)
set(LIBRARY_VERSION 1.3.1)   # normally set by JSBSim's top-level CMakeLists
set(LIBRARY_SOVERSION 1)
set(_fsim_saved_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)                 # JSBSim as DLL (LGPL relinking, design 3)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${FSIM_JSBSIM_SOURCE_DIR}/src ${CMAKE_BINARY_DIR}/third_party/jsbsim EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS ${_fsim_saved_shared})
set(PROJECT_VERSION ${_fsim_saved_project_version})

if(MSVC)
    # JSBSim is not warning-clean under /W4; keep its noise out of our build log.
    target_compile_options(libJSBSim PRIVATE /W1 /wd4251 /wd4275)
    foreach(_obj Atmosphere FlightControl GeographicLib Init InputOutput IOStreams
                 Magvar Math Misc Models Properties Propulsion Xml)
        if(TARGET ${_obj})
            target_compile_options(${_obj} PRIVATE /W1)
        endif()
    endforeach()
endif()

# Consumers see JSBSim headers as system headers so their warnings do not
# trip /WX in platform code (CMake >= 3.25).
set_target_properties(libJSBSim PROPERTIES SYSTEM ON)

add_library(fsim::jsbsim ALIAS libJSBSim)

# ---------------------------------------------------------------------------
# Catch2 (BSL-1.0) - tests only.
# ---------------------------------------------------------------------------
if(FSIM_BUILD_TESTS)
    FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# ---------------------------------------------------------------------------
# VulkanSceneGraph stack (MIT) - viewer builds only. Built and installed by the
# superbuild in deps/ (see deps/CMakeLists.txt); found through CMAKE_PREFIX_PATH.
# ---------------------------------------------------------------------------
if(FSIM_WITH_RENDER)
    find_package(vsg 1.1.14 REQUIRED)
    find_package(vsgXchange REQUIRED)
    find_package(vsgImGui REQUIRED)
    message(STATUS "flightsim: viewer enabled (vsg ${vsg_VERSION})")
else()
    message(STATUS "flightsim: headless build (FSIM_WITH_RENDER=OFF)")
endif()
