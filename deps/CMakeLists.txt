# Superbuild for the VulkanSceneGraph stack (design section 11).
#
# vsgXchange and vsgImGui locate VSG with find_package(), so the three are
# built and installed in sequence into one prefix; the main project then finds
# them with CMAKE_PREFIX_PATH=<prefix>. JSBSim is not here: it is part of the
# main build because its data tree lives in the same submodule.
#
#   cmake -S deps --preset deps-ucrt64 && cmake --build build/deps-ucrt64
#
cmake_minimum_required(VERSION 3.25)
project(flightsim-deps NONE)

include(ExternalProject)

set(FSIM_DEPS_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install prefix for the VSG stack")
set(FSIM_DEPS_BUILD_TYPE "Release" CACHE STRING "Build type for the VSG stack")
set(FSIM_DEPS_SHARED OFF CACHE BOOL "Build the VSG stack as shared libraries")

set(_third_party "${CMAKE_CURRENT_SOURCE_DIR}/../third_party")

# Arguments every sub-build shares (toolchain, prefix, generator).
set(_common_args
    -G "${CMAKE_GENERATOR}"
    -DCMAKE_BUILD_TYPE=${FSIM_DEPS_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${FSIM_DEPS_INSTALL_PREFIX}
    -DCMAKE_PREFIX_PATH=${FSIM_DEPS_INSTALL_PREFIX}
    -DBUILD_SHARED_LIBS=${FSIM_DEPS_SHARED}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
if(CMAKE_TOOLCHAIN_FILE)
    list(APPEND _common_args -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
endif()
if(CMAKE_MAKE_PROGRAM)
    list(APPEND _common_args -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
endif()

ExternalProject_Add(vsg
    SOURCE_DIR ${_third_party}/vsg
    CMAKE_ARGS ${_common_args}
        -DVSG_SUPPORTS_ShaderCompiler=ON      # runtime GLSL -> SPIR-V via glslang (developer builds)
        -DVSG_SUPPORTS_ShaderOptimizer=OFF
        -DVSG_SUPPORTS_Windowing=ON
        -DBUILD_TESTING=OFF
    BUILD_ALWAYS OFF)

ExternalProject_Add(vsgXchange
    SOURCE_DIR ${_third_party}/vsgXchange
    DEPENDS vsg
    CMAKE_ARGS ${_common_args}
        -DBUILD_vsgXchange_applications=OFF   # no vsgconv binary
    BUILD_ALWAYS OFF)

ExternalProject_Add(vsgImGui
    SOURCE_DIR ${_third_party}/vsgImGui
    DEPENDS vsg
    CMAKE_ARGS ${_common_args}
    BUILD_ALWAYS OFF)
