# A trainer built against an installed package: find_package(fsim) only.
#   cmake -S tests/package_consumer -B build/consumer -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ucrt64.cmake -Dfsim_DIR=dist/flightsim/lib/cmake/fsim
cmake_minimum_required(VERSION 3.25)
project(fsim_consumer CXX)
find_package(fsim CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE fsim::sdk)
target_compile_features(consumer PRIVATE cxx_std_17)
if(TARGET fsim::vision)
    target_link_libraries(consumer PRIVATE fsim::vision)
    target_compile_definitions(consumer PRIVATE HAVE_FSIM_VISION)
endif()
# Run from the package's bin/ so the DLLs and share/ are found.
add_custom_target(run COMMAND $<TARGET_FILE:consumer> WORKING_DIRECTORY ${fsim_BIN_DIR} DEPENDS consumer)
