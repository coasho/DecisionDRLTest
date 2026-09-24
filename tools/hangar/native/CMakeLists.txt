# meshkit: hangar's mesher (signed-distance scenes to closed, simplified
# triangle meshes). A plain C DLL that hangar loads with ctypes; it sits in
# the staged Python directory, which hangar runs with on its path.
add_library(hangar_meshkit SHARED
    meshkit.cpp
    sdf.cpp
    polygonize.cpp
    decimate.cpp)
target_link_libraries(hangar_meshkit PRIVATE fsim::core)
target_include_directories(hangar_meshkit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
fsim_set_warnings(hangar_meshkit)
set_target_properties(hangar_meshkit PROPERTIES
    PREFIX ""
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
if(NOT MSVC)
    # a DLL that ctypes can load on its own: libstdc++, libgcc and winpthread linked in
    target_link_options(hangar_meshkit PRIVATE -static)
endif()
