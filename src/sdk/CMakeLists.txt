# fsim.dll: the public SDK (C++ pimpl class + versioned C ABI). Internal
# static libraries are linked in; only fsim/*.h symbols are exported.
add_library(fsim SHARED fsim.cpp World.cpp fsim_world_c.cpp Scenario.cpp Recording.cpp LastError.h)
add_library(fsim::sdk ALIAS fsim)

target_include_directories(fsim
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_SOURCE_DIR}/src)
# Whole-archive: built-in controllers/effects are reached only through the
# registry or by trainer code, so nothing in the DLL would otherwise pull
# their objects in and export them.
target_link_libraries(fsim PRIVATE
    "$<LINK_LIBRARY:WHOLE_ARCHIVE,fsim::env,fsim::session,fsim::control,fsim::effects,fsim::comm,fsim::ipc>"
    fsim::sim fsim::io fsim::core fsim::platform)
target_compile_definitions(fsim PRIVATE FSIM_BUILDING FSIM_VERSION_STRING="${PROJECT_VERSION}")
set_target_properties(fsim PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    OUTPUT_NAME fsim)
fsim_set_warnings(fsim)

# JSBSim.dll next to fsim.dll for consumers built in-tree.
add_custom_command(TARGET fsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim>)
