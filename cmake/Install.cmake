# Install and package (design 11.3 "Distribution"): the layout an installed
# tree has is the one the build tree already mimics (bin/ + share/flightsim),
# so io::AssetResolver finds models and the JSBSim data either way.
#
#   cmake --build --preset ucrt64-release --target deploy   # runtime DLLs next to the executables
#   cmake --install build/ucrt64-release --prefix dist/flightsim
#   cd build/ucrt64-release && cpack                        # flightsim-<version>-win64.zip
#
# Consumers: find_package(fsim CONFIG) -> fsim::sdk (and fsim::vision when
# the package was built with the viewer stack).
include(GNUInstallDirs)

set(_fsim_export_targets fsim)
set_target_properties(fsim PROPERTIES EXPORT_NAME sdk)
if(TARGET fsim_vision)
    list(APPEND _fsim_export_targets fsim_vision)
    set_target_properties(fsim_vision PROPERTIES EXPORT_NAME vision)
endif()
install(TARGETS ${_fsim_export_targets} EXPORT fsimTargets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/fsim DESTINATION include)

# Executables and tools (no export).
set(_fsim_apps flightsim)
foreach(_t flightsim-viewer tile_prefetch scenario_runner)
    if(TARGET ${_t})
        list(APPEND _fsim_apps ${_t})
    endif()
endforeach()
install(TARGETS ${_fsim_apps} RUNTIME DESTINATION bin)

# Every DLL the deploy target put next to the executables (JSBSim, the
# MinGW runtime, assimp, curl, glslang, ...). Run `deploy` before installing.
install(CODE "file(GLOB _fsim_dlls \"${CMAKE_BINARY_DIR}/bin/*.dll\")
    file(INSTALL \${_fsim_dlls} DESTINATION \"\${CMAKE_INSTALL_PREFIX}/bin\")")

# Data: models + manifests, the JSBSim tree, scenarios, docs.
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/flightsim/ DESTINATION share/flightsim)
install(DIRECTORY ${FSIM_JSBSIM_DATA_DIR}/aircraft ${FSIM_JSBSIM_DATA_DIR}/engine ${FSIM_JSBSIM_DATA_DIR}/systems
        DESTINATION share/flightsim/jsbsim)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/examples/scenarios DESTINATION share/flightsim)
install(FILES ${CMAKE_SOURCE_DIR}/README.md ${CMAKE_SOURCE_DIR}/LICENSE ${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md DESTINATION share/doc/flightsim OPTIONAL)
install(FILES ${CMAKE_SOURCE_DIR}/third_party/jsbsim/COPYING DESTINATION share/doc/flightsim RENAME LICENSE-JSBSim.txt OPTIONAL)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/docs/sdk DESTINATION share/doc/flightsim)

# CMake package.
include(CMakePackageConfigHelpers)
install(EXPORT fsimTargets NAMESPACE fsim:: DESTINATION lib/cmake/fsim)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/fsimConfig.cmake.in
    ${CMAKE_BINARY_DIR}/fsimConfig.cmake
    INSTALL_DESTINATION lib/cmake/fsim
    PATH_VARS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_DATADIR)
write_basic_package_version_file(${CMAKE_BINARY_DIR}/fsimConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_BINARY_DIR}/fsimConfig.cmake ${CMAKE_BINARY_DIR}/fsimConfigVersion.cmake DESTINATION lib/cmake/fsim)

# Zip.
set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_NAME flightsim)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "flightsim-${PROJECT_VERSION}-win64")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_PACKAGE_DIRECTORY ${CMAKE_BINARY_DIR}/package)
include(CPack)
