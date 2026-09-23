# Distribution (design 11.3). Five things are built here and none of them
# belongs in the same directory as another:
#
#   viewer    the visualisation application, standalone - executable, every
#             DLL it needs, its configuration and its map assets
#   sdk       fsim.dll, the headers and the CMake package a trainer links
#   python    the Python SDK: the fsim package, and a wheel of it
#             (python/CMakeLists.txt)
#   tools     the headless command-line application
#   examples  demo programs written against the SDK
#
# Each is an install component, so one set of rules produces separate trees:
#
#   cmake --build --preset ucrt64-release --target dist
#       -> dist/viewer, dist/sdk, dist/python, dist/tools, dist/examples
#
#   cmake --install build/ucrt64-release --component viewer --prefix <dir>
#   cd build/ucrt64-release && cpack       # flightsim-<version>-win64.zip (all of it)
#
# Test executables are in neither: they live in build/<preset>/tests and are
# not installed at all.
#
# Consumers: find_package(fsim CONFIG) -> fsim::sdk (and fsim::vision when
# the package was built with the viewer stack).
include(GNUInstallDirs)

# Resolving "which DLLs does this need" is the same walk the deploy target
# makes; both call the one function so the two can never disagree.
set(_fsim_dep_dirs "${FSIM_UCRT64_ROOT}/bin" "${CMAKE_BINARY_DIR}/bin")
set(_fsim_objdump "${FSIM_UCRT64_ROOT}/bin/objdump.exe")
set(_fsim_runtime_deps_module "${CMAKE_SOURCE_DIR}/cmake/RuntimeDeps.cmake")

# fsim_install_runtime_deps(<component> <exe-name>...)
# Stages the DLLs beside executables this component has already installed into
# bin/. Declared after the install(TARGETS) it depends on, because install
# rules run in the order they are written and it reads the installed files.
function(fsim_install_runtime_deps component)
    set(_names "")
    foreach(_n ${ARGN})
        list(APPEND _names "\${CMAKE_INSTALL_PREFIX}/bin/${_n}")
    endforeach()
    install(CODE "
        include(\"${_fsim_runtime_deps_module}\")
        fsim_copy_runtime_deps(
            EXES ${_names}
            SEARCH ${_fsim_dep_dirs}
            OBJDUMP \"${_fsim_objdump}\"
            DEST \"\${CMAKE_INSTALL_PREFIX}/bin\"
            LABEL \"package ${component}\")"
        COMPONENT ${component})
endfunction()

# ---------------------------------------------------------------- sdk
set(_fsim_export_targets fsim)
set_target_properties(fsim PROPERTIES EXPORT_NAME sdk)
if(TARGET fsim_vision)
    list(APPEND _fsim_export_targets fsim_vision)
    set_target_properties(fsim_vision PROPERTIES EXPORT_NAME vision)
endif()
install(TARGETS ${_fsim_export_targets} EXPORT fsimTargets
    RUNTIME DESTINATION bin COMPONENT sdk
    LIBRARY DESTINATION lib COMPONENT sdk
    ARCHIVE DESTINATION lib COMPONENT sdk)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/fsim DESTINATION include COMPONENT sdk)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/docs/sdk DESTINATION share/doc/flightsim COMPONENT sdk)
set(_fsim_sdk_runtime "libfsim.dll")
if(TARGET fsim_vision)
    list(APPEND _fsim_sdk_runtime "libfsim_vision.dll")
endif()
fsim_install_runtime_deps(sdk ${_fsim_sdk_runtime})

# Aircraft designed with tools/hangar (aircraft/<name>/): their JSBSim files
# and model, to share/flightsim/aircraft/<name> (cmake/StageDesigns.cmake).
function(fsim_install_designs component)
    install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -DSRC=${CMAKE_SOURCE_DIR}/aircraft
                                  \"-DDEST=\${CMAKE_INSTALL_PREFIX}/share/flightsim/aircraft\"
                                  -P ${CMAKE_SOURCE_DIR}/cmake/StageDesigns.cmake)" COMPONENT ${component})
endfunction()

# ---------------------------------------------------------------- viewer
if(TARGET flightsim-viewer)
    set(_fsim_viewer_targets flightsim-viewer)
    set(_fsim_viewer_runtime "flightsim-viewer.exe")
    if(TARGET tile_prefetch)
        # Shipped with the viewer on purpose: it is how the map assets in
        # maps/ are filled, so the package can carry its own terrain.
        list(APPEND _fsim_viewer_targets tile_prefetch)
        list(APPEND _fsim_viewer_runtime "tile_prefetch.exe")
    endif()
    install(TARGETS ${_fsim_viewer_targets} RUNTIME DESTINATION bin COMPONENT viewer)
    fsim_install_runtime_deps(viewer ${_fsim_viewer_runtime})

    # Assets, found at runtime through io::AssetResolver's <exe>/../share/flightsim.
    install(DIRECTORY ${CMAKE_BINARY_DIR}/share/flightsim/ DESTINATION share/flightsim COMPONENT viewer)
    install(DIRECTORY ${FSIM_JSBSIM_DATA_DIR}/aircraft ${FSIM_JSBSIM_DATA_DIR}/engine ${FSIM_JSBSIM_DATA_DIR}/systems
            DESTINATION share/flightsim/jsbsim COMPONENT viewer)
    fsim_install_designs(viewer)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/examples/scenarios DESTINATION share/flightsim COMPONENT viewer)

    # Configuration, read from <exe>/../config/viewer.json in both the build
    # tree and the package, so the two layouts behave identically.
    # Generated, not copied: the packaged settings differ from the development
    # ones in three values and nothing else (see cmake/PackageConfig.cmake).
    install(CODE "include(\"${CMAKE_SOURCE_DIR}/cmake/PackageConfig.cmake\")
        file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/config\")
        fsim_write_packaged_config(\"${CMAKE_SOURCE_DIR}/assets/config/viewer.json\"
                                   \"\${CMAKE_INSTALL_PREFIX}/config/viewer.json\")"
        COMPONENT viewer)
    install(FILES ${CMAKE_SOURCE_DIR}/assets/config/offline-map-plan.json DESTINATION config COMPONENT viewer)
    install(FILES ${CMAKE_SOURCE_DIR}/assets/config/maps.README.md DESTINATION maps RENAME README.md COMPONENT viewer)
    install(FILES ${CMAKE_SOURCE_DIR}/assets/config/run-viewer.cmd DESTINATION . COMPONENT viewer)

    # The map assets themselves. A distributed viewer never downloads a tile,
    # so whatever it is going to draw has to be in the package: assets/maps is
    # copied in whole. It is gigabytes, and it is the reason `dist` does not
    # wipe the viewer's maps/ before installing - see the dist target below.
    if(EXISTS ${CMAKE_SOURCE_DIR}/assets/maps)
        install(DIRECTORY ${CMAKE_SOURCE_DIR}/assets/maps/ DESTINATION maps COMPONENT viewer)
    else()
        install(CODE "message(WARNING \"assets/maps is empty - run fetch-maps.cmd, or the package has no terrain\")"
                COMPONENT viewer)
    endif()
    install(FILES ${CMAKE_SOURCE_DIR}/README.md ${CMAKE_SOURCE_DIR}/LICENSE ${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md
            DESTINATION share/doc/flightsim COMPONENT viewer OPTIONAL)
    install(FILES ${CMAKE_SOURCE_DIR}/third_party/jsbsim/COPYING
            DESTINATION share/doc/flightsim RENAME LICENSE-JSBSim.txt COMPONENT viewer OPTIONAL)
endif()

# ---------------------------------------------------------------- tools
install(TARGETS flightsim RUNTIME DESTINATION bin COMPONENT tools)
install(DIRECTORY ${FSIM_JSBSIM_DATA_DIR}/aircraft ${FSIM_JSBSIM_DATA_DIR}/engine ${FSIM_JSBSIM_DATA_DIR}/systems
        DESTINATION share/flightsim/jsbsim COMPONENT tools)
fsim_install_designs(tools)
fsim_install_runtime_deps(tools "flightsim.exe")

# ---------------------------------------------------------------- examples
set(_fsim_example_targets "")
set(_fsim_example_runtime "")
foreach(_t minimal_trainer multi_level_control ppo_trainer scenario_runner udp_peer vision_capture)
    if(TARGET ${_t})
        list(APPEND _fsim_example_targets ${_t})
        list(APPEND _fsim_example_runtime "${_t}.exe")
    endif()
endforeach()
if(_fsim_example_targets)
    install(TARGETS ${_fsim_example_targets} RUNTIME DESTINATION bin COMPONENT examples)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/examples/scenarios DESTINATION share/flightsim COMPONENT examples)
    fsim_install_runtime_deps(examples ${_fsim_example_runtime})
endif()

# ---------------------------------------------------------------- CMake package
include(CMakePackageConfigHelpers)
install(EXPORT fsimTargets NAMESPACE fsim:: DESTINATION lib/cmake/fsim COMPONENT sdk)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/fsimConfig.cmake.in
    ${CMAKE_BINARY_DIR}/fsimConfig.cmake
    INSTALL_DESTINATION lib/cmake/fsim
    PATH_VARS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_DATADIR)
write_basic_package_version_file(${CMAKE_BINARY_DIR}/fsimConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_BINARY_DIR}/fsimConfig.cmake ${CMAKE_BINARY_DIR}/fsimConfigVersion.cmake
        DESTINATION lib/cmake/fsim COMPONENT sdk)

# ---------------------------------------------------------------- dist/
# One target that writes every component into its own directory under dist/.
set(_fsim_components sdk tools examples)
if(TARGET flightsim-viewer)
    list(APPEND _fsim_components viewer)
endif()
if(TARGET fsim_python)
    list(APPEND _fsim_components python)
endif()
set(_fsim_package_commands "")
foreach(_c ${_fsim_components})
    # Clear everything but maps/: it is gigabytes of tiles that install() will
    # skip if they are already identical, and deleting it first would turn
    # every package build into a full re-copy.
    list(APPEND _fsim_package_commands
        COMMAND ${CMAKE_COMMAND} -DDIR=${CMAKE_BINARY_DIR}/dist/${_c} -DKEEP=maps
                -P ${CMAKE_SOURCE_DIR}/cmake/CleanDist.cmake)
    if(_c STREQUAL "viewer")
        # Mirror, not accumulate: drop tiles the map plan no longer has.
        list(APPEND _fsim_package_commands
            COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_SOURCE_DIR}/assets/maps
                    -DDEST=${CMAKE_BINARY_DIR}/dist/viewer/maps -DKEEP=README.md
                    -P ${CMAKE_SOURCE_DIR}/cmake/MirrorPrune.cmake)
    endif()
    list(APPEND _fsim_package_commands
        COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR}
                --component ${_c} --prefix ${CMAKE_BINARY_DIR}/dist/${_c})
endforeach()
# Named `dist` rather than `package`, which CPack reserves.
add_custom_target(dist
    ${_fsim_package_commands}
    COMMENT "Packaging into dist/: ${_fsim_components}"
    VERBATIM)
add_dependencies(dist deploy)
if(TARGET python_package)
    add_dependencies(dist python_package)
endif()

# ---------------------------------------------------------------- zip
set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_NAME flightsim)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "flightsim-${PROJECT_VERSION}-win64")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_PACKAGE_DIRECTORY ${CMAKE_BINARY_DIR}/package)
include(CPack)
