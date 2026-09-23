# cmake -DSRC=<repo>/aircraft -DDEST=<share/flightsim/aircraft> -P StageDesigns.cmake
#
# Copies what tools/hangar builds for each design - aircraft/<name>/<name>.xml,
# its Engines/ and the <name>.glb model with its manifest - to DEST/<name>,
# where io::AssetResolver::findAircraft finds it in a package. The design's
# own files (the .toml, out/) stay behind. Run at build or install time, so a
# new design needs no reconfigure.
if(NOT SRC OR NOT DEST)
    message(FATAL_ERROR "StageDesigns.cmake: SRC and DEST are required")
endif()
file(GLOB _designs LIST_DIRECTORIES true "${SRC}/*")
set(_n 0)
foreach(_d IN LISTS _designs)
    get_filename_component(_name "${_d}" NAME)
    if(NOT EXISTS "${_d}/${_name}.xml")
        continue()
    endif()
    set(_to "${DEST}/${_name}")
    file(MAKE_DIRECTORY "${_to}")
    file(COPY "${_d}/${_name}.xml" DESTINATION "${_to}")
    if(EXISTS "${_d}/Engines")
        file(COPY "${_d}/Engines" DESTINATION "${_to}")
    endif()
    foreach(_f "${_name}.glb" "${_name}.glb.manifest")
        if(EXISTS "${_d}/${_f}")
            file(COPY "${_d}/${_f}" DESTINATION "${_to}")
        endif()
    endforeach()
    math(EXPR _n "${_n} + 1")
endforeach()
message(STATUS "designed aircraft: ${_n} staged in ${DEST}")
