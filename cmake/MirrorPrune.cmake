# Script mode: cmake -DSRC=<dir> -DDEST=<dir> -P MirrorPrune.cmake
#
# Remove from DEST every file that is no longer in SRC. install(DIRECTORY)
# copies what is new or changed but never deletes, so without this a package
# built after the map plan was tightened keeps every tile the plan dropped -
# and the budget it was built to stops being true. Paired with install() it
# makes DEST a mirror of SRC while still copying only what changed.
cmake_minimum_required(VERSION 3.25)
if(NOT IS_DIRECTORY "${DEST}")
    return()
endif()
file(GLOB_RECURSE _dest LIST_DIRECTORIES false RELATIVE "${DEST}" "${DEST}/*")
set(_removed 0)
foreach(_f IN LISTS _dest)
    if(NOT EXISTS "${SRC}/${_f}")
        file(REMOVE "${DEST}/${_f}")
        math(EXPR _removed "${_removed} + 1")
    endif()
endforeach()
message(STATUS "maps: ${_removed} file(s) no longer in ${SRC} removed from the package")
