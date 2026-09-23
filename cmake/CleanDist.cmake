# Script mode: cmake -DDIR=<dir> [-DKEEP=<name>] -P CleanDist.cmake
#
# Empty a package directory before it is installed into again, except for one
# child: the viewer's maps/, gigabytes that install() copies only where they
# changed. Everything else goes, including what no install ever put there: a
# package that has been run in place collects whatever the run left behind -
# the NVIDIA driver writes "NVIDIA Corporation/umdlogs" into the working
# directory - and none of that belongs in what is shipped.
cmake_minimum_required(VERSION 3.25)
if(NOT IS_DIRECTORY "${DIR}")
    return()
endif()
file(GLOB _children LIST_DIRECTORIES true "${DIR}/*")
foreach(_child IN LISTS _children)
    get_filename_component(_name "${_child}" NAME)
    if(NOT _name STREQUAL "${KEEP}")
        file(REMOVE_RECURSE "${_child}")
    endif()
endforeach()
