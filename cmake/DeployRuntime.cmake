# Script mode: cmake -DEXES=<a;b> -DSEARCH=<dir> -DOBJDUMP=<objdump>
#              [-DDEST=<dir>] [-DEXTRA=<a;b>] [-DLABEL=<text>]
#              -P DeployRuntime.cmake
#
# Stages a directory of the build tree so it runs without ucrt64/bin on PATH
# (design 11.3 "Distribution"). The walk itself lives in RuntimeDeps.cmake,
# which the install rules use as well.
cmake_minimum_required(VERSION 3.25)
include("${CMAKE_CURRENT_LIST_DIR}/RuntimeDeps.cmake")

fsim_copy_runtime_deps(
    EXES ${EXES}
    EXTRA ${EXTRA}
    SEARCH "${SEARCH}"
    OBJDUMP "${OBJDUMP}"
    DEST "${DEST}"
    LABEL "${LABEL}")
