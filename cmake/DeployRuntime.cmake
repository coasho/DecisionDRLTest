# Script mode: cmake -DEXES=<a;b> -DSEARCH=<dir> -DOBJDUMP=<objdump> -P DeployRuntime.cmake
# Copies every non-system DLL the executables depend on (transitively) next to
# them, so bin/ runs without ucrt64/bin on PATH (design 11.3 "Distribution").
cmake_minimum_required(VERSION 3.25)
set(CMAKE_OBJDUMP "${OBJDUMP}")
file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES ${EXES}
    RESOLVED_DEPENDENCIES_VAR resolved
    UNRESOLVED_DEPENDENCIES_VAR unresolved
    CONFLICTING_DEPENDENCIES_PREFIX conflicts
    DIRECTORIES ${SEARCH}
    PRE_EXCLUDE_REGEXES "^api-ms-.*" "^ext-ms-.*"
    # Keep only what lives in the toolchain's bin directory; everything else
    # (Windows system DLLs, driver DLLs) must come from the target machine.
    POST_INCLUDE_REGEXES "^${SEARCH}/.*"
    POST_EXCLUDE_REGEXES ".*")
list(GET EXES 0 first)
get_filename_component(dest "${first}" DIRECTORY)
set(copied 0)
foreach(dll ${resolved})
    get_filename_component(name "${dll}" NAME)
    if(NOT EXISTS "${dest}/${name}")
        file(COPY "${dll}" DESTINATION "${dest}")
        math(EXPR copied "${copied}+1")
    endif()
endforeach()
list(LENGTH resolved total)
message(STATUS "deploy: ${total} runtime DLL(s) resolved, ${copied} copied to ${dest}")
# Unresolved names are Windows components not present on this machine's
# search path (optional OS features); harmless for the executables.
if(unresolved)
    message(STATUS "deploy: unresolved (system) names ignored: ${unresolved}")
endif()
