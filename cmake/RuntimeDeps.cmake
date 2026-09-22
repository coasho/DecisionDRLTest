# One implementation of "which DLLs does this executable actually need, and put
# them here". Two callers: the `deploy` target, which stages the build tree so
# bin/ and tests/ run without ucrt64/bin on PATH, and the install rules, which
# stage each distribution component separately. Having written the same walk
# twice before and watched the copies disagree, it lives in one place.

# Without this, file(GET_RUNTIME_DEPENDENCIES) hands back Windows paths with
# backslashes while POST_INCLUDE_REGEXES below is written with forward slashes,
# so nothing matches and every DLL is silently dropped. The function is defined
# after the policy is set so it carries that setting wherever it is called.
if(POLICY CMP0207)
    cmake_policy(SET CMP0207 NEW)
endif()

# fsim_copy_runtime_deps(EXES <exe...> SEARCH <dir> OBJDUMP <objdump>
#                        DEST <dir> [EXTRA <file...>] [LABEL <text>])
#
# EXES     executables (and DLLs) whose dependencies are resolved transitively.
# SEARCH   directories to resolve dependencies from: the toolchain's bin, and
#          the build tree's bin for our own DLLs (fsim, JSBSim). Anything that
#          resolves under the Windows system root is left for the target
#          machine to provide.
# EXTRA    files copied verbatim - our own DLLs, which live in the build tree
#          and so are not reachable through the search path.
function(fsim_copy_runtime_deps)
    cmake_parse_arguments(A "" "OBJDUMP;DEST;LABEL" "EXES;EXTRA;SEARCH" ${ARGN})
    if(NOT A_DEST)
        list(GET A_EXES 0 _first)
        get_filename_component(A_DEST "${_first}" DIRECTORY)
    endif()
    if(NOT A_LABEL)
        set(A_LABEL "deploy")
    endif()
    file(MAKE_DIRECTORY "${A_DEST}")

    foreach(_extra ${A_EXTRA})
        if(EXISTS "${_extra}")
            file(COPY "${_extra}" DESTINATION "${A_DEST}")
        endif()
    endforeach()

    set(_existing "")
    foreach(_exe ${A_EXES})
        if(EXISTS "${_exe}")
            list(APPEND _existing "${_exe}")
        endif()
    endforeach()
    if(NOT _existing)
        message(STATUS "${A_LABEL}: nothing to resolve")
        return()
    endif()

    set(CMAKE_OBJDUMP "${A_OBJDUMP}")
    file(GET_RUNTIME_DEPENDENCIES
        EXECUTABLES ${_existing}
        RESOLVED_DEPENDENCIES_VAR _resolved
        UNRESOLVED_DEPENDENCIES_VAR _unresolved
        CONFLICTING_DEPENDENCIES_PREFIX _conflicts
        DIRECTORIES ${A_SEARCH}
        PRE_EXCLUDE_REGEXES "^api-ms-.*" "^ext-ms-.*")

    # A name present in more than one search directory is reported as a
    # conflict rather than resolved, and dropping those on the floor is not an
    # option: libstdc++-6, libgcc_s_seh-1 and libwinpthread-1 sit in both the
    # toolchain's bin and our own staged bin, and a package without them exits
    # 127 before reaching main. SEARCH is in priority order, so the earliest
    # directory that has the file wins.
    foreach(_name ${_conflicts_FILENAMES})
        set(_pick "")
        foreach(_dir ${A_SEARCH})
            foreach(_candidate ${_conflicts_${_name}})
                if(NOT _pick)
                    string(FIND "${_candidate}" "${_dir}/" _at)
                    if(_at EQUAL 0)
                        set(_pick "${_candidate}")
                    endif()
                endif()
            endforeach()
        endforeach()
        if(NOT _pick)
            list(GET _conflicts_${_name} 0 _pick)
        endif()
        list(APPEND _resolved "${_pick}")
    endforeach()

    # Everything that is not part of Windows. Selecting *for* the toolchain
    # directory instead looks tidier and is wrong: once a DLL has been staged
    # beside an executable, that copy is what resolution finds, so a run that
    # stages a second directory resolves them all to the first one and matches
    # none of them. Whatever they resolve to, they are ours unless they live
    # under the system root.
    string(TOLOWER "$ENV{SystemRoot}" _winroot)
    if(NOT _winroot)
        set(_winroot "c:/windows")
    endif()
    string(REPLACE "\\" "/" _winroot "${_winroot}")

    set(_copied 0)
    set(_total 0)
    foreach(_dll ${_resolved})
        string(TOLOWER "${_dll}" _lower)
        string(REPLACE "\\" "/" _lower "${_lower}")
        if(_lower MATCHES "^${_winroot}/")
            continue()
        endif()
        math(EXPR _total "${_total}+1")
        get_filename_component(_name "${_dll}" NAME)
        if(NOT EXISTS "${A_DEST}/${_name}")
            file(COPY "${_dll}" DESTINATION "${A_DEST}")
            math(EXPR _copied "${_copied}+1")
        endif()
    endforeach()
    message(STATUS "${A_LABEL}: ${_total} runtime DLL(s) resolved, ${_copied} copied to ${A_DEST}")
    # Unresolved names are Windows components not present on this machine's
    # search path (optional OS features); harmless for the executables.
    if(_unresolved)
        message(STATUS "${A_LABEL}: unresolved (system) names ignored: ${_unresolved}")
    endif()
endfunction()
