# The Python SDK (docs/sdk/python.md).
#
# fsim._native (and fsim._vision, with the renderer) are C extension modules
# over the C ABI, built with this project's GCC against the stable ABI of a
# python.org or conda CPython: one binary for every interpreter from 3.11 on.
# Not MSYS2's Python - PyTorch, Gymnasium and Stable-Baselines3 live in the
# other one. The package is staged in <build>/python, importable from there
# (PYTHONPATH), and installed as the `python` component: dist/python/fsim and
# a wheel beside it.

set(FSIM_PYTHON_EXECUTABLE "" CACHE FILEPATH "CPython >= 3.11 (python.org or conda) the Python SDK is built for; empty = the py launcher's")

# A usable CPython: 3.11+, with the stable-ABI DLL (python3.dll) beside it.
set(_fsim_py_probe [=[
import os, sys, sysconfig
dll = os.path.join(sys.base_prefix, "python3.dll")
ok = sys.version_info >= (3, 11) and os.path.isfile(dll) and os.path.isfile(os.path.join(sysconfig.get_paths()["include"], "Python.h"))
print("|".join([str(int(ok)), "%d.%d.%d" % sys.version_info[:3], sysconfig.get_paths()["include"], dll, sys.executable]).replace("\\", "/"))
]=])

set(_fsim_py_candidates "")
if(FSIM_PYTHON_EXECUTABLE)
    list(APPEND _fsim_py_candidates "${FSIM_PYTHON_EXECUTABLE}")
else()
    find_program(_fsim_py_launcher NAMES py PATHS "$ENV{SystemRoot}" NO_CACHE)
    if(_fsim_py_launcher)
        execute_process(COMMAND "${_fsim_py_launcher}" -3 -c "import sys; print(sys.executable)"
                        OUTPUT_VARIABLE _exe OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE _rc)
        if(_rc EQUAL 0 AND _exe)
            list(APPEND _fsim_py_candidates "${_exe}")
        endif()
    endif()
endif()

set(_fsim_py_ok FALSE)
foreach(_exe ${_fsim_py_candidates})
    execute_process(COMMAND "${_exe}" -c "${_fsim_py_probe}" OUTPUT_VARIABLE _out OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET RESULT_VARIABLE _rc)
    if(_rc EQUAL 0 AND _out MATCHES "^1\\|")
        string(REPLACE "|" ";" _fields "${_out}")
        list(GET _fields 1 FSIM_PYTHON_VERSION)
        list(GET _fields 2 FSIM_PYTHON_INCLUDE)
        list(GET _fields 3 FSIM_PYTHON3_DLL)
        list(GET _fields 4 FSIM_PYTHON)
        set(_fsim_py_ok TRUE)
        break()
    endif()
endforeach()

if(NOT _fsim_py_ok)
    message(STATUS "Python SDK: skipped - no CPython >= 3.11 with python3.dll found (set FSIM_PYTHON_EXECUTABLE)")
    return()
endif()
message(STATUS "Python SDK: CPython ${FSIM_PYTHON_VERSION} at ${FSIM_PYTHON}")
set(FSIM_PYTHON "${FSIM_PYTHON}" PARENT_SCOPE)

set(FSIM_PY_STAGE ${CMAKE_BINARY_DIR}/python)
set(FSIM_PY_PACKAGE ${FSIM_PY_STAGE}/fsim)

# `fsim python <script>` runs this: the interpreter the SDK was built for,
# with the staged package on its path.
file(WRITE ${FSIM_PY_STAGE}/fsim-python.cmd
     "@echo off\r\nset \"PYTHONPATH=%~dp0;%PYTHONPATH%\"\r\n\"${FSIM_PYTHON}\" %*\r\n")

# fsim_python_module(<target> <module name> <source> <library>)
function(fsim_python_module target name source library)
    add_library(${target} MODULE ${source} src/fsim_py.h)
    set_target_properties(${target} PROPERTIES
        PREFIX "" OUTPUT_NAME ${name} SUFFIX ".pyd"
        LIBRARY_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
        RUNTIME_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
        C_STANDARD 11 C_STANDARD_REQUIRED ON)
    target_include_directories(${target} SYSTEM PRIVATE ${FSIM_PYTHON_INCLUDE})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    # The stable ABI's DLL, linked directly: the interpreter's, never shipped.
    target_link_libraries(${target} PRIVATE ${library} "${FSIM_PYTHON3_DLL}")
    target_compile_definitions(${target} PRIVATE Py_LIMITED_API=0x030B0000)
    # C, not C++: the platform's C++ warning set does not apply, and CPython's
    # slot tables need the function/object pointer casts -Wpedantic rejects.
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wsign-conversion -Wcast-align
                           -Wnull-dereference $<$<BOOL:${FSIM_WARNINGS_AS_ERRORS}>:-Werror>)
endfunction()

fsim_python_module(fsim_python _native src/_native.c fsim::sdk)
set(_fsim_py_modules fsim_python)
set(_fsim_py_exes "$<TARGET_FILE:fsim_python>")
# The platform's own DLLs change with every build; the dependency walk below
# only copies what is missing (right for the toolchain's), so these are
# refreshed explicitly.
set(_fsim_py_own_dlls $<TARGET_FILE:fsim> $<TARGET_FILE:libJSBSim>)
if(TARGET fsim_vision)
    fsim_python_module(fsim_python_vision _vision src/_vision.c fsim::vision)
    list(APPEND _fsim_py_modules fsim_python_vision)
    set(_fsim_py_exes "${_fsim_py_exes};$<TARGET_FILE:fsim_python_vision>")
    list(APPEND _fsim_py_own_dlls $<TARGET_FILE:fsim_vision>)
endif()

# The staged package: the Python sources, the modules (built in place), the
# platform's DLLs in bin/ and its data in share/flightsim - the same layout
# as an installed prefix, which is where libfsim.dll looks for its data.
set(_fsim_py_share ${FSIM_PY_PACKAGE}/share/flightsim)
set(_fsim_py_stage_commands
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_CURRENT_SOURCE_DIR}/fsim ${FSIM_PY_PACKAGE})
foreach(_d aircraft engine systems)
    list(APPEND _fsim_py_stage_commands
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${FSIM_JSBSIM_DATA_DIR}/${_d} ${_fsim_py_share}/jsbsim/${_d})
endforeach()
if(TARGET fsim_vision AND EXISTS ${CMAKE_BINARY_DIR}/share/flightsim/models)
    list(APPEND _fsim_py_stage_commands
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_BINARY_DIR}/share/flightsim/models ${_fsim_py_share}/models)
endif()
add_custom_target(python_package ALL
    ${_fsim_py_stage_commands}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FSIM_PY_PACKAGE}/bin
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_fsim_py_own_dlls} ${FSIM_PY_PACKAGE}/bin
    COMMAND ${CMAKE_COMMAND}
        "-DEXES=${_fsim_py_exes}"
        "-DSEARCH=${FSIM_UCRT64_ROOT}/bin;${CMAKE_BINARY_DIR}/bin"
        "-DOBJDUMP=${FSIM_UCRT64_ROOT}/bin/objdump.exe"
        "-DDEST=${FSIM_PY_PACKAGE}/bin"
        "-DLABEL=python package"
        -P ${CMAKE_SOURCE_DIR}/cmake/DeployRuntime.cmake
    COMMENT "Staging the Python package in ${FSIM_PY_STAGE}"
    VERBATIM)
add_dependencies(python_package ${_fsim_py_modules})

if(FSIM_BUILD_TESTS)
    # The C half of benchmarks/overhead.py: the same loops through the C ABI.
    add_executable(fsim_python_baseline benchmarks/baseline.c)
    target_link_libraries(fsim_python_baseline PRIVATE fsim::sdk)
    set_target_properties(fsim_python_baseline PROPERTIES C_STANDARD 11)
    add_test(NAME python_sdk COMMAND "${FSIM_PYTHON}" -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -t ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    set(_fsim_py_test_env "PYTHONPATH=${FSIM_PY_STAGE}" "FSIM_TEST_OUTPUT=${CMAKE_BINARY_DIR}/python-test")
    if(TARGET climb_task)  # the C++ plugin example: test_plugin.py trains nothing, but loads and checks it
        list(APPEND _fsim_py_test_env "FSIM_TEST_PLUGIN=$<TARGET_FILE:climb_task>")
    endif()
    set_tests_properties(python_sdk PROPERTIES ENVIRONMENT "${_fsim_py_test_env}" TIMEOUT 900)
endif()

# Installed as its own component: dist/python/fsim, and a wheel beside it
# (pip install fsim-<version>-cp311-abi3-win_amd64.whl).
install(DIRECTORY ${FSIM_PY_PACKAGE} DESTINATION . COMPONENT python PATTERN "__pycache__" EXCLUDE)
install(CODE "execute_process(COMMAND \"${FSIM_PYTHON}\" \"${CMAKE_CURRENT_SOURCE_DIR}/tools/make_wheel.py\"
                                      \"\${CMAKE_INSTALL_PREFIX}/fsim\" \"${PROJECT_VERSION}\" \"\${CMAKE_INSTALL_PREFIX}\"
                              RESULT_VARIABLE _rc)
              if(NOT _rc EQUAL 0)
                  message(FATAL_ERROR \"python: the wheel could not be built\")
              endif()"
        COMPONENT python)
