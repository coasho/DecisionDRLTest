# MSYS2 UCRT64 (GCC, mingw-w64, UCRT) toolchain - the project's mandated toolchain.
# Override FSIM_UCRT64_ROOT if MSYS2 lives elsewhere.
set(FSIM_UCRT64_ROOT "D:/ENV/DevLanguages/Cpp/msys2/ucrt64" CACHE PATH "MSYS2 UCRT64 root")

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_C_COMPILER   "${FSIM_UCRT64_ROOT}/bin/gcc.exe"   CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER "${FSIM_UCRT64_ROOT}/bin/g++.exe"   CACHE FILEPATH "")
set(CMAKE_RC_COMPILER  "${FSIM_UCRT64_ROOT}/bin/windres.exe" CACHE FILEPATH "")
set(CMAKE_MAKE_PROGRAM "${FSIM_UCRT64_ROOT}/bin/ninja.exe" CACHE FILEPATH "")

# Let find_package/pkg-config see the MSYS2 packages (Vulkan, glslang, assimp, curl, ...).
list(APPEND CMAKE_PREFIX_PATH "${FSIM_UCRT64_ROOT}")
set(ENV{PKG_CONFIG_PATH} "${FSIM_UCRT64_ROOT}/lib/pkgconfig")
set(Vulkan_INCLUDE_DIR "${FSIM_UCRT64_ROOT}/include" CACHE PATH "")
set(Vulkan_LIBRARY     "${FSIM_UCRT64_ROOT}/lib/libvulkan-1.dll.a" CACHE FILEPATH "")
set(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE "${FSIM_UCRT64_ROOT}/bin/glslangValidator.exe" CACHE FILEPATH "")
