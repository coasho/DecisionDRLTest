# Runtime asset tree assembled under the build directory: <build>/share/flightsim.
# io::AssetResolver looks in <exe>/../share/flightsim, so bin/ + share/ form the
# same layout an installed package has (design 11.3 "Distribution").
set(FSIM_SHARE_DIR ${CMAKE_BINARY_DIR}/share/flightsim)
file(MAKE_DIRECTORY ${FSIM_SHARE_DIR}/models)

# Copy manifests / notices from the source tree.
file(GLOB _fsim_model_manifests ${CMAKE_SOURCE_DIR}/assets/models/*.manifest ${CMAKE_SOURCE_DIR}/assets/models/NOTICE.md)
foreach(_m ${_fsim_model_manifests})
    configure_file(${_m} ${FSIM_SHARE_DIR}/models/ COPYONLY)
endforeach()

# Sample aircraft model (Apache-2.0, CesiumGS/cesium sample data), fetched once.
set(_fsim_air ${FSIM_SHARE_DIR}/models/Cesium_Air.glb)
if(NOT EXISTS ${_fsim_air})
    message(STATUS "flightsim: downloading sample vehicle model Cesium_Air.glb")
    file(DOWNLOAD
        https://raw.githubusercontent.com/CesiumGS/cesium/main/Apps/SampleData/models/CesiumAir/Cesium_Air.glb
        ${_fsim_air}
        STATUS _fsim_dl TIMEOUT 60)
    list(GET _fsim_dl 0 _fsim_dl_code)
    if(NOT _fsim_dl_code EQUAL 0)
        message(WARNING "flightsim: sample model download failed (${_fsim_dl}); the viewer falls back to the placeholder")
        file(REMOVE ${_fsim_air})
    endif()
endif()
