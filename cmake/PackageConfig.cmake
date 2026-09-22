# The development viewer.json and the packaged one differ in exactly three
# values, so the packaged one is generated from the other rather than kept as a
# second copy to maintain. A distributed viewer reads its tiles from the maps/
# directory beside it, never fetches, and does not ask for levels deeper than
# assets/config/offline-map-plan.json puts there.
#
# Those two ceilings are the plan's deepest levels; `tile_prefetch --dry-run`
# prints them. Change the plan and these move with it.
set(FSIM_PACKAGE_MAX_LEVEL 14)
set(FSIM_PACKAGE_ELEVATION_MAX_LEVEL 12)

function(fsim_write_packaged_config src dest)
    file(READ "${src}" cfg)
    string(REPLACE "\"offline\": false" "\"offline\": true" cfg "${cfg}")
    string(REPLACE "\"tileCache\": \"\"" "\"tileCache\": \"../maps\"" cfg "${cfg}")
    string(REPLACE "\"maxLevel\": 17" "\"maxLevel\": ${FSIM_PACKAGE_MAX_LEVEL}" cfg "${cfg}")
    string(REPLACE "\"elevationMaxLevel\": 15"
                   "\"elevationMaxLevel\": ${FSIM_PACKAGE_ELEVATION_MAX_LEVEL}" cfg "${cfg}")
    file(WRITE "${dest}" "${cfg}")
    message(STATUS "viewer.json: offline, tiles from ../maps, levels capped at "
                   "${FSIM_PACKAGE_MAX_LEVEL}/${FSIM_PACKAGE_ELEVATION_MAX_LEVEL}")
endfunction()
