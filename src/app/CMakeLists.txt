# flightsim.exe: headless runs, benchmarks and (later) --serve / --viewer.
add_executable(flightsim
    main.cpp
    Options.h
    Options.cpp)

target_link_libraries(flightsim PRIVATE fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(flightsim)

# JSBSim.dll must sit next to the executable.
add_custom_command(TARGET flightsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim>
    COMMENT "Copy JSBSim shared library next to flightsim")
