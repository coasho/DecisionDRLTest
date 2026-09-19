# platform: the only module allowed to include OS headers (design rule 5.2).
add_library(fsim_platform STATIC
    Clock.h
    Paths.h
    Threads.h
    Platform.cpp)
add_library(fsim::platform ALIAS fsim_platform)

target_include_directories(fsim_platform PUBLIC ${CMAKE_SOURCE_DIR}/src)
if(WIN32)
    target_link_libraries(fsim_platform PUBLIC winmm)
endif()
fsim_set_warnings(fsim_platform)
