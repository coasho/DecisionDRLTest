# comm: abstract communication interfaces (design 9.6).
add_library(fsim_comm STATIC
    Comm.h
    Comm.cpp)
add_library(fsim::comm ALIAS fsim_comm)
target_include_directories(fsim_comm PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_comm PUBLIC fsim::control fsim::core fsim::platform)
fsim_set_warnings(fsim_comm)
