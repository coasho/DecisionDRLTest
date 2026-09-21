add_executable(vision_capture main.cpp)
target_link_libraries(vision_capture PRIVATE fsim::sdk fsim::vision)
target_compile_features(vision_capture PRIVATE cxx_std_17)
fsim_set_warnings(vision_capture)
