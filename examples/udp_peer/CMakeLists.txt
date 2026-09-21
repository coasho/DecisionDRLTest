add_executable(udp_peer main.cpp)
target_link_libraries(udp_peer PRIVATE fsim::sdk ws2_32)
target_compile_features(udp_peer PRIVATE cxx_std_17)
fsim_set_warnings(udp_peer)
