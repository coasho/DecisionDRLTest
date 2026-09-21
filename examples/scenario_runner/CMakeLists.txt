add_executable(scenario_runner main.cpp)
target_link_libraries(scenario_runner PRIVATE fsim::sdk)
target_compile_features(scenario_runner PRIVATE cxx_std_17)
fsim_set_warnings(scenario_runner)
