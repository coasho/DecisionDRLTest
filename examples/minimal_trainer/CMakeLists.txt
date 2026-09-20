# Consumes only the public SDK (include/fsim + fsim.dll), like an external trainer would.
add_executable(minimal_trainer main.cpp)
target_link_libraries(minimal_trainer PRIVATE fsim::sdk)
target_compile_features(minimal_trainer PRIVATE cxx_std_17)
fsim_set_warnings(minimal_trainer)
