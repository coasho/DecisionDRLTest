# A C++ task plugin for the Python SDK: a DLL that links the public SDK and
# registers itself when fsim.load_plugin() loads it (climb_task.cpp).
add_library(climb_task SHARED climb_task.cpp)
target_link_libraries(climb_task PRIVATE fsim::sdk)
target_compile_features(climb_task PRIVATE cxx_std_17)
set_target_properties(climb_task PROPERTIES PREFIX "")
fsim_set_warnings(climb_task)
