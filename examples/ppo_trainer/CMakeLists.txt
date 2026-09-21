add_executable(ppo_trainer main.cpp)
target_link_libraries(ppo_trainer PRIVATE fsim::sdk)
target_compile_features(ppo_trainer PRIVATE cxx_std_17)
fsim_set_warnings(ppo_trainer)
# Numeric code mixes float storage with double accumulation on purpose.
if(NOT MSVC)
    target_compile_options(ppo_trainer PRIVATE -Wno-double-promotion)
endif()
