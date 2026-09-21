add_executable(ppo_trainer main.cpp)
target_link_libraries(ppo_trainer PRIVATE fsim::sdk)
if(TARGET fsim_vision)
    # --depth WxH: a downsampled depth camera joins the observation (needs the vision library).
    target_link_libraries(ppo_trainer PRIVATE fsim::vision)
    target_compile_definitions(ppo_trainer PRIVATE HAVE_FSIM_VISION)
endif()
target_compile_features(ppo_trainer PRIVATE cxx_std_17)
fsim_set_warnings(ppo_trainer)
# Numeric code mixes float storage with double accumulation on purpose.
if(NOT MSVC)
    target_compile_options(ppo_trainer PRIVATE -Wno-double-promotion)
endif()
