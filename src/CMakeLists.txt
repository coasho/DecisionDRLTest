# Module layering (design section 5). Each module is a static library; the
# dependency direction is enforced by what each target links.
add_subdirectory(platform)
add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(sim)
add_subdirectory(env)
add_subdirectory(sdk)
if(FSIM_WITH_RENDER)
    add_subdirectory(render)
    add_subdirectory(world)
    add_subdirectory(ui)
endif()
add_subdirectory(app)
