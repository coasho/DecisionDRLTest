# Module layering (design section 5). Each module is a static library; the
# dependency direction is enforced by what each target links.
add_subdirectory(platform)
add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(sim)
add_subdirectory(app)
