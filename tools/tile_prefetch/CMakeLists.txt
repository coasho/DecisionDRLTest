# Headless: only io (terrain maths) and platform (HTTP, paths); no VSG.
add_executable(tile_prefetch main.cpp)
target_link_libraries(tile_prefetch PRIVATE fsim::io fsim::core fsim::platform)
fsim_set_warnings(tile_prefetch)
