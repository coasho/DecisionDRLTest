# Headless: only io (terrain maths) and platform (HTTP, paths); no VSG.
add_executable(tile_prefetch main.cpp)
target_link_libraries(tile_prefetch PRIVATE fsim::io fsim::core fsim::platform)
fsim_set_warnings(tile_prefetch)
# stb_image_write, to store elevation tiles smaller than they are served, with
# zlib doing the deflate (zlib1.dll ships already, for curl and assimp).
find_package(ZLIB REQUIRED)
target_include_directories(tile_prefetch SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/stb)
target_link_libraries(tile_prefetch PRIVATE ZLIB::ZLIB)
