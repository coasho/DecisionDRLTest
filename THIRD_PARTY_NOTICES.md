# Third-party notices

flightsim is MIT licensed (see [LICENSE](LICENSE)). It builds on and ships
with the following components; each keeps its own licence.

| Component | Use | Licence | Shipped as |
| --- | --- | --- | --- |
| [JSBSim](https://github.com/JSBSim-Team/jsbsim) | flight dynamics (`src/sim`) and the aircraft data tree (`share/flightsim/jsbsim`) | LGPL-2.1 | `libJSBSim.dll` (shared library, unmodified; source in `third_party/jsbsim`) |
| [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) | viewer and vision rendering | MIT (c) 2018 Robert Osfield | statically linked into `flightsim-viewer.exe` and `libfsim_vision.dll` |
| [vsgXchange](https://github.com/vsg-dev/vsgXchange) | model/image loading, tile fetching | MIT (c) 2019 Robert Osfield | statically linked |
| [vsgImGui](https://github.com/vsg-dev/vsgImGui), [Dear ImGui](https://github.com/ocornut/imgui), [ImPlot](https://github.com/epezent/implot) | viewer UI | MIT | statically linked into `flightsim-viewer.exe` |
| [stb_image / stb_image_write](https://github.com/nothings/stb) | PNG decode (elevation tiles), PNG write (camera images) | MIT / public domain | `third_party/stb` |
| [Open Asset Import Library (assimp)](https://github.com/assimp/assimp) | glTF loading through vsgXchange | BSD-3-Clause | `libassimp-6.dll` (MSYS2 package) |
| [curl](https://curl.se/) | tile downloads in the viewer (vsgXchange) | curl licence (MIT-style) | `libcurl-4.dll` and its dependencies (OpenSSL: Apache-2.0; nghttp2, ngtcp2, nghttp3: MIT; libssh2: BSD; libpsl, libidn2, libunistring: LGPL/MIT; brotli: MIT; zstd: BSD; zlib: zlib) - MSYS2 packages |
| [glslang](https://github.com/KhronosGroup/glslang), [SPIRV-Tools](https://github.com/KhronosGroup/SPIRV-Tools) | runtime shader compilation | BSD-3-Clause / Apache-2.0 | MSYS2 packages |
| [Vulkan loader](https://github.com/KhronosGroup/Vulkan-Loader) | | Apache-2.0 | `vulkan-1.dll` from the graphics driver / MSYS2 |
| MinGW-w64 / GCC runtime (`libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`) | C++ runtime | GPL with runtime exception / MIT (winpthreads) | MSYS2 UCRT64 packages |
| [Catch2](https://github.com/catchorg/Catch2) | tests only | BSL-1.0 | not shipped |
| Cesium Air (`Cesium_Air.glb`) | sample aircraft model | Apache-2.0, (c) CesiumGS, Inc. and Contributors | downloaded at configure time (`assets/models/NOTICE.md`) |

## Data services

The viewer and the terrain physics fetch public tiles at run time and cache
them locally; using them is subject to the providers' terms:

- **Esri World Imagery** (`server.arcgisonline.com`): Esri, Maxar, Earthstar
  Geographics, and the GIS User Community. Attribution is required when
  imagery is shown or published.
- **AWS Terrain Tiles** (`s3.amazonaws.com/elevation-tiles-prod`, Terrarium
  encoding): Mapzen / Amazon Open Data; sources include USGS 3DEP, SRTM,
  GMTED2010, ETOPO1 and others - see the AWS Open Data registry entry.
- **OpenStreetMap** tiles (optional `--imagery osm`): (c) OpenStreetMap
  contributors, ODbL; the tile usage policy applies.
