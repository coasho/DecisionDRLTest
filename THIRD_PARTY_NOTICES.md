# Third-party notices

flightsim is MIT licensed (see [LICENSE](LICENSE)). It builds on and ships
with the following components; each keeps its own licence.

| Component | Use | Licence | Shipped as |
| --- | --- | --- | --- |
| [JSBSim](https://github.com/JSBSim-Team/jsbsim) | flight dynamics (`src/sim`) and the aircraft data tree (`share/flightsim/jsbsim`) | LGPL-2.1 | `libJSBSim.dll` (shared library, source in `third_party/jsbsim`), built with one fix of flightsim's - see below |
| [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) | viewer and vision rendering | MIT (c) 2018 Robert Osfield | statically linked into `flightsim-viewer.exe` and `libfsim_vision.dll` |
| [vsgXchange](https://github.com/vsg-dev/vsgXchange) | model/image loading, tile fetching | MIT (c) 2019 Robert Osfield | statically linked |
| [vsgImGui](https://github.com/vsg-dev/vsgImGui), [Dear ImGui](https://github.com/ocornut/imgui), [ImPlot](https://github.com/epezent/implot) | viewer UI | MIT | statically linked into `flightsim-viewer.exe` |
| [stb_image / stb_image_write](https://github.com/nothings/stb) | PNG decode (elevation tiles), PNG write (camera images) | MIT / public domain | `third_party/stb` |
| [Open Asset Import Library (assimp)](https://github.com/assimp/assimp) | glTF loading through vsgXchange | BSD-3-Clause | `libassimp-6.dll` (MSYS2 package) |
| [curl](https://curl.se/) | tile downloads in the viewer (vsgXchange) | curl licence (MIT-style) | `libcurl-4.dll` and its dependencies (OpenSSL: Apache-2.0; nghttp2, ngtcp2, nghttp3: MIT; libssh2: BSD; libpsl, libidn2, libunistring: LGPL/MIT; brotli: MIT; zstd: BSD; zlib: zlib) - MSYS2 packages |
| [glslang](https://github.com/KhronosGroup/glslang), [SPIRV-Tools](https://github.com/KhronosGroup/SPIRV-Tools) | runtime shader compilation | BSD-3-Clause / Apache-2.0 | MSYS2 packages |
| [Vulkan loader](https://github.com/KhronosGroup/Vulkan-Loader) | | Apache-2.0 | `vulkan-1.dll` from the graphics driver / MSYS2 |
| MinGW-w64 / GCC runtime (`libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`) | C++ runtime | GPL with runtime exception / MIT (winpthreads) | MSYS2 UCRT64 packages |
| [World Magnetic Model 2025](https://www.ncei.noaa.gov/products/world-magnetic-model) (NOAA National Centers for Environmental Information and the British Geological Survey) | the magnetic field and declination (`src/control/Magnetic.cpp`): its coefficients (`WMM.COF` of 11/13/2024) built in; its test values (`tests/data/wmm2025_test_values.txt`) test it | a work of the US Government, public domain | compiled into `libfsim.dll`; the test values not shipped |
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

## Changes to JSBSim

flightsim builds JSBSim 1.3.1 from the unmodified submodule in `third_party/jsbsim`, with
four changes:

- In `src/models/FGLGear.cpp`, the projection of a wheel's strut on the ground normal is
  bounded at 45 degrees. Upstream divides the wheel's compression and ground force by that
  projection, so a wheel meeting the ground sideways, as in a cartwheel, got an unbounded force.
- In `src/models/propulsion/FGRotor.h` and `FGRotor.cpp` (the helicopter rotor,
  docs/rotorcraft.md): a reset restarts the rotor, which kept the last run's speed and
  inflow; a rotor may choose the classical force model (`<model>classical</model>`) - the
  in-plane forces and the torque of NASA TM-73254 (Talbot and Corliss, 1977), equations 1
  to 4 - instead of Heffley's, which stays the default - and with it the ground effect scales
  the steady inflow rather than the lagged one; and a start or a reset converges every
  rotor's inflow and its ground-height filter before the first step (upstream's begin from
  nothing and settle over the first seconds). Past that start, a rotor that does not ask for
  the classical model flies as upstream's does.
- In `src/models/propulsion/FGElectric.cpp`, an electric engine whose file gives a specific
  fuel consumption (`<bsfc>`, as a piston engine's does) burns that times its power from the
  tanks feeding it, and gives no power once they are empty (hangar's helicopters fly their
  turboshafts as such a governed power source). An electric engine without one burns
  nothing and runs as upstream's does.
- In `src/models/FGAccelerations.cpp`, an impact the step cannot resolve is made inelastic.
  The ground contacts' springs and dampers act for a whole step from the state it begins
  with, so an aircraft striking the ground faster than its contacts can take in a step
  (docs/rotorcraft.md: a 27 g quadrotor at 35 m/s) was sent back up faster than it came
  down. When the ground's forces would send a contact point back out more than 5 m/s faster
  than it came in, the step applies instead the forces that stop the contact points'
  approach, each no more than its contact's own, solved as JSBSim solves its friction, and
  bounds the friction by them. Otherwise the ground's forces are upstream's, bit for bit.

`cmake/JsbsimPatches.cmake` makes the changes: it writes the changed files into the build
tree, with each edit marked "flightsim patch" (two files the rotor's header reaches,
`FGEngine.cpp` and `FGTurboProp.cpp`, are compiled from there unchanged). Packages carry the
changed files and that script in `share/doc/flightsim/jsbsim-changes/`.
