# Map assets

This directory is the viewer's tile cache: the imagery and elevation tiles it
draws the Earth from. `config/viewer.json` points at it with

    "tileCache": "../maps"

so everything downloaded stays inside this package rather than in the
per-user cache. Copy the package to another machine and the terrain goes
with it.

It starts empty and fills itself as you fly — anything already here is used
without touching the network, and anything missing is fetched once. To fill a
region ahead of time, so the package works with no network at all:

    bin\tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --cache ..\maps

`--min-level` / `--max-level` bound the pyramid (the viewer draws imagery to
level 17 and elevation to level 15 by default), and `--elevation-only` or
`--imagery-only` fetch just one layer. A 30 km radius to full detail is a few
hundred megabytes; the whole Earth is not a realistic target.

To use the shared per-user cache instead — sensible on a development machine,
where several builds can share one download — set `"tileCache": ""` in
`config/viewer.json` and the viewer falls back to
`%LOCALAPPDATA%\flightsim\tilecache`.

Tiles are served by their respective providers under their own terms; see
`share/doc/flightsim/THIRD_PARTY_NOTICES.md`.
