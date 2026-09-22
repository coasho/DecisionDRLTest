# Map assets

This directory can hold the viewer's tile cache: the imagery and elevation
tiles it draws the Earth from.

It is **not** used by default. Out of the box `config/viewer.json` has

    "tileCache": ""

which means the shared per-user cache in `%LOCALAPPDATA%\flightsim\tilecache`,
so every build and package on the machine share one download and the Earth is
already there the first time you start. An empty package cache would instead
show a blank blue sphere for the first several minutes of flying.

Point the viewer here instead when you want the package to be portable:

    "tileCache": "../maps"

Everything downloaded then stays inside this package rather than in the
per-user cache, so copying the package to another machine takes the terrain
with it. It fills itself as you fly — anything already here is used without
touching the network, and anything missing is fetched once. Fill a region
ahead of time and the package works with no network at all:

    bin\tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --cache ..\maps

`--min-level` / `--max-level` bound the pyramid (the viewer draws imagery to
level 17 and elevation to level 15 by default), and `--elevation-only` or
`--imagery-only` fetch just one layer. A 30 km radius to full detail is a few
hundred megabytes; the whole Earth is not a realistic target.

Tiles are served by their respective providers under their own terms; see
`share/doc/flightsim/THIRD_PARTY_NOTICES.md`.
