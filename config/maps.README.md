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
touching the network, and anything missing is fetched once. Fill it ahead of
time and the package works with no network at all.

## Choosing what to download

`bin/tile_prefetch` takes a region and a range of levels per layer. A region
is a disc, a lat/lon box, a corridor along a route, or the whole globe:

    bin\tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --levels 0-14
    bin\tile_prefetch --bbox 45.8,6.0,47.2,10.5 --elevation-levels 8-11
    bin\tile_prefetch --route 37.62,-122.4;34.05,-118.24 --width-km 40 --levels 7-10
    bin\tile_prefetch --global --levels 0-6

`--imagery-levels` and `--elevation-levels` set the two layers separately, and
a layer with no range is not fetched at all. That is the main lever: a city is
flat and wants imagery, a mountain range wants elevation, and the ocean wants
neither beyond the global base.

`--dry-run` counts and prices the tiles without downloading any, so a plan can
be checked against a budget first. Overlapping regions are counted once.

For a whole offline set, put the regions in a file and pass `--plan`:

    bin\tile_prefetch --plan config/offline-map-plan.json --cache ..\maps --dry-run

`config/offline-map-plan.json` in this package is a worked example: a global
base, eight mountain ranges at elevation only, thirty airports at imagery
level 14, and three route corridors — 66,425 tiles, about 1.95 GB, sized to
leave a 2 GB package with room for the application.

## How much disk

Measured from a real cache: an imagery tile averages 14.5 KiB and a Terrarium
elevation tile about 70 KiB. At the shipped levels, around 45 degrees latitude:

| region | imagery | elevation | total |
| --- | --- | --- | --- |
| 10 km radius | 0.1 GB | 0.04 GB | **0.2 GB** |
| 30 km radius | 1.1 GB | 0.3 GB | **1.5 GB** |
| 100 km radius | 12.4 GB | 3.7 GB | **16 GB** |
| Switzerland | 16.3 GB | 4.9 GB | **21 GB** |
| France | 218 GB | 66 GB | **283 GB** |

Cost goes with area, so doubling the radius quadruples the download.

The imagery level is the lever worth pulling. Each level down divides the
imagery by four, and two levels down (`--max-level 15`, about 1.2 m per pixel
at the equator) turns a 30 km radius into 0.4 GB and Switzerland into 5.9 GB.
Elevation is unaffected - it has nothing below level 15 to fetch.

The whole Earth is not a realistic target: every tile to the shipped levels is
about 400 TiB, and roughly 140 TiB if the oceans are skipped.

Tiles are served by their respective providers under their own terms; see
`share/doc/flightsim/THIRD_PARTY_NOTICES.md`.
