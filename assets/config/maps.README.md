# Map assets

The imagery and elevation tiles the viewer draws the Earth from.

## In a package

`maps/` holds the whole offline map, and `config/viewer.json` is written to
use it and nothing else:

    "offline": true,
    "tileCache": "../maps",
    "maxLevel": 14,
    "elevationMaxLevel": 12

The viewer opens no network connection at all. Wherever the package has no
tile at the level a view wants, it draws the nearest coarser tile it does
have (see *How the viewer reads a partial pyramid* below).

## In the source tree

The maps live in `assets/maps`. They are gigabytes, so they are not in git:
`fetch-maps.cmd` downloads them as `assets/config/offline-map-plan.json` asks,
and `fsim dist` copies them into the package - as a mirror, so a tile the plan
has dropped is dropped from the package too.

A viewer run from the build tree is online instead, with `"tileCache": ""`:
the shared per-user cache in `%LOCALAPPDATA%\flightsim\tilecache`, which every
build on the machine shares and which fills as you fly.

## Choosing what to download

`bin/tile_prefetch` takes a region and a range of levels per layer. A region
is a disc, a lat/lon box, a corridor along a route, the whole globe, or every
tile with land in it:

    bin\tile_prefetch --lat 37.62 --lon -122.4 --radius-km 30 --levels 0-14
    bin\tile_prefetch --bbox 45.8,6.0,47.2,10.5 --elevation-levels 8-11
    bin\tile_prefetch --route 37.62,-122.4;34.05,-118.24 --width-km 40 --levels 7-10
    bin\tile_prefetch --global --levels 0-6
    bin\tile_prefetch --land --taper --imagery-levels 7-9

`--imagery-levels` and `--elevation-levels` set the two layers separately, and
a layer with no range is not fetched at all. That is the main lever: a city is
flat and wants imagery, a mountain range wants elevation, and the ocean wants
neither beyond the global base.

`--land` is what lets the oceans stay coarse. It reads the level-6 elevation
(fetching it first if needed) and keeps a tile if any of it is above sea
level: 44.7 % of level-7 tiles, 38.7 % of level-8, 31 % by level 12.

`--taper` makes the levels the equator's. Web Mercator tiles shrink with the
cosine of latitude, so a level-8 tile at 60 degrees is as sharp as a level-9
tile at the equator, and at 75 degrees as sharp as a level-10 one. One level
everywhere therefore spends most of its tiles near the poles - 62 % of the
land's level-8 tiles lie beyond 60 degrees - on detail the equator never
gets. Tapered, each level stops where the one above already has the equator's
ground resolution: with level 9 at the top, level 9 reaches 60 degrees, level
8 reaches 75.5 and level 7 reaches 82.8. Nowhere on land is coarser than the
equator, and none of the budget goes on polar detail the equator does not get.

Every region is fetched a little wider than it is asked for, because a camera
looking at the middle of one still has the far edge of its view outside it:
`--margin` (0.15) widens every level by a fraction, and `--feather` (2) adds a
skirt measured in *tiles at that level*, so detail tapers off over a few
levels instead of ending at a wall. Tiles, not a fraction: a fraction
multiplies area, and four levels up that is seven times the ground.

### How elevation is stored

Elevation is kept smaller than it is served, and still as Terrarium, so every
reader is none the wiser:

- **128 x 128 texels** (`--elevation-size`, the plan's `elevationSize`)
  instead of 256 x 256. The mesh draws 64 x 64 of each tile, so that is still
  twice what is drawn in each direction. Box-filtered, as the mesh does.
- **Whole metres.** Terrarium spends a third byte on 1/256 m, which after a
  box filter is only noise to a compressor. The DEMs beneath are good to
  metres, and rounding moves no texel by more than half a metre.
- **zlib at its best**, where stb on its own writes fixed Huffman codes.

A tile then costs a quarter of what it is served as: 65 KB to 16 KB,
measured on 32 land tiles from four continents. Re-running the tool re-stores
any tile on disk that is not in this form yet, in place.

### Pricing a plan

`--dry-run` counts and prices the tiles without downloading any, so a plan can
be checked against a budget first. Overlapping regions are counted once. The
price is per tile, by latitude, from what this plan's own tiles weigh - good
to about 5 % for a plan like it, and a city costs some 15 % more than the land
around it. `--list <file>` writes every tile the plan asks for, one per line,
for pricing it any other way.

It also prints the two ceilings a package built from the plan should use:

    for an offline package set viewer.json  "maxLevel": 14,  "elevationMaxLevel": 12

Set those, or the viewer spends its time asking for detail the package does
not contain. The package build sets them from `cmake/PackageConfig.cmake`.

`--prune` deletes cached tiles the plan does not ask for, so a package built
to a budget carries only what it needs after the plan has been tightened.

For a whole offline set, put the regions in a file and pass `--plan`:

    bin\tile_prefetch --plan config/offline-map-plan.json --cache ..\maps --dry-run

`config/offline-map-plan.json` in this package is the plan the package was
built from, in three tiers:

- **globe** - every tile on Earth to level 6, both layers
- **land** - every tile with land in it, imagery and relief to 9, tapered
- **detail** - 21 mountain ranges with relief to 11 and imagery to 10 over
  95 km, and imagery to 11 within 50 km of each one's heart; 31 airports with
  imagery to 14 and relief to 12; and two flight corridors with imagery to 11

Every tier starts at or below level 7, so the pyramid is contiguous: the
viewer never meets a level it must stop at while something deeper exists.

## What a 2.5 GB package can and cannot show

This package measures 2.41 GB as files (2.24 GiB) - 1.35 GB of imagery,
0.99 GB of elevation and 0.07 GB of application - and 2.38 GB zipped.

Outside the detail regions the land is the land tier: 306 m per pixel at the
equator, and between 150 and 306 m everywhere else. In the viewer's
1600 x 900 window with its 30 degree field of view, one texel of that covers
about two screen pixels looking straight down from 260 km, four from 130 km
and ten from 50 km. So the land is sharp from high up, softens on the way
down, and at the altitudes aircraft fly it is a smooth painting everywhere
except the mountain ranges, airports and corridors of the detail tier.

That is the budget, not the plan. Each level of imagery halves those
distances, and with the globe and detail tiers fixed, costs this much:

| land tier, tapered | land imagery at the equator | package |
| --- | --- | --- |
| imagery 8, relief 8 | 611 m/px | ~1.3 GB |
| **imagery 9, relief 9 (this package)** | **306 m/px** | **2.41 GB, measured** |
| imagery 10, relief 10 | 153 m/px | ~6.3 GB |
| imagery 11, relief 10 | 76 m/px | ~14 GB |
| imagery 12, relief 11 | 38 m/px | ~51 GB |

The estimates are `tile_prefetch --dry-run` with only the land tier changed.

## How the viewer reads a partial pyramid

VSG's tile reader assumes both layers exist at every level it draws. Offline,
a missing tile is instead made from its nearest real ancestor in that layer,
wherever anything real exists at that level nearby, and refinement stops
where the data does (`world/OfflineTiles.h`). Without that, imagery with no
elevation beneath it is drawn at sea level, and elevation with no imagery is
not drawn at all.

The satellite imagery is itself a mosaic of different captures, and some of
the hard-edged colour rectangles you may see are joins in it, present online
and offline alike.

## What full detail costs

For scale, at the viewer's online levels (imagery 17, about 1.2 m per pixel
at the equator; elevation 15), with tiles as they are served - an imagery
tile averages 14.5 KiB and an elevation tile about 70 KiB - around 45 degrees
latitude:

| region | imagery | elevation | total |
| --- | --- | --- | --- |
| 10 km radius | 0.1 GB | 0.04 GB | **0.2 GB** |
| 30 km radius | 1.1 GB | 0.3 GB | **1.5 GB** |
| 100 km radius | 12.4 GB | 3.7 GB | **16 GB** |
| Switzerland | 16.3 GB | 4.9 GB | **21 GB** |
| France | 218 GB | 66 GB | **283 GB** |

Cost goes with area, so doubling the radius quadruples the download. Each
imagery level less divides the imagery by four; elevation stored by
`tile_prefetch` is about a quarter of the column above.

The whole Earth is not a realistic target: every tile to those levels is
about 400 TiB, and roughly 140 TiB if the oceans are skipped.

Tiles are served by their respective providers under their own terms; see
`share/doc/flightsim/THIRD_PARTY_NOTICES.md`.
