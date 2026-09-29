#pragma once

// Operational zones' geometry (docs/flight-autonomy.md, 4.43; A-GRA's ZoneType): checked as A-GRA's schema restricts them,
// laid out in the plane at their reference point, and the questions a must fly asks of them - is the aircraft inside, where
// is their nearest point, where does a bearing from their centre leave them. Plane coordinates are north and east metres
// from the reference (x and y along the axes of a frame the zone is in); nothing here allocates.

#include "fsim/Control.h"

namespace fsim::control::zones {

/// The first thing at fault in a zone, as the field it is in (-1: none): 0 its shape; 1 its vertices (too few or many, not
/// finite, off the Earth, or a polygon not simple); 2 its holes (too many, too few vertices, not simple, or not inside it);
/// 3 its centre; 4 its dimensions; 5 its bearings; 6 its band; 7 its frame's fields; 8 its velocity. `frameKnown` says the
/// world has its frame (where it names one).
int fault(const OpZone& zone, bool frameKnown) noexcept;

/// `zone` laid out into `out`, its frame `spec` where it is in one (and its time left out: `nowS`).
void layOut(const OpZone& zone, const FrameSpec* spec, double nowS, MustFlyArea& out) noexcept;

/// Where a place is in the area's plane now: north and east of its reference (moving: carried on from its time), or along
/// its frame's axes, the frame as `pose` has it. False for a frame without a pose.
bool toPlane(const MustFlyArea& a, const FramePose* pose, double nowS, double latitudeRad, double longitudeRad, double& x, double& y) noexcept;
/// How far its plane's axes are turned from north now: its frame's yaw or track as `pose` has it (0: on the Earth, unturned).
double turnNow(const MustFlyArea& a, const FramePose* pose) noexcept;
/// Where a point of its plane is on the Earth now.
bool fromPlane(const MustFlyArea& a, const FramePose* pose, double nowS, double x, double y, double& latitudeRad, double& longitudeRad) noexcept;

/// Inside it, over the ground (its band apart).
bool contains(const MustFlyArea& a, double x, double y) noexcept;
/// Its centre: a polygon's vertices' mean, the others' own.
void centre(const MustFlyArea& a, double& x, double& y) noexcept;
/// Its nearest point to (x, y), outside it; `inward` the direction into it there (unit).
void nearest(const MustFlyArea& a, double x, double y, double& nx, double& ny, double& inX, double& inY) noexcept;
/// Where a bearing from its centre last leaves it (its boundary's farthest crossing that way), or its centre where none does.
void ray(const MustFlyArea& a, double bearingRad, double& nx, double& ny) noexcept;
/// How far across it is, about: its greatest extent from its centre.
double extent(const MustFlyArea& a) noexcept;

} // namespace fsim::control::zones
