#pragma once

// Operational volumes' geometry (docs/flight-autonomy.md, 4.45; A-GRA's OpVolumeType): checked as A-GRA's schema restricts them,
// laid out at their point - north and east metres from it in the plane zones use (x and y along the axes of a frame the
// volume is in), and metres below it - and the questions a must fly asks of them: is the aircraft inside, and where is a point
// well inside. Every shape is convex, so the host finds where to go in by these alone. Nothing here allocates.

#include "fsim/Control.h"

namespace fsim::control::volumes {

/// The first thing at fault in a volume, as the field it is in (-1: none): 0 its shape; 1 its point (a place not finite, off
/// the Earth or given both ways, an altitude missing on the Earth or not finite, a reference without it; a geocentric
/// volume's given one); 2 its dimensions; 3 its attitude; 4 a geocentric volume's bounds; 5 its frame's fields; 6 its
/// velocity. `frameKnown` says the world has its frame (where it names one).
int fault(const OpVolume& volume, bool frameKnown) noexcept;

/// `volume` laid out into `out` (its `volume` its shape), its frame `spec` where it is in one (and its time left out: `nowS`).
void layOut(const OpVolume& volume, const FrameSpec* spec, double nowS, MustFlyArea& out) noexcept;

/// Its point's altitude now, in its reference: as given, moving at its down speed - or, in a frame and given none, the frame's
/// origin's (`pose`, above sea level).
double pointAltitude(const MustFlyArea& a, const FramePose* pose, double nowS) noexcept;

/// Inside it: (x, y) in its plane now (zones::toPlane), `downM` below its point as it is now.
bool contains(const MustFlyArea& a, double x, double y, double downM) noexcept;

/// A point well inside it, in its plane and below its point: where the host aims into it toward.
void inner(const MustFlyArea& a, double& x, double& y, double& downM) noexcept;

/// How far across it is, about (a volume without end: as far as the host looks into it).
double extent(const MustFlyArea& a) noexcept;

} // namespace fsim::control::volumes
