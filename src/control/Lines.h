#pragma once

// Operational lines' geometry (docs/flight-autonomy.md, 4.44; A-GRA's LineType): checked as A-GRA's schema restricts them,
// and laid out in the plane at their reference point - north and east metres from it (x and y along the axes of a frame the
// line is in) - with the altitude, band and widths at each vertex, for a must fly to fly through as a corridor. Nothing
// here allocates.

#include "fsim/Control.h"

namespace fsim::control::lines {

/// The first thing at fault in a line, as the field it is in (-1: none): 0 its vertices (too few or many; a place not finite,
/// off the Earth or given both ways; two in a row at one place; a vertex's altitude, range, reference or widths, or its
/// altitude outside the band there); 1 its projection; 2 its widths; 3 its band; 4 its frame's fields; 5 its velocity.
/// `frameKnown` says the world has its frame (where it names one).
int fault(const OpLine& line, bool frameKnown) noexcept;

/// `line` laid out into `out` - its shape Count, its lineCount its vertices' - its frame `spec` where it is in one (and its
/// time left out: `nowS`). At each vertex: its own altitude; its own range, else the line's band; its own widths, else the
/// line's.
void layOut(const OpLine& line, const FrameSpec* spec, double nowS, MustFlyArea& out) noexcept;

} // namespace fsim::control::lines
