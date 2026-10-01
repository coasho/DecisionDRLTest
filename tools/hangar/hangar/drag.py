"""Drag devices (docs/flight-autonomy.md, 4.55): what an aircraft opens into the flow to slow down - airbrake plates on
the fuselage or split at the tail or wing tips, spoilers on a wing's upper surface, or control surfaces its flight
control system deflects against each other as a speedbrake. One speedbrake command opens them all
(fcs/speedbrake-cmd-norm), each to its own most over its transit time.

[[drag_device]]
name = "speed brake"
kind = "airbrake"             # plates hinged into the flow
area = 2.93                   # m2, all its plates together
position = [10.2, 0.0, 1.1]   # their centroid, opened (the pitching moment of their drag)
max_deg = 45                  # opened this far
transit_s = 2.0

[[drag_device]]
name = "spoilers"
kind = "spoiler"              # panels on a wing's upper surface: drag, and the lift of the wing behind them lost
surface = "wing"
span = [0.15, 0.55]           # of the surface's half span, both halves
chord_fraction = 0.12         # the panels' chord, a share of the wing's
hinge = 0.62                  # the hinge line's chord fraction
max_deg = 60
in_flight = false             # opened on an approach (airbrakes and surfaces: yes; spoilers that dump lift: no, unless they
                              # are the type's airbrakes)

[[drag_device]]
name = "surfaces"
kind = "surfaces"             # control surfaces deflected against each other: their drag, their forces cancelling
channels = { rudder = 30 }    # each channel's deflection (deg) at the full command
"""
import numpy as np

KINDS = ("airbrake", "spoiler", "surfaces")

#: a flat plate's normal force across the stream, per its area's dynamic pressure, at 90 deg (Hoerner, Fluid-Dynamic Drag:
#: about 1.2 for a plate on a surface, 1.17 to 1.28 free); at deflection d its normal force is that times sin d and its drag
#: that times sin^2 d
PLATE_CN = 1.2
#: a spoiler's share of the lift of the wing behind it lost at full effect, reached at SPOILER_FULL_DEG (an estimate after
#: Hoerner, Fluid-Dynamic Lift: spoilers destroy most of the spanned section's lift by 40 to 60 deg)
SPOILER_LIFT_LOSS = 0.7
SPOILER_FULL_DEG = 45.0


class DragDevice:
    def __init__(self, spec, aircraft):
        self.name = str(spec.get("name", "speed brake"))
        where = "drag_device %r" % self.name
        self.kind = spec.get("kind", "airbrake")
        if self.kind not in KINDS:
            raise ValueError("%s: kind must be one of %s" % (where, ", ".join(KINDS)))
        self.max_deg = float(spec.get("max_deg", 60.0))
        self.transit_s = float(spec.get("transit_s", 2.0))
        if not (0.0 < self.max_deg <= 90.0) or not (self.transit_s > 0.0):
            raise ValueError("%s: max_deg within (0, 90] and transit_s above 0" % where)
        self.channels = {}
        self.in_flight = bool(spec.get("in_flight", self.kind != "spoiler"))
        if self.kind == "airbrake":
            self.area = float(spec["area"])
            self.position = np.asarray(spec["position"], float)
            if not (self.area > 0.0) or self.position.shape != (3,):
                raise ValueError("%s: area (m2) above 0 and position = [x, y, z] are required" % where)
        elif self.kind == "spoiler":
            names = {s.name: s for s in aircraft.surfaces}
            if spec.get("surface") not in names:
                raise ValueError("%s: surface must name one of %s" % (where, ", ".join(names)))
            self.surface = names[spec["surface"]]
            span = spec.get("span")
            if not span or len(span) != 2 or not (0.0 <= float(span[0]) < float(span[1]) <= 1.0):
                raise ValueError("%s: span = [eta_start, eta_end] within [0, 1] is required" % where)
            self.eta0, self.eta1 = float(span[0]), float(span[1])
            self.chord_fraction = float(spec.get("chord_fraction", 0.12))
            self.hinge = float(spec.get("hinge", 0.62))
            if not (0.0 < self.chord_fraction < 0.5) or not (0.0 < self.hinge < 1.0):
                raise ValueError("%s: chord_fraction within (0, 0.5) and hinge within (0, 1)" % where)
            self._spoiler_geometry()
        else:
            self.channels = {str(k): float(v) for k, v in spec.get("channels", {}).items()}
            known = set(aircraft.channels())
            if not self.channels or any(k not in known for k in self.channels):
                raise ValueError("%s: channels = { channel = deg } over the design's channels (%s)" % (where, ", ".join(sorted(known))))

    def _spoiler_geometry(self, n=200):
        # the wing area behind the panels, the panels' own, and where the lift lost acts (its centroid of chord)
        s = self.surface
        eta = np.linspace(self.eta0, self.eta1, n + 1)
        mid = 0.5 * (eta[1:] + eta[:-1])
        ds = np.diff(eta) * s.half_arc
        chords = np.array([s.station(e)[1] for e in mid])
        quarter = np.array([s.station(e)[0][0] + 0.25 * s.station(e)[1] for e in mid])
        hinge_x = np.array([s.station(e)[0][0] + self.hinge * s.station(e)[1] for e in mid])
        hinge_z = np.array([s.station(e)[0][2] for e in mid])
        sides = 2 if s.mirror else 1
        self.wing_area = float(np.sum(chords * ds) * sides)
        self.area = float(np.sum(self.chord_fraction * chords * ds) * sides)
        self.lift_x = float(np.sum(quarter * chords * ds) / np.sum(chords * ds))
        self.position = np.array([float(np.sum(hinge_x * chords * ds) / np.sum(chords * ds)), 0.0,
                                  float(np.sum(hinge_z * chords * ds) / np.sum(chords * ds))])


def drag_devices(aircraft):
    return [DragDevice(d, aircraft) for d in aircraft.spec.get("drag_device", [])]


#: a landing gear's components' drag per their frontal area (Raymer, Aircraft Design: A Conceptual Approach, 12.5.6,
#: Table 12.6): a regular wheel and tyre (its diameter by its width), a wheel and tyre in tandem behind another, and a
#: round strut (its diameter by its length)
GEAR_WHEEL = 0.25
GEAR_TANDEM = 0.15
GEAR_STRUT = 0.30


def gear_drag(aircraft):
    """The retractable gear's drag extended, by Raymer's component buildup (docs/hangar.md, Gear drag): each leg's wheels -
    its first axle's at GEAR_WHEEL, a bogie's further axles' and a leg's in the wake of one ahead of it at GEAR_TANDEM -
    and its strut, from its hinge to its axle, at GEAR_STRUT. (D/q in m2, where it acts [x, y, z]), or None without
    retractable gear; fixed gear's is in the drag area already (aero.model, Raymer 12.5.6)."""
    from .shape.gear import legs
    out = [leg for leg in legs(aircraft) if leg.retractable]
    if not out:
        return None
    total, moment = 0.0, np.zeros(3)
    for leg in out:
        centres, span = leg.wheel_centres(0)
        # in the wake of a leg ahead of it: on its side, overlapping it across, within five of its wheels' diameters
        wake = any(o is not leg and leg.axle[0] - 10.0 * o.r < o.axle[0] < leg.axle[0] - 1e-6 and
                   abs(o.axle[1] - leg.axle[1]) < 0.5 * (span + o.wheel_centres(0)[1]) for o in out)
        wheel = 2.0 * leg.r * leg.w
        first = (GEAR_TANDEM if wake else GEAR_WHEEL) * wheel * leg.wheels
        rest = GEAR_TANDEM * wheel * leg.wheels * (leg.axles - 1)
        length = float(np.linalg.norm(leg.hinge - leg.axle))
        strut = GEAR_STRUT * 2.0 * leg.strut_r * length
        total += first + rest + strut
        moment += (first + rest) * leg.axle + strut * 0.5 * (leg.hinge + leg.axle)
    return total, moment / total
