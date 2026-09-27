"""What the platform's discovery rests on (docs/flight-autonomy.md, section 5.2):
the aircraft's physical characteristics as its design declares them, each with
the public source it rests on.

    [applicability]
    vertical_flight = false         # holds a point in the air (rule R1)
    ground_contact = "wheels"       # wheels, skids or legs (R2)
    carrier = "none"                # none, catapult_arrested or deck (R3 to R5)
    retractable_gear = true         # (R6)
    flaps = true                    # a flap function (R7)
    drag_devices = "devices"        # none, devices (spoilers, airbrakes) or surfaces the flight
                                    # control system deploys as a speedbrake (R8)
    releasable_stores = "weapons"   # none, weapons, palletized or dispensers (R9)
    aerobatic = true                # cleared for aerobatic manoeuvres (R10)

    [applicability.sources]
    vertical_flight = "..."         # the public source of each field declared

A field left out is not declared, and the platform treats what it governs as
applicable: no evidence, no exception. hangar writes the fields into the
aircraft's profile (fsim/applicability/<field>) and each source into the file
header as a <reference>, which the platform reports as the evidence.
"""
from xml.sax.saxutils import quoteattr

VERSION = 1

#: field -> its codes (None: a flag, written 0 or 1). Codes start at 1: an
#: absent field is "not declared" (include/fsim/VehicleProfile.h, ApplicabilitySection).
FIELDS = {
    "vertical_flight": None,
    "ground_contact": {"wheels": 1, "skids": 2, "legs": 3},
    "carrier": {"none": 1, "catapult_arrested": 2, "deck": 3},
    "retractable_gear": None,
    "flaps": None,
    "drag_devices": {"none": 1, "devices": 2, "surfaces": 3},
    "releasable_stores": {"none": 1, "weapons": 2, "palletized": 3, "dispensers": 4},
    "aerobatic": None,
}


def declared(spec):
    """({field: number} for the profile, {field: source}) from the design's
    [applicability]; both empty without one. Raises ValueError for a field or
    value it does not know, a field without a source, or a source without a field."""
    section = dict(spec.get("applicability") or {})
    sources = dict(section.pop("sources", {}) or {})
    fields = {}
    for name, value in section.items():
        if name not in FIELDS:
            raise ValueError("[applicability] %s: not a characteristic (%s)" % (name, ", ".join(FIELDS)))
        codes = FIELDS[name]
        if codes is None:
            if not isinstance(value, bool):
                raise ValueError("[applicability] %s: true or false, not %r" % (name, value))
            fields[name] = 1 if value else 0
        else:
            if value not in codes:
                raise ValueError("[applicability] %s: one of %s, not %r" % (name, ", ".join(codes), value))
            fields[name] = codes[value]
        source = sources.get(name)
        if not isinstance(source, str) or not source.strip():
            raise ValueError("[applicability] %s is declared without a source in [applicability.sources]" % name)
    for name in sources:
        if name not in fields:
            raise ValueError("[applicability.sources] %s: no such field is declared" % name)
    return fields, {k: " ".join(v.split()) for k, v in sources.items()}


def contradictions(spec, retractable_gear=None, rotorcraft=None, ground_contact=None):
    """What the declarations say that the design itself contradicts: the gear
    it draws (whether it retracts; "wheels", "skids" or "legs"), what kind of
    aircraft it is."""
    fields, _ = declared(spec)
    out = []
    if rotorcraft is not None and "vertical_flight" in fields and bool(fields["vertical_flight"]) != rotorcraft:
        out.append("vertical_flight = %s, but the design is %s" % (bool(fields["vertical_flight"]), "a rotorcraft" if rotorcraft else "a fixed wing"))
    if retractable_gear is not None and "retractable_gear" in fields and bool(fields["retractable_gear"]) != retractable_gear:
        out.append("retractable_gear = %s, but the design's gear %s" % (bool(fields["retractable_gear"]),
                                                                      "retracts" if retractable_gear else "does not retract"))
    if ground_contact is not None and "ground_contact" in fields and fields["ground_contact"] != FIELDS["ground_contact"][ground_contact]:
        declared_as = next(k for k, v in FIELDS["ground_contact"].items() if v == fields["ground_contact"])
        out.append("ground_contact = %s, but the design stands on %s" % (declared_as, ground_contact))
    return out


def references_xml(spec, indent):
    """The sources as JSBSim <reference> elements for the file header ("" without any)."""
    _, sources = declared(spec)
    return "".join("%s<reference refID=\"fsim/applicability/%s\" author=\"the design\" title=%s date=\"n/a\"/>\n" % (
        indent, name, quoteattr(sources[name])) for name in sorted(sources))
