"""hangar: design, analyse and validate aircraft for JSBSim.

An aircraft is one TOML file (aircraft/<name>/<name>.toml): parametric
surfaces, bodies, engines, gear and masses in metres. Each stage turns it into
something checkable - drawings, aerodynamic tables, mass properties, a JSBSim
aircraft, flight tests - and writes images and numbers a person (or Claude)
can look at before the next stage. See docs/hangar.md.
"""
__version__ = "0.1.0"
