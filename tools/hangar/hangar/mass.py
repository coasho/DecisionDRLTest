"""Mass properties: empty mass, centre of gravity and inertia tensor.

Each component's mass is either given in the spec or estimated with Raymer's
weight equations ("Aircraft Design: A Conceptual Approach": general aviation,
15.3.3; fighter/attack, 15.3.1, for a jet); the systems and equipment make up
the rest of a given empty mass, spread through the fuselage.
Masses are spread over the geometry that carries them - a wing's over its
skin, a fuselage's over its skin, an engine at its mounting - so the inertia
comes from the shape, not from a radius-of-gyration guess. When the spec
gives the empty centre of gravity (from a weight-and-balance report), the
systems mass is placed to meet it.

JSBSim takes the empty aircraft's CG and inertia (about the CG, the
structural frame's products); the payload and fuel are its own point masses
and tanks.
"""
import numpy as np

LB, FT, IN = 0.45359237, 0.3048, 0.0254
G0 = 9.80665


def _tri_area_centroid(v, t):
    a = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1)
    return a, v[t].mean(axis=1)


def _raymer(aircraft, mtow_kg, q_cruise, n_ult=5.7, fuel_kg=0.0):
    """Raymer's general-aviation structural weights (kg) - wing, tails,
    fuselage, landing gear - from the design's own geometry."""
    a = aircraft
    W = mtow_kg / LB
    NzW = n_ult * W
    q = q_cruise * 0.020885  # Pa -> lb/ft2
    out = {}
    for s in a.surfaces:
        S = s.area / FT**2
        lam = max(s.taper, 0.05)
        tc = s.thickness_ratio
        cosL = np.cos(np.radians(s.sweep_deg(0.25)))
        A = s.aspect_ratio if s.mirror else s.aspect_ratio
        if s.kind in ("wing",):
            Wfw = max(fuel_kg / LB, 1.0)
            out[s.name] = 0.036 * S**0.758 * Wfw**0.0035 * (A / cosL**2) ** 0.6 * q**0.006 * lam**0.04 \
                * (100 * tc / cosL) ** -0.3 * NzW**0.49
        elif s.kind in ("htail", "canard"):
            out[s.name] = 0.016 * NzW**0.414 * q**0.168 * S**0.896 * (100 * tc / cosL) ** -0.12 \
                * (A / cosL**2) ** 0.043 * lam**-0.02
        else:  # fins
            out[s.name] = 0.073 * NzW**0.376 * q**0.122 * S**0.873 * (100 * tc / cosL) ** -0.49 \
                * (A / cosL**2) ** 0.357 * lam**0.039
    tails = [s for s in a.surfaces if s.kind in ("htail", "fin", "vtail")]
    for b in a.bodies:
        wet = 0.0
        v, t, _ = b.skin(40, 24)
        wet = _tri_area_centroid(v, t)[0].sum() / FT**2 / len(b.copies())
        if b.kind == "fuselage":
            lt = (np.mean([s.mac[1][0] for s in tails]) - a.aero_point[0]) / FT if tails else b.length / FT * 0.5
            ld = b.length / max(b.max_height, 0.1)
            out[b.name] = (0.052 * wet**1.086 * NzW**0.177 * max(lt, 1.0) ** -0.051 * ld**-0.072 * q**0.241) * len(b.copies())
        else:  # nacelles, booms, pods: a light fairing
            out[b.name] = 0.25 * wet * len(b.copies())  # ~1.2 kg/m2 of skin
    nl = 4.5  # ultimate landing load factor
    for g in a.gear:
        n = 2 if g.mirror else 1
        length = (g.attach[2] - g.position[2]) / IN if g.attach is not None else 24.0
        if g.steerable or abs(g.position[1]) < 0.05:
            out[g.name] = 0.125 * (nl * W) ** 0.566 * (length / 12) ** 0.845
        else:
            out[g.name] = 0.095 * (nl * W) ** 0.768 * (length / 12) ** 0.409 * (n / 2.0)
        if g.retractable:
            out[g.name] *= 1.2
    return {k: v * LB for k, v in out.items()}


def _raymer_fighter(aircraft, dg_kg, fuel_kg=0.0, n_limit=9.0, max_mach=2.0):
    """Raymer's fighter/attack structural weights (kg; 15.3.1, eqs. 15.1-15.5,
    15.8-15.9) - wing, tails, fuselage, landing gear - from the geometry."""
    a = aircraft
    W = dg_kg / LB
    Nz = 1.5 * n_limit
    out = {}
    tails = [s for s in a.surfaces if s.kind in ("htail", "canard")]
    fins = [s for s in a.surfaces if s.kind in ("fin", "vtail")]
    fus = [b for b in a.bodies if b.kind == "fuselage"]
    delta = not any(s.kind == "htail" for s in a.surfaces)
    wing = a.wing
    for s in a.surfaces:
        S = s.area / FT**2
        lam = max(s.taper, 0.05)
        cosL = np.cos(np.radians(s.sweep_deg(0.25)))
        tc = s.sections[0].airfoil.thickness_ratio
        if s is wing or s.kind == "strake":
            ref = wing
            Sw = ref.area / FT**2
            Scs = sum(c_area(ref, c) for c in ref.controls) / FT**2
            w = 0.0103 * (0.768 if delta else 1.0) * (W * Nz) ** 0.5 * Sw**0.622 * ref.aspect_ratio**0.785 \
                * max(ref.sections[0].airfoil.thickness_ratio, 0.02) ** -0.4 * (1 + max(ref.taper, 0.05)) ** 0.05 \
                / np.cos(np.radians(ref.sweep_deg(0.25))) * max(Scs, 1.0) ** 0.04
            out[s.name] = w * (S / Sw)          # a strake: the wing's weight per area
        elif s.kind in ("htail", "canard"):
            fw = 0.0
            if fus:
                x = s.sections[0].le[0] + 0.5 * s.sections[0].chord
                fw = float(fus[0].section(x)[0]) / FT
            bh = s.span / FT
            out[s.name] = 3.316 * (1 + fw / max(bh, 1e-3)) ** -2.0 * (W * Nz / 1000.0) ** 0.260 * S**0.806
        else:
            n = 2 if s.mirror else 1
            Sv = S / n
            arm = max((s.mac[1][0] + 0.25 * s.mac[0] - a.aero_point[0]) / FT, 3.0)
            Sr = sum(c_area(s, c) for c in s.controls) / FT**2 / n
            A = (s.half_arc / FT) ** 2 / Sv
            out[s.name] = n * 0.452 * (W * Nz) ** 0.488 * Sv**0.718 * max_mach**0.341 / arm * (1 + Sr / Sv) ** 0.348 \
                * A**0.223 * (1 + lam) ** 0.25 * cosL**-0.323
    for b in a.bodies:
        if b.kind == "fuselage":
            out[b.name] = 0.499 * (0.774 if delta else 1.0) * W**0.35 * Nz**0.25 * (b.length / FT) ** 0.5 \
                * (b.max_height / FT) ** 0.849 * (b.max_width / FT) ** 0.685
        else:
            v, t, _ = b.skin(40, 24)
            out[b.name] = 0.25 * _tri_area_centroid(v, t)[0].sum() / FT**2 / len(b.copies())  # a light fairing, ~1.2 kg/m2
    Nl = 1.5 * 4.0
    Wl = W - 0.5 * fuel_kg / LB
    for g in a.gear:
        length = (g.attach[2] - g.position[2]) / IN if g.attach is not None else 40.0
        if g.steerable or abs(g.position[1]) < 0.05:
            out[g.name] = (Wl * Nl) ** 0.290 * length**0.5 * 1.0
        else:
            out[g.name] = (Wl * Nl) ** 0.25 * length**0.973 * (2 if g.mirror else 1) / 2.0
    return {k: v * LB for k, v in out.items()}


def c_area(surface, ctrl, n=40):
    """Planform area of a control surface (m2, both halves of a mirrored surface)."""
    e = np.linspace(ctrl.eta0, ctrl.eta1, n + 1)
    mid = 0.5 * (e[1:] + e[:-1])
    ds = np.diff(e) * surface.half_arc
    area = sum(surface.station(x)[1] * ctrl.chord_fraction(x) * d for x, d in zip(mid, ds))
    return area * (2 if surface.mirror else 1)


class MassModel:
    def __init__(self, aircraft):
        self.aircraft = a = aircraft
        spec = a.spec.get("mass", {})
        self.spec = spec
        self.payload = [dict(p) for p in spec.get("payload", [])]
        self.tanks = [dict(t) for t in spec.get("tank", [])]
        for t in self.tanks:
            if not 0.0 <= float(t.get("fill", 1.0)) <= 1.0:
                raise ValueError("[[mass.tank]] %r: fill is the part of its capacity it holds, 0 to 1" % t.get("name"))
        fuel = sum(t.get("capacity", 0.0) for t in self.tanks)
        self.empty_target = spec.get("empty")
        self.mtow = float(spec.get("mtow", (self.empty_target or 0) + sum(p["mass"] for p in self.payload) + fuel))
        if not self.mtow:
            raise ValueError("[mass]: give 'empty' (kg) or 'mtow'")
        q = 0.5 * 1.0 * float(spec.get("cruise_speed", a.spec.get("analysis", {}).get("speed", 50.0))) ** 2
        self.jet = any(e.type == "turbofan" for e in a.engines)
        if self.jet:
            fc = a.spec.get("flight_control", {})
            est = _raymer_fighter(a, self.mtow, fuel_kg=fuel, n_limit=float(fc.get("n_max", 9.0)),
                                  max_mach=float(a.spec.get("targets", {}).get("max_mach", 2.0)))
        else:
            est = _raymer(a, self.mtow, q, fuel_kg=fuel)
        self.estimates = est
        given = spec.get("component", {})
        self.items = []   # (name, kg, points (N, 3), weights (N,))
        for s in a.surfaces:
            v, t, g = s.skin(24, 12)
            area, cen = _tri_area_centroid(v, t)
            keep = g >= 0
            self._add(s.name, given.get(s.name, est.get(s.name, 0.0)), cen[keep], area[keep], "structure")
        for b in a.bodies:
            v, t, _ = b.skin(40, 24)
            area, cen = _tri_area_centroid(v, t)
            self._add(b.name, given.get(b.name, est.get(b.name, 0.0)), cen, area, "structure")
        for g in a.gear:
            pts = np.array([p for _, p in g.positions()]) + np.array([0.0, 0.0, 0.5 * g.wheel_diameter])
            m = given.get(g.name, est.get(g.name, 0.0))
            self._add(g.name, m, pts, np.ones(len(pts)), "gear")
        for e in a.engines:
            m = float(e.mass) if e.mass is not None else self._engine_mass(e)
            for name, pos, prop, _ in e.copies():
                if e.type == "turbofan":
                    # a jet engine is long: its mass along a cylinder centred on its position
                    length, diameter = e.jet_size()
                    xs = np.linspace(-0.5, 0.5, 7) * length
                    ring = [(0.3 * diameter * np.cos(t), 0.3 * diameter * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 6, endpoint=False)]
                    pts = pos + np.array([[x, yy, zz] for x in xs for yy, zz in ring])
                    self._add(name, m, pts, np.ones(len(pts)), "engine")
                    continue
                # engine block around its mounting point, the propeller at its hub
                pts = pos + np.array([[dx, dy, dz] for dx in (-0.25, 0.25) for dy in (-0.2, 0.2) for dz in (-0.15, 0.15)])
                if e.prop_mass is not None:     # the propeller weighed apart from the engine
                    self._add(name, m, pts, np.ones(len(pts)), "engine")
                    self._add(name + " propeller", e.prop_mass, prop[None, :], np.ones(1), "engine")
                    continue
                self._add(name, 0.9 * m, pts, np.ones(len(pts)), "engine")
                self._add(name + " propeller", 0.1 * m, prop[None, :], np.ones(1), "engine")
        for item in spec.get("item", []):   # anything else, placed by hand
            self._add(item["name"], float(item["mass"]), np.asarray([item["position"]], float), np.ones(1), "item")
        # systems and equipment: the rest of the empty mass, in the cabin (or where
        # it puts the empty CG on the given one)
        structure = sum(m for _, m, _, _, _ in self.items)
        self.systems = 0.0
        if self.empty_target is not None:
            self.systems = float(self.empty_target) - structure
            if self.systems < 0:
                # the estimates overshoot: scale the estimated structure down to fit
                est_names = {n for n in est if n not in given}
                scale = (float(self.empty_target) - (structure - sum(m for n, m, *_ in self.items if n in est_names))) \
                    / max(sum(m for n, m, *_ in self.items if n in est_names), 1e-9)
                scale = max(scale, 0.3)
                self.items = [(n, m * scale if n in est_names else m, p, w, k) for n, m, p, w, k in self.items]
                self.systems = float(self.empty_target) - sum(m for _, m, _, _, _ in self.items)
            self.systems = max(self.systems, 0.0)
        if self.systems > 0:
            spread = self._systems_spread() if self.jet else None
            if spread is not None:
                self._add("systems", self.systems, spread[0], spread[1], "systems")
            else:
                pos = self._systems_position()
                self._add("systems", self.systems, pos[None, :], np.ones(1), "systems")

    def _add(self, name, mass, pts, weights, kind):
        if mass <= 0:
            return
        w = np.asarray(weights, float)
        self.items.append((name, float(mass), np.asarray(pts, float), w / w.sum(), kind))

    def _engine_mass(self, e):
        """Installed piston engine (Raymer 15.3.3: 2.575 W_en^0.922, W_en
        ~1.4 lb/hp dry) or an electric motor with controller (~5 kW/kg)."""
        if e.type == "turbofan":
            # Raymer 10.4, afterburning turbofan: W = 0.063 T^1.1 M^0.25 exp(-0.81 BPR) lb
            t = (e.thrust_wet_kn or e.thrust_dry_kn) * 1000.0 / 4.448222
            return 0.063 * t ** 1.1 * e.design_mach ** 0.25 * np.exp(-0.81 * e.bypass_ratio) * LB
        hp = e.power_kw / 0.7457
        if e.type == "electric":
            return e.power_kw / 5.0 * 1.3
        if e.type == "turboprop":
            # a turboprop's dry weight: about 4 kW of rated power per kg (the
            # AE 2100D3's 3,458 kW weigh 790 kg, the T56-A-14's 3,424 kW 857 kg)
            return e.power_kw / 4.0
        dry = 1.4 * hp
        return 2.575 * dry**0.922 * LB * 0.75

    def _systems_position(self):
        """Where the systems mass goes: at the given empty CG's position
        solved for, or 30 % down the fuselage."""
        target = self.spec.get("empty_cg")
        m0 = sum(m for _, m, _, _, _ in self.items)
        c0 = sum(m * (w[:, None] * p).sum(axis=0) for _, m, p, w, _ in self.items) / max(m0, 1e-9)
        if target is not None:
            target = np.asarray(target, float)
            return (target * (m0 + self.systems) - c0 * m0) / self.systems
        fus = [b for b in self.aircraft.bodies if b.kind == "fuselage"]
        if fus:
            b = fus[0]
            x = b.x[0] + 0.3 * b.length
            w, top, bot, yc, _ = b.section(x)
            return np.array([x, 0.0, 0.5 * (top + bot)])
        return self.aircraft.aero_point.copy()

    def _systems_spread(self):
        """A fighter's systems (avionics, fuel system, hydraulics, ECS, ...)
        fill its fuselage: points along its axis from 5 to 90 % of its length,
        weighted by the section area, tilted fore or aft to put the empty CG
        where the spec gives it. None if no tilt reaches it: then they go to
        the one point that does (spreading part of them further out would
        only add inertia)."""
        fus = [b for b in self.aircraft.bodies if b.kind == "fuselage"]
        if not fus:
            return None
        b = fus[0]
        xs = b.x[0] + b.length * np.linspace(0.05, 0.9, 40)
        w, top, bot, yc, _ = b.section(xs)
        pts = np.column_stack([xs, yc, 0.5 * (top + bot)])
        area = np.maximum(b.area_at(xs), 1e-6)
        target = self.spec.get("empty_cg")
        if target is None:
            return pts, area
        x_s = self._systems_position()[0]
        xm = float(np.average(xs, weights=area))
        span = xs[-1] - xs[0]

        def centroid(t):
            wt = area * np.maximum(1.0 + t * (xs - xm) / span, 0.0)
            return float(np.average(xs, weights=wt)), wt
        lo, hi = -2.0, 2.0
        c_lo, _ = centroid(lo)
        c_hi, _ = centroid(hi)
        if not (min(c_lo, c_hi) <= x_s <= max(c_lo, c_hi)):
            return None
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            c, _ = centroid(mid)
            if (c - x_s) * (c_hi - x_s) > 0:
                hi, c_hi = mid, c
            else:
                lo = mid
        _, wt = centroid(0.5 * (lo + hi))
        # the vertical position as the spec's CG asks, by a common offset
        z_s = self._systems_position()[2]
        pts[:, 2] += z_s - float(np.average(pts[:, 2], weights=wt))
        return pts, wt

    # -- results --------------------------------------------------------------------------------
    def empty(self):
        """Mass, CG and inertia (about the CG; the structural frame's
        moments and plain products of inertia) of the empty aircraft."""
        m = sum(mi for _, mi, _, _, _ in self.items)
        cg = sum(mi * (w[:, None] * p).sum(axis=0) for _, mi, p, w, _ in self.items) / m
        J = np.zeros((3, 3))
        for _, mi, p, w, _ in self.items:
            r = p - cg
            dm = mi * w
            J += np.einsum("n,ni,nj->ij", dm, r, r)
        # moments of inertia and the products the structural frame gives
        ixx = J[1, 1] + J[2, 2]
        iyy = J[0, 0] + J[2, 2]
        izz = J[0, 0] + J[1, 1]
        ixz = J[0, 2]
        g = self.spec.get("gyration")
        if g is not None:
            # published (or class-typical) radii of gyration, Roskam's
            # non-dimensional ones, over the build-up's: a fighter's weight
            # statement rarely says where its equipment sits
            a = self.aircraft
            b = a.span_overall
            length = a.length_overall
            k = izz
            ixx = m * (float(g[0]) * b / 2.0) ** 2
            iyy = m * (float(g[1]) * length / 2.0) ** 2
            izz = m * (float(g[2]) * (b + length) / 4.0) ** 2
            ixz *= izz / max(k, 1e-9)
        return {"mass": m, "cg": cg, "ixx": ixx, "iyy": iyy, "izz": izz,
                "ixy": J[0, 1], "ixz": ixz, "iyz": J[1, 2]}

    @staticmethod
    def fuel(tank, fuel_fraction=None):
        """The fuel in a tank (kg): its capacity times its fill ([[mass.tank]]
        fill, full unless given - what the JSBSim file starts it with), or
        times fuel_fraction when that is given."""
        f = float(tank.get("fill", 1.0)) if fuel_fraction is None else float(fuel_fraction)
        return float(tank.get("capacity", 0.0)) * f

    def loaded(self, fuel_fraction=None, payload=True):
        """Mass and CG with payload and fuel (for checks and the flight tests):
        each tank as filled, or all at fuel_fraction."""
        e = self.empty()
        m, mc = e["mass"], e["mass"] * e["cg"]
        for p in (self.payload if payload else []):
            m += p["mass"]
            mc = mc + p["mass"] * np.asarray(p["position"], float)
        for t in self.tanks:
            f = self.fuel(t, fuel_fraction)
            m += f
            mc = mc + f * np.asarray(t["position"], float)
        return m, mc / m

    def breakdown(self):
        return [(n, m, k) for n, m, _, _, k in self.items]

    def gyration(self):
        """Non-dimensional radii of gyration (Roskam's R_x, R_y, R_z) of the
        empty aircraft, to compare with the statistics of its class."""
        e = self.empty()
        a = self.aircraft
        b, length = a.span_overall, a.length_overall
        m = e["mass"]
        return {"Rx": 2 * np.sqrt(e["ixx"] / m) / b, "Ry": 2 * np.sqrt(e["iyy"] / m) / length,
                "Rz": 2 * np.sqrt(e["izz"] / m) / (0.5 * (b + length))}
