"""The performance tables (ADR-29 FA-3a; docs/flight-autonomy.md, SUB-02): what a design flies level, climbs
and descends at, and the fuel it burns, against altitude, weight and speed.

Flown in the platform's own JSBSim through its velocity loop, the envelope protection off (as the flight tests
fly: they measure the aircraft, not the platform's limits), every condition at once in one world. At each
altitude (fractions of the service ceiling the flight tests found, and the height the design publishes its top
speed at) and weight (the tanks a tenth, half and wholly full of their capacity, and as the aircraft file fills them
- where its flight tests fly - if that is none of those):

- full power, level, from a little above the stall to where the drag meets the thrust: the excess power at each
  speed, the top level speed, the best climb;
- where the run did not fly, short level runs at full power, as the flight tests' ceiling runs are flown: below
  where a run begun faster began (near the ceiling an aircraft's best climb may lie between 1.2 and 1.8 times the
  stall), and past a drag rise the run could not accelerate through (a fighter high and heavy: a transonic pocket) -
  the excess power there, its best climb (up high a fighter's is supersonic) and the top speed where full power
  holds level;
- idle, level, from the top speed down: the excess power at idle - the descent rate and the deceleration at each
  speed;
- level flight at sixteen speeds from the slowest full power flew level at to 97 % of the top: the fuel flow (the
  engines', over ten seconds), the throttle and the angle of attack - each point counted only where it held its speed
  (within 2 %) and its height (within 1 m/s), off the ground. From them the best-endurance speed (the least fuel flow)
  and the best-range speed (the most distance per kilogram), within the band. An aircraft that flies on a battery
  (FA-3b: the Skua) has the power its battery gives in place of the fuel flow, and its best speeds are the least
  power's and the most distance per joule's;
- the stall, as the flight tests fly it: the idle run's least speed before it reached the limit angle of attack, broke
  or could no longer hold its height - at every weight from the highest lift coefficient any weight's run reached at
  that altitude (a lift coefficient does not depend on the weight; a light aircraft's loop at low dynamic pressure
  can oscillate and end its run early, reading its stall high).

Near its ceiling an aircraft cannot fly level at 1.2 times the stall: its full-power run begins faster there
(STARTS). Each flies at its weight, the fuel frozen (JSBSim's fuel freeze) but for the level points' last ten
seconds, when the engines report the fuel they burn. The velocity loop holds a vertical speed, not a height (a
fighter accelerating through Mach 1 at 100 m sinks 2 m/s, into the ground): the vertical speed asked of it follows
the height (HOLD_GAIN), and the excess power counts the climb or sink it makes. A run that came within
GROUND_CLEARANCE_M of the ground (on its wheels, or - a fighter's gear up - on its belly) flew nothing level.
"""
import math
import time

import numpy as np

KT = 0.514444
LBF = 4.448222
LBS = 0.45359237
FT = 0.3048
G0 = 9.80665
RHO0 = 1.225

ALTITUDE_FRACTIONS = (0.0, 0.2, 0.4, 0.6, 0.75, 0.85, 0.95)  # of the service ceiling (closer near it); the lowest at LOWEST_M
FUEL_FRACTIONS = (0.1, 0.55, 1.0)                              # of the tanks' capacity: the weights...
SPAWN_APART = 0.05                                             # ...and as it spawns, unless within this of one
SPEED_FRACTIONS = tuple(k / 15.0 for k in range(16))           # between the slowest level speed and 97 % of the top
LOWEST_M = 100.0
EXTRA_APART_M = 300.0  # a published height nearer a row than this is not flown again
SETTLE_S = 40.0        # a level point settles...
AVERAGE_S = 10.0       # ...then is averaged: counted if it held its speed within HELD_SPEED and its height within HELD_VS_MS
HELD_SPEED = 0.02
HELD_VS_MS = 1.0
BAND_MIN = 1.05        # a band narrower than this (97 % of the top over the least speed) is none: at its ceiling
CURVE_END = 0.03       # a curve's end value stands this far past its first or last speed (a run is recorded from
                       # half a second in: its first speed is a little past where the band begins)
HEIGHT_LOST_M = 20.0   # near the stall, a height lost this far is a sink too (a light, slow aircraft glides down gently)
SPOOL_S = 8.0          # a full-power run's first seconds, the engines spooling up (a turboprop's excess power climbs
                       # from 7 to 11 m/s over 6 s), are left out of its curve...
ELECTRIC_SPOOL_S = 1.0  # ...an electric motor's first second only: it gives full power at once. A light electric
                        # aircraft (the Skua) is past its best-endurance and best-range speeds within that second,
                        # so its level points begin at the envelope's least speed, each flown and held (or none)
FULL_SECONDS = 1200.0  # the full-power run's longest (a heavy jet at altitude settles slowly; a run settled ends there)
IDLE_SECONDS = 900.0   # the idle run's longest (a bomber loses speed slowly)
STARTS = (1.2, 1.8, 2.6)  # a full-power run's start, times the stall: faster again where it could not fly level
SAMPLE_S = 0.5         # the runs' histories, every half second
ENGINE_ELECTRIC = 4    # fsim/propulsion/type (profile.ENGINE_TYPES)
HOLD_GAIN = 0.1        # 1/s: the vertical speed asked per metre off the height...
HOLD_MAX_MS = 3.0      # ...at most
GROUND_CLEARANCE_M = 10.0  # a run whose centre of gravity came this near the ground touched it (gear up, on its belly)
SPOT_FACTORS = (1.2, 1.35, 1.5, 1.65, 1.8, 2.0, 2.2, 2.4, 2.6)  # times the stall: short runs below where a run began
SPOT_MACHS = (1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4)          # past a drag rise: the flight tests' ceiling runs' supersonic ones
SPOT_SETTLE_S = 10.0   # a short run: level at its speed...
SPOT_RUN_S = 20.0      # ...then at full power, the excess power over its last half


def sigma(h):
    """The ISA density over sea level's at h metres."""
    if h <= 11000.0:
        return (1.0 - 2.25577e-5 * h) ** 4.2559
    return 0.29708 * math.exp(-(h - 11000.0) / 6341.6)


def speed_of_sound(h):
    return math.sqrt(1.4 * 287.05 * (288.15 - 0.0065 * min(h, 11000.0)))


def altitudes(ceiling_m, extra=()):
    """The table's altitudes: fractions of the service ceiling, the lowest a little above the ground, each to 10 m;
    and each of `extra` (the height the design publishes its top speed at, where its flight tests fly it) between the
    lowest and the highest, unless within EXTRA_APART_M of one of them."""
    hs = [max(LOWEST_M, round(f * ceiling_m / 10.0) * 10.0) for f in ALTITUDE_FRACTIONS]
    for h in extra:
        h = float(h)
        if math.isfinite(h) and hs[0] < h < hs[-1] and min(abs(h - x) for x in hs) > EXTRA_APART_M:
            hs.append(round(h, 1))
    return sorted(hs)


class _Fleet:
    """Vehicles flying at once in one world, their states read together."""

    def __init__(self, fsim, world):
        self.fsim, self.w = fsim, world
        self.n = 0
        self.tanks = None
        self.capacity_lbs = []  # each tank's
        self.spawn_fuel_lbs = 0.0  # what the aircraft file starts it with (a B-52H's tanks: 40 %)
        self.battery_j = None  # its battery's capacity (J), if it flies on one

    def spawn(self, kind, h, v, fuel_fraction=None):
        """A vehicle at height h (m) and true airspeed v (m/s), the envelope protection off, its tanks at
        fuel_fraction of their capacity (None: as the aircraft file fills them), the fuel frozen."""
        self.n += 1
        veh = self.w.create_vehicle("perf-%d" % self.n, type=kind, latitude_deg=30.0 + 0.05 * (self.n % 400), longitude_deg=0.0,
                                    altitude_msl_m=h, airspeed_ms=v, heading_deg=0.0)
        veh.set_protection("off")
        if self.tanks is None:  # counted once, by asking past the last (quietly); each one's capacity from its fill
            level = self.fsim._native.log_level()
            self.fsim.set_log_level("off")
            try:
                self.tanks = next(i for i in range(64) if not np.isfinite(self.prop(veh, "propulsion/tank[%d]/contents-lbs" % i)))
                battery = self.prop(veh, "fsim/battery/capacity-j")
                self.battery_j = float(battery) if np.isfinite(battery) else None
            finally:
                self.fsim.set_log_level(level)
            for i in range(self.tanks):
                contents, pct = veh.get_property("propulsion/tank[%d]/contents-lbs" % i), veh.get_property("propulsion/tank[%d]/pct-full" % i)
                self.capacity_lbs.append(contents / (pct / 100.0) if pct > 0.0 else contents)
                self.spawn_fuel_lbs += contents
        for i in range(self.tanks if fuel_fraction is not None else 0):  # every tank to the same fraction of its capacity
            veh.set_property("propulsion/tank[%d]/contents-lbs" % i, self.capacity_lbs[i] * fuel_fraction)
        # and kept there: a condition is flown at its weight (an afterburner burns a fifth of a fighter in the
        # minutes a run takes), its engines still reporting the fuel they burn
        veh.set_property("propulsion/fuel_freeze", 1)
        return veh

    def fuel_flow(self, veh):
        """The fuel its engines burn now, kg/s."""
        return sum(self.prop(veh, "propulsion/engine[%d]/fuel-flow-rate-pps" % e, 0.0) for e in range(veh.state.engine_count)) * LBS

    @staticmethod
    def prop(veh, name, default=float("nan")):
        try:
            return veh.get_property(name)
        except Exception:
            return default

    def level(self, veh, v):
        """Level flight at true airspeed v (m/s), heading north, through the velocity loop: its activity."""
        return veh.submit(self.fsim.Level.VELOCITY, airspeed_ms=float(v), vertical_speed_ms=0.0, heading_rad=0.0)

    def power(self, veh, throttle):
        """Level flight at a throttle held: the velocity loop flies the vertical speed and the heading on the pitch
        and lateral axes, and the throttle owns the thrust (the loop's airspeed hold would pace the acceleration).
        The velocity loop's activity."""
        Axis = self.fsim.Axis
        act = veh.submit(self.fsim.Level.VELOCITY, vertical_speed_ms=0.0, heading_rad=0.0, axes=Axis.LATERAL | Axis.PITCH)
        veh.submit(self.fsim.Level.ACTUATOR, throttle=float(throttle), axes=Axis.THRUST)
        return act

    @staticmethod
    def hold(act, h_ref, h, **fields):
        """The height held: the vertical speed the velocity loop's activity asks for, from the height it is off
        (fields: the activity's others, an airspeed)."""
        act.update(vertical_speed_ms=float(np.clip(HOLD_GAIN * (h_ref - h), -HOLD_MAX_MS, HOLD_MAX_MS)), heading_rad=0.0, **fields)


def _grounded(st):
    """Whether a vehicle is on the ground, or as good as: on its wheels, or its centre of gravity within
    GROUND_CLEARANCE_M of it (a fighter flies with its gear up: on its belly the wheels report nothing)."""
    return bool(st.on_ground) or st.altitude_agl_m < GROUND_CLEARANCE_M


def _rate(t, x, window):
    """dx/dt by a least-squares line over a sliding window (s); NaN where too few points."""
    out = np.full(len(t), np.nan)
    half = window / 2.0
    j0 = j1 = 0
    for i, ti in enumerate(t):
        while t[j0] < ti - half:
            j0 += 1
        while j1 < len(t) and t[j1] <= ti + half:
            j1 += 1
        if j1 - j0 >= 4:
            out[i] = np.polyfit(t[j0:j1], x[j0:j1], 1)[0]
    return out


def _excess_power(hist, window=6.0):
    """(speed, excess power) along a level run: the climb it makes plus the speed it gains, as a rate of height - the
    energy height's rate over the window (its climb and its speed read over the same seconds: the height held, the
    two trade)."""
    t, v, h = (np.asarray(hist[k], dtype=float) for k in ("t", "v", "h"))
    if len(t) < 8:
        return np.array([]), np.array([])
    ps = _rate(t, h + v * v / (2.0 * G0), window)
    ok = np.isfinite(ps)
    return v[ok], ps[ok]


def _settled(t, v):
    """The speed a full-power run settles at: its last speeds once settled, else its acceleration over the last
    minute against the speed extrapolated to zero - never below where it got, and never more than 2 % past the
    speeds it flew (a steep drag rise has no line to extrapolate along: the last speeds, then)."""
    t, v = np.asarray(t), np.asarray(v)
    k = t > t[-1] - 60.0
    dv = _rate(t, v, 10.0)
    ok = k & np.isfinite(dv)
    last = float(np.mean(v[t > t[-1] - 10.0]))
    if not np.any(ok) or abs(float(np.mean(dv[ok]))) < 0.01 or float(np.ptp(v[ok])) < 0.01 * last:
        return last
    slope, icpt = np.polyfit(v[ok], dv[ok], 1)
    est = float(-icpt / slope) if slope < 0 else last
    est = max(est, last) if float(np.mean(dv[ok])) > 0 else min(est, last)
    lo, hi = float(np.min(v[ok])), float(np.max(v[ok]))
    return est if 0.98 * lo <= est <= 1.02 * hi else last


def _along_curve(v, speeds, powers):
    """A curve's value at speed v (its speeds rising): linear between its points; within CURVE_END past either end,
    that end's value; NaN beyond."""
    if speeds[0] * (1.0 - CURVE_END) <= v < speeds[0]:
        return float(powers[0])
    if speeds[-1] < v <= speeds[-1] * (1.0 + CURVE_END):
        return float(powers[-1])
    return float(np.interp(v, speeds, powers, left=np.nan, right=np.nan))


def _extremum(x, y, most):
    """Where y is least (most=False) or most over samples x, refined by the parabola through its neighbours."""
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = np.asarray(x)[ok], np.asarray(y)[ok]
    if len(x) < 3:
        return float("nan"), float("nan")
    i = int(np.argmax(y) if most else np.argmin(y))
    if i == 0 or i == len(x) - 1:
        return float(x[i]), float(y[i])  # (at an end of the speeds flown: that end)
    a, b, c = np.polyfit(x[i - 1:i + 2], y[i - 1:i + 2], 2)
    if (a < 0) != most or a == 0:
        return float(x[i]), float(y[i])
    xm = float(np.clip(-b / (2 * a), x[i - 1], x[i + 1]))
    return xm, float(np.polyval([a, b, c], xm))


def fly(kind, ceiling_m, stall_tas_sl_ms, alpha_max_deg, extra_altitudes=(), workers=0, log=print, jsbsim_root=None):
    """The tables of `kind` (jsbsim:<name>): see the module. `ceiling_m` sets the altitudes (with `extra_altitudes`,
    the published top-speed height); `stall_tas_sl_ms` (at sea level, as it spawns) where the full-power runs begin;
    `alpha_max_deg` the angle the stall is read at."""
    import fsim
    kind = kind if ":" in kind else "jsbsim:" + kind
    opts = dict(publish=False, workers=workers, terrain=False)
    if jsbsim_root:
        opts["jsbsim_root"] = jsbsim_root
    world = fsim.World("hangar-performance", **opts)
    level = fsim._native.log_level()
    fsim.set_log_level("error")
    fleet = _Fleet(fsim, world)
    dt = world.step_seconds
    every = max(1, int(round(SAMPLE_S / dt)))
    hs = altitudes(ceiling_m, extra_altitudes)
    t0 = time.time()
    try:
        # the weights: a vehicle with its tanks at each fraction - and as the file fills them (where its flight
        # tests fly), a weight of its own unless near one of those
        first = fleet.spawn(kind, 1000.0, 2.0 * stall_tas_sl_ms)
        spool_s = ELECTRIC_SPOOL_S if fleet.prop(first, "fsim/propulsion/type") == ENGINE_ELECTRIC else SPOOL_S
        capacity = sum(fleet.capacity_lbs) * LBS
        spawn_fuel = fleet.spawn_fuel_lbs * LBS
        fill = spawn_fuel / capacity if capacity > 0.0 else 1.0
        fractions = sorted(set(FUEL_FRACTIONS) | ({round(fill, 4)} if min(abs(fill - f) for f in FUEL_FRACTIONS) > SPAWN_APART else set()))
        if capacity <= 0.0:
            fractions = [1.0]  # (no fuel - an electric aircraft: one weight)
        probe = [fleet.spawn(kind, 1000.0, 2.0 * stall_tas_sl_ms, f) for f in fractions]
        world.step()  # (the mass follows the tanks at the next step)
        weights = [fleet.prop(v, "inertia/weight-lbs") * LBS for v in probe]
        area = fleet.prop(probe[0], "metrics/Sw-sqft") * FT * FT
        spawn_weight = weights[-1] - (capacity - spawn_fuel)  # (as its flight tests fly it: the heaviest's less the rest)
        for v in probe + [first]:
            v.remove()
        conds = [(i, j) for i in range(len(hs)) for j in range(len(fractions))]

        def stall_tas(i, j):
            return stall_tas_sl_ms * math.sqrt(weights[j] / spawn_weight / sigma(hs[i]))

        # -- full power, level, from a little above the stall (faster again where it could not fly level) ---------
        full = {}
        todo = list(conds)
        for factor in STARTS:
            group = {}
            for c in todo:
                i, j = c
                v0 = factor * stall_tas(i, j)
                veh = fleet.spawn(kind, hs[i], v0, fractions[j])
                fleet.level(veh, v0)
                group[c] = {"veh": veh, "h0": hs[i], "start": v0, "t": [], "v": [], "vs": [], "h": [], "alpha": [], "done": False}
            run_group(fleet, group, settle_s=20.0, seconds=FULL_SECONDS, every=every, throttle=1.0, stop=_full_done)
            todo = []
            for c, r in group.items():
                r["v_max"] = _settled(r["t"], r["v"]) if len(r["t"]) > 20 else float("nan")
                r["held"] = _held(r, hs[c[0]]) and np.isfinite(r["v_max"])
                full[c] = r
                if not r["held"]:
                    todo.append(c)
            if not todo:
                break
        log("  performance: full power (%.0f s)" % (time.time() - t0))
        # -- short runs where the runs did not fly: below a retry's start, past a drag rise -------------------
        spots, flown = _spots(fleet, kind, hs, conds, full, stall_tas, every, fractions)
        curves = {c: _curve(full[c], spots.get(c, []), 1.15 * stall_tas(*c), spool_s, from_lowest=spool_s == ELECTRIC_SPOOL_S)
                  for c in conds}
        if flown:
            log("  performance: %d short runs, %d held (%.0f s)" % (flown, sum(len(p) for p in spots.values()), time.time() - t0))
        # -- idle, level, from near the top speed down ------------------------------------------------------
        idle = {}
        for c in conds:
            top = curves[c]["top"]
            if not np.isfinite(top):
                continue
            v0 = top
            veh = fleet.spawn(kind, hs[c[0]], v0, fractions[c[1]])
            fleet.level(veh, v0)
            idle[c] = {"veh": veh, "h0": hs[c[0]], "start": v0, "stall_est": stall_tas(*c), "floor": 0.7 * stall_tas(*c), "t": [], "v": [],
                       "vs": [], "h": [], "alpha": [], "done": False}
        run_group(fleet, idle, settle_s=15.0, seconds=IDLE_SECONDS, every=every, throttle=0.0, stop=lambda r: _idle_done(r, alpha_max_deg))
        log("  performance: idle (%.0f s)" % (time.time() - t0))
        # -- level flight at sixteen speeds, a weight at a time -------------------------------------------------
        steady = {}
        for j in range(len(fractions)):
            batch = {}
            for c in conds:
                if c[1] != j or c not in idle:
                    continue
                lo, hi = curves[c]["lo"], 0.97 * curves[c]["top"]
                for k, f in enumerate(SPEED_FRACTIONS):
                    v = lo + f * (hi - lo)
                    veh = fleet.spawn(kind, hs[c[0]], v, fractions[j])
                    batch[c + (k,)] = {"veh": veh, "v": v, "h0": hs[c[0]], "act": fleet.level(veh, v)}
            _fly_level(fleet, batch, every)
            steady.update(batch)
        log("  performance: level (%.0f s)" % (time.time() - t0))
    finally:
        fsim.set_log_level(level)
        world.close()
    t = _tables(hs, weights, area, capacity, alpha_max_deg, curves, idle, steady, time.time() - t0, battery_j=fleet.battery_j)
    t["spawn_fuel_kg"], t["spawn_weight_kg"] = spawn_fuel, spawn_weight
    return t


def _fly_level(fleet, points, every):
    """Level points flown: settled at their speeds (the height held) for SETTLE_S, then - the fuel flowing, for the
    engines to report what they burn - averaged over AVERAGE_S; each point's speed, climb, angle of attack,
    throttle, fuel flow and air density recorded, and its vehicle removed."""
    world, dt = fleet.w, fleet.w.step_seconds
    vehs = list(points.items())
    for s in range(int(round(SETTLE_S / dt))):
        world.step()
        if s % every == 0:
            for _, p in vehs:
                st = p["veh"].state
                p["touched"] = p.get("touched", False) or _grounded(st)
                fleet.hold(p["act"], p["h0"], st.altitude_msl_m, airspeed_ms=p["v"])
    for _, p in vehs:
        p["veh"].set_property("propulsion/fuel_freeze", 0)
    battery = fleet.battery_j is not None
    acc = {key: np.zeros(7) for key in points}
    for s in range(int(round(AVERAGE_S / dt))):
        world.step()
        if s % every:
            continue
        for key, p in vehs:
            st = p["veh"].state
            acc[key] += (st.airspeed_true_ms, -st.velocity_ned_ms[2], math.degrees(st.alpha_rad), st.throttle_position[0],
                         fleet.fuel_flow(p["veh"]), fleet.prop(p["veh"], "fsim/battery/power-w") if battery else 0.0, 1.0)
            p["touched"] = p.get("touched", False) or _grounded(st)
            fleet.hold(p["act"], p["h0"], st.altitude_msl_m, airspeed_ms=p["v"])
    for key, p in vehs:
        a = acc[key]
        p["flown"], p["vs"], p["alpha"], p["throttle"], p["fuel_kg_s"], p["power_w"] = (a[:6] / max(a[6], 1.0)).tolist()
        p["rho"] = fleet.prop(p["veh"], "atmosphere/rho-slugs_ft3") * 515.379
        p["held"] = (not p.get("touched") and not _grounded(p["veh"].state) and abs(p["vs"]) <= HELD_VS_MS
                     and abs(p["flown"] - p["v"]) <= HELD_SPEED * p["v"])
        p["veh"].remove()


def _spots(fleet, kind, hs, conds, full, stall_tas, every, fractions):
    """Short level runs at full power where each condition's run from below did not fly, as the flight tests fly
    their ceiling runs: SPOT_SETTLE_S level at its speed, then SPOT_RUN_S at full power, the height held; its excess
    power the energy height's rate over the last half, counted only where it held its height at 1 g. Where:

    - below where the run held began, when it was a retry (near its ceiling an aircraft cannot fly level at 1.2
      times the stall, and a retry begins at 1.8: its best climb may lie between) - at SPOT_FACTORS of the stall;
      where no run held, at all of them;
    - for a design that flies supersonic, past where the run got (by 3 %): at the flight tests' Mach numbers up to
      0.1 past the fastest any run got, at no more than 1.1 times the dynamic pressure of its fastest (a drag rise
      it could not accelerate through: past it, full power may hold level again).

    ({condition: [(speed, excess power)]} of those counted, the number flown)."""
    reach = {c: r["v_max"] / speed_of_sound(hs[c[0]]) for c, r in full.items() if r["held"]}
    supersonic = bool(reach) and max(reach.values()) >= 1.05
    cap = max(reach.values()) + 0.1 if reach else 0.0
    q_max = max((0.5 * RHO0 * sigma(hs[c[0]]) * full[c]["v_max"] ** 2 for c in reach), default=0.0)
    runs = {}
    for c in conds:
        i, j = c
        r, stall = full[c], stall_tas(i, j)
        speeds = []
        if not r["held"] or r["start"] > 1.3 * stall:
            below = 0.97 * r["start"] if r["held"] else float("inf")
            speeds += [f * stall for f in SPOT_FACTORS if f * stall < below]
        if supersonic:
            a = speed_of_sound(hs[i])
            got = reach.get(c, 0.0)
            speeds += [m * a for m in SPOT_MACHS if 1.03 * got <= m <= cap and m * a >= 1.2 * stall
                       and 0.5 * RHO0 * sigma(hs[i]) * (m * a) ** 2 <= 1.1 * q_max]
        for v in speeds:
            veh = fleet.spawn(kind, hs[i], v, fractions[j])
            runs[c + (round(v, 3),)] = {"veh": veh, "h0": hs[i], "v0": v, "act": fleet.level(veh, v), "t": [], "v": [], "h": [], "nz": [],
                                        "ground": False}
    if not runs:
        return {}, 0
    world, dt = fleet.w, fleet.w.step_seconds
    live = list(runs.values())
    for s in range(int(round(SPOT_SETTLE_S / dt))):
        world.step()
        if s % every == 0:
            for r in live:
                fleet.hold(r["act"], r["h0"], r["veh"].state.altitude_msl_m, airspeed_ms=r["v0"])
    for r in live:
        r["act"] = fleet.power(r["veh"], 1.0)
    for s in range(int(round(SPOT_RUN_S / dt))):
        world.step()
        if s % every:
            continue
        for r in live:
            st = r["veh"].state
            r["t"].append(s * dt), r["v"].append(st.airspeed_true_ms), r["h"].append(st.altitude_msl_m), r["nz"].append(st.load_factor)
            r["ground"] = r["ground"] or _grounded(st)
            fleet.hold(r["act"], r["h0"], st.altitude_msl_m)
    out = {}
    for key, r in runs.items():
        r["veh"].remove()
        t, v, h, nz = (np.asarray(r[k]) for k in ("t", "v", "h", "nz"))
        k = t >= t[-1] - SPOT_RUN_S / 2.0 if len(t) else t
        if r["ground"] or np.sum(k) < 5 or abs(float(np.mean(nz[k])) - 1.0) > 0.15 or float(np.ptp(h[k])) > 100.0:
            continue
        ps = float(np.polyfit(t[k], h[k] + v[k] ** 2 / (2.0 * G0), 1)[0])
        out.setdefault(key[:2], []).append((float(np.mean(v[k])), ps))
    return out, len(runs)


def _curve(r, spots, lowest, spool_s=SPOOL_S, from_lowest=False):
    """A condition's excess power at full power along its speeds: its run from below (held: from its start to where
    it settled), the short runs below where it began, and those past where it got; its top level speed - where the
    excess power last falls through zero (a drag rise the run could not accelerate through, and full power holding
    level beyond it: past it) - and its least level speed, the slowest it flew level at with 0.25 m/s to spare
    (never below `lowest`). NaN top: nothing held level."""
    held = bool(r["held"])
    reach = float(r["v_max"]) if held else 0.0
    v, ps = _excess_power(r) if held else (np.array([]), np.array([]))
    below = []
    if held:
        spooled = float(np.interp(spool_s, r["t"], r["v"])) if len(r["t"]) else r["start"]  # (its speed once at full power)
        k = (v >= max(r["start"], spooled)) & (v <= reach)
        v, ps = v[k], ps[k]
        below = [(sv, sp) for sv, sp in spots if sv < r["start"]]
    beyond = sorted((sv, sp) for sv, sp in spots if sv > reach)
    top = reach if held else float("nan")
    prev = (reach, 0.0) if held else None
    for sv, sp in beyond:
        if sp >= 0.0:
            top = sv  # (at least as fast: still climbing there)
        elif prev is not None and prev[1] >= 0.0:
            top = prev[0] + prev[1] / (prev[1] - sp) * (sv - prev[0])
        prev = (sv, sp)
    speeds = np.concatenate([[b[0] for b in below], v, [b[0] for b in beyond]])
    powers = np.concatenate([[b[1] for b in below], ps, [b[1] for b in beyond]])
    o = np.argsort(speeds)
    speeds, powers = speeds[o], powers[o]
    # (the band begins where the curve does: its runs' slowest speed flown with the margin - a run held level at
    # its start, but its record begins as it gathers speed)
    flyable = speeds[(powers >= 0.25) & (speeds <= top)] if np.isfinite(top) else speeds[:0]
    lo = (lowest if from_lowest else max(lowest, float(flyable.min()))) if len(flyable) else float("nan")
    if not (np.isfinite(top) and np.isfinite(lo) and 0.97 * top >= BAND_MIN * lo):
        top = float("nan")  # (no band of level speeds left: at its ceiling, flown as none)
    return {"v": speeds, "ps": powers, "top": float(top), "lo": float(lo), "reach": reach if held else float("nan"),
            "spots": len(below) + len(beyond)}


def run_group(fleet, runs, settle_s, seconds, every, throttle, stop):
    """Fly each run's vehicle settled at its start for settle_s, then level at `throttle` (its height held),
    recording its history every `every` steps until stop(run) or `seconds` pass; each run's vehicle removed as it
    ends."""
    world, dt = fleet.w, fleet.w.step_seconds
    world.step(int(round(settle_s / dt)))
    live = list(runs.values())
    for r in live:
        r["act"] = fleet.power(r["veh"], throttle)
    n = int(round(seconds / dt))
    for s in range(n):
        world.step()
        if s % every or not live:
            continue
        t = (s + 1) * dt
        still = []
        for r in live:
            st = r["veh"].state
            r["t"].append(t)
            r["v"].append(st.airspeed_true_ms)
            r.setdefault("cas", []).append(st.airspeed_calibrated_ms)
            r["vs"].append(-st.velocity_ned_ms[2])
            r["h"].append(st.altitude_msl_m)
            r["alpha"].append(math.degrees(st.alpha_rad))
            r["ground"] = r.get("ground", False) or _grounded(st)
            if stop(r):
                r["veh"].remove()
                r["done"] = True
            else:
                fleet.hold(r["act"], r["h0"], st.altitude_msl_m)
                still.append(r)
        live = still
    for r in live:
        r["veh"].remove()


def _full_done(r):
    """A full-power run has settled: its speed has changed less than 0.2 % over 20 s, twice running."""
    t, v = r["t"], r["v"]
    if len(t) < 90:
        return False
    k = int(20.0 / (t[1] - t[0]))
    return abs(v[-1] - v[-1 - k]) < 0.002 * v[-1] and abs(v[-1 - k] - v[-1 - 2 * k]) < 0.004 * v[-1]


def _idle_done(r, alpha_max_deg):
    """An idle run ends as the flight tests' stall runs break (flight._stall_break): at the limit angle of attack
    (within half a degree: a law holds it there), its angle of attack falling 2 deg back from a peak within 3 deg of
    the limit, or its sink passing 4 m/s or its height falling 20 m (a light, slow aircraft glides down gently) - the
    last two only near the stall (below 1.5 times the one expected: high up and at idle an aircraft sinks well above
    it); or on the ground (no stall read), or - something amiss - well below the stall expected."""
    alpha, near = r["alpha"][-1], r["v"][-1] < 1.5 * r["stall_est"]
    r["alpha_peak"] = max(r.get("alpha_peak", -1e9), alpha)
    if r["ground"]:
        r["ended"] = "ground"
    elif alpha > alpha_max_deg - 0.5:
        r["ended"] = "alpha"
    elif near and r["alpha_peak"] > alpha_max_deg - 3.0 and alpha < r["alpha_peak"] - 2.0:
        r["ended"] = "alpha break"  # (near the limit angle: a loop's oscillation well below it is no stall)
    elif near and (r["vs"][-1] < -4.0 or r["h"][-1] < r["h0"] - HEIGHT_LOST_M):
        r["ended"] = "sink"
    elif r["v"][-1] < r["floor"]:
        r["ended"] = "slow"
    return "ended" in r


def _held(r, h):
    """Whether a full-power run flew level: at its end neither climbing nor sinking, near its height, faster than it
    began (above its ceiling it cannot: it sinks, or slows) - and never on the ground (one that sank to the ground
    settles at the speed it slides along it)."""
    return (len(r["h"]) > 0 and not r.get("ground") and abs(r["vs"][-1]) < 1.0 and abs(r["h"][-1] - h) < max(150.0, 0.03 * h)
            and r["v"][-1] > 1.05 * r["start"])


def _tables(hs, weights, area, capacity, alpha_max_deg, curves, idle, steady, seconds, battery_j=None):
    nh, nw, nk = len(hs), len(weights), len(SPEED_FRACTIONS)
    battery = battery_j is not None  # (flies on a battery: its power where a fuel burner has its fuel flow)
    grid = lambda *shape: np.full(shape, np.nan)  # noqa: E731
    t2 = {k: grid(nh, nw) for k in ("min_tas_ms", "max_tas_ms", "stall_cas_ms", "best_endurance_tas_ms", "best_endurance_fuel_kg_s",
                                     "best_range_tas_ms", "best_range_fuel_kg_s", "max_climb_ms", "climb_tas_ms", "reach_tas_ms")}
    t3 = {k: grid(nh, nw, nk) for k in ("tas_ms", "fuel_kg_s", "throttle", "alpha_deg", "ps_full_ms", "ps_idle_ms")}
    if battery:
        t2.update({k: grid(nh, nw) for k in ("best_endurance_power_w", "best_range_power_w")})
        t3["power_w"] = grid(nh, nw, nk)
    use = "power_w" if battery else "fuel_kg_s"
    for i in range(nh):
        for j in range(nw):
            c = (i, j)
            if c not in idle:
                continue
            cu = curves[c]
            t2["min_tas_ms"][c] = cu["lo"]
            t2["max_tas_ms"][c] = cu["top"]
            t2["reach_tas_ms"][c] = cu["reach"]
            vf, pf = cu["v"], cu["ps"]
            band = (vf >= cu["lo"]) & (vf <= cu["top"])
            if np.any(band):
                i_best = int(np.argmax(np.where(band, pf, -np.inf)))
                t2["max_climb_ms"][c], t2["climb_tas_ms"][c] = float(pf[i_best]), float(vf[i_best])
            vi, pi = _excess_power(idle[c])
            order_i = np.argsort(vi)
            speeds, fuel, rho, alpha = [], [], [], []
            for k in range(nk):
                # the excess power at the point's own speed along its band (the platform reads each point at its
                # fraction of the band): from the runs, whether or not its level point held
                vk = cu["lo"] + SPEED_FRACTIONS[k] * (0.97 * cu["top"] - cu["lo"])
                if len(vf) >= 2:
                    t3["ps_full_ms"][c + (k,)] = _along_curve(vk, vf, pf)
                if len(vi) >= 2:
                    t3["ps_idle_ms"][c + (k,)] = _along_curve(vk, vi[order_i], pi[order_i])
                p = steady.get(c + (k,))
                if p is None or not p.get("held"):
                    continue  # (its fuel flow: none where it did not hold its speed and height)
                v = p["flown"]
                t3["tas_ms"][c + (k,)] = v
                t3["throttle"][c + (k,)] = p["throttle"]
                t3["alpha_deg"][c + (k,)] = p["alpha"]
                t3[use][c + (k,)] = p[use]
                speeds.append(v), fuel.append(p[use]), rho.append(p["rho"]), alpha.append(p["alpha"])
            speeds, fuel, rho, alpha = (np.array(x) for x in (speeds, fuel, rho, alpha))
            if len(speeds) >= 3 and np.all(fuel > 0):
                # (fuel: the fuel flow, kg/s - or a battery's power, W)
                band = (cu["lo"], 0.97 * cu["top"])  # (a speed a point held, clipped to the band it was flown along)
                ve, fe = _extremum(speeds, fuel, most=False)
                t2["best_endurance_tas_ms"][c] = float(np.clip(ve, *band))
                t2["best_endurance_" + use][c] = fe if band[0] <= ve <= band[1] else float(np.interp(np.clip(ve, *band), speeds, fuel))
                vr, _ = _extremum(speeds, speeds / fuel, most=True)
                t2["best_range_tas_ms"][c] = float(np.clip(vr, *band))
                t2["best_range_" + use][c] = float(np.interp(np.clip(vr, *band), speeds, fuel))
    # the stall, as the flight tests fly it (idle, the height held, the speed bleeding off to the break; its least
    # speed up to it): at each weight from the highest lift coefficient any weight's run reached at the altitude
    for i in range(nh):
        cls = [2.0 * weights[j] * G0 / (RHO0 * float(min(d["cas"])) ** 2 * area) for j in range(nw)
               for d in [idle.get((i, j))] if d and d.get("ended") in ("alpha", "alpha break", "sink") and d.get("cas")]
        if cls:
            for j in range(nw):
                if (i, j) in idle:
                    t2["stall_cas_ms"][i, j] = math.sqrt(2.0 * weights[j] * G0 / (RHO0 * area * max(cls)))
    return {"altitude_m": hs, "weight_kg": weights, "speed_fraction": list(SPEED_FRACTIONS),
            "fuel_capacity_kg": None if battery else capacity, "battery_capacity_j": battery_j,
            "wing_area_m2": area, "alpha_max_deg": alpha_max_deg, **{k: v for k, v in t2.items()}, **{k: v for k, v in t3.items()},
            "spot_runs": int(sum(cu["spots"] for cu in curves.values())), "seconds": seconds}


def climbs(kind, points, jsbsim_root=None, workers=0):
    """Climbs at full power, flown to check the tables: each (altitude m, true airspeed m/s, climb m/s) begun
    level at its speed, then at full power holding that climb for 40 s; over the last 20 s, its excess power - the
    climb it made and the speed it gained or lost, as a rate of height - at the mean height and speed it flew.
    Returns [(height m, speed m/s, excess power m/s)], NaN where it could not fly it."""
    import fsim
    opts = dict(publish=False, workers=workers, terrain=False)
    if jsbsim_root:
        opts["jsbsim_root"] = jsbsim_root
    world = fsim.World("hangar-climbs", **opts)
    level = fsim._native.log_level()
    fsim.set_log_level("error")
    fleet = _Fleet(fsim, world)
    dt = world.step_seconds
    kind = kind if ":" in kind else "jsbsim:" + kind
    out = []
    try:
        vehs = []
        for h, v, climb in points:
            veh = fleet.spawn(kind, max(h, 30.0), v)  # (its tanks as the flight tests fly them: as the file fills them)
            fleet.level(veh, v)
            vehs.append((veh, climb))
        world.step(int(round(20.0 / dt)))
        Axis = fsim.Axis
        for veh, climb in vehs:
            veh.submit(fsim.Level.VELOCITY, vertical_speed_ms=float(climb), heading_rad=0.0, axes=Axis.LATERAL | Axis.PITCH)
            veh.submit(fsim.Level.ACTUATOR, throttle=1.0, axes=Axis.THRUST)
        hist = [([], [], [], []) for _ in vehs]
        n = int(round(40.0 / dt))
        for s in range(n):
            world.step()
            if s * dt < 20.0 or s % 6:
                continue
            for (veh, _), (t, v, vs, h) in zip(vehs, hist):
                st = veh.state
                t.append(s * dt), v.append(st.airspeed_true_ms), vs.append(-st.velocity_ned_ms[2]), h.append(st.altitude_msl_m)
        for (veh, _), (t, v, vs, h) in zip(vehs, hist):
            veh.remove()
            if len(t) < 10:
                out.append((float("nan"),) * 3)
                continue
            dv = np.polyfit(t, v, 1)[0]
            out.append((float(np.mean(h)), float(np.mean(v)), float(np.mean(vs) + np.mean(v) / G0 * dv)))
    finally:
        fsim.set_log_level(level)
        world.close()
    return out


SERVICE_MS = 0.508  # the service ceiling's climb: 100 ft/min


def ceiling(tables, j):
    """The service ceiling at weight j: rising through the altitudes, where the best climb first falls below 0.5 m/s
    (100 ft/min) - a height nothing held level at climbing nothing, as the flight tests count it - linear between
    that altitude and the one below; climbing at the highest flown, the last two extended, no further than as high
    again (as the platform's tablesCeilingM: a rotorcraft's lies far above its rows). NaN if it climbs at none."""
    hs = np.asarray(tables["altitude_m"], dtype=float)
    climb = np.asarray(tables["max_climb_ms"], dtype=float)[:, j]
    last = None  # the highest altitude so far that climbs
    for a in range(len(hs)):
        if np.isfinite(climb[a]) and climb[a] >= SERVICE_MS:
            last = a
            continue
        if last is not None:
            c = climb[a] if np.isfinite(climb[a]) else 0.0
            return float(hs[last] + (climb[last] - SERVICE_MS) / (climb[last] - c) * (hs[a] - hs[last]))
    if last is not None and last == len(hs) - 1 and last >= 1 and np.isfinite(climb[last - 1]) and climb[last - 1] > climb[last]:
        above = (climb[last] - SERVICE_MS) / (climb[last - 1] - climb[last]) * (hs[last] - hs[last - 1])
        return float(hs[last] + above) if above <= hs[last] else float("nan")
    return float("nan")


def ps_at_weight(tables, h, v, w, name="ps_full_ms"):
    """ps_at at weight w (kg): linear between the weights flown either side (NaN outside them; one weight flown - no
    fuel - its own)."""
    ws = np.asarray(tables["weight_kg"], dtype=float)
    if not (ws[0] - 1.0 <= w <= ws[-1] + 1.0):
        return float("nan")
    if len(ws) == 1:
        return ps_at(tables, h, v, 0, name)
    j = int(np.clip(np.searchsorted(ws, w) - 1, 0, len(ws) - 2))
    f = float(np.clip((w - ws[j]) / (ws[j + 1] - ws[j]), 0.0, 1.0))
    a, b = ps_at(tables, h, v, j, name), ps_at(tables, h, v, j + 1, name)
    return a if f == 0.0 else b if f == 1.0 else a + f * (b - a)


def ceiling_at(tables, w):
    """The service ceiling at weight w (kg): each weight's, then linear in weight (as the platform's tablesCeilingM)."""
    ws = np.asarray(tables["weight_kg"], dtype=float)
    if not (ws[0] - 1.0 <= w <= ws[-1] + 1.0):
        return float("nan")
    if len(ws) == 1:
        return ceiling(tables, 0)
    j = int(np.clip(np.searchsorted(ws, w) - 1, 0, len(ws) - 2))
    f = float(np.clip((w - ws[j]) / (ws[j + 1] - ws[j]), 0.0, 1.0))
    a, b = ceiling(tables, j), ceiling(tables, j + 1)
    return a if f == 0.0 else b if f == 1.0 else a + f * (b - a)


def ps_at(tables, h, v, j=-1, name="ps_full_ms"):
    """The excess power at altitude h (m) and true airspeed v (m/s) for weight j, read as the platform reads it
    (tablesAt, fsim/VehicleProfile.h): v's fraction of the band - from the least level speed to 97 % of the top,
    each linear in altitude - each altitude's points at that fraction (from 2 % below the band to 6 % past it), then
    linear in altitude; below the lowest altitude, its own. NaN above the highest, or outside the band."""
    hs = np.asarray(tables["altitude_m"], dtype=float)
    fr = np.asarray(tables["speed_fraction"], dtype=float)
    lo, top = np.asarray(tables["min_tas_ms"], dtype=float)[:, j], np.asarray(tables["max_tas_ms"], dtype=float)[:, j]
    ps = np.asarray(tables[name], dtype=float)[:, j]
    if not h <= hs[-1] + 1.0:
        return float("nan")
    h = max(h, hs[0])

    def mix(a, b, f):  # (as the platform's: a point's own value on it, whatever its neighbour's)
        return a if abs(f) < 1e-9 else b if abs(f - 1.0) < 1e-9 else a + f * (b - a)

    i = int(np.clip(np.searchsorted(hs, h, side="right") - 1, 0, len(hs) - 2)) if len(hs) > 1 else 0
    f = (h - hs[i]) / (hs[i + 1] - hs[i]) if len(hs) > 1 else 0.0
    i1 = min(i + 1, len(hs) - 1)
    lo_h, hi_h = mix(lo[i], lo[i1], f), 0.97 * mix(top[i], top[i1], f)
    if not hi_h > lo_h:
        return float("nan")
    x = (v - lo_h) / (hi_h - lo_h)
    if not fr[0] - 0.02 <= x <= fr[-1] + 0.06:
        return float("nan")
    k = int(np.clip(np.searchsorted(fr, x, side="right") - 1, 0, len(fr) - 2))
    g = (x - fr[k]) / (fr[k + 1] - fr[k])
    return float(mix(mix(ps[i][k], ps[i][k + 1], g), mix(ps[i1][k], ps[i1][k + 1], g), f))


def at(tables, name, h, j=-1):
    """A 2D table's value at altitude h (m) for weight j, linear in altitude (NaN outside the altitudes flown)."""
    hs = np.asarray(tables["altitude_m"])
    col = np.asarray(tables[name], dtype=float)[:, j]
    ok = np.isfinite(col)
    if np.sum(ok) < 1 or h < hs[ok][0] - 1.0 or h > hs[ok][-1] + 1.0:
        return float("nan")
    return float(np.interp(h, hs[ok], col[ok]))


def at_weight(tables, name, h, w):
    """A 2D table's value at altitude h (m) and weight w (kg): linear in altitude at each weight, then in weight
    (NaN outside the altitudes flown or the weights)."""
    ws = np.asarray(tables["weight_kg"], dtype=float)
    vals = np.array([at(tables, name, h, j) for j in range(len(ws))])
    ok = np.isfinite(vals)
    if len(ws) == 1:  # (one weight flown - no fuel: its own)
        return float(vals[0]) if abs(w - ws[0]) <= 1.0 else float("nan")
    if np.sum(ok) < 2 or not (ws[ok][0] - 1.0 <= w <= ws[ok][-1] + 1.0):
        return float("nan")
    return float(np.interp(w, ws[ok], vals[ok]))
