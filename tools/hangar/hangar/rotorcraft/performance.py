"""A rotorcraft's performance tables (ADR-29 FA-3b; docs/flight-autonomy.md, 4.13): the power it needs level and
the fuel or charge that costs, against altitude, weight and speed; the climb full power gives; the speeds that fly
longest and furthest - the tables the fixed wings fly (hangar/performance.py), from the hover up.

Flown in the platform's own JSBSim with the fly stage's hold (fly.Hold: the speed, the height and the heading held
through the attitudes, as a pilot flies a rotorcraft), one run at a time, each at its weight: the fuel frozen but for
a level point's last seconds, when the engines report the fuel they burn.

- The rows: four altitudes, 100 m to 3,000 m. The rotorcraft here fly low, and their models' power does not fall with
  the air's density (a helicopter's engines give their rating at any height; a multirotor's thrust is its rotors'
  speed squared), so no ceiling is flown. The weights: a helicopter's tank a tenth, half and wholly full; a
  multirotor's one weight, its battery weighing the same spent.
- The top level speed: the fastest a level trim holds within the power and the design's attitude limits (its pitch
  limit, nose down). A helicopter's engines stay short of their limit with the rotor governed; a multirotor's motors
  stay short of full thrust. It is found by doubling the speed from 1 m/s until a trim fails, then halving the
  bracket. Where full power cannot hold the hover, the least level speed is found the same way.
- Level at sixteen speeds from the hover (or that least speed) to 97 % of the top: the power - a helicopter's
  engines', a multirotor's battery's - and the fuel a helicopter burns. From them come the best-endurance speed (the
  least fuel flow or power) and the best-range speed (the most distance per kilogram or joule).
- A helicopter at full power at each of those speeds, the height free: the climb it makes, its engines just short of
  their limit and the rotor governed. The power rises to the limit over 15 s (at once, a UH-60A at 130 kt pitched
  48 deg nose down chasing its speed). The climb is averaged over half a minute: between 95 and 120 kt the UH-60A's
  hold rings in a full-power climb (a ten-second cycle of +-3 m/s of speed), and the average is the climb. It counts
  where it held its speed and the average's halves agree (a stationary average), its rotor governed. A multirotor's
  rotors are JSBSim direct thrusters here, their thrust unmoved by a climb's inflow, so a full-power climb would say
  nothing true: none is flown.

A rotorcraft neither stalls nor idles as a wing does: those tables stay empty.
"""
import math
import time

import numpy as np

from . import fly
from ..performance import SPEED_FRACTIONS, _extremum

KT = 0.514444
LB = 0.45359237
ALTITUDES_M = (100.0, 1000.0, 2000.0, 3000.0)
FUEL_FRACTIONS = (0.1, 0.55, 1.0)
TOP = 0.97               # the level points run to this fraction of the top speed
SETTLE_S = 40.0          # a level point settles...
AVERAGE_S = 5.0          # ...then is averaged, a helicopter's fuel flowing
CLIMB_RAMP_S = 15.0      # a full-power climb's power rises from the level point's to the limit over this...
CLIMB_SETTLE_S = 30.0    # ...it settles (from the start)...
CLIMB_AVERAGE_S = 30.0   # ...then its climb is averaged over half a minute (where the hold rings, several cycles)...
CLIMB_HALVES_MS = 0.5    # ...counted where it held its speed (CLIMB_SPEED) and the average's halves agree this well
CLIMB_SPEED = (0.05, 0.5)  # (a steep climb costs the hold a little speed: a climb within 5 % and 0.5 m/s of it)
POWER_LIMIT = 0.995      # full power: a helicopter's engines this near their limit
GOVERNED = 0.01          # within the power: the rotor within this of its governed speed
HELD_VS_MS = 0.2         # a level point held its height within this...
HELD_SPEED = (0.03, 0.1)  # ...and its speed within 3 % and 0.1 m/s
BISECT = 8               # the top speed's bracket halved this often
TILT_MARGIN_DEG = 0.5    # a trim at its pitch limit (within this) could not hold its speed


class _Runs:
    """The runs of one rotorcraft: a trim at a speed and a weight, level or at full power."""

    def __init__(self, r):
        self.r, self.heli = r, r.kind == "helicopter"
        self.s = fly.Session(r)
        # the tilt the platform's velocity loop flies within (src/control/Laws.cpp: 30 degrees - a helicopter's 20 - and
        # 0.8 of the envelope's bank and pitch): a speed only a steeper tilt holds is one the platform does not fly, so
        # the tables end short of it (ADR-29 FA-3e: the multirotors' best range lies at the top of their speeds)
        env = r.spec.get("envelope", {})
        tilt = 20.0 if self.heli else 30.0
        for key in ("bank_max_deg", "pitch_max_deg"):
            if key in env:
                tilt = min(tilt, 0.8 * abs(float(env[key])))
        self.tilt = math.radians(tilt)
        self.gains = fly._gains(r)
        if self.heli:
            from .heli import rotor_data
            self.rpm = rotor_data(r.spec)["rpm"]
        self.count = 0

    def close(self):
        self.s.close()

    def run(self, h, speed, fraction, climb=False, col0=None, power0=None):
        """One run at `speed` (m/s true, north) at `h` (m), a helicopter's tank at `fraction`: a level trim, the height
        held, or (`climb`) full power, the height free - the power rising to it from `power0` (the level point's). The
        averages of its last seconds (and how far its climb's second half's is from its first's), or None if it
        diverged."""
        r, s, heli = self.r, self.s, self.heli
        self.count += 1
        s.altitude_m = h
        v = s.spawn(speed, fraction if heli else None)
        power = (lambda: v.get_property("fcs/%s/power-demand" % r.name)) if climb else None
        col = col0 if col0 is not None else (0.5 if heli else s._hover_throttle())
        hold = fly.Hold(r.kind, s.dt, col, speed, v.state.altitude_msl_m, level=heli and speed >= 60 * KT, gains=self.gains,
                        tilt_max=self.tilt, power=power, power_target=POWER_LIMIT if climb else None)
        settle, average = (CLIMB_SETTLE_S, CLIMB_AVERAGE_S) if climb else (SETTLE_S, AVERAGE_S)
        n_settle, n = int(round(settle / s.dt)), int(round((settle + average) / s.dt))
        acc = []
        for k in range(n):
            if k == n_settle and heli and not climb:
                v.set_property("propulsion/fuel_freeze", 0)  # (the engines report the fuel they burn)
            if climb:
                hold.power_target = min(POWER_LIMIT, power0 + (POWER_LIMIT - power0) * (k + 1) * s.dt / CLIMB_RAMP_S)
            st = v.state
            u, vv = s.body_velocity(v)
            c = hold(st, u, vv)
            v.command_actuator(aileron=c["ail"], elevator=c["ele"], rudder=c["rud"], throttle=c["col"])
            s.world.step()
            if v.state.diverged:
                return None
            if k < n_settle:
                continue
            st = v.state
            u, vv = s.body_velocity(v)
            row = [u, -st.velocity_ned_ms[2], c["col"], math.degrees(st.euler_rad[1]), v.get_property("propulsion/engine[0]/rotor-rpm")]
            if heli:
                row += [v.get_property("fcs/%s/power" % r.name), v.get_property("propulsion/engine[0]/fuel-flow-rate-pps") * LB]
            else:
                row += [v.get_property("fsim/battery/power-w"), float("nan")]
            acc.append(row)
        a = np.mean(acc, axis=0)
        half = len(acc) // 2
        out = dict(tas_ms=float(a[0]), vz_ms=float(a[1]), throttle=float(a[2]), pitch_deg=float(a[3]), rotor_rpm=float(a[4]),
                   vz_halves=float(np.mean(np.array(acc[half:])[:, 1]) - np.mean(np.array(acc[:half])[:, 1])))
        if heli:
            out.update(power=float(a[5]), shaft_power_w=float(a[5]) * r.spec["engine"]["power_shp"] * 745.7, fuel_kg_s=float(a[6]))
        else:
            out.update(power_w=float(a[5]))
        return out

    def within(self, res, speed):
        """Whether a level trim held its speed and height within the power and the pitch limit."""
        if res is None:
            return False
        held = abs(res["vz_ms"]) <= HELD_VS_MS and abs(res["tas_ms"] - speed) <= HELD_SPEED[0] * speed + HELD_SPEED[1]
        tilted = abs(res["pitch_deg"]) >= math.degrees(self.tilt) - TILT_MARGIN_DEG
        if self.heli:
            power = res["power"] < POWER_LIMIT and abs(res["rotor_rpm"] / self.rpm - 1.0) < GOVERNED
        else:
            power = res["throttle"] < 0.99
        return held and power and not tilted

    def climbed(self, res, speed):
        """Whether a full-power climb held its speed, its average stationary, the rotor governed."""
        return (res is not None and abs(res["tas_ms"] - speed) <= CLIMB_SPEED[0] * speed + CLIMB_SPEED[1]
                and abs(res["vz_halves"]) <= CLIMB_HALVES_MS and abs(res["rotor_rpm"] / self.rpm - 1.0) < GOVERNED)

    def bracket(self, h, fraction):
        """The least and the top level speed at a condition (NaN, NaN if none holds): the hover's 0 if it holds,
        else doubling from 1 m/s to the first that holds; then doubling on to the first that does not, and the
        bracket halved BISECT times."""
        holds = lambda v: self.within(self.run(h, v, fraction), v)  # noqa: E731
        lo = 0.0 if holds(0.0) else None
        if lo is None:
            v = 1.0
            while v <= 256.0 and not holds(v):
                v *= 2.0
            if v > 256.0:
                return float("nan"), float("nan")
            a, b = v / 2.0 if v > 1.0 else 0.0, v  # the least: between the last that failed and the first that held
            for _ in range(BISECT):
                m = 0.5 * (a + b)
                a, b = (a, m) if holds(m) else (m, b)
            lo = b
        top, v = lo, max(1.0, 2.0 * lo)
        while v <= 256.0 and holds(v):
            top, v = v, 2.0 * v
        a, b = top, v
        for _ in range(BISECT):
            m = 0.5 * (a + b)
            a, b = (m, b) if holds(m) else (a, m)
        return lo, a


def fly_tables(r, log=print):
    """The tables of the rotorcraft r (rotorcraft.Rotorcraft): see the module."""
    import fsim
    level = fsim._native.log_level()
    fsim.set_log_level("error")  # (a trim past the top may diverge: it failed, and says so)
    try:
        return _fly_tables(r, log)
    finally:
        fsim.set_log_level(level)


def _fly_tables(r, log):
    runs = _Runs(r)
    heli = runs.heli
    t0 = time.time()
    fractions = FUEL_FRACTIONS if heli else (None,)
    nh, nw, nk = len(ALTITUDES_M), len(fractions), len(SPEED_FRACTIONS)
    grid = lambda *shape: np.full(shape, np.nan)  # noqa: E731
    t2 = {k: grid(nh, nw) for k in ("min_tas_ms", "max_tas_ms", "reach_tas_ms", "stall_cas_ms", "best_endurance_tas_ms",
                                     "best_range_tas_ms", "max_climb_ms", "climb_tas_ms")}
    use = "fuel_kg_s" if heli else "power_w"  # (what it consumes: a helicopter's fuel, a multirotor's charge)
    t2.update({"best_endurance_" + use: grid(nh, nw), "best_range_" + use: grid(nh, nw)})
    t3 = {k: grid(nh, nw, nk) for k in ("tas_ms", "throttle", "pitch_deg", "rotor_rpm", use, "ps_full_ms", "ps_idle_ms")}
    if heli:
        t3["shaft_power_w"] = grid(nh, nw, nk)
    try:
        # the weights, and a multirotor's battery: from a vehicle at each
        weights, battery_j = [], None
        for f in fractions:
            v = runs.s.spawn(0.0, f if heli else None)
            runs.s.world.step()
            weights.append(v.get_property("inertia/weight-lbs") * LB)
            if not heli:
                battery_j = v.get_property("fsim/battery/capacity-j")
        for i, h in enumerate(ALTITUDES_M):
            for j, f in enumerate(fractions):
                c = (i, j)
                lo, top = runs.bracket(h, f)
                if not np.isfinite(top):
                    continue
                t2["min_tas_ms"][c], t2["max_tas_ms"][c], t2["reach_tas_ms"][c] = lo, top, top
                speeds, use_of, climbs = [], [], []
                for k, frac in enumerate(SPEED_FRACTIONS):
                    v = lo + frac * (TOP * top - lo)
                    p = runs.run(h, v, f)
                    if not runs.within(p, v):
                        continue  # (none where it did not hold its speed and height within the power)
                    for key in t3:
                        if key in p:
                            t3[key][c + (k,)] = p[key]
                    speeds.append(p["tas_ms"]), use_of.append(p[use])
                    if heli:  # full power at the point's speed, the height free: its climb
                        q = runs.run(h, v, f, climb=True, col0=p["throttle"], power0=p["power"])
                        if runs.climbed(q, v):
                            t3["ps_full_ms"][c + (k,)] = q["vz_ms"]
                            climbs.append((p["tas_ms"], q["vz_ms"]))
                speeds, use_of = np.array(speeds), np.array(use_of)
                if len(speeds) >= 3 and np.all(use_of > 0):
                    band = (lo, TOP * top)
                    ve, ue = _extremum(speeds, use_of, most=False)
                    t2["best_endurance_tas_ms"][c] = float(np.clip(ve, *band))
                    t2["best_endurance_" + use][c] = ue if band[0] <= ve <= band[1] else float(np.interp(np.clip(ve, *band), speeds, use_of))
                    vr, _ = _extremum(speeds, speeds / use_of, most=True)
                    t2["best_range_tas_ms"][c] = float(np.clip(vr, *band))
                    t2["best_range_" + use][c] = float(np.interp(np.clip(vr, *band), speeds, use_of))
                if len(climbs) >= 3:
                    vc, pc = _extremum(*np.array(climbs).T, most=True)
                    t2["max_climb_ms"][c], t2["climb_tas_ms"][c] = pc, vc
                log("  performance: %.0f m, %.0f kg: %.1f to %.1f m/s (%.0f s)" % (h, weights[j], lo, top, time.time() - t0))
    finally:
        runs.close()
    capacity = None
    if heli:
        from .heli import fuel_of
        capacity = fuel_of(r.spec)["capacity_lb"] * LB
    return {"altitude_m": list(ALTITUDES_M), "weight_kg": weights, "speed_fraction": list(SPEED_FRACTIONS),
            "fuel_capacity_kg": capacity, "battery_capacity_j": battery_j, **{k: v.tolist() for k, v in t2.items()},
            **{k: v.tolist() for k, v in t3.items()}, "runs": runs.count, "seconds": time.time() - t0}


BURN_S = 600.0  # the flown burn: ten minutes at the best-endurance speed, or half what it has left there


def flown_burn(r, t):
    """Level at the best-endurance speed, 100 m, as it spawns (a helicopter's tank full), the fuel flowing, for ten
    minutes or half what it has left there (a Crazyflie's battery lasts seven): what it used a second - fuel (kg/s)
    or charge (W) - against the tables' least consumption there."""
    heli = r.kind == "helicopter"
    runs = _Runs(r)
    j = len(t["weight_kg"]) - 1
    speed = t["best_endurance_tas_ms"][0][j]
    try:
        s = runs.s
        s.altitude_m = ALTITUDES_M[0]
        v = s.spawn(speed, 1.0 if heli else None)
        hold = fly.Hold(r.kind, s.dt, 0.5 if heli else s._hover_throttle(), speed, v.state.altitude_msl_m, level=heli and speed >= 60 * KT,
                        gains=runs.gains, tilt_max=runs.tilt)
        store = (lambda: v.get_property("propulsion/total-fuel-lbs") * LB) if heli else (lambda: v.get_property("fsim/battery/charge-j"))
        n_settle, k, n = int(round(SETTLE_S / s.dt)), 0, None
        while n is None or k < n:
            if k == n_settle:
                v.set_property("propulsion/fuel_freeze", 0)
                before = store()
                seconds = min(BURN_S, 0.5 * before / t["best_endurance_" + ("fuel_kg_s" if heli else "power_w")][0][j])
                n = n_settle + int(round(seconds / s.dt))
            st = v.state
            u, vv = s.body_velocity(v)
            c = hold(st, u, vv)
            v.command_actuator(aileron=c["ail"], elevator=c["ele"], rudder=c["rud"], throttle=c["col"])
            s.world.step()
            k += 1
        used = (before - store()) / ((n - n_settle) * s.dt)
    finally:
        runs.close()
    return speed, used, (n - n_settle) * s.dt


def checks(r, t, flown):
    """The tables against the flight tests (fly) and the design, and against themselves."""
    from ..pipeline import check, info
    heli = r.kind == "helicopter"
    use = "fuel_kg_s" if heli else "power_w"
    j = len(t["weight_kg"]) - 1  # (as it spawns: a helicopter's tank full)
    out = []
    if heli:
        # the hover and the trim table's speeds (the fly stage's trims, at 100 m as it spawns): the power
        hover = (flown.get("hover_trim") or {}).get("power_shp")
        mine = t["shaft_power_w"][0][j][0] / 745.7
        if hover:
            out.append(check("hover power", mine, 0.95 * hover, 1.05 * hover, "shp", note="the fly stage's %.0f shp" % hover))
        speeds, power = np.array(t["tas_ms"][0][j], dtype=float), np.array(t["shaft_power_w"][0][j], dtype=float) / 745.7
        ok = np.isfinite(speeds) & np.isfinite(power)
        for row in flown.get("trim_table") or []:
            kt, want = row["target"]["kt"], row["model"]["power_shp"]
            got = float(np.interp(kt * KT, speeds[ok], power[ok], left=np.nan, right=np.nan))
            if np.isfinite(got):
                out.append(check("power at %g kt" % kt, got, 0.95 * want, 1.05 * want, "shp", note="the fly stage's trim %.0f shp" % want))
        # the lighter needs less, at every altitude and speed
        breaches = 0
        for i in range(len(t["altitude_m"])):
            rows = np.array(t[use][i], dtype=float)
            for k in range(rows.shape[1]):
                col = rows[:, k]
                if np.all(np.isfinite(col)) and not all(col[a] <= 1.01 * col[a + 1] for a in range(len(col) - 1)):
                    breaches += 1
        out.append(check("the lighter burns less", breaches, 0, 0, "breaches", note="at every altitude and speed flown", fmt="%.0f"))
    else:
        b = r.spec["battery"]
        want = float(b["capacity_wh"]) * 3600.0 / (float(b["hover_endurance_min"]) * 60.0)
        mine = t["power_w"][0][j][0]
        out.append(check("hover power", mine, 0.95 * want, 1.05 * want, "W", note="the design's %.1f W (its flight time, a hover's)" % want))
    speed, used, seconds = flown_burn(r, t)
    table = t["best_endurance_" + use][0][j]
    unit, scale = ("kg/h", 3600.0) if heli else ("W", 1.0)
    out.append(check("a flown burn at the best-endurance speed", used * scale, 0.95 * table * scale, 1.05 * table * scale, unit,
                     note="%.0f s level at %.1f m/s, 100 m; the tables' %.4g" % (seconds, speed, table * scale)))
    out.append(info("best endurance and best range at 100 m, as it spawns", "%.1f / %.1f m/s" % (
        t["best_endurance_tas_ms"][0][j], t["best_range_tas_ms"][0][j]), note="%.4g and %.4g %s" % (
        t["best_endurance_" + use][0][j] * scale, t["best_range_" + use][0][j] * scale, unit)))
    cas_max = r.spec.get("envelope", {}).get("cas_max_kt")
    out.append(info("top level speed at 100 m, as it spawns", t["max_tas_ms"][0][j], "m/s", note=(
        "%.0f kt; the design's never-exceed %.0f kt" % (t["max_tas_ms"][0][j] / KT, cas_max)) if cas_max else "%.0f kt" % (t["max_tas_ms"][0][j] / KT)))
    if heli:
        out.append(info("best climb at 100 m, as it spawns", t["max_climb_ms"][0][j], "m/s", note="at %.1f m/s, full power" % t["climb_tas_ms"][0][j]))
    flown_n = int(np.sum(np.isfinite(np.array(t["max_tas_ms"], dtype=float))))
    out.append(info("conditions flown", "%d of %d" % (flown_n, len(t["altitude_m"]) * len(t["weight_kg"])), note="altitudes %s m; weights %s kg; %d runs, %.0f s" % (
        ", ".join("%.0f" % h for h in t["altitude_m"]), ", ".join("%.4g" % w for w in t["weight_kg"]), t["runs"], t["seconds"])))
    return out
