"""The aircraft's responses to its controls, identified for the platform.

Every vehicle on the platform can be commanded at five levels above its
surfaces (docs/sdk/control.md): attitude, acceleration, velocity, position
and behaviours, each flown by a built-in loop that commands the one below
it. The platform designs those loops for each aircraft from what was
measured on it - its plant (docs/control-architecture.md, section 7) - so
this stage measures it:

- identify: from level flight at a reference condition (3,000 m, the speed
  at which the wing carries the weight at CL 0.35, faster if that needs
  more than 90 % throttle, and no more than 80 % of the top speed), small
  steps at the actuator level - aileron 0.1 (the roll rate it settles to,
  and its lag), elevator 0.05 (the load factor, and its lag), rudder 0.1
  (the sideslip it holds), throttle 0.15 (the thrust, and the engine's
  lag) - plus level flight at three speeds for the elevator that trims it
  and the angle of attack it flies at. A fly-by-wire law is levelled at
  neutral stick, which holds its flight path; others by JSBSim's trim.
- fit: the trim law of a surface-controlled aircraft (the elevator of level
  flight, growing with lift as 1 / eas^2), the zero-lift angle of attack,
  the throttle of level flight.

The results go into autopilot.toml ([reference], [identified]) and from
there into the JSBSim aircraft's plant section. The platform places the
loops' poles on those first-order models, schedules the gains on the
airspeed and feeds forward the trim (src/control/Laws.cpp). evaluate() then
flies the aircraft as built - the platform designing its loops, as it will
for a trainer - through standard manoeuvres at three speeds.

Gains written by hand in autopilot.toml ([pid_attitude], ...) still go into
the aircraft as fsim/control/<controller>/<parameter>, and win.
"""
import math
import os
import time

import numpy as np

G0 = 9.80665
FT = 0.3048
KT = 0.514444
ALTITUDE_M = 3000.0
CL_REF = 0.35
MAX_BANK_DEG = {"fighter": 60.0, "attack": 50.0, "light_ga": 45.0, "uav": 40.0}   # 30 for the rest

ATTITUDE, ACCELERATION, VELOCITY, POSITION = "pid_attitude", "pid_acceleration", "pid_velocity", "pid_position"
LEVELS = {ATTITUDE: 1, ACCELERATION: 2, VELOCITY: 3, POSITION: 4}   # fsim.Level


# -- fits and pole placement ---------------------------------------------------------------------
def first_order(t, y, t_max):
    """Fit y = k (1 - exp(-(t - td) / tau)) to a step response over
    t <= t_max: (k, tau, td) by least squares, k exactly for each (tau, td)."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = t <= t_max
    t, y = t[m], y[m]
    best = (float("inf"), 0.0, 1.0, 0.0)
    for td in np.arange(0.0, 0.31, 0.02):
        for tau in np.geomspace(0.03, 6.0, 90):
            x = np.where(t > td, 1.0 - np.exp(-(t - td) / tau), 0.0)
            xx = float(x @ x)
            if xx <= 0.0:
                continue
            k = float(x @ y) / xx
            e = float(np.sum((y - k * x) ** 2))
            if e < best[0]:
                best = (e, k, tau, td)
    return best[1], best[2], best[3]


def _fbw(aircraft):
    return aircraft.spec.get("flight_control", {}).get("type", "direct") == "fbw"


def top_speed_ms(d):
    """The lowest top level speed the fly stage measured, m/s, or None."""
    fly = d.load("fly") or {}
    v = [c["value"] for c in fly.get("checks", []) if c["name"].startswith("maximum level speed") and c.get("value")]
    return min(v) * KT if v else None


# -- identification --------------------------------------------------------------------------------
class Identify:
    """Small steps from level flight, at the actuator level."""

    def __init__(self, d, log=print):
        from .flight import Flight
        self.d = d
        self.log = log
        self.fbw = _fbw(d.aircraft)
        self.f = Flight(d.aircraft.name, name="hangar-autopilot-" + d.aircraft.name)
        self.alt = ALTITUDE_M
        self.top = top_speed_ms(d)
        v = self.f.spawn(self.alt, 100.0)
        w = v.get_property("inertia/weight-lbs")
        s = v.get_property("metrics/Sw-sqft")
        rho = v.get_property("atmosphere/rho-slugs_ft3")
        v.remove()
        cap = 0.8 * self.top if self.top else float("inf")
        speed = min(FT * math.sqrt(2.0 * w / (rho * s * CL_REF)), cap)
        # faster where level flight would need more than 90 % throttle (a slow
        # delta's drag, or the afterburner)
        for _ in range(12):
            ok, thr = self.level(speed)
            if (ok and thr <= 0.9) or speed >= cap:
                break
            speed = min(1.12 * speed, cap)
        self.v_ref, self.ok, self.thr = speed, ok, thr

    def close(self):
        self.f.close()

    def _neutral(self, v, thr, seconds):
        w = self.f.spawn(self.alt, v)
        h = self.f.run(w, seconds, lambda t, s, veh: veh.command_actuator(throttle=thr))
        return w, h

    def level(self, v):
        """(ok, throttle) of level flight at v: JSBSim's trim, or for a
        fly-by-wire law - which holds its flight path at neutral stick - the
        throttle at which the speed holds, from three short flights."""
        if not self.fbw:
            w = self.f.spawn(self.alt, v)
            t = self.f.trim(w)
            w.remove()
            return bool(t["ok"]), float(t["throttle"])
        rates = []
        for thr in (0.3, 0.6, 0.9):
            w, h = self._neutral(v, thr, 5.0)
            w.remove()
            k = h["t"] > 2.0
            if k.sum() < 5 or not np.all(np.isfinite(h["tas"])):
                return False, 1.0
            rates.append(np.polyfit(h["t"][k], h["tas"][k], 1)[0])
        slope, icpt = np.polyfit((0.3, 0.6, 0.9), rates, 1)
        if slope <= 0.0:
            return False, 1.0
        thr = -icpt / slope
        return bool(0.0 <= thr <= 1.0), float(np.clip(thr, 0.0, 1.0))

    def trimmed(self, v=None):
        """A vehicle in level flight at v (the reference speed by default)."""
        v = v or self.v_ref
        if self.fbw:
            ok, thr = (self.ok, self.thr) if v == self.v_ref else self.level(v)
            w, _ = self._neutral(v, thr, 3.0)
            return w, {"ok": ok, "throttle": thr, "pitch_trim": 0.0}
        w = self.f.spawn(self.alt, v)
        t = self.f.trim(w)
        # the state still shows the spawn until the world steps: a moment at the trim first
        thr = min(max(t["throttle"], 0.0), 1.0)
        self.f.run(w, 0.2, lambda tt, s, veh: veh.command_actuator(throttle=thr))
        return w, t

    def run(self):
        f = self.f
        out = {"tas_ms": self.v_ref, "altitude_m": self.alt, "fbw": self.fbw}
        v, trim = self.trimmed()
        thr = min(max(trim["throttle"], 0.0), 1.0)
        out["eas_ms"] = v.state.airspeed_calibrated_ms
        out["throttle"] = thr
        v.remove()

        # level flight at three speeds: the elevator that trims it (a law that
        # trims itself has none to find) and the angle of attack it flies at
        trims, alphas = [], []
        for k in (0.75, 1.0, 1.35):
            w, t = self.trimmed(min(k * self.v_ref, 0.9 * self.top) if self.top else k * self.v_ref)
            if t["ok"]:
                if not self.fbw:
                    trims.append((w.state.airspeed_calibrated_ms, float(t["pitch_trim"])))
                alphas.append((w.state.airspeed_calibrated_ms, float(w.state.alpha_rad)))
            w.remove()
        out["trim_sweep"], out["alpha_sweep"] = trims, alphas

        # roll: aileron 0.1
        v, _ = self.trimmed()
        h = f.run(v, 3.0, lambda t, s, veh: veh.command_actuator(aileron=0.1, throttle=thr))
        k, tau, td = first_order(h["t"], np.radians(h["p"]), 3.0)
        out["roll"] = {"gain": k / 0.1, "lag_s": tau + td}
        v.remove()

        # pitch: elevator -0.05 (nose up), the load factor it pulls
        v, _ = self.trimmed()
        h = f.run(v, 3.0, lambda t, s, veh: veh.command_actuator(elevator=-0.05, throttle=thr))
        k, tau, td = first_order(h["t"], h["nz"] - h["nz"][0], 2.0)
        out["pitch"] = {"gain": k / 0.05, "lag_s": tau + td}
        v.remove()

        # yaw: rudder 0.1, the sideslip it holds
        v, _ = self.trimmed()
        h = f.run(v, 2.0, lambda t, s, veh: veh.command_actuator(rudder=0.1, throttle=thr))
        tail = h["t"] > 1.2
        out["yaw"] = {"gain": float(np.mean(np.radians(h["beta"][tail]) - math.radians(h["beta"][0]))) / 0.1}
        v.remove()

        # speed: throttle 0.15 up (down near full), the thrust and its lag
        v, _ = self.trimmed()
        step = 0.15 if thr < 0.8 else -0.15
        thrust = []

        def throttle(t, s, veh):
            veh.command_actuator(throttle=thr + step)
            thrust.append(sum(s.thrust_n[i] for i in range(min(s.engine_count, 4))))

        h = f.run(v, 6.0, throttle)
        th = np.array(thrust)
        k, tau, _ = first_order(h["t"][: len(th)], th - th[0], 6.0)
        mass = v.get_property("inertia/weight-lbs") * 0.45359237
        out["speed"] = {"gain": k / step / mass, "lag_s": tau}
        v.remove()
        return out


# -- fits ----------------------------------------------------------------------------------------------
def fits(ident):
    """What the level-flight sweeps give, for the plant: the throttle of level
    flight; a surface-controlled aircraft's trim law, stick = trim - lift +
    lift (eas_ref / eas)^2 at 1 g (a law that trims itself has none); the
    angle of attack of zero lift, level flight's alpha against 1 / eas^2."""
    eas = ident["eas_ms"]
    out = {"throttle_trim": float(ident["throttle"]), "elevator_trim": 0.0, "elevator_trim_lift": 0.0, "alpha_zero_lift_rad": 0.0}
    if not ident["fbw"] and len(ident["trim_sweep"]) >= 2:
        x = [(eas / e) ** 2 for e, _ in ident["trim_sweep"]]
        lift, zero = np.polyfit(x, [st for _, st in ident["trim_sweep"]], 1)
        out["elevator_trim"], out["elevator_trim_lift"] = float(zero + lift), float(lift)
    if len(ident["alpha_sweep"]) >= 2:
        _, alpha0 = np.polyfit([(eas / e) ** 2 for e, _ in ident["alpha_sweep"]], [a for _, a in ident["alpha_sweep"]], 1)
        out["alpha_zero_lift_rad"] = float(np.clip(alpha0, -0.2, 0.2))
    return out


# -- the settings file and the JSBSim properties -------------------------------------------------
def write_toml(d, ident, date=None):
    """autopilot.toml beside the design: the reference condition and what
    was identified there - reviewable, and read into the aircraft's plant
    section by the build."""
    lines = ["# Written by hangar autopilot (%s) for %s: the aircraft's responses to its" % (
             date or time.strftime("%Y-%m-%d %H:%M"), os.path.basename(d.path)),
             "# controls, identified at a reference condition (docs/hangar.md, The autopilot). The",
             "# build writes them into the JSBSim aircraft's plant section; the platform designs its",
             "# control loops from them. Delete this file to fly the platform's shared defaults.",
             "[reference]   # where they were measured",
             "tas_ms = %s" % _num(ident["tas_ms"]), "eas_ms = %s" % _num(ident["eas_ms"]), "altitude_m = %.0f" % ident["altitude_m"]]
    lines += identified_toml(ident)
    path = os.path.join(d.dir, "autopilot.toml")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return path


def identified_toml(ident):
    """The responses measured at the reference, as autopilot.toml's
    [identified] table: the build writes them into the aircraft's profile
    (the plant section; hangar/profile.py)."""
    lines = ["", "[identified]   # the responses at the reference: per unit of aileron (roll rate, rad/s), elevator (load factor,",
             "               # g), rudder (sideslip, rad) and throttle (acceleration, m/s2), each with its lag (s); level",
             "               # flight's throttle, a surface-controlled aircraft's trim law, the zero-lift angle of attack"]
    for axis in ("roll", "pitch", "yaw", "speed"):
        r = ident.get(axis) or {}
        fields = ", ".join("%s = %s" % (k, _num(r[k])) for k in ("gain", "lag_s") if k in r)
        if fields:
            lines.append("%s = { %s }" % (axis, fields))
    for key in ("throttle_trim", "elevator_trim", "elevator_trim_lift", "alpha_zero_lift_rad"):
        if key in ident:
            lines.append("%s = %s" % (key, _num(ident[key])))
    return lines


def load_identification(path):
    """autopilot.toml's [reference] and [identified] tables ({} and {} without them)."""
    import tomllib
    if not os.path.isfile(path):
        return {}, {}
    with open(path, "rb") as f:
        data = tomllib.load(f)
    return data.get("reference", {}), data.get("identified", {})


def _num(x):
    x = float(x)
    return "0.0" if x == 0.0 else ("%.6g" % x if "e" not in "%.6g" % x else "%.6e" % x)


def load_settings(path):
    """autopilot.toml -> {controller: {parameter: value}}, the parameters
    with their dots ("pitch.kp"); {} without the file."""
    import tomllib
    if not os.path.isfile(path):
        return {}
    with open(path, "rb") as f:
        data = tomllib.load(f)
    out = {}
    for controller, params in data.items():
        if controller in ("reference", "identified") or not isinstance(params, dict):
            continue
        flat = {}

        def walk(node, prefix):
            for k, x in node.items():
                name = prefix + k
                if isinstance(x, dict):
                    walk(x, name + ".")
                else:
                    flat[name] = float(x)

        walk(params, "")
        out[controller] = flat
    return out


def properties_xml(settings, indent="      "):
    """The settings as JSBSim property declarations in the flight control
    section: fsim/control/<controller>/<parameter, its dots as slashes>."""
    if not settings:
        return ""
    lines = [indent + "<!-- the platform's built-in control loops, tuned for this aircraft (hangar autopilot):",
             indent + "     every vehicle of the type takes these as its controllers' parameters -->"]
    for controller in sorted(settings):
        for k in sorted(settings[controller]):
            lines.append(indent + "<property value=\"%s\">fsim/control/%s/%s</property>" % (
                _num(settings[controller][k]), controller, k.replace(".", "/")))
    return "\n".join(lines) + "\n"


# -- evaluation -------------------------------------------------------------------------------------
class Evaluate:
    """The aircraft as built - the platform designing its loops from the plant
    in the JSBSim file - flown through standard manoeuvres, each from a fresh
    start and 40 s of velocity hold."""

    def __init__(self, d, ident):
        from .flight import Flight
        self.f = Flight(d.aircraft.name, name="hangar-autopilot-eval-" + d.aircraft.name)
        self.dt = self.f.dt
        self.alt = ident["altitude_m"]
        self.max_bank = design_max_bank(d.aircraft)

    def close(self):
        self.f.close()

    def fly(self, v, seconds, sample):
        rows = []
        for _ in range(int(round(seconds / self.dt))):
            self.f.world.step()
            s = v.state
            if s.diverged or s.altitude_msl_m < 200.0:
                return rows, True
            rows.append(sample(s))
        return rows, False

    def settled(self, speed):
        v = self.f.spawn(self.alt, speed)
        v.command_velocity(airspeed_ms=speed, vertical_speed_ms=0.0, heading_rad=0.0)
        rows, lost = self.fly(v, 40.0, lambda s: (s.airspeed_true_ms, -s.velocity_ned_ms[2], s.euler_rad[0], s.altitude_msl_m))
        if lost or not rows:
            return v, {"lost": True}
        a = np.array(rows[len(rows) * 3 // 4:])
        return v, {"lost": False, "speed_error_ms": float(a[-1, 0] - speed), "vz_std_ms": float(a[:, 1].std()),
                   "bank_std_deg": float(math.degrees(a[:, 2].std())), "height_change_m": float(a[-1, 3] - self.alt)}

    def manoeuvres(self, speed):
        out = {"speed_ms": speed}
        v, out["settle"] = self.settled(speed)
        v.remove()
        if out["settle"]["lost"]:
            return out
        tests = (
            ("bank", 12.0, lambda w, s0: w.command_attitude(roll_rad=math.radians(30), pitch_rad=s0.euler_rad[1], airspeed_ms=speed),
             lambda s, s0: math.degrees(s.euler_rad[0]), 30.0),
            ("pitch", 12.0, lambda w, s0: w.command_attitude(roll_rad=0.0, pitch_rad=s0.euler_rad[1] + math.radians(5), airspeed_ms=speed),
             lambda s, s0: math.degrees(s.euler_rad[1] - s0.euler_rad[1]), 5.0),
            ("climb", 20.0, lambda w, s0: w.command_velocity(airspeed_ms=speed, vertical_speed_ms=0.05 * speed, heading_rad=0.0),
             lambda s, s0: -s.velocity_ned_ms[2], 0.05 * speed),
            ("load_factor", 4.0, lambda w, s0: w.command_acceleration(load_factor_g=1.5, roll_rate_rad_s=0.0),
             lambda s, s0: s.load_factor - 1.0, 0.5),
            ("roll_rate", 3.0, lambda w, s0: w.command_acceleration(load_factor_g=1.0, roll_rate_rad_s=0.25),
             lambda s, s0: s.angular_rate_body_rad_s[0], 0.25))
        for name, seconds, command, sample, target in tests:
            w, _ = self.settled(speed)
            s0 = type(w.state).from_buffer_copy(w.state)
            command(w, s0)
            rows, lost = self.fly(w, seconds, lambda s: sample(s, s0))
            w.remove()
            out[name] = {"lost": True} if lost else dict(step_metrics(rows, target, self.dt), target=target,
                                                         trace=trace(rows, self.dt))
        # a 90 deg turn, for as long as it takes at the bank limit and 30 s more
        w, _ = self.settled(speed)
        psi0 = w.state.euler_rad[2]
        w.command_velocity(airspeed_ms=speed, vertical_speed_ms=0.0, heading_rad=psi0 + math.radians(90))
        turn = math.radians(90) / (G0 * math.tan(self.max_bank) / speed)
        rows, lost = self.fly(w, turn + 30.0, lambda s: (s.euler_rad[2], s.euler_rad[0]))
        w.remove()
        if lost:
            out["heading"] = {"lost": True}
        else:
            a = np.array(rows)
            psi = np.degrees(np.unwrap(a[:, 0]) - psi0)
            m = step_metrics(list(psi), 90.0, self.dt)
            m["target"], m["trace"] = 90.0, trace(list(psi), self.dt)
            late = np.nonzero(np.abs(psi - 90.0) > 3.0)[0]
            m["settle_s"] = float((late[-1] + 1) * self.dt) if len(late) else 0.0
            m["max_bank_deg"] = float(np.degrees(np.abs(a[:, 1]).max()))
            m["turn_s"] = turn
            out["heading"] = m
        return out


def design_max_bank(aircraft):
    return math.radians(MAX_BANK_DEG.get(aircraft.spec.get("aircraft", {}).get("category", ""), 30.0))


def trace(y, dt, every=6):
    """A response thinned for the report: {"t": [...], "y": [...]}."""
    return {"t": [round((i + 1) * dt, 3) for i in range(0, len(y), every)], "y": [float(y[i]) for i in range(0, len(y), every)]}


def step_metrics(y, target, dt):
    """A step from 0 to target: the time to 90 % (inf if never), the
    overshoot (fraction of the step), and how often it crosses the target
    after first reaching it."""
    y = np.asarray(y, float)
    if len(y) < 10:
        return {"lost": True}
    prog = y / target
    hit = np.nonzero(prog >= 0.9)[0]
    sign = np.sign(y - target)
    crossings = int(np.sum(np.abs(np.diff(sign[hit[0]:])) > 0)) if len(hit) else 0
    return {"lost": False, "rise_s": float((hit[0] + 1) * dt) if len(hit) else float("inf"),
            "overshoot": max(0.0, float(prog.max() - 1.0)), "crossings": crossings,
            "final_error": float(np.mean(y[int(len(y) * 0.8):]) / target - 1.0)}


def speeds(d, ident):
    top = top_speed_ms(d)
    return [min(k * ident["tas_ms"], 0.9 * top) if top else k * ident["tas_ms"] for k in (0.7, 1.0, 1.5)]


# -- the stage ------------------------------------------------------------------------------------------
def stage(d):
    """Identify, write autopilot.toml, rebuild the JSBSim aircraft with its
    plant, and fly it as the platform designs its loops."""
    from .pipeline import check, info
    t0 = time.time()
    ident_run = Identify(d, d.log)
    try:
        ident = ident_run.run()
    finally:
        ident_run.close()
    ident.update(fits(ident))
    write_toml(d, ident)
    d.build()   # reads autopilot.toml into the JSBSim aircraft's plant section
    ev = Evaluate(d, ident)
    try:
        flown = [ev.manoeuvres(s) for s in speeds(d, ident)]
    finally:
        ev.close()
    checks = [info("reference: level flight at %.0f m" % ident["altitude_m"], ident["tas_ms"], "m/s",
                   note="%s; the platform designs the loops from what follows" % ("fly-by-wire" if ident["fbw"] else "surfaces")),
              info("roll rate per unit aileron (lag)", ident["roll"]["gain"], "rad/s", note="%.2f s" % ident["roll"]["lag_s"]),
              info("load factor per unit elevator (lag)", ident["pitch"]["gain"], "g", note="%.2f s" % ident["pitch"]["lag_s"])]
    checks += evaluation_checks(flown, check)
    from .report import plots
    images = []
    try:
        plots.autopilot(flown, d.img("autopilot.png"), d.aircraft.name)
        images.append("autopilot.png")
    except Exception as e:   # the plot is a convenience; the numbers are in autopilot.json
        d.log("  (no autopilot plot: %s)" % e)
    return d.save("autopilot", {"identification": ident, "flown": flown, "checks": checks,
                                "images": images, "seconds": time.time() - t0})


def evaluation_checks(flown, check):
    """The worst of each manoeuvre over the speeds flown."""
    def worst(key, field, f=max):
        vals = [r[key][field] for r in flown if key in r and not r[key].get("lost")]
        return f(vals) if vals else None

    def lost(rows):
        return [("%.0f m/s %s" % (r["speed_ms"], k)) for r in rows for k, m in r.items() if isinstance(m, dict) and m.get("lost")]

    ref, other = lost(flown[1:2]), lost(flown[:1] + flown[2:])
    checks = [check("manoeuvres that lost control at the reference speed", float(len(ref)), None, 0.0, note=", ".join(ref[:4])),
              check("manoeuvres that lost control, slower and faster", float(len(other)), None, 0.0, level="warn",
                    note=", ".join(other[:4]))]
    h = worst("settle", "height_change_m", lambda v: max(abs(x) for x in v))
    if h is not None:
        checks.append(check("velocity hold, 40 s: height kept within", h, None, 50.0, "m", level="warn"))
    for key, label, hi in (("bank", "30 deg bank step", 0.3), ("pitch", "5 deg pitch step", 0.4), ("climb", "climb step", 0.4),
                           ("load_factor", "1.5 g step", 0.4), ("roll_rate", "roll-rate step", 0.4)):
        o = worst(key, "overshoot")
        if o is not None:
            checks.append(check("%s: overshoot" % label, 100.0 * o, None, 100.0 * hi, "%", level="warn",
                                note="rise %.1f s" % worst(key, "rise_s", min)))
    s = worst("heading", "settle_s")
    if s is not None:
        checks.append(check("90 deg turn: settled within 3 deg", s, None, max(r["heading"].get("turn_s", 0) for r in flown
                                                                              if "heading" in r) + 30.0, "s", level="warn"))
    return checks
