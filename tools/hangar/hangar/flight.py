"""Flight tests of a JSBSim aircraft in the platform's own JSBSim.

The same tests fly any JSBSim aircraft - the design, and a reference to
compare it with - through the fsim SDK, at the actuator level with small
autopilots written here (so they measure the aircraft, not the platform's
controllers): trimmed level flight across the speed range, the stall, the
climb, the dynamic modes, and a robustness sweep of random attitudes and
rates of the kind an RL agent will reach. Speeds are true airspeed in m/s
unless named *_kcas.
"""
import math

import numpy as np

KT = 0.514444
FT = 0.3048
LBF = 4.448222
G0 = 9.80665


class Flight:
    def __init__(self, aircraft_type, name="hangar-test", jsbsim_root=None, lat=37.6, lon=-122.4):
        import fsim  # the SDK, only needed here
        self.fsim = fsim
        opts = dict(publish=False, workers=1)
        if jsbsim_root:
            opts["jsbsim_root"] = jsbsim_root
        self.world = fsim.World(name, **opts)
        self.type = aircraft_type if ":" in aircraft_type else "jsbsim:" + aircraft_type
        self.lat, self.lon = lat, lon
        self.dt = self.world.step_seconds
        self._n = 0

    def close(self):
        self.world.close()

    def spawn(self, altitude_m=1500.0, speed_ms=55.0, heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
        self._n += 1
        return self.world.create_vehicle("t%d" % self._n, type=self.type, latitude_deg=self.lat, longitude_deg=self.lon,
                                         altitude_msl_m=altitude_m, heading_deg=heading_deg, airspeed_ms=speed_ms,
                                         pitch_deg=pitch_deg, roll_deg=roll_deg)

    @staticmethod
    def prop(v, name, default=float("nan")):
        try:
            return v.get_property(name)
        except Exception:
            return default

    def trim(self, v, flaps=0.0):
        """Level flight at the vehicle's initial condition: JSBSim's own trim
        first, and if that fails (its engine steady state marches in 0.5 s
        steps, too coarse for a small propeller) the aircraft is flown to trim.
        Returns what it found, and leaves the throttle commanded to hold it."""
        if flaps:
            self.settle_flaps(v, flaps)
        ok = True
        level = self.fsim._native.log_level()
        self.fsim.set_log_level("off")      # a failed trim is handled below, not an error
        try:
            v.set_property("simulation/do_simple_trim", 1)
        except Exception:
            ok = False
        finally:
            self.fsim.set_log_level(level)
        thr = self.prop(v, "fcs/throttle-cmd-norm")
        if not ok:
            return self.fly_trim(v, flaps)
        res = {"ok": ok and 0.0 <= thr <= 1.0 + 1e-6, "alpha_deg": self.prop(v, "aero/alpha-deg"),
               "throttle": thr, "elevator_deg": self.prop(v, "fcs/elevator-pos-deg"),
               "pitch_trim": self.prop(v, "fcs/pitch-trim-cmd-norm"), "pitch_deg": self.prop(v, "attitude/theta-deg"),
               "speed_ms": v.state.airspeed_true_ms, "kcas": self.prop(v, "velocities/vc-kts"), "method": "jsbsim"}
        v.command_actuator(throttle=min(max(thr, 0.0), 1.0), flaps=flaps)
        return res

    def fly_trim(self, v, flaps=0.0, seconds=45.0):
        """Trim by flying: hold the height with the pitch attitude and the
        speed with the throttle until both settle, then read the trim off.
        Slower than JSBSim's trim, but it works for any aircraft JSBSim flies."""
        s0 = v.state
        v_ref, h_ref = s0.airspeed_true_ms, s0.altitude_msl_m
        ap = Autopilot(self.dt, math.degrees(s0.euler_rad[1]), throttle=0.5, flaps=flaps)
        integ = [0.5]
        elev = []

        def control(t, s, veh):
            err = v_ref - s.airspeed_true_ms
            integ[0] = float(np.clip(integ[0] + 0.02 * err * self.dt, 0.0, 1.0))
            ap.throttle = float(np.clip(integ[0] + 0.08 * err, 0.0, 1.0))
            ap.command(veh, s, ap.altitude(s, h_ref))
            if t > seconds - 5.0:
                elev.append(ap.elevator)

        h = self.run(v, seconds, control)
        k = h["t"] > seconds - 5.0
        # the elevator it took goes to the pitch trim, where JSBSim's own trim
        # leaves it, so a test commanding the elevator starts from trim
        try:
            v.set_property("fcs/pitch-trim-cmd-norm", float(np.mean(elev)) if elev else 0.0)
        except Exception:
            pass
        steady = bool(np.all(np.abs(h["vs"][k]) < 0.3) and np.all(np.abs(h["tas"][k] - v_ref) < 0.5))
        thr = float(np.mean(h["thr"][k]))
        res = {"ok": steady and thr < 0.995, "alpha_deg": float(np.mean(h["alpha"][k])), "throttle": thr,
               "elevator_deg": float(np.mean(h["de"][k])), "pitch_trim": 0.0, "pitch_deg": float(np.mean(h["theta"][k])),
               "speed_ms": float(np.mean(h["tas"][k])), "kcas": float(np.mean(h["kcas"][k])), "method": "flown"}
        v.command_actuator(throttle=min(max(thr, 0.0), 1.0), flaps=flaps, elevator=0.0)
        return res

    def settle_flaps(self, v, flaps, seconds=8.0):
        v.command_actuator(flaps=flaps)
        self.world.step(int(seconds / self.dt))

    # -- a controlled run -----------------------------------------------------------------------
    def run(self, v, seconds, control=None, record=("t",)):
        """Step for `seconds`, calling control(t, state, v) each step (it
        commands the vehicle); returns the time history as arrays."""
        n = int(round(seconds / self.dt))
        hist = {k: np.empty(n + 1) for k in ("t", "alt", "tas", "kcas", "alpha", "beta", "theta", "phi", "psi", "p", "q", "r",
                                             "vs", "nz", "de", "da", "dr", "thr", "rpm", "cl")}
        W = self.prop(v, "inertia/weight-lbs") * LBF
        S = self.prop(v, "metrics/Sw-sqft") * FT * FT
        # a jet has no propeller: asked once, not every step
        has_prop = v.state.engine_count > 0 and np.isfinite(self.prop(v, "propulsion/engine/propeller-rpm"))
        for i in range(n + 1):
            s = v.state
            t = i * self.dt
            if control is not None and i < n:
                control(t, s, v)
            hist["t"][i] = t
            hist["alt"][i] = s.altitude_msl_m
            hist["tas"][i] = s.airspeed_true_ms
            hist["kcas"][i] = s.airspeed_calibrated_ms / KT
            hist["alpha"][i] = math.degrees(s.alpha_rad)
            hist["beta"][i] = math.degrees(s.beta_rad)
            hist["phi"][i] = math.degrees(s.euler_rad[0])
            hist["theta"][i] = math.degrees(s.euler_rad[1])
            hist["psi"][i] = math.degrees(s.euler_rad[2])
            hist["p"][i], hist["q"][i], hist["r"][i] = (math.degrees(x) for x in s.angular_rate_body_rad_s)
            hist["vs"][i] = -s.velocity_ned_ms[2]
            hist["nz"][i] = s.load_factor
            hist["de"][i] = math.degrees(s.elevator_rad)
            hist["da"][i] = math.degrees(s.aileron_rad)
            hist["dr"][i] = math.degrees(s.rudder_rad)
            hist["thr"][i] = s.throttle_position[0] if s.engine_count else 0.0
            hist["rpm"][i] = self.prop(v, "propulsion/engine/propeller-rpm", 0.0) if has_prop else 0.0
            qbar = 0.5 * 1.225 * (s.airspeed_calibrated_ms) ** 2
            hist["cl"][i] = s.load_factor * W / max(qbar * S, 1e-6)
            if s.diverged:
                return {k: a[: i + 1] for k, a in hist.items()}
            if i < n:
                self.world.step()
        return hist


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, lo=-1.0, hi=1.0):
        self.kp, self.ki, self.kd, self.lo, self.hi = kp, ki, kd, lo, hi
        self.i = 0.0
        self.prev = None

    def __call__(self, err, dt, rate=None):
        self.i = float(np.clip(self.i + err * dt, -1.0 / max(self.ki, 1e-9), 1.0 / max(self.ki, 1e-9))) if self.ki else 0.0
        d = rate if rate is not None else (0.0 if self.prev is None else (err - self.prev) / dt)
        self.prev = err
        return float(np.clip(self.kp * err + self.ki * self.i + self.kd * d, self.lo, self.hi))


def wing_leveller(dt):
    """Aileron on bank (with roll-rate damping) and rudder on sideslip."""
    bank = PID(0.04, 0.005, 0.0)
    slip = PID(0.08, 0.0, 0.0)

    def act(s, target_bank_deg=0.0):
        phi = math.degrees(s.euler_rad[0])
        p = math.degrees(s.angular_rate_body_rad_s[0])
        a = bank(target_bank_deg - phi, dt) - 0.015 * p
        # wind from the right (beta > 0): yaw right, which in JSBSim's convention
        # (positive rudder = trailing edge left = nose left) is negative rudder
        r = -slip(math.degrees(s.beta_rad), dt)
        return float(np.clip(a, -1, 1)), float(np.clip(r, -1, 1))
    return act


# -- the tests ----------------------------------------------------------------------------------
def trim_sweep(f, speeds, altitude_m=1500.0):
    rows = []
    for vtas in speeds:
        v = f.spawn(altitude_m, vtas)
        r = f.trim(v)
        rows.append(r)
        v.remove()
    return rows


class Autopilot:
    """Pitch attitude hold (elevator) with altitude or airspeed on top, wings
    level (aileron) and no sideslip (rudder), throttle as set - plain
    PIDs in normalised commands, enough to fly the tests steadily."""

    def __init__(self, dt, theta0_deg, throttle=0.0, flaps=0.0):
        self.dt = dt
        self.theta_cmd = theta0_deg
        self.throttle = throttle
        self.flaps = flaps
        self.level = wing_leveller(dt)
        self.i_theta = 0.0
        self.i_outer = 0.0

    def pitch(self, s, theta_cmd):
        theta = math.degrees(s.euler_rad[1])
        q = math.degrees(s.angular_rate_body_rad_s[1])
        err = theta_cmd - theta
        self.i_theta = float(np.clip(self.i_theta + err * self.dt * 0.02, -0.6, 0.6))
        return float(np.clip(-(0.06 * err + self.i_theta) + 0.025 * q, -1, 1))  # nose up = negative elevator

    def altitude(self, s, h_ref):
        vs = -s.velocity_ned_ms[2]
        err = h_ref - s.altitude_msl_m
        self.i_outer = float(np.clip(self.i_outer + err * self.dt * 0.01, -10, 10))
        return float(np.clip(0.15 * err - 1.0 * vs + self.i_outer, -15, 25))

    def airspeed(self, s, v_ref):
        err = s.airspeed_true_ms - v_ref        # too fast: nose up
        self.i_outer = float(np.clip(self.i_outer + err * self.dt * 0.15, -15, 25))
        return float(np.clip(1.2 * err + self.i_outer, -15, 25))

    def command(self, veh, s, theta_cmd):
        a, r = self.level(s)
        self.elevator = self.pitch(s, theta_cmd)
        veh.command_actuator(aileron=a, rudder=r, elevator=self.elevator, throttle=self.throttle, flaps=self.flaps)


def stall(f, altitude_m=1500.0, start_ms=None, flaps=0.0, max_s=90.0):
    """The 1-g stall: the autopilot holds the height at idle power, so the
    speed bleeds off at about 1 kt/s and the angle of attack rises. The stall
    is the break: the angle of attack stops rising and falls back, or the
    nose drops, or the sink rate passes 4 m/s, or the elevator runs out. The
    stall speed is the lowest calibrated speed up to the break; CL max the
    highest CL (from the load factor) up to just after it."""
    v = f.spawn(altitude_m, start_ms or 40.0)
    tr = f.trim(v, flaps=flaps)
    ap = Autopilot(f.dt, tr["pitch_deg"], throttle=0.0, flaps=flaps)
    ap.i_outer = tr["pitch_deg"]

    def control(t, s, veh):
        ap.command(veh, s, ap.altitude(s, altitude_m))

    h = f.run(v, max_s, control)
    v.remove()
    t, a, th, vs, de = h["t"], h["alpha"], h["theta"], h["vs"], h["de"]
    lo_de = np.min(de)
    brk = len(t) - 1
    peak_a = -1e9
    for i in range(int(3.0 / f.dt), len(t)):
        peak_a = max(peak_a, a[i])
        why = None
        if a[i] < peak_a - 2.0:
            why = "alpha break"
        elif vs[i] < -4.0:
            why = "sink rate"
        elif de[i] <= lo_de + 0.05 and i > 0 and de[i - 1] <= lo_de + 0.05 and np.min(de[: i + 1]) < -15:
            why = "elevator limit"
        if why:
            brk = i
            break
    i_min = int(np.argmin(h["kcas"][: brk + 1]))
    j = min(brk + int(1.0 / f.dt), len(t) - 1)
    return {"stall_kcas": float(h["kcas"][i_min]), "stall_tas_ms": float(h["tas"][i_min]),
            "cl_max": float(np.max(h["cl"][int(3.0 / f.dt): j + 1])), "alpha_at_stall": float(np.max(a[: j + 1])),
            "time_s": float(t[brk]), "break": why if brk < len(t) - 1 else "none", "trim": tr}, h


def climb(f, altitude_m, speed_ms, seconds=45.0):
    """Full throttle at a held true airspeed, wings level: the rate of
    climb, averaged once the climb has settled."""
    v = f.spawn(altitude_m, speed_ms)
    tr = f.trim(v)
    ap = Autopilot(f.dt, tr["pitch_deg"], throttle=1.0)
    ap.i_outer = tr["pitch_deg"]

    def control(t, s, veh):
        ap.command(veh, s, ap.airspeed(s, speed_ms))

    h = f.run(v, seconds, control)
    v.remove()
    k = h["t"] > 0.5 * seconds
    ok = bool(np.any(k)) and abs(np.mean(h["tas"][k]) - speed_ms) < 3.0
    return {"altitude_m": altitude_m, "speed_ms": speed_ms, "rate_ms": float(np.mean(h["vs"][k])) if ok else float("nan")}, h


def climb_performance(f, stall_tas_sl, altitudes=(0.0, 1500.0, 3000.0, 4500.0)):
    """Best rate of climb over speed at several altitudes, and the service
    ceiling (0.5 m/s, 100 ft/min). A ceiling above the altitudes flown is
    flown to: up to two more climbs just below the estimate, so the ceiling
    comes from climbs near it rather than a long extrapolation."""
    def best_at(alt):
        best = None
        for k in (1.25, 1.4, 1.6):
            sigma = (1 - 2.25577e-5 * min(alt, 11000.0)) ** 4.2559 * math.exp(-max(alt - 11000.0, 0.0) / 6341.6)
            vref = k * stall_tas_sl / math.sqrt(sigma)
            r, _ = climb(f, max(alt, 30.0), vref)
            if np.isfinite(r["rate_ms"]) and (best is None or r["rate_ms"] > best["rate_ms"]):
                best = r
        return best or {"altitude_m": alt, "speed_ms": float("nan"), "rate_ms": float("nan")}

    def ceiling_of(rows):
        good = [r for r in rows if np.isfinite(r["rate_ms"])]
        if len(good) < 2 or good[0]["rate_ms"] <= 0.5:
            return float("nan")
        top = sorted(good, key=lambda r: r["altitude_m"])[-3:]       # the climbs nearest the ceiling
        slope, icpt = np.polyfit([r["altitude_m"] for r in top], [r["rate_ms"] for r in top], 1)
        return float((0.508 - icpt) / slope) if slope < 0 else float("nan")
    rows = [best_at(alt) for alt in altitudes]
    ceiling = ceiling_of(rows)
    for _ in range(2):
        highest = max(r["altitude_m"] for r in rows)
        if not np.isfinite(ceiling) or ceiling < highest + 600.0 or highest >= 15000.0:
            break
        rows.append(best_at(min(max(highest + 1000.0, ceiling - 500.0), 15000.0)))
        ceiling = ceiling_of(rows)
    rows.sort(key=lambda r: r["altitude_m"])
    return {"rows": rows, "service_ceiling_m": ceiling, "highest_climb_m": max(r["altitude_m"] for r in rows)}


def max_level_speed(rows):
    """From a trim sweep: the speed at which level flight needs full throttle."""
    ok = [r for r in rows if r["ok"]]
    if not ok:
        return float("nan")
    sp = np.array([r["speed_ms"] for r in ok])
    th = np.array([r["throttle"] for r in ok])
    fast = sp >= sp[np.argmin(th)]
    if np.max(th[fast]) < 0.999:
        # extrapolate the throttle-speed curve to full throttle
        coef = np.polyfit(sp[fast], th[fast], 2 if np.sum(fast) >= 3 else 1)
        coef[-1] -= 1.0
        roots = np.roots(coef)
        roots = roots[np.isreal(roots)].real
        roots = roots[roots > sp[fast].max()]
        return float(roots.min()) if len(roots) else float("nan")
    return float(np.interp(1.0, th[fast], sp[fast]))


def signal_3211(t, t0, unit, amp):
    """The 3-2-1-1 input of flight-test identification: pulses of 3, 2, 1, 1
    units, alternating sign - rich in frequencies around the modes."""
    edges = np.cumsum([0, 3, 2, 1, 1]) * unit + t0
    for k, sign in enumerate((1, -1, 1, -1)):
        if edges[k] <= t < edges[k + 1]:
            return sign * amp
    return 0.0


def identify_linear(h, states, inputs, t0, t1):
    """Equation-error least squares: xdot = A x + B u + c over [t0, t1],
    derivatives by central differences. Returns A, B and A's eigenvalues."""
    k = (h["t"] >= t0) & (h["t"] <= t1)
    X = np.column_stack([h[s] for s in states])[k]
    U = np.column_stack([h[s] for s in inputs])[k]
    t = h["t"][k]
    Xd = np.gradient(X, t, axis=0)
    reg = np.column_stack([X, U, np.ones(len(t))])
    theta, *_ = np.linalg.lstsq(reg, Xd, rcond=None)
    A = theta[: len(states)].T
    B = theta[len(states): len(states) + len(inputs)].T
    return A, B, np.linalg.eigvals(A)


def modes_from_eigs(eigs):
    out = []
    for e in eigs:
        if e.imag > 1e-6:
            wn = abs(e)
            out.append({"omega_n": float(wn), "zeta": float(-e.real / wn), "period_s": float(2 * math.pi / e.imag)})
        else:
            out.append({"real": float(e.real), "time_constant_s": float(-1.0 / e.real) if e.real != 0 else float("inf")})
    return out


def longitudinal_modes(eig):
    """Short period and phugoid from the four longitudinal eigenvalues: the
    fast pair and the slow pair (either may be two real roots)."""
    eig = sorted(eig, key=lambda e: -abs(e))
    fast, slow = eig[:2], eig[2:]

    def pair(p):
        wn = float(np.sqrt(abs(p[0] * p[1])))
        zeta = float(-(p[0] + p[1]).real / (2 * wn)) if wn > 0 else float("nan")
        im = max(abs(p[0].imag), 1e-12)
        return {"omega_n": wn, "zeta": zeta, "period_s": float(2 * math.pi / im) if im > 1e-9 else float("inf")}
    return pair(fast), pair(slow)


def modes(f, altitude_m=1500.0, speed_ms=55.0):
    """The dynamic modes at trimmed cruise, identified from JSBSim's own
    response to 3-2-1-1 inputs: longitudinal (u, alpha, q, theta from the
    elevator) and lateral (beta, p, r, phi from aileron and rudder)."""
    out = {}
    v = f.spawn(altitude_m, speed_ms)
    tr = f.trim(v)
    thr = min(max(tr["throttle"], 0.0), 1.0)

    level = wing_leveller(f.dt)

    def lon(t, s, veh):
        de = signal_3211(t, 2.0, 0.35, 0.08) + signal_3211(t, 20.0, 3.0, 0.03)
        a, r = level(s)
        veh.command_actuator(elevator=de, aileron=a, rudder=r, throttle=thr)
    h = f.run(v, 90.0, lon)
    v.remove()
    h["u"], h["a_rad"], h["q_rad"], h["th_rad"] = h["tas"], np.radians(h["alpha"]), np.radians(h["q"]), np.radians(h["theta"])
    A, B, eig = identify_linear(h, ("u", "a_rad", "q_rad", "th_rad"), ("de",), 1.5, 88.0)
    out["short_period"], out["phugoid"] = longitudinal_modes(list(eig))
    out["long_eigs"] = [complex(e) for e in eig]
    out["hist_long"] = h
    v = f.spawn(altitude_m, speed_ms)
    tr = f.trim(v)
    thr = min(max(tr["throttle"], 0.0), 1.0)
    ap = Autopilot(f.dt, tr["pitch_deg"], throttle=thr)

    def lat(t, s, veh):
        da = signal_3211(t, 2.0, 0.6, 0.15)
        dr = signal_3211(t, 10.0, 0.8, 0.2)
        veh.command_actuator(aileron=da, rudder=dr, elevator=ap.pitch(s, tr["pitch_deg"]), throttle=thr)
    h = f.run(v, 40.0, lat)
    v.remove()
    h["b_rad"], h["p_rad"], h["r_rad"], h["ph_rad"] = np.radians(h["beta"]), np.radians(h["p"]), np.radians(h["r"]), np.radians(h["phi"])
    A, B, eig = identify_linear(h, ("b_rad", "p_rad", "r_rad", "ph_rad"), ("da", "dr"), 1.5, 38.0)
    cplx = [e for e in eig if e.imag > 1e-6]
    real = sorted([e.real for e in eig if abs(e.imag) <= 1e-6])
    out["dutch_roll"] = modes_from_eigs([cplx[0]])[0] if cplx else {"omega_n": float("nan"), "zeta": float("nan"), "period_s": float("nan")}
    out["roll"] = {"time_constant_s": float(-1.0 / real[0]) if real and real[0] < 0 else float("nan")}
    spiral = real[-1] if len(real) > 1 else float("nan")
    out["spiral"] = {"eigenvalue": float(spiral),
                     "time_to_double_s": float(math.log(2) / spiral) if np.isfinite(spiral) and spiral > 0 else float("inf"),
                     "time_to_half_s": float(math.log(2) / -spiral) if np.isfinite(spiral) and spiral < 0 else float("inf")}
    out["lat_eigs"] = [complex(e) for e in eig]
    out["hist_lat"] = h
    return out


# name: height m, speed (times the stall speed), pitch deg, roll deg
CRASHES = {
    "flat drop": (4.0, 1.2, 0.0, 0.0),
    "nose dive": (6.0, 1.5, -60.0, 0.0),
    "wing tip": (8.0, 1.5, -15.0, 70.0),
    "tail strike": (4.0, 0.8, 35.0, 0.0),
    "inverted": (6.0, 1.5, -10.0, 180.0),
    "cartwheel": (8.0, 1.6, -30.0, 100.0),
}


def contact_points(path):
    """The contact points of a JSBSim aircraft file (structural frame, metres)."""
    import xml.etree.ElementTree as ET
    scale = {"M": 1.0, "IN": 0.0254, "FT": 0.3048}
    pts = []
    for c in ET.parse(path).getroot().find("ground_reactions").findall("contact"):
        loc = c.find("location")
        pts.append([float(loc.find(k).text) * scale[loc.get("unit", "IN").upper()] for k in "xyz"])
    return np.array(pts)


def lowest_contact(f, v, points):
    """Height of the lowest contact point above the ground (negative: below),
    from the points, the centre of gravity and the attitude."""
    cg = np.array([f.prop(v, "inertia/cg-%s-in" % a) for a in "xyz"]) * 0.0254
    r = points - cg
    rb = np.stack([-r[:, 0], r[:, 1], -r[:, 2]], axis=1)  # structural (x aft, z up) -> body (x forward, z down)
    s = v.state
    phi, theta = s.euler_rad[0], s.euler_rad[1]
    down = (-math.sin(theta) * rb[:, 0] + math.sin(phi) * math.cos(theta) * rb[:, 1]
            + math.cos(phi) * math.cos(theta) * rb[:, 2])
    return float(np.min(s.altitude_agl_m - down))


def crash_tests(f, stall_tas, points, seconds=8.0):
    """Into flat ground in six attitudes, at idle and hands off: does the
    aircraft stop, or bounce, sink through or blow up? A model fit for
    training survives the crashes an agent will make - the episode ends,
    but the numbers must stay sane, and the vehicle must reset cleanly."""
    out = {}
    for name, (height, k, pitch, roll) in CRASHES.items():
        v = f.spawn(height, k * stall_tas, heading_deg=90.0, pitch_deg=pitch, roll_deg=roll)
        touch, impact, fastest, rate, bounce, deepest, blown = None, float("nan"), 0.0, 0.0, 0.0, 0.0, False
        for i in range(int(seconds / f.dt)):
            v.command_actuator(throttle=0.0, aileron=0.0, elevator=0.0, rudder=0.0)
            f.world.step()
            s = v.state
            speed = float(np.linalg.norm(s.velocity_ned_ms))
            if s.diverged or not np.isfinite(speed):
                blown = True
                break
            low = lowest_contact(f, v, points)
            deepest = max(deepest, -low)
            if touch is None and low < 0.0:
                touch, impact = s.altitude_agl_m, speed
            if touch is not None:
                bounce = max(bounce, s.altitude_agl_m - touch)
                fastest = max(fastest, speed)
                rate = max(rate, float(np.max(np.abs(s.angular_rate_body_rad_s))))
        s = v.state
        out[name] = {"blew_up": blown, "impact_ms": impact, "fastest_after_ms": fastest, "max_rate_deg_s": math.degrees(rate),
                     "bounce_m": bounce, "deepest_m": deepest,
                     "final_speed_ms": float(np.linalg.norm(s.velocity_ned_ms)) if not blown else float("nan")}
        v.remove()
    return out


def robustness(f, n=60, seconds=6.0, seed=1):
    """Random attitudes, rates, speeds and control inputs - the states a
    learning agent reaches. Counts the runs that diverge (a NaN or an
    impossible state): a model fit for training has none."""
    rng = np.random.default_rng(seed)
    diverged = []
    for i in range(n):
        speed = float(rng.uniform(15.0, 90.0))
        v = f.spawn(float(rng.uniform(800.0, 3000.0)), speed, heading_deg=float(rng.uniform(0, 360)),
                    pitch_deg=float(rng.uniform(-80, 80)), roll_deg=float(rng.uniform(-180, 180)))
        cmd = rng.uniform(-1, 1, 3)

        def control(t, s, veh, cmd=cmd):
            veh.command_actuator(aileron=float(cmd[0]), elevator=float(cmd[1]), rudder=float(cmd[2]), throttle=1.0)
        h = f.run(v, seconds, control)
        if len(h["t"]) < int(seconds / f.dt) or v.state.diverged:
            diverged.append(i)
        v.remove()
    return {"runs": n, "diverged": len(diverged), "which": diverged}


# -- fighters: flown through their fly-by-wire ----------------------------------------------------
class FighterPilot:
    """Flies a fly-by-wire aircraft (hangar/fcs.py) through its commands:
    the stick asks for load factor beyond the gravity reference, the ailerons
    for roll rate. Holds a height with the load factor and a bank with the
    roll rate; the throttle is set."""

    def __init__(self, dt, n_max=9.0, n_min=-3.0):
        self.dt = dt
        self.pull, self.push = n_max - 1.0, 1.0 - n_min
        self.i_h = 0.0

    def stick(self, dn):
        return float(np.clip(-dn / self.pull if dn > 0 else -dn / self.push, -1.0, 1.0))

    def height(self, s, h_ref, dn_extra=0.0):
        """The load factor (beyond the reference) that holds a height."""
        vs = -s.velocity_ned_ms[2]
        err = h_ref - s.altitude_msl_m
        self.i_h = float(np.clip(self.i_h + 0.0004 * err * self.dt, -0.5, 0.5))
        return 0.004 * err - 0.04 * vs + self.i_h + dn_extra

    @staticmethod
    def bank(s, phi_ref_deg):
        err = (phi_ref_deg - math.degrees(s.euler_rad[0]) + 180.0) % 360.0 - 180.0
        return float(np.clip(err / 45.0, -1.0, 1.0))

    def fly(self, veh, s, h_ref, throttle, phi_ref_deg=0.0, dn_extra=0.0):
        veh.command_actuator(elevator=self.stick(self.height(s, h_ref, dn_extra)), aileron=self.bank(s, phi_ref_deg),
                             rudder=0.0, throttle=throttle)


def _smooth_rate(t, x, window=2.0):
    """dx/dt by a least-squares line over a sliding window."""
    out = np.full(len(t), np.nan)
    half = window / 2.0
    for i in range(len(t)):
        k = (t >= t[i] - half) & (t <= t[i] + half)
        if np.sum(k) > 4:
            out[i] = np.polyfit(t[k], x[k], 1)[0]
    return out


def level_acceleration(f, pilot, altitude_m, speed_ms, throttle, seconds, stop_below=0.03):
    """Level flight at a throttle from a speed: the speed over time, and the
    specific excess power P_s = V dV/dt / g at each speed on the way (s
    wind-free, height held: the climb rate the aircraft could trade for).
    Stops early once the speed has settled."""
    v = f.spawn(altitude_m, speed_ms)
    rec = {"t": [], "tas": [], "mach": [], "alt": []}
    steady = [0]

    def control(t, s, veh):
        pilot.fly(veh, s, altitude_m, throttle)
        rec["t"].append(t)
        rec["tas"].append(s.airspeed_true_ms)
        rec["mach"].append(s.mach)
        rec["alt"].append(s.altitude_msl_m)

    n = int(round(seconds / f.dt))
    chunk = int(round(5.0 / f.dt))
    done = 0
    while done < n:
        k = min(chunk, n - done)
        for _ in range(k):
            s = v.state
            control(done * f.dt, s, v)
            if s.diverged:
                break
            f.world.step()
            done += 1
        if v.state.diverged or done < 2 * chunk:
            if v.state.diverged:
                break
            continue
        t = np.array(rec["t"][-chunk:])
        vv = np.array(rec["tas"][-chunk:])
        if abs(np.polyfit(t, vv, 1)[0]) < stop_below:
            break
    v.remove()
    t, tas = np.array(rec["t"]), np.array(rec["tas"])
    dv = _smooth_rate(t[::6], tas[::6])
    ps = tas[::6] * dv / G0
    return {"t": t[::6], "tas": tas[::6], "mach": np.array(rec["mach"])[::6], "alt": np.array(rec["alt"])[::6],
            "ps": ps, "settled": bool(len(t) and t[-1] < seconds - 1.0)}


def max_level_mach(f, pilot, altitude_m, throttle=1.0, start_mach=0.85, seconds=300.0):
    """The Mach number where thrust meets drag in level flight at a throttle:
    accelerate from start_mach until the speed settles, or extrapolate the
    excess power of the last part of the run to zero."""
    a = _speed_of_sound(altitude_m)
    r = level_acceleration(f, pilot, altitude_m, start_mach * a, throttle, seconds)
    m, ps = r["mach"], r["ps"]
    ok = np.isfinite(ps)
    if r["settled"] or not np.any(ok):
        return float(m[-1]), r
    k = ok & (r["t"] > r["t"][-1] - 30.0)
    if np.sum(k) > 5:
        slope, icpt = np.polyfit(m[k], ps[k], 1)
        if slope < 0:
            return float(-icpt / slope), r
    return float(m[-1]), r


def _speed_of_sound(h):
    t = 288.15 - 0.0065 * min(h, 11000.0)
    return math.sqrt(1.4 * 287.053 * t)


def turn_performance(f, pilot, altitude_m, mach, throttle=1.0, loads=(3.0, 5.0, 7.0, 9.0)):
    """Sustained turn: level turns at load factors, the speed's rate of
    change at each; the load factor where the aircraft neither gains nor
    loses speed, and its turn rate g sqrt(n^2 - 1) / V."""
    V = mach * _speed_of_sound(altitude_m)
    rows = []
    for n in loads:
        phi = math.degrees(math.acos(1.0 / n))
        v = f.spawn(altitude_m, V)
        # roll in wings-level first, then pull: 4 s to settle, 4 s measured
        h = f.run(v, 9.0, lambda t, s, veh, n=n, phi=phi: pilot.fly(
            veh, s, altitude_m, throttle, phi_ref_deg=phi if t > 0.5 else 0.0,
            dn_extra=(n - math.cos(math.radians(phi))) * min(max((t - 1.0) / 1.5, 0.0), 1.0)))
        v.remove()
        k = h["t"] > 5.0
        if np.sum(k) < 10:
            continue
        dvdt = np.polyfit(h["t"][k], h["tas"][k], 1)[0]
        rows.append({"n_cmd": n, "n": float(np.mean(h["nz"][k])), "dvdt": float(dvdt), "alpha": float(np.mean(h["alpha"][k])),
                     "tas": float(np.mean(h["tas"][k]))})
    good = [r for r in rows if np.isfinite(r["dvdt"])]
    n_sus = float("nan")
    for a, b in zip(good[:-1], good[1:]):
        if a["dvdt"] >= 0.0 > b["dvdt"]:
            n_sus = a["n"] + (b["n"] - a["n"]) * a["dvdt"] / (a["dvdt"] - b["dvdt"])
    if good and good[-1]["dvdt"] >= 0.0:
        n_sus = good[-1]["n"]       # sustains the most it was asked for
    rate = math.degrees(G0 * math.sqrt(max(n_sus * n_sus - 1.0, 0.0)) / V) if np.isfinite(n_sus) else float("nan")
    return {"altitude_m": altitude_m, "mach": mach, "rows": rows, "n_sustained": n_sus, "rate_deg_s": rate}


def pull_and_roll(f, pilot, altitude_m, speed_ms, throttle=1.0):
    """Handling at a speed: full aft stick for 4 s (the load and angle of
    attack reached, the instantaneous turn rate); a 3 g command's response;
    full stick roll from wings level (roll rate, time to 90 deg of bank)."""
    out = {}
    v = f.spawn(altitude_m, speed_ms)
    h = f.run(v, 6.0, lambda t, s, veh: veh.command_actuator(elevator=-1.0 if 1.0 <= t < 5.0 else 0.0, aileron=0.0,
                                                               rudder=0.0, throttle=throttle))
    v.remove()
    k = (h["t"] >= 1.0) & (h["t"] < 5.0)
    i = int(np.argmax(np.where(k, h["nz"], -1e9)))
    out["pull"] = {"n_max": float(h["nz"][i]), "alpha_max": float(np.max(h["alpha"][k])),
                   "rate_deg_s": float(math.degrees(G0 * math.sqrt(max(h["nz"][i] ** 2 - 1.0, 0.0)) / h["tas"][i]))}
    v = f.spawn(altitude_m, speed_ms)
    dn3 = 3.0 - 1.0
    h = f.run(v, 7.0, lambda t, s, veh: veh.command_actuator(elevator=pilot.stick(dn3) if t >= 1.0 else 0.0, aileron=0.0,
                                                               rudder=0.0, throttle=throttle))
    v.remove()
    k = h["t"] >= 1.0
    steady = float(np.mean(h["nz"][h["t"] > 5.0]))
    peak = float(np.max(h["nz"][k]))
    rise = h["t"][k][np.argmax(h["nz"][k] >= 1.0 + 0.9 * (steady - 1.0))] - 1.0 if np.any(h["nz"][k] >= 1.0 + 0.9 * (steady - 1.0)) else float("nan")
    out["step"] = {"n_steady": steady, "overshoot": (peak - steady) / max(steady - 1.0, 1e-6), "rise_s": float(rise)}
    v = f.spawn(altitude_m, speed_ms)
    h = f.run(v, 4.0, lambda t, s, veh: veh.command_actuator(elevator=pilot.stick(pilot.height(s, altitude_m)),
                                                               aileron=1.0 if t >= 1.0 else 0.0, rudder=0.0, throttle=throttle))
    v.remove()
    k = h["t"] >= 1.0
    phi = np.unwrap(np.radians(h["phi"]))
    t90 = h["t"][k][np.argmax(np.degrees(phi[k] - phi[k][0]) >= 90.0)] - 1.0 if np.any(np.degrees(phi[k] - phi[k][0]) >= 90.0) else float("nan")
    out["roll"] = {"p_max": float(np.max(h["p"][k])), "time_to_90_s": float(t90), "beta_max": float(np.max(np.abs(h["beta"][k])))}
    return out


def fighter_tests(f, opts, quick=False):
    """A fighter's performance and handling, flown through its fly-by-wire
    (opts: the [flight_control] options): the fastest level speeds at sea
    level and at 36,000 ft, the specific excess power at sea level (its
    peak is the best rate of climb), the service ceiling (where the best
    excess power falls to 0.5 m/s), the sustained turn at 15,000 ft, and the
    handling at 5,000 ft: the load reached with the stick full aft, a 3 g
    step, a full-stick roll. The ceiling is the subsonic one, from the
    excess power at Mach 0.9, as service ceilings are published."""
    pilot = FighterPilot(f.dt, opts["n_max"], opts["n_min"])
    out = {}
    m_sl, run_sl = max_level_mach(f, pilot, 100.0, start_mach=0.4, seconds=150.0 if quick else 240.0)
    out["max_mach_sl"] = m_sl
    ok = np.isfinite(run_sl["ps"])
    i = int(np.argmax(np.where(ok, run_sl["ps"], -1e9)))
    out["ps_sl"] = {"mach": run_sl["mach"][ok], "ps": run_sl["ps"][ok]}
    out["climb_rate_ms"] = float(run_sl["ps"][i])
    out["climb_mach"] = float(run_sl["mach"][i])
    m_36, run_36 = max_level_mach(f, pilot, 10973.0, start_mach=0.9, seconds=200.0 if quick else 360.0)
    out["max_mach_36k"] = m_36
    out["ps_36k"] = {"mach": run_36["mach"][np.isfinite(run_36["ps"])], "ps": run_36["ps"][np.isfinite(run_36["ps"])]}
    # the service ceiling (subsonic, as published): the excess power at
    # Mach 0.9 at a few heights, where it falls to 0.5 m/s
    rows = []
    for h in ((12000.0, 14500.0) if quick else (12000.0, 13500.0, 15000.0, 16500.0)):
        r = level_acceleration(f, pilot, h, 0.88 * _speed_of_sound(h), 1.0, 25.0, stop_below=0.0)
        k = np.isfinite(r["ps"]) & (r["mach"] >= 0.86) & (r["mach"] <= 0.95)
        rows.append({"altitude_m": h, "ps_max": float(np.median(r["ps"][k])) if np.any(k) else float("nan")})
    good = [r for r in rows if np.isfinite(r["ps_max"])]
    ceiling = float("nan")
    if len(good) >= 2:
        slope, icpt = np.polyfit([r["altitude_m"] for r in good], [r["ps_max"] for r in good], 1)
        if slope < 0:
            ceiling = float((0.508 - icpt) / slope)
    out["ceiling_rows"] = rows
    out["service_ceiling_m"] = ceiling
    out["turn"] = turn_performance(f, pilot, 4572.0, 0.9, loads=(4.0, 6.0, 8.0) if quick else (3.0, 5.0, 6.0, 7.0, 8.0, 9.0))
    out["handling"] = pull_and_roll(f, pilot, 1524.0, 180.0)
    return out
