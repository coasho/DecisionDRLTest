"""A rotorcraft's flight tests, in the platform's own JSBSim through the fsim SDK, at the actuator
level with a hold written here (so they measure the aircraft, not the platform's controllers):
trim across its speed range against the design's targets, and the hover plant identified - each
axis's control power, damping and lag, the heave's, and the hover's trims - which becomes the
profile's hover section, the platform's rotorcraft loops are designed from.

Senses, the platform's: aileron + right, elevator + nose down, rudder + nose left, throttle the
collective (a helicopter) or each rotor's thrust (a multirotor, with roll, pitch and yaw mixed in).
"""
import math
import os

import numpy as np

KT = 0.514444
G0 = 9.80665
FDM_DT = 1.0 / 120.0  # the platform's JSBSim step (four to a world step)


class Hold:
    """Speed, side velocity, height and heading held through the attitudes, as a pilot flies a
    rotorcraft: the cyclic (or the mixer) tilts it, the collective (or the thrust) holds the height,
    the pedals (or the yaw mixer) the heading. `level` holds the wings level instead of the side
    velocity (TM-85890 trims so above 60 kt)."""

    def __init__(self, kind, dt, throttle, speed_ms, height_m, psi=0.0, level=False, gains=None):
        self.kind, self.dt, self.level = kind, dt, level
        self.u_ref, self.h_ref, self.psi_ref = speed_ms, height_m, psi
        self.i = {"ail": 0.0, "ele": 0.0, "rud": 0.0, "col": throttle, "u": 0.0, "v": 0.0}
        g = {"th": 2.0, "q": 1.2, "ph": 2.0, "p": 0.8, "r": 1.2, "psi": 1.0, "vz": 0.05, "vzi": 0.03, "att_i": 0.3}
        g.update(gains or {})
        self.g = g
        self.cmd = dict(ail=0.0, ele=0.0, rud=0.0, col=throttle)

    def __call__(self, s, u, v):
        g, i, dt = self.g, self.i, self.dt
        phi, theta, psi = s.euler_rad
        p, q, r = s.angular_rate_body_rad_s
        vz = -s.velocity_ned_ms[2]
        eu, ev = self.u_ref - u, -v
        i["u"] = float(np.clip(i["u"] + 0.0005 * eu * dt, -0.3, 0.3))
        i["v"] = float(np.clip(i["v"] + 0.0005 * ev * dt, -0.3, 0.3))
        th_ref = float(np.clip(-0.03 * eu - 10 * i["u"], -0.35, 0.35))
        ph_ref = 0.0 if self.level else float(np.clip(0.03 * ev + 10 * i["v"], -0.35, 0.35))
        e_th, e_ph = th_ref - theta, ph_ref - phi
        i["ele"] = float(np.clip(i["ele"] - g["att_i"] * e_th * dt, -1, 1))
        i["ail"] = float(np.clip(i["ail"] + g["att_i"] * e_ph * dt, -1, 1))
        e_psi = math.atan2(math.sin(self.psi_ref - psi), math.cos(self.psi_ref - psi))
        r_cmd = g["psi"] * e_psi
        i["rud"] = float(np.clip(i["rud"] - 0.5 * (r_cmd - r) * dt, -1, 1))
        vz_cmd = float(np.clip(0.4 * (self.h_ref - s.altitude_msl_m), -3, 3))
        i["col"] = float(np.clip(i["col"] + g["vzi"] * (vz_cmd - vz) * dt, 0, 1))
        self.cmd = dict(ele=float(np.clip(i["ele"] - g["th"] * e_th + g["q"] * q, -1, 1)),
                        ail=float(np.clip(i["ail"] + g["ph"] * e_ph - g["p"] * p, -1, 1)),
                        rud=float(np.clip(i["rud"] - g["r"] * (r_cmd - r), -1, 1)),
                        col=float(np.clip(i["col"] + g["vz"] * (vz_cmd - vz), 0, 1)))
        return self.cmd


def _gains(r):
    """The hold's gains in the platform's normalised commands: a helicopter's are the pilot's
    (they fly both helicopters); a multirotor's from its control power (the mixer's moment per
    unit command over the inertia) with its motors' lag setting the bandwidth."""
    if r.kind == "helicopter":
        return None
    from . import multi
    rows, kT, kQ, wmax = multi.rotors(r.spec)
    tmax = kT * wmax ** 2
    m = r.spec["mass"]
    Lr = sum(multi.MIX * tmax * abs(y) for _, _, y, _ in rows) / m["ixx"]
    Lq = sum(multi.MIX * tmax * abs(x) for _, x, _, _ in rows) / m["iyy"]
    Nr = sum(multi.MIX * tmax * kQ / kT for _ in rows) / m["izz"]
    wr = min(15.0, 1.0 / (3.0 * r.spec["rotors"]["lag_up_s"]))  # the rate loop's bandwidth
    wa = wr / 4.0
    return {"th": wa * wr / Lq, "q": wr / Lq, "ph": wa * wr / Lr, "p": wr / Lr, "r": 2.0 * wr / Nr, "psi": wa,
            "vz": 0.25 / (4 * tmax / (m["mass_kg"])) * 4, "vzi": 0.1, "att_i": 0.0}


class Session:
    def __init__(self, r, altitude_m=100.0):
        import fsim
        self.fsim, self.r = fsim, r
        self.world = fsim.World("hangar-rotorcraft", dt=FDM_DT, frame_skip=4, publish=False, workers=1)
        self.dt = self.world.step_seconds
        self.altitude_m = altitude_m
        self.n = 0
        self.v = None

    def close(self):
        self.world.close()

    def spawn(self, speed_ms):
        """A fresh vehicle for the next test; the last one leaves the world."""
        if self.v is not None:
            self.v.remove()
        self.n += 1
        self.v = self.world.create_vehicle("r%d" % self.n, type="jsbsim:" + self.r.name, latitude_deg=37.6, longitude_deg=-122.4,
                                           altitude_msl_m=self.altitude_m, heading_deg=0.0, airspeed_ms=speed_ms)
        self.v.set_protection("off")  # the aircraft, not the platform's envelope protection (flight.Flight)
        return self.v

    @staticmethod
    def body_velocity(v):
        return v.get_property("velocities/u-fps") * 0.3048, v.get_property("velocities/v-fps") * 0.3048

    def trim(self, speed_ms, seconds=60.0, level=False, throttle0=None):
        """Flies the hold to a steady trim; returns the averages of its last 5 s and the vehicle."""
        v = self.spawn(speed_ms)
        throttle0 = throttle0 if throttle0 is not None else (0.5 if self.r.kind == "helicopter" else self._hover_throttle())
        hold = Hold(self.r.kind, self.dt, throttle0, speed_ms, v.state.altitude_msl_m, level=level, gains=_gains(self.r))
        acc = []
        n = int(round(seconds / self.dt))
        for k in range(n):
            s = v.state
            u, vv = self.body_velocity(v)
            c = hold(s, u, vv)
            v.command_actuator(aileron=c["ail"], elevator=c["ele"], rudder=c["rud"], throttle=c["col"])
            self.world.step()
            if v.state.diverged:
                return None, v
            if k * self.dt > seconds - 5.0:
                s = v.state
                u, vv = self.body_velocity(v)
                acc.append([c["ail"], c["ele"], c["rud"], c["col"], math.degrees(s.euler_rad[0]), math.degrees(s.euler_rad[1]),
                            u, vv, -s.velocity_ned_ms[2], v.get_property("propulsion/engine[0]/rotor-rpm")]
                           + ([v.get_property("fcs/%s/power" % self.r.name)] if self.r.kind == "helicopter" else []))
        a = np.mean(acc, axis=0)
        out = dict(aileron=a[0], elevator=a[1], rudder=a[2], throttle=a[3], phi_deg=a[4], theta_deg=a[5], u_ms=a[6], v_ms=a[7],
                   vz_ms=a[8], rpm=a[9], steady=bool(abs(a[8]) < 0.2 and abs(a[6] - speed_ms) < 0.5))
        if self.r.kind == "helicopter":
            out["power_shp"] = a[10] * self.r.spec["engine"]["power_shp"]
        return out, v

    def _hover_throttle(self):
        from . import multi
        _, kT, _, wmax = multi.rotors(self.r.spec)
        return self.r.spec["mass"]["mass_kg"] * G0 / (4 * kT * wmax ** 2)

    def step_response(self, trim, axis, size, seconds):
        """From a fresh hover trim held 1 s, one command stepped by `size` (normalised); the body
        rate (or vertical acceleration) and the command, every step."""
        v = self.spawn(0.0)
        base = dict(aileron=trim["aileron"], elevator=trim["elevator"], rudder=trim["rudder"], throttle=trim["throttle"])
        # re-trim by flying the hold briefly (a fresh vehicle starts from a neutral control system)
        hold = Hold(self.r.kind, self.dt, trim["throttle"], 0.0, v.state.altitude_msl_m, gains=_gains(self.r))
        hold.i.update(ail=trim["aileron"], ele=trim["elevator"], rud=trim["rudder"], col=trim["throttle"])
        for _ in range(int(round(20.0 / self.dt))):
            s = v.state
            u, vv = self.body_velocity(v)
            c = hold(s, u, vv)
            v.command_actuator(aileron=c["ail"], elevator=c["ele"], rudder=c["rud"], throttle=c["col"])
            self.world.step()
        base = dict(aileron=hold.cmd["ail"], elevator=hold.cmd["ele"], rudder=hold.cmd["rud"], throttle=hold.cmd["col"])
        key = {"roll": "aileron", "pitch": "elevator", "yaw": "rudder", "heave": "throttle"}[axis]
        stepped = dict(base)
        stepped[key] = base[key] + size
        v.command_actuator(**stepped)
        t, y, x = [], [], []
        for k in range(int(round(seconds / self.dt))):
            self.world.step()
            s = v.state
            p, q, rr = s.angular_rate_body_rad_s
            t.append((k + 1) * self.dt)
            if axis == "heave":
                y.append(-v.get_property("accelerations/n-pilot-z-norm") * G0 - G0)  # vertical acceleration, + up
                x.append(-s.velocity_ned_ms[2])
            else:
                y.append({"roll": p, "pitch": q, "yaw": rr}[axis])
        return np.array(t), np.array(y), np.array(x) if x else None


def step_input(t, size, lag, delay):
    """The step as the airframe feels it: through the actuator's lag, a JSBSim step late (its
    forces move the aircraft on the next step: Propagate runs first)."""
    te = np.maximum(t - delay, 0.0)
    return size * (1.0 - np.exp(-te / lag)) if lag > 0 else np.where(t > delay, size, 0.0)


def identify(t, y, size, lag, dt, x=None, delay=FDM_DT):
    """ydot = -d y + P u_f, u_f the step through the actuator's lag: least squares for the control
    power P (per unit command, per s^2) and the damping d (1/s). For the heave the measured signal
    is the acceleration a = -d w + P u_f, w the vertical speed (x)."""
    u_f = step_input(t, size, lag, delay)
    if x is None:
        ydot = np.gradient(y, dt)
        A = np.column_stack([-y, u_f])
        b = ydot
    else:
        A = np.column_stack([-x, u_f])
        b = y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    d, P = float(sol[0]), float(sol[1])
    fit = A @ sol
    err = float(np.sqrt(np.mean((fit - b) ** 2)) / max(1e-9, np.max(np.abs(b))))
    return P, d, err


def run(r):
    s = Session(r)
    checks, results = [], {}
    try:
        kind = r.kind
        hover, v = s.trim(0.0)
        if hover is None:
            raise RuntimeError("diverged in the hover trim")
        results["hover_trim"] = hover
        checks.append(_c("hover trim steady", 1.0 if hover["steady"] else 0.0, "", "1", "pass" if hover["steady"] else "fail"))
        tg = r.spec.get("targets", {})
        if kind == "helicopter":
            t_in = r.spec["controls"]["travel_in"]
            coll_in = hover["throttle"] * t_in["collective"]
            checks.append(_c("hover collective", coll_in, "in", "%.3g (flight)" % tg["hover_collective_in"] if "hover_collective_in" in tg else "", "info"))
            checks.append(_c("hover power", hover["power_shp"], "shp", "", "info"))
            checks.append(_c("hover pitch attitude", hover["theta_deg"], "deg", "", "info"))
            # the trim table, against the design's (TM-85890 table 4)
            rows = []
            for row in tg.get("trim", []):
                kt = row["kt"]
                res, _ = s.trim(kt * KT, level=kt >= 60, throttle0=row["coll_in"] / t_in["collective"])
                if res is None:
                    checks.append(_c("trim at %g kt" % kt, 0.0, "", "steady", "fail"))
                    continue
                got = dict(kt=kt, long_in=res["elevator"] * t_in["longitudinal"] / 2, lat_in=res["aileron"] * t_in["lateral"] / 2,
                           coll_in=res["throttle"] * t_in["collective"], pedal_in=-res["rudder"] * t_in["pedal"] / 2,
                           theta_deg=res["theta_deg"], phi_deg=res["phi_deg"], power_shp=res["power_shp"])
                rows.append({"target": row, "model": got})
                for key, tol, unit in (("coll_in", 0.3, "in"), ("long_in", 0.5, "in"), ("pedal_in", 0.3, "in"), ("theta_deg", 1.5, "deg"),
                                       ("lat_in", 1.2, "in")):
                    err = got[key] - row[key]
                    checks.append(_c("trim %g kt: %s" % (kt, key), got[key], unit, "%.3g +- %g" % (row[key], tol),
                                     "pass" if abs(err) <= tol else "warn"))
            results["trim_table"] = rows
            if "trim_long_60kt_in" in tg:
                res, _ = s.trim(60 * KT)
                if res is not None:
                    long_in = res["elevator"] * t_in["longitudinal"] / 2
                    results["trim_60kt"] = res
                    checks.append(_c("60 kt longitudinal stick", long_in, "in", "%.3g (flight)" % tg["trim_long_60kt_in"],
                                     "pass" if abs(long_in - tg["trim_long_60kt_in"]) < 0.5 else "warn"))
        # the hover plant: each axis's step, fitted
        lags = _lags(r)
        plant, steps = {}, {}
        for axis, size in (("roll", 0.03), ("pitch", 0.03), ("yaw", 0.05), ("heave", 0.03)):
            t, y, x = s.step_response(hover, axis, size, 1.0 if kind == "helicopter" else 0.5)
            P, d, err = identify(t, y, size, lags[axis], s.dt, x)
            plant[axis] = dict(power=P, damping=d, lag_s=lags[axis], delay_s=FDM_DT, fit_error=err)
            steps[axis] = dict(t=t.tolist(), y=y.tolist(), x=None if x is None else x.tolist(), size=size)
            checks.append(_c("hover %s control power" % axis, P, "rad/s2" if axis != "heave" else "m/s2", "", "info",
                             note="damping %.3g 1/s, lag %.3g s, fit %.0f %%" % (d, lags[axis], 100 * err)))
        results["plant"] = plant
        results["steps"] = steps
        results["hover"] = _hover_section(r, hover, plant, s.altitude_m)
        results["performance"] = {}
    finally:
        s.close()
    results["checks"] = checks
    from . import plots
    results["images"] = plots.flight(r, results)
    return results


def _lags(r):
    """The actuators' lags: a helicopter's cyclic follows its flapping lag, its pedals the tail
    rotor's, its collective the design's (or the flapping's); a multirotor's thrust its motors'."""
    if r.kind == "helicopter":
        from . import heli
        rd = heli.rotor_data(r.spec)
        c = r.spec["controls"]
        flap = c.get("flap_lag_s", rd["flap_lag"])
        return {"roll": flap, "pitch": flap, "yaw": rd["tr_flap_lag"], "heave": c.get("collective_lag_s", flap)}
    lag = r.spec["rotors"]["lag_up_s"]
    return {"roll": lag, "pitch": lag, "yaw": lag, "heave": lag}


def _hover_section(r, hover, plant, altitude_m):
    mass = r.spec["mass"]
    kg = mass["mass_kg"] if "mass_kg" in mass else mass["weight_lb"] * 0.45359237
    out = {"altitude_m": altitude_m, "mass_kg": kg, "throttle_trim": hover["throttle"], "aileron_trim": hover["aileron"],
           "elevator_trim": hover["elevator"], "rudder_trim": hover["rudder"], "roll_attitude_deg": hover["phi_deg"],
           "pitch_attitude_deg": hover["theta_deg"]}
    for axis, p in plant.items():
        out[axis + "/power"] = p["power"]
        out[axis + "/damping"] = max(0.0, p["damping"])
        out[axis + "/lag_s"] = p["lag_s"]
    return out


def _c(name, value, unit, expected, status, note=""):
    return {"name": name, "value": float(value), "unit": unit, "expected": expected, "status": status, "note": note}
