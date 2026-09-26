"""The rotorcraft flight tests' plots (out/fly_*.png): the hover steps against the identified plant,
and a helicopter's trim across its speed range against the flight data it is checked with."""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEG = 57.29577951308232


def flight(r, res):
    """The fly stage's images, written to r.out; returns their file names."""
    images = []
    if res.get("steps"):
        images.append(_steps(r, res))
    if res.get("trim_table"):
        images.append(_trim_table(r, res))
    return images


def replay(step, plant):
    """The identified plant's answer to the step (fly.identify's model): a rate integrated from the
    first sample, or the heave's acceleration from the measured vertical speed."""
    from .fly import step_input
    t = np.array(step["t"])
    u_f = step_input(t, step["size"], plant["lag_s"], plant.get("delay_s", 0.0))
    if step.get("x") is not None:
        return -plant["damping"] * np.array(step["x"]) + plant["power"] * u_f
    y = np.empty_like(t)
    y[0] = step["y"][0]
    for k in range(1, len(t)):
        y[k] = y[k - 1] + (t[k] - t[k - 1]) * (-plant["damping"] * y[k - 1] + plant["power"] * u_f[k - 1])
    return y


def _steps(r, res):
    labels = {"roll": ("roll rate p (deg/s)", DEG), "pitch": ("pitch rate q (deg/s)", DEG), "yaw": ("yaw rate r (deg/s)", DEG),
              "heave": ("vertical acceleration (m/s2)", 1.0)}
    command = {"roll": "aileron", "pitch": "elevator", "yaw": "rudder", "heave": "throttle"}
    fig, axes = plt.subplots(2, 2, figsize=(12, 6.5), dpi=100)
    for ax, axis in zip(axes.flat, ("roll", "pitch", "yaw", "heave")):
        step, plant = res["steps"].get(axis), res["plant"].get(axis)
        if not step:
            ax.set_visible(False)
            continue
        label, k = labels[axis]
        t = np.array(step["t"])
        ax.plot(t, k * np.array(step["y"]), lw=1.4, label="the aircraft")
        ax.plot(t, k * replay(step, plant), "--", lw=1.2, label="identified plant")
        ax.set_title("%s %+g (%s): power %.3g, damping %.3g 1/s, lag %.3g s" % (command[axis], step["size"], axis, plant["power"],
                                                                                  plant["damping"], plant["lag_s"]), fontsize=9)
        ax.set_ylabel(label, fontsize=8)
        ax.set_xlabel("time after the step (s)", fontsize=8)
        ax.grid(True, lw=0.3)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(fontsize=8)
    fig.suptitle("%s: hover steps from the trim (the platform's normalised commands)" % r.name)
    fig.tight_layout()
    name = "fly_steps.png"
    fig.savefig(os.path.join(r.out, name))
    plt.close(fig)
    return name


def _trim_table(r, res):
    panels = (("coll_in", "collective (in)"), ("long_in", "longitudinal stick (in, + forward)"), ("lat_in", "lateral stick (in, + right)"),
              ("pedal_in", "pedal (in, + right)"), ("theta_deg", "pitch attitude (deg)"), ("phi_deg", "bank (deg)"))
    rows = res["trim_table"]
    source = r.spec.get("targets", {}).get("trim_source", "the reference")
    kt = [row["target"]["kt"] for row in rows]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.5), dpi=100)
    for ax, (key, label) in zip(axes.flat, panels):
        ax.plot(kt, [row["target"].get(key, np.nan) for row in rows], "o", ms=6, mfc="none", label=source)
        ax.plot(kt, [row["model"].get(key, np.nan) for row in rows], ".-", lw=1.2, label="this model")
        ax.set_ylabel(label, fontsize=8)
        ax.set_xlabel("airspeed (kt)", fontsize=8)
        ax.grid(True, lw=0.3)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(fontsize=8)
    fig.suptitle("%s: level-flight trim against %s" % (r.name, source))
    fig.tight_layout()
    name = "fly_trim.png"
    fig.savefig(os.path.join(r.out, name))
    plt.close(fig)
    return name
