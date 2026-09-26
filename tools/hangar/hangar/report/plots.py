"""Plots of every stage's numbers, made to be looked at: curves with the
values a check compares against marked on them."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def polars(polars, path, deltas=None, title="section polars"):
    """cl, cd, cm of named SectionPolars over the whole circle and zoomed
    into the attached range. deltas: flap deflections (deg) to add for the
    first polar that has a flap."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5), dpi=100)
    full = np.radians(np.linspace(-180, 180, 1441))
    near = np.radians(np.linspace(-25, 30, 551))
    for name, p in polars.items():
        for row, a in enumerate((full, near)):
            cl, cd, cm = p.evaluate(a)
            deg = np.degrees(a)
            line, = axes[row, 0].plot(deg, cl, label=name)
            axes[row, 1].plot(deg, cd, color=line.get_color())
            axes[row, 2].plot(deg, cm, color=line.get_color())
            if deltas and p.flap_chord:
                for d in deltas:
                    cl, cd, cm = p.evaluate(a, np.radians(d))
                    kw = dict(color=line.get_color(), ls="--", lw=0.8)
                    axes[row, 0].plot(deg, cl, **kw)
                    axes[row, 1].plot(deg, cd, **kw)
                    axes[row, 2].plot(deg, cm, **kw)
    for row in range(2):
        for col, lab in enumerate(("cl", "cd", "cm (c/4)")):
            ax = axes[row, col]
            ax.set_xlabel("alpha (deg)")
            ax.set_ylabel(lab)
            ax.grid(True, lw=0.3)
            ax.axhline(0, color="k", lw=0.5)
            ax.axvline(0, color="k", lw=0.5)
    axes[0, 0].legend(fontsize=8)
    axes[1, 1].set_ylim(0, 0.12)
    fig.suptitle(title + (" (dashed: flap %s deg)" % ", ".join("%g" % d for d in deltas) if deltas else ""))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def histories(runs, path, title, channels=("alt", "tas", "alpha", "theta", "de", "thr", "vs", "rpm")):
    """Time histories of several runs overlaid (name -> history dict)."""
    labels = {"alt": "altitude (m)", "tas": "true airspeed (m/s)", "kcas": "KCAS", "alpha": "alpha (deg)",
              "beta": "beta (deg)", "theta": "pitch (deg)", "phi": "bank (deg)", "p": "p (deg/s)", "q": "q (deg/s)",
              "r": "r (deg/s)", "de": "elevator (deg)", "da": "aileron (deg)", "dr": "rudder (deg)", "thr": "throttle",
              "vs": "vertical speed (m/s)", "rpm": "propeller rpm", "cl": "CL (from load factor)", "nz": "load factor"}
    n = len(channels)
    cols = 2
    rows = (n + 1) // 2
    fig, axes = plt.subplots(rows, cols, figsize=(14, 2.6 * rows), dpi=100, sharex=True)
    for ax, ch in zip(axes.flat, channels):
        for name, h in runs.items():
            if ch in h:
                ax.plot(h["t"], h[ch], lw=1.0, label=name)
        ax.set_ylabel(labels.get(ch, ch), fontsize=8)
        ax.grid(True, lw=0.3)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("time (s)", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def coefficients(tabs, path, name):
    """The six base coefficients over the whole alpha circle (beta 0 and a
    few sideslips), with a zoom on the flight range and the control
    increments."""
    a, b = tabs["alpha"], tabs["beta"]
    base = tabs["base"]
    fig, axes = plt.subplots(3, 4, figsize=(18, 11), dpi=95)
    betas = [0.0, 10.0, 30.0, 70.0]
    for col, (k, lab) in enumerate((("CL", "CL"), ("CD", "CD"), ("Cm", "Cm (about ARP)"))):
        ax = axes[0, col]
        for bb in betas:
            j = int(np.argmin(np.abs(b - bb)))
            ax.plot(a, base[k][:, j], lw=1.0, label="beta %g" % b[j])
        ax.set_title(lab + ", whole circle", fontsize=9)
        ax.grid(True, lw=0.3)
        ax.set_xlim(-180, 180)
        ax.axhline(0, color="k", lw=0.5)
        ax = axes[1, col]
        j0 = int(np.argmin(np.abs(b)))
        k2 = (a >= -25) & (a <= 35)
        ax.plot(a[k2], base[k][k2, j0], "k", lw=1.4, label="clean")
        for ch, t in tabs["controls"].items():
            if k in t:
                d = t["deflection"]
                for jd in (0, len(d) - 1):
                    if abs(d[jd]) < 1e-9:
                        continue
                    ax.plot(a[k2], base[k][k2, j0] + t[k][k2, jd], lw=0.8, ls="--", label="%s %+g" % (ch, d[jd]))
        ax.set_title(lab + ", flight range, beta 0, with controls", fontsize=9)
        ax.grid(True, lw=0.3)
        ax.axhline(0, color="k", lw=0.5)
        ax.legend(fontsize=6)
    axes[0, 0].legend(fontsize=7)
    # lift-drag polar
    ax = axes[0, 3]
    j0 = int(np.argmin(np.abs(b)))
    k2 = (a >= -10) & (a <= 25)
    ax.plot(base["CD"][k2, j0], base["CL"][k2, j0], "k")
    ld = base["CL"][k2, j0] / np.maximum(base["CD"][k2, j0], 1e-6)
    i = int(np.argmax(ld))
    ax.plot(base["CD"][k2, j0][i], base["CL"][k2, j0][i], "ro")
    ax.set_title("drag polar (L/D max %.1f at CL %.2f)" % (ld[i], base["CL"][k2, j0][i]), fontsize=9)
    ax.set_xlabel("CD")
    ax.set_ylabel("CL")
    ax.grid(True, lw=0.3)
    # lateral coefficients vs beta at a few alphas
    for col, k in enumerate(("CY", "Cl", "Cn")):
        ax = axes[2, col]
        for aa in (0.0, 10.0, 20.0, 40.0):
            i = int(np.argmin(np.abs(a - aa)))
            ax.plot(b, base[k][i, :], lw=1.0, label="alpha %g" % a[i])
        ax.set_title("%s over beta" % k, fontsize=9)
        ax.set_xlabel("beta (deg)")
        ax.grid(True, lw=0.3)
        ax.axhline(0, color="k", lw=0.5)
        ax.legend(fontsize=7)
    ax = axes[2, 3]
    ge = tabs["ground_effect"]
    ax.plot(ge["h_b"], ge["lift"], label="lift factor")
    ax.plot(ge["h_b"], ge["drag"], label="induced drag factor")
    ax.set_title("ground effect (h/b)", fontsize=9)
    ax.grid(True, lw=0.3)
    ax.legend(fontsize=7)
    axes[1, 3].axis("off")
    axes[1, 3].text(0.0, 1.0, _derivative_text(tabs), va="top", family="monospace", fontsize=8)
    fig.suptitle("%s: aerodynamic tables" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _derivative_text(tabs):
    from ..aero.tables import derivatives
    d = derivatives(tabs)
    rows = ["stability derivatives (per rad, alpha 2 deg)", ""]
    keys = [("CLa", "CL_alpha"), ("Cma", "Cm_alpha"), ("CYb", "CY_beta"), ("Clb", "Cl_beta"), ("Cnb", "Cn_beta"),
            ("Clp", "Cl_p"), ("Cnp", "Cn_p"), ("Clr", "Cl_r"), ("Cnr", "Cn_r"), ("CLq", "CL_q"), ("Cmq", "Cm_q"),
            ("Cmad", "Cm_alphadot"), ("Cm_elevator", "Cm_de"), ("CL_elevator", "CL_de"), ("Cl_aileron", "Cl_da"),
            ("Cn_aileron", "Cn_da"), ("Cn_rudder", "Cn_dr"), ("CY_rudder", "CY_dr"), ("CL_flap", "CL_df")]
    for k, lab in keys:
        if k in d:
            rows.append("%-12s %9.4f" % (lab, d[k]))
    return chr(10).join(rows)


def derivatives_vs_alpha(tabs, path, name):
    """Damping and control derivatives over alpha: where they change sign
    (roll damping turning into autorotation past the stall) is where the
    aircraft departs."""
    a = tabs["alpha"]
    k = (a >= -30) & (a <= 60)
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), dpi=95)
    r = tabs["rates"]
    panels = [("Cl", "p", "roll damping Cl_p"), ("Cn", "r", "yaw damping Cn_r"), ("Cm", "q", "pitch damping Cm_q")]
    for ax, (c, rate, lab) in zip(axes[0], panels):
        ax.plot(a[k], r[rate][c][k], "k")
        ax.set_title(lab, fontsize=9)
        ax.axhline(0, color="r", lw=0.6)
        ax.grid(True, lw=0.3)
    ctl = tabs["controls"]
    for ax, (ch, c, lab) in zip(axes[1], (("elevator", "Cm", "Cm from elevator"), ("aileron", "Cl", "Cl from aileron"),
                                           ("rudder", "Cn", "Cn from rudder"))):
        if ch in ctl:
            t = ctl[ch]
            for jd, d in enumerate(t["deflection"]):
                if abs(d) > 1e-9:
                    ax.plot(a[k], t[c][k, jd], lw=0.8, label="%+g deg" % d)
            ax.legend(fontsize=6, ncol=2)
        ax.set_title(lab, fontsize=9)
        ax.axhline(0, color="k", lw=0.5)
        ax.grid(True, lw=0.3)
        ax.set_xlabel("alpha (deg)")
    fig.suptitle("%s: rate and control derivatives over alpha" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def propeller(tabs, path):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), dpi=95)
    for name, t in tabs.items():
        J, ct, cp = t["J"], t["CT"], t["CP"]
        eff = np.where(cp > 1e-6, J * ct / np.maximum(cp, 1e-9), np.nan)
        axes[0].plot(J, ct, label=name)
        axes[1].plot(J, cp, label=name)
        axes[2].plot(J, np.clip(eff, 0, 1), label=name)
    for ax, lab in zip(axes, ("thrust coefficient CT", "power coefficient CP", "efficiency")):
        ax.set_xlabel("advance ratio J")
        ax.set_title(lab, fontsize=9)
        ax.grid(True, lw=0.3)
        ax.axhline(0, color="k", lw=0.5)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def turboprop(maps, path):
    """Each turboprop's constant-speed propeller - efficiency and power
    coefficient over advance ratio at each blade angle, the efficiency's
    envelope - and its engine's shaft power over Mach at a few heights."""
    fig, axes = plt.subplots(len(maps), 3, figsize=(16, 4.4 * len(maps)), dpi=95, squeeze=False)
    for row, (name, m) in zip(axes, maps.items()):
        t = m["tables"]
        J, angles, CT, CP = t["J"], t["blade_angle"], t["CT"], t["CP"]
        eff = np.where(CP > 1e-4, J[:, None] * CT / np.maximum(CP, 1e-4), np.nan)
        for k, b in enumerate(angles):
            c = plt.cm.viridis(k / max(len(angles) - 1, 1))
            ok = (CT[:, k] > 0) & (CP[:, k] > 0)
            row[0].plot(J[ok], eff[ok, k], color=c, lw=0.9, label="%.0f deg" % b)
            row[1].plot(J, CP[:, k], color=c, lw=0.9)
        row[0].plot(J, np.nanmax(np.where(np.isfinite(eff), eff, 0.0), axis=1), "k--", lw=1.2, label="envelope")
        row[0].set_ylim(0, 1)
        row[0].set_title("%s: efficiency at each blade angle (75 %% radius)" % name, fontsize=9)
        row[1].set_title("power coefficient CP", fontsize=9)
        row[1].axhline(0, color="k", lw=0.5)
        for ax in row[:2]:
            ax.set_xlabel("advance ratio J")
            ax.grid(True, lw=0.3)
        row[0].legend(fontsize=7, ncol=2)
        lp = m["lapse"]
        for h, p in zip(lp["altitude_ft"], lp["power"]):
            row[2].plot(lp["mach"], p, "o-", ms=3, label="%.0f ft" % h)
        row[2].set_title("shaft power available / rating", fontsize=9)
        row[2].set_xlabel("Mach")
        row[2].set_ylim(0, 1.1)
        row[2].grid(True, lw=0.3)
        row[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def turbofan(tables, path):
    """Thrust lapse of each turbofan: military and maximum thrust over Mach
    at a few altitudes, as fractions of sea-level-static thrust."""
    from ..propulsion import _ALT_FT, _MACH
    fig, axes = plt.subplots(1, len(tables), figsize=(6.2 * len(tables), 4.2), squeeze=False)
    for ax, (name, tab) in zip(axes[0], tables.items()):
        for j, h in enumerate(_ALT_FT):
            if h < 0 or h > 50000:
                continue
            c = plt.cm.viridis(j / len(_ALT_FT))
            ax.plot(_MACH, [row[j] for row in tab["MilThrust"]], "--", color=c)
            ax.plot(_MACH, [row[j] for row in tab["AugThrust"]], "-", color=c, label="%.0f ft" % h)
        ax.set_title("%s: thrust lapse (solid: afterburner, dashed: military)" % name, fontsize=10)
        ax.set_xlabel("Mach")
        ax.set_ylabel("thrust / sea-level static")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def trim_sweep(runs, path):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.2), dpi=95)
    for name, rows in runs.items():
        ok = [r for r in rows if r["ok"]]
        v = np.array([r["kcas"] for r in ok])
        for ax, key in zip(axes, ("alpha_deg", "elevator_deg", "throttle", "pitch_deg")):
            ax.plot(v, [r[key] for r in ok], "o-", ms=3, label=name)
    for ax, lab in zip(axes, ("trim alpha (deg)", "trim elevator (deg)", "trim throttle", "trim pitch (deg)")):
        ax.set_xlabel("KCAS")
        ax.set_title(lab, fontsize=9)
        ax.grid(True, lw=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("trimmed level flight at sea level")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def climb(runs, path):
    fig, ax = plt.subplots(1, 1, figsize=(7, 4.5), dpi=95)
    for name, c in runs.items():
        rows = [r for r in c["rows"] if r and r.get("rate_ms") is not None and np.isfinite(r["rate_ms"])]
        ax.plot([r["rate_ms"] / 0.3048 * 60 for r in rows], [r["altitude_m"] / 0.3048 for r in rows], "o-", label=name)
    ax.axvline(100, color="k", lw=0.6, ls="--")
    ax.set_xlabel("best rate of climb (ft/min)")
    ax.set_ylabel("altitude (ft)")
    ax.set_title("climb at full throttle (service ceiling at 100 ft/min)", fontsize=9)
    ax.grid(True, lw=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def reference_comparison(cmp, path, name):
    """The design's coefficients over alpha against its reference aircraft
    (a JSBSim model built from wind-tunnel data)."""
    a = cmp["alpha"]
    R, D = cmp["reference"], cmp["design"]
    panels = [("CL", "lift CL"), ("CD", "drag CD"), ("Cm", "pitching moment Cm (about the ARP)"), (None, "drag polar"),
              ("CYb", "side force CY_beta (/rad)"), ("Clb", "dihedral effect Cl_beta (/rad)"),
              ("Cnb", "weathercock Cn_beta (/rad)"), ("Cm_elevator", "elevator power Cm_de (/rad)"),
              ("Clp", "roll damping Cl_p"), ("Cnr", "yaw damping Cn_r"), ("Cmq", "pitch damping Cm_q"),
              ("Cl_aileron", "aileron power Cl_da (/rad); rudder Cn_dr")]
    fig, axes = plt.subplots(3, 4, figsize=(18, 11), dpi=95)
    for ax, (k, title) in zip(axes.flat, panels):
        if k is None:
            ax.plot(R["CD"], R["CL"], "k-", lw=1.4, label="reference")
            ax.plot(D["CD"], D["CL"], "C0-", lw=1.4, label="hangar")
            ax.set_xlabel("CD")
            ax.set_ylabel("CL")
        else:
            if k in R:
                ax.plot(a, R[k], "k-", lw=1.4, label="reference")
            if k in D:
                ax.plot(a, D[k], "C0-", lw=1.4, label="hangar")
            if k == "Cl_aileron":
                if "Cn_rudder" in R:
                    ax.plot(a, R["Cn_rudder"], "k--", lw=1.0, label="reference Cn_dr")
                if "Cn_rudder" in D:
                    ax.plot(a, D["Cn_rudder"], "C0--", lw=1.0, label="hangar Cn_dr")
            ax.set_xlabel("alpha (deg)")
            ax.axhline(0, color="k", lw=0.5)
        ax.set_title(title, fontsize=9)
        ax.grid(True, lw=0.3)
    axes[0, 0].legend(fontsize=8)
    axes[2, 3].legend(fontsize=7)
    fig.suptitle("%s against %s (reference: black)" % (name, cmp["name"]))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def mach_effects(mt, path, name):
    """The compressibility factors over Mach (aero/mach.py)."""
    m = mt["mach"]
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.3), dpi=95)
    ax = axes[0]
    ax.plot(m, mt["K_L"], "k-o", ms=3, label="lift K_L")
    ax.plot(m, mt["K_Y"], "C1-o", ms=3, label="lateral K_Y")
    for ch in ("elevator", "aileron", "rudder", "flap"):
        if "K_" + ch in mt:
            ax.plot(m, mt["K_" + ch], "--", lw=1.0, label=ch)
    ax.set_title("factors on the low-speed values", fontsize=9)
    ax.legend(fontsize=7)
    ax = axes[1]
    ax.plot(m, mt["x_np"], "k-o", ms=3)
    ax.set_title("neutral point x (m)", fontsize=9)
    ax = axes[2]
    ax.plot(m, mt["CD0"] + mt["dCD0"], "k-o", ms=3)
    ax.axvline(mt["M_cr"], color="C1", lw=0.8, ls=":")
    ax.axvline(mt["M_dd"], color="C3", lw=0.8, ls=":")
    ax.set_title("zero-lift drag (critical %.2f, divergence %.2f; A_max %.2f m2, l %.1f m, E_WD %.1f)"
                 % (mt["M_cr"], mt["M_dd"], mt["A_max_m2"], mt["length_m"], mt["E_WD"]), fontsize=8)
    ax = axes[3]
    ax.plot(m, mt["K0"] + mt["dK"] * 0 + (mt["dK"] + mt["K0"] / mt["K_L"] ** 2), "k-o", ms=3)
    ax.set_title("induced-drag factor K (CD_i = K CL^2)", fontsize=9)
    for ax in axes:
        ax.set_xlabel("Mach")
        ax.grid(True, lw=0.3)
    fig.suptitle("%s: compressibility" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fbw_gains(fbw, path, name):
    """The fly-by-wire's gain tables over dynamic pressure, one line per Mach
    number."""
    keys = [("k_alpha", "alpha feedback (rad/rad)"), ("k_q", "pitch-rate feedback (rad/(rad/s))"),
            ("k_ff", "load-factor feedforward (rad/g)"), ("k_roll", "roll-rate feedback (rad/(rad/s))"),
            ("p_max", "roll rate commanded at full stick (rad/s)"), ("k_yaw_r", "yaw damper (rad/(rad/s))")]
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), dpi=95)
    q = fbw["qbar_psf"]
    for ax, (k, title) in zip(axes.flat, keys):
        for j, m in enumerate(fbw["mach"]):
            ax.semilogx(q, fbw["gains"][k][:, j], "-o", ms=3, label="Mach %.2f" % m)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("dynamic pressure (psf)")
        ax.grid(True, lw=0.3)
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("%s: fly-by-wire gains (hangar/fcs.py)" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fighter(runs, path, name):
    """A fighter's performance flown: specific excess power over Mach at sea
    level and at altitude (where it crosses zero: the top speed), the
    sustained turn (speed change over load factor), the ceiling (the best
    excess power at each height, over Mach numbers)."""
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.3), dpi=95)
    for label, r in runs.items():
        l, = axes[0].plot(r["ps_sl"]["mach"], r["ps_sl"]["ps"], lw=1.2, label=label)
        axes[1].plot(r["ps_top"]["mach"], r["ps_top"]["ps"], lw=1.2, color=l.get_color())
        rows = r["turn"]["rows"]
        axes[2].plot([x["n"] for x in rows], [x["dvdt"] for x in rows], "-o", ms=3, color=l.get_color())
        c = r["ceiling_rows"]
        axes[3].plot([x["ps_max"] for x in c], [x["altitude_m"] / 0.3048 for x in c], "-o", ms=3, color=l.get_color())
    first = next(iter(runs.values()))
    axes[0].set_title("excess power at sea level, %s" % ("full afterburner" if first.get("augmented", True) else "full thrust"),
                      fontsize=9)
    top = first["top_altitude_m"] / 0.3048
    axes[1].set_title("excess power at {:,.0f} ft".format(top), fontsize=9)
    axes[2].set_title("level turns at Mach %.1f, 15,000 ft" % first["turn"].get("mach", 0.9), fontsize=9)
    axes[3].set_title("best excess power over height", fontsize=9)
    for ax, xl, yl in zip(axes, ("Mach", "Mach", "load factor", "P_s (m/s)"), ("P_s (m/s)", "P_s (m/s)", "dV/dt (m/s2)", "ft")):
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.grid(True, lw=0.3)
        ax.axhline(0, color="k", lw=0.5) if ax is not axes[3] else ax.axvline(0.508, color="k", lw=0.5)
    axes[0].legend(fontsize=7)
    fig.suptitle("%s: fighter performance, flown" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def autopilot(flown, path, name):
    """The platform's loops with the aircraft's own gains: each manoeuvre's
    response at the three speeds flown, over its target."""
    panels = (("bank", "30 deg bank (attitude)", "deg"), ("pitch", "5 deg pitch up (attitude)", "deg"),
              ("climb", "climb at 5 % of the speed (velocity)", "m/s"), ("heading", "90 deg turn (velocity)", "deg"),
              ("load_factor", "1.5 g (acceleration)", "g above 1"), ("roll_rate", "roll rate 0.25 rad/s (acceleration)", "rad/s"))
    fig, axes = plt.subplots(2, 3, figsize=(16, 7.5), dpi=95)
    for ax, (key, title, unit) in zip(axes.flat, panels):
        for r in flown:
            m = r.get(key) or {}
            if m.get("lost"):
                ax.plot([], [], label="%.0f m/s: lost" % r["speed_ms"])
                continue
            if "trace" in m:
                line, = ax.plot(m["trace"]["t"], m["trace"]["y"], lw=1.2, label="%.0f m/s" % r["speed_ms"])
                if m.get("target") is not None:   # each speed's own (a climb's is 5 % of it)
                    ax.axhline(m["target"], color=line.get_color(), lw=0.6, ls="--")
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("s")
        ax.set_ylabel(unit)
        ax.grid(True, lw=0.3)
        ax.legend(fontsize=7)
    fig.suptitle("%s: the platform's control loops with its own gains" % name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
