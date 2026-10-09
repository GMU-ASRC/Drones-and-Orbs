"""Plot and diagnose a baro_takeoff_test run, with concrete tuning suggestions.

Run: python3 plot_run.py                 # newest run under runs/
     python3 plot_run.py runs/<name>     # a specific run
     python3 plot_run.py --no-plot       # diagnostics only, no matplotlib needed
"""

import argparse
import csv
import glob
import os
import statistics as st

# Gates used to turn a measurement into advice. These are judgement calls, not
# vendor spec -- edit them as you learn what this airframe tolerates.
THRUST_JITTER_MAX = 0.03    # high-frequency thrust noise we consider acceptable
SAT_PCT_MAX = 5.0           # % of samples pinned at a thrust limit
OVERSHOOT_PCT_MAX = 25.0
OSCILLATION_HZ_MAX = 0.8    # qualified climb-error sign changes per second
OSCILLATION_DEADBAND_MPS = 0.25   # a swing smaller than this is noise, not ringing
SETTLE_ERR_M = 0.10

PANELS = ("altitude", "climb", "thrust", "attitude", "timing")


def newest_run(base):
    runs = sorted(glob.glob(os.path.join(base, "runs", "*")))
    if not runs:
        raise SystemExit("no runs found under %s/runs" % base)
    return runs[-1]


def load(run_dir):
    """Return {column: [float or str]}. Missing columns simply aren't there."""
    path = os.path.join(run_dir, "flight.csv")
    if not os.path.exists(path):
        raise SystemExit("no flight.csv in %s" % run_dir)

    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("flight.csv is empty")

    out = {}
    for key in rows[0]:
        vals = [r[key] for r in rows]
        try:
            out[key] = [float(v) for v in vals]
        except ValueError:
            out[key] = vals
    return out


def highpass(v, n=10):
    """What's left of v after removing an n-sample moving average."""
    out = []
    for i in range(len(v)):
        window = v[max(0, i - n + 1):i + 1]
        out.append(v[i] - st.mean(window))
    return out


def phase_slice(d, name):
    idx = [i for i, p in enumerate(d.get("phase", [])) if p == name]
    return idx


def sign_flips(v, deadband):
    """Sign changes that actually swing past +/-deadband.

    A plain zero-crossing count is useless here: the climb estimate hovers
    around zero in noise, which reads as tens of flips per second even when
    the loop is behaving.
    """
    flips, state = 0, 0
    for x in v:
        if x > deadband and state != 1:
            flips += state == -1
            state = 1
        elif x < -deadband and state != -1:
            flips += state == 1
            state = -1
    return flips


def describe(d, run_dir):
    t = d["t"]
    dt = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    has = lambda k: k in d and any(v != 0.0 for v in d[k])

    print("=" * 70)
    print("run        %s" % os.path.basename(run_dir))
    print("samples    %d over %.2f s" % (len(t), t[-1] - t[0]))
    print("loop       %.2f Hz, jitter %.2f ms (sigma), worst gap %.1f ms"
          % (1 / st.mean(dt), st.pstdev(dt) * 1000, max(dt) * 1000))
    phases = []
    for p in ("dry", "bench", "climb", "hover", "land"):
        n = len(phase_slice(d, p))
        if n:
            phases.append("%s %d" % (p, n))
    print("phases     %s" % (", ".join(phases) or "none recorded"))

    print("-" * 70)
    print("ALTITUDE SIGNAL")
    for key, label in (("alt_raw_m", "raw baro"), ("baro_alt_m", "raw baro"),
                       ("alt_filt_m", "filtered"), ("rng_m", "rangefinder"),
                       ("fused_alt_m", "EKF fused")):
        if key in d:
            v = d[key]
            print("  %-12s sigma %.4f m   p2p %.3f m" % (label, st.pstdev(v), max(v) - min(v)))
    if "press_hpa" in d:
        levels = len(set(d["press_hpa"]))
        print("  %-12s %d distinct values (0.01 hPa = 0.084 m per step)"
              % ("pressure", levels))

    if has("thrust"):
        print("-" * 70)
        print("CONTROL")
        thr = d["thrust"]
        jitter = st.pstdev(highpass(thr))
        print("  thrust       mean %.3f   range %.3f..%.3f   jitter(sigma) %.4f"
              % (st.mean(thr), min(thr), max(thr), jitter))
        if has("thrust_p") or has("thrust_i"):
            print("  P term       sigma %.4f   |max| %.3f"
                  % (st.pstdev(d["thrust_p"]), max(abs(v) for v in d["thrust_p"])))
            print("  I term       final %.4f   |max| %.3f"
                  % (d["thrust_i"][-1], max(abs(v) for v in d["thrust_i"])))
        if has("climb_err_mps"):
            print("  climb error  RMS %.3f m/s   |max| %.3f m/s"
                  % (rms(d["climb_err_mps"]), max(abs(v) for v in d["climb_err_mps"])))

        # At steady hover the P term averages out, so whatever the integral
        # settled on IS the error in HOVER_THRUST. This is the only way to
        # measure real hover thrust -- a props-off bench ramp cannot.
        hover = phase_slice(d, "hover")
        if hover and has("thrust_i"):
            tail = [d["thrust_i"][i] for i in hover[len(hover) // 2:]]
            print("  hover thrust implied %.3f  (HOVER_THRUST + %+.3f from the "
                  "integral)" % (0.5 + st.mean(tail), st.mean(tail)))

    if "roll_deg" in d:
        print("-" * 70)
        print("ATTITUDE")
        for key in ("roll_deg", "pitch_deg", "yaw_deg"):
            v = d[key]
            print("  %-10s sigma %.4f deg   drift %+.4f deg/s"
                  % (key[:-4], st.pstdev(v), slope(t, v)))


def rms(v):
    return (sum(x * x for x in v) / len(v)) ** 0.5


def slope(t, v):
    mt, mv = st.mean(t), st.mean(v)
    den = sum((x - mt) ** 2 for x in t)
    return sum((t[i] - mt) * (v[i] - mv) for i in range(len(t))) / den if den else 0.0


def suggest(d, target_alt_m):
    """Turn the measurements into things to change, loudest first."""
    out = []
    t = d["t"]
    has = lambda k: k in d and any(v != 0.0 for v in d[k])

    if has("thrust"):
        thr = d["thrust"]
        jitter = st.pstdev(highpass(thr))
        if jitter > THRUST_JITTER_MAX:
            out.append("Thrust jitter %.4f exceeds %.2f. Raise BARO_TAU_S "
                       "(try %.2f) or lower --kp-climb."
                       % (jitter, THRUST_JITTER_MAX, 0.30 * jitter / THRUST_JITTER_MAX))

        pinned = sum(1 for v in thr if v <= 0.001 or v >= 0.699)
        sat = 100.0 * pinned / len(thr)
        if sat > SAT_PCT_MAX:
            out.append("Thrust pinned at a limit %.1f%% of the run. Lower "
                       "--kp-alt or MAX_CLIMB_MPS; if it pins high while "
                       "climbing, the airframe is thrust-limited." % sat)

    hover = phase_slice(d, "hover")
    if hover and "alt_filt_m" in d:
        alt = [d["alt_filt_m"][i] for i in hover]
        peak = max(alt)
        over = 100.0 * (peak - target_alt_m) / target_alt_m
        if over > OVERSHOOT_PCT_MAX:
            out.append("Overshoot %.0f%% (peak %.2f m vs %.2f m target). Lower "
                       "--kp-alt, or lower MAX_CLIMB_MPS so the approach is slower."
                       % (over, peak, target_alt_m))

        tail = alt[len(alt) // 2:]
        err = abs(st.mean(tail) - target_alt_m)
        if err > SETTLE_ERR_M:
            out.append("Steady-state error %.3f m over the back half of hover. "
                       "Raise --ki-climb (try %.3f) so the integral trims the "
                       "hover thrust." % (err, 0.06 * 1.5))

    if hover and has("climb_err_mps"):
        e = [d["climb_err_mps"][i] for i in hover]
        span = t[hover[-1]] - t[hover[0]]
        hz = sign_flips(e, OSCILLATION_DEADBAND_MPS) / span if span else 0.0
        if hz > OSCILLATION_HZ_MAX:
            out.append("Climb error changes sign %.1f times/s while hovering -- "
                       "the inner loop is ringing. Lower --kp-climb (try %.3f)."
                       % (hz, 0.12 * 0.7))

    climb = phase_slice(d, "climb")
    if climb and "alt_filt_m" in d:
        rise_s = t[climb[-1]] - t[climb[0]]
        if rise_s > 12.0:
            out.append("Climb took %.1f s to settle. Raise --kp-alt or "
                       "MAX_CLIMB_MPS if the thrust was never pinned." % rise_s)

    if "yaw_deg" in d and abs(slope(t, d["yaw_deg"])) > 0.05:
        out.append("Yaw drifts %+.3f deg/s with no absolute reference. Harmless "
                   "for a hop; do not trust heading on a longer run."
                   % slope(t, d["yaw_deg"]))

    print("-" * 70)
    if not out:
        print("SUGGESTIONS\n  Nothing flagged. Every gate in this script passed.")
    else:
        print("SUGGESTIONS")
        for i, s in enumerate(out, 1):
            print("  %d. %s" % (i, s))
    print("=" * 70)


def plot(d, run_dir, save, panels):
    try:
        import matplotlib
        if save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("matplotlib not installed. Use --no-plot for "
                         "diagnostics, or: pip3 install matplotlib")

    t = d["t"]
    want = [p for p in PANELS if p in panels]
    fig, axes = plt.subplots(len(want), 1, figsize=(11, 2.5 * len(want)),
                             sharex=True)
    if len(want) == 1:
        axes = [axes]
    fig.suptitle(os.path.basename(run_dir), fontsize=11)

    for ax, name in zip(axes, want):
        if name == "altitude":
            if "alt_sp_m" in d:
                ax.plot(t, d["alt_sp_m"], "k--", lw=1, label="setpoint")
            for key, label, style in (("alt_raw_m", "raw baro", {"lw": .8, "alpha": .55}),
                                      ("baro_alt_m", "raw baro", {"lw": .8, "alpha": .55}),
                                      ("alt_filt_m", "filtered", {"lw": 2}),
                                      ("rng_m", "rangefinder", {"lw": 1}),
                                      ("fused_alt_m", "EKF fused", {"lw": 1})):
                if key in d:
                    ax.plot(t, d[key], label=label, **style)
            ax.set_ylabel("alt (m)")

        elif name == "climb":
            if "climb_sp_mps" in d:
                ax.plot(t, d["climb_sp_mps"], "k--", lw=1, label="demand")
            if "climb_mps" in d:
                ax.plot(t, d["climb_mps"], lw=2, label="measured")
            ax.axhline(0, color="gray", lw=.6)
            ax.set_ylabel("climb (m/s)")

        elif name == "thrust":
            if "thrust" in d:
                ax.plot(t, d["thrust"], lw=2, label="thrust")
            for key, label in (("thrust_p", "P term"), ("thrust_i", "I term")):
                if key in d:
                    ax.plot(t, [0.5 + v for v in d[key]], lw=1, label=label + " (+hover)")
            ax.axhline(0.5, color="gray", ls=":", lw=1, label="hover")
            ax.axhline(0.70, color="crimson", ls=":", lw=1, label="limit")
            ax.set_ylabel("thrust")

        elif name == "attitude":
            for key in ("roll_deg", "pitch_deg", "yaw_deg"):
                if key in d:
                    ax.plot(t, d[key], lw=1, label=key[:-4])
            ax.set_ylabel("deg")

        elif name == "timing":
            dt = [0] + [(t[i] - t[i - 1]) * 1000 for i in range(1, len(t))]
            ax.plot(t, dt, lw=.8)
            ax.axhline(1000 / 20.0, color="gray", ls=":", lw=1, label="20 Hz")
            ax.set_ylabel("loop dt (ms)")

        ax.grid(alpha=.18)
        ax.legend(fontsize=8, loc="upper right", ncol=3)

    axes[-1].set_xlabel("t (s)")
    fig.tight_layout()

    if save:
        fig.savefig(save, dpi=130)
        print("wrote %s" % save)
    else:
        plt.show()


def parse_args():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_dir", nargs="?", default=None,
                   help="run directory; default is the newest under runs/")
    p.add_argument("--alt", type=float, default=1.0, metavar="M",
                   help="target altitude the run was commanded to")
    p.add_argument("--no-plot", action="store_true",
                   help="print diagnostics only; works without matplotlib")
    p.add_argument("--save", default=None, metavar="PNG",
                   help="write the figure instead of opening a window")
    p.add_argument("--panels", default=",".join(PANELS), metavar="LIST",
                   help="comma-separated subset of: " + ",".join(PANELS))
    args = p.parse_args()
    args.run_dir = args.run_dir or newest_run(here)
    return args


def main():
    args = parse_args()
    d = load(args.run_dir)

    describe(d, args.run_dir)
    suggest(d, args.alt)

    if not args.no_plot:
        plot(d, args.run_dir, args.save, args.panels.split(","))


if __name__ == "__main__":
    main()
