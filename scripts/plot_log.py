"""Plot a telemetry CSV from logs/ — turn a run into a picture.

Usage:
    python scripts/plot_log.py                 # newest log in logs/
    python scripts/plot_log.py logs/foo.csv    # a specific log
    python scripts/plot_log.py --show          # open a window too

Saves <logfile>.png next to the CSV either way. Plots whatever
recognised columns the file has (depth/heading/efforts + setpoints),
so it works for depth_hold, heading_hold, teleop_assisted and mission
logs alike.

Needs matplotlib:  pip install matplotlib
"""

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")            # save without needing a display...
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[1]

# (column, matching setpoint column, axis label)
PANELS = [
    ("depth", "depth_sp", "depth (m, +down)"),
    ("heading", "heading_sp", "heading (deg)"),
    ("effort", None, "effort"),
    ("v_eff", "y_eff", "efforts"),   # both efforts share one panel
]


def newest_log() -> Path:
    logs = sorted((REPO / "logs").glob("*.csv"),
                  key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit("No CSV files in logs/ — run something first.")
    return logs[-1]


def column(rows, name):
    """Extract a numeric column; None where blank/absent."""
    out = []
    for r in rows:
        v = r.get(name, "")
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(None)
    return out if any(v is not None for v in out) else None


def main() -> None:
    show = "--show" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = Path(args[0]) if args else newest_log()

    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path} is empty.")
    t = column(rows, "t") or list(range(len(rows)))

    panels = []
    for main_col, sp_col, label in PANELS:
        series = column(rows, main_col)
        if series is None:
            continue
        sp = column(rows, sp_col) if sp_col else None
        panels.append((main_col, series, sp_col, sp, label))
    if not panels:
        sys.exit("No recognised columns to plot.")

    fig, axes = plt.subplots(len(panels), 1, sharex=True,
                             figsize=(10, 2.8 * len(panels)))
    if len(panels) == 1:
        axes = [axes]
    for ax, (name, series, sp_name, sp, label) in zip(axes, panels):
        ax.plot(t, series, label=name)
        if sp:
            ax.plot(t, sp, "--", label=sp_name)
        if name == "depth":
            ax.invert_yaxis()        # deeper = down on the page
        ax.set_ylabel(label)
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle(path.name)
    fig.tight_layout()

    out = path.with_suffix(".png")
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    if show:
        matplotlib.use("TkAgg")
        plt.show()


if __name__ == "__main__":
    main()
