#!/usr/bin/env python3
"""Plot three-way benchmark results.

Reads results.csv (written by run_three_way.sh) and emits one PNG per regime:
x = num_stas (log2), y = wall_clock_s, three lines (pure_ns3 / ns3sionna / sionnart).

Usage:
    python plot_results.py [results.csv]
"""

import csv
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt

BACKEND_STYLE = {
    "pure_ns3":  {"color": "#1f77b4", "marker": "o", "label": "pure ns-3 (Friis)"},
    "ns3sionna": {"color": "#d62728", "marker": "s", "label": "ns3sionna (ZMQ)"},
    "sionnart":  {"color": "#2ca02c", "marker": "^", "label": "sionnart (embedded)"},
}


def load(path):
    by_regime = defaultdict(lambda: defaultdict(list))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            n = int(row["num_stas"])
            t = float(row["wall_clock_s"])
            by_regime[row["regime"]][row["backend"]].append((n, t))
    return by_regime


def plot_regime(regime, series, out_path):
    fig, ax = plt.subplots(figsize=(7, 5))
    for backend, points in series.items():
        points.sort()
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        style = BACKEND_STYLE.get(backend, {"label": backend})
        ax.plot(xs, ys, linewidth=2, markersize=7, **style)
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    ax.set_xlabel("Number of STAs")
    ax.set_ylabel("Wall-clock time [s]")
    ax.set_title(f"Three-way benchmark — {regime}")
    ax.grid(True, which="both", linestyle=":", alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"wrote {out_path}")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    csv_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "results.csv")
    if not os.path.exists(csv_path):
        sys.exit(f"no such file: {csv_path}")

    by_regime = load(csv_path)
    if not by_regime:
        sys.exit("results.csv is empty")

    for regime, series in by_regime.items():
        out = os.path.join(here, f"plot_{regime}.png")
        plot_regime(regime, series, out)


if __name__ == "__main__":
    main()
