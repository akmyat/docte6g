#!/usr/bin/env python3
"""Aggregate results.csv (median + IQR over reps) and emit research-ready plots.

Outputs (in this directory):
  - results_aggregated.csv       median / min / max / reps per (backend, regime, N)
  - plot_<regime>.png            wall-clock vs N, log-log, three lines, error bars
  - plot_overhead_vs_pure_ns3.png  ns3sionna and sionnart overhead factor vs pure_ns3
"""

import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_IN = os.path.join(HERE, "results.csv")

REGIMES = ["stationary_high_load", "low_mob_low_load", "high_mob_high_load"]
REGIME_TITLE = {
    "stationary_high_load": "Stationary / high load (UDP every 20 ms)",
    "low_mob_low_load":     "Low mobility (1 m/s) / low load (UDP every 1000 ms)",
    "high_mob_high_load":   "High mobility (7 m/s) / high load (UDP every 20 ms)",
}

BACKENDS = ["pure_ns3", "ns3sionna", "sionnart"]
STYLE = {
    "pure_ns3":  dict(color="#1f77b4", marker="o", label="pure ns-3 (Friis)"),
    "ns3sionna": dict(color="#d62728", marker="s", label="ns3sionna (ZMQ)"),
    "sionnart":  dict(color="#2ca02c", marker="^", label="sionnart (embedded)"),
}


def load():
    """{(backend, regime, N): [t1, t2, ...]}"""
    g = defaultdict(list)
    with open(CSV_IN) as f:
        for row in csv.DictReader(f):
            key = (row["backend"], row["regime"], int(row["num_stas"]))
            g[key].append(float(row["wall_clock_s"]))
    return g


def median(vs):
    s = sorted(vs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def aggregate(g):
    rows = []
    for (b, r, n), vs in sorted(g.items()):
        rows.append(dict(backend=b, regime=r, num_stas=n, reps=len(vs),
                         min=min(vs), median=median(vs), max=max(vs),
                         mean=sum(vs) / len(vs)))
    return rows


def write_aggregated(rows):
    out = os.path.join(HERE, "results_aggregated.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["backend", "regime", "num_stas",
                                          "reps", "min", "median", "max", "mean"])
        w.writeheader()
        for r in rows:
            r2 = dict(r)
            for k in ("min", "median", "max", "mean"):
                r2[k] = round(r2[k], 4)
            w.writerow(r2)
    print(f"wrote {out}")


def is_sionnart_anomaly(regime, n, t):
    """Disabled after the 2026-05-02 naming-convention fix in sionnart_main.cc.
    Pre-fix, sionnart's wall-clock was bimodal because STA-STA pairs were
    incorrectly classified as TX-RX links by IsSameRoleLink (substring match
    for 'Tx'/'Rx'); names 'AP'/'STA_*' failed that test and triggered
    Python-side path-tracing failures. After renaming to 'Tx1'/'Rx<i+1>',
    scaling is monotonic and no rows need to be flagged."""
    return False


def plot_per_regime(g):
    for regime in REGIMES:
        fig, ax = plt.subplots(figsize=(7.5, 5.0))
        for b in BACKENDS:
            xs, ys, ymin, ymax = [], [], [], []
            anom_x, anom_y = [], []
            for n in [1, 2, 4, 8, 16, 32]:
                vs = g.get((b, regime, n))
                if not vs:
                    continue
                med = median(vs)
                if b == "sionnart" and is_sionnart_anomaly(regime, n, med):
                    anom_x.append(n); anom_y.append(med)
                    continue
                xs.append(n); ys.append(med)
                ymin.append(med - min(vs)); ymax.append(max(vs) - med)
            if xs:
                ax.errorbar(xs, ys, yerr=[ymin, ymax], capsize=3, linewidth=2,
                            markersize=8, **STYLE[b])
            if anom_x:
                ax.scatter(anom_x, anom_y, marker="x", s=80,
                           color=STYLE[b]["color"], alpha=0.5,
                           label=f"{STYLE[b]['label']} (anomalous)")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xticks([1, 2, 4, 8, 16, 32])
        ax.set_xticklabels(["1", "2", "4", "8", "16", "32"])
        ax.set_xlabel("Number of STAs (log₂)")
        ax.set_ylabel("Wall-clock simulation time [s] (log)")
        ax.set_title(REGIME_TITLE[regime])
        ax.grid(True, which="both", linestyle=":", alpha=0.5)
        ax.legend(loc="upper left", fontsize=9)
        fig.tight_layout()
        out = os.path.join(HERE, f"plot_{regime}.png")
        fig.savefig(out, dpi=140)
        plt.close(fig)
        print(f"wrote {out}")


def plot_overhead(g):
    """Overhead factor = t_backend / t_pure_ns3 per (regime, N). Excludes
    sionnart anomalies."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)
    for ax, regime in zip(axes, REGIMES):
        for b in ("ns3sionna", "sionnart"):
            xs, ys = [], []
            for n in [1, 2, 4, 8, 16, 32]:
                base = g.get(("pure_ns3", regime, n))
                cand = g.get((b, regime, n))
                if not base or not cand:
                    continue
                if b == "sionnart" and is_sionnart_anomaly(regime, n, median(cand)):
                    continue
                xs.append(n)
                ys.append(median(cand) / median(base))
            if xs:
                ax.plot(xs, ys, linewidth=2, markersize=8, **STYLE[b])
        ax.axhline(1.0, color="#1f77b4", linestyle="--", alpha=0.6,
                   label="pure ns-3 baseline (1×)")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xticks([1, 2, 4, 8, 16, 32])
        ax.set_xticklabels(["1", "2", "4", "8", "16", "32"])
        ax.set_xlabel("Number of STAs")
        ax.set_title(REGIME_TITLE[regime].split("/")[0].strip())
        ax.grid(True, which="both", linestyle=":", alpha=0.5)
        ax.legend(loc="upper left", fontsize=8)
    axes[0].set_ylabel("Wall-clock overhead vs pure ns-3 (×)")
    fig.suptitle("Ray-tracing channel overhead per regime", y=1.02)
    fig.tight_layout()
    out = os.path.join(HERE, "plot_overhead_vs_pure_ns3.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def main():
    g = load()
    rows = aggregate(g)
    write_aggregated(rows)
    plot_per_regime(g)
    plot_overhead(g)


if __name__ == "__main__":
    main()
