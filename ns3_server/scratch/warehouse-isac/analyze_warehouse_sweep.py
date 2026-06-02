#!/usr/bin/env python3
"""Summarize one warehouse array sweep without hiding non-monotonic results."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def read_summary_value(path: Path, key: str) -> float:
    if not path.exists():
        return 0.0
    prefix = f"{key}:"
    for line in path.read_text().splitlines():
        if line.startswith(prefix):
            return float(line.removeprefix(prefix).strip())
    return 0.0


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def challenge_flow(row: dict[str, str]) -> bool:
    port = int(row["DstPort"])
    return 12000 <= port < 16000


def summarize_run(results_dir: Path, mode: str, size: int) -> dict[str, object]:
    run_dir = results_dir / f"{mode}-{size}x{size}"
    flows = [row for row in read_csv(run_dir / "flow_stats.csv") if challenge_flow(row)]
    propagation = read_csv(run_dir / "propagation_stats.csv")
    gains = read_csv(run_dir / "mimo_channel_gain_stats.csv")
    detections = [
        row
        for row in read_csv(run_dir / "sensing_stats.csv")
        if row.get("DetectionIndex", "").strip()
    ]

    tx_packets = sum(int(row["TxPackets"]) for row in flows)
    rx_packets = sum(int(row["RxPackets"]) for row in flows)
    return {
        "mode": mode,
        "array": f"{size}x{size}",
        "challenge_flows": len(flows),
        "delivery_pct": 100.0 * rx_packets / tx_packets if tx_packets else 0.0,
        "goodput_kbps": sum(float(row["SimGoodput_Kbps"]) for row in flows),
        "mean_loss_db": mean([float(row["Loss_dB"]) for row in propagation]),
        "mean_array_gain_db": mean(
            [float(row["AvgEffectiveChannelGain_dB"]) for row in gains]
        ),
        "detections": len(detections),
        "wall_clock_s": read_summary_value(run_dir / "summary.txt", "simulator_run_wall_clock_s"),
    }


def nondecreasing(values: list[float], tolerance: float = 1e-9) -> bool:
    return all(right + tolerance >= left for left, right in zip(values, values[1:]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results_dir", type=Path)
    args = parser.parse_args()

    rows = [
        summarize_run(args.results_dir, mode, size)
        for mode in ("no-isac", "isac")
        for size in (2, 4, 8)
    ]
    output = args.results_dir / "sweep_summary.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print("mode     array  delivery   goodput Kbps  loss dB  array gain dB  detections  wall s")
    for row in rows:
        print(
            f"{row['mode']:8s} {row['array']:>4s}  {row['delivery_pct']:8.2f}%"
            f"  {row['goodput_kbps']:12.2f}  {row['mean_loss_db']:7.2f}"
            f"  {row['mean_array_gain_db']:13.2f}  {row['detections']:10d}"
            f"  {row['wall_clock_s']:6.1f}"
        )

    for mode in ("no-isac", "isac"):
        selected = [row for row in rows if row["mode"] == mode]
        gains = [float(row["mean_array_gain_db"]) for row in selected]
        print(f"{mode}: monotonic array gain = {nondecreasing(gains)}")
    print(f"summary: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
