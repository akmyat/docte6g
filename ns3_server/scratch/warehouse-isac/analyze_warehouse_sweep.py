#!/usr/bin/env python3
"""Summarize one warehouse array sweep without hiding non-monotonic results."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def csv_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes"}


def read_summary_value(path: Path, key: str) -> float | bool | str:
    if not path.exists():
        return 0.0
    prefix = f"{key}:"
    for line in path.read_text().splitlines():
        if line.startswith(prefix):
            value = line.removeprefix(prefix).strip()
            if value.lower() == "true":
                return True
            if value.lower() == "false":
                return False
            try:
                return float(value)
            except ValueError:
                return value
    return 0.0


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def challenge_flow(row: dict[str, str]) -> bool:
    port = int(row["DstPort"])
    return 12000 <= port < 16000


def port_class(row: dict[str, str]) -> str:
    port = int(row["DstPort"])
    if 12000 <= port < 13000:
        return "static"
    if 13000 <= port < 14000:
        return "robot"
    if 14000 <= port < 16000:
        return "uplink"
    return "other"


def summarize_flows(flows: list[dict[str, str]]) -> dict[str, float]:
    tx_packets = sum(int(row["TxPackets"]) for row in flows)
    rx_packets = sum(int(row["RxPackets"]) for row in flows)
    return {
        "flows": len(flows),
        "delivery_pct": 100.0 * rx_packets / tx_packets if tx_packets else 0.0,
        "goodput_kbps": sum(float(row["SimGoodput_Kbps"]) for row in flows),
    }


def robot_motion_distance_m(run_dir: Path, robot_ue_start_index: int, robot_count: int = 3) -> float:
    rows = read_csv(run_dir / "mobility_trace.csv")
    if not rows:
        return 0.0

    # Node creation order in the scenario is one gNB followed by UE nodes.
    robot_node_ids = {
        str(1 + robot_ue_start_index + offset) for offset in range(robot_count)
    }
    last_pos: dict[str, tuple[float, float, float]] = {}
    total = 0.0
    for row in rows:
        node_id = row["NodeId"]
        if node_id not in robot_node_ids:
            continue
        pos = (float(row["X"]), float(row["Y"]), float(row["Z"]))
        previous = last_pos.get(node_id)
        if previous is not None:
            total += math.dist(previous, pos)
        last_pos[node_id] = pos
    return total


def summarize_run(results_dir: Path, mode: str, size: int) -> dict[str, object] | None:
    run_dir = results_dir / f"{mode}-{size}x{size}"
    summary_path = run_dir / "summary.txt"
    flow_path = run_dir / "flow_stats.csv"
    if not summary_path.exists() or not flow_path.exists():
        return None
    flows = [row for row in read_csv(flow_path) if challenge_flow(row)]
    static_flows = [row for row in flows if port_class(row) == "static"]
    robot_flows = [row for row in flows if port_class(row) == "robot"]
    aggregate = summarize_flows(flows)
    static = summarize_flows(static_flows)
    robot = summarize_flows(robot_flows)
    propagation = read_csv(run_dir / "propagation_stats.csv")
    gains = read_csv(run_dir / "mimo_channel_gain_stats.csv")
    detections = [
        row
        for row in read_csv(run_dir / "sensing_stats.csv")
        if row.get("DetectionIndex", "").strip()
    ]
    beams = [
        row
        for row in read_csv(run_dir / "isac_beam_stats.csv")
        if csv_bool(row.get("Active", ""))
    ]
    robot_ue_start = int(read_summary_value(summary_path, "mobile_robot_ue_start_index") or 3)

    return {
        "mode": mode,
        "array": f"{size}x{size}",
        "challenge_flows": aggregate["flows"],
        "delivery_pct": aggregate["delivery_pct"],
        "goodput_kbps": aggregate["goodput_kbps"],
        "static_delivery_pct": static["delivery_pct"],
        "static_goodput_kbps": static["goodput_kbps"],
        "robot_delivery_pct": robot["delivery_pct"],
        "robot_goodput_kbps": robot["goodput_kbps"],
        "mean_loss_db": mean([float(row["Loss_dB"]) for row in propagation]),
        "mean_array_gain_db": mean(
            [float(row["AvgEffectiveChannelGain_dB"]) for row in gains]
        ),
        "detections": len(detections),
        "active_beams": len(beams),
        "robot_motion_distance_m": robot_motion_distance_m(run_dir, robot_ue_start),
        "ideal_analog_array_gain": read_summary_value(summary_path, "ideal_analog_array_gain"),
        "beamforming_periodicity_s": read_summary_value(summary_path, "beamforming_periodicity_s"),
        "challenge_start_s": read_summary_value(summary_path, "radio_challenge_start_s"),
        "robot_route_start_s": read_summary_value(summary_path, "robot_route_start_s"),
        "robot_drop_start_s": read_summary_value(summary_path, "robot_drop_start_s"),
        "warehouse_workflow_enabled": read_summary_value(
            summary_path, "warehouse_workflow_enabled"
        ),
        "wall_clock_s": read_summary_value(summary_path, "simulator_run_wall_clock_s"),
    }


def nondecreasing(values: list[float], tolerance: float = 1e-9) -> bool:
    return all(right + tolerance >= left for left, right in zip(values, values[1:]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results_dir", type=Path)
    args = parser.parse_args()

    expected = [(mode, size) for mode in ("no-isac", "isac") for size in (2, 4, 8)]
    rows = []
    missing = []
    for mode, size in expected:
        row = summarize_run(args.results_dir, mode, size)
        if row is None:
            missing.append(f"{mode}-{size}x{size}")
            continue
        rows.append(row)
    if not rows:
        print(f"no complete runs found in {args.results_dir}", flush=True)
        if missing:
            print("missing: " + ", ".join(missing), flush=True)
        return 1
    output = args.results_dir / "sweep_summary.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print(
        "mode     array  delivery   goodput Kbps  robot del  robot Kbps"
        "  robot m  loss dB  array gain dB  detections  beams  wall s"
    )
    for row in rows:
        print(
            f"{row['mode']:8s} {row['array']:>4s}  {row['delivery_pct']:8.2f}%"
            f"  {row['goodput_kbps']:12.2f}  {row['robot_delivery_pct']:8.2f}%"
            f"  {row['robot_goodput_kbps']:10.2f}  {row['robot_motion_distance_m']:7.1f}"
            f"  {row['mean_loss_db']:7.2f}"
            f"  {row['mean_array_gain_db']:13.2f}  {row['detections']:10d}"
            f"  {row['active_beams']:5d}"
            f"  {row['wall_clock_s']:6.1f}"
        )

    if missing:
        print("missing: " + ", ".join(missing))

    for mode in ("no-isac", "isac"):
        selected = [row for row in rows if row["mode"] == mode]
        if len(selected) != 3:
            print(f"{mode}: monotonic checks skipped; incomplete array set")
            continue
        gains = [float(row["mean_array_gain_db"]) for row in selected]
        static_goodput = [float(row["static_goodput_kbps"]) for row in selected]
        print(f"{mode}: monotonic array gain = {nondecreasing(gains)}")
        print(f"{mode}: monotonic static goodput = {nondecreasing(static_goodput)}")
    print(f"summary: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
