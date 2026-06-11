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


def run_succeeded(run_dir: Path) -> bool:
    log_path = run_dir / "run.log"
    if not log_path.exists():
        return False
    log = log_path.read_text(errors="replace")
    return (
        "ERROR: Warehouse MQTT verification failed." not in log
        and "returned non-zero exit status" not in log
    )


def challenge_flow(row: dict[str, str]) -> bool:
    port = int(row["DstPort"])
    # ISAC tracks mobile robots, so the primary comparison uses only their DL
    # mission payloads. Camera UL is reported separately because the all-flex
    # TDD scheduler can starve it while DL mission queues are active.
    return 20001 <= port <= 20003


def port_class(row: dict[str, str]) -> str:
    port = int(row["DstPort"])
    if 20001 <= port <= 20003:
        return "robot"   # mobile robot DL showcase payload
    if port == 10000:
        return "video"   # fixed camera video UL diagnostic
    return "other"


def summarize_flows(flows: list[dict[str, str]]) -> dict[str, float]:
    tx_packets = sum(int(row["TxPackets"]) for row in flows)
    rx_packets = sum(int(row["RxPackets"]) for row in flows)
    confirmed_loss_available = all("UnreceivedPackets" in row for row in flows)
    lost_packets = (
        sum(int(row.get("LostPackets", 0)) for row in flows)
        if confirmed_loss_available
        else 0
    )
    tx_bytes = sum(int(row["TxBytes"]) for row in flows)
    rx_bytes = sum(int(row["RxBytes"]) for row in flows)
    sim_time = max(
        (
            float(row["RxBytes"]) * 8.0
            / (float(row["SimGoodput_Kbps"]) * 1024.0)
            for row in flows
            if float(row["SimGoodput_Kbps"]) > 0.0
        ),
        default=0.0,
    )
    return {
        "flows": len(flows),
        "confirmed_loss_available": confirmed_loss_available,
        "delivery_pct": 100.0 * rx_packets / tx_packets if tx_packets else 0.0,
        "loss_pct": 100.0 * lost_packets / tx_packets if tx_packets else 0.0,
        "unreceived_pct": 100.0 * (tx_packets - rx_packets) / tx_packets
        if tx_packets
        else 0.0,
        "offered_kbps": tx_bytes * 8.0 / sim_time / 1024.0 if sim_time else 0.0,
        "goodput_kbps": rx_bytes * 8.0 / sim_time / 1024.0 if sim_time else 0.0,
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
    all_flows = read_csv(flow_path)
    video_flows = [row for row in all_flows if port_class(row) == "video"]
    robot_flows = [row for row in flows if port_class(row) == "robot"]
    aggregate = summarize_flows(flows)
    video = summarize_flows(video_flows)
    robot = summarize_flows(robot_flows)
    propagation = read_csv(run_dir / "propagation_stats.csv")
    gains = read_csv(run_dir / "mimo_channel_gain_stats.csv")
    detections = [
        row
        for row in read_csv(run_dir / "sensing_stats.csv")
        if row.get("DetectionIndex", "").strip()
    ]
    detection_powers = [
        float(row["Power_W"])
        for row in detections
        if row.get("Power_W", "").strip() and float(row.get("Power_W", "0") or "0") > 0
    ]
    mean_detection_power_w = mean(detection_powers) if detection_powers else 0.0
    mean_detection_power_dbw = (
        10 * __import__("math").log10(mean_detection_power_w)
        if mean_detection_power_w > 0 else float("-inf")
    )
    beams = [
        row
        for row in read_csv(run_dir / "isac_beam_stats.csv")
        if csv_bool(row.get("Active", ""))
    ]
    robot_ue_start = int(read_summary_value(summary_path, "mobile_robot_ue_start_index") or 3)
    mean_pathloss_db = mean([float(row["Pathloss_dB"]) for row in propagation])
    mean_array_gain_db = mean(
        [float(row["AvgEffectiveChannelGain_dB"]) for row in gains]
    )

    return {
        "mode": mode,
        "array": f"{size}x{size}",
        "run_succeeded": run_succeeded(run_dir),
        "confirmed_loss_available": aggregate["confirmed_loss_available"],
        "challenge_flows": aggregate["flows"],
        "delivery_pct": aggregate["delivery_pct"],
        "loss_pct": aggregate["loss_pct"],
        "unreceived_pct": aggregate["unreceived_pct"],
        "offered_kbps": aggregate["offered_kbps"],
        "goodput_kbps": aggregate["goodput_kbps"],
        "video_delivery_pct": video["delivery_pct"],
        "video_loss_pct": video["loss_pct"],
        "video_goodput_kbps": video["goodput_kbps"],
        "robot_delivery_pct": robot["delivery_pct"],
        "robot_loss_pct": robot["loss_pct"],
        "robot_goodput_kbps": robot["goodput_kbps"],
        "mean_pathloss_db": mean_pathloss_db,
        "mean_array_gain_db": mean_array_gain_db,
        "mean_net_link_loss_db": mean_pathloss_db - mean_array_gain_db,
        "detections": len(detections),
        "mean_detection_power_w": mean_detection_power_w,
        "mean_detection_power_dbw": mean_detection_power_dbw,
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
        "mode     array valid  delivery  confirmed/unrecv loss   goodput Kbps  robot del"
        "  pathloss  array gain  net loss  detections  det power dBW  wall s"
    )
    for row in rows:
        det_pwr = row["mean_detection_power_dbw"]
        det_pwr_str = f"{det_pwr:13.1f}" if det_pwr != float("-inf") else "          n/a"
        print(
            f"{row['mode']:8s} {row['array']:>4s}  {str(row['run_succeeded']):>5s}"
            f"  {row['delivery_pct']:7.2f}%"
            f"  {row['loss_pct']:5.2f}/{row['unreceived_pct']:5.2f}%"
            f"{'' if row['confirmed_loss_available'] else '*'}"
            f"  {row['goodput_kbps']:12.2f}  {row['robot_delivery_pct']:8.2f}%"
            f"  {row['mean_pathloss_db']:8.2f}"
            f"  {row['mean_array_gain_db']:10.2f}"
            f"  {row['mean_net_link_loss_db']:8.2f}  {row['detections']:10d}"
            f"  {det_pwr_str}  {row['wall_clock_s']:6.1f}"
        )

    if missing:
        print("missing: " + ", ".join(missing))
    if any(not row["confirmed_loss_available"] for row in rows):
        print("* legacy flow_stats.csv: confirmed FlowMonitor loss is unavailable; "
              "the old LostPackets column was unreceived packets")

    for mode in ("no-isac", "isac"):
        selected = [row for row in rows if row["mode"] == mode]
        if len(selected) != 3:
            print(f"{mode}: monotonic checks skipped; incomplete array set")
            continue
        gains = [float(row["mean_array_gain_db"]) for row in selected]
        print(f"{mode}: monotonic array gain = {nondecreasing(gains)}")
    print(f"summary: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
