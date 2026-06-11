#!/usr/bin/env python3
"""Generate a rack-blocked warehouse layout for the ISAC array sweep."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


BOUNDS = {"x_min": -24.0, "x_max": 24.0, "y_min": -19.0, "y_max": 19.0}
FREQUENCY_HZ = 15e9
BANDWIDTH_HZ = 190.08e6
GNB_TX_POWER_DBM = 30.0
UE_NOISE_FIGURE_DB = 17.0

# These footprints match the warehouse_v4 comments in warehouse-scenario.cc.
RACKS = [
    {"name": "rack_A", "x_min": -4.9, "x_max": -2.8, "y_min": -3.8, "y_max": 14.2},
    {"name": "rack_B", "x_min": 4.7, "x_max": 6.8, "y_min": -3.8, "y_max": 14.2},
    {"name": "rack_C", "x_min": 13.3, "x_max": 15.4, "y_min": -3.8, "y_max": 14.2},
    {"name": "rack_D", "x_min": 22.0, "x_max": 24.0, "y_min": -3.8, "y_max": 14.2},
]

GNB = {"name": "gNB", "role": "gnb", "position_m": [-22.0, -17.0, 6.0]}
GNB_LOOK_AT = [8.0, 9.0, 1.5]

UES = [
    {
        "name": "rack_sensor_1",
        "role": "fixed_rack_sensor",
        "position_m": [-1.8, 13.0, 1.5],
        "radio_class": "far_nlos",
    },
    {
        "name": "rack_sensor_2",
        "role": "fixed_rack_sensor",
        "position_m": [7.8, 13.0, 1.5],
        "radio_class": "far_nlos",
    },
    {
        "name": "rack_sensor_3",
        "role": "fixed_rack_sensor",
        "position_m": [12.3, 13.0, 1.5],
        "radio_class": "far_nlos",
    },
    {
        "name": "rack_sensor_4",
        "role": "fixed_rack_sensor",
        "position_m": [21.0, 13.0, 1.5],
        "radio_class": "far_nlos",
    },
    {
        "name": "robot_1",
        "role": "mobile_robot",
        "position_m": [-18.0, -13.0, 1.5],
        "radio_class": "mobile_transition",
    },
    {
        "name": "robot_2",
        "role": "mobile_robot",
        "position_m": [-12.0, -13.0, 1.5],
        "radio_class": "mobile_transition",
    },
    {
        "name": "robot_3",
        "role": "mobile_robot",
        "position_m": [-6.0, -13.0, 1.5],
        "radio_class": "mobile_transition",
    },
    {
        "name": "camera_1",
        "role": "fixed_camera",
        "position_m": [20.0, 16.0, 4.0],
        "radio_class": "far_nlos_high_load",
    },
]

PACKAGE_SENSORS = [
    {"name": "package_sensor_1", "position_m": [19.0, -15.5, 1.2]},
    {"name": "package_sensor_2", "position_m": [9.0, -15.5, 1.2]},
]

DROP_ZONE = [22.0, -8.0, 1.5]
ROUTES = [
    {
        "name": "robot_1_route",
        "waypoints_m": [
            [-18.0, -13.0, 1.5],
            [19.0, -15.5, 1.5],
            [7.8, 16.0, 1.5],
            DROP_ZONE,
        ],
    },
    {
        "name": "robot_2_route",
        "waypoints_m": [
            [-12.0, -13.0, 1.5],
            [9.0, -15.5, 1.5],
            [12.3, 16.0, 1.5],
            DROP_ZONE,
        ],
    },
    {
        "name": "robot_3_route",
        "waypoints_m": [
            [-6.0, -13.0, 1.5],
            [19.0, -15.5, 1.5],
            [-1.8, 16.0, 1.5],
            DROP_ZONE,
        ],
    },
]


def distance_3d(left: list[float], right: list[float]) -> float:
    return math.dist(left, right)


def free_space_path_loss_db(distance_m: float) -> float:
    wavelength_m = 299_792_458.0 / FREQUENCY_HZ
    return 20.0 * math.log10(4.0 * math.pi * distance_m / wavelength_m)


def thermal_noise_dbm() -> float:
    return -174.0 + 10.0 * math.log10(BANDWIDTH_HZ) + UE_NOISE_FIGURE_DB


def build_design() -> dict[str, object]:
    gnb_position = GNB["position_m"]
    nodes = [GNB]
    for ue in UES:
        position = ue["position_m"]
        distance = distance_3d(gnb_position, position)
        node = dict(ue)
        node["distance_to_gnb_m"] = round(distance, 2)
        node["free_space_path_loss_db"] = round(free_space_path_loss_db(distance), 2)
        nodes.append(node)

    return {
        "name": "warehouse_diagonal_nlos",
        "purpose": (
            "Keep 30 dBm gNB power while moving challenge UEs out of CQI/MCS "
            "saturation and forcing mobile links through rack-edge transitions."
        ),
        "scene": {
            "source": "assets/scenes/warehouse/warehouse_v4.xml",
            "usable_size_m": [48.0, 38.0],
            "bounds_m": BOUNDS,
            "racks": RACKS,
        },
        "radio": {
            "carrier_frequency_hz": FREQUENCY_HZ,
            "bandwidth_hz": BANDWIDTH_HZ,
            "gnb_tx_power_dbm": GNB_TX_POWER_DBM,
            "ue_noise_figure_db": UE_NOISE_FIGURE_DB,
            "receiver_noise_floor_dbm": round(thermal_noise_dbm(), 2),
            "measured_mean_array_gain_db": {
                "2x2": 1.93,
                "4x4": 5.38,
                "8x8": 11.55,
            },
            "calibration_target": {
                "challenge_path_loss_db": [105.0, 115.0],
                "challenge_net_link_loss_db": [95.0, 110.0],
                "two_by_two_estimated_sinr_db": [-9.0, 1.0],
                "eight_by_eight_estimated_sinr_db": [0.0, 10.0],
                "note": (
                    "These are ray-tracing calibration targets, not assumed "
                    "losses. Add or move blockers if measured path loss is below "
                    "105 dB; relax one blocker if it exceeds 115 dB."
                ),
            },
        },
        "gnb_look_at_m": GNB_LOOK_AT,
        "nodes": nodes,
        "package_sensors": PACKAGE_SENSORS,
        "drop_zone_m": DROP_ZONE,
        "robot_routes": ROUTES,
        "experiment_requirements": [
            "Use identical geometry, seed, mobility, offered load, and baseline beam period.",
            "Use sustained UDP traffic; 500 KB mission bursts do not load a 190 MHz link.",
            "Keep at least one moderate control UE and report results per UE, not only averages.",
            "Tune geometry until 2x2 CQI is mostly 4-10 and 8x8 CQI is mostly 9-14.",
            "For ISAC gain, route robots around rack ends so azimuth and blockage change abruptly.",
        ],
    }


def draw_design(output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.add_patch(
        Rectangle(
            (BOUNDS["x_min"], BOUNDS["y_min"]),
            BOUNDS["x_max"] - BOUNDS["x_min"],
            BOUNDS["y_max"] - BOUNDS["y_min"],
            facecolor="#f7f7f7",
            edgecolor="#222222",
            linewidth=2,
        )
    )

    for rack in RACKS:
        ax.add_patch(
            Rectangle(
                (rack["x_min"], rack["y_min"]),
                rack["x_max"] - rack["x_min"],
                rack["y_max"] - rack["y_min"],
                facecolor="#737373",
                edgecolor="#333333",
                alpha=0.85,
            )
        )
        ax.text(
            (rack["x_min"] + rack["x_max"]) / 2,
            rack["y_max"] - 0.8,
            rack["name"],
            color="white",
            ha="center",
            va="top",
            fontsize=9,
            rotation=90,
        )

    colors = ["#0072b2", "#d55e00", "#009e73"]
    for route, color in zip(ROUTES, colors):
        xs = [point[0] for point in route["waypoints_m"]]
        ys = [point[1] for point in route["waypoints_m"]]
        ax.plot(xs, ys, color=color, linewidth=2, linestyle="--", alpha=0.8)

    gx, gy, _ = GNB["position_m"]
    ax.scatter([gx], [gy], marker="^", s=220, color="#cc0000", zorder=5)
    ax.annotate("gNB 30 dBm\n(-22, -17, 6)", (gx, gy), xytext=(8, 8),
                textcoords="offset points", fontsize=10, weight="bold")

    role_style = {
        "fixed_rack_sensor": ("s", "#6a3d9a"),
        "mobile_robot": ("o", "#ff7f00"),
        "fixed_camera": ("D", "#1b9e77"),
    }
    for ue in UES:
        x, y, _ = ue["position_m"]
        marker, color = role_style[ue["role"]]
        ax.scatter([x], [y], marker=marker, s=90, color=color, zorder=6)
        ax.annotate(ue["name"], (x, y), xytext=(5, 5),
                    textcoords="offset points", fontsize=8)

    for sensor in PACKAGE_SENSORS:
        x, y, _ = sensor["position_m"]
        ax.scatter([x], [y], marker="P", s=100, color="#e6ab02", zorder=6)
        ax.annotate(sensor["name"], (x, y), xytext=(5, -12),
                    textcoords="offset points", fontsize=8)

    ax.scatter([DROP_ZONE[0]], [DROP_ZONE[1]], marker="X", s=130,
               color="#000000", zorder=6)
    ax.annotate("drop_zone", (DROP_ZONE[0], DROP_ZONE[1]), xytext=(-58, 8),
                textcoords="offset points", fontsize=8)

    ax.set_title("Proposed diagonal-NLOS warehouse geometry at 15 GHz")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(BOUNDS["x_min"] - 1, BOUNDS["x_max"] + 1)
    ax.set_ylim(BOUNDS["y_min"] - 1, BOUNDS["y_max"] + 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.4, alpha=0.4)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "generated",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    design = build_design()
    json_path = args.output_dir / "warehouse_diagonal_nlos.json"
    json_path.write_text(json.dumps(design, indent=2) + "\n")
    draw_design(args.output_dir / "warehouse_diagonal_nlos.png")
    draw_design(args.output_dir / "warehouse_diagonal_nlos.svg")
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
