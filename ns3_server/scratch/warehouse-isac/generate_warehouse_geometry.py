#!/usr/bin/env python3
"""Generate a dimensioned warehouse floor-plan proposal for the ISAC scenario."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Patch, Rectangle


@dataclass(frozen=True)
class RectangleSpec:
    name: str
    x_m: float
    y_m: float
    width_m: float
    depth_m: float
    height_m: float
    material: str


@dataclass(frozen=True)
class PointSpec:
    name: str
    x_m: float
    y_m: float
    z_m: float


WAREHOUSE_WIDTH_M = 48.0
WAREHOUSE_DEPTH_M = 32.0
WAREHOUSE_HEIGHT_M = 7.0

GNB = PointSpec("gnb", 3.0, 3.0, 5.0)
DROPOFF = RectangleSpec("dropoff_zone", 40.0, 1.0, 7.0, 5.0, 0.0, "floor_marking")
STAGING = RectangleSpec("package_staging_zone", 40.0, 24.0, 7.0, 7.0, 0.0, "floor_marking")
ROBOT_SPAWN = RectangleSpec("reserved_robot_spawn_zone", 1.0, 24.0, 6.0, 7.0, 0.0, "floor_marking")

RACKS = [
    RectangleSpec(f"rack_{index + 1}", x_m, 8.0, 2.0, 18.0, 3.2, "metal")
    for index, x_m in enumerate((10.0, 15.0, 20.0, 25.0, 30.0, 35.0))
]

OBSTACLES = [
    RectangleSpec("packing_partition", 39.0, 20.0, 0.3, 12.0, 3.0, "concrete"),
    RectangleSpec("dropoff_safety_barrier", 39.0, 0.0, 0.3, 7.0, 1.2, "metal"),
    RectangleSpec("utility_room", 1.0, 8.0, 6.0, 8.0, 3.0, "concrete"),
]

PACKAGE_SENSORS = [
    PointSpec("package_sensor_1", 41.5, 26.0, 1.5),
    PointSpec("package_sensor_2", 43.5, 26.0, 1.5),
    PointSpec("package_sensor_3", 45.5, 26.0, 1.5),
]

# These fixed sensors provide realistic radio-test candidates before robots are added.
# Their links cross multiple metal rack rows from the southwest gNB.
RACK_SENSORS = [
    PointSpec("rack_sensor_1", 36.4, 10.5, 1.5),
    PointSpec("rack_sensor_2", 36.4, 16.5, 1.5),
    PointSpec("rack_sensor_3", 36.4, 22.5, 1.5),
]


def add_dimension(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    label: str,
    text_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="|-|",
        mutation_scale=8,
        linewidth=1.0,
        color="#333333",
        clip_on=False,
    )
    ax.add_patch(arrow)
    x_mid = (start[0] + end[0]) / 2 + text_offset[0]
    y_mid = (start[1] + end[1]) / 2 + text_offset[1]
    ax.text(x_mid, y_mid, label, ha="center", va="center", fontsize=8, color="#222222")


def add_rectangle(ax: plt.Axes, spec: RectangleSpec, color: str, alpha: float = 0.75) -> None:
    ax.add_patch(
        Rectangle(
            (spec.x_m, spec.y_m),
            spec.width_m,
            spec.depth_m,
            facecolor=color,
            edgecolor="#333333",
            linewidth=1.0,
            alpha=alpha,
        )
    )


def draw_floor_plan(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 10))
    ax.set_aspect("equal")
    ax.set_xlim(-4.5, WAREHOUSE_WIDTH_M + 5.0)
    ax.set_ylim(-4.0, WAREHOUSE_DEPTH_M + 4.5)
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title("Proposed 30 dBm Warehouse ISAC Geometry (top-down XY plan)", fontsize=15)
    ax.grid(True, color="#d8d8d8", linewidth=0.6, zorder=0)

    ax.add_patch(
        Rectangle(
            (0.0, 0.0),
            WAREHOUSE_WIDTH_M,
            WAREHOUSE_DEPTH_M,
            facecolor="#f7f5ef",
            edgecolor="#111111",
            linewidth=2.0,
        )
    )

    add_rectangle(ax, DROPOFF, "#81c784", 0.55)
    add_rectangle(ax, STAGING, "#ffe082", 0.60)
    add_rectangle(ax, ROBOT_SPAWN, "#90caf9", 0.55)
    for obstacle in OBSTACLES:
        add_rectangle(ax, obstacle, "#8d8d8d" if obstacle.material == "concrete" else "#546e7a")
    for rack in RACKS:
        add_rectangle(ax, rack, "#78909c")

    for rack in RACKS:
        ax.text(
            rack.x_m + rack.width_m / 2,
            rack.y_m + rack.depth_m / 2,
            rack.name.replace("_", " ").title(),
            ha="center",
            va="center",
            rotation=90,
            fontsize=8,
            color="white",
        )

    ax.text(43.5, 3.5, "DROPOFF\n7 x 5 m", ha="center", va="center", fontsize=9)
    ax.text(43.5, 29.0, "PACKAGE STAGING\n7 x 7 m", ha="center", va="center", fontsize=9)
    ax.text(4.0, 28.5, "RESERVED ROBOT\nSPAWN ZONE\n6 x 7 m", ha="center", va="center", fontsize=8)
    ax.text(4.0, 12.0, "UTILITY ROOM\n6 x 8 m", ha="center", va="center", fontsize=8, color="white")

    # Draw the reserved longitudinal robot aisles between racks.
    for x_center in (8.5, 13.5, 18.5, 23.5, 28.5, 33.5, 38.0):
        ax.plot([x_center, x_center], [7.0, 27.0], linestyle="--", linewidth=0.8, color="#5c6bc0")
    ax.text(23.5, 27.2, "reserved 3 m robot aisles", ha="center", fontsize=8, color="#3949ab")

    ax.scatter(GNB.x_m, GNB.y_m, marker="^", s=180, color="#d32f2f", edgecolor="white", zorder=5)
    ax.text(GNB.x_m + 0.6, GNB.y_m - 0.6, "gNB (3, 3, 5 m)", fontsize=8, color="#b71c1c")

    for index, sensor in enumerate(PACKAGE_SENSORS, start=1):
        ax.scatter(sensor.x_m, sensor.y_m, marker="P", s=90, color="#ef6c00", edgecolor="white", zorder=5)
        ax.text(sensor.x_m, sensor.y_m + 0.7, f"P{index}", ha="center", fontsize=8)

    for sensor in RACK_SENSORS:
        ax.scatter(sensor.x_m, sensor.y_m, marker="s", s=70, color="#1565c0", edgecolor="white", zorder=5)
        ax.text(sensor.x_m + 0.7, sensor.y_m, sensor.name.replace("_", " "), va="center", fontsize=7)

    # Show why the fixed radio-test sensors are useful for array-size comparisons.
    for sensor in RACK_SENSORS:
        ax.plot([GNB.x_m, sensor.x_m], [GNB.y_m, sensor.y_m], color="#d32f2f", linestyle=":", linewidth=0.8)
    ax.text(20.0, 4.7, "candidate NLOS radio-test links cross metal racks", fontsize=8, color="#b71c1c")

    add_dimension(ax, (0.0, -2.0), (WAREHOUSE_WIDTH_M, -2.0), "48 m")
    add_dimension(ax, (-2.0, 0.0), (-2.0, WAREHOUSE_DEPTH_M), "32 m", (-0.9, 0.0))
    add_dimension(ax, (10.0, 6.8), (12.0, 6.8), "rack width 2 m", (0.0, -0.45))
    add_dimension(ax, (12.0, 5.8), (15.0, 5.8), "aisle 3 m", (0.0, -0.45))
    add_dimension(ax, (8.8, 8.0), (8.8, 26.0), "rack length 18 m", (-1.25, 0.0))

    handles = [
        Line2D([0], [0], marker="^", color="none", markerfacecolor="#d32f2f", markersize=10, label="gNB"),
        Patch(facecolor="#78909c", edgecolor="#333333", label="metal rack: 2 x 18 x 3.2 m"),
        Patch(facecolor="#8d8d8d", edgecolor="#333333", label="concrete obstacle"),
        Line2D([0], [0], marker="P", color="none", markerfacecolor="#ef6c00", markersize=8, label="package sensor"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#1565c0", markersize=8, label="candidate fixed rack sensor"),
        Patch(facecolor="#81c784", edgecolor="#333333", alpha=0.55, label="dropoff zone"),
        Line2D([0], [0], linestyle="--", color="#5c6bc0", label="reserved robot aisle"),
    ]
    ax.legend(handles=handles, loc="lower left", fontsize=8, framealpha=0.95)
    ax.text(
        9.0,
        33.2,
        "Room height: 7 m | gNB: wall-mounted at z=5 m | sensors: z=1.5 m | robots intentionally omitted",
        fontsize=9,
    )

    fig.tight_layout()
    fig.savefig(output_dir / "warehouse_geometry_proposal.png", dpi=180)
    fig.savefig(output_dir / "warehouse_geometry_proposal.svg")
    plt.close(fig)


def write_geometry_json(output_dir: Path) -> None:
    geometry = {
        "units": "meters",
        "warehouse": {
            "width_m": WAREHOUSE_WIDTH_M,
            "depth_m": WAREHOUSE_DEPTH_M,
            "height_m": WAREHOUSE_HEIGHT_M,
        },
        "gnb": asdict(GNB),
        "racks": [asdict(rack) for rack in RACKS],
        "obstacles": [asdict(obstacle) for obstacle in OBSTACLES],
        "zones": {
            "dropoff": asdict(DROPOFF),
            "package_staging": asdict(STAGING),
            "reserved_robot_spawn": asdict(ROBOT_SPAWN),
        },
        "package_sensors": [asdict(sensor) for sensor in PACKAGE_SENSORS],
        "candidate_fixed_rack_sensors": [asdict(sensor) for sensor in RACK_SENSORS],
        "reserved_robot_aisle_width_m": 3.0,
        "design_notes": [
            "Robots are intentionally omitted and can be added in the reserved aisles later.",
            "Candidate fixed rack sensors sit behind multiple metal rows relative to the southwest gNB.",
            "Validate Sionna path loss and CQI at 30 dBm before selecting final radio-test positions.",
        ],
    }
    with (output_dir / "warehouse_geometry_proposal.json").open("w", encoding="utf-8") as output:
        json.dump(geometry, output, indent=2)
        output.write("\n")


def main() -> None:
    default_output = Path(__file__).resolve().parent / "generated"
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=default_output)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw_floor_plan(args.output_dir)
    write_geometry_json(args.output_dir)
    print(f"Generated warehouse geometry in {args.output_dir}")


if __name__ == "__main__":
    main()
