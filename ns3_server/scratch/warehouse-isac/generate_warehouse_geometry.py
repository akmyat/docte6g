#!/usr/bin/env python3
"""Generate a validated, dimensioned warehouse floor plan for the ISAC scenario."""

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


WAREHOUSE_WIDTH_M = 60.0
WAREHOUSE_DEPTH_M = 40.0
WAREHOUSE_HEIGHT_M = 8.0
MIN_OBJECT_CLEARANCE_M = 0.5

GNB = PointSpec("gnb", 2.0, 4.0, 6.0)

# Floor-marked operational areas. They are separated from storage by a
# partition and deliberately do not overlap one another.
DROPOFF = RectangleSpec("dropoff_zone", 50.0, 2.0, 8.0, 6.0, 0.0, "floor_marking")
PACKING = RectangleSpec("packing_zone", 50.0, 13.0, 8.0, 8.0, 0.0, "floor_marking")
STAGING = RectangleSpec("package_staging_zone", 50.0, 26.0, 8.0, 10.0, 0.0, "floor_marking")
ROBOT_SPAWN = RectangleSpec("reserved_robot_spawn_zone", 2.0, 32.0, 7.0, 6.0, 0.0, "floor_marking")
ZONES = [DROPOFF, PACKING, STAGING, ROBOT_SPAWN]

# Long metal rack rows with realistic 3.5 m forklift/AMR aisles. Front and rear
# cross-aisles are left open at y=0..9 m and y=31..40 m.
RACK_X_POSITIONS = (12.0, 17.5, 23.0, 28.5, 34.0, 39.5)
RACKS = [
    RectangleSpec(f"rack_{index + 1}", x_m, 10.0, 2.0, 20.0, 4.0, "metal")
    for index, x_m in enumerate(RACK_X_POSITIONS)
]

# Physical obstacles are kept outside all rack footprints and operational zones.
OBSTACLES = [
    RectangleSpec("utility_room", 2.0, 12.0, 7.0, 9.0, 3.5, "concrete"),
    RectangleSpec("operations_partition", 47.0, 10.0, 0.4, 28.0, 3.5, "concrete"),
    RectangleSpec("dropoff_safety_barrier", 47.0, 1.0, 0.4, 7.0, 1.2, "metal"),
]

PACKAGE_SENSORS = [
    PointSpec("package_sensor_1", 52.0, 28.0, 1.5),
    PointSpec("package_sensor_2", 54.0, 28.0, 1.5),
    PointSpec("package_sensor_3", 56.0, 28.0, 1.5),
]

# Fixed radio-test sensors sit in the east service aisle, not inside a rack.
# Their links cross multiple rack rows from the southwest wall-mounted gNB.
RACK_SENSORS = [
    PointSpec("rack_sensor_1", 44.0, 12.5, 1.5),
    PointSpec("rack_sensor_2", 44.0, 20.0, 1.5),
    PointSpec("rack_sensor_3", 44.0, 27.5, 1.5),
]


def rectangles_overlap(a: RectangleSpec, b: RectangleSpec, clearance_m: float = 0.0) -> bool:
    return not (
        a.x_m + a.width_m + clearance_m <= b.x_m
        or b.x_m + b.width_m + clearance_m <= a.x_m
        or a.y_m + a.depth_m + clearance_m <= b.y_m
        or b.y_m + b.depth_m + clearance_m <= a.y_m
    )


def point_inside_rectangle(point: PointSpec, rectangle: RectangleSpec) -> bool:
    return (
        rectangle.x_m <= point.x_m <= rectangle.x_m + rectangle.width_m
        and rectangle.y_m <= point.y_m <= rectangle.y_m + rectangle.depth_m
    )


def validate_geometry() -> None:
    physical_objects = [*RACKS, *OBSTACLES]
    for rectangle in [*physical_objects, *ZONES]:
        if rectangle.x_m < 0.0 or rectangle.y_m < 0.0:
            raise ValueError(f"{rectangle.name} starts outside the warehouse")
        if rectangle.x_m + rectangle.width_m > WAREHOUSE_WIDTH_M:
            raise ValueError(f"{rectangle.name} exceeds the warehouse width")
        if rectangle.y_m + rectangle.depth_m > WAREHOUSE_DEPTH_M:
            raise ValueError(f"{rectangle.name} exceeds the warehouse depth")

    for index, first in enumerate(physical_objects):
        for second in physical_objects[index + 1 :]:
            if rectangles_overlap(first, second, MIN_OBJECT_CLEARANCE_M):
                raise ValueError(
                    f"physical objects overlap or violate {MIN_OBJECT_CLEARANCE_M} m clearance: "
                    f"{first.name}, {second.name}"
                )

    for index, first in enumerate(ZONES):
        for second in ZONES[index + 1 :]:
            if rectangles_overlap(first, second):
                raise ValueError(f"operational zones overlap: {first.name}, {second.name}")

    for sensor in RACK_SENSORS:
        if any(point_inside_rectangle(sensor, rectangle) for rectangle in physical_objects):
            raise ValueError(f"{sensor.name} is inside a physical object")
    for sensor in PACKAGE_SENSORS:
        if not point_inside_rectangle(sensor, STAGING):
            raise ValueError(f"{sensor.name} must stay inside the package staging zone")


def add_dimension(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    label: str,
    text_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="|-|",
            mutation_scale=8,
            linewidth=1.0,
            color="#333333",
            clip_on=False,
        )
    )
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
    fig, ax = plt.subplots(figsize=(17, 11))
    ax.set_aspect("equal")
    ax.set_xlim(-5.0, WAREHOUSE_WIDTH_M + 5.0)
    ax.set_ylim(-4.0, WAREHOUSE_DEPTH_M + 5.0)
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title("Validated 30 dBm Warehouse ISAC Geometry (top-down XY plan)", fontsize=15)
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

    zone_colors = {
        "dropoff_zone": "#81c784",
        "packing_zone": "#ffcc80",
        "package_staging_zone": "#ffe082",
        "reserved_robot_spawn_zone": "#90caf9",
    }
    for zone in ZONES:
        add_rectangle(ax, zone, zone_colors[zone.name], 0.58)
    for obstacle in OBSTACLES:
        add_rectangle(ax, obstacle, "#8d8d8d" if obstacle.material == "concrete" else "#546e7a")
    for rack in RACKS:
        add_rectangle(ax, rack, "#78909c")
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

    ax.text(54.0, 5.0, "DROPOFF\n8 x 6 m", ha="center", va="center", fontsize=9)
    ax.text(54.0, 17.0, "PACKING\n8 x 8 m", ha="center", va="center", fontsize=9)
    ax.text(54.0, 33.2, "PACKAGE STAGING\n8 x 10 m", ha="center", va="center", fontsize=9)
    ax.text(5.5, 35.0, "RESERVED ROBOT\nSPAWN ZONE\n7 x 6 m", ha="center", va="center", fontsize=8)
    ax.text(5.5, 16.5, "UTILITY ROOM\n7 x 9 m", ha="center", va="center", fontsize=8, color="white")

    # Centerlines visualize the reserved rack aisles without occupying storage.
    aisle_centers = [(left + 2.0 + right) / 2 for left, right in zip(RACK_X_POSITIONS, RACK_X_POSITIONS[1:])]
    for x_center in aisle_centers:
        ax.plot([x_center, x_center], [9.0, 31.0], linestyle="--", linewidth=0.9, color="#5c6bc0")
    ax.text(27.0, 31.0, "rear cross-aisle: 9 m", ha="center", fontsize=8, color="#3949ab")
    ax.text(27.0, 8.2, "front cross-aisle: 10 m", ha="center", fontsize=8, color="#3949ab")
    ax.text(27.0, 30.4, "3.5 m rack aisles", ha="center", fontsize=8, color="#3949ab")

    ax.scatter(GNB.x_m, GNB.y_m, marker="^", s=180, color="#d32f2f", edgecolor="white", zorder=5)
    ax.text(GNB.x_m + 0.7, GNB.y_m - 0.6, "gNB (2, 4, 6 m)", fontsize=8, color="#b71c1c")
    for index, sensor in enumerate(PACKAGE_SENSORS, start=1):
        ax.scatter(sensor.x_m, sensor.y_m, marker="P", s=90, color="#ef6c00", edgecolor="white", zorder=5)
        ax.text(sensor.x_m, sensor.y_m + 0.8, f"P{index}", ha="center", fontsize=8)
    for sensor in RACK_SENSORS:
        ax.scatter(sensor.x_m, sensor.y_m, marker="s", s=70, color="#1565c0", edgecolor="white", zorder=5)
        ax.text(sensor.x_m + 0.8, sensor.y_m, sensor.name.replace("_", " "), va="center", fontsize=7)
        ax.plot([GNB.x_m, sensor.x_m], [GNB.y_m, sensor.y_m], color="#d32f2f", linestyle=":", linewidth=0.8)
    ax.text(22.0, 5.8, "candidate NLOS radio links cross multiple metal racks", fontsize=8, color="#b71c1c")

    add_dimension(ax, (0.0, -2.0), (WAREHOUSE_WIDTH_M, -2.0), "60 m")
    add_dimension(ax, (-2.0, 0.0), (-2.0, WAREHOUSE_DEPTH_M), "40 m", (-1.0, 0.0))
    add_dimension(ax, (12.0, 8.0), (14.0, 8.0), "rack width 2 m", (0.0, -0.5))
    add_dimension(ax, (14.0, 7.0), (17.5, 7.0), "aisle 3.5 m", (0.0, -0.5))
    add_dimension(ax, (10.7, 10.0), (10.7, 30.0), "rack length 20 m", (-1.4, 0.0))

    handles = [
        Line2D([0], [0], marker="^", color="none", markerfacecolor="#d32f2f", markersize=10, label="gNB"),
        Patch(facecolor="#78909c", edgecolor="#333333", label="metal rack: 2 x 20 x 4 m"),
        Patch(facecolor="#8d8d8d", edgecolor="#333333", label="concrete obstacle"),
        Line2D([0], [0], marker="P", color="none", markerfacecolor="#ef6c00", markersize=8, label="package sensor"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#1565c0", markersize=8, label="fixed rack sensor"),
        Patch(facecolor="#81c784", edgecolor="#333333", alpha=0.58, label="operational zone"),
        Line2D([0], [0], linestyle="--", color="#5c6bc0", label="reserved rack aisle"),
    ]
    ax.legend(handles=handles, loc="lower left", fontsize=8, framealpha=0.95)
    ax.text(
        12.0,
        41.5,
        "Room height: 8 m | gNB: wall-mounted at z=6 m | sensors: z=1.5 m | robots intentionally omitted",
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
        "minimum_physical_object_clearance_m": MIN_OBJECT_CLEARANCE_M,
        "gnb": asdict(GNB),
        "racks": [asdict(rack) for rack in RACKS],
        "obstacles": [asdict(obstacle) for obstacle in OBSTACLES],
        "zones": {zone.name: asdict(zone) for zone in ZONES},
        "package_sensors": [asdict(sensor) for sensor in PACKAGE_SENSORS],
        "candidate_fixed_rack_sensors": [asdict(sensor) for sensor in RACK_SENSORS],
        "rack_aisle_width_m": 3.5,
        "front_cross_aisle_width_m": 10.0,
        "rear_cross_aisle_width_m": 9.0,
        "geometry_validation": "passed",
        "design_notes": [
            "Physical objects are validated for overlap and a 0.5 m minimum clearance.",
            "Robots are intentionally omitted and can be added in the reserved aisles later.",
            "Fixed rack sensors sit in the east service aisle, outside rack footprints.",
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

    validate_geometry()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw_floor_plan(args.output_dir)
    write_geometry_json(args.output_dir)
    print(f"Generated validated warehouse geometry in {args.output_dir}")


if __name__ == "__main__":
    main()
