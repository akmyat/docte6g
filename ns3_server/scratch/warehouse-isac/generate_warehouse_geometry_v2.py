#!/usr/bin/env python3
"""
Warehouse ISAC geometry v2 — all contrib/warehouse applications deployed.

Applications (from contrib/warehouse/model/):
  WarehouseControllerApp    — MQTT broker/controller         → Control Room
  WarehouseCameraApp        — Robot camera streaming         → Robot nodes
  WarehouseRobotApp         — Robot mobility/task logic      → Robot nodes
  WarehouseTempSensorApp    — Temperature sensor             → Rack nodes
  WarehouseHumiditySensorApp— Humidity sensor                → Rack nodes
  WarehouseRackSensorApp    — Rack capacity/inventory        → Rack nodes
  WarehousePackageSensorApp — Package detection              → Dock + Staging
  WarehouseVideoClientApp   — Video stream consumer          → Staging + Control
  WarehouseWithdrawalApp    — Order / withdrawal manager     → Dock node
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, FancyArrowPatch, Rectangle

# ---------------------------------------------------------------------------
# Warehouse dimensions
# ---------------------------------------------------------------------------
W = 60.0   # width  (x)
D = 40.0   # depth  (y)
H =  8.0   # ceiling height (m) — gNB z = 6 m

GNB_X, GNB_Y, GNB_Z = 30.0, 36.5, 6.0   # ceiling-mounted, centre-north

# ---------------------------------------------------------------------------
# Rack geometry
# ---------------------------------------------------------------------------
RACK_W = 2.0
AISLE_W = 3.5   # longitudinal aisle between rack columns
X_STARTS = [6.0, 10.5, 15.0, 19.5, 30.5, 35.0, 39.5, 44.0]
RACK_S_Y0, RACK_S_Y1 = 6.0, 22.0   # south bank
RACK_N_Y0, RACK_N_Y1 = 26.0, 34.0  # north bank
RACK_H = 3.2

# ---------------------------------------------------------------------------
# Zone rectangles
# ---------------------------------------------------------------------------
DOCK_X,   DOCK_Y,   DOCK_W,   DOCK_D   = 54.0,  5.0, 6.0, 14.0
CHARGE_X, CHARGE_Y, CHARGE_W, CHARGE_D =  0.0,  5.0, 5.0, 12.0
STAGE_X,  STAGE_Y,  STAGE_W,  STAGE_D  = 54.0, 25.0, 6.0, 12.0
CTRL_X,   CTRL_Y,   CTRL_W,   CTRL_D   =  0.0, 29.0, 7.0, 11.0

# ---------------------------------------------------------------------------
# Beam footprints (illustrative) — sized to stay within 60×40 warehouse
# ---------------------------------------------------------------------------
BEAMS = [
    ("2×2", 14.0, 12.0, 0.10, "#1565c0"),
    ("4×4", 22.0, 20.0, 0.10, "#2e7d32"),
    ("8×8", 30.0, 34.0, 0.10, "#b71c1c"),
]

# ---------------------------------------------------------------------------
# App deployment positions — (x, y, label, app_type)
# ---------------------------------------------------------------------------

# WarehouseControllerApp — control room server
CONTROLLER = [(3.5, 35.0, "CTRL", "controller")]

# WarehouseRobotApp + WarehouseCameraApp — 4 robots in longitudinal aisles
# Robots traverse the aisles; positions here are mid-route snapshots
ROBOTS = [
    (8.25,  14.0, "R1", "robot"),   # aisle west of rack col 1 — deep NLOS
    (17.25, 10.0, "R2", "robot"),   # aisle between col 3/4    — deep NLOS
    (32.25, 18.0, "R3", "robot"),   # aisle between col 5/6    — shallow NLOS
    (41.25, 28.0, "R4", "robot"),   # aisle east of col 7, north bank — LOS
]

# WarehouseTempSensorApp + WarehouseHumiditySensorApp + WarehouseRackSensorApp
# One combined rack-sensor node per rack column, mid-south-bank
RACK_SENSORS = [
    (xs + RACK_W / 2, (RACK_S_Y0 + RACK_S_Y1) / 2, f"RS{i+1}", "rack_sensor")
    for i, xs in enumerate(X_STARTS)
]

# WarehousePackageSensorApp — loading dock + staging
PACKAGE_SENSORS = [
    (56.0, 10.0, "PS1", "package_sensor"),   # dock south
    (56.0, 16.0, "PS2", "package_sensor"),   # dock centre
    (56.0, 28.0, "PS3", "package_sensor"),   # staging south
    (56.0, 33.0, "PS4", "package_sensor"),   # staging north
]

# WarehouseVideoClientApp — staging area viewing stations
VIDEO_CLIENTS = [
    (56.0, 25.5, "VC1", "video_client"),
    (58.0, 31.0, "VC2", "video_client"),
]

# WarehouseWithdrawalApp — dock management terminal
WITHDRAWAL = [(57.0, 19.5, "WD", "withdrawal")]

ALL_NODES = CONTROLLER + ROBOTS + RACK_SENSORS + PACKAGE_SENSORS + VIDEO_CLIENTS + WITHDRAWAL

# ---------------------------------------------------------------------------
# Visual style per app type
# ---------------------------------------------------------------------------
APP_STYLE = {
    "controller":    dict(marker="s", color="#6a1b9a", size=130, label="WarehouseControllerApp (MQTT broker)"),
    "robot":         dict(marker="o", color="#1565c0", size=110, label="WarehouseRobotApp + CameraApp (robots)"),
    "rack_sensor":   dict(marker="^", color="#2e7d32", size=90,  label="TempSensor + HumiditySensor + RackSensorApp"),
    "package_sensor":dict(marker="D", color="#e65100", size=90,  label="WarehousePackageSensorApp (dock & staging)"),
    "video_client":  dict(marker="P", color="#00838f", size=100, label="WarehouseVideoClientApp (staging viewers)"),
    "withdrawal":    dict(marker="*", color="#c62828", size=150, label="WarehouseWithdrawalApp (dock terminal)"),
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def add_rect(ax, x, y, w, d, fc, ec="#333333", lw=1.0, alpha=0.85, zorder=2):
    ax.add_patch(Rectangle((x, y), w, d,
                            facecolor=fc, edgecolor=ec,
                            linewidth=lw, alpha=alpha, zorder=zorder))


def dim_arrow(ax, start, end, label, offset=(0.0, 0.0)):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="|-|",
        mutation_scale=7, linewidth=0.9,
        color="#444444", clip_on=False,
    ))
    mx = (start[0] + end[0]) / 2 + offset[0]
    my = (start[1] + end[1]) / 2 + offset[1]
    ax.text(mx, my, label, ha="center", va="center",
            fontsize=7.5, color="#222222",
            bbox=dict(facecolor="white", edgecolor="none", pad=1))


# ---------------------------------------------------------------------------
# Main drawing
# ---------------------------------------------------------------------------

def draw(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(20, 14))
    ax.set_aspect("equal")
    ax.set_xlim(-5, W + 2)
    ax.set_ylim(-5, D + 6)
    ax.set_xlabel("X position (m)", fontsize=10)
    ax.set_ylabel("Y position (m)", fontsize=10)
    ax.set_title(
        "Warehouse ISAC Geometry v2 — All Applications Deployed\n"
        "60 × 40 × 8 m  |  gNB ceiling-mounted at (30, 36.5, 6 m)  |  "
        "Beam footprints show 2×2 / 4×4 / 8×8 coverage reach",
        fontsize=12, pad=10,
    )
    ax.grid(True, color="#e8e8e8", linewidth=0.5, zorder=0)

    # Warehouse floor
    add_rect(ax, 0, 0, W, D, "#f5f3ed", ec="#111111", lw=2.0, alpha=1.0, zorder=1)

    # ---- Beam footprint ellipses ----------------------------------------
    for _lbl, rx, ry, alpha, color in BEAMS:
        ax.add_patch(Ellipse(
            (GNB_X, GNB_Y), width=2 * rx, height=2 * ry,
            facecolor=color, edgecolor=color,
            linewidth=0, alpha=alpha, zorder=1, clip_on=True,
        ))
    for _lbl, rx, ry, _a, color in BEAMS:
        ax.add_patch(Ellipse(
            (GNB_X, GNB_Y), width=2 * rx, height=2 * ry,
            facecolor="none", edgecolor=color,
            linewidth=1.6, linestyle="--", zorder=3, clip_on=True,
        ))

    # ---- Functional zones -----------------------------------------------
    add_rect(ax, DOCK_X,   DOCK_Y,   DOCK_W,   DOCK_D,   "#a5d6a7", alpha=0.55)
    add_rect(ax, CHARGE_X, CHARGE_Y, CHARGE_W, CHARGE_D, "#90caf9", alpha=0.55)
    add_rect(ax, STAGE_X,  STAGE_Y,  STAGE_W,  STAGE_D,  "#ffe082", alpha=0.60)
    add_rect(ax, CTRL_X,   CTRL_Y,   CTRL_W,   CTRL_D,   "#ce93d8", alpha=0.50)

    ax.text(DOCK_X   + DOCK_W/2,   DOCK_Y   + DOCK_D/2,   "LOADING\nDOCK",   ha="center", va="center", fontsize=8, zorder=6)
    ax.text(CHARGE_X + CHARGE_W/2, CHARGE_Y + CHARGE_D/2, "ROBOT\nCHARGING", ha="center", va="center", fontsize=7.5, zorder=6)
    ax.text(STAGE_X  + STAGE_W/2,  STAGE_Y  + STAGE_D/2,  "STAGING\nAREA",   ha="center", va="center", fontsize=8, zorder=6)
    ax.text(CTRL_X   + CTRL_W/2,   CTRL_Y   + CTRL_D/2,   "CONTROL\nROOM",   ha="center", va="center", fontsize=7.5, zorder=6)

    # ---- Racks ----------------------------------------------------------
    for xs in X_STARTS:
        add_rect(ax, xs, RACK_S_Y0, RACK_W, RACK_S_Y1 - RACK_S_Y0, "#78909c", zorder=4)
        ax.text(xs + RACK_W/2, (RACK_S_Y0 + RACK_S_Y1)/2,
                "RACK", ha="center", va="center", rotation=90,
                fontsize=6, color="white", zorder=5)
        add_rect(ax, xs, RACK_N_Y0, RACK_W, RACK_N_Y1 - RACK_N_Y0, "#546e7a", zorder=4)
        ax.text(xs + RACK_W/2, (RACK_N_Y0 + RACK_N_Y1)/2,
                "RACK", ha="center", va="center", rotation=90,
                fontsize=6, color="white", zorder=5)

    # Cross-aisle label
    ax.text(W/2, 24.0, "← 4 m cross-aisle →", ha="center", va="center",
            fontsize=8, color="#3949ab",
            bbox=dict(facecolor="white", edgecolor="none", pad=1), zorder=6)

    # Longitudinal aisles (dashed guide lines)
    for xs in X_STARTS:
        cx = xs + RACK_W + AISLE_W / 2
        if cx < W - 2:
            ax.plot([cx, cx], [4.5, D - 4.5],
                    linestyle=":", linewidth=0.7, color="#9fa8da", zorder=2)

    # ---- gNB ------------------------------------------------------------
    ax.scatter(GNB_X, GNB_Y, marker="^", s=260, color="#d32f2f",
               edgecolor="white", linewidth=1.3, zorder=9)
    ax.text(GNB_X, GNB_Y + 1.3,
            f"gNB  (x={GNB_X:.0f}, y={GNB_Y:.0f}, z={GNB_Z:.0f} m)  ceiling",
            ha="center", fontsize=8.5, color="#b71c1c", fontweight="bold", zorder=10)

    # ---- MQTT links (thin dashed lines from nodes to controller) --------
    ctrl_x, ctrl_y = CONTROLLER[0][0], CONTROLLER[0][1]
    for x, y, _lbl, atype in ALL_NODES:
        if atype == "controller":
            continue
        color = APP_STYLE[atype]["color"]
        ax.plot([ctrl_x, x], [ctrl_y, y],
                linestyle=":", linewidth=0.55, color=color, alpha=0.30, zorder=3)

    # ---- Deploy all application nodes -----------------------------------
    for x, y, lbl, atype in ALL_NODES:
        st = APP_STYLE[atype]
        ax.scatter(x, y,
                   marker=st["marker"], s=st["size"],
                   color=st["color"], edgecolor="white",
                   linewidth=0.9, zorder=8)
        # offset label so it doesn't sit on the marker
        dy = 1.1 if y < D - 4 else -1.5
        ax.text(x, y + dy, lbl,
                ha="center", fontsize=7, color=st["color"],
                fontweight="bold", zorder=9)

    # ---- Dimension annotations ------------------------------------------
    dim_arrow(ax, (0, -3.2), (W, -3.2), "60 m")
    dim_arrow(ax, (-3.8, 0), (-3.8, D), "40 m", (-0.8, 0))
    dim_arrow(ax, (X_STARTS[0], 4.2), (X_STARTS[0] + RACK_W, 4.2), "2 m")
    dim_arrow(ax, (X_STARTS[0] + RACK_W, 3.1),
              (X_STARTS[1], 3.1), f"{AISLE_W:.1f} m")

    # ---- Legend ---------------------------------------------------------
    handles = []

    # Beam footprints
    for lbl, _rx, _ry, _a, color in BEAMS:
        handles.append(Line2D([0], [0], linestyle="--", color=color, linewidth=1.6,
                               label=f"Beam coverage — {lbl} array"))
    handles.append(mpatches.Patch(fc="none", ec="none", label=""))

    # App types
    for atype, st in APP_STYLE.items():
        handles.append(Line2D([0], [0], marker=st["marker"], color="none",
                               markerfacecolor=st["color"], markersize=9,
                               label=st["label"]))
    handles.append(mpatches.Patch(fc="none", ec="none", label=""))

    # Infrastructure
    handles += [
        Line2D([0], [0], marker="^", color="none",
               markerfacecolor="#d32f2f", markersize=11, label="gNB (ceiling, z=6 m)"),
        mpatches.Patch(fc="#78909c", ec="#333", label="Rack south bank (2×16×3.2 m)"),
        mpatches.Patch(fc="#546e7a", ec="#333", label="Rack north bank (2×8×3.2 m)"),
        mpatches.Patch(fc="#a5d6a7", alpha=0.55, label="Loading dock"),
        mpatches.Patch(fc="#ffe082", alpha=0.60, label="Staging area"),
        mpatches.Patch(fc="#ce93d8", alpha=0.50, label="Control room"),
        mpatches.Patch(fc="#90caf9", alpha=0.55, label="Robot charging"),
    ]

    ax.legend(handles=handles, loc="lower left", fontsize=7.5,
              framealpha=0.96, ncol=1, borderpad=0.8)

    # ---- App-to-location summary table (top-right inset) ----------------
    table_txt = (
        "Application deployment summary\n"
        "─────────────────────────────────────────\n"
        " ● ControllerApp    → Control Room (server)\n"
        " ○ RobotApp         → R1–R4 (mobile, aisles)\n"
        " ○ CameraApp        → R1–R4 (on each robot)\n"
        " ▲ TempSensorApp    → RS1–RS8 (rack nodes)\n"
        " ▲ HumiditySensorApp→ RS1–RS8 (rack nodes)\n"
        " ▲ RackSensorApp    → RS1–RS8 (rack nodes)\n"
        " ◆ PackageSensorApp → PS1–PS4 (dock+staging)\n"
        " + VideoClientApp   → VC1–VC2 (staging)\n"
        " ★ WithdrawalApp    → WD  (dock terminal)\n"
        "─────────────────────────────────────────\n"
        "Array-size impact by zone\n"
        "  R1/R2  (deep NLOS)  → 8×8 critical\n"
        "  R3     (NLOS)       → 4×4 adequate\n"
        "  R4/VC  (LOS)        → 2×2 sufficient\n"
        "  RS*    (in rack)    → beamforming reach\n"
        "  PS*/WD (dock)       → edge of coverage"
    )
    ax.text(W - 1.5, D + 4.5, table_txt,
            va="top", ha="right", fontsize=7.2, family="monospace",
            zorder=11,
            bbox=dict(facecolor="#f3e5f5", edgecolor="#7b1fa2",
                      boxstyle="round,pad=0.6", alpha=0.95))

    fig.tight_layout()
    out_png = output_dir / "warehouse_geometry_v2.png"
    out_svg = output_dir / "warehouse_geometry_v2.svg"
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    fig.savefig(out_svg, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_png}")
    print(f"Saved: {out_svg}")


def main() -> None:
    default_out = Path(__file__).resolve().parent / "generated"
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=default_out)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw(args.output_dir)


if __name__ == "__main__":
    main()
