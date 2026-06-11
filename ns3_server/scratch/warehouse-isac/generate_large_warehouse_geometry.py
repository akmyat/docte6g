#!/usr/bin/env python3
"""
200 m × 100 m warehouse geometry for 30 dBm gNB TX power differentiation.

WHY 30 dBm fails on the current geometry
-----------------------------------------
Current large warehouse: ~90 m × 60 m, gNB in SW corner.
Typical UE distance: 40–60 m. FSPL at 50 m, 15 GHz ≈ 90 dB.
Even 2×2 gets SNR ≈ +24 dB — saturated at max MCS.
No array size difference is visible when every link is already at the ceiling.

TARGET operating window (30 dBm, 200 MHz BW)
----------------------------------------------
Noise floor: −84 dBm  (200 MHz, 7 dB NF).
FSPL at 160 m, 15 GHz ≈ 100 dB  → 2×2 SNR_base = +20 dB (still saturated).
Each metal rack row crossed adds 8–15 dB penetration loss at 15 GHz.

  0 racks (LOS aisle):  2×2 = +20 dB / 8×8 = +32 dB  → both saturated (control)
  1 rack  (+12 dB est): 2×2 = + 8 dB / 8×8 = +20 dB  → 2×2 marginal, 8×8 good
  2 racks (+24 dB est): 2×2 = − 4 dB / 8×8 = + 8 dB  → 2×2 outage, 8×8 functional

The 12 dB gap between 2×2 and 8×8 (per rack crossing) is the proof.
ISAC beam-steering adds 2–4 dB equivalent on top by pre-steering to
the reflected path before the link degrades mid-route.

GEOMETRY DESIGN
----------------
Warehouse:  200 m N-S × 100 m E-W
Coordinate: x ∈ [−50, +50],  y ∈ [−100, +100]
            south wall y=−100, north wall y=+100

gNB: x=+8, y=−95, z=8 m  (south end, centre of Aisle 3, 8 m ceiling mount)

E-W layout (west→east):
  8 m west corridor | R1 5 m | A1 14 m | R2 5 m | A2 14 m |
  R3 5 m | A3 14 m (gNB) | R4 5 m | A4 14 m | R5 5 m | 11 m east corridor

Rack rows: 5 m wide, 5 m tall, 160 m long (y=−80 to +80)

N-S layout:
  y = −100 → −80 : south staging (20 m) — robots park here
  y = −80  → +80 : rack zone (160 m)    — 5 parallel rack rows
  y = +80  → +100 : north buffer (20 m)

Rack sensors (NR UEs):
  S1 x=−30, y=+65  Aisle 1  signal path crosses R3 then R2  →  2 racks
  S2 x=−11, y=+65  Aisle 2  signal path crosses R3          →  1 rack
  S3 x= +8, y=+65  Aisle 3  stays in centre aisle (LOS)     →  0 racks
  S4 x=+27, y=+65  Aisle 4  signal path crosses R4          →  1 rack  (sym S2)

Mobile robots: 3, starting in south staging area, one per served aisle.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from textwrap import dedent

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ---------------------------------------------------------------------------
# Radio parameters
# ---------------------------------------------------------------------------
CARRIER_HZ   = 15e9
BW_HZ        = 200e6          # 200 MHz  (1584 sub-carriers × 120 kHz SCS)
GNB_TX_DBM   = 30.0
UE_NF_DB     =  7.0
WAVELENGTH_M = 299_792_458.0 / CARRIER_HZ
ARRAY_GAINS  = {"2x2": 6.0, "4x4": 12.0, "8x8": 18.0}   # dBi beamforming gain
RACK_LOSS_DB = 12.0            # assumed penetration loss per metal rack row

# ---------------------------------------------------------------------------
# Scene bounds  (200 m × 100 m)
# ---------------------------------------------------------------------------
BOUNDS = {"x_min": -50.0, "x_max": 50.0, "y_min": -100.0, "y_max": 100.0}
CEILING_HEIGHT_M = 10.0
WALL_THICKNESS_M =  0.3

# ---------------------------------------------------------------------------
# E-W layout (west→east, total 100 m):
#   8 m west corr | R1(5m) | A1(14m) | R2(5m) | A2(14m) |
#   R3(5m) | A3(14m,gNB) | R4(5m) | A4(14m) | R5(5m) | 11 m east corr
#
# Rack row centres (x): R1=−39.5, R2=−20.5, R3=−1.5, R4=+17.5, R5=+36.5
# Aisle centres   (x): A1=−30,   A2=−11,   A3=+8,   A4=+27
# ---------------------------------------------------------------------------
RACK_HALF_WIDTH    = 2.5     # 5 m total width
RACK_HEIGHT        = 5.0
RACK_Y_START       = -80.0
RACK_Y_END         = +80.0
RACK_ROW_CENTERS_X = [-39.5, -20.5, -1.5, 17.5, 36.5]

RACKS = [
    {
        "name": f"rack_row_{i + 1}",
        "x_min": cx - RACK_HALF_WIDTH,
        "x_max": cx + RACK_HALF_WIDTH,
        "y_min": RACK_Y_START,
        "y_max": RACK_Y_END,
        "z_min": 0.0,
        "z_max": RACK_HEIGHT,
        "material": "itu_metal",
    }
    for i, cx in enumerate(RACK_ROW_CENTERS_X)
]

AISLE_EDGES = [
    # (x_west, x_east, centre, label)
    (-50.0, -42.0, -46.0, "W-corr"),
    (-37.0, -23.0, -30.0, "A1"),
    (-18.0,  -4.0, -11.0, "A2"),
    (  1.0,  15.0,   8.0, "A3"),
    ( 20.0,  34.0,  27.0, "A4"),
    ( 39.0,  50.0,  44.5, "E-corr"),
]
AISLE_CENTERS_X = [-30.0, -11.0, 8.0, 27.0]   # A1–A4

# ---------------------------------------------------------------------------
# Conveyor belt in south staging area (wired, used by package sensors)
# ---------------------------------------------------------------------------
CONVEYOR = {
    "name": "conveyor_belt",
    "x_min": -10.0, "x_max": 10.0,
    "y_min": -97.0, "y_max": -88.0,
    "z_min":  0.0,  "z_max":  0.8,
    "material": "itu_metal",
}

# ---------------------------------------------------------------------------
# Radio nodes
# ---------------------------------------------------------------------------
GNB_POS     = [8.0, -95.0, 8.0]
GNB_LOOK_AT = [0.0,   0.0, 1.5]

SENSOR_Y = 65.0    # all rack sensors at this Y (15 m south of rack end)

RACK_SENSORS = [
    {
        "name":          "rack_sensor_1",
        "role":          "fixed_rack_sensor",
        "aisle":         "A1",
        "rack_crossings": 2,
        "crossing_rows": ["R3", "R2"],
        "position_m":    [-30.0, SENSOR_Y, 1.5],
    },
    {
        "name":          "rack_sensor_2",
        "role":          "fixed_rack_sensor",
        "aisle":         "A2",
        "rack_crossings": 1,
        "crossing_rows": ["R3"],
        "position_m":    [-11.0, SENSOR_Y, 1.5],
    },
    {
        "name":          "rack_sensor_3",
        "role":          "fixed_rack_sensor",
        "aisle":         "A3",
        "rack_crossings": 0,
        "crossing_rows": [],
        "position_m":    [8.0, SENSOR_Y, 1.5],
    },
    {
        "name":          "rack_sensor_4",
        "role":          "fixed_rack_sensor",
        "aisle":         "A4",
        "rack_crossings": 1,
        "crossing_rows": ["R4"],
        "position_m":    [27.0, SENSOR_Y, 1.5],
    },
]

ROBOT_STARTS = [
    [-30.0, -85.0, 1.5],   # A1 side → serves S1
    [-11.0, -85.0, 1.5],   # A2 side → serves S2
    [  8.0, -85.0, 1.5],   # A3 (gNB aisle) → serves S3
]

ROBOT_ROUTES = [
    {
        "name": "robot_1_route",
        "waypoints_m": [
            [-30.0, -85.0, 1.5],
            [-30.0,  65.0, 1.5],
            [-30.0, -85.0, 1.5],
        ],
    },
    {
        "name": "robot_2_route",
        "waypoints_m": [
            [-11.0, -85.0, 1.5],
            [-11.0,  65.0, 1.5],
            [-11.0, -85.0, 1.5],
        ],
    },
    {
        "name": "robot_3_route",
        "waypoints_m": [
            [ 8.0, -85.0, 1.5],
            [ 8.0,  65.0, 1.5],
            [ 8.0, -85.0, 1.5],
        ],
    },
]

PACKAGE_SENSORS = [
    {"name": "package_sensor_1", "position_m": [-5.0, -91.0, 1.2]},
    {"name": "package_sensor_2", "position_m": [ 5.0, -91.0, 1.2]},
]

VIDEO_CLIENT = {"name": "video_client_1", "position_m": [48.0, 90.0, 1.5]}
DROP_ZONE    = [0.0, -90.0, 1.5]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _d3(a, b):
    return math.dist(a[:3], b[:3])


def _fspl(d_m):
    return 20.0 * math.log10(4.0 * math.pi * d_m / WAVELENGTH_M)


def _noise():
    return -174.0 + 10.0 * math.log10(BW_HZ) + UE_NF_DB


def _snr(pl_db, gain_db):
    return GNB_TX_DBM - pl_db + gain_db - _noise()


def _sensor_snr(rs):
    d = _d3(GNB_POS, rs["position_m"])
    fspl = _fspl(d)
    nc = rs["rack_crossings"]
    pl = fspl + nc * RACK_LOSS_DB
    return {
        "distance_m": round(d, 1),
        "fspl_db":    round(fspl, 1),
        "total_pl_db": round(pl, 1),
        "snr_db": {k: round(_snr(pl, g), 1) for k, g in ARRAY_GAINS.items()},
        "snr_fspl_only": {k: round(_snr(fspl, g), 1) for k, g in ARRAY_GAINS.items()},
    }


# ---------------------------------------------------------------------------
# Mitsuba3 / Sionna XML
# ---------------------------------------------------------------------------

def _cube_transform(cx, cy, cz, sx, sy, sz):
    return dedent(f"""\
        <transform name="to_world">
            <scale value="{sx/2:.4f} {sy/2:.4f} {sz/2:.4f}"/>
            <translate value="{cx:.4f} {cy:.4f} {cz:.4f}"/>
        </transform>""")


def generate_sionna_xml():
    lines = ['<scene version="2.1.0">\n']
    lines += [
        '    <integrator type="path" id="elm__0" name="elm__0">',
        '        <integer name="max_depth" value="12"/>',
        '    </integrator>\n',
    ]

    for mat in ["itu_concrete", "itu_metal", "itu_glass", "itu_wood"]:
        lines += [
            f'    <bsdf type="twosided" id="mat-{mat}" name="mat-{mat}">',
            f'        <bsdf type="principled" name="bsdf">',
            f'            <rgb value="0.800000 0.800000 0.800000" name="base_color"/>',
            f'            <float name="roughness" value="0.250000"/>',
            f'            <float name="specular" value="0.500000"/>',
            f'        </bsdf>',
            f'    </bsdf>',
        ]
    lines.append('')

    xmin, xmax = BOUNDS["x_min"], BOUNDS["x_max"]
    ymin, ymax = BOUNDS["y_min"], BOUNDS["y_max"]
    bw = xmax - xmin
    bd = ymax - ymin
    cx_b = (xmin + xmax) / 2
    cy_b = (ymin + ymax) / 2
    t = WALL_THICKNESS_M
    h = CEILING_HEIGHT_M

    lines += [
        '    <!-- Floor -->',
        '    <shape type="cube" id="mesh-floor" name="mesh-floor">',
        f'        {_cube_transform(cx_b, cy_b, -0.05, bw, bd, 0.1)}',
        '        <ref id="mat-itu_concrete" name="bsdf"/>',
        '    </shape>\n',
        '    <!-- Ceiling -->',
        '    <shape type="cube" id="mesh-ceiling" name="mesh-ceiling">',
        f'        {_cube_transform(cx_b, cy_b, h + 0.05, bw, bd, 0.1)}',
        '        <ref id="mat-itu_concrete" name="bsdf"/>',
        '    </shape>\n',
    ]

    lines.append('    <!-- Walls -->')
    for name, wx, wy, wz, wsx, wsy, wsz in [
        ("wall_south", cx_b, ymin - t/2, h/2, bw + 2*t, t, h),
        ("wall_north", cx_b, ymax + t/2, h/2, bw + 2*t, t, h),
        ("wall_west",  xmin - t/2, cy_b, h/2, t, bd, h),
        ("wall_east",  xmax + t/2, cy_b, h/2, t, bd, h),
    ]:
        lines += [
            f'    <shape type="cube" id="mesh-{name}" name="mesh-{name}">',
            f'        {_cube_transform(wx, wy, wz, wsx, wsy, wsz)}',
            '        <ref id="mat-itu_concrete" name="bsdf"/>',
            '    </shape>',
        ]
    lines.append('')

    lines.append('    <!-- Metal rack rows (5 rows, N-S oriented) -->')
    for rack in RACKS:
        rx = (rack["x_min"] + rack["x_max"]) / 2
        ry = (rack["y_min"] + rack["y_max"]) / 2
        rz = rack["z_max"] / 2
        lines += [
            f'    <shape type="cube" id="mesh-{rack["name"]}" name="mesh-{rack["name"]}">',
            f'        {_cube_transform(rx, ry, rz, rack["x_max"]-rack["x_min"], rack["y_max"]-rack["y_min"], rack["z_max"])}',
            '        <ref id="mat-itu_metal" name="bsdf"/>',
            '    </shape>',
        ]
    lines.append('')

    lines.append('    <!-- Conveyor belt -->')
    cvx = (CONVEYOR["x_min"] + CONVEYOR["x_max"]) / 2
    cvy = (CONVEYOR["y_min"] + CONVEYOR["y_max"]) / 2
    cvz = CONVEYOR["z_max"] / 2
    lines += [
        '    <shape type="cube" id="mesh-conveyor_belt" name="mesh-conveyor_belt">',
        f'        {_cube_transform(cvx, cvy, cvz, CONVEYOR["x_max"]-CONVEYOR["x_min"], CONVEYOR["y_max"]-CONVEYOR["y_min"], CONVEYOR["z_max"])}',
        '        <ref id="mat-itu_metal" name="bsdf"/>',
        '    </shape>\n',
    ]

    lines.append('</scene>')
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Collision OBJ  (for robot pathfinding in ns-3)
# ---------------------------------------------------------------------------

def generate_collision_obj():
    verts = []
    faces = []

    def add_box(x0, x1, y0, y1, z0, z1):
        base = len(verts) + 1
        corners = [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
        ]
        verts.extend(corners)
        b = base
        faces.extend([
            (b+0, b+2, b+1), (b+0, b+3, b+2),
            (b+4, b+5, b+6), (b+4, b+6, b+7),
            (b+0, b+1, b+5), (b+0, b+5, b+4),
            (b+2, b+3, b+7), (b+2, b+7, b+6),
            (b+0, b+4, b+7), (b+0, b+7, b+3),
            (b+1, b+2, b+6), (b+1, b+6, b+5),
        ])

    xmin, xmax = BOUNDS["x_min"], BOUNDS["x_max"]
    ymin, ymax = BOUNDS["y_min"], BOUNDS["y_max"]
    t = WALL_THICKNESS_M
    h = CEILING_HEIGHT_M

    add_box(xmin, xmax, ymin, ymax, -0.1, 0.0)   # floor
    add_box(xmin-t, xmax+t, ymin-t, ymin,    0.0, h)  # south wall
    add_box(xmin-t, xmax+t, ymax,   ymax+t,  0.0, h)  # north wall
    add_box(xmin-t, xmin,   ymin-t, ymax+t,  0.0, h)  # west wall
    add_box(xmax,   xmax+t, ymin-t, ymax+t,  0.0, h)  # east wall
    for rack in RACKS:
        add_box(rack["x_min"], rack["x_max"],
                rack["y_min"], rack["y_max"],
                rack["z_min"], rack["z_max"])

    lines = ["# warehouse_large_v2 collision mesh — auto-generated", ""]
    for vx, vy, vz in verts:
        lines.append(f"v {vx:.4f} {vy:.4f} {vz:.4f}")
    lines.append("")
    for fa, fb, fc in faces:
        lines.append(f"f {fa} {fb} {fc}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Layout diagram
# ---------------------------------------------------------------------------
SENSOR_COLORS = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4"]   # S1..S4
ROBOT_COLORS  = ["#9467bd", "#8c564b", "#e377c2"]


def _beam_crossings(gx, gy, sx, sy):
    """Return list of (x_enter, y_enter) where beam crosses each rack row."""
    crossings = []
    dy = sy - gy
    dx = sx - gx
    if abs(dy) < 1e-9:
        return crossings
    for rack in RACKS:
        rx0, rx1 = rack["x_min"], rack["x_max"]
        # find y where x = rx0 and rx1 along the line
        # x(y) = gx + dx/dy * (y - gy)
        if abs(dx) < 1e-9:
            if gx < rx0 or gx > rx1:
                continue
            # beam always inside this rack — skip (shouldn't happen)
            continue
        slope_xy = dx / dy    # dx per unit dy
        # y where x = rx0:  rx0 = gx + slope_xy*(y-gy) → y = gy + (rx0-gx)/slope_xy
        y_at_rx0 = gy + (rx0 - gx) / slope_xy
        y_at_rx1 = gy + (rx1 - gx) / slope_xy
        y_enter = min(y_at_rx0, y_at_rx1)
        y_exit  = max(y_at_rx0, y_at_rx1)
        # only count if crossing is between gNB and sensor
        y_lo = min(gy, sy)
        y_hi = max(gy, sy)
        if y_exit <= y_lo or y_enter >= y_hi:
            continue
        y_enter = max(y_enter, y_lo)
        y_exit  = min(y_exit,  y_hi)
        x_enter = gx + slope_xy * (y_enter - gy)
        crossings.append((x_enter, y_enter, x_enter + (slope_xy * (y_exit - y_enter)), y_exit))
    return crossings


def draw_layout(output_stem: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 20))

    xmin = BOUNDS["x_min"]
    xmax = BOUNDS["x_max"]
    ymin = BOUNDS["y_min"]
    ymax = BOUNDS["y_max"]
    W = xmax - xmin
    H = ymax - ymin

    # ── warehouse floor ──────────────────────────────────────────────────────
    ax.add_patch(Rectangle((xmin, ymin), W, H,
                            facecolor="#f5f5f0", edgecolor="#222", linewidth=2.5))

    # ── staging zone ─────────────────────────────────────────────────────────
    ax.add_patch(Rectangle((xmin, ymin), W, RACK_Y_START - ymin,
                            facecolor="#e8f4e8", edgecolor="none", alpha=0.7))
    ax.axhline(RACK_Y_START, color="#888", linewidth=1, linestyle=":")
    ax.text(xmin + 1, RACK_Y_START - 5, "South staging / pickup zone  (20 m)",
            fontsize=9, color="#555", style="italic")

    # ── north buffer ─────────────────────────────────────────────────────────
    ax.add_patch(Rectangle((xmin, RACK_Y_END), W, ymax - RACK_Y_END,
                            facecolor="#e8f0f8", edgecolor="none", alpha=0.5))
    ax.axhline(RACK_Y_END, color="#888", linewidth=1, linestyle=":")
    ax.text(xmin + 1, RACK_Y_END + 2, "North buffer  (20 m)",
            fontsize=9, color="#555", style="italic")

    # ── rack rows ─────────────────────────────────────────────────────────────
    for i, rack in enumerate(RACKS):
        rh = rack["y_max"] - rack["y_min"]
        rw = rack["x_max"] - rack["x_min"]
        ax.add_patch(Rectangle((rack["x_min"], rack["y_min"]), rw, rh,
                                facecolor="#3a3a3a", edgecolor="#111", alpha=0.88))
        cx_r = (rack["x_min"] + rack["x_max"]) / 2
        cy_r = (rack["y_min"] + rack["y_max"]) / 2
        ax.text(cx_r, cy_r, f"R{i+1}\n5 m", color="white",
                ha="center", va="center", fontsize=8, weight="bold", rotation=90)

    # ── conveyor belt ─────────────────────────────────────────────────────────
    ax.add_patch(Rectangle((CONVEYOR["x_min"], CONVEYOR["y_min"]),
                            CONVEYOR["x_max"] - CONVEYOR["x_min"],
                            CONVEYOR["y_max"] - CONVEYOR["y_min"],
                            facecolor="#b5651d", edgecolor="#333", alpha=0.75))
    ax.text(0, (CONVEYOR["y_min"] + CONVEYOR["y_max"]) / 2,
            "conveyor", ha="center", va="center", fontsize=7, color="white")

    # ── signal paths from gNB to each sensor ─────────────────────────────────
    gx, gy = GNB_POS[0], GNB_POS[1]
    for rs, col in zip(RACK_SENSORS, SENSOR_COLORS):
        sx, sy = rs["position_m"][0], rs["position_m"][1]
        ax.plot([gx, sx], [gy, sy], color=col, linewidth=1.2,
                linestyle="--", alpha=0.50, zorder=3)
        for x0, y0, x1, y1 in _beam_crossings(gx, gy, sx, sy):
            ax.plot([x0, x1], [y0, y1], color=col, linewidth=4,
                    alpha=0.55, zorder=4, solid_capstyle="round")

    # ── robot routes ─────────────────────────────────────────────────────────
    for route, col in zip(ROBOT_ROUTES, ROBOT_COLORS):
        wps = route["waypoints_m"]
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        ax.plot(xs, ys, color=col, linewidth=1.8, linestyle=":",
                alpha=0.7, zorder=5)
        ax.scatter([xs[0]], [ys[0]], s=90, color=col, marker="o",
                   edgecolors="black", linewidth=0.7, zorder=9)
        ax.annotate(route["name"].replace("_route", "").replace("robot_", "R"),
                    (xs[0], ys[0]), xytext=(-12, -2), textcoords="offset points",
                    fontsize=8, color=col, weight="bold")

    # ── rack sensors ─────────────────────────────────────────────────────────
    for rs, col in zip(RACK_SENSORS, SENSOR_COLORS):
        sx, sy = rs["position_m"][0], rs["position_m"][1]
        budget = _sensor_snr(rs)
        nc = rs["rack_crossings"]
        snr2 = budget["snr_db"]["2x2"]
        snr4 = budget["snr_db"]["4x4"]
        snr8 = budget["snr_db"]["8x8"]
        d    = budget["distance_m"]
        ax.scatter([sx], [sy], marker="s", s=130, color=col, zorder=10,
                   edgecolors="black", linewidth=0.9)
        label = (
            f"{rs['name']}  ({rs['aisle']})\n"
            f"d={d:.0f} m,  {nc} rack{'s' if nc != 1 else ''} crossed\n"
            f"SNR  2×2: {snr2:+.0f} dB\n"
            f"      4×4: {snr4:+.0f} dB\n"
            f"      8×8: {snr8:+.0f} dB"
        )
        x_off = 5 if sx < 0 else -135
        y_off = 5
        ax.annotate(label, (sx, sy), xytext=(x_off, y_off),
                    textcoords="offset points", fontsize=7.5, color=col,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                              edgecolor=col, alpha=0.88))

    # ── package sensors ───────────────────────────────────────────────────────
    for ps in PACKAGE_SENSORS:
        px, py = ps["position_m"][0], ps["position_m"][1]
        ax.scatter([px], [py], marker="P", s=100, color="#ff7f00", zorder=9)
    ax.annotate("pkg sensors\n(wired)",
                (PACKAGE_SENSORS[0]["position_m"][0], PACKAGE_SENSORS[0]["position_m"][1]),
                xytext=(-60, 6), textcoords="offset points", fontsize=7.5, color="#ff7f00")

    # ── video client ──────────────────────────────────────────────────────────
    vx, vy = VIDEO_CLIENT["position_m"][0], VIDEO_CLIENT["position_m"][1]
    ax.scatter([vx], [vy], marker="D", s=100, color="#1b9e77", zorder=9)
    ax.annotate("video client\n(wired)", (vx, vy),
                xytext=(-72, 6), textcoords="offset points", fontsize=7.5)

    # ── drop zone ─────────────────────────────────────────────────────────────
    ax.scatter([DROP_ZONE[0]], [DROP_ZONE[1]], marker="X", s=140,
               color="black", zorder=9)
    ax.annotate("drop zone", (DROP_ZONE[0], DROP_ZONE[1]),
                xytext=(6, 4), textcoords="offset points", fontsize=7.5)

    # ── gNB ───────────────────────────────────────────────────────────────────
    ax.scatter([gx], [gy], marker="^", s=380, color="#cc0000", zorder=11)
    ax.annotate(f"gNB\n30 dBm\n({gx:.0f}, {gy:.0f}, z={GNB_POS[2]:.0f} m)",
                (gx, gy), xytext=(8, 6), textcoords="offset points",
                fontsize=9, weight="bold", color="#cc0000",
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                          edgecolor="#cc0000", alpha=0.85))

    # ── dimension annotations ─────────────────────────────────────────────────
    dim_y = ymax + 5
    ax.annotate("", xy=(xmax, dim_y), xytext=(xmin, dim_y),
                arrowprops=dict(arrowstyle="<->", color="#333", lw=1.5))
    ax.text((xmin+xmax)/2, dim_y + 2, "100 m (E-W)",
            ha="center", va="bottom", fontsize=10, weight="bold")

    dim_x = xmin - 7
    ax.annotate("", xy=(dim_x, ymax), xytext=(dim_x, ymin),
                arrowprops=dict(arrowstyle="<->", color="#333", lw=1.5))
    ax.text(dim_x - 1, (ymin+ymax)/2, "200 m (N-S)",
            ha="right", va="center", fontsize=10, weight="bold", rotation=90)

    # ── aisle / rack width labels at top ─────────────────────────────────────
    label_y = ymax + 11
    for xw, xe, xc, name in AISLE_EDGES:
        if name.endswith("corr"):
            continue
        ax.text(xc, label_y, f"{name}\n14 m", ha="center", va="bottom",
                fontsize=8, color="#555")

    # ── rack row width labels ─────────────────────────────────────────────────
    for rack in RACKS:
        cx_r = (rack["x_min"] + rack["x_max"]) / 2
        ax.annotate("", xy=(rack["x_max"], ymax + 8), xytext=(rack["x_min"], ymax + 8),
                    arrowprops=dict(arrowstyle="<->", color="#999", lw=0.8))

    # ── distance from gNB to sensor-Y label ──────────────────────────────────
    ax.annotate("", xy=(gx + 2, SENSOR_Y), xytext=(gx + 2, gy),
                arrowprops=dict(arrowstyle="<->", color="#cc0000",
                                lw=1.2, linestyle="dashed"))
    ax.text(gx + 4, (SENSOR_Y + gy) / 2,
            f"{abs(SENSOR_Y - gy):.0f} m\n(vertical)", fontsize=8,
            color="#cc0000", va="center")

    # ── rack zone label ───────────────────────────────────────────────────────
    ax.text(xmax - 2, 0, "Rack zone\n160 m", ha="right", va="center",
            fontsize=9, color="#777", style="italic")

    # ── SNR assumption note ───────────────────────────────────────────────────
    ax.text(xmin + 1, ymin + 1,
            f"SNR annotations: {RACK_LOSS_DB:.0f} dB/rack (est. metal shelving at 15 GHz)\n"
            f"Noise floor = −84 dBm  (200 MHz BW, 7 dB NF)\n"
            f"Beamforming gains: 2×2=6 dBi, 4×4=12 dBi, 8×8=18 dBi",
            fontsize=7.5, color="#555",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                      edgecolor="#ccc", alpha=0.9))

    # ── legend ────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(color="#cc0000",  label="gNB — 30 dBm, z=8 m, centre of Aisle 3"),
        mpatches.Patch(color="#3a3a3a",  label="Metal rack rows — 5 m wide × 5 m tall × 160 m long"),
        mpatches.Patch(color="#e8f4e8",  label="South staging / pickup zone"),
    ]
    for rs, col in zip(RACK_SENSORS, SENSOR_COLORS):
        nc = rs["rack_crossings"]
        legend_handles.append(
            mpatches.Patch(color=col,
                           label=f"{rs['name']} ({rs['aisle']}) — {nc} rack{'s' if nc!=1 else ''} crossed"))
    legend_handles += [
        plt.Line2D([0],[0], linestyle="--", color="gray",
                   label="Signal path from gNB (thick = rack crossing)"),
        plt.Line2D([0],[0], linestyle=":", color="#9467bd",
                   label="Robot routes (dashed)"),
    ]
    ax.legend(handles=legend_handles, loc="lower right", fontsize=8,
              framealpha=0.95)

    ax.set_title(
        "Warehouse geometry — 200 m × 100 m  |  30 dBm gNB  |  15 GHz  |  ISAC array-size sweep\n"
        "5 metal rack rows force 0–2 penetrations per sensor path  →  "
        "clear 2×2 / 4×4 / 8×8 SNR differentiation at realistic TX power",
        fontsize=11, pad=14,
    )
    ax.set_xlabel("x (m) — East–West", fontsize=10)
    ax.set_ylabel("y (m) — South–North", fontsize=10)
    ax.set_xlim(xmin - 12, xmax + 8)
    ax.set_ylim(ymin - 6, ymax + 22)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.35, alpha=0.4)

    fig.tight_layout()
    fig.savefig(output_stem.with_suffix(".png"), dpi=180, bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] {output_stem}.png / .svg")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    out = Path(__file__).resolve().parent / "generated"
    out.mkdir(parents=True, exist_ok=True)

    # SNR table
    print(f"\n{'Sensor':<18} {'Dist':>6} {'FSPL':>6} {'Racks':>5} "
          f"{'Total PL':>9} {'SNR 2×2':>8} {'SNR 4×4':>8} {'SNR 8×8':>8}")
    print("-" * 80)
    for rs in RACK_SENSORS:
        b = _sensor_snr(rs)
        print(f"{rs['name']:<18} {b['distance_m']:>5.1f}m {b['fspl_db']:>5.1f}dB "
              f"{rs['rack_crossings']:>5}  {b['total_pl_db']:>8.1f}dB "
              f"{b['snr_db']['2x2']:>7.1f}  {b['snr_db']['4x4']:>7.1f}  {b['snr_db']['8x8']:>7.1f}")

    # JSON spec
    spec = {
        "name": "warehouse_large_v2",
        "warehouse_size_m": "200 N-S × 100 E-W",
        "gnb_tx_power_dbm": GNB_TX_DBM,
        "carrier_frequency_hz": CARRIER_HZ,
        "bandwidth_hz": BW_HZ,
        "design_rationale": (
            "200 m × 100 m warehouse forces 160 m gNB-to-sensor paths. "
            "5 metal rack rows provide 0-2 penetrations per sensor. "
            "At 30 dBm, FSPL alone gives SNR=+20 dB (saturated). "
            "Each rack crossing adds ~12 dB loss, creating clear 2×2/4×4/8×8 tiers."
        ),
        "scene": {
            "bounds_m": BOUNDS,
            "ceiling_height_m": CEILING_HEIGHT_M,
            "rack_zone": {"y_start": RACK_Y_START, "y_end": RACK_Y_END,
                          "length_m": RACK_Y_END - RACK_Y_START},
            "staging_zone": {"y_start": BOUNDS["y_min"], "y_end": RACK_Y_START,
                             "length_m": RACK_Y_START - BOUNDS["y_min"]},
            "racks": RACKS,
            "conveyor": CONVEYOR,
        },
        "gnb": {
            "position_m": GNB_POS,
            "look_at_m": GNB_LOOK_AT,
            "note": "Centre of Aisle 3 at south end, 8 m elevation",
        },
        "rack_sensors": [
            {**rs, **_sensor_snr(rs)} for rs in RACK_SENSORS
        ],
        "mobile_robots": [
            {
                "name": f"robot_{i+1}",
                "start_position_m": s,
                "route": r,
            }
            for i, (s, r) in enumerate(zip(ROBOT_STARTS, ROBOT_ROUTES))
        ],
        "package_sensors": PACKAGE_SENSORS,
        "video_client": VIDEO_CLIENT,
        "drop_zone_m": DROP_ZONE,
        "ns3_integration": {
            "rackSensorPositions": [rs["position_m"] for rs in RACK_SENSORS],
            "mobileRobotPositions": ROBOT_STARTS,
            "gnbPos": GNB_POS,
            "gnbLookAt": GNB_LOOK_AT,
            "bounds_BoxValue": (
                f"Box({BOUNDS['x_min']}, {BOUNDS['x_max']}, "
                f"{BOUNDS['y_min']}, {BOUNDS['y_max']}, 0.0, 2.0)"
            ),
        },
    }

    json_path = out / "warehouse_large_v2.json"
    json_path.write_text(json.dumps(spec, indent=2) + "\n")
    print(f"[OK] {json_path}")

    xml_path = out / "warehouse_large_v2.xml"
    xml_path.write_text(generate_sionna_xml())
    print(f"[OK] {xml_path}")

    obj_path = out / "warehouse_large_v2.obj"
    obj_path.write_text(generate_collision_obj())
    print(f"[OK] {obj_path}")

    draw_layout(out / "warehouse_large_v2")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
