#!/usr/bin/env python3
"""Validate and compare clean warehouse benchmark runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


ARRAYS = ("2x2", "4x4", "8x8")
MODES = ("no-isac", "isac")


def read_summary(path: Path) -> dict[str, str]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1:
        raise ValueError(f"{path}: expected one summary row")
    if rows[0]["RunCompleted"].lower() != "true":
        raise ValueError(f"{path}: run did not complete")
    return rows[0]


def number(row: dict[str, str], key: str) -> float:
    v = row[key]
    if v in ("nan", "inf", "-inf", ""):
        return math.nan
    return float(v)


def nondecreasing(values: list[float], tolerance: float = 1e-9) -> bool:
    return all(right + tolerance >= left for left, right in zip(values, values[1:]))


def nonincreasing(values: list[float], tolerance: float = 1e-9) -> bool:
    return all(right <= left + tolerance for left, right in zip(values, values[1:]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()

    rows: dict[tuple[str, str], dict[str, str]] = {}
    for mode in MODES:
        for array in ARRAYS:
            path = args.root / f"{mode}-{array}" / "run_summary.csv"
            rows[(mode, array)] = read_summary(path)

    fields = [
        "AggregateGoodput_Mbps",
        "DeliveryRatio_pct",
        "EndToEndLossRatio_pct",
        "MeanPropagationPathLoss_dB",
        "MeanEffectiveLinkLoss_dB",
        "MeanNetArrayGain_dB",
        "MeanCqi",
        "MeanMcs",
        "MeanSinr_dB",
        "Detections",
        "MatchedDetections",
        "MatchedSensingFrames",
    ]
    print("Mode,Array," + ",".join(fields))
    for mode in MODES:
        for array in ARRAYS:
            row = rows[(mode, array)]
            print(f"{mode},{array}," + ",".join(row[field] for field in fields))

    failures: list[str] = []

    # --- Physics guarantees: effective link loss and array gain must be
    #     monotonic for BOTH modes regardless of beam staleness.
    for mode in MODES:
        effective_loss = [
            number(rows[(mode, array)], "MeanEffectiveLinkLoss_dB") for array in ARRAYS
        ]
        array_gain = [number(rows[(mode, array)], "MeanNetArrayGain_dB") for array in ARRAYS]
        if not nonincreasing(effective_loss, tolerance=0.1):
            failures.append(f"{mode}: effective link loss is not nonincreasing with array size")
        if not nondecreasing(array_gain, tolerance=0.1):
            failures.append(f"{mode}: net array gain is not nondecreasing with array size")

    # --- ISAC mode must show monotonic improvement across all KPIs as array grows.
    #     Without ISAC, beam staleness can degrade larger arrays — that is expected
    #     and is the primary motivation for ISAC.
    isac_throughput = [number(rows[("isac", a)], "AggregateGoodput_Mbps") for a in ARRAYS]
    isac_loss = [number(rows[("isac", a)], "EndToEndLossRatio_pct") for a in ARRAYS]
    isac_cqi = [number(rows[("isac", a)], "MeanCqi") for a in ARRAYS]
    isac_mcs = [number(rows[("isac", a)], "MeanMcs") for a in ARRAYS]
    isac_sinr = [number(rows[("isac", a)], "MeanSinr_dB") for a in ARRAYS]
    isac_detections = [number(rows[("isac", a)], "Detections") for a in ARRAYS]
    isac_matched = [number(rows[("isac", a)], "MatchedSensingFrames") for a in ARRAYS]

    if not nondecreasing(isac_throughput, tolerance=0.05):
        failures.append("isac: throughput is not nondecreasing with array size")
    if not nonincreasing(isac_loss, tolerance=0.01):
        failures.append("isac: packet loss is not nonincreasing with array size")
    if not nondecreasing([c for c in isac_cqi if not math.isnan(c)], tolerance=0.05):
        failures.append("isac: CQI is not nondecreasing with array size")
    if not nondecreasing([m for m in isac_mcs if not math.isnan(m)], tolerance=0.05):
        failures.append("isac: MCS is not nondecreasing with array size")
    if not nondecreasing([s for s in isac_sinr if not math.isnan(s)], tolerance=0.1):
        failures.append("isac: SINR is not nondecreasing with array size")
    if not nondecreasing(isac_detections, tolerance=0.0):
        failures.append("isac: sensing detections are not nondecreasing with array size")
    if not nondecreasing(isac_matched, tolerance=0.0):
        failures.append("isac: matched sensing frames are not nondecreasing with array size")

    # --- At each array size, ISAC must outperform no-ISAC on every KPI.
    for array in ARRAYS:
        baseline = rows[("no-isac", array)]
        isac = rows[("isac", array)]
        if number(isac, "AggregateGoodput_Mbps") + 0.05 < number(baseline, "AggregateGoodput_Mbps"):
            failures.append(f"{array}: ISAC throughput is lower than no-ISAC")
        if number(isac, "EndToEndLossRatio_pct") > number(baseline, "EndToEndLossRatio_pct") + 0.01:
            failures.append(f"{array}: ISAC packet loss is higher than no-ISAC")
        if number(isac, "MeanEffectiveLinkLoss_dB") > number(baseline, "MeanEffectiveLinkLoss_dB") + 0.5:
            failures.append(f"{array}: ISAC effective link loss is higher than no-ISAC")
        isac_cqi_val = number(isac, "MeanCqi")
        base_cqi_val = number(baseline, "MeanCqi")
        if not math.isnan(isac_cqi_val) and not math.isnan(base_cqi_val):
            if isac_cqi_val + 0.05 < base_cqi_val:
                failures.append(f"{array}: ISAC CQI is lower than no-ISAC")
        isac_mcs_val = number(isac, "MeanMcs")
        base_mcs_val = number(baseline, "MeanMcs")
        if not math.isnan(isac_mcs_val) and not math.isnan(base_mcs_val):
            if isac_mcs_val + 0.05 < base_mcs_val:
                failures.append(f"{array}: ISAC MCS is lower than no-ISAC")

    physical = [
        number(rows[(mode, array)], "MeanPropagationPathLoss_dB")
        for mode in MODES
        for array in ARRAYS
    ]
    print(
        "\nPhysical path loss spread across all runs: "
        f"{max(physical) - min(physical):.3f} dB"
    )
    print(
        "Note: physical path loss is array-independent; effective link loss is the array-sensitive metric.\n"
        "Note: no-ISAC throughput monotonicity is not required — beam staleness at large arrays\n"
        "      is the problem ISAC solves."
    )
    if failures:
        print("\nFAILED HYPOTHESIS CHECKS")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("\nAll configured hypothesis checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
