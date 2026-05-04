import shutil
import subprocess
from pathlib import Path

import pandas as pd


SCRIPT_DIR = Path(__file__).parent.absolute()
NS3_ROOT = SCRIPT_DIR.parent.parent
NS3_EXE = NS3_ROOT / "ns3"
OUTPUT_BASE = Path("/home/aung/code/new_docte6g/results") / "mimo_channel_smoke"


def run_case(size):
    output_dir = OUTPUT_BASE / f"sionna_{size}x{size}"
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(NS3_EXE),
        "run",
        (
            "sionnart-nr-urban-macro "
            "--simMode=sionna "
            "--simTime=5 "
            "--numUes=6 "
            "--ueDist_min=200 "
            "--ueDist_max=350 "
            "--gnbTxPowerDbm=38 "
            f"--gnbAntennaRows={size} "
            f"--gnbAntennaCols={size} "
            f"--outputDir={output_dir}"
        ),
    ]
    result = subprocess.run(cmd, cwd=NS3_ROOT, capture_output=True, text=True)
    (output_dir / "run_stdout.log").write_text(result.stdout)
    (output_dir / "run_stderr.log").write_text(result.stderr)
    if result.returncode != 0:
        raise RuntimeError(
            f"{size}x{size} failed with exit code {result.returncode}\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return output_dir


def weighted_mean(df, value_col):
    samples = df["Samples"].sum()
    if samples <= 0:
        raise AssertionError("No samples available for weighted mean")
    return float((df[value_col] * df["Samples"]).sum() / samples)


def load_metrics(output_dir):
    cqi = pd.read_csv(output_dir / "urban_macro_cqi_feedback_stats.csv")
    gain = pd.read_csv(output_dir / "urban_macro_mimo_channel_gain_stats.csv")
    prop = pd.read_csv(output_dir / "urban_macro_propagation_stats.csv")
    flow = pd.read_csv(output_dir / "urban_macro_flow_stats.csv")

    if cqi.empty:
        raise AssertionError(f"No CQI feedback samples in {output_dir}")
    if gain.empty:
        raise AssertionError(f"No MIMO channel-gain samples in {output_dir}")

    dl_gain = gain[gain["SrcId"] == 0]
    ul_gain = gain[gain["DstId"] == 0]
    if dl_gain.empty:
        raise AssertionError(f"No DL MIMO channel-gain rows in {output_dir}")
    if ul_gain.empty:
        raise AssertionError(f"No UL/reverse MIMO channel-gain rows in {output_dir}")
    if (dl_gain["TxPorts"] <= 0).any() or (dl_gain["RxPorts"] <= 0).any():
        raise AssertionError(f"Invalid DL MIMO matrix dimensions in {output_dir}")
    if (ul_gain["TxPorts"] <= 0).any() or (ul_gain["RxPorts"] <= 0).any():
        raise AssertionError(f"Invalid UL/reverse MIMO matrix dimensions in {output_dir}")

    app_flow = flow[flow["IsApplicationFlow"] == 1]
    tx_packets = int(app_flow["TxPackets"].sum())
    rx_packets = int(app_flow["RxPackets"].sum())
    delivery = 100.0 * rx_packets / tx_packets if tx_packets else 0.0

    dl_prop = prop[prop["SrcId"] == 0]
    return {
        "cqi": weighted_mean(cqi, "MeanCqi"),
        "mcs": weighted_mean(cqi, "MeanMcs"),
        "rank": weighted_mean(cqi, "MeanRank"),
        "dl_gain_db": weighted_mean(dl_gain, "AvgEffectiveChannelGain_dB"),
        "ul_gain_db": weighted_mean(ul_gain, "AvgEffectiveChannelGain_dB"),
        "pathloss_db": float(dl_prop["Pathloss_dB"].mean()),
        "throughput_mbps": float(app_flow["Throughput_Kbps"].sum()) / 1000.0,
        "delivery_pct": delivery,
    }


def assert_non_decreasing(label, values, tolerance=0.25):
    for left, right in zip(values, values[1:]):
        if right + tolerance < left:
            raise AssertionError(f"{label} is not non-decreasing: {values}")


def main():
    OUTPUT_BASE.mkdir(parents=True, exist_ok=True)

    metrics = {}
    for size in (2, 4, 8):
        out = run_case(size)
        metrics[size] = load_metrics(out)

    print("array,cqi,mcs,rank,dl_gain_db,ul_gain_db,pathloss_db,throughput_mbps,delivery_pct")
    for size in (2, 4, 8):
        m = metrics[size]
        print(
            f"{size}x{size},{m['cqi']:.3f},{m['mcs']:.3f},{m['rank']:.3f},"
            f"{m['dl_gain_db']:.3f},{m['ul_gain_db']:.3f},{m['pathloss_db']:.3f},"
            f"{m['throughput_mbps']:.3f},{m['delivery_pct']:.3f}"
        )

    pathloss_values = [metrics[size]["pathloss_db"] for size in (2, 4, 8)]
    if max(pathloss_values) - min(pathloss_values) > 1.5:
        raise AssertionError(f"Path loss changed too much with array size: {pathloss_values}")

    assert_non_decreasing("CQI", [metrics[size]["cqi"] for size in (2, 4, 8)])
    assert_non_decreasing("MCS", [metrics[size]["mcs"] for size in (2, 4, 8)])
    assert_non_decreasing("rank", [metrics[size]["rank"] for size in (2, 4, 8)])

    if metrics[8]["delivery_pct"] + 1.0 < metrics[2]["delivery_pct"]:
        raise AssertionError("8x8 delivery regressed by more than 1 percentage point vs 2x2")

    print(f"MIMO channel smoke test passed. Outputs: {OUTPUT_BASE}")


if __name__ == "__main__":
    main()
