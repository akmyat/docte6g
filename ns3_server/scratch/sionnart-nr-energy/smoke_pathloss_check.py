import math
import shutil
import subprocess
from pathlib import Path

import pandas as pd


SCRIPT_DIR = Path(__file__).parent.absolute()
NS3_ROOT = SCRIPT_DIR.parent.parent
NS3_EXE = NS3_ROOT / "ns3"
OUTPUT_BASE = Path("/home/aung/code/new_docte6g/results") / "pathloss_smoke"


def run_case(name, sim_mode, rows, cols):
    output_dir = OUTPUT_BASE / name
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(NS3_EXE),
        "run",
        (
            "sionnart-nr-urban-macro "
            f"--simMode={sim_mode} "
            f"--gnbAntennaRows={rows} "
            f"--gnbAntennaCols={cols} "
            "--ueAntennaRows=2 "
            "--ueAntennaCols=2 "
            "--numUes=1 "
            "--ueDist_min=100 "
            "--ueDist_max=100 "
            "--loadType=low "
            "--seed=41 "
            "--simTime=2.0 "
            f"--outputDir={output_dir}"
        ),
    ]
    result = subprocess.run(cmd, cwd=NS3_ROOT, capture_output=True, text=True)
    (output_dir / "run_stdout.log").write_text(result.stdout)
    (output_dir / "run_stderr.log").write_text(result.stderr)
    if result.returncode != 0:
        raise RuntimeError(
            f"{name} failed with exit code {result.returncode}\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return output_dir


def fspl_db(frequency_hz, distance_m):
    return 20.0 * math.log10(max(distance_m, 1.0)) + 20.0 * math.log10(frequency_hz) - 147.55


def load_coverage(output_dir):
    cov = pd.read_csv(output_dir / "free_space_coverage_snapshot.csv")
    if len(cov) != 1:
        raise AssertionError(f"Expected one UE coverage row in {output_dir}, found {len(cov)}")
    return cov.iloc[0]


def load_link_pathloss(output_dir):
    cov = load_coverage(output_dir)
    ue_node_id = int(cov["UeNodeId"])
    prop = pd.read_csv(output_dir / "free_space_propagation_stats.csv")
    link = prop[
        ((prop["SrcId"] == 0) & (prop["DstId"] == ue_node_id))
        | ((prop["SrcId"] == ue_node_id) & (prop["DstId"] == 0))
    ]
    if link.empty:
        raise AssertionError(f"No gNB<->UE propagation samples in {output_dir}")
    return float(link["Pathloss_dB"].mean())


def weighted_effective_channel_gain_db(output_dir):
    gain = pd.read_csv(output_dir / "free_space_mimo_channel_gain_stats.csv")
    if gain.empty:
        raise AssertionError(f"No MIMO channel-gain samples in {output_dir}")
    return float(
        (gain["AvgEffectiveChannelGain_dB"] * gain["Samples"]).sum()
        / gain["Samples"].sum()
    )


def assert_close(label, actual, expected, tolerance):
    error = abs(actual - expected)
    print(f"{label}: actual={actual:.3f}, expected={expected:.3f}, error={error:.3f}")
    if error > tolerance:
        raise AssertionError(f"{label} differs by {error:.3f}, tolerance {tolerance:.3f}")


def main():
    OUTPUT_BASE.mkdir(parents=True, exist_ok=True)

    ns3_dir = run_case("ns3_2x2_100m", "ns3", 2, 2)
    sionna_2x2_dir = run_case("sionna_2x2_100m", "sionna", 2, 2)
    sionna_8x8_dir = run_case("sionna_8x8_100m", "sionna", 8, 8)

    ns3_cov = load_coverage(ns3_dir)
    expected = fspl_db(3.5e9, float(ns3_cov["Distance_m"]))
    assert_close("coverage FSPL export", float(ns3_cov["FreeSpacePathloss_dB"]), expected, 0.05)

    ns3_link_loss = load_link_pathloss(ns3_dir)
    assert_close("ns-3 Friis propagation path loss", ns3_link_loss, expected, 1.0)

    sionna_2_cov = load_coverage(sionna_2x2_dir)
    sionna_8_cov = load_coverage(sionna_8x8_dir)
    assert_close(
        "Sionna coverage path loss 2x2 vs 8x8",
        float(sionna_8_cov["FreeSpacePathloss_dB"]),
        float(sionna_2_cov["FreeSpacePathloss_dB"]),
        0.01,
    )

    sionna_2_loss = load_link_pathloss(sionna_2x2_dir)
    sionna_8_loss = load_link_pathloss(sionna_8x8_dir)
    assert_close("Sionna scalar path loss near free-space budget", sionna_2_loss, expected, 6.0)
    print(f"Sionna scalar path loss: 2x2={sionna_2_loss:.3f} dB, 8x8={sionna_8_loss:.3f} dB")
    if abs(sionna_8_loss - sionna_2_loss) > 1.5:
        raise AssertionError("Sionna scalar path loss changed too much with array size")

    gain_2 = weighted_effective_channel_gain_db(sionna_2x2_dir)
    gain_8 = weighted_effective_channel_gain_db(sionna_8x8_dir)
    print(f"Sionna effective channel gain: 2x2={gain_2:.3f} dB, 8x8={gain_8:.3f} dB")
    if gain_8 <= gain_2 + 1.0:
        raise AssertionError("8x8 effective channel gain did not improve over 2x2 by at least 1 dB")

    print(f"Path-loss smoke test passed. Outputs: {OUTPUT_BASE}")


if __name__ == "__main__":
    main()
