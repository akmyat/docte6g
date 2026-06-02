#!/usr/bin/env bash

# Compare communication-only and ISAC warehouse runs for 2x2, 4x4, and 8x8
# gNB arrays. The same topology, random seed, mobility, and offered load are
# used in every run.
#
# The default showcase profile is sized for a complete sweep within 2-3 hours:
# - 30 simulated seconds per run for propagation and application-flow metrics
# - downlink probe traffic and a link-margin stress power to expose gNB array gain
# - six lightweight sensing frames per ISAC run, including post-start robot motion
# - a hard 180-minute wall-clock limit for the complete sweep
#
# Usage from ns3_server:
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --quick
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --full-workflow
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --sim-time 120

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_DIR="$(cd "${NS3_DIR}/.." && pwd)"

RESULTS_DIR="${REPO_DIR}/results/warehouse"
SIM_TIME=30
DL_PACKET_INTERVAL_US=1000
UL_PACKET_INTERVAL_US=2500
ENABLE_UPLINK=false
ISAC_FRAME_INTERVAL=5
ISAC_SAMPLES_PER_SRC=20000
SIONNA_FIXED_UL_MCS=3
GNB_TX_POWER_DBM=-40
UE_TX_POWER_DBM=23
MAX_SWEEP_MINUTES=180
EXPAT_LIB="/home/aung/anaconda3/envs/6G/lib/libexpat.so"

usage() {
    sed -n '3,17p' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --quick)
            SIM_TIME=10
            ISAC_FRAME_INTERVAL=10
            ISAC_SAMPLES_PER_SRC=5000
            shift
            ;;
        --full-workflow)
            SIM_TIME=120
            ISAC_FRAME_INTERVAL=5
            shift
            ;;
        --sim-time)
            SIM_TIME="$2"
            shift 2
            ;;
        --results-dir)
            RESULTS_DIR="$2"
            shift 2
            ;;
        --dl-packet-interval-us)
            DL_PACKET_INTERVAL_US="$2"
            shift 2
            ;;
        --ul-packet-interval-us)
            UL_PACKET_INTERVAL_US="$2"
            shift 2
            ;;
        --downlink-only)
            ENABLE_UPLINK=false
            shift
            ;;
        --bidirectional)
            ENABLE_UPLINK=true
            shift
            ;;
        --isac-frame-interval)
            ISAC_FRAME_INTERVAL="$2"
            shift 2
            ;;
        --isac-samples-per-src)
            ISAC_SAMPLES_PER_SRC="$2"
            shift 2
            ;;
        --sionna-fixed-ul-mcs)
            SIONNA_FIXED_UL_MCS="$2"
            shift 2
            ;;
        --gnb-tx-power-dbm)
            GNB_TX_POWER_DBM="$2"
            shift 2
            ;;
        --ue-tx-power-dbm)
            UE_TX_POWER_DBM="$2"
            shift 2
            ;;
        --max-sweep-minutes)
            MAX_SWEEP_MINUTES="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "${CONDA_PREFIX:-}" ]]; then
    echo "Activate the 6G conda environment before running this script." >&2
    exit 2
fi

if [[ -f "${EXPAT_LIB}" ]]; then
    export LD_PRELOAD="${EXPAT_LIB}${LD_PRELOAD:+:${LD_PRELOAD}}"
fi

run_sweep() {
    cd "${NS3_DIR}"
    for scenario in warehouse-no-isac warehouse-isac; do
        local result_prefix="no-isac"
        [[ "${scenario}" == "warehouse-isac" ]] && result_prefix="isac"

        for size in 2 4 8; do
            local output_dir="${RESULTS_DIR}/${result_prefix}-${size}x${size}"
            mkdir -p "${output_dir}"

            echo
            echo ">>> ${scenario} gNB ${size}x${size}"
            echo "    results: ${output_dir}"

            local scenario_args=(
                "--simTime=${SIM_TIME}"
                "--gnbAntennaRows=${size}"
                "--gnbAntennaCols=${size}"
                "--enableChallengeTraffic=true"
                "--challengeEnableUl=${ENABLE_UPLINK}"
                "--challengePacketIntervalUs=${DL_PACKET_INTERVAL_US}"
                "--challengeUlPacketIntervalUs=${UL_PACKET_INTERVAL_US}"
                "--sionnaFixedUlMcs=${SIONNA_FIXED_UL_MCS}"
                "--gnbTxPowerDbm=${GNB_TX_POWER_DBM}"
                "--ueTxPowerDbm=${UE_TX_POWER_DBM}"
                "--outputDir=${output_dir}"
            )
            if [[ "${scenario}" == "warehouse-isac" ]]; then
                scenario_args+=(
                    "--isacSensingFrameInterval=${ISAC_FRAME_INTERVAL}"
                    "--isacSamplesPerSrc=${ISAC_SAMPLES_PER_SRC}"
                )
            fi

            ./ns3 run "${scenario} ${scenario_args[*]}" 2>&1 | tee "${output_dir}/run.log"
        done
    done
    python3 "${SCRIPT_DIR}/analyze_warehouse_sweep.py" "${RESULTS_DIR}"
}

export -f run_sweep
export SCRIPT_DIR NS3_DIR RESULTS_DIR SIM_TIME DL_PACKET_INTERVAL_US UL_PACKET_INTERVAL_US
export ENABLE_UPLINK ISAC_FRAME_INTERVAL ISAC_SAMPLES_PER_SRC SIONNA_FIXED_UL_MCS
export GNB_TX_POWER_DBM UE_TX_POWER_DBM

timeout --foreground "${MAX_SWEEP_MINUTES}m" bash -c run_sweep
