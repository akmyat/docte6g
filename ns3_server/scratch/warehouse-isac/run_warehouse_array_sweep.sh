#!/usr/bin/env bash

# Compare communication-only and ISAC warehouse runs for 2x2, 4x4, and 8x8
# gNB arrays. The same topology, random seed, mobility, and offered load are
# used in every run.
#
# The default showcase profile is sized for a complete sweep within 2-3 hours:
# - 30 simulated seconds per run for propagation and application-flow metrics
# - interference-stressed downlink probe traffic over the V3 warehouse geometry at 30 dBm gNB power
# - stale communication-only beam tracking for no-ISAC mobile UEs
# - faster sensing-assisted beam tracking for ISAC mobile UEs
# - direction-sensitive beamforming gain so stale beams affect robot links
# - lightweight sensing frames for ISAC; use --full-workflow for route-motion validation
# - deterministic robot routes by default so mobility is identical across runs
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
CHALLENGE_PACKET_SIZE_BYTES=1200
CHALLENGE_START_SEC=-1
ROBOT_ROUTE_START_SEC=-1
ROBOT_DROP_START_SEC=-1
ENABLE_UPLINK=false
ENABLE_WAREHOUSE_WORKFLOW=false
ISAC_FRAME_INTERVAL=2
ISAC_SAMPLES_PER_SRC=20000
ISAC_BEAMWIDTH_DEG=20
SIONNA_FIXED_UL_MCS=3
GNB_TX_POWER_DBM=30
UE_TX_POWER_DBM=23
GNB_NOISE_FIGURE_DB=5
UE_NOISE_FIGURE_DB=7
NO_ISAC_BEAMFORMING_PERIOD=10
ISAC_BEAMFORMING_PERIOD=1
IDEAL_ANALOG_ARRAY_GAIN=false
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
            ISAC_FRAME_INTERVAL=2
            ENABLE_WAREHOUSE_WORKFLOW=true
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
        --challenge-packet-size-bytes)
            CHALLENGE_PACKET_SIZE_BYTES="$2"
            shift 2
            ;;
        --challenge-start)
            CHALLENGE_START_SEC="$2"
            shift 2
            ;;
        --robot-route-start)
            ROBOT_ROUTE_START_SEC="$2"
            shift 2
            ;;
        --robot-drop-start)
            ROBOT_DROP_START_SEC="$2"
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
        --enable-warehouse-workflow)
            ENABLE_WAREHOUSE_WORKFLOW=true
            shift
            ;;
        --disable-warehouse-workflow)
            ENABLE_WAREHOUSE_WORKFLOW=false
            shift
            ;;
        --isac-frame-interval)
            ISAC_FRAME_INTERVAL="$2"
            shift 2
            ;;
        --isac-beamwidth-deg)
            ISAC_BEAMWIDTH_DEG="$2"
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
        --gnb-noise-figure-db)
            GNB_NOISE_FIGURE_DB="$2"
            shift 2
            ;;
        --ue-noise-figure-db)
            UE_NOISE_FIGURE_DB="$2"
            shift 2
            ;;
        --no-isac-beamforming-period)
            NO_ISAC_BEAMFORMING_PERIOD="$2"
            shift 2
            ;;
        --isac-beamforming-period)
            ISAC_BEAMFORMING_PERIOD="$2"
            shift 2
            ;;
        --ideal-analog-array-gain)
            IDEAL_ANALOG_ARRAY_GAIN="$2"
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
                "--enableWarehouseWorkflow=${ENABLE_WAREHOUSE_WORKFLOW}"
                "--challengePacketIntervalUs=${DL_PACKET_INTERVAL_US}"
                "--challengeUlPacketIntervalUs=${UL_PACKET_INTERVAL_US}"
                "--challengePacketSizeBytes=${CHALLENGE_PACKET_SIZE_BYTES}"
                "--challengeStart=${CHALLENGE_START_SEC}"
                "--robotRouteStart=${ROBOT_ROUTE_START_SEC}"
                "--robotDropStart=${ROBOT_DROP_START_SEC}"
                "--sionnaFixedUlMcs=${SIONNA_FIXED_UL_MCS}"
                "--gnbTxPowerDbm=${GNB_TX_POWER_DBM}"
                "--ueTxPowerDbm=${UE_TX_POWER_DBM}"
                "--gnbNoiseFigureDb=${GNB_NOISE_FIGURE_DB}"
                "--ueNoiseFigureDb=${UE_NOISE_FIGURE_DB}"
                "--idealAnalogArrayGain=${IDEAL_ANALOG_ARRAY_GAIN}"
                "--outputDir=${output_dir}"
            )
            if [[ "${scenario}" == "warehouse-isac" ]]; then
                scenario_args+=(
                    "--beamformingPeriodicity=${ISAC_BEAMFORMING_PERIOD}"
                    "--isacSensingFrameInterval=${ISAC_FRAME_INTERVAL}"
                    "--isacBeamwidthDeg=${ISAC_BEAMWIDTH_DEG}"
                    "--isacSamplesPerSrc=${ISAC_SAMPLES_PER_SRC}"
                )
            else
                scenario_args+=(
                    "--beamformingPeriodicity=${NO_ISAC_BEAMFORMING_PERIOD}"
                )
            fi

            ./ns3 run "${scenario} ${scenario_args[*]}" 2>&1 | tee "${output_dir}/run.log"
        done
    done
    python3 "${SCRIPT_DIR}/analyze_warehouse_sweep.py" "${RESULTS_DIR}"
}

export -f run_sweep
export SCRIPT_DIR NS3_DIR RESULTS_DIR SIM_TIME DL_PACKET_INTERVAL_US UL_PACKET_INTERVAL_US
export CHALLENGE_PACKET_SIZE_BYTES CHALLENGE_START_SEC ROBOT_ROUTE_START_SEC ROBOT_DROP_START_SEC
export ENABLE_UPLINK ENABLE_WAREHOUSE_WORKFLOW ISAC_FRAME_INTERVAL ISAC_SAMPLES_PER_SRC ISAC_BEAMWIDTH_DEG SIONNA_FIXED_UL_MCS
export GNB_TX_POWER_DBM UE_TX_POWER_DBM GNB_NOISE_FIGURE_DB UE_NOISE_FIGURE_DB
export NO_ISAC_BEAMFORMING_PERIOD ISAC_BEAMFORMING_PERIOD IDEAL_ANALOG_ARRAY_GAIN

timeout --foreground "${MAX_SWEEP_MINUTES}m" bash -c run_sweep
