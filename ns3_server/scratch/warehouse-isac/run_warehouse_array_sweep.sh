#!/usr/bin/env bash

# Compare communication-only and ISAC warehouse runs for 2x2, 4x4, and 8x8
# gNB arrays. The same topology, random seed, mobility, and offered load are
# used in every run.
#
# The default showcase profile is sized for a complete sweep within several hours:
# - 180 simulated seconds per run for the complete warehouse workflow
# - 500 KB showcase payload per mission; configurable gNB/UE power; 17 dB UE NF; 200 MHz bandwidth
# - identical beamforming period for both ISAC and no-ISAC; only sensing is the variable
# - direction-sensitive beamforming gain so stale beams affect robot links
# - lightweight sensing frames for ISAC; use --full-workflow for route-motion validation
# - deterministic robot routes by default so mobility is identical across runs
# - a hard 180-minute wall-clock limit for the complete sweep
# - "large" geometry profile (100x70m, gNB SW corner) enabled by --geometry-profile large
#
# Usage from ns3_server:
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --quick
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --full-workflow
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --sim-time 120
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --geometry-profile large

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_DIR="$(cd "${NS3_DIR}/.." && pwd)"

RESULTS_DIR="${REPO_DIR}/results/warehouse"
GEOMETRY_PROFILE="baseline"
SIM_TIME=180
ISAC_FRAME_INTERVAL=5
ISAC_SAMPLES_PER_SRC=2000000
ISAC_BEAMWIDTH_DEG=20
SIONNA_FIXED_UL_MCS=3
GNB_TX_POWER_DBM=24
UE_TX_POWER_DBM=23
GNB_NOISE_FIGURE_DB=5
UE_NOISE_FIGURE_DB=17
BEAMFORMING_PERIOD=1
SHOWCASE_BYTES=131072
PACING_RATE_BPS=50000000
IDEAL_ANALOG_ARRAY_GAIN=true
ISAC_MIN_POWER=1e-25
USE_ELEMENT_MIMO_CSI=true
BEAMFORMING_METHOD="ns3::DirectPathBeamforming"
MAX_SWEEP_MINUTES=360
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
            [[ ${SIM_TIME} -lt 180 ]] && SIM_TIME=180
            ISAC_FRAME_INTERVAL=2
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
        --beamforming-period)
            BEAMFORMING_PERIOD="$2"
            shift 2
            ;;
        --showcase-bytes)
            SHOWCASE_BYTES="$2"
            shift 2
            ;;
        --ideal-analog-array-gain)
            IDEAL_ANALOG_ARRAY_GAIN="$2"
            shift 2
            ;;
        --geometry-profile)
            GEOMETRY_PROFILE="$2"
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

# The ns-3 binary embeds libpython from the conda env.  Without PYTHONHOME
# pointing at the same env, Python's stdlib search falls back to the system
# Python paths, causing an ABI mismatch when numpy's C extensions try to
# import the datetime capsule.
export PYTHONHOME="${CONDA_PREFIX}"
export PYTHONPATH="${CONDA_PREFIX}/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

run_sweep() {
    cd "${NS3_DIR}"
    for scenario in warehouse-no-isac warehouse-isac; do
        local result_prefix="no-isac"
        [[ "${scenario}" == "warehouse-isac" ]] && result_prefix="isac"

        for size in 2 4 8; do
            local output_dir="${RESULTS_DIR}/${result_prefix}-${size}x${size}"
            mkdir -p "${output_dir}"

            # Scale beamforming period inversely with array size: narrower beams need
            # faster updates to keep moving robots within the main lobe.
            local beam_period
            case "${size}" in
                2) beam_period=1.0 ;;
                4) beam_period=0.5 ;;
                8) beam_period=0.25 ;;
                *) beam_period="${BEAMFORMING_PERIOD}" ;;
            esac

            echo
            echo ">>> ${scenario} gNB ${size}x${size} (beamPeriod=${beam_period}s)"
            echo "    results: ${output_dir}"

            local scenario_args=(
                "--simTime=${SIM_TIME}"
                "--gnbAntennaRows=${size}"
                "--gnbAntennaCols=${size}"
                "--sionnaFixedUlMcs=${SIONNA_FIXED_UL_MCS}"
                "--gnbTxPowerDbm=${GNB_TX_POWER_DBM}"
                "--ueTxPowerDbm=${UE_TX_POWER_DBM}"
                "--gnbNoiseFigureDb=${GNB_NOISE_FIGURE_DB}"
                "--ueNoiseFigureDb=${UE_NOISE_FIGURE_DB}"
                "--idealAnalogArrayGain=${IDEAL_ANALOG_ARRAY_GAIN}"
                "--useElementMimoCsi=${USE_ELEMENT_MIMO_CSI}"
                "--beamformingMethod=${BEAMFORMING_METHOD}"
                "--showcaseBytes=${SHOWCASE_BYTES}"
                "--pacingRateBps=${PACING_RATE_BPS}"
                "--geometryProfile=${GEOMETRY_PROFILE}"
                "--outputDir=${output_dir}"
            )
            scenario_args+=(
                "--beamformingPeriodicity=${beam_period}"
            )
            if [[ "${scenario}" == "warehouse-isac" ]]; then
                scenario_args+=(
                    "--isacSensingFrameInterval=${ISAC_FRAME_INTERVAL}"
                    "--isacBeamwidthDeg=${ISAC_BEAMWIDTH_DEG}"
                    "--isacSamplesPerSrc=${ISAC_SAMPLES_PER_SRC}"
                    "--isacMinPower=${ISAC_MIN_POWER}"
                )
            fi

            ./ns3 run "${scenario} ${scenario_args[*]}" 2>&1 | tee "${output_dir}/run.log"
        done
    done
    python3 "${SCRIPT_DIR}/analyze_warehouse_sweep.py" "${RESULTS_DIR}"
}

export -f run_sweep
export SCRIPT_DIR NS3_DIR RESULTS_DIR SIM_TIME
export ISAC_FRAME_INTERVAL ISAC_SAMPLES_PER_SRC ISAC_BEAMWIDTH_DEG SIONNA_FIXED_UL_MCS
export GNB_TX_POWER_DBM UE_TX_POWER_DBM GNB_NOISE_FIGURE_DB UE_NOISE_FIGURE_DB
export BEAMFORMING_PERIOD IDEAL_ANALOG_ARRAY_GAIN ISAC_MIN_POWER USE_ELEMENT_MIMO_CSI SHOWCASE_BYTES PACING_RATE_BPS GEOMETRY_PROFILE BEAMFORMING_METHOD

timeout --foreground "${MAX_SWEEP_MINUTES}m" bash -c run_sweep
