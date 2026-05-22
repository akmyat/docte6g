#!/usr/bin/env bash

# Run warehouse communication and ISAC scenarios for square gNB arrays.
#
# Usage from ns3_server:
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh
#   scratch/warehouse-isac/run_warehouse_array_sweep.sh --sim-time 30

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_DIR="$(cd "${NS3_DIR}/.." && pwd)"
RESULTS_DIR="${REPO_DIR}/results/warehouse"
SIM_TIME=180
EXPAT_LIB="/home/aung/anaconda3/envs/6G/lib/libexpat.so"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim-time)
            SIM_TIME="$2"
            shift 2
            ;;
        --results-dir)
            RESULTS_DIR="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '3,8p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
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

cd "${NS3_DIR}"

for scenario in warehouse-no-isac warehouse-isac; do
    case "${scenario}" in
        warehouse-no-isac) result_prefix="no-isac" ;;
        warehouse-isac) result_prefix="isac" ;;
    esac

    for size in 2 4 8; do
        output_dir="${RESULTS_DIR}/${result_prefix}-${size}x${size}"
        mkdir -p "${output_dir}"

        echo
        echo ">>> ${scenario} gNB ${size}x${size}"
        echo "    results: ${output_dir}"

        ./ns3 run \
            "${scenario} --simTime=${SIM_TIME} --gnbAntennaRows=${size} --gnbAntennaCols=${size} --outputDir=${output_dir}" \
            2>&1 | tee "${output_dir}/run.log"
    done
done
