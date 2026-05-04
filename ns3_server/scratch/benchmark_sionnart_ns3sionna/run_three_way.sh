#!/usr/bin/env bash
# Three-way benchmark sweep driver.
#
# Sweeps N STAs x 3 workload regimes x selected backends, parses the
# per-run "RESULT ..." line each binary prints, appends to results.csv.
#
# Operator notes:
#   - ns3sionna requires the Python server running first in conda env 6Gold:
#       conda activate 6Gold
#       cd ns3_server/contrib/sionna/model/ns3sionna && ./run_python_proto.sh
#   - sionnart requires this script to run under conda env 6G (the embedded
#     Python interpreter inherits the shell's env):
#       conda activate 6G && ./run_three_way.sh
#   - pure_ns3 needs no Python.
#
# Usage:
#   ./run_three_way.sh                       # all 3 backends, all regimes, full sweep
#   ./run_three_way.sh --backends pure_ns3,sionnart
#   ./run_three_way.sh --regimes stationary_high_load
#   ./run_three_way.sh --max-stas 16
#   ./run_three_way.sh --sim-seconds 5

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RESULTS_CSV="${SCRIPT_DIR}/results.csv"
SERVER_SH="${SCRIPT_DIR}/server.sh"
CONDA_SH=/home/aung/anaconda3/etc/profile.d/conda.sh

BACKENDS="pure_ns3,ns3sionna,sionnart"
REGIMES="stationary_high_load,low_mob_low_load,high_mob_high_load"
MAX_STAS=32
SIM_SECONDS=9
ZMQ_URL="tcp://localhost:5555"
ASSETS_ROOT="/home/aung/code/new_docte6g/assets"
AUTO_SERVER=1   # auto-start ns3sionna ZMQ server when ns3sionna is in BACKENDS

while [[ $# -gt 0 ]]; do
    case "$1" in
        --backends)      BACKENDS="$2"; shift 2 ;;
        --regimes)       REGIMES="$2"; shift 2 ;;
        --max-stas)      MAX_STAS="$2"; shift 2 ;;
        --sim-seconds)   SIM_SECONDS="$2"; shift 2 ;;
        --zmq-url)       ZMQ_URL="$2"; shift 2 ;;
        --assets-root)   ASSETS_ROOT="$2"; shift 2 ;;
        --no-auto-server) AUTO_SERVER=0; shift ;;
        -h|--help)
            sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "Unknown arg: $1"; exit 2 ;;
    esac
done

# Activate 6G for sionnart's site-packages and so libpython3.12 deps are visible.
# (server.sh activates 6Gold internally for the ZMQ server; doesn't affect this shell.)
# shellcheck disable=SC1090
source "${CONDA_SH}" && conda activate 6G

# Auto-start ns3sionna ZMQ server if needed
if [[ ",${BACKENDS}," == *",ns3sionna,"* ]] && [[ "${AUTO_SERVER}" == "1" ]]; then
    if ! "${SERVER_SH}" status >/dev/null 2>&1; then
        echo "[run_three_way.sh] starting ns3sionna server (6Gold)..."
        "${SERVER_SH}" start
    else
        echo "[run_three_way.sh] ns3sionna server already running"
    fi
fi

regime_args() {
    case "$1" in
        stationary_high_load) echo "--mobile_scenario=false --mobile_speed=0.0 --udp_pkt_interval=20" ;;
        low_mob_low_load)     echo "--mobile_scenario=true  --mobile_speed=1.0 --udp_pkt_interval=1000" ;;
        high_mob_high_load)   echo "--mobile_scenario=true  --mobile_speed=7.0 --udp_pkt_interval=20" ;;
        *) echo "Unknown regime: $1" >&2; exit 2 ;;
    esac
}

backend_extra_args() {
    case "$1" in
        ns3sionna) echo "--zmqUrl=${ZMQ_URL} --environment=/home/aung/code/new_docte6g/ns3_server/contrib/ns3sionna/model/ns3sionna/models/free_space/free_space.xml" ;;
        sionnart)  echo "--assetsRoot=${ASSETS_ROOT}" ;;
        pure_ns3)  echo "" ;;
        *) echo "Unknown backend: $1" >&2; exit 2 ;;
    esac
}

if [[ ! -f "${RESULTS_CSV}" ]]; then
    echo "backend,regime,num_stas,wall_clock_s,sim_mem_usage_mib,peak_util_percent" > "${RESULTS_CSV}"
fi

cd "${NS3_DIR}"

IFS=',' read -ra BACKEND_LIST <<< "${BACKENDS}"
IFS=',' read -ra REGIME_LIST  <<< "${REGIMES}"

echo "=== three-way benchmark sweep ==="
echo "  backends : ${BACKENDS}"
echo "  regimes  : ${REGIMES}"
echo "  max_stas : ${MAX_STAS}"
echo "  sim_s    : ${SIM_SECONDS}"
echo "  results  : ${RESULTS_CSV}"
echo

for backend in "${BACKEND_LIST[@]}"; do
    binary="${backend}_benchmark"
    extra="$(backend_extra_args "${backend}")"

    for regime in "${REGIME_LIST[@]}"; do
        rargs="$(regime_args "${regime}")"

        n=1
        while (( n <= MAX_STAS )); do
            cmd="${binary} --num_stas=${n} ${rargs} --sim_seconds=${SIM_SECONDS} --regime=${regime} ${extra}"
            echo ">>> ${backend} | regime=${regime} | N=${n}"
            if grep -q "^${backend},${regime},${n}," "${RESULTS_CSV}" 2>/dev/null; then
                echo "    already in ${RESULTS_CSV}, skipping"
                n=$(( n * 2 ))
                continue
            fi
            # Start GPU monitoring in background
            TMP_GPU_FILE=$(mktemp)
            # Capture baseline compute memory
            base_mem=$(nvidia-smi --query-compute-apps=used_memory --format=csv,noheader,nounits 2>/dev/null | awk '{sum+=$1} END {print sum+0}')
            echo "${base_mem} 0" > "${TMP_GPU_FILE}"
            (
                peak_mem=${base_mem}
                peak_util=0
                while true; do
                    # Get utilization
                    curr_util=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null || echo 0)
                    # Get compute app memory (summed)
                    curr_mem=$(nvidia-smi --query-compute-apps=used_memory --format=csv,noheader,nounits 2>/dev/null | awk '{sum+=$1} END {print sum+0}')
                    
                    # Handle non-numeric or empty output
                    [[ "$curr_util" =~ ^[0-9.]+$ ]] || curr_util=0
                    
                    # Update peaks
                    if (( $(echo "$curr_mem > $peak_mem" | bc -l 2>/dev/null || [ "$curr_mem" -gt "$peak_mem" ]) )); then peak_mem=$curr_mem; fi
                    if (( $(echo "$curr_util > $peak_util" | bc -l 2>/dev/null || [ "$curr_util" -gt "$peak_util" ]) )); then peak_util=$curr_util; fi
                    
                    echo "${peak_mem} ${peak_util}" > "${TMP_GPU_FILE}"
                    sleep 0.5
                done
            ) &
            MONITOR_PID=$!

            log="$(./ns3 run "${cmd}" 2>&1)" || {
                kill "${MONITOR_PID}" || true
                rm -f "${TMP_GPU_FILE}"
                echo "    ERROR (rc=$?), skipping. Last 20 lines:"
                echo "${log}" | tail -20 | sed 's/^/      /'
                n=$(( n * 2 ))
                continue
            }

            kill "${MONITOR_PID}" || true
            read -r peak_mem peak_util < "${TMP_GPU_FILE}"
            rm -f "${TMP_GPU_FILE}"

            # Calculate simulation usage (delta)
            sim_mem_usage=$(echo "${peak_mem} - ${base_mem}" | bc)
            # Ensure it's not negative (can happen if another app closes during run)
            if (( $(echo "${sim_mem_usage} < 0" | bc -l) )); then sim_mem_usage=0; fi

            result_line="$(echo "${log}" | grep '^RESULT ' | tail -1 || true)"
            if [[ -z "${result_line}" ]]; then
                echo "    no RESULT line, skipping"
                n=$(( n * 2 ))
                continue
            fi

            wall=$(echo "${result_line}" | sed -n 's/.*wall_clock_s=\([0-9.eE+-]*\).*/\1/p')
            echo "    wall_clock_s=${wall} | sim_mem_usage=${sim_mem_usage} MiB | peak_util=${peak_util}%"
            echo "${backend},${regime},${n},${wall},${sim_mem_usage},${peak_util}" >> "${RESULTS_CSV}"

            n=$(( n * 2 ))
        done
    done
done

echo
echo "=== done. ${RESULTS_CSV} ==="
