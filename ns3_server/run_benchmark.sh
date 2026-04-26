#!/usr/bin/env bash
# run_benchmark.sh
#
# Side-by-side wall-clock comparison:
#   - ns3sionna  (ZMQ-based Sionna, external Python server)
#   - sionnart   (pybind11-embedded Sionna, no server needed)
#
# Same scenario for both:
#   - 1 static gNB at (0, 0, 10)
#   - NUM_UES static UEs at increasing distances
#   - free_space Sionna scene
#   - Propagation loss + delay query every MEASURE_INTERVAL seconds for SIM_TIME seconds
#
# Usage:
#   ./run_benchmark.sh [NUM_UES] [SIM_TIME]
#
# Prerequisites:
#   - ns3 built:  ./ns3 configure && ./ns3 build
#   - For ns3sionna: ns3sionna Python server must be running:
#       cd contrib/ns3sionna/model/ns3sionna && python ns3sionna_server.py

set -euo pipefail

# ── Parameters ────────────────────────────────────────────────────────────────
NS3_ROOT="$(cd "$(dirname "$0")" && pwd)"
NUM_UES="${1:-4}"
SIM_TIME="${2:-10}"
MEASURE_INTERVAL="0.5"
ZMQ_URL="tcp://localhost:5555"
ASSETS_ROOT="/home/aung/code/new_docte6g/assets"
SIONNA_SERVER_DIR="${NS3_ROOT}/contrib/ns3sionna/model/ns3sionna"
RESULTS_DIR="/home/aung/code/new_docte6g/results"

mkdir -p "${RESULTS_DIR}"

TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
RESULT_FILE="${RESULTS_DIR}/benchmark_${TIMESTAMP}.txt"

echo "============================================================" | tee "${RESULT_FILE}"
echo "ns3sionna vs sionnart — propagation benchmark" | tee -a "${RESULT_FILE}"
echo "  Timestamp    : ${TIMESTAMP}" | tee -a "${RESULT_FILE}"
echo "  UEs          : ${NUM_UES}" | tee -a "${RESULT_FILE}"
echo "  Sim time     : ${SIM_TIME} s" | tee -a "${RESULT_FILE}"
echo "  Query interval: ${MEASURE_INTERVAL} s" | tee -a "${RESULT_FILE}"
echo "  Assets root  : ${ASSETS_ROOT}" | tee -a "${RESULT_FILE}"
echo "============================================================" | tee -a "${RESULT_FILE}"

# ── Helper: extract a labelled value from program output ──────────────────────
extract() {
    local label="$1" text="$2"
    echo "${text}" | grep "${label}" | awk -F':' '{print $2}' | xargs
}

# ── 1. sionnart (embedded Python — no server needed) ─────────────────────────
echo "" | tee -a "${RESULT_FILE}"
echo "--- sionnart (pybind11 embedded) ---" | tee -a "${RESULT_FILE}"
SIONNART_OUT=$(cd "${NS3_ROOT}" && ./ns3 run \
    "benchmark-sionnart \
     --numUes=${NUM_UES} \
     --simTime=${SIM_TIME} \
     --measureInterval=${MEASURE_INTERVAL} \
     --assetsRoot=${ASSETS_ROOT}" 2>&1)

echo "${SIONNART_OUT}" | tee -a "${RESULT_FILE}"
SIONNART_WALL=$(extract "Wall-clock time" "${SIONNART_OUT}")
SIONNART_QPQ=$(extract "Time per query" "${SIONNART_OUT}")

# ── 2. ns3sionna (ZMQ external Python server) ─────────────────────────────────
echo "" | tee -a "${RESULT_FILE}"
echo "--- ns3sionna (ZMQ + external Python server) ---" | tee -a "${RESULT_FILE}"

# Start the ns3sionna Python server if it's not already running
SERVER_PID=""
if ! nc -z localhost 5555 2>/dev/null; then
    echo "Starting ns3sionna Python server..." | tee -a "${RESULT_FILE}"
    if [ -f "${SIONNA_SERVER_DIR}/ns3sionna_server.py" ]; then
        (cd "${SIONNA_SERVER_DIR}" && conda run --no-capture-output -n 6Gold python ns3sionna_server.py) &
        SERVER_PID=$!
        sleep 15  # give the server time to load Sionna and GPU resources
        echo "Server started (PID ${SERVER_PID})" | tee -a "${RESULT_FILE}"
    else
        echo "WARNING: ns3sionna_server.py not found at ${SIONNA_SERVER_DIR}" | tee -a "${RESULT_FILE}"
        echo "         Start the server manually before running this benchmark." | tee -a "${RESULT_FILE}"
    fi
else
    echo "ns3sionna server already running on port 5555" | tee -a "${RESULT_FILE}"
fi

NS3SIONNA_OUT=$(cd "${NS3_ROOT}" && ./ns3 run \
    "benchmark-ns3sionna \
     --numUes=${NUM_UES} \
     --simTime=${SIM_TIME} \
     --measureInterval=${MEASURE_INTERVAL} \
     --zmqUrl=${ZMQ_URL} \
     --assetsRoot=${ASSETS_ROOT}" 2>&1) || true

echo "${NS3SIONNA_OUT}" | tee -a "${RESULT_FILE}"
NS3SIONNA_WALL=$(extract "Wall-clock time" "${NS3SIONNA_OUT}")
NS3SIONNA_QPQ=$(extract "Time per query" "${NS3SIONNA_OUT}")

# Clean up server we started
if [ -n "${SERVER_PID}" ]; then
    kill "${SERVER_PID}" 2>/dev/null || true
    echo "Stopped ns3sionna server (PID ${SERVER_PID})" | tee -a "${RESULT_FILE}"
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo "" | tee -a "${RESULT_FILE}"
echo "============================================================" | tee -a "${RESULT_FILE}"
echo "SUMMARY  (${NUM_UES} UEs, ${SIM_TIME} s sim, ${MEASURE_INTERVAL} s interval)" | tee -a "${RESULT_FILE}"
echo "------------------------------------------------------------" | tee -a "${RESULT_FILE}"
printf "%-35s  %s\n" "Implementation" "Wall-clock (s)    ms/query" | tee -a "${RESULT_FILE}"
printf "%-35s  %s\n" "-------------------------------" "----------------------------" | tee -a "${RESULT_FILE}"
printf "%-35s  %-18s  %s\n" "sionnart  (pybind11 embedded)" "${SIONNART_WALL:-N/A}"  "${SIONNART_QPQ:-N/A}" | tee -a "${RESULT_FILE}"
printf "%-35s  %-18s  %s\n" "ns3sionna (ZMQ + ext server)"  "${NS3SIONNA_WALL:-N/A}" "${NS3SIONNA_QPQ:-N/A}" | tee -a "${RESULT_FILE}"
echo "============================================================" | tee -a "${RESULT_FILE}"
echo "Full output saved to: ${RESULT_FILE}"
