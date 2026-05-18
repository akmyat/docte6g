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
ASSETS_ROOT="/home/aung/code/docte6g/assets"
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

# sionnart embeds Python via pybind11::scoped_interpreter, which doesn't set
# PyConfig.home — libpython falls back to compile-time defaults and can't
# locate the conda env's stdlib (manifests as "PyCapsule_Import datetime" on
# numpy import). We pass the active env's prefix to the benchmark binary
# *per-invocation* (not via a global export) so it doesn't leak into
# server.sh, which activates a different conda env (6Gold, Python 3.10) and
# would otherwise crash with "No module named 'encodings'".
SIONNART_PYTHONHOME="${CONDA_PREFIX:-/home/aung/anaconda3/envs/6G}"

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
    echo "backend,regime,num_stas,wall_clock_s,sim_mem_usage_mib,peak_util_percent,mean_util_percent,peak_power_w,peak_sm_clock_mhz" > "${RESULTS_CSV}"
fi
# Schema notes:
#  - util/memory are per-process (filtered to the benchmark binary).
#  - peak_power_w / peak_sm_clock_mhz are GPU-wide (only the benchmark uses
#    the GPU on this box). They exist because on Hopper/Blackwell hardware
#    `utilization.gpu` undercounts short kernel bursts — power draw and SM
#    clock are far more honest signals (idle ~3.7 W / 214 MHz; busy spikes
#    to 10+ W / 2400 MHz even when util.gpu is still reporting 0).
#  - Old rows written under the prior schema are not directly comparable.

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
            # Start per-process GPU monitoring in background, filtered by the
            # *PID* of the actual benchmark process (not by command name, which
            # nvidia-smi truncates to 15 chars and which doesn't match the
            # ns3.<ver>-<binary>-<profile> on-disk name anyway).
            #
            # We don't know the PID until ./ns3 run has spawned the child, so
            # the samplers poll `pgrep -f` for the binary basename until they
            # see one, then start filtering nvidia-smi output.
            #
            #   1) pmon -s u -d 1   -> per-process SM util at 1 Hz (driver min)
            #   2) query-compute-apps loop @ 0.5 s -> per-process FB memory MiB
            TMP_PMON_FILE=$(mktemp)
            TMP_UTIL_FILE=$(mktemp)
            TMP_MEM_FILE=$(mktemp)
            TMP_PIDS_FILE=$(mktemp)

            # Backend-specific list of pgrep patterns. For ns3sionna the GPU
            # work happens in the persistent ZMQ Python server, not in the
            # benchmark binary itself, so we must include the server's
            # process pattern. Otherwise sim_mem_usage stays 0.
            #
            # We report *peak resident GPU memory* across this PID set
            # (no baseline subtraction): for sionnart that's the per-run
            # allocation (~465 MiB); for ns3sionna it's the server's loaded
            # model + simulation state (~2 GiB) — which is the GPU footprint
            # an operator cares about. Subtracting a baseline would drop
            # ns3sionna to ~0 because the server pre-allocates once at
            # startup and reuses buffers, so per-run delta is negligible
            # even though the backend actively occupies the GPU.
            case "${backend}" in
                ns3sionna) target_patterns=("${binary}" "ns3sionna_server.py") ;;
                sionnart)  target_patterns=("${binary}") ;;
                pure_ns3)  target_patterns=("${binary}") ;;
                *)         target_patterns=("${binary}") ;;
            esac

            # Per-process SM util at 1 Hz (driver minimum) — used for the
            # *mean* utilization, which is per-process-attributed.
            nvidia-smi pmon -d 1 -s u -c 100000 > "${TMP_PMON_FILE}" 2>/dev/null &
            PMON_PID=$!

            # GPU-wide telemetry at 10 Hz. We capture utilization.gpu *and*
            # power.draw + clocks.current.sm because on Hopper/Blackwell the
            # util.gpu metric is sampled too coarsely to register short kernel
            # bursts (idle util can read 0 while SM clock jumps to boost and
            # power draw triples). Power and clock are honest signals; util
            # is kept for backwards-compat with the old schema.
            nvidia-smi --query-gpu=utilization.gpu,power.draw,clocks.current.sm \
                       --format=csv,noheader,nounits --loop-ms=100 \
                       > "${TMP_UTIL_FILE}" 2>/dev/null &
            UTIL_PID=$!

            # Persist target_patterns into a file so the background subshell
            # can read them without inheriting an array.
            TMP_PATTERNS_FILE=$(mktemp)
            printf '%s\n' "${target_patterns[@]}" > "${TMP_PATTERNS_FILE}"

            (
                while true; do
                    # Union of live PIDs matching any target pattern, persisted
                    # so brief children aren't lost between samples.
                    while IFS= read -r pat; do
                        [[ -z "${pat}" ]] && continue
                        pgrep -f "${pat}" 2>/dev/null >> "${TMP_PIDS_FILE}" || true
                    done < "${TMP_PATTERNS_FILE}"
                    seen_pids=$(sort -u "${TMP_PIDS_FILE}" | tr '\n' '|' | sed 's/|$//')
                    if [[ -n "${seen_pids}" ]]; then
                        nvidia-smi --query-compute-apps=pid,used_memory \
                            --format=csv,noheader,nounits 2>/dev/null \
                            | awk -F',' -v pids="${seen_pids}" '
                                BEGIN { n=split(pids, arr, "|"); for (i=1;i<=n;i++) want[arr[i]]=1 }
                                { gsub(/^[ \t]+|[ \t]+$/, "", $1); gsub(/^[ \t]+|[ \t]+$/, "", $2) }
                                ($1 in want) { sum += $2+0 }
                                END { print sum+0 }
                              ' >> "${TMP_MEM_FILE}"
                    else
                        echo 0 >> "${TMP_MEM_FILE}"
                    fi
                    sleep 0.5
                done
            ) &
            MEM_PID=$!

            # Only sionnart needs PYTHONHOME (it embeds Python). For ns3sionna
            # / pure_ns3, leaving PYTHONHOME unset is critical so unrelated
            # subprocesses (e.g. the ns3sionna ZMQ server, which uses a
            # different conda env) aren't broken by a stale PYTHONHOME.
            run_rc=0
            if [[ "${backend}" == "sionnart" ]]; then
                log="$(env "PYTHONHOME=${SIONNART_PYTHONHOME}" ./ns3 run "${cmd}" 2>&1)" || run_rc=$?
            else
                log="$(env -u PYTHONHOME ./ns3 run "${cmd}" 2>&1)" || run_rc=$?
            fi
            if [[ "${run_rc}" -ne 0 ]]; then
                kill "${PMON_PID}" "${UTIL_PID}" "${MEM_PID}" 2>/dev/null || true
                wait "${PMON_PID}" "${UTIL_PID}" "${MEM_PID}" 2>/dev/null || true
                rm -f "${TMP_PMON_FILE}" "${TMP_UTIL_FILE}" "${TMP_MEM_FILE}" "${TMP_PIDS_FILE}" "${TMP_PATTERNS_FILE}"
                echo "    ERROR (rc=${run_rc}), skipping. Last 20 lines:"
                echo "${log}" | tail -20 | sed 's/^/      /'
                n=$(( n * 2 ))
                continue
            fi

            kill "${PMON_PID}" "${UTIL_PID}" "${MEM_PID}" 2>/dev/null || true
            wait "${PMON_PID}" "${UTIL_PID}" "${MEM_PID}" 2>/dev/null || true

            # Peak from the 10 Hz GPU-wide sampler. Columns: util%, power_W, sm_MHz.
            # awk uses ", " as field separator (csv format from nvidia-smi).
            peak_util=$(awk -F', *' 'BEGIN{p=0} NF>=3 && $1 ~ /^[0-9.]+$/ { if ($1+0 > p) p=$1+0 } END{print p+0}' "${TMP_UTIL_FILE}")
            peak_power=$(awk -F', *' 'BEGIN{p=0} NF>=3 && $2 ~ /^[0-9.]+$/ { if ($2+0 > p) p=$2+0 } END{printf "%.2f", p}' "${TMP_UTIL_FILE}")
            peak_sm_clock=$(awk -F', *' 'BEGIN{p=0} NF>=3 && $3 ~ /^[0-9.]+$/ { if ($3+0 > p) p=$3+0 } END{print p+0}' "${TMP_UTIL_FILE}")

            # Mean from pmon (per-process attribution; excludes idle GPU time
            # before the benchmark process attached and any unrelated GPU
            # activity from other PIDs).
            # pmon -s u columns: "# gpu  pid  type  sm  mem  enc  dec  command"
            seen_pids=$(sort -u "${TMP_PIDS_FILE}" 2>/dev/null | tr '\n' ',' | sed 's/,$//')

            mean_util=$(awk -v pids="${seen_pids}" '
                BEGIN { n=split(pids, arr, ","); for (i=1;i<=n;i++) want[arr[i]]=1 }
                /^#/ { next }
                ($2 in want) { sum += $4+0; cnt++ }
                END { if (cnt > 0) printf "%.2f", sum/cnt; else print "0.00" }
            ' "${TMP_PMON_FILE}")

            # Peak resident GPU memory (MiB) summed across the backend's PID
            # set during the run. Reported as-is, with no baseline subtracted:
            # this is the GPU footprint the backend actively occupies, which
            # is what we want to compare across backends.
            sim_mem_usage=$(awk 'BEGIN{m=0} {if ($1+0 > m) m=$1+0} END{print m+0}' "${TMP_MEM_FILE}")

            rm -f "${TMP_PMON_FILE}" "${TMP_UTIL_FILE}" "${TMP_MEM_FILE}" "${TMP_PIDS_FILE}" "${TMP_PATTERNS_FILE}"

            result_line="$(echo "${log}" | grep '^RESULT ' | tail -1 || true)"
            if [[ -z "${result_line}" ]]; then
                echo "    no RESULT line, skipping"
                n=$(( n * 2 ))
                continue
            fi

            wall=$(echo "${result_line}" | sed -n 's/.*wall_clock_s=\([0-9.eE+-]*\).*/\1/p')

            # The binaries print wall_clock_s=-1 when their RunSimulation()
            # bails out (e.g., SionnaInitialize failed). Don't record garbage:
            # surface the last 30 stderr lines so the operator sees the cause.
            if awk "BEGIN{exit !(${wall} <= 0)}"; then
                echo "    sim returned wall_clock_s=${wall} (failure sentinel). Last 30 lines:"
                echo "${log}" | tail -30 | sed 's/^/      /'
                n=$(( n * 2 ))
                continue
            fi

            echo "    wall_clock_s=${wall} | sim_mem_usage=${sim_mem_usage} MiB | peak_util=${peak_util}% | mean_util=${mean_util}% | peak_power=${peak_power} W | peak_sm_clock=${peak_sm_clock} MHz"
            echo "${backend},${regime},${n},${wall},${sim_mem_usage},${peak_util},${mean_util},${peak_power},${peak_sm_clock}" >> "${RESULTS_CSV}"

            n=$(( n * 2 ))
        done
    done
done

echo
echo "=== done. ${RESULTS_CSV} ==="
