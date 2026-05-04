#!/usr/bin/env bash
# Single-shot benchmark wrapper. Picks the right conda env per backend and
# auto-starts the ns3sionna ZMQ server if the user picks the ns3sionna backend.
#
# Usage:
#   ./bench.sh pure_ns3   [extra_args...]
#   ./bench.sh sionnart   [extra_args...]
#   ./bench.sh ns3sionna  [extra_args...]
#   ./bench.sh all        [extra_args...]   # all three, sequentially, same args
#
# Defaults applied if not present in extra_args:
#   --num_stas=1 --sim_seconds=2 --regime=smoke
#
# Examples:
#   ./bench.sh sionnart --num_stas=4 --sim_seconds=5 --mobile_scenario=true --mobile_speed=1.0
#   ./bench.sh all --num_stas=2

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVER_SH="${SCRIPT_DIR}/server.sh"

CONDA_SH=/home/aung/anaconda3/etc/profile.d/conda.sh

if [[ $# -lt 1 ]]; then
    echo "usage: $0 {pure_ns3|sionnart|ns3sionna|all} [extra ./ns3 args ...]"
    exit 2
fi

backend="$1"; shift

# Apply default args only when caller didn't already pass them
default_arg() {
    local needle="$1"; shift
    for arg in "$@"; do
        case "${arg}" in ${needle}=*) return 0 ;; esac
    done
    return 1
}
apply_defaults() {
    local out=("$@")
    default_arg "--num_stas"     "$@" || out+=("--num_stas=1")
    default_arg "--sim_seconds"  "$@" || out+=("--sim_seconds=2")
    default_arg "--regime"       "$@" || out+=("--regime=smoke")
    # Space-separate so the caller can fold into a single ./ns3 run "binary args" string.
    printf '%s' "${out[*]}"
}

run_one() {
    local b="$1"; shift
    local args; args="$(apply_defaults "$@")"
    local extra=""
    case "${b}" in
        pure_ns3)
            ;;
        sionnart)
            # No env for the script itself; libpython3.12 is baked in at link time.
            # But site-packages are picked from the active env, so activate 6G.
            # shellcheck disable=SC1090
            source "${CONDA_SH}" && conda activate 6G
            ;;
        ns3sionna)
            # Need the ZMQ server. Auto-start if not already up.
            if ! "${SERVER_SH}" status >/dev/null 2>&1; then
                echo "[bench.sh] ns3sionna server not running; starting in 6Gold..."
                "${SERVER_SH}" start
            else
                echo "[bench.sh] ns3sionna server already running"
            fi
            # shellcheck disable=SC1090
            source "${CONDA_SH}" && conda activate 6G
            ;;
        *)
            echo "unknown backend: ${b}" >&2
            exit 2
            ;;
    esac
    echo "================================================================"
    echo "[bench.sh] running ${b}_benchmark ${args}"
    echo "================================================================"
    cd "${NS3_DIR}"
    # Quote the whole arg string for ns3
    # shellcheck disable=SC2086
    ./ns3 run "${b}_benchmark ${args}"
}

case "${backend}" in
    pure_ns3|sionnart|ns3sionna)
        run_one "${backend}" "$@"
        ;;
    all)
        # Pre-start the server once so it's reused across runs
        if ! "${SERVER_SH}" status >/dev/null 2>&1; then
            echo "[bench.sh] starting ns3sionna server (used by 'all' run)"
            "${SERVER_SH}" start
        fi
        run_one pure_ns3   "$@"
        run_one sionnart   "$@"
        run_one ns3sionna  "$@"
        ;;
    *)
        echo "unknown backend: ${backend}" >&2
        echo "usage: $0 {pure_ns3|sionnart|ns3sionna|all} [extra args]" >&2
        exit 2
        ;;
esac
