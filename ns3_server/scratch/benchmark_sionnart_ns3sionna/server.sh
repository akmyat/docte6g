#!/usr/bin/env bash
# ns3sionna ZMQ server lifecycle (runs in conda env 6Gold).
#
# Usage:
#   ./server.sh start    # start in background, log to /tmp/ns3sionna_server.log
#   ./server.sh stop     # kill whoever owns tcp:5555
#   ./server.sh status   # show running pid + tail of log
#   ./server.sh restart
#   ./server.sh log      # tail -f the log
#
# The server is required ONLY by ns3sionna_benchmark. pure_ns3 and sionnart
# don't need it.

set -euo pipefail

PORT=5555
LOG=/tmp/ns3sionna_server.log
PIDFILE=/tmp/ns3sionna_server.pid
PY6Gold=/home/aung/anaconda3/envs/6Gold/bin/python
SERVER_DIR="$(cd "$(dirname "$0")/../../contrib/ns3sionna/model/ns3sionna" && pwd)"

pid_on_port() {
    # set -e + pipefail would abort us when nothing's listening; isolate the pipe.
    local out
    out="$(lsof -ti "tcp:${PORT}" 2>/dev/null || true)"
    [[ -n "${out}" ]] && echo "${out}" | head -1
    return 0
}

cmd_status() {
    local pid; pid="$(pid_on_port)"
    if [[ -n "${pid}" ]]; then
        echo "ns3sionna server: RUNNING (pid=${pid}, port=${PORT})"
        echo "log: ${LOG}"
        if [[ -f "${LOG}" ]]; then
            echo "--- last 5 log lines ---"
            tail -5 "${LOG}"
        fi
        return 0
    else
        echo "ns3sionna server: NOT RUNNING"
        return 1
    fi
}

cmd_start() {
    # Aggressively clean up any existing instances or zombies before starting
    local existing_pid; existing_pid="$(pid_on_port)"
    if [[ -n "${existing_pid}" ]]; then
        echo "Found existing process on port ${PORT} (pid=${existing_pid}). Stopping it first..."
        cmd_stop
    fi

    # Also kill any orphaned ns3sionna_server.py processes in 6Gold env
    pkill -9 -f "ns3sionna_server.py" || true

    if [[ ! -x "${PY6Gold}" ]]; then
        echo "ERROR: ${PY6Gold} not found. Is conda env 6Gold installed?" >&2
        return 2
    fi

    # Remove stale logs
    rm -f "${LOG}"

    echo "starting ns3sionna server (cwd=${SERVER_DIR}, python=6Gold)"
    cd "${SERVER_DIR}"
    # Blackwell optimization: reduce parallel links to 8 to prevent driver-level hangs during high-load sweeps.
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
        nohup "${PY6Gold}" ns3sionna_server.py --rt_max_parallel_links 8 --rt_fast > "${LOG}" 2>&1 &
    echo $! > "${PIDFILE}"

    # wait up to 30 s for the socket to bind
    for _ in $(seq 1 30); do
        sleep 1
        if [[ -n "$(pid_on_port)" ]]; then
            echo "started OK (pid=$(pid_on_port), port=${PORT})"
            return 0
        fi
        if grep -i "Traceback\|Error\|Address already in use" "${LOG}" 2>/dev/null; then
            echo "ERROR: server crashed on startup. Last 20 lines of ${LOG}:" >&2
            tail -20 "${LOG}" >&2
            return 3
        fi
    done
    echo "ERROR: server did not bind to :${PORT} within 30 s. Last 20 log lines:" >&2
    tail -20 "${LOG}" >&2
    return 4
}

cmd_stop() {
    local pid; pid="$(pid_on_port)"
    if [[ -z "${pid}" ]]; then
        echo "not running"
        return 0
    fi
    echo "stopping pid=${pid}"
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
        sleep 1
        [[ -z "$(pid_on_port)" ]] && { echo "stopped"; return 0; }
    done
    echo "still running, sending SIGKILL"
    kill -9 "${pid}" 2>/dev/null || true
}

cmd_restart() { cmd_stop; cmd_start; }
cmd_log()     { exec tail -f "${LOG}"; }

case "${1:-status}" in
    start)   cmd_start ;;
    stop)    cmd_stop ;;
    status)  cmd_status ;;
    restart) cmd_restart ;;
    log)     cmd_log ;;
    *) echo "usage: $0 {start|stop|status|restart|log}"; exit 2 ;;
esac
