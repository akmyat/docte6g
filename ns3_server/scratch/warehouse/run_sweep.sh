#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
NS3="$ROOT/ns3_server"
OUT="${1:-$ROOT/results/warehouse-clean}"
SIM_TIME="${SIM_TIME:-40}"
ISAC_SAMPLES_PER_SOURCE="${ISAC_SAMPLES_PER_SOURCE:-50000}"
CONDA_ENV="${CONDA_PREFIX:-/home/aung/anaconda3/envs/6G}"
EXPAT_LIB="$CONDA_ENV/lib/libexpat.so"

if [[ ! -d "$CONDA_ENV" ]]; then
  echo "Missing Python environment: $CONDA_ENV" >&2
  exit 2
fi

export PYTHONHOME="$CONDA_ENV"
export PYTHONPATH="$CONDA_ENV/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$CONDA_ENV/bin:$PATH"
if [[ -f "$EXPAT_LIB" ]]; then
  export LD_PRELOAD="$EXPAT_LIB${LD_PRELOAD:+:$LD_PRELOAD}"
fi

mkdir -p "$OUT"

for mode in no-isac isac; do
  executable="warehouse-benchmark-$mode"
  for size in 2 4 8; do
    run_dir="$OUT/$mode-${size}x${size}"
    mkdir -p "$run_dir"
    (
      cd "$NS3"
      ./ns3 run "scratch/warehouse/$executable \
        --simTime=$SIM_TIME \
        --gnbAntennaRows=$size \
        --gnbAntennaCols=$size \
        --isacSamplesPerSource=$ISAC_SAMPLES_PER_SOURCE \
        --outputDir=$run_dir"
    )
  done
done

python3 "$NS3/scratch/warehouse/analyze_sweep.py" "$OUT"
