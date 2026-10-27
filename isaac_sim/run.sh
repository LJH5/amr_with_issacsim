#!/usr/bin/env bash
set -euo pipefail

sim_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
    /usr/bin/python3 "$sim_dir/run_amr.py" --help
    exit 0
fi

# Standalone: ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh
# pip installation: ISAAC_SIM_PYTHON=/path/to/its/python
runtime="${ISAAC_SIM_PYTHON:-$sim_dir/../../IsaacSim/python.sh}"
if ! command -v "$runtime" >/dev/null; then
    echo 'ISAAC_SIM_PYTHON을 Isaac Sim 전용 Python 또는 python.sh 경로로 지정하세요.' >&2
    exit 1
fi
cd "$sim_dir"
env -u PYTHONPATH -u PYTHONHOME "$runtime" run_amr.py "$@"
