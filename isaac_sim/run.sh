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
runtime_dir="$(cd -- "$(dirname -- "$(command -v "$runtime")")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-$runtime_dir}"
bridge_lib="$isaac_root/exts/isaacsim.ros2.bridge/jazzy/lib"
if [[ -d "$bridge_lib" ]]; then
    # System Jazzy uses Python 3.12; this SDK embeds Python 3.11 and its own ROS libraries.
    env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
        -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
        LD_LIBRARY_PATH="$bridge_lib" ROS_DISTRO=jazzy \
        RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
        "$runtime" run_amr.py "$@"
else
    env -u PYTHONPATH -u PYTHONHOME "$runtime" run_amr.py "$@"
fi
