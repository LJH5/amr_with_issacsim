#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo 'Ubuntu 24.04용 설치 스크립트입니다.' >&2
    exit 1
fi

# ROS 2 apt 저장소가 등록된 환경에서 실행합니다.
mapfile -t packages < "$project_dir/requirements-apt.txt"
sudo apt-get update
sudo apt-get install -y "${packages[@]}"

/usr/bin/python3 -m venv "$project_dir/.venv"
"$project_dir/.venv/bin/python" -I -m pip install -r "$project_dir/requirements-dev.txt"
mkdir -p "$project_dir/ros2_ws/src"
bash "$project_dir/scripts/freeze_environment.sh"
