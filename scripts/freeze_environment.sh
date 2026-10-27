#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 기본 freeze에서 제외되는 pip도 포함합니다. ROS/Isaac 패키지는 넣지 않습니다.
freeze="$("$project_dir/.venv/bin/python" -I -m pip --no-cache-dir freeze --all)"
printf '%s\n' "$freeze" > "$project_dir/requirements-dev.txt"

{
    echo '# 직접 지정한 apt 패키지의 설치 현황입니다. 전체 전이 의존성 lock이 아닙니다.'
    echo '# name=version: 설치됨 / MISSING: 미설치 (apt 설치 성공 후 다시 생성하세요).'
    while IFS= read -r package; do
        if state="$(dpkg-query -W -f='${db:Status-Abbrev}|${Version}' "$package" 2>/dev/null)" && [[ "$state" == ii* ]]; then
            printf '%s=%s\n' "$package" "${state#*|}"
        else
            printf '# MISSING: %s\n' "$package"
        fi
    done < "$project_dir/requirements-apt.txt"
} > "$project_dir/system-packages.freeze.txt"
